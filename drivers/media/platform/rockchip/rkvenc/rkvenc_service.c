// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip VEPU580 encoder driver - MPP service char device and ioctl dispatch
 * Ported from Rockchip BSP mpp_service.c / mpp_common.c
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/device/class.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/poll.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "rkvenc_hw.h"

/* ---- MSG v1 structure (from userspace) ---- */
struct mpp_msg_v1 {
	__u32 cmd;
	__u32 flags;
	__u32 size;
	__u32 offset;
	__u64 data_ptr;
};

/* ---- Session helpers ---- */
static struct rkvenc_session *rkvenc_session_init(void)
{
	struct rkvenc_session *session;

	session = kzalloc(sizeof(*session), GFP_KERNEL);
	if (!session)
		return NULL;

	/* Closed files may leave asynchronous tasks holding this session. */
	__module_get(THIS_MODULE);
	kref_init(&session->ref);
	mutex_init(&session->ioctl_lock);
	session->pid = current->pid;
	mutex_init(&session->pending_lock);
	INIT_LIST_HEAD(&session->pending_list);
	INIT_LIST_HEAD(&session->service_link);
	INIT_LIST_HEAD(&session->session_link);
	atomic_set(&session->task_count, 0);
	atomic_set(&session->release_request, 0);

	return session;
}

static void rkvenc_session_destroy(struct kref *ref)
{
	struct rkvenc_session *session = container_of(ref, struct rkvenc_session, ref);

	{
		int i;

		for (i = 0; i < RKVENC_MAX_CORE_NUM; i++)
			if (session->core_dma[i])
				rkvenc_dma_session_destroy(session->core_dma[i]);
	}

	if (session->priv) {
		kfree(session->priv);
		session->priv = NULL;
	}

	kfree(session);
	module_put(THIS_MODULE);
}

void rkvenc_session_put(struct rkvenc_session *session)
{
	kref_put(&session->ref, rkvenc_session_destroy);
}

/* ---- Attach session to encoder device ---- */
static int rkvenc_session_attach_device(struct rkvenc_session *session,
					struct rkvenc_dev *mpp)
{
	session->dma = rkvenc_dma_session_create(mpp,
						 mpp->session_max_buffers);
	if (!session->dma) {
		rkvenc_err("failed to create dma session\n");
		return -ENOMEM;
	}

	session->mpp = mpp;
	session->core_dma[mpp->core_id] = session->dma;
	return 0;
}

/* ---- Class msg alloc/free ---- */
static int rkvenc_alloc_class_msg(struct rkvenc_task *task, u32 class)
{
	const struct rkvenc_hw_info *hw = task->hw_info;
	u32 base_s = hw->reg_msg[class].base_s;
	u32 base_e = hw->reg_msg[class].base_e;
	/*
	 * BSP treats base_e as the last valid dword (inclusive), unlike the usual
	 * half-open [base_s, base_e) convention. Keep this to match BSP IOCTL layout.
	 */
	u32 size = base_e - base_s + sizeof(u32);

	if (task->reg[class].data)
		return 0;

	task->reg[class].data = kzalloc(size, GFP_KERNEL);
	if (!task->reg[class].data)
		return -ENOMEM;

	task->reg[class].size = size;
	return 0;
}

static void rkvenc_free_class_msg(struct rkvenc_task *task)
{
	int i;

	for (i = 0; i < RKVENC_CLASS_BUTT; i++) {
		kfree(task->reg[i].data);
		task->reg[i].data = NULL;
		task->reg[i].size = 0;
		task->reg[i].valid = 0;
	}
}

/* Validate before inclusive endpoint arithmetic or copying register data. */
static bool rkvenc_valid_reg_request(const struct mpp_request *req)
{
	return req->size && IS_ALIGNED(req->offset, sizeof(u32)) &&
		IS_ALIGNED(req->size, sizeof(u32)) &&
		req->offset <= U32_MAX - (req->size - sizeof(u32));
}

/* ---- Check if request overlaps a class ---- */
static bool req_over_class(struct mpp_request *req,
			   struct rkvenc_task *task, u32 class)
{
	const struct rkvenc_hw_info *hw = task->hw_info;
	/* BSP overlap check is inclusive; use last dword of request (offset+size-4). */
	u32 req_e = req->offset + (req->size - sizeof(u32));
	u32 base_s = hw->reg_msg[class].base_s;
	u32 base_e = hw->reg_msg[class].base_e;

	return (req->offset <= base_e && req_e >= base_s);
}

/* ---- Update request to fit within class boundaries ---- */
static void rkvenc_update_req(struct rkvenc_task *task, u32 class,
			      struct mpp_request *src, struct mpp_request *dst)
{
	const struct rkvenc_hw_info *hw = task->hw_info;
	u32 base_s = hw->reg_msg[class].base_s;
	u32 base_e = hw->reg_msg[class].base_e;
	u32 req_s = src->offset;
	/* Clamp to BSP's inclusive end to avoid truncating the last register. */
	u32 req_e = src->offset + src->size - sizeof(u32);
	u32 s = max(req_s, base_s);
	u32 e = min(req_e, base_e);

	dst->cmd = src->cmd;
	dst->flags = src->flags;
	dst->offset = s;
	dst->size = e - s + sizeof(u32);
	/* Adjust data pointer for the offset */
	dst->data = (void __user *)((u8 __user *)src->data + (s - req_s));
}

/* ---- Extract RCB info ---- */
static int rkvenc2_extract_rcb_info(struct rkvenc2_rcb_info *rcb_inf,
				    struct mpp_request *req)
{
	int max_size = ARRAY_SIZE(rcb_inf->elem);
	int cnt = req->size / sizeof(rcb_inf->elem[0]);

	if (req->size % sizeof(rcb_inf->elem[0]) ||
	    req->size > sizeof(rcb_inf->elem)) {
		rkvenc_err("count %d, max_size %d\n", cnt, max_size);
		return -EINVAL;
	}
	if (copy_from_user(rcb_inf->elem, req->data, req->size)) {
		rkvenc_err("copy_from_user failed\n");
		return -EINVAL;
	}
	rcb_inf->cnt = cnt;

	return 0;
}

/* ---- Extract task messages from userspace ---- */
static int rkvenc_extract_task_msg(struct rkvenc_session *session,
				   struct rkvenc_task *task,
				   struct rkvenc_task_msgs *msgs)
{
	int ret;
	u32 i, j;
	struct mpp_request *req;
	const struct rkvenc_hw_info *hw = task->hw_info;

	for (i = 0; i < msgs->req_cnt; i++) {
		req = &msgs->reqs[i];
		if (!req->size)
			continue;

		if ((req->cmd == MPP_CMD_SET_REG_WRITE ||
		     req->cmd == MPP_CMD_SET_REG_READ) &&
		    !rkvenc_valid_reg_request(req)) {
			ret = -EINVAL;
			goto fail;
		}

		switch (req->cmd) {
		case MPP_CMD_SET_REG_WRITE: {
			void *data;
			struct mpp_request *wreq;

			for (j = 0; j < hw->reg_class; j++) {
				if (!req_over_class(req, task, j))
					continue;

				ret = rkvenc_alloc_class_msg(task, j);
				if (ret) {
					rkvenc_err("alloc class msg %d fail.\n", j);
					goto fail;
				}
				if (task->w_req_cnt >= ARRAY_SIZE(task->w_reqs)) {
					ret = -E2BIG;
					goto fail;
				}
				wreq = &task->w_reqs[task->w_req_cnt];
				rkvenc_update_req(task, j, req, wreq);
				data = (u8 *)task->reg[j].data + (wreq->offset - hw->reg_msg[j].base_s);
				if (!data) {
					rkvenc_err("get class reg fail, offset %08x\n", wreq->offset);
					ret = -EINVAL;
					goto fail;
				}
				if (copy_from_user(data, wreq->data, wreq->size)) {
					rkvenc_err("copy_from_user fail, offset %08x\n", wreq->offset);
					ret = -EIO;
					goto fail;
				}
				task->reg[j].valid = 1;
				task->w_req_cnt++;
			}
		} break;
		case MPP_CMD_SET_REG_READ: {
			struct mpp_request *rreq;

			for (j = 0; j < hw->reg_class; j++) {
				if (!req_over_class(req, task, j))
					continue;

				ret = rkvenc_alloc_class_msg(task, j);
				if (ret) {
					rkvenc_err("alloc class msg reg %d fail.\n", j);
					goto fail;
				}
				if (task->r_req_cnt >= ARRAY_SIZE(task->r_reqs)) {
					ret = -E2BIG;
					goto fail;
				}
				rreq = &task->r_reqs[task->r_req_cnt];
				rkvenc_update_req(task, j, req, rreq);
				task->reg[j].valid = 1;
				task->r_req_cnt++;
			}
		} break;
		case MPP_CMD_SET_REG_ADDR_OFFSET: {
			ret = rkvenc_extract_reg_offset_info(&task->off_inf, req);
			if (ret)
				goto fail;
		} break;
		case MPP_CMD_SET_RCB_INFO: {
			struct rkvenc2_session_priv *priv = session->priv;

			if (priv) {
				ret = rkvenc2_extract_rcb_info(&priv->rcb_inf, req);
				if (ret)
					goto fail;
			}
		} break;
		default:
			break;
		}
	}

	return 0;

fail:
	rkvenc_free_class_msg(task);
	return ret;
}

/* ---- Get task format from register data ---- */
static int rkvenc_task_get_format(struct rkvenc_dev *mpp,
				  struct rkvenc_task *task)
{
	u32 offset, val;
	const struct rkvenc_hw_info *hw = task->hw_info;
	u32 class = hw->fmt_reg.class;
	u32 *class_reg = task->reg[class].data;
	u32 class_size = task->reg[class].size;
	u32 class_base = hw->reg_msg[class].base_s;
	u32 bitpos = hw->fmt_reg.bitpos;
	u32 bitlen = hw->fmt_reg.bitlen;

	if (!class_reg || !class_size)
		return -EINVAL;

	offset = hw->fmt_reg.base - class_base;
	val = class_reg[offset / sizeof(u32)];
	task->fmt = (val >> bitpos) & ((1 << bitlen) - 1);

	return 0;
}

/* ---- Setup task DCHS ID ---- */
static void rkvenc2_setup_task_id(u32 session_id, struct rkvenc_task *task)
{
	const struct rkvenc_hw_info *hw = task->hw_info;
	u32 val;

	/* always enable tx */
	val = task->reg[RKVENC_CLASS_PIC].data[hw->dcsh_class_ofst] | DCHS_TXE;
	if (hw->dcsh_class_ofst)
		task->reg[RKVENC_CLASS_PIC].data[hw->dcsh_class_ofst] = val;
	task->dchs_id.val[0] = (((u64)session_id << 32) | val);

	task->dchs_id.txid_orig = task->dchs_id.txid;
	task->dchs_id.rxid_orig = task->dchs_id.rxid;
	task->dchs_id.txid_map = task->dchs_id.txid;
	task->dchs_id.rxid_map = task->dchs_id.rxid;

	task->dchs_id.txe_orig = task->dchs_id.txe;
	task->dchs_id.rxe_orig = task->dchs_id.rxe;
	task->dchs_id.txe_map = task->dchs_id.txe;
	task->dchs_id.rxe_map = task->dchs_id.rxe;
}

/* ---- Check for slice split task ---- */
static void rkvenc2_check_split_task(struct rkvenc_dev *mpp, struct rkvenc_task *task)
{
	u32 slen_fifo_en = 0;
	u32 sli_split_en = 0;

	if (task->reg[RKVENC_CLASS_PIC].valid) {
		u32 *reg = task->reg[RKVENC_CLASS_PIC].data;

		slen_fifo_en = (reg[RKVENC2_REG_ENC_PIC] & RKVENC2_BIT_SLEN_FIFO) ? 1 : 0;
		sli_split_en = (reg[RKVENC2_REG_SLI_SPLIT] & RKVENC2_BIT_SLI_SPLIT) ? 1 : 0;

		/* H.264 bug: external line buffer + slice flush = bad */
		if (sli_split_en && slen_fifo_en &&
		    (reg[RKVENC2_REG_ENC_PIC] & RKVENC2_BIT_ENC_STND) == RKVENC2_BIT_VAL_H264 &&
		    reg[RKVENC2_REG_EXT_LINE_BUF_BASE])
			reg[RKVENC2_REG_SLI_SPLIT] &= ~RKVENC2_BIT_SLI_FLUSH;
	}

	task->task_split = sli_split_en && slen_fifo_en;

	if (task->task_split)
		INIT_KFIFO(task->slice_info);
}

/* ---- Set per-core DRAM RCB addresses from this task's snapshot ---- */
static int rkvenc2_set_rcbbuf(struct rkvenc_dev *enc, struct rkvenc_task *task)
{
	const struct rkvenc_hw_info *hw = task->hw_info;
	u32 offset = 0;
	int i;

	for (i = 0; i < task->rcb_inf.cnt; i++) {
		u32 index = task->rcb_inf.elem[i].index;
		u32 size = task->rcb_inf.elem[i].size;
		u32 addr;
		int c;

		if (index > U32_MAX / sizeof(u32))
			return -EINVAL;
		addr = index * sizeof(u32);
		/* Retain the userspace DRAM mapping if scratch space is exhausted. */
		if (size > enc->sram_used - offset)
			continue;
		for (c = 0; c < hw->reg_class; c++) {
			u32 base = hw->reg_msg[c].base_s;

			if (addr < base || addr > hw->reg_msg[c].base_e)
				continue;
			if (!task->reg[c].data)
				return -EINVAL;
			task->reg[c].data[(addr - base) / sizeof(u32)] =
				enc->sram_iova + offset;
			break;
		}
		if (c == hw->reg_class)
			return -EINVAL;
		offset += size;
	}
	return 0;
}

/* ---- Alloc task from session ---- */
static struct rkvenc_task *rkvenc_alloc_task(struct rkvenc_session *session,
					     struct rkvenc_task_msgs *msgs)
{
	int ret;
	struct rkvenc_task *task;
	struct rkvenc_mpp_task *mpp_task;
	struct rkvenc_dev *mpp = session->mpp;

	task = kzalloc(sizeof(*task), GFP_KERNEL);
	if (!task)
		return NULL;

	mpp_task = &task->mpp_task;
	rkvenc_task_init(session, mpp_task);
	mpp_task->hw_info = &mpp->hw_info->hw;
	task->hw_info = mpp->hw_info;
	task->core_mask = BIT(mpp->core_id);
	if (!(msgs->flags & MPP_FLAGS_REG_FD_NO_TRANS)) {
		int i;

		for (i = 0; i < RKVENC_MAX_CORE_NUM; i++)
			if (mpp->queue->cores[i])
				__set_bit(i, &task->core_mask);
	}

	/* extract reqs for current task */
	ret = rkvenc_extract_task_msg(session, task, msgs);
	if (ret)
		goto free_task;
	mpp_task->reg = task->reg[0].data;
	if (session->priv)
		task->rcb_inf = ((struct rkvenc2_session_priv *)session->priv)->rcb_inf;

	/* get format */
	ret = rkvenc_task_get_format(mpp, task);
	if (ret)
		goto free_task;

	/* process fd in register */
	if (!(msgs->flags & MPP_FLAGS_REG_FD_NO_TRANS)) {
		u32 i, j;
		int cnt;
		u32 off;
		const u16 *tbl;
		const struct rkvenc_hw_info *hw = task->hw_info;
		int fd_bs = -1;

		for (i = 0; i < hw->fd_class; i++) {
			u32 class = hw->fd_reg[i].class;
			u32 fmt = hw->fd_reg[i].base_fmt + task->fmt;
			u32 *reg = task->reg[class].data;
			u32 ss = hw->reg_msg[class].base_s / sizeof(u32);
			u32 mem_count_before;

			if (!reg)
				continue;

			if (fmt == RKVENC_FMT_JPEGE && class == RKVENC_CLASS_PIC && fd_bs == -1) {
				int bs_index = mpp->trans_info[fmt].table[2];

				fd_bs = reg[bs_index];
				task->offset_bs = rkvenc_query_reg_offset_info(&task->off_inf,
									       bs_index + ss);
			}

			mem_count_before = mpp_task->mem_count;
			ret = rkvenc_translate_reg_address(session, mpp_task, fmt, class, reg, NULL);
			if (ret)
				goto fail;

			cnt = session->trans_count ? session->trans_count :
				mpp->trans_info[fmt].count;
			tbl = session->trans_count ? session->trans_table :
				mpp->trans_info[fmt].table;
			for (j = 0; j < cnt; j++) {
				off = rkvenc_query_reg_offset_info(&task->off_inf, tbl[j] + ss);
				if (off > U32_MAX - reg[tbl[j]]) {
					ret = -EINVAL;
					goto fail;
				}
				reg[tbl[j]] += off;
			}

			/* Guardrail: ensure translated regs fall inside their mapped buffers */
			{
				u32 mc;

				for (mc = mem_count_before; mc < mpp_task->mem_count; mc++) {
					struct rkvenc_mem_region *mr = &mpp_task->mem_regions[mc];
					u32 reg_size;
					u32 reg_val;
					unsigned long reg_off;
					bool allow_end;
					dma_addr_t end;
					dma_addr_t reg_iova;

					if (mr->reg_class != class)
						continue;
					if (!mr->len)
						continue;

					reg_size = task->reg[class].size / sizeof(u32);
					if (mr->reg_idx >= reg_size) {
						dev_err(mpp->dev,
							"guardrail: class %u reg_idx %u out of range (%u dwords) fd %d\n",
							class, mr->reg_idx, reg_size, mr->fd);
						ret = -EINVAL;
						goto fail;
					}

					reg_val = reg[mr->reg_idx];
					reg_iova = (dma_addr_t)reg_val;
					end = mr->iova + mr->len;
					reg_off = rkvenc_query_reg_offset_info(&task->off_inf,
									   mr->reg_idx + ss);
					/* BSP encodes BSBT as base+size (end pointer); allow end-of-buffer when offset==len. */
					allow_end = (reg_iova == end) && (reg_off == mr->len);
					if (reg_iova < mr->iova || (!allow_end && reg_iova >= end)) {
						dev_err(mpp->dev,
							"guardrail: class %u reg[%u]=%#08x outside iova [%pad..%pad) fd %d\n",
							class, mr->reg_idx, reg_val, &mr->iova, &end, mr->fd);
						ret = -EINVAL;
						goto fail;
					}
				}
			}
		}

		if (fd_bs >= 0) {
			struct rkvenc_dma_buffer *bs_buf =
				rkvenc_dma_find_buffer_fd(session->dma, fd_bs);

			if (bs_buf && task->offset_bs > 0)
				rkvenc_dma_buf_sync(bs_buf, 0, task->offset_bs,
						   DMA_TO_DEVICE, false);
			task->bs_buf = bs_buf;
		}
	}

	rkvenc2_setup_task_id(session->index, task);
	task->clk_mode = CLK_MODE_NORMAL;
	rkvenc2_check_split_task(mpp, task);

	/* Init wait queue and reference count */
	init_waitqueue_head(&mpp_task->wait);
	kref_init(&mpp_task->ref);
	atomic_set(&mpp_task->abort_request, 0);
	mpp_task->task_index = atomic_inc_return(&mpp->task_index);
	mpp_task->task_id = atomic_inc_return(&mpp->queue->task_id);
	kref_get(&session->ref);
	atomic_inc(&session->task_count);
	atomic_inc(&mpp->task_count);

	return task;

fail:
	rkvenc_task_finalize(session, mpp_task);
free_task:
	rkvenc_free_class_msg(task);
	kfree(task);
	return NULL;
}

/* ---- Result: copy status regs back to userspace ---- */
static int rkvenc_result(struct rkvenc_dev *mpp,
			 struct rkvenc_mpp_task *mpp_task)
{
	u32 i;
	struct rkvenc_task *task = container_of(mpp_task, struct rkvenc_task, mpp_task);

	for (i = 0; i < task->r_req_cnt; i++) {
		struct mpp_request *req = &task->r_reqs[i];
		const struct rkvenc_hw_info *hw = task->hw_info;
		u32 class_base;
		u32 *reg;
		int c;

		/* Find class for this read request offset */
		for (c = 0; c < hw->reg_class; c++) {
			if (req->offset >= hw->reg_msg[c].base_s &&
			    req->offset <= hw->reg_msg[c].base_e) {
				class_base = hw->reg_msg[c].base_s;
				reg = (u32 *)((u8 *)task->reg[c].data + (req->offset - class_base));
				break;
			}
		}

		if (c == hw->reg_class) {
			rkvenc_err("read request offset %x not in any class\n", req->offset);
			return -EINVAL;
		}

		if (copy_to_user(req->data, reg, req->size)) {
			rkvenc_err("copy_to_user reg fail\n");
			return -EIO;
		}
	}

	return 0;
}

/* Replace main-core IOVAs with the selected core's DMA mappings. */
static int rkvenc_place_task(struct rkvenc_task *task, int core_id)
{
	struct rkvenc_mem_region *mr;

	list_for_each_entry(mr, &task->mpp_task.mem_region_list, reg_link) {
		u32 *reg = task->reg[mr->reg_class].data;
		struct rkvenc_dma_buffer *buffer = mr->core_hdl[core_id];

		if (!buffer)
			return -ENODEV;
		reg[mr->reg_idx] = reg[mr->reg_idx] - mr->iova + buffer->iova;
		if (task->bs_buf == mr->hdl)
			task->bs_buf = buffer;
	}
	return 0;
}

/* ---- Task ISR bottom half (called from kthread worker) ---- */
void rkvenc_task_worker_default(struct kthread_work *work)
{
	struct rkvenc_dev *enc = container_of(work, struct rkvenc_dev, work);
	struct rkvenc_taskqueue *queue = enc->queue;
	struct rkvenc_mpp_task *mpp_task;
	struct rkvenc_task *task;
	struct rkvenc_session *session;

	/* Process finished tasks (ISR bottom half) */
	mpp_task = enc->cur_task;
	if (mpp_task && test_bit(TASK_STATE_IRQ, &mpp_task->state)) {
		task = container_of(mpp_task, struct rkvenc_task, mpp_task);
		session = mpp_task->session;

		task->irq_status = enc->irq_status;

		/* Update DCHS state */
		if (enc->ccu) {
			unsigned long flags;

			spin_lock_irqsave(&enc->ccu->lock_dchs, flags);
			enc->ccu->dchs[enc->core_id].val[0] = 0;
			enc->ccu->dchs[enc->core_id].val[1] = 0;
			spin_unlock_irqrestore(&enc->ccu->lock_dchs, flags);
		}

		/* Check for errors */
		if (task->irq_status & enc->hw_info->err_mask)
			atomic_inc(&enc->reset_request);

		rkvenc_task_finish(session, mpp_task);
		mpp_task = NULL;
	}

	/* Process timed-out tasks */
	if (mpp_task && test_bit(TASK_STATE_TIMEOUT, &mpp_task->state)) {
		session = mpp_task->session;

		rkvenc_err("task %d timeout, reset\n", mpp_task->task_index);
		atomic_inc(&enc->reset_request);

		mpp_task->result = -ETIMEDOUT;
		rkvenc_task_finish(session, mpp_task);
	}

	/* Fill all idle cores; submission work can be coalesced. */
	for (;;) {
		struct rkvenc_mpp_task *pending_task = NULL;
		unsigned long flags;
		unsigned long core_idle;
		unsigned long usable = 0;
		s32 core_id;
		int i;

		mutex_lock(&queue->pending_lock);
		spin_lock_irqsave(&queue->running_lock, flags);
		for (i = 0; i < RKVENC_MAX_CORE_NUM; i++)
			if (queue->cores[i] && !queue->cores[i]->disable)
				__set_bit(i, &usable);
		pending_task = list_first_entry_or_null(&queue->pending_list,
				struct rkvenc_mpp_task, queue_link);
		if (pending_task) {
			struct rkvenc_task *head = container_of(pending_task,
						struct rkvenc_task, mpp_task);

			usable &= head->core_mask;
			core_idle = queue->core_idle & usable;
			core_id = find_first_bit(&core_idle, RKVENC_MAX_CORE_NUM);
			if (core_id < RKVENC_MAX_CORE_NUM) {
				list_del_init(&pending_task->queue_link);
				clear_bit(core_id, &queue->core_idle);
				pending_task->mpp = queue->cores[core_id];
				pending_task->core_id = core_id;
			} else if (!usable) {
				list_del_init(&pending_task->queue_link);
			} else {
				pending_task = NULL;
			}
		}
		spin_unlock_irqrestore(&queue->running_lock, flags);
		mutex_unlock(&queue->pending_lock);

		if (pending_task && !usable) {
			rkvenc_task_complete(pending_task, -ENODEV);
			continue;
		}
		if (!pending_task)
			break;

		if (pending_task) {
			struct rkvenc_dev *target = pending_task->mpp;
			struct rkvenc_task *enc_task = container_of(pending_task,
								   struct rkvenc_task,
								   mpp_task);
			int run_ret;

			run_ret = rkvenc_place_task(enc_task, target->core_id);
			if (!run_ret)
				run_ret = rkvenc2_set_rcbbuf(target, enc_task);
			if (!run_ret) {
				set_bit(TASK_STATE_RUNNING, &pending_task->state);
				run_ret = rkvenc_hw_run(target, pending_task);
			}
			if (run_ret) {
				dev_err(target->dev,
					"hw_run failed: %d, failing task %d\n",
					run_ret, pending_task->task_index);
				set_bit(target->core_id, &queue->core_idle);
				rkvenc_task_complete(pending_task, run_ret);
			}
		}
	}
}

/* ---- Process ioctl requests ---- */
static int rkvenc_check_cmd(unsigned int cmd)
{
	if (cmd >= MPP_CMD_BUTT)
		return -EINVAL;
	return 0;
}

static int rkvenc_process_request(struct rkvenc_session *session,
				  struct mpp_request *req,
				  struct rkvenc_task_msgs *msgs)
{
	rkvenc_dbg(DEBUG_IOCTL, "cmd %x, size %d, offset %x\n",
		   req->cmd, req->size, req->offset);

	switch (req->cmd) {
	case MPP_CMD_QUERY_HW_SUPPORT: {
		u32 val = session->srv->hw_support;

		if (put_user(val, (u32 __user *)req->data))
			return -EFAULT;
	} break;
	case MPP_CMD_QUERY_HW_ID: {
		struct rkvenc_dev *mpp = session->mpp;
		u32 val = mpp ? mpp->hw_info->hw.hw_id : 0;

		if (put_user(val, (u32 __user *)req->data))
			return -EFAULT;
	} break;
	case MPP_CMD_INIT_CLIENT_TYPE: {
		u32 client_type;

		if (get_user(client_type, (u32 __user *)req->data))
			return -EFAULT;

		if (client_type != MPP_DEVICE_RKVENC)
			return -ENODEV;
		session->device_type = client_type;

		/* Attach to the encoder device */
		if (!session->mpp) {
			struct rkvenc_dev *dev =
				session->srv->sub_devices[MPP_DEVICE_RKVENC];

			int ret;

			if (!dev)
				return -ENODEV;
			ret = rkvenc_session_attach_device(session, dev);
			if (ret)
				return ret;
		}

		/* Init session private data */
		if (!session->priv) {
			struct rkvenc2_session_priv *priv;

			priv = kzalloc(sizeof(*priv), GFP_KERNEL);
			if (!priv)
				return -ENOMEM;
			init_rwsem(&priv->rw_sem);
			session->priv = priv;
		}
	} break;
	case MPP_CMD_INIT_TRANS_TABLE: {
		int cnt = req->size / sizeof(u16);

		if (req->size % sizeof(u16) ||
		    req->size > sizeof(session->trans_table)) {
			rkvenc_err("trans count %d too large\n", cnt);
			return -EINVAL;
		}
		if (copy_from_user(session->trans_table, req->data, req->size)) {
			rkvenc_err("copy_from_user trans_table failed\n");
			return -EFAULT;
		}
		session->trans_count = cnt;
	} break;
	case MPP_CMD_SET_REG_WRITE:
	case MPP_CMD_SET_REG_READ:
	case MPP_CMD_SET_REG_ADDR_OFFSET:
	case MPP_CMD_SET_RCB_INFO: {
		msgs->set_cnt++;
	} break;
	case MPP_CMD_POLL_HW_FINISH: {
		msgs->poll_cnt++;
		msgs->poll_req = req;
	} break;
	case MPP_CMD_TRANS_FD_TO_IOVA: {
		u32 fds[MPP_MAX_REG_TRANS_NUM];
		u32 i;

		if (!session->dma || !req->size ||
		    req->size % sizeof(u32) || req->size > sizeof(fds))
			return -EINVAL;
		if (copy_from_user(fds, req->data, req->size))
			return -EFAULT;
		for (i = 0; i < req->size / sizeof(u32); i++) {
			struct rkvenc_dma_buffer *buf;

			rkvenc_iommu_down_read(session->mpp->iommu_info);
			buf = rkvenc_dma_import_fd(session->mpp->iommu_info,
						   session->dma, fds[i], 1);
			rkvenc_iommu_up_read(session->mpp->iommu_info);
			if (IS_ERR(buf))
				return PTR_ERR(buf);
			fds[i] = buf->iova;
		}
		if (copy_to_user(req->data, fds, req->size))
			return -EFAULT;
	} break;
	case MPP_CMD_RELEASE_FD: {
		u32 fds[MPP_MAX_REG_TRANS_NUM];
		u32 i;
		int ret;

		if (!session->dma || !req->size ||
		    req->size % sizeof(u32) || req->size > sizeof(fds))
			return -EINVAL;
		if (copy_from_user(fds, req->data, req->size))
			return -EFAULT;
		rkvenc_iommu_down_read(session->mpp->iommu_info);
		for (i = 0; i < req->size / sizeof(u32); i++) {
			ret = rkvenc_dma_release_fd(session->dma, fds[i]);
			if (ret) {
				rkvenc_iommu_up_read(session->mpp->iommu_info);
				return ret;
			}
		}
		rkvenc_iommu_up_read(session->mpp->iommu_info);
	} break;
	case MPP_CMD_SEND_CODEC_INFO: {
		/* Codec info is optional, just store it */
		if (session->priv) {
			int ci;
			int cnt;
			struct codec_info_elem elem;
			struct rkvenc2_session_priv *priv = session->priv;

			cnt = req->size / sizeof(elem);
			cnt = (cnt > ENC_INFO_BUTT) ? ENC_INFO_BUTT : cnt;
			for (ci = 0; ci < cnt; ci++) {
				if (copy_from_user(&elem,
						   (void __user *)req->data + ci * sizeof(elem),
						   sizeof(elem)))
					continue;
				if (elem.type > ENC_INFO_BASE && elem.type < ENC_INFO_BUTT) {
					priv->codec_info[elem.type].flag = elem.flag;
					priv->codec_info[elem.type].val = elem.data;
				}
			}
		}
	} break;
	default:
		break;
	}

	return 0;
}

/* ---- Collect messages from one ioctl call ---- */
static int rkvenc_collect_msgs(struct rkvenc_session *session,
			       unsigned int cmd, void __user *msg,
			       struct rkvenc_task_msgs *msgs)
{
	struct mpp_msg_v1 msg_v1;
	struct mpp_request *req;
	int last = 1;
	int ret;

	if (cmd != MPP_IOC_CFG_V1) {
		rkvenc_err("unknown ioctl cmd %x\n", cmd);
		return -EINVAL;
	}

next:
	if (copy_from_user(&msg_v1, msg, sizeof(msg_v1)))
		return -EFAULT;

	msg += sizeof(msg_v1);

	if (rkvenc_check_cmd(msg_v1.cmd)) {
		rkvenc_err("cmd %x not supported\n", msg_v1.cmd);
		return -EFAULT;
	}

	if (msg_v1.flags & MPP_FLAGS_MULTI_MSG)
		last = (msg_v1.flags & MPP_FLAGS_LAST_MSG) ? 1 : 0;
	else
		last = 1;

	if (msgs->req_cnt >= MPP_MAX_MSG_NUM) {
		rkvenc_err("message count %d more than %d\n",
			   msgs->req_cnt, MPP_MAX_MSG_NUM);
		return -EINVAL;
	}

	req = &msgs->reqs[msgs->req_cnt++];
	req->cmd = msg_v1.cmd;
	req->flags = msg_v1.flags;
	req->size = msg_v1.size;
	req->offset = msg_v1.offset;
	req->data = (void __user *)(unsigned long)msg_v1.data_ptr;

	/* Update session flags */
	session->msg_flags = msg_v1.flags;
	msgs->flags = msg_v1.flags;

	ret = rkvenc_process_request(session, req, msgs);
	if (ret) {
		rkvenc_err("process cmd %x ret %d\n", req->cmd, ret);
		return ret;
	}

	if (!last)
		goto next;

	return 0;
}

/* ---- Wait for task result ---- */
static int rkvenc_wait_result(struct rkvenc_session *session,
			      struct rkvenc_task_msgs *msgs)
{
	struct rkvenc_mpp_task *task;
	struct rkvenc_task *enc_task;
	union rkvenc2_slice_len_info slice_info;
	int ret = 0;

	mutex_lock(&session->pending_lock);
	task = list_first_entry_or_null(&session->pending_list,
					struct rkvenc_mpp_task,
					pending_link);
	mutex_unlock(&session->pending_lock);

	if (!task) {
		rkvenc_err("session %p pending list is empty!\n", session);
		return -EIO;
	}

	enc_task = container_of(task, struct rkvenc_task, mpp_task);

	if (!enc_task->task_split || enc_task->task_split_done) {
task_done_ret:
		ret = wait_event_interruptible(task->wait,
					       test_bit(TASK_STATE_DONE, &task->state));
		if (ret)
			return ret;

		/* Keep the pending reference on interrupted waits. */
		ret = task->result;
		if (!ret)
			ret = rkvenc_result(task->mpp, task);

		/* Pop from pending list */
		mutex_lock(&session->pending_lock);
		list_del_init(&task->pending_link);
		mutex_unlock(&session->pending_lock);

		kref_put(&task->ref, rkvenc_free_task_callback);

		return ret;
	}

	/* Slice split mode: wait for all slices */
	do {
		ret = wait_event_interruptible(task->wait,
				!kfifo_is_empty(&enc_task->slice_info) ||
				test_bit(TASK_STATE_DONE, &task->state));
		if (ret)
			return ret;
		if (!kfifo_out(&enc_task->slice_info, &slice_info, 1))
			goto task_done_ret;

		enc_task->slice_rd_cnt++;

		if (slice_info.last)
			goto task_done_ret;
	} while (1);
}

/* ---- Main ioctl handler ---- */
static long rkvenc_dev_ioctl_locked(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rkvenc_session *session = filp->private_data;
	struct rkvenc_task_msgs msgs;
	struct rkvenc_task *task;
	int ret = 0;

	if (!session || !session->srv) {
		rkvenc_err("session %p\n", session);
		return -EINVAL;
	}

	if (atomic_read(&session->release_request) > 0)
		return -EBUSY;
	if (atomic_read(&session->srv->shutdown_request) > 0)
		return -EBUSY;

	/* Init msgs */
	memset(&msgs, 0, sizeof(msgs));
	INIT_LIST_HEAD(&msgs.list);
	msgs.session = session;

	/* Phase 1: Collect all messages from the ioctl */
	ret = rkvenc_collect_msgs(session, cmd, (void __user *)arg, &msgs);
	if (ret) {
		rkvenc_err("collect msgs failed %d\n", ret);
		return ret;
	}

	/* Phase 2: If we have SET requests, create a task and submit */
	if (msgs.set_cnt && session->mpp) {
		struct rkvenc_taskqueue *queue = session->mpp->queue;

		mutex_lock(&queue->dev_lock);
		task = rkvenc_alloc_task(session, &msgs);
		mutex_unlock(&queue->dev_lock);
		if (!task) {
			rkvenc_err("alloc task failed\n");
			return -ENOMEM;
		}

		msgs.task = &task->mpp_task;
		msgs.mpp = session->mpp;
		msgs.queue = queue;

		/* Add to pending list */
		kref_get(&task->mpp_task.ref);
		mutex_lock(&session->pending_lock);
		list_add_tail(&task->mpp_task.pending_link, &session->pending_list);
		mutex_unlock(&session->pending_lock);

		/* Add to task queue and trigger worker */
		set_bit(TASK_STATE_PENDING, &task->mpp_task.state);
		mutex_lock(&queue->pending_lock);
		list_add_tail(&task->mpp_task.queue_link, &queue->pending_list);
		mutex_unlock(&queue->pending_lock);

		/* Trigger the worker to run the task */
		kthread_queue_work(&queue->worker, &session->mpp->work);
	}

	/* Phase 3: If we have POLL requests, wait for result */
	if (msgs.poll_cnt) {
		ret = rkvenc_wait_result(session, &msgs);
		if (ret)
			rkvenc_err("wait result ret %d\n", ret);
	}

	return ret;
}

static long rkvenc_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rkvenc_session *session = filp->private_data;
	long ret;

	if (!session)
		return -EINVAL;
	if (mutex_lock_interruptible(&session->ioctl_lock))
		return -ERESTARTSYS;
	ret = rkvenc_dev_ioctl_locked(filp, cmd, arg);
	mutex_unlock(&session->ioctl_lock);
	return ret;
}

/* ---- File operations ---- */
static int rkvenc_dev_open(struct inode *inode, struct file *filp)
{
	struct rkvenc_service *srv = container_of(inode->i_cdev,
						  struct rkvenc_service,
						  mpp_cdev);
	struct rkvenc_session *session;

	session = rkvenc_session_init();
	if (!session)
		return -ENOMEM;

	session->srv = srv;

	if (!srv->sub_devices[MPP_DEVICE_RKVENC]) {
		rkvenc_session_put(session);
		return -ENODEV;
	}

	mutex_lock(&srv->session_lock);
	list_add_tail(&session->service_link, &srv->session_list);
	session->index = atomic_inc_return(&srv->sub_devices[MPP_DEVICE_RKVENC]->session_index);
	mutex_unlock(&srv->session_lock);

	filp->private_data = session;

	return nonseekable_open(inode, filp);
}

static int rkvenc_dev_release(struct inode *inode, struct file *filp)
{
	struct rkvenc_session *session = filp->private_data;

	if (!session) {
		rkvenc_err("session is null\n");
		return -EINVAL;
	}

	atomic_inc(&session->release_request);

	/* Remove from service list */
	if (session->srv) {
		mutex_lock(&session->srv->session_lock);
		list_del_init(&session->service_link);
		mutex_unlock(&session->srv->session_lock);
	}

	/* Wait for all tasks to complete */
	if (atomic_read(&session->task_count) > 0) {
		struct rkvenc_mpp_task *task, *n;

		mutex_lock(&session->pending_lock);
		list_for_each_entry_safe(task, n, &session->pending_list, pending_link) {
			/* Wait for task completion with timeout */
			wait_event_timeout(task->wait,
					   test_bit(TASK_STATE_DONE, &task->state),
					   msecs_to_jiffies(2000));
			list_del_init(&task->pending_link);
			kref_put(&task->ref, rkvenc_free_task_callback);
		}
		mutex_unlock(&session->pending_lock);
	}

	/* Tasks retain the session and its DMA caches after a close timeout. */
	rkvenc_session_put(session);
	filp->private_data = NULL;

	return 0;
}

const struct file_operations rkvenc_fops = {
	.owner		= THIS_MODULE,
	.open		= rkvenc_dev_open,
	.release	= rkvenc_dev_release,
	.unlocked_ioctl = rkvenc_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = rkvenc_dev_ioctl,
#endif
};

/* ---- Service probe/remove ---- */
int rkvenc_service_probe(struct platform_device *pdev)
{
	int ret;
	struct rkvenc_service *srv;
	struct device *dev = &pdev->dev;
	u32 taskqueue_cnt = 0;
	u32 resetgroup_cnt = 0;

	srv = devm_kzalloc(dev, sizeof(*srv), GFP_KERNEL);
	if (!srv)
		return -ENOMEM;

	srv->dev = dev;
	mutex_init(&srv->session_lock);
	INIT_LIST_HEAD(&srv->session_list);
	atomic_set(&srv->shutdown_request, 0);

	/* Read DTS properties */
	of_property_read_u32(dev->of_node, "rockchip,taskqueue-count", &taskqueue_cnt);
	of_property_read_u32(dev->of_node, "rockchip,resetgroup-count", &resetgroup_cnt);
	if (resetgroup_cnt > ARRAY_SIZE(srv->reset_groups))
		return -EINVAL;
	srv->taskqueue_cnt = taskqueue_cnt;
	srv->reset_group_cnt = resetgroup_cnt;

	/* Create the task queue for RKVENC */
	{
		struct rkvenc_taskqueue *queue;

		queue = devm_kzalloc(dev, sizeof(*queue), GFP_KERNEL);
		if (!queue)
			return -ENOMEM;

		mutex_init(&queue->session_lock);
		mutex_init(&queue->pending_lock);
		spin_lock_init(&queue->running_lock);
		mutex_init(&queue->dev_lock);
		INIT_LIST_HEAD(&queue->session_attach);
		INIT_LIST_HEAD(&queue->session_detach);
		INIT_LIST_HEAD(&queue->pending_list);
		INIT_LIST_HEAD(&queue->running_list);
		INIT_LIST_HEAD(&queue->dev_list);
		atomic_set(&queue->reset_request, 0);
		atomic_set(&queue->detach_count, 0);
		atomic_set(&queue->task_id, 0);
		queue->core_idle = 0;

		/* Create kthread worker */
		kthread_init_worker(&queue->worker);
		srv->task_queues[MPP_DEVICE_RKVENC] = queue;
	}

	/* Create reset group */
	if (resetgroup_cnt > 0) {
		struct rkvenc_reset_group *rg;

		rg = devm_kzalloc(dev, sizeof(*rg), GFP_KERNEL);
		if (!rg)
			return -ENOMEM;

		init_rwsem(&rg->rw_sem);
		rg->rw_sem_on = true;
		srv->reset_groups[0] = rg;
	}

	/* Allocate char device */
	ret = alloc_chrdev_region(&srv->dev_id, 0, 1, MPP_SERVICE_NAME);
	if (ret) {
		dev_err(dev, "alloc_chrdev_region failed: %d\n", ret);
		return ret;
	}

	cdev_init(&srv->mpp_cdev, &rkvenc_fops);
	srv->mpp_cdev.owner = THIS_MODULE;

	ret = cdev_add(&srv->mpp_cdev, srv->dev_id, 1);
	if (ret) {
		dev_err(dev, "cdev_add failed: %d\n", ret);
		goto err_cdev;
	}

	srv->cls = class_create(MPP_CLASS_NAME);
	if (IS_ERR(srv->cls)) {
		ret = PTR_ERR(srv->cls);
		dev_err(dev, "class_create failed: %d\n", ret);
		goto err_class;
	}

	srv->child_dev = device_create(srv->cls, dev, srv->dev_id,
				       NULL, MPP_SERVICE_NAME);
	if (IS_ERR(srv->child_dev)) {
		ret = PTR_ERR(srv->child_dev);
		dev_err(dev, "device_create failed: %d\n", ret);
		goto err_device;
	}

	/* Start only after all fallible allocations, so probe failure leaks no thread. */
	{
		struct rkvenc_taskqueue *queue = srv->task_queues[MPP_DEVICE_RKVENC];

		queue->kworker_task = kthread_run(kthread_worker_fn, &queue->worker,
						  "rkvenc-worker");
		if (IS_ERR(queue->kworker_task)) {
			ret = PTR_ERR(queue->kworker_task);
			device_destroy(srv->cls, srv->dev_id);
			goto err_device;
		}
	}
	platform_set_drvdata(pdev, srv);
	dev_info(dev, "mpp_service probe success\n");

	return 0;

err_device:
	class_destroy(srv->cls);
err_class:
	cdev_del(&srv->mpp_cdev);
err_cdev:
	unregister_chrdev_region(srv->dev_id, 1);
	return ret;
}

int rkvenc_service_remove(struct platform_device *pdev)
{
	struct rkvenc_service *srv = platform_get_drvdata(pdev);

	if (!srv)
		return 0;

	/* Stop kthread worker */
	if (srv->task_queues[MPP_DEVICE_RKVENC]) {
		struct rkvenc_taskqueue *queue = srv->task_queues[MPP_DEVICE_RKVENC];

		if (queue->kworker_task) {
			kthread_flush_worker(&queue->worker);
			kthread_stop(queue->kworker_task);
		}
	}

	device_destroy(srv->cls, srv->dev_id);
	class_destroy(srv->cls);
	cdev_del(&srv->mpp_cdev);
	unregister_chrdev_region(srv->dev_id, 1);

	return 0;
}
