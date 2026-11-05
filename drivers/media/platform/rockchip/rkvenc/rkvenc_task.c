// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip VEPU580 encoder driver - Task lifecycle management
 * Ported from Rockchip BSP mpp_common.c
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "rkvenc_hw.h"

/* ---- Task init/finish/finalize ---- */
int rkvenc_task_init(struct rkvenc_session *session, struct rkvenc_mpp_task *task)
{
	INIT_LIST_HEAD(&task->pending_link);
	INIT_LIST_HEAD(&task->queue_link);
	INIT_LIST_HEAD(&task->mem_region_list);
	task->state = 0;
	task->mem_count = 0;
	task->session = session;

	return 0;
}

static struct rkvenc_dev *rkvenc_get_task_used_device(struct rkvenc_mpp_task *task,
						     struct rkvenc_session *session)
{
	if (task->mpp)
		return task->mpp;
	return session->mpp;
}

void rkvenc_free_task_callback(struct kref *ref)
{
	struct rkvenc_mpp_task *task = container_of(ref, struct rkvenc_mpp_task, ref);
	struct rkvenc_session *session;
	struct rkvenc_dev *mpp;
	struct rkvenc_task *enc_task = container_of(task, struct rkvenc_task, mpp_task);

	if (!task->session) {
		rkvenc_err("task %p, task->session is null.\n", task);
		return;
	}
	session = task->session;
	mpp = rkvenc_get_task_used_device(task, session);

	/* Release memory regions and free class register buffers */
	rkvenc_task_finalize(session, task);

	/* Free class register buffers */
	{
		int i;

		for (i = 0; i < RKVENC_CLASS_BUTT; i++) {
			kfree(enc_task->reg[i].data);
			enc_task->reg[i].data = NULL;
			enc_task->reg[i].size = 0;
			enc_task->reg[i].valid = 0;
		}
	}

	atomic_dec(&session->task_count);
	atomic_dec(&mpp->task_count);

	kfree(enc_task);
}

int rkvenc_task_finish(struct rkvenc_session *session,
		       struct rkvenc_mpp_task *task)
{
	struct rkvenc_dev *mpp = rkvenc_get_task_used_device(task, session);
	struct rkvenc_taskqueue *queue = mpp->queue;

	/* Read status registers back from HW */
	rkvenc_hw_finish(mpp, task);

	/* Handle reset if needed */
	if (mpp->reset_group) {
		up_read(&mpp->reset_group->rw_sem);
		if (atomic_read(&mpp->reset_request) > 0)
			rkvenc_hw_reset(mpp);
	}

	/* Power off encoder and its IOMMU */
	rkvenc_hw_clk_off(mpp);
	pm_relax(mpp->dev);
	pm_runtime_mark_last_busy(mpp->dev);
	pm_runtime_put_autosuspend(mpp->dev);
	if (mpp->iommu_info && mpp->iommu_info->pdev)
		pm_runtime_put_sync(&mpp->iommu_info->pdev->dev);

	set_bit(TASK_STATE_FINISH, &task->state);
	set_bit(TASK_STATE_DONE, &task->state);

	/* Wake up the GET thread */
	wake_up(&task->wait);

	/* Pop from running queue */
	{
		unsigned long flags;

		spin_lock_irqsave(&queue->running_lock, flags);
		list_del_init(&task->queue_link);
		spin_unlock_irqrestore(&queue->running_lock, flags);
		kref_put(&task->ref, rkvenc_free_task_callback);
	}

	return 0;
}

int rkvenc_task_finalize(struct rkvenc_session *session,
			 struct rkvenc_mpp_task *task)
{
	struct rkvenc_mem_region *mem_region = NULL, *n;
	struct rkvenc_dev *mpp = rkvenc_get_task_used_device(task, session);

	list_for_each_entry_safe(mem_region, n, &task->mem_region_list, reg_link) {
		if (!mem_region->is_dup) {
			rkvenc_iommu_down_read(mpp->iommu_info);
			rkvenc_dma_release(session->dma, mem_region->hdl);
			rkvenc_iommu_up_read(mpp->iommu_info);
		}
		list_del_init(&mem_region->reg_link);
	}

	return 0;
}

/* ---- Memory region attach ---- */
static struct rkvenc_mem_region *
rkvenc_task_attach_fd(struct rkvenc_mpp_task *task, int fd)
{
	struct rkvenc_mem_region *mem_region = NULL, *loop = NULL, *n;
	struct rkvenc_dma_buffer *buffer = NULL;
	struct rkvenc_dev *mpp = task->session->mpp;
	struct rkvenc_dma_session *dma = task->session->dma;
	u32 mem_num = ARRAY_SIZE(task->mem_regions);
	bool found = false;

	if (fd <= 0 || !dma || !mpp)
		return ERR_PTR(-EINVAL);

	if (task->mem_count > mem_num) {
		rkvenc_err("mem_count %d must less than %d\n", task->mem_count, mem_num);
		return ERR_PTR(-ENOMEM);
	}

	/* find fd whether had import */
	list_for_each_entry_safe_reverse(loop, n, &task->mem_region_list, reg_link) {
		if (loop->fd == fd) {
			found = true;
			break;
		}
	}

	mem_region = &task->mem_regions[task->mem_count];
	if (found) {
		memcpy(mem_region, loop, sizeof(*loop));
		mem_region->reg_class = 0;
		mem_region->is_dup = true;
	} else {
		rkvenc_iommu_down_read(mpp->iommu_info);
		buffer = rkvenc_dma_import_fd(mpp->iommu_info, dma, fd, 0);
		rkvenc_iommu_up_read(mpp->iommu_info);
		if (IS_ERR(buffer)) {
			rkvenc_err("can't import dma-buf %d\n", fd);
			return ERR_CAST(buffer);
		}

		mem_region->hdl = buffer;
		mem_region->iova = buffer->iova;
		mem_region->len = buffer->size;
		mem_region->fd = fd;
		mem_region->reg_class = 0;
		mem_region->is_dup = false;
	}
	task->mem_count++;
	INIT_LIST_HEAD(&mem_region->reg_link);
	list_add_tail(&mem_region->reg_link, &task->mem_region_list);

	return mem_region;
}

/* ---- FD translation ---- */
int rkvenc_translate_reg_address(struct rkvenc_session *session,
				 struct rkvenc_mpp_task *task, int fmt, u32 reg_class,
				 u32 *reg, struct reg_offset_info *off_inf)
{
	int i;
	int cnt;
	const u16 *tbl;
	struct rkvenc_dev *mpp = rkvenc_get_task_used_device(task, session);

	if (session->trans_count > 0) {
		cnt = session->trans_count;
		tbl = session->trans_table;
	} else {
		cnt = mpp->trans_info[fmt].count;
		tbl = mpp->trans_info[fmt].table;
	}

	for (i = 0; i < cnt; i++) {
		int usr_fd;
		u32 offset;
		struct rkvenc_mem_region *mem_region;

		if (session->msg_flags & MPP_FLAGS_REG_NO_OFFSET) {
			usr_fd = reg[tbl[i]];
			offset = 0;
		} else {
			usr_fd = reg[tbl[i]] & 0x3ff;
			offset = reg[tbl[i]] >> 10;
		}

		if (usr_fd == 0)
			continue;

		mem_region = rkvenc_task_attach_fd(task, usr_fd);
		if (IS_ERR(mem_region)) {
			rkvenc_err("reg[%3d]: 0x%08x fd %d failed\n",
				   tbl[i], reg[tbl[i]], usr_fd);
			return PTR_ERR(mem_region);
		}
		mem_region->reg_class = reg_class;
		mem_region->reg_idx = tbl[i];
		reg[tbl[i]] = mem_region->iova + offset;
	}

	return 0;
}

int rkvenc_extract_reg_offset_info(struct reg_offset_info *off_inf,
				   struct mpp_request *req)
{
	int max_size = ARRAY_SIZE(off_inf->elem);
	int cnt = req->size / sizeof(off_inf->elem[0]);

	if ((cnt + off_inf->cnt) > max_size) {
		rkvenc_err("count %d, total %d, max_size %d\n",
			   cnt, off_inf->cnt, max_size);
		return -EINVAL;
	}
	if (copy_from_user(&off_inf->elem[off_inf->cnt], req->data, req->size)) {
		rkvenc_err("copy_from_user failed\n");
		return -EINVAL;
	}
	off_inf->cnt += cnt;

	return 0;
}

int rkvenc_query_reg_offset_info(struct reg_offset_info *off_inf, u32 index)
{
	if (off_inf) {
		int i;

		for (i = 0; i < off_inf->cnt; i++) {
			if (off_inf->elem[i].index == index)
				return off_inf->elem[i].offset;
		}
	}

	return 0;
}

/* ---- Task timeout ---- */
void rkvenc_task_timeout_work(struct work_struct *work_s)
{
	struct rkvenc_mpp_task *task = container_of(to_delayed_work(work_s),
						   struct rkvenc_mpp_task,
						   timeout_work);
	struct rkvenc_dev *mpp;
	struct rkvenc_session *session = task->session;

	if (!session) {
		rkvenc_err("task %p, task->session is null.\n", task);
		return;
	}

	mpp = rkvenc_get_task_used_device(task, session);
	if (!mpp) {
		rkvenc_err("mpp is null\n");
		return;
	}

	disable_irq(mpp->irq);
	if (test_and_set_bit(TASK_STATE_HANDLE, &task->state)) {
		enable_irq(mpp->irq);
		return;
	}
	rkvenc_err("task %d processing time out!\n", task->task_index);
	set_bit(TASK_STATE_TIMEOUT, &task->state);
	enable_irq(mpp->irq);

	kthread_queue_work(&mpp->queue->worker, &mpp->work);
}
