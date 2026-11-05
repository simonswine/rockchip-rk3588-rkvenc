// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip VEPU580 encoder driver - Hardware operations
 * Ported from Rockchip BSP mpp_rkvenc2.c
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include "rkvenc_hw.h"

unsigned int rkvenc_debug;
module_param_named(debug, rkvenc_debug, uint, 0644);
MODULE_PARM_DESC(debug, "Debug level bitmask");

/* ---- FD translation tables for VEPU580 ---- */
static const u16 trans_tbl_h264e_v2[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23,
};

static const u16 trans_tbl_h265e_v2[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23,
};

static const u16 trans_tbl_jpege_v2[] = {
	5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16,
};

static const u16 trans_tbl_h264e_v2_osd[] = {
	20, 21, 22, 23, 24, 25, 26, 27,
};

static const u16 trans_tbl_h265e_v2_osd[] = {
	20, 21, 22, 23, 24, 25, 26, 27,
};

static const u16 trans_tbl_jpege_v2_osd[] = {
	3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
	13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
	23, 24, 25, 26, 27, 28, 29, 30, 31, 32,
};

const struct rkvenc_trans_info trans_rkvenc_v2[] = {
	[RKVENC_FMT_H264E] = {
		.count = ARRAY_SIZE(trans_tbl_h264e_v2),
		.table = trans_tbl_h264e_v2,
	},
	[RKVENC_FMT_H265E] = {
		.count = ARRAY_SIZE(trans_tbl_h265e_v2),
		.table = trans_tbl_h265e_v2,
	},
	[RKVENC_FMT_JPEGE] = {
		.count = ARRAY_SIZE(trans_tbl_jpege_v2),
		.table = trans_tbl_jpege_v2,
	},
	[RKVENC_FMT_H264E_OSD] = {
		.count = ARRAY_SIZE(trans_tbl_h264e_v2_osd),
		.table = trans_tbl_h264e_v2_osd,
	},
	[RKVENC_FMT_H265E_OSD] = {
		.count = ARRAY_SIZE(trans_tbl_h265e_v2_osd),
		.table = trans_tbl_h265e_v2_osd,
	},
	[RKVENC_FMT_JPEGE_OSD] = {
		.count = ARRAY_SIZE(trans_tbl_jpege_v2_osd),
		.table = trans_tbl_jpege_v2_osd,
	},
};

/* ---- VEPU580 HW info ---- */
struct rkvenc_hw_info rkvenc_v2_hw_info = {
	.hw = {
		.reg_num = 254,
		.reg_id = 0,
		.reg_en = 4,
		.reg_start = 160,
		.reg_end = 253,
	},
	.reg_class = RKVENC_CLASS_BUTT,
	.reg_msg = {
		[RKVENC_CLASS_BASE] = { .base_s = 0x0000, .base_e = 0x0058 },
		[RKVENC_CLASS_PIC]  = { .base_s = 0x0280, .base_e = 0x03f4 },
		[RKVENC_CLASS_RC]   = { .base_s = 0x1000, .base_e = 0x10e0 },
		[RKVENC_CLASS_PAR]  = { .base_s = 0x1700, .base_e = 0x1cd4 },
		[RKVENC_CLASS_SQI]  = { .base_s = 0x2000, .base_e = 0x21e4 },
		[RKVENC_CLASS_SCL]  = { .base_s = 0x2200, .base_e = 0x2c98 },
		[RKVENC_CLASS_OSD]  = { .base_s = 0x3000, .base_e = 0x347c },
		[RKVENC_CLASS_ST]   = { .base_s = 0x4000, .base_e = 0x42cc },
		[RKVENC_CLASS_DBG]  = { .base_s = 0x5000, .base_e = 0x5354 },
	},
	.fd_class = RKVENC_CLASS_FD_BUTT,
	.fd_reg = {
		[RKVENC_CLASS_FD_BASE] = {
			.class = RKVENC_CLASS_PIC,
			.base_fmt = RKVENC_FMT_BASE,
		},
		[RKVENC_CLASS_FD_OSD] = {
			.class = RKVENC_CLASS_OSD,
			.base_fmt = RKVENC_FMT_OSD_BASE,
		},
	},
	.fmt_reg = {
		.class = RKVENC_CLASS_PIC,
		.base = 0x0300,
		.bitpos = 0,
		.bitlen = 1,
	},
	.enc_start_base = 0x0010,
	.enc_clr_base = 0x0014,
	.int_en_base = 0x0020,
	.int_mask_base = 0x0024,
	.int_clr_base = 0x0028,
	.int_sta_base = 0x002c,
	.enc_wdg_base = 0x0038,
	.err_mask = 0x03f0,
	.enc_rsl = 0x0310,
	.dcsh_class_ofst = 33,
	.vepu_type = RKVENC_VEPU_580,
};

/* ---- Timeout thresholds by resolution ---- */
static const u32 rkvenc2_timeout_thd_by_rsl[][2] = {
	{  3840 * 2160, 200 },
	{  7680 * 4320, 500 },
	{ 0xffffffff,   800 },
};

/* ---- Class register helpers ---- */
static int rkvenc_get_class_msg(struct rkvenc_task *task,
				u32 addr, struct mpp_request *msg)
{
	int i;
	const struct rkvenc_hw_info *hw = task->hw_info;

	if (!msg)
		return -EINVAL;

	memset(msg, 0, sizeof(*msg));
	for (i = 0; i < hw->reg_class; i++) {
		u32 base_s = hw->reg_msg[i].base_s;
		u32 base_e = hw->reg_msg[i].base_e;

		if (addr >= base_s && addr < base_e) {
			msg->offset = base_s;
			msg->size = task->reg[i].size;
			msg->data = task->reg[i].data;
			return 0;
		}
	}

	return -EINVAL;
}

static u32 *rkvenc_get_class_reg(struct rkvenc_task *task, u32 addr)
{
	int i;
	u8 *reg = NULL;
	const struct rkvenc_hw_info *hw = task->hw_info;

	for (i = 0; i < hw->reg_class; i++) {
		u32 base_s = hw->reg_msg[i].base_s;
		u32 base_e = hw->reg_msg[i].base_e;

		if (addr >= base_s && addr < base_e) {
			reg = (u8 *)task->reg[i].data + (addr - base_s);
			break;
		}
	}

	return (u32 *)reg;
}

/* ---- DCHS (Dual-Core Handshake) ---- */
static void rkvenc2_patch_dchs(struct rkvenc_dev *enc, struct rkvenc_task *task)
{
	struct rkvenc_ccu *ccu;
	union rkvenc2_dual_core_handshake_id *dchs;
	union rkvenc2_dual_core_handshake_id *task_dchs = &task->dchs_id;
	const struct rkvenc_hw_info *hw = task->hw_info;
	int core_num;
	int core_id = enc->core_id;
	unsigned long flags;
	int i;

	if (!enc->ccu)
		return;

	if (core_id >= RKVENC_MAX_CORE_NUM) {
		dev_err(enc->dev, "invalid core id %d max %d\n",
			core_id, RKVENC_MAX_CORE_NUM);
		return;
	}

	ccu = enc->ccu;
	dchs = ccu->dchs;
	/* Core IDs need not be dense (e.g. only core 1 enabled). */
	core_num = RKVENC_MAX_CORE_NUM;

	spin_lock_irqsave(&ccu->lock_dchs, flags);

	if (dchs[core_id].working) {
		spin_unlock_irqrestore(&ccu->lock_dchs, flags);
		rkvenc_err("can not config when core %d is still working\n", core_id);
		return;
	}

	/* Find free TX/RX IDs */
	{
		unsigned long id_valid = (unsigned long)-1;
		int txid_map = -1;
		int rxid_map = -1;

		for (i = 0; i < core_num; i++) {
			if (!dchs[i].working)
				continue;
			clear_bit(dchs[i].txid_map, &id_valid);
			clear_bit(dchs[i].rxid_map, &id_valid);
		}

		if (task_dchs->rxe) {
			for (i = 0; i < core_num; i++) {
				if (i == core_id || !dchs[i].working)
					continue;
				if (task_dchs->session_id != dchs[i].session_id)
					continue;
				if (task_dchs->rxid_orig != dchs[i].txid_orig)
					continue;
				rxid_map = dchs[i].txid_map;
				break;
			}
		}

		txid_map = find_first_bit(&id_valid, RKVENC_MAX_DCHS_ID);
		if (txid_map == RKVENC_MAX_DCHS_ID) {
			spin_unlock_irqrestore(&ccu->lock_dchs, flags);
			rkvenc_err("failed to find a txid\n");
			return;
		}

		clear_bit(txid_map, &id_valid);
		task_dchs->txid_map = txid_map;

		if (rxid_map < 0) {
			rxid_map = find_first_bit(&id_valid, RKVENC_MAX_DCHS_ID);
			if (rxid_map == RKVENC_MAX_DCHS_ID) {
				spin_unlock_irqrestore(&ccu->lock_dchs, flags);
				rkvenc_err("failed to find a rxid\n");
				return;
			}
			task_dchs->rxe_map = 0;
		}

		task_dchs->rxid_map = rxid_map;
	}

	task_dchs->txid = task_dchs->txid_map;
	task_dchs->rxid = task_dchs->rxid_map;
	task_dchs->rxe = task_dchs->rxe_map;

	dchs[core_id].val[0] = task_dchs->val[0];
	dchs[core_id].val[1] = task_dchs->val[1];
	task->reg[RKVENC_CLASS_PIC].data[hw->dcsh_class_ofst] = task_dchs->val[0];

	dchs[core_id].working = 1;

	spin_unlock_irqrestore(&ccu->lock_dchs, flags);
}

static void rkvenc2_update_dchs(struct rkvenc_dev *enc,
				struct rkvenc_task *task)
{
	struct rkvenc_ccu *ccu = enc->ccu;
	int core_id = enc->core_id;
	unsigned long flags;

	if (!ccu)
		return;

	if (core_id >= RKVENC_MAX_CORE_NUM) {
		dev_err(enc->dev, "invalid core id %d max %d\n",
			core_id, RKVENC_MAX_CORE_NUM);
		return;
	}

	spin_lock_irqsave(&ccu->lock_dchs, flags);
	ccu->dchs[core_id].val[0] = 0;
	ccu->dchs[core_id].val[1] = 0;
	spin_unlock_irqrestore(&ccu->lock_dchs, flags);
}

/* ---- Timeout threshold calculation ---- */
static void rkvenc2_calc_timeout_thd(struct rkvenc_dev *enc)
{
	const struct rkvenc_hw_info *hw = enc->hw_info;
	union rkvenc2_frame_resolution frm_rsl;
	u32 timeout_ms = 0;
	u32 timeout_thd = 0;
	u32 timeout_thd_cnt;
	u32 i;

	timeout_thd = rkvenc_read(enc, RKVENC_WDG) & 0xff000000;
	frm_rsl.val = rkvenc_read(enc, hw->enc_rsl);
	frm_rsl.val = (frm_rsl.pic_wd8 + 1) * (frm_rsl.pic_hd8 + 1) * 64;

	timeout_thd_cnt = ARRAY_SIZE(rkvenc2_timeout_thd_by_rsl);
	for (i = 0; i < timeout_thd_cnt; i++) {
		if (frm_rsl.val <= rkvenc2_timeout_thd_by_rsl[i][0]) {
			timeout_ms = rkvenc2_timeout_thd_by_rsl[i][1];
			break;
		}
	}

	/* Sub-module watchdogs use one quarter of the frame threshold. */
	if (enc->core_clk_info.clk)
		timeout_thd |= timeout_ms * (clk_get_rate(enc->core_clk_info.clk) / 256000);

	rkvenc_write(enc, RKVENC_WDG, timeout_thd);
}

/* ---- Slice reading ---- */
static void rkvenc2_read_slice_len(struct rkvenc_dev *mpp,
				   struct rkvenc_task *task,
				   u32 *irq_status)
{
	const struct rkvenc_hw_info *hw = mpp->hw_info;
	u32 sli_num = rkvenc_read_relaxed(mpp, RKVENC2_REG_SLICE_NUM_BASE) & 0x3f;
	u32 new_irq_status = rkvenc_read(mpp, hw->int_sta_base);
	union rkvenc2_slice_len_info slice_info;
	u32 i;
	u32 last = 0;
	u32 split = task->task_split;

	if ((new_irq_status != *irq_status) && (new_irq_status & INT_STA_ENC_DONE_STA)) {
		*irq_status |= new_irq_status;
		sli_num = rkvenc_read_relaxed(mpp, RKVENC2_REG_SLICE_NUM_BASE) & 0x3f;
		rkvenc_write(mpp, hw->int_clr_base, new_irq_status);
	}

	last = *irq_status & INT_STA_ENC_DONE_STA;

	for (i = 0; i < sli_num; i++) {
		slice_info.val = rkvenc_read_relaxed(mpp, RKVENC2_REG_SLICE_LEN_BASE);
		last |= slice_info.last;
		if (last && i == sli_num - 1) {
			task->last_slice_found = 1;
			break;
		}

		if (split) {
			kfifo_in(&task->slice_info, &slice_info, 1);
			task->slice_wr_cnt++;
		}
	}

	if (split) {
		if (last && !task->last_slice_found) {
			slice_info.last = 1;
			slice_info.slice_len = 0;
			kfifo_in(&task->slice_info, &slice_info, 1);
		}
	}
}

/* ---- Bitstream overflow handling ---- */
static void rkvenc2_bs_overflow_handle(struct rkvenc_dev *mpp)
{
	struct rkvenc_mpp_task *mpp_task = mpp->cur_task;
	u32 bs_rd, bs_wr, bs_top, bs_bot;

	bs_rd = rkvenc_read(mpp, RKVENC580_REG_ADR_BSBR);
	bs_wr = rkvenc_read(mpp, RKVENC2_REG_ST_BSB);
	bs_top = rkvenc_read(mpp, RKVENC2_REG_ADR_BSBT);
	bs_bot = rkvenc_read(mpp, RKVENC2_REG_ADR_BSBB);

	bs_wr += 128;
	if (bs_wr >= bs_top)
		bs_wr = bs_bot;
	rkvenc_write(mpp, RKVENC580_REG_ADR_BSBS, bs_wr);

	if (mpp_task)
		dev_err(mpp->dev, "task %d bitstream overflow [top=%#08x bot=%#08x wr=%#08x rd=%#08x]\n",
			mpp_task->task_index, bs_top, bs_bot, bs_wr, bs_rd);
}

/* ---- IRQ handler ---- */
irqreturn_t rkvenc_hw_irq(int irq, void *param)
{
	struct rkvenc_dev *enc = param;
	const struct rkvenc_hw_info *hw = enc->hw_info;
	struct rkvenc_mpp_task *mpp_task = NULL;
	struct rkvenc_task *task = NULL;
	u32 irq_status;
	int ret = IRQ_NONE;

	irq_status = rkvenc_read(enc, hw->int_sta_base);

	if (!irq_status)
		return ret;

	/* clear int first */
	rkvenc_write(enc, hw->int_clr_base, irq_status);

	/* prevent watch dog irq storm */
	if (irq_status & INT_STA_WDG_STA)
		rkvenc_write(enc, hw->int_mask_base, INT_STA_WDG_STA);

	if (enc->cur_task) {
		mpp_task = enc->cur_task;
		task = container_of(mpp_task, struct rkvenc_task, mpp_task);
	}

	/* 1. slice split read */
	if (task && task->task_split &&
	    (irq_status & (INT_STA_SLC_DONE_STA | INT_STA_ENC_DONE_STA))) {
		rkvenc2_read_slice_len(enc, task, &irq_status);
		wake_up(&mpp_task->wait);
	}

	/* 2. process slice irq */
	if (irq_status & INT_STA_SLC_DONE_STA)
		ret = IRQ_HANDLED;

	/* 3. process bitstream overflow */
	if (irq_status & INT_STA_BSF_OFLW_STA) {
		rkvenc2_bs_overflow_handle(enc);
		enc->bs_overflow = 1;
		ret = IRQ_HANDLED;
	}

	/* 4. process frame done irq */
	if (irq_status & INT_STA_ENC_DONE_STA) {
		enc->irq_status = irq_status;

		if (enc->bs_overflow) {
			enc->irq_status |= INT_STA_BSF_OFLW_STA;
			enc->bs_overflow = 0;
		}

		if (mpp_task) {
			if (test_and_set_bit(TASK_STATE_HANDLE, &mpp_task->state)) {
				dev_err(enc->dev, "error, task %d already handled, irq %#x\n",
					mpp_task->task_index, enc->irq_status);
				ret = IRQ_HANDLED;
				goto done;
			}
			cancel_delayed_work(&mpp_task->timeout_work);
			set_bit(TASK_STATE_IRQ, &mpp_task->state);
			mpp_task->irq_status = enc->irq_status;
			rkvenc_iommu_dev_deactivate(enc->iommu_info, enc);

			/* Trigger the worker to process the ISR bottom half */
			kthread_queue_work(&enc->queue->worker, &enc->work);
		}

		ret = IRQ_HANDLED;
	}

	/* 5. process error irq */
	if (irq_status & INT_STA_ERROR) {
		enc->irq_status = irq_status;
		dev_err(enc->dev, "error status %08x\n", irq_status);

		if (mpp_task) {
			if (!test_and_set_bit(TASK_STATE_HANDLE, &mpp_task->state)) {
				cancel_delayed_work(&mpp_task->timeout_work);
				set_bit(TASK_STATE_IRQ, &mpp_task->state);
				mpp_task->irq_status = enc->irq_status;
				rkvenc_iommu_dev_deactivate(enc->iommu_info, enc);
				kthread_queue_work(&enc->queue->worker, &enc->work);
			}
		}

		ret = IRQ_HANDLED;
	}

done:
	return ret;
}

/* ---- Run task: write registers and start HW ---- */
int rkvenc_hw_run(struct rkvenc_dev *enc, struct rkvenc_mpp_task *mpp_task)
{
	u32 i, j;
	u32 start_val = 0;
	int ret;
	struct rkvenc_task *task = container_of(mpp_task, struct rkvenc_task, mpp_task);
	const struct rkvenc_hw_info *hw = enc->hw_info;

	rkvenc_debug_enter();

	/* Failed resumes must not lead to MMIO accesses or unbalanced puts. */
	if (enc->iommu_info && enc->iommu_info->pdev) {
		ret = pm_runtime_resume_and_get(&enc->iommu_info->pdev->dev);
		if (ret < 0)
			return ret;
	}
	ret = pm_runtime_resume_and_get(enc->dev);
	if (ret < 0) {
		if (enc->iommu_info && enc->iommu_info->pdev)
			pm_runtime_put_sync(&enc->iommu_info->pdev->dev);
		return ret;
	}
	pm_stay_awake(enc->dev);
	rkvenc_hw_clk_on(enc);

	/* Reset group down_read if available */
	if (enc->reset_group)
		down_read(&enc->reset_group->rw_sem);

	/* Match BSP: ensure IOMMU is attached and fault handler is active */
	ret = rkvenc_iommu_attach(enc->iommu_info);
	if (ret) {
		rkvenc_err("iommu attach failed: %d\n", ret);
		goto err_unlock;
	}

	ret = rkvenc_iommu_dev_activate(enc->iommu_info, enc);
	if (ret) {
		rkvenc_err("iommu activate failed: %d\n", ret);
		goto err_unlock;
	}

	/* Add force clear to avoid pagefault (VEPU580 workaround) */
	if (hw->vepu_type == RKVENC_VEPU_580) {
		rkvenc_write(enc, hw->enc_clr_base, 0x2);
		udelay(5);
		rkvenc_write(enc, hw->enc_clr_base, 0x0);
	}

	/* Clear hardware counter */
	rkvenc_write_relaxed(enc, 0x5300, 0x2);

	/* Patch dual-core handshake IDs */
	rkvenc2_patch_dchs(enc, task);

	/* Write all class registers except enc_start */
	for (i = 0; i < task->w_req_cnt; i++) {
		u32 s, e;
		u32 *regs;
		struct mpp_request msg;
		struct mpp_request *req = &task->w_reqs[i];

		ret = rkvenc_get_class_msg(task, req->offset, &msg);
		if (ret)
			goto err_deactivate;

		s = (req->offset - msg.offset) / sizeof(u32);
		e = s + req->size / sizeof(u32);
		regs = (u32 *)msg.data;
		for (j = s; j < e; j++) {
			u32 off = msg.offset + j * sizeof(u32);

			if (off == hw->enc_start_base) {
				start_val = regs[j];
				continue;
			}
			rkvenc_write_relaxed(enc, off, regs[j]);
		}
	}

	/* flush tlb before starting hardware */
	rkvenc_iommu_flush_tlb(enc->iommu_info);

	/* init current task */
	enc->cur_task = mpp_task;

	/* Calculate and write watchdog timeout threshold */
	rkvenc2_calc_timeout_thd(enc);

	/* Schedule timeout work */
	INIT_DELAYED_WORK(&mpp_task->timeout_work, rkvenc_task_timeout_work);
	schedule_delayed_work(&mpp_task->timeout_work,
			      msecs_to_jiffies(MPP_WORK_TIMEOUT_DELAY));

	/* memory barrier before starting HW */
	wmb();
	rkvenc_write(enc, hw->enc_start_base, start_val);

	rkvenc_debug_leave();

	return 0;

err_deactivate:
	rkvenc2_update_dchs(enc, task);
	rkvenc_iommu_dev_deactivate(enc->iommu_info, enc);
err_unlock:
	if (enc->reset_group)
		up_read(&enc->reset_group->rw_sem);
	rkvenc_hw_clk_off(enc);
	pm_relax(enc->dev);
	pm_runtime_mark_last_busy(enc->dev);
	pm_runtime_put_autosuspend(enc->dev);
	if (enc->iommu_info && enc->iommu_info->pdev)
		pm_runtime_put_sync(&enc->iommu_info->pdev->dev);

	return ret;
}

/* ---- Finish: read status registers back from HW ---- */
int rkvenc_hw_finish(struct rkvenc_dev *mpp, struct rkvenc_mpp_task *mpp_task)
{
	u32 i, j;
	u32 *reg;
	struct rkvenc_task *task = container_of(mpp_task, struct rkvenc_task, mpp_task);

	rkvenc_debug_enter();

	/* Clear DCHS working flag so this core can accept the next task */
	rkvenc2_update_dchs(mpp, task);

	for (i = 0; i < task->r_req_cnt; i++) {
		int ret;
		int s, e;
		struct mpp_request msg;
		struct mpp_request *req = &task->r_reqs[i];

		ret = rkvenc_get_class_msg(task, req->offset, &msg);
		if (ret)
			return -EINVAL;
		s = (req->offset - msg.offset) / sizeof(u32);
		e = s + req->size / sizeof(u32);
		reg = (u32 *)msg.data;
		for (j = s; j < e; j++)
			reg[j] = rkvenc_read_relaxed(mpp, msg.offset + j * sizeof(u32));
	}

	/* Sync bitstream buffer if present */
	if (task->bs_buf) {
		u32 bs_size = rkvenc_read(mpp, 0x4064);

		rkvenc_dma_buf_sync(task->bs_buf, 0, bs_size + task->offset_bs,
				    DMA_FROM_DEVICE, true);
	}

	/* Revert irq status register for userspace readback */
	reg = rkvenc_get_class_reg(task, task->hw_info->int_sta_base);
	if (reg)
		*reg = task->irq_status;

	rkvenc_debug_leave();

	return 0;
}

/* ---- Soft reset ---- */
static int rkvenc_soft_reset(struct rkvenc_dev *enc)
{
	const struct rkvenc_hw_info *hw = enc->hw_info;
	u32 rst_status = 0;
	int ret;

	/* safe reset */
	rkvenc_write(enc, hw->int_mask_base, 0x3FF);
	rkvenc_write(enc, hw->enc_clr_base, 0x1);
	ret = readl_relaxed_poll_timeout(enc->reg_base + hw->int_sta_base,
					 rst_status,
					 rst_status & RKVENC_SCLR_DONE_STA,
					 0, 5);
	if (ret)
		rkvenc_err("safe reset failed\n");
	rkvenc_write(enc, hw->enc_clr_base, 0x2);
	udelay(5);
	rkvenc_write(enc, hw->enc_clr_base, 0);
	rkvenc_write(enc, hw->int_clr_base, 0xffffffff);
	rkvenc_write(enc, hw->int_sta_base, 0);

	return ret;
}

/* ---- Full reset (soft + CRU fallback) ---- */
int rkvenc_hw_reset(struct rkvenc_dev *enc)
{
	int ret;
	struct rkvenc_taskqueue *queue = enc->queue;
	struct rkvenc_ccu *ccu = enc->ccu;
	unsigned long flags;

	/* safe reset first */
	ret = rkvenc_soft_reset(enc);

	/* Suspend the MMU before CRU reset, then restore its page-table base.
	 * A put/get pair is insufficient when device links hold extra PM refs.
	 */
	if (ret && enc->rst_a && enc->rst_h && enc->rst_core) {
		struct device *iommu_dev = &enc->iommu_info->pdev->dev;

		rkvenc_err("soft reset timeout, use cru reset\n");
		rkvenc_iommu_down_write(enc->iommu_info);
		ret = pm_runtime_force_suspend(iommu_dev);
		if (ret) {
			rkvenc_iommu_up_write(enc->iommu_info);
			goto reset_done;
		}
		rkvenc_safe_reset(enc->rst_a);
		rkvenc_safe_reset(enc->rst_h);
		rkvenc_safe_reset(enc->rst_core);
		udelay(5);
		rkvenc_safe_unreset(enc->rst_a);
		rkvenc_safe_unreset(enc->rst_h);
		rkvenc_safe_unreset(enc->rst_core);

		ret = pm_runtime_force_resume(iommu_dev);
		rkvenc_iommu_up_write(enc->iommu_info);
	}

reset_done:
	if (ret) {
		dev_err(enc->dev, "reset failed: %d; disabling core\n", ret);
		enc->disable = 1;
		clear_bit(enc->core_id, &queue->core_idle);
	}

	if (ccu) {
		spin_lock_irqsave(&ccu->lock_dchs, flags);
		ccu->dchs[enc->core_id].val[0] = 0;
		ccu->dchs[enc->core_id].val[1] = 0;
		spin_unlock_irqrestore(&ccu->lock_dchs, flags);
	}

	atomic_set(&enc->reset_request, 0);

	return ret;
}

/* ---- Clock on/off ---- */
void rkvenc_hw_clk_on(struct rkvenc_dev *enc)
{
	rkvenc_clk_safe_enable(enc->aclk_info.clk);
	rkvenc_clk_safe_enable(enc->hclk_info.clk);
	rkvenc_clk_safe_enable(enc->core_clk_info.clk);
}

void rkvenc_hw_clk_off(struct rkvenc_dev *enc)
{
	clk_disable_unprepare(enc->aclk_info.clk);
	clk_disable_unprepare(enc->hclk_info.clk);
	clk_disable_unprepare(enc->core_clk_info.clk);
}

/* ---- HW init during probe ---- */
static int rkvenc_hw_init_clocks(struct rkvenc_dev *enc)
{
	int ret;
	struct device *dev = enc->dev;

	enc->aclk_info.clk = devm_clk_get(dev, "aclk_vcodec");
	if (IS_ERR(enc->aclk_info.clk)) {
		ret = PTR_ERR(enc->aclk_info.clk);
		enc->aclk_info.clk = NULL;
		dev_err(dev, "failed to get aclk_vcodec: %d\n", ret);
	}

	enc->hclk_info.clk = devm_clk_get(dev, "hclk_vcodec");
	if (IS_ERR(enc->hclk_info.clk)) {
		ret = PTR_ERR(enc->hclk_info.clk);
		enc->hclk_info.clk = NULL;
		dev_err(dev, "failed to get hclk_vcodec: %d\n", ret);
	}

	enc->core_clk_info.clk = devm_clk_get(dev, "clk_core");
	if (IS_ERR(enc->core_clk_info.clk)) {
		ret = PTR_ERR(enc->core_clk_info.clk);
		enc->core_clk_info.clk = NULL;
		dev_err(dev, "failed to get clk_core: %d\n", ret);
	}

	/* Set default rates */
	enc->aclk_info.default_rate_hz = 300000000;
	enc->core_clk_info.default_rate_hz = 600000000;

	return 0;
}

static int rkvenc_hw_init_resets(struct rkvenc_dev *enc)
{
	struct device *dev = enc->dev;

	enc->rst_a = devm_reset_control_get_optional(dev, "video_a");
	if (IS_ERR(enc->rst_a)) {
		enc->rst_a = NULL;
		dev_warn(dev, "no video_a reset\n");
	}

	enc->rst_h = devm_reset_control_get_optional(dev, "video_h");
	if (IS_ERR(enc->rst_h)) {
		enc->rst_h = NULL;
		dev_warn(dev, "no video_h reset\n");
	}

	enc->rst_core = devm_reset_control_get_optional(dev, "video_core");
	if (IS_ERR(enc->rst_core)) {
		enc->rst_core = NULL;
		dev_warn(dev, "no video_core reset\n");
	}

	return 0;
}

int rkvenc_hw_probe(struct rkvenc_dev *enc, struct platform_device *pdev)
{
	int ret;
	struct resource *res;
	struct device *dev = &pdev->dev;

	enc->dev = dev;

	/* Read task-capacity from DTS */
	ret = of_property_read_u32(dev->of_node, "rockchip,task-capacity",
				   &enc->task_capacity);
	if (ret)
		enc->task_capacity = 1;

	/* PM runtime */
	pm_runtime_set_autosuspend_delay(dev, 2000);
	pm_runtime_use_autosuspend(dev);
	device_init_wakeup(dev, true);
	pm_runtime_enable(dev);

	/* IRQ */
	enc->irq = platform_get_irq(pdev, 0);
	if (enc->irq < 0) {
		dev_err(dev, "no interrupt resource found\n");
		ret = -ENODEV;
		goto failed;
	}

	/* Register space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto failed;
	}
	enc->reg_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!enc->reg_base) {
		dev_err(dev, "ioremap failed for resource %pR\n", res);
		ret = -ENOMEM;
		goto failed;
	}
	enc->io_base = res->start;

	/* Clocks and resets */
	ret = rkvenc_hw_init_clocks(enc);
	if (ret)
		goto failed;
	ret = rkvenc_hw_init_resets(enc);
	if (ret)
		goto failed;

	/* Set HW info early so it's available for HW ID read */
	enc->hw_info = &rkvenc_v2_hw_info;
	enc->trans_info = trans_rkvenc_v2;

	/* IOMMU */
	enc->iommu_info = rkvenc_iommu_probe(dev);
	if (IS_ERR(enc->iommu_info)) {
		ret = PTR_ERR(enc->iommu_info);
		enc->iommu_info = NULL;
		dev_err_probe(dev, ret, "failed to attach iommu\n");
		goto failed;
	}

	/* Read hardware ID */
	pm_runtime_get_sync(dev);
	rkvenc_hw_clk_on(enc);
	rkvenc_v2_hw_info.hw.hw_id = rkvenc_read(enc,
					rkvenc_v2_hw_info.hw.reg_id * sizeof(u32));
	rkvenc_hw_clk_off(enc);
	pm_runtime_put_sync(dev);

	/* Init state */
	atomic_set(&enc->reset_request, 0);
	atomic_set(&enc->session_index, 0);
	atomic_set(&enc->task_count, 0);
	atomic_set(&enc->task_index, 0);

	enc->session_max_buffers = RKVENC_SESSION_MAX_BUFFERS;

	return 0;

failed:
	device_init_wakeup(dev, false);
	pm_runtime_disable(dev);
	return ret;
}

int rkvenc_hw_remove(struct rkvenc_dev *enc)
{
	rkvenc_iommu_remove(enc->iommu_info);
	device_init_wakeup(enc->dev, false);
	pm_runtime_disable(enc->dev);
	return 0;
}
