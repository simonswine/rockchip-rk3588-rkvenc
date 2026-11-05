/* SPDX-License-Identifier: (GPL-2.0+ OR MIT) */
/*
 * Rockchip VEPU580 encoder driver - Hardware definitions
 * Ported from Rockchip BSP mpp_rkvenc2.c / mpp_common.h
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 */

#ifndef __RKVENC_HW_H__
#define __RKVENC_HW_H__

#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/dma-direction.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <uapi/linux/rkvenc.h>

/* ---- Debug infrastructure ---- */
extern unsigned int rkvenc_debug;

#define DEBUG_IRQ_STATUS		0x00000004
#define DEBUG_IOMMU			0x00000008
#define DEBUG_IOCTL			0x00000010
#define DEBUG_FUNCTION			0x00000020
#define DEBUG_TASK_INFO			0x00000200
#define DEBUG_DUMP_ERR_REG		0x00000400
#define DEBUG_SRAM_INFO			0x00200000
#define DEBUG_CCU			0x01000000
#define DEBUG_CORE			0x02000000
#define DEBUG_SLICE			0x00000002

#define rkvenc_debug_unlikely(type)	(unlikely(rkvenc_debug & (type)))

#define rkvenc_debug_func(type, fmt, args...)			\
	do {							\
		if (unlikely(rkvenc_debug & (type)))		\
			pr_info("%s:%d: " fmt,			\
				__func__, __LINE__, ##args);	\
	} while (0)

#define rkvenc_dbg(type, fmt, args...)				\
	do {							\
		if (unlikely(rkvenc_debug & (type)))		\
			pr_info(fmt, ##args);			\
	} while (0)

#define rkvenc_debug_enter()					\
	do {							\
		if (unlikely(rkvenc_debug & DEBUG_FUNCTION))	\
			pr_info("%s:%d: enter\n",		\
				__func__, __LINE__);		\
	} while (0)

#define rkvenc_debug_leave()					\
	do {							\
		if (unlikely(rkvenc_debug & DEBUG_FUNCTION))	\
			pr_info("%s:%d: leave\n",		\
				__func__, __LINE__);		\
	} while (0)

#define rkvenc_err(fmt, args...)				\
	pr_err("%s:%d: " fmt, __func__, __LINE__, ##args)

#define rkvenc_dbg_core(fmt, args...)				\
	do {							\
		if (unlikely(rkvenc_debug & DEBUG_CORE))		\
			pr_info(fmt, ##args);			\
	} while (0)

#define rkvenc_dbg_slice(fmt, args...)				\
	do {							\
		if (unlikely(rkvenc_debug & DEBUG_SLICE))	\
			pr_info(fmt, ##args);			\
	} while (0)

/* ---- Constants ---- */
#define MPP_DRIVER_NAME			"rkvenc"
#define MPP_SERVICE_NAME		"mpp_service"
#define MPP_CLASS_NAME			"mpp_class"

#define MPP_MAX_CORE_NUM		4
#define MPP_MAX_TASK_CAPACITY		16
#define MPP_MAX_MSG_NUM			32
#define MPP_MAX_REG_TRANS_NUM		128
/* Above the largest hardware watchdog threshold (800 ms). */
#define MPP_WORK_TIMEOUT_DELAY		1000
#define RKVENC_SESSION_MAX_BUFFERS	40
#define RKVENC_MAX_CORE_NUM		2
#define RKVENC_MAX_DCHS_ID		4

/* ---- Device types (must match MPP userspace MppClientType enum) ---- */
enum MPP_DEVICE_TYPE {
	MPP_DEVICE_VDPU1		= 0,
	MPP_DEVICE_VDPU2		= 1,
	MPP_DEVICE_VDPU1_PP		= 2,
	MPP_DEVICE_VDPU2_PP		= 3,
	MPP_DEVICE_AV1DEC		= 4,

	MPP_DEVICE_HEVC_DEC		= 8,
	MPP_DEVICE_RKVDEC		= 9,

	MPP_DEVICE_AVSPLUS_DEC		= 12,
	MPP_DEVICE_RKJPEGD		= 13,

	MPP_DEVICE_RKVENC		= 16,
	MPP_DEVICE_VEPU1		= 17,
	MPP_DEVICE_VEPU2		= 18,
	MPP_DEVICE_VEPU2_JPEG		= 19,
	MPP_DEVICE_RKJPEGE		= 20,

	MPP_DEVICE_VEPU22		= 24,

	MPP_DEVICE_IEP2			= 28,
	MPP_DEVICE_VDPP			= 29,
	MPP_DEVICE_BUTT			= 30,
};

/* ---- Register class definitions for VEPU580 ---- */
enum RKVENC_CLASS_TYPE {
	RKVENC_CLASS_BASE	= 0,
	RKVENC_CLASS_PIC	= 1,
	RKVENC_CLASS_RC		= 2,
	RKVENC_CLASS_PAR	= 3,
	RKVENC_CLASS_SQI	= 4,
	RKVENC_CLASS_SCL	= 5,
	RKVENC_CLASS_OSD	= 6,
	RKVENC_CLASS_ST		= 7,
	RKVENC_CLASS_DBG	= 8,
	RKVENC_CLASS_BUTT,
};

enum RKVENC_CLASS_FD {
	RKVENC_CLASS_FD_BASE	= 0,
	RKVENC_CLASS_FD_OSD	= 1,
	RKVENC_CLASS_FD_BUTT,
};

enum RKVENC_FORMAT_TYPE {
	RKVENC_FMT_BASE		= 0x0000,
	RKVENC_FMT_H264E	= RKVENC_FMT_BASE + 0,
	RKVENC_FMT_H265E	= RKVENC_FMT_BASE + 1,
	RKVENC_FMT_JPEGE	= RKVENC_FMT_BASE + 2,

	RKVENC_FMT_OSD_BASE	= 0x1000,
	RKVENC_FMT_H264E_OSD	= RKVENC_FMT_OSD_BASE + 0,
	RKVENC_FMT_H265E_OSD	= RKVENC_FMT_OSD_BASE + 1,
	RKVENC_FMT_JPEGE_OSD	= RKVENC_FMT_OSD_BASE + 2,
	RKVENC_FMT_BUTT,
};

enum RKVENC_VEPU_TYPE {
	RKVENC_VEPU_580		= 0,
	RKVENC_VEPU_BUTT,
};

/* ---- Register offsets ---- */
#define RKVENC_WDG			0x0038

/* PIC class data array indices (relative to PIC class base 0x0280) */
#define RKVENC2_PIC_BASE		0x0280
#define RKVENC2_REG_ENC_PIC		((0x0300 - RKVENC2_PIC_BASE) / sizeof(u32))
#define RKVENC2_REG_EXT_LINE_BUF_BASE	22
#define RKVENC2_REG_SLI_SPLIT		56

/* Absolute MMIO byte offsets used by runtime paths (match BSP mpp_rkvenc2.c) */
#define RKVENC2_REG_ADR_BSBT		0x02b0
#define RKVENC2_REG_ADR_BSBB		0x02b4
#define RKVENC2_REG_ADR_BSBS		0x02b8
#define RKVENC2_REG_ADR_BSBR		0x02bc

#define RKVENC580_REG_ADR_BSBR		0x02b8
#define RKVENC580_REG_ADR_BSBS		0x02bc

#define RKVENC2_REG_ST_BSB		0x402c

#define RKVENC2_REG_SLICE_NUM_BASE	0x4034
#define RKVENC2_REG_SLICE_LEN_BASE	0x4038

#define DCHS_REG_OFFSET			0x0304

#define RKVENC2_BIT_ENC_STND		BIT(0)
#define RKVENC2_BIT_SLEN_FIFO		BIT(30)
#define RKVENC2_BIT_SLI_SPLIT		BIT(0)
#define RKVENC2_BIT_SLI_FLUSH		BIT(15)
#define RKVENC2_BIT_REC_FBC_DIS		BIT(31)
#define RKVENC2_BIT_VAL_H264		0
#define RKVENC2_BIT_VAL_H265		1

#define RKVENC_SCLR_DONE_STA		BIT(2)

/* VEPU580 interrupt status bits (not register indices). */
#define INT_STA_ENC_DONE_STA		BIT(0)
#define INT_STA_SLC_DONE_STA		BIT(3)
#define INT_STA_BSF_OFLW_STA		BIT(4)
#define INT_STA_BRSP_OTSD_STA		BIT(5)
#define INT_STA_WBUS_ERR_STA		BIT(6)
#define INT_STA_RBUS_ERR_STA		BIT(7)
#define INT_STA_WDG_STA			BIT(8)
#define INT_STA_ERROR			(INT_STA_BRSP_OTSD_STA | \
					 INT_STA_WBUS_ERR_STA | \
					 INT_STA_RBUS_ERR_STA | INT_STA_WDG_STA)

/* Dual-core handshake */
#define DCHS_TXE			BIT(4)

/* ---- Clock mode ---- */
enum MPP_CLOCK_MODE {
	CLK_MODE_DEFAULT,
	CLK_MODE_REDUCE,
	CLK_MODE_NORMAL,
	CLK_MODE_ADVANCED,
	CLK_MODE_DEBUG,
};

/* ---- Task state bits ---- */
enum {
	TASK_STATE_PENDING	= 0,
	TASK_STATE_RUNNING	= 1,
	TASK_STATE_START	= 2,
	TASK_STATE_IRQ		= 3,
	TASK_STATE_HANDLE	= 4,
	TASK_STATE_TIMEOUT	= 5,
	TASK_STATE_FINISH	= 6,
	TASK_STATE_DONE		= 7,
	TASK_STATE_ABORT	= 8,
};

/* ---- Clock info ---- */
struct rkvenc_clk_info {
	struct clk *clk;
	unsigned long debug_rate_hz;
	unsigned long reduce_rate_hz;
	unsigned long normal_rate_hz;
	unsigned long advanced_rate_hz;
	unsigned long default_rate_hz;
	unsigned long used_rate_hz;
	unsigned long real_rate_hz;
};

/* ---- HW info structure (register class map for VEPU580) ---- */
struct mpp_hw_info {
	s32 reg_num;
	s32 reg_id;
	s32 reg_en;
	s32 reg_start;
	s32 reg_end;
	u32 hw_id;
};

struct rkvenc_reg_msg {
	u32 base_s;
	u32 base_e;
};

struct rkvenc_fmt_reg {
	u32 class;
	u32 base;
	u32 bitpos;
	u32 bitlen;
};

struct rkvenc_fd_reg {
	u32 class;
	u32 base_fmt;
};

struct rkvenc_hw_info {
	struct mpp_hw_info hw;

	u32 reg_class;
	struct rkvenc_reg_msg reg_msg[RKVENC_CLASS_BUTT];

	u32 fd_class;
	struct rkvenc_fd_reg fd_reg[RKVENC_CLASS_FD_BUTT];

	struct rkvenc_fmt_reg fmt_reg;

	u32 enc_start_base;
	u32 enc_clr_base;
	u32 int_en_base;
	u32 int_mask_base;
	u32 int_clr_base;
	u32 int_sta_base;
	u32 enc_wdg_base;
	u32 err_mask;
	u32 enc_rsl;
	u32 dcsh_class_ofst;
	u32 vepu_type;
};

/* ---- FD translation table ---- */
struct rkvenc_trans_info {
	const int count;
	const u16 *table;
};

/* ---- Memory region for DMA-buf tracking ---- */
#define MPP_MAX_MEM_REGION		128

struct rkvenc_mem_region {
	struct list_head reg_link;
	/* DMABUF information */
	dma_addr_t iova;
	unsigned long len;
	int fd;
	u32 reg_class;
	u32 reg_idx;
	bool is_dup;
	/* Import into every usable core before returning from submission. */
	void *hdl;
	struct rkvenc_dma_buffer *core_hdl[RKVENC_MAX_CORE_NUM];
};

/* ---- Offset info for register address offsets ---- */
struct reg_offset_elem {
	u32 index;
	u32 offset;
};

struct reg_offset_info {
	u32 cnt;
	struct reg_offset_elem elem[MPP_MAX_REG_TRANS_NUM];
};

/* ---- DMA buffer management ---- */
#define MPP_SESSION_MAX_BUFFERS		60

struct rkvenc_dma_buffer {
	struct list_head link;
	struct rkvenc_dma_session *dma;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	enum dma_data_direction dir;
	dma_addr_t iova;
	unsigned long size;
	void *vaddr;
	struct kref ref;
	struct device *dev;
};

struct rkvenc_dma_session {
	struct list_head unused_list;
	struct list_head used_list;
	struct list_head static_list;
	struct rkvenc_dma_buffer dma_bufs[MPP_SESSION_MAX_BUFFERS];
	struct mutex list_mutex;
	u32 max_buffers;
	int buffer_count;
	struct device *dev;
};

/* ---- IOMMU info ---- */
struct rkvenc_iommu_info {
	struct rw_semaphore *rw_sem;
	struct rw_semaphore rw_sem_self;
	struct device *dev;
	struct platform_device *pdev;
	struct iommu_domain *domain;
	struct iommu_group *group;
	spinlock_t dev_lock;
	struct rkvenc_dev *dev_active;
	int irq;
	int got_irq;
};

/* ---- Task queue ---- */
struct rkvenc_taskqueue {
	struct mutex session_lock;
	struct mutex pending_lock;
	spinlock_t running_lock;
	struct mutex dev_lock;

	struct list_head session_attach;
	struct list_head session_detach;
	struct list_head pending_list;
	struct list_head running_list;
	struct list_head dev_list;

	struct kthread_worker worker;
	struct task_struct *kworker_task;

	u32 task_capacity;
	atomic_t reset_request;
	atomic_t detach_count;
	atomic_t task_id;

	/* Multi-core support */
	struct rkvenc_dev *cores[MPP_MAX_CORE_NUM];
	u32 core_count;
	u32 core_id_max;
	unsigned long core_idle;
	unsigned long dev_active_flags;
};

/* ---- Reset group ---- */
struct rkvenc_reset_group {
	struct rw_semaphore rw_sem;
	bool rw_sem_on;
	struct reset_control *resets[3];
	struct rkvenc_taskqueue *queue;
};

/* ---- GRF info ---- */
#define MPP_GRF_VAL_MASK		0xffff

struct rkvenc_grf_info {
	struct regmap *grf;
	u32 offset;
	u32 val;
};

/* ---- MPP service ---- */
struct rkvenc_service {
	struct device *dev;
	struct cdev mpp_cdev;
	dev_t dev_id;
	struct class *cls;
	struct device *child_dev;

	u32 taskqueue_cnt;
	struct rkvenc_taskqueue *task_queues[MPP_DEVICE_BUTT];
	u32 reset_group_cnt;
	struct rkvenc_reset_group *reset_groups[MPP_DEVICE_BUTT];

	struct rkvenc_dev *sub_devices[MPP_DEVICE_BUTT];
	unsigned long hw_support;
	struct rkvenc_grf_info grf_infos[MPP_DEVICE_BUTT];

	struct mutex session_lock;
	struct list_head session_list;

	u32 timing_en;
	atomic_t shutdown_request;
};

/* ---- MPP task ---- */
struct rkvenc_mpp_task {
	struct list_head pending_link;
	struct list_head queue_link;
	struct list_head mem_region_list;
	unsigned long state;
	u32 mem_count;
	struct rkvenc_session *session;
	struct rkvenc_dev *mpp;
	s32 core_id;
	const struct mpp_hw_info *hw_info;
	u32 *reg;

	struct kref ref;
	wait_queue_head_t wait;
	atomic_t abort_request;
	u32 task_index;
	u32 task_id;
	struct delayed_work timeout_work;
	u32 irq_status;
	int result;
	u32 hw_cycles;
	s64 hw_time;

	/* Memory regions */
	struct rkvenc_mem_region mem_regions[MPP_MAX_MEM_REGION];

	/* Timing */
	ktime_t start;
	ktime_t part;
	ktime_t on_create;
	ktime_t on_create_end;
	ktime_t on_pending;
	ktime_t on_run;
	ktime_t on_sched_timeout;
	ktime_t on_run_end;
	ktime_t on_irq;
	ktime_t on_cancel_timeout;
	ktime_t on_finish;
};

/* ---- RKVENC-specific task ---- */
struct rkvenc_class_msg {
	u32 *data;
	u32 size;
	u32 valid;
};

union rkvenc2_dual_core_handshake_id {
	u64 val[2];
	struct {
		/* val[0]: low dword is the VEPU580 register at 0x304. */
		u32 txid	: 2;
		u32 rxid	: 2;
		u32 txe		: 1;
		u32 rxe		: 1;
		u32 reserved0	: 2;
		u32 dly		: 8;
		u32 offset	: 11;
		u32 reserved1	: 5;
		u32 session_id;
		/* val[1]: software bookkeeping only. */
		u32 working	: 1;
		u32 txe_orig	: 1;
		u32 rxe_orig	: 1;
		u32 txid_orig	: 2;
		u32 rxid_orig	: 2;
		u32 txe_map	: 1;
		u32 rxe_map	: 1;
		u32 txid_map	: 2;
		u32 rxid_map	: 2;
		u32 reserved2	: 19;
		u32 reserved3;
	};
};

union rkvenc2_slice_len_info {
	u32 val;
	struct {
		u32 slice_len	: 31;
		u32 last	: 1;
	};
};

union rkvenc2_frame_resolution {
	u32 val;
	struct {
		u32 pic_wd8	: 11;
		u32 reserved0	: 5;
		u32 pic_hd8	: 11;
		u32 reserved1	: 5;
	};
};

struct rkvenc_poll_slice_cfg {
	u32 count_max;
	u32 count_ret;
};

#define RKVENC_SLICE_FIFO_LEN	512

struct rkvenc2_rcb_info_elem {
	u32 index;
	u32 size;
};

struct rkvenc2_rcb_info {
	u32 cnt;
	struct rkvenc2_rcb_info_elem elem[20];
};

struct rkvenc_task {
	struct rkvenc_mpp_task mpp_task;
	const struct rkvenc_hw_info *hw_info;

	/* Class-based register buffers */
	struct rkvenc_class_msg reg[RKVENC_CLASS_BUTT];
	struct mpp_request w_reqs[RKVENC_CLASS_BUTT];
	struct mpp_request r_reqs[RKVENC_CLASS_BUTT];
	u32 w_req_cnt;
	u32 r_req_cnt;

	/* Snapshot scratch layout; later submissions may change session RCB info. */
	struct rkvenc2_rcb_info rcb_inf;

	/* Register offset info */
	struct reg_offset_info off_inf;

	/* Raw IOVAs are valid only on the session's main core. */
	unsigned long core_mask;

	/* Format and flags */
	u32 fmt;
	int clk_mode;
	u32 irq_status;
	u32 task_split;
	u32 task_split_done;

	/* Dual-core handshake ID */
	union rkvenc2_dual_core_handshake_id dchs_id;

	/* Slice split info */
	DECLARE_KFIFO(slice_info, union rkvenc2_slice_len_info, RKVENC_SLICE_FIFO_LEN);
	u32 slice_wr_cnt;
	u32 slice_rd_cnt;
	u32 last_slice_found;

	/* Bitstream buffer tracking */
	struct rkvenc_dma_buffer *bs_buf;
	u32 offset_bs;
};

/* Forward declaration */
struct rkvenc_task_msgs;

/* ---- Session ---- */
struct rkvenc_session {
	struct kref ref;
	struct mutex ioctl_lock;
	int pid;
	int index;

	struct rkvenc_dev *mpp;
	struct rkvenc_service *srv;
	enum MPP_DEVICE_TYPE device_type;
	/* Main-core cache for pre-import ioctls, plus per-core task mappings. */
	struct rkvenc_dma_session *dma;
	struct rkvenc_dma_session *core_dma[RKVENC_MAX_CORE_NUM];

	struct mutex pending_lock;
	struct list_head pending_list;
	struct list_head service_link;
	struct list_head session_link;

	atomic_t task_count;
	atomic_t release_request;

	/* FD translation table (from userspace) */
	u16 trans_table[MPP_MAX_REG_TRANS_NUM];
	u32 trans_count;
	u32 msg_flags;

	/* Session private data */
	void *priv;

};

/* ---- Task messages ---- */
struct rkvenc_task_msgs {
	struct list_head list;
	struct list_head list_session;

	struct rkvenc_session *session;
	struct rkvenc_taskqueue *queue;
	struct rkvenc_mpp_task *task;
	struct rkvenc_dev *mpp;

	u32 flags;
	u32 req_cnt;
	u32 set_cnt;
	u32 poll_cnt;
	struct mpp_request *poll_req;
	struct mpp_request reqs[MPP_MAX_MSG_NUM];

	int ext_fd;
};

/* ---- Session private data ---- */

enum {
	ENC_INFO_BASE = 0,
	ENC_INFO_WIDTH,
	ENC_INFO_HEIGHT,
	ENC_INFO_FORMAT,
	ENC_INFO_FPS_IN,
	ENC_INFO_FPS_OUT,
	ENC_INFO_RC_MODE,
	ENC_INFO_BITRATE,
	ENC_INFO_GOP_SIZE,
	ENC_INFO_FPS_CALC,
	ENC_INFO_PROFILE,
	ENC_INFO_BUTT,
};

enum {
	CODEC_INFO_FLAG_NULL = 0,
	CODEC_INFO_FLAG_NUMBER,
	CODEC_INFO_FLAG_STRING,
	CODEC_INFO_FLAG_BUTT,
};

struct codec_info_elem {
	u32 type;
	u32 flag;
	u64 data;
};

struct rkvenc2_codec_info {
	u32 flag;
	u64 val;
};

struct rkvenc2_session_priv {
	struct rw_semaphore rw_sem;
	struct rkvenc2_rcb_info rcb_inf;
	struct rkvenc2_codec_info codec_info[ENC_INFO_BUTT];
};

/* ---- CCU (Core Coordination Unit) ---- */
struct rkvenc_ccu {
	struct mutex lock;
	struct list_head core_list;
	int core_num;
	struct rkvenc_dev *main_core;
	spinlock_t lock_dchs;
	union rkvenc2_dual_core_handshake_id dchs[RKVENC_MAX_CORE_NUM];
};

/* ---- Per-core encoder device ---- */
struct rkvenc_dev {
	struct device *dev;
	void __iomem *reg_base;
	resource_size_t io_base;
	int irq;
	u32 irq_status;

	/* Framework */
	struct rkvenc_service *srv;
	struct rkvenc_taskqueue *queue;
	struct rkvenc_iommu_info *iommu_info;
	struct rkvenc_reset_group *reset_group;
	struct rkvenc_grf_info *grf_info;
	struct rkvenc_mpp_task *cur_task;
	iommu_fault_handler_t fault_handler;

	/* HW variant data */
	const struct rkvenc_hw_info *hw_info;
	const struct rkvenc_trans_info *trans_info;

	s32 core_id;
	u32 task_capacity;
	u32 session_max_buffers;
	u32 msgs_cap;
	bool auto_freq_en;

	/* Clocks and resets */
	struct rkvenc_clk_info aclk_info;
	struct rkvenc_clk_info hclk_info;
	struct rkvenc_clk_info core_clk_info;
	struct reset_control *rst_a;
	struct reset_control *rst_h;
	struct reset_control *rst_core;

	/* Multi-core */
	struct rkvenc_ccu *ccu;
	struct list_head core_link;
	struct list_head queue_link;

	/* Work */
	struct kthread_work work;

	/* State */
	atomic_t reset_request;
	atomic_t session_index;
	atomic_t task_count;
	atomic_t task_index;

	/* SRAM RCB */
	dma_addr_t sram_iova;
	u32 sram_size;
	u32 sram_used;
	int sram_enabled;
	struct page *rcb_page;

	/* Bitstream overflow flag */
	u32 bs_overflow;

	/* PM */
	u32 disable;
};

/* ---- Helper macros ---- */
static inline u32 rkvenc_read(struct rkvenc_dev *mpp, u32 reg)
{
	u32 val = readl(mpp->reg_base + reg);
	return val;
}

static inline u32 rkvenc_read_relaxed(struct rkvenc_dev *mpp, u32 reg)
{
	return readl_relaxed(mpp->reg_base + reg);
}

static inline void rkvenc_write(struct rkvenc_dev *mpp, u32 reg, u32 val)
{
	writel(val, mpp->reg_base + reg);
}

static inline void rkvenc_write_relaxed(struct rkvenc_dev *mpp, u32 reg, u32 val)
{
	writel_relaxed(val, mpp->reg_base + reg);
}

static inline void rkvenc_clk_safe_enable(struct clk *clk)
{
	if (clk)
		clk_prepare_enable(clk);
}

static inline void rkvenc_clk_safe_disable(struct clk *clk)
{
	if (clk)
		clk_disable_unprepare(clk);
}

static inline void rkvenc_safe_reset(struct reset_control *rst)
{
	if (rst)
		reset_control_assert(rst);
}

static inline void rkvenc_safe_unreset(struct reset_control *rst)
{
	if (rst)
		reset_control_deassert(rst);
}

/* ---- Extern declarations ---- */

/* rkvenc_service.c */
int rkvenc_service_probe(struct platform_device *pdev);
int rkvenc_service_remove(struct platform_device *pdev);
void rkvenc_task_worker_default(struct kthread_work *work);
extern const struct file_operations rkvenc_fops;
void rkvenc_session_put(struct rkvenc_session *session);

/* rkvenc_iommu.c */
struct rkvenc_iommu_info *rkvenc_iommu_probe(struct device *dev);
int rkvenc_iommu_remove(struct rkvenc_iommu_info *info);
int rkvenc_iommu_attach(struct rkvenc_iommu_info *info);
int rkvenc_iommu_detach(struct rkvenc_iommu_info *info);
int rkvenc_iommu_flush_tlb(struct rkvenc_iommu_info *info);
int rkvenc_iommu_dev_activate(struct rkvenc_iommu_info *info, struct rkvenc_dev *dev);
int rkvenc_iommu_dev_deactivate(struct rkvenc_iommu_info *info, struct rkvenc_dev *dev);

struct rkvenc_dma_session *rkvenc_dma_session_create(struct device *dev, u32 max_buffers);
int rkvenc_dma_session_destroy(struct rkvenc_dma_session *dma);
struct rkvenc_dma_buffer *rkvenc_dma_import_fd(struct rkvenc_iommu_info *iommu_info,
					       struct rkvenc_dma_session *dma,
					       int fd, int static_use);
struct rkvenc_dma_buffer *rkvenc_dma_find_buffer_fd(struct rkvenc_dma_session *dma, int fd);
int rkvenc_dma_release(struct rkvenc_dma_session *dma, struct rkvenc_dma_buffer *buffer);
int rkvenc_dma_release_fd(struct rkvenc_dma_session *dma, int fd);
void rkvenc_dma_buf_sync(struct rkvenc_dma_buffer *buffer, u32 offset, u32 length,
			 enum dma_data_direction dir, bool for_cpu);

static inline int rkvenc_iommu_down_read(struct rkvenc_iommu_info *info)
{
	if (info)
		down_read(info->rw_sem);
	return 0;
}

static inline int rkvenc_iommu_up_read(struct rkvenc_iommu_info *info)
{
	if (info)
		up_read(info->rw_sem);
	return 0;
}

static inline int rkvenc_iommu_down_write(struct rkvenc_iommu_info *info)
{
	if (info)
		down_write(info->rw_sem);
	return 0;
}

static inline int rkvenc_iommu_up_write(struct rkvenc_iommu_info *info)
{
	if (info)
		up_write(info->rw_sem);
	return 0;
}

/* rkvenc_task.c */
void rkvenc_free_task_callback(struct kref *ref);
int rkvenc_task_init(struct rkvenc_session *session, struct rkvenc_mpp_task *task);
int rkvenc_task_finish(struct rkvenc_session *session, struct rkvenc_mpp_task *task);
void rkvenc_task_complete(struct rkvenc_mpp_task *task, int result);
int rkvenc_task_finalize(struct rkvenc_session *session, struct rkvenc_mpp_task *task);
int rkvenc_translate_reg_address(struct rkvenc_session *session,
				  struct rkvenc_mpp_task *task, int fmt, u32 reg_class,
				  u32 *reg, struct reg_offset_info *off_inf);
int rkvenc_extract_reg_offset_info(struct reg_offset_info *off_inf,
				   struct mpp_request *req);
int rkvenc_query_reg_offset_info(struct reg_offset_info *off_inf, u32 index);
void rkvenc_task_timeout_work(struct work_struct *work_s);

/* rkvenc_hw.c */
int rkvenc_hw_probe(struct rkvenc_dev *enc, struct platform_device *pdev);
int rkvenc_hw_remove(struct rkvenc_dev *enc);
irqreturn_t rkvenc_hw_irq(int irq, void *param);
int rkvenc_hw_run(struct rkvenc_dev *mpp, struct rkvenc_mpp_task *mpp_task);
int rkvenc_hw_finish(struct rkvenc_dev *mpp, struct rkvenc_mpp_task *mpp_task);
int rkvenc_hw_reset(struct rkvenc_dev *enc);
void rkvenc_hw_clk_on(struct rkvenc_dev *enc);
void rkvenc_hw_clk_off(struct rkvenc_dev *enc);

/* HW info for VEPU580 */
extern struct rkvenc_hw_info rkvenc_v2_hw_info;
extern const struct rkvenc_trans_info trans_rkvenc_v2[];

#endif /* __RKVENC_HW_H__ */
