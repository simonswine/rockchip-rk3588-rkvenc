// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip VEPU580 encoder driver - IOMMU and DMA buffer helpers
 * Ported from Rockchip BSP mpp_iommu.c
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/kref.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include "rkvenc_hw.h"

/* ---- DMA buffer find ---- */
struct rkvenc_dma_buffer *
rkvenc_dma_find_buffer_fd(struct rkvenc_dma_session *dma, int fd)
{
	struct dma_buf *dmabuf;
	struct rkvenc_dma_buffer *out = NULL;
	struct rkvenc_dma_buffer *buffer = NULL, *n;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return NULL;

	mutex_lock(&dma->list_mutex);
	list_for_each_entry_safe(buffer, n, &dma->used_list, link) {
		if (buffer->dmabuf == dmabuf) {
			out = buffer;
			list_move_tail(&buffer->link, &buffer->dma->used_list);
			break;
		}
	}
	if (!out) {
		list_for_each_entry_safe(buffer, n, &dma->static_list, link) {
			if (buffer->dmabuf == dmabuf) {
				out = buffer;
				list_move_tail(&buffer->link, &buffer->dma->static_list);
				break;
			}
		}
	}
	mutex_unlock(&dma->list_mutex);
	dma_buf_put(dmabuf);

	return out;
}

/* ---- DMA buffer release ---- */
static void rkvenc_dma_release_buffer(struct kref *ref)
{
	struct rkvenc_dma_buffer *buffer =
		container_of(ref, struct rkvenc_dma_buffer, ref);

	buffer->dma->buffer_count--;
	list_move_tail(&buffer->link, &buffer->dma->unused_list);

	dma_buf_unmap_attachment(buffer->attach, buffer->sgt, buffer->dir);
	dma_buf_detach(buffer->dmabuf, buffer->attach);
	dma_buf_put(buffer->dmabuf);
	buffer->dma = NULL;
	buffer->dmabuf = NULL;
	buffer->attach = NULL;
	buffer->sgt = NULL;
	buffer->iova = 0;
	buffer->size = 0;
	buffer->vaddr = NULL;
}

static int rkvenc_dma_remove_extra_buffer(struct rkvenc_dma_session *dma)
{
	struct rkvenc_dma_buffer *n;
	struct rkvenc_dma_buffer *removable = NULL, *buffer = NULL;

	if (dma->buffer_count > dma->max_buffers) {
		mutex_lock(&dma->list_mutex);
		list_for_each_entry_safe(buffer, n, &dma->used_list, link) {
			if (kref_read(&buffer->ref) == 1) {
				removable = buffer;
				break;
			}
		}
		if (removable)
			kref_put(&removable->ref, rkvenc_dma_release_buffer);
		mutex_unlock(&dma->list_mutex);
	}

	return 0;
}

int rkvenc_dma_release(struct rkvenc_dma_session *dma,
		       struct rkvenc_dma_buffer *buffer)
{
	mutex_lock(&dma->list_mutex);
	kref_put(&buffer->ref, rkvenc_dma_release_buffer);
	mutex_unlock(&dma->list_mutex);

	return 0;
}

int rkvenc_dma_release_fd(struct rkvenc_dma_session *dma, int fd)
{
	struct rkvenc_dma_buffer *buffer;

	buffer = rkvenc_dma_find_buffer_fd(dma, fd);
	if (IS_ERR_OR_NULL(buffer)) {
		dev_err(dma->dev, "can not find %d buffer in list\n", fd);
		return -EINVAL;
	}

	mutex_lock(&dma->list_mutex);
	kref_put(&buffer->ref, rkvenc_dma_release_buffer);
	mutex_unlock(&dma->list_mutex);

	return 0;
}

/* ---- DMA buffer import ---- */
struct rkvenc_dma_buffer *
rkvenc_dma_import_fd(struct rkvenc_iommu_info *iommu_info,
		     struct rkvenc_dma_session *dma,
		     int fd, int static_use)
{
	int ret = 0;
	struct sg_table *sgt;
	struct dma_buf *dmabuf;
	struct rkvenc_dma_buffer *buffer;
	struct dma_buf_attachment *attach;

	if (!dma) {
		rkvenc_err("dma session is null\n");
		return ERR_PTR(-EINVAL);
	}

	rkvenc_dma_remove_extra_buffer(dma);

	/* Check whether in dma session */
	buffer = rkvenc_dma_find_buffer_fd(dma, fd);
	if (!IS_ERR_OR_NULL(buffer)) {
		if (kref_get_unless_zero(&buffer->ref))
			return buffer;
	}

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		rkvenc_err("dma_buf_get fd %d failed(%d)\n", fd, ret);
		return ERR_PTR(ret);
	}

	mutex_lock(&dma->list_mutex);
	buffer = list_first_entry_or_null(&dma->unused_list,
					  struct rkvenc_dma_buffer, link);
	if (!buffer) {
		ret = -ENOMEM;
		mutex_unlock(&dma->list_mutex);
		goto fail;
	}
	list_del_init(&buffer->link);
	mutex_unlock(&dma->list_mutex);

	buffer->dmabuf = dmabuf;
	buffer->dir = DMA_BIDIRECTIONAL;

	attach = dma_buf_attach(buffer->dmabuf, dma->dev);
	if (IS_ERR(attach)) {
		ret = PTR_ERR(attach);
		rkvenc_err("dma_buf_attach fd %d failed(%d)\n", fd, ret);
		goto fail_attach;
	}

	sgt = dma_buf_map_attachment(attach, buffer->dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		rkvenc_err("dma_buf_map_attachment fd %d failed(%d)\n", fd, ret);
		goto fail_map;
	}
	buffer->iova = sg_dma_address(sgt->sgl);
	buffer->size = sg_dma_len(sgt->sgl);
	buffer->attach = attach;
	buffer->sgt = sgt;
	buffer->dma = dma;

	kref_init(&buffer->ref);

	if (!static_use)
		kref_get(&buffer->ref);

	mutex_lock(&dma->list_mutex);
	dma->buffer_count++;
	if (static_use)
		list_add_tail(&buffer->link, &dma->static_list);
	else
		list_add_tail(&buffer->link, &dma->used_list);
	mutex_unlock(&dma->list_mutex);

	return buffer;

fail_map:
	dma_buf_detach(buffer->dmabuf, attach);
fail_attach:
	mutex_lock(&dma->list_mutex);
	list_add_tail(&buffer->link, &dma->unused_list);
	mutex_unlock(&dma->list_mutex);
fail:
	dma_buf_put(dmabuf);
	return ERR_PTR(ret);
}

/* ---- DMA buffer sync ---- */
void rkvenc_dma_buf_sync(struct rkvenc_dma_buffer *buffer, u32 offset, u32 length,
			 enum dma_data_direction dir, bool for_cpu)
{
	struct device *dev = buffer->dma->dev;
	struct sg_table *sgt = buffer->sgt;
	struct scatterlist *sg = sgt->sgl;
	dma_addr_t sg_dma_addr = sg_dma_address(sg);
	unsigned int len = 0;
	int i;

	for_each_sgtable_sg(sgt, sg, i) {
		unsigned int sg_offset, sg_left, size = 0;

		len += sg->length;
		if (len <= offset) {
			sg_dma_addr += sg->length;
			continue;
		}

		sg_left = len - offset;
		sg_offset = sg->length - sg_left;

		size = (length < sg_left) ? length : sg_left;

		if (for_cpu)
			dma_sync_single_range_for_cpu(dev, sg_dma_addr,
						      sg_offset, size, dir);
		else
			dma_sync_single_range_for_device(dev, sg_dma_addr,
							 sg_offset, size, dir);

		offset += size;
		length -= size;
		sg_dma_addr += sg->length;

		if (length == 0)
			break;
	}
}

/* ---- DMA session management ---- */
int rkvenc_dma_session_destroy(struct rkvenc_dma_session *dma)
{
	struct rkvenc_dma_buffer *n, *buffer = NULL;

	if (!dma)
		return -EINVAL;

	mutex_lock(&dma->list_mutex);
	list_for_each_entry_safe(buffer, n, &dma->used_list, link)
		kref_put(&buffer->ref, rkvenc_dma_release_buffer);
	list_for_each_entry_safe(buffer, n, &dma->static_list, link)
		kref_put(&buffer->ref, rkvenc_dma_release_buffer);
	mutex_unlock(&dma->list_mutex);

	kfree(dma);
	return 0;
}

struct rkvenc_dma_session *
rkvenc_dma_session_create(struct device *dev, u32 max_buffers)
{
	int i;
	struct rkvenc_dma_session *dma;
	struct rkvenc_dma_buffer *buffer;

	dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return NULL;

	mutex_init(&dma->list_mutex);
	INIT_LIST_HEAD(&dma->unused_list);
	INIT_LIST_HEAD(&dma->used_list);
	INIT_LIST_HEAD(&dma->static_list);

	if (max_buffers > MPP_SESSION_MAX_BUFFERS)
		dma->max_buffers = MPP_SESSION_MAX_BUFFERS;
	else
		dma->max_buffers = max_buffers;

	for (i = 0; i < ARRAY_SIZE(dma->dma_bufs); i++) {
		buffer = &dma->dma_bufs[i];
		buffer->dma = dma;
		INIT_LIST_HEAD(&buffer->link);
		list_add_tail(&buffer->link, &dma->unused_list);
	}
	dma->dev = dev;

	return dma;
}

/* ---- IOMMU helpers ---- */
int rkvenc_iommu_detach(struct rkvenc_iommu_info *info)
{
	if (!info)
		return 0;

	iommu_detach_group(info->domain, info->group);
	return 0;
}

int rkvenc_iommu_attach(struct rkvenc_iommu_info *info)
{
	if (!info)
		return 0;

	if (info->domain == iommu_get_domain_for_dev(info->dev))
		return 0;

	return iommu_attach_group(info->domain, info->group);
}

int rkvenc_iommu_flush_tlb(struct rkvenc_iommu_info *info)
{
	if (!info)
		return 0;

	if (info->domain && info->domain->ops)
		iommu_flush_iotlb_all(info->domain);

	return 0;
}

static int rkvenc_iommu_fault_handler(struct iommu_domain *iommu,
				      struct device *iommu_dev,
				      unsigned long iova,
				      int status, void *arg)
{
	dev_err(iommu_dev, "IOMMU fault addr 0x%08lx status %x\n",
		iova, status);

	return 0;
}

int rkvenc_iommu_dev_activate(struct rkvenc_iommu_info *info, struct rkvenc_dev *dev)
{
	unsigned long flags;

	if (!info)
		return 0;

	spin_lock_irqsave(&info->dev_lock, flags);

	if (info->dev_active || !dev) {
		dev_err(info->dev, "can not activate\n");
		spin_unlock_irqrestore(&info->dev_lock, flags);
		return -EINVAL;
	}

	info->dev_active = dev;
	if (info->domain && info->domain->cookie_type == IOMMU_COOKIE_NONE)
		iommu_set_fault_handler(info->domain, rkvenc_iommu_fault_handler, dev);

	spin_unlock_irqrestore(&info->dev_lock, flags);

	return 0;
}

int rkvenc_iommu_dev_deactivate(struct rkvenc_iommu_info *info, struct rkvenc_dev *dev)
{
	unsigned long flags;

	if (!info)
		return 0;

	spin_lock_irqsave(&info->dev_lock, flags);
	info->dev_active = NULL;
	spin_unlock_irqrestore(&info->dev_lock, flags);

	return 0;
}

struct rkvenc_iommu_info *rkvenc_iommu_probe(struct device *dev)
{
	int ret = 0;
	struct device_node *np;
	struct platform_device *pdev;
	struct rkvenc_iommu_info *info;
	struct iommu_domain *domain;
	struct iommu_group *group;

	np = of_parse_phandle(dev->of_node, "iommus", 0);
	if (!np || !of_device_is_available(np)) {
		rkvenc_err("failed to get IOMMU device node\n");
		return ERR_PTR(-ENODEV);
	}

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev) {
		rkvenc_err("failed to get IOMMU platform device\n");
		return ERR_PTR(-ENODEV);
	}

	group = iommu_group_get(dev);
	if (!group) {
		ret = -EINVAL;
		goto err_put_pdev;
	}

	domain = iommu_get_domain_for_dev(dev);
	if (!domain) {
		ret = -EINVAL;
		goto err_put_group;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		ret = -ENOMEM;
		goto err_put_group;
	}

	init_rwsem(&info->rw_sem_self);
	info->rw_sem = &info->rw_sem_self;
	spin_lock_init(&info->dev_lock);
	info->dev = dev;
	info->pdev = pdev;
	info->group = group;
	info->domain = domain;
	info->dev_active = NULL;
	info->irq = platform_get_irq(pdev, 0);
	info->got_irq = (info->irq < 0) ? false : true;

	return info;

err_put_group:
	if (group)
		iommu_group_put(group);
err_put_pdev:
	if (pdev)
		platform_device_put(pdev);

	return ERR_PTR(ret);
}

int rkvenc_iommu_remove(struct rkvenc_iommu_info *info)
{
	if (!info)
		return 0;

	iommu_group_put(info->group);
	platform_device_put(info->pdev);

	return 0;
}
