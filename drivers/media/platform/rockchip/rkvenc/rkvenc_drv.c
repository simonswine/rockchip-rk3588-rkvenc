// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Rockchip VEPU580 encoder driver - Platform driver
 * Ported from Rockchip BSP mpp_rkvenc2.c
 *
 * Copyright (C) 2023 Rockchip Electronics Co., Ltd.
 * Copyright (C) 2026 Ross Cawston
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "rkvenc_hw.h"

/* ---- PM ops ---- */
static int __maybe_unused rkvenc_runtime_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused rkvenc_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops rkvenc_pm_ops = {
	SET_RUNTIME_PM_OPS(rkvenc_runtime_suspend, rkvenc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

/* ---- CCU probe ---- */
static int rkvenc_ccu_probe(struct platform_device *pdev)
{
	struct rkvenc_ccu *ccu;
	struct device *dev = &pdev->dev;

	ccu = devm_kzalloc(dev, sizeof(*ccu), GFP_KERNEL);
	if (!ccu)
		return -ENOMEM;

	platform_set_drvdata(pdev, ccu);

	mutex_init(&ccu->lock);
	INIT_LIST_HEAD(&ccu->core_list);
	spin_lock_init(&ccu->lock_dchs);

	dev_info(dev, "rkvenc ccu probe success\n");
	return 0;
}

/* ---- Attach core to CCU ---- */
static int rkvenc_attach_ccu(struct device *dev, struct rkvenc_dev *enc)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct rkvenc_ccu *ccu;

	np = of_parse_phandle(dev->of_node, "rockchip,ccu", 0);
	if (!np || !of_device_is_available(np))
		return -ENODEV;

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -ENODEV;

	ccu = platform_get_drvdata(pdev);
	if (!ccu)
		return -EPROBE_DEFER;

	INIT_LIST_HEAD(&enc->core_link);
	mutex_lock(&ccu->lock);
	ccu->core_num++;
	list_add_tail(&enc->core_link, &ccu->core_list);
	mutex_unlock(&ccu->lock);

	/* First core becomes the main core */
	if (!ccu->main_core) {
		ccu->main_core = enc;
	} else {
		struct rkvenc_iommu_info *main_iommu =
			ccu->main_core ? ccu->main_core->iommu_info : NULL;
		struct rkvenc_iommu_info *sec_iommu = enc->iommu_info;
		struct iommu_domain *shared;
		int ret;

		shared = main_iommu ? main_iommu->domain : NULL;
		if (!shared || !sec_iommu || !sec_iommu->group) {
			dev_err(dev, "missing IOMMU info for shared domain\n");
			return -ENODEV;
		}

		/*
		 * Share the main core's IOMMU domain with the secondary core.
		 * This programs the secondary IOMMU hardware to use the same
		 * page tables as the main core (matching BSP mpp_rkvenc2.c).
		 *
		 * All DMA buffer imports go through the main core's device
		 * (via session->mpp which is always the main core), so mappings
		 * land in main_core->default_domain.  We do NOT need to modify
		 * iommu_get_dma_domain() because no DMA API calls are ever
		 * made through the secondary core's device.
		 */
		sec_iommu->domain = shared;
		sec_iommu->rw_sem = main_iommu->rw_sem;

		ret = iommu_attach_group(shared, sec_iommu->group);
		if (ret) {
			dev_err(dev, "attach shared IOMMU domain failed: %d\n", ret);
			sec_iommu->domain = NULL;
			sec_iommu->rw_sem = &sec_iommu->rw_sem_self;
			return ret;
		}
	}
	enc->ccu = ccu;

	dev_info(dev, "attach ccu as core %d%s\n", enc->core_id,
		 enc == ccu->main_core ? " [main]" :
		 (enc->queue && enc->core_id >= 0 &&
		  enc->core_id < MPP_MAX_CORE_NUM &&
		  enc->queue->cores[enc->core_id] == enc ?
		  " [secondary, active]" : " [secondary, inactive]"));
	return 0;
}

/* ---- Attach core to service ---- */
static int rkvenc_attach_service(struct rkvenc_dev *enc)
{
	struct device_node *np;
	struct platform_device *pdev;
	struct rkvenc_service *srv;
	struct rkvenc_taskqueue *queue;
	u32 taskqueue_node = 0;
	u32 resetgroup_node = 0;

	np = of_parse_phandle(enc->dev->of_node, "rockchip,srv", 0);
	if (!np || !of_device_is_available(np))
		return -ENODEV;

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return -ENODEV;

	srv = platform_get_drvdata(pdev);
	if (!srv)
		return -EPROBE_DEFER;

	enc->srv = srv;

	of_property_read_u32(enc->dev->of_node, "rockchip,taskqueue-node", &taskqueue_node);
	of_property_read_u32(enc->dev->of_node, "rockchip,resetgroup-node", &resetgroup_node);

	/* Attach to task queue */
	queue = srv->task_queues[MPP_DEVICE_RKVENC];
	if (!queue) {
		dev_err(enc->dev, "no task queue for RKVENC\n");
		return -ENODEV;
	}
	enc->queue = queue;

	/* Register core in the task queue */
	if (enc->core_id >= 0 && enc->core_id < MPP_MAX_CORE_NUM) {
		queue->cores[enc->core_id] = enc;
		queue->core_count++;
		if (enc->core_id > queue->core_id_max)
			queue->core_id_max = enc->core_id;
		set_bit(enc->core_id, &queue->core_idle);
	}

	/* Attach to reset group */
	if (resetgroup_node < srv->reset_group_cnt)
		enc->reset_group = srv->reset_groups[resetgroup_node];

	/* Init kthread work */
	kthread_init_work(&enc->work, rkvenc_task_worker_default);

	return 0;
}

/* ---- SRAM RCB allocation ---- */
static int rkvenc2_alloc_rcbbuf(struct platform_device *pdev, struct rkvenc_dev *enc)
{
	int ret;
	u32 vals[2];
	dma_addr_t iova;
	u32 sram_used, sram_size;
	struct device_node *sram_np;
	struct resource sram_res;
	resource_size_t sram_start, sram_end;
	struct iommu_domain *domain;
	struct device *dev = &pdev->dev;

	ret = device_property_read_u32_array(dev, "rockchip,rcb-iova", vals, 2);
	if (ret)
		return ret;

	iova = PAGE_ALIGN(vals[0]);
	sram_used = PAGE_ALIGN(vals[1]);
	if (!sram_used) {
		dev_err(dev, "sram rcb invalid\n");
		return -EINVAL;
	}

	sram_np = of_parse_phandle(dev->of_node, "rockchip,sram", 0);
	if (!sram_np) {
		dev_err(dev, "could not find phandle sram\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(sram_np, 0, &sram_res);
	of_node_put(sram_np);
	if (ret) {
		dev_err(dev, "find sram res error\n");
		return ret;
	}

	sram_start = round_up(sram_res.start, PAGE_SIZE);
	sram_end = round_down(sram_res.start + resource_size(&sram_res), PAGE_SIZE);
	if (sram_end <= sram_start) {
		dev_err(dev, "no available sram\n");
		return -ENOMEM;
	}
	sram_size = sram_end - sram_start;
	sram_size = sram_used < sram_size ? sram_used : sram_size;

	if (!enc->iommu_info || !enc->iommu_info->domain)
		return -ENODEV;

	domain = enc->iommu_info->domain;
	ret = iommu_map(domain, iova, sram_start, sram_size, IOMMU_READ | IOMMU_WRITE,
			GFP_KERNEL);
	if (ret) {
		dev_err(dev, "sram iommu_map error\n");
		return ret;
	}

	if (sram_size < sram_used) {
		struct page *page;
		size_t page_size = PAGE_ALIGN(sram_used - sram_size);

		page = alloc_pages(GFP_KERNEL | __GFP_ZERO, get_order(page_size));
		if (!page) {
			dev_err(dev, "unable to allocate pages\n");
			iommu_unmap(domain, iova, sram_size);
			return -ENOMEM;
		}
		ret = iommu_map(domain, iova + sram_size, page_to_phys(page),
				page_size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret) {
			dev_err(dev, "page iommu_map error\n");
			__free_pages(page, get_order(page_size));
			iommu_unmap(domain, iova, sram_size);
			return ret;
		}
		enc->rcb_page = page;
	}

	enc->sram_size = sram_size;
	enc->sram_used = sram_used;
	enc->sram_iova = iova;
	enc->sram_enabled = -1;
	dev_info(dev, "sram iova %pad size %u used %u\n",
		 &enc->sram_iova, enc->sram_size, enc->sram_used);

	return 0;
}

/* ---- Core probe ---- */
static int rkvenc_core_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct rkvenc_dev *enc;

	enc = devm_kzalloc(dev, sizeof(*enc), GFP_KERNEL);
	if (!enc)
		return -ENOMEM;

	platform_set_drvdata(pdev, enc);

	/* Get core ID from alias, falling back to DTS property */
	enc->core_id = of_alias_get_id(dev->of_node, "rkvenc");
	if (enc->core_id < 0) {
		u32 core_id = 0;

		of_property_read_u32(dev->of_node, "rockchip,core-id", &core_id);
		enc->core_id = core_id;
	}

	/* HW probe: clocks, resets, IOMMU, register space */
	ret = rkvenc_hw_probe(enc, pdev);
	if (ret)
		return ret;

	/* Attach to service */
	ret = rkvenc_attach_service(enc);
	if (ret) {
		dev_err_probe(dev, ret, "failed to attach service\n");
		goto err_hw;
	}

	/* Attach core to CCU */
	ret = rkvenc_attach_ccu(dev, enc);
	if (ret) {
		dev_err_probe(dev, ret, "attach ccu failed\n");
		goto err_hw;
	}

	/*
	 * Try SRAM allocation (optional, non-fatal).
	 * For secondary cores sharing a CCU domain, the main core has
	 * already mapped the SRAM into the shared domain.  Just inherit
	 * the SRAM info so rkvenc2_set_rcbbuf() works on both cores.
	 */
	if (enc->ccu && enc != enc->ccu->main_core &&
	    enc->ccu->main_core->sram_iova) {
		enc->sram_iova = enc->ccu->main_core->sram_iova;
		enc->sram_size = enc->ccu->main_core->sram_size;
		enc->sram_used = enc->ccu->main_core->sram_used;
		enc->sram_enabled = -1;
		dev_info(dev, "sram inherited from main core: iova %pad size %u used %u\n",
			 &enc->sram_iova, enc->sram_size, enc->sram_used);
	} else {
		rkvenc2_alloc_rcbbuf(pdev, enc);
	}

	/* Register IRQ */
	ret = devm_request_irq(dev, enc->irq, rkvenc_hw_irq,
			       IRQF_SHARED, dev_name(dev), enc);
	if (ret) {
		dev_err(dev, "register interrupt failed: %d\n", ret);
		goto err_hw;
	}

	/* If this is the main core, register with the service */
	if (enc->ccu && enc == enc->ccu->main_core) {
		enc->srv->sub_devices[MPP_DEVICE_RKVENC] = enc;
		set_bit(MPP_DEVICE_RKVENC, &enc->srv->hw_support);
	}

	dev_info(dev, "rkvenc core %d probe success (hw_id: %08x)\n",
		 enc->core_id, enc->hw_info->hw.hw_id);
	return 0;

err_hw:
	rkvenc_hw_remove(enc);
	return ret;
}

/* ---- Core remove ---- */
static int rkvenc_core_remove(struct platform_device *pdev)
{
	struct rkvenc_dev *enc = platform_get_drvdata(pdev);

	if (!enc)
		return 0;

	if (enc->ccu) {
		mutex_lock(&enc->ccu->lock);
		list_del_init(&enc->core_link);
		enc->ccu->core_num--;
		mutex_unlock(&enc->ccu->lock);
	}

	/* Free SRAM — only the core that mapped it (main or standalone) */
	if (enc->sram_iova && enc->iommu_info && enc->iommu_info->domain &&
	    (!enc->ccu || enc == enc->ccu->main_core)) {
		struct iommu_domain *domain = enc->iommu_info->domain;

		if (enc->rcb_page) {
			size_t page_size = PAGE_ALIGN(enc->sram_used - enc->sram_size);

			iommu_unmap(domain, enc->sram_iova + enc->sram_size, page_size);
			__free_pages(enc->rcb_page, get_order(page_size));
		}
		iommu_unmap(domain, enc->sram_iova, enc->sram_size);
	}

	rkvenc_hw_remove(enc);
	dev_info(&pdev->dev, "rkvenc core removed\n");
	return 0;
}

/* ---- Top-level probe dispatcher ---- */
static int rkvenc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	dev_info(dev, "probing start\n");

	if (strstr(np->name, "ccu"))
		return rkvenc_ccu_probe(pdev);
	else if (strstr(np->name, "core"))
		return rkvenc_core_probe(pdev);
	else if (of_device_is_compatible(np, "rockchip,mpp-service"))
		return rkvenc_service_probe(pdev);

	dev_err(dev, "unknown node type: %s\n", np->name);
	return -ENODEV;
}

static void rkvenc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	if (strstr(np->name, "ccu")) {
		dev_info(dev, "remove ccu\n");
	} else if (strstr(np->name, "core")) {
		rkvenc_core_remove(pdev);
	} else if (of_device_is_compatible(np, "rockchip,mpp-service")) {
		rkvenc_service_remove(pdev);
	}
}

static void rkvenc_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rkvenc_dev *enc;
	int ret, val;

	if (strstr(np->name, "ccu"))
		return;
	if (of_device_is_compatible(np, "rockchip,mpp-service"))
		return;

	enc = platform_get_drvdata(pdev);
	if (!enc || !enc->srv)
		return;

	dev_info(dev, "shutdown device\n");
	atomic_inc(&enc->srv->shutdown_request);

	ret = readx_poll_timeout(atomic_read, &enc->task_count,
				 val, val == 0, 20000, 200000);
	if (ret == -ETIMEDOUT)
		dev_err(dev, "wait total %d running time out\n",
			atomic_read(&enc->task_count));
	else
		dev_info(dev, "shutdown success\n");
}

/* ---- OF match table ---- */
static const struct of_device_id rkvenc_dt_match[] = {
	{ .compatible = "rockchip,mpp-service" },
	{ .compatible = "rockchip,rkv-encoder-v2-ccu" },
	{ .compatible = "rockchip,rkv-encoder-v2-core" },
	{},
};
MODULE_DEVICE_TABLE(of, rkvenc_dt_match);

static struct platform_driver rkvenc_driver = {
	.probe = rkvenc_probe,
	.remove = rkvenc_remove,
	.shutdown = rkvenc_shutdown,
	.driver = {
		.name = MPP_DRIVER_NAME,
		.of_match_table = rkvenc_dt_match,
		.pm = &rkvenc_pm_ops,
	},
};

module_platform_driver(rkvenc_driver);

MODULE_DESCRIPTION("Rockchip VEPU580 (RKVENC v2) H.265/H.264/JPEG encoder driver");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Rockchip Electronics Co., Ltd.");
MODULE_IMPORT_NS("DMA_BUF");
