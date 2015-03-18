/*
 * Hisilicon Terminal SoCs drm driver
 *
 * Copyright (c) 2014-2015 Hisilicon Limited.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/of_platform.h>

#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "hisi_drm_ade.h"
#include "hisi_drm_dsi.h"
#include "hisi_drm_fb.h"
#ifdef CONFIG_DRM_HISI_FBDEV
#include "hisi_drm_fbdev.h"
#endif
#include "hisi_drm_drv.h"


static int hisi_drm_sub_drivers_init(struct drm_device *drm_dev)
{
	int ret;
	struct device_node *child_np;
	struct platform_device *pdev;
	struct device *parent_dev = &drm_dev->platformdev->dev;
	struct device_node *parent_np = parent_dev->of_node;

	/*
	 * First add devices and then initialize drivers
	 */

	 for_each_available_child_of_node(parent_np, child_np) {
		/* skip nodes without compatible property */
		if (!of_find_property(child_np, "compatible", NULL))
			continue;
		pdev = of_platform_device_create(child_np, NULL, parent_dev);
		if (!pdev) {
			ret = -ENODEV;
			DRM_ERROR("fail to create plat dev for node:%s\n",
				 of_node_full_name(child_np));
			return ret;
		}
		/* let drm_dev as platform data,
		so that sub drivers can access it */
		pdev->dev.platform_data = drm_dev;
	}
#if 0
	/* fbdev initialization should be put at last position */
#ifdef CONFIG_DRM_HISI_FBDEV
	ret = hisi_drm_fbdev_init(drm_dev);
	if (ret) {
		DRM_ERROR("failed to initialize fbdev\n");
		goto err_fbdev_init;
	}
#endif
#endif
	return 0;

#ifdef CONFIG_DRM_HISI_FBDEV
/* err_fbdev_init:*/
	hisi_drm_dsi_exit();
#endif

	return ret;
}

static void hisi_drm_sub_drivers_exit(struct drm_device *drm_dev)
{
	struct device *dev = &drm_dev->platformdev->dev;

	of_platform_depopulate(dev);
#ifdef CONFIG_DRM_HISI_FBDEV
	hisi_drm_fbdev_exit(drm_dev);
#endif
}

static int hisi_drm_unload(struct drm_device *drm_dev)
{
	struct hisi_drm_private *private = drm_dev->dev_private;

	drm_mode_config_cleanup(drm_dev);
	hisi_drm_sub_drivers_exit(drm_dev);
	kfree(private);
	drm_dev->dev_private = NULL;
	return 0;
}

static int hisi_drm_load(struct drm_device *drm_dev, unsigned long flags)
{
	int ret;
	struct hisi_drm_private *private;

	/* debug setting
	drm_debug = DRM_UT_DRIVER|DRM_UT_KMS|DRM_UT_CORE|DRM_UT_PRIME; */
	DRM_DEBUG_DRIVER("enter.\n");

	private = kzalloc(sizeof(struct hisi_drm_private), GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	drm_dev->dev_private = private;
	dev_set_drvdata(drm_dev->dev, drm_dev);

	/* drm_dev->mode_config initialization */
	drm_mode_config_init(drm_dev);
	hisi_drm_mode_config_init(drm_dev);

	/* sub drivers initialization */
	ret = hisi_drm_sub_drivers_init(drm_dev);
	if (ret) {
		DRM_ERROR("failed to initialize sub drivers\n");
		goto err_sub_drivers_init;
	}


	DRM_DEBUG_DRIVER("exit successfully.\n");
	return 0;

err_sub_drivers_init:
	drm_mode_config_cleanup(drm_dev);

	return ret;
}

static const struct file_operations hisi_drm_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= no_llseek,
	.mmap		= drm_gem_cma_mmap,
};

static struct dma_buf *hisi_drm_gem_prime_export(struct drm_device *dev,
						struct drm_gem_object *obj,
						int flags)
{
	/* we want to be able to write in mmapped buffer */
	flags |= O_RDWR;
	return drm_gem_prime_export(dev, obj, flags);
}

static struct drm_driver hisi_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME,
	.load			= hisi_drm_load,
	.unload                 = hisi_drm_unload,
	.fops			= &hisi_drm_fops,
	.set_busid		= drm_platform_set_busid,

	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,

	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= hisi_drm_gem_prime_export,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,

	.name			= "hisi",
	.desc			= "Hisilicon Terminal SoCs DRM Driver",
	.date			= "20141224",
	.major			= 1,
	.minor			= 0,
};

/* -----------------------------------------------------------------------------
 * Platform driver
 */

static int hisi_drm_probe(struct platform_device *pdev)
{
	return drm_platform_init(&hisi_drm_driver, pdev);
}

static int hisi_drm_remove(struct platform_device *pdev)
{
	drm_put_dev(platform_get_drvdata(pdev));

	return 0;
}

static struct of_device_id hisi_drm_of_match[] = {
	{ .compatible = "hisilicon,hi6220-drm" },      /* hi6220 SoC drm */
	{}
};
MODULE_DEVICE_TABLE(of, hisi_drm_of_match);

static struct platform_driver hisi_drm_platform_driver = {
	.probe		= hisi_drm_probe,
	.remove		= hisi_drm_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "hisi-drm",
		.of_match_table = hisi_drm_of_match,
	},
};

module_platform_driver(hisi_drm_platform_driver);

MODULE_AUTHOR("Xinliang Liu <z.liuxinliang@huawei.com>");
MODULE_DESCRIPTION("HISI DRM Driver");
MODULE_LICENSE("GPL");
