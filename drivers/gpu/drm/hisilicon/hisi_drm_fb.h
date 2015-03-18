/*
 * Hisilicon Terminal SoCs drm fbdev driver
 *
 * Copyright (c) 2014-2015 Hisilicon Limited.
 * Author: z.liuxinliang@huawei.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __HISI_DRM_FB_H__
#define __HISI_DRM_FB_H__

#define HISI_NUM_FRAMEBUFFERS  2

struct hisi_drm_fb {
	struct drm_framebuffer		fb;
	struct drm_gem_cma_object	*obj[4];
	bool is_fbdev_fb;
};

extern void hisi_drm_fb_destroy(struct drm_framebuffer *fb);
extern struct hisi_drm_fb *hisi_drm_fb_alloc(struct drm_device *dev,
	struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_cma_object **obj,
	unsigned int num_planes, bool is_fbdev_fb);
extern struct drm_gem_cma_object *hisi_drm_fb_get_gem_obj(
	struct drm_framebuffer *fb, unsigned int plane);
extern void hisi_drm_mode_config_init(struct drm_device *drm_dev);
extern struct hisi_drm_fb *to_hisi_drm_fb(struct drm_framebuffer *fb);

#endif /* __HISI_DRM_FB_H__ */
