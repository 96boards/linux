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

#include <drm/drmP.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_crtc_helper.h>

#include "hisi_drm_fb.h"

struct hisi_drm_fb *to_hisi_drm_fb(struct drm_framebuffer *fb)
{
	return container_of(fb, struct hisi_drm_fb, fb);
}

void hisi_drm_fb_destroy(struct drm_framebuffer *fb)
{
	struct hisi_drm_fb *hisi_fb = to_hisi_drm_fb(fb);
	int i;

	for (i = 0; i < 4; i++) {
		if (hisi_fb->obj[i])
			drm_gem_object_unreference_unlocked( \
				&hisi_fb->obj[i]->base);
	}

	drm_framebuffer_cleanup(fb);
	kfree(hisi_fb);
}

static int hisi_drm_fb_create_handle(struct drm_framebuffer *fb,
	struct drm_file *file_priv, unsigned int *handle)
{
	struct hisi_drm_fb *hisi_fb = to_hisi_drm_fb(fb);

	return drm_gem_handle_create(file_priv,
			&hisi_fb->obj[0]->base, handle);
}

static int hisi_drm_fb_dirty(struct drm_framebuffer *fb,
				struct drm_file *file_priv, unsigned flags,
				unsigned color, struct drm_clip_rect *clips,
				unsigned num_clips)
{
	/* TODO */
	return 0;
}

static struct drm_framebuffer_funcs hisi_drm_fb_funcs = {
	.destroy	= hisi_drm_fb_destroy,
	.create_handle	= hisi_drm_fb_create_handle,
	.dirty		= hisi_drm_fb_dirty,
};

struct hisi_drm_fb *hisi_drm_fb_alloc(struct drm_device *dev,
	struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_cma_object **obj,
	unsigned int num_planes, bool is_fbdev_fb)
{
	struct hisi_drm_fb *hisi_fb;
	int ret;
	int i;

	hisi_fb = kzalloc(sizeof(*hisi_fb), GFP_KERNEL);
	if (!hisi_fb)
		return ERR_PTR(-ENOMEM);

	hisi_fb->is_fbdev_fb = is_fbdev_fb;
	drm_helper_mode_fill_fb_struct(&hisi_fb->fb, mode_cmd);

	for (i = 0; i < num_planes; i++)
		hisi_fb->obj[i] = obj[i];

	ret = drm_framebuffer_init(dev, &hisi_fb->fb, &hisi_drm_fb_funcs);
	if (ret) {
		DRM_ERROR("Failed to initialize framebuffer: %d\n", ret);
		kfree(hisi_fb);
		return ERR_PTR(ret);
	}

	return hisi_fb;
}

/**
 * hisi_drm_fb_create() - (struct drm_mode_config_funcs *)->fb_create callback function
 *
 * If your hardware has special alignment or pitch requirements these should be
 * checked before calling this function.
 */
static struct drm_framebuffer *hisi_drm_fb_create(struct drm_device *dev,
	struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct hisi_drm_fb *hisi_fb;
	struct drm_gem_cma_object *objs[4];
	struct drm_gem_object *obj;
	unsigned int hsub;
	unsigned int vsub;
	int ret;
	int i;


	/* TODO: Need to use ion heaps to create frame buffer?? */

	hsub = drm_format_horz_chroma_subsampling(mode_cmd->pixel_format);
	vsub = drm_format_vert_chroma_subsampling(mode_cmd->pixel_format);

	for (i = 0; i < drm_format_num_planes(mode_cmd->pixel_format); i++) {
		unsigned int width = mode_cmd->width / (i ? hsub : 1);
		unsigned int height = mode_cmd->height / (i ? vsub : 1);
		unsigned int min_size;

		obj = drm_gem_object_lookup(dev, file_priv,
						mode_cmd->handles[i]);
		if (!obj) {
			DRM_ERROR("Failed to lookup GEM object\n");
			ret = -ENXIO;
			goto err_gem_object_unreference;
		}

		min_size = (height - 1) * mode_cmd->pitches[i]
			 + width*drm_format_plane_cpp(mode_cmd->pixel_format, i)
			 + mode_cmd->offsets[i];

		if (obj->size < min_size) {
			drm_gem_object_unreference_unlocked(obj);
			ret = -EINVAL;
			goto err_gem_object_unreference;
		}
		objs[i] = to_drm_gem_cma_obj(obj);
	}

	hisi_fb = hisi_drm_fb_alloc(dev, mode_cmd, objs, i, false);
	if (IS_ERR(hisi_fb)) {
		ret = PTR_ERR(hisi_fb);
		goto err_gem_object_unreference;
	}

	return &hisi_fb->fb;

err_gem_object_unreference:
	for (i--; i >= 0; i--)
		drm_gem_object_unreference_unlocked(&objs[i]->base);
	return ERR_PTR(ret);
}

/**
 * hisi_drm_fb_get_gem_obj() - Get CMA GEM object for framebuffer
 * @fb: The framebuffer
 * @plane: Which plane
 *
 * Return the CMA GEM object for given framebuffer.
 *
 * This function will usually be called from the CRTC callback functions.
 */
struct drm_gem_cma_object *hisi_drm_fb_get_gem_obj(struct drm_framebuffer *fb,
	unsigned int plane)
{
	struct hisi_drm_fb *hisi_fb = to_hisi_drm_fb(fb);

	if (plane >= 4)
		return NULL;

	return hisi_fb->obj[plane];
}

static const struct drm_mode_config_funcs hisi_drm_mode_config_funcs = {
	.fb_create = hisi_drm_fb_create,
};

void hisi_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;

	dev->mode_config.funcs = &hisi_drm_mode_config_funcs;
}
