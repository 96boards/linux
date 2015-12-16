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

#include <linux/bitops.h>
#include <linux/clk.h>
#include <video/display_timing.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "hisi_drm_fb.h"
#include "hisi_ade_reg.h"
#include "hisi_ldi_reg.h"
#include "hisi_drm_ade.h"

#define FORCE_PIXEL_CLOCK_SAME_OR_HIGHER 0

#define SC_MEDIA_RSTDIS		(0x530)
#define SC_MEDIA_RSTEN		(0x52C)
#define NOC_ADE0_QOSGENERATOR_MODE       0x010C
#define NOC_ADE0_QOSGENERATOR_EXTCONTROL 0x0118
#define NOC_ADE1_QOSGENERATOR_MODE       0x020C
#define NOC_ADE1_QOSGENERATOR_EXTCONTROL 0x0218

enum {
	LDI_TEST = 0,
	LDI_WORK
};

enum {
	LDI_ISR_FRAME_END_INT           = 0x2,
	LDI_ISR_UNDER_FLOW_INT          = 0x4
};

enum {
	ADE_ISR1_RES_SWITCH_CMPL         = 0x80000000
};

enum {
	LDI_DISP_MODE_NOT_3D_FBF = 0,
	LDI_DISP_MODE_3D_FBF
};

enum {
	ADE_RGB = 0,
	ADE_BGR
};
enum {
	ADE_DISABLE = 0,
	ADE_ENABLE
};

enum {
	ADE_OUT_RGB_565 = 0,
	ADE_OUT_RGB_666,
	ADE_OUT_RGB_888
};

enum ADE_FORMAT {
	ADE_RGB_565 = 0,
	ADE_BGR_565,
	ADE_XRGB_8888,
	ADE_XBGR_8888,
	ADE_ARGB_8888,
	ADE_ABGR_8888,
	ADE_RGBA_8888,
	ADE_BGRA_8888,
	ADE_RGB_888,
	ADE_BGR_888 = 9,
	ADE_YUYV_I = 16,
	ADE_YVYU_I,
	ADE_UYVY_I,
	ADE_VYUY_I,
	ADE_YUV444,
	ADE_NV12,
	ADE_NV21,
	ADE_FORMAT_NOT_SUPPORT = 800
};

enum {
	TOP_DISP_CH_SRC_RDMA = 2
};

enum {
	ADE_ISR_DMA_ERROR               = 0x2000000
};


struct hisi_drm_ade_crtc {
	bool enable;
	u8 __iomem  *ade_base;
	u8 __iomem  *media_base;
	void __iomem  *media_noc_base;
	u32 ade_core_rate;
	u32 media_noc_rate;
	u32 x , y;

	struct drm_device  *drm_dev;
	struct drm_crtc    crtc;
	struct drm_display_mode *dmode;

	struct clk *ade_core_clk;
	struct clk *media_noc_clk;
	struct clk *ade_pix_clk;
	bool power_on;

};

static int hisi_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					struct drm_framebuffer *old_fb);
static void ldi_init(struct hisi_drm_ade_crtc *crtc_ade);

void ade_set_medianoc_qos(struct hisi_drm_ade_crtc *crtc_ade)
{
	void __iomem *base = crtc_ade->media_noc_base;
	void __iomem *reg;
	u32 val;

	reg = base + NOC_ADE0_QOSGENERATOR_MODE;
	val = (readl(reg) & 0xfffffffc) | 0x2;
	writel(val, reg);

	reg = base + NOC_ADE0_QOSGENERATOR_EXTCONTROL;
	val = readl(reg) | 0x1;
	writel(val, reg);

	reg = base + NOC_ADE1_QOSGENERATOR_MODE;
	val = (readl(reg) & 0xfffffffc) | 0x2;
	writel(val, reg);

	reg = base + NOC_ADE1_QOSGENERATOR_EXTCONTROL;
	val = readl(reg) | 0x1;
	writel(val, reg);
}

static void ade_init(struct hisi_drm_ade_crtc *crtc_ade)
{
	u8 __iomem *ade_base = crtc_ade->ade_base;
	u32    cpu0_mask;
	u32    cpu1_mask;

	cpu0_mask = ADE_ISR_DMA_ERROR;
	cpu1_mask = ADE_ISR1_RES_SWITCH_CMPL;

	writel(cpu0_mask, ade_base + INTR_MASK_CPU_0_REG);
	writel(cpu1_mask, ade_base + INTR_MASK_CPU_1_REG);
	set_TOP_CTL_frm_end_start(ade_base, 2);

	/* disable wdma2 and wdma3 frame discard */
	writel(0x0, ade_base + ADE_FRM_DISGARD_CTRL_REG);

	writel(0, ade_base + ADE_SOFT_RST_SEL0_REG);
	writel(0, ade_base + ADE_SOFT_RST_SEL1_REG);
	writel(0, ade_base + ADE_RELOAD_DIS0_REG);
	writel(0, ade_base + ADE_RELOAD_DIS1_REG);

	/* enable clk gate */
	set_TOP_CTL_clk_gate_en(ade_base, 1);

	/* init ovly ctrl, if not,when the first
	* frame is hybrid, will happen 32112222
	*/
	writel(0, ade_base + ADE_OVLY_CTL_REG);

	/* TODO:init scl coeff */
}

static int ade_power_up(struct hisi_drm_ade_crtc *crtc_ade)
{
	u8 __iomem *media_base = crtc_ade->media_base;
	int ret;

	ret = clk_set_rate(crtc_ade->ade_core_clk, crtc_ade->ade_core_rate);
	if (ret) {
		DRM_ERROR("clk_set_rate ade_core_rate error\n");
		return ret;
	}
	ret = clk_set_rate(crtc_ade->media_noc_clk, crtc_ade->media_noc_rate);
	if (ret) {
		DRM_ERROR("media_noc_clk media_noc_rate error\n");
		return ret;
	}
	ret = clk_prepare_enable(crtc_ade->media_noc_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable media_noc_clk\n");
		return ret;
	}

	writel(0x20, media_base + SC_MEDIA_RSTDIS);

	ret = clk_prepare_enable(crtc_ade->ade_core_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable ade_core_clk\n");
		return ret;
	}
	crtc_ade->power_on = true;
	return 0;
}

static int ade_power_down(struct hisi_drm_ade_crtc *crtc_ade)
{
	u8 __iomem *media_base = crtc_ade->media_base;

	clk_disable_unprepare(crtc_ade->ade_core_clk);
	writel(0x20, media_base + SC_MEDIA_RSTEN);
	clk_disable_unprepare(crtc_ade->media_noc_clk);
	crtc_ade->power_on = false;
	return 0;
}

static int hisi_drm_crtc_ade_enable(struct hisi_drm_ade_crtc *crtc_ade)
{
	int ret;

	if (!crtc_ade->power_on) {
		ret = ade_power_up(crtc_ade);
		if (ret) {
		DRM_ERROR("failed to initialize ade clk\n");
		return ret;
		}
	}

	ade_set_medianoc_qos(crtc_ade);
	ade_init(crtc_ade);
	ldi_init(crtc_ade);
	if (crtc_ade->crtc.primary->fb)
		hisi_drm_crtc_mode_set_base(&crtc_ade->crtc, 0, 0, NULL);
	return 0;
}

static int hisi_drm_crtc_ade_disable(struct hisi_drm_ade_crtc *crtc_ade)
{
	int ret;
	u8 __iomem *ade_base = crtc_ade->ade_base;

	set_LDI_CTRL_ldi_en(ade_base, ADE_DISABLE);
	/* dsi pixel off */
	set_reg(ade_base + LDI_HDMI_DSI_GT_REG, 0x1, 1, 0);
	ret = ade_power_down(crtc_ade);
	if (ret) {
		DRM_ERROR("failed to initialize ade clk\n");
		return ret;
	}

	return 0;
}

static void ldi_init(struct hisi_drm_ade_crtc *crtc_ade)
{
	struct drm_display_mode *mode = crtc_ade->dmode;
	void __iomem *ade_base = crtc_ade->ade_base;
	u32 hfp, hbp, hsw, vfp, vbp, vsw;
	u32 plr_flags;
	u32 ldi_mask;

	plr_flags = (mode->flags & DRM_MODE_FLAG_NVSYNC)
			? HISI_LDI_FLAG_NVSYNC : 0;
	plr_flags |= (mode->flags & DRM_MODE_FLAG_NHSYNC)
			? HISI_LDI_FLAG_NHSYNC : 0;
	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	hsw = mode->hsync_end - mode->hsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;
	vsw = mode->vsync_end - mode->vsync_start;
	if (vsw > 15) {
		pr_err("%s: vsw exceeded 15\n", __func__);
		vsw = 15;
	}

	writel((hbp << 20) | (hfp << 0), ade_base + LDI_HRZ_CTRL0_REG);
	/* p3-73 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(hsw - 1, ade_base + LDI_HRZ_CTRL1_REG);
	writel((vbp << 20) | (vfp << 0), ade_base + LDI_VRT_CTRL0_REG);
	/* p3-74 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(vsw - 1, ade_base + LDI_VRT_CTRL1_REG);

	/* p3-75 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(((mode->vdisplay - 1) << 20) | ((mode->hdisplay - 1) << 0),
	       ade_base + LDI_DSP_SIZE_REG);
	writel(plr_flags, ade_base + LDI_PLR_CTRL_REG);

	/*
	 * other parameters setting
	 */
	writel(BIT(0), ade_base + LDI_WORK_MODE_REG);
	ldi_mask = LDI_ISR_FRAME_END_INT | LDI_ISR_UNDER_FLOW_INT;
	writel(ldi_mask, ade_base + LDI_INT_EN_REG);
	writel((0x3c << 6) | (ADE_OUT_RGB_888 << 3) | BIT(2) | BIT(0),
	       ade_base + LDI_CTRL_REG);

	writel(0xFFFFFFFF, ade_base + LDI_INT_CLR_REG);
	set_reg(ade_base + LDI_DE_SPACE_LOW_REG, 0x1, 1, 1);
	/* dsi pixel on */
	set_reg(ade_base + LDI_HDMI_DSI_GT_REG, 0x0, 1, 0);
}

/* -----------------------------------------------------------------------------
 * CRTC
 */

#define to_hisi_crtc(c)  container_of(c, struct hisi_drm_ade_crtc, crtc)

static void hisi_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct hisi_drm_ade_crtc *crtc_ade = to_hisi_crtc(crtc);
	bool enable = (mode == DRM_MODE_DPMS_ON);

	DRM_DEBUG_DRIVER("crtc_dpms  enter successfully.\n");
	if (crtc_ade->enable == enable)
		return;

	if (enable)
		hisi_drm_crtc_ade_enable(crtc_ade);
	else
		hisi_drm_crtc_ade_disable(crtc_ade);

	crtc_ade->enable = enable;
	DRM_DEBUG_DRIVER("crtc_dpms exit successfully.\n");
}

static bool hisi_drm_crtc_mode_fixup(struct drm_crtc *crtc,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adj_mode)
{
	struct hisi_drm_ade_crtc *crtc_ade = to_hisi_crtc(crtc);
	u32 clock_kHz = mode->clock;
	int ret;

	DRM_DEBUG_DRIVER("mode_fixup  enter successfully.\n");

	if (!crtc_ade->power_on)
		if (ade_power_up(crtc_ade))
			DRM_ERROR("%s: failed to power up ade\n", __func__);

	do {
		ret = clk_set_rate(crtc_ade->ade_pix_clk, clock_kHz * 1000);
		if (ret) {
			DRM_ERROR("set ade_pixel_clk_rate fail\n");
			return false;
		}
		adj_mode->clock = clk_get_rate(crtc_ade->ade_pix_clk) / 1000;
#if FORCE_PIXEL_CLOCK_SAME_OR_HIGHER
		if (adj_mode->clock >= clock_kHz)
#endif
		/* This avoids a bad 720p DSI clock with 1.2GHz DPI PLL */
		if (adj_mode->clock != 72000)
			break;

		clock_kHz += 10;
	} while (1);

	pr_info("%s: pixel clock: req %dkHz -> actual: %dkHz\n",
		__func__, mode->clock, adj_mode->clock);


	DRM_DEBUG_DRIVER("mode_fixup  exit successfully.\n");
	return true;
}

static void hisi_drm_crtc_mode_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG_DRIVER(" enter successfully.\n");
	DRM_DEBUG_DRIVER(" exit successfully.\n");
}

static int hisi_drm_crtc_mode_set(struct drm_crtc *crtc,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adj_mode,
				   int x, int y,
				   struct drm_framebuffer *old_fb)
{
	struct hisi_drm_ade_crtc *crtc_ade = to_hisi_crtc(crtc);

	DRM_DEBUG_DRIVER("mode_set  enter successfully.\n");
	crtc_ade->dmode = mode;
	DRM_DEBUG_DRIVER("mode_set  exit successfully.\n");
	return 0;
}

static void hisi_drm_crtc_mode_commit(struct drm_crtc *crtc)
{

	DRM_DEBUG_DRIVER("mode_commit enter successfully.\n");
	DRM_DEBUG_DRIVER("mode_commit exit successfully.\n");
}

static int hisi_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					struct drm_framebuffer *old_fb)
{
	struct hisi_drm_ade_crtc *crtc_ade = to_hisi_crtc(crtc);
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct drm_gem_cma_object *obj = hisi_drm_fb_get_gem_obj(fb, 0);
	struct hisi_drm_fb *hisi_fb = to_hisi_drm_fb(fb);

	u8 __iomem *ade_base;
	u32 stride;
	u32 display_addr;
	u32 offset;
	u32 fb_hight;
	int bytes_pp = (fb->bits_per_pixel + 1) / 8;

	ade_base = crtc_ade->ade_base;
	stride = fb->pitches[0];
	offset = y * fb->pitches[0] + x * (fb->bits_per_pixel >> 3);
	display_addr = (u32)obj->paddr + offset;
	fb_hight = hisi_fb->is_fbdev_fb ? fb->height / HISI_NUM_FRAMEBUFFERS
			: fb->height;

	DRM_DEBUG_DRIVER("enter: fb stride=%d, paddr=0x%x, display_addr=0x%x, "
			 "fb=%dx%d, scanout=%dx%d\n",
			 stride, (u32)obj->paddr, display_addr,
			 fb->width, fb_hight, crtc->mode.hdisplay,
			 crtc->mode.vdisplay);

	/* TOP setting */
	writel(0, ade_base + ADE_WDMA2_SRC_CFG_REG);
	writel(0, ade_base + ADE_SCL3_MUX_CFG_REG);
	writel(0, ade_base + ADE_SCL1_MUX_CFG_REG);
	writel(0, ade_base + ADE_ROT_SRC_CFG_REG);
	writel(0, ade_base + ADE_SCL2_SRC_CFG_REG);
	writel(0, ade_base + ADE_SEC_OVLY_SRC_CFG_REG);
	writel(0, ade_base + ADE_WDMA3_SRC_CFG_REG);
	writel(0, ade_base + ADE_OVLY1_TRANS_CFG_REG);
	writel(0, ade_base + ADE_CTRAN5_TRANS_CFG_REG);
	writel(0, ade_base + ADE_OVLY_CTL_REG);
	writel(0, ade_base + ADE_SOFT_RST_SEL0_REG);
	writel(0, ade_base + ADE_SOFT_RST_SEL1_REG);
	set_TOP_SOFT_RST_SEL0_disp_rdma(ade_base, 1);
	set_TOP_SOFT_RST_SEL0_ctran5(ade_base, 1);
	set_TOP_SOFT_RST_SEL0_ctran6(ade_base, 1);
	writel(0, ade_base + ADE_RELOAD_DIS0_REG);
	writel(0, ade_base + ADE_RELOAD_DIS1_REG);
	writel(TOP_DISP_CH_SRC_RDMA, ade_base + ADE_DISP_SRC_CFG_REG);

	/* DISP DMA setting */
	if (16 == fb->bits_per_pixel)
		writel((ADE_RGB_565 << 16) & 0x1f0000,
		    ade_base + RD_CH_DISP_CTRL_REG);
	else if (32 == fb->bits_per_pixel)
		writel((ADE_ARGB_8888 << 16) & 0x1f0000,
		    ade_base + RD_CH_DISP_CTRL_REG);

	writel(display_addr, ade_base + RD_CH_DISP_ADDR_REG);
	writel((crtc->mode.vdisplay << 16) | crtc->mode.hdisplay * bytes_pp,
	       ade_base + RD_CH_DISP_SIZE_REG);
	writel(stride, ade_base + RD_CH_DISP_STRIDE_REG);
	writel(crtc->mode.vdisplay * stride, ade_base + RD_CH_DISP_SPACE_REG);
	writel(1, ade_base + RD_CH_DISP_EN_REG);

	/* ctran5 setting */
	writel(1, ade_base + ADE_CTRAN5_DIS_REG);
	writel(crtc->mode.hdisplay * crtc->mode.vdisplay - 1,
		ade_base + ADE_CTRAN5_IMAGE_SIZE_REG);

	/* ctran6 setting */
	writel(1, ade_base + ADE_CTRAN6_DIS_REG);
	writel(crtc->mode.hdisplay * crtc->mode.vdisplay - 1,
		ade_base + ADE_CTRAN6_IMAGE_SIZE_REG);

	/* enable ade and ldi */
	writel(ADE_ENABLE, ade_base + ADE_EN_REG);
	set_TOP_CTL_frm_end_start(ade_base, 1);
	set_LDI_CTRL_ldi_en(ade_base, ADE_ENABLE);

	DRM_DEBUG_DRIVER("mode_set_base exit successfully.\n");
	return 0;
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.dpms = hisi_drm_crtc_dpms,
	.mode_fixup = hisi_drm_crtc_mode_fixup,
	.prepare = hisi_drm_crtc_mode_prepare,
	.commit = hisi_drm_crtc_mode_commit,
	.mode_set = hisi_drm_crtc_mode_set,
	.mode_set_base = hisi_drm_crtc_mode_set_base,
};

static int hisi_drm_crtc_page_flip(struct drm_crtc *crtc,
				    struct drm_framebuffer *fb,
				    struct drm_pending_vblank_event *event,
				    uint32_t page_flip_flags)
{
	struct drm_framebuffer *old_fb;
	int ret;

	DRM_DEBUG_DRIVER("page_flip enter successfully.\n");

	old_fb = crtc->primary->fb;
	crtc->primary->fb = fb;

	ret = hisi_drm_crtc_mode_set_base(crtc, crtc->x, crtc->y, old_fb);
	if (ret) {
		DRM_ERROR("failed to hisi drm crtc mode set base\n");
		return ret;
	}

	DRM_DEBUG_DRIVER("page_flip exit successfully.\n");

	return 0;
}

static const struct drm_crtc_funcs crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_crtc_helper_set_config,
	.page_flip = hisi_drm_crtc_page_flip,
};

static int hisi_drm_crtc_create(struct hisi_drm_ade_crtc *crtc_ade)
{
	struct drm_crtc *crtc = &crtc_ade->crtc;
	int ret;

	crtc_ade->enable = false;
	crtc_ade->power_on = false;
	ret = drm_crtc_init(crtc_ade->drm_dev, crtc, &crtc_funcs);
	if (ret < 0)
		return ret;

	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	return 0;
}

static int hisi_drm_ade_dts_parse(struct platform_device *pdev,
				    struct hisi_drm_ade_crtc *crtc_ade)
{
	struct resource	    *res;
	struct device	    *dev;
	struct device_node  *np;
	int ret;

	dev = &pdev->dev;
	np  = dev->of_node;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ade_base");
	crtc_ade->ade_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(crtc_ade->ade_base)) {
		DRM_ERROR("failed to remap io region0\n");
		ret = PTR_ERR(crtc_ade->ade_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "media_base");
	crtc_ade->media_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(crtc_ade->media_base)) {
		DRM_ERROR("failed to remap io region1\n");
		ret = PTR_ERR(crtc_ade->media_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "media_noc_base");
	crtc_ade->media_noc_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(crtc_ade->media_noc_base)) {
		DRM_ERROR("failed to remap io region1\n");
		ret = PTR_ERR(crtc_ade->media_noc_base);
	}

	crtc_ade->ade_core_clk = devm_clk_get(&pdev->dev, "clk_ade_core");
	if (crtc_ade->ade_core_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_CORE\n");
	    return -ENODEV;
	}
	crtc_ade->media_noc_clk = devm_clk_get(&pdev->dev,
					"aclk_codec_jpeg_src");
	if (crtc_ade->media_noc_clk == NULL) {
		DRM_ERROR("failed to parse the CODEC_JPEG\n");
	    return -ENODEV;
	}
	crtc_ade->ade_pix_clk = devm_clk_get(&pdev->dev, "clk_ade_pix");
	if (crtc_ade->ade_pix_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_PIX_SRC\n");
	    return -ENODEV;
	}

	ret = of_property_read_u32(np, "ade_core_clk_rate",
				    &crtc_ade->ade_core_rate);
	if (ret) {
		DRM_ERROR("failed to parse the ade_core_clk_rate\n");
	    return -ENODEV;
	}
	ret = of_property_read_u32(np, "media_noc_clk_rate",
				    &crtc_ade->media_noc_rate);
	if (ret) {
		DRM_ERROR("failed to parse the media_noc_clk_rate\n");
	    return -ENODEV;
	}

	return ret;
}

static int hisi_ade_probe(struct platform_device *pdev)
{
	struct hisi_drm_ade_crtc *crtc_ade;
	int ret;

	/* debug setting */
	DRM_DEBUG_DRIVER("drm_ade enter.\n");

	crtc_ade = devm_kzalloc(&pdev->dev, sizeof(*crtc_ade), GFP_KERNEL);
	if (crtc_ade == NULL) {
		DRM_ERROR("failed to alloc the space\n");
		return -ENOMEM;
	}

	crtc_ade->drm_dev = dev_get_platdata(&pdev->dev);
	if (crtc_ade->drm_dev == NULL) {
		DRM_ERROR("no platform data\n");
		return -EINVAL;
	}

	ret = hisi_drm_ade_dts_parse(pdev, crtc_ade);
	if (ret) {
		DRM_ERROR("failed to dts parse\n");
		return ret;
	}

	ret = hisi_drm_crtc_create(crtc_ade);
	if (ret) {
		DRM_ERROR("failed to crtc creat\n");
		return ret;
	}

	DRM_DEBUG_DRIVER("drm_ade exit successfully.\n");

	return 0;
}

static int hisi_ade_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id hisi_ade_of_match[] = {
	{ .compatible = "hisilicon,hi6220-ade" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_ade_of_match);

static struct platform_driver ade_driver = {
	.remove = hisi_ade_remove,
	.driver = {
		   .name = "hisi-ade",
		   .owner = THIS_MODULE,
		   .of_match_table = hisi_ade_of_match,
	},
};

module_platform_driver_probe(ade_driver, hisi_ade_probe);
