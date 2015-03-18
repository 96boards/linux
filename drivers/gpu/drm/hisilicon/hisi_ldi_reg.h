/*
 *  Hisilicon Terminal SoCs drm driver
 *
 *  Copyright (c) 2014-2015 Hisilicon Limited.
 *  Author:
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#ifndef __HISI_LDI_REG_H__
#define __HISI_LDI_REG_H__

/********** LDI Register Offset ***********/
#define LDI_HRZ_CTRL0_REG		(0x7400)
#define LDI_HRZ_CTRL1_REG		(0x7404)
#define LDI_VRT_CTRL0_REG		(0x7408)
#define LDI_VRT_CTRL1_REG		(0x740C)
#define LDI_PLR_CTRL_REG		(0x7410)
#define LDI_DSP_SIZE_REG		(0x7414)
#define LDI_INT_EN_REG			(0x741C)
#define LDI_CTRL_REG			(0x7420)
#define LDI_ORG_INT_REG			(0x7424)
#define LDI_MSK_INT_REG			(0x7428)
#define LDI_INT_CLR_REG			(0x742C)
#define LDI_WORK_MODE_REG		(0x7430)
#define LDI_DE_SPACE_LOW_REG		(0x7438)
#define LDI_MCU_INTS_REG		(0x7450)
#define LDI_MCU_INTE_REG		(0x7454)
#define LDI_MCU_INTC_REG		(0x7458)
#define LDI_HDMI_DSI_GT_REG		(0x7434)


/* LDI Timing Polarity defines */
#define HISI_LDI_FLAG_NVSYNC		BIT(0)
#define HISI_LDI_FLAG_NHSYNC		BIT(1)
#define HISI_LDI_FLAG_NPIXCLK		BIT(2)
#define HISI_LDI_FLAG_NDE		BIT(3)

/********** LDI Register Union Struct ***********/
union U_LDI_HRZ_CTRL0 {
struct {
	unsigned int	hfp                   :12;
	unsigned int	Reserved_564          :8;
	unsigned int	hbp                   :12;
	} bits;
	unsigned int    u32;
};

union U_LDI_HRZ_CTRL1 {
struct {
	unsigned int    hsw                   :12;
	unsigned int    Reserved_566          :20;
	} bits;
	unsigned int    u32;
};

union U_LDI_VRT_CTRL0 {
struct {
	unsigned int    vfp                   :12;
	unsigned int    Reserved_567          :8;
	unsigned int    vbp                   :12;
	} bits;
	unsigned int    u32;
};

union U_LDI_VRT_CTRL1 {
struct {
	unsigned int    vsw                   :12;
	unsigned int    Reserved_568          :20;
	} bits;
	unsigned int    u32;
};

union U_LDI_DSP_SIZE {
struct {
	unsigned int    hsize                 :12;
	unsigned int    Reserved_570          :8;
	unsigned int    vsize                 :12;
	} bits;
	unsigned int    u32;
};

union U_LDI_CTRL {
struct {
	unsigned int    ldi_en                :1;
	unsigned int    disp_mode_buf         :1;
	unsigned int    date_gate_en          :1;
	unsigned int    bpp                   :2;
	unsigned int    wait_vsync_en         :1;
	unsigned int    corlorbar_width       :7;
	unsigned int    bgr                   :1;
	unsigned int    color_mode            :1;
	unsigned int    shutdown              :1;
	unsigned int    vactive_line          :12;
	unsigned int    ldi_en_self_clr       :1;
	unsigned int    Reserved_573          :3;
	} bits;
	unsigned int    u32;
};

union U_LDI_WORK_MODE {
struct {
	unsigned int    work_mode             :1;
	unsigned int    wback_en              :1;
	unsigned int    colorbar_en           :1;
	unsigned int    Reserved_577          :29;
	} bits;
	unsigned int    u32;
};


/********** LDI Register Write/Read Helper functions ***********/
static inline void set_reg(u8 *addr, u32 val, u32 bw, u32 bs)
{
	u32 mask = (1 << bw) - 1;
	u32 tmp = readl(addr);

	tmp &= ~(mask << bs);
	writel(tmp | ((val & mask) << bs), addr);
}

static inline void set_LDI_HRZ_CTRL0(u8 *ade_base, u32 hfp, u32 hbp)
{
	volatile union U_LDI_HRZ_CTRL0 ldi_hrz_ctrl0;
	u8 *addr = ade_base + LDI_HRZ_CTRL0_REG;

	ldi_hrz_ctrl0.u32 = readl(addr);
	ldi_hrz_ctrl0.bits.hfp = hfp;
	ldi_hrz_ctrl0.bits.hbp = hbp;
	writel(ldi_hrz_ctrl0.u32, addr);
}

static inline void set_LDI_HRZ_CTRL1_hsw(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_HRZ_CTRL1 ldi_hrz_ctrl1;
	u8 *addr = ade_base + LDI_HRZ_CTRL1_REG;

	ldi_hrz_ctrl1.u32 = readl(addr);
	ldi_hrz_ctrl1.bits.hsw = (nVal > 0) ? nVal - 1 : 0;
	writel(ldi_hrz_ctrl1.u32, addr);
}

static inline void set_LDI_VRT_CTRL0(u8 *ade_base, u32 vfp, u32 vbp)
{
	volatile union U_LDI_VRT_CTRL0 ldi_vrt_ctrl0;
	u8 *addr = ade_base + LDI_VRT_CTRL0_REG;

	ldi_vrt_ctrl0.u32 = readl(addr);
	ldi_vrt_ctrl0.bits.vfp = vfp;
	ldi_vrt_ctrl0.bits.vbp = vbp;
	writel(ldi_vrt_ctrl0.u32, addr);
}

static inline void set_LDI_VRT_CTRL1_vsw(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_VRT_CTRL1 ldi_vrt_ctrl1;
	u8 *addr = ade_base + LDI_VRT_CTRL1_REG;

	ldi_vrt_ctrl1.u32 = readl(addr);
	ldi_vrt_ctrl1.bits.vsw = (nVal > 0) ? nVal - 1 : 0;
	writel(ldi_vrt_ctrl1.u32, addr);
}

static inline void set_LDI_DSP_SIZE_size(u8 *ade_base, u32 hVal, u32 vVal)
{
	volatile union U_LDI_DSP_SIZE ldi_dsp_size;
	u8 *addr = ade_base + LDI_DSP_SIZE_REG;

	ldi_dsp_size.bits.hsize = (hVal > 0) ? hVal-1 : 0;
	ldi_dsp_size.bits.vsize = (vVal > 0) ? vVal-1 : 0;
	writel(ldi_dsp_size.u32, addr);
}

static inline void set_LDI_CTRL_ldi_en(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_CTRL ldi_ctrl;
	u8 *addr = ade_base + LDI_CTRL_REG;

	ldi_ctrl.u32 = readl(addr);
	ldi_ctrl.bits.ldi_en = nVal;
	writel(ldi_ctrl.u32, addr);
}

static inline void set_LDI_CTRL_disp_mode(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_CTRL ldi_ctrl;
	u8 *addr = ade_base + LDI_CTRL_REG;

	ldi_ctrl.u32 = readl(addr);
	ldi_ctrl.bits.disp_mode_buf = nVal;
	writel(ldi_ctrl.u32, addr);
}

static inline void set_LDI_CTRL_bpp(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_CTRL ldi_ctrl;
	u8 *addr = ade_base + LDI_CTRL_REG;

	ldi_ctrl.u32 = readl(addr);
	ldi_ctrl.bits.bpp = nVal;
	writel(ldi_ctrl.u32, addr);
}

static inline void set_LDI_CTRL_corlorbar_width(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_CTRL ldi_ctrl;
	u8 *addr = ade_base + LDI_CTRL_REG;

	ldi_ctrl.u32 = readl(addr);
	ldi_ctrl.bits.corlorbar_width = (nVal > 0) ? nVal - 1 : 0;
	writel(ldi_ctrl.u32, addr);
}

static inline void set_LDI_CTRL_bgr(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_CTRL ldi_ctrl;
	u8 *addr = ade_base + LDI_CTRL_REG;

	ldi_ctrl.u32 = readl(addr);
	ldi_ctrl.bits.bgr = nVal;
	writel(ldi_ctrl.u32, addr);
}

static inline void set_LDI_WORK_MODE_work_mode(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_WORK_MODE ldi_work_mode;
	u8 *addr = ade_base + LDI_WORK_MODE_REG;

	ldi_work_mode.u32 = readl(addr);
	ldi_work_mode.bits.work_mode = nVal;
	writel(ldi_work_mode.u32, addr);
}

static inline void set_LDI_WORK_MODE_colorbar_en(u8 *ade_base, u32 nVal)
{
	volatile union U_LDI_WORK_MODE ldi_work_mode;
	u8 *addr = ade_base + LDI_WORK_MODE_REG;

	ldi_work_mode.u32 = readl(addr);
	ldi_work_mode.bits.colorbar_en = nVal;
	writel(ldi_work_mode.u32, addr);
}

#endif  /* __LDI_REG_H__ */
