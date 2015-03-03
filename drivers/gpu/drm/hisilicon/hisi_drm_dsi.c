/*
 *  Hisilicon Terminal SoCs drm driver
 *
 *  Copyright (c) 2014-2015 Hisilicon Limited.
 *  Author:
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/types.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_encoder_slave.h>

#ifdef CONFIG_DRM_HISI_FBDEV
#include "hisi_drm_fbdev.h"
#endif
#include "hisi_drm_ade.h"
#include "hisi_mipi_reg.h"
#include "hisi_drm_dsi.h"

/* DSI Relevant Registers */
#define LDI_HDMI_DSI_GT_REG         (0x7434)
#define DSI_CMD_MOD_CTRL_REG        (0x743C)
#define set_LDI_DSI_CMD_MOD_CTRL(nVal) \
	{writel(nVal, hisi_drm_get_ade_base() + DSI_CMD_MOD_CTRL_REG); }

#define connector_to_dsi(connector) \
	container_of(connector, struct hisi_dsi, connector)
#define encoder_to_dsi(encoder) \
	container_of(encoder, struct hisi_dsi, base.base)

#define DEFAULT_MIPI_CLK_RATE   19200000
#define MAX_TX_ESC_CLK    (10)
#define DSI_BURST_MODE    DSI_NON_BURST_SYNC_PULSES
#define ROUND(x, y) ((x) / (y) + ((x) % (y) * 10 / (y) >= 5 ? 1 : 0))

//#define USE_DEFAULT_720P_MODE 1

u8 *reg_base_mipi_dsi;


struct hisi_dsi {
	struct drm_connector connector;
	struct drm_encoder_slave base;
	struct device *dev;
	struct i2c_client *client;
	struct drm_i2c_encoder_driver *drm_i2c_driver;
	struct clk *dsi_cfg_clk;
	struct videomode vm;

	u8 __iomem *reg_base;
	u8 color_mode;

	u32 lanes;
	u32 format;
	u32 dphy_freq;
	u32 date_enable_pol;
	u32 vc;
	u32 mode_flags;

	int dpms;
};

struct mipi_dsi_phy_register {
	u32 clk_t_lpx;
	u32 clk_t_hs_prepare;
	u32 clk_t_hs_zero;
	u32 clk_t_hs_trial;
	u32 clk_t_wakeup;
	u32 data_t_lpx;
	u32 data_t_hs_prepare;
	u32 data_t_hs_zero;
	u32 data_t_hs_trial;
	u32 data_t_ta_go;
	u32 data_t_ta_get;
	u32 data_t_wakeup;
	u32 rg_hstx_ckg_sel;
	u32 rg_pll_fbd_div5f;
	u32 rg_pll_fbd_div1f;
	u32 rg_pll_fbd_2p;
	u32 rg_pll_enbwt;
	u32 rg_pll_fbd_p;
	u32 rg_pll_fbd_s;
	u32 rg_pll_pre_div1p;
	u32 rg_pll_pre_p;
	u32 rg_pll_vco_750M;
	u32 rg_pll_lpf_rs;
	u32 rg_pll_lpf_cs;
	u32 phy_clklp2hs_time;
	u32 phy_clkhs2lp_time;
	u32 phy_lp2hs_time;
	u32 phy_hs2lp_time;
	u32 clk_to_data_delay;
	u32 data_to_clk_delay;
	u32 lane_byte_clk;
	u32 clk_division;
	u32 burst_mode;
};

enum {
	DSI_16BITS_1 = 0,
	DSI_16BITS_2,
	DSI_16BITS_3,
	DSI_18BITS_1,
	DSI_18BITS_2,
	DSI_24BITS_1,
	DSI_24BITS_2,
	DSI_24BITS_3,
};

struct dsi_phy_seq_info {
	u32 min_range;
	u32 max_range;
	u32 rg_pll_vco_750M;
	u32 rg_hstx_ckg_sel;
};

enum {
	DSI_NON_BURST_SYNC_PULSES = 0,
	DSI_NON_BURST_SYNC_EVENTS,
	DSI_BURST_SYNC_PULSES_1,
	DSI_BURST_SYNC_PULSES_2
};

struct dsi_phy_seq_info dphy_seq_info[] = {
	{46,    62,    1,    7},
	{62,    93,    0,    7},
	{93,    125,   1,    6},
	{125,   187,   0,    6},
	{187,   250,   1,    5},
	{250,   375,   0,    5},
	{375,   500,   1,    4},
	{500,   750,   0,    4},
	{750,   1000,  1,    0},
	{1000,  1500,  0,    0}
};

static inline void set_reg(u8 *addr, u32 val, u32 bw, u32 bs)
{
	u32 mask = (1 << bw) - 1;
	u32 tmp = inp32(addr);

	tmp &= ~(mask << bs);
	outp32(addr, tmp | ((val & mask) << bs));
}

static inline struct drm_encoder_slave_funcs *
		get_slave_funcs(struct drm_encoder *enc)
{
	return to_encoder_slave(enc)->slave_funcs;
}

void get_dsi_phy_register(u32 *phy_freq,
		struct mipi_dsi_phy_register *phy_register)
{
	u32 ui = 0;
	u32 t_cfg_clk = 0;
	u32 seq_info_count = 0;
	u32 i = 0;
	u32 q_pll = 0;
	u32 m_pll = 0;
	u32 n_pll = 0;
	u32 r_pll = 0;
	u32 m_n = 0;
	u32 m_n_int = 0;

	DRM_DEBUG_DRIVER("enter.\n");
	BUG_ON(phy_freq == NULL);
	BUG_ON(phy_register == NULL);

	t_cfg_clk = 1000 / (DEFAULT_MIPI_CLK_RATE / 1000000);

	/* PLL parameters calculation */
	seq_info_count = sizeof(dphy_seq_info) / sizeof(struct dsi_phy_seq_info);
	for (i = 0; i < seq_info_count; i++) {
		if (*phy_freq > dphy_seq_info[i].min_range
				&& *phy_freq <= dphy_seq_info[i].max_range) {
			phy_register->rg_pll_vco_750M = dphy_seq_info[i].rg_pll_vco_750M;
			phy_register->rg_hstx_ckg_sel = dphy_seq_info[i].rg_hstx_ckg_sel;
			break;
		}
	}

	switch (phy_register->rg_hstx_ckg_sel) {
	case 7:
		q_pll = 16;
		break;
	case 6:
		q_pll = 8;
		break;
	case 5:
		q_pll = 4;
		break;
	case 4:
		q_pll = 2;
		break;
	default:
		q_pll = 1;
		break;
	}

	m_n_int = (*phy_freq) * q_pll * t_cfg_clk / 1000;
	m_n = (*phy_freq) * q_pll * t_cfg_clk % 1000 * 10 / 1000;
	if (m_n_int % 2 == 0) {
		if (m_n * 6 >= 50) {
			n_pll = 2;
			m_pll = (m_n_int + 1) * n_pll;
		} else if (m_n * 6 >= 30) {
			n_pll = 3;
			m_pll = m_n_int * n_pll + 2;
		} else {
			n_pll = 1;
			m_pll = m_n_int * n_pll;
		}
	} else {
		if (m_n * 6 >= 50) {
			n_pll = 1;
			m_pll = (m_n_int + 1) * n_pll;
		} else if (m_n * 6 >= 30) {
			n_pll = 1;
			m_pll = (m_n_int + 1) * n_pll;
		} else if (m_n * 6 >= 10) {
			n_pll = 3;
			m_pll = m_n_int * n_pll + 1;
		} else {
			n_pll = 2;
			m_pll = m_n_int * n_pll;
		}
	}

	if (n_pll == 1) {
		phy_register->rg_pll_fbd_p = 0;
		phy_register->rg_pll_pre_div1p = 1;
	} else {
		phy_register->rg_pll_fbd_p = n_pll;
		phy_register->rg_pll_pre_div1p = 0;
	}

	switch (phy_register->rg_pll_fbd_2p) {
	case 7:
		r_pll = 16;
		break;
	case 6:
		r_pll = 8;
		break;
	case 5:
		r_pll = 4;
		break;
	case 4:
		r_pll = 2;
		break;
	default:
		r_pll = 1;
		break;
	}

	if (m_pll == 2) {
		phy_register->rg_pll_pre_p = 0;
		phy_register->rg_pll_fbd_s = 0;
		phy_register->rg_pll_fbd_div1f = 0;
		phy_register->rg_pll_fbd_div5f = 1;
	} else if (m_pll >= 2 * 2 * r_pll && m_pll <= 2 * 4 * r_pll) {
		phy_register->rg_pll_pre_p = m_pll / (2 * r_pll);
		phy_register->rg_pll_fbd_s = 0;
		phy_register->rg_pll_fbd_div1f = 1;
		phy_register->rg_pll_fbd_div5f = 0;
	} else if (m_pll >= 2 * 5 * r_pll && m_pll <= 2 * 150 * r_pll) {
		if (((m_pll / (2 * r_pll)) % 2) == 0) {
			phy_register->rg_pll_pre_p = (m_pll / (2 * r_pll)) / 2 - 1;
			phy_register->rg_pll_fbd_s = (m_pll / (2 * r_pll)) % 2 + 2;
		} else {
			phy_register->rg_pll_pre_p = (m_pll / (2 * r_pll)) / 2;
			phy_register->rg_pll_fbd_s = (m_pll / (2 * r_pll)) % 2;
		}
		phy_register->rg_pll_fbd_div1f = 0;
		phy_register->rg_pll_fbd_div5f = 0;
	} else {
		phy_register->rg_pll_pre_p = 0;
		phy_register->rg_pll_fbd_s = 0;
		phy_register->rg_pll_fbd_div1f = 0;
		phy_register->rg_pll_fbd_div5f = 1;
	}

	*phy_freq = 1000 * m_pll / (t_cfg_clk * n_pll * q_pll);
	ui = 1000 / (*phy_freq);

	phy_register->clk_t_lpx = ROUND(50, 8 * ui);
	phy_register->clk_t_hs_prepare = ROUND(133, 16 * ui) - 1;

	phy_register->clk_t_hs_zero = ROUND(262, 8 * ui);
	phy_register->clk_t_hs_trial = 2 * (ROUND(60, 8 * ui) - 1);
	phy_register->clk_t_wakeup = ROUND(1000000, t_cfg_clk - 1) > 0xFF ? 0xFF : ROUND(1000000, t_cfg_clk - 1);
	phy_register->data_t_wakeup = phy_register->clk_t_wakeup;
	phy_register->data_t_lpx = phy_register->clk_t_lpx;
	phy_register->data_t_hs_prepare = ROUND(125 + 10 * ui, 16 * ui) - 1;
	phy_register->data_t_hs_zero = ROUND(105 + 6 * ui, 8 * ui);
	phy_register->data_t_hs_trial = 2 * (ROUND(60 + 4 * ui, 8 * ui) - 1);
	phy_register->data_t_ta_go = 3;
	phy_register->data_t_ta_get = 4;

	phy_register->rg_pll_enbwt = 1;
	phy_register->phy_clklp2hs_time = ROUND(407, 8 * ui) + 12;
	phy_register->phy_clkhs2lp_time = ROUND(105 + 12 * ui, 8 * ui);
	phy_register->phy_lp2hs_time = ROUND(240 + 12 * ui, 8 * ui) + 1;
	phy_register->phy_hs2lp_time = phy_register->phy_clkhs2lp_time;
	phy_register->clk_to_data_delay = 1 + phy_register->phy_clklp2hs_time;
	phy_register->data_to_clk_delay = ROUND(60 + 52 * ui, 8 * ui) + phy_register->phy_clkhs2lp_time;

	phy_register->lane_byte_clk = *phy_freq / 8;
	phy_register->clk_division = ((phy_register->lane_byte_clk % MAX_TX_ESC_CLK) > 0) ?
		(phy_register->lane_byte_clk / MAX_TX_ESC_CLK + 1) :
		(phy_register->lane_byte_clk / MAX_TX_ESC_CLK);

	phy_register->burst_mode = DSI_BURST_MODE;
	DRM_DEBUG_DRIVER("exit success.\n");
}

int mipi_init(struct hisi_dsi *dsi)
{
	union MIPIDSI_PHY_STATUS_UNION  mipidsi_phy_status;
	u32 hline_time = 0;
	u32 hsa_time = 0;
	u32 hbp_time = 0;
	u32 pixel_clk = 0;
	u32 i = 0;
	bool is_ready = false;
	u32 delay_count = 0;
	struct mipi_dsi_phy_register phy_register = {0};

	DRM_DEBUG_DRIVER("enter.\n");
	get_dsi_phy_register(&dsi->dphy_freq, &phy_register);
	set_MIPIDSI_PHY_IF_CFG_n_lanes(dsi->lanes-1);
	set_MIPIDSI_CLKMGR_CFG_tx_esc_clk_division(phy_register.clk_division);

	/*Video screen, enable Halt signal only*/
	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO)
		set_LDI_DSI_CMD_MOD_CTRL(0x2);

	set_MIPIDSI_PHY_RSTZ(0x00000000);
	set_MIPIDSI_PHY_TST_CTRL0(0x00000000);
	set_MIPIDSI_PHY_TST_CTRL0(0x00000001);
	set_MIPIDSI_PHY_TST_CTRL0(0x00000000);

	 /* clock lane Timing control - TLPX */
	set_MIPIDSI_PHY_TST_CTRL1(0x10010);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1(phy_register.clk_t_lpx);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/* clock lane Timing control - THS-PREPARE */
	set_MIPIDSI_PHY_TST_CTRL1(0x10011);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1(phy_register.clk_t_hs_prepare);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	 /* clock lane Timing control - THS-ZERO */
	set_MIPIDSI_PHY_TST_CTRL1(0x10012);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1(phy_register.clk_t_hs_zero);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/* clock lane Timing control - THS-TRAIL */
	set_MIPIDSI_PHY_TST_CTRL1(0x10013);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1(phy_register.clk_t_hs_trial);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/* clock lane Timing control - TWAKEUP */
	set_MIPIDSI_PHY_TST_CTRL1(0x10014);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1(phy_register.clk_t_wakeup);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/* data lane */
	for (i = 0; i < dsi->lanes; i++) {
		/* Timing control - TLPX*/
		set_MIPIDSI_PHY_TST_CTRL1(0x10020 + (i << 4));
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
		set_MIPIDSI_PHY_TST_CTRL1(phy_register.data_t_lpx);
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);

		/* Timing control - THS-PREPARE */
		set_MIPIDSI_PHY_TST_CTRL1(0x10021 + (i << 4));
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
		set_MIPIDSI_PHY_TST_CTRL1(phy_register.data_t_hs_prepare);
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);

		/* Timing control - THS-ZERO */
		set_MIPIDSI_PHY_TST_CTRL1(0x10022 + (i << 4));
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
		set_MIPIDSI_PHY_TST_CTRL1(phy_register.data_t_hs_zero);
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);

		/* Timing control - THS-TRAIL */
		set_MIPIDSI_PHY_TST_CTRL1(0x10023 + (i << 4));
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
		set_MIPIDSI_PHY_TST_CTRL1(phy_register.data_t_hs_trial);
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);

		/* Timing control - TTA-GO */
		set_MIPIDSI_PHY_TST_CTRL1(0x10024 + (i << 4));
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
		set_MIPIDSI_PHY_TST_CTRL1(phy_register.data_t_ta_go);
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);

		/* Timing control - TTA-GET */
		set_MIPIDSI_PHY_TST_CTRL1(0x10025 + (i << 4));
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
		set_MIPIDSI_PHY_TST_CTRL1(phy_register.data_t_ta_get);
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);

		/*  Timing control - TWAKEUP */
		set_MIPIDSI_PHY_TST_CTRL1(0x10026 + (i << 4));
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
		set_MIPIDSI_PHY_TST_CTRL1(phy_register.data_t_wakeup);
		set_MIPIDSI_PHY_TST_CTRL0(0x2);
		set_MIPIDSI_PHY_TST_CTRL0(0x0);
	}

	/* physical configuration I  */
	set_MIPIDSI_PHY_TST_CTRL1(0x10060);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1(phy_register.rg_hstx_ckg_sel);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/* physical configuration pll II  */
	set_MIPIDSI_PHY_TST_CTRL1(0x10063);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1((phy_register.rg_pll_fbd_div5f << 5) +
		(phy_register.rg_pll_fbd_div1f << 4) + (phy_register.rg_pll_fbd_2p << 1) +
		phy_register.rg_pll_enbwt);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/* physical configuration pll II  */
	set_MIPIDSI_PHY_TST_CTRL1(0x10064);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1(phy_register.rg_pll_fbd_p);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/* physical configuration pll III  */
	set_MIPIDSI_PHY_TST_CTRL1(0x10065);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1((1 << 4) + phy_register.rg_pll_fbd_s);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/*physical configuration pll IV*/
	set_MIPIDSI_PHY_TST_CTRL1(0x10066);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1((phy_register.rg_pll_pre_div1p << 7) +
		phy_register.rg_pll_pre_p);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	/*physical configuration pll V*/
	set_MIPIDSI_PHY_TST_CTRL1(0x10067);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);
	set_MIPIDSI_PHY_TST_CTRL1((5 << 5) + (phy_register.rg_pll_vco_750M << 4) +
		(phy_register.rg_pll_lpf_rs << 2) + phy_register.rg_pll_lpf_cs);
	set_MIPIDSI_PHY_TST_CTRL0(0x2);
	set_MIPIDSI_PHY_TST_CTRL0(0x0);

	set_MIPIDSI_PHY_RSTZ(0x00000004);
	udelay(1);
	set_MIPIDSI_PHY_RSTZ(0x00000005);
	udelay(1);
	set_MIPIDSI_PHY_RSTZ(0x00000007);
	msleep(1);

	while (1) {
		mipidsi_phy_status.ul32 = get_MIPIDSI_PHY_STATUS();
		if ((0x1 == mipidsi_phy_status.bits.phy_lock) || delay_count > 100) {
			is_ready = (delay_count < 100) ? true : false;
			delay_count = 0;
			break;
		}

		udelay(1);
		++delay_count;
	}

	if (!is_ready)
		DRM_INFO("phylock is not ready.\n");

	while (1) {
		mipidsi_phy_status.ul32 = get_MIPIDSI_PHY_STATUS();
		if ((0x1 == mipidsi_phy_status.bits.phy_stopstateclklane) ||
			delay_count > 100) {
			is_ready = (delay_count < 100) ? true : false;
			break;
		}

		udelay(1);
		++delay_count;
	}

	if (!is_ready)
		DRM_INFO("phystopstateclklane is not ready.\n");

	/* --------------configuring the DPI packet transmission----------------
	 *
	 * 1. Global configuration
	 * Configure Register PHY_IF_CFG with the correct number of lanes
	 * to be used by the controller.
	 *
	 * 2. Configure the DPI Interface:
	 * This defines how the DPI interface interacts with the controller.
	 *
	 * 3. Configure the TX_ESC clock frequency to a frequency lower than 20 MHz
	 * that is the maximum allowed frequency for D-PHY ESCAPE mode.
	 *
	*/

	set_MIPIDSI_DPI_VCID(dsi->vc);
	set_MIPIDSI_DPI_COLOR_CODING_dpi_color_coding(dsi->color_mode);
	set_MIPIDSI_DPI_CFG_POL_hsync_active_low(dsi->vm.flags & DISPLAY_FLAGS_HSYNC_HIGH ? 0 : 1);
	set_MIPIDSI_DPI_CFG_POL_dataen_active_low(dsi->date_enable_pol);
	set_MIPIDSI_DPI_CFG_POL_vsync_active_low(dsi->vm.flags & DISPLAY_FLAGS_VSYNC_HIGH ? 0 : 1);
	set_MIPIDSI_DPI_CFG_POL_shutd_active_low(0);
	set_MIPIDSI_DPI_CFG_POL_colorm_active_low(0);
	if (dsi->format == MIPI_DSI_FMT_RGB666)
		set_MIPIDSI_DPI_COLOR_CODING_loosely18_en(1);

	/*
	 * 4. Define the DPI Horizontal timing configuration:
	 *
	 * Hsa_time = HSA*(PCLK period/Clk Lane Byte Period);
	 * Hbp_time = HBP*(PCLK period/Clk Lane Byte Period);
	 * Hline_time = (HSA+HBP+HACT+HFP)*(PCLK period/Clk Lane Byte Period);
	*/

	pixel_clk = dsi->vm.pixelclock;
	hsa_time = dsi->vm.hsync_len * phy_register.lane_byte_clk / pixel_clk;
	hbp_time = dsi->vm.hback_porch * phy_register.lane_byte_clk / pixel_clk;
	hline_time  = (dsi->vm.hsync_len + dsi->vm.hback_porch +
		dsi->vm.hactive + dsi->vm.hfront_porch) *
		phy_register.lane_byte_clk / pixel_clk;
	set_MIPIDSI_VID_HSA_TIME(hsa_time);
	set_MIPIDSI_VID_HBP_TIME(hbp_time);
	set_MIPIDSI_VID_HLINE_TIME(hline_time);

	DRM_INFO("%s,pixcel_clk=%d,dphy_freq=%d,hsa=%d,hbp=%d,hline=%d", __func__,
			pixel_clk, dsi->dphy_freq, hsa_time, hbp_time, hline_time);
	/*
	 * 5. Define the Vertical line configuration:
	 *
	*/
	if (dsi->vm.vsync_len > 15)
		dsi->vm.vsync_len = 15;

	set_MIPIDSI_VID_VSA_LINES(dsi->vm.vsync_len);
	set_MIPIDSI_VID_VBP_LINES(dsi->vm.vback_porch);
	set_MIPIDSI_VID_VFP_LINES(dsi->vm.vfront_porch);
	set_MIPIDSI_VID_VACTIVE_LINES(dsi->vm.vactive);
	set_MIPIDSI_VID_PKT_SIZE(dsi->vm.hactive);

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		set_MIPIDSI_VID_MODE_CFG_lp_vsa_en(1);
		set_MIPIDSI_VID_MODE_CFG_lp_vbp_en(1);
		set_MIPIDSI_VID_MODE_CFG_lp_vfp_en(1);
		set_MIPIDSI_VID_MODE_CFG_lp_vact_en(1);
		set_MIPIDSI_VID_MODE_CFG_lp_hbp_en(1);
		set_MIPIDSI_VID_MODE_CFG_lp_hfp_en(1);
	 /*VSA/VBP/VFP max transfer byte in LP mode*/
		set_MIPIDSI_DPI_CFG_LP_TIM(0);
	 /*enable LP command transfer*/
		set_MIPIDSI_VID_MODE_CFG_lp_cmd_en(1);
	 /*config max read time*/
		set_MIPIDSI_PHY_TMR_CFG_max_rd_time(0xFFFF);
	}

	/* Configure core's phy parameters */
	set_MIPIDSI_BTA_TO_CNT_bta_to_cnt(4095);
	set_MIPIDSI_PHY_TMR_CFG_phy_lp2hs_time(phy_register.phy_lp2hs_time);
	set_MIPIDSI_PHY_TMR_CFG_phy_hs2lp_time(phy_register.phy_hs2lp_time);
	set_MIPIDSI_PHY_TMR_LPCLK_CFG_phy_clklp2hs_time(phy_register.phy_clklp2hs_time);
	set_MIPIDSI_PHY_TMR_LPCLK_CFG_phy_clkhs2lp_time(phy_register.phy_clkhs2lp_time);
	set_MIPIDSI_PHY_TMR_clk_to_data_delay(phy_register.clk_to_data_delay);
	set_MIPIDSI_PHY_TMR_data_to_clk_delay(phy_register.data_to_clk_delay);
	/*
	 * 3. Select the Video Transmission Mode:
	 * This defines how the processor requires the video line to be
	 * transported through the DSI link.
	*/
	set_MIPIDSI_VID_MODE_CFG_frame_bta_ack_en(0);
	set_MIPIDSI_VID_MODE_CFG_vid_mode_type(phy_register.burst_mode);
	set_MIPIDSI_LPCLK_CTRL_auto_clklane_ctrl(0);
	/* for dsi read */
	set_MIPIDSI_PCKHDL_CFG_bta_en(1);
	/* Enable EOTP TX; Enable EDPI, ALLOWED_CMD_SIZE = 720*/
	if (dsi->mode_flags == MIPI_DSI_MODE_VIDEO)
		set_MIPIDSI_EDPI_CMD_SIZE(dsi->vm.hactive);

	/*------------DSI and D-PHY Initialization-----------------*/
	/* Waking up Core */
	set_MIPIDSI_PWR_UP_shutdownz(1);
	DRM_INFO("%s , exit success!\n", __func__);
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

static int hisi_dsi_init(struct hisi_dsi *dsi)
{
	int ret = 0;

	DRM_DEBUG_DRIVER("enter.\n");
	/* mipi dphy clock enable */
	ret = clk_prepare_enable(dsi->dsi_cfg_clk);
	if (ret != 0) {
		dev_err(dsi->dev, "failed to enable dsi_cfg_clk, error=%d!\n", ret);
		return ret;
	}
	/* dsi pixel on */
	set_reg(hisi_drm_get_ade_base() + LDI_HDMI_DSI_GT_REG, 0x0, 1, 0);
	DRM_DEBUG_DRIVER("exit success.\n");
	return mipi_init(dsi);
}

static int hisi_dsi_enable(struct hisi_dsi *dsi)
{
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");
	ret = hisi_dsi_init(dsi);
	if (ret)
		DRM_ERROR("failed to init dsi\n");

	set_dsi_mode(MIPIDSI_VIDEO_MODE);
	set_MIPIDSI_GEN_HDR(NULL, 0x137);
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

static int hisi_dsi_disable(struct hisi_dsi *dsi)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

static void hisi_drm_encoder_destroy(struct drm_encoder *encoder)
{
	struct hisi_dsi *dsi = encoder_to_dsi(encoder);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	DRM_DEBUG_DRIVER("enter.\n");
	/*release*/
	if (sfuncs && sfuncs->destroy)
		sfuncs->destroy(encoder);
	drm_encoder_cleanup(encoder);
	kfree(dsi);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static struct drm_encoder_funcs hisi_encoder_funcs = {
	.destroy = hisi_drm_encoder_destroy
};

static void hisi_drm_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct hisi_dsi *dsi = encoder_to_dsi(encoder);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	DRM_DEBUG_DRIVER("enter. dpms=%d\n", mode);
	if (dsi->dpms == mode)
		return;

	dsi->dpms = mode;
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		hisi_dsi_enable(dsi);
		break;
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
	case DRM_MODE_DPMS_OFF:
		hisi_dsi_disable(dsi);
		break;
	default:
		break;
	}

	if (sfuncs && sfuncs->dpms)
		sfuncs->dpms(encoder, mode);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static bool
hisi_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	bool ret = true;

	DRM_DEBUG_DRIVER("enter.\n");
	if (sfuncs && sfuncs->mode_fixup)
		ret =  sfuncs->mode_fixup(encoder, mode, adjusted_mode);

	DRM_DEBUG_DRIVER("exit success.ret=%d\n", ret);

	return ret;
}

static void hisi_drm_encoder_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
	struct hisi_dsi *dsi = encoder_to_dsi(encoder);
	struct videomode *vm = &dsi->vm;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	DRM_DEBUG_DRIVER("enter.\n");
	vm->pixelclock = mode->clock/1000;
	vm->hactive = mode->hdisplay;
	vm->vactive = mode->vdisplay;
	vm->vfront_porch = mode->vsync_start - mode->vdisplay;
	vm->vback_porch = mode->vtotal - mode->vsync_end;
	vm->vsync_len = mode->vsync_end - mode->vsync_start;
	vm->hfront_porch = mode->hsync_start - mode->hdisplay;
	vm->hback_porch = mode->htotal - mode->hsync_end;
	vm->hsync_len = mode->hsync_end - mode->hsync_start;

#if USE_DEFAULT_720P_MODE != 1
	/* laneBitRate >= pixelClk*24/lanes */
	dsi->dphy_freq = vm->pixelclock*24/dsi->lanes; /*  + 30; */
#endif

	vm->flags = 0;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		vm->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		vm->flags |= DISPLAY_FLAGS_HSYNC_LOW;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		vm->flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		vm->flags |= DISPLAY_FLAGS_VSYNC_LOW;

	if (sfuncs && sfuncs->mode_set)
		sfuncs->mode_set(encoder, mode, adjusted_mode);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_drm_encoder_prepare(struct drm_encoder *encoder)
{
	/*do nothing at all*/
}

static void hisi_drm_encoder_commit(struct drm_encoder *encoder)
{
	DRM_DEBUG_DRIVER("enter.\n");
	hisi_drm_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_drm_encoder_disable(struct drm_encoder *encoder)
{
	DRM_DEBUG_DRIVER("enter.\n");
	hisi_drm_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static struct drm_encoder_helper_funcs hisi_encoder_helper_funcs = {
	.dpms		= hisi_drm_encoder_dpms,
	.mode_fixup	= hisi_drm_encoder_mode_fixup,
	.mode_set	= hisi_drm_encoder_mode_set,
	.prepare	= hisi_drm_encoder_prepare,
	.commit		= hisi_drm_encoder_commit,
	.disable	= hisi_drm_encoder_disable
};

static enum drm_connector_status
hisi_dsi_detect(struct drm_connector *connector, bool force)
{
	struct hisi_dsi *dsi = connector_to_dsi(connector);
	struct drm_encoder *encoder = &dsi->base.base;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	enum drm_connector_status status = connector_status_unknown;

	DRM_DEBUG_DRIVER("enter.\n");
	if (sfuncs && sfuncs->detect)
		status = sfuncs->detect(encoder, connector);

	DRM_DEBUG_DRIVER("exit success. status=%d\n", status);
	return status;
}

static void hisi_dsi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs hisi_dsi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = hisi_dsi_detect,
	.destroy = hisi_dsi_connector_destroy
};

static int hisi_get_default_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct hisi_dsi *dsi = connector_to_dsi(connector);

	DRM_DEBUG_DRIVER("enter.\n");
	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}

	mode->vrefresh = 60;
	mode->clock = 75000;
	mode->hdisplay = 1280;
	mode->hsync_start = 1500;
	mode->hsync_end = 1540;
	mode->htotal = 1650;
	mode->vdisplay = 720;
	mode->vsync_start = 740;
	mode->vsync_end = 745;
	mode->vtotal = 750;
	mode->type = 0x40;
	mode->flags = 0xa;
	drm_mode_probed_add(connector, mode);

	dsi->dphy_freq = 640; /*  640M for 720p */
	DRM_DEBUG_DRIVER("exit successfully.\n");
	return 1;
}
static int hisi_dsi_get_modes(struct drm_connector *connector)
{
	struct hisi_dsi *dsi = connector_to_dsi(connector);
	struct drm_encoder *encoder = &dsi->base.base;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	int count = 0;

	DRM_DEBUG_DRIVER("enter.\n");
	if (sfuncs && sfuncs->get_modes)
		count = sfuncs->get_modes(encoder, connector);

	DRM_DEBUG_DRIVER("exit success. count=%d\n", count);
#if USE_DEFAULT_720P_MODE != 1
	return count;
#else
	return hisi_get_default_modes(connector);
#endif
}

static struct drm_encoder *
hisi_dsi_best_encoder(struct drm_connector *connector)
{
	struct hisi_dsi *dsi = connector_to_dsi(connector);

	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return &dsi->base.base;
}

static int hisi_drm_connector_mode_valid(struct drm_connector *connector,
					  struct drm_display_mode *mode)
{

	struct hisi_dsi *dsi = connector_to_dsi(connector);
	struct drm_encoder *encoder = &dsi->base.base;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	int ret = MODE_OK;

#if USE_DEFAULT_720P_MODE
	if (mode->vdisplay != 720)
		return MODE_ONE_SIZE;
#endif

	DRM_DEBUG_DRIVER("enter.\n");
	if (sfuncs && sfuncs->mode_valid)
		ret = sfuncs->mode_valid(encoder, mode);

	DRM_DEBUG_DRIVER("exit success. ret=%d\n", ret);
	return ret;
}

static struct drm_connector_helper_funcs hisi_dsi_connector_helper_funcs = {
	.get_modes = hisi_dsi_get_modes,
	.best_encoder =  hisi_dsi_best_encoder,
	.mode_valid = hisi_drm_connector_mode_valid,
};

static int hisi_drm_encoder_create(struct drm_device *dev, struct hisi_dsi *dsi)
{
	/* int ret; */
	struct drm_encoder *encoder = &dsi->base.base;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");
	dsi->dpms = DRM_MODE_DPMS_OFF;
	encoder->possible_crtcs = 1;
	drm_encoder_init(dev, encoder, &hisi_encoder_funcs, DRM_MODE_ENCODER_DSI);
	drm_encoder_helper_add(encoder, &hisi_encoder_helper_funcs);

	ret = dsi->drm_i2c_driver->encoder_init(dsi->client, dev, &dsi->base);
	if (ret) {
		DRM_ERROR("drm_i2c_encoder_init error\n");
		return ret;
	}

	if (!dsi->base.slave_funcs) {
		DRM_ERROR("failed check encoder function\n");
		return -ENODEV;
	}

	return 0;
	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_drm_connector_create(struct drm_device *dev, struct hisi_dsi *dsi)
{
	int ret;
	struct drm_encoder *encoder = &dsi->base.base;
	struct drm_connector *connector = &dsi->connector;

	DRM_DEBUG_DRIVER("enter.\n");
	ret = drm_connector_init(encoder->dev, connector,
					&hisi_dsi_connector_funcs,
					DRM_MODE_CONNECTOR_DSI);
	drm_connector_helper_add(connector, &hisi_dsi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static int hisi_dsi_probe(struct platform_device *pdev)
{
	int ret;
	struct hisi_dsi *dsi;
	struct resource *res;
	struct drm_device *dev = dev_get_platdata(&pdev->dev);
	struct device_node *slave_node;
	struct device_node *np = pdev->dev.of_node;

	DRM_DEBUG_DRIVER("enter.\n");

	dsi = devm_kzalloc(&pdev->dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi) {
		dev_err(&pdev->dev, "failed to allocate dsi object.\n");
		ret = -ENOMEM;
	}

	dsi->dsi_cfg_clk = clk_get(&pdev->dev, "pclk_dsi");
	if (IS_ERR(dsi->dsi_cfg_clk)) {
		dev_info(&pdev->dev, "failed to get dsi clock");
		ret = PTR_ERR(dsi->dsi_cfg_clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->reg_base = devm_ioremap_resource(&pdev->dev, res);
	reg_base_mipi_dsi = dsi->reg_base;
	if (IS_ERR(dsi->reg_base)) {
		dev_err(&pdev->dev, "failed to remap io region\n");
		ret = PTR_ERR(dsi->reg_base);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "vc", &dsi->vc);
	if (ret)
		dev_err(&pdev->dev, "failed to get vc");

	slave_node = of_parse_phandle(np, "encoder-slave", 0);
	if (!slave_node) {
		DRM_ERROR("failed to get slave encoder node\n");
		return -EINVAL;
	}

	dsi->client = of_find_i2c_device_by_node(slave_node);
	of_node_put(slave_node);
	if (!dsi->client) {
		DRM_ERROR("failed to find slave encoder i2c client\n");
		return -EPROBE_DEFER;
	}

	dsi->drm_i2c_driver = to_drm_i2c_encoder_driver(
		to_i2c_driver(dsi->client->dev.driver));
	if (!dsi->drm_i2c_driver) {
		DRM_ERROR("failed initialize encoder driver\n");
		return -EPROBE_DEFER;
	}

	dsi->color_mode = DSI_24BITS_1;
	dsi->date_enable_pol = 0;
	dsi->lanes = 3;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	ret = hisi_drm_encoder_create(dev, dsi);
	if (ret) {
		DRM_ERROR("failed to create encoder\n");
		return ret;
	}
	hisi_drm_connector_create(dev, dsi);

	platform_set_drvdata(pdev, dsi);

	/* fbdev initialization should be put at last position */
#ifdef CONFIG_DRM_HISI_FBDEV
	ret = hisi_drm_fbdev_init(dev);
	if (ret) {
		DRM_ERROR("failed to initialize fbdev\n");
		return ret;
	}
#endif

	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

static int hisi_dsi_remove(struct platform_device *pdev)
{

	struct hisi_dsi *dsi = dev_get_drvdata(&pdev->dev);

	devm_kfree(&pdev->dev, dsi);
	return 0;
}

static struct of_device_id hisi_dsi_of_match[] = {
	{.compatible = "hisilicon,hi6220-dsi"},
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_dsi_of_match);

static struct platform_driver dsi_driver = {
	.probe = hisi_dsi_probe,
	.remove = hisi_dsi_remove,
	.driver = {
		.name = "hisi-dsi",
		.owner = THIS_MODULE,
		.of_match_table = hisi_dsi_of_match,
	},
};

module_platform_driver(dsi_driver);
