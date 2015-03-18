/*
 *  Hisilicon Kirin620 SoC drm driver
 *
 *  Copyright (c) 2014-2015 Hisilicon Limited.
 *  Author:
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
#include <linux/io.h>
#include <linux/types.h>
#include "hisi_mipi_reg.h"

void set_MIPIDSI_PWR_UP_shutdownz(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PWR_UP_ADDR;
	union MIPIDSI_PWR_UP_UNION  mipidsi_pwr_up;

	mipidsi_pwr_up.ul32 = inp32(addr);
	mipidsi_pwr_up.bits.shutdownz = nVal;

	outp32(addr, mipidsi_pwr_up.ul32);
}

void set_MIPIDSI_CLKMGR_CFG_tx_esc_clk_division(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_CLKMGR_CFG_ADDR;
	union MIPIDSI_CLKMGR_CFG_UNION  mipidsi_clkmgr_cfg;

	mipidsi_clkmgr_cfg.ul32 = inp32(addr);
	mipidsi_clkmgr_cfg.bits.tx_esc_clk_division = nVal;

	outp32(addr, mipidsi_clkmgr_cfg.ul32);
}

void set_MIPIDSI_DPI_VCID(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_VCID_ADDR;
	union MIPIDSI_DPI_VCID_UNION mipidsi_dpi_vcid;

	mipidsi_dpi_vcid.ul32 = inp32(addr);
	mipidsi_dpi_vcid.bits.dpi_vcid = nVal;

	outp32(addr, mipidsi_dpi_vcid.ul32);
}

void set_MIPIDSI_DPI_COLOR_CODING_dpi_color_coding(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_COLOR_CODING_ADDR;
	union MIPIDSI_DPI_COLOR_CODING_UNION  mipidsi_dpi_color_coding;

	mipidsi_dpi_color_coding.ul32 = inp32(addr);
	mipidsi_dpi_color_coding.bits.dpi_color_coding = nVal;

	outp32(addr, mipidsi_dpi_color_coding.ul32);
}

void set_MIPIDSI_DPI_COLOR_CODING_loosely18_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_COLOR_CODING_ADDR;
	union MIPIDSI_DPI_COLOR_CODING_UNION  mipidsi_dpi_color_coding;

	mipidsi_dpi_color_coding.ul32 = inp32(addr);
	mipidsi_dpi_color_coding.bits.loosely18_en = nVal;

	outp32(addr, mipidsi_dpi_color_coding.ul32);
}

void set_MIPIDSI_DPI_CFG_POL_dataen_active_low(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_CFG_POL_ADDR;
	union MIPIDSI_DPI_CFG_POL_UNION  mipidsi_dpi_cfg_pol;

	mipidsi_dpi_cfg_pol.ul32 = inp32(addr);
	mipidsi_dpi_cfg_pol.bits.dataen_active_low = nVal;

	outp32(addr, mipidsi_dpi_cfg_pol.ul32);
}

void set_MIPIDSI_DPI_CFG_POL_vsync_active_low(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_CFG_POL_ADDR;
	union MIPIDSI_DPI_CFG_POL_UNION  mipidsi_dpi_cfg_pol;

	mipidsi_dpi_cfg_pol.ul32 = inp32(addr);
	mipidsi_dpi_cfg_pol.bits.vsync_active_low = nVal;

	outp32(addr, mipidsi_dpi_cfg_pol.ul32);
}

void set_MIPIDSI_DPI_CFG_POL_hsync_active_low(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_CFG_POL_ADDR;
	union MIPIDSI_DPI_CFG_POL_UNION  mipidsi_dpi_cfg_pol;

	mipidsi_dpi_cfg_pol.ul32 = inp32(addr);
	mipidsi_dpi_cfg_pol.bits.hsync_active_low = nVal;

	outp32(addr, mipidsi_dpi_cfg_pol.ul32);
}

void set_MIPIDSI_DPI_CFG_POL_shutd_active_low(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_CFG_POL_ADDR;
	union MIPIDSI_DPI_CFG_POL_UNION  mipidsi_dpi_cfg_pol;

	mipidsi_dpi_cfg_pol.ul32 = inp32(addr);
	mipidsi_dpi_cfg_pol.bits.shutd_active_low = nVal;

	outp32(addr, mipidsi_dpi_cfg_pol.ul32);
}

void set_MIPIDSI_DPI_CFG_POL_colorm_active_low(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_CFG_POL_ADDR;
	union MIPIDSI_DPI_CFG_POL_UNION  mipidsi_dpi_cfg_pol;

	mipidsi_dpi_cfg_pol.ul32 = inp32(addr);
	mipidsi_dpi_cfg_pol.bits.colorm_active_low  = nVal;

	outp32(addr, mipidsi_dpi_cfg_pol.ul32);
}

void set_MIPIDSI_DPI_CFG_LP_TIM(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_DPI_LP_CMD_TIM_ADDR;
	union MIPIDSI_DPI_LP_CMD_TIM_UNION  mipidsi_dpi_cfg_tim;

	mipidsi_dpi_cfg_tim.ul32 = inp32(addr);
	mipidsi_dpi_cfg_tim.bits.outvact_lpcmd_time = nVal;

	outp32(addr, mipidsi_dpi_cfg_tim.ul32);
}

void set_MIPIDSI_PCKHDL_CFG_bta_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PCKHDL_CFG_ADDR;
	union MIPIDSI_PCKHDL_CFG_UNION  mipidsi_pckhdl_cfg;

	mipidsi_pckhdl_cfg.ul32 = inp32(addr);
	mipidsi_pckhdl_cfg.bits.bta_en = nVal;

	outp32(addr, mipidsi_pckhdl_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_lp_vsa_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.lp_vsa_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_lp_vbp_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.lp_vbp_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_lp_vfp_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.lp_vfp_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_lp_vact_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.lp_vact_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_lp_hbp_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.lp_hbp_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_lp_hfp_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.lp_hfp_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_frame_bta_ack_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.frame_bta_ack_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_lp_cmd_en(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.lp_cmd_en = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_VID_MODE_CFG_vid_mode_type(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_MODE_CFG_ADDR;
	union MIPIDSI_VID_MODE_CFG_UNION  mipidsi_vid_mode_cfg;

	mipidsi_vid_mode_cfg.ul32 = inp32(addr);
	mipidsi_vid_mode_cfg.bits.vid_mode_type = nVal;

	outp32(addr, mipidsi_vid_mode_cfg.ul32);
}

void set_MIPIDSI_MODE_CFG(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_MODE_CFG_ADDR;
	union MIPIDSI_MODE_CFG_UNION  mipidsi_mode_cfg;

	mipidsi_mode_cfg.ul32 = inp32(addr);
	mipidsi_mode_cfg.bits.cmd_video_mode = nVal;

	outp32(addr, mipidsi_mode_cfg.ul32);
}

void set_MIPIDSI_VID_PKT_SIZE(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_PKT_SIZE_ADDR;
	union MIPIDSI_VID_PKT_SIZE_UNION  mipidsi_vid_pkt_size;

	mipidsi_vid_pkt_size.ul32 = inp32(addr);
	mipidsi_vid_pkt_size.bits.vid_pkt_size = nVal;

	outp32(addr, mipidsi_vid_pkt_size.ul32);
}

void set_MIPIDSI_VID_HSA_TIME(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_HSA_TIME_ADDR;
	union MIPIDSI_VID_HSA_TIME_UNION  mipidsi_vid_hsa_time;

	mipidsi_vid_hsa_time.ul32 = inp32(addr);
	mipidsi_vid_hsa_time.bits.vid_hsa_time = nVal;

	outp32(addr, mipidsi_vid_hsa_time.ul32);
}

void set_MIPIDSI_VID_HBP_TIME(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_HBP_TIME_ADDR;
	union MIPIDSI_VID_HBP_TIME_UNION  mipidsi_vid_hbp_time;

	mipidsi_vid_hbp_time.ul32 = inp32(addr);
	mipidsi_vid_hbp_time.bits.vid_hbp_time = nVal;

	outp32(addr, mipidsi_vid_hbp_time.ul32);
}

void set_MIPIDSI_VID_HLINE_TIME(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_HLINE_TIME_ADDR;
	union MIPIDSI_VID_HLINE_TIME_UNION  mipidsi_vid_hline_time;

	mipidsi_vid_hline_time.ul32 = inp32(addr);
	mipidsi_vid_hline_time.bits.vid_hline_time = nVal;

	outp32(addr, mipidsi_vid_hline_time.ul32);
}

void set_MIPIDSI_VID_VSA_LINES(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_VSA_LINES_ADDR;
	union MIPIDSI_VID_VSA_LINES_UNION  mipidsi_vid_vsa_lines;

	mipidsi_vid_vsa_lines.ul32 = inp32(addr);
	mipidsi_vid_vsa_lines.bits.vsa_lines = nVal;

	outp32(addr, mipidsi_vid_vsa_lines.ul32);
}

void set_MIPIDSI_VID_VBP_LINES(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_VBP_LINES_ADDR;
	union MIPIDSI_VID_VBP_LINES_UNION  mipidsi_vid_vbp_lines;

	mipidsi_vid_vbp_lines.ul32 = inp32(addr);
	mipidsi_vid_vbp_lines.bits.vbp_lines = nVal;

	outp32(addr, mipidsi_vid_vbp_lines.ul32);
}

void set_MIPIDSI_VID_VFP_LINES(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_VFP_LINES_ADDR;
	union MIPIDSI_VID_VFP_LINES_UNION  mipidsi_vid_vfp_lines;

	mipidsi_vid_vfp_lines.ul32 = inp32(addr);
	mipidsi_vid_vfp_lines.bits.vfp_lines = nVal;

	outp32(addr, mipidsi_vid_vfp_lines.ul32);
}

void set_MIPIDSI_VID_VACTIVE_LINES(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_VID_VACTIVE_LINES_ADDR;
	union MIPIDSI_VID_VACTIVE_LINES_UNION mipidsi_vid_vactive_lines;

	mipidsi_vid_vactive_lines.ul32 = inp32(addr);
	mipidsi_vid_vactive_lines.bits.v_active_lines = nVal;

	outp32(addr, mipidsi_vid_vactive_lines.ul32);
}

void set_MIPIDSI_CMD_MODE_CFG_all_en_flag(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_CMD_MODE_CFG_ADDR;
	union MIPIDSI_CMD_MODE_CFG_UNION  mipidsi_cmd_mode_cfg;

	mipidsi_cmd_mode_cfg.ul32 = inp32(addr);

	mipidsi_cmd_mode_cfg.bits.gen_sw_0p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.gen_sw_1p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.gen_sw_2p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.gen_sr_0p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.gen_sr_1p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.gen_sr_2p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.gen_lw_tx  = nVal;
	mipidsi_cmd_mode_cfg.bits.dcs_sw_0p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.dcs_sw_1p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.dcs_sr_0p_tx = nVal;
	mipidsi_cmd_mode_cfg.bits.dcs_lw_tx  = nVal;
	mipidsi_cmd_mode_cfg.bits.max_rd_pkt_size = nVal;

	outp32(addr, mipidsi_cmd_mode_cfg.ul32);
}

void set_MIPIDSI_BTA_TO_CNT_bta_to_cnt(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_BTA_TO_CNT_ADDR;
	union MIPIDSI_BTA_TO_CNT_UNION  mipidsi_bta_to_cnt;

	mipidsi_bta_to_cnt.ul32 = inp32(addr);
	mipidsi_bta_to_cnt.bits.bta_to_cnt = nVal;

	outp32(addr, mipidsi_bta_to_cnt.ul32);
}

void set_MIPIDSI_PHY_TMR_CFG_max_rd_time(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_TMR_CFG_ADDR;
	union MIPIDSI_PHY_TMR_CFG_UNION  mipidsi_phy_tmr_cfg;

	mipidsi_phy_tmr_cfg.ul32 = inp32(addr);
	mipidsi_phy_tmr_cfg.bits.max_rd_time = nVal;

	outp32(addr, mipidsi_phy_tmr_cfg.ul32);
}

void set_MIPIDSI_PHY_TMR_CFG_phy_lp2hs_time(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_TMR_CFG_ADDR;
	union MIPIDSI_PHY_TMR_CFG_UNION  mipidsi_phy_tmr_cfg;

	mipidsi_phy_tmr_cfg.ul32 = inp32(addr);
	mipidsi_phy_tmr_cfg.bits.phy_lp2hs_time = nVal;

	outp32(addr, mipidsi_phy_tmr_cfg.ul32);
}

void set_MIPIDSI_PHY_TMR_CFG_phy_hs2lp_time(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_TMR_CFG_ADDR;
	union MIPIDSI_PHY_TMR_CFG_UNION  mipidsi_phy_tmr_cfg;

	mipidsi_phy_tmr_cfg.ul32 = inp32(addr);
	mipidsi_phy_tmr_cfg.bits.phy_hs2lp_time = nVal;

	outp32(addr, mipidsi_phy_tmr_cfg.ul32);
}

void set_MIPIDSI_GEN_HDR(u8 *dsi_base, u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_GEN_HDR_ADDR;

	if (dsi_base != NULL)
		addr = dsi_base + MIPIDSI_GEN_HDR_ADDR;

	outp32(addr, nVal);
}

void set_MIPIDSI_GEN_PLD_DATA(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_GEN_PLD_DATA_ADDR;

	outp32(addr, nVal);
}

void set_MIPIDSI_LPCLK_CTRL_auto_clklane_ctrl(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_LPCLK_CTRL_ADDR;
	union MIPIDSI_LPCLK_CTRL_UNION  mipidsi_lpclk_ctrl;

	mipidsi_lpclk_ctrl.ul32 = inp32(addr);
	mipidsi_lpclk_ctrl.bits.auto_clklane_ctrl = nVal;

	outp32(addr, mipidsi_lpclk_ctrl.ul32);
}

void set_MIPIDSI_LPCLK_CTRL_phy_txrequestclkhs(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_LPCLK_CTRL_ADDR;
	union MIPIDSI_LPCLK_CTRL_UNION  mipidsi_lpclk_ctrl;

	mipidsi_lpclk_ctrl.ul32 = inp32(addr);
	mipidsi_lpclk_ctrl.bits.phy_txrequestclkhs = nVal;

	outp32(addr, mipidsi_lpclk_ctrl.ul32);
}

void set_MIPIDSI_PHY_RSTZ(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_RSTZ_ADDR;

	outp32(addr, nVal);
}

void set_MIPIDSI_PHY_IF_CFG_n_lanes(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_IF_CFG_ADDR;
	union MIPIDSI_PHY_IF_CFG_UNION  mipidsi_phy_if_cfg;

	mipidsi_phy_if_cfg.ul32 = inp32(addr);
	mipidsi_phy_if_cfg.bits.n_lanes = nVal;
	mipidsi_phy_if_cfg.bits.phy_stop_wait_time = 0x30;

	outp32(addr, mipidsi_phy_if_cfg.ul32);
}

void set_MIPIDSI_PHY_TST_CTRL0(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_TST_CTRL0_ADDR;

	outp32(addr, nVal);
}

void set_MIPIDSI_PHY_TST_CTRL1(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_TST_CTRL1_ADDR;

	outp32(addr, nVal);
}

void set_MIPIDSI_EDPI_CMD_SIZE(u32 nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_EDPI_CMD_SIZE_ADDR;
	union MIPIDSI_EDPI_CMD_SIZE_UNION  mipidsi_edpi_cmd_size;

	mipidsi_edpi_cmd_size.ul32 = inp32(addr);
	mipidsi_edpi_cmd_size.bits.edpi_allowed_cmd_size = nVal;

	outp32(addr, mipidsi_edpi_cmd_size.ul32);
}

void set_MIPIDSI_PHY_TMR_LPCLK_CFG_phy_clklp2hs_time(unsigned int nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_TMR_LPCLK_CFG_ADDR;
	union MIPIDSI_PHY_TMR_LPCLK_CFG_UNION  mipidsi_phy_tmr_lpclk_cfg;

	mipidsi_phy_tmr_lpclk_cfg.ul32 = inp32(addr);
	mipidsi_phy_tmr_lpclk_cfg.bits.phy_clklp2hs_time = nVal;
	outp32(addr, mipidsi_phy_tmr_lpclk_cfg.ul32);
}

void set_MIPIDSI_PHY_TMR_LPCLK_CFG_phy_clkhs2lp_time(unsigned int nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_PHY_TMR_LPCLK_CFG_ADDR;
	union MIPIDSI_PHY_TMR_LPCLK_CFG_UNION  mipidsi_phy_tmr_lpclk_cfg;

	mipidsi_phy_tmr_lpclk_cfg.ul32 = inp32(addr);
	mipidsi_phy_tmr_lpclk_cfg.bits.phy_clkhs2lp_time = nVal;
	outp32(addr, mipidsi_phy_tmr_lpclk_cfg.ul32);
}

void set_MIPIDSI_PHY_TMR_clk_to_data_delay(unsigned int nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_NO_CONTINUE_ADDR;
	union MIPIDSI_PHY_CLK_DATA_TMG_CFG_UNION  mipidsi_phy_clk_data_tmg_cfg;

	mipidsi_phy_clk_data_tmg_cfg.ul32 = inp32(addr);
	mipidsi_phy_clk_data_tmg_cfg.bits.clk_to_data_delay = nVal;
	outp32(addr, mipidsi_phy_clk_data_tmg_cfg.ul32);
}

void set_MIPIDSI_PHY_TMR_data_to_clk_delay(unsigned int nVal)
{
	u8 *addr = get_dsi_virtual_addr() + MIPIDSI_NO_CONTINUE_ADDR;
	union MIPIDSI_PHY_CLK_DATA_TMG_CFG_UNION  mipidsi_phy_clk_data_tmg_cfg;

	mipidsi_phy_clk_data_tmg_cfg.ul32 = inp32(addr);
	mipidsi_phy_clk_data_tmg_cfg.bits.data_to_clk_delay = nVal;
	outp32(addr, mipidsi_phy_clk_data_tmg_cfg.ul32);
}
