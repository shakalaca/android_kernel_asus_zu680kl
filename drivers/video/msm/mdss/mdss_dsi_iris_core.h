/* Copyright (c) 2013, Pixelworks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef MDSS_DSI_IRIS_CORE_H
#define MDSS_DSI_IRIS_CORE_H

#include "mdss_mdp.h"
#include "mdss_dsi.h"
#include "linux/fb.h"
#include <linux/types.h>

#define MIPI_SWAP
#define DSI_BUF_SIZE	1024

#define IRIS_DTG_ADDR	0xF1200000
#define IRIS_DPORT_ADDR	0xF1220000
#define IRIS_PWIL_ADDR	0xF1240000
#define IRIS_PSR_MIF_ADDR	0xF1400000
#define IRIS_PSR_COMP_ADDR	0xFF420000
#define IRIS_BLENDING_ADDR	0xF1520000
#define IRIS_FRC_ADDR	0xFF800000
#define IRIS_COMP_ADDR	0xFF880000
#define IRIS_GMD_ADDR	0xFF8A0000
#define IRIS_FBD_ADDR	0xFF8C0000
#define IRIS_CAD_ADDR	0xFF8E0000
#define IRIS_DECOMP0_ADDR	0xFF9A0000
#define IRIS_DECOMP1_ADDR	0xFF9C0000
#define FRCC_CTRL_REG5_ADDR		0xF2010014
#define FRCC_CTRL_REG7_ADDR     0xF201001C
#define FRCC_CTRL_REG8_ADDR		0xF2010020
#define FRCC_CTRL_REG16_ADDR	0xF2010040
#define FRCC_CTRL_REG17_ADDR	0xF2010044
#define FRCC_CTRL_REG18_ADDR	0xF2010048
#define FRCC_CMD_MOD_TH			0xF201004c
#define FRCC_REG_SHOW       0xF2011198
#ifdef MIPI_SWAP
#define IRIS_MIPI_RX_ADDR	0xF0140000
#define IRIS_MIPI_TX_ADDR	0xF01c0000
#else
#define IRIS_MIPI_RX_ADDR	0xF0100000
#define IRIS_MIPI_TX_ADDR	0xF0180000
#endif
#define IRIS_PROXY_ADDR	0xF0040000
#define IRIS_SYS_ADDR		0xF0000000
#define PWIL_STATUS_ADDR	0xF1240080
#define IRIS_PWIL_OUT_FRAME_SHIFT 24
#define IRIS_PWIL_IN_FRAME_SHIFT 8
#define IRIS_MVC_ADDR 0xF2100000
#define IRIS_BLC_PWM_ADDR 0xF10C0000
#define SCALE 0x1D0
#define FI_RANGE_CTRL          0xf2160014
#define FI_DEMO_COL_SIZE       0xf2160018
#define FI_DEMO_MODE_CTRL      0xf216001c
#define FI_DEMO_MODE_RING      0xf2160020
#define FI_DEMO_ROW_SIZE       0xf2160024
#define FI_SHADOW_UPDATE       0xf217ff00
#define PEAKING_CTRL           0xf1a0005c
#define PEAKING_STARTWIN       0xf1a00060
#define PEAKING_ENDWIN         0xf1a00064
#define PEAKING_SHADOW_UPDATE  0xf1a1ff00
#define UNIT_CONTRL_ADDR       0xf0060000

#define IRIS_FIRMWARE_NAME	"iris2.fw"

#define DSI_DMA_TX_BUF_SIZE	SZ_64K
#define FW_DW_CNT_IRIS2 (240)

#define ALIGN_UP(x, size)	(((x)+((size)-1))&(~((size)-1)))

//PWIL View Descriptor Valid Word Number
#define PWIL_ViewD_LEN 0x0A
//PWIL Display Descriptor Valid Word Number
#define PWIL_DispD_LEN 0x05

// bypass iris when sent light-up-commands
#define IRIS_BYPASS
#define REF_PLL_19_2_MHZ    //19.2 Mhz
// demo board1 need to comment following efuse rewrite
#define EFUSE_REWRITE
// panel will be light up in MCU mode
#define POWER_MODE_SLEEP 0x10
#define POWER_MODE_DISPLAY_NORMAL 0x8
#define POWER_MODE_DISPLAY_ON 0xC
#define POWER_MODE_ZERO 0

#define DGRESULT_IRIS_PANEL_ON 0x2
#define DGRESULT_FW_DOWNLOAD_BEGIN 0x4
#define DGRESULT_FW_DOWNLOAD_FINISH 0x5
#define DGRESULT_FW_DOWNLOAD_MIFUNIDLE 0x6
#define DGRESULT_FW_REMAP_SUCCESS 0xA
#define DGRESULT_IRIS_INFO 0xE

#define FW_DOWNLOAD_FINISH 1
#define FW_DOWNLOAD_RETRY   2
#define FW_DOWNLOAD_ERR   3

#define IRIS_WAKEUP_RETRYCNT_MAX 3
#define FW_DOWNLOAD_RETRYCNT_MAX 3

#define IRIS_CONFIGURE_GET_VALUE_CORRECT 0
#define IRIS_CONFIGURE_GET_VALUE_ERROR 1

#define MDP_IRIS_MODE_RFB				0x0
#define MDP_IRIS_MODE_FRC_PREPARE	0x1
#define MDP_IRIS_MODE_FRC_PREPARE_DONE	0x2
#define MDP_IRIS_MODE_FRC			0x3
#define MDP_IRIS_MODE_FRC_CANCEL	0x4
#define MDP_IRIS_MODE_FRC_PREPARE_RFB		0x5
#define MDP_IRIS_MODE_FRC_PREPARE_TIMEOUT	0x6

#define PWIL_TAG(a, b, c, d) d, c, b, a
#define PWIL_U32(x) \
	(__u8)(((x)) & 0xff), \
	(__u8)(((x) >>  8) & 0xff), \
	(__u8)(((x) >> 16) & 0xff), \
	(__u8)(((x) >> 24) & 0xff)

#define PWIL_U16(x) \
	(__u8)(((x)) & 0xff), \
	(__u8)(((x) >> 8) & 0xff)


#define PWIL_CHECK_FORMAT(cmds)	\
	do {	\
		int valid_word_num = (ARRAY_SIZE(cmds) - 12) / 4; \
		int non_burst_len = valid_word_num - 1; \
		if (!strncmp(cmds, "LIWP", 4)) { \
			if (!strncmp(cmds + 4, "PCRG", 4)) { \
				cmds[8] = valid_word_num & 0xFF; \
				cmds[9] = (valid_word_num >> 8) & 0xFF; \
				cmds[10] = (valid_word_num >> 16) & 0xFF; \
				cmds[11] = (valid_word_num >> 24) & 0xFF; \
				cmds[14] = non_burst_len & 0xFF; \
				cmds[15] = (non_burst_len >> 8) & 0xFF; \
			} else if (!strncmp(cmds + 4, "WEIV", 4)) { \
				cmds[8] = PWIL_ViewD_LEN & 0xFF; \
				cmds[9] = (PWIL_ViewD_LEN >> 8) & 0xFF; \
				cmds[10] = (PWIL_ViewD_LEN >> 16) & 0xFF; \
				cmds[11] = (PWIL_ViewD_LEN >> 24) & 0xFF; \
			} else if (!strncmp(cmds + 4, "PSID", 4)) { \
				cmds[8] = PWIL_DispD_LEN & 0xFF; \
				cmds[9] = (PWIL_DispD_LEN >> 8) & 0xFF; \
				cmds[10] = (PWIL_DispD_LEN >> 16) & 0xFF; \
				cmds[11] = (PWIL_DispD_LEN >> 24) & 0xFF; \
			} else { \
				\
			} \
		} else { \
			pr_err("PWIL Packet format error!\n"); \
		} \
	} while (0)

enum iris_mipirx_mode_enum {
	IRIS_MIPIRX_VIDEO = 0x0,
	IRIS_MIPIRX_CMD = 0x01,
};

struct  iris_timing_para {
	u16 hfp;
	u16 hres;
	u16 hbp;
	u16 hsw;
	u16 vfp;
	u16 vres;
	u16 vbp;
	u16 vsw;
};

struct  iris_dsc_para {
	u16 slice_number;
	u16 slice_height;
	u16 bpp;
};

struct iris_mipitx_config {
	u32 dpi_res;
	u32 hsync_count;
	u32 hbp_count;
	u32 hfp_count;
	u32 h_res;
	u32 vsync_count;
	u32 vbp_count;
	u32 vfp_count;
	u32 v_res;
};

struct  iris_mipi_param_calc {
	u8 trim1_divider_ratio;
	u16 ratio_panel_iris;
};

enum iris_work_mode {
	IRIS_PT_MODE = 0,
	IRIS_PSR_MODE,
	IRIS_MEMC_MODE,
	IRIS_FBO_MODE,
};

enum iris_cmdmode_cmds {
	PANEL_ON_CMDS = 0x01,	// means panel_videomode_on_cmds
	PANEL_OFF_CMDS = 0x02,	// means panel_videomode_off_cmds
	MIPIRX_CMDMODE_CMDS = 0x03,	// means mipirx_cmdmode_cmds
};
struct mipi_mode_t {
	u32 rx_mode:1;    // 0-video/1-cmd
	u32 rx_ch:1;    // 0-single/1-dual
	u32 rx_dsc:1;    // 0-non DSC/1-DSC
	u32 bypass_en:1;   // 0-PT mode/ 1-bypass mode
	u32 rx_pxl_mode:1;
	u32 reversed0:11;

	u32 tx_mode:1;    // 0-video/1-cmd
	u32 tx_ch:1;    // 0-single/1-dual
	u32 tx_dsc:1;    // 0-non DSC/1-DSC
	u32 tx_pxl_mode:1;
	u32 te_120_to_60:1;	//half te frequency
	u32 reversed1:11;
};

// be used by iris2 & command mode
struct iris_mipi {
	struct dsi_panel_cmds panel_videomode_on_cmds[2];
	struct dsi_panel_cmds panel_videomode_off_cmds[2];
	struct dsi_panel_cmds mipirx_cmdmode_cmds;
	struct mipi_mode_t mipi_mode;
	struct iris_timing_para iris_in_timing;
	struct iris_timing_para iris_out_timing;
	struct iris_dsc_para iris_in_dsc;
	struct iris_dsc_para iris_out_dsc;
	u8 iris_timing_flag;
	int delta_period_max;
	int delta_period_min;
};

enum iris_config_type {
	IRIS_PEAKING,		// MB3[3:0] | [24]
	IRIS_SHARPNESS,		// MB3[7:4] | [25]
	IRIS_MEMC_DEMO,		// MB3[9:8] | [26]
	IRIS_PEAKING_DEMO,	// MB3[11:10] | [27]
	IRIS_GAMMA,	// MB3[13:12] | [28]
	IRIS_MEMC_LEVEL,	// MB3[15:14] | [29]
	IRIS_CONTRAST,		// MB3[23:16] | [30]
	IRIS_BRIGHTNESS,	// MB5[14:8]
	IRIS_EXTERNAL_PWM,	// MB5[15]
	IRIS_DBC_QUALITY,	// MB5[19:16]
	IRIS_DLV_SENSITIVITY,	// MB5[31:20]
	IRIS_DBC_CONFIG,	// MB5
	IRIS_PQ_CONFIG,		// MB3
	IRIS_MEMC_ENABLE,	//0: OFF, 1: ON
	IRIS_MEMC_OPTION,	//0: PANEL_RESOLUTION, 1: NATIVE_RESOLUTION, 2: LOW_POWER
	IRIS_LPMEMC_CONFIG,
	IRIS_DCE_LEVEL,			// MB5[7:4]
	IRIS_USER_DEMO_WND,
	IRIS_MEMC_ACTIVE,       // read only
	IRIS_WHITE_LIST_ADD,    // add white list
	IRIS_WHITE_LIST_RST,    // reset white list
	IRIS_BLACK_LIST_ADD,    // add black list
	IRIS_BLACK_LIST_RST,    // reset black list
	IRIS_COLOR_ADJUST,
	IRIS_LAYER_SIZE,        // layer size
	IRIS_BLACK_BORDER,      // 1: has black border, 0: no black border
	IRIS_CONFIG_TYPE_MAX
};

enum iris_memc_option {
	IRIS_MEMC_PANEL_RESOLUTION,
	IRIS_MEMC_NATIVE_RESOLUTION,
	IRIS_MEMC_LOW_POWER,
};

struct iris_pq_setting {
	uint32_t peaking:4;
	uint32_t sharpness:4;
	uint32_t memcDemo:2;
	uint32_t peakingDemo:2;
	uint32_t gamma:2;
	uint32_t memcLevel:2;
	uint32_t contrast:8;
	uint32_t peakingUpdate:1;
	uint32_t sharpnessUpdate:1;
	uint32_t memcDemoUpdate:1;
	uint32_t peakingDemoUpdate:1;
	uint32_t gammeUpdate:1;
	uint32_t memcLevelUpdate:1;
	uint32_t contrastUpdate:1;
	uint32_t reserved:1;
};

struct iris_dbc_setting {
	uint32_t dbcUpdate:1;
	uint32_t reserved:3;
	uint32_t DCELevel:4;
	uint32_t brightness:7;
	uint32_t externalPWM:1;
	uint32_t dbcQuality:4;
	uint32_t dlvSensitivity:12;
};

struct iris_config_setting {
	int    update;
	u8	level;
	union {
		uint32_t value;
		struct iris_pq_setting pqSetting;
	};
};

void mdss_dsi_iris_init(struct msm_fb_data_type *mfd);
void iris_mipi_pt_enter(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_mipi_pwil(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_mipi_mcu(struct mdss_dsi_ctrl_pdata *ctrl);
#ifdef REF_PLL_19_2_MHZ
void iris_set_sys_efuse(struct mdss_dsi_ctrl_pdata *ctrl);
#endif
void iris_init_info_send(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_panel_on_start(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_extra_info_set(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_panel_on_finish(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_panel_off_start(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_panel_off_finish(struct mdss_dsi_ctrl_pdata *ctrl);
u8 iris_mipi_check_power_mode(struct mdss_dsi_ctrl_pdata *ctrl);
u8 iris_mipi_wakeup(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_params_parse(struct device_node *np, struct mdss_panel_info *panel_info, struct  iris_mipi_param_calc *iris_param);
void mdss_dsi_parse_iris_mipi(struct device_node *np, char mode, u32 panel_destination);
void iris_cmds(struct mdss_dsi_ctrl_pdata *ctrl, u8 cflag, int dsiIndex);
void iris_lightup_mode(struct mdss_dsi_ctrl_pdata *ctrl);
void iris_mipi_bypass_ex(struct mdss_dsi_ctrl_pdata *ctrl);
void mdss_dsi_iris_init_ctl(struct mdss_mdp_ctl *ctl);
void mdss_dsi_iris_init_pdata(struct mdss_panel_data *pdata);

#endif //MDSS_DSI_IRIS_CORE_H
