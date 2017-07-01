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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/msm_mdp.h>
#include <linux/gpio.h>
#include <linux/circ_buf.h>
#include <linux/gcd.h>
#include <linux/uaccess.h>
#include <linux/clk.h>

#include "mdss_mdp.h"
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_debug.h"
#include "mdss_dsi_iris.h"
#include <linux/vmalloc.h>

#define DSI_VIDEO_BASE 0xE0000
#define DCS_WRITE_MEM_START 0x2C
#define DCS_WRITE_MEM_CONTINUE 0x3C

//if it use debug info should open DEBUG, or not DEBUG info
//#define DEBUG

//#define FPGA_DEBUG
#ifdef FPGA_DEBUG
#define WAKEUP_TIME 500
#define CMD_PROC 10
#define INIT_INT 100
#define INIT_WAIT 100
#else
#define WAKEUP_TIME 50
#define CMD_PROC 2
#define INIT_INT 100
#define INIT_WAIT 20
#endif

#define IRIS_RETRY 5
#define RESEND_TIMES 5
#define IRIS2_RESEND_TIMES 2
// assume 1 entry 1 ms
#define IRIS_CMD_FIFO_EMPTY 16
#define IRIS_REPEAT_NO     0
#define IRIS_REPEAT_FORCE  1
#define IRIS_REPEAT_CAPDIS 2

extern int pw_iris2_status;
extern int entry_mode;
extern void iris2_io_bimc_enable(int on);
extern void iris2_io_snoc_enable(int on);
void iris_disable_pwil_capen(struct mdss_dsi_ctrl_pdata *ctrl);

// iris2, iris workmode
static char iris_mipi_mode[] = {
	0x80, 0x87, 0x0, 0x3,
	PWIL_U32(0x0)
};
struct iris_mipi iris_mipi_info;

struct msm_fb_data_type *g_mfd;
struct mdss_dsi_ctrl_pdata *g_dsi_ctrl;
struct mdss_mdp_ctl *g_ctl0, *g_ctl1;
struct mdss_panel_data *g_pdata0, *g_pdata1;
static u8 *g_firmware_buf;

#define ENABLE_SUPPORT_MEMC_WITH_GRID_LINE
#define DEBUG_PATTERN_DATA_INTEGRITY
static u16 *g_osd_buf = NULL;
static int f_grid_size = 3;
static u16 f_gride_line_color = 0xffff;
static int f_camera_memc_enable = 0;
static struct osd_pattern_info f_osd_pattern_info;
bool g_sendframe = false;
bool g_gridpattern_downloaded = false;
bool g_gridpattern_showing = false;
bool g_gridpattern_show_required = false;


static u32 f_Iris_ip_reg_addr = IRIS_PWIL_ADDR;
static u32 f_Iris_ip_reg_addr_value = 0;

/* Per Panel different */
#define IRIS_DTG_EVS_DLY   124
#define IRIS_DTG_E2OVS_DLY 4
#define IRIS_DTG_FI_PRELOAD_DLY  124
#define IRIS_DTG_EVS_NEW_DLY  1

/* debug send meta */
static bool debug_send_meta_enabled = 1;
/* debug new frame flag in video mode */
static bool debug_new_frame_enabled = 1;
/* debug in/out ratio flag */
static bool debug_ratio_enabled = 1;
static bool debug_hlmd_enabled;
/* debug repeat flag */
static u8 debug_repeat_enabled = 1;
/* debug te flag */
static bool debug_te_enabled = 1;
/* debug dtg */
static bool debug_dtg_enabled;
static bool frc_repeat_enter;
static int debug_new_repeat = 1;
/* debug send mode switch */
static bool debug_mode_switch_enabled = 1;
static int rfb_delay;
static int frc_delay;
static int prep_delay;
/* debug usb workaround */
static bool debug_usb_w_enabled = 0; //disable, use ot_lim to instead
static bool usb_w_enabled_by_configure = 0;

static bool debug_clock_gate = 1;

#if defined(CONFIG_IRIS2_DRC_SUPPORT)
/* drc scaling feature */
uint32_t guFrcLPTiming = 0;
#endif
struct iris_mgmt_t {
	struct work_struct iris_worker;
	struct workqueue_struct *iris_wq;
	void (*iris_handler)(void);
	bool fbo_enable;
	bool sbs_enable;
	struct msm_fb_data_type *mfd;
};
static struct iris_mgmt_t iris_mgmt;
// FIXME mdp5 use add vsync handler and no find DMA_P bit
//void mdp4_dsi_video_wait4dmap_done(int cndx);

/* Activate Delay 0, FBO Enable: 0, Display Mode: Normal,
* PSR Command: PSR Enable, Capture Enable: -
*/
static int first_boot = 1;
static char pt_enable[2] = { 0x00, 0x00 };
static char memc_enable[2] = {0x04, 0x2};

static struct iris_pq_setting pq_setting_current = {
	.peaking = 0,
	.memcDemo = 0,
	.peakingDemo = 0,
	.memcLevel = 3,
	.contrast = 50,
	.peakingUpdate = 1,
	.memcDemoUpdate = 1,
	.peakingDemoUpdate = 1,
	.memcLevelUpdate = 1,
	.contrastUpdate = 1,
	.cinema = 0,
};
static struct iris_dbc_setting dbc_setting_current = {
	.dbcUpdate = 1,
	.DCELevel = 0,
	.dbcQuality = 5,
	.dlvSensitivity = 0,
};
static struct iris_config_setting LPMemc_setting_current = {
	.level = 0,
};
static struct demo_win_info_for_FI demowinFI_setting_default = {0};
static struct demo_win_info demo_win_info_setting = { 0 };
static bool demo_win_FI_update;
static bool pq_setting_update;
static bool dbc_setting_update;
static bool LPMemc_setting_update;
static bool black_border_value;
static bool black_border_update;
static u8 iris_dbc_mode;
static u8 color_adjust_current_value = 50;
static bool color_adjust_update;

static u32 m_fiVrangeTop = 0xa;

static struct dsi_cmd_desc iris_pt_enable_cmd[] = {
	{ { DTYPE_GEN_WRITE2,  1, 0, 0, 0, sizeof(pt_enable) }, pt_enable },
};

/* Activate Delay 0, FBO Enable: 1, Display Mode: FRC Enable,
* PSR Command: PSR update, Capture Enable: Video
*/
static char fbo_update[2] = {0x15, 0x02};

#define META_PKT_SIZE 512
#define META_HEADER 16
static char imeta_header[META_HEADER] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x3),
	0x00,
	0x00,
	PWIL_U16(0x2),
};
static char imeta[META_PKT_SIZE] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x3),
	0x00,
	0x00,
	PWIL_U16(0x2),
};
//iris_meta_pkts[1] will also be updated  by memc_enable
static struct dsi_cmd_desc iris_meta_pkts[] = {
	{{DTYPE_GEN_LWRITE, 0, 0, 0, 0, sizeof(imeta)}, imeta},
	{{ DTYPE_GEN_WRITE2, 0, 0, 0, 0, sizeof(fbo_update) }, fbo_update },
};

static char iris_bypass_mode[1] = {0xff};

static char iris_pwil_mode[1] = {0xbf};

static struct dsi_cmd_desc iris_mipi_pwil_cmds[] = {
	{ { DTYPE_GEN_WRITE1, 1, 0, 0, 1,   sizeof(iris_pwil_mode) }, iris_pwil_mode},
};

static char iris_mcu_mode[1] = {0x3f};

static struct dsi_cmd_desc iris_mipi_mcu_cmds[] = {
	{ { DTYPE_GEN_WRITE1, 1, 0, 0, 1,   sizeof(iris_mcu_mode) }, iris_mcu_mode},
};

static char iris_pt_enter_cmds[] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x00000005),	//valid word number
	0x00,					//burst mode
	0x00,					//reserved
	PWIL_U16(0x0004),		//burst length
	PWIL_U32(IRIS_PROXY_ADDR + 0x10),	//proxy MB2
	PWIL_U32(0x800000),
	PWIL_U32(IRIS_PROXY_ADDR + 0x08), //proxy MB1
	PWIL_U32(0x800000)
};

static char iris_memc_enter_cmds[] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x00000005),	//valid word number
	0x00,					//burst mode
	0x00,					//reserved
	PWIL_U16(0x0004),		//burst length
	PWIL_U32(IRIS_PROXY_ADDR + 0x10),	//proxy MB2
	PWIL_U32(0x800000),
	PWIL_U32(IRIS_PROXY_ADDR + 0x08), //proxy MB1
	PWIL_U32(0x800002)
};

/*static char iris_memc_cancel_cmds[] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x00000003),	//valid word number
	0x00,					//burst mode
	0x00,					//reserved
	PWIL_U16(0x0002),		//burst length
	PWIL_U32(IRIS_PROXY_ADDR + 0x08), //proxy MB1
	PWIL_U32(0x40000002)
};*/

static struct dsi_cmd_desc pt_data_path_config[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(iris_pt_enter_cmds) }, iris_pt_enter_cmds},
};

static struct dsi_cmd_desc pt_mode_enter[] = {
	{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(pt_enable) }, pt_enable},

};

static struct dsi_cmd_desc memc_data_path_config[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(iris_memc_enter_cmds) }, iris_memc_enter_cmds},
};

static struct dsi_cmd_desc memc_mode_enter[] = {
	{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(memc_enable) }, memc_enable},
};

/*static struct dsi_cmd_desc memc_cancel[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(iris_memc_cancel_cmds) }, iris_memc_cancel_cmds},
};*/

/* Assume the panel can accept this period increase */
#define IRIS_DELTA_VT_P        (31)
/* VFP needs at least 2 lines VSW needs at least 1 line */
#define IRIS_DELTA_VT_M        (-4)

static char panel_config_startflag[4] = {0x80, 0x87, 0x1, 0x0};
static char panel_config_finishflag[4] = {0x80, 0x87, 0x2, 0x0};
static char panel_off_flag[4] = {0x80, 0x87, 0x3, 0x0};
static char appcode_download_startflag[4] = {0x80, 0x87, 0x4, 0x0};
static char appcode_download_finishflag[4] = {0x80, 0x87, 0x5, 0x0};

// temp, need to be update
static char panel_info_to_iris[] = {
	0x80, 0x87, 0x0, 0x0,
	PWIL_U32(0x01e00014),
	PWIL_U32(0x00160010),
	PWIL_U32(0x0320000a),
	PWIL_U32(0x00080008),
	PWIL_U32(0x1f),
	PWIL_U32(0x01e00014),
	PWIL_U32(0x00160010),
	PWIL_U32(0x0320000a),
	PWIL_U32(0x00080008),
	PWIL_U32(0x1f),
	PWIL_U32(0x00100008),
	PWIL_U32(0x80),
	PWIL_U32(0x00100008),
	PWIL_U32(0x80)
};

static char iris_code_size[] = {
	0x80, 0x87, 0x0, 0x1,
	// tcm clip of appcode
	PWIL_U32(IRIS_PROXY_ADDR + 0x38),
	PWIL_U32(0x0001e000),
};

static char mipirx_phy_reset[] = {
	0x80, 0x87, 0x0, 0x2,
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20030),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20030),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20030),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20030),
	PWIL_U32(0x00000001)
};

static char mipirx_info_to_iris[] = {
	0x80, 0x87, 0x0, 0x2,
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20000),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x0000c),
	PWIL_U32(0x000f0000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x00014),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x00018),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x2000c),
	PWIL_U32(0x00000422),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x2002c),
	PWIL_U32(0x0000ff04),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20010),
	PWIL_U32(0xffffffff),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x2001C),
	PWIL_U32(0x00000546),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20058),
	PWIL_U32(0x00080016),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x2005C),
	PWIL_U32(0x00000546),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x1004),	//DMAWC_BASE_ADDR
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x1008),	//DMARC_BASE_ADDR
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x1048),	//MIPI_MIF_HSTRIDE
	PWIL_U32(0x00000020),
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x1ffe8),
	PWIL_U32(0x00000043),
#ifndef ENABLE_IRIS2_480X800_PANEL
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20024),
	PWIL_U32(0x00000001),
#endif
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x20000),
	PWIL_U32(0x00000001),
};

static char mipitx_info_to_iris[] = {
	0x80, 0x87, 0x0, 0x2,
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20000),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20074),
	PWIL_U32(0x00000001),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2000c),
	PWIL_U32(0x00002202),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20010),
	PWIL_U32(0x00ffffff),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20014),
	PWIL_U32(0x20),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20018),
	PWIL_U32(0x0000001f),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2001C),
	PWIL_U32(0x000000ff),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20020),
	PWIL_U32(0x032001e0),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20028),
	PWIL_U32(0x00000022),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2002C),
	PWIL_U32(0x00000019),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20030),
	PWIL_U32(0x0000001f),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20034),
	PWIL_U32(0x000002da),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20038),
	PWIL_U32(0x00000008),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2003C),
	PWIL_U32(0x00000008),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20040),
	PWIL_U32(0x0000000a),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20044),
	PWIL_U32(0x00000014),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2004C),
	PWIL_U32(0x00000546),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20050),
	PWIL_U32(0x00000546),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20058),
	PWIL_U32(0x00000003),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2005C),
	PWIL_U32(0x00000006),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20060),
	PWIL_U32(0x00000003),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20064),
	PWIL_U32(0x001b0009),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20068),
	PWIL_U32(0x00000003),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2006C),
	PWIL_U32(0x04020400),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20070),
	PWIL_U32(0x04020a01),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x2007C),
	PWIL_U32(0xeeb5384c), // TRIM1[25:23] from 011 to 101 for MIPI TX0&1 to enlarge margin on MIPI LP mode with lower power supply like 1.1V, 1.045V
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x20000),
	PWIL_U32(0x00000001),
};


#ifdef REF_PLL_19_2_MHZ
static char efuse_ctrl_to_iris[] = {
	0x80, 0x87, 0x0, 0x1,
	PWIL_U32(IRIS_SYS_ADDR + 0x10000),
	PWIL_U32(0x80000104),
	PWIL_U32(IRIS_SYS_ADDR + 0x1c),
	PWIL_U32(0x1),
	PWIL_U32(IRIS_SYS_ADDR + 0x1c),
	PWIL_U32(0x3),
	PWIL_U32(IRIS_SYS_ADDR + 0x1c),
	PWIL_U32(0x2)
};
#endif

// iris dpll config
#ifdef REF_PLL_19_2_MHZ
static char dpll_info_to_iris[] = {
	0x80, 0x87, 0x0, 0x1,
	PWIL_U32(IRIS_SYS_ADDR + 0x218),
	PWIL_U32(0x0),
	PWIL_U32(IRIS_SYS_ADDR + 0x10),
	PWIL_U32(0x1),
	PWIL_U32(IRIS_SYS_ADDR + 0x10),
	PWIL_U32(0x0),
	PWIL_U32(IRIS_SYS_ADDR + 0x150),
	PWIL_U32(0x1a),
	PWIL_U32(IRIS_SYS_ADDR + 0x154),
	PWIL_U32(0x211301),
	PWIL_U32(IRIS_SYS_ADDR + 0x158),
	PWIL_U32(0x0),
	PWIL_U32(IRIS_SYS_ADDR + 0x104),
	PWIL_U32(0x00561301),
	PWIL_U32(IRIS_SYS_ADDR + 0x144),
	PWIL_U32(0x221201),//0x381201 for 2560x1600(270M), for 1080p used 0x221201(163M)
	PWIL_U32(IRIS_SYS_ADDR + 0x148),
	PWIL_U32(0x3fffdb),
	PWIL_U32(IRIS_SYS_ADDR + 0x200),
	PWIL_U32(0x1400e),
	PWIL_U32(IRIS_SYS_ADDR + 0x200),
	PWIL_U32(0x14000),
	PWIL_U32(IRIS_PSR_MIF_ADDR + 0x0),
	PWIL_U32(0x0320ee14),
	PWIL_U32(IRIS_PSR_MIF_ADDR + 0x1ff00),
	PWIL_U32(0x100),
	PWIL_U32(IRIS_SYS_ADDR + 0x218),
	PWIL_U32(0x1),
	PWIL_U32(IRIS_SYS_ADDR + 0x10),
	PWIL_U32(0x1),
	PWIL_U32(IRIS_SYS_ADDR + 0x10),
	PWIL_U32(0x0),
	// add TRIM1[25:23] from 011 to 101 setting for MIPI RX0
	// to enlarge margin on MIPI LP mode with lower power supply like 1.1V, 1.045V
	PWIL_U32(0xf0120038), //MIPI RX0 RX_AFE_TRIM_1
	PWIL_U32(0xeeb5384c),
};
#else
static char dpll_info_to_iris[] = {
	0x80, 0x87, 0x0, 0x1,
	PWIL_U32(IRIS_SYS_ADDR + 0x110),
	PWIL_U32(0x1a),
	PWIL_U32(IRIS_SYS_ADDR + 0x114),
	PWIL_U32(0x211301),
	PWIL_U32(IRIS_SYS_ADDR + 0x118),
	PWIL_U32(0x0),
	PWIL_U32(IRIS_SYS_ADDR + 0x200),
	PWIL_U32(0x14002),
	PWIL_U32(IRIS_SYS_ADDR + 0x200),
	PWIL_U32(0x14000),
	// add TRIM1[25:23] from 011 to 101 setting for MIPI RX0
	// to enlarge margin on MIPI LP mode with lower power supply like 1.1V, 1.045V
	PWIL_U32(0xf0120038), //MIPI RX0 RX_AFE_TRIM_1
	PWIL_U32(0xeeb5384c),
};
#endif

static struct dsi_cmd_desc iris_init_info[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC,
		sizeof(iris_mipi_mode) }, iris_mipi_mode},
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC * 3,
		sizeof(mipirx_phy_reset) }, mipirx_phy_reset},
	{ { DTYPE_GEN_LWRITE, 0, 0, 0, 0,
		sizeof(dpll_info_to_iris) }, dpll_info_to_iris},
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC * 3,
		sizeof(mipirx_info_to_iris) }, mipirx_info_to_iris},
	{ { DTYPE_GEN_LWRITE, 0, 0, 0, 0,
		sizeof(mipitx_info_to_iris) }, mipitx_info_to_iris},
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, INIT_WAIT,
		sizeof(panel_info_to_iris) }, panel_info_to_iris}
};

static struct dsi_cmd_desc panel_lightup_start[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC,
		sizeof(panel_config_startflag) }, panel_config_startflag},
};

static struct dsi_cmd_desc panel_lightup_finish[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, INIT_WAIT,
		sizeof(panel_config_finishflag) }, panel_config_finishflag},
};

static struct dsi_cmd_desc panel_off_start[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, INIT_WAIT,
		sizeof(panel_config_startflag) }, panel_config_startflag},
};

static struct dsi_cmd_desc panel_off_finish[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC,
		sizeof(panel_off_flag) }, panel_off_flag},
};
static struct dsi_cmd_desc appcode_download_start[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC,
		sizeof(appcode_download_startflag) }, appcode_download_startflag}
};

static struct dsi_cmd_desc appcode_download_finish[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC,
		sizeof(appcode_download_finishflag) }, appcode_download_finishflag}
};

static int iris_set_ratio(struct iris_config *iris_cfg);
static void iris_regs_clear(void);
static void iris_reg_add(u32 addr, u32 val);
static int iris_regs_meta_build(void);
static u32 iris_pi_read(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr);
static u32 iris_pi_write(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr, u32 value);
static void iris_fiSearchRangeTop(void);
static u32 iris_LowPowerMemcFrcCal(u32 value);

static void iris_dump_packet(u8 *data, u8 size)
{
	int i = 0;

	pr_debug("size: %i\n", size);
	for (i = 0; i < size; i += 4)
		pr_debug("0x%02x 0x%02x 0x%02x 0x%02x\n",
			*(data+i), *(data+i+1), *(data+i+2), *(data+i+3));
}

static void iris_mode_switch_reset(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	iris_cfg->mode_changed = false;
	iris_cfg->sf_notify_mode = MDP_IRIS_MODE_RFB;
	rfb_delay = 0;
	frc_delay = 0;
	prep_delay = 0;
	iris_cfg->iris_ratio_updated = false;
	iris_cfg->repeat = IRIS_REPEAT_NO;
	iris_cfg->check_appcode_rfb_ready = false;
	iris_cfg->check_pwil_rfb_ready = false;
}

void mdss_dsi_panel_cmds_send_ex(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	/*please pay attention to call this funtion, it only used to send write cmd when panel on/off*/
	pinfo = &(ctrl->panel_data.panel_info);
	/* TODO:
		Comment below code for partial update, no impact current system.
		If enable dcs_cmd_by_left, the Iris + panel can't light up.
		Need to debug later.
	*/

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;
	if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

void iris_panel_on_start(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	pr_info("send iris panel on start\n");
	iris_dump_packet(panel_config_startflag, sizeof(panel_config_startflag));
	panel_cmds.cmds = panel_lightup_start;
	panel_cmds.cmd_cnt = ARRAY_SIZE(panel_lightup_start);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_panel_off_start(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	iris_cfg->ready = false;
	iris_mode_switch_reset(ctrl);
	pr_debug("send iris panel off start\n");
	panel_cmds.cmds = panel_off_start;
	panel_cmds.cmd_cnt = ARRAY_SIZE(panel_off_start);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_panel_off_finish(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo = &(ctrl->panel_data.panel_info);
	struct dsi_panel_cmds panel_cmds;

	pr_debug("send iris panel off finish\n");

	panel_cmds.cmds = panel_off_finish;
	panel_cmds.cmd_cnt = ARRAY_SIZE(panel_off_finish);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	if (pinfo->iris_wakeup_gpio != -1) {
		pr_debug("standby by GPIO start\n");
		gpio_set_value(pinfo->iris_wakeup_gpio, 0);
		pr_debug("standby by GPIO finish\n");
	}
}

void iris_codedownload_start(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	pr_info("send iris download start flag\n");
	panel_cmds.cmds = appcode_download_start;
	panel_cmds.cmd_cnt = ARRAY_SIZE(appcode_download_start);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_codedownload_finish(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	pr_info("send iris download finish flag\n");
	panel_cmds.cmds = appcode_download_finish;
	panel_cmds.cmd_cnt = ARRAY_SIZE(appcode_download_finish);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

static char get_power_mode[1] = {0x0a};
static struct dsi_cmd_desc iris_power_mode_cmd = {
	{ DTYPE_DCS_READ, 1, 0, 1, 0, sizeof(get_power_mode) }, get_power_mode};
static u8 iris_power_mode;
char iris_read_buf[16];
static void iris_mipi_power_mode_cb(int len)
{
	if (len != 1)
		pr_err("%s: not short read responese, return len [%02x] !=1\n", __func__, len);

	iris_power_mode = (u8)iris_read_buf[0];
	pr_info("power mode [%02x]\n", iris_power_mode);
}

static u8 iris_mipi_power_mode(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dcs_cmd_req cmdreq;

	cmdreq.cmds = &iris_power_mode_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1; /* short read, can NOT set to zero */
	cmdreq.rbuf = iris_read_buf;
	cmdreq.cb = iris_mipi_power_mode_cb; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return iris_power_mode;
}

u8 iris_mipi_check_power_mode(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u8 i = 0;
	u8 powermode;
	//fpga mipi rx phy cannot return correct value for first three times. need to check on asic.
	do {
		i++;
		powermode = iris_mipi_power_mode(ctrl);
		if (powermode == 0x0) {
			// delay 5ms, to avoid send command to iris when iris mipi is being init.
			msleep(5);

		} else {
			break;
		}
	} while ((powermode == 0x0) && i < 20);

	return powermode;
}

static char wakeup[1] = {0x5};
static struct dsi_cmd_desc wakeup_cmd[] = {
	{ { DTYPE_GEN_WRITE1, 1, 0, 0, 10,   sizeof(wakeup) }, wakeup},
};

u8 iris_mipi_wakeup(struct mdss_dsi_ctrl_pdata *ctrl)
{
	u8 powermode;

	struct mdss_panel_info *pinfo = &(ctrl->panel_data.panel_info);
	struct dsi_panel_cmds panel_cmds;

	if (pinfo->iris_wakeup_gpio == -1) {
		pr_info("trigger wake up by MIPI_RX start\n");

		panel_cmds.cmds = wakeup_cmd;
		panel_cmds.cmd_cnt = ARRAY_SIZE(wakeup_cmd);
		panel_cmds.link_state = DSI_LP_MODE;
		mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
		pr_info("trigger wake up by MIPI_RX finish\n");
	} else {
		pr_info("wake up by GPIO start\n");
		gpio_set_value(pinfo->iris_wakeup_gpio, 1);
		pr_info("wake up by GPIO finish\n");
		//TODO efuse
	}

	powermode = iris_mipi_check_power_mode(ctrl);
	pr_info("check power mode finish\n");

	return powermode;
}

static void iris_update_configure(void)
{
	pr_debug("iris_update_configure enter\n");

	// There is no default settings, so we update all settings.
	pq_setting_current.peakingUpdate = 1;
	pq_setting_current.sharpnessUpdate = 1;
	if (pq_setting_current.memcDemo == 2) {
		//user define level
		demo_win_FI_update = true; //first panel on, enter MEMC setting
		pr_debug("iris: first time configure user demo window for MEMC setting ---\n");
	}
	pq_setting_current.memcDemoUpdate = 1;
	if (pq_setting_current.peakingDemo == 2) {
		//user define level
		iris_reg_add(PEAKING_STARTWIN, (demo_win_info_setting.startX & 0x3fff) + ((demo_win_info_setting.startY & 0x3fff) << 16));
		iris_reg_add(PEAKING_ENDWIN, (demo_win_info_setting.endX & 0x3fff) + ((demo_win_info_setting.endY & 0x3fff) << 16));
		iris_reg_add(PEAKING_CTRL, 1 | demo_win_info_setting.SharpnessEn<<1);
		iris_reg_add(PEAKING_SHADOW_UPDATE, 1);
		pr_debug("iris: first time configure user demo window for peaking setting ---\n");
	}
	pq_setting_current.peakingDemoUpdate = true;
	pq_setting_current.gammeUpdate = 1;
	pq_setting_current.memcLevelUpdate = 1;
	pq_setting_current.contrastUpdate = 1;
	pq_setting_update = true;

	dbc_setting_current.dbcUpdate = 1;
	dbc_setting_update = true;

	LPMemc_setting_current.value = iris_LowPowerMemcFrcCal(LPMemc_setting_current.level);
	LPMemc_setting_update = true;

	color_adjust_update = true;

	//update dbc mode
	if (dbc_setting_update) {
		if (dbc_setting_current.dbcQuality == 5)
			iris_dbc_mode &= ~(1 << 1);
		else
			iris_dbc_mode |= 1 << 1;
		if (dbc_setting_current.dlvSensitivity == 0)
			iris_dbc_mode &= ~1;
		else
			iris_dbc_mode |= 1;
	}
}

#define IRIS_DTG_INIT_VTOTAL 2071
static u8 iris_extra_info[] = {
	0x80, 0x87, 0x0, 0x1,
	PWIL_U32(IRIS_DTG_ADDR + 0x0038), //dtg te width
	PWIL_U32(0x001901e0),
	PWIL_U32(IRIS_DTG_ADDR + 0x001c),   // EVS_DLY: 124
	PWIL_U32(IRIS_DTG_EVS_DLY | (IRIS_DTG_E2OVS_DLY << 8)),
	PWIL_U32(IRIS_DTG_ADDR + 0x0070),   // TE2OVS_DLY: 90, near to EVS_DLY
	PWIL_U32(0x005a0000),
	PWIL_U32(IRIS_DTG_ADDR + 0x0034),   // TE_CTRL: TE_EN TE_SEL SW_TE_EN
	PWIL_U32(0xd | ((IRIS_DTG_INIT_VTOTAL - IRIS_DTG_EVS_DLY) << 16)),
	PWIL_U32(IRIS_DTG_ADDR + 0x0044),   // DTG_CTRL1: mode = 3 cmd = 1
	PWIL_U32(0x3c01),
	PWIL_U32(IRIS_DTG_ADDR + 0x0064),   // DVS_CTRL
	PWIL_U32(IRIS_DTG_INIT_VTOTAL << 8),
	PWIL_U32(IRIS_DTG_ADDR + 0x0018), //disp_int
	PWIL_U32(IRIS_DTG_FI_PRELOAD_DLY | (IRIS_DTG_EVS_NEW_DLY << 16)),
	PWIL_U32(IRIS_DTG_ADDR + 0x10000),
	PWIL_U32(0x00010000), //forceupdate
//mipi rx/tx ext
#ifdef MIPI_SWAP
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x0000c - 0x40000),
	PWIL_U32(0xcf0000),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x0000c - 0x40000),
	PWIL_U32(0x409620),
#else
	PWIL_U32(IRIS_MIPI_RX_ADDR + 0x4000c),
	PWIL_U32(0xcf0000),
	PWIL_U32(IRIS_MIPI_TX_ADDR + 0x4000c),
	PWIL_U32(0x409620),
#endif
};

void iris_extra_info_set(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;
	u16 uTeVdh = iris_mipi_info.iris_out_timing.vsw + iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp - 2;
	u16 uTeHdh = iris_mipi_info.iris_out_timing.hres;
	u32 uVtotal = iris_mipi_info.iris_out_timing.vsw + iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp + iris_mipi_info.iris_out_timing.vres;
	u32 uTemp;
	struct dsi_cmd_desc iris_extra_info_cmds[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC,  sizeof(iris_extra_info) },  iris_extra_info},
	};

	iris_extra_info[8] = uTeHdh & 0xff;
	iris_extra_info[9] = (uTeHdh >> 8) & 0xff;

	iris_extra_info[10] = uTeVdh & 0xff;
	iris_extra_info[11] = (uTeVdh >> 8) & 0xff;

	uTemp = uVtotal - IRIS_DTG_EVS_DLY;
	uTemp = uTemp << 16;
	uTemp = 0xd | uTemp;
	iris_extra_info[32] = uTemp & 0xff;
	iris_extra_info[33] = (uTemp >> 8) & 0xff;
	iris_extra_info[34] = (uTemp >> 16) & 0xff;
	iris_extra_info[35] = (uTemp >> 24) & 0xff;

	iris_extra_info[48] = 0;
	iris_extra_info[49] = uVtotal & 0xff;
	iris_extra_info[50] = (uVtotal >> 8) & 0xff;
	iris_extra_info[51] = (uVtotal >> 16) & 0xff;

	iris_extra_info[56] = (uVtotal - 12)&0xff;
	iris_extra_info[57] = ((uVtotal - 12)>>8)& 0xff;
	iris_extra_info[58] = ((uVtotal - 5) ) & 0xff;
	iris_extra_info[59] = ((uVtotal - 5) >> 8) & 0xff;

	panel_cmds.cmds = iris_extra_info_cmds;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_extra_info_cmds);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

}

//from bootloader setting
static u8 iris_add8_line_feature[]=
{
	0x80, 0x87, 0x0, 0x1,
	PWIL_U32(IRIS_DISPLAY_SCALER1D_ADDR + 0x0010),
	PWIL_U32(0x00000460),  //enable extend_bottom
	PWIL_U32(IRIS_DISPLAY_SCALER1D_ADDR + 0x0014),
	PWIL_U32(0x00080000),
	PWIL_U32(IRIS_DISPLAY_SCALER1D_ADDR + 0x0024),
	PWIL_U32(0x06030a03),
	PWIL_U32(IRIS_PEAKING2D_ADDR + 0x0068),
	PWIL_U32(0x00010000),
	PWIL_U32(IRIS_PEAKING2D_ADDR + 0x1ff00),
	PWIL_U32(0x00000001),
};

void iris_add8lines_set(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	struct dsi_cmd_desc iris_add8lines_cmds[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC, sizeof(iris_add8_line_feature) }, iris_add8_line_feature},
	};

	panel_cmds.cmds = iris_add8lines_cmds;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_add8lines_cmds);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_panel_on_finish(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;
	struct mdss_dsi_ctrl_pdata *ctrl0;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_info("send panel on finish\n");
	if ((iris_mipi_info.mipi_mode.rx_ch == 0) && (ctrl->ndx == DSI_CTRL_LEFT)) {
		iris_dump_packet(panel_config_finishflag, sizeof(panel_config_finishflag));

		panel_cmds.cmds = panel_lightup_finish;
		panel_cmds.cmd_cnt = ARRAY_SIZE(panel_lightup_finish);
		panel_cmds.link_state = DSI_LP_MODE;
		mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
	} else if ((iris_mipi_info.mipi_mode.rx_ch == 1) && (ctrl->ndx == DSI_CTRL_RIGHT)) {
		ctrl0 = container_of(g_pdata0,
struct mdss_dsi_ctrl_pdata, panel_data);
		iris_mipi_pwil(ctrl);

		iris_mipi_mcu(ctrl0);
		iris_dump_packet(panel_config_finishflag, sizeof(panel_config_finishflag));
		panel_cmds.cmds = panel_lightup_finish;
		panel_cmds.cmd_cnt = ARRAY_SIZE(panel_lightup_finish);
		panel_cmds.link_state = DSI_LP_MODE;
		mdss_dsi_panel_cmds_send_ex(ctrl0, &panel_cmds);

		iris_mipi_pwil(ctrl0);
	}
	// update configure
	if (ctrl->ndx == DSI_CTRL_LEFT)
	    iris_update_configure();
	iris_cfg->ready = true;
}

static char get_mipi_rx_status[1] = {0xaf};
static struct dsi_cmd_desc iris2_mipi_rx_status_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 1, sizeof(get_mipi_rx_status)}, get_mipi_rx_status};
static unsigned short iris2_mipi_rx_status;

static void iris2_mipi_rx_status_cb(int len)
{
	//short response, return 2 bytes.
	if (len != 2)
		pr_err("%s: not short read responese, return len [%02x] != 2\n", __func__, len);

	iris2_mipi_rx_status = (iris_read_buf[0] & 0xFF) | ((iris_read_buf[1]&0x0f) << 8);
	pr_info("mipi_rx result [%04x]\n", iris2_mipi_rx_status);
}

static unsigned short iris2_mipi_status_result(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dcs_cmd_req cmdreq;
	memset(iris_read_buf, 0, sizeof(iris_read_buf));
	cmdreq.cmds = &iris2_mipi_rx_status_cmd;
	cmdreq.cmds_cnt = 1; // iris2_mipi_rx_status_cmd including 1cmd, it can bring more cmds
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 2; /* using CMD_REQ_RX, here meaning shor read. */
	cmdreq.rbuf = iris_read_buf;
	cmdreq.cb = iris2_mipi_rx_status_cb; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return iris2_mipi_rx_status;
}

static unsigned short iris2_mipi_check_status_result(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i = 0;
	unsigned short mipirx_res;
	do {
		i++;
		mipirx_res = iris2_mipi_status_result(ctrl);
	} while ((mipirx_res == 0x0) && i < 4);

	return mipirx_res;
}

static u8 iris2_fw_download_sts(struct mdss_dsi_ctrl_pdata *ctrl)
{
	unsigned short download_status;
	int i = 1;

	download_status = iris2_mipi_check_status_result(ctrl) & 0x0f00;
	while (download_status != 0x0100) {
		if (i >= IRIS2_RESEND_TIMES)
			return FW_DOWNLOAD_RETRY;
		usleep(21000);
		i++;
		download_status = iris2_mipi_check_status_result(ctrl) & 0xf00;
	}

	return FW_DOWNLOAD_FINISH;

}

void iris2_after_fw_dwonload_set(struct mdss_dsi_ctrl_pdata *ctrl)
{
	//restore mipi_rx setting, PB meta, now mipi rx is working on pwil_cmd mode
	char pb_meta[] = {0x0c, 0x01}; //for appcode download mode.
	char fw_after_conf[] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x00000007),	// valid word number
		0x00,			// burst mode
		0x00,			// reserved
		PWIL_U16(0x06),	// burst length
		PWIL_U32(IRIS_PWIL_ADDR + 0x0004),  //PWIL ctrl1 confirm transfer mode and cmd mode
		PWIL_U32(0x0000209a),
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0xc), //mipi_rx setting DBI_bus.
		PWIL_U32(0x000f0000), //only for signal channel.
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0x0018), //mipi_rx time out threshold.
		PWIL_U32(0x0000077f)
	};
	struct dsi_cmd_desc iris2_fw_restore[] = {
		{ {DTYPE_GEN_WRITE2, 1, 0, 0, 1, sizeof(pb_meta)}, pb_meta},
		{ {DTYPE_GEN_LWRITE, 1, 0, 0, 0,  sizeof(fw_after_conf)}, fw_after_conf}
	};
	struct dsi_panel_cmds panel_cmds;

	//confirm pwil work mode, video or cmd.
	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode)
		fw_after_conf[20] = 0x18;
	else
		fw_after_conf[20] = 0x1a;

	//confirm dual or signal channel mode
	if (0 == iris_mipi_info.mipi_mode.rx_ch) {
		;//single channel
	} else {
		fw_after_conf[20] |= 0x01;
		fw_after_conf[30] = 0x8f;
		fw_after_conf[36] = 0xff;
		fw_after_conf[37] = 0x9;
	}

	panel_cmds.cmds = iris2_fw_restore;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris2_fw_restore);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	iris_cmds(ctrl, MIPIRX_CMDMODE_CMDS, ctrl->ndx);

	mdss_dsi_cmd_hs_mode(0, &ctrl->panel_data);
	//restore mipi_rx to mcu mode to cover the following light up command
	iris_mipi_mcu(ctrl);

}

static int iris2_send_firmware(struct mdss_dsi_ctrl_pdata *ctrl,
		const u8 *data, size_t size)
{
	//for iris2, host send cmd video data to mipi_rx, mipi_rx re-organize data as pb protocol to pwil.
	//pwil capture size should larger than data. default size is 1920x1080x3(bytes), and appcode size is 128K
	//iris2 firmware download mode only use signal channel, cmd mode.
	char fw_download_pb_meta[] = {0x0c, 0x20}; //for appcode download mode.
	char fw_download_configure[] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x000000013),	// valid word number
		0x00,			// burst mode
		0x00,			// reserved
		PWIL_U16(0x0012),	// burst length
		PWIL_U32(IRIS_PWIL_ADDR + 0x0004),  //PWIL ctrl1 confirm transfer mode and cmd mode, single channel.
		PWIL_U32(0x0000209a),
		PWIL_U32(IRIS_PWIL_ADDR + 0x0218),  //CAPEN
		PWIL_U32(0xc0000003),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1140),  //channel order..
		PWIL_U32(0xc6120010),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1144),  //pixelformat.
		PWIL_U32(0x888),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1158), //mem addr.
		PWIL_U32(0x00000000),
		PWIL_U32(IRIS_PWIL_ADDR + 0x10000), //update setting. using SW update mode.
		PWIL_U32(0x00000100),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1fff0), //clear down load int.
		PWIL_U32(0x00008000),//bit 15
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0xc), //mipi_rx setting DBI_bus.
		PWIL_U32(0x000f0000), //only for signal channel.
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0x0018), //mipi_rx time out threshold.
		PWIL_U32(0xffffffff) //only for signal channel.
	};

	struct dsi_cmd_desc appcode_download_PWIL_set[] = {
		//set PWIL mif registers, include pixelformat, mem addr and so on.
		{ {DTYPE_GEN_LWRITE, 1, 0, 0, 0,  sizeof(fw_download_configure)}, fw_download_configure},
		//set mipi_rx meta to switch PWIL capture mode to mcu mode, using generic short write(0x23), 2 useful bytes.
		{ {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(fw_download_pb_meta)}, fw_download_pb_meta},
	};

	//mipi_rx should send firmware as video data on cmd mode.
	//host send every packet size should be pixel_numbers *3 bytes (8bit, so every pixel has 3bytes)
#define MAX_PACKET_SIZE  (256) //speed up is 512
#define TIME_INTERVAL (20000) //larger value to protect different case, video_signel(1635), cmd_single(2300). us.
	//the resolution is total send data size.  every sending data number should be equal packet_len(which is <= MAX_packet_size).
	//if mipi_rx received data less than resoluiton, it will add 0 in the last.
	char set_pixelformat[2] = {0x3a, 0x77}; //set ==mipi_rx==,  the data format is 8bit.
	char set_mem_addr[2] = {0x36, 0x0}; //set ==mipi_rx==,  write data is top to bottom, left to right.
	char set_col_addr[5] = {0x2A, 0x00, 0x00, 0x03, 0xFF};  //set ==mipi_rx==, imgae resolution, width-1
	char set_page_addr[5] = {0x2B, 0x00, 0x00, 0x03, 0xFF};  //set ==mipi_rx==, image resolution, height-1.

	int end_col = (MAX_PACKET_SIZE/3) - 1; //start from 0.
	int packet_len = (end_col + 1)*3;
	int end_page = (size + packet_len - 1) / packet_len - 1;//ALIGN_UP(size, packet_len) / packet_len - 1;
	size_t total_cnt = end_page + 1;//ALIGN_UP(size, packet_len) / packet_len;
	u32 cmd_index = 0, buf_index = 0, threshold = 0;

	u8 *buf;
	size_t len = 0, cnt = 0;
	int pending_len = 0;

	struct dsi_panel_cmds panel_cmds;

	char iris_mipirx_pwilcmd_mode[1] = {0x7f};
	struct dsi_cmd_desc iris_mipi_rx_mode[] = {
		{ { DTYPE_GEN_WRITE1, 1, 0, 0, 0,   sizeof(iris_mipirx_pwilcmd_mode) }, iris_mipirx_pwilcmd_mode},
	};
	struct dsi_cmd_desc iris2_set_addr_cmd[] = {
		{ { DTYPE_DCS_WRITE1, 0, 0, 0, 0,  sizeof(set_mem_addr) }, set_mem_addr},
		{ { DTYPE_DCS_WRITE1, 0, 0, 0, 0,  sizeof(set_pixelformat) }, set_pixelformat},
		{ { DTYPE_DCS_LWRITE, 0, 0, 0, 0, sizeof(set_col_addr) }, set_col_addr},
		{ { DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(set_page_addr) }, set_page_addr},
	};
	static struct dsi_cmd_desc iris2_send_fw[FW_DW_CNT_IRIS2];
	unsigned long long start_ns, dur_ns;

	pr_debug("%s: %d, start to download iris2 appcode!size = %zu\n", __func__, __LINE__, size);

	threshold = ctrl->pclk_rate/1000000;
	threshold *= TIME_INTERVAL;//avoid data overflow
	pr_debug("%s: %d, pclk = %d, threshold = %d\n", __func__, __LINE__, ctrl->pclk_rate, threshold);
	fw_download_configure[84] = (__u8)(threshold & 0xff);
	fw_download_configure[85] = (__u8)((threshold >> 8) & 0xff);
	fw_download_configure[86] = (__u8)((threshold >> 16) & 0xff);
	fw_download_configure[87] = (__u8)((threshold >> 24) & 0xff);

	iris_codedownload_start(ctrl);

	//change mipi_rx to cmd mode firstly.
	panel_cmds.cmds = iris_mipi_rx_mode;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_mipi_rx_mode);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	set_col_addr[3] = (end_col >> 8) & 0xFF;
	set_col_addr[4] = end_col & 0xFF;
	set_page_addr[3] = (end_page >> 8) & 0xFF;
	set_page_addr[4] = end_page & 0xFF;

	pr_info("mipi_rx setting, col_addr=%d, page_addr=%d\n", end_col, end_page);

	memset(iris2_send_fw, 0, sizeof(iris2_send_fw));

	if(g_firmware_buf)
		buf = g_firmware_buf;
	else
		return -1;

	//all mode should be send by using hs mode when mipi_rx is working on cmd mode.
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);

	//setting PWIL
	panel_cmds.cmds = appcode_download_PWIL_set;
	panel_cmds.cmd_cnt = ARRAY_SIZE(appcode_download_PWIL_set);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	//setting mipi_rx
	panel_cmds.cmds = iris2_set_addr_cmd;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris2_set_addr_cmd);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	// add timing measure and printk for fw download command only
	// (six other small commands are send seperately
	pr_info("%s: %d, total_cnt = %zu, packet_len %d\n",
		__func__, __LINE__, total_cnt, packet_len);
	start_ns = sched_clock();

	while (size) {
		if (size >= packet_len)
			len = packet_len;
		else {
			len = size;
			pending_len = packet_len - len;
		}

		cnt++;
		if ((cnt % FW_DW_CNT_IRIS2) == 0)
			cmd_index = FW_DW_CNT_IRIS2 - 1;
		else
			cmd_index = cnt % FW_DW_CNT_IRIS2 - 1;

		/*here should be the previously packet len, but for this case,
		only the last one is small than patcket_len, the others are same.
		so we used packet_len to replace previous packet len.
		*/
		buf_index = cmd_index * (packet_len + 1);

		if (1 == cnt)
			buf[0] = DCS_WRITE_MEM_START;
		else
			buf[buf_index] = DCS_WRITE_MEM_CONTINUE;

		memcpy(buf + buf_index + 1, data, len);

		iris2_send_fw[cmd_index].dchdr.last = 0;
		iris2_send_fw[cmd_index].dchdr.dtype = 0x39;
		iris2_send_fw[cmd_index].dchdr.dlen = packet_len + 1; //add buf[0]
		iris2_send_fw[cmd_index].payload = buf + buf_index;

		if ((cmd_index == (FW_DW_CNT_IRIS2 - 1)) || (cnt == total_cnt)) {
			iris2_send_fw[cmd_index].dchdr.last = 1;
			if (cnt == total_cnt)
				iris2_send_fw[cmd_index].dchdr.wait = 0; //IRIS_CMD_FIFO_EMPTY;
			panel_cmds.cmds = iris2_send_fw;
			panel_cmds.cmd_cnt = (cmd_index+1);
			panel_cmds.link_state = DSI_HS_MODE;
			mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
		}

		data += len;
		size -= len;

	}

	dur_ns = sched_clock() - start_ns;
	do_div(dur_ns, 1000);
	pr_info("%s: %d, fw download took %llu usec\n",
		__func__, __LINE__, dur_ns);

	return 0;
}

static int __iris_fw_download(struct mdss_dsi_ctrl_pdata *ctrl, const struct firmware *fw)
{
	int ret = 0;
	u8 itrycnt = 0;

	pr_info("%s, start\n", __func__);
	pw_iris2_status = -1;
	for (itrycnt = 0; itrycnt < FW_DOWNLOAD_RETRYCNT_MAX; itrycnt++) {
		ret = iris2_send_firmware(ctrl, fw->data, fw->size);
		if (iris2_fw_download_sts(ctrl) == FW_DOWNLOAD_FINISH) {
			pr_info("firmware download success!\n");
			iris2_after_fw_dwonload_set(ctrl);
			iris_codedownload_finish(ctrl);
			pw_iris2_status = 0;
			break;
		} else {
			iris2_after_fw_dwonload_set(ctrl);
		}
	}
	if (itrycnt >= FW_DOWNLOAD_RETRYCNT_MAX)
		pr_err(" firmware download error! retry times = %d\n", itrycnt);

	pr_info("%s, end\n", __func__);

	return ret;
}

#ifdef REF_PLL_19_2_MHZ
void iris_set_sys_efuse(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_cmd_desc iris_efuse_rewrite[] = {
		{ { DTYPE_GEN_LWRITE, 1, 0, 0, 60,  sizeof(efuse_ctrl_to_iris) },  efuse_ctrl_to_iris},
	};
	struct dsi_panel_cmds panel_cmds;

	pr_info("iris: send efuse ctrl\n");

	panel_cmds.cmds = iris_efuse_rewrite;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_efuse_rewrite);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}
#endif

bool iris_mcuclk_divider_change(struct mdss_dsi_ctrl_pdata *ctrl, char lowMcu)
{
#if 0
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	int swichenable = 0;
	//int dceEnable = dbc_setting_current.DCELevel & 0x8;
	mutex_lock(&iris_cfg->cmd_mutex);
	if(dbc_setting_current.dbcQuality == 0x5)
	{
		swichenable = 1;
		pr_info("could switch to lower  mcu clock\n");
	}
	pr_info("level: %x,dvQu: %x\n",dbc_setting_current.DCELevel,dbc_setting_current.dbcQuality );
	if(lowMcu && swichenable)
		iris_reg_add(IRIS_SYS_ADDR+0x218, 0x10901);
	else
		iris_reg_add(IRIS_SYS_ADDR+0x218, 0x1);
	iris_reg_add(IRIS_SYS_ADDR + 0x10, 1);	//reg_update
	iris_reg_add(IRIS_SYS_ADDR + 0x10, 0);	//reg_update
	mutex_unlock(&iris_cfg->cmd_mutex);
	pr_info("iris: %s, lowMcu: %d\n",  __func__, lowMcu);
	return swichenable;
#else
	return false;
#endif
}

void iris_init_info_send(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	pr_err("send iris init info\n");
	memcpy(&iris_mipi_mode[4], &(iris_mipi_info.mipi_mode), 4);
	panel_cmds.cmds = iris_init_info;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_init_info);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_firmware_download(struct mdss_dsi_ctrl_pdata *ctrl, const char *name)
{
	const struct firmware *fw = NULL;
	int ret = 0;
	struct dsi_cmd_desc iris_code_size_cmd[] = {
		{ { DTYPE_GEN_LWRITE, 1, 0, 0, CMD_PROC, sizeof(iris_code_size) }, iris_code_size},
	};
	struct dsi_panel_cmds panel_cmds;

	if (name) {
		/* Firmware file must be in /system/etc/firmware/ */
		ret = request_firmware(&fw, name, g_mfd->fbi->dev);
		if (ret) {
			pr_err("%s: %d, failed to request firmware: %s, ret = %d\n",
				__func__, __LINE__, name, ret);
			pw_iris2_status = -1;
		} else {
			pr_info("%s: %d, request firmware: name = %s, size = %zu bytes\n",
				__func__, __LINE__, name, fw->size);

			iris_code_size[8] = (__u8)(fw->size & 0xff);
			iris_code_size[9] = (__u8)((fw->size >> 8) & 0xff);
			iris_code_size[10] = (__u8)((fw->size >> 16) & 0xff);
			iris_code_size[11] = (__u8)((fw->size >> 24) & 0xff);

			// send firmware size
			panel_cmds.cmds = iris_code_size_cmd;
			panel_cmds.cmd_cnt = ARRAY_SIZE(iris_code_size_cmd);
			panel_cmds.link_state = DSI_LP_MODE;
			mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

			__iris_fw_download(ctrl, fw);
			release_firmware(fw);
		}
	} else  {
		pr_err("%s: %d, firmware is null\n", __func__, __LINE__);
	}
}

/*
* update dpll register, according to panel info and iris_param
* use h_res, v_res, frame_rate and pclk to search iris_dpll table
*/
static void iris_params_dpll(struct device_node *np)
{
	int rc;
#ifdef REF_PLL_19_2_MHZ
	int indx = 24;
#else
	int indx = 0;
#endif
	u32 iris_dpll_ctrl0, iris_dpll_ctrl1, iris_dpll_ctrl2 = 0;

	rc = of_property_read_u32(np, "qcom,iris-dpll0", &iris_dpll_ctrl0);
	if (rc) {
		pr_err("%s:%d, iris_dpll0 failed\n",
				__func__, __LINE__);
	}
	rc = of_property_read_u32(np, "qcom,iris-dpll1", &iris_dpll_ctrl1);
	if (rc) {
		pr_err("%s:%d, iris_dpll1 failed\n",
				__func__, __LINE__);
	}
	rc = of_property_read_u32(np, "qcom,iris-dpll2", &iris_dpll_ctrl2);
	if (rc) {
		pr_err("%s:%d, iris_dpll2 failed\n",
				__func__, __LINE__);
	}
	//config dpll ctrl0
	dpll_info_to_iris[indx + 8] = iris_dpll_ctrl0 & 0xff;
	dpll_info_to_iris[indx + 9] = 0x0;
	dpll_info_to_iris[indx + 10] = 0x0;
	dpll_info_to_iris[indx + 11] = 0x0;
	//config dpll ctrl1
	dpll_info_to_iris[indx + 16] = iris_dpll_ctrl1 & 0xff;
	dpll_info_to_iris[indx + 17] = (iris_dpll_ctrl1 >> 8) & 0xff;
	dpll_info_to_iris[indx + 18] = (iris_dpll_ctrl1 >> 16) & 0xff;
	dpll_info_to_iris[indx + 19] = iris_dpll_ctrl1 >> 24;
	//config dpll ctrl2
	dpll_info_to_iris[indx + 24] = iris_dpll_ctrl2 & 0xff;
	dpll_info_to_iris[indx + 25] = (iris_dpll_ctrl2 >> 8) & 0xff;
	dpll_info_to_iris[indx + 26] = (iris_dpll_ctrl2 >> 16) & 0xff;
	dpll_info_to_iris[indx + 27] = iris_dpll_ctrl2 >> 24;

}

/*
* update dtg register, according to panel info
*/
static void iris_params_dtg(struct device_node *np,
					struct mdss_panel_info *panel_info)
{
	int rc = 0;
	u32 tmp;
	struct iris_timing_para iris_timing;
	struct iris_dsc_para iris_dsc;

	memset(&iris_timing, 0, sizeof(iris_timing));

	iris_timing.hfp = panel_info->lcdc.h_front_porch;
	iris_timing.hres = panel_info->xres;
	iris_timing.hbp = panel_info->lcdc.h_back_porch;
	iris_timing.hsw = panel_info->lcdc.h_pulse_width;

	iris_timing.vfp = panel_info->lcdc.v_front_porch;
	iris_timing.vres = panel_info->yres;
	iris_timing.vbp = panel_info->lcdc.v_back_porch;
	iris_timing.vsw = panel_info->lcdc.v_pulse_width;

	// config hsctrl0 of dtg
	panel_info_to_iris[4] = iris_timing.hfp & 0xff;
	panel_info_to_iris[5] = (iris_timing.hfp >> 8) & 0xff;
	panel_info_to_iris[6] = iris_timing.hres & 0xff;
	panel_info_to_iris[7] = (iris_timing.hres >> 8) & 0xff;
	// config hsctrl1 of dtg
	panel_info_to_iris[8] = iris_timing.hbp & 0xff;
	panel_info_to_iris[9] = (iris_timing.hbp >> 8) & 0xff;
	panel_info_to_iris[10] = iris_timing.hsw & 0xff;
	panel_info_to_iris[11] = (iris_timing.hsw >> 8) & 0xff;

	// config vsctrl0 of dtg
	panel_info_to_iris[12] = iris_timing.vfp & 0xff;
	panel_info_to_iris[13] = (iris_timing.vfp >> 8) & 0xff;
	panel_info_to_iris[14] = iris_timing.vres & 0xff;
	panel_info_to_iris[15] = (iris_timing.vres >> 8) & 0xff;
	// config vsctrl1 of dtg
	panel_info_to_iris[16] = iris_timing.vbp & 0xff;
	panel_info_to_iris[17] = (iris_timing.vbp >> 8) & 0xff;
	panel_info_to_iris[18] = iris_timing.vsw & 0xff;
	panel_info_to_iris[19] = (iris_timing.vsw >> 8) & 0xff;

	iris_mipi_info.iris_in_timing = iris_timing;

	rc = of_property_read_u32(np, "qcom,iris-out-panel-width", &tmp);
	if (rc) {
		iris_mipi_info.iris_out_timing = iris_timing;
		/*copy input timing to output timing*/
		memcpy(&panel_info_to_iris[24], &panel_info_to_iris[4], 16);
	} else {
		/*parse output timing*/
		iris_timing.hres = (!rc ? tmp : 640);

		rc = of_property_read_u32(np, "qcom,iris-out-panel-height", &tmp);
		iris_timing.vres = (!rc ? tmp : 480);
		rc = of_property_read_u32(np, "qcom,iris-out-h-front-porch", &tmp);
		iris_timing.hfp = (!rc ? tmp : 6);
		rc = of_property_read_u32(np, "qcom,iris-out-h-back-porch", &tmp);
		iris_timing.hbp = (!rc ? tmp : 6);
		rc = of_property_read_u32(np, "qcom,iris-out-h-pulse-width", &tmp);
		iris_timing.hsw = (!rc ? tmp : 2);
		rc = of_property_read_u32(np, "qcom,iris-out-v-back-porch", &tmp);
		iris_timing.vbp = (!rc ? tmp : 6);
		rc = of_property_read_u32(np, "qcom,iris-out-v-front-porch", &tmp);
		iris_timing.vfp = (!rc ? tmp : 6);
		rc = of_property_read_u32(np, "qcom,iris-out-v-pulse-width", &tmp);
		iris_timing.vsw = (!rc ? tmp : 2);

		iris_mipi_info.iris_out_timing = iris_timing;
		// config hsctrl0 of dtg
		panel_info_to_iris[24] = iris_timing.hfp & 0xff;
		panel_info_to_iris[25] = (iris_timing.hfp >> 8) & 0xff;
		panel_info_to_iris[26] = iris_timing.hres & 0xff;
		panel_info_to_iris[27] = (iris_timing.hres >> 8) & 0xff;
		// config hsctrl1 of dtg
		panel_info_to_iris[28] = iris_timing.hbp & 0xff;
		panel_info_to_iris[29] = (iris_timing.hbp >> 8) & 0xff;
		panel_info_to_iris[30] = iris_timing.hsw & 0xff;
		panel_info_to_iris[31] = (iris_timing.hsw >> 8) & 0xff;

		// config vsctrl0 of dtg
		panel_info_to_iris[32] = iris_timing.vfp & 0xff;
		panel_info_to_iris[33] = (iris_timing.vfp >> 8) & 0xff;
		panel_info_to_iris[34] = iris_timing.vres & 0xff;
		panel_info_to_iris[35] = (iris_timing.vres >> 8) & 0xff;
		// config vsctrl1 of dtg
		panel_info_to_iris[36] = iris_timing.vbp & 0xff;
		panel_info_to_iris[37] = (iris_timing.vbp >> 8) & 0xff;
		panel_info_to_iris[38] = iris_timing.vsw & 0xff;
		panel_info_to_iris[39] = (iris_timing.vsw >> 8) & 0xff;
	}
	/*check input timing and output timing is same or different*/
	if (0 == memcmp(&iris_mipi_info.iris_in_timing, &iris_mipi_info.iris_out_timing, sizeof(struct iris_timing_para)))
		iris_mipi_info.iris_timing_flag = 0;
	else
		iris_mipi_info.iris_timing_flag = 1;

	/*parse input DSC para*/
	rc = of_property_read_u32(np, "qcom,iris-in-slice-number", &tmp);
	iris_dsc.slice_number = (!rc ? tmp : 8);
	rc = of_property_read_u32(np, "qcom,iris-in-slice-height", &tmp);
	iris_dsc.slice_height = (!rc ? tmp : 16);
	rc = of_property_read_u32(np, "qcom,iris-in-bpp", &tmp);
	iris_dsc.bpp = (!rc ? tmp : 0x80);
	iris_mipi_info.iris_in_dsc = iris_dsc;
	panel_info_to_iris[44] = iris_dsc.slice_number & 0xff;
	panel_info_to_iris[45] = (iris_dsc.slice_number >> 8) & 0xff;
	panel_info_to_iris[46] = iris_dsc.slice_height & 0xff;
	panel_info_to_iris[47] = (iris_dsc.slice_height >> 8) & 0xff;
	panel_info_to_iris[48] = iris_dsc.bpp & 0xff;
	panel_info_to_iris[49] = (iris_dsc.bpp >> 8) & 0xff;

	/*parse output DSC para*/
	rc = of_property_read_u32(np, "qcom,iris-out-slice-number", &tmp);
	iris_dsc.slice_number = (!rc ? tmp : 8);
	rc = of_property_read_u32(np, "qcom,iris-out-slice-height", &tmp);
	iris_dsc.slice_height = (!rc ? tmp : 16);
	rc = of_property_read_u32(np, "qcom,iris-out-bpp", &tmp);
	iris_dsc.bpp = (!rc ? tmp : 0x80);
	iris_mipi_info.iris_out_dsc = iris_dsc;
	panel_info_to_iris[52] = iris_dsc.slice_number & 0xff;
	panel_info_to_iris[53] = (iris_dsc.slice_number >> 8) & 0xff;
	panel_info_to_iris[54] = iris_dsc.slice_height & 0xff;
	panel_info_to_iris[55] = (iris_dsc.slice_height >> 8) & 0xff;
	panel_info_to_iris[56] = iris_dsc.bpp & 0xff;
	panel_info_to_iris[57] = (iris_dsc.bpp >> 8) & 0xff;

	/*parse delta period min & max*/
	rc = of_property_read_u32(np, "qcom,iris-delta-period-max", &tmp);
	iris_mipi_info.delta_period_max = (!rc ? tmp : panel_info->lcdc.v_front_porch);
	rc = of_property_read_u32(np, "qcom,iris-delta-period-min", &tmp);
	iris_mipi_info.delta_period_min = (!rc ? (0 - tmp) : (0 - panel_info->lcdc.v_front_porch));

}

/*
* update mipi rx register, according to panel info and iris_param
*/
static void iris_params_mipirx(struct device_node *np,
							struct mdss_panel_info *panel_info)
{
	int rc = 0;
	u8 index;
	u32 mipirx_dsi_func_program, mipirx_data_lane_timing = 0;
	u32 mipirx_hsync_count, mipirx_vsync_count = 0;
	u32 iris_rx_ch = 1;

	rc = of_property_read_u32(np, "qcom,mipirx-dsi-functional-program", &mipirx_dsi_func_program);
	if (rc) {
		pr_err("%s:%d, mipirx_dsi_func_program failed\n",
				__func__, __LINE__);
	}

	rc = of_property_read_u32(np, "qcom,iris-mipirx-channel", &iris_rx_ch);
	//config mipirx-dual-ch-enable
	if (!rc && (2 == iris_rx_ch)) {
		index = 16;
		mipirx_info_to_iris[index + 2] |= (1 << 7);
	}
	//config mipirx-frame-column-addr, 0x00014
	index = 24;
	mipirx_info_to_iris[index + 2] = (panel_info->xres * iris_rx_ch - 1) & 0xff;
	mipirx_info_to_iris[index + 3] = ((panel_info->xres * iris_rx_ch - 1) >> 8) & 0xff;

	//config mipirx-abnormal-count-thres, 0x00018
	index = 32;
	mipirx_info_to_iris[index + 0] = (panel_info->xres * iris_rx_ch - 1) & 0xff;
	mipirx_info_to_iris[index + 1] = ((panel_info->xres * iris_rx_ch - 1) >> 8) & 0xff;

	//config mipirx-data-lane-timing-param
	index = 40;
	mipirx_info_to_iris[index] = mipirx_dsi_func_program & 0xff;
	mipirx_info_to_iris[index + 1] = (mipirx_dsi_func_program >> 8) & 0xff;
	mipirx_info_to_iris[index + 2] = 0;
	mipirx_info_to_iris[index + 3] = 0;


	rc = of_property_read_u32(np, "qcom,mipirx-data-lane-timing-param", &mipirx_data_lane_timing);
	if (rc) {
		pr_err("%s:%d, mipirx_data_lane_timing failed\n",
				__func__, __LINE__);
	}

	//config mipirx-data-lane-timing-param
	index = 48;
	mipirx_info_to_iris[index] = mipirx_data_lane_timing & 0xff;
	mipirx_info_to_iris[index + 1] = (mipirx_data_lane_timing >> 8) & 0xff;
	mipirx_info_to_iris[index + 2] = 0;
	mipirx_info_to_iris[index + 3] = 0;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &mipirx_hsync_count);
	if (rc) {
		pr_err("%s:%d, mipirx_hsync_count failed\n",
				__func__, __LINE__);
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &mipirx_vsync_count);
	if (rc) {
		pr_err("%s:%d, mipirx_vsync_count failed\n",
				__func__, __LINE__);
	}
	//config mipirx hv sync count
	index = 72;
	mipirx_info_to_iris[index] = mipirx_hsync_count & 0xff;
	mipirx_info_to_iris[index + 1] = (mipirx_hsync_count >> 8) & 0xff;
	mipirx_info_to_iris[index + 2] = mipirx_vsync_count & 0xff;
	mipirx_info_to_iris[index + 3] = (mipirx_vsync_count >> 8) & 0xff;
}

/*
* update mipi tx register, according to panel info and iris_param
*/
static void iris_params_mipitx(struct device_node *np,
				struct mdss_panel_info *panel_info,
				struct  iris_mipi_param_calc *iris_param)
{
	struct iris_mipitx_config iris_mipitx;
	u8 index = 0;
	u32 mipitx_dsi_func_program, mipitx_hs_trans_timeout,
	mipitx_lp_receive_timeout, mipitx_hs2lp_switch_count, mipitx_pll_lock_count,
	mipitx_clocklane_switch_count, mipitx_lp_eq_byteclk = 0;

	u32 mipitx_dphy_param, mipitx_data_lane_timing = 0;
	u32 mipitx_video_mode = 0;
	int rc = 0;
	struct iris_timing_para *iris_timing;
	//config video mode format of mipitx

	rc = of_property_read_u32(np, "qcom,mipitx-video-mode", &mipitx_video_mode);
	if (rc) {
		mipitx_video_mode = 0x3;
		pr_err("%s:%d, set mipitx-video-mode to default\n",
				__func__, __LINE__);
	}
	index = 152;
	mipitx_info_to_iris[index] = mipitx_video_mode & 0xff;

	memset(&iris_mipitx, 0, sizeof(iris_mipitx));
	iris_timing = &(iris_mipi_info.iris_out_timing);
	iris_mipitx.dpi_res = (iris_timing->vres << 16) + iris_timing->hres;
	iris_mipitx.hsync_count = iris_timing->hsw * iris_param->ratio_panel_iris / 10000;
	iris_mipitx.hbp_count = iris_timing->hbp * iris_param->ratio_panel_iris / 10000;
	iris_mipitx.h_res = iris_timing->hres * iris_param->ratio_panel_iris / 10000;
	iris_mipitx.hfp_count = (iris_timing->hfp + iris_timing->hbp
						+ iris_timing->hsw + iris_timing->hres)
						* iris_param->ratio_panel_iris / 10000
						- (iris_mipitx.hsync_count + iris_mipitx.hbp_count + iris_mipitx.h_res);

	iris_mipitx.vsync_count = iris_timing->vsw;
	iris_mipitx.vbp_count = iris_timing->vbp;
	iris_mipitx.vfp_count = iris_timing->vfp;

	//config dpi res of mipitx
	index = 64;
	mipitx_info_to_iris[index] = iris_mipitx.dpi_res & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.dpi_res >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.dpi_res >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.dpi_res >> 24;

	//config hsync count of mipitx
	index = 72;
	mipitx_info_to_iris[index] = iris_mipitx.hsync_count & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.hsync_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.hsync_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.hsync_count >> 24;

	//config hbp count of mipitx
	index = 80;
	mipitx_info_to_iris[index] = iris_mipitx.hbp_count & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.hbp_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.hbp_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.hbp_count >> 24;

	//config hfp count of mipitx
	index = 88;
	mipitx_info_to_iris[index] = iris_mipitx.hfp_count & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.hfp_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.hfp_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.hfp_count >> 24;

	//config hres count of mipitx
	index = 96;
	mipitx_info_to_iris[index] = iris_mipitx.h_res & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.h_res >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.h_res >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.h_res >> 24;

	//config vsync count of mipitx
	index = 104;
	mipitx_info_to_iris[index] = iris_mipitx.vsync_count & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.vsync_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.vsync_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.vsync_count >> 24;

	//config vbp count of mipitx
	index = 112;
	mipitx_info_to_iris[index] = iris_mipitx.vbp_count & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.vbp_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.vbp_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.vbp_count >> 24;

	//config vfp count of mipitx
	index = 120;
	mipitx_info_to_iris[index] = iris_mipitx.vfp_count & 0xff;
	mipitx_info_to_iris[index + 1] = (iris_mipitx.vfp_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (iris_mipitx.vfp_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = iris_mipitx.vfp_count >> 24;

	//config mipitx-dsi-functional-program
	rc = of_property_read_u32(np, "qcom,mipitx-dsi-functional-program", &mipitx_dsi_func_program);
	if (rc) {
		pr_err("%s:%d, mipirx_dsi_func_program failed\n",
				__func__, __LINE__);
	}
	index = 24;
	mipitx_info_to_iris[index] = mipitx_dsi_func_program & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_dsi_func_program >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = 0;
	mipitx_info_to_iris[index + 3] = 0;

	//config mipitx-hs-transmit-timeout
	rc = of_property_read_u32(np, "qcom,mipitx-hs-transmit-timeout", &mipitx_hs_trans_timeout);
	if (rc) {
		pr_err("%s:%d, mipitx_hs_trans_timeout failed\n",
				__func__, __LINE__);
	}
	index = 32;
	mipitx_info_to_iris[index] = mipitx_hs_trans_timeout & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_hs_trans_timeout >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (mipitx_hs_trans_timeout >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = 0;

	//config mipitx-lp-receive-timeout
	rc = of_property_read_u32(np, "qcom,mipitx-lp-receive-timeout", &mipitx_lp_receive_timeout);
	if (rc) {
		pr_err("%s:%d, mipitx_lp_receive_timeout failed\n",
				__func__, __LINE__);
	}
	index = 40;
	mipitx_info_to_iris[index] = mipitx_lp_receive_timeout & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_lp_receive_timeout >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (mipitx_lp_receive_timeout >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = 0;

	//config mipitx-hs-lp-switching-time-count
	rc = of_property_read_u32(np, "qcom,mipitx-hs-lp-switching-time-count", &mipitx_hs2lp_switch_count);
	if (rc) {
		pr_err("%s:%d, mipitx_hs2lp_switch_count failed\n",
				__func__, __LINE__);
	}
	index = 128;
	mipitx_info_to_iris[index] = mipitx_hs2lp_switch_count & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_hs2lp_switch_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (mipitx_hs2lp_switch_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = mipitx_hs2lp_switch_count >> 24;

	//config mipitx-pll-lock-count
	rc = of_property_read_u32(np, "qcom,mipitx-pll-lock-count", &mipitx_pll_lock_count);
	if (rc) {
		pr_err("%s:%d, mipitx_pll_lock_count failed\n",
				__func__, __LINE__);
	}
	index = 136;
	mipitx_info_to_iris[index] = mipitx_pll_lock_count & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_pll_lock_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = 0;
	mipitx_info_to_iris[index + 3] = 0;
	//config initialisation count
	index = 144;
	mipitx_info_to_iris[index] = mipitx_pll_lock_count & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_pll_lock_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = 0;
	mipitx_info_to_iris[index + 3] = 0;

	//config mipitx-clock-lane-switching-time-count
	rc = of_property_read_u32(np, "qcom,mipitx-clock-lane-switching-time-count", &mipitx_clocklane_switch_count);
	if (rc) {
		pr_err("%s:%d, mipitx_clocklane_switch_count failed\n",
				__func__, __LINE__);
	}
	index = 176;
	mipitx_info_to_iris[index] = mipitx_clocklane_switch_count & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_clocklane_switch_count >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (mipitx_clocklane_switch_count >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = mipitx_clocklane_switch_count >> 24;

	//config mipitx-lp-equivalent-byteclk
	rc = of_property_read_u32(np, "qcom,mipitx-lp-equivalent-byteclk", &mipitx_lp_eq_byteclk);
	if (rc) {
		pr_err("%s:%d, mipitx_lp_eq_byteclk failed\n",
				__func__, __LINE__);
	}
	index = 184;
	mipitx_info_to_iris[index] = mipitx_lp_eq_byteclk & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_lp_eq_byteclk >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = 0;
	mipitx_info_to_iris[index + 3] = 0;

	//config mipitx-dphy-param
	rc = of_property_read_u32(np, "qcom,mipitx-dphy-param", &mipitx_dphy_param);
	if (rc) {
		pr_err("%s:%d, mipitx_dphy_param failed\n",
				__func__, __LINE__);
	}
	index = 192;
	mipitx_info_to_iris[index] = mipitx_dphy_param & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_dphy_param >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (mipitx_dphy_param >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = mipitx_dphy_param >> 24;

	//config mipitx-data-lane-timing-param
	rc = of_property_read_u32(np, "qcom,mipitx-data-lane-timing-param", &mipitx_data_lane_timing);
	if (rc) {
		pr_err("%s:%d, mipitx_data_lane_timing failed\n",
				__func__, __LINE__);
	}
	index = 200;
	mipitx_info_to_iris[index] = mipitx_data_lane_timing & 0xff;
	mipitx_info_to_iris[index + 1] = (mipitx_data_lane_timing >> 8) & 0xff;
	mipitx_info_to_iris[index + 2] = (mipitx_data_lane_timing >> 16) & 0xff;
	mipitx_info_to_iris[index + 3] = mipitx_data_lane_timing >> 24;

	//config the byte1 of trim1 mipitx
	index = 208;
	mipitx_info_to_iris[index] = iris_param->trim1_divider_ratio & 0xff;

}

void iris_params_parse(struct device_node *np,
						struct mdss_panel_info *panel_info, struct  iris_mipi_param_calc *iris_param)
{
	iris_params_dpll(np);
	iris_params_dtg(np, panel_info);
	iris_params_mipirx(np, panel_info);
	iris_params_mipitx(np, panel_info, iris_param);
}

/*
* update iris work mode, according to iris_param
*/
static void iris_params_mipimode(struct device_node *np, char mode)
{
	int rc;
	u32 iris_rx_ch = 1, iris_tx_ch = 1, iris_rx_dsc = 0, iris_tx_dsc = 0;
	u32 tmp, iris_rx_pxl_mod = 0, iris_tx_pxl_mod = 1, iris_te_120_to_60 = 0;

	rc = of_property_read_u32(np, "qcom,iris-mipirx-channel", &tmp);
	iris_rx_ch = (!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,iris-mipitx-channel", &tmp);
	iris_tx_ch = (!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,iris-mipirx-dsc", &tmp);
	iris_rx_dsc = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,iris-mipitx-dsc", &tmp);
	iris_tx_dsc = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,iris-mipirx-pxl-mode", &tmp);
	iris_rx_pxl_mod = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,iris-mipitx-pxl-mode", &tmp);
	iris_tx_pxl_mod = (!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,iris-te-120-to-60", &tmp);
	iris_te_120_to_60 = (!rc ? tmp : 0);

	/*iris mipirx mode*/
	iris_mipi_info.mipi_mode.rx_mode = (DSI_VIDEO_MODE == mode) ? IRIS_MIPIRX_VIDEO : IRIS_MIPIRX_CMD;
	iris_mipi_info.mipi_mode.rx_ch = (iris_rx_ch == 1) ? 0 : 1;
	iris_mipi_info.mipi_mode.rx_dsc = iris_rx_dsc;
	iris_mipi_info.mipi_mode.bypass_en = 0;
	iris_mipi_info.mipi_mode.rx_pxl_mode = iris_rx_pxl_mod;
	/*iris mipitx mode*/
	iris_mipi_info.mipi_mode.tx_mode = IRIS_MIPIRX_VIDEO;
	iris_mipi_info.mipi_mode.tx_ch = (iris_tx_ch == 1) ? 0 : 1;
	iris_mipi_info.mipi_mode.tx_dsc = iris_tx_dsc;
	iris_mipi_info.mipi_mode.tx_pxl_mode = iris_tx_pxl_mod;

	iris_mipi_info.mipi_mode.te_120_to_60 = iris_te_120_to_60;
}

static int mdss_dsi_parse_dcs_cmds_ex(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len > sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	data = of_get_property(np, link_key, NULL);
	if (data && !strcmp(data, "dsi_hs_mode"))
		pcmds->link_state = DSI_HS_MODE;
	else
		pcmds->link_state = DSI_LP_MODE;

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}

void mdss_dsi_parse_iris_mipi(struct device_node *np,
					char mode, u32 panel_destination)
{

	if (panel_destination == DISPLAY_1) {
		iris_params_mipimode(np, mode);

		if (mode == DSI_CMD_MODE) {
			mdss_dsi_parse_dcs_cmds_ex(np, &(iris_mipi_info.mipirx_cmdmode_cmds),
				"qcom,mdss-dsi-on-command-to-iris-mipirx", "qcom,mdss-dsi-on-command-to-iris-mipirx-state");
			mdss_dsi_parse_dcs_cmds_ex(np, &(iris_mipi_info.panel_videomode_on_cmds[0]),
				"qcom,mdss-dsi-on-command-to-video-panel", "qcom,mdss-dsi-on-command-to-video-panel-state");
			mdss_dsi_parse_dcs_cmds_ex(np, &(iris_mipi_info.panel_videomode_off_cmds[0]),
				"qcom,mdss-dsi-off-command-to-video-panel", "qcom,mdss-dsi-off-command-to-video-panel-state");
		}
	} else if (mode == DSI_CMD_MODE) {
		mdss_dsi_parse_dcs_cmds_ex(np, &(iris_mipi_info.panel_videomode_on_cmds[1]),
			"qcom,mdss-dsi-on-command-to-video-panel", "qcom,mdss-dsi-on-command-to-video-panel-state");
		mdss_dsi_parse_dcs_cmds_ex(np, &(iris_mipi_info.panel_videomode_off_cmds[1]),
			"qcom,mdss-dsi-off-command-to-video-panel", "qcom,mdss-dsi-off-command-to-video-panel-state");
	}

}

void iris_cmds(struct mdss_dsi_ctrl_pdata *ctrl, u8 cflag, int dsiIndex)
{
	struct dsi_panel_cmds *pcmds = NULL;

	if ((iris_mipi_info.mipi_mode.rx_mode == IRIS_MIPIRX_VIDEO)
		&& (cflag == MIPIRX_CMDMODE_CMDS)) {
		pr_debug("%s %d mipirx is IRIS_MIPIRX_VIDEO\n", __func__, __LINE__);
		return;
	}

	switch (cflag) {
	case PANEL_ON_CMDS:
		if (iris_mipi_info.mipi_mode.rx_mode == IRIS_MIPIRX_CMD)
			pcmds = &iris_mipi_info.panel_videomode_on_cmds[dsiIndex];
		else
			pcmds = &ctrl->on_cmds;

		break;
	case PANEL_OFF_CMDS:
		if (iris_mipi_info.mipi_mode.rx_mode == IRIS_MIPIRX_CMD)
			pcmds = &iris_mipi_info.panel_videomode_off_cmds[dsiIndex];
		else
			pcmds = &ctrl->off_cmds;

		break;
	case MIPIRX_CMDMODE_CMDS:
		if (iris_mipi_info.mipi_mode.rx_mode == IRIS_MIPIRX_CMD)
			pcmds = &iris_mipi_info.mipirx_cmdmode_cmds;

		break;
	default:
		pr_err("%s %d the value of cflag is %d\n", __func__, __LINE__, cflag);
		break;
	}
	if (pcmds->cmd_cnt)
		mdss_dsi_panel_cmds_send_ex(ctrl, pcmds);
}

void iris_lightup_mode(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if (iris_mipi_info.mipi_mode.bypass_en) {
		iris_mipi_bypass_ex(ctrl);
	} else {
		iris_mipi_pwil(ctrl);
		iris_mipi_pt_enter(ctrl);
	}
}

static struct dsi_cmd_desc iris_mipi_bypass_cmds_ex[] = {
	{{ DTYPE_GEN_WRITE1, 1, 0, 0, 1,   sizeof(iris_bypass_mode) }, iris_bypass_mode}
};

void iris_mipi_bypass_ex(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	pr_info("iris: send iris bypass mode\n");
	iris_dump_packet(iris_bypass_mode, sizeof(iris_bypass_mode));
	panel_cmds.cmds = iris_mipi_bypass_cmds_ex;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_mipi_bypass_cmds_ex);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_mipi_pwil(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode)
		iris_pwil_mode[0] = 0xbf;
	else
		iris_pwil_mode[0] = 0x7f;

	pr_info("iris: send iris pwil\n");
	iris_dump_packet(iris_pwil_mode, sizeof(iris_pwil_mode));
	panel_cmds.cmds = iris_mipi_pwil_cmds;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_mipi_pwil_cmds);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_mipi_mcu(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	pr_info("iris: send iris mcu mode\n");
	iris_dump_packet(iris_mcu_mode, sizeof(iris_mcu_mode));
	panel_cmds.cmds = iris_mipi_mcu_cmds;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_mipi_mcu_cmds);
	panel_cmds.link_state = DSI_LP_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

static void iris_cmds_tx(struct work_struct *data)
{
	struct iris_mgmt_t *mgmt = container_of(data, struct iris_mgmt_t, iris_worker);

	if (mgmt->iris_handler)
		mgmt->iris_handler();
}

static u32  iris_pi_write(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr, u32 value)
{
	//struct mdss_overlay_private *mdp5_data;
	//struct mdss_panel_data *pdata;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	static char pwil_write[24] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x3),
		0x00,
		0x00,
		PWIL_U16(0x2),
		PWIL_U32(IRIS_PROXY_ADDR + 0x00), //default set to proxy MB0
		PWIL_U32(0x00000000)
	};

	static struct dsi_cmd_desc iris_pwil_write_cmd = {
		{ DTYPE_GEN_LWRITE,  1, 0, 0, 0, sizeof(pwil_write) }, pwil_write };

	struct dcs_cmd_req cmdreq;

	if (!iris_cfg->ready) {
		pr_err("%s:%u: iris not ready!\n", __func__, __LINE__);
		return -EINVAL;
	}

	pr_debug("%s, addr: 0x%x, value: 0x%x\n", __func__, addr, value);

	pwil_write[16] = addr         & 0xff;
	pwil_write[17] = (addr >>  8) & 0xff;
	pwil_write[18] = (addr >> 16) & 0xff;
	pwil_write[19] = (addr >> 24) & 0xff;
	pwil_write[20] = value          & 0xff;
	pwil_write[21] = (value  >>  8) & 0xff;
	pwil_write[22] = (value  >> 16) & 0xff;
	pwil_write[23] = (value  >> 24) & 0xff;

	cmdreq.cmds = &iris_pwil_write_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	return 0;
}

static u32 iris_pi_read(struct mdss_dsi_ctrl_pdata *ctrl, u32 addr)
{
	u32 value;

	char pi_address[16] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('S', 'G', 'L', 'W'),
		PWIL_U32(0x01),	//valid body word(4bytes)
		PWIL_U32(IRIS_PROXY_ADDR),   // proxy MB0
	};

	struct dsi_cmd_desc pi_read_addr_cmd[] = {
		{ { DTYPE_GEN_LWRITE,  1, 0, 0, 0, sizeof(pi_address) }, pi_address },
	};

	//char pktsize[2] = {0x04, 0x00}; /* LSB tx first, 10 bytes */

	char pi_read[1] = { 0x00 };
	struct dsi_cmd_desc pi_read_cmd = {
		{ DTYPE_GEN_READ1,   1, 0, 1, 0, sizeof(pi_read) }, pi_read
	};

	char read_buf[16]; //total 4*32bit register
	struct dcs_cmd_req cmdreq;

	pi_address[12] = addr         & 0xff;
	pi_address[13] = (addr >>  8) & 0xff;
	pi_address[14] = (addr >> 16) & 0xff;
	pi_address[15] = (addr >> 24) & 0xff;

	cmdreq.cmds = pi_read_addr_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	cmdreq.cmds = &pi_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_RX | CMD_REQ_COMMIT | CMD_REQ_NO_MAX_PKT_SIZE;
	cmdreq.rlen = 4;
	cmdreq.rbuf = (char *)read_buf;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	value = ctrl->rx_buf.data[0] | (ctrl->rx_buf.data[1] << 8) |
		(ctrl->rx_buf.data[2] << 16) | (ctrl->rx_buf.data[3] << 24);

	return value;
}

static struct completion iris_vsync_comp;

static void mdss_iris_vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t vtime)
{
	pr_debug("#### %s:%d vtime=%lld\n", __func__, __LINE__, vtime.tv64);
	complete(&iris_vsync_comp);
}

static struct mdss_mdp_vsync_handler iris_vsync_handler = {
	.vsync_handler = mdss_iris_vsync_handler,
};

int iris_wait_for_vsync(struct mdss_mdp_ctl *ctl)
{
	int rc;

	pr_debug("#### %s:%d\n", __func__, __LINE__);
	init_completion(&iris_vsync_comp);
	ctl->ops.add_vsync_handler(ctl, &iris_vsync_handler);
	rc = wait_for_completion_interruptible_timeout(
		&iris_vsync_comp, msecs_to_jiffies(100));
	ctl->ops.remove_vsync_handler(ctl, &iris_vsync_handler);
	if (rc < 0)
		pr_err("#### %s:%d: error %d\n", __func__, __LINE__, rc);
	else if (rc == 0) {
		pr_debug("#### %s:%d: timeout\n", __func__, __LINE__);
		rc = -ETIMEDOUT;
	}
	return rc;
}

void iris_mipi_pt_enter(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_panel_cmds panel_cmds;

	if (iris_mipi_info.iris_timing_flag
			|| (IRIS_MIPIRX_CMD == iris_mipi_info.mipi_mode.rx_mode)) {
		pt_enable[0] = 0x3 << 2;
		pt_enable[1] = 0x1;
	} else {
		pt_enable[0] = 0x0;
		pt_enable[1] = 0x1;
	}
	pr_info("%s: pt mode: %x, %x\n", __func__, pt_enable[0], pt_enable[1]);
	panel_cmds.cmds = iris_pt_enable_cmd;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_pt_enable_cmd);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

static void iris_nfrv_vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t vtime)
{
	//u32 off, mixercfg;

	pr_debug("#### %s:%d vtime=%lld\n", __func__, __LINE__, vtime.tv64);
	/*
	mixercfg = MDSS_MDP_LM_BORDER_COLOR;
	off = MDSS_MDP_REG_CTL_LAYER(0);
	mdss_mdp_ctl_write(ctl, off, mixercfg);
	*/
	ctl->force_screen_state = MDSS_SCREEN_FORCE_BLANK;
}

static struct mdss_mdp_vsync_handler nfrv_vsync_handler = {
	.vsync_handler = iris_nfrv_vsync_handler,
};

int iris_fbo_enable(struct msm_fb_data_type *mfd, int enable)
{
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;

	if (enable && !mfd->iris_fbo_enable) {
		mfd->iris_fbo_enable = true;
		ctl->ops.add_vsync_handler(ctl, &nfrv_vsync_handler);
		pr_err("%s:%d enable\n", __func__, __LINE__);
	} else if (!enable && mfd->iris_fbo_enable) {
		mfd->iris_fbo_enable = false;
		ctl->ops.remove_vsync_handler(ctl, &nfrv_vsync_handler);
		ctl->force_screen_state = MDSS_SCREEN_DEFAULT;
	}

	return 0;
}

int iris_sbs_enable(struct msm_fb_data_type *mfd, int enable)
{
	if (enable && !mfd->iris_sbs_enable) {
		mfd->iris_sbs_enable = true;
		pr_err("%s:%d enable\n", __func__, __LINE__);
	} else if (!enable && mfd->iris_sbs_enable) {
		mfd->iris_sbs_enable = false;
		pr_err("%s:%d disable\n", __func__, __LINE__);
	}

	return 0;
}

int iris_register_write(struct msm_fb_data_type *mfd,	u32 addr, u32 value)
{
	struct mdss_overlay_private *mdp5_data;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	static char pwil_write[24] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x3),
		0x00,
		0x00,
		PWIL_U16(0x2),
		PWIL_U32(IRIS_PROXY_ADDR + 0x00), //default set to proxy MB0
		PWIL_U32(0x00000000)
	};

	static struct dsi_cmd_desc iris_pwil_write_cmd = {
		{ DTYPE_GEN_LWRITE,  1, 0, 0, 0, sizeof(pwil_write) }, pwil_write };

	struct dcs_cmd_req cmdreq;

	if (!iris_cfg->ready) {
		pr_err("%s:%u: iris not ready!\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (mfd->panel_power_state == MDSS_PANEL_POWER_OFF)
		return 0;

	mdp5_data = mfd_to_mdp5_data(mfd);
	pdata = mdp5_data->ctl->panel_data;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_debug("%s, addr: 0x%x, value: 0x%x\n", __func__, addr, value);

	pwil_write[16] = addr         & 0xff;
	pwil_write[17] = (addr >>  8) & 0xff;
	pwil_write[18] = (addr >> 16) & 0xff;
	pwil_write[19] = (addr >> 24) & 0xff;
	pwil_write[20] = value          & 0xff;
	pwil_write[21] = (value  >>  8) & 0xff;
	pwil_write[22] = (value  >> 16) & 0xff;
	pwil_write[23] = (value  >> 24) & 0xff;

	cmdreq.cmds = &iris_pwil_write_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode) {
		while ((atomic_read(&mfd->iris_conf.mode_switch_cnt)))
			usleep(17000);
	}

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode) {
		/* wait 1 vsync to sure command issue */
		usleep(17000);
	}

	return 0;
}

int iris_notify_video_frame_rate(struct msm_fb_data_type *mfd,
				void __user *argp)
{
	uint32_t frame_rate_ms = 0;
	uint32_t r;
	int ret = copy_from_user(&frame_rate_ms, argp, sizeof(frame_rate_ms));

	pr_info("frame_rate_ms = %u\n", frame_rate_ms);

	// round to integer for 23976 and 29976
	mfd->iris_conf.input_frame_rate = (frame_rate_ms + 100) / 1000;
	mfd->iris_conf.output_frame_rate = 60;
	mfd->iris_conf.iris_ratio_updated = false;

	r = gcd(mfd->iris_conf.input_frame_rate, mfd->iris_conf.output_frame_rate);
	mfd->iris_conf.in_ratio = mfd->iris_conf.input_frame_rate / r;
	mfd->iris_conf.out_ratio = mfd->iris_conf.output_frame_rate / r;
	pr_debug("%s, in_ratio = %d, out_ratio = %d\n", __func__, mfd->iris_conf.in_ratio, mfd->iris_conf.out_ratio);

	return ret;
}

int iris_set_meta(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	struct iris_meta user_meta;

	ret = copy_from_user((void *)&user_meta, argp, sizeof(struct iris_meta));
	if (ret != 0)
		return -EINVAL;

	mutex_lock(&iris_cfg->meta_mutex);
	iris_cfg->meta_set.op |= user_meta.op;
	if (user_meta.op & MDP_IRIS_OP_NEW_FRAME)
		iris_cfg->meta_set.new_frame = user_meta.new_frame;
	if (user_meta.op & MDP_IRIS_OP_RESTART)
		iris_cfg->meta_set.restart = user_meta.restart;
	if (user_meta.op & MDP_IRIS_OP_VTS)
		iris_cfg->meta_set.video_ts = user_meta.video_ts;
	if (user_meta.op & MDP_IRIS_OP_STS)
		iris_cfg->meta_set.sys_ts = user_meta.sys_ts;
	if (user_meta.op & MDP_IRIS_OP_VID)
		iris_cfg->meta_set.vid = user_meta.vid;
	if (user_meta.op & MDP_IRIS_OP_TE)
		iris_cfg->meta_set.te_period = user_meta.te_period;
	if (user_meta.op & MDP_IRIS_OP_CP) {
		iris_cfg->meta_set.content_period = user_meta.content_period;
		iris_cfg->meta_set.content_period_frac = user_meta.content_period_frac;
	}
	if (user_meta.op & MDP_IRIS_OP_MOTION)
		iris_cfg->meta_set.motion = user_meta.motion;
	if (user_meta.op & MDP_IRIS_OP_JITTER)
		iris_cfg->meta_set.jitter = user_meta.jitter;
	if (user_meta.op & MDP_IRIS_OP_NRV)
		iris_cfg->meta_set.nrv = user_meta.nrv;
	if (user_meta.op & MDP_IRIS_OP_FLG)
		iris_cfg->meta_set.flags = user_meta.flags;
	if (user_meta.op & MDP_IRIS_OP_RPT)
		iris_cfg->meta_set.repeat = user_meta.repeat;
	mutex_unlock(&iris_cfg->meta_mutex);

	pr_debug("op [%08x] vTimestamp [%u] sTimestamp [%u] flag [%u]\n",
		iris_cfg->meta_set.op, iris_cfg->meta_set.video_ts, iris_cfg->meta_set.sys_ts, iris_cfg->meta_set.flags);

	if (iris_cfg->meta_set.op & MDP_IRIS_OP_RPT)
		pr_debug("repeat: %d\n", iris_cfg->meta_set.repeat);
	if (iris_cfg->meta_set.op & MDP_IRIS_OP_NRV) {
		struct iris_nrv_meta *nrv_meta = &iris_cfg->meta_set.nrv;

		pr_debug("NRV enable [%u]\n", nrv_meta->nrvEnable);
		pr_debug("Capture [%u][%u] [%u][%u]\n", nrv_meta->captureLeft, nrv_meta->captureRight,
												nrv_meta->captureTop, nrv_meta->captureBottom);
		pr_debug("Display [%u][%u] [%u][%u]\n", nrv_meta->displayLeft, nrv_meta->displayRight,
												nrv_meta->displayTop, nrv_meta->displayBottom);
	}
	return ret;
}

int iris_set_mode(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	uint32_t mode;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	ret = copy_from_user(&mode, argp, sizeof(uint32_t));

	pr_debug("mode = %d, c_mode = %d\n", mode, iris_cfg->sf_notify_mode);

	if (mode == iris_cfg->sf_notify_mode)
		iris_cfg->mode_changed = false;
	else {
		iris_cfg->mode_changed = true;
		iris_cfg->sf_notify_mode = mode;
		pw_iris2_status = mode;
	}
	return ret;
}

int iris_get_mode(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	uint32_t mode;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	mode = iris_cfg->sf_notify_mode;
	pr_debug("mode = %d\n", iris_cfg->sf_notify_mode);
	ret = copy_to_user(argp, &mode, sizeof(uint32_t));

	return ret;
}

int iris_set_rotation(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	bool rotationen;

	ret = copy_from_user(&rotationen, argp, sizeof(bool));
	pr_debug("rotationen = %d\n", rotationen);

	mutex_lock(&mfd->iris_conf.cmd_mutex);
	iris_reg_add(IRIS_MVC_ADDR + 0xc, rotationen << 1);
	iris_reg_add(IRIS_MVC_ADDR + 0x1FF00, 1);
	mutex_unlock(&mfd->iris_conf.cmd_mutex);

	return ret;
}
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
int iris_Drc_LPMemc_update(struct msm_fb_data_type *mfd) {

	u32 configAddr = 0;
	u32 configValue = 0;

	configValue = g_mfd->iris_conf.drc_size | 0x80000000;
	configAddr = IRIS_PROXY_ADDR + 0x20;

	if (0 == configValue && 0 == configAddr) {
		pr_warn("iris_Drc_LPMemc_update failed!\n");
		return -EINVAL;
	}

	return iris_register_write(mfd, configAddr, configValue);

}

int iris_get_frc_timing(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	pr_debug("guFrcLPTiming = %x\n", guFrcLPTiming);
	ret = copy_to_user(argp, &guFrcLPTiming, sizeof(uint32_t));
	//TODO
	return ret;
}

/*****
* DRC Dynamic resolution change
*
******/
int iris_set_drc_size(struct msm_fb_data_type *mfd, void __user *argp)
{
	int ret;
	uint32_t utemp;
	ret = copy_from_user(&utemp, argp, sizeof(uint32_t));

	g_mfd->iris_conf.drc_enable = (utemp > 0) ? true : false;
	g_mfd->iris_conf.drc_size = utemp;

	return ret;
}

void iris_calc_drc_enter(struct msm_fb_data_type *mfd)
{
	int width, height;
	int top, left;


	int Htotal = (iris_mipi_info.mipi_mode.rx_ch  ?  iris_mipi_info.iris_in_timing.hres * 2 : iris_mipi_info.iris_in_timing.hres);

	left = Htotal - (g_mfd->iris_conf.drc_size & 0xffff);
	top = iris_mipi_info.iris_in_timing.vres - (g_mfd->iris_conf.drc_size >> 16);

	width = g_mfd->iris_conf.drc_size & 0xffff;
	height = g_mfd->iris_conf.drc_size >> 16;

	mutex_lock(&mfd->iris_conf.cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x104c, ((uint32_t)top << 16 | left));
	iris_reg_add(IRIS_PWIL_ADDR + 0x1050, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x1054, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x106c, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	mutex_unlock(&mfd->iris_conf.cmd_mutex);
}

void iris_calc_drc_exit(struct msm_fb_data_type *mfd)
{
	int width, height;
	int top, left;


	int Htotal = (iris_mipi_info.mipi_mode.rx_ch  ?  iris_mipi_info.iris_in_timing.hres * 2 : iris_mipi_info.iris_in_timing.hres);

	left = 0;
	top = 0;

	width = Htotal;
	height = iris_mipi_info.iris_in_timing.vres;

	mutex_lock(&mfd->iris_conf.cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x104c, ((uint32_t)top << 16 | left));
	iris_reg_add(IRIS_PWIL_ADDR + 0x1050, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x1054, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x106c, ((uint32_t)height << 16) | (uint32_t)width);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	mutex_unlock(&mfd->iris_conf.cmd_mutex);
}
#endif
int iris_frc_path_update(struct msm_fb_data_type *mfd, void __user *argp)
{
	struct msmfb_iris_frc_path frc_path;
	int ret;

	ret = copy_from_user(&frc_path, argp, sizeof(frc_path));
	if (ret)
		return ret;
	pr_debug("frc_path.bit_index=%d enable =%d\n", frc_path.bit_index, frc_path.enable);
	if (frc_path.enable)
		g_mfd->iris_conf.frc_path |= BIT(frc_path.bit_index);
	else
		g_mfd->iris_conf.frc_path &= ~BIT(frc_path.bit_index);

	return 0;
}
static inline u32 mdss_mdp_cmd_vsync_count(struct mdss_mdp_ctl *ctl)
{
	struct mdss_mdp_mixer *mixer;
	u32 cnt = 0xffff;	/* init to an invalid value */

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_ON);

	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer) {
		mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_RIGHT);
		if (!mixer) {
			mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);
			goto exit;
		}
	}
	cnt = (mdss_mdp_pingpong_read(mixer->pingpong_base, MDSS_MDP_REG_PP_INT_COUNT_VAL) >> 16) & 0xffff;

	mdss_mdp_clk_ctrl(MDP_BLOCK_POWER_OFF);

exit:
	return cnt;
}

static void iris_proc_te(struct iris_config *iris_cfg, u32 fcnt, u32 lcnt, u32 fps, u32 vper, u32 hper)
{
	static u32 fcnt0, lcnt0;
	static u64 time0;
	static u32 te_period;
	ktime_t ktime = ktime_get();
	u64 time = ktime_to_us(ktime);

	if (fcnt - fcnt0 >= 1200) {
		if (time - time0) {
			u32 detla_t = time - time0;

			te_period = ((fcnt - fcnt0) * vper + lcnt - lcnt0)*1000/fps/(detla_t/1000);
			pr_debug("te_period=%u\n", te_period);
			if (abs(te_period - vper) > (vper >> 5))
				te_period = vper;
		}
		fcnt0 = fcnt;
		lcnt0 = lcnt;
		time0 = time;
	}

	te_period = vper;

	iris_cfg->meta.te_period = te_period;
	pr_debug("fcnt %u fcnt0 %u lcnt %u lcnt0 %u fps %u\n", fcnt, fcnt0, lcnt, lcnt0, fps);
	pr_debug("time %llu time0 %llu\n", time, time0);
	pr_debug("te %u vper %u\n", te_period, vper);
}

static int iris_vary_te(struct iris_config *iris_cfg, u32 fcnt, int vper)
{
#define THRESHOLD 0
#define FRAME_CNT 120
#define PLAYING 0x01
#define FIRST_FRAME 0x04
	static u32 fcnt0, vts0, time0, sts0;
	static bool player_sts;
	u32 time = iris_cfg->meta.sys_ts;
	u32 vts = iris_cfg->meta.video_ts;
	int delta_time, delta_period;
	int delta_t, delta_v, delta_sts, ret_val = false;
	u32 DisplayVtotal = (iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp +
		iris_mipi_info.iris_out_timing.vres + iris_mipi_info.iris_out_timing.vsw);
	ktime_t ktime = ktime_get();
	u32 sts = (u32) ktime_to_us(ktime);

	pr_debug("meta.op=0x%x, meta.flags=0x%x, meta.video_ts=%u, delta_period_range(%d, %d) \n",
		iris_cfg->meta.op, iris_cfg->meta.flags, iris_cfg->meta.video_ts,
		iris_mipi_info.delta_period_max, iris_mipi_info.delta_period_min);

	if (!(iris_cfg->meta.op & MDP_IRIS_OP_FLG)) {
		pr_debug("flag invalid\n");
		if (iris_cfg->sw_te_period != DisplayVtotal) {
			iris_cfg->sw_te_period = DisplayVtotal;
			ret_val = true;
		}
		return ret_val;
	}
	iris_cfg->meta.op &= ~MDP_IRIS_OP_FLG;

	if (!(iris_cfg->meta.flags & PLAYING)) {
		if (player_sts)
			pr_debug("play stop\n");
		//if video is stopped, retore TE to 60hz
		player_sts = 0;
		if (iris_cfg->sw_te_period != DisplayVtotal) {
			iris_cfg->sw_te_period = DisplayVtotal;
			ret_val = true;
		}
		return ret_val;
	}

	//get reference frame
	if (iris_cfg->meta.flags & FIRST_FRAME) {
		player_sts = 1;
		vts0 = iris_cfg->meta.video_ts;
		time0 = time;
		fcnt0 = fcnt;
		sts0 = sts;
		pr_debug("get reference frame ats0 %u vts0 %u sts0 %u f0 %u\n", time0, vts0, sts0, fcnt0);
	}

	delta_t = time - time0;
	delta_v = vts - vts0;
	delta_sts = sts - sts0;
	delta_time = delta_v - delta_t;

	if ((fcnt - fcnt0 >= FRAME_CNT) && vts && delta_t && delta_v && player_sts) {
		if (abs(delta_v - delta_t) > THRESHOLD) {
			// line_time = 1000000us / (60 * vper);
			// delta_period = delta_time / line_time;
			delta_period = (delta_time * vper) / 16667;
			delta_period = DIV_ROUND_CLOSEST(delta_period, FRAME_CNT);

			if (delta_period < iris_mipi_info.delta_period_min) {
				pr_debug("delta_period:%d out of min range\n", delta_period);
				delta_period = iris_mipi_info.delta_period_min;
			} else if (delta_period > iris_mipi_info.delta_period_max) {
				pr_debug("delta_period:%d out of max range\n", delta_period);
				delta_period = iris_mipi_info.delta_period_max;
			}
			iris_cfg->sw_te_period = vper + delta_period;
			ret_val = true;
		}
		pr_debug("fcnt %u fcnt0 %u vts %u vts0 %u delta_v %u\n", fcnt, fcnt0, vts, vts0, delta_v);
		pr_debug("time %u time0 %u delta_t %u\n", time, time0, delta_t);
		pr_debug("sts %u sts0 %u delta_sts %u\n", sts, sts0, delta_sts);
		pr_debug("delta_time %i delta_period %i vper %u ret_val %u\n", delta_time, delta_period, vper, ret_val);

		fcnt0 = fcnt;
		return ret_val;
	}
	return false;
}

static void iris_proc_ct(struct iris_config *iris_cfg, u32 fps)
{
	u32 prev_vts;
	u32 vts;
	u32 te;

	prev_vts = iris_cfg->prev_vts;
	vts = iris_cfg->meta.video_ts;
	te = iris_cfg->meta.te_period;
	iris_cfg->meta.content_period = (vts - prev_vts) * fps / 1000 * te / 1000;
	iris_cfg->meta.content_period_frac = (((vts - prev_vts) * fps / 1000 * te) & 0xfff) / (1000 >> 8);
	iris_cfg->meta.content_period_frac &= 0xff;
}

static void iris_proc_vp(struct iris_config *iris_cfg)
{
	iris_cfg->meta.vs_period = iris_cfg->meta.te_period;
	iris_cfg->meta.vs_period_frac = 0;
}

static void iris_proc_sts(struct iris_config *iris_cfg, u32 fps, u32 lcnt, u32 vper)
{
	u32 sts;
	ktime_t ktime = ktime_get();
	u64 time = ktime_to_us(ktime);

	if (iris_cfg->meta.op & MDP_IRIS_OP_STS) {
		pr_debug("sts %u\n", iris_cfg->meta.sys_ts);
		return;
	}

	sts = (u32) time;
	sts -= 1000000000 / fps / vper * lcnt / 1000;
	iris_cfg->meta.sys_ts = sts;
}

static void iris_proc_restart(struct iris_config *iris_cfg)
{
	if (!(iris_cfg->meta.op & MDP_IRIS_OP_RESTART))
		iris_cfg->meta.restart = 1;
	else
		iris_cfg->meta.restart = (iris_cfg->prev_vts == iris_cfg->meta.video_ts);
}

static int iris_proc_vts(struct iris_config *iris_cfg)
{
	int ret;

	ret = (iris_cfg->prev_vts != iris_cfg->meta.video_ts);
	iris_cfg->prev_vts = iris_cfg->meta.video_ts;
	return ret;
}

static void iris_set_te(struct iris_config *iris_cfg, int te_flag)
{
	if (!debug_te_enabled || !te_flag)
		return;

	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_DTG_ADDR + 0x00064, (iris_cfg->sw_te_period << 8));
	iris_reg_add(IRIS_DTG_ADDR + 0x10000, 1);	//reg_update
	mutex_unlock(&iris_cfg->cmd_mutex);
	pr_debug("set_te: %d\n", iris_cfg->sw_te_period);
}

static void iris_set_dtg(struct iris_config *iris_cfg)
{
	if (!debug_dtg_enabled)
		return;
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_DTG_ADDR + 0x10004, iris_cfg->meta.sys_ts);
	iris_reg_add(IRIS_DTG_ADDR + 0x10008, iris_cfg->meta.video_ts);
	iris_reg_add(IRIS_DTG_ADDR + 0x1000c, ((iris_cfg->meta.vs_period & 0xffff) << 8 |
					       (iris_cfg->meta.vs_period_frac & 0xff)));
	iris_reg_add(IRIS_DTG_ADDR + 0x10010, iris_cfg->meta.te_period);
	iris_reg_add(IRIS_DTG_ADDR + 0x10014, ((iris_cfg->meta.content_period & 0xffff) << 8 |
					       (iris_cfg->meta.content_period_frac & 0xff)));
	iris_reg_add(IRIS_DTG_ADDR + 0x10018, ((iris_cfg->meta.restart & 1) << 8 |
					       (iris_cfg->meta.motion & 0xff)));
	iris_reg_add(IRIS_DTG_ADDR + 0x1001c, 1);
	mutex_unlock(&iris_cfg->cmd_mutex);
	pr_debug("dtg set\n");
}

static void iris_proc_scale(struct iris_config *iris_cfg, u32 dvts, u32 prev_dvts)
{
	u32 scale;

	if (abs(dvts-prev_dvts) <= ((dvts + prev_dvts) >> 5))
		scale = 64;
	else {
		scale = (dvts * 64 + prev_dvts / 2) / prev_dvts;
		scale = min_t(u32, 255, scale);
		scale = max_t(u32, 16, scale);
	}
	iris_cfg->scale = scale;
	pr_debug("pdvts %u dvts %u scale %u\n", prev_dvts, dvts, scale);
}

static void iris_update_frcc(struct iris_config *iris_cfg) {

	unsigned int MaxFIFOFI = 0;
	switch (iris_cfg->input_frame_rate) {
	case 24://24fps 25fps
	case 25:
		MaxFIFOFI = 3;
		break;
	case 30://30fps
		MaxFIFOFI = 2;
		break;
	case 15://15fps
		MaxFIFOFI = 5;
		break;
	default:
		pr_err("%s, using default frcc parameters\n", __func__);
		break;
	}

	pr_debug("b4 iris_cfg->val_frcc_cmd_th 0x%08x\n", iris_cfg->val_frcc_cmd_th);
	iris_cfg->val_frcc_cmd_th &= 0xfffffff;
	iris_cfg->val_frcc_cmd_th |= (MaxFIFOFI * 2 << 28);
	pr_debug("after iris_cfg->val_frcc_cmd_th 0x%08x\n", iris_cfg->val_frcc_cmd_th);
}

static void iris_set_constant_ratio(struct iris_config *iris_cfg)
{
	unsigned int reg_in, reg_out, reg_scale, reg_cap;

	reg_in = iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT | (1 << 15);
	reg_out = iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT;
	reg_scale = 4096/iris_cfg->scale << 24 | 64 << 16 | iris_cfg->scale << 8 | iris_cfg->scale;
	/* duplicated video frame */
	reg_cap = (iris_cfg->meta.repeat != IRIS_REPEAT_CAPDIS) << 1;
	reg_cap |= 0xc0000001;
	iris_cfg->iris_ratio_updated = true;
	//when do asus camera preview, fps will change during frc. so need to  udpate frcc
	iris_update_frcc(iris_cfg);
	pr_debug("reg_cap 0x%08x\n", reg_cap);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
	iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
	if (debug_new_repeat == 0)
		iris_reg_add(IRIS_PWIL_ADDR + 0x0218, reg_cap);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	iris_reg_add(IRIS_MVC_ADDR + 0x1D0, reg_scale);
	iris_reg_add(IRIS_MVC_ADDR + 0x1FF00, 1);

	iris_reg_add(FRCC_CMD_MOD_TH, iris_cfg->val_frcc_cmd_th);
	iris_reg_add(FRCC_REG_SHOW, 0x2);
	mutex_unlock(&iris_cfg->cmd_mutex);
}

static int iris_proc_constant_ratio(struct iris_config *iris_cfg)
{
	u32 dvts, in_t, out_t;
	uint32_t r;

	dvts = 1000000 / iris_cfg->input_frame_rate;
	in_t = (dvts * iris_cfg->output_frame_rate + 50000) / 100000;
	out_t = 10;

	r = gcd(in_t, out_t);
	pr_debug("in_t %u out_t %u r %u\n", in_t, out_t, r);
	iris_cfg->in_ratio = out_t / r;
	iris_cfg->out_ratio = in_t / r;
	iris_proc_scale(iris_cfg, dvts, dvts);
	pr_debug("in/out %u:%u\n", iris_cfg->in_ratio, iris_cfg->out_ratio);
	// update register
	iris_set_constant_ratio(iris_cfg);

	return 0;
}

static int iris_proc_ratio(struct iris_config *iris_cfg)
{
	int ret = 0;
	u32 prev_dvts;
	u32 dvts, in_t, out_t;
	uint32_t r;

	dvts = iris_cfg->meta.video_ts - iris_cfg->prev_vts;
	prev_dvts = iris_cfg->prev_dvts;

	pr_debug("vts %u pvts %u dvts %u\n", iris_cfg->meta.video_ts, iris_cfg->prev_vts, dvts);
	if (dvts > 200000)
		return 0;

	if ((iris_cfg->iris_ratio_updated == true) && (abs(dvts - prev_dvts) < 3000))
		return 0;

	if (iris_cfg->repeat == IRIS_REPEAT_FORCE)
		return 0;

	if (debug_hlmd_enabled) {
		pr_debug("enable hlmd function.\n");
		// constant ratio
		ret = iris_proc_constant_ratio(iris_cfg);
		iris_cfg->prev_dvts = dvts;
		return ret;
	} else {
		pr_debug("don't enable hlmd function.\n");
		if (!iris_cfg->meta.video_ts && !dvts) {
			// constant ratio
			ret = iris_proc_constant_ratio(iris_cfg);
			iris_cfg->prev_dvts = dvts;
			return ret;
		}
	}

	if (prev_dvts && dvts) {
		in_t = (dvts * iris_cfg->output_frame_rate + 50000) / 100000;
		out_t = 10;

		r = gcd(in_t, out_t);
		pr_debug("in_t %u out_t %u r %u\n", in_t, out_t, r);
		iris_cfg->in_ratio = out_t / r;
		iris_cfg->out_ratio = in_t / r;
		iris_proc_scale(iris_cfg, dvts, prev_dvts);
		iris_cfg->iris_ratio_updated = (abs(dvts - prev_dvts) < 3000) ? true : false;
		ret = 1;
		pr_debug("in/out %u:%u\n", iris_cfg->in_ratio, iris_cfg->out_ratio);
	}

	if (prev_dvts && !dvts)
		ret = 1;

	if (dvts)
		iris_cfg->prev_dvts = dvts;

	return ret;
}

void iris_calc_nrv(struct mdss_mdp_ctl *ctl)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	uint16_t width, height;

#define VIDEO_CTRL3	0x104c
#define VIDEO_CTRL4	0x1050
#define VIDEO_CTRL5	0x1054
#define VIDEO_CTRL11	0x106c
#define VIDEO_CTRL12	0x1070
#define DISP_CTRL2		0x120c
#define REG_UPDATE      0x10000
#define MB1				0x0008
#define MB2				0x0010
#define MB4				0x0020
#define MB6				0x0030

	if (iris_cfg->meta.op & MDP_IRIS_OP_NRV) {
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL3, ((uint32_t)iris_cfg->meta.nrv.captureTop << 16) |
													(uint32_t)iris_cfg->meta.nrv.captureLeft);
		width = iris_cfg->meta.nrv.captureRight - iris_cfg->meta.nrv.captureLeft;
		height = iris_cfg->meta.nrv.captureBottom - iris_cfg->meta.nrv.captureTop;
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL4, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL5, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL11, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + VIDEO_CTRL12, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PWIL_ADDR + DISP_CTRL2, ((uint32_t)iris_cfg->meta.nrv.displayTop << 16) |
													(uint32_t)iris_cfg->meta.nrv.displayLeft);
		iris_reg_add(IRIS_PWIL_ADDR + REG_UPDATE, 0x100);

		width = iris_cfg->meta.nrv.displayRight - iris_cfg->meta.nrv.displayLeft;
		height = iris_cfg->meta.nrv.displayBottom - iris_cfg->meta.nrv.displayTop;
		iris_reg_add(IRIS_PROXY_ADDR + MB4, ((uint32_t)height << 16) | (uint32_t)width);
		iris_reg_add(IRIS_PROXY_ADDR + MB6, ((uint32_t)iris_cfg->meta.nrv.displayTop << 16) |
											(uint32_t)iris_cfg->meta.nrv.displayLeft);

		//iris_reg_add(IRIS_PROXY_ADDR + MB1, )
		//iris_reg_add(IRIS_PROXY_ADDR + MB2, 1<<10);
		if (iris_cfg->meta.nrv.nrvEnable)
			g_mfd->iris_conf.frc_path |= BIT(10);
		else
			g_mfd->iris_conf.frc_path &= ~BIT(10);
		iris_cfg->nrv_enable = iris_cfg->meta.nrv.nrvEnable;
	}
}

int iris_calc_meta(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;
	struct mdss_panel_data *pdata = mdp5_data->ctl->panel_data;
	u32 fps, fcnt, lcnt;
	u32 vper, hper;
	int ret = 0, te_flag = 0;

	if ((atomic_read(&mfd->iris_conf.mode_switch_cnt))) {
		iris_proc_vts(iris_cfg);
		return ret;
	}

	if (!debug_send_meta_enabled)
		return ret;
	// TODO
	//if (iris_cfg->current_mode != IRIS_MEMC_MODE)
	//	return 0;

	fps = mdss_panel_get_framerate(&pdata->panel_info);
	vper = mdss_panel_get_vtotal(&pdata->panel_info);
	hper = mdss_panel_get_htotal(&pdata->panel_info, false);

	if (fps == 0)
		return ret;

	lcnt = 0;//ctl->read_line_cnt_fnc(ctl);
	if (pdata->panel_info.type == MIPI_CMD_PANEL) {
		fcnt = mdss_mdp_cmd_vsync_count(ctl);
		iris_proc_sts(iris_cfg, fps, lcnt, vper);
		iris_proc_restart(iris_cfg);
		iris_proc_te(iris_cfg, fcnt, lcnt, fps, vper, hper);
		te_flag = iris_vary_te(iris_cfg, fcnt, (int)vper);
		iris_proc_ct(iris_cfg, fps);
		iris_proc_vp(iris_cfg);
		ret = iris_proc_vts(iris_cfg);
	} else if (pdata->panel_info.type == MIPI_VIDEO_PANEL) {
		fcnt = mdss_mdp_video_vsync_count(ctl);
		iris_proc_sts(iris_cfg, fps, lcnt, vper);
		iris_proc_restart(iris_cfg);
		ret = iris_proc_vts(iris_cfg);
	}

	if (pdata->panel_info.type == MIPI_CMD_PANEL)
		iris_set_te(iris_cfg, te_flag);

	iris_set_dtg(iris_cfg);

	pr_debug("sts=%u fps=%u fcnt=%u vts=%u restart=%u in_ratio=%u out_ratio=%u\n", iris_cfg->meta.sys_ts, fps, fcnt,
		iris_cfg->meta.video_ts, iris_cfg->meta.restart, iris_cfg->in_ratio, iris_cfg->out_ratio);

	memset((void *)&iris_cfg->meta, 0, sizeof(struct iris_meta));

	return ret;
}

void iris_send_frame(struct iris_config *iris_cfg);
void iris_show_grid_line(int show);

int iris_configure_osd(struct msm_fb_data_type *mfd)
{
#ifdef ENABLE_SUPPORT_MEMC_WITH_GRID_LINE
    struct iris_config *iris_cfg = &mfd->iris_conf;
    static int maxDeferFrameCount = 3;
    static int defercounter = 0;
#ifdef _DEBUG_DEFER_FRAME_MAX_COUNT_
       if(maxDeferFrameCount == 0)
        {
            maxDeferFrameCount = 8;
        }
#endif

        if(defercounter == 0)
        {
            //reset
            defercounter = maxDeferFrameCount;
        }

        if (g_sendframe
            && (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC)
            && (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {   // enable only after in stable MEMC
                 defercounter --;
                  if(defercounter == 0)
                   {
#ifdef _DEBUG_DEFER_FRAME_MAX_COUNT_
                    //defer several video frames for MEMC to be stable, so as to avoid garbage occurred when dowloading grid patttern data.
                    pr_debug("maxDeferFrameCount = %d\n", maxDeferFrameCount);
                     maxDeferFrameCount --;
#endif
		    iris_send_frame(iris_cfg);
                   }
	}

        if(g_gridpattern_downloaded
            && (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC)) {
            if(!g_gridpattern_showing && g_gridpattern_show_required) { //if not showing but required, show it.
                iris_show_grid_line(1);
            }
            else if(g_gridpattern_showing && !g_gridpattern_show_required) { // if showing but not required, hide it.
                iris_show_grid_line(0);
            }
         }
        else {
            g_gridpattern_showing = false;
            g_gridpattern_downloaded = false;

            if(g_gridpattern_show_required) { //if required, but not in FRC mode, then retain this task and try again later.
                g_sendframe = true;
             }
         }
#endif
        return 0;
}


int iris_set_configure(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;

	if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_PREPARE ||
		iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_PREPARE_DONE)
		return 0;

	if (first_boot) {
		iris_update_configure();
		first_boot = 0;
	}

	// no update
	if (!pq_setting_update && !dbc_setting_update && !LPMemc_setting_update
			&& !black_border_update)
		return 0;

	mutex_lock(&iris_cfg->config_mutex);
	// PQ setting, MB3
	if (pq_setting_update) {
		iris_reg_add(IRIS_PQ_SETTING_ADDR, *((u32 *)&pq_setting_current));
		pq_setting_current.peakingUpdate = 0;
		pq_setting_current.sharpnessUpdate = 0;
		pq_setting_current.memcDemoUpdate = 0;
		pq_setting_current.peakingDemoUpdate = 0;
		pq_setting_current.gammeUpdate = 0;
		pq_setting_current.memcLevelUpdate = 0;
		pq_setting_current.contrastUpdate = 0;
		pq_setting_update = false;
	}

	// DBC setting, MB5
	if (dbc_setting_update) {
		iris_reg_add(IRIS_PROXY_ADDR + 0x28, *((u32 *)&dbc_setting_current));
		dbc_setting_current.dbcUpdate = 0;
		dbc_setting_update = false;
	}

	if (LPMemc_setting_update) {
		iris_reg_add(IRIS_PROXY_ADDR + 0x20, LPMemc_setting_current.value | 0x80000000);
		LPMemc_setting_current.update = 0;
		LPMemc_setting_update = false;
	}

	if (black_border_update) {
		iris_reg_add(IRIS_MVC_ADDR + 0x17c, black_border_value ? 0x100c201f : 0x1006201f);
		black_border_update = false;
	}

	if (color_adjust_update) {
		iris_reg_add(IRIS_PROXY_ADDR + 0x30, color_adjust_current_value);
		color_adjust_update = false;
	}
	mutex_unlock(&iris_cfg->config_mutex);

	return 0;
}

#define LPMeMcLevel   (5)
#define IMG1080P_HSIZE   (1920)
#define IMG1080P_VSIZE   (1080)
#define IMG720P_HSIZE    (1280)
#define IMG720P_VSIZE    (720)

static uint16_t m_sLPMeMCLevelSelection_16to10[LPMeMcLevel][2] = {
	{1728, 1080},  //16:10
	{1280, 800},    //16:10
	{1024, 640},
	{640, 400},
	{480, 300}
};

// remove 1920*1080 frc size, bad performance
static uint16_t m_sLPMeMCLevelSelection_16to9[LPMeMcLevel][2] = {
	{1280, 720},    //16:9
	{1024, 576},
	{960, 540},
	{640, 360},
	{480, 270}
};

static uint16_t m_sLPMeMCLevelSelection_4to3[LPMeMcLevel][2] = {
	{1200, 900},//{1600, 1200},   //4:3
	{1024, 768},  //4:3
	{800, 600},    //4:3
	{640, 480},
	{480, 360}
};


#define LPRatio_NUM  (8)
//----- 6/8, 5/8, 4/8, 3/8, 2/8---
static uint16_t m_sLPMeMcRation720Over[LPMeMcLevel] = {
	8,
	6,
	5,
	4,
	3
};

static uint16_t m_sLPMeMcRationSmall[LPMeMcLevel] = {
	8,
	8,
	6,
	5,
	4
};

#ifdef ENABLE_IRIS2_480X800_PANEL
#define IRIS_DTG_HRES_SETTING 480
#define IRIS_DTG_VRES_SETTING 800
#else
#define IRIS_DTG_HRES_SETTING 768
#define IRIS_DTG_VRES_SETTING 2048
#endif
static u16 iris_LPMeMcTiming[] = {IRIS_DTG_HRES_SETTING, IRIS_DTG_VRES_SETTING};

#define IRIS_DTG_HRES_CAMERA_MEMC  648
#define IRIS_DTG_VRES_CAMERA_MEMC   1152
static u16 iris_LPCameraMeMcTiming[] = {IRIS_DTG_HRES_CAMERA_MEMC, IRIS_DTG_VRES_CAMERA_MEMC};


static u32 iris_LowPowerMemcFrcCal(u32 value)
{
	struct iris_timing_para *iris_timing;
	//u8 uratio4to3;
	uint32_t uHres;//, uVres;
	uint32_t uFrcLPTiming = (uint32_t)iris_LPMeMcTiming[1] << 16 | (uint32_t)iris_LPMeMcTiming[0];

	iris_timing = &(iris_mipi_info.iris_in_timing);
	uHres = iris_mipi_info.mipi_mode.rx_ch ? (uint32_t)iris_timing->hres * 2 : (uint32_t)iris_timing->hres;

	if (value >= LPMeMcLevel) {
		value = LPMeMcLevel - 1;
		pr_debug("#### %s:%d, Low Power MEMC level is out of range.\n", __func__, __LINE__);
	}
	pr_debug("#### %s:%d, Low Power MEMC level uHres = %d, vres = %d, rx_ch = %d.\n", __func__, __LINE__, uHres, iris_timing->vres, iris_mipi_info.mipi_mode.rx_ch);
	if (((uint32_t)uHres * (uint32_t)iris_timing->vres) >= ((uint32_t) IMG1080P_HSIZE * (uint32_t) IMG1080P_VSIZE)) {
		if (uHres > iris_timing->vres) {
			if ((uHres / 4)  == (iris_timing->vres / 3)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_4to3[value][0];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_4to3[value][1];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_4to3[value-1][0];
					iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_4to3[value-1][1];
				}
			} else if ((uHres / 16)  == (iris_timing->vres / 10)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to10[value][0];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to10[value][1];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to10[value-1][0];
					iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to10[value-1][1];
				}
			} else {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to9[value][0];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to9[value][1];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to9[value-1][0];
					iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to9[value-1][1];
				}
			}
		} else {
			if ((uHres / 3)  == (iris_timing->vres / 4)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_4to3[value][1];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_4to3[value][0];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] =  m_sLPMeMCLevelSelection_4to3[value-1][1];
					iris_LPMeMcTiming[1] =  m_sLPMeMCLevelSelection_4to3[value-1][0];
				}
			} else if ((uHres / 10)  == (iris_timing->vres / 16)) {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to10[value][1];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to10[value][0];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] =  m_sLPMeMCLevelSelection_16to10[value-1][1];
					iris_LPMeMcTiming[1] =  m_sLPMeMCLevelSelection_16to10[value-1][0];
				}
			} else {
				iris_LPMeMcTiming[0] = m_sLPMeMCLevelSelection_16to9[value][1];
				iris_LPMeMcTiming[1] = m_sLPMeMCLevelSelection_16to9[value][0];
				if (uHres*10 / (uint32_t)iris_LPMeMcTiming[0] > 40) {
					iris_LPMeMcTiming[0] =  m_sLPMeMCLevelSelection_16to9[value-1][1];
					iris_LPMeMcTiming[1] =  m_sLPMeMCLevelSelection_16to9[value-1][0];
				}
			}
		}
	} else if ((uHres * (uint32_t)iris_timing->vres) > ((uint32_t) IMG720P_HSIZE * (uint32_t) IMG720P_VSIZE)) {
		iris_LPMeMcTiming[0] = (uint32_t)uHres * m_sLPMeMcRation720Over[value] / LPRatio_NUM;
		iris_LPMeMcTiming[1] = (uint32_t)iris_timing->vres * m_sLPMeMcRation720Over[value] / LPRatio_NUM;
	} else {
		iris_LPMeMcTiming[0] = (uint32_t)uHres * m_sLPMeMcRationSmall[value] / LPRatio_NUM;
		iris_LPMeMcTiming[1] = (uint32_t)iris_timing->vres * m_sLPMeMcRationSmall[value] / LPRatio_NUM;
	}

	uFrcLPTiming = (uint32_t)iris_LPMeMcTiming[1] << 16 | (uint32_t)iris_LPMeMcTiming[0];
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	guFrcLPTiming = uFrcLPTiming;
#endif
	pr_debug("#### %s:%d,wHres: %d, wVres: %d, uFrcLPTiming: 0x%x\n", __func__, __LINE__, iris_LPMeMcTiming[0], iris_LPMeMcTiming[1], uFrcLPTiming);

	return uFrcLPTiming;
}

static void iris_set_scanline(struct iris_config *iris_cfg)
{
	u32 scanline;
	u32 m_hres = iris_LPMeMcTiming[0];
	u32 m_vres = iris_LPMeMcTiming[1];
	bool hmode = (m_hres*m_vres > IMG720P_HSIZE*IMG720P_VSIZE) ? true : false;
	struct iris_timing_para iris_dtg = iris_mipi_info.iris_out_timing;
	u32 vtotal = iris_dtg.vfp + iris_dtg.vsw + iris_dtg.vbp + iris_dtg.vres;
	u32 val_frcc_reg17 = iris_cfg->val_frcc_reg17;

	if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC) {
		// scanline =  (hmode == true)? (vtotal/5): (vtotal - IRIS_DTG_EVS_DLY);
		scanline = vtotal / 5;
	} else {
		scanline = vtotal - IRIS_DTG_EVS_DLY;
		val_frcc_reg17 &= 0xffff8000;
		val_frcc_reg17 |= 50;	// frcc_input_hw_meta0/hw_input_meta_vd_cf_idx will keep at 0
	}

	mutex_lock(&iris_cfg->cmd_mutex);
	// TE_CTRL
	iris_reg_add(IRIS_DTG_ADDR + 0x00034, (0xd | (scanline << 16)));
	// DTG_CTRL1
	//iris_reg_add(IRIS_DTG_ADDR + 0x00044, 0x3c01);
	// DVS_CTRL
	//iris_reg_add(IRIS_DTG_ADDR + 0x00064, (vtotal << 8));
	iris_reg_add(IRIS_DTG_ADDR + 0x10000, 1);   //reg_update
	iris_reg_add(FRCC_CTRL_REG17_ADDR, val_frcc_reg17);
	iris_reg_add(FRCC_REG_SHOW, 0x2);

	mutex_unlock(&iris_cfg->cmd_mutex);
	pr_debug("scanline %d m_hres %d m_vres %d hmode %i\n", scanline, m_hres, m_vres, hmode);
}

static int irisDsiStsGet(struct mdss_dsi_ctrl_pdata *ctrl)
{
	//8094_TODO;
	return 0;
}

static void iris_fiSearchRangeTop(void)
{
	m_fiVrangeTop = 0x40000 / (u32)iris_LPMeMcTiming[0] - 4;

	if (m_fiVrangeTop > 510)
		m_fiVrangeTop = 510;

	m_fiVrangeTop = m_fiVrangeTop / 2 - 1;

	pr_debug("iris_fiSearchRangeTop: %d\n", m_fiVrangeTop);

}
int iris_MEMC_reg_write(struct msm_fb_data_type *mfd, struct demo_win_info_for_FI sdemowinFI_setting_default)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	int displaywidth = (iris_mipi_info.mipi_mode.tx_ch  ?  iris_mipi_info.iris_out_timing.hres * 2 : iris_mipi_info.iris_out_timing.hres);
	int /*width, height,*/ frcEndX, frcEndY, frcStartX, frcStartY;
	static int lpmMemcHres, lpmMemcVres;

	if ((iris_LPMeMcTiming[0] != lpmMemcHres) || (iris_LPMeMcTiming[1] != lpmMemcVres)) {
		demo_win_FI_update = true;
		lpmMemcHres = iris_LPMeMcTiming[0];
		lpmMemcVres = iris_LPMeMcTiming[1];
	}

	pr_debug("iris_cfg->nrv_enable: %d\n", iris_cfg->nrv_enable);
	if (demo_win_FI_update && (pq_setting_current.memcDemo == 2)) {
		demo_win_FI_update = false;
		if (!iris_cfg->nrv_enable) {
			frcStartX = demo_win_info_setting.startX * iris_LPMeMcTiming[0] / displaywidth;
			frcStartY = demo_win_info_setting.startY * iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;
			frcEndX = demo_win_info_setting.endX *  iris_LPMeMcTiming[0] / displaywidth;
			frcEndY = demo_win_info_setting.endY *  iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;

			if (frcEndY + demo_win_info_setting.BorderWidth >= iris_LPMeMcTiming[1])
				frcEndY = iris_LPMeMcTiming[1] - demo_win_info_setting.BorderWidth;

			pr_debug("iris: %s: frcStartX = %d, frcStartY = %d, frcEndX = %d, frcEndY = %d, iris_LPMeMcTiming[0] = %d, iris_LPMeMcTiming[1] = %d.\n",
					__func__, frcStartX, frcStartY, frcEndX, frcEndY, iris_LPMeMcTiming[0], iris_LPMeMcTiming[1]);
			sdemowinFI_setting_default.colsize = (frcStartX & 0xfff) | ((frcEndX & 0xfff) << 16);
			sdemowinFI_setting_default.rowsize = (frcStartY & 0xfff) | ((frcEndY & 0xfff) << 16);
		}

		iris_reg_add(FI_DEMO_COL_SIZE, sdemowinFI_setting_default.colsize);
		iris_reg_add(FI_DEMO_MODE_RING, sdemowinFI_setting_default.color);
		iris_reg_add(FI_DEMO_ROW_SIZE, sdemowinFI_setting_default.rowsize);
		iris_reg_add(FI_DEMO_MODE_CTRL, sdemowinFI_setting_default.modectrl);
		iris_reg_add(FI_SHADOW_UPDATE, 1);
	}
	return 0;
}

void iris_mipirx_addr_set(struct mdss_dsi_ctrl_pdata *ctrl, u16 column, u16 page)
{
	char mem_addr[2] = {0x36, 0x0};
	char pixel_format[2] = {0x3a, 0x77};
	char col_addr[5] = {0x2a, 0x00, 0x00, 0x03, 0xff};
	char page_addr[5] = {0x2b, 0x00, 0x00, 0x03, 0xff};
	struct dsi_cmd_desc iris2_set_addr_cmd[] = {
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0, sizeof(mem_addr)}, mem_addr},
		{{DTYPE_DCS_WRITE1, 0, 0, 0, 0, sizeof(pixel_format)}, pixel_format},
		{{DTYPE_DCS_LWRITE, 0, 0, 0, 0, sizeof(col_addr)}, col_addr},
		{{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(page_addr)}, page_addr},
	};
	struct dsi_panel_cmds panel_cmds;

	col_addr[3] = (column >> 8) & 0xff;
	col_addr[4] = column & 0xff;
	page_addr[3] = (page >> 8) & 0xff;
	page_addr[4] = page & 0xff;

	pr_debug("iris: set mipirx addr: %x, %x\n", column, page);

	panel_cmds.cmds = iris2_set_addr_cmd;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris2_set_addr_cmd);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

#define IRISALIGN(x,y) ((((x)+(y)-1)/(y))*(y))
void iris_pwil_capen(struct mdss_dsi_ctrl_pdata *ctrl, bool enable)
{
	char pwil_capen[] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x00000003),	// valid word number
		0x00,			// burst mode
		0x00,			// reserved
		PWIL_U16(0x02),	// burst length
		PWIL_U32(IRIS_PWIL_ADDR + 0x0218),	//CAPEN
		PWIL_U32(0xc0000001)
	};
	struct dsi_cmd_desc disable_pwil_capen[] = {
		{ {DTYPE_GEN_LWRITE, 1, 0, 0, 0,  sizeof(pwil_capen)}, pwil_capen}
	};
	struct dsi_panel_cmds panel_cmds;

	if (enable)
		pwil_capen[20] = 0x03;

	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);
	panel_cmds.cmds = disable_pwil_capen;
	panel_cmds.cmd_cnt = ARRAY_SIZE(disable_pwil_capen);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}
int iris_send_frame_data(struct mdss_dsi_ctrl_pdata *ctrl,
		const u8 *data, size_t size)
{
	//for iris2, host send cmd video data to mipi_rx, mipi_rx re-organize data as pb protocol to pwil.
	//pwil capture size should larger than data. default size is 1920x1080x3(bytes), and appcode size is 128K
	//iris2 firmware download mode only use signal channel, cmd mode.
	char fw_download_pb_meta[] = {0x04, 0x20}; //for appcode download mode.
	char fw_download_configure[] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x000000013),	// valid word number
		0x00,			// burst mode
		0x00,			// reserved
		PWIL_U16(0x0012),	// burst length
		PWIL_U32(IRIS_PWIL_ADDR + 0x0004),  //PWIL ctrl1 confirm transfer mode and cmd mode, single channel.
		PWIL_U32(0x0004309a),
		PWIL_U32(IRIS_PWIL_ADDR + 0x0218),  //CAPEN
		PWIL_U32(0xc0000003),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1140),  //channel order..
		PWIL_U32(0xc6120010),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1144),  //pixelformat.
		PWIL_U32(0x888),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1158), //mem addr.
		PWIL_U32(0x00240000), //0x00240000, 0x0023ff00
		PWIL_U32(IRIS_PWIL_ADDR + 0x10000), //update setting. using SW update mode.
		PWIL_U32(0x00000100),
		PWIL_U32(IRIS_PWIL_ADDR + 0x1fff0), //clear down load int.
		PWIL_U32(0x00008000),//bit 15
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0xc), //mipi_rx setting DBI_bus.
		PWIL_U32(0x000f0000), //only for signal channel.
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0x0018), //mipi_rx time out threshold.
		PWIL_U32(0xffffffff) //only for signal channel.
	};

	struct dsi_cmd_desc appcode_download_PWIL_set[] = {
		//set PWIL mif registers, include pixelformat, mem addr and so on.
		{ {DTYPE_GEN_LWRITE, 1, 0, 0, 0,  sizeof(fw_download_configure)}, fw_download_configure},
		//set mipi_rx meta to switch PWIL capture mode to mcu mode, using generic short write(0x23), 2 useful bytes.
		{ {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(fw_download_pb_meta)}, fw_download_pb_meta},
	};

	u32 cmd_index = 0, buf_index = 0, threshold = 0;
	u8 *buf;
	size_t len = 0, cnt = 0;
	int pending_len = 0;

	struct dsi_panel_cmds panel_cmds;
	//the resolution is total send data size.  every sending data number should be equal packet_len(which is <= MAX_packet_size).
	//if mipi_rx received data less than resoluiton, it will add 0 in the last.
	char set_pixelformat[2] = {0x3a, 0x77}; //set ==mipi_rx==,  the data format is 8bit.
	char set_mem_addr[2] = {0x36, 0x0}; //set ==mipi_rx==,  write data is top to bottom, left to right.
	char set_col_addr[5] = {0x2A, 0x00, 0x00, 0x03, 0xFF};  //set ==mipi_rx==, imgae resolution, width-1
	char set_page_addr[5] = {0x2B, 0x00, 0x00, 0x03, 0xFF};  //set ==mipi_rx==, image resolution, height-1.

	char iris_mipirx_pwilcmd_mode[1] = {0x7f};
	struct dsi_cmd_desc iris_mipi_rx_mode[] = {
		{ { DTYPE_GEN_WRITE1, 1, 0, 0, 0,   sizeof(iris_mipirx_pwilcmd_mode) }, iris_mipirx_pwilcmd_mode},
	};
	struct dsi_cmd_desc iris2_set_addr_cmd[] = {
		{ { DTYPE_DCS_WRITE1, 0, 0, 0, 0,  sizeof(set_mem_addr) }, set_mem_addr},
		{ { DTYPE_DCS_WRITE1, 0, 0, 0, 0,  sizeof(set_pixelformat) }, set_pixelformat},
		{ { DTYPE_DCS_LWRITE, 0, 0, 0, 0, sizeof(set_col_addr) }, set_col_addr},
		{ { DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(set_page_addr) }, set_page_addr},
	};
	static struct dsi_cmd_desc iris2_send_fw[FW_DW_CNT_IRIS2];
	unsigned long long start_ns, dur_ns;

	//mipi_rx should send firmware as video data on cmd mode.
	//host send every packet size should be pixel_numbers *3 bytes (8bit, so every pixel has 3bytes)
#define MAX_PACKET_SIZE  (256) //speed up is 512
#define TIME_INTERVAL (20000) //larger value to protect different case, video_signel(1635), cmd_single(2300). us.

	int packet_len, i, end_col, end_page;
	size_t total_cnt, pxl_cnt;

	for (i=(MAX_PACKET_SIZE/3); i>0;i--) {
		packet_len = i*3;
		if (!(size%packet_len))
			break;
	}
	if (i <= 0)
		pr_err("==========>fatal error! cant find suitable packe_len<=========\n");
	end_col = packet_len/3;
	pxl_cnt = size/3;
	end_page = pxl_cnt/end_col;
	total_cnt = IRISALIGN(size, packet_len)/packet_len;

	while(end_page > 640) { //should not be greater than binary_v_capture_height defined in 0xf124114c
		end_col+=1;
		while(pxl_cnt%end_col)
			end_col+=1;
		end_page = pxl_cnt/end_col;
	}
	if (end_col*end_page != pxl_cnt)
		pr_err("==========>fatal error!<=========\n");
	end_col-=1;
	end_page-=1;

	threshold = ctrl->pclk_rate/1000000;
	threshold *= TIME_INTERVAL;//avoid data overflow
	pr_debug("%s: %d, pclk = %d, threshold = %d\n", __func__, __LINE__, ctrl->pclk_rate, threshold);
	fw_download_configure[84] = (__u8)(threshold & 0xff);
	fw_download_configure[85] = (__u8)((threshold >> 8) & 0xff);
	fw_download_configure[86] = (__u8)((threshold >> 16) & 0xff);
	fw_download_configure[87] = (__u8)((threshold >> 24) & 0xff);

#ifdef DEBUG_PATTERN_DATA_INTEGRITY
    if(1)
      {
            pr_debug("tail   byte buffer:(offset %d) [0x%x]\n", (end_page*(end_col+1)*3) + end_col*3 + 1, *(data+(end_page*(end_col+1)*3) + end_col*3 + 1) );
            pr_debug("final byte buffer:(offset %d) [0x%x]\n", (end_page*(end_col+1)*3) + end_col*3 + 2, *(data+(end_page*(end_col+1)*3) + end_col*3 + 2) );
            //pr_debug("tail   byte buffer:(offset %d) [0x%x]\n", (1151*648*2) + 647*2, *(data+(1151*648*2) + 647*2) );
            //pr_debug("final byte buffer:(offset %d) [0x%x]\n", (1151*648*2) + 647*2 + 1, *(data+(1151*648*2) + 647*2 +1) );
      }
#endif

	//change mipi_rx to cmd mode firstly if rx is not cmd mode
	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode) {
		panel_cmds.cmds = iris_mipi_rx_mode;
		panel_cmds.cmd_cnt = ARRAY_SIZE(iris_mipi_rx_mode);
		panel_cmds.link_state = DSI_LP_MODE;
		mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
	}

	set_col_addr[3] = (end_col >> 8) & 0xFF;
	set_col_addr[4] = end_col & 0xFF;
	set_page_addr[3] = (end_page >> 8) & 0xFF;
	set_page_addr[4] = end_page & 0xFF;

	pr_info("mipi_rx setting, col_addr=%d, page_addr=%d, iris2 appcode!size = %zu\n", end_col, end_page, size);

	memset(iris2_send_fw, 0, sizeof(iris2_send_fw));

	if(g_firmware_buf)
		buf = g_firmware_buf;
	else
		return -1;

	//all mode should be send by using hs mode when mipi_rx is working on cmd mode.
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);

	//setting PWIL
	panel_cmds.cmds = appcode_download_PWIL_set;
	panel_cmds.cmd_cnt = ARRAY_SIZE(appcode_download_PWIL_set);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	//setting mipi_rx
	panel_cmds.cmds = iris2_set_addr_cmd;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris2_set_addr_cmd);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	// add timing measure and printk for fw download command only
	// (six other small commands are send seperately
	pr_info("%s: %d, total_cnt = %zu, packet_len %d\n",
		__func__, __LINE__, total_cnt, packet_len);
	start_ns = sched_clock();

	while (size) {
		if (size >= packet_len)
			len = packet_len;
		else {
			len = size;
			pending_len = packet_len - len;
		}

		cnt++;
		if ((cnt % FW_DW_CNT_IRIS2) == 0)
			cmd_index = FW_DW_CNT_IRIS2 - 1;
		else
			cmd_index = cnt % FW_DW_CNT_IRIS2 - 1;

		/*here should be the previously packet len, but for this case,
		only the last one is small than patcket_len, the others are same.
		so we used packet_len to replace previous packet len.
		*/
		buf_index = cmd_index * (packet_len + 1);

		if (1 == cnt)
			buf[0] = DCS_WRITE_MEM_START;
		else
			buf[buf_index] = DCS_WRITE_MEM_CONTINUE;

		memcpy(buf + buf_index + 1, data, len);

		iris2_send_fw[cmd_index].dchdr.last = 0;
		iris2_send_fw[cmd_index].dchdr.dtype = 0x39;
		iris2_send_fw[cmd_index].dchdr.dlen = packet_len + 1; //add buf[0]
		iris2_send_fw[cmd_index].payload = buf + buf_index;

		if ((cmd_index == (FW_DW_CNT_IRIS2 - 1)) || (cnt == total_cnt)) {
			iris2_send_fw[cmd_index].dchdr.last = 1;
			if (cnt == total_cnt)
				iris2_send_fw[cmd_index].dchdr.wait = 0; //IRIS_CMD_FIFO_EMPTY;
			panel_cmds.cmds = iris2_send_fw;
			panel_cmds.cmd_cnt = (cmd_index+1);
			panel_cmds.link_state = DSI_HS_MODE;
			mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
		}

		data += len;
		size -= len;

	}

	dur_ns = sched_clock() - start_ns;
	do_div(dur_ns, 1000);
	pr_info("%s: %d, fw download took %llu usec\n",
		__func__, __LINE__, dur_ns);
	//iris_pwil_capen(ctrl,0);
         pr_info("remove me: capture disabled always!\n");
	return 0;
}

void iris_send_frame_data_done(struct mdss_dsi_ctrl_pdata *ctrl)
{
	//restore mipi_rx setting, PB meta, now mipi rx is working on pwil_cmd mode
	char pb_meta[] = {0x04, 0x02}; // frc mode download
	char fw_after_conf[] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x00000007),
		0x00,
		0x00,
		PWIL_U16(0x06),
		PWIL_U32(IRIS_PWIL_ADDR + 0x0004),
		PWIL_U32(0x0004301b),//need check here!!!
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0xc), //mipi_rx setting DBI_bus.
		PWIL_U32(0x000f0000), //only for signal channel.
		PWIL_U32(IRIS_MIPI_RX_ADDR + 0x0018), //mipi_rx time out threshold.
		PWIL_U32(0x0000077f)
	};
	struct dsi_cmd_desc iris2_fw_restore[] = {
		//{ {DTYPE_GEN_WRITE2, 1, 0, 0, 1, sizeof(pb_meta)}, pb_meta},
		{ {DTYPE_GEN_LWRITE, 1, 0, 0, 0,  sizeof(fw_after_conf)}, fw_after_conf}
	};
	struct dsi_cmd_desc iris2_fw_restore_meta[] = {
		{ {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(pb_meta)}, pb_meta}
	};
	struct dsi_panel_cmds panel_cmds;
	u32 threshold = 0, col_addr = 0, page_addr = 0;

	//confirm pwil work mode, video or cmd.
	if (IRIS_MIPIRX_VIDEO == iris_mipi_info.mipi_mode.rx_mode)
		fw_after_conf[20] = 0x18;
	else
		fw_after_conf[20] = 0x1a;

	if (1 == iris_mipi_info.mipi_mode.rx_ch) {
		fw_after_conf[20] += 1;
		fw_after_conf[30] = 0x8f;
	}

	if (1 == iris_mipi_info.mipi_mode.rx_ch) {
		threshold = iris_mipi_info.iris_in_timing.hres * 2 - 1;
		col_addr = iris_mipi_info.iris_in_timing.hres * 2 - 1;
	} else {
		threshold = iris_mipi_info.iris_in_timing.hres - 1;
		col_addr = iris_mipi_info.iris_in_timing.hres - 1;
	}
	page_addr = iris_mipi_info.iris_in_timing.vres - 1;
	*(u32 *)(fw_after_conf + 36) = cpu_to_le32(threshold);

	panel_cmds.cmds = iris2_fw_restore;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris2_fw_restore);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

	if (IRIS_MIPIRX_CMD == iris_mipi_info.mipi_mode.rx_mode)
		iris_mipirx_addr_set(ctrl, col_addr, page_addr);

	panel_cmds.cmds = iris2_fw_restore_meta;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris2_fw_restore_meta);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
}

void iris_transfer_frame(u8 *data, u32 size)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	iris_pwil_capen(ctrl,0);
	msleep(20);
	iris_send_frame_data(ctrl, data, size);
	iris_send_frame_data_done(ctrl);
}

void iris_draw_grid_line(u16* addr)
{
	int i,j;
	int width = iris_LPCameraMeMcTiming[0];
	int height = iris_LPCameraMeMcTiming[1];
	//Sephy, 3x3 test, will do nxn, specical shape etc. add input parameters union.
	int n = f_grid_size;
         u16 color = f_gride_line_color;

    int VSpan = height/n;
    int HSpan = width/n;
    int totalRawPixels = 0;
    int totalColumPixels = 0;

    for (i=1; i< n; i ++)
       for (j=0; j<width; j++) {
            *(addr+i*VSpan*width+j) = color;
            totalRawPixels ++;
        }

    for (i=0;i<height; i++)
        for (j=1; j<n; j++) {
            *(addr+i*width+j*HSpan) = color;
            totalColumPixels ++;
         }

#ifdef DEBUG_PATTERN_DATA_INTEGRITY
     pr_debug("[%d x %d]Vertical Span: %d, Horizontal Span %d, grid %dx%d\n", width, height, VSpan, HSpan, n, n);
     pr_debug("totalRawPixels: %d, totalColumPixels %d.set to color\n", totalRawPixels, totalColumPixels);
     if(1)
      {
            *(addr+(height - 1)*width + width - 1)  = 0xdead;
            pr_debug("tail word buffer:(%d, %d) [0x%x]\n", (width - 1), (height - 1), *(addr+(height - 1)*width + width - 1) );
      }
#endif
}

void iris_show_grid_line(int show)
{
    if(show > 0) {
        //Set MB3(0xf0040018) as 0x00000001 to show grid line.
        iris_reg_add(IRIS_PROXY_ADDR + 0x18, 0x00000001);
        g_gridpattern_showing = true;
        pr_debug("grid line shown\n");
    }
    else {
        //Set DISP_CTRL0(0xf1241204) (bit24~bit31) as 0 to disable grid line. 12020001->00020001
        iris_reg_add(IRIS_PWIL_ADDR + 0x1204, 0x00020001);
        g_gridpattern_showing = false;
        pr_debug("grid line hiden\n");
    }
}

void iris_send_frame(struct iris_config *iris_cfg)
{
	ktime_t ktime = ktime_get();
	u64 time_start = ktime_to_us(ktime);
	u64 time_vm, time_minit, time_send, time_draw;
	static size_t osd_frame_size;

	if (!g_osd_buf) {
		osd_frame_size = sizeof(u16)*(iris_LPCameraMeMcTiming[0]*iris_LPCameraMeMcTiming[1]);
		g_osd_buf = vmalloc(osd_frame_size);
	}
	else if (osd_frame_size != (sizeof(u16)*iris_LPCameraMeMcTiming[0]*iris_LPCameraMeMcTiming[1])) {
		vfree(g_osd_buf);
		osd_frame_size = sizeof(u16)*(iris_LPCameraMeMcTiming[0]*iris_LPCameraMeMcTiming[1]);
		g_osd_buf = vmalloc(osd_frame_size);
	}

	if (!g_osd_buf) {
		pr_err("can't allocate memory for OSD!\n");
		return ;
	} else {
		ktime = ktime_get();
		time_vm = ktime_to_us(ktime);
		pr_debug("alloc memory cost %lldus\n",(time_vm - time_start));

		memset(g_osd_buf, 0x0, osd_frame_size);
		ktime = ktime_get();
		time_minit = ktime_to_us(ktime);
		pr_debug("memset cost %lldus!\n", (time_minit - time_vm));

		iris_draw_grid_line(g_osd_buf);
		ktime = ktime_get();
		time_draw = ktime_to_us(ktime);
		pr_debug("draw osd cost %lldus!\n", (time_draw - time_minit));


                if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC) {
		iris_transfer_frame((u8*)g_osd_buf, osd_frame_size);
		ktime = ktime_get();
		time_send = ktime_to_us(ktime);
		pr_debug("transfer frame data cost %lldus!\n", (time_send - time_draw));
                      pr_debug("grid pattern timing: %d x %d, pixel buffer size %d \n",
                                        iris_LPCameraMeMcTiming[0], iris_LPCameraMeMcTiming[1], (int)osd_frame_size);

                     g_sendframe = false;
                     g_gridpattern_downloaded = true;
                 }

                pr_debug("total time cost = %lldus\n", (time_send - time_start));
	}
}

int iris_configure(struct msm_fb_data_type *mfd, u32 type, u32 value)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	u32 configAddr = 0;
	u32 configValue = 0;

	struct mdss_overlay_private *mdp5_data;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl;

	mdp5_data = mfd_to_mdp5_data(mfd);
	pdata = mdp5_data->ctl->panel_data;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	pr_debug("iris_configure: %d - 0x%x\n", type, value);

	if (type >= IRIS_CONFIG_TYPE_MAX)
		return -EINVAL;

	mutex_lock(&iris_cfg->config_mutex);
	switch (type) {
	case IRIS_PEAKING:
		pq_setting_current.peaking = value & 0xf;
		pq_setting_current.peakingUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_SHARPNESS:
		pq_setting_current.sharpness = value & 0xf;
		pq_setting_current.sharpnessUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_MEMC_DEMO:
		pq_setting_current.memcDemo = value & 0x3;
		pq_setting_current.memcDemoUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_PEAKING_DEMO:
		pq_setting_current.peakingDemo = value & 0x3;
		pq_setting_current.peakingDemoUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_GAMMA:
		pq_setting_current.gamma = value & 0x3;
		pq_setting_current.gammeUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_MEMC_LEVEL:
		pq_setting_current.memcLevel = value & 0x3;
		pq_setting_current.memcLevelUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_CONTRAST:
		pq_setting_current.contrast = value & 0xff;
		pq_setting_current.contrastUpdate = 1;
		pq_setting_update = true;
		break;
	case IRIS_BRIGHTNESS:
		dbc_setting_current.brightness = value & 0x7f;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_EXTERNAL_PWM:
		dbc_setting_current.externalPWM = value & 0x1;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_DBC_QUALITY:
		dbc_setting_current.dbcQuality = value & 0xf;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_DLV_SENSITIVITY:
		dbc_setting_current.dlvSensitivity = value & 0xfff;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_DBC_CONFIG:
		dbc_setting_current = *((struct iris_dbc_setting *)&value);
		dbc_setting_update = true;
		break;
	case IRIS_PQ_CONFIG:
		pq_setting_current = *((struct iris_pq_setting *)&value);
		pq_setting_update = value;
		break;
	case IRIS_LPMEMC_CONFIG:
		LPMemc_setting_current.level = value;
		LPMemc_setting_current.value = iris_LowPowerMemcFrcCal(value);
		LPMemc_setting_update = true;
		break;
	case IRIS_DCE_LEVEL:
		dbc_setting_current.DCELevel = value & 0x0f;
		dbc_setting_current.dbcUpdate = 1;
		dbc_setting_update = true;
		break;
	case IRIS_BLACK_BORDER:
		black_border_value = value != 0;
		black_border_update = true;
		break;
	case IRIS_CINEMA_MODE:
		pq_setting_current.cinema = value & 0x01;
		pq_setting_update = true;
		break;
	case IRIS_COLOR_ADJUST:
		color_adjust_current_value = value & 0xff;
		color_adjust_update = true;
		break;
	case IRIS_ADJUST_CLK_ENABLE:
		usb_w_enabled_by_configure = (value & 0x1);
		mutex_unlock(&iris_cfg->config_mutex);
		return 0;
	case IRIS_SEND_FRAME:
		//iris_send_frame();
		g_sendframe = true;
		mutex_unlock(&iris_cfg->config_mutex);
		return 0;
        case IRIS_DBG_TARGET_PI_REGADDR_SET:
                  f_Iris_ip_reg_addr = value;
                  mutex_unlock(&iris_cfg->config_mutex);
		return 0;
        case IRIS_DBG_TARGET_REGADDR_VALUE_SET:
                  iris_pi_write(ctrl, f_Iris_ip_reg_addr, value);
                  mutex_unlock(&iris_cfg->config_mutex);
		return 0;
        case IRIS_MEMC_ENABLE_FOR_ASUS_CAMERA:
                  f_camera_memc_enable = value;
                  if(f_camera_memc_enable > 0)
                    {
                           //configure with iris_LPCameraMeMcTiming(1152x648)
                           pr_debug("#### %s:%d, current level = %d, value 0x%x.\n",
                                            __func__, __LINE__,
                                             LPMemc_setting_current.level,
                                             LPMemc_setting_current.value);
                           //= value;
			LPMemc_setting_current.value =
				(uint32_t)iris_LPCameraMeMcTiming[1] << 16 | (uint32_t)iris_LPCameraMeMcTiming[0];

                           pr_debug("#### %s:%d, changing into  value 0x%x.\n",
                                            __func__, __LINE__,
                                             LPMemc_setting_current.value);
                    }
                  else
                    {
                            g_sendframe = false;
                            g_gridpattern_show_required = false;

                           //restore with iris_LPMeMcTiming
			LPMemc_setting_current.value
				= (uint32_t)iris_LPMeMcTiming[1] << 16 | (uint32_t)iris_LPMeMcTiming[0];

                            pr_debug("#### %s:%d, restoring into  value 0x%x.\n",
                                            __func__, __LINE__,
                                             LPMemc_setting_current.value);
                    }

		LPMemc_setting_update = true; //will update when iris_set_configure() during kickoff
		mutex_unlock(&iris_cfg->config_mutex);
		return 0;
	default:
		mutex_unlock(&iris_cfg->config_mutex);
		return -EINVAL;
	}

	//update dbc mode
	if (dbc_setting_update) {
		if (dbc_setting_current.dbcQuality == 5)
			iris_dbc_mode &= ~(1 << 1);
		else
			iris_dbc_mode |= 1 << 1;
		if (dbc_setting_current.dlvSensitivity == 0)
			iris_dbc_mode &= ~1;
		else
			iris_dbc_mode |= 1;
#if 0
		if(dbc_setting_current.dbcQuality == 0x5)
		{
			iris_register_write(mfd, IRIS_SYS_ADDR+0x218, 0x10901);
		}
		else
		{
			iris_register_write(mfd, IRIS_SYS_ADDR+0x218, 0x1);
		}
		iris_register_write(mfd, IRIS_SYS_ADDR+0x10, 1);
		iris_register_write(mfd, IRIS_SYS_ADDR+0x10, 0);
#endif
	}

	// other mode, use meta method
	if (iris_cfg->sf_notify_mode != MDP_IRIS_MODE_RFB) {
		mutex_unlock(&iris_cfg->config_mutex);
		return 0;
	}

	// PQ setting, MB3
	if (pq_setting_update) {
		configValue = *((u32 *)&pq_setting_current);
		configAddr = IRIS_PQ_SETTING_ADDR;
		pq_setting_update = false;
	} else if (dbc_setting_update) {
		configValue = *((u32 *)&dbc_setting_current);
		configAddr = IRIS_PROXY_ADDR + 0x28;
		dbc_setting_update = false;
	} else if (LPMemc_setting_update) {
		configValue = LPMemc_setting_current.value | 0x80000000;
		configAddr = IRIS_PROXY_ADDR + 0x20;
		LPMemc_setting_update = false;
	} else if (black_border_update) {
		configValue = black_border_value ? 0x100c201f : 0x1006201f;
		configAddr = IRIS_MVC_ADDR + 0x17c;
		black_border_update = false;
	} else if (color_adjust_update) {
		configValue = ( u32 )color_adjust_current_value;
		configAddr = IRIS_PROXY_ADDR + 0x30;
		color_adjust_update = false;
	}
	mutex_unlock(&iris_cfg->config_mutex);

	if (0 == configValue && 0 == configAddr) {
		pr_warn(" no configValue and configAddr specified, possibly wrong type(%d)!\n", type);
		return -EINVAL;
	}

	return iris_register_write(mfd, configAddr, configValue);
}

int iris_configure_ex_osd_pattern_info(struct msm_fb_data_type *mfd, u32 *values)
{
	//struct iris_config *iris_cfg = &g_mfd->iris_conf;
	struct osd_pattern_info *posd_info;
	//int ret = -EINVAL;

        if(NULL == mfd
            || NULL == values)
        {
            return -EINVAL;
        }

        posd_info = (struct osd_pattern_info *)values;
         if(NULL == posd_info)
        {
            return -EINVAL;
        }

#ifndef ENABLE_SUPPORT_MEMC_WITH_GRID_LINE
        pr_debug("osd_pattern_info: note ENABLE_SUPPORT_MEMC_WITH_GRID_LINE is NOT defined.\n");
#endif

         pr_debug("osd_pattern_info: %d, %d, 0x%x\n", posd_info->show, posd_info->format, posd_info->color);

         f_grid_size = posd_info->format;

         f_gride_line_color = posd_info->color & 0xffff; // rgba5551

        if(posd_info->show > 0) {
            g_sendframe = true;
            g_gridpattern_show_required = true;
        }
        else {
            g_sendframe = false;
            g_gridpattern_show_required = false;
          }

         memcpy(&f_osd_pattern_info, posd_info, sizeof(struct osd_pattern_info));

        return 0;
}

int iris_configure_ex_demo_win_info(struct msm_fb_data_type *mfd, u32 *values)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	struct demo_win_info *pdemo_win_info;
	int width, height, frcEndX, frcEndY, frcStartX, frcStartY;
	int color = 0, colsize = 0, rowsize = 0, modectrl = 0x3f00, peakingctrl = 0, winstart = 0, winend = 0;
	int displaywidth = (iris_mipi_info.mipi_mode.tx_ch  ?  iris_mipi_info.iris_out_timing.hres * 2 : iris_mipi_info.iris_out_timing.hres);
	int ret;

        if(NULL == mfd
            || NULL == values)
        {
            return -EINVAL;
        }

	pdemo_win_info = (struct demo_win_info *)values;
         if(NULL == pdemo_win_info)
        {
            return -EINVAL;
        }

	memcpy(&demo_win_info_setting, values, sizeof(struct demo_win_info));
	pr_debug("%s: startx =%x, starty=%x, endx=%x, endy=%x, color=%x, boardwidth=%x, MEMCdemoEn = %x, peakingdemoEn = %x\n",
			__func__, demo_win_info_setting.startX,
			demo_win_info_setting.startY, demo_win_info_setting.endX, demo_win_info_setting.endY,
			demo_win_info_setting.color, demo_win_info_setting.BorderWidth,
			demo_win_info_setting.MEMCEn, demo_win_info_setting.SharpnessEn);

	if (displaywidth < 100 || iris_mipi_info.iris_out_timing.vres < 100) {
		pr_err("panel size too small!\n");
		return -EINVAL;
	}

	if (pdemo_win_info->startX >  displaywidth ||
			pdemo_win_info->startY >  iris_mipi_info.iris_out_timing.vres) {
		pr_err("user defined window start point over range!\n");
		return -EINVAL;
	}

	if (pdemo_win_info->endX >  displaywidth ||
		pdemo_win_info->endY >  iris_mipi_info.iris_out_timing.vres) {
		pr_err("user defined end point over range!\n");
		return -EINVAL;
	}

	if (pdemo_win_info->startX >  pdemo_win_info->endX ||
		pdemo_win_info->startY >  pdemo_win_info->endY) {
		pr_err("user defined start point > end point!\n");
		return -EINVAL;
	}

	pr_debug("iris_cfg->nrv_enable: %d\n", iris_cfg->nrv_enable);
	if (iris_cfg->nrv_enable) {
		width = iris_cfg->meta.nrv.captureRight - iris_cfg->meta.nrv.captureLeft;
		height = iris_cfg->meta.nrv.captureBottom - iris_cfg->meta.nrv.captureTop;
		frcStartX = pdemo_win_info->startX * width/displaywidth;
		frcStartY = pdemo_win_info->startY * height/iris_mipi_info.iris_out_timing.vres;
		frcEndX = pdemo_win_info->endX * width/displaywidth;
		frcEndY = pdemo_win_info->endY * height/iris_mipi_info.iris_out_timing.vres;
	} else {
		frcStartX = pdemo_win_info->startX * iris_LPMeMcTiming[0] / displaywidth;
		frcStartY = pdemo_win_info->startY * iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;
		frcEndX = pdemo_win_info->endX *  iris_LPMeMcTiming[0] / displaywidth;
		frcEndY = pdemo_win_info->endY *  iris_LPMeMcTiming[1] / iris_mipi_info.iris_out_timing.vres;
	}

	pr_debug("frc mode resolution: %d - %d - %d - %d - %d - %d\n", frcStartX, frcStartY, frcEndX, frcEndY, iris_LPMeMcTiming[0], iris_LPMeMcTiming[1]);
	if (frcEndY + pdemo_win_info->BorderWidth >= iris_LPMeMcTiming[1])
		frcEndY = iris_LPMeMcTiming[1] - pdemo_win_info->BorderWidth;
	winstart = (pdemo_win_info->startX & 0x3fff) + ((pdemo_win_info->startY & 0x3fff) << 16);
	winend =  (pdemo_win_info->endX & 0x3fff) + ((pdemo_win_info->endY & 0x3fff) << 16);

	peakingctrl = 1 | pdemo_win_info->SharpnessEn<<1;

	color = pdemo_win_info->color;

	colsize = (frcStartX & 0xfff) | ((frcEndX & 0xfff)<<16);
	rowsize = (frcStartY & 0xfff) | ((frcEndY & 0xfff)<<16);
	pr_debug("%s:BorderWidth =%x\n", __func__, pdemo_win_info->BorderWidth);
	modectrl = modectrl | pdemo_win_info->MEMCEn;
	modectrl = modectrl | 1<<1;
	modectrl = modectrl | ((pdemo_win_info->BorderWidth & 0x7)<<4);

	pr_debug("%s: COL_SIZE =%x, MODE_RING=%x, ROW_SIZE=%x, STARTWIN=%x, ENDWIN=%x, MODE_CTRL=%x, PEAKING_CTRL = %x\n",
		__func__, colsize, color, rowsize, winstart, winend, modectrl, peakingctrl);

	if (pdemo_win_info->MEMCEn) {
		//backup FI setting for demo window, because when setting demo window, the work mode may can't be MEMC mode, so
		//FI setting can't write.
		demowinFI_setting_default.colsize = colsize;
		demowinFI_setting_default.color = color;
		demowinFI_setting_default.rowsize = rowsize;
		demowinFI_setting_default.modectrl = modectrl;
		demo_win_FI_update = true;

		//set registers
		if ((iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC) ||
			(iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_PREPARE)) {
			demo_win_FI_update = false;
			iris_reg_add(FI_DEMO_COL_SIZE, colsize);
			iris_reg_add(FI_DEMO_MODE_RING, color);
			iris_reg_add(FI_DEMO_ROW_SIZE, rowsize);
			iris_reg_add(FI_DEMO_MODE_CTRL, modectrl);
			iris_reg_add(FI_SHADOW_UPDATE, 1);
		}
	}
	if (pdemo_win_info->SharpnessEn) {
		if (iris_cfg->sf_notify_mode != MDP_IRIS_MODE_RFB) {
			//mutex_lock(&iris_cfg->cmd_mutex);
			iris_reg_add(PEAKING_STARTWIN, winstart);
			iris_reg_add(PEAKING_ENDWIN, winend);
			iris_reg_add(PEAKING_CTRL, peakingctrl);
			iris_reg_add(PEAKING_SHADOW_UPDATE, 1);
			//mutex_unlock(&iris_cfg->cmd_mutex);
		} else {
			ret = iris_register_write(mfd, PEAKING_STARTWIN, winstart);
			if (ret != 0)
				return ret;
			ret = iris_register_write(mfd, PEAKING_ENDWIN, winend);
			if (ret != 0)
				return ret;
			ret = iris_register_write(mfd, PEAKING_CTRL, peakingctrl);
			if (ret != 0)
				return ret;
			ret = iris_register_write(mfd, PEAKING_SHADOW_UPDATE, 1);
			if (ret != 0)
				return ret;
		}
	}
	return 0;
}

int iris_configure_ex(struct msm_fb_data_type *mfd, u32 type, u32 count, u32 *values)
{
        struct iris_config *iris_cfg = &g_mfd->iris_conf;
        int ret =  -EINVAL;

        if (type >= IRIS_CONFIG_TYPE_MAX)
            return -EINVAL;

        if(count <= 1)
           return  iris_configure( mfd, type, *values);

        mutex_lock(&iris_cfg->config_mutex);
        switch (type) {
            case IRIS_USER_DEMO_WND:
                    {
                        ret = iris_configure_ex_demo_win_info(mfd, values);
                    }
                break;
            case IRIS_OSD_PATTERN_SHOW:
                    {
                        ret = iris_configure_ex_osd_pattern_info(mfd, values);
                    }
                break;
             default:
                pr_debug("iris_configure_ex: unsupported type %x\n", type);
                break;
            }
        mutex_unlock(&iris_cfg->config_mutex);
        return ret;
}


int iris_configure_get(struct msm_fb_data_type *mfd, u32 type, u32 count, u32 *values)
{
	int ret = 0;

	struct mdss_overlay_private *mdp5_data;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl;

	mdp5_data = mfd_to_mdp5_data(mfd);
	pdata = mdp5_data->ctl->panel_data;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	if ((type >= IRIS_CONFIG_TYPE_MAX) || (mfd->panel_power_state == MDSS_PANEL_POWER_OFF))
		return -EFAULT;

	ret = irisDsiStsGet(ctrl);
	if (ret != IRIS_CONFIGURE_GET_VALUE_CORRECT)
		return ret;

	switch (type) {
	case IRIS_PEAKING:
		*values = pq_setting_current.peaking;
		break;
	case IRIS_SHARPNESS:
		*values = pq_setting_current.sharpness;
		break;
	case IRIS_MEMC_DEMO:
		*values = pq_setting_current.memcDemo;
		break;
	case IRIS_PEAKING_DEMO:
		*values = pq_setting_current.peakingDemo;
		break;
	case IRIS_GAMMA:
		*values = pq_setting_current.gamma;
		break;
	case IRIS_MEMC_LEVEL:
		*values = pq_setting_current.memcLevel;
		break;
	case IRIS_CONTRAST:
		*values = pq_setting_current.contrast;
		break;
	case IRIS_BRIGHTNESS:
		*values = pq_setting_current.sharpness;
		break;
	case IRIS_EXTERNAL_PWM:
		*values = dbc_setting_current.externalPWM;
		break;
	case IRIS_DBC_QUALITY:
		*values = dbc_setting_current.dbcQuality;
		break;
	case IRIS_DLV_SENSITIVITY:
		*values = dbc_setting_current.dlvSensitivity;
		break;
	case IRIS_DCE_LEVEL:
		*values = dbc_setting_current.DCELevel;
		break;
	case IRIS_DBC_CONFIG:
		*values = *((u32 *)&dbc_setting_current);
		break;
	case IRIS_PQ_CONFIG:
		*values = *((u32 *)&pq_setting_current);
		break;
	case IRIS_LPMEMC_CONFIG:
		*values = LPMemc_setting_current.level;
		break;
	case IRIS_USER_DEMO_WND:
		memcpy(values, &demo_win_info_setting, count * sizeof(u32));
		break;
	case IRIS_CINEMA_MODE:
		*values = pq_setting_current.cinema;
		break;
        case IRIS_DBG_TARGET_REGADDR_VALUE_GET:
                  f_Iris_ip_reg_addr_value = iris_pi_read(ctrl, f_Iris_ip_reg_addr);
                  *values = f_Iris_ip_reg_addr_value;
                  pr_debug("addr[0x%x]value[0x%x]\n", f_Iris_ip_reg_addr, f_Iris_ip_reg_addr_value);
                  break;
	default:
		return -EFAULT;
	}
	return ret;
}

static int iris_set_repeat(struct iris_config *iris_cfg)
{
	unsigned int reg_in, reg_out;
	unsigned int val_frcc_cmd_th = iris_cfg->val_frcc_cmd_th;
	unsigned int val_frcc_reg8 = iris_cfg->val_frcc_reg8;
	unsigned int val_frcc_reg16 = iris_cfg->val_frcc_reg16;
	bool cap_enable = true;

	if (!debug_repeat_enabled || iris_cfg->sf_notify_mode != MDP_IRIS_MODE_FRC)
		return true;

	if ((iris_cfg->repeat == IRIS_REPEAT_FORCE) && (!frc_repeat_enter)) {
		reg_in = (1 << IRIS_PWIL_IN_FRAME_SHIFT) | (1 << 15);
		reg_out = 1 << IRIS_PWIL_OUT_FRAME_SHIFT;
		val_frcc_cmd_th &= 0x1fffffff;
		val_frcc_cmd_th |= 0x20000000;
		frc_repeat_enter = true;
		val_frcc_reg8 |= 0x3f00;
		val_frcc_reg16 &= 0xffff7fff;
		mutex_lock(&iris_cfg->cmd_mutex);
		iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
		iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
		iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
		iris_reg_add(FRCC_REG_SHOW, 0x2);
		iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
		iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
		iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
		mutex_unlock(&iris_cfg->cmd_mutex);
	} else if (iris_cfg->repeat != IRIS_REPEAT_FORCE) {
		reg_in = iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT | (1 << 15);
		reg_out = iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT;
		cap_enable = (iris_cfg->repeat != IRIS_REPEAT_CAPDIS);
		mutex_lock(&iris_cfg->cmd_mutex);
		if (frc_repeat_enter) {
			frc_repeat_enter = false;
			iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
			iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
			iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
			iris_reg_add(FRCC_REG_SHOW, 0x2);
			iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
			iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
			iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
		}
		mutex_unlock(&iris_cfg->cmd_mutex);
	}

	pr_debug("vts %u pvts %u cap_en %d\n", iris_cfg->meta.video_ts, iris_cfg->prev_vts, cap_enable);

	return cap_enable;
}

#define MAX_CAD_LEN 5
void iris_cmd_cadence_check(struct mdss_mdp_ctl *ctl)
{
	static u32 prev_frame_addr;
	static u32 prev_frame_count, prev_frames;
	static u32 cadence[MAX_CAD_LEN][3];
	static int count[MAX_CAD_LEN];
	static int cadence_length, cadence_count;
	static int skip;

	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(ctl->mfd);
	struct mdss_mdp_mixer *mixer;
	u32 frame_addr, frame_count, frames;
	int i;
	char *addr;

	if (!mdp5_data)
		return;

	/* another CPU could change  mdp5_data->yuv_frame_addr_reg while the ISR is running
	use volatile to protect */
	addr = ACCESS_ONCE(mdp5_data->yuv_frame_addr_reg);
	if (!addr)
		return;

	// only check the left ctl
	mixer = mdss_mdp_mixer_get(ctl, MDSS_MDP_MIXER_MUX_LEFT);
	if (!mixer)
		return;

	frame_count = (mdss_mdp_pingpong_read(mixer->pingpong_base, MDSS_MDP_REG_PP_INT_COUNT_VAL) >> 16) & 0xffff;
	frames = frame_count - prev_frame_count;
	frame_addr = readl_relaxed(addr);
	//pr_debug("=== frame %08x count %u diff %u\n",
	//	 frame_addr, frame_count, frames);
	if (frame_addr == prev_frame_addr)
		return;
	prev_frame_addr = frame_addr;
	prev_frame_count = frame_count;
	ATRACE_INT("KFrameDiff", frames);
	for (i = 0; i < MAX_CAD_LEN; i++)
		cadence[i][0] = ((cadence[i][0] << 4) | (frames & 0x0f)) &
				GENMASK((i + 1) * 4 - 1, 0);
	if (cadence_length && ++cadence_count >= cadence_length) {
		cadence_count = 0;
		i = cadence_length - 1;
		if (cadence[i][0] != cadence[i][1]) {
			ATRACE_INT("CadenceSkip", skip);
			skip = !skip;
			pr_debug("=== ctl %u frame %08x count %u: "
				 "cadence skip l %d (%08x %08x)\n",
				 ctl->num, frame_addr, frame_count,
				 cadence_length, cadence[i][1], cadence[i][0]);
			cadence_length = 0;
		}
	}
	if (!cadence_length) {
		for (i = 0; i < MAX_CAD_LEN; i++) {
			if (cadence[i][0] == cadence[i][1] &&
			    cadence[i][0] == cadence[i][2])
				cadence_length = i + 1;
		}
	}
	for (i = 0; i < MAX_CAD_LEN; i++) {
		if (++count[i] > i) {
			cadence[i][2] = cadence[i][1];
			cadence[i][1] = cadence[i][0];
			count[i] = 0;
		}
	}
	prev_frames = frames;
}

static bool is_memc_doable(int vp_num, int gp_num)
{
	int ret = (vp_num == 1) ? 1 : 0;
	return ret;
}

static int check_mode_status(struct mdss_dsi_ctrl_pdata *ctrl, int mode)
{
	int i;
	u32 val = 0;
	int try_cnt = 10;
	int ret = 0;

	if (!debug_mode_switch_enabled)
		return ret;

	for (i = 0; i < try_cnt; i++) {
		msleep(16);
		val = iris_pi_read(ctrl, IRIS_PROXY_ADDR + 0x08);
		if (val == mode)
			break;
		pr_err("%s:%d: %08x, cnt = %d\n", __func__, __LINE__, val, i);

	}

	if (i == try_cnt) {
		pr_err("%s: check mode (%d) error\n", __func__, mode);
		ret = -1;
	}
	return ret;
}

static void iris_pt_entry_wq_handler(struct work_struct *work)
{

	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);

	if (iris_mipi_info.iris_timing_flag
				|| (IRIS_MIPIRX_CMD == iris_mipi_info.mipi_mode.rx_mode)) {
		pt_enable[0] = 0x3 << 2;
		pt_enable[1] = 0x1;
	} else {
		pt_enable[0] = 0x0;
		pt_enable[1] = 0x1;
	}
	// pt_mode_enter
	mdss_dsi_cmds_tx(ctrl, pt_mode_enter,
			ARRAY_SIZE(pt_mode_enter), (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));
	// pt_data_path_config
	iris_pt_enter_cmds[21] = (iris_pt_enter_cmds[21] & 0xFC) | iris_dbc_mode;
	mdss_dsi_cmds_tx(ctrl, pt_data_path_config,
			ARRAY_SIZE(pt_data_path_config), (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));

	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------\n", __func__);
}

static void iris_memc_path_commands_update(void)
{
	*(u32 *)(iris_memc_enter_cmds + 20) = cpu_to_le32(g_mfd->iris_conf.frc_path);
	iris_memc_enter_cmds[21] = (iris_memc_enter_cmds[21] & 0xFC) | iris_dbc_mode;
}

static void iris_memc_prepare_handler(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct mdss_panel_info *pinfo = g_mfd->panel_info;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	BUG_ON(ctrl == NULL || pinfo == NULL);

	if (check_mode_status(ctrl, IRIS_MEMC_MODE) == 0) {
		iris_cfg->sf_notify_mode = MDP_IRIS_MODE_FRC_PREPARE_DONE;
		pw_iris2_status = MDP_IRIS_MODE_FRC_PREPARE_DONE;
	}
	else
		pr_err("%s MDP_IRIS_MODE_FRC_PREPARE failed\n", __func__);

	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------sf_notify_mode = %d\n", __func__, iris_cfg->sf_notify_mode);
}

static void iris_memc_cancel_handler(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	BUG_ON(ctrl == NULL || iris_cfg == NULL);

	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------\n", __func__);
}

static void iris_memc_entry_handler(struct work_struct *work)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct mdss_panel_info *pinfo = g_mfd->panel_info;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	pr_debug("%s ++++++\n", __func__);
	BUG_ON(ctrl == NULL || pinfo == NULL);
	iris_proc_frcc_setting(g_mfd);
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);
	mdss_dsi_cmds_tx(ctrl, memc_mode_enter,
			ARRAY_SIZE(memc_mode_enter), (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));
	atomic_dec(&iris_cfg->mode_switch_cnt);
	pr_debug("%s ------\n", __func__);
}

void iris_disable_pwil_capen(struct mdss_dsi_ctrl_pdata *ctrl)
{
	char pb_meta[] = {0x0c, 0x01};
	char pwil_capen[] = {
		PWIL_TAG('P', 'W', 'I', 'L'),
		PWIL_TAG('G', 'R', 'C', 'P'),
		PWIL_U32(0x00000003),	// valid word number
		0x00,			// burst mode
		0x00,			// reserved
		PWIL_U16(0x02),	// burst length
		PWIL_U32(IRIS_PWIL_ADDR + 0x0218),	//CAPEN
		PWIL_U32(0xc0000001)
	};
	struct dsi_cmd_desc disable_pwil_capen[] = {
		{ {DTYPE_GEN_WRITE2, 1, 0, 0, 1, sizeof(pb_meta)}, pb_meta},
		{ {DTYPE_GEN_LWRITE, 1, 0, 0, 0,  sizeof(pwil_capen)}, pwil_capen}
	};
	struct dsi_panel_cmds panel_cmds;

	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);
	panel_cmds.cmds = disable_pwil_capen;
	panel_cmds.cmd_cnt = ARRAY_SIZE(disable_pwil_capen);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);
	mdss_dsi_cmd_hs_mode(0, &ctrl->panel_data);
}

int iris_fw_download_cont_splash(struct mdss_panel_data *pdata, bool debug)
{
	struct mdss_dsi_ctrl_pdata *ctrl = container_of(pdata,
                                struct mdss_dsi_ctrl_pdata, panel_data);

	pr_debug("%s is called, pdata=%p\n", __func__, pdata);

	if (entry_mode == 2) {
		pr_info("%s: entry_mode = %d, return.\n", __func__, entry_mode);
		return 0;
	}


	if (debug) {
		mdss_dsi_clk_ctrl(ctrl, DSI_ALL_CLKS, 1);
		mdss_mdp_lock(g_mfd, 1);
	}
	//diable pwil_capen
	pr_debug("off video\n");
	iris_disable_pwil_capen(ctrl);
	msleep(20);

	//switch to MCU mode
	iris_mipi_mcu(ctrl);
	//download appcode
	iris_firmware_download(ctrl, IRIS_FIRMWARE_NAME);
	// set the flag to iris
	iris_panel_on_finish(ctrl);
	//switch to PWIL mode
	iris_mipi_pwil(ctrl);
	//wait for appcode remap done
	msleep(100);

	pr_debug("on video\n");
	if (debug) {
		mdss_mdp_lock(g_mfd, 0);
		mdss_dsi_clk_ctrl(ctrl, DSI_ALL_CLKS, 0);
	}
	return 0;
}

static void iris_meta_wq_handler(void)
{
	struct mdss_mdp_ctl *ctl = mfd_to_ctl(g_mfd);
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct mdss_panel_data *pdata = ctl->panel_data;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	int cmd;
	int cnt = 2;

	//if (ctl->power_state == MDSS_PANEL_POWER_OFF)
	//	return;
	if (pdata->panel_info.panel_power_state == MDSS_PANEL_POWER_OFF)
		return;
	pr_debug("%s ++++++\n", __func__);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_regs_meta_build();
	iris_regs_clear();
	mutex_unlock(&iris_cfg->cmd_mutex);
	// TODO: when okay use ioctl or other side band to enable new frame
	iris_meta_pkts[0].dchdr.last = debug_new_frame_enabled ? 0 : 1;
	iris_meta_pkts[1].dchdr.last = debug_new_frame_enabled ? 1 : 0;
	for (cmd = 0; cmd < cnt; cmd++) {
		pr_debug("dchdr: %02x %02x %02x %02x %02x %02x\n",
		iris_meta_pkts[cmd].dchdr.dtype,
		iris_meta_pkts[cmd].dchdr.last,
		iris_meta_pkts[cmd].dchdr.vc,
		iris_meta_pkts[cmd].dchdr.ack,
		iris_meta_pkts[cmd].dchdr.wait,
		iris_meta_pkts[cmd].dchdr.dlen);
		{
		int i;
		for (i = 0; i < iris_meta_pkts[cmd].dchdr.dlen; i += 8)
			pr_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
			iris_meta_pkts[cmd].payload[i],   iris_meta_pkts[cmd].payload[i+1],
			iris_meta_pkts[cmd].payload[i+2], iris_meta_pkts[cmd].payload[i+3],
			iris_meta_pkts[cmd].payload[i+4], iris_meta_pkts[cmd].payload[i+5],
			iris_meta_pkts[cmd].payload[i+6], iris_meta_pkts[cmd].payload[i+7]);
		}
	}
	mdss_dsi_cmd_hs_mode(1, &ctrl->panel_data);
	// TODO: assume 2 meta packet will both issued at same kickoff
	if (iris_meta_pkts[0].dchdr.dlen > META_HEADER)
		mdss_dsi_cmds_tx(ctrl, iris_meta_pkts, cnt, (CMD_REQ_DMA_TPG & CMD_REQ_COMMIT));
	memset(imeta, 0, sizeof(imeta));
	pr_debug("%s ------\n", __func__);
}

int iris_mode_switch(struct msm_fb_data_type *mfd)
{
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_panel_data *pdata;
	struct mdss_mdp_pipe *pipe;
	struct iris_config *iris_cfg = &mfd->iris_conf;
	int used_vp = 0;
	int used_gp = 0;
	int used_dp = 0;

	if (!g_dsi_ctrl) {
		pdata = mdp5_data->ctl->panel_data;
		g_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	if (iris_cfg->mode_changed == false)
		return -EFAULT;

	iris_cfg->mode_changed = false;

	if (mfd->index != 0)
		return -EFAULT;

	pr_debug("memc_enable = %d, cur_mode = %d, video_on = %d\n",
		iris_cfg->memc_enable, iris_cfg->current_mode, iris_cfg->video_on);

	list_for_each_entry(pipe, &mdp5_data->pipes_used, list) {
		if (pipe->type == MDSS_MDP_PIPE_TYPE_VIG)
			used_vp++;

		if (pipe->type == MDSS_MDP_PIPE_TYPE_RGB)
			used_gp++;

		if (pipe->type == MDSS_MDP_PIPE_TYPE_DMA)
			used_dp++;
	}

	pr_debug("memc_enable %d, doable = %d, cur_mode = %d\n",
			iris_cfg->memc_enable,
			is_memc_doable(used_vp, used_gp),
			iris_cfg->sf_notify_mode);
	pr_debug("vp = %d, gp = %d dp = %d\n", used_vp, used_gp, used_dp);

	if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_PREPARE
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->memc_prepare_work);
	} else if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->memc_work);
	} else if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_CANCEL
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->memc_cancel_work);
	} else if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_RFB
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		atomic_inc(&iris_cfg->mode_switch_cnt);
		schedule_work(&iris_cfg->pt_work);
	}

	return 0;
}

static void iris_check_rfb_ready(struct iris_config *iris_cfg){
	u32 val;
	if (iris_cfg->check_appcode_rfb_ready) {
		val = iris_pi_read(g_dsi_ctrl, IRIS_PROXY_ADDR + 0x08);
		if (val == IRIS_PT_MODE) {
			pr_info("appcode switch to RFB, rfb_delay: %d\n", rfb_delay);
			iris_cfg->check_appcode_rfb_ready = false;
		} else {
			pr_debug("appcode not switch to RFB\n");
		}
	}
	if (iris_cfg->check_pwil_rfb_ready) {
		val = iris_pi_read(g_dsi_ctrl, IRIS_PWIL_ADDR + 0x80);
		if ((val & 0x7e0) == 0x80) {
			pr_info("pwil switch to RFB, rfb_delay: %d\n", rfb_delay);
			iris_cfg->check_pwil_rfb_ready = false;
		} else {
			pr_debug("pwil not switch to RFB\n");
		}
	}
}

int iris_mode_switch_cmd(struct msm_fb_data_type *mfd)
{
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_panel_data *pdata;
	struct iris_config *iris_cfg = &mfd->iris_conf;
	struct dcs_cmd_req cmdreq;
	struct dsi_cmd_desc cmd_rfb;
	bool memc_enable;
	u32 val;
	if (!g_dsi_ctrl) {
		pdata = mdp5_data->ctl->panel_data;
		g_dsi_ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	if (iris_cfg->mode_changed == false)
		return -EFAULT;

	if (mfd->index != 0)
		return -EFAULT;

	if (!iris_cfg->ready) {
		iris_cfg->mode_changed = false;
		iris_cfg->sf_notify_mode = MDP_IRIS_MODE_RFB;
		pr_info("forbid mode switch, iris not ready!\n");
		return -EFAULT;
	}

	if (debug_usb_w_enabled) {
		memc_enable = iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_PREPARE ||
			iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC;
		if (memc_enable && !iris_cfg->memc_perf_hack && usb_w_enabled_by_configure) {
			pr_info("memc_perf_hack enable\n");
			iris2_io_bimc_enable(true);
			iris2_io_snoc_enable(true);
			iris_cfg->memc_perf_hack = true;
		}
		else if (!memc_enable && iris_cfg->memc_perf_hack) {
			pr_info("memc_perf_hack disable\n");
			iris2_io_bimc_enable(false);
			iris2_io_snoc_enable(false);
			iris_cfg->memc_perf_hack = false;
		}
	}

	cmdreq.cmds = &cmd_rfb;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_HS_MODE;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	cmdreq.cmds_cnt = 1;

	iris_cfg->mode_changed = false;
	if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_PREPARE
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		if (prep_delay == 0) {
			iris_check_rfb_ready(iris_cfg);
			if (iris_cfg->check_appcode_rfb_ready || iris_cfg->check_pwil_rfb_ready) {
				iris_cfg->mode_changed = true;
				return 0;
			}
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
			// drc LP MEMC update, don't take effect now
			if (g_mfd->iris_conf.drc_enable && (g_mfd->iris_conf.drc_size != guFrcLPTiming))
				iris_Drc_LPMemc_update(mfd);
#endif
			iris_pi_write(g_dsi_ctrl,IRIS_SYS_ADDR, 0x20000000);
			iris_memc_path_commands_update();
			// memc_data_path_config
			if (debug_mode_switch_enabled) {
				memcpy(&cmd_rfb, memc_data_path_config, sizeof(struct dsi_cmd_desc));
				mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			}
			prep_delay++;
			iris_cfg->mode_changed = true;
		} else if (prep_delay++ < 30) {
			if (debug_mode_switch_enabled)
				val = iris_pi_read(g_dsi_ctrl, IRIS_PROXY_ADDR + 0x08);
			else
				val = IRIS_MEMC_MODE;
			if (val != IRIS_MEMC_MODE) {
				pr_debug("iris: mode = %08x, cnt = %d\n", val, prep_delay);
				if (val == IRIS_PT_MODE) {
					pr_info("iris: still in RFB mode, retry mode switch\n");
					iris_cfg->sf_notify_mode = MDP_IRIS_MODE_FRC_PREPARE_RFB;
					prep_delay = 0;
					return 0;
				}
				iris_cfg->mode_changed = true;
			} else {
				prep_delay = 0;
				frc_delay = 0;
				frc_repeat_enter = false;
				iris_cfg->cap_enable = true;
				iris_cfg->cap_change = false;
				iris_cfg->sf_notify_mode = MDP_IRIS_MODE_FRC_PREPARE_DONE;
				pw_iris2_status = MDP_IRIS_MODE_FRC_PREPARE_DONE;
				pr_debug("iris: sf_notify_mode = %d\n", iris_cfg->sf_notify_mode);
			}
		} else {
			pr_info("iris: memc prep time out\n");
			iris_cfg->sf_notify_mode = MDP_IRIS_MODE_FRC_PREPARE_TIMEOUT;
			prep_delay = 0;
		}
	} else if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		pr_debug("sf_notify_mode: %d, frc_delay: %d\n", iris_cfg->sf_notify_mode, frc_delay);
		if (frc_delay == 0) {
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
			// drc enter
			if (g_mfd->iris_conf.drc_enable)
				iris_calc_drc_enter(mfd);
#endif
			iris_proc_frcc_setting(g_mfd);
			iris_MEMC_reg_write(g_mfd, demowinFI_setting_default);
			iris_mcuclk_divider_change(g_dsi_ctrl, 0);
			// memc_mode_enter
			if (debug_mode_switch_enabled) {
				memcpy(&cmd_rfb, memc_mode_enter, sizeof(struct dsi_cmd_desc));
				mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			}
			rfb_delay = 0;
			frc_delay = 1;
			iris_cfg->mode_changed = true;
		} else if (frc_delay == 1) {
			frc_delay++;
			iris_cfg->mode_changed = true;
		} else if (frc_delay == 2) {
			iris_set_scanline(iris_cfg);
			if (debug_clock_gate) {
				frc_delay++;
				iris_cfg->mode_changed = true;
			}
		}
		else if (debug_clock_gate && (frc_delay <= 8)) {
			val = iris_pi_read(g_dsi_ctrl, IRIS_PWIL_ADDR + 0x80); //check PWIL_STATUS
			if((val &0x7e0) == 0x100){
#ifndef ENABLE_SUPPORT_MEMC_WITH_GRID_LINE //should not gateoff the PSR CLK if grid line show is required.
				iris_reg_add(IRIS_SYS_ADDR, 0x20000400);
#endif
				pr_info("/////////sf_notify_mode: %d, frc_delay: %d\n", iris_cfg->sf_notify_mode, frc_delay); 
				iris_cfg->mode_changed = false;
			}
			else{
				frc_delay++;
				iris_cfg->mode_changed = true;
				if(frc_delay >8){
					pr_err("***********sf_notify_mode: %d, frc_delay: %d\n", iris_cfg->sf_notify_mode, frc_delay);
			}
			}
		}
	} else if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC_CANCEL
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		pr_debug("sf_notify_mode: %d\n", iris_cfg->sf_notify_mode);
		iris_pt_enter_cmds[21] = (iris_pt_enter_cmds[21] & 0xFC) | iris_dbc_mode;
		if (debug_mode_switch_enabled) {
			memcpy(&cmd_rfb, pt_data_path_config, sizeof(struct dsi_cmd_desc));
			mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
		}
		iris_mcuclk_divider_change(g_dsi_ctrl, 1);
#ifdef IRIS_CLOCK_GATING
		iris_pi_write(g_dsi_ctrl,IRIS_SYS_ADDR, 0x20000000);
#endif
		prep_delay = 0;
	} else if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_RFB
			&& (atomic_read(&g_mfd->iris_conf.mode_switch_cnt) == 0)) {
		pr_debug("sf_notify_mode: %d, rfb_delay: %d\n", iris_cfg->sf_notify_mode, rfb_delay);
		iris_cfg->iris_ratio_updated = false;
		iris_cfg->prev_dvts = 0;
		iris_cfg->repeat = IRIS_REPEAT_NO;
		prep_delay = 0;
		if (0 == rfb_delay) {
			// workaround, change prameters before FRC exit
			u32 DisplayVtotal = (iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp +
					iris_mipi_info.iris_out_timing.vres + iris_mipi_info.iris_out_timing.vsw);
			unsigned int val_frcc_cmd_th = iris_cfg->val_frcc_cmd_th;
			unsigned int val_frcc_reg8 = iris_cfg->val_frcc_reg8;
			unsigned int val_frcc_reg16 = iris_cfg->val_frcc_reg16;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
			// drc exit
			if (g_mfd->iris_conf.drc_enable)
				iris_calc_drc_exit(mfd);
#endif
			iris_pi_write(g_dsi_ctrl,IRIS_SYS_ADDR, 0x20000000);
			val_frcc_cmd_th &= 0x1fffffff;
			val_frcc_cmd_th |= 0x20000000;
			val_frcc_reg8 |= 0x3f00;
			val_frcc_reg16 &= 0xffff7fff;
			iris_fiSearchRangeTop();
			//iris_set_scanline(iris_cfg);
			mutex_lock(&iris_cfg->cmd_mutex);
			iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
			iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
			iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
			iris_reg_add(FRCC_REG_SHOW, 0x2);
			// set the ratio in/out as 1:1
			iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, ((1 << IRIS_PWIL_IN_FRAME_SHIFT) | (1 << 15)));
			iris_reg_add(IRIS_PWIL_ADDR + 0x0638, 1 << IRIS_PWIL_OUT_FRAME_SHIFT);
			iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
			iris_reg_add(IRIS_DTG_ADDR + 0x018, (DisplayVtotal - 12) | ((DisplayVtotal - 5) << 16)); //FI_PRELOAD_DLY | EVS_NEW_DLY
			iris_reg_add(IRIS_DTG_ADDR + 0x10000, 1);   //reg_update
			iris_reg_add(FI_RANGE_CTRL, m_fiVrangeTop | (11 << 9) | (511 << 18));
			iris_reg_add(FI_SHADOW_UPDATE, 1);
			mutex_unlock(&iris_cfg->cmd_mutex);
			rfb_delay = 1;
			iris_cfg->mode_changed = true;
			//return 0;
		} else if (rfb_delay == 1) {
			rfb_delay++;
			iris_set_scanline(iris_cfg);
			iris_cfg->mode_changed = true;
		} else if (rfb_delay == 2) {
			val = iris_pi_read(g_dsi_ctrl, UNIT_CONTRL_ADDR + 0x08);
			val |= 0x800; //enable bit 11
			iris_pi_write(g_dsi_ctrl, UNIT_CONTRL_ADDR + 0x08, val);
			if (iris_mipi_info.iris_timing_flag
					|| (IRIS_MIPIRX_CMD == iris_mipi_info.mipi_mode.rx_mode)) {
				pt_enable[0] = 0x3 << 2;
				pt_enable[1] = 0x1;
			} else {
				pt_enable[0] = 0x0;
				pt_enable[1] = 0x1;
			}
			// pt_mode_enter
			memcpy(&cmd_rfb, pt_mode_enter, sizeof(struct dsi_cmd_desc));
			mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			// pt_data_path_config
			iris_pt_enter_cmds[21] = (iris_pt_enter_cmds[21] & 0xFC) | iris_dbc_mode;
			if (debug_mode_switch_enabled) {
				memcpy(&cmd_rfb, pt_data_path_config, sizeof(struct dsi_cmd_desc));
				mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			}
			if (debug_new_repeat == 0) {
				unsigned int reg_cap = 0xc0000003;
				mutex_lock(&iris_cfg->cmd_mutex);
				iris_reg_add(IRIS_PWIL_ADDR + 0x0218, reg_cap);
				mutex_unlock(&iris_cfg->cmd_mutex);
			}
			iris_mcuclk_divider_change(g_dsi_ctrl, 1);
			if (debug_clock_gate) {
				rfb_delay++;
				iris_cfg->mode_changed = true;
			}
			iris_cfg->check_appcode_rfb_ready = true;
			iris_cfg->check_pwil_rfb_ready = true;
		}
		else if (debug_clock_gate && (rfb_delay <= 8)) {
			rfb_delay++;
			iris_cfg->mode_changed = true;
		}
		else if (debug_clock_gate && (rfb_delay <= 15)) {
			iris_check_rfb_ready(iris_cfg);

			if (!iris_cfg->check_appcode_rfb_ready && !iris_cfg->check_pwil_rfb_ready) {
				iris_reg_add(IRIS_SYS_ADDR, 0x20003000);
				iris_cfg->mode_changed = false;
				pr_info("RFB switch finished.\n");
			} else {
				rfb_delay++;
				iris_cfg->mode_changed = true;
				if(rfb_delay >15) {
					pr_info("RFB switch not finished, appcode ready: %d, pwil ready: %d\n",
						!iris_cfg->check_appcode_rfb_ready, !iris_cfg->check_pwil_rfb_ready);
				}
			}
		}
	}

	return 0;
}

static int iris_reg_cnt;
struct iris_reg_t {
	u32 addr;
	u32 val;
};
#define IRIS_REGS 40
static struct iris_reg_t iris_regs[IRIS_REGS];

static void iris_regs_clear(void)
{
	iris_reg_cnt = 0;
	//memset(iris_regs, 0, sizeof(iris_regs));
}

static void iris_reg_add(u32 addr, u32 val)
{
	if (iris_reg_cnt >= IRIS_REGS)
		return;
	pr_debug("regs[%i:%08x] = %08x\n", iris_reg_cnt, addr, val);
	iris_regs[iris_reg_cnt].addr = addr;
	iris_regs[iris_reg_cnt].val = val;
	iris_reg_cnt++;
}

static int  iris_regs_meta_build(void)
{
	int i;
	int size;

	pr_debug("reg_cnt: %02x", iris_reg_cnt);
	memcpy(imeta, imeta_header, META_HEADER);
	// pair
	for (i = 0; i < iris_reg_cnt; i++) {
		*(u32 *)(imeta + META_HEADER + i*8) = cpu_to_le32(iris_regs[i].addr);
		*(u32 *)(imeta + META_HEADER + i*8 + 4) = cpu_to_le32(iris_regs[i].val);
		/*
		imeta[META_HEADER + i*8    ] = iris_regs[i].addr         & 0xff;
		imeta[META_HEADER + i*8 + 1] = (iris_regs[i].addr >>  8) & 0xff;
		imeta[META_HEADER + i*8 + 2] = (iris_regs[i].addr >> 16) & 0xff;
		imeta[META_HEADER + i*8 + 3] = (iris_regs[i].addr >> 24) & 0xff;

		imeta[META_HEADER + i*8 + 4] = iris_regs[i].addr         & 0xff;
		imeta[META_HEADER + i*8 + 5] = (iris_regs[i].addr >>  8) & 0xff;
		imeta[META_HEADER + i*8 + 6] = (iris_regs[i].addr >> 16) & 0xff;
		imeta[META_HEADER + i*8 + 7] = (iris_regs[i].addr >> 24) & 0xff;
		*/
	}
	// size update
	size = iris_reg_cnt * 2;
	*(u32 *)(imeta + 8) = cpu_to_le32(size + 1);
	*(u16 *)(imeta + 14) = cpu_to_le16(size);
	iris_meta_pkts[0].dchdr.dlen = META_HEADER + iris_reg_cnt * 8;
	return iris_reg_cnt;
}

void iris_copy_meta(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	// copy meta
	mutex_lock(&iris_cfg->meta_mutex);
	if (iris_cfg->meta_set.op) {
		memcpy((void *)&iris_cfg->meta, (void *)&iris_cfg->meta_set, sizeof(struct iris_meta));
		memset((void *)&iris_cfg->meta_set, 0, sizeof(struct iris_meta));
		pr_debug("iris_copy_meta\n");
	}
	mutex_unlock(&iris_cfg->meta_mutex);
}

/*
 * There are 3 commands in iris_meta_pkts
 * first is meta, and second is for capture enable/disable. Third is FBO.
 * When capture changed:
 * If meta is null(iris_reg_cnt), send the capture command directly.
 * If meta is not null, copy capture command to iris_meta_pkts[1], and send together.
 *
 */

void iris_send_meta_cmd(struct mdss_mdp_ctl *ctl)
{
	struct mdss_dsi_ctrl_pdata *ctrl = g_dsi_ctrl;
	struct dcs_cmd_req cmdreq;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	int cmd;

	BUG_ON(ctrl == NULL);

	if(entry_mode==2){
		return;
	}

	if (!iris_cfg->ready) {
		pr_info("%s ++++++, iris not ready!\n", __func__);
		return;
	}

	if (!debug_send_meta_enabled)
		return;

	if ((atomic_read(&g_mfd->iris_conf.mode_switch_cnt)))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	if (iris_cfg->cap_change) {
#ifdef ENABLE_SUPPORT_MEMC_WITH_GRID_LINE
		static char osd_enable[2] = {0x04, 0x08};
		static struct dsi_cmd_desc osd_mode_enter[] = {
			{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(osd_enable) }, osd_enable},};
#else
		static char cursor_enable[2] = {0x04, 0x10};
		static struct dsi_cmd_desc cursor_mode_enter[] = {
			{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(cursor_enable) }, cursor_enable},};
#endif
		if (iris_cfg->cap_enable)
			cmdreq.cmds = memc_mode_enter;
		else
#ifdef ENABLE_SUPPORT_MEMC_WITH_GRID_LINE
			cmdreq.cmds = osd_mode_enter; //post the discarded frame into osd channel. cursor channel would be used for grid line show.
#else
			cmdreq.cmds = cursor_mode_enter;
#endif

		if(iris_reg_cnt == 0) {
			mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);
			iris_cfg->cap_change = false;
		} else
			memcpy(&iris_meta_pkts[1], cmdreq.cmds, sizeof(struct dsi_cmd_desc));

		pr_debug("cap_change: %d\n", iris_cfg->cap_enable);
	}

	if (iris_reg_cnt !=0)
		pr_debug("%s ++++++, iris_reg_cnt: %d\n", __func__, iris_reg_cnt);

	mutex_lock(&g_mfd->iris_conf.cmd_mutex);
	if (!iris_regs_meta_build()) {
		mutex_unlock(&g_mfd->iris_conf.cmd_mutex);
		return;
	}

	iris_regs_clear();
	mutex_unlock(&g_mfd->iris_conf.cmd_mutex);
	cmdreq.cmds = iris_meta_pkts;

	if (iris_cfg->cap_change == false)
		iris_meta_pkts[0].dchdr.last = 1;
	else {
		iris_meta_pkts[1].dchdr.last = 1;
		iris_meta_pkts[0].dchdr.last = 0;
		cmdreq.cmds_cnt = 2;
	}

	iris_cfg->cap_change = false;

	for (cmd = 0; cmd < cmdreq.cmds_cnt; cmd++) {
		pr_debug("dchdr: %02x %02x %02x %02x %02x %02x\n",
			iris_meta_pkts[cmd].dchdr.dtype,
			iris_meta_pkts[cmd].dchdr.last,
			iris_meta_pkts[cmd].dchdr.vc,
			iris_meta_pkts[cmd].dchdr.ack,
			iris_meta_pkts[cmd].dchdr.wait,
			iris_meta_pkts[cmd].dchdr.dlen);
		{
		int i;
		for (i = 0; i < iris_meta_pkts[cmd].dchdr.dlen; i += 8)
			pr_debug("%02x %02x %02x %02x %02x %02x %02x %02x\n",
				iris_meta_pkts[cmd].payload[i],   iris_meta_pkts[cmd].payload[i+1],
				iris_meta_pkts[cmd].payload[i+2], iris_meta_pkts[cmd].payload[i+3],
				iris_meta_pkts[cmd].payload[i+4], iris_meta_pkts[cmd].payload[i+5],
				iris_meta_pkts[cmd].payload[i+6], iris_meta_pkts[cmd].payload[i+7]);
		}
	}

	mdss_dsi_cmdlist_put(g_dsi_ctrl, &cmdreq);

	//memset(imeta, 0, sizeof(imeta));
}

void iris_send_meta_video(struct mdss_mdp_ctl *ctl)
{
	struct msm_fb_data_type *mfd = ctl->mfd;
	struct iris_config *iris_cfg = &mfd->iris_conf;

	BUG_ON(iris_cfg == NULL);

	if (!debug_send_meta_enabled)
		return;

	if ((atomic_read(&g_mfd->iris_conf.mode_switch_cnt)))
		return;

	//schedule_work(&iris_cfg->meta_work);
	iris_mgmt.iris_handler = iris_meta_wq_handler;
	queue_work(iris_mgmt.iris_wq, &iris_mgmt.iris_worker);
}


// shall be called before params_changed clear to 0
static int iris_proc_repeat(struct iris_config *iris_cfg)
{
	u8 prev_repeat;
	int ret;

	prev_repeat = iris_cfg->repeat;

	if (debug_repeat_enabled > 3)
		iris_cfg->repeat = debug_repeat_enabled - 3;
	else
		iris_cfg->repeat = (iris_cfg->meta.op & MDP_IRIS_OP_RPT) ? iris_cfg->meta.repeat : iris_cfg->repeat;

	pr_debug("repeat = %d\n", iris_cfg->repeat);

	ret = ((iris_cfg->repeat != prev_repeat) || (iris_cfg->repeat == IRIS_REPEAT_FORCE));
	return ret;
}

static int iris_set_ratio(struct iris_config *iris_cfg)
{
	unsigned int reg_in, reg_out, reg_scale;
	bool cap_enable;

	reg_in = iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT | (1 << 15);
	reg_out = iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT;
	reg_scale = 4096/iris_cfg->scale << 24 | 64 << 16 | iris_cfg->scale << 8 | iris_cfg->scale;
	/* duplicated video frame */
	cap_enable = iris_cfg->repeat != IRIS_REPEAT_CAPDIS;
	/* set ratio after mode switch to FRC */
	if (!debug_ratio_enabled || iris_cfg->sf_notify_mode != MDP_IRIS_MODE_FRC)
		return true;

	pr_debug("vts %u pvts %u cap_enable %d\n", iris_cfg->meta.video_ts, iris_cfg->prev_vts, cap_enable);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(IRIS_PWIL_ADDR + 0x12FC, reg_in);
	iris_reg_add(IRIS_PWIL_ADDR + 0x0638, reg_out);
	iris_reg_add(IRIS_PWIL_ADDR + 0x10000, (1 << 8) | (1 << 6));
	iris_reg_add(IRIS_MVC_ADDR + 0x1D0, reg_scale);
	iris_reg_add(IRIS_MVC_ADDR + 0x1FF00, 1);
	mutex_unlock(&iris_cfg->cmd_mutex);
	return cap_enable;
}

bool iris_frc_repeat(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	int ret_r, ret_p;
	int cap_enable = true;
	bool ret;

	if (iris_cfg->sf_notify_mode != MDP_IRIS_MODE_FRC)
		return cap_enable;

	ret_r = iris_proc_ratio(iris_cfg);
	ret_p = iris_proc_repeat(iris_cfg);
	if (ret_p)
		cap_enable = iris_set_repeat(iris_cfg);
	else if (ret_r)
		cap_enable = iris_set_ratio(iris_cfg);
	else {
		cap_enable = iris_cfg->cap_enable;
		pr_debug("keep the last value: %d!\n", cap_enable);
	}

	if (iris_cfg->sf_notify_mode == MDP_IRIS_MODE_FRC) {
		if (cap_enable != iris_cfg->cap_enable) {
			pr_debug("capture-change: %d!\n", cap_enable);
			if (debug_new_repeat == 1)
				iris_cfg->cap_change = true;
			else if (debug_new_repeat == 0) {
				unsigned int reg_cap;

				if (cap_enable)
					reg_cap = 0xc0000003;
				else
					reg_cap = 0xc0000001;
				mutex_lock(&iris_cfg->cmd_mutex);
				iris_reg_add(IRIS_PWIL_ADDR + 0x0218, reg_cap);
				mutex_unlock(&iris_cfg->cmd_mutex);
			}
			iris_cfg->cap_enable = cap_enable;
		}
	}

	ret = ((debug_new_repeat == 2) ? cap_enable : true);
	return ret;
}

int iris_proc_frcc_setting(struct msm_fb_data_type *mfd)
{
	struct iris_config *iris_cfg = &mfd->iris_conf;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_panel_data *pdata = mdp5_data->ctl->panel_data;

	//default val of reference register which need host to set.
	u32 val_frcc_reg5 = 0x3c010000;
	u32 val_frcc_reg8 = 0x10000000;
	u32 val_frcc_reg16 = 0x413120c8;
	u32 val_frcc_reg17 = 0x8000;
	u32 val_frcc_reg18 = 0;
	u32 val_frcc_cmd_th = 0x8000;

	//formula variable
	u32 ThreeCoreEn, VD_CAP_DLY1_EN;
	u32 MaxFIFOFI, KeepTH, CarryTH, RepeatP1_TH;
	u32 RepeatCF_TH, TS_FRC_EN, INPUT_RECORD_THR, MERAREC_THR_VALID;
	u32 MetaGen_TH1, MetaGen_TH2, MetaRec_TH1, MetaRec_TH2;

	//timing and feature variable
	u32 te_fps, display_vsync, Input_Vres, Scaler_EN = false, Capture_EN, Input_Vtotal;
	u32 DisplayVtotal, HsyncFreqIn, HsyncFreqOut, InVactive, StartLine, Vsize;
	int inputwidth = (iris_mipi_info.mipi_mode.rx_ch  ?  iris_mipi_info.iris_in_timing.hres * 2 : iris_mipi_info.iris_in_timing.hres);
	u32 Infps = iris_cfg->input_frame_rate;
	int adjustmemclevel = 3;
	int hlmd_func_enable = 0;

	//init variable
	te_fps = mdss_panel_get_framerate(&pdata->panel_info);
	display_vsync = 60;//iris to panel, TODO, or 120
	Input_Vres = pdata->panel_info.yres;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	Capture_EN = iris_cfg->nrv_enable | iris_cfg->drc_enable;
#else
	Capture_EN = iris_cfg->nrv_enable;
#endif
	Input_Vtotal = mdss_panel_get_vtotal(&pdata->panel_info);
	if (iris_LPMeMcTiming[0] != inputwidth)
		Scaler_EN = true;
	else
		Scaler_EN = false;
	DisplayVtotal = (iris_mipi_info.iris_out_timing.vbp + iris_mipi_info.iris_out_timing.vfp +
		iris_mipi_info.iris_out_timing.vres + iris_mipi_info.iris_out_timing.vsw);
	HsyncFreqIn = te_fps * Input_Vtotal;
	HsyncFreqOut = display_vsync * DisplayVtotal;
	InVactive = iris_cfg->meta.nrv.captureBottom - iris_cfg->meta.nrv.captureTop;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	if (iris_cfg->drc_enable)
		InVactive = (iris_cfg->drc_size >> 16);
#endif
	if (Capture_EN)
		StartLine = Input_Vres - InVactive;
	else if (Scaler_EN)
		StartLine = 5;
	else
		StartLine = 0;
	if (Capture_EN)
		Vsize = InVactive;
	else
		Vsize = DisplayVtotal;

	pr_debug("%s: get timing info, infps=%d, displayVtotal = %d, InVactive = %d, StartLine = %d, Vsize = %d\n",
		__func__, Infps, DisplayVtotal, InVactive, StartLine, Vsize);
	pr_debug("TE_fps = %d, display_vsync = %d, inputVres = %d, Scaler_EN = %d, capture_en = %d, InputVtotal = %d\n",
		te_fps, display_vsync, Input_Vres, Scaler_EN, Capture_EN, Input_Vtotal);

	if (mfd->panel_info->type == MIPI_VIDEO_PANEL) {
		//video mode
		ThreeCoreEn = 1; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		goto VAL_CALC;
	}

	if (iris_cfg->fbo_enable) {
		//TODO mbo mode
		ThreeCoreEn = 1; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		goto VAL_CALC;
	}

	//check input is variable frame rate or not.
	switch (iris_cfg->input_vfr) {
	case 15:// 15 fps from 24/25 fps.
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 5; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		if (debug_hlmd_enabled) {
			hlmd_func_enable = 1;
			RepeatP1_TH = 1;
			CarryTH = 1;
		} else {
			hlmd_func_enable = 0;
		}
		goto VAL_CALC;
	case 50:// vfr from 50 drop
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		goto VAL_CALC;
	case 60:// vfr from 60 drop
		ThreeCoreEn = 1; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 4; KeepTH = 61; CarryTH = 1;
		RepeatP1_TH = 1; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 0;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		goto VAL_CALC;
	case 0:// vfr is invalid, frame rate is constant
	default :
		break;
	}

	switch (Infps) {
	case 24://24fps
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 3; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		break;
	case 30://30fps
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 2; KeepTH = 60; CarryTH = 5;
		RepeatP1_TH = 5; RepeatCF_TH = 60; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		break;
	case 25://25fps
		ThreeCoreEn = 1; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 3; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 0; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 3 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		break;
	case 15://15fps
		if (debug_hlmd_enabled) {
			hlmd_func_enable = 1;
			RepeatP1_TH = 1;
			CarryTH = 1;
		} else {
			hlmd_func_enable = 0;
			RepeatP1_TH = 2;
			CarryTH = 2;
		}
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 5; KeepTH = 61;
		RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
	case 12://12fps
		ThreeCoreEn = 0; VD_CAP_DLY1_EN = 0; MaxFIFOFI = 5; KeepTH = 61; CarryTH = 2;
		RepeatP1_TH = 2; RepeatCF_TH = 61; TS_FRC_EN = 1; MERAREC_THR_VALID = 1;
		MetaGen_TH1 = (Vsize / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaGen_TH2 = (Vsize * 6 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH1 = (Vsize * 5 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		MetaRec_TH2 = (Vsize * 7 / 8 + StartLine) * HsyncFreqOut / HsyncFreqIn;
		INPUT_RECORD_THR = (Vsize  / 2 + StartLine) * HsyncFreqOut / HsyncFreqIn - 10;
		break;
	default:
		pr_err("%s, using default frcc parameters\n", __func__);
		goto SET_REG;
	}

VAL_CALC:
	if (pq_setting_current.memcLevel == 3)
		adjustmemclevel = 3;
	else if (pq_setting_current.memcLevel == 2)
		adjustmemclevel = 3;
	else if (pq_setting_current.memcLevel == 1)
		adjustmemclevel = 2;
	else if (pq_setting_current.memcLevel == 0)
		adjustmemclevel = 0;

	//val_frcc_reg5 = val_frcc_reg5 + ((pq_setting_current.memcLevel & 0x3) << 17) + (KeepTH * 2 << 7) + CarryTH;
	val_frcc_reg5 = val_frcc_reg5 + ((adjustmemclevel & 0x3) << 17) + (KeepTH * 2 << 7) + CarryTH;
	val_frcc_reg8 = val_frcc_reg8 + (RepeatP1_TH * 2 << 7) + RepeatCF_TH * 2;
	val_frcc_reg16 = val_frcc_reg16 + (TS_FRC_EN * 2 << 30) + (ThreeCoreEn*2 << 14) + VD_CAP_DLY1_EN;
	val_frcc_reg17 = val_frcc_reg17 + (DisplayVtotal * 2 << 15) + INPUT_RECORD_THR;
	val_frcc_reg18 = val_frcc_reg18 + (MERAREC_THR_VALID * 2 << 30) + (MetaRec_TH2 * 2 << 15) + MetaRec_TH1;
	val_frcc_cmd_th = val_frcc_cmd_th + (MaxFIFOFI * 2 << 28) + (MetaGen_TH2 * 2 << 15) + MetaGen_TH1;

SET_REG:
	pr_debug("%s: reg5=%x, reg8=%x, reg16=%x, reg17=%x, reg18=%x, cmd_th=%x\n", __func__,
		val_frcc_reg5, val_frcc_reg8, val_frcc_reg16, val_frcc_reg17, val_frcc_reg18, val_frcc_cmd_th);
	mutex_lock(&iris_cfg->cmd_mutex);
	iris_reg_add(FRCC_CTRL_REG5_ADDR, val_frcc_reg5);
	iris_reg_add(FRCC_CTRL_REG8_ADDR, val_frcc_reg8);
	iris_reg_add(FRCC_CTRL_REG16_ADDR, val_frcc_reg16);
	iris_reg_add(FRCC_CTRL_REG17_ADDR, val_frcc_reg17);
	iris_reg_add(FRCC_CTRL_REG18_ADDR, val_frcc_reg18);
	iris_reg_add(FRCC_CMD_MOD_TH, val_frcc_cmd_th);
	iris_reg_add(IRIS_MVC_ADDR + 0x1ffe8, 0x00000000);
	if (debug_hlmd_enabled) {
		if (hlmd_func_enable)
			iris_reg_add(IRIS_MVC_ADDR + 0x1ffe8, 0x00200000);
	}
	mutex_unlock(&iris_cfg->cmd_mutex);
	iris_cfg->val_frcc_cmd_th = val_frcc_cmd_th;
	iris_cfg->val_frcc_reg8 = val_frcc_reg8;
	iris_cfg->val_frcc_reg16 = val_frcc_reg16;
	iris_cfg->val_frcc_reg17 = val_frcc_reg17;
	return 0;
}

#ifdef ENABLE_TCON_CABC
static u8 mipitx_write_brightness[80] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x11),
	0x00,
	0x00,
	PWIL_U16(0x10),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c014),
	PWIL_U32(0x00ff0000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c010),
	PWIL_U32(0x00008001),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c018),
	PWIL_U32(0x00000010),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c020),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c014),
	PWIL_U32(0x00510000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c010),
	PWIL_U32(0x00008001),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c018),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c020),
	PWIL_U32(0x00000001),
};

static struct dsi_cmd_desc mipitx_write_cmds[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(mipitx_write_brightness) }, mipitx_write_brightness},
};

void iris_tcon_cabc_configure(struct mdss_dsi_ctrl_pdata *ctrl, u8 brightness)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	struct dcs_cmd_req cmdreq;

	if (!iris_cfg->ready)
		return;

	mipitx_write_brightness[68] = brightness & 0xff;

	printk("brightness is set as %d \n",brightness);

	cmdreq.cmds = mipitx_write_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static u8 mipitx_write_tcon_onoff[80] = {
	PWIL_TAG('P', 'W', 'I', 'L'),
	PWIL_TAG('G', 'R', 'C', 'P'),
	PWIL_U32(0x11),
	0x00,
	0x00,
	PWIL_U16(0x10),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c014),
	PWIL_U32(0x00ff0000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c010),
	PWIL_U32(0x00008001),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c018),
	PWIL_U32(0x00000010),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c020),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c014),
	PWIL_U32(0x00550000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c010),
	PWIL_U32(0x00008001),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c018),
	PWIL_U32(0x00000000),
	PWIL_U32(IRIS_MIPI_TX_ADDR+0x1c020),
	PWIL_U32(0x00000001),
};

void iris_tcon_cabc_on_off(u8 addr, u8 value)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	struct dcs_cmd_req cmdreq;

	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(g_mfd);
	struct mdss_dsi_ctrl_pdata *ctrl;

	struct dsi_cmd_desc mipitx_write_cmds[] = {
		{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
			sizeof(mipitx_write_tcon_onoff) }, mipitx_write_tcon_onoff},
	};

	if (!iris_cfg->ready)
		return;
	ctrl = container_of(mdp5_data->ctl->panel_data, struct mdss_dsi_ctrl_pdata, panel_data);

	mipitx_write_tcon_onoff[54] = addr & 0xff;
	mipitx_write_tcon_onoff[68] = value & 0xff;

	printk("iris_tcon_cabc_on_off add=0x%x value=0x%x \n", (unsigned int)addr, (unsigned int)value);

	cmdreq.cmds = mipitx_write_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_HS_MODE | CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);

}
EXPORT_SYMBOL(iris_tcon_cabc_on_off);
#endif


static ssize_t iris_dbg_fw_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	ssize_t rc;
	struct mdss_overlay_private *mdp5_data;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl;

	mdp5_data = mfd_to_mdp5_data(mfd);
	pdata = mdp5_data->ctl->panel_data;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata, panel_data);

	rc = iris_fw_download_cont_splash(pdata, 1);
	if (rc)
		return rc;

	return count;
}

static const struct file_operations iris_dbg_fw_fops = {
	.open = simple_open,
	.write = iris_dbg_fw_write,
};

static ssize_t iris_dbg_fbo_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	switch (val) {
	case 0:
		pr_info("%s:%d native frame rate video disable\n", __func__, __LINE__);
		iris_fbo_enable(mfd, 0);
		break;
	case 1:
		pr_info("%s:%d native frame rate video enable\n", __func__, __LINE__);
		iris_fbo_enable(mfd, 1);
		break;
	default:
		pr_err("%s:%d invalid input\n", __func__, __LINE__);
		break;
	}

	return count;
}

static const struct file_operations iris_dbg_fbo_fops = {
	.open = simple_open,
	.write = iris_dbg_fbo_write,
};

static ssize_t iris_dbg_sbs_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	pr_info("%s:%d sbs_enable %li\n", __func__, __LINE__, val);
	switch (val) {
	case 0:
		iris_sbs_enable(mfd, 0);
		break;
	case 1:
		iris_sbs_enable(mfd, 1);
		break;
	default:
		pr_err("%s:%d invalid input\n", __func__, __LINE__);
		break;
	}

	return count;
}

static const struct file_operations iris_dbg_sbs_fops = {
	.open = simple_open,
	.write = iris_dbg_sbs_write,
};


static bool debug_vsync_enabled;
static void debug_vsync_handler(struct mdss_mdp_ctl *ctl, ktime_t vtime)
{
	// NOP
}

static struct mdss_mdp_vsync_handler iris_debug_vsync_handler = {
	.vsync_handler = debug_vsync_handler,
};

static ssize_t iris_dbg_vsync_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	struct mdss_overlay_private *mdp5_data = mfd_to_mdp5_data(mfd);
	struct mdss_mdp_ctl *ctl = mdp5_data->ctl;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	pr_info("%s:%d vsync_enable %li\n", __func__, __LINE__, val);
	if (val && !debug_vsync_enabled) {
		ctl->ops.add_vsync_handler(ctl, &iris_debug_vsync_handler);
		debug_vsync_enabled = true;
	} else if (!val && debug_vsync_enabled) {
		ctl->ops.remove_vsync_handler(ctl, &iris_debug_vsync_handler);
		debug_vsync_enabled = false;
	}
	return count;
}

static const struct file_operations iris_dbg_vsync_fops = {
	.open = simple_open,
	.write = iris_dbg_vsync_write,
};

static ssize_t iris_dbg_meta_enable_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct msm_fb_data_type *mfd = g_mfd;
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;
	uint32_t r;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	pr_info("%s:%d meta_enabled %u in/out %u/%u\n", __func__, __LINE__, (u32)val, iris_cfg->input_frame_rate, iris_cfg->output_frame_rate);
	debug_send_meta_enabled = val;

	r = gcd(mfd->iris_conf.input_frame_rate, mfd->iris_conf.output_frame_rate);
	mfd->iris_conf.in_ratio = mfd->iris_conf.input_frame_rate / r;
	mfd->iris_conf.out_ratio = mfd->iris_conf.output_frame_rate / r;

	iris_register_write(mfd, IRIS_PWIL_ADDR + 0x0638,
		(iris_cfg->out_ratio << IRIS_PWIL_OUT_FRAME_SHIFT));
	iris_register_write(mfd, IRIS_PWIL_ADDR + 0x12FC,
		(iris_cfg->in_ratio << IRIS_PWIL_IN_FRAME_SHIFT));
	return count;
}

static ssize_t iris_dbg_meta_enable_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_send_meta_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_meta_fops = {
	.open = simple_open,
	.write = iris_dbg_meta_enable_write,
	.read = iris_dbg_meta_enable_read,
};

static ssize_t iris_dbg_ratio_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_ratio_enabled = val;

	pr_info("ratio_enabled %u in/out %u/%u ratio %u/%u\n", (u32)val, iris_cfg->input_frame_rate, iris_cfg->output_frame_rate,
		iris_cfg->in_ratio, iris_cfg->out_ratio);

	return count;
}

static ssize_t iris_dbg_ratio_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "ratio_enabled %u in/out %u/%u ratio %u/%u\n", debug_ratio_enabled,
			iris_cfg->input_frame_rate, iris_cfg->output_frame_rate, iris_cfg->in_ratio, iris_cfg->out_ratio);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_ratio_fops = {
	.open = simple_open,
	.write = iris_dbg_ratio_write,
	.read = iris_dbg_ratio_read,
};

static ssize_t iris_dbg_mode_switch_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_mode_switch_enabled = val;

	pr_debug("debug_mode_switch_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_mode_switch_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "debug_mode_switch_enabled %u\n", debug_mode_switch_enabled);
	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_mode_switch_fops = {
	.open = simple_open,
	.write = iris_dbg_mode_switch_write,
	.read = iris_dbg_mode_switch_read,
};

static ssize_t iris_dbg_repeat_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_repeat_enabled = val;

	pr_info("repeat_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_repeat_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_repeat_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_repeat_fops = {
	.open = simple_open,
	.write = iris_dbg_repeat_write,
	.read = iris_dbg_repeat_read,
};

static ssize_t iris_dbg_te_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_te_enabled = val;

	pr_info("te_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_te_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_te_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_te_fops = {
	.open = simple_open,
	.write = iris_dbg_te_write,
	.read = iris_dbg_te_read,
};

static ssize_t iris_dbg_dtg_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_dtg_enabled = val;

	pr_info("dtg_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_dtg_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_dtg_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_dtg_fops = {
	.open = simple_open,
	.write = iris_dbg_dtg_write,
	.read = iris_dbg_dtg_read,
};

static ssize_t iris_dbg_new_repeat_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_new_repeat = val;

	pr_info("debug_new_repeat %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_new_repeat_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_new_repeat);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_new_repeat_fops = {
	.open = simple_open,
	.write = iris_dbg_new_repeat_write,
	.read = iris_dbg_new_repeat_read,
};


static ssize_t iris_dbg_new_frame_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_new_frame_enabled = val;

	pr_info("repeat_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_new_frame_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_new_frame_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_new_frame_fops = {
	.open = simple_open,
	.write = iris_dbg_new_frame_write,
	.read = iris_dbg_new_frame_read,
};


static ssize_t iris_dbg_frc_path_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;

	g_mfd->iris_conf.frc_path = val;

	return count;
}

static ssize_t iris_dbg_frc_path_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%x\n", g_mfd->iris_conf.frc_path);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_frc_path_fops = {
	.open = simple_open,
	.write = iris_dbg_frc_path_write,
	.read = iris_dbg_frc_path_read,
};

static ssize_t iris_dbg_input_vfr_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	iris_cfg->input_vfr = val;

	pr_info("input_vfr = %d\n", iris_cfg->input_vfr);

	return count;
}

static ssize_t iris_dbg_input_vfr_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];
	struct iris_config *iris_cfg = &g_mfd->iris_conf;

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "input_vfr = %d\n", iris_cfg->input_vfr);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_input_vfr_fops = {
	.open = simple_open,
	.write = iris_dbg_input_vfr_write,
	.read = iris_dbg_input_vfr_read,
};

static ssize_t iris_dbg_hlmd_write(struct file *file, const char __user *buff,
	size_t count, loff_t *ppos)
{
	//struct iris_config *iris_cfg = &g_mfd->iris_conf;
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_hlmd_enabled = val;

	pr_info("hlmd_enabled %u\n", (u32)val);

	return count;
}

static ssize_t iris_dbg_hlmd_read(struct file *file, char __user *buff,
	size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];
	//struct iris_config *iris_cfg = &g_mfd->iris_conf;
	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_hlmd_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
}

static const struct file_operations iris_dbg_hlmd_fops = {
	.open = simple_open,
	.write = iris_dbg_hlmd_write,
	.read = iris_dbg_hlmd_read,
};

static ssize_t iris_dbg_usb_w_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_usb_w_enabled = val;

	pr_info("usb_w_enabled %u\n", (u32)val);

	return count;
};

static ssize_t iris_dbg_usb_w_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_usb_w_enabled);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
};

static const struct file_operations iris_dbg_usb_w_fops = {
	.open = simple_open,
	.write = iris_dbg_usb_w_write,
	.read = iris_dbg_usb_w_read,
};

static ssize_t iris_dbg_clock_gate_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	unsigned long val;

	if (kstrtoul_from_user(buff, count, 0, &val))
		return -EFAULT;
	debug_clock_gate = val;

	pr_info("debug_clock_gate %u\n", (u32)val);

	return count;
};

static ssize_t iris_dbg_clock_gate_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	int len, tot = 0;
	char bp[512];

	if (*ppos)
		return 0;

	len = sizeof(bp);
	tot = scnprintf(bp, len, "%u\n", debug_clock_gate);

	if (copy_to_user(buff, bp, tot))
		return -EFAULT;

	*ppos += tot;

	return tot;
};

static const struct file_operations iris_dbg_clock_gate_fops = {
	.open = simple_open,
	.write = iris_dbg_clock_gate_write,
	.read = iris_dbg_clock_gate_read,
};

int iris_debugfs_init(struct msm_fb_data_type *mfd)
{
	if (debugfs_create_file("iris_fw", 0644, NULL, mfd, &iris_dbg_fw_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_fbo", 0644, NULL, mfd, &iris_dbg_fbo_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_sbs", 0644, NULL, mfd, &iris_dbg_sbs_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_vsync_debug", 0644, NULL, mfd,
				&iris_dbg_vsync_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_ratio", 0644, NULL, mfd,
				&iris_dbg_ratio_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_mode_switch", 0644, NULL, mfd,
				&iris_dbg_mode_switch_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_repeat", 0644, NULL, mfd,
				&iris_dbg_repeat_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_dtg", 0644, NULL, mfd,
				&iris_dbg_dtg_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_te", 0644, NULL, mfd,
				&iris_dbg_te_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_new_frame", 0644, NULL, mfd,
				&iris_dbg_new_frame_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_new_repeat", 0644, NULL, mfd,
				&iris_dbg_new_repeat_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_send_meta", 0644, NULL, mfd,
				&iris_dbg_meta_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_frc_path", 0644, NULL, mfd,
				&iris_dbg_frc_path_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_input_vfr", 0644, NULL, mfd,
				&iris_dbg_input_vfr_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_hlmd", 0644, NULL, mfd,
				&iris_dbg_hlmd_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_set_usb_w", 0644, NULL, mfd,
				&iris_dbg_usb_w_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	if (debugfs_create_file("iris_clock_gate", 0644, NULL, mfd,
				&iris_dbg_clock_gate_fops)
			== NULL) {
		pr_err("debugfs_create_file: index fail\n");
		return -EFAULT;
	}

	return 0;
}

void mdss_dsi_iris_init_ctl(struct mdss_mdp_ctl *ctl)
{
	static int flag;

	if (flag == 0) {
		g_ctl0 = ctl;
		pr_debug("###%s:%d: g_ctl: %p\n", __func__, __LINE__, g_ctl0);
	} else if (flag == 1) {
		g_ctl1 = ctl;
		pr_debug("###%s:%d: g_ctl1: %p\n", __func__, __LINE__, g_ctl1);
	}
	flag++;
}

void mdss_dsi_iris_init_pdata(struct mdss_panel_data *pdata)
{
	static int flag;

	if (flag == 0) {
		g_pdata0 = pdata;
		pr_debug("###%s:%d: g_pdata: %p\n", __func__, __LINE__, g_pdata0);
	} else if (flag == 1) {
		g_pdata1 = pdata;
		pr_debug("###%s:%d: g_pdata1: %p\n", __func__, __LINE__, g_pdata1);
	}
	flag++;
}

void mdss_dsi_iris_init(struct msm_fb_data_type *mfd)
{
	pr_info("###%s:%d: mfd->panel.type: %i mfd->panel.id: %i\n", __func__, __LINE__, mfd->panel.type, mfd->panel.id);
	if (mfd->index != 0)
		return;
	if (!(mfd->panel.type == MIPI_VIDEO_PANEL || mfd->panel.type == MIPI_CMD_PANEL))
		return;

	g_mfd = mfd;
	pr_info("###%s:%d: g_mfd: %p\n", __func__, __LINE__, g_mfd);
	iris_mgmt.iris_wq = create_singlethread_workqueue("iris_wq");
	INIT_WORK(&iris_mgmt.iris_worker, iris_cmds_tx);

	mfd->iris_conf.current_mode = IRIS_PT_MODE;
	mfd->iris_conf.sf_notify_mode  = MDP_IRIS_MODE_RFB;
	mfd->iris_conf.mode_changed = false;
	mfd->iris_conf.fbo_enable = false;
	mfd->iris_conf.memc_enable = false;
	mfd->iris_conf.memc_perf_hack = false;
	mfd->iris_conf.mode_switch_finish = true;
	mfd->iris_conf.repeat = IRIS_REPEAT_NO;
	mfd->iris_conf.video_on = false;
	mfd->iris_conf.frc_path = 0x800000;
	atomic_set(&mfd->iris_conf.mode_switch_cnt, 0);
	mfd->iris_conf.input_frame_rate = 60;
	mfd->iris_conf.output_frame_rate = 60;
	mfd->iris_conf.input_vfr = 0;
	mfd->iris_conf.in_ratio = 1;
	mfd->iris_conf.out_ratio = 1;
	mfd->iris_conf.vp_continous = 0;
	mfd->iris_conf.nrv_enable = false;
	mfd->iris_conf.ready = false;
	mfd->iris_conf.prev_dvts = 0;
	mfd->iris_conf.iris_ratio_updated = false;
	mfd->iris_conf.cap_change = false;
	mfd->iris_conf.check_appcode_rfb_ready = false;
	mfd->iris_conf.check_pwil_rfb_ready = false;
#if defined(CONFIG_IRIS2_DRC_SUPPORT)
	mfd->iris_conf.drc_enable = false;
	mfd->iris_conf.drc_size = 0;
#endif
	memset((void *)&mfd->iris_conf.meta, 0, sizeof(struct iris_meta));
	memset((void *)&mfd->iris_conf.meta_set, 0, sizeof(struct iris_meta));
	spin_lock_init(&mfd->iris_conf.iris_reset_lock);
	iris_regs_clear();
	INIT_WORK(&mfd->iris_conf.pt_work, iris_pt_entry_wq_handler);
	INIT_WORK(&mfd->iris_conf.memc_work, iris_memc_entry_handler);
	INIT_WORK(&mfd->iris_conf.memc_prepare_work, iris_memc_prepare_handler);
	INIT_WORK(&mfd->iris_conf.memc_cancel_work, iris_memc_cancel_handler);
	mutex_init(&mfd->iris_conf.cmd_mutex);
	mutex_init(&mfd->iris_conf.config_mutex);
	mutex_init(&mfd->iris_conf.meta_mutex);
	/*allocate memory for firmware download*/
	g_firmware_buf = kzalloc(DSI_DMA_TX_BUF_SIZE, GFP_KERNEL);
	if (!g_firmware_buf)
		pr_err("%s: failed to alloc mem, size = %d\n", __func__, DSI_DMA_TX_BUF_SIZE);

	iris_debugfs_init(mfd);
}
