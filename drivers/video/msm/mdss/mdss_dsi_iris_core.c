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

#include "mdss_mdp.h"
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_dsi_iris.h"

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

/* Per Panel different */
#define IRIS_DTG_EVS_DLY   124
#define IRIS_DTG_E2OVS_DLY 4

/* debug in/out ratio flag */
static bool debug_hlmd_enabled;

struct iris_mgmt_t {
	struct work_struct iris_worker;
	struct workqueue_struct *iris_wq;
	void (*iris_handler)(void);
	bool fbo_enable;
	bool sbs_enable;
	struct msm_fb_data_type *mfd;
};
static struct iris_mgmt_t iris_mgmt;

static char pt_enable[2] = { 0x00, 0x00 };

static u8 iris_dbc_mode;

static struct dsi_cmd_desc iris_pt_enable_cmd[] = {
	{ { DTYPE_GEN_WRITE2,  1, 0, 0, 0, sizeof(pt_enable) }, pt_enable },
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

static struct dsi_cmd_desc pt_data_path_config[] = {
	{ { DTYPE_GEN_LWRITE, 1, 0, 0, 0,
		sizeof(iris_pt_enter_cmds) }, iris_pt_enter_cmds},
};

static struct dsi_cmd_desc pt_mode_enter[] = {
	{ { DTYPE_GEN_WRITE2, 1, 0, 0, 0,
		sizeof(pt_enable) }, pt_enable},

};

/* Assume the panel can accept this period increase */
#define IRIS_DELTA_VT_P        (31)
/* VFP needs at least 2 lines VSW needs at least 1 line */
#define IRIS_DELTA_VT_M        (-4)

static char panel_config_startflag[4] = {0x80, 0x87, 0x1, 0x0};
static char panel_config_finishflag[4] = {0x80, 0x87, 0x2, 0x0};
static char panel_off_flag[4] = {0x80, 0x87, 0x3, 0x0};

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
	PWIL_U32(0x80000126),
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
	PWIL_U32(0x381201),//for 2560x1600 sharp panel
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

static void iris_dump_packet(u8 *data, u8 size)
{
	int i = 0;

	pr_debug("size: %i\n", size);
	for (i = 0; i < size; i += 4)
		pr_debug("0x%02x 0x%02x 0x%02x 0x%02x\n",
			*(data+i), *(data+i+1), *(data+i+2), *(data+i+3));
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

#define IRIS_DTG_INIT_VTOTAL 2071
static u8 iris_extra_info[] = {
	0x80, 0x87, 0x0, 0x1,
	//dtg te width
	PWIL_U32(IRIS_DTG_ADDR + 0x0038),
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
	PWIL_U32(IRIS_DTG_ADDR + 0x10000),
	PWIL_U32(0x00000001),
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

	panel_cmds.cmds = iris_extra_info_cmds;
	panel_cmds.cmd_cnt = ARRAY_SIZE(iris_extra_info_cmds);
	panel_cmds.link_state = DSI_HS_MODE;
	mdss_dsi_panel_cmds_send_ex(ctrl, &panel_cmds);

}

void iris_panel_on_finish(struct mdss_dsi_ctrl_pdata *ctrl)
{
	//u8 diag = 0;
	//int i = 0;
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
	iris_cfg->ready = true;
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

static void iris_memc_prepare_handler(struct work_struct *work)
{
}

static void iris_memc_cancel_handler(struct work_struct *work)
{
}

static void iris_memc_entry_handler(struct work_struct *work)
{
}

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

int iris_debugfs_init(struct msm_fb_data_type *mfd)
{
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
	//memset((void *)&mfd->iris_conf.meta, 0, sizeof(struct iris_meta));
	//memset((void *)&mfd->iris_conf.meta_set, 0, sizeof(struct iris_meta));
	spin_lock_init(&mfd->iris_conf.iris_reset_lock);
	INIT_WORK(&mfd->iris_conf.pt_work, iris_pt_entry_wq_handler);
	INIT_WORK(&mfd->iris_conf.memc_work, iris_memc_entry_handler);
	INIT_WORK(&mfd->iris_conf.memc_prepare_work, iris_memc_prepare_handler);
	INIT_WORK(&mfd->iris_conf.memc_cancel_work, iris_memc_cancel_handler);
	mutex_init(&mfd->iris_conf.cmd_mutex);
	mutex_init(&mfd->iris_conf.config_mutex);
	mutex_init(&mfd->iris_conf.meta_mutex);

	iris_debugfs_init(mfd);
}
