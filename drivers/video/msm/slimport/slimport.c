/*
* Copyright(c) 2012-2013, Analogix Semiconductor All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
*/

#define pr_fmt(fmt) "%s %s: " fmt, "anx7805", __func__

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/slimport.h>
#include <linux/async.h>
#include <linux/of_platform.h>

#include "slimport_tx_drv.h"

#include <video/msm_dba.h>
#include "../msm_dba/msm_dba_internal.h"

#include <linux/slimport.h>
struct anx7805_data {
	struct i2c_client *client;
	struct anx7805_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock slimport_lock;
	struct msm_dba_device_info dev_info;
	int gpio_p_dwn;
	int gpio_reset;
	int gpio_int;
	int gpio_cbl_det;
	int gpio_v10_en;
	const char *vdd10_name;
	const char *avdd33_name;
	struct regulator *avdd_reg;
	struct regulator *vdd_reg;
//struct platform_device *hdmi_pdev;
//	struct msm_hdmi_sp_ops *hdmi_sp_ops;
	bool update_chg_type;
};

struct anx7805_data *the_chip;
/*
#ifdef HDCP_EN
static bool hdcp_enable = 1;
#else
static bool hdcp_enable;
#endif
*/
struct completion init_aux_ch_completion;
//static uint32_t sp_tx_chg_current_ma = NORMAL_CHG_I_MA;

static int notify_control = 0;

int EDID_ready = 0;
EXPORT_SYMBOL(EDID_ready);

extern BYTE bEDID_twoblock[256];
extern uint slimport_chip_id;

extern int msm_splash_handoff_done;

static struct anx7805_data *anx7805_get_platform_data(void *client)
{
	struct anx7805_data *pdata = NULL;
	struct msm_dba_device_info *dev;
	struct msm_dba_client_info *cinfo =
		(struct msm_dba_client_info *)client;

	if (!cinfo) {
		pr_err("%s: invalid client data\n", __func__);
		goto end;
	}

	dev = cinfo->dev;
	if (!dev) {
		pr_err("%s: invalid device data\n", __func__);
		goto end;
	}

	pdata = container_of(dev, struct anx7805_data, dev_info);
	if (!pdata)
		pr_err("%s: invalid platform data\n", __func__);

end:
	return pdata;
}

void anx7805_notify_clients(struct msm_dba_device_info *dev,
		enum msm_dba_callback_event event)
{
	struct msm_dba_client_info *c;
	struct list_head *pos = NULL;

	pr_info("%s++\n", __func__);

	if (!dev) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}

	list_for_each(pos, &dev->client_list) {
		c = list_entry(pos, struct msm_dba_client_info, list);

		pr_info("%s: notifying event %d to client %s\n", __func__,
			event, c->client_name);

		if (c && c->cb)
			c->cb(c->cb_data, event);
	}

	pr_info("%s--\n", __func__);
}
EXPORT_SYMBOL(anx7805_notify_clients);

static int anx7805_avdd_3p3_power(struct anx7805_data *chip, int on)
{
	static int on_state;
	int ret = 0;

	if (on_state == on) {
		pr_info("avdd 3.3V is already %s\n", on_state ? "on" : "off");
		goto out;
	}

	if (!chip->avdd_reg) {
		chip->avdd_reg = regulator_get(NULL, chip->avdd33_name);
		if (IS_ERR(chip->avdd_reg)) {
			ret = PTR_ERR(chip->avdd_reg);
			pr_err("regulator_get %s failed. rc = %d\n",
			       chip->avdd33_name, ret);
			chip->avdd_reg = NULL;
			goto out;
		}
	}

	if (on) {
		ret = regulator_enable(chip->avdd_reg);
		if (ret) {
			pr_err("avdd_reg enable failed (%d)\n", ret);
			goto err_reg;
		}
	} else {
		ret = regulator_disable(chip->avdd_reg);
		if (ret) {
			pr_err("avdd_reg disable failed (%d)\n", ret);
			goto err_reg;
		}
	}

	on_state = on;
	return 0;

err_reg:
	regulator_put(chip->avdd_reg);
	chip->avdd_reg = NULL;
out:
	return ret;
}

static int anx7805_vdd_1p0_power(struct anx7805_data *chip, int on)
{
	static int on_state;
	int ret = 0;

	if (on_state == on) {
		pr_info("vdd 1.0V is already %s\n", on_state ? "on" : "off");
		goto out;
	}

	if (!chip->vdd_reg) {
		chip->vdd_reg = regulator_get(NULL, chip->vdd10_name);
		if (IS_ERR(chip->vdd_reg)) {
			ret = PTR_ERR(chip->vdd_reg);
			pr_err("regulator_get %s failed. ret = %d\n",
			       chip->vdd10_name, ret);
			chip->vdd_reg = NULL;
			goto out;
		}
	}

	if (on) {
		ret = regulator_enable(chip->vdd_reg);
		if (ret) {
			pr_err("vdd_reg enable failed (%d)\n", ret);
			goto err_reg;
		}

		/* trun on 1.0V by gpio_v10_en after PR */
		if (g_ASUS_hwID >= 4)
			gpio_direction_output(chip->gpio_v10_en, 1);
	} else {
		ret = regulator_disable(chip->vdd_reg);
		if (ret) {
			pr_err("vdd_reg disable failed (%d)\n", ret);
			goto err_reg;
		}

		/* trun off 1.0V by gpio_v10_en after PR */
		if (g_ASUS_hwID >= 4)
			gpio_direction_output(chip->gpio_v10_en, 0);
	}

	on_state = on;
	return 0;

err_reg:
	regulator_put(chip->vdd_reg);
	chip->vdd_reg = NULL;
out:
	return ret;
}

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;

	if (!the_chip)
		return -EINVAL;

	the_chip->client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(the_chip->client, offset);
	if (ret < 0) {
		pr_err("failed to read i2c addr=%x\n", slave_addr);
		return ret;
	}
	*buf = (uint8_t) ret;

	return 0;
}

int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value)
{
	int ret = 0;

	if (!the_chip)
		return -EINVAL;

	the_chip->client->addr = (slave_addr >> 1);
	ret = i2c_smbus_write_byte_data(the_chip->client, offset, value);
	if (ret < 0) {
		pr_err("failed to write i2c addr=%x\n", slave_addr);
	}
	return ret;
}

void sp_tx_hardware_poweron(void)
{
	if (!the_chip)
		return;

	gpio_direction_output(the_chip->gpio_reset, 0);
	msleep(1);
	gpio_direction_output(the_chip->gpio_p_dwn, 0);
	msleep(2);
	anx7805_vdd_1p0_power(the_chip, 1);
	msleep(5);
	gpio_direction_output(the_chip->gpio_reset, 1);

	pr_info("anx7805 power on\n");
}

void sp_tx_hardware_powerdown(void)
{
//int status = 0;

	if (!the_chip)
		return;

	gpio_direction_output(the_chip->gpio_reset, 0);
	msleep(1);
	anx7805_vdd_1p0_power(the_chip, 0);
	msleep(2);
	gpio_direction_output(the_chip->gpio_p_dwn, 1);
	msleep(1);

	/* turn off hpd */
	/*
	if (the_chip->hdmi_sp_ops->set_upstream_hpd) {
	status = the_chip->hdmi_sp_ops->set_upstream_hpd(
	the_chip->hdmi_pdev, 0);
	if (status)
	pr_err("failed to turn off hpd");
	}
	*/
	pr_info("anx7805 power down\n");
}

/*
static void sp_tx_power_down_and_init(void)
{
	vbus_power_ctrl();
	sp_tx_power_down(SP_TX_PWR_REG);
	sp_tx_power_down(SP_TX_PWR_TOTAL);
	sp_tx_hardware_powerdown();
	sp_tx_pd_mode = 1;
	sp_tx_link_config_done = 0;
	sp_tx_hw_lt_enable = 0;
	sp_tx_hw_lt_done = 0;
	sp_tx_rx_type = RX_NULL;
	sp_tx_rx_type_backup = RX_NULL;
	sp_tx_set_sys_state(STATE_CABLE_PLUG);
}

*/



int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	if (block == 0) {
		memcpy(edid_buf, bEDID_firstblock, sizeof(bEDID_firstblock));
	} else if (block == 1) {
		memcpy(edid_buf, bEDID_extblock, sizeof(bEDID_extblock));
	} else {
		pr_err("%s: block number %d is invalid\n", __func__, block);
		return -EINVAL;
	}

	return 0;
}

EXPORT_SYMBOL(slimport_read_edid_block);

int update_audio_format_setting(unsigned char  bAudio_Fs, unsigned char bAudio_word_len, int Channel_Num, I2SLayOut layout)
{
	//pr_info("bAudio_Fs = %d, bAudio_word_len = %d, Channel_Num = %d, layout = %d\n", bAudio_Fs, bAudio_word_len, Channel_Num, layout); //liu
	SP_CTRL_AUDIO_FORMAT_Set(AUDIO_I2S,bAudio_Fs ,bAudio_word_len);
	SP_CTRL_I2S_CONFIG_Set(Channel_Num , layout);
	audio_format_change=1;

	return 0;
}
EXPORT_SYMBOL(update_audio_format_setting);

int hdcp_eanble_setting(bool on)
{
	hdcp_enable=on;
	return 0;
}
EXPORT_SYMBOL(hdcp_eanble_setting);

static int anx7805_mipi_timing_setting(void *client, bool on,
		struct msm_dba_video_cfg *cfg, u32 flags)
{
	if (!cfg) {
		pr_err("%s: invalid input\n", __func__);
		return 1;
	}

	mipi_video_timing_table[bMIPIFormatIndex].MIPI_HTOTAL = cfg->h_active + cfg->h_front_porch +
	      cfg->h_pulse_width + cfg->h_back_porch;
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_VTOTAL= cfg->v_active + cfg->v_front_porch +
	      cfg->v_pulse_width + cfg->v_back_porch;

	mipi_video_timing_table[bMIPIFormatIndex].MIPI_HActive = cfg->h_active;
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_H_Sync_Width= cfg->h_pulse_width;
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_H_Front_Porch= cfg->h_front_porch;
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_H_Back_Porch= cfg->h_back_porch;

	mipi_video_timing_table[bMIPIFormatIndex].MIPI_VActive = cfg->v_active;
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_V_Sync_Width= cfg->v_pulse_width;
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_V_Front_Porch= cfg->v_front_porch;
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_V_Back_Porch= cfg->v_back_porch;

	mipi_lane_count = cfg->num_of_input_lanes;
	//mipi_video_timing_table[bMIPIFormatIndex].MIPI_pixel_frequency=(unsigned int)(cfg->pclk_khz/1000);
	mipi_video_timing_table[bMIPIFormatIndex].MIPI_pixel_frequency =
		mipi_video_timing_table[bMIPIFormatIndex].MIPI_HTOTAL *
		mipi_video_timing_table[bMIPIFormatIndex].MIPI_VTOTAL * 60 / 1000;

	pr_info("cfg->pclk_khz = %d\n", cfg->pclk_khz);

	pr_info("h_total = %d, h_active = %d, hfp = %d, hpw = %d, hbp = %d\n",
		mipi_video_timing_table[bMIPIFormatIndex].MIPI_HTOTAL, cfg->h_active, cfg->h_front_porch,
		cfg->h_pulse_width, cfg->h_back_porch);

	pr_info("v_total = %d, v_active = %d, vfp = %d, vpw = %d, vbp = %d\n",
		mipi_video_timing_table[bMIPIFormatIndex].MIPI_VTOTAL, cfg->v_active, cfg->v_front_porch,
		cfg->v_pulse_width, cfg->v_back_porch);

	pr_info("pixel clock = %ld, lane count = %d, \n", mipi_video_timing_table[bMIPIFormatIndex].MIPI_pixel_frequency, cfg->num_of_input_lanes);

	return 0;
}

bool slimport_is_connected(void)
{
	bool result = false;

	if (!the_chip)
		return false;

	if (gpio_get_value_cansleep(the_chip->gpio_cbl_det)) {
		mdelay(10);
		if (gpio_get_value_cansleep(the_chip->gpio_cbl_det)) {
			pr_info("slimport cable is detected\n");
			result = true;
		}
	}

	return result;
}
EXPORT_SYMBOL(slimport_is_connected);


static void anx7805_free_gpio(struct anx7805_data *anx7805)
{
	gpio_free(anx7805->gpio_cbl_det);
	gpio_free(anx7805->gpio_int);
	gpio_free(anx7805->gpio_reset);
	gpio_free(anx7805->gpio_p_dwn);
}

static int anx7805_pinctrl_configure(struct pinctrl *key_pinctrl, bool active)
{
	struct pinctrl_state *set_state;
	int retval;

	if (active) {
		set_state = pinctrl_lookup_state(key_pinctrl, "pmx_anx_int_active");
		if (IS_ERR(set_state)) {
			pr_err("%s: cannot get anx7805 pinctrl active state\n", __func__);
			return PTR_ERR(set_state);
		}
	}
	else {
		/* suspend setting here */
	}
	retval = pinctrl_select_state(key_pinctrl, set_state);
	if (retval) {
		pr_err("%s: cannot set anx7805 pinctrl state\n", __func__);
		return retval;
	}

	pr_info("%s: configure pinctrl success\n", __func__);
	return 0;
}

static int anx7805_init_gpio(struct anx7805_data *anx7805)
{
	int ret = 0;
	struct pinctrl *key_pinctrl;

	pr_info("%s+\n", __func__);

	/* Get pinctrl if target uses pinctrl */
	key_pinctrl = devm_pinctrl_get(&anx7805->client->dev);
	if (IS_ERR(key_pinctrl)) {
		if (PTR_ERR(key_pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		pr_debug("Target does not use pinctrl\n");
		key_pinctrl = NULL;
	}
	if (key_pinctrl) {
		pr_debug("Target uses pinctrl\n");
		ret = anx7805_pinctrl_configure(key_pinctrl, true);
		if (ret)
			pr_err("%s: cannot configure anx_int pinctrl\n", __func__);
	}

	ret = gpio_request_one(anx7805->gpio_p_dwn,
	                       GPIOF_OUT_INIT_LOW, "anx_p_dwn_ctl");
	if (ret) {
		pr_err("failed to request gpio %d\n", anx7805->gpio_p_dwn);
		goto out;
	}

	ret = gpio_request_one(anx7805->gpio_reset,
	                       GPIOF_OUT_INIT_LOW, "anx7805_reset_n");
	if (ret) {
		pr_err("failed to request gpio %d\n", anx7805->gpio_reset);
		goto err0;
	}

	ret = gpio_request_one(anx7805->gpio_int,
	                       GPIOF_IN, "anx7805_int_n");

	if (ret) {
		pr_err("failed to request gpio %d\n", anx7805->gpio_int);
		goto err1;
	}

	ret = gpio_request_one(anx7805->gpio_cbl_det,
	                       GPIOF_IN, "anx7805_cbl_det");
	if (ret) {
		pr_err("failed to request gpio %d\n", anx7805->gpio_cbl_det);
		goto err2;
	}

	/* request gpio_v10_en for PR */
	if (g_ASUS_hwID >= 4) {
		ret = gpio_request_one(anx7805->gpio_v10_en,
		                       GPIOF_OUT_INIT_LOW, "anx7805_v10_en");
		if (ret) {
			pr_err("failed to request gpio %d\n", anx7805->gpio_v10_en);
			goto err3;
		}
	}

	gpio_direction_input(anx7805->gpio_cbl_det);

	gpio_direction_output(anx7805->gpio_reset, 0);
//	gpio_direction_output(anx7805->gpio_p_dwn, 1);

	goto out;

err3:
	gpio_free(anx7805->gpio_v10_en);
err2:
	gpio_free(anx7805->gpio_int);
err1:
	gpio_free(anx7805->gpio_reset);
err0:
	gpio_free(anx7805->gpio_p_dwn);
out:
	pr_info("%s-\n", __func__);
	return ret;
}

static int anx7805_system_init(void)
{
	int ret = 0;

	ret = SP_CTRL_Chip_Detect();
	if (ret == 0) {
		pr_err("failed to detect anx7805\n");
		return -ENODEV;
	}

	SP_CTRL_Chip_Initial();
	return 0;
}

static irqreturn_t anx7805_cbl_det_isr(int irq, void *data)
{
	struct anx7805_data *anx7805 = data;
	int status;

	if (gpio_get_value(anx7805->gpio_cbl_det)) {
		wake_lock(&anx7805->slimport_lock);
		pr_info("detect cable insertion\n");
		queue_delayed_work(anx7805->workqueue, &anx7805->work, 0);
	} else {		
		/* check HPD state again after 5 ms to see if it is HPD irq event */
		#ifdef Standard_DP
		mdelay(5);

		if (!gpio_get_value(anx7805->gpio_cbl_det)) { // if it is one IRQ, should not destroy ANX7805 work queue
		#endif
			
		pr_info("detect cable removal\n");
		status = cancel_delayed_work_sync(&anx7805->work);
		if (status == 0)
			flush_workqueue(anx7805->workqueue);
		//when HPD low, power down ANX7805
		if(sp_tx_pd_mode==0)
		{
			SP_CTRL_Set_System_State(SP_TX_WAIT_SLIMPORT_PLUGIN);
			system_power_ctrl(0);
		}
		
		wake_unlock(&anx7805->slimport_lock);
		wake_lock_timeout(&anx7805->slimport_lock, 2*HZ);



			/* Notify DBA framework disconnect event */
			anx7805_notify_clients(&anx7805->dev_info,
				MSM_DBA_CB_HPD_DISCONNECT);

			/* clear notify_control */
			notify_control = 0;

			/* clear EDID_ready */
			EDID_ready = 0;
		#ifdef Standard_DP
		}
		#endif
	}
	return IRQ_HANDLED;
}

static void anx7805_work_func(struct work_struct *work)
{
#ifndef EYE_TEST
	struct anx7805_data *td = container_of(work, struct anx7805_data,
	                                       work.work);
	int workqueu_timer = 0;
	if(get_system_state() >= SP_TX_PLAY_BACK)
		workqueu_timer = 500;
	else
		workqueu_timer = 100;

	if (!msm_splash_handoff_done) {
		pr_info("%s: mdp splash handoff does not finish, delay 3 seconds.\n", __func__);
		queue_delayed_work(td->workqueue, &td->work, msecs_to_jiffies(3000));
		return;
	}

	SP_CTRL_Main_Procss();
	queue_delayed_work(td->workqueue, &td->work,
	                   msecs_to_jiffies(workqueu_timer));

	if (!notify_control && EDID_ready && slimport_is_connected()) {
		/* Notify DBA framework connect event */
		anx7805_notify_clients(&td->dev_info,
				MSM_DBA_CB_HPD_CONNECT);

		notify_control = 1;
	}
#endif
}

static int anx7805_parse_dt(struct device_node *node,
                            struct anx7805_data *anx7805)
{
	int ret = 0;
//struct platform_device *hdmi_pdev = NULL;
	struct device_node *hdmi_tx_node = NULL;

	anx7805->gpio_p_dwn =
	    of_get_named_gpio(node, "analogix,p-dwn-gpio", 0);
	if (anx7805->gpio_p_dwn < 0) {
		pr_err("failed to get analogix,p-dwn-gpio.\n");
		ret = anx7805->gpio_p_dwn;
		goto out;
	}

	anx7805->gpio_reset =
	    of_get_named_gpio(node, "analogix,reset-gpio", 0);
	if (anx7805->gpio_reset < 0) {
		pr_err("failed to get analogix,reset-gpio.\n");
		ret = anx7805->gpio_reset;
		goto out;
	}

	anx7805->gpio_int =
	    of_get_named_gpio(node, "analogix,irq-gpio", 0);
	if (anx7805->gpio_int < 0) {
		pr_err("failed to get analogix,irq-gpio.\n");
		ret = anx7805->gpio_int;
		goto out;
	}

	anx7805->gpio_cbl_det =
	    of_get_named_gpio(node, "analogix,cbl-det-gpio", 0);
	if (anx7805->gpio_cbl_det < 0) {
		pr_err("failed to get analogix,cbl-det-gpio.\n");
		ret = anx7805->gpio_cbl_det;
		goto out;
	}

	anx7805->gpio_v10_en =
	    of_get_named_gpio(node, "analogix,v10-en-gpio", 0);
	if (anx7805->gpio_v10_en < 0) {
		pr_err("failed to get analogix,v10-en-gpio.\n");
		ret = anx7805->gpio_v10_en;
		goto out;
	}

	ret = of_property_read_string(node, "analogix,vdd10-name",
	                              &anx7805->vdd10_name);
	if (ret) {
		pr_err("failed to get vdd10-name.\n");
		goto out;
	}

	ret = of_property_read_string(node, "analogix,avdd33-name",
	                              &anx7805->avdd33_name);
	if (ret) {
		pr_err("failed to get avdd33-name.\n");
		goto out;
	}

	/* parse phandle for hdmi tx handle */
	hdmi_tx_node = of_parse_phandle(node, "analogix,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("can't find hdmi phandle\n");
//		ret = -EINVAL;
//		goto out;
	}
	/*
	hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!hdmi_pdev) {
	pr_err("can't find the deivce by node\n");
	ret = -EINVAL;
	goto out;
	}
	anx7805->hdmi_pdev = hdmi_pdev;
	*/
out:
	return ret;
}

static int anx7805_get_raw_edid(void *client,
	u32 size, char *buf, u32 flags)
{
	struct anx7805_data *pdata =
		anx7805_get_platform_data(client);

	if (!pdata || !buf) {
		pr_err("%s: invalid data\n", __func__);
		goto end;
	}

	mutex_lock(&pdata->lock);

	pr_info("%s: size=%d\n", __func__, size);
	size = min_t(u32, size, sizeof(bEDID_twoblock));

	pr_info("%s: memcpy EDID block, size=%d\n", __func__, size);
	memcpy(buf, bEDID_twoblock, size);

	mutex_unlock(&pdata->lock);
end:
	return 0;
}

static int anx7805_configure_audio(void *client,
		struct msm_dba_audio_cfg *cfg, u32 flags)
{

	pr_info("%s: configure anx7805 I2S\n", __func__);

	/* Fix me should configure anx7805 I2S by user space */

	update_audio_format_setting(AUDIO_FS_48K, AUDIO_W_LEN_24_24MAX,
			I2S_CH_2,I2S_LAYOUT_0);

	return 0;
}

static int anx7805_register_dba(struct anx7805_data *pdata)
{
	struct msm_dba_ops *client_ops;
	struct msm_dba_device_ops *dev_ops;

	if (!pdata)
		return -EINVAL;

	client_ops = &pdata->dev_info.client_ops;
	dev_ops = &pdata->dev_info.dev_ops;

	client_ops->power_on        = NULL;
	client_ops->video_on        = anx7805_mipi_timing_setting;
	client_ops->configure_audio = anx7805_configure_audio;
	client_ops->hdcp_enable     = NULL;
	client_ops->hdmi_cec_on     = NULL;
	client_ops->hdmi_cec_write  = NULL;
	client_ops->hdmi_cec_read   = NULL;
	client_ops->get_edid_size   = NULL;
	client_ops->get_raw_edid    = anx7805_get_raw_edid;
	client_ops->check_hpd	    = NULL;

	strlcpy(pdata->dev_info.chip_name, "anx7805",
		sizeof(pdata->dev_info.chip_name));

	pdata->dev_info.instance_id = 0;

	mutex_init(&pdata->dev_info.dev_mutex);

	INIT_LIST_HEAD(&pdata->dev_info.client_list);

	return msm_dba_add_probed_device(&pdata->dev_info);
}

static ssize_t anx7805_rev_check_show(
	struct device *dev, struct device_attribute *attr,
	 char *buf)
{
	/* slimport_chip_id should be 0x7805 */
	pr_info("slimport_chip_id = 0x%x\n", slimport_chip_id & 0x0000FFFF);

	return sprintf(buf, "%d\n", ((slimport_chip_id & 0x0000FFFF) == 0x7805) ? 1 : 0);
}

static ssize_t anx7805_rev_check_store(
	struct device *dev, struct device_attribute *attr,
	 const char *buf, size_t count)
{
	int cmd;

	sscanf(buf, "%d", &cmd);

	switch (cmd) {
		case 1:
			pr_info("%s: read anx7805 chip id\n", __func__);
			anx7805_system_init();
			break;

		default:
			pr_info("%s: unknown command\n", __func__);
			break;
	}
	return count;
}

static struct device_attribute slimport_device_attrs[] = {
	__ATTR(rev_check, S_IRUGO | S_IWUSR, anx7805_rev_check_show, anx7805_rev_check_store),
};

static int anx7805_create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(slimport_device_attrs); i++)
		if (device_create_file(dev, &slimport_device_attrs[i]))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, &slimport_device_attrs[i]);

	return -EINVAL;
}

static int anx7805_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
	struct anx7805_data *anx7805;
	struct anx7805_platform_data *pdata;
	struct device_node *dev_node = client->dev.of_node;
	//struct msm_hdmi_sp_ops *hdmi_sp_ops = NULL;
	int ret = 0;

	pr_info("%s++\n", __func__);

	if (!i2c_check_functionality(client->adapter,
	                             I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("i2c bus does not support anx7805\n");
		ret = -ENODEV;
		goto exit;
	}

	pr_info("%s: i2c device name=%s, addr=0x%x, adapter nr=%d\n", __func__,
			client->name, client->addr, client->adapter->nr);

	anx7805 = kzalloc(sizeof(struct anx7805_data), GFP_KERNEL);
	if (!anx7805) {
		pr_err("failed to allocate driver data\n");
		ret = -ENOMEM;
		goto exit;
	}

	anx7805->client = client;
	i2c_set_clientdata(client, anx7805);

	if (dev_node) {
		ret = anx7805_parse_dt(dev_node, anx7805);
		if (ret) {
			pr_err("failed to parse dt\n");
			goto err0;
		}
	} else {
		pdata = client->dev.platform_data;
		if (pdata == NULL) {
			pr_err("no platform data.\n");
			goto err0;
		}

		anx7805->gpio_p_dwn = pdata->gpio_p_dwn;
		anx7805->gpio_reset = pdata->gpio_reset;
		anx7805->gpio_int = pdata->gpio_int;
		anx7805->gpio_cbl_det = pdata->gpio_cbl_det;
		anx7805->vdd10_name = pdata->vdd10_name;
		anx7805->avdd33_name = pdata->avdd33_name;		
	}

	/* initialize hdmi_sp_ops */
	/*
	hdmi_sp_ops = devm_kzalloc(&client->dev,
	                           sizeof(struct msm_hdmi_sp_ops),
	                           GFP_KERNEL);
	if (!hdmi_sp_ops) {
		pr_err("alloc hdmi sp ops failed\n");
		goto err0;
	}
	
	if (anx7805->hdmi_pdev) {
	ret = msm_hdmi_register_sp(anx7805->hdmi_pdev,
	hdmi_sp_ops);
	if (ret) {
	pr_err("register with hdmi_failed\n");
	goto err0;
	}
	}
	
	anx7805->hdmi_sp_ops = hdmi_sp_ops;
*/
	the_chip = anx7805;

	mutex_init(&anx7805->lock);
	init_completion(&init_aux_ch_completion);
	ret = anx7805_init_gpio(anx7805);
	if (ret) {
		pr_err("failed to initialize gpio\n");
		goto err0;
	}

	INIT_DELAYED_WORK(&anx7805->work, anx7805_work_func);

	anx7805->workqueue = create_singlethread_workqueue("anx7805_work");
	if (!anx7805->workqueue) {
		pr_err("failed to create work queue\n");
		ret = -ENOMEM;
		goto err1;
	}

	ret = anx7805_avdd_3p3_power(anx7805, true);
	if (ret)
		goto err2;

	ret = anx7805_vdd_1p0_power(anx7805, false);
	if (ret)
		goto err3;

	ret = anx7805_system_init();
	if (ret) {
		pr_err("failed to initialize anx7805\n");
		goto err4;
	}

	client->irq = gpio_to_irq(anx7805->gpio_cbl_det);
	if (client->irq < 0) {
		pr_err("failed to get gpio irq\n");
		goto err4;
	}

	wake_lock_init(&anx7805->slimport_lock, WAKE_LOCK_SUSPEND,
	               "slimport_wake_lock");

	ret = request_threaded_irq(client->irq, NULL, anx7805_cbl_det_isr,
	                           IRQF_TRIGGER_RISING
	                           | IRQF_TRIGGER_FALLING
	                           | IRQF_ONESHOT,
	                           "anx7805", anx7805);
	if (ret < 0) {
		pr_err("failed to request irq\n");
		goto err5;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("interrupt wake enable fail\n");
		goto err6;
	}

	ret = anx7805_create_sysfs_interfaces(&client->dev);
	if (ret)
		pr_err("%s: fail to create sysfs interface %d\n", __func__, ret);

	/* Register msm dba device */
	ret = anx7805_register_dba(anx7805);
	if (ret) {
		pr_err("%s: Error registering with DBA %d\n",
			__func__, ret);
	}

	pr_info("%s succeed!\n", __func__);

	goto exit;

err6:
	free_irq(client->irq, anx7805);
err5:
	wake_lock_destroy(&anx7805->slimport_lock);
err4:
	if (!anx7805->vdd_reg)
		regulator_put(anx7805->vdd_reg);
err3:
	if (!anx7805->avdd_reg)
		regulator_put(anx7805->avdd_reg);
err2:
	destroy_workqueue(anx7805->workqueue);
err1:
	anx7805_free_gpio(anx7805);
err0:
	the_chip = NULL;
	kfree(anx7805);
exit:
	pr_info("%s--\n", __func__);
	return ret;
}

static int anx7805_i2c_remove(struct i2c_client *client)
{
	struct anx7805_data *anx7805 = i2c_get_clientdata(client);

	free_irq(client->irq, anx7805);
	wake_lock_destroy(&anx7805->slimport_lock);
	if (!anx7805->vdd_reg)
		regulator_put(anx7805->vdd_reg);
	if (!anx7805->avdd_reg)
		regulator_put(anx7805->avdd_reg);
	destroy_workqueue(anx7805->workqueue);
	anx7805_free_gpio(anx7805);
	the_chip = NULL;
	kfree(anx7805);
	return 0;
}

static const struct i2c_device_id anx7805_id[] = {
	{ "anx7805", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, anx7805_id);

static struct of_device_id anx_match_table[] = {
	{ .compatible = "analogix,anx7805",},
	{ },
};

static struct i2c_driver anx7805_driver = {
	.driver = {
		.name = "anx7805",
		.owner = THIS_MODULE,
		.of_match_table = anx_match_table,
	},
	.probe = anx7805_i2c_probe,
	.remove = anx7805_i2c_remove,
	.id_table = anx7805_id,
};

static void __init anx7805_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	pr_info("%s\n", __func__);

	ret = i2c_add_driver(&anx7805_driver);
	if (ret)
		pr_err("%s: failed to register anx7805 driver\n", __func__);
}

static int __init anx7805_init(void)
{
	pr_info("%s\n", __func__);
	async_schedule(anx7805_init_async, NULL);
	return 0;
}

static void __exit anx7805_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&anx7805_driver);
}

module_init(anx7805_init);
module_exit(anx7805_exit);

MODULE_DESCRIPTION("Slimport transmitter ANX7805 driver");
MODULE_AUTHOR("swang@analogixsemi.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");
