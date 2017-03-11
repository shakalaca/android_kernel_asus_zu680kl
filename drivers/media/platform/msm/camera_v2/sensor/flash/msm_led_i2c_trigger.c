/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_led_flash.h"
#include "msm_camera_io_util.h"
#include "../msm_sensor.h"
#include "../cci/msm_cci.h"
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#define FLASH_NAME "camera-led-flash"
#define CAM_FLASH_PINCTRL_STATE_SLEEP "cam_flash_suspend"
#define CAM_FLASH_PINCTRL_STATE_DEFAULT "cam_flash_default"
/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

extern int g_ftm_mode;

static void *g_fctrl;
//ASUS_BSP +++ PJ "implement asus_flash control node"
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
//ASUS_BSP --- PJ "implement asus_flash control node"
static int ATD_status; //ASUS_BSP PJ "add flash status"
#define MAX_OTG_TORCH_CURRENT 50
#define MAX_TORCH_CURRENT 200
#define MAX_FLASH_CURRENT 900
#define MAX_FLASH_DURATION 800
#define MAX_ZENFLASH_DURATION 80
#define ZENFLASH_MIN_CURRENT 300
#define ZENFLASH_CURRENT 900
#define ZENFLASH_TESTTIME_RESET_CMD 100
int32_t msm_led_i2c_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	*subdev_id = fctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	return 0;
}

int32_t msm_led_i2c_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	int i = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	CDBG("called led_state %d\n", cfg->cfgtype);

	if (!fctrl->func_tbl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	switch (cfg->cfgtype) {

	case MSM_CAMERA_LED_INIT:
		if (fctrl->func_tbl->flash_led_init)
			rc = fctrl->func_tbl->flash_led_init(fctrl);
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			cfg->flash_current[i] =
				fctrl->flash_max_current[i];
			cfg->flash_duration[i] =
				fctrl->flash_max_duration[i];
			cfg->torch_current[i] =
				fctrl->torch_max_current[i];
		}
		break;

	case MSM_CAMERA_LED_RELEASE:
		if (fctrl->func_tbl->flash_led_release)
			rc = fctrl->func_tbl->
				flash_led_release(fctrl);
		break;

	case MSM_CAMERA_LED_OFF:
		if (fctrl->func_tbl->flash_led_off)
			rc = fctrl->func_tbl->flash_led_off(fctrl);
		break;

	case MSM_CAMERA_LED_LOW:
		for (i = 0; i < fctrl->torch_num_sources; i++) {
			if (fctrl->torch_max_current[i] > 0) {
				fctrl->torch_op_current[i] =
					(cfg->torch_current[i] < fctrl->torch_max_current[i]) ?
					cfg->torch_current[i] : fctrl->torch_max_current[i];
				CDBG("torch source%d: op_current %d max_current %d\n",
					i, fctrl->torch_op_current[i], fctrl->torch_max_current[i]);
			}
		}
		if (fctrl->func_tbl->flash_led_low)
			rc = fctrl->func_tbl->flash_led_low(fctrl);
		break;

	case MSM_CAMERA_LED_HIGH:
		for (i = 0; i < fctrl->flash_num_sources; i++) {
			if (fctrl->flash_max_current[i] > 0) {
				fctrl->flash_op_current[i] =
					(cfg->flash_current[i] < fctrl->flash_max_current[i]) ?
					cfg->flash_current[i] : fctrl->flash_max_current[i];
				CDBG("flash source%d: op_current %d max_current %d\n",
					i, fctrl->flash_op_current[i], fctrl->flash_max_current[i]);
			}
		}
		if (fctrl->func_tbl->flash_led_high)
			rc = fctrl->func_tbl->flash_led_high(fctrl);
		break;
	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}
static int msm_flash_pinctrl_init(struct msm_led_flash_ctrl_t *ctrl)
{
	struct msm_pinctrl_info *flash_pctrl = NULL;

	flash_pctrl = &ctrl->pinctrl_info;

	if (ctrl->pdev != NULL)
		flash_pctrl->pinctrl = devm_pinctrl_get(&ctrl->pdev->dev);
	else
		flash_pctrl->pinctrl = devm_pinctrl_get(&ctrl->
					flash_i2c_client->
					client->dev);
	if (IS_ERR_OR_NULL(flash_pctrl->pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_active = pinctrl_lookup_state(
					       flash_pctrl->pinctrl,
					       CAM_FLASH_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_suspend = pinctrl_lookup_state(
						flash_pctrl->pinctrl,
						CAM_FLASH_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

struct msm_led_flash_ctrl_t *gBsp_led_flash_ctrl = 0;
EXPORT_SYMBOL(gBsp_led_flash_ctrl);
int msm_flash_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL)
		pr_err("%s:%d mux install\n", __func__, __LINE__);

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			return rc;
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		CDBG("%s:%d PC:: flash pins setting to active state",
				__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}
	msleep(20);

	CDBG("before FL_RESET\n");
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	fctrl->led_state = MSM_CAMERA_LED_INIT;
	return rc;
}

int msm_flash_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0, ret = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s:%d called\n", __func__, __LINE__);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->led_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		return -EINVAL;
	}
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_LOW);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_RESET],
				GPIO_OUT_LOW);

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		ret = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (ret < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	return 0;
}

int msm_flash_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CDBG("%s:%d called\n", __func__, __LINE__);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_LOW);

	return rc;
}

int msm_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);


	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_NOW],
			GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}

int msm_flash_led0(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led0_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_led1(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led1_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_led01(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led01_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_led2(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led2_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_led0_flash(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led0_flash_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_led1_flash(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led1_flash_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_led01_flash(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led01_flash_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_led2_flash(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->led2_flash_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
int msm_flash_write_custom_commands(struct msm_led_flash_ctrl_t *fctrl,
		struct msm_camera_i2c_reg_setting *custom_flash_setting)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_EN],
		GPIO_OUT_HIGH);

	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_NOW] == 1)
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			custom_flash_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	return rc;
}
static int32_t msm_led_get_dt_data(struct device_node *of_node,
		struct msm_led_flash_ctrl_t *fctrl)
{
	int32_t rc = 0, i = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct device_node *flash_src_node = NULL;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint32_t count = 0;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	uint32_t id_info[3];

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->flashdata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->flashdata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&flashdata->sensor_name);
	CDBG("%s label %s, rc %d\n", __func__,
		flashdata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_i2c_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	fctrl->pinctrl_info.use_pinctrl = false;
	fctrl->pinctrl_info.use_pinctrl = of_property_read_bool(of_node,
						"qcom,enable_pinctrl");
	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("failed\n");
			return -EINVAL;
		}
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl->flash_trigger_name[i]);
			if (rc < 0) {
				pr_err("failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				 fctrl->flash_trigger_name[i]);

			rc = of_property_read_u32(flash_src_node,
				"qcom,max-current",
				&fctrl->flash_op_current[i]);
			if (rc < 0) {
				pr_err("failed rc %d\n", rc);
				of_node_put(flash_src_node);
				continue;
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->flash_op_current[i]);

			led_trigger_register_simple(
				fctrl->flash_trigger_name[i],
				&fctrl->flash_trigger[i]);
		}

	} else { /*Handle LED Flash Ctrl by GPIO*/
		power_info->gpio_conf =
			 kzalloc(sizeof(struct msm_camera_gpio_conf),
				 GFP_KERNEL);
		if (!power_info->gpio_conf) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			return rc;
		}
		gconf = power_info->gpio_conf;

		gpio_array_size = of_gpio_count(of_node);
		CDBG("%s gpio count %d\n", __func__, gpio_array_size);

		if (gpio_array_size) {
			gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
				GFP_KERNEL);
			if (!gpio_array) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				rc = -ENOMEM;
				goto ERROR4;
			}
			for (i = 0; i < gpio_array_size; i++) {
				gpio_array[i] = of_get_gpio(of_node, i);
				CDBG("%s gpio_array[%d] = %d\n", __func__, i,
					gpio_array[i]);
			}

			rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR4;
			}

			rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR5;
			}

			rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR6;
			}
		}

		/* Read the max current for an LED if present */
		if (of_get_property(of_node, "qcom,max-current", &count)) {
			count /= sizeof(uint32_t);

			if (count > MAX_LED_TRIGGERS) {
				pr_err("failed\n");
				rc = -EINVAL;
				goto ERROR8;
			}

			fctrl->flash_num_sources = count;
			fctrl->torch_num_sources = count;

			rc = of_property_read_u32_array(of_node,
				"qcom,max-current",
				fctrl->flash_max_current, count);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR8;
			}

			for (; count < MAX_LED_TRIGGERS; count++)
				fctrl->flash_max_current[count] = 0;

			for (count = 0; count < MAX_LED_TRIGGERS; count++)
				fctrl->torch_max_current[count] =
					fctrl->flash_max_current[count] >> 1;
		}

		/* Read the max duration for an LED if present */
		if (of_get_property(of_node, "qcom,max-duration", &count)) {
			count /= sizeof(uint32_t);

			if (count > MAX_LED_TRIGGERS) {
				pr_err("failed\n");
				rc = -EINVAL;
				goto ERROR8;
			}

			rc = of_property_read_u32_array(of_node,
				"qcom,max-duration",
				fctrl->flash_max_duration, count);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR8;
			}

			for (; count < MAX_LED_TRIGGERS; count++)
				fctrl->flash_max_duration[count] = 0;
		}

		flashdata->slave_info =
			kzalloc(sizeof(struct msm_camera_slave_info),
				GFP_KERNEL);
		if (!flashdata->slave_info) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto ERROR8;
		}

		rc = of_property_read_u32_array(of_node, "qcom,slave-id",
			id_info, 3);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR9;
		}
		fctrl->flashdata->slave_info->sensor_slave_addr = id_info[0];
		fctrl->flashdata->slave_info->sensor_id_reg_addr = id_info[1];
		fctrl->flashdata->slave_info->sensor_id = id_info[2];

		kfree(gpio_array);
		return rc;
ERROR9:
		kfree(fctrl->flashdata->slave_info);
ERROR8:
		kfree(fctrl->flashdata->power_info.gpio_conf->gpio_num_info);
ERROR6:
		kfree(gconf->cam_gpio_set_tbl);
ERROR5:
		kfree(gconf->cam_gpio_req_tbl);
ERROR4:
		kfree(gconf);
ERROR1:
		kfree(fctrl->flashdata);
		kfree(gpio_array);
	}
	return rc;
}

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
};

#ifdef CONFIG_DEBUG_FS
static int set_led_status(void *data, u64 val)
{
	struct msm_led_flash_ctrl_t *fctrl =
		 (struct msm_led_flash_ctrl_t *)data;
	int rc = -1;
	pr_debug("set_led_status: Enter val: %llu", val);
	if (!fctrl) {
		pr_err("set_led_status: fctrl is NULL");
		return rc;
	}
	if (!fctrl->func_tbl) {
		pr_err("set_led_status: fctrl->func_tbl is NULL");
		return rc;
	}
	switch(val) {
	case 9:
		pr_debug("set_led_status: val is disable");
		rc = msm_flash_led_off(fctrl);
		if (rc < 0) {
			pr_err("%s led_off failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led_release(fctrl);
		if (rc < 0) {
			pr_err("%s led_release failed line %d\n", __func__, __LINE__);
			return rc;
		}
	break;
	case 1:
		rc = msm_flash_led_init(fctrl);
		if (rc < 0) {
			pr_err("%s led_init failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led0(fctrl);
		if (rc < 0) {
			pr_err("%s led_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case 2:
		rc = msm_flash_led_init(fctrl);
		if (rc < 0) {
			pr_err("%s led_init failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led1(fctrl);
		if (rc < 0) {
			pr_err("%s led_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case 3:
		rc = msm_flash_led_init(fctrl);
		if (rc < 0) {
			pr_err("%s led_init failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led2(fctrl);
		if (rc < 0) {
			pr_err("%s led_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case 4:
		rc = msm_flash_led_init(fctrl);
		if (rc < 0) {
			pr_err("%s led_init failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led0_flash(fctrl);
		if (rc < 0) {
			pr_err("%s led_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case 5:
		rc = msm_flash_led_init(fctrl);
		if (rc < 0) {
			pr_err("%s led_init failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led1_flash(fctrl);
		if (rc < 0) {
			pr_err("%s led_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case 6:
		rc = msm_flash_led_init(fctrl);
		if (rc < 0) {
			pr_err("%s led_init failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led2_flash(fctrl);
		if (rc < 0) {
			pr_err("%s led_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	case 7:
		pr_err("set_led_status: val is enable");
		rc = msm_flash_led_init(fctrl);
		if (rc < 0) {
			pr_err("%s led_init failed line %d\n", __func__, __LINE__);
			return rc;
		}
		rc = msm_flash_led_low(fctrl);
		if (rc < 0) {
			pr_err("%s led_low failed line %d\n", __func__, __LINE__);
			return rc;
		}
		break;
	default:
		//do nothing
		break;
	}

	return rc;
}

DEFINE_SIMPLE_ATTRIBUTE(ledflashdbg_fops,
	NULL, set_led_status, "%llu\n");
#endif

static void msm_led_i2c_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct msm_led_flash_ctrl_t *fctrl = NULL;

	if (g_fctrl == NULL)
		return;

	fctrl = (struct msm_led_flash_ctrl_t *) g_fctrl;

	if (value > LED_OFF) {
		if (fctrl->func_tbl->flash_led_init)
			fctrl->func_tbl->flash_led_init(fctrl);
		if (fctrl->func_tbl->flash_led_low)
			fctrl->func_tbl->flash_led_low(fctrl);
	} else {
		if (fctrl->func_tbl->flash_led_off)
			fctrl->func_tbl->flash_led_off(fctrl);
		if (fctrl->func_tbl->flash_led_release)
			fctrl->func_tbl->flash_led_release(fctrl);
	}
};

static struct led_classdev msm_torch_i2c_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_i2c_torch_brightness_set,
	.brightness		= LED_OFF,
};

static int32_t msm_i2c_torch_create_classdev(struct device *dev ,
				void *data)
{
	int rc;
	msm_led_i2c_torch_brightness_set(&msm_torch_i2c_led, LED_OFF);
	rc = led_classdev_register(dev, &msm_torch_i2c_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};
static int32_t asus_tranlate_flash_data_to_bsp(struct msm_flash_cfg_data_t flash_data, bsp_flash_data *cfg) {
  int32_t rc = 0;
  int i = 0;
  for(i = 0; i < MAX_LED_TRIGGERS; i++) {
    cfg->led_current[i] = flash_data.flash_current[i];
    pr_err("%s led_current[%d] = %d\n", __func__, i, cfg->led_current[i]);
  }
  switch (flash_data.cfg_type) {
    case CFG_FLASH_LOW:
      cfg->cmd = 1;
      break;
    case CFG_FLASH_HIGH:
      cfg->cmd = 4;
      break;
    default:
      pr_err("%s No-op supported here.", __func__);
  }
  return rc;
}
#define	ZENFLASH_PROC_FILE	"driver/asus_flash_trigger_time"
static struct proc_dir_entry *zenflash_proc_file;
static int zenflash_status;
static int zenflash_test_time;


static ssize_t zenflash_proc_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	int rc = 0, len;
  struct msm_led_flash_ctrl_t *flash_ctrl = gBsp_led_flash_ctrl;
	struct msm_flash_cfg_data_t flash_data;
  struct msm_camera_i2c_reg_array i2c_commands[3];
	struct msm_camera_i2c_reg_setting custom_flash_setting;
	int duration = 0, zenflash_current = 0;
	bsp_flash_data cfg;
	len = (count > DBG_TXT_BUF_SIZE-1)? (DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf, buf, len))
			return -EFAULT;
	debugTxtBuf[len] = 0; //add string end
	pr_err("%s %d, count: %d\n", __func__, __LINE__, (int) count);
	sscanf(debugTxtBuf, "%d %d", &duration, &zenflash_current);
	pr_err("%s %d, duration = %d, current = %d\n", __func__, __LINE__, duration, zenflash_current);
	if(flash_ctrl->led_state != MSM_CAMERA_LED_INIT ) {
		rc = msm_flash_led_init(flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d camera_flash_init failed rc = %d",
				__func__, __LINE__, rc);
				rc = msm_flash_led_release(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_release failed rc = %d",
						__func__, __LINE__, rc);
					*ppos += count;
          rc = count;
          return rc;
				}
			*ppos += count;
      rc = count;
      return rc;
		}
	}
	if(duration == ZENFLASH_TESTTIME_RESET_CMD) {
    zenflash_test_time = 0;
    pr_err("%s:%d zenflash test time has been set to 0", __func__, __LINE__);
    *ppos += count;
    rc = count;
    return rc;
	}
	else if(duration > 0 && duration <= MAX_ZENFLASH_DURATION) {
	    if(zenflash_current == 0) {
	      pr_err("%s:%d zenflash_current has not been set, using default %d",
						__func__, __LINE__, ZENFLASH_CURRENT);
        zenflash_current = ZENFLASH_CURRENT;
	    }
	    else if(zenflash_current < ZENFLASH_MIN_CURRENT) {
	      pr_err("%s:%d zenflash_current has been set to %d, which is too low. Use min current %d",
						__func__, __LINE__, zenflash_current, ZENFLASH_MIN_CURRENT);
        zenflash_current = ZENFLASH_MIN_CURRENT;
	    }
      printk(KERN_INFO "[AsusZenFlashDuration] duration now in 1 ~ %d\n", MAX_ZENFLASH_DURATION);
			flash_data.flash_current[0] = (zenflash_current + 6 )/12;
			flash_data.flash_current[1] = 0;
			flash_data.flash_current[2] = 0;
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT) {
        int led_data_cnt = 0;
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d\n",
						__func__, __LINE__, rc);
					*ppos += count;
          rc = count;
          return rc;
				}
				flash_data.cfg_type = CFG_FLASH_HIGH;
				asus_tranlate_flash_data_to_bsp(flash_data, &cfg);
        led_data_cnt = msm_flash_led_data_to_i2c(i2c_commands, cfg, 3);
        if (led_data_cnt > 0) {
          custom_flash_setting.reg_setting = i2c_commands;
          custom_flash_setting.size = led_data_cnt;//ARRAY_SIZE(i2c_commands);
          custom_flash_setting.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
          custom_flash_setting.data_type = MSM_CAMERA_I2C_BYTE_DATA;
          custom_flash_setting.delay = 0;
          rc = msm_flash_write_custom_commands(gBsp_led_flash_ctrl, &custom_flash_setting);
          rc = msm_flash_led_high(flash_ctrl);
          zenflash_status = 1;
          zenflash_test_time++;
          msleep(duration);
          rc = msm_flash_led_off(flash_ctrl);
          zenflash_status = 0;
          if (rc < 0) {
            pr_err("%s:%d camera_flash_high failed rc = %d",
            __func__, __LINE__, rc);
            *ppos += count;
            rc = count;
            return rc;
          }
        }
	    }
	}
	else {
    printk(KERN_INFO "[AsusZenFlashDuration] duration now is %d, invalid value.\n", duration);
    return -EINVAL;
	}
	*ppos += count;
   rc = count;
   return rc;
}

static int zenflash_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d %d\n", zenflash_status, zenflash_test_time);
	return 0;
}

static int zenflash_open(struct inode *inode, struct file *file)
{
	return single_open(file, zenflash_read, NULL);
}

static const struct file_operations zenflash_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= zenflash_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= zenflash_proc_write,
};

#define	FLASH_BRIGHTNESS_PROC_FILE	"driver/asus_flash_brightness"
#define	FLASH_STATUS_PROC_FILE	"driver/flash_status"

static struct proc_dir_entry *flash_brightness_proc_file;
static struct proc_dir_entry *flash_status_proc_file;

static int last_flash_brightness_value;
static int flash_status;

static int msm_flash_brightness_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", last_flash_brightness_value);
    return 0;
}

static int msm_flash_brightness_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, msm_flash_brightness_proc_read, NULL);
}

static ssize_t msm_flash_brightness_proc_write(struct file *filp, const char __user *buf, size_t count, loff_t *ppos)
{
	int set_val = -1, now_flash_brightness_value = -1;
	int MAX_FLASHLIGHT_CURRENT = MAX_TORCH_CURRENT;
	int rc = 0, len, i;
	struct msm_led_flash_ctrl_t *flash_ctrl = gBsp_led_flash_ctrl;
	struct msm_flash_cfg_data_t flash_data;
  struct msm_camera_i2c_reg_array i2c_commands[3];
	struct msm_camera_i2c_reg_setting custom_flash_setting;
	bsp_flash_data cfg;
	len = (count > DBG_TXT_BUF_SIZE-1)? (DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf, buf, len))
			return -EFAULT;
	debugTxtBuf[len] = 0; //add string end
	sscanf(debugTxtBuf, "%d", &now_flash_brightness_value);
	set_val = now_flash_brightness_value * MAX_FLASHLIGHT_CURRENT / 99;
	*ppos=len;
	pr_err("[AsusFlashBrightness]flash brightness value= %d now_flash_brightness_value = %d\n", set_val,now_flash_brightness_value);
	if (last_flash_brightness_value == now_flash_brightness_value
	    || (now_flash_brightness_value < 0 || now_flash_brightness_value > 99)) {
		pr_info("[AsusFlashBrightness] now_flash_brightness_value = last_flash_brightness_value or now_flash_brightness_value out of range, so do nothing\n");
		return len;
	}
	last_flash_brightness_value = now_flash_brightness_value;
	if (now_flash_brightness_value != 0 )
		flash_status = 1;
	for (i = 0; i < MAX_LED_TRIGGERS; i++) {
		flash_data.flash_current[i] = (flash_ctrl->flash_max_current[i] + 6)/12;
	}
	if(flash_ctrl->led_state != MSM_CAMERA_LED_INIT ) {
		rc = msm_flash_led_init(flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d camera_flash_init failed rc = %d",
				__func__, __LINE__, rc);
				rc = msm_flash_led_release(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_release failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
			return rc;
		}
	}
	if (set_val > MAX_FLASHLIGHT_CURRENT) {
		flash_data.flash_current[0] = (MAX_FLASHLIGHT_CURRENT + 6)/12;
		flash_data.flash_current[1] = 0;
		flash_data.flash_current[2] = 0;
		if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT) {
			rc = msm_flash_led_off(flash_ctrl);
			if (rc < 0) {
				pr_err("%s:%d camera_flash_off failed rc = %d",
					__func__, __LINE__, rc);
				return rc;
			}
      flash_data.cfg_type = CFG_FLASH_LOW;
			asus_tranlate_flash_data_to_bsp(flash_data, &cfg);
		  count = msm_flash_led_data_to_i2c(i2c_commands, cfg, 3);

      if (count > 0) {
        custom_flash_setting.reg_setting = i2c_commands;
        custom_flash_setting.size = count;//ARRAY_SIZE(i2c_commands);
        custom_flash_setting.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
        custom_flash_setting.data_type = MSM_CAMERA_I2C_BYTE_DATA;
        custom_flash_setting.delay = 0;
        rc = msm_flash_write_custom_commands(gBsp_led_flash_ctrl, &custom_flash_setting);
        rc = msm_flash_led_low(flash_ctrl);
        if (rc < 0) {
          pr_err("%s:%d camera_flash_low failed rc = %d",
          __func__, __LINE__, rc);
          return rc;
        }
      }
		}
	} else if (set_val <= 0) {
		if(flash_ctrl->led_state== MSM_CAMERA_LED_INIT) {
			rc = msm_flash_led_off(flash_ctrl);
			if (rc < 0) {
				pr_err("%s:%d camera_flash_off failed rc = %d",
					__func__, __LINE__, rc);
				return rc;
			}
			rc = msm_flash_led_release(flash_ctrl);
			if (rc < 0) {
				pr_err("%s:%d camera_flash_release failed rc = %d",
					__func__, __LINE__, rc);
				return rc;
			}
			flash_status = 0;
		}
	} else if (0 < set_val && set_val < (MAX_FLASHLIGHT_CURRENT + 1)) {
		printk(KERN_INFO "[AsusFlashBrightness] current now in 1~%d", MAX_FLASHLIGHT_CURRENT);
			flash_data.flash_current[0] = (set_val + 6)/12;;
			flash_data.flash_current[1] = 0;
			flash_data.flash_current[2] = 0;
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT) {
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				flash_data.cfg_type = CFG_FLASH_LOW;
				asus_tranlate_flash_data_to_bsp(flash_data, &cfg);
        count = msm_flash_led_data_to_i2c(i2c_commands, cfg, 3);
        if (count > 0) {
          custom_flash_setting.reg_setting = i2c_commands;
          custom_flash_setting.size = count;//ARRAY_SIZE(i2c_commands);
          custom_flash_setting.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
          custom_flash_setting.data_type = MSM_CAMERA_I2C_BYTE_DATA;
          custom_flash_setting.delay = 0;
          rc = msm_flash_write_custom_commands(gBsp_led_flash_ctrl, &custom_flash_setting);
          rc = msm_flash_led_low(flash_ctrl);
          if (rc < 0) {
            pr_err("%s:%d camera_flash_low failed rc = %d",
            __func__, __LINE__, rc);
            return rc;
          }
        }
  		}
	} else {
		if(flash_ctrl->led_state== MSM_CAMERA_LED_INIT)
			rc = msm_flash_led_release(flash_ctrl);
		flash_status = 0;
		if (rc < 0) {
			pr_err("%s:%d camera_flash_release failed rc = %d",
				__func__, __LINE__, rc);
			return rc;
		}
		return -1;
	}
	return len;
}

static const struct file_operations flash_brightness_fops = {
	.owner = THIS_MODULE,
	.open = msm_flash_brightness_proc_open,
	.read = seq_read,
	.write = msm_flash_brightness_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

//ASUS_BSP +++ PJ "implement asus_flash control node"
#define	ASUS_FLASH_PROC_FILE	"driver/asus_flash"
static struct proc_dir_entry *asus_flash_proc_file;

static int asus_flash_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", ATD_status);//ASUS_BSP PJ "add flash status"
	ATD_status = 0;//ASUS_BSP PJ "add flash status"
	return 0;
}

static int asus_flash_open(struct inode *inode, struct file *file)
{
	return single_open(file, asus_flash_read, NULL);
}

static ssize_t asus_flash_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int mode = -1, set_val = -1, set_val2 = -1;
	int rc = 0, len, i;
	struct msm_led_flash_ctrl_t *flash_ctrl = gBsp_led_flash_ctrl;
	struct msm_flash_cfg_data_t flash_data;
  struct msm_camera_i2c_reg_array i2c_commands[3];
	struct msm_camera_i2c_reg_setting custom_flash_setting;
	bsp_flash_data cfg;
	len = (count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
			return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%d %d %d", &mode, &set_val, &set_val2);
	*ppos=len;

	pr_info("[AsusFlash]flash mode=%d value=%d value2=%d\n", mode, set_val, set_val2);
	flash_data.ctrl_state = set_val2;
	for (i = 0; i < MAX_LED_TRIGGERS; i++) {
		flash_data.flash_current[i] = flash_ctrl->flash_max_current[i];
		flash_data.flash_duration[i] = MAX_FLASH_DURATION;
	}
	if(flash_ctrl->led_state != MSM_CAMERA_LED_INIT ) {
		rc = msm_flash_led_init(flash_ctrl);
		if (rc < 0) {
			pr_err("%s:%d camera_flash_init failed rc = %d",
				__func__, __LINE__, rc);
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_release failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
			return rc;
		}
	}

	if(mode == 0) {
	  flash_data.cfg_type = CFG_FLASH_LOW;
		if (set_val < 0 || set_val > 200 || set_val == 1) {
			flash_data.flash_current[0] = (120 + 6 )/12;
			flash_data.flash_current[1] = 0;
      flash_data.flash_current[2] = 0;
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT) {
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				asus_tranlate_flash_data_to_bsp(flash_data, &cfg);
				count = msm_flash_led_data_to_i2c(i2c_commands, cfg, 3);

      if (count > 0) {
        custom_flash_setting.reg_setting = i2c_commands;
        custom_flash_setting.size = count;//ARRAY_SIZE(i2c_commands);
        custom_flash_setting.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
        custom_flash_setting.data_type = MSM_CAMERA_I2C_BYTE_DATA;
        custom_flash_setting.delay = 0;
        rc = msm_flash_write_custom_commands(gBsp_led_flash_ctrl, &custom_flash_setting);
        rc = msm_flash_led_low(flash_ctrl);
        if (rc < 0) {
          pr_err("%s:%d camera_flash_low failed rc = %d",
          __func__, __LINE__, rc);
          return rc;
        }
      }
				ATD_status = 1;//ASUS_BSP PJ "add flash status"
			}
		} else if (set_val == 0 ) {
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT) {
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				rc = msm_flash_led_release(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_release failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				ATD_status = 1;//ASUS_BSP PJ "add flash status"
			}
		} else if(0 < set_val && set_val <= 200) {
			printk(KERN_INFO "[AsusFlash] current now in 1~200");
			flash_data.flash_current[0] = (set_val + 6)/12;
			flash_data.flash_current[1] = 0;
      flash_data.flash_current[2] = 0;
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT) {
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
        asus_tranlate_flash_data_to_bsp(flash_data, &cfg);
				count = msm_flash_led_data_to_i2c(i2c_commands, cfg, 3);

        if (count > 0) {
          custom_flash_setting.reg_setting = i2c_commands;
          custom_flash_setting.size = count;//ARRAY_SIZE(i2c_commands);
          custom_flash_setting.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
          custom_flash_setting.data_type = MSM_CAMERA_I2C_BYTE_DATA;
          custom_flash_setting.delay = 0;
          rc = msm_flash_write_custom_commands(gBsp_led_flash_ctrl, &custom_flash_setting);
          rc = msm_flash_led_low(flash_ctrl);
          if (rc < 0) {
            pr_err("%s:%d camera_flash_low failed rc = %d",
            __func__, __LINE__, rc);
            return rc;
          }
        }

				ATD_status = 1;//ASUS_BSP PJ "add flash status"
			}
		} else {
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT)
				rc = msm_flash_led_release(flash_ctrl);
			if (rc < 0) {
				pr_err("%s:%d camera_flash_release failed rc = %d",
					__func__, __LINE__, rc);
				return rc;
			}
			return -1;
		}
	} else if(mode == 1) {
	  flash_data.cfg_type = CFG_FLASH_HIGH;
		if (set_val == 1 || set_val < 0 || set_val > 1000) {
			flash_data.flash_current[0] = (625 + 6)/12;
			flash_data.flash_current[1] = 0;
      flash_data.flash_current[2] = 0;
			if(flash_ctrl->led_state== MSM_CAMERA_LED_INIT) {
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				asus_tranlate_flash_data_to_bsp(flash_data, &cfg);
				count = msm_flash_led_data_to_i2c(i2c_commands, cfg, 3);
        if (count > 0) {
          custom_flash_setting.reg_setting = i2c_commands;
          custom_flash_setting.size = count;//ARRAY_SIZE(i2c_commands);
          custom_flash_setting.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
          custom_flash_setting.data_type = MSM_CAMERA_I2C_BYTE_DATA;
          custom_flash_setting.delay = 0;
          rc = msm_flash_write_custom_commands(gBsp_led_flash_ctrl, &custom_flash_setting);
          rc = msm_flash_led_high(flash_ctrl);
          if (rc < 0) {
            pr_err("%s:%d camera_flash_high failed rc = %d",
            __func__, __LINE__, rc);
            return rc;
          }
        }

				ATD_status = 1;//ASUS_BSP PJ "add flash status"
			}
		} else if (set_val == 0) {
			if(flash_ctrl->led_state== MSM_CAMERA_LED_INIT) {
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				rc = msm_flash_led_release(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_release failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				ATD_status = 1;//ASUS_BSP PJ "add flash status"
			}
		} else if (0 < set_val && set_val <= 1000) {
			printk(KERN_INFO "[AsusFlash] Flash current now in 1~1000");
			flash_data.flash_current[0] = (set_val + 6)/12;
			flash_data.flash_current[1] = 0;
      flash_data.flash_current[2] = 0;
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT) {
				rc = msm_flash_led_off(flash_ctrl);
				if (rc < 0) {
					pr_err("%s:%d camera_flash_off failed rc = %d",
						__func__, __LINE__, rc);
					return rc;
				}
				asus_tranlate_flash_data_to_bsp(flash_data, &cfg);
				count = msm_flash_led_data_to_i2c(i2c_commands, cfg, 3);

        if (count > 0) {
          custom_flash_setting.reg_setting = i2c_commands;
          custom_flash_setting.size = count;//ARRAY_SIZE(i2c_commands);
          custom_flash_setting.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
          custom_flash_setting.data_type = MSM_CAMERA_I2C_BYTE_DATA;
          custom_flash_setting.delay = 0;
          rc = msm_flash_write_custom_commands(gBsp_led_flash_ctrl, &custom_flash_setting);
          rc = msm_flash_led_high(flash_ctrl);
          if (rc < 0) {
            pr_err("%s:%d camera_flash_high failed rc = %d",
            __func__, __LINE__, rc);
            return rc;
          }
        }
				ATD_status = 1;//ASUS_BSP PJ "add flash status"
			}
		} else {
			if(flash_ctrl->led_state == MSM_CAMERA_LED_INIT)
				rc = msm_flash_led_release(flash_ctrl);
			if (rc < 0) {
				pr_err("%s:%d camera_flash_release failed rc = %d",
					__func__, __LINE__, rc);
				return rc;
			}
			return -1;
		}
	} else {
		return -1;
	}
	return len;
}

static const struct file_operations asus_flash_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= asus_flash_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= asus_flash_write,
};

static int flash_status_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", flash_status);
	return 0;
}

static int flash_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, flash_status_read, NULL);
}

static const struct file_operations flash_status_proc_fops = {
	.open		= flash_status_open,
	.read		= seq_read,
};

//ASUS_BSP +++ PJ "create flash control node"
static void create_proc_file(void)
{
    asus_flash_proc_file = proc_create(ASUS_FLASH_PROC_FILE, 0666, NULL, &asus_flash_proc_fops);
    if (asus_flash_proc_file) {
      printk("%s asus_flash_proc_file sucessed!\n", __func__);
    } else {
      printk("%s asus_flash_proc_file failed!\n", __func__);
    }

    flash_brightness_proc_file = proc_create(FLASH_BRIGHTNESS_PROC_FILE, 0666, NULL, &flash_brightness_fops);
    if (flash_brightness_proc_file) {
      printk("%s flash_brightness_proc_file sucessed!\n", __func__);
    } else {
      printk("%s flash_brightness_proc_file failed!\n", __func__);
    }

    flash_status_proc_file = proc_create(FLASH_STATUS_PROC_FILE, 0666, NULL, &flash_status_proc_fops);
    if (flash_status_proc_file) {
      printk("%s flash_status_proc_file sucessed!\n", __func__);
    } else {
      printk("%s flash_status_proc_file failed!\n", __func__);
    }

    zenflash_proc_file = proc_create(ZENFLASH_PROC_FILE, 0666, NULL, &zenflash_proc_fops);
    if (zenflash_proc_file) {
      printk("%s zenflash_proc_file sucessed!\n", __func__);
      zenflash_status = 0;
      zenflash_test_time = 0;
    } else {
      printk("%s zenflash_proc_file failed!\n", __func__);
    }
}
//ASUS_BSP --- PJ "create flash control node"

int msm_flash_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_led_flash_ctrl_t *fctrl = NULL;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	fctrl = (struct msm_led_flash_ctrl_t *)(id->driver_data);
	if (fctrl->flash_i2c_client)
		fctrl->flash_i2c_client->client = client;
	/* Set device type as I2C */
	fctrl->flash_device_type = MSM_CAMERA_I2C_DEVICE;

	/* Assign name for sub device */
	snprintf(fctrl->msm_sd.sd.name, sizeof(fctrl->msm_sd.sd.name),
		"%s", id->name);

	rc = msm_led_get_dt_data(client->dev.of_node, fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true)
		msm_flash_pinctrl_init(fctrl);

	if (fctrl->flash_i2c_client != NULL) {
		fctrl->flash_i2c_client->client = client;
		if (fctrl->flashdata->slave_info->sensor_slave_addr)
			fctrl->flash_i2c_client->client->addr =
				fctrl->flashdata->slave_info->
				sensor_slave_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	if (!fctrl->flash_i2c_client->i2c_func_tbl)
		fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_qup_func_tbl;

	rc = msm_led_i2c_flash_create_v4lsubdev(fctrl);
#ifdef CONFIG_DEBUG_FS
	dentry = debugfs_create_file("ledflash",
			g_ftm_mode ? S_IRUGO | S_IWUGO : S_IRUGO, NULL, (void *)fctrl,
			&ledflashdbg_fops);
	if (!dentry)
		pr_err("Failed to create the debugfs ledflash file");
#endif

	/* Assign Global flash control sturcture for local usage */
	g_fctrl = (void *) fctrl;
	rc = msm_i2c_torch_create_classdev(&(client->dev), NULL);
	if (rc) {
		pr_err("%s failed to create classdev %d\n", __func__, __LINE__);
		return rc;
	}
	gBsp_led_flash_ctrl = g_fctrl;
  create_proc_file();
	CDBG("%s:%d probe success\n", __func__, __LINE__);
	return 0;

probe_failure:
	CDBG("%s:%d probe failed\n", __func__, __LINE__);
	return rc;
}

int msm_flash_probe(struct platform_device *pdev,
	const void *data)
{
	int rc = 0;
	struct msm_led_flash_ctrl_t *fctrl =
		(struct msm_led_flash_ctrl_t *)data;
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_cci_client *cci_client = NULL;

	if (!of_node) {
		pr_err("of_node NULL\n");
		goto probe_failure;
	}
	fctrl->pdev = pdev;

	rc = msm_led_get_dt_data(pdev->dev.of_node, fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true)
		msm_flash_pinctrl_init(fctrl);

	/* Assign name for sub device */
	snprintf(fctrl->msm_sd.sd.name, sizeof(fctrl->msm_sd.sd.name),
			"%s", fctrl->flashdata->sensor_name);
	/* Set device type as Platform*/
	fctrl->flash_device_type = MSM_CAMERA_PLATFORM_DEVICE;

	if (NULL == fctrl->flash_i2c_client) {
		pr_err("%s flash_i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
		goto probe_failure;
	}

	fctrl->flash_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!fctrl->flash_i2c_client->cci_client) {
		pr_err("%s failed line %d kzalloc failed\n",
			__func__, __LINE__);
		return rc;
	}

	cci_client = fctrl->flash_i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = fctrl->cci_i2c_master;
	if (fctrl->flashdata->slave_info->sensor_slave_addr)
		cci_client->sid =
			fctrl->flashdata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;

	if (!fctrl->flash_i2c_client->i2c_func_tbl)
		fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_cci_func_tbl;

	rc = msm_led_flash_create_v4lsubdev(pdev, fctrl);

	CDBG("%s: probe success\n", __func__);
	return 0;

probe_failure:
	CDBG("%s probe failed\n", __func__);
	return rc;
}
