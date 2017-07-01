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
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"
#define FLASH_NAME "qcom,led-flash"

#undef CDBG
#define CDBG(fmt, args...) pr_err(fmt, ##args)

static struct msm_led_flash_ctrl_t fctrl;
static struct i2c_driver sky81298_i2c_driver;

static struct msm_camera_i2c_reg_array sky81298_init_array[] = {
	{0x08, 0x00}, /*disable LED1, 2,3*/
	{0x00, 0x42},
	{0x01, 0x42},
	{0x02, 0x10},
	{0x03, 0xff},
	{0x04, 0x4f},
	{0x05, 0x10},
	{0x06, 0x10},
	{0x07, 0x10},
	{0x09, 0x00},
	{0x0A, 0xA0},
};

static struct msm_camera_i2c_reg_array sky81298_off_array[] = {
	{0x08, 0x00}, /*disable LED1, 2,3*/
};

static struct msm_camera_i2c_reg_array sky81298_release_array[] = {
	{0x08, 0x00}, /*disable LED1, 2,3*/
};

static struct msm_camera_i2c_reg_array sky81298_low_array[] = {
	{0x08, 0x15}, /*enable LED1, 2,3, movie mode*/
};

static struct msm_camera_i2c_reg_array sky81298_high_array[] = {
	{0x08, 0x2A}, /*disable LED1, 2,3, flash mode*/
};

static struct msm_camera_i2c_reg_array sky81298_led0_array[] = {
	{0x08, 0x01}, /*enable LED1, movie mode*/
};
static struct msm_camera_i2c_reg_array sky81298_led1_array[] = {
	{0x08, 0x04}, /*enable LED2, movie mode*/
};
static struct msm_camera_i2c_reg_array sky81298_led01_array[] = {
	{0x08, 0x05}, /*enable LED1, movie mode*/
};
static struct msm_camera_i2c_reg_array sky81298_led2_array[] = {
	{0x08, 0x10}, /*enable LED3, movie mode*/
};
static struct msm_camera_i2c_reg_array sky81298_led0_flash_array[] = {
	{0x08, 0x02}, /*enable LED1, movie mode*/
};
static struct msm_camera_i2c_reg_array sky81298_led1_flash_array[] = {
	{0x08, 0x08}, /*enable LED2, movie mode*/
};
static struct msm_camera_i2c_reg_array sky81298_led01_flash_array[] = {
	{0x08, 0x0A}, /*enable LED2, movie mode*/
};
static struct msm_camera_i2c_reg_array sky81298_led2_flash_array[] = {
	{0x08, 0x20}, /*enable LED3, movie mode*/
};

static void __exit msm_flash_sky81298_i2c_remove(void)
{
	i2c_del_driver(&sky81298_i2c_driver);
	return;
}

static const struct of_device_id sky81298_trigger_dt_match[] = {
	{.compatible = "qcom,led-flash", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, sky81298_trigger_dt_match);

static const struct i2c_device_id flash_i2c_id[] = {
	{"qcom,led-flash", (kernel_ulong_t)&fctrl},
	{ }
};

static const struct i2c_device_id sky81298_i2c_id[] = {
	{FLASH_NAME, (kernel_ulong_t)&fctrl},
	{ }
};

static int msm_flash_sky81298_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	CDBG("%s: E\n", __func__);
	if (!id) {
		pr_err("msm_flash_sky81298_i2c_probe: id is NULL");
		id = sky81298_i2c_id;
	}
	ret = msm_flash_i2c_probe(client, id);
	CDBG("%s: X\n", __func__);
	return ret;
}

static struct i2c_driver sky81298_i2c_driver = {
	.id_table = sky81298_i2c_id,
	.probe  = msm_flash_sky81298_i2c_probe,
	.remove = __exit_p(msm_flash_sky81298_i2c_remove),
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = sky81298_trigger_dt_match,
	},
};

static int msm_flash_sky81298_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	match = of_match_device(sky81298_trigger_dt_match, &pdev->dev);
	if (!match)
		return -EFAULT;
	return msm_flash_probe(pdev, match->data);
}

static struct platform_driver sky81298_platform_driver = {
	.probe = msm_flash_sky81298_platform_probe,
	.driver = {
		.name = "qcom,led-flash",
		.owner = THIS_MODULE,
		.of_match_table = sky81298_trigger_dt_match,
	},
};

static int __init msm_flash_sky81298_init_module(void)
{
	int32_t rc = 0;
	CDBG("%s: E\n", __func__);
	rc = platform_driver_register(&sky81298_platform_driver);
	if (fctrl.pdev != NULL && rc == 0) {
		pr_err("sky81298 platform_driver_register success");
		return rc;
	} else if (rc != 0) {
		pr_err("sky81298 platform_driver_register failed");
		return rc;
	} else {
		rc = i2c_add_driver(&sky81298_i2c_driver);
		if (!rc)
			pr_err("sky81298 i2c_add_driver success");
	}
	CDBG("%s: X\n", __func__);
	return rc;
}

static void __exit msm_flash_sky81298_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&sky81298_platform_driver);
	else
		i2c_del_driver(&sky81298_i2c_driver);
}

static struct msm_camera_i2c_client sky81298_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static struct msm_camera_i2c_reg_setting sky81298_init_setting = {
	.reg_setting = sky81298_init_array,
	.size = ARRAY_SIZE(sky81298_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81298_off_setting = {
	.reg_setting = sky81298_off_array,
	.size = ARRAY_SIZE(sky81298_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81298_release_setting = {
	.reg_setting = sky81298_release_array,
	.size = ARRAY_SIZE(sky81298_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81298_low_setting = {
	.reg_setting = sky81298_low_array,
	.size = ARRAY_SIZE(sky81298_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81298_high_setting = {
	.reg_setting = sky81298_high_array,
	.size = ARRAY_SIZE(sky81298_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting sky81298_led0_setting = {
	.reg_setting = sky81298_led0_array,
	.size = ARRAY_SIZE(sky81298_led0_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting sky81298_led1_setting = {
	.reg_setting = sky81298_led1_array,
	.size = ARRAY_SIZE(sky81298_led1_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting sky81298_led01_setting = {
	.reg_setting = sky81298_led01_array,
	.size = ARRAY_SIZE(sky81298_led01_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting sky81298_led2_setting = {
	.reg_setting = sky81298_led2_array,
	.size = ARRAY_SIZE(sky81298_led2_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting sky81298_led0_flash_setting = {
	.reg_setting = sky81298_led0_flash_array,
	.size = ARRAY_SIZE(sky81298_led0_flash_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting sky81298_led1_flash_setting = {
	.reg_setting = sky81298_led1_flash_array,
	.size = ARRAY_SIZE(sky81298_led1_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting sky81298_led01_flash_setting = {
	.reg_setting = sky81298_led01_flash_array,
	.size = ARRAY_SIZE(sky81298_led01_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_camera_i2c_reg_setting sky81298_led2_flash_setting = {
	.reg_setting = sky81298_led2_flash_array,
	.size = ARRAY_SIZE(sky81298_led2_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};
static struct msm_led_flash_reg_t sky81298_regs = {
	.init_setting = &sky81298_init_setting,
	.off_setting = &sky81298_off_setting,
	.low_setting = &sky81298_low_setting,
	.high_setting = &sky81298_high_setting,
	.release_setting = &sky81298_release_setting,
	.led0_setting = &sky81298_led0_setting,
	.led1_setting = &sky81298_led1_setting,
	.led01_setting = &sky81298_led01_setting,
	.led2_setting = &sky81298_led2_setting,
	.led0_flash_setting = &sky81298_led0_flash_setting,
	.led1_flash_setting = &sky81298_led1_flash_setting,
	.led01_flash_setting = &sky81298_led01_flash_setting,
	.led2_flash_setting = &sky81298_led2_flash_setting,
};
const int msm_flash_led_data_to_i2c(struct msm_camera_i2c_reg_array * const dest_array,
		const bsp_flash_data config, const unsigned int size)
{
	int count = 0;
	int i;
	switch(config.cmd)
	{
	case 1:
		if (size < 3)
			return 0;
		count = 3;
		for (i = 0; i < 3; i++) {
			dest_array[i].reg_addr = 0x05 + i;
			dest_array[i].reg_data = config.led_current[i] & 0x1f;
			pr_err("%s dest_array[%d].reg_addr = %x, dest_array[%d].reg_data = %x",
			  __func__, i, dest_array[i].reg_addr, i, dest_array[i].reg_data);
		}
		break;
	case 4:
		if (size < 3)
			return 0;
		count = 3;
		for (i = 0; i < 3; i++) {
			dest_array[i].reg_addr = 0x00 + i;
			dest_array[i].reg_data = config.led_current[i] & 0x7f;
			pr_err("%s dest_array[%d].reg_addr = %x, dest_array[%d].reg_data = %x",
			  __func__, i, dest_array[i].reg_addr, i, dest_array[i].reg_data);
		}
		break;
	}
	//pr_err("command size = %d\n", count);
	return count;

}
static struct msm_flash_fn_t sky81298_func_tbl = {
	.flash_get_subdev_id = msm_led_i2c_trigger_get_subdev_id,
	.flash_led_config = msm_led_i2c_trigger_config,
	.flash_led_init = msm_flash_led_init,
	.flash_led_release = msm_flash_led_release,
	.flash_led_off = msm_flash_led_off,
	.flash_led_low = msm_flash_led_low,
	.flash_led_high = msm_flash_led_high,
	.flash_led0 = msm_flash_led0,
	.flash_led1 = msm_flash_led1,
	.flash_led01 = msm_flash_led01,
	.flash_led2 = msm_flash_led2,
	.flash_led0_flash = msm_flash_led0_flash,
	.flash_led1_flash = msm_flash_led1_flash,
	.flash_led01_flash = msm_flash_led01_flash,
	.flash_led2_flash = msm_flash_led2_flash,
	.custom_flash_command = msm_flash_led_data_to_i2c,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.flash_i2c_client = &sky81298_i2c_client,
	.reg_setting = &sky81298_regs,
	.func_tbl = &sky81298_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_sky81298_init_module);
module_exit(msm_flash_sky81298_exit_module);
MODULE_DESCRIPTION("sky81298 FLASH");
MODULE_LICENSE("GPL v2");
