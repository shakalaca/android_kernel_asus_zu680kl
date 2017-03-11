/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
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
#include "anx_ohio_driver.h"
#include "anx_ohio_private_interface.h"
#include "anx_ohio_public_interface.h"
#include "anx_ohio_update.h"
#include <linux/power_supply.h>
#include <linux/usb/class-dual-role.h>
/* Use device tree structure data when defined "CONFIG_OF"  */
/* #define CONFIG_OF */

static int create_sysfs_interfaces(struct device *dev);
extern unsigned char OCM_CODE_AB[];
extern unsigned char OCM_CODE_AD[];
unsigned char *OCM_CODE;
extern int asus_otg_boost_enable(int, bool);
extern int g_reverse_charger_mode;
static unsigned char confirmed_cable_det(void *data);
/* to access global platform data */
#ifdef PD_CHARGING_DRIVER_SUPPORT
struct work_struct pdwork;
extern void pd_send_rdo_resutl(void);
#endif
static struct ohio_platform_data *g_pdata;
static int fw_ver;
#define DONGLE_CABLE_INSERT  1
#define USB_ID  0x12
struct i2c_client *ohio_client;
struct ohio_data *g_ohio;
struct power_supply *usb_psy;
extern bool msm_otg_id_state(void);
extern unsigned char downstream_pd_cap;
struct completion rdo_completion;
struct completion prswap_completion;
struct ohio_platform_data {
	int gpio_p_on;
	int gpio_reset;
	int gpio_cbl_det;
#ifdef SUP_OHIO_INT_VECTOR
	int gpio_intr_comm;
#endif
#ifdef SUP_VBUS_CTL
	int gpio_vbus_ctrl;	
#endif
	spinlock_t lock;
	int boost;
};
struct ohio_data {
	struct ohio_platform_data *pdata;
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock ohio_lock;
	int role_dfp;
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
};

/* ohio power status, sync with interface and cable detection thread */

inline unsigned char OhioReadReg(unsigned char RegAddr)
{
	int ret = 0;

	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_byte_data(ohio_client, RegAddr);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
	return (uint8_t) ret;

}

inline int OhioReadBlockReg(u8 RegAddr, u8 len, u8 *dat)
{
	int ret = 0;

	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_read_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}


inline int OhioWriteBlockReg(u8 RegAddr, u8 len, const u8 *dat)
{
	int ret = 0;

	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_i2c_block_data(ohio_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
		return -EPERM;
	}

	return (int)ret;
}

inline void OhioWriteReg(unsigned char RegAddr, unsigned char RegVal)
{
	int ret = 0;
	//ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);
	ret = i2c_smbus_write_byte_data(ohio_client, RegAddr, RegVal);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
		       __func__, OHIO_SLAVE_I2C_ADDR);
	}
}

void ohio_set_power_supply(struct power_supply *psy)
{
	if (psy) {
		usb_psy = psy;
		pr_info("%s : Set power supply\n", LOG_TAG);
		return;

	}
	pr_err("%s : Set power supply failed\n", LOG_TAG);
}
EXPORT_SYMBOL(ohio_set_power_supply);

int ohio_get_id_state(void)
{
	int usb_id = 1;

	if (atomic_read(&ohio_power_status) == 1) {
		usb_id = !((OhioReadReg(0x3f) & USB_ID) == 0x02);

		/*msm_otg driver will call at state machien initiate
	 	* turn on vbus by sm_init */
		if (!usb_id)
			asus_otg_boost_enable(1, (bool)g_reverse_charger_mode);
		else if (g_reverse_charger_mode)
			try_source();
	}

	pr_info("%s %s : role is %s\n", LOG_TAG, __func__, usb_id ? "UFP" : "DFP");

	return usb_id;

}
EXPORT_SYMBOL(ohio_get_id_state);

void ohio_power_standby(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	mdelay(1);
	gpio_set_value(pdata->gpio_p_on, 0);
	mdelay(1);

	pr_info("%s %s: Ohio power down\n", LOG_TAG, __func__);
}

void ohio_hardware_poweron(void)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif
	int retry_count, i;

	pr_info("%s %s: Ohio power on\n", LOG_TAG, __func__);

	for (retry_count = 0; retry_count < 3; retry_count++) {
		#ifdef OHIO_DEBUG
		pr_info("%s %s: Ohio check ocm loading...\n", LOG_TAG, __func__);
		#endif
		/*power on pin enable */
		gpio_set_value(pdata->gpio_p_on, 1);
		mdelay(10);

		/*power reset pin enable */
		gpio_set_value(pdata->gpio_reset, 1);
		mdelay(1);

		/* load delay T3 : eeprom 3.2s,  OTP 20ms*/
		for (i = 0; i < OHIO_OCM_LOADING_TIME; i++) {
			/*Interface work? */
			if (OhioReadReg(0x16) == 0x80) {
				#ifdef OHIO_DEBUG
				pr_info("%s %s: interface initialization\n", LOG_TAG, __func__);
				#endif
				
				chip_register_init();
				interface_init();
				send_initialized_setting();
				if (OhioReadReg(0x7F) == 0x01) 
					pr_info("%s %s: OTP chip is power on! firmware version is 0x%x\n", LOG_TAG, __func__, OhioReadReg(0x44));
				else 
					pr_info("%s %s: EEPROM chip is power on! firmware version is 0x%x\n", LOG_TAG, __func__, OhioReadReg(0x44));
				fw_ver = OhioReadReg(0x44);
				return;
			}
			mdelay(1);
			/*check cable det state 4 times again avoid system block by wrong cable interrupt*/
			if ((i+1) % 4 == 0) {
				if (confirmed_cable_det((void *)g_ohio) == 0) {
					pr_err("ohio : wrong cable detect\n");
					ohio_power_standby();
					return;
				}
			}
			printk(".");
		}
		ohio_power_standby();
		mdelay(10);
	}

}

void ohio_vbus_control(bool value)
{
#ifdef SUP_VBUS_CTL

#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

	
	if(value)
		gpio_set_value(pdata->gpio_vbus_ctrl, 1);
	else
		gpio_set_value(pdata->gpio_vbus_ctrl, 0);
	
#endif
	
}

void ohio_chip_enable(bool value)
{
#ifdef CONFIG_OF
	struct ohio_platform_data *pdata = g_pdata;
#else
	struct ohio_platform_data *pdata = ohio_client->dev.platform_data;
#endif

	
	if(value) {
		/*power on pin enable */
		gpio_set_value(pdata->gpio_p_on, 1);
		mdelay(10);

		/*power reset pin enable */
		gpio_set_value(pdata->gpio_reset, 1);
		mdelay(1);
	}
	else {
		gpio_set_value(pdata->gpio_reset, 0);
		mdelay(1);
		gpio_set_value(pdata->gpio_p_on, 0);
		mdelay(1);
	}

}

static void ohio_free_gpio(struct ohio_data *ohio)
{
	gpio_free(ohio->pdata->gpio_cbl_det);
	gpio_free(ohio->pdata->gpio_reset);
	gpio_free(ohio->pdata->gpio_p_on);
#ifdef SUP_OHIO_INT_VECTOR
	gpio_free(ohio->pdata->gpio_intr_comm);
#endif
#ifdef SUP_VBUS_CTL
	gpio_free(ohio->pdata->gpio_vbus_ctrl);
#endif
}

static int ohio_init_gpio(struct ohio_data *ohio)
{
	int ret = 0;

	pr_info("%s %s: ohio init gpio\n", LOG_TAG, __func__);
	/*  gpio for chip power down  */
	ret = gpio_request(ohio->pdata->boost, "ohio_boost");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->boost);
		goto err0;
	}
	gpio_direction_output(ohio->pdata->boost, 1);
	/*  gpio for chip power down  */
	ret = gpio_request(ohio->pdata->gpio_p_on, "ohio_p_on_ctl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_p_on);
		goto err0;
	}
	gpio_direction_output(ohio->pdata->gpio_p_on, 0);
	/*  gpio for chip reset  */
	ret = gpio_request(ohio->pdata->gpio_reset, "ohio_reset_n");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(ohio->pdata->gpio_reset, 0);

	/*  gpio for ohio cable detect  */
	ret = gpio_request(ohio->pdata->gpio_cbl_det, "ohio_cbl_det");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(ohio->pdata->gpio_cbl_det);

	#ifdef SUP_OHIO_INT_VECTOR
	/*  gpio for chip interface communaction */
	ret = gpio_request(ohio->pdata->gpio_intr_comm, "ohio_intr_comm");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_intr_comm);
		goto err3;
	}
	gpio_direction_input(ohio->pdata->gpio_intr_comm);
	#endif
	
	#ifdef SUP_VBUS_CTL
	/*  gpio for vbus control  */
	ret = gpio_request(ohio->pdata->gpio_vbus_ctrl, "ohio_vbus_ctrl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
		       ohio->pdata->gpio_vbus_ctrl);
		goto err4;
	}
	gpio_direction_output(ohio->pdata->gpio_vbus_ctrl, 0);
	#endif

	goto out;

#ifdef SUP_VBUS_CTL	
err4:
	gpio_free(ohio->pdata->gpio_vbus_ctrl);
#endif
#ifdef SUP_OHIO_INT_VECTOR
err3:
	gpio_free(ohio->pdata->gpio_intr_comm);
#endif
err2:
	gpio_free(ohio->pdata->gpio_cbl_det);
err1:
	gpio_free(ohio->pdata->gpio_reset);
err0:
	gpio_free(ohio->pdata->gpio_p_on);

	return 1;
out:
	return 0;
}

void ohio_main_process(void)
{
	/* do main loop, do what you want to do */
}

#ifdef CABLE_DET_PIN_HAS_GLITCH
static unsigned char confirmed_cable_det(void *data)
{
	struct ohio_data *anxohio = data;
	unsigned int count = 9;
	unsigned int cable_det_count = 0;
	u8 val = 0;
	
	do {
		val = gpio_get_value(anxohio->pdata->gpio_cbl_det);
		if (DONGLE_CABLE_INSERT == val) 
			cable_det_count++;		 
		mdelay(1);;
	} while (count--);

	if (cable_det_count > 7)
		return 1;
	else if (cable_det_count < 3)
		return 0;
	else 
		return atomic_read(&ohio_power_status);
}
#endif

static irqreturn_t ohio_cbl_det_isr(int irq, void *data)
{
	struct ohio_data *ohio = data;
	int cable_connected = 0;
	
	#ifdef CABLE_DET_PIN_HAS_GLITCH
	cable_connected = confirmed_cable_det((void *)ohio);
	#else
	cable_connected = gpio_get_value(ohio->pdata->gpio_cbl_det);
	#endif


	#ifdef OHIO_DEBUG
	pr_info("%s %s : cable plug pin status %d\n", LOG_TAG, __func__, cable_connected);
	#endif
	
	if (cable_connected == DONGLE_CABLE_INSERT) {
		if (atomic_read(&ohio_power_status) == 1) {
			#ifdef CABLE_DET_PIN_HAS_GLITCH
			mdelay(2); 
			ohio_power_standby();
			#else
			return IRQ_HANDLED;
			#endif
		}
		atomic_set(&ohio_power_status, 1);
		ohio_hardware_poweron();
	} else {
		atomic_set(&ohio_power_status, 0);
		asus_otg_boost_enable(0, (bool)g_reverse_charger_mode);
		if(!msm_otg_id_state())
			power_supply_set_usb_otg(usb_psy, 0);
		#ifdef SUP_VBUS_CTL
		gpio_set_value(ohio->pdata->gpio_vbus_ctrl, 0);
		#endif
		ohio_power_standby();
	}

	if (ohio->dual_role)
		dual_role_instance_changed(ohio->dual_role);

	return IRQ_HANDLED;
}

#ifdef SUP_OHIO_INT_VECTOR
static irqreturn_t ohio_intr_comm_isr(int irq, void *data)
{
	if (atomic_read(&ohio_power_status) != 1)
		return IRQ_NONE;

	if (is_soft_reset_intr()) { 
#ifdef OHIO_DEBUG_RX
		pr_info("%s %s : ======I=====\n", LOG_TAG, __func__);
#endif

	handle_intr_vector();
	}
	return IRQ_HANDLED;
}
#endif

#if 0
static void ohio_work_func(struct work_struct *work)
{
	struct ohio_data *td = container_of(work, struct ohio_data,
					    work.work);
	int workqueu_timer = 0;
	workqueu_timer = 1;
	mutex_lock(&td->lock);
	ohio_main_process();
	mutex_unlock(&td->lock);
	queue_delayed_work(td->workqueue, &td->work,
			   msecs_to_jiffies(workqueu_timer));
}
#endif

#ifdef PD_CHARGING_DRIVER_SUPPORT
static void ohio_pd_work_func(struct work_struct *work)
{
	u32 timeout;
	pr_info("wait for PD request response\n");
	INIT_COMPLETION(rdo_completion);
	timeout =wait_for_completion_timeout(&rdo_completion, msecs_to_jiffies(5000));
	if (!timeout) {
		pr_err("PD response time out\n");
		return;
	}
	pd_send_rdo_resutl();

}
#endif

#ifdef CONFIG_OF
static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->gpio_p_on =
	    of_get_named_gpio_flags(np, "analogix,p-on-gpio", 0, NULL);

	pdata->gpio_reset =
	    of_get_named_gpio_flags(np, "analogix,reset-gpio", 0, NULL);

	pdata->gpio_cbl_det =
	    of_get_named_gpio_flags(np, "analogix,cbl-det-gpio", 0, NULL);

#ifdef SUP_VBUS_CTL
	pdata->gpio_vbus_ctrl =
	    of_get_named_gpio_flags(np, "analogix,vbus-ctrl-gpio", 0, NULL);
#endif
#ifdef SUP_OHIO_INT_VECTOR
	pdata->gpio_intr_comm =
	    of_get_named_gpio_flags(np, "analogix,intr-comm-gpio", 0, NULL);
#endif
	pdata->boost =
	    of_get_named_gpio_flags(np, "analogix,boost", 0, NULL);

	pr_info("%s gpio p_on : %d, reset : %d,  gpio_cbl_det %d\n",
		LOG_TAG, pdata->gpio_p_on,
		pdata->gpio_reset, pdata->gpio_cbl_det);

	return 0;
}
#else
static int ohio_parse_dt(struct device *dev, struct ohio_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifdef CONFIG_DUAL_ROLE_USB_INTF
static enum dual_role_property ohio_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};

static int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

/* Callback for "cat /sys/class/dual_role_usb/otg_default/<property>" */
static int dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
				    enum dual_role_property prop,
				    unsigned int *val)
{
	struct ohio_data *ohio = dual_role_get_drvdata(dual_role);
	int mode = 0;
	
	if (!ohio)
		return -EINVAL;

	if (atomic_read(&ohio_power_status) == 0)
		mode = -1;
	else
		mode = get_power_role();

	if (mode == 1) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (mode == 0) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

static int dual_role_set_mode_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct ohio_data *ohio = dual_role_get_drvdata(dual_role);
	int ret = 0;
	int mode = 0;
	u32 timeout;

	if (!ohio)
		return -EINVAL;

	if (*val != DUAL_ROLE_PROP_MODE_DFP && *val != DUAL_ROLE_PROP_MODE_UFP)
		return -EINVAL;

	if (atomic_read(&ohio_power_status) == 0)
		return 0;

	mode = get_power_role();

	if (*val == DUAL_ROLE_PROP_MODE_DFP && mode == 1)
		return 0;

	if (*val == DUAL_ROLE_PROP_MODE_UFP && mode == 0)
		return 0;

	printk("%s: start %s PD command\n", __func__, downstream_pd_cap ? "with":"without");

	if (mode == 0) {
		pr_err("%s: try reversing, form Sink to Source\n", __func__);
		ret = try_source();
		if (!ret) {
			if(!downstream_pd_cap) {
				pr_err("success power role is %s\n", get_power_role() ? "Source" : "Sink");
			} else {
				INIT_COMPLETION(prswap_completion);
				timeout =wait_for_completion_timeout(&prswap_completion, msecs_to_jiffies(1000));
				if (!timeout)
					pr_warn("power swap timeout\n");
				msleep(1000);
				if(get_data_role() == 0) {
					pr_info("data role is ufp\n");
					interface_dr_swap();
				}
			}
		} else {
			pr_err("failed power role is %s\n", get_power_role() ? "Source" : "Sink");
			ret = -EIO;
		}


	} else if (mode == 1) {
		pr_err("%s: try reversing, form Source to Sink\n", __func__);
		ret = try_sink();
		if (!ret) {
			if(!downstream_pd_cap)
				pr_err("success power role is %s\n", get_power_role() ? "Source" : "Sink");
			else {
				INIT_COMPLETION(prswap_completion);
				timeout =wait_for_completion_timeout(&prswap_completion, msecs_to_jiffies(1000));
				if (!timeout)
					pr_warn("power swap timeout\n");
				msleep(1000);
				if(get_data_role() == 1) {
					pr_info("data role is dfp\n");
					interface_dr_swap();
				}

			}
		} else {
			if(!downstream_pd_cap) {
				asus_otg_boost_enable(1, (bool)g_reverse_charger_mode);
				pr_err("failed power role is %s\n", get_power_role() ? "Source" : "Sink");
				ret = -EIO;
			}
		}
			
	} else {
		pr_err("%s: get role failed\n", __func__);	
		ret = -EIO;
	}

	printk("%s: end ret = %d\n", __func__, ret);

	return ret;
}

static int dual_role_set_pr_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	return 0;
}

static int dual_role_set_dr_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	return 0;
}

static int dual_role_set_vconn_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	return 0;
}

/* Callback for "echo <value> >
 *                      /sys/class/dual_role_usb/<name>/<property>"
 * Block until the entire final state is reached.
 * Blocking is one of the better ways to signal when the operation
 * is done.
 * This function tries to switch to Attached.SRC or Attached.SNK
 * by forcing the mode into SRC or SNK.
 * On failure, we fall back to Try.SNK state machine.
 */
static int dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop,
			      const unsigned int *val)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return dual_role_set_mode_prop(dual_role, prop, val);
	if (prop == DUAL_ROLE_PROP_PR)
		return dual_role_set_pr_prop(dual_role, prop, val);
	if (prop == DUAL_ROLE_PROP_DR)
		return dual_role_set_dr_prop(dual_role, prop, val);
	if (prop == DUAL_ROLE_PROP_VCONN_SUPPLY)
		return dual_role_set_vconn_prop(dual_role, prop, val);
	else
		return -EINVAL;
}

#endif

static int ohio_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{

	struct ohio_data *ohio;
	struct ohio_platform_data *pdata;
	struct dual_role_phy_desc *desc;
	struct dual_role_phy_instance *dual_role;
	int ret = 0;
	int cbl_det_irq = 0;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s:ohio's i2c bus doesn't support\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	ohio = kzalloc(sizeof(struct ohio_data), GFP_KERNEL);
	if (!ohio) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				     sizeof(struct ohio_platform_data),
				     GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;

		/* device tree parsing function call */
		ret = ohio_parse_dt(&client->dev, pdata);
		if (ret != 0)	/* if occurs error */
			goto err0;

		ohio->pdata = pdata;
	} else {
		ohio->pdata = client->dev.platform_data;
	}

	/* to access global platform data */
	g_pdata = ohio->pdata;
	ohio_client = client;
	ohio_client->addr = (OHIO_SLAVE_I2C_ADDR >> 1);

	atomic_set(&ohio_power_status, 0);

	mutex_init(&ohio->lock);

	if (!ohio->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	ret = ohio_init_gpio(ohio);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

	#if 0
	INIT_DELAYED_WORK(&ohio->work, ohio_work_func);
	#endif

#ifdef PD_CHARGING_DRIVER_SUPPORT
	INIT_WORK(&pdwork, ohio_pd_work_func);
#endif
	ohio->workqueue = create_singlethread_workqueue("ohio_work");
	if (ohio->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	ohio_cbl_det_isr(0,ohio);

	cbl_det_irq = gpio_to_irq(ohio->pdata->gpio_cbl_det);
	if (cbl_det_irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		goto err1;
	}

	wake_lock_init(&ohio->ohio_lock, WAKE_LOCK_SUSPEND, "ohio_wake_lock");

		ret = request_threaded_irq(cbl_det_irq, NULL, ohio_cbl_det_isr,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
					| IRQF_ONESHOT, "ohio-cbl-det", ohio);
		if (ret < 0) {
			pr_err("%s : failed to request irq\n", __func__);
			goto err3;
		}

	ret = irq_set_irq_wake(cbl_det_irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		goto err4;
	}

	ret = enable_irq_wake(cbl_det_irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		goto err4;
	}
#ifdef SUP_OHIO_INT_VECTOR
	client->irq = gpio_to_irq(ohio->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get ohio gpio comm irq\n", __func__);
		goto err3;
	}
	init_completion(&rdo_completion);
	init_completion(&prswap_completion);

	ret = request_threaded_irq(client->irq, NULL, ohio_intr_comm_isr,
				   IRQF_TRIGGER_RISING  | IRQF_ONESHOT, "ohio-intr-comm", ohio);
	
	if (ret < 0) {
		pr_err("%s : failed to request interface irq\n", __func__);
		goto err4;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for interface communaction", __func__);
		goto err4;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for interface communaction", __func__);
		goto err4;
	}
#endif
	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
		goto err4;
	}

#ifdef CONFIG_DUAL_ROLE_USB_INTF
	desc = devm_kzalloc(&client->dev, sizeof(struct dual_role_phy_desc),GFP_KERNEL);
	if (!desc) {
		pr_err("%s : alloctate dul_role_phy_desc failed", __func__);
		goto err4;
	}

	desc->name = "otg_default";
	desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
	desc->set_property = dual_role_set_prop;
	desc->get_property = dual_role_get_local_prop;
	desc->num_properties = ARRAY_SIZE(ohio_drp_properties);
	desc->properties = ohio_drp_properties;
	desc->property_is_writeable = dual_role_is_writeable;
	dual_role = devm_dual_role_instance_register(&client->dev, desc);
	dual_role->drv_data = ohio;
	ohio->dual_role = dual_role;
	ohio->desc = desc;
#endif
	g_ohio = ohio;
	/*when probe ohio device, enter standy mode */
	if (atomic_read(&ohio_power_status) == 0)
		ohio_power_standby();

	pr_info("ohio_i2c_probe successfully %s %s end\n", LOG_TAG, __func__);
	goto exit;

err4:
	free_irq(client->irq, ohio);
err3:
	free_irq(cbl_det_irq, ohio);
err1:
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
err0:
	ohio_client = NULL;
	kfree(ohio);
exit:
	return ret;
}

static void ohio_shutdown(struct i2c_client *client)
{
	pr_info("ohio shutdown\n");
	if (g_reverse_charger_mode)
		asus_otg_boost_enable(0,true);
	else
		asus_otg_boost_enable(0,false);
}


static int ohio_i2c_remove(struct i2c_client *client)
{
	struct ohio_data *ohio = i2c_get_clientdata(client);
	printk("ohio_i2c_remove\n");
	free_irq(client->irq, ohio);
	ohio_free_gpio(ohio);
	destroy_workqueue(ohio->workqueue);
	wake_lock_destroy(&ohio->ohio_lock);
	kfree(ohio);
	return 0;
}

static int ohio_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int ohio_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id ohio_id[] = {
	{"ohio", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ohio_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,ohio",},
	{},
};
#endif

static struct i2c_driver ohio_driver = {
	.driver = {
		   .name = "ohio",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = anx_match_table,
#endif
		   },
	.probe = ohio_i2c_probe,
	.remove = ohio_i2c_remove,
	.suspend = ohio_i2c_suspend,
	.resume = ohio_i2c_resume,
	.shutdown = ohio_shutdown,
	.id_table = ohio_id,
};

static void __init ohio_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&ohio_driver);
	if (ret < 0)
		pr_err("%s: failed to register ohio i2c drivern", __func__);
}

static int __init ohio_init(void)
{
	async_schedule(ohio_init_async, NULL);
	return 0;
}

static void __exit ohio_exit(void)
{
	i2c_del_driver(&ohio_driver);
}

#ifdef OHIO_DEBUG
void dump_reg(void)
{
	int i = 0;
	u8 val = 0;

	printk("dump registerad:\n");
	printk("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (i = 0; i < 256; i++) {
		val = OhioReadReg(i);

		if ((i) % 0x10 == 0x00)
			printk("\n[%x]:%02x ", i, val);
		else
			printk("%02x ", val);

	}
	printk("\n");
}

ssize_t anx_ohio_send_pd_cmd(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	switch (cmd) {
	case TYPE_PWR_SRC_CAP:
		send_pd_msg(TYPE_PWR_SRC_CAP, 0, 0);
		break;

	case TYPE_DP_SNK_IDENTITY:
		send_pd_msg(TYPE_DP_SNK_IDENTITY, 0, 0);
		break;

	case TYPE_PSWAP_REQ:
		send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
		break;
	case TYPE_DSWAP_REQ:
		send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
		break;

	case TYPE_GOTO_MIN_REQ:
		send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
		break;

	case TYPE_PWR_OBJ_REQ:
		interface_send_request();
		break;
	case TYPE_ACCEPT:
		interface_send_accept();
		break;
	case TYPE_REJECT:
		interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		send_pd_msg(TYPE_SOFT_RST, 0, 0);
		break;
	case TYPE_HARD_RST:
		send_pd_msg(TYPE_HARD_RST, 0, 0);
		break;
		
	case 0xFD:
		pr_info("fetch powerrole: %d\n", get_power_role());
		break;
	case 0xFE:
		pr_info("fetch datarole: %d\n", get_data_role());
		break;

	case 0xff:
		dump_reg();
		break;
	}
	return count;
}


ssize_t anx_ohio_send_pswap(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_power_swap());
}

ssize_t anx_ohio_send_dswap(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_data_swap());
}

ssize_t anx_ohio_try_source(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_source());
}

ssize_t anx_ohio_try_sink(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_sink());
}

ssize_t anx_ohio_get_data_role(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_data_role());
}

ssize_t anx_ohio_get_power_role(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_power_role());
}

ssize_t anx_ohio_update_otp(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t count)
{
	int fwver;

	fwver = OhioReadReg(0x44);

	if(fwver == 0x11)
		OCM_CODE = OCM_CODE_AB;// fw 1.2
	else if (fwver == 0xd0 || fwver == 0x20)
		OCM_CODE = OCM_CODE_AD;// fw 2.1
	else
		return count;

	update_otp();

	return count;
}

ssize_t anx_ohio_verify_otp(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t count)
{
	verify_otp();
	return count;
}

ssize_t anx_ohio_rd_reg(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	printk("reg[%x] = %x\n", cmd, OhioReadReg(cmd));

	return count;

}

ssize_t anx_ohio_wr_reg(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int cmd, val;
	int result;

	result = sscanf(buf, "%d  %d", &cmd, &val);
	pr_info("c %x val %x\n", cmd, val);
	OhioWriteReg(cmd, val);
	pr_info("reg[%x] = %x\n", cmd, OhioReadReg(cmd));
	return count;
}

ssize_t anx_ohio_dump_register(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int i = 0;
	for (i = 0; i < 256; i++) {
		printk("%x", OhioReadReg(i));
		if (i % 0x10 == 0)
			pr_info("\n");

		snprintf(&buf[i], sizeof(u8), "%d", OhioReadReg(i));
	}

	printk("\n");

	return i;
}

ssize_t anx_ohio_atd_test(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int result;

	if (OhioReadReg(0x44) == fw_ver)
		result = 1;
	else
		result = 0;

	pr_info("ATD read fw verison = %x, reg[44] = %x\n", fw_ver, OhioReadReg(0x44));

	return snprintf(buf, PAGE_SIZE, "%d\n", result);
}

ssize_t anx_ohio_atd_cc1_test(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int result = 0;
	int power_ctrl;
	int ana_status;

	power_ctrl = OhioReadReg(0x0d);
	ana_status = OhioReadReg(0x40);

	pr_info("ATD CC1 reg[0d] = %x , reg[40] = %x\n", power_ctrl, ana_status);

	if((ana_status & 0x08) && (power_ctrl & 0x80))
		result = 1;

	return snprintf(buf, PAGE_SIZE, "%s\n", result ? "PASS" : "FAIL");
}

ssize_t anx_ohio_atd_cc2_test(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int result = 0;
	int power_ctrl;
	int ana_status;

	power_ctrl = OhioReadReg(0x0d);
	ana_status = OhioReadReg(0x40);

	pr_info("ATD CC2 reg[0d] = %x , reg[40] = %x\n", power_ctrl, ana_status);

	if((ana_status & 0x08) && (power_ctrl & 0x10))
		result = 1;

	return snprintf(buf, PAGE_SIZE, "%s\n", result ? "PASS" : "FAIL");
}

ssize_t anx_ohio_select_rdo_index(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int cmd;
	cmd = sscanf(buf, "%d", &cmd);
	if (cmd <= 0)
		return 0;

	pr_info("NewRDO idx %d, Old idx %d\n", cmd, sel_voltage_pdo_index);
	sel_voltage_pdo_index = cmd;
	return count;
}

/* for debugging */
static struct device_attribute anx_ohio_device_attrs[] = {
	__ATTR(pdcmd, S_IWUSR, NULL,
	       anx_ohio_send_pd_cmd),
	__ATTR(rdreg, S_IWUSR, NULL,
	       anx_ohio_rd_reg),
	__ATTR(wrreg, S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(rdoidx, S_IWUSR, NULL,
	       anx_ohio_wr_reg),
	__ATTR(dumpreg, S_IRUGO, anx_ohio_dump_register,
	       NULL),
	__ATTR(prole, S_IRUGO, anx_ohio_get_power_role,
	       NULL),
	__ATTR(drole, S_IRUGO, anx_ohio_get_data_role,
	       NULL),
	__ATTR(atd, S_IRUGO, anx_ohio_atd_test,
	       NULL),
	__ATTR(atd_cc1, S_IRUGO, anx_ohio_atd_cc1_test,
	       NULL),
	__ATTR(atd_cc2, S_IRUGO, anx_ohio_atd_cc2_test,
	       NULL),
	__ATTR(trysrc, S_IRUGO, anx_ohio_try_source,
	       NULL),
	__ATTR(trysink, S_IRUGO, anx_ohio_try_sink,
	       NULL),
	__ATTR(pswap, S_IRUGO, anx_ohio_send_pswap,
	       NULL),
	__ATTR(dswap, S_IRUGO, anx_ohio_send_dswap,
	       NULL),
	__ATTR(updateotp, S_IWUSR, NULL,
	       anx_ohio_update_otp),
	__ATTR(verifyotp, S_IWUSR, NULL,
	       anx_ohio_verify_otp)
};
#else
static struct device_attribute anx_ohio_device_attrs[] = {  };
#endif

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	pr_info("ohio create system fs interface ...\n");
	for (i = 0; i < ARRAY_SIZE(anx_ohio_device_attrs); i++)
		if (device_create_file(dev, &anx_ohio_device_attrs[i]))
			goto error;
	pr_info("success\n");
	return 0;
error:

	for (; i >= 0; i--)
		device_remove_file(dev, &anx_ohio_device_attrs[i]);
	pr_err("%s %s: ohio Unable to create interface", LOG_TAG, __func__);
	return -EINVAL;
}

void notify_dual_role_change(void){

	if (!g_ohio)
		pr_err("ohio not ready");

	pr_info("update dual role status\n");
	dual_role_instance_changed(g_ohio->dual_role);
}

module_init(ohio_init);
module_exit(ohio_exit);

MODULE_DESCRIPTION("USB PD Ohio driver");
MODULE_AUTHOR("Xia Junhua <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.71");
