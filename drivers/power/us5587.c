/*
ADC US5587 Driver
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include <linux/gpio.h>


#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/spmi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

#include <linux/i2c/ads1015.h>

//ASUS BSP Austin_T : global ADS1013_READY +++
bool us5587_ready;
EXPORT_SYMBOL(us5587_ready);

//Define register addresses of us5587 0X38
#define us5587_raddr 0x38

struct us5587_data
{			
	u32 gpio_134;	
	u32 gpio_flags134;
};

struct i2c_client *us5587_client;

/*
us5587_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming 
write_val  :	the value will be written 
*/
int us5587_write_reg(uint8_t slave_addr, uint8_t cmd_reg, uint8_t write_val)
{
	int ret = 0;
	
	us5587_client->addr = slave_addr; //real SMBus address (8 bits)
	ret = i2c_smbus_write_byte_data(us5587_client, cmd_reg, 0x4);
	if (ret < 0) {
		printk("%s: failed to write i2c addr=%x\n",
			__func__, slave_addr);			
	}	
	return ret;
}

/*
us5587_read_reg():	read 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming 
store_read_val  :	value be read will store here

*/
int us5587_read_reg(uint8_t slave_addr, uint8_t cmd_reg, uint8_t *store_read_val)
{
	int ret = 0;
		
	us5587_client->addr = slave_addr;
	printk("[BAT][CHG] us5587_read_reg addr=%x\n", slave_addr);
	ret = i2c_smbus_read_byte_data(us5587_client, cmd_reg);
	printk("[BAT][CHG] ret=%d\n", ret);
	if (ret < 0) {
		printk("%s: failed to read i2c addr=%x\n",	__func__, slave_addr);		
	}

	*store_read_val = (uint8_t) ret;

	return 0;
}

u8 us5587_dump_value(void)
{
	u8 my_read_value = 0;

	us5587_read_reg(us5587_raddr, 0x04, &my_read_value);
	printk("[BAT][CHG] AIN2 voltage value = 0x%xh\n", my_read_value);

	return my_read_value;
}
EXPORT_SYMBOL(us5587_dump_value);

#define thermistor_range 80
// index 0 represents value when ntc is in 31 degree celsius
// index 79 represents value when ntc is in 110 degree celsius
static unsigned int thermistor_array[thermistor_range] = {
	0xFF, 0xFF, 0xFB, 0xF4, 0xEE, 0xE7, 0xE1, 0xDB, 0xD4, 0xCE,
	0xC9, 0xC3, 0xBD, 0xB8, 0xB2, 0xAD, 0xA8, 0xA3, 0x9E, 0x99,
	0x94, 0x90, 0x8C, 0x87, 0x83, 0x7F, 0x7B, 0x77, 0x74, 0x70,
	0x6D, 0x69, 0x66, 0x63, 0x60, 0x5D, 0x5A, 0x57, 0x54, 0x52,
	0x4F, 0x4D, 0x4A, 0x48, 0x46, 0x44, 0x42, 0x40, 0x3E, 0x3C,
	0x3A, 0x38, 0x37, 0x35, 0x33, 0x32, 0x30, 0x2F, 0x2D, 0x2C,
	0x2B, 0x29, 0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,
	0x20, 0x1F, 0x1E, 0x1D, 0x1C, 0x1B, 0x1B, 0x1A, 0x19, 0x18 };

static u16 find_closest_in_array(u8 val)
{
	int n;
	for (n = 0; n < thermistor_range;  n++) {
		if (val >= thermistor_array[n])
			break;
	}
	return n + 31;
}

u16 us5587_read_pmi8952_thermal(void)
{
	u8 my_read_value = 0;
	u16 temp = 0;

	if (us5587_ready) {
		us5587_read_reg(us5587_raddr, 0x03, &my_read_value);
		temp = find_closest_in_array(my_read_value);
		printk("[BAT][CHG] AIN1 voltage value = 0x%xh, temp: %d\n", my_read_value, temp);
	} else {
		printk("[BAT][CHG] driver not ready\n");
		return 70;
	}

	return temp;
}
EXPORT_SYMBOL(us5587_read_pmi8952_thermal);

u16 us5587_read_smb1351_thermal(void)
{
	u8 my_read_value = 0;
	u16 temp = 0;

	if (us5587_ready) {
		us5587_read_reg(us5587_raddr, 0x05, &my_read_value);
		temp = find_closest_in_array(my_read_value);
		printk("[BAT][CHG] AIN3 voltage value = 0x%xh, temp: %d\n", my_read_value, temp);
	} else {
		printk("[BAT][CHG] driver not ready\n");
                return 70;
	}
	return temp;
}
EXPORT_SYMBOL(us5587_read_smb1351_thermal);

static ssize_t adc_reg_value_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u16 ret;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);
	ret = us5587_dump_value();

	return sprintf(buf, "reg value = 0x%xh\n", ret);
}

static ssize_t pmi8952_thermal_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u16 ret;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);
	ret = us5587_read_pmi8952_thermal();

	return sprintf(buf, "%d\n", ret);
}

static ssize_t smb1351_thermal_show(struct device *dev, struct device_attribute *da,
	char *buf)
{
	u16 ret;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);
	ret = us5587_read_smb1351_thermal();

	return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(adc_reg_value, 0666, adc_reg_value_show, NULL);
static DEVICE_ATTR(pmi8952_thermal, 0666, pmi8952_thermal_show, NULL);
static DEVICE_ATTR(smb1351_thermal, 0666, smb1351_thermal_show, NULL);

static struct attribute *dump_reg_attrs[] = {
	&dev_attr_adc_reg_value.attr,
	&dev_attr_pmi8952_thermal.attr,
	&dev_attr_smb1351_thermal.attr,
	NULL
};

static const struct attribute_group dump_reg_attr_group = {
	.attrs = dump_reg_attrs,
};

static int us5587_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct us5587_data *data;
	int rc;
	int ret;
	u8 reg;

	printk("[BAT][CHG] %s start\n", __FUNCTION__);

	us5587_ready = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("[BAT][CHG] %s: i2c bus does not support the us5587\n", __FUNCTION__);
	}

	data = devm_kzalloc(&client->dev, sizeof(struct us5587_data), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	us5587_client = client;
	i2c_set_clientdata(client, data);

	ret = us5587_read_reg(us5587_raddr, 0x04, &reg);
	if (ret < 0) {
		printk("[BAT][CHG] %s: i2c slave_addr 0x38 for us5587 not ACK\n", __FUNCTION__);
		return -ENODEV;
	}

	//to get gpio information +++
	if(client->dev.of_node)
	{
		data->gpio_134 = of_get_named_gpio_flags(client->dev.of_node, 
				"us5587-gpios134",0, &data->gpio_flags134);
		printk("[BAT][CHG] get GPIO information done \n");
	}
	
	printk("[BAT][CHG] gpio_134[%d], Slave address[0x%02xh]\n",
			data->gpio_134, client->addr);
	//to get gpio information ---
	
	//Enable gpio_134 +++	
	ret = gpio_request(data->gpio_134, "us5587-gpio-134");
	if (ret) 
		printk("[BAT][CHG] failed to request gpio134\n");

	gpio_direction_output(data->gpio_134, 1);
	msleep(100);
	//Enable gpio_134  ---

	rc = sysfs_create_group(&client->dev.kobj, &dump_reg_attr_group);
	if (rc)
		goto exit_remove;

	us5587_ready = 1;

	printk("[BAT][CHG] %s end\n", __FUNCTION__);

	return 0;

exit_remove:
		sysfs_remove_group(&client->dev.kobj, &dump_reg_attr_group);
	return rc;
	
}

static struct of_device_id us5587_match_table[] = {
	{ .compatible = "qcom,us5587",},
	{ },
};

static const struct i2c_device_id us5587_id[] = {
	{ "us5587", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, us5587_id);

static struct i2c_driver us5587_driver = {
	.driver = {
		.name = "us5587",
		.owner		= THIS_MODULE,
		.of_match_table	= us5587_match_table,
	},
	.probe = us5587_probe,
	.id_table = us5587_id,
};

module_i2c_driver(us5587_driver);

MODULE_AUTHOR("Dirk Eibach <eibach@gdsys.de>");
MODULE_DESCRIPTION("US5587 driver");
MODULE_LICENSE("GPL");

