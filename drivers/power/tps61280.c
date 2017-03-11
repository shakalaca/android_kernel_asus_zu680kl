#define pr_fmt(fmt) "TPS61280 %s: " fmt, __func__

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

/**********************************************************
  *
  *   [Register]
  *
  *********************************************************/
#define TPS61280_REG_NUM 6
#define TPS61280_REG_VERSION         (0x00)
#define TPS61280_REG_CONFIG          (0x01)
#define TPS61280_REG_VOUTFLOORSET    (0x02)
#define TPS61280_REG_VOUTROOFSET     (0x03)
#define TPS61280_REG_ILIMSET         (0x04)
#define TPS61280_REG_STATUS          (0x05)
#define TPS61280_REG_E2PROMCTRL      (0xFF)

/**********************************************************
  *
  *   [MASK/SHIFT]
  *
  *********************************************************/
#define REG_CONFIG_RESET_MASK            (0x01)
#define REG_CONFIG_RESET_SHIFT           (7)
#define REG_CONFIG_ENABLE_MASK           (0x03)
#define REG_CONFIG_ENABLE_SHIFT          (5)
#define REG_CONFIG_RESERVED_MASK         (0x01)
#define REG_CONFIG_RESERVED_SHIFT        (4)
#define REG_CONFIG_GPIOCFG_MASK          (0x01)
#define REG_CONFIG_GPIOCFG_SHIFT         (3)
#define REG_CONFIG_SSFM_MASK             (0x01)
#define REG_CONFIG_SSFM_SHIFT            (2)
#define REG_CONFIG_MODE_CTRL_MASK        (0x03)
#define REG_CONFIG_MODE_CTRL_SHIFT       (0)

#define REG_VOUTFLOORSET_RESERVED_MASK   (0x07)
#define REG_VOUTFLOORSET_RESERVED_SHIFT  (5)
#define REG_VOUTFLOORSET_VOUT_TH_MASK    (0x1F)
#define REG_VOUTFLOORSET_VOUT_TH_SHIFT   (0)

#define REG_VOUTROOFSET_RESERVED_MASK   (0x07)
#define REG_VOUTROOFSET_RESERVED_SHIFT  (5)
#define REG_VOUTROOFSET_VOUT_TH_MASK    (0x1F)
#define REG_VOUTROOFSET_VOUT_TH_SHIFT   (0)

#define REG_ILIMSET_RESERVED_MASK       (0x03)
#define REG_ILIMSET_RESERVED_SHIFT      (6)
#define REG_ILIMSET_ILIM_OFF_MASK       (0x01)
#define REG_ILIMSET_ILIM_OFF_SHIFT      (5)
#define REG_ILIMSET_SOFTSTART_MASK      (0x01)
#define REG_ILIMSET_SOFTSTART_SHIFT     (4)
#define REG_ILIMSET_ILIM_MASK           (0x0F)
#define REG_ILIMSET_ILIM_SHIFT          (0)

#define REG_STATUS_TSD_MASK             (0x01)
#define REG_STATUS_TSD_SHIFT            (7)
#define REG_STATUS_HOTDIE_MASK          (0x01)
#define REG_STATUS_HOTDIE_SHIFT         (6)
#define REG_STATUS_DCDCMODE_MASK        (0x01)
#define REG_STATUS_DCDCMODE_SHIFT       (5)
#define REG_STATUS_OPMODE_MASK          (0x01)
#define REG_STATUS_OPMODE_SHIFT         (4)
#define REG_STATUS_ILIMPT_MASK          (0x01)
#define REG_STATUS_ILIMPT_SHIFT         (3)
#define REG_STATUS_ILIMBST_MASK         (0x01)
#define REG_STATUS_ILIMBST_SHIFT        (2)
#define REG_STATUS_FAULT_MASK           (0x01)
#define REG_STATUS_FAULT_SHIFT          (1)
#define REG_STATUS_PGOOD_MASK           (0x01)
#define REG_STATUS_PGOOD_SHIFT          (0)

#define REG_E2PROMCTRL_WEN_MASK         (0x01)
#define REG_E2PROMCTRL_WEN_SHIFT        (7)
#define REG_E2PROMCTRL_WP_MASK          (0x01)
#define REG_E2PROMCTRL_WP_SHIFT         (6)
#define REG_E2PROMCTRL_ISE2PROMWP_MASK  (0x01)
#define REG_E2PROMCTRL_ISE2PROMWP_SHIFT (5)
#define REG_E2PROMCTRL_RESERVED_MASK    (0x0F)
#define REG_E2PROMCTRL_RESERVED_SHIFT   (0)

u8 tps61280_reg[TPS61280_REG_NUM] = {0};

struct tps61280_chip {
	struct i2c_client       *client;
	struct device           *dev;
};

static int tps61280_write_byte(struct tps61280_chip *chip, int reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err("i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int tps61280_read_byte(struct tps61280_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from %02x: %c\n", reg, ret);
		return -1;
	} else {
		*val = ret;
	}

	return 0;
}

static int tps61280_dump_register(struct tps61280_chip *chip)
{
	int i = 0;
	int rc;
	for (i=0; i<TPS61280_REG_NUM; i++)
	{
		rc = tps61280_read_byte(chip, i, &tps61280_reg[i]);
		if (rc) {
			pr_err("read reg failed, [0x%x]\n", i);
		}
		pr_info("[0x%x] = 0x%x", i, tps61280_reg[i]);
	}
	return 0;
}

static int tps61280_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tps61280_chip *chip;
	int rc;
	u8 val = 0;

	pr_info("++\n");
	if (!(chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL))) {
		pr_err("probe alloc memory failed!!!\n");
		return -1;
	}

	chip->client = client;
	chip->dev = &client->dev;
	i2c_set_clientdata(client, chip);

	pr_info("dump reg value before config\n");
	rc = tps61280_dump_register(chip);
	if (rc) {
		pr_err("dump register failed. probe not success.\n");
		return -1;
	}

	tps61280_write_byte(chip, TPS61280_REG_VOUTROOFSET, 0xF);
	tps61280_read_byte(chip, TPS61280_REG_VOUTROOFSET, &val);
	pr_info("VOUTROOFSET after setting is %s", val == 0xF ? "3.6V" : "not 3.6V");
	pr_info("--\n");
	return 0;
}

static int tps61280_remove(struct i2c_client *client)
{
	return 0;
}

static struct of_device_id tps61280_match_table[] = {
	{ .compatible = "tps61280",},
	{ },
};

static const struct i2c_device_id tps61280_id[] = {
	{"tps61280", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, tps61280_id);

static struct i2c_driver boost_tps61280_driver = {
	.driver		= {
		.name		= "tps61280",
		.owner		= THIS_MODULE,
		.of_match_table = tps61280_match_table,
	},
	.probe		= tps61280_probe,
	.remove		= tps61280_remove,
	.id_table	= tps61280_id,
};

module_i2c_driver(boost_tps61280_driver);

MODULE_DESCRIPTION("POWER BOOST TPS61280");
MODULE_LICENSE("GPL v2");
