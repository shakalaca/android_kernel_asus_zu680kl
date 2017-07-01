/*
 * msm_debugfs.c
 *
 *  Created on: Jan 13, 2015
 *      Author: charles
 *      Email: Weiche_Tsai@asus.com
 */
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include "msm_debugfs.h"
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#define MAX_CAMERA 2
static int number_of_camera = 0;
static struct debugfs dbgfs[MAX_CAMERA];
static struct otp_struct otp_data;
static struct msm_actuator_ctrl_t *s_ctrl_vcm;
static struct msm_ois_ctrl_t *o_ctrl_ois;
static struct msm_sensor_ctrl_t *s_ctrl_imx318;
static int imx318_power_on;
extern int g_ftm_mode;
#define OTP_SIZE 32
static unsigned char imx318_otp[OTP_SIZE] = {0};
static unsigned char ov8856_otp[OTP_SIZE] = {0};
static unsigned char ov8856_otp_1[OTP_SIZE] = {0};
static unsigned char ov8856_otp_2[OTP_SIZE] = {0};
static int ois_mode_err;
static int ois_accel_gain_err;
#define VCM_NOISE_WA 1

/* Defines for OTP Data Registers */
#define T4K35_OTP_START_ADDR	0x3504
#define T4K35_OTP_PAGE_REG	0x3502
#define T4K35_OTP_ENABLE		0x3500
#define T4K35_OTP_PAGE_SIZE	32
#define T4K35_OTP_DATA_SIZE	512
#define T4K35_OTP_READ_ONETIME	32

#define DEFAULT_UID "000000000000000000000000"
#define DEFAULT_OTP "NO available OTP"

static int dbg_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_dump_imx318_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", imx318_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", imx318_otp[i]);
	}
	pr_debug("OTP=%s\n", debug_buf);

#if 0
	len = snprintf(bp, dlen,
		"0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
		otp_data.af_inf[0], otp_data.af_inf[1],
		otp_data.af_30cm[0], otp_data.af_30cm[1],
		otp_data.af_5cm[0], otp_data.af_5cm[1],
		otp_data.start_current[0], otp_data.start_current[1],
		otp_data.module_id, otp_data.vendor_id);
#endif

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_imx318_otp_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_imx318_otp_read,
};

static ssize_t dbg_dump_imx318_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < 12; i++)
		len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "%02X", imx318_otp[10 + i]);
	pr_debug("UID=%s\n", debug_buf);

#if 0
	len = snprintf(bp, dlen,
		"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		otp_data.dc[0], otp_data.dc[1],
		otp_data.dc[2], otp_data.dc[3],
		otp_data.sn[0], otp_data.sn[1],
		otp_data.sn[2], otp_data.sn[3],
		otp_data.pn[0], otp_data.pn[1],
		otp_data.pn[2], otp_data.pn[3]);
#endif

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_imx318_uid_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_imx318_uid_read,
};

static ssize_t dbg_dump_ov8856_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[768];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_1[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_1[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_2[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_2[i]);
	}
	pr_debug("OTP=%s\n", debug_buf);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov8856_otp_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_ov8856_otp_read,
};

static ssize_t dbg_dump_ov8856_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = 0;
	for (i = 0; i < 12; i++)
		len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "%02X", ov8856_otp[10 + i]);
	pr_debug("UID=%s\n", debug_buf);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov8856_uid_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_dump_ov8856_uid_read,
};

static ssize_t dbg_default_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%s\n", DEFAULT_OTP);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_default_otp_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_default_otp_read,
};

static ssize_t dbg_default_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%s\n", DEFAULT_UID);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_default_uid_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_default_uid_read,
};

static ssize_t dbg_dump_vcm_test_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int val1 = 0, val2 = 0;
	int ret = 0;
	uint16_t status = 0;
	//uint16_t read_data = 0;
	int i=0;

	struct msm_camera_i2c_client client = s_ctrl_vcm->i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_vcm->i2c_client.i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_vcm->i2c_client.i2c_func_tbl->i2c_read;

	if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%x %x", &val1, &val2);
	pr_err("%s: val1=0x%x, val2=0x%x\n", __func__, val1, val2);

	if (s_ctrl_vcm==NULL)
		pr_err("%s: s_ctrl_vcm is null\n", __func__);
	if (i2c_write==NULL)
		pr_err("%s: i2c_write is null\n", __func__);

	client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	i2c_read(&client, 0x3304, &status, 1);
	pr_err("%s: 0x3304 status=%d\n", __func__, status);

	ret |= i2c_write(&client, 0xB315, 0x00, 1);	// 1 means data type is byte

	i2c_read(&client, 0xB973, &status, 1);
	pr_err("%s: init status=%d\n", __func__, status);
	if (status == 4) {
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(&client, 0x3370, 0x84, 1);	// 1 means data type is byte
		for (i=0; i<300; i++) {
			i2c_read(&client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

#if 0 /* test read data*/
	ret |= i2c_write(&client, 0x3373, 0x01, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3374, 0x02, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3378, 0x72, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3379, 0xF0, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3370, 0x82, 1);	// 1 means data type is byte
	i2c_read(&client, 0xB973, &status, 1);
	i2c_read(&client, 0x337a, &read_data, 1);
	pr_err("%s: read1 status=%d, read_data=%x\n", __func__, status, read_data);
#endif

	ret |= i2c_write(&client, 0x3374, 0x03, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3378, 0x72, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3379, 0xE0, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x337a, 0x01, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3370, 0x81, 1);	// 1 means data type is byte
	for (i=0; i<300; i++) {
		i2c_read(&client, 0x3370, &status, 1);
		pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
		if ((status&0x80)==0)
			break;
		usleep_range(10000, 11000);
	}
	i2c_read(&client, 0xB973, &status, 1);
	pr_err("%s: init status=%d\n", __func__, status);
	if (status == 4) {
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(&client, 0x3370, 0x84, 1);	// 1 means data type is byte
		for (i=0; i<300; i++) {
			i2c_read(&client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	ret |= i2c_write(&client, 0x3374, 0x04, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3378, 0x72, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3379, 0xA0, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x337a, val1, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x337b, val2, 1);	// 1 means data type is byte
	ret |= i2c_write(&client, 0x3370, 0x81, 1);	// 1 means data type is byte
	for (i=0; i<300; i++) {
		i2c_read(&client, 0x3370, &status, 1);
		pr_err("%s: %d, command send status polling=0x%x\n", __func__, i, status);
		if ((status&0x80)==0)
			break;
		usleep_range(10000, 11000);
	}
	i2c_read(&client, 0xB973, &status, 1);
	pr_err("%s: init status=%d\n", __func__, status);
	if (status == 4) {
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(&client, 0x3370, 0x84, 1);	// 1 means data type is byte
		for (i=0; i<300; i++) {
			i2c_read(&client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	return count;
}

static const struct file_operations dbg_dump_vcm_test_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_dump_vcm_test_write,
};

static ssize_t dbg_ois_mode_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int ois_mode = 0;
	struct msm_camera_i2c_client client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type);

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_write = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d", &ois_mode);
	pr_err("%s: set ois_mode to %d\n", __func__, ois_mode);

	if (o_ctrl_ois==NULL)
		pr_err("%s: o_ctrl_ois is null\n", __func__);
	if (i2c_write==NULL)
		pr_err("%s: i2c_write is null\n", __func__);

	switch (ois_mode) {
	case 0: /* turn off ois, only centering on */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	case 1: /* movie mode */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		ois_mode_err |= i2c_write(&client, 0x8436, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8440, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x8443, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x841B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x84B6, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C0, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C3, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x849B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x8438, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x84B8, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x8447, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x84C7, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0D0D, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	case 2: /* still mode */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		ois_mode_err |= i2c_write(&client, 0x8436, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8440, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x8443, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x841B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x84B6, 0xFd7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C0, 0xF07F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C3, 0xB41E, 2);
		ois_mode_err |= i2c_write(&client, 0x849B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x8438, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x84B8, 0x020D, 2);
		ois_mode_err |= i2c_write(&client, 0x8447, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x84C7, 0x862E, 2);
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0D0D, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	case 3: /* test mode */
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0C0C, 2);
		ois_mode_err |= i2c_write(&client, 0x8436, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8440, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x8443, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x841B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x84B6, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C0, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x84C3, 0xFF7F, 2);
		ois_mode_err |= i2c_write(&client, 0x849B, 0x8000, 2);
		ois_mode_err |= i2c_write(&client, 0x8438, 0x5209, 2);
		ois_mode_err |= i2c_write(&client, 0x84B8, 0x5209, 2);
		ois_mode_err |= i2c_write(&client, 0x8447, 0xF240, 2);
		ois_mode_err |= i2c_write(&client, 0x84C7, 0xF240, 2);
		ois_mode_err |= i2c_write(&client, 0x847F, 0x0D0D, 2);
		if (ois_mode_err < 0)
			pr_err("ois_mode %d set fail\n", ois_mode);
		break;
	default:
		pr_err("do not support this ois_mode %d\n", ois_mode);
		break;
	}

	return count;
}

static ssize_t dbg_ois_mode_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%d\n", ois_mode_err);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}


static const struct file_operations dbg_ois_mode_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_ois_mode_write,
	.read		= dbg_ois_mode_read,
};

static ssize_t dbg_ois_gyro_x_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int ret = 0;
	uint16_t gyro_x = 0;

	struct msm_camera_i2c_client client;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type);

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_read = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

	if (*ppos)
		return 0;	/* the end */

	ret |= i2c_read(&client, 0x8455, &gyro_x, 2);
	if (ret < 0)
		pr_err("%s: ret=%d\n", __func__, ret);

	len = snprintf(bp, dlen,"%d\n", gyro_x);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ois_gyro_x_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_ois_gyro_x_read,
};

static ssize_t dbg_ois_gyro_y_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int ret = 0;
	uint16_t gyro_y = 0;

	struct msm_camera_i2c_client client;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type);

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_read = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

	if (*ppos)
		return 0;	/* the end */

	ret |= i2c_read(&client, 0x8456, &gyro_y, 2);
	if (ret < 0)
		pr_err("%s: ret=%d\n", __func__, ret);

	len = snprintf(bp, dlen,"%d\n", gyro_y);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ois_gyro_y_fops = {
	.open		= dbg_dump_open,
	.read		= dbg_ois_gyro_y_read,
};

static ssize_t dbg_ois_accel_gain_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	unsigned int accel_gain_x = 0, accel_gain_y = 0;
	unsigned int accel_gain_x_swap = 0, accel_gain_y_swap = 0;

	struct msm_camera_i2c_client client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type);

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_write = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "%d %d", &accel_gain_x, &accel_gain_y);
	pr_err("%s: set accel_gain_x to %d, accel_gain_y=%d\n", __func__, accel_gain_x, accel_gain_y);

	if (o_ctrl_ois==NULL)
		pr_err("%s: o_ctrl_ois is null\n", __func__);
	if (i2c_write==NULL)
		pr_err("%s: i2c_write is null\n", __func__);

	/* should write low byte first */
	accel_gain_x_swap = ((accel_gain_x & 0xFF) << 8) | ((accel_gain_x & 0xFF00) >> 8 );
	accel_gain_y_swap = ((accel_gain_y & 0xFF) << 8) | ((accel_gain_y & 0xFF00) >> 8 );

	ois_accel_gain_err |= i2c_write(&client, 0x828B, accel_gain_x_swap, 2);
	ois_accel_gain_err |= i2c_write(&client, 0x82CB, accel_gain_y_swap, 2);
	if (ois_accel_gain_err < 0)
		pr_err("ois_accel_gain set fail\n");

	return count;
}

static ssize_t dbg_ois_accel_gain_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen,"%d\n", ois_accel_gain_err);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_ois_accel_gain_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_ois_accel_gain_write,
	.read		= dbg_ois_accel_gain_read,
};

static int g_reg, g_value;
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
static int ois_rw_proc_read(struct seq_file *buf, void *v)
{
	pr_debug("ois_rw_proc_read reg 0x%x=0x%x\n", g_reg, g_value);
	seq_printf(buf, "%x\n", g_value);
	return 0;
}

static ssize_t ois_rw_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int reg = -1, value = -1, temp;
	int rc, len;

	struct msm_camera_i2c_client client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type);
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type);

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_write = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;
	i2c_read = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

	len = (count > DBG_TXT_BUF_SIZE - 1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len))
		return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%x %x", &reg, &value);
	*ppos=len;
	if (reg != -1 && value != -1) {
		pr_debug("ois write reg=0x%x value=0x%x\n", reg, value);
		/* write to ois IC low byte first */
		temp = ((value & 0xFF) << 8) | ((value & 0xFF00) >> 8);
		rc = i2c_write(&client, (uint32_t)reg, (uint16_t)temp, 2);
		if (rc < 0) {
			pr_err("%s: failed to write 0x%x = 0x%x\n",
				 __func__, reg, value);
			return rc;
		}
	} else if (reg != -1) {
		rc = i2c_read(&client, (uint32_t)reg, (uint16_t *)&value, 2);
		pr_debug("ois read reg=0x%x value=0x%x\n", reg, value);
		if (rc < 0) {
			pr_err("%s: failed to read 0x%x\n",
				 __func__, reg);
			return rc;
		}
	}
	g_reg = reg;
	g_value = value & 0xFFFF;

	return len;
}

static int ois_rw_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_rw_proc_read, NULL);
}

static const struct file_operations ois_rw_proc_fops = {
	.owner = THIS_MODULE,
	.open = ois_rw_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = ois_rw_proc_write,
};

static int g_ois_read_times_status;
static int ois_read_times_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", g_ois_read_times_status);
	return 0;
}

static ssize_t ois_read_times_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int times = -1, i;
	int rc, len;
	static uint16_t OIS_GYRO_ACC_VALUE[4] = {
		0x8455,  /*Gyro X*/
		0x8456,  /*Gyro Y*/
		0x8280,  /*ACC X*/
		0x82C0   /*ACC Y*/
	};
	int gyro_x = 0, gyro_y = 0, accel_x = 0, accel_y = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buffer[5];
	char *filename = "/data/data/OIS_debug";

	struct msm_camera_i2c_client client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type);
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type);

	g_ois_read_times_status = 1;

	if (!imx318_power_on || o_ctrl_ois == NULL) {
		pr_err("please power on imx318 first");
		g_ois_read_times_status = 0;
		return -EPERM;
	}

	client = o_ctrl_ois->i2c_client;
	i2c_write = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_write;
	i2c_read = o_ctrl_ois->i2c_client.i2c_func_tbl->i2c_read;

	len = (count > DBG_TXT_BUF_SIZE - 1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len)) {
		g_ois_read_times_status = 0;
		return -EFAULT;
	}
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%d", &times);
	*ppos=len;

	if (times < 0) {
		pr_err("%s: times %d is invalid\n", __func__, times);
		g_ois_read_times_status = 0;
		return false;
	}

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("%s: openfail line = %d\n", __func__, __LINE__);
		g_ois_read_times_status = 0;
		return false;
	}
	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		for (i = 0; i < times; i++) {
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[0], (uint16_t *)&gyro_x, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[0], gyro_x);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[0]);
				g_ois_read_times_status = 0;
				return rc;
			}
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[1], (uint16_t *)&gyro_y, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[1], gyro_y);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[1]);
				g_ois_read_times_status = 0;
				return rc;
			}
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[2], (uint16_t *)&accel_x, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[2], accel_x);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[2]);
				g_ois_read_times_status = 0;
				return rc;
			}
			rc = i2c_read(&client, (uint32_t)OIS_GYRO_ACC_VALUE[3], (uint16_t *)&accel_y, 2);
			pr_debug("ois read reg=0x%x value=0x%x\n", OIS_GYRO_ACC_VALUE[3], accel_y);
			if (rc < 0) {
				pr_err("%s: failed to read 0x%x\n", __func__, OIS_GYRO_ACC_VALUE[3]);
				g_ois_read_times_status = 0;
				return rc;
			}

			sprintf(buffer, "%04x", gyro_x);
			buffer[4] = ',';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
			sprintf(buffer, "%04x", gyro_y);
			buffer[4] = ',';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
			sprintf(buffer, "%04x", accel_x);
			buffer[4] = ',';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
			sprintf(buffer, "%04x", accel_y);
			buffer[4] = '\n';
			fp->f_op->write(fp, buffer, 5, &fp->f_pos);
		}
	} else {
		/* Set addr_limit of the current process back to its own */
		set_fs(old_fs);

		/* Close file */
		filp_close(fp, NULL);
		pr_err("%s: f_op = null or write = null, fail line = %d\n", __func__, __LINE__);

		return false;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* Close file */
	filp_close(fp, NULL);

	return len;
}

static int ois_read_times_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ois_read_times_proc_read, NULL);
}

static const struct file_operations ois_read_times_proc_fops = {
	.owner = THIS_MODULE,
	.open = ois_read_times_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = ois_read_times_proc_write,
};

static int imx318_rw_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "reg 0x%x=0x%x\n", g_reg, g_value);
	return 0;
}

static ssize_t imx318_rw_proc_write(struct file *dev, const char *buf, size_t count, loff_t *ppos)
{
	int reg = -1, value = -1;
	int rc, len;
	struct msm_camera_i2c_client *client = s_ctrl_imx318->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_read;

	if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	len = (count > DBG_TXT_BUF_SIZE - 1) ? (DBG_TXT_BUF_SIZE-1) : (count);
	if (copy_from_user(debugTxtBuf, buf, len))
		return -EFAULT;
	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%x %x", &reg, &value);
	*ppos=len;
	if (reg != -1 && value != -1) {
		pr_info("imx318 write reg=0x%x value=0x%x\n", reg, value);
		rc = i2c_write(client, (uint32_t)reg, (uint16_t)value, 1);
		if (rc < 0) {
			pr_err("%s: failed to write 0x%x = 0x%x\n",
				 __func__, reg, value);
			return rc;
		}
	} else if (reg != -1) {
		rc = i2c_read(client, (uint32_t)reg, (uint16_t *)&value, 1);
		pr_info("imx318 read reg=0x%x value=0x%x\n", reg, value);
		if (rc < 0) {
			pr_err("%s: failed to read 0x%x\n",
				 __func__, reg);
			return rc;
		}
	}
	g_reg = reg;
	g_value = value & 0xFF;

	return len;
}

static int imx318_rw_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, imx318_rw_proc_read, NULL);
}

static const struct file_operations imx318_rw_proc_fops = {
	.owner = THIS_MODULE,
	.open = imx318_rw_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = imx318_rw_proc_write,
};

static int rear_otp_proc_read(struct seq_file *buf, void *v)
{
	int len = 0;
	char debug_buf[256];
	int i;

	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", imx318_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", imx318_otp[i]);
	}
	pr_debug("OTP=%s\n", debug_buf);

	seq_printf(buf, "%s", debug_buf);
	return 0;
}

static int rear_otp_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, rear_otp_proc_read, NULL);
}

static const struct file_operations rear_otp_proc_fops = {
	.owner = THIS_MODULE,
	.open = rear_otp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int front_otp_proc_read(struct seq_file *buf, void *v)
{
	int len = 0;
	char debug_buf[768];
	int i;

	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_1[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_1[i]);
	}
	len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "\n");
	for (i = 0; i < OTP_SIZE; i++) {
		if (i % 8 == 7)
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X\n", ov8856_otp_2[i]);
		else
			len += scnprintf(debug_buf + len, sizeof(debug_buf) - len, "0x%02X ", ov8856_otp_2[i]);
	}
	pr_debug("OTP=%s\n", debug_buf);

	seq_printf(buf, "%s", debug_buf);
	return 0;
}

static int front_otp_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, front_otp_proc_read, NULL);
}

static const struct file_operations front_otp_proc_fops = {
	.owner = THIS_MODULE,
	.open = front_otp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int rear_thermal_proc_read(struct seq_file *buf, void *v)
{
	int ret = 0;
	uint16_t temp = 0;
	int temp_translate = 0;

	struct msm_camera_i2c_client *client = s_ctrl_imx318->sensor_i2c_client;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_read;

	if (!imx318_power_on) {
		seq_printf(buf, "0\n");
		return 0;
	}

	ret |= i2c_read(client, 0x013a, &temp, 1);
	if (ret < 0)
		pr_err("%s: ret=%d\n", __func__, ret);

	if (temp <= 0x77)
		temp_translate = (int)temp;
	else if (temp >= 0x78 && temp <= 0x7f)
		temp_translate = 120;
	else if (temp >= 0x81 && temp <= 0xEC)
		temp_translate = -20;
	else if (temp >= 0xED && temp <= 0xFF)
		temp_translate = temp - 256;
	pr_debug("temp=0x%x, temp_translate=%d\n", temp, temp_translate);

	seq_printf(buf, "%d\n", temp_translate);
	return 0;
}

static int rear_thermal_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, rear_thermal_proc_read, NULL);
}

static const struct file_operations rear_thermal_proc_fops = {
	.owner = THIS_MODULE,
	.open = rear_thermal_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

int i2c_write_thu_imx318(uint8_t *value, int num) {
	int ret = 0, i = 0;
	uint16_t status = 0;
	struct msm_camera_i2c_client *client = s_ctrl_imx318->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_read;

	/* to make sure the channel is ok */
	i2c_read(client, 0xB973, &status, 1);
	if (status == 4) { // 2 is normal, 4 is communication error
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(client, 0x3370, 0x84, 1);
		for (i=0; i<300; i++) {
			i2c_read(client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	/* start to transfer data to imx318, then imx318 send to vcm */
	ret |= i2c_write(client, 0x3373, 0, 1); // 1 means data type is byte
	ret |= i2c_write(client, 0x3374, num, 1); // 1 means data type is byte
	for (i = 0; i < num; i++)
		ret |= i2c_write(client, 0x3378+i, (uint16_t)value[i], 1);
	ret |= i2c_write(client, 0x3370, 0x81, 1);

	/* polling to make sure the communication is done */
	for (i = 0; i < 300; i++) {
		i2c_read(client, 0x3370, &status, 1);
		if ((status&0x80)==0)
			break;
		pr_err("%s: %d, command send status polling=0x%x\n", __func__, i, status);
		usleep_range(10000, 11000);
	}
	return ret;
}

int i2c_read_thu_imx318(uint8_t *value, int num, uint8_t *read_data, int read_num) {
	int ret = 0, i = 0;
	uint16_t status_0x3370 = 0;
	uint16_t status = 0;
	struct msm_camera_i2c_client *client = s_ctrl_imx318->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl_imx318->sensor_i2c_client->i2c_func_tbl->i2c_read;

	/* to make sure the channel is ok */
	i2c_read(client, 0xB973, &status, 1);
	if (status == 4) { // 2 is normal, 4 is communication error
		pr_err("%s: communication reset\n", __func__);
		ret |= i2c_write(client, 0x3370, 0x84, 1);
		for (i=0; i<300; i++) {
			i2c_read(client, 0x3370, &status, 1);
			pr_err("%s: %d, init send status polling=0x%x\n", __func__, i, status);
			if ((status&0x80)==0)
				break;
			usleep_range(10000, 11000);
		}
	}

	/* start to transfer data to imx318, then imx318 send to vcm */
	ret |= i2c_write(client, 0x3373, read_num, 1); // 1 means data type is byte
	ret |= i2c_write(client, 0x3374, num, 1);
	for (i = 0; i < num; i++) {
		//pr_err("%d write 0x%x 0x%x\n", i, 0x3378+i, (uint16_t)value[i]);
		ret |= i2c_write(client, 0x3378+i, (uint16_t)value[i], 1);
	}
	ret |= i2c_write(client, 0x3370, 0x82, 1);
	usleep_range(100, 110);

	/* polling to make sure the communication is done */
	for (i = 0; i < 300; i++) {
		i2c_read(client, 0x3370, &status_0x3370, 1);
		if ((status&0x80)==0)
			break;
		pr_err("%s: %d, command send status polling=0x%x\n", __func__, i, status_0x3370);
		usleep_range(10000, 11000);
	}
	/* read data from vcm */
	for (i = 0; i < read_num; i++) {
		i2c_read(client, 0x3378+num+i, (uint16_t*)&(read_data[i]), 1);
		pr_debug("%s: read_data[%d]=0x%x\n", __func__, i, read_data[i]);
	}

	return ret;
}

static int vcm_z1;
static int vcm_z2;
static int vcm_Z1_Z2_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "0x%02x 0x%02x 0x%02x 0x%02x\n", (vcm_z1 & 0xFF00) >> 8, vcm_z1 & 0xFF, (vcm_z2 & 0xFF00) >> 8, vcm_z2 & 0xFF);
	return 0;
}

static int vcm_Z1_Z2_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, vcm_Z1_Z2_proc_read, NULL);
}

static const struct file_operations vcm_Z1_Z2_proc_fops = {
	.owner = THIS_MODULE,
	.open = vcm_Z1_Z2_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

void vcm_Z1_Z2_read(uint16_t *actuator_data_Z1, uint16_t *actuator_data_Z2)
{
	int ret = 0;
	uint8_t value1[2] = {0x72, 0xF0};
	uint8_t value2[2] = {0x73, 0x2E};
	uint8_t value3[2] = {0x73, 0x2F};
	uint8_t value4[2] = {0x73, 0x30};
	uint8_t value5[2] = {0x73, 0x31};
	uint8_t read_data[2] = {0};
	uint8_t read_data2[2] = {0};
	int vcm_z1_temp = 0;
	int vcm_z2_temp = 0;

	if (!imx318_power_on || s_ctrl_imx318 == NULL) {
		pr_err("imx318 not power on\n");
		return;
	}

	ret = i2c_read_thu_imx318(value1, 2, read_data, 1);
	pr_err("%s: read 0xF0=0x%x\n", __func__, read_data[0]);
	ret = i2c_read_thu_imx318(value2, 2, read_data, 1);
	ret = i2c_read_thu_imx318(value3, 2, read_data2, 1);
	pr_err("%s: read 0x2E, 0x2F={ 0x%x, 0x%x }\n", __func__, read_data[0], read_data2[0]);
	vcm_z1_temp = (read_data[0] << 8) | read_data2[0];
	vcm_z1 = (((vcm_z1_temp << 4)+ 0x8000)&0xffff) >> 4;
	pr_err("%s: convert Z1 to %d, 0x%x\n", __func__, vcm_z1, vcm_z1);
	ret = i2c_read_thu_imx318(value4, 2, read_data, 1);
	ret = i2c_read_thu_imx318(value5, 2, read_data2, 1);
	pr_err("%s: read 0x30, 0x31={ 0x%x, 0x%x }\n", __func__, read_data[0], read_data2[0]);
	vcm_z2_temp = (read_data[0] << 8) | read_data2[0];
	vcm_z2 = (((vcm_z2_temp << 4)+ 0x8000)&0xffff) >> 4;
	pr_err("%s: convert Z2 to %d, 0x%x\n", __func__, vcm_z2, vcm_z2);
	*actuator_data_Z1 = vcm_z1;
	*actuator_data_Z2 = vcm_z2;
}

void update_vcm_fw(int version)
{
	uint8_t value_init1[3] = {0x72, 0x84, 0x00};
	uint8_t value_init2[3] = {0x72, 0x87, 0x00};
	uint8_t value_init3[3] = {0x72, 0x8E, 0x00};
	uint8_t value_read_fw_id[2] = {0x73, 0x48};
	uint8_t read_data[32] = {0};

	uint8_t value_eeprom_w_enable1[3] = {0x72, 0x9C, 0xAF};
	uint8_t value_eeprom_w_enable2[3] = {0x72, 0x9D, 0x80};
	uint8_t value_read_status[2] = {0x72, 0xED};
	uint8_t value1_47[10] = {0x73, 0x20, 0x1F, 0x00, 0x19, 0x8B, 0x40, 0xFF, 0x40, 0x0E};
	uint8_t value2_47[10] = {0x73, 0x48, 0x47, 0xF0, 0x7F, 0xF0, 0x8E, 0xC0, 0x71, 0xD0};
	uint8_t value3_47[10] = {0x73, 0x50, 0x63, 0x10, 0x41, 0xB0, 0x28, 0x00, 0x00, 0x00};
	uint8_t value4_47[10] = {0x73, 0x58, 0x50, 0xC0, 0xC6, 0xD0, 0x7F, 0xF0, 0x06, 0x20};
	uint8_t value5_47[10] = {0x73, 0x60, 0x04, 0xA0, 0x76, 0xB0, 0x7F, 0xF0, 0xA8, 0x00};
	uint8_t value_eeprom_w_disable1[3] = {0x72, 0x9C, 0x00};
	uint8_t value_eeprom_w_disable2[3] = {0x72, 0x9D, 0x00};
	int wait_count = 0;
	int i = 0;
	int mismatch_count = 0;
	uint8_t read_value1[2] = {0x73, 0x20};
	uint8_t read_value2[2] = {0x73, 0x48};
	uint8_t read_value3[2] = {0x73, 0x50};
	uint8_t read_value4[2] = {0x73, 0x58};
	uint8_t read_value5[2] = {0x73, 0x60};
	uint8_t value1_40[10] = {0x73, 0x20, 0x1F, 0x00, 0x19, 0x8B, 0x40, 0xFF, 0x40, 0x18};
	uint8_t value2_40[10] = {0x73, 0x48, 0x40, 0x30, 0x7F, 0xF0, 0x8F, 0x70, 0x71, 0x30};
	uint8_t value3_40[10] = {0x73, 0x50, 0x61, 0xB0, 0x40, 0x30, 0x28, 0x00, 0x00, 0x00};
	uint8_t value4_40[10] = {0x73, 0x58, 0x50, 0xC0, 0xBF, 0xD0, 0x7F, 0xF0, 0x06, 0x20};
	uint8_t value5_40[10] = {0x73, 0x60, 0x09, 0x90, 0x6C, 0xF0, 0x7F, 0xF0, 0xA8, 0x00};

	uint8_t value1[10] = {0x73, 0x20, 0x1F, 0x00, 0x19, 0x8B, 0x40, 0xFF, 0x40, 0x0E};
	uint8_t value2[10] = {0x73, 0x48, 0x47, 0xF0, 0x7F, 0xF0, 0x8E, 0xC0, 0x71, 0xD0};
	uint8_t value3[10] = {0x73, 0x50, 0x63, 0x10, 0x41, 0xB0, 0x28, 0x00, 0x00, 0x00};
	uint8_t value4[10] = {0x73, 0x58, 0x50, 0xC0, 0xC6, 0xD0, 0x7F, 0xF0, 0x06, 0x20};
	uint8_t value5[10] = {0x73, 0x60, 0x04, 0xA0, 0x76, 0xB0, 0x7F, 0xF0, 0xA8, 0x00};

	if (version == 0x40) {
		memcpy(value1,  value1_40, sizeof(value1));
		memcpy(value2,  value2_40, sizeof(value2));
		memcpy(value3,  value3_40, sizeof(value3));
		memcpy(value4,  value4_40, sizeof(value4));
		memcpy(value5,  value5_40, sizeof(value5));
	} else if (version == 0x47) {
		memcpy(value1,  value1_47, sizeof(value1));
		memcpy(value2,  value2_47, sizeof(value2));
		memcpy(value3,  value3_47, sizeof(value3));
		memcpy(value4,  value4_47, sizeof(value4));
		memcpy(value5,  value5_47, sizeof(value5));
	}

	/* init and read vcm fw id */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	i2c_read_thu_imx318(value_read_fw_id, 2, read_data, 1);

	/* update fw */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	i2c_write_thu_imx318(value_eeprom_w_enable1, 3);
	i2c_write_thu_imx318(value_eeprom_w_enable2, 3);
	i2c_write_thu_imx318(value1, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value2, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value3, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value4, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value5, 10);
	wait_count = 0;
	do {
		usleep_range(3000, 3500);
		i2c_read_thu_imx318(value_read_status, 2, read_data, 1);
		wait_count++;
	} while (read_data[0] != 0x00 && wait_count < 5);
	i2c_write_thu_imx318(value_eeprom_w_disable1, 3);
	i2c_write_thu_imx318(value_eeprom_w_disable2, 3);

	/* verify fw write success */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	mismatch_count = 0;
	i2c_read_thu_imx318(read_value1, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value1[i+2]) {
			pr_err("%s: value1 %d expect 0x%x, actual 0x%x\n", __func__, i, value1[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value2, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value2[i+2]) {
			pr_err("%s: value2 %d expect 0x%x, actual 0x%x\n", __func__, i, value2[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value3, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value3[i+2]) {
			pr_err("%s: value3 %d expect 0x%x, actual 0x%x\n", __func__, i, value3[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value4, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value4[i+2]) {
			pr_err("%s: value4 %d expect 0x%x, actual 0x%x\n", __func__, i, value4[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	i2c_read_thu_imx318(read_value5, 2, read_data, 8);
	for (i = 0; i < 8; i++) {
		if (read_data[i] != value5[i+2]) {
			pr_err("%s: value5 %d expect 0x%x, actual 0x%x\n", __func__, i, value5[i+2], read_data[i]);
			mismatch_count++;
		}
	}
	pr_err("%s, mismatch_count=%d\n", __func__, mismatch_count);

}

uint8_t read_vcm_fw_id(void)
{
	uint8_t value_init1[3] = {0x72, 0x84, 0x00};
	uint8_t value_init2[3] = {0x72, 0x87, 0x00};
	uint8_t value_init3[3] = {0x72, 0x8E, 0x00};
	uint8_t value_read_fw_id[2] = {0x73, 0x48};
	uint8_t read_data[32] = {0};

	/* init and read vcm fw id */
	i2c_write_thu_imx318(value_init1, 3);
	i2c_write_thu_imx318(value_init2, 3);
	i2c_write_thu_imx318(value_init3, 3);
	i2c_read_thu_imx318(value_read_fw_id, 2, read_data, 1);

	pr_err("%s: fw_id=0x%x\n", __func__, read_data[0]);
	return read_data[0];
}

static ssize_t dbg_dump_vcm_fw_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	uint8_t fw_id = 0;

	if (*ppos)
		return 0;	/* the end */

	if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		fw_id = 0;
	} else
		fw_id = read_vcm_fw_id();

	len = snprintf(bp, dlen,"0x%x\n", fw_id);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_dump_vcm_fw_write(
	struct file *file,
	const char __user *buf,
	size_t count,
	loff_t *ppos)
{
	char debug_buf[256];
	int cnt = 0;
	unsigned int fw_id = 0;

	if (!imx318_power_on) {
		pr_err("please power on imx318 first");
		return -EPERM;
	}

	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;
	debug_buf[count] = '\0';	/* end of string */

	cnt = sscanf(debug_buf, "0x%x", &fw_id);
	pr_err("%s: fw_id=0x%x\n", __func__, fw_id);

	if (s_ctrl_vcm == NULL)
		pr_err("%s: s_ctrl_vcm is null\n", __func__);

	update_vcm_fw(fw_id);

	return count;
}

static const struct file_operations dbg_dump_vcm_fw_fops = {
	.open		= dbg_dump_open,
	.write		= dbg_dump_vcm_fw_write,
	.read		= dbg_dump_vcm_fw_read,
};

#if VCM_NOISE_WA
static int vcm_min_max_proc_read(struct seq_file *buf, void *v)
{
	uint16_t vcm_Z1_minus_25um = 0;
	vcm_Z1_minus_25um = vcm_z1 - ((vcm_z2 - vcm_z1)*25/300);
	seq_printf(buf, "0x%02x 0x%02x 0x%02x 0x%02x\n", (vcm_Z1_minus_25um & 0xFF00) >> 8, vcm_Z1_minus_25um & 0xFF, (vcm_z2 & 0xFF00) >> 8, vcm_z2 & 0xFF);
	return 0;
}

static int vcm_min_max_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, vcm_min_max_proc_read, NULL);
}

static const struct file_operations vcm_min_max_proc_fops = {
	.owner = THIS_MODULE,
	.open = vcm_min_max_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

int msm_debugfs_init(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_sensor_slave_info *slave_info)
{
	struct dentry *debugfs_dir;
	int camera_id = slave_info->camera_id;
	int chip_id = slave_info->sensor_id_info.sensor_id;
	const char *sensor_name = s_ctrl->sensordata->sensor_name;
	int index = number_of_camera++;
	char folder_name[10];

	if (index >= MAX_CAMERA) {
		pr_err("Invalid! number of camera (%d) > MAX Camera (%d)",
				number_of_camera, MAX_CAMERA);
		return -1;
	}
	sprintf(folder_name, "camera%d", index);
	debugfs_dir = debugfs_create_dir(folder_name, NULL);

	dbgfs[index].sensor_id = chip_id;

	debugfs_create_u8((camera_id ? "vga_status" : "camera_status"),
			0644, debugfs_dir, &dbgfs[index].status);
	debugfs_create_x16("sensor_id", 0644, debugfs_dir,
			&dbgfs[index].sensor_id);
	debugfs_create_u8("exposure_return0", 0666, debugfs_dir,
			&dbgfs[index].exposure_return0);

	if (camera_id == 0) {
		(void) debugfs_create_file("vcm_test", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_vcm_test_fops);
		(void) debugfs_create_file("vcm_fw", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_vcm_fw_fops);
		(void) debugfs_create_file("ois_gyro_x", S_IRUGO,
			debugfs_dir, NULL, &dbg_ois_gyro_x_fops);
		(void) debugfs_create_file("ois_gyro_y", S_IRUGO,
			debugfs_dir, NULL, &dbg_ois_gyro_y_fops);
		if (g_ftm_mode) {
			(void) debugfs_create_file("ois_mode", S_IRUGO | S_IWUGO,
				debugfs_dir, NULL, &dbg_ois_mode_fops);
			(void) debugfs_create_file("ois_accel_gain", S_IRUGO | S_IWUGO,
				debugfs_dir, NULL, &dbg_ois_accel_gain_fops);
			proc_create(OIS_RW_PROC_FILE, 0666, NULL, &ois_rw_proc_fops);
			proc_create(OIS_READ_TIMES_PROC_FILE, 0666, NULL, &ois_read_times_proc_fops);
		} else {
			(void) debugfs_create_file("ois_mode", S_IRUGO,
				debugfs_dir, NULL, &dbg_ois_mode_fops);
			(void) debugfs_create_file("ois_accel_gain", S_IRUGO,
				debugfs_dir, NULL, &dbg_ois_accel_gain_fops);
			proc_create(OIS_RW_PROC_FILE, 0664, NULL, &ois_rw_proc_fops);
			proc_create(OIS_READ_TIMES_PROC_FILE, 0664, NULL, &ois_read_times_proc_fops);
		}
	}

	if (!strcmp(sensor_name,"imx318")) {
		(void) debugfs_create_file("CameraOTP", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_imx318_otp_fops);
		(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_imx318_uid_fops);
		proc_create(REAR_OTP_PROC_FILE, 0664, NULL, &rear_otp_proc_fops);
		proc_create(REAR_OTP_THERMAL_FILE, 0664, NULL, &rear_thermal_proc_fops);
		proc_create(IMX318_RW_PROC_FILE, 0664, NULL, &imx318_rw_proc_fops);
		proc_create(VCM_Z1_Z2_PROC_FILE, 0664, NULL, &vcm_Z1_Z2_proc_fops);
#if VCM_NOISE_WA
		proc_create(VCM_MIN_MAX_PROC_FILE, 0664, NULL, &vcm_min_max_proc_fops);
#endif
	} else if (!strcmp(sensor_name,"ov8856")) {
		(void) debugfs_create_file("CameraOTP", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_ov8856_otp_fops);
		(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_ov8856_uid_fops);
		proc_create(FRONT_OTP_PROC_FILE, 0664, NULL, &front_otp_proc_fops);
	}
	return index;
}

void msm_debugfs_set_status(unsigned int index, unsigned int status)
{
	if (index < MAX_CAMERA)
		dbgfs[index].status = status;
}

static int ov8856_read_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	int  i;
	u16 local_data, package;
	u8 buf[32];
	u16 start_addr, end_addr;
	struct msm_camera_i2c_client *client = s_ctrl->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int (*i2c_read) (struct msm_camera_i2c_client *, uint32_t, uint16_t *,
		enum msm_camera_i2c_data_type) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read;
	int32_t (*i2c_read_seq)(struct msm_camera_i2c_client *, uint32_t,
		uint8_t *, uint32_t) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq;

	 /* make sure reset sensor as default */
	i2c_write(client, 0x0103, 0x01, 1);
	/*set 0x5001[3] to 0 */
	i2c_read(client, 0x5001, &local_data, 1);
	i2c_write(client, 0x5001, ((0x00 & 0x08) | (local_data & (~0x08))), 1);

	for (package = 2; package >= 0; package--) {
		if (package == 0) {
			start_addr = 0x7010;
			end_addr   = 0x702f;
		} else if (package == 1) {
			start_addr = 0x7030;
			end_addr   = 0x704f;
		} else if (package == 2) {
			start_addr = 0x7050;
			end_addr   = 0x706f;
		}
		/* [6] Manual mode(partial) */
		i2c_write(client, 0x3d84, 0xc0, 1);
		/* rst default:13 */
		i2c_write(client, 0x3d85, 0x06, 1);
		/*otp start addr*/
		i2c_write(client, 0x3d88, (start_addr >> 8) & 0xff, 1);
		i2c_write(client, 0x3d89, start_addr & 0xff, 1);
		/*otp end addr*/
		i2c_write(client, 0x3d8A, (end_addr >> 8) & 0xff, 1);
		i2c_write(client, 0x3d8B, end_addr & 0xff, 1);
		/* trigger auto_load */
		i2c_write(client, 0x0100, 0x01, 1);
		/*load otp into buffer*/
		i2c_write(client, 0x3d81, 0x01, 1);
		msleep(5);

		i2c_read_seq(client, start_addr, buf, 32);
#if 0
		if (buf[8] != 0 || buf[9] != 0) {
			memcpy(&ov8856_otp, (u8 *)&buf, sizeof(buf));
			pr_info("ov8856 otp read success\n");
			goto out;
		}
#endif
		if (package == 0) {
			memcpy(&ov8856_otp, (u8 *)&buf, sizeof(buf));
			goto out;
		}
		if (package == 1) {
			memcpy(&ov8856_otp_1, (u8 *)&buf, sizeof(buf));
		}
		if (package == 2) {
			memcpy(&ov8856_otp_2, (u8 *)&buf, sizeof(buf));
		}
	}
out:

	for (i = start_addr ; i <= end_addr ; i++)
		i2c_write(client, i, 0x00, 1);
	i2c_read(client, 0x5001, &local_data, 1);
	i2c_write(client, 0x5001, (0x08 & 0x08) | (local_data & (~0x08)), 1);
	i2c_write(client, 0x0100, 0x00, 1);
	pr_info("ov8856 otp read success\n");
	return 0;
}

static int t4k35_read_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	int page = 0;
	u16 index;
	u8 buf[32];
	struct msm_camera_i2c_client *client = s_ctrl->sensor_i2c_client;
	int (*i2c_write) (struct msm_camera_i2c_client *, uint32_t, uint16_t,
		enum msm_camera_i2c_data_type) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write;
	int32_t (*i2c_read_seq)(struct msm_camera_i2c_client *, uint32_t,
		uint8_t *, uint32_t) = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq;

	i2c_write(client, T4K35_OTP_ENABLE, 0x81, MSM_CAMERA_I2C_BYTE_DATA);

	for (page = 2; page >= 0; page--) {
		/*set page NO.*/
		i2c_write(client, T4K35_OTP_PAGE_REG, page, MSM_CAMERA_I2C_BYTE_DATA);
		for (index = 0 ; index + T4K35_OTP_READ_ONETIME <= T4K35_OTP_PAGE_SIZE ;
			index += T4K35_OTP_READ_ONETIME) {
			i2c_read_seq(client, T4K35_OTP_START_ADDR + index,
				&buf[index], T4K35_OTP_READ_ONETIME);
		}
		if ((buf[0] != 0 || buf[1] != 0) && (buf[0] != 0xff || buf[1] != 0xff)) {
			memcpy(&otp_data, &buf, sizeof(buf));
			pr_info("t4k35 otp read success\n");
			goto out;
		}
	}
	pr_err("t4k35 otp read failed\n");
out:
	return 0;
}

int msm_read_otp(struct msm_sensor_ctrl_t *s_ctrl, struct msm_camera_sensor_slave_info *slave_info)
{
	const char *sensor_name = s_ctrl->sensordata->sensor_name;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}

	if (!strcmp(sensor_name,"t4k35"))
		t4k35_read_otp(s_ctrl);
	else if (!strcmp(sensor_name,"ov8856"))
		ov8856_read_otp(s_ctrl);

	if (!strcmp(sensor_name,"imx318")) {
		s_ctrl_imx318 = s_ctrl;
	}

	return 0;
}
void msm_set_actuator_ctrl(struct msm_actuator_ctrl_t *s_ctrl)
{
	s_ctrl_vcm = s_ctrl;
}
void msm_set_ois_ctrl(struct msm_ois_ctrl_t *o_ctrl)
{
	o_ctrl_ois = o_ctrl;
}
void get_eeprom_OTP(struct msm_eeprom_memory_block_t *block)
{
	memcpy(imx318_otp, block->mapdata, OTP_SIZE);
}
void imx318_power_state(int power_on)
{
	imx318_power_on = power_on;
}
