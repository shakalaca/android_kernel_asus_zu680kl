/*! @file spi_drv.c
 *******************************************************************************
 **  SPI Driver Interface Functions
 **
 **  This file contains the SPI driver interface functions.
 **
 **  Copyright 2013-2014 Synaptics, Inc. All Rights Reserved.
 **
 **  This program is free software; you can redistribute it and/or
 **  modify it under the terms of the GNU General Public License
 **  as published by the Free Software Foundation; either version 2
 **  of the License, or (at your option) any later version.
 **
 **  This program is distributed in the hope that it will be useful,
 **  but WITHOUT ANY WARRANTY; without even the implied warranty of
 **  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 **  GNU General Public License for more details.
 **
 **  You should have received a copy of the GNU General Public License
 **  along with this program; if not, write to the Free Software
 **  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 **  MA  02110-1301, USA."
 **
 */

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/fdtable.h>
#include <linux/eventfd.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif /* CONFIG_COMPAT */

#define DEBUG 0
#include <vfs_drv_ioctl.h>
#include <vfsspi_drv.h>
#include "asus_fp_id.h"

/* The pdev driver private structure. */
/**
 * vfs_platform_dev_data - The pdev driver private structure
 * @devt:Device ID
 * @cdev:Character device handle
 * @vfs_platform_lock:The lock for the pdev device
 * @pdev:The platform device
 * @device_entry:Device entry list
 * @buffer_mutex:The lock for the transfer buffer
 * @is_opened:Indicates that driver is opened
 * @drdy_pin:DRDY GPIO pin number
 * @sleep_pin:Sleep GPIO pin number
 * @user_pid:User process ID, to which the kernel signal
 indicating DRDY event is to be sent
 * @signal_id:Signal ID which kernel uses to indicating
 user mode driver that DRDY is asserted
 * @drdy_enabled:Indicates that DRDY irq is enabled
 */
struct vfs_platform_dev_data {
	dev_t devt;
	struct cdev cdev;
	spinlock_t vfs_platform_lock;
	struct platform_device *pdev;
	struct class *dev_class;
	struct list_head device_entry;
	struct mutex buffer_mutex;
	unsigned int is_opened;

	int drdy_pin;
	int sleep_pin;
	int vcc_3v3;
	int vcc_1v8;

	int gpio_irq;
	int wakeup_source;
	int user_pid;
	int signal_id;
	unsigned int drdy_enabled;
	struct input_dev *input;
};

static int vfs_platform_open(struct inode *inode, struct file *filp);
static int vfs_platform_release(struct inode *inode, struct file *filp);

static int vfs_platform_probe(struct platform_device *pdev);
static int vfs_platform_remove(struct platform_device *pdev);

static int vfs_platform_dev_init(struct platform_device *pdev,
		struct vfs_platform_dev_data **pdata);
static void vfs_platform_dev_uninit(struct vfs_platform_dev_data *vfs_platform_dev);

static int vfs_platform_chr_dev_register(struct vfs_platform_dev_data *vfs_platform_dev);
static void vfs_platform_chr_dev_unregister(struct vfs_platform_dev_data *vfs_platform_dev);

static int vfs_platform_gpio_init(struct vfs_platform_dev_data *vfs_platform_dev);
static void vfs_platform_gpio_uninit(struct vfs_platform_dev_data *vfs_platform_dev);

static irqreturn_t vfs_platform_irq(int irq, void *context);
static void vfs_platform_enable_irq(struct vfs_platform_dev_data *vfs_platform_dev);
static void vfs_platform_disable_irq(struct vfs_platform_dev_data *vfs_platform_dev);

static int vfs_platform_send_drdy_notify(struct vfs_platform_dev_data *vfs_platform_dev);

static long vfs_platform_unlocked_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg);
#ifdef CONFIG_COMPAT
static long vfs_platform_compat_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg);
#endif /* CONFIG_COMPAT */
static long vfs_platform_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg, int compat);
static void vfs_platform_hard_reset(struct vfs_platform_dev_data *vfs_platform_dev);
static void vfs_platform_suspend(struct vfs_platform_dev_data *vfs_platform_dev);
static int vfs_platform_register_drdy_signal(struct vfs_platform_dev_data *vfs_platform_dev,
		unsigned long arg);
static int vfs_platform_set_drdy_int(struct vfs_platform_dev_data *vfs_platform_dev,
		unsigned long arg);
static int vfs_platform_select_drdy_notify_type(struct vfs_platform_dev_data *vfs_platform_dev,
		unsigned long arg);

#ifdef CONFIG_OF
static int vfs_platform_parse_dt(struct device *dev,
		struct vfs_platform_dev_data *vfs_platform_dev)
{

	struct device_node *np = dev->of_node;

	/* +++reset, irq gpio info+++ */
	vfs_platform_dev->sleep_pin = of_get_named_gpio_flags(np, "sleep-gpio",
				0, NULL);
	if (vfs_platform_dev->sleep_pin < 0)
		return vfs_platform_dev->sleep_pin;

	vfs_platform_dev->drdy_pin = of_get_named_gpio_flags(np, "irq-gpio",
				0, NULL);
	if (vfs_platform_dev->drdy_pin < 0)
		return vfs_platform_dev->drdy_pin;


	vfs_platform_dev->vcc_1v8 = of_get_named_gpio_flags(np, "osvcc-gpio",
				0, NULL);

	vfs_platform_dev->vcc_3v3 = of_get_named_gpio_flags(np, "vcc3v3-gpio",
				0, NULL);

	return 0;
}

static struct of_device_id syna_of_table[] = {
	{ .compatible = "syna,vfsspi",},
	{ },
};
#else
static int vfs_platform_parse_dt(struct device *dev,
		struct vfs_platform_dev_data *vfs_platform_dev)
{
	return 0;
}
#define syna_of_table NULL
#endif
/* SPI driver info */
struct platform_driver vfs_platform = {
	.driver = {
		.name = SYNA_PART_NAME,
		.owner = THIS_MODULE,
		.of_match_table = syna_of_table,
	},
	.probe = vfs_platform_probe,
	.remove = vfs_platform_remove,
};

/* file operations associated with device */
const struct file_operations vfs_platform_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vfs_platform_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vfs_platform_compat_ioctl,
#endif /* CONFIG_COMPAT */
	.open = vfs_platform_open,
	.release = vfs_platform_release,
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_mutex);

static int vfs_platform_dev_init(struct platform_device *pdev,
		struct vfs_platform_dev_data **pdata)
{
	int status;
	struct vfs_platform_dev_data *vfs_platform_dev = NULL;

	PR_INFO("vfs_platform_dev_init\n");

	vfs_platform_dev = kzalloc(sizeof(*vfs_platform_dev), GFP_KERNEL);
	if (vfs_platform_dev == NULL) {
		PR_ERR("Failed to allocate buffer\n");
		status = -ENOMEM;
		goto cleanup;
	}
	vfs_platform_dev->pdev = pdev;

	/* Initialize driver data. */
	spin_lock_init(&vfs_platform_dev->vfs_platform_lock);
	mutex_init(&vfs_platform_dev->buffer_mutex);
	INIT_LIST_HEAD(&vfs_platform_dev->device_entry);

	status = vfs_platform_chr_dev_register(vfs_platform_dev);
	if (0 != status) {
		PR_ERR("vfs_platform_chr_dev_register failed! status= %d\n", status);
		goto cleanup;
	}

	platform_set_drvdata(pdev, vfs_platform_dev);

	*pdata = vfs_platform_dev;

cleanup:
	if (0 != status && NULL != vfs_platform_dev) {
		/* Release allocated resources */
		mutex_destroy(&vfs_platform_dev->buffer_mutex);
		kfree(vfs_platform_dev);
	}

	return status;
}

static void vfs_platform_dev_uninit(struct vfs_platform_dev_data *vfs_platform_dev)
{
	PR_INFO("vfs_platform_dev_uninit\n");

	if (vfs_platform_dev != NULL) {
		spin_lock_irq(&vfs_platform_dev->vfs_platform_lock);
		platform_set_drvdata(vfs_platform_dev->pdev, NULL);
		vfs_platform_dev->pdev = NULL;
		spin_unlock_irq(&vfs_platform_dev->vfs_platform_lock);

		vfs_platform_chr_dev_unregister(vfs_platform_dev);

		mutex_destroy(&vfs_platform_dev->buffer_mutex);

		kfree(vfs_platform_dev);
	}
}

static int vfs_platform_chr_dev_register(struct vfs_platform_dev_data *vfs_platform_dev)
{
	int status;
	struct device *dev = NULL;
	int chr_dev_alloc = 0;
	int chr_dev_init = 0;
	int class_create = 0;

	PR_INFO("vfs_platform_chr_dev_register\n");

	mutex_lock(&device_list_mutex);

	/* register major number for character device */
	status = alloc_chrdev_region(&(vfs_platform_dev->devt),
			0, 1, SYNA_PART_NAME);
	if (status < 0) {
		PR_ERR("alloc_chrdev_region failed\n");
		goto cleanup;
	}

	chr_dev_alloc = 1;

	cdev_init(&(vfs_platform_dev->cdev), &vfs_platform_fops);
	vfs_platform_dev->cdev.owner = THIS_MODULE;

	status = cdev_add(&(vfs_platform_dev->cdev), vfs_platform_dev->devt, 1);
	if (status < 0) {
		PR_ERR("cdev_add failed\n");
		goto cleanup;
	}

	chr_dev_init = 1;

	vfs_platform_dev->dev_class = class_create(THIS_MODULE, SYNA_PART_NAME);
	if (IS_ERR(vfs_platform_dev->dev_class)) {
		PR_ERR("class_create failed\n");
		status = PTR_ERR(vfs_platform_dev->dev_class);
		goto cleanup;
	}

	class_create = 1;

	dev = device_create(vfs_platform_dev->dev_class, &vfs_platform_dev->pdev->dev,
			vfs_platform_dev->devt, vfs_platform_dev, SYNA_DEV_NAME);
	if (IS_ERR(dev)) {
		PR_ERR("device_create failed\n");
		status = PTR_ERR(dev);
		goto cleanup;
	}

	list_add(&vfs_platform_dev->device_entry, &device_list);
	status = 0;

cleanup:
	mutex_unlock(&device_list_mutex);

	if (0 != status) {
		/* Release allocated resources */
		if (0 != class_create)
			class_destroy(vfs_platform_dev->dev_class);

		if (0 != chr_dev_init)
			cdev_del(&(vfs_platform_dev->cdev));

		if (0 != chr_dev_alloc)
			unregister_chrdev_region(vfs_platform_dev->devt, 1);
	}

	return status;
}

static void vfs_platform_chr_dev_unregister(struct vfs_platform_dev_data *vfs_platform_dev)
{
	PR_INFO("vfs_platform_chr_dev_unregister\n");

	if (NULL != vfs_platform_dev) {
		mutex_lock(&device_list_mutex);

		/* Destroy character device */
		list_del(&vfs_platform_dev->device_entry);

		device_destroy(vfs_platform_dev->dev_class, vfs_platform_dev->devt);
		class_destroy(vfs_platform_dev->dev_class);

		cdev_del(&(vfs_platform_dev->cdev));
		unregister_chrdev_region(vfs_platform_dev->devt, 1);

		mutex_unlock(&device_list_mutex);
	}
}

static int vfs_platform_gpio_init(struct vfs_platform_dev_data *vfs_platform_dev)
{
	int status = 0;

	PR_INFO("vfs_platform_gpio_init\n");

	if (vfs_platform_dev == NULL) {
		PR_ERR("vfs_platform_gpio_init: vfs_platform_dev is NULL\n");
		status = -EFAULT;
		goto cleanup;
	}

	status = gpio_request_one(vfs_platform_dev->drdy_pin, GPIOF_DIR_IN,
			"vfs_platform_drdy");
	if (status < 0) {
		PR_ERR("gpio_request(DRDY) is failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup;
	}

	status = gpio_request_one(vfs_platform_dev->sleep_pin, GPIOF_OUT_INIT_HIGH,
			"vfs_platform_sleep");
	if (status < 0) {
		PR_ERR("gpio_request(SLEEP)is failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup_drdy;
	}

	vfs_platform_dev->gpio_irq = gpio_to_irq(vfs_platform_dev->drdy_pin);
	if (vfs_platform_dev->gpio_irq < 0) {
		PR_ERR("gpio_to_irq failed! gpio_irq=%d\n",
				vfs_platform_dev->gpio_irq);
		status = -EBUSY;
		goto cleanup_all;
	}

	status = request_threaded_irq(vfs_platform_dev->gpio_irq, NULL,
			vfs_platform_irq, DRDY_IRQ_FLAG | IRQF_ONESHOT,
			"vfs_platform_irq", vfs_platform_dev);
	if (status < 0) {
		PR_ERR("request_irq failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup_all;
	}

	vfs_platform_dev->drdy_enabled = DRDY_IRQ_ENABLE;

	if (vfs_platform_dev->vcc_1v8 > 0) {
		status = gpio_request_one(vfs_platform_dev->vcc_1v8,
				GPIOF_OUT_INIT_HIGH, "fp_1_8");
		if (status < 0) {
			PR_ERR("gpio_request(fp_1_8)is failed! status=%d\n",
					status);
			goto cleanup_both;
		}
	}

	if (vfs_platform_dev->vcc_3v3 > 0) {
		status = gpio_request_one(vfs_platform_dev->vcc_3v3,
				GPIOF_OUT_INIT_HIGH, "fp_3_3");
		if (status < 0) {
			PR_ERR("gpio_request(fp_3_3)is failed! status=%d\n",
					status);
			goto cleanup_all;
		}
	}

	return status;
cleanup_all:
	if (vfs_platform_dev->vcc_1v8 > 0)
		gpio_free(vfs_platform_dev->vcc_1v8);
cleanup_both:
	gpio_free(vfs_platform_dev->sleep_pin);
cleanup_drdy:
	gpio_free(vfs_platform_dev->drdy_pin);
cleanup:
	return status;
}

static void vfs_platform_gpio_uninit(struct vfs_platform_dev_data *vfs_platform_dev)
{
	PR_INFO("vfs_platform_gpio_uninit\n");

	if (vfs_platform_dev != NULL) {
		free_irq(vfs_platform_dev->gpio_irq, vfs_platform_dev);
		vfs_platform_dev->drdy_enabled = DRDY_IRQ_DISABLE;
		gpio_free(vfs_platform_dev->sleep_pin);
		gpio_free(vfs_platform_dev->drdy_pin);
	}
}

static void vfs_platform_enable_irq(struct vfs_platform_dev_data *vfs_platform_dev)
{
	PR_INFO("vfs_platform_enable_irq\n");

	if (vfs_platform_dev->drdy_enabled == DRDY_IRQ_ENABLE)
		PR_DEBUG("DRDY irq already enabled\n");
	else {
		enable_irq(vfs_platform_dev->gpio_irq);
		vfs_platform_dev->drdy_enabled = DRDY_IRQ_ENABLE;
	}
}

static void vfs_platform_disable_irq(struct vfs_platform_dev_data *vfs_platform_dev)
{
	PR_INFO("vfs_platform_disable_irq\n");

	if (vfs_platform_dev->drdy_enabled == DRDY_IRQ_DISABLE)
		PR_DEBUG("DRDY irq already disabled\n");
	else {
		disable_irq_nosync(vfs_platform_dev->gpio_irq);
		vfs_platform_dev->drdy_enabled = DRDY_IRQ_DISABLE;
	}
}

static irqreturn_t vfs_platform_irq(int irq, void *context)
{
	struct vfs_platform_dev_data *vfs_platform_dev = context;
	int gpio_val;

	PR_INFO("vfs_platform_irq\n");

	/* Linux kernel is designed so that when you disable
	   an edge-triggered interrupt, and the edge happens while
	   the interrupt is disabled, the system will re-play the
	   interrupt at enable time.
	   Therefore, we are checking DRDY GPIO pin state to make sure
	   if the interrupt handler has been called actually by DRDY
	   interrupt and it's not a previous interrupt re-play */
	gpio_val = gpio_get_value(vfs_platform_dev->drdy_pin);
	PR_INFO("%s: gpio_get_value %d", __func__, gpio_val);
	if (gpio_get_value(vfs_platform_dev->drdy_pin)) {
		vfs_platform_disable_irq(vfs_platform_dev);
		vfs_platform_send_drdy_notify(vfs_platform_dev);
		PR_INFO("vfs_platform_send_drdy_notify");
	}

	return IRQ_HANDLED;
}

static int vfs_platform_send_drdy_notify(struct vfs_platform_dev_data *vfs_platform_dev)
{
	struct task_struct *t;
	struct file *efd_file = NULL;
	struct eventfd_ctx *efd_ctx = NULL;
	int status = 0;

	PR_INFO("vfs_platform_send_drdy_notify\n");

	if (vfs_platform_dev->user_pid != 0) {
		/* find the task_struct associated with userpid */
		PR_INFO("Searching task with PID=%08x\n", vfs_platform_dev->user_pid);

		rcu_read_lock();
		t = pid_task(find_pid_ns(vfs_platform_dev->user_pid, &init_pid_ns),
				PIDTYPE_PID);
		if (t == NULL) {
			rcu_read_unlock();
			PR_DEBUG("No such pid\n");
			status = -ENODEV;
			goto cleanup;
		}

		efd_file = fcheck_files(t->files, vfs_platform_dev->signal_id);
		rcu_read_unlock();

		if (efd_file == NULL) {
			PR_DEBUG("No such efd_file\n");
			status = -ENODEV;
			goto cleanup;
		}

		efd_ctx = eventfd_ctx_fileget(efd_file);
		if (efd_ctx == NULL) {
			PR_ERR("eventfd_ctx_fileget is failed\n");
			status = -ENODEV;
			goto cleanup;
		}

		/* notify DRDY eventfd to user process */
		eventfd_signal(efd_ctx, 1);

		/* Release eventfd context */
		eventfd_ctx_put(efd_ctx);
	}

cleanup:
	return status;
}

long vfs_platform_unlocked_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	PR_INFO("vfs_platform_unlocked_ioctl");
	return vfs_platform_ioctl(filp, cmd, arg, 0);
}

#ifdef CONFIG_COMPAT
long vfs_platform_compat_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	PR_INFO("vfs_platform_compat_ioctl");
	return vfs_platform_ioctl(filp, cmd, arg, 1);
}
#endif /* CONFIG_COMPAT */

long vfs_platform_ioctl(struct file *filp, unsigned int cmd, unsigned long arg,
		int compat)
{
	int status = 0;
	struct vfs_platform_dev_data *vfs_platform_dev = NULL;

	if (_IOC_TYPE(cmd) != VFSSPI_IOCTL_MAGIC) {
		PR_DEBUG("invalid magic. cmd=0x%X Received=0x%X "
				"Expected=0x%X\n",
				cmd, _IOC_TYPE(cmd), VFSSPI_IOCTL_MAGIC);
		status = -ENOTTY;
		goto cleanup;
	}

	vfs_platform_dev = filp->private_data;
	if (NULL == vfs_platform_dev) {
		PR_ERR("filp->private_data is NULL\n");
		status = -EFAULT;
		goto cleanup;
	}

	mutex_lock(&vfs_platform_dev->buffer_mutex);

	switch (cmd) {
	case VFSSPI_IOCTL_DEVICE_SUSPEND:
		{
			PR_INFO("VFSSPI_IOCTL_DEVICE_SUSPEND:");
			vfs_platform_suspend(vfs_platform_dev);
			break;
		}
	case VFSSPI_IOCTL_DEVICE_RESET:
		{
			PR_INFO("VFSSPI_IOCTL_DEVICE_RESET:");
			vfs_platform_hard_reset(vfs_platform_dev);
			break;
		}
	case VFSSPI_IOCTL_RW_SPI_MESSAGE:
	case VFSSPI_IOCTL_SET_CLK:
		{
			pr_err("vfsspi TEE driver no longer support 0x%x.\n", cmd);
			status = -EFAULT;
			break;
		}
	case VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:
		{
			PR_INFO("VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:");
			status = vfs_platform_register_drdy_signal(vfs_platform_dev, arg);
			break;
		}
	case VFSSPI_IOCTL_SET_DRDY_INT:
		{
			PR_INFO("VFSSPI_IOCTL_SET_DRDY_INT:");
			status = vfs_platform_set_drdy_int(vfs_platform_dev, arg);
			break;
		}
	case VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:
		{
			PR_INFO("VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:");
			status = vfs_platform_select_drdy_notify_type(vfs_platform_dev, arg);
			break;
		}
	case VFSSPI_IOCTL_POWER_ON:
		{
			/* Add code here to turn on sensor power, if need */
			break;
		}
	case VFSSPI_IOCTL_POWER_OFF:
		{
			/* Add code here to turn off sensor power, if need */
			break;
		}
	case VFSSPI_IOCTL_SET_SPI_CONFIGURATION:
		{
			/* Perform SPI core initialization and/or SPI clock enabling
			   from power consumption perspective */
			break;
		}
	case VFSSPI_IOCTL_RESET_SPI_CONFIGURATION:
		{
			/* Perform SPI clock disabling and/or SPI core un-initialization
			   from power consumption perspective */
			break;
		}
	default:
		{
			PR_DEBUG("Unknown cmd=0x%X\n", cmd);
			status = -EFAULT;
			break;
		}

	}
	mutex_unlock(&vfs_platform_dev->buffer_mutex);

cleanup:
	return status;
}

void vfs_platform_hard_reset(struct vfs_platform_dev_data *vfs_platform_dev)
{
	PR_INFO("vfs_platform_hard_reset\n");

	if (vfs_platform_dev != NULL) {
		spin_lock(&vfs_platform_dev->vfs_platform_lock);
		gpio_set_value(vfs_platform_dev->sleep_pin, 0);
		mdelay(1);
		gpio_set_value(vfs_platform_dev->sleep_pin, 1);
		spin_unlock(&vfs_platform_dev->vfs_platform_lock);
		mdelay(5);
	}
}

void vfs_platform_suspend(struct vfs_platform_dev_data *vfs_platform_dev)
{
	PR_INFO("vfs_platform_suspend\n");

	if (vfs_platform_dev != NULL) {
		spin_lock(&vfs_platform_dev->vfs_platform_lock);
		gpio_set_value(vfs_platform_dev->sleep_pin, 0);
		spin_unlock(&vfs_platform_dev->vfs_platform_lock);
	}
}

static int vfs_platform_register_drdy_signal(struct vfs_platform_dev_data *vfs_platform_dev,
		unsigned long arg)
{
	struct vfsspi_ioc_reg_signal user_signal;
	int status = 0;

	if (copy_from_user(&user_signal, (void *)arg,
				sizeof(user_signal)) != 0) {
		PR_ERR("copy from user failed.\n");
		status = -EFAULT;
	} else {
		vfs_platform_dev->user_pid = user_signal.user_pid;
		vfs_platform_dev->signal_id = user_signal.signal_id;
	}

	return status;
}

static int vfs_platform_set_drdy_int(struct vfs_platform_dev_data *vfs_platform_dev,
		unsigned long arg)
{
	int status = 0;
	unsigned short drdy_enable_flag;

	if (copy_from_user(&drdy_enable_flag, (void *)arg,
				sizeof(drdy_enable_flag)) != 0) {
		PR_ERR("Failed copy from user.\n");
		status = -EFAULT;
	} else {
		if (drdy_enable_flag == 0)
			vfs_platform_disable_irq(vfs_platform_dev);
		else {
			/* Workaround the issue where the system
			   misses DRDY notification to host when
			   DRDY pin was asserted before enabling
			   device.*/
			if (gpio_get_value(vfs_platform_dev->drdy_pin))
				vfs_platform_send_drdy_notify(vfs_platform_dev);
			else
				vfs_platform_enable_irq(vfs_platform_dev);
		}
	}

	return status;
}

static int vfs_platform_select_drdy_notify_type(struct vfs_platform_dev_data *vfs_platform_dev,
		unsigned long arg)
{
	int status = 0;
	vfsspi_ioc_select_drdy_notify_type_t drdy_notify_type;

	drdy_notify_type.selected_type = VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD;
	if (copy_to_user((void *)arg, &(drdy_notify_type),
				sizeof(vfsspi_ioc_select_drdy_notify_type_t)) != 0) {
		PR_ERR("copy to user failed\n");
		status = -EFAULT;
	}

	return status;
}

static ssize_t show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "snfp\n");
}

static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static ssize_t vfs_platform_wakeup_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count) {
	struct vfs_platform_dev_data *vfs_platform_dev = dev_get_drvdata(dev);
	int wakeup;

	pr_err("%s: wakeup %s\n", __func__, buf);
	wakeup = !strncmp(buf, "true", 4);
	if (vfs_platform_dev->wakeup_source != wakeup) {
		irq_set_irq_wake(vfs_platform_dev->gpio_irq, wakeup);
		vfs_platform_dev->wakeup_source = wakeup;
		pr_err("%s: wakeup %d\n", __func__, wakeup);
	}
	return count;
}

static DEVICE_ATTR(wakeup, S_IWUSR, NULL, vfs_platform_wakeup_store);

int vfs_platform_open(struct inode *inode, struct file *filp)
{
	struct vfs_platform_dev_data *vfs_platform_dev = NULL;
	int status = -ENXIO;

	pr_debug("vfs_platform_open\n");

	mutex_lock(&device_list_mutex);
	list_for_each_entry(vfs_platform_dev, &device_list, device_entry) {
		if (vfs_platform_dev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (vfs_platform_dev->is_opened != 0) {
			status = -EBUSY;
		} else {
			vfs_platform_dev->user_pid = 0;
			vfs_platform_dev->is_opened = 1;
			filp->private_data = vfs_platform_dev;
			nonseekable_open(inode, filp);
		}
	}

	mutex_unlock(&device_list_mutex);

	return status;
}

int vfs_platform_release(struct inode *inode, struct file *filp)
{
	struct vfs_platform_dev_data *vfs_platform_dev = NULL;
	int status = 0;

	pr_debug("vfs_platform_release\n");

	mutex_lock(&device_list_mutex);
	vfs_platform_dev = filp->private_data;
	if (vfs_platform_dev != NULL) {
		filp->private_data = NULL;
		vfs_platform_dev->is_opened = 0;

	}
	mutex_unlock(&device_list_mutex);

	return status;
}

int vfs_platform_probe(struct platform_device *pdev)
{
	int status = 0;
	struct vfs_platform_dev_data *vfs_platform_dev = NULL;

	PR_INFO("vfs_platform_probe\n");
	if (!asus_match_hw_id(pdev->dev.of_node, 0)) {
		pr_info("probe failed, returned.\n");
		return -ENODEV;
	}

	status = vfs_platform_dev_init(pdev, &vfs_platform_dev);
	if (0 != status) {
		PR_ERR("vfs_platform_dev_init failed! status= %d\n", status);
		goto cleanup;
	}
	status = vfs_platform_parse_dt(&pdev->dev, vfs_platform_dev);
	if (status) {
		PR_ERR("vfs_platform_parse_dt failed! status= %d\n", status);
		goto cleanup;
	}

	status = vfs_platform_gpio_init(vfs_platform_dev);
	if (0 != status) {
		PR_ERR("vfs_platform_gpio_init failed! status= %d\n", status);
		vfs_platform_dev_uninit(vfs_platform_dev);
		goto cleanup;
	}

	status = device_create_file(&(pdev->dev),
			&dev_attr_name);
	if (status != 0)
		pr_warn("Failed to crate name %d\n", status);

	status = device_create_file(&(pdev->dev),
			&dev_attr_wakeup);
	if (status != 0)
		pr_warn("Failed to crate wakeup %d\n", status);
	vfs_platform_dev->wakeup_source = 0;

	PR_INFO("vfs_platform_probe succeeded\n");

cleanup:
	return status;
}

int vfs_platform_remove(struct platform_device *pdev)
{
	int status = 0;
	struct vfs_platform_dev_data *vfs_platform_dev = NULL;

	PR_INFO("vfs_platform_remove\n");

	vfs_platform_dev = platform_get_drvdata(pdev);

	if (NULL != vfs_platform_dev) {
		vfs_platform_gpio_uninit(vfs_platform_dev);

		vfs_platform_dev_uninit(vfs_platform_dev);
	}


	return status;
}

static int __init vfs_platform_init(void)
{
	int status = 0;

	PR_INFO("vfs_platform_init\n");

	status = platform_driver_register(&vfs_platform);
	if (status < 0)
		PR_ERR("spi_register_driver is failed with status %d", status);
	else
		PR_INFO("vfs_platform_init succeeded\n");


	return status;
}

static void __exit vfs_platform_exit(void)
{
	platform_driver_unregister(&vfs_platform);
	PR_INFO("vfs_platform_exit\n");

}

module_init(vfs_platform_init);
module_exit(vfs_platform_exit);

MODULE_DESCRIPTION("Validity FPS sensor");
MODULE_LICENSE("GPL");
