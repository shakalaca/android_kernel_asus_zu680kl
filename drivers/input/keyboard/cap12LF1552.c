#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c/cap12LF1552.h>

DEVICE_ATTR(virtual_keys, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(irq_status, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(firmware_version, 0444, show_attrs_handler, NULL);
DEVICE_ATTR(threshold1, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(threshold2, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(standby_channel, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sampling_configuration, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(queue_control, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(addititonal_sample_cap_selection, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(background_noise, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensitivity, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(calibration, 0644, show_attrs_handler, NULL);

static struct attribute *cap12LF1552_attrs[] = {
    &dev_attr_virtual_keys.attr,
    &dev_attr_irq_status.attr,
    &dev_attr_firmware_version.attr,
    &dev_attr_threshold1.attr,
    &dev_attr_threshold2.attr,
    &dev_attr_standby_channel.attr,
    &dev_attr_sampling_configuration.attr,
    &dev_attr_queue_control.attr,
    &dev_attr_addititonal_sample_cap_selection.attr,
    &dev_attr_background_noise.attr,
    &dev_attr_sensitivity.attr,
    &dev_attr_calibration.attr,
    NULL
};

static s32 cap12LF1552_read_reg(struct i2c_client *client, u8 command) {
    return i2c_smbus_read_byte_data(client, command);
}

static s32 cap12LF1552_write_reg(struct i2c_client *client, u8 command, u8 value) {
    return i2c_smbus_write_byte_data(client, command, value);
}

static int select_interrupt_pin_status(struct device *dev) {
    struct pinctrl *int_pinctrl;
    int_pinctrl = devm_pinctrl_get_select(dev, "cap12LF1552_default");
    if (IS_ERR(int_pinctrl)) {
        CAP_DEBUG("fail to get pin status\n");
        int_pinctrl = NULL;
        return -1;
    }
    return 0;
}

static int cap12LF1552_init_sensor(struct i2c_client *client) {
    unsigned int rc = 0;

    cap12LF1552_write_reg(client, SLEEP_CONTROL, 0x00);
    cap12LF1552_write_reg(client, STANDBY_CHANNEL, 0x07);
    cap12LF1552_write_reg(client, SAMPLE_SELECTION, 0x07);
    cap12LF1552_write_reg(client, MAX_DURATION, 0x30);

    rc = cap12LF1552_read_reg(client, REVISION);
    if (rc != 0x10) return -ENXIO; // not CAP12LF1552
    rc = cap12LF1552_read_reg(client, SENSOR_STATUS);
    if (rc == 0x3C) return -ENXIO; // IQS263
    return rc;
}
static irqreturn_t cap12LF1552_interrupt_handler(int irq, void *dev) {
    struct cap12LF1552_data *data = i2c_get_clientdata(dev);
    queue_delayed_work(data->cap_wq, &data->work, 0);
    return IRQ_HANDLED;
}

static int cap12LF1552_config_irq(struct i2c_client *client) {
    int ret = 0;
    struct cap12LF1552_data *data = i2c_get_clientdata(client);

    if (gpio_is_valid(data->det_gpio)) {
        ret = gpio_request(data->det_gpio, data->det_gpio_name);
        if (ret) {
            CAP_DEBUG("fail to request GPIO %d: %d\n", data->det_gpio, ret);
            goto config_fail;
        }

        ret = gpio_direction_input(data->det_gpio);
        if (ret) {
            CAP_DEBUG("Failed to configure output direction for GPIO\n");
            goto config_fail;
        }
        client->irq = gpio_to_irq(data->det_gpio);
        ret = request_irq(client->irq, cap12LF1552_interrupt_handler,
              IRQF_TRIGGER_FALLING, data->det_gpio_name, client);
        if (ret) {
            CAP_DEBUG("IRQ %d busy error %d\n", client->irq, ret);
            goto config_fail;
        }
    } else {
        CAP_DEBUG("gpio is invalid\n");
        ret = -EPERM;
        goto config_fail;
    }

    return 0;

config_fail:
    return ret;
}
#ifdef CONFIG_FB
static int cap12LF1552_fb_notifier_cb(struct notifier_block *self,
                unsigned long event, void *data) {
        int *transition;
        struct fb_event *evdata = data;
        struct cap12LF1552_data* cap_data = container_of(self, struct cap12LF1552_data,
                                           fb_notifier);

        if (evdata && evdata->data && cap_data) {
                if (event == FB_EVENT_BLANK) {
                        transition = evdata->data;
                        if (*transition == FB_BLANK_POWERDOWN) {
                                cap12LF1552_suspend(&cap_data->client->dev);
                        }
                        else if (*transition == FB_BLANK_UNBLANK) {
                                cap12LF1552_resume(&cap_data->client->dev);
                        }
                }
        }

        return 0;
}
#endif

static void cap12LF1552_music_settings(struct work_struct *work) {
    struct cap12LF1552_data *data =
        container_of((struct delayed_work *)work, struct cap12LF1552_data, music_work);
    mutex_lock(&cap_mtx);
    if (change_settings) {
        cap12LF1552_write_reg(data->client, SAMPLE_CONFIG, 0x07);
        cap12LF1552_write_reg(data->client, SENSOR1_THRESHOLD, 0xC8);
        cap12LF1552_write_reg(data->client, SENSOR2_THRESHOLD, 0xC8);
        schedule_delayed_work(&data->calibration_work, msecs_to_jiffies(5000));
        CAP_DEBUG("Enter Workaround\n");
    } else {
        cap12LF1552_write_reg(data->client, SAMPLE_CONFIG, 0x0A);
        cap12LF1552_write_reg(data->client, SENSOR1_THRESHOLD, 0x96);
        cap12LF1552_write_reg(data->client, SENSOR2_THRESHOLD, 0x96);
        cancel_delayed_work_sync(&data->calibration_work);
        CAP_DEBUG("Leave Workaround\n");
    }
    mutex_unlock(&cap_mtx);
}

void notify_speaker_status(int music_on) {
    change_settings = music_on;
    if (music_on)
        schedule_delayed_work(&cap_data->music_work, 0);
    else
        schedule_delayed_work(&cap_data->music_work, msecs_to_jiffies(2000));
}
EXPORT_SYMBOL(notify_speaker_status);

static void calibration_work_function(struct work_struct *work) {
    struct cap12LF1552_data *data =
        container_of((struct delayed_work *)work, struct cap12LF1552_data, calibration_work);
    mutex_lock(&cap_mtx);
    cap12LF1552_write_reg(data->client, SAMPLE_CONFIG, 0x07);
    CAP_DEBUG("calibration\n");
    schedule_delayed_work(&data->calibration_work, msecs_to_jiffies(5000));
    mutex_unlock(&cap_mtx);
}

static void cap12LF1552_work_function(struct work_struct *work) {
    int val, change;
    struct cap12LF1552_data *data =
        container_of((struct delayed_work *)work, struct cap12LF1552_data, work);

    val = cap12LF1552_read_reg(data->client, SENSOR_STATUS);
    if ((val & BACK) && (val & APP_SWITCH)) {
        if (change_settings) {
            cap12LF1552_write_reg(data->client, SAMPLE_CONFIG, 0x07);
        }
        else {
            cap12LF1552_write_reg(data->client, SAMPLE_CONFIG, 0x0A);
        }
        return;
    }
    change = val ^ prev_val;
    if ((change & BACK) && (val & BACK)) {
        input_report_key(data->input_back, KEY_BACK, 1);
        input_sync(data->input_back);
        CAP_DEBUG("Press back key.\n");
        if (change_settings)
            cancel_delayed_work_sync(&data->calibration_work);
    } else if ((change & APP_SWITCH) && (val & APP_SWITCH)) {
        input_report_key(data->input_app_switch, KEY_APP_SWITCH, 1);
        input_sync(data->input_app_switch);
        CAP_DEBUG("Press recent apps key.\n");
        if (change_settings)
            cancel_delayed_work_sync(&data->calibration_work);
    } else {
        //key release
        if ((change & BACK) && !(val & BACK)) {
            input_report_key(data->input_back, KEY_BACK, 0);
            input_sync(data->input_back);
            CAP_DEBUG("Release back key.\n");
        }
        else if ((change & APP_SWITCH) && !(val & APP_SWITCH)) {
            input_report_key(data->input_app_switch, KEY_APP_SWITCH, 0);
            input_sync(data->input_app_switch);
            CAP_DEBUG("Release recent apps key.\n");
        }
        if (change_settings)
            schedule_delayed_work(&data->calibration_work, msecs_to_jiffies(5000));
    }
    prev_val = val;
}

static ssize_t show_attrs_handler(struct device *dev,
    struct device_attribute *devattr, char *buf) {

    struct i2c_client *client = to_i2c_client(dev);
    const char *attr_name = devattr->attr.name;
    int ret = -1;
    CAP_DEBUG("devattr->attr->name: %s\n", devattr->attr.name);
    mutex_lock(&cap_mtx);
    if (!strcmp(attr_name, dev_attr_virtual_keys.attr.name)) {
        ret = cap12LF1552_read_reg(client, SLEEP_CONTROL);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_irq_status.attr.name)) {
        mutex_unlock(&cap_mtx);
        if (irq_enabled)
            return snprintf(buf, 9, "enable\n");
        else
            return snprintf(buf, 9, "disable\n");
    } else if (!strcmp(attr_name, dev_attr_firmware_version.attr.name)) {
        mutex_unlock(&cap_mtx);
        ret = cap12LF1552_read_reg(client, REVISION);
        return snprintf(buf, 8, "0x%X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_threshold1.attr.name)) {
        ret = cap12LF1552_read_reg(client, 0x02);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_threshold2.attr.name)) {
        ret = cap12LF1552_read_reg(client, 0x03);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_standby_channel.attr.name)) {
        ret = cap12LF1552_read_reg(client, 0x05);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_sampling_configuration.attr.name)) {
        ret = cap12LF1552_read_reg(client, 0x06);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_queue_control.attr.name)) {
        ret = cap12LF1552_read_reg(client, 0x07);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_addititonal_sample_cap_selection.attr.name)) {
        mutex_unlock(&cap_mtx);
        ret = cap12LF1552_read_reg(client, 0x08);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_background_noise.attr.name)) {
        ret = cap12LF1552_read_reg(client, 0x09);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_sensitivity.attr.name)) {
        ret = cap12LF1552_read_reg(client, 0x0A);
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "0x%02X\n", ret);
    } else if (!strcmp(attr_name, dev_attr_calibration.attr.name)) {
        mutex_unlock(&cap_mtx);
        return snprintf(buf, 8, "%d\n", change_settings);
    }

    return 0;
}

static ssize_t store_attrs_handler(struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count) {

    int ret;
    struct i2c_client *client = to_i2c_client(dev);
    const char *attr_name = devattr->attr.name;
    unsigned long value;

    if (kstrtoul(buf, 16, &value)) return -EINVAL;
    CAP_DEBUG("devattr->attr->name: %s, value: 0x%lX\n", devattr->attr.name, value);

    mutex_lock(&cap_mtx);
    if (!strcmp(attr_name, dev_attr_virtual_keys.attr.name)) {
        ret = cap12LF1552_write_reg(client, SLEEP_CONTROL, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_irq_status.attr.name)) {
        if (!value) {
            disable_irq(client->irq);
            irq_enabled = 0;
        }
        else {
            enable_irq(client->irq);
            irq_enabled = 1;
        }
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_threshold1.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x02, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_threshold2.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x03, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_standby_channel.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x05, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_sampling_configuration.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x06, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_queue_control.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x07, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_addititonal_sample_cap_selection.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x08, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_background_noise.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x09, value);
        mutex_unlock(&cap_mtx);
    } else if (!strcmp(attr_name, dev_attr_sensitivity.attr.name)) {
        ret = cap12LF1552_write_reg(client, 0x0A, value);
        mutex_unlock(&cap_mtx);
    }

    return strnlen(buf, count);
}

static int cap12LF1552_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct cap12LF1552_data *data;
    int ret = 0;
    enum of_gpio_flags flags;
    struct device_node *np = client->dev.of_node;
    struct input_dev *input_back;
    struct input_dev *input_app_switch;
    prev_val = 0;

    BACK = (g_ASUS_hwID >= 4 ? 1 : 2);
    APP_SWITCH = (g_ASUS_hwID >= 4 ? 2 : 4);

    data = kzalloc(sizeof(struct cap12LF1552_data), GFP_KERNEL);
    input_back = input_allocate_device();
    input_app_switch = input_allocate_device();
    if (!data || !input_back || !input_app_switch) {
        ret = -ENOMEM;
        CAP_DEBUG("Failed to allocate data or device\n");
        goto probe_failed;
    }

    data->input_back = input_back;
    input_set_drvdata(input_back, data);
    data->input_app_switch = input_app_switch;
    input_set_drvdata(input_app_switch, data);

    input_back->name = "virtual_keys_back";
    input_app_switch->name = "virtual_keys_app_switch";

    input_set_capability(input_back, EV_KEY, KEY_BACK);
    input_set_capability(input_app_switch, EV_KEY, KEY_APP_SWITCH);

    ret = input_register_device(input_back);
    if (ret) {
       CAP_DEBUG("Unable to register input device: %d\n", ret);
       goto probe_failed;
    }
    ret = input_register_device(input_app_switch);
    if (ret) {
       CAP_DEBUG("Unable to register input device: %d\n", ret);
       goto probe_failed;
    }

    data->cap_wq = create_singlethread_workqueue("cap_wq");
    if(!data->cap_wq) {
        CAP_DEBUG("Failed to create singlethread workqueue\n");
        ret = -ENOMEM;
        goto probe_failed;
    }

    INIT_DELAYED_WORK(&data->work, cap12LF1552_work_function);
    INIT_DELAYED_WORK(&data->music_work, cap12LF1552_music_settings);
    INIT_DELAYED_WORK(&data->calibration_work, calibration_work_function);

    data->client = client;
    i2c_set_clientdata(client, data);
    select_interrupt_pin_status(&client->dev);
    data->det_gpio = of_get_named_gpio_flags(np, "det_gpio", 0, &flags);
    data->client->flags = 0;
    data->det_gpio_name = "cap12LF1552_irq";
    strlcpy(data->client->name, "cap12LF1552", I2C_NAME_SIZE);

    data->attrs.attrs = cap12LF1552_attrs;
    ret = sysfs_create_group(&data->client->dev.kobj, &data->attrs);
    if (ret) {
        CAP_DEBUG("Create the sysfs group failed\n");
        goto probe_failed;
    }

    ret = cap12LF1552_init_sensor(data->client);
    if (ret == -ENXIO) {
        CAP_DEBUG("This is not CAP12LF1552 cap sensor\n");
        goto sensor_doesnot_match;
    }

    ret = cap12LF1552_config_irq(data->client);
    if (ret) {
        CAP_DEBUG("config IRQ failed\n");
        irq_enabled = 0;
        goto probe_failed;
    } else {
        irq_enabled = 1;
    }
#ifdef CONFIG_FB
    data->fb_notifier.notifier_call = cap12LF1552_fb_notifier_cb;
    fb_register_client(&data->fb_notifier);
#endif
    cap_data = data;
    CAP_DEBUG("probe successfully\n.");
    return 0;

probe_failed:
sensor_doesnot_match:
    input_free_device(input_app_switch);
    input_free_device(input_back);
    kfree(data);
    return ret;
}

static int cap12LF1552_suspend(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    struct cap12LF1552_data *cap_data = dev_get_drvdata(dev);
    cancel_delayed_work_sync(&cap_data->work);
    cap12LF1552_write_reg(client, SLEEP_CONTROL, 0x0F);
    disable_irq(client->irq);
    //CAP_DEBUG("vk suspend\n");
    return 0;
}

static int cap12LF1552_resume(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    cap12LF1552_write_reg(client, SLEEP_CONTROL, 0x00);
    enable_irq(client->irq);
    //CAP_DEBUG("vk resume\n");
    return 0;
}

static const struct of_device_id cap12LF1552_dt_ids[] = {
    { .compatible ="microchip,cap12LF1552", },
    {}
};

MODULE_DEVICE_TABLE(of, cap12LF1552_dt_ids);

static const struct i2c_device_id cap12LF1552_id[] = {
    { "cap12LF1552", 0 },
    {}
};

MODULE_DEVICE_TABLE(i2c, cap12LF1552_id);

static struct i2c_driver cap12LF1552_driver = {
    .driver = {
        .name  = "cap12LF1552",
        .owner = THIS_MODULE,
        .of_match_table = cap12LF1552_dt_ids,
        .pm = &cap1106_pm_ops,
    },
    .probe     = cap12LF1552_probe,
    .id_table  = cap12LF1552_id,
};

module_i2c_driver(cap12LF1552_driver);

MODULE_DESCRIPTION("Microchip cap12LF1552 Driver");
MODULE_LICENSE("GPL");
