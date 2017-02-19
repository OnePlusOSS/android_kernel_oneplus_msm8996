/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <soc/qcom/scm.h>

#include <linux/wakelock.h>
#include <linux/input.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include <linux/project_info.h>

static unsigned int ignor_home_for_ESD = 0;
module_param(ignor_home_for_ESD, uint, S_IRUGO | S_IWUSR);

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250
#define FPC_TTW_HOLD_TIME 1000

#define ONEPLUS_EDIT  //Onplus modify for msm8996 platform and 15801 HW

struct fpc1020_data {
	struct device *dev;
    struct wake_lock ttw_wl;
	int irq_gpio;
	int rst_gpio;
	int irq_num;
	struct mutex lock;
	bool prepared;

	struct pinctrl         *ts_pinctrl;
	struct pinctrl_state   *gpio_state_active;
	struct pinctrl_state   *gpio_state_suspend;

	#ifdef ONEPLUS_EDIT
    int EN_VDD_gpio;
    int id0_gpio;
    int id1_gpio;
    struct input_dev	*input_dev;
    int screen_state;//1: on 0:off
	#endif
	#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
    #endif
};

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		*gpio = rc;
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_info(dev, "%s - gpio: %d\n", label, *gpio);
	return 0;
}
#ifndef ONEPLUS_EDIT
/* -------------------------------------------------------------------- */
static int fpc1020_pinctrl_init(struct fpc1020_data *fpc1020)
{
	int ret = 0;
	struct device *dev = fpc1020->dev;

	fpc1020->ts_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(fpc1020->ts_pinctrl)) {
		dev_err(dev, "Target does not use pinctrl\n");
		ret = PTR_ERR(fpc1020->ts_pinctrl);
		goto err;
	}

	fpc1020->gpio_state_active =
		pinctrl_lookup_state(fpc1020->ts_pinctrl, "pmx_fp_active");
	if (IS_ERR_OR_NULL(fpc1020->gpio_state_active)) {
		dev_err(dev, "Cannot get active pinstate\n");
		ret = PTR_ERR(fpc1020->gpio_state_active);
		goto err;
	}

	fpc1020->gpio_state_suspend =
		pinctrl_lookup_state(fpc1020->ts_pinctrl, "pmx_fp_suspend");
	if (IS_ERR_OR_NULL(fpc1020->gpio_state_suspend)) {
		dev_err(dev, "Cannot get sleep pinstate\n");
		ret = PTR_ERR(fpc1020->gpio_state_suspend);
		goto err;
	}

	return 0;
err:
	fpc1020->ts_pinctrl = NULL;
	fpc1020->gpio_state_active = NULL;
	fpc1020->gpio_state_suspend = NULL;
	return ret;
}

/* -------------------------------------------------------------------- */
static int fpc1020_pinctrl_select(struct fpc1020_data *fpc1020, bool on)
{
	int ret = 0;
	struct pinctrl_state *pins_state;
	struct device *dev = fpc1020->dev;

	pins_state = on ? fpc1020->gpio_state_active : fpc1020->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(fpc1020->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(dev, "can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(dev, "not a valid '%s' pinstate\n",
			on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return ret;
}
#endif
/*
static int hw_reset(struct  fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;

	int rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);
exit:
	return rc;
}
*/
static int hw_reset(struct  fpc1020_data *fpc1020)
{
    int irq_gpio;
	struct device *dev = fpc1020->dev;
	int counter = 2;

	gpio_set_value(fpc1020->EN_VDD_gpio, 0);
    mdelay(3);
	gpio_set_value(fpc1020->EN_VDD_gpio, 1);
	mdelay(3);
	//gpio_direction_output(fpc1020->EN_VDD_gpio,1);

	while (counter) {
		counter--;

		gpio_set_value(fpc1020->rst_gpio, 1);
		udelay(FPC1020_RESET_HIGH1_US);

		gpio_set_value(fpc1020->rst_gpio, 0);
		udelay(FPC1020_RESET_LOW_US);

		gpio_set_value(fpc1020->rst_gpio, 1);
		udelay(FPC1020_RESET_HIGH2_US);

		irq_gpio = gpio_get_value(fpc1020->irq_gpio);
		dev_err(dev, "IRQ after reset %d\n", irq_gpio);
		if (irq_gpio) {
			//printk(KERN_INFO "%s OK !\n", __func__);
			counter = 0;
		} else {
			dev_err(dev, "%s timed out,retrying ...\n",
				__func__);

			udelay(1250);
		}
	}
	return 0;
}
static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(fpc1020);
	else
		return -EINVAL;
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);


/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device* device,
			     struct device_attribute* attribute,
			     char* buffer)
{
	struct fpc1020_data* fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}


/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device* device,
			     struct device_attribute* attribute,
			     const char* buffer, size_t count)
{
	struct fpc1020_data* fpc1020 = dev_get_drvdata(device);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);
extern void int_touch(void);
extern struct completion key_cm;
extern bool virtual_key_enable;
extern bool s1302_is_keypad_stopped(void);

bool key_home_pressed = false;
EXPORT_SYMBOL(key_home_pressed);

static ssize_t report_home_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
    unsigned long time;
	bool ignore_keypad;

	if (s1302_is_keypad_stopped() || virtual_key_enable)
		ignore_keypad = true;
	else
		ignore_keypad = false;

	if(ignor_home_for_ESD)
		return -EINVAL;
	if (!strncmp(buf, "down", strlen("down")))
	{
        if(ignore_keypad){
                key_home_pressed = true;
        }else{
            input_report_key(fpc1020->input_dev,
                            KEY_HOME, 1);
            input_sync(fpc1020->input_dev);
        }
	}
	else if (!strncmp(buf, "up", strlen("up")))
	{
        if(ignore_keypad){
                key_home_pressed = false;
        }else{
            input_report_key(fpc1020->input_dev,
                            KEY_HOME, 0);
            input_sync(fpc1020->input_dev);
        }
	}
	else
		return -EINVAL;
    if(ignore_keypad){
        if(!key_home_pressed){
            reinit_completion(&key_cm);
            time = wait_for_completion_timeout(&key_cm,msecs_to_jiffies(60));
            if (!time)
                int_touch();
        }else{
            int_touch();
        }
    }
	return count;
}
static DEVICE_ATTR(report_home, S_IWUSR, NULL, report_home_set);

static ssize_t update_info_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	//struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "n", strlen("n")))
	{
		push_component_info(FINGERPRINTS,"N/A" , "N/A");
	}

	return count;
}
static DEVICE_ATTR(update_info, S_IWUSR, NULL, update_info_set);

static ssize_t screen_state_get(struct device* device,
			     struct device_attribute* attribute,
			     char* buffer)
{
	struct fpc1020_data* fpc1020 = dev_get_drvdata(device);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", fpc1020->screen_state);
}

static DEVICE_ATTR(screen_state, S_IRUSR , screen_state_get, NULL);

static struct attribute *attributes[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_irq.attr,
	&dev_attr_report_home.attr,
	&dev_attr_update_info.attr,
	&dev_attr_screen_state.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

int fpc1020_input_init(struct fpc1020_data *fpc1020)
{
	int error = 0;


	dev_dbg(fpc1020->dev, "%s\n", __func__);

	fpc1020->input_dev = input_allocate_device();

	if (!fpc1020->input_dev) {
		dev_err(fpc1020->dev, "Input_allocate_device failed.\n");
		error  = -ENOMEM;
	}

	if (!error) {
		fpc1020->input_dev->name = "fpc1020";

		/* Set event bits according to what events we are generating */
		set_bit(EV_KEY, fpc1020->input_dev->evbit);

        set_bit(KEY_POWER, fpc1020->input_dev->keybit);
        set_bit(KEY_F2, fpc1020->input_dev->keybit);
        set_bit(KEY_HOME, fpc1020->input_dev->keybit);

		/* Register the input device */
		error = input_register_device(fpc1020->input_dev);


		if (error) {
			dev_err(fpc1020->dev, "Input_register_device failed.\n");
			input_free_device(fpc1020->input_dev);
			fpc1020->input_dev = NULL;
		}
	}

	return error;
}

/* -------------------------------------------------------------------- */
void fpc1020_input_destroy(struct fpc1020_data *fpc1020)
{
	dev_dbg(fpc1020->dev, "%s\n", __func__);

	if (fpc1020->input_dev != NULL)
		input_free_device(fpc1020->input_dev);
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct fpc1020_data *fpc1020 = container_of(self, struct fpc1020_data, fb_notif);

	if(FB_EARLY_EVENT_BLANK != event && FB_EVENT_BLANK != event)
	return 0;
	if((evdata) && (evdata->data) && (fpc1020)) {
		blank = evdata->data;
		if( *blank == FB_BLANK_UNBLANK && (event == FB_EARLY_EVENT_BLANK )) {
			dev_err(fpc1020->dev, "%s screen on\n", __func__);
			fpc1020->screen_state = 1;
			sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_screen_state.attr.name);

		} else if( *blank == FB_BLANK_POWERDOWN && (event == FB_EARLY_EVENT_BLANK/*FB_EVENT_BLANK*/ )) {
            dev_err(fpc1020->dev, "%s screen off\n", __func__);
		    fpc1020->screen_state = 0;
			sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_screen_state.attr.name);
		}
	}
	return 0;
}
#endif

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;
	//dev_err(fpc1020->dev, "%s\n", __func__);

	/* Make sure 'wakeup_enabled' is updated before using it
	** since this is interrupt context (other thread...) */
	//smp_rmb();
/*
	if (fpc1020->wakeup_enabled ) {
		wake_lock_timeout(&fpc1020->ttw_wl, msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}
*/
	wake_lock_timeout(&fpc1020->ttw_wl, msecs_to_jiffies(FPC_TTW_HOLD_TIME));
    //dev_err(fpc1020->dev, "%s before sysfs_notify\n", __func__);

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);
	//dev_err(fpc1020->dev, "%s after sysfs_notify\n", __func__);

	return IRQ_HANDLED;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	int irqf;
	struct device_node *np = dev->of_node;

	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	printk(KERN_INFO "%s\n", __func__);

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,irq-gpio",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;

	rc = gpio_direction_input(fpc1020->irq_gpio);
	
	if (rc) {
		dev_err(fpc1020->dev,
			"gpio_direction_input (irq) failed.\n");
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,reset-gpio",
			&fpc1020->rst_gpio);
	if (rc)
		goto exit;

    #ifdef ONEPLUS_EDIT
    rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_id0",
			&fpc1020->id0_gpio);
	if(gpio_is_valid(fpc1020->id0_gpio))
    {
        dev_err(dev, "%s: gpio_is_valid(fpc1020->id0_gpio=%d)\n", __func__,fpc1020->id0_gpio);
        gpio_direction_input(fpc1020->id0_gpio);
    }

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_id1",
			&fpc1020->id1_gpio);
	if(gpio_is_valid(fpc1020->id1_gpio))
    {
        dev_err(dev, "%s: gpio_is_valid(fpc1020->id1_gpio=%d)\n", __func__,fpc1020->id1_gpio);
        gpio_direction_input(fpc1020->id1_gpio);
    }

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_1V8_EN",
			&fpc1020->EN_VDD_gpio);
	if (rc)
		goto exit;
    gpio_direction_output(fpc1020->EN_VDD_gpio,1);

    #else
	rc = fpc1020_pinctrl_init(fpc1020);
	if (rc)
		goto exit;

	rc = fpc1020_pinctrl_select(fpc1020, true);
	if (rc)
		goto exit;
		
    #endif
    rc = fpc1020_input_init(fpc1020);
    if (rc)
		goto exit;

    #if defined(CONFIG_FB)
	fpc1020->fb_notif.notifier_call = fb_notifier_callback;
	rc = fb_register_client(&fpc1020->fb_notif);
	if(rc)
		dev_err(fpc1020->dev, "Unable to register fb_notifier: %d\n", rc);
    fpc1020->screen_state = 1;
    #endif

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}
	dev_info(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should not be wakeable */
	//disable_irq_wake( gpio_to_irq( fpc1020->irq_gpio ) );

	enable_irq_wake( gpio_to_irq( fpc1020->irq_gpio ) );
	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");
	device_init_wakeup(fpc1020->dev, 1);

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}
	
    #if 0
	rc = gpio_direction_output(fpc1020->rst_gpio, 1);

	if (rc) {
		dev_err(fpc1020->dev,
			"gpio_direction_output (reset) failed.\n");
		goto exit;
	}

	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH1_US);
	
	gpio_set_value(fpc1020->rst_gpio, 0);
	udelay(FPC1020_RESET_LOW_US);
	
	gpio_set_value(fpc1020->rst_gpio, 1);
	udelay(FPC1020_RESET_HIGH2_US);
    #endif
    /**
    *           ID0(GPIO25)   ID1(GPIO143)
    *   O-film   0            0
    *   DT       0            1
    *   CT       1            0
    */
	if(!gpio_get_value(fpc1020->id0_gpio) && !gpio_get_value(fpc1020->id1_gpio))
        push_component_info(FINGERPRINTS,"fpc1245" , "FPC(OF)");
    else if(!gpio_get_value(fpc1020->id0_gpio) && gpio_get_value(fpc1020->id1_gpio))
        push_component_info(FINGERPRINTS,"fpc1245" , "FPC(Primax)");
    else if(gpio_get_value(fpc1020->id0_gpio) && !gpio_get_value(fpc1020->id1_gpio))
        push_component_info(FINGERPRINTS,"fpc1245" , "FPC(CT)");
    else
        push_component_info(FINGERPRINTS,"fpc1245" , "FPC");

	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}


static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name		= "fpc1020",
		.owner		= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe = fpc1020_probe,
};
module_platform_driver(fpc1020_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
