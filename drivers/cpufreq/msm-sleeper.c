/*
 * msm-sleeper.c
 *
 * Copyright (C) 2016 Aaron Segaert <asegaert@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/fb.h>
#include <linux/platform_device.h>

#define MSM_SLEEPER "msm_sleeper"
#define MSM_SLEEPER_MAJOR_VERSION	5
#define MSM_SLEEPER_MINOR_VERSION	0
#define MSM_SLEEPER_ENABLED		1

struct msm_sleeper_data {
	unsigned int enabled;
	struct notifier_block notif;
	struct work_struct suspend_work;
	struct work_struct resume_work;
} sleeper_data = {
	.enabled = MSM_SLEEPER_ENABLED
};

static struct workqueue_struct *sleeper_wq;

static void msm_sleeper_suspend(struct work_struct *work)
{
	int cpu;
	
	for_each_possible_cpu(cpu) {
		if (cpu > 1 && cpu_online(cpu)) {
			cpu_down(cpu);
		}
	}
}

static void __ref msm_sleeper_resume(struct work_struct *work)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		if (cpu && cpu_is_offline(cpu)) {
			cpu_up(cpu);
		}
	}
}

static int fb_notifier_callback(struct notifier_block *this,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (!sleeper_data.enabled)
		return NOTIFY_OK;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		switch (*blank) {
			case FB_BLANK_UNBLANK:
				//display on
				queue_work_on(0, sleeper_wq, &sleeper_data.resume_work);
				break;
			case FB_BLANK_POWERDOWN:
			case FB_BLANK_HSYNC_SUSPEND:
			case FB_BLANK_VSYNC_SUSPEND:
			case FB_BLANK_NORMAL:
				//display off
				queue_work_on(0, sleeper_wq, &sleeper_data.suspend_work);
				break;
		}
	}

	return NOTIFY_OK;
}

static ssize_t show_enable(struct device *dev,
				   struct device_attribute *msm_sleeper_attrs, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", sleeper_data.enabled);
}

static ssize_t __ref store_enable(struct device *dev,
				    struct device_attribute *msm_sleeper_attrs,
				    const char *buf, size_t count)
{
	int ret, cpu;
	unsigned long val;
	
	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	
	sleeper_data.enabled = val;

	if (!sleeper_data.enabled) {
		for_each_possible_cpu(cpu) {
			if (cpu_is_offline(cpu)) {
				cpu_up(cpu);
			}
		}
	}

	return count;
}

static DEVICE_ATTR(enabled, (S_IWUSR|S_IRUGO), show_enable, store_enable);

static struct attribute *msm_sleeper_attrs[] = {
	&dev_attr_enabled.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = msm_sleeper_attrs,
};

static struct platform_device msm_sleeper_device = {
	.name = MSM_SLEEPER,
	.id = -1,
};

static int msm_sleeper_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("msm-sleeper version %d.%d\n",
		MSM_SLEEPER_MAJOR_VERSION,
		MSM_SLEEPER_MINOR_VERSION);

	sleeper_wq = alloc_workqueue("msm_sleeper_wq",
				WQ_HIGHPRI | WQ_FREEZABLE, 0);
	if (!sleeper_wq) {
		ret = -ENOMEM;
		goto err_out;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	if (ret) {
		ret = -EINVAL;
		goto err_dev;
	}	

	sleeper_data.notif.notifier_call = fb_notifier_callback;
	if (fb_register_client(&sleeper_data.notif)) {
		ret = -EINVAL;
		goto err_dev;
	}	
	
	INIT_WORK(&sleeper_data.resume_work, msm_sleeper_resume);
	INIT_WORK(&sleeper_data.suspend_work, msm_sleeper_suspend);
	
	return ret;

err_dev:
	destroy_workqueue(sleeper_wq);

err_out:
	return ret;
}

static int msm_sleeper_remove(struct platform_device *pdev)
{
	destroy_workqueue(sleeper_wq);

	return 0;
}

static struct platform_driver msm_sleeper_driver = {
	.probe = msm_sleeper_probe,
	.remove = msm_sleeper_remove,
	.driver = {
		.name = MSM_SLEEPER,
		.owner = THIS_MODULE,
	},
};

static int __init msm_sleeper_init(void)
{
	int ret;

	ret = platform_driver_register(&msm_sleeper_driver);
	if (ret) {
		pr_err("%s: Driver register failed: %d\n", MSM_SLEEPER, ret);
		return ret;
	}

	ret = platform_device_register(&msm_sleeper_device);
	if (ret) {
		pr_err("%s: Device register failed: %d\n", MSM_SLEEPER, ret);
		return ret;
	}

	pr_info("%s: Device init\n", MSM_SLEEPER);

	return ret;
}

static void __exit msm_sleeper_exit(void)
{
	platform_device_unregister(&msm_sleeper_device);
	platform_driver_unregister(&msm_sleeper_driver);
}

late_initcall(msm_sleeper_init);
module_exit(msm_sleeper_exit);
