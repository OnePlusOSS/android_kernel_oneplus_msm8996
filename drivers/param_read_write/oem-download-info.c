
#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/export.h>
#include <linux/module.h>
#include <linux/err.h>

#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/types.h>

#include <asm/system_misc.h>

#include <linux/param_rw.h>

param_download_t download_info;

static ssize_t
msm_get_smt_download_time(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.smt_download_time1);
}


static ssize_t
msm_get_smt_download_version(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.smt_download_version1);
}

static ssize_t
msm_get_upgrade_download_time1(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.upgrade_download_time1);
}


static ssize_t
msm_get_upgrade_download_version1(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.upgrade_download_version1);
}
static ssize_t
msm_get_upgrade_download_time2(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.upgrade_download_time2);
}


static ssize_t
msm_get_upgrade_download_version2(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.upgrade_download_version2);
}

static ssize_t
msm_get_upgrade_download_time3(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.upgrade_download_time3);
}


static ssize_t
msm_get_upgrade_download_version3(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		download_info.upgrade_download_version3);
}


static struct device_attribute oem_attr_smt_download_time =
	__ATTR(smt_download_time, S_IRUGO, msm_get_smt_download_time,  NULL);

static struct device_attribute oem_attr_smt_download_version =
	__ATTR(smt_download_version, S_IRUGO, msm_get_smt_download_version,  NULL);

static struct device_attribute oem_attr_upgrade_download_time1 =
	__ATTR(upgrade_download_time1, S_IRUGO, msm_get_upgrade_download_time1,  NULL);

static struct device_attribute oem_attr_upgrade_download_version1 =
	__ATTR(upgrade_download_version1, S_IRUGO, msm_get_upgrade_download_version1, NULL);

static struct device_attribute oem_attr_upgrade_download_time2 =
	__ATTR(upgrade_download_time2, S_IRUGO, msm_get_upgrade_download_time2,  NULL);

static struct device_attribute oem_attr_upgrade_download_version2 =
	__ATTR(upgrade_download_version2, S_IRUGO, msm_get_upgrade_download_version2, NULL);

static struct device_attribute oem_attr_upgrade_download_time3 =
	__ATTR(upgrade_download_time3, S_IRUGO, msm_get_upgrade_download_time3,  NULL);

static struct device_attribute oem_attr_upgrade_download_version3 =
	__ATTR(upgrade_download_version3, S_IRUGO, msm_get_upgrade_download_version3, NULL);


static void __init populate_oem_sysfs_files(struct device *oem_download_info_device)
{
	device_create_file(oem_download_info_device, &oem_attr_smt_download_time);
	device_create_file(oem_download_info_device, &oem_attr_smt_download_version);
	device_create_file(oem_download_info_device, &oem_attr_upgrade_download_time1);
	device_create_file(oem_download_info_device, &oem_attr_upgrade_download_version1);
	device_create_file(oem_download_info_device, &oem_attr_upgrade_download_time2);
	device_create_file(oem_download_info_device, &oem_attr_upgrade_download_version2);
	device_create_file(oem_download_info_device, &oem_attr_upgrade_download_time3);
	device_create_file(oem_download_info_device, &oem_attr_upgrade_download_version3);

	return;
}


static void download_info_release(struct device *dev)
{
	kfree(dev);
}

static int __init download_info_init_sysfs(void)
{
	struct device *oem_download_info_device;
	int ret;
	
	oem_download_info_device = kzalloc(sizeof(struct device), GFP_KERNEL);
	if (!oem_download_info_device) {
		pr_err("Download info Device alloc failed!\n");
		return -ENOMEM;
	}

	oem_download_info_device->release = download_info_release;

	dev_set_name(oem_download_info_device, "download_info");

	ret = device_register(oem_download_info_device);
	if (ret)
		goto out;

	populate_oem_sysfs_files(oem_download_info_device);
	return 0;

out:
	kfree(oem_download_info_device);
	return ret;
}

late_initcall(download_info_init_sysfs);


int __init download_info_init(void)
{
	static bool download_info_init_done;
	int ret;

	if (download_info_init_done)
		return 0;

	ret = get_param_download_info(&download_info);
	if (ret < 0) {
		pr_err("Can't get download_info.\n");
		return -1;
	}
	else
		download_info_init_done = true;

	return 0;
}
subsys_initcall(download_info_init);


