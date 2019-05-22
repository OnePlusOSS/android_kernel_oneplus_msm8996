/*
 * fusb301.c -- FUSB301 USB TYPE-C Controller device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Revised by Bright Yang < bright.yang@farichildsemi.com>
 *
 */


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/random.h>
#include <linux/version.h>
#include <linux/power_supply.h>

#include "type-c-fusb301.h"

#include <linux/type-c_notifier.h>

#define OTG_WL_HOLD_TIME 5000

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_DEV_ID		0x01
#define REG_MOD			0x02
#define REG_CON			0x03
#define REG_MAN			0x04
#define REG_RST			0x05
#define REG_MSK			0x10
#define REG_STAT		0x11
#define REG_TYPE		0x12
#define REG_INT			0x13

/******************************************************************************
* Register bits
******************************************************************************/
/*	  REG_DEV_ID (0x01)    */
#define ID_REV					0x0F
#define ID_VER_SHIFT			4
#define ID_VER					(0x0F << ID_VER_SHIFT)

/*    REG_MOD (0x02)    */
#define MOD_SRC 				0x01
#define MOD_SRC_ACC_SHIFT		1
#define MOD_SRC_ACC 			(0x01 << MOD_SRC_ACC_SHIFT)
#define MOD_SNK_SHIFT			2
#define MOD_SNK 				(0x01 << MOD_SNK_SHIFT)
#define MOD_SNK_ACC_SHIFT 		3
#define MOD_SNK_ACC 			(0x01 << MOD_SNK_SHIFT)
#define MOD_DRP_SHIFT			4
#define MOD_DRP 				(0x01 << MOD_DRP_SHIFT)
#define MOD_DRP_ACC_SHIFT 		5
#define MOD_DRP_ACC 			(0x01 << MOD_DRP_ACC_SHIFT)

/*    REG_CON (0x03)    */
#define CON_INT_MSK				0x01
#define CON_HOST_CUR_SHIFT		1
#define CON_HOST_CUR 			(0x03 << CON_HOST_CUR_SHIFT)
#define CON_DRP_TGL_SHIFT		4
#define CON_DRP_TGL 			(0x03 << CON_DRP_TGL_SHIFT)

/*    REG_MAN (0x04)    */
#define MAN_ERR_REC				0x01
#define MAN_DIS_SHIFT			1
#define MAN_DIS 				(0x01 << MAN_DIS_SHIFT)
#define MAN_UNATT_SRC_SHIFT		2
#define MAN_UNATT_SRC 			(0x01 << MAN_UNATT_SRC_SHIFT)
#define MAN_UNATT_SNK_SHIFT		3
#define MAN_UNATT_SNK 			(0x01 << MAN_UNATT_SNK_SHIFT)

/*    REG_RST (0x05)    */
#define RST_SW					0x01

/*    REG_MSK (0x10)    */
#define MSK_ATTACH 				0x01
#define MSK_DETACH_SHIFT		1
#define MSK_DETACH 				(0x01 << MSK_DETACH_SHIFT)
#define MSK_BC_LVL_SHIFT		2
#define MSK_BC_LVL 				(0x01 << MSK_BC_LVL_SHIFT)
#define MSK_ACC_CHG_SHIFT		3
#define MSK_ACC_CHG 			(0x01 << MSK_ACC_CHG_SHIFT)

/*    REG_STAT (0x11)    */
#define STAT_ATTACH				0x01
#define STAT_BC_LVL_SHIFT		1
#define STAT_BC_LVL 			(0x03 << STAT_BC_LVL_SHIFT)
#define STAT_VBUS_OK_SHIFT		3
#define STAT_VBUS_OK 			(0x01 << STAT_VBUS_OK_SHIFT)
#define STAT_ORIENT_SHIFT		4
#define STAT_ORIENT 			(0x03 << STAT_ORIENT_SHIFT)

/*    REG_TYPE (0x12)    */
#define TYPE_AUDIO_ACC			0x01
#define TYPE_DBG_ACC_SHIFT		1
#define TYPE_DBG_ACC			(0x01 << TYPE_DBG_ACC_SHIFT)
#define TYPE_PWR_ACC_SHIFT		2
#define TYPE_PWR_ACC 			(0x01 << TYPE_PWR_ACC_SHIFT)
#define TYPE_SRC_SHIFT			4
#define TYPE_SRC 				(0x01 << TYPE_SRC_SHIFT)
#define TYPE_SNK_SHIFT 			3
#define TYPE_SNK 				(0x01 << TYPE_SNK_SHIFT)

/*    REG_INT (0x13)    */
#define INT_FAKE				0x00
#define INT_ATTACH				0x01
#define INT_DETACH_SHIFT		1
#define INT_DETACH 				(0x01 << INT_DETACH_SHIFT)
#define INT_BC_LVL_SHIFT		2
#define INT_BC_LVL 				(0x01 << INT_BC_LVL_SHIFT)
#define INT_ACC_CHG_SHIFT		3
#define INT_ACC_CHG				(0x01 << INT_ACC_CHG_SHIFT)

/*       Try Sink       */
#define TTRYTO_EXP_TIME	((get_random_int() % 200) + 400) //400~600
#define TCCDEBOUNCEMAX_TIME	200
#define USE_TIMER_WHEN_DFP_TO_DETETC_UFP

/* 2017/12/26 infi@bsp handle irq storm issue    */
extern int otg_switch;

/******************************************************************************/
enum fusb301_drp_toggle{
	FUSB301_TOGGLE_SNK35_SRC15 = 0,  // default
	FUSB301_TOGGLE_SNK30_SRC20,
	FUSB301_TOGGLE_SNK25_SRC25,
	FUSB301_TOGGLE_SNK20_SRC30,
};

enum fusb301_host_cur{
	FUSB301_HOST_CUR_NO = 0,  // no current
	FUSB301_HOST_CUR_80,  // default USB
	FUSB301_HOST_CUR_180,  // 1.5A
	FUSB301_HOST_CUR_330,  // 3A
};

enum fusb301_orient{
	FUSB301_ORIENT_NO_CONN = 0,
	FUSB301_ORIENT_CC1_CC,
	FUSB301_ORIENT_CC2_CC,
	FUSB301_ORIENT_FAULT
};

enum fusb301_config_modes{
	FUSB301_MODE_SRC = 0,
	FUSB301_MODE_SRC_ACC,
	FUSB301_MODE_SNK,
	FUSB301_MODE_SNK_ACC,
	FUSB301_MODE_DRP,
	FUSB301_MODE_DRP_ACC
};

struct fusb301_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct mutex		mutex;
	struct class *fusb_class;
	struct power_supply	*usb_psy;
	int irq;
	//int OTG_USB_ID_irq;
	//int switch_sel_gpio;
	int irq_gpio;
	int ID_gpio;
	enum fusb301_type fusb_type;
	enum fusb301_orient fusb_orient;
	struct work_struct otg_work;
	//struct workqueue_struct *otg_wqueue;
	struct wake_lock otg_wl;
	#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
	enum fusb301_state state;
	int TriedSink;
	struct timer_list	try_sink_timer;
	struct workqueue_struct *try_sink_wqueue;
	struct work_struct try_sink_work;
	#endif
	bool otg_present;
	/* 2017/12/26 infi@bsp handle irq storm issue	 */
	bool irq_enwake_flag;
};


static int fusb301_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct fusb301_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		dev_err(&info->i2c->dev,"%s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int fusb301_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct fusb301_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		dev_err(&info->i2c->dev,"%s:reg(0x%x), ret(%d)\n",
				__func__, reg, ret);

	return ret;
}

static int fusb301_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct fusb301_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (val & mask) | (old_val & (~mask));
		ret = i2c_smbus_write_byte_data(i2c, reg, new_val);
	}
	mutex_unlock(&info->mutex);
	return ret;
}

static ssize_t show_current_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct fusb301_info *info = dev_get_drvdata(dev);

	switch(info->fusb_type){
		case FUSB301_TYPE_AUDIO:
			return sprintf(buf, "FUSB301_TYPE_AUDIO\n");
		case FUSB301_TYPE_DEBUG:
			return sprintf(buf, "FUSB301_TYPE_DEBUG\n");
		case FUSB301_TYPE_POWER_ACC:
			return sprintf(buf, "FUSB301_TYPE_POWER_ACC\n");
		case FUSB301_TYPE_SOURCE:
			return sprintf(buf, "FUSB301_SOURCE\n");
		case FUSB301_TYPE_SINK:
			return sprintf(buf, "FUSB301_TYPE_SINK\n");
		default:
			return sprintf(buf, "Not Connected\n");
	}

}
static ssize_t show_CC_state(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct fusb301_info *info = dev_get_drvdata(dev);

	switch(info->fusb_orient){
		case FUSB301_ORIENT_CC1_CC:
			return sprintf(buf, "cc1\n");
		case FUSB301_ORIENT_CC2_CC:
			return sprintf(buf, "cc2\n");
		case FUSB301_ORIENT_NO_CONN:
			return sprintf(buf, "disconnect\n");
		case FUSB301_ORIENT_FAULT:
        default:
			return sprintf(buf, "fault\n");
	}

}
static ssize_t config_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int error;
    unsigned long data;
	u8 rdata;
	struct fusb301_info *info = dev_get_drvdata(dev);

	error = sstrtoul(buf, 10, &data);
	if(error)
		return error;

	if(data == FUSB301_MODE_SRC)
	    fusb301_write_reg(info->i2c, REG_MOD, MOD_SRC);
	else if(data == FUSB301_MODE_SRC_ACC)
	    fusb301_write_reg(info->i2c, REG_MOD, MOD_SRC_ACC);
	else if(data == FUSB301_MODE_SNK)
	    fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK);
	else if(data == FUSB301_MODE_SNK_ACC)
	    fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK_ACC);
	else if(data == FUSB301_MODE_DRP)
	    fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP);
	else if(data == FUSB301_MODE_DRP_ACC)
	    fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP_ACC);
	else
		dev_err(&info->i2c->dev,"%s: Argument is not match!!\n", __func__);

    fusb301_read_reg(info->i2c, REG_MOD, &rdata);
	dev_err(&info->i2c->dev,"%s: REG_MOD(0x%02x)\n", __func__, rdata);

    return count;
}

static DEVICE_ATTR(type, S_IRUGO, show_current_type, NULL);
static DEVICE_ATTR(CC_state, S_IRUGO, show_CC_state, NULL);
static DEVICE_ATTR(mode, S_IWUSR, NULL, config_mode);

#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
static void fusb301_try_sink_work_func(struct work_struct *work)
{
	struct fusb301_info *info =
		container_of(work, struct fusb301_info, try_sink_work);

	//mutex_lock(&info->mutex);
	if(info->state == FUSB301_UNATTACHED_SNK)
	{
	    //dev_err(&info->i2c->dev,"%s: FUSB301_UNATTACHED_SNK!\n", __func__);
		mod_timer(&info->try_sink_timer, jiffies + msecs_to_jiffies(TCCDEBOUNCEMAX_TIME));
		info->state = FUSB301_UNATTACHED_SRC;
		fusb301_write_reg(info->i2c, REG_MOD, MOD_SRC);
		fusb301_write_reg(info->i2c, REG_MAN, MAN_UNATT_SRC);
	}
	else if(info->state == FUSB301_UNATTACHED_SRC)
	{
	    dev_err(&info->i2c->dev,"%s: FUSB301_UNATTACHED_SRC!\n", __func__);
		info->state = FUSB301_UNATTACHED_DRP;
		info->TriedSink = 0;
		fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP);
		fusb301_write_reg(info->i2c, REG_MAN, MAN_UNATT_SNK);
	}
	//mutex_unlock(&info->mutex);

}

static void __fusb302_try_sink_timer_func(unsigned long param)
{
    //int ret = 0;
    struct fusb301_info *info = (struct fusb301_info *)param;

	//dev_err(&info->i2c->dev,"%s entered and queue work\n", __func__);
	queue_work(info->try_sink_wqueue, &info->try_sink_work);
}
#endif

static void fusb301_check_type(struct fusb301_info *info, u8 type)
{
    if(type & TYPE_AUDIO_ACC)
    {
        info->fusb_type = FUSB301_TYPE_AUDIO;
    }
	else if(type & TYPE_DBG_ACC)
	{
	    info->fusb_type = FUSB301_TYPE_DEBUG;
	}
	else if(type & TYPE_PWR_ACC)
	{
	    info->fusb_type = FUSB301_TYPE_POWER_ACC;
	}
	else if(type & TYPE_SRC)
	{
	    info->fusb_type = FUSB301_TYPE_SOURCE;
	}
	else if(type & TYPE_SNK)
	{
	    info->fusb_type = FUSB301_TYPE_SINK;
	}
	else
	{
	    dev_err(&info->i2c->dev,"%s: No device type!\n", __func__);
	}

}

static void fusb301_check_orient(struct fusb301_info *info, u8 status)
{
	u8 orient = ((status & STAT_ORIENT)>>STAT_ORIENT_SHIFT);
	info->fusb_orient = orient;
}
/*Yangfb add begin to notify pmic to checkout usb unplug */
static int set_property_on_smbchg(enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };
	struct power_supply *battery_psy;

	battery_psy = power_supply_get_by_name("battery");
	if (!battery_psy) {
		pr_err("battery_psy not exist\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = battery_psy->set_property(battery_psy, prop, &ret);
	if (rc)
		pr_err("bms psy does not allow updating prop %d rc = %d\n",prop, rc);

	return rc;
}
static irqreturn_t fusb301_irq_thread(int irq, void *handle)
{
    u8 intr, rdata;
	int bc_lvl;
	struct fusb301_info *info = (struct fusb301_info *)handle;

    fusb301_read_reg(info->i2c, REG_INT, &intr);
	dev_err(&info->i2c->dev,"%s: type<%d> int(0x%02x)\n", __func__,info->fusb_type, intr);

	if(intr & INT_ATTACH)
	{
		fusb301_read_reg(info->i2c, REG_TYPE, &rdata);
		fusb301_check_type(info, rdata);
		fusb301_read_reg(info->i2c, REG_STAT, &rdata);
		fusb301_check_orient(info,rdata);
		dev_err(&info->i2c->dev,"%s: Attach interrupt! TYPE is %d, Orient is %d\n", __func__, info->fusb_type, info->fusb_orient);
		/*
		if (info->fusb_orient == FUSB301_ORIENT_CC1_CC)
		{
			gpio_set_value(info->switch_sel_gpio, 1);
		}
		else if (info->fusb_orient == FUSB301_ORIENT_CC2_CC)
		{
			gpio_set_value(info->switch_sel_gpio, 0);
		}*/

		if(info->fusb_type == FUSB301_TYPE_SOURCE)//phone as source
		{
			//SOURCE
			#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP

			/* 2017/12/26 infi@bsp handle irq storm issue    */
			if(!otg_switch && info->irq_enwake_flag)
			{	/*otg-switch closed but irq-wake enabled in first-boot disable irq wake.*/
				dev_err(&info->i2c->dev,"%s : otg_switch=(%d),irq_enwake_flag=(%d),first boot disable irq wake!\n",\
					__func__,otg_switch,info->irq_enwake_flag);
				disable_irq_wake(irq);
				info->irq_enwake_flag = false;
				goto done;

			}
			else if(!otg_switch && !info->irq_enwake_flag)
			{	/*otg-switch closed and irq-wake disabled,irq storm or normal insertion without open otg-switch do nothing*/
				dev_err(&info->i2c->dev,"%s : otg_switch=(%d),irq_enwake_flag=(%d) irq storm or insertion without open otg-switch!\n",\
					__func__,otg_switch,info->irq_enwake_flag);

				goto done;
			}
			else if(otg_switch && !info->irq_enwake_flag)
			{	/*otg-switch opened but irq-wake disabled,go on!.*/
				dev_err(&info->i2c->dev,"%s : otg_switch=(%d),irq_enwake_flag=(%d),enable_irq_wake fail!\n",\
					__func__,otg_switch,info->irq_enwake_flag);

			}
			else /*otg-switch opened and irq-wake enabled,go work!*/
				dev_err(&info->i2c->dev,"%s : otg_switch=(%d),irq_enwake_flag=(%d)\n",\
					__func__,otg_switch,info->irq_enwake_flag);

			info->state = FUSB301_ATTACHED_SRC;

			if(!info->TriedSink)
			{
				dev_err(&info->i2c->dev, "%s: Try.SNK!\n", __func__);
				mod_timer(&info->try_sink_timer, jiffies + msecs_to_jiffies(TTRYTO_EXP_TIME));
				info->TriedSink = 1;
				info->state = FUSB301_UNATTACHED_SNK;
				fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK); //0x02,0x04
				fusb301_write_reg(info->i2c, REG_MAN, MAN_UNATT_SNK); //0x04,0x08
				wake_lock_timeout(&info->otg_wl, msecs_to_jiffies(OTG_WL_HOLD_TIME));
			}
			else
			{
				dev_err(&info->i2c->dev, "%s: TriedSink is true & del timer\n", __func__);
				info->TriedSink = 0;
				del_timer(&info->try_sink_timer);

				info->otg_present = true;
				dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,info->otg_present);
				power_supply_set_usb_otg(info->usb_psy, info->otg_present ? 1 : 0);
			}
			#else
			info->otg_present = true;
			dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,info->otg_present);
			power_supply_set_usb_otg(info->usb_psy, info->otg_present ? 1 : 0);
			#endif
			//TODO triger OTG isr,open 5V VBUS and change USB PHY to HOST
		}
		else if(info->fusb_type == FUSB301_TYPE_SINK)
		{
		    // SINK
			#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
		    info->state = FUSB301_ATTACHED_SNK;

			if(info->TriedSink)
			{
				dev_err(&info->i2c->dev, "%s: Attached.SNK after Try.SNK! \n", __func__);
				info->TriedSink = 0;
				del_timer(&info->try_sink_timer);
			}
			#endif
			if(info->otg_present == true){
				info->otg_present = false;
				dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,info->otg_present);
				power_supply_set_usb_otg(info->usb_psy, info->otg_present ? 1 : 0);
			}
			fusb301_read_reg(info->i2c, REG_STAT, &rdata);
			bc_lvl = (rdata & STAT_BC_LVL) >> STAT_BC_LVL_SHIFT;
			dev_err(&info->i2c->dev,"%s: BC_LVL %d, 1:500ma 2:1.5A  3:3A!\n", __func__, bc_lvl);
			bc_notifier_call_chain(bc_lvl);
		}

	}
	else if(intr & INT_DETACH)
	{
	    // Detach
	    dev_err(&info->i2c->dev,"%s: Detach interrupt!\n", __func__);
	info->fusb_type = FUSB301_TYPE_NONE;
        info->fusb_orient= FUSB301_ORIENT_NO_CONN;

		#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
		fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP_ACC);
		info->state = FUSB301_UNATTACHED_DRP;
		info->TriedSink = 0;
		#endif

		if(info->otg_present == true){
			info->otg_present = false;
			dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,info->otg_present);
			power_supply_set_usb_otg(info->usb_psy, info->otg_present ? 1 : 0);
		}
/* Yangfb add to check usb unplug */
		set_property_on_smbchg(POWER_SUPPLY_PROP_CHECK_USB_UNPLUG, true);
	}
	else if(intr & INT_BC_LVL)
	{
	    dev_err(&info->i2c->dev,"%s: BC_LVL interrupt!\n", __func__);
	}
	else if(intr & INT_ACC_CHG)
	{
	    dev_err(&info->i2c->dev,"%s: Accessory change interrupt!\n", __func__);
	}
	else
		dev_err(&info->i2c->dev,"%s: weird interrupt!\n", __func__);
/* 2017/12/26 infi@bsp handle irq storm issue    */
done:
    return IRQ_HANDLED;
}

static void fusb301_reboot_scan_otg(struct fusb301_info *info)
{
    u8  rdata;
	int bc_lvl;

	fusb301_read_reg(info->i2c, REG_TYPE, &rdata);
	fusb301_check_type(info, rdata);
	fusb301_read_reg(info->i2c, REG_STAT, &rdata);
	fusb301_check_orient(info,rdata);
	dev_err(&info->i2c->dev,"%s: TYPE is %d, Orient is %d\n", __func__, info->fusb_type, info->fusb_orient);

	if(info->fusb_type == FUSB301_TYPE_SOURCE)//phone as source
	{
		//SOURCE
		#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
		info->state = FUSB301_ATTACHED_SRC;

		if(!info->TriedSink)
		{
			dev_err(&info->i2c->dev, "%s: Try.SNK!\n", __func__);
			mod_timer(&info->try_sink_timer, jiffies + msecs_to_jiffies(TTRYTO_EXP_TIME));
			info->TriedSink = 1;
			info->state = FUSB301_UNATTACHED_SNK;
			fusb301_write_reg(info->i2c, REG_MOD, MOD_SNK); //0x02,0x04
			fusb301_write_reg(info->i2c, REG_MAN, MAN_UNATT_SNK); //0x04,0x08
		}
		#endif
	}
	else if(info->fusb_type == FUSB301_TYPE_SINK)
	{
	    // SINK
		#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
	    info->state = FUSB301_ATTACHED_SNK;

		if(info->TriedSink)
		{
			dev_err(&info->i2c->dev, "%s: Attached.SNK after Try.SNK! \n", __func__);
			info->TriedSink = 0;
			del_timer(&info->try_sink_timer);
		}
		#endif
		fusb301_read_reg(info->i2c, REG_STAT, &rdata);
		bc_lvl = (rdata & STAT_BC_LVL) >> STAT_BC_LVL_SHIFT;
		dev_err(&info->i2c->dev,"%s: BC_LVL %d, 1:500ma 2:1.5A  3:3A!\n", __func__, bc_lvl);
		bc_notifier_call_chain(bc_lvl);
	}
}

/*
#if 0
static void fusb301_otg_work(struct work_struct *work)
{
    struct fusb301_info *info = container_of(work, struct fusb301_info, otg_work);
    bool otg_present = !gpio_get_value(info->ID_gpio);
    dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,otg_present);
    power_supply_set_usb_otg(info->usb_psy, otg_present ? 1 : 0);
}

static irqreturn_t fusb301_otg_irq_thread(int irq, void *handle)
{
    struct fusb301_info *info = (struct fusb301_info *)handle;
    wake_lock_timeout(&info->otg_wl, msecs_to_jiffies(OTG_WL_HOLD_TIME));
    queue_work(info->otg_wqueue, &info->otg_work);
    return IRQ_HANDLED;
}
#endif
*/

static void fusb301_initialization(struct fusb301_info *info)
{
	info->fusb_type = FUSB301_TYPE_NONE;
	info->fusb_orient= FUSB301_ORIENT_NO_CONN;

	#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
	info->state = FUSB301_UNATTACHED_DRP;
	info->TriedSink = 0;
	#endif

	fusb301_write_reg(info->i2c, REG_MOD, MOD_DRP);
	fusb301_update_reg(info->i2c, REG_CON, 0, CON_INT_MSK);  //unmask global interrupts
}

static int fusb301_gpio_configure(struct fusb301_info *info)
{
	int retval = 0;

		if (gpio_is_valid(info->irq_gpio)) {
			/* configure touchscreen irq gpio */
			retval = gpio_request(info->irq_gpio,
				"fusb301_irq_gpio");
			if (retval) {
				dev_err(&info->i2c->dev,
					"unable to request gpio [%d]\n",
					info->irq_gpio);
				goto err_irq_gpio_req;
			}
			retval = gpio_direction_input(info->irq_gpio);
			if (retval) {
				dev_err(&info->i2c->dev,
					"unable to set dir for gpio[%d]\n",
					info->irq_gpio);
				goto err_irq_gpio_dir;
			}
			info->irq = gpio_to_irq(info->irq_gpio);
		if (info->irq < 0) {
			dev_err(&info->i2c->dev,
				"Unable to get irq number for GPIO %d, error %d\n",
					info->irq_gpio, info->irq);
			retval = info->irq;
			goto err_irq_gpio_dir;
		}
		} else {
			dev_err(&info->i2c->dev,
				"irq gpio not provided\n");
			goto err_irq_gpio_req;
		}

		if (gpio_is_valid(info->ID_gpio)) {
			/* configure touchscreen reset out gpio */
			retval = gpio_request(info->ID_gpio,
					"fusb301_OTG_USB_ID_gpio");
			if (retval) {
				dev_err(&info->i2c->dev, "unable to request gpio [%d]\n",
					info->ID_gpio);
				goto err_irq_gpio_to_irq;
			}

			retval = gpio_direction_input(info->ID_gpio);
			if (retval) {
				dev_err(&info->i2c->dev,
					"unable to set dir for gpio [%d]\n",
					info->ID_gpio);
				goto err_irq_gpio_to_irq;
			}
			/*
			#if 0
			info->OTG_USB_ID_irq = gpio_to_irq(info->ID_gpio);
			if (info->OTG_USB_ID_irq < 0) {
				dev_err(&info->i2c->dev,
					"Unable to get irq number for GPIO %d, error %d\n",
						info->ID_gpio, info->OTG_USB_ID_irq);
				retval = info->OTG_USB_ID_irq;
				goto err_irq_gpio_dir;
			}
			#endif
			*/
		}else {
			dev_err(&info->i2c->dev,
				"OTG_USB_ID gpio not provided\n");
			goto err_id_gpio_dir;
		}
        dev_err(&info->i2c->dev,"%s OK!\n",__func__);
		return 0;

err_id_gpio_dir:
	if (gpio_is_valid(info->ID_gpio))
		gpio_free(info->ID_gpio);
err_irq_gpio_to_irq:
    free_irq(info->irq, info);
err_irq_gpio_dir:
	if (gpio_is_valid(info->irq_gpio))
		gpio_free(info->irq_gpio);
err_irq_gpio_req:
	return retval;
}

/*
   Add by yangrujin@bsp 2015/1/25, fix [BUG] RAIN-405: connect OTG and U-Disk to device,
   then shutdown device, device will auto reboot instead of pwroff.
 */
#include <linux/notifier.h>
#include <linux/reboot.h>
static struct fusb301_info *ginfo;

static int fusb301_power_down_callback(
		struct notifier_block *nfb, unsigned long action, void *data)
{
	if(ginfo == NULL){
		return NOTIFY_OK;
	}

	switch (action) {
	case SYS_POWER_OFF:
	case SYS_RESTART:
	case SYS_HALT:
		pr_info("%s : action=0x%x\n", __func__, (unsigned)action);
		/*
		#if 0
		disable_irq_wake(ginfo->OTG_USB_ID_irq);
		free_irq(ginfo->OTG_USB_ID_irq, ginfo);
		#endif
		*/
		//set to UFP to get a chance to make sure get charging from other DRP typeC device
		fusb301_write_reg(ginfo->i2c, REG_MOD, MOD_SNK); //0x02,0x04
		power_supply_set_usb_otg(ginfo->usb_psy, 0);
		break;
	default:
		return NOTIFY_DONE;
	}

	return NOTIFY_OK;
}

static struct notifier_block fusb301_reboot_notifier = {
    .notifier_call = fusb301_power_down_callback,
};

/*add by infi@bsp 2012/12/26, handle the irq storm issue*/
extern int otg_switch_register_client(struct notifier_block *nb);

static int fusb301_fake_trigger(struct fusb301_info *info)
{
    u8 intr, rdata;

	dev_err(&info->i2c->dev,"%s: Fake irq trigger!\n",__func__);
    fusb301_read_reg(info->i2c, REG_INT, &intr);
	dev_err(&info->i2c->dev,"%s: type<%d> int(0x%02x)\n", __func__,info->fusb_type, intr);

	if(intr == INT_FAKE)// fake trigger 0x00
	{
		fusb301_read_reg(info->i2c, REG_TYPE, &rdata);
		fusb301_check_type(info, rdata);
		fusb301_read_reg(info->i2c, REG_STAT, &rdata);
		fusb301_check_orient(info,rdata);
		dev_err(&info->i2c->dev,"%s: Fake interrupt! TYPE is %d, Orient is %d\n", __func__, info->fusb_type, info->fusb_orient);

		if(info->fusb_type == FUSB301_TYPE_SOURCE)//as source
		{
			//SOURCE
			/*for fake trigger,need handle source scenario to trigger otg*/
			info->otg_present = true;
			dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,info->otg_present);
			power_supply_set_usb_otg(info->usb_psy, info->otg_present ? 1 : 0);
			//TODO triger OTG isr,open 5V VBUS and change USB PHY to HOST
		}
		else if(info->fusb_type == FUSB301_TYPE_SINK)//as sink
		{
		    // SINK
			dev_err(&info->i2c->dev, "%s: type<%d> do nothing!\n", __func__,info->fusb_type);
		}

	}
	else if(intr & INT_DETACH)
	{
	    // Detach
	    dev_err(&info->i2c->dev,"%s: Detach interrupt!\n", __func__);
	}
	else if(intr & INT_BC_LVL)
	{
	    dev_err(&info->i2c->dev,"%s: BC_LVL interrupt!\n", __func__);
	}
	else if(intr & INT_ACC_CHG)
	{
	    dev_err(&info->i2c->dev,"%s: Accessory change interrupt!\n", __func__);
	}
	else
		dev_err(&info->i2c->dev,"%s: weird interrupt!\n", __func__);

    return 0;
}


static int otg_switch_callback(
		struct notifier_block *nb, unsigned long value, void *data)
{
	if(ginfo == NULL){
		return NOTIFY_OK;
	}

	if(value == 1){
		dev_err(&ginfo->i2c->dev,"%s: value=(%ld)\n",__func__,value);
		if(otg_switch && !ginfo->irq_enwake_flag){
			/*otg-switch opened but irq wake disabled,need enable irq wake.*/
			dev_err(&ginfo->i2c->dev,"%s : otg_switch=(%d),irq_enwake_flag=(%d),enable_irq_wake\n",\
					__func__,otg_switch,ginfo->irq_enwake_flag);
			enable_irq_wake(ginfo->irq);
			ginfo->irq_enwake_flag = true;
			/*insertion without open otg-switch first time,when open otg-switch update insertion event->irq 0x00*/
			fusb301_fake_trigger(ginfo);
		}
	}
	else if(value == 0){
		dev_err(&ginfo->i2c->dev,"%s: value=(%ld)\n",__func__,value);
		if(!otg_switch && ginfo->irq_enwake_flag){
			/*otg-switch closed but irq wake enabled,need disable irq wake.*/
			dev_err(&ginfo->i2c->dev,"%s : otg_switch=(%d),irq_enwake_flag=(%d),disable_irq_wake\n",\
					__func__,otg_switch,ginfo->irq_enwake_flag);
			disable_irq_wake(ginfo->irq);
			ginfo->irq_enwake_flag = false;
		}
	}
	else
		pr_err("%s:otg_switch value=(%ld)\n",__func__,value);

	return NOTIFY_OK;
}

static struct notifier_block otg_switch_notifier = {
	.notifier_call = otg_switch_callback,
};


static int fusb301_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	u8 rdata = 0;
	struct fusb301_info *info;
	struct device_node *np = client->dev.of_node;
	struct power_supply *usb_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB power_supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}

	info = kzalloc(sizeof(struct fusb301_info), GFP_KERNEL);
	info->i2c = client;
	info->usb_psy = usb_psy;
	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	dev_err(&client->dev, "%s\n",__func__);



	fusb301_read_reg(info->i2c, REG_DEV_ID, &rdata);
	//if(rdata != 0x10){
	if(rdata != 0x12){
	    dev_err(&client->dev, "DEV_ID(0x%x)!= 0x10\n",rdata);
	    ret = -EINVAL;
	    goto parse_dt_failed;
	}
/*
	info->irq = irq_of_parse_and_map(np, 0);///////////TODO,changhua modify to add a irq_gpio,then gpio_request()-->gpio_direction_input()--->gpio_to_irq()
	if (info->irq <= 0) {
		printk(KERN_ERR "invalid eint number - %d\n", info->irq);
		return -EINVAL;
	}

	//get gpio to control fusb340
	info->switch_sel_gpio = of_get_named_gpio(np, "cc,sel-gpio", 0);
	if (info->switch_sel_gpio <=0 )
	{
		printk(KERN_ERR "invalid gpio number - %d\n", info->switch_sel_gpio);
		return -EINVAL;
	}
*/
    /* USBID, irq gpio info */
	info->ID_gpio = of_get_named_gpio(np,"fusb301,ID-gpio", 0);
	info->irq_gpio = of_get_named_gpio(np,"fusb301,irq-gpio", 0);
	ret = fusb301_gpio_configure(info);
	if(ret != 0)
	{
	    goto parse_dt_failed;
	}


	info->fusb_class = class_create(THIS_MODULE, "type-c-fusb301");
	info->dev_t = device_create(info->fusb_class, NULL, 0, NULL, "fusb301");
	device_create_file(info->dev_t, &dev_attr_type);
	device_create_file(info->dev_t, &dev_attr_mode);
	device_create_file(info->dev_t, &dev_attr_CC_state);
	dev_set_drvdata(info->dev_t, info);

	#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
	info->try_sink_wqueue = create_singlethread_workqueue("fusb301_wqueue");
	INIT_WORK(&info->try_sink_work, fusb301_try_sink_work_func);
	#endif

	fusb301_initialization(info);

	#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
	setup_timer(&info->try_sink_timer, __fusb302_try_sink_timer_func,
		(unsigned long)info);
	#endif

	//info->otg_wqueue = create_singlethread_workqueue("fusb301_otg_wqueue");
	//INIT_WORK(&info->otg_work, fusb301_otg_work);

	ret = request_threaded_irq(info->irq, NULL, fusb301_irq_thread,
			  IRQF_TRIGGER_LOW | IRQF_ONESHOT, "fusb301_irq", info);

	if (ret){
		dev_err(&client->dev, "failed to reqeust IRQ\n");
		goto request_irq_failed;
	}

	/*
	ret = request_threaded_irq(info->OTG_USB_ID_irq, NULL, fusb301_otg_irq_thread,
			  IRQF_TRIGGER_LOW | IRQF_ONESHOT, "fusb301_otg_irq", info);
	*/
	/*
	#if 0
	ret = request_irq(info->OTG_USB_ID_irq, fusb301_otg_irq_thread,
				IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "fusb301_otg_irq", info);
	#endif
	*/
	if (ret){
		dev_err(&client->dev, "failed to reqeust OTG IRQ\n");
		goto request_irq_failed;
	}

	ret = enable_irq_wake(info->irq);
	if (ret < 0){
	    dev_err(&client->dev,
		 "failed to enable wakeup src %d\n", ret);
		goto enable_irq_failed;
	}
	/* 2017/12/26 infi@bsp handle irq storm issue	 */
	info->irq_enwake_flag = true;
	/*
	#if 0
	ret = enable_irq_wake(info->OTG_USB_ID_irq);
	if (ret < 0){
	    dev_err(&client->dev,
		 "failed to enable wakeup src %d\n", ret);
		goto enable_irq_failed;
	}
	#endif
	*/

/*
 Add by yangrujin@bsp 2015/1/25, fix [BUG] RAIN-405: connect OTG and U-Disk to device,
 then shutdown device, device will auto reboot instead of pwroff.
 */
    ginfo = info;
    register_reboot_notifier(&fusb301_reboot_notifier);
	otg_switch_register_client(&otg_switch_notifier);

    dev_err(&info->i2c->dev,"%s OK!\n",__func__);
	wake_lock_init(&info->otg_wl, WAKE_LOCK_SUSPEND, "fusb301_otg_wl");
	/* Update initial state to USB */
	if(!gpio_get_value(info->ID_gpio)){
		//fusb301_otg_irq_thread(info->OTG_USB_ID_irq, info);
		fusb301_reboot_scan_otg(info);
	}
	return 0;

enable_irq_failed:
	free_irq(info->irq,NULL);
	//free_irq(info->OTG_USB_ID_irq,NULL);
request_irq_failed:
	device_remove_file(info->dev_t, &dev_attr_type);
	device_destroy(info->fusb_class, 0);
	class_destroy(info->fusb_class);

parse_dt_failed:
    mutex_destroy(&info->mutex);
    i2c_set_clientdata(client, NULL);
	kfree(info);

	return ret;

}

static int fusb301_remove(struct i2c_client *client)
{
    struct fusb301_info *info = i2c_get_clientdata(client);

    if (client->irq) {
        disable_irq_wake(client->irq);
        free_irq(client->irq, info);
    }
    device_remove_file(info->dev_t, &dev_attr_type);
    device_destroy(info->fusb_class, 0);
    class_destroy(info->fusb_class);
    mutex_destroy(&info->mutex);
    i2c_set_clientdata(client, NULL);

    kfree(info);
    return 0;
}


static int  fusb301_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  fusb301_resume(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id fusb301_id[] = {
		{.compatible = "fusb301"},
		{},
};
MODULE_DEVICE_TABLE(of, fusb301_id);


static const struct i2c_device_id fusb301_i2c_id[] = {
	{ "fusb301", 0 },
	{ }
};

static struct i2c_driver fusb301_i2c_driver = {
	.driver = {
		.name = "fusb301",
		.of_match_table = of_match_ptr(fusb301_id),
	},
	.probe    = fusb301_probe,
	.remove   = fusb301_remove,
	.suspend  = fusb301_suspend,
	.resume	  = fusb301_resume,
	.id_table = fusb301_i2c_id,
};

static __init int fusb301_i2c_init(void)
{

	return i2c_add_driver(&fusb301_i2c_driver);
}

static __exit void fusb301_i2c_exit(void)
{
	i2c_del_driver(&fusb301_i2c_driver);
}

module_init(fusb301_i2c_init);
module_exit(fusb301_i2c_exit);

MODULE_AUTHOR("chris.jeong@fairchildsemi.com");
MODULE_DESCRIPTION("I2C bus driver for FUSB301 USB Type-C");
MODULE_LICENSE("GPL v2");
