/*
 * tusb320.c -- TUSB320 USB TYPE-C Controller device driver
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

#include <linux/version.h>
#include <linux/power_supply.h>

#include <linux/type-c_notifier.h>

/******************************************************************************/

#define USE_TIMER_WHEN_DFP_TO_DETETC_UFP
#define OTG_WL_HOLD_TIME 5000

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif


#define TUSB320_ATTACH	1
#define TUSB320_DETACH	0

enum tusb320_type{
	TUSB320_TYPE_NONE = 0,
	TUSB320_TYPE_AUDIO,
	TUSB320_TYPE_DEBUG,
	TUSB320_TYPE_POWER_ACC,
	TUSB320_TYPE_SOURCE,
	TUSB320_TYPE_SINK
};

enum tusb320_bc_lvl{
	TUSB320_BC_LVL_RA = 0,
	TUSB320_BC_LVL_USB,
	TUSB320_BC_LVL_1P5,
	TUSB320_BC_LVL_3A
};

enum tusb320_drp_toggle{
	TUSB320_TOGGLE_SNK35_SRC15 = 0,  // default
	TUSB320_TOGGLE_SNK30_SRC20,
	TUSB320_TOGGLE_SNK25_SRC25,
	TUSB320_TOGGLE_SNK20_SRC30,
};

enum tusb320_host_cur{
	TUSB320_HOST_CUR_NO = 0,  // no current
	TUSB320_HOST_CUR_80,  // default USB
	TUSB320_HOST_CUR_180,  // 1.5A
	TUSB320_HOST_CUR_330,  // 3A
};

enum tusb320_orient{
	TUSB320_ORIENT_NO_CONN = 0,
	TUSB320_ORIENT_CC1_CC,
	TUSB320_ORIENT_CC2_CC,
	TUSB320_ORIENT_FAULT
};

enum tusb320_config_modes{
	TUSB320_MODE_SRC = 0,
	TUSB320_MODE_SRC_ACC,
	TUSB320_MODE_SNK,
	TUSB320_MODE_SNK_ACC,
	TUSB320_MODE_DRP,
	TUSB320_MODE_DRP_ACC
};

struct tusb320_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct mutex		mutex;
	struct class *fusb_class;
	struct power_supply	*usb_psy;
	int irq;
	int OTG_USB_ID_irq;
	//int switch_sel_gpio;
	int irq_gpio;
	int ID_gpio;
	enum tusb320_type fusb_type;
	enum tusb320_orient fusb_orient;
	struct timer_list s_timer;
	struct work_struct otg_work;
	struct workqueue_struct *otg_wqueue;
	struct wake_lock otg_wl;
};

u8 Try_Snk_Attempt = 0x00; //Global Variable for Try.Snk attempt.


static int tusb320_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct tusb320_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		printk("%s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int tusb320_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct tusb320_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		printk("%s:reg(0x%x), ret(%d)\n",
				__func__, reg, ret);

	return ret;
}
/*
static int tusb320_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct tusb320_info *info = i2c_get_clientdata(i2c);
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
*/

static int tusb320_clear_intr(struct i2c_client *i2c)
{
    struct tusb320_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	/*
	ret = i2c_smbus_read_byte_data(i2c, 0x09);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (old_val | 0x10);
		ret = i2c_smbus_write_byte_data(i2c, 0x09, new_val);
	}*/
	ret = i2c_smbus_write_byte_data(i2c, 0x09, 0x10);
	mutex_unlock(&info->mutex);
	return ret;
}

#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
static int tusb320_soft_reset(struct i2c_client *i2c)
{
	struct tusb320_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	/*
	ret = i2c_smbus_read_byte_data(i2c, 0x0a);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (old_val | 0x08);
		ret = i2c_smbus_write_byte_data(i2c, 0x0a, new_val);
	}*/
	ret = i2c_smbus_write_byte_data(i2c, 0x0a, 0x08);
	mutex_unlock(&info->mutex);
	return ret;
}
static int tusb320_change_DRP_duty_cycle_and_clear_intr(struct i2c_client *i2c)
{
    struct tusb320_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	/*
	ret = i2c_smbus_read_byte_data(i2c, 0x09);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (old_val | 0x10);
		ret = i2c_smbus_write_byte_data(i2c, 0x09, new_val);
	}*/
	ret = i2c_smbus_write_byte_data(i2c, 0x09, 0x16);
	mutex_unlock(&info->mutex);
	return ret;
}

#if 0
static void timer_handle(unsigned long data)
{
    struct tusb320_info *info = (struct tusb320_info *)data;
    Try_Snk_Attempt = 0x00; //Clear flag
    tusb320_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Set DRP duty cycle to default
}

static void Start_WatchDog_Timer(struct tusb320_info *info)
{
	init_timer(&info->s_timer);
    info->s_timer.function = &timer_handle;
    info->s_timer.expires = jiffies + 250/HZ;//25ms
    info->s_timer.expires = (unsigned long)info;

    add_timer(&info->s_timer);

}
static void Clear_Stop_Watchdog_timer(struct tusb320_info *info)
{

    //del_timer_sync(&info->s_timer);//should not be called in interrupt contex
    del_timer(&info->s_timer);

}
#endif
#endif

static ssize_t show_current_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct tusb320_info *info = dev_get_drvdata(dev);

	switch(info->fusb_type){
		case TUSB320_TYPE_AUDIO:
			return sprintf(buf, "TUSB320_TYPE_AUDIO\n");
		case TUSB320_TYPE_DEBUG:
			return sprintf(buf, "TUSB320_TYPE_DEBUG\n");
		case TUSB320_TYPE_POWER_ACC:
			return sprintf(buf, "TUSB320_TYPE_POWER_ACC\n");
		case TUSB320_TYPE_SOURCE:
			return sprintf(buf, "TUSB320_SOURCE\n");
		case TUSB320_TYPE_SINK:
			return sprintf(buf, "TUSB320_TYPE_SINK\n");
		default:
			return sprintf(buf, "Not Connected\n");
	}

}
static ssize_t show_CC_state(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct tusb320_info *info = dev_get_drvdata(dev);

	switch(info->fusb_orient){
		case TUSB320_ORIENT_CC1_CC:
			return sprintf(buf, "cc1\n");
		case TUSB320_ORIENT_CC2_CC:
			return sprintf(buf, "cc2\n");
		case TUSB320_ORIENT_NO_CONN:
			return sprintf(buf, "disconnect\n");
        case TUSB320_ORIENT_FAULT:
        default:
			return sprintf(buf, "fault\n");
	}
}

#define REG_MOD			0x0a
static ssize_t config_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int error;
    unsigned long data;
	u8 rdata;
	struct tusb320_info *info = dev_get_drvdata(dev);

	error = sstrtoul(buf, 10, &data);
	if(error)
		return error;

	if(data == TUSB320_MODE_SRC)
	    tusb320_write_reg(info->i2c, REG_MOD, 0x20);
	else if(data == TUSB320_MODE_SNK)
	    tusb320_write_reg(info->i2c, REG_MOD, 0x10);
	else if(data == TUSB320_MODE_DRP)
	    tusb320_write_reg(info->i2c, REG_MOD, 0x30);
	else
		printk("%s: Argument is not match!!\n", __func__);

    tusb320_read_reg(info->i2c, REG_MOD, &rdata);
	printk("%s: REG_MOD(0x%02x)\n", __func__, rdata);

    return count;
}

static DEVICE_ATTR(type, S_IRUGO, show_current_type, NULL);
static DEVICE_ATTR(CC_state, S_IRUGO, show_CC_state, NULL);
static DEVICE_ATTR(mode, S_IWUSR, NULL, config_mode);

static void tusb320_check_type(struct tusb320_info *info, u8 type)
{
    if(type == 0x00)
    {
        info->fusb_type = TUSB320_TYPE_NONE;
    }
	else if(type == 0x40)
	{
	    info->fusb_type = TUSB320_TYPE_SOURCE;
	}
	else if(type == 0x80)
	{
	    info->fusb_type = TUSB320_TYPE_SINK;
	}
	else if(type == 0xc0)
	{
	    info->fusb_type = TUSB320_TYPE_POWER_ACC;
	}
	else
	{
	    printk("%s: No device type!\n", __func__);
	}

}

static void tusb320_check_orient(struct tusb320_info *info, u8 status)
{
	if(status)
    {
        info->fusb_orient = TUSB320_ORIENT_CC2_CC;
        printk("%s: Orient is CC2 %d\n", __func__,info->fusb_orient);
    }else{
        info->fusb_orient = TUSB320_ORIENT_CC1_CC;
        printk("%s: Orient is CC1 %d\n", __func__,info->fusb_orient);
    }
}

static irqreturn_t tusb320_irq_thread(int irq, void *handle)
{
    u8 intr;
    u8 current_mode;
	u8 Attached_State = 0x00;

	struct tusb320_info *info = (struct tusb320_info *)handle;

	/*
	* #ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
	* Clear_Stop_Watchdog_timer(info);
	* #endif
	*/

    tusb320_read_reg(info->i2c, 0x09, &intr);
	dev_err(&info->i2c->dev,"\n%s: from type<%d> int(0x%02x)\n", __func__,info->fusb_type, intr);

    tusb320_check_type(info, intr & 0xc0);
	dev_err(&info->i2c->dev,"%s: TYPE is %d\n", __func__, info->fusb_type);

	tusb320_check_orient(info,intr & 0x20);
	dev_err(&info->i2c->dev,"%s: Orient is %d\n", __func__, info->fusb_orient);
	if(info->fusb_type == TUSB320_TYPE_NONE)//cause tusb320 cc status default is cc2 not no connect
	{
	    info->fusb_orient = TUSB320_ORIENT_NO_CONN;
	    dev_err(&info->i2c->dev,"%s: Orient correct to %d\n", __func__, info->fusb_orient);
	}
	if(info->fusb_type == TUSB320_TYPE_SINK)//DFP attached
		{
		    // SINK
		    tusb320_read_reg(info->i2c, 0x08, &current_mode);
	        dev_err(&info->i2c->dev,"\n%s: current_mode(0x%02x)\n", __func__, current_mode);
		    if((current_mode&0x30) == 0x30){
		        dev_err(&info->i2c->dev,"%s: 3A!\n", __func__);
		        bc_notifier_call_chain(TUSB320_BC_LVL_3A);
		    }
		    if((current_mode&0x30) == 0x00 || (current_mode&0x30) == 0x20){
		        dev_err(&info->i2c->dev,"%s: default(500/900)ma!\n", __func__);
		        bc_notifier_call_chain(TUSB320_BC_LVL_USB);
		    }
		    if((current_mode&0x30) == 0x10){
		        dev_err(&info->i2c->dev,"%s: 1.5A!\n", __func__);
			    bc_notifier_call_chain(TUSB320_BC_LVL_1P5);
			}
		}

    Attached_State = intr & 0xc0;
    #ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
    if (Try_Snk_Attempt) //Try.Snk has been attempted.
    {
		if(Attached_State != 0x00)
			Try_Snk_Attempt = 0x00; //Clear flag.

        if (Attached_State == 0x40) //DFP
        {
            //Do DFP routine.
            //Enable VBUS.
            //TODO triger OTG isr,open 5V VBUS and change USB PHY to HOST

        }
        else //UFP or Accessory or Unattached
        {
            //Do UFP, or Accessory, Unattach Routines
        }
        tusb320_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Clear Interrupt status and set DRP duty cycle to default
    }
    else //Try.Snk has NOT been attempted
    {
        if (Attached_State == 0x40) //DFP. Perform Try.SNK.
        {
            Try_Snk_Attempt = 0x01; //Set Try_Snk flag.
            tusb320_soft_reset(info->i2c);//write_csr(0x0A, 0x08); //Set Soft Reset
            mdelay(25);//wait_time(25); // Wait 25ms.
            tusb320_soft_reset(info->i2c);//write_csr(0x0A, 0x08); // Set Soft Reset
            //Start_WatchDog_Timer(info);
            tusb320_change_DRP_duty_cycle_and_clear_intr(info->i2c);//write_csr(0x09, 0x16); // Change DRP duty Cycle and Interrupt status
            }
        else // NOT DFP
        {
            //Do UFP ,or Accessory, Unattach Routines
            Try_Snk_Attempt = 0x00;
            tusb320_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Clear Interrupt Status
        }
    }
    #else
    tusb320_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Clear Interrupt Status
    #endif

    return IRQ_HANDLED;
}

static void tusb320_reboot_scan_otg(struct tusb320_info *info)
{
    u8 intr;
    u8 current_mode;
	u8 Attached_State = 0x00;

    tusb320_read_reg(info->i2c, 0x09, &intr);
	dev_err(&info->i2c->dev,"\n%s: from type<%d> int(0x%02x)\n", __func__,info->fusb_type, intr);

    tusb320_check_type(info, intr & 0xc0);
	dev_err(&info->i2c->dev,"%s: TYPE is %d\n", __func__, info->fusb_type);

	tusb320_check_orient(info,intr & 0x20);
	dev_err(&info->i2c->dev,"%s: Orient is %d\n", __func__, info->fusb_orient);
	if(info->fusb_type == TUSB320_TYPE_NONE)//cause tusb320 cc status default is cc2 not no connect
	{
	    info->fusb_orient = TUSB320_ORIENT_NO_CONN;
	    dev_err(&info->i2c->dev,"%s: Orient correct to %d\n", __func__, info->fusb_orient);
	}
	if(info->fusb_type == TUSB320_TYPE_SINK)//DFP attached
		{
		    // SINK
		    tusb320_read_reg(info->i2c, 0x08, &current_mode);
	        dev_err(&info->i2c->dev,"\n%s: current_mode(0x%02x)\n", __func__, current_mode);
		    if((current_mode&0x30) == 0x30){
		        dev_err(&info->i2c->dev,"%s: 3A!\n", __func__);
		        bc_notifier_call_chain(TUSB320_BC_LVL_3A);
		    }
		    if((current_mode&0x30) == 0x00 || (current_mode&0x30) == 0x20){
		        dev_err(&info->i2c->dev,"%s: default(500/900)ma!\n", __func__);
		        bc_notifier_call_chain(TUSB320_BC_LVL_USB);
		    }
		    if((current_mode&0x30) == 0x10){
		        dev_err(&info->i2c->dev,"%s: 1.5A!\n", __func__);
			    bc_notifier_call_chain(TUSB320_BC_LVL_1P5);
			}
		}

    Attached_State = intr & 0xc0;
    #ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP
    if (Try_Snk_Attempt) //Try.Snk has been attempted.
    {
		if(Attached_State != 0x00)
			Try_Snk_Attempt = 0x00; //Clear flag.

        if (Attached_State == 0x40) //DFP
        {
            //Do DFP routine.
            //Enable VBUS.
            //TODO triger OTG isr,open 5V VBUS and change USB PHY to HOST

        }
        else //UFP or Accessory or Unattached
        {
            //Do UFP, or Accessory, Unattach Routines
        }
        tusb320_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Clear Interrupt status and set DRP duty cycle to default
    }
    else //Try.Snk has NOT been attempted
    {
        if (Attached_State == 0x40) //DFP. Perform Try.SNK.
        {
            Try_Snk_Attempt = 0x01; //Set Try_Snk flag.
            tusb320_soft_reset(info->i2c);//write_csr(0x0A, 0x08); //Set Soft Reset
            mdelay(25);//wait_time(25); // Wait 25ms.
            tusb320_soft_reset(info->i2c);//write_csr(0x0A, 0x08); // Set Soft Reset
            //Start_WatchDog_Timer(info);
            tusb320_change_DRP_duty_cycle_and_clear_intr(info->i2c);//write_csr(0x09, 0x16); // Change DRP duty Cycle and Interrupt status
            }
        else // NOT DFP
        {
            //Do UFP ,or Accessory, Unattach Routines
            Try_Snk_Attempt = 0x00;
            tusb320_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Clear Interrupt Status
        }
    }
    #else
    tusb320_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Clear Interrupt Status
    #endif
}

static void tusb320_otg_work(struct work_struct *work)
{
    struct tusb320_info *info = container_of(work, struct tusb320_info, otg_work);
    bool otg_present = !gpio_get_value(info->ID_gpio);
    dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,otg_present);
    power_supply_set_usb_otg(info->usb_psy, otg_present ? 1 : 0);
}

static irqreturn_t tusb320_otg_irq_thread(int irq, void *handle)
{
    struct tusb320_info *info = (struct tusb320_info *)handle;
    wake_lock_timeout(&info->otg_wl, msecs_to_jiffies(OTG_WL_HOLD_TIME));
    queue_work(info->otg_wqueue, &info->otg_work);
    return IRQ_HANDLED;
}

static void tusb320_initialization(struct tusb320_info *info)
{
	info->fusb_type = TUSB320_TYPE_NONE;
	info->fusb_orient= TUSB320_ORIENT_NO_CONN;
}

static int tusb320_gpio_configure(struct tusb320_info *info)
{
	int retval = 0;

		if (gpio_is_valid(info->irq_gpio)) {
			/* configure touchscreen irq gpio */
			retval = gpio_request(info->irq_gpio,
				"tusb320_irq_gpio");
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
					"tusb320_OTG_USB_ID_gpio");
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
			info->OTG_USB_ID_irq = gpio_to_irq(info->ID_gpio);
		if (info->OTG_USB_ID_irq < 0) {
			dev_err(&info->i2c->dev,
				"Unable to get irq number for GPIO %d, error %d\n",
					info->ID_gpio, info->OTG_USB_ID_irq);
			retval = info->OTG_USB_ID_irq;
			goto err_irq_gpio_dir;
		}
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
static struct tusb320_info *ginfo;

static int tusb320_power_down_callback(
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
        disable_irq_wake(ginfo->OTG_USB_ID_irq);
        free_irq(ginfo->OTG_USB_ID_irq, ginfo);
        //set to UFP to get a chance to make sure get charging from other DRP typeC device
        tusb320_write_reg(ginfo->i2c, REG_MOD, 0x10);//0x0a,0x10  MOD_SNK
        power_supply_set_usb_otg(ginfo->usb_psy, 0);
        break;
    default:
        return NOTIFY_DONE;
    }

    return NOTIFY_OK;
}

static struct notifier_block tusb320_reboot_notifier = {
    .notifier_call = tusb320_power_down_callback,
};
static int tusb320_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	u8 rdata = 0;
	struct tusb320_info *info;
	struct device_node *np = client->dev.of_node;
	struct power_supply *usb_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB power_supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}


	info = kzalloc(sizeof(struct tusb320_info), GFP_KERNEL);
	info->i2c = client;
	info->usb_psy = usb_psy;
	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	dev_err(&client->dev, "%s\n",__func__);

	tusb320_read_reg(info->i2c, 0x00, &rdata);
	if(rdata != 0x30){
	    dev_err(&client->dev, "DEV_ID(0x%x)!= 0x30\n",rdata);
	    ret = -EINVAL;
	    goto parse_dt_failed;
	}
	else
	{
	    dev_err(&client->dev, "DEV_ID(0x%x) OK!\n",rdata);
	}

    /* USBID, irq gpio info */
	info->ID_gpio = of_get_named_gpio(np,"tusb320,ID-gpio", 0);
	info->irq_gpio = of_get_named_gpio(np,"tusb320,irq-gpio", 0);
	ret = tusb320_gpio_configure(info);
	if(ret != 0)
	{
	    goto parse_dt_failed;
	}

	info->fusb_class = class_create(THIS_MODULE, "type-c-tusb320");
	info->dev_t = device_create(info->fusb_class, NULL, 0, NULL, "tusb320");
	device_create_file(info->dev_t, &dev_attr_type);
	device_create_file(info->dev_t, &dev_attr_mode);
	device_create_file(info->dev_t, &dev_attr_CC_state);
	dev_set_drvdata(info->dev_t, info);


    info->otg_wqueue = create_singlethread_workqueue("tusb320_otg_wqueue");
	INIT_WORK(&info->otg_work, tusb320_otg_work);

	ret = request_threaded_irq(info->irq, NULL, tusb320_irq_thread,
			  IRQF_TRIGGER_LOW | IRQF_ONESHOT, "tusb320_irq", info);

	if (ret){
	    dev_err(&client->dev, "failed to reqeust IRQ\n");
	    goto request_irq_failed;
	}

	wake_lock_init(&info->otg_wl, WAKE_LOCK_SUSPEND, "tusb320_otg_wl");
    /* Update initial state to USB */
    if(!gpio_get_value(info->ID_gpio)){
	    tusb320_otg_irq_thread(info->OTG_USB_ID_irq, info);
		tusb320_reboot_scan_otg(info);/*Anderson-triger_scan_otg_after_reboot*/
	}
    /*
	ret = request_threaded_irq(info->OTG_USB_ID_irq, NULL, tusb320_otg_irq_thread,
			  IRQF_TRIGGER_FALLING| IRQF_ONESHOT, "tusb320_otg_irq", info);
	*/
    ret = request_irq(info->OTG_USB_ID_irq, tusb320_otg_irq_thread,
			  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "tusb320_otg_irq", info);

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
	ret = enable_irq_wake(info->OTG_USB_ID_irq);
	if (ret < 0){
	    dev_err(&client->dev,
		 "failed to enable wakeup src %d\n", ret);
		goto enable_irq_failed;
	}

/*
 Add by yangrujin@bsp 2015/1/25, fix [BUG] RAIN-405: connect OTG and U-Disk to device,
 then shutdown device, device will auto reboot instead of pwroff.
 */
    ginfo = info;
    register_reboot_notifier(&tusb320_reboot_notifier);

    tusb320_initialization(info);
    dev_err(&info->i2c->dev,"%s OK!\n",__func__);
	return 0;

enable_irq_failed:
	free_irq(info->irq,NULL);
	free_irq(info->OTG_USB_ID_irq,NULL);
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

static int tusb320_remove(struct i2c_client *client)
{
    struct tusb320_info *info = i2c_get_clientdata(client);

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


static int  tusb320_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  tusb320_resume(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id tusb320_id[] = {
		{.compatible = "tusb320"},
		{},
};
MODULE_DEVICE_TABLE(of, tusb320_id);


static const struct i2c_device_id tusb320_i2c_id[] = {
	{ "tusb320", 0 },
	{ }
};

static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		.name = "tusb320",
		.of_match_table = of_match_ptr(tusb320_id),
	},
	.probe    = tusb320_probe,
	.remove   = tusb320_remove,
	.suspend  = tusb320_suspend,
	.resume	  = tusb320_resume,
	.id_table = tusb320_i2c_id,
};

static __init int tusb320_i2c_init(void)
{

	return i2c_add_driver(&tusb320_i2c_driver);
}

static __exit void tusb320_i2c_exit(void)
{
	i2c_del_driver(&tusb320_i2c_driver);
}

module_init(tusb320_i2c_init);
module_exit(tusb320_i2c_exit);

MODULE_AUTHOR("oem@OP");
MODULE_DESCRIPTION("I2C bus driver for TUSB320 USB Type-C");
MODULE_LICENSE("GPL v2");
