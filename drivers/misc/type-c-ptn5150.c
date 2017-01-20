/*
 * ptn5150.c -- PTN5150 USB TYPE-C Controller device driver
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

#include <linux/type-c_notifier.h>

#include <linux/version.h>
#include <linux/power_supply.h>

/******************************************************************************/

//#define USE_TIMER_WHEN_DFP_TO_DETETC_UFP
#define OTG_WL_HOLD_TIME 5000

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38))
#define KERNEL_ABOVE_2_6_38
#endif

#ifdef KERNEL_ABOVE_2_6_38
#define sstrtoul(...) kstrtoul(__VA_ARGS__)
#else
#define sstrtoul(...) strict_strtoul(__VA_ARGS__)
#endif


#define PTN5150_ATTACH	1
#define PTN5150_DETACH	0

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_DEV_ID		0x01
#define REG_MOD			0x02
#define REG_CON			0x03
#define REG_STAT		0x04
#define REG_RST			0x10
#define REG_MSK			0x18
#define REG_INT			0x19

enum ptn5150_type{
	PTN5150_TYPE_NONE = 0,
	PTN5150_TYPE_AUDIO,
	PTN5150_TYPE_DEBUG,
	PTN5150_TYPE_POWER_ACC,
	PTN5150_TYPE_SOURCE,
	PTN5150_TYPE_SINK
};

enum ptn5150_bc_lvl{
	PTN5150_BC_LVL_RA = 0,
	PTN5150_BC_LVL_USB,
	PTN5150_BC_LVL_1P5,
	PTN5150_BC_LVL_3A
};

enum ptn5150_drp_toggle{
	PTN5150_TOGGLE_SNK35_SRC15 = 0,  // default
	PTN5150_TOGGLE_SNK30_SRC20,
	PTN5150_TOGGLE_SNK25_SRC25,
	PTN5150_TOGGLE_SNK20_SRC30,
};

enum ptn5150_host_cur{
	PTN5150_HOST_CUR_NO = 0,  // no current
	PTN5150_HOST_CUR_80,  // default USB
	PTN5150_HOST_CUR_180,  // 1.5A
	PTN5150_HOST_CUR_330,  // 3A
};

enum ptn5150_orient{
	PTN5150_ORIENT_NO_CONN = 0,
	PTN5150_ORIENT_CC1_CC,
	PTN5150_ORIENT_CC2_CC,
	PTN5150_ORIENT_FAULT
};

enum ptn5150_config_modes{
	PTN5150_MODE_SRC = 0,
	PTN5150_MODE_SRC_ACC,
	PTN5150_MODE_SNK,
	PTN5150_MODE_SNK_ACC,
	PTN5150_MODE_DRP,
	PTN5150_MODE_DRP_ACC
};

#define	PTN5150_I2C_ADDRESS		0x3a		// When ADR pin is pulled down, address = 0x3a
											// When ADR pin is pulled up, address = x07a

#define	PTN5150_VERSION_REG			0x01

#define	VERSION_ID_MASK			0xf8
#define	VENDOR_ID_MASK			0x07

#define	PTN5150_CONTROL_REG			0x02

#define	RP_SELECT_DEFULAT		0x00
#define	RP_SELECT_MEDIUM		0x08
#define	RP_SELECT_HIGH			0x18

#define	PORT_SET_UFP			0x00
#define	PORT_SET_DFP			0x02
#define	PORT_SET_DRP			0x04

#define	ATTACH_INT_MASK_EN		0x00
#define	ATTACH_INT_MASK_DIS		0x01

#define	PTN5150_INT_STATUS_REG		0x03

#define	CABLE_DETACH_INT		0x02
#define	CABLE_ATTACH_INT		0x01

#define	PTN5150_CC_STATUS_REG		0x04

#define	VBUS_DETECTION			0x80

#define	RP_DET_AS_NONE			0x00
#define	RP_DET_AS_DEFAULT		0x01
#define	RP_DET_AS_MEDIUM		0x02
#define	RP_DET_AS_HIGH			0x03

#define	ATTACHED_IS_NONE		0x00
#define	ATTACHED_IS_DFP			0x01
#define	ATTACHED_IS_UFP			0x02
#define	ATTACHED_IS_AUDIO		0x03
#define	ATTACHED_IS_DEBUG		0x04

#define	CC_POLARITY_NONE		0x00
#define	CC_POLARITY_CC1			0x01
#define	CC_POLARITY_CC2			0x02

#define	PTN5150_CONDET_REG			0x09

#define	CON_DET_EN				0x00
#define	CON_DET_DIS				0x01

#define	PTN5150_VCONN_REG			0x0a

#define	VCONN_STANDBY			0x00
#define	VCONN_CC1				0x01
#define	VCONN_CC2				0x02

#define	PTN5150_VCONN_ACCESS_REG		0x43

#define	VCONN_ACCESS_CODE		0xe0

struct ptn5150_info {
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
	enum ptn5150_type fusb_type;
	enum ptn5150_orient fusb_orient;
	struct timer_list s_timer;
	struct work_struct otg_work;
	struct workqueue_struct *otg_wqueue;
	struct wake_lock otg_wl;
};

#if 0
static int ptn5150_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct ptn5150_info *info = i2c_get_clientdata(i2c);
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

static int ptn5150_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct ptn5150_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		printk("%s:reg(0x%x), ret(%d)\n",
				__func__, reg, ret);

	return ret;
}
#else
static int ptn5150_read_reg(struct i2c_client *client, u8 addr, u8 *pdata)
{
	//struct ptn5150_info *info = i2c_get_clientdata(i2c);
	int ret = -1;
	u8 buf[2] = {addr, 0};

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf+1,
		},
	};

	if (i2c_transfer(client->adapter, msgs, 2) != 2) {
		ret = -EIO;
        pr_err("%s: i2c read error: %d\n", __func__, ret);
	}
	*pdata = buf[1];

	return ret;
}

static int ptn5150_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	//struct ptn5150_info *info = i2c_get_clientdata(i2c);
	//u8 buf[3];
	u8 buf[2] = {reg, value};
	int ret = 0;

    struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= buf,
		},
	};

	if (i2c_transfer(client->adapter, msg, 1) != 1) {
		ret = -EIO;
        pr_err("%s: i2c write error: %d\n", __func__, ret);
	}
	return ret;
}
#endif
/*
static int ptn5150_update_reg(struct i2c_client *i2c, u8 reg, u8 val, u8 mask)
{
	struct ptn5150_info *info = i2c_get_clientdata(i2c);
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


#ifdef USE_TIMER_WHEN_DFP_TO_DETETC_UFP

static int ptn5150_clear_intr(struct i2c_client *i2c)
{
    struct ptn5150_info *info = i2c_get_clientdata(i2c);
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
static int ptn5150_soft_reset(struct i2c_client *i2c)
{
	struct ptn5150_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	/*
	ret = i2c_smbus_read_byte_data(i2c, 0x10);
	if (ret >= 0) {
		u8 old_val = ret & 0xff;
		u8 new_val = (old_val | 0x01);
		ret = i2c_smbus_write_byte_data(i2c, 0x10, new_val);
	}*/
	ret = i2c_smbus_write_byte_data(i2c, 0x10, 0x01);
	mutex_unlock(&info->mutex);
	return ret;
}
static int ptn5150_change_DRP_duty_cycle_and_clear_intr(struct i2c_client *i2c)
{
    struct ptn5150_info *info = i2c_get_clientdata(i2c);
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

u8 Try_Snk_Attempt = 0x00; //Global Variable for Try.Snk attempt.
static void timer_handle(unsigned long data)
{
    struct ptn5150_info *info = (struct ptn5150_info *)data;
    Try_Snk_Attempt = 0x00; //Clear flag
    ptn5150_clear_intr(info->i2c);//write_csr(0x09, 0x10); //Set DRP duty cycle to default
}
static void Start_WatchDog_Timer(struct ptn5150_info *info)
{
	init_timer(&info->s_timer);
    info->s_timer.function = &timer_handle;
    info->s_timer.expires = jiffies + 250/HZ;//25ms
    info->s_timer.expires = (unsigned long)info;

    add_timer(&info->s_timer);

}
static void Clear_Stop_Watchdog_timer(struct ptn5150_info *info)
{

    //del_timer_sync(&info->s_timer);//should not be called in interrupt contex
    del_timer(&info->s_timer);

}
#endif

static ssize_t show_current_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ptn5150_info *info = dev_get_drvdata(dev);

	switch(info->fusb_type){
		case PTN5150_TYPE_AUDIO:
			return sprintf(buf, "PTN5150_TYPE_AUDIO\n");
		case PTN5150_TYPE_DEBUG:
			return sprintf(buf, "PTN5150_TYPE_DEBUG\n");
		case PTN5150_TYPE_POWER_ACC:
			return sprintf(buf, "PTN5150_TYPE_POWER_ACC\n");
		case PTN5150_TYPE_SOURCE:
			return sprintf(buf, "PTN5150_SOURCE\n");
		case PTN5150_TYPE_SINK:
			return sprintf(buf, "PTN5150_TYPE_SINK\n");
		default:
			return sprintf(buf, "Not Connected\n");
	}

}
static ssize_t show_CC_state(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ptn5150_info *info = dev_get_drvdata(dev);

	switch(info->fusb_orient){
		case PTN5150_ORIENT_CC1_CC:
			return sprintf(buf, "cc1\n");
		case PTN5150_ORIENT_CC2_CC:
			return sprintf(buf, "cc2\n");
		case PTN5150_ORIENT_NO_CONN:
			return sprintf(buf, "disconnect\n");
		case PTN5150_ORIENT_FAULT:
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
	struct ptn5150_info *info = dev_get_drvdata(dev);

	error = sstrtoul(buf, 10, &data);
	if(error)
		return error;

	if(data == PTN5150_MODE_SRC)
	    ptn5150_write_reg(info->i2c, REG_MOD, 0x02);
	else if(data == PTN5150_MODE_SNK)
	    ptn5150_write_reg(info->i2c, REG_MOD, 0x00);
	else if(data == PTN5150_MODE_DRP)
	    ptn5150_write_reg(info->i2c, REG_MOD, 0x04);
	else
		printk("%s: Argument is not match!!\n", __func__);

    ptn5150_read_reg(info->i2c, REG_MOD, &rdata);
	printk("%s: REG_MOD(0x%02x)\n", __func__, rdata);

    return count;
}

static ssize_t reg_dump(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    u8 i = 0x00;
	u8 rdata;
	struct ptn5150_info *info = dev_get_drvdata(dev);

    for(i= 0x01; i <=0x19; i++)
    {
        ptn5150_read_reg(info->i2c, i, &rdata);
	    dev_err(&info->i2c->dev,"%s:[dump] REG0x%02x(0x%02x)\n", __func__,i, rdata);
    }
	return sprintf(buf, "ok\n");
}

static DEVICE_ATTR(type, S_IRUGO, show_current_type, NULL);
static DEVICE_ATTR(CC_state, S_IRUGO, show_CC_state, NULL);
static DEVICE_ATTR(reg, S_IRUGO, reg_dump, NULL);
static DEVICE_ATTR(mode, S_IWUSR, NULL, config_mode);

static void ptn5150_check_type(struct ptn5150_info *info, u8 type)
{
    switch(type & 0x1c)
    {
        case 0x04:
            info->fusb_type = PTN5150_TYPE_SOURCE;
            break;
        case 0x08:
            info->fusb_type = PTN5150_TYPE_SINK;
            break;
        case 0x0c:
            info->fusb_type = PTN5150_TYPE_AUDIO;
            break;
        case 0x10:
            info->fusb_type = PTN5150_TYPE_DEBUG;
            break;
        case 0x1c:
            info->fusb_type = PTN5150_TYPE_POWER_ACC;
            break;
        default:
            printk("%s: No device type!\n", __func__);
    }

}

static void ptn5150_check_orient(struct ptn5150_info *info, u8 status)
{
	if((status&0x03) == 0x02)
    {
        info->fusb_orient = PTN5150_ORIENT_CC2_CC;
        printk("%s: Orient is CC2 %d\n", __func__,info->fusb_orient);
    }else if((status&0x03) == 0x01){
        info->fusb_orient = PTN5150_ORIENT_CC1_CC;
        printk("%s: Orient is CC1 %d\n", __func__,info->fusb_orient);
    }
    else{
        info->fusb_orient = PTN5150_ORIENT_NO_CONN;
        printk("%s: Orient is Cable Not Attached %d\n", __func__,info->fusb_orient);
    }
}

static irqreturn_t ptn5150_irq_thread(int irq, void *handle)
{
    u8 intr,rdata;
	//int bc_lvl;
	struct ptn5150_info *info = (struct ptn5150_info *)handle;

    //ptn5150_read_reg(info->i2c, REG_INT, &intr);
	//dev_err(&info->i2c->dev,"\n%s: type<%d> intr(0x%02x)\n", __func__,info->fusb_type, intr);

	ptn5150_read_reg(info->i2c, REG_CON, &intr);
	dev_err(&info->i2c->dev,"\n%s: con(0x%02x)\n", __func__, intr);

	if(intr & 0x01)
	{
	    dev_err(&info->i2c->dev,"%s: Attach interrupt!\n", __func__);
		ptn5150_read_reg(info->i2c, REG_STAT, &rdata);
		dev_err(&info->i2c->dev,"\n%s: STAT(0x%02x)\n", __func__, rdata);
		ptn5150_check_type(info, rdata);
		dev_err(&info->i2c->dev,"%s:Port Attached type %d\n", __func__, info->fusb_type);
		ptn5150_check_orient(info,rdata);
		dev_err(&info->i2c->dev,"%s: Orient is %d\n", __func__, info->fusb_orient);

		if(info->fusb_type == PTN5150_TYPE_SINK)//UFP attached
		{
		    //SOURCE
		    //ptn5150_read_reg(info->i2c, REG_STAT, &rdata);
			//bc_lvl = (rdata & STAT_BC_LVL) >> STAT_BC_LVL_SHIFT;
			//TODO triger OTG isr,open 5V VBUS and change USB PHY to HOST
		}
		else if(info->fusb_type == PTN5150_TYPE_SOURCE)//DFP attached
		{
		    // SINK

		    if((rdata&0x60) == 0x60){
		        dev_err(&info->i2c->dev,"%s: 3A!\n", __func__);
		        bc_notifier_call_chain(PTN5150_BC_LVL_3A);
		    }
		    if((rdata&0x60) == 0x20){
		        dev_err(&info->i2c->dev,"%s: default(500/900)ma!\n", __func__);
		        bc_notifier_call_chain(PTN5150_BC_LVL_USB);
		    }
		    if((rdata&0x60) == 0x40){
		        dev_err(&info->i2c->dev,"%s: 1.5A!\n", __func__);
			    bc_notifier_call_chain(PTN5150_BC_LVL_1P5);
			}
		}

	}
	if(intr & 0x02)
	{
	    // Detach
	    dev_err(&info->i2c->dev,"%s: Detach interrupt!\n", __func__);
     	info->fusb_type = PTN5150_TYPE_NONE;
        info->fusb_orient= PTN5150_ORIENT_NO_CONN;
	}
	else
	{
	    dev_err(&info->i2c->dev,"%s: interrupt!\n", __func__);
		ptn5150_read_reg(info->i2c, REG_STAT, &rdata);
		dev_err(&info->i2c->dev,"\n%s: STAT(0x%02x)\n", __func__, rdata);
		ptn5150_check_type(info, rdata);
		dev_err(&info->i2c->dev,"%s: Port Attached type %d\n", __func__, info->fusb_type);
		ptn5150_check_orient(info,rdata);
		dev_err(&info->i2c->dev,"%s: Orient is %d\n", __func__, info->fusb_orient);

		if(info->fusb_type == PTN5150_TYPE_SINK)//UFP attached
		{
		    //SOURCE
		    //ptn5150_read_reg(info->i2c, REG_STAT, &rdata);
			//bc_lvl = (rdata & STAT_BC_LVL) >> STAT_BC_LVL_SHIFT;
			//TODO triger OTG isr,open 5V VBUS and change USB PHY to HOST
		}
		else if(info->fusb_type == PTN5150_TYPE_SOURCE)//DFP attached
		{
		    // SINK
		    if((rdata&0x60) == 0x60)
		        dev_err(&info->i2c->dev,"%s: 3A!\n", __func__);
		    if((rdata&0x60) == 0x20)
		        dev_err(&info->i2c->dev,"%s: default(500/900)ma!\n", __func__);
		    if((rdata&0x60) == 0x40)
		        dev_err(&info->i2c->dev,"%s: 1.5A!\n", __func__);
		}
	}
    return IRQ_HANDLED;
}
#if 0
static void ptn5150_reboot_scan_otg(struct ptn5150_info *info)
{
    u8 rdata;

	dev_err(&info->i2c->dev,"%s: Attach interrupt!\n", __func__);
	ptn5150_read_reg(info->i2c, REG_STAT, &rdata);
	dev_err(&info->i2c->dev,"\n%s: STAT(0x%02x)\n", __func__, rdata);
	ptn5150_check_type(info, rdata);
	dev_err(&info->i2c->dev,"%s:Port Attached type %d\n", __func__, info->fusb_type);
	ptn5150_check_orient(info,rdata);
	dev_err(&info->i2c->dev,"%s: Orient is %d\n", __func__, info->fusb_orient);

	if(info->fusb_type == PTN5150_TYPE_SINK)//UFP attached
	{
		//SOURCE
		//ptn5150_read_reg(info->i2c, REG_STAT, &rdata);
		//bc_lvl = (rdata & STAT_BC_LVL) >> STAT_BC_LVL_SHIFT;
		//TODO triger OTG isr,open 5V VBUS and change USB PHY to HOST
	}
	else if(info->fusb_type == PTN5150_TYPE_SOURCE)//DFP attached
	{
		// SINK

		if((rdata&0x60) == 0x60){
			dev_err(&info->i2c->dev,"%s: 3A!\n", __func__);
			bc_notifier_call_chain(PTN5150_BC_LVL_3A);
		}
		if((rdata&0x60) == 0x20){
			dev_err(&info->i2c->dev,"%s: default(500/900)ma!\n", __func__);
			bc_notifier_call_chain(PTN5150_BC_LVL_USB);
		}
		if((rdata&0x60) == 0x40){
			dev_err(&info->i2c->dev,"%s: 1.5A!\n", __func__);
			bc_notifier_call_chain(PTN5150_BC_LVL_1P5);
		}
	}
}
#endif
static void ptn5150_otg_work(struct work_struct *work)
{
    struct ptn5150_info *info = container_of(work, struct ptn5150_info, otg_work);
    bool otg_present = !gpio_get_value(info->ID_gpio);
    dev_err(&info->i2c->dev,"%s : otg_present = (%d)\n",__func__,otg_present);
    power_supply_set_usb_otg(info->usb_psy, otg_present ? 1 : 0);
}

static irqreturn_t ptn5150_otg_irq_thread(int irq, void *handle)
{
    struct ptn5150_info *info = (struct ptn5150_info *)handle;
    wake_lock_timeout(&info->otg_wl, msecs_to_jiffies(OTG_WL_HOLD_TIME));
    queue_work(info->otg_wqueue, &info->otg_work);
	return IRQ_HANDLED;
}

static void ptn5150_initialization(struct ptn5150_info *info)
{
	info->fusb_type = PTN5150_TYPE_NONE;
	info->fusb_orient= PTN5150_ORIENT_NO_CONN;

    ptn5150_write_reg(info->i2c, REG_MOD, 0x04);//MOD_DRP

    //ptn5150_write_reg(info->i2c, REG_MSK, 0x00);  //unmask global interrupts

	//ptn5150_write_reg (info->i2c, PTN5150_VCONN_ACCESS_REG,VCONN_ACCESS_CODE);			// This is to allow VCONN register access
	//ptn5150_write_reg (info->i2c, PTN5150_CONDET_REG,CON_DET_EN);				// Enable CON_DET if you use this signal

	//ptn5150_write_reg (info->i2c, PTN5150_CONTROL_REG,(/*RP_SELECT_HIGH |*/ PORT_SET_DRP | ATTACH_INT_MASK_EN));		// Configure PTN5150 (change if necessary)

}

static int ptn5150_gpio_configure(struct ptn5150_info *info)
{
	int retval = 0;

		if (gpio_is_valid(info->irq_gpio)) {
			/* configure touchscreen irq gpio */
			retval = gpio_request(info->irq_gpio,
				"ptn5150_irq_gpio");
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
					"ptn5150_OTG_USB_ID_gpio");
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

#include <linux/notifier.h>
#include <linux/reboot.h>
static struct ptn5150_info *ginfo;

static int ptn5150_power_down_callback(
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
        ptn5150_write_reg(ginfo->i2c, REG_MOD, 0x00);//0x02,0x00  MOD_SNK
        power_supply_set_usb_otg(ginfo->usb_psy, 0);
        break;
    default:
        return NOTIFY_DONE;
    }

    return NOTIFY_OK;
}

static struct notifier_block ptn5150_reboot_notifier = {
    .notifier_call = ptn5150_power_down_callback,
};
static int ptn5150_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	u8 rdata = 0;
	struct ptn5150_info *info;
	struct device_node *np = client->dev.of_node;
	struct power_supply *usb_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB power_supply not found, defer probe\n");
		return -EPROBE_DEFER;
	}


	info = kzalloc(sizeof(struct ptn5150_info), GFP_KERNEL);
	info->i2c = client;
	info->usb_psy = usb_psy;
	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	dev_err(&client->dev, "%s\n",__func__);

	ptn5150_read_reg(info->i2c, 0x01, &rdata);
	if(rdata != 0x0b && rdata !=0x09){
	    dev_err(&client->dev, "DEV_ID(0x%02x)!= 0x0b || 0x09\n",rdata);
	    ret = -EINVAL;
	    goto parse_dt_failed;
	}
	else
	{
	    dev_err(&client->dev, "DEV_ID(0x%x) OK!\n",rdata);
	}

    /* USBID, irq gpio info */
	info->ID_gpio = of_get_named_gpio(np,"ptn5150,ID-gpio", 0);
	info->irq_gpio = of_get_named_gpio(np,"ptn5150,irq-gpio", 0);
	ret = ptn5150_gpio_configure(info);
	if(ret != 0)
	{
	    goto parse_dt_failed;
	}

	info->fusb_class = class_create(THIS_MODULE, "type-c-ptn5150");
	info->dev_t = device_create(info->fusb_class, NULL, 0, NULL, "ptn5150");
	device_create_file(info->dev_t, &dev_attr_type);
	device_create_file(info->dev_t, &dev_attr_reg);
	device_create_file(info->dev_t, &dev_attr_mode);
	device_create_file(info->dev_t, &dev_attr_CC_state);
	dev_set_drvdata(info->dev_t, info);

    ptn5150_initialization(info);

    info->otg_wqueue = create_singlethread_workqueue("ptn5150_otg_wqueue");
	INIT_WORK(&info->otg_work, ptn5150_otg_work);

	ret = request_threaded_irq(info->irq, NULL, ptn5150_irq_thread,
			  IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ptn5150_irq", info);

	if (ret){
	    dev_err(&client->dev, "failed to reqeust IRQ\n");
	    goto request_irq_failed;
	}

	wake_lock_init(&info->otg_wl, WAKE_LOCK_SUSPEND, "ptn5150_otg_wl");
    /* Update initial state to USB */
    //if(!gpio_get_value(info->ID_gpio)){
	//    ptn5150_otg_irq_thread(info->OTG_USB_ID_irq, info);
	//	  ptn5150_reboot_scan_otg(info);/*Anderson-triger_scan_otg_after_reboot*/
	//}
	/*
	ret = request_threaded_irq(info->OTG_USB_ID_irq, NULL, ptn5150_otg_irq_thread,
			  IRQF_TRIGGER_FALLING| IRQF_TRIGGER_RISING, "ptn5150_otg_irq", info);
    */
	ret = request_irq(info->OTG_USB_ID_irq, ptn5150_otg_irq_thread,
			  IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "ptn5150_otg_irq", info);

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

    ginfo = info;
    register_reboot_notifier(&ptn5150_reboot_notifier);

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

static int ptn5150_remove(struct i2c_client *client)
{
    struct ptn5150_info *info = i2c_get_clientdata(client);

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


static int  ptn5150_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  ptn5150_resume(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id ptn5150_id[] = {
		{.compatible = "ptn5150"},
		{},
};
MODULE_DEVICE_TABLE(of, ptn5150_id);


static const struct i2c_device_id ptn5150_i2c_id[] = {
	{ "ptn5150", 0 },
	{ }
};

static struct i2c_driver ptn5150_i2c_driver = {
	.driver = {
		.name = "ptn5150",
		.of_match_table = of_match_ptr(ptn5150_id),
	},
	.probe    = ptn5150_probe,
	.remove   = ptn5150_remove,
	.suspend  = ptn5150_suspend,
	.resume	  = ptn5150_resume,
	.id_table = ptn5150_i2c_id,
};

static __init int ptn5150_i2c_init(void)
{

	return i2c_add_driver(&ptn5150_i2c_driver);
}

static __exit void ptn5150_i2c_exit(void)
{
	i2c_del_driver(&ptn5150_i2c_driver);
}

module_init(ptn5150_i2c_init);
module_exit(ptn5150_i2c_exit);

MODULE_AUTHOR("oem@OP");
MODULE_DESCRIPTION("I2C bus driver for PTN5150 USB Type-C");
MODULE_LICENSE("GPL v2");
