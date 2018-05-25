/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "BQ: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/time.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/err.h>
#include <linux/rtc.h>

#ifdef CONFIG_OF
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#endif
#include <linux/wakelock.h>
#include "oem_external_fg.h"


#define DRIVER_VERSION			"1.1.0"
/* Bq27541 standard data commands */
#define BQ27541_REG_CNTL		0x00
#define BQ27541_REG_AR			0x02
#define BQ27541_REG_ARTTE		0x04
#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_NAC			0x0C
#define BQ27541_REG_FAC			0x0e
#define BQ27541_REG_RM			0x10
#define BQ27541_REG_FCC			0x12
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_TTE			0x16
#define BQ27541_REG_TTF			0x18
#define BQ27541_REG_SI			0x1a
#define BQ27541_REG_STTE		0x1c
#define BQ27541_REG_MLI			0x1e
#define BQ27541_REG_MLTTE		0x20
#define BQ27541_REG_AE			0x22
#define BQ27541_REG_AP			0x24
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOH			0x28
#define BQ27541_REG_SOC			0x2c
#define BQ27541_REG_NIC			0x2e
#define BQ27541_REG_ICR			0x30
#define BQ27541_REG_LOGIDX		0x32
#define BQ27541_REG_LOGBUF		0x34

#define BQ27541_FLAG_DSC		BIT(0)
#define BQ27541_FLAG_FC			BIT(9)

#define BQ27541_CS_DLOGEN		BIT(15)
#define BQ27541_CS_SS		    BIT(13)

/* Control subcommands */
#define BQ27541_SUBCMD_CTNL_STATUS  0x0000
#define BQ27541_SUBCMD_DEVCIE_TYPE  0x0001
#define BQ27541_SUBCMD_FW_VER  0x0002
#define BQ27541_SUBCMD_HW_VER  0x0003
#define BQ27541_SUBCMD_DF_CSUM  0x0004
#define BQ27541_SUBCMD_PREV_MACW   0x0007
#define BQ27541_SUBCMD_CHEM_ID   0x0008
#define BQ27541_SUBCMD_BD_OFFSET   0x0009
#define BQ27541_SUBCMD_INT_OFFSET  0x000a
#define BQ27541_SUBCMD_CC_VER   0x000b
#define BQ27541_SUBCMD_OCV  0x000c
#define BQ27541_SUBCMD_BAT_INS   0x000d
#define BQ27541_SUBCMD_BAT_REM   0x000e
#define BQ27541_SUBCMD_SET_HIB   0x0011
#define BQ27541_SUBCMD_CLR_HIB   0x0012
#define BQ27541_SUBCMD_SET_SLP   0x0013
#define BQ27541_SUBCMD_CLR_SLP   0x0014
#define BQ27541_SUBCMD_FCT_RES   0x0015
#define BQ27541_SUBCMD_ENABLE_DLOG  0x0018
#define BQ27541_SUBCMD_DISABLE_DLOG 0x0019
#define BQ27541_SUBCMD_SEALED   0x0020
#define BQ27541_SUBCMD_ENABLE_IT    0x0021
#define BQ27541_SUBCMD_DISABLE_IT   0x0023
#define BQ27541_SUBCMD_CAL_MODE  0x0040
#define BQ27541_SUBCMD_RESET   0x0041
#define ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN   (-2731)
#define BQ27541_INIT_DELAY   ((HZ)*1)

#define BQ27541_REG_DCAP_L			(0x3c)
#define BQ27541_REG_DCAP_H			(0x3d)
#define BQ27541_VALUE_DCAP_L			(0X07)
#define BQ27541_VALUE_DCAP_H			(0X0D)

#define ERROR_SOC  33
#define ERROR_BATT_VOL  (3800 * 1000)
/* If the system has several batteries we need a different name for each
 * of them...
 */
static DEFINE_IDR(battery_id);
static DEFINE_MUTEX(battery_mutex);

struct bq27541_device_info;
struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di);
};

struct bq27541_device_info {
	struct device			*dev;
	int				id;
	struct bq27541_access_methods	*bus;
	struct i2c_client		*client;
	struct work_struct		counter;
	/* 300ms delay is needed after bq27541 is powered up
	 * and before any successful I2C transaction
	 */
	struct  delayed_work		hw_config;
	struct  delayed_work		battery_soc_work;
	struct wake_lock update_soc_wake_lock;
	struct power_supply 	*batt_psy;
	int saltate_counter;
	/*  Add for retry when config fail */
	int retry_count;
	/*  Add for get right soc when sleep long time */
	int soc_pre;
	int  batt_vol_pre;
	int current_pre;
	int health_pre;
	unsigned long rtc_resume_time;
	unsigned long rtc_suspend_time;
	atomic_t suspended;
	int temp_pre;
	int lcd_off_delt_soc;
	int low_bat_capacity;
	int low_bat_voltage_uv;
	bool lcd_is_off;
	bool alow_reading;
	bool fastchg_started;
	bool support_4p4v_bat;
	bool is_4p4v_bat;
	unsigned long	lcd_off_time;
	unsigned long	soc_pre_time;
	unsigned long	soc_store_time;
};


#include <linux/workqueue.h>

struct update_pre_capacity_data{
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	int suspend_time;
};
static struct update_pre_capacity_data update_pre_capacity_data;

static int coulomb_counter;
static spinlock_t lock; /* protect access to coulomb_counter */

static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di);

static int bq27541_read(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret;
	int temp = 0;
	static int count = 0;

	/* Add for get right soc when sleep long time */
	if(atomic_read(&di->suspended) == 1) {
		return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	}

	if(di->alow_reading) {
		ret = bq27541_read(BQ27541_REG_TEMP, &temp, 0, di);
		/* Add for don't report battery not connect when reading error once. */
		if (ret) {
			count++;
			pr_err("error reading temperature\n");
			if(count > 1) {
				count = 0;
				/* Add for it report bad status when plug out battery */
				di->temp_pre = -400 - ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
				return -400;
			} else {
				return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
			}
		}
		count = 0;
	} else {
		return di->temp_pre + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
	}
	di->temp_pre = temp;
	return temp + ZERO_DEGREE_CELSIUS_IN_TENTH_KELVIN;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret;
	int volt = 0;

	/* Add for get right soc when sleep long time */
	if (atomic_read(&di->suspended) == 1) {
		return di->batt_vol_pre;
	}
	if (di->alow_reading) {
		ret = bq27541_read(BQ27541_REG_VOLT, &volt, 0, di);
		if (ret) {
			pr_err("error reading voltage,ret:%d\n", ret);
			return ret;
		}
	} else {
		return di->batt_vol_pre;
	}
	di->batt_vol_pre = volt * 1000;
	return volt * 1000;
}

static void bq27541_cntl_cmd(struct bq27541_device_info *di,
		int subcmd)
{
	bq27541_i2c_txsubcmd(BQ27541_REG_CNTL, subcmd, di);
}

/*
 * i2c specific code
 */
static int bq27541_i2c_txsubcmd(u8 reg, unsigned short subcmd,
		struct bq27541_device_info *di)
{
	struct i2c_msg msg;
	unsigned char data[3];
	int ret;

	if (!di->client)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = subcmd & 0x00FF;
	data[2] = (subcmd & 0xFF00) >> 8;

	msg.addr = di->client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	ret = i2c_transfer(di->client->adapter, &msg, 1);
	if (ret < 0)
		return -EIO;

	return 0;
}

static bool bq27541_regist_done = false ;
static bool check_4p4v_bat_present(struct bq27541_device_info *di)
{
	int flags = 0, ret = 0;
	ret = bq27541_read(BQ27541_REG_DCAP_H, &flags, 0, di);
	if(ret < 0)
	{
		pr_err("get bq27541 DCAP_H fail\n");
		bq27541_regist_done = true;
		return false;
	}
	if((flags & 0xff) != BQ27541_VALUE_DCAP_H){
		pr_err("err bq dcap_l value:0x%x\n",flags);
		return false;
	}
	ret = bq27541_read(BQ27541_REG_DCAP_L, &flags, 0, di);
	if(ret < 0)
	{
		pr_err("get bq27541 DCAP_L fail\n");
		bq27541_regist_done = true;
		return false;
	}
	if((flags & 0xff) != BQ27541_VALUE_DCAP_L){
		pr_err("err bq dcap_l value:0x%x\n",flags);
		return false;
	}
	return true;
}

static int bq27541_chip_config(struct bq27541_device_info *di)
{
	int flags = 0, ret = 0;

	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	ret = bq27541_read(BQ27541_REG_CNTL, &flags, 0, di);
	if (ret < 0) {
		pr_err("error reading register %02x ret = %d\n",
				BQ27541_REG_CNTL, ret);
		return ret;
	}
	udelay(66);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_IT);
	udelay(66);

	if (!(flags & BQ27541_CS_DLOGEN)) {
		bq27541_cntl_cmd(di, BQ27541_SUBCMD_ENABLE_DLOG);
		udelay(66);
	}

	return 0;
}

static void bq27541_coulomb_counter_work(struct work_struct *work)
{
	int value = 0, temp = 0, index = 0, ret = 0;
	struct bq27541_device_info *di;
	unsigned long flags;
	int count = 0;

	di = container_of(work, struct bq27541_device_info, counter);

	/* retrieve 30 values from FIFO of coulomb data logging buffer
	 * and average over time
	 */
	do {
		ret = bq27541_read(BQ27541_REG_LOGBUF, &temp, 0, di);
		if (ret < 0)
			break;
		if (temp != 0x7FFF) {
			++count;
			value += temp;
		}
		/* delay 66uS, waiting time between continuous reading
		 * results
		 */
		udelay(66);
		ret = bq27541_read(BQ27541_REG_LOGIDX, &index, 0, di);
		if (ret < 0)
			break;
		udelay(66);
	} while (index != 0 || temp != 0x7FFF);

	if (ret < 0) {
		pr_err("Error reading datalog register\n");
		return;
	}

	if (count) {
		spin_lock_irqsave(&lock, flags);
		coulomb_counter = value/count;
		spin_unlock_irqrestore(&lock, flags);
	}
}

struct bq27541_device_info *bq27541_di;
#define TEN_PERCENT                            10
#define SOC_SHUTDOWN_VALID_LIMITS              20
#define TEN_MINUTES                            600
#define FIVE_MINUTES                           300
#define TWO_POINT_FIVE_MINUTES                 150
#define ONE_MINUTE                             60
#define TWENTY_MINUTES                         1200
#define TWENTY_PERCENT                         20

#define CAPACITY_SALTATE_COUNTER_60            38 /* 40 1min */
#define CAPACITY_SALTATE_COUNTER_95            78 /* 60 2.5min */
#define CAPACITY_SALTATE_COUNTER_FULL          200 /* 150 120 5min */
#define CAPACITY_SALTATE_COUNTER_CHARGING_TERM 30 /* 30 1min */
#define CAPACITY_SALTATE_COUNTER               4
#define CAPACITY_SALTATE_COUNTER_NOT_CHARGING  24 /* >=40sec */
#define CAPACITY_CALIBRATE_TIME_60_PERCENT     45 /* 45s */

static int bq27541_average_current(struct bq27541_device_info *di);

extern int get_charging_status(void);
extern int fuelgauge_battery_temp_region_get(void);
extern int load_soc(void);
extern void backup_soc_ex(int soc);
extern bool get_oem_charge_done_status(void);
extern void clean_backup_soc_ex(void);

static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
				__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
				CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
				CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

int get_prop_pre_shutdown_soc(void)
{
	int soc_load;

	soc_load = load_soc();
	if (soc_load == -1)
		return 50;
	else
		return soc_load;
}

static int fg_soc_calibrate(struct  bq27541_device_info *di, int soc)
{
	union power_supply_propval ret = {0,};
	unsigned int soc_calib;
	unsigned long soc_current_time, time_last;
	static bool first_enter = false;
	static int charging_status, charging_status_pre = 0;
	bool chg_done;
	int temp_region, vbat_mv, ibat_ma, soc_load, soc_temp, counter_temp = 0;

	if (false == first_enter) {
		di->batt_psy = power_supply_get_by_name("battery");
		if(di->batt_psy) {
			first_enter = true;
			soc_load = load_soc();
			pr_info("soc=%d, soc_load=%d\n", soc, soc_load);
			if (soc_load == -1) {
				/* get last soc error */
				di->soc_pre = soc;
			} else if (soc_load > 0 && soc_load < 100) {
				if(soc_load > soc)
					di->soc_pre = soc_load - 1;
				else
					di->soc_pre = soc_load;
			} else if (soc_load == 100
					&& abs(soc_load - soc) > TEN_PERCENT) {
				/* decrease soc when gap between soc_load and */
				/* real_soc is over 10%                       */
				di->soc_pre = soc_load - 1;
			} else {
				di->soc_pre = soc_load;
			}

			if (!di->batt_psy) {
				pr_err("batt_psy is absent, soc_pre=%d\n", di->soc_pre);
				return di->soc_pre;
			}
			/* store the soc when boot first time */
			get_current_time(&di->soc_pre_time);
			clean_backup_soc_ex();
		} else {
			return soc;
		}
	}
	soc_temp = di->soc_pre;

	if (!di->batt_psy) {
		soc_calib = soc;
		goto out;
	}

	ret.intval = get_charging_status();
	di->batt_vol_pre = bq27541_battery_voltage(di);
	chg_done = get_oem_charge_done_status();
	temp_region = fuelgauge_battery_temp_region_get();
	if ((temp_region == BATT_TEMP_LITTLE_COOL
			|| temp_region == BATT_TEMP_COOL
			|| temp_region == BATT_TEMP_NORMAL
			|| temp_region == BATT_TEMP_PRE_NORMAL)
			&& chg_done) {
		ret.intval = POWER_SUPPLY_STATUS_FULL;
	}

	if (ret.intval == POWER_SUPPLY_STATUS_CHARGING
			|| ret.intval == POWER_SUPPLY_STATUS_FULL
			|| bq27541_di->fastchg_started)
		charging_status = 1;
	else
		charging_status = 0;

	if (charging_status ^ charging_status_pre) {
		if (charging_status_pre) {
			get_current_time(&soc_current_time);
			di->soc_store_time = soc_current_time - di->soc_pre_time;
		}

		get_current_time(&di->soc_pre_time);
		if (!charging_status_pre && di->soc_store_time)
			di->soc_pre_time -= di->soc_store_time;
		charging_status_pre = charging_status;
		di->saltate_counter = 0;
	}

	get_current_time(&soc_current_time);
	time_last = soc_current_time - di->soc_pre_time;
	if (charging_status) { /* is charging */
		if (ret.intval == POWER_SUPPLY_STATUS_FULL) {
			soc_calib = di->soc_pre;
			if (di->soc_pre < 100
					&& (temp_region == BATT_TEMP_LITTLE_COOL
					|| temp_region == BATT_TEMP_NORMAL
					|| temp_region == BATT_TEMP_PRE_NORMAL
					|| temp_region == BATT_TEMP_COOL)) {
				if (di->saltate_counter < CAPACITY_SALTATE_COUNTER_CHARGING_TERM) {
					di->saltate_counter++;
				} else {
					soc_calib = di->soc_pre + 1;
					di->saltate_counter = 0;
				}
			}
		} else {
			if (soc - di->soc_pre > 0) {
				di->saltate_counter++;
				if ((bq27541_di->fastchg_started && time_last < 20)
						|| (!bq27541_di->fastchg_started && time_last < 30))
					return di->soc_pre;
				else
					di->saltate_counter = 0;
				soc_calib = di->soc_pre + 1;
			} else if (soc < (di->soc_pre - 1)) {
				di->saltate_counter++;
				if (di->soc_pre == 100) {
					counter_temp = CAPACITY_SALTATE_COUNTER_FULL; /* t>=5min */
				} else if (di->soc_pre > 95) {
					counter_temp = CAPACITY_SALTATE_COUNTER_95; /* t>=2.5min */
				} else if (di->soc_pre > 60) {
					counter_temp = CAPACITY_SALTATE_COUNTER_60; /* t>=1min */
				} else {
					if (time_last > CAPACITY_CALIBRATE_TIME_60_PERCENT
							&& (soc - di->soc_pre) < 0)
						counter_temp = 0;
					else
						/* t>=40sec */
						counter_temp = CAPACITY_SALTATE_COUNTER_NOT_CHARGING;
				}

				/* avoid dead battery shutdown */
				if (di->batt_vol_pre <= di->low_bat_voltage_uv
						&& di->batt_vol_pre > 2500 * 1000
						&& di->soc_pre <= di->low_bat_capacity) {
					/* check again */
					vbat_mv = bq27541_battery_voltage(di);
					if (vbat_mv <= di->low_bat_voltage_uv
							&& vbat_mv > 2500 * 1000) {
						/* about 9s */
						counter_temp = CAPACITY_SALTATE_COUNTER - 1;
					}
				}

				ibat_ma = bq27541_average_current(di);
				/* don't allow soc down if chg current > -200mA */
				if (di->saltate_counter < counter_temp
						|| ibat_ma < -200 * 1000)
					return di->soc_pre;
				else
					di->saltate_counter = 0;

				soc_calib = di->soc_pre - 1;
			} else if ((soc == 0 && soc < di->soc_pre)
					&& di->soc_pre <= 2) {
				di->saltate_counter++;
				if (time_last > CAPACITY_CALIBRATE_TIME_60_PERCENT
						&& (soc - di->soc_pre) < 0)
					counter_temp = 0;
				else
					/* t>=40sec */
					counter_temp = CAPACITY_SALTATE_COUNTER_NOT_CHARGING;

				if (di->saltate_counter < counter_temp)
					return di->soc_pre;
				else
					di->saltate_counter = 0;
				soc_calib = di->soc_pre - 1;
			} else {
				soc_calib = di->soc_pre;
			}
		}
	} else { /* not charging */
		if ((soc < di->soc_pre)
				|| (di->batt_vol_pre <= di->low_bat_voltage_uv
				&& di->batt_vol_pre > 2500 * 1000)) {
			if(di->soc_pre == 100){
				counter_temp = FIVE_MINUTES;
			}
			else if (di->soc_pre > 95){
				counter_temp = TWO_POINT_FIVE_MINUTES;
			}
			else if (di->soc_pre > 60){
				counter_temp = ONE_MINUTE;
			}
			else {
				counter_temp = CAPACITY_CALIBRATE_TIME_60_PERCENT;
			}

			/* avoid dead battery shutdown */
			if (di->batt_vol_pre <= di->low_bat_voltage_uv
					&& di->batt_vol_pre > 2500 * 1000
					&& di->soc_pre <= di->low_bat_capacity) {
				/* check again */
				vbat_mv = bq27541_battery_voltage(di);
				if (vbat_mv <= di->low_bat_voltage_uv
						&& vbat_mv > 2500 * 1000 && time_last > 9)
					counter_temp = 0;
			}

			if (time_last < counter_temp)
				return di->soc_pre;
		}

		if (soc < di->soc_pre)
			soc_calib = di->soc_pre - 1;
		else if (di->batt_vol_pre <= di->low_bat_voltage_uv
				&& di->batt_vol_pre > 2500 * 1000
				&& di->soc_pre > 0 && time_last > 9)
			soc_calib = di->soc_pre - 1;
		else
			soc_calib = di->soc_pre;
	}

out:
	if(soc_calib > 100)
		soc_calib = 100;
	if(soc_calib < 0)
		soc_calib = 0;
	if (soc_calib == 0 && !charging_status
		&& (bq27541_battery_voltage(di) / 1000) > 3400)
		soc_calib = 1;

	di->soc_pre = soc_calib;

	if(soc_temp != soc_calib) {
		get_current_time(&di->soc_pre_time);
		/* store when soc changed */
		power_supply_changed(di->batt_psy);
		pr_info("soc:%d, soc_calib:%d, VOLT:%d, current:%d\n",
				soc, soc_calib, bq27541_battery_voltage(di) / 1000,
				bq27541_average_current(di) / 1000);
	}

	return soc_calib;
}


static int bq27541_battery_soc(struct bq27541_device_info *di, int suspend_time_ms)
{
	int ret;
	int soc = 0;
	int soc_delt = 0;
	static int soc_pre;
	bool fg_soc_changed=false;
	/* Add for get right soc when sleep long time */
	if(atomic_read(&di->suspended) == 1) {
		dev_warn(di->dev, "di->suspended di->soc_pre=%d\n", di->soc_pre);
		return di->soc_pre;
	}
	if(di->alow_reading) {
		ret = bq27541_read(BQ27541_REG_SOC, &soc, 0, di);
		if (ret) {
			pr_err("error reading soc=%d, ret:%d\n", soc, ret);
			goto read_soc_err;
		}
		if(soc_pre != soc)
			pr_err("bq27541_battery_soc = %d\n", soc);

		soc_pre = soc;
	} else {
		if(di->soc_pre)
			return di->soc_pre;
		else
			return 0;
	}
	/* Add for get right soc when sleep long time */
	if (suspend_time_ms && di->lcd_is_off) {
		if (soc < di->soc_pre) {
			soc_delt  =  di->soc_pre - soc;
			fg_soc_changed = (soc < TWENTY_PERCENT
					|| soc_delt > di->lcd_off_delt_soc
					|| suspend_time_ms > TEN_MINUTES);
			pr_info("suspend_time_ms=%d,soc_delt=%d,di->lcd_off_delt_soc=%d\n",
					suspend_time_ms, soc_delt, di->lcd_off_delt_soc);
			if (fg_soc_changed) {
				if (suspend_time_ms/TEN_MINUTES) {
					di->soc_pre -= (suspend_time_ms / TEN_MINUTES < soc_delt
							? suspend_time_ms / TEN_MINUTES : soc_delt);
				} else {
					di->soc_pre -= 1;
				}
				/* store when soc changed */
				get_current_time(&di->soc_pre_time);
				power_supply_changed(di->batt_psy);
				pr_err("system resume,soc:%d, soc_calib:%d,VOLT:%d,current:%d\n",
					soc, di->soc_pre, bq27541_battery_voltage(di) / 1000,
					bq27541_average_current(di) / 1000);
			}
		}
		goto read_soc_err;
	}
	soc = fg_soc_calibrate(di,soc);
	return soc;

read_soc_err:
	if(di->soc_pre) {
		dev_warn(di->dev, "read_soc_exit ,di->soc_pre=%d\n", di->soc_pre);
		return di->soc_pre;
	}
	else
		return 0;
}

static int bq27541_average_current(struct bq27541_device_info *di)
{
	int ret;
	int curr = 0;

	/* Add for get right soc when sleep long time */
	if(atomic_read(&di->suspended) == 1)
		return -di->current_pre;
	if (di->alow_reading) {
		ret = bq27541_read(BQ27541_REG_AI, &curr, 0, di);
		if (ret) {
			pr_err("error reading current.\n");
			return ret;
		}
	} else {
		return -di->current_pre;
	}
	/* negative current */
	if (curr & 0x8000)
		curr = -((~(curr-1)) & 0xFFFF);
	di->current_pre = 1000 * curr;
	return -curr * 1000;
}

static int bq27541_remaining_capacity(struct bq27541_device_info *di)
{
	int ret;
	int cap = 0;

	if(di->alow_reading) {
		ret = bq27541_read(BQ27541_REG_RM, &cap, 0, di);
		if (ret) {
			pr_err("error reading capacity.\n");
			return ret;
		}
	}

	return cap;
}

static int bq27541_batt_health(struct bq27541_device_info *di)
{
	int ret;
	int health = 0;

	if(di->alow_reading) {
		ret = bq27541_read(BQ27541_REG_NIC, &health, 0, di);
		if (ret) {
			pr_err("error reading health\n");
			return ret;
		}
		di->health_pre = health;
	}

	return di->health_pre;
}

static int bq27541_get_battery_mvolts(void)
{
	return bq27541_battery_voltage(bq27541_di);
}

static int bq27541_get_batt_remaining_capacity(void)
{
	return bq27541_remaining_capacity(bq27541_di);
}

static int bq27541_get_batt_health(void)
{
	return bq27541_batt_health(bq27541_di);
}

static int bq27541_get_battery_temperature(void)
{
	return bq27541_battery_temperature(bq27541_di);
}
static bool bq27541_is_battery_present(void)
{
	return true;
}

static bool bq27541_is_battery_temp_within_range(void)
{
	return true;
}

static bool bq27541_is_battery_id_valid(void)
{
	return true;
}

static int bq27541_get_battery_soc(void)
{
	return bq27541_battery_soc(bq27541_di, 0);
}

static int bq27541_get_average_current(void)
{
	return bq27541_average_current(bq27541_di);
}

static int bq27541_set_alow_reading(int enable)
{
	if(bq27541_di)
		bq27541_di->alow_reading = enable;

	return 0;
}

static int bq27541_set_lcd_off_status(int off)
{
	int soc,ret;

	pr_info("off=%d\n", off);
	if (bq27541_di) {
		if (off) {
			ret = bq27541_read(BQ27541_REG_SOC, &soc, 0, bq27541_di);
			if (ret) {
				bq27541_di->lcd_off_delt_soc=0;
				pr_err("soc error reading ret=%d,soc%d\n", ret, soc);
			} else {
				bq27541_di->lcd_off_delt_soc = bq27541_di->soc_pre-soc;
				pr_info("lcd_off_delt_soc:%d,soc=%d,soc_pre=%d\n",
						bq27541_di->lcd_off_delt_soc, soc,
						bq27541_di->soc_pre);
			}
			get_current_time(&bq27541_di->lcd_off_time);
			bq27541_di->lcd_is_off=true;
		} else {
			bq27541_di->lcd_is_off=false;
			bq27541_di->lcd_off_delt_soc=0;
		}
	}
	return 0;
}

static int bq27541_get_fastchg_started_status(bool fastchg_started_status)
{
	if(bq27541_di)
		bq27541_di->fastchg_started = fastchg_started_status;

	return 0;
}
static bool bq27541_get_4p4v_battery_present(void)
{
	if(bq27541_di)
		return bq27541_di->is_4p4v_bat;
	return false;
}


static struct external_battery_gauge bq27541_batt_gauge = {
	.get_battery_mvolts     = bq27541_get_battery_mvolts,
	.get_battery_temperature    = bq27541_get_battery_temperature,
	.is_battery_present     = bq27541_is_battery_present,
	.is_battery_temp_within_range   = bq27541_is_battery_temp_within_range,
	.is_battery_id_valid        = bq27541_is_battery_id_valid,
	.get_batt_remaining_capacity        =bq27541_get_batt_remaining_capacity,
	.get_batt_health        = bq27541_get_batt_health,
	.get_battery_soc            = bq27541_get_battery_soc,
	.get_average_current        = bq27541_get_average_current,
	.set_alow_reading		= bq27541_set_alow_reading,
	.set_lcd_off_status		= bq27541_set_lcd_off_status,
	.fast_chg_started_status	= bq27541_get_fastchg_started_status,
	.get_4p4v_battery_present	= bq27541_get_4p4v_battery_present,
};
#define BATTERY_SOC_UPDATE_MS 6000
#define RESUME_SCHDULE_SOC_UPDATE_WORK_MS 60000

static int is_usb_pluged(void)
{
	static struct power_supply *psy;
	union power_supply_propval ret = {0,};
	int Vbus;
	if (!psy) {
		psy = power_supply_get_by_name("battery");
		if (!psy) {
			pr_err("failed to get ps battery\n");
			return -EINVAL;
		}
	}

	if (psy->get_property(psy, POWER_SUPPLY_PROP_CHARGE_NOW, &ret))
		return -EINVAL;

	if (ret.intval <= 0)
		return -EINVAL;

	Vbus = ret.intval;
	return (Vbus > 2500) ? true : false;
}

static bool get_dash_started(void)
{
	if(bq27541_di && bq27541_di->fastchg_started)
		return bq27541_di->fastchg_started ;
	else
		return false;
}

static void update_battery_soc_work(struct work_struct *work)
{
	if(is_usb_pluged()== true) {
		schedule_delayed_work(
				&bq27541_di->battery_soc_work,
				msecs_to_jiffies(BATTERY_SOC_UPDATE_MS));
		if(get_dash_started()==true)
			return;
		if(!bq27541_di->alow_reading)
			bq27541_set_alow_reading(true);
		return;
	}
	bq27541_set_alow_reading(true);
	bq27541_get_battery_mvolts();
	bq27541_get_average_current();
	bq27541_get_battery_temperature();
	bq27541_get_battery_soc();
	bq27541_get_batt_remaining_capacity();
	bq27541_set_alow_reading(false);
	schedule_delayed_work(&bq27541_di->battery_soc_work,
			msecs_to_jiffies(BATTERY_SOC_UPDATE_MS));
}

bool get_extern_fg_regist_done(void)
{
	return bq27541_regist_done;
}
static int set_property_on_fg(enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };
	struct power_supply *bms_psy;

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		pr_err("bms_psy not exist\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = bms_psy->set_property(bms_psy, prop, &ret);
	if (rc)
		pr_err("bms psy does not allow updating prop %d rc = %d\n",prop, rc);

	return rc;
}

static void check_bat_type(struct bq27541_device_info * di)
{
		di->is_4p4v_bat = check_4p4v_bat_present(di);
		set_property_on_fg(POWER_SUPPLY_PROP_BATTERY_4P4V_PRESENT,di->is_4p4v_bat);
		pr_info("is_4p4v_bat=%d,support_4p4v_bat=%d\n",di->is_4p4v_bat,di->support_4p4v_bat);
}

static int is_battery_type_right(struct bq27541_device_info * di)
{
		if(di->support_4p4v_bat)
		{
			if(di->is_4p4v_bat)
				return true;
		}
		else
		{
			if(!di->is_4p4v_bat)
				return true;
		}
		return false;
}
static void bq27541_hw_config(struct work_struct *work)
{
	int ret = 0, flags = 0, type = 0, fw_ver = 0;
	struct bq27541_device_info *di;

	di = container_of(work, struct bq27541_device_info,
			hw_config.work);
	ret = bq27541_chip_config(di);
	if (ret) {
		pr_err("Failed to config Bq27541\n");
		/* Add for retry when config fail */
		di->retry_count--;
		if(di->retry_count > 0)
			schedule_delayed_work(&di->hw_config, HZ);
		else
			bq27541_regist_done = true;

		return;
	}

	ret = is_battery_type_right(di);
	if(!ret)
		return;
	external_battery_gauge_register(&bq27541_batt_gauge);
	bq27541_information_register(&bq27541_batt_gauge);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_CTNL_STATUS);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &flags, 0, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DEVCIE_TYPE);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &type, 0, di);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_FW_VER);
	udelay(66);
	bq27541_read(BQ27541_REG_CNTL, &fw_ver, 0, di);
	bq27541_regist_done = true;
	pr_info("DEVICE_TYPE is 0x%02X, FIRMWARE_VERSION is 0x%02X\n",
			type, fw_ver);
	pr_info("Complete bq27541 configuration 0x%02X\n", flags);
}

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[2];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;
	mutex_lock(&battery_mutex);

	/* Write register */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;
	data[0] = reg;
	/* Read data */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	if (!b_single)
		msg[1].len = 2;
	else
		msg[1].len = 1;
	msg[1].buf = data;
	err = i2c_transfer(client->adapter, msg, 2);
	if (err >= 0) {
		if (!b_single)
			*rt_value = get_unaligned_le16(data);
		else
			*rt_value = data[0];
		mutex_unlock(&battery_mutex);
		return 0;
	}
	mutex_unlock(&battery_mutex);
	return err;
}

#ifdef CONFIG_BQ27541_TEST_ENABLE
static int reg;
static int subcmd;
static ssize_t bq27541_read_stdcmd(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (reg <= BQ27541_REG_ICR && reg > 0x00) {
		ret = bq27541_read(reg, &temp, 0, di);
		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_stdcmd(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	reg = cmd;
	return ret;
}

static ssize_t bq27541_read_subcmd(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int temp = 0;
	struct platform_device *client;
	struct bq27541_device_info *di;

	client = to_platform_device(dev);
	di = platform_get_drvdata(client);

	if (subcmd == BQ27541_SUBCMD_DEVCIE_TYPE ||
			subcmd == BQ27541_SUBCMD_FW_VER ||
			subcmd == BQ27541_SUBCMD_HW_VER ||
			subcmd == BQ27541_SUBCMD_CHEM_ID) {

		bq27541_cntl_cmd(di, subcmd); /* Retrieve Chip status */
		udelay(66);
		ret = bq27541_read(BQ27541_REG_CNTL, &temp, 0, di);

		if (ret)
			ret = snprintf(buf, PAGE_SIZE, "Read Error!\n");
		else
			ret = snprintf(buf, PAGE_SIZE, "0x%02x\n", temp);
	} else
		ret = snprintf(buf, PAGE_SIZE, "Register Error!\n");

	return ret;
}

static ssize_t bq27541_write_subcmd(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int cmd;

	sscanf(buf, "%x", &cmd);
	subcmd = cmd;
	return ret;
}

static DEVICE_ATTR(std_cmd, S_IRUGO|S_IWUGO, bq27541_read_stdcmd,
		bq27541_write_stdcmd);
static DEVICE_ATTR(sub_cmd, S_IRUGO|S_IWUGO, bq27541_read_subcmd,
		bq27541_write_subcmd);
static struct attribute *fs_attrs[] = {
	&dev_attr_std_cmd.attr,
	&dev_attr_sub_cmd.attr,
	NULL,
};
static struct attribute_group fs_attr_group = {
	.attrs = fs_attrs,
};

static struct platform_device this_device = {
	.name			= "bq27541-test",
	.id			= -1,
	.dev.platform_data	= NULL,
};
#endif

static void update_pre_capacity_func(struct work_struct *w)
{
	pr_info("enter\n");
	bq27541_set_alow_reading(true);
	bq27541_battery_soc(bq27541_di, update_pre_capacity_data.suspend_time);
	bq27541_set_alow_reading(false);
	wake_unlock(&bq27541_di->update_soc_wake_lock);
	pr_info("exit\n");
}

#define MAX_RETRY_COUNT	5
#define DEFAULT_INVALID_SOC_PRE  -22
static int bq27541_parse_dt(struct bq27541_device_info *di)
{
	struct device_node *dev_node = di->client->dev.of_node;
	if (!dev_node) {
		pr_err("device tree info. missing\n");
		return -EINVAL;
	}
	if(of_property_read_bool(dev_node, "oem,support-4p4v-battery"))
		di->support_4p4v_bat = true;

	of_property_read_u32(di->client->dev.of_node,
				"op,low-bat-voltage-uv",
				&di->low_bat_voltage_uv);
	of_property_read_u32(di->client->dev.of_node,
			"op,low-bat-capacity",
			&di->low_bat_capacity);
	return 0;
}

static int bq27541_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	char *name;
	struct bq27541_device_info *di;
	struct bq27541_access_methods *bus;
	int num;
	int retval = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;
	update_pre_capacity_data.workqueue = create_workqueue("update_pre_capacity");
	INIT_DELAYED_WORK(&(update_pre_capacity_data.work), update_pre_capacity_func);

	mutex_lock(&battery_mutex);
	num = idr_alloc(&battery_id, client, 0, 0, GFP_KERNEL);
	mutex_unlock(&battery_mutex);
	if (retval < 0)
		return retval;

	name = kasprintf(GFP_KERNEL, "%s-%d", id->name, num);
	if (!name) {
		pr_err("failed to allocate device name\n");
		retval = -ENOMEM;
		goto batt_failed_1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		pr_err("failed to allocate device info data\n");
		retval = -ENOMEM;
		goto batt_failed_2;
	}
	di->id = num;

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		pr_err("failed to allocate access method data\n");
		retval = -ENOMEM;
		goto batt_failed_3;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27541_read_i2c;
	di->bus = bus;
	di->client = client;
	bq27541_parse_dt(di);

	wake_lock_init(&di->update_soc_wake_lock,
			WAKE_LOCK_SUSPEND, "bq_delt_soc_wake_lock");
	di->soc_pre = DEFAULT_INVALID_SOC_PRE;
	di->temp_pre = 0;
	di->alow_reading = true;
	/* Add for retry when config fail */
	di->retry_count = MAX_RETRY_COUNT;
	atomic_set(&di->suspended, 0);


#ifdef CONFIG_BQ27541_TEST_ENABLE
	platform_set_drvdata(&this_device, di);
	retval = platform_device_register(&this_device);
	if (!retval) {
		retval = sysfs_create_group(&this_device.dev.kobj,
				&fs_attr_group);
		if (retval)
			goto batt_failed_4;
	} else
		goto batt_failed_4;
#endif

	if (retval) {
		pr_err("failed to setup bq27541\n");
		goto batt_failed_4;
	}

	if (retval) {
		pr_err("failed to powerup bq27541\n");
		goto batt_failed_4;
	}

	spin_lock_init(&lock);

	bq27541_di = di;
	di->lcd_is_off=false;
	INIT_WORK(&di->counter, bq27541_coulomb_counter_work);
	INIT_DELAYED_WORK(&di->hw_config, bq27541_hw_config);
	INIT_DELAYED_WORK(&di->battery_soc_work, update_battery_soc_work);
	schedule_delayed_work(&di->hw_config, BQ27541_INIT_DELAY);
	schedule_delayed_work(&di->battery_soc_work, BATTERY_SOC_UPDATE_MS);
	pr_info("probe sucdess\n");
	check_bat_type(di);
	return 0;

batt_failed_4:
	kfree(bus);
batt_failed_3:
	kfree(di);
batt_failed_2:
	kfree(name);
batt_failed_1:
	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, num);
	mutex_unlock(&battery_mutex);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	external_battery_gauge_unregister(&bq27541_batt_gauge);
	bq27541_information_unregister(&bq27541_batt_gauge);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_DLOG);
	udelay(66);
	bq27541_cntl_cmd(di, BQ27541_SUBCMD_DISABLE_IT);
	cancel_delayed_work_sync(&di->hw_config);

	kfree(di->bus);

	mutex_lock(&battery_mutex);
	idr_remove(&battery_id, di->id);
	mutex_unlock(&battery_mutex);

	kfree(di);
	return 0;
}

static int bq27541_battery_suspend(struct i2c_client *client, pm_message_t message)
{
	int ret=0;
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	cancel_delayed_work_sync(&di->battery_soc_work);
	atomic_set(&di->suspended, 1);
	ret = get_current_time(&di->rtc_suspend_time);
	if (ret ) {
		pr_err("Failed to read RTC time\n");
		return 0;
	}
	return 0;
}

/*1 minute*/
#define RESUME_TIME  1*60
static int bq27541_battery_resume(struct i2c_client *client)
{
	int ret=0;
	int suspend_time;
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	atomic_set(&di->suspended, 0);
	ret = get_current_time(&di->rtc_resume_time);
	if (ret ) {
		pr_err("Failed to read RTC time\n");
		return 0;
	}
	suspend_time =  di->rtc_resume_time - di->rtc_suspend_time;
	pr_info("suspend_time=%d\n", suspend_time);
	update_pre_capacity_data.suspend_time = suspend_time;

	for (ret = 0; ret < NR_CPUS; ret++) {
		if (cpu_online(ret) && (ret != raw_smp_processor_id()))
			break;
	}

	if (di->rtc_resume_time - di->lcd_off_time >= TWO_POINT_FIVE_MINUTES) {
		pr_err("di->rtc_resume_time - di->lcd_off_time=%ld\n",
				di->rtc_resume_time - di->lcd_off_time);
		wake_lock(&di->update_soc_wake_lock);
		get_current_time(&di->lcd_off_time);
		queue_delayed_work_on(ret != NR_CPUS ? ret : 0,
				update_pre_capacity_data.workqueue,
				&(update_pre_capacity_data.work), msecs_to_jiffies(1000));
	}
	schedule_delayed_work(&bq27541_di->battery_soc_work,
			msecs_to_jiffies(RESUME_SCHDULE_SOC_UPDATE_WORK_MS));
	return 0;
}
static void bq27541_shutdown(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	if(di->soc_pre != DEFAULT_INVALID_SOC_PRE)
		backup_soc_ex(di->soc_pre);
}

static const struct of_device_id bq27541_match[] = {
	{ .compatible = "ti,bq27541-battery" },
	{ },
};

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27541_id);

static struct i2c_driver bq27541_battery_driver = {
	.driver		= {
		.name = "bq27541-battery",
	},
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
	.suspend	= bq27541_battery_suspend ,
	.resume		= bq27541_battery_resume,
	.shutdown	= bq27541_shutdown,
	.id_table	= bq27541_id,
};

static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register BQ27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("BQ27541 battery monitor driver");
