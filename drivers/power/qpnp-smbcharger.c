/* Copyright (c) 2014-2016 The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "SMBCHG: %s: " fmt, __func__

#include <linux/spmi.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/bitops.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/spmi.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/leds.h>
#include <linux/rtc.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/batterydata-lib.h>
#include <linux/of_batterydata.h>
#include <linux/msm_bcl.h>
#include <linux/ktime.h>
#include "pmic-voter.h"
#include "oem_external_fg.h"
#include <linux/type-c_notifier.h>
#include <linux/wakelock.h>

#include <linux/proc_fs.h>

#if defined(CONFIG_FB)
/* yangfangbiao@oneplus.cn,20150519  Add for reset charge current when screen is off */
#include <linux/notifier.h>
#include <linux/fb.h>
#endif/*CONFIG_FB*/
/* Mask/Bit helpers */
#define _SMB_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
	_SMB_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))
#define SOC_INVALID			0x7E
#define SOC_DATA_REG_0		0x88D

static bool use_fake_chgvol = false;
static int fake_chgvol = 0;
static bool use_fake_temp = false;
static int fake_temp = 300;
static bool use_fake_authentic = false;
static int fake_authentic = 0;
static int charger_type_value;

/*Modify for  charge standard */
#define BATT_HEARTBEAT_INTERVAL                              6000//6S
#define AUTO_CHARGING_BATT_TEMP_T0                           -30
#define AUTO_CHARGING_BATT_TEMP_T1                            0
#define AUTO_CHARGING_BATT_TEMP_T2                            50
#define AUTO_CHARGING_BATT_TEMP_T3                            120
#define AUTO_CHARGING_BATT_TEMP_T4                            160
#define AUTO_CHARGING_BATT_TEMP_T5                            450
#define AUTO_CHARGING_BATT_TEMP_T6                            530
#define AUTO_CHARGING_BATT_REMOVE_TEMP                        -400
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_HOT_TO_WARM      20
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_WARM_TO_NORMAL   20
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL   20
#define AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COLD_TO_COOL     20
#define CHG_CURRENT_500MA                                     500
#define CHG_CURRENT_300MA                                     300
#define CHG_CURRENT_900MA                                     900
#define CHG_CURRENT_1500MA                                    1500
#define CHARGER_SOFT_OVP_VOLTAGE		5800
#define CHARGER_SOFT_UVP_VOLTAGE		4300
#define CHARGER_VOLTAGE_NORMAL		    5000
#define BATTERY_SOFT_OVP_VOLTAGE		4500 * 1000
#define BATT_CHG_TIMEOUT_COUNT_DCP				10*10*60// //6*10*60//4	//10HOURS
#define BATT_CHG_TIMEOUT_COUNT_USB_PRO			10*10*60 //10HOURS
#define DWC3_HVDCP_CHG_MAX                             1800


typedef enum{
	/* The charger is good		*/
	CHARGER_STATUS_GOOD,
	/* The charger is bad		*/
	CHARGER_STATUS_BAD,
	/* The charger is weak		*/
	CHARGER_STATUS_WEAK,
	CHARGER_STATUS_OVER,
	/* Invalid charger status.	*/
	CHARGER_STATUS_INVALID
}chg_charger_status_type;
enum chg_battery_status_type {
	/* The battery is good		  */
	BATTERY_STATUS_GOOD,
	/* The battery is cold/hot	  */
	BATTERY_STATUS_BAD_TEMP,
	/* The battery is bad		  */
	BATTERY_STATUS_BAD,
	/* The battery is removed	  */
	BATTERY_STATUS_REMOVED, 	/* on v2.2 only */
	BATTERY_STATUS_INVALID_v1 = BATTERY_STATUS_REMOVED,
	/* Invalid battery status.	  */
	BATTERY_STATUS_INVALID
};

typedef enum{
	/*! Battery is cold 			  */
	CV_BATTERY_TEMP_REGION__COLD,
	/*! Battery is little cold				 */
	CV_BATTERY_TEMP_REGION_LITTLE__COLD,
	/*! Battery is cool 			  */
	CV_BATTERY_TEMP_REGION__COOL,
	/*! Battery is little cool 			  */
	CV_BATTERY_TEMP_REGION__LITTLE_COOL,
	/*! Battery is pre normal			  */
	CV_BATTERY_TEMP_REGION__PRE_NORMAL,
	/*! Battery is normal			  */
	CV_BATTERY_TEMP_REGION__NORMAL,
	/*! Battery is warm 			  */
	CV_BATTERY_TEMP_REGION__WARM,
	/*! Battery is hot				  */
	CV_BATTERY_TEMP_REGION__HOT,
	/*! Invalid battery temp region   */
	CV_BATTERY_TEMP_REGION__INVALID,
}chg_cv_battery_temp_region_type;

#define NON_STANDARD_CHARGER_CHECK_MS 30000

/* Config registers */
struct smbchg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct parallel_usb_cfg {
	struct power_supply		*psy;
	int				min_current_thr_ma;
	int				min_9v_current_thr_ma;
	int				allowed_lowering_ma;
	int				current_max_ma;
	bool				avail;
	struct mutex			lock;
	int				initial_aicl_ma;
	ktime_t				last_disabled;
	bool				enabled_once;
};

struct ilim_entry {
	int vmin_uv;
	int vmax_uv;
	int icl_pt_ma;
	int icl_lv_ma;
	int icl_hv_ma;
};

struct ilim_map {
	int			num;
	struct ilim_entry	*entries;
};

struct smbchg_version_tables {
	const int			*dc_ilim_ma_table;
	int				dc_ilim_ma_len;
	const int			*usb_ilim_ma_table;
	int				usb_ilim_ma_len;
	const int			*iterm_ma_table;
	int				iterm_ma_len;
	const int			*fcc_comp_table;
	int				fcc_comp_len;
	const int			*aicl_rerun_period_table;
	int				aicl_rerun_period_len;
	int				rchg_thr_mv;
};

struct smbchg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	int				schg_version;

	/* peripheral register address bases */
	u16				chgr_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				otg_base;
	u16				misc_base;

	int				fake_battery_soc;
	u8				revision[4];

	/* configuration parameters */
	int				iterm_ma;
	int				usb_max_current_ma;
	int				typec_current_ma;
	int				dc_max_current_ma;
	int				dc_target_current_ma;
	int				cfg_fastchg_current_ma;
	int				fastchg_current_ma;
	int				vfloat_mv;
	int				fastchg_current_comp;
	int				float_voltage_comp;
	int				resume_delta_mv;
	int				safety_time;
	int				prechg_safety_time;
	int				bmd_pin_src;
	int				jeita_temp_hard_limit;
	int				aicl_rerun_period_s;
	bool				use_vfloat_adjustments;
	bool				iterm_disabled;
	bool				bmd_algo_disabled;
	bool				soft_vfloat_comp_disabled;
	bool				chg_enabled;
	bool				charge_unknown_battery;
	bool				chg_inhibit_en;
	bool				chg_inhibit_source_fg;
	bool				low_volt_dcin;
	bool				cfg_chg_led_support;
	bool				cfg_chg_led_sw_ctrl;
	bool				vbat_above_headroom;
	bool				force_aicl_rerun;
	bool				hvdcp3_supported;
	bool				restricted_charging;
	bool				skip_usb_suspend_for_fake_battery;
	bool				hvdcp_not_supported;
	bool				otg_pinctrl;
	u8				original_usbin_allowance;
	struct parallel_usb_cfg		parallel;
	struct delayed_work		parallel_en_work;
	struct dentry			*debug_root;
	struct smbchg_version_tables	tables;

	/* wipower params */
	struct ilim_map			wipower_default;
	struct ilim_map			wipower_pt;
	struct ilim_map			wipower_div2;
	struct qpnp_vadc_chip		*vadc_dev;
	bool				wipower_dyn_icl_avail;
	struct ilim_entry		current_ilim;
	struct mutex			wipower_config;
	bool				wipower_configured;
	struct qpnp_adc_tm_btm_param	param;

	/* flash current prediction */
	int				rpara_uohm;
	int				rslow_uohm;
	int				vled_max_uv;

	/* vfloat adjustment */
	int				max_vbat_sample;
	int				n_vbat_samples;

	/* status variables */
	int				wake_reasons;
	int				previous_soc;
	int				usb_online;
	bool				dc_present;
	bool				usb_present;
	bool				batt_present;
	bool				hvdcp_present;
	bool				dash_present;
	bool				usb_enum_status;
	bool				nonstandard_charger;
	bool				redect_charger_type_done;
	int				otg_retries;
	ktime_t				otg_enable_time;
	bool				aicl_deglitch_short;
	bool				safety_timer_en;
	bool				aicl_complete;
	bool				usb_ov_det;
	bool				otg_pulse_skip_dis;
	const char			*battery_type;
	enum power_supply_type		usb_supply_type;
	bool				very_weak_charger;
	bool				parallel_charger_detected;
	bool				chg_otg_enabled;
	bool				flash_triggered;
	bool				flash_active;
	bool				icl_disabled;
	u32				wa_flags;
	int				usb_icl_delta;
	bool				typec_dfp;

	/* jeita and temperature */
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;

	/* irqs */
	int				batt_hot_irq;
	int				batt_warm_irq;
	int				batt_cool_irq;
	int				batt_cold_irq;
	int				batt_missing_irq;
	int				vbat_low_irq;
	int				chg_hot_irq;
	int				chg_term_irq;
	int				taper_irq;
	bool				taper_irq_enabled;
	struct mutex			taper_irq_lock;
	int				recharge_irq;
	int				fastchg_irq;
	int				wdog_timeout_irq;
	int				power_ok_irq;
	int				dcin_uv_irq;
	int				usbin_uv_irq;
	int				usbin_ov_irq;
	int				src_detect_irq;
	int				otg_fail_irq;
	int				otg_oc_irq;
	int				aicl_done_irq;
	int				usbid_change_irq;
	int				chg_error_irq;
	bool				enable_aicl_wake;

	/* psy */
	struct power_supply		*usb_psy;
	struct power_supply		batt_psy;
	struct power_supply		dc_psy;
	struct power_supply		*bms_psy;
	struct power_supply		*typec_psy;
	int				dc_psy_type;
	const char			*bms_psy_name;
	const char			*battery_psy_name;
	bool				psy_registered;

	struct smbchg_regulator		otg_vreg;
	struct smbchg_regulator		ext_otg_vreg;
	struct work_struct		usb_set_online_work;
	struct delayed_work		vfloat_adjust_work;
	struct delayed_work		hvdcp_det_work;
	spinlock_t			sec_access_lock;
	struct mutex			therm_lvl_lock;
	struct mutex			usb_set_online_lock;
	struct mutex			pm_lock;
	/* aicl deglitch workaround */
	unsigned long			first_aicl_seconds;
	int				aicl_irq_count;
	struct mutex			usb_status_lock;
	bool				hvdcp_3_det_ignore_uv;
	struct completion		src_det_lowered;
	struct completion		src_det_raised;
	struct completion		usbin_uv_lowered;
	struct completion		usbin_uv_raised;
	int				pulse_cnt;
	struct led_classdev		led_cdev;
	bool				skip_usb_notification;
	u32				vchg_adc_channel;
	struct qpnp_vadc_chip		*vchg_vadc_dev;

	/* voters */
	struct votable			*fcc_votable;
	struct votable			*usb_icl_votable;
	struct votable			*dc_icl_votable;
	struct votable			*usb_suspend_votable;
	struct votable			*dc_suspend_votable;
	struct votable			*battchg_suspend_votable;
	struct wake_lock chg_wake_lock;
	bool oem_lcd_is_on;
	bool probe_success;
	int lcd_on_iusb;
	int typeC_support_current_ma;
	/*Modify for  charge standard */
	chg_cv_battery_temp_region_type mBatteryTempRegion;
	chg_charger_status_type charger_status;
	enum chg_battery_status_type battery_status;
	/* david.liu@oneplus.tw,20151119  Add delayed workqueue to re-detect charger type */
	struct delayed_work		re_det_work;
	struct delayed_work		check_switch_dash_work;
	struct delayed_work		nonstandard_charger_charge_current_set_work;

	short	mBatteryTempBoundT0;
	short	mBatteryTempBoundT1;
	short	mBatteryTempBoundT2;
	short	mBatteryTempBoundT3;
	short	mBatteryTempBoundT4;
	short	mBatteryTempBoundT5;
	short	mBatteryTempBoundT6;
	bool	oem_disable_charge;
	bool	oem_enable_charge;
	bool chg_done;
	bool recharge_pending;
	bool recharge_status;
	bool re_detect_charger_type_status;

	int temp_more_cool_vbatdel ;
	int temp_more_cool_rechgvbat;
	int temp_cool_vbatdel	  ;
	int temp_cool_recharge_vbatdel;


	int temp_littel_cool_vbatdel;
	int temp_littel_cool_recharge_vbatdel;

	int temp_prenormal_vbatdel	   ;
	int temp_prenormal_recharge_vbatdel;

	int temp_normal_vbatdel 	;
	int temp_normal_recharge_vbatdel;

	int temp_warm_vbatdel	   ;
	int temp_warm_recharge_vbatdel;


	int temp_more_cool_current ;// 300
	int temp_cool_current	  ; // 900
	int temp_littel_cool_current;// 1500
	int temp_prenormal_current     ;
	int temp_normal_current 	;// 2000
	int temp_warm_current	   ;//	900
	bool temp_littel_cool_set_current_0_point_25c;

	int temp_normal_current_hvdcp;	// 3000
	int temp_warm_current_hvdcp;	// 1200

	struct delayed_work 	update_heartbeat_work;
	int batt_health;
	bool is_power_changed;
	unsigned int aicl_current;
	bool time_out;
	bool fastchg_start_disable_normal_chg;
	bool allow_print_log;
#if defined(CONFIG_FB)
	/* yangfangbiao@oneplus.cn,20150519  Add for reset charge current when screen is off */
	struct notifier_block fb_notif;
#endif /*CONFIG_FB*/
	struct notifier_block typec_notif;

	bool ship_mode;

	struct votable			*hw_aicl_rerun_disable_votable;
	struct votable			*hw_aicl_rerun_enable_indirect_votable;
	struct votable			*aicl_deglitch_short_votable;
};

enum qpnp_schg {
	QPNP_SCHG,
	QPNP_SCHG_LITE,
};

static char *version_str[] = {
	[QPNP_SCHG]		= "SCHG",
	[QPNP_SCHG_LITE]	= "SCHG_LITE",
};

enum pmic_subtype {
	PMI8994		= 10,
	PMI8950		= 17,
	PMI8996		= 19,
	PMI8937		= 55,
};

enum smbchg_wa {
	SMBCHG_AICL_DEGLITCH_WA = BIT(0),
	SMBCHG_HVDCP_9V_EN_WA	= BIT(1),
	SMBCHG_USB100_WA = BIT(2),
	SMBCHG_BATT_OV_WA = BIT(3),
	SMBCHG_CC_ESR_WA = BIT(4),
	SMBCHG_FLASH_ICL_DISABLE_WA = BIT(5),
	SMBCHG_RESTART_WA = BIT(6),
	SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA = BIT(7),
};

enum print_reason {
	PR_REGISTER	= BIT(0),
	PR_INTERRUPT	= BIT(1),
	PR_STATUS	= BIT(2),
	PR_DUMP		= BIT(3),
	PR_PM		= BIT(4),
	PR_MISC		= BIT(5),
	PR_WIPOWER	= BIT(6),
	PR_TYPEC	= BIT(7),
};

enum wake_reason {
	PM_PARALLEL_CHECK = BIT(0),
	PM_REASON_VFLOAT_ADJUST = BIT(1),
	PM_ESR_PULSE = BIT(2),
	PM_PARALLEL_TAPER = BIT(3),
};

enum fcc_voters {
	ESR_PULSE_FCC_VOTER,
	BATT_TYPE_FCC_VOTER,
	RESTRICTED_CHG_FCC_VOTER,
	NUM_FCC_VOTER,
};

enum icl_voters {
	PSY_ICL_VOTER,
	THERMAL_ICL_VOTER,
	HVDCP_ICL_VOTER,
	USER_ICL_VOTER,
	WEAK_CHARGER_ICL_VOTER,
	SW_AICL_ICL_VOTER,
	CHG_SUSPEND_WORKAROUND_ICL_VOTER,
	NUM_ICL_VOTER,
};

enum enable_voters {
	/* userspace has suspended charging altogether */
	USER_EN_VOTER,
	/*
	 * this specific path has been suspended through the power supply
	 * framework
	 */
	POWER_SUPPLY_EN_VOTER,
	/*
	 * the usb driver has suspended this path by setting a current limit
	 * of < 2MA
	 */
	USB_EN_VOTER,
	/*
	 * when a wireless charger comes online,
	 * the dc path is suspended for a second
	 */
	WIRELESS_EN_VOTER,
	/*
	 * the thermal daemon can suspend a charge path when the system
	 * temperature levels rise
	 */
	THERMAL_EN_VOTER,
	/*
	 * an external OTG supply is being used, suspend charge path so the
	 * charger does not accidentally try to charge from the external supply.
	 */
	OTG_EN_VOTER,
	/*
	 * the charger is very weak, do not draw any current from it
	 */
	WEAK_CHARGER_EN_VOTER,
	/*
	 * fake battery voter, if battery id-resistance around 7.5 Kohm
	 */
	FAKE_BATTERY_EN_VOTER,
	NUM_EN_VOTERS,
};

enum battchg_enable_voters {
	/* userspace has disabled battery charging */
	BATTCHG_USER_EN_VOTER,
	/* battery charging disabled while loading battery profiles */
	BATTCHG_UNKNOWN_BATTERY_EN_VOTER,
	NUM_BATTCHG_EN_VOTERS,
};

enum hw_aicl_rerun_enable_indirect_voters {
	/* enabled via device tree */
	DEFAULT_CONFIG_HW_AICL_VOTER,
	/* Varb workaround voter */
	VARB_WORKAROUND_VOTER,
	/* SHUTDOWN workaround voter */
	SHUTDOWN_WORKAROUND_VOTER,
	NUM_HW_AICL_RERUN_ENABLE_INDIRECT_VOTERS,
};

enum hw_aicl_rerun_disable_voters {
	/* the results from enabling clients */
	HW_AICL_RERUN_ENABLE_INDIRECT_VOTER,
	/* Weak charger voter */
	WEAK_CHARGER_HW_AICL_VOTER,
	NUM_HW_AICL_DISABLE_VOTERS,
};

enum aicl_short_deglitch_voters {
	/* Varb workaround voter */
	VARB_WORKAROUND_SHORT_DEGLITCH_VOTER,
	/* QC 2.0 */
	HVDCP_SHORT_DEGLITCH_VOTER,
	NUM_HW_SHORT_DEGLITCH_VOTERS,
};
static int smbchg_debug_mask;
module_param_named(
		debug_mask, smbchg_debug_mask, int, S_IRUSR | S_IWUSR
		);

static int smbchg_parallel_en = 1;
module_param_named(
		parallel_en, smbchg_parallel_en, int, S_IRUSR | S_IWUSR
		);

static int smbchg_main_chg_fcc_percent = 50;
module_param_named(
	main_chg_fcc_percent, smbchg_main_chg_fcc_percent,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_main_chg_icl_percent = 60;
module_param_named(
	main_chg_icl_percent, smbchg_main_chg_icl_percent,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_default_hvdcp_icl_ma = 1800;
module_param_named(
	default_hvdcp_icl_ma, smbchg_default_hvdcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_default_hvdcp3_icl_ma = 3000;
module_param_named(
	default_hvdcp3_icl_ma, smbchg_default_hvdcp3_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int smbchg_default_dcp_icl_ma = 1800;
module_param_named(
	default_dcp_icl_ma, smbchg_default_dcp_icl_ma,
	int, S_IRUSR | S_IWUSR
);

static int wipower_dyn_icl_en;
module_param_named(
		dynamic_icl_wipower_en, wipower_dyn_icl_en,
		int, S_IRUSR | S_IWUSR
		);

static int wipower_dcin_interval = ADC_MEAS1_INTERVAL_2P0MS;
module_param_named(
		wipower_dcin_interval, wipower_dcin_interval,
		int, S_IRUSR | S_IWUSR
		);

#define WIPOWER_DEFAULT_HYSTERISIS_UV	250000
static int wipower_dcin_hyst_uv = WIPOWER_DEFAULT_HYSTERISIS_UV;
module_param_named(
		wipower_dcin_hyst_uv, wipower_dcin_hyst_uv,
		int, S_IRUSR | S_IWUSR
		);

#define pr_smb(reason, fmt, ...)				\
	do {							\
		if (smbchg_debug_mask & (reason))		\
		pr_info(fmt, ##__VA_ARGS__);		\
		else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
	} while (0)

#define pr_smb_rt(reason, fmt, ...)					\
	do {								\
		if (smbchg_debug_mask & (reason))			\
		pr_info_ratelimited(fmt, ##__VA_ARGS__);	\
		else							\
		pr_debug_ratelimited(fmt, ##__VA_ARGS__);	\
	} while (0)

struct smbchg_chip *g_chip;
static int smbchg_charging_en(struct smbchg_chip *chip, bool en);
static int get_prop_charger_voltage_now(struct smbchg_chip *chip);
static chg_cv_battery_temp_region_type qpnp_battery_temp_region_get(struct smbchg_chip *chip);
static void qpnp_check_charger_uovp(struct smbchg_chip *chip);
static int qpnp_check_battery_temp(struct smbchg_chip *chip);
static int get_prop_charger_voltage_now(struct smbchg_chip *chip);
static int smbchg_set_usb_current_max(struct smbchg_chip *chip,
		int current_ma);
static void qpnp_battery_temp_region_set(struct smbchg_chip *chip,
		chg_cv_battery_temp_region_type batt_temp_region);
void set_chg_parameter_in_different_temp(struct smbchg_chip *chip,int ibat,int vfloat );
static int fake_insertion_removal(struct smbchg_chip *chip, bool insertion);
static int update_dash_unplug_status(void);
static int get_prop_fast_chg_started(struct smbchg_chip *chip);
static int set_dash_charger_present(int status);
static int qpnp_get_fast_chg_ing(struct smbchg_chip *chip);

static int smbchg_read(struct smbchg_chip *chip, u8 *val,
		u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "spmi read failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}
	return 0;
}

/*
 * Writes an arbitrary number of bytes to a specified register
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_write(struct smbchg_chip *chip, u8 *val,
		u16 addr, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (addr == 0) {
		dev_err(chip->dev, "addr cannot be zero addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, addr, val, count);
	if (rc) {
		dev_err(chip->dev, "write failed addr=0x%02x sid=0x%02x rc=%d\n",
				addr, spmi->sid, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * Do not use this function for register writes if possible. Instead use the
 * smbchg_masked_write function.
 *
 * The sec_access_lock must be held for all register writes and this function
 * does not do that. If this function is used, please hold the spinlock or
 * random secure access writes may fail.
 */
static int smbchg_masked_write_raw(struct smbchg_chip *chip, u16 base, u8 mask,
		u8 val)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi read failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;

	pr_smb(PR_REGISTER, "addr = 0x%x writing 0x%x\n", base, reg);

	rc = smbchg_write(chip, &reg, base, 1);
	if (rc) {
		dev_err(chip->dev, "spmi write failed: addr=%03X, rc=%d\n",
				base, rc);
		return rc;
	}

	return 0;
}

/*
 * Writes a register to the specified by the base and limited by the bit mask
 *
 * This function holds a spin lock to ensure secure access register writes goes
 * through. If the secure access unlock register is armed, any old register
 * write can unarm the secure access unlock, causing the next write to fail.
 *
 * Note: do not use this for sec_access registers. Instead use the function
 * below: smbchg_sec_masked_write
 */
static int smbchg_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
		u8 val)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&chip->sec_access_lock, flags);
	rc = smbchg_masked_write_raw(chip, base, mask, val);
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);

	return rc;
}

/*
 * Unlocks sec access and writes to the register specified.
 *
 * This function holds a spin lock to exclude other register writes while
 * the two writes are taking place.
 */
#define SEC_ACCESS_OFFSET	0xD0
#define SEC_ACCESS_VALUE	0xA5
#define PERIPHERAL_MASK		0xFF
static int smbchg_sec_masked_write(struct smbchg_chip *chip, u16 base, u8 mask,
		u8 val)
{
	unsigned long flags;
	int rc;
	u16 peripheral_base = base & (~PERIPHERAL_MASK);

	spin_lock_irqsave(&chip->sec_access_lock, flags);

	rc = smbchg_masked_write_raw(chip, peripheral_base + SEC_ACCESS_OFFSET,
			SEC_ACCESS_VALUE, SEC_ACCESS_VALUE);
	if (rc) {
		dev_err(chip->dev, "Unable to unlock sec_access: %d", rc);
		goto out;
	}

	rc = smbchg_masked_write_raw(chip, base, mask, val);

out:
	spin_unlock_irqrestore(&chip->sec_access_lock, flags);
	return rc;
}

static void smbchg_stay_awake(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons | reason;
	if (reasons != 0 && chip->wake_reasons == 0) {
		pr_smb(PR_PM, "staying awake: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_stay_awake(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
}

static void smbchg_relax(struct smbchg_chip *chip, int reason)
{
	int reasons;

	mutex_lock(&chip->pm_lock);
	reasons = chip->wake_reasons & (~reason);
	if (reasons == 0 && chip->wake_reasons != 0) {
		pr_smb(PR_PM, "relaxing: 0x%02x (bit %d)\n",
				reasons, reason);
		pm_relax(chip->dev);
	}
	chip->wake_reasons = reasons;
	mutex_unlock(&chip->pm_lock);
};

enum pwr_path_type {
	UNKNOWN = 0,
	PWR_PATH_BATTERY = 1,
	PWR_PATH_USB = 2,
	PWR_PATH_DC = 3,
};

#define PWR_PATH		0x08
#define PWR_PATH_MASK		0x03
static enum pwr_path_type smbchg_get_pwr_path(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + PWR_PATH, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read PWR_PATH rc = %d\n", rc);
		return PWR_PATH_BATTERY;
	}

	return reg & PWR_PATH_MASK;
}

#define RID_STS				0xB
#define RID_MASK			0xF
#define IDEV_STS			0x8
#define RT_STS				0x10
#define USBID_MSB			0xE
#define USBIN_UV_BIT			BIT(0)
#define USBIN_OV_BIT			BIT(1)
#define USBIN_SRC_DET_BIT		BIT(2)
#define FMB_STS_MASK			SMB_MASK(3, 0)
#define USBID_GND_THRESHOLD		0x495
static bool is_otg_present_schg(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;
	u8 usbid_reg[2];
	u16 usbid_val;
	/*
	 * After the falling edge of the usbid change interrupt occurs,
	 * there may still be some time before the ADC conversion for USB RID
	 * finishes in the fuel gauge. In the worst case, this could be up to
	 * 15 ms.
	 *
	 * Sleep for 20 ms (minimum msleep time) to wait for the conversion to
	 * finish and the USB RID status register to be updated before trying
	 * to detect OTG insertions.
	 */

	msleep(20);

	/*
	 * There is a problem with USBID conversions on PMI8994 revisions
	 * 2.0.0. As a workaround, check that the cable is not
	 * detected as factory test before enabling OTG.
	 */
	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read IDEV_STS rc = %d\n", rc);
		return false;
	}

	if ((reg & FMB_STS_MASK) != 0) {
		pr_smb(PR_STATUS, "IDEV_STS = %02x, not ground\n", reg);
		return false;
	}

	rc = smbchg_read(chip, usbid_reg, chip->usb_chgpth_base + USBID_MSB, 2);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read USBID rc = %d\n", rc);
		return false;
	}
	usbid_val = (usbid_reg[0] << 8) | usbid_reg[1];

	if (usbid_val > USBID_GND_THRESHOLD) {
		pr_smb(PR_STATUS, "USBID = 0x%04x, too high to be ground\n",
				usbid_val);
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RID_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read usb rid status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "RID_STS = %02x\n", reg);

	return (reg & RID_MASK) == 0;
}

#define RID_GND_DET_STS			BIT(2)
static bool is_otg_present_schg_lite(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->otg_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read otg RT status rc = %d\n", rc);
		return false;
	}

	return !!(reg & RID_GND_DET_STS);
}

static bool is_otg_present(struct smbchg_chip *chip)
{
	if (chip->schg_version == QPNP_SCHG_LITE)
		return is_otg_present_schg_lite(chip);

	return is_otg_present_schg(chip);
}

#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)
#define INPUT_STS			0x0D
#define DCIN_UV_BIT			BIT(0)
#define DCIN_OV_BIT			BIT(1)
static bool is_dc_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->dc_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read dc status rc = %d\n", rc);
		return false;
	}

	if ((reg & DCIN_UV_BIT) || (reg & DCIN_OV_BIT))
		return false;

	return true;
}

static bool is_usb_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	if (!(reg & USBIN_SRC_DET_BIT) || (reg & USBIN_OV_BIT))
		return false;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + INPUT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return false;
	}

	return !!(reg & (USBIN_9V | USBIN_UNREG | USBIN_LV));
}

static char *usb_type_str[] = {
	"SDP",		/* bit 0 */
	"OTHER",	/* bit 1 */
	"DCP",		/* bit 2 */
	"CDP",		/* bit 3 */
	"NONE",		/* bit 4 error case */
};

#define N_TYPE_BITS		4
#define TYPE_BITS_OFFSET	4

static int get_type(u8 type_reg)
{
	unsigned long type = type_reg;
	type >>= TYPE_BITS_OFFSET;
	return find_first_bit(&type, N_TYPE_BITS);
}

/* helper to return the string of USB type */
static inline char *get_usb_type_name(int type)
{
	return usb_type_str[type];
}

static enum power_supply_type usb_type_enum[] = {
	POWER_SUPPLY_TYPE_USB,		/* bit 0 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 1 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 2 */
	POWER_SUPPLY_TYPE_USB_CDP,	/* bit 3 */
	POWER_SUPPLY_TYPE_USB_DCP,	/* bit 4 error case, report DCP */
};

/* helper to return enum power_supply_type of USB type */
static inline enum power_supply_type get_usb_supply_type(int type)
{
	return usb_type_enum[type];
}

static void read_usb_type(struct smbchg_chip *chip, char **usb_type_name,
		enum power_supply_type *usb_supply_type)
{
	int rc, type;
	u8 reg;

	if(chip->hvdcp_present) {
		*usb_type_name = "HVDCP";
		*usb_supply_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		return;
	}
	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		*usb_type_name = "Other";
		*usb_supply_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}
	type = get_type(reg);
	*usb_type_name = get_usb_type_name(type);
	*usb_supply_type = get_usb_supply_type(type);
}

#define CHGR_STS			0x0E
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_INHIBIT_BIT			BIT(1)
#define BAT_TCC_REACHED_BIT		BIT(7)
static int get_prop_batt_status(struct smbchg_chip *chip)
{
	int rc, status = POWER_SUPPLY_STATUS_DISCHARGING;
	u8 reg = 0, chg_type;
	bool charger_present, chg_inhibit;

	charger_present = is_usb_present(chip) | is_dc_present(chip) |
		chip->hvdcp_3_det_ignore_uv;
	if (!charger_present)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & BAT_TCC_REACHED_BIT)
		return POWER_SUPPLY_STATUS_FULL;

	chg_inhibit = reg & CHG_INHIBIT_BIT;
	if (chg_inhibit)
		return POWER_SUPPLY_STATUS_FULL;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & CHG_HOLD_OFF_BIT) {
		/*
		 * when chg hold off happens the battery is
		 * not charging
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;

	if (chg_type == BATT_NOT_CHG_VAL && !chip->hvdcp_3_det_ignore_uv)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;
out:
	pr_smb_rt(PR_MISC, "CHGR_STS = 0x%02x\n", reg);
	return status;
}

#define BAT_PRES_STATUS			0x08
#define BAT_PRES_BIT			BIT(7)
static int get_prop_batt_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->bat_if_base + BAT_PRES_STATUS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	return !!(reg & BAT_PRES_BIT);
}

static int get_prop_charge_type(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, chg_type;

	rc = smbchg_read(chip, &reg, chip->chgr_base + CHGR_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read CHGR_STS rc = %d\n", rc);
		return 0;
	}

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int set_property_on_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	ret.intval = val;
	rc = chip->bms_psy->set_property(chip->bms_psy, prop, &ret);
	if (rc)
		pr_smb(PR_STATUS,
				"bms psy does not allow updating prop %d rc = %d\n",
				prop, rc);

	return rc;
}

static int get_property_from_fg(struct smbchg_chip *chip,
		enum power_supply_property prop, int *val)
{
	int rc;
	union power_supply_propval ret = {0, };

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (!chip->bms_psy) {
		pr_smb(PR_STATUS, "no bms psy found\n");
		return -EINVAL;
	}

	rc = chip->bms_psy->get_property(chip->bms_psy, prop, &ret);
	if (rc) {
		pr_smb(PR_STATUS,
				"bms psy doesn't support reading prop %d rc = %d\n",
				prop, rc);
		return rc;
	}

	*val = ret.intval;
	return rc;
}

#define DEFAULT_BATT_CAPACITY	50
static int get_prop_batt_capacity(struct smbchg_chip *chip)
{
	int capacity, rc;

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CAPACITY, &capacity);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get capacity rc = %d\n", rc);
		capacity = DEFAULT_BATT_CAPACITY;
	}
	return capacity;
}

#define DEFAULT_BATT_TEMP		200
static int get_prop_batt_temp(struct smbchg_chip *chip)
{
	int temp, rc;
	if(use_fake_temp)
		return fake_temp;
	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_TEMP, &temp);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get temperature rc = %d\n", rc);
		temp = DEFAULT_BATT_TEMP;
	}
	return temp;
}

#define DEFAULT_BATT_CURRENT_NOW	0
static int get_prop_batt_current_now(struct smbchg_chip *chip)
{
	int ua, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get current rc = %d\n", rc);
		ua = DEFAULT_BATT_CURRENT_NOW;
	}
	return ua;
}

#define DEFAULT_BATT_VOLTAGE_NOW	0
static int get_prop_batt_voltage_now(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_NOW;
	}
	return uv;
}

#define DEFAULT_BATT_VOLTAGE_MAX_DESIGN	4200000
static int get_prop_batt_voltage_max_design(struct smbchg_chip *chip)
{
	int uv, rc;

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &uv);
	if (rc) {
		pr_smb(PR_STATUS, "Couldn't get voltage rc = %d\n", rc);
		uv = DEFAULT_BATT_VOLTAGE_MAX_DESIGN;
	}
	return uv;
}

static int get_prop_batt_health(struct smbchg_chip *chip)
{
	if (chip->batt_hot)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		return POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		return POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		return POWER_SUPPLY_HEALTH_COOL;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static void get_property_from_typec(struct smbchg_chip *chip,
				enum power_supply_property property,
				union power_supply_propval *prop)
{
	int rc;

	rc = chip->typec_psy->get_property(chip->typec_psy, property, prop);
	if (rc)
		pr_smb(PR_TYPEC,
			"typec psy doesn't support reading prop %d rc = %d\n",
			property, rc);
}

static void update_typec_status(struct smbchg_chip *chip)
{
	union power_supply_propval type = {0, };
	union power_supply_propval capability = {0, };
	int rc;

	get_property_from_typec(chip, POWER_SUPPLY_PROP_TYPE, &type);
	if (type.intval != POWER_SUPPLY_TYPE_UNKNOWN) {
		get_property_from_typec(chip,
				POWER_SUPPLY_PROP_CURRENT_CAPABILITY,
				&capability);
		chip->typec_current_ma = capability.intval;

		if (!chip->skip_usb_notification) {
			rc = chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
				&capability);
			if (rc)
				pr_err("typec failed to set current max rc=%d\n",
					rc);
			pr_smb(PR_TYPEC, "SMB Type-C mode = %d, current=%d\n",
					type.intval, capability.intval);
		}
	} else {
		pr_smb(PR_TYPEC,
			"typec detection not completed continuing with USB update\n");
	}
}

/*
 * finds the index of the closest value in the array. If there are two that
 * are equally close, the lower index will be returned
 */
static int find_closest_in_array(const int *arr, int len, int val)
{
	int i, closest = 0;

	if (len == 0)
		return closest;
	for (i = 0; i < len; i++)
		if (abs(val - arr[i]) < abs(val - arr[closest]))
			closest = i;

	return closest;
}

/* finds the index of the closest smaller value in the array. */
static int find_smaller_in_array(const int *table, int val, int len)
{
	int i;

	for (i = len - 1; i >= 0; i--) {
		if (val >= table[i])
			break;
	}

	return i;
}

static const int iterm_ma_table_8994[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	500,
	600
};

static const int iterm_ma_table_8996[] = {
	300,
	50,
	100,
	150,
	200,
	250,
	400,
	500
};

static const int usb_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static const int usb_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
	2500,
	2600,
	2700,
	2800,
	2900,
	3000
};

static int dc_ilim_ma_table_8994[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static int dc_ilim_ma_table_8996[] = {
	300,
	400,
	500,
	600,
	700,
	800,
	900,
	1000,
	1100,
	1200,
	1300,
	1400,
	1450,
	1500,
	1550,
	1600,
	1700,
	1800,
	1900,
	1950,
	2000,
	2050,
	2100,
	2200,
	2300,
	2400,
};

static const int fcc_comp_table_8994[] = {
	250,
	700,
	900,
	1200,
};

static const int fcc_comp_table_8996[] = {
	250,
	1100,
	1200,
	1500,
};

static const int aicl_rerun_period[] = {
	45,
	90,
	180,
	360,
};

static const int aicl_rerun_period_schg_lite[] = {
	3,	/* 2.8s  */
	6,	/* 5.6s  */
	11,	/* 11.3s */
	23,	/* 22.5s */
	45,
	90,
	180,
	360,
};

static void use_pmi8994_tables(struct smbchg_chip *chip)
{
	chip->tables.usb_ilim_ma_table = usb_ilim_ma_table_8994;
	chip->tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8994);
	chip->tables.dc_ilim_ma_table = dc_ilim_ma_table_8994;
	chip->tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8994);
	chip->tables.iterm_ma_table = iterm_ma_table_8994;
	chip->tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8994);
	chip->tables.fcc_comp_table = fcc_comp_table_8994;
	chip->tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8994);
	chip->tables.rchg_thr_mv = 200;
	chip->tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

static void use_pmi8996_tables(struct smbchg_chip *chip)
{
	chip->tables.usb_ilim_ma_table = usb_ilim_ma_table_8996;
	chip->tables.usb_ilim_ma_len = ARRAY_SIZE(usb_ilim_ma_table_8996);
	chip->tables.dc_ilim_ma_table = dc_ilim_ma_table_8996;
	chip->tables.dc_ilim_ma_len = ARRAY_SIZE(dc_ilim_ma_table_8996);
	chip->tables.iterm_ma_table = iterm_ma_table_8996;
	chip->tables.iterm_ma_len = ARRAY_SIZE(iterm_ma_table_8996);
	chip->tables.fcc_comp_table = fcc_comp_table_8996;
	chip->tables.fcc_comp_len = ARRAY_SIZE(fcc_comp_table_8996);
	chip->tables.rchg_thr_mv = 150;
	chip->tables.aicl_rerun_period_table = aicl_rerun_period;
	chip->tables.aicl_rerun_period_len = ARRAY_SIZE(aicl_rerun_period);
}

#define CMD_CHG_REG	0x42
#define EN_BAT_CHG_BIT		BIT(1)
static int smbchg_charging_en(struct smbchg_chip *chip, bool en)
{
	/* The en bit is configured active low */
	dev_err(chip->dev, "%s:en=%d",__func__,en);
	return smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			EN_BAT_CHG_BIT, en ? 0 : EN_BAT_CHG_BIT);
}

#define CMD_IL			0x40
#define USBIN_SUSPEND_BIT	BIT(4)
#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define CURRENT_1500_MA		1500
#define SUSPEND_CURRENT_MA	2
#define ICL_OVERRIDE_BIT	BIT(2)
static int smbchg_usb_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc;
	dev_err(chip->dev, "%s:%d\n",__func__,suspend);
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_SUSPEND_BIT, suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set usb suspend rc = %d\n", rc);
	return rc;
}

#define DCIN_SUSPEND_BIT	BIT(3)
static int smbchg_dc_suspend(struct smbchg_chip *chip, bool suspend)
{
	int rc = 0;

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			DCIN_SUSPEND_BIT, suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set dc suspend rc = %d\n", rc);
	return rc;
}

#define IL_CFG			0xF2
#define DCIN_INPUT_MASK	SMB_MASK(4, 0)
static int smbchg_set_dc_current_max(struct smbchg_chip *chip, int current_ma)
{
	int i;
	u8 dc_cur_val;

	i = find_smaller_in_array(chip->tables.dc_ilim_ma_table,
			current_ma, chip->tables.dc_ilim_ma_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dma current_table\n",
				current_ma);
		return -EINVAL;
	}

	chip->dc_max_current_ma = chip->tables.dc_ilim_ma_table[i];
	dc_cur_val = i & DCIN_INPUT_MASK;

	pr_smb(PR_STATUS, "dc current set to %d mA\n",
			chip->dc_max_current_ma);
	return smbchg_sec_masked_write(chip, chip->dc_chgpth_base + IL_CFG,
			DCIN_INPUT_MASK, dc_cur_val);
}

#define AICL_WL_SEL_CFG			0xF5
#define AICL_WL_SEL_MASK		SMB_MASK(1, 0)
#define AICL_WL_SEL_SCHG_LITE_MASK	SMB_MASK(2, 0)
static int smbchg_set_aicl_rerun_period_s(struct smbchg_chip *chip,
		int period_s)
{
	int i;
	u8 reg, mask;

	i = find_smaller_in_array(chip->tables.aicl_rerun_period_table,
			period_s, chip->tables.aicl_rerun_period_len);

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %ds in aicl rerun period\n",
				period_s);
		return -EINVAL;
	}

	if (chip->schg_version == QPNP_SCHG_LITE)
		mask = AICL_WL_SEL_SCHG_LITE_MASK;
	else
		mask = AICL_WL_SEL_MASK;

	reg = i & mask;

	pr_smb(PR_STATUS, "aicl rerun period set to %ds\n",
			chip->tables.aicl_rerun_period_table[i]);
	return smbchg_sec_masked_write(chip,
			chip->dc_chgpth_base + AICL_WL_SEL_CFG,
			mask, reg);
}
enum type_c_host_cur{
	TYPE_C_HOST_CUR_NO = 0,  // no current
	TYPE_C_HOST_CUR_900_500,  // default USB
	TYPE_C_HOST_CUR_1500,  // 1.5A
	TYPE_C_HOST_CUR_3000,  // 3A
};


static ssize_t test_chg_vol_show(struct device *dev,
		struct device_attribute *attr,  char *buf) {
	return sprintf(buf, "%d\n", fake_chgvol);
}
static ssize_t test_chg_vol_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size) {
	long val = simple_strtol(buf, NULL, 10);

	if(val == 9898) {
		use_fake_chgvol= false;
		fake_chgvol= 0;
	}else {
		use_fake_chgvol = true;
		fake_chgvol = val;
	}

	return size;
}
static DEVICE_ATTR(test_chg_vol, S_IRUGO | S_IWUSR, test_chg_vol_show, test_chg_vol_store);

static ssize_t test_temp_show(struct device *dev,struct device_attribute *attr,  char *buf){
	return sprintf(buf, "%d\n", fake_temp);
}
static ssize_t test_temp_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	long val = simple_strtol(buf, NULL, 10);

	if(val == 9898) {
		use_fake_temp = false;
		fake_temp = 300;
	}else {
		use_fake_temp = true;
		fake_temp = val;
	}

	return size;
}

static DEVICE_ATTR(test_temp, S_IRUGO | S_IWUSR, test_temp_show, test_temp_store);
static ssize_t test_authentic_show(struct device *dev,struct device_attribute *attr,  char *buf){
	return sprintf(buf, "%d\n", fake_authentic);
}
static ssize_t test_authentic_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	long val = simple_strtol(buf, NULL, 10);

	if(val == 9898) {
		use_fake_authentic = false;
		fake_authentic = 4000;
	}else {
		use_fake_authentic = true;
		fake_authentic = val;
	}

	return size;
}

static DEVICE_ATTR(test_authentic, S_IRUGO | S_IWUSR, test_authentic_show, test_authentic_store);
enum chg_protect_status_type {
    CHG_PROTECT_CHARGER_VOP=1, //VCHG >5.8V
    CHG_PROTECT_BATTERY_STATUS_BAD,// 2   battery miss
    CHG_PROTECT_CHG_OVER_TIME, // 3   charge time  too long
    CHG_PROTECT_BATTERY_OVER_VOLTAGE, // 4   vbat >= 4.5
    CHG_PROTECT_TEMP_REGION__HOT, // 5   t >55
    CHG_PROTECT_TEMP_REGION__COLD, // 6  -3 >=  t
    CHG_PROTECT_TEMP_LITTLE__COLD, //7   -3 <=  t < 0
    CHG_PROTECT_TEMP_REGION__COOL, // 8  0<=  t < 10
    CHG_PROTECT_TEMP_REGION__WARM //9  45 <=  t < 55
};

static int get_prop_chg_protect_status(struct smbchg_chip *chip)
{
	int charger_present = 0;
	int temp;
	if(use_fake_authentic)
		return fake_authentic;
	charger_present = is_usb_present(chip) | is_dc_present(chip);
	if (!charger_present)
		return 0;
	temp = get_prop_batt_temp(chip);
	if((CHARGER_STATUS_OVER==chip->charger_status)&&(CHARGER_SOFT_OVP_VOLTAGE -100 <= get_prop_charger_voltage_now(chip)))//charger over voltage
		return CHG_PROTECT_CHARGER_VOP;
	else if(( AUTO_CHARGING_BATT_REMOVE_TEMP > temp)||(1 !=get_prop_batt_present(chip))) //battery remove
		return  CHG_PROTECT_BATTERY_STATUS_BAD;
	else if(BATTERY_STATUS_BAD == chip->battery_status)   //battery over voltage
		return CHG_PROTECT_BATTERY_OVER_VOLTAGE;
	else if(true == chip->time_out)   //chg time too long
		return CHG_PROTECT_CHG_OVER_TIME;
	else if(CV_BATTERY_TEMP_REGION__HOT== qpnp_battery_temp_region_get(chip)) //temp hot
		return CHG_PROTECT_TEMP_REGION__HOT;
	else if(CV_BATTERY_TEMP_REGION__COLD==qpnp_battery_temp_region_get(chip)) //temp cold
		return CHG_PROTECT_TEMP_REGION__COLD;
	else if(CV_BATTERY_TEMP_REGION_LITTLE__COLD==qpnp_battery_temp_region_get(chip) && (true==chip->chg_done || chip->recharge_status==true))//-3 <=  t < 0
		return CHG_PROTECT_TEMP_LITTLE__COLD;
	else if(CV_BATTERY_TEMP_REGION__WARM == qpnp_battery_temp_region_get(chip)&& (true==chip->chg_done || chip->recharge_status==true))// 45 <=  t < 55
		return CHG_PROTECT_TEMP_REGION__WARM;
	else
		return 0;

}

static bool get_prop_fastchg_status (struct smbchg_chip *chip)
{
	bool dash_started;
	int capacity = get_prop_batt_capacity(chip);
	dash_started = chip->dash_present;
		if (dash_started || (chip->hvdcp_present
				&& capacity >= 1 && capacity <= 85))
			return true;

	return false;
}
static bool get_prop_fastchg_starting (struct smbchg_chip *chip)
{
	if( qpnp_get_fast_chg_ing(chip) == 1)
		return true;
	else
		return  false;
}
static struct power_supply *get_parallel_psy(struct smbchg_chip *chip)
{
	if (!chip->parallel.avail)
		return NULL;
	if (chip->parallel.psy)
		return chip->parallel.psy;
	chip->parallel.psy = power_supply_get_by_name("usb-parallel");
	if (!chip->parallel.psy)
		pr_smb(PR_STATUS, "parallel charger not found\n");
	return chip->parallel.psy;
}

static void smbchg_usb_update_online_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
			struct smbchg_chip,
			usb_set_online_work);
	bool user_enabled = !get_client_vote(chip->usb_suspend_votable,
			USER_EN_VOTER);
	int online;

	online = user_enabled && chip->usb_present && !chip->very_weak_charger;
	pr_info("chip->usb_online = %d, online = %d\n", chip->usb_online, online);
	mutex_lock(&chip->usb_set_online_lock);
	if (chip->usb_online != online) {
		pr_smb(PR_MISC, "setting usb psy online = %d\n", online);
		power_supply_set_online(chip->usb_psy, online);
		chip->usb_online = online;
	}
	mutex_unlock(&chip->usb_set_online_lock);
}

#define CHGPTH_CFG		0xF4
#define CFG_USB_2_3_SEL_BIT	BIT(7)
#define CFG_USB_2		0
#define CFG_USB_3		BIT(7)
#define USBIN_INPUT_MASK	SMB_MASK(4, 0)
#define USBIN_MODE_CHG_BIT	BIT(0)
#define USBIN_LIMITED_MODE	0
#define USBIN_HC_MODE		BIT(0)
#define USB51_MODE_BIT		BIT(1)
#define USB51_100MA		0
#define USB51_500MA		BIT(1)
static int smbchg_set_high_usb_chg_current(struct smbchg_chip *chip,
		int current_ma)
{
	int i, rc;
	u8 usb_cur_val;

	if (current_ma == CURRENT_100_MA) {
		rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				CFG_USB_2_3_SEL_BIT, CFG_USB_2);
		if (rc < 0) {
			pr_err("Couldn't set CFG_USB_2 rc=%d\n", rc);
			return rc;
		}

		rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT | USB51_MODE_BIT | ICL_OVERRIDE_BIT,
				USBIN_LIMITED_MODE | USB51_100MA | ICL_OVERRIDE_BIT);
		if (rc < 0) {
			pr_err("Couldn't set ICL_OVERRIDE rc=%d\n", rc);
			return rc;
		}

		pr_smb(PR_STATUS,
				"Forcing 100mA current limit\n");
		chip->usb_max_current_ma = CURRENT_100_MA;
		return rc;
	}

	i = find_smaller_in_array(chip->tables.usb_ilim_ma_table,
			current_ma, chip->tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
				"Cannot find %dma current_table using %d\n",
				current_ma, CURRENT_150_MA);

		rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				CFG_USB_2_3_SEL_BIT, CFG_USB_3);
		rc |= smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
				USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
				USBIN_LIMITED_MODE | USB51_100MA);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_150_MA, rc);
		else
			chip->usb_max_current_ma = 150;
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + IL_CFG,
			USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USBIN_MODE_CHG_BIT, USBIN_HC_MODE);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't write cfg 5 rc = %d\n", rc);
	chip->usb_max_current_ma = chip->tables.usb_ilim_ma_table[i];
	return rc;
}

/* if APSD results are used
 *	if SDP is detected it will look at 500mA setting
 *		if set it will draw 500mA
 *		if unset it will draw 100mA
 *	if CDP/DCP it will look at 0x0C setting
 *		i.e. values in 0x41[1, 0] does not matter
 */
static int smbchg_set_usb_current_max(struct smbchg_chip *chip,
		int current_ma)
{
	int rc = 0;

	/*
	 * if the battery is not present, do not allow the usb ICL to lower in
	 * order to avoid browning out the device during a hotswap.
	 */
	if (!chip->batt_present && current_ma < chip->usb_max_current_ma) {
		pr_info_ratelimited("Ignoring usb current->%d, battery is absent\n",
				current_ma);
		return 0;
	}
	dev_err(chip->dev,"USB current_ma = %d,typeC_support_current_ma=%d\n", current_ma,chip->typeC_support_current_ma);
	if(chip->typeC_support_current_ma < current_ma)
		current_ma =chip->typeC_support_current_ma;
	if (current_ma <= SUSPEND_CURRENT_MA) {
		/* suspend the usb if current <= 2mA */
		rc = vote(chip->usb_suspend_votable, USB_EN_VOTER, true, 0);
		chip->usb_max_current_ma = 0;
		goto out;
	} else {
		rc = vote(chip->usb_suspend_votable, USB_EN_VOTER, false, 0);
	}

	switch (chip->usb_supply_type) {
    	case POWER_SUPPLY_TYPE_USB:
    		if ((current_ma < CURRENT_150_MA) &&
    				(chip->wa_flags & SMBCHG_USB100_WA))
    			current_ma = CURRENT_150_MA;

    		if (current_ma < CURRENT_150_MA) {
    			/* force 100mA */
    			rc = smbchg_sec_masked_write(chip,
    					chip->usb_chgpth_base + CHGPTH_CFG,
    					CFG_USB_2_3_SEL_BIT, CFG_USB_2);
    			if (rc < 0) {
    				pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
    				goto out;
    			}
    			rc = smbchg_masked_write(chip,
    					chip->usb_chgpth_base + CMD_IL,
    					USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
    					USBIN_LIMITED_MODE | USB51_100MA);
    			if (rc < 0) {
    				pr_err("Couldn't set CMD_IL rc = %d\n", rc);
    				goto out;
    			}
    			chip->usb_max_current_ma = 100;
    		}
			/* specific current values */
			if (current_ma == CURRENT_150_MA) {
				rc = smbchg_sec_masked_write(chip,
						chip->usb_chgpth_base + CHGPTH_CFG,
						CFG_USB_2_3_SEL_BIT, CFG_USB_3);
				if (rc < 0) {
					pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
					goto out;
				}
				rc = smbchg_masked_write(chip,
						chip->usb_chgpth_base + CMD_IL,
						USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
						USBIN_LIMITED_MODE | USB51_100MA);
				if (rc < 0) {
					pr_err("Couldn't set CMD_IL rc = %d\n", rc);
					goto out;
				}
				chip->usb_max_current_ma = 150;
			}
			if (current_ma == CURRENT_500_MA) {
				rc = smbchg_sec_masked_write(chip,
						chip->usb_chgpth_base + CHGPTH_CFG,
						CFG_USB_2_3_SEL_BIT, CFG_USB_2);
				if (rc < 0) {
					pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
					goto out;
				}
				rc = smbchg_masked_write(chip,
						chip->usb_chgpth_base + CMD_IL,
						USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
						USBIN_LIMITED_MODE | USB51_500MA);
				if (rc < 0) {
					pr_err("Couldn't set CMD_IL rc = %d\n", rc);
					goto out;
				}
				chip->usb_max_current_ma = 500;
			}
			if (current_ma == CURRENT_900_MA) {
				rc = smbchg_sec_masked_write(chip,
						chip->usb_chgpth_base + CHGPTH_CFG,
						CFG_USB_2_3_SEL_BIT, CFG_USB_3);
				if (rc < 0) {
					pr_err("Couldn't set CHGPTH_CFG rc = %d\n", rc);
					goto out;
				}
				rc = smbchg_masked_write(chip,
						chip->usb_chgpth_base + CMD_IL,
						USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
						USBIN_LIMITED_MODE | USB51_500MA);
				if (rc < 0) {
					pr_err("Couldn't set CMD_IL rc = %d\n", rc);
					goto out;
				}
				chip->usb_max_current_ma = 900;
			}
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			if (current_ma < CURRENT_1500_MA) {
				/* use override for CDP */
				rc = smbchg_masked_write(chip,
						chip->usb_chgpth_base + CMD_IL,
						ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT);
				if (rc < 0)
					pr_err("Couldn't set override rc = %d\n", rc);
			}
			/* fall through */
		default:
			rc = smbchg_set_high_usb_chg_current(chip, current_ma);
			if (rc < 0)
				pr_err("Couldn't set %dmA rc = %d\n", current_ma, rc);
			break;
	}

out:
	pr_smb(PR_STATUS, "usb type = %d current set to %d mA\n",
			chip->usb_supply_type, chip->usb_max_current_ma);
	return rc;
}

#define USBIN_HVDCP_STS				0x0C
#define USBIN_HVDCP_SEL_BIT			BIT(4)
#define USBIN_HVDCP_SEL_9V_BIT			BIT(1)
#define SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT	BIT(2)
#define SCHG_LITE_USBIN_HVDCP_SEL_BIT		BIT(0)
static int smbchg_get_min_parallel_current_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel, hvdcp_sel_9v;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb status rc = %d\n", rc);
		return 0;
	}
	if (chip->schg_version == QPNP_SCHG_LITE) {
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = SCHG_LITE_USBIN_HVDCP_SEL_9V_BIT;
	} else {
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;
		hvdcp_sel_9v = USBIN_HVDCP_SEL_9V_BIT;
	}

	if ((reg & hvdcp_sel) && (reg & hvdcp_sel_9v))
		return chip->parallel.min_9v_current_thr_ma;
	return chip->parallel.min_current_thr_ma;
}

static bool is_hvdcp_present(struct smbchg_chip *chip)
{
	int rc;
	u8 reg, hvdcp_sel;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc < 0) {
		pr_err("Couldn't read hvdcp status rc = %d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP_STS = 0x%02x\n", reg);
	/*
	 * If a valid HVDCP is detected, notify it to the usb_psy only
	 * if USB is still present.
	 */
	if (chip->schg_version == QPNP_SCHG_LITE)
		hvdcp_sel = SCHG_LITE_USBIN_HVDCP_SEL_BIT;
	else
		hvdcp_sel = USBIN_HVDCP_SEL_BIT;

	if ((reg & hvdcp_sel) && is_usb_present(chip))
		return true;

	return false;
}

#define FCC_CFG			0xF2
#define FCC_500MA_VAL		0x4
#define FCC_MASK		SMB_MASK(4, 0)
static int smbchg_set_fastchg_current_raw(struct smbchg_chip *chip,
		int current_ma)
{
	int i, rc;
	u8 cur_val;

	/* the fcc enumerations are the same as the usb currents */
	i = find_smaller_in_array(chip->tables.usb_ilim_ma_table,
			current_ma, chip->tables.usb_ilim_ma_len);
	if (i < 0) {
		dev_err(chip->dev,
				"Cannot find %dma current_table using %d\n",
				current_ma, CURRENT_500_MA);

		rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
				FCC_MASK,
				FCC_500MA_VAL);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set %dmA rc=%d\n",
					CURRENT_500_MA, rc);
		else
			chip->fastchg_current_ma = 500;
		return rc;
	}

	if (chip->tables.usb_ilim_ma_table[i] == chip->fastchg_current_ma) {
		pr_smb(PR_STATUS, "skipping fastchg current request: %d\n",
				chip->fastchg_current_ma);
		return 0;
	}

	cur_val = i & FCC_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CFG,
			FCC_MASK, cur_val);
	if (rc < 0) {
		dev_err(chip->dev, "cannot write to fcc cfg rc = %d\n", rc);
		return rc;
	}
	pr_err("fastcharge current requested %d, set to %d\n",
			current_ma, chip->tables.usb_ilim_ma_table[cur_val]);

	chip->fastchg_current_ma = chip->tables.usb_ilim_ma_table[cur_val];
	return rc;
}

#define ICL_STS_1_REG			0x7
#define ICL_STS_2_REG			0x9
#define ICL_STS_MASK			0x1F
#define AICL_SUSP_BIT			BIT(6)
#define AICL_STS_BIT			BIT(5)
#define USBIN_SUSPEND_STS_BIT		BIT(3)
#define USBIN_ACTIVE_PWR_SRC_BIT	BIT(1)
#define DCIN_ACTIVE_PWR_SRC_BIT		BIT(0)
#define PARALLEL_REENABLE_TIMER_MS	1000
#define PARALLEL_CHG_THRESHOLD_CURRENT	1800
static bool smbchg_is_usbin_active_pwr_src(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_2_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 2: %d\n", rc);
		return false;
	}

	return !(reg & USBIN_SUSPEND_STS_BIT)
		&& (reg & USBIN_ACTIVE_PWR_SRC_BIT);
}

static int smbchg_parallel_usb_charging_en(struct smbchg_chip *chip, bool en)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };

	if (!parallel_psy || !chip->parallel_charger_detected)
		return 0;

	pval.intval = en;
	return parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &pval);
}

#define ESR_PULSE_CURRENT_DELTA_MA	200
static int smbchg_sw_esr_pulse_en(struct smbchg_chip *chip, bool en)
{
	int rc, fg_current_now, icl_ma;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_CURRENT_NOW,
			&fg_current_now);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	icl_ma = max(chip->iterm_ma + ESR_PULSE_CURRENT_DELTA_MA,
			fg_current_now - ESR_PULSE_CURRENT_DELTA_MA);
	rc = vote(chip->fcc_votable, ESR_PULSE_FCC_VOTER, en, icl_ma);
	if (rc < 0) {
		pr_err("Couldn't Vote FCC en = %d rc = %d\n", en, rc);
		return rc;
	}
	rc = smbchg_parallel_usb_charging_en(chip, !en);
	return rc;
}

#define USB_AICL_CFG				0xF3
#define AICL_EN_BIT				BIT(2)
static void smbchg_rerun_aicl(struct smbchg_chip *chip)
{
	pr_err("Rerunning AICL...\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	/* Add a delay so that AICL successfully clears */
	msleep(50);
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
}

static void taper_irq_en(struct smbchg_chip *chip, bool en)
{
	mutex_lock(&chip->taper_irq_lock);
	if (en != chip->taper_irq_enabled) {
		if (en) {
			enable_irq(chip->taper_irq);
			enable_irq_wake(chip->taper_irq);
		} else {
			disable_irq_wake(chip->taper_irq);
			disable_irq_nosync(chip->taper_irq);
		}
		chip->taper_irq_enabled = en;
	}
	mutex_unlock(&chip->taper_irq_lock);
}

static int smbchg_get_aicl_level_ma(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Could not read usb icl sts 1: %d\n", rc);
		return 0;
	}
	if (reg & AICL_SUSP_BIT) {
		pr_warn("AICL suspended: %02x\n", reg);
		return 0;
	}
	reg &= ICL_STS_MASK;
	if (reg >= chip->tables.usb_ilim_ma_len) {
		pr_warn("invalid AICL value: %02x\n", reg);
		return 0;
	}
	return chip->tables.usb_ilim_ma_table[reg];
}

static void smbchg_parallel_usb_disable(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;
	pr_smb(PR_STATUS, "disabling parallel charger\n");
	chip->parallel.last_disabled = ktime_get_boottime();
	taper_irq_en(chip, false);
	chip->parallel.initial_aicl_ma = 0;
	chip->parallel.current_max_ma = 0;
	power_supply_set_current_limit(parallel_psy,
			SUSPEND_CURRENT_MA * 1000);
	power_supply_set_present(parallel_psy, false);
	smbchg_set_fastchg_current_raw(chip,
			get_effective_result_locked(chip->fcc_votable));
	smbchg_set_usb_current_max(chip,
			get_effective_result_locked(chip->usb_icl_votable));
	smbchg_rerun_aicl(chip);
}

#define PARALLEL_TAPER_MAX_TRIES		3
#define PARALLEL_FCC_PERCENT_REDUCTION		75
#define MINIMUM_PARALLEL_FCC_MA			500
#define CHG_ERROR_BIT		BIT(0)
#define BAT_TAPER_MODE_BIT	BIT(6)
static void smbchg_parallel_usb_taper(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int parallel_fcc_ma, tries = 0;
	u8 reg = 0;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_TAPER);
try_again:
	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	tries += 1;
	parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_STATUS, "try #%d parallel charger fcc = %d\n",
			tries, parallel_fcc_ma);
	if (parallel_fcc_ma < MINIMUM_PARALLEL_FCC_MA
			|| tries > PARALLEL_TAPER_MAX_TRIES) {
		smbchg_parallel_usb_disable(chip);
		goto done;
	}
	pval.intval = ((parallel_fcc_ma
				* PARALLEL_FCC_PERCENT_REDUCTION) / 100);
	pr_smb(PR_STATUS, "reducing FCC of parallel charger to %d\n",
			pval.intval);
	/* Change it to uA */
	pval.intval *= 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	/*
	 * sleep here for 100 ms in order to make sure the charger has a chance
	 * to go back into constant current charging
	 */
	mutex_unlock(&chip->parallel.lock);
	msleep(100);

	mutex_lock(&chip->parallel.lock);
	if (chip->parallel.current_max_ma == 0) {
		pr_smb(PR_STATUS, "Not parallel charging, skipping\n");
		goto done;
	}
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (reg & BAT_TAPER_MODE_BIT) {
		mutex_unlock(&chip->parallel.lock);
		goto try_again;
	}
	taper_irq_en(chip, true);
done:
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_TAPER);
}

static void smbchg_parallel_usb_enable(struct smbchg_chip *chip,
		int total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int new_parallel_cl_ma, set_parallel_cl_ma, new_pmi_cl_ma, rc;
	int current_table_index, target_icl_ma;
	int fcc_ma, main_fastchg_current_ma;
	int target_parallel_fcc_ma, supplied_parallel_fcc_ma;
	int parallel_chg_fcc_percent;

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	pr_smb(PR_STATUS, "Attempting to enable parallel charger\n");

	rc = power_supply_set_voltage_limit(parallel_psy, chip->vfloat_mv + 50);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't set Vflt on parallel psy rc: %d\n", rc);
		return;
	}
	/* Set USB ICL */
	target_icl_ma = get_effective_result_locked(chip->usb_icl_votable);
	new_parallel_cl_ma = total_current_ma
			* (100 - smbchg_main_chg_icl_percent) / 100;
	taper_irq_en(chip, true);
	power_supply_set_present(parallel_psy, true);
	power_supply_set_current_limit(parallel_psy,
			new_parallel_cl_ma * 1000);
	/* read back the real amount of current we are getting */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	set_parallel_cl_ma = pval.intval / 1000;
	chip->parallel.current_max_ma = new_parallel_cl_ma;
	pr_smb(PR_MISC, "Requested ICL = %d from parallel, got %d\n",
		new_parallel_cl_ma, set_parallel_cl_ma);
	new_pmi_cl_ma = max(0, target_icl_ma - set_parallel_cl_ma);
	pr_smb(PR_STATUS, "New Total USB current = %d[%d, %d]\n",
			total_current_ma, new_pmi_cl_ma,
			set_parallel_cl_ma);
	smbchg_set_usb_current_max(chip, new_pmi_cl_ma);

	/* begin splitting the fast charge current */
	fcc_ma = get_effective_result_locked(chip->fcc_votable);
	parallel_chg_fcc_percent =
		100 - smbchg_main_chg_fcc_percent;
	target_parallel_fcc_ma =
		(fcc_ma * parallel_chg_fcc_percent) / 100;
	pval.intval = target_parallel_fcc_ma * 1000;
	parallel_psy->set_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	/* check how much actual current is supplied by the parallel charger */
	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX, &pval);
	supplied_parallel_fcc_ma = pval.intval / 1000;
	pr_smb(PR_MISC, "Requested FCC = %d from parallel, got %d\n",
		target_parallel_fcc_ma, supplied_parallel_fcc_ma);

	/* then for the main charger, use the left over FCC */
	current_table_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			fcc_ma - supplied_parallel_fcc_ma,
			chip->tables.usb_ilim_ma_len);
	main_fastchg_current_ma =
		chip->tables.usb_ilim_ma_table[current_table_index];
	smbchg_set_fastchg_current_raw(chip, main_fastchg_current_ma);
	pr_smb(PR_STATUS, "FCC = %d[%d, %d]\n", fcc_ma, main_fastchg_current_ma,
					supplied_parallel_fcc_ma);

	chip->parallel.enabled_once = true;

	return;
}

static bool smbchg_is_parallel_usb_ok(struct smbchg_chip *chip,
		int *ret_total_current_ma)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	union power_supply_propval pval = {0, };
	int min_current_thr_ma, rc, type;
	int total_current_ma, current_limit_ma, parallel_cl_ma;
	ktime_t kt_since_last_disable;
	u8 reg;
	int fcc_ma = get_effective_result_locked(chip->fcc_votable);
	int fcc_voter_id = get_effective_client_id_locked(chip->fcc_votable);
	int usb_icl_ma = get_effective_result_locked(chip->usb_icl_votable);

	if (!parallel_psy || !smbchg_parallel_en
			|| !chip->parallel_charger_detected) {
		pr_smb(PR_STATUS, "Parallel charging not enabled\n");
		return false;
	}

	kt_since_last_disable = ktime_sub(ktime_get_boottime(),
			chip->parallel.last_disabled);
	if (chip->parallel.current_max_ma == 0
			&& chip->parallel.enabled_once
			&& ktime_to_ms(kt_since_last_disable)
			< PARALLEL_REENABLE_TIMER_MS) {
		pr_smb(PR_STATUS, "Only been %lld since disable, skipping\n",
				ktime_to_ms(kt_since_last_disable));
		return false;
	}

	/*
	 * If the battery is not present, try not to change parallel charging
	 * from OFF to ON or from ON to OFF, as it could cause the device to
	 * brown out in the instant that the USB settings are changed.
	 *
	 * Only allow parallel charging check to report false (thereby turnin
	 * off parallel charging) if the battery is still there, or if parallel
	 * charging is disabled in the first place.
	 */
	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST
			&& (get_prop_batt_present(chip)
				|| chip->parallel.current_max_ma == 0)) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return false;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		return false;
	}

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return false;
	}

	type = get_type(reg);
	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB_CDP) {
		pr_smb(PR_STATUS, "CDP adapter, skipping\n");
		return false;
	}

	if (get_usb_supply_type(type) == POWER_SUPPLY_TYPE_USB) {
		pr_smb(PR_STATUS, "SDP adapter, skipping\n");
		return false;
	}

	/*
	 * If USBIN is suspended or not the active power source, do not enable
	 * parallel charging. The device may be charging off of DCIN.
	 */
	if (!smbchg_is_usbin_active_pwr_src(chip)) {
		pr_smb(PR_STATUS, "USB not active power source: %02x\n", reg);
		return false;
	}

	min_current_thr_ma = smbchg_get_min_parallel_current_ma(chip);
	if (min_current_thr_ma <= 0) {
		pr_smb(PR_STATUS, "parallel charger unavailable for thr: %d\n",
				min_current_thr_ma);
		return false;
	}

	if (usb_icl_ma < min_current_thr_ma) {
		pr_smb(PR_STATUS, "Weak USB chg skip enable: %d < %d\n",
				usb_icl_ma, min_current_thr_ma);
		return false;
	}

	/*
	 * Suspend the parallel charger if the charging current is < 1800 mA
	 * and is not because of an ESR pulse.
	 */
	if (fcc_voter_id != ESR_PULSE_FCC_VOTER
			&& fcc_ma < PARALLEL_CHG_THRESHOLD_CURRENT) {
		pr_smb(PR_STATUS, "FCC %d lower than %d\n",
				fcc_ma,
				PARALLEL_CHG_THRESHOLD_CURRENT);
		return false;
	}

	current_limit_ma = smbchg_get_aicl_level_ma(chip);
	if (current_limit_ma <= 0)
		return false;

	if (chip->parallel.initial_aicl_ma == 0) {
		if (current_limit_ma < min_current_thr_ma) {
			pr_smb(PR_STATUS, "Initial AICL very low: %d < %d\n",
					current_limit_ma, min_current_thr_ma);
			return false;
		}
		chip->parallel.initial_aicl_ma = current_limit_ma;
	}

	parallel_psy->get_property(parallel_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &pval);
	parallel_cl_ma = pval.intval / 1000;
	/*
	 * Read back the real amount of current we are getting
	 * Treat 2mA as 0 because that is the suspend current setting
	 */
	if (parallel_cl_ma <= SUSPEND_CURRENT_MA)
		parallel_cl_ma = 0;

	/*
	 * Set the parallel charge path's input current limit (ICL)
	 * to the total current / 2
	 */
	total_current_ma = min(current_limit_ma + parallel_cl_ma, usb_icl_ma);

	if (total_current_ma < chip->parallel.initial_aicl_ma
			- chip->parallel.allowed_lowering_ma) {
		pr_smb(PR_STATUS,
				"Total current reduced a lot: %d (%d + %d) < %d - %d\n",
				total_current_ma,
				current_limit_ma, parallel_cl_ma,
				chip->parallel.initial_aicl_ma,
				chip->parallel.allowed_lowering_ma);
		return false;
	}

	*ret_total_current_ma = total_current_ma;
	return true;
}

#define PARALLEL_CHARGER_EN_DELAY_MS	500
static void smbchg_parallel_usb_en_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
			struct smbchg_chip,
			parallel_en_work.work);
	int previous_aicl_ma, total_current_ma, aicl_ma;
	bool in_progress;

	/* do a check to see if the aicl is stable */
	previous_aicl_ma = smbchg_get_aicl_level_ma(chip);
	msleep(PARALLEL_CHARGER_EN_DELAY_MS);
	aicl_ma = smbchg_get_aicl_level_ma(chip);
	if (previous_aicl_ma == aicl_ma) {
		pr_smb(PR_STATUS, "AICL at %d\n", aicl_ma);
	} else {
		pr_smb(PR_STATUS,
				"AICL changed [%d -> %d], recheck %d ms\n",
				previous_aicl_ma, aicl_ma,
				PARALLEL_CHARGER_EN_DELAY_MS);
		goto recheck;
	}

	mutex_lock(&chip->parallel.lock);
	in_progress = (chip->parallel.current_max_ma != 0);
	if (smbchg_is_parallel_usb_ok(chip, &total_current_ma)) {
		smbchg_parallel_usb_enable(chip, total_current_ma);
	} else {
		if (in_progress) {
			pr_smb(PR_STATUS, "parallel charging unavailable\n");
			smbchg_parallel_usb_disable(chip);
		}
	}
	mutex_unlock(&chip->parallel.lock);
	smbchg_relax(chip, PM_PARALLEL_CHECK);
	return;

recheck:
	schedule_delayed_work(&chip->parallel_en_work, 0);
}

static void smbchg_parallel_usb_check_ok(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);

	if (!parallel_psy || !chip->parallel_charger_detected)
		return;

	smbchg_stay_awake(chip, PM_PARALLEL_CHECK);
	schedule_delayed_work(&chip->parallel_en_work, 0);
}

static int charging_suspend_vote_cb(struct device *dev, int suspend,
		int client, int last_suspend,
		int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_charging_en(chip, !suspend);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't configure batt chg: 0x%x rc = %d\n",
				!suspend, rc);
	}

	return rc;
}

static int usb_suspend_vote_cb(struct device *dev, int suspend,
		int client, int last_suspend,
		int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_usb_suspend(chip, suspend);
	if (rc < 0)
		return rc;

	if (client == THERMAL_EN_VOTER || client == POWER_SUPPLY_EN_VOTER ||
				client == USER_EN_VOTER ||
				client == FAKE_BATTERY_EN_VOTER)
		smbchg_parallel_usb_check_ok(chip);

	return rc;
}

static int dc_suspend_vote_cb(struct device *dev, int suspend,
		int client, int last_suspend,
		int last_client)
{
	int rc;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_dc_suspend(chip, suspend);
	if (rc < 0)
		return rc;

	if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
		power_supply_changed(&chip->dc_psy);

	return rc;
}

static int set_fastchg_current_vote_cb(struct device *dev,
		int fcc_ma,
		int client,
		int last_fcc_ma,
		int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);
	int rc;

	if (chip->parallel.current_max_ma == 0) {
		rc = smbchg_set_fastchg_current_raw(chip, fcc_ma);
		if (rc < 0) {
			pr_err("Can't set FCC fcc_ma=%d rc=%d\n", fcc_ma, rc);
			return rc;
		}
	}
	/*
	 * check if parallel charging can be enabled, and if enabled,
	 * distribute the fcc
	 */
	smbchg_parallel_usb_check_ok(chip);
	return 0;
}

static int smbchg_set_fastchg_current_user(struct smbchg_chip *chip,
		int current_ma)
{
	int rc = 0;

	pr_smb(PR_STATUS, "User setting FCC to %d\n", current_ma);

	rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true, current_ma);
	if (rc < 0)
		pr_err("Couldn't vote en rc %d\n", rc);
	return rc;
}

static struct ilim_entry *smbchg_wipower_find_entry(struct smbchg_chip *chip,
		struct ilim_map *map, int uv)
{
	int i;
	struct ilim_entry *ret = &(chip->wipower_default.entries[0]);

	for (i = 0; i < map->num; i++) {
		if (is_between(map->entries[i].vmin_uv, map->entries[i].vmax_uv,
					uv))
			ret = &map->entries[i];
	}
	return ret;
}

#define ZIN_ICL_PT	0xFC
#define ZIN_ICL_LV	0xFD
#define ZIN_ICL_HV	0xFE
#define ZIN_ICL_MASK	SMB_MASK(4, 0)
static int smbchg_dcin_ilim_config(struct smbchg_chip *chip, int offset, int ma)
{
	int i, rc;

	i = find_smaller_in_array(chip->tables.dc_ilim_ma_table,
			ma, chip->tables.dc_ilim_ma_len);

	if (i < 0)
		i = 0;

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + offset,
			ZIN_ICL_MASK, i);
	if (rc)
		dev_err(chip->dev, "Couldn't write bat if offset %d value = %d rc = %d\n",
				offset, i, rc);
	return rc;
}

static int smbchg_wipower_ilim_config(struct smbchg_chip *chip,
		struct ilim_entry *ilim)
{
	int rc = 0;

	if (chip->current_ilim.icl_pt_ma != ilim->icl_pt_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_PT, ilim->icl_pt_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_PT, ilim->icl_pt_ma, rc);
		else
			chip->current_ilim.icl_pt_ma =  ilim->icl_pt_ma;
	}

	if (chip->current_ilim.icl_lv_ma !=  ilim->icl_lv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_LV, ilim->icl_lv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_LV, ilim->icl_lv_ma, rc);
		else
			chip->current_ilim.icl_lv_ma =  ilim->icl_lv_ma;
	}

	if (chip->current_ilim.icl_hv_ma !=  ilim->icl_hv_ma) {
		rc = smbchg_dcin_ilim_config(chip, ZIN_ICL_HV, ilim->icl_hv_ma);
		if (rc)
			dev_err(chip->dev, "failed to write batif offset %d %dma rc = %d\n",
					ZIN_ICL_HV, ilim->icl_hv_ma, rc);
		else
			chip->current_ilim.icl_hv_ma =  ilim->icl_hv_ma;
	}
	return rc;
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx);
static int smbchg_wipower_dcin_btm_configure(struct smbchg_chip *chip,
		struct ilim_entry *ilim)
{
	int rc;

	if (ilim->vmin_uv == chip->current_ilim.vmin_uv
			&& ilim->vmax_uv == chip->current_ilim.vmax_uv)
		return 0;

	chip->param.channel = DCIN;
	chip->param.btm_ctx = chip;
	if (wipower_dcin_interval < ADC_MEAS1_INTERVAL_0MS)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_0MS;

	if (wipower_dcin_interval > ADC_MEAS1_INTERVAL_16S)
		wipower_dcin_interval = ADC_MEAS1_INTERVAL_16S;

	chip->param.timer_interval = wipower_dcin_interval;
	chip->param.threshold_notification = &btm_notify_dcin;
	chip->param.high_thr = ilim->vmax_uv + wipower_dcin_hyst_uv;
	chip->param.low_thr = ilim->vmin_uv - wipower_dcin_hyst_uv;
	chip->param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	rc = qpnp_vadc_channel_monitor(chip->vadc_dev, &chip->param);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure btm for dcin rc = %d\n",
				rc);
	} else {
		chip->current_ilim.vmin_uv = ilim->vmin_uv;
		chip->current_ilim.vmax_uv = ilim->vmax_uv;
		pr_smb(PR_STATUS, "btm ilim = (%duV %duV %dmA %dmA %dmA)\n",
				ilim->vmin_uv, ilim->vmax_uv,
				ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
	}
	return rc;
}

static int smbchg_wipower_icl_configure(struct smbchg_chip *chip,
		int dcin_uv, bool div2)
{
	int rc = 0;
	struct ilim_map *map = div2 ? &chip->wipower_div2 : &chip->wipower_pt;
	struct ilim_entry *ilim = smbchg_wipower_find_entry(chip, map, dcin_uv);

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config ilim rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
				rc, dcin_uv, div2,
				ilim->vmin_uv, ilim->vmax_uv,
				ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}

	rc = smbchg_wipower_dcin_btm_configure(chip, ilim);
	if (rc) {
		dev_err(chip->dev, "failed to config btm rc = %d, dcin_uv = %d , div2 = %d, ilim = (%duV %duV %dmA %dmA %dmA)\n",
				rc, dcin_uv, div2,
				ilim->vmin_uv, ilim->vmax_uv,
				ilim->icl_pt_ma, ilim->icl_lv_ma, ilim->icl_hv_ma);
		return rc;
	}
	chip->wipower_configured = true;
	return 0;
}

static void smbchg_wipower_icl_deconfigure(struct smbchg_chip *chip)
{
	int rc;
	struct ilim_entry *ilim = &(chip->wipower_default.entries[0]);

	if (!chip->wipower_configured)
		return;

	rc = smbchg_wipower_ilim_config(chip, ilim);
	if (rc)
		dev_err(chip->dev, "Couldn't config default ilim rc = %d\n",
				rc);

	rc = qpnp_vadc_end_channel_monitor(chip->vadc_dev);
	if (rc)
		dev_err(chip->dev, "Couldn't de configure btm for dcin rc = %d\n",
				rc);

	chip->wipower_configured = false;
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	chip->current_ilim.icl_pt_ma = ilim->icl_pt_ma;
	chip->current_ilim.icl_lv_ma = ilim->icl_lv_ma;
	chip->current_ilim.icl_hv_ma = ilim->icl_hv_ma;
	pr_smb(PR_WIPOWER, "De config btm\n");
}

#define FV_STS		0x0C
#define DIV2_ACTIVE	BIT(7)
static void __smbchg_wipower_check(struct smbchg_chip *chip)
{
	int chg_type;
	bool usb_present, dc_present;
	int rc;
	int dcin_uv;
	bool div2;
	struct qpnp_vadc_result adc_result;
	u8 reg;

	if (!wipower_dyn_icl_en) {
		smbchg_wipower_icl_deconfigure(chip);
		return;
	}

	chg_type = get_prop_charge_type(chip);
	usb_present = is_usb_present(chip);
	dc_present = is_dc_present(chip);
	if (chg_type != POWER_SUPPLY_CHARGE_TYPE_NONE
			&& !usb_present
			&& dc_present
			&& chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER) {
		rc = qpnp_vadc_read(chip->vadc_dev, DCIN, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		dcin_uv = adc_result.physical;

		/* check div_by_2 */
		rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS, 1);
		if (rc) {
			pr_smb(PR_STATUS, "error DCIN read rc = %d\n", rc);
			return;
		}
		div2 = !!(reg & DIV2_ACTIVE);

		pr_smb(PR_WIPOWER,
				"config ICL chg_type = %d usb = %d dc = %d dcin_uv(adc_code) = %d (0x%x) div2 = %d\n",
				chg_type, usb_present, dc_present, dcin_uv,
				adc_result.adc_code, div2);
		smbchg_wipower_icl_configure(chip, dcin_uv, div2);
	} else {
		pr_smb(PR_WIPOWER,
				"deconfig ICL chg_type = %d usb = %d dc = %d\n",
				chg_type, usb_present, dc_present);
		smbchg_wipower_icl_deconfigure(chip);
	}
}

static void smbchg_wipower_check(struct smbchg_chip *chip)
{
	if (!chip->wipower_dyn_icl_avail)
		return;

	mutex_lock(&chip->wipower_config);
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static void btm_notify_dcin(enum qpnp_tm_state state, void *ctx)
{
	struct smbchg_chip *chip = ctx;

	mutex_lock(&chip->wipower_config);
	pr_smb(PR_WIPOWER, "%s state\n",
			state  == ADC_TM_LOW_STATE ? "low" : "high");
	chip->current_ilim.vmin_uv = 0;
	chip->current_ilim.vmax_uv = 0;
	__smbchg_wipower_check(chip);
	mutex_unlock(&chip->wipower_config);
}

static int force_dcin_icl_write(void *data, u64 val)
{
	struct smbchg_chip *chip = data;

	smbchg_wipower_check(chip);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_dcin_icl_ops, NULL,
		force_dcin_icl_write, "0x%02llx\n");

/*
 * set the dc charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_dc_current_limit_vote_cb(struct device *dev,
		int icl_ma,
		int client,
		int last_icl_ma,
		int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	return smbchg_set_dc_current_max(chip, icl_ma);
}

/*
 * set the usb charge path's maximum allowed current draw
 * that may be limited by the system's thermal level
 */
static int set_usb_current_limit_vote_cb(struct device *dev,
		int icl_ma,
		int client,
		int last_icl_ma,
		int last_client)
{
	struct smbchg_chip *chip = dev_get_drvdata(dev);
	int rc, aicl_ma, effective_id;

	effective_id = get_effective_client_id_locked(chip->usb_icl_votable);

	/* disable parallel charging if HVDCP is voting for 300mA */
	if (effective_id == HVDCP_ICL_VOTER)
		smbchg_parallel_usb_disable(chip);

	if (chip->parallel.current_max_ma == 0) {
		rc = smbchg_set_usb_current_max(chip, icl_ma);
		if (rc) {
			pr_err("Failed to set usb current max: %d\n", rc);
			return rc;
		}
	}

	/* skip the aicl rerun if hvdcp icl voter is active */
	if (effective_id == HVDCP_ICL_VOTER)
		return 0;

	aicl_ma = smbchg_get_aicl_level_ma(chip);
	if (icl_ma > aicl_ma)
		smbchg_rerun_aicl(chip);
	smbchg_parallel_usb_check_ok(chip);
	return 0;
}

static int smbchg_system_temp_level_set(struct smbchg_chip *chip,
		int lvl_sel)
{
	int rc = 0;
	int prev_therm_lvl;
	int thermal_icl_ma;

	if (!chip->thermal_mitigation) {
		dev_err(chip->dev, "Thermal mitigation not supported\n");
		return -EINVAL;
	}

	if (lvl_sel < 0) {
		dev_err(chip->dev, "Unsupported level selected %d\n", lvl_sel);
		return -EINVAL;
	}

	if (lvl_sel >= chip->thermal_levels) {
		dev_err(chip->dev, "Unsupported level selected %d forcing %d\n",
				lvl_sel, chip->thermal_levels - 1);
		lvl_sel = chip->thermal_levels - 1;
	}

	if (lvl_sel == chip->therm_lvl_sel)
		return 0;

	mutex_lock(&chip->therm_lvl_lock);
	prev_therm_lvl = chip->therm_lvl_sel;
	chip->therm_lvl_sel = lvl_sel;
	if (chip->therm_lvl_sel == 0) {
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable USB thermal ICL vote rc=%d\n",
					rc);

		rc = vote(chip->dc_icl_votable, THERMAL_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't disable DC thermal ICL vote rc=%d\n",
					rc);
	} else {
		thermal_icl_ma =
			(int)chip->thermal_mitigation[chip->therm_lvl_sel];
		rc = vote(chip->usb_icl_votable, THERMAL_ICL_VOTER, true,
				thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for USB thermal ICL rc=%d\n", rc);

		rc = vote(chip->dc_icl_votable, THERMAL_ICL_VOTER, true,
				thermal_icl_ma);
		if (rc < 0)
			pr_err("Couldn't vote for DC thermal ICL rc=%d\n", rc);
	}

	if (prev_therm_lvl == chip->thermal_levels - 1) {
		/*
		 * If previously highest value was selected charging must have
		 * been disabed. Enable charging by taking the DC and USB path
		 * out of suspend.
		 */
		rc = vote(chip->dc_suspend_votable, THERMAL_EN_VOTER, false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't set dc suspend rc %d\n", rc);
			goto out;
		}
		rc = vote(chip->usb_suspend_votable, THERMAL_EN_VOTER,
				false, 0);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't set usb suspend rc %d\n", rc);
			goto out;
		}
	}
out:
	mutex_unlock(&chip->therm_lvl_lock);
	return rc;
}

static int smbchg_ibat_ocp_threshold_ua = 4500000;
module_param(smbchg_ibat_ocp_threshold_ua, int, 0644);

#define UCONV			1000000LL
#define MCONV			1000LL
#define FLASH_V_THRESHOLD	3000000
#define FLASH_VDIP_MARGIN	100000
#define VPH_FLASH_VDIP		(FLASH_V_THRESHOLD + FLASH_VDIP_MARGIN)
#define BUCK_EFFICIENCY		800LL
static int smbchg_calc_max_flash_current(struct smbchg_chip *chip)
{
	int ocv_uv, esr_uohm, rbatt_uohm, ibat_now, rc;
	int64_t ibat_flash_ua, avail_flash_ua, avail_flash_power_fw;
	int64_t ibat_safe_ua, vin_flash_uv, vph_flash_uv;

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_VOLTAGE_OCV, &ocv_uv);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support OCV\n");
		return 0;
	}

	rc = get_property_from_fg(chip, POWER_SUPPLY_PROP_RESISTANCE,
			&esr_uohm);
	if (rc) {
		pr_smb(PR_STATUS, "bms psy does not support resistance\n");
		return 0;
	}

	rc = msm_bcl_read(BCL_PARAM_CURRENT, &ibat_now);
	if (rc) {
		pr_smb(PR_STATUS, "BCL current read failed: %d\n", rc);
		return 0;
	}

	rbatt_uohm = esr_uohm + chip->rpara_uohm + chip->rslow_uohm;
	/*
	 * Calculate the maximum current that can pulled out of the battery
	 * before the battery voltage dips below a safe threshold.
	 */
	ibat_safe_ua = div_s64((ocv_uv - VPH_FLASH_VDIP) * UCONV,
			rbatt_uohm);

	if (ibat_safe_ua <= smbchg_ibat_ocp_threshold_ua) {
		/*
		 * If the calculated current is below the OCP threshold, then
		 * use it as the possible flash current.
		 */
		ibat_flash_ua = ibat_safe_ua - ibat_now;
		vph_flash_uv = VPH_FLASH_VDIP;
	} else {
		/*
		 * If the calculated current is above the OCP threshold, then
		 * use the ocp threshold instead.
		 *
		 * Any higher current will be tripping the battery OCP.
		 */
		ibat_flash_ua = smbchg_ibat_ocp_threshold_ua - ibat_now;
		vph_flash_uv = ocv_uv - div64_s64((int64_t)rbatt_uohm
				* smbchg_ibat_ocp_threshold_ua, UCONV);
	}
	/* Calculate the input voltage of the flash module. */
	vin_flash_uv = max((chip->vled_max_uv + 500000LL),
			div64_s64((vph_flash_uv * 1200), 1000));
	/* Calculate the available power for the flash module. */
	avail_flash_power_fw = BUCK_EFFICIENCY * vph_flash_uv * ibat_flash_ua;
	/*
	 * Calculate the available amount of current the flash module can draw
	 * before collapsing the battery. (available power/ flash input voltage)
	 */
	avail_flash_ua = div64_s64(avail_flash_power_fw, vin_flash_uv * MCONV);
	pr_smb(PR_MISC,
			"avail_iflash=%lld, ocv=%d, ibat=%d, rbatt=%d\n",
			avail_flash_ua, ocv_uv, ibat_now, rbatt_uohm);
	return (int)avail_flash_ua;
}

#define FCC_CMP_CFG	0xF3
#define FCC_COMP_MASK	SMB_MASK(1, 0)
static int smbchg_fastchg_current_comp_set(struct smbchg_chip *chip,
		int comp_current)
{
	int rc;
	u8 i;

	for (i = 0; i < chip->tables.fcc_comp_len; i++)
		if (comp_current == chip->tables.fcc_comp_table[i])
			break;

	if (i >= chip->tables.fcc_comp_len)
		return -EINVAL;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FCC_CMP_CFG,
			FCC_COMP_MASK, i);

	if (rc)
		dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
				rc);

	return rc;
}

#define CFG_TCC_REG			0xF9
#define CHG_ITERM_MASK			SMB_MASK(2, 0)
static int smbchg_iterm_set(struct smbchg_chip *chip, int iterm_ma)
{
	int rc;
	u8 reg;

	reg = find_closest_in_array(
			chip->tables.iterm_ma_table,
			chip->tables.iterm_ma_len,
			iterm_ma);

	rc = smbchg_sec_masked_write(chip,
			chip->chgr_base + CFG_TCC_REG,
			CHG_ITERM_MASK, reg);
	if (rc) {
		dev_err(chip->dev,
				"Couldn't set iterm rc = %d\n", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "set tcc (%d) to 0x%02x\n",
			iterm_ma, reg);
	chip->iterm_ma = iterm_ma;

	return 0;
}

#define FV_CMP_CFG	0xF5
#define FV_COMP_MASK	SMB_MASK(5, 0)
static int smbchg_float_voltage_comp_set(struct smbchg_chip *chip, int code)
{
	int rc;
	u8 val;

	val = code & FV_COMP_MASK;
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + FV_CMP_CFG,
			FV_COMP_MASK, val);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
				rc);

	return rc;
}

#define VFLOAT_CFG_REG			0xF4
#define MIN_FLOAT_MV			3600
#define MAX_FLOAT_MV			4500
#define VFLOAT_MASK			SMB_MASK(5, 0)

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4360
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2C
#define VHIGH_RANGE_FLOAT_STEP_MV	20
static int smbchg_float_voltage_set(struct smbchg_chip *chip, int vfloat_mv)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc, delta;
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
				vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		/* mid range */
		delta = vfloat_mv - MID_RANGE_FLOAT_MV_MIN;
		temp = MID_RANGE_FLOAT_MIN_VAL + delta
			/ MID_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % MID_RANGE_FLOAT_STEP_MV;
	} else if (vfloat_mv <= VHIGH_RANGE_FLOAT_MIN_MV) {
		/* high range */
		delta = vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV;
		temp = HIGH_RANGE_FLOAT_MIN_VAL + delta
			/ HIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % HIGH_RANGE_FLOAT_STEP_MV;
	} else {
		/* very high range */
		delta = vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV;
		temp = VHIGH_RANGE_FLOAT_MIN_VAL + delta
			/ VHIGH_RANGE_FLOAT_STEP_MV;
		vfloat_mv -= delta % VHIGH_RANGE_FLOAT_STEP_MV;
	}

	if (parallel_psy) {
		rc = power_supply_set_voltage_limit(parallel_psy,
				vfloat_mv + 50);
		if (rc)
			dev_err(chip->dev, "Couldn't set float voltage on parallel psy rc: %d\n",
					rc);
	}

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + VFLOAT_CFG_REG,
			VFLOAT_MASK, temp);

	if (rc)
		dev_err(chip->dev, "Couldn't set float voltage rc = %d\n", rc);
	else
		chip->vfloat_mv = vfloat_mv;

	return rc;
}

static int smbchg_float_voltage_get(struct smbchg_chip *chip)
{
	return chip->vfloat_mv;
}

#define SFT_CFG				0xFD
#define SFT_EN_MASK			SMB_MASK(5, 4)
#define SFT_TO_MASK			SMB_MASK(3, 2)
#define PRECHG_SFT_TO_MASK		SMB_MASK(1, 0)
#define SFT_TIMER_DISABLE_BIT		BIT(5)
#define PRECHG_SFT_TIMER_DISABLE_BIT	BIT(4)
#define SAFETY_TIME_MINUTES_SHIFT	2
static int smbchg_safety_timer_enable(struct smbchg_chip *chip, bool enable)
{
	int rc;
	u8 reg;

	if (enable == chip->safety_timer_en)
		return 0;

	if (enable)
		reg = 0;
	else
		reg = SFT_TIMER_DISABLE_BIT | PRECHG_SFT_TIMER_DISABLE_BIT;

	rc = smbchg_sec_masked_write(chip, chip->chgr_base + SFT_CFG,
			SFT_EN_MASK, reg);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't %s safety timer rc = %d\n",
				enable ? "enable" : "disable", rc);
		return rc;
	}
	chip->safety_timer_en = enable;
	return 0;
}

enum skip_reason {
	REASON_OTG_ENABLED	= BIT(0),
	REASON_FLASH_ENABLED	= BIT(1)
};

#define BAT_IF_TRIM7_REG	0xF7
#define CFG_750KHZ_BIT		BIT(1)
#define MISC_CFG_NTC_VOUT_REG	0xF3
#define CFG_NTC_VOUT_FSW_BIT	BIT(0)
static int smbchg_switch_buck_frequency(struct smbchg_chip *chip,
				bool flash_active)
{
	int rc;

	if (!(chip->wa_flags & SMBCHG_FLASH_BUCK_SWITCH_FREQ_WA))
		return 0;

	if (chip->flash_active == flash_active) {
		pr_smb(PR_STATUS, "Fsw not changed, flash_active: %d\n",
			flash_active);
		return 0;
	}

	/*
	 * As per the systems team recommendation, before the flash fires,
	 * buck switching frequency(Fsw) needs to be increased to 1MHz. Once the
	 * flash is disabled, Fsw needs to be set back to 750KHz.
	 */
	rc = smbchg_sec_masked_write(chip, chip->misc_base +
				MISC_CFG_NTC_VOUT_REG, CFG_NTC_VOUT_FSW_BIT,
				flash_active ? CFG_NTC_VOUT_FSW_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set switching frequency multiplier rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + BAT_IF_TRIM7_REG,
			CFG_750KHZ_BIT, flash_active ? 0 : CFG_750KHZ_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Fsw @ %sHz\n", flash_active ? "1M" : "750K");
	chip->flash_active = flash_active;
	return 0;
}

#define OTG_TRIM6		0xF6
#define TR_ENB_SKIP_BIT		BIT(2)
#define OTG_EN_BIT		BIT(0)
static int smbchg_otg_pulse_skip_disable(struct smbchg_chip *chip,
		enum skip_reason reason, bool disable)
{
	int rc;
	bool disabled;

	disabled = !!chip->otg_pulse_skip_dis;
	pr_smb(PR_STATUS, "%s pulse skip, reason %d\n",
			disable ? "disabling" : "enabling", reason);
	if (disable)
		chip->otg_pulse_skip_dis |= reason;
	else
		chip->otg_pulse_skip_dis &= ~reason;
	if (disabled == !!chip->otg_pulse_skip_dis)
		return 0;
	disabled = !!chip->otg_pulse_skip_dis;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_TRIM6,
			TR_ENB_SKIP_BIT, disabled ? TR_ENB_SKIP_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't %s otg pulse skip rc = %d\n",
				disabled ? "disable" : "enable", rc);
		return rc;
	}
	pr_smb(PR_STATUS, "%s pulse skip\n", disabled ? "disabled" : "enabled");
	return 0;
}

#define LOW_PWR_OPTIONS_REG	0xFF
#define FORCE_TLIM_BIT		BIT(4)
static int smbchg_force_tlim_en(struct smbchg_chip *chip, bool enable)
{
	int rc;

	rc = smbchg_sec_masked_write(chip, chip->otg_base + LOW_PWR_OPTIONS_REG,
			FORCE_TLIM_BIT, enable ? FORCE_TLIM_BIT : 0);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't %s otg force tlim rc = %d\n",
				enable ? "enable" : "disable", rc);
		return rc;
	}
	return rc;
}

static void smbchg_vfloat_adjust_check(struct smbchg_chip *chip)
{
	if (!chip->use_vfloat_adjustments)
		return;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	pr_smb(PR_STATUS, "Starting vfloat adjustments\n");
	schedule_delayed_work(&chip->vfloat_adjust_work, 0);
}

#define FV_STS_REG			0xC
#define AICL_INPUT_STS_BIT		BIT(6)
static bool smbchg_is_input_current_limited(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->chgr_base + FV_STS_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read FV_STS rc=%d\n", rc);
		return false;
	}

	return !!(reg & AICL_INPUT_STS_BIT);
}

#define SW_ESR_PULSE_MS			1500
static void smbchg_cc_esr_wa_check(struct smbchg_chip *chip)
{
	int rc, esr_count;

	if (!(chip->wa_flags & SMBCHG_CC_ESR_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "No inputs present, skipping\n");
		return;
	}

	if (get_prop_charge_type(chip) != POWER_SUPPLY_CHARGE_TYPE_FAST) {
		pr_smb(PR_STATUS, "Not in fast charge, skipping\n");
		return;
	}

	if (!smbchg_is_input_current_limited(chip)) {
		pr_smb(PR_STATUS, "Not input current limited, skipping\n");
		return;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_ESR_COUNT, &esr_count);
	if (rc) {
		pr_smb(PR_STATUS,
				"could not read ESR counter rc = %d\n", rc);
		return;
	}

	/*
	 * The esr_count is counting down the number of fuel gauge cycles
	 * before a ESR pulse is needed.
	 *
	 * After a successful ESR pulse, this count is reset to some
	 * high number like 28. If this reaches 0, then the fuel gauge
	 * hardware should force a ESR pulse.
	 *
	 * However, if the device is in constant current charge mode while
	 * being input current limited, the ESR pulse will not affect the
	 * battery current, so the measurement will fail.
	 *
	 * As a failsafe, force a manual ESR pulse if this value is read as
	 * 0.
	 */
	if (esr_count != 0) {
		pr_smb(PR_STATUS, "ESR count is not zero, skipping\n");
		return;
	}

	pr_smb(PR_STATUS, "Lowering charge current for ESR pulse\n");
	smbchg_stay_awake(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, true);
	msleep(SW_ESR_PULSE_MS);
	pr_smb(PR_STATUS, "Raising charge current for ESR pulse\n");
	smbchg_relax(chip, PM_ESR_PULSE);
	smbchg_sw_esr_pulse_en(chip, false);
}

static void smbchg_soc_changed(struct smbchg_chip *chip)
{
	return;//do nothing
	smbchg_cc_esr_wa_check(chip);
}

#define DC_AICL_CFG			0xF3
#define MISC_TRIM_OPT_15_8		0xF5
#define USB_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define USB_AICL_DEGLITCH_LONG		0
#define DC_AICL_DEGLITCH_MASK		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_SHORT		(BIT(5) | BIT(4) | BIT(3))
#define DC_AICL_DEGLITCH_LONG		0
#define AICL_RERUN_MASK			(BIT(5) | BIT(4))
#define AICL_RERUN_ON			(BIT(5) | BIT(4))
#define AICL_RERUN_OFF			0

static int smbchg_hw_aicl_rerun_enable_indirect_cb(struct device *dev,
						int enable,
						int client, int last_enable,
						int last_client)
{
	int rc = 0;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	/*
	 * If the indirect voting result of all the clients is to enable hw aicl
	 * rerun, then remove our vote to disable hw aicl rerun
	 */
	rc = vote(chip->hw_aicl_rerun_disable_votable,
		HW_AICL_RERUN_ENABLE_INDIRECT_VOTER, !enable, 0);
	if (rc < 0) {
		pr_err("Couldn't vote for hw rerun rc= %d\n", rc);
		return rc;
	}

	return rc;
}

static int smbchg_hw_aicl_rerun_disable_cb(struct device *dev, int disable,
						int client, int last_disable,
						int last_client)
{
	int rc = 0;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_sec_masked_write(chip,
		chip->misc_base + MISC_TRIM_OPT_15_8,
		AICL_RERUN_MASK, disable ? AICL_RERUN_OFF : AICL_RERUN_ON);
	if (rc < 0)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n", rc);

	return rc;
}

static int smbchg_aicl_deglitch_config_cb(struct device *dev, int shorter,
						int client, int last_result,
						int last_client)
{
	int rc = 0;
	struct smbchg_chip *chip = dev_get_drvdata(dev);

	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USB_AICL_CFG,
		USB_AICL_DEGLITCH_MASK,
		shorter ? USB_AICL_DEGLITCH_SHORT : USB_AICL_DEGLITCH_LONG);
	if (rc < 0) {
		pr_err("Couldn't write to USB_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	rc = smbchg_sec_masked_write(chip,
		chip->dc_chgpth_base + DC_AICL_CFG,
		DC_AICL_DEGLITCH_MASK,
		shorter ? DC_AICL_DEGLITCH_SHORT : DC_AICL_DEGLITCH_LONG);
	if (rc < 0) {
		pr_err("Couldn't write to DC_AICL_CFG rc=%d\n", rc);
		return rc;
	}
	return rc;
}

static void smbchg_aicl_deglitch_wa_en(struct smbchg_chip *chip, bool en)
{
	int rc;

	rc = vote(chip->aicl_deglitch_short_votable,
		VARB_WORKAROUND_VOTER, en, 0);
	if (rc < 0) {
		pr_err("Couldn't vote %s deglitch rc=%d\n",
				en ? "short" : "long", rc);
		return;
	}
	pr_smb(PR_STATUS, "AICL deglitch set to %s\n", en ? "short" : "long");

	rc = vote(chip->hw_aicl_rerun_enable_indirect_votable,
			VARB_WORKAROUND_VOTER, en, 0);
	if (rc < 0) {
		pr_err("Couldn't vote hw aicl rerun rc= %d\n", rc);
		return;
	}
	chip->aicl_deglitch_short = en;
}

static void smbchg_aicl_deglitch_wa_check(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	int rc;
	bool low_volt_chgr = true;

	if (!(chip->wa_flags & SMBCHG_AICL_DEGLITCH_WA))
		return;

	if (!is_usb_present(chip) && !is_dc_present(chip)) {
		pr_smb(PR_STATUS, "Charger removed\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	if (!chip->bms_psy)
		return;

	if (is_usb_present(chip)) {
		if (is_hvdcp_present(chip))
			low_volt_chgr = false;
	} else if (is_dc_present(chip)) {
		if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
			low_volt_chgr = false;
		else
			low_volt_chgr = chip->low_volt_dcin;
	}

	if (!low_volt_chgr) {
		pr_smb(PR_STATUS, "High volt charger! Don't set deglitch\n");
		smbchg_aicl_deglitch_wa_en(chip, false);
		return;
	}

	/* It is possible that battery voltage went high above threshold
	 * when the charger is inserted and can go low because of system
	 * load. We shouldn't be reconfiguring AICL deglitch when this
	 * happens as it will lead to oscillation again which is being
	 * fixed here. Do it once when the battery voltage crosses the
	 * threshold (e.g. 4.2 V) and clear it only when the charger
	 * is removed.
	 */
	if (!chip->vbat_above_headroom) {
		rc = chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MIN, &prop);
		if (rc < 0) {
			pr_err("could not read voltage_min, rc=%d\n", rc);
			return;
		}
		chip->vbat_above_headroom = !prop.intval;
	}
	smbchg_aicl_deglitch_wa_en(chip, chip->vbat_above_headroom);
}

#define MISC_TEST_REG		0xE2
#define BB_LOOP_DISABLE_ICL	BIT(2)
static int smbchg_icl_loop_disable_check(struct smbchg_chip *chip)
{
	bool icl_disabled = !chip->chg_otg_enabled && chip->flash_triggered;
	int rc = 0;

	if ((chip->wa_flags & SMBCHG_FLASH_ICL_DISABLE_WA)
			&& icl_disabled != chip->icl_disabled) {
		rc = smbchg_sec_masked_write(chip,
				chip->misc_base + MISC_TEST_REG,
				BB_LOOP_DISABLE_ICL,
				icl_disabled ? BB_LOOP_DISABLE_ICL : 0);
		chip->icl_disabled = icl_disabled;
	}

	return rc;
}

#define UNKNOWN_BATT_TYPE	"Unknown Battery"
#define LOADING_BATT_TYPE	"Loading Battery Data"
static int smbchg_config_chg_battery_type(struct smbchg_chip *chip)
{
	int rc = 0, max_voltage_uv = 0, fastchg_ma = 0, ret = 0, iterm_ua = 0;
	struct device_node *batt_node, *profile_node;
	struct device_node *node = chip->spmi->dev.of_node;
	union power_supply_propval prop = {0,};

	rc = chip->bms_psy->get_property(chip->bms_psy,
			POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
	if (rc) {
		pr_smb(PR_STATUS, "Unable to read battery-type rc=%d\n", rc);
		return 0;
	}
	if (!strcmp(prop.strval, UNKNOWN_BATT_TYPE) ||
			!strcmp(prop.strval, LOADING_BATT_TYPE)) {
		pr_smb(PR_MISC, "Battery-type not identified\n");
		return 0;
	}
	/* quit if there is no change in the battery-type from previous */
	if (chip->battery_type && !strcmp(prop.strval, chip->battery_type))
		return 0;

	batt_node = of_parse_phandle(node, "qcom,battery-data", 0);
	if (!batt_node) {
		pr_smb(PR_MISC, "No batterydata available\n");
		return 0;
	}

	profile_node = of_batterydata_get_best_profile(batt_node,
			"bms", NULL);
	if (!profile_node) {
		pr_err("couldn't find profile handle\n");
		return -EINVAL;
	}
	chip->battery_type = prop.strval;

	/* change vfloat */
	rc = of_property_read_u32(profile_node, "qcom,max-voltage-uv",
			&max_voltage_uv);
	if (rc) {
		pr_warn("couldn't find battery max voltage rc=%d\n", rc);
		ret = rc;
	} else {
		if (chip->vfloat_mv != (max_voltage_uv / 1000)) {
			pr_info("Vfloat changed from %dmV to %dmV for battery-type %s\n",
					chip->vfloat_mv, (max_voltage_uv / 1000),
					chip->battery_type);
			rc = smbchg_float_voltage_set(chip,
					(max_voltage_uv / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
						"Couldn't set float voltage rc = %d\n", rc);
				return rc;
			}
		}
	}

	/* change chg term */
	rc = of_property_read_u32(profile_node, "qcom,chg-term-ua",
			&iterm_ua);
	if (rc && rc != -EINVAL) {
		pr_warn("couldn't read battery term current=%d\n", rc);
		ret = rc;
	} else if (!rc) {
		if (chip->iterm_ma != (iterm_ua / 1000)
				&& !chip->iterm_disabled) {
			pr_info("Term current changed from %dmA to %dmA for battery-type %s\n",
					chip->iterm_ma, (iterm_ua / 1000),
					chip->battery_type);
			rc = smbchg_iterm_set(chip,
					(iterm_ua / 1000));
			if (rc < 0) {
				dev_err(chip->dev,
						"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}
		}
		chip->iterm_ma = iterm_ua / 1000;
	}

	/*
	 * Only configure from profile if fastchg-ma is not defined in the
	 * charger device node.
	 */
	if (!of_find_property(chip->spmi->dev.of_node,
				"qcom,fastchg-current-ma", NULL)) {
		rc = of_property_read_u32(profile_node,
				"qcom,fastchg-current-ma", &fastchg_ma);
		if (rc) {
			ret = rc;
		} else {
			pr_smb(PR_MISC,
					"fastchg-ma changed from to %dma for battery-type %s\n",
					fastchg_ma, chip->battery_type);
			rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,
					fastchg_ma);
			if (rc < 0) {
				dev_err(chip->dev,
						"Couldn't vote for fastchg current rc=%d\n",
						rc);
				return rc;
			}
		}
	}

	return ret;
}

#define MAX_INV_BATT_ID		7700
#define MIN_INV_BATT_ID		7300
static void check_battery_type(struct smbchg_chip *chip)
{
	union power_supply_propval prop = {0,};
	bool en;

	if (!chip->bms_psy && chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_BATTERY_TYPE, &prop);
		en = (strcmp(prop.strval, UNKNOWN_BATT_TYPE) != 0
				|| chip->charge_unknown_battery)
			&& (strcmp(prop.strval, LOADING_BATT_TYPE) != 0);
		vote(chip->battchg_suspend_votable,
				BATTCHG_UNKNOWN_BATTERY_EN_VOTER, !en, 0);

		if (!chip->skip_usb_suspend_for_fake_battery) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_RESISTANCE_ID, &prop);
			/* suspend USB path for invalid battery-id */
			en = (prop.intval <= MAX_INV_BATT_ID &&
				prop.intval >= MIN_INV_BATT_ID) ? 1 : 0;
			vote(chip->usb_suspend_votable, FAKE_BATTERY_EN_VOTER,
				en, 0);
		}
	}
}

static struct external_battery_gauge *fast_charger = NULL;
extern void mcu_en_gpio_set(int value);//
extern void set_mcu_en_gpio_value(int value);
extern void usb_sw_gpio_set(int value);//

static void switch_mode_to_normal(void)
{
	usb_sw_gpio_set(0);
	mcu_en_gpio_set(1);
}

void fastcharge_information_register(struct external_battery_gauge *fast_chg)
{
	if (fast_charger) {
		fast_charger = fast_chg;
		pr_err("qpnp-charger %s multiple battery gauge called\n",
				__func__);
	} else {
		fast_charger = fast_chg;
	}
}
EXPORT_SYMBOL(fastcharge_information_register);

void fastcharge_information_unregister(struct external_battery_gauge *fast_chg)
{
	fast_charger = NULL;
}
EXPORT_SYMBOL(fastcharge_information_unregister);

static int
get_prop_charger_voltage_now(struct smbchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	if(use_fake_chgvol)
		return fake_chgvol;

	if (chip->revision == 0 ) {
		pr_err("vchg reading not supported for 1.0 rc=%d\n", rc);
		return 0;
	} else {
		rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
		if (rc) {
			pr_err("Unable to read vchg rc=%d\n", rc);
			return 0;
		}
		return (int)results.physical/1000;
	}
}
static int
get_prop_fast_chg_started(struct smbchg_chip *chip)
{
	if (fast_charger && fast_charger->fast_chg_started)
	{
	return fast_charger->fast_chg_started();
	}
	else {
		pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

static int
qpnp_get_fast_low_temp_full(struct smbchg_chip *chip)
{
	if (fast_charger && fast_charger->get_fast_low_temp_full)
		return fast_charger->get_fast_low_temp_full();
	else {
		pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

static int
qpnp_get_fast_chg_allow(struct smbchg_chip *chip)
{
	if (fast_charger && fast_charger->get_fast_chg_allow)
		return fast_charger->get_fast_chg_allow();
	else {
		pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

static int
get_prop_fast_switch_to_normal(struct smbchg_chip *chip)
{
	if (fast_charger && fast_charger->fast_switch_to_normal)
		return fast_charger->fast_switch_to_normal();
	else {
		pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}
static int
set_prop_fast_switch_to_normal_false(struct smbchg_chip *chip)
{
	if (fast_charger && fast_charger->set_switch_to_noraml_false)
		return fast_charger->set_switch_to_noraml_false();
	else {
		pr_err("qpnp-charger no batt gauge assuming,set_fast_switch_to_normal_false false\n");
		return false;
	}
}
static int get_fastchg_firmware_updated_status(struct smbchg_chip *chip)
{
	if(fast_charger && fast_charger->get_fastchg_firmware_already_updated)
		return fast_charger->get_fastchg_firmware_already_updated();
	else
	{
	pr_err("qpnp-charger no batt gauge assuming false,get_fastchg_firmware_updated_status false\n");
	return false;
	}
}

bool is_alow_fast_chg(struct smbchg_chip *chip)
{
	int temp = 0;
	int cap = 0;
	static int pre_temp=0;
	bool low_temp_full = 0;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
	temp = get_prop_batt_temp(chip);
	cap = get_prop_batt_capacity(chip);
	low_temp_full = qpnp_get_fast_low_temp_full(chip);
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if(get_fastchg_firmware_updated_status(chip)==false)
	{
	return false;
	}
	if(temp < 165 || temp > 430 )
	{
		if(temp != pre_temp)
		{
		pr_err("temp is not right, not allow swith fastchg\n");
		}
		pre_temp =temp;
	return false;
	}
	if(get_prop_fast_switch_to_normal(chip) == true){
		return false;
	}
	return true;
}

static int qpnp_set_fast_chg_allow(struct smbchg_chip *chip,int enable)
{
	if (fast_charger && fast_charger->set_fast_chg_allow)
		return fast_charger->set_fast_chg_allow(enable);
	else {
		pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

static bool usb_switched=false;
static void switch_fast_chg(struct smbchg_chip *chip)
{
	if(gpio_get_value(135))//usb-sw-gpio
		return;
	if(!is_usb_present(chip))
		return;
	if(qpnp_get_fast_chg_allow(chip) == false){
		if(is_alow_fast_chg(chip) == true) {
			//swtich on fast chg
			set_mcu_en_gpio_value(1);
			msleep(10);
			usb_sw_gpio_set(1);
			msleep(10);
			mcu_en_gpio_set(0);
			usb_switched =true;
			qpnp_set_fast_chg_allow(chip,true);
		}

	}
}
static int qpnp_get_fast_chg_ing(struct smbchg_chip *chip)
{
	if (fast_charger && fast_charger->get_fast_chg_ing)
		return fast_charger->get_fast_chg_ing();
	else {
		pr_err("qpnp-charger no batt gauge assuming false\n");
		return false;
	}
}

static void dump_regs(struct smbchg_chip *chip);

static void smbchg_external_power_changed(struct power_supply *psy)
{
	struct smbchg_chip *chip = container_of(psy,
			struct smbchg_chip, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0, soc;
	static int pre_current_limit_value=0;
	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->bms_psy) {
		soc = get_prop_batt_capacity(chip);
		if (chip->previous_soc != soc) {
			chip->previous_soc = soc;
			smbchg_soc_changed(chip);
		}

		rc = smbchg_config_chg_battery_type(chip);
		if (rc)
			pr_smb(PR_MISC,
					"Couldn't update charger configuration rc=%d\n",
					rc);
	}

	rc = chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &prop);
	if (rc == 0)
		vote(chip->usb_suspend_votable, POWER_SUPPLY_EN_VOTER,
				!prop.intval, 0);

	rc = chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc == 0)
		current_limit = prop.intval / 1000;
	dev_err(chip->dev,"current_limit = %d\n", current_limit);
	if (prop.intval <= 2  && get_prop_batt_present(chip)) {

		pre_current_limit_value=0;	//charger removed,set pre_current_limit_value 0
		chip->charger_status = CHARGER_STATUS_GOOD;
		chip->aicl_current = 0;
	}
	if((current_limit != 0)&&(current_limit != pre_current_limit_value))
	{
		//if current_limmit is the same with before value ,do not check	qpnp_check_battery_temp
		chip->is_power_changed =true;
		qpnp_check_battery_temp(chip);
		//use oem temp control logic
		pr_info("changed chip->usb_target_current_ma = %d\n",
				current_limit);
		rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit);
		if (rc < 0)
			pr_err("Couldn't update USB PSY ICL vote rc=%d\n", rc);
		pre_current_limit_value = current_limit;
	}


	smbchg_vfloat_adjust_check(chip);

	power_supply_changed(&chip->batt_psy);
}

static int smbchg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	chip->otg_retries = 0;
	chip->chg_otg_enabled = true;
	smbchg_icl_loop_disable_check(chip);
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, true);

	/* If pin control mode then return from here */
	if (chip->otg_pinctrl)
		return rc;

	/* sleep to make sure the pulse skip is actually disabled */
	msleep(20);
	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't enable OTG mode rc=%d\n", rc);
	else
		chip->otg_enable_time = ktime_get();
	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	if (!chip->otg_pinctrl) {
		rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
				OTG_EN_BIT, 0);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n",
					rc);
	}

	chip->chg_otg_enabled = false;
	smbchg_otg_pulse_skip_disable(chip, REASON_OTG_ENABLED, false);
	smbchg_icl_loop_disable_check(chip);
	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = smbchg_read(chip, &reg, chip->bat_if_base + CMD_CHG_REG, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return (reg & OTG_EN_BIT) ? 1 : 0;
}

struct regulator_ops smbchg_otg_reg_ops = {
	.enable		= smbchg_otg_regulator_enable,
	.disable	= smbchg_otg_regulator_disable,
	.is_enabled	= smbchg_otg_regulator_is_enable,
};

#define USBIN_CHGR_CFG			0xF1
#define ADAPTER_ALLOWANCE_MASK		0x7
#define USBIN_ADAPTER_9V		0x3
#define USBIN_ADAPTER_5V_9V_CONT	0x2
#define USBIN_ADAPTER_5V_UNREGULATED_9V	0x5
#define HVDCP_EN_BIT			BIT(3)
static int smbchg_external_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->usb_suspend_votable, OTG_EN_VOTER, true, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't suspend charger rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);
		return rc;
	}

	/*
	 * To disallow source detect and usbin_uv interrupts, set the adapter
	 * allowance to 9V, so that the audio boost operating in reverse never
	 * gets detected as a valid input
	 */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			0xFF, USBIN_ADAPTER_9V);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Enabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	rc = vote(chip->usb_suspend_votable, OTG_EN_VOTER, false, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend charger rc=%d\n", rc);
		return rc;
	}

	/*
	 * Reenable HVDCP and set the adapter allowance back to the original
	 * value in order to allow normal USBs to be recognized as a valid
	 * input.
	 */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			0xFF, chip->original_usbin_allowance);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_STATUS, "Disabling OTG Boost\n");
	return rc;
}

static int smbchg_external_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	struct smbchg_chip *chip = rdev_get_drvdata(rdev);

	return get_client_vote(chip->usb_suspend_votable, OTG_EN_VOTER);
}

struct regulator_ops smbchg_external_otg_reg_ops = {
	.enable		= smbchg_external_otg_regulator_enable,
	.disable	= smbchg_external_otg_regulator_disable,
	.is_enabled	= smbchg_external_otg_regulator_is_enable,
};

static int smbchg_regulator_init(struct smbchg_chip *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};
	struct device_node *regulator_node;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-boost-otg");

	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smbchg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->otg_vreg.rdev = devm_regulator_register(chip->dev,
				&chip->otg_vreg.rdesc, &cfg);
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
						"OTG reg failed, rc=%d\n", rc);
		}
	}

	if (rc)
		return rc;

	regulator_node = of_get_child_by_name(chip->dev->of_node,
			"qcom,smbcharger-external-otg");
	if (!regulator_node) {
		dev_dbg(chip->dev, "external-otg node absent\n");
		return 0;
	}
	init_data = of_get_regulator_init_data(chip->dev, regulator_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		if (of_get_property(chip->dev->of_node,
					"otg-parent-supply", NULL))
			init_data->supply_regulator = "otg-parent";
		chip->ext_otg_vreg.rdesc.owner = THIS_MODULE;
		chip->ext_otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->ext_otg_vreg.rdesc.ops = &smbchg_external_otg_reg_ops;
		chip->ext_otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = regulator_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		chip->ext_otg_vreg.rdev = devm_regulator_register(chip->dev,
				&chip->ext_otg_vreg.rdesc,
				&cfg);
		if (IS_ERR(chip->ext_otg_vreg.rdev)) {
			rc = PTR_ERR(chip->ext_otg_vreg.rdev);
			chip->ext_otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
						"external OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

#define CMD_CHG_LED_REG		0x43
#define CHG_LED_CTRL_BIT		BIT(0)
#define LED_SW_CTRL_BIT		0x1
#define LED_CHG_CTRL_BIT		0x0
#define CHG_LED_ON		0x03
#define CHG_LED_OFF		0x00
#define LED_BLINKING_PATTERN1		0x01
#define LED_BLINKING_PATTERN2		0x02
#define LED_BLINKING_CFG_MASK		SMB_MASK(2, 1)
#define CHG_LED_SHIFT		1
static int smbchg_chg_led_controls(struct smbchg_chip *chip)
{
	u8 reg, mask;
	int rc;

	if (chip->cfg_chg_led_sw_ctrl) {
		/* turn-off LED by default for software control */
		mask = CHG_LED_CTRL_BIT | LED_BLINKING_CFG_MASK;
		reg = LED_SW_CTRL_BIT;
	} else {
		mask = CHG_LED_CTRL_BIT;
		reg = LED_CHG_CTRL_BIT;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_LED_REG,
			mask, reg);
	if (rc < 0)
		dev_err(chip->dev,
				"Couldn't write LED_CTRL_BIT rc=%d\n", rc);
	return rc;
}

static void smbchg_chg_led_brightness_set(struct led_classdev *cdev,
		enum led_brightness value)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg;
	int rc;

	reg = (value > LED_OFF) ? CHG_LED_ON << CHG_LED_SHIFT :
		CHG_LED_OFF << CHG_LED_SHIFT;

	if (value > LED_OFF)
		power_supply_set_hi_power_state(chip->bms_psy, 1);
	else
		power_supply_set_hi_power_state(chip->bms_psy, 0);

	pr_smb(PR_STATUS,
			"set the charger led brightness to value=%d\n",
			value);
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

	static enum
led_brightness smbchg_chg_led_brightness_get(struct led_classdev *cdev)
{
	struct smbchg_chip *chip = container_of(cdev,
			struct smbchg_chip, led_cdev);
	u8 reg_val, chg_led_sts;
	int rc;

	rc = smbchg_read(chip, &reg_val, chip->bat_if_base + CMD_CHG_LED_REG,
			1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read CHG_LED_REG sts rc=%d\n",
				rc);
		return rc;
	}

	chg_led_sts = (reg_val & LED_BLINKING_CFG_MASK) >> CHG_LED_SHIFT;

	pr_smb(PR_STATUS, "chg_led_sts = %02x\n", chg_led_sts);

	return (chg_led_sts == CHG_LED_OFF) ? LED_OFF : LED_FULL;
}

static void smbchg_chg_led_blink_set(struct smbchg_chip *chip,
		unsigned long blinking)
{
	u8 reg;
	int rc;

	if (blinking == 0) {
		reg = CHG_LED_OFF << CHG_LED_SHIFT;
		power_supply_set_hi_power_state(chip->bms_psy, 0);
	} else {
		power_supply_set_hi_power_state(chip->bms_psy, 1);
		if (blinking == 1)
			reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
		else if (blinking == 2)
			reg = LED_BLINKING_PATTERN2 << CHG_LED_SHIFT;
		else
			reg = LED_BLINKING_PATTERN1 << CHG_LED_SHIFT;
	}

	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + CMD_CHG_LED_REG,
			LED_BLINKING_CFG_MASK, reg);
	if (rc)
		dev_err(chip->dev, "Couldn't write CHG_LED rc=%d\n",
				rc);
}

static ssize_t smbchg_chg_led_blink_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct smbchg_chip *chip = container_of(cdev, struct smbchg_chip,
			led_cdev);
	unsigned long blinking;
	ssize_t rc = -EINVAL;

	rc = kstrtoul(buf, 10, &blinking);
	if (rc)
		return rc;

	smbchg_chg_led_blink_set(chip, blinking);

	return len;
}

static DEVICE_ATTR(blink, 0664, NULL, smbchg_chg_led_blink_store);

static struct attribute *led_blink_attributes[] = {
	&dev_attr_blink.attr,
	NULL,
};

static struct attribute_group smbchg_led_attr_group = {
	.attrs = led_blink_attributes
};

static int smbchg_register_chg_led(struct smbchg_chip *chip)
{
	int rc;

	chip->led_cdev.name = "red";
	chip->led_cdev.brightness_set = smbchg_chg_led_brightness_set;
	chip->led_cdev.brightness_get = smbchg_chg_led_brightness_get;

	rc = led_classdev_register(chip->dev, &chip->led_cdev);
	if (rc) {
		dev_err(chip->dev, "unable to register charger led, rc=%d\n",
				rc);
		return rc;
	}

	rc = sysfs_create_group(&chip->led_cdev.dev->kobj,
			&smbchg_led_attr_group);
	if (rc) {
		dev_err(chip->dev, "led sysfs rc: %d\n", rc);
		return rc;
	}

	return rc;
}

static int vf_adjust_low_threshold = 5;
module_param(vf_adjust_low_threshold, int, 0644);

static int vf_adjust_high_threshold = 7;
module_param(vf_adjust_high_threshold, int, 0644);

static int vf_adjust_n_samples = 10;
module_param(vf_adjust_n_samples, int, 0644);

static int vf_adjust_max_delta_mv = 40;
module_param(vf_adjust_max_delta_mv, int, 0644);

static int vf_adjust_trim_steps_per_adjust = 1;
module_param(vf_adjust_trim_steps_per_adjust, int, 0644);

#define CENTER_TRIM_CODE		7
#define MAX_LIN_CODE			14
#define MAX_TRIM_CODE			15
#define SCALE_SHIFT			4
#define VF_TRIM_OFFSET_MASK		SMB_MASK(3, 0)
#define VF_STEP_SIZE_MV			10
#define SCALE_LSB_MV			17
static int smbchg_trim_add_steps(int prev_trim, int delta_steps)
{
	int scale_steps;
	int linear_offset, linear_scale;
	int offset_code = prev_trim & VF_TRIM_OFFSET_MASK;
	int scale_code = (prev_trim & ~VF_TRIM_OFFSET_MASK) >> SCALE_SHIFT;

	if (abs(delta_steps) > 1) {
		pr_smb(PR_STATUS,
				"Cant trim multiple steps delta_steps = %d\n",
				delta_steps);
		return prev_trim;
	}
	if (offset_code <= CENTER_TRIM_CODE)
		linear_offset = offset_code + CENTER_TRIM_CODE;
	else if (offset_code > CENTER_TRIM_CODE)
		linear_offset = MAX_TRIM_CODE - offset_code;

	if (scale_code <= CENTER_TRIM_CODE)
		linear_scale = scale_code + CENTER_TRIM_CODE;
	else if (scale_code > CENTER_TRIM_CODE)
		linear_scale = scale_code - (CENTER_TRIM_CODE + 1);

	/* check if we can accomodate delta steps with just the offset */
	if (linear_offset + delta_steps >= 0
			&& linear_offset + delta_steps <= MAX_LIN_CODE) {
		linear_offset += delta_steps;

		if (linear_offset > CENTER_TRIM_CODE)
			offset_code = linear_offset - CENTER_TRIM_CODE;
		else
			offset_code = MAX_TRIM_CODE - linear_offset;

		return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
	}

	/* changing offset cannot satisfy delta steps, change the scale bits */
	scale_steps = delta_steps > 0 ? 1 : -1;

	if (linear_scale + scale_steps < 0
			|| linear_scale + scale_steps > MAX_LIN_CODE) {
		pr_smb(PR_STATUS,
				"Cant trim scale_steps = %d delta_steps = %d\n",
				scale_steps, delta_steps);
		return prev_trim;
	}

	linear_scale += scale_steps;

	if (linear_scale > CENTER_TRIM_CODE)
		scale_code = linear_scale - CENTER_TRIM_CODE;
	else
		scale_code = linear_scale + (CENTER_TRIM_CODE + 1);
	prev_trim = (prev_trim & VF_TRIM_OFFSET_MASK)
		| scale_code << SCALE_SHIFT;

	/*
	 * now that we have changed scale which is a 17mV jump, change the
	 * offset bits (10mV) too so the effective change is just 7mV
	 */
	delta_steps = -1 * delta_steps;

	linear_offset = clamp(linear_offset + delta_steps, 0, MAX_LIN_CODE);
	if (linear_offset > CENTER_TRIM_CODE)
		offset_code = linear_offset - CENTER_TRIM_CODE;
	else
		offset_code = MAX_TRIM_CODE - linear_offset;

	return (prev_trim & ~VF_TRIM_OFFSET_MASK) | offset_code;
}

#define TRIM_14		0xFE
#define VF_TRIM_MASK	0xFF
static int smbchg_adjust_vfloat_mv_trim(struct smbchg_chip *chip,
		int delta_mv)
{
	int sign, delta_steps, rc = 0;
	u8 prev_trim, new_trim;
	int i;

	sign = delta_mv > 0 ? 1 : -1;
	delta_steps = (delta_mv + sign * VF_STEP_SIZE_MV / 2)
		/ VF_STEP_SIZE_MV;

	rc = smbchg_read(chip, &prev_trim, chip->misc_base + TRIM_14, 1);
	if (rc) {
		dev_err(chip->dev, "Unable to read trim 14: %d\n", rc);
		return rc;
	}

	for (i = 1; i <= abs(delta_steps)
			&& i <= vf_adjust_trim_steps_per_adjust; i++) {
		new_trim = (u8)smbchg_trim_add_steps(prev_trim,
				delta_steps > 0 ? 1 : -1);
		if (new_trim == prev_trim) {
			pr_smb(PR_STATUS,
					"VFloat trim unchanged from %02x\n", prev_trim);
			/* treat no trim change as an error */
			return -EINVAL;
		}

		rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_14,
				VF_TRIM_MASK, new_trim);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't change vfloat trim rc=%d\n", rc);
		}
		pr_smb(PR_STATUS,
				"VFlt trim %02x to %02x, delta steps: %d\n",
				prev_trim, new_trim, delta_steps);
		prev_trim = new_trim;
	}

	return rc;
}

#define VFLOAT_RESAMPLE_DELAY_MS	10000
static void smbchg_vfloat_adjust_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
			struct smbchg_chip,
			vfloat_adjust_work.work);
	int vbat_uv, vbat_mv, ibat_ua, rc, delta_vfloat_mv;
	bool taper, enable;

	smbchg_stay_awake(chip, PM_REASON_VFLOAT_ADJUST);
	taper = (get_prop_charge_type(chip)
			== POWER_SUPPLY_CHARGE_TYPE_TAPER);
	enable = taper && (chip->parallel.current_max_ma == 0);

	if (!enable) {
		pr_smb(PR_MISC,
				"Stopping vfloat adj taper=%d parallel_ma = %d\n",
				taper, chip->parallel.current_max_ma);
		goto stop;
	}

	if (get_prop_batt_health(chip) != POWER_SUPPLY_HEALTH_GOOD) {
		pr_smb(PR_STATUS, "JEITA active, skipping\n");
		goto stop;
	}

	set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_NOW, 1);
	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &vbat_uv);
	if (rc) {
		pr_smb(PR_STATUS,
				"bms psy does not support voltage rc = %d\n", rc);
		goto stop;
	}
	vbat_mv = vbat_uv / 1000;

	if ((vbat_mv - chip->vfloat_mv) < -1 * vf_adjust_max_delta_mv) {
		pr_smb(PR_STATUS, "Skip vbat out of range: %d\n", vbat_mv);
		goto reschedule;
	}

	rc = get_property_from_fg(chip,
			POWER_SUPPLY_PROP_CURRENT_NOW, &ibat_ua);
	if (rc) {
		pr_smb(PR_STATUS,
				"bms psy does not support current_now rc = %d\n", rc);
		goto stop;
	}

	if (ibat_ua / 1000 > -chip->iterm_ma) {
		pr_smb(PR_STATUS, "Skip ibat too high: %d\n", ibat_ua);
		goto reschedule;
	}

	pr_smb(PR_STATUS, "sample number = %d vbat_mv = %d ibat_ua = %d\n",
			chip->n_vbat_samples,
			vbat_mv,
			ibat_ua);

	chip->max_vbat_sample = max(chip->max_vbat_sample, vbat_mv);
	chip->n_vbat_samples += 1;
	if (chip->n_vbat_samples < vf_adjust_n_samples) {
		pr_smb(PR_STATUS, "Skip %d samples; max = %d\n",
				chip->n_vbat_samples, chip->max_vbat_sample);
		goto reschedule;
	}
	/* if max vbat > target vfloat, delta_vfloat_mv could be negative */
	delta_vfloat_mv = chip->vfloat_mv - chip->max_vbat_sample;
	pr_smb(PR_STATUS, "delta_vfloat_mv = %d, samples = %d, mvbat = %d\n",
			delta_vfloat_mv, chip->n_vbat_samples, chip->max_vbat_sample);
	/*
	 * enough valid samples has been collected, adjust trim codes
	 * based on maximum of collected vbat samples if necessary
	 */
	if (delta_vfloat_mv > vf_adjust_high_threshold
			|| delta_vfloat_mv < -1 * vf_adjust_low_threshold) {
		rc = smbchg_adjust_vfloat_mv_trim(chip, delta_vfloat_mv);
		if (rc) {
			pr_smb(PR_STATUS,
					"Stopping vfloat adj after trim adj rc = %d\n",
					rc);
			goto stop;
		}
		chip->max_vbat_sample = 0;
		chip->n_vbat_samples = 0;
		goto reschedule;
	}

stop:
	chip->max_vbat_sample = 0;
	chip->n_vbat_samples = 0;
	smbchg_relax(chip, PM_REASON_VFLOAT_ADJUST);
	return;

reschedule:
	schedule_delayed_work(&chip->vfloat_adjust_work,
			msecs_to_jiffies(VFLOAT_RESAMPLE_DELAY_MS));
	return;
}

static int smbchg_charging_status_change(struct smbchg_chip *chip)
{
	smbchg_vfloat_adjust_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
	return 0;
}

#define BB_CLMP_SEL		0xF8
#define BB_CLMP_MASK		SMB_MASK(1, 0)
#define BB_CLMP_VFIX_3338MV	0x1
#define BB_CLMP_VFIX_3512MV	0x2
static int smbchg_set_optimal_charging_mode(struct smbchg_chip *chip, int type)
{
	int rc;
	bool hvdcp2 = (type == POWER_SUPPLY_TYPE_USB_HVDCP
			&& smbchg_is_usbin_active_pwr_src(chip));

	/*
	 * Set the charger switching freq to 1MHZ if HVDCP 2.0,
	 * or 750KHZ otherwise
	 */
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BAT_IF_TRIM7_REG,
			CFG_750KHZ_BIT, hvdcp2 ? 0 : CFG_750KHZ_BIT);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	/*
	 * Set the charger switch frequency clamp voltage threshold to 3.338V
	 * if HVDCP 2.0, or 3.512V otherwise.
	 */
	rc = smbchg_sec_masked_write(chip, chip->bat_if_base + BB_CLMP_SEL,
			BB_CLMP_MASK,
			hvdcp2 ? BB_CLMP_VFIX_3338MV : BB_CLMP_VFIX_3512MV);
	if (rc) {
		dev_err(chip->dev, "Cannot set switching freq: %d\n", rc);
		return rc;
	}

	return 0;
}

#define DEFAULT_SDP_MA		500
#define DEFAULT_CDP_MA		1500
static int smbchg_change_usb_supply_type(struct smbchg_chip *chip,
						enum power_supply_type type)
{
	int rc, current_limit_ma;

	/*
	 * if the type is not unknown, set the type before changing ICL vote
	 * in order to ensure that the correct current limit registers are
	 * used
	 */
	if (type != POWER_SUPPLY_TYPE_UNKNOWN)
		chip->usb_supply_type = type;

	/*
	 * Type-C only supports STD(900), MEDIUM(1500) and HIGH(3000) current
	 * modes, skip all BC 1.2 current if external typec is supported.
	 * Note: for SDP supporting current based on USB notifications.
	 */
	if (chip->typec_psy && (type != POWER_SUPPLY_TYPE_USB))
		current_limit_ma = chip->typec_current_ma;
	else if (type == POWER_SUPPLY_TYPE_USB)
		current_limit_ma = DEFAULT_SDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB)
		current_limit_ma = DEFAULT_SDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_CDP)
		current_limit_ma = DEFAULT_CDP_MA;
	else if (type == POWER_SUPPLY_TYPE_USB_HVDCP)
		current_limit_ma = smbchg_default_hvdcp_icl_ma;
	else if (type == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		current_limit_ma = smbchg_default_hvdcp3_icl_ma;
	else
		current_limit_ma = smbchg_default_dcp_icl_ma;

	pr_smb(PR_STATUS, "Type %d: setting mA = %d\n",
		type, current_limit_ma);
	rc = vote(chip->usb_icl_votable, PSY_ICL_VOTER, true,
				current_limit_ma);
	if (rc < 0) {
		pr_err("Couldn't vote for new USB ICL rc=%d\n", rc);
		goto out;
	}

	if (!chip->skip_usb_notification)
		power_supply_set_supply_type(chip->usb_psy, type);

	/* otherwise if it is unknown, set type after the vote */
	if (type == POWER_SUPPLY_TYPE_UNKNOWN)
		chip->usb_supply_type = type;

	/* set the correct buck switching frequency */
	rc = smbchg_set_optimal_charging_mode(chip, type);
	if (rc < 0)
		pr_err("Couldn't set charger optimal mode rc=%d\n", rc);

out:
	return rc;
}

#define HVDCP_ADAPTER_SEL_MASK	SMB_MASK(5, 4)
#define HVDCP_5V		0x00
#define HVDCP_9V		0x10
#define USB_CMD_HVDCP_1		0x42
#define FORCE_HVDCP_2p0		BIT(3)

static int force_9v_hvdcp(struct smbchg_chip *chip)
{
	int rc;

	/* Force 5V HVDCP */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc) {
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);
		return rc;
	}

	/* Force QC2.0 */
	rc = smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, FORCE_HVDCP_2p0);
	rc |= smbchg_masked_write(chip,
			chip->usb_chgpth_base + USB_CMD_HVDCP_1,
			FORCE_HVDCP_2p0, 0);
	if (rc < 0) {
		pr_err("Couldn't force QC2.0 rc=%d\n", rc);
		return rc;
	}

	/* Delay to switch into HVDCP 2.0 and avoid UV */
	msleep(500);

	/* Force 9V HVDCP */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc)
		pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n", rc);

	return rc;
}

static void smbchg_hvdcp_det_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
			struct smbchg_chip,
			hvdcp_det_work.work);
	int rc;

	if (is_hvdcp_present(chip)) {
		pr_err("HVDCP is present\n");
		chip->hvdcp_present = true;
		chip->aicl_current = DWC3_HVDCP_CHG_MAX;
		if (!chip->hvdcp3_supported &&
				(chip->wa_flags & SMBCHG_HVDCP_9V_EN_WA)) {
			/* force HVDCP 2.0 */
			rc = force_9v_hvdcp(chip);
			if (rc)
				pr_err("could not force 9V HVDCP continuing rc=%d\n",
						rc);
		}
		smbchg_change_usb_supply_type(chip,
				POWER_SUPPLY_TYPE_USB_HVDCP);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_aicl_deglitch_wa_check(chip);
	}
}

static int set_usb_psy_dp_dm(struct smbchg_chip *chip, int state)
{
	int rc;
	u8 reg;

	/*
	 * ensure that we are not in the middle of an insertion where usbin_uv
	 * is low and src_detect hasnt gone high. If so force dp=F dm=F
	 * which guarantees proper type detection
	 */
	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (!rc && !(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "overwriting state = %d with %d\n",
				state, POWER_SUPPLY_DP_DM_DPF_DMF);
		state = POWER_SUPPLY_DP_DM_DPF_DMF;
	}
	pr_smb(PR_MISC, "setting usb psy dp dm = %d\n", state);
	return power_supply_set_dp_dm(chip->usb_psy, state);
}

#define APSD_CFG		0xF5
#define AUTO_SRC_DETECT_EN_BIT	BIT(0)
#define APSD_TIMEOUT_MS		1500
static void restore_from_hvdcp_detection(struct smbchg_chip *chip)
{
	int rc;

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	/* switch to 9V HVDCP */
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0)
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);

	/* enable HVDCP */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);

	/* enable APSD */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + APSD_CFG,
			AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable APSD rc=%d\n", rc);

	/* Reset back to 5V unregulated */
	rc = smbchg_sec_masked_write(chip,
		chip->usb_chgpth_base + USBIN_CHGR_CFG,
		ADAPTER_ALLOWANCE_MASK, USBIN_ADAPTER_5V_UNREGULATED_9V);
	if (rc < 0)
		pr_err("Couldn't write usb allowance rc=%d\n", rc);

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	if (rc < 0)
		pr_err("Couldn't enable AICL rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = false;
	chip->pulse_cnt = 0;
}

#define RESTRICTED_CHG_FCC_PERCENT	50
static int smbchg_restricted_charging(struct smbchg_chip *chip, bool enable)
{
	int current_table_index, fastchg_current;
	int rc = 0;

	/* If enable, set the fcc to the set point closest
	 * to 50% of the configured fcc while remaining below it
	 */
	current_table_index = find_smaller_in_array(
			chip->tables.usb_ilim_ma_table,
			chip->cfg_fastchg_current_ma
			* RESTRICTED_CHG_FCC_PERCENT / 100,
			chip->tables.usb_ilim_ma_len);
	fastchg_current =
		chip->tables.usb_ilim_ma_table[current_table_index];
	rc = vote(chip->fcc_votable, RESTRICTED_CHG_FCC_VOTER, enable,
			fastchg_current);

	pr_smb(PR_STATUS, "restricted_charging set to %d\n", enable);
	chip->restricted_charging = enable;

	return rc;
}

static void handle_usb_removal(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	int rc;

	pr_smb(PR_STATUS, "triggered\n");
	pr_err("%s triggered\n",__func__);
if(chip->re_detect_charger_type_status==false)
{
	qpnp_set_fast_chg_allow(chip,false);
	set_prop_fast_switch_to_normal_false(chip);
	pr_err("%s switch off fastchg\n", __func__);
	usb_sw_gpio_set(0);
	mcu_en_gpio_set(1);
	chip->chg_done=false;// usb unplug,set chg_done false
	chip->time_out = false;// usb unplug agin ,set charge time_out false
	chip->recharge_status=false;
	chip->usb_enum_status=false;
	chip->nonstandard_charger = false;
	qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__INVALID);//charger removed ,set temp reginon invalid
	wake_unlock(&chip->chg_wake_lock);
}
	smbchg_aicl_deglitch_wa_check(chip);
	/* Clear the OV detected status set before */
	if (chip->usb_ov_det)
		chip->usb_ov_det = false;
	/* Clear typec current status */
	if (chip->typec_psy)
		chip->typec_current_ma = 0;
	smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_UNKNOWN);
	if (!chip->skip_usb_notification) {
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}
	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPR_DMR);
	if(!chip->skip_usb_notification)
	{
	schedule_work(&chip->usb_set_online_work);
	}
	pr_smb(PR_MISC, "setting usb psy health UNKNOWN\n");
	rc = power_supply_set_health_state(chip->usb_psy,
			POWER_SUPPLY_HEALTH_UNKNOWN);
	if (rc < 0)
		pr_smb(PR_STATUS,
				"usb psy does not allow updating prop %d rc = %d\n",
				POWER_SUPPLY_HEALTH_UNKNOWN, rc);

	if (parallel_psy && chip->parallel_charger_detected)
		power_supply_set_present(parallel_psy, false);
	if (chip->parallel.avail && chip->aicl_done_irq
			&& chip->enable_aicl_wake) {
		disable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = false;
	}
	chip->parallel.enabled_once = false;
	chip->vbat_above_headroom = false;
	vote(chip->usb_icl_votable, WEAK_CHARGER_ICL_VOTER, false, 0);
	chip->usb_icl_delta = 0;
	vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, false, 0);
	vote(chip->aicl_deglitch_short_votable,
		HVDCP_SHORT_DEGLITCH_VOTER, false, 0);
	if (!chip->hvdcp_not_supported)
		restore_from_hvdcp_detection(chip);
	if(chip->re_detect_charger_type_status==false)
	{
	set_dash_charger_present(false);
	}
}

static bool is_src_detect_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_SRC_DET_BIT;
}

static bool is_usbin_uv_high(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		return false;
	}
	return reg &= USBIN_UV_BIT;
}

static int REDET_DELAY_MS = 5000;
module_param(REDET_DELAY_MS, int, 0644);

static bool redet_en = 0;
module_param(redet_en, bool, 0644);

static void smbchg_re_det_chg_type(struct smbchg_chip  *chip)
{
	int rc = 0;
	bool charger_present;
	chip->re_detect_charger_type_status=true;
	pr_err("Re-detect charger type\n");
	rc = vote(chip->usb_icl_votable,
			CHG_SUSPEND_WORKAROUND_ICL_VOTER, true, 300);
	if (rc < 0)
		pr_err("Couldn't vote for 300mA for suspend wa, going ahead rc=%d\n",
				rc);

	pr_err("Faking Removal\n");
	fake_insertion_removal(chip, false);
	msleep(500);
	pr_err("Faking Insertion\n");
	fake_insertion_removal(chip, true);

	rc = vote(chip->usb_icl_votable,
			CHG_SUSPEND_WORKAROUND_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't vote for 0 for suspend wa, going ahead rc=%d\n",
				rc);
	chip->re_detect_charger_type_status=false;
	msleep(100);
	charger_present = (get_prop_charger_voltage_now(chip) > 2500)?true:false;////vbus  > 2.5v
	if(charger_present != chip->usb_online)
	{
	chip->usb_online = charger_present;
	power_supply_set_online(chip->usb_psy, chip->usb_online);
	pr_err("redetect done ,set usb online = %d\n",chip->usb_online);
	}
}


static void smbchg_re_det_work(struct work_struct *work)
{
	struct smbchg_chip *chip = container_of(work,
			struct smbchg_chip,
			re_det_work.work);
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
    if(chip->usb_enum_status !=true)//should add at work actual queueing
    {
	smbchg_re_det_chg_type(chip);
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)
		{
		schedule_delayed_work(&chip->check_switch_dash_work,
					msecs_to_jiffies(500));
		}
		else
		{
		schedule_delayed_work(&chip->nonstandard_charger_charge_current_set_work,
				msecs_to_jiffies(NON_STANDARD_CHARGER_CHECK_MS));
		}
    }
	chip->redect_charger_type_done =true;
}


#define HVDCP_NOTIFY_MS		2500
#define DEFAULT_WALL_CHG_MA	1800
static void handle_usb_insertion(struct smbchg_chip *chip)
{
	struct power_supply *parallel_psy = get_parallel_psy(chip);
	enum power_supply_type usb_supply_type;
	int rc;
	char *usb_type_name = "null";

	pr_smb(PR_STATUS, "triggered\n");
	/* usb inserted */
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	charger_type_value= usb_supply_type;
	/* david.liu@oneplus.tw,20151119  Add delayed workqueue to re-detect charger type */
	if (redet_en && usb_supply_type == POWER_SUPPLY_TYPE_USB) {
		schedule_delayed_work(&chip->re_det_work,
				msecs_to_jiffies(REDET_DELAY_MS));
	}
	pr_err("inserted type = %d (%s)", usb_supply_type, usb_type_name);
	if((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__COLD)&&(qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__HOT) )
	{
		smbchg_charging_en(chip, true);// usb plug in agin ,enable charge
		wake_lock(&chip->chg_wake_lock);
	}
	set_property_on_fg(chip, POWER_SUPPLY_PROP_SET_ALLOW_READ_EXTERN_FG_IIC, true);//set allow read extern fg IIC
	smbchg_aicl_deglitch_wa_check(chip);
	if (chip->typec_psy)
		update_typec_status(chip);
	smbchg_change_usb_supply_type(chip, usb_supply_type);
	if (!chip->skip_usb_notification) {
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}

	/* Notify the USB psy if OV condition is not present */
	if (!chip->usb_ov_det) {
		/*
		 * Note that this could still be a very weak charger
		 * if the handle_usb_insertion was triggered from
		 * the falling edge of an USBIN_OV interrupt
		 */
		pr_smb(PR_MISC, "setting usb psy health %s\n",
				chip->very_weak_charger
				? "UNSPEC_FAILURE" : "GOOD");
		rc = power_supply_set_health_state(chip->usb_psy,
				chip->very_weak_charger
				? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
				: POWER_SUPPLY_HEALTH_GOOD);
		if (rc < 0)
			pr_smb(PR_STATUS,
					"usb psy does not allow updating prop %d rc = %d\n",
					POWER_SUPPLY_HEALTH_GOOD, rc);
	}
		if(!chip->skip_usb_notification)
		{
		schedule_work(&chip->usb_set_online_work);
		}

	if (!chip->hvdcp_not_supported &&
			(usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP)) {
		cancel_delayed_work_sync(&chip->hvdcp_det_work);
		schedule_delayed_work(&chip->hvdcp_det_work,
					msecs_to_jiffies(HVDCP_NOTIFY_MS));
	}

	if (parallel_psy) {
		rc = power_supply_set_present(parallel_psy, true);
		chip->parallel_charger_detected = rc ? false : true;
		if (rc)
			pr_debug("parallel-charger absent rc=%d\n", rc);
	}

	if (chip->parallel.avail && chip->aicl_done_irq
			&& !chip->enable_aicl_wake) {
		rc = enable_irq_wake(chip->aicl_done_irq);
		chip->enable_aicl_wake = true;
	}
	chip->allow_print_log = true;
	if(chip->re_detect_charger_type_status==false && is_usb_present(chip))
	{
		if(chip->probe_success == true)
		{
		schedule_delayed_work(&chip->check_switch_dash_work,
					msecs_to_jiffies(500));
		if(usb_supply_type == POWER_SUPPLY_TYPE_USB)
		{
		schedule_delayed_work(&chip->nonstandard_charger_charge_current_set_work,
					msecs_to_jiffies(NON_STANDARD_CHARGER_CHECK_MS));
		}
		}
	}
}

void update_usb_status(struct smbchg_chip *chip, bool usb_present, bool force)
{
	mutex_lock(&chip->usb_status_lock);
	if (force) {
		chip->usb_present = usb_present;
		chip->usb_present ? handle_usb_insertion(chip)
			: handle_usb_removal(chip);
		goto unlock;
	}
	if (!chip->usb_present && usb_present) {
		chip->usb_present = usb_present;
		handle_usb_insertion(chip);
	} else if (chip->usb_present && !usb_present) {
	if (chip->hvdcp_present) {
		pr_err("HVDCP is removed\n");
		chip->hvdcp_present = false;
	}
		chip->usb_present = usb_present;
		handle_usb_removal(chip);
	}

	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));
unlock:
	mutex_unlock(&chip->usb_status_lock);
}

static int otg_oc_reset(struct smbchg_chip *chip)
{
	int rc;

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, 0);
	if (rc)
		pr_err("Failed to disable OTG rc=%d\n", rc);

	msleep(20);

	/*
	 * There is a possibility that an USBID interrupt might have
	 * occurred notifying USB power supply to disable OTG. We
	 * should not enable OTG in such cases.
	 */
	if (!is_otg_present(chip)) {
		pr_smb(PR_STATUS,
				"OTG is not present, not enabling OTG_EN_BIT\n");
		goto out;
	}

	rc = smbchg_masked_write(chip, chip->bat_if_base + CMD_CHG_REG,
			OTG_EN_BIT, OTG_EN_BIT);
	if (rc)
		pr_err("Failed to re-enable OTG rc=%d\n", rc);

out:
	return rc;
}

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

#define AICL_IRQ_LIMIT_SECONDS	60
#define AICL_IRQ_LIMIT_COUNT	25
static void increment_aicl_count(struct smbchg_chip *chip)
{
	bool bad_charger = false;
	int max_aicl_count, rc;
	u8 reg;
	long elapsed_seconds;
	unsigned long now_seconds;

	pr_smb(PR_INTERRUPT, "aicl count c:%d dgltch:%d first:%ld\n",
			chip->aicl_irq_count, chip->aicl_deglitch_short,
			chip->first_aicl_seconds);

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + ICL_STS_1_REG, 1);
	if (!rc)
		chip->aicl_complete = reg & AICL_STS_BIT;
	else
		chip->aicl_complete = false;

	if (chip->aicl_deglitch_short || chip->force_aicl_rerun) {
		if (!chip->aicl_irq_count)
			get_current_time(&chip->first_aicl_seconds);
		get_current_time(&now_seconds);
		elapsed_seconds = now_seconds
			- chip->first_aicl_seconds;

		if (elapsed_seconds > AICL_IRQ_LIMIT_SECONDS) {
			pr_smb(PR_INTERRUPT,
					"resetting: elp:%ld first:%ld now:%ld c=%d\n",
					elapsed_seconds, chip->first_aicl_seconds,
					now_seconds, chip->aicl_irq_count);
			chip->aicl_irq_count = 1;
			get_current_time(&chip->first_aicl_seconds);
			return;
		}
		/*
		 * Double the amount of AICLs allowed if parallel charging is
		 * enabled.
		 */
		max_aicl_count = AICL_IRQ_LIMIT_COUNT
			* (chip->parallel.avail ? 2 : 1);
		chip->aicl_irq_count++;

		if (chip->aicl_irq_count > max_aicl_count) {
			pr_smb(PR_INTERRUPT, "elp:%ld first:%ld now:%ld c=%d\n",
					elapsed_seconds, chip->first_aicl_seconds,
					now_seconds, chip->aicl_irq_count);
			pr_smb(PR_INTERRUPT, "Disable AICL rerun\n");
			chip->very_weak_charger = true;
			bad_charger = true;

			/*
			 * Disable AICL rerun since many interrupts were
			 * triggered in a short time
			 */
			/* disable hw aicl */
			rc = vote(chip->hw_aicl_rerun_disable_votable,
				WEAK_CHARGER_HW_AICL_VOTER, true, 0);
			if (rc < 0) {
				pr_err("Couldn't disable hw aicl rerun rc=%d\n",
					rc);
				return;
			}

			/* Vote 100mA current limit */
			rc = vote(chip->usb_icl_votable, WEAK_CHARGER_ICL_VOTER,
					true, CURRENT_100_MA);
			if (rc < 0) {
				pr_err("Can't vote %d current limit rc=%d\n",
						CURRENT_100_MA, rc);
			}

			chip->aicl_irq_count = 0;
		} else if ((get_prop_charge_type(chip) ==
					POWER_SUPPLY_CHARGE_TYPE_FAST) &&
				(reg & AICL_SUSP_BIT)) {
			/*
			 * If the AICL_SUSP_BIT is on, then AICL reruns have
			 * already been disabled. Set the very weak charger
			 * flag so that the driver reports a bad charger
			 * and does not reenable AICL reruns.
			 */
			chip->very_weak_charger = true;
			bad_charger = true;
		}
		if (bad_charger) {
			pr_smb(PR_MISC,
					"setting usb psy health UNSPEC_FAILURE\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
			if (rc)
				pr_err("Couldn't set health on usb psy rc:%d\n",
						rc);
			schedule_work(&chip->usb_set_online_work);
		}
	}
}

static int wait_for_usbin_uv(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->usbin_uv_lowered;
	bool usbin_uv;

	if (high)
		completion = &chip->usbin_uv_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	usbin_uv = is_usbin_uv_high(chip);

	if (high == usbin_uv)
		return 0;

	pr_err("usbin uv didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			usbin_uv ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int wait_for_src_detect(struct smbchg_chip *chip, bool high)
{
	int rc;
	int tries = 3;
	struct completion *completion = &chip->src_det_lowered;
	bool src_detect;

	if (high)
		completion = &chip->src_det_raised;

	while (tries--) {
		rc = wait_for_completion_interruptible_timeout(
				completion,
				msecs_to_jiffies(APSD_TIMEOUT_MS));
		if (rc >= 0)
			break;
	}

	src_detect = is_src_detect_high(chip);

	if (high == src_detect)
		return 0;

	pr_err("src detect didnt go to a %s state, still at %s, tries = %d, rc = %d\n",
			high ? "risen" : "lowered",
			src_detect ? "high" : "low",
			tries, rc);
	return -EINVAL;
}

static int fake_insertion_removal(struct smbchg_chip *chip, bool insertion)
{
	int rc;
	bool src_detect;
	bool usbin_uv;

	if (insertion) {
		reinit_completion(&chip->src_det_raised);
		reinit_completion(&chip->usbin_uv_lowered);
	} else {
		reinit_completion(&chip->src_det_lowered);
		reinit_completion(&chip->usbin_uv_raised);
	}

	/* ensure that usbin uv real time status is in the right state */
	usbin_uv = is_usbin_uv_high(chip);
	if (usbin_uv != insertion) {
		pr_err("Skip faking, usbin uv is already %d\n", usbin_uv);
		return -EINVAL;
	}

	/* ensure that src_detect real time status is in the right state */
	src_detect = is_src_detect_high(chip);
	if (src_detect == insertion) {
		pr_err("Skip faking, src detect is already %d\n", src_detect);
		return -EINVAL;
	}

	pr_smb(PR_MISC, "Allow only %s charger\n",
			insertion ? "5-9V" : "9V only");
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + USBIN_CHGR_CFG,
			ADAPTER_ALLOWANCE_MASK,
			insertion ?
			USBIN_ADAPTER_5V_9V_CONT : USBIN_ADAPTER_9V);
	if (rc < 0) {
		pr_err("Couldn't write usb allowance rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s usbin uv\n",
			insertion ? "falling" : "rising");
	rc = wait_for_usbin_uv(chip, !insertion);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on %s src det\n",
			insertion ? "rising" : "falling");
	rc = wait_for_src_detect(chip, insertion);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int smbchg_prepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;
	u8 reg;

	/* switch to 5V HVDCP */
	pr_err("Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	/* disable HVDCP */
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;
	/* fake a removal */
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);
		goto handle_removal;
	}

	/* disable APSD */
	pr_smb(PR_MISC, "Disabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + APSD_CFG,
			AUTO_SRC_DETECT_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable APSD rc=%d\n", rc);
		goto out;
	}

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto handle_removal;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DMF);
	/*
	 * DCP will switch to HVDCP in this time by removing the short
	 * between DP DM
	 */
	msleep(HVDCP_NOTIFY_MS);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and the usb type should be none since APSD was disabled
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if ((reg >> TYPE_BITS_OFFSET) != 0) {
		pr_smb(PR_MISC, "type bits set after 2s sleep - abort\n");
		rc = -EINVAL;
		goto out;
	}

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DP0P6_DM3P3);
	/* Wait 60mS after entering continuous mode */
	msleep(60);

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
handle_removal:
	chip->hvdcp_3_det_ignore_uv = false;
	update_usb_status(chip, 0, 0);
	return rc;
}

static int smbchg_unprepare_for_pulsing(struct smbchg_chip *chip)
{
	int rc = 0;

	set_usb_psy_dp_dm(chip, POWER_SUPPLY_DP_DM_DPF_DMF);

	/* switch to 9V HVDCP */
	pr_err("Switch to 9V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 9V rc=%d\n", rc);
		return rc;
	}

	/* enable HVDCP */
	pr_smb(PR_MISC, "Enable HVDCP\n");
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, HVDCP_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable HVDCP rc=%d\n", rc);
		return rc;
	}

	/* enable APSD */
	pr_smb(PR_MISC, "Enabling APSD\n");
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + APSD_CFG,
			AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
	if (rc < 0) {
		pr_err("Couldn't enable APSD rc=%d\n", rc);
		return rc;
	}

	/* Disable AICL */
	pr_smb(PR_MISC, "Disable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't disable AICL rc=%d\n", rc);
		return rc;
	}

	/* fake a removal */
	chip->hvdcp_3_det_ignore_uv = true;
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0) {
		pr_err("Couldn't fake removal rc=%d\n", rc);
		goto out;
	}

	/*
	 * reset the enabled once flag for parallel charging so
	 * parallel charging can immediately restart after the HVDCP pulsing
	 * is complete
	 */
	chip->parallel.enabled_once = false;

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0) {
		pr_err("Couldn't fake insertion rc=%d\n", rc);
		goto out;
	}
	chip->hvdcp_3_det_ignore_uv = false;

	/* Enable AICL */
	pr_smb(PR_MISC, "Enable AICL\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);
	if (rc < 0) {
		pr_err("Couldn't enable AICL rc=%d\n", rc);
		return rc;
	}

out:
	/*
	 * There are many QC 2.0 chargers that collapse before the aicl deglitch
	 * timer can mitigate. Hence set the aicl deglitch time to a shorter
	 * period.
	 */

	rc = vote(chip->aicl_deglitch_short_votable,
		HVDCP_SHORT_DEGLITCH_VOTER, true, 0);
	if (rc < 0)
		pr_err("Couldn't reduce aicl deglitch rc=%d\n", rc);

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = false;
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "HVDCP removed\n");
		update_usb_status(chip, 0, 0);
	}
	return rc;
}

#define USB_CMD_APSD		0x41
#define APSD_RERUN		BIT(0)
static int rerun_apsd(struct smbchg_chip *chip)
{
	int rc;

	reinit_completion(&chip->src_det_raised);
	reinit_completion(&chip->usbin_uv_lowered);
	reinit_completion(&chip->src_det_lowered);
	reinit_completion(&chip->usbin_uv_raised);

	/* re-run APSD */
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + USB_CMD_APSD,
			APSD_RERUN, APSD_RERUN);
	if (rc) {
		pr_err("Couldn't re-run APSD rc=%d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising usbin uv\n");
	rc = wait_for_usbin_uv(chip, true);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling src det\n");
	rc = wait_for_src_detect(chip, false);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on falling usbin uv\n");
	rc = wait_for_usbin_uv(chip, false);
	if (rc < 0) {
		pr_err("wait for usbin uv failed rc = %d\n", rc);
		return rc;
	}

	pr_smb(PR_MISC, "Waiting on rising src det\n");
	rc = wait_for_src_detect(chip, true);
	if (rc < 0) {
		pr_err("wait for src detect failed rc = %d\n", rc);
		return rc;
	}

	return rc;
}

#define SCHG_LITE_USBIN_HVDCP_5_9V		0x8
#define SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK	0x38
#define SCHG_LITE_USBIN_HVDCP_SEL_IDLE		BIT(3)
static bool is_hvdcp_5v_cont_mode(struct smbchg_chip *chip)
{
	int rc;
	u8 reg = 0;

	rc = smbchg_read(chip, &reg,
			chip->usb_chgpth_base + USBIN_HVDCP_STS, 1);
	if (rc) {
		pr_err("Unable to read HVDCP status rc=%d\n", rc);
		return false;
	}

	pr_smb(PR_STATUS, "HVDCP status = %x\n", reg);

	if (reg & SCHG_LITE_USBIN_HVDCP_SEL_IDLE) {
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + INPUT_STS, 1);
		if (rc) {
			pr_err("Unable to read INPUT status rc=%d\n", rc);
			return false;
		}
		pr_smb(PR_STATUS, "INPUT status = %x\n", reg);
		if ((reg & SCHG_LITE_USBIN_HVDCP_5_9V_SEL_MASK) ==
				SCHG_LITE_USBIN_HVDCP_5_9V)
			return true;
	}
	return false;
}

static int smbchg_prepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	/* check if HVDCP is already in 5V continuous mode */
	if (is_hvdcp_5v_cont_mode(chip)) {
		pr_smb(PR_MISC, "HVDCP by default is in 5V continuous mode\n");
		return 0;
	}

	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		goto out;
	}

	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		goto out;
	}

	pr_smb(PR_MISC, "HVDCP voting for 300mA ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, true, 300);
	if (rc < 0) {
		pr_err("Couldn't vote for 300mA HVDCP ICL rc=%d\n", rc);
		goto out;
	}

	pr_smb(PR_MISC, "Disable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, 0);

	chip->hvdcp_3_det_ignore_uv = true;

	/* re-run APSD */
	rc = rerun_apsd(chip);
	if (rc) {
		pr_err("APSD rerun failed\n");
		goto out;
	}

	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_MISC, "Enable AICL\n");
	smbchg_sec_masked_write(chip, chip->usb_chgpth_base + USB_AICL_CFG,
			AICL_EN_BIT, AICL_EN_BIT);
	/*
	 * DCP will switch to HVDCP in this time by removing the short
	 * between DP DM
	 */
	msleep(HVDCP_NOTIFY_MS);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and the usb type should be none since APSD was disabled
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 2s sleep\n");
		rc = -EINVAL;
		goto out;
	}

	/* We are set if HVDCP in 5V continuous mode */
	if (!is_hvdcp_5v_cont_mode(chip)) {
		pr_err("HVDCP could not be set in 5V continuous mode\n");
		goto out;
	}

	return 0;
out:
	chip->hvdcp_3_det_ignore_uv = false;
	restore_from_hvdcp_detection(chip);
	return rc;
}

static int smbchg_unprepare_for_pulsing_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Forcing 9V HVDCP 2.0\n");
	rc = force_9v_hvdcp(chip);
	if (rc) {
		pr_err("Failed to force 9V HVDCP=%d\n",	rc);
		return rc;
	}

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	return rc;
}

#define CMD_HVDCP_2		0x43
#define SINGLE_INCREMENT	BIT(0)
#define SINGLE_DECREMENT	BIT(1)
static int smbchg_dp_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Increment DP\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
			SINGLE_INCREMENT, SINGLE_INCREMENT);
	if (rc)
		pr_err("Single-increment failed rc=%d\n", rc);

	return rc;
}

static int smbchg_dm_pulse_lite(struct smbchg_chip *chip)
{
	int rc = 0;

	pr_smb(PR_MISC, "Decrement DM\n");
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_HVDCP_2,
			SINGLE_DECREMENT, SINGLE_DECREMENT);
	if (rc)
		pr_err("Single-decrement failed rc=%d\n", rc);

	return rc;
}

static int smbchg_hvdcp3_confirmed(struct smbchg_chip *chip)
{
	int rc = 0;

	/*
	 * reset the enabled once flag for parallel charging because this is
	 * effectively a new insertion.
	 */
	chip->parallel.enabled_once = false;

	pr_smb(PR_MISC, "Retracting HVDCP vote for ICL\n");
	rc = vote(chip->usb_icl_votable, HVDCP_ICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't retract HVDCP ICL vote rc=%d\n", rc);

	smbchg_change_usb_supply_type(chip, POWER_SUPPLY_TYPE_USB_HVDCP_3);

	return rc;
}

static int smbchg_dp_dm(struct smbchg_chip *chip, int val)
{
	int rc = 0;
	int target_icl_vote_ma;

	switch (val) {
	case POWER_SUPPLY_DP_DM_PREPARE:
		if (!is_hvdcp_present(chip)) {
			pr_err("No pulsing unless HVDCP\n");
			return -ENODEV;
		}
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_prepare_for_pulsing_lite(chip);
		else
			rc = smbchg_prepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_UNPREPARE:
		if (chip->schg_version == QPNP_SCHG_LITE)
			rc = smbchg_unprepare_for_pulsing_lite(chip);
		else
			rc = smbchg_unprepare_for_pulsing(chip);
		break;
	case POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3:
		rc = smbchg_hvdcp3_confirmed(chip);
		break;
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DP_PULSE);
		else
			rc = smbchg_dp_pulse_lite(chip);
		if (!rc)
			chip->pulse_cnt++;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		if (chip->schg_version == QPNP_SCHG)
			rc = set_usb_psy_dp_dm(chip,
					POWER_SUPPLY_DP_DM_DM_PULSE);
		else
			rc = smbchg_dm_pulse_lite(chip);
		if (!rc && chip->pulse_cnt)
			chip->pulse_cnt--;
		pr_smb(PR_MISC, "pulse_cnt = %d\n", chip->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_HVDCP3_SUPPORTED:
		chip->hvdcp3_supported = true;
		pr_smb(PR_MISC, "HVDCP3 supported\n");
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		chip->usb_icl_delta -= 100;
		target_icl_vote_ma = get_client_vote(chip->usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->usb_icl_delta);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
		chip->usb_icl_delta += 100;
		target_icl_vote_ma = get_client_vote(chip->usb_icl_votable,
						PSY_ICL_VOTER);
		vote(chip->usb_icl_votable, SW_AICL_ICL_VOTER, true,
				target_icl_vote_ma + chip->usb_icl_delta);
		break;
	default:
		break;
	}

	return rc;
}

static void update_typec_capability_status(struct smbchg_chip *chip,
					const union power_supply_propval *val)
{
	int rc;

	pr_smb(PR_TYPEC, "typec capability = %dma\n", val->intval);

	if (!chip->skip_usb_notification) {
		rc = chip->usb_psy->set_property(chip->usb_psy,
				POWER_SUPPLY_PROP_INPUT_CURRENT_MAX, val);
		if (rc)
			pr_err("typec failed to set current max rc=%d\n", rc);
	}

	pr_debug("changing ICL from %dma to %dma\n", chip->typec_current_ma,
			val->intval);
	chip->typec_current_ma = val->intval;
	smbchg_change_usb_supply_type(chip, chip->usb_supply_type);
}

static void update_typec_otg_status(struct smbchg_chip *chip, int mode,
					bool force)
{
	pr_smb(PR_TYPEC, "typec mode = %d\n", mode);

	if (mode == POWER_SUPPLY_TYPE_DFP) {
		chip->typec_dfp = true;
		power_supply_set_usb_otg(chip->usb_psy, chip->typec_dfp);
		/* update FG */
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
				get_prop_batt_status(chip));
	} else if (force || chip->typec_dfp) {
		chip->typec_dfp = false;
		power_supply_set_usb_otg(chip->usb_psy, chip->typec_dfp);
		/* update FG */
		set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
				get_prop_batt_status(chip));
	}
}

#define CHARGE_OUTPUT_VTG_RATIO		840
static int smbchg_get_iusb(struct smbchg_chip *chip)
{
	int rc, iusb_ua = -EINVAL;
	struct qpnp_vadc_result adc_result;

	if (!is_usb_present(chip) && !is_dc_present(chip))
		return 0;

	if (chip->vchg_vadc_dev && chip->vchg_adc_channel != -EINVAL) {
		rc = qpnp_vadc_read(chip->vchg_vadc_dev,
				chip->vchg_adc_channel, &adc_result);
		if (rc) {
			pr_smb(PR_STATUS,
				"error in VCHG (channel-%d) read rc = %d\n",
						chip->vchg_adc_channel, rc);
			return 0;
		}
		iusb_ua = div_s64(adc_result.physical * 1000,
						CHARGE_OUTPUT_VTG_RATIO);
	}

	return iusb_ua;
}

static enum power_supply_property smbchg_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
	POWER_SUPPLY_PROP_FLASH_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHG_PROTECT_STATUS,
	POWER_SUPPLY_PROP_FASTCHG_STATUS,
	POWER_SUPPLY_PROP_FASTCHG_STARTING,
	POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_FLASH_ACTIVE,
	POWER_SUPPLY_PROP_FLASH_TRIGGER,
	POWER_SUPPLY_PROP_DP_DM,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED,
	POWER_SUPPLY_PROP_RERUN_AICL,
	POWER_SUPPLY_PROP_RESTRICTED_CHARGING,
};

static int smbchg_battery_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
			struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		vote(chip->battchg_suspend_votable, BATTCHG_USER_EN_VOTER,
				!val->intval, 0);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		if(get_prop_fast_chg_started(chip) == true)
		{
		switch_mode_to_normal();
		qpnp_set_fast_chg_allow(chip,false);
		}
		rc = vote(chip->usb_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		rc = vote(chip->dc_suspend_votable, USER_EN_VOTER,
				!val->intval, 0);
		chip->chg_enabled = val->intval;
		schedule_work(&chip->usb_set_online_work);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		pr_err("smbchg_system_temp_level_set=%d\n",val->intval);
		smbchg_system_temp_level_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smbchg_set_fastchg_current_user(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		rc = smbchg_float_voltage_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		rc = smbchg_safety_timer_enable(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		rc = smbchg_switch_buck_frequency(chip, val->intval);
		if (rc) {
			pr_err("Couldn't switch buck frequency, rc=%d\n", rc);
			/*
			 * Trigger a panic if there is an error while switching
			 * buck frequency. This will prevent LS FET damage.
			 */
			BUG_ON(1);
		}

		rc = smbchg_otg_pulse_skip_disable(chip,
				REASON_FLASH_ENABLED, val->intval);
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		chip->flash_triggered = !!val->intval;
		smbchg_icl_loop_disable_check(chip);
		break;
	case POWER_SUPPLY_PROP_FORCE_TLIM:
		rc = smbchg_force_tlim_en(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		rc = smbchg_dp_dm(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		smbchg_rerun_aicl(chip);
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		rc = smbchg_restricted_charging(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_CAPABILITY:
		if (chip->typec_psy)
			update_typec_capability_status(chip, val);
		break;
	case POWER_SUPPLY_PROP_TYPEC_MODE:
		if (chip->typec_psy)
			update_typec_otg_status(chip, val->intval, false);
		break;
	case POWER_SUPPLY_PROP_CHECK_USB_UNPLUG:
		if(chip->usb_online==true && chip->dash_present== false)
		update_dash_unplug_status();
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smbchg_battery_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	int rc;

	switch (prop) {
		case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_CAPACITY:
		case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		case POWER_SUPPLY_PROP_DP_DM:
		case POWER_SUPPLY_PROP_RERUN_AICL:
		case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
			rc = 1;
			break;
		default:
			rc = 0;
			break;
	}
	return rc;
}

static int smbchg_battery_get_property(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
			struct smbchg_chip, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		if ((true==chip->chg_done || chip->recharge_status==true)
			&&(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__LITTLE_COOL
			|| qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__NORMAL
			|| qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__PRE_NORMAL
			|| qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COOL)
			&& (90 < get_prop_batt_capacity(chip)))
			{
			val->intval = POWER_SUPPLY_STATUS_FULL;
			}
		else if(get_prop_fast_chg_started(chip) == true)
		{
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED:
		val->intval =
			!get_effective_result(chip->battchg_suspend_votable);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->chg_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = smbchg_float_voltage_get(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_FLASH_CURRENT_MAX:
		val->intval = smbchg_calc_max_flash_current(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = chip->fastchg_current_ma * 1000;
		break;
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = smbchg_get_aicl_level_ma(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = (int)chip->aicl_complete;
		break;
	case POWER_SUPPLY_PROP_RESTRICTED_CHARGING:
		val->intval = (int)chip->restricted_charging;
		break;
	/* properties from fg */
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = get_prop_batt_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_batt_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_prop_batt_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = get_prop_charger_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_CHG_PROTECT_STATUS:
		val->intval = get_prop_chg_protect_status(chip);
		break;
	case POWER_SUPPLY_PROP_FASTCHG_STATUS:
		val->intval = get_prop_fastchg_status(chip);
		break;
	case POWER_SUPPLY_PROP_FASTCHG_STARTING:
		val->intval = get_prop_fastchg_starting(chip);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = get_prop_batt_voltage_max_design(chip);
		break;
	case POWER_SUPPLY_PROP_SAFETY_TIMER_ENABLE:
		val->intval = chip->safety_timer_en;
		break;
	case POWER_SUPPLY_PROP_FLASH_ACTIVE:
		val->intval = chip->otg_pulse_skip_dis;
		break;
	case POWER_SUPPLY_PROP_FLASH_TRIGGER:
		val->intval = chip->flash_triggered;
		break;
	case POWER_SUPPLY_PROP_DP_DM:
		val->intval = chip->pulse_cnt;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMITED:
		val->intval = smbchg_is_input_current_limited(chip);
		break;
	case POWER_SUPPLY_PROP_RERUN_AICL:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		val->intval = smbchg_get_iusb(chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static char *smbchg_dc_supplicants[] = {
	"bms",
};

static enum power_supply_property smbchg_dc_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int smbchg_dc_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	int rc = 0;
	struct smbchg_chip *chip = container_of(psy,
			struct smbchg_chip, dc_psy);

	switch (prop) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			rc = vote(chip->dc_suspend_votable, POWER_SUPPLY_EN_VOTER,
					!val->intval, 0);
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			rc = vote(chip->dc_icl_votable, USER_ICL_VOTER, true,
					val->intval / 1000);
			break;
		default:
			return -EINVAL;
	}

	return rc;
}

static int smbchg_dc_get_property(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *val)
{
	struct smbchg_chip *chip = container_of(psy,
			struct smbchg_chip, dc_psy);

	switch (prop) {
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = is_dc_present(chip);
			break;
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
			val->intval = !get_effective_result(chip->dc_suspend_votable);
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			/* return if dc is charging the battery */
			val->intval = (smbchg_get_pwr_path(chip) == PWR_PATH_DC)
				&& (get_prop_batt_status(chip)
						== POWER_SUPPLY_STATUS_CHARGING);
			break;
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			val->intval = chip->dc_max_current_ma * 1000;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int smbchg_dc_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	int rc;

	switch (prop) {
		case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		case POWER_SUPPLY_PROP_CURRENT_MAX:
			rc = 1;
			break;
		default:
			rc = 0;
			break;
	}
	return rc;
}

#define HOT_BAT_HARD_BIT	BIT(0)
#define HOT_BAT_SOFT_BIT	BIT(1)
#define COLD_BAT_HARD_BIT	BIT(2)
#define COLD_BAT_SOFT_BIT	BIT(3)
#define BAT_OV_BIT		BIT(4)
#define BAT_LOW_BIT		BIT(5)
#define BAT_MISSING_BIT		BIT(6)
#define BAT_TERM_MISSING_BIT	BIT(7)
static irqreturn_t batt_hot_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_hot = !!(reg & HOT_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cold_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cold = !!(reg & COLD_BAT_HARD_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_warm_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_warm = !!(reg & HOT_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_cool_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_cool = !!(reg & COLD_BAT_SOFT_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t batt_pres_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	chip->batt_present = !(reg & BAT_MISSING_BIT);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_HEALTH,
			get_prop_batt_health(chip));
	return IRQ_HANDLED;
}

static irqreturn_t vbat_low_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("vbat low\n");
	return IRQ_HANDLED;
}

#define CHG_COMP_SFT_BIT	BIT(3)
static irqreturn_t chg_error_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc = 0;
	u8 reg;

	pr_smb(PR_INTERRUPT, "chg-error triggered\n");

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Unable to read RT_STS rc = %d\n", rc);
	} else {
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
		if (reg & CHG_COMP_SFT_BIT)
			set_property_on_fg(chip,
					POWER_SUPPLY_PROP_SAFETY_TIMER_EXPIRED,
					1);
	}

	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t fastchg_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;

	pr_smb(PR_INTERRUPT, "p2f triggered\n");
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_hot_handler(int irq, void *_chip)
{
	pr_warn_ratelimited("chg hot\n");
	smbchg_wipower_check(_chip);
	return IRQ_HANDLED;
}

static irqreturn_t chg_term_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg = 0;
	bool terminated = false;

	rc = smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	if (rc) {
		dev_err(chip->dev, "Error reading RT_STS rc= %d\n", rc);
	} else {
		terminated = !!(reg & BAT_TCC_REACHED_BIT);
		pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	}
	/*
	 * If charging has not actually terminated, then this means that
	 * either this is a manual call to chg_term_handler during
	 * determine_initial_status(), or the charger has instantly restarted
	 * charging.
	 *
	 * In either case, do not do the usual status updates here. If there
	 * is something that needs to be updated, the recharge handler will
	 * handle it.
	 */
	if (terminated) {
		smbchg_parallel_usb_check_ok(chip);
		if (chip->psy_registered)
			power_supply_changed(&chip->batt_psy);
		smbchg_charging_status_change(chip);
		set_property_on_fg(chip, POWER_SUPPLY_PROP_CHARGE_DONE, 1);
		chip->chg_done =true;
		pr_err("chg_term_handler: chip->chg_done:temp:%d,soc_calib:%d,VOLT:%d,current:%d\n",
			get_prop_batt_temp(chip),get_prop_batt_capacity(chip),get_prop_batt_voltage_now(chip),get_prop_batt_current_now(chip));
		smbchg_charging_en(chip, 0);
	}
	return IRQ_HANDLED;
}

static irqreturn_t taper_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	taper_irq_en(chip, false);
	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_taper(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

static irqreturn_t recharge_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->chgr_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	smbchg_parallel_usb_check_ok(chip);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

static irqreturn_t wdog_timeout_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_warn_ratelimited("wdog timeout rt_stat = 0x%02x\n", reg);
	if (chip->psy_registered)
		power_supply_changed(&chip->batt_psy);
	smbchg_charging_status_change(chip);
	return IRQ_HANDLED;
}

/**
 * power_ok_handler() - called when the switcher turns on or turns off
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating switcher turning on or off
 */
static irqreturn_t power_ok_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	u8 reg = 0;

	smbchg_read(chip, &reg, chip->misc_base + RT_STS, 1);
	pr_smb(PR_INTERRUPT, "triggered: 0x%02x\n", reg);
	return IRQ_HANDLED;
}

/**
 * dcin_uv_handler() - called when the dc voltage crosses the uv threshold
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating whether dc voltage is uv
 */
#define DCIN_UNSUSPEND_DELAY_MS		1000
static irqreturn_t dcin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool dc_present = is_dc_present(chip);

	pr_smb(PR_STATUS, "chip->dc_present = %d dc_present = %d\n",
			chip->dc_present, dc_present);

	if (chip->dc_present != dc_present) {
		/* dc changed */
		chip->dc_present = dc_present;
		if (chip->dc_psy_type != -EINVAL && chip->psy_registered)
			power_supply_changed(&chip->dc_psy);
		smbchg_charging_status_change(chip);
		smbchg_aicl_deglitch_wa_check(chip);
		chip->vbat_above_headroom = false;
	}

	smbchg_wipower_check(chip);
	return IRQ_HANDLED;
}

/**
 * usbin_ov_handler() - this is called when an overvoltage condition occurs
 * @chip: pointer to smbchg_chip chip
 */
static irqreturn_t usbin_ov_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int rc;
	u8 reg;
	bool usb_present;

	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read usb rt status rc = %d\n", rc);
		goto out;
	}

	/* OV condition is detected. Notify it to USB psy */
	if (reg & USBIN_OV_BIT) {
		chip->usb_ov_det = true;
		if (chip->usb_psy) {
			pr_smb(PR_MISC, "setting usb psy health OV\n");
			rc = power_supply_set_health_state(chip->usb_psy,
					POWER_SUPPLY_HEALTH_OVERVOLTAGE);
			if (rc)
				pr_smb(PR_STATUS,
						"usb psy does not allow updating prop %d rc = %d\n",
						POWER_SUPPLY_HEALTH_OVERVOLTAGE, rc);
		}
	} else {
		chip->usb_ov_det = false;
		/* If USB is present, then handle the USB insertion */
		usb_present = is_usb_present(chip);
		if (usb_present)
			update_usb_status(chip, usb_present, false);
	}
out:
	return IRQ_HANDLED;
}

/**
 * usbin_uv_handler() - this is called when USB charger is removed
 * @chip: pointer to smbchg_chip chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
#define ICL_MODE_MASK		SMB_MASK(5, 4)
#define ICL_MODE_HIGH_CURRENT	0
static irqreturn_t usbin_uv_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	int aicl_level = smbchg_get_aicl_level_ma(chip);
	int rc;
	u8 reg;
	pr_err("%s triggered\n",__func__);
	if((get_prop_fast_chg_started(chip)== true) || (usb_switched == true))
		{
		if(usb_switched == true)
			{
			usb_switched = false;
		}
			return IRQ_HANDLED;
		}
	rc = smbchg_read(chip, &reg, chip->usb_chgpth_base + RT_STS, 1);
	if (rc) {
		pr_err("could not read rt sts: %d", rc);
		goto out;
	}
	pr_err("%s chip->usb_present = %d rt_sts = 0x%02x hvdcp_3_det_ignore_uv = %d aicl = %d\n",
			chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
			chip->usb_present, reg, chip->hvdcp_3_det_ignore_uv,
			aicl_level);
	/*
	 * set usb_psy's dp=f dm=f if this is a new insertion, i.e. it is
	 * not already src_detected and usbin_uv is seen falling
	 */
	if (!(reg & USBIN_UV_BIT) && !(reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
	}

	if (reg & USBIN_UV_BIT)
		complete_all(&chip->usbin_uv_raised);
	else
		complete_all(&chip->usbin_uv_lowered);

	if (chip->hvdcp_3_det_ignore_uv)
		goto out;

	if ((reg & USBIN_UV_BIT) && (reg & USBIN_SRC_DET_BIT)) {
		pr_smb(PR_STATUS, "Very weak charger detected\n");
		chip->very_weak_charger = true;
		rc = smbchg_read(chip, &reg,
				chip->usb_chgpth_base + ICL_STS_2_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Could not read usb icl sts 2: %d\n",
					rc);
			goto out;
		}
		if ((reg & ICL_MODE_MASK) != ICL_MODE_HIGH_CURRENT) {
			/*
			 * If AICL is not even enabled, this is either an
			 * SDP or a grossly out of spec charger. Do not
			 * draw any current from it.
			 */
			rc = vote(chip->usb_suspend_votable,
					WEAK_CHARGER_EN_VOTER, true, 0);
			if (rc < 0)
				pr_err("could not disable charger: %d", rc);
		} else if (aicl_level == chip->tables.usb_ilim_ma_table[0]) {
			/*
			 * we are in a situation where the adapter is not able
			 * to supply even 300mA. Disable hw aicl reruns else it
			 * is only a matter of time when we get back here again
			 */
			rc = vote(chip->hw_aicl_rerun_disable_votable,
				WEAK_CHARGER_HW_AICL_VOTER, true, 0);
			if (rc < 0)
				pr_err("Couldn't disable hw aicl rerun rc=%d\n",
						rc);
		}
		pr_smb(PR_MISC, "setting usb psy health UNSPEC_FAILURE\n");
		rc = power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		if (rc)
			pr_err("Couldn't set health on usb psy rc:%d\n", rc);
		schedule_work(&chip->usb_set_online_work);
	}

	smbchg_wipower_check(chip);
out:
	return IRQ_HANDLED;
}

/**
 * src_detect_handler() - this is called on rising edge when USB charger type
 *			is detected and on falling edge when USB voltage falls
 *			below the coarse detect voltage(1V), use it for
 *			handling USB charger insertion and removal.
 * @chip: pointer to smbchg_chip
 * @rt_stat: the status bit indicating chg insertion/removal
 */
static irqreturn_t src_detect_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	bool src_detect = is_src_detect_high(chip);
	int rc;

	pr_info("%s chip->usb_present = %d usb_present = %d src_detect = %d hvdcp_3_det_ignore_uv=%d\n",
		chip->hvdcp_3_det_ignore_uv ? "Ignoring":"",
		chip->usb_present, usb_present, src_detect,
		chip->hvdcp_3_det_ignore_uv);

	if(get_prop_fast_chg_started(chip)== true)
	{
	power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_DASH);
	return IRQ_HANDLED;
	}

	if (src_detect)
		complete_all(&chip->src_det_raised);
	else
		complete_all(&chip->src_det_lowered);

	if (chip->hvdcp_3_det_ignore_uv)
		goto out;

	/*
	 * When VBAT is above the AICL threshold (4.25V) - 180mV (4.07V),
	 * an input collapse due to AICL will actually cause an USBIN_UV
	 * interrupt to fire as well.
	 *
	 * Handle USB insertions and removals in the source detect handler
	 * instead of the USBIN_UV handler since the latter is untrustworthy
	 * when the battery voltage is high.
	 */
	chip->very_weak_charger = false;
	/*
	 * a src detect marks a new insertion or a real removal,
	 * vote for enable aicl hw reruns
	 */
	rc = vote(chip->hw_aicl_rerun_disable_votable,
		WEAK_CHARGER_HW_AICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't enable hw aicl rerun rc=%d\n", rc);

	rc = vote(chip->usb_suspend_votable, WEAK_CHARGER_EN_VOTER, false, 0);
	if (rc < 0)
		pr_err("could not enable charger: %d\n", rc);

	if (src_detect) {
		update_usb_status(chip, usb_present, true);
	} else {
		update_usb_status(chip, 0, true);
		chip->aicl_irq_count = 0;
	}
out:
	return IRQ_HANDLED;
}

/**
 * otg_oc_handler() - called when the usb otg goes over current
 */
#define NUM_OTG_RETRIES			5
#define OTG_OC_RETRY_DELAY_US		50000
static irqreturn_t otg_oc_handler(int irq, void *_chip)
{
	int rc;
	struct smbchg_chip *chip = _chip;
	s64 elapsed_us = ktime_us_delta(ktime_get(), chip->otg_enable_time);

	pr_smb(PR_INTERRUPT, "triggered\n");

	if (chip->schg_version == QPNP_SCHG_LITE) {
		pr_warn("OTG OC triggered - OTG disabled\n");
		return IRQ_HANDLED;
	}

	if (elapsed_us > OTG_OC_RETRY_DELAY_US)
		chip->otg_retries = 0;

	/*
	 * Due to a HW bug in the PMI8994 charger, the current inrush that
	 * occurs when connecting certain OTG devices can cause the OTG
	 * overcurrent protection to trip.
	 *
	 * The work around is to try reenabling the OTG when getting an
	 * overcurrent interrupt once.
	 */
	if (chip->otg_retries < NUM_OTG_RETRIES) {
		chip->otg_retries += 1;
		pr_smb(PR_STATUS,
				"Retrying OTG enable. Try #%d, elapsed_us %lld\n",
				chip->otg_retries, elapsed_us);
		rc = otg_oc_reset(chip);
		if (rc)
			pr_err("Failed to reset OTG OC state rc=%d\n", rc);
		chip->otg_enable_time = ktime_get();
	}
	return IRQ_HANDLED;
}

/**
 * otg_fail_handler() - called when the usb otg fails
 * (when vbat < OTG UVLO threshold)
 */
static irqreturn_t otg_fail_handler(int irq, void *_chip)
{
	pr_smb(PR_INTERRUPT, "triggered\n");
	return IRQ_HANDLED;
}

/**
 * aicl_done_handler() - called when the usb AICL algorithm is finished
 *			and a current is set.
 */
static irqreturn_t aicl_done_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool usb_present = is_usb_present(chip);
	int aicl_level = smbchg_get_aicl_level_ma(chip);

	pr_smb(PR_INTERRUPT, "triggered, aicl: %d\n", aicl_level);

	increment_aicl_count(chip);

	if (usb_present)
		smbchg_parallel_usb_check_ok(chip);

	if (chip->aicl_complete)
		power_supply_changed(&chip->batt_psy);

	return IRQ_HANDLED;
}

/**
 * usbid_change_handler() - called when the usb RID changes.
 * This is used mostly for detecting OTG
 */
static irqreturn_t usbid_change_handler(int irq, void *_chip)
{
	struct smbchg_chip *chip = _chip;
	bool otg_present;

	pr_smb(PR_INTERRUPT, "triggered\n");

	otg_present = is_otg_present(chip);
	if (chip->usb_psy) {
		pr_smb(PR_MISC, "setting usb psy OTG = %d\n",
				otg_present ? 1 : 0);
		power_supply_set_usb_otg(chip->usb_psy, otg_present ? 1 : 0);
	}
	if (otg_present)
		pr_smb(PR_STATUS, "OTG detected\n");

	/* update FG */
	set_property_on_fg(chip, POWER_SUPPLY_PROP_STATUS,
			get_prop_batt_status(chip));

	return IRQ_HANDLED;
}

static int determine_initial_status(struct smbchg_chip *chip)
{
	union power_supply_propval type = {0, };

	/*
	 * It is okay to read the interrupt status here since
	 * interrupts aren't requested. reading interrupt status
	 * clears the interrupt so be careful to read interrupt
	 * status only in interrupt handling code
	 */

	batt_pres_handler(0, chip);
	batt_hot_handler(0, chip);
	batt_warm_handler(0, chip);
	batt_cool_handler(0, chip);
	batt_cold_handler(0, chip);
	if (chip->typec_psy) {
		get_property_from_typec(chip, POWER_SUPPLY_PROP_TYPE, &type);
		update_typec_otg_status(chip, type.intval, true);
	} else {
		usbid_change_handler(0, chip);
	}
	src_detect_handler(0, chip);

	chip->usb_present = is_usb_present(chip);
	chip->dc_present = is_dc_present(chip);

	if (chip->usb_present) {

		pr_smb(PR_MISC, "setting usb psy dp=f dm=f\n");
		power_supply_set_dp_dm(chip->usb_psy,
				POWER_SUPPLY_DP_DM_DPF_DMF);
		handle_usb_insertion(chip);
	} else {
		handle_usb_removal(chip);
	}

	return 0;
}

static int prechg_time[] = {
	24,
	48,
	96,
	192,
};
static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

enum bpd_type {
	BPD_TYPE_BAT_NONE,
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
	BPD_TYPE_DEFAULT,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_NONE]		= "bpd_none",
	[BPD_TYPE_BAT_ID]		= "bpd_id",
	[BPD_TYPE_BAT_THM]		= "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID]	= "bpd_thm_id",
};

static inline int get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

#define REVISION1_REG			0x0
#define DIG_MINOR			0
#define DIG_MAJOR			1
#define ANA_MINOR			2
#define ANA_MAJOR			3
#define CHGR_CFG1			0xFB
#define RECHG_THRESHOLD_SRC_BIT		BIT(1)
#define TERM_I_SRC_BIT			BIT(2)
#define TERM_SRC_FG			BIT(2)
#define CHG_INHIB_CFG_REG		0xF7
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x01
#define CHG_INHIBIT_200MV_VAL		0x02
#define CHG_INHIBIT_300MV_VAL		0x03
#define CHG_INHIBIT_MASK		0x03
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define CHGR_CFG2			0xFC
#define CHG_EN_SRC_BIT			BIT(7)
#define CHG_EN_POLARITY_BIT		BIT(6)
#define P2F_CHG_TRAN			BIT(5)
#define CHG_BAT_OV_ECC			BIT(4)
#define I_TERM_BIT			BIT(3)
#define AUTO_RECHG_BIT			BIT(2)
#define CHARGER_INHIBIT_BIT		BIT(0)
#define USB51_COMMAND_POL		BIT(2)
#define USB51AC_CTRL			BIT(1)
#define TR_8OR32B			0xFE
#define BUCK_8_16_FREQ_BIT		BIT(0)
#define BM_CFG				0xF3
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BMD_PIN_SRC_MASK		SMB_MASK(1, 0)
#define PIN_SRC_SHIFT			0
#define CHGR_CFG			0xFF
#define RCHG_LVL_BIT			BIT(0)
#define VCHG_EN_BIT			BIT(1)
#define VCHG_INPUT_CURRENT_BIT		BIT(3)
#define CFG_AFVC			0xF6
#define VFLOAT_COMP_ENABLE_MASK		SMB_MASK(2, 0)
#define TR_RID_REG			0xFA
#define FG_INPUT_FET_DELAY_BIT		BIT(3)
#define TRIM_OPTIONS_7_0		0xF6
#define INPUT_MISSING_POLLER_EN_BIT	BIT(3)
#define CHGR_CCMP_CFG			0xFA
#define JEITA_TEMP_HARD_LIMIT_BIT	BIT(5)
#define HVDCP_ADAPTER_SEL_MASK		SMB_MASK(5, 4)
#define HVDCP_ADAPTER_SEL_9V_BIT	BIT(4)
#define HVDCP_AUTH_ALG_EN_BIT		BIT(6)
#define CMD_APSD			0x41
#define APSD_RERUN_BIT			BIT(0)
#define OTG_CFG				0xF1
#define HICCUP_ENABLED_BIT		BIT(6)
#define OTG_PIN_POLARITY_BIT		BIT(4)
#define OTG_PIN_ACTIVE_LOW		BIT(4)
#define OTG_EN_CTRL_MASK		SMB_MASK(3, 2)
#define OTG_PIN_CTRL_RID_DIS		0x04
#define OTG_CMD_CTRL_RID_EN		0x08
#define AICL_ADC_BIT			BIT(6)
static void batt_ov_wa_check(struct smbchg_chip *chip)
{
	int rc;
	u8 reg;

	/* disable-'battery OV disables charging' feature */
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_BAT_OV_ECC, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return;
	}

	/*
	 * if battery OV is set:
	 * restart charging by disable/enable charging
	 */
	rc = smbchg_read(chip, &reg, chip->bat_if_base + RT_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev,
				"Couldn't read Battery RT status rc = %d\n", rc);
		return;
	}

	if (reg & BAT_OV_BIT) {
		rc = smbchg_charging_en(chip, false);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't disable charging: rc = %d\n", rc);
			return;
		}

		/* delay for charging-disable to take affect */
		msleep(200);

		rc = smbchg_charging_en(chip, true);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't enable charging: rc = %d\n", rc);
			return;
		}
	}
}

static int smbchg_hw_init(struct smbchg_chip *chip)
{
	int rc, i;
	u8 reg, mask;

	rc = smbchg_read(chip, chip->revision,
			chip->misc_base + REVISION1_REG, 4);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read revision rc=%d\n",
				rc);
		return rc;
	}
	pr_smb(PR_STATUS, "Charger Revision DIG: %d.%d; ANA: %d.%d\n",
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR]);

	if (!chip->hvdcp_not_supported) {
		/* Setup 9V HVDCP */
		rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_9V);
		if (rc < 0) {
			pr_err("Couldn't set hvdcp config in chgpath_chg rc=%d\n",
					rc);
			return rc;
		}
	} else {
		pr_info("enable APSD\n");
		rc = smbchg_sec_masked_write(chip,
				chip->usb_chgpth_base + APSD_CFG,
				AUTO_SRC_DETECT_EN_BIT, AUTO_SRC_DETECT_EN_BIT);
		if (rc < 0) {
			pr_err("Couldn't enable APSD rc=%d\n", rc);
			return rc;
		}

		pr_info("disabe HVDCP\n");
		rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_EN_BIT, 0);
		if (rc < 0) {
			pr_err("Couldn't disable HVDCP rc=%d\n", rc);
			return rc;
		}
	}

	if (chip->aicl_rerun_period_s > 0) {
		rc = smbchg_set_aicl_rerun_period_s(chip,
				chip->aicl_rerun_period_s);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set AICL rerun timer rc=%d\n",
					rc);
			return rc;
		}
	}

	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + TR_RID_REG,
			FG_INPUT_FET_DELAY_BIT, FG_INPUT_FET_DELAY_BIT);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable fg input fet delay rc=%d\n",
				rc);
		return rc;
	}

	rc = smbchg_sec_masked_write(chip, chip->misc_base + TRIM_OPTIONS_7_0,
			INPUT_MISSING_POLLER_EN_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't disable input missing poller rc=%d\n",
				rc);
		return rc;
	}

	/*
	 * Do not force using current from the register i.e. use auto
	 * power source detect (APSD) mA ratings for the initial current values.
	 *
	 * If this is set, AICL will not rerun at 9V for HVDCPs
	 */
	rc = smbchg_masked_write(chip, chip->usb_chgpth_base + CMD_IL,
			USE_REGISTER_FOR_CURRENT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable auto recharge by default.
	 * enable current termination and charge inhibition based on
	 * the device tree configuration.
	 */
	rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG2,
			CHG_EN_SRC_BIT | CHG_EN_POLARITY_BIT | P2F_CHG_TRAN
			| I_TERM_BIT | AUTO_RECHG_BIT | CHARGER_INHIBIT_BIT,
			CHG_EN_POLARITY_BIT
			| (chip->chg_inhibit_en ? CHARGER_INHIBIT_BIT : 0)
			| (chip->iterm_disabled ? I_TERM_BIT : 0));
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * enable battery charging to make sure it hasn't been changed earlier
	 * by the bootloader.
	 */
	rc = smbchg_charging_en(chip, true);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't enable battery charging=%d\n", rc);
		return rc;
	}

	/*
	 * Based on the configuration, use the analog sensors or the fuelgauge
	 * adc for recharge threshold source.
	 */

	if (chip->chg_inhibit_source_fg)
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
				TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT,
				TERM_SRC_FG | RECHG_THRESHOLD_SRC_BIT);
	else
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG1,
				TERM_I_SRC_BIT | RECHG_THRESHOLD_SRC_BIT, 0);

	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set chgr_cfg2 rc=%d\n", rc);
		return rc;
	}

	/*
	 * control USB suspend via command bits and set correct 100/500mA
	 * polarity on the usb current
	 */
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			USB51_COMMAND_POL | USB51AC_CTRL, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set usb_chgpth cfg rc=%d\n", rc);
		return rc;
	}

	check_battery_type(chip);

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smbchg_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set vfloat to %d\n", chip->vfloat_mv);
	}

	/* set the fast charge current compensation */
	if (chip->fastchg_current_comp != -EINVAL) {
		rc = smbchg_fastchg_current_comp_set(chip,
				chip->fastchg_current_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set fastchg current comp rc = %d\n",
					rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set fastchg current comp to %d\n",
				chip->fastchg_current_comp);
	}

	/* set the float voltage compensation */
	if (chip->float_voltage_comp != -EINVAL) {
		rc = smbchg_float_voltage_comp_set(chip,
				chip->float_voltage_comp);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set float voltage comp rc = %d\n",
					rc);
			return rc;
		}
		pr_smb(PR_STATUS, "set float voltage comp to %d\n",
				chip->float_voltage_comp);
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			smbchg_iterm_set(chip, chip->iterm_ma);
		}
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		reg = (chip->safety_time > 0 ? 0 : SFT_TIMER_DISABLE_BIT) |
			(chip->prechg_safety_time > 0
			 ? 0 : PRECHG_SFT_TIMER_DISABLE_BIT);

		for (i = 0; i < ARRAY_SIZE(chg_time); i++) {
			if (chip->safety_time <= chg_time[i]) {
				reg |= i << SAFETY_TIME_MINUTES_SHIFT;
				break;
			}
		}
		for (i = 0; i < ARRAY_SIZE(prechg_time); i++) {
			if (chip->prechg_safety_time <= prechg_time[i]) {
				reg |= i;
				break;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + SFT_CFG,
				SFT_EN_MASK | SFT_TO_MASK |
				(chip->prechg_safety_time > 0
				 ? PRECHG_SFT_TO_MASK : 0), reg);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't set safety timer rc = %d\n",
					rc);
			return rc;
		}
		chip->safety_timer_en = true;
	} else {
		rc = smbchg_read(chip, &reg, chip->chgr_base + SFT_CFG, 1);
		if (rc < 0)
			dev_err(chip->dev, "Unable to read SFT_CFG rc = %d\n",
					rc);
		else if (!(reg & SFT_EN_MASK))
			chip->safety_timer_en = true;
	}

	/* configure jeita temperature hard limit */
	if (chip->jeita_temp_hard_limit >= 0) {
		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + CHGR_CCMP_CFG,
				JEITA_TEMP_HARD_LIMIT_BIT,
				chip->jeita_temp_hard_limit
				? 0 : JEITA_TEMP_HARD_LIMIT_BIT);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't set jeita temp hard limit rc = %d\n",
					rc);
			return rc;
		}
	}

	/* make the buck switch faster to prevent some vbus oscillation */
	rc = smbchg_sec_masked_write(chip,
			chip->usb_chgpth_base + TR_8OR32B,
			BUCK_8_16_FREQ_BIT, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set buck frequency rc = %d\n", rc);
		return rc;
	}

	/* battery missing detection */
	mask =  BATT_MISSING_ALGO_BIT;
	reg = chip->bmd_algo_disabled ? BATT_MISSING_ALGO_BIT : 0;
	if (chip->bmd_pin_src < BPD_TYPE_DEFAULT) {
		mask |= BMD_PIN_SRC_MASK;
		reg |= chip->bmd_pin_src << PIN_SRC_SHIFT;
	}
	rc = smbchg_sec_masked_write(chip,
			chip->bat_if_base + BM_CFG, mask, reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't set batt_missing config = %d\n",
				rc);
		return rc;
	}

	if (chip->vchg_adc_channel != -EINVAL) {
		/* configure and enable VCHG */
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CHGR_CFG,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT,
				VCHG_INPUT_CURRENT_BIT | VCHG_EN_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	smbchg_charging_status_change(chip);

	vote(chip->usb_suspend_votable, USER_EN_VOTER, !chip->chg_enabled, 0);
	vote(chip->dc_suspend_votable, USER_EN_VOTER, !chip->chg_enabled, 0);
	/* resume threshold */
	if (chip->resume_delta_mv != -EINVAL) {

		/*
		 * Configure only if the recharge threshold source is not
		 * fuel gauge ADC.
		 */
		if (!chip->chg_inhibit_source_fg) {
			if (chip->resume_delta_mv < 100)
				reg = CHG_INHIBIT_50MV_VAL;
			else if (chip->resume_delta_mv < 200)
				reg = CHG_INHIBIT_100MV_VAL;
			else if (chip->resume_delta_mv < 300)
				reg = CHG_INHIBIT_200MV_VAL;
			else
				reg = CHG_INHIBIT_300MV_VAL;

			rc = smbchg_sec_masked_write(chip,
					chip->chgr_base + CHG_INHIB_CFG_REG,
					CHG_INHIBIT_MASK, reg);
			if (rc < 0) {
				dev_err(chip->dev, "Couldn't set inhibit val rc = %d\n",
						rc);
				return rc;
			}
		}

		rc = smbchg_sec_masked_write(chip,
				chip->chgr_base + CHGR_CFG,
				RCHG_LVL_BIT,
				(chip->resume_delta_mv
				 < chip->tables.rchg_thr_mv)
				? 0 : RCHG_LVL_BIT);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set recharge rc = %d\n",
					rc);
			return rc;
		}
	}

	/* DC path current settings */
	if (chip->dc_psy_type != -EINVAL) {
		rc = vote(chip->dc_icl_votable, PSY_ICL_VOTER, true,
				chip->dc_target_current_ma);
		if (rc < 0) {
			dev_err(chip->dev,
					"Couldn't vote for initial DC ICL rc=%d\n", rc);
			return rc;
		}
	}


	/*
	 * on some devices the battery is powered via external sources which
	 * could raise its voltage above the float voltage. smbchargers go
	 * in to reverse boost in such a situation and the workaround is to
	 * disable float voltage compensation (note that the battery will appear
	 * hot/cold when powered via external source).
	 */
	if (chip->soft_vfloat_comp_disabled) {
		rc = smbchg_sec_masked_write(chip, chip->chgr_base + CFG_AFVC,
				VFLOAT_COMP_ENABLE_MASK, 0);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't disable soft vfloat rc = %d\n",
					rc);
			return rc;
		}
	}
	rc = vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,500);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't vote fastchg ma rc = %d\n", rc);
		return rc;
	}
	rc = smbchg_read(chip, &chip->original_usbin_allowance,
			chip->usb_chgpth_base + USBIN_CHGR_CFG, 1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't read usb allowance rc=%d\n", rc);

	if (chip->wipower_dyn_icl_avail) {
		rc = smbchg_wipower_ilim_config(chip,
				&(chip->wipower_default.entries[0]));
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set default wipower ilim = %d\n",
					rc);
			return rc;
		}
	}
	/* unsuspend dc path, it could be suspended by the bootloader */
	rc = smbchg_dc_suspend(chip, 0);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't unsuspend dc path= %d\n", rc);
		return rc;
	}

	if (chip->force_aicl_rerun) {
		/* vote to enable hw aicl */
		rc = vote(chip->hw_aicl_rerun_enable_indirect_votable,
			DEFAULT_CONFIG_HW_AICL_VOTER, true, 0);
		if (rc < 0) {
			pr_err("Couldn't vote enable hw aicl rerun rc=%d\n",
				rc);
			return rc;
		}
	}

	if (chip->schg_version == QPNP_SCHG_LITE) {
		/* enable OTG hiccup mode */
		rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG,
					HICCUP_ENABLED_BIT, HICCUP_ENABLED_BIT);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't set OTG OC config rc = %d\n",
					rc);
	}

	if (chip->otg_pinctrl) {
		/* configure OTG enable to pin control active low */
		rc = smbchg_sec_masked_write(chip, chip->otg_base + OTG_CFG,
				OTG_PIN_POLARITY_BIT | OTG_EN_CTRL_MASK,
				OTG_PIN_ACTIVE_LOW | OTG_PIN_CTRL_RID_DIS);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't set OTG EN config rc = %d\n",
				rc);
			return rc;
		}
	}

	if (chip->wa_flags & SMBCHG_BATT_OV_WA)
		batt_ov_wa_check(chip);

	/* turn off AICL adc for improved accuracy */
	rc = smbchg_sec_masked_write(chip,
			chip->misc_base + MISC_TRIM_OPT_15_8, AICL_ADC_BIT, 0);
	if (rc)
		pr_err("Couldn't write to MISC_TRIM_OPTIONS_15_8 rc=%d\n",
				rc);

	return rc;
}

static struct of_device_id smbchg_match_table[] = {
	{
		.compatible     = "qcom,qpnp-smbcharger",
	},
	{ },
};

#define DC_MA_MIN 300
#define DC_MA_MAX 2000
#define OF_PROP_READ(chip, prop, dt_property, retval, optional)		\
	do {									\
		if (retval)							\
		break;							\
		if (optional)							\
		prop = -EINVAL;						\
		\
		retval = of_property_read_u32(chip->spmi->dev.of_node,		\
				"qcom," dt_property	,	\
				&prop);				\
		\
		if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
		else if (retval)						\
		dev_err(chip->dev, "Error reading " #dt_property	\
				" property rc = %d\n", rc);		\
	} while (0)

#define ILIM_ENTRIES		3
#define VOLTAGE_RANGE_ENTRIES	2
#define RANGE_ENTRY		(ILIM_ENTRIES + VOLTAGE_RANGE_ENTRIES)
static int smb_parse_wipower_map_dt(struct smbchg_chip *chip,
		struct ilim_map *map, char *property)
{
	struct device_node *node = chip->dev->of_node;
	int total_elements, size;
	struct property *prop;
	const __be32 *data;
	int num, i;

	prop = of_find_property(node, property, &size);
	if (!prop) {
		dev_err(chip->dev, "%s missing\n", property);
		return -EINVAL;
	}

	total_elements = size / sizeof(int);
	if (total_elements % RANGE_ENTRY) {
		dev_err(chip->dev, "%s table not in multiple of %d, total elements = %d\n",
				property, RANGE_ENTRY, total_elements);
		return -EINVAL;
	}

	data = prop->value;
	num = total_elements / RANGE_ENTRY;
	map->entries = devm_kzalloc(chip->dev,
			num * sizeof(struct ilim_entry), GFP_KERNEL);
	if (!map->entries) {
		dev_err(chip->dev, "kzalloc failed for default ilim\n");
		return -ENOMEM;
	}
	for (i = 0; i < num; i++) {
		map->entries[i].vmin_uv =  be32_to_cpup(data++);
		map->entries[i].vmax_uv =  be32_to_cpup(data++);
		map->entries[i].icl_pt_ma =  be32_to_cpup(data++);
		map->entries[i].icl_lv_ma =  be32_to_cpup(data++);
		map->entries[i].icl_hv_ma =  be32_to_cpup(data++);
	}
	map->num = num;
	return 0;
}

static int smb_parse_wipower_dt(struct smbchg_chip *chip)
{
	int rc = 0;

	chip->wipower_dyn_icl_avail = false;

	if (!chip->vadc_dev)
		goto err;

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_default,
			"qcom,wipower-default-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_pt,
			"qcom,wipower-pt-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-pt-ilim-map rc = %d\n",
				rc);
		goto err;
	}

	rc = smb_parse_wipower_map_dt(chip, &chip->wipower_div2,
			"qcom,wipower-div2-ilim-map");
	if (rc) {
		dev_err(chip->dev, "failed to parse wipower-div2-ilim-map rc = %d\n",
				rc);
		goto err;
	}
	chip->wipower_dyn_icl_avail = true;
	return 0;
err:
	chip->wipower_default.num = 0;
	chip->wipower_pt.num = 0;
	chip->wipower_default.num = 0;
	if (chip->wipower_default.entries)
		devm_kfree(chip->dev, chip->wipower_default.entries);
	if (chip->wipower_pt.entries)
		devm_kfree(chip->dev, chip->wipower_pt.entries);
	if (chip->wipower_div2.entries)
		devm_kfree(chip->dev, chip->wipower_div2.entries);
	chip->wipower_default.entries = NULL;
	chip->wipower_pt.entries = NULL;
	chip->wipower_div2.entries = NULL;
	chip->vadc_dev = NULL;
	return rc;
}

#define DEFAULT_VLED_MAX_UV		3500000
#define DEFAULT_FCC_MA			2000
static int smb_parse_dt(struct smbchg_chip *chip)
{
	int rc = 0, ocp_thresh = -EINVAL;
	struct device_node *node = chip->dev->of_node;
	const char *dc_psy_type, *bpd;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* read optional u32 properties */
	OF_PROP_READ(chip, ocp_thresh,
			"ibat-ocp-threshold-ua", rc, 1);
	if (ocp_thresh >= 0)
		smbchg_ibat_ocp_threshold_ua = ocp_thresh;
	OF_PROP_READ(chip, chip->iterm_ma, "iterm-ma", rc, 1);
	OF_PROP_READ(chip, chip->cfg_fastchg_current_ma,
			"fastchg-current-ma", rc, 1);
	if (chip->cfg_fastchg_current_ma == -EINVAL)
		chip->cfg_fastchg_current_ma = DEFAULT_FCC_MA;
	OF_PROP_READ(chip, chip->vfloat_mv, "float-voltage-mv", rc, 1);
	OF_PROP_READ(chip, chip->safety_time, "charging-timeout-mins", rc, 1);
	OF_PROP_READ(chip, chip->vled_max_uv, "vled-max-uv", rc, 1);
	if (chip->vled_max_uv < 0)
		chip->vled_max_uv = DEFAULT_VLED_MAX_UV;
	OF_PROP_READ(chip, chip->rpara_uohm, "rparasitic-uohm", rc, 1);
	if (chip->rpara_uohm < 0)
		chip->rpara_uohm = 0;
	OF_PROP_READ(chip, chip->prechg_safety_time, "precharging-timeout-mins",
			rc, 1);
	OF_PROP_READ(chip, chip->fastchg_current_comp, "fastchg-current-comp",
			rc, 1);
	OF_PROP_READ(chip, chip->float_voltage_comp, "float-voltage-comp",
			rc, 1);
	if (chip->safety_time != -EINVAL &&
			(chip->safety_time > chg_time[ARRAY_SIZE(chg_time) - 1])) {
		dev_err(chip->dev, "Bad charging-timeout-mins %d\n",
				chip->safety_time);
		return -EINVAL;
	}
	if (chip->prechg_safety_time != -EINVAL &&
			(chip->prechg_safety_time >
			 prechg_time[ARRAY_SIZE(prechg_time) - 1])) {
		dev_err(chip->dev, "Bad precharging-timeout-mins %d\n",
				chip->prechg_safety_time);
		return -EINVAL;
	}
	OF_PROP_READ(chip, chip->resume_delta_mv, "resume-delta-mv", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_current_thr_ma,
			"parallel-usb-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.min_9v_current_thr_ma,
			"parallel-usb-9v-min-current-ma", rc, 1);
	OF_PROP_READ(chip, chip->parallel.allowed_lowering_ma,
			"parallel-allowed-lowering-ma", rc, 1);
	if (chip->parallel.min_current_thr_ma != -EINVAL
			&& chip->parallel.min_9v_current_thr_ma != -EINVAL)
		chip->parallel.avail = true;
	/*
	 * use the dt values if they exist, otherwise do not touch the params
	 */
	of_property_read_u32(chip->spmi->dev.of_node,
					"qcom,parallel-main-chg-fcc-percent",
					&smbchg_main_chg_fcc_percent);
	of_property_read_u32(chip->spmi->dev.of_node,
					"qcom,parallel-main-chg-icl-percent",
					&smbchg_main_chg_icl_percent);
	pr_smb(PR_STATUS, "parallel usb thr: %d, 9v thr: %d\n",
			chip->parallel.min_current_thr_ma,
			chip->parallel.min_9v_current_thr_ma);
	OF_PROP_READ(chip, chip->jeita_temp_hard_limit,
			"jeita-temp-hard-limit", rc, 1);
	OF_PROP_READ(chip, chip->aicl_rerun_period_s,
			"aicl-rerun-period-s", rc, 1);
	OF_PROP_READ(chip, chip->vchg_adc_channel,
			"vchg-adc-channel-id", rc, 1);

	/* read boolean configuration properties */
	chip->use_vfloat_adjustments = of_property_read_bool(node,
			"qcom,autoadjust-vfloat");
	chip->bmd_algo_disabled = of_property_read_bool(node,
			"qcom,bmd-algo-disabled");
	chip->iterm_disabled = of_property_read_bool(node,
			"qcom,iterm-disabled");
	chip->soft_vfloat_comp_disabled = of_property_read_bool(node,
			"qcom,soft-vfloat-comp-disabled");
	chip->chg_enabled = !(of_property_read_bool(node,
				"qcom,charging-disabled"));
	chip->charge_unknown_battery = of_property_read_bool(node,
			"qcom,charge-unknown-battery");
	chip->chg_inhibit_en = of_property_read_bool(node,
			"qcom,chg-inhibit-en");
	chip->chg_inhibit_source_fg = of_property_read_bool(node,
			"qcom,chg-inhibit-fg");
	chip->low_volt_dcin = of_property_read_bool(node,
			"qcom,low-volt-dcin");
	chip->force_aicl_rerun = of_property_read_bool(node,
					"qcom,force-aicl-rerun");
	chip->skip_usb_suspend_for_fake_battery = of_property_read_bool(node,
				"qcom,skip-usb-suspend-for-fake-battery");

	/* parse the battery missing detection pin source */
	rc = of_property_read_string(chip->spmi->dev.of_node,
			"qcom,bmd-pin-src", &bpd);
	if (rc) {
		/* Select BAT_THM as default BPD scheme */
		chip->bmd_pin_src = BPD_TYPE_DEFAULT;
		rc = 0;
	} else {
		chip->bmd_pin_src = get_bpd(bpd);
		if (chip->bmd_pin_src < 0) {
			dev_err(chip->dev,
					"failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}

	/* parse the dc power supply configuration */
	rc = of_property_read_string(node, "qcom,dc-psy-type", &dc_psy_type);
	if (rc) {
		chip->dc_psy_type = -EINVAL;
		rc = 0;
	} else {
		if (strcmp(dc_psy_type, "Mains") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
		else if (strcmp(dc_psy_type, "Wireless") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
		else if (strcmp(dc_psy_type, "Wipower") == 0)
			chip->dc_psy_type = POWER_SUPPLY_TYPE_WIPOWER;
	}
	if (chip->dc_psy_type != -EINVAL) {
		OF_PROP_READ(chip, chip->dc_target_current_ma,
				"dc-psy-ma", rc, 0);
		if (rc)
			return rc;
		if (chip->dc_target_current_ma < DC_MA_MIN
				|| chip->dc_target_current_ma > DC_MA_MAX) {
			dev_err(chip->dev, "Bad dc mA %d\n",
					chip->dc_target_current_ma);
			return -EINVAL;
		}
	}

	if (chip->dc_psy_type == POWER_SUPPLY_TYPE_WIPOWER)
		smb_parse_wipower_dt(chip);

	/* read the bms power supply name */
	rc = of_property_read_string(node, "qcom,bms-psy-name",
			&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	/* read the battery power supply name */
	rc = of_property_read_string(node, "qcom,battery-psy-name",
			&chip->battery_psy_name);
	if (rc)
		chip->battery_psy_name = "battery";

	/* Get the charger led support property */
	chip->cfg_chg_led_sw_ctrl =
		of_property_read_bool(node, "qcom,chg-led-sw-controls");
	chip->cfg_chg_led_support =
		of_property_read_bool(node, "qcom,chg-led-support");

	if (of_find_property(node, "qcom,thermal-mitigation",
				&chip->thermal_levels)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
				chip->thermal_levels,
				GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			dev_err(chip->dev, "thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			dev_err(chip->dev,
					"Couldn't read threm limits rc = %d\n", rc);
			return rc;
		}
	}

	chip->skip_usb_notification
		= of_property_read_bool(node,
				"qcom,skip-usb-notification");

	chip->otg_pinctrl = of_property_read_bool(node, "qcom,otg-pinctrl");

	return 0;
}

#define SUBTYPE_REG			0x5
#define SMBCHG_CHGR_SUBTYPE		0x1
#define SMBCHG_OTG_SUBTYPE		0x8
#define SMBCHG_BAT_IF_SUBTYPE		0x3
#define SMBCHG_USB_CHGPTH_SUBTYPE	0x4
#define SMBCHG_DC_CHGPTH_SUBTYPE	0x5
#define SMBCHG_MISC_SUBTYPE		0x7
#define SMBCHG_LITE_CHGR_SUBTYPE	0x51
#define SMBCHG_LITE_OTG_SUBTYPE		0x58
#define SMBCHG_LITE_BAT_IF_SUBTYPE	0x53
#define SMBCHG_LITE_USB_CHGPTH_SUBTYPE	0x54
#define SMBCHG_LITE_DC_CHGPTH_SUBTYPE	0x55
#define SMBCHG_LITE_MISC_SUBTYPE	0x57
#define REQUEST_IRQ(chip, resource, irq_num, irq_name, irq_handler, flags, rc)\
	do {									\
		irq_num = spmi_get_irq_byname(chip->spmi,			\
				resource, irq_name);		\
		if (irq_num < 0) {						\
			dev_err(chip->dev, "Unable to get " irq_name " irq\n");	\
			return -ENXIO;						\
		}								\
		rc = devm_request_threaded_irq(chip->dev,			\
				irq_num, NULL, irq_handler, flags, irq_name,	\
				chip);						\
		if (rc < 0) {							\
			dev_err(chip->dev, "Unable to request " irq_name " irq: %d\n",\
					rc);					\
			return -ENXIO;						\
		}								\
	} while (0)

static int smbchg_request_irqs(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;
	unsigned long flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
							| IRQF_ONESHOT;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
		case SMBCHG_CHGR_SUBTYPE:
		case SMBCHG_LITE_CHGR_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->chg_error_irq,
				"chg-error", chg_error_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->taper_irq,
				"chg-taper-thr", taper_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			disable_irq_nosync(chip->taper_irq);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_term_irq,
				"chg-tcc-thr", chg_term_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource, chip->recharge_irq,
				"chg-rechg-thr", recharge_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->fastchg_irq,
				"chg-p2f-thr", fastchg_handler, flags, rc);
			enable_irq_wake(chip->chg_term_irq);
			enable_irq_wake(chip->chg_error_irq);
			enable_irq_wake(chip->fastchg_irq);
			break;
		case SMBCHG_BAT_IF_SUBTYPE:
		case SMBCHG_LITE_BAT_IF_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->batt_hot_irq,
				"batt-hot", batt_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_warm_irq,
				"batt-warm", batt_warm_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cool_irq,
				"batt-cool", batt_cool_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_cold_irq,
				"batt-cold", batt_cold_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->batt_missing_irq,
				"batt-missing", batt_pres_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->vbat_low_irq,
				"batt-low", vbat_low_handler, flags, rc);
			enable_irq_wake(chip->batt_hot_irq);
			enable_irq_wake(chip->batt_warm_irq);
			enable_irq_wake(chip->batt_cool_irq);
			enable_irq_wake(chip->batt_cold_irq);
			enable_irq_wake(chip->batt_missing_irq);
			enable_irq_wake(chip->vbat_low_irq);
			break;
		case SMBCHG_USB_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_uv_irq,
				"usbin-uv", usbin_uv_handler,
				flags | IRQF_EARLY_RESUME, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->usbin_ov_irq,
				"usbin-ov", usbin_ov_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->src_detect_irq,
				"usbin-src-det",
				src_detect_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->aicl_done_irq,
				"aicl-done",
				aicl_done_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
				rc);
			if (chip->schg_version != QPNP_SCHG_LITE) {
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_fail_irq, "otg-fail",
					otg_fail_handler, flags, rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->otg_oc_irq, "otg-oc",
					otg_oc_handler,
					(IRQF_TRIGGER_RISING | IRQF_ONESHOT),
					rc);
				REQUEST_IRQ(chip, spmi_resource,
					chip->usbid_change_irq, "usbid-change",
					usbid_change_handler,
					(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
					rc);
				enable_irq_wake(chip->otg_oc_irq);
				enable_irq_wake(chip->usbid_change_irq);
				enable_irq_wake(chip->otg_fail_irq);
			}
			enable_irq_wake(chip->usbin_uv_irq);
			enable_irq_wake(chip->usbin_ov_irq);
			enable_irq_wake(chip->src_detect_irq);
			if (chip->parallel.avail && chip->usb_present) {
				rc = enable_irq_wake(chip->aicl_done_irq);
				chip->enable_aicl_wake = true;
			}
			break;
		case SMBCHG_DC_CHGPTH_SUBTYPE:
		case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->dcin_uv_irq,
				"dcin-uv", dcin_uv_handler, flags, rc);
			enable_irq_wake(chip->dcin_uv_irq);
			break;
		case SMBCHG_MISC_SUBTYPE:
		case SMBCHG_LITE_MISC_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource, chip->power_ok_irq,
				"power-ok", power_ok_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource, chip->chg_hot_irq,
				"temp-shutdown", chg_hot_handler, flags, rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->wdog_timeout_irq,
				"wdog-timeout",
				wdog_timeout_handler, flags, rc);
			enable_irq_wake(chip->chg_hot_irq);
			enable_irq_wake(chip->wdog_timeout_irq);
			break;
		case SMBCHG_OTG_SUBTYPE:
			break;
		case SMBCHG_LITE_OTG_SUBTYPE:
			REQUEST_IRQ(chip, spmi_resource,
				chip->usbid_change_irq, "usbid-change",
				usbid_change_handler,
				(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
				rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_oc_irq, "otg-oc",
				otg_oc_handler,
				(IRQF_TRIGGER_RISING | IRQF_ONESHOT), rc);
			REQUEST_IRQ(chip, spmi_resource,
				chip->otg_fail_irq, "otg-fail",
				otg_fail_handler, flags, rc);
			enable_irq_wake(chip->usbid_change_irq);
			enable_irq_wake(chip->otg_oc_irq);
			enable_irq_wake(chip->otg_fail_irq);
			break;
		}
	}

	return rc;
}

#define REQUIRE_BASE(chip, base, rc)					\
	do {									\
		if (!rc && !chip->base) {					\
			dev_err(chip->dev, "Missing " #base "\n");		\
			rc = -EINVAL;						\
		}								\
	} while (0)

static int smbchg_parse_peripherals(struct smbchg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
			dev_err(chip->dev, "spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
				IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			dev_err(chip->dev, "node %s IO resource absent!\n",
					spmi->dev.of_node->full_name);
			return rc;
		}

		rc = smbchg_read(chip, &subtype,
				resource->start + SUBTYPE_REG, 1);
		if (rc) {
			dev_err(chip->dev, "Peripheral subtype read failed rc=%d\n",
					rc);
			return rc;
		}

		switch (subtype) {
			case SMBCHG_CHGR_SUBTYPE:
			case SMBCHG_LITE_CHGR_SUBTYPE:
				chip->chgr_base = resource->start;
				break;
			case SMBCHG_BAT_IF_SUBTYPE:
			case SMBCHG_LITE_BAT_IF_SUBTYPE:
				chip->bat_if_base = resource->start;
				break;
			case SMBCHG_USB_CHGPTH_SUBTYPE:
			case SMBCHG_LITE_USB_CHGPTH_SUBTYPE:
				chip->usb_chgpth_base = resource->start;
				break;
			case SMBCHG_DC_CHGPTH_SUBTYPE:
			case SMBCHG_LITE_DC_CHGPTH_SUBTYPE:
				chip->dc_chgpth_base = resource->start;
				break;
			case SMBCHG_MISC_SUBTYPE:
			case SMBCHG_LITE_MISC_SUBTYPE:
				chip->misc_base = resource->start;
				break;
			case SMBCHG_OTG_SUBTYPE:
			case SMBCHG_LITE_OTG_SUBTYPE:
				chip->otg_base = resource->start;
				break;
		}
	}

	REQUIRE_BASE(chip, chgr_base, rc);
	REQUIRE_BASE(chip, bat_if_base, rc);
	REQUIRE_BASE(chip, usb_chgpth_base, rc);
	REQUIRE_BASE(chip, dc_chgpth_base, rc);
	REQUIRE_BASE(chip, misc_base, rc);

	return rc;
}

static inline void dump_reg(struct smbchg_chip *chip, u16 addr,
		const char *name)
{
	u8 reg;

	smbchg_read(chip, &reg, addr, 1);
	pr_smb(PR_DUMP, "%s - %04X = %02X\n", name, addr, reg);
}

/* dumps useful registers for debug */
static void dump_regs(struct smbchg_chip *chip)
{
	u16 addr;

	/* charger peripheral */
	for (addr = 0xB; addr <= 0x10; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Status");
	for (addr = 0xF0; addr <= 0xFF; addr++)
		dump_reg(chip, chip->chgr_base + addr, "CHGR Config");
	/* battery interface peripheral */
	dump_reg(chip, chip->bat_if_base + RT_STS, "BAT_IF Status");
	dump_reg(chip, chip->bat_if_base + CMD_CHG_REG, "BAT_IF Command");
	for (addr = 0xF0; addr <= 0xFB; addr++)
		dump_reg(chip, chip->bat_if_base + addr, "BAT_IF Config");
	/* usb charge path peripheral */
	for (addr = 0x7; addr <= 0x10; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Status");
	dump_reg(chip, chip->usb_chgpth_base + CMD_IL, "USB Command");
	for (addr = 0xF0; addr <= 0xF5; addr++)
		dump_reg(chip, chip->usb_chgpth_base + addr, "USB Config");
	/* dc charge path peripheral */
	dump_reg(chip, chip->dc_chgpth_base + RT_STS, "DC Status");
	for (addr = 0xF0; addr <= 0xF6; addr++)
		dump_reg(chip, chip->dc_chgpth_base + addr, "DC Config");
	/* misc peripheral */
	dump_reg(chip, chip->misc_base + IDEV_STS, "MISC Status");
	dump_reg(chip, chip->misc_base + RT_STS, "MISC Status");
	for (addr = 0xF0; addr <= 0xF3; addr++)
		dump_reg(chip, chip->misc_base + addr, "MISC CFG");
}

static int create_debugfs_entries(struct smbchg_chip *chip)
{
	struct dentry *ent;

	chip->debug_root = debugfs_create_dir("qpnp-smbcharger", NULL);
	if (!chip->debug_root) {
		dev_err(chip->dev, "Couldn't create debug dir\n");
		return -EINVAL;
	}

	ent = debugfs_create_file("force_dcin_icl_check",
			S_IFREG | S_IWUSR | S_IRUGO,
			chip->debug_root, chip,
			&force_dcin_icl_ops);
	if (!ent) {
		dev_err(chip->dev,
				"Couldn't create force dcin icl check file\n");
		return -EINVAL;
	}

	return 0;
}

static chg_cv_battery_temp_region_type qpnp_battery_temp_region_get(struct smbchg_chip *chip)
{
	return chip->mBatteryTempRegion;
}

static void qpnp_battery_temp_region_set(struct smbchg_chip *chip,
		chg_cv_battery_temp_region_type batt_temp_region)
{
	chip->mBatteryTempRegion = batt_temp_region;
}

int get_charging_status(void)
{
	if (!g_chip)
		return POWER_SUPPLY_STATUS_DISCHARGING;

	return get_prop_batt_status(g_chip);
}

int fuelgauge_battery_temp_region_get(void)
{
	if (!g_chip)
		return CV_BATTERY_TEMP_REGION__NORMAL;
	return qpnp_battery_temp_region_get(g_chip);
}

static int set_prop_batt_health(struct smbchg_chip *chip, int batt_health)
{
	chip->batt_health = batt_health;
	return 0;
}


#if 0
	static int
get_prop_vph_voltage_now(struct smbchg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	if (chip->revision == 0 ) {
		pr_err("VSYS reading not supported for 1.0 rc=%d\n", rc);
		return 0;
	} else {
		rc = qpnp_vadc_read(chip->vadc_dev_pm8994, VSYS, &results);
		if (rc) {
			pr_err("Unable to read VSYS rc=%d\n", rc);
			return 0;
		}
		return (int)results.physical/1000;
	}
}
#endif

static void rerun_hvdcp_det_if_necessary(struct smbchg_chip *chip)
{
	enum power_supply_type usb_supply_type;
	char *usb_type_name;
	int rc;

	if (!(chip->wa_flags & SMBCHG_RESTART_WA))
		return;

	if (!is_usb_present(chip) || chip->hvdcp_not_supported)
		return;

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if (usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP
		&& !is_hvdcp_present(chip)) {
		pr_smb(PR_STATUS, "DCP found rerunning APSD\n");
		rc = vote(chip->usb_icl_votable,
				CHG_SUSPEND_WORKAROUND_ICL_VOTER, true, 300);
		if (rc < 0)
			pr_err("Couldn't vote for 300mA for suspend wa, going ahead rc=%d\n",
					rc);

		pr_smb(PR_STATUS, "Faking Removal\n");
		fake_insertion_removal(chip, false);
		msleep(500);
		pr_smb(PR_STATUS, "Faking Insertion\n");
		fake_insertion_removal(chip, true);

		read_usb_type(chip, &usb_type_name, &usb_supply_type);
		if (usb_supply_type != POWER_SUPPLY_TYPE_USB_DCP) {
			msleep(500);
			pr_smb(PR_STATUS, "Fake Removal again as type!=DCP\n");
			fake_insertion_removal(chip, false);
			msleep(500);
			pr_smb(PR_STATUS, "Fake Insert again as type!=DCP\n");
			fake_insertion_removal(chip, true);
		}

		rc = vote(chip->usb_icl_votable,
				CHG_SUSPEND_WORKAROUND_ICL_VOTER, false, 0);
		if (rc < 0)
			pr_err("Couldn't vote for 0 for suspend wa, going ahead rc=%d\n",
					rc);
	}
}

static int qpnp_start_charging(struct smbchg_chip *chip)
{
	int rc = -1;
	int batt_temp = get_prop_batt_temp(chip);
	bool charger_present;
	u8 reg;
	int type;
	charger_present = is_usb_present(chip) | is_dc_present(chip);
	pr_err("%s:starting to enable charging\n", __func__);

	if (!charger_present){
		pr_err("%s:charger maybe removed \n", __func__);
		return rc;
	}

	rc = smbchg_read(chip, &reg, chip->misc_base + IDEV_STS, 1);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read status 5 rc = %d\n", rc);
		return false;
	}
	type = get_type(reg);
	if (batt_temp <= chip->mBatteryTempBoundT0){   // -3
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__COLD);
		smbchg_charging_en(chip, false);
		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_COLD);
		return rc;
	}else if (batt_temp <= chip->mBatteryTempBoundT1){ // -3 ~ 0
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION_LITTLE__COLD);
		set_chg_parameter_in_different_temp(chip, chip->temp_more_cool_current,chip->temp_more_cool_vbatdel );
	}else if (batt_temp <= chip->mBatteryTempBoundT2){ // 0 ~ 10
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__COOL);
		set_chg_parameter_in_different_temp(chip, chip->temp_cool_current,chip->temp_cool_vbatdel );
	} else if(batt_temp <= chip->mBatteryTempBoundT3){	//10-20
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__LITTLE_COOL);
		set_chg_parameter_in_different_temp(chip, chip->temp_littel_cool_current,chip->temp_littel_cool_vbatdel );
	} else if (batt_temp <= chip->mBatteryTempBoundT5){ // 20 ~ 45
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__NORMAL);
		if (chip->hvdcp_present)
			set_chg_parameter_in_different_temp(chip, chip->temp_normal_current_hvdcp,chip->temp_normal_vbatdel );
		else
			set_chg_parameter_in_different_temp(chip, chip->temp_normal_current,chip->temp_normal_vbatdel );
	}else if (batt_temp <= chip->mBatteryTempBoundT6){  // 45 ~ 55
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__WARM);
		if (chip->hvdcp_present)
			set_chg_parameter_in_different_temp(chip, chip->temp_warm_current_hvdcp,chip->temp_warm_vbatdel );
		else
			set_chg_parameter_in_different_temp(chip, chip->temp_warm_current,chip->temp_warm_vbatdel );
	}else{
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__HOT);
		smbchg_charging_en(chip, false);
		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_OVERHEAT);
		return rc;
	}
	smbchg_charging_en(chip, true);
	return rc;
}
static void qpnp_check_charge_timeout(struct smbchg_chip *chip)
{
	static int count = 0;
	union power_supply_propval ret = {0,};
	if (chip->chg_done)
		return;

	if((is_usb_present(chip) | is_dc_present(chip))&& (POWER_SUPPLY_STATUS_CHARGING== get_prop_batt_status(chip)))//usb plug in && charging
		count++;
	else
		count = 0;

	chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &ret);

	if(((ret.intval / 1000 != 500) && count > BATT_CHG_TIMEOUT_COUNT_DCP)
			||((ret.intval / 1000 == 500) && count > BATT_CHG_TIMEOUT_COUNT_USB_PRO)){
		pr_err("%s:chg timeout stop chaging\n", __func__);
		smbchg_charging_en(chip, false);
		chip->time_out = true;
	}
}
static enum chg_battery_status_type qpnp_battery_status_get(struct smbchg_chip *chip)
{
	return chip->battery_status;
}
static void qpnp_battery_status_set(struct smbchg_chip *chip,
		enum chg_battery_status_type battery_status)
{
	chip->battery_status = battery_status;
}
static int qpnp_handle_battery_uovp(struct smbchg_chip *chip)
{
	pr_info("%s\n", __func__);
	set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_OVERVOLTAGE);
	smbchg_charging_en(chip, false);
	return 0;
}

static int qpnp_handle_battery_restore_from_uovp(struct smbchg_chip *chip)
{
	pr_info("%s\n", __func__);
	/*restore charging form battery ovp*/
	smbchg_charging_en(chip, true);

	set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);

	return 0;
}

static void qpnp_check_battery_uovp(struct smbchg_chip *chip)
{
	int battery_voltage=0;
	enum chg_battery_status_type battery_status_pre;

	if (!(is_usb_present(chip) | is_dc_present(chip)))
		return;
	battery_status_pre = qpnp_battery_status_get(chip);

	battery_voltage = get_prop_batt_voltage_now(chip);
	pr_debug("%s bat vol:%d\n", __func__, battery_voltage);
	if(battery_voltage > BATTERY_SOFT_OVP_VOLTAGE) {
		if (battery_status_pre == BATTERY_STATUS_GOOD) {

			pr_err("%s BATTERY_SOFT_OVP_VOLTAGE\n", __func__);
			qpnp_battery_status_set(chip, BATTERY_STATUS_BAD);
			qpnp_handle_battery_uovp(chip);
		}
	}
	else {
		if (battery_status_pre == BATTERY_STATUS_BAD) {

			pr_err("%s battery_restore_from_uovp\n", __func__);
			qpnp_battery_status_set(chip, BATTERY_STATUS_GOOD);
			qpnp_handle_battery_restore_from_uovp(chip);
			smbchg_rerun_aicl(chip);
		}
	}

	return;
}

static void qpnp_check_charger_uovp(struct smbchg_chip *chip)
{
	int vchg_mv = CHARGER_VOLTAGE_NORMAL;
	bool charger_present;
	static  int over_volt_cout=0,not_over_volt_cout=0;
	static bool uovp_satus,pre_uovp_satus;
	int detect_time = 3;// 3*6s=18s
	charger_present = is_usb_present(chip) | is_dc_present(chip);
	if (!charger_present) {
		return;
	}

	vchg_mv = get_prop_charger_voltage_now(chip);

	pr_debug("%s %d %d\n", __func__, vchg_mv, chip->charger_status);

	if(chip->charger_status == CHARGER_STATUS_GOOD) {
		if(vchg_mv > CHARGER_SOFT_OVP_VOLTAGE ||
				vchg_mv <= CHARGER_SOFT_UVP_VOLTAGE) {
			pr_err("charger over voltage\n");
			uovp_satus = true;
			if(pre_uovp_satus == true)
				over_volt_cout = over_volt_cout+1;
			else
				over_volt_cout =0;

			pr_err("uovp_satus=%d, pre_uovp_satus=%d,over_volt_cout=%d\n",uovp_satus,pre_uovp_satus,over_volt_cout);
			if(detect_time <= over_volt_cout)//vchg continuous higher than 5.8v
			{

				pr_err("charger over voltage true\n");
				smbchg_charging_en(chip, false);

				chip->charger_status = CHARGER_STATUS_OVER;
			}
		}
	}else if(chip->charger_status == CHARGER_STATUS_OVER){

		pr_debug("charger not over voltage\n");
		if(vchg_mv < (CHARGER_SOFT_OVP_VOLTAGE - 100) &&
				vchg_mv > (CHARGER_SOFT_UVP_VOLTAGE + 100)) {

			uovp_satus = false;
			if(pre_uovp_satus == false)
				not_over_volt_cout = not_over_volt_cout+1;
			else
				not_over_volt_cout =0;

			pr_err("uovp_satus=%d, pre_uovp_satus=%d,not_over_volt_cout=%d\n",uovp_satus,pre_uovp_satus,not_over_volt_cout);
			if(detect_time <= not_over_volt_cout)//vchg continuous lower than 5.7v
			{

				pr_err("charger not over voltage true \n");
				smbchg_charging_en(chip, true);
				qpnp_start_charging(chip);
				chip->charger_status = CHARGER_STATUS_GOOD;
				smbchg_rerun_aicl(chip);
			}
		}
	}
	pre_uovp_satus =uovp_satus;
	return ;
}

void set_chg_parameter_in_different_temp(struct smbchg_chip *chip,int ibat,int vfloat )
{
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	pr_err("%s:inserted type = %d (%s),charger_type_value=%d\n",__func__, usb_supply_type, usb_type_name,charger_type_value);
	usb_supply_type =charger_type_value;

	if(usb_supply_type == 1 || usb_supply_type == 4){//pc usb:cdp or sdp
		vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,ibat);
	}
	else
	{//not cdp or sdp
		vote(chip->fcc_votable, BATT_TYPE_FCC_VOTER, true,ibat);
	}
	smbchg_float_voltage_set(chip,vfloat);
	set_property_on_fg(chip, POWER_SUPPLY_PROP_CC_TO_CV_POINT, vfloat-100);//set cc to cv 100mv lower than vfloat

}


/*Tbatt <-3C*/
static int handle_batt_temp_cold(struct smbchg_chip *chip)
{
	if ((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__COLD)||( chip->is_power_changed== true))
	{
		pr_debug("%s\n", __func__);
		chip->is_power_changed=false;

		smbchg_charging_en(chip, false);

		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__COLD);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COLD_TO_COOL;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1 ;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2 ;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3 ;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;
		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_COLD);
	}
	return 0;
}

/* -3 C <=Tbatt <= 0C*/
static int handle_batt_temp_little_cold(struct smbchg_chip *chip)
{
	if(chip->charger_status == CHARGER_STATUS_OVER)
		return 0;

	if ((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION_LITTLE__COLD)||( chip->is_power_changed== true)||(chip->recharge_pending==true))
	{
		pr_debug("%s\n", __func__);
		chip->recharge_pending=false;
		chip->is_power_changed=false;

		if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__HOT ||
				qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COLD)
			smbchg_charging_en(chip, true);
		set_chg_parameter_in_different_temp(chip, chip->temp_more_cool_current,chip->temp_more_cool_vbatdel );
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION_LITTLE__COLD);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;


		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);

	}
	return 0;
}

/* 0 C <Tbatt <= 5C*/
static int handle_batt_temp_cool(struct smbchg_chip *chip)
{
	if(chip->charger_status == CHARGER_STATUS_OVER)
		return 0;

	if ((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__COOL)||( chip->is_power_changed== true)||(chip->recharge_pending==true))
	{
		pr_debug("%s\n", __func__);

		chip->recharge_pending=false;
		chip->is_power_changed = false;
		if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__HOT ||
				qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COLD)
			smbchg_charging_en(chip, true);

		set_chg_parameter_in_different_temp(chip, chip->temp_cool_current,chip->temp_cool_vbatdel );
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__COOL);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1 ;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);

	}
	return 0;
}
/* 5 C <Tbatt <= 12C*/
static int handle_batt_temp_little_cool(struct smbchg_chip *chip)
{
	if(chip->charger_status == CHARGER_STATUS_OVER)
		return 0;

	if ((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__LITTLE_COOL)||( chip->is_power_changed== true)||(chip->recharge_pending==true))
	{
		pr_debug("%s\n", __func__);

		chip->recharge_pending=false;
		chip->is_power_changed = false;
		if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__HOT ||
				qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COLD)
			smbchg_charging_en(chip, true);
		if(get_prop_batt_voltage_now(chip)> 4180*1000)
			{
			set_chg_parameter_in_different_temp(chip,450,chip->temp_littel_cool_vbatdel );
			chip->temp_littel_cool_set_current_0_point_25c =false;
			}else
			{
			set_chg_parameter_in_different_temp(chip, chip->temp_littel_cool_current,chip->temp_littel_cool_vbatdel );
			 chip->temp_littel_cool_set_current_0_point_25c =true;
			}
		/* Update battery temp region */
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__LITTLE_COOL);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2 ;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3 + AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;

		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);

	}
	return 0;
}
/* 12 C <Tbatt <22*/
static int handle_batt_temp_prenormal(struct smbchg_chip *chip)
{
	if(chip->charger_status == CHARGER_STATUS_OVER)
		return 0;

	if ((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__PRE_NORMAL)||( chip->is_power_changed== true)||(chip->recharge_pending==true))
	{
		pr_debug("%s\n", __func__);

		chip->recharge_pending=false;
		chip->is_power_changed = false;
		if (qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__HOT ||
				qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COLD)
			smbchg_charging_en(chip, true);
		if (chip->hvdcp_present)
			set_chg_parameter_in_different_temp(chip, chip->temp_normal_current_hvdcp,chip->temp_prenormal_vbatdel );
		else
			set_chg_parameter_in_different_temp(chip, chip->temp_prenormal_current,chip->temp_prenormal_vbatdel );
		/* Update battery temp region */
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__PRE_NORMAL);
		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4+AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_COOL_TO_NORMAL;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;
		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);
	}
	return 0;
}

/* 15 C <Tbatt <45C*/
static int handle_batt_temp_normal(struct smbchg_chip *chip)
{
	if(chip->charger_status == CHARGER_STATUS_OVER)
		return 0;

	if ((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__NORMAL)||( chip->is_power_changed== true)||(chip->recharge_pending==true))
	{
		pr_debug("%s\n", __func__);

		chip->recharge_pending=false;
		chip->is_power_changed = false;
		if (qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__HOT ||
				qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COLD)
			smbchg_charging_en(chip, true);
		if (chip->hvdcp_present)
			set_chg_parameter_in_different_temp(chip, chip->temp_normal_current_hvdcp,chip->temp_normal_vbatdel );
		else
			set_chg_parameter_in_different_temp(chip, chip->temp_normal_current,chip->temp_normal_vbatdel );
		/* Update battery temp region */
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__NORMAL);
		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;
		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);
	}
	return 0;
}

/* 45C <=Tbatt <=55C*/
static int handle_batt_temp_warm(struct smbchg_chip *chip)
{
	if(chip->charger_status == CHARGER_STATUS_OVER)
		return 0;

	if((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__WARM)||( chip->is_power_changed== true)||(chip->recharge_pending==true))
	{
		chip->is_power_changed = false;
		chip->recharge_pending=false;
		pr_debug("%s\n", __func__);
		if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__HOT ||
				qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COLD)
			smbchg_charging_en(chip, true);
		if (chip->hvdcp_present)
			set_chg_parameter_in_different_temp(chip, chip->temp_warm_current_hvdcp,chip->temp_warm_vbatdel );
		else
			set_chg_parameter_in_different_temp(chip, chip->temp_warm_current,chip->temp_warm_vbatdel );
		/* Update battery temp region */
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__WARM);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5 - AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_WARM_TO_NORMAL;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;
		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_GOOD);
	}
	return 0;
}

/* 55C <Tbatt*/
static int handle_batt_temp_hot(struct smbchg_chip *chip)
{
	if((qpnp_battery_temp_region_get(chip) != CV_BATTERY_TEMP_REGION__HOT)||( chip->is_power_changed== true))
	{
		chip->is_power_changed = false;
		pr_debug("%s\n", __func__);
		smbchg_charging_en(chip, false);

		/* Update battery temp region */
		qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__HOT);

		/* Update the temperature boundaries */
		chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
		chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
		chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
		chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
		chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
		chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
		chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6 - AUTO_CHARGING_BATTERY_TEMP_HYST_FROM_HOT_TO_WARM;
		set_prop_batt_health(chip, POWER_SUPPLY_HEALTH_OVERHEAT);
	}
	return 0;
}

static int qpnp_check_battery_temp(struct smbchg_chip *chip)
{
	int rc = -1;
	int temperature = 0;
	bool charger_present;
	charger_present = is_usb_present(chip) | is_dc_present(chip);
	pr_debug("%s charger_present:%d\n", __func__, charger_present);

	if (!charger_present)
		return rc;
	temperature = get_prop_batt_temp(chip);
	pr_debug("%s temp:%d\n", __func__, temperature);

	if(temperature < chip->mBatteryTempBoundT0) /* battery is cold */
	{
		rc = handle_batt_temp_cold(chip);
	}
	else if( (temperature >=  chip->mBatteryTempBoundT0) &&
			(temperature < chip->mBatteryTempBoundT1) ) /* battery is more cool */
	{
		rc = handle_batt_temp_little_cold(chip);
	}
	else if( (temperature >=  chip->mBatteryTempBoundT1) &&
			(temperature < chip->mBatteryTempBoundT2) ) /* battery is cool */
	{
		rc = handle_batt_temp_cool(chip);
	}
	else if( (temperature >= chip->mBatteryTempBoundT2) &&
			(temperature < chip->mBatteryTempBoundT3) ) /* battery is little cool */
	{
		rc = handle_batt_temp_little_cool(chip);
	}
	else if( (temperature >= chip->mBatteryTempBoundT3) &&
			(temperature < chip->mBatteryTempBoundT4) ) /* battery is prenormal */
	{
		rc = handle_batt_temp_prenormal(chip);
	}
	else if( (temperature >= chip->mBatteryTempBoundT4) &&
			(temperature < chip->mBatteryTempBoundT5) ) /* battery is normal */
	{
		rc = handle_batt_temp_normal(chip);
	}
	else if( (temperature >= chip->mBatteryTempBoundT5) &&
			(temperature <=  chip->mBatteryTempBoundT6) ) /* battery is warm */
	{
		rc = handle_batt_temp_warm(chip);
	}
	else if(temperature > chip->mBatteryTempBoundT6)/* battery is hot */
	{
		rc = handle_batt_temp_hot(chip);
	}

	return rc;
}
static void qpnp_charge_info_init(struct smbchg_chip *chip)
{


	qpnp_battery_temp_region_set(chip, CV_BATTERY_TEMP_REGION__NORMAL);

	chip->mBatteryTempBoundT0 = AUTO_CHARGING_BATT_TEMP_T0;
	chip->mBatteryTempBoundT1 = AUTO_CHARGING_BATT_TEMP_T1;
	chip->mBatteryTempBoundT2 = AUTO_CHARGING_BATT_TEMP_T2;
	chip->mBatteryTempBoundT3 = AUTO_CHARGING_BATT_TEMP_T3;
	chip->mBatteryTempBoundT4 = AUTO_CHARGING_BATT_TEMP_T4;
	chip->mBatteryTempBoundT5 = AUTO_CHARGING_BATT_TEMP_T5;
	chip->mBatteryTempBoundT6 = AUTO_CHARGING_BATT_TEMP_T6;
	chip->charger_status = CHARGER_STATUS_GOOD;
	chip->is_power_changed =false;
	chip->chg_done=false;
	chip->recharge_pending=false;
	chip->recharge_status=false;

	chip->temp_more_cool_rechgvbat=3700;   // -3 <= T < 0
	chip->temp_more_cool_vbatdel=3980;   // -3 <= T < 0
	chip->temp_cool_vbatdel=4320;        //0 <= T < 5
	chip->temp_cool_recharge_vbatdel=4100;
	chip->temp_littel_cool_vbatdel=4320; // 5 <= T < 12
	chip->temp_littel_cool_recharge_vbatdel=4220;
	chip->temp_prenormal_vbatdel=4320;      //12 <= T < = 16
	chip->temp_prenormal_recharge_vbatdel=4220;
	chip->temp_normal_vbatdel=4320;      //16<= T < = 45
	chip->temp_normal_recharge_vbatdel=4220;
	chip->temp_warm_vbatdel = 4080;    //45<= T < 53
	chip->temp_warm_recharge_vbatdel=4000;

	chip->temp_more_cool_current = 300;   //   Vterm=4.0V		I=300 ,  -3 <= T < 0
	chip->temp_cool_current	   = 400;	 //    Vterm=4.2V		I=0.3C , 0 <= T < 5
	chip->temp_littel_cool_current = 700;  // Vterm=4.32V	I=0.5C ,  5 <= T < 12
	chip->temp_prenormal_current 	 = 1200;  //   Vterm=4.32V		I=0.4C ,  12 <= T < = 16
	chip->temp_normal_current 	 = 1910;  //   Vterm=4.3V		I=0.6C ,  16 <= T < = 45
	chip->temp_warm_current		 =700;  //    Vterm=4.0V		I=0.5c , 43 <= T < 53
	chip->temp_normal_current_hvdcp	= 3000;  //   Vterm=4.3V	I=1.5C/2C ,  22 <= T < = 43
	chip->temp_warm_current_hvdcp	= 1200;  //    Vterm=4.0V	I=0.4c , 43 <= T < 53
	chip->lcd_on_iusb = 3100;
	chip->typeC_support_current_ma=4000;//default value set 4000MA, will not limmit Iusb
	chip->temp_littel_cool_set_current_0_point_25c =false;
	chip->oem_lcd_is_on =false;
	chip->time_out = false;
	chip->battery_status=BATTERY_STATUS_GOOD;
	chip->fastchg_start_disable_normal_chg =false;
	chip->usb_enum_status = false;
	chip->nonstandard_charger = false;
}
static int typeC_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct smbchg_chip *chip =
	container_of(self, struct smbchg_chip, typec_notif);

	if(event == TYPE_C_HOST_CUR_1500)
	{
		chip->typeC_support_current_ma =1500;//will limmit Iusb 1500MA
	}
	else
	{
		chip->typeC_support_current_ma =4000;
	}
	return 0;
}

#if defined(CONFIG_FB)
/* yangfangbiao@oneplus.cn,20150519  Add for reset charge current when screen is off */
static int fb_notifier_callback(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct smbchg_chip *chip =
		container_of(self, struct smbchg_chip, fb_notif);

	if (evdata && evdata->data && chip) {
		if (event == FB_EVENT_BLANK) {
			blank = evdata->data;
			if (*blank == FB_BLANK_UNBLANK) {
				if(chip->oem_lcd_is_on != true)
				set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_LCD_IS_OFF, 0);
				chip->oem_lcd_is_on =true ;
			}

			else if (*blank == FB_BLANK_POWERDOWN) {
				if(chip->oem_lcd_is_on != false)
				set_property_on_fg(chip, POWER_SUPPLY_PROP_UPDATE_LCD_IS_OFF, 1);
				chip->oem_lcd_is_on =false;
			}
		}

	}

	return 0;
}
#endif /*CONFIG_FB*/
#define SOFT_CHG_TERM_CURRENT 100	//100MA
void checkout_term_current(struct smbchg_chip *chip,int batt_temp)
{
	static int term_current_reached=0;
	int current_ma,voltage_mv;
	if(POWER_SUPPLY_STATUS_CHARGING != get_prop_batt_status(chip))
	{
		return ;
	}
	current_ma = get_prop_batt_current_now(chip)/1000;
	if(((current_ma >= (-SOFT_CHG_TERM_CURRENT))&&(current_ma <= SOFT_CHG_TERM_CURRENT)) != true)//soft charge term set to  to 100ma
	{
		term_current_reached=0;
		return ;
	}
	voltage_mv= get_prop_batt_voltage_now(chip)/1000;
	if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION_LITTLE__COLD)
		{
			if(voltage_mv >= chip->temp_more_cool_vbatdel)
			{
			term_current_reached++;
			}
			else
			{
			term_current_reached=0;
			return;
			}
		}
		else if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__COOL)
		{
			if(voltage_mv >=chip->temp_cool_vbatdel)
			{
			term_current_reached++;
			}
			else
			{
			term_current_reached=0;
			return;
			}
		}
		else if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__LITTLE_COOL)
		{
			if(voltage_mv >= chip->temp_littel_cool_vbatdel)
			{
			term_current_reached++;
			}
			else
			{
			term_current_reached=0;
			return;
			}
		}
		else if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__PRE_NORMAL)
		{
			if(voltage_mv >= chip->temp_prenormal_vbatdel)
			{
			term_current_reached++;
			}
			else
			{
			term_current_reached=0;
			return;
			}
		}
		else if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__NORMAL)
		{
			if(voltage_mv >= chip->temp_normal_vbatdel)
			{
			term_current_reached++;
			}
			else
			{
			term_current_reached=0;
			return;
			}
		}
		else if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__WARM)
		{
			if(voltage_mv >= chip->temp_warm_vbatdel)
			{
			term_current_reached++;
			}
			else
			{
			term_current_reached=0;
			return;
			}
		}
	if(term_current_reached >= 5)
	{
		smbchg_charging_status_change(chip);
		set_property_on_fg(chip, POWER_SUPPLY_PROP_CHARGE_DONE, 1);
		chip->chg_done =true;
		term_current_reached=0;
		pr_err("checkout_term_current: chip->chg_done:temp:%d,soc_calib:%d,VOLT:%d,current:%d\n",
			get_prop_batt_temp(chip),get_prop_batt_capacity(chip),get_prop_batt_voltage_now(chip),get_prop_batt_current_now(chip));
		smbchg_charging_en(chip, 0);
	}
}
static void check_allow_switch_dash(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
		struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip, check_switch_dash_work);
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
	bool charger_present;
	charger_present = is_usb_present(chip) | is_dc_present(chip);
	if (!charger_present) {
		return;
	}
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if((usb_supply_type == POWER_SUPPLY_TYPE_USB_DCP && chip->redect_charger_type_done ==true ) || (chip->nonstandard_charger == true)) {
				switch_fast_chg(chip);
			}
}


static void check_nonstandard_charger_charge_current(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
		struct smbchg_chip *chip = container_of(dwork,
				struct smbchg_chip, nonstandard_charger_charge_current_set_work);
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
	int aicl_limited_current;

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if (((usb_supply_type == POWER_SUPPLY_TYPE_USB) ||(usb_supply_type == POWER_SUPPLY_TYPE_USB_CDP) )
		&& (chip->usb_enum_status !=true) )
	{
		smbchg_rerun_aicl(chip);
		msleep(50);
		/* use override for  non standard charger*/
		if(smbchg_masked_write(chip,chip->usb_chgpth_base + CMD_IL,ICL_OVERRIDE_BIT, ICL_OVERRIDE_BIT) < 0)
		{
		pr_err("Couldn't set override\n");
		}
		aicl_limited_current = smbchg_get_aicl_level_ma(chip);
		if(aicl_limited_current > 0)
		{
		if (smbchg_set_high_usb_chg_current(chip, aicl_limited_current) < 0)
			pr_err("Couldn't set %dmA \n", aicl_limited_current);
		}
		power_supply_set_supply_type(chip->usb_psy, POWER_SUPPLY_TYPE_USB_DCP);
		power_supply_set_current_limit(chip->usb_psy,CURRENT_1500_MA * 1000);
		power_supply_changed(&chip->batt_psy);
		pr_err("%s nonstandard_charger,fast chg not started,GPIO135:%d\n",__func__,gpio_get_value(135));
		chip->nonstandard_charger = true;
	}
}

static void update_heartbeat(struct work_struct *work)
{

	struct delayed_work *dwork = to_delayed_work(work);
	struct smbchg_chip *chip = container_of(dwork,
			struct smbchg_chip, update_heartbeat_work);
	static int batt_temp=0;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";
	bool charger_present;
	qpnp_check_charge_timeout(chip);
	charger_present = is_usb_present(chip) | is_dc_present(chip);
	if (!charger_present) {
		goto out;
	}
	/* usb inserted */
	power_supply_changed(&chip->batt_psy);
	if(get_prop_fast_chg_started(chip) == true) {

		switch_fast_chg(chip);
		pr_debug("%s fast chg started,GPIO135:%d\n", __func__,gpio_get_value(135));
		//add for disable normal charge begin
		if( qpnp_get_fast_chg_ing(chip) == 1){
			if(chip->fastchg_start_disable_normal_chg ==false)
			smbchg_charging_en(chip, false);
			chip->fastchg_start_disable_normal_chg =true;
		}
		// add for disable normal charge end
		/*update time 6s*/
		schedule_delayed_work(&chip->update_heartbeat_work,
				round_jiffies_relative(msecs_to_jiffies
					(BATT_HEARTBEAT_INTERVAL)));
		return;
	}
	else{
		if(chip->fastchg_start_disable_normal_chg == true)
		{
			chip->fastchg_start_disable_normal_chg =false;
			smbchg_charging_en(chip, true);
		}
		schedule_delayed_work(&chip->check_switch_dash_work,
							msecs_to_jiffies(100));
	}
	if (!chip->hvdcp_present)
		qpnp_check_charger_uovp(chip);
	qpnp_check_battery_uovp(chip);

	if(qpnp_battery_temp_region_get(chip) == CV_BATTERY_TEMP_REGION__LITTLE_COOL )
		{
		if((get_prop_batt_voltage_now(chip)/1000 > (4180 + 20))
				&& (chip->temp_littel_cool_set_current_0_point_25c ==true))
			{
			chip->is_power_changed = true;
			}
		else if((get_prop_batt_voltage_now(chip)/1000 < (4180 - 10))
				&& (chip->temp_littel_cool_set_current_0_point_25c ==false))
				{
				chip->is_power_changed = true;
				}
		}

	batt_temp = get_prop_batt_temp(chip);
	checkout_term_current(chip, batt_temp);
	if(((AUTO_CHARGING_BATT_TEMP_T0 <= batt_temp) &&(batt_temp < AUTO_CHARGING_BATT_TEMP_T1))&&(chip->charger_status == CHARGER_STATUS_GOOD))
	{
		if((chip->temp_more_cool_rechgvbat >= get_prop_batt_voltage_now(chip)/1000)&&(chip->chg_done==true))
		{
			chip->chg_done=false;
			chip->recharge_pending=true;
			chip->recharge_status=true;

			smbchg_charging_en(chip, true);
			pr_debug("ctemp_more_cool hip->recharge_pending\n");
		}


	}
	else if(((AUTO_CHARGING_BATT_TEMP_T1 <= batt_temp) &&(batt_temp < AUTO_CHARGING_BATT_TEMP_T2))&&(chip->charger_status == CHARGER_STATUS_GOOD))
	{
		if((chip->temp_cool_recharge_vbatdel >= get_prop_batt_voltage_now(chip)/1000)&&(chip->chg_done==true))
		{
			chip->chg_done=false;
			chip->recharge_pending=true;
			chip->recharge_status=true;

			smbchg_charging_en(chip, true);
			pr_debug("temp_cool chip->recharge_pending\n");
		}


	}
	else if(((AUTO_CHARGING_BATT_TEMP_T2 <= batt_temp) &&(batt_temp < AUTO_CHARGING_BATT_TEMP_T3))&&(chip->charger_status == CHARGER_STATUS_GOOD))
	{
		/*temp_littel_cool  vchg can reach 4.32v ,chg_done will be set in chg_term_handler( ) */
		if((chip->temp_littel_cool_recharge_vbatdel >= get_prop_batt_voltage_now(chip)/1000)&&(chip->chg_done==true))
		{
			chip->chg_done=false;
			chip->recharge_pending=true;
			chip->recharge_status=true;

			smbchg_charging_en(chip, true);
			pr_debug("temp_littel_cool chip->recharge_pending\n");
		}


	}
	else if(((AUTO_CHARGING_BATT_TEMP_T3 <= batt_temp) &&(batt_temp < AUTO_CHARGING_BATT_TEMP_T5))&&(chip->charger_status == CHARGER_STATUS_GOOD))
	{
		/*temp_normal  vchg can reach 4.32v ,chg_done will be set in chg_term_handler( ) */
		if((chip->temp_normal_recharge_vbatdel >= get_prop_batt_voltage_now(chip)/1000)&&(chip->chg_done==true))
		{
			chip->chg_done=false;
			chip->recharge_pending=true;
			chip->recharge_status=true;

			smbchg_charging_en(chip, true);
			pr_debug("temp_normal chip->recharge_pending\n");
		}


	}
	else if(((AUTO_CHARGING_BATT_TEMP_T5 <= batt_temp) &&(batt_temp < AUTO_CHARGING_BATT_TEMP_T6))&&(chip->charger_status == CHARGER_STATUS_GOOD))
	{
		if((chip->temp_warm_recharge_vbatdel >= get_prop_batt_voltage_now(chip)/1000)&&(chip->chg_done==true))
		{
			chip->chg_done=false;
			chip->recharge_pending=true;
			chip->recharge_status=true;

			smbchg_charging_en(chip, true);
			pr_debug("temp_warm chip->recharge_pending\n");
		}


	}
	if((chip->charger_status == CHARGER_STATUS_GOOD)&&(chip->battery_status == BATTERY_STATUS_GOOD)&&(chip->time_out != true)){
		qpnp_check_battery_temp(chip);
	}
	if(chip->allow_print_log==true || (smbchg_debug_mask&PR_DUMP))
	{
	chip->allow_print_log = false;
	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	pr_err("charger_type:%d,bat_voltage:%d,charger_voltage:%d,current:%d,temp:%d,capacity:%d\n",
	charger_type_value,get_prop_batt_voltage_now(chip),get_prop_charger_voltage_now(chip),
	get_prop_batt_current_now(chip),get_prop_batt_temp(chip),get_prop_batt_capacity(chip));
	dump_regs(chip);
	}
out:
	/*update time 6s*/
	schedule_delayed_work(&chip->update_heartbeat_work,
			round_jiffies_relative(msecs_to_jiffies
				(BATT_HEARTBEAT_INTERVAL)));
}

/* yangfangbiao@oneplus.cn, 2015/05/11  Modify for backup soc  Begin  */
static int load_data(struct smbchg_chip *chip)
{
	u8 stored_soc = 0;
	int rc = 0;
	int shutdown_soc = 0;

	if (chip == NULL) {
		printk(KERN_ERR "%s: chip is NULL !\n", __func__);
		return SOC_INVALID;
	}


	rc = smbchg_read(chip, &stored_soc, SOC_DATA_REG_0, 1);
	if (0 != rc) {
		printk(KERN_ERR "%s: failed to read addr[0x%x], rc=%d\n", __func__, SOC_DATA_REG_0, rc);
		return SOC_INVALID;
	}

	if ((stored_soc%2 )==1)//the fist time connect battery ,the reg 0x88d is 0x0,we do not need load this data.
		shutdown_soc = (stored_soc >> 1 );//get data from  bit1~bit7
	else
		shutdown_soc = SOC_INVALID;
	printk(KERN_ERR "%s: stored_soc[0x%x], shutdown_soc[%d]\n", __func__, stored_soc, shutdown_soc);
	return shutdown_soc;
}

int load_soc(void)
{
	int soc = 0;

	soc = load_data(g_chip);
	if (soc == SOC_INVALID || soc < 0 || soc > 100)
		return -1;
	return soc;
}
static void clear_backup_soc(struct smbchg_chip *chip)
{
	int rc = 0;
	u8 soc_temp = 0;
	rc = smbchg_write(chip, &soc_temp, SOC_DATA_REG_0,1);
	if (rc) {
		printk(KERN_ERR "%s: failed to clean addr[0x%x], rc=%d\n", __func__, SOC_DATA_REG_0, rc);
	}
}
void clean_backup_soc_ex(void)
{
	if(g_chip != NULL)
		clear_backup_soc(g_chip);
}

static void backup_soc(struct smbchg_chip *chip, int soc)
{
	int rc = 0;
	u8 invalid_soc = SOC_INVALID;
	u8 soc_temp = (soc << 1) +1;//store data in bit1~bit7
	if (chip == NULL || soc < 0 || soc > 100) {
		printk(KERN_ERR "%s: chip or soc invalid, store an invalid soc\n", __func__);
		if(chip != NULL){
			rc = smbchg_write(chip, &invalid_soc, SOC_DATA_REG_0, 1);
		}
		return;
	}

	printk(KERN_ERR "%s: backup_soc[%d]\n", __func__, soc);
	rc = smbchg_write(chip, &soc_temp, SOC_DATA_REG_0,1);
	if (rc) {
		printk(KERN_ERR "%s: failed to write addr[0x%x], rc=%d\n", __func__, SOC_DATA_REG_0, rc);
	}
}
void backup_soc_ex(int soc)
{
	if(g_chip != NULL)
		backup_soc(g_chip, soc);
}
/* yangfangbiao@oneplus.cn, 2015/05/11  Modify for backup soc  End  */
bool get_oem_charge_done_status(void)/* yangfangbiao@oneplus.cn, 2015/05/15,add for notifiy chg_done status  */
{
	if(g_chip != NULL)
		return g_chip->chg_done;
	else
		return false;
}
static int update_dash_unplug_status(void)
{
	int charger_present;
	charger_present = (get_prop_charger_voltage_now(g_chip) > 2500)?true:false;//vbus  > 2.5v
	update_usb_status(g_chip, charger_present, true);
	return 0;
}
static int set_dash_charger_present(int status)
{
	int charger_present;
	if(g_chip != NULL)
	{
	charger_present = is_usb_present(g_chip) | is_dc_present(g_chip);
	g_chip->dash_present = status && charger_present;
	if(g_chip->dash_present == true)
	{
	power_supply_set_online(g_chip->usb_psy, true);
	pr_err("set usb online = 1\n");
	power_supply_set_supply_type(g_chip->usb_psy, POWER_SUPPLY_TYPE_DASH);
	power_supply_set_current_limit(g_chip->usb_psy,DEFAULT_WALL_CHG_MA * 1000);
	}
	power_supply_changed(&g_chip->batt_psy);
	pr_err("chip->dash_present =%d,charger_present=%d\n",g_chip->dash_present,charger_present);
	}
	else
	pr_err("set_dash_charger_present error\n");
	return 0;
}

static struct notify_dash_event notify_unplug_event  = {
	.notify_event 	= update_dash_unplug_status,
	.notify_dash_charger_present 	=set_dash_charger_present,
};

static int smbchg_check_chg_version(struct smbchg_chip *chip)
{
	struct pmic_revid_data *pmic_rev_id;
	struct device_node *revid_dev_node;
	int rc;

	revid_dev_node = of_parse_phandle(chip->spmi->dev.of_node,
					"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_err("Missing qcom,pmic-revid property - driver failed\n");
		return -EINVAL;
	}

	pmic_rev_id = get_revid_data(revid_dev_node);
	if (IS_ERR(pmic_rev_id)) {
		rc = PTR_ERR(revid_dev_node);
		if (rc != -EPROBE_DEFER)
			pr_err("Unable to get pmic_revid rc=%d\n", rc);
		return rc;
	}

	switch (pmic_rev_id->pmic_subtype) {
	case PMI8994:
		chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA
				| SMBCHG_BATT_OV_WA
				| SMBCHG_CC_ESR_WA
				| SMBCHG_RESTART_WA;
		use_pmi8994_tables(chip);
		chip->schg_version = QPNP_SCHG;
		break;
	case PMI8950:
	case PMI8937:
		chip->wa_flags |= SMBCHG_BATT_OV_WA;
		if (pmic_rev_id->rev4 < 2) /* PMI8950 1.0 */ {
			chip->wa_flags |= SMBCHG_AICL_DEGLITCH_WA;
		} else	{ /* rev > PMI8950 v1.0 */
			chip->wa_flags |= SMBCHG_HVDCP_9V_EN_WA
					| SMBCHG_USB100_WA;
		}
		use_pmi8994_tables(chip);
		chip->schg_version = QPNP_SCHG_LITE;
		if (pmic_rev_id->pmic_subtype == PMI8937)
			chip->hvdcp_not_supported = true;
		break;
	case PMI8996:
		chip->wa_flags |= SMBCHG_CC_ESR_WA
				| SMBCHG_FLASH_ICL_DISABLE_WA
				| SMBCHG_RESTART_WA;
		use_pmi8996_tables(chip);
		chip->schg_version = QPNP_SCHG;
		chip->hvdcp_not_supported = true;
		break;
	default:
		pr_err("PMIC subtype %d not supported, WA flags not set\n",
				pmic_rev_id->pmic_subtype);
	}

	pr_smb(PR_STATUS, "pmic=%s, wa_flags=0x%x, hvdcp_supported=%s\n",
			pmic_rev_id->pmic_name, chip->wa_flags,
			chip->hvdcp_not_supported ? "false" : "true");

	return 0;
}


#ifdef CONFIG_PROC_FS

#define SHIP_MODE_OFFSET 0x40

static ssize_t write_ship_mode(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct smbchg_chip *chip = g_chip;
	if (count) {
		chip->ship_mode = true;
		pr_err(" * * * XCB * * * write_ship_mode\n");
	}

	return count;
}

static const struct file_operations proc_ship_mode_operations = {
	.write		= write_ship_mode,
	.llseek		= noop_llseek,
};


#endif

static int set_usb_redetec(const char *val, struct kernel_param *kp)
{
	struct smbchg_chip *chip = g_chip;
	enum power_supply_type usb_supply_type;
	char *usb_type_name = "null";

	if (!chip->usb_present)
		return 0;
	pr_info("%s:\n",__func__);

	read_usb_type(chip, &usb_type_name, &usb_supply_type);
	if (usb_supply_type == POWER_SUPPLY_TYPE_USB) {
		fake_insertion_removal(chip, false);
		fake_insertion_removal(chip, true);
	}
	return 0;
}

module_param_call(usb_redetec, set_usb_redetec, NULL, NULL, 0644);

static int notify_usb_enumeration_function(int status)
{
       pr_err("%s:%d\n",__func__,status);
       g_chip->usb_enum_status =status;
       return  g_chip->usb_enum_status;
}

static struct notify_usb_enumeration_status usb_enumeration  = {
       .notify_usb_enumeration         = notify_usb_enumeration_function,
};
static int smbchg_probe(struct spmi_device *spmi)
{
	int rc;
	struct smbchg_chip *chip;
	struct power_supply *usb_psy, *typec_psy = NULL;
	struct qpnp_vadc_chip *vadc_dev, *vchg_vadc_dev;
	const char *typec_psy_name;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_smb(PR_STATUS, "USB supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	if (of_property_read_bool(spmi->dev.of_node, "qcom,external-typec")) {
		/* read the type power supply name */
		rc = of_property_read_string(spmi->dev.of_node,
				"qcom,typec-psy-name", &typec_psy_name);
		if (rc) {
			pr_err("failed to get prop typec-psy-name rc=%d\n",
				rc);
			return rc;
		}

		typec_psy = power_supply_get_by_name(typec_psy_name);
		if (!typec_psy) {
			pr_smb(PR_STATUS,
				"Type-C supply not found, deferring probe\n");
			return -EPROBE_DEFER;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,dcin-vadc", NULL)) {
		vadc_dev = qpnp_get_vadc(&spmi->dev, "dcin");
		if (IS_ERR(vadc_dev)) {
			rc = PTR_ERR(vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc rc=%d\n",
						rc);
			return rc;
		}
	}

	if (of_find_property(spmi->dev.of_node, "qcom,vchg_sns-vadc", NULL)) {
		vchg_vadc_dev = qpnp_get_vadc(&spmi->dev, "vchg_sns");
		if (IS_ERR(vchg_vadc_dev)) {
			rc = PTR_ERR(vchg_vadc_dev);
			if (rc != -EPROBE_DEFER)
				dev_err(&spmi->dev, "Couldn't get vadc 'vchg' rc=%d\n",
						rc);
			return rc;
		}
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->fcc_votable = create_votable(&spmi->dev,
			"SMBCHG: fcc",
			VOTE_MIN, NUM_FCC_VOTER, 2000,
			set_fastchg_current_vote_cb);
	if (IS_ERR(chip->fcc_votable))
		return PTR_ERR(chip->fcc_votable);

	chip->usb_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_usb_current_limit_vote_cb);
	if (IS_ERR(chip->usb_icl_votable))
		return PTR_ERR(chip->usb_icl_votable);

	chip->dc_icl_votable = create_votable(&spmi->dev,
			"SMBCHG: dcl_icl",
			VOTE_MIN, NUM_ICL_VOTER, 3000,
			set_dc_current_limit_vote_cb);
	if (IS_ERR(chip->dc_icl_votable))
		return PTR_ERR(chip->dc_icl_votable);

	chip->usb_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: usb_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			usb_suspend_vote_cb);
	if (IS_ERR(chip->usb_suspend_votable))
		return PTR_ERR(chip->usb_suspend_votable);

	chip->dc_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: dc_suspend",
			VOTE_SET_ANY, NUM_EN_VOTERS, 0,
			dc_suspend_vote_cb);
	if (IS_ERR(chip->dc_suspend_votable))
		return PTR_ERR(chip->dc_suspend_votable);

	chip->battchg_suspend_votable = create_votable(&spmi->dev,
			"SMBCHG: battchg_suspend",
			VOTE_SET_ANY, NUM_BATTCHG_EN_VOTERS, 0,
			charging_suspend_vote_cb);
	if (IS_ERR(chip->battchg_suspend_votable))
		return PTR_ERR(chip->battchg_suspend_votable);

	chip->hw_aicl_rerun_disable_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_disable",
			VOTE_SET_ANY, NUM_HW_AICL_DISABLE_VOTERS, 0,
			smbchg_hw_aicl_rerun_disable_cb);
	if (IS_ERR(chip->hw_aicl_rerun_disable_votable))
		return PTR_ERR(chip->hw_aicl_rerun_disable_votable);

	chip->hw_aicl_rerun_enable_indirect_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_enable_indirect",
			VOTE_SET_ANY, NUM_HW_AICL_RERUN_ENABLE_INDIRECT_VOTERS,
			0, smbchg_hw_aicl_rerun_enable_indirect_cb);
	if (IS_ERR(chip->hw_aicl_rerun_enable_indirect_votable))
		return PTR_ERR(chip->hw_aicl_rerun_enable_indirect_votable);

	chip->aicl_deglitch_short_votable = create_votable(&spmi->dev,
			"SMBCHG: hwaicl_short_deglitch",
			VOTE_SET_ANY, NUM_HW_SHORT_DEGLITCH_VOTERS, 0,
			smbchg_aicl_deglitch_config_cb);
	if (IS_ERR(chip->aicl_deglitch_short_votable))
		return PTR_ERR(chip->aicl_deglitch_short_votable);

	INIT_WORK(&chip->usb_set_online_work, smbchg_usb_update_online_work);
	INIT_DELAYED_WORK(&chip->parallel_en_work,
			smbchg_parallel_usb_en_work);
	INIT_DELAYED_WORK(&chip->vfloat_adjust_work, smbchg_vfloat_adjust_work);
	INIT_DELAYED_WORK(&chip->hvdcp_det_work, smbchg_hvdcp_det_work);
	chip->redect_charger_type_done =false;
	INIT_DELAYED_WORK(&chip->re_det_work, smbchg_re_det_work);
	INIT_DELAYED_WORK(&chip->check_switch_dash_work, check_allow_switch_dash);
	INIT_DELAYED_WORK(&chip->nonstandard_charger_charge_current_set_work, check_nonstandard_charger_charge_current);
	init_completion(&chip->src_det_lowered);
	init_completion(&chip->src_det_raised);
	init_completion(&chip->usbin_uv_lowered);
	init_completion(&chip->usbin_uv_raised);
	chip->vadc_dev = vadc_dev;
	chip->vchg_vadc_dev = vchg_vadc_dev;
	chip->spmi = spmi;
	chip->dev = &spmi->dev;
	chip->usb_psy = usb_psy;
	chip->typec_psy = typec_psy;
	chip->fake_battery_soc = -EINVAL;
	chip->usb_online = -EINVAL;
	dev_set_drvdata(&spmi->dev, chip);

	spin_lock_init(&chip->sec_access_lock);
	mutex_init(&chip->therm_lvl_lock);
	mutex_init(&chip->usb_set_online_lock);
	mutex_init(&chip->parallel.lock);
	mutex_init(&chip->taper_irq_lock);
	mutex_init(&chip->pm_lock);
	mutex_init(&chip->wipower_config);
	mutex_init(&chip->usb_status_lock);
	device_init_wakeup(chip->dev, true);
	wake_lock_init(&chip->chg_wake_lock,
			WAKE_LOCK_SUSPEND, "chg_wake_lock");

	rc = smbchg_parse_peripherals(chip);
	if (rc) {
		dev_err(chip->dev, "Error parsing DT peripherals: %d\n", rc);
		return rc;
	}

	rc = smbchg_check_chg_version(chip);
	if (rc) {
		pr_err("Unable to check schg version rc=%d\n", rc);
		return rc;
	}
	qpnp_charge_info_init(chip);

	rc = smb_parse_dt(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to parse DT nodes: %d\n", rc);
		return rc;
	}

	rc = smbchg_regulator_init(chip);
	if (rc) {
		dev_err(&spmi->dev,
				"Couldn't initialize regulator rc=%d\n", rc);
		return rc;
	}

	rc = smbchg_hw_init(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
				"Unable to intialize hardware rc = %d\n", rc);
		goto out;
	}

	rc = determine_initial_status(chip);
	if (rc < 0) {
		dev_err(&spmi->dev,
				"Unable to determine init status rc = %d\n", rc);
		goto out;
	}

	chip->previous_soc = -EINVAL;
	chip->batt_psy.name		= chip->battery_psy_name;
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smbchg_battery_get_property;
	chip->batt_psy.set_property	= smbchg_battery_set_property;
	chip->batt_psy.properties	= smbchg_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smbchg_battery_properties);
	chip->batt_psy.external_power_changed = smbchg_external_power_changed;
	chip->batt_psy.property_is_writeable = smbchg_battery_is_writeable;
	g_chip = chip;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&spmi->dev,
				"Unable to register batt_psy rc = %d\n", rc);
		goto out;
	}
	if (chip->dc_psy_type != -EINVAL) {
		chip->dc_psy.name		= "dc";
		chip->dc_psy.type		= chip->dc_psy_type;
		chip->dc_psy.get_property	= smbchg_dc_get_property;
		chip->dc_psy.set_property	= smbchg_dc_set_property;
		chip->dc_psy.property_is_writeable = smbchg_dc_is_writeable;
		chip->dc_psy.properties		= smbchg_dc_properties;
		chip->dc_psy.num_properties = ARRAY_SIZE(smbchg_dc_properties);
		chip->dc_psy.supplied_to = smbchg_dc_supplicants;
		chip->dc_psy.num_supplicants
			= ARRAY_SIZE(smbchg_dc_supplicants);
		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			dev_err(&spmi->dev,
					"Unable to register dc_psy rc = %d\n", rc);
			goto unregister_batt_psy;
		}
	}
	chip->psy_registered = true;

	if (chip->cfg_chg_led_support &&
			chip->schg_version == QPNP_SCHG_LITE) {
		rc = smbchg_register_chg_led(chip);
		if (rc) {
			dev_err(chip->dev,
					"Unable to register charger led: %d\n",
					rc);
			goto unregister_dc_psy;
		}

		rc = smbchg_chg_led_controls(chip);
		if (rc) {
			dev_err(chip->dev,
					"Failed to set charger led controld bit: %d\n",
					rc);
			goto unregister_led_class;
		}
	}

	rc = smbchg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&spmi->dev, "Unable to request irqs rc = %d\n", rc);
		goto unregister_led_class;
	}

	if (!chip->skip_usb_notification) {
		pr_smb(PR_MISC, "setting usb psy present = %d\n",
				chip->usb_present);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}

	if (is_usb_present(chip)) {
		schedule_delayed_work(&chip->re_det_work,
				msecs_to_jiffies(REDET_DELAY_MS));
	}
	else
	{
	chip->redect_charger_type_done =true;
	}

	INIT_DELAYED_WORK(&chip->update_heartbeat_work,
			update_heartbeat);
	schedule_delayed_work(&chip->update_heartbeat_work,
			round_jiffies_relative(msecs_to_jiffies
				(BATT_HEARTBEAT_INTERVAL)));

	rc = device_create_file(chip->dev, &dev_attr_test_chg_vol);
	if (rc < 0) {
		pr_err("%s: creat test_chg_vol file failed ret = %d\n",
				__func__, rc);
	}
	rc = device_create_file(chip->dev, &dev_attr_test_temp);
	if (rc < 0) {
		pr_err("%s: creat test temp file failed ret = %d\n",
				__func__, rc);
	}
	rc = device_create_file(chip->dev, &dev_attr_test_authentic);
	if (rc < 0) {
		pr_err("%s: creat test vbat file failed ret = %d\n",
				__func__, rc);
	}
#if defined(CONFIG_FB)
	/* yangfangbiao@oneplus.cn,20150519  Add for reset charge current when screen is off */
	chip->fb_notif.notifier_call = fb_notifier_callback;
	rc = fb_register_client(&chip->fb_notif);
	if (rc)
		pr_err("Unable to register fb_notifier: %d\n", rc);
#endif /*CONFIG_FB*/
	chip->typec_notif.notifier_call = typeC_notifier_callback;
	rc = bc_register_client(&chip->typec_notif);
	if (rc)
		pr_err("Unable to register  bc_notifier: %d\n", rc);
	regsister_notify_usb_enumeration_status(&usb_enumeration);
	notify_dash_unplug_register(&notify_unplug_event);
	rerun_hvdcp_det_if_necessary(chip);
	dump_regs(chip);
	create_debugfs_entries(chip);

#ifdef CONFIG_PROC_FS
	if (!proc_create("ship_mode", S_IFREG | S_IWUSR | S_IRUGO, NULL,
		 &proc_ship_mode_operations))
	pr_err("Failed to register proc interface\n");
#endif
	chip->probe_success=true;
	dev_info(chip->dev,
			"SMBCHG successfully probe Charger version=%s Revision DIG:%d.%d ANA:%d.%d batt=%d dc=%d usb=%d\n",
			version_str[chip->schg_version],
			chip->revision[DIG_MAJOR], chip->revision[DIG_MINOR],
			chip->revision[ANA_MAJOR], chip->revision[ANA_MINOR],
			get_prop_batt_present(chip),
			chip->dc_present, chip->usb_present);
	return 0;

unregister_led_class:
	if (chip->cfg_chg_led_support && chip->schg_version == QPNP_SCHG_LITE)
	led_classdev_unregister(&chip->led_cdev);
unregister_dc_psy:
	power_supply_unregister(&chip->dc_psy);
unregister_batt_psy:
	power_supply_unregister(&chip->batt_psy);
out:
	handle_usb_removal(chip);
	return rc;
}

static int smbchg_remove(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);

	debugfs_remove_recursive(chip->debug_root);

	if (chip->dc_psy_type != -EINVAL)
		power_supply_unregister(&chip->dc_psy);
	notify_dash_unplug_unregister(&notify_unplug_event);
	power_supply_unregister(&chip->batt_psy);

	return 0;
}

static void smbchg_shutdown(struct spmi_device *spmi)
{
	struct smbchg_chip *chip = dev_get_drvdata(&spmi->dev);
	int i, rc;
#ifdef CONFIG_PROC_FS
	pr_info("smbchg_shutdown\n");
	if (chip->ship_mode) {
		pr_info("smbchg_shutdown enter ship_mode\n");
		smbchg_sec_masked_write(chip, chip->bat_if_base + SHIP_MODE_OFFSET,BIT(0), 0);
		clear_backup_soc(chip);
		msleep(1000);
		pr_err("after 1s\n");
		while(1);
	}
#endif
	if (!(chip->wa_flags & SMBCHG_RESTART_WA))
		return;

	if (!is_hvdcp_present(chip))
		return;

	pr_smb(PR_MISC, "Disable Parallel\n");
	mutex_lock(&chip->parallel.lock);
	smbchg_parallel_en = 0;
	smbchg_parallel_usb_disable(chip);
	mutex_unlock(&chip->parallel.lock);

	pr_smb(PR_MISC, "Disable all interrupts\n");
	disable_irq(chip->aicl_done_irq);
	disable_irq(chip->batt_cold_irq);
	disable_irq(chip->batt_cool_irq);
	disable_irq(chip->batt_hot_irq);
	disable_irq(chip->batt_missing_irq);
	disable_irq(chip->batt_warm_irq);
	disable_irq(chip->chg_error_irq);
	disable_irq(chip->chg_hot_irq);
	disable_irq(chip->chg_term_irq);
	disable_irq(chip->dcin_uv_irq);
	disable_irq(chip->fastchg_irq);
	disable_irq(chip->otg_fail_irq);
	disable_irq(chip->otg_oc_irq);
	disable_irq(chip->power_ok_irq);
	disable_irq(chip->recharge_irq);
	disable_irq(chip->src_detect_irq);
	disable_irq(chip->taper_irq);
	disable_irq(chip->usbid_change_irq);
	disable_irq(chip->usbin_ov_irq);
	disable_irq(chip->usbin_uv_irq);
	disable_irq(chip->vbat_low_irq);
	disable_irq(chip->wdog_timeout_irq);

	/* remove all votes for short deglitch */
	for (i = 0; i < NUM_HW_SHORT_DEGLITCH_VOTERS; i++)
		vote(chip->aicl_deglitch_short_votable, i, false, 0);

	/* vote to ensure AICL rerun is enabled */
	rc = vote(chip->hw_aicl_rerun_enable_indirect_votable,
			SHUTDOWN_WORKAROUND_VOTER, true, 0);
	if (rc < 0)
		pr_err("Couldn't vote to enable indirect AICL rerun\n");
	rc = vote(chip->hw_aicl_rerun_disable_votable,
		WEAK_CHARGER_HW_AICL_VOTER, false, 0);
	if (rc < 0)
		pr_err("Couldn't vote to enable AICL rerun\n");

	/* switch to 5V HVDCP */
	pr_smb(PR_MISC, "Switch to 5V HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
				HVDCP_ADAPTER_SEL_MASK, HVDCP_5V);
	if (rc < 0) {
		pr_err("Couldn't configure HVDCP 5V rc=%d\n", rc);
		return;
	}

	pr_smb(PR_MISC, "Wait 500mS to lower to 5V\n");
	/* wait for HVDCP to lower to 5V */
	msleep(500);
	/*
	 * Check if the same hvdcp session is in progress. src_det should be
	 * high and that we are still in 5V hvdcp
	 */
	if (!is_src_detect_high(chip)) {
		pr_smb(PR_MISC, "src det low after 500mS sleep\n");
		return;
	}

	/* disable HVDCP */
	pr_smb(PR_MISC, "Disable HVDCP\n");
	rc = smbchg_sec_masked_write(chip, chip->usb_chgpth_base + CHGPTH_CFG,
			HVDCP_EN_BIT, 0);
	if (rc < 0)
		pr_err("Couldn't disable HVDCP rc=%d\n", rc);

	chip->hvdcp_3_det_ignore_uv = true;
	/* fake a removal */
	pr_smb(PR_MISC, "Faking Removal\n");
	rc = fake_insertion_removal(chip, false);
	if (rc < 0)
		pr_err("Couldn't fake removal HVDCP Removed rc=%d\n", rc);

	/* fake an insertion */
	pr_smb(PR_MISC, "Faking Insertion\n");
	rc = fake_insertion_removal(chip, true);
	if (rc < 0)
		pr_err("Couldn't fake insertion rc=%d\n", rc);

	pr_smb(PR_MISC, "Wait 1S to settle\n");
	msleep(1000);
	chip->hvdcp_3_det_ignore_uv = false;

	pr_smb(PR_STATUS, "wrote power off configurations\n");
}

static const struct dev_pm_ops smbchg_pm_ops = {
};

MODULE_DEVICE_TABLE(spmi, smbchg_id);

static struct spmi_driver smbchg_driver = {
	.driver		= {
		.name		= "qpnp-smbcharger",
		.owner		= THIS_MODULE,
		.of_match_table	= smbchg_match_table,
		.pm		= &smbchg_pm_ops,
	},
	.probe		= smbchg_probe,
	.remove		= smbchg_remove,
	.shutdown	= smbchg_shutdown,
};

static int __init smbchg_init(void)
{
	return spmi_driver_register(&smbchg_driver);
}

static void __exit smbchg_exit(void)
{
	return spmi_driver_unregister(&smbchg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("QPNP SMB Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:qpnp-smbcharger");
