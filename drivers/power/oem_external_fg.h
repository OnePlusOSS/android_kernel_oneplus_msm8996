#ifndef	__OEM_EXTERNAL_FG_H__
#define __OEM_EXTERNAL_FG_H__

struct external_battery_gauge {
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	int (*is_battery_present) (void);
	int (*is_battery_temp_within_range) (void);
	int (*is_battery_id_valid) (void);
	int (*get_battery_status)(void);
	int (*get_batt_remaining_capacity) (void);
	int (*monitor_for_recharging) (void);
	int (*get_battery_soc) (void);
	int (*get_average_current) (void);
	int (*is_battery_authenticated) (void);//wangjc add for authentication
	int (*get_batt_cc) (void);/* yangfangbiao@oneplus.cn, 2015/02/13  Add fcc interface */
	int (*get_batt_fcc) (void);  /* yangfangbiao@oneplus.cn, 2015/01/06  Modify for  sync with KK charge standard  */
	int	(*fast_chg_started) (void);
	int (*fast_switch_to_normal) (void);
	int (*set_switch_to_noraml_false) (void);
	int (*set_fast_chg_allow) (int enable);
	int (*get_fast_chg_allow) (void);
	int (*fast_normal_to_warm)	(void);
	int (*set_normal_to_warm_false)	(void);
	int	(*get_fast_chg_ing)	(void);
	int	(*get_fast_low_temp_full)	(void);
	int	(*set_low_temp_full_false)	(void);
	int (*set_alow_reading) (int enable);
	int (*set_lcd_off_status) (int status);
	int (*fast_chg_started_status) (int status);
	int (*get_fastchg_firmware_already_updated) (void);
};
struct notify_dash_event {
	int (*notify_event) (void);
	int (*notify_dash_charger_present) (int true);
};
struct notify_usb_enumeration_status {
	int (*notify_usb_enumeration) (int status);
};
void  regsister_notify_usb_enumeration_status(struct  notify_usb_enumeration_status *event);

void notify_dash_unplug_register(struct  notify_dash_event *event);
void notify_dash_unplug_unregister(struct  notify_dash_event *event);

void fastcharge_information_unregister(struct external_battery_gauge *fast_chg);
void fastcharge_information_register(struct external_battery_gauge *fast_chg);
void external_battery_gauge_register(struct external_battery_gauge *batt_gauge);
void external_battery_gauge_unregister(struct external_battery_gauge *batt_gauge);

void bq27541_information_register(struct external_battery_gauge *fast_chg);
void bq27541_information_unregister(struct external_battery_gauge *fast_chg);
bool get_extern_fg_regist_done(void );
int get_prop_pre_shutdown_soc(void);
#endif
