#ifndef	__OEM_EXTERNAL_FG_H__
#define __OEM_EXTERNAL_FG_H__

enum {
	ADAPTER_FW_UPDATE_NONE,
	ADAPTER_FW_NEED_UPDATE,
	ADAPTER_FW_UPDATE_SUCCESS,
	ADAPTER_FW_UPDATE_FAIL,
};
struct op_adapter_chip {
	int		timer_delay;
	bool	tx_byte_over;
	bool	rx_byte_over;
	bool	rx_timeout;
	unsigned long uart_tx_gpio;
	unsigned long uart_rx_gpio;
	char	*adapter_firmware_data;
	unsigned int	adapter_fw_data_count;
	unsigned int	tx_invalid_val;
	bool	adapter_update_ing;
	struct op_adapter_operations	*vops;
};
struct op_adapter_operations {
	bool (*adapter_update) (struct op_adapter_chip *chip,
			unsigned long tx_pin, unsigned long rx_pin);
};

enum chg_protect_status_type {
    PROTECT_CHG_OVP = 1,                  /* 1: VCHG > 5.8V     */
    PROTECT_BATT_MISSING,                 /* 2: battery missing */
    PROTECT_CHG_OVERTIME,                 /* 3: charge overtime */
    PROTECT_BATT_OVP,                     /* 4: vbat >= 4.5     */
    PROTECT_BATT_TEMP_REGION__HOT,        /* 5: 55 < t          */
    PROTECT_BATT_TEMP_REGION_COLD,        /* 6:      t <= -3    */
    PROTECT_BATT_TEMP_REGION_LITTLE_COLD, /* 7: -3 < t <=  0    */
    PROTECT_BATT_TEMP_REGION_COOL,        /* 8:  0 < t <=  5    */
    PROTECT_BATT_TEMP_REGION_WARM         /* 9: 45 < t <= 55    */
};

struct external_battery_gauge {
	int (*get_battery_mvolts) (void);
	int (*get_battery_temperature) (void);
	bool (*is_battery_present) (void);
	bool (*is_battery_temp_within_range) (void);
	bool (*is_battery_id_valid) (void);
	int (*get_battery_status)(void);
	int (*get_batt_remaining_capacity) (void);
	int (*get_batt_health) (void);
	int (*monitor_for_recharging) (void);
	int (*get_battery_soc) (void);
	int (*get_average_current) (void);
	int (*get_batt_cc) (void);
	int (*get_batt_fcc) (void);
	bool (*fast_chg_started) (void);
	bool (*fast_switch_to_normal) (void);
	int (*set_switch_to_noraml_false) (void);
	int (*set_fast_chg_allow) (bool enable);
	bool (*get_fast_chg_allow) (void);
	int (*fast_normal_to_warm)	(void);
	int (*set_normal_to_warm_false)	(void);
	int (*get_adapter_update)	(void);
	bool (*get_fast_chg_ing)	(void);
	int (*get_ng_count)(void);
	bool (*get_fast_low_temp_full)	(void);
	int (*set_low_temp_full_false)	(void);
	int (*set_alow_reading) (int enable);
	int (*set_lcd_off_status) (int status);
	int (*fast_chg_started_status) (bool status);
	bool (*get_fastchg_firmware_already_updated) (void);
	bool (*get_4p4v_battery_present)(void);
};

struct notify_dash_event {
	int (*notify_event) (void);
	int (*notify_dash_charger_present) (int true);
};

struct notify_usb_enumeration_status {
	int (*notify_usb_enumeration) (int status);
};

typedef enum {
	BATT_TEMP_COLD = 0,
	BATT_TEMP_LITTLE_COLD,
	BATT_TEMP_COOL,
	BATT_TEMP_LITTLE_COOL,
	BATT_TEMP_PRE_NORMAL,
	BATT_TEMP_NORMAL,
	BATT_TEMP_WARM,
	BATT_TEMP_HOT,
	BATT_TEMP_INVALID,
} temp_region_type;

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

/*add for dash adapter update*/
extern bool dash_adapter_update_is_tx_gpio(unsigned long gpio_num);
extern bool dash_adapter_update_is_rx_gpio(unsigned long gpio_num);
#endif
