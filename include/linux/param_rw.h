#ifndef __PARAM_RW_H
#define __PARAM_RW_H

#define PARAM_SID_LENGTH 1024
#define DEFAULT_PARAM_PART_SZIE  (1*1024*1024)
#define DEFAULT_PARAM_DUMP_SIZE 64

typedef unsigned int       uint32;

typedef struct
{
    char                     sid_name[16];
    uint32                   sid_index;
    uint32                   sid_length;   //default 1024bytes
}param_product_desc_head_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    char project_name[8];  //14049
    int hw_version;             //11
    int rf_version;
    char rf_config_str[16];
    int operator_num;
    char operator_str[16];
    char pcba_number[28];
    unsigned int boot_aging_count;
    uint32 network_info_array[10];
}param_product_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    uint32 default_param_data_dump_enable;
    uint32 default_param_data_dump_size;
    //Add value must below here
}param_config_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    uint gamma_select;
    uint lcm_acl;
    uint lcm_srgb_mode;
    uint lcm_adobe_rgb_mode;
    uint lcm_dci_p3_mode;
	
}param_lcd_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    //Add value must below here
}param_tp_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    //Add value must below here
}param_tp_kpd_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    uint32 laser_sensor_offset;
    uint32 laser_sensor_cross_talk;
}param_camera_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    //Add value must below here
}param_sensors_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    //Add value must below here
}param_battery_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    //Add value must below here
}param_rtc_t;


/****************** PARAM CRASH RECORD FEATURE ******************
 * If you want to modify PARAM crash record struct,             *
 * please sync with                                             *
 * /system/core/include/param/param.h and                       *
 * /bootable/bootloader/lk/platform/oneplus/include/op_param.h  *
 *                                                              *
 * liochen@BSP, 2016/07/22, store crash record in PARAM         *
 ****************************************************************/
typedef struct
{
    param_product_desc_head_t sid_head;
    int crash_count;
    char crash_record_0[20];
    char crash_record_1[20];
    char crash_record_2[20];
    char crash_record_3[20];
    char crash_record_4[20];
    char crash_record_5[20];
    char crash_record_6[20];
    char crash_record_7[20];
    char crash_record_8[20];
    char crash_record_9[20];
    char crash_record_10[20];
    char crash_record_11[20];
    char crash_record_12[20];
    char crash_record_13[20];
    char crash_record_14[20];
    char crash_record_15[20];
    int restart_08_count;
    int restart_other_count;
}param_crash_record_t;
/***************************************************/

/****************** PARAM PHONE HISTORY RECORD FEATURE *****************
* If you want to modify PARAM phone history record struct,             *
* please sync with                                                     *
* /kernel/msm-4.4/include/linux/param_rw.h and                         *
* /bootable/bootloader/edk2/QcomModulePkg/Include/Library/OpParam.h    *
*                                                                      *
* wuaijun@system, 2017/04/26, store phone history record in PARAM      *
************************************************************************/
typedef struct {
	param_product_desc_head_t sid_head;
	int normal_reboot_count;
	int abnormal_reboot_count;
	int update_count;
	int fastboot_count;
    /* liochen@BSP, 2017/07/17, Add param unlock_count */
	int unlock_count;
	int poweron_count;
	int poweroff_count;
} param_phonehistory_t;
/***************************************************/

/****************** PARAM CHARGER TYPE RECORD FEATURE ******************
 *                                                              *
 * yangfb@BSP, 2016/09/14, store chager type record in PARAM         *
 ****************************************************************/
typedef struct
{
    param_product_desc_head_t sid_head;
    int type_count;
    char type_record_0[20];
    char type_record_1[20];
    char type_record_2[20];
    char type_record_3[20];
    char type_record_4[20];
    char type_record_5[20];
    char type_record_6[20];
    char type_record_7[20];
    char type_record_8[20];
    char type_record_9[20];
    char type_record_10[20];
    char type_record_11[20];
    char type_record_12[20];
    char type_record_13[20];
    char type_record_14[20];
    char type_record_15[20];
    char type_record_16[20];
    char type_record_17[20];
    char type_record_18[20];
    char type_record_19[20];
    char type_record_20[20];
    char type_record_21[20];
    char type_record_22[20];
    char type_record_23[20];
    char type_record_24[20];
    char type_record_25[20];
    char type_record_26[20];
    char type_record_27[20];
    char type_record_28[20];
    char type_record_29[20];
    char type_record_30[20];
    char type_record_31[20];
    char type_record_32[20];
    char type_record_33[20];
    char type_record_34[20];
    char type_record_35[20];
    char type_record_36[20];
    char type_record_37[20];
    char type_record_38[20];
    char type_record_39[20];
    char type_record_40[20];
    char type_record_41[20];
    char type_record_42[20];
    char type_record_43[20];
    char type_record_44[20];
    char type_record_45[20];
    char type_record_46[20];
    char type_record_47[20];

}param_chg_type_record_t;
/***************************************************/


typedef struct
{
    param_product_desc_head_t sid_head;
    /* Only for wlan evm chip */
    int use_special_boarddata;
    //Add value must below here
}param_misc_t;

typedef struct
{
    param_product_desc_head_t sid_head;
    int is_rooted;
    int root_time;
    char flash_0[16]; //the recently flashed partition
    char flash_1[16]; //the next-recently flashed partition
    char flash_2[16]; //the third recently flashed partition
    char erase_0[16]; //the recently erased partition
    char erase_1[16]; //the next-recently erased partition
    char erase_2[16]; //the third recently erased partition
    int is_angela;
}param_saleinfo_t;


typedef struct
{
    param_product_desc_head_t sid_head;
    char smt_download_time1[32];
	char smt_download_version1[32];
	char smt_download_time2[32];
	char smt_download_version2[32];
	char smt_download_time3[32];
	char smt_download_version3[32];
	char upgrade_download_time1[32];
	char upgrade_download_version1[32];
	char upgrade_download_time2[32];
	char upgrade_download_version2[32];
	char upgrade_download_time3[32];
	char upgrade_download_version3[32];
	int boot_stage;
	int data_stage;
	int reset_devinfo;
	int boottype;
}param_download_t;

typedef enum {
	PARAM_SID_PRODUCT = 0,
	PARAM_SID_CONFIG,
	PARAM_SID_LCD,
	PARAM_SID_TP,
	PARAM_SID_TP_KPD,
	PARAM_SID_CAMERA,
	PARAM_SID_SENSORS,
	PARAM_SID_BATTERY,
	PARAM_SID_RTC,
	PARAM_SID_CRASH_RECORD,
	PARAM_SID_SALEINFO,
	PARAM_SID_MISC,
	PARAM_SID_DOWNLOAD,
	PARAM_SID_PHONE_HISTORY,
	PARAM_SID_INVALID
} param_sid_index_t;

typedef enum {
	BOOT_UP = 0x0,
	BOOT_INTO_LK = 0x1234,
	BOOT_INTO_KERNEL = 0x5678,
	BOOT_INTO_ANDROID = 0x9012
} device_boot_stage_t;

#if 0
static char * sid_name_strs[PARAM_SID_INVALID] = {
"PRODUCT",
"CONFIG",
"LCD",
"TP",
"TP_KPD",
"TP_CAMERA",
"SENSORS",
"BATTERY",
"RTC",
"CRASH_RECORD",
"SALEINFO",
"MISC",
"DOWNLOAD"
};
#endif

#define CHUNK_UNION(_name) \
 union { param_##_name##_t _name##_chunk; \
 unsigned char c[1024]; } _name##_chunk_u

typedef struct
{
    //union { param_product_t  product_chunk; unsigned char c[1024]; } product_chunk_u;
    CHUNK_UNION(product);
    CHUNK_UNION(config);
    CHUNK_UNION(lcd);
    CHUNK_UNION(tp);
    CHUNK_UNION(tp_kpd);
    CHUNK_UNION(camera);
    CHUNK_UNION(sensors);
    CHUNK_UNION(battery);
    CHUNK_UNION(rtc);
    CHUNK_UNION(crash_record);
    CHUNK_UNION(misc);
	CHUNK_UNION(download);
    CHUNK_UNION(phonehistory);
}global_param_data_t;

int get_param_camera_laser_sensor_offset(uint * laser_sensor_offset);
int set_param_camera_laser_sensor_offset(uint * laser_sensor_offset);
int get_param_camera_laser_sensor_cross_talk(uint * laser_sensor_cross_talk);
int set_param_camera_laser_sensor_cross_talk(uint * laser_sensor_cross_talk);
int get_param_gamma_select(uint * gamma_select);
int get_param_pcba_number(char * pcbe_number);
/* Only for wlan evm chip */
int get_param_nvm_boarddata(uint * nvm_boarddata_select);
//yankelong add
int set_param_lcm_srgb_mode(uint * lcm_srgb_mdoe);
int get_param_lcm_srgb_mode(uint *lcm_srgb_mode);
int set_param_lcm_adobe_rgb_mode(uint * lcm_adobe_rgb_mode);
int get_param_lcm_adobe_rgb_mode(uint *lcm_adobe_rgb_mode);
int set_param_lcm_dci_p3_mode(uint * lcm_dci_p3_mode);
int get_param_lcm_dci_p3_mode(uint *lcm_dci_p3_mode);


int get_param_download_info(param_download_t *download_info);
/* liochen@BSP, 2016/07/26, store crash record in PARAM */
int get_param_crash_record_count(uint *crash_record_count);
int set_param_crash_record_count(uint *crash_record_count);
int set_param_crash_record_value(uint offset, char *crash_record_value, uint size);
int add_restart_08_count(void);
int add_restart_other_count(void);

/*  yangfb@BSP, 2016/09/14, store chager type record in PARAM  */
int get_param_charger_type_count(uint *type_record_count);
int set_param_charger_type_count(uint *type_record_count);
int set_param_charger_type_value(uint offset, char *crash_record_value, uint size);
int get_param_poweroff_count(uint *poweroff_count);
int set_param_poweroff_count(uint *poweroff_count);

//end
#endif
