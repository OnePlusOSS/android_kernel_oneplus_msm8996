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

typedef struct
{
    param_product_desc_head_t sid_head;
    int android_crash_count;
    int kernel_crash_count;
    int modem_crash_count;
    int wifi_bt_crash_count;
    int venus_crash_count;
}param_crash_record_t;

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
	PARAM_SID_INVALID
} param_sid_index_t;

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
}global_param_data_t;

int get_param_camera_laser_sensor_offset(uint * laser_sensor_offset);
int set_param_camera_laser_sensor_offset(uint * laser_sensor_offset);
int get_param_camera_laser_sensor_cross_talk(uint * laser_sensor_cross_talk);
int set_param_camera_laser_sensor_cross_talk(uint * laser_sensor_cross_talk);
int get_param_gamma_select(uint * gamma_select);
int get_param_pcba_number(char * pcbe_number);
/* Only for wlan evm chip */
int get_param_nvm_boarddata(uint * nvm_boarddata_select);
int set_param_lcm_srgb_mode(uint * lcm_srgb_mdoe);
int get_param_lcm_srgb_mode(uint *lcm_srgb_mdoe);

#endif
