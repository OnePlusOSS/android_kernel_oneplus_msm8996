#ifndef _PROJECT_INFO_H_
#define _PROJECT_INFO_H_ 1
typedef		__u32		uint32;

struct project_info{
       char project_name[8];  //eg, 14049
       uint32  hw_version;  //PCB number, T0, EVT
       uint32  rf_v1;   //v1 for mainboard_rf_version
       uint32  rf_v2;   //v2 for aboard_rf_version
       uint32  rf_v3;
       uint32  modem;
       uint32  operator;
       uint32  ddr_manufacture_info;
       uint32  ddr_raw;
       uint32  ddr_column;
       uint32  ddr_reserve_info;
	   uint32  platform_id;
};

struct component_info{
	char *version;
	char *manufacture;
};

enum{
	HW_VERSION__UNKNOWN,
	HW_VERSION__10 = 10,//all EVB
	HW_VERSION__11, 	//T0
	HW_VERSION__12, 	//EVT1
	HW_VERSION__13, 	//EVT2
	HW_VERSION__14, 	//EVT3
	HW_VERSION__15, 	//DVT
	HW_VERSION__16, 	//PVT/MP
	HW_VERSION__26 = 26,//15811 PVT
	HW_VERSION__27 ,    //15811 PVT2
};

enum COMPONENT_TYPE{
	DDR,
	EMMC,
	F_CAMERA,
	R_CAMERA,
	TP,
	LCD,
	WCN,
	I_SENSOR,
	G_SENSOR,
	M_SENSOR,
	GYRO,
	BACKLIGHT,
	MAINBOARD,
	/*Add new component here*/
	FINGERPRINTS,
	TOUCH_KEY,
	UFS,
	ABOARD,
	NFC,
	FAST_CHARGE,
	CPU,
	COMPONENT_MAX,
};


int push_component_info(enum COMPONENT_TYPE type, char *version, char * manufacture);
int reset_component_info(enum COMPONENT_TYPE type);
uint32 get_hw_version(void);


#endif
