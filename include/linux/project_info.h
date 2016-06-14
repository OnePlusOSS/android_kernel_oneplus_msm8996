#ifndef _PROJECT_INFO_H_
#define _PROJECT_INFO_H_ 1

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
	COMPONENT_MAX,
};

int push_component_info(enum COMPONENT_TYPE type, char *version, char * manufacture);
int reset_component_info(enum COMPONENT_TYPE type);

#endif
