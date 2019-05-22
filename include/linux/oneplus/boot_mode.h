#ifndef _BOOT_MODE_H_
#define _BOOT_MODE_H_ 1

enum oem_boot_mode{
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__FASTBOOT,
	MSM_BOOT_MODE__RECOVERY,
	MSM_BOOT_MODE__AGING, //shankai@bsp , add for aging mode 2016-1-14
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
	MSM_BOOT_MODE__CHARGE,
};
enum oem_boot_mode get_boot_mode(void);
#endif
