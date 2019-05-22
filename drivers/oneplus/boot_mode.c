#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/oneplus/boot_mode.h>

static enum oem_boot_mode boot_mode = MSM_BOOT_MODE__NORMAL;

char *enum_ftm_mode[] = {"normal",
                         "fastboot",
                         "recovery",
                         "aging",
                         "ftm_at",
                         "ftm_rf",
                         "ftm_wlan",
                         "ftm_mos",
                         "charger"
};

enum oem_boot_mode get_boot_mode(void)
{
    return boot_mode;
}
EXPORT_SYMBOL(get_boot_mode);


static int __init boot_mode_init(void)
{
    char *substrftm = strstr(boot_command_line, "androidboot.ftm_mode=");
    char *substrnormal = strstr(boot_command_line, "androidboot.mode=");
    char *substrftmstr = NULL ;
    char *substrnormalstr = NULL ;
       
    substrftmstr = substrftm + strlen("androidboot.ftm_mode=");
    substrnormalstr = substrnormal + strlen("androidboot.mode=");

    if(substrftm != NULL && substrftmstr != NULL) {

        if(strncmp(substrftmstr, "ftm_at", 6) == 0)
            boot_mode = MSM_BOOT_MODE__FACTORY;
        else if(strncmp(substrftmstr, "ftm_rf", 6) == 0)
            boot_mode = MSM_BOOT_MODE__RF;
        else if(strncmp(substrftmstr, "ftm_wlan", 8) == 0)
            boot_mode = MSM_BOOT_MODE__WLAN;
        else if(strncmp(substrftmstr, "ftm_mos", 7) == 0)
            boot_mode = MSM_BOOT_MODE__MOS;
        else if(strncmp(substrftmstr, "ftm_recovery", 12) == 0)
            boot_mode = MSM_BOOT_MODE__RECOVERY;
        else if(strncmp(substrftmstr, "ftm_aging", 9) == 0)
            boot_mode = MSM_BOOT_MODE__AGING;
   }else if(substrnormal != NULL && substrnormalstr != NULL) {

        if(strncmp(substrnormalstr, "recovery", 8) == 0)
            boot_mode = MSM_BOOT_MODE__RECOVERY;
        else if(strncmp(substrnormalstr, "charger", 7) == 0)
            boot_mode = MSM_BOOT_MODE__CHARGE;
   }

    pr_info("kernel boot_mode = %s[%d]\n",enum_ftm_mode[boot_mode],boot_mode);
    return 0;
}
arch_initcall(boot_mode_init);
