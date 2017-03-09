#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/oneplus/boot_mode.h>

static enum oem_boot_mode boot_mode = MSM_BOOT_MODE__NORMAL;

char *enum_ftm_mode[] = {"normal", "fastboot","recovery","aging" "ftm_at", "ftm_rf",
"ftm_wlan","ftm_mos","charge"};

enum oem_boot_mode get_boot_mode(void)
{
	return boot_mode;
}
EXPORT_SYMBOL(get_boot_mode);

static int __init boot_mode_init(void)
{
    char *substr = strstr(boot_command_line, "androidboot.ftm_mode=");

    if(NULL == substr)
	return 0;

    substr += strlen("androidboot.ftm_mode=");
    if(substr) {
        if(strncmp(substr, "ftm_at", 6) == 0)
            boot_mode = MSM_BOOT_MODE__FACTORY;
        else if(strncmp(substr, "ftm_rf", 6) == 0)
            boot_mode = MSM_BOOT_MODE__RF;
        else if(strncmp(substr, "ftm_wlan", 8) == 0)
            boot_mode = MSM_BOOT_MODE__WLAN;
        else if(strncmp(substr, "ftm_mos", 7) == 0)
            boot_mode = MSM_BOOT_MODE__MOS;
        else if(strncmp(substr, "ftmrecovery", 11) == 0)
            boot_mode = MSM_BOOT_MODE__RECOVERY;
        else if(strncmp(substr, "ftm_aging", 9) == 0)
            boot_mode = MSM_BOOT_MODE__AGING;

   }

    pr_info("kernel boot_mode = %s[%d]\n",enum_ftm_mode[boot_mode],boot_mode);
    return 0;
}
arch_initcall(boot_mode_init);
