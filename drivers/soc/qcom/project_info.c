/*For OEM project information
*such as project name, hardware ID
*/

#include <linux/export.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/project_info.h>

#include <soc/qcom/smem.h>

#include <linux/gpio.h>



static struct component_info component_info_desc[COMPONENT_MAX];

static struct kobject *project_info_kobj;
static struct project_info * project_info_desc;

static struct kobject *component_info;

static ssize_t project_info_get(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t component_info_get(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(project_name, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(hw_id, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(rf_id_v1, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(rf_id_v2, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(rf_id_v3, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(modem, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(operator_no, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(ddr_manufacture_info, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(ddr_raw, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(ddr_column, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(ddr_reserve_info, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(secboot_status, S_IRUGO, project_info_get, NULL);
static DEVICE_ATTR(platform_id, S_IRUGO, project_info_get, NULL);


uint32 get_secureboot_fuse_status(void)
{
    void __iomem *oem_config_base;
    uint32 secure_oem_config = 0;

    oem_config_base = ioremap(0x70378 , 10);
    secure_oem_config = __raw_readl(oem_config_base);
    iounmap(oem_config_base);
    pr_err("secure_oem_config 0x%x\n", secure_oem_config);

    return secure_oem_config;
}

static ssize_t project_info_get(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	if(project_info_desc)
	{
		if (attr == &dev_attr_project_name)
			return sprintf(buf, "%s\n", project_info_desc->project_name);
		if (attr == &dev_attr_hw_id)
			return sprintf(buf, "%d\n", project_info_desc->hw_version);
		if (attr == &dev_attr_rf_id_v1)
			return sprintf(buf, "%d\n", project_info_desc->rf_v1);
		if (attr == &dev_attr_rf_id_v2)
			return sprintf(buf, "%d\n", project_info_desc->rf_v2);
		if (attr == &dev_attr_rf_id_v3)
			return sprintf(buf, "%d\n", project_info_desc->rf_v3);
		if (attr == &dev_attr_modem)
			return sprintf(buf, "%d\n", project_info_desc->modem);
		if (attr == &dev_attr_operator_no)
			return sprintf(buf, "%d\n", project_info_desc->operator);
		if (attr == &dev_attr_ddr_manufacture_info)
			return sprintf(buf, "%d\n", project_info_desc->ddr_manufacture_info);
		if (attr == &dev_attr_ddr_raw)
			return sprintf(buf, "%d\n", project_info_desc->ddr_raw);
		if (attr == &dev_attr_ddr_column)
			return sprintf(buf, "%d\n", project_info_desc->ddr_column);
		if (attr == &dev_attr_ddr_reserve_info)
			return sprintf(buf, "%d\n", project_info_desc->ddr_reserve_info);
		if (attr == &dev_attr_secboot_status) {
			//return sprintf(buf, "%d\n", project_info_desc->ddr_reserve_info);
			return sprintf(buf, "%d\n",get_secureboot_fuse_status());
		}
		if (attr == &dev_attr_platform_id)
			return sprintf(buf, "%d\n", project_info_desc->platform_id);
	}

	return -EINVAL;

}

static struct attribute *project_info_sysfs_entries[] = {
	&dev_attr_project_name.attr,
	&dev_attr_hw_id.attr,
	&dev_attr_rf_id_v1.attr,
	&dev_attr_rf_id_v2.attr,
	&dev_attr_rf_id_v3.attr,
	&dev_attr_modem.attr,
	&dev_attr_operator_no.attr,
	&dev_attr_ddr_manufacture_info.attr,
	&dev_attr_ddr_raw.attr,
	&dev_attr_ddr_column.attr,
	&dev_attr_ddr_reserve_info.attr,
	&dev_attr_secboot_status.attr,
	&dev_attr_platform_id.attr,
	NULL,
};

static struct attribute_group project_info_attr_group = {
	.attrs	= project_info_sysfs_entries,
};

static DEVICE_ATTR(ddr, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(emmc, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(f_camera, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(r_camera, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(tp, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(lcd, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(wcn, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(l_sensor, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(g_sensor, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(m_sensor, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(gyro, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(backlight, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(mainboard, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(fingerprints, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(touch_key, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(ufs, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(Aboard, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(nfc, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(fast_charge, S_IRUGO, component_info_get, NULL);
static DEVICE_ATTR(cpu, S_IRUGO, component_info_get, NULL);


char *get_component_version( enum COMPONENT_TYPE type)
{
	if(type >= COMPONENT_MAX){
		pr_err("%s == type %d invalid\n",__func__,type);
		return "N/A";
	}
	return component_info_desc[type].version?:"N/A";
}

char *get_component_manufacture( enum COMPONENT_TYPE type)
{
	if(type >= COMPONENT_MAX){
		pr_err("%s == type %d invalid\n",__func__,type);
		return "N/A";
	}
	return component_info_desc[type].manufacture?:"N/A";

}

int push_component_info(enum COMPONENT_TYPE type, char *version, char * manufacture)
{
	if(type >= COMPONENT_MAX){
			pr_err("%s == type %d invalid\n",__func__,type);
			return -1;
	}
	component_info_desc[type].version = version;
	component_info_desc[type].manufacture = manufacture;

	return 0;
}
EXPORT_SYMBOL(push_component_info);

int reset_component_info(enum COMPONENT_TYPE type)
{
	if(type >= COMPONENT_MAX){
			pr_err("%s == type %d invalid\n",__func__,type);
			return -1;
	}
	component_info_desc[type].version = NULL;
	component_info_desc[type].manufacture = NULL;

	return 0;
}
EXPORT_SYMBOL(reset_component_info);


static struct attribute *component_info_sysfs_entries[] = {
	&dev_attr_ddr.attr,
	&dev_attr_emmc.attr,
	&dev_attr_f_camera.attr,
	&dev_attr_r_camera.attr,
	&dev_attr_tp.attr,
	&dev_attr_lcd.attr,
	&dev_attr_wcn.attr,
	&dev_attr_l_sensor.attr,
	&dev_attr_g_sensor.attr,
	&dev_attr_m_sensor.attr,
	&dev_attr_gyro.attr,
	&dev_attr_backlight.attr,
	&dev_attr_mainboard.attr,
	&dev_attr_fingerprints.attr,
    &dev_attr_touch_key.attr,
    &dev_attr_ufs.attr,
    &dev_attr_Aboard.attr,
    &dev_attr_nfc.attr,
    &dev_attr_fast_charge.attr,
    &dev_attr_cpu.attr,
	NULL,
};

static struct attribute_group component_info_attr_group = {
	.attrs	= component_info_sysfs_entries,
};

static ssize_t component_info_get(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	if (attr == &dev_attr_ddr)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(DDR), get_component_manufacture(DDR));
	if (attr == &dev_attr_emmc)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(EMMC), get_component_manufacture(EMMC));
	if (attr == &dev_attr_f_camera)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(F_CAMERA), get_component_manufacture(F_CAMERA));
	if (attr == &dev_attr_r_camera)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(R_CAMERA), get_component_manufacture(R_CAMERA));
	if (attr == &dev_attr_tp)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(TP), get_component_manufacture(TP));
	if (attr == &dev_attr_lcd)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(LCD), get_component_manufacture(LCD));
	if (attr == &dev_attr_wcn)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(WCN), get_component_manufacture(WCN));
	if (attr == &dev_attr_l_sensor)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(I_SENSOR), get_component_manufacture(I_SENSOR));
	if (attr == &dev_attr_g_sensor)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(G_SENSOR), get_component_manufacture(G_SENSOR));
	if (attr == &dev_attr_m_sensor)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(M_SENSOR), get_component_manufacture(M_SENSOR));
	if (attr == &dev_attr_gyro)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(GYRO), get_component_manufacture(GYRO));
	if (attr == &dev_attr_backlight)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(BACKLIGHT), get_component_manufacture(BACKLIGHT));
	if (attr == &dev_attr_mainboard)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(MAINBOARD), get_component_manufacture(MAINBOARD));
    if (attr == &dev_attr_fingerprints)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(FINGERPRINTS), get_component_manufacture(FINGERPRINTS));
    if (attr == &dev_attr_touch_key)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(TOUCH_KEY), get_component_manufacture(TOUCH_KEY));
	if (attr == &dev_attr_ufs)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(UFS), get_component_manufacture(UFS));
	if (attr == &dev_attr_Aboard)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(ABOARD), get_component_manufacture(ABOARD));
    if (attr == &dev_attr_nfc)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(NFC), get_component_manufacture(NFC));
	if (attr == &dev_attr_fast_charge)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(FAST_CHARGE), get_component_manufacture(FAST_CHARGE));
	if (attr == &dev_attr_cpu)
		return sprintf(buf, "VER:\t%s\nMANU:\t%s\n", get_component_version(CPU), get_component_manufacture(CPU));
	return -EINVAL;
}

static int __init project_info_init_sysfs(void)
{
	int error = 0;

	project_info_kobj = kobject_create_and_add("project_info", NULL);
	if (!project_info_kobj)
		return -ENOMEM;
	error = sysfs_create_group(project_info_kobj, &project_info_attr_group);
	if (error){
		pr_err("project_info_init_sysfs project_info_attr_group failure\n");
		return error;
	}

	component_info = kobject_create_and_add("component_info", project_info_kobj);
	pr_info("project_info_init_sysfs success\n");
	if (!component_info)
		return -ENOMEM;

	error = sysfs_create_group(component_info, &component_info_attr_group);
	if (error){
		pr_err("project_info_init_sysfs project_info_attr_group failure\n");
		return error;
	}
	return 0;
}

late_initcall(project_info_init_sysfs);

struct ddr_manufacture{
	int id;
	char name[20];
};
//ddr id and ddr name
static char ddr_version[32] = {0};
static char ddr_manufacture[20] = {0};
static char cpu_type[20] = {0};

struct ddr_manufacture ddr_manufacture_list[]={
     {1,"Samsung "},
	 {2,"Qimonda "},
	 {3,"Elpida "},
	 {4,"Etpon "},
	 {5,"Nanya "},
	 {6,"Hynix "},
	 {7,"Mosel "},
	 {8,"Winbond "},
	 {9,"Esmt "},
	 {255, "Micron"},
	 {0,"Unknown"},
};


struct cpu_list{
	int id;
	char name[20];
};

struct cpu_list cpu_list_msm[]={
     {194,"MSM8974AC "},
	 {217,"MSM8974AA "},
	 {218,"MSM8974AB "},
	 {207,"MSM8994 "},
	 {246,"MSM8996 "},
	 {302,"MSM8996L "},
	 {305,"MSM8996SG "},
	 {310,"MSM8996AU "},
	 {0,"Unknown"},
};


void get_ddr_manufacture_name(void){
	int i;

	if(project_info_desc)
	{
		int id = project_info_desc->ddr_manufacture_info;
		for(i = 0; i < (sizeof(ddr_manufacture_list)/sizeof(ddr_manufacture_list[0])); i++)
		{
			if(ddr_manufacture_list[i].id == id)
			{
				sprintf(ddr_manufacture, "%s", ddr_manufacture_list[i].name);
				break;
			}
		}
	}
}

void get_cpu_type(void){
	int i;

	if(project_info_desc)
	{
		int id = project_info_desc->platform_id;
		for(i = 0; i < (sizeof(cpu_list_msm)/sizeof(cpu_list_msm[0])); i++)
		{
			if(cpu_list_msm[i].id == id)
			{
				sprintf(cpu_type, "%s", cpu_list_msm[i].name);
				break;
			}
		}
	}
}

static char mainboard_version[16] = {0};
static char mainboard_manufacture[8] = {'O', 'N', 'E', 'P', 'L', 'U', 'S','\0'};
//extern unsigned long totalram_pages __read_mostly;
static char Aboard_version[4] = {0};
static int Aboard_gpio = 130;//msm8996 kernel gpio_num == hw gpio num now

void init_a_board_gpio(void)
{
    if(gpio_is_valid(Aboard_gpio))
    {
		if (gpio_request(Aboard_gpio, "ID_ANT_PCBA")) {
			pr_err("%s: gpio_request(%d) fail!\n",__func__,Aboard_gpio);
		}
        gpio_direction_input(Aboard_gpio);
    }
    else
        pr_err("%s: Aboard_gpio %d is invalid\n",__func__,Aboard_gpio);
}

uint32 get_hw_version(void)
{
	project_info_desc = smem_find(SMEM_PROJECT_INFO,
				sizeof(struct project_info),
				0,
				SMEM_ANY_HOST_FLAG);

	if (IS_ERR_OR_NULL(project_info_desc))
		pr_err("%s: get project_info failure\n",__func__);
	else
	{
		pr_err("%s: hw version: %d\n",__func__, project_info_desc->hw_version);
		return project_info_desc->hw_version;
	}
	return 0;
}


int __init init_project_info(void)
{
	static bool project_info_init_done;
	int ddr_size;

	if (project_info_init_done)
		return 0;

	project_info_desc = smem_find(SMEM_PROJECT_INFO,
				sizeof(struct project_info),
				0,
				SMEM_ANY_HOST_FLAG);

	if (IS_ERR_OR_NULL(project_info_desc))
	{
		pr_err("%s: get project_info failure\n",__func__);
		return 0;
	}
	else
		pr_err("%s: project_name: %s hw_version: %d rf_v1: %d rf_v2: %d: rf_v3: %d  paltform_id:%d\n",
						__func__, project_info_desc->project_name, project_info_desc->hw_version,
								project_info_desc->rf_v1, project_info_desc->rf_v2, project_info_desc->rf_v3 ,project_info_desc->platform_id);

	//snprintf(mainboard_version, sizeof(mainboard_version), "%d",project_info_desc->hw_version);
	switch(project_info_desc->hw_version) {
		case 11:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "T0");
		    break;
		case 12:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "EVT1");
		    break;
		case 13:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "EVT2");
		    break;
		case 14:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "EVT3");
		    break;
		case 15:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "DVT");
		    break;
		case 16:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "PVT|MP");
		    break;
		case 26:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "PVT");
		    break;
		case 27:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "PVT2");
		    break;
		case 28:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%s %s", project_info_desc->project_name, "PVT3");
		    break;
		default:
		    snprintf(mainboard_version, sizeof(mainboard_version), "%d",project_info_desc->hw_version);
		    break;
	}
	push_component_info(MAINBOARD,mainboard_version, mainboard_manufacture);

    if(project_info_desc->hw_version <= 12)//EVT1    EVB:10 T0:11 EVT1:12 EVT2:13 DVT:14 PVT/MP:15
    {
        init_a_board_gpio();
        snprintf(Aboard_version, sizeof(Aboard_version), "%d",gpio_get_value(Aboard_gpio));
        push_component_info(ABOARD,Aboard_version, mainboard_manufacture);
        pr_err("%s: Aboard_gpio(%d) value(%d)\n",__func__,Aboard_gpio,gpio_get_value(Aboard_gpio));
    }
    else//start from EVT2 use MPP07
    {
        pr_err("%s: aboard_version: %d \n",
						__func__, project_info_desc->rf_v2);
		snprintf(Aboard_version, sizeof(Aboard_version), "%d",project_info_desc->rf_v2);
		push_component_info(ABOARD,Aboard_version, mainboard_manufacture);
    }

	//add ddr row, column information and manufacture name information
	get_ddr_manufacture_name();

	if(totalram_pages > 5*(1<<18)){
		ddr_size = 6;
	}else if(totalram_pages > 4*(1<<18)){
		ddr_size = 5;
	}else if(totalram_pages > 3*(1<<18)){
		ddr_size = 4;
	}else if(totalram_pages > 2*(1<<18)){
		ddr_size = 3;
	}else if(totalram_pages > 1*(1<<18)){
		ddr_size = 2;
	}

	snprintf(ddr_version, sizeof(ddr_version), "size_%dG_r_%d_c_%d", ddr_size, project_info_desc->ddr_raw,project_info_desc->ddr_column);
	push_component_info(DDR,ddr_version, ddr_manufacture);

	get_cpu_type();
	push_component_info(CPU,cpu_type, "Qualcomm");
	project_info_init_done = true;

	return 0;
}

subsys_initcall(init_project_info);
