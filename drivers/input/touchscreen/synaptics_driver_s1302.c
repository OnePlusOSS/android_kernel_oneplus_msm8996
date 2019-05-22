/************************************************************************************
 ** File: - /android/kernel/drivers/input/touchscreen/synaptic_s1302.c
 ** Copyright (C), 2008-2012, OEM Mobile Comm Corp., Ltd
 **
 ** Description:
 **      touch panel driver for synaptics
 **      can change MAX_POINT_NUM value to support multipoint
 ** Version: 1.0
 ** Date created: 10:49:46,18/01/2012
 ** Author: Yixue.Ge@BasicDrv.TP
 **
 ** --------------------------- Revision History: --------------------------------
 ** 	<author>	<data>			<desc>
 **  chenggang.li@BSP.TP modified for oem 2014-07-30 14005 tp_driver
 ************************************************************************************/
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>

#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/machine.h>

#include <linux/kthread.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/task_work.h>

#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/timer.h>
#include <linux/time.h>

#ifdef CONFIG_FB
#include <linux/fb.h>
#include <linux/notifier.h>
#endif

#include <linux/input/mt.h>

#include "synaptics_s1302_redremote.h"
//#include <linux/boot_mode.h>
#include <linux/project_info.h>
enum oem_boot_mode{
	MSM_BOOT_MODE__NORMAL,
	MSM_BOOT_MODE__FASTBOOT,
	MSM_BOOT_MODE__RECOVERY,
	MSM_BOOT_MODE__FACTORY,
	MSM_BOOT_MODE__RF,
	MSM_BOOT_MODE__WLAN,
	MSM_BOOT_MODE__MOS,
	MSM_BOOT_MODE__CHARGE,
};

#define get_boot_mode() MSM_BOOT_MODE__NORMAL

/*------------------------------------------------Global Define--------------------------------------------*/
#define TP_TEST_ENABLE 1
#define TPD_DEVICE "HWK,synaptics,s1302"
#define LOG_TAG		"touchkey,s1302"

#define SUPPORT_TP_SLEEP_MODE
#define TYPE_B_PROTOCOL      //Multi-finger operation
#define TP_FW_NAME_MAX_LEN 128

/******************for Red function*****************/
#define CONFIG_SYNAPTIC_RED

/*********************for Debug LOG switch*******************/
#define TPD_ERR(a, arg...)  pr_err(LOG_TAG ": " a, ##arg)
#define TPDTM_DMESG(a, arg...)  printk(LOG_TAG ": " a, ##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
		pr_err(LOG_TAG ": " a,##arg);\
	}while(0)


//#define SUPPORT_FOR_COVER_ESD
#define SUPPORT_VIRTUAL_KEY
/*---------------------------------------------Global Variable----------------------------------------------*/
static unsigned int tp_debug = 0;
static int force_update = 0;
static int key_reverse = 0;
static struct synaptics_ts_data *tc_g = NULL;
int test_err = 0;
static int touchkey_wait_time = 45;
static bool key_disable=false;
/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F11_2D_QUERY_BASE;
static int F11_2D_CMD_BASE;
static int F11_2D_CTRL_BASE;
static int F11_2D_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_CTRL01;

#if TP_TEST_ENABLE
static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00
#endif

/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_ts_resume(struct device *dev);
static int synaptics_ts_suspend(struct device *dev);
static int synaptics_ts_remove(struct i2c_client *client);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force);

static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
		unsigned char addr,unsigned short length,unsigned char *data);

static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
		unsigned char addr, unsigned short length, unsigned char const *data);

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr);

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data);

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr);

//static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
//		unsigned char addr,unsigned short data);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
static int synaptics_soft_reset(struct synaptics_ts_data *ts);
static void synaptics_hard_reset(struct synaptics_ts_data *ts);

/*-------------------------------Using Struct----------------------------------*/
static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct of_device_id synaptics_match_table[] = {
	{ .compatible = TPD_DEVICE,},
	{ },
};

static const struct dev_pm_ops synaptic_pm_ops = {
#ifdef CONFIG_PM
	.suspend = NULL,
	.resume = NULL,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
};

//add by jiachenghui for boot time optimize 2015-5-13
static int probe_ret;
struct synaptics_optimize_data{
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct i2c_client *client;
	const struct i2c_device_id *dev_id;
};
static struct synaptics_optimize_data optimize_data;
static void synaptics_ts_probe_func(struct work_struct *w)
{
	struct i2c_client *client_optimize = optimize_data.client;
	const struct i2c_device_id *dev_id = optimize_data.dev_id;
	TPD_ERR("boot_time: after optimize call [synaptics_ts_probe] on cpu %d\n",smp_processor_id());
	probe_ret = synaptics_ts_probe(client_optimize,dev_id);
	TPD_ERR("boot_time: synaptics_ts_probe return %d\n", probe_ret);
}

static int oem_synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	optimize_data.client = client;
	optimize_data.dev_id = id;
	optimize_data.workqueue = create_workqueue("tc_probe_optimize");
	INIT_DELAYED_WORK(&(optimize_data.work), synaptics_ts_probe_func);
	TPD_ERR("boot_time: before optimize [synaptics_ts_probe]  on cpu %d\n",smp_processor_id());
	//spin_lock_irqsave(&oem_lock, flags);
	if(get_boot_mode() == MSM_BOOT_MODE__NORMAL)
	{
	    //add by lifeng@bsp 2015-12-10 for only one cpu on line
		for (i = 0; i < NR_CPUS; i++){
            TPD_ERR("boot_time: [synaptics_ts_probe] CPU%d is %s\n",i,cpu_is_offline(i)?"offline":"online");
            if (cpu_online(i) && (i != smp_processor_id()))
                break;
	    }
        queue_delayed_work_on(i != NR_CPUS?i:0,optimize_data.workqueue,&(optimize_data.work),msecs_to_jiffies(300));
        //end add by lifeng@bsp 2015-12-10 for only one cpu on line
	}else{
		queue_delayed_work_on(0,optimize_data.workqueue,&(optimize_data.work),msecs_to_jiffies(300));
	}
	return probe_ret;
}
//end add by jiachenghui for boot time optimize 2015-5-13

static struct i2c_driver tc_i2c_driver = {
//add by jiachenghui for boot time optimize 2015-5-13
	.probe		= oem_synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
	.id_table	= synaptics_ts_id,
	.driver = {
		//		.owner  = THIS_MODULE,
		.name	= TPD_DEVICE,
		.of_match_table =  synaptics_match_table,
		.pm = &synaptic_pm_ops,
	},
};

static unsigned int polling_interval = 30;
module_param(polling_interval, uint, 0644);

struct synaptics_ts_data {
	struct i2c_client *client;
	struct mutex mutex;
	int irq;
	int irq_gpio;
	int reset_gpio;
	int en3v_gpio;
	int enable_remote;
	int pre_btn_state;
	int is_suspended;
	uint32_t irq_flags;
	struct delayed_work dwork;
#ifdef SUPPORT_FOR_COVER_ESD
    bool cover_reject;
    bool key_back;
    bool key_app_select;
    struct hrtimer timer;
#endif
	uint32_t using_polling;
	struct delayed_work get_touchkey_work;
	struct work_struct  work;
	struct delayed_work speed_up_work;
	struct input_dev *input_dev;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif

	/******power*******/
	struct regulator *vdd_2v8;
	struct regulator *vcc_i2c_1v8;

	/*pinctrl******/
	struct device						*dev;
	struct pinctrl 						*pinctrl;
	struct pinctrl_state 				*pinctrl_state_active;
	struct pinctrl_state 				*pinctrl_state_suspend;

	/*******for FW update*******/
	bool suspended;
	bool loading_fw;
	char fw_name[TP_FW_NAME_MAX_LEN];
	char fw_id[12];
	char manu_name[12];
};

static int tc_hw_pwron(struct synaptics_ts_data *ts)
{
	int rc;

	//enable the 2v8 power
	if (!IS_ERR(ts->vdd_2v8)) {
		rc = regulator_enable(ts->vdd_2v8);
		if(rc){
			dev_err(&ts->client->dev,
					"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	msleep(1);

	if( ts->en3v_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the en3v_gpio\n");
		gpio_direction_output(ts->en3v_gpio, 1);
	}

	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		rc = regulator_enable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
		}
	}
	msleep(3);
	if( ts->reset_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 1);
	}
    msleep(500);
	return rc;
}

static int tc_hw_pwroff(struct synaptics_ts_data *ts)
{
	int rc = 0;
	if( ts->reset_gpio > 0 ) {
		TPD_ERR("synaptics:disable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 0);
	}

	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		rc = regulator_disable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}

	if (!IS_ERR(ts->vdd_2v8)) {
		rc = regulator_disable(ts->vdd_2v8);
		if (rc) {
			dev_err(&ts->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	}

	if( ts->en3v_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the en3v_gpio\n");
		gpio_direction_output(ts->en3v_gpio, 0);
	}
	return rc;
}

static int tc_power(struct synaptics_ts_data *ts, unsigned int on)
{
	int ret;
	if(on)
		ret = tc_hw_pwron(ts);
	else
		ret = tc_hw_pwroff(ts);

	return ret;
}

static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
	uint8_t buf[4];
	int ret;
	memset(buf, 0, sizeof(buf));
	ret = synaptics_rmi4_i2c_write_byte( ts->client, 0xff, 0x0 );
	if( ret < 0 ){
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xDD, 4, &(buf[0x0]));
	if( ret < 0 ){
		TPD_ERR("failed for page select!\n");
		return -1;
	}

	F11_2D_QUERY_BASE = buf[0];
	F11_2D_CMD_BASE = buf[1];
	F11_2D_CTRL_BASE = buf[2];
	F11_2D_DATA_BASE = buf[3];

	TPD_ERR("F11_2D_QUERY_BASE = %x \n \
			F11_2D_CMD_BASE  = %x \n\
			F11_2D_CTRL_BASE	= %x \n\
			F11_2D_DATA_BASE	= %x \n\
			",F11_2D_QUERY_BASE,F11_2D_CMD_BASE,F11_2D_CTRL_BASE,F11_2D_DATA_BASE);


	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE3, 4, &(buf[0x0]));
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2];
	F01_RMI_DATA_BASE = buf[3];
	TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
			F01_RMI_CMD_BASE  = %x \n\
			F01_RMI_CTRL_BASE	= %x \n\
			F01_RMI_DATA_BASE	= %x \n\
			", F01_RMI_QUERY_BASE, F01_RMI_CMD_BASE, F01_RMI_CTRL_BASE, F01_RMI_DATA_BASE);

	ret = synaptics_rmi4_i2c_read_block( ts->client, 0xE9, 4, &(buf[0x0]) );
	F34_FLASH_QUERY_BASE = buf[0];
	F34_FLASH_CMD_BASE = buf[1];
	F34_FLASH_CTRL_BASE = buf[2];
	F34_FLASH_DATA_BASE = buf[3];
	TPD_DEBUG("F34_FLASH_QUERY_BASE = %x \n\
			F34_FLASH_CMD_BASE	= %x \n\
			F34_FLASH_CTRL_BASE	= %x \n\
			F34_FLASH_DATA_BASE	= %x \n\
			", F34_FLASH_QUERY_BASE, F34_FLASH_CMD_BASE, F34_FLASH_CTRL_BASE, F34_FLASH_DATA_BASE);

	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if( ret < 0 ){
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2];
	F51_CUSTOM_DATA_BASE = buf[3];

	TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
			F51_CUSTOM_CMD_BASE  = %x \n\
			F51_CUSTOM_CTRL_BASE    = %x \n\
			F51_CUSTOM_DATA_BASE    = %x \n\
			", F51_CUSTOM_QUERY_BASE, F51_CUSTOM_CMD_BASE, F51_CUSTOM_CTRL_BASE, F51_CUSTOM_DATA_BASE);

#if TP_TEST_ENABLE
    ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	TPD_DEBUG("F54_QUERY_BASE = %x \n\
			F54_CMD_BASE  = %x \n\
			F54_CTRL_BASE	= %x \n\
			F54_DATA_BASE	= %x \n\
			", F54_ANALOG_QUERY_BASE, F54_ANALOG_COMMAND_BASE , F54_ANALOG_CONTROL_BASE, F54_ANALOG_DATA_BASE);
#endif
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	return 0;
}

static int synaptics_read_product_id(struct synaptics_ts_data *ts,char *id)
{
	uint8_t buf[10]={ 0 };
	int ret ;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPD_ERR("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0x83, sizeof(buf), buf);
	if( ret < 0 ){
		TPD_ERR("synaptics_read_product_id error %s\n",&buf[0]);
		return -1;
	}
	memcpy(id,&buf[0],sizeof(buf));
	TPD_ERR("product id is %s\n",&buf[0]);
	return 0;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t abs_status_int;

	return 0;

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPDTM_DMESG("%s: select page failed ret = %d\n",__func__,
		    ret);
		return -1;
	}
	if( enable ) {
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE+1);
		if( ret < 0 ) {
			TPDTM_DMESG("%s :clear interrupt bits failed\n",__func__);
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL01, abs_status_int);
	if( ret < 0 ) {
		TPDTM_DMESG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n", __func__, abs_status_int);
		return -1;
	}
	return 0;
}

static void delay_qt_ms(unsigned long  w_ms)
{
	unsigned long i;
	unsigned long j;
	for(i = 0; i < w_ms; i++) {
		for (j = 0; j < 1000; j++) {
			udelay(1);
		}
	}
}

static void int_state(struct synaptics_ts_data *ts)
{
	int ret = -1;
	return;
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
	if(ret){
		TPD_ERR("%s:cannot reset touch panel \n",__func__);
		return;
	}
	delay_qt_ms(170);
	TPD_DEBUG("%s %d\n",__func__,__LINE__);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL_BASE, 0x80);
	if( ret < 0 ){
		TPD_ERR("%s:to wakeup failed\n", __func__);
	}
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret){
		TPD_DEBUG("%s:cannot  enable interrupt \n",__func__);
		return;
	}
}

//Added for larger than 32 length read!
static int synaptics_rmi4_i2c_read_block(struct i2c_client* client,
		unsigned char addr,unsigned short length,unsigned char *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};
	buf = addr & 0xFF;
	for( retry = 0; retry < 2; retry++ ) {
		if( i2c_transfer(client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if( retry == 2 ) {
		dev_err(&client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		//rst_flag_counter = 1;//reset tp
		retval = -5;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

static int synaptics_rmi4_i2c_write_block(struct i2c_client* client,
		unsigned char addr, unsigned short length, unsigned char const *data)
{
	int retval;
	unsigned char retry;
	unsigned char buf[length + 1];
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = addr & 0xff;
	memcpy(&buf[1], &data[0], length);

	for (retry = 0; retry < 2; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1) {
			retval = length;
			break;
		}
		msleep(20);
	}
	if (retry == 2) {
		//rst_flag_counter = 1;//rest tp
		retval = -EIO;
	} else {
		//rst_flag_counter = 0;
	}
	return retval;
}

static int synaptics_rmi4_i2c_read_byte(struct i2c_client* client,
		unsigned char addr)
{
	int retval = 0;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,1,buf);
	if(retval >= 0)
		retval = buf[0]&0xff;
	return retval;
}

static int synaptics_rmi4_i2c_write_byte(struct i2c_client* client,
		unsigned char addr,unsigned char data)
{
	int retval;
	unsigned char data_send = data;
	retval = synaptics_rmi4_i2c_write_block(client,addr,1,&data_send);
	return retval;
}

static int synaptics_rmi4_i2c_read_word(struct i2c_client* client,
		unsigned char addr)
{
	int retval;
	unsigned char buf[2] = {0};
	retval = synaptics_rmi4_i2c_read_block(client,addr,2,buf);
	if(retval >= 0)
		retval = buf[1]<<8|buf[0];
	return retval;
}

static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
		unsigned char addr,unsigned short data)
{
	int retval;
	unsigned char buf[2] = {data&0xff,(data>>8)&0xff};
	retval = synaptics_rmi4_i2c_write_block(client,addr,2,buf);
	if(retval >= 0)
		retval = buf[1]<<8|buf[0];
	return retval;
}

static char log_count = 0;
static bool is_report_key = true;
#define REP_KEY_BACK (key_reverse?(KEY_APPSELECT):(KEY_BACK))
#define REP_KEY_MENU (key_reverse?(KEY_BACK):(KEY_APPSELECT))

#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/29, add flag to enable virtual key
bool virtual_key_enable = false;
EXPORT_SYMBOL(virtual_key_enable);
struct completion key_cm;
bool key_appselect_pressed = false;
bool key_back_pressed = false;
bool check_key_down= false;
EXPORT_SYMBOL(key_appselect_pressed);
EXPORT_SYMBOL(key_back_pressed);
EXPORT_SYMBOL(key_cm);
EXPORT_SYMBOL(check_key_down);

extern void int_touch(void);

static void int_virtual_key(struct synaptics_ts_data *ts )
{

    int ret;
    int button_key;
    long time =0 ;
    bool key_up_report = false;

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x02 );
    if (ret < 0) {
        TPD_ERR("%s: Failed to change page 2!!\n",
                __func__);
        return;
    }

    button_key = synaptics_rmi4_i2c_read_byte(ts->client,0x00);
    if (6 == (++log_count % 12))
        printk("%s	button_key:%d   pre_btn_state:%d\n",__func__,button_key,ts->pre_btn_state);
    if((button_key & 0x01) && !(ts->pre_btn_state & 0x01))//back
    {
	key_appselect_pressed = true;
    }else if(!(button_key & 0x01) && (ts->pre_btn_state & 0x01)){
	key_appselect_pressed = false;
	key_up_report = true;
    }

    if((button_key & 0x02) && !(ts->pre_btn_state & 0x02))//menu
    {
	key_back_pressed = true;
    }else if(!(button_key & 0x02) && (ts->pre_btn_state & 0x02)){
	key_back_pressed = false;
	key_up_report = true;
    }

    if((button_key & 0x04) && !(ts->pre_btn_state & 0x04))//home
    {
        input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);//KEY_HOMEPAGE
        input_sync(ts->input_dev);
    }else if(!(button_key & 0x04) && (ts->pre_btn_state & 0x04)){
        input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
        input_sync(ts->input_dev);
    }
    if(key_up_report){
        reinit_completion(&key_cm);
        time = wait_for_completion_timeout(&key_cm,msecs_to_jiffies(touchkey_wait_time));
        if (!time){
		check_key_down = false;
		int_touch();
	}
    }else{
        check_key_down = true;
        int_touch();
    }
    ts->pre_btn_state = button_key & 0x07;
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
    if (ret < 0) {
        TPD_ERR("%s: Failed to change page 2!!\n",
                __func__);
        return;
    }
    return;
}
#endif
static void int_key(struct synaptics_ts_data *ts )
{

    int ret;
	int button_key;

    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x02 );
    if (ret < 0) {
        TPD_ERR("%s: line[%d]Failed to change page!!\n",
                __func__,__LINE__);
        return;
    }

    button_key = synaptics_rmi4_i2c_read_byte(ts->client,0x00);
    if (6 == (++log_count % 12))
        TPD_ERR("touch_key[0x%x],touchkey_state[0x%x]\n",button_key,ts->pre_btn_state);
    if (!is_report_key)
        return;
    if((button_key & 0x01) && !(ts->pre_btn_state & 0x01) && !key_disable)//back
    {
        input_report_key(ts->input_dev, REP_KEY_MENU, 1);
        input_sync(ts->input_dev);
    }else if(!(button_key & 0x01) && (ts->pre_btn_state & 0x01) && !key_disable){
        input_report_key(ts->input_dev, REP_KEY_MENU, 0);
        input_sync(ts->input_dev);
    }

    if((button_key & 0x02) && !(ts->pre_btn_state & 0x02) && !key_disable)//menu
    {
        input_report_key(ts->input_dev, REP_KEY_BACK, 1);
        input_sync(ts->input_dev);
    }else if(!(button_key & 0x02) && (ts->pre_btn_state & 0x02) && !key_disable){
        input_report_key(ts->input_dev, REP_KEY_BACK, 0);
        input_sync(ts->input_dev);
    }

    if((button_key & 0x04) && !(ts->pre_btn_state & 0x04))//home
    {
        input_report_key(ts->input_dev, KEY_HOMEPAGE, 1);//KEY_HOMEPAGE
        input_sync(ts->input_dev);
    }else if(!(button_key & 0x04) && (ts->pre_btn_state & 0x04)){
        input_report_key(ts->input_dev, KEY_HOMEPAGE, 0);
        input_sync(ts->input_dev);
    }

    ts->pre_btn_state = button_key & 0x07;
	//input_sync(ts->input_dev);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
    if (ret < 0) {
        TPD_ERR("%s: line[%d]Failed to change page!!\n",
                __func__,__LINE__);
        return;
    }
    return;
}
#ifdef SUPPORT_FOR_COVER_ESD
#define TIMER_NS    100000000
static bool is_in_cover = false;

static void int_key_cover(struct synaptics_ts_data *ts )
{

    int ret;
	int button_key;
    if (!is_report_key)
        return;
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x02 );
    if (ret < 0) {
        TPD_ERR("%s: line[%d]Failed to change page!!\n",
                __func__,__LINE__);
        return;
    }

    button_key = synaptics_rmi4_i2c_read_byte(ts->client,0x00);
    TPD_DEBUG("button_key[%d]\n",button_key);
    if (button_key == 0x03){
        is_in_cover = true;
        ts->key_back = false;
        ts->key_app_select = false;
        ret = hrtimer_cancel(&ts->timer);
        TPD_DEBUG("timer cancel ret[%s]\n",ret?"active":"no active");  
    }
    if (button_key == 0x00){
        is_in_cover = false;
    }
    if (is_in_cover){
        ts->pre_btn_state = button_key & 0x07;
        return;
    }
    if((button_key & 0x01) && !(ts->pre_btn_state & 0x01))//back
    {
        hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
        ts->key_back = true;
    }else if(!(button_key & 0x01) && (ts->pre_btn_state & 0x01)){
        ts->key_back = false;
        input_report_key(ts->input_dev, REP_KEY_BACK, 0);
        input_sync(ts->input_dev);
    }

    if((button_key & 0x02) && !(ts->pre_btn_state & 0x02))//menu
    {
        hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
        ts->key_app_select = true;
    }else if(!(button_key & 0x02) && (ts->pre_btn_state & 0x02)){
        ts->key_app_select = false;
        input_report_key(ts->input_dev, REP_KEY_MENU, 0);
        input_sync(ts->input_dev);
    }

    ts->pre_btn_state = button_key & 0x07;
	//input_sync(ts->input_dev);
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
    if (ret < 0) {
        TPD_ERR("%s: line[%d]Failed to change page!!\n",
                __func__,__LINE__);
        return;
    }
    return;
}
static enum hrtimer_restart process_key_timer(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);

    TPD_DEBUG("timer enter!key_back[%d],key_app_select[%d]\n",ts->key_back,ts->key_app_select);
    if (ts->key_back){
        ts->key_back = false;
        input_report_key(ts->input_dev, REP_KEY_BACK, 1);
        input_sync(ts->input_dev);
    }
    else if (ts->key_app_select){
        ts->key_back = false;
        input_report_key(ts->input_dev, REP_KEY_MENU, 1);
        input_sync(ts->input_dev);
    }
	return HRTIMER_NORESTART;
}
static void process_key_timer_init(struct synaptics_ts_data *ts )
{
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = process_key_timer;
}
#endif
static void synaptics_ts_report(struct synaptics_ts_data *ts )
{
    int ret;
    uint8_t status = 0;
    uint8_t inte = 0;
    if( ts->enable_remote) {
        goto END;
    }
    ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00 );
    ret = synaptics_rmi4_i2c_read_word(ts->client, 0x13);

    if( ret < 0 ) {
        TPDTM_DMESG("Synaptic:ret = %d\n", ret);
        synaptics_hard_reset(ts);
        goto END;
    }
    status = ret & 0xff;
    inte = (ret & 0x7f00)>>8;
    if(status) {
        int_state(ts);
        //goto END;
    }
    if( inte & 0x10) {
#if (defined SUPPORT_FOR_COVER_ESD)
        if (ts->cover_reject)
            int_key_cover(ts);
        else
            int_key(ts);
#elif (defined SUPPORT_VIRTUAL_KEY)
        if (virtual_key_enable)
            int_virtual_key(ts);
        else
            int_key(ts);
#endif
    }
END:
	return;
}
static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)dev_id;
	mutex_lock(&ts->mutex);
	synaptics_ts_report(ts);
	mutex_unlock(&ts->mutex);
	return IRQ_HANDLED;
}

static void synaptics_work_thread_fn(struct work_struct *work)
{
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, dwork.work);
	mutex_lock(&ts->mutex);
	synaptics_ts_report(ts);
	mutex_unlock(&ts->mutex);
	schedule_delayed_work(&ts->dwork, msecs_to_jiffies(polling_interval));
}


static int	synaptics_input_init(struct synaptics_ts_data *ts)
{
	int ret = 0;

	TPD_DEBUG("%s is called\n",__func__);
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("synaptics_ts_probe: Failed to allocate input device\n");
		return ret;
	}
    ts->input_dev->name = TPD_DEVICE;;
    ts->input_dev->dev.parent = &ts->client->dev;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->input_dev->keybit);
	set_bit(KEY_HOMEPAGE, ts->input_dev->keybit);
	input_set_drvdata(ts->input_dev, ts);

	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);
		return -1;
	}
	return 0;
}

/*********************FW Update Func******************************************/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int ret;
	char fw_id_temp[12];
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0 ;

	TPD_DEBUG("%s is called\n",__func__);

	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");
		return 0;
	}
	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
				ts->fw_name, ret);
		return ret;
	}
	ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
	if(ret < 0){
		TPD_ERR("FW update not success try again\n");
		ret = synapitcs_ts_update(ts->client, fw->data, fw->size, force);
		if(ret < 0){
			TPD_ERR("FW update failed twice, quit updating process!\n");
			return ret;
		}
	}
	release_firmware(fw);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	ret = synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL_BASE, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	sprintf(fw_id_temp,"0x%x",CURRENT_FIRMWARE_ID);
	strcpy(ts->fw_id,fw_id_temp);
	synaptics_enable_interrupt(ts,1);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);
	input_sync(ts->input_dev);
	return 0;
}

static int synaptics_s1302_fw_show(struct seq_file *seq, void *offset)
{
	struct synaptics_ts_data *ts = tc_g;
	seq_printf(seq, "\
		synaptics ic type is %s\n\
		firmware name is %s\n\
		firmware version is %s\n\
		is update firmware state: %s\n",\
		ts->manu_name,ts->fw_name,ts->fw_id,\
		(ts->loading_fw)?("loading..."):("no update"));
	return 0;
}
static ssize_t synaptics_s1302_fw_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
	int val = 0;
	struct synaptics_ts_data *ts = tc_g;
    if (NULL == tc_g)
        return -EINVAL;
	TPD_ERR("start update ******* fw_name:%s\n",tc_g->fw_name);
	if (t > 2)
		return -EINVAL;

	sscanf(page, "%d", &val);

	if(!val)
		val = force_update;
	if(ts->using_polling)
		cancel_delayed_work_sync(&ts->dwork);
	else
		disable_irq_nosync(tc_g->irq);
	msleep(30);
	mutex_lock(&tc_g->mutex);
	tc_g->loading_fw = true;
	synatpitcs_fw_update(tc_g->dev, val);
	tc_g->loading_fw = false;
	mutex_unlock(&tc_g->mutex);
	if(ts->using_polling)
		schedule_delayed_work(&ts->dwork, msecs_to_jiffies(polling_interval));
	else
		enable_irq(tc_g->irq);
	force_update = 0;
	return t;
}
static int synaptics_s1302_fw_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_s1302_fw_show, inode->i_private);
}
const struct file_operations proc_firmware_update =
{
	.owner		= THIS_MODULE,
	.open		= synaptics_s1302_fw_open,
	.read		= seq_read,
	.write		= synaptics_s1302_fw_write,
	.llseek 	= seq_lseek,
	.release	= single_release,
};
static ssize_t synaptics_s1302_key_reverse_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
	int ret = 0;
	char buf[10]={0};

	if( t > 2)
		return t;
	if( copy_from_user(buf, page, t) ){
		TPD_ERR("%s: read proc input error.\n", __func__);
		return t;
	}

	sscanf(buf, "%d", &ret);
    TPD_ERR("%s key_reverse:%d\n",__func__,ret);
	if( (ret == 0 )||(ret == 1) )
    {
        key_reverse = ret;
    }
    return t;
}
static int synaptics_s1302_key_reverse_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 menu key in %s\n",key_reverse?("right"):("left"));
    return 0 ;
}
static int synaptics_s1302_key_reverse_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_s1302_key_reverse_show, inode->i_private);
}
const struct file_operations proc_reverse_key =
{
	.owner		= THIS_MODULE,
	.open		= synaptics_s1302_key_reverse_open,
	.read		= seq_read,
	.write      = synaptics_s1302_key_reverse_write,
	.llseek 	= seq_lseek,
	.release	= single_release,
};
static int page ,address,block;
static int synaptics_s1302_radd_show(struct seq_file *seq, void *offset)
{
	int ret;
	char buffer[256];
    int i;

	struct synaptics_ts_data *ts = tc_g;
    printk("%s page=0x%x,address=0x%x,block=0x%x\n",__func__,page,address,block);
    ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,page);
    ret = synaptics_rmi4_i2c_read_block(ts->client,address,block,buffer);
    for (i=0;i < block;i++)
    {
        printk("buffer[%d]=0x%x\n",i,buffer[i]);
    }
	seq_printf(seq,"page:0x%x;address:0x%x;buff[%d]:[0]0x%x,[1]0x%x,[2]0x%x,[3]0x%x\n",\
		page,address,block,buffer[0],buffer[1],buffer[2],buffer[3]);
	return 0;
}

static ssize_t synaptics_s1302_radd_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int buf[128];
    int ret,i;
	struct synaptics_ts_data *ts = tc_g;
    int temp_block,wbyte;
    char reg[30];

    ret = sscanf(buffer,"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",\
    &buf[0],&buf[1],&buf[2],&buf[3],&buf[4],&buf[5],&buf[6],&buf[7],&buf[8],&buf[9],\
    &buf[10],&buf[11],&buf[12],&buf[13],&buf[14],&buf[15],&buf[16],&buf[17]);
    for (i = 0;i < ret;i++)
    {
        printk("buf[i]=0x%x,",buf[i]);
    }
    printk("\n");
    page= buf[0];
    address = buf[1];
    temp_block = buf[2];
    wbyte = buf[3];
    if (0xFF == temp_block)//the  mark is to write register else read register
    {
        for (i=0;i < wbyte;i++)
        {
            reg[i] = (char)buf[4+i];
        }
        ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,page);
        ret = synaptics_rmi4_i2c_write_block(ts->client,(char)address,wbyte,reg);
        printk("%s write page=0x%x,address=0x%x\n",__func__,page,address);
        for (i=0;i < wbyte;i++)
        {
            printk("reg=0x%x\n",reg[i]);
        }
    }
    else
        block = temp_block;
	return count;
}
static int synaptics_s1302_radd_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_s1302_radd_show, inode->i_private);
}
const struct file_operations proc_radd =
{
    .owner      = THIS_MODULE,
    .open       = synaptics_s1302_radd_open,
    .read       = seq_read,
    .write      = synaptics_s1302_radd_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};
static ssize_t synaptics_s1302_reset_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    int ret,write_flag;
	struct synaptics_ts_data *ts = tc_g;

	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!!\n",__func__);
		return count;
	}

    ret = sscanf(buffer,"%x",&write_flag);
    TPD_ERR("%s write %d\n",__func__,write_flag);
    if (1 == write_flag)
    {
        ret = synaptics_soft_reset(ts);
    }
    else if(2 == write_flag)
    {
        synaptics_hard_reset(ts);
    }
    else if(3 == write_flag)
    {
        is_report_key = true;
    }
    else if(4 == write_flag)
    {
        is_report_key = false;
    }
	return count;
}
static int synaptics_s1302_reset_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "1:[soft reset],2:[hard reset]\n3:[enable report key],4:[disable report key]\n");
    return 0 ;
}

static int synaptics_s1302_reset_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_s1302_reset_show, inode->i_private);
}

const struct file_operations proc_reset =
{
    .owner      = THIS_MODULE,
    .open       = synaptics_s1302_reset_open,
    .read       = seq_read,
    .write      = synaptics_s1302_reset_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};
static ssize_t synaptics_s1302_debug_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    int ret,write_flag;

    ret = sscanf(buffer,"%x",&write_flag);
    TPD_ERR("%s write %d\n",__func__,write_flag);
    tp_debug = write_flag?1:0;
	return count;
}
static int synaptics_s1302_debug_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 debug log is %s!\n",tp_debug?"on":"off");
    return 0 ;
}

static int synaptics_s1302_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_s1302_debug_show, inode->i_private);
}

const struct file_operations proc_debug =
{
    .owner      = THIS_MODULE,
    .open       = synaptics_s1302_debug_open,
    .read       = seq_read,
    .write      = synaptics_s1302_debug_write,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static ssize_t synaptics_s1302_wait_time_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	int ret,write_flag;
	ret = sscanf(buffer,"%d",&write_flag);
	TPD_ERR("%s write %d\n",__func__,write_flag);
	touchkey_wait_time = write_flag;
	return count;
}

static int synaptics_s1302_wait_time_show(struct seq_file *seq, void *offset)
{
	seq_printf(seq, "s1302 wait_time is %d ms\n",touchkey_wait_time);
	return 0 ;
}


static int synaptics_s1302_wait_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_s1302_wait_time_show, inode->i_private);
}

const struct file_operations setting_wait_time_proc_fops =
{
	.owner      = THIS_MODULE,
	.open       = synaptics_s1302_wait_time_open,
	.read       = seq_read,
	.write      = synaptics_s1302_wait_time_write,
	.llseek     = seq_lseek,
	.release    = single_release,
};


static void synaptics_rawdata_get(struct synaptics_ts_data *ts,char *buffer)
{
	int ret = 0;
	int tx, rx;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0, tmp_h = 0;
	uint16_t count = 0;
	uint16_t delta[2][5];
	unsigned char buf[4];
	char f54_data_base_add,f54_command_base_add,f54_control_base_add;

	/*disable irq when read data from IC*/
	memset(delta,0,sizeof(delta));
	if(ts->using_polling)
		cancel_delayed_work_sync(&ts->dwork);
	else
		disable_irq_nosync(ts->irq);
	delay_qt_ms(30);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	f54_command_base_add = buf[1];
	f54_control_base_add = buf[2];
	f54_data_base_add = buf[3];
	TPD_DEBUG("buf0[0x%x],buf1[0x%x],buf2[0x%x],buf3[0x%x]\n",buf[0],buf[1],buf[2],buf[3]);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, f54_data_base_add, 0x02);//select report type 0x02
	ret = synaptics_rmi4_i2c_write_word(ts->client, f54_data_base_add+1, 0x00);//set fifo 00
	ret = synaptics_rmi4_i2c_write_byte(ts->client, f54_command_base_add, 0X01);//get report
	//checkCMD();
	do {
		delay_qt_ms(50); //wait 10ms
		ret = synaptics_rmi4_i2c_read_byte(ts->client, f54_command_base_add);
		count++;
	}while( (ret > 0x00) && (count < 30) );
	count = 0;
	for( tx = 0; tx < 2; tx++ ){
		for( rx = 0; rx < 5; rx++ ){
			ret = synaptics_rmi4_i2c_read_byte(ts->client, f54_data_base_add+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(ts->client, f54_data_base_add+3);
			tmp_h = ret&0xff;
			delta[tx][rx] = (tmp_h<<8)|tmp_l;
			pr_err("delta[%d][%d]%3d\n",tx,rx, delta[tx][rx]);
			if( tx==0 && rx==3 && delta[tx][rx] < 1000)
				num_read_chars += sprintf(&(buffer[num_read_chars]), "left:%d\n", delta[tx][rx]);
			if( tx==0 && rx==4 && delta[tx][rx] < 1000)
				num_read_chars += sprintf(&(buffer[num_read_chars]), "right:%d\n", delta[tx][rx]);
			//num_read_chars += sprintf(&(buffer[num_read_chars]), "%3d ", delta[tx][rx]);
		}
	}
	//ret = i2c_smbus_write_byte_data(ts->client,f54_command_base_add,0X02);
	delay_qt_ms(60);
	synaptics_enable_interrupt(ts, 1);
	if(ts->using_polling)
		schedule_delayed_work(&ts->dwork, msecs_to_jiffies(polling_interval));
	else
		enable_irq(ts->irq);

}
static int synaptics_key_strength_show(struct seq_file *seq, void *offset)
{
    char buffer[128];
    struct synaptics_ts_data *ts = tc_g;

    memset(buffer,0,sizeof(buffer));
	mutex_lock(&ts->mutex);
    synaptics_rawdata_get(ts,buffer);
	mutex_unlock(&ts->mutex);
	seq_printf(seq, "%s\n",buffer);
	return 0;
}

static int synaptics_key_strength_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_key_strength_show, inode->i_private);
}
const struct file_operations proc_key_strength =
{
	.owner		= THIS_MODULE,
	.open		= synaptics_key_strength_open,
	.read		= seq_read,
	.llseek 	= seq_lseek,
	.release	= single_release,
};
static int16_t delta_baseline[30][30];
static void checkCMD(void)
{
	int ret;
	int flag_err = 0;

	TPD_DEBUG("<kernel> enter checkCMD!\n");
	do {
		delay_qt_ms(30); //wait 10ms
		ret = synaptics_rmi4_i2c_read_byte(tc_g->client, F54_ANALOG_COMMAND_BASE);
		flag_err++;
		TPD_ERR("try read touch ic %d time \n", flag_err);
	}while( (ret > 0x00) && (flag_err < 30) );
	//}while( ret != 0x00 );
	if( ret > 0x00 )
		TPD_ERR("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);
}

static ssize_t tp_baseline_show(struct file *file, char __user *buf, size_t size, loff_t *ppos)

{
	int ret = 0;
	int x,y;
	char * kernel_buf;
	ssize_t num_read_chars = 0;
	//uint8_t tmp = 0;
	uint8_t tmp_old = 0;
//	uint8_t tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint16_t count = 0;
	//struct synaptics_ts_data *ts = tc_g;
	int16_t baseline_data[2][5];
	char f54_data_base_add,f54_command_base_add,f54_control_base_add;

	if(tc_g->is_suspended == 1)
		return count;
	if(!tc_g)
		return count;

	kernel_buf = kmalloc(4096, GFP_KERNEL);
	if(kernel_buf == NULL)
	{
		TPD_ERR("kmalloc error!\n");
		return 0;
	}
	if(tc_g->using_polling)
		cancel_delayed_work_sync(&tc_g->dwork);
	else
		disable_irq(tc_g->irq);
	msleep(30);

	memset(delta_baseline, 0, sizeof(delta_baseline));
	ret = synaptics_rmi4_i2c_write_byte(tc_g->client, 0xff, 0x1);
	ret = synaptics_rmi4_i2c_read_block(tc_g->client, 0xE9, 4, &(buf[0x0]));
	f54_command_base_add = buf[1];
	f54_control_base_add = buf[2];
	f54_data_base_add = buf[3];
	TPD_DEBUG("buf0[0x%x],buf1[0x%x],buf2[0x%x],buf3[0x%x]\n",buf[0],buf[1],buf[2],buf[3]);
	ret = synaptics_rmi4_i2c_write_byte(tc_g->client, f54_data_base_add, 0x02);//select report type 0x02
	ret = synaptics_rmi4_i2c_write_word(tc_g->client, f54_data_base_add+1, 0x00);//set fifo 00
	ret = synaptics_rmi4_i2c_write_byte(tc_g->client, f54_command_base_add, 0X01);//get report
	mutex_lock(&tc_g->mutex);

	TPDTM_DMESG("\n wanghao test start\n");
	TPDTM_DMESG("\n step 1:select report type 0x03 baseline\n");

	//step 1:check raw capacitance.
	ret = synaptics_rmi4_i2c_write_byte(tc_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPD_DEBUG("read_baseline: synaptics_rmi4_i2c_write_byte failed \n");
		//return sprintf(buf, "i2c err!");
	}

	ret = synaptics_rmi4_i2c_write_byte(tc_g->client, F54_ANALOG_CONTROL_BASE+81, 0x01);
	ret = synaptics_rmi4_i2c_read_byte(tc_g->client, F54_ANALOG_CONTROL_BASE+86);
	tmp_old = ret & 0xff;
	TPDTM_DMESG("ret = %x ,tmp_old =%x ,tmp_new = %x\n", ret,tmp_old, (tmp_old & 0xef));
	ret = synaptics_rmi4_i2c_write_byte(tc_g->client, F54_ANALOG_CONTROL_BASE+86, (tmp_old & 0xef));
	ret = synaptics_rmi4_i2c_write_word(tc_g->client, F54_ANALOG_COMMAND_BASE, 0x04);
	if(!(tc_g->using_polling)){
		ret = synaptics_rmi4_i2c_write_byte(tc_g->client, F54_ANALOG_CONTROL_BASE+29, 0x01);// Forbid NoiseMitigation
		ret = synaptics_rmi4_i2c_write_word(tc_g->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
		checkCMD();
	}
	TPDTM_DMESG("Forbid NoiseMitigation oK\n");\
	ret = synaptics_rmi4_i2c_write_byte(tc_g->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
	ret = synaptics_rmi4_i2c_write_word(tc_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
	ret = synaptics_rmi4_i2c_write_byte(tc_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;

	for(x = 0;x < 2; x++)
	{
		TPDTM_DMESG("\n[%2d]",x);
		for(y = 0; y < 5; y++)
		{
			ret = synaptics_rmi4_i2c_read_byte(tc_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(tc_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data[x][y] = (tmp_h<<8)|tmp_l;

			TPDTM_DMESG("%d,",baseline_data[x][y]);
			if( x==0 && y==3/* && baseline_data[x][y] < 1500 && baseline_data[x][y] > 500*/)
				num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "left:%d[0000,9999]\n", baseline_data[x][y]);
			if( x==0 && y==4/* && baseline_data[x][y] < 1500 && baseline_data[x][y] > 500*/)
				num_read_chars += sprintf(&(kernel_buf[num_read_chars]), "right:%d[0000,9999]\n", baseline_data[x][y]);
		}
	}

	TPDTM_DMESG("\n report type2 delta image \n");

	ret = synaptics_rmi4_i2c_write_byte(tc_g->client,F54_ANALOG_COMMAND_BASE,0X02);
	delay_qt_ms(60);
	ret = synaptics_soft_reset(tc_g);
	delay_qt_ms(100);
	synaptics_enable_interrupt(tc_g,1);
	mutex_unlock(&tc_g->mutex);

	TPDTM_DMESG("\nreport delta image end\n");
	TPD_ERR("num_read_chars = %zd , size = %zd\n", num_read_chars, size);
	num_read_chars += sprintf( &( kernel_buf[num_read_chars] ), "%s" , "\r\n" );
	ret = simple_read_from_buffer(buf, size, ppos, kernel_buf, strlen(kernel_buf));
	kfree(kernel_buf);
	delay_qt_ms(10);
	if(tc_g->using_polling)
		schedule_delayed_work(&tc_g->dwork, msecs_to_jiffies(polling_interval));
	else
		enable_irq(tc_g->irq);
	return ret;
}
static const struct file_operations tp_baseline_image_proc_fops =
{
	.read = tp_baseline_show,
	.owner = THIS_MODULE,
};

static ssize_t synaptics_s1302_virtual_key_enable_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
	int ret = 0;
	char buf[10]={0};

	if( t > 2)
		return t;
	if( copy_from_user(buf, page, t) ){
		TPD_ERR("%s: read proc input error.\n", __func__);
		return t;
	}

	sscanf(buf, "%d", &ret);
	TPD_ERR("%s 0virtual key :%d\n",__func__,ret);
	ret &= 0x1;
	if( ret == 1 ){
		virtual_key_enable = true;
	}else{
		virtual_key_enable = false;
	}
    return t;
}
static int synaptics_s1302_virtual_key_enable_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 virtual key %s\n",virtual_key_enable?("enabled"):("disabled"));
    return 0 ;
}
static int synaptics_s1302_virtual_key_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_s1302_virtual_key_enable_show, inode->i_private);
}
const struct file_operations proc_virtual_key =
{
	.owner		= THIS_MODULE,
	.open		= synaptics_s1302_virtual_key_enable_open,
	.read		= seq_read,
	.write      = synaptics_s1302_virtual_key_enable_write,
	.llseek 	= seq_lseek,
	.release	= single_release,
};

static int key_disable_read_func(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 key_disable %s\n",key_disable?("disable"):("enable"));
    return 0 ;
}

static int key_disable_open_func(struct inode *inode, struct file *file)
{
	return single_open(file, key_disable_read_func, inode->i_private);
}

static ssize_t key_disable_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[10]={0};

	if( count > sizeof(buf)){
		TPD_ERR("%s error\n",__func__);
		return count;
	}

	if(copy_from_user(buf, buffer, count))
	{
		TPD_ERR("%s copy error\n", __func__);
		return count;
	}
	if (NULL != strstr(buf,"disable"))
	{
		key_disable =true;
	}
	else if (NULL != strstr(buf,"enable"))
	{
		key_disable =false;
	}
	TPD_ERR("%s key_disable:%d \n",__func__,key_disable);
	return count;
}

const struct file_operations key_disable_proc_fops =
{
	.owner		= THIS_MODULE,
	.open		= key_disable_open_func,
	.read		= seq_read,
	.write          = key_disable_write_func,
	.llseek 	= seq_lseek,
	.release	= single_release,
};

static int synaptics_s1302_proc(void)
{
    struct proc_dir_entry *proc_entry=0;

    struct proc_dir_entry *procdir = proc_mkdir( "s1302", NULL );
    //for firmware version
    proc_entry = proc_create_data("fw_update", 0666, procdir,&proc_firmware_update,NULL);
    proc_entry = proc_create_data("key_rep", 0666, procdir,&proc_reverse_key,NULL);
    proc_entry = proc_create_data("radd", 0666, procdir,&proc_radd,NULL);
    proc_entry = proc_create_data("reset", 0666, procdir,&proc_reset,NULL);
    proc_entry = proc_create_data("debug", 0666, procdir,&proc_debug,NULL);
    proc_entry = proc_create_data("strength", 0444, procdir,&proc_key_strength,NULL);
    proc_entry = proc_create_data("virtual_key", 0666, procdir,&proc_virtual_key,NULL);
    proc_entry = proc_create_data("touchkey_baseline_test", 0644, procdir, &tp_baseline_image_proc_fops,NULL);
    proc_entry = proc_create_data("setting_wait_time_test", 0644, procdir, &setting_wait_time_proc_fops,NULL);
    proc_entry = proc_create_data("key_disable", 0666, procdir, &key_disable_proc_fops,NULL);
    TPD_ERR("create nodes is successe!\n");

    return 0;
}
/******************************end****************************/

/****************************S3203*****update**********************************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

static void re_scan_PDT(struct i2c_client *client)
{
	uint8_t buf[8];
	i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	SynaF34DataBase = buf[3];
	SynaF34QueryBase = buf[0];
	i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	SynaF01DataBase = buf[3];
	SynaF01CommandBase = buf[1];
	i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);

	SynaF34Reflash_BlockNum = SynaF34DataBase;
	SynaF34Reflash_BlockData = SynaF34DataBase + 2;
	SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 2;
	SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 3;
	SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +5;
	SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 7;
	SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 9;
	i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
	SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	TPD_DEBUG("SynaFirmwareBlockSize s1302 is %d\n", SynaFirmwareBlockSize);
	SynaF34_FlashControl = SynaF34DataBase + 0x12;
}
struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
		(unsigned int)ptr[1] * 0x100 +
		(unsigned int)ptr[2] * 0x10000 +
		(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_DEBUG(" debug checksume is 0x%x\n", header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" debug firmware_size is 0x%x\n", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" debug header->config_size is 0x%x\n", header->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_DEBUG(" debug header->contains_firmware_id is %x\n", header->contains_firmware_id);
	if( header->contains_firmware_id )
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

static int checkFlashState(struct i2c_client *client)
{
	int ret ;
	int count = 0;
	ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl);
	while ( (ret != 0x80)&&(count < 8) ) {
		msleep(3); //wait 3ms
		ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl);
		count++;
	}
	if(count == 8)
		return 1;
	else
		return 0;
}

static int synaptics_fw_check(struct synaptics_ts_data *ts )
{
	int ret;
	uint8_t buf[5];
	uint32_t bootloader_mode;

	if(!ts){
		TPD_ERR("%s ts is NULL\n",__func__);
		return -1;
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}

	/*read product id */
	ret = synaptics_read_product_id(ts,&buf[0]);
	if(ret) {
		TPD_ERR("failed to read product info \n");
		return -1;
	}

	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40;
	TPD_DEBUG("afte fw update,program memory self-check bootloader_mode = 0x%x\n",bootloader_mode);

	if(bootloader_mode == 0x40) {
		TPD_ERR("Something terrible wrong \n Trying Update the Firmware again\n");
		return -1;
	}
	return 0;
}

static int synapitcs_ts_update(struct i2c_client *client, const uint8_t *data, uint32_t data_len ,bool force)
{
	int ret,j;
	uint8_t buf[8];
	uint8_t bootloder_id[10];
	uint16_t block,firmware,configuration;
	uint32_t CURRENT_FIRMWARE_ID = 0 , FIRMWARE_ID = 0;
	const uint8_t *Config_Data = NULL;
	const uint8_t *Firmware_Data = NULL;
	struct image_header_data header;
	struct synaptics_ts_data *ts = dev_get_drvdata(&client->dev);

	TPD_DEBUG("%s is called\n",__func__);
	if(!client)
		return -1;

	parse_header(&header,data);
	if((header.firmware_size + header.config_size + 0x100) > data_len) {
		TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
		return -1;
	}

	Firmware_Data = data + 0x100;
	Config_Data = Firmware_Data + header.firmware_size;
	ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);

	ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL_BASE, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
	TPD_ERR("use firmware version[%x], image firmware version[%x]\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID);
	//TPD_ERR("synaptics force is %d\n", force);
	if(!force) {
		if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
			return 0;
		}
	}
	re_scan_PDT(client);
	block = 16;
	TPD_DEBUG("block is %d \n",block);
	firmware = (header.firmware_size)/16;
	TPD_DEBUG("firmware is %d \n",firmware);
	configuration = (header.config_size)/16;
	TPD_DEBUG("configuration is %d \n",configuration);


	ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));
	TPD_DEBUG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	TPDTM_DMESG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);

	synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x0F);
	msleep(10);
	TPD_DEBUG("attn step 4 SynaF34_FlashControl:0x%x\n",SynaF34_FlashControl);
	ret=checkFlashState(client);
	if(ret > 0) {
		TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
		return -1;
	}
	ret = i2c_smbus_read_byte_data(client,0x04);
	TPD_DEBUG("The status(device state) is %x\n",ret);
	ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
	TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
	ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret|0x04);
	/********************get into prog end************/
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	TPD_DEBUG("ret is %d\n",ret);
	re_scan_PDT(client);
	i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
	i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
	i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
	msleep(1500);
	ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
	TPDTM_DMESG("going to flash firmware area synaF34_FlashControl %d\n",ret);

	TPD_ERR("update-----------------firmware ------------------update!\n");
	TPD_DEBUG("cnt %d\n",firmware);
	for(j=0; j<firmware; j++) {
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Firmware_Data[j*16]);
		synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x02);
		ret=checkFlashState(client);
		if(ret > 0) {
			TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}
	}
	//step 7 configure data
	//TPD_ERR("going to flash configuration area\n");
	//TPD_ERR("header.firmware_size is 0x%x\n", header.firmware_size);
	//TPD_ERR("bootloader_size is 0x%x\n", bootloader_size);
	TPD_ERR("update-----------------configuration ------------------update!\n");
	for(j=0;j<configuration;j++) {
		//a)write SynaF34Reflash_BlockNum to access
		buf[0]=j&0x00ff;
		buf[1]=(j&0xff00)>>8;
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockNum,2,buf);
		//b) write data
		synaptics_rmi4_i2c_write_block(client,SynaF34Reflash_BlockData,16,&Config_Data[j*16]);
		//c) issue write
		synaptics_rmi4_i2c_write_byte(client,SynaF34_FlashControl,0x06);
		//d) wait attn
		ret = checkFlashState(client);
		if(ret > 0) {
			TPD_ERR("Configuration:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
			return -1;
		}
	}

	//step 1 issue reset
	synaptics_rmi4_i2c_write_byte(client,SynaF01CommandBase,0x01);
	//step2 wait ATTN
	//delay_qt_ms(1000);
	mdelay(1500);
	synaptics_read_register_map(ts);
	//FW flash check!
	ret =synaptics_fw_check(ts);
	if(ret < 0 ) {
		TPD_ERR("Firmware self check failed\n");
		return -1;
	}
	TPD_ERR("Firmware self check Ok\n");
	return 0;
}
static int synaptics_soft_reset(struct synaptics_ts_data *ts)
{
    int ret;

    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
    if (ret < 0){
    TPD_ERR("reset error ret=%d\n",ret);
    }
    TPD_ERR("%s !!!\n",__func__);
    return ret;
}
static void synaptics_hard_reset(struct synaptics_ts_data *ts)
{
    return; //WAR 1302 hw error[RST short issue]
    if(ts->reset_gpio > 0)
    {
        gpio_set_value(ts->reset_gpio,0);
        msleep(5);
        gpio_set_value(ts->reset_gpio,1);
        msleep(100);
        TPD_ERR("%s !!!\n",__func__);
    }

}
static int synaptics_parse_dts(struct device *dev, struct synaptics_ts_data *ts)
{
	int rc;
	int retval;
	struct device_node *np;

	np = dev->of_node;
	ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, &(ts->irq_flags));
	if( ts->irq_gpio < 0 ){
		TPD_DEBUG("ts->irq_gpio not specified\n");
	}

	ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
	if( ts->reset_gpio < 0 ){
		TPD_DEBUG("ts->reset-gpio  not specified\n");
	}
	ts->en3v_gpio = of_get_named_gpio(np, "synaptics,en3v_gpio", 0);
	if( ts->en3v_gpio < 0 ){
		TPD_DEBUG("ts->en3v_gpio not specified\n");
	}

	/***********power regulator_get****************/
	
	ts->vdd_2v8 = regulator_get(&ts->client->dev, "vdd_2v8");
	if( IS_ERR(ts->vdd_2v8) ){
		rc = PTR_ERR(ts->vdd_2v8);
		TPD_DEBUG("Regulator get failed vdd rc=%d\n", rc);
	}

	ts->vcc_i2c_1v8 = regulator_get(&ts->client->dev, "vcc_i2c_1v8");
	if( IS_ERR(ts->vcc_i2c_1v8) ){
		rc = PTR_ERR(ts->vcc_i2c_1v8);
		TPD_DEBUG("Regulator get failed vcc_i2c rc=%d\n", rc);
	}

	if( ts->reset_gpio > 0){
		if( gpio_is_valid(ts->reset_gpio) ){
			rc = gpio_request(ts->reset_gpio, "s1302_reset");
			if(rc){
				TPD_ERR("unable to request gpio [%d]\n", ts->reset_gpio);
			}
		}
	}

	if( ts->en3v_gpio > 0){
		if( gpio_is_valid(ts->en3v_gpio) ){
			rc = gpio_request(ts->en3v_gpio, "s1302_en3v");
			if(rc)
				TPD_ERR("unable to request gpio [%d]\n", ts->en3v_gpio);
            retval = gpio_direction_output(ts->en3v_gpio,1);
		}
	}
#ifdef SUPPORT_FOR_COVER_ESD
	if(of_property_read_bool(np, "oem,cover_reject"))
		ts->cover_reject=true;
	else
		ts->cover_reject=false;
	TPD_ERR("%s cover_reject [%s]\n",__func__,ts->cover_reject?"true":"false");
#endif
	return rc;
}
static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->pinctrl = devm_pinctrl_get((ts->dev));
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		retval = PTR_ERR(ts->pinctrl);
        printk("%s %d error!\n",__func__,__LINE__);
		goto err_pinctrl_get;
	}

	ts->pinctrl_state_active
		= pinctrl_lookup_state(ts->pinctrl, "pmx_tk_active");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
		retval = PTR_ERR(ts->pinctrl_state_active);
        printk("%s %d error!\n",__func__,__LINE__);
		goto err_pinctrl_lookup;
	}
    
	ts->pinctrl_state_suspend
		= pinctrl_lookup_state(ts->pinctrl, "pmx_tk_suspend");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
		retval = PTR_ERR(ts->pinctrl_state_suspend);
        printk("%s %d !\n",__func__,__LINE__);
		goto err_pinctrl_lookup;
	}
	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ts->pinctrl);
err_pinctrl_get:
	ts->pinctrl = NULL;
	return retval;
}

static int choice_gpio_function(struct synaptics_ts_data *ts)
{
	int ret=0;
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client, 0x37, 0x81);
	if(ret < 0){
		ret = i2c_smbus_write_byte_data(ts->client, 0x37, 0x81);
	}
	msleep(5);
	synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00 );
	synaptics_rmi4_i2c_read_word(ts->client, 0x13);
	msleep(5);
	if (gpio_is_valid(ts->irq_gpio)) {
		/* configure touchscreen irq gpio */
		ret = gpio_request(ts->irq_gpio,"s1302_int");
		if (ret) {
			TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
		}
		ret = gpio_direction_input(ts->irq_gpio);
		msleep(50);
		ts->irq = gpio_to_irq(ts->irq_gpio);
	}
	TPD_ERR("synaptic:ts->irq is %d\n",ts->irq);
	if(!gpio_get_value(ts->irq_gpio)){
                msleep(2);
		if(!gpio_get_value(ts->irq_gpio))
			ts->using_polling = 1;
		else
			ts->using_polling = 0;
	}else{
		ts->using_polling = 0;
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0x37, 0x80);
	if(ret < 0){
		ret = i2c_smbus_write_byte_data(ts->client, 0x37, 0x80);
	}
	return ret;
}

static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef CONFIG_SYNAPTIC_RED
	struct remotepanel_data *premote_data = NULL;
#endif
	struct synaptics_ts_data *ts = NULL;
	int ret = -1;
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0;
	uint32_t bootloader_mode;

	TPD_ERR("%s  is called\n",__func__);

	ts = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if( ts == NULL ) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->dev = &client->dev;
	ts->loading_fw = false;
    tc_g = ts;
#if (defined SUPPORT_VIRTUAL_KEY)
    init_completion(&key_cm);
#endif
    ret = synaptics_dsx_pinctrl_init(ts);
    if (!ret && ts->pinctrl) {
        ret = pinctrl_select_state(ts->pinctrl,
                ts->pinctrl_state_active);
        }

	synaptics_parse_dts(&client->dev, ts);
    /***power_init*****/
	ret = tc_power(ts, 1);
	if( ret < 0 )
		TPD_ERR("regulator_enable is called\n");

	mutex_init(&ts->mutex);
	ts->is_suspended = 0;

	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ){
		TPD_ERR("%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	ret = synaptics_read_product_id(ts,ts->manu_name);
	if( ret < 0 ) {
		test_err = 1;
		synaptics_hard_reset(ts);
		ret = synaptics_read_product_id(ts,ts->manu_name);
		if( ret < 0 ) {
			TPD_ERR("synaptics is no exist!\n");
			goto err_check_functionality_failed;
		}
	}

	synaptics_read_register_map(ts);
	synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL_BASE, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
	TPD_ERR("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);
    sprintf(ts->fw_id,"0x%x",CURRENT_FIRMWARE_ID);

	memset(ts->fw_name,TP_FW_NAME_MAX_LEN,0);
	strcpy(ts->fw_name,"tp/fw_synaptics_touchkey.img");
	TPD_DEBUG("synatpitcs_fw: fw_name = %s \n",ts->fw_name);

	push_component_info(TOUCH_KEY, ts->fw_id, ts->manu_name);

	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0x40;
	TPD_ERR("before fw update,bootloader_mode = 0x%x\n", bootloader_mode);
    if(0x40 == bootloader_mode){
		force_update = 1;
		TPD_ERR("This FW need to be updated!\n");
	} else {
		force_update = 0;
	}

	ret = synaptics_input_init(ts);
	if(ret < 0) {
		TPD_ERR("synaptics_input_init failed!\n");
	}
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret < 0)
		TPD_ERR("%s enable interrupt error ret=%d\n",__func__,ret);
#if defined(CONFIG_FB)
	ts->suspended = 0;
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret)
		TPD_ERR("Unable to register fb_notifier: %d\n", ret);
#endif
	ret = choice_gpio_function(ts);
	if(ret < 0){
		TPD_ERR("choice_gpio_function faile\n");
	}

	TPD_ERR("touchkey using poll status:%d \n",ts->using_polling);
	if(ts->using_polling){
			INIT_DELAYED_WORK(&ts->dwork, synaptics_work_thread_fn);
			schedule_delayed_work(&ts->dwork, msecs_to_jiffies(polling_interval));
	}else{
			ret = request_threaded_irq(ts->irq, NULL,
							synaptics_irq_thread_fn,
							ts->irq_flags | IRQF_ONESHOT,
							"synaptics-s1302", ts);
			if(ret < 0)
				TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
	}

	ret = synaptics_soft_reset(ts);
	if (ret < 0){
		TPD_ERR("%s faile to reset device\n",__func__);
	}

#ifdef SUPPORT_FOR_COVER_ESD
    process_key_timer_init(ts);
#endif
    synaptics_s1302_proc();
#ifdef CONFIG_SYNAPTIC_RED
	premote_data = remote_alloc_panel_data_s1302();
	if(premote_data) {
		premote_data->client 		= client;
		premote_data->input_dev		= ts->input_dev;
		premote_data->pmutex		= &ts->mutex;
		premote_data->irq_gpio 		= ts->irq_gpio;
		premote_data->irq			= client->irq;
		premote_data->enable_remote = &(ts->enable_remote);
		register_remote_device_s1302(premote_data);
    }
#endif
	TPDTM_DMESG("synaptics_ts_probe s1302: normal end\n");
	return 0;

err_check_functionality_failed:
err_alloc_data_failed:
	kfree(ts);
	ts = NULL;
	tc_g = NULL;
	printk("touchkey,s1302 probe: not normal end\n");
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	printk("touchkey,s1302 %s is called\n",__func__);
#ifdef CONFIG_SYNAPTIC_RED
	unregister_remote_device_s1302();
#endif

#if defined(CONFIG_FB)
	if( fb_unregister_client(&ts->fb_notif) )
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPD_DEBUG("%s: is called\n", __func__);

	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		return -1;
	}
	if(ts->loading_fw) {
		TPD_ERR("FW is updating while suspending");
		return -1;
	}
	ts->is_suspended = 1;
	if(ts->using_polling)
		cancel_delayed_work_sync(&ts->dwork);
	else
		disable_irq_nosync(ts->irq);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0) {
		TPD_ERR("%s: line[%d]Failed to change page!!\n",__func__,__LINE__);
		return -1;
	}
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL_BASE, 0x81);
	if( ret < 0 ){
		TPD_ERR("%s to sleep failed\n", __func__);
		return -1;
	}
	TPD_DEBUG("%s:normal end\n", __func__);
	return 0;
}

static int synaptics_ts_resume(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	TPD_DEBUG("%s is called\n", __func__);
	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		goto ERR_RESUME;
	}

	ts->is_suspended = 0;
	mutex_lock(&ts->mutex);

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPD_ERR("%s: failed for page select try again later\n", __func__);
		msleep(20);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
		if( ret < 0 ){
			TPD_ERR("%s: failed for page select try again later\n", __func__);
		}
	}

	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL_BASE, 0x80);
	if( ret < 0 ){
		TPD_ERR("%s:to wakeup failed\n", __func__);
		goto ERR_RESUME;
	}
	if(ts->using_polling)
		schedule_delayed_work(&ts->dwork, msecs_to_jiffies(polling_interval));
	else
		enable_irq(ts->irq);
	TPD_DEBUG("%s:normal end!\n", __func__);
ERR_RESUME:
	mutex_unlock(&ts->mutex);
	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct synaptics_ts_data *ts = container_of(self, struct synaptics_ts_data, fb_notif);

	if(FB_EVENT_BLANK != event)
	return 0;
	if((evdata) && (evdata->data) && (ts) && (ts->client)&&(event == FB_EVENT_BLANK)) {
		blank = evdata->data;
		if( *blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_NORMAL) {
			TPD_DEBUG("%s going TP resume\n", __func__);
			if(ts->suspended == 1){
				ts->suspended = 0;
				synaptics_ts_resume(&ts->client->dev);
			}
		} else if( *blank == FB_BLANK_POWERDOWN) {
			TPD_DEBUG("%s : going TP suspend\n", __func__);
			if(ts->suspended == 0) {
				ts->suspended = 1;
				synaptics_ts_suspend(&ts->client->dev);
			}
		}
	}
	return 0;
}
#endif

static int __init tc_driver_init(void)
{
	if( i2c_add_driver(&tc_i2c_driver)!= 0 ){
		TPD_ERR("unable to add i2c driver.\n");
		return -1;
	}
	return 0;
}

/* should never be called */
static void __exit tc_driver_exit(void)
{
	i2c_del_driver(&tc_i2c_driver);
	return;
}

module_init(tc_driver_init);
module_exit(tc_driver_exit);

MODULE_DESCRIPTION("Synaptics S1302 Touchscreen Driver");
MODULE_LICENSE("GPL");
