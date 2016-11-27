/************************************************************************************
 ** File: - /android/kernel/drivers/input/touchscreen/synaptic_s3320.c
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

#include "synaptics_redremote.h"
#include <linux/project_info.h>
#include "synaptics_baseline.h"

/*------------------------------------------------Global Define--------------------------------------------*/

#define TP_UNKNOWN 0
#define TP_G2Y 1
#define TP_TPK 2
#define TP_TRULY 3
#define TP_OFILM 4
#define TP_JDI_TPK 6
#define TP_TEST_ENABLE 1

#define DiagonalUpperLimit  1100
#define DiagonalLowerLimit  900

#define PAGESIZE 512
#define TPD_USE_EINT

#define TPD_DEVICE "synaptics,s3320"

//#define SUPPORT_SLEEP_POWEROFF
#define SUPPORT_GESTURE
#define RESET_ONESECOND
//#define SUPPORT_GLOVES_MODE
//#define REPORT_2D_PRESSURE
#define SUPPORT_VIRTUAL_KEY


#define SUPPORT_TP_SLEEP_MODE
#define TYPE_B_PROTOCOL      //Multi-finger operation
#define TP_FW_NAME_MAX_LEN 128

#define TEST_MAGIC1 0x494D494C
#define TEST_MAGIC2 0x474D4954

struct test_header {
	unsigned int magic1;
	unsigned int magic2;
	unsigned int withCBC;
	unsigned int array_limit_offset;
	unsigned int array_limit_size;
	unsigned int array_limitcbc_offset;
	unsigned int array_limitcbc_size;
};

/******************for Red function*****************/
#define CONFIG_SYNAPTIC_RED

/*********************for gesture*******************/
#ifdef SUPPORT_GESTURE
#define ENABLE_UNICODE  0x40
#define ENABLE_VEE      0x20
#define ENABLE_CIRCLE   0x08
#define ENABLE_SWIPE    0x02
#define ENABLE_DTAP     0x01

#define UNICODE_DETECT  0x0b
#define VEE_DETECT      0x0a
#define CIRCLE_DETECT   0x08
#define SWIPE_DETECT    0x07
#define DTAP_DETECT     0x03


#define UnkownGestrue       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W

#define BIT0 (0x1 << 0)
#define BIT1 (0x1 << 1)
#define BIT2 (0x1 << 2)
#define BIT3 (0x1 << 3)
#define BIT4 (0x1 << 4)
#define BIT5 (0x1 << 5)
#define BIT6 (0x1 << 6)
#define BIT7 (0x1 << 7)

int LeftVee_gesture = 0; //">"
int RightVee_gesture = 0; //"<"
int DouSwip_gesture = 0; // "||"
int Circle_gesture = 0; // "O"
int UpVee_gesture = 0; //"V"
int DownVee_gesture = 0; //"^"
int DouTap_gesture = 0; //"double tap"

int Left2RightSwip_gesture=0;//"(-->)"
int Right2LeftSwip_gesture=0;//"(<--)"
int Up2DownSwip_gesture =0;//"up to down |"
int Down2UpSwip_gesture =0;//"down to up |"

int Wgestrue_gesture =0;//"(W)"
int Mgestrue_gesture =0;//"(M)"

#endif

/*********************for Debug LOG switch*******************/
#define TPD_ERR(a, arg...)  pr_err(TPD_DEVICE ": " a, ##arg)
#define TPDTM_DMESG(a, arg...)  printk(TPD_DEVICE ": " a, ##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
		pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)

/*---------------------------------------------Global Variable----------------------------------------------*/
static int baseline_ret = 0;
static int TP_FW;
static int tp_dev = 6;
static unsigned int tp_debug = 1;
static int button_map[3];
static int tx_rx_num[2];
static int16_t Rxdata[30][30];
static int16_t delta_baseline[16][28];
static int16_t baseline[8][16];
static int16_t delta[8][16];
static int TX_NUM;
static int RX_NUM;
static int report_key_point_y = 0;
static int force_update = 0;
static int LCD_WIDTH ;
static int LCD_HEIGHT ;
static int get_tp_base = 0;
//static int ch_getbase_status = 0;
//struct timeval start_time,end_time;

#ifdef SUPPORT_TP_SLEEP_MODE
static int sleep_enable;
#endif

static struct synaptics_ts_data *ts_g = NULL;
static struct workqueue_struct *synaptics_wq = NULL;
static struct workqueue_struct *synaptics_report = NULL;
static struct workqueue_struct *get_base_report = NULL;
static struct proc_dir_entry *prEntry_tp = NULL;


#ifdef SUPPORT_GESTURE
static uint32_t clockwise;
static uint32_t gesture;

static uint32_t gesture_upload;

/****point position*****/
struct Coordinate {
	uint32_t x;
	uint32_t y;
};
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
#endif

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

static int F12_2D_QUERY_BASE;
static int F12_2D_CMD_BASE;
static int F12_2D_CTRL_BASE;
static int F12_2D_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_DATA01;
static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;

static int F12_2D_CTRL08;
static int F12_2D_CTRL32;
static int F12_2D_DATA04;
static int F12_2D_DATA38;
static int F12_2D_DATA39;
static int F12_2D_CMD00;
static int F12_2D_CTRL20;
static int F12_2D_CTRL27;

static int F34_FLASH_CTRL00;

static int F51_CUSTOM_CTRL00;
static int F51_CUSTOM_DATA04;
static int F51_CUSTOM_DATA11;

#if TP_TEST_ENABLE
static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00
#endif

/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_i2c_suspend(struct device *dev);
static int synaptics_i2c_resume(struct device *dev);
/**************I2C resume && suspend end*********/
static void speedup_synaptics_resume(struct work_struct *work);
static int synaptics_ts_resume(struct device *dev);
static int synaptics_ts_suspend(struct device *dev);
static int synaptics_ts_remove(struct i2c_client *client);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static ssize_t synaptics_rmi4_baseline_show(struct device *dev, char *buf, bool savefile)	;
static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev, struct device_attribute *attr, char *buf);
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

static int synaptics_rmi4_i2c_write_word(struct i2c_client* client,
		unsigned char addr,unsigned short data);
static int synaptics_mode_change(int mode);
#ifdef TPD_USE_EINT
static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id);
#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#endif
static int synaptics_soft_reset(struct synaptics_ts_data *ts);
static void synaptics_hard_reset(struct synaptics_ts_data *ts);
static int set_changer_bit(struct synaptics_ts_data *ts);
static int tp_baseline_get(struct synaptics_ts_data *ts,bool flag);

/*-------------------------------Using Struct----------------------------------*/
struct point_info {
	unsigned char status;
	int x;
	int raw_x;
	int y;
	int raw_y;
	int z;
#ifdef REPORT_2D_PRESSURE
    unsigned char pressure;
#endif
};

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
	.suspend = synaptics_i2c_suspend,
	.resume = synaptics_i2c_resume,
#else
	.suspend = NULL,
	.resume = NULL,
#endif
};

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
	TPD_ERR("after on cpu [%d]\n",smp_processor_id());
	probe_ret = synaptics_ts_probe(client_optimize,dev_id);
}

static int oem_synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i;
	optimize_data.client = client;
	optimize_data.dev_id = id;
	optimize_data.workqueue = create_workqueue("tpd_probe_optimize");
	INIT_DELAYED_WORK(&(optimize_data.work), synaptics_ts_probe_func);
	TPD_ERR("before on cpu [%d]\n",smp_processor_id());

	//add by lifeng@bsp 2015-12-10 for only one cpu on line
	for (i = 0; i < NR_CPUS; i++){
         TPD_ERR("check CPU[%d] is [%s]\n",i,cpu_is_offline(i)?"offline":"online");
		 if (cpu_online(i) && (i != smp_processor_id()))
            break;
    }
    queue_delayed_work_on(i != NR_CPUS?i:0,optimize_data.workqueue,&(optimize_data.work),msecs_to_jiffies(300));
    //end add by lifeng@bsp 2015-12-10 for only one cpu on line

	return probe_ret;
}

static struct i2c_driver tpd_i2c_driver = {
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

struct synaptics_ts_data {
	struct i2c_client *client;
	struct mutex mutex;
	struct mutex mutexreport;
	int irq;
	int irq_gpio;
	atomic_t irq_enable;
	int id1_gpio;
	int id2_gpio;
	int id3_gpio;
	int reset_gpio;
	int v1p8_gpio;
	int support_hw_poweroff;
	int enable2v8_gpio;
	int max_num;
	int enable_remote;
	uint32_t irq_flags;
	uint32_t max_x;
	uint32_t max_y;
	uint32_t max_y_real;
	uint32_t btn_state;
	uint32_t pre_finger_state;
	uint32_t pre_btn_state;
	struct delayed_work  base_work;
	struct work_struct  report_work;
	struct delayed_work speed_up_work;
	struct input_dev *input_dev;
	struct hrtimer timer;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	/******gesture*******/
	int gesture_enable;
	int in_gesture_mode;
	int glove_enable;
    int changer_connet;
	int is_suspended;
    atomic_t is_stop;
    spinlock_t lock;

	/********test*******/
	int i2c_device_test;

	/******power*******/
	struct regulator *vdd_2v8;
	struct regulator *vcc_i2c_1v8;

	/*pinctrl******/
	struct device						*dev;
	struct pinctrl 						*pinctrl;
	struct pinctrl_state 				*pinctrl_state_active;
	struct pinctrl_state 				*pinctrl_state_suspend;

	/*******for FW update*******/
	bool loading_fw;
    bool support_ft;//support force touch
	char fw_name[TP_FW_NAME_MAX_LEN];
	char test_limit_name[TP_FW_NAME_MAX_LEN];
	char fw_id[12];
	char manu_name[30];
#ifdef SUPPORT_VIRTUAL_KEY
        struct kobject *properties_kobj;
#endif
};

static struct device_attribute attrs_oem[] = {
	//	__ATTR(baseline_test, 0664, synaptics_rmi4_baseline_show, NULL),
	__ATTR(vendor_id, 0664, synaptics_rmi4_vendor_id_show, NULL),
};

static void touch_enable (struct synaptics_ts_data *ts)
{
    spin_lock(&ts->lock);
    if(0 == atomic_read(&ts->irq_enable))
    {
        if(ts->irq)
            enable_irq(ts->irq);
        atomic_set(&ts->irq_enable,1);
        //TPD_ERR("test %%%% enable irq\n");
    }
    spin_unlock(&ts->lock);
}

static void touch_disable(struct synaptics_ts_data *ts)
{
    spin_lock(&ts->lock);
    if(1 == atomic_read(&ts->irq_enable))
    {
        if(ts->irq)
            disable_irq_nosync(ts->irq);
        atomic_set(&ts->irq_enable,0);
        //TPD_ERR("test ****************** disable irq\n");
    }
    spin_unlock(&ts->lock);
}

static int tpd_hw_pwron(struct synaptics_ts_data *ts)
{
	int rc;

	/***enable the 2v8 power*****/
	if (!IS_ERR(ts->vdd_2v8)) {
		//regulator_set_optimum_mode(ts->vdd_2v8,100000);
		rc = regulator_enable(ts->vdd_2v8);
		if(rc){
			dev_err(&ts->client->dev,
					"Regulator vdd enable failed rc=%d\n", rc);
			//return rc;
		}
	}
	if( ts->v1p8_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the v1p8_gpio\n");
		gpio_direction_output(ts->v1p8_gpio, 1);
	}
	//msleep(100);

	if( ts->enable2v8_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 1);
	}

	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		//regulator_set_optimum_mode(ts->vcc_i2c_1v8,100000);
		rc = regulator_enable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			//return rc;
		}
	}
	if( ts->reset_gpio > 0 ) {
		gpio_direction_output(ts->reset_gpio, 1);
        //msleep(10);
        usleep_range(10*1000, 10*1000);
		gpio_direction_output(ts->reset_gpio, 0);
        //msleep(5);
        usleep_range(5*1000, 5*1000);
		gpio_direction_output(ts->reset_gpio, 1);
		TPD_DEBUG("synaptics:enable the reset_gpio\n");
	}
	return rc;
}

static int tpd_hw_pwroff(struct synaptics_ts_data *ts)
{
	int rc = 0;
	if( ts->reset_gpio > 0 ) {
		TPD_DEBUG("%s set reset gpio low\n",__func__);
		gpio_direction_output(ts->reset_gpio, 0);
	}

	if (!IS_ERR(ts->vcc_i2c_1v8)) {
		rc = regulator_disable( ts->vcc_i2c_1v8 );
		if(rc) {
			dev_err(&ts->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
	}
	if( ts->v1p8_gpio > 0 ) {
		TPD_DEBUG("synaptics:disable the v1p8_gpio\n");
		gpio_direction_output(ts->v1p8_gpio, 0);
	}
	if (!IS_ERR(ts->vdd_2v8)) {
		rc = regulator_disable(ts->vdd_2v8);
		if (rc) {
			dev_err(&ts->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	}
	if( ts->enable2v8_gpio > 0 ) {
		TPD_DEBUG("synaptics:enable the enable2v8_gpio\n");
		gpio_direction_output(ts->enable2v8_gpio, 0);
	}
	return rc;
}

static int tpd_power(struct synaptics_ts_data *ts, unsigned int on)
{
	int ret;
	if(on)
		ret = tpd_hw_pwron(ts);
	else
		ret = tpd_hw_pwroff(ts);

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

	F12_2D_QUERY_BASE = buf[0];
	F12_2D_CMD_BASE = buf[1];
	F12_2D_CTRL_BASE = buf[2];
	F12_2D_DATA_BASE = buf[3];

	TPD_ERR("F12_2D_QUERY_BASE = %x \n \
			F12_2D_CMD_BASE  = %x \n\
			F12_2D_CTRL_BASE	= %x \n\
			F12_2D_DATA_BASE	= %x \n\
			",F12_2D_QUERY_BASE,F12_2D_CMD_BASE,F12_2D_CTRL_BASE,F12_2D_DATA_BASE);


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
	TPD_ERR("F34_FLASH_QUERY_BASE = %x \n\
			F34_FLASH_CMD_BASE	= %x \n\
			F34_FLASH_CTRL_BASE	= %x \n\
			F34_FLASH_DATA_BASE	= %x \n\
			", F34_FLASH_QUERY_BASE, F34_FLASH_CMD_BASE, F34_FLASH_CTRL_BASE, F34_FLASH_DATA_BASE);

	F01_RMI_QUERY11 = F01_RMI_QUERY_BASE+11;
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_RMI_DATA01 = F01_RMI_DATA_BASE + 1;

	F12_2D_CTRL08 = F12_2D_CTRL_BASE;
	F12_2D_CTRL32 = F12_2D_CTRL_BASE + 15;
	F12_2D_DATA38 = F12_2D_DATA_BASE + 54;
	F12_2D_DATA39 = F12_2D_DATA_BASE + 55;
	F12_2D_CMD00 = F12_2D_CMD_BASE;
	F12_2D_CTRL20 = F12_2D_CTRL_BASE + 0x07;
	F12_2D_CTRL27 = F12_2D_CTRL_BASE + 0x0c;


	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	if( ret < 0 ){
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2];
	F51_CUSTOM_DATA_BASE = buf[3];
	F51_CUSTOM_CTRL00 = F51_CUSTOM_CTRL_BASE;
	F51_CUSTOM_DATA04 = F51_CUSTOM_DATA_BASE;
	F51_CUSTOM_DATA11 = F51_CUSTOM_DATA_BASE;

	TPD_DEBUG("F51_CUSTOM_QUERY_BASE = %x \n\
			F51_CUSTOM_CMD_BASE  = %x \n\
			F51_CUSTOM_CTRL_BASE    = %x \n\
			F51_CUSTOM_DATA_BASE    = %x \n\
			", F51_CUSTOM_QUERY_BASE, F51_CUSTOM_CMD_BASE, F51_CUSTOM_CTRL_BASE, F51_CUSTOM_DATA_BASE);

#if TP_TEST_ENABLE
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x01);
	if(ret < 0) {
		TPD_ERR("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];
	TPD_ERR("F54_QUERY_BASE = %x \n\
			F54_CMD_BASE  = %x \n\
			F54_CTRL_BASE	= %x \n\
			F54_DATA_BASE	= %x \n\
			", F54_ANALOG_QUERY_BASE, F54_ANALOG_COMMAND_BASE , F54_ANALOG_CONTROL_BASE, F54_ANALOG_DATA_BASE);
#endif
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	return 0;
}

#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	unsigned char reportbuf[4];
	//chenggang.li@BSP.TP modified for gesture
	TPD_DEBUG("%s is called\n", __func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPD_ERR("%s: select page failed ret = %d\n", __func__, ret);
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ) {
		TPD_DEBUG("read reg F12_2D_CTRL20[0x%x] failed\n",F12_2D_CTRL20);
		return -1;
	}

	if( enable ) {
		ts->in_gesture_mode = 1;
		reportbuf[2] |= 0x02 ;
	} else {
		ts->in_gesture_mode = 0;
		reportbuf[2] &= 0xfd ;
	}
	TPD_DEBUG("F12_2D_CTRL20:0x%x=[2]:0x%x\n", F12_2D_CTRL20, reportbuf[2]);
	ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
	if( ret < 0 ){
		TPD_ERR("%s :Failed to write report buffer\n", __func__);
		return -1;
	}
	gesture = UnkownGestrue;
	return 0;
}
#endif

#ifdef SUPPORT_GLOVES_MODE
#define GLOVES_ADDR 0x001f //0x001D 0x001f
static int synaptics_glove_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	TPD_DEBUG("glove mode enable\n");
	/* page select = 0x4 */
	if( 1 == ts->glove_enable)  {
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data(ts->client, GLOVES_ADDR);
		//TPDTM_DMESG("enable glove  ret is %x ret|0x20 is %x\n", ret, ret|0x20);
		ret = i2c_smbus_write_byte_data(ts->client, GLOVES_ADDR, ret | 0x01);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	}else{
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
		ret = i2c_smbus_read_byte_data(ts->client, GLOVES_ADDR);
		ret = i2c_smbus_write_byte_data(ts->client, GLOVES_ADDR, ret & 0xFE);
		if( ret < 0 ){
			TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
			goto GLOVE_ENABLE_END;
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	if( ret < 0 ){
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		goto GLOVE_ENABLE_END;
	}

GLOVE_ENABLE_END:
	return ret;
}
#endif

#ifdef SUPPORT_TP_SLEEP_MODE
static int synaptics_sleep_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	if( 1 == sleep_enable ){
		/*0x00:enable glove mode,0x02:disable glove mode,*/
		TPDTM_DMESG("sleep mode enable\n");
        ret = synaptics_mode_change(0x01);
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}else{
		TPDTM_DMESG("sleep mode disable\n");
        ret = synaptics_mode_change(0x84);
		if( ret < 0 ){
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	if( ret < 0 ){
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}

SLEEP_ENABLE_END:
	return ret;
}
#endif

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;

	memset(buf1, 0 , sizeof(buf1));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY11, 8, &(buf1[0x0]));
	ret = synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));
	if( ret < 0 ){
		TPD_ERR("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;

	TPD_DEBUG("%s is called!\n",__func__);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if( ret < 0 ){
		TPD_ERR("init_panel failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/

    ret = synaptics_mode_change(0x80);//change tp to doze mode
	if( ret < 0 ){
		msleep(150);
        ret = synaptics_mode_change(0x80);
		if( ret < 0 ){
			TPD_ERR("%s failed for mode select\n",__func__);
		}
	}

	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{
	int ret;
	uint8_t abs_status_int;

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if( ret < 0 ) {
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}
	if( enable ) {
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE+1);
		if( ret < 0 ) {
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F01_RMI_CTRL00+1, abs_status_int);
	if( ret < 0 ) {
		TPDTM_DMESG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n", __func__, abs_status_int);
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_CTRL00+1);
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
/*
static void int_state(struct synaptics_ts_data *ts)
{
	int ret = -1;
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
	if(ret){
		TPD_ERR("%s:error cannot reset touch panel!\n",__func__);
		return;
	}
	//delay_qt_ms(170);
	delay_qt_ms(100);
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	ret = synaptics_init_panel(ts);
	if( ret < 0 ){
		TPD_DEBUG("%s:error cannot change mode!\n",__func__);
		return;
	}
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret){
		TPD_DEBUG("%s:error cannot enable interrupt!\n",__func__);
		return;
	}
}
*/
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

//chenggang.li@BSP.TP modified for oem 2014-08-05 gesture_judge
/***************start****************/
#ifdef SUPPORT_GESTURE
static void synaptics_get_coordinate_point(struct synaptics_ts_data *ts)
{
	int ret,i;
	uint8_t coordinate_buf[25] = {0};
	uint16_t trspoint = 0;
/* add by lifeng 2016/1/19 workarounds for the gestrue two interrupts begin*/
    static uint8_t coordinate_buf_last[25];
/* add by lifeng 2016/1/19 workarounds for the gestrue two interrupts end*/

	TPD_DEBUG("%s is called!\n",__func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x4);
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11, 8, &(coordinate_buf[0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 8, 8, &(coordinate_buf[8]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 16, 8, &(coordinate_buf[16]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 24, 1, &(coordinate_buf[24]));

/* add by lifeng 2016/1/19 workarounds for the gestrue two interrupts begin*/
    if(!strncmp(coordinate_buf_last,coordinate_buf,sizeof(coordinate_buf)))
    {
        TPD_ERR("%s reject the same gestrue[%d]\n",__func__,gesture);
        gesture = UnkownGestrue;
    }
    strncpy(coordinate_buf_last,coordinate_buf,sizeof(coordinate_buf));
/* add by lifeng 2016/1/19 workarounds for the gestrue two interrupts end*/

	for(i = 0; i< 23; i += 2) {
		trspoint = coordinate_buf[i]|coordinate_buf[i+1] << 8;
		TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n",i,trspoint);
	}

	TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n",coordinate_buf[24]);

	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8)) * LCD_WIDTH/ (ts->max_x);
	Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8)) * LCD_HEIGHT/ (ts->max_y);
	Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8)) * LCD_HEIGHT / (ts->max_y);
	Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8)) * LCD_HEIGHT / (ts->max_y);
	clockwise     = (coordinate_buf[24] & 0x10) ? 1 :
		(coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
}

static void gesture_judge(struct synaptics_ts_data *ts)
{
	unsigned int keyCode = KEY_F4;
	int ret = 0,gesture_sign, regswipe;
	uint8_t gesture_buffer[10];
	unsigned char reportbuf[3];
	F12_2D_DATA04 = 0x000A;

	TPD_DEBUG("%s start!\n",__func__);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00);
	if (ret < 0) {
		TPDTM_DMESG("failed to transfer the data, ret = %d\n", ret);
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	ret = i2c_smbus_read_i2c_block_data(ts->client,  F12_2D_DATA04, 5, &(gesture_buffer[0]));
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4);
	regswipe = i2c_smbus_read_byte_data(ts->client, F51_CUSTOM_DATA04+0x18);
	TPD_DEBUG("Gesture Type[0x%x]=[0x%x],lpwg Swipe ID[0x4%x] = [0x%x]\n",\
        F12_2D_DATA04,gesture_buffer[0],(F51_CUSTOM_DATA04+0x18),regswipe);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	gesture_sign = gesture_buffer[0];
	//detect the gesture mode
	switch (gesture_sign) {
		case DTAP_DETECT:
			    gesture = DouTap;
			break;
		case SWIPE_DETECT:
			gesture = (regswipe == 0x41) ? Left2RightSwip   :
				(regswipe == 0x42) ? Right2LeftSwip   :
				(regswipe == 0x44) ? Up2DownSwip      :
				(regswipe == 0x48) ? Down2UpSwip      :
				(regswipe == 0x84) ? DouSwip          :
				UnkownGestrue;
			break;
		case CIRCLE_DETECT:
			    gesture = Circle;
			break;
		case VEE_DETECT:
			gesture = (gesture_buffer[2] == 0x01) ? DownVee  :
				(gesture_buffer[2] == 0x02) ? UpVee    :
				(gesture_buffer[2] == 0x04) ? RightVee :
				(gesture_buffer[2] == 0x08) ? LeftVee  :
				UnkownGestrue;
			break;
		case UNICODE_DETECT:
			gesture = (gesture_buffer[2] == 0x77) ? Wgestrue :
				(gesture_buffer[2] == 0x6d) ? Mgestrue :
				UnkownGestrue;
	}


	TPD_ERR("detect %s gesture\n", gesture == DouTap ? "(double tap)" :
			gesture == UpVee ? "(V)" :
			gesture == DownVee ? "(^)" :
			gesture == LeftVee ? "(>)" :
			gesture == RightVee ? "(<)" :
			gesture == Circle ? "(O)" :
			gesture == DouSwip ? "(||)" :
			gesture == Left2RightSwip ? "(-->)" :
			gesture == Right2LeftSwip ? "(<--)" :
			gesture == Up2DownSwip ? "(up to down |)" :
			gesture == Down2UpSwip ? "(down to up |)" :
			gesture == Mgestrue ? "(M)" :
			gesture == Wgestrue ? "(W)" : "[unknown]");
	synaptics_get_coordinate_point(ts);

    TPD_DEBUG("gesture suport LeftVee:%d RightVee:%d DouSwip:%d Circle:%d UpVee:%d DouTap:%d\n",\
        LeftVee_gesture,RightVee_gesture,DouSwip_gesture,Circle_gesture,UpVee_gesture,DouTap_gesture);
	if((gesture == DouTap && DouTap_gesture)||(gesture == RightVee && RightVee_gesture)\
        ||(gesture == LeftVee && LeftVee_gesture)||(gesture == UpVee && UpVee_gesture)\
        ||(gesture == Circle && Circle_gesture)||(gesture == DouSwip && DouSwip_gesture)){
		gesture_upload = gesture;
		input_report_key(ts->input_dev, keyCode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keyCode, 0);
		input_sync(ts->input_dev);
	}else{

		ret = i2c_smbus_read_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) );
		ret = reportbuf[2] & 0x20;
		if(ret == 0)
			reportbuf[2] |= 0x02 ;
		ret = i2c_smbus_write_i2c_block_data( ts->client, F12_2D_CTRL20, 3, &(reportbuf[0x0]) ); //enable gesture
		if( ret < 0 ){
			TPD_ERR("%s :Failed to write report buffer\n", __func__);
			return;
		}
	}
    TPD_DEBUG("%s end!\n",__func__);
}
#endif
/***************end****************/
static char prlog_count = 0;
#ifdef REPORT_2D_PRESSURE
static unsigned char pres_value = 1;
#endif
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
extern struct completion key_cm;
extern bool key_back_pressed;
extern bool key_appselect_pressed;
extern bool key_home_pressed;
extern bool virtual_key_enable;
#endif
void int_touch(void)
{
	int ret = -1,i = 0;
	uint8_t buf[80];
	uint8_t finger_num = 0;
	uint8_t finger_status = 0;
	struct point_info points;
	uint32_t finger_info = 0;
	static uint8_t current_status = 0;
	uint8_t last_status = 0 ;
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
	bool key_appselect_check = false;
	bool key_back_check = false;
	bool key_home_check = false;
	bool key_pressed = key_appselect_pressed || key_back_pressed;// || key_home_pressed;
#endif
	struct synaptics_ts_data *ts = ts_g;

	memset(buf, 0, sizeof(buf));
	points.x = 0;
	points.y = 0;
	points.z = 0;
	points.status = 0;

	mutex_lock(&ts->mutexreport);
#ifdef REPORT_2D_PRESSURE
    if (ts->support_ft){
        ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4);
        ret = synaptics_rmi4_i2c_read_block(ts->client, 0x19,\
            sizeof(points.pressure), &points.pressure);
        if (ret < 0) {
            TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
            goto INT_TOUCH_END;
        }
        if (0 == points.pressure)//workaround for have no pressure value input reader into hover mode
        {
            pres_value++;
            if (255 == pres_value)
                pres_value = 1;
        }
        else
        {
            pres_value = points.pressure;
        }
    }
#endif
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	ret = synaptics_rmi4_i2c_read_block(ts->client, F12_2D_DATA_BASE, 80, buf);
	if (ret < 0) {
		TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
		goto INT_TOUCH_END;
	}
	for( i = 0; i < ts->max_num; i++ ) {
		points.status = buf[i*8];
		points.x = ((buf[i*8+2]&0x0f)<<8) | (buf[i*8+1] & 0xff);
		points.raw_x = buf[i*8+6] & 0x0f;
		points.y = ((buf[i*8+4]&0x0f)<<8) | (buf[i*8+3] & 0xff);
		points.raw_y = buf[i*8+7] & 0x0f;
		points.z = buf[i*8+5];
		finger_info <<= 1;
		finger_status =  points.status & 0x03;
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
            if(virtual_key_enable){
                if(points.y > 0x780 && key_pressed){
                        TPD_DEBUG("Drop TP event due to key pressed\n");
                        finger_status = 0;
                }else{
                    finger_status =  points.status & 0x03;
                }
            }else{
                finger_status =  points.status & 0x03;
            }
            if(virtual_key_enable){
                    if (!finger_status){
                        if (key_appselect_pressed && !key_appselect_check){
                            points.x = 0xb4;
                            points.y = 0x7e2;
                            points.z = 0x33;
                            points.raw_x = 4;
                            points.raw_y = 6;
                            key_appselect_check = true;
                            points.status = 1;
                            finger_status =  points.status & 0x03;
                        }else if (key_back_pressed && !key_back_check){
                            points.x = 0x384;
                            points.y = 0x7e2;
                            points.z = 0x33;
                            points.raw_x = 4;
                            points.raw_y = 6;
                            key_back_check = true;
                            points.status = 1;
                            finger_status =  points.status & 0x03;
                        }else if(key_home_pressed && !key_home_check){
                            points.x = 0x21c;
                            points.y = 0x7e2;
                            points.z = 0x33;
                            points.raw_x = 4;
                            points.raw_y = 6;
                            key_home_check = true;
                            points.status = 1;
                            finger_status =  points.status & 0x03;
                    }else{
                            //TPD_DEBUG(" finger %d with !finger_statue and no key match\n",i);
                        }
                    }
            }
#endif
		if (finger_status) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, finger_status);
			input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, points.x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, points.y);
			//#ifdef REPORT_2D_W
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, max(points.raw_x, points.raw_y));
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MINOR, min(points.raw_x, points.raw_y));
			//#endif
#ifdef REPORT_2D_PRESSURE
            if (ts->support_ft){
                input_report_abs(ts->input_dev,ABS_MT_PRESSURE,pres_value);
                TPD_DEBUG("%s: pressure%d[%d]\n",__func__,i,pres_value);
            }
#endif
#ifndef TYPE_B_PROTOCOL
			input_mt_sync(ts->input_dev);
#endif
#ifdef SUPPORT_VIRTUAL_KEY //WayneChang, 2015/12/02, add for key to abs, simulate key in abs through virtual key system
            if(virtual_key_enable){
                complete(&key_cm);
            }
#endif
			finger_num++;
			finger_info |= 1 ;
			//TPD_DEBUG("%s: Finger %d: status = 0x%02x "
					//"x = %4d, y = %4d, wx = %2d, wy = %2d\n",
					//__func__, i, points.status, points.x, points.y, points.raw_x, points.raw_y);

		}
	}

	for ( i = 0; i < ts->max_num; i++ )
	{
		if(0/*(check_key > 1) && (i == 0 )*/){ 
			TPD_ERR("useless solt filter report this one\n");
		}else{
			finger_status = (finger_info>>(ts->max_num-i-1)) & 1 ;
			if(!finger_status)
			{
				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, finger_status);
			}
		}
	}

	last_status = current_status & 0x02;

	if (finger_num == 0/* && last_status && (check_key <= 1)*/)
	{
		if (3 == (++prlog_count % 6))
			TPD_ERR("all finger up\n");
		input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(ts->input_dev);
#endif
	}
	input_sync(ts->input_dev);

	if ((finger_num == 0) && (get_tp_base == 0)){//all finger up do get base once
		get_tp_base = 1;
		TPD_ERR("start get base data:%d\n",get_tp_base);
		tp_baseline_get(ts, false);
	}

#ifdef SUPPORT_GESTURE
	if (ts->in_gesture_mode == 1 && ts->is_suspended == 1) {
		gesture_judge(ts);
	}
#endif
    INT_TOUCH_END:
	mutex_unlock(&ts->mutexreport);
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret,status_check;
	uint8_t status = 0;
	uint8_t inte = 0;

    	struct synaptics_ts_data *ts = ts_g;

	if (atomic_read(&ts->is_stop) == 1)
	{
		touch_disable(ts);
		return;
	}

	if( ts->enable_remote) {
		goto END;
	}
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x00 );
	ret = synaptics_rmi4_i2c_read_word(ts->client, F01_RMI_DATA_BASE);

	if( ret < 0 ) {
		TPDTM_DMESG("Synaptic:ret = %d\n", ret);
        synaptics_hard_reset(ts);
		goto END;
	}
	status = ret & 0xff;
	inte = (ret & 0x7f00)>>8;
	//TPD_ERR("%s status[0x%x],inte[0x%x]\n",__func__,status,inte);
        if(status & 0x80){
		TPD_DEBUG("enter reset tp status,and ts->in_gesture_mode is:%d\n",ts->in_gesture_mode);
		status_check = synaptics_init_panel(ts);
		if (status_check < 0) {
			TPD_ERR("synaptics_init_panel failed\n");
		}
		if ((ts->is_suspended == 1) && (ts->gesture_enable == 1)){
			synaptics_enable_interrupt_for_gesture(ts, 1);
		}
	}
/*
	if(0 != status && 1 != status) {//0:no error;1: after hard reset;the two state don't need soft reset
        TPD_ERR("%s status[0x%x],inte[0x%x]\n",__func__,status,inte);
		int_state(ts);
		goto END;
	}
*/
	if( inte & 0x04 ) {

		int_touch();
	}
END:
	//ret = set_changer_bit(ts);
	touch_enable(ts);
	return;
}

#ifndef TPD_USE_EINT
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	mutex_lock(&ts->mutex);
	synaptics_ts_work_func(ts);
	mutex_unlock(&ts->mutex);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t synaptics_irq_thread_fn(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)dev_id;
	mutex_lock(&ts->mutex);
    touch_disable(ts);
	queue_work(synaptics_report, &ts->report_work);
	mutex_unlock(&ts->mutex);
	return IRQ_HANDLED;
}
#endif

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static ssize_t tp_baseline_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return baseline_ret;
	if(baseline_ret == 0){
		count = synaptics_rmi4_baseline_show(ts->dev,page,0);
		baseline_ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	}else{
		baseline_ret = 0;
	}
	return baseline_ret;
}
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

static ssize_t i2c_device_test_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts_g)
		return ret;
	TPD_DEBUG("gesture enable is: %d\n", ts->gesture_enable);
	ret = sprintf(page, "%d\n", ts->i2c_device_test);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

#ifdef SUPPORT_GESTURE
static ssize_t tp_gesture_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_DEBUG("gesture enable is: %d\n", ts->gesture_enable);
	ret = sprintf(page, "%d\n", ts->gesture_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_gesture_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[10];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;
	if( count > 2 || ts->is_suspended)
		return count;
	if( copy_from_user(buf, buffer, count) ){
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	TPD_ERR("%s write [0x%x]\n",__func__,buf[0]);

    UpVee_gesture = (buf[0] & BIT0)?1:0; //"V"
    DouSwip_gesture = (buf[0] & BIT1)?1:0;//"||"
    LeftVee_gesture = (buf[0] & BIT3)?1:0; //">"
    RightVee_gesture = (buf[0] & BIT4)?1:0;//"<"
    Circle_gesture = (buf[0] & BIT6)?1:0; //"O"
    DouTap_gesture = (buf[0] & BIT7)?1:0; //double tap

	if(DouTap_gesture||Circle_gesture||UpVee_gesture||LeftVee_gesture\
        ||RightVee_gesture||DouSwip_gesture)
	{
		ts->gesture_enable = 1;
	}
	else
    {
        ts->gesture_enable = 0;
    }
	return count;
}
static ssize_t coordinate_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_ERR("%s:gesture_upload = %d \n",__func__,gesture_upload);
	ret = sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture_upload,
			Point_start.x, Point_start.y, Point_end.x, Point_end.y,
			Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
			Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
			clockwise);

	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}


// chenggang.li@BSP.TP modified for oem 2014-08-08 create node
/******************************start****************************/
static const struct file_operations tp_gesture_proc_fops = {
	.write = tp_gesture_write_func,
	.read =  tp_gesture_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations coordinate_proc_fops = {
	.read =  coordinate_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif
static int page ,address,block;
static ssize_t synap_read_address(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret;
	char buffer[PAGESIZE];
    char buf[128];
    int i;
    int cnt = 0;

	struct synaptics_ts_data *ts = ts_g;
    TPD_DEBUG("%s page=0x%x,address=0x%x,block=0x%x\n",__func__,page,address,block);
    cnt += sprintf(&(buffer[cnt]), "page=0x%x,address=0x%x,block=0x%x\n",page,address,block);
    ret = synaptics_rmi4_i2c_write_byte(ts->client,0xff,page);
    ret = synaptics_rmi4_i2c_read_block(ts->client,address,block,buf);
    for (i=0;i < block;i++)
    {
        cnt += sprintf(&(buffer[cnt]), "buf[%d]=0x%x\n",i,buf[i]);
        TPD_DEBUG("buffer[%d]=0x%x\n",i,buffer[i]);
    }
    ret = simple_read_from_buffer(user_buf, count, ppos, buffer, strlen(buffer));
	return ret;
}

static ssize_t synap_write_address(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int buf[128];
    int ret,i;
	struct synaptics_ts_data *ts = ts_g;
    int temp_block,wbyte;
    char reg[30];

    ret = sscanf(buffer,"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",\
    &buf[0],&buf[1],&buf[2],&buf[3],&buf[4],&buf[5],&buf[6],&buf[7],&buf[8],&buf[9],\
    &buf[10],&buf[11],&buf[12],&buf[13],&buf[14],&buf[15],&buf[16],&buf[17]);
    for (i = 0;i < ret;i++)
    {
        TPD_DEBUG("buf[i]=0x%x,",buf[i]);
    }
    TPD_DEBUG("\n");
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
        TPD_DEBUG("%s write page=0x%x,address=0x%x\n",__func__,page,address);
        for (i=0;i < wbyte;i++)
        {
            TPD_DEBUG("reg=0x%x\n",reg[i]);
        }
    }
    else
        block = temp_block;
	return count;
}

#ifdef SUPPORT_GLOVES_MODE
static ssize_t tp_glove_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	TPD_DEBUG("glove mode enable is: %d\n", ts->glove_enable);
	ret = sprintf(page, "%d\n", ts->glove_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_glove_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts= ts_g;
	int ret = 0 ;
	char buf[10]={0};

	if( count > 10 )
		goto GLOVE_ENABLE_END;
	if( copy_from_user( buf, buffer, count) ){
		TPD_ERR("%s: read proc input error.\n", __func__);
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf, "%d", &ret);
	if(!ts)
		return count;
	TPDTM_DMESG("tp_glove_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ){
		ts->glove_enable = ret;
		synaptics_glove_mode_enable(ts);
	}
	switch(ret){
		case 0:
			TPDTM_DMESG("tp_glove_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_glove_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
	return count;
}
#endif


#ifdef SUPPORT_TP_SLEEP_MODE
static ssize_t tp_sleep_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	TPD_DEBUG("sleep mode enable is: %d\n", sleep_enable);
	ret = sprintf(page, "%d\n", sleep_enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t tp_sleep_write_func(struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	char buf[10]={0};
	struct synaptics_ts_data *ts = ts_g;
	int ret = 0 ;
	if( count > 10 )
		return count;
	if(!ts)
		return count;
	if( copy_from_user( buf, buffer, count) ) {
		TPD_ERR(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}
	sscanf(buf, "%d", &ret);
	TPDTM_DMESG("tp_sleep_write_func:buf = %d,ret = %d\n", *buf, ret);
	if( (ret == 0 ) || (ret == 1) ) {
		sleep_enable = ret;
		synaptics_sleep_mode_enable(ts);
	}
	switch(ret) {
		case 0:
			TPDTM_DMESG("tp_sleep_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_sleep_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}
#endif

static ssize_t tp_show(struct device_driver *ddri, char *buf)
{
	// uint8_t ret = 0;
	struct synaptics_ts_data *ts = ts_g;
	int a ;
	int b,c;
	if(!ts)
		return 0;
	a = synaptics_rmi4_i2c_read_word(ts->client, F01_RMI_DATA_BASE);
	if( a < 0 )
		TPD_ERR("tp_show read i2c err\n");
	b = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA01);
	if( b < 0 )
		TPD_ERR("tp_show read i2c err\n");
	c = synaptics_rmi4_i2c_read_byte(ts->client, F12_2D_DATA_BASE);
	if( c < 0 )
		TPD_ERR("tp_show read i2c err\n");

	return sprintf(buf, "F01_RMI_DATA_BASE[0x%x]=0x%x;F01_RMI_DATA01[0x%x]=0x%x;F12_2D_DATA_BASE[0x%x]=0x%x;\n", \
			F01_RMI_DATA_BASE,a,F01_RMI_DATA01,b,F12_2D_DATA_BASE,c);
}

static ssize_t store_tp(struct device_driver *ddri, const char *buf, size_t count)
{
	int tmp = 0;
	if( 1 == sscanf(buf, "%d", &tmp) ){
		tp_debug = tmp;
	} else {
		TPDTM_DMESG("invalid content: '%s', length = %zd\n", buf, count);
	}
	return count;
}
static ssize_t vendor_id_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[4];
	ret = sprintf(page, "%d\n",7);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

#if TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{
	unsigned char buf[4];
	int ret;
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);
	if( ret < 0 ) {
		TPD_ERR("synaptics_rmi4_i2c_write_byte failed for page select\n");
		return -1;
	}
	ret = synaptics_rmi4_i2c_read_block(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	F54_ANALOG_COMMAND_BASE = buf[1];
	F54_ANALOG_CONTROL_BASE = buf[2];
	F54_ANALOG_DATA_BASE = buf[3];

	TPD_ERR("F54_ANALOG_QUERY_BASE   = 0x%x \n \
			F54_ANALOG_COMMAND_BASE  = 0x%x \n\
			F54_ANALOG_CONTROL_BASE	 = 0x%x \n\
			F54_ANALOG_DATA_BASE	 = 0x%x \n\
			",F54_ANALOG_QUERY_BASE,F54_ANALOG_COMMAND_BASE,F54_ANALOG_CONTROL_BASE,F54_ANALOG_DATA_BASE);
	return 0;
}

static void checkCMD(void)
{
	int ret;
	int flag_err = 0;
	struct synaptics_ts_data *ts = ts_g;
	do {
		delay_qt_ms(10); //wait 10ms
		ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_COMMAND_BASE);
		flag_err++;
	}while( (ret > 0x00) && (flag_err < 30) );
	if( ret > 0x00 || flag_err >= 30)
		TPD_ERR("checkCMD error ret is %x flag_err is %d\n", ret, flag_err);
}
#endif

static ssize_t tp_baseline_show(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint16_t tmp_old = 0;
	uint16_t tmp_new = 0;
	uint16_t count = 0;
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
	/*disable irq when read data from IC*/
	touch_disable(ts);
	mutex_lock(&ts->mutex);
	synaptics_read_register_map_page1(ts);

	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");

	/*step 1:check raw capacitance*/
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	if( ret < 0 ){
		TPD_ERR("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+20, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+23);
	tmp_old = ret & 0xff;
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+23, (tmp_old & 0xef));
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
	tmp_new = ret & 0xdf;
	i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation

	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD();
	//TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	//TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
		//printk("\n[%d]", x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h << 8)|tmp_l;
			//printk("%d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%5d", delta_baseline[x][y]);
		}
	}
	TPD_DEBUG("\nread all is oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);
	delay_qt_ms(60);

#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts,1);
	ret = synaptics_soft_reset(ts);
	if (ret < 0){
           TPD_ERR("%s faile to reset device\n",__func__);
        }
	mutex_unlock(&ts->mutex);
	return num_read_chars;

}

static ssize_t tp_rawdata_show(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	int x, y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0, tmp_h = 0;
	uint16_t count = 0;
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return 0;
	memset(delta_baseline, 0, sizeof(delta_baseline));
	/*disable irq when read data from IC*/
	touch_disable(ts);
	mutex_lock(&ts->mutex);
	synaptics_read_register_map_page1(ts);

	//TPD_DEBUG("\nstep 2:report type2 delta image\n");
	memset(delta_baseline, 0, sizeof(delta_baseline));
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_DATA_BASE, 0x02);//select report type 0x02
	ret = synaptics_rmi4_i2c_write_word(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = synaptics_rmi4_i2c_write_byte(ts->client, F54_ANALOG_COMMAND_BASE, 0X01);//get report
	checkCMD();
	count = 0;
	for( x = 0; x < TX_NUM; x++ ){
		//printk("\n[%d]", x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]", x);
		for( y = 0; y < RX_NUM; y++ ){
			ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			//printk("%3d,", delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%3d ", delta_baseline[x][y]);
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_COMMAND_BASE,0X02);
	delay_qt_ms(60);
	synaptics_enable_interrupt(ts, 1);
	mutex_unlock(&ts->mutex);
	touch_enable(ts);
	return num_read_chars;
}

static ssize_t tp_delta_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	TPDTM_DMESG("tp_test_store is not support\n");
	return count;
}

static ssize_t synaptics_rmi4_baseline_show_s3508(struct device *dev, char *buf, bool savefile)
{

	ssize_t num_read_chars = 0;
#if TP_TEST_ENABLE
	int ret = 0;
	uint8_t x,y;
	int tx_datal;
	int16_t baseline_data = 0;
	uint8_t tmp_old = 0;
	uint8_t	tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint16_t count = 0;
	int error_count = 0;
	uint8_t buffer[9];
	int16_t *baseline_data_test;
	int enable_cbc = 1;
	int readdata_fail=0,first_check=0;
	int fd = -1;
	struct timespec   now_time;
	struct rtc_time   rtc_now_time;
	uint8_t  data_buf[64];
	mm_segment_t old_fs;

	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	/*
	   const struct firmware *fw = NULL;
	   struct test_header *ph = NULL;
	   ret = request_firmware(&fw, ts->test_limit_name, dev);
	   if (ret < 0) {
	   TPD_ERR("Request firmware failed - %s (%d)\n",ts->test_limit_name, ret);
	   error_count++;
	   num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);
	   num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
	   return num_read_chars;
	   }

	//ph = (struct test_header *)(fw->data);
	 */
READDATA_AGAIN:
	msleep(30);
	mutex_lock(&ts->mutex);
	touch_disable(ts);

	memset(Rxdata, 0, sizeof(Rxdata));
	synaptics_read_register_map_page1(ts);
	//TPDTM_DMESG("step 1:select report type 0x03\n");

	if(savefile) {
		getnstimeofday(&now_time);
		rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
		sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv",
				(rtc_now_time.tm_year+1900)%100, rtc_now_time.tm_mon+1, rtc_now_time.tm_mday,
				rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);

		old_fs = get_fs();
		set_fs(KERNEL_DS);

		fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
		if (fd < 0) {
			TPD_ERR("Open log file '%s' failed.\n", data_buf);
			set_fs(old_fs);
		}
	}

	//step 1:check raw capacitance.
TEST_WITH_CBC_s3508:
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	if( ret < 0 ){
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed \n");
		goto END;
	}
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+20, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23);
	tmp_old = ret&0xff;

	if(enable_cbc){
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23,(tmp_old | 0x10));
		ret = i2c_smbus_write_word_data(ts->client,F54_ANALOG_COMMAND_BASE,0x04);
		checkCMD();
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
		tmp_new = ret | 0x20;
		i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
		TPD_DEBUG("Test open cbc\n");
		baseline_data_test = (int16_t *)baseline_cap_data[0];

	}else{
		TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
		ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23,(tmp_old & 0xef));
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
		ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
		tmp_new = ret & 0xdf;
		i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
		ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+7, 0x01);// Forbid NoiseMitigation
		baseline_data_test = (int16_t *)baseline_cap_data[1];
	}
	/******write No Relax to 1******/
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04); // force update
	checkCMD();
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);//force Cal
	checkCMD();
	TPD_DEBUG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x00);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();

	count = 0;
	for( x = 0; x < TX_NUM; x++ ){

		for( y = 0; y < RX_NUM; y++ ){
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts->client, F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			if (fd >= 0){
				sprintf(data_buf, "%d,", baseline_data);
				sys_write(fd, data_buf, strlen(data_buf));
			}
			if( (y < RX_NUM ) && (x < TX_NUM) ){
				//printk("%4d ,",baseline_data);
				if(((baseline_data+60) < *(baseline_data_test+count*2)) || ((baseline_data-60) > *(baseline_data_test+count*2+1))){
					TPD_ERR("touchpanel failed,RX_NUM:%d,TX_NUM:%d,baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",y,x,baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
					if((baseline_data <= 0) && (first_check == 0)){
						first_check = 1;
						readdata_fail = 1;
					}
					num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
					error_count++;
					goto END;
				}
			}
			/*
			//test virtual key
			if( (y==(RX_NUM-1)) && (x>= TX_NUM-3) ){
			TPD_DEBUG("synaptics:test virtual key,y= %d ,x = %d\n",y,x);
			TPD_DEBUG("Synaptic:test virtual key;baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
			if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
			TPD_ERR("Synaptic:test virtual key failed------------;baseline_data is %d,TPK_array_limit[%d*2]=%d,TPK_array_limit[%d*2+1]=%d\n ",baseline_data,count,*(baseline_data_test+count*2),count,*(baseline_data_test+count*2+1));
			num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
			error_count++;
			goto END;
			}
			}
			 */
			count++;
		}
		//printk("\n synaptics:s3320 TX_NUM:%d\n",x);
		if (fd >= 0){
			sys_write(fd, "\n", 1);
		}
	}

	if(!enable_cbc){
		enable_cbc = 0;
		TPD_ERR("test cbc baseline again\n");
		goto TEST_WITH_CBC_s3508;
	}
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);
	checkCMD();

	//step 2 :check tx-to-tx and tx-to-vdd
	TPD_ERR("step 2:check TRx-TRx & TRx-Vdd short\n" );
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x1A);//select report type 26
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	//msleep(100);
	checkCMD();
	tx_datal = i2c_smbus_read_i2c_block_data(ts->client, F54_ANALOG_DATA_BASE+3, 7, buffer);
    buffer[0]&=0xfc;//no care 0,1 chanel
    buffer[4]&=0xfc;//no care 32,33 chanel
	for(x = 0;x < 7; x++)
	{
		if(buffer[x]){
			error_count++;
            TPD_ERR("step 2:error_count[%d] buff%d[0x%x] ERROR!\n",error_count,x,buffer[x]);
			goto END;
		}
	}
	num_read_chars += sprintf(buf, "1");

	//Step3 : Check trx-to-ground
	TPD_ERR("step 3:Check trx-to-ground\n" );
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);//software reset TP
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x1A);//select report type 26
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();
	tx_datal = i2c_smbus_read_i2c_block_data(ts->client, F54_ANALOG_DATA_BASE+3, 7, buffer);
    buffer[0]&=0xfc;//no care 0,1 chanel
    buffer[4]&=0xfc;//no care 32,33 chanel
	for(x = 0;x < 7; x++)
	{
		if(buffer[x]){
			error_count++;
            TPD_ERR("step 2:error_count[%d] buff%d[0x%x] ERROR!\n",error_count,x,buffer[x]);
			goto END;
		}
	}

END:
	if (fd >= 0) {
		sys_close(fd);
		set_fs(old_fs);
	}
	//release_firmware(fw);
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);
	delay_qt_ms(60);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
	msleep(150);

#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts, 1);
	touch_enable(ts);
	TPD_ERR("\n\nstep5 reset and open irq complete\n");
	mutex_unlock(&ts->mutex);
#endif
	if(readdata_fail == 1){
		TPD_ERR("readdata_fail...try again:%d\n",first_check);
		readdata_fail =0;
		goto READDATA_AGAIN;
	}
	TPD_ERR("status...first_check:%d:readdata_fail:%d\n",first_check,readdata_fail);
	num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n", TP_FW, TP_FW);
	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");
	return num_read_chars;
}

static ssize_t tp_baseline_show_with_cbc(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0;
	uint8_t tmp_old, tmp_new;
	uint16_t count = 0;
	struct synaptics_ts_data *ts = ts_g;
	if(ts->is_suspended == 1)
		return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
	if(!ts)
		return 0;
	/*disable irq when read data from IC*/
	touch_disable(ts);
	mutex_lock(&ts->mutex);
	synaptics_read_register_map_page1(ts);
	TPD_DEBUG("\nstep 1:select report type 0x03 baseline\n");
	//step 1:check raw capacitance.

	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPDTM_DMESG("step 1: select report type 0x03 failed \n");
		//return sprintf(buf, "i2c err!");
	}

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+20, 0x01);
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23);
	tmp_old = ret&0xff;
	TPD_DEBUG("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
	ret = i2c_smbus_write_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+23,(tmp_old | 0x10));
	ret = i2c_smbus_write_word_data(ts->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	TPD_DEBUG("open CBC oK\n");
	ret = i2c_smbus_read_byte_data(ts->client,F54_ANALOG_CONTROL_BASE+27);
	tmp_new = ret | 0x20;
	i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CONTROL_BASE+27, tmp_new);
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);


	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
	checkCMD();
	TPD_DEBUG("forbid Forbid NoiseMitigation oK\n");

	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("Force Cal oK\n");

	ret = synaptics_rmi4_i2c_write_word(ts->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++) {
		TPD_DEBUG("\n[%d]",x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
		for(y = 0; y < RX_NUM; y++){
			ret = synaptics_rmi4_i2c_read_byte(ts->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = synaptics_rmi4_i2c_read_byte(ts->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			TPD_DEBUG("%d,",delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",delta_baseline[x][y]);
		}
	}

	ret = synaptics_rmi4_i2c_write_byte(ts->client,F54_ANALOG_COMMAND_BASE,0X02);
	delay_qt_ms(60);
	synaptics_enable_interrupt(ts,1);
	mutex_unlock(&ts->mutex);
	touch_enable(ts);
	return num_read_chars;
}

static ssize_t synaptics_rmi4_baseline_show(struct device *dev,	char *buf, bool savefile)
{
	return synaptics_rmi4_baseline_show_s3508(dev,buf,savefile);
}

static ssize_t tp_test_store(struct device_driver *ddri,
		const char *buf, size_t count)
{
	TPDTM_DMESG("tp_test_store is not support\n");
	return count;
}

static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if( (tp_dev == TP_G2Y) || (tp_dev == TP_TPK) )
		return sprintf(buf, "%d\n", TP_TPK);
	if(tp_dev == TP_TRULY)
		return sprintf(buf, "%d\n", TP_TRULY);
	if(tp_dev == TP_OFILM)
		return sprintf(buf, "%d\n", TP_OFILM);
	return sprintf(buf, "%d\n", tp_dev);
}


static int	synaptics_input_init(struct synaptics_ts_data *ts)
{
	int attr_count = 0;
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
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef SUPPORT_GESTURE
	set_bit(KEY_F4 , ts->input_dev->keybit);//doulbe-tap resume
#endif
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, (ts->max_x-1), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, (ts->max_y-1), 0, 0);
#ifdef REPORT_2D_PRESSURE
    if (ts->support_ft){
        input_set_abs_params(ts->input_dev,ABS_MT_PRESSURE,0,255, 0, 0);
    }
#endif
#ifdef TYPE_B_PROTOCOL
	input_mt_init_slots(ts->input_dev, ts->max_num, 0);
#endif
	input_set_drvdata(ts->input_dev, ts);

	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);
		input_unregister_device(ts->input_dev);
		input_free_device(ts->input_dev);
		return -1;
	}
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs_oem); attr_count++) {
		ret = sysfs_create_file(&ts->input_dev->dev.kobj,
				&attrs_oem[attr_count].attr);
		if (ret < 0) {
			dev_err(&ts->client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			for (attr_count--; attr_count >= 0; attr_count--) {
				sysfs_remove_file(&ts->input_dev->dev.kobj,
						&attrs_oem[attr_count].attr);
			}
			return -1;
		}
	}
	return 0;
}

#include "fw_update_v7.if"
static int check_hardware_version(struct device *dev)
{
        int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
        const struct firmware *fw = NULL;
	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");
		return 0;
	}
	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
		return ret;
	}

        ret = fwu_start_reflash_check(fw->data,ts->client);
	release_firmware(fw);
        if (ret < 0)
		return -1;
        else
            	return ret;
}
static int check_version = 0;
/*********************FW Update Func******************************************/
static int synatpitcs_fw_update(struct device *dev, bool force)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int ret;
	char fw_id_temp[12];
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0 ;

        static bool check_onetime = true;

        TPD_DEBUG("%s is called\n",__func__);
	if(!ts->client) {
		TPD_ERR("i2c client point is NULL\n");
		return 0;
	}
        if(check_onetime){
		check_onetime = false;
		check_version = check_hardware_version(dev);
		TPD_ERR("%s:first check hardware version %d\n",__func__,check_version);
		if(check_version < 0){
			TPD_ERR("checkversion fail....\n");
			return -1;
		}
	}

        if(1 == check_version ) {
		TPD_DEBUG("enter version 15801 update mode\n");
	        strcpy(ts->fw_name,"tp/fw_synaptics_15801.img");
		push_component_info(TP, ts->fw_id, "S3718_vA");
		ret = request_firmware(&fw, ts->fw_name, dev);
		if (ret < 0) {
			TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
			return ret;
               }

	 }else{
                TPD_DEBUG("enter version 15801 vb update mode\n");
		push_component_info(TP, ts->fw_id, "S3718_vB");
		ret = request_firmware(&fw, ts->fw_name, dev);
		if (ret < 0) {
			TPD_ERR("Request firmware failed - %s (%d)\n",ts->fw_name, ret);
			return ret;
               }
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
	ret = synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL00, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	sprintf(fw_id_temp,"0x%x",CURRENT_FIRMWARE_ID);
	strcpy(ts->fw_id,fw_id_temp);
	report_key_point_y = ts->max_y*button_map[2]/LCD_HEIGHT;
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif
	synaptics_init_panel(ts);
	synaptics_enable_interrupt(ts,1);
	return 0;
}

static ssize_t synaptics_update_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t synaptics_update_fw_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

    if (ts->is_suspended && ts->support_hw_poweroff){
        TPD_ERR("power off firmware abort!\n");
        return size;
    }

    if (strncmp(ts->manu_name,"S3718",5)){
        TPD_ERR("product name[%s] do not update!\n",ts->manu_name);
        return size;
    }
	TPD_ERR("start update ******* fw_name:%s\n",ts->fw_name);

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if(!val)
		val = force_update;

	touch_disable(ts);
	mutex_lock(&ts->mutex);
	ts->loading_fw = true;
	synatpitcs_fw_update(dev, val);
	ts->loading_fw = false;
	mutex_unlock(&ts->mutex);
	touch_enable(ts);
	force_update = 0;
	return size;
}
/*********************FW Update Func End*************************************/


static ssize_t synaptics_test_limit_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	int ret = 0;
	uint16_t *prow = NULL;
	uint16_t *prowcbc = NULL;
	const struct firmware *fw = NULL;
	struct test_header *ph = NULL;
	int i = 0;
	int temp = 0;
	static int cat_cbc_change = 0;
	ret = request_firmware(&fw, ts->test_limit_name, dev);
	if (ret < 0) {
		TPD_ERR("Request firmware failed - %s (%d)\n",
				ts->test_limit_name, ret);
		temp = temp + sprintf(&buf[temp],"Request failed,Check the path %d",temp);
		return temp;
	}

	ph = (struct test_header *)(fw->data);
	prow = (uint16_t *)(fw->data + ph->array_limit_offset);

	prowcbc = (uint16_t *)(fw->data + ph->array_limitcbc_offset);

	TPD_DEBUG("synaptics_test_limit_show:array_limit_offset = %x array_limitcbc_offset = %x\n",
			ph->array_limit_offset,ph->array_limitcbc_offset);

	TPD_DEBUG("test begin:\n");
	if(cat_cbc_change == 0 || ph->withCBC == 0) {
		temp += sprintf(buf, "Without cbc:");
		for(i = 0 ;i < (ph->array_limit_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
				temp += sprintf(&(buf[temp]), "\n[%d] ",(i/RX_NUM)/2);
			temp += sprintf(&buf[temp],"%d,",prow[i]);
			printk("%d,",prow[i]);
		}
		cat_cbc_change = 1;
	}else{
		temp += sprintf(buf, "With cbc:");
		cat_cbc_change = 0;
		if( ph->withCBC == 0){
			return temp;
		}
		for(i = 0 ;i < (ph->array_limitcbc_size/2 ); i++){
			if(i % (2*RX_NUM) == 0)
				temp += sprintf(&(buf[temp]), "\n[%d] ",(i/RX_NUM)/2);
			temp += sprintf(&buf[temp],"%d,",prowcbc[i]);
			printk("%d,",prowcbc[i]);
		}
	}
	release_firmware(fw);
	return temp;
}

static ssize_t synaptics_test_limit_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	return size;
}

//static DRIVER_ATTR(tp_baseline_image_with_cbc, 0664, tp_baseline_show_with_cbc, tp_test_store);
static DEVICE_ATTR(test_limit, 0664, synaptics_test_limit_show, synaptics_test_limit_store);
static DRIVER_ATTR(tp_baseline_image, 0664, tp_baseline_show, tp_delta_store);
static DRIVER_ATTR(tp_baseline_image_with_cbc, 0664, tp_baseline_show_with_cbc, tp_test_store);
static DRIVER_ATTR(tp_delta_image, 0664, tp_rawdata_show, NULL);
static DRIVER_ATTR(tp_debug_log, 0664, tp_show, store_tp);
static DEVICE_ATTR(tp_fw_update, 0664, synaptics_update_fw_show, synaptics_update_fw_store);
static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts);

static ssize_t tp_reset_write_func (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    int ret,write_flag,i;
	struct synaptics_ts_data *ts = ts_g;

	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!!\n",__func__);
		return count;
	}

    ret = sscanf(buffer,"%x",&write_flag);
    TPD_ERR("%s write [%d]\n",__func__,write_flag);
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
        disable_irq_nosync(ts->irq);
    }
    else if(4 == write_flag)
    {
        enable_irq(ts->irq);
    }
    else if(8 == write_flag)
    {
        touch_enable(ts);
    }
    else if(9 == write_flag)
    {
        touch_disable(ts);
    }
    else if(5 == write_flag)
    {
        synaptics_read_register_map(ts);
    }
    else if(6 == write_flag)
    {

        for (i = 0; i < ts->max_num; i++)
        {
            input_mt_slot(ts->input_dev, i);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
            input_mt_slot(ts->input_dev, i);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
        }
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(ts->input_dev);
#endif
        input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);
    }
	return count;
}

//chenggang.li@bsp add for 14045
static const struct file_operations base_register_address= {
	.write = synap_write_address,
	.read =  synap_read_address,
	.open = simple_open,
	.owner = THIS_MODULE,
};


//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations i2c_device_test_fops = {
	.read =  i2c_device_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
static const struct file_operations tp_baseline_test_proc_fops = {
	.read =  tp_baseline_test_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end

#ifdef SUPPORT_GLOVES_MODE
static const struct file_operations glove_mode_enable_proc_fops = {
	.write = tp_glove_write_func,
	.read =  tp_glove_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#endif

static const struct file_operations sleep_mode_enable_proc_fops = {
	.write = tp_sleep_write_func,
	.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static const struct file_operations tp_reset_proc_fops = {
	.write = tp_reset_write_func,
	//.read =  tp_sleep_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static const struct file_operations vendor_id_proc_fops = {
	.read =  vendor_id_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
static int set_changer_bit(struct synaptics_ts_data *ts)
{
    int mode;
    int ret;
    mode = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_CTRL00);
    if (ts->changer_connet)
        mode = mode | 0x20;
    else
        mode = mode & 0xDF;
    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, mode);
    return ret;
}
static ssize_t changer_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	struct synaptics_ts_data *ts = ts_g;
	if(!ts)
		return ret;
	ret = sprintf(page, "the changer is %s!\n", ts->changer_connet?("conneted"):("disconneted"));
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t changer_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts= ts_g;
	int ret = 0 ;

	sscanf(&buffer[0], "%d", &ret);
	if(!ts)
		return count;
	if( (ret == 0 ) || (ret == 1) ){
		ts->changer_connet = ret;
        ret = set_changer_bit(ts);
	}
	TPDTM_DMESG("%s:ts->changer_connet = %d\n",__func__,ts->changer_connet);
	return count;
}
static const struct file_operations changer_ops = {
	.write = changer_write_func,
	.read =  changer_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
#define SUBABS(x,y) ((x)-(y))
static int tp_baseline_get(struct synaptics_ts_data *ts, bool flag)
{
	int ret = 0;
	int x, y;
	uint8_t *value;
	int k = 0;

	if(!ts)
		return -1;

	atomic_set(&ts->is_stop,1);
	touch_disable(ts);
	TPD_DEBUG("%s start!\n",__func__);
	value = kzalloc(TX_NUM*RX_NUM*2, GFP_KERNEL);
	memset(delta_baseline,0,sizeof(delta_baseline));

	mutex_lock(&ts->mutex);
	if (ts->gesture_enable)
		synaptics_enable_interrupt_for_gesture(ts,false);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x1);

	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_DATA_BASE, 0x03);//select report type 0x03
	ret = i2c_smbus_write_word_data(ts->client, F54_ANALOG_DATA_BASE+1, 0);//set fifo 00
	ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x01);//get report
	checkCMD();

	ret = synaptics_rmi4_i2c_read_block(ts->client,F54_ANALOG_DATA_BASE+3,2*TX_NUM*RX_NUM,value);
	for( x = 0; x < TX_NUM; x++ ){
		for( y = 0; y < RX_NUM; y++ ){
			delta_baseline[x][y] =  (int16_t)(((uint16_t)( value [k])) | ((uint16_t)( value [k+1] << 8)));
			k = k + 2;

			if (y >= 20){
				if (flag)
					delta[y-20][x] = SUBABS(delta_baseline[x][y],baseline[y-20][x]);
				else
					baseline[y-20][x] = delta_baseline[x][y];
			}
		}
	}
        //ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0X02);
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);//soft reset
	mutex_unlock(&ts->mutex);
	atomic_set(&ts->is_stop,0);
	msleep(2);
	touch_enable(ts);
	TPD_DEBUG("%s end! \n",__func__);
	kfree(value);
	return 0;
}
static void tp_baseline_get_work(struct work_struct *work)
{
	struct synaptics_ts_data *ts= ts_g;

	tp_baseline_get(ts, true);//get the delta data
}

static ssize_t touch_press_status_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	int x,y;
	int press_points = 0;
	int points_misspresee =0;
	int str_n = 0;

	char *page = kzalloc(1024*2,GFP_KERNEL);
	if (!page){
		TPD_ERR("%s malloc memery error!",__func__);
		return -ENOMEM;
	}
	TPD_ERR("%s",__func__);

	for (x = 0; x < 8; x++){
		str_n += sprintf(&page[str_n], "\n");
		for (y = 0; y < 16; y++){
			str_n += sprintf(&page[str_n],"%4d",delta[x][y]);
			if ((delta[x][y] < -30) && (delta[x][y] > -250))
				press_points++;
			if((delta[x][y] > 30) && (delta[x][y] < 200))
				points_misspresee ++;
		}

	}

	if(points_misspresee > 4)
		get_tp_base = 0;
	TPD_ERR("points_mispressee num:%d,get_tp_base:%d\n",points_misspresee,get_tp_base);
	str_n += sprintf(&page[str_n], "\n%s %d points delta > [25]\n",(press_points>4)?"near":"away", press_points);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	kfree(page);
	return ret;
}

static ssize_t touch_press_status_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct synaptics_ts_data *ts= ts_g;
	int ret = 0 ;

	sscanf(&buffer[0], "%d", &ret);
	if(!ts)
		return count;

	TPD_ERR("%s write %d\n",__func__,ret);
	if (ret == 0){
		tp_baseline_get(ts,false);
	}
	else if(ret == 1) {
		if (0 == ts->gesture_enable)
			queue_delayed_work(get_base_report, &ts->base_work,msecs_to_jiffies(120));
		else
			queue_delayed_work(get_base_report, &ts->base_work,msecs_to_jiffies(1));
	}
	return count;
}
static const struct file_operations touch_press_status = {
	.write = touch_press_status_write,
	.read =  touch_press_status_read,
	.open = simple_open,
	.owner = THIS_MODULE,
};

static int init_synaptics_proc(void)
{
	int ret = 0;
	struct proc_dir_entry *prEntry_tmp  = NULL;
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if( prEntry_tp == NULL ){
		ret = -ENOMEM;
		TPD_ERR("Couldn't create touchpanel\n");
	}

#ifdef SUPPORT_GESTURE
	prEntry_tmp = proc_create( "gesture_enable", 0666, prEntry_tp, &tp_gesture_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create gesture_enable\n");
	}

	prEntry_tmp = proc_create("coordinate", 0444, prEntry_tp, &coordinate_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create coordinate\n");
	}
#endif

#ifdef SUPPORT_GLOVES_MODE
	prEntry_tmp = proc_create( "glove_mode_enable", 0666, prEntry_tp,&glove_mode_enable_proc_fops);
	if(prEntry_tmp == NULL) {
		ret = -ENOMEM;
        TPD_ERR("Couldn't create glove_mode_enable\n");
	}
#endif

#ifdef SUPPORT_TP_SLEEP_MODE
	prEntry_tmp = proc_create("sleep_mode_enable", 0666, prEntry_tp, &sleep_mode_enable_proc_fops);
	if( prEntry_tmp == NULL ){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create sleep_mode_enable\n");
	}
#endif

#ifdef RESET_ONESECOND
	prEntry_tmp = proc_create( "tp_reset", 0666, prEntry_tp, &tp_reset_proc_fops);
	if( prEntry_tmp == NULL ){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create tp_reset\n");
	}
#endif
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  begin
	prEntry_tmp = proc_create( "baseline_test", 0666, prEntry_tp, &tp_baseline_test_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create baseline_test\n");
	}
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\baseline_test"  end
	//wangwenxue@BSP add for change baseline_test to "proc\touchpanel\i2c_device_test"  begin
	prEntry_tmp = proc_create( "i2c_device_test", 0666, prEntry_tp, &i2c_device_test_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create i2c_device_test\n");
	}

	prEntry_tmp = proc_create( "radd", 0777, prEntry_tp, &base_register_address);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create radd\n");
	}
	prEntry_tmp = proc_create("vendor_id", 0444, prEntry_tp, &vendor_id_proc_fops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create vendor_id\n");
	}
	prEntry_tmp = proc_create("changer_connet", 0666, prEntry_tp, &changer_ops);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create changer_connet\n");
	}

	prEntry_tmp = proc_create("touch_press", 0666, prEntry_tp, &touch_press_status);
	if(prEntry_tmp == NULL){
		ret = -ENOMEM;
        TPD_ERR("Couldn't create touch_press\n");
	}
	return ret;
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
	SynaF34Reflash_BlockData = SynaF34DataBase + 1;
	SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
	SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
	SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
	SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
	i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
	SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	TPD_DEBUG("SynaFirmwareBlockSize 3310 is %d\n", SynaFirmwareBlockSize);
	SynaF34_FlashControl = SynaF34DataBase + 2;
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
	TPD_DEBUG(" debug checksume is %x", header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_DEBUG(" debug bootloader_version is %d\n", header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_DEBUG(" debug firmware_size is %x", header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_DEBUG(" debug header->config_size is %x", header->config_size);

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
	ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
	while ( (ret != 0x80)&&(count < 8) ) {
		msleep(3); //wait 3ms
		ret =  synaptics_rmi4_i2c_read_byte(client,SynaF34_FlashControl+1);
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
	uint8_t buf[4];
	uint32_t bootloader_mode;
	int max_y_ic = 0;
	int max_x_ic = 0;
	if(!ts){
		TPD_ERR("%s ts is NULL\n",__func__);
		return -1;
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}

	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_ERR("failed to read product info \n");
		return -1;
	}
	/*read max_x ,max_y*/
	ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
	if (ret < 0) {
		ret = synaptics_rmi4_i2c_write_byte(ts->client, 0xff, 0x0);
		if(ret < 0 ){
			TPD_ERR("synaptics_rmi4_i2c_write_byte failed for page select\n");
			return -1;
		}
	}

	i2c_smbus_read_i2c_block_data(ts->client, F12_2D_CTRL08, 14, buf);
	max_x_ic = ( (buf[1]<<8)&0xffff ) | (buf[0]&0xffff);
	max_y_ic = ( (buf[3]<<8)&0xffff ) | (buf[2]&0xffff);

	TPD_ERR("max_x = %d,max_y = %d; max_x_ic = %d,max_y_ic = %d\n",ts->max_x,ts->max_y,max_x_ic,max_y_ic);
	if((ts->max_x == 0) ||(ts->max_y ==0 )) {
		ts->max_x = max_x_ic;
		ts->max_y = max_y_ic;
	}
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40;
	TPD_DEBUG("afte fw update,program memory self-check bootloader_mode = 0x%x\n",bootloader_mode);

	if((max_x_ic == 0)||(max_y_ic == 0)||(bootloader_mode == 0x40)) {
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
    if (!strncmp(ts->manu_name,"S3718",5)){
        Config_Data = data + 0x8f0;
        ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);
        ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL00, 4, buf);
        CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
        FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];
	if(1 == check_version)
		TPD_ERR("CURRENT_FW_ID:%x------, FW_ID:%x------,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
	else
		TPD_ERR("CURRENT_FW_ID:%xvB------, FW_ID:%xvB------,FW_NAME:%s\n", CURRENT_FIRMWARE_ID, FIRMWARE_ID,ts->fw_name);
        //TPD_ERR("synaptics force is %d\n", force);
        if(!force) {
            if(CURRENT_FIRMWARE_ID == FIRMWARE_ID) {
                return 0;
            }
        }
        ret = fwu_start_reflash(data,client);
        if (ret){
            return -1;
        }
    }else{
	parse_header(&header,data);
	if((header.firmware_size + header.config_size + 0x100) > data_len) {
		TPDTM_DMESG("firmware_size + config_size + 0x100 > data_len data_len = %d \n",data_len);
		return -1;
	}

	Firmware_Data = data + 0x100;
	Config_Data = Firmware_Data + header.firmware_size;
	ret = synaptics_rmi4_i2c_write_byte(client, 0xff, 0x0);

	ret = synaptics_rmi4_i2c_read_block(client, F34_FLASH_CTRL00, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	FIRMWARE_ID = (Config_Data[0]<<24)|(Config_Data[1]<<16)|(Config_Data[2]<<8)|Config_Data[3];

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
	TPD_DEBUG("attn step 4\n");
	ret=checkFlashState(client);
	if(ret > 0) {
		TPD_ERR("Get in prog:The status(Image) of flashstate is %x\n",ret);
		return -1;
	}
	ret = i2c_smbus_read_byte_data(client,0x04);
	TPD_DEBUG("The status(device state) is %x\n",ret);
	ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
	TPD_DEBUG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
	ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);
	/********************get into prog end************/
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	TPD_DEBUG("ret is %d\n",ret);
	re_scan_PDT(client);
	i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
	i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
	i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
	msleep(2000);
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
    }
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

	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!\n",__func__);
		return -1;
	}
    touch_disable(ts);
    ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
    if (ret < 0){
    TPD_ERR("reset error ret=%d\n",ret);
    }
    TPD_ERR("%s !!!\n",__func__);
    msleep(100);
    touch_enable(ts);
    return ret;
}
static void synaptics_hard_reset(struct synaptics_ts_data *ts)
{
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
	struct device_node *np;
	int temp_array[2];


	np = dev->of_node;
	ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio", 0, &(ts->irq_flags));
	if( ts->irq_gpio < 0 ){
		TPD_DEBUG("ts->irq_gpio not specified\n");
	}

	ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
	if( ts->reset_gpio < 0 ){
		TPD_DEBUG("ts->reset-gpio  not specified\n");
	}
	ts->v1p8_gpio = of_get_named_gpio(np, "synaptics,1v8-gpio", 0);
	if( ts->v1p8_gpio < 0 ){
		TPD_DEBUG("ts->1v8-gpio  not specified\n");
	}

	if(of_property_read_bool(np, "oem,support_hw_poweroff"))
		ts->support_hw_poweroff=true;
	else
		ts->support_hw_poweroff=false;

	TPD_ERR("%s ts->support_hw_poweroff =%d\n",__func__,ts->support_hw_poweroff);

	ts->enable2v8_gpio = of_get_named_gpio(np, "synaptics,enable2v8-gpio", 0);
	if( ts->enable2v8_gpio < 0 ){
		TPD_DEBUG("ts->enable2v8_gpio not specified\n");
	}

	rc = of_property_read_u32(np, "synaptics,max-num-support", &ts->max_num);
	if(rc){
		TPD_DEBUG("ts->max_num not specified\n");
		ts->max_num = 10;
	}

	rc = of_property_read_u32_array(np, "synaptics,button-map", button_map, 3);
	if(rc){
		TPD_DEBUG("button-map not specified\n");
		//button_map[0] = 180;
		//button_map[1] = 180;
		//button_map[2] = 2021;
	}
	TPD_DEBUG("synaptics:button map readed is %d %d %d\n", button_map[0], button_map[1], button_map[2]);

	rc = of_property_read_u32_array(np, "synaptics,tx-rx-num", tx_rx_num,2);
	if(rc){
		TPD_ERR("button-map not specified\n");
		TX_NUM =  30;
		RX_NUM =  17;
	}else{
		TX_NUM =  tx_rx_num[0];
		RX_NUM =  tx_rx_num[1];
	}
	TPD_ERR("synaptics,tx-rx-num is %d %d \n", TX_NUM,RX_NUM);

	rc = of_property_read_u32_array(np, "synaptics,display-coords", temp_array, 2);
	if(rc){
		TPD_ERR("lcd size not specified\n");
		LCD_WIDTH = 1080;
		LCD_HEIGHT = 1920;
	}else{
		LCD_WIDTH = temp_array[0];
		LCD_HEIGHT = temp_array[1];
	}
	rc = of_property_read_u32_array(np, "synaptics,panel-coords", temp_array, 2);
	if(rc){
		ts->max_x = 1080;
		ts->max_y = 1920;
	}else{
		ts->max_x = temp_array[0];
		ts->max_y = temp_array[1];
	}
    TPDTM_DMESG("synaptic:ts->irq_gpio:%d irq_flags:%u max_num %d\n"\
        ,ts->irq_gpio, ts->irq_flags, ts->max_num);

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
			rc = gpio_request(ts->reset_gpio, "tp-s3320-reset");
			if(rc){
				TPD_ERR("unable to request reset_gpio [%d]\n", ts->reset_gpio);
			}
		}
	}
	if( ts->v1p8_gpio > 0){
		if( gpio_is_valid(ts->v1p8_gpio) ){
			rc = gpio_request(ts->v1p8_gpio, "tp-s3320-1v8");
			if(rc){
				TPD_ERR("unable to request v1p8_gpio [%d]\n", ts->v1p8_gpio);
			}
		}
	}

	if( ts->enable2v8_gpio > 0){
		if( gpio_is_valid(ts->enable2v8_gpio) ){
			rc = gpio_request(ts->enable2v8_gpio, "rmi4-enable2v8-gpio");
			if(rc)
				TPD_ERR("unable to request enable2v8_gpio [%d]\n", ts->enable2v8_gpio);

		}
	}

	return rc;
}

static int synaptics_dsx_pinctrl_init(struct synaptics_ts_data *ts)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	ts->pinctrl = devm_pinctrl_get((ts->dev));
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		retval = PTR_ERR(ts->pinctrl);
        TPD_ERR("%s pinctrl error!\n",__func__);
		goto err_pinctrl_get;
	}

	ts->pinctrl_state_active
		= pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_active)) {
		retval = PTR_ERR(ts->pinctrl_state_active);
        TPD_ERR("%s pinctrl state active error!\n",__func__);
		goto err_pinctrl_lookup;
	}

	ts->pinctrl_state_suspend
		= pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pinctrl_state_suspend)) {
		retval = PTR_ERR(ts->pinctrl_state_suspend);
        TPD_ERR("%s pinctrl state suspend error!\n",__func__);
		goto err_pinctrl_lookup;
	}
	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ts->pinctrl);
err_pinctrl_get:
	ts->pinctrl = NULL;
	return retval;
}

#ifdef SUPPORT_VIRTUAL_KEY
#define VK_KEY_X    180
#define VK_CENTER_Y 2020//2260
#define VK_WIDTH    170
#define VK_HIGHT    200
static ssize_t vk_syna_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
    int len ;

    len =  sprintf(buf,
            __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT)  ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":%d:%d:%d:%d" "\n",
            VK_KEY_X,   VK_CENTER_Y, VK_WIDTH, VK_HIGHT,
            VK_KEY_X*3, VK_CENTER_Y, VK_WIDTH, VK_HIGHT,
            VK_KEY_X*5, VK_CENTER_Y, VK_WIDTH, VK_HIGHT);

    return len ;
}

static struct kobj_attribute vk_syna_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &vk_syna_show,
};

static struct attribute *syna_properties_attrs[] = {
    &vk_syna_attr.attr,
    NULL
};

static struct attribute_group syna_properties_attr_group = {
    .attrs = syna_properties_attrs,
};
static int synaptics_ts_init_virtual_key(struct synaptics_ts_data *ts )
{
    int ret = 0;

    /* virtual keys */
    if(ts->properties_kobj)
        return 0 ;
    ts->properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (ts->properties_kobj)
        ret = sysfs_create_group(ts->properties_kobj, &syna_properties_attr_group);

    if (!ts->properties_kobj || ret)
        printk("%s: failed to create board_properties\n", __func__);
    /* virtual keys */
    return ret;
}
#endif

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
	ts->support_ft = true;
	ts_g = ts;
	get_tp_base = 0;

	synaptics_parse_dts(&client->dev, ts);

	/***power_init*****/
	ret = tpd_power(ts, 1);
	if( ret < 0 )
		TPD_ERR("regulator_enable is called\n");
    ret = synaptics_dsx_pinctrl_init(ts);
    if (!ret && ts->pinctrl) {
        ret = pinctrl_select_state(ts->pinctrl,
                ts->pinctrl_state_active);
        }
    msleep(100);//after power on tp need sometime from bootloader to ui mode
	mutex_init(&ts->mutex);
	mutex_init(&ts->mutexreport);
    atomic_set(&ts->irq_enable,0);

	ts->is_suspended = 0;
	atomic_set(&ts->is_stop,0);
    spin_lock_init(&ts->lock);
	/*****power_end*********/
	if( !i2c_check_functionality(client->adapter, I2C_FUNC_I2C) ){
		TPD_ERR("%s [ERR]need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
	if( ret < 0 ) {
		ret = synaptics_rmi4_i2c_read_byte(client, 0x13);
		if( ret < 0 ) {
		#ifdef SUPPORT_VIRTUAL_KEY
                        virtual_key_enable = 0;//if touch is no valid report key
                #endif
                        TPD_ERR("tp is no exist!\n");
			goto err_check_functionality_failed;
		}
	}

	ts->i2c_device_test = ret;

	synaptics_read_register_map(ts);
	bootloader_mode = synaptics_rmi4_i2c_read_byte(ts->client, F01_RMI_DATA_BASE);

	bootloader_mode = bootloader_mode&0x40;
	TPD_ERR("before fw update bootloader_mode[0x%x]\n", bootloader_mode);

	synaptics_rmi4_i2c_read_block(ts->client, F34_FLASH_CTRL00, 4, buf);
	CURRENT_FIRMWARE_ID = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
	TPD_ERR("CURRENT_FIRMWARE_ID = 0x%x\n", CURRENT_FIRMWARE_ID);
	TP_FW = CURRENT_FIRMWARE_ID;
	sprintf(ts->fw_id,"0x%x",TP_FW);

	memset(ts->fw_name,0,TP_FW_NAME_MAX_LEN);
	memset(ts->test_limit_name,0,TP_FW_NAME_MAX_LEN);

	//sprintf(ts->manu_name, "TP_SYNAPTICS");
    synaptics_rmi4_i2c_read_block(ts->client, F01_RMI_QUERY11,\
        sizeof(ts->manu_name), ts->manu_name);

	strcpy(ts->fw_name,"tp/fw_synaptics_15801b.img");
	strcpy(ts->test_limit_name,"tp/14049/14049_Limit_jdi.img");
	TPD_DEBUG("synatpitcs_fw: fw_name = %s \n",ts->fw_name);

	//push_component_info(TP, ts->fw_id, ts->manu_name);

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if( !synaptics_wq ){
		ret = -ENOMEM;
		goto exit_createworkqueue_failed;
	}
	INIT_DELAYED_WORK(&ts->speed_up_work,speedup_synaptics_resume);

	synaptics_report = create_singlethread_workqueue("synaptics_report");
	if( !synaptics_report ){
		ret = -ENOMEM;
		goto exit_createworkqueue_failed;
	}

	get_base_report = create_singlethread_workqueue("get_base_report");
	if( !get_base_report ){
		ret = -ENOMEM;
		goto exit_createworkqueue_failed;
	}
	INIT_WORK(&ts->report_work,synaptics_ts_work_func);
	INIT_DELAYED_WORK(&ts->base_work,tp_baseline_get_work);

	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPD_ERR("synaptics_init_panel failed\n");
	}

	//Detect whether TP FW is error, max_x,max_y may be incoorect while it has been damaged!
	ret = synaptics_fw_check(ts);
	if(ret < 0 ) {
		force_update = 1;
		TPD_ERR("This FW need to be updated!\n");
	} else {
		force_update = 0;
	}
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if( ret < 0 ) {
		TPD_ERR(" synaptics_ts_probe: disable interrupt failed\n");
	}
    ret = synaptics_soft_reset(ts);
    if (ret < 0){
        TPD_ERR("%s faile to reset device\n",__func__);
    }
	ret = synaptics_input_init(ts);
	if(ret < 0) {
		TPD_ERR("synaptics_input_init failed!\n");
	}
#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if(ret)
		TPD_ERR("Unable to register fb_notifier: %d\n", ret);
#endif



#ifndef TPD_USE_EINT
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#ifdef TPD_USE_EINT
	/****************
	  shoud set the irq GPIO
	 *******************/
	if (gpio_is_valid(ts->irq_gpio)) {
        /* configure touchscreen irq gpio */
        ret = gpio_request(ts->irq_gpio,"tp-s3320-irq");
        if (ret) {
            TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
        }
        ret = gpio_direction_input(ts->irq_gpio);
        msleep(50);
        ts->irq = gpio_to_irq(ts->irq_gpio);
	}
	TPD_ERR("synaptic:ts->irq is %d\n",ts->irq);

	ret = request_threaded_irq(ts->irq, NULL,
			synaptics_irq_thread_fn,
			ts->irq_flags | IRQF_ONESHOT,
			TPD_DEVICE, ts);
	if(ret < 0)
		TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
    msleep(5);
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret < 0)
		TPD_ERR("%s enable interrupt error ret=%d\n",__func__,ret);
#endif

	if (device_create_file(&client->dev, &dev_attr_test_limit)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	TPD_DEBUG("synaptics_ts_probe: going to create files--tp_fw_update\n");
	if (device_create_file(&client->dev, &dev_attr_tp_fw_update)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_debug_log)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_baseline_image_with_cbc)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_baseline_image)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if( driver_create_file(&tpd_i2c_driver.driver, &driver_attr_tp_delta_image)) {
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
#ifdef SUPPORT_VIRTUAL_KEY
    synaptics_ts_init_virtual_key(ts);
#endif
#ifdef CONFIG_SYNAPTIC_RED
	premote_data = remote_alloc_panel_data();
	if(premote_data) {
		premote_data->client 		= client;
		premote_data->input_dev		= ts->input_dev;
		premote_data->pmutex		= &ts->mutex;
		premote_data->irq_gpio 		= ts->irq_gpio;
		premote_data->irq			= client->irq;
		premote_data->enable_remote = &(ts->enable_remote);
		register_remote_device(premote_data);

	}
#endif
	init_synaptics_proc();
	TPDTM_DMESG("synaptics_ts_probe 3203: normal end\n");
	return 0;

exit_init_failed:
	free_irq(client->irq,ts);
exit_createworkqueue_failed:
	destroy_workqueue(synaptics_wq);
	synaptics_wq = NULL;
	destroy_workqueue(synaptics_report);
	synaptics_report = NULL;
	destroy_workqueue(get_base_report);
	get_base_report = NULL;

err_check_functionality_failed:
	tpd_power(ts, 0);
err_alloc_data_failed:
	tpd_i2c_driver.driver.pm=NULL;
	kfree(ts);
	ts = NULL;
	ts_g = NULL;
	TPD_ERR("synaptics_ts_probe: not normal end\n");
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	int attr_count;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	TPD_ERR("%s is called\n",__func__);
#ifdef CONFIG_SYNAPTIC_RED
	unregister_remote_device();
#endif

#if defined(CONFIG_FB)
	if( fb_unregister_client(&ts->fb_notif) )
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif

#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif

	for(attr_count = 0; attr_count < ARRAY_SIZE(attrs_oem); attr_count++){
		sysfs_remove_file(&ts->input_dev->dev.kobj, &attrs_oem[attr_count].attr);
	}
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
	tpd_power(ts,0);
	return 0;
}

static int synaptics_ts_suspend(struct device *dev)
{
	int ret,i;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		return -1;
	}
	TPD_DEBUG("%s enter\n", __func__);
	for (i = 0; i < ts->max_num; i++)
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);

#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif

#ifdef SUPPORT_GESTURE
	if( ts->gesture_enable ){
		atomic_set(&ts->is_stop,0);
		if (mutex_trylock(&ts->mutex)){
			touch_enable(ts);
			synaptics_enable_interrupt_for_gesture(ts, 1);
			mutex_unlock(&ts->mutex);
			TPD_ERR("enter gesture mode\n");
		}
	}
#endif
	TPD_DEBUG("%s normal end\n", __func__);
	return 0;
}

static void speedup_synaptics_resume(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = ts_g;

//#ifdef SUPPORT_SLEEP_POWEROFF
	TPD_DEBUG("%s enter!\n", __func__);
    if (ts->support_hw_poweroff){
        if(0 == ts->gesture_enable){
			if (ts->pinctrl) {
				ret = pinctrl_select_state(ts->pinctrl, ts->pinctrl_state_active);
			}
            ret = tpd_power(ts,1);
            if (ret < 0)
                TPD_ERR("%s power on err\n",__func__);
        }
    }
	TPD_DEBUG("%s end!\n", __func__);
//#endif
}

static int synaptics_ts_resume(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	int i;

	TPD_DEBUG("%s enter!\n", __func__);

	if(ts->loading_fw) {
		TPD_ERR("%s FW is updating break!\n",__func__);
		return -1;
	}

	if(ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("input_dev  registration is not complete\n");
		goto ERR_RESUME;
	}
	for (i = 0; i < ts->max_num; i++)
	{
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_sync(ts->input_dev);

    //touch_enable(ts);

	TPD_DEBUG("%s:normal end!\n", __func__);
ERR_RESUME:
	return 0;
}

static int synaptics_i2c_suspend(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	TPD_DEBUG("%s: is called\n", __func__);
	if (ts->gesture_enable == 1){
		/*enable gpio wake system through intterrupt*/
		enable_irq_wake(ts->irq);
	}
//#ifdef SUPPORT_SLEEP_POWEROFF
	if(ts->loading_fw) {
		TPD_ERR("FW is updating while suspending");
		return -1;
	}
    if(ts->support_hw_poweroff && (ts->gesture_enable == 0)){
	    ret = tpd_power(ts,0);
	    if (ret < 0)
	        TPD_ERR("%s power off err\n",__func__);
		if (ts->pinctrl){
			ret = pinctrl_select_state(ts->pinctrl,
					ts->pinctrl_state_suspend);
		}
	}
//#endif
	return 0;
}

static int synaptics_i2c_resume(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);

	TPD_DEBUG("%s is called\n", __func__);
    queue_delayed_work(synaptics_wq,&ts->speed_up_work, msecs_to_jiffies(5));
	if (ts->gesture_enable == 1){
		/*disable gpio wake system through intterrupt*/
		disable_irq_wake(ts->irq);
	}
	return 0;
}

static int synaptics_mode_change(int mode)
{
	int ret;
    int tmp_mode;
    tmp_mode = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_CTRL00);
    tmp_mode = tmp_mode & 0xF8;//bit0-bit2(mode)
    tmp_mode = tmp_mode | mode;
    if (ts_g->changer_connet)
        tmp_mode = tmp_mode | 0x20;//set bit6(change status)
    else
        tmp_mode = tmp_mode & 0xDF;//clear bit6(change status)
    TPD_DEBUG("%s: set TP to mode[0x%x]\n", __func__,tmp_mode);
	ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00, tmp_mode);
	if(ret<0)
		TPD_ERR("%s: set dose mode[0x%x] err!!\n", __func__,tmp_mode);
	return ret;
}
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	//int ret;

	struct synaptics_ts_data *ts = container_of(self, struct synaptics_ts_data, fb_notif);

	if(FB_EARLY_EVENT_BLANK != event && FB_EVENT_BLANK != event)
	return 0;
	if((evdata) && (evdata->data) && (ts) && (ts->client))
    {
		blank = evdata->data;
        TPD_DEBUG("%s blank[%d],event[0x%lx]\n", __func__,*blank,event);

		if((*blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_VSYNC_SUSPEND || *blank == FB_BLANK_NORMAL)\
            //&& (event == FB_EVENT_BLANK ))
            && (event == FB_EARLY_EVENT_BLANK ))
        {
            if (ts->is_suspended == 1)
            {
                TPD_DEBUG("%s going TP resume start\n", __func__);
                ts->is_suspended = 0;
				queue_delayed_work(get_base_report, &ts->base_work,msecs_to_jiffies(1));
				synaptics_ts_resume(&ts->client->dev);
                //atomic_set(&ts->is_stop,0);
                TPD_DEBUG("%s going TP resume end\n", __func__);
            }
		}else if( *blank == FB_BLANK_POWERDOWN && (event == FB_EARLY_EVENT_BLANK ))
		{
            if (ts->is_suspended == 0)
            {
				TPD_DEBUG("%s : going TP suspend start\n", __func__);
                ts->is_suspended = 1;
                atomic_set(&ts->is_stop,1);
				if(!(ts->gesture_enable)){
					touch_disable(ts);
				}
                synaptics_ts_suspend(&ts->client->dev);
				TPD_DEBUG("%s : going TP suspend end\n", __func__);
            }
		}
	}
	return 0;
}
#endif

static int __init tpd_driver_init(void)
{
	TPD_ERR("%s enter\n", __func__);
	if( i2c_add_driver(&tpd_i2c_driver)!= 0 ){
		TPD_ERR("unable to add i2c driver.\n");
		return -1;
	}
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	i2c_del_driver(&tpd_i2c_driver);
	if(synaptics_wq ){
		destroy_workqueue(synaptics_wq);
		synaptics_wq = NULL;
	}
	return;
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("Synaptics S3203 Touchscreen Driver");
MODULE_LICENSE("GPL");
