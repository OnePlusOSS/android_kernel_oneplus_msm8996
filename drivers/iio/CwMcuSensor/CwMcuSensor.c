#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h> /* BUS_I2C */
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>

#include <linux/CwMcuSensor.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/irq_work.h>


#define MAIN_VERSION  					"2.11_0602"
#define SENSOR_HUB_TAG                  "[SensorHub] "
#define DEBUG                           1
#if defined(DEBUG)
#define SH_FUN(f)                       printk(KERN_INFO SENSOR_HUB_TAG"%s\n", __FUNCTION__)
#define SH_ERR(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_LOG(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_DBG(fmt, args...)            printk(KERN_INFO SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#else
#define SH_FUN(f)                       printk(KERN_INFO SENSOR_HUB_TAG"%s\n", __FUNCTION__)
#define SH_ERR(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d ERROR: "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_LOG(fmt, args...)            printk(KERN_ERR  SENSOR_HUB_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SH_DBG(fmt, args...)
#endif

#define CWMCU_MUTEX
#define NON_WAKEUP_IRQ

/* for N5 NFC define */
#define GPIO_CW_MCU_WAKE_UP		85
#define GPIO_CW_MCU_BOOT		122
#define GPIO_CW_MCU_RESET		131
#define GPIO_CW_MCU_INTERRUPT	126
#ifdef  NON_WAKEUP_IRQ
#define GPIO_CW_MCU_NON_WAKEUP	73
#endif
#define QueueSystemInfoMsgSize  30
#define QueueWarningMsgSize          30

#define ACK                     0x79
#define NACK                    0x1F

#define DPS_MAX         (1 << (16 - 1))

/* Input poll interval in milliseconds */

#define CWMCU_POLL_INTERVAL 10
#define CWMCU_POLL_MAX      2000
#define FT_VTG_MIN_UV       1800000
#define FT_VTG_MAX_UV       1800000
#define FT_VTG_MIN_B_UV     285000
#define FT_VTG_MAX_B_UV     2850000
#define FT_VTG_MIN_C_UV     3300000
#define FT_VTG_MAX_C_UV     3300000
#define FT_I2C_VTG_MIN_UV   1800000
#define FT_I2C_VTG_MAX_UV   1800000
// neil test
#define FT_VTG_ALS_P_M_MIN_UV		3000000
#define FT_VTG_ALS_P_M_MAX_UV		3000000
// neil end

#define CWMCU_MAX_OUTPUT_ID     (CW_SNAP+1)
#define CWMCU_MAX_OUTPUT_BYTE       (CWMCU_MAX_OUTPUT_ID * 7)
#define CWMCU_MAX_DRIVER_OUTPUT_BYTE        256

/* turn on gpio interrupt if defined */
#define CWMCU_INTERRUPT

#define CWMCU_CALIB_SAVE_IN_FLASH
#define CWM_DATA_READ_USE_IRQ

#ifdef CWMCU_INTERRUPT
#define CWMCU_POLL_MIN      50
#endif

struct CWMCU_T {
    struct i2c_client *client;
    struct regulator *vdd;
    struct regulator *vcc_i2c;
    struct regulator *vdd_als_p_M;  // neil test
    struct regulator *vdd_sensors;
    struct regulator *vdd_ir;
    struct input_polled_dev *input_polled;
    struct input_dev *input;
    struct workqueue_struct *driver_wq;
    struct work_struct work;
#ifdef  NON_WAKEUP_IRQ
	struct work_struct non_wakeup_work;
#endif
    struct delayed_work delay_work;
    struct CWMCU_SENSORS_INFO sensors_info[HANDLE_ID_END][SENSORS_ID_END];
    SensorsInit_T   hw_info[DRIVER_ID_END];
    RegInformation *pReadRegInfo;
    RegInformation *pWriteRegInfo;
    u8 m_cReadRegCount;
    u8 m_cWriteRegCount;
    uint8_t initial_hw_config;
    // neiltsai, 20151211, add for probe_success read permission warning
    bool mcu_probe_success;
    // neiltsai end
    int mcu_mode;
    uint8_t kernel_status;

    /* enable & batch list */
    uint32_t enabled_list[HANDLE_ID_END];
    uint32_t interrupt_status;
    uint8_t calibratordata[DRIVER_ID_END][30];
    uint8_t calibratorUpdate[DRIVER_ID_END];
    uint8_t CalibratorStatus[DRIVER_ID_END];
    uint8_t SelfTestStatus[DRIVER_ID_END];

    /* Mcu site enable list*/

    /* power status */
volatile    uint32_t power_on_list;

    /* Calibrator status */
    int cal_cmd;
    int cal_type;
    int cal_id;

    /* gpio */
    int irq_gpio;
    int wakeup_gpio;
    int reset_gpio;
    int boot_gpio;
    int IRQ;

#ifdef  NON_WAKEUP_IRQ
    int non_wakeup_gpio;
	int non_wakeup;
#endif
    uint32_t debug_log;

    int cmd;
    uint32_t addr;
    int len;
    int mcu_slave_addr;
    int firmware_update_status;
    int cw_i2c_rw;  /* r = 0 , w = 1 */
    int cw_i2c_len;
    uint8_t cw_i2c_data[300];

    s32 iio_data[6];
    struct iio_dev *indio_dev;
    struct irq_work iio_irq_work;
    struct iio_trigger  *trig;
    atomic_t pseudo_irq_enable;

    struct class *sensor_class;
    struct device *sensor_dev;
    atomic_t delay;
    int supend_flag;

    int wq_polling_time;
#ifdef CWMCU_MUTEX
    struct mutex mutex_lock;
    struct mutex mutex_lock_i2c;
    struct mutex mutex_wakeup_gpio;
#endif

    unsigned char loge_buff[QueueSystemInfoMsgSize*2];
    unsigned char loge_buff_count;
    unsigned char logw_buff[QueueWarningMsgSize*2];
    unsigned char logw_buff_count;
    uint8_t logcat_cmd;

    /*Data for GPS and Health Info*/
    uint8_t GPSData[40];
    uint8_t HEALTHData[30];

	int mcu_status;
	int mcu_init_count;
};

static void cwmcu_reset_mcu(void);

//for geater than 32 bytes read

static void wakeup_pin_reset(void)
{
    gpio_set_value(GPIO_CW_MCU_WAKE_UP, 0);
    usleep_range(200, 300);
    gpio_set_value(GPIO_CW_MCU_WAKE_UP, 1);
}
static int CWMCU_Object_read(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
    int ret;

    struct i2c_msg msgs[] = {
        {
            .addr   = sensor->client->addr,
            .flags  = 0,
            .len    = 1,
            .buf    = &reg_addr,
        },
        {
            .addr   = sensor->client->addr,
            .flags  = I2C_M_RD,
            .len    = len,
            .buf    = data,
        },
    };

#ifdef CWMCU_MUTEX
mutex_lock(&sensor->mutex_wakeup_gpio);
#endif
    ret = i2c_transfer(sensor->client->adapter, msgs, 2);
#ifdef CWMCU_MUTEX
	mutex_unlock(&sensor->mutex_wakeup_gpio);
#endif
    return (ret == 2) ? len : ret;
}

static int CWMCU_reg_read(struct CWMCU_T *sensor, u8 reg_addr, u8 *data)
{
    RegInformation *pReadRegInfoInx = sensor->pReadRegInfo;
    int i;
    u8 cReadRegCount = sensor->m_cReadRegCount;
    int wRetVal = 0;

    if(pReadRegInfoInx == NULL || cReadRegCount == 0)
    {
        SH_ERR("pReadRegInfoInx==NULL or cReadRegCount==0\n");
        wRetVal = -1;
        return wRetVal;
    }

    for(i = 0; i < cReadRegCount; i++)
    {
        if(pReadRegInfoInx[i].cIndex == reg_addr)
            break;
    }
    if(i >= cReadRegCount)
    {
        wRetVal = -1;
    }
    else
    {
        if(pReadRegInfoInx[i].cObjLen != 0)
            wRetVal = CWMCU_Object_read(sensor, pReadRegInfoInx[i].cIndex, data, pReadRegInfoInx[i].cObjLen);
    }
    return wRetVal;
}
// Returns the number of read bytes on success

static int CWMCU_I2C_R(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
    int rty = 0;
    int err = 0;
#ifdef CWMCU_MUTEX
    mutex_lock(&sensor->mutex_lock_i2c);
#endif
retry:
    err = i2c_smbus_read_i2c_block_data(sensor->client, reg_addr, len, data);
    if(err <0){
        wakeup_pin_reset();
        if(rty<3)
        {
            rty++;
            usleep_range(300*(rty+1), 500*(rty+1));
            goto retry;
        }
        else
        {
            pr_err("%s:%s:(i2c read error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,err);
        }
    }
#ifdef CWMCU_MUTEX
    mutex_unlock(&sensor->mutex_lock_i2c);
#endif
    return err;
}

// write format    1.slave address  2.data[0]  3.data[1] 4.data[2]
static int CWMCU_I2C_W(struct CWMCU_T *sensor, u8 reg_addr, u8 *data, u8 len)
{
    int rty = 0;
    int err = 0;
#ifdef CWMCU_MUTEX
    mutex_lock(&sensor->mutex_lock_i2c);
#endif
retry:
    err = i2c_smbus_write_i2c_block_data(sensor->client, reg_addr, len, data);
    if(err <0){
        wakeup_pin_reset();
        if(rty<3)
        {
            rty++;
            usleep_range(300*(rty+1), 500*(rty+1));
            goto retry;
        }
        else
        {
            pr_err("%s:%s:(i2c write error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,err);
        }
    }
#ifdef CWMCU_MUTEX
    mutex_unlock(&sensor->mutex_lock_i2c);
#endif
    return err;
}

static int CWMCU_I2C_W_SERIAL(struct CWMCU_T *sensor,u8 *data, int len)
{
    int dummy;
    dummy = i2c_master_send(sensor->client, data, len);
    if (dummy < 0) {
        pr_err("%s:%s:(i2c write error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,dummy);
        return dummy;
    }
    return 0;
}

static int CWMCU_I2C_R_SERIAL(struct CWMCU_T *sensor,u8 *data, int len)
{
    int dummy;
    dummy = i2c_master_recv(sensor->client, data, len);
    if (dummy < 0) {
        pr_err("%s:%s:(i2c read error =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,dummy);
        return dummy;
    }
    return 0;
}

//neil test
static int get_bootloader_version(struct CWMCU_T *sensor)
{
	u8 data[10] = {0};

	data[0] = 0x01;
	data[1] = 0xfe;
	CWMCU_I2C_W_SERIAL(sensor, &data[0], 2);

	CWMCU_I2C_R_SERIAL(sensor, &data[2], 1);

	CWMCU_I2C_R_SERIAL(sensor, &data[3], 1);

	CWMCU_I2C_R_SERIAL(sensor, &data[4], 1);

	CWMCU_I2C_R_SERIAL(sensor, &data[5], 1);

	SH_LOG(" (5 0 0)%s:%s: %x %x %x %x\n \n",LOG_TAG_KERNEL ,__FUNCTION__, data[2], data[3], data[4], data[5]);
	return 0;
}
// neil end


static int cw_send_event(struct CWMCU_T *sensor, u8 handle, u8 id, u8 *data)
{
    u8 event[21];/* Sensor HAL uses fixed 21 bytes */

    if (id == CWMCU_NODATA)
        return FAIL;

    event[0] = handle;
    event[1] = id;
    memcpy(&event[2], data, 19);

    if (sensor->debug_log & (1<<D_IIO_DATA))
        printk("%s: id%d,data:%d,%d,%d\n",
          __func__,id,data[0],data[1],data[2]);
    if (sensor->indio_dev->active_scan_mask &&
        (!bitmap_empty(sensor->indio_dev->active_scan_mask,
               sensor->indio_dev->masklength))) {
		iio_push_to_buffers(sensor->indio_dev, event);
        return 0;
	}
	else if (NULL == sensor->indio_dev->active_scan_mask)
	{
		SH_ERR("active_scan_mask = NULL, event might be missing\n");
	}

    return -EIO;
}

static void power_pin_sw(struct CWMCU_T *sensor,SWITCH_POWER_ID id, int onoff)
{
    int value = 0;
#ifdef CWMCU_MUTEX
	mutex_lock(&sensor->mutex_wakeup_gpio);
#endif
    value = gpio_get_value(GPIO_CW_MCU_WAKE_UP);
    if (onoff) {
        sensor->power_on_list |= ((uint32_t)(1) << id);
        if(value ==0){
        gpio_set_value(GPIO_CW_MCU_WAKE_UP, onoff);
            usleep_range(100, 100);
        }
    } else {
        sensor->power_on_list &= ~(1 << id);
        if (sensor->power_on_list == 0 && value == 1) {
            gpio_set_value(GPIO_CW_MCU_WAKE_UP, onoff);
            usleep_range(100, 100);
        }
    }
#ifdef CWMCU_MUTEX
	mutex_unlock(&sensor->mutex_wakeup_gpio);
#endif
}

static void cwmcu_kernel_status(struct CWMCU_T *sensor,uint8_t status)
{
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return ;
    }
    sensor->kernel_status = status;
	if (CWMCU_I2C_W(sensor, RegMapW_SetHostStatus, &sensor->kernel_status, 1) < 0)
	{
		SH_LOG("Write SetHostStatus Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
	}
}

static int check_enable_list(struct CWMCU_T *sensor)
{
    int i = 0,j=0;
    int count = 0;
    int handle = 0;
	uint8_t data[10] = {0};
    int error_msg = 0;
    uint32_t enabled_list[HANDLE_ID_END] = {0};
    uint32_t enabled_list_temp = 0;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_LOG("mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
		return 0;
	}

	if( (CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8)) < 0)
	{
		SH_LOG("Read GetHostEnableList Fail [I2C] , func: %s , li: %d\n",__func__,__LINE__);
		return FAIL;
	}

    enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
    enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
    enabled_list[InternalHandle] = 0;

    if((enabled_list[NonWakeUpHandle] != sensor->enabled_list[NonWakeUpHandle])
            || (enabled_list[WakeUpHandle] != sensor->enabled_list[WakeUpHandle]))
    {
            SH_LOG("Enable List Check AP0:%d,MCU0:%d;AP1:%d,MCU1:%d\n",
        sensor->enabled_list[NonWakeUpHandle],enabled_list[NonWakeUpHandle],
        sensor->enabled_list[WakeUpHandle],enabled_list[WakeUpHandle]);

        for(j = 0; j < InternalHandle; j++)
        {
            handle = j;
            enabled_list_temp = sensor->enabled_list[handle]^enabled_list[handle];
            for (i = 0; i < SENSORS_ID_END; i++)
            {
                if (enabled_list_temp & (1<<i))
                {
                    data[0] = handle;
                    data[1] = i;
                    if (sensor->sensors_info[handle][i].en)
                    {
                        sensor->sensors_info[handle][i].rate = (sensor->sensors_info[handle][i].rate ==0)?200:sensor->sensors_info[handle][i].rate;
                        data[2] = sensor->sensors_info[handle][i].rate;
                        data[3] = (uint8_t)sensor->sensors_info[handle][i].timeout;
                        data[4] = (uint8_t)(sensor->sensors_info[handle][i].timeout >>8);
                        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
                    }
                    else
                    {
                        data[2] = 0;
                        data[3] = 0;
                        data[4] = 0;
                        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetDisable, data, 5);
                    }
                    if (error_msg < 0)
                        SH_ERR("I2c Write Fail;%d,%d\n", handle, i);
                    count++;
                    if(count >15)
                    {
                        count = 0;
                        msleep(20);
                    }
                }
            }
        }
    }
        else {
        }
    return 0;
}

static int cwmcu_read_buff(struct CWMCU_T *sensor , u8 handle)
{
    uint8_t count_reg;
    uint8_t data_reg;
    uint8_t data[24] = {0};
    uint16_t count = 0;
    int i = 0;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_LOG("-CWMCU- mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
		return -1;
	}

    if (handle == NonWakeUpHandle)
    {
        count_reg = RegMapR_StreamCount;
        data_reg = RegMapR_StreamEvent;
    }
    else if (handle == WakeUpHandle)
    {
        count_reg = RegMapR_BatchCount;
        data_reg = RegMapR_BatchEvent;
    }
    else
    {
        return FAIL;
    }

    if (CWMCU_I2C_R(sensor, count_reg, data, 2) >= 0)
    {
        count = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
    }
    else
    {
        SH_ERR("check count failed)\n");
        return FAIL;
    }
    if((data[0] ==0xFF) && (data[1] ==0xFF))
        return NO_ERROR;

    for (i = 0; i < count; i++)
    {
        if (CWMCU_I2C_R(sensor, data_reg, data, 9) >= 0)
        {
            cw_send_event(sensor,handle,data[0],&data[1]);
        }
		else
		{
			SH_LOG("Read stream buffer fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
		}
    }
        return NO_ERROR;
}

static int cwmcu_read_gesture(struct CWMCU_T *sensor )
{
    uint8_t data[24] = {0};
    uint8_t count = 0;
    int i = 0;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_LOG("mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
		return 0;
	}

    if (CWMCU_I2C_R(sensor, RegMapR_GestureCount, &count, 1) < 0)
    {
        SH_ERR("check count failed)\n");
            return FAIL;
    }
    if(count ==0xFF)
        return NO_ERROR;

    for (i = 0; i < count; i++)
    {
        if (CWMCU_I2C_R(sensor, RegMapR_GestureEvent, data, 9) >= 0)
        {
                cw_send_event(sensor,NonWakeUpHandle,data[0],&data[1]);
		}else{
			SH_LOG("Read GestureEvent fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
        }
    }
    return NO_ERROR;
}

#define QueueSystemInfoMsgBuffSize      QueueSystemInfoMsgSize*5

static void parser_mcu_info(char *data)
{
    static unsigned char loge_bufftemp[QueueSystemInfoMsgBuffSize];
    static int buff_counttemp = 0;
    int i;

    for (i = 0; i < QueueSystemInfoMsgSize; i++)
    {
        loge_bufftemp[buff_counttemp] = (unsigned char)data[i];
        buff_counttemp++;
        if (data[i] == '\n' || (buff_counttemp >=QueueSystemInfoMsgBuffSize))
        {
            SH_LOG("%s:%s", "MSG",loge_bufftemp);
            memset(loge_bufftemp,0x00,QueueSystemInfoMsgBuffSize);
            buff_counttemp = 0;
        }
    }
}

static void read_mcu_info(struct CWMCU_T *sensor)
{
    uint8_t data[40];
    uint16_t count = 0;
    int i = 0;

    SH_FUN();
	if (sensor->mcu_mode == CW_BOOT) {
		SH_LOG("-CWMCU- mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
		return ;
	}

    if (CWMCU_I2C_R(sensor, RegMapR_SystemInfoMsgCount, data, 1) >= 0)
    {
        count = (uint16_t)data[0];
    }
    else
    {
		SH_LOG("Read SystemInfoMsgCount Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
		return ;
    }
	if(count ==0xFF){
		SH_LOG("Count = 0xFF , func: %s , li: %d\n",__func__,__LINE__);
        return ;
	}

    for (i = 0; i < count; i++)
    {
        if (CWMCU_I2C_R(sensor, RegMapR_SystemInfoMsgEvent, data, 30) >= 0)
        {
            parser_mcu_info(data);
        }else{
            SH_LOG("Read SystemInfoMsgEvent Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
        }
    }

}




/*==========sysfs node=====================*/
static int cwmcu_find_mindelay(struct CWMCU_T *sensor, int handle)
{
    int i;
    int min_delay = 30;
    for (i = 0; i < SENSORS_ID_END; i++)
    {
        if(sensor->sensors_info[handle][i].en
                && (sensor->sensors_info[handle][i].rate >= 10)
                && (sensor->sensors_info[handle][i].rate < min_delay)
          )
        {
            min_delay = sensor->sensors_info[handle][i].rate;
        }
    }
    min_delay = (min_delay<=10)? 10: min_delay;
    return min_delay;
}

static ssize_t active_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int enabled = 0;
    int id = 0;
    int handle = 0;
    int error_msg = 0;
    uint8_t data[10];

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode = CW_BOOT\n");
        return count;
    }

    sscanf(buf, "%d %d %d\n", &handle, &id, &enabled);

    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);

    sensor->sensors_info[handle][id].en = enabled;
    data[0] = handle;
    data[1] = id;
    if (enabled)
    {
        sensor->enabled_list[handle] |= 1<<id;
        data[2] = (sensor->sensors_info[handle][id].rate ==0)?200:sensor->sensors_info[handle][id].rate;
        data[3] = (uint8_t)sensor->sensors_info[handle][id].timeout;
        data[4] = (uint8_t)(sensor->sensors_info[handle][id].timeout >>8);
        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
    }
    else
    {
        sensor->enabled_list[handle] &= ~(1<<id);
        sensor->sensors_info[handle][id].rate = 0;
        sensor->sensors_info[handle][id].timeout = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        error_msg = CWMCU_I2C_W(sensor, RegMapW_SetDisable, data, 5);
    }

    if (error_msg < 0)
		SH_LOG("Write SetEn/Disable Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);

    msleep(5);
    check_enable_list(sensor);
    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);

    if (NonWakeUpHandle == handle)
    {
        sensor->wq_polling_time = cwmcu_find_mindelay(sensor,NonWakeUpHandle);
        if (sensor->wq_polling_time != atomic_read(&sensor->delay))
		{
						if (sensor->debug_log & (1<<D_EN))
							SH_LOG("sensor->wq_polling_time != atomic_read(&sensor->delay)\n");

        if(sensor->enabled_list[NonWakeUpHandle])
        {
								if (sensor->debug_log & (1<<D_EN))
                	SH_LOG("sensor->enabled_list[NonWakeUpHandle] == 1\n");
                atomic_set(&sensor->delay, sensor->wq_polling_time);

        }
        	else
			{
            atomic_set(&sensor->delay, CWMCU_POLL_MAX);
        }
    }
    }
    if (sensor->debug_log & (1<<D_EN))
        SH_LOG("%d,%d,%d,%d,%d\n", handle, id, enabled, (int)sensor->sensors_info[handle][id].rate, (int)sensor->sensors_info[handle][id].timeout);
    return count;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[10] = {0};
    uint32_t enabled_list[2] ={0, 0};
    int err = 0;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode = CW_BOOT\n");
        return sprintf(buf, "In Boot Mode\n");
    }

    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8);
    power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);
    if (err >= 0)
    {
        enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
        enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
        if (sensor->debug_log & (1<<D_EN))
            SH_LOG("MCU En Status:%d,%d\n", enabled_list[NonWakeUpHandle], enabled_list[WakeUpHandle]);

        return sprintf(buf, "%d %d %d %d\n", sensor->enabled_list[NonWakeUpHandle],
                sensor->enabled_list[WakeUpHandle], enabled_list[NonWakeUpHandle], enabled_list[WakeUpHandle]);
    }
    else
    {
		SH_LOG("Read GetHostEnableList Fail [I2C], func: %s , ln: %d\n",__func__,__LINE__);
		return sprintf(buf, "read RegMapR_GetHostEnableList failed!\n");
    }
}

static ssize_t batch_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint32_t id = 0;
    uint32_t handle = 0;
    uint32_t mode = -1;
    uint32_t rate = 0;
    uint32_t timeout = 0;
    //int error_msg = 0;
    uint8_t data[5];
    int err = 0;

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode = CW_BOOT\n");
        return count;
    }

    sscanf(buf, "%d %d %d %d %d\n", &handle, &id, &mode, &rate, &timeout);
    if (0 == mode)
    {
				if (sensor->debug_log & (1<<D_EN))
					SH_LOG("%d %d %d %d %d\n", handle, id, mode, rate, timeout);
        sensor->sensors_info[handle][id].rate = (uint8_t)rate;
        sensor->sensors_info[handle][id].timeout = (uint16_t)timeout;
        data[0] = handle;
        data[1] = id;
        data[2] = sensor->sensors_info[handle][id].rate;
        data[3] = (uint8_t)sensor->sensors_info[handle][id].timeout;
        data[4] = (uint8_t)(sensor->sensors_info[handle][id].timeout >>8);
        if(sensor->sensors_info[handle][id].en)
        {
            power_pin_sw(sensor,SWITCH_POWER_BATCH, 1);
            err = CWMCU_I2C_W(sensor, RegMapW_SetEnable, data, 5);
        	power_pin_sw(sensor,SWITCH_POWER_BATCH, 0);
			if (err < 0)
            {
                SH_ERR("Write Fail:id:%d, mode:%d, rate:%d, timeout:%d)\n", id, mode, rate, timeout);
        }
    }

    if (sensor->debug_log & (1<<D_EN))
			SH_LOG("id:%d, mode:%d, rate:%d, timeout:%d\n", id, mode, rate, timeout);
    }

    return count;
}

static ssize_t flush_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int id = 0;
    //int error_msg = 0;
    int handle = 0;
    uint8_t data[2];
    int err = 0;

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return count;
    }

    sscanf(buf, "%d %d\n", &handle, &id);
		if (sensor->debug_log & (1<<D_EN))
    SH_LOG("flush:id:%d\n", id);
    data[0] = (uint8_t)handle;
    data[1] = (uint8_t)id;
    power_pin_sw(sensor,SWITCH_POWER_FLUSH, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_SetFlush, data, 2);
    power_pin_sw(sensor,SWITCH_POWER_FLUSH, 0);
    if (err < 0)
	{
		SH_LOG("Write SetFlush Fail [I2C], func: %s , ln: %d\n",__func__,__LINE__);

	}

    return count;
}

static int CWMCU_Write_Mcu_Memory(struct CWMCU_T *sensor,const char *buf)
{
    uint8_t WriteMemoryCommand[2];
    uint8_t data[300];
    uint8_t received[10];
    uint8_t XOR = 0;
    uint16_t i = 0;

    WriteMemoryCommand[0] = 0x31;
    WriteMemoryCommand[1] = 0xCE;
    if (CWMCU_I2C_W_SERIAL(sensor,(uint8_t *)WriteMemoryCommand, 2) < 0)
    {
        return -EAGAIN;
    }

    if (CWMCU_I2C_R_SERIAL(sensor,(uint8_t *)received, 1) < 0)
    {
        return -EAGAIN;
        }

    if (received[0] != ACK)
    {
        return -EAGAIN;
    }

    data[0] = (uint8_t) (sensor->addr >> 24);
    data[1] = (uint8_t) (sensor->addr >> 16);
    data[2] = (uint8_t) (sensor->addr >> 8);
    data[3] = (uint8_t) sensor->addr;
    data[4] = data[0] ^ data[1] ^ data[2] ^ data[3];
    if (CWMCU_I2C_W_SERIAL(sensor,(uint8_t *)data, 5) < 0)
    {
        return -EAGAIN;
        }

    if (CWMCU_I2C_R_SERIAL(sensor,(uint8_t *)received, 1) < 0)
    {
        return -EAGAIN;
        }

    if (received[0] != ACK)
    {
        return -EAGAIN;
        }

    data[0] = sensor->len - 1;
    XOR = sensor->len - 1;
    for (i = 0; i < sensor->len; i++)
    {
        data[i+1] = buf[i];
        XOR ^= buf[i];
    }
    data[sensor->len+1] = XOR;

    if (CWMCU_I2C_W_SERIAL(sensor,(uint8_t *)data, (sensor->len + 2)) < 0)
    {
        return -EAGAIN;
        }

    return 0;
}

static int set_calib_cmd(struct CWMCU_T *sensor, uint8_t cmd, uint8_t id, uint8_t type)
{
    uint8_t data[4];
    int err;

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return -1;
    }

    data[0] = cmd;
    data[1] = id;
    data[2] = type;
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorCmd, data, 4);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
	if(err)
		SH_LOG("Write CalibratorCmd Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);

    return err;
}

/*
    sensors default calibrator flow:
        sensors_calib_start(sensors, id);
        do{
            sensors_calib_status(sensors, id,&status);
        }while(status ==CALIB_STATUS_INPROCESS)
        if(status ==CALIB_STATUS_PASS)
            sensors_calib_data_read(sensors, id,data);
        save data
*/
static int sensors_calib_start(struct CWMCU_T *sensor, uint8_t id)
{
    int err;
    err = set_calib_cmd(sensor, CALIB_EN, id, CALIB_TYPE_DEFAULT);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    err = set_calib_cmd(sensor, CALIB_CHECK_STATUS, id, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    return err;
}

static int sensors_calib_status(struct CWMCU_T *sensor, uint8_t id, int *status)
{
    int err;
    uint8_t i2c_data[31] = {0};

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if (err < 0)
    {
		SH_LOG("Read CalibratorData Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
        return I2C_FAIL;
    }
    status[0] = (int)((int8_t)i2c_data[0]);

    return NO_ERROR;
}

static int sensors_calib_data_read(struct CWMCU_T *sensor, uint8_t id, uint8_t *data)
{
    int err;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_READ, id, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if (err < 0)
    {
		SH_LOG("Read CalibratorData Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);

        return err;
    }

    return NO_ERROR;
}

static int sensors_calib_data_write(struct CWMCU_T *sensor, uint8_t id, uint8_t *data)
{
    int err;

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_WRITE, id, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if (err < 0)
    {
		SH_LOG("Write CalibratorData fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
        return err;
    }

    return NO_ERROR;
}

static int proximity_calib_en(struct CWMCU_T *sensor, int en)
{
    int err;

    SH_LOG("en=%d\n", en);
    if(en)
        err = set_calib_cmd(sensor, CALIB_EN, PROXIMITY, CALIB_TYPE_SENSORS_ENABLE);
    else
        err = set_calib_cmd(sensor, CALIB_EN, PROXIMITY, CALIB_TYPE_SENSORS_DISABLE);

    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return err;
    }

    return NO_ERROR;
}

/*
    FUN: proximity_calib_data
    |data[0]: Proximity sensors raw data
    |data[1] is Hight threshold to check sensors is near
    |data[2] is low threshold to check sensors is far
*/
static int proximity_calib_data(struct CWMCU_T *sensor, int *data)
{
    int err;
    uint8_t i2c_data[31] = {0};
    int *ptr;
    ptr = (int *)i2c_data;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_READ, PROXIMITY, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("set_calib_cmd Fail!\n");
        return err;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, i2c_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if (err < 0)
    {
		SH_LOG("Read CalibratorData Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
        return I2C_FAIL;
    }

    data[0] = ptr[3];
    data[1] = ptr[1];
    data[2] = ptr[2];

    SH_LOG("raw:%d, close:%d, far:%d\n", data[0], data[1], data[2]);
    return NO_ERROR;
}

static int proximity_set_threshold(struct CWMCU_T *sensor, int near_th, int far_th)
{
    int err;
    uint8_t i2c_data[31] = {0};
    int *ptr;
    ptr = (int *)i2c_data;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT\n");
        return FAIL;
    }

    err = set_calib_cmd(sensor, CALIB_DATA_WRITE, PROXIMITY, CALIB_TYPE_NON);
    if (err < 0)
    {
        SH_ERR("set_calib_cmd Fail!\n");
        return err;
    }

    ptr[0] = 0;
    ptr[1] = near_th;
    ptr[2] = far_th;

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, i2c_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if (err < 0)
    {
		SH_LOG("Write CalibratorData fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
        return -1;
    }

    SH_LOG("close:%d, far:%d\n", near_th, far_th);
    return NO_ERROR;
}

static ssize_t set_firmware_update_cmd(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    u8 data[300] = {0};
    int i = 0;
    int status = 0;
    int proximity_data[3] = {0};
    u8 cReadRegCount = sensor->m_cReadRegCount;
    u8 cWriteRegCount = sensor->m_cWriteRegCount;
    RegInformation *pReadRegInfoInx = sensor->pReadRegInfo;
    RegInformation *pWriteRegInfoInx = sensor->pWriteRegInfo;

    sscanf(buf, "%d %d %d\n", &sensor->cmd, &sensor->addr, &sensor->len);

    SH_LOG("cmd=%d addr=%d len=%d\n", sensor->cmd, sensor->addr, sensor->len);

    power_pin_sw(sensor,SWITCH_POWER_FIRMWARE_COMMAND, 1);

    switch (sensor->cmd) {
    case CHANGE_TO_BOOTLOADER_MODE:
			SH_LOG(" (1 0 0)%s:%s:(CHANGE_TO_BOOTLOADER_MODE)\n",LOG_TAG_KERNEL ,__FUNCTION__);

			sensor->firmware_update_status = 0;
			sensor->mcu_mode = CW_BOOT;
			sensor->mcu_slave_addr = sensor->client->addr;
			sensor->client->addr = 0x39;

			gpio_direction_output(GPIO_CW_MCU_BOOT, 1);
            gpio_direction_output(GPIO_CW_MCU_RESET, 1);
			msleep(50);
            gpio_set_value(GPIO_CW_MCU_RESET, 0);
			msleep(50);
            gpio_set_value(GPIO_CW_MCU_RESET, 1);
			msleep(100);

			gpio_direction_input(GPIO_CW_MCU_INTERRUPT);
			msleep(100);



            break;

    case CHANGE_TO_NORMAL_MODE:
            sensor->firmware_update_status = 0;
            sensor->client->addr = 0x3a;
            sensor->mcu_mode = CW_NORMAL;
            sensor->firmware_update_status = 2;

            gpio_set_value(GPIO_CW_MCU_BOOT, 0);
            gpio_set_value(GPIO_CW_MCU_RESET, 1);
            msleep(50);
            gpio_set_value(GPIO_CW_MCU_RESET, 0);
            msleep(50);
            gpio_set_value(GPIO_CW_MCU_RESET, 1);
            msleep(100);
            gpio_direction_input(GPIO_CW_MCU_RESET);
			SH_LOG(" (5 0 0)%s:%s:(CHANGE_TO_NORMAL_MODE)\n",LOG_TAG_KERNEL ,__FUNCTION__);
            break;

    case CHECK_FIRMWAVE_VERSION:
            if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, data, 4) >= 0) {
                printk("%s:%s:(CHECK_FIRMWAVE_VERSION:%u,%u,%u,%u)\n",LOG_TAG_KERNEL ,__FUNCTION__, data[0],data[1],data[2],data[3]);
            }
            break;

    case GET_FWPROJECT_ID:
            if (CWMCU_reg_read(sensor, RegMapR_GetProjectID, data) >= 0) {
                printk("%s:%s:(PROJECT ID:%s) \n",LOG_TAG_KERNEL ,__FUNCTION__, data);
            }
            break;

    case SHOW_THE_REG_INFO:
            if (pWriteRegInfoInx != NULL && pReadRegInfoInx != NULL) {
               printk("(number of read reg:%u number of write reg:%u) \n",cReadRegCount ,cWriteRegCount);
               printk("--------------------READ REGISTER INFORMATION------------------------\n");
               for(i =0; i < cReadRegCount; i++)
               {
                   printk("(read tag number:%u and lengh:%u) \n",pReadRegInfoInx->cIndex,pReadRegInfoInx->cObjLen);
                   pReadRegInfoInx++;
               }

               printk("--------------------WRITE REGISTER INFORMATION-----------------------\n");
               for(i =0; i < cWriteRegCount; i++)
               {
                   printk("(write tag number:%u and lengh:%u) \n",pWriteRegInfoInx->cIndex ,pWriteRegInfoInx->cObjLen);
                   pWriteRegInfoInx++;
               }
            }
            break;

    case SET_DEBUG_LOG:
                    if(sensor->len)
                        sensor->debug_log  |= (1<< sensor->addr);
                    else
                        sensor->debug_log  &= ~(1<< sensor->addr);
            printk("%s:%s:(SET_DEBUG_LOG%u)\n",LOG_TAG_KERNEL ,__FUNCTION__,sensor->debug_log);
            break;
    case SET_SYSTEM_COMMAND:
            data[0] = sensor->addr;
            data[1] = sensor->len;
			if (CWMCU_I2C_W(sensor, RegMapW_SetSystemCommand, data, 2) < 0)
			{
				SH_LOG(" Write SetSystemCommand Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
			}else{
            printk("%s:%s:(SET_SYSTEM_COMMAND)\n",LOG_TAG_KERNEL ,__FUNCTION__);
			}
            break;
    case GET_SYSTEM_TIMESTAMP:
            if (CWMCU_I2C_R(sensor, RegMapR_GetSystemTimestamp, data, 4) >= 0) {
                printk("%s:%s:(Timestamp:%u)\n",LOG_TAG_KERNEL ,__FUNCTION__,(((uint32_t)data[3])<<24) |(((uint32_t)data[2])<<16) |(((uint32_t)data[1])<<8) | ((uint32_t)data[0]));
			}else{
				SH_LOG(" Read GetSystemTimestamp Fail [I2C], func: %s , li: %d \n",__func__,__LINE__);
            }
            break;
    case SET_HW_INITIAL_CONFIG_FLAG:
            sensor->initial_hw_config = sensor->addr;
            break;
    case SET_SENSORS_POSITION:
            data[0] = sensor->addr;
            data[1] = sensor->len;
			if (CWMCU_I2C_W(sensor, RegMapW_SetSensorAxisReference, data, 2) < 0 )
			{
				SH_LOG(" Write SetSensorAxisReference Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
			}
            break;
    case CMD_CALIBRATOR_START:
        sensors_calib_start(sensor,sensor->addr);
        break;
    case CMD_CALIBRATOR_STATUS:
        sensors_calib_status(sensor,sensor->addr,&status);
        break;
    case CMD_CALIBRATOR_READ:
        sensors_calib_data_read(sensor,sensor->addr,sensor->cw_i2c_data);
        break;
    case CMD_CALIBRATOR_WRITE:
        sensors_calib_data_write(sensor,sensor->addr,sensor->cw_i2c_data);
        break;
    case CMD_PROXIMITY_EN:
        proximity_calib_en(sensor,sensor->addr);
        break;
    case CMD_PROXIMITY_DATA:
        proximity_calib_data(sensor,proximity_data);
            printk("%s:%s:(Proximity data:%d,%d,%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,proximity_data[0],proximity_data[1],proximity_data[2]);
        break;
    case CMD_PROXIMITY_TH:
        proximity_set_threshold(sensor,sensor->addr,sensor->len);
        printk("%s:%s:(Proximity th:%d,%d)\n",LOG_TAG_KERNEL ,__FUNCTION__,sensor->addr,sensor->len);
            break;
		case MCU_RESET: //20 0 0 mcu_reset
			sensor->firmware_update_status = 0;
            cwmcu_reset_mcu();
			break;
    // neil test
    case CMD_MCU_TEST:
        get_bootloader_version(sensor);
        break;
    // neil end

    }
    power_pin_sw(sensor,SWITCH_POWER_FIRMWARE_COMMAND, 0);
    return count;
}

static ssize_t set_firmware_update_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    SH_LOG("%s\n", buf);
	sensor->firmware_update_status = 1;
	sensor->firmware_update_status = CWMCU_Write_Mcu_Memory(sensor,buf);
    return count;
}

static ssize_t get_firmware_update_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
	SH_LOG("firmware_update_status = %d\n", sensor->firmware_update_status);
	return sprintf(buf, "%d\n", sensor->firmware_update_status);
}

static ssize_t set_firmware_update_i2(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int intsize = sizeof(int);

    SH_FUN();
    memcpy(&sensor->cw_i2c_rw, buf, intsize);
    memcpy(&sensor->cw_i2c_len, &buf[4], intsize);
    memcpy(sensor->cw_i2c_data, &buf[8], sensor->cw_i2c_len);
    return count;
}

static ssize_t get_firmware_update_i2(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int status = 0;

    SH_FUN();
    if (sensor->cw_i2c_rw)
    {
        if (CWMCU_I2C_W_SERIAL(sensor,sensor->cw_i2c_data, sensor->cw_i2c_len) < 0)
        {
            status = -1;
        }
        memcpy(buf, &status, sizeof(int));
        return 4;
    }
    else
    {
        if (CWMCU_I2C_R_SERIAL(sensor,sensor->cw_i2c_data, sensor->cw_i2c_len) < 0)
        {
            status = -1;
            memcpy(buf, &status, sizeof(int));
            return 4;
        }
        memcpy(buf, &status, sizeof(int));
        memcpy(&buf[4], sensor->cw_i2c_data, sensor->cw_i2c_len);
        return 4+sensor->cw_i2c_len;
    }
    return  0;
}

static ssize_t mcu_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", sensor->mcu_mode);
}

static ssize_t mcu_model_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int mode = 0;
    sscanf(buf, "%d\n", &mode);
    sensor->mcu_mode = mode;
    return count;
}

static ssize_t set_calibrator_cmd(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    int err;

    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return count;
    }

    sscanf(buf, "%d %d %d\n", &sensor->cal_cmd, &sensor->cal_id, &sensor->cal_type);
    err = set_calib_cmd(sensor, sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
    if (sensor->debug_log & (1<<D_CALIB))
        SH_LOG("cmd:%d,id:%d,type:%d\n", sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
    if (err < 0)
        SH_ERR("I2c Write Fail!\n");

    return count;
}

static ssize_t get_calibrator_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    return sprintf(buf, "Cmd:%d,Id:%d,Type:%d\n", sensor->cal_cmd, sensor->cal_id, sensor->cal_type);
}

static ssize_t get_calibrator_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t Cal_data[31] = {0};
    int err;

    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_CalibratorData, Cal_data, 30);
	if (err < 0 )
	{
		SH_LOG(" Read CalibratorData Fail [I2C], func: %s , li: %d \n",__func__,__LINE__);
	}

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    if(sensor->cal_cmd == CALIB_DATA_READ && err >=0){
        memcpy(sensor->calibratordata[sensor->cal_id],Cal_data,30);
        sensor->calibratorUpdate[sensor->cal_id] = 1;
    }
    return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
        err,
        Cal_data[0], Cal_data[1], Cal_data[2],
        Cal_data[3], Cal_data[4], Cal_data[5], Cal_data[6], Cal_data[7], Cal_data[8], Cal_data[9], Cal_data[10], Cal_data[11], Cal_data[12],
        Cal_data[13], Cal_data[14], Cal_data[15], Cal_data[16], Cal_data[17], Cal_data[18], Cal_data[19], Cal_data[20], Cal_data[21], Cal_data[22],
        Cal_data[23], Cal_data[24], Cal_data[25], Cal_data[26], Cal_data[27], Cal_data[28], Cal_data[29]);
}

static ssize_t set_calibrator_data(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[30];
    int temp[33] = {0};
    int i,err;

    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return count;
    }

    sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
        &temp[0], &temp[1], &temp[2],
        &temp[3], &temp[4], &temp[5], &temp[6], &temp[7], &temp[8], &temp[9], &temp[10], &temp[11], &temp[12],
        &temp[13], &temp[14], &temp[15], &temp[16], &temp[17], &temp[18], &temp[19], &temp[20], &temp[21], &temp[22],
        &temp[23], &temp[24], &temp[25], &temp[26], &temp[27], &temp[28], &temp[29]);

    for (i = 0 ; i < 30; i++)
        data[i] = (uint8_t)temp[i];

    if(sensor->cal_cmd == CALIB_DATA_WRITE){
        memcpy(sensor->calibratordata[sensor->cal_id],data,30);
        sensor->calibratorUpdate[sensor->cal_id] = 1;
    }

    power_pin_sw(sensor,SWITCH_POWER_CALIB, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_CalibratorData, data, 30);
    if (err < 0)
		SH_LOG(" Write CalibratorData Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);


    power_pin_sw(sensor,SWITCH_POWER_CALIB, 0);
    return count;
}

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[4];
    int16_t version = -1;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
		return FAIL;
    }

    power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
    if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, data, 4) >= 0)
    {
        version = (int16_t)( ((uint16_t)data[1])<<8 | (uint16_t)data[0]);
		SH_LOG(" Check FW Version[3-0]: (M:%u,D:%u,V:%u,SV:%u)\n", data[3], data[2], data[1], data[0]);
    }
    else
    {
		SH_LOG("Read Get FW Version Fail [I2C], func: %s ,ln: %d\n",__func__,__LINE__);
		data[0] = 1;
		data[1] = 0;
    }
    power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);
    return sprintf(buf, "%d\n", version);
}

static ssize_t library_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[4] = {0, 0, 0, 0};

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }

    power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
    if (CWMCU_I2C_R(sensor, RegMapR_GetLibVersion, data, 4) >= 0)
    {
        SH_LOG("check_library_version:%u,%u,%u,%u\n", data[3], data[2], data[1], data[0]);
    }
    else
    {
        SH_ERR("i2c read fail)\n");
    }
    power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);
    return sprintf(buf, "%d %d %d %d\n", data[3], data[2], data[1], data[0]);
}

static ssize_t timestamp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[4];
    uint32_t *ptr;
    int err;
    ptr = (uint32_t *)data;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }
    power_pin_sw(sensor,SWITCH_POWER_TIME, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetSystemTimestamp, data, 4);
    power_pin_sw(sensor,SWITCH_POWER_TIME, 0);
    return sprintf(buf, "%d %u\n", err, ptr[0]);
}


static ssize_t set_sys_cmd(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[8];
    int temp[8] = {0};
    int i,err;

    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return count;
    }

    sscanf(buf, "%d %d %d %d %d %d %d %d\n",
        &temp[0], &temp[1], &temp[2],
        &temp[3], &temp[4], &temp[5], &temp[6], &temp[7]);

    for (i = 0 ; i < 8; i++)
        data[i] = (uint8_t)temp[i];

    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_SetSystemCommand, data, 8);
    if (err < 0)
        SH_ERR("I2c Write Fail!\n");
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return count;
}

static ssize_t sensorhub_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct CWMCU_T *sensor = dev_get_drvdata(dev);
	uint8_t fw_data[4];
        int16_t version = -1;
	uint8_t data[10] = {0};
	uint32_t mcu_enabled_list[2] ={0};
	int len = 0;
	int i = 0;

	if (sensor->mcu_mode == CW_BOOT)
		return 0;

	len += sprintf(buf+len , "=================================================================\n");
	/* version show*/
	power_pin_sw(sensor,SWITCH_POWER_VERSION, 1);
	if (CWMCU_I2C_R(sensor, RegMapR_GetFWVersion, fw_data, 4) >= 0) {
		version = (int16_t)( ((uint16_t)fw_data[1])<<8 | (uint16_t)fw_data[0]);
		printk("%s:%s:(CHECK_FIRMWAVE_VERSION : M:%u,D:%u,V:%u,SV:%u)\n",LOG_TAG_HAL ,__FUNCTION__, fw_data[3], fw_data[2], fw_data[1], fw_data[0]);
	}else{
		SH_LOG("Read GetFWVersion Fail [I2C], func: %s ,li: %d\n",__func__,__LINE__);
		data[0] = 1;
		data[1] = 0;
	}
	power_pin_sw(sensor,SWITCH_POWER_VERSION, 0);

	len += sprintf(buf+len , "Kernel version : %s \nFirmware version : %d.%d\n",MAIN_VERSION, fw_data[1],fw_data[0]);


	/* flag show */

	len += sprintf(buf+len , "mcu_init_count: %d\nmcu_status: %d\nmcu_mode: %d\n", sensor->mcu_init_count,sensor->mcu_status,sensor->mcu_mode);

	/* active show (sensor_hub info) */
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 1);
	if (CWMCU_I2C_R(sensor, RegMapR_GetHostEnableList, data, 8) >= 0)
	{
		mcu_enabled_list[NonWakeUpHandle] = (uint32_t)data[3]<<24 |(uint32_t)data[2]<<16 |(uint32_t)data[1]<<8 |(uint32_t)data[0];
		mcu_enabled_list[WakeUpHandle] = (uint32_t)data[7]<<24 |(uint32_t)data[6]<<16 |(uint32_t)data[5]<<8 |(uint32_t)data[4];
		if (sensor->debug_log & (1<<D_EN))
			SH_LOG("MCU En Status:%d,%d\n", mcu_enabled_list[NonWakeUpHandle], mcu_enabled_list[WakeUpHandle]);
	}
	else
	{
		SH_LOG(" Read GetHostEnableList Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
	}
	power_pin_sw(sensor,SWITCH_POWER_ENABLE, 0);

	len += sprintf(buf+len , "mcu(Nonwp , wp): %d , %d | AP(Nonwp , wp): %d , %d\n",mcu_enabled_list[NonWakeUpHandle],
                        mcu_enabled_list[WakeUpHandle], sensor->enabled_list[NonWakeUpHandle],sensor->enabled_list[WakeUpHandle]);

	/* sensor status include : nonwakeup_sensor*/

	len += sprintf(buf+len , "=================================================================\n");
	for(i = 0; i < SENSORS_ID_END; i++){
		len += sprintf(buf+len ,"id: %4d , en: %2d , rate: %4d , timeout: %6d , power_mode: %2d\n",i,sensor->sensors_info[NonWakeUpHandle][i].en , sensor->sensors_info[NonWakeUpHandle][i].rate,
                               sensor->sensors_info[NonWakeUpHandle][i].timeout , sensor->sensors_info[NonWakeUpHandle][i].mode);
	}
	return len;
}

static void read_calib_info(struct CWMCU_T *sensor)
{
    uint8_t data[24] = {0};
    int status = 0;
    uint16_t *ptr;
    ptr = (uint16_t *)data;

	if (sensor->mcu_mode == CW_BOOT) {
		SH_LOG("mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
                return;
	}

    if(set_calib_cmd(sensor, CALIB_CHECK_STATUS, sensor->cal_id, sensor->cal_type)){
        SH_ERR("I2c Write Fail!\n");
        return;
    }

    if(sensors_calib_status(sensor,  sensor->cal_id, &status) >=0){
        SH_LOG("Calib id:%d:status:%d\n", sensor->cal_id , status);
        if(status ==CALIB_STATUS_PASS){
            ptr[0] =  (uint16_t)sensor->cal_id;
            cw_send_event(sensor,NonWakeUpHandle,CALIBRATOR_UPDATE,data);
        }
    }

    return ;

}
static void read_error_code(struct CWMCU_T *sensor)
{
    uint8_t data[4] = {0};
    int8_t *ptr;
    int err;
    ptr = (int8_t *)data;
    err = CWMCU_I2C_R(sensor, RegMapR_ErrorCode, data, 4);
    if (err < 0)
        SH_ERR("I2c Write Fail!\n");
    if(ptr[0] ==ERR_TASK_BLOCK){
        SH_LOG("ERR_TASK_BLOCK\n");
    }
    SH_LOG("%s:%d,%d,%d,%d)\n",__FUNCTION__ , ptr[0], ptr[1], ptr[2], ptr[3]);

}
static ssize_t get_raw_data0(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetAccelerationRawData, data, 6);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("read RegMapR_GetAccelerationRawData failed!\n");
        return FAIL;
    }
    SH_LOG("RawData0:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}
static ssize_t get_raw_data1(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetMagneticRawData, data, 6);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("read RegMapR_GetMagneticRawData failed!\n");
        return err;
    }
    SH_LOG("RawData1:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}

static ssize_t get_raw_data2(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetGyroRawData, data, 6);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("read RegMapR_GetGyroRawData failed!\n");
        return err;
    }
    SH_LOG("RawData2:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}

static ssize_t get_raw_data3(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }

    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetLightRawData, data, 6);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("read RegMapR_GetLightRawData failed!\n");
        return err;
    }
    SH_LOG("RawData3:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}

static ssize_t get_raw_data4(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[6];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;

    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetProximityRawData, data, 6);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("read RegMapR_GetProximityRawData failed!\n");
        return err;
    }
    SH_LOG("RawData4:%u,%u,%u)\n", ptr[0], ptr[1], ptr[2]);
    return sprintf(buf, "%d %u %u %u\n", err, ptr[0], ptr[1], ptr[2]);
}

static ssize_t get_mag_special_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[64];
    uint16_t *ptr;
    int err;
    ptr = (uint16_t *)data;
    SH_FUN();
    if (CW_BOOT == sensor->mcu_mode)
    {
        SH_LOG("mcu_mode == CW_BOOT!\n");
        return FAIL;
    }
    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_Object_read(sensor, RegMapR_MagSpecialData, data, 64);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("read RegMapR_MagSpecialData failed!\n");
        return err;
    }
    memcpy(buf,data,64);
    return 64;
}

static ssize_t set_sys_msg(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t data[40] = {0};
    int temp[40] = {0};
    int i,err = 0;

    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return count;
    }

    sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
        &temp[0], &temp[1], &temp[2],
        &temp[3], &temp[4], &temp[5], &temp[6], &temp[7], &temp[8], &temp[9], &temp[10], &temp[11], &temp[12],
        &temp[13], &temp[14], &temp[15], &temp[16], &temp[17], &temp[18], &temp[19], &temp[20], &temp[21], &temp[22],
        &temp[23], &temp[24], &temp[25], &temp[26], &temp[27], &temp[28], &temp[29], &temp[30], &temp[31]);

    for (i = 0 ; i < 40; i++)
        data[i] = (uint8_t)temp[i];

    if(data[0] == GPSINFO)
    {
        memset(sensor->GPSData, 0 ,sizeof(uint8_t)*40);
        memcpy(sensor->GPSData, data, sizeof(uint8_t)*40);
    }
    if(data[0] == HEALTHINFO)
    {
        memset(sensor->HEALTHData, 0 ,sizeof(uint8_t)*30);
        memcpy(sensor->HEALTHData, data, sizeof(uint8_t)*30);
    }

    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_SetSystemMsg, data, 40);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
        SH_ERR("I2c Write Fail!\n");

    return count;
}

static ssize_t get_sys_msg(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    uint8_t extrahub_data[30] = {0};
    int err;

    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_R(sensor, RegMapR_GetSystemMsg, extrahub_data, 30);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
        err,
        extrahub_data[0], extrahub_data[1], extrahub_data[2],extrahub_data[3], extrahub_data[4], extrahub_data[5], extrahub_data[6], extrahub_data[7],
        extrahub_data[8], extrahub_data[9], extrahub_data[10], extrahub_data[11], extrahub_data[12], extrahub_data[13], extrahub_data[14], extrahub_data[15],
        extrahub_data[16], extrahub_data[17], extrahub_data[18], extrahub_data[19], extrahub_data[20], extrahub_data[21], extrahub_data[22],
        extrahub_data[23], extrahub_data[24], extrahub_data[25], extrahub_data[26], extrahub_data[27], extrahub_data[28], extrahub_data[29]);
}

#ifndef CWMCU_CALIB_SAVE_IN_FLASH
static void reload_calib_data(struct CWMCU_T *sensor)
{
    int i;
    for(i = 0;i < DRIVER_ID_END ; i ++)
    {
        if(sensor->calibratorUpdate[i])
        {
            sensors_calib_data_write(sensor, i, sensor->calibratordata[i]);
            msleep(10);
        }
    }
}
#endif

int CWM_Restore_GPSINFO(struct CWMCU_T *sensor)
{
    int err = 0;
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_SetSystemMsg, sensor->GPSData, 40);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return 0;
    }
    return 1;
}

int CWM_Restore_HealthINFO(struct CWMCU_T *sensor)
{
    int err = 0;
    if (sensor->mcu_mode == CW_BOOT)
    {
        SH_ERR("mcu_mode == CW_BOOT!\n");
        return 0;
    }

    power_pin_sw(sensor,SWITCH_POWER_SYS, 1);
    err = CWMCU_I2C_W(sensor, RegMapW_SetSystemMsg, sensor->HEALTHData, 30);
    power_pin_sw(sensor,SWITCH_POWER_SYS, 0);
    if (err < 0)
    {
        SH_ERR("I2c Write Fail!\n");
        return 0;
    }
    return 1;
}

static void cwmcu_reinit(struct CWMCU_T *sensor)
{
    sensor->mcu_init_count++;

#ifndef CWMCU_CALIB_SAVE_IN_FLASH
    reload_calib_data(sensor);
#endif
    check_enable_list(sensor);
    cwmcu_kernel_status(sensor,KERNEL_RESUME);

    CWM_Restore_GPSINFO(sensor);
    CWM_Restore_HealthINFO(sensor);
}

// neiltsai, 20151211, add for probe_success read permission warning
static ssize_t show_probe_success(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n", sensor->mcu_probe_success);
}
// neiltsai end


static struct device_attribute attributes[] = {
	__ATTR(enable, 0644,  active_show, active_set),
	__ATTR(batch, 0200, NULL, batch_set),
	__ATTR(flush, 0200, NULL, flush_set),
	__ATTR(mcu_mode, 0644, mcu_mode_show, mcu_model_set),
	__ATTR(calibrator_cmd, 0644,  get_calibrator_cmd, set_calibrator_cmd),
	__ATTR(calibrator_data, 0644, get_calibrator_data, set_calibrator_data),
	__ATTR(firmware_update_i2c, 0644, get_firmware_update_i2, set_firmware_update_i2),
	__ATTR(firmware_update_cmd, 0200, NULL, set_firmware_update_cmd),
	__ATTR(firmware_update_data, 0200, NULL, set_firmware_update_data),
	__ATTR(firmware_update_status, 0444, get_firmware_update_status, NULL),
	__ATTR(version, 0444,  version_show, NULL),
    __ATTR(library_version, 0444,  library_version_show, NULL),
	__ATTR(timestamp, 0444, timestamp_show, NULL),
    __ATTR(sys_cmd, 0200,  NULL, set_sys_cmd),
    __ATTR(sys_msg, 0644,  get_sys_msg, set_sys_msg),
    __ATTR(raw_data0, 0444, get_raw_data0, NULL),
    __ATTR(raw_data1, 0444, get_raw_data1, NULL),
    __ATTR(raw_data2, 0444, get_raw_data2, NULL),
    __ATTR(raw_data3, 0444, get_raw_data3, NULL),
    __ATTR(raw_data4, 0444, get_raw_data4, NULL),
    __ATTR(mag_special_data, 0444, get_mag_special_data, NULL),
	__ATTR(ssh_info, 0444, sensorhub_info_show, NULL),
};

static struct device_attribute attributes_hw_detect[] = {
// neiltsai, 20151211, modify for read permission warning
    __ATTR(probe_success, 0444, show_probe_success, NULL)
// neiltsai end
};

static void CWMCU_IRQ(struct CWMCU_T *sensor)
{
    uint8_t temp[2] = {0};
    uint8_t data_event[24] = {0};

	if (sensor->mcu_mode == CW_BOOT) {
		SH_LOG("mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
                return;
	}

	sensor->interrupt_status = 0;
    if (CWMCU_I2C_R(sensor, RegMapR_InterruptStatus, temp, 2) >= 0)
    {
        sensor->interrupt_status = (u32)temp[1] << 8 | (u32)temp[0];

        if (sensor->debug_log & (1<<D_IRQ)){
			SH_LOG("interrupt_status:%x ,temp[0-1]: %x %x,func:%s, line:%d\n ",
				sensor->interrupt_status, (u32)temp[0],(u32)temp[1],__func__, __LINE__);
		}
		if( sensor->interrupt_status >= (1<<IRQ_MAX_SIZE)){
			sensor->interrupt_status = 0;
			SH_LOG("interrupt_status > IRQ_MAX_SIZE func:%s, line:%d\n", __func__, __LINE__);
		}
    }
    else
    {
		SH_LOG(" Read interrupt_status Fail [I2C], func: %s , li: %d\n",__func__,__LINE__);
		sensor->interrupt_status = 0;
    }

    if (sensor->interrupt_status & (1<<IRQ_INIT))
    {
        cwmcu_reinit(sensor);
        cw_send_event(sensor, NonWakeUpHandle, MCU_REINITIAL, data_event);
    }

    if (sensor->interrupt_status & (1<<IRQ_GESTURE))
    {
        cwmcu_read_gesture(sensor);
    }

    if ((sensor->interrupt_status & (1<<IRQ_BATCH_TIMEOUT)) ||(sensor->interrupt_status & (1<<IRQ_BATCH_FULL)) )
    {
        cwmcu_read_buff(sensor,WakeUpHandle);
    }

    if (sensor->interrupt_status & (1<<IRQ_DATA_READY))
    {
		cwmcu_read_buff(sensor, NonWakeUpHandle);
    }

    if (sensor->interrupt_status & (1<<IRQ_INFO))
    {
        read_mcu_info(sensor);
    }
    if (sensor->interrupt_status & (1<<IRQ_CALIB))
    {
        read_calib_info(sensor);
    }
    if (sensor->interrupt_status & (1<<IRQ_ERROR))
    {
        read_error_code(sensor);
    }
}

static int CWMCU_suspend(struct device *dev)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);

	SH_LOG(" CWMCU_suspend.\n");

	disable_irq(sensor->non_wakeup);
    power_pin_sw(sensor,SWITCH_POWER_PROBE, 1);
	cwmcu_kernel_status(sensor,KERNEL_SUSPEND);
    power_pin_sw(sensor,SWITCH_POWER_PROBE, 0);
    return 0;
}

static int CWMCU_resume(struct device *dev)
{
    struct CWMCU_T *sensor = dev_get_drvdata(dev);

	SH_LOG(" CWMCU_resume.\n");

    power_pin_sw(sensor,SWITCH_POWER_PROBE, 1);
	cwmcu_kernel_status(sensor,KERNEL_RESUME);
    power_pin_sw(sensor,SWITCH_POWER_PROBE, 0);
	enable_irq(sensor->non_wakeup);

    return 0;
}

/*=======iio device reg=========*/
static void iio_trigger_work(struct irq_work *work)
{
    struct CWMCU_T *mcu_data = container_of((struct irq_work *)work, struct CWMCU_T, iio_irq_work);

    //iio_trigger_poll(mcu_data->trig, iio_get_time_ns());
    iio_trigger_poll(mcu_data->trig);
}

static irqreturn_t cw_trigger_handler(int irq, void *p)
{
    struct iio_poll_func *pf = p;
    struct iio_dev *indio_dev = pf->indio_dev;
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

#ifdef CWMCU_MUTEX
    mutex_lock(&mcu_data->mutex_lock);
#endif
    iio_trigger_notify_done(mcu_data->indio_dev->trig);
#ifdef CWMCU_MUTEX
    mutex_unlock(&mcu_data->mutex_lock);
#endif

    return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops cw_buffer_setup_ops = {
//    .preenable = &iio_sw_buffer_preenable,
    .postenable = &iio_triggered_buffer_postenable,
    .predisable = &iio_triggered_buffer_predisable,
};

static int cw_pseudo_irq_enable(struct iio_dev *indio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

    if (!atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 0, 1))
    {
        SH_FUN();
        cancel_work_sync(&mcu_data->work);
        queue_work(mcu_data->driver_wq, &mcu_data->work);
#ifdef NON_WAKEUP_IRQ
		cancel_work_sync(&mcu_data->non_wakeup_work);
        queue_work(mcu_data->driver_wq, &mcu_data->non_wakeup_work);
#endif
    }

    return 0;
}

static int cw_pseudo_irq_disable(struct iio_dev *indio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

    if (atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 1, 0))
    {
        cancel_work_sync(&mcu_data->work);
#ifdef NON_WAKEUP_IRQ
		cancel_work_sync(&mcu_data->non_wakeup_work);
#endif
        SH_FUN();
    }
    return 0;
}

static int cw_set_pseudo_irq(struct iio_dev *indio_dev, int enable)
{
    if (enable)
        cw_pseudo_irq_enable(indio_dev);
    else
        cw_pseudo_irq_disable(indio_dev);
    return 0;
}

static int cw_data_rdy_trigger_set_state(struct iio_trigger *trig, bool state)
{
    struct iio_dev *indio_dev = (struct iio_dev *)iio_trigger_get_drvdata(trig);
#ifdef CWMCU_MUTEX
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);
    mutex_lock(&mcu_data->mutex_lock);
#endif
    cw_set_pseudo_irq(indio_dev, state);
#ifdef CWMCU_MUTEX
    mutex_unlock(&mcu_data->mutex_lock);
#endif

    return 0;
}

static const struct iio_trigger_ops cw_trigger_ops = {
    .owner = THIS_MODULE,
    .set_trigger_state = &cw_data_rdy_trigger_set_state,
};

static int cw_probe_trigger(struct iio_dev *iio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(iio_dev);
    int ret;

    iio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time, &cw_trigger_handler,
                            IRQF_ONESHOT, iio_dev, "%s_consumer%d", iio_dev->name, iio_dev->id);
    if (NULL == iio_dev->pollfunc)
    {
        ret = -ENOMEM;
        goto error_ret;
    }

    mcu_data->trig = iio_trigger_alloc("%s-dev%d",
            iio_dev->name,
            iio_dev->id);
    if (!mcu_data->trig) {
        ret = -ENOMEM;
        goto error_dealloc_pollfunc;
    }

    mcu_data->trig->dev.parent = &mcu_data->client->dev;
    mcu_data->trig->ops = &cw_trigger_ops;
    iio_trigger_set_drvdata(mcu_data->trig, iio_dev);

    ret = iio_trigger_register(mcu_data->trig);
    if (ret)
        goto error_free_trig;

    return 0;

error_free_trig:
    iio_trigger_free(mcu_data->trig);
error_dealloc_pollfunc:
    iio_dealloc_pollfunc(iio_dev->pollfunc);
error_ret:
    return ret;
}

static int cw_probe_buffer(struct iio_dev *iio_dev)
{
    int ret;
    struct iio_buffer *buffer;

    buffer = iio_kfifo_allocate(iio_dev);
    if (!buffer)
    {
        ret = -ENOMEM;
        goto error_ret;
    }

    buffer->scan_timestamp = true;
    iio_dev->buffer = buffer;
    iio_dev->setup_ops = &cw_buffer_setup_ops;
    iio_dev->modes |= INDIO_BUFFER_TRIGGERED;
    ret = iio_buffer_register(iio_dev, iio_dev->channels,
                  iio_dev->num_channels);
    if (ret)
        goto error_free_buf;

    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_ID);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_X);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Y);
    iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Z);
    return 0;

error_free_buf:
    iio_kfifo_free(iio_dev->buffer);
error_ret:
    return ret;
}

static int cw_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
                            int *val, int *val2, long mask)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);
    int ret = -EINVAL;

    if (chan->type != IIO_ACCEL)
        return ret;

#ifdef CWMCU_MUTEX
    mutex_lock(&mcu_data->mutex_lock);
#endif
    switch (mask)
    {
    case 0:
        *val = mcu_data->iio_data[chan->channel2 - IIO_MOD_X];
        ret = IIO_VAL_INT;
        break;

    case IIO_CHAN_INFO_SCALE:
        /* Gain : counts / uT = 1000 [nT] */
        /* Scaling factor : 1000000 / Gain = 1000 */
        *val = 0;
        *val2 = 1000;
        ret = IIO_VAL_INT_PLUS_MICRO;
        break;

        default:
            break;
    }
#ifdef CWMCU_MUTEX
    mutex_unlock(&mcu_data->mutex_lock);
#endif
    return ret;
}

enum CW_axis {
       AXIS_CW_SCAN_ID,
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
};

#define CW_CHANNEL(_axis)                    \
{                                           \
    .type = IIO_ACCEL,                      \
    .modified = 1,                          \
	.channel2 = _axis+1,			\
    .info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),  \
	.scan_index = _axis,			\
    .scan_type = {'u', 32, 32, 0},     \
}

static const struct iio_chan_spec cw_channels[] = {
    CW_CHANNEL(CW_SCAN_ID),
    CW_CHANNEL(CW_SCAN_X),
    CW_CHANNEL(CW_SCAN_Y),
    CW_CHANNEL(CW_SCAN_Z),
    //CW_CHANNEL(X),
    //CW_CHANNEL(Y),
    //CW_CHANNEL(Z),
    IIO_CHAN_SOFT_TIMESTAMP(CW_SCAN_TIMESTAMP)
};

static const struct iio_info cw_info = {
    .read_raw = &cw_read_raw,
    .driver_module = THIS_MODULE,
};



static int create_sysfs_interfaces(struct CWMCU_T *mcu_data, bool probe_success)
{
    int i;
    int res;

    SH_FUN();
    mcu_data->sensor_class = class_create(THIS_MODULE, "cywee_sensorhub");
    // neiltsai, 20151211, add for probe_success read permission warning
    mcu_data->mcu_probe_success = 0;
    // neiltsai end
    if (IS_ERR(mcu_data->sensor_class))
        return PTR_ERR(mcu_data->sensor_class);

    mcu_data->sensor_dev = device_create(mcu_data->sensor_class, NULL, 0, "%s", "sensor_hub");
    if (IS_ERR(mcu_data->sensor_dev))
    {
        res = PTR_ERR(mcu_data->sensor_dev);
        goto err_device_create;
    }

    dev_set_drvdata(mcu_data->sensor_dev, mcu_data);
    /*
    res = dev_set_drvdata(mcu_data->sensor_dev, mcu_data);
    if (res)
        goto err_set_drvdata;*/

    for (i = 0; i < ARRAY_SIZE(attributes); i++)
        if (device_create_file(mcu_data->sensor_dev, attributes + i))
            goto error;
    // neiltsai, 20151211, modify for probe_success read permission warning
    if(probe_success){
	   device_create_file(mcu_data->sensor_dev, attributes_hw_detect);
       mcu_data->mcu_probe_success = 1;
    }
    // neiltsai end
    res = sysfs_create_link(&mcu_data->sensor_dev->kobj, &mcu_data->indio_dev->dev.kobj, "iio");
    if (res < 0)
        goto error;

    return 0;

error:
    while (--i >= 0)
        device_remove_file(mcu_data->sensor_dev, attributes + i);
/*
err_set_drvdata:
    put_device(mcu_data->sensor_dev);
    device_unregister(mcu_data->sensor_dev);*/
err_device_create:
    class_destroy(mcu_data->sensor_class);
    return res;
}

#ifdef CWMCU_INTERRUPT
static irqreturn_t CWMCU_interrupt_thread(int irq, void *data)
{
    struct CWMCU_T *sensor = data;
    // printk(KERN_DEBUG "CwMcu:%s in\n", __func__);
    if (sensor->mcu_mode == CW_BOOT) {
        SH_LOG(" %s:(sensor->mcu_mode = CW_BOOT)\n",__FUNCTION__);
        return IRQ_HANDLED;
    }
    schedule_work(&sensor->work);

    return IRQ_HANDLED;
}

static void cwmcu_work_report(struct work_struct *work)
{
   struct CWMCU_T *sensor = container_of((struct work_struct *)work,
            struct CWMCU_T, work);

    if (sensor->mcu_mode == CW_BOOT) {
		SH_LOG("mcu_mode = boot, func:%s, line:%d\n", __func__, __LINE__);
        return;
    }

    power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 1);
    CWMCU_IRQ(sensor);
    power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 0);
}
#endif

#ifdef NON_WAKEUP_IRQ
static irqreturn_t CWMCU_nonwakeup_interrupt_thread(int irq, void *data)
{
    struct CWMCU_T *sensor = data;
    if (sensor->mcu_mode == CW_BOOT) {
        printk("%s:%s:(sensor->mcu_mode = CW_BOOT)\n",LOG_TAG_KERNEL ,__FUNCTION__);
        return IRQ_HANDLED;
    }
    schedule_work(&sensor->non_wakeup_work);
    return IRQ_HANDLED;
}
static void cwmcu_nonwake_work_report(struct work_struct *work)
{
   struct CWMCU_T *sensor = container_of((struct work_struct *)work,
            struct CWMCU_T, non_wakeup_work);
    if (sensor->mcu_mode == CW_BOOT) {
        printk("%s:%s:(sensor->mcu_mode = CW_BOOT)\n",LOG_TAG_KERNEL ,__FUNCTION__);
        return;
    }
    power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 1);
    CWMCU_IRQ(sensor);
    power_pin_sw(sensor,SWITCH_POWER_INTERRUPT, 0);
}
#endif
static int cwstm_parse_dt(struct device *dev,
             struct CWMCU_T *sensor)
{
    struct device_node *np = dev->of_node;
    int ret = 0;

    ret = of_get_named_gpio(np, "cwstm,irq-gpio", 0);
    if (ret < 0) {
        pr_err("failed to get \"cwstm,irq_gpio\"\n");
        goto err;
    }
    sensor->irq_gpio = ret;
    ret = of_get_named_gpio(np, "cwstm,wakeup-gpio", 0);
    if (ret < 0) {
        pr_err("failed to get \"wakeup\"\n");
        goto err;
    }
    sensor->wakeup_gpio = ret;
#ifdef NON_WAKEUP_IRQ
	ret = of_get_named_gpio(np, "cwstm,non-irq-gpio", 0);
    if (ret < 0) {
        pr_err("failed to get \"cwstm,non-irq-gpio\"\n");
        goto err;
    }
    sensor->non_wakeup = ret;
#endif

/*************zhujp2 add start*************************/
        ret = of_get_named_gpio(np, "cwstm,boot-gpio", 0);
        if (ret < 0) {
                pr_err("failed to get \"reset\"\n");
                goto err;
        }
        sensor->boot_gpio = ret;

        ret = of_get_named_gpio(np, "cwstm,reset-gpio", 0);
        if (ret < 0) {
                pr_err("failed to get \"reset\"\n");
                goto err;
        }
        sensor->reset_gpio = ret;
/*************zhujp2 add end**************************/
err:
    return ret;
}

static void cwmcu_remove_trigger(struct iio_dev *indio_dev)
{
    struct CWMCU_T *mcu_data = iio_priv(indio_dev);

    iio_trigger_unregister(mcu_data->trig);
    iio_trigger_free(mcu_data->trig);
    iio_dealloc_pollfunc(indio_dev->pollfunc);
}

static void cwmcu_remove_buffer(struct iio_dev *indio_dev)
{
    iio_buffer_unregister(indio_dev);
    iio_kfifo_free(indio_dev->buffer);
}


static int cwstm_power_on(struct CWMCU_T *sensor,bool on)
{
    int rc;
    int rc1;

    if (!on)
        goto power_off;

    rc = regulator_enable(sensor->vdd);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }

 /************zhujp2 add*************/
    rc = regulator_enable(sensor->vdd_sensors);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }


    rc = regulator_enable(sensor->vdd_ir);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd enable failed rc=%d\n", rc);
        return rc;
    }
/**************zhujp2 add end**********/

    rc = regulator_enable(sensor->vcc_i2c);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vcc_i2c enable failed rc=%d\n", rc);
        regulator_disable(sensor->vdd);
    }

    // neil test
    rc = regulator_enable(sensor->vdd_als_p_M);
    if (rc)
    {
        SH_ERR("Regulator vdd_als_p_M enable failed rc=%d\n", rc);
        rc1 = regulator_disable(sensor->vdd);
		if (rc1)
		{
		  SH_ERR("Regulator vdd enable failed rc=%d\n", rc1);
		  return  rc1;
		}
    rc1 = regulator_disable(sensor->vcc_i2c);
    if (rc1)
		{
		  SH_ERR("Regulator vcc_i2c enable failed rc=%d\n", rc1);
		  return  rc1;
		}
    }
    // neil end
    return rc;


power_off:
    rc = regulator_disable(sensor->vdd);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }
/***********zhujp2 add ****************/
    rc = regulator_disable(sensor->vdd_sensors);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }

    rc = regulator_disable(sensor->vdd_ir);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vdd disable failed rc=%d\n", rc);
        return rc;
    }
/*************zhujp2 add end***************/
    rc = regulator_disable(sensor->vcc_i2c);
    if (rc) {
        dev_err(&sensor->client->dev,
            "Regulator vcc_i2c disable failed rc=%d\n", rc);
        regulator_disable(sensor->vdd);
    }

    return rc;
}

static int cwstm_power_init(struct CWMCU_T *sensor,bool on)
{
    int rc;

    if (!on)
        goto pwr_deinit;

    sensor->vdd = regulator_get(&sensor->client->dev, "cwstm,vdd_ana");
    if (IS_ERR(sensor->vdd)) {
        rc = PTR_ERR(sensor->vdd);
        dev_err(&sensor->client->dev,
            "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(sensor->vdd) > 0) {
        rc = regulator_set_voltage(sensor->vdd, FT_VTG_MIN_UV,
                       FT_VTG_MAX_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
                "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_put;
        }
    }

    sensor->vcc_i2c = regulator_get(&sensor->client->dev, "cwstm,vcc_i2c");
    if (IS_ERR(sensor->vcc_i2c)) {
        rc = PTR_ERR(sensor->vcc_i2c);
        dev_err(&sensor->client->dev,
            "Regulator get failed vcc_i2c rc=%d\n", rc);
        goto reg_vdd_set_vtg;
    }

    if (regulator_count_voltages(sensor->vcc_i2c) > 0) {
        rc = regulator_set_voltage(sensor->vcc_i2c, FT_I2C_VTG_MIN_UV,
                       FT_I2C_VTG_MAX_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
            "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
            goto reg_vcc_i2c_put;
        }
    }
    // neil test
    sensor->vdd_als_p_M = regulator_get(&sensor->client->dev, "cwstm,vdd_als_p_M");
    if (IS_ERR(sensor->vdd_als_p_M)) {
        rc = PTR_ERR(sensor->vdd_als_p_M);
        dev_err(&sensor->client->dev,
            "Regulator get failed vdd_als_p_M rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(sensor->vdd_als_p_M) > 0) {
        rc = regulator_set_voltage(sensor->vdd_als_p_M, FT_VTG_ALS_P_M_MIN_UV,
                       FT_VTG_ALS_P_M_MAX_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
            "Regulator set_vtg failed vdd_als_p_M rc=%d\n", rc);
            return rc;
        }
    }
    // neil end

 /**************zhujp2 add*****************/

    sensor->vdd_sensors = regulator_get(&sensor->client->dev, "vdd_sensors");
    if (IS_ERR(sensor->vdd_sensors)) {
        rc = PTR_ERR(sensor->vdd_sensors);
        dev_err(&sensor->client->dev,
            "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(sensor->vdd_sensors) > 0) {
        rc = regulator_set_voltage(sensor->vdd_sensors, FT_VTG_MIN_B_UV,
                       FT_VTG_MAX_B_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
                "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_sensors_put;
        }
    }


    sensor->vdd_ir = regulator_get(&sensor->client->dev, "vdd_ir");
    if (IS_ERR(sensor->vdd_ir)) {
        rc = PTR_ERR(sensor->vdd_ir);
        dev_err(&sensor->client->dev,
            "Regulator get failed vdd rc=%d\n", rc);
        return rc;
    }

    if (regulator_count_voltages(sensor->vdd_ir) > 0) {
        rc = regulator_set_voltage(sensor->vdd_ir, FT_VTG_MIN_C_UV,
                       FT_VTG_MAX_C_UV);
        if (rc) {
            dev_err(&sensor->client->dev,
                "Regulator set_vtg failed vdd rc=%d\n", rc);
            goto reg_vdd_ir_put;
        }
    }



/****************zhujp2 add end*************/
    return 0;

reg_vcc_i2c_put:
    regulator_put(sensor->vcc_i2c);
reg_vdd_set_vtg:
    if (regulator_count_voltages(sensor->vdd) > 0)
        regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
    regulator_put(sensor->vdd);
reg_vdd_sensors_put:                                //zhujp2 add
    regulator_put(sensor->vdd_sensors);
reg_vdd_ir_put:                                //zhujp2 add
    regulator_put(sensor->vdd_ir);
    return rc;

pwr_deinit:
    if (regulator_count_voltages(sensor->vdd) > 0)
        regulator_set_voltage(sensor->vdd, 0, FT_VTG_MAX_UV);

    regulator_put(sensor->vdd);

    if (regulator_count_voltages(sensor->vcc_i2c) > 0)
        regulator_set_voltage(sensor->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

    regulator_put(sensor->vcc_i2c);
    return 0;
}

static void cwmcu_hw_config_init(struct CWMCU_T *sensor)
{
    int i = 0;
    int j = 0;

    sensor->initial_hw_config = 0;
    for(i = 0; i < HANDLE_ID_END; i++)
    {
        sensor->enabled_list[i] = 0;
        for(j = 0;j<SENSORS_ID_END;j++)
        {
            sensor->sensors_info[i][j].en = 0;
            sensor->sensors_info[i][j].mode= 0;
            sensor->sensors_info[i][j].rate = 0;
            sensor->sensors_info[i][j].timeout= 0;
        }
    }

    sensor->interrupt_status = 0;
    sensor->power_on_list = 0;
    sensor->cal_cmd = 0;
    sensor->cal_type = 0;
    sensor->cal_id = 0;
    sensor->debug_log = 0;
    for(i = 0;i<DRIVER_ID_END;i++){
        sensor->hw_info[i].hw_id=0;
        sensor->calibratorUpdate[i]=0;
        for(j = 0;j<30;j++)
        {
            sensor->calibratordata[i][j]=0;
        }
    }

}

static void cwmcu_reset_mcu(void)
{
    // neil test
	gpio_direction_output(GPIO_CW_MCU_BOOT, 0);
	gpio_direction_output(GPIO_CW_MCU_RESET, 0);
	//gpio_direction_output(GPIO_CW_MCU_INTERRUPT, 1);
	gpio_direction_output(GPIO_CW_MCU_WAKE_UP, 0);
	// neil end
	msleep(50);
	gpio_set_value(GPIO_CW_MCU_RESET, 1);
	msleep(50);
	gpio_set_value(GPIO_CW_MCU_RESET, 0);
	msleep(50);
	gpio_set_value(GPIO_CW_MCU_RESET, 1);
	msleep(100);
	gpio_direction_input(GPIO_CW_MCU_INTERRUPT);

}

static int CWMCU_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct CWMCU_T *mcu;
    struct iio_dev *indio_dev;
    int error;
    bool probe_success;
    u8 data[10] = {0};  // neil test

    SH_LOG(" MAIN_VERSION : %s \n", MAIN_VERSION);
    SH_LOG(" %s:%s:(sensor->mcu_mode = CW_BOOT)\n",LOG_TAG_KERNEL ,__FUNCTION__);

    dev_dbg(&client->dev, "%s:\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "-CWMCU- i2c_check_functionality error\n");
        return -EIO;
    }

    indio_dev = iio_device_alloc(sizeof(*mcu));
    if (!indio_dev) {
		SH_LOG(" %s: iio_device_alloc failed\n", __func__);
        return -ENOMEM;
    }

    i2c_set_clientdata(client, indio_dev);

    indio_dev->name = CWMCU_I2C_NAME;
    indio_dev->dev.parent = &client->dev;
    indio_dev->info = &cw_info;
    indio_dev->channels = cw_channels;
    indio_dev->num_channels = ARRAY_SIZE(cw_channels);
    indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

    mcu = iio_priv(indio_dev);
    mcu->client = client;
    mcu->indio_dev = indio_dev;

/**************************zhujp2 modify********************/
    error = cwstm_power_init(mcu,true);

    if (error) {
        dev_err(&client->dev, "power init failed");
    }
    printk("stm_power_init success !!!!!!!!!!!!!! \n");
    error = cwstm_power_on(mcu,true);
    if (error) {
        dev_err(&client->dev, "power on failed");
    }
    printk("cwstm_power_on success !!!!!!!!!!!!!! \n");
/*****************************zhujp2 modify*****************/

    mcu->mcu_mode = CW_BOOT;//castor
    error = cwstm_parse_dt(&client->dev, mcu);
    if (error < 0) {
        pr_err("failed to parse device tree: %d\n", error);
        goto err_parse_dt;
    }
    printk("cwstm_parse_dt success !!!!!!!!!!!!!! \n");
    gpio_request(mcu->wakeup_gpio, "cwstm,wakeup-gpio");
    printk("wakeup_gpio gpio_request success !!!!!!!!!!!!!! \n");
    gpio_request(mcu->irq_gpio, "cwstm,irq-gpio");
    printk("irq-gpio gpio_request success !!!!!!!!!!!!!! \n");
/************zhujp2 add ********/
    gpio_request(mcu->boot_gpio, "cwstm,boot-gpio");
    gpio_direction_output(mcu->boot_gpio, 0);
 //   gpio_set_value(mcu->boot_gpio, 0);
    printk("boot-gpio  gpio_request success !!!!!!!!!!!!!! \n");
    gpio_request(mcu->reset_gpio, "cwstm,reset-gpio");
    gpio_direction_output(mcu->reset_gpio, 1);
 //   gpio_set_value(mcu->reset_gpio, 1);
   printk("reset-gpio  gpio_request success !!!!!!!!!!!!!! \n");
/************zhujp2 add end ********/

/***************************
    error = cwstm_power_init(mcu,true);
    if (error) {
        dev_err(&client->dev, "power init failed");
    }

    error = cwstm_power_on(mcu,true);
    if (error) {
        dev_err(&client->dev, "power on failed");
    }
**************************************/

	cwmcu_reset_mcu();//castor


#ifdef CWMCU_MUTEX
    mutex_init(&mcu->mutex_lock);
    mutex_init(&mcu->mutex_lock_i2c);
    mutex_init(&mcu->mutex_wakeup_gpio);
    mcu->supend_flag = 1;
#endif
    //mcu->mcu_mode = CW_NORMAL;  // neil test

    error = cw_probe_buffer(indio_dev);
    if (error) {
        printk("%s: iio yas_probe_buffer failed\n", __func__);
        goto error_free_dev;
    }
    error = cw_probe_trigger(indio_dev);
    if (error) {
        printk("%s: iio yas_probe_trigger failed\n", __func__);
        goto error_remove_buffer;
    }
    error = iio_device_register(indio_dev);
    if (error) {
        printk("%s: iio iio_device_register failed\n", __func__);
        goto error_remove_trigger;
    }
  // neil test
  probe_success = (CWMCU_I2C_R(mcu, RegMapR_GetFWVersion, data, 4) >= 0) ? 1 : 0;
  printk("%s:(check firmware version: %u,%u,%u,%u)\n", __func__, data[0],data[1],data[2],data[3]);
  // neil end

    error = create_sysfs_interfaces(mcu, probe_success);
    if (error)
        goto err_free_mem;

    atomic_set(&mcu->delay, 20);
    init_irq_work(&mcu->iio_irq_work, iio_trigger_work);

    cwmcu_hw_config_init(mcu);

    mcu->driver_wq = create_singlethread_workqueue("cywee_mcu");
    i2c_set_clientdata(client, mcu);
    pm_runtime_enable(&client->dev);
    power_pin_sw(mcu,SWITCH_POWER_PROBE, 1);
    cwmcu_kernel_status(mcu,KERNEL_PROBE);
    power_pin_sw(mcu,SWITCH_POWER_PROBE, 0);


	mcu->mcu_init_count = 0;

#ifdef CWMCU_INTERRUPT
    mcu->client->irq =gpio_to_irq(mcu->irq_gpio);
    //gpio_request(sensor->irq_gpio, "cwstm,irq-gpio");
    gpio_direction_output(mcu->irq_gpio, 1);
    //udelay(20000);
	mdelay(20);
    pr_info("%s:irq gpio vlaue %d\n", __func__, gpio_get_value(mcu->irq_gpio));
    gpio_direction_input(mcu->irq_gpio);
    //udelay(5000);
	mdelay(5);

    printk("%s:%s:(sensor->client->irq  =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__, mcu->client->irq);


        error = request_threaded_irq(mcu->client->irq, NULL,
                           CWMCU_interrupt_thread,
                           IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                           "cwmcu", mcu);
        if (error < 0) {
                pr_err("request irq %d failed\n", mcu->client->irq);
                goto exit_destroy_mutex;
        }

        INIT_WORK(&mcu->work, cwmcu_work_report);

        error = enable_irq_wake(mcu->client->irq);
        if (error < 0)
            printk("[CWMCU] could not enable irq as wakeup source %d\n", error);

#endif
#ifdef NON_WAKEUP_IRQ
    mcu->non_wakeup = gpio_to_irq(GPIO_CW_MCU_NON_WAKEUP);
	SH_LOG(" %s:%s:(mcu->non_wakeup  =%d)\n",LOG_TAG_KERNEL ,__FUNCTION__, mcu->non_wakeup);
    if (mcu->non_wakeup > 0) {
        error = request_threaded_irq(mcu->non_wakeup, NULL,
                           CWMCU_nonwakeup_interrupt_thread,
                           IRQF_TRIGGER_FALLING |  IRQF_ONESHOT,
                           "cwmcu-non-wakeup", mcu);
        if (error < 0) {
			SH_LOG("request irq %d failed\n", mcu->non_wakeup);
                goto exit_destroy_mutex;
        }
		INIT_WORK(&mcu->non_wakeup_work, cwmcu_nonwake_work_report);
		error = enable_irq_wake(mcu->non_wakeup);
		if (error < 0)
			SH_LOG(" could not enable irq as wakeup source %d\n", error);
	}
#endif

    printk("%s:%s:(probe success)\n",LOG_TAG_KERNEL ,__FUNCTION__);
	mcu->mcu_mode = CW_NORMAL;  // neil test

    return 0;

err_free_mem:
    iio_device_unregister(indio_dev);
error_remove_trigger:
    cwmcu_remove_trigger(indio_dev);
error_remove_buffer:
    cwmcu_remove_buffer(indio_dev);
error_free_dev:
err_parse_dt:
#ifdef CWMCU_INTERRUPT
exit_destroy_mutex:
#endif
    iio_device_free(indio_dev);
    i2c_set_clientdata(client, NULL);
	SH_LOG(" sensorhub probe fail.\n");
    return error;
}

static int CWMCU_i2c_remove(struct i2c_client *client)
{
    struct CWMCU_T *sensor = i2c_get_clientdata(client);
    kfree(sensor);
    return 0;
}

static struct of_device_id cwstm_match_table[] = {
    { .compatible = "cwstm,cwstm32",},
    { },
};

static const struct dev_pm_ops CWMCU_pm_ops = {
    .suspend = CWMCU_suspend,
    .resume = CWMCU_resume
};

static const struct i2c_device_id CWMCU_id[] = {
    { CWMCU_I2C_NAME, 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, CWMCU_id);

static struct i2c_driver CWMCU_driver = {
    .driver = {
        .name = CWMCU_I2C_NAME,
        .owner = THIS_MODULE,
        .pm = &CWMCU_pm_ops,
        .of_match_table = cwstm_match_table,
    },
    .probe    = CWMCU_i2c_probe,
    .remove   = CWMCU_i2c_remove,
    .id_table = CWMCU_id,
};

static int __init CWMCU_i2c_init(void){
    printk("%s:%s:(init)\n",LOG_TAG_KERNEL ,__FUNCTION__);
    return i2c_add_driver(&CWMCU_driver);
}

static void __exit CWMCU_i2c_exit(void){
    i2c_del_driver(&CWMCU_driver);
}

module_init(CWMCU_i2c_init);
module_exit(CWMCU_i2c_exit);

MODULE_DESCRIPTION("CWMCU I2C Bus Driver");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
