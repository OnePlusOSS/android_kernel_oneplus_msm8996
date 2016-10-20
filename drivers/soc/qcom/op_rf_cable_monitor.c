/*For OEM project monitor RF cable connection status,
*and config different RF configuration
*
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
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <soc/qcom/subsystem_restart.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of_gpio.h>

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

static struct project_info * project_info_desc;

struct cable_data {
	int irq;
	int cable_gpio;

	//struct regulator *vdd_io;
	//struct work_struct work;
	struct delayed_work work;
	struct workqueue_struct *wqueue;
	struct device *dev;
	struct wake_lock wl;
	atomic_t running;
	atomic_t pre_state;
	//struct input_dev *input;

	//struct pinctrl * key_pinctrl;
	//struct pinctrl_state * set_state;
    spinlock_t lock;
    int enable;
};
static struct cable_data *_cdata;
//static int cable_gpio = -1;//use gpio_11 for test, USE gpio_131 for DVT
static DEFINE_MUTEX(sem);
#define CABLE_WAKELOCK_HOLD_TIME 5000


static char *cmdline_find_option(char *str)
{
    extern char *saved_command_line;
    return strstr(saved_command_line, str);
}

int modify_rf_cable_smem_info(uint32 status)
{
	project_info_desc = smem_find(SMEM_PROJECT_INFO,
				sizeof(struct project_info),
				0,
				SMEM_ANY_HOST_FLAG);

	if (IS_ERR_OR_NULL(project_info_desc))
		pr_err("%s: get project_info failure\n",__func__);
	else
	{
	    project_info_desc->rf_v3 = status;
		pr_err("%s: rf_cable: %d\n",__func__, project_info_desc->rf_v3);
	}
	return 0;
}

uint32 get_hw_version(void)
{
	project_info_desc = smem_find(SMEM_PROJECT_INFO,
				sizeof(struct project_info),
				0,
				SMEM_ANY_HOST_FLAG);

	if (IS_ERR_OR_NULL(project_info_desc))
		/* Retry for old fw version */
		project_info_desc = smem_find(SMEM_PROJECT_INFO,
					sizeof(struct project_info) -
					sizeof(uint32), 0,
					SMEM_ANY_HOST_FLAG);

	if (IS_ERR_OR_NULL(project_info_desc))
		pr_err("%s: get project_info failure\n",__func__);
	else
	{
		pr_info("%s: hw version: %d\n",__func__, project_info_desc->hw_version);
		return project_info_desc->hw_version;
	}
	return 0;
}

extern void op_restart_modem(void);
static void rf_cable_work(struct work_struct *work)
{
    unsigned long flags;
    //mutex_lock(&sem);
    spin_lock_irqsave(&_cdata->lock, flags);
    disable_irq_nosync(_cdata->irq);
    spin_unlock_irqrestore(&_cdata->lock, flags);
    if (atomic_read(&_cdata->running) == 0)
    {
        atomic_set(&_cdata->running,1);
        mdelay(300);//debounce
        if (atomic_read(&_cdata->pre_state) != gpio_get_value(_cdata->cable_gpio))
        {
            atomic_set(&_cdata->pre_state, gpio_get_value(_cdata->cable_gpio));
            modify_rf_cable_smem_info(gpio_get_value(_cdata->cable_gpio));
            op_restart_modem();
        }
        //mdelay(2000);
    }
    spin_lock_irqsave(&_cdata->lock, flags);
    enable_irq(_cdata->irq);
    atomic_set(&_cdata->running,0);
    spin_unlock_irqrestore(&_cdata->lock, flags);
	//mutex_unlock(&sem);
}

irqreturn_t cable_interrupt(int irq, void *_dev)
{
    //mutex_lock(&sem);

        wake_lock_timeout(&_cdata->wl, msecs_to_jiffies(CABLE_WAKELOCK_HOLD_TIME));
        //schedule_work(&_cdata->work);
        //queue_work(_cdata->wqueue, &_cdata->work);
        queue_delayed_work(_cdata->wqueue, &_cdata->work, msecs_to_jiffies(1));
    //mutex_unlock(&sem);

	return IRQ_HANDLED;
}

#define PAGESIZE 512
static ssize_t rf_cable_proc_read_func(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
	int ret = 0;
	char page[PAGESIZE];
	ret = sprintf(page, "%d\n", _cdata->enable);
	ret = simple_read_from_buffer(user_buf, count, ppos, page, strlen(page));
	return ret;
}

static ssize_t rf_cable_proc_write_func(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    unsigned long flags;
    int enable = 0;
    #if 0
	char buf[10];
	if( copy_from_user(buf, buffer, count) ){
		pr_err("%s: read proc input error.\n", __func__);
		return count;
	}
	#else
    sscanf(buffer, "%d", &enable);
    pr_err("%s: cable_enable (%d)\n",__func__,enable);
	#endif
	if(enable != _cdata->enable)
	{
	    _cdata->enable = enable;
    	if(!_cdata->enable)
    	{
            spin_lock_irqsave(&_cdata->lock, flags);
            disable_irq_nosync(_cdata->irq);
            spin_unlock_irqrestore(&_cdata->lock, flags);
            pr_err("%s: pre_state (%d)\n",__func__,atomic_read(&_cdata->pre_state));
            if (atomic_read(&_cdata->pre_state) != 1)
            {
                atomic_set(&_cdata->pre_state, 1);
                modify_rf_cable_smem_info(1);
                op_restart_modem();
            }
        }
        else
        {
            spin_lock_irqsave(&_cdata->lock, flags);
            enable_irq(_cdata->irq);
            spin_unlock_irqrestore(&_cdata->lock, flags);
            pr_err("%s: pre_state (%d)\n",__func__,atomic_read(&_cdata->pre_state));
            if (atomic_read(&_cdata->pre_state) != gpio_get_value(_cdata->cable_gpio))
            {
                atomic_set(&_cdata->pre_state, gpio_get_value(_cdata->cable_gpio));
                modify_rf_cable_smem_info(gpio_get_value(_cdata->cable_gpio));
                op_restart_modem();
            }
        }
    }
	return count;
}
static const struct file_operations rf_enable_proc_fops = {
	.write = rf_cable_proc_write_func,
	.read =  rf_cable_proc_read_func,
	.open = simple_open,
	.owner = THIS_MODULE,
};
int create_rf_cable_procfs(void)
{
    int ret = 0;
    #if 0
    static struct proc_dir_entry *proc_rf_dir = NULL;
	proc_rf_dir = proc_mkdir("rf_config", NULL);
	if (!proc_rf_dir) {
		pr_err("%s: proc_rf_dir rf_config fail!\n",__func__);
		ret = -1;
	}
	if (!proc_create("enable", 0666, proc_rf_dir,&rf_enable_proc_fops)){
		pr_err("%s: proc_create enable fail!\n",__func__);
		ret = -1;
	}
	#else
	if (!proc_create("rf_cable_config", 0666, NULL,&rf_enable_proc_fops)){
		pr_err("%s: proc_create enable fail!\n",__func__);
		ret = -1;
	}
	#endif
	_cdata->enable = 1;
	return ret;
}

static int op_rf_request_named_gpio(		const char *label, int *gpio)
{
	struct device *dev = _cdata->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		*gpio = rc;
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_info(dev, "%s - gpio: %d\n", label, *gpio);
	return 0;
}

static int op_rf_cable_probe(struct platform_device *pdev)
{
    int rc = 0;
    struct device *dev = &pdev->dev;

    if (cmdline_find_option("ftm_mode"))
    {
        pr_err("%s: ftm_mode FOUND! use 1 always\n",__func__);
        modify_rf_cable_smem_info(1);
    }
    else
    {
        _cdata = kzalloc(sizeof(struct cable_data), GFP_KERNEL);
        if (!_cdata) {
    		pr_err("%s: failed to allocate memory\n",__func__);
    		rc = -ENOMEM;
    		goto exit;
    	}
    	
    	_cdata->dev = dev;
        dev_set_drvdata(dev, _cdata);
        
        rc = op_rf_request_named_gpio("rf,cable-gpio", &_cdata->cable_gpio);
        if(rc)
        {
            pr_err("%s: op_rf_request_named_gpio fail, DO NOT SUPPORT rf detect!!!\n",__func__);
            goto exit_gpio;
        }
        gpio_direction_input(_cdata->cable_gpio);
        
        _cdata->wqueue = create_singlethread_workqueue("op_rf_cable_wqueue");
        //INIT_WORK(&_cdata->work, rf_cable_work);
        INIT_DELAYED_WORK(&_cdata->work, rf_cable_work);
        _cdata->irq = gpio_to_irq(_cdata->cable_gpio);
        if (_cdata->irq < 0) {
        		pr_err("Unable to get irq number for GPIO %d, error %d\n",
        				_cdata->cable_gpio, _cdata->irq);
        		rc = _cdata->irq;
        		goto exit_gpio;
        }

        rc = request_irq(_cdata->irq, cable_interrupt,
			    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "op_rf_cable", _cdata);
    	if (rc) {
    		pr_err("could not request irq %d\n", _cdata->irq);
    		goto exit_gpio;
    	}
    	pr_err("requested irq %d\n", _cdata->irq);
    	enable_irq_wake(_cdata->irq);

    	wake_lock_init(&_cdata->wl, WAKE_LOCK_SUSPEND, "rf_cable_wake_lock");
    	spin_lock_init(&_cdata->lock);
    	atomic_set(&_cdata->running, 0);
    	atomic_set(&_cdata->pre_state, gpio_get_value(_cdata->cable_gpio));

        modify_rf_cable_smem_info(gpio_get_value(_cdata->cable_gpio));
        create_rf_cable_procfs();
    }
    pr_err("%s: probe ok!\n", __func__);
    return 0;

exit_gpio:
    kfree(_cdata);
exit:
    pr_err("%s: probe Fail!\n", __func__);

    return rc;
}
static struct of_device_id rf_of_match[] = {
	{ .compatible = "oneplus,rf_cable", },
	{}
};
MODULE_DEVICE_TABLE(of, rf_of_match);

static struct platform_driver op_rf_cable_driver = {
	.driver = {
		.name		= "op_rf_cable",
		.owner		= THIS_MODULE,
		.of_match_table = rf_of_match,
	},
	.probe = op_rf_cable_probe,
};

static int __init op_rf_cable_init(void)
{
    int ret;

    ret = platform_driver_register(&op_rf_cable_driver);
    if (ret)
        pr_err("op_rf_cable: platform_driver_register failed: %d\n", ret);

    return ret;
}

MODULE_LICENSE("GPL v2");
subsys_initcall(op_rf_cable_init);
