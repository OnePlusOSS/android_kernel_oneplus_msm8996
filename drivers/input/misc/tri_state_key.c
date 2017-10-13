/*
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#include <linux/regulator/consumer.h>

#include <linux/timer.h>

#define DRV_NAME	"tri-state-key"

/*
	        KEY1(GPIO1)	KEY2(GPIO92)
1脚和4脚连接	0	            1         | MUTE
2脚和5脚连接	1	            1         | Do Not Disturb
4脚和3脚连接	1	            0         | Normal

*/
typedef enum {
    MODE_UNKNOWN,
	MODE_MUTE,
	MODE_DO_NOT_DISTURB,
	MODE_NORMAL,
	MODE_MAX_NUM
} tri_mode_t;

struct switch_dev_data {
	//tri_mode_t last_type;
	//tri_mode_t mode_type;
	//int switch_enable;
	int irq_key3;
	int irq_key2;
	int irq_key1;
	int key1_gpio;//key1 gpio34
	int key2_gpio;//key2 gpio77
	int key3_gpio;

	struct regulator *vdd_io;
	//bool power_enabled;

	struct work_struct work;
	struct switch_dev sdev;
	struct device *dev;
	//struct input_dev *input;

	struct timer_list s_timer;
	struct pinctrl * key_pinctrl;
	struct pinctrl_state * set_state;

};

static struct switch_dev_data *switch_data;
static DEFINE_MUTEX(sem);
#if 0
static int set_gpio_by_pinctrl(void)
{
    printk(KERN_ERR "tristate_key set_gpio_by_pinctrl. \n");
    return pinctrl_select_state(switch_data->key_pinctrl, switch_data->set_state);
}
#endif
static void switch_dev_work(struct work_struct *work)
{

	int key1,key2,key3;
	//pr_err("%s  gpio_get_value(%d)=%d\n",__func__,switch_data->key1_gpio,gpio_get_value(switch_data->key1_gpio));
	//pr_err("%s  gpio_get_value(%d)=%d\n",__func__,switch_data->key2_gpio,gpio_get_value(switch_data->key2_gpio));
	//pr_err("%s  gpio_get_value(%d)=%d\n",__func__,switch_data->key3_gpio,gpio_get_value(switch_data->key3_gpio));

      mutex_lock(&sem);
	key1=gpio_get_value(switch_data->key1_gpio);
	key2=gpio_get_value(switch_data->key2_gpio);
	key3=gpio_get_value(switch_data->key3_gpio);

	if(((key1==0)&&(key2==1)&&(key3==1))||((key1==1)&&(key2==0)&&(key3==1))||((key1==1)&&(key2==1)&&(key3==0)))
	{
		//gpio_set_value(switch_data->key3_gpio,0);
		//pr_err("%s  gpio_22get_value(%d)=%d\n",__func__,switch_data->key3_gpio,gpio_get_value(switch_data->key3_gpio));
		if(!key2)
		{
		    switch_set_state(&switch_data->sdev, MODE_DO_NOT_DISTURB);
		}

		if(!key3)
		{
		    switch_set_state(&switch_data->sdev, MODE_NORMAL);

		}

		if(!key1)
		{
		    switch_set_state(&switch_data->sdev, MODE_MUTE);
		}

        printk(KERN_ERR "%s ,tristatekey set to state(%d) \n",__func__,switch_data->sdev.state);
	}
	mutex_unlock(&sem);
}
irqreturn_t switch_dev_interrupt(int irq, void *_dev)
{
//printk("%s\n",__func__);
    schedule_work(&switch_data->work);

		return IRQ_HANDLED;
}

static void timer_handle(unsigned long arg)
{
    //mod_timer(&s_timer, jiffies + HZ);
  //  if(set_gpio_by_pinctrl() < 0)
   //      printk(KERN_ERR "tristate_key set_gpio_by_pinctrl FAILD!!!. \n");
    schedule_work(&switch_data->work);
    //del_timer(&switch_data->s_timer);

    //printk(KERN_ERR "tristate_key set gpio77 timer. \n");
}

/* //no need cause switch_class.c state_show()
static ssize_t switch_dev_print_state(struct switch_dev *sdev, char *buf)
{
	tri_mode_t state;
		state = switch_data->mode_type;

	if (state)
		return sprintf(buf, "%d\n", state);
	return -1;
}
*/

#ifdef CONFIG_OF
static int switch_dev_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;

	node = dev->of_node;
	if (!node)
		return -EINVAL;

	switch_data->key3_gpio= of_get_named_gpio(node, "tristate,gpio_key3", 0);
	if ((!gpio_is_valid(switch_data->key3_gpio)))
		return -EINVAL;
	pr_err("switch_data->key3_gpio=%d \n", switch_data->key3_gpio);

	switch_data->key2_gpio= of_get_named_gpio(node, "tristate,gpio_key2", 0);
	if ((!gpio_is_valid(switch_data->key2_gpio)))
		return -EINVAL;
	pr_err("switch_data->key2_gpio=%d \n", switch_data->key2_gpio);
//printk("%s, key2 gpio:%d \n", __func__, switch_data->key2_gpio);

	switch_data->key1_gpio= of_get_named_gpio(node, "tristate,gpio_key1", 0);
	if ((!gpio_is_valid(switch_data->key1_gpio)))
		return -EINVAL;
	pr_err("switch_data->key1_gpio=%d \n", switch_data->key1_gpio);
//printk("%s, key1 gpio:%d \n", __func__, switch_data->key1_gpio);
	return 0;
}

static struct of_device_id tristate_dev_of_match[] = {
	{ .compatible = "oneplus,tri-state-key", },
	{ },
};
MODULE_DEVICE_TABLE(of, tristate_dev_of_match);

#else

static inline int
switch_dev_get_devtree_pdata(struct device *dev)
{
	return 0;
}
#endif
/*
#define SUPPLY_1V8		1800000UL
#define SUPPLY_IO_MIN		SUPPLY_1V8
#define SUPPLY_IO_MAX		SUPPLY_1V8
#define SUPPLY_IO_REQ_CURRENT	6000U

int tristate_regulator_release(void)
{



	if (switch_data->vdd_io != NULL) {
		regulator_put(switch_data->vdd_io);
		switch_data->vdd_io = NULL;
	}

	switch_data->power_enabled = false;

	return 0;
}

int tristate_regulator_set(bool enable)
{
	int error = 0;

	if (switch_data->vdd_io == NULL) {
		dev_err(switch_data->dev,
			"Regulators not set\n");
			return -EINVAL;
	}

	if (enable) {
		dev_dbg(switch_data->dev, "%s on\n", __func__);

		regulator_set_optimum_mode(switch_data->vdd_io,
					SUPPLY_IO_REQ_CURRENT);

		error = (regulator_is_enabled(switch_data->vdd_io) == 0) ?
					regulator_enable(switch_data->vdd_io) : 0;

		if (error) {
			dev_err(switch_data->dev,
				"Regulator vdd_io enable failed, error=%d\n",
				error);
			goto out_err;
		}

	} else {
		dev_dbg(switch_data->dev, "%s off\n", __func__);

		error = (switch_data->power_enabled &&
			regulator_is_enabled(switch_data->vdd_io) > 0) ?
				 regulator_disable(switch_data->vdd_io) : 0;

		if (error) {
			dev_err(switch_data->dev,
				"Regulator vdd_io disable failed, error=%d\n",
				 error);
			goto out_err;
		}

	}
    switch_data->power_enabled = enable;
	return 0;

out_err:
	tristate_regulator_release();
	return error;
}

static int tristate_supply_init(void)
{
	int error = 0;

	switch_data->vdd_io = regulator_get(switch_data->dev, "vdd_io");
	if (IS_ERR(switch_data->vdd_io)) {
		error = PTR_ERR(switch_data->vdd_io);
		dev_err(switch_data->dev,
			"Regulator get failed, vdd_io, error=%d\n", error);
		goto err;
	}

	if (regulator_count_voltages(switch_data->vdd_io) > 0) {
		error = regulator_set_voltage(switch_data->vdd_io,
						SUPPLY_IO_MIN, SUPPLY_IO_MAX);
		if (error) {
			dev_err(switch_data->dev,
				"regulator set(io) failed, error=%d\n", error);
			goto err;
		}
	}
	if (error) {
		dev_err(switch_data->dev,
			"regulator configuration failed.\n");
		goto err;
	}

	error = tristate_regulator_set(true);
	if (error) {
		dev_err(switch_data->dev,
			"regulator enable failed.\n");
		goto err;
	}

err:
	return error;
}

*/

static int tristate_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	int error=0;

	//void __iomem *cfg_reg;

        switch_data = kzalloc(sizeof(struct switch_dev_data), GFP_KERNEL);
        switch_data->dev = dev;
        #if 0
	    switch_data->key_pinctrl = devm_pinctrl_get(switch_data->dev);
         if (IS_ERR_OR_NULL(switch_data->key_pinctrl)) {
		        dev_err(switch_data->dev, "Failed to get pinctrl \n");
		        goto err_switch_dev_register;
	     }
         switch_data->set_state =pinctrl_lookup_state(switch_data->key_pinctrl,"pmx_tri_state_key_active");
         if (IS_ERR_OR_NULL(switch_data->set_state)) {
		        dev_err(switch_data->dev, "Failed to lookup_state \n");
		        goto err_switch_dev_register;
	     }

	     set_gpio_by_pinctrl();
		#endif
        //switch_data->last_type = MODE_UNKNOWN;

        //tristate_supply_init();
		error = switch_dev_get_devtree_pdata(dev);
		if (error) {
			dev_err(dev, "parse device tree fail!!!\n");
			goto err_switch_dev_register;
		}

		//config irq gpio and request irq
	switch_data->irq_key1 = gpio_to_irq(switch_data->key1_gpio);
       if (switch_data->irq_key1 <= 0)
       {
            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, switch_data->irq_key1, switch_data->key1_gpio);
            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key1_gpio,"tristate_key1-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key1_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}

			error = request_irq(switch_data->irq_key1, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key1", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key1);

        		switch_data->irq_key1 = -EINVAL;
        		goto err_request_irq;
            }
       }
       //config irq gpio and request irq
	 switch_data->irq_key2 = gpio_to_irq(switch_data->key2_gpio);
       if (switch_data->irq_key2 <= 0)
       {
            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, switch_data->irq_key2, switch_data->key2_gpio);
            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key2_gpio,"tristate_key2-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key2_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}

			error = request_irq(switch_data->irq_key2, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key2", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key2);

        		switch_data->irq_key2 = -EINVAL;
        		goto err_request_irq;
            }

       }

	   switch_data->irq_key3 = gpio_to_irq(switch_data->key3_gpio);
	   if (switch_data->irq_key3 <= 0)
	   {
	            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, \
	            switch_data->irq_key3, switch_data->key3_gpio);
	            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key3_gpio,"tristate_key3-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key3_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}


			error = request_irq(switch_data->irq_key3, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key3", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key3);

        		switch_data->irq_key3 = -EINVAL;
        		goto err_request_irq;
            }

       }


        INIT_WORK(&switch_data->work, switch_dev_work);

        init_timer(&switch_data->s_timer);
        switch_data->s_timer.function = &timer_handle;
        switch_data->s_timer.expires = jiffies + 5*HZ;

        add_timer(&switch_data->s_timer);

        enable_irq_wake(switch_data->irq_key1);
        enable_irq_wake(switch_data->irq_key2);
	    enable_irq_wake(switch_data->irq_key3);


        switch_data->sdev.name = DRV_NAME;
       	error = switch_dev_register(&switch_data->sdev);
	    if (error < 0)
		    goto err_request_gpio;
		 //set_gpio_by_pinctrl();
        //report the first switch
        //switch_dev_work(&switch_data->work);
        return 0;


err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->key2_gpio);
	gpio_free(switch_data->key1_gpio);
	gpio_free(switch_data->key3_gpio);
err_switch_dev_register:
	kfree(switch_data);

	return error;
}

static int tristate_dev_remove(struct platform_device *pdev)
{
printk("%s\n",__func__);
	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->key1_gpio);
	gpio_free(switch_data->key2_gpio);
	gpio_free(switch_data->key3_gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver tristate_dev_driver = {
	.probe	= tristate_dev_probe,
	.remove	= tristate_dev_remove,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tristate_dev_of_match),
	},
};
module_platform_driver(tristate_dev_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("switch Profiles by this triple key driver");

