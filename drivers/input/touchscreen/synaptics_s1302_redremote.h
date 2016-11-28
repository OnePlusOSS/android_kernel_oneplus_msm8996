#ifndef _SYNAPTICS_REDREMOTE_H_
#define _SYNAPTICS_REDREMOTE_H_
struct remotepanel_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *kpd;
	struct mutex *pmutex;
	int irq_gpio;
	unsigned int irq;
	int *enable_remote;
};
struct remotepanel_data *remote_alloc_panel_data_s1302(void);
int register_remote_device_s1302(struct remotepanel_data *pdata);
void unregister_remote_device_s1302(void);
#endif