#include <linux/module.h>
#include <linux/kernel.h>    
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define DRIVER_AUTHOR "flar2 (asegaert at gmail.com)"
#define DRIVER_DESCRIPTION "sweep2sleep driver"
#define DRIVER_VERSION "4.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

//sweep2sleep
#define S2S_PWRKEY_DUR          60
#define S2S_Y_MAX             	1920
#define S2S_Y_LIMIT             S2S_Y_MAX-180
#define SWEEP_RIGHT		0x01
#define SWEEP_LEFT		0x02
#define VIB_STRENGTH		20

extern void set_vibrate(int value);

// 1=sweep right, 2=sweep left, 3=both
static int s2s_switch = 2;

static int touch_x = 0, touch_y = 0, firstx = 0;
static bool touch_x_called = false, touch_y_called = false;
static bool scr_on_touch = false, barrier[2] = {false, false};
static bool exec_count = true;
static struct input_dev * sweep2sleep_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static struct workqueue_struct *s2s_input_wq;
static struct work_struct s2s_input_work;
static int vib_strength = VIB_STRENGTH;

void sweep2sleep_setdev(struct input_dev * input_device) {
	sweep2sleep_pwrdev = input_device;
	return;
}

/* PowerKey work func */
static void sweep2sleep_presspwr(struct work_struct * sweep2sleep_presspwr_work) {

	if (!mutex_trylock(&pwrkeyworklock))
                return;
	input_event(sweep2sleep_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(sweep2sleep_pwrdev, EV_SYN, 0, 0);
	msleep(S2S_PWRKEY_DUR);
	input_event(sweep2sleep_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(sweep2sleep_pwrdev, EV_SYN, 0, 0);
	msleep(S2S_PWRKEY_DUR);
        mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(sweep2sleep_presspwr_work, sweep2sleep_presspwr);

/* PowerKey trigger */
static void sweep2sleep_pwrtrigger(void) {
	set_vibrate(vib_strength);
	schedule_work(&sweep2sleep_presspwr_work);
        return;
}

/* reset on finger release */
static void sweep2sleep_reset(void) {
	exec_count = true;
	barrier[0] = false;
	barrier[1] = false;
	firstx = 0;
	scr_on_touch = false;
}

/* Sweep2sleep main function */
static void detect_sweep2sleep(int x, int y, bool st)
{
        int prevx = 0, nextx = 0;
        bool single_touch = st;

	if (firstx == 0)
		firstx = x;

	if (s2s_switch > 3)
		s2s_switch = 3;

	//left->right
	if (single_touch && firstx < 810 && (s2s_switch & SWEEP_RIGHT)) {
		scr_on_touch=true;
		prevx = firstx;
		nextx = prevx + 180;
		if ((barrier[0] == true) ||
		   ((x > prevx) &&
		    (x < nextx) &&
		    (y > S2S_Y_LIMIT))) {
			prevx = nextx;
			nextx += 200;
			barrier[0] = true;
			if ((barrier[1] == true) ||
			   ((x > prevx) &&
			    (x < nextx) &&
			    (y > S2S_Y_LIMIT))) {
				prevx = nextx;
				barrier[1] = true;
				if ((x > prevx) &&
				    (y > S2S_Y_LIMIT)) {
					if (x > (nextx + 180)) {
						if (exec_count) {
							sweep2sleep_pwrtrigger();
							exec_count = false;
						}
					}
				}
			}
		}
	//right->left
	} else if (firstx >= 180 && (s2s_switch & SWEEP_LEFT)) {
		scr_on_touch=true;
		prevx = firstx;
		nextx = prevx - 180;
		if ((barrier[0] == true) ||
		   ((x < prevx) &&
		    (x > nextx) &&
		    (y > S2S_Y_LIMIT))) {
			prevx = nextx;
			nextx -= 200;
			barrier[0] = true;
			if ((barrier[1] == true) ||
			   ((x < prevx) &&
			    (x > nextx) &&
			    (y > S2S_Y_LIMIT))) {
				prevx = nextx;
				barrier[1] = true;
				if ((x < prevx) &&
				    (y > S2S_Y_LIMIT)) {
					if (x < (nextx - 180)) {
						if (exec_count) {
							sweep2sleep_pwrtrigger();
							exec_count = false;
						}
					}
				}
			}
		}
	}
}


static void s2s_input_callback(struct work_struct *unused) {

	detect_sweep2sleep(touch_x, touch_y, true);

	return;
}

static void s2s_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {

	if (code == ABS_MT_SLOT) {
		sweep2sleep_reset();
		return;
	}

	if (code == ABS_MT_TRACKING_ID && value == -1) {
		sweep2sleep_reset();
		return;
	}

	if (code == ABS_MT_POSITION_X) {
		touch_x = value;
		touch_x_called = true;
	}

	if (code == ABS_MT_POSITION_Y) {
		touch_y = value;
		touch_y_called = true;
	}

	if (touch_x_called && touch_y_called) {
		touch_x_called = false;
		touch_y_called = false;
		queue_work_on(0, s2s_input_wq, &s2s_input_work);
	}
}

static int input_dev_filter(struct input_dev *dev) {
	if (strstr(dev->name, "synaptics,s3320")) {
		return 0;
	} else {
		return 1;
	}
}

static int s2s_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "s2s";

	error = input_register_handle(handle);

	error = input_open_device(handle);

	return 0;

}

static void s2s_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id s2s_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler s2s_input_handler = {
	.event		= s2s_input_event,
	.connect	= s2s_input_connect,
	.disconnect	= s2s_input_disconnect,
	.name		= "s2s_inputreq",
	.id_table	= s2s_ids,
};

static ssize_t sweep2sleep_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", s2s_switch);
}

static ssize_t sweep2sleep_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 3)
		input = 0;				

	s2s_switch = input;			
	
	return count;
}

static DEVICE_ATTR(sweep2sleep, (S_IWUSR|S_IRUGO),
	sweep2sleep_show, sweep2sleep_dump);

static ssize_t vib_strength_show(struct device *dev,
		 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vib_strength);
}

static ssize_t vib_strength_dump(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 90)
		input = 20;				

	vib_strength = input;			
	
	return count;
}

static DEVICE_ATTR(vib_strength, (S_IWUSR|S_IRUGO),
	vib_strength_show, vib_strength_dump);


static struct kobject *sweep2sleep_kobj; 

static int __init sweep2sleep_init(void)
{
	int rc = 0;

	s2s_input_wq = create_workqueue("s2s_iwq");
	if (!s2s_input_wq) {
		pr_err("%s: Failed to create workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&s2s_input_work, s2s_input_callback);

	rc = input_register_handler(&s2s_input_handler);
	if (rc)
		pr_err("%s: Failed to register s2s_input_handler\n", __func__);

	sweep2sleep_kobj = kobject_create_and_add("sweep2sleep", NULL) ;
	if (sweep2sleep_kobj == NULL) {
		pr_warn("%s: sweep2sleep_kobj failed\n", __func__);
	}

	rc = sysfs_create_file(sweep2sleep_kobj, &dev_attr_sweep2sleep.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for sweep2sleep\n", __func__);

	rc = sysfs_create_file(sweep2sleep_kobj, &dev_attr_vib_strength.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for vib_strength\n", __func__);

	return 0;
}

static void __exit sweep2sleep_exit(void)
{
	kobject_del(sweep2sleep_kobj); 
	input_unregister_handler(&s2s_input_handler);
	destroy_workqueue(s2s_input_wq);
	input_unregister_device(sweep2sleep_pwrdev);
	input_free_device(sweep2sleep_pwrdev);

	return;
}

module_init(sweep2sleep_init);
module_exit(sweep2sleep_exit);
