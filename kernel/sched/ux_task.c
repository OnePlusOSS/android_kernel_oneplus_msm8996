#include <asm/atomic.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include "sched.h"


extern char * cgroup_get_cpuctl_path(struct task_struct *tsk, char *buf);
extern void reset_cpu_max_freq(void);
unsigned int ux_notify = 1;

static int ux_notify_show(char *buf, const struct kernel_param *kp)
{
    return snprintf(buf, PAGE_SIZE,	"%u", ux_notify);
}

static int ux_notify_store(const char *buf, const struct kernel_param *kp)
{
    unsigned int val;

    if (sscanf(buf, "%u\n", &val) <= 0)
        return -EINVAL;
    ux_notify = val;
    if(ux_notify == 0)
        reset_cpu_max_freq();
    return 0;
}

static const struct kernel_param_ops param_ops_ux_notify = {
    .set = ux_notify_store,
    .get = ux_notify_show,
};

module_param_cb(ux_notify, &param_ops_ux_notify, NULL, 0644);

bool oneplus_is_background(struct task_struct *t)
{
    char *buf;
    char *path;
    int val = 0;

    if(!strcmp(t->group_leader->comm, "surfaceflinger") || !memcmp(t->group_leader->comm, "swapper", 7))
        return 1;
    buf = kmalloc(PATH_MAX, GFP_KERNEL);
    path = cgroup_get_cpuctl_path(t, buf);

    if(!strcmp(path, "/bg_non_interactive"))
        val = 1;
    else
        val = 0;
    kfree(buf);
    return val;
}
