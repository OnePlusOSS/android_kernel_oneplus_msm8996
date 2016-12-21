/*
 * Copyright (c) 2015-2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/fb.h>
#include <uapi/linux/msm-core-interface.h>

#include "crpl.h"

/* define per cpu variable and state spin lock */
static DEFINE_PER_CPU(struct crpl_data, cpu_state);
static DEFINE_SPINLOCK(state_lock);
struct timer_list timer;
struct task_struct *crpl_detect_thread;
struct task_struct *crpl_rq_thread;

/* define rq list */
LIST_HEAD(rq_list);
LIST_HEAD(rq_native_list);
static DEFINE_SPINLOCK(rq_lock);

int _crpl_data_offset[__CRPL_DATA_MAX] = {
	[CRPL_DATA_INITED] = offsetof(struct crpl_data, inited),
	[CRPL_DATA_CPU] = offsetof(struct crpl_data, cpu),
	[CRPL_DATA_SIB] = offsetof(struct crpl_data, sib),
	[CRPL_DATA_FIRST_CPU] = offsetof(struct crpl_data, first_cpu),
	[CRPL_DATA_LRU] = offsetof(struct crpl_data, lru),
	[CRPL_DATA_USE_CUSTOMIZED_TABLE] = offsetof(struct crpl_data, use_customized_table),
	[CRPL_DATA_TEMP] = offsetof(struct crpl_data, temp),
	[CRPL_DATA_CORE_NUMS] = offsetof(struct crpl_data, core_nums),
	[CRPL_DATA_FREQ] = offsetof(struct crpl_data, freq),
	[CRPL_DATA_POWER_BASE] = offsetof(struct crpl_data, power_base),
	[CRPL_DATA_POWER_LIMIT] = offsetof(struct crpl_data, power_limit),
	[CRPL_DATA_PENDING] = offsetof(struct crpl_data, pending),
	[CRPL_DATA_CRPL_THREAD] = offsetof(struct crpl_data, crpl_thread),
	[CRPL_DATA_FREQ_LIMIT] = offsetof(struct crpl_data, freq_limit),
	[CRPL_DATA_FREQ_LVS] = offsetof(struct crpl_data, freq_lvs),
	[CRPL_DATA_FREQ_TBL] = offsetof(struct crpl_data, freq_tbl),
	[CRPL_DATA_PCOST_LATEST] = offsetof(struct crpl_data, pcost_latest),
	[CRPL_DATA_TBL] = offsetof(struct crpl_data, tbl),
	[CRPL_DATA_TBL_VERSION] = offsetof(struct crpl_data, tbl_version),
	[CRPL_DATA_DEBOUNCE_ENABLE] = offsetof(struct crpl_data, debounce_enable),
	[CRPL_DATA_LAST_UPDATE_TIME] = offsetof(struct crpl_data, last_update_time),
	[CRPL_DATA_LVS] = offsetof(struct crpl_data, lvs),
	[CRPL_DATA_SCALE_MAX_TEMP] = offsetof(struct crpl_data, scale_max_temp),
	[CRPL_DATA_KOBJ] = offsetof(struct crpl_data, kobj)
};
gen_type_offset_impl(crpl_data);

int _crpl_rq_offset[__CRPL_RQ_MAX] = {
	[CRPL_RQ_COMM] = offsetof(crpl_rq, comm),
	[CRPL_RQ_LVS] = offsetof(crpl_rq, lvs),
	[CRPL_RQ_THRES] = offsetof(crpl_rq, thres),
	[CRPL_RQ_RATIO] = offsetof(crpl_rq, ratio),
	[CRPL_RQ_KICK_OFF] = offsetof(crpl_rq, kick_off),
	[CRPL_RQ_PERIOD] = offsetof(crpl_rq, period),
	[CRPL_RQ_RELEASE_PREPARE] = offsetof(crpl_rq, release_prepare),
	[CRPL_RQ_TIMER] = offsetof(crpl_rq, timer),
	[CRPL_RQ_NODE] = offsetof(crpl_rq, node)
};
gen_type_offset_impl(crpl_rq);

void __ref crpl_state_spin_lock_irqsave(unsigned long *flags)
{
	unsigned long tmp = *flags;
	spin_lock_irqsave(&state_lock, tmp);
	*flags = tmp;
}

void __ref crpl_state_spin_unlock_irqrestore(unsigned long *flags)
{
	unsigned long tmp = *flags;
	spin_unlock_irqrestore(&state_lock, tmp);
	*flags = tmp;
}

struct crpl_data* __ref crpl_get_crpl_data(int cpu)
{
	return &per_cpu(cpu_state, cpu);
}

int __ref crpl_get_temp_point(void)
{
	return TEMP_DATA_POINTS;
}

/*
 * sysfs definitions
 *
 * parameters:
 *
 * crpl: crpl detail status
 * lvs: crpl power limit level (%)
 * scale_max_temp: scale power cost to max temp
 * debounce_time: debounce validate time, default 100 ms
 * debounce_enable: debounce validate enable or not
 * use_customized_table: using customized table or not
 * crpl_dump: dump crpl power table
 * cpus: show cpu status (online/offline)
 * table_version: power table version
 */
gen_crpl_sysfs_rw_impl(crpl);
gen_crpl_sysfs_rw_impl(lvs);
gen_crpl_sysfs_rw_impl(scale_max_temp);
gen_crpl_sysfs_rw_impl(debounce_time);
gen_crpl_sysfs_rw_impl(debounce_enable);
gen_crpl_sysfs_rw_impl(use_customized_table);
gen_crpl_sysfs_rw_impl(scale_load);

/* rq related sysfs nodes */
gen_crpl_sysfs_rw_impl(rq_switch_on);
gen_crpl_sysfs_rw_impl(rq_bat_thres);
gen_crpl_sysfs_rw_impl(rq_usb_detect);
gen_crpl_sysfs_ro_impl(rq_dump);
gen_crpl_sysfs_ro_impl(rq_statistic);
gen_crpl_sysfs_wo_impl(rq_acquire);
gen_crpl_sysfs_wo_impl(rq_release);
gen_crpl_sysfs_wo_impl(rq_update);
gen_crpl_sysfs_wo_impl(rq_cleanup);

gen_crpl_sysfs_ro_impl(crpl_dump);
gen_crpl_sysfs_ro_impl(cpus);
gen_crpl_sysfs_ro_impl(table_version);

struct crpl_attr {
	struct attribute attr;
	ssize_t (*show)(struct crpl_data *, char *);
	ssize_t (*store)(struct crpl_data *, const char *, size_t count);
};

#define crpl_attr_ro(_name)		\
static struct crpl_attr _name =	\
__ATTR(_name, 0444, show_##_name, NULL)

#define crpl_attr_rw(_name)			\
static struct crpl_attr _name =		\
__ATTR(_name, 0664, show_##_name, store_##_name)

#define crpl_attr_wo(_name)		\
static struct crpl_attr _name =	\
__ATTR(_name, 0220, NULL, store_##_name)

crpl_attr_rw(crpl);
crpl_attr_rw(lvs);
crpl_attr_rw(scale_max_temp);
crpl_attr_rw(debounce_time);
crpl_attr_rw(debounce_enable);
crpl_attr_rw(use_customized_table);
crpl_attr_rw(scale_load);

/* rq related sysfs nodes */
crpl_attr_rw(rq_switch_on);
crpl_attr_rw(rq_bat_thres);
crpl_attr_rw(rq_usb_detect);
crpl_attr_ro(rq_dump);
crpl_attr_ro(rq_statistic);
crpl_attr_wo(rq_acquire);
crpl_attr_wo(rq_release);
crpl_attr_wo(rq_update);
crpl_attr_wo(rq_cleanup);

crpl_attr_ro(crpl_dump);
crpl_attr_ro(cpus);
crpl_attr_ro(table_version);

static struct attribute *default_attrs[] = {
	&lvs.attr,
	&scale_max_temp.attr,
	&debounce_enable.attr,
	&debounce_time.attr,
	&use_customized_table.attr,
	&scale_load.attr,

	/* rq related sysfs nodes */
	&rq_switch_on.attr,
	&rq_bat_thres.attr,
	&rq_usb_detect.attr,
	&rq_dump.attr,
	&rq_statistic.attr,
	&rq_acquire.attr,
	&rq_release.attr,
	&rq_update.attr,
	&rq_cleanup.attr,

	&crpl.attr,
	&crpl_dump.attr,
	&cpus.attr,
	&table_version.attr,
	NULL
};

#define to_crpl_data(k) container_of(k, struct crpl_data, kobj)
#define to_attr(a) container_of(a, struct crpl_attr, attr)
static ssize_t show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	struct crpl_data *data = to_crpl_data(kobj);
	struct crpl_attr *cattr = to_attr(attr);
	ssize_t ret = -EIO;

	if (cattr->show)
		ret = cattr->show(data, buf);

	return ret;
}

static ssize_t store(struct kobject *kobj, struct attribute *attr,
		     const char *buf, size_t count)
{
	struct crpl_data *data = to_crpl_data(kobj);
	struct crpl_attr *cattr = to_attr(attr);
	ssize_t ret = -EIO;

	if (cattr->store)
		ret = cattr->store(data, buf, count);

	return ret;
}

static const struct sysfs_ops sysfs_ops = {
	.show	= show,
	.store	= store,
};

static struct kobj_type ktype_crpl = {
	.sysfs_ops	= &sysfs_ops,
	.default_attrs	= default_attrs,
};

/* rq part */
struct list_head *crpl_get_rq_head(void)
{
	return &rq_list;
}

struct list_head *crpl_get_rq_native_head(void)
{
	return &rq_native_list;
}

void crpl_rq_spin_lock_irqsave(unsigned long *flags)
{
	unsigned long tmp;
	spin_lock_irqsave(&rq_lock, tmp);
	*flags = tmp;
}

void crpl_rq_spin_unlock_irqrestore(unsigned long *flags)
{
	unsigned long tmp = *flags;
	spin_unlock_irqrestore(&rq_lock, tmp);
}

void crpl_list_add(struct list_head *_new, struct list_head *_head)
{
	list_add(_new, _head);
}

void crpl_list_add_tail(struct list_head *_new, struct list_head *_head)
{
	list_add_tail(_new, _head);
}

#define to_cpu_rq(k) container_of(k, crpl_rq, node)
void crpl_list_del_and_free(struct list_head *node)
{
	crpl_rq *rq = to_cpu_rq(node);

	if (!crpl_list_rq_is_exist(rq->comm, crpl_get_rq_head())) {
		pr_err("rq cant found in list\n");
		return;
	}

	list_del(node);
	kfree(rq);
}

void crpl_list_del(struct list_head *node)
{
	list_del(node);
}

int crpl_list_empty(struct list_head *node)
{
	return list_empty(node);
}

crpl_rq *crpl_list_rq_is_exist(const char *key, struct list_head *head)
{
	crpl_rq *rq;

	list_for_each_entry(rq, head, node) {
		if (!strncmp(rq->comm, key, CRPL_RQ_COMM_LEN)) {
			/* found rq, inc rq ref */
			return rq;
		}
	}
	return NULL;
}

crpl_rq *__crpl_rq_instance(const char *settings, crpl_rq *_rq)
{
	crpl_rq* rq;
	bool external_rq = false;

	if (_rq) {
		rq = _rq;
		memset(rq, 0, sizeof(crpl_rq));
		external_rq = true;
	} else
		rq = kzalloc(sizeof(crpl_rq), GFP_KERNEL);

	if (rq) {
		int ret = 0;

		/* pre settings, all new request is lvs = 100 */
		rq->lvs[0] = 100;
		rq->lvs[1] = 100;

		/* limit the comm len to 31 to align crpl rq structure definition */
		ret = sscanf(settings, "%31s %u %u %u %u %u %u\n",
				rq->comm, (rq->thres+0), (rq->ratio+0), (rq->thres+1), (rq->ratio+1), &rq->kick_off, &rq->period);
		pr_info("new instance: comm: %s, lvs: %u:%u, thres: %u:%u, ratio: %u:%u, kick_off: %u, period: %u\n",
				rq->comm, rq->lvs[0], rq->lvs[1], rq->thres[0], rq->thres[1], rq->ratio[0], rq->ratio[1], rq->kick_off, rq->period);

		/* do config sanity check in case of any invalid rq to crash system */
		if (ret != 7) {
			pr_err("crpl request out of size\n");
			goto error_out;
		}

		/* check thres */
		if (rq->thres[0] < 10  || rq->thres[1] < 10 ||
			rq->thres[0] > 100 || rq->thres[1] > 100) {
			pr_err("crpl requset thres less than 10 or large than 100 is not allowed\n");
			goto error_out;
		}

		/* check ratio */
		if (rq->ratio[0] < 0   || rq->ratio[1] < 0 ||
			rq->ratio[0] > 100 || rq->ratio[1] > 100) {
			pr_err("crpl requset ratio less than 0 or large than 100 is not allowed\n");
			goto error_out;
		}

		/* check kick_off & period */
		if (rq->kick_off < 0 || rq->period < 0) {
			pr_err("crpl requset kick_off period less than 0 are not allowed\n");
			goto error_out;
		}

		/* return on succeed */
		return rq;
	}

	return NULL;

error_out:
	if (!external_rq)
		kfree(rq);
	return NULL;
}

crpl_rq *crpl_new_rq_instance(const char *settings)
{
	return __crpl_rq_instance(settings, NULL);
}

crpl_rq *crpl_prepare_rq_instance(const char *settings, crpl_rq *rq)
{
	return __crpl_rq_instance(settings, rq);
}

void crpl_rq_reset(void)
{
	/* remove rq and armed timer */
	int idx = 0;
	crpl_rq *rq, *next;

	/* native rq */
	list_for_each_entry_safe(rq, next, crpl_get_rq_native_head(), node) {
		if (rq && !rq->release_prepare) {
			rq->lvs[0] = 100;
			rq->lvs[1] = 100;
			if (crpl_rq_timer_pending(&rq->timer))
				crpl_rq_del_timer(&rq->timer);

			pr_info("crpl: reset native rq[%d]: comm: %s\n", idx, rq->comm);
			++idx;
		}
	}

	/* normal rq */
	list_for_each_entry_safe(rq, next, crpl_get_rq_head(), node) {
		if (rq && !rq->release_prepare) {
			rq->lvs[0] = 100;
			rq->lvs[1] = 100;
			if (crpl_rq_timer_pending(&rq->timer))
				crpl_rq_del_timer(&rq->timer);

			pr_info("crpl: clean normal rq[%d]: comm: %s\n", idx, rq->comm);
			crpl_list_del_and_free(&rq->node);
			++idx;
		}
	}
}

void crpl_rq_init(void)
{
	crpl_rq_init_core();
}

void crpl_rq_exit(void)
{
	crpl_rq_exit_core();
}

void crpl_rq_native_dump(void)
{
	unsigned long flags;
	crpl_rq *rq;
	int count = 0;
	int flag_on_rq_complete = 0;

	spin_lock_irqsave(&rq_lock, flags);
	/* native rq */
	pr_info("crpl: native rq dump:\n");
	list_for_each_entry(rq, crpl_get_rq_native_head(), node) {
		flag_on_rq_complete = 0;
		if ((rq->lvs[0] == rq->thres[0] && rq->lvs[1] == rq->thres[1]) ||
			(rq->lvs[0] == 100 && rq->lvs[1] == 100))
			flag_on_rq_complete = 1;

		pr_info("crpl: rq[%d]: comm: %s, lvs: %u:%u, thres: %u:%u, ratio: %u:%u, kick_off: %u, period: %u, expire: %llu, till_expire_ms: %llu\n",
						count, rq->comm,
						rq->lvs[0], rq->lvs[1],
						rq->thres[0], rq->thres[1],
						rq->ratio[0], rq->ratio[1],
						rq->kick_off, rq->period,
						flag_on_rq_complete? 0: crpl_rq_get_timer_expires(&rq->timer),
						flag_on_rq_complete? 0: (crpl_rq_get_timer_expires(&rq->timer)?
						(crpl_rq_get_timer_expires(&rq->timer) - jiffies) * 10: 0));
		++count;
	}

	/* normal rq */
	pr_info("crpl: normal rq dump:\n");
	list_for_each_entry(rq, crpl_get_rq_head(), node) {
		flag_on_rq_complete = 0;
		if ((rq->lvs[0] == rq->thres[0] && rq->lvs[1] == rq->thres[1]) ||
			(rq->lvs[0] == 100 && rq->lvs[1] == 100))
			flag_on_rq_complete = 1;

		pr_info("crpl: rq[%d]: comm: %s, lvs: %u:%u, thres: %u:%u, ratio: %u:%u, kick_off: %u, period: %u, expire: %llu, till_expire_ms: %llu\n",
						count, rq->comm,
						rq->lvs[0], rq->lvs[1],
						rq->thres[0], rq->thres[1],
						rq->ratio[0], rq->ratio[1],
						rq->kick_off, rq->period,
						flag_on_rq_complete? 0: crpl_rq_get_timer_expires(&rq->timer),
						flag_on_rq_complete? 0: (crpl_rq_get_timer_expires(&rq->timer)?
						(crpl_rq_get_timer_expires(&rq->timer) - jiffies) * 10: 0));
		++count;
	}
	spin_unlock_irqrestore(&rq_lock, flags);
}

ssize_t crpl_rq_userspace_dump(char *buf)
{
	unsigned long flags;
	crpl_rq *rq;
	int count = 0, msg_cnt = 0;
	/* kernel limit stack usage no more than 2048 bytes */
	char msg[PAGE_SIZE >> 2] = {0};
	int flag_on_rq_complete = 0;
	int dump_in_kmsg = 0;

	/*
	 * roughly calculate, one rq will cost about 120 bytes,
	 * 1k byte should be able to accommodate up to 8 rqs
	 */

	spin_lock_irqsave(&rq_lock, flags);

	/* native rq */
	if (msg_cnt > (((PAGE_SIZE >> 2) * 8 )/ 10))
		msg_cnt += sprintf(msg + msg_cnt, "native rq dump:\n");

	list_for_each_entry(rq, crpl_get_rq_native_head(), node) {
		/* rough check, if msg_cnt > 1k * 0.8, break */
		if (msg_cnt > (((PAGE_SIZE >> 2) * 8 )/ 10)) {
			msg_cnt += sprintf(msg + msg_cnt, "*** reach msg threshold, just break it, find more in kmsg ***\n");
			dump_in_kmsg = 1;
			break;
		}

		flag_on_rq_complete = 0;
		if ((rq->lvs[0] == rq->thres[0] && rq->lvs[1] == rq->thres[1]) ||
			(rq->lvs[0] == 100 && rq->lvs[1] == 100))
			flag_on_rq_complete = 1;

		msg_cnt += sprintf(msg + msg_cnt, "rq[%d]: comm: %s, lvs: %u:%u, thres: %u:%u, ratio: %u:%u, kick_off: %u, period: %u, expire: %llu, till_expire_ms: %llu\n",
						count, rq->comm,
						rq->lvs[0], rq->lvs[1],
						rq->thres[0], rq->thres[1],
						rq->ratio[0], rq->ratio[1],
						rq->kick_off, rq->period,
						flag_on_rq_complete? 0: crpl_rq_get_timer_expires(&rq->timer),
						flag_on_rq_complete? 0: (crpl_rq_get_timer_expires(&rq->timer)?
						(crpl_rq_get_timer_expires(&rq->timer) - jiffies) * 10: 0));
		++count;
	}

	/* normal rq */
	if (msg_cnt > (((PAGE_SIZE >> 2) * 8 )/ 10))
		msg_cnt += sprintf(msg + msg_cnt, "normal rq dump:\n");

	list_for_each_entry(rq, crpl_get_rq_head(), node) {
		/* rough check, if msg_cnt > 1k * 0.8, break */
		if (msg_cnt > (((PAGE_SIZE >> 2) * 8 )/ 10)) {
			msg_cnt += sprintf(msg + msg_cnt, "*** reach msg threshold, just break it, find more in kmsg ***\n");
			dump_in_kmsg = 1;
			break;
		}

		flag_on_rq_complete = 0;
		if ((rq->lvs[0] == rq->thres[0] && rq->lvs[1] == rq->thres[1]) ||
			(rq->lvs[0] == 100 && rq->lvs[1] == 100))
			flag_on_rq_complete = 1;

		msg_cnt += sprintf(msg + msg_cnt, "rq[%d]: comm: %s, lvs: %u:%u, thres: %u:%u, ratio: %u:%u, kick_off: %u, period: %u, expire: %llu, till_expire_ms: %llu\n",
						count, rq->comm,
						rq->lvs[0], rq->lvs[1],
						rq->thres[0], rq->thres[1],
						rq->ratio[0], rq->ratio[1],
						rq->kick_off, rq->period,
						flag_on_rq_complete? 0: crpl_rq_get_timer_expires(&rq->timer),
						flag_on_rq_complete? 0: (crpl_rq_get_timer_expires(&rq->timer)?
						(crpl_rq_get_timer_expires(&rq->timer) - jiffies) * 10: 0));
		++count;
	}
	spin_unlock_irqrestore(&rq_lock, flags);

	if (!count)
		msg_cnt = sprintf(msg + msg_cnt, "crpl no any rq registed\n");

	memcpy(buf, msg, msg_cnt);

	/* out of range to show message in user space, dump in kmsg */
	if (dump_in_kmsg)
		crpl_rq_native_dump();

	return msg_cnt;
}

void crpl_rq_thread_func_outer(unsigned int *m)
{
	crpl_rq *rq;

	list_for_each_entry(rq, crpl_get_rq_head(), node) {
		if (m[0] > rq->lvs[0])
			m[0] = rq->lvs[0];
		if (m[1] > rq->lvs[1])
			m[1] = rq->lvs[1];
	}

	list_for_each_entry(rq, crpl_get_rq_native_head(), node) {
		if (m[0] > rq->lvs[0])
			m[0] = rq->lvs[0];
		if (m[1] > rq->lvs[1])
			m[1] = rq->lvs[1];
	}
}

void crpl_free_rq_instance(crpl_rq *rq)
{
	kfree(rq);
}

void crpl_rq_timer_init(struct timer_list *timer,
							unsigned long rq_addr,
							void(*func)(unsigned long))
{
	init_timer(timer);
	timer->function = func;
	timer->data = rq_addr;
}

void crpl_rq_del_timer_sync(struct timer_list *timer)
{
	del_timer_sync(timer);
}

int crpl_rq_timer_pending(struct timer_list *timer)
{
	return timer_pending(timer);
}

void crpl_rq_set_timer_expires(struct timer_list *timer, u64 expires)
{
	timer->expires = expires;
}

u64 crpl_rq_get_timer_expires(struct timer_list *timer)
{
	return timer->expires;
}

void crpl_rq_add_timer(struct timer_list *timer)
{
	add_timer(timer);
}

void crpl_rq_del_timer(struct timer_list *timer)
{
	del_timer(timer);
}

/* init part */
int crpl_group_init(struct cpumask *mask)
{
	struct device *dev;
	unsigned int first_cpu = cpumask_first(mask);
	struct crpl_data *f = &per_cpu(cpu_state, first_cpu);
	struct crpl_data *state;
	unsigned int cpu;

	if (likely(f->inited))
		return 0;

	dev = ctech_find_cpu_device(first_cpu);
	if (!dev)
		return -ENODEV;

	pr_info("Creating CPU group %d\n", first_cpu);

	/* init per-cluster CRPL data */
	{
		struct cpufreq_frequency_table *tbl;
		unsigned int num_freqs = 0;
		tbl = ctech_get_freq_tbl(first_cpu);
		while (tbl[num_freqs].frequency != CPUFREQ_TABLE_END)
			num_freqs++;
		f->freq_lvs = num_freqs;
		f->freq_tbl =
			kmalloc(sizeof(unsigned int)*num_freqs, GFP_KERNEL);
		while (num_freqs) {
			f->freq_tbl[num_freqs-1] =
				tbl[num_freqs-1].frequency;
			num_freqs--;
		}

		f->debounce_enable = 1; /* default enable debounce */
	}

	atomic_set(&f->pending, 0);
	f->crpl_thread =
		kthread_run(cluster_relative_power_limit,
			    (void *)f,
			    "crpl/%d",
			    first_cpu
			    );
	set_user_nice(f->crpl_thread, -10);
	INIT_LIST_HEAD(&f->lru);

	for_each_cpu(cpu, mask) {
		pr_info("Init CPU%u state\n", cpu);

		state = &per_cpu(cpu_state, cpu);
		state->cpu = cpu;
		state->first_cpu = first_cpu;

		list_add_tail(&state->sib, &f->lru);
		state->last_update_time = ctech_get_time();
	}

	for_each_cpu(cpu, mask) {
		state = &per_cpu(cpu_state, cpu);
		state->inited = true;
	}

	f->tbl_version = POWER_TABLE_VERSION;

	kobject_init(&f->kobj, &ktype_crpl);
	return kobject_add(&f->kobj, &dev->kobj, "crpl");
}

/* cpu hotplug call back function */
static int __ref cpu_cb(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	return crpl_cpu_cb_core(nfb, action, hcpu);
}

static struct notifier_block __refdata cpu_notifier = {
	.notifier_call = cpu_cb,
	.priority = INT_MAX,
};

/* cpufreq governor call back function */
static int __ref cpufreq_gov_cb(struct notifier_block *nb, unsigned long val, void *data)
{
	return crpl_cpufreq_gov_cb_core(nb, val, data);
}

static struct notifier_block cpufreq_gov_nb = {
	.notifier_call = cpufreq_gov_cb,
	.priority = -INT_MAX,
};

/* cpufreq policy call back function */
static int __ref cpufreq_policy_cb(struct notifier_block *nb, unsigned long val, void *data)
{
	return crpl_cpufreq_policy_cb_core(nb, val, data);
}

static struct notifier_block cpufreq_pol_nb = {
	.notifier_call = cpufreq_policy_cb,
};

/* fb notify call back function (for detect display status) */
static int __ref fb_notify_cb(struct notifier_block *nb, unsigned long event, void *data)
{
	struct fb_event* evdata = data;

	if (FB_EARLY_EVENT_BLANK != event)
		return 0;

	if (FB_BLANK_UNBLANK == *(int*) evdata->data)
		crpl_display_status_core(true);
	else if(FB_BLANK_POWERDOWN == *(int*) evdata->data)
		crpl_display_status_core(false);

	return 0;
}

static struct notifier_block fb_notify_nb = {
	.notifier_call = fb_notify_cb,
	.priority = INT_MAX
};

static int __init crpl_init(void)
{
	struct cpufreq_policy *policy;
	unsigned int cpu;

	register_cpu_notifier(&cpu_notifier);
	cpufreq_register_notifier(&cpufreq_pol_nb, CPUFREQ_POLICY_NOTIFIER);
	cpufreq_register_notifier(&cpufreq_gov_nb, CPUFREQ_GOVINFO_NOTIFIER);
	fb_register_client(&fb_notify_nb);

	ctech_block_hotplug();
	for_each_online_cpu(cpu) {
		policy = ctech_get_policy(cpu);
		if (policy) {
			crpl_group_init(policy->related_cpus);
			ctech_put_policy(policy);
		}
	}
	ctech_unblock_hotplug();

	/* update power table at init stage */
	update_pwr_tbl();

	crpl_rq_thread =
		kthread_run(crpl_rq_thread_func,
				NULL,
			    "crpl-detect"
			    );
	set_user_nice(crpl_rq_thread, -10);

	/* init rq */
	crpl_rq_init();

	/* crpl hook funcs */
	ctech_crpl_hook_battery_capacity_cb(crpl_battery_capacity_cb_core);
	ctech_crpl_hook_usb_detect_cb(crpl_usb_detect_cb_core);
	ctech_crpl_hook_calculate_saved_energy_cb(crpl_calculate_saved_energy_core);

	return 0;
}

static void __exit crpl_exit(void)
{
	int cpu;
	struct crpl_data *pcpu;

	unregister_cpu_notifier(&cpu_notifier);
	cpufreq_unregister_notifier(&cpufreq_pol_nb, CPUFREQ_POLICY_NOTIFIER);
	cpufreq_unregister_notifier(&cpufreq_gov_nb, CPUFREQ_GOVINFO_NOTIFIER);
	fb_unregister_client(&fb_notify_nb);

	for_each_possible_cpu(cpu) {
		pcpu = &per_cpu(cpu_state, cpu);
		if (pcpu->inited && cpu == pcpu->first_cpu) {
			pcpu->inited = false;
			kobject_put(&pcpu->kobj);
			/*
			 * stop CRPL handler and release
			 * resources allocated before
			 */
			kthread_stop(pcpu->crpl_thread);
			kfree(pcpu->freq_tbl);
		}
		pcpu->inited = false;
	}

	/* reclaim power table */
	clear_pwr_tbl();

	kthread_stop(crpl_rq_thread);

	/* de-init rq */
	crpl_rq_exit();

	/* unhook crpl funcs */
	ctech_crpl_hook_battery_capacity_cb(NULL);
	ctech_crpl_hook_usb_detect_cb(NULL);
	ctech_crpl_hook_calculate_saved_energy_cb(NULL);
}

module_init(crpl_init);
module_exit(crpl_exit);

MODULE_DESCRIPTION("OnePlus cluster relative power limit driver");
MODULE_LICENSE("BSD");
