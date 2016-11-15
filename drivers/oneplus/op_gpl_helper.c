/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/stddef.h>

#include "op_gpl_helper.h"
#include "../kernel/sched/sched.h"

void ctech_block_hotplug(void)
{
	get_online_cpus();
}
EXPORT_SYMBOL(ctech_block_hotplug);

void ctech_unblock_hotplug(void)
{
	put_online_cpus();
}
EXPORT_SYMBOL(ctech_unblock_hotplug);

s64 ctech_get_time(void)
{
	return ktime_to_ms(ktime_get());
}
EXPORT_SYMBOL(ctech_get_time);

struct cpufreq_policy *ctech_get_policy(int cpu)
{
	return cpufreq_cpu_get(cpu);
}
EXPORT_SYMBOL(ctech_get_policy);

void ctech_put_policy(struct cpufreq_policy *policy)
{
	cpufreq_cpu_put(policy);
}
EXPORT_SYMBOL(ctech_put_policy);

struct device *ctech_find_cpu_device(unsigned cpu)
{
	return get_cpu_device(cpu);
}
EXPORT_SYMBOL(ctech_find_cpu_device);

int __ref ctech_online_core(unsigned int cpu)
{
	return cpu_up(cpu);
}
EXPORT_SYMBOL(ctech_online_core);

struct cpufreq_frequency_table *ctech_get_freq_tbl(unsigned int cpu)
{
	return cpufreq_frequency_get_table(cpu);
}
EXPORT_SYMBOL(ctech_get_freq_tbl);

int ctech_sysfs_create_link(struct kobject *kobj, struct kobject *target,
						     const char *name)
{
	return sysfs_create_link(kobj, target, name);
}
EXPORT_SYMBOL(ctech_sysfs_create_link);

void ctech_sysfs_remove_link(struct kobject *kobj, const char *name)
{
	sysfs_remove_link(kobj, name);
}
EXPORT_SYMBOL(ctech_sysfs_remove_link);

struct rq *ctech_get_task_rq(struct task_struct *t)
{
	return task_rq(t);
}
EXPORT_SYMBOL(ctech_get_task_rq);

struct rq *ctech_get_cpu_rq(int cpu)
{
	return cpu_rq(cpu);
}
EXPORT_SYMBOL(ctech_get_cpu_rq);

unsigned int ctech_task_cpu(const struct task_struct *p)
{
	return task_cpu(p);
}
EXPORT_SYMBOL(ctech_task_cpu);

u64 ctech_get_max_possible_capacity(void)
{
	return max_possible_capacity;
}
EXPORT_SYMBOL(ctech_get_max_possible_capacity);

/*TODO: change to dynamic*/
u64 ctech_get_min_capacity(void)
{
	return min_capacity;
}
EXPORT_SYMBOL(ctech_get_min_capacity);

unsigned int ctech_max_task_load(void)
{
	return max_task_load();
}
EXPORT_SYMBOL(ctech_max_task_load);

unsigned int ctech_num_present_cpus(void)
{
	return num_present_cpus();
}
EXPORT_SYMBOL(ctech_num_present_cpus);
