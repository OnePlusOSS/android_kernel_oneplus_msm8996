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
#ifndef _OP_GPL_HELPER_INC_
#define _OP_GPL_HELPER_INC_

extern s64 ctech_get_time(void);
extern void ctech_block_hotplug(void);
extern void ctech_unblock_hotplug(void);
extern void ctech_put_policy(struct cpufreq_policy *policy);
extern enum msm_cpu ctech_socinfo_get_msm_cpu(void);
extern struct device *ctech_find_cpu_device(unsigned cpu);
extern struct cpufreq_policy *ctech_get_policy(int cpu);
extern struct cpufreq_frequency_table *ctech_get_freq_tbl(unsigned int cpu);
extern int  ctech_online_core(unsigned int cpu);
extern int  ctech_sysfs_create_link(struct kobject *kobj, struct kobject *target,
								const char *name);
extern void ctech_sysfs_remove_link(struct kobject *kobj, const char *name);
extern struct rq *ctech_get_task_rq(struct task_struct *t);
extern struct rq *ctech_get_cpu_rq(int cpu);
extern unsigned int ctech_task_cpu(const struct task_struct *p);
extern u64 ctech_get_max_possible_capacity(void);
extern u64 ctech_get_min_capacity(void);
extern unsigned int ctech_max_task_load(void);
extern unsigned int ctech_num_present_cpus(void);
#endif //_OP_GPL_HELPER_INC_
