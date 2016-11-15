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

#ifndef _OP_STRUCT_OFFSET_HELPER_INC_
#define _OP_STRUCT_OFFSET_HELPER_INC_

/* define macro to extern function declaration */
#define gen_type_offset(type) \
	extern unsigned int get_##type##_offset(int m);

/* define macro to create offset impl and export get offset/value symbol */
#define gen_type_offset_impl(type) \
	unsigned int get_##type##_offset(int m) { \
		return _##type##_offset[m]; } \
	EXPORT_SYMBOL(get_##type##_offset);

/* enum of struct cpufreq_policy member */
enum {
	CPUFREQ_POLICY_MIN,
	CPUFREQ_POLICY_MAX,
	CPUFREQ_POLICY_CUR,
	CPUFREQ_POLICY_RELATED_CPUS,
	CPUFREQ_POLICY_CPUINFO,

	__CPUFREQ_POLICY_MAX
};
#define CPUFREQ_POLICY_MIN_R(p)  (*(unsigned int *)((char *)p + get_cpufreq_policy_offset(CPUFREQ_POLICY_MIN)))
#define CPUFREQ_POLICY_MAX_R(p)  (*(unsigned int *)((char *)p + get_cpufreq_policy_offset(CPUFREQ_POLICY_MAX)))
#define CPUFREQ_POLICY_CUR_R(p)  (*(unsigned int *)((char *)p + get_cpufreq_policy_offset(CPUFREQ_POLICY_CUR)))
#define CPUFREQ_POLICY_RELATED_CPUS_R(p)  (*(cpumask_var_t *)((char *)p + get_cpufreq_policy_offset(CPUFREQ_POLICY_RELATED_CPUS)))
#define CPUFREQ_POLICY_CPUINFO_R(p)  (*(struct cpufreq_cpuinfo *)((char *)p + get_cpufreq_policy_offset(CPUFREQ_POLICY_CPUINFO)))
gen_type_offset(cpufreq_policy);

/* enum of struct cpufreq_govinfo member */
enum {
	CPUFREQ_GOVINFO_CPU,
	CPUFREQ_GOVINFO_LOAD,

	__CPUFREQ_GOVINFO_MAX
};
#define CPUFREQ_GOVINFO_CPU_R(p)  (*(unsigned int *)((char *)p + get_cpufreq_govinfo_offset(CPUFREQ_GOVINFO_CPU)))
#define CPUFREQ_GOVINFO_LOAD_R(p) (*(unsigned int *)((char *)p + get_cpufreq_govinfo_offset(CPUFREQ_GOVINFO_LOAD)))
gen_type_offset(cpufreq_govinfo);


/* enum of struct cpufreq_cpuinfo member */
enum {
	CPUFREQ_CPUINFO_MAX_FREQ,

	__CPUFREQ_CPUINFO_MAX
};
#define CPUFREQ_CPUINFO_MAX_FREQ_R(p)  (*(unsigned int *)((char *)p + get_cpufreq_cpuinfo_offset(CPUFREQ_CPUINFO_MAX_FREQ)))
gen_type_offset(cpufreq_cpuinfo);

/* enum of struct task struct */
enum {
	TASK_OFFSET_PID,
	TASK_OFFSET_TGID,
	TASK_OFFSET_GROUP_LEADER,
	TASK_OFFSET_COMM,
	TASK_OFFSET_UTASK_TAG,
	TASK_OFFSET_UTASK_TAG_BASE,
	TASK_OFFSET_ETASK_CLAIM,

	__TASK_OFFSET_MAX
};
#define TASK_PID_R(t) (*(pid_t *)((char *)t + get_task_struct_offset(TASK_OFFSET_PID)))
#define TASK_TGID_R(t) (*(pid_t *)((char *)t + get_task_struct_offset(TASK_OFFSET_TGID)))
#define TASK_GROUP_LEADER_R(t) (*(struct task_struct **)((char *)t + get_task_struct_offset(TASK_OFFSET_GROUP_LEADER)))
#define TASK_COMM_R(t) ((char *)t + get_task_struct_offset(TASK_OFFSET_COMM))
#define TASK_UTASK_TAG_R(t) (*(u32 *)((char *)t + get_task_struct_offset(TASK_OFFSET_UTASK_TAG)))
#define TASK_UTASK_TAG_BASE_R(t) (*(u64 *)((char *)t + get_task_struct_offset(TASK_OFFSET_UTASK_TAG_BASE)))
#define TASK_ETASK_CLAIM_R(t) (*(int *)((char *)t + get_task_struct_offset(TASK_OFFSET_ETASK_CLAIM)))
gen_type_offset(task_struct);

/* enum of struct rq */
enum {
	RQ_OFFSET_CLOCK,

	__RQ_OFFSET_MAX
};
#define RQ_CLOCK_R(rq) (*(u64 *)((char *)rq + get_rq_offset(RQ_OFFSET_CLOCK)))
gen_type_offset(rq);

#endif //_OP_STRUCT_OFFSET_HELPER_INC_
