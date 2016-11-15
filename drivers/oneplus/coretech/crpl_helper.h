/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#ifndef _LINUX_CRPL_HELPER_H
#define _LINUX_CRPL_HELPER_H

#include <linux/module.h>
#include <linux/types.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <soc/qcom/socinfo.h>

/* define for specific soc id */
#define CTECH_MSM8996pro (305)  // [305] = {MSM_CPU_8996, "MSM8996pro"}
#define CTECH_MSM8996    (246)  // [246] = {MSM_CPU_8996, "MSM8996"}
#define CTECH_MSM8994    (207)  // [207] = {MSM_CPU_8994, "MSM8994"}

extern void ctech_crpl_set_cpuload(int cpu, unsigned int load);
extern unsigned int ctech_crpl_get_cpuload(int cpu);

extern void ctech_crpl_set_target_freq(int cpu, unsigned int target);
extern unsigned int ctech_crpl_get_target_freq(int cpu);

extern void ctech_crpl_hook_battery_capacity(unsigned int capacity);
extern void ctech_crpl_hook_battery_capacity_cb(void (*cb)(unsigned int));

extern void ctech_crpl_hook_usb_detect(int status);
extern void ctech_crpl_hook_usb_detect_cb(void (*cb)(int));

extern void ctech_crpl_hook_calculate_saved_energy(int cpu);
extern void ctech_crpl_hook_calculate_saved_energy_cb(void (*cb)(int));

extern int  ctech_is_online_idle_core(unsigned int cpu);
extern unsigned int ctech_get_freq_cost(unsigned int cpu, unsigned int freq);
extern unsigned int ctech_get_cost_at_temp(unsigned int cpu, unsigned int freq,
								 long temp);
extern bool ctech_is_msm8994(void);
extern bool ctech_is_msm8996(void);
extern bool ctech_is_msm8996pro(void);
extern unsigned int ctech_socinfo_get_id(void);
extern bool ctech_crpl_support(void);
#endif
