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

#include "crpl_helper.h"
#include "../kernel/sched/sched.h"

static DEFINE_PER_CPU(unsigned int, crpl_cpuload);
static DEFINE_PER_CPU(unsigned int, crpl_target);

void ctech_crpl_set_cpuload(int cpu, unsigned int load)
{
	per_cpu(crpl_cpuload, cpu) = load;
}
EXPORT_SYMBOL(ctech_crpl_set_cpuload);

unsigned int ctech_crpl_get_cpuload(int cpu)
{
	return per_cpu(crpl_cpuload, cpu);
}
EXPORT_SYMBOL(ctech_crpl_get_cpuload);

void ctech_crpl_set_target_freq(int cpu, unsigned int target)
{
	per_cpu(crpl_target, cpu) = target;
}
EXPORT_SYMBOL(ctech_crpl_set_target_freq);

unsigned int ctech_crpl_get_target_freq(int cpu)
{
	return per_cpu(crpl_target, cpu);
}
EXPORT_SYMBOL(ctech_crpl_get_target_freq);

void (*crpl_hook_battery_capacity)(unsigned int capacity) = NULL;
void (*crpl_hook_usb_detect)(int status) = NULL;
void (*crpl_hook_calculate_saved_energy)(int cpu) = NULL;

void ctech_crpl_hook_battery_capacity(unsigned int capacity)
{
	if (crpl_hook_battery_capacity)
		crpl_hook_battery_capacity(capacity);
}
EXPORT_SYMBOL(ctech_crpl_hook_battery_capacity);

void ctech_crpl_hook_battery_capacity_cb(void (*cb)(unsigned int capacity))
{
	crpl_hook_battery_capacity = cb;
}
EXPORT_SYMBOL(ctech_crpl_hook_battery_capacity_cb);

void ctech_crpl_hook_usb_detect(int status)
{
	if (crpl_hook_usb_detect)
		crpl_hook_usb_detect(status);
}
EXPORT_SYMBOL(ctech_crpl_hook_usb_detect);

void ctech_crpl_hook_usb_detect_cb(void (*cb)(int))
{
	crpl_hook_usb_detect = cb;
}
EXPORT_SYMBOL(ctech_crpl_hook_usb_detect_cb);

void ctech_crpl_hook_calculate_saved_energy(int cpu)
{
	if (crpl_hook_calculate_saved_energy)
		crpl_hook_calculate_saved_energy(cpu);
}
EXPORT_SYMBOL(ctech_crpl_hook_calculate_saved_energy);

void ctech_crpl_hook_calculate_saved_energy_cb(void (*cb)(int))
{
	crpl_hook_calculate_saved_energy = cb;
}
EXPORT_SYMBOL(ctech_crpl_hook_calculate_saved_energy_cb);

int __ref ctech_is_online_idle_core(unsigned int cpu)
{
	int ret = idle_cpu(cpu) && !nr_iowait_cpu(cpu);
	return ret;
}
EXPORT_SYMBOL(ctech_is_online_idle_core);

unsigned int __ref ctech_get_freq_cost(unsigned int cpu, unsigned int freq)
{
#ifdef CONFIG_SCHED_HMP
	return power_cost_at_freq(cpu, freq);
#else
	/* NOT supported */
	return 0;
#endif
}
EXPORT_SYMBOL(ctech_get_freq_cost);

unsigned int ctech_get_cost_at_temp(
	unsigned int cpu,
	unsigned int freq,
	long temp)
{
#ifdef CONFIG_SCHED_HMP
	return power_cost_at_freq_at_temp(cpu, freq, temp);
#else
	/* NOT supported */
	return 0;
#endif
}
EXPORT_SYMBOL(ctech_get_cost_at_temp);

bool ctech_is_msm8994(void)
{
	return CTECH_MSM8994 == socinfo_get_id();
}
EXPORT_SYMBOL(ctech_is_msm8994);

bool ctech_is_msm8996(void)
{
	return CTECH_MSM8996 == socinfo_get_id();
}
EXPORT_SYMBOL(ctech_is_msm8996);

bool ctech_is_msm8996pro(void)
{
	return CTECH_MSM8996pro == socinfo_get_id();
}
EXPORT_SYMBOL(ctech_is_msm8996pro);

unsigned int ctech_socinfo_get_id(void)
{
	return (unsigned int) socinfo_get_id();
}
EXPORT_SYMBOL(ctech_socinfo_get_id);

bool ctech_crpl_support(void)
{
	uint32_t soc_id = socinfo_get_id();

	if (soc_id == CTECH_MSM8996pro ||
		soc_id == CTECH_MSM8996    ||
		soc_id == CTECH_MSM8994)
		return true;
	return false;
}
EXPORT_SYMBOL(ctech_crpl_support);
