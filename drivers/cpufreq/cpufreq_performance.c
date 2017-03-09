/*
 *  linux/drivers/cpufreq/cpufreq_performance.c
 *
 *  Copyright (C) 2002 - 2003 Dominik Brodowski <linux@brodo.de>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/module.h>

#ifdef CONFIG_CPU_FREQ_LIMIT_BOOT_CURRENT
#include <linux/oneplus/boot_mode.h>
#define C0_MAX_CPUFREQ	1593600
#define C1_MAX_CPUFREQ	2054400
#endif

static int cpufreq_governor_performance(struct cpufreq_policy *policy,
					unsigned int event)
{
#ifdef CONFIG_CPU_FREQ_LIMIT_BOOT_CURRENT
	unsigned int index = 0;
	static unsigned int c0_valid_freq = 0, c1_valid_freq = 0;
	struct cpufreq_frequency_table *table, *pos;
#endif

	switch (event) {
	case CPUFREQ_GOV_START:
#ifdef CONFIG_CPU_FREQ_LIMIT_BOOT_CURRENT
		if ((get_boot_mode() ==  MSM_BOOT_MODE__FACTORY)
			&&  cluster1_first_cpu) {
			table = cpufreq_frequency_get_table(policy->cpu);
			if (!table) {
				pr_err("cpufreq: Failed to get frequency table for CPU%u\n",0);
			} else {
				cpufreq_for_each_valid_entry(pos, table) {
					index = pos - table;
					if (policy->cpu < cluster1_first_cpu) {
						if (table[index].frequency == C0_MAX_CPUFREQ) {
							c0_valid_freq = table[index].frequency;
							break;
						}
                        		} else {
						if (table[index].frequency == C1_MAX_CPUFREQ) {
                                        	        c1_valid_freq = table[index].frequency;
							break;
						}
           		             	}
				}
			}
		}
#endif
	case CPUFREQ_GOV_LIMITS:
		pr_debug("setting to %u kHz because of event %u\n",
						policy->max, event);
#ifdef CONFIG_CPU_FREQ_LIMIT_BOOT_CURRENT
		if ((get_boot_mode() ==  MSM_BOOT_MODE__FACTORY)
			&& cluster1_first_cpu ) {
			if (policy->cpu < cluster1_first_cpu) {
				if (c0_valid_freq)
					__cpufreq_driver_target(policy, c0_valid_freq,
						CPUFREQ_RELATION_H);
				else
					__cpufreq_driver_target(policy, policy->max,
                                                CPUFREQ_RELATION_H);
			} else {
				if (c1_valid_freq)
					__cpufreq_driver_target(policy, c1_valid_freq,
						CPUFREQ_RELATION_H);
				else
					__cpufreq_driver_target(policy, policy->max,
                                                CPUFREQ_RELATION_H);
			}
		} else
			__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
#else
		__cpufreq_driver_target(policy, policy->max,
						CPUFREQ_RELATION_H);
#endif
		break;
	default:
		break;
	}
	return 0;
}

#ifdef CONFIG_CPU_FREQ_GOV_PERFORMANCE_MODULE
static
#endif
struct cpufreq_governor cpufreq_gov_performance = {
	.name		= "performance",
	.governor	= cpufreq_governor_performance,
	.owner		= THIS_MODULE,
};

static int __init cpufreq_gov_performance_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_performance);
}

static void __exit cpufreq_gov_performance_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_performance);
}

MODULE_AUTHOR("Dominik Brodowski <linux@brodo.de>");
MODULE_DESCRIPTION("CPUfreq policy governor 'performance'");
MODULE_LICENSE("GPL");

fs_initcall(cpufreq_gov_performance_init);
module_exit(cpufreq_gov_performance_exit);
