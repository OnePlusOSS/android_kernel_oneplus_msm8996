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

#ifndef _CRPL_INC_H_
#define _CRPL_INC_H_

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/gpl_helper.h>
#include <linux/struct_offset_helper.h>
#include <../../drivers/oneplus/coretech/crpl_helper.h>

struct crpl_data {
	/* Per CPU data. */
	bool	inited;
	unsigned int cpu;
	struct list_head sib;
	unsigned int first_cpu;

	struct list_head lru;
	/* flag to choose if use customized table */
	bool use_customized_table;
	/* add per-cluster CRPL definitions */
	long temp;
	unsigned int core_nums;
	unsigned int freq;
	unsigned int power_base;
	unsigned int power_limit;
	atomic_t pending;
	struct task_struct *crpl_thread;
	unsigned int freq_limit;
	unsigned int freq_lvs;
	unsigned int *freq_tbl;
	unsigned int pcost_latest;
	/* power table point to power or performance cluster */
	uint32_t **tbl;
	/* table version, tracking purpose */
	unsigned int tbl_version;
	/* debounce time */
	bool debounce_enable;
	s64 last_update_time;
	/* crpl lvs */
	unsigned int lvs;
	unsigned int scale_max_temp;

	struct kobject kobj;
};

enum {
	CRPL_DATA_INITED,
	CRPL_DATA_CPU,
	CRPL_DATA_SIB,
	CRPL_DATA_FIRST_CPU,
	CRPL_DATA_LRU,
	CRPL_DATA_USE_CUSTOMIZED_TABLE,
	CRPL_DATA_TEMP,
	CRPL_DATA_CORE_NUMS,
	CRPL_DATA_FREQ,
	CRPL_DATA_POWER_BASE,
	CRPL_DATA_POWER_LIMIT,
	CRPL_DATA_PENDING,
	CRPL_DATA_CRPL_THREAD,
	CRPL_DATA_FREQ_LIMIT,
	CRPL_DATA_FREQ_LVS,
	CRPL_DATA_FREQ_TBL,
	CRPL_DATA_PCOST_LATEST,
	CRPL_DATA_TBL,
	CRPL_DATA_TBL_VERSION,
	CRPL_DATA_DEBOUNCE_ENABLE,
	CRPL_DATA_LAST_UPDATE_TIME,
	CRPL_DATA_LVS,
	CRPL_DATA_LVS_LIMIT,
	CRPL_DATA_SCALE_MAX_TEMP,
	CRPL_DATA_RATIO,
	CRPL_DATA_KOBJ,

	__CRPL_DATA_MAX
};

#define CRPL_DATA_INITED_R(p)        (*(bool *)((char *)p + get_crpl_data_offset(CRPL_DATA_INITED)))
#define CRPL_DATA_PENDING_R(p)       (*(bool *)((char *)p + get_crpl_data_offset(CRPL_DATA_PENDING)))
#define CRPL_DATA_TEMP_R(p)          (*(long *)((char *)p + get_crpl_data_offset(CRPL_DATA_TEMP)))
#define CRPL_DATA_TBL_PP(p)          (*(uint32_t ***)((char *)p + get_crpl_data_offset(CRPL_DATA_TBL)))
#define CRPL_DATA_SIB_R(p)           (*(struct list_head *)((char *)p + get_crpl_data_offset(CRPL_DATA_SIB)))
#define CRPL_DATA_LRU_R(p)           (*(struct list_head *)((char *)p + get_crpl_data_offset(CRPL_DATA_LRU)))
#define CRPL_DATA_CPU_R(p)           (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_CPU)))
#define CRPL_DATA_FIRST_CPU_R(p)     (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_FIRST_CPU)))
#define CRPL_DATA_CORE_NUMS_R(p)     (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_CORE_NUMS)))
#define CRPL_DATA_FREQ_R(p)          (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_FREQ)))
#define CRPL_DATA_POWER_BASE_R(p)    (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_POWER_BASE)))
#define CRPL_DATA_POWER_LIMIT_R(p)   (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_POWER_LIMIT)))
#define CRPL_DATA_FREQ_LIMIT_R(p)    (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_FREQ_LIMIT)))
#define CRPL_DATA_FREQ_LVS_R(p)      (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_FREQ_LVS)))
#define CRPL_DATA_PCOST_LATEST_R(p)  (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_PCOST_LATEST)))
#define CRPL_DATA_TBL_VERSION_R(p)   (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_TBL_VERSION)))
#define CRPL_DATA_LVS_R(p)           (*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_LVS)))
#define CRPL_DATA_SCALE_MAX_TEMP_R(p)(*(unsigned int *)((char *)p + get_crpl_data_offset(CRPL_DATA_SCALE_MAX_TEMP)))
#define CRPL_DATA_FREQ_TBL_P(p)      (*(unsigned int **)((char *)p + get_crpl_data_offset(CRPL_DATA_FREQ_TBL)))
#define CRPL_DATA_CRPL_THREAD_P(p)   (*(struct task_struct **)((char *)p + get_crpl_data_offset(CRPL_DATA_CRPL_THREAD)))
#define CRPL_DATA_KOBJ_R(p)          (*(struct kobject *)((char *)p + get_crpl_data_offset(CRPL_DATA_KOBJ)))
#define CRPL_DATA_DEBOUNCE_ENABLE_R(p)    (*(bool *)((char *)p + get_crpl_data_offset(CRPL_DATA_DEBOUNCE_ENABLE)))
#define CRPL_DATA_LAST_UPDATE_TIME_R(p)   (*(s64 *)((char *)p + get_crpl_data_offset(CRPL_DATA_LAST_UPDATE_TIME)))
#define CRPL_DATA_USE_CUSTOMIZED_TABLE_R(p)    (*(bool *)((char *)p + get_crpl_data_offset(CRPL_DATA_USE_CUSTOMIZED_TABLE)))
gen_type_offset(crpl_data);


/* define crpl request struct */
#define CRPL_RQ_COMM_LEN 32
#define CRPL_CLUSTER_NUMS 2
typedef struct crpl_rq {
	char comm[CRPL_RQ_COMM_LEN];
	unsigned int lvs[CRPL_CLUSTER_NUMS];
	unsigned int thres[CRPL_CLUSTER_NUMS];
	unsigned int ratio[CRPL_CLUSTER_NUMS];
	unsigned int kick_off;
	unsigned int period;
	bool release_prepare;
	struct timer_list timer;
	struct list_head node;
} crpl_rq, *crpl_rq_p;

enum {
	CRPL_RQ_COMM,
	CRPL_RQ_LVS,
	CRPL_RQ_THRES,
	CRPL_RQ_RATIO,
	CRPL_RQ_KICK_OFF,
	CRPL_RQ_PERIOD,
	CRPL_RQ_RELEASE_PREPARE,
	CRPL_RQ_TIMER,
	CRPL_RQ_NODE,

	__CRPL_RQ_MAX
};

#define CRPL_RQ_COMM_R(p)          ((char *)((char *)p + get_crpl_rq_offset(CRPL_RQ_COMM)))
#define CRPL_RQ_LVS_R(p)           ((unsigned int *)((char *)p + get_crpl_rq_offset(CRPL_RQ_LVS)))
#define CRPL_RQ_THRES_R(p)         ((unsigned int *)((char *)p + get_crpl_rq_offset(CRPL_RQ_THRES)))
#define CRPL_RQ_RATIO_R(p)         ((unsigned int *)((char *)p + get_crpl_rq_offset(CRPL_RQ_RATIO)))
#define CRPL_RQ_KICK_OFF_R(p)      (*(unsigned int *)((char *)p + get_crpl_rq_offset(CRPL_RQ_KICK_OFF)))
#define CRPL_RQ_PERIOD_R(p)        (*(unsigned int *)((char *)p + get_crpl_rq_offset(CRPL_RQ_PERIOD)))
#define CRPL_RQ_RELEASE_PREPARE_R(p) (*(bool *)((char *)p + get_crpl_rq_offset(CRPL_RQ_RELEASE_PREPARE)))
#define CRPL_RQ_TIMER_R(p)         ((struct timer_list *)((char *)p + get_crpl_rq_offset(CRPL_RQ_TIMER)))
#define CRPL_RQ_NODE_R(p)          (*(struct list_head *)((char *)p + get_crpl_rq_offset(CRPL_RQ_NODE)))
gen_type_offset(crpl_rq);

/* cur table version 2 */
#define POWER_TABLE_VERSION (2)

void clear_pwr_tbl(void);
void update_pwr_tbl(void);
int crpl_get_temp_point(void);
int crpl_group_init(struct cpumask *mask);
int cluster_relative_power_limit(void *data);
struct crpl_data* crpl_get_crpl_data(int cpu);

/* crpl rq */
enum {
	CRPL_RQ_SUCCESS,
	CRPL_RQ_NOT_TURN_ON,
	CRPL_RQ_NATIVE_ACQUIRE_FAILED,
	CRPL_RQ_NATIVE_NOT_REGISTER_FAILED,
	CRPL_RQ_CANT_FOUND_RQ_FAILED,
	CRPL_RQ_NEW_RQ_INSTANCE_FAILED,
	CRPL_RQ_ALREADY_REGISTERED_FAILED,
	CRPL_RQ_UPDATE_FAILED,

	CRPL_RQ_ERROR_CODE_MAX
};

struct list_head *crpl_get_rq_head(void);
struct list_head *crpl_get_rq_native_head(void);
void crpl_list_add(struct list_head *_new, struct list_head *_head);
void crpl_list_add_tail(struct list_head *_new, struct list_head *_head);
void crpl_list_del_and_free(struct list_head *node);
void crpl_list_del(struct list_head *node);
int crpl_list_empty(struct list_head *node);
crpl_rq *crpl_list_rq_is_exist(const char *key, struct list_head *head);
crpl_rq *crpl_new_rq_instance(const char *settings);
crpl_rq *crpl_prepare_rq_instance(const char *settings, crpl_rq *rq);
void crpl_free_rq_instance(crpl_rq* rq);
void crpl_rq_timer_init(struct timer_list *timer, unsigned long rq_addr, void(*func)(unsigned long));
void crpl_rq_del_timer_sync(struct timer_list *timer);
void crpl_rq_add_timer(struct timer_list *timer);
void crpl_rq_del_timer(struct timer_list *timer);
void crpl_rq_set_timer_expires(struct timer_list *timer, u64 expires);
u64  crpl_rq_get_timer_expires(struct timer_list *timer);
int crpl_rq_timer_pending(struct timer_list *timer);
void crpl_rq_reset(void);
int crpl_rq_thread_func(void *data);
ssize_t crpl_rq_userspace_dump(char *buf);
void crpl_rq_native_dump(void);
void crpl_rq_spin_lock_irqsave(unsigned long *flags);
void crpl_rq_spin_unlock_irqrestore(unsigned long *flags);
unsigned int crpl_rq_init_core(void);
unsigned int crpl_rq_exit_core(void);
void crpl_rq_thread_func_outer(unsigned int *m);
extern struct task_struct *crpl_rq_thread;

/* call back */
int crpl_cpu_cb_core(struct notifier_block *nfb, unsigned long action, void *hcpu);
int crpl_cpufreq_gov_cb_core(struct notifier_block *nb, unsigned long val, void *data);
int crpl_cpufreq_policy_cb_core(struct notifier_block *nb, unsigned long val, void *data);

/* detect related call back function */
void crpl_battery_capacity_cb_core(unsigned int capacity);
void crpl_usb_detect_cb_core(int status);
void crpl_display_status_core(bool on);
void crpl_calculate_saved_energy_core(int cpu);
void crpl_is_ux_core(void);
extern struct task_struct *crpl_detect_thread;

/* sysfs function declare */
#define gen_crpl_sysfs_ro_core(node) \
        ssize_t crpl_show_##node##_core(struct crpl_data *state, char *buf);
#define gen_crpl_sysfs_ro_impl(node) \
        static ssize_t show_##node(struct crpl_data *state, char *buf) { \
                return crpl_show_##node##_core(state, buf); \
        }
#define gen_crpl_sysfs_wo_core(node) \
        ssize_t crpl_store_##node##_core(struct crpl_data *state, const char *buf, size_t count);
#define gen_crpl_sysfs_wo_impl(node) \
        static ssize_t store_##node(struct crpl_data *state, const char *buf, size_t count) {\
                return crpl_store_##node##_core(state, buf, count); \
        }
#define gen_crpl_sysfs_rw_core(node) \
        gen_crpl_sysfs_ro_core(node); \
        gen_crpl_sysfs_wo_core(node);

#define gen_crpl_sysfs_rw_impl(node) \
        gen_crpl_sysfs_ro_impl(node); \
        gen_crpl_sysfs_wo_impl(node);

gen_crpl_sysfs_rw_core(crpl);
gen_crpl_sysfs_rw_core(lvs);
gen_crpl_sysfs_rw_core(scale_max_temp);
gen_crpl_sysfs_rw_core(debounce_time);
gen_crpl_sysfs_rw_core(debounce_enable);
gen_crpl_sysfs_rw_core(use_customized_table);
gen_crpl_sysfs_rw_core(scale_load);
/* rq related sysfs nodes */
gen_crpl_sysfs_rw_core(rq_switch_on);
gen_crpl_sysfs_rw_core(rq_bat_thres);
gen_crpl_sysfs_rw_core(rq_usb_detect);
gen_crpl_sysfs_ro_core(rq_dump);
gen_crpl_sysfs_ro_core(rq_statistic);
gen_crpl_sysfs_wo_core(rq_acquire);
gen_crpl_sysfs_wo_core(rq_release);
gen_crpl_sysfs_wo_core(rq_update);
gen_crpl_sysfs_wo_core(rq_cleanup);

gen_crpl_sysfs_ro_core(crpl_dump);
gen_crpl_sysfs_ro_core(cpus);
gen_crpl_sysfs_ro_core(table_version);
#endif /* _CRPL_INC_H_*/
