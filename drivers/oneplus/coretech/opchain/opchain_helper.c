/*
 * Copyright (c) 2015-2017, The OnePlus corporation. All rights reserved.
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

#include <linux/module.h>
#include <linux/types.h>
#include "../kernel/sched/sched.h"
#include "opchain_define.h"

void (*opc_binder_pass_t)(size_t dsize, uint32_t *data, int send) = NULL;
bool (*is_opc_task_t)(struct task_struct *t, int type) = NULL;
void (*opc_task_switch_t)(bool enqueue, int cpu, struct task_struct *p, u64 clock) = NULL;
int (*opc_get_claim_on_cpu_t)(int cpu) = NULL;
unsigned int (*opc_get_claims_t)(void) = NULL;
int (*opc_select_path_t)(struct task_struct *cur, struct task_struct *t, int prev_cpu) = NULL;
void (*opc_update_cpu_cravg_demand_t)(u64 load, unsigned int window) = NULL;
u64 (*opc_cpu_cravg_sync_t)(int cpu, u64 rq_cravg, int op_path) = NULL;
bool (*opc_balance_utask_t)(struct task_struct *t, int *upid, int *ustage, int *nr_claim_to_move, int claim, bool is_utf, int *nr_opc_cnt) = NULL;
void (*opc_balance_no_migrate_t)(struct task_struct *t, int *upid, int *ustage, int *nr_opc_cnt) = NULL;

u64 op_min_cap_load;
unsigned int *opc_boost = NULL;
unsigned int *opc_boost_target_load = NULL;
unsigned int *opc_boost_min_sample_time = NULL;

void opc_binder_pass(size_t data_size, uint32_t *data, int send)
{
	if (opc_binder_pass_t)
		opc_binder_pass_t(data_size, data, send);
}
EXPORT_SYMBOL(opc_binder_pass);

void opc_binder_pass_cb(void (*cb)(size_t dsize, uint32_t *data, int send))
{
	opc_binder_pass_t = cb;
}
EXPORT_SYMBOL(opc_binder_pass_cb);

bool is_opc_task(struct task_struct *t, int type)
{
	if (is_opc_task_t)
		return is_opc_task_t(t, type);
	return 0;
}
EXPORT_SYMBOL(is_opc_task);

void is_opc_task_cb(bool (*cb)(struct task_struct *t, int type))
{
	is_opc_task_t = cb;
}
EXPORT_SYMBOL(is_opc_task_cb);

void opc_task_switch(bool enqueue, int cpu, struct task_struct *p, u64 clock) {
    if (opc_task_switch_t)
        opc_task_switch_t(enqueue, cpu, (struct task_struct *)p, clock);
}
EXPORT_SYMBOL(opc_task_switch);

void opc_task_switch_cb(
		   void (*cb)(bool enqueue, int cpu, struct task_struct *p, u64 clock))
{
	opc_task_switch_t = cb;
}
EXPORT_SYMBOL(opc_task_switch_cb);

int opc_get_claim_on_cpu(int cpu)
{
	if (opc_get_claim_on_cpu_t)
		return opc_get_claim_on_cpu_t(cpu);
	return 0;
}
EXPORT_SYMBOL(opc_get_claim_on_cpu);

void opc_get_claim_on_cpu_cb(
    int (*cb)(int cpu))
{
	opc_get_claim_on_cpu_t = cb;
}
EXPORT_SYMBOL(opc_get_claim_on_cpu_cb);

unsigned int opc_get_claims(void)
{
	if (opc_get_claims_t)
		return opc_get_claims_t();
	return 0;
}
EXPORT_SYMBOL(opc_get_claims);

void opc_get_claims_cb(unsigned int (*cb)(void))
{
	opc_get_claims_t = cb;
}
EXPORT_SYMBOL(opc_get_claims_cb);

int opc_select_path(struct task_struct *cur, struct task_struct *t, int prev_cpu)
{
	if (opc_select_path_t)
		return opc_select_path_t(cur, t, prev_cpu);
	return OP_PATH_NORMAL;
}
EXPORT_SYMBOL(opc_select_path);

void opc_select_path_cb(int (*cb)(struct task_struct *cur, struct task_struct *t, int prev_cpu))
{
	opc_select_path_t = cb;
}
EXPORT_SYMBOL(opc_select_path_cb);

void opc_update_cpu_cravg_demand(u64 load)
{
	if (opc_update_cpu_cravg_demand_t)
		opc_update_cpu_cravg_demand_t(load, sched_ravg_window);
}
EXPORT_SYMBOL(opc_update_cpu_cravg_demand);

void opc_update_cpu_cravg_demand_cb(void (*cb)(u64 load, unsigned int window))
{
	opc_update_cpu_cravg_demand_t = cb;
}
EXPORT_SYMBOL(opc_update_cpu_cravg_demand_cb);

u64 opc_cpu_cravg_sync(int cpu, int sync, int op_path)
{
	if (opc_cpu_cravg_sync_t)
		return opc_cpu_cravg_sync_t(cpu, cpu_cravg_sync(cpu, sync), op_path);
	return cpu_cravg_sync(cpu, sync);
}
EXPORT_SYMBOL(opc_cpu_cravg_sync);

void opc_cpu_cravg_sync_cb(u64 (*cb)(int cpu, u64 rq_cravg, int op_path))
{
	opc_cpu_cravg_sync_t = cb;
}
EXPORT_SYMBOL(opc_cpu_cravg_sync_cb);

void opc_set_boost(unsigned int *boost, unsigned int *boost_tl, unsigned int *boost_sample_time)
{
	opc_boost = boost;
	opc_boost_target_load = boost_tl;
	opc_boost_min_sample_time = boost_sample_time;
}
EXPORT_SYMBOL(opc_set_boost);

u64 opc_get_min_cap_load(void)
{
	return op_min_cap_load;
}
EXPORT_SYMBOL(opc_get_min_cap_load);

struct rq *opc_get_task_rq(struct task_struct *t)
{
	return task_rq(t);
}
EXPORT_SYMBOL(opc_get_task_rq);

struct rq *opc_get_cpu_rq(int cpu)
{
	return cpu_rq(cpu);
}
EXPORT_SYMBOL(opc_get_cpu_rq);

unsigned int opc_num_present_cpus(void)
{
	return num_present_cpus();
}
EXPORT_SYMBOL(opc_num_present_cpus);

unsigned int opc_task_load(struct task_struct *p)
{
	return task_load(p);
}
EXPORT_SYMBOL(opc_task_load);

int opc_cpu_online(int cpu)
{
	return cpu_online(cpu);
}
EXPORT_SYMBOL(opc_cpu_online);
