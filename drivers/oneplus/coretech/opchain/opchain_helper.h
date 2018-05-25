#ifndef _LINUX_OPCHAIN_HELPER_H
#define _LINUX_OPCHAIN_HELPER_H
#include "opchain_define.h"

extern u64 op_min_cap_load;
extern unsigned int *opc_boost;
extern unsigned int *opc_boost_target_load;
extern unsigned int *opc_boost_min_sample_time;

extern bool is_opc_task(struct task_struct *t, int type);
extern void is_opc_task_cb(bool (*cb)(struct task_struct *t, int type));
extern void opc_task_switch(bool enqueue, int cpu, struct task_struct *p, u64 clock);
extern void opc_task_switch_cb(void (*cb)(bool enqueue, int cpu, struct task_struct *p, u64 clock));
extern int opc_get_claim_on_cpu(int cpu);
extern void opc_get_claim_on_cpu_cb(int (*cb)(int cpu));
extern unsigned int opc_get_claims(void);
extern void opc_get_claims_cb(unsigned int (*cb)(void));
extern int opc_select_path(struct task_struct *cur, struct task_struct *t, int prev_cpu);
extern void opc_select_path_cb(int (*cb)(struct task_struct *cur, struct task_struct *t, int prev_cpu));
extern u64 opc_cpu_cravg_sync(int cpu, u64 rq_cravg, int op_path);
extern void opc_cpu_cravg_sync_cb(u64 (*cb)(int cpu, u64 rq_cravg, int op_path));
extern void opc_update_cpu_cravg_demand(u64 load);
extern void opc_update_cpu_cravg_demand_cb(void (*cb)(u64 load, unsigned int window));
extern u64 opc_get_min_cap_load(void);
extern void opc_set_boost(unsigned int *boost, unsigned int *boost_tl, unsigned int *boost_sample_time);

extern void *opc_get_task_rq(void *t);
extern struct rq *opc_get_cpu_rq(int cpu);
extern unsigned int opc_num_present_cpus(void);
extern unsigned int opc_task_load(struct task_struct *p);
extern int opc_cpu_online(int cpu);
#endif
