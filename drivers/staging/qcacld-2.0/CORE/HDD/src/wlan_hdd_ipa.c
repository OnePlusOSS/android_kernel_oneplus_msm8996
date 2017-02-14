/*
 * Copyright (c) 2013-2015 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */
/*========================================================================

\file  wlan_hdd_ipa.c

\brief   WLAN HDD and ipa interface implementation

========================================================================*/

/*--------------------------------------------------------------------------
Include Files
------------------------------------------------------------------------*/
#ifdef IPA_OFFLOAD
#include <wlan_hdd_includes.h>
#include <wlan_hdd_ipa.h>

#include <linux/etherdevice.h>
#include <linux/atomic.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/list.h>
#include <linux/debugfs.h>
#include <linux/inetdevice.h>
#include <linux/ip.h>
#include <wlan_hdd_softap_tx_rx.h>

#include "vos_sched.h"
#include "tl_shim.h"
#include "wlan_qct_tl.h"

#ifdef IPA_UC_OFFLOAD
#include "wma.h"
#include "wma_api.h"
#endif /* IPA_UC_OFFLOAD */

#define HDD_IPA_DESC_BUFFER_RATIO 4
#define HDD_IPA_IPV4_NAME_EXT "_ipv4"
#define HDD_IPA_IPV6_NAME_EXT "_ipv6"

#define HDD_IPA_RX_INACTIVITY_MSEC_DELAY 1000
#ifdef IPA_UC_OFFLOAD
#define HDD_IPA_UC_WLAN_HDR_DES_MAC_OFFSET 12
#define HDD_IPA_UC_WLAN_8023_HDR_SIZE      14
/* WDI TX and RX PIPE */
#define HDD_IPA_UC_NUM_WDI_PIPE            2
#define HDD_IPA_UC_MAX_PENDING_EVENT       33

#define HDD_IPA_UC_DEBUG_DUMMY_MEM_SIZE    32000
#define HDD_IPA_UC_RT_DEBUG_PERIOD         300
#define HDD_IPA_UC_RT_DEBUG_BUF_COUNT      30
#define HDD_IPA_UC_RT_DEBUG_FILL_INTERVAL  10000

#define MAX_PENDING_EVENT_COUNT            20
#endif /* IPA_UC_OFFLOAD */

#ifdef IPA_UC_OFFLOAD
typedef enum {
	HDD_IPA_UC_OPCODE_TX_SUSPEND = 0,
	HDD_IPA_UC_OPCODE_TX_RESUME  = 1,
	HDD_IPA_UC_OPCODE_RX_SUSPEND = 2,
	HDD_IPA_UC_OPCODE_RX_RESUME  = 3,
	HDD_IPA_UC_OPCODE_STATS      = 4,
	HDD_IPA_UC_OPCODE_UC_READY   = 5,
	/* keep this last */
	HDD_IPA_UC_OPCODE_MAX
} hdd_ipa_uc_op_code;

typedef enum {
	HDD_IPA_UC_STAT_REASON_NONE,
	HDD_IPA_UC_STAT_REASON_DEBUG,
	HDD_IPA_UC_STAT_REASON_BW_CAL
} hdd_ipa_uc_stat_reason;
#endif /* IPA_UC_OFFLOAD */

struct llc_snap_hdr {
	uint8_t dsap;
	uint8_t ssap;
	uint8_t resv[4];
	__be16 eth_type;
} __packed;

struct hdd_ipa_tx_hdr {
	struct ethhdr eth;
	struct llc_snap_hdr llc_snap;
} __packed;

/* For Tx pipes, use 802.3 Header format */
static struct hdd_ipa_tx_hdr ipa_tx_hdr = {
	{
		{0xDE, 0xAD, 0xBE, 0xEF, 0xFF, 0xFF},
		{0xDE, 0xAD, 0xBE, 0xEF, 0xFF, 0xFF},
		0x00 /* length can be zero */
	},
	{
		/* LLC SNAP header 8 bytes */
		0xaa, 0xaa,
		{0x03, 0x00, 0x00, 0x00},
		0x0008 /* type value(2 bytes) ,filled by wlan  */
			/* 0x0800 - IPV4, 0x86dd - IPV6 */
	}
};

#ifdef IPA_UC_OFFLOAD
struct frag_header {
	uint32
		length:16,	/* length field is LSB of the FRAG DESC */
		reserved16:16;
	uint32 reserved32;
} __packed;

struct ipa_header {
	uint32
		vdev_id:8,	/* vdev_id field is LSB of IPA DESC */
		reserved:24;
} __packed;

struct hdd_ipa_uc_tx_hdr {
	struct frag_header frag_hd;
	struct ipa_header  ipa_hd;
	struct ethhdr eth;
} __packed;

/* For Tx pipes, use Ethernet-II Header format */
struct hdd_ipa_uc_tx_hdr ipa_uc_tx_hdr = {
	{
		0x00000000,
		0x00000000
	},
	{
		0x00000000
	},
	{
		{0x00, 0x03, 0x7f, 0xaa, 0xbb, 0xcc},
		{0x00, 0x03, 0x7f, 0xdd, 0xee, 0xff},
		0x0008
	}
};
#endif /* IPA_UC_OFFLOAD */

/*
   +----------+----------+--------------+--------+
   | Reserved | QCMAP ID | interface id | STA ID |
   +----------+----------+--------------+--------+
 */
struct hdd_ipa_cld_hdr {
	uint8_t reserved[2];
	uint8_t iface_id;
	uint8_t sta_id;
} __packed;

struct hdd_ipa_rx_hdr {
	struct hdd_ipa_cld_hdr cld_hdr;
	struct ethhdr eth;
} __packed;

struct hdd_ipa_pm_tx_cb {
	struct hdd_ipa_iface_context *iface_context;
	struct ipa_rx_data *ipa_tx_desc;
};

#ifdef IPA_UC_OFFLOAD
struct hdd_ipa_uc_rx_hdr {
	struct ethhdr eth;
} __packed;
#endif /* IPA_UC_OFFLOAD */

#define HDD_IPA_WLAN_CLD_HDR_LEN	sizeof(struct hdd_ipa_cld_hdr)
#ifdef IPA_UC_OFFLOAD
#define HDD_IPA_UC_WLAN_CLD_HDR_LEN	0
#endif /* IPA_UC_OFFLOAD */

#define HDD_IPA_WLAN_TX_HDR_LEN		sizeof(ipa_tx_hdr)
#ifdef IPA_UC_OFFLOAD
#define HDD_IPA_UC_WLAN_TX_HDR_LEN	sizeof(ipa_uc_tx_hdr)
#endif /* IPA_UC_OFFLOAD */

#define HDD_IPA_WLAN_RX_HDR_LEN		sizeof(struct hdd_ipa_rx_hdr)
#ifdef IPA_UC_OFFLOAD
#define HDD_IPA_UC_WLAN_RX_HDR_LEN	sizeof(struct hdd_ipa_uc_rx_hdr)
#endif /* IPA_UC_OFFLOAD */

#define HDD_IPA_WLAN_HDR_DES_MAC_OFFSET 0

#define HDD_IPA_GET_IFACE_ID(_data) \
	(((struct hdd_ipa_cld_hdr *) (_data))->iface_id)


#define HDD_IPA_LOG(LVL, fmt, args...) VOS_TRACE(VOS_MODULE_ID_HDD, LVL, \
				"%s:%d: "fmt, __func__, __LINE__, ## args)

#define HDD_IPA_DBG_DUMP(_lvl, _prefix, _buf, _len) \
	do {\
		VOS_TRACE(VOS_MODULE_ID_HDD, _lvl, "%s:", _prefix); \
		VOS_TRACE_HEX_DUMP(VOS_MODULE_ID_HDD, _lvl, _buf, _len); \
	} while(0)

enum hdd_ipa_rm_state {
	HDD_IPA_RM_RELEASED,
	HDD_IPA_RM_GRANT_PENDING,
	HDD_IPA_RM_GRANTED,
};

#define HDD_IPA_MAX_IFACE 3
#define HDD_IPA_MAX_SYSBAM_PIPE 4
#define HDD_IPA_RX_PIPE  HDD_IPA_MAX_IFACE

static struct hdd_ipa_adapter_2_client {
	enum ipa_client_type cons_client;
	enum ipa_client_type prod_client;
} hdd_ipa_adapter_2_client[HDD_IPA_MAX_IFACE] = {
#ifdef IPA_UC_OFFLOAD
	{IPA_CLIENT_WLAN2_CONS, IPA_CLIENT_WLAN1_PROD},
	{IPA_CLIENT_WLAN3_CONS, IPA_CLIENT_WLAN1_PROD},
	{IPA_CLIENT_WLAN4_CONS, IPA_CLIENT_WLAN1_PROD},
#else
	{IPA_CLIENT_WLAN1_CONS, IPA_CLIENT_WLAN1_PROD},
	{IPA_CLIENT_WLAN2_CONS, IPA_CLIENT_WLAN1_PROD},
	{IPA_CLIENT_WLAN3_CONS, IPA_CLIENT_WLAN1_PROD},
#endif
};

struct hdd_ipa_sys_pipe {
	uint32_t conn_hdl;
	uint8_t conn_hdl_valid;
	struct ipa_sys_connect_params ipa_sys_params;
};

struct hdd_ipa_iface_stats {
	uint64_t num_tx;
	uint64_t num_tx_drop;
	uint64_t num_tx_err;
	uint64_t num_tx_cac_drop;
	uint64_t num_rx_prefilter;
	uint64_t num_rx_ipa_excep;
	uint64_t num_rx_recv;
	uint64_t num_rx_recv_mul;
	uint64_t num_rx_send_desc_err;
	uint64_t max_rx_mul;
};

struct hdd_ipa_priv;

struct hdd_ipa_iface_context {
	struct hdd_ipa_priv *hdd_ipa;
	hdd_adapter_t  *adapter;
	void *tl_context;

	enum ipa_client_type cons_client;
	enum ipa_client_type prod_client;

	uint8_t iface_id; /* This iface ID */
	uint8_t sta_id; /* This iface station ID */
	adf_os_spinlock_t interface_lock;
	uint32_t ifa_address;
	struct hdd_ipa_iface_stats stats;
};


struct hdd_ipa_stats {
	uint32_t event[IPA_WLAN_EVENT_MAX];
	uint64_t num_send_msg;
	uint64_t num_free_msg;

	uint64_t num_rm_grant;
	uint64_t num_rm_release;
	uint64_t num_rm_grant_imm;
	uint64_t num_cons_perf_req;
	uint64_t num_prod_perf_req;

	uint64_t num_rx_drop;
	uint64_t num_rx_ipa_tx_dp;
	uint64_t num_rx_ipa_splice;
	uint64_t num_rx_ipa_loop;
	uint64_t num_rx_ipa_tx_dp_err;
	uint64_t num_rx_ipa_write_done;
	uint64_t num_max_ipa_tx_mul;
	uint64_t num_rx_ipa_hw_maxed_out;
	uint64_t max_pend_q_cnt;

	uint64_t num_tx_comp_cnt;
	uint64_t num_tx_queued;
	uint64_t num_tx_dequeued;
	uint64_t num_max_pm_queue;

	uint64_t num_freeq_empty;
	uint64_t num_pri_freeq_empty;
	uint64_t num_rx_excep;
	uint64_t num_tx_bcmc;
	uint64_t num_tx_bcmc_err;
};

#ifdef IPA_UC_OFFLOAD
struct ipa_uc_stas_map {
	v_BOOL_t is_reserved;
	uint8_t sta_id;
};

struct op_msg_type {
	uint8_t msg_t;
	uint8_t rsvd;
	uint16_t op_code;
	uint16_t len;
	uint16_t rsvd_snd;
};

struct ipa_uc_fw_stats {
	uint32_t tx_comp_ring_base;
	uint32_t tx_comp_ring_size;
	uint32_t tx_comp_ring_dbell_addr;
	uint32_t tx_comp_ring_dbell_ind_val;
	uint32_t tx_comp_ring_dbell_cached_val;
	uint32_t tx_pkts_enqueued;
	uint32_t tx_pkts_completed;
	uint32_t tx_is_suspend;
	uint32_t tx_reserved;
	uint32_t rx_ind_ring_base;
	uint32_t rx_ind_ring_size;
	uint32_t rx_ind_ring_dbell_addr;
	uint32_t rx_ind_ring_dbell_ind_val;
	uint32_t rx_ind_ring_dbell_ind_cached_val;
	uint32_t rx_ind_ring_rdidx_addr;
	uint32_t rx_ind_ring_rd_idx_cached_val;
	uint32_t rx_refill_idx;
	uint32_t rx_num_pkts_indicated;
	uint32_t rx_buf_refilled;
	uint32_t rx_num_ind_drop_no_space;
	uint32_t rx_num_ind_drop_no_buf;
	uint32_t rx_is_suspend;
	uint32_t rx_reserved;
};

struct ipa_uc_pending_event {
	vos_list_node_t node;
	hdd_adapter_t *adapter;
	enum ipa_wlan_event type;
	uint8_t sta_id;
	uint8_t mac_addr[VOS_MAC_ADDR_SIZE];
};

static const char *op_string[HDD_IPA_UC_OPCODE_MAX] = {
	"TX_SUSPEND",
	"TX_RESUME",
	"RX_SUSPEND",
	"RX_RESUME",
	"STATS",
	"OPCODE_MAX"
};

struct uc_rm_work_struct {
	struct work_struct work;
	enum ipa_rm_event event;
};

struct uc_op_work_struct {
	struct work_struct work;
	struct op_msg_type *msg;
};
static uint8_t vdev_to_iface[CSR_ROAM_SESSION_MAX];

struct uc_rt_debug_info {
	v_TIME_t time;
	uint64_t ipa_excp_count;
	uint64_t rx_drop_count;
	uint64_t net_sent_count;
	uint64_t rx_discard_count;
	uint64_t rx_mcbc_count;
	uint64_t tx_mcbc_count;
	uint64_t tx_fwd_count;
	uint64_t rx_destructor_call;
};
#endif /* IPA_UC_OFFLOAD */

struct hdd_ipa_priv {
	struct hdd_ipa_sys_pipe sys_pipe[HDD_IPA_MAX_SYSBAM_PIPE];
	struct hdd_ipa_iface_context iface_context[HDD_IPA_MAX_IFACE];
	uint8_t num_iface;
	enum hdd_ipa_rm_state rm_state;
	/*
	 * IPA driver can send RM notifications with IRQ disabled so using adf
	 * APIs as it is taken care gracefully. Without this, kernel would throw
	 * an warning if spin_lock_bh is used while IRQ is disabled
	 */
	adf_os_spinlock_t rm_lock;
	struct work_struct rm_work;
#ifdef IPA_UC_OFFLOAD
	struct uc_rm_work_struct uc_rm_work;
	struct uc_op_work_struct uc_op_work[HDD_IPA_UC_OPCODE_MAX];
#endif
	vos_wake_lock_t wake_lock;
	struct delayed_work wake_lock_work;
	bool wake_lock_released;

	enum ipa_client_type prod_client;

	atomic_t tx_ref_cnt;
	adf_nbuf_queue_t pm_queue_head;
	struct work_struct pm_work;
	adf_os_spinlock_t pm_lock;
	bool suspended;

	uint32_t pending_hw_desc_cnt;
	uint32_t hw_desc_cnt;
	spinlock_t q_lock;
	uint32_t freeq_cnt;
	struct list_head free_desc_head;

	uint32_t pend_q_cnt;
	struct list_head pend_desc_head;

	hdd_context_t *hdd_ctx;

	struct dentry *debugfs_dir;
	struct hdd_ipa_stats stats;

	struct notifier_block ipv4_notifier;
	uint32_t curr_prod_bw;
	uint32_t curr_cons_bw;

#ifdef IPA_UC_OFFLOAD
	uint8_t activated_fw_pipe;
	uint8_t sap_num_connected_sta;
#ifdef IPA_UC_STA_OFFLOAD
	uint8_t sta_connected;
#endif
	uint32_t tx_pipe_handle;
	uint32_t rx_pipe_handle;
	v_BOOL_t resource_loading;
	v_BOOL_t resource_unloading;
	v_BOOL_t pending_cons_req;
	struct ipa_uc_stas_map  assoc_stas_map[WLAN_MAX_STA_COUNT];
	vos_list_t pending_event;
	vos_lock_t event_lock;
	uint32_t ipa_tx_packets_diff;
	uint32_t ipa_rx_packets_diff;
	uint32_t ipa_p_tx_packets;
	uint32_t ipa_p_rx_packets;
	uint64_t ipa_tx_forward;
	uint64_t ipa_rx_discard;
	uint64_t ipa_rx_net_send_count;
	uint64_t ipa_rx_internel_drop_count;
	uint64_t ipa_rx_destructor_count;
        hdd_ipa_uc_stat_reason stat_req_reason;
	struct ipa_wdi_in_params cons_pipe_in;
	struct ipa_wdi_in_params prod_pipe_in;
	v_BOOL_t uc_loaded;
	v_BOOL_t wdi_enabled;
	bool ipa_pipes_down;
	vos_timer_t rt_debug_timer;
	struct uc_rt_debug_info rt_bug_buffer[HDD_IPA_UC_RT_DEBUG_BUF_COUNT];
	unsigned int rt_buf_fill_index;
	vos_timer_t rt_debug_fill_timer;
	vos_lock_t rt_debug_lock;
	vos_lock_t ipa_lock;
#endif /* IPA_UC_OFFLOAD */
};

static struct hdd_ipa_priv *ghdd_ipa;

#define HDD_IPA_ENABLE_MASK			BIT(0)
#define HDD_IPA_PRE_FILTER_ENABLE_MASK		BIT(1)
#define HDD_IPA_IPV6_ENABLE_MASK		BIT(2)
#define HDD_IPA_RM_ENABLE_MASK			BIT(3)
#define HDD_IPA_CLK_SCALING_ENABLE_MASK		BIT(4)
#define HDD_IPA_REAL_TIME_DEBUGGING             BIT(8)

#define HDD_IPA_IS_CONFIG_ENABLED(_hdd_ctx, _mask)\
	(((_hdd_ctx)->cfg_ini->IpaConfig & (_mask)) == (_mask))

#ifdef IPA_UC_OFFLOAD
#define HDD_IPA_INCREASE_INTERNAL_DROP_COUNT(hdd_ipa) \
	hdd_ipa->ipa_rx_internel_drop_count++
#define HDD_IPA_INCREASE_NET_SEND_COUNT(hdd_ipa) \
	hdd_ipa->ipa_rx_net_send_count++
#else
#define HDD_IPA_INCREASE_INTERNAL_DROP_COUNT(hdd_ipa) {}
#define HDD_IPA_INCREASE_NET_SEND_COUNT(hdd_ipa) {}
#endif /* IPA_UC_OFFLOAD */

/* Local Function Prototypes */
static void hdd_ipa_i2w_cb(void *priv, enum ipa_dp_evt_type evt,
		unsigned long data);
static void hdd_ipa_w2i_cb(void *priv, enum ipa_dp_evt_type evt,
		unsigned long data);
static void hdd_ipa_msg_free_fn(void *buff, uint32_t len, uint32_t type);

#ifdef IPA_UC_OFFLOAD
extern int process_wma_set_command(int sessid, int paramid,
                                   int sval, int vpdev);
#endif /* IPA_UC_OFFLOAD */
static void hdd_ipa_cleanup_iface(struct hdd_ipa_iface_context *iface_context);

bool hdd_ipa_is_enabled(hdd_context_t *hdd_ctx)
{
	return HDD_IPA_IS_CONFIG_ENABLED(hdd_ctx, HDD_IPA_ENABLE_MASK);
}

static inline bool hdd_ipa_uc_is_enabled(struct hdd_ipa_priv *hdd_ipa)
{
#ifdef IPA_UC_OFFLOAD
	return (hdd_ipa && hdd_ipa->hdd_ctx->cfg_ini->IpaUcOffloadEnabled);
#else
	return false;
#endif /* IPA_UC_OFFLOAD */
}

static inline bool hdd_ipa_uc_sta_is_enabled(struct hdd_ipa_priv *hdd_ipa)
{
#ifdef IPA_UC_STA_OFFLOAD
	return (hdd_ipa_uc_is_enabled(hdd_ipa) &&
		hdd_ipa->hdd_ctx->cfg_ini->ipa_uc_sta_offload);
#else
	return false;
#endif /* IPA_UC_STA_OFFLOAD */
}

static inline bool hdd_ipa_is_pre_filter_enabled(struct hdd_ipa_priv *hdd_ipa)
{
	hdd_context_t *hdd_ctx = hdd_ipa->hdd_ctx;
	return HDD_IPA_IS_CONFIG_ENABLED(hdd_ctx, HDD_IPA_PRE_FILTER_ENABLE_MASK);
}

static inline bool hdd_ipa_is_ipv6_enabled(struct hdd_ipa_priv *hdd_ipa)
{
	hdd_context_t *hdd_ctx = hdd_ipa->hdd_ctx;
	return HDD_IPA_IS_CONFIG_ENABLED(hdd_ctx, HDD_IPA_IPV6_ENABLE_MASK);
}

static inline bool hdd_ipa_is_rm_enabled(struct hdd_ipa_priv *hdd_ipa)
{
	hdd_context_t *hdd_ctx = hdd_ipa->hdd_ctx;
	return HDD_IPA_IS_CONFIG_ENABLED(hdd_ctx, HDD_IPA_RM_ENABLE_MASK);
}

static inline bool hdd_ipa_is_clk_scaling_enabled(struct hdd_ipa_priv *hdd_ipa)
{
	hdd_context_t *hdd_ctx = hdd_ipa->hdd_ctx;
	return HDD_IPA_IS_CONFIG_ENABLED(hdd_ctx,
			HDD_IPA_CLK_SCALING_ENABLE_MASK |
			HDD_IPA_RM_ENABLE_MASK);
}

static inline bool hdd_ipa_is_rt_debugging_enabled(hdd_context_t *hdd_ctx)
{
	return HDD_IPA_IS_CONFIG_ENABLED(hdd_ctx, HDD_IPA_REAL_TIME_DEBUGGING);
}

static struct ipa_tx_data_desc *hdd_ipa_alloc_data_desc(
		struct hdd_ipa_priv *hdd_ipa, int priority)
{
	struct ipa_tx_data_desc *desc = NULL;

	spin_lock_bh(&hdd_ipa->q_lock);

	/* Keep the descriptors for priority alloc which can be used for
	 * anchor nodes
	 */
	if (hdd_ipa->freeq_cnt < (HDD_IPA_DESC_BUFFER_RATIO * 2) && !priority) {
		hdd_ipa->stats.num_freeq_empty++;
		goto end;
	}

	if (!list_empty(&hdd_ipa->free_desc_head)) {
		desc = list_first_entry(&hdd_ipa->free_desc_head,
				struct ipa_tx_data_desc, link);
		list_del(&desc->link);
		hdd_ipa->freeq_cnt--;
	} else {
		hdd_ipa->stats.num_pri_freeq_empty++;
	}

end:
	spin_unlock_bh(&hdd_ipa->q_lock);

	return desc;
}

static void hdd_ipa_free_data_desc(struct hdd_ipa_priv *hdd_ipa,
		struct ipa_tx_data_desc *desc)
{
	desc->priv = NULL;
	desc->pyld_buffer = NULL;
	desc->pyld_len = 0;
	spin_lock_bh(&hdd_ipa->q_lock);
	list_add_tail(&desc->link, &hdd_ipa->free_desc_head);
	hdd_ipa->freeq_cnt++;
	spin_unlock_bh(&hdd_ipa->q_lock);
}

static struct iphdr * hdd_ipa_get_ip_pkt(void *data, uint16_t *eth_type)
{
	struct ethhdr *eth = (struct ethhdr *)data;
	struct llc_snap_hdr *ls_hdr;
	struct iphdr *ip_hdr;

	ip_hdr = NULL;
	*eth_type = be16_to_cpu(eth->h_proto);
	if (*eth_type < 0x600) {
		/* Non Ethernet II framing format */
		ls_hdr = (struct llc_snap_hdr *)((uint8_t *)data +
						sizeof(struct ethhdr));

		if (((ls_hdr->dsap == 0xAA) && (ls_hdr->ssap == 0xAA)) ||
			((ls_hdr->dsap == 0xAB) && (ls_hdr->ssap == 0xAB)))
			*eth_type = be16_to_cpu(ls_hdr->eth_type);
		ip_hdr = (struct iphdr *)((uint8_t *)data +
				sizeof(struct ethhdr) + sizeof(struct llc_snap_hdr));
	} else if (*eth_type == ETH_P_IP) {
		ip_hdr = (struct iphdr *)((uint8_t *)data +
						sizeof(struct ethhdr));
	}

	return ip_hdr;
}

static bool hdd_ipa_can_send_to_ipa(hdd_adapter_t *adapter, struct hdd_ipa_priv *hdd_ipa, void *data)
{
	uint16_t eth_type;
	struct iphdr *ip_hdr = NULL;

	if (!hdd_ipa_is_pre_filter_enabled(hdd_ipa))
		return true;
	ip_hdr = hdd_ipa_get_ip_pkt(data, &eth_type);

	/* Check if the dest IP address is itself, then bypass IPA */
	if (eth_type == ETH_P_IP) {
		if (ip_hdr->daddr != ((struct hdd_ipa_iface_context *)(adapter->ipa_context))->ifa_address)
			return true;
		else
			return false;
	}

	if (hdd_ipa_is_ipv6_enabled(hdd_ipa) && eth_type == ETH_P_IPV6)
		return true;

	return false;
}

#ifdef IPA_UC_OFFLOAD
/**
 * hdd_ipa_uc_rt_debug_host_fill() - fill rt debug buffer
 * @ctext: pointer to hdd context.
 *
 * If rt debug enabled, periodically called, and fill debug buffer
 *
 * Return: none
 */
static void hdd_ipa_uc_rt_debug_host_fill(void *ctext)
{
	hdd_context_t *hdd_ctx = ctext;
	struct hdd_ipa_priv *hdd_ipa;
	struct uc_rt_debug_info *dump_info = NULL;

	if (wlan_hdd_validate_context(hdd_ctx))
		return;

	if (!hdd_ctx->hdd_ipa ||
		!hdd_ipa_uc_is_enabled((struct hdd_ipa_priv *)hdd_ctx->hdd_ipa)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: IPA UC is not enabled", __func__);
		return;
	}

	hdd_ipa = (struct hdd_ipa_priv *)hdd_ctx->hdd_ipa;

	vos_lock_acquire(&hdd_ipa->rt_debug_lock);
	dump_info = &hdd_ipa->rt_bug_buffer[
		hdd_ipa->rt_buf_fill_index % HDD_IPA_UC_RT_DEBUG_BUF_COUNT];
	if (!dump_info) {
		vos_lock_release(&hdd_ipa->rt_debug_lock);
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: invalid dump pointer", __func__);
		return;
	}

	dump_info->time = vos_timer_get_system_time();
	dump_info->ipa_excp_count = hdd_ipa->stats.num_rx_excep;
	dump_info->rx_drop_count = hdd_ipa->ipa_rx_internel_drop_count;
	dump_info->net_sent_count = hdd_ipa->ipa_rx_net_send_count;
	dump_info->rx_discard_count = hdd_ipa->ipa_rx_discard;
	dump_info->tx_mcbc_count = hdd_ipa->stats.num_tx_bcmc;
	dump_info->tx_fwd_count = hdd_ipa->ipa_tx_forward;
	dump_info->rx_destructor_call = hdd_ipa->ipa_rx_destructor_count;
	hdd_ipa->rt_buf_fill_index++;
	vos_lock_release(&hdd_ipa->rt_debug_lock);

	vos_timer_start(&hdd_ipa->rt_debug_fill_timer,
		HDD_IPA_UC_RT_DEBUG_FILL_INTERVAL);
}

/**
 * hdd_ipa_uc_rt_debug_host_dump() - dump rt debug buffer
 * @pHddCtx: pointer to hdd context.
 *
 * If rt debug enabled, dump debug buffer contents based on requirement
 *
 * Return: none
 */
void hdd_ipa_uc_rt_debug_host_dump(hdd_context_t *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa;
	unsigned int dump_count;
	unsigned int dump_index;
	struct uc_rt_debug_info *dump_info = NULL;

	if (wlan_hdd_validate_context(hdd_ctx))
		return;

	hdd_ipa = hdd_ctx->hdd_ipa;
	if (!hdd_ipa ||
		!hdd_ipa_uc_is_enabled((struct hdd_ipa_priv *)hdd_ctx->hdd_ipa)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: IPA UC is not enabled", __func__);
		return;
	}

	pr_err("========= WLAN-IPA DEBUG BUF DUMP ==========\n");
	pr_err("     TM     :   EXEP   :   DROP   :   NETS   :   MCBC   :   TXFD   :   DSTR   :   DSCD\n");

	vos_lock_acquire(&hdd_ipa->rt_debug_lock);
	for (dump_count = 0;
		dump_count < HDD_IPA_UC_RT_DEBUG_BUF_COUNT;
		dump_count++) {
		dump_index = (hdd_ipa->rt_buf_fill_index + dump_count) %
			HDD_IPA_UC_RT_DEBUG_BUF_COUNT;
		dump_info = &hdd_ipa->rt_bug_buffer[dump_index];
		if ((dump_index > HDD_IPA_UC_RT_DEBUG_BUF_COUNT) ||
			(!dump_info)) {
			vos_lock_release(&hdd_ipa->rt_debug_lock);
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"INVALID");
			return;
		}
		pr_err("%12lu:%10llu:%10llu:%10llu:%10llu:%10llu:%10llu:%10llu\n",
			dump_info->time, dump_info->ipa_excp_count,
			dump_info->rx_drop_count, dump_info->net_sent_count,
			dump_info->tx_mcbc_count, dump_info->tx_fwd_count,
			dump_info->rx_destructor_call,
			dump_info->rx_discard_count);
	}
	vos_lock_release(&hdd_ipa->rt_debug_lock);
	pr_err("======= WLAN-IPA DEBUG BUF DUMP END ========\n");
}

/**
 * hdd_ipa_uc_rt_debug_handler - periodic memory health monitor handler
 * @ctext: pointer to hdd context.
 *
 * periodically called by timer expire
 * will try to alloc dummy memory and detect out of memory condition
 * if out of memory detected, dump wlan-ipa stats
 *
 * Return: none
 */
static void hdd_ipa_uc_rt_debug_handler(void *ctext)
{
	hdd_context_t *hdd_ctx= ctext;
	struct hdd_ipa_priv *hdd_ipa = hdd_ctx->hdd_ipa;
	void *dummy_ptr;

	if (wlan_hdd_validate_context(hdd_ctx))
		return;

	if (!hdd_ipa_is_rt_debugging_enabled(hdd_ctx)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
			"%s: IPA RT debug is not enabled", __func__);
		return;
	}

	/*
	 * Allocate dummy buffer periodically and free immediately
	 * This will proactively detect OOM condition
	 * And if allocation fail, will dump WLAN IPA stats
	 */
	dummy_ptr = kmalloc(HDD_IPA_UC_DEBUG_DUMMY_MEM_SIZE,
		GFP_KERNEL | GFP_ATOMIC);
	if (!dummy_ptr) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_FATAL,
			"%s: Dummy alloc fail", __func__);
		hdd_ipa_uc_rt_debug_host_dump(hdd_ctx);
		hdd_ipa_uc_stat_request(
			hdd_get_adapter(hdd_ctx, WLAN_HDD_SOFTAP), 1);
	} else {
		kfree(dummy_ptr);
	}

	vos_timer_start(&hdd_ipa->rt_debug_timer,
		HDD_IPA_UC_RT_DEBUG_PERIOD);
}

/**
 * hdd_ipa_uc_rt_debug_destructor - called by data packet free
 * @skb: packet pinter
 *
 * when free data packet, will be invoked by wlan client and will increase
 * free counter
 *
 * Return: none
 */
void hdd_ipa_uc_rt_debug_destructor(struct sk_buff *skb)
{
	if (!ghdd_ipa) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: invalid hdd context", __func__);
		return;
	}

	ghdd_ipa->ipa_rx_destructor_count++;
}

/**
 * hdd_ipa_uc_rt_debug_deinit - remove resources to handle rt debugging
 * @pHddCtx: hdd main context
 *
 * free all rt debugging resources
 *
 * Return: none
 */
static void hdd_ipa_uc_rt_debug_deinit(hdd_context_t *pHddCtx)
{
	struct hdd_ipa_priv *hdd_ipa = pHddCtx->hdd_ipa;

	if ( VOS_TIMER_STATE_STOPPED !=
		vos_timer_getCurrentState(&hdd_ipa->rt_debug_fill_timer)) {
		vos_timer_stop(&hdd_ipa->rt_debug_fill_timer);
	}
	vos_timer_destroy(&hdd_ipa->rt_debug_fill_timer);
	vos_lock_destroy(&hdd_ipa->rt_debug_lock);

	if (!hdd_ipa_is_rt_debugging_enabled(pHddCtx)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
			"%s: IPA RT debug is not enabled", __func__);
		return;
	}

	if ( VOS_TIMER_STATE_STOPPED !=
		vos_timer_getCurrentState(&hdd_ipa->rt_debug_timer)) {
		vos_timer_stop(&hdd_ipa->rt_debug_timer);
	}
	vos_timer_destroy(&hdd_ipa->rt_debug_timer);
}

/**
 * hdd_ipa_uc_rt_debug_init - intialize resources to handle rt debugging
 * @pHddCtx: hdd main context
 *
 * alloc and initialize all rt debugging resources
 *
 * Return: none
 */
static void hdd_ipa_uc_rt_debug_init(hdd_context_t *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa = hdd_ctx->hdd_ipa;

	/* Histogram intialize by default */
	vos_lock_init(&hdd_ipa->rt_debug_lock);
	vos_timer_init(&hdd_ipa->rt_debug_fill_timer, VOS_TIMER_TYPE_SW,
		hdd_ipa_uc_rt_debug_host_fill, (void *)hdd_ctx);

	hdd_ipa->rt_buf_fill_index = 0;
	vos_mem_zero(hdd_ipa->rt_bug_buffer,
		sizeof(struct uc_rt_debug_info) * HDD_IPA_UC_RT_DEBUG_BUF_COUNT);
	hdd_ipa->ipa_tx_forward = 0;
	hdd_ipa->ipa_rx_discard = 0;
	hdd_ipa->ipa_rx_net_send_count = 0;
	hdd_ipa->ipa_rx_internel_drop_count = 0;
	hdd_ipa->ipa_rx_destructor_count = 0;

	vos_timer_start(&hdd_ipa->rt_debug_fill_timer,
		HDD_IPA_UC_RT_DEBUG_FILL_INTERVAL);

	/* Realtime debug enable only when feature enabled */
	if (!hdd_ipa_is_rt_debugging_enabled(hdd_ctx)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: IPA RT debug is not enabled", __func__);
		return;
	}
	vos_timer_init(&hdd_ipa->rt_debug_timer, VOS_TIMER_TYPE_SW,
		hdd_ipa_uc_rt_debug_handler, (void *)hdd_ctx);

	vos_timer_start(&hdd_ipa->rt_debug_timer,
		HDD_IPA_UC_RT_DEBUG_PERIOD);
}

void hdd_ipa_uc_stat_query(hdd_context_t *pHddCtx,
	uint32_t *ipa_tx_diff, uint32_t *ipa_rx_diff)
{
	struct hdd_ipa_priv *hdd_ipa;

	hdd_ipa = (struct hdd_ipa_priv *)pHddCtx->hdd_ipa;
	*ipa_tx_diff = 0;
	*ipa_rx_diff = 0;

	if (!hdd_ipa_is_enabled(pHddCtx) ||
		!(hdd_ipa_uc_is_enabled(hdd_ipa))) {
		return;
	}

	vos_lock_acquire(&hdd_ipa->ipa_lock);
	if ((HDD_IPA_UC_NUM_WDI_PIPE == hdd_ipa->activated_fw_pipe) &&
		(VOS_FALSE == hdd_ipa->resource_loading)) {
		*ipa_tx_diff = hdd_ipa->ipa_tx_packets_diff;
		*ipa_rx_diff = hdd_ipa->ipa_rx_packets_diff;
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
			"%s: STAT Query TX DIFF %d, RX DIFF %d",
			__func__, *ipa_tx_diff, *ipa_rx_diff);
	}
	vos_lock_release(&hdd_ipa->ipa_lock);
	return;
}

void hdd_ipa_uc_stat_request( hdd_adapter_t *adapter, uint8_t reason)
{
	hdd_context_t *pHddCtx;
	struct hdd_ipa_priv *hdd_ipa;

	if (!adapter) {
		return;
	}

	pHddCtx = (hdd_context_t *)adapter->pHddCtx;
	hdd_ipa = (struct hdd_ipa_priv *)pHddCtx->hdd_ipa;
	if (!hdd_ipa_is_enabled(pHddCtx) ||
		!(hdd_ipa_uc_is_enabled(hdd_ipa))) {
		return;
	}

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: STAT REQ Reason %d",
		__func__, reason);
	vos_lock_acquire(&hdd_ipa->ipa_lock);
	if ((HDD_IPA_UC_NUM_WDI_PIPE == hdd_ipa->activated_fw_pipe) &&
		(VOS_FALSE == hdd_ipa->resource_loading)) {
		hdd_ipa->stat_req_reason = (hdd_ipa_uc_stat_reason)reason;
		process_wma_set_command(
			(int)adapter->sessionId,
			(int)WMA_VDEV_TXRX_GET_IPA_UC_FW_STATS_CMDID,
			0, VDEV_CMD);
	}
	vos_lock_release(&hdd_ipa->ipa_lock);
}

static v_BOOL_t hdd_ipa_uc_find_add_assoc_sta(
	struct hdd_ipa_priv *hdd_ipa,
	v_BOOL_t sta_add,
	uint8_t sta_id)
{
	/* Found associated sta */
	v_BOOL_t sta_found = VOS_FALSE;
	uint8_t idx;

	for (idx = 0; idx < WLAN_MAX_STA_COUNT; idx++) {
		if ((hdd_ipa->assoc_stas_map[idx].is_reserved) &&
			(hdd_ipa->assoc_stas_map[idx].sta_id == sta_id)) {
			sta_found = VOS_TRUE;
			break;
		}
	}

	/* Try to add sta which is already in
	 * If the sta is already in, just return sta_found */
	if (sta_add && sta_found) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: STA ID %d already exist, cannot add",
			__func__, sta_id);
		return sta_found;
	}

	if (sta_add) {
		/* Find first empty slot */
		for (idx = 0; idx < WLAN_MAX_STA_COUNT; idx++) {
			if (!hdd_ipa->assoc_stas_map[idx].is_reserved) {
				hdd_ipa->assoc_stas_map[idx].is_reserved =
					VOS_TRUE;
				hdd_ipa->assoc_stas_map[idx].sta_id = sta_id;
				return sta_found;
			}
		}
	}

	/* Delete STA from map, but could not find STA within the map
	 * Error case, add error log */
	if (!sta_add && !sta_found) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: STA ID %d does not exist, cannot delete",
			__func__, sta_id);
		return sta_found;
	}

	if (!sta_add) {
		for (idx = 0; idx < WLAN_MAX_STA_COUNT; idx++) {
			if ((hdd_ipa->assoc_stas_map[idx].is_reserved) &&
			(hdd_ipa->assoc_stas_map[idx].sta_id == sta_id)) {
				hdd_ipa->assoc_stas_map[idx].is_reserved =
					VOS_FALSE;
				hdd_ipa->assoc_stas_map[idx].sta_id = 0xFF;
				return sta_found;
			}
		}
	}

	return sta_found;
}

static int hdd_ipa_uc_enable_pipes(struct hdd_ipa_priv *hdd_ipa)
{
	int result;

	/* ACTIVATE TX PIPE */
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: Enable TX PIPE(tx_pipe_handle=%d)",
		__func__, hdd_ipa->tx_pipe_handle);
	result = ipa_enable_wdi_pipe(hdd_ipa->tx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Enable TX PIPE fail, code %d",
			__func__, result);
		return result;
	}
	result = ipa_resume_wdi_pipe(hdd_ipa->tx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Resume TX PIPE fail, code %d",
			__func__, result);
		return result;
	}
	WLANTL_SetUcActive(hdd_ipa->hdd_ctx->pvosContext,
		VOS_TRUE, VOS_TRUE);

	/* ACTIVATE RX PIPE */
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: Enable RX PIPE(rx_pipe_handle=%d)"
		, __func__, hdd_ipa->rx_pipe_handle);
	result = ipa_enable_wdi_pipe(hdd_ipa->rx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Enable RX PIPE fail, code %d",
			__func__, result);
		return result;
	}
	result = ipa_resume_wdi_pipe(hdd_ipa->rx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Resume RX PIPE fail, code %d",
			__func__, result);
		return result;
	}
	WLANTL_SetUcActive(hdd_ipa->hdd_ctx->pvosContext,
		VOS_TRUE, VOS_FALSE);

	hdd_ipa->ipa_pipes_down = false;
	return 0;
}

static int hdd_ipa_uc_disable_pipes(struct hdd_ipa_priv *hdd_ipa)
{
	int result;

	hdd_ipa->ipa_pipes_down = true;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: Disable RX PIPE", __func__);
	result = ipa_suspend_wdi_pipe(hdd_ipa->rx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Suspend RX PIPE fail, code %d",
			__func__, result);
		return result;
	}
	result = ipa_disable_wdi_pipe(hdd_ipa->rx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Disable RX PIPE fail, code %d",
			__func__, result);
		return result;
	}

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: Disable TX PIPE", __func__);
	result = ipa_suspend_wdi_pipe(hdd_ipa->tx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Suspend TX PIPE fail, code %d",
			__func__, result);
		return result;
	}
	result = ipa_disable_wdi_pipe(hdd_ipa->tx_pipe_handle);
	if (result) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Disable TX PIPE fail, code %d",
			__func__, result);
		return result;
	}

	return 0;
}

static int hdd_ipa_uc_handle_first_con(struct hdd_ipa_priv *hdd_ipa)
{
	hdd_ipa->activated_fw_pipe = 0;
	hdd_ipa->resource_loading = VOS_TRUE;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "+%s", __func__);

	/* If RM feature enabled
	 * Request PROD Resource first
	 * PROD resource may return sync or async manners */
	if (hdd_ipa_is_rm_enabled(hdd_ipa)) {
		if (!ipa_rm_request_resource(IPA_RM_RESOURCE_WLAN_PROD)) {
			/* RM PROD request sync return
			 * enable pipe immediately */
			if (hdd_ipa_uc_enable_pipes(hdd_ipa)) {
				HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
						"%s: IPA WDI Pipes activate fail", __func__);
				hdd_ipa->resource_loading = VOS_FALSE;
				return -EBUSY;
			}
		}
	} else {
		/* RM Disabled
		 * Just enabled all the PIPEs */
		if (hdd_ipa_uc_enable_pipes(hdd_ipa)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"%s: IPA WDI Pipes activate fail", __func__);
			hdd_ipa->resource_loading = VOS_FALSE;
			return -EBUSY;
		}
	}
	return 0;
}

static int hdd_ipa_uc_handle_last_discon(struct hdd_ipa_priv *hdd_ipa)
{
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "+%s", __func__);

	hdd_ipa->resource_unloading = VOS_TRUE;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: Disable FW RX PIPE", __func__);
	WLANTL_SetUcActive(hdd_ipa->hdd_ctx->pvosContext,
		VOS_FALSE, VOS_FALSE);
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: Disable FW TX PIPE", __func__);
	WLANTL_SetUcActive(hdd_ipa->hdd_ctx->pvosContext,
		VOS_FALSE, VOS_TRUE);
	return 0;
}

static void hdd_ipa_uc_rm_notify_handler(void *context, enum ipa_rm_event event)
{
	struct hdd_ipa_priv *hdd_ipa = context;
	VOS_STATUS status = VOS_STATUS_SUCCESS;

	/*
	 * When SSR is going on or driver is unloading, just return.
	 */
	status = wlan_hdd_validate_context(hdd_ipa->hdd_ctx);
	if (0 != status) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "HDD context is not valid");
		return;
	}

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		return;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%s, event code %d",
		__func__, event);

	switch (event) {
	case IPA_RM_RESOURCE_GRANTED:
		/* Differed RM Granted */
		vos_lock_acquire(&hdd_ipa->ipa_lock);
		if ((VOS_FALSE == hdd_ipa->resource_unloading) &&
			(!hdd_ipa->activated_fw_pipe)) {
			hdd_ipa_uc_enable_pipes(hdd_ipa);
		}
		vos_lock_release(&hdd_ipa->ipa_lock);
		break;

	case IPA_RM_RESOURCE_RELEASED:
		/* Differed RM Released */
		hdd_ipa->resource_unloading = VOS_FALSE;
		break;

	default:
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s, invalid event code %d",
			__func__, event);
		break;
	}
}

static void hdd_ipa_uc_rm_notify_defer(struct work_struct *work)
{
	enum ipa_rm_event event;
	struct uc_rm_work_struct *uc_rm_work = container_of(work,
			struct uc_rm_work_struct, work);
	struct hdd_ipa_priv *hdd_ipa = container_of(uc_rm_work,
			struct hdd_ipa_priv, uc_rm_work);

	vos_ssr_protect(__func__);
	event = uc_rm_work->event;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_HIGH,
		"%s, posted event %d", __func__, event);

	hdd_ipa_uc_rm_notify_handler(hdd_ipa, event);
	vos_ssr_unprotect(__func__);

	return;
}

static int hdd_ipa_uc_proc_pending_event(struct hdd_ipa_priv *hdd_ipa)
{
	v_SIZE_t pending_event_count;
	struct ipa_uc_pending_event *pending_event = NULL;

	vos_list_size(&hdd_ipa->pending_event, &pending_event_count);
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s, Pending Event Count %d", __func__, pending_event_count);
	if (!pending_event_count) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
			"%s, No Pending Event", __func__);
		return 0;
	}

	vos_list_remove_front(&hdd_ipa->pending_event,
			(vos_list_node_t **)&pending_event);
	while (pending_event != NULL) {
		hdd_ipa_wlan_evt(pending_event->adapter,
			pending_event->type,
			pending_event->sta_id,
			pending_event->mac_addr);
		vos_mem_free(pending_event);
		pending_event = NULL;
		vos_list_remove_front(&hdd_ipa->pending_event,
			(vos_list_node_t **)&pending_event);
	}
	return 0;
}

static void hdd_ipa_uc_loaded_handler(struct hdd_ipa_priv *ipa_ctxt)
{
	hdd_context_t *hdd_ctx = ipa_ctxt->hdd_ctx;
	struct ipa_wdi_out_params pipe_out;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
		"%s : UC READY", __func__);
	if (VOS_TRUE == ipa_ctxt->uc_loaded) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_HIGH,
			"%s : UC already loaded", __func__);
		return;
	}

	ipa_ctxt->uc_loaded = VOS_TRUE;
	/* Connect pipe */
	ipa_connect_wdi_pipe(&ipa_ctxt->cons_pipe_in, &pipe_out);
	ipa_ctxt->tx_pipe_handle = pipe_out.clnt_hdl;
	hdd_ctx->tx_comp_doorbell_paddr = (v_U32_t)pipe_out.uc_door_bell_pa;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
		"%s : TX PIPE Handle %d, DBPA 0x%x",
		__func__, ipa_ctxt->tx_pipe_handle, (v_U32_t)pipe_out.uc_door_bell_pa);

	ipa_connect_wdi_pipe(&ipa_ctxt->prod_pipe_in, &pipe_out);
	ipa_ctxt->rx_pipe_handle = pipe_out.clnt_hdl;
	hdd_ctx->rx_ready_doorbell_paddr = pipe_out.uc_door_bell_pa;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
		"%s : RX PIPE Handle %d, DBPA 0x%x",
		__func__, ipa_ctxt->rx_pipe_handle, (v_U32_t)pipe_out.uc_door_bell_pa);

	/* If already any STA connected, enable IPA/FW PIPEs */
	if (ipa_ctxt->sap_num_connected_sta) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
			"Client already connected, enable IPA/FW PIPEs");
		hdd_ipa_uc_handle_first_con(ipa_ctxt);
	}
}
void hdd_ipa_uc_loaded_uc_cb(void *priv_ctxt)
{
	struct hdd_ipa_priv *hdd_ipa;
	struct op_msg_type *msg;
	struct uc_op_work_struct *uc_op_work;

	if (NULL == priv_ctxt) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Invalid IPA context");
		return;
	}

	hdd_ipa = (struct hdd_ipa_priv *)priv_ctxt;
	msg = (struct op_msg_type *)adf_os_mem_alloc(NULL,
					sizeof(struct op_msg_type));
        if (!msg) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "op_msg allocation fails");
		return;
	}

	msg->op_code = HDD_IPA_UC_OPCODE_UC_READY;

	uc_op_work = &hdd_ipa->uc_op_work[msg->op_code];
	if (uc_op_work->msg)
		/* When the same uC OPCODE is already pended, just return */
		return;

	uc_op_work->msg = msg;
	schedule_work(&uc_op_work->work);
	return;
}

#define HDD_BW_GET_DIFF(_x, _y) (unsigned long)((ULONG_MAX - (_y)) + (_x) + 1)
static void hdd_ipa_uc_op_cb(struct op_msg_type *op_msg, void *usr_ctxt)
{
	struct op_msg_type *msg = op_msg;
	struct ipa_uc_fw_stats *uc_fw_stat;
	struct IpaHwStatsWDIInfoData_t ipa_stat;
	struct hdd_ipa_priv *hdd_ipa;
	hdd_context_t *hdd_ctx;
	VOS_STATUS status = VOS_STATUS_SUCCESS;
	struct ipa_msg_meta meta;
	struct ipa_wlan_msg *ipa_msg;
	int ret = 0;

	if (!op_msg || !usr_ctxt) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s, INVALID ARG", __func__);
		return;
	}

	if (HDD_IPA_UC_OPCODE_MAX <= msg->op_code) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s, INVALID OPCODE %d", __func__, msg->op_code);
		adf_os_mem_free(op_msg);
		return;
	}

	hdd_ctx = (hdd_context_t *)usr_ctxt;
	hdd_ipa = (struct hdd_ipa_priv *)hdd_ctx->hdd_ipa;

	/*
	 * When SSR is going on or driver is unloading, just return.
	 */
	status = wlan_hdd_validate_context(hdd_ctx);
	if (0 != status) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "HDD context is not valid");
		adf_os_mem_free(op_msg);
		return;
	}

	HDD_IPA_LOG(VOS_TRACE_LEVEL_DEBUG,
		"%s, OPCODE %s", __func__, op_string[msg->op_code]);

	if ((HDD_IPA_UC_OPCODE_TX_RESUME == msg->op_code) ||
		(HDD_IPA_UC_OPCODE_RX_RESUME == msg->op_code)) {
		vos_lock_acquire(&hdd_ipa->ipa_lock);
		hdd_ipa->activated_fw_pipe++;
		if (HDD_IPA_UC_NUM_WDI_PIPE == hdd_ipa->activated_fw_pipe) {
			hdd_ipa->resource_loading = VOS_FALSE;
			if (VOS_FALSE == hdd_ipa->wdi_enabled) {
				hdd_ipa->wdi_enabled = VOS_TRUE;
				/* WDI enable message to IPA */
				meta.msg_len = sizeof(*ipa_msg);
				ipa_msg = adf_os_mem_alloc(NULL, meta.msg_len);
				if (ipa_msg == NULL) {
					hddLog(VOS_TRACE_LEVEL_ERROR,
						"msg allocation failed");
					adf_os_mem_free(op_msg);
					vos_lock_release(&hdd_ipa->ipa_lock);
					return;
				}

				meta.msg_type = WLAN_WDI_ENABLE;
				hddLog(VOS_TRACE_LEVEL_INFO,
					"ipa_send_msg(Evt:%d)", meta.msg_type);
				ret = ipa_send_msg(&meta, ipa_msg,
					hdd_ipa_msg_free_fn);
				if (ret) {
					hddLog(VOS_TRACE_LEVEL_ERROR,
						"ipa_send_msg(Evt:%d)-fail=%d",
					meta.msg_type,  ret);
					adf_os_mem_free(ipa_msg);
				}
#ifdef IPA_UC_STA_OFFLOAD
				else {
					/* Send SCC/MCC Switching event to IPA */
					hdd_ipa_send_mcc_scc_msg(hdd_ctx,
						hdd_ctx->mcc_mode);
				}
#endif
			}

			hdd_ipa_uc_proc_pending_event(hdd_ipa);

			if (hdd_ipa->pending_cons_req)
				ipa_rm_notify_completion(
						IPA_RM_RESOURCE_GRANTED,
						IPA_RM_RESOURCE_WLAN_CONS);
			hdd_ipa->pending_cons_req = VOS_FALSE;
		}
		vos_lock_release(&hdd_ipa->ipa_lock);
	} else if ((HDD_IPA_UC_OPCODE_TX_SUSPEND == msg->op_code) ||
		(HDD_IPA_UC_OPCODE_RX_SUSPEND == msg->op_code)) {
		vos_lock_acquire(&hdd_ipa->ipa_lock);
		hdd_ipa->activated_fw_pipe--;
		if (!hdd_ipa->activated_fw_pipe) {
			hdd_ipa_uc_disable_pipes(hdd_ipa);
			if (hdd_ipa_is_rm_enabled(hdd_ipa))
				ipa_rm_release_resource(
					IPA_RM_RESOURCE_WLAN_PROD);
			/* Sync return success from IPA
			 * Enable/resume all the PIPEs */
			hdd_ipa->resource_unloading = VOS_FALSE;
			hdd_ipa_uc_proc_pending_event(hdd_ipa);
			hdd_ipa->pending_cons_req = VOS_FALSE;
		}
		vos_lock_release(&hdd_ipa->ipa_lock);
	}

	if ((HDD_IPA_UC_OPCODE_STATS == msg->op_code) &&
		(HDD_IPA_UC_STAT_REASON_DEBUG == hdd_ipa->stat_req_reason)) {
		/* STATs from host */
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC WLAN_HOST CE ====\n"
			"CE RING BASE: 0x%x\n"
			"CE RING SIZE: %d\n"
			"CE REG ADDR : 0x%x",
			hdd_ctx->ce_sr_base_paddr,
			hdd_ctx->ce_sr_ring_size,
			hdd_ctx->ce_reg_paddr);
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC WLAN_HOST TX ====\n"
			"COMP RING BASE: 0x%x\n"
			"COMP RING SIZE: %d\n"
			"NUM ALLOC BUF: %d\n"
			"COMP RING DBELL : 0x%x",
			hdd_ctx->tx_comp_ring_base_paddr,
			hdd_ctx->tx_comp_ring_size,
			hdd_ctx->tx_num_alloc_buffer,
			hdd_ctx->tx_comp_doorbell_paddr);
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC WLAN_HOST RX ====\n"
			"IND RING BASE: 0x%x\n"
			"IND RING SIZE: %d\n"
			"IND RING DBELL : 0x%x\n"
			"PROC DONE IND ADDR : 0x%x\n"
			"NUM EXCP PKT : %llu\n"
			"NUM TX BCMC : %llu\n"
			"NUM TX BCMC ERR : %llu",
			hdd_ctx->rx_rdy_ring_base_paddr,
			hdd_ctx->rx_rdy_ring_size,
			hdd_ctx->rx_ready_doorbell_paddr,
			hdd_ctx->rx_proc_done_idx_paddr,
			hdd_ipa->stats.num_rx_excep,
			hdd_ipa->stats.num_tx_bcmc,
			hdd_ipa->stats.num_tx_bcmc_err);
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC WLAN_HOST CONTROL ====\n"
			"SAP NUM STAs: %d\n"
#ifdef IPA_UC_STA_OFFLOAD
			"STA CONNECTED: %d\n"
#endif
			"TX PIPE HDL: %d\n"
			"RX PIPE HDL : %d\n"
			"RSC LOADING : %d\n"
			"RSC UNLOADING : %d\n"
			"PNDNG CNS RQT : %d",
			hdd_ipa->sap_num_connected_sta,
#ifdef IPA_UC_STA_OFFLOAD
			hdd_ipa->sta_connected,
#endif
			hdd_ipa->tx_pipe_handle,
			hdd_ipa->rx_pipe_handle,
			(unsigned int)hdd_ipa->resource_loading,
			(unsigned int)hdd_ipa->resource_unloading,
			(unsigned int)hdd_ipa->pending_cons_req);

		/* STATs from FW */
		uc_fw_stat = (struct ipa_uc_fw_stats *)
			((v_U8_t *)op_msg + sizeof(struct op_msg_type));
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC WLAN_FW TX ====\n"
			"COMP RING BASE: 0x%x\n"
			"COMP RING SIZE: %d\n"
			"COMP RING DBELL : 0x%x\n"
			"COMP RING DBELL IND VAL : %d\n"
			"COMP RING DBELL CACHED VAL : %d\n"
			"COMP RING DBELL CACHED VAL : %d\n"
			"PKTS ENQ : %d\n"
			"PKTS COMP : %d\n"
			"IS SUSPEND : %d\n"
			"RSVD : 0x%x",
			uc_fw_stat->tx_comp_ring_base,
			uc_fw_stat->tx_comp_ring_size,
			uc_fw_stat->tx_comp_ring_dbell_addr,
			uc_fw_stat->tx_comp_ring_dbell_ind_val,
			uc_fw_stat->tx_comp_ring_dbell_cached_val,
			uc_fw_stat->tx_comp_ring_dbell_cached_val,
			uc_fw_stat->tx_pkts_enqueued,
			uc_fw_stat->tx_pkts_completed,
			uc_fw_stat->tx_is_suspend,
			uc_fw_stat->tx_reserved);
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC WLAN_FW RX ====\n"
			"IND RING BASE: 0x%x\n"
			"IND RING SIZE: %d\n"
			"IND RING DBELL : 0x%x\n"
			"IND RING DBELL IND VAL : %d\n"
			"IND RING DBELL CACHED VAL : %d\n"
			"RDY IND ADDR : 0x%x\n"
			"RDY IND CACHE VAL : %d\n"
			"RFIL IND : %d\n"
			"NUM PKT INDICAT : %d\n"
			"BUF REFIL : %d\n"
			"NUM DROP NO SPC : %d\n"
			"NUM DROP NO BUF : %d\n"
			"IS SUSPND : %d\n"
			"RSVD : 0x%x\n",
			uc_fw_stat->rx_ind_ring_base,
			uc_fw_stat->rx_ind_ring_size,
			uc_fw_stat->rx_ind_ring_dbell_addr,
			uc_fw_stat->rx_ind_ring_dbell_ind_val,
			uc_fw_stat->rx_ind_ring_dbell_ind_cached_val,
			uc_fw_stat->rx_ind_ring_rdidx_addr,
			uc_fw_stat->rx_ind_ring_rd_idx_cached_val,
			uc_fw_stat->rx_refill_idx,
			uc_fw_stat->rx_num_pkts_indicated,
			uc_fw_stat->rx_buf_refilled,
			uc_fw_stat->rx_num_ind_drop_no_space,
			uc_fw_stat->rx_num_ind_drop_no_buf,
			uc_fw_stat->rx_is_suspend,
			uc_fw_stat->rx_reserved);
		/* STATs from IPA */
		ipa_get_wdi_stats(&ipa_stat);
		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC IPA TX ====\n"
			"NUM PROCD : %d\n"
			"CE DBELL : 0x%x\n"
			"NUM DBELL FIRED : %d\n"
			"COMP RNG FULL : %d\n"
			"COMP RNG EMPT : %d\n"
			"COMP RNG USE HGH : %d\n"
			"COMP RNG USE LOW : %d\n"
			"BAM FIFO FULL : %d\n"
			"BAM FIFO EMPT : %d\n"
			"BAM FIFO USE HGH : %d\n"
			"BAM FIFO USE LOW : %d\n"
			"NUM DBELL : %d\n"
			"NUM UNEXP DBELL : %d\n"
			"NUM BAM INT HDL : 0x%x\n"
			"NUM BAM INT NON-RUN : 0x%x\n"
			"NUM QMB INT HDL : 0x%x",
			ipa_stat.tx_ch_stats.num_pkts_processed,
			ipa_stat.tx_ch_stats.copy_engine_doorbell_value,
			ipa_stat.tx_ch_stats.num_db_fired,
			ipa_stat.tx_ch_stats.tx_comp_ring_stats.ringFull,
			ipa_stat.tx_ch_stats.tx_comp_ring_stats.ringEmpty,
			ipa_stat.tx_ch_stats.tx_comp_ring_stats.ringUsageHigh,
			ipa_stat.tx_ch_stats.tx_comp_ring_stats.ringUsageLow,
			ipa_stat.tx_ch_stats.bam_stats.bamFifoFull,
			ipa_stat.tx_ch_stats.bam_stats.bamFifoEmpty,
			ipa_stat.tx_ch_stats.bam_stats.bamFifoUsageHigh,
			ipa_stat.tx_ch_stats.bam_stats.bamFifoUsageLow,
			ipa_stat.tx_ch_stats.num_db,
			ipa_stat.tx_ch_stats.num_unexpected_db,
			ipa_stat.tx_ch_stats.num_bam_int_handled,
			ipa_stat.tx_ch_stats.num_bam_int_in_non_runnning_state,
			ipa_stat.tx_ch_stats.num_qmb_int_handled);

		VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
			"==== IPA_UC IPA RX ====\n"
			"MAX OST PKT : %d\n"
			"NUM PKT PRCSD : %d\n"
			"RNG RP : 0x%x\n"
			"COMP RNG FULL : %d\n"
			"COMP RNG EMPT : %d\n"
			"COMP RNG USE HGH : %d\n"
			"COMP RNG USE LOW : %d\n"
			"BAM FIFO FULL : %d\n"
			"BAM FIFO EMPT : %d\n"
			"BAM FIFO USE HGH : %d\n"
			"BAM FIFO USE LOW : %d\n"
			"NUM DB : %d\n"
			"NUM UNEXP DB : %d\n"
			"NUM BAM INT HNDL : 0x%x\n",
			ipa_stat.rx_ch_stats.max_outstanding_pkts,
			ipa_stat.rx_ch_stats.num_pkts_processed,
			ipa_stat.rx_ch_stats.rx_ring_rp_value,
			ipa_stat.rx_ch_stats.rx_ind_ring_stats.ringFull,
			ipa_stat.rx_ch_stats.rx_ind_ring_stats.ringEmpty,
			ipa_stat.rx_ch_stats.rx_ind_ring_stats.ringUsageHigh,
			ipa_stat.rx_ch_stats.rx_ind_ring_stats.ringUsageLow,
			ipa_stat.rx_ch_stats.bam_stats.bamFifoFull,
			ipa_stat.rx_ch_stats.bam_stats.bamFifoEmpty,
			ipa_stat.rx_ch_stats.bam_stats.bamFifoUsageHigh,
			ipa_stat.rx_ch_stats.bam_stats.bamFifoUsageLow,
			ipa_stat.rx_ch_stats.num_db,
			ipa_stat.rx_ch_stats.num_unexpected_db,
			ipa_stat.rx_ch_stats.num_bam_int_handled);
	} else if ((HDD_IPA_UC_OPCODE_STATS == msg->op_code) &&
		(HDD_IPA_UC_STAT_REASON_BW_CAL == hdd_ipa->stat_req_reason)) {
		/* STATs from FW */
		uc_fw_stat = (struct ipa_uc_fw_stats *)
			((v_U8_t *)op_msg + sizeof(struct op_msg_type));
		vos_lock_acquire(&hdd_ipa->ipa_lock);
		hdd_ipa->ipa_tx_packets_diff = HDD_BW_GET_DIFF(
			uc_fw_stat->tx_pkts_completed,
			hdd_ipa->ipa_p_tx_packets);
		hdd_ipa->ipa_rx_packets_diff = HDD_BW_GET_DIFF(
			(uc_fw_stat->rx_num_ind_drop_no_space +
			uc_fw_stat->rx_num_ind_drop_no_buf +
			uc_fw_stat->rx_num_pkts_indicated),
			hdd_ipa->ipa_p_rx_packets);

		hdd_ipa->ipa_p_tx_packets = uc_fw_stat->tx_pkts_completed;
		hdd_ipa->ipa_p_rx_packets =
			(uc_fw_stat->rx_num_ind_drop_no_space +
			uc_fw_stat->rx_num_ind_drop_no_buf +
			uc_fw_stat->rx_num_pkts_indicated);
		vos_lock_release(&hdd_ipa->ipa_lock);
	} else if (HDD_IPA_UC_OPCODE_UC_READY == msg->op_code) {
		vos_lock_acquire(&hdd_ipa->ipa_lock);
		hdd_ipa_uc_loaded_handler(hdd_ipa);
		vos_lock_release(&hdd_ipa->ipa_lock);
	}

	adf_os_mem_free(op_msg);
}

static void hdd_ipa_uc_fw_op_event_handler(struct work_struct *work)
{
	struct op_msg_type *msg;
	struct uc_op_work_struct *uc_op_work = container_of(work,
			struct uc_op_work_struct, work);
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;

	vos_ssr_protect(__func__);

	msg = uc_op_work->msg;
	uc_op_work->msg = NULL;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_HIGH,
			"%s, posted msg %d", __func__, msg->op_code);

	hdd_ipa_uc_op_cb(msg, hdd_ipa->hdd_ctx);

	vos_ssr_unprotect(__func__);

	return;
}

static void hdd_ipa_uc_op_event_handler(v_U8_t *op_msg, void *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa;
	struct op_msg_type *msg;
	struct uc_op_work_struct *uc_op_work;

	if (NULL == hdd_ctx) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Invalid HDD context");
		goto end;
	}

	msg = (struct op_msg_type *)op_msg;
	hdd_ipa = ((hdd_context_t *)hdd_ctx)->hdd_ipa;

	if (unlikely(!hdd_ipa))
		goto end;

	if (HDD_IPA_UC_OPCODE_MAX <= msg->op_code) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "%s: Invalid OP Code (%d)",
				__func__, msg->op_code);
		goto end;
	}

	uc_op_work = &hdd_ipa->uc_op_work[msg->op_code];
	if (uc_op_work->msg)
		/* When the same uC OPCODE is already pended, just return */
		goto end;

	uc_op_work->msg = msg;
	schedule_work(&uc_op_work->work);
	return;

end:
	adf_os_mem_free(op_msg);
}

static VOS_STATUS hdd_ipa_uc_ol_init(hdd_context_t *hdd_ctx)
{
	struct ipa_wdi_in_params  pipe_in;
	struct ipa_wdi_out_params pipe_out;
	struct hdd_ipa_priv *ipa_ctxt = (struct hdd_ipa_priv *)hdd_ctx->hdd_ipa;
	uint8_t i;
	struct ipa_wdi_db_params dbpa;

	vos_mem_zero(&ipa_ctxt->cons_pipe_in, sizeof(struct ipa_wdi_in_params));
	vos_mem_zero(&ipa_ctxt->prod_pipe_in, sizeof(struct ipa_wdi_in_params));

	vos_mem_zero(&pipe_in, sizeof(struct ipa_wdi_in_params));
	vos_mem_zero(&pipe_out, sizeof(struct ipa_wdi_out_params));

	vos_list_init(&ipa_ctxt->pending_event);
	vos_lock_init(&ipa_ctxt->event_lock);
	vos_lock_init(&ipa_ctxt->ipa_lock);

	/* TX PIPE */
	pipe_in.sys.ipa_ep_cfg.nat.nat_en = IPA_BYPASS_NAT;
	pipe_in.sys.ipa_ep_cfg.hdr.hdr_len = HDD_IPA_UC_WLAN_TX_HDR_LEN;
	pipe_in.sys.ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid = 1;
	pipe_in.sys.ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 0;
	pipe_in.sys.ipa_ep_cfg.hdr.hdr_additional_const_len =
		HDD_IPA_UC_WLAN_8023_HDR_SIZE;
	pipe_in.sys.ipa_ep_cfg.mode.mode = IPA_BASIC;
	pipe_in.sys.client = IPA_CLIENT_WLAN1_CONS;
	pipe_in.sys.desc_fifo_sz = hdd_ctx->cfg_ini->IpaDescSize;
	pipe_in.sys.priv = hdd_ctx->hdd_ipa;
	pipe_in.sys.ipa_ep_cfg.hdr_ext.hdr_little_endian = true;
	pipe_in.sys.notify = hdd_ipa_i2w_cb;
	if (!hdd_ipa_is_rm_enabled(ghdd_ipa)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
			"%s: IPA RM DISABLED, IPA AWAKE", __func__);
		pipe_in.sys.keep_ipa_awake = TRUE;
	}

	pipe_in.u.dl.comp_ring_base_pa = hdd_ctx->tx_comp_ring_base_paddr;
	pipe_in.u.dl.comp_ring_size = hdd_ctx->tx_comp_ring_size * 4;
	pipe_in.u.dl.ce_ring_base_pa = hdd_ctx->ce_sr_base_paddr;
	pipe_in.u.dl.ce_door_bell_pa = hdd_ctx->ce_reg_paddr;
	pipe_in.u.dl.ce_ring_size = hdd_ctx->ce_sr_ring_size * 8;
	pipe_in.u.dl.num_tx_buffers  = hdd_ctx->tx_num_alloc_buffer;

	vos_mem_copy(&ipa_ctxt->cons_pipe_in,
		&pipe_in,
		sizeof(struct ipa_wdi_in_params));
	dbpa.client = IPA_CLIENT_WLAN1_CONS;
	ipa_uc_wdi_get_dbpa(&dbpa);
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s CONS DB get dbpa 0x%x",
		__func__, (unsigned int)dbpa.uc_door_bell_pa);
	hdd_ctx->tx_comp_doorbell_paddr = dbpa.uc_door_bell_pa;
	if (VOS_TRUE == ipa_ctxt->uc_loaded) {
		/* Connect WDI IPA PIPE */
		ipa_connect_wdi_pipe(&ipa_ctxt->cons_pipe_in, &pipe_out);
		/* Micro Controller Doorbell register */
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s CONS DB pipe out 0x%x",
			__func__, (unsigned int)pipe_out.uc_door_bell_pa);

		hdd_ctx->tx_comp_doorbell_paddr = (v_U32_t)pipe_out.uc_door_bell_pa;
		/* WLAN TX PIPE Handle */
		ipa_ctxt->tx_pipe_handle = pipe_out.clnt_hdl;
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_HIGH,
			"TX : CRBPA 0x%x, CRS %d, CERBPA 0x%x, CEDPA 0x%x,"
			" CERZ %d, NB %d, CDBPAD 0x%x",
			(unsigned int)pipe_in.u.dl.comp_ring_base_pa,
			pipe_in.u.dl.comp_ring_size,
			(unsigned int)pipe_in.u.dl.ce_ring_base_pa,
			(unsigned int)pipe_in.u.dl.ce_door_bell_pa,
			pipe_in.u.dl.ce_ring_size,
			pipe_in.u.dl.num_tx_buffers,
			(unsigned int)hdd_ctx->tx_comp_doorbell_paddr);
	}

	/* RX PIPE */
	pipe_in.sys.ipa_ep_cfg.nat.nat_en = IPA_BYPASS_NAT;
	pipe_in.sys.ipa_ep_cfg.hdr.hdr_len = HDD_IPA_UC_WLAN_RX_HDR_LEN;
	pipe_in.sys.ipa_ep_cfg.hdr.hdr_ofst_metadata_valid = 0;
	pipe_in.sys.ipa_ep_cfg.hdr.hdr_metadata_reg_valid = 1;
	pipe_in.sys.ipa_ep_cfg.mode.mode = IPA_BASIC;
	pipe_in.sys.client = IPA_CLIENT_WLAN1_PROD;
	pipe_in.sys.desc_fifo_sz = hdd_ctx->cfg_ini->IpaDescSize +
				sizeof(struct sps_iovec);
	pipe_in.sys.notify = hdd_ipa_w2i_cb;
	if (!hdd_ipa_is_rm_enabled(ghdd_ipa)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: IPA RM DISABLED, IPA AWAKE", __func__);
		pipe_in.sys.keep_ipa_awake = TRUE;
	}

	pipe_in.u.ul.rdy_ring_base_pa = hdd_ctx->rx_rdy_ring_base_paddr;
	pipe_in.u.ul.rdy_ring_size = hdd_ctx->rx_rdy_ring_size;
	pipe_in.u.ul.rdy_ring_rp_pa = hdd_ctx->rx_proc_done_idx_paddr;

	vos_mem_copy(&ipa_ctxt->prod_pipe_in,
		&pipe_in,
		sizeof(struct ipa_wdi_in_params));
	dbpa.client = IPA_CLIENT_WLAN1_PROD;
	ipa_uc_wdi_get_dbpa(&dbpa);
	hdd_ctx->rx_ready_doorbell_paddr = dbpa.uc_door_bell_pa;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s PROD DB get dbpa 0x%x",
		__func__, (unsigned int)dbpa.uc_door_bell_pa);
	if (VOS_TRUE == ipa_ctxt->uc_loaded) {
		ipa_connect_wdi_pipe(&ipa_ctxt->prod_pipe_in, &pipe_out);
		hdd_ctx->rx_ready_doorbell_paddr = pipe_out.uc_door_bell_pa;
		ipa_ctxt->rx_pipe_handle = pipe_out.clnt_hdl;
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s PROD DB pipe out 0x%x",
			__func__, (unsigned int)pipe_out.uc_door_bell_pa);
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_HIGH,
			"RX : RRBPA 0x%x, RRS %d, PDIPA 0x%x, RDY_DB_PAD 0x%x",
			(unsigned int)pipe_in.u.ul.rdy_ring_base_pa,
			pipe_in.u.ul.rdy_ring_size,
			(unsigned int)pipe_in.u.ul.rdy_ring_rp_pa,
			(unsigned int)hdd_ctx->rx_ready_doorbell_paddr);
	}
	WLANTL_SetUcDoorbellPaddr((pVosContextType)(hdd_ctx->pvosContext),
			(v_U32_t)hdd_ctx->tx_comp_doorbell_paddr,
			(v_U32_t)hdd_ctx->rx_ready_doorbell_paddr);

	WLANTL_RegisterOPCbFnc((pVosContextType)(hdd_ctx->pvosContext),
			hdd_ipa_uc_op_event_handler, (void *)hdd_ctx);

	for (i = 0; i < HDD_IPA_UC_OPCODE_MAX; i++) {
		cnss_init_work(&ipa_ctxt->uc_op_work[i].work,
			hdd_ipa_uc_fw_op_event_handler);
		ipa_ctxt->uc_op_work[i].msg = NULL;
	}

	return VOS_STATUS_SUCCESS;
}

/**
 * hdd_ipa_uc_force_pipe_shutdown() - Force shutdown IPA pipe
 * @hdd_ctx: hdd main context
 *
 * Force shutdown IPA pipe
 * Independent of FW pipe status, IPA pipe shutdonw progress
 * in case, any STA does not leave properly, IPA HW pipe should cleaned up
 * independent from FW pipe status
 *
 * Return: NONE
 */
void hdd_ipa_uc_force_pipe_shutdown(hdd_context_t *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa;

	if (!hdd_ipa_is_enabled(hdd_ctx) || !hdd_ctx->hdd_ipa)
		return;

	hdd_ipa = (struct hdd_ipa_priv *)hdd_ctx->hdd_ipa;
	if (false == hdd_ipa->ipa_pipes_down) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"IPA pipes are not down yet, force shutdown");
		hdd_ipa_uc_disable_pipes(hdd_ipa);
	} else {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
			"IPA pipes are down, do nothing");
	}

	return;
}

/**
 * hdd_ipa_uc_ssr_deinit() - handle ipa deinit for SSR
 *
 * Deinit basic IPA UC host side to be in sync reloaded FW during
 * SSR
 *
 * Return: 0 - Success
 */
int hdd_ipa_uc_ssr_deinit()
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	int idx;
	struct hdd_ipa_iface_context *iface_context;

	if (!hdd_ipa_uc_is_enabled(hdd_ipa))
		return 0;

	/* Clean up HDD IPA interfaces */
	for (idx = 0; (hdd_ipa->num_iface > 0) &&
		(idx < HDD_IPA_MAX_IFACE); idx++) {
		iface_context = &hdd_ipa->iface_context[idx];
		if (iface_context && iface_context->adapter)
			hdd_ipa_cleanup_iface(iface_context);
	}

	/* After SSR, wlan driver reloads FW again. But we need to protect
	 * IPA submodule during SSR transient state. So deinit basic IPA
	 * UC host side to be in sync with reloaded FW during SSR
	 */
	hdd_ipa_uc_disable_pipes(hdd_ipa);

	vos_lock_acquire(&hdd_ipa->ipa_lock);
	for (idx = 0; idx < WLAN_MAX_STA_COUNT; idx++) {
		hdd_ipa->assoc_stas_map[idx].is_reserved = false;
		hdd_ipa->assoc_stas_map[idx].sta_id = 0xFF;
	}
	vos_lock_release(&hdd_ipa->ipa_lock);

	/* Full IPA driver cleanup not required since wlan driver is now
	 * unloaded and reloaded after SSR.
	 */
	return 0;
}

/**
 * hdd_ipa_uc_ssr_reinit() - handle ipa reinit after SSR
 *
 * Init basic IPA UC host side to be in sync with reloaded FW after
 * SSR to resume IPA UC operations
 *
 * Return: 0 - Success
 */
int hdd_ipa_uc_ssr_reinit()
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;

	if (!hdd_ipa_uc_is_enabled(hdd_ipa))
		return 0;

	/* After SSR is complete, IPA UC can resume operation. But now wlan
	 * driver will be unloaded and reloaded, which takes care of IPA cleanup
	 * and initialization.
	 * This is a placeholder func if IPA has to resume operations without
	 * driver reload.
	 */
	return 0;
}
#else
/**
 * hdd_ipa_uc_rt_debug_destructor - called by data packet free
 * @skb: packet pinter
 *
 * when free data packet, will be invoked by wlan client and will increase
 * free counter
 *
 * Return: none
 */
void hdd_ipa_uc_rt_debug_destructor(struct sk_buff *skb)
{
	return;
}
#endif /* IPA_UC_OFFLOAD */

static int hdd_ipa_rm_request(struct hdd_ipa_priv *hdd_ipa)
{
	int ret = 0;

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		return 0;

	adf_os_spin_lock_bh(&hdd_ipa->rm_lock);

	switch(hdd_ipa->rm_state) {
	case HDD_IPA_RM_GRANTED:
		adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);
		return 0;
	case HDD_IPA_RM_GRANT_PENDING:
		adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);
		return -EINPROGRESS;
	case HDD_IPA_RM_RELEASED:
		hdd_ipa->rm_state = HDD_IPA_RM_GRANT_PENDING;
		break;
	}

	adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);

	ret = ipa_rm_inactivity_timer_request_resource(
			IPA_RM_RESOURCE_WLAN_PROD);

	adf_os_spin_lock_bh(&hdd_ipa->rm_lock);
	if (ret == 0) {
		hdd_ipa->rm_state = HDD_IPA_RM_GRANTED;
		hdd_ipa->stats.num_rm_grant_imm++;
	}

	cancel_delayed_work(&hdd_ipa->wake_lock_work);
	if (hdd_ipa->wake_lock_released) {
		vos_wake_lock_acquire(&hdd_ipa->wake_lock,
                                      WIFI_POWER_EVENT_WAKELOCK_IPA);
		hdd_ipa->wake_lock_released = false;
	}
	adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);

	return ret;
}

static void hdd_ipa_wake_lock_timer_func(struct work_struct *work)
{
	struct hdd_ipa_priv *hdd_ipa = container_of(to_delayed_work(work),
			struct hdd_ipa_priv, wake_lock_work);

	adf_os_spin_lock_bh(&hdd_ipa->rm_lock);

	if (hdd_ipa->rm_state != HDD_IPA_RM_RELEASED)
		goto end;

	hdd_ipa->wake_lock_released = true;
	vos_wake_lock_release(&hdd_ipa->wake_lock,
			      WIFI_POWER_EVENT_WAKELOCK_IPA);

end:
	adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);
}

static int hdd_ipa_rm_try_release(struct hdd_ipa_priv *hdd_ipa)
{
	int ret = 0;

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		return 0;

	if (atomic_read(&hdd_ipa->tx_ref_cnt))
		return -EAGAIN;

#ifndef IPA_UC_STA_OFFLOAD
	spin_lock_bh(&hdd_ipa->q_lock);
	if (hdd_ipa->pending_hw_desc_cnt || hdd_ipa->pend_q_cnt) {
		spin_unlock_bh(&hdd_ipa->q_lock);
		return -EAGAIN;
	}
	spin_unlock_bh(&hdd_ipa->q_lock);
#endif

	adf_os_spin_lock_bh(&hdd_ipa->pm_lock);

	if (!adf_nbuf_is_queue_empty(&hdd_ipa->pm_queue_head)) {
		adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);
		return -EAGAIN;
	}
	adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);


	adf_os_spin_lock_bh(&hdd_ipa->rm_lock);
	switch(hdd_ipa->rm_state) {
	case HDD_IPA_RM_GRANTED:
		break;
	case HDD_IPA_RM_GRANT_PENDING:
		adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);
		return -EINPROGRESS;
	case HDD_IPA_RM_RELEASED:
		adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);
		return 0;
	}

	/* IPA driver returns immediately so set the state here to avoid any
	 * race condition.
	 */
	hdd_ipa->rm_state = HDD_IPA_RM_RELEASED;
	hdd_ipa->stats.num_rm_release++;
	adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);

	ret = ipa_rm_inactivity_timer_release_resource(
			IPA_RM_RESOURCE_WLAN_PROD);

	adf_os_spin_lock_bh(&hdd_ipa->rm_lock);
	if (unlikely(ret != 0)) {
		hdd_ipa->rm_state = HDD_IPA_RM_GRANTED;
		WARN_ON(1);
	}

	/*
	 * If wake_lock is released immediately, kernel would try to suspend
	 * immediately as well, Just avoid ping-pong between suspend-resume
	 * while there is healthy amount of data transfer going on by
	 * releasing the wake_lock after some delay.
	 */
	schedule_delayed_work(&hdd_ipa->wake_lock_work,
			msecs_to_jiffies(HDD_IPA_RX_INACTIVITY_MSEC_DELAY));

	adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);

	return ret;
}

static void hdd_ipa_send_pkt_to_ipa(struct hdd_ipa_priv *hdd_ipa)
{
	struct ipa_tx_data_desc *send_desc, *desc, *tmp;
	uint32_t cur_send_cnt = 0, pend_q_cnt;
	adf_nbuf_t buf;
	struct ipa_tx_data_desc *send_desc_head = NULL;

	/* Unloading is in progress so do not proceed to send the packets to
	 * IPA
	 */
	if (hdd_ipa->hdd_ctx->isUnloadInProgress)
		return;

	/* Make it priority queue request as send descriptor */
	send_desc_head = hdd_ipa_alloc_data_desc(hdd_ipa, 1);

	/* Try again later  when descriptors are available */
	if (!send_desc_head)
		return;

	INIT_LIST_HEAD(&send_desc_head->link);

	spin_lock_bh(&hdd_ipa->q_lock);

	if (hdd_ipa->pending_hw_desc_cnt >= hdd_ipa->hw_desc_cnt) {
		hdd_ipa->stats.num_rx_ipa_hw_maxed_out++;
		spin_unlock_bh(&hdd_ipa->q_lock);
		hdd_ipa_free_data_desc(hdd_ipa, send_desc_head);
		return;
	}

	pend_q_cnt = hdd_ipa->pend_q_cnt;

	if (pend_q_cnt == 0) {
		spin_unlock_bh(&hdd_ipa->q_lock);
		hdd_ipa_free_data_desc(hdd_ipa, send_desc_head);
		return;
	}

	/* If hardware has more room than what is pending in the queue update
	 * the send_desc_head right away without going through the loop
	 */
	if ((hdd_ipa->pending_hw_desc_cnt + pend_q_cnt) <
			hdd_ipa->hw_desc_cnt) {
		list_splice_tail_init(&hdd_ipa->pend_desc_head,
				&send_desc_head->link);
		cur_send_cnt = pend_q_cnt;
		hdd_ipa->pend_q_cnt = 0;
		hdd_ipa->stats.num_rx_ipa_splice++;
	} else {
		while (((hdd_ipa->pending_hw_desc_cnt + cur_send_cnt) <
					hdd_ipa->hw_desc_cnt) && pend_q_cnt > 0)
		{
			send_desc = list_first_entry(&hdd_ipa->pend_desc_head,
					struct ipa_tx_data_desc, link);
			list_del(&send_desc->link);
			list_add_tail(&send_desc->link, &send_desc_head->link);
			cur_send_cnt++;
			pend_q_cnt--;
		}
		hdd_ipa->stats.num_rx_ipa_loop++;

		hdd_ipa->pend_q_cnt -= cur_send_cnt;

		VOS_ASSERT(hdd_ipa->pend_q_cnt == pend_q_cnt);
	}

	hdd_ipa->pending_hw_desc_cnt += cur_send_cnt;
	spin_unlock_bh(&hdd_ipa->q_lock);

	if (ipa_tx_dp_mul(hdd_ipa->prod_client, send_desc_head) != 0) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"ipa_tx_dp_mul failed: %u, q_cnt: %u!",
				hdd_ipa->pending_hw_desc_cnt,
				hdd_ipa->pend_q_cnt);
		goto ipa_tx_failed;
	}

	hdd_ipa->stats.num_rx_ipa_tx_dp += cur_send_cnt;
	if (cur_send_cnt > hdd_ipa->stats.num_max_ipa_tx_mul)
		hdd_ipa->stats.num_max_ipa_tx_mul = cur_send_cnt;

	return;

ipa_tx_failed:

	spin_lock_bh(&hdd_ipa->q_lock);
	hdd_ipa->pending_hw_desc_cnt -= cur_send_cnt;
	spin_unlock_bh(&hdd_ipa->q_lock);

	list_for_each_entry_safe(desc, tmp, &send_desc_head->link, link) {
		list_del(&desc->link);
		buf = desc->priv;
		adf_nbuf_free(buf);
		hdd_ipa_free_data_desc(hdd_ipa, desc);
		hdd_ipa->stats.num_rx_ipa_tx_dp_err++;
	}

	/* Return anchor node */
	hdd_ipa_free_data_desc(hdd_ipa, send_desc_head);
}


static void hdd_ipa_rm_send_pkt_to_ipa(struct work_struct *work)
{
	struct hdd_ipa_priv *hdd_ipa = container_of(work,
			struct hdd_ipa_priv, rm_work);

	return hdd_ipa_send_pkt_to_ipa(hdd_ipa);
}

static void hdd_ipa_rm_notify(void *user_data, enum ipa_rm_event event,
		unsigned long data)
{
	struct hdd_ipa_priv *hdd_ipa = user_data;

	if (unlikely(!hdd_ipa))
		return;

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		return;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "Evt: %d", event);

	switch(event) {
	case IPA_RM_RESOURCE_GRANTED:
#ifdef IPA_UC_OFFLOAD
		if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
			/* RM Notification comes with ISR context
			 * it should be serialized into work queue to avoid
			 * ISR sleep problem */
			hdd_ipa->uc_rm_work.event = event;
			schedule_work(&hdd_ipa->uc_rm_work.work);
			break;
		}
#endif /* IPA_UC_OFFLOAD */
		adf_os_spin_lock_bh(&hdd_ipa->rm_lock);
		hdd_ipa->rm_state = HDD_IPA_RM_GRANTED;
		adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);
		hdd_ipa->stats.num_rm_grant++;

		schedule_work(&hdd_ipa->rm_work);
		break;
	case IPA_RM_RESOURCE_RELEASED:
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "RM Release");
#ifdef IPA_UC_OFFLOAD
		hdd_ipa->resource_unloading = VOS_FALSE;
#endif /* IPA_UC_OFFLOAD */
		break;
	default:
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Unknown RM Evt: %d", event);
		break;
	}
}

static int hdd_ipa_rm_cons_release(void)
{
#ifdef IPA_UC_OFFLOAD
	/* Do Nothing */
#endif /* IPA_UC_OFFLOAD */
	return 0;
}

static int hdd_ipa_rm_cons_request(void)
{
	int ret = 0;

#ifdef IPA_UC_OFFLOAD
	if (ghdd_ipa->resource_loading) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_FATAL,
			"%s: ipa resource loading in progress",
			__func__);
		ghdd_ipa->pending_cons_req = VOS_TRUE;
		ret= -EINPROGRESS;
	} else if (ghdd_ipa->resource_unloading) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_FATAL,
			"%s: ipa resource unloading in progress",
			__func__);
		ghdd_ipa->pending_cons_req = VOS_TRUE;
		ret = -EPERM;
	}
#endif /* IPA_UC_OFFLOAD */
	return ret;
}

int hdd_ipa_set_perf_level(hdd_context_t *hdd_ctx, uint64_t tx_packets,
		uint64_t rx_packets)
{
	uint32_t  next_cons_bw, next_prod_bw;
	struct hdd_ipa_priv *hdd_ipa = hdd_ctx->hdd_ipa;
	struct ipa_rm_perf_profile profile;
	int ret;

	if ((!hdd_ipa_is_enabled(hdd_ctx)) ||
		(!hdd_ipa_is_clk_scaling_enabled(hdd_ipa)))
		return 0;

	memset(&profile, 0, sizeof(profile));

	if (tx_packets > (hdd_ctx->cfg_ini->busBandwidthHighThreshold / 2))
		next_cons_bw = hdd_ctx->cfg_ini->IpaHighBandwidthMbps;
	else if (tx_packets >
			(hdd_ctx->cfg_ini->busBandwidthMediumThreshold / 2))
		next_cons_bw = hdd_ctx->cfg_ini->IpaMediumBandwidthMbps;
	else
		next_cons_bw = hdd_ctx->cfg_ini->IpaLowBandwidthMbps;

	if (rx_packets > (hdd_ctx->cfg_ini->busBandwidthHighThreshold / 2))
		next_prod_bw = hdd_ctx->cfg_ini->IpaHighBandwidthMbps;
	else if (rx_packets >
			(hdd_ctx->cfg_ini->busBandwidthMediumThreshold / 2))
		next_prod_bw = hdd_ctx->cfg_ini->IpaMediumBandwidthMbps;
	else
		next_prod_bw = hdd_ctx->cfg_ini->IpaLowBandwidthMbps;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"CONS perf curr: %d, next: %d",
		hdd_ipa->curr_cons_bw, next_cons_bw);
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"PROD perf curr: %d, next: %d",
		hdd_ipa->curr_prod_bw, next_prod_bw);

	if (hdd_ipa->curr_cons_bw != next_cons_bw) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"Requesting CONS perf curr: %d, next: %d",
				hdd_ipa->curr_cons_bw, next_cons_bw);
		profile.max_supported_bandwidth_mbps = next_cons_bw;
		ret = ipa_rm_set_perf_profile(IPA_RM_RESOURCE_WLAN_CONS,
				&profile);
		if (ret) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"RM CONS set perf profile failed: %d",
					ret);

			return ret;
		}
		hdd_ipa->curr_cons_bw = next_cons_bw;
		hdd_ipa->stats.num_cons_perf_req++;
	}

	if (hdd_ipa->curr_prod_bw != next_prod_bw) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"Requesting PROD perf curr: %d, next: %d",
				hdd_ipa->curr_prod_bw, next_prod_bw);
		profile.max_supported_bandwidth_mbps = next_prod_bw;
		ret = ipa_rm_set_perf_profile(IPA_RM_RESOURCE_WLAN_PROD,
				&profile);
		if (ret) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"RM PROD set perf profile failed: %d",
					ret);
			return ret;
		}
		hdd_ipa->curr_prod_bw = next_prod_bw;
		hdd_ipa->stats.num_prod_perf_req++;
	}

	return 0;
}

static int hdd_ipa_setup_rm(struct hdd_ipa_priv *hdd_ipa)
{
	struct ipa_rm_create_params create_params = {0};
	int ret;

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		return 0;

#ifdef CONFIG_CNSS
	cnss_init_work(&hdd_ipa->rm_work, hdd_ipa_rm_send_pkt_to_ipa);
#else
	INIT_WORK(&hdd_ipa->rm_work, hdd_ipa_rm_send_pkt_to_ipa);
#endif
#ifdef IPA_UC_OFFLOAD
	cnss_init_work(&hdd_ipa->uc_rm_work.work, hdd_ipa_uc_rm_notify_defer);
#endif
	memset(&create_params, 0, sizeof(create_params));
	create_params.name = IPA_RM_RESOURCE_WLAN_PROD;
	create_params.reg_params.user_data = hdd_ipa;
	create_params.reg_params.notify_cb = hdd_ipa_rm_notify;
	create_params.floor_voltage = IPA_VOLTAGE_SVS;

	ret = ipa_rm_create_resource(&create_params);
	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"Create RM resource failed: %d",
			ret);
		goto setup_rm_fail;
	}

	memset(&create_params, 0, sizeof(create_params));
	create_params.name = IPA_RM_RESOURCE_WLAN_CONS;
	create_params.request_resource= hdd_ipa_rm_cons_request;
	create_params.release_resource= hdd_ipa_rm_cons_release;
	create_params.floor_voltage = IPA_VOLTAGE_SVS;

	ret = ipa_rm_create_resource(&create_params);
	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"Create RM CONS resource failed: %d", ret);
		goto delete_prod;
	}

	ipa_rm_add_dependency(IPA_RM_RESOURCE_WLAN_PROD,
			IPA_RM_RESOURCE_APPS_CONS);

	ret = ipa_rm_inactivity_timer_init(IPA_RM_RESOURCE_WLAN_PROD,
			HDD_IPA_RX_INACTIVITY_MSEC_DELAY);
	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Timer init failed: %d",
				ret);
		goto timer_init_failed;
	}

	/* Set the lowest bandwidth to start with */
	ret = hdd_ipa_set_perf_level(hdd_ipa->hdd_ctx, 0, 0);

	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"Set perf level failed: %d", ret);
		goto set_perf_failed;
	}

	vos_wake_lock_init(&hdd_ipa->wake_lock, "wlan_ipa");
#ifdef CONFIG_CNSS
	cnss_init_delayed_work(&hdd_ipa->wake_lock_work,
			hdd_ipa_wake_lock_timer_func);
#else
	INIT_DELAYED_WORK(&hdd_ipa->wake_lock_work,
			hdd_ipa_wake_lock_timer_func);
#endif
	adf_os_spinlock_init(&hdd_ipa->rm_lock);
	hdd_ipa->rm_state = HDD_IPA_RM_RELEASED;
	hdd_ipa->wake_lock_released = true;
	atomic_set(&hdd_ipa->tx_ref_cnt, 0);

	return ret;

set_perf_failed:
	ipa_rm_inactivity_timer_destroy(IPA_RM_RESOURCE_WLAN_PROD);

timer_init_failed:
	ipa_rm_delete_resource(IPA_RM_RESOURCE_WLAN_CONS);

delete_prod:
	ipa_rm_delete_resource(IPA_RM_RESOURCE_WLAN_PROD);

setup_rm_fail:
	return ret;
}

static void hdd_ipa_destory_rm_resource(struct hdd_ipa_priv *hdd_ipa)
{
	int ret;

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		return;

	cancel_delayed_work_sync(&hdd_ipa->wake_lock_work);
	vos_wake_lock_destroy(&hdd_ipa->wake_lock);

#ifdef WLAN_OPEN_SOURCE
	cancel_work_sync(&hdd_ipa->rm_work);
#ifdef IPA_UC_OFFLOAD
	cancel_work_sync(&hdd_ipa->uc_rm_work.work);
#endif
#endif
	adf_os_spinlock_destroy(&hdd_ipa->rm_lock);

	ipa_rm_inactivity_timer_destroy(IPA_RM_RESOURCE_WLAN_PROD);

	ret = ipa_rm_delete_resource(IPA_RM_RESOURCE_WLAN_PROD);
	if (ret)
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"RM PROD resource delete failed %d", ret);

	ret = ipa_rm_delete_resource(IPA_RM_RESOURCE_WLAN_CONS);
	if (ret)
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"RM CONS resource delete failed %d", ret);
}

#define IPA_WLAN_RX_SOFTIRQ_THRESH 16

static void hdd_ipa_send_skb_to_network(adf_nbuf_t skb, hdd_adapter_t *adapter)
{
	int result;

	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	unsigned int cpu_index;

	if (!adapter || adapter->magic != WLAN_HDD_ADAPTER_MAGIC) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_LOW, "Invalid adapter: 0x%p",
				adapter);
		HDD_IPA_INCREASE_INTERNAL_DROP_COUNT(hdd_ipa);
		adf_nbuf_free(skb);
		return;
	}

	if (hdd_ipa->hdd_ctx->isUnloadInProgress) {
		HDD_IPA_INCREASE_INTERNAL_DROP_COUNT(hdd_ipa);
		adf_nbuf_free(skb);
		return;
	}

	skb->destructor = hdd_ipa_uc_rt_debug_destructor;
	skb->dev = adapter->dev;
	skb->protocol = eth_type_trans(skb, skb->dev);
	skb->ip_summed = CHECKSUM_NONE;

	cpu_index = wlan_hdd_get_cpu();

	++adapter->hdd_stats.hddTxRxStats.rxPackets[cpu_index];
#ifdef QCA_CONFIG_SMP
	result = netif_rx_ni(skb);
#else
	if (hdd_ipa->stats.num_rx_excep % IPA_WLAN_RX_SOFTIRQ_THRESH == 0)
		result = netif_rx_ni(skb);
	else
		result = netif_rx(skb);
#endif
	if (result == NET_RX_SUCCESS)
		++adapter->hdd_stats.hddTxRxStats.rxDelivered[cpu_index];
	else
		++adapter->hdd_stats.hddTxRxStats.rxRefused[cpu_index];

	HDD_IPA_INCREASE_NET_SEND_COUNT(hdd_ipa);
	adapter->dev->last_rx = jiffies;
}

VOS_STATUS hdd_ipa_process_rxt(v_VOID_t *vosContext, adf_nbuf_t rx_buf_list,
		v_U8_t sta_id)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	hdd_adapter_t *adapter = NULL;
	struct hdd_ipa_iface_context *iface_context = NULL;
	adf_nbuf_t buf, next_buf;
	uint8_t cur_cnt = 0;
	struct hdd_ipa_cld_hdr *cld_hdr;
	struct ipa_tx_data_desc *send_desc = NULL;

	if (!hdd_ipa_is_enabled(hdd_ipa->hdd_ctx))
		return VOS_STATUS_E_INVAL;

	adapter = hdd_ipa->hdd_ctx->sta_to_adapter[sta_id];
	if (!adapter || !adapter->ipa_context ||
			adapter->magic != WLAN_HDD_ADAPTER_MAGIC) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_LOW, "Invalid sta_id: %d",
				sta_id);
		hdd_ipa->stats.num_rx_drop++;
		if (adapter)
			adapter->stats.rx_dropped++;
		return VOS_STATUS_E_FAILURE;
	}

	iface_context = (struct hdd_ipa_iface_context *) adapter->ipa_context;

	buf = rx_buf_list;
	while (buf) {
		HDD_IPA_DBG_DUMP(VOS_TRACE_LEVEL_DEBUG, "RX data",
				buf->data, 24);

		next_buf = adf_nbuf_queue_next(buf);
		adf_nbuf_set_next(buf, NULL);

		adapter->stats.rx_packets++;
		adapter->stats.rx_bytes += buf->len;
		/*
		 * we want to send Rx packets to IPA only when it is
		 * IPV4 or IPV6 (if IPV6 is enabled). All other packets
		 * will be sent to network stack directly.
		 */
		if (!hdd_ipa_can_send_to_ipa(adapter, hdd_ipa, buf->data)) {
			iface_context->stats.num_rx_prefilter++;
			hdd_ipa_send_skb_to_network(buf, adapter);
			buf = next_buf;
			continue;
		}

		cld_hdr = (struct hdd_ipa_cld_hdr *) skb_push(buf,
				HDD_IPA_WLAN_CLD_HDR_LEN);
		cld_hdr->sta_id = sta_id;
		cld_hdr->iface_id = iface_context->iface_id;

		send_desc = hdd_ipa_alloc_data_desc(hdd_ipa, 0);
		if (!send_desc) {
			adf_nbuf_free(buf); /*No desc available; drop*/
			buf = next_buf;
			iface_context->stats.num_rx_send_desc_err++;
			continue;
		}

		send_desc->priv = buf;
		send_desc->pyld_buffer = buf->data;
		send_desc->pyld_len = buf->len;
		spin_lock_bh(&hdd_ipa->q_lock);
		list_add_tail(&send_desc->link, &hdd_ipa->pend_desc_head);
		hdd_ipa->pend_q_cnt++;
		spin_unlock_bh(&hdd_ipa->q_lock);
		cur_cnt++;
		buf = next_buf;
	}

	iface_context->stats.num_rx_recv += cur_cnt;
	if (cur_cnt > 1)
		iface_context->stats.num_rx_recv_mul++;

	if (cur_cnt > iface_context->stats.max_rx_mul)
		iface_context->stats.max_rx_mul = cur_cnt;

	if (hdd_ipa->pend_q_cnt > hdd_ipa->stats.max_pend_q_cnt)
		hdd_ipa->stats.max_pend_q_cnt = hdd_ipa->pend_q_cnt;

	if (cur_cnt && hdd_ipa_rm_request(hdd_ipa) == 0) {
		hdd_ipa_send_pkt_to_ipa(hdd_ipa);
	}

	return VOS_STATUS_SUCCESS;
}

static void hdd_ipa_set_adapter_ip_filter(hdd_adapter_t *adapter)
{
	struct in_ifaddr **ifap = NULL;
	struct in_ifaddr *ifa = NULL;
	struct in_device *in_dev;
	struct net_device *dev;
	struct hdd_ipa_iface_context *iface_context;

	iface_context = (struct hdd_ipa_iface_context *)adapter->ipa_context;
	dev = adapter->dev;
	if (!dev || !iface_context) {
		return;
	}
	/* This optimization not needed for Station mode one of
	 * the reason being sta-usb tethered mode
	 */
	if (adapter->device_mode == WLAN_HDD_INFRA_STATION) {
		iface_context->ifa_address = 0;
		return;
	}


	/* Get IP address */
	if (dev->priv_flags & IFF_BRIDGE_PORT) {
#ifdef WLAN_OPEN_SOURCE
		rcu_read_lock();
#endif
		dev = netdev_master_upper_dev_get_rcu(adapter->dev);
#ifdef WLAN_OPEN_SOURCE
		rcu_read_unlock();
#endif
		if (!dev)
			return;
	}
	if ((in_dev = __in_dev_get_rtnl(dev)) != NULL) {
	   for (ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL;
		   ifap = &ifa->ifa_next) {
			if (dev->name && !strcmp(dev->name, ifa->ifa_label))
				break; /* found */
		}
	}
	if(ifa && ifa->ifa_address) {
		iface_context->ifa_address = ifa->ifa_address;

		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,"%s: %d.%d.%d.%d", dev->name,
		iface_context->ifa_address & 0x000000ff,
		iface_context->ifa_address >> 8 & 0x000000ff,
		iface_context->ifa_address >> 16 & 0x000000ff,
		iface_context->ifa_address >> 24 & 0x000000ff);
	}
}

static int hdd_ipa_ipv4_changed(struct notifier_block *nb,
		unsigned long data, void *arg)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	hdd_adapter_list_node_t *padapter_node = NULL, *pnext = NULL;
	hdd_adapter_t *padapter;
	VOS_STATUS status;
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"IPv4 Change detected. Updating wlan IPv4 local filters");

	status = hdd_get_front_adapter(hdd_ipa->hdd_ctx, &padapter_node);
	while (padapter_node && VOS_STATUS_SUCCESS == status) {
		padapter = padapter_node->pAdapter;
		if (padapter)
			hdd_ipa_set_adapter_ip_filter(padapter);

		status = hdd_get_next_adapter(hdd_ipa->hdd_ctx, padapter_node, &pnext);
		padapter_node = pnext;
	}
	return 0;
}


#define FW_RX_DESC_DISCARD_M 0x1
#define FW_RX_DESC_FORWARD_M 0x2

static void hdd_ipa_w2i_cb(void *priv, enum ipa_dp_evt_type evt,
		unsigned long data)
{
	struct hdd_ipa_priv *hdd_ipa = NULL;
	hdd_adapter_t *adapter = NULL;
	struct ipa_tx_data_desc *done_desc_head, *done_desc, *tmp;
	adf_nbuf_t skb;
	uint8_t iface_id;
	struct hdd_ipa_iface_context *iface_context;
#ifdef IPA_UC_OFFLOAD
	uint8_t session_id;
#ifdef INTRA_BSS_FWD_OFFLOAD
	adf_nbuf_t copy;
	uint8_t fw_desc;
	int ret;
#endif
#endif
	adf_nbuf_t buf;

	hdd_ipa = (struct hdd_ipa_priv *)priv;

	switch (evt) {
	case IPA_RECEIVE:
		skb = (adf_nbuf_t) data;

#ifdef IPA_UC_OFFLOAD
		if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
			session_id = (uint8_t)skb->cb[0];
			iface_id = vdev_to_iface[session_id];
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_HIGH,
				"IPA_RECEIVE: session_id=%u, iface_id=%u",
				session_id, iface_id);
		} else
#endif /* IPA_UC_OFFLOAD */
		{
			iface_id = HDD_IPA_GET_IFACE_ID(skb->data);
		}

		if (iface_id >= HDD_IPA_MAX_IFACE) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"IPA_RECEIVE: Invalid iface_id: %u",
					iface_id);
			HDD_IPA_DBG_DUMP(VOS_TRACE_LEVEL_INFO_HIGH,
				"w2i -- skb", skb->data, 8);
			HDD_IPA_INCREASE_INTERNAL_DROP_COUNT(hdd_ipa);
			adf_nbuf_free(skb);
			return;
		}

		iface_context = &hdd_ipa->iface_context[iface_id];
		adapter = iface_context->adapter;

		HDD_IPA_DBG_DUMP(VOS_TRACE_LEVEL_DEBUG,
			"w2i -- skb", skb->data, 16);
#ifdef IPA_UC_OFFLOAD
		if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
			hdd_ipa->stats.num_rx_excep++;
			skb_pull(skb, HDD_IPA_UC_WLAN_CLD_HDR_LEN);
		} else
#endif /* IPA_UC_OFFLOAD */
		{
			skb_pull(skb, HDD_IPA_WLAN_CLD_HDR_LEN);
		}

		iface_context->stats.num_rx_ipa_excep++;

#if defined(IPA_UC_OFFLOAD) && defined(INTRA_BSS_FWD_OFFLOAD)
		/* Disable to forward Intra-BSS Rx packets when
		 * ap_isolate=1 in hostapd.conf
		 */
		if ((NULL != iface_context->tl_context) &&
			!WLANTL_disable_intrabss_fwd(iface_context->tl_context))
		{
			/*
			 * When INTRA_BSS_FWD_OFFLOAD is enabled, FW will send
			 * all Rx packets to IPA uC, which need to be forwarded
			 * to other interface.
			 * And, IPA driver will send back to WLAN host driver
			 * through exception pipe with fw_desc field set by FW.
			 * Here we are checking fw_desc field for FORWARD bit
			 * set, and forward to Tx. Then copy to kernel stack
			 * only when DISCARD bit is not set.
			 */
			fw_desc = (uint8_t)skb->cb[1];

			if (fw_desc & FW_RX_DESC_FORWARD_M) {
				HDD_IPA_LOG(
					VOS_TRACE_LEVEL_INFO,
					"Forward packet to Tx (fw_desc=%d)",
					fw_desc);
				copy = adf_nbuf_copy(skb);
				if (copy) {
					hdd_ipa->ipa_tx_forward++;
					ret = hdd_softap_hard_start_xmit(
						(struct sk_buff *)copy,
						adapter->dev);
					if (ret) {
						HDD_IPA_LOG(
							VOS_TRACE_LEVEL_ERROR,
							"Forward packet Tx fail"
							);
						hdd_ipa->stats.
							num_tx_bcmc_err++;
					} else {
						hdd_ipa->stats.num_tx_bcmc++;
					}
				}
			}

			if (fw_desc & FW_RX_DESC_DISCARD_M) {
				hdd_ipa->ipa_rx_internel_drop_count++;
				hdd_ipa->ipa_rx_discard++;
				adf_nbuf_free(skb);
				break;
			}

		}
		else
		{
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO_HIGH,
				"Intra-BSS FWD is disabled-skip forward to Tx");
		}
#endif

		hdd_ipa_send_skb_to_network(skb, adapter);
		break;
	case IPA_WRITE_DONE:
		done_desc_head = (struct ipa_tx_data_desc *)data;
		list_for_each_entry_safe(done_desc, tmp,
				&done_desc_head->link, link) {
			list_del(&done_desc->link);
			buf = done_desc->priv;
			adf_nbuf_free(buf);
			hdd_ipa_free_data_desc(hdd_ipa, done_desc);
			spin_lock_bh(&hdd_ipa->q_lock);
			hdd_ipa->pending_hw_desc_cnt--;
			spin_unlock_bh(&hdd_ipa->q_lock);
			hdd_ipa->stats.num_rx_ipa_write_done++;
		}
		/* add anchor node also back to free list */
		hdd_ipa_free_data_desc(hdd_ipa, done_desc_head);

		hdd_ipa_send_pkt_to_ipa(hdd_ipa);

		hdd_ipa_rm_try_release(hdd_ipa);
		break;
	default:
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"w2i cb wrong event: 0x%x", evt);
		return;
	}
}

#ifdef QCA_MDM_DEVICE
static void hdd_ipa_nbuf_cb(adf_nbuf_t skb)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_DEBUG, "%lx", NBUF_OWNER_PRIV_DATA(skb));
	ipa_free_skb((struct ipa_rx_data *) NBUF_OWNER_PRIV_DATA(skb));

	hdd_ipa->stats.num_tx_comp_cnt++;

	atomic_dec(&hdd_ipa->tx_ref_cnt);

	hdd_ipa_rm_try_release(hdd_ipa);
}
#endif /* QCA_MDM_DEVICE */

static void hdd_ipa_send_pkt_to_tl(struct hdd_ipa_iface_context *iface_context,
		struct ipa_rx_data *ipa_tx_desc)
{
	struct hdd_ipa_priv *hdd_ipa = iface_context->hdd_ipa;
	v_U8_t interface_id;
	hdd_adapter_t  *adapter = NULL;
	adf_nbuf_t skb;

	adf_os_spin_lock_bh(&iface_context->interface_lock);
	adapter = iface_context->adapter;
	if (!adapter) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_WARN, "Interface Down");
		ipa_free_skb(ipa_tx_desc);
		iface_context->stats.num_tx_drop++;
		adf_os_spin_unlock_bh(&iface_context->interface_lock);
		hdd_ipa_rm_try_release(hdd_ipa);
		return;
	}

	/*
	 * During CAC period, data packets shouldn't be sent over the air so
	 * drop all the packets here
	 */
	if (WLAN_HDD_GET_AP_CTX_PTR(adapter)->dfs_cac_block_tx) {
		ipa_free_skb(ipa_tx_desc);
		adf_os_spin_unlock_bh(&iface_context->interface_lock);
		iface_context->stats.num_tx_cac_drop++;
		hdd_ipa_rm_try_release(hdd_ipa);
		return;
	}

	interface_id = adapter->sessionId;
	++adapter->stats.tx_packets;

	adf_os_spin_unlock_bh(&iface_context->interface_lock);

	skb = ipa_tx_desc->skb;

	adf_os_mem_set(skb->cb, 0, sizeof(skb->cb));
#ifdef QCA_MDM_DEVICE
	NBUF_OWNER_ID(skb) = IPA_NBUF_OWNER_ID;
	NBUF_CALLBACK_FN(skb) = hdd_ipa_nbuf_cb;
#ifdef IPA_UC_STA_OFFLOAD
	NBUF_MAPPED_PADDR_LO(skb) = ipa_tx_desc->dma_addr
		+ sizeof(struct frag_header) + sizeof(struct ipa_header);
	ipa_tx_desc->skb->len -=
		sizeof(struct frag_header) + sizeof(struct ipa_header);
#else
	NBUF_MAPPED_PADDR_LO(skb) = ipa_tx_desc->dma_addr;
#endif

	NBUF_OWNER_PRIV_DATA(skb) = (unsigned long)ipa_tx_desc;
#endif /* QCA_MDM_DEVICE */

	adapter->stats.tx_bytes += ipa_tx_desc->skb->len;

	skb = WLANTL_SendIPA_DataFrame(hdd_ipa->hdd_ctx->pvosContext,
			iface_context->tl_context, ipa_tx_desc->skb,
			interface_id);
	if (skb) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_DEBUG, "TLSHIM tx fail");
		ipa_free_skb(ipa_tx_desc);
		iface_context->stats.num_tx_err++;
		hdd_ipa_rm_try_release(hdd_ipa);
		return;
	}

	atomic_inc(&hdd_ipa->tx_ref_cnt);

	iface_context->stats.num_tx++;

}

static void hdd_ipa_pm_send_pkt_to_tl(struct work_struct *work)
{
	struct hdd_ipa_priv *hdd_ipa = container_of(work,
			struct hdd_ipa_priv, pm_work);
	struct hdd_ipa_pm_tx_cb *pm_tx_cb = NULL;
	adf_nbuf_t skb;
	uint32_t dequeued = 0;

	adf_os_spin_lock_bh(&hdd_ipa->pm_lock);

	while (((skb = adf_nbuf_queue_remove(&hdd_ipa->pm_queue_head)) !=
				NULL)) {
		adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);

		pm_tx_cb = (struct hdd_ipa_pm_tx_cb *)skb->cb;

		dequeued++;

		hdd_ipa_send_pkt_to_tl(pm_tx_cb->iface_context,
				pm_tx_cb->ipa_tx_desc);

		adf_os_spin_lock_bh(&hdd_ipa->pm_lock);
	}

	adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);

	hdd_ipa->stats.num_tx_dequeued += dequeued;
	if (dequeued > hdd_ipa->stats.num_max_pm_queue)
		hdd_ipa->stats.num_max_pm_queue = dequeued;
}

static void hdd_ipa_i2w_cb(void *priv, enum ipa_dp_evt_type evt,
		unsigned long data)
{
	struct hdd_ipa_priv *hdd_ipa = NULL;
	struct ipa_rx_data *ipa_tx_desc;
	struct hdd_ipa_iface_context *iface_context;
	adf_nbuf_t skb;
	struct hdd_ipa_pm_tx_cb *pm_tx_cb = NULL;
	VOS_STATUS status = VOS_STATUS_SUCCESS;

	iface_context = (struct hdd_ipa_iface_context *) priv;
	if (evt != IPA_RECEIVE) {
		skb = (adf_nbuf_t) data;
		dev_kfree_skb_any(skb);
		iface_context->stats.num_tx_drop++;
		return;
	}
	ipa_tx_desc = (struct ipa_rx_data *)data;
	hdd_ipa = iface_context->hdd_ipa;

	/*
	 * When SSR is going on or driver is unloading, just drop the packets.
	 * During SSR, there is no use in queueing the packets as STA has to
	 * connect back any way
	 */
	status = wlan_hdd_validate_context(hdd_ipa->hdd_ctx);
	if (0 != status) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "HDD context is not valid");
		ipa_free_skb(ipa_tx_desc);
		iface_context->stats.num_tx_drop++;
		return;
	}

	skb = ipa_tx_desc->skb;

	HDD_IPA_DBG_DUMP(VOS_TRACE_LEVEL_DEBUG, "i2w", skb->data, 8);

	/*
	 * If PROD resource is not requested here then there may be cases where
	 * IPA hardware may be clocked down because of not having proper
	 * dependency graph between WLAN CONS and modem PROD pipes. Adding the
	 * workaround to request PROD resource while data is going over CONS
	 * pipe to prevent the IPA hardware clockdown.
	 */
	hdd_ipa_rm_request(hdd_ipa);


	adf_os_spin_lock_bh(&hdd_ipa->pm_lock);
	/*
	 * If host is still suspended then queue the packets and these will be
	 * drained later when resume completes. When packet is arrived here and
	 * host is suspended, this means that there is already resume is in
	 * progress.
	 */
	if (hdd_ipa->suspended) {
		adf_os_mem_set(skb->cb, 0, sizeof(skb->cb));
		pm_tx_cb = (struct hdd_ipa_pm_tx_cb *)skb->cb;
		pm_tx_cb->iface_context = iface_context;
		pm_tx_cb->ipa_tx_desc = ipa_tx_desc;
		adf_nbuf_queue_add(&hdd_ipa->pm_queue_head, skb);
		hdd_ipa->stats.num_tx_queued++;

		adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);
		return;
	}

	adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);

	/*
	 * If we are here means, host is not suspended, wait for the work queue
	 * to finish.
	 */
#ifdef WLAN_OPEN_SOURCE
	flush_work(&hdd_ipa->pm_work);
#endif

	return hdd_ipa_send_pkt_to_tl(iface_context, ipa_tx_desc);
}

int hdd_ipa_suspend(hdd_context_t *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa = hdd_ctx->hdd_ipa;

	if (!hdd_ipa_is_enabled(hdd_ctx))
		return 0;

	/*
	 * Check if IPA is ready for suspend, If we are here means, there is
	 * high chance that suspend would go through but just to avoid any race
	 * condition after suspend started, these checks are conducted before
	 * allowing to suspend.
	 */
	if (atomic_read(&hdd_ipa->tx_ref_cnt))
		return -EAGAIN;

	adf_os_spin_lock_bh(&hdd_ipa->rm_lock);

	if (hdd_ipa->rm_state != HDD_IPA_RM_RELEASED) {
		adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);
		return -EAGAIN;
	}
	adf_os_spin_unlock_bh(&hdd_ipa->rm_lock);

	adf_os_spin_lock_bh(&hdd_ipa->pm_lock);
	hdd_ipa->suspended = true;
	adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);

	return 0;
}

int hdd_ipa_resume(hdd_context_t *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa = hdd_ctx->hdd_ipa;

	if (!hdd_ipa_is_enabled(hdd_ctx))
		return 0;

	schedule_work(&hdd_ipa->pm_work);

	adf_os_spin_lock_bh(&hdd_ipa->pm_lock);
	hdd_ipa->suspended = false;
	adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);

	return 0;
}

static int hdd_ipa_setup_sys_pipe(struct hdd_ipa_priv *hdd_ipa)
{
	int i, ret = 0;
	struct ipa_sys_connect_params *ipa;
	uint32_t desc_fifo_sz;

	/* The maximum number of descriptors that can be provided to a BAM at
	 * once is one less than the total number of descriptors that the buffer
	 * can contain.
	 * If max_num_of_descriptors = (BAM_PIPE_DESCRIPTOR_FIFO_SIZE / sizeof
	 * (SPS_DESCRIPTOR)), then (max_num_of_descriptors - 1) descriptors can
	 * be provided at once.
	 * Because of above requirement, one extra descriptor will be added to
	 * make sure hardware always has one descriptor.
	 */
	desc_fifo_sz = hdd_ipa->hdd_ctx->cfg_ini->IpaDescSize
		+ sizeof(struct sps_iovec);

	/*setup TX pipes */
	for (i = 0; i < HDD_IPA_MAX_IFACE; i++) {
		ipa = &hdd_ipa->sys_pipe[i].ipa_sys_params;

		ipa->client = hdd_ipa_adapter_2_client[i].cons_client;
		ipa->desc_fifo_sz = desc_fifo_sz;
		ipa->priv = &hdd_ipa->iface_context[i];
		ipa->notify = hdd_ipa_i2w_cb;

#ifdef IPA_UC_STA_OFFLOAD
		ipa->ipa_ep_cfg.hdr.hdr_len = HDD_IPA_UC_WLAN_TX_HDR_LEN;
		ipa->ipa_ep_cfg.nat.nat_en = IPA_BYPASS_NAT;
		ipa->ipa_ep_cfg.hdr.hdr_ofst_pkt_size_valid = 1;
		ipa->ipa_ep_cfg.hdr.hdr_ofst_pkt_size = 0;
		ipa->ipa_ep_cfg.hdr.hdr_additional_const_len =
			HDD_IPA_UC_WLAN_8023_HDR_SIZE;
		ipa->ipa_ep_cfg.hdr_ext.hdr_little_endian = true;
#else
		ipa->ipa_ep_cfg.hdr.hdr_len = HDD_IPA_WLAN_TX_HDR_LEN;
#endif
		ipa->ipa_ep_cfg.mode.mode = IPA_BASIC;

		if (!hdd_ipa_is_rm_enabled(hdd_ipa))
			ipa->keep_ipa_awake = 1;

		ret = ipa_setup_sys_pipe(ipa, &(hdd_ipa->sys_pipe[i].conn_hdl));
		if (ret) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Failed for pipe %d"
					" ret: %d", i, ret);
			goto setup_sys_pipe_fail;
		}
		hdd_ipa->sys_pipe[i].conn_hdl_valid = 1;
	}

#ifndef IPA_UC_STA_OFFLOAD
	/*
	 * Hard code it here, this can be extended if in case PROD pipe is also
	 * per interface. Right now there is no advantage of doing this.
	 */
	hdd_ipa->prod_client = IPA_CLIENT_WLAN1_PROD;

	ipa = &hdd_ipa->sys_pipe[HDD_IPA_RX_PIPE].ipa_sys_params;

	ipa->client = hdd_ipa->prod_client;

	ipa->desc_fifo_sz = desc_fifo_sz;
	ipa->priv = hdd_ipa;
	ipa->notify = hdd_ipa_w2i_cb;

	ipa->ipa_ep_cfg.nat.nat_en = IPA_BYPASS_NAT;
	ipa->ipa_ep_cfg.hdr.hdr_len = HDD_IPA_WLAN_RX_HDR_LEN;
	ipa->ipa_ep_cfg.hdr.hdr_ofst_metadata_valid = 1;
	ipa->ipa_ep_cfg.mode.mode = IPA_BASIC;

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		ipa->keep_ipa_awake = 1;

	ret = ipa_setup_sys_pipe(ipa, &(hdd_ipa->sys_pipe[i].conn_hdl));
	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Failed for RX pipe: %d",
				ret);
		goto setup_sys_pipe_fail;
	}
	hdd_ipa->sys_pipe[HDD_IPA_RX_PIPE].conn_hdl_valid = 1;
#endif /* IPA_UC_STA_OFFLOAD */

	return ret;

setup_sys_pipe_fail:

	while (--i >= 0) {
		ipa_teardown_sys_pipe(hdd_ipa->sys_pipe[i].conn_hdl);
		adf_os_mem_zero(&hdd_ipa->sys_pipe[i],
				sizeof(struct hdd_ipa_sys_pipe ));
	}

	return ret;
}

/* Disconnect all the Sys pipes */
static void hdd_ipa_teardown_sys_pipe(struct hdd_ipa_priv *hdd_ipa)
{
	int ret = 0, i;
	for (i = 0; i < HDD_IPA_MAX_SYSBAM_PIPE; i++) {
		if (hdd_ipa->sys_pipe[i].conn_hdl_valid) {
			ret = ipa_teardown_sys_pipe(
						hdd_ipa->sys_pipe[i].conn_hdl);
			if (ret)
				HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Failed: %d",
						ret);

			hdd_ipa->sys_pipe[i].conn_hdl_valid = 0;
		}
	}
}

static int hdd_ipa_register_interface(struct hdd_ipa_priv *hdd_ipa,
		struct hdd_ipa_iface_context *iface_context)
{
	struct ipa_tx_intf tx_intf;
	struct ipa_rx_intf rx_intf;
	struct ipa_ioc_tx_intf_prop *tx_prop = NULL;
	struct ipa_ioc_rx_intf_prop *rx_prop = NULL;
	char *ifname = iface_context->adapter->dev->name;

	char ipv4_hdr_name[IPA_RESOURCE_NAME_MAX];
	char ipv6_hdr_name[IPA_RESOURCE_NAME_MAX];

	int num_prop = 1;
	int ret = 0;

	if (hdd_ipa_is_ipv6_enabled(hdd_ipa))
		num_prop++;

	/* Allocate TX properties for TOS categories, 1 each for IPv4 & IPv6 */
	tx_prop = adf_os_mem_alloc(NULL,
			sizeof(struct ipa_ioc_tx_intf_prop) * num_prop);
	if (!tx_prop) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "tx_prop allocation failed");
		goto register_interface_fail;
	}

	/* Allocate RX properties, 1 each for IPv4 & IPv6 */
	rx_prop = adf_os_mem_alloc(NULL,
			sizeof(struct ipa_ioc_rx_intf_prop) * num_prop);
	if (!rx_prop) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "rx_prop allocation failed");
		goto register_interface_fail;
	}

	adf_os_mem_zero(&tx_intf, sizeof(tx_intf));
	adf_os_mem_zero(&rx_intf, sizeof(rx_intf));

	snprintf(ipv4_hdr_name, IPA_RESOURCE_NAME_MAX, "%s%s",
		ifname, HDD_IPA_IPV4_NAME_EXT);
	snprintf(ipv6_hdr_name, IPA_RESOURCE_NAME_MAX, "%s%s",
		ifname, HDD_IPA_IPV6_NAME_EXT);

	rx_prop[IPA_IP_v4].ip = IPA_IP_v4;
	rx_prop[IPA_IP_v4].src_pipe = iface_context->prod_client;
#ifdef IPA_UC_OFFLOAD
	rx_prop[IPA_IP_v4].hdr_l2_type = IPA_HDR_L2_ETHERNET_II;
#endif

	rx_prop[IPA_IP_v4].attrib.attrib_mask = IPA_FLT_META_DATA;

	/*
	 * Interface ID is 3rd byte in the CLD header. Add the meta data and
	 * mask to identify the interface in IPA hardware
	 */
	rx_prop[IPA_IP_v4].attrib.meta_data =
		htonl(iface_context->adapter->sessionId<< 16);
	rx_prop[IPA_IP_v4].attrib.meta_data_mask = htonl(0x00FF0000);

	rx_intf.num_props++;
	if (hdd_ipa_is_ipv6_enabled(hdd_ipa)) {
		rx_prop[IPA_IP_v6].ip = IPA_IP_v6;
		rx_prop[IPA_IP_v6].src_pipe = iface_context->prod_client;
#ifdef IPA_UC_OFFLOAD
		rx_prop[IPA_IP_v6].hdr_l2_type = IPA_HDR_L2_ETHERNET_II;
#endif

		rx_prop[IPA_IP_v4].attrib.attrib_mask = IPA_FLT_META_DATA;
		rx_prop[IPA_IP_v4].attrib.meta_data =
			htonl(iface_context->adapter->sessionId<< 16);
		rx_prop[IPA_IP_v4].attrib.meta_data_mask = htonl(0x00FF0000);

		rx_intf.num_props++;
	}

	tx_prop[IPA_IP_v4].ip = IPA_IP_v4;
#ifdef IPA_UC_OFFLOAD
	tx_prop[IPA_IP_v4].hdr_l2_type = IPA_HDR_L2_ETHERNET_II;
	tx_prop[IPA_IP_v4].dst_pipe = IPA_CLIENT_WLAN1_CONS;
	tx_prop[IPA_IP_v4].alt_dst_pipe = iface_context->cons_client;
#else
	tx_prop[IPA_IP_v4].dst_pipe = iface_context->cons_client;
#endif
	strlcpy(tx_prop[IPA_IP_v4].hdr_name, ipv4_hdr_name,
			IPA_RESOURCE_NAME_MAX);
	tx_intf.num_props++;

	if (hdd_ipa_is_ipv6_enabled(hdd_ipa)) {
		tx_prop[IPA_IP_v6].ip = IPA_IP_v6;
#ifdef IPA_UC_OFFLOAD
		tx_prop[IPA_IP_v6].hdr_l2_type = IPA_HDR_L2_ETHERNET_II;
		tx_prop[IPA_IP_v6].dst_pipe = IPA_CLIENT_WLAN1_CONS;
		tx_prop[IPA_IP_v6].alt_dst_pipe = iface_context->cons_client;
#else
		tx_prop[IPA_IP_v6].dst_pipe = iface_context->cons_client;
#endif
		strlcpy(tx_prop[IPA_IP_v6].hdr_name, ipv6_hdr_name,
				IPA_RESOURCE_NAME_MAX);
		tx_intf.num_props++;
	}

	tx_intf.prop = tx_prop;
	rx_intf.prop = rx_prop;

	/* Call the ipa api to register interface */
	ret = ipa_register_intf(ifname, &tx_intf, &rx_intf);

register_interface_fail:
	adf_os_mem_free(tx_prop);
	adf_os_mem_free(rx_prop);
	return ret;
}

static void hdd_remove_ipa_header(char *name)
{
	struct ipa_ioc_get_hdr hdrlookup;
	int ret = 0, len;
	struct ipa_ioc_del_hdr *ipa_hdr;

	adf_os_mem_zero(&hdrlookup, sizeof(hdrlookup));
	strlcpy(hdrlookup.name, name, sizeof(hdrlookup.name));
	ret = ipa_get_hdr(&hdrlookup);
	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "Hdr deleted already %s, %d",
				name, ret);
		return;
	}


	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "hdl: 0x%x", hdrlookup.hdl);
	len = sizeof(struct ipa_ioc_del_hdr) + sizeof(struct ipa_hdr_del)*1;
	ipa_hdr = (struct ipa_ioc_del_hdr *) adf_os_mem_alloc(NULL, len);
	if (ipa_hdr == NULL) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "ipa_hdr allocation failed");
		return;
	}
	ipa_hdr->num_hdls = 1;
	ipa_hdr->commit = 0;
	ipa_hdr->hdl[0].hdl = hdrlookup.hdl;
	ipa_hdr->hdl[0].status = -1;
	ret = ipa_del_hdr(ipa_hdr);
	if (ret != 0)
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "Delete header failed: %d",
				ret);

	adf_os_mem_free(ipa_hdr);
}


static int hdd_ipa_add_header_info(struct hdd_ipa_priv *hdd_ipa,
		struct hdd_ipa_iface_context *iface_context, uint8_t *mac_addr)
{
	hdd_adapter_t *adapter = iface_context->adapter;
	char *ifname;
	struct ipa_ioc_add_hdr *ipa_hdr = NULL;
	int ret = -EINVAL;
	struct hdd_ipa_tx_hdr *tx_hdr = NULL;
#ifdef IPA_UC_OFFLOAD
	struct hdd_ipa_uc_tx_hdr *uc_tx_hdr = NULL;
#endif /* IPA_UC_OFFLOAD */

	ifname = adapter->dev->name;


	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "Add Partial hdr: %s, %pM",
			ifname, mac_addr);

	/* dynamically allocate the memory to add the hdrs */
	ipa_hdr = adf_os_mem_alloc(NULL, sizeof(struct ipa_ioc_add_hdr)
			+ sizeof(struct ipa_hdr_add));
	if (!ipa_hdr) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"%s: ipa_hdr allocation failed", ifname);
		ret = -ENOMEM;
		goto end;
	}

	ipa_hdr->commit = 0;
	ipa_hdr->num_hdrs = 1;

	if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
#ifdef IPA_UC_OFFLOAD
		uc_tx_hdr = (struct hdd_ipa_uc_tx_hdr *)ipa_hdr->hdr[0].hdr;
		memcpy(uc_tx_hdr, &ipa_uc_tx_hdr, HDD_IPA_UC_WLAN_TX_HDR_LEN);
		memcpy(uc_tx_hdr->eth.h_source, mac_addr, ETH_ALEN);
		uc_tx_hdr->ipa_hd.vdev_id = iface_context->adapter->sessionId;
		HDD_IPA_LOG(VOS_TRACE_LEVEL_DEBUG,
			"ifname=%s, vdev_id=%d",
			ifname, uc_tx_hdr->ipa_hd.vdev_id);
		snprintf(ipa_hdr->hdr[0].name, IPA_RESOURCE_NAME_MAX, "%s%s",
				ifname, HDD_IPA_IPV4_NAME_EXT);
		ipa_hdr->hdr[0].hdr_len = HDD_IPA_UC_WLAN_TX_HDR_LEN;
		ipa_hdr->hdr[0].type = IPA_HDR_L2_ETHERNET_II;
		ipa_hdr->hdr[0].is_partial = 1;
		ipa_hdr->hdr[0].hdr_hdl = 0;
		ipa_hdr->hdr[0].is_eth2_ofst_valid = 1;
		ipa_hdr->hdr[0].eth2_ofst = HDD_IPA_UC_WLAN_HDR_DES_MAC_OFFSET;

		ret = ipa_add_hdr(ipa_hdr);
#endif /* IPA_UC_OFFLOAD */
	} else {
		tx_hdr = (struct hdd_ipa_tx_hdr *)ipa_hdr->hdr[0].hdr;

		/* Set the Source MAC */
		memcpy(tx_hdr, &ipa_tx_hdr, HDD_IPA_WLAN_TX_HDR_LEN);
		memcpy(tx_hdr->eth.h_source, mac_addr, ETH_ALEN);

		snprintf(ipa_hdr->hdr[0].name, IPA_RESOURCE_NAME_MAX, "%s%s",
			ifname, HDD_IPA_IPV4_NAME_EXT);
		ipa_hdr->hdr[0].hdr_len = HDD_IPA_WLAN_TX_HDR_LEN;
		ipa_hdr->hdr[0].is_partial = 1;
		ipa_hdr->hdr[0].hdr_hdl = 0;
		ipa_hdr->hdr[0].is_eth2_ofst_valid = 1;
		ipa_hdr->hdr[0].eth2_ofst = HDD_IPA_WLAN_HDR_DES_MAC_OFFSET;

		/* Set the type to IPV4 in the header*/
		tx_hdr->llc_snap.eth_type = cpu_to_be16(ETH_P_IP);

		ret = ipa_add_hdr(ipa_hdr);
	}
	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "%s IPv4 add hdr failed: %d",
				ifname, ret);
		goto end;
	}

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%s: IPv4 hdr_hdl: 0x%x",
			ipa_hdr->hdr[0].name, ipa_hdr->hdr[0].hdr_hdl);

	if (hdd_ipa_is_ipv6_enabled(hdd_ipa)) {
		snprintf(ipa_hdr->hdr[0].name, IPA_RESOURCE_NAME_MAX, "%s%s",
				ifname, HDD_IPA_IPV6_NAME_EXT);

#ifdef IPA_UC_OFFLOAD
		if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
			/* Set the type to IPV6 in the header*/
			uc_tx_hdr = (struct hdd_ipa_uc_tx_hdr *)ipa_hdr->hdr[0].hdr;
			uc_tx_hdr->eth.h_proto = cpu_to_be16(ETH_P_IPV6);
		} else
#endif /* IPA_UC_OFFLOAD */
		{
			/* Set the type to IPV6 in the header*/
			tx_hdr = (struct hdd_ipa_tx_hdr *)ipa_hdr->hdr[0].hdr;
			tx_hdr->llc_snap.eth_type = cpu_to_be16(ETH_P_IPV6);
		}

		ret = ipa_add_hdr(ipa_hdr);

		if (ret) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"%s: IPv6 add hdr failed: %d",
					ifname, ret);
			goto clean_ipv4_hdr;
		}

		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%s: IPv6 hdr_hdl: 0x%x",
				ipa_hdr->hdr[0].name, ipa_hdr->hdr[0].hdr_hdl);
	}

	adf_os_mem_free(ipa_hdr);

	return ret;

clean_ipv4_hdr:
	snprintf(ipa_hdr->hdr[0].name, IPA_RESOURCE_NAME_MAX, "%s%s",
			ifname, HDD_IPA_IPV4_NAME_EXT);
	hdd_remove_ipa_header(ipa_hdr->hdr[0].name);
end:
	if(ipa_hdr)
		adf_os_mem_free(ipa_hdr);

	return ret;
}

static void hdd_ipa_clean_hdr(hdd_adapter_t *adapter)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	int ret;
	char name_ipa[IPA_RESOURCE_NAME_MAX];

	/* Remove the headers */
	snprintf(name_ipa, IPA_RESOURCE_NAME_MAX, "%s%s",
		adapter->dev->name, HDD_IPA_IPV4_NAME_EXT);
	hdd_remove_ipa_header(name_ipa);

	if (hdd_ipa_is_ipv6_enabled(hdd_ipa)) {
		snprintf(name_ipa, IPA_RESOURCE_NAME_MAX, "%s%s",
			adapter->dev->name, HDD_IPA_IPV6_NAME_EXT);
		hdd_remove_ipa_header(name_ipa);
	}
	/* unregister the interface with IPA */
	ret = ipa_deregister_intf(adapter->dev->name);
	if (ret)
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: ipa_deregister_intf fail: %d",
				adapter->dev->name, ret);
}

static void hdd_ipa_cleanup_iface(struct hdd_ipa_iface_context *iface_context)
{
	if (iface_context == NULL)
		return;

	hdd_ipa_clean_hdr(iface_context->adapter);

	adf_os_spin_lock_bh(&iface_context->interface_lock);
	iface_context->adapter->ipa_context = NULL;
	iface_context->adapter = NULL;
	iface_context->tl_context = NULL;
	adf_os_spin_unlock_bh(&iface_context->interface_lock);
	iface_context->ifa_address = 0;
	if (!iface_context->hdd_ipa->num_iface) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"NUM INTF 0, Invalid");
		VOS_ASSERT(0);
	}
	iface_context->hdd_ipa->num_iface--;
}


static int hdd_ipa_setup_iface(struct hdd_ipa_priv *hdd_ipa,
		hdd_adapter_t *adapter, uint8_t sta_id)
{
	struct hdd_ipa_iface_context *iface_context = NULL;
	void *tl_context = NULL;
	int i, ret = 0;

	/* Lower layer may send multiple START_BSS_EVENT in DFS mode or during
	 * channel change indication. Since these indications are sent by lower
	 * layer as SAP updates and IPA doesn't have to do anything for these
	 * updates so ignoring!
	 */
	if (WLAN_HDD_SOFTAP == adapter->device_mode && adapter->ipa_context)
		return 0;

	for (i = 0; i < HDD_IPA_MAX_IFACE; i++) {
		if (hdd_ipa->iface_context[i].adapter == NULL) {
			iface_context = &(hdd_ipa->iface_context[i]);
			break;
		}
	}

	if (iface_context == NULL) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"All the IPA interfaces are in use");
		ret = -ENOMEM;
		goto end;
	}


	adapter->ipa_context = iface_context;
	iface_context->adapter = adapter;
	iface_context->sta_id = sta_id;
	tl_context = tl_shim_get_vdev_by_sta_id(hdd_ipa->hdd_ctx->pvosContext,
			sta_id);

	if (tl_context == NULL) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"Not able to get TL context sta_id: %d",
				sta_id);
		ret = -EINVAL;
		goto end;
	}

	iface_context->tl_context = tl_context;

	ret = hdd_ipa_add_header_info(hdd_ipa, iface_context,
			adapter->dev->dev_addr);

	if (ret)
		goto end;

	/* Configure the TX and RX pipes filter rules */
	ret = hdd_ipa_register_interface(hdd_ipa, iface_context);
	if (ret)
		goto cleanup_header;

	hdd_ipa->num_iface++;
	return ret;

cleanup_header:

	hdd_ipa_clean_hdr(adapter);
end:
	if (iface_context)
		hdd_ipa_cleanup_iface(iface_context);
	return ret;
}


static void hdd_ipa_msg_free_fn(void *buff, uint32_t len, uint32_t type)
{
	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "msg type:%d, len:%d", type, len);
	ghdd_ipa->stats.num_free_msg++;
	adf_os_mem_free(buff);
}

#ifdef IPA_UC_STA_OFFLOAD
int hdd_ipa_send_mcc_scc_msg(hdd_context_t *hdd_ctx, bool mcc_mode)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	hdd_adapter_list_node_t *adapter_node = NULL, *next = NULL;
	VOS_STATUS status;
	hdd_adapter_t *pAdapter;
	struct ipa_msg_meta meta;
	struct ipa_wlan_msg *msg;
	int ret;

	if (!hdd_ipa_uc_sta_is_enabled(hdd_ipa))
		return -EINVAL;

	if (!mcc_mode) {
		/* Flush TxRx queue for each adapter before switch to SCC */
		status =  hdd_get_front_adapter (hdd_ctx, &adapter_node);
		while (NULL != adapter_node && VOS_STATUS_SUCCESS == status) {
			pAdapter = adapter_node->pAdapter;
			if (pAdapter->device_mode == WLAN_HDD_INFRA_STATION ||
				pAdapter->device_mode == WLAN_HDD_SOFTAP) {
				hddLog(LOG1,
					"MCC->SCC: Flush TxRx queue(d_mode %s(%d))",
					hdd_device_mode_to_string(
						pAdapter->device_mode),
                                        pAdapter->device_mode);
				hdd_deinit_tx_rx(pAdapter);
			}
			status = hdd_get_next_adapter(
					hdd_ctx, adapter_node, &next);
			adapter_node = next;
		}
	}

	/* Send SCC/MCC Switching event to IPA */
	meta.msg_len = sizeof(*msg);
	msg = adf_os_mem_alloc(NULL, meta.msg_len);
	if (msg == NULL) {
		hddLog(VOS_TRACE_LEVEL_ERROR, "msg allocation failed");
		return -ENOMEM;
	}

	meta.msg_type = mcc_mode ? WLAN_SWITCH_TO_MCC : WLAN_SWITCH_TO_SCC;
	hddLog(VOS_TRACE_LEVEL_INFO, "ipa_send_msg(Evt:%d)", meta.msg_type);

	ret = ipa_send_msg(&meta, msg, hdd_ipa_msg_free_fn);

	if (ret) {
		hddLog(VOS_TRACE_LEVEL_ERROR, "ipa_send_msg(Evt:%d) - fail=%d",
			meta.msg_type,  ret);
		adf_os_mem_free(msg);
	}

	return ret;
}
#endif

static inline char *hdd_ipa_wlan_event_to_str(enum ipa_wlan_event event)
{
	switch(event) {
	case WLAN_CLIENT_CONNECT:  return "WLAN_CLIENT_CONNECT";
	case WLAN_CLIENT_DISCONNECT: return "WLAN_CLIENT_DISCONNECT";
	case WLAN_CLIENT_POWER_SAVE_MODE: return "WLAN_CLIENT_POWER_SAVE_MODE";
	case WLAN_CLIENT_NORMAL_MODE: return "WLAN_CLIENT_NORMAL_MODE";
	case SW_ROUTING_ENABLE: return "SW_ROUTING_ENABLE";
	case SW_ROUTING_DISABLE: return "SW_ROUTING_DISABLE";
	case WLAN_AP_CONNECT: return "WLAN_AP_CONNECT";
	case WLAN_AP_DISCONNECT: return "WLAN_AP_DISCONNECT";
	case WLAN_STA_CONNECT: return "WLAN_STA_CONNECT";
	case WLAN_STA_DISCONNECT: return "WLAN_STA_DISCONNECT";
	case WLAN_CLIENT_CONNECT_EX: return "WLAN_CLIENT_CONNECT_EX";

	case IPA_WLAN_EVENT_MAX:
	default:
	return "UNKNOWN";
	}
}

#ifdef IPA_UC_OFFLOAD
static void hdd_ipa_uc_offload_enable_disable(hdd_adapter_t *adapter,
			tANI_U32 offload_type, tANI_U32 enable)
{
	struct sir_ipa_offload_enable_disable ipa_offload_enable_disable;

	/* Lower layer may send multiple START_BSS_EVENT in DFS mode or during
	 * channel change indication. Since these indications are sent by lower
	 * layer as SAP updates and IPA doesn't have to do anything for these
	 * updates so ignoring!
	*/
	if (adapter->ipa_context)
		return;

	vos_mem_zero(&ipa_offload_enable_disable,
		sizeof(ipa_offload_enable_disable));
	ipa_offload_enable_disable.offload_type = offload_type;
	ipa_offload_enable_disable.vdev_id = adapter->sessionId;
	ipa_offload_enable_disable.enable = enable;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"%s: offload_type=%d, vdev_id=%d, enable=%d", __func__,
		ipa_offload_enable_disable.offload_type,
		ipa_offload_enable_disable.vdev_id,
		ipa_offload_enable_disable.enable);

	if (eHAL_STATUS_SUCCESS !=
		sme_ipa_offload_enable_disable(WLAN_HDD_GET_HAL_CTX(adapter),
			adapter->sessionId, &ipa_offload_enable_disable)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
			"%s: Failure to enable IPA offload \
			(offload_type=%d, vdev_id=%d, enable=%d)", __func__,
			ipa_offload_enable_disable.offload_type,
			ipa_offload_enable_disable.vdev_id,
			ipa_offload_enable_disable.enable);
	}
}
#endif

int hdd_ipa_wlan_evt(hdd_adapter_t *adapter, uint8_t sta_id,
			enum ipa_wlan_event type, uint8_t *mac_addr)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	struct ipa_msg_meta meta;
	struct ipa_wlan_msg *msg;
	struct ipa_wlan_msg_ex *msg_ex = NULL;
	int ret;

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%s: %s evt, MAC: %pM sta_id: %d",
			adapter->dev->name, hdd_ipa_wlan_event_to_str(type),
			mac_addr,
			sta_id);

	if (type >= IPA_WLAN_EVENT_MAX)
		return -EINVAL;

	if (!hdd_ipa || !hdd_ipa_is_enabled(hdd_ipa->hdd_ctx)) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "IPA OFFLOAD NOT ENABLED");
		return -EINVAL;
	}

#ifdef IPA_UC_OFFLOAD
	if (hdd_ipa_uc_is_enabled(hdd_ipa) &&
#ifdef IPA_UC_STA_OFFLOAD
		!hdd_ipa_uc_sta_is_enabled(hdd_ipa) &&
#endif
		(WLAN_HDD_SOFTAP != adapter->device_mode)) {
		return 0;
	}

	if (WARN_ON(is_zero_ether_addr(mac_addr)))
		return -EINVAL;

	/* During IPA UC resource loading/unloading
	 * new event issued.
	 * Store event seperatly and handle later */
	if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
		if (hdd_ipa->resource_loading) {
			v_SIZE_t pending_event_count;
			struct ipa_uc_pending_event *pending_event = NULL;

			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"%s: IPA resource load inprogress", __func__);

			vos_list_size(&hdd_ipa->pending_event,
					&pending_event_count);
			if (pending_event_count >= MAX_PENDING_EVENT_COUNT) {
				HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
					"%s: Reached max pending event count",
					__func__);
				vos_list_remove_front(&hdd_ipa->pending_event,
					(vos_list_node_t **)&pending_event);
			} else {
				pending_event = (struct ipa_uc_pending_event *)
						vos_mem_malloc(sizeof(
						struct ipa_uc_pending_event));
			}

			if (!pending_event) {
				HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"Pending event memory alloc fail");
				return -ENOMEM;
			}
			pending_event->adapter = adapter;
			pending_event->sta_id = sta_id;
			pending_event->type = type;
			vos_mem_copy(pending_event->mac_addr,
					mac_addr,
					VOS_MAC_ADDR_SIZE);
			vos_list_insert_back(&hdd_ipa->pending_event,
					&pending_event->node);
			return 0;
		} else if (hdd_ipa->resource_unloading) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"%s: IPA resource unload inprogress", __func__);
			return 0;
		}
	}
#endif /* IPA_UC_OFFLOAD */

	hdd_ipa->stats.event[type]++;

	switch (type) {
	case WLAN_STA_CONNECT:
#if defined(IPA_UC_OFFLOAD) && defined(IPA_UC_STA_OFFLOAD)
		/* STA alreadu connected and without disconnect, connect again
		 * This is Roaming scenario */
		if (hdd_ipa->sta_connected) {
			hdd_ipa_cleanup_iface(adapter->ipa_context);
		}

		if ((hdd_ipa_uc_sta_is_enabled(hdd_ipa)) &&
			(!hdd_ipa->sta_connected)) {
			hdd_ipa_uc_offload_enable_disable(adapter,
			SIR_STA_RX_DATA_OFFLOAD, 1);
		}

		vos_lock_acquire(&hdd_ipa->event_lock);
		if (!hdd_ipa_uc_is_enabled(hdd_ipa)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: Evt: %d, IPA UC OFFLOAD NOT ENABLED",
				msg_ex->name, meta.msg_type);
		} else if ((!hdd_ipa->sap_num_connected_sta) &&
				(!hdd_ipa->sta_connected)) {
			/* Enable IPA UC TX PIPE when STA connected */
			ret = hdd_ipa_uc_handle_first_con(hdd_ipa);
			if (ret) {
				vos_lock_release(&hdd_ipa->event_lock);
				HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"handle 1st con ret %d", ret);
				hdd_ipa_uc_offload_enable_disable(adapter,
					SIR_STA_RX_DATA_OFFLOAD, 0);
				goto end;
			}
		}
#endif
		ret = hdd_ipa_setup_iface(hdd_ipa, adapter, sta_id);
		if (ret) {
#ifdef IPA_UC_OFFLOAD
			vos_lock_release(&hdd_ipa->event_lock);
			hdd_ipa_uc_offload_enable_disable(adapter,
				SIR_STA_RX_DATA_OFFLOAD, 0);
#endif /* IPA_UC_OFFLOAD */
			goto end;
		}

#ifdef IPA_UC_OFFLOAD
		vdev_to_iface[adapter->sessionId] =
			((struct hdd_ipa_iface_context *)
				(adapter->ipa_context))->iface_id;
#ifdef IPA_UC_STA_OFFLOAD
		hdd_ipa->sta_connected = 1;
#endif
		vos_lock_release(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		break;

	case WLAN_AP_CONNECT:
		/* For DFS channel we get two start_bss event (before and after
		 * CAC). Also when ACS range includes both DFS and non DFS
		 * channels, we could possibly change channel many times due to
		 * RADAR detection and chosen channel may not be a DFS channels.
		 * So dont return error here. Just discard the event.
		 */
		if (adapter->ipa_context)
			return 0;

#ifdef IPA_UC_OFFLOAD
		if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
			hdd_ipa_uc_offload_enable_disable(adapter,
				SIR_AP_RX_DATA_OFFLOAD, 1);
		}
		vos_lock_acquire(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		ret = hdd_ipa_setup_iface(hdd_ipa, adapter, sta_id);
		if (ret) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: Evt: %d, Interface setup failed",
				msg_ex->name, meta.msg_type);
#ifdef IPA_UC_OFFLOAD
			vos_lock_release(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
			goto end;
		}

#ifdef IPA_UC_OFFLOAD
		vdev_to_iface[adapter->sessionId] =
			((struct hdd_ipa_iface_context *)
				(adapter->ipa_context))->iface_id;
		vos_lock_release(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		break;

	case WLAN_STA_DISCONNECT:
#ifdef IPA_UC_OFFLOAD
		vos_lock_acquire(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		hdd_ipa_cleanup_iface(adapter->ipa_context);

#if defined(IPA_UC_OFFLOAD) && defined(IPA_UC_STA_OFFLOAD)
		if (!hdd_ipa->sta_connected) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: Evt: %d, STA already disconnected",
				msg_ex->name, meta.msg_type);
			vos_lock_release(&hdd_ipa->event_lock);
			return -EINVAL;
		}
		hdd_ipa->sta_connected = 0;
		if (!hdd_ipa_uc_is_enabled(hdd_ipa)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: IPA UC OFFLOAD NOT ENABLED",
				msg_ex->name);
		} else {
			/* Disable IPA UC TX PIPE when STA disconnected */
			if ((!hdd_ipa->sap_num_connected_sta) ||
				((!hdd_ipa->num_iface) &&
					(HDD_IPA_UC_NUM_WDI_PIPE ==
					hdd_ipa->activated_fw_pipe))) {
				hdd_ipa_uc_handle_last_discon(hdd_ipa);
			}
		}

                if (hdd_ipa_uc_sta_is_enabled(hdd_ipa)) {
			hdd_ipa_uc_offload_enable_disable(adapter,
				SIR_STA_RX_DATA_OFFLOAD, 0);
			vdev_to_iface[adapter->sessionId] = HDD_IPA_MAX_IFACE;
		}
#endif
#ifdef IPA_UC_OFFLOAD
		vos_lock_release(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		break;

	case WLAN_AP_DISCONNECT:
		if (!adapter->ipa_context) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: Evt: %d, SAP already disconnected",
				msg_ex->name, meta.msg_type);
			return -EINVAL;
		}

#ifdef IPA_UC_OFFLOAD
		vos_lock_acquire(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		hdd_ipa_cleanup_iface(adapter->ipa_context);

#ifdef IPA_UC_OFFLOAD
		if ((!hdd_ipa->num_iface) &&
			(HDD_IPA_UC_NUM_WDI_PIPE == hdd_ipa->activated_fw_pipe)) {
			if (hdd_ipa->hdd_ctx->isUnloadInProgress) {
				/* We disable WDI pipes directly here since
				 * IPA_OPCODE_TX/RX_SUSPEND message will not be
				 * processed when unloding WLAN driver is in
				 * progress
				 */
				hdd_ipa_uc_disable_pipes(hdd_ipa);
			} else {
				HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"NO INTF left, "
					"but still pipe clean up");
				hdd_ipa_uc_handle_last_discon(hdd_ipa);
			}
		}

		if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
			hdd_ipa_uc_offload_enable_disable(adapter,
				SIR_AP_RX_DATA_OFFLOAD, 0);
			vdev_to_iface[adapter->sessionId] = HDD_IPA_MAX_IFACE;
		}
		vos_lock_release(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		break;

	case WLAN_CLIENT_CONNECT_EX:
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%d %d",
				adapter->dev->ifindex, sta_id);

#ifdef IPA_UC_OFFLOAD
		if (!hdd_ipa_uc_is_enabled(hdd_ipa)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: Evt: %d, IPA UC OFFLOAD NOT ENABLED",
				adapter->dev->name, meta.msg_type);
			return 0;
		}

		vos_lock_acquire(&hdd_ipa->event_lock);
		if (hdd_ipa_uc_find_add_assoc_sta(hdd_ipa,
				VOS_TRUE, sta_id)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"%s: STA ID %d found, not valid",
				adapter->dev->name, sta_id);
			vos_lock_release(&hdd_ipa->event_lock);
			return 0;
		}
		vos_lock_release(&hdd_ipa->event_lock);
#endif

		meta.msg_type = type;
		meta.msg_len = (sizeof(struct ipa_wlan_msg_ex) +
				sizeof(struct ipa_wlan_hdr_attrib_val));
		msg_ex = adf_os_mem_alloc (NULL, meta.msg_len);

		if (msg_ex == NULL) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"msg_ex allocation failed");
			return -ENOMEM;
		}
		strlcpy(msg_ex->name, adapter->dev->name,
				IPA_RESOURCE_NAME_MAX);
		msg_ex->num_of_attribs = 1;
		msg_ex->attribs[0].attrib_type = WLAN_HDR_ATTRIB_MAC_ADDR;
#ifdef IPA_UC_OFFLOAD
		if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
			msg_ex->attribs[0].offset =
				HDD_IPA_UC_WLAN_HDR_DES_MAC_OFFSET;
		} else
#endif /* IPA_UC_OFFLOAD */
		{
			msg_ex->attribs[0].offset =
				HDD_IPA_WLAN_HDR_DES_MAC_OFFSET;
		}
		memcpy(msg_ex->attribs[0].u.mac_addr, mac_addr,
				IPA_MAC_ADDR_SIZE);

		ret = ipa_send_msg(&meta, msg_ex, hdd_ipa_msg_free_fn);

		if (ret) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%s: Evt: %d : %d",
					msg_ex->name, meta.msg_type,  ret);
			adf_os_mem_free(msg_ex);
			return ret;
		}
		hdd_ipa->stats.num_send_msg++;
#ifdef IPA_UC_OFFLOAD
		vos_lock_acquire(&hdd_ipa->event_lock);
		/* Enable IPA UC Data PIPEs when first STA connected */
		if ((0 == hdd_ipa->sap_num_connected_sta)
#ifdef IPA_UC_STA_OFFLOAD
			&& (!hdd_ipa_uc_sta_is_enabled(hdd_ipa)
			|| !hdd_ipa->sta_connected)
#endif
			&& (VOS_TRUE == hdd_ipa->uc_loaded)
		) {
			ret = hdd_ipa_uc_handle_first_con(hdd_ipa);
			if (ret) {
				vos_lock_release(&hdd_ipa->event_lock);
				HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"%s: handle 1st con ret %d",
					adapter->dev->name, ret);
				return ret;
			}
		}

		hdd_ipa->sap_num_connected_sta++;

		vos_lock_release(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		return ret;

	case WLAN_CLIENT_DISCONNECT:
#ifdef IPA_UC_OFFLOAD
                if (!hdd_ipa_uc_is_enabled(hdd_ipa)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: IPA UC OFFLOAD NOT ENABLED",
				msg_ex->name);
			return 0;
		}

		vos_lock_acquire(&hdd_ipa->event_lock);
		if (!hdd_ipa_uc_find_add_assoc_sta(hdd_ipa,
					VOS_FALSE,
					sta_id)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"%s: STA ID %d NOT found, not valid",
				msg_ex->name, sta_id);
			vos_lock_release(&hdd_ipa->event_lock);
			return 0;
		}
		hdd_ipa->sap_num_connected_sta--;
		/* Disable IPA UC TX PIPE when last STA disconnected */
		if (!hdd_ipa->sap_num_connected_sta
#ifdef IPA_UC_STA_OFFLOAD
			&& (!hdd_ipa_uc_sta_is_enabled(hdd_ipa) ||
			!hdd_ipa->sta_connected)
#endif
			&& (VOS_TRUE == hdd_ipa->uc_loaded)
			&& (VOS_FALSE == hdd_ipa->resource_unloading)
			&& (HDD_IPA_UC_NUM_WDI_PIPE ==
				hdd_ipa->activated_fw_pipe)
		) {
			hdd_ipa_uc_handle_last_discon(hdd_ipa);
		}
		vos_lock_release(&hdd_ipa->event_lock);
#endif /* IPA_UC_OFFLOAD */
		break;

	default:
		return 0;
	}

	meta.msg_len = sizeof(struct ipa_wlan_msg);
	msg = adf_os_mem_alloc(NULL, meta.msg_len);
	if (msg == NULL) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR, "msg allocation failed");
		return -ENOMEM;
	}

	meta.msg_type = type;
	strlcpy(msg->name, adapter->dev->name, IPA_RESOURCE_NAME_MAX);
	memcpy(msg->mac_addr, mac_addr, ETH_ALEN);

	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%s: Evt: %d",
					msg->name, meta.msg_type);

	ret = ipa_send_msg(&meta, msg, hdd_ipa_msg_free_fn);

	if (ret) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO, "%s: Evt: %d fail:%d",
					msg->name, meta.msg_type,  ret);
		adf_os_mem_free(msg);
		return ret;
	}

	hdd_ipa->stats.num_send_msg++;

end:
	return ret;
}


static void hdd_ipa_rx_pipe_desc_free(void)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	uint32_t i = 0, max_desc_cnt;
	struct ipa_tx_data_desc *desc, *tmp;

	max_desc_cnt = hdd_ipa->hw_desc_cnt * HDD_IPA_DESC_BUFFER_RATIO;

	spin_lock_bh(&hdd_ipa->q_lock);

	list_for_each_entry_safe(desc, tmp, &hdd_ipa->pend_desc_head, link) {
		list_del(&desc->link);
		adf_nbuf_free(desc->priv);
		spin_unlock_bh(&hdd_ipa->q_lock);
		hdd_ipa_free_data_desc(hdd_ipa, desc);
		spin_lock_bh(&hdd_ipa->q_lock);
	}

	list_for_each_entry_safe(desc, tmp, &hdd_ipa->free_desc_head, link) {
		list_del(&desc->link);
		spin_unlock_bh(&hdd_ipa->q_lock);
		adf_os_mem_free(desc);
		spin_lock_bh(&hdd_ipa->q_lock);
		i++;
	}
	spin_unlock_bh(&hdd_ipa->q_lock);

	if (i != max_desc_cnt)
		HDD_IPA_LOG(VOS_TRACE_LEVEL_FATAL, "free desc leak: %u, %u", i,
				max_desc_cnt);

}


static int hdd_ipa_rx_pipe_desc_alloc(void)
{
	struct hdd_ipa_priv *hdd_ipa = ghdd_ipa;
	uint32_t i, max_desc_cnt;
	int ret = 0;
	struct ipa_tx_data_desc *tmp_desc;

	hdd_ipa->hw_desc_cnt = IPA_NUM_OF_FIFO_DESC(
				hdd_ipa->hdd_ctx->cfg_ini->IpaDescSize);
	max_desc_cnt = hdd_ipa->hw_desc_cnt * HDD_IPA_DESC_BUFFER_RATIO;

	spin_lock_init(&hdd_ipa->q_lock);

	INIT_LIST_HEAD(&hdd_ipa->free_desc_head);
	INIT_LIST_HEAD(&hdd_ipa->pend_desc_head);
	hdd_ipa->freeq_cnt = max_desc_cnt;
	for (i = 0; i < max_desc_cnt; i++) {
		tmp_desc = adf_os_mem_alloc(NULL,
				sizeof(struct ipa_tx_data_desc));
		if (!tmp_desc) {
			ret = -ENOMEM;

			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
					"Descriptor allocation failed");
			goto fail;
		}
		spin_lock_bh(&hdd_ipa->q_lock);
		list_add_tail(&tmp_desc->link, &hdd_ipa->free_desc_head);
		spin_unlock_bh(&hdd_ipa->q_lock);
	}


	HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
		"Desc sz:%d h_desc_cnt:%d freeq_cnt:%u",
		hdd_ipa->hdd_ctx->cfg_ini->IpaDescSize, hdd_ipa->hw_desc_cnt,
						hdd_ipa->freeq_cnt);
	return ret;
fail:
	hdd_ipa_rx_pipe_desc_free();
	return ret;
}

static inline char *hdd_ipa_rm_state_to_str(enum hdd_ipa_rm_state state)
{
	switch (state) {
	case HDD_IPA_RM_RELEASED: return "RELEASED";
	case HDD_IPA_RM_GRANT_PENDING: return "GRANT_PENDING";
	case HDD_IPA_RM_GRANTED: return "GRANTED";
	}

	return "UNKNOWN";
}

static ssize_t hdd_ipa_debugfs_read_ipa_stats(struct file *file,
		char __user *user_buf, size_t count, loff_t *ppos)
{
	struct  hdd_ipa_priv *hdd_ipa = file->private_data;
	char *buf;
	unsigned int len = 0, buf_len = 4096;
	ssize_t ret_cnt;
	int i;
	struct hdd_ipa_iface_context *iface_context = NULL;
#define HDD_IPA_STATS(_buf, _len, _hdd_ipa, _name) \
	scnprintf(_buf, _len, "%30s: %llu\n", #_name, _hdd_ipa->stats._name)

#define HDD_IPA_IFACE_STATS(_buf, _len, _iface, _name) \
	scnprintf(_buf, _len, "%30s: %llu\n", #_name, _iface->stats._name)

	buf = kzalloc(buf_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += scnprintf(buf + len, buf_len - len,
			"\nhw_desc_cnt/pending_cnt: %u/%u, "
			"freeq_cnt/pend_q_cnt: %u/%u\n", hdd_ipa->hw_desc_cnt,
			hdd_ipa->pending_hw_desc_cnt, hdd_ipa->freeq_cnt,
			hdd_ipa->pend_q_cnt);

	len += scnprintf(buf + len, buf_len - len,
			"\n<------------------ WLAN EVENTS STATS"
			" ------------------>\n");
	for (i = 0; i < IPA_WLAN_EVENT_MAX; i++) {
		len += scnprintf(buf + len, buf_len - len, "%30s: %u\n",
				hdd_ipa_wlan_event_to_str(i),
				hdd_ipa->stats.event[i]);
	}
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa, num_send_msg);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa, num_free_msg);

	if (!hdd_ipa_is_rm_enabled(hdd_ipa))
		goto skip;

	len += scnprintf(buf + len, buf_len - len,
			"\n<------------------ IPA RM STATS"
			" ------------------>\n");

	len += scnprintf(buf + len, buf_len - len, "%30s: %s\n", "rm_state",
			hdd_ipa_rm_state_to_str(hdd_ipa->rm_state));

	len += scnprintf(buf + len, buf_len - len, "%30s: %s\n", "wake_lock",
			hdd_ipa->wake_lock_released ? "RELEASED" : "ACQUIRED");

	len += scnprintf(buf + len, buf_len - len, "%30s: %d\n", "tx_ref_cnt",
			atomic_read(&hdd_ipa->tx_ref_cnt));

	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa, num_rm_grant);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa, num_rm_release);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_rm_grant_imm);

	if (!hdd_ipa_is_clk_scaling_enabled(hdd_ipa))
		goto skip;

	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_cons_perf_req);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_prod_perf_req);
	len += scnprintf(buf + len, buf_len - len, "%30s: %u\n", "curr_prod_bw",
			hdd_ipa->curr_prod_bw);
	len += scnprintf(buf + len, buf_len - len, "%30s: %u\n", "curr_cons_bw",
			hdd_ipa->curr_cons_bw);

skip:
	len += scnprintf(buf + len, buf_len - len,
			"\n<------------------ IPA STATS"
			" ------------------>\n");
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa, num_rx_drop);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_rx_ipa_tx_dp);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_rx_ipa_splice);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_rx_ipa_loop);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_rx_ipa_tx_dp_err);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_rx_ipa_write_done);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_max_ipa_tx_mul);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_rx_ipa_hw_maxed_out);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			max_pend_q_cnt);

	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_tx_comp_cnt);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_tx_queued);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_tx_dequeued);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_max_pm_queue);

	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_freeq_empty);
	len += HDD_IPA_STATS(buf + len, buf_len - len, hdd_ipa,
			num_pri_freeq_empty);

	len += scnprintf(buf + len, buf_len - len,
			"\n<------------------ IPA IFACE STATS"
			" ------------------>\n");

	for (i = 0; i < HDD_IPA_MAX_IFACE; i++) {

		iface_context = &hdd_ipa->iface_context[i];

		if (iface_context->adapter == NULL)
			continue;

		len += scnprintf(buf + len, buf_len - len,
				"\n%s: iface_id: %u, sta_id: %u,"
				" device_mode: %u\n",
				iface_context->adapter->dev->name,
				iface_context->iface_id,
				iface_context->sta_id,
				iface_context->adapter->device_mode);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_tx);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_tx_drop);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_tx_err);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_tx_cac_drop);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_rx_prefilter);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_rx_ipa_excep);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_rx_recv);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_rx_recv_mul);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, num_rx_send_desc_err);
		len += HDD_IPA_IFACE_STATS(buf + len, buf_len - len,
				iface_context, max_rx_mul);
	}

	ret_cnt = simple_read_from_buffer(user_buf, count, ppos, buf, len);

	kfree(buf);
	return ret_cnt;
#undef HDD_IPA_STATS
#undef HDD_IPA_IFACE_STATS
}

static ssize_t hdd_ipa_debugfs_write_ipa_stats(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct  hdd_ipa_priv *hdd_ipa = file->private_data;
	struct hdd_ipa_iface_context *iface_context = NULL;
	int ret;
	uint32_t val;
	int i;

	ret = kstrtou32_from_user(user_buf, count, 0, &val);

	if (ret)
		return ret;

	if (val == 0) {
		for (i = 0; i < HDD_IPA_MAX_IFACE; i++) {
			iface_context = &hdd_ipa->iface_context[i];
			memset(&iface_context->stats, 0,
					sizeof(iface_context->stats));
		}

		memset(&hdd_ipa->stats, 0, sizeof(hdd_ipa->stats));
	}

	return count;
}

static const struct file_operations fops_ipa_stats = {
		.read = hdd_ipa_debugfs_read_ipa_stats,
		.write = hdd_ipa_debugfs_write_ipa_stats,
		.open = simple_open,
		.owner = THIS_MODULE,
		.llseek = default_llseek,
};


static int hdd_ipa_debugfs_init(struct hdd_ipa_priv *hdd_ipa)
{
#ifdef WLAN_OPEN_SOURCE
	hdd_ipa->debugfs_dir = debugfs_create_dir("cld",
					hdd_ipa->hdd_ctx->wiphy->debugfsdir);
	if (!hdd_ipa->debugfs_dir)
		return -ENOMEM;

	debugfs_create_file("ipa-stats", S_IRUSR, hdd_ipa->debugfs_dir,
						hdd_ipa, &fops_ipa_stats);
#endif
	return 0;
}

static void hdd_ipa_debugfs_remove(struct hdd_ipa_priv *hdd_ipa)
{
#ifdef WLAN_OPEN_SOURCE
	debugfs_remove_recursive(hdd_ipa->debugfs_dir);
#endif
}

/**
* hdd_ipa_init() - Allocate hdd_ipa resources, ipa pipe resource and register
* wlan interface with IPA module.
* @param
* hdd_ctx  : [in] pointer to HDD context
* @return         : VOS_STATUS_E_FAILURE - Errors
*                 : VOS_STATUS_SUCCESS - Ok
*/
VOS_STATUS hdd_ipa_init(hdd_context_t *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa = NULL;
	int ret=0, i;
	struct hdd_ipa_iface_context *iface_context = NULL;
#ifdef IPA_UC_OFFLOAD
	struct ipa_wdi_uc_ready_params uc_ready_param;
	struct ipa_msg_meta meta;
	struct ipa_wlan_msg *ipa_msg;
#endif /* IPA_UC_OFFLOAD */

	if (!hdd_ipa_is_enabled(hdd_ctx))
		return VOS_STATUS_SUCCESS;

	hdd_ipa = adf_os_mem_alloc(NULL, sizeof(*hdd_ipa));
	if (!hdd_ipa) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_FATAL, "hdd_ipa allocation failed");
		goto fail_setup_rm;
	}
	adf_os_mem_zero(hdd_ipa, sizeof(*hdd_ipa));

	hdd_ctx->hdd_ipa = hdd_ipa;
	ghdd_ipa = hdd_ipa;
	hdd_ipa->hdd_ctx = hdd_ctx;
	hdd_ipa->num_iface = 0;

	/* Create the interface context */
	for (i = 0; i < HDD_IPA_MAX_IFACE; i++) {
		iface_context = &hdd_ipa->iface_context[i];
		iface_context->hdd_ipa = hdd_ipa;
		iface_context->cons_client =
			hdd_ipa_adapter_2_client[i].cons_client;
		iface_context->prod_client =
			hdd_ipa_adapter_2_client[i].prod_client;
		iface_context->iface_id = i;
		iface_context->adapter = NULL;
		adf_os_spinlock_init(&iface_context->interface_lock);
	}

#ifdef CONFIG_CNSS
	cnss_init_work(&hdd_ipa->pm_work, hdd_ipa_pm_send_pkt_to_tl);
#else
	INIT_WORK(&hdd_ipa->pm_work, hdd_ipa_pm_send_pkt_to_tl);
#endif
	adf_os_spinlock_init(&hdd_ipa->pm_lock);
	adf_nbuf_queue_init(&hdd_ipa->pm_queue_head);

	ret = hdd_ipa_setup_rm(hdd_ipa);
	if (ret)
		goto fail_setup_rm;

#ifdef IPA_UC_OFFLOAD
	if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
		hdd_ipa_uc_rt_debug_init(hdd_ctx);
		vos_mem_zero(&hdd_ipa->stats, sizeof(hdd_ipa->stats));
		hdd_ipa->sap_num_connected_sta = 0;
		hdd_ipa->ipa_tx_packets_diff = 0;
		hdd_ipa->ipa_rx_packets_diff = 0;
		hdd_ipa->ipa_p_tx_packets = 0;
		hdd_ipa->ipa_p_rx_packets = 0;
		hdd_ipa->resource_loading = VOS_FALSE;
		hdd_ipa->resource_unloading = VOS_FALSE;
#ifdef IPA_UC_STA_OFFLOAD
		hdd_ipa->sta_connected = 0;

		/* Setup IPA sys_pipe for MCC */
		if (hdd_ipa_uc_sta_is_enabled(hdd_ipa)) {
			ret = hdd_ipa_setup_sys_pipe(hdd_ipa);
			if (ret)
				goto fail_create_sys_pipe;
		}
#endif
		hdd_ipa->wdi_enabled = VOS_FALSE;
		hdd_ipa->uc_loaded = VOS_FALSE;
		hdd_ipa->ipa_pipes_down = true;
		uc_ready_param.priv = (void *)hdd_ipa;
		uc_ready_param.notify = hdd_ipa_uc_loaded_uc_cb;
		if (ipa_uc_reg_rdyCB(&uc_ready_param)) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"UC Ready CB register fail");
			goto fail_setup_rm;
		}
		if (TRUE == uc_ready_param.is_uC_ready) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"UC Ready");
			hdd_ipa->uc_loaded = VOS_TRUE;
		} else {
			/* WDI disable message to IPA */
			meta.msg_len = sizeof(*ipa_msg);
			ipa_msg = adf_os_mem_alloc(NULL, meta.msg_len);
			if (ipa_msg == NULL) {
				hddLog(VOS_TRACE_LEVEL_ERROR,
					"msg allocation failed");
				goto fail_setup_rm;
			}
			meta.msg_type = WLAN_WDI_DISABLE;
			hddLog(VOS_TRACE_LEVEL_INFO,
				"ipa_send_msg(Evt:%d)", meta.msg_type);
			ret = ipa_send_msg(&meta, ipa_msg, hdd_ipa_msg_free_fn);
			if (ret) {
				hddLog(VOS_TRACE_LEVEL_ERROR,
					"ipa_send_msg(Evt:%d) - fail=%d",
					meta.msg_type, ret);
				adf_os_mem_free(ipa_msg);
				goto fail_setup_rm;
			}
		}
		hdd_ipa_uc_ol_init(hdd_ctx);
	} else
#endif /* IPA_UC_OFFLOAD */
	{
		ret = hdd_ipa_setup_sys_pipe(hdd_ipa);
		if (ret)
			goto fail_create_sys_pipe;

		ret = hdd_ipa_rx_pipe_desc_alloc();
		if (ret)
			goto fail_alloc_rx_pipe_desc;
	}

	ret = hdd_ipa_debugfs_init(hdd_ipa);
	if (ret)
		goto fail_alloc_rx_pipe_desc;

	if (!hdd_ipa_uc_is_enabled(hdd_ipa)) {
		hdd_ipa->ipv4_notifier.notifier_call = hdd_ipa_ipv4_changed;
		ret = register_inetaddr_notifier(&hdd_ipa->ipv4_notifier);
		if (ret)
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"WLAN IPv4 local filter register failed");
	}

	return VOS_STATUS_SUCCESS;
fail_alloc_rx_pipe_desc:
	hdd_ipa_rx_pipe_desc_free();
fail_create_sys_pipe:
	hdd_ipa_destory_rm_resource(hdd_ipa);
fail_setup_rm:
	if (hdd_ipa)
		adf_os_mem_free(hdd_ipa);

	return VOS_STATUS_E_FAILURE;
}

#ifdef IPA_UC_OFFLOAD
/**
* hdd_ipa_cleanup_pending_event() - Cleanup IPA pending event list
* @param
* hdd_ipa  : [in] pointer to HDD IPA struct
* @return  : none
*/
void hdd_ipa_cleanup_pending_event(struct hdd_ipa_priv *hdd_ipa)
{
	struct ipa_uc_pending_event *pending_event = NULL;

	while(vos_list_remove_front(&hdd_ipa->pending_event,
		(vos_list_node_t **)&pending_event) == VOS_STATUS_SUCCESS) {
		vos_mem_free(pending_event);
	}

	vos_list_destroy(&hdd_ipa->pending_event);
}
#endif

VOS_STATUS hdd_ipa_cleanup(hdd_context_t *hdd_ctx)
{
	struct hdd_ipa_priv *hdd_ipa = hdd_ctx->hdd_ipa;
	int i;
	struct hdd_ipa_iface_context *iface_context = NULL;
	adf_nbuf_t skb;
	struct hdd_ipa_pm_tx_cb *pm_tx_cb = NULL;

	if (!hdd_ipa_is_enabled(hdd_ctx))
		return VOS_STATUS_SUCCESS;

	if (!hdd_ipa_uc_is_enabled(hdd_ipa)) {
		unregister_inetaddr_notifier(&hdd_ipa->ipv4_notifier);
		hdd_ipa_teardown_sys_pipe(hdd_ipa);
	}

#ifdef IPA_UC_STA_OFFLOAD
	/* Teardown IPA sys_pipe for MCC */
	if (hdd_ipa_uc_sta_is_enabled(hdd_ipa))
		hdd_ipa_teardown_sys_pipe(hdd_ipa);
#endif

	hdd_ipa_destory_rm_resource(hdd_ipa);

#ifdef WLAN_OPEN_SOURCE
	cancel_work_sync(&hdd_ipa->pm_work);
#endif

	adf_os_spin_lock_bh(&hdd_ipa->pm_lock);

	while (((skb = adf_nbuf_queue_remove(&hdd_ipa->pm_queue_head)) !=
				NULL)) {
		adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);

		pm_tx_cb = (struct hdd_ipa_pm_tx_cb *)skb->cb;
		ipa_free_skb(pm_tx_cb->ipa_tx_desc);

		adf_os_spin_lock_bh(&hdd_ipa->pm_lock);
	}
	adf_os_spin_unlock_bh(&hdd_ipa->pm_lock);

	adf_os_spinlock_destroy(&hdd_ipa->pm_lock);

	/* Destroy the interface lock */
	for (i = 0; i < HDD_IPA_MAX_IFACE; i++) {
		iface_context = &hdd_ipa->iface_context[i];
		adf_os_spinlock_destroy(&iface_context->interface_lock);
	}

	hdd_ipa_debugfs_remove(hdd_ipa);

	/* This should never hit but still make sure that there are no pending
	 * descriptor in IPA hardware
	 */
	if (hdd_ipa->pending_hw_desc_cnt != 0) {
		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"IPA Pending write done: %d Waiting!",
				hdd_ipa->pending_hw_desc_cnt);

		for (i = 0; hdd_ipa->pending_hw_desc_cnt != 0 && i < 10; i++) {
			usleep_range(100, 100);
		}

		HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"IPA Pending write done: desc: %d %s(%d)!",
				hdd_ipa->pending_hw_desc_cnt,
				hdd_ipa->pending_hw_desc_cnt == 0 ? "completed"
				: "leak", i);
	}

#ifdef IPA_UC_OFFLOAD
	if (hdd_ipa_uc_is_enabled(hdd_ipa)) {
		if (ipa_uc_dereg_rdyCB())
			HDD_IPA_LOG(VOS_TRACE_LEVEL_ERROR,
				"UC Ready CB deregister fail");
		hdd_ipa_uc_rt_debug_deinit(hdd_ctx);
		if (VOS_TRUE == hdd_ipa->uc_loaded) {
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: Disconnect TX PIPE", __func__);
			ipa_disconnect_wdi_pipe(hdd_ipa->tx_pipe_handle);
			HDD_IPA_LOG(VOS_TRACE_LEVEL_INFO,
				"%s: Disconnect RX PIPE", __func__);
			ipa_disconnect_wdi_pipe(hdd_ipa->rx_pipe_handle);
		}
		vos_lock_destroy(&hdd_ipa->event_lock);
		hdd_ipa_cleanup_pending_event(hdd_ipa);
		vos_lock_destroy(&hdd_ipa->ipa_lock);
#ifdef WLAN_OPEN_SOURCE
		for (i = 0; i < HDD_IPA_UC_OPCODE_MAX; i++) {
			cancel_work_sync(&hdd_ipa->uc_op_work[i].work);
			hdd_ipa->uc_op_work[i].msg = NULL;
		}
#endif
	} else
#endif /* IPA_UC_OFFLOAD */
	{
		hdd_ipa_rx_pipe_desc_free();
	}

	adf_os_mem_free(hdd_ipa);
	hdd_ctx->hdd_ipa = NULL;

	return VOS_STATUS_SUCCESS;
}
#endif
