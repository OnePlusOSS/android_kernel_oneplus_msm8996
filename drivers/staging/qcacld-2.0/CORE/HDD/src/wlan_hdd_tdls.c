/*
 * Copyright (c) 2012-2016 The Linux Foundation. All rights reserved.
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

/**========================================================================

  \file  wlan_hdd_tdls.c

  \brief WLAN Host Device Driver implementation for TDLS

  ========================================================================*/

#include <wlan_hdd_includes.h>
#include <aniGlobal.h>
#include <wlan_hdd_hostapd.h>
#include <net/cfg80211.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/list.h>
#include <linux/etherdevice.h>
#include <net/ieee80211_radiotap.h>
#include "wlan_hdd_tdls.h"
#include "wlan_hdd_cfg80211.h"
#include "wlan_hdd_assoc.h"
#include "sme_Api.h"
#include "vos_sched.h"

/**
 * wlan_hdd_tdls_determine_channel_opclass() - determine channel and opclass
 * @hddctx: pointer to hdd context
 * @adapter: pointer to adapter
 * @curr_peer: pointer to current tdls peer
 * @channel: pointer to channel
 * @opclass: pointer to opclass
 *
 * Function determines the channel and operating class
 *
 * Return: None
 */
static void wlan_hdd_tdls_determine_channel_opclass(hdd_context_t *hddctx,
			hdd_adapter_t *adapter, hddTdlsPeer_t *curr_peer,
			uint32_t *channel, uint32_t *opclass)
{
	hdd_station_ctx_t *hdd_sta_ctx;

	/*
	 * If tdls offchannel is not enabled then we provide base channel
	 * and in that case pass opclass as 0 since opclass is mainly needed
	 * for offchannel cases.
	 */
	if (!(hddctx->cfg_ini->fEnableTDLSOffChannel) ||
		(hddctx->tdls_fw_off_chan_mode != ENABLE_CHANSWITCH)) {
		hdd_sta_ctx = WLAN_HDD_GET_STATION_CTX_PTR(adapter);
		*channel = hdd_sta_ctx->conn_info.operationChannel;
		*opclass = 0;
	} else {
		*channel = curr_peer->pref_off_chan_num;
		*opclass = curr_peer->op_class_for_pref_off_chan;
	}
}

static u8 wlan_hdd_tdls_hash_key (const u8 *mac)
{
    int i;
    u8 key = 0;

    for (i = 0; i < 6; i++)
       key ^= mac[i];

    return key;
}

/**
 * wlan_hdd_tdls_disable_offchan_and_teardown_links - Disable offchannel
 * and teardown TDLS links
 * @hddCtx : pointer to hdd context
 *
 * Return: None
 */
void wlan_hdd_tdls_disable_offchan_and_teardown_links(hdd_context_t *hddctx)
{
	u16 connected_tdls_peers = 0;
	u8 staidx;
	hddTdlsPeer_t *curr_peer;
	hdd_adapter_t *adapter = NULL;

	if (eTDLS_SUPPORT_NOT_ENABLED == hddctx->tdls_mode) {
		hddLog(LOG1, FL("TDLS mode is disabled OR not enabled in FW"));
		return ;
	}

	adapter = hdd_get_adapter(hddctx, WLAN_HDD_INFRA_STATION);

	if (adapter == NULL) {
		hddLog(LOGE, FL("Station Adapter Not Found"));
		return;
	}

	connected_tdls_peers = wlan_hdd_tdlsConnectedPeers(adapter);

	if (!connected_tdls_peers) {
		hddLog(LOG1, FL("No TDLS connected peers to delete"));
		return ;
	}

	/* TDLS is not supported in case of concurrency.
	 * Disable TDLS Offchannel in FW to avoid more
	 * than two concurrent channels and generate TDLS
	 * teardown indication to supplicant.
	 * Below function Finds the first connected peer and
	 * disables TDLS offchannel for that peer.
	 * FW enables TDLS offchannel only when there is
	 * one TDLS peer. When there are more than one TDLS peer,
	 * there will not be TDLS offchannel in FW.
	 * So to avoid sending multiple request to FW, for now,
	 * just invoke offchannel mode functions only once
	 */
	hdd_set_tdls_offchannel(hddctx, hddctx->cfg_ini->fTDLSPrefOffChanNum);
	hdd_set_tdls_secoffchanneloffset(hddctx,
			TDLS_SEC_OFFCHAN_OFFSET_40PLUS);
	hdd_set_tdls_offchannelmode(adapter, DISABLE_CHANSWITCH);

	/* Send Msg to PE for deleting all the TDLS peers */
	sme_delete_all_tdls_peers(hddctx->hHal, adapter->sessionId);


	for (staidx = 0; staidx < hddctx->max_num_tdls_sta;
							staidx++) {
		if (!hddctx->tdlsConnInfo[staidx].staId)
			continue;

		curr_peer = wlan_hdd_tdls_find_all_peer(hddctx,
				hddctx->tdlsConnInfo[staidx].peerMac.bytes);
		if (!curr_peer)
			continue;

		hddLog(LOG1, FL("indicate TDLS teardown (staId %d)"),
			curr_peer->staId);

		/* Indicate teardown to supplicant */
		wlan_hdd_tdls_indicate_teardown(
				curr_peer->pHddTdlsCtx->pAdapter,
				curr_peer,
				eSIR_MAC_TDLS_TEARDOWN_UNSPEC_REASON);

		/*
		 * Del Sta happened already as part of sme_delete_all_tdls_peers
		 * Hence clear hdd data structure.
		 */
		hdd_roamDeregisterTDLSSTA(adapter,
				hddctx->tdlsConnInfo[staidx].staId);
		wlan_hdd_tdls_decrement_peer_count(adapter);
		wlan_hdd_tdls_reset_peer(adapter, curr_peer->peerMac);

		hddctx->tdlsConnInfo[staidx].staId = 0;
		hddctx->tdlsConnInfo[staidx].sessionId = 255;

		vos_mem_zero(&hddctx->tdlsConnInfo[staidx].peerMac,
			sizeof(v_MACADDR_t));
	}
	wlan_hdd_tdls_check_bmps(adapter);
}

/**
 * hdd_tdls_notify_mode_change - Notify mode change
 * @adapter: pointer to hdd adapter
 * @hddCtx : pointer to hdd context
 *
 * Return: None
 */
void hdd_tdls_notify_mode_change(hdd_adapter_t *adapter, hdd_context_t *hddctx)
{
	wlan_hdd_tdls_disable_offchan_and_teardown_links(hddctx);
}

/* Caller has to take the lock before calling this function */
static tANI_S32 wlan_hdd_tdls_peer_reset_discovery_processed(tdlsCtx_t *pHddTdlsCtx)
{
    int i;
    struct list_head *head;
    hddTdlsPeer_t *tmp;
    struct list_head *pos, *q;

    for (i = 0; i < TDLS_PEER_LIST_SIZE; i++) {
        head = &pHddTdlsCtx->peer_list[i];
        list_for_each_safe (pos, q, head) {
            tmp = list_entry(pos, hddTdlsPeer_t, node);
            tmp->discovery_processed = 0;
        }
    }

    return 0;
}

static tANI_U32 wlan_hdd_tdls_discovery_sent_cnt(hdd_context_t *pHddCtx)
{
    hdd_adapter_list_node_t *pAdapterNode = NULL, *pNext = NULL;
    hdd_adapter_t *pAdapter = NULL;
    tdlsCtx_t *pHddTdlsCtx = NULL;
    VOS_STATUS status = 0;
    tANI_U32 count = 0;

    status = hdd_get_front_adapter ( pHddCtx, &pAdapterNode );
    while ( NULL != pAdapterNode && VOS_STATUS_SUCCESS == status )
    {
        pAdapter = pAdapterNode->pAdapter;

        pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
        if (NULL != pHddTdlsCtx)
        {
            count = count + pHddTdlsCtx->discovery_sent_cnt;
        }
        status = hdd_get_next_adapter ( pHddCtx, pAdapterNode, &pNext );
        pAdapterNode = pNext;
    }
    return count;
}

static void wlan_hdd_tdls_check_power_save_prohibited(hdd_adapter_t *pAdapter)
{
    tdlsCtx_t *pHddTdlsCtx = NULL;
    hdd_context_t *pHddCtx = NULL;

    if ((NULL == pAdapter) || (WLAN_HDD_ADAPTER_MAGIC != pAdapter->magic)) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("invalid pAdapter: %p"), pAdapter);
         return;
     }

    pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    pHddCtx = WLAN_HDD_GET_CTX(pAdapter);
    if ((NULL == pHddTdlsCtx) || (NULL == pHddCtx))
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                FL("pHddCtx or pHddTdlsCtx points to NULL"));
       return;
    }

    if ((0 == pHddCtx->connected_peer_count) &&
        (0 == wlan_hdd_tdls_discovery_sent_cnt(pHddCtx)))
    {
        sme_SetTdlsPowerSaveProhibited(WLAN_HDD_GET_HAL_CTX(pHddTdlsCtx->pAdapter),
                                       pAdapter->sessionId, 0);
        return;
    }
    sme_SetTdlsPowerSaveProhibited(WLAN_HDD_GET_HAL_CTX(pHddTdlsCtx->pAdapter),
                                   pAdapter->sessionId, 1);
    return;
}

static v_VOID_t wlan_hdd_tdls_discovery_timeout_peer_cb(v_PVOID_t userData)
{
    int i;
    struct list_head *head;
    hddTdlsPeer_t *tmp;
    struct list_head *pos, *q;
    tdlsCtx_t *pHddTdlsCtx;
    hdd_context_t *pHddCtx;

    ENTER();

    pHddTdlsCtx = (tdlsCtx_t *)userData;
    if ((NULL == pHddTdlsCtx) || (NULL == pHddTdlsCtx->pAdapter) )
    {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("pHddTdlsCtx or pAdapter points to NULL"));
        return;
    }

    if (WLAN_HDD_ADAPTER_MAGIC != pHddTdlsCtx->pAdapter->magic) {
        hddLog(LOGE, FL("pAdapter has invalid magic"));
        return;
    }

    pHddCtx = WLAN_HDD_GET_CTX( pHddTdlsCtx->pAdapter );
    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return;

    mutex_lock(&pHddCtx->tdls_lock);

    for (i = 0; i < TDLS_PEER_LIST_SIZE; i++) {
        head = &pHddTdlsCtx->peer_list[i];
        list_for_each_safe (pos, q, head) {
            tmp = list_entry(pos, hddTdlsPeer_t, node);
            if (eTDLS_LINK_DISCOVERING == tmp->link_status)
            {
                VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                           "%s: " MAC_ADDRESS_STR " to idle state", __func__,
                           MAC_ADDR_ARRAY(tmp->peerMac));
                mutex_unlock(&pHddCtx->tdls_lock);
                wlan_hdd_tdls_set_peer_link_status(tmp,
                                                   eTDLS_LINK_IDLE,
                                                   eTDLS_LINK_NOT_SUPPORTED);
                mutex_lock(&pHddCtx->tdls_lock);
            }
        }
    }

    pHddTdlsCtx->discovery_sent_cnt = 0;
    wlan_hdd_tdls_check_power_save_prohibited(pHddTdlsCtx->pAdapter);

    mutex_unlock(&pHddCtx->tdls_lock);

    wlan_hdd_tdls_check_bmps(pHddTdlsCtx->pAdapter);

    EXIT();
    return;
}

static void wlan_hdd_tdls_free_list(tdlsCtx_t *pHddTdlsCtx,
                                    bool del_forced_peer)
{
    int i;
    struct list_head *head;
    hddTdlsPeer_t *tmp;
    struct list_head *pos, *q;

    if (NULL == pHddTdlsCtx)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("pHddTdlsCtx is NULL"));
       return;
    }
    for (i = 0; i < TDLS_PEER_LIST_SIZE; i++)
    {
        head = &pHddTdlsCtx->peer_list[i];
        list_for_each_safe (pos, q, head) {
            tmp = list_entry(pos, hddTdlsPeer_t, node);
            /* Don't delete TDLS forced peers during STA disconnection */
            if (!del_forced_peer && tmp->isForcedPeer)
                continue;
            list_del(pos);
            vos_mem_free(tmp);
            tmp = NULL;
        }
    }
}

static void wlan_hdd_tdls_schedule_scan(struct work_struct *work)
{
    tdls_scan_context_t *scan_ctx =
          container_of(work, tdls_scan_context_t, tdls_scan_work.work);

    if (NULL == scan_ctx)
    {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("scan_ctx is NULL"));
        return;
    }

    if (unlikely(TDLS_CTX_MAGIC != scan_ctx->magic))
        return;

    scan_ctx->attempt++;

    wlan_hdd_cfg80211_scan(scan_ctx->wiphy,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
                           scan_ctx->dev,
#endif
                           scan_ctx->scan_request);
}

/* stop all monitoring timers per Adapter */
static void wlan_hdd_tdls_monitor_timers_stop(tdlsCtx_t *pHddTdlsCtx)
{
    vos_timer_stop(&pHddTdlsCtx->peerDiscoveryTimeoutTimer);
}

/* stop all the tdls timers running */
static void wlan_hdd_tdls_timers_stop(tdlsCtx_t *pHddTdlsCtx)
{
    wlan_hdd_tdls_monitor_timers_stop(pHddTdlsCtx);
}

int wlan_hdd_tdls_init(hdd_adapter_t *pAdapter)
{
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX( pAdapter );
    tdlsCtx_t *pHddTdlsCtx = NULL;
    int i;
    v_U8_t staIdx;
    tdlsInfo_t *tInfo;
    eHalStatus halStatus = eHAL_STATUS_FAILURE;

    if (NULL == pHddCtx)
        return -1;

    mutex_lock(&pHddCtx->tdls_lock);

    /* QCA 2.0 Discrete ANDs feature capability in cfg_ini with that
     * received from target, so cfg_ini gives combined intersected result
     */
    if ((FALSE == pHddCtx->cfg_ini->fEnableTDLSSupport)
       )
    {
        pHddCtx->tdls_mode = eTDLS_SUPPORT_NOT_ENABLED;
        pAdapter->sessionCtx.station.pHddTdlsCtx = NULL;
        mutex_unlock(&pHddCtx->tdls_lock);
        hddLog(VOS_TRACE_LEVEL_ERROR,
               "%s TDLS not enabled (%d) or FW doesn't support",
               __func__, pHddCtx->cfg_ini->fEnableTDLSSupport);
        return 0;
    }
    /* TDLS is supported only in STA / P2P Client modes,
     * hence the check for TDLS support in a specific Device mode.
     * Do not return a failure rather do not continue further
     * with the initialization as tdls_init would be called
     * during the open adapter for a p2p interface at which point
     * the device mode would be a P2P_DEVICE. The point here is to
     * continue initialization for STA / P2P Client modes.
     * TDLS exit also check for the device mode for clean up hence
     * there is no issue even if success is returned.
     */
    if (0 == WLAN_HDD_IS_TDLS_SUPPORTED_ADAPTER(pAdapter))
    {
        mutex_unlock(&pHddCtx->tdls_lock);
        return 0;
    }
    /* Check for the valid pHddTdlsCtx. If valid do not further
     * allocate the memory, rather continue with the initialization.
     * If tdls_initialization would get reinvoked  without tdls_exit
     * getting invoked (SSR) there is no point to further proceed
     * with the memory allocations.
     */
    if (NULL == pAdapter->sessionCtx.station.pHddTdlsCtx)
    {
        pHddTdlsCtx = vos_mem_malloc(sizeof(tdlsCtx_t));

        if (NULL == pHddTdlsCtx) {
            pAdapter->sessionCtx.station.pHddTdlsCtx = NULL;
            mutex_unlock(&pHddCtx->tdls_lock);
            hddLog(VOS_TRACE_LEVEL_ERROR, "%s malloc failed!", __func__);
            return -1;
        }
        /* initialize TDLS pAdater context */
        vos_mem_zero(pHddTdlsCtx, sizeof(tdlsCtx_t));

        vos_timer_init(&pHddTdlsCtx->peerDiscoveryTimeoutTimer,
                VOS_TIMER_TYPE_SW,
                wlan_hdd_tdls_discovery_timeout_peer_cb,
                pHddTdlsCtx);

        pAdapter->sessionCtx.station.pHddTdlsCtx = pHddTdlsCtx;

        for (i = 0; i < TDLS_PEER_LIST_SIZE; i++)
            INIT_LIST_HEAD(&pHddTdlsCtx->peer_list[i]);
    } else {
        struct list_head *head, *pos, *q;
        hddTdlsPeer_t *tmp = NULL;

        pHddTdlsCtx = pAdapter->sessionCtx.station.pHddTdlsCtx;

        /* stop all timers */
        wlan_hdd_tdls_timers_stop(pHddTdlsCtx);

        /* remove entries from peer list only if peer is not forced */
        for (i = 0; i < TDLS_PEER_LIST_SIZE; i++) {
            head = &pHddTdlsCtx->peer_list[i];
            list_for_each_safe(pos, q, head) {
                tmp = list_entry(pos, hddTdlsPeer_t, node);
                if (FALSE == tmp->isForcedPeer) {
                    list_del(pos);
                    vos_mem_free(tmp);
                    tmp = NULL;
                } else {
                    tmp->link_status = eTDLS_LINK_IDLE;
                    tmp->reason = eTDLS_LINK_UNSPECIFIED;
                    tmp->staId = 0;
                    tmp->discovery_attempt = 0;
                }
            }
        }
        /* reset tdls peer count to 0 */
        pHddCtx->connected_peer_count = 0;
    }

    /* initialize TDLS global context */
    pHddCtx->connected_peer_count = 0;
    pHddCtx->tdls_nss_switch_in_progress = false;
    pHddCtx->tdls_teardown_peers_cnt = 0;
    pHddCtx->tdls_nss_teardown_complete = false;
    pHddCtx->tdls_nss_transition_mode = TDLS_NSS_TRANSITION_UNKNOWN;

    sme_SetTdlsPowerSaveProhibited(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                   pAdapter->sessionId, 0);

    pHddCtx->tdls_scan_ctxt.magic = 0;
    pHddCtx->tdls_scan_ctxt.attempt = 0;
    pHddCtx->tdls_scan_ctxt.reject = 0;
    pHddCtx->tdls_scan_ctxt.scan_request = NULL;

    if (pHddCtx->cfg_ini->fEnableTDLSSleepSta ||
        pHddCtx->cfg_ini->fEnableTDLSBufferSta ||
        pHddCtx->cfg_ini->fEnableTDLSOffChannel)
        pHddCtx->max_num_tdls_sta = HDD_MAX_NUM_TDLS_STA_P_UAPSD_OFFCHAN;
    else
        pHddCtx->max_num_tdls_sta = HDD_MAX_NUM_TDLS_STA;

    hddLog(VOS_TRACE_LEVEL_INFO_HIGH, FL("max_num_tdls_sta: %d"),
           pHddCtx->max_num_tdls_sta);

    for (staIdx = 0; staIdx < pHddCtx->max_num_tdls_sta; staIdx++)
    {
         pHddCtx->tdlsConnInfo[staIdx].staId = 0;
         pHddCtx->tdlsConnInfo[staIdx].sessionId = 255;
         vos_mem_zero(&pHddCtx->tdlsConnInfo[staIdx].peerMac,
                                            sizeof(v_MACADDR_t)) ;
    }

    pHddTdlsCtx->pAdapter = pAdapter;

    pHddTdlsCtx->curr_candidate = NULL;
    pHddTdlsCtx->magic = 0;

    /* remember configuration even if it is not used right now. it could be used later */
    pHddTdlsCtx->threshold_config.tx_period_t = pHddCtx->cfg_ini->fTDLSTxStatsPeriod;
    pHddTdlsCtx->threshold_config.tx_packet_n = pHddCtx->cfg_ini->fTDLSTxPacketThreshold;
    pHddTdlsCtx->threshold_config.discovery_period_t = pHddCtx->cfg_ini->fTDLSDiscoveryPeriod;
    pHddTdlsCtx->threshold_config.discovery_tries_n = pHddCtx->cfg_ini->fTDLSMaxDiscoveryAttempt;
    pHddTdlsCtx->threshold_config.idle_timeout_t = pHddCtx->cfg_ini->fTDLSIdleTimeout;
    pHddTdlsCtx->threshold_config.idle_packet_n = pHddCtx->cfg_ini->fTDLSIdlePacketThreshold;
    pHddTdlsCtx->threshold_config.rssi_hysteresis = pHddCtx->cfg_ini->fTDLSRSSIHysteresis;
    pHddTdlsCtx->threshold_config.rssi_trigger_threshold = pHddCtx->cfg_ini->fTDLSRSSITriggerThreshold;
    pHddTdlsCtx->threshold_config.rssi_teardown_threshold = pHddCtx->cfg_ini->fTDLSRSSITeardownThreshold;
    pHddTdlsCtx->threshold_config.rssi_delta = pHddCtx->cfg_ini->fTDLSRSSIDelta;

    if (FALSE == pHddCtx->cfg_ini->fEnableTDLSImplicitTrigger)
    {
        pHddCtx->tdls_mode = eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY;
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s TDLS Implicit trigger not enabled!", __func__);
    } else if (TRUE == pHddCtx->cfg_ini->fTDLSExternalControl) {
        pHddCtx->tdls_mode = eTDLS_SUPPORT_EXTERNAL_CONTROL;
    } else {
        pHddCtx->tdls_mode = eTDLS_SUPPORT_ENABLED;
    }

#ifdef CONFIG_CNSS
    cnss_init_delayed_work(&pHddCtx->tdls_scan_ctxt.tdls_scan_work,
                   wlan_hdd_tdls_schedule_scan);
#else
#ifdef WLAN_OPEN_SOURCE
    INIT_DELAYED_WORK(&pHddCtx->tdls_scan_ctxt.tdls_scan_work,
                      wlan_hdd_tdls_schedule_scan);
#endif
#endif

    /*
     * Release tdls lock before calling in SME api
     * which would try to acquire sme lock.
     */
    mutex_unlock(&pHddCtx->tdls_lock);
    tInfo = vos_mem_malloc(sizeof(tdlsInfo_t));
    if (NULL == tInfo)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR,
               "%s: vos_mem_alloc failed for tInfo", __func__);
        vos_timer_destroy(&pHddTdlsCtx->peerDiscoveryTimeoutTimer);
        vos_mem_free(pHddTdlsCtx);
        return -1;
    }

    tInfo->vdev_id = pAdapter->sessionId;
    tInfo->tdls_state = pHddCtx->tdls_mode;
    tInfo->notification_interval_ms = pHddTdlsCtx->threshold_config.tx_period_t;
    tInfo->tx_discovery_threshold = pHddTdlsCtx->threshold_config.tx_packet_n;
    tInfo->tx_teardown_threshold = pHddTdlsCtx->threshold_config.idle_packet_n;
    tInfo->rssi_teardown_threshold =
        pHddTdlsCtx->threshold_config.rssi_teardown_threshold;
    tInfo->rssi_delta = pHddTdlsCtx->threshold_config.rssi_delta;
    tInfo->tdls_options = 0;
    if (pHddCtx->cfg_ini->fEnableTDLSOffChannel) {
        tInfo->tdls_options |= ENA_TDLS_OFFCHAN;
        pHddCtx->tdls_fw_off_chan_mode = ENABLE_CHANSWITCH;
    }
    if (pHddCtx->cfg_ini->fEnableTDLSBufferSta)
        tInfo->tdls_options |= ENA_TDLS_BUFFER_STA;
    if (pHddCtx->cfg_ini->fEnableTDLSSleepSta)
        tInfo->tdls_options |= ENA_TDLS_SLEEP_STA;
    tInfo->peer_traffic_ind_window =
        pHddCtx->cfg_ini->fTDLSPuapsdPTIWindow;
    tInfo->peer_traffic_response_timeout =
        pHddCtx->cfg_ini->fTDLSPuapsdPTRTimeout;
    tInfo->puapsd_mask =
        pHddCtx->cfg_ini->fTDLSUapsdMask;
    tInfo->puapsd_inactivity_time =
        pHddCtx->cfg_ini->fTDLSPuapsdInactivityTimer;
    tInfo->puapsd_rx_frame_threshold =
        pHddCtx->cfg_ini->fTDLSRxFrameThreshold;
    tInfo->teardown_notification_ms =
        pHddCtx->cfg_ini->fTDLSIdleTimeout;
    tInfo->tdls_peer_kickout_threshold =
        pHddCtx->cfg_ini->tdls_peer_kickout_threshold;

    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
              "%s: Setting tdls state and param in fw: "
              "vdev_id: %d, "
              "tdls_state: %d, "
              "notification_interval_ms: %d, "
              "tx_discovery_threshold: %d, "
              "tx_teardown_threshold: %d, "
              "rssi_teardown_threshold: %d, "
              "rssi_delta: %d, "
              "tdls_options: 0x%x, "
              "peer_traffic_ind_window: %d, "
              "peer_traffic_response_timeout: %d, "
              "puapsd_mask: 0x%x, "
              "puapsd_inactivity_time: %d, "
              "puapsd_rx_frame_threshold: %d, "
              "teardown_notification_ms: %d, "
              "tdls_peer_kickout_threshold: %d ",
              __func__,
              tInfo->vdev_id,
              tInfo->tdls_state,
              tInfo->notification_interval_ms,
              tInfo->tx_discovery_threshold,
              tInfo->tx_teardown_threshold,
              tInfo->rssi_teardown_threshold,
              tInfo->rssi_delta,
              tInfo->tdls_options,
              tInfo->peer_traffic_ind_window,
              tInfo->peer_traffic_response_timeout,
              tInfo->puapsd_mask,
              tInfo->puapsd_inactivity_time,
              tInfo->puapsd_rx_frame_threshold,
              tInfo->teardown_notification_ms,
              tInfo->tdls_peer_kickout_threshold);

    halStatus = sme_UpdateFwTdlsState(pHddCtx->hHal, tInfo, TRUE);
    if (eHAL_STATUS_SUCCESS != halStatus)
    {
        vos_mem_free(tInfo);
        vos_timer_destroy(&pHddTdlsCtx->peerDiscoveryTimeoutTimer);
        vos_mem_free(pHddTdlsCtx);
        return -1;
    }

    return 0;
}

static void wlan_hdd_tdls_monitor_timers_destroy(tdlsCtx_t *pHddTdlsCtx)
{
    vos_timer_stop(&pHddTdlsCtx->peerDiscoveryTimeoutTimer);
    vos_timer_destroy(&pHddTdlsCtx->peerDiscoveryTimeoutTimer);
}

/* destroy all the tdls timers running */
static void wlan_hdd_tdls_timers_destroy(tdlsCtx_t *pHddTdlsCtx)
{
    wlan_hdd_tdls_monitor_timers_destroy(pHddTdlsCtx);
}

static void wlan_hdd_tdls_free_scan_request(tdls_scan_context_t *tdls_scan_ctx)
{
    if (NULL == tdls_scan_ctx) {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                FL("tdls_scan_ctx is NULL"));
       return;
    }

    tdls_scan_ctx->attempt = 0;
    tdls_scan_ctx->reject = 0;
    tdls_scan_ctx->magic = 0;
    tdls_scan_ctx->scan_request = NULL;
    return;
}

void wlan_hdd_tdls_exit(hdd_adapter_t *pAdapter)
{
    tdlsCtx_t *pHddTdlsCtx;
    hdd_context_t *pHddCtx;
    tdlsInfo_t *tInfo;
    eHalStatus halStatus = eHAL_STATUS_FAILURE;

    pHddCtx = WLAN_HDD_GET_CTX( pAdapter );
    if (!pHddCtx)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_WARN,
                 FL("pHddCtx is NULL"));
       return;
    }

    if (!test_bit(TDLS_INIT_DONE, &pAdapter->event_flags)) {
        hddLog(LOGE, FL("TDLS init was not done, exit"));
        return;
    }

    mutex_lock(&pHddCtx->tdls_lock);

    pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    if (NULL == pHddTdlsCtx) {
       /* TDLS context can be null and might have been freed up during
        * cleanup for STA adapter
        */
       mutex_unlock(&pHddCtx->tdls_lock);

       hddLog(LOG2, FL("pHddTdlsCtx is NULL, adapter device mode %s(%d)"),
              hdd_device_mode_to_string(pAdapter->device_mode),
              pAdapter->device_mode);
       goto done;
    }

    vos_flush_delayed_work(&pHddCtx->tdls_scan_ctxt.tdls_scan_work);


    /* must stop timer here before freeing peer list, because peerIdleTimer is
    part of peer list structure. */
    wlan_hdd_tdls_timers_destroy(pHddTdlsCtx);
    wlan_hdd_tdls_free_list(pHddTdlsCtx, true);

    mutex_unlock(&pHddCtx->tdls_lock);

    wlan_hdd_tdls_free_scan_request(&pHddCtx->tdls_scan_ctxt);

    /* No need to post message during driver unload because MC thread is
      already shutdown */
    if ( !pHddCtx->isUnloadInProgress)
    {
        tInfo = vos_mem_malloc(sizeof(tdlsInfo_t));
        if (NULL != tInfo)
        {
            tInfo->vdev_id = pAdapter->sessionId;
            tInfo->tdls_state = eTDLS_SUPPORT_DISABLED;

            mutex_lock(&pHddCtx->tdls_lock);

            tInfo->notification_interval_ms =
              pHddTdlsCtx->threshold_config.tx_period_t;
            tInfo->tx_discovery_threshold =
              pHddTdlsCtx->threshold_config.tx_packet_n;
            tInfo->tx_teardown_threshold =
              pHddTdlsCtx->threshold_config.idle_packet_n;
            tInfo->rssi_teardown_threshold =
              pHddTdlsCtx->threshold_config.rssi_teardown_threshold;
            tInfo->rssi_delta = pHddTdlsCtx->threshold_config.rssi_delta;

            mutex_unlock(&pHddCtx->tdls_lock);

            tInfo->tdls_options = 0;
            if (pHddCtx->cfg_ini->fEnableTDLSOffChannel)
                tInfo->tdls_options |= ENA_TDLS_OFFCHAN;
            if (pHddCtx->cfg_ini->fEnableTDLSBufferSta)
                tInfo->tdls_options |= ENA_TDLS_BUFFER_STA;
            if (pHddCtx->cfg_ini->fEnableTDLSSleepSta)
                tInfo->tdls_options |= ENA_TDLS_SLEEP_STA;
            tInfo->peer_traffic_ind_window =
                pHddCtx->cfg_ini->fTDLSPuapsdPTIWindow;
            tInfo->peer_traffic_response_timeout =
                pHddCtx->cfg_ini->fTDLSPuapsdPTRTimeout;
            tInfo->puapsd_mask =
                pHddCtx->cfg_ini->fTDLSUapsdMask;
            tInfo->puapsd_inactivity_time =
                pHddCtx->cfg_ini->fTDLSPuapsdInactivityTimer;
            tInfo->puapsd_rx_frame_threshold =
                pHddCtx->cfg_ini->fTDLSRxFrameThreshold;
            tInfo->teardown_notification_ms =
                pHddCtx->cfg_ini->fTDLSIdleTimeout;
            tInfo->tdls_peer_kickout_threshold =
                pHddCtx->cfg_ini->tdls_peer_kickout_threshold;


            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                  "%s: Setting tdls state and param in fw: "
                  "vdev_id: %d, "
                  "tdls_state: %d, "
                  "notification_interval_ms: %d, "
                  "tx_discovery_threshold: %d, "
                  "tx_teardown_threshold: %d, "
                  "rssi_teardown_threshold: %d, "
                  "rssi_delta: %d, "
                  "tdls_options: 0x%x, "
                  "peer_traffic_ind_window: %d, "
                  "peer_traffic_response_timeout: %d, "
                  "puapsd_mask: 0x%x, "
                  "puapsd_inactivity_time: %d, "
                  "puapsd_rx_frame_threshold: %d, "
                  "teardown_notification_ms: %d, "
                  "tdls_peer_kickout_threshold: %d ",
                  __func__,
                  tInfo->vdev_id,
                  tInfo->tdls_state,
                  tInfo->notification_interval_ms,
                  tInfo->tx_discovery_threshold,
                  tInfo->tx_teardown_threshold,
                  tInfo->rssi_teardown_threshold,
                  tInfo->rssi_delta,
                  tInfo->tdls_options,
                  tInfo->peer_traffic_ind_window,
                  tInfo->peer_traffic_response_timeout,
                  tInfo->puapsd_mask,
                  tInfo->puapsd_inactivity_time,
                  tInfo->puapsd_rx_frame_threshold,
                  tInfo->teardown_notification_ms,
                  tInfo->tdls_peer_kickout_threshold);

            halStatus = sme_UpdateFwTdlsState(pHddCtx->hHal, tInfo, FALSE);
            if (eHAL_STATUS_SUCCESS != halStatus)
            {
                vos_mem_free(tInfo);
            }
      }
      else
      {
        hddLog(VOS_TRACE_LEVEL_ERROR,
               "%s: vos_mem_alloc failed for tInfo", __func__);
      }
   }

    mutex_lock(&pHddCtx->tdls_lock);

    pHddTdlsCtx->magic = 0;
    pHddTdlsCtx->pAdapter = NULL;

    vos_mem_free(pHddTdlsCtx);
    pAdapter->sessionCtx.station.pHddTdlsCtx = NULL;
    pHddTdlsCtx = NULL;

    mutex_unlock(&pHddCtx->tdls_lock);

done:
    clear_bit(TDLS_INIT_DONE, &pAdapter->event_flags);
}

/* if mac address exist, return pointer
   if mac address doesn't exist, create a list and add, return pointer
   return NULL if fails to get new mac address
*/
hddTdlsPeer_t *wlan_hdd_tdls_get_peer(hdd_adapter_t *pAdapter, const u8 *mac)
{
    struct list_head *head;
    hddTdlsPeer_t *peer;
    u8 key;
    tdlsCtx_t *pHddTdlsCtx;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return NULL;

    /* if already there, just update */
    peer = wlan_hdd_tdls_find_peer(pAdapter, mac, TRUE);
    if (peer != NULL)
    {
        return peer;
    }

    /* not found, allocate and add the list */
    peer = vos_mem_malloc(sizeof(hddTdlsPeer_t));
    if (NULL == peer) {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s peer malloc failed!", __func__);
        return NULL;
    }

    mutex_lock(&pHddCtx->tdls_lock);

    pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);

    if (NULL == pHddTdlsCtx) {
        vos_mem_free(peer);
        mutex_unlock(&pHddCtx->tdls_lock);
        hddLog(LOG1, FL("pHddTdlsCtx is NULL"));
        return NULL;
    }

    key = wlan_hdd_tdls_hash_key(mac);
    head = &pHddTdlsCtx->peer_list[key];

    vos_mem_zero(peer, sizeof(hddTdlsPeer_t));
    vos_mem_copy(peer->peerMac, mac, sizeof(peer->peerMac));
    peer->pHddTdlsCtx = pHddTdlsCtx;
    peer->pref_off_chan_num = pHddCtx->cfg_ini->fTDLSPrefOffChanNum;
    peer->op_class_for_pref_off_chan =
          wlan_hdd_find_opclass(pHddCtx->hHal, peer->pref_off_chan_num,
                                pHddCtx->cfg_ini->fTDLSPrefOffChanBandwidth);

    list_add_tail(&peer->node, head);
    mutex_unlock(&pHddCtx->tdls_lock);
    return peer;
}

int wlan_hdd_tdls_set_cap(hdd_adapter_t *pAdapter,
                                   const u8* mac,
                                   tTDLSCapType cap)
{
    hddTdlsPeer_t *curr_peer;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    curr_peer->tdls_support = cap;

    return 0;
}

void wlan_hdd_tdls_set_peer_link_status(hddTdlsPeer_t *curr_peer,
                                        tTDLSLinkStatus status,
                                        tTDLSLinkReason reason)
{
    tANI_U32 state = 0;
    tANI_S32 res = 0;
    hdd_context_t *pHddCtx;

    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return;
    }

    if (curr_peer->pHddTdlsCtx == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer->pHddTdlsCtx is NULL"));
       return;
    }

    pHddCtx = WLAN_HDD_GET_CTX(curr_peer->pHddTdlsCtx->pAdapter);

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("pHddCtx is not valid"));
       return;
    }

    hddLog(VOS_TRACE_LEVEL_WARN, "tdls set peer " MAC_ADDRESS_STR " link status to %u",
            MAC_ADDR_ARRAY(curr_peer->peerMac), status);

    mutex_lock(&pHddCtx->tdls_lock);

    curr_peer->link_status = status;

    /* If TDLS link status is already passed the discovery state
     * then clear discovery attempt count
     */
    if (status >= eTDLS_LINK_DISCOVERED)
    {
        curr_peer->discovery_attempt = 0;
    }

    mutex_unlock(&pHddCtx->tdls_lock);

    if (curr_peer->isForcedPeer && curr_peer->state_change_notification)
    {
        uint32_t opclass;
        uint32_t channel;
        hdd_adapter_t *adapter = curr_peer->pHddTdlsCtx->pAdapter;

        /*save the reason for any further query*/
        curr_peer->reason = reason;

        wlan_hdd_tdls_determine_channel_opclass(pHddCtx, adapter, curr_peer,
                                                &channel, &opclass);

        wlan_hdd_tdls_get_wifi_hal_state(curr_peer, &state, &res);

        (*curr_peer->state_change_notification)(curr_peer->peerMac, opclass,
                                                channel, state, res,
                                                adapter);

    }
    return;
}

void wlan_hdd_tdls_set_link_status(hdd_adapter_t *pAdapter,
                                   const u8* mac,
                                   tTDLSLinkStatus linkStatus,
                                   tTDLSLinkReason reason)
{
    tANI_U32 state = 0;
    tANI_S32 res = 0;
    hddTdlsPeer_t *curr_peer;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("pHddCtx is not valid"));
       return;
    }

    curr_peer = wlan_hdd_tdls_find_peer(pAdapter, mac, TRUE);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return;
    }

    mutex_lock(&pHddCtx->tdls_lock);

    curr_peer->link_status= linkStatus;

    /* If TDLS link status is already passed the discovery state
     * then clear discovery attempt count
     */
    if (linkStatus >= eTDLS_LINK_DISCOVERED)
    {
        curr_peer->discovery_attempt = 0;
    }

    mutex_unlock(&pHddCtx->tdls_lock);

    if (curr_peer->isForcedPeer && curr_peer->state_change_notification)
    {
        uint32_t opclass;
        uint32_t channel;
        hdd_adapter_t *adapter = curr_peer->pHddTdlsCtx->pAdapter;

        /*save the reason for any further query*/
        curr_peer->reason = reason;


        wlan_hdd_tdls_determine_channel_opclass(pHddCtx, adapter, curr_peer,
                                                &channel, &opclass);

        wlan_hdd_tdls_get_wifi_hal_state(curr_peer, &state, &res);

        (curr_peer->state_change_notification)(mac, opclass, channel, state,
                                          res, adapter);

    }

    return;
}

int wlan_hdd_tdls_recv_discovery_resp(hdd_adapter_t *pAdapter, u8 *mac)
{
    hddTdlsPeer_t *curr_peer;
    tdlsCtx_t *pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    hdd_context_t *pHddCtx;

    ENTER();

    if ( NULL == pHddTdlsCtx )
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("pHddTdlsCtx is NULL"));
       return -1;
    }

    pHddCtx = WLAN_HDD_GET_CTX(pHddTdlsCtx->pAdapter);

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return -1;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);

    if (NULL == curr_peer)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    if (pHddTdlsCtx->discovery_sent_cnt)
        pHddTdlsCtx->discovery_sent_cnt--;

    mutex_lock(&pHddCtx->tdls_lock);
    wlan_hdd_tdls_check_power_save_prohibited(pAdapter);

    mutex_unlock(&pHddCtx->tdls_lock);
    if (0 == pHddTdlsCtx->discovery_sent_cnt)
    {
        vos_timer_stop(&pHddTdlsCtx->peerDiscoveryTimeoutTimer);
    }

    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
               "Discovery(%u) Response from " MAC_ADDRESS_STR " link_status %d",
               pHddTdlsCtx->discovery_sent_cnt, MAC_ADDR_ARRAY(curr_peer->peerMac),
               curr_peer->link_status);

    if (eTDLS_LINK_DISCOVERING == curr_peer->link_status)
    {
        /*
         * Since we are here, it means Throughput threshold is already met.
         * Make sure RSSI threshold is also met before setting up TDLS link
         */
        if ((tANI_S32) curr_peer->rssi > (tANI_S32) pHddTdlsCtx->threshold_config.rssi_trigger_threshold)
        {
            wlan_hdd_tdls_set_peer_link_status(curr_peer,
                                               eTDLS_LINK_DISCOVERED,
                                               eTDLS_LINK_SUCCESS);
            VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
            "Rssi Threshold met: "MAC_ADDRESS_STR" rssi = %d threshold= %d" ,
             MAC_ADDR_ARRAY(curr_peer->peerMac), curr_peer->rssi,
             pHddTdlsCtx->threshold_config.rssi_trigger_threshold);
            cfg80211_tdls_oper_request(pAdapter->dev, curr_peer->peerMac, NL80211_TDLS_SETUP, FALSE, GFP_KERNEL);
        }
        else
        {
            VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
            "Rssi Threshold not met: "MAC_ADDRESS_STR" rssi = %d threshold = %d ",
            MAC_ADDR_ARRAY(curr_peer->peerMac), curr_peer->rssi,
            pHddTdlsCtx->threshold_config.rssi_trigger_threshold);
            wlan_hdd_tdls_set_peer_link_status(curr_peer,
                                               eTDLS_LINK_IDLE,
                                               eTDLS_LINK_UNSPECIFIED);

            /* if RSSI threshold is not met then allow further discovery
             * attempts by decrementing count for the last attempt
             */
            if (curr_peer->discovery_attempt)
                curr_peer->discovery_attempt--;
        }
    }
    else
    {
        wlan_hdd_tdls_check_bmps(pAdapter);
    }

    curr_peer->tdls_support = eTDLS_CAP_SUPPORTED;
    EXIT();
    return 0;
}

int wlan_hdd_tdls_set_peer_caps(hdd_adapter_t *pAdapter,
                                const u8 *mac,
                                tCsrStaParams *StaParams,
                                tANI_BOOLEAN isBufSta,
                                tANI_BOOLEAN isOffChannelSupported,
                                bool is_qos_wmm_sta)
{
    hddTdlsPeer_t *curr_peer;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    curr_peer->uapsdQueues = StaParams->uapsd_queues;
    curr_peer->maxSp = StaParams->max_sp;
    curr_peer->isBufSta = isBufSta;
    curr_peer->isOffChannelSupported = isOffChannelSupported;

    vos_mem_copy(curr_peer->supported_channels,
                 StaParams->supported_channels,
                 StaParams->supported_channels_len);

    curr_peer->supported_channels_len =
               StaParams->supported_channels_len;

    vos_mem_copy(curr_peer->supported_oper_classes,
                 StaParams->supported_oper_classes,
                 StaParams->supported_oper_classes_len);

    curr_peer->supported_oper_classes_len =
               StaParams->supported_oper_classes_len;

    curr_peer->qos = is_qos_wmm_sta;

    return 0;
}

int wlan_hdd_tdls_get_link_establish_params(hdd_adapter_t *pAdapter,
                                            const u8 *mac,
                                            tCsrTdlsLinkEstablishParams* tdlsLinkEstablishParams)
{
    hddTdlsPeer_t *curr_peer;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    tdlsLinkEstablishParams->isResponder = curr_peer->is_responder;
    tdlsLinkEstablishParams->uapsdQueues = curr_peer->uapsdQueues;
    tdlsLinkEstablishParams->maxSp = curr_peer->maxSp;
    tdlsLinkEstablishParams->isBufSta = curr_peer->isBufSta;
    tdlsLinkEstablishParams->isOffChannelSupported =
                                 curr_peer->isOffChannelSupported;

    vos_mem_copy(tdlsLinkEstablishParams->supportedChannels,
                 curr_peer->supported_channels,
                 curr_peer->supported_channels_len);

    tdlsLinkEstablishParams->supportedChannelsLen =
                 curr_peer->supported_channels_len;

    vos_mem_copy(tdlsLinkEstablishParams->supportedOperClasses,
                 curr_peer->supported_oper_classes,
                 curr_peer->supported_oper_classes_len);

    tdlsLinkEstablishParams->supportedOperClassesLen =
                 curr_peer->supported_oper_classes_len;

    tdlsLinkEstablishParams->qos = curr_peer->qos;

    return 0;
}

int wlan_hdd_tdls_set_rssi(hdd_adapter_t *pAdapter, const u8 *mac,
                           tANI_S8 rxRssi)
{
    hddTdlsPeer_t *curr_peer;

    curr_peer = wlan_hdd_tdls_find_peer(pAdapter, mac, TRUE);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    curr_peer->rssi = rxRssi;

    return 0;
}

int wlan_hdd_tdls_set_responder(hdd_adapter_t *pAdapter, const u8 *mac,
                                tANI_U8 responder)
{
    hddTdlsPeer_t *curr_peer;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    curr_peer->is_responder = responder;

    return 0;
}

int wlan_hdd_tdls_set_signature(hdd_adapter_t *pAdapter, const u8 *mac,
                                tANI_U8 uSignature)
{
    hddTdlsPeer_t *curr_peer;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    curr_peer->signature = uSignature;

    return 0;
}


void wlan_hdd_tdls_extract_da(struct sk_buff *skb, u8 *mac)
{
    memcpy(mac, skb->data, 6);
}

void wlan_hdd_tdls_extract_sa(struct sk_buff *skb, u8 *mac)
{
    memcpy(mac, skb->data+6, 6);
}

int wlan_hdd_tdls_increment_pkt_count(hdd_adapter_t *pAdapter, const u8 *mac,
                                      u8 tx)
{
    hddTdlsPeer_t *curr_peer;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if (eTDLS_SUPPORT_ENABLED != pHddCtx->tdls_mode &&
        eTDLS_SUPPORT_EXTERNAL_CONTROL != pHddCtx->tdls_mode)
        return -1;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL) {
        hddLog(LOG1, FL("curr_peer is NULL"));
        return -1;
    }

    if (tx)
        curr_peer->tx_pkt++;
    else
        curr_peer->rx_pkt++;

    return 0;
}

static int wlan_hdd_tdls_check_config(tdls_config_params_t *config)
{
    if (config->tdls > 2)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 1st argument %d. <0...2>", __func__, config->tdls);
        return -1;
    }
    if (config->tx_period_t < CFG_TDLS_TX_STATS_PERIOD_MIN ||
        config->tx_period_t > CFG_TDLS_TX_STATS_PERIOD_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 2nd argument %d. <%d...%ld>", __func__, config->tx_period_t,
            CFG_TDLS_TX_STATS_PERIOD_MIN, CFG_TDLS_TX_STATS_PERIOD_MAX);
        return -1;
    }
    if (config->tx_packet_n < CFG_TDLS_TX_PACKET_THRESHOLD_MIN ||
        config->tx_packet_n > CFG_TDLS_TX_PACKET_THRESHOLD_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 3rd argument %d. <%d...%ld>",               __func__,
               config->tx_packet_n,
               CFG_TDLS_TX_PACKET_THRESHOLD_MIN,
               CFG_TDLS_TX_PACKET_THRESHOLD_MAX);
        return -1;
    }
    if (config->discovery_period_t < CFG_TDLS_DISCOVERY_PERIOD_MIN ||
        config->discovery_period_t > CFG_TDLS_DISCOVERY_PERIOD_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 4th argument %d. <%d...%ld>",               __func__,
               config->discovery_period_t,
               CFG_TDLS_DISCOVERY_PERIOD_MIN,
               CFG_TDLS_DISCOVERY_PERIOD_MAX);
        return -1;
    }
    if (config->discovery_tries_n < CFG_TDLS_MAX_DISCOVERY_ATTEMPT_MIN ||
        config->discovery_tries_n > CFG_TDLS_MAX_DISCOVERY_ATTEMPT_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 5th argument %d. <%d...%d>",
               __func__,
               config->discovery_tries_n,
               CFG_TDLS_MAX_DISCOVERY_ATTEMPT_MIN,
               CFG_TDLS_MAX_DISCOVERY_ATTEMPT_MAX);
        return -1;
    }
    if (config->idle_timeout_t < CFG_TDLS_IDLE_TIMEOUT_MIN ||
        config->idle_timeout_t > CFG_TDLS_IDLE_TIMEOUT_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 6th argument %d. <%d...%d>",                __func__,
               config->idle_timeout_t,
               CFG_TDLS_IDLE_TIMEOUT_MIN,
               CFG_TDLS_IDLE_TIMEOUT_MAX);
        return -1;
    }
    if (config->idle_packet_n < CFG_TDLS_IDLE_PACKET_THRESHOLD_MIN ||
        config->idle_packet_n > CFG_TDLS_IDLE_PACKET_THRESHOLD_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 7th argument %d. <%d...%d>", __func__, config->idle_packet_n,
            CFG_TDLS_IDLE_PACKET_THRESHOLD_MIN, CFG_TDLS_IDLE_PACKET_THRESHOLD_MAX);
        return -1;
    }
    if (config->rssi_hysteresis < CFG_TDLS_RSSI_HYSTERESIS_MIN ||
        config->rssi_hysteresis > CFG_TDLS_RSSI_HYSTERESIS_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 8th argument %d. <%d...%d>", __func__, config->rssi_hysteresis,
            CFG_TDLS_RSSI_HYSTERESIS_MIN, CFG_TDLS_RSSI_HYSTERESIS_MAX);
        return -1;
    }
    if (config->rssi_trigger_threshold < CFG_TDLS_RSSI_TRIGGER_THRESHOLD_MIN ||
        config->rssi_trigger_threshold > CFG_TDLS_RSSI_TRIGGER_THRESHOLD_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 9th argument %d. <%d...%d>", __func__, config->rssi_trigger_threshold,
            CFG_TDLS_RSSI_TRIGGER_THRESHOLD_MIN, CFG_TDLS_RSSI_TRIGGER_THRESHOLD_MAX);
        return -1;
    }
    if (config->rssi_teardown_threshold < CFG_TDLS_RSSI_TEARDOWN_THRESHOLD_MIN ||
        config->rssi_teardown_threshold > CFG_TDLS_RSSI_TEARDOWN_THRESHOLD_MAX)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s invalid 10th argument %d. <%d...%d>", __func__, config->rssi_teardown_threshold,
            CFG_TDLS_RSSI_TEARDOWN_THRESHOLD_MIN, CFG_TDLS_RSSI_TEARDOWN_THRESHOLD_MAX);
        return -1;
    }
    return 0;
}

/* Caller has to take the lock before calling this function */
static void wlan_tdd_tdls_reset_tx_rx(tdlsCtx_t *pHddTdlsCtx)
{
    int i;
    struct list_head *head;
    hddTdlsPeer_t *tmp;
    struct list_head *pos, *q;

    for (i = 0; i < TDLS_PEER_LIST_SIZE; i++) {
        head = &pHddTdlsCtx->peer_list[i];
        list_for_each_safe (pos, q, head) {
            tmp = list_entry(pos, hddTdlsPeer_t, node);
            tmp->tx_pkt = 0;
            tmp->rx_pkt = 0;
        }
    }

    return;
}

static void wlan_hdd_tdls_implicit_disable(tdlsCtx_t *pHddTdlsCtx)
{
    wlan_hdd_tdls_timers_stop(pHddTdlsCtx);
}

static void wlan_hdd_tdls_implicit_enable(tdlsCtx_t *pHddTdlsCtx)
{
    wlan_hdd_tdls_peer_reset_discovery_processed(pHddTdlsCtx);
    pHddTdlsCtx->discovery_sent_cnt = 0;
    wlan_tdd_tdls_reset_tx_rx(pHddTdlsCtx);
    wlan_hdd_tdls_check_power_save_prohibited(pHddTdlsCtx->pAdapter);
}

static void wlan_hdd_tdls_set_mode(hdd_context_t *pHddCtx,
                            eTDLSSupportMode tdls_mode,
                            v_BOOL_t bUpdateLast)
{
    hdd_adapter_list_node_t *pAdapterNode = NULL, *pNext = NULL;
    VOS_STATUS status;
    hdd_adapter_t *pAdapter;
    tdlsCtx_t *pHddTdlsCtx;

    ENTER();

    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
               "%s mode %d", __func__, (int)tdls_mode);

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return;

    mutex_lock(&pHddCtx->tdls_lock);

    if (pHddCtx->tdls_mode == tdls_mode)
    {
        mutex_unlock(&pHddCtx->tdls_lock);
        hddLog(VOS_TRACE_LEVEL_INFO,
               "%s already in mode %d", __func__, (int)tdls_mode);
        return;
    }

    status = hdd_get_front_adapter ( pHddCtx, &pAdapterNode );

    while ( NULL != pAdapterNode && VOS_STATUS_SUCCESS == status )
    {
       pAdapter = pAdapterNode->pAdapter;
       pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
       if (NULL != pHddTdlsCtx)
       {
           if(eTDLS_SUPPORT_ENABLED == tdls_mode ||
              eTDLS_SUPPORT_EXTERNAL_CONTROL == tdls_mode)
               wlan_hdd_tdls_implicit_enable(pHddTdlsCtx);
           else if((eTDLS_SUPPORT_DISABLED == tdls_mode) ||
                   (eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY == tdls_mode))
               wlan_hdd_tdls_implicit_disable(pHddTdlsCtx);
       }
       status = hdd_get_next_adapter ( pHddCtx, pAdapterNode, &pNext );
       pAdapterNode = pNext;
    }
    if(bUpdateLast)
    {
        pHddCtx->tdls_mode_last = tdls_mode;
    }
    else
    {
        pHddCtx->tdls_mode_last = pHddCtx->tdls_mode;
    }
    pHddCtx->tdls_mode = tdls_mode;

    mutex_unlock(&pHddCtx->tdls_lock);
    EXIT();
}

int wlan_hdd_tdls_set_params(struct net_device *dev, tdls_config_params_t *config)
{
    hdd_adapter_t *pAdapter = WLAN_HDD_GET_PRIV_PTR(dev);
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX( pAdapter );
    tdlsCtx_t *pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    eTDLSSupportMode req_tdls_mode;
    tdlsInfo_t *tdlsParams;
    eHalStatus halStatus = eHAL_STATUS_FAILURE;

    if (NULL == pHddTdlsCtx)
    {
       hddLog(VOS_TRACE_LEVEL_ERROR, FL("TDLS not enabled!"));
       return -1;
    }

    if (wlan_hdd_tdls_check_config(config) != 0)
    {
        return -1;
    }

    /* config->tdls is mapped to 0->1, 1->2, 2->3 */
    req_tdls_mode = config->tdls + 1;
    if (pHddCtx->tdls_mode == req_tdls_mode)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR, "%s already in mode %d", __func__, config->tdls);
        return -1;
    }

    /* copy the configuration only when given tdls mode is implicit trigger enable */
    if (eTDLS_SUPPORT_ENABLED == req_tdls_mode ||
        eTDLS_SUPPORT_EXTERNAL_CONTROL == req_tdls_mode)
    {
        memcpy(&pHddTdlsCtx->threshold_config, config, sizeof(tdls_config_params_t));
    }

    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
            "iw set tdls params: %d %d %d %d %d %d %d %d %d %d",
            config->tdls,
            config->tx_period_t,
            config->tx_packet_n,
            config->discovery_period_t,
            config->discovery_tries_n,
            config->idle_timeout_t,
            config->idle_packet_n,
            config->rssi_hysteresis,
            config->rssi_trigger_threshold,
            config->rssi_teardown_threshold);

    wlan_hdd_tdls_set_mode(pHddCtx, req_tdls_mode, TRUE);

    tdlsParams = vos_mem_malloc(sizeof(tdlsInfo_t));
    if (NULL == tdlsParams)
    {
        hddLog(VOS_TRACE_LEVEL_ERROR,
               "%s: vos_mem_alloc failed for tdlsParams", __func__);
        return -1;
    }

    tdlsParams->vdev_id = pAdapter->sessionId;
    tdlsParams->tdls_state = config->tdls;
    tdlsParams->notification_interval_ms = config->tx_period_t;
    tdlsParams->tx_discovery_threshold = config->tx_packet_n;
    tdlsParams->tx_teardown_threshold = config->idle_packet_n;
    tdlsParams->rssi_teardown_threshold = config->rssi_teardown_threshold;
    tdlsParams->rssi_delta = config->rssi_delta;
    tdlsParams->tdls_options = 0;
    if (pHddCtx->cfg_ini->fEnableTDLSOffChannel)
        tdlsParams->tdls_options |= ENA_TDLS_OFFCHAN;
    if (pHddCtx->cfg_ini->fEnableTDLSBufferSta)
        tdlsParams->tdls_options |= ENA_TDLS_BUFFER_STA;
    if (pHddCtx->cfg_ini->fEnableTDLSSleepSta)
        tdlsParams->tdls_options |= ENA_TDLS_SLEEP_STA;
    tdlsParams->peer_traffic_ind_window =
        pHddCtx->cfg_ini->fTDLSPuapsdPTIWindow;
    tdlsParams->peer_traffic_response_timeout =
        pHddCtx->cfg_ini->fTDLSPuapsdPTRTimeout;
    tdlsParams->puapsd_mask =
        pHddCtx->cfg_ini->fTDLSUapsdMask;
    tdlsParams->puapsd_inactivity_time =
        pHddCtx->cfg_ini->fTDLSPuapsdInactivityTimer;
    tdlsParams->puapsd_rx_frame_threshold =
        pHddCtx->cfg_ini->fTDLSRxFrameThreshold;
    tdlsParams->teardown_notification_ms =
        pHddCtx->cfg_ini->fTDLSIdleTimeout;
    tdlsParams->tdls_peer_kickout_threshold =
        pHddCtx->cfg_ini->tdls_peer_kickout_threshold;


    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
              "%s: Setting tdls state and param in fw: "
              "vdev_id: %d, "
              "tdls_state: %d, "
              "notification_interval_ms: %d, "
              "tx_discovery_threshold: %d, "
              "tx_teardown_threshold: %d, "
              "rssi_teardown_threshold: %d, "
              "rssi_delta: %d, "
              "tdls_options: 0x%x, "
              "peer_traffic_ind_window: %d, "
              "peer_traffic_response_timeout: %d, "
              "puapsd_mask: 0x%x, "
              "puapsd_inactivity_time: %d, "
              "puapsd_rx_frame_threshold: %d, "
              "teardown_notification_ms: %d, "
              "tdls_peer_kickout_threshold: %d ",
              __func__,
              tdlsParams->vdev_id,
              tdlsParams->tdls_state,
              tdlsParams->notification_interval_ms,
              tdlsParams->tx_discovery_threshold,
              tdlsParams->tx_teardown_threshold,
              tdlsParams->rssi_teardown_threshold,
              tdlsParams->rssi_delta,
              tdlsParams->tdls_options,
              tdlsParams->peer_traffic_ind_window,
              tdlsParams->peer_traffic_response_timeout,
              tdlsParams->puapsd_mask,
              tdlsParams->puapsd_inactivity_time,
              tdlsParams->puapsd_rx_frame_threshold,
              tdlsParams->teardown_notification_ms,
              tdlsParams->tdls_peer_kickout_threshold);

    halStatus = sme_UpdateFwTdlsState(pHddCtx->hHal, tdlsParams, TRUE);
    if (eHAL_STATUS_SUCCESS != halStatus)
    {
        vos_mem_free(tdlsParams);
        return -1;
    }

    return 0;
}

/**
 * wlan_hdd_update_tdls_info - update tdls status info
 * @adapter: ptr to device adapter.
 * @tdls_prohibited: indicates whether tdls is prohibited.
 * @tdls_chan_swit_prohibited: indicates whether tdls channel switch
 *                             is prohibited.
 *
 * Normally an AP does not influence TDLS connection between STAs
 * associated to it. But AP may set bits for TDLS Prohibited or
 * TDLS Channel Switch Prohibited in Extended Capability IE in
 * Assoc/Re-assoc response to STA. So after STA is connected to
 * an AP, call this function to update TDLS status as per those
 * bits set in Ext Cap IE in received Assoc/Re-assoc response
 * from AP.
 *
 * Return: None.
 */
void wlan_hdd_update_tdls_info(hdd_adapter_t *adapter, bool tdls_prohibited,
                               bool tdls_chan_swit_prohibited)
{
    hdd_context_t *hdd_ctx = WLAN_HDD_GET_CTX(adapter);
    tdlsCtx_t *hdd_tdls_ctx = WLAN_HDD_GET_TDLS_CTX_PTR(adapter);
    tdlsInfo_t *tdls_param;
    eHalStatus hal_status;

    if (!hdd_tdls_ctx) {
        /* may be TDLS is not applicable for this adapter */
        hddLog(LOG1, FL("HDD TDLS context is null"));
        return;
    }

    /* If TDLS support is disabled then no need to update target */
    if (FALSE == hdd_ctx->cfg_ini->fEnableTDLSSupport) {
        hddLog(LOG1, FL("TDLS not enabled"));
        return;
    }

    /* If AP indicated TDLS Prohibited then disable tdls mode */
    mutex_lock(&hdd_ctx->tdls_lock);
    if (tdls_prohibited) {
        hdd_ctx->tdls_mode = eTDLS_SUPPORT_NOT_ENABLED;
    } else {
        if (FALSE == hdd_ctx->cfg_ini->fEnableTDLSImplicitTrigger) {
            hdd_ctx->tdls_mode = eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY;
        } else if (TRUE == hdd_ctx->cfg_ini->fTDLSExternalControl) {
            hdd_ctx->tdls_mode = eTDLS_SUPPORT_EXTERNAL_CONTROL;
        } else {
            hdd_ctx->tdls_mode = eTDLS_SUPPORT_ENABLED;
        }
    }
    mutex_unlock(&hdd_ctx->tdls_lock);

    tdls_param = vos_mem_malloc(sizeof(*tdls_param));
    if (!tdls_param) {
        hddLog(LOGE,
               FL("memory allocation failed for tdlsParams"));
        return;
    }

    tdls_param->vdev_id = adapter->sessionId;
    tdls_param->tdls_state = hdd_ctx->tdls_mode;
    tdls_param->notification_interval_ms =
        hdd_tdls_ctx->threshold_config.tx_period_t;
    tdls_param->tx_discovery_threshold =
        hdd_tdls_ctx->threshold_config.tx_packet_n;
    tdls_param->tx_teardown_threshold =
        hdd_tdls_ctx->threshold_config.idle_packet_n;
    tdls_param->rssi_teardown_threshold =
        hdd_tdls_ctx->threshold_config.rssi_teardown_threshold;
    tdls_param->rssi_delta = hdd_tdls_ctx->threshold_config.rssi_delta;

    tdls_param->tdls_options = 0;

    /* Do not enable TDLS offchannel, if AP prohibited TDLS channel switch */
    if ((hdd_ctx->cfg_ini->fEnableTDLSOffChannel) &&
        (!tdls_chan_swit_prohibited)) {
        tdls_param->tdls_options |= ENA_TDLS_OFFCHAN;
    }

    if (hdd_ctx->cfg_ini->fEnableTDLSBufferSta)
        tdls_param->tdls_options |= ENA_TDLS_BUFFER_STA;

    if (hdd_ctx->cfg_ini->fEnableTDLSSleepSta)
        tdls_param->tdls_options |= ENA_TDLS_SLEEP_STA;

    tdls_param->peer_traffic_ind_window =
        hdd_ctx->cfg_ini->fTDLSPuapsdPTIWindow;
    tdls_param->peer_traffic_response_timeout =
        hdd_ctx->cfg_ini->fTDLSPuapsdPTRTimeout;
    tdls_param->puapsd_mask =
        hdd_ctx->cfg_ini->fTDLSUapsdMask;
    tdls_param->puapsd_inactivity_time =
        hdd_ctx->cfg_ini->fTDLSPuapsdInactivityTimer;
    tdls_param->puapsd_rx_frame_threshold =
        hdd_ctx->cfg_ini->fTDLSRxFrameThreshold;
    tdls_param->teardown_notification_ms =
        hdd_ctx->cfg_ini->fTDLSIdleTimeout;
    tdls_param->tdls_peer_kickout_threshold =
        hdd_ctx->cfg_ini->tdls_peer_kickout_threshold;


    hddLog(LOG1,
           FL("Setting tdls state and param in fw: "
              "vdev_id: %d, "
              "tdls_state: %d, "
              "notification_interval_ms: %d, "
              "tx_discovery_threshold: %d, "
              "tx_teardown_threshold: %d, "
              "rssi_teardown_threshold: %d, "
              "rssi_delta: %d, "
              "tdls_options: 0x%x, "
              "peer_traffic_ind_window: %d, "
              "peer_traffic_response_timeout: %d, "
              "puapsd_mask: 0x%x, "
              "puapsd_inactivity_time: %d, "
              "puapsd_rx_frame_threshold: %d, "
              "teardown_notification_ms: %d, "
              "tdls_peer_kickout_threshold: %d "),
              tdls_param->vdev_id,
              tdls_param->tdls_state,
              tdls_param->notification_interval_ms,
              tdls_param->tx_discovery_threshold,
              tdls_param->tx_teardown_threshold,
              tdls_param->rssi_teardown_threshold,
              tdls_param->rssi_delta,
              tdls_param->tdls_options,
              tdls_param->peer_traffic_ind_window,
              tdls_param->peer_traffic_response_timeout,
              tdls_param->puapsd_mask,
              tdls_param->puapsd_inactivity_time,
              tdls_param->puapsd_rx_frame_threshold,
              tdls_param->teardown_notification_ms,
              tdls_param->tdls_peer_kickout_threshold);

    hal_status = sme_UpdateFwTdlsState(hdd_ctx->hHal, tdls_param, TRUE);
    if (eHAL_STATUS_SUCCESS != hal_status) {
        vos_mem_free(tdls_param);
        return;
    }

    return;
}

int wlan_hdd_tdls_set_sta_id(hdd_adapter_t *pAdapter, const u8 *mac, u8 staId)
{
    hddTdlsPeer_t *curr_peer;

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    curr_peer->staId = staId;

    return 0;
}

int wlan_hdd_tdls_set_extctrl_param(hdd_adapter_t *pAdapter, const uint8_t *mac,
                                    uint32_t chan, uint32_t max_latency,
                                    uint32_t op_class, uint32_t min_bandwidth)
{
    hddTdlsPeer_t *curr_peer;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if (!pHddCtx)
        return -1;

    mutex_lock(&pHddCtx->tdls_lock);

    curr_peer = wlan_hdd_tdls_find_peer(pAdapter, mac, FALSE);
    if (curr_peer == NULL)
    {
        mutex_unlock(&pHddCtx->tdls_lock);
        return -1;
    }

    curr_peer->op_class_for_pref_off_chan = (uint8_t)op_class;
    curr_peer->pref_off_chan_num = (uint8_t)chan;

    mutex_unlock(&pHddCtx->tdls_lock);
    return 0;
}

/**
 * wlan_hdd_tdls_update_peer_mac() - Update the peer mac information to firmware
 * @pAdapter: hdd adapter to interface
 * @mac: Mac address of the peer to be added
 * @peerState: Current state of the peer
 *
 * This function updates TDLS peer state to firmware. Firmware will update
 * connection table based on new peer state.
 *
 * Return:success (0) or failure (errno value)
 */
int wlan_hdd_tdls_update_peer_mac(hdd_adapter_t *pAdapter,
	const uint8_t *mac,
	uint32_t peerState)
{
	tSmeTdlsPeerStateParams sme_tdls_peer_state_params;
	eHalStatus hal_status = eHAL_STATUS_FAILURE;
	hdd_context_t *hdd_ctx = WLAN_HDD_GET_CTX(pAdapter);

	vos_mem_zero(&sme_tdls_peer_state_params,
		sizeof(sme_tdls_peer_state_params));
	sme_tdls_peer_state_params.vdevId = pAdapter->sessionId;
	vos_mem_copy(&sme_tdls_peer_state_params.peerMacAddr,
		mac, sizeof(sme_tdls_peer_state_params.peerMacAddr));
	sme_tdls_peer_state_params.peerState = peerState;
	hal_status = sme_UpdateTdlsPeerState(hdd_ctx->hHal,
		&sme_tdls_peer_state_params);
	if (eHAL_STATUS_SUCCESS != hal_status) {
		hddLog(LOGE, FL("sme_UpdateTdlsPeerState failed for "
		MAC_ADDRESS_STR), MAC_ADDR_ARRAY(mac));
		return -EPERM;
	}
	return 0;
}

int wlan_hdd_tdls_set_force_peer(hdd_adapter_t *pAdapter, const u8 *mac,
                                 tANI_BOOLEAN forcePeer)
{
    hddTdlsPeer_t *curr_peer;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if (!pHddCtx)
        return -1;

    mutex_lock(&pHddCtx->tdls_lock);

    curr_peer = wlan_hdd_tdls_find_peer(pAdapter, mac, FALSE);
    if (curr_peer == NULL)
    {
        mutex_unlock(&pHddCtx->tdls_lock);
        return -1;
    }

    curr_peer->isForcedPeer = forcePeer;
    mutex_unlock(&pHddCtx->tdls_lock);
    return 0;
}

/* if peerMac is found, then it returns pointer to hddTdlsPeer_t
 *   otherwise, it returns NULL
 */
hddTdlsPeer_t *wlan_hdd_tdls_find_peer(hdd_adapter_t *pAdapter,
                                       const u8 *mac,
                                       tANI_BOOLEAN mutexLock)
{
    u8 key;
    struct list_head *pos;
    struct list_head *head;
    hddTdlsPeer_t *curr_peer;
    tdlsCtx_t *pHddTdlsCtx;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    ENTER();

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return NULL;

    if ( mutexLock )
    {
       mutex_lock(&pHddCtx->tdls_lock);
    }
    pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    if (NULL == pHddTdlsCtx)
    {
        if ( mutexLock )
            mutex_unlock(&pHddCtx->tdls_lock);
        return NULL;
    }

    key = wlan_hdd_tdls_hash_key(mac);

    head = &pHddTdlsCtx->peer_list[key];

    list_for_each(pos, head) {
        curr_peer = list_entry (pos, hddTdlsPeer_t, node);
        if (!memcmp(mac, curr_peer->peerMac, 6)) {
            VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                     "findTdlsPeer: found staId %d", curr_peer->staId);
            if ( mutexLock )
                mutex_unlock(&pHddCtx->tdls_lock);
            return curr_peer;
        }
    }
    if ( mutexLock )
        mutex_unlock(&pHddCtx->tdls_lock);
    EXIT();
    return NULL;
}

hddTdlsPeer_t *wlan_hdd_tdls_find_all_peer(hdd_context_t *pHddCtx,
                                           const u8 *mac)
{
    hdd_adapter_list_node_t *pAdapterNode = NULL, *pNext = NULL;
    hdd_adapter_t *pAdapter = NULL;
    tdlsCtx_t *pHddTdlsCtx = NULL;
    hddTdlsPeer_t *curr_peer= NULL;
    VOS_STATUS status = 0;

    mutex_lock(&pHddCtx->tdls_lock);

    status = hdd_get_front_adapter ( pHddCtx, &pAdapterNode );
    while ( NULL != pAdapterNode && VOS_STATUS_SUCCESS == status )
    {
        pAdapter = pAdapterNode->pAdapter;

        pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
        if (NULL != pHddTdlsCtx)
        {
            curr_peer = wlan_hdd_tdls_find_peer(pAdapter, mac, FALSE);
            if (curr_peer)
            {
                mutex_unlock(&pHddCtx->tdls_lock);
                return curr_peer;
            }
        }
        status = hdd_get_next_adapter ( pHddCtx, pAdapterNode, &pNext );
        pAdapterNode = pNext;
    }
    mutex_unlock(&pHddCtx->tdls_lock);
    return curr_peer;
}


int wlan_hdd_tdls_reset_peer(hdd_adapter_t *pAdapter, const u8 *mac)
{
    hdd_context_t *pHddCtx;
    hddTdlsPeer_t *curr_peer;

    pHddCtx = WLAN_HDD_GET_CTX( pAdapter );

    curr_peer = wlan_hdd_tdls_get_peer(pAdapter, mac);
    if (curr_peer == NULL)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
       return -1;
    }

    /*
     * Reset preferred offchannel and opclass for offchannel as
     * per INI configuration only if peer is not forced one. For
     * forced peer, offchannel and opclass is set in HAL API at the
     * time of enabling TDLS for that specific peer and so do not overwrite
     * those set by user space.
     */
    if (FALSE == curr_peer->isForcedPeer) {
        curr_peer->pref_off_chan_num = pHddCtx->cfg_ini->fTDLSPrefOffChanNum;
        curr_peer->op_class_for_pref_off_chan =
            wlan_hdd_find_opclass(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                  curr_peer->pref_off_chan_num,
                                  pHddCtx->cfg_ini->fTDLSPrefOffChanBandwidth);
    }

    wlan_hdd_tdls_set_peer_link_status(curr_peer,
                                       eTDLS_LINK_IDLE,
                                       eTDLS_LINK_UNSPECIFIED);
    curr_peer->staId = 0;

    return 0;
}

tANI_U16 wlan_hdd_tdlsConnectedPeers(hdd_adapter_t *pAdapter)
{
    hdd_context_t *pHddCtx = NULL;

   if ((NULL == pAdapter) || (WLAN_HDD_ADAPTER_MAGIC != pAdapter->magic)) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("invalid pAdapter: %p"), pAdapter);
        return 0;
    }
    pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    ENTER();

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return 0;

    EXIT();
    return pHddCtx->connected_peer_count;
}

int wlan_hdd_tdls_get_all_peers(hdd_adapter_t *pAdapter, char *buf, int buflen)
{
    int i;
    int len, init_len;
    struct list_head *head;
    struct list_head *pos;
    hddTdlsPeer_t *curr_peer;
    tdlsCtx_t *pHddTdlsCtx;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    ENTER();

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return 0;

    init_len = buflen;
    len = scnprintf(buf, buflen, "\n%-18s%-3s%-4s%-3s%-5s\n",
            "MAC", "Id", "cap", "up", "RSSI");
    buf += len;
    buflen -= len;
    /*                           1234567890123456789012345678901234567 */
    len = scnprintf(buf, buflen, "---------------------------------\n");
    buf += len;
    buflen -= len;

    mutex_lock(&pHddCtx->tdls_lock);

    pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    if (NULL == pHddTdlsCtx) {
        mutex_unlock(&pHddCtx->tdls_lock);
        len = scnprintf(buf, buflen, "TDLS not enabled\n");
        return len;
    }
    for (i = 0; i < TDLS_PEER_LIST_SIZE; i++) {
        head = &pHddTdlsCtx->peer_list[i];

        list_for_each(pos, head) {
            curr_peer= list_entry (pos, hddTdlsPeer_t, node);

            if (buflen < 32+1)
                break;
            len = scnprintf(buf, buflen,
                MAC_ADDRESS_STR"%3d%4s%3s%5d\n",
                MAC_ADDR_ARRAY(curr_peer->peerMac),
                curr_peer->staId,
                (curr_peer->tdls_support == eTDLS_CAP_SUPPORTED) ? "Y":"N",
                TDLS_IS_CONNECTED(curr_peer) ? "Y":"N",
                curr_peer->rssi);
            buf += len;
            buflen -= len;
        }
    }
    mutex_unlock(&pHddCtx->tdls_lock);
    EXIT();
    return init_len-buflen;
}

void wlan_hdd_tdls_connection_callback(hdd_adapter_t *pAdapter)
{
    tdlsCtx_t *pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if ((NULL == pHddCtx) || (NULL == pHddTdlsCtx))
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_WARN,
                 FL("pHddCtx or pHddTdlsCtx points to NULL"));
       return;
    }

    mutex_lock(&pHddCtx->tdls_lock);

    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
    "%s, update %d discover %d", __func__,
        pHddTdlsCtx->threshold_config.tx_period_t,
        pHddTdlsCtx->threshold_config.discovery_period_t);

    if (eTDLS_SUPPORT_ENABLED == pHddCtx->tdls_mode ||
        eTDLS_SUPPORT_EXTERNAL_CONTROL == pHddCtx->tdls_mode) {
       wlan_hdd_tdls_peer_reset_discovery_processed(pHddTdlsCtx);
       pHddTdlsCtx->discovery_sent_cnt = 0;
       wlan_hdd_tdls_check_power_save_prohibited(pHddTdlsCtx->pAdapter);

    }
    mutex_unlock(&pHddCtx->tdls_lock);

}

void wlan_hdd_tdls_disconnection_callback(hdd_adapter_t *pAdapter)
{
    tdlsCtx_t *pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if ((NULL == pHddCtx) || (NULL == pHddTdlsCtx))
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                FL("pHddCtx or pHddTdlsCtx points to NULL"));
       return;
    }

    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,"%s", __func__);

    mutex_lock(&pHddCtx->tdls_lock);

    if (NULL == pHddTdlsCtx)
    {
       mutex_unlock(&pHddCtx->tdls_lock);
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                FL("pHddTdlsCtx is NULL"));
        return;
    }
    pHddTdlsCtx->discovery_sent_cnt = 0;
    wlan_hdd_tdls_check_power_save_prohibited(pHddTdlsCtx->pAdapter);

    wlan_hdd_tdls_monitor_timers_stop(pHddTdlsCtx);
    wlan_hdd_tdls_free_list(pHddTdlsCtx, false);

    pHddTdlsCtx->curr_candidate = NULL;

    mutex_unlock(&pHddCtx->tdls_lock);
}

void wlan_hdd_tdls_mgmt_completion_callback(hdd_adapter_t *pAdapter, tANI_U32 statusCode)
{
    pAdapter->mgmtTxCompletionStatus = statusCode;
    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
               "%s: Mgmt TX Completion %d",
               __func__, statusCode);
    complete(&pAdapter->tdls_mgmt_comp);
}

void wlan_hdd_tdls_increment_peer_count(hdd_adapter_t *pAdapter)
{
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    ENTER();

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return;

    mutex_lock(&pHddCtx->tdls_lock);

    pHddCtx->connected_peer_count++;
    wlan_hdd_tdls_check_power_save_prohibited(pAdapter);

    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO, "%s: %d",
               __func__, pHddCtx->connected_peer_count);

    mutex_unlock(&pHddCtx->tdls_lock);
    EXIT();
}

void wlan_hdd_tdls_decrement_peer_count(hdd_adapter_t *pAdapter)
{
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    ENTER();

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return;

    mutex_lock(&pHddCtx->tdls_lock);

    if (pHddCtx->connected_peer_count)
        pHddCtx->connected_peer_count--;
    wlan_hdd_tdls_check_power_save_prohibited(pAdapter);

    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO, "%s: %d",
               __func__, pHddCtx->connected_peer_count);

    mutex_unlock(&pHddCtx->tdls_lock);
    EXIT();
}

void wlan_hdd_tdls_check_bmps(hdd_adapter_t *pAdapter)
{
    tdlsCtx_t *pHddTdlsCtx = NULL;
    hdd_context_t *pHddCtx = NULL;
    hddTdlsPeer_t *curr_peer;

    if ((NULL == pAdapter) || (WLAN_HDD_ADAPTER_MAGIC != pAdapter->magic)) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("invalid pAdapter: %p"), pAdapter);
        return;
    }
    pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if ((NULL == pHddCtx) || (NULL == pHddTdlsCtx))
    {
       //getting over logged, so moving log-level to INFO.
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                 FL("pHddCtx or pHddTdlsCtx points to NULL"));
       return;
    }

    curr_peer = wlan_hdd_tdls_is_progress(pHddCtx, NULL, 0);
    if (NULL != curr_peer)
    {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                  "%s: tdls in progress. Dont check for BMPS " MAC_ADDRESS_STR,
                __func__, MAC_ADDR_ARRAY (curr_peer->peerMac));
        return;
    }
    /*
     * If Powersave Offload is enabled
     * Fw will take care incase of concurrency
     */
    if(!pHddCtx->cfg_ini->enablePowersaveOffload)
    {
        if ((TDLS_CTX_MAGIC != pHddCtx->tdls_scan_ctxt.magic) &&
            (0 == pHddCtx->connected_peer_count) &&
            (0 == pHddTdlsCtx->discovery_sent_cnt))
        {
            if (FALSE == sme_IsPmcBmps(WLAN_HDD_GET_HAL_CTX(pAdapter)))
            {
                VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_WARN,
                     "%s: No TDLS peer connected/discovery sent. Enable BMPS",
                      __func__);
                hdd_enable_bmps_imps(pHddCtx);
            }
        }
        else
        {
            if (TRUE == sme_IsPmcBmps(WLAN_HDD_GET_HAL_CTX(pAdapter)))
            {
                VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                           "%s: TDLS peer connected. Disable BMPS", __func__);
                hdd_disable_bmps_imps(pHddCtx, WLAN_HDD_INFRA_STATION);
            }
        }
    }
    else
    {
        if ((TDLS_CTX_MAGIC != pHddCtx->tdls_scan_ctxt.magic) &&
            (0 == pHddCtx->connected_peer_count) &&
            (0 == pHddTdlsCtx->discovery_sent_cnt))
        {
            VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                      "%s: No TDLS peer connected/discovery sent. Enable BMPS",
                      __func__);
            sme_SetTdlsPowerSaveProhibited(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                           pAdapter->sessionId, FALSE);
            sme_PsOffloadEnablePowerSave(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                        pAdapter->sessionId);
        }
        else
        {
            VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                      "%s: TDLS peer connected. Disable BMPS", __func__);
            sme_SetTdlsPowerSaveProhibited(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                           pAdapter->sessionId, TRUE);
            sme_PsOffloadDisablePowerSave(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                        pAdapter->sessionId);

        }
    }
    return;
}

/* return pointer to hddTdlsPeer_t if TDLS is ongoing. Otherwise return NULL.
 * mac - if NULL check for all the peer list, otherwise, skip this mac when skip_self is TRUE
 * skip_self - if TRUE, skip this mac. otherwise, check all the peer list. if
   mac is NULL, this argument is ignored, and check for all the peer list.
 */
static hddTdlsPeer_t *wlan_hdd_tdls_find_progress_peer(hdd_adapter_t *pAdapter,
                                                       const u8 *mac,
                                                       u8 skip_self)
{
    int i;
    struct list_head *head;
    hddTdlsPeer_t *curr_peer;
    struct list_head *pos;
    tdlsCtx_t *pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);;

    if (NULL == pHddTdlsCtx)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                FL("pHddTdlsCtx is NULL"));
       return NULL;
     }

    for (i = 0; i < TDLS_PEER_LIST_SIZE; i++) {
        head = &pHddTdlsCtx->peer_list[i];
        list_for_each(pos, head) {
            curr_peer = list_entry (pos, hddTdlsPeer_t, node);
            if (skip_self && mac && !memcmp(mac, curr_peer->peerMac, 6)) {
                continue;
            }
            else
            {
                if (eTDLS_LINK_CONNECTING == curr_peer->link_status)
                {
                  VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                            "%s:" MAC_ADDRESS_STR " eTDLS_LINK_CONNECTING",
                            __func__, MAC_ADDR_ARRAY(curr_peer->peerMac));
                  return curr_peer;
                }
            }
        }
    }
    return NULL;
}

hddTdlsPeer_t *wlan_hdd_tdls_is_progress(hdd_context_t *pHddCtx,
                                         const u8 *mac, u8 skip_self)
{
    hdd_adapter_list_node_t *pAdapterNode = NULL, *pNext = NULL;
    hdd_adapter_t *pAdapter = NULL;
    tdlsCtx_t *pHddTdlsCtx = NULL;
    hddTdlsPeer_t *curr_peer= NULL;
    VOS_STATUS status = 0;

    mutex_lock(&pHddCtx->tdls_lock);

    status = hdd_get_front_adapter ( pHddCtx, &pAdapterNode );
    while ( NULL != pAdapterNode && VOS_STATUS_SUCCESS == status )
    {
        pAdapter = pAdapterNode->pAdapter;

        pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
        if (NULL != pHddTdlsCtx)
        {
            curr_peer = wlan_hdd_tdls_find_progress_peer(pAdapter, mac, skip_self);
            if (curr_peer)
            {
                mutex_unlock(&pHddCtx->tdls_lock);
                return curr_peer;
            }
        }
        status = hdd_get_next_adapter ( pHddCtx, pAdapterNode, &pNext );
        pAdapterNode = pNext;
    }
    mutex_unlock(&pHddCtx->tdls_lock);
    return NULL;
}

void wlan_hdd_tdls_implicit_send_discovery_request(tdlsCtx_t * pHddTdlsCtx)
{
    hdd_context_t *pHddCtx;
    hddTdlsPeer_t *curr_peer;
    hddTdlsPeer_t *temp_peer;
    tSirMacAddr peer_mac;

    ENTER();

    if (NULL == pHddTdlsCtx)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("pHddTdlsCtx is NULL"));
       return;
    }

    pHddCtx = WLAN_HDD_GET_CTX(pHddTdlsCtx->pAdapter);

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("pHddCtx is not valid"));
       return;
    }

    mutex_lock(&pHddCtx->tdls_lock);

    curr_peer = pHddTdlsCtx->curr_candidate;

    if (NULL == curr_peer)
    {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                FL("pHddCtx is not valid"));
       goto done;
    }

    vos_mem_copy(&peer_mac, curr_peer->peerMac, sizeof(peer_mac));

    mutex_unlock(&pHddCtx->tdls_lock);

    /*
     * If Powersave Offload is enabled
     * Fw will take care incase of concurrency
     */
    if(!pHddCtx->cfg_ini->enablePowersaveOffload)
    {
        if (TRUE == sme_IsPmcBmps(WLAN_HDD_GET_HAL_CTX(pHddTdlsCtx->pAdapter)))
        {
            VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                           "%s: Disable BMPS", __func__);
            hdd_disable_bmps_imps(pHddCtx, WLAN_HDD_INFRA_STATION);
        }
    }

    temp_peer = wlan_hdd_tdls_is_progress(pHddCtx, NULL, 0);

    if (NULL != temp_peer)
    {
        VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO, "%s: " MAC_ADDRESS_STR " ongoing. pre_setup ignored",
            __func__, MAC_ADDR_ARRAY(temp_peer->peerMac));
        goto done;
    }

    if (eTDLS_CAP_UNKNOWN != curr_peer->tdls_support)
        wlan_hdd_tdls_set_peer_link_status(curr_peer,
                                           eTDLS_LINK_DISCOVERING,
                                           eTDLS_LINK_SUCCESS);

    mutex_lock(&pHddCtx->tdls_lock);

    /* Ignore discovery attempt if External Control is enabled, that
     * is, peer is forced. In that case, continue discovery attempt
     * regardless attempt count
     */
    if (FALSE == curr_peer->isForcedPeer)
    {
        if (curr_peer->discovery_attempt >=
            pHddTdlsCtx->threshold_config.discovery_tries_n)
        {
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                      "%s: discovery attempt (%d) reached max (%d) for peer "
                      MAC_ADDRESS_STR ", ignore discovery trigger from fw",
                      __func__, curr_peer->discovery_attempt,
                      pHddTdlsCtx->threshold_config.discovery_tries_n,
                      MAC_ADDR_ARRAY(curr_peer->peerMac));
            curr_peer->tdls_support = eTDLS_CAP_NOT_SUPPORTED;
            /* Since TDLS discovery attempt reached the
             * maximum threshold, so we remove the peer
             * from the FW connection table.
             */
            if (0 != wlan_hdd_tdls_update_peer_mac(pHddTdlsCtx->pAdapter,
                curr_peer->peerMac, eSME_TDLS_PEER_REMOVE_MAC_ADDR))
                hddLog(LOGE, FL("TDLS Peer mac update Failed "
                       MAC_ADDRESS_STR),
                       MAC_ADDR_ARRAY(curr_peer->peerMac));
            goto done;
        }
    }

    mutex_unlock(&pHddCtx->tdls_lock);

    wlan_hdd_tdls_set_peer_link_status(curr_peer,
                                       eTDLS_LINK_DISCOVERING,
                                       eTDLS_LINK_SUCCESS);

    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
              "%s: Implicit TDLS, Send Discovery request event", __func__);

    cfg80211_tdls_oper_request(pHddTdlsCtx->pAdapter->dev,
                               curr_peer->peerMac,
                               NL80211_TDLS_DISCOVERY_REQ,
                               FALSE,
                               GFP_KERNEL);

    mutex_lock(&pHddCtx->tdls_lock);

    pHddTdlsCtx->discovery_sent_cnt++;

    curr_peer->discovery_attempt++;

    wlan_hdd_tdls_check_power_save_prohibited(pHddTdlsCtx->pAdapter);

    VOS_TRACE( VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
               "%s: discovery count %u timeout %u msec",
               __func__, pHddTdlsCtx->discovery_sent_cnt,
               pHddTdlsCtx->threshold_config.tx_period_t - TDLS_DISCOVERY_TIMEOUT_BEFORE_UPDATE);

    wlan_hdd_tdls_timer_restart(pHddTdlsCtx->pAdapter,
                                &pHddTdlsCtx->peerDiscoveryTimeoutTimer,
                                pHddTdlsCtx->threshold_config.tx_period_t - TDLS_DISCOVERY_TIMEOUT_BEFORE_UPDATE);

done:
    pHddTdlsCtx->curr_candidate = NULL;
    pHddTdlsCtx->magic = 0;
    mutex_unlock(&pHddCtx->tdls_lock);
    EXIT();
    return;
}

int wlan_hdd_tdls_copy_scan_context(hdd_context_t *pHddCtx,
                            struct wiphy *wiphy,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
                            struct net_device *dev,
#endif
                            struct cfg80211_scan_request *request)
{
    tdls_scan_context_t *scan_ctx;

    ENTER();

    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return -1;

    scan_ctx = &pHddCtx->tdls_scan_ctxt;

    scan_ctx->wiphy = wiphy;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
    scan_ctx->dev = dev;
#endif

    scan_ctx->scan_request = request;
    EXIT();
    return 0;
}

static void wlan_hdd_tdls_scan_init_work(hdd_context_t *pHddCtx,
                                struct wiphy *wiphy,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
                                struct net_device *dev,
#endif
                                struct cfg80211_scan_request *request,
                                unsigned long delay)
{
    if (TDLS_CTX_MAGIC != pHddCtx->tdls_scan_ctxt.magic)
    {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
        wlan_hdd_tdls_copy_scan_context(pHddCtx, wiphy, dev, request);
#else
        wlan_hdd_tdls_copy_scan_context(pHddCtx, wiphy, request);
#endif
        pHddCtx->tdls_scan_ctxt.attempt = 0;
        pHddCtx->tdls_scan_ctxt.magic = TDLS_CTX_MAGIC;
    }
    schedule_delayed_work(&pHddCtx->tdls_scan_ctxt.tdls_scan_work, delay);
}

/* return negative = caller should stop and return error code immediately
   return 0 = caller should stop and return success immediately
   return 1 = caller can continue to scan
 */
int wlan_hdd_tdls_scan_callback (hdd_adapter_t *pAdapter,
                                struct wiphy *wiphy,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
                                struct net_device *dev,
#endif
                                struct cfg80211_scan_request *request)
{
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);
    u16 connectedTdlsPeers;
    hddTdlsPeer_t *curr_peer;
    unsigned long delay;
    int ret;

    ENTER();
    ret = wlan_hdd_validate_context(pHddCtx);
    if (ret)
       return ret;

    /* if tdls is not enabled, then continue scan */
    if (eTDLS_SUPPORT_NOT_ENABLED == pHddCtx->tdls_mode)
        return 1;

    curr_peer = wlan_hdd_tdls_is_progress(pHddCtx, NULL, 0);
    if (NULL != curr_peer)
    {
        if (pHddCtx->tdls_scan_ctxt.reject++ >= TDLS_MAX_SCAN_REJECT)
        {
            pHddCtx->tdls_scan_ctxt.reject = 0;
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                    "%s: " MAC_ADDRESS_STR ". scan rejected %d. force it to idle",
                    __func__, MAC_ADDR_ARRAY (curr_peer->peerMac), pHddCtx->tdls_scan_ctxt.reject);

            wlan_hdd_tdls_set_peer_link_status (curr_peer,
                                                eTDLS_LINK_IDLE,
                                                eTDLS_LINK_UNSPECIFIED);
            return 1;
        }
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_WARN,
                "%s: tdls in progress. scan rejected %d",
                __func__, pHddCtx->tdls_scan_ctxt.reject);
        return -EBUSY;
    }

    /* tdls teardown is ongoing */
    if (eTDLS_SUPPORT_DISABLED == pHddCtx->tdls_mode)
    {
        connectedTdlsPeers = wlan_hdd_tdlsConnectedPeers(pAdapter);
        if (connectedTdlsPeers && (pHddCtx->tdls_scan_ctxt.attempt < TDLS_MAX_SCAN_SCHEDULE))
        {
            delay = (unsigned long)(TDLS_DELAY_SCAN_PER_CONNECTION*connectedTdlsPeers);
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                    "%s: tdls disabled, but still connected_peers %d attempt %d. schedule scan %lu msec",
                    __func__, connectedTdlsPeers, pHddCtx->tdls_scan_ctxt.attempt, delay);

            wlan_hdd_tdls_scan_init_work (pHddCtx, wiphy,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
                                          dev,
#endif
                                          request,
                                          msecs_to_jiffies(delay));
            /* scan should not continue */
            return 0;
        }
        /* no connected peer or max retry reached, scan continue */
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                "%s: tdls disabled. connected_peers %d attempt %d. scan allowed",
                __func__, connectedTdlsPeers, pHddCtx->tdls_scan_ctxt.attempt);
        return 1;
    }
    /* while tdls is up, first time scan */
    else if (eTDLS_SUPPORT_ENABLED == pHddCtx->tdls_mode ||
        eTDLS_SUPPORT_EXTERNAL_CONTROL == pHddCtx->tdls_mode ||
        eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY == pHddCtx->tdls_mode)
    {
        /* Disable implicit trigger logic & tdls operation */
        wlan_hdd_tdls_set_mode(pHddCtx, eTDLS_SUPPORT_DISABLED, FALSE);
        /* indicate the teardown all connected to peer */
        connectedTdlsPeers = wlan_hdd_tdlsConnectedPeers(pAdapter);
        if (connectedTdlsPeers)
        {
            tANI_U8 staIdx;
            tANI_U8 num = 0;
            tANI_U8 i;
            tANI_BOOLEAN allPeersBufStas = 1;
            hddTdlsPeer_t *curr_peer;
            hddTdlsPeer_t *connectedPeerList[HDD_MAX_NUM_TDLS_STA];

            /* If TDLSScan is enabled then allow scan and maintain tdls link
             * regardless if peer is buffer sta capable or not and if device
             * is sleep sta capable or not. If peer is not buffer sta capable,
             * then Tx would stop when device initiates scan and there will be
             * loss of Rx packets since peer would not know when device moves
             * away from the tdls channel.
             */
            if (1 == pHddCtx->cfg_ini->enable_tdls_scan) {
                hddLog(LOG1,
                       FL("TDLSScan enabled, keep tdls link and allow scan, connectedTdlsPeers: %d"),
                       connectedTdlsPeers);
                return 1;
            }

            for (staIdx = 0; staIdx < pHddCtx->max_num_tdls_sta; staIdx++)
            {
                if (pHddCtx->tdlsConnInfo[staIdx].staId)
                {
                    curr_peer = wlan_hdd_tdls_find_all_peer(pHddCtx,
                                  pHddCtx->tdlsConnInfo[staIdx].peerMac.bytes);
                    if (curr_peer)
                    {
                        connectedPeerList[num++] = curr_peer;
                        if (!(curr_peer->isBufSta))
                            allPeersBufStas = 0;
                    }
                }
            }

            if ((TDLS_MAX_CONNECTED_PEERS_TO_ALLOW_SCAN ==
                 connectedTdlsPeers) &&
                (pHddCtx->cfg_ini->fEnableTDLSSleepSta) &&
                (allPeersBufStas))
            {
                /* All connected peers bufStas and we can be sleepSta
                 * so allow scan
                 */
                VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                          "%s: All peers (num %d) bufSTAs, we can be sleep sta, so allow scan, tdls mode changed to %d",
                          __func__, connectedTdlsPeers, pHddCtx->tdls_mode);
                return 1;
            }
            else
            {
                for (i = 0; i < num; i++)
                {
                    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                              "%s: indicate TDLS teadown (staId %d)",
                              __func__, connectedPeerList[i]->staId);
#ifdef CONFIG_TDLS_IMPLICIT
                    wlan_hdd_tdls_indicate_teardown(
                            connectedPeerList[i]->pHddTdlsCtx->pAdapter,
                            connectedPeerList[i],
                            eSIR_MAC_TDLS_TEARDOWN_UNSPEC_REASON);
#endif
                }
            }
            /* schedule scan */
            delay = (unsigned long)(TDLS_DELAY_SCAN_PER_CONNECTION*connectedTdlsPeers);

            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                    "%s: tdls enabled (mode %d), connected_peers %d. schedule scan %lu msec",
                    __func__, pHddCtx->tdls_mode, wlan_hdd_tdlsConnectedPeers(pAdapter),
                    delay);

            wlan_hdd_tdls_scan_init_work (pHddCtx, wiphy,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)) && !defined(WITH_BACKPORTS)
                                          dev,
#endif
                                          request,
                                          msecs_to_jiffies(delay));
            /* scan should not continue */
            return 0;
        }
        /* no connected peer, scan continue */
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                "%s: tdls_mode %d, and no tdls connection. scan allowed",
                 __func__, pHddCtx->tdls_mode);
    }
    EXIT();
    return 1;
}

void wlan_hdd_tdls_scan_done_callback(hdd_adapter_t *pAdapter)
{
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    ENTER();
    if (0 != (wlan_hdd_validate_context(pHddCtx)))
       return;

    /* free allocated memory at scan time */
    wlan_hdd_tdls_free_scan_request (&pHddCtx->tdls_scan_ctxt);

    /* if tdls was enabled before scan, re-enable tdls mode */
    if(eTDLS_SUPPORT_ENABLED == pHddCtx->tdls_mode_last ||
       eTDLS_SUPPORT_EXTERNAL_CONTROL == pHddCtx->tdls_mode_last ||
       eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY == pHddCtx->tdls_mode_last)
    {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                       ("%s: revert tdls mode %d"), __func__, pHddCtx->tdls_mode_last);

        wlan_hdd_tdls_set_mode(pHddCtx, pHddCtx->tdls_mode_last, FALSE);
    }
    wlan_hdd_tdls_check_bmps(pAdapter);
    EXIT();
}

void wlan_hdd_tdls_timer_restart(hdd_adapter_t *pAdapter,
                                 vos_timer_t *timer,
                                 v_U32_t expirationTime)
{
    hdd_station_ctx_t *pHddStaCtx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);

    /* Check whether driver load unload is in progress */
    if (vos_is_load_unload_in_progress(VOS_MODULE_ID_VOSS, NULL)) {
       hddLog(LOGE, FL("Driver load/unload is in progress."));
       return;
    }

    if (hdd_connIsConnected(pHddStaCtx)) {
        vos_timer_stop(timer);
        vos_timer_start(timer, expirationTime);
    }
}

void wlan_hdd_tdls_indicate_teardown(hdd_adapter_t *pAdapter,
                                           hddTdlsPeer_t *curr_peer,
                                           tANI_U16 reason)
{
    if ((NULL == pAdapter || WLAN_HDD_ADAPTER_MAGIC != pAdapter->magic) ||
        (NULL == curr_peer)) {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                FL("parameters passed are invalid"));
       return;
    }

    if (eTDLS_LINK_CONNECTED != curr_peer->link_status)
        return;

    wlan_hdd_tdls_set_peer_link_status(curr_peer,
                                       eTDLS_LINK_TEARING,
                                       eTDLS_LINK_UNSPECIFIED);
    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
              FL("Indicating NL80211_TDLS_TEARDOWN to supplicant, reason %d"),
                    reason);
    cfg80211_tdls_oper_request(pAdapter->dev,
                               curr_peer->peerMac,
                               NL80211_TDLS_TEARDOWN,
                               reason,
                               GFP_KERNEL);
}

/*EXT TDLS*/
int wlan_hdd_set_callback(hddTdlsPeer_t *curr_peer,
                         cfg80211_exttdls_callback callback)
{
    hdd_context_t *pHddCtx;
    hdd_adapter_t   *pAdapter;

    if (!curr_peer) return -1;

    pAdapter = curr_peer->pHddTdlsCtx->pAdapter;
    pHddCtx = WLAN_HDD_GET_CTX(pAdapter);
    if ((NULL == pHddCtx)) return -1;

    mutex_lock(&pHddCtx->tdls_lock);

    curr_peer->state_change_notification = callback;

    mutex_unlock(&pHddCtx->tdls_lock);
    return 0;
}

void wlan_hdd_tdls_get_wifi_hal_state(hddTdlsPeer_t *curr_peer,
                                      tANI_U32 *state,
                                      tANI_S32 *reason)
{
    hdd_context_t *hddctx;
    hdd_adapter_t *adapter;

    if (!curr_peer) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("curr_peer is NULL"));
        return;
    }

    adapter = curr_peer->pHddTdlsCtx->pAdapter;
    hddctx = WLAN_HDD_GET_CTX(adapter);

    if (0 != (wlan_hdd_validate_context(hddctx)))
        return;

    *reason = curr_peer->reason;

    switch(curr_peer->link_status)
    {
        case eTDLS_LINK_IDLE:
        case eTDLS_LINK_DISCOVERED:
            *state = QCA_WIFI_HAL_TDLS_ENABLED;
            break;
        case eTDLS_LINK_DISCOVERING:
        case eTDLS_LINK_CONNECTING:
            *state = QCA_WIFI_HAL_TDLS_ENABLED;
            break;
        case eTDLS_LINK_CONNECTED:
            if ((hddctx->cfg_ini->fEnableTDLSOffChannel) &&
                (hddctx->tdls_fw_off_chan_mode == ENABLE_CHANSWITCH))
                *state = QCA_WIFI_HAL_TDLS_ESTABLISHED_OFF_CHANNEL;
            else
                *state = QCA_WIFI_HAL_TDLS_ESTABLISHED;
            break;
        case eTDLS_LINK_TEARING:
            *state = QCA_WIFI_HAL_TDLS_DROPPED;
            break;
    }
}

int wlan_hdd_tdls_get_status(hdd_adapter_t *pAdapter,
                             const tANI_U8* mac,
                             uint32_t *opclass,
                             uint32_t *channel,
                             tANI_U32 *state,
                             tANI_S32 *reason)
{
    hddTdlsPeer_t *curr_peer;
    hdd_context_t *hddctx = WLAN_HDD_GET_CTX(pAdapter);

    if (0 != (wlan_hdd_validate_context(hddctx)))
        return -EINVAL;

    mutex_lock(&hddctx->tdls_lock);

    curr_peer = wlan_hdd_tdls_find_peer(pAdapter, mac, FALSE);
    if (curr_peer == NULL) {
       mutex_unlock(&hddctx->tdls_lock);
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is NULL"));
        *state = QCA_WIFI_HAL_TDLS_DISABLED;
        *reason = eTDLS_LINK_UNSPECIFIED;
        return -EINVAL;
    }

    if (hddctx->cfg_ini->fTDLSExternalControl &&
       (FALSE == curr_peer->isForcedPeer)) {
       VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                 FL("curr_peer is not Forced"));
       *state = QCA_WIFI_HAL_TDLS_DISABLED;
       *reason = eTDLS_LINK_UNSPECIFIED;
    } else {
        wlan_hdd_tdls_determine_channel_opclass(hddctx, pAdapter, curr_peer,
                                                channel, opclass);

        wlan_hdd_tdls_get_wifi_hal_state(curr_peer, state, reason);
    }

    mutex_unlock(&hddctx->tdls_lock);

    return 0;
}

static hddTdlsPeer_t *
wlan_hdd_tdls_find_first_connected_peer(hdd_adapter_t *pAdapter)
{
    int i;
    struct list_head *head;
    struct list_head *pos;
    hddTdlsPeer_t *curr_peer = NULL;
    tdlsCtx_t *pHddTdlsCtx;
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);

    if (0 != (wlan_hdd_validate_context(pHddCtx))) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("pHddCtx is not valid"));
        return NULL;
    }
    mutex_lock(&pHddCtx->tdls_lock);
    pHddTdlsCtx = WLAN_HDD_GET_TDLS_CTX_PTR(pAdapter);
    if (NULL == pHddTdlsCtx) {
        mutex_unlock(&pHddCtx->tdls_lock);
        return NULL;
    }
    for (i = 0; i < TDLS_PEER_LIST_SIZE; i++) {
        head = &pHddTdlsCtx->peer_list[i];
        list_for_each(pos, head) {
            curr_peer = list_entry (pos, hddTdlsPeer_t, node);
            if (curr_peer && (curr_peer->link_status == eTDLS_LINK_CONNECTED)) {
                mutex_unlock(&pHddCtx->tdls_lock);
                VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
                          FL(MAC_ADDRESS_STR "eTDLS_LINK_CONNECTED"),
                          MAC_ADDR_ARRAY(curr_peer->peerMac));
                return curr_peer;
            }
        }
    }
    mutex_unlock(&pHddCtx->tdls_lock);
    return NULL;
}

int hdd_set_tdls_offchannel(hdd_context_t *pHddCtx, int offchannel)
{
    if ((TRUE == pHddCtx->cfg_ini->fEnableTDLSOffChannel) &&
        (eTDLS_SUPPORT_ENABLED == pHddCtx->tdls_mode ||
         eTDLS_SUPPORT_EXTERNAL_CONTROL == pHddCtx->tdls_mode ||
         eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY == pHddCtx->tdls_mode)) {
        if (offchannel < CFG_TDLS_PREFERRED_OFF_CHANNEL_NUM_MIN ||
              offchannel > CFG_TDLS_PREFERRED_OFF_CHANNEL_NUM_MAX) {
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                      FL("Invalid tdls off channel %u"),
                      offchannel);
            return -EINVAL;
        }
    } else {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("Either TDLS or TDLS Off-channel is not enabled"));
        return  -ENOTSUPP;
    }
    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
              FL("change tdls off channel from %d to %d"),
              pHddCtx->tdls_off_channel, offchannel);
    pHddCtx->tdls_off_channel = offchannel;
    return 0;
}

int hdd_set_tdls_secoffchanneloffset(hdd_context_t *pHddCtx, int offchanoffset)
{
    if ((TRUE == pHddCtx->cfg_ini->fEnableTDLSOffChannel) &&
        (eTDLS_SUPPORT_ENABLED == pHddCtx->tdls_mode ||
         eTDLS_SUPPORT_EXTERNAL_CONTROL == pHddCtx->tdls_mode ||
         eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY == pHddCtx->tdls_mode)) {
        pHddCtx->tdls_channel_offset = 0;
        switch (offchanoffset) {
        case TDLS_SEC_OFFCHAN_OFFSET_0:
            pHddCtx->tdls_channel_offset = (1 << BW_20_OFFSET_BIT);
            break;
        case TDLS_SEC_OFFCHAN_OFFSET_40PLUS:
        case TDLS_SEC_OFFCHAN_OFFSET_40MINUS:
            pHddCtx->tdls_channel_offset = (1 << BW_40_OFFSET_BIT);
            break;
        case TDLS_SEC_OFFCHAN_OFFSET_80:
            pHddCtx->tdls_channel_offset = (1 << BW_80_OFFSET_BIT);
            break;
        case TDLS_SEC_OFFCHAN_OFFSET_160:
            pHddCtx->tdls_channel_offset = (1 << BW_160_OFFSET_BIT);
            break;
        default:
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                      FL("Invalid tdls secondary off channel offset %d"),
                      offchanoffset);
            return -EINVAL;
        }/* end switch */
    } else {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("Either TDLS or TDLS Off-channel is not enabled"));
        return  -ENOTSUPP;
    }
    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
              FL("change tdls secondary off channel offset to 0x%x"),
              pHddCtx->tdls_channel_offset);
    return 0;
}

int hdd_set_tdls_offchannelmode(hdd_adapter_t *pAdapter, int offchanmode)
{
    hddTdlsPeer_t *connPeer = NULL;
    hdd_station_ctx_t *pHddStaCtx = WLAN_HDD_GET_STATION_CTX_PTR(pAdapter);
    hdd_context_t *pHddCtx = WLAN_HDD_GET_CTX(pAdapter);
    tSmeTdlsChanSwitchParams chanSwitchParams;
    eHalStatus status = eHAL_STATUS_FAILURE;

    if (offchanmode < ENABLE_CHANSWITCH || offchanmode > DISABLE_CHANSWITCH) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("Invalid tdls off channel mode %d"),
                  offchanmode);
        return -EINVAL;
    }
    if (eConnectionState_Associated != pHddStaCtx->conn_info.connState) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("tdls off channel mode req in not associated state %d"),
                  offchanmode);
        return -EPERM;
    }
    if ((TRUE == pHddCtx->cfg_ini->fEnableTDLSOffChannel) &&
        (eTDLS_SUPPORT_ENABLED == pHddCtx->tdls_mode ||
         eTDLS_SUPPORT_EXTERNAL_CONTROL == pHddCtx->tdls_mode ||
         eTDLS_SUPPORT_EXPLICIT_TRIGGER_ONLY == pHddCtx->tdls_mode)) {
        connPeer = wlan_hdd_tdls_find_first_connected_peer(pAdapter);
        if (NULL == connPeer) {
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_FATAL,
                      FL("No TDLS Connected Peer"));
            return -EPERM;
       }
    } else {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_FATAL,
                  FL("TDLS Connection not supported"));
        return -ENOTSUPP;
    }

    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
              FL("TDLS Channel Switch in swmode=%d tdls_off_channel %d offchanoffset %d"),
              offchanmode, pHddCtx->tdls_off_channel,
              pHddCtx->tdls_channel_offset);

    switch (offchanmode) {
    case ENABLE_CHANSWITCH:
        if (pHddCtx->tdls_off_channel && pHddCtx->tdls_channel_offset) {
            chanSwitchParams.tdls_off_channel = pHddCtx->tdls_off_channel;
            chanSwitchParams.tdls_off_ch_bw_offset =
                                  pHddCtx->tdls_channel_offset;
            chanSwitchParams.opclass =
                wlan_hdd_find_opclass(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                      chanSwitchParams.tdls_off_channel,
                                      chanSwitchParams.tdls_off_ch_bw_offset);
        } else {
            VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                      FL("TDLS off-channel parameters are not set yet!!!"));
            return -EINVAL;
        }
        break;
    case DISABLE_CHANSWITCH:
        chanSwitchParams.tdls_off_channel = 0;
        chanSwitchParams.tdls_off_ch_bw_offset = 0;
        chanSwitchParams.opclass = 0;
        break;
    default:
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("Incorrect Parameters mode: %d tdls_off_channel: %d offchanoffset: %d"),
                  offchanmode, pHddCtx->tdls_off_channel,
                  pHddCtx->tdls_channel_offset);
        return -EINVAL;
    }/* end switch */

    chanSwitchParams.vdev_id = pAdapter->sessionId;
    chanSwitchParams.tdls_off_ch_mode = offchanmode;
    chanSwitchParams.is_responder = connPeer->is_responder;
    vos_mem_copy(&chanSwitchParams.peer_mac_addr,
                 &connPeer->peerMac, sizeof(tSirMacAddr));

    VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_INFO,
              FL("Peer " MAC_ADDRESS_STR " vdevId: %d, off channel: %d, offset: %d, mode: %d, is_responder: %d"),
              MAC_ADDR_ARRAY(chanSwitchParams.peer_mac_addr),
              chanSwitchParams.vdev_id,
              chanSwitchParams.tdls_off_channel,
              chanSwitchParams.tdls_off_ch_bw_offset,
              chanSwitchParams.tdls_off_ch_mode,
              chanSwitchParams.is_responder);

    status = sme_SendTdlsChanSwitchReq(WLAN_HDD_GET_HAL_CTX(pAdapter),
                                      &chanSwitchParams);

    if (status != eHAL_STATUS_SUCCESS) {
        VOS_TRACE(VOS_MODULE_ID_HDD, VOS_TRACE_LEVEL_ERROR,
                  FL("Failed to send channel switch request to sme"));
        return -EINVAL;
    }

    pHddCtx->tdls_fw_off_chan_mode = offchanmode;

    if (ENABLE_CHANSWITCH == offchanmode) {
        connPeer->pref_off_chan_num = chanSwitchParams.tdls_off_channel;
        connPeer->op_class_for_pref_off_chan = chanSwitchParams.opclass;
    }

    return 0;
}


/**
 * wlan_hdd_tdls_teardown_links() - teardown tdls links
 * @hddCtx : pointer to hdd context
 *
 * Return: 0 if success else non zero
 */
static int wlan_hdd_tdls_teardown_links(hdd_context_t *hddctx, uint32_t mode)
{
	uint16_t connected_tdls_peers = 0;
	uint8_t staidx;
	int ret = 0;
	hddTdlsPeer_t *curr_peer = NULL;
	hdd_adapter_t *adapter = NULL;

	if (eTDLS_SUPPORT_NOT_ENABLED == hddctx->tdls_mode) {
		hddLog(LOG1, FL("TDLS mode is disabled OR not enabled in FW"));
		return 0;
	}

	adapter = hdd_get_adapter(hddctx, WLAN_HDD_INFRA_STATION);

	if (adapter == NULL) {
		hddLog(LOGE, FL("Station Adapter Not Found"));
		return 0;
	}

	connected_tdls_peers = wlan_hdd_tdlsConnectedPeers(adapter);

	if (!connected_tdls_peers)
		return 0;

	for (staidx = 0; staidx < hddctx->max_num_tdls_sta;
							staidx++) {
		if (!hddctx->tdlsConnInfo[staidx].staId)
			continue;

		curr_peer = wlan_hdd_tdls_find_all_peer(hddctx,
				hddctx->tdlsConnInfo[staidx].peerMac.bytes);

		if (!curr_peer)
			continue;

		/* Check, is  connected peer supports more than one stream */
		if (curr_peer->spatial_streams == TDLS_NSS_1x1_MODE)
			continue;

		hddLog(LOG1, FL("indicate TDLS teardown (staId %d)"),
				curr_peer->staId);

		wlan_hdd_tdls_indicate_teardown(
					curr_peer->pHddTdlsCtx->pAdapter,
					curr_peer,
					eSIR_MAC_TDLS_TEARDOWN_UNSPEC_REASON);
		mutex_lock(&hddctx->tdls_lock);
		hddctx->tdls_teardown_peers_cnt++;
		mutex_unlock(&hddctx->tdls_lock);
	}
	mutex_lock(&hddctx->tdls_lock);
	if (hddctx->tdls_teardown_peers_cnt >= 1) {
		hddctx->tdls_nss_switch_in_progress = true;
		hddLog(LOG1, FL("TDLS peers to be torn down = %d"),
			hddctx->tdls_teardown_peers_cnt);
		/*  Antenna switch 2x2 to 1x1 */
		if (mode == HDD_ANTENNA_MODE_1X1) {
			hddctx->tdls_nss_transition_mode =
				TDLS_NSS_TRANSITION_2x2_to_1x1;
			ret = -EAGAIN;
		} else {
		/*  Antenna switch 1x1 to 2x2 */
			hddctx->tdls_nss_transition_mode =
				TDLS_NSS_TRANSITION_1x1_to_2x2;
			ret = 0;
		}
		hddLog(LOG1,
		       FL("TDLS teardown for antenna switch operation starts"));
	}
	mutex_unlock(&hddctx->tdls_lock);
	return ret;
}


/**
 * wlan_hdd_tdls_antenna_switch() - Dynamic TDLS antenna  switch 1x1 <-> 2x2
 * antenna mode in standalone station
 * @hdd_ctx: Pointer to hdd contex
 * @adapter: Pointer to hdd adapter
 *
 * Return: 0 if success else non zero
 */
int wlan_hdd_tdls_antenna_switch(hdd_context_t *hdd_ctx,
					hdd_adapter_t *adapter, uint32_t mode)
{
	hdd_station_ctx_t *sta_ctx = &adapter->sessionCtx.station;
	uint8_t tdls_peer_cnt;
	uint32_t vdev_nss;
	hdd_config_t *cfg_ini = hdd_ctx->cfg_ini;

	/* Check whether TDLS antenna switch is in progress */
	if (hdd_ctx->tdls_nss_switch_in_progress) {
		if (hdd_ctx->tdls_nss_teardown_complete == false) {
			hddLog(LOGE, FL("TDLS nss switch is in progress"));
			return -EAGAIN;
		} else {
			goto tdls_ant_sw_done;
		}
	}

	/* Check whether TDLS is connected or not */
	mutex_lock(&hdd_ctx->tdls_lock);
	tdls_peer_cnt = hdd_ctx->connected_peer_count;
	mutex_unlock(&hdd_ctx->tdls_lock);
	if (tdls_peer_cnt <= 0) {
		hddLog(LOG1, FL("No TDLS connection established"));
		goto tdls_ant_sw_done;
	}

	/* Check the supported nss for TDLS */
	if (IS_5G_CH(sta_ctx->conn_info.operationChannel))
		vdev_nss = CFG_TDLS_NSS(cfg_ini->vdev_type_nss_5g);
	else
		vdev_nss = CFG_TDLS_NSS(cfg_ini->vdev_type_nss_2g);

	if (vdev_nss == HDD_ANTENNA_MODE_1X1) {
		hddLog(LOG1,
		       FL("Supported NSS is 1X1, no need to teardown TDLS links"));
		goto tdls_ant_sw_done;
	}

	/* teardown all the tdls connections */
	return wlan_hdd_tdls_teardown_links(hdd_ctx, mode);

tdls_ant_sw_done:
	return 0;
}

/**
 * hdd_set_tdls_scan_type - set scan during active tdls session
 * @hdd_ctx: ptr to hdd context.
 * @val: scan type value: 0 or 1.
 *
 * Set scan type during tdls session. If set to 1, that means driver
 * shall maintain tdls link and allow scan regardless if tdls peer is
 * buffer sta capable or not and/or if device is sleep sta capable or
 * not. If tdls peer is not buffer sta capable then during scan there
 * will be loss of Rx packets and Tx would stop when device moves away
 * from tdls channel. If set to 0, then driver shall teardown tdls link
 * before initiating scan if peer is not buffer sta capable and device
 * is not sleep sta capable. By default, scan type is set to 0.
 *
 * Return: success (0) or failure (errno value)
 */
int hdd_set_tdls_scan_type(hdd_context_t *hdd_ctx, int val)
{
	if ((val != 0) && (val != 1)) {
		hddLog(LOGE, FL("Incorrect value of tdls scan type: %d"),
		       val);
		return -EINVAL;
	} else {
		hdd_ctx->cfg_ini->enable_tdls_scan = val;
		return 0;
	}
}
