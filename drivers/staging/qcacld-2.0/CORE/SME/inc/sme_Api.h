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


#if !defined( __SME_API_H )
#define __SME_API_H


/**=========================================================================

  \file  smeApi.h

  \brief prototype for SME APIs

  ========================================================================*/

/* $Header$ */

/*--------------------------------------------------------------------------
  Include Files
  ------------------------------------------------------------------------*/
#include "ccmApi.h"
#include "csrApi.h"
#include "pmcApi.h"
#include "vos_mq.h"
#include "vos_lock.h"
#include "halTypes.h"
#include "sirApi.h"
#include "btcApi.h"
#include "vos_nvitem.h"
#include "p2p_Api.h"
#include "smeInternal.h" 
#include "regdomain.h"

#ifdef FEATURE_OEM_DATA_SUPPORT
#include "oemDataApi.h"
#endif

#if defined WLAN_FEATURE_VOWIFI
#include "smeRrmInternal.h"
#endif

/*--------------------------------------------------------------------------
  Preprocessor definitions and constants
  ------------------------------------------------------------------------*/

#define SME_SUMMARY_STATS         1
#define SME_GLOBAL_CLASSA_STATS   2
#define SME_GLOBAL_CLASSB_STATS   4
#define SME_GLOBAL_CLASSC_STATS   8
#define SME_GLOBAL_CLASSD_STATS  16
#define SME_PER_STA_STATS        32

#define SME_INVALID_COUNTRY_CODE "XX"

#define SME_2_4_GHZ_MAX_FREQ    3000

#define SME_SET_CHANNEL_REG_POWER(reg_info_1, val) do { \
    reg_info_1 &= 0xff00ffff;             \
    reg_info_1 |= ((val & 0xff) << 16);   \
} while(0)

#define SME_SET_CHANNEL_MAX_TX_POWER(reg_info_2, val) do { \
    reg_info_2 &= 0xffff00ff;             \
    reg_info_2 |= ((val & 0xff) << 8);   \
} while(0)

#define NUM_OF_BANDS 2
/*--------------------------------------------------------------------------
  Type declarations
  ------------------------------------------------------------------------*/
typedef struct _smeConfigParams
{
   tCsrConfigParam  csrConfig;
#if defined WLAN_FEATURE_VOWIFI
   struct rrm_config_param rrmConfig;
#endif
#if defined FEATURE_WLAN_LFR
    tANI_U8   isFastRoamIniFeatureEnabled;
    tANI_U8   MAWCEnabled;
#endif
#if defined FEATURE_WLAN_ESE
    tANI_U8   isEseIniFeatureEnabled;
#endif
#if  defined (WLAN_FEATURE_VOWIFI_11R) || defined (FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_LFR)
    tANI_U8       isFastTransitionEnabled;
    tANI_U8       RoamRssiDiff;
    tANI_BOOLEAN  isWESModeEnabled;
#endif
    tANI_BOOLEAN  fScanOffload;
    tANI_U8       max_amsdu_num;
    tANI_BOOLEAN  fP2pListenOffload;
    tANI_BOOLEAN  pnoOffload;
    tANI_U8       fEnableDebugLog;
    tANI_U8       max_intf_count;
    tANI_BOOLEAN  enable5gEBT;
    tANI_BOOLEAN  enableSelfRecovery;
    uint32_t       f_sta_miracast_mcc_rest_time_val;
#ifdef FEATURE_AP_MCC_CH_AVOIDANCE
    bool          sap_channel_avoidance;
#endif /* FEATURE_AP_MCC_CH_AVOIDANCE */
    uint8_t       f_prefer_non_dfs_on_radar;
    uint32_t      fine_time_meas_cap;
    int8_t       first_scan_bucket_threshold;
} tSmeConfigParams, *tpSmeConfigParams;

typedef enum
{
    eSME_ROAM_TRIGGER_NONE = 0,
    eSME_ROAM_TRIGGER_SCAN = 1,
    eSME_ROAM_TRIGGER_FAST_ROAM = 2,
    eSME_ROAM_TRIGGER_MAX
} tSmeFastRoamTrigger;

#ifdef FEATURE_WLAN_TDLS

#define SME_TDLS_MAX_SUPP_CHANNELS       128
#define SME_TDLS_MAX_SUPP_OPER_CLASSES   32

typedef struct _smeTdlsPeerCapParams {
   tANI_U8 isPeerResponder;
   tANI_U8 peerUapsdQueue;
   tANI_U8 peerMaxSp;
   tANI_U8 peerBuffStaSupport;
   tANI_U8 peerOffChanSupport;
   tANI_U8 peerCurrOperClass;
   tANI_U8 selfCurrOperClass;
   tANI_U8 peerChanLen;
   tANI_U8 peerChan[SME_TDLS_MAX_SUPP_CHANNELS];
   tANI_U8 peerOperClassLen;
   tANI_U8 peerOperClass[SME_TDLS_MAX_SUPP_OPER_CLASSES];
   tANI_U8 prefOffChanNum;
   tANI_U8 prefOffChanBandwidth;
   tANI_U8 opClassForPrefOffChan;
} tSmeTdlsPeerCapParams;

typedef enum
{
    eSME_TDLS_PEER_STATE_PEERING,
    eSME_TDLS_PEER_STATE_CONNECTED,
    eSME_TDLS_PEER_STATE_TEARDOWN,
    /* Add peer mac into connection table */
    eSME_TDLS_PEER_ADD_MAC_ADDR,
    /* Remove peer mac from connection table */
    eSME_TDLS_PEER_REMOVE_MAC_ADDR,
} eSmeTdlsPeerState;

typedef struct _smeTdlsPeerStateParams
{
    tANI_U32 vdevId;
    tSirMacAddr peerMacAddr;
    tANI_U32 peerState;
    tSmeTdlsPeerCapParams peerCap;
} tSmeTdlsPeerStateParams;

#define ENABLE_CHANSWITCH  1
#define DISABLE_CHANSWITCH 2
#define BW_20_OFFSET_BIT   0
#define BW_40_OFFSET_BIT   1
#define BW_80_OFFSET_BIT   2
#define BW_160_OFFSET_BIT  3

typedef struct _smeTdlsChanSwitchParams
{
   tANI_U32    vdev_id;
   tSirMacAddr peer_mac_addr;
   tANI_U16    tdls_off_ch_bw_offset;/* Target Off Channel Bandwidth offset */
   tANI_U8     tdls_off_channel;     /* Target Off Channel */
   tANI_U8     tdls_off_ch_mode;     /* TDLS Off Channel Mode */
   tANI_U8     is_responder;         /* is peer responder or initiator */
   uint8_t     opclass;              /* tdls operating class */
} tSmeTdlsChanSwitchParams;
#endif /* FEATURE_WLAN_TDLS */

/* Thermal Mitigation*/

typedef struct {
    u_int16_t smeMinTempThreshold;
    u_int16_t smeMaxTempThreshold;
} tSmeThermalLevelInfo;


enum sme_max_bad_peer_thresh_levels {
	IEEE80211_B_LEVEL = 0,
	IEEE80211_AG_LEVEL,
	IEEE80211_N_LEVEL,
	IEEE80211_AC_LEVEL,
	IEEE80211_MAX_LEVEL,
};

struct sme_bad_peer_thresh{
	uint32_t cond;
	uint32_t delta;
	uint32_t percentage;
	uint32_t thresh;
	uint32_t limit;
};

struct sme_bad_peer_txctl_param{
	/* Array of thermal levels */
	struct sme_bad_peer_thresh thresh[IEEE80211_MAX_LEVEL];
	uint32_t enabled;
	uint32_t period;
	uint32_t txq_limit;
	uint32_t tgt_backoff;
	uint32_t tgt_report_prd;
};


#define SME_MAX_THERMAL_LEVELS (4)

typedef struct {
    /* Array of thermal levels */
    tSmeThermalLevelInfo smeThermalLevels[SME_MAX_THERMAL_LEVELS];
    u_int8_t smeThermalMgmtEnabled;
    u_int32_t smeThrottlePeriod;
} tSmeThermalParams;

#ifdef WLAN_FEATURE_APFIND
struct sme_ap_find_request_req{
    u_int16_t request_data_len;
    const u_int8_t* request_data;
};
#endif /* WLAN_FEATURE_APFIND */

/*-------------------------------------------------------------------------
  Function declarations and documentation
  ------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------

  \brief sme_Open() - Initialize all SME modules and put them at idle state

  The function initializes each module inside SME, PMC, CCM, CSR, etc. . Upon
  successfully return, all modules are at idle state ready to start.

  smeOpen must be called before any other SME APIs can be involved.
  smeOpen must be called after macOpen.

  \param hHal - The handle returned by macOpen.

  \return eHAL_STATUS_SUCCESS - SME is successfully initialized.

          Other status means SME is failed to be initialized
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_Open(tHalHandle hHal);

/*--------------------------------------------------------------------------

  \brief sme_init_chan_list() - Initialize sme channel info

  This function initializes sme channel information which can
  be known only after getting the regulatory information which, for discrete,
  will come from target after the fw got downloaded. For ISOC, this information
  will be available in nv.

  \param hal - The handle returned by macOpen.

  \param alpha2 - Country code passed by the hdd context.

  \param cc_src - Country code source passed by the hdd context.

  \return eHAL_STATUS_SUCCESS - SME is successfully initialized.

        Other status means SME is failed to be initialized
  \sa

---------------------------------------------------------------------------*/
eHalStatus sme_init_chan_list(tHalHandle hal, v_U8_t *alpha2,
                              COUNTRY_CODE_SOURCE cc_src);

/*--------------------------------------------------------------------------

  \brief sme_Close() - Release all SME modules and their resources.

  The function release each module in SME, PMC, CCM, CSR, etc. . Upon
  return, all modules are at closed state.

  No SME APIs can be involved after sme_Close except sme_Open.
  sme_Close must be called before macClose.

  \param hHal - The handle returned by macOpen.

  \return eHAL_STATUS_SUCCESS - SME is successfully close.

          Other status means SME is failed to be closed but caller still cannot
          call any other SME functions except smeOpen.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_Close(tHalHandle hHal);

/*--------------------------------------------------------------------------

  \brief sme_Start() - Put all SME modules at ready state.

  The function starts each module in SME, PMC, CCM, CSR, etc. . Upon
  successfully return, all modules are ready to run.

  \param hHal - The handle returned by macOpen.

  \return eHAL_STATUS_SUCCESS - SME is ready.

          Other status means SME is failed to start.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_Start(tHalHandle hHal);

/*--------------------------------------------------------------------------

  \brief sme_Stop() - Stop all SME modules and put them at idle state

  The function stops each module in SME, PMC, CCM, CSR, etc. . Upon
  return, all modules are at idle state ready to start.


  \param hHal - The handle returned by macOpen.

  \param tHalStopType - reason for stopping

  \return eHAL_STATUS_SUCCESS - SME is stopped.

          Other status means SME is failed to stop but caller should still consider
          SME is stopped.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_Stop(tHalHandle hHal, tHalStopType stopType);


/*--------------------------------------------------------------------------

  \brief sme_OpenSession() - Open a session for scan/roam operation.

  This is a synchronous API.


  \param hHal - The handle returned by macOpen.
  \param callback - A pointer to the function caller specifies for roam/connect status indication
  \param pContext - The context passed with callback
  \param pSelfMacAddr - Caller allocated memory filled with self MAC address (6 bytes)
  \param pbSessionId - pointer to a caller allocated buffer for returned session ID

  \return eHAL_STATUS_SUCCESS - session is opened. sessionId returned.

          Other status means SME is failed to open the session.
          eHAL_STATUS_RESOURCES - no more session available.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_OpenSession(tHalHandle hHal, csrRoamCompleteCallback callback,
                           void *pContext, tANI_U8 *pSelfMacAddr,
                           tANI_U8 *pbSessionId, tANI_U32 type,
                           tANI_U32 subType);

/*--------------------------------------------------------------------------

  \brief sme_SetCurrDeviceMode() - Sets the current operating device mode.
  \param hHal - The handle returned by macOpen.
  \param currDeviceMode - Current operating device mode.
  --------------------------------------------------------------------------*/

void sme_SetCurrDeviceMode (tHalHandle hHal, tVOS_CON_MODE currDeviceMode);

/*--------------------------------------------------------------------------

  \brief sme_CloseSession() - Open a session for scan/roam operation.

  This is a synchronous API.


  \param hHal - The handle returned by macOpen.

  \param sessionId - A previous opened session's ID.

  \return eHAL_STATUS_SUCCESS - session is closed.

          Other status means SME is failed to open the session.
          eHAL_STATUS_INVALID_PARAMETER - session is not opened.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_CloseSession(tHalHandle hHal, tANI_U8 sessionId,
                         csrRoamSessionCloseCallback callback, void *pContext);

eHalStatus sme_update_roam_params(tHalHandle hHal, uint8_t session_id,
		struct roam_ext_params roam_params_src, int update_param);

/*--------------------------------------------------------------------------

  \brief sme_UpdateConfig() - Change configurations for all SME modules

  The function updates some configuration for modules in SME, CCM, CSR, etc
  during SMEs close -> open sequence.

  Modules inside SME apply the new configuration at the next transaction.


  \param hHal - The handle returned by macOpen.
  \Param pSmeConfigParams - a pointer to a caller allocated object of
  typedef struct _smeConfigParams.

  \return eHAL_STATUS_SUCCESS - SME update the config parameters successfully.

          Other status means SME is failed to update the config parameters.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_UpdateConfig(tHalHandle hHal, tpSmeConfigParams pSmeConfigParams);

#ifdef FEATURE_WLAN_SCAN_PNO
/*--------------------------------------------------------------------------

  \brief sme_UpdateChannelConfig() - Update channel configuration in RIVA.

  It is used at driver start up to inform RIVA of the default channel
  configuration.

  This is a synchronous call

  \param hHal - The handle returned by macOpen.

  \return eHAL_STATUS_SUCCESS - SME update the channel config successfully.

          Other status means SME is failed to update the channel config.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_UpdateChannelConfig(tHalHandle hHal);

#endif // FEATURE_WLAN_SCAN_PNLO

#if defined(FEATURE_WLAN_ESE) && defined(FEATURE_WLAN_ESE_UPLOAD)
/*--------------------------------------------------------------------------

  \brief sme_SetPlmRequest() - send PLM request to firmware

  It is used to initiate PLM request

  This is a synchronous call

  \param hHal - The handle returned by macOpen.

  \return eHAL_STATUS_SUCCESS - SME could successfully sent PLM command

          Other status means SME is failed to send plm command

  --------------------------------------------------------------------------*/
eHalStatus sme_SetPlmRequest(tHalHandle hHal, tpSirPlmReq pPlm);

#endif

/*--------------------------------------------------------------------------

  \brief sme_set11dinfo() - Set the 11d information about valid channels
   and there power using information from nvRAM
   This function is called only for AP.

  This is a synchronous call

  \param hHal - The handle returned by macOpen.
  \Param pSmeConfigParams - a pointer to a caller allocated object of
  typedef struct _smeConfigParams.

  \return eHAL_STATUS_SUCCESS - SME update the config parameters successfully.

          Other status means SME is failed to update the config parameters.
  \sa
--------------------------------------------------------------------------*/

eHalStatus sme_set11dinfo(tHalHandle hHal,  tpSmeConfigParams pSmeConfigParams);

/*--------------------------------------------------------------------------

  \brief sme_getSoftApDomain() - Get the current regulatory domain of softAp.

  This is a synchronous call

  \param hHal - The handle returned by HostapdAdapter.
  \Param v_REGDOMAIN_t - The current Regulatory Domain requested for SoftAp.

  \return eHAL_STATUS_SUCCESS - SME successfully completed the request.

          Other status means, failed to get the current regulatory domain.
  \sa
--------------------------------------------------------------------------*/

eHalStatus sme_getSoftApDomain(tHalHandle hHal,  v_REGDOMAIN_t *domainIdSoftAp);

eHalStatus sme_setRegInfo(tHalHandle hHal,  tANI_U8 *apCntryCode);


/* ---------------------------------------------------------------------------
    \fn sme_ChangeConfigParams
    \brief The SME API exposed for HDD to provide config params to SME during
    SMEs stop -> start sequence.

    If HDD changed the domain that will cause a reset. This function will
    provide the new set of 11d information for the new domain. Currrently this
    API provides info regarding 11d only at reset but we can extend this for
    other params (PMC, QoS) which needs to be initialized again at reset.

    This is a synchronous call

    \param hHal - The handle returned by macOpen.

    \Param
    pUpdateConfigParam - a pointer to a structure (tCsrUpdateConfigParam) that
                currently provides 11d related information like Country code,
                Regulatory domain, valid channel list, Tx power per channel, a
                list with active/passive scan allowed per valid channel.

    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ChangeConfigParams(tHalHandle hHal,
                                 tCsrUpdateConfigParam *pUpdateConfigParam);

/*--------------------------------------------------------------------------

  \brief sme_HDDReadyInd() - SME sends eWNI_SME_SYS_READY_IND to PE to inform that the NIC
  is ready tio run.

  The function is called by HDD at the end of initialization stage so PE/HAL can enable the NIC
  to running state.


  \param hHal - The handle returned by macOpen.

  \return eHAL_STATUS_SUCCESS - eWNI_SME_SYS_READY_IND is sent to PE successfully.

          Other status means SME failed to send the message to PE.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_HDDReadyInd(tHalHandle hHal);


/*--------------------------------------------------------------------------

  \brief sme_ProcessMsg() - The main message processor for SME.

  The function is called by a message dispatcher when to process a message
  targeted for SME.


  \param hHal - The handle returned by macOpen.
  \param pMsg - A pointer to a caller allocated object of tSirMsgQ.

  \return eHAL_STATUS_SUCCESS - SME successfully process the message.

          Other status means SME failed to process the message.
  \sa

  --------------------------------------------------------------------------*/
eHalStatus sme_ProcessMsg(tHalHandle hHal, vos_msg_t* pMsg);

v_VOID_t sme_FreeMsg( tHalHandle hHal, vos_msg_t* pMsg );

/* ---------------------------------------------------------------------------
    \fn sme_ScanRequest
    \brief a wrapper function to Request a 11d or full scan from CSR.
    \param pScanRequestID - pointer to an object to get back the request ID
    \param callback - a callback function that scan calls upon finish, will not
                      be called if csrScanRequest returns error
    \param pContext - a pointer passed in for the callback
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ScanRequest(tHalHandle hHal, tANI_U8 sessionId, tCsrScanRequest *,
                           tANI_U32 *pScanRequestID,
                           csrScanCompleteCallback callback, void *pContext);


/* ---------------------------------------------------------------------------
    \fn sme_ScanSetBGScanparams
    \brief a wrapper function to request CSR to set BG scan params in PE
    \param pScanReq - BG scan request structure
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ScanSetBGScanparams(tHalHandle hHal, tANI_U8 sessionId, tCsrBGScanRequest *pScanReq);


/* ---------------------------------------------------------------------------
    \fn sme_ScanGetResult
    \brief a wrapper function to request scan results from CSR.
    \param pFilter - If pFilter is NULL, all cached results are returned
    \param phResult - an object for the result.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ScanGetResult(tHalHandle hHal, tANI_U8 sessionId, tCsrScanResultFilter *pFilter,
                            tScanResultHandle *phResult);

VOS_STATUS sme_get_ap_channel_from_scan_cache(tHalHandle hHal,
                                              tCsrRoamProfile *profile,
                                              tScanResultHandle *scan_cache,
                                              tANI_U8 *ap_chnl_id);
bool sme_store_joinreq_param(tHalHandle hal_handle,
                             tCsrRoamProfile *profile,
                             tScanResultHandle scan_cache,
                             uint32_t *roam_id,
                             uint32_t session_id);
bool sme_clear_joinreq_param(tHalHandle hal_handle,
                             uint32_t session_id);
VOS_STATUS sme_issue_stored_joinreq(tHalHandle hal_handle,
                                    uint32_t *roam_id,
                                    uint32_t session_id);

/* ---------------------------------------------------------------------------
    \fn sme_ScanFlushResult
    \brief a wrapper function to request CSR to clear scan results.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ScanFlushResult(tHalHandle hHal, tANI_U8 sessionId);

/*
 * ---------------------------------------------------------------------------
 *  \fn sme_FilterScanResults
 *  \brief a wrapper function to request CSR to filter the scan results based
 *   on valid channel list.
 *  \param hHal - The handle returned by macOpen.
 *  \param sessionId - the sessionId returned by sme_OpenSession.
 *  \return eHalStatus
 *---------------------------------------------------------------------------
 */
eHalStatus sme_FilterScanResults(tHalHandle hHal, tANI_U8 sessionId);

eHalStatus sme_ScanFlushP2PResult(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_ScanResultGetFirst
    \brief a wrapper function to request CSR to returns the first element of
           scan result.
    \param hScanResult - returned from csrScanGetResult
    \return tCsrScanResultInfo * - NULL if no result
  ---------------------------------------------------------------------------*/
tCsrScanResultInfo *sme_ScanResultGetFirst(tHalHandle,
                                          tScanResultHandle hScanResult);

/* ---------------------------------------------------------------------------
    \fn sme_ScanResultGetNext
    \brief a wrapper function to request CSR to returns the next element of
           scan result. It can be called without calling csrScanResultGetFirst
           first
    \param hScanResult - returned from csrScanGetResult
    \return Null if no result or reach the end
  ---------------------------------------------------------------------------*/
tCsrScanResultInfo *sme_ScanResultGetNext(tHalHandle,
                                          tScanResultHandle hScanResult);

/* ---------------------------------------------------------------------------
    \fn sme_ScanResultPurge
    \brief a wrapper function to request CSR to remove all items(tCsrScanResult)
           in the list and free memory for each item
    \param hScanResult - returned from csrScanGetResult. hScanResult is
                         considered gone by
    calling this function and even before this function returns.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ScanResultPurge(tHalHandle hHal, tScanResultHandle hScanResult);

/* ---------------------------------------------------------------------------
    \fn sme_ScanGetPMKIDCandidateList
    \brief a wrapper function to return the PMKID candidate list
    \param pPmkidList - caller allocated buffer point to an array of
                        tPmkidCandidateInfo
    \param pNumItems - pointer to a variable that has the number of
                       tPmkidCandidateInfo allocated when returning, this is
                       either the number needed or number of items put into
                       pPmkidList
    \return eHalStatus - when fail, it usually means the buffer allocated is not
                         big enough and pNumItems
    has the number of tPmkidCandidateInfo.
    \Note: pNumItems is a number of tPmkidCandidateInfo,
           not sizeof(tPmkidCandidateInfo) * something
  ---------------------------------------------------------------------------*/
eHalStatus sme_ScanGetPMKIDCandidateList(tHalHandle hHal, tANI_U8 sessionId,
                                        tPmkidCandidateInfo *pPmkidList,
                                        tANI_U32 *pNumItems );


/*----------------------------------------------------------------------------
  \fn sme_RoamRegisterLinkQualityIndCallback

  \brief
  a wrapper function to allow HDD to register a callback handler with CSR for
  link quality indications.

  Only one callback may be registered at any time.
  In order to deregister the callback, a NULL cback may be provided.

  Registration happens in the task context of the caller.

  \param callback - Call back being registered
  \param pContext - user data

  DEPENDENCIES: After CSR open

  \return eHalStatus
-----------------------------------------------------------------------------*/
eHalStatus sme_RoamRegisterLinkQualityIndCallback(tHalHandle hHal, tANI_U8 sessionId,
                                                  csrRoamLinkQualityIndCallback   callback,
                                                  void                           *pContext);


/* ---------------------------------------------------------------------------
    \fn sme_RoamConnect
    \brief a wrapper function to request CSR to initiate an association
    \param sessionId - the sessionId returned by sme_OpenSession.
    \param pProfile - can be NULL to join to any open ones
    \param pRoamId - to get back the request ID
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamConnect(tHalHandle hHal, tANI_U8 sessionId, tCsrRoamProfile *pProfile,
                           tANI_U32 *pRoamId);

/* ---------------------------------------------------------------------------
    \fn sme_RoamReassoc
    \brief a wrapper function to request CSR to initiate a re-association
    \param pProfile - can be NULL to join the currently connected AP. In that
    case modProfileFields should carry the modified field(s) which could trigger
    reassoc
    \param modProfileFields - fields which are part of tCsrRoamConnectedProfile
    that might need modification dynamically once STA is up & running and this
    could trigger a reassoc
    \param pRoamId - to get back the request ID
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamReassoc(tHalHandle hHal, tANI_U8 sessionId, tCsrRoamProfile *pProfile,
                          tCsrRoamModifyProfileFields modProfileFields,
                          tANI_U32 *pRoamId, v_BOOL_t fForce);

/* ---------------------------------------------------------------------------
    \fn sme_RoamConnectToLastProfile
    \brief a wrapper function to request CSR to disconnect and reconnect with
           the same profile
    \return eHalStatus. It returns fail if currently connected
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamConnectToLastProfile(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_RoamDisconnect
    \brief a wrapper function to request CSR to disconnect from a network
    \param reason -- To indicate the reason for disconnecting. Currently, only
                     eCSR_DISCONNECT_REASON_MIC_ERROR is meaningful.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamDisconnect(tHalHandle hHal, tANI_U8 sessionId, eCsrRoamDisconnectReason reason);

/* ---------------------------------------------------------------------------
    \fn sme_RoamStopBss
    \brief a wrapper function to request CSR to stop bss
    \param sessionId    - sessionId of SoftAP
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamStopBss(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_RoamGetAssociatedStas
    \brief To probe the list of associated stations from various modules of CORE stack.
    \This is an asynchronous API.
    \param sessionId    - sessionId of SoftAP
    \param modId        - Module from whom list of associated stations is to be
                          probed.
                          If an invalid module is passed then by default VOS_MODULE_ID_PE will be probed
    \param pUsrContext  - Opaque HDD context
    \param pfnSapEventCallback  - Sap event callback in HDD
    \param pAssocBuf    - Caller allocated memory to be filled with associated
                          stations info
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamGetAssociatedStas(tHalHandle hHal, tANI_U8 sessionId,
                                        VOS_MODULE_ID modId, void *pUsrContext,
                                        void *pfnSapEventCallback, tANI_U8 *pAssocStasBuf);

/* ---------------------------------------------------------------------------
    \fn sme_RoamDisconnectSta
    \brief To disassociate a station. This is an asynchronous API.
    \param pPeerMacAddr - Caller allocated memory filled with peer MAC address (6 bytes)
    \return eHalStatus  SUCCESS  Roam callback will be called to indicate actual results
  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamDisconnectSta(tHalHandle hHal, tANI_U8 sessionId,
                         struct tagCsrDelStaParams *pDelStaParams);

/* ---------------------------------------------------------------------------
    \fn sme_RoamDeauthSta
    \brief To disassociate a station. This is an asynchronous API.
    \param hHal - Global structure
    \param sessionId - sessionId of SoftAP
    \param pDelStaParams- Pointer to parameters of the station to deauthenticate
    \return eHalStatus  SUCCESS  Roam callback will be called to indicate actual results
  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamDeauthSta(tHalHandle hHal, tANI_U8 sessionId,
                             struct tagCsrDelStaParams *pDelStaParams);

/* ---------------------------------------------------------------------------
    \fn sme_RoamTKIPCounterMeasures
    \brief To start or stop TKIP counter measures. This is an asynchronous API.
    \param sessionId - sessionId of SoftAP
    \param bEnable - Flag to start/stop TKIP countermeasures
    \return eHalStatus  SUCCESS  Roam callback will be called to indicate actual results
  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamTKIPCounterMeasures(tHalHandle hHal, tANI_U8 sessionId, tANI_BOOLEAN bEnable);

/* ---------------------------------------------------------------------------
    \fn sme_RoamGetWpsSessionOverlap
    \brief To get the WPS PBC session overlap information.
    \This is an asynchronous API.
    \param sessionId    - sessionId of SoftAP
    \param pUsrContext  - Opaque HDD context
    \param pfnSapEventCallback  - Sap event callback in HDD
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamGetWpsSessionOverlap(tHalHandle hHal, tANI_U8 sessionId,
                                        void *pUsrContext, void *pfnSapEventCallback,
                                        v_MACADDR_t pRemoveMac);

/* ---------------------------------------------------------------------------
    \fn sme_RoamGetConnectState
    \brief a wrapper function to request CSR to return the current connect state
           of Roaming
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamGetConnectState(tHalHandle hHal, tANI_U8 sessionId, eCsrConnectState *pState);

/* ---------------------------------------------------------------------------
    \fn sme_RoamGetConnectProfile
    \brief a wrapper function to request CSR to return the current connect
           profile. Caller must call csrRoamFreeConnectProfile after it is done
           and before reuse for another csrRoamGetConnectProfile call.
    \param pProfile - pointer to a caller allocated structure
                      tCsrRoamConnectedProfile
    \return eHalStatus. Failure if not connected
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamGetConnectProfile(tHalHandle hHal, tANI_U8 sessionId,
                                     tCsrRoamConnectedProfile *pProfile);

/* ---------------------------------------------------------------------------
    \fn sme_RoamFreeConnectProfile
    \brief a wrapper function to request CSR to free and reinitialize the
           profile returned previously by csrRoamGetConnectProfile.
    \param pProfile - pointer to a caller allocated structure
                      tCsrRoamConnectedProfile
    \return eHalStatus.
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamFreeConnectProfile(tHalHandle hHal,
                                      tCsrRoamConnectedProfile *pProfile);

/* ---------------------------------------------------------------------------
    \fn sme_RoamSetPMKIDCache
    \brief a wrapper function to request CSR to return the PMKID candidate list
    \param pPMKIDCache - caller allocated buffer point to an array of
                         tPmkidCacheInfo
    \param numItems - a variable that has the number of tPmkidCacheInfo
                      allocated when returning, this is either the number needed
                      or number of items put into pPMKIDCache
    \param update_entire_cache - if TRUE, then it overwrites the entire cache
                                 with pPMKIDCache, else it updates entry by
                                 entry without deleting the old entries.
    \return eHalStatus - when fail, it usually means the buffer allocated is not
                         big enough and pNumItems has the number of
                         tPmkidCacheInfo.
    \Note: pNumItems is a number of tPmkidCacheInfo,
           not sizeof(tPmkidCacheInfo) * something
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamSetPMKIDCache( tHalHandle hHal, tANI_U8 sessionId,
                                  tPmkidCacheInfo *pPMKIDCache,
                                  tANI_U32 numItems,
                                  tANI_BOOLEAN update_entire_cache );

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
/* ---------------------------------------------------------------------------
 * \fn sme_RoamSetPSK_PMK
 * \brief a wrapper function to request CSR to save PSK/PMK
 *  This is a synchronous call.
 * \param hHal - Global structure
 * \param sessionId - SME sessionId
 * \param pPSK_PMK - pointer to an array of Psk[]/Pmk[]
 * \param pmk_len - Length could be only 16 bytes in case if LEAP
 *                  connections. Need to pass this information to
 *                  firmware.
 *\return eHalStatus -status whether PSK/PMK is set or not
 * ---------------------------------------------------------------------------*/
eHalStatus sme_RoamSetPSK_PMK (tHalHandle hHal, tANI_U8 sessionId,
                               tANI_U8 *pPSK_PMK, size_t pmk_len);
#endif
/* ---------------------------------------------------------------------------
    \fn sme_RoamGetSecurityReqIE
    \brief a wrapper function to request CSR to return the WPA or RSN or WAPI IE CSR
           passes to PE to JOIN request or START_BSS request
    This is a synchronous call.
    \param sessionId - returned by sme_OpenSession.
    \param pLen - caller allocated memory that has the length of pBuf as input.
                  Upon returned, *pLen has the needed or IE length in pBuf.
    \param pBuf - Caller allocated memory that contain the IE field, if any,
                  upon return
    \param secType - Specifies whether looking for WPA/WPA2/WAPI IE
    \return eHalStatus - when fail, it usually means the buffer allocated is not
                         big enough
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamGetSecurityReqIE(tHalHandle hHal, tANI_U8 sessionId, tANI_U32 *pLen,
                                  tANI_U8 *pBuf, eCsrSecurityType secType);

/* ---------------------------------------------------------------------------
    \fn sme_RoamGetSecurityRspIE
    \brief a wrapper function to request CSR to return the WPA or RSN or WAPI IE from
           the beacon or probe rsp if connected
    \param sessionId - returned by sme_OpenSession.
    \param pLen - caller allocated memory that has the length of pBuf as input.
                  Upon returned, *pLen has the needed or IE length in pBuf.
    \param pBuf - Caller allocated memory that contain the IE field, if any,
                  upon return
    \param secType - Specifies whether looking for WPA/WPA2/WAPI IE
    \return eHalStatus - when fail, it usually means the buffer allocated is not
                         big enough
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamGetSecurityRspIE(tHalHandle hHal, tANI_U8 sessionId, tANI_U32 *pLen,
                                  tANI_U8 *pBuf, eCsrSecurityType secType);


/* ---------------------------------------------------------------------------
    \fn sme_RoamGetNumPMKIDCache
    \brief a wrapper function to request CSR to return number of PMKID cache
           entries
    \return tANI_U32 - the number of PMKID cache entries
  ---------------------------------------------------------------------------*/
tANI_U32 sme_RoamGetNumPMKIDCache(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_RoamGetPMKIDCache
    \brief a wrapper function to request CSR to return PMKID cache from CSR
    \param pNum - caller allocated memory that has the space of the number of
                  pBuf tPmkidCacheInfo as input. Upon returned, *pNum has the
                  needed or actually number in tPmkidCacheInfo.
    \param pPmkidCache - Caller allocated memory that contains PMKID cache, if
                         any, upon return
    \return eHalStatus - when fail, it usually means the buffer allocated is not
                         big enough
  ---------------------------------------------------------------------------*/
eHalStatus sme_RoamGetPMKIDCache(tHalHandle hHal, tANI_U8 sessionId, tANI_U32 *pNum,
                                 tPmkidCacheInfo *pPmkidCache);

/* ---------------------------------------------------------------------------
    \fn sme_GetConfigParam
    \brief a wrapper function that HDD calls to get the global settings
           currently maintained by CSR.
    \param pParam - caller allocated memory
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetConfigParam(tHalHandle hHal, tSmeConfigParams *pParam);

/* ---------------------------------------------------------------------------
    \fn sme_GetStatistics
    \brief a wrapper function that client calls to register a callback to get
    different PHY level statistics from CSR.

    \param requesterId - different client requesting for statistics, HDD, UMA/GAN etc
    \param statsMask - The different category/categories of stats requester is looking for
    The order in which you set the bits in the statsMask for requesting
    different type of stats is:

      eCsrSummaryStats = bit 0
      eCsrGlobalClassAStats = bit 1
      eCsrGlobalClassBStats = bit 2
      eCsrGlobalClassCStats = bit 3
      eCsrGlobalClassDStats = bit 4
      eCsrPerStaStats = bit 5

    \param callback - SME sends back the requested stats using the callback
    \param periodicity - If requester needs periodic update, 0 means it's an one
                         time request
    \param cache - If requester is happy with cached stats
    \param staId - The station ID for which the stats is requested for
    \param pContext - user context to be passed back along with the callback
    \param sessionId - sme session interface
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetStatistics(tHalHandle hHal, eCsrStatsRequesterType requesterId,
                             tANI_U32 statsMask,
                             tCsrStatsCallback callback,
                             tANI_U32 periodicity, tANI_BOOLEAN cache,
                             tANI_U8 staId, void *pContext, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn smeGetTLSTAState
    \helper function to get the TL STA State whenever the function is called.

    \param staId - The staID to be passed to the TL
            to get the relevant TL STA State
    \return the state as tANI_U16
  ---------------------------------------------------------------------------*/
tANI_U16 smeGetTLSTAState(tHalHandle hHal, tANI_U8 staId);

/* ---------------------------------------------------------------------------
    \fn sme_GetRssi
    \brief a wrapper function that client calls to register a callback to get
           RSSI

    \param hHal - HAL handle for device
    \param callback - SME sends back the requested stats using the callback
    \param staId -    The station ID for which the stats is requested for
    \param bssid - The bssid of the connected session
    \param lastRSSI - RSSI value at time of request. In case fw cannot provide
                      RSSI, do not hold up but return this value.
    \param pContext - user context to be passed back along with the callback
    \param pVosContext - vos context
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetRssi(tHalHandle hHal,
                       tCsrRssiCallback callback,
                       tANI_U8 staId, tCsrBssid bssId, tANI_S8 lastRSSI,
                       void *pContext, void* pVosContext);

/* ---------------------------------------------------------------------------
    \fn sme_GetSnr
    \brief a wrapper function that client calls to register a callback to get
    SNR from FW

    \param callback - SME sends back the requested stats using the callback
    \param staId - The station ID for which the stats is requested for
    \param bssid - The bssid of the connected session
    \param pContext - user context to be passed back along with the callback
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetSnr(tHalHandle hHal,
                       tCsrSnrCallback callback,
                       tANI_U8 staId, tCsrBssid bssId,
                       void *pContext);

#if defined(FEATURE_WLAN_ESE) && defined(FEATURE_WLAN_ESE_UPLOAD)
/* ---------------------------------------------------------------------------
    \fn sme_GetTsmStats
    \brief a wrapper function that client calls to register a callback to get
           TSM Stats
    \param callback - SME sends back the requested stats using the callback
    \param staId - The station ID for which the stats is requested for
    \param pContext - user context to be passed back along with the callback
    \param pVosContext - vos context
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetTsmStats(tHalHandle hHal,
                             tCsrTsmStatsCallback callback,
                             tANI_U8 staId, tCsrBssid bssId,
                             void *pContext, void* pVosContext, tANI_U8 tid);
/* ---------------------------------------------------------------------------
    \fn sme_SetCCKMIe
    \brief function to store the CCKM IE passed from supplicant and use it
          while packing reassociation request
    \param  hHal - HAL handle for device
    \param  pCckmIe - pointer to CCKM IE data
    \param  pCckmIeLen - length of the CCKM IE
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_SetCCKMIe(tHalHandle hHal,
                         tANI_U8 sessionId,
                         tANI_U8 *pCckmIe,
                         tANI_U8 cckmIeLen);
/* ---------------------------------------------------------------------------
    \fn sme_SetEseBeaconRequest
    \brief  function to set ESE beacon request parameters
    \param  hHal - HAL handle for device
    \param  pEseBcnReq - pointer to ESE beacon request
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_SetEseBeaconRequest(tHalHandle hHal, const tANI_U8 sessionId,
                                   const tCsrEseBeaconReq* pEseBcnReq);

#endif /*FEATURE_WLAN_ESE && FEATURE_WLAN_ESE_UPLOAD */

/* ---------------------------------------------------------------------------
    \fn sme_CfgSetInt
    \brief a wrapper function that HDD calls to set parameters in CFG.
    \param cfgId - Configuration Parameter ID (type) for STA.
    \param ccmValue - The information related to Configuration Parameter ID
                      which needs to be saved in CFG
    \param callback - To be registered by CSR with CCM. Once the CFG done with
                      saving the information in the database, it notifies CCM &
                      then the callback will be invoked to notify.
    \param toBeSaved - To save the request for future reference
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_CfgSetInt(tHalHandle hHal, tANI_U32 cfgId, tANI_U32 ccmValue,
                         tCcmCfgSetCallback callback, eAniBoolean toBeSaved) ;

/* ---------------------------------------------------------------------------
    \fn sme_CfgSetStr
    \brief a wrapper function that HDD calls to set parameters in CFG.
    \param cfgId - Configuration Parameter ID (type) for STA.
    \param pStr - Pointer to the byte array which carries the information needs
                  to be saved in CFG
    \param length - Length of the data to be saved
    \param callback - To be registered by CSR with CCM. Once the CFG done with
                      saving the information in the database, it notifies CCM &
                      then the callback will be invoked to notify.
    \param toBeSaved - To save the request for future reference
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_CfgSetStr(tHalHandle hHal, tANI_U32 cfgId, tANI_U8 *pStr,
                         tANI_U32 length, tCcmCfgSetCallback callback,
                         eAniBoolean toBeSaved) ;


/* ---------------------------------------------------------------------------
    \fn sme_GetModifyProfileFields
    \brief HDD or SME - QOS calls this function to get the current values of
    connected profile fields, changing which can cause reassoc.
    This function must be called after CFG is downloaded and STA is in connected
    state. Also, make sure to call this function to get the current profile
    fields before calling the reassoc. So that pModifyProfileFields will have
    all the latest values plus the one(s) has been updated as part of reassoc
    request.
    \param pModifyProfileFields - pointer to the connected profile fields
    changing which can cause reassoc

    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_GetModifyProfileFields(tHalHandle hHal, tANI_U8 sessionId,
                                     tCsrRoamModifyProfileFields * pModifyProfileFields);


/*--------------------------------------------------------------------------
    \fn sme_SetConfigPowerSave
    \brief  Wrapper fn to change power save configuration in SME (PMC) module.
            For BMPS related configuration, this function also updates the CFG
            and sends a message to FW to pick up the new values. Note: Calling
            this function only updates the configuration and does not enable
            the specified power save mode.
    \param  hHal - The handle returned by macOpen.
    \param  psMode - Power Saving mode being modified
    \param  pConfigParams - a pointer to a caller allocated object of type
            tPmcSmpsConfigParams or tPmcBmpsConfigParams or tPmcImpsConfigParams
    \return eHalStatus
  --------------------------------------------------------------------------*/
eHalStatus sme_SetConfigPowerSave(tHalHandle hHal, tPmcPowerSavingMode psMode,
                                  void *pConfigParams);

/*--------------------------------------------------------------------------
    \fn sme_GetConfigPowerSave
    \brief  Wrapper fn to retrieve power save configuration in SME (PMC) module
    \param  hHal - The handle returned by macOpen.
    \param  psMode - Power Saving mode
    \param  pConfigParams - a pointer to a caller allocated object of type
            tPmcSmpsConfigParams or tPmcBmpsConfigParams or tPmcImpsConfigParams
    \return eHalStatus
  --------------------------------------------------------------------------*/
eHalStatus sme_GetConfigPowerSave(tHalHandle hHal, tPmcPowerSavingMode psMode,
                                  void *pConfigParams);

/* ---------------------------------------------------------------------------
    \fn sme_EnablePowerSave
    \brief  Enables one of the power saving modes. This API does not cause a
            device state change. This is purely a configuration API.
    \param  hHal - The handle returned by macOpen.
    \param  psMode - The power saving mode to enable.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_EnablePowerSave (
   tHalHandle hHal,
   tPmcPowerSavingMode psMode);

/* ---------------------------------------------------------------------------
    \fn sme_DisablePowerSave
    \brief   Disables one of the power saving modes.Disabling does not imply
             that device will be brought out of the current PS mode. This is
             purely a configuration API.
    \param  hHal - The handle returned by macOpen.
    \param  psMode - The power saving mode to disable.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_DisablePowerSave (
   tHalHandle hHal,
   tPmcPowerSavingMode psMode);

 /* ---------------------------------------------------------------------------
   \fn sme_SetHostPowerSave
    \brief   The BMPS logic is controlled by the User level Apps
    \param  hHal - The handle returned by macOpen.
    \param  psMode - The power saving mode to enable.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_SetHostPowerSave (
   tHalHandle hHal,
   v_BOOL_t psMode);

/* ---------------------------------------------------------------------------
    \fn sme_StartAutoBmpsTimer
    \brief  Starts a timer that periodically polls all the registered
            module for entry into Bmps mode. This timer is started only if BMPS is
            enabled and whenever the device is in full power.
    \param  hHal - The handle returned by macOpen.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_StartAutoBmpsTimer ( tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_StopAutoBmpsTimer
    \brief  Stops the Auto BMPS Timer that was started using sme_startAutoBmpsTimer
            Stopping the timer does not cause a device state change. Only the timer
            is stopped. If "Full Power" is desired, use the sme_RequestFullPower API
    \param  hHal - The handle returned by macOpen.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_StopAutoBmpsTimer ( tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_QueryPowerState
    \brief  Returns the current power state of the device.
    \param  hHal - The handle returned by macOpen.
    \param pPowerState - pointer to location to return power state
    \param pSwWlanSwitchState - ptr to location to return SW WLAN Switch state
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_QueryPowerState (
   tHalHandle hHal,
   tPmcPowerState *pPowerState,
   tPmcSwitchState *pSwWlanSwitchState);

/* ---------------------------------------------------------------------------
    \fn sme_IsPowerSaveEnabled
    \brief  Checks if the device is able to enter a particular power save mode
            This does not imply that the device is in a particular PS mode
    \param  hHal - The handle returned by macOpen.
    \param sessionId - sme sessionid
    \param psMode - the power saving mode
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern tANI_BOOLEAN sme_IsPowerSaveEnabled (
   tHalHandle hHal,
   tANI_U32 sessionId,
   tPmcPowerSavingMode psMode);

/* ---------------------------------------------------------------------------
    \fn sme_RequestFullPower
    \brief  Request that the device be brought to full power state.
            Note 1: If "fullPowerReason" specified in this API is set to
            eSME_FULL_PWR_NEEDED_BY_HDD, PMC will clear any "buffered wowl" requests
            and also clear any "buffered BMPS requests by HDD". Assumption is that since
            HDD is requesting full power, we need to undo any previous HDD requests for
            BMPS (using sme_RequestBmps) or WoWL (using sme_EnterWoWL). If the reason is
            specified anything other than above, the buffered requests for BMPS and WoWL
            will not be cleared.
            Note 2: Requesting full power (no matter what the fullPowerReason is) doesn't
            disable the "auto bmps timer" (if it is enabled) or clear any "buffered uapsd
            request".
            Note 3: When the device finally enters Full Power PMC will start a timer
            if any of the following holds true:
            - Auto BMPS mode is enabled
            - Uapsd request is pending
            - HDD's request for BMPS is pending
            - HDD's request for WoWL is pending
            On timer expiry PMC will attempt to put the device in BMPS mode if following
            (in addition to those listed above) holds true:
            - Polling of all modules through the Power Save Check routine passes
            - STA is associated to an access point
    \param  hHal - The handle returned by macOpen.
    \param  - callbackRoutine Callback routine invoked in case of success/failure
    \param  - callbackContext -  Cookie to be passed back during callback
    \param  - fullPowerReason - Reason why this API is being invoked. SME needs to
              distinguish between BAP and HDD requests
    \return eHalStatus - status
     eHAL_STATUS_SUCCESS - device brought to full power state
     eHAL_STATUS_FAILURE - device cannot be brought to full power state
     eHAL_STATUS_PMC_PENDING - device is being brought to full power state,
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_RequestFullPower (
   tHalHandle hHal,
   void (*callbackRoutine) (void *callbackContext, eHalStatus status),
   void *callbackContext,
   tRequestFullPowerReason fullPowerReason);

/* ---------------------------------------------------------------------------
    \fn sme_RequestBmps
    \brief  Request that the device be put in BMPS state. Request will be
            accepted only if BMPS mode is enabled and power save check routine
            passes. Only HDD should invoke this API.
    \param  hHal - The handle returned by macOpen.
    \param  - callbackRoutine Callback routine invoked in case of success/failure
    \param  - callbackContext -  Cookie to be passed back during callback
    \return eHalStatus
      eHAL_STATUS_SUCCESS - device is in BMPS state
      eHAL_STATUS_FAILURE - device cannot be brought to BMPS state
      eHAL_STATUS_PMC_PENDING - device is being brought to BMPS state
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_RequestBmps (
   tHalHandle hHal,
   void (*callbackRoutine) (void *callbackContext, eHalStatus status),
   void *callbackContext);

/* ---------------------------------------------------------------------------
    \fn  sme_SetDHCPTillPowerActiveFlag
    \brief  Sets/Clears DHCP related flag in PMC to disable/enable auto BMPS
            entry by PMC
    \param  hHal - The handle returned by macOpen.
  ---------------------------------------------------------------------------*/
void  sme_SetDHCPTillPowerActiveFlag(tHalHandle hHal, tANI_U8 flag);


/* ---------------------------------------------------------------------------
    \fn sme_StartUapsd
    \brief  Request that the device be put in UAPSD state. If the device is in
            Full Power it will be put in BMPS mode first and then into UAPSD
            mode.
    \param  hHal - The handle returned by macOpen.
    \param  - callbackRoutine Callback routine invoked in case of success/failure
    \param  - callbackContext -  Cookie to be passed back during callback
      eHAL_STATUS_SUCCESS - device is in UAPSD state
      eHAL_STATUS_FAILURE - device cannot be brought to UAPSD state
      eHAL_STATUS_PMC_PENDING - device is being brought to UAPSD state
      eHAL_STATUS_PMC_DISABLED - UAPSD is disabled or BMPS mode is disabled
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_StartUapsd (
   tHalHandle hHal,
   void (*callbackRoutine) (void *callbackContext, eHalStatus status),
   void *callbackContext);

/* ---------------------------------------------------------------------------
    \fn sme_StopUapsd
    \brief  Request that the device be put out of UAPSD state. Device will be
            put in in BMPS state after stop UAPSD completes. Buffered requests for
            UAPSD will be cleared after this.
    \param  hHal - The handle returned by macOpen.
    \return eHalStatus
      eHAL_STATUS_SUCCESS - device is put out of UAPSD and back in BMPS state
      eHAL_STATUS_FAILURE - device cannot be brought out of UAPSD state
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_StopUapsd (tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_RequestStandby
    \brief  Request that the device be put in standby. It is HDD's responsibility
            to bring the chip to full power and do a disconnect before calling
            this API. Request for standby will be rejected if STA is associated
            to an AP.
    \param  hHal - The handle returned by macOpen.
    \param  - callbackRoutine Callback routine invoked in case of success/failure
    \param  - callbackContext -  Cookie to be passed back during callback
    \return eHalStatus
      eHAL_STATUS_SUCCESS - device is in Standby mode
      eHAL_STATUS_FAILURE - device cannot be put in standby mode
      eHAL_STATUS_PMC_PENDING - device is being put in standby mode
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_RequestStandby (
   tHalHandle hHal,
   void (*callbackRoutine) (void *callbackContext, eHalStatus status),
   void *callbackContext);

/* ---------------------------------------------------------------------------
    \fn sme_RegisterPowerSaveCheck
    \brief  Register a power save check routine that is called whenever
            the device is about to enter one of the power save modes.
    \param  hHal - The handle returned by macOpen.
    \param  checkRoutine -  Power save check routine to be registered
    \param  callbackContext -  Cookie to be passed back during callback
    \return eHalStatus
            eHAL_STATUS_SUCCESS - successfully registered
            eHAL_STATUS_FAILURE - not successfully registered
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_RegisterPowerSaveCheck (
   tHalHandle hHal,
   tANI_BOOLEAN (*checkRoutine) (void *checkContext), void *checkContext);

/* ---------------------------------------------------------------------------
    \fn sme_Register11dScanDoneCallback
    \brief  Register a routine of type csrScanCompleteCallback which is
            called whenever an 11d scan is done
    \param  hHal - The handle returned by macOpen.
    \param  callback -  11d scan complete routine to be registered
    \return eHalStatus
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_Register11dScanDoneCallback (
   tHalHandle hHal,
   csrScanCompleteCallback);

/* ---------------------------------------------------------------------------
    \fn sme_DeregisterPowerSaveCheck
    \brief  Deregister a power save check routine
    \param  hHal - The handle returned by macOpen.
    \param  checkRoutine -  Power save check routine to be deregistered
    \return eHalStatus
            eHAL_STATUS_SUCCESS - successfully deregistered
            eHAL_STATUS_FAILURE - not successfully deregistered
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_DeregisterPowerSaveCheck (
   tHalHandle hHal,
   tANI_BOOLEAN (*checkRoutine) (void *checkContext));

/* ---------------------------------------------------------------------------
    \fn sme_RegisterDeviceStateUpdateInd
    \brief  Register a callback routine that is called whenever
            the device enters a new device state (Full Power, BMPS, UAPSD)
    \param  hHal - The handle returned by macOpen.
    \param  callbackRoutine -  Callback routine to be registered
    \param  callbackContext -  Cookie to be passed back during callback
    \return eHalStatus
            eHAL_STATUS_SUCCESS - successfully registered
            eHAL_STATUS_FAILURE - not successfully registered
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_RegisterDeviceStateUpdateInd (
   tHalHandle hHal,
   void (*callbackRoutine) (void *callbackContext, tPmcState pmcState),
   void *callbackContext);

/* ---------------------------------------------------------------------------
    \fn sme_DeregisterDeviceStateUpdateInd
    \brief  Deregister a routine that was registered for device state changes
    \param  hHal - The handle returned by macOpen.
    \param  callbackRoutine -  Callback routine to be deregistered
    \return eHalStatus
            eHAL_STATUS_SUCCESS - successfully deregistered
            eHAL_STATUS_FAILURE - not successfully deregistered
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_DeregisterDeviceStateUpdateInd (
   tHalHandle hHal,
   void (*callbackRoutine) (void *callbackContext, tPmcState pmcState));

/* ---------------------------------------------------------------------------
    \fn sme_WowlAddBcastPattern
    \brief  Add a pattern for Pattern Byte Matching in Wowl mode. Firmware will
            do a pattern match on these patterns when Wowl is enabled during BMPS
            mode.
    \param  hHal - The handle returned by macOpen.
    \param  pattern -  Pattern to be added
    \return eHalStatus
            eHAL_STATUS_FAILURE  Cannot add pattern
            eHAL_STATUS_SUCCESS  Request accepted.
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_WowlAddBcastPattern (
   tHalHandle hHal,
   tpSirWowlAddBcastPtrn pattern,
   tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_WowlDelBcastPattern
    \brief  Delete a pattern that was added for Pattern Byte Matching.
    \param  hHal - The handle returned by macOpen.
    \param  pattern -  Pattern to be deleted
    \return eHalStatus
            eHAL_STATUS_FAILURE  Cannot delete pattern
            eHAL_STATUS_SUCCESS  Request accepted.
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_WowlDelBcastPattern (
   tHalHandle hHal,
   tpSirWowlDelBcastPtrn pattern,
   tANI_U8   sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_EnterWowl
    \brief  This is the API to request entry into WOWL mode.
            WoWLAN works on top of BMPS mode. If the device is not in BMPS mode,
            SME will will cache the information that WOWL has been requested and
            attempt to put the device in BMPS first. On entry into BMPS, SME will
            enter the WOWL mode.
            Note 1: After WoWL request is accepted, If module other than HDD requests
            full power BEFORE WoWL request is completed, PMC will buffer the WoWL request
            and attempt to put the chip into BMPS+WOWL based on a timer.
            Note 2: Buffered request for WoWL will be cleared immediately AFTER
            "enter Wowl" completes or if HDD requests full power or
            if sme_ExitWoWL API is invoked.
            Note 3: Both UAPSD and WOWL work on top of BMPS. On entry into BMPS, SME
            will give priority to UAPSD and enable only UAPSD if both UAPSD and WOWL
            are required. Currently there is no requirement or use case to support UAPSD
            and WOWL at the same time.
            Note 4. Request for WoWL is rejected if there is a pending UAPSD request.
            Note 5. Request for WoWL is rejected if BMPS is disabled.

    \param  hHal - The handle returned by macOpen.
    \param  enterWowlCallbackRoutine -  Callback routine provided by HDD.
                               Used for success/failure notification by SME
    \param  enterWowlCallbackContext - A cookie passed by HDD, that is passed back to HDD
                              at the time of callback.
    \param  wakeReasonIndCB -  Callback routine provided by HDD.
                               Used for Wake Reason Indication by SME
    \param  wakeReasonIndCBContext - A cookie passed by HDD, that is passed back to HDD
                              at the time of callback.
    \return eHalStatus
            eHAL_STATUS_SUCCESS  Device is already in WoWLAN mode
            eHAL_STATUS_FAILURE  Device cannot enter WoWLAN mode.
            eHAL_STATUS_PMC_PENDING  Request accepted. SME will enable WOWL when BMPS
                                      mode is entered.
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_EnterWowl (
    tHalHandle hHal,
    void (*enterWowlCallbackRoutine) (void *callbackContext, eHalStatus status),
    void *enterWowlCallbackContext,
#ifdef WLAN_WAKEUP_EVENTS
    void (*wakeReasonIndCB) (void *callbackContext, tpSirWakeReasonInd pWakeReasonInd),
    void *wakeReasonIndCBContext,
#endif // WLAN_WAKEUP_EVENTS
    tpSirSmeWowlEnterParams wowlEnterParams, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_ExitWowl
    \brief  This is the SME API exposed to HDD to request exit from WoWLAN mode.
            SME will initiate exit from WoWLAN mode and device will be put in BMPS
            mode. Any Buffered request for WoWL will be cleared after this API.
    \param  hHal - The handle returned by macOpen.
    \param  wowlExitParams - Carries info on which smesession wowl exit
                             is requested.
    \return eHalStatus
            eHAL_STATUS_FAILURE  Device cannot exit WoWLAN mode. This can happen
                                  only if the previous "Enter WOWL" transaction has
                                  not even completed.
            eHAL_STATUS_SUCCESS  Request accepted to exit WoWLAN mode.
  ---------------------------------------------------------------------------*/
extern eHalStatus sme_ExitWowl (tHalHandle hHal, tpSirSmeWowlExitParams wowlExitParams);

/* ---------------------------------------------------------------------------

    \fn sme_RoamSetKey

    \brief To set encryption key. This function should be called only when connected
    This is an asynchronous API.

    \param pSetKeyInfo - pointer to a caller allocated object of tCsrSetContextInfo

    \param pRoamId  Upon success return, this is the id caller can use to identify the request in roamcallback

    \return eHalStatus  SUCCESS  Roam callback will be called indicate actually results

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamSetKey(tHalHandle, tANI_U8 sessionId, tCsrRoamSetKey *pSetKey, tANI_U32 *pRoamId);

/* ---------------------------------------------------------------------------

    \fn sme_RoamRemoveKey

    \brief To set encryption key. This is an asynchronous API.

    \param pRemoveKey - pointer to a caller allocated object of tCsrRoamRemoveKey

    \param pRoamId  Upon success return, this is the id caller can use to identify the request in roamcallback

    \return eHalStatus  SUCCESS  Roam callback will be called indicate actually results

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamRemoveKey(tHalHandle, tANI_U8 sessionId, tCsrRoamRemoveKey *pRemoveKey, tANI_U32 *pRoamId);


/* ---------------------------------------------------------------------------

    \fn sme_GetCountryCode

    \brief To return the current country code. If no country code is applied, default country code is
    used to fill the buffer.
    If 11d supported is turned off, an error is return and the last applied/default country code is used.
    This is a synchronous API.

    \param pBuf - pointer to a caller allocated buffer for returned country code.

    \param pbLen  For input, this parameter indicates how big is the buffer.
                   Upon return, this parameter has the number of bytes for country. If pBuf
                   doesn't have enough space, this function returns
                   fail status and this parameter contains the number that is needed.

    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_GetCountryCode(tHalHandle hHal, tANI_U8 *pBuf, tANI_U8 *pbLen);

/* ---------------------------------------------------------------------------

    \fn sme_SetCountryCode

    \brief To change the current/default country code.
    If 11d supported is turned off, an error is return.
    This is a synchronous API.

    \param pCountry - pointer to a caller allocated buffer for the country code.

    \param pfRestartNeeded  A pointer to caller allocated memory, upon successful return, it indicates
    whether a reset is required.

    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_SetCountryCode(tHalHandle hHal, tANI_U8 *pCountry, tANI_BOOLEAN *pfRestartNeeded);

/* ---------------------------------------------------------------------------
    \fn sme_ResetCountryCodeInformation
    \brief this function is to reset the country code current being used back to EEPROM default
    this includes channel list and power setting. This is a synchronous API.
    \param pfRestartNeeded - pointer to a caller allocated space. Upon successful return, it indicates whether
    a restart is needed to apply the change
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_ResetCountryCodeInformation(tHalHandle hHal, tANI_BOOLEAN *pfRestartNeeded);

/* ---------------------------------------------------------------------------
    \fn sme_GetSupportedCountryCode
    \brief this function is to get a list of the country code current being supported
    \param pBuf - Caller allocated buffer with at least 3 bytes, upon success return,
    this has the country code list. 3 bytes for each country code. This may be NULL if
    caller wants to know the needed byte count.
    \param pbLen - Caller allocated, as input, it indicates the length of pBuf. Upon success return,
    this contains the length of the data in pBuf. If pbuf is NULL, as input, *pbLen should be 0.
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_GetSupportedCountryCode(tHalHandle hHal, tANI_U8 *pBuf, tANI_U32 *pbLen);

/* ---------------------------------------------------------------------------
    \fn sme_GetCurrentRegulatoryDomain
    \brief this function is to get the current regulatory domain. This is a synchronous API.
    This function must be called after CFG is downloaded and all the band/mode setting already passed into
    SME. The function fails if 11d support is turned off.
    \param pDomain - Caller allocated buffer to return the current domain.
    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.
  -------------------------------------------------------------------------------*/
eHalStatus sme_GetCurrentRegulatoryDomain(tHalHandle hHal, v_REGDOMAIN_t *pDomain);

/* ---------------------------------------------------------------------------
    \fn sme_SetRegulatoryDomain
    \brief this function is to set the current regulatory domain.
    This function must be called after CFG is downloaded and all the band/mode setting already passed into
    SME. This is a synchronous API.
    \param domainId - indicate the domain (defined in the driver) needs to set to.
    See v_REGDOMAIN_t for definition
    \param pfRestartNeeded - pointer to a caller allocated space. Upon successful return, it indicates whether
    a restart is needed to apply the change
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_SetRegulatoryDomain(tHalHandle hHal, v_REGDOMAIN_t domainId, tANI_BOOLEAN *pfRestartNeeded);

/* ---------------------------------------------------------------------------

    \fn sme_GetRegulatoryDomainForCountry

    \brief To return a regulatory domain base on a country code. This is a synchronous API.

    \param pCountry - pointer to a caller allocated buffer for input country code.

    \param pDomainId  Upon successful return, it is the domain that country belongs to.
    If it is NULL, returning success means that the country code is known.

    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_GetRegulatoryDomainForCountry(tHalHandle hHal, tANI_U8 *pCountry, v_REGDOMAIN_t *pDomainId);



/* ---------------------------------------------------------------------------

    \fn sme_GetSupportedRegulatoryDomains

    \brief To return a list of supported regulatory domains. This is a synchronous API.

    \param pDomains - pointer to a caller allocated buffer for returned regulatory domains.

    \param pNumDomains  For input, this parameter indicates how many domains
                        pDomains can hold. Upon return, this parameter has the
                        number for supported domains. If pDomains doesn't have
                        enough space for all the supported domains, this
                        function returns fail status and this parameter
                        contains the number that is needed.

    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_GetSupportedRegulatoryDomains(tHalHandle hHal, v_REGDOMAIN_t *pDomains, tANI_U32 *pNumDomains);

//some support functions
tANI_BOOLEAN sme_Is11dSupported(tHalHandle hHal);
tANI_BOOLEAN sme_Is11hSupported(tHalHandle hHal);
tANI_BOOLEAN sme_IsWmmSupported(tHalHandle hHal);
//Upper layer to get the list of the base channels to scan for passively 11d info from csr
eHalStatus sme_ScanGetBaseChannels( tHalHandle hHal, tCsrChannelInfo * pChannelInfo );

typedef void ( *tSmeChangeCountryCallback)(void *pContext);
/* ---------------------------------------------------------------------------

    \fn sme_ChangeCountryCode

    \brief Change Country code from upper layer during WLAN driver operation.
           This is a synchronous API.

    \param hHal - The handle returned by macOpen.

    \param pCountry New Country Code String

    \param sendRegHint If we want to send reg hint to nl80211

    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_ChangeCountryCode( tHalHandle hHal,
                                  tSmeChangeCountryCallback callback,
                                  tANI_U8 *pCountry,
                                  void *pContext,
                                  void* pVosContext,
                                  tAniBool countryFromUserSpace,
                                  tAniBool sendRegHint);

/* ---------------------------------------------------------------------------

    \fn sme_GenericChangeCountryCode

    \brief Generic API to change country code

    \param hHal - The handle returned by macOpen.

    \param pCountry New Country Code String

    \param reg_domain Regulatory domain for the new country code

    \return eHalStatus  SUCCESS.

                        FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_GenericChangeCountryCode( tHalHandle hHal,
                                         tANI_U8 *pCountry,
                                         v_REGDOMAIN_t reg_domain);

/* ---------------------------------------------------------------------------

    \fn sme_TXFailMonitorStartStopInd

    \brief Indicate FW about TX Fail Monitor Indication`

    \param hHal - The handle returned by macOpen.

    \param tx_fail_count number of failures after which the firmware sends
                         an indication to host

    \param txFailIndCallback function to be called after receiving TX Fail
                             indication
    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_TXFailMonitorStartStopInd(tHalHandle hHal,
                                         tANI_U8 tx_fail_count,
                                         void * txFailIndCallback);

/* ---------------------------------------------------------------------------

    \fn sme_DHCPStartInd

    \brief Indicate FW about DHCP start event.

    \param hHal - The handle returned by macOpen.

    \param device_mode the mode of the device

    \param macAddr the MAC address of the adapter

    \param sessionId session ID

    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/

eHalStatus sme_DHCPStartInd( tHalHandle hHal,
                             tANI_U8 device_mode,
                             tANI_U8 *macAddr,
                             tANI_U8 sessionId );

/* ---------------------------------------------------------------------------

    \fn sme_DHCPStopInd

    \brief Indicate FW about DHCP stop event.

    \param hHal - The handle returned by macOpen.

    \param device_mode the mode of the device

    \param macAddr the MAC address of the adapter

    \param sessionId session ID

    \return eHalStatus  SUCCESS.

                         FAILURE or RESOURCES  The API finished and failed.

 -------------------------------------------------------------------------------*/
eHalStatus sme_DHCPStopInd( tHalHandle hHal,
                            tANI_U8 device_mode,
                            tANI_U8 *macAddr,
                            tANI_U8 sessionId );

/* ---------------------------------------------------------------------------
    \fn sme_BtcSignalBtEvent
    \brief  API to signal Bluetooth (BT) event to the WLAN driver. Based on the
            BT event type and the current operating mode of Libra (full power,
            BMPS, UAPSD etc), appropriate Bluetooth Coexistence (BTC) strategy
            would be employed.
    \param  hHal - The handle returned by macOpen.
    \param  pBtcBtEvent -  Pointer to a caller allocated object of type tSmeBtEvent
                           Caller owns the memory and is responsible for freeing it.
    \return VOS_STATUS
            VOS_STATUS_E_FAILURE  BT Event not passed to HAL. This can happen
                                   if driver has not yet been initialized or if BTC
                                   Events Layer has been disabled.
            VOS_STATUS_SUCCESS    BT Event passed to HAL
  ---------------------------------------------------------------------------*/
VOS_STATUS sme_BtcSignalBtEvent (tHalHandle hHal, tpSmeBtEvent pBtcBtEvent);

/* ---------------------------------------------------------------------------
    \fn sme_BtcSetConfig
    \brief  API to change the current Bluetooth Coexistence (BTC) configuration
            This function should be invoked only after CFG download has completed.
            Calling it after sme_HDDReadyInd is recommended.
    \param  hHal - The handle returned by macOpen.
    \param  pSmeBtcConfig - Pointer to a caller allocated object of type
                            tSmeBtcConfig. Caller owns the memory and is responsible
                            for freeing it.
    \return VOS_STATUS
            VOS_STATUS_E_FAILURE  Config not passed to HAL.
            VOS_STATUS_SUCCESS  Config passed to HAL
  ---------------------------------------------------------------------------*/
VOS_STATUS sme_BtcSetConfig (tHalHandle hHal, tpSmeBtcConfig pSmeBtcConfig);

/* ---------------------------------------------------------------------------
    \fn sme_BtcGetConfig
    \brief  API to retrieve the current Bluetooth Coexistence (BTC) configuration
    \param  hHal - The handle returned by macOpen.
    \param  pSmeBtcConfig - Pointer to a caller allocated object of type tSmeBtcConfig.
                            Caller owns the memory and is responsible for freeing it.
    \return VOS_STATUS
            VOS_STATUS_E_FAILURE - failure
            VOS_STATUS_SUCCESS  success
  ---------------------------------------------------------------------------*/
VOS_STATUS sme_BtcGetConfig (tHalHandle hHal, tpSmeBtcConfig pSmeBtcConfig);

/* ---------------------------------------------------------------------------
    \fn sme_SetCfgPrivacy
    \brief  API to set configure privacy parameters
    \param  hHal - The handle returned by macOpen.
    \param  pProfile - Pointer CSR Roam profile.
    \param  fPrivacy - This parameter indicates status of privacy

    \return void
  ---------------------------------------------------------------------------*/
void sme_SetCfgPrivacy(tHalHandle hHal, tCsrRoamProfile *pProfile, tANI_BOOLEAN fPrivacy);

/* ---------------------------------------------------------------------------
    \fn sme_getRecoveryStats
    \brief  API to get recovery stats for SME stuck cmds.
    \param  hHal - The handle returned by macOpen.

    \return void
  ---------------------------------------------------------------------------*/
void sme_getRecoveryStats(tHalHandle hHal);

#if defined WLAN_FEATURE_VOWIFI
/* ---------------------------------------------------------------------------
    \fn sme_NeighborReportRequest
    \brief  API to request neighbor report.
    \param  hHal - The handle returned by macOpen.
    \param  pRrmNeighborReq - Pointer to a caller allocated object of type
                            tRrmNeighborReq. Caller owns the memory and is responsible
                            for freeing it.
    \return VOS_STATUS
            VOS_STATUS_E_FAILURE - failure
            VOS_STATUS_SUCCESS  success
  ---------------------------------------------------------------------------*/
VOS_STATUS sme_NeighborReportRequest (tHalHandle hHal, tANI_U8 sessionId,
                tpRrmNeighborReq pRrmNeighborReq, tpRrmNeighborRspCallbackInfo callbackInfo);
#endif

VOS_STATUS sme_GetWcnssWlanCompiledVersion(tHalHandle hHal,
                                           tSirVersionType *pVersion);
VOS_STATUS sme_GetWcnssWlanReportedVersion(tHalHandle hHal,
                                           tSirVersionType *pVersion);
VOS_STATUS sme_GetWcnssSoftwareVersion(tHalHandle hHal,
                                       tANI_U8 *pVersion,
                                       tANI_U32 versionBufferSize);
VOS_STATUS sme_GetWcnssHardwareVersion(tHalHandle hHal,
                                       tANI_U8 *pVersion,
                                       tANI_U32 versionBufferSize);

#ifdef FEATURE_WLAN_WAPI
/* ---------------------------------------------------------------------------
    \fn sme_ScanGetBKIDCandidateList
    \brief a wrapper function to return the BKID candidate list
    \param pBkidList - caller allocated buffer point to an array of
                        tBkidCandidateInfo
    \param pNumItems - pointer to a variable that has the number of
                       tBkidCandidateInfo allocated when returning, this is
                       either the number needed or number of items put into
                       pPmkidList
    \return eHalStatus - when fail, it usually means the buffer allocated is not
                         big enough and pNumItems
    has the number of tBkidCandidateInfo.
    \Note: pNumItems is a number of tBkidCandidateInfo,
           not sizeof(tBkidCandidateInfo) * something
  ---------------------------------------------------------------------------*/
eHalStatus sme_ScanGetBKIDCandidateList(tHalHandle hHal, tANI_U32 sessionId,
                                        tBkidCandidateInfo *pBkidList,
                                        tANI_U32 *pNumItems );
#endif /* FEATURE_WLAN_WAPI */

#ifdef FEATURE_OEM_DATA_SUPPORT
/********************************************************************************************
  Oem data related modifications
*********************************************************************************************/
/* ---------------------------------------------------------------------------
    \fn sme_OemDataReq
    \param sessionId - session id of session to be used for oem data req.
    \param pOemDataReqID - pointer to an object to get back the request ID
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_OemDataReq(tHalHandle hHal,
                                       tANI_U8 sessionId,
                                       tOemDataReqConfig *,
                                       tANI_U32 *pOemDataReqID);

#endif /*FEATURE_OEM_DATA_SUPPORT*/



/* ---------------------------------------------------------------------------

    \fn sme_RoamUpdateAPWPSIE

    \brief To update AP's WPS IE. This function should be called after SME AP session is created
    This is an asynchronous API.

    \param pAPWPSIES - pointer to a caller allocated object of tCsrRoamAPWPSIES

    \return eHalStatus  SUCCESS  Roam callback will be called indicate actually results

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/

eHalStatus sme_RoamUpdateAPWPSIE(tHalHandle, tANI_U8 sessionId, tSirAPWPSIEs *pAPWPSIES);
/* ---------------------------------------------------------------------------

    \fn sme_RoamUpdateAPWPARSNIEs

    \brief To update AP's WPA/RSN IEs. This function should be called after SME AP session is created
    This is an asynchronous API.

    \param pAPSirRSNie - pointer to a caller allocated object of tSirRSNie with WPS/RSN IEs

    \return eHalStatus  SUCCESS

                         FAILURE or RESOURCES  The API finished and failed.

  -------------------------------------------------------------------------------*/
eHalStatus sme_RoamUpdateAPWPARSNIEs(tHalHandle hHal, tANI_U8 sessionId, tSirRSNie * pAPSirRSNie);

/* ---------------------------------------------------------------------------

    sme_ChangeMCCBeaconInterval

    \brief To update P2P-GO's beacon Interval.

    \return eHalStatus  SUCCESS
                        FAILURE or RESOURCES
                        The API finished and failed.
  -------------------------------------------------------------------------------*/
eHalStatus sme_ChangeMCCBeaconInterval(tHalHandle hHal, tANI_U8 sessionId);


/* ---------------------------------------------------------------------------
    \fn sme_SetHostOffload
    \brief  API to set the host offload feature.
    \param  hHal - The handle returned by macOpen.
    \param  pRequest -  Pointer to the offload request.
    \return eHalStatus
   ---------------------------------------------------------------------------*/
eHalStatus sme_SetHostOffload (tHalHandle hHal, tANI_U8 sessionId,
                                    tpSirHostOffloadReq pRequest);

/* ---------------------------------------------------------------------------
    \fn sme_SetKeepAlive
    \brief  API to set the Keep Alive feature.
    \param  hHal - The handle returned by macOpen.
    \param  pRequest -  Pointer to the Keep Alive request.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetKeepAlive (tHalHandle hHal, tANI_U8 sessionId,
                                  tpSirKeepAliveReq pRequest);

/* ----------------------------------------------------------------------------
   \fn sme_GetOperationChannel
   \brief API to get current channel on which STA is parked
   this function gives channel information only of infra station or IBSS station.
   \param hHal, pointer to memory location and sessionId
   \returns eHAL_STATUS_SUCCESS
            eHAL_STATUS_FAILURE
-------------------------------------------------------------------------------*/
eHalStatus sme_GetOperationChannel(tHalHandle hHal, tANI_U32 *pChannel, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------

    \fn sme_RegisterMgtFrame

    \brief To register management frame of specified type and subtype.
    \param frameType - type of the frame that needs to be passed to HDD.
    \param matchData - data which needs to be matched before passing frame
                       to HDD.
    \param matchDataLen - Length of matched data.
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_RegisterMgmtFrame(tHalHandle hHal, tANI_U8 sessionId,
                     tANI_U16 frameType, tANI_U8* matchData, tANI_U16 matchLen);

/* ---------------------------------------------------------------------------

    \fn sme_DeregisterMgtFrame

    \brief To De-register management frame of specified type and subtype.
    \param frameType - type of the frame that needs to be passed to HDD.
    \param matchData - data which needs to be matched before passing frame
                       to HDD.
    \param matchDataLen - Length of matched data.
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_DeregisterMgmtFrame(tHalHandle hHal, tANI_U8 sessionId,
                     tANI_U16 frameType, tANI_U8* matchData, tANI_U16 matchLen);

/* ---------------------------------------------------------------------------

  \fn    sme_ConfigureRxpFilter

  \brief
    SME will pass this request to lower mac to set/reset the filter on RXP for
    multicast & broadcast traffic.

  \param

    hHal - The handle returned by macOpen.

    filterMask- Currently the API takes a 1 or 0 (set or reset) as filter.
    Basically to enable/disable the filter (to filter "all" mcbc traffic) based
    on this param. In future we can use this as a mask to set various types of
    filters as suggested below:
    FILTER_ALL_MULTICAST:
    FILTER_ALL_BROADCAST:
    FILTER_ALL_MULTICAST_BROADCAST:


  \return eHalStatus


--------------------------------------------------------------------------- */
eHalStatus sme_ConfigureRxpFilter( tHalHandle hHal,
                              tpSirWlanSetRxpFilters  wlanRxpFilterParam);

/* ---------------------------------------------------------------------------

  \fn    sme_ConfigureAppsCpuWakeupState

  \brief
    SME will pass this request to lower mac to dynamically adjusts the listen
    interval based on the WLAN/MSM activity. This feature is named as
    Telescopic Beacon wakeup feature.

  \param

    hHal - The handle returned by macOpen.

    isAppsAwake- Depicts the state of the Apps CPU


  \return eHalStatus


--------------------------------------------------------------------------- */
eHalStatus sme_ConfigureAppsCpuWakeupState( tHalHandle hHal, tANI_BOOLEAN  isAppsAwake);

/* ---------------------------------------------------------------------------

  \fn    sme_ConfigureSuspendInd

  \brief
    SME will pass this request to lower mac to Indicate that the wlan needs to
    be suspended

  \param

    hHal - The handle returned by macOpen.

    wlanSuspendParam- Depicts the wlan suspend params

    csrReadyToSuspendCallback - Callback to be called when ready to suspend
                                event is received.
    callbackContext  - Context associated with csrReadyToSuspendCallback.

  \return eHalStatus


--------------------------------------------------------------------------- */
eHalStatus sme_ConfigureSuspendInd( tHalHandle hHal,
                             tpSirWlanSuspendParam  wlanSuspendParam,
                             csrReadyToSuspendCallback,
                             void *callbackContext);

/* ---------------------------------------------------------------------------

  \fn    sme_ConfigureResumeReq

  \brief
    SME will pass this request to lower mac to Indicate that the wlan needs to
    be Resumed

  \param

    hHal - The handle returned by macOpen.

    wlanResumeParam- Depicts the wlan resume params


  \return eHalStatus


--------------------------------------------------------------------------- */
eHalStatus sme_ConfigureResumeReq( tHalHandle hHal,
                             tpSirWlanResumeParam  wlanResumeParam);

#ifdef WLAN_FEATURE_EXTWOW_SUPPORT
/* ---------------------------------------------------------------------------

  \fn    sme_ConfigureExtWoW

  \brief
    SME will pass this request to lower mac to configure Indoor WoW parameters.

  \param

    hHal - The handle returned by macOpen.

    wlanExtParams- Depicts the wlan Ext params

  \return eHalStatus


--------------------------------------------------------------------------- */
eHalStatus sme_ConfigureExtWoW( tHalHandle hHal,
                          tpSirExtWoWParams  wlanExtParams,
                          csrReadyToSuspendCallback callback,
                          void *callbackContext);


/* ---------------------------------------------------------------------------

  \fn    sme_ConfigureAppType1Params

  \brief
    SME will pass this request to lower mac to configure Indoor WoW parameters.

  \param

    hHal - The handle returned by macOpen.

    wlanAppType1Params- Depicts the wlan Indoor params

  \return eHalStatus


--------------------------------------------------------------------------- */
eHalStatus sme_ConfigureAppType1Params( tHalHandle hHal,
                          tpSirAppType1Params  wlanAppType1Params);

/* ---------------------------------------------------------------------------

  \fn    sme_ConfigureAppType2Params

  \brief
    SME will pass this request to lower mac to configure Indoor WoW parameters.

  \param

    hHal - The handle returned by macOpen.

    wlanAppType2Params- Depicts the wlan Indoor params

  \return eHalStatus


--------------------------------------------------------------------------- */
eHalStatus sme_ConfigureAppType2Params( tHalHandle hHal,
                          tpSirAppType2Params  wlanAppType2Params);
#endif

/* ---------------------------------------------------------------------------

    \fn sme_GetInfraSessionId

    \brief To get the session ID for infra session, if connected
    This is a synchronous API.

    \param hHal - The handle returned by macOpen.

    \return sessionid, -1 if infra session is not connected

  -------------------------------------------------------------------------------*/
tANI_S8 sme_GetInfraSessionId(tHalHandle hHal);

/* ---------------------------------------------------------------------------

    \fn sme_GetInfraOperationChannel

    \brief To get the operating channel for infra session, if connected
    This is a synchronous API.

    \param hHal - The handle returned by macOpen.
    \param sessionId - the sessionId returned by sme_OpenSession.

    \return operating channel, 0 if infra session is not connected

  -------------------------------------------------------------------------------*/
tANI_U8 sme_GetInfraOperationChannel( tHalHandle hHal, tANI_U8 sessionId);
/* ---------------------------------------------------------------------------

    \fn sme_GetConcurrentOperationChannel

    \brief To get the operating channel for other concurrent sessions, if connected
    This is a synchronous API.

    \param hHal - The handle returned by macOpen.
    \param currentPersona - persona that is trying to come up.

    \return operating channel, 0 if infra session is not connected

  -------------------------------------------------------------------------------*/
tANI_U8 sme_GetConcurrentOperationChannel( tHalHandle hHal );

#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
v_U16_t sme_CheckConcurrentChannelOverlap( tHalHandle hHal, v_U16_t sap_ch,
                                eCsrPhyMode sapPhyMode, v_U8_t cc_switch_mode);
#endif
/* ---------------------------------------------------------------------------
    \fn sme_AbortMacScan
    \brief  API to cancel MAC scan.
    \param  hHal - The handle returned by macOpen.
    \param  sessionId - sessionId for interface
    \param  reason - Reason to abort the scan
    \return VOS_STATUS
            VOS_STATUS_E_FAILURE - failure
            VOS_STATUS_SUCCESS  success
  ---------------------------------------------------------------------------*/
eHalStatus sme_AbortMacScan(tHalHandle hHal, tANI_U8 sessionId,
                            eCsrAbortReason reason);

/* ---------------------------------------------------------------------------
    \fn sme_GetCfgValidChannels
    \brief  API to get valid channel list
    \param  hHal - The handle returned by macOpen.
    \param  aValidChannels -  Pointer to the valid channel list
    \param  len -  valid channel list length
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetCfgValidChannels(tHalHandle hHal, tANI_U8 *aValidChannels, tANI_U32 *len);

#ifdef FEATURE_WLAN_SCAN_PNO

/* ---------------------------------------------------------------------------
    \fn sme_SetPreferredNetworkList
    \brief  API to set the Preferred Network List Offload feature.
    \param  hHal - The handle returned by macOpen.
    \param  pRequest -  Pointer to the offload request.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetPreferredNetworkList (tHalHandle hHal, tpSirPNOScanReq pRequest, tANI_U8 sessionId, preferredNetworkFoundIndCallback callbackRoutine, void *callbackContext );

/******************************************************************************
*
* Name: sme_PreferredNetworkFoundInd
*
* Description:
*    Invoke Preferred Network Found Indication
*
* Parameters:
*    hHal - HAL handle for device
*    pMsg - found network description
*
* Returns: eHalStatus
*
******************************************************************************/
eHalStatus sme_PreferredNetworkFoundInd (tHalHandle hHal, void* pMsg);
#endif // FEATURE_WLAN_SCAN_PNO

/* ---------------------------------------------------------------------------
    \fn sme_SetPowerParams
    \brief  API to set Power Parameters
    \param  hHal - The handle returned by macOpen.
    \param  pwParams -  Pointer to the power parameters requested.
    \param  forced - if true, not to be dropped silently in host, it must reach
    FW; It is added to avoid a race condition scenario where LIM hasn't deleted
    the session yet before power params gets sent to PMC
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetPowerParams(tHalHandle hHal, tSirSetPowerParamsReq* pwParams, tANI_BOOLEAN forced);

/* ---------------------------------------------------------------------------
    \fn sme_SetTxPerTracking
    \brief  Set Tx PER tracking configuration parameters
    \param  hHal - The handle returned by macOpen.
    \param  pTxPerTrackingParam - Tx PER configuration parameters
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetTxPerTracking (
   tHalHandle hHal,
   void (*pCallbackfn) (void *pCallbackContext),
   void *pCallbackContext,
   tpSirTxPerTrackingParam pTxPerTrackingParam);

#ifdef WLAN_FEATURE_PACKET_FILTERING
/* ---------------------------------------------------------------------------
    \fn sme_ReceiveFilterSetFilter
    \brief  API to set 8023 Multicast Address List
    \param  hHal - The handle returned by macOpen.
    \param  pMulticastAddrs - Pointer to the Multicast Address List
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_8023MulticastList(tHalHandle hHal, tANI_U8 sessionId, tpSirRcvFltMcAddrList pMulticastAddrs);

/* ---------------------------------------------------------------------------
    \fn sme_ReceiveFilterSetFilter
    \brief  API to set Receive Packet Filter
    \param  hHal - The handle returned by macOpen.
    \param  pRcvPktFilterCfg - Receive Packet Filter parameter
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ReceiveFilterSetFilter(tHalHandle hHal, tpSirRcvPktFilterCfgType pRcvPktFilterCfg,
                                           tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_GetFilterMatchCount
    \brief  API to get D0 PC Filter Match Count
    \param  hHal - The handle returned by macOpen
    \param  callbackRoutine - Callback routine invoked to receive Packet Coalescing Filter Match Count
    \param  callbackContext - Cookie to be passed back during callback
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetFilterMatchCount(tHalHandle hHal,
                                   FilterMatchCountCallback callbackRoutine,
                                   void *callbackContext,
                                   tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_ReceiveFilterClearFilter
    \brief  API to clear Receive Packet Filter
    \param  hHal - The handle returned by macOpen.
    \param  pRcvFltPktClearParam - Receive Packet Filter Clear parameter
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ReceiveFilterClearFilter(tHalHandle hHal,
                                        tpSirRcvFltPktClearParam pRcvFltPktClearParam,
                                        tANI_U8  sessionId);
#endif // WLAN_FEATURE_PACKET_FILTERING
/* ---------------------------------------------------------------------------

    \fn sme_IsChannelValid
    \brief To check if the channel is valid for currently established domain
    This is a synchronous API.

    \param hHal - The handle returned by macOpen.
    \param channel - channel to verify

    \return TRUE/FALSE, TRUE if channel is valid

  -------------------------------------------------------------------------------*/
tANI_BOOLEAN sme_IsChannelValid(tHalHandle hHal, tANI_U8 channel);

/* ---------------------------------------------------------------------------
    \fn sme_SetFreqBand
    \brief  Used to set frequency band.
    \param  hHal
    \sessionId Session identifier
    \eBand  band value to be configured
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetFreqBand(tHalHandle hHal, tANI_U8 sessionId, eCsrBand eBand);

/* ---------------------------------------------------------------------------
    \fn sme_GetFreqBand
    \brief  Used to get the current band settings.
    \param  hHal
    \pBand  pointer to hold the current band value
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_GetFreqBand(tHalHandle hHal, eCsrBand *pBand);

/* ---------------------------------------------------------------------------

    \fn sme_SetTxPerTracking
    \brief  Set Tx PER tracking configuration parameters
    \param  hHal - The handle returned by macOpen.
    \param  pTxPerTrackingParam - Tx PER configuration parameters
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetTxPerTracking (
   tHalHandle hHal,
   void (*pCallbackfn) (void *pCallbackContext),
   void *pCallbackContext,
   tpSirTxPerTrackingParam pTxPerTrackingParam);

#ifdef WLAN_FEATURE_GTK_OFFLOAD
/* ---------------------------------------------------------------------------
    \fn sme_SetGTKOffload
    \brief  API to set GTK offload feature.
    \param  hHal - The handle returned by macOpen.
    \param  pRequest -  Pointer to the GTK offload request.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetGTKOffload (tHalHandle hHal, tpSirGtkOffloadParams pRequest, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_GetGTKOffload
    \brief  API to get GTK offload information.
    \param  hHal - The handle returned by macOpen.
    \param  pRequest -  Pointer to the GTK offload response.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_GetGTKOffload (tHalHandle hHal, GTKOffloadGetInfoCallback callbackRoutine,
                                    void *callbackContext, tANI_U8 sessionId);
#endif // WLAN_FEATURE_GTK_OFFLOAD

#ifdef WLAN_WAKEUP_EVENTS
eHalStatus sme_WakeReasonIndCallback (tHalHandle hHal, void* pMsg);
#endif // WLAN_WAKEUP_EVENTS

/* ---------------------------------------------------------------------------
    \fn sme_SetTxPerTracking
    \brief  Set Tx PER tracking configuration parameters
    \param  hHal - The handle returned by macOpen.
    \param  pTxPerTrackingParam - Tx PER configuration parameters
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetTxPerTracking (
   tHalHandle hHal,
   void (*pCallbackfn) (void *pCallbackContext),
   void *pCallbackContext,
   tpSirTxPerTrackingParam pTxPerTrackingParam);


//return frequency for a particular channel
tANI_U16 sme_ChnToFreq(tANI_U8 chanNum);

tANI_BOOLEAN sme_IsChannelValid(tHalHandle hHal, tANI_U8 channel);

/* ---------------------------------------------------------------------------
    \fn sme_SetMaxTxPower
    \brief  Used to set the Maximum Transmit Power dynamically. Note: this
    setting will not persist over reboots
    \param  hHal
    \param pBssid  BSSID to set the power cap for
    \param pBssid  pSelfMacAddress self MAC Address
    \param pBssid  power to set in dB
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetMaxTxPower(tHalHandle hHal, tSirMacAddr pBssid,
                             tSirMacAddr pSelfMacAddress, v_S7_t dB);

/* ---------------------------------------------------------------------------
    \fn sme_SetMaxTxPowerPerBand
    \brief  Used to set the Maximum Transmit Power for
    specific band dynamically. Note: this setting will not persist over reboots
    \param band
    \param power to set in dB
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetMaxTxPowerPerBand(eCsrBand band, v_S7_t db);

/* ---------------------------------------------------------------------------
    \fn sme_SetTxPower
    \brief Set Transmit Power dynamically.
    \param  hHal
    \param sessionId  Target Session ID
    \param pBSSId BSSId
    \param dev_mode device mode
    \param power  power to set in dBm
    \- return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetTxPower(tHalHandle hHal, v_U8_t sessionId,
                          tSirMacAddr pBSSId,
                          tVOS_CON_MODE dev_mode, int power);


/* ---------------------------------------------------------------------------

    \fn sme_SetCustomMacAddr

    \brief Set the customer Mac Address.

    \param customMacAddr  customer MAC Address
    \- return eHalStatus

  ---------------------------------------------------------------------------*/
eHalStatus sme_SetCustomMacAddr(tSirMacAddr customMacAddr);

/* ---------------------------------------------------------------------------

    \fn sme_HideSSID

    \brief Enable/Disables hidden SSID dynamically. Note: this setting will
    not persist over reboots.

    \param hHal
    \param sessionId
    \param ssidHidden 0 - Broadcast SSID, 1 - Disable broadcast SSID
    \- return eHalStatus

  -------------------------------------------------------------------------------*/
eHalStatus sme_HideSSID(tHalHandle hHal, v_U8_t sessionId, v_U8_t ssidHidden);

/* ---------------------------------------------------------------------------

    \fn sme_SetTmLevel
    \brief  Set Thermal Mitigation Level to RIVA
    \param  hHal - The handle returned by macOpen.
    \param  newTMLevel - new Thermal Mitigation Level
    \param  tmMode - Thermal Mitigation handle mode, default 0
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetTmLevel(tHalHandle hHal, v_U16_t newTMLevel, v_U16_t tmMode);

/*---------------------------------------------------------------------------

  \brief sme_featureCapsExchange() - SME interface to exchange capabilities between
  Host and FW.

  \param  hHal - HAL handle for device

  \return NONE

---------------------------------------------------------------------------*/
void sme_featureCapsExchange(tHalHandle hHal);

/*---------------------------------------------------------------------------

  \brief sme_disableActiveModeOffload() - SME interface to disable Active
                                  mode Offload capability between Host and FW.

  \param  hHal - HAL handle for device

  \return NONE

---------------------------------------------------------------------------*/
void sme_disableFeatureCapablity(tANI_U8 feature_index);

/*---------------------------------------------------------------------------

  \brief sme_GetDefaultCountryCodeFrmNv() - SME interface to get the default
         country code
  Host and FW.

  \param  hHal - HAL handle for device
  \param  pCountry - pointer to country code

  \return Success or failure

  ---------------------------------------------------------------------------*/
eHalStatus sme_GetDefaultCountryCodeFrmNv(tHalHandle hHal, tANI_U8 *pCountry);

/*---------------------------------------------------------------------------

  \brief sme_GetCurrentCountryCode() - SME interface to get the current operating
          country code.

  \param  hHal - HAL handle for device
  \param  pCountry - pointer to country code

  \return Success or failure

  ---------------------------------------------------------------------------*/
eHalStatus sme_GetCurrentCountryCode(tHalHandle hHal, tANI_U8 *pCountry);

/* ---------------------------------------------------------------------------
    \fn sme_transportDebug
    \brief  Dynamically monitoring Transport channels
            Private IOCTL will query transport channel status if driver loaded
    \param  hHal Upper MAC context
    \param  displaySnapshot Display transport channel snapshot option
    \param  toggleStallDetect Enable stall detect feature
                              This feature will take effect to data performance
                              Not integrate till fully verification
    \- return NONE
    -------------------------------------------------------------------------*/
void sme_transportDebug(tHalHandle hHal, v_BOOL_t displaySnapshot, v_BOOL_t toggleStallDetect);

/* ---------------------------------------------------------------------------
    \fn     sme_ResetPowerValuesFor5G
    \brief  Reset the power values for 5G band with NV power values.
    \param  hHal - HAL handle for device
    \- return NONE
    -------------------------------------------------------------------------*/
void sme_ResetPowerValuesFor5G (tHalHandle hHal);

#if  defined (WLAN_FEATURE_VOWIFI_11R) || defined (FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_LFR)
/* ---------------------------------------------------------------------------
    \fn sme_UpdateRoamPrefer5GHz
    \brief  enable/disable Roam prefer 5G runtime option
            This function is called through dynamic setConfig callback function
            to configure the Roam prefer 5G runtime option
    \param  hHal - HAL handle for device
    \param  nRoamPrefer5GHz Enable/Disable Roam prefer 5G runtime option
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_UpdateRoamPrefer5GHz(tHalHandle hHal, v_BOOL_t nRoamPrefer5GHz);

/* ---------------------------------------------------------------------------
    \fn sme_setRoamIntraBand
    \brief  enable/disable Intra band roaming
            This function is called through dynamic setConfig callback function
            to configure the intra band roaming
    \param  hHal - HAL handle for device
    \param  nRoamIntraBand Enable/Disable Intra band roaming
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_setRoamIntraBand(tHalHandle hHal, const v_BOOL_t nRoamIntraBand);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateRoamScanNProbes
    \brief  function to update roam scan N probes
            This function is called through dynamic setConfig callback function
            to update roam scan N probes
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nProbes number of probe requests to be sent out
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_UpdateRoamScanNProbes(tHalHandle hHal, tANI_U8 sessionId,
                                     const v_U8_t nProbes);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateRoamScanHomeAwayTime
    \brief  function to update roam scan Home away time
            This function is called through dynamic setConfig callback function
            to update roam scan home away time
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nRoamScanAwayTime Scan home away time
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_UpdateRoamScanHomeAwayTime(tHalHandle hHal, tANI_U8 sessionId,
                                          const v_U16_t nRoamScanHomeAwayTime,
                                          const eAniBoolean bSendOffloadCmd);

/* ---------------------------------------------------------------------------
    \fn sme_getRoamIntraBand
    \brief  get Intra band roaming
    \param  hHal - HAL handle for device
    \- return Success or failure
    -------------------------------------------------------------------------*/
v_BOOL_t sme_getRoamIntraBand(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_getRoamScanNProbes
    \brief  get N Probes
    \param  hHal - HAL handle for device
    \- return Success or failure
    -------------------------------------------------------------------------*/
v_U8_t sme_getRoamScanNProbes(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_getRoamScanHomeAwayTime
    \brief  get Roam scan home away time
    \param  hHal - HAL handle for device
    \- return Success or failure
    -------------------------------------------------------------------------*/
v_U16_t sme_getRoamScanHomeAwayTime(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateImmediateRoamRssiDiff
    \brief  Update nImmediateRoamRssiDiff
            This function is called through dynamic setConfig callback function
            to configure nImmediateRoamRssiDiff
            Usage: adb shell iwpriv wlan0 setConfig gImmediateRoamRssiDiff=[0 .. 125]
    \param  hHal - HAL handle for device
    \param  nImmediateRoamRssiDiff - minimum rssi difference between potential
            candidate and current AP.
    \param  sessionId - Session identifier
    \- return Success or failure
    -------------------------------------------------------------------------*/

eHalStatus sme_UpdateImmediateRoamRssiDiff(tHalHandle hHal,
                                           v_U8_t nImmediateRoamRssiDiff,
                                           tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateRoamRssiDiff
    \brief  Update RoamRssiDiff
            This function is called through dynamic setConfig callback function
            to configure RoamRssiDiff
            Usage: adb shell iwpriv wlan0 setConfig RoamRssiDiff=[0 .. 125]
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  RoamRssiDiff - minimum rssi difference between potential
            candidate and current AP.
    \- return Success or failure
    -------------------------------------------------------------------------*/

eHalStatus sme_UpdateRoamRssiDiff(tHalHandle hHal, tANI_U8 sessionId,
                                  v_U8_t RoamRssiDiff);

/*--------------------------------------------------------------------------
  \brief sme_UpdateFastTransitionEnabled() - enable/disable Fast Transition support at runtime
  It is used at in the REG_DYNAMIC_VARIABLE macro definition of
  isFastTransitionEnabled.
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update isFastTransitionEnabled config successfully.
          Other status means SME is failed to update isFastTransitionEnabled.
  \sa
  --------------------------------------------------------------------------*/

eHalStatus sme_UpdateFastTransitionEnabled(tHalHandle hHal,
        v_BOOL_t isFastTransitionEnabled);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateWESMode
    \brief  Update WESMode
            This function is called through dynamic setConfig callback function
            to configure isWESModeEnabled
    \param  hHal - HAL handle for device
    \param  isWESModeEnabled - Enable/Disable WES Mode
    \param  sessionId - Session identifier
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_UpdateWESMode(tHalHandle hHal, v_BOOL_t isWESModeEnabled,
                               tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_SetRoamScanControl
    \brief  Set roam scan control
            This function is called to set roam scan control
            if roam scan control is set to 0, roaming scan cache is cleared
            any value other than 0 is treated as invalid value
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \return eHAL_STATUS_SUCCESS - SME update config successfully.
          Other status means SME failure to update
    -------------------------------------------------------------------------*/
eHalStatus sme_SetRoamScanControl(tHalHandle hHal, tANI_U8 sessionId,
                                  v_BOOL_t roamScanControl);
#endif /* (WLAN_FEATURE_VOWIFI_11R) || (FEATURE_WLAN_ESE) || (FEATURE_WLAN_LFR) */

#ifdef FEATURE_WLAN_LFR
/*--------------------------------------------------------------------------
  \brief sme_UpdateIsFastRoamIniFeatureEnabled() - enable/disable LFR support at runtime
  It is used at in the REG_DYNAMIC_VARIABLE macro definition of
  isFastRoamIniFeatureEnabled.
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update isFastRoamIniFeatureEnabled config successfully.
          Other status means SME is failed to update isFastRoamIniFeatureEnabled.
  \sa
  --------------------------------------------------------------------------*/

eHalStatus sme_UpdateIsFastRoamIniFeatureEnabled(tHalHandle hHal,
                                                 tANI_U8 sessionId,
        const v_BOOL_t isFastRoamIniFeatureEnabled);

/*--------------------------------------------------------------------------
  \brief sme_UpdateIsMAWCIniFeatureEnabled() -
  Enable/disable LFR MAWC support at runtime
  It is used at in the REG_DYNAMIC_VARIABLE macro definition of
  isMAWCIniFeatureEnabled.
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update MAWCEnabled config successfully.
          Other status means SME is failed to update MAWCEnabled.
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_UpdateIsMAWCIniFeatureEnabled(tHalHandle hHal,
        const v_BOOL_t MAWCEnabled);

#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
/*--------------------------------------------------------------------------
  \brief sme_stopRoaming() - Stop roaming for a given sessionId
   This is a synchronous call
  \param hHal      - The handle returned by macOpen
  \param sessionId - sessionId
  \param reason - reason
  \return eHAL_STATUS_SUCCESS on success
          Other status on failure
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_stopRoaming(tHalHandle hHal, tANI_U8 sessionId,
                           tANI_U8 reason);

/*--------------------------------------------------------------------------
  \brief sme_startRoaming() - Start roaming for a given sessionId
   This is a synchronous call
  \param hHal      - The handle returned by macOpen
  \param sessionId - sessionId
  \param reason - reason
  \return eHAL_STATUS_SUCCESS on success
          Other status on failure
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_startRoaming(tHalHandle hHal, tANI_U8 sessionId,
                            tANI_U8 reason);

/*--------------------------------------------------------------------------
  \brief sme_UpdateEnableFastRoamInConcurrency() - enable/disable LFR if
  Concurrent session exists
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS
          Other status means SME is failed
  \sa
  --------------------------------------------------------------------------*/

eHalStatus sme_UpdateEnableFastRoamInConcurrency(tHalHandle hHal,
         v_BOOL_t bFastRoamInConIniFeatureEnabled);
#endif
#endif /* FEATURE_WLAN_LFR */

#ifdef FEATURE_WLAN_ESE
/*--------------------------------------------------------------------------
  \brief sme_UpdateIsEseFeatureEnabled() - enable/disable ESE support at runtime
  It is used at in the REG_DYNAMIC_VARIABLE macro definition of
  isEseIniFeatureEnabled.
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update isEseIniFeatureEnabled config successfully.
          Other status means SME is failed to update isEseIniFeatureEnabled.
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_UpdateIsEseFeatureEnabled(tHalHandle hHal, tANI_U8 sessionId,
                                       const v_BOOL_t isEseIniFeatureEnabled);

#endif /* FEATURE_WLAN_ESE */

/*--------------------------------------------------------------------------
  \brief sme_UpdateConfigFwRssiMonitoring() - enable/disable firmware
                                              RSSI Monitoring at runtime
  It is used at in the REG_DYNAMIC_VARIABLE macro definition of
  fEnableFwRssiMonitoring.
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update fEnableFwRssiMonitoring config successfully.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/

eHalStatus sme_UpdateConfigFwRssiMonitoring(tHalHandle hHal,
        v_BOOL_t fEnableFwRssiMonitoring);

#ifdef WLAN_FEATURE_NEIGHBOR_ROAMING
/* ---------------------------------------------------------------------------
    \fn     sme_SetRoamRescanRssiDiff
    \brief  Update Roam Rescan RSSI diff
            This function is called through dynamic setConfig callback function
            to configure nRoamRescanRssiDiff
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nRoamRescanRssiDiff - Roam Rescan Rssi Diff
    \return eHAL_STATUS_SUCCESS - SME update nRoamRescanRssiDiff config
            successfully.
            else SME is failed to update nRoamRescanRssiDiff
    -------------------------------------------------------------------------*/

eHalStatus sme_SetRoamRescanRssiDiff(tHalHandle hHal,
                                     tANI_U8 sessionId,
                                     const v_U8_t nRoamRescanRssiDiff);

/*--------------------------------------------------------------------------
  \fn    sme_GetRoamRescanRssiDiff
  \brief gets Roam Rescan RSSI diff
         This is a synchronous call
  \param hHal - The handle returned by macOpen
  \return v_U8_t - nRoamRescanRssiDiff
  \sa
  --------------------------------------------------------------------------*/
v_U8_t sme_GetRoamRescanRssiDiff(tHalHandle hHal);


/* ---------------------------------------------------------------------------
    \fn     sme_SetRoamOpportunisticScanThresholdDiff
    \brief  Update Opportunistic Scan threshold diff
            This function is called through dynamic setConfig callback function
            to configure  nOpportunisticThresholdDiff
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nOpportunisticThresholdDiff - Opportunistic Scan threshold diff
    \return eHAL_STATUS_SUCCESS - SME update nOpportunisticThresholdDiff config
            successfully.
            else SME is failed to update nOpportunisticThresholdDiff.
    -------------------------------------------------------------------------*/

eHalStatus sme_SetRoamOpportunisticScanThresholdDiff(tHalHandle hHal,
                            tANI_U8 sessionId,
                            const v_U8_t nOpportunisticThresholdDiff);

/*--------------------------------------------------------------------------
  \fn    sme_GetRoamOpportunisticScanThresholdDiff()
  \brief gets Opportunistic Scan threshold diff
         This is a synchronous call
  \param hHal - The handle returned by macOpen
  \return v_U8_t - nOpportunisticThresholdDiff
  \sa
  --------------------------------------------------------------------------*/
v_U8_t sme_GetRoamOpportunisticScanThresholdDiff(tHalHandle hHal);

/*--------------------------------------------------------------------------
  \brief sme_setNeighborLookupRssiThreshold() - update neighbor lookup rssi threshold
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_setNeighborLookupRssiThreshold(tHalHandle hHal,
                                              tANI_U8 sessionId,
                                           v_U8_t neighborLookupRssiThreshold);

/*--------------------------------------------------------------------------
  \brief sme_set_delay_before_vdev_stop() - update delay before vdev stop
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_set_delay_before_vdev_stop(tHalHandle hHal,
                                         tANI_U8 sessionId,
                                         v_U8_t delay_before_vdev_stop);

/*--------------------------------------------------------------------------
  \brief sme_setNeighborReassocRssiThreshold() - update neighbor reassoc rssi threshold
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_setNeighborReassocRssiThreshold(tHalHandle hHal,
                                               tANI_U8 sessionId,
        v_U8_t neighborReassocRssiThreshold);

/*--------------------------------------------------------------------------
  \brief sme_getNeighborLookupRssiThreshold() - get neighbor lookup rssi threshold
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
v_U8_t sme_getNeighborLookupRssiThreshold(tHalHandle hHal);

/*--------------------------------------------------------------------------
  \brief sme_setNeighborScanRefreshPeriod() - set neighbor scan results refresh period
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_setNeighborScanRefreshPeriod(tHalHandle hHal,
                                            tANI_U8 sessionId,
        v_U16_t neighborScanResultsRefreshPeriod);

/*--------------------------------------------------------------------------
  \brief sme_getNeighborScanRefreshPeriod() - get neighbor scan results refresh period
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
v_U16_t sme_getNeighborScanRefreshPeriod(tHalHandle hHal);

/*--------------------------------------------------------------------------
  \brief sme_getEmptyScanRefreshPeriod() - get empty scan refresh period
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
v_U16_t sme_getEmptyScanRefreshPeriod(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateEmptyScanRefreshPeriod
    \brief  Update nEmptyScanRefreshPeriod
            This function is called through dynamic setConfig callback function
            to configure nEmptyScanRefreshPeriod
            Usage: adb shell iwpriv wlan0 setConfig nEmptyScanRefreshPeriod=[0 .. 60]
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nEmptyScanRefreshPeriod - scan period following empty scan results.
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_UpdateEmptyScanRefreshPeriod(tHalHandle hHal,tANI_U8 sessionId,
                                            v_U16_t nEmptyScanRefreshPeriod);

/* ---------------------------------------------------------------------------
    \fn sme_setNeighborScanMinChanTime
    \brief  Update nNeighborScanMinChanTime
            This function is called through dynamic setConfig callback function
            to configure gNeighborScanChannelMinTime
            Usage: adb shell iwpriv wlan0 setConfig gNeighborScanChannelMinTime=[0 .. 60]
    \param  hHal - HAL handle for device
    \param  nNeighborScanMinChanTime - Channel minimum dwell time
    \param  sessionId - Session identifier
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_setNeighborScanMinChanTime(tHalHandle hHal,
                                        const v_U16_t nNeighborScanMinChanTime,
                                        tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_setNeighborScanMaxChanTime
    \brief  Update nNeighborScanMaxChanTime
            This function is called through dynamic setConfig callback function
            to configure gNeighborScanChannelMaxTime
            Usage: adb shell iwpriv wlan0 setConfig gNeighborScanChannelMaxTime=[0 .. 60]
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nNeighborScanMinChanTime - Channel maximum dwell time
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_setNeighborScanMaxChanTime(tHalHandle hHal, tANI_U8 sessionId,
                                       const v_U16_t nNeighborScanMaxChanTime);

/* ---------------------------------------------------------------------------
    \fn sme_getNeighborScanMinChanTime
    \brief  get neighbor scan min channel time
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session identifier
    \return v_U16_t - channel min time value
    -------------------------------------------------------------------------*/
v_U16_t sme_getNeighborScanMinChanTime(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_getNeighborRoamState
    \brief  get neighbor roam state
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session Identifier
    \return v_U32_t - neighbor roam state
    -------------------------------------------------------------------------*/
v_U32_t sme_getNeighborRoamState(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_getCurrentRoamState
    \brief  get current roam state
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session Identifier
    \return v_U32_t - current roam state
    -------------------------------------------------------------------------*/
v_U32_t sme_getCurrentRoamState(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_getCurrentRoamSubState
    \brief  get neighbor roam sub state
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session Identifier
    \return v_U32_t - current roam sub state
    -------------------------------------------------------------------------*/
v_U32_t sme_getCurrentRoamSubState(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_getLimSmeState
    \brief  get Lim Sme state
    \param hHal - The handle returned by macOpen.
    \return v_U32_t - Lim Sme state
    -------------------------------------------------------------------------*/
v_U32_t sme_getLimSmeState(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_getLimMlmState
    \brief  get Lim Mlm state
    \param hHal - The handle returned by macOpen.
    \return v_U32_t - Lim Mlm state
    -------------------------------------------------------------------------*/
v_U32_t sme_getLimMlmState(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_IsLimSessionValid
    \brief  is Lim session valid
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session Identifier
    \return v_BOOL_t - true or false
    -------------------------------------------------------------------------*/
v_BOOL_t sme_IsLimSessionValid(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_getLimSmeSessionState
    \brief  get Lim Sme session state
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session Identifier
    \return v_U32_t - Lim Sme session state
    -------------------------------------------------------------------------*/
v_U32_t sme_getLimSmeSessionState(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_getLimMlmSessionState
    \brief  get Lim Mlm session state
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session Identifier
    \return v_U32_t - Lim Mlm session state
    -------------------------------------------------------------------------*/
v_U32_t sme_getLimMlmSessionState(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_getNeighborScanMaxChanTime
    \brief  get neighbor scan max channel time
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session identifier
    \return v_U16_t - channel max time value
    -------------------------------------------------------------------------*/
v_U16_t sme_getNeighborScanMaxChanTime(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_setNeighborScanPeriod
    \brief  Update nNeighborScanPeriod
            This function is called through dynamic setConfig callback function
            to configure nNeighborScanPeriod
            Usage: adb shell iwpriv wlan0 setConfig nNeighborScanPeriod=[0 .. 60]
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nNeighborScanPeriod - neighbor scan period
    \- return Success or failure
    -------------------------------------------------------------------------*/
eHalStatus sme_setNeighborScanPeriod(tHalHandle hHal, tANI_U8 sessionId,
                                     const v_U16_t nNeighborScanPeriod);

/* ---------------------------------------------------------------------------
    \fn sme_getNeighborScanPeriod
    \brief  get neighbor scan period
    \param hHal - The handle returned by macOpen.
    \param  sessionId - Session identifier
    \return v_U16_t - neighbor scan period
    -------------------------------------------------------------------------*/
v_U16_t sme_getNeighborScanPeriod(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn     sme_SetRoamBmissFirstBcnt
    \brief  Update Roam count for first beacon miss
            This function is called through dynamic setConfig callback function
            to configure nRoamBmissFirstBcnt
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nRoamBmissFirstBcnt - Roam first bmiss count
    \return eHAL_STATUS_SUCCESS - SME update nRoamBmissFirstBcnt
            successfully.
            else SME is failed to update nRoamBmissFirstBcnt
    -------------------------------------------------------------------------*/

eHalStatus sme_SetRoamBmissFirstBcnt(tHalHandle hHal,
                                     tANI_U8 sessionId,
                                     const v_U8_t nRoamBmissFirstBcnt);

/*--------------------------------------------------------------------------
  \fn    sme_GetRoamBmissFirstBcnt
  \brief gets Roam count for first beacon miss
         This is a synchronous call
  \param hHal - The handle returned by macOpen
  \return v_U8_t - nRoamBmissFirstBcnt
  \sa
  --------------------------------------------------------------------------*/
v_U8_t sme_GetRoamBmissFirstBcnt(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn     sme_SetRoamBmissFinalBcnt
    \brief  Update Roam count for final beacon miss
            This function is called through dynamic setConfig callback function
            to configure nRoamBmissFinalBcnt
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nRoamBmissFinalBcnt - Roam final bmiss count
    \return eHAL_STATUS_SUCCESS - SME update nRoamBmissFinalBcnt
            successfully.
            else SME is failed to update nRoamBmissFinalBcnt
    -------------------------------------------------------------------------*/

eHalStatus sme_SetRoamBmissFinalBcnt(tHalHandle hHal, tANI_U8 sessionId,
                                     const v_U8_t nRoamBmissFinalBcnt);

/*--------------------------------------------------------------------------
  \fn    sme_GetRoamBmissFinalBcnt
  \brief gets Roam count for final beacon miss
         This is a synchronous call
  \param hHal - The handle returned by macOpen
  \return v_U8_t - nRoamBmissFinalBcnt
  \sa
  --------------------------------------------------------------------------*/
v_U8_t sme_GetRoamBmissFinalBcnt(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn     sme_SetRoamBeaconRssiWeight
    \brief  Update Roam beacon rssi weight
            This function is called through dynamic setConfig callback function
            to configure nRoamBeaconRssiWeight
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  nRoamBeaconRssiWeight - Roam beacon rssi weight
    \return eHAL_STATUS_SUCCESS - SME update nRoamBeaconRssiWeight config
            successfully.
            else SME is failed to update nRoamBeaconRssiWeight
    -------------------------------------------------------------------------*/

eHalStatus sme_SetRoamBeaconRssiWeight(tHalHandle hHal, tANI_U8 sessionId,
                                     const v_U8_t nRoamBeaconRssiWeight);

/*--------------------------------------------------------------------------
  \fn    sme_GetRoamBeaconRssiWeight
  \brief gets Roam beacon rssi weight
         This is a synchronous call
  \param hHal - The handle returned by macOpen
  \return v_U8_t - nRoamBeaconRssiWeight
  \sa
  --------------------------------------------------------------------------*/
v_U8_t sme_GetRoamBeaconRssiWeight(tHalHandle hHal);
#endif

#if  defined (WLAN_FEATURE_VOWIFI_11R) || defined (FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_LFR)
/*--------------------------------------------------------------------------
  \brief sme_getRoamRssiDiff() - get Roam rssi diff
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
v_U8_t sme_getRoamRssiDiff(tHalHandle hHal);

/*--------------------------------------------------------------------------
  \brief sme_ChangeRoamScanChannelList() - Change roam scan channel list
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_ChangeRoamScanChannelList(tHalHandle hHal, tANI_U8 sessionId,
                                         tANI_U8 *pChannelList,
                                         tANI_U8 numChannels);

#ifdef FEATURE_WLAN_ESE_UPLOAD
/*--------------------------------------------------------------------------
  \brief sme_SetEseRoamScanChannelList() - set ese roam scan channel list
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_SetEseRoamScanChannelList(tHalHandle hHal, tANI_U8 sessionId,
                                                   tANI_U8 *pChannelList,
                                                   tANI_U8 numChannels);
#endif

/*--------------------------------------------------------------------------
  \brief sme_getRoamScanChannelList() - get roam scan channel list
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param pChannelList - Output channel list
  \param pNumChannels - Output number of channels
  \param  sessionId - Session identifier
  \return eHAL_STATUS_SUCCESS - SME update config successful.
          Other status means SME is failed to update
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_getRoamScanChannelList(tHalHandle hHal, tANI_U8 *pChannelList,
                                      tANI_U8 *pNumChannels, tANI_U8 sessionId);

/*--------------------------------------------------------------------------
  \brief sme_getIsEseFeatureEnabled() - get ESE feature enabled or not
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return TRUE (1) - if the ESE feature is enabled
          FALSE (0) - if feature is disabled (compile or runtime)
  \sa
  --------------------------------------------------------------------------*/
tANI_BOOLEAN sme_getIsEseFeatureEnabled(tHalHandle hHal);

/*--------------------------------------------------------------------------
  \brief sme_getWESMode() - getWES Mode
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return v_U8_t - WES Mode Enabled(1)/Disabled(0)
  \sa
  --------------------------------------------------------------------------*/
v_BOOL_t sme_GetWESMode(tHalHandle hHal);

/*--------------------------------------------------------------------------
  \brief sme_GetRoamScanControl() - get scan control
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return v_BOOL_t - Enabled(1)/Disabled(0)
  \sa
  --------------------------------------------------------------------------*/
v_BOOL_t sme_GetRoamScanControl(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateEmptyScanRefreshPeriod
    \brief  Update nnEmptyScanRefreshPeriod
            This function is called through dynamic setConfig callback function
            to configure nnEmptyScanRefreshPeriod
            Usage: adb shell iwpriv wlan0 setConfig nEmptyScanRefreshPeriod=[0 .. 60]
    \param  hHal - HAL handle for device
    \param  nEmptyScanRefreshPeriod - scan period following empty scan results.
    \- return Success or failure
    -------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------
  \brief sme_getIsLfrFeatureEnabled() - get LFR feature enabled or not
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return TRUE (1) - if the feature is enabled
          FALSE (0) - if feature is disabled (compile or runtime)
  \sa
  --------------------------------------------------------------------------*/
tANI_BOOLEAN sme_getIsLfrFeatureEnabled(tHalHandle hHal);

/*--------------------------------------------------------------------------
  \brief sme_getIsFtFeatureEnabled() - get FT feature enabled or not
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return TRUE (1) - if the feature is enabled
          FALSE (0) - if feature is disabled (compile or runtime)
  \sa
  --------------------------------------------------------------------------*/
tANI_BOOLEAN sme_getIsFtFeatureEnabled(tHalHandle hHal);

#endif

#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
/*--------------------------------------------------------------------------
  \brief sme_UpdateRoamScanOffloadEnabled() - enable/disable roam scan offload
                                              feature
  It is used at in the REG_DYNAMIC_VARIABLE macro definition of
  gRoamScanOffloadEnabled.
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return eHAL_STATUS_SUCCESS - SME update config successfully.
          Other status means SME is failed to update.
  \sa
  --------------------------------------------------------------------------*/

eHalStatus sme_UpdateRoamScanOffloadEnabled(tHalHandle hHal, v_BOOL_t nRoamScanOffloadEnabled);
#endif


/* ---------------------------------------------------------------------------
    \fn sme_IsFeatureSupportedByFW
    \brief  Check if an feature is enabled by FW

    \param  feattEnumValue - Enumeration value of the feature to be checked.
                A value from enum placeHolderInCapBitmap

    \- return 1/0 (TRUE/FALSE)
    -------------------------------------------------------------------------*/
tANI_U8 sme_IsFeatureSupportedByFW(tANI_U8 featEnumValue);
#ifdef FEATURE_WLAN_TDLS

/* ---------------------------------------------------------------------------
    \fn sme_SendTdlsLinkEstablishParams
    \brief  API to send TDLS Link Establishment Parameters.

    \param  peerMac - peer's Mac Address.
    \param  tdlsLinkEstablishParams - TDLS Peer Link Establishment Parameters
    \- return VOS_STATUS_SUCCES
    -------------------------------------------------------------------------*/

VOS_STATUS sme_SendTdlsLinkEstablishParams(tHalHandle hHal,
                                           tANI_U8 sessionId,
                                           const tSirMacAddr peerMac,
                                           tCsrTdlsLinkEstablishParams *tdlsLinkEstablishParams);

/* ---------------------------------------------------------------------------
    \fn sme_SendTdlsMgmtFrame
    \brief  API to send TDLS management frames.

    \param  peerMac - peer's Mac Address.
    \param frame_type - Type of TDLS mgmt frame to be sent.
    \param dialog - dialog token used in the frame.
    \param status - status to be included in the frame.
    \param peerCapability - peerCapability to be included in the frame.
    \param buf - additional IEs to be included
    \param len - length of additional Ies
    \param responder - Tdls request type
    \- return VOS_STATUS_SUCCES
    -------------------------------------------------------------------------*/
VOS_STATUS sme_SendTdlsMgmtFrame(tHalHandle hHal, tANI_U8 sessionId,
                                 const tSirMacAddr peerMac, tANI_U8 frame_type,
                                 tANI_U8 dialog, tANI_U16 status,
                                 tANI_U32 peerCapability, tANI_U8 *buf,
                                 tANI_U8 len, tANI_U8 responder);
/* ---------------------------------------------------------------------------
    \fn sme_ChangeTdlsPeerSta
    \brief  API to Update TDLS peer sta parameters.

    \param  peerMac - peer's Mac Address.
    \param  staParams - Peer Station Parameters.
    \- return VOS_STATUS_SUCCES
    -------------------------------------------------------------------------*/
VOS_STATUS sme_ChangeTdlsPeerSta(tHalHandle hHal, tANI_U8 sessionId,
                                 const tSirMacAddr peerMac,
                                 tCsrStaParams *pstaParams);
/* ---------------------------------------------------------------------------
    \fn sme_AddTdlsPeerSta
    \brief  API to Add TDLS peer sta entry.

    \param  peerMac - peer's Mac Address.
    \- return VOS_STATUS_SUCCES
    -------------------------------------------------------------------------*/
VOS_STATUS sme_AddTdlsPeerSta(tHalHandle hHal,
                              tANI_U8 sessionId,
                              const tSirMacAddr peerMac);
/* ---------------------------------------------------------------------------
    \fn sme_DeleteTdlsPeerSta
    \brief  API to Delete TDLS peer sta entry.

    \param  peerMac - peer's Mac Address.
    \- return VOS_STATUS_SUCCES
    -------------------------------------------------------------------------*/
VOS_STATUS sme_DeleteTdlsPeerSta(tHalHandle hHal, tANI_U8 sessionId,
                                 const tSirMacAddr peerMac);
/* ---------------------------------------------------------------------------
    \fn sme_SetTdlsPowerSaveProhibited
    \API to set/reset the isTdlsPowerSaveProhibited.

    \- return void
    -------------------------------------------------------------------------*/
void sme_SetTdlsPowerSaveProhibited(tHalHandle hHal, tANI_U32 sessionId,
                                            v_BOOL_t val);

/* ---------------------------------------------------------------------------
    \fn  sme_SendTdlsChanSwitchReq
    \API to set tdls channel switch parameters.

    \- return void
    -------------------------------------------------------------------------*/
eHalStatus sme_SendTdlsChanSwitchReq(tHalHandle hHal,
                                     tSmeTdlsChanSwitchParams *chSwitchParams);
#endif
/* ---------------------------------------------------------------------------
    \fn sme_IsPmcBmps
    \brief  API to Check if PMC state is BMPS.

    \- return v_BOOL_t
    -------------------------------------------------------------------------*/
v_BOOL_t sme_IsPmcBmps(tHalHandle hHal);

eHalStatus sme_UpdateDfsSetting(tHalHandle hHal, tANI_U8 fUpdateEnableDFSChnlScan);

/*
 * SME API to enable/disable WLAN driver initiated SSR
 */
void sme_UpdateEnableSSR(tHalHandle hHal, tANI_BOOLEAN enableSSR);

/* ---------------------------------------------------------------------------

    \fn sme_SetPhyMode

    \brief Changes the PhyMode.

    \param hHal - The handle returned by macOpen.

    \param phyMode new phyMode which is to set

    \return eHalStatus  SUCCESS.

  -------------------------------------------------------------------------------*/
eHalStatus sme_SetPhyMode(tHalHandle hHal, eCsrPhyMode phyMode);

/* ---------------------------------------------------------------------------

    \fn sme_GetPhyMode

    \brief gets current PhyMode.

    \param hHal - The handle returned by macOpen.

    \return eHalStatus PhyMode

  -------------------------------------------------------------------------------*/
eCsrPhyMode sme_GetPhyMode(tHalHandle hHal);

/*
 * SME API to determine the channel bonding mode
 */
eIniChanBondState sme_SelectCBMode(tHalHandle hHal, eCsrPhyMode eCsrPhyMode,
                            uint8_t channel, uint8_t ht_sec_dh,
                            uint16_t *vht_channel_width,
                            uint16_t ch_width_orig);

#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
/*--------------------------------------------------------------------------
  \brief sme_HandoffRequest() - a wrapper function to Request a handoff
  from CSR.
  This is a synchronous call
  \param hHal - The handle returned by macOpen
  \param  sessionId - Session identifier
  \param pHandoffInfo - info provided by HDD with the handoff request (namely:
  BSSID, channel etc.)
  \return eHAL_STATUS_SUCCESS - SME passed the request to CSR successfully.
          Other status means SME is failed to send the request.
  \sa
  --------------------------------------------------------------------------*/

eHalStatus sme_HandoffRequest(tHalHandle hHal, tANI_U8 sessionId,
                              tCsrHandoffRequest *pHandoffInfo);
#endif
/*--------------------------------------------------------------------------
  \brief sme_isSta_p2p_clientConnected() - a wrapper function to check if there
                                           is any connected session .
  This is a synchronous call
  \param hHal - The handle returned by macOpen
  \return VOS_STATUS - SME passed the request to CSR successfully.
          Other status means SME is failed to send the request.
  \sa
  --------------------------------------------------------------------------*/
VOS_STATUS sme_isSta_p2p_clientConnected(tHalHandle hHal);

#ifdef FEATURE_WLAN_LPHB
/* ---------------------------------------------------------------------------
    \fn sme_LPHBConfigReq
    \API to make configuration LPHB within FW.
    \param hHal - The handle returned by macOpen
    \param lphdReq - LPHB request argument by client
    \param pCallbackfn - LPHB timeout notification callback function pointer
    \- return Configuration message posting status, SUCCESS or Fail
    -------------------------------------------------------------------------*/
eHalStatus sme_LPHBConfigReq(
                      tHalHandle hHal,
                      tSirLPHBReq *lphdReq,
                      void (*pCallbackfn)(void *pHddCtx, tSirLPHBInd *indParam));
#endif /* FEATURE_WLAN_LPHB */

/* ---------------------------------------------------------------------------
    \fn sme_AddPeriodicTxPtrn
    \brief  API to Periodic TX Pattern Offload feature
    \param  hHal - The handle returned by macOpen
    \param  addPeriodicTxPtrnParams -  Pointer to the add pattern structure
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_AddPeriodicTxPtrn(tHalHandle hHal, tSirAddPeriodicTxPtrn
                                 *addPeriodicTxPtrnParams);

/* ---------------------------------------------------------------------------
    \fn sme_DelPeriodicTxPtrn
    \brief  API to Periodic TX Pattern Offload feature
    \param  hHal - The handle returned by macOpen
    \param  delPeriodicTxPtrnParams -  Pointer to the deleting pattern structure
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_DelPeriodicTxPtrn(tHalHandle hHal, tSirDelPeriodicTxPtrn
                                 *delPeriodicTxPtrnParams);
/*--------------------------------------------------------------------------
  \brief sme_enable_disable_split_scan() - a wrapper function to set the split
                                          scan parameter.
  This is a synchronous call
  \param hHal - The handle returned by macOpen
  \return None.
  \sa
  --------------------------------------------------------------------------*/
void sme_enable_disable_split_scan (tHalHandle hHal, tANI_U8 nNumStaChan,
                                    tANI_U8 nNumP2PChan);

/**
 * sme_enable_rmc() - enable RMC
 * @hHal: handle
 * @sessionId: session id
 *
 * Return: eHalStatus
 */
eHalStatus sme_enable_rmc(tHalHandle hHal, tANI_U32 sessionId);

/**
 * sme_disable_rmc() - disable RMC
 * @hHal: handle
 * @sessionId: session id
 *
 * Return: eHalStatus
 */
eHalStatus sme_disable_rmc(tHalHandle hHal, tANI_U32 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_SendRmcActionPeriod
    \brief  Used to send RMC action period param to fw
    \param  hHal
    \param  sessionId
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SendRmcActionPeriod(tHalHandle hHal, tANI_U32 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_SendRateUpdateInd
    \brief  API to Update rate
    \param  hHal - The handle returned by macOpen
    \param  rateUpdateParams - Pointer to rate update params
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SendRateUpdateInd(tHalHandle hHal, tSirRateUpdateInd *rateUpdateParams);

/* ---------------------------------------------------------------------------
    \fn sme_GetIBSSPeerInfo
    \brief  Used to disable RMC
    setting will not persist over reboots
    \param  hHal
    \param  ibssPeerInfoReq  multicast Group IP address
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_RequestIBSSPeerInfo(tHalHandle hHal, void *pUserData,
                                            pIbssPeerInfoCb peerInfoCbk,
                                            tANI_BOOLEAN allPeerInfoReqd,
                                            tANI_U8 staIdx);

/* ---------------------------------------------------------------------------
    \fn sme_SendCesiumEnableInd
    \brief  Used to send proprietary cesium enable indication to fw
    \param  hHal
    \param  sessionId
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SendCesiumEnableInd(tHalHandle hHal, tANI_U32 sessionId);

/*
 * sme API to trigger fast BSS roam to a given BSSID independent of RSSI
 * triggers
 * return status
*/
eHalStatus smeIssueFastRoamNeighborAPEvent (tHalHandle hHal,
                                            tANI_U8 *bssid,
                                            tSmeFastRoamTrigger fastRoamTrig,
                                            tANI_U8 sessionId);

eHalStatus sme_RoamDelPMKIDfromCache(tHalHandle hHal,
                                     tANI_U8 sessionId,
                                     const tANI_U8 *pBSSId,
                                     tANI_BOOLEAN flush_cache);

void smeGetCommandQStatus( tHalHandle hHal );

/*
 * SME API to enable/disable idle mode power save
 * This should be called only if power save offload
 * is enabled
 */
VOS_STATUS sme_SetIdlePowersaveConfig(v_PVOID_t vosContext, tANI_U32 value);
VOS_STATUS sme_notify_modem_power_state(tHalHandle hHal, tANI_U32 value);

eHalStatus sme_ConfigEnablePowerSave (tHalHandle hHal, tPmcPowerSavingMode psMode);
eHalStatus sme_ConfigDisablePowerSave (tHalHandle hHal, tPmcPowerSavingMode psMode);
eHalStatus sme_PsOffloadEnablePowerSave (tHalHandle hHal, tANI_U32 sessionId);
eHalStatus sme_PsOffloadDisablePowerSave (tHalHandle hHal, tANI_U32 sessionId);
eHalStatus sme_PsOffloadEnableDeferredPowerSave (tHalHandle hHal,
                                                 tANI_U32 sessionId,
                                                 tANI_BOOLEAN isReassoc);
eHalStatus sme_PsOffloadDisableDeferredPowerSave (tHalHandle hHal,
                                                  tANI_U32 sessionId);


/*SME API to convert convert the ini value to the ENUM used in csr and MAC*/
ePhyChanBondState sme_GetCBPhyStateFromCBIniValue(tANI_U32 cb_ini_value);

int sme_UpdateHTConfig(tHalHandle hHal, tANI_U8 sessionId, tANI_U16 htCapab,
                         int value);
tANI_S16 sme_GetHTConfig(tHalHandle hHal, tANI_U8 session_id, tANI_U16 ht_capab);

#ifdef QCA_HT_2040_COEX
VOS_STATUS sme_notify_ht2040_mode(tHalHandle hHal, tANI_U16 staId,
             v_MACADDR_t macAddrSTA, v_U8_t sessionId, tANI_U8 channel_type);
eHalStatus sme_SetHT2040Mode(tHalHandle hHal, tANI_U8 sessionId,
                              tANI_U8 channel_type, tANI_BOOLEAN obssEnabled);
#endif

eHalStatus sme_getRegInfo(tHalHandle hHal, tANI_U8 chanId,
                          tANI_U32 *regInfo1, tANI_U32 *regInfo2);

#ifdef FEATURE_WLAN_TDLS
eHalStatus sme_UpdateFwTdlsState(tHalHandle hHal, void *psmeTdlsParams,
                                 tANI_BOOLEAN useSmeLock);
eHalStatus sme_UpdateTdlsPeerState(tHalHandle hHal,
                                   tSmeTdlsPeerStateParams *pPeerStateParams);
#endif /* FEATURE_WLAN_TDLS */

#ifdef FEATURE_WLAN_CH_AVOID
/* ---------------------------------------------------------------------------
    \fn sme_AddChAvoidCallback
    \brief  Used to plug in callback function
        Which notify channel may not be used with SAP or P2PGO mode.
        Notification come from FW.
    \param  hHal
    \param  pCallbackfn : callback function pointer should be plugged in
    \- return eHalStatus
-------------------------------------------------------------------------*/
eHalStatus sme_AddChAvoidCallback
(
   tHalHandle hHal,
   void (*pCallbackfn)(void *hdd_context, void *indi_param)
);

/* ---------------------------------------------------------------------------
    \fn sme_ChAvoidUpdateReq
    \API to request channel avoidance update from FW.
    \param hHal - The handle returned by macOpen
    \param update_type - The udpate_type parameter of this request call
    \- return Configuration message posting status, SUCCESS or Fail
    -------------------------------------------------------------------------*/
eHalStatus sme_ChAvoidUpdateReq
(
   tHalHandle hHal
);
#endif /* FEATURE_WLAN_CH_AVOID */

#ifdef FEATURE_WLAN_AUTO_SHUTDOWN
/* ---------------------------------------------------------------------------
    \fn sme_auto_shutdown_cb
    \brief  Used to plug in callback function for receiving auto shutdown evt
    \param  hHal
    \param  pCallbackfn : callback function pointer should be plugged in
    \- return eHalStatus
-------------------------------------------------------------------------*/
eHalStatus sme_set_auto_shutdown_cb(tHalHandle hHal,
   void (*pCallbackfn)(void));

/* ---------------------------------------------------------------------------
    \fn sme_set_auto_shutdown_timer
    \API to set auto shutdown timer value in FW.
    \param hHal - The handle returned by macOpen
    \param timer_val - The auto shutdown timer value to be set
    \- return Configuration message posting status, SUCCESS or Fail
    -------------------------------------------------------------------------*/
eHalStatus sme_set_auto_shutdown_timer(tHalHandle hHal, tANI_U32 timer_value);
#endif

eHalStatus sme_RoamChannelChangeReq(tHalHandle hHal, tCsrBssid bssid,
                                    tANI_U32 cbMode, tCsrRoamProfile *pprofile);

eHalStatus sme_RoamStartBeaconReq( tHalHandle hHal,
                   tCsrBssid bssid, tANI_U8 dfsCacWaitStatus);
/* -------------------------------------------------------------------------
   \fn sme_RoamCsaIeRequest
   \brief API to request CSA IE transmission from PE
   \param hHal - The handle returned by macOpen
   \param pDfsCsaReq - CSA IE request
   \param bssid - SAP bssid
   \param ch_bandwidth - Channel offset
   \return eHalStatus
---------------------------------------------------------------------------*/
eHalStatus sme_RoamCsaIeRequest(tHalHandle hHal, tCsrBssid bssid,
                  tANI_U8 targetChannel, tANI_U8 csaIeReqd,
                  u_int8_t ch_bandwidth);

/* ---------------------------------------------------------------------------
    \fn sme_InitThermalInfo
    \brief  SME API to initialize the thermal mitigation parameters
    \param  hHal
    \param  thermalParam : thermal mitigation parameters
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_InitThermalInfo( tHalHandle hHal, tSmeThermalParams thermalParam );
/* ---------------------------------------------------------------------------
    \fn sme_InitThermalInfo
    \brief  SME API to set the thermal mitigation level
    \param  hHal
    \param  level : thermal mitigation level
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetThermalLevel( tHalHandle hHal, tANI_U8 level );
/* ---------------------------------------------------------------------------
   \fn sme_TxpowerLimit
   \brief SME API to set txpower limits
   \param hHal
   \param psmetx : power limits for 2g/5g
   \- return eHalStatus
  -------------------------------------------------------------------------*/
eHalStatus sme_TxpowerLimit( tHalHandle hHal, tSirTxPowerLimit *psmetx);
/* ---------------------------------------------------------------------------
   \fn sme_GetLinkSpeed
   \brief SME API to get the link speed for peermac
   \param hHal
   \param lsReq: peermac address to retrieve link speed
   \param plsContext: callback context
   \param pCallbackfn: callback fn with response (link speed)
   \- return eHalStatus
 -------------------------------------------------------------------------*/
eHalStatus sme_GetLinkSpeed(tHalHandle hHal,tSirLinkSpeedInfo *lsReq,void *plsContext,
                            void (*pCallbackfn)(tSirLinkSpeedInfo *indParam, void *pContext) );

eHalStatus sme_get_rssi(tHalHandle hal, struct sir_rssi_req req,
			void *context,
			void (*callbackfn)(struct sir_rssi_resp *param,
						void *pcontext));

/*----------------------------------------------------------------------------
 \fn  sme_ModifyAddIE
 \brief  This function sends msg to updates the additional IE buffers in PE
 \param  hHal - global structure
 \param  pModifyIE - pointer to tSirModifyIE structure
 \param  updateType - type of buffer
 \- return Success or failure
-----------------------------------------------------------------------------*/
eHalStatus sme_ModifyAddIE(tHalHandle hHal,
                           tSirModifyIE *pModifyIE,
                           eUpdateIEsType updateType);

/*----------------------------------------------------------------------------
 \fn  sme_UpdateAddIE
 \brief  This function sends msg to updates the additional IE buffers in PE
 \param  hHal - global structure
 \param  pUpdateIE - pointer to structure tSirUpdateIE
 \param  updateType - Type of buffer
 \- return Success or failure
-----------------------------------------------------------------------------*/
eHalStatus sme_UpdateAddIE(tHalHandle hHal,
                           tSirUpdateIE *pUpdateIE,
                           eUpdateIEsType updateType);

eHalStatus sme_UpdateConnectDebug(tHalHandle hHal, tANI_U32 set_value);
const char * sme_requestTypetoString(const v_U8_t requestType);
const char * sme_PmcStatetoString(const v_U8_t pmcState);
eHalStatus sme_ApDisableIntraBssFwd(tHalHandle hHal, tANI_U8 sessionId,
                                    tANI_BOOLEAN disablefwd);
tANI_U32 sme_GetChannelBondingMode5G(tHalHandle hHal);
tANI_U32 sme_GetChannelBondingMode24G(tHalHandle hHal);

#if defined(CONFIG_HL_SUPPORT) && defined(QCA_BAD_PEER_TX_FLOW_CL)

eHalStatus sme_init_bad_peer_txctl_info(tHalHandle hHal,
			struct sme_bad_peer_txctl_param param );
#else
static inline eHalStatus sme_init_bad_peer_txctl_info(tHalHandle hHal,
			struct sme_bad_peer_txctl_param param )
{
	/* no-op */
	return eHAL_STATUS_SUCCESS;
}
#endif

#ifdef WLAN_FEATURE_STATS_EXT

typedef struct sStatsExtRequestReq {
  tANI_U32 request_data_len;
  tANI_U8* request_data;
} tStatsExtRequestReq, *tpStatsExtRequestReq;

typedef void (* StatsExtCallback)(void *, tStatsExtEvent *);

void sme_StatsExtRegisterCallback(tHalHandle hHal, StatsExtCallback callback);

eHalStatus sme_StatsExtRequest(tANI_U8 session_id, tpStatsExtRequestReq input);

eHalStatus sme_StatsExtEvent (tHalHandle hHal, void* pMsg);

#endif

/* -------------------------------------------------------------------------
   \fn sme_set_dot11p_config
   \brief API to Set 802.11p config
   \param hal - The handle returned by macOpen
   \param enable_dot11p - 802.11p config param
   \return eHalStatus
---------------------------------------------------------------------------*/
void sme_set_dot11p_config(tHalHandle hal, bool enable_dot11p);

eHalStatus sme_ocb_set_config(tHalHandle hHal, void *context,
                              ocb_callback callback,
                              struct sir_ocb_config *config);

eHalStatus sme_ocb_set_utc_time(struct sir_ocb_utc *utc);

eHalStatus sme_ocb_start_timing_advert(struct sir_ocb_timing_advert
                                       *timing_advert);

eHalStatus sme_ocb_stop_timing_advert(struct sir_ocb_timing_advert
                                      *timing_advert);

eHalStatus sme_ocb_get_tsf_timer(tHalHandle hHal, void *context,
                                 ocb_callback callback,
                                 struct sir_ocb_get_tsf_timer *request);

eHalStatus sme_dcc_get_stats(tHalHandle hHal, void *context,
                             ocb_callback callback,
                             struct sir_dcc_get_stats *request);

eHalStatus sme_dcc_clear_stats(uint32_t vdev_id, uint32_t dcc_stats_bitmap);

eHalStatus sme_dcc_update_ndl(tHalHandle hHal, void* context,
                              ocb_callback callback,
                              struct sir_dcc_update_ndl *request);

eHalStatus sme_register_for_dcc_stats_event(tHalHandle hHal, void* context,
                                            ocb_callback callback);

/* ---------------------------------------------------------------------------
    \fn sme_UpdateDFSScanMode
    \brief  Update DFS roam scan mode
            This function is called through dynamic setConfig callback function
            to configure allowDFSChannelRoam.
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \param  allowDFSChannelRoam - DFS roaming scan mode 0 (disable),
            1 (passive), 2 (active)
    \return eHAL_STATUS_SUCCESS - SME update DFS roaming scan config
            successfully.
            Other status means SME failed to update DFS roaming scan config.
    \sa
    -------------------------------------------------------------------------*/
eHalStatus sme_UpdateDFSScanMode(tHalHandle hHal,
                                 tANI_U8 sessionId,
                                 v_U8_t allowDFSChannelRoam);

/*--------------------------------------------------------------------------
  \brief sme_GetDFSScanMode() - get DFS roam scan mode
            This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \return DFS roaming mode 0 (disabled), 1 (passive), 2 (active)
  \sa
  --------------------------------------------------------------------------*/
v_BOOL_t sme_GetDFSScanMode(tHalHandle hHal);

/* ---------------------------------------------------------------------------
    \fn sme_staInMiddleOfRoaming
    \brief  This function returns TRUE if STA is in the middle of roaming state
    \param  hHal - HAL handle for device
    \param  sessionId - Session identifier
    \- return TRUE or FALSE
    -------------------------------------------------------------------------*/
tANI_BOOLEAN sme_staInMiddleOfRoaming(tHalHandle hHal, tANI_U8 sessionId);

/* ---------------------------------------------------------------------------
    \fn sme_PsOffloadIsStaInPowerSave
    \brief  This function returns TRUE if STA is in power save
    \param  hHal - HAL handle for device
    \param  sessionId - Session Identifier
    \- return TRUE or FALSE
    -------------------------------------------------------------------------*/
tANI_BOOLEAN sme_PsOffloadIsStaInPowerSave(tHalHandle hHal, tANI_U8 sessionId);

#ifdef FEATURE_WLAN_EXTSCAN
/* ---------------------------------------------------------------------------
    \fn sme_GetValidChannelsByBand
    \brief  SME API to fetch all valid channel filtered by band
    \param  hHal
    \param  wifiBand: RF band information
    \param  aValidChannels: Array to store channel info
    \param  len: number of channels
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_GetValidChannelsByBand (tHalHandle hHal, tANI_U8 wifiBand,
                              tANI_U32 *aValidChannels, tANI_U8 *pNumChannels);

/* ---------------------------------------------------------------------------
    \fn sme_ExtScanGetCapabilities
    \brief  SME API to fetch extscan capabilities
    \param  hHal
    \param  pReq: extscan capabilities structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_ExtScanGetCapabilities (tHalHandle hHal,
                                    tSirGetExtScanCapabilitiesReqParams *pReq);

/* ---------------------------------------------------------------------------
    \fn sme_ExtScanStart
    \brief  SME API to issue extscan start
    \param  hHal
    \param  pStartCmd: extscan start structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_ExtScanStart (tHalHandle hHal,
                          tSirWifiScanCmdReqParams *pStartCmd);

/* ---------------------------------------------------------------------------
    \fn sme_ExtScanStop
    \brief  SME API to issue extscan stop
    \param  hHal
    \param  pStopReq: extscan stop structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_ExtScanStop(tHalHandle hHal,
                           tSirExtScanStopReqParams *pStopReq);

/* ---------------------------------------------------------------------------
    \fn sme_SetBssHotlist
    \brief  SME API to set BSSID hotlist
    \param  hHal
    \param  pSetHotListReq: extscan set hotlist structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetBssHotlist (tHalHandle hHal,
                          tSirExtScanSetBssidHotListReqParams *pSetHotListReq);

/* ---------------------------------------------------------------------------
    \fn sme_ResetBssHotlist
    \brief  SME API to reset BSSID hotlist
    \param  hHal
    \param  pSetHotListReq: extscan set hotlist structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_ResetBssHotlist (tHalHandle hHal,
                             tSirExtScanResetBssidHotlistReqParams *pResetReq);

eHalStatus
sme_set_ssid_hotlist(tHalHandle hal,
		     struct sir_set_ssid_hotlist_request *request);

/* ---------------------------------------------------------------------------
    \fn sme_SetSignificantChange
    \brief  SME API to set significant change
    \param  hHal
    \param  pSetSignificantChangeReq: extscan set significant change structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetSignificantChange (tHalHandle hHal,
                   tSirExtScanSetSigChangeReqParams* pSetSignificantChangeReq);

/* ---------------------------------------------------------------------------
    \fn sme_ResetSignificantChange
    \brief  SME API to reset significant change
    \param  hHal
    \param  pResetReq: extscan reset significant change structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_ResetSignificantChange (tHalHandle hHal,
                        tSirExtScanResetSignificantChangeReqParams *pResetReq);

/* ---------------------------------------------------------------------------
    \fn sme_getCachedResults
    \brief  SME API to get cached results
    \param  hHal
    \param  pCachedResultsReq: extscan get cached results structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_getCachedResults (tHalHandle hHal,
                      tSirExtScanGetCachedResultsReqParams *pCachedResultsReq);

eHalStatus sme_set_epno_list(tHalHandle hal,
                                struct wifi_epno_params *req_msg);
eHalStatus sme_set_passpoint_list(tHalHandle hal,
				  struct wifi_passpoint_req *req_msg);
eHalStatus sme_reset_passpoint_list(tHalHandle hal,
				    struct wifi_passpoint_req *req_msg);

/* ---------------------------------------------------------------------------
    \fn sme_ExtScanRegisterCallback
    \brief  SME API to register extscan notification callback
    \param  pExtScanIndCb
    \- return void
    -------------------------------------------------------------------------*/
eHalStatus sme_ExtScanRegisterCallback (tHalHandle hHal,
                        void (*pExtScanIndCb)(void *, const tANI_U16, void *));

#endif /* FEATURE_WLAN_EXTSCAN */

VOS_STATUS sme_set_beacon_filter(uint32_t vdev_id, uint32_t *ie_map);
VOS_STATUS sme_unset_beacon_filter(uint32_t vdev_id);

#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
/* ---------------------------------------------------------------------------
    \fn sme_abortRoamScan
    \brief  API to abort current roam scan cycle by roam scan offload module.
    \param  hHal - The handle returned by macOpen.
    \param  sessionId - Session identifier
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_abortRoamScan(tHalHandle hHal, tANI_U8 sessionId);
#endif //#if WLAN_FEATURE_ROAM_SCAN_OFFLOAD

#ifdef WLAN_FEATURE_LINK_LAYER_STATS

/* ---------------------------------------------------------------------------
    \fn sme_LLStatsClearReq
    \brief  SME API to clear Link Layer Statistics
    \param  hHal
    \param  pclearStatsReq: Link Layer clear stats request params structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_LLStatsClearReq (tHalHandle hHal,
                        tSirLLStatsClearReq *pclearStatsReq);

/* ---------------------------------------------------------------------------
    \fn sme_LLStatsSetReq
    \brief  SME API to set the Link Layer Statistics
    \param  hHal
    \param  psetStatsReq: Link Layer set stats request params structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_LLStatsSetReq (tHalHandle hHal,
                        tSirLLStatsSetReq *psetStatsReq);

/* ---------------------------------------------------------------------------
    \fn sme_LLStatsGetReq
    \brief  SME API to get the Link Layer Statistics
    \param  hHal
    \param  pgetStatsReq: Link Layer get stats request params structure
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_LLStatsGetReq (tHalHandle hHal,
                        tSirLLStatsGetReq *pgetStatsReq);

/* ---------------------------------------------------------------------------
    \fn sme_SetLinkLayerStatsIndCB
    \brief  SME API to trigger the stats are available  after get request
    \param  hHal
    \param callbackRoutine - HDD callback which needs to be invoked after
           getting status notification from FW
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetLinkLayerStatsIndCB
(
    tHalHandle hHal,
    void (*callbackRoutine) (void *callbackCtx, int indType, void *pRsp)
);

#endif /* WLAN_FEATURE_LINK_LAYER_STATS */

eHalStatus sme_fw_mem_dump(tHalHandle hHal, void *recvd_req);
eHalStatus sme_fw_mem_dump_register_cb(tHalHandle hHal,
    void (*callback_routine)(void *cb_context, struct fw_dump_rsp *rsp));
eHalStatus sme_fw_mem_dump_unregister_cb(tHalHandle hHal);

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
/*--------------------------------------------------------------------------
  \brief sme_UpdateRoamOffloadEnabled() - enable/disable roam offload feature
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param nRoamOffloadEnabled - The boolean to update with
  \return eHAL_STATUS_SUCCESS - SME update config successfully.
          Other status means SME is failed to update.
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_UpdateRoamOffloadEnabled(tHalHandle hHal,
                                     v_BOOL_t nRoamOffloadEnabled);

/*--------------------------------------------------------------------------
  \brief sme_UpdateRoamKeyMgmtOffloadEnabled() - enable/disable key mgmt offload
  This is a synchronous call
  \param hHal - The handle returned by macOpen.
  \param  sessionId - Session Identifier
  \param nRoamKeyMgmtOffloadEnabled - The boolean to update with
  \return eHAL_STATUS_SUCCESS - SME update config successfully.
          Other status means SME is failed to update.
  \sa
  --------------------------------------------------------------------------*/
eHalStatus sme_UpdateRoamKeyMgmtOffloadEnabled(tHalHandle hHal,
                                     tANI_U8 sessionId,
                                     v_BOOL_t nRoamKeyMgmtOffloadEnabled);

#endif

#ifdef WLAN_FEATURE_NAN
/******************************************************************************
  \fn sme_NanEvent

  \brief
  a callback function called when SME received eWNI_SME_NAN_EVENT
  event from WDA

  \param hHal - HAL handle for device
  \param pMsg - Message body passed from WDA; includes NAN header

  \return VOS_STATUS
******************************************************************************/
VOS_STATUS sme_NanEvent(tHalHandle hHal, void* pMsg);
#endif /* WLAN_FEATURE_NAN */

/*--------------------------------------------------------------------------
  \brief sme_getLinkStatus() - api to get the link status
  \param hHal - The handle returned by macOpen.
  \param callback - callback function for link status result from FW
  \param pContext - The context passed with callback
  \param sessionId - SME sessionId
  \return eHalStatus
  --------------------------------------------------------------------------*/
eHalStatus sme_getLinkStatus(tHalHandle hHal,
                             tCsrLinkStatusCallback callback,
                             void *pContext,
                             tANI_U8 sessionId);

eHalStatus sme_get_fw_state(tHalHandle hHal,
                             tcsr_fw_state_callback callback,
                             void *context);

/* ---------------------------------------------------------------------------
   \fn sme_GetTemperature
   \brief SME API to get the pdev temperature
   \param hHal
   \param temperature context
   \param pCallbackfn: callback fn with response (temperature)
   \- return eHalStatus
   -------------------------------------------------------------------------*/
eHalStatus sme_GetTemperature(tHalHandle hHal,
        void *tempContext,
        void (*pCallbackfn)(int temperature, void *pContext));

/* ---------------------------------------------------------------------------
    \fn sme_SetScanningMacOui
    \brief  SME API to set scanning mac oui
    \param  hHal
    \param  pScanMacOui: Scanning Mac Oui (input 3 bytes)
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_SetScanningMacOui(tHalHandle hHal, tSirScanMacOui *pScanMacOui);

#ifdef IPA_UC_OFFLOAD
/* ---------------------------------------------------------------------------
    \fn sme_ipa_offload_enable_disable
    \brief  API to enable/disable IPA offload
    \param  hHal - The handle returned by macOpen.
    \param  sessionId - Session Identifier
    \param  pRequest -  Pointer to the offload request.
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_ipa_offload_enable_disable(tHalHandle hal, tANI_U8 session_id,
                               struct sir_ipa_offload_enable_disable *request);
#endif

#ifdef DHCP_SERVER_OFFLOAD
/* ---------------------------------------------------------------------------
    \fn sme_setDhcpSrvOffload
    \brief  SME API to set DHCP server offload info
    \param  hHal
    \param  pDhcpSrvInfo : DHCP server offload info struct
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_setDhcpSrvOffload(tHalHandle hHal,
                                tSirDhcpSrvOffloadInfo *pDhcpSrvInfo);
#endif /* DHCP_SERVER_OFFLOAD */

#ifdef WLAN_FEATURE_GPIO_LED_FLASHING
/* ---------------------------------------------------------------------------
    \fn sme_SetLedFlashing
    \brief  API to set the LED flashing feature.
    \param  hHal - The handle returned by macOpen.
    \param  x0, x1 -  led flashing parameters
    \param  gpio_num -  GPIO number
    \return eHalStatus
  ---------------------------------------------------------------------------*/
eHalStatus sme_SetLedFlashing (tHalHandle hHal, tANI_U8 type,
                               tANI_U32 x0, tANI_U32 x1, tANI_U32 gpio_num);
#endif
/* ---------------------------------------------------------------------------
    \fn sme_handle_dfS_chan_scan
    \brief  SME API to enable/disable DFS channel scan
    \param  hHal
    \param dfs_flag: whether dfs needs to be enabled or disabled
    \return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_handle_dfs_chan_scan(tHalHandle hHal, tANI_U8 dfs_flag);
eHalStatus sme_ext_change_channel(tHalHandle hHal, uint32_t channel,
						uint8_t session_id);

#ifdef MDNS_OFFLOAD
/* ---------------------------------------------------------------------------
    \fn sme_setMDNSOffload
    \brief  SME API to set mDNS offload info
    \param  hHal
    \param  pMDNSInfo : mDNS offload info struct
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_setMDNSOffload(tHalHandle hHal,
                                tSirMDNSOffloadInfo *pMDNSInfo);

/* ---------------------------------------------------------------------------
    \fn sme_setMDNSFqdn
    \brief  SME API to set mDNS Fqdn info
    \param  hHal
    \param  pMDNSFqdnInfo : mDNS Fqdn info struct
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_setMDNSFqdn(tHalHandle hHal,
                                tSirMDNSFqdnInfo *pMDNSFqdnInfo);

/* ---------------------------------------------------------------------------
    \fn sme_setMDNSResponse
    \brief  SME API to set mDNS response info
    \param  hHal
    \param  pMDNSRespInfo : mDNS response info struct
    \- return eHalStatus
    -------------------------------------------------------------------------*/
eHalStatus sme_setMDNSResponse(tHalHandle hHal,
                                tSirMDNSResponseInfo *pMDNSRespInfo);
#endif /* MDNS_OFFLOAD */

#ifdef SAP_AUTH_OFFLOAD
/**
 * sme_set_sap_auth_offload() enable/disable Software AP Auth Offload
 * @hHal: hal layer handler
 * @sap_auth_offload_info: the information of Software AP Auth Offload
 *
 * This function provide enable/disable Software AP authenticaiton offload
 * feature on target firmware
 *
 * Return: Return eHalStatus.
 */
eHalStatus sme_set_sap_auth_offload(tHalHandle hHal,
                      struct tSirSapOffloadInfo *sap_auth_offload_info);

eHalStatus sme_set_client_block_info(tHalHandle hHal,
			struct sblock_info *client_block_info);

#else

static inline eHalStatus sme_set_client_block_info(tHalHandle hHal,
			struct sblock_info *client_block_info)
{
	return eHAL_STATUS_FAILURE;
}

#endif /* SAP_AUTH_OFFLOAD */

eHalStatus sme_set_tsfcb(tHalHandle hHal,
	int (*pcallbackfn)(void *pcallbackcontext, struct stsf *ptsf),
	void *pcallbackcontext);

#ifdef WLAN_FEATURE_APFIND
VOS_STATUS sme_apfind_set_cmd(struct sme_ap_find_request_req *input);
#endif /* WLAN_FEATURE_APFIND */

/**
 * sme_enable_disable_mas() - Function to set MAS value to UMAC
 * @val:        1-Enable, 0-Disable
 *
 * This function passes down the value of MAS to the UMAC. A
 * value of 1 will enable MAS and a value of 0 will disable MAS
 *
 * Return: Configuration message posting status, SUCCESS or Fail
 *
 */
eHalStatus sme_set_mas(tANI_U32 val);

/**
 * sme_set_miracast() - Function to set miracast value to UMAC
 * @hal:                Handle returned by macOpen
 * @filter_type:        0-Disabled, 1-Source, 2-sink
 *
 * This function passes down the value of miracast set by
 * framework to UMAC
 *
 * Return: Configuration message posting status, SUCCESS or Fail
 *
 */
eHalStatus sme_set_miracast(tHalHandle hal, uint8_t filter_type);
bool sme_validate_sap_channel_switch(tHalHandle hal,
					uint16_t sap_ch,
					eCsrPhyMode sap_phy_mode,
					uint8_t cc_switch_mode,
					uint32_t session_id);

eHalStatus sme_configure_modulated_dtim(tHalHandle hHal, tANI_U8 session_id,
					tANI_U32 modulated_dtim);

eHalStatus sme_configure_stats_avg_factor(tHalHandle hHal, tANI_U8 session_id,
					  tANI_U16 stats_avg_factor);

eHalStatus sme_configure_guard_time(tHalHandle hHal, tANI_U8 session_id,
                                    tANI_U32 guard_time);
eHalStatus sme_update_roam_scan_hi_rssi_scan_params(tHalHandle hal_handle,
	uint8_t session_id,
	uint32_t notify_id,
	int32_t val);

eHalStatus sme_wifi_start_logger(tHalHandle hal,
		struct sir_wifi_start_log start_log);

#ifdef WLAN_FEATURE_TSF
eHalStatus sme_set_tsf_gpio(tHalHandle hHal, uint32_t pinvalue);
#else
static inline eHalStatus
sme_set_tsf_gpio(tHalHandle hHal, uint32_t pinvalue)
{
	return eHAL_STATUS_FAILURE;
}
#endif

bool smeNeighborMiddleOfRoaming(tHalHandle hHal, tANI_U8 sessionId);
eHalStatus sme_register_mgmt_frame_ind_callback(tHalHandle hal,
      sir_mgmt_frame_ind_callback callback);

eHalStatus sme_update_nss(tHalHandle h_hal, uint8_t nss);
void sme_enable_phy_error_logs(tHalHandle hal, bool enable_log);

uint8_t    sme_is_any_session_in_connected_state(tHalHandle h_hal);

typedef void ( *tSmeSetThermalLevelCallback)(void *pContext, u_int8_t level);
void sme_add_set_thermal_level_callback(tHalHandle hHal,
                   tSmeSetThermalLevelCallback callback);

eHalStatus sme_disable_non_fcc_channel(tHalHandle hHal,
				       bool fcc_constraint);

eHalStatus sme_set_rssi_monitoring(tHalHandle hal,
					struct rssi_monitor_req *input);
eHalStatus sme_set_rssi_threshold_breached_cb(tHalHandle hal,
			void (*cb)(void *, struct rssi_breach_event *));
void sme_set_pdev_ht_vht_ies(tHalHandle hHal, bool enable2x2);

void sme_update_vdev_type_nss(tHalHandle hal, uint8_t max_supp_nss,
		uint32_t vdev_type_nss, eCsrBand band);
void sme_update_user_configured_nss(tHalHandle hal, uint8_t nss);
void sme_set_vdev_nss(tHalHandle hal, bool enable2x2);
void sme_set_per_band_chainmask_supp(tHalHandle hal, bool val);
void sme_set_lte_coex_supp(tHalHandle hal, bool val);
void sme_set_bcon_offload_supp(tHalHandle hal, bool val);
void sme_SetDefDot11Mode(tHalHandle hal);

#ifdef FEATURE_WLAN_TDLS
void sme_get_opclass(tHalHandle hal, uint8_t channel, uint8_t bw_offset,
		uint8_t *opclass);
#else
static inline void
sme_get_opclass(tHalHandle hal, uint8_t channel, uint8_t bw_offset,
		uint8_t *opclass)
{
}
#endif



#ifdef WLAN_FEATURE_UDP_RESPONSE_OFFLOAD
VOS_STATUS sme_set_udp_resp_offload(struct udp_resp_offload *pudp_resp_cmd);
#else
static inline VOS_STATUS sme_set_udp_resp_offload(struct udp_resp_offload
							*pudp_resp_cmd)
{
	return VOS_STATUS_E_FAILURE;
}
#endif


eHalStatus sme_set_lost_link_info_cb(tHalHandle hal,
                                     void (*cb)(void *,
                                                struct sir_lost_link_info *));

eHalStatus sme_roam_set_default_key_index(tHalHandle hal, uint8_t session_id,
					uint8_t default_idx);

eHalStatus sme_set_smps_force_mode_cb(tHalHandle hal,
		void (*cb)(void *, struct sir_smps_force_mode_event *));

eHalStatus sme_update_mimo_power_save(tHalHandle hHal,
				      uint8_t is_ht_smps_enabled,
				      uint8_t ht_smps_mode);

bool sme_is_sta_smps_allowed(tHalHandle hHal, uint8_t session_id);

void sme_update_fine_time_measurement_capab(tHalHandle hal, uint32_t val);

eHalStatus sme_delete_all_tdls_peers(tHalHandle hal, uint8_t session_id);

VOS_STATUS sme_is_session_valid(tHalHandle hal_handle, uint8_t session_id);
eHalStatus sme_register_p2p_ack_ind_callback(tHalHandle hal,
					sir_p2p_ack_ind_callback callback);
#endif //#if !defined( __SME_API_H )
