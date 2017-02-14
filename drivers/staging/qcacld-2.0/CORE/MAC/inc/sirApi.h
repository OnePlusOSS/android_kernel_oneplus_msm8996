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


/*
 * This file sirApi.h contains definitions exported by
 * Sirius software.
 * Author:        Chandra Modumudi
 * Date:          04/16/2002
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 */

#ifndef __SIR_API_H
#define __SIR_API_H

#include "sirTypes.h"
#include "sirMacProtDef.h"
#include "aniSystemDefs.h"
#include "sirParams.h"

#if defined(FEATURE_WLAN_ESE) && !defined(FEATURE_WLAN_ESE_UPLOAD)
#include "eseGlobal.h"
#endif

/// Start of Sirius software/Host driver message types
#define SIR_HAL_HOST_MSG_START         0x1000

/// Max supported channel list
#define SIR_MAX_SUPPORTED_CHANNEL_LIST      96

#define SIR_MDIE_SIZE               3

// Increase dwell time for P2P search in ms
#define P2P_SEARCH_DWELL_TIME_INCREASE   20
#define P2P_SOCIAL_CHANNELS              3

/* Max number of channels are 165, but to access 165th element of array,
 *array of 166 is required.
 */
#define SIR_MAX_24G_5G_CHANNEL_RANGE      166
#define SIR_BCN_REPORT_MAX_BSS_DESC       4

#define SIR_NUM_11B_RATES 4   //1,2,5.5,11
#define SIR_NUM_11A_RATES 8  //6,9,12,18,24,36,48,54

#define SIR_PM_SLEEP_MODE   0
#define SIR_PM_ACTIVE_MODE        1

//hidden SSID options
#define SIR_SCAN_NO_HIDDEN_SSID                      0
#define SIR_SCAN_HIDDEN_SSID_PE_DECISION             1

#define SIR_MAC_ADDR_LEN        6
#define SIR_IPV4_ADDR_LEN       4

typedef tANI_U8 tSirIpv4Addr[SIR_IPV4_ADDR_LEN];

#define SIR_VERSION_STRING_LEN 64
typedef tANI_U8 tSirVersionString[SIR_VERSION_STRING_LEN];

/* Periodic Tx pattern offload feature */
#define PERIODIC_TX_PTRN_MAX_SIZE 1536
#define MAXNUM_PERIODIC_TX_PTRNS 6

#define WIFI_SCANNING_MAC_OUI_LENGTH 3

#define MAX_LEN_UDP_RESP_OFFLOAD 128

#ifdef FEATURE_WLAN_EXTSCAN

#define WLAN_EXTSCAN_MAX_CHANNELS                 36
#define WLAN_EXTSCAN_MAX_BUCKETS                  16
#define WLAN_EXTSCAN_MAX_HOTLIST_APS              128
#define WLAN_EXTSCAN_MAX_SIGNIFICANT_CHANGE_APS   64
#define WLAN_EXTSCAN_MAX_HOTLIST_SSIDS            8

typedef enum
{
    eSIR_EXTSCAN_INVALID,
    eSIR_EXTSCAN_START_RSP,
    eSIR_EXTSCAN_STOP_RSP,
    eSIR_EXTSCAN_CACHED_RESULTS_RSP,
    eSIR_EXTSCAN_SET_BSSID_HOTLIST_RSP,
    eSIR_EXTSCAN_RESET_BSSID_HOTLIST_RSP,
    eSIR_EXTSCAN_SET_SIGNIFICANT_WIFI_CHANGE_RSP,
    eSIR_EXTSCAN_RESET_SIGNIFICANT_WIFI_CHANGE_RSP,

    eSIR_EXTSCAN_GET_CAPABILITIES_IND,
    eSIR_EXTSCAN_HOTLIST_MATCH_IND,
    eSIR_EXTSCAN_SIGNIFICANT_WIFI_CHANGE_RESULTS_IND,
    eSIR_EXTSCAN_CACHED_RESULTS_IND,
    eSIR_EXTSCAN_SCAN_RES_AVAILABLE_IND,
    eSIR_EXTSCAN_SCAN_PROGRESS_EVENT_IND,
    eSIR_EXTSCAN_FULL_SCAN_RESULT_IND,
    eSIR_EPNO_NETWORK_FOUND_IND,
    eSIR_PASSPOINT_NETWORK_FOUND_IND,
    eSIR_EXTSCAN_SET_SSID_HOTLIST_RSP,
    eSIR_EXTSCAN_RESET_SSID_HOTLIST_RSP,
    eSIR_EXTSCAN_HOTLIST_SSID_MATCH_IND,

    /* Keep this last */
    eSIR_EXTSCAN_CALLBACK_TYPE_MAX,
} tSirExtScanCallbackType;


#endif /* FEATURE_WLAN_EXTSCAN */

#define SIR_KRK_KEY_LEN 16
#ifdef WLAN_FEATURE_ROAM_OFFLOAD
#define SIR_BTK_KEY_LEN 32
#define SIR_KCK_KEY_LEN 16
#define SIR_KEK_KEY_LEN 16
#define SIR_REPLAY_CTR_LEN 8

#define SIR_UAPSD_BITOFFSET_ACVO     0
#define SIR_UAPSD_BITOFFSET_ACVI     1
#define SIR_UAPSD_BITOFFSET_ACBK     2
#define SIR_UAPSD_BITOFFSET_ACBE     3

#define SIR_UAPSD_FLAG_ACVO     (1 << SIR_UAPSD_BITOFFSET_ACVO)
#define SIR_UAPSD_FLAG_ACVI     (1 << SIR_UAPSD_BITOFFSET_ACVI)
#define SIR_UAPSD_FLAG_ACBK     (1 << SIR_UAPSD_BITOFFSET_ACBK)
#define SIR_UAPSD_FLAG_ACBE     (1 << SIR_UAPSD_BITOFFSET_ACBE)
#define SIR_UAPSD_GET(ac, mask)      (((mask) & (SIR_UAPSD_FLAG_ ## ac)) >> SIR_UAPSD_BITOFFSET_ ## ac)
#endif
enum eSirHostMsgTypes
{
    SIR_HAL_APP_SETUP_NTF = SIR_HAL_HOST_MSG_START,
    SIR_HAL_INITIAL_CAL_FAILED_NTF,
    SIR_HAL_NIC_OPER_NTF,
    SIR_HAL_INIT_START_REQ,
    SIR_HAL_SHUTDOWN_REQ,
    SIR_HAL_SHUTDOWN_CNF,
    SIR_HAL_RESET_REQ,
    SIR_HAL_RADIO_ON_OFF_IND,
    SIR_HAL_RESET_CNF,
    SIR_WRITE_TO_TD,
    SIR_HAL_HDD_ADDBA_REQ, // MAC -> HDD
    SIR_HAL_HDD_ADDBA_RSP, // HDD -> HAL
    SIR_HAL_DELETEBA_IND, // MAC -> HDD
    SIR_HAL_BA_FAIL_IND, // HDD -> MAC
    SIR_TL_HAL_FLUSH_AC_REQ,
    SIR_HAL_TL_FLUSH_AC_RSP
};



/**
 * Module ID definitions.
 */
enum {
    SIR_BOOT_MODULE_ID = 1,
    SIR_HAL_MODULE_ID  = 0x10,
    SIR_CFG_MODULE_ID = 0x12,
    SIR_LIM_MODULE_ID,
    SIR_ARQ_MODULE_ID,
    SIR_SCH_MODULE_ID,
    SIR_PMM_MODULE_ID,
    SIR_MNT_MODULE_ID,
    SIR_DBG_MODULE_ID,
    SIR_DPH_MODULE_ID,
    SIR_SYS_MODULE_ID,
    SIR_SMS_MODULE_ID,

    SIR_PHY_MODULE_ID = 0x20,


    // Add any modules above this line
    SIR_DVT_MODULE_ID
};

#define SIR_WDA_MODULE_ID SIR_HAL_MODULE_ID

/**
 * First and last module definition for logging utility
 *
 * NOTE:  The following definitions need to be updated if
 *        the above list is changed.
 */
#define SIR_FIRST_MODULE_ID     SIR_HAL_MODULE_ID
#define SIR_LAST_MODULE_ID      SIR_DVT_MODULE_ID


// Type declarations used by Firmware and Host software

// Scan type enum used in scan request
typedef enum eSirScanType
{
    eSIR_PASSIVE_SCAN,
    eSIR_ACTIVE_SCAN,
    eSIR_BEACON_TABLE,
} tSirScanType;

/// Result codes Firmware return to Host SW
typedef enum eSirResultCodes
{
    eSIR_SME_SUCCESS,

    eSIR_EOF_SOF_EXCEPTION,
    eSIR_BMU_EXCEPTION,
    eSIR_LOW_PDU_EXCEPTION,
    eSIR_USER_TRIG_RESET,
    eSIR_LOGP_EXCEPTION,
    eSIR_CP_EXCEPTION,
    eSIR_STOP_BSS,
    eSIR_AHB_HANG_EXCEPTION,
    eSIR_DPU_EXCEPTION,
    eSIR_RPE_EXCEPTION,
    eSIR_TPE_EXCEPTION,
    eSIR_DXE_EXCEPTION,
    eSIR_RXP_EXCEPTION,
    eSIR_MCPU_EXCEPTION,
    eSIR_MCU_EXCEPTION,
    eSIR_MTU_EXCEPTION,
    eSIR_MIF_EXCEPTION,
    eSIR_FW_EXCEPTION,
    eSIR_PS_MUTEX_READ_EXCEPTION,
    eSIR_PHY_HANG_EXCEPTION,
    eSIR_MAILBOX_SANITY_CHK_FAILED,
    eSIR_RADIO_HW_SWITCH_STATUS_IS_OFF, // Only where this switch is present
    eSIR_CFB_FLAG_STUCK_EXCEPTION,

    eSIR_SME_BASIC_RATES_NOT_SUPPORTED_STATUS=30,

    eSIR_SME_INVALID_PARAMETERS=500,
    eSIR_SME_UNEXPECTED_REQ_RESULT_CODE,
    eSIR_SME_RESOURCES_UNAVAILABLE,
    eSIR_SME_SCAN_FAILED,   // Unable to find a BssDescription
                            // matching requested scan criteria
    eSIR_SME_BSS_ALREADY_STARTED_OR_JOINED,
    eSIR_SME_LOST_LINK_WITH_PEER_RESULT_CODE,
    eSIR_SME_REFUSED,
    eSIR_SME_JOIN_DEAUTH_FROM_AP_DURING_ADD_STA,
    eSIR_SME_JOIN_TIMEOUT_RESULT_CODE,
    eSIR_SME_AUTH_TIMEOUT_RESULT_CODE,
    eSIR_SME_ASSOC_TIMEOUT_RESULT_CODE,
    eSIR_SME_REASSOC_TIMEOUT_RESULT_CODE,
    eSIR_SME_MAX_NUM_OF_PRE_AUTH_REACHED,
    eSIR_SME_AUTH_REFUSED,
    eSIR_SME_INVALID_WEP_DEFAULT_KEY,
    eSIR_SME_NO_KEY_MAPPING_KEY_FOR_PEER,
    eSIR_SME_ASSOC_REFUSED,
    eSIR_SME_REASSOC_REFUSED,

    /* Received Deauth while joining or pre-authenticating */
    eSIR_SME_DEAUTH_WHILE_JOIN,

    eSIR_SME_DISASSOC_WHILE_JOIN, //Received Disassociation while joining.
    eSIR_SME_DEAUTH_WHILE_REASSOC, //Received Deauth while ReAssociate.
    eSIR_SME_DISASSOC_WHILE_REASSOC, //Received Disassociation while ReAssociate
    eSIR_SME_STA_NOT_AUTHENTICATED,
    eSIR_SME_STA_NOT_ASSOCIATED,
    eSIR_SME_STA_DISASSOCIATED,
    eSIR_SME_ALREADY_JOINED_A_BSS,
    eSIR_ULA_COMPLETED,
    eSIR_ULA_FAILURE,
    eSIR_SME_LINK_ESTABLISHED,
    eSIR_SME_UNABLE_TO_PERFORM_MEASUREMENTS,
    eSIR_SME_UNABLE_TO_PERFORM_DFS,
    eSIR_SME_DFS_FAILED,
    eSIR_SME_TRANSFER_STA, // To be used when STA need to be LB'ed
    eSIR_SME_INVALID_LINK_TEST_PARAMETERS,// Given in LINK_TEST_START_RSP
    eSIR_SME_LINK_TEST_MAX_EXCEEDED,    // Given in LINK_TEST_START_RSP
    eSIR_SME_UNSUPPORTED_RATE,          // Given in LINK_TEST_RSP if peer does
                                        // support requested rate in
                                        // LINK_TEST_REQ
    eSIR_SME_LINK_TEST_TIMEOUT,         // Given in LINK_TEST_IND if peer does
                                        // not respond before next test packet
                                        // is sent
    eSIR_SME_LINK_TEST_COMPLETE,        // Given in LINK_TEST_IND at the end
                                        // of link test
    eSIR_SME_LINK_TEST_INVALID_STATE,   // Given in LINK_TEST_START_RSP
    eSIR_SME_LINK_TEST_TERMINATE,       // Given in LINK_TEST_START_RSP
    eSIR_SME_LINK_TEST_INVALID_ADDRESS, // Given in LINK_TEST_STOP_RSP
    eSIR_SME_SETCONTEXT_FAILED,         // Given in SME_SETCONTEXT_REQ when
                                        // unable to plumb down keys
    eSIR_SME_BSS_RESTART,               // Given in SME_STOP_BSS_REQ

    eSIR_SME_MORE_SCAN_RESULTS_FOLLOW,  // Given in SME_SCAN_RSP message
                                        // that more SME_SCAN_RSP
                                        // messages are following.
                                        // SME_SCAN_RSP message with
                                        // eSIR_SME_SUCCESS status
                                        // code is the last one.
    eSIR_SME_INVALID_ASSOC_RSP_RXED,    // Sent in SME_JOIN/REASSOC_RSP
                                        // messages upon receiving
                                        // invalid Re/Assoc Rsp frame.
    eSIR_SME_MIC_COUNTER_MEASURES,      // STOP BSS triggered by MIC failures: MAC software to disassoc all stations
                                        // with MIC_FAILURE reason code and perform the stop bss operation
    eSIR_SME_ADDTS_RSP_TIMEOUT,         // didn't get response from peer within
                                        // timeout interval
    eSIR_SME_ADDTS_RSP_FAILED,          // didn't get success response from HAL
    eSIR_SME_RECEIVED,
    // TBA - TSPEC related Result Codes

    eSIR_SME_CHANNEL_SWITCH_FAIL,        // failed to send out Channel Switch Action Frame
    eSIR_SME_INVALID_STA_ROLE,
    eSIR_SME_INVALID_STATE,
    eSIR_SME_CHANNEL_SWITCH_DISABLED,    // either 11h is disabled or channelSwitch is currently active
    eSIR_SME_HAL_SCAN_INIT_FAILED,       // SIR_HAL_SIR_HAL_INIT_SCAN_RSP returned failed status
    eSIR_SME_HAL_SCAN_START_FAILED,      // SIR_HAL_START_SCAN_RSP returned failed status
    eSIR_SME_HAL_SCAN_END_FAILED,        // SIR_HAL_END_SCAN_RSP returned failed status
    eSIR_SME_HAL_SCAN_FINISH_FAILED,     // SIR_HAL_FINISH_SCAN_RSP returned failed status
    eSIR_SME_HAL_SEND_MESSAGE_FAIL,      // Failed to send a message to HAL
#ifdef FEATURE_OEM_DATA_SUPPORT
    eSIR_SME_HAL_OEM_DATA_REQ_START_FAILED,
#endif
    eSIR_SME_STOP_BSS_FAILURE,           // Failed to stop the bss
    eSIR_SME_STA_ASSOCIATED,
    eSIR_SME_INVALID_PMM_STATE,
    eSIR_SME_CANNOT_ENTER_IMPS,
    eSIR_SME_IMPS_REQ_FAILED,
    eSIR_SME_BMPS_REQ_FAILED,
    eSIR_SME_BMPS_REQ_REJECT,
    eSIR_SME_UAPSD_REQ_FAILED,
    eSIR_SME_WOWL_ENTER_REQ_FAILED,
    eSIR_SME_WOWL_EXIT_REQ_FAILED,
#if defined(WLAN_FEATURE_VOWIFI_11R) || defined(FEATURE_WLAN_ESE) || \
    defined(FEATURE_WLAN_LFR)
    eSIR_SME_FT_REASSOC_TIMEOUT_FAILURE,
    eSIR_SME_FT_REASSOC_FAILURE,
#endif
    eSIR_SME_SEND_ACTION_FAIL,
#ifdef WLAN_FEATURE_PACKET_FILTERING
    eSIR_SME_PC_FILTER_MATCH_COUNT_REQ_FAILED,
#endif // WLAN_FEATURE_PACKET_FILTERING

#ifdef WLAN_FEATURE_GTK_OFFLOAD
    eSIR_SME_GTK_OFFLOAD_GETINFO_REQ_FAILED,
#endif // WLAN_FEATURE_GTK_OFFLOAD
    eSIR_SME_DEAUTH_STATUS,
    eSIR_SME_SAP_AUTH_OFFLOAD_PEER_UPDATE_STATUS,
    eSIR_DONOT_USE_RESULT_CODE = SIR_MAX_ENUM_SIZE
} tSirResultCodes;

#define RMENABLEDCAP_MAX_LEN 5

struct rrm_config_param
{
	uint8_t rrm_enabled;
	uint8_t max_randn_interval;
	uint8_t rm_capability[RMENABLEDCAP_MAX_LEN];
};

/* each station added has a rate mode which specifies the sta attributes */
typedef enum eStaRateMode {
    eSTA_11b,
    eSTA_11bg,
    eSTA_11a,
    eSTA_11n,
#ifdef WLAN_FEATURE_11AC
    eSTA_11ac,
#endif
    eSTA_INVALID_RATE_MODE
} tStaRateMode, *tpStaRateMode;

//although in tSirSupportedRates each IE is 16bit but PE only passes IEs in 8 bits with MSB=1 for basic rates.
//change the mask for bit0-7 only so HAL gets correct basic rates for setting response rates.
#define IERATE_BASICRATE_MASK     0x80
#define IERATE_RATE_MASK          0x7f
#define IERATE_IS_BASICRATE(x)   ((x) & IERATE_BASICRATE_MASK)

const char * lim_BssTypetoString(const v_U8_t bssType);
const char * lim_ScanTypetoString(const v_U8_t scanType);
const char * lim_BackgroundScanModetoString(const v_U8_t mode);

typedef struct sSirSupportedRates {
    /*
    * For Self STA Entry: this represents Self Mode.
    * For Peer Stations, this represents the mode of the peer.
    * On Station:
    * --this mode is updated when PE adds the Self Entry.
    * -- OR when PE sends 'ADD_BSS' message and station context in BSS is used to indicate the mode of the AP.
    * ON AP:
    * -- this mode is updated when PE sends 'ADD_BSS' and Sta entry for that BSS is used
    *     to indicate the self mode of the AP.
    * -- OR when a station is associated, PE sends 'ADD_STA' message with this mode updated.
    */

    tStaRateMode        opRateMode;
    // 11b, 11a and aniLegacyRates are IE rates which gives rate in unit of 500Kbps
    tANI_U16             llbRates[SIR_NUM_11B_RATES];
    tANI_U16             llaRates[SIR_NUM_11A_RATES];

    /*
    * 0-76 bits used, remaining reserved
    * bits 0-15 and 32 should be set.
    */
    tANI_U8 supportedMCSSet[SIR_MAC_MAX_SUPPORTED_MCS_SET];

    /*
     * RX Highest Supported Data Rate defines the highest data
     * rate that the STA is able to receive, in unites of 1Mbps.
     * This value is derived from "Supported MCS Set field" inside
     * the HT capability element.
     */
    tANI_U16 rxHighestDataRate;

#ifdef WLAN_FEATURE_11AC
   /*Indicates the Maximum MCS that can be received for each number
        of spacial streams */
    tANI_U16 vhtRxMCSMap;
   /*Indicate the highest VHT data rate that the STA is able to receive*/
    tANI_U16 vhtRxHighestDataRate;
   /*Indicates the Maximum MCS that can be transmitted for each number
        of spacial streams */
    tANI_U16 vhtTxMCSMap;
   /*Indicate the highest VHT data rate that the STA is able to transmit*/
    tANI_U16 vhtTxHighestDataRate;
#endif
} tSirSupportedRates, *tpSirSupportedRates;


typedef enum eSirRFBand
{
    SIR_BAND_UNKNOWN,
    SIR_BAND_2_4_GHZ,
    SIR_BAND_5_GHZ,
} tSirRFBand;


/*
* Specifies which beacons are to be indicated up to the host driver when
* Station is in power save mode.
*/
typedef enum eBeaconForwarding
{
    ePM_BEACON_FWD_NTH,
    ePM_BEACON_FWD_TIM,
    ePM_BEACON_FWD_DTIM,
    ePM_BEACON_FWD_NONE
} tBeaconForwarding;


typedef struct sSirRemainOnChnReq
{
    tANI_U16 messageType;
    tANI_U16 length;
    tANI_U8 sessionId;
    tSirMacAddr selfMacAddr;
    tANI_U8  chnNum;
    tANI_U8  phyMode;
    tANI_U32 duration;
    tANI_U8  isProbeRequestAllowed;
    tANI_U8  probeRspIe[1];
}tSirRemainOnChnReq, *tpSirRemainOnChnReq;

typedef struct sSirRegisterMgmtFrame
{
    tANI_U16 messageType;
    tANI_U16 length;
    tANI_U8 sessionId;
    tANI_BOOLEAN registerFrame;
    tANI_U16 frameType;
    tANI_U16 matchLen;
    tANI_U8  matchData[1];
}tSirRegisterMgmtFrame, *tpSirRegisterMgmtFrame;

/// Generic type for sending a response message
/// with result code to host software
typedef struct sSirSmeRsp
{
    tANI_U16             messageType; // eWNI_SME_*_RSP
    tANI_U16             length;
    tANI_U8              sessionId;  // To support BT-AMP
    tANI_U16             transactionId;   // To support BT-AMP
    tSirResultCodes statusCode;
} tSirSmeRsp, *tpSirSmeRsp;

/// Definition for kick starting Firmware on STA
typedef struct sSirSmeStartReq
{
    tANI_U16   messageType;      // eWNI_SME_START_REQ
    tANI_U16   length;
    tANI_U8      sessionId;      //Added for BT-AMP Support
    tANI_U16     transcationId;  //Added for BT-AMP Support
    tSirMacAddr  bssId;          //Added For BT-AMP Support
    tANI_U32   sendNewBssInd;
} tSirSmeStartReq, *tpSirSmeStartReq;

/// Definition for indicating all modules ready on STA
typedef struct sSirSmeReadyReq
{
    tANI_U16   messageType; // eWNI_SME_SYS_READY_IND
    tANI_U16   length;
    tANI_U16   transactionId;
} tSirSmeReadyReq, *tpSirSmeReadyReq;

/// Definition for response message to previously issued start request
typedef struct sSirSmeStartRsp
{
    tANI_U16             messageType; // eWNI_SME_START_RSP
    tANI_U16             length;
    tSirResultCodes statusCode;
    tANI_U16             transactionId;
} tSirSmeStartRsp, *tpSirSmeStartRsp;


/// Definition for Load structure
typedef struct sSirLoad
{
    tANI_U16             numStas;
    tANI_U16             channelUtilization;
} tSirLoad, *tpSirLoad;

/// BSS type enum used in while scanning/joining etc
typedef enum eSirBssType
{
    eSIR_INFRASTRUCTURE_MODE,
    eSIR_INFRA_AP_MODE,                    //Added for softAP support
    eSIR_IBSS_MODE,
    eSIR_BTAMP_STA_MODE,                     //Added for BT-AMP support
    eSIR_BTAMP_AP_MODE,                     //Added for BT-AMP support
    eSIR_AUTO_MODE,
    eSIR_DONOT_USE_BSS_TYPE = SIR_MAX_ENUM_SIZE
} tSirBssType;

/// Power Capability info used in 11H
typedef struct sSirMacPowerCapInfo
{
    tANI_U8              minTxPower;
    tANI_U8              maxTxPower;
} tSirMacPowerCapInfo, *tpSirMacPowerCapInfo;

/// Supported Channel info used in 11H
typedef struct sSirSupChnl
{
    tANI_U8              numChnl;
    tANI_U8              channelList[SIR_MAX_SUPPORTED_CHANNEL_LIST];
} tSirSupChnl, *tpSirSupChnl;

typedef enum eSirNwType
{
    eSIR_11A_NW_TYPE,
    eSIR_11B_NW_TYPE,
    eSIR_11G_NW_TYPE,
    eSIR_11N_NW_TYPE,
#ifdef WLAN_FEATURE_11AC
    eSIR_11AC_NW_TYPE,
#endif
    eSIR_DONOT_USE_NW_TYPE = SIR_MAX_ENUM_SIZE
} tSirNwType;

/// Definition for new iBss peer info
typedef struct sSirNewIbssPeerInfo
{
    tSirMacAddr    peerAddr;
    tANI_U16            aid;
} tSirNewIbssPeerInfo, *tpSirNewIbssPeerInfo;

/// Definition for Alternate BSS info
typedef struct sSirAlternateRadioInfo
{
    tSirMacAddr    bssId;
    tANI_U8             channelId;
} tSirAlternateRadioInfo, *tpSirAlternateRadioInfo;

/// Definition for Alternate BSS list
typedef struct sSirAlternateRadioList
{
    tANI_U8                       numBss;
    tSirAlternateRadioInfo   alternateRadio[1];
} tSirAlternateRadioList, *tpSirAlternateRadioList;

//HT configuration values
typedef __ani_attr_pre_packed struct sSirHtConfig
{
   //Enable/Disable receiving LDPC coded packets
   tANI_U32 ht_rx_ldpc:1;
   //Enable/Disable TX STBC
   tANI_U32 ht_tx_stbc:1;
   //Enable/Disable RX STBC
   tANI_U32 ht_rx_stbc:2;
   //Enable/Disable SGI
   tANI_U32 ht_sgi:1;
   tANI_U32 unused:27;
} __ani_attr_packed tSirHTConfig, *tpSirHTConfig;

typedef __ani_attr_pre_packed struct sSirAddIeParams{
      tANI_U16       probeRespDataLen;
      tANI_U8       *probeRespData_buff;
      tANI_U16       assocRespDataLen;
      tANI_U8       *assocRespData_buff;
      tANI_U16       probeRespBCNDataLen;
      tANI_U8       *probeRespBCNData_buff;
} tSirAddIeParams, *tpSirAddIeParams;

/// Definition for kick starting BSS
/// ---> MAC
/**
 * Usage of ssId, numSSID & ssIdList:
 * ---------------------------------
 * 1. ssId.length of zero indicates that Broadcast/Suppress SSID
 *    feature is enabled.
 * 2. If ssId.length is zero, MAC SW will advertise NULL SSID
 *    and interpret the SSID list from numSSID & ssIdList.
 * 3. If ssId.length is non-zero, MAC SW will advertise the SSID
 *    specified in the ssId field and it is expected that
 *    application will set numSSID to one (only one SSID present
 *    in the list) and SSID in the list is same as ssId field.
 * 4. Application will always set numSSID >= 1.
 */
//*****NOTE: Please make sure all codes are updated if inserting field into this structure..**********
typedef struct sSirSmeStartBssReq
{
    tANI_U16                messageType;       // eWNI_SME_START_BSS_REQ
    tANI_U16                length;
    tANI_U8                 sessionId;       //Added for BT-AMP Support
    tANI_U16                transactionId;   //Added for BT-AMP Support
    tSirMacAddr             bssId;           //Added for BT-AMP Support
    tSirMacAddr             selfMacAddr;     //Added for BT-AMP Support
    tANI_U16                beaconInterval;  //Added for BT-AMP Support
    tANI_U8                 dot11mode;
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
    tANI_U8                 cc_switch_mode;
#endif
    tSirBssType             bssType;
    tSirMacSSid             ssId;
    tANI_U8                 channelId;
    ePhyChanBondState       cbMode;
    tANI_U8                 vht_channel_width;

    tANI_U8                 privacy;
    tANI_U8                 apUapsdEnable;
    tANI_U8                 ssidHidden;
    tANI_BOOLEAN            fwdWPSPBCProbeReq;
    tANI_BOOLEAN            protEnabled;
    tANI_BOOLEAN            obssProtEnabled;
    tANI_U16                ht_capab;
    tAniAuthType            authType;
    tANI_U32                dtimPeriod;
    tANI_U8                 wps_state;

    /* Coalescing on/off knob */
    tANI_U8                 isCoalesingInIBSSAllowed;
    tVOS_CON_MODE           bssPersona;

    tANI_U8                 txLdpcIniFeatureEnabled;

    tSirRSNie               rsnIE;             // RSN IE to be sent in
                                               // Beacon and Probe
                                               // Response frames
    tSirNwType              nwType;            // Indicates 11a/b/g
    tSirMacRateSet          operationalRateSet;// Has 11a or 11b rates
    tSirMacRateSet          extendedRateSet;    // Has 11g rates
    tSirHTConfig            htConfig;

#ifdef WLAN_FEATURE_11W
    tANI_BOOLEAN            pmfCapable;
    tANI_BOOLEAN            pmfRequired;
#endif

    tSirAddIeParams         addIeParams;

    tANI_BOOLEAN            obssEnabled;
    uint8_t                 sap_dot11mc;

} tSirSmeStartBssReq, *tpSirSmeStartBssReq;

#define GET_IE_LEN_IN_BSS(lenInBss) ( lenInBss + sizeof(lenInBss) - \
              ((uintptr_t)OFFSET_OF( tSirBssDescription, ieFields)))

#define WSCIE_PROBE_RSP_LEN (317 + 2)

typedef struct sSirBssDescription
{
    //offset of the ieFields from bssId.
    tANI_U16             length;
    tSirMacAddr          bssId;
    v_TIME_t             scanSysTimeMsec;
    tANI_U32             timeStamp[2];
    tANI_U16             beaconInterval;
    tANI_U16             capabilityInfo;
    tSirNwType           nwType; // Indicates 11a/b/g
    tANI_S8              rssi;
    tANI_S8              rssi_raw;
    tANI_S8              sinr;
    //channelId what peer sent in beacon/probersp.
    tANI_U8              channelId;
    //channelId on which we are parked at.
    //used only in scan case.
    tANI_U8              channelIdSelf;
    tANI_U8              sSirBssDescriptionRsvd[3];
    tANI_TIMESTAMP nReceivedTime;     //base on a tick count. It is a time stamp, not a relative time.
#if defined WLAN_FEATURE_VOWIFI
    tANI_U32       parentTSF;
    tANI_U32       startTSF[2];
#endif
#ifdef WLAN_FEATURE_VOWIFI_11R
    tANI_U8              mdiePresent;
    tANI_U8              mdie[SIR_MDIE_SIZE];                // MDIE for 11r, picked from the beacons
#endif
#ifdef FEATURE_WLAN_ESE
    tANI_U16             QBSSLoad_present;
    tANI_U16             QBSSLoad_avail;
    tANI_U32             reservedPadding5; // To achieve 8-byte alignment with ESE enabled
#endif
    // Please keep the structure 4 bytes aligned above the ieFields

    tANI_U8              fProbeRsp; //whether it is from a probe rsp
    tANI_U8              reservedPadding1;
    tANI_U8              reservedPadding2;
    tANI_U8              reservedPadding3;
    tANI_U32             WscIeLen;
    tANI_U8              WscIeProbeRsp[WSCIE_PROBE_RSP_LEN];
    tANI_U8              reservedPadding4;
    tANI_U32             tsf_delta;

    tANI_U32             ieFields[1];
} tSirBssDescription, *tpSirBssDescription;

#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
typedef struct sSirSmeHTProfile
{
    tANI_U8             dot11mode;
    tANI_U8             htCapability;
    tANI_U8             htSupportedChannelWidthSet;
    tANI_U8             htRecommendedTxWidthSet;
    ePhyChanBondState   htSecondaryChannelOffset;
#ifdef WLAN_FEATURE_11AC
    tANI_U8             vhtCapability;
    tANI_U8             vhtTxChannelWidthSet;
    tANI_U8             apCenterChan;
    tANI_U8             apChanWidth;
#endif
} tSirSmeHTProfile;
#endif
/// Definition for response message to previously
/// issued start BSS request
/// MAC --->
typedef struct sSirSmeStartBssRsp
{
    tANI_U16            messageType; // eWNI_SME_START_BSS_RSP
    tANI_U16            length;
    tANI_U8             sessionId;
    tANI_U16            transactionId;//transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirBssType         bssType;//Add new type for WDS mode
    tANI_U16            beaconInterval;//Beacon Interval for both type
    tANI_U32            staId;  /* Station ID for Self */
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
    tSirSmeHTProfile    HTProfile;
#endif
    tSirBssDescription  bssDescription;//Peer BSS description
} tSirSmeStartBssRsp, *tpSirSmeStartBssRsp;


typedef struct sSirChannelList
{
    tANI_U8          numChannels;
    tANI_U8          channelNumber[SIR_ESE_MAX_MEAS_IE_REQS];
} tSirChannelList, *tpSirChannelList;

typedef struct sSirDFSChannelList
{
    tANI_U32         timeStamp[SIR_MAX_24G_5G_CHANNEL_RANGE];

} tSirDFSChannelList, *tpSirDFSChannelList;

#ifdef FEATURE_WLAN_ESE
typedef struct sTspecInfo {
    tANI_U8         valid;
    tSirMacTspecIE  tspec;
} tTspecInfo;

#define SIR_ESE_MAX_TSPEC_IES   4
typedef struct sESETspecTspecInfo {
    tANI_U8 numTspecs;
    tTspecInfo tspec[SIR_ESE_MAX_TSPEC_IES];
} tESETspecInfo;
#endif


/// Definition for Radar Info
typedef struct sSirRadarInfo
{
    tANI_U8          channelNumber;
    tANI_U16         radarPulseWidth; // in usecond
    tANI_U16         numRadarPulse;
} tSirRadarInfo, *tpSirRadarInfo;

#define SIR_RADAR_INFO_SIZE                (sizeof(tANI_U8) + 2 *sizeof(tANI_U16))

/// Two Background Scan mode
typedef enum eSirBackgroundScanMode
{
    eSIR_AGGRESSIVE_BACKGROUND_SCAN = 0,
    eSIR_NORMAL_BACKGROUND_SCAN = 1,
    eSIR_ROAMING_SCAN = 2,
} tSirBackgroundScanMode;

/// Two types of traffic check
typedef enum eSirLinkTrafficCheck
{
    eSIR_DONT_CHECK_LINK_TRAFFIC_BEFORE_SCAN = 0,
    eSIR_CHECK_LINK_TRAFFIC_BEFORE_SCAN = 1,
    eSIR_CHECK_ROAMING_SCAN = 2,
} tSirLinkTrafficCheck;

#define SIR_BG_SCAN_RETURN_CACHED_RESULTS              0x0
#define SIR_BG_SCAN_PURGE_RESUTLS                      0x80
#define SIR_BG_SCAN_RETURN_FRESH_RESULTS               0x01
#define SIR_SCAN_MAX_NUM_SSID                          0x0A
#define SIR_BG_SCAN_RETURN_LFR_CACHED_RESULTS          0x02
#define SIR_BG_SCAN_PURGE_LFR_RESULTS                  0x40

/// Definition for scan request
typedef struct sSirSmeScanReq
{
    tANI_U16        messageType; // eWNI_SME_SCAN_REQ
    tANI_U16        length;
    tANI_U8         sessionId;         // Session ID
    tANI_U16        transactionId;     // Transaction ID for cmd
    tSirMacAddr     bssId;
    tSirMacSSid     ssId[SIR_SCAN_MAX_NUM_SSID];
    tSirMacAddr     selfMacAddr; //Added For BT-AMP Support
    tSirBssType     bssType;
    tANI_U8         dot11mode;
    tSirScanType    scanType;
    /**
     * minChannelTime. Not used if scanType is passive.
     * 0x0 - Dont Use min channel timer. Only max channel timeout will used.
     *       11k measurements set this to zero to user only single duration for scan.
     * <valid timeout> - Timeout value used for min channel timeout.
     */
    tANI_U32        minChannelTime;
    /**
     * maxChannelTime.
     * 0x0 - Invalid. In case of active scan.
     * In case of passive scan, MAX( maxChannelTime, WNI_CFG_PASSIVE_MAXIMUM_CHANNEL_TIME) is used.
     *
     */
    tANI_U32        maxChannelTime;
    /**
     * returnAfterFirstMatch can take following values:
     * 0x00 - Return SCAN_RSP message after complete channel scan
     * 0x01 -  Return SCAN_RSP message after collecting BSS description
     *        that matches scan criteria.
     * 0xC0 - Return after collecting first 11d IE from 2.4 GHz &
     *        5 GHz band channels
     * 0x80 - Return after collecting first 11d IE from 5 GHz band
     *        channels
     * 0x40 - Return after collecting first 11d IE from 2.4 GHz
     *        band channels
     *
     * Values of 0xC0, 0x80 & 0x40 are to be used by
     * Roaming/application when 11d is enabled.
     */
    tANI_U32 minChannelTimeBtc;    //in units of milliseconds
    tANI_U32 maxChannelTimeBtc;    //in units of milliseconds
    tANI_U32 restTime;              //in units of milliseconds, ignored when not connected
    /*in units of milliseconds, ignored when not connected*/
    uint32_t min_rest_time;
    /*in units of milliseconds, ignored when not connected*/
    uint32_t idle_time;

    tANI_U8              returnAfterFirstMatch;

    /**
     * returnUniqueResults can take following values:
     * 0 - Collect & report all received BSS descriptions from same BSS.
     * 1 - Collect & report unique BSS description from same BSS.
     */
    tANI_U8              returnUniqueResults;

    /**
     * returnFreshResults can take following values:
     * 0x00 - Return background scan results.
     * 0x80 - Return & purge background scan results
     * 0x01 - Trigger fresh scan instead of returning background scan
     *        results.
     * 0x81 - Trigger fresh scan instead of returning background scan
     *        results and purge background scan results.
     */
    tANI_U8              returnFreshResults;

    /*  backgroundScanMode can take following values:
     *  0x0 - aggressive scan
     *  0x1 - normal scan where HAL will check for link traffic
     *        prior to proceeding with the scan
     */
    tSirBackgroundScanMode   backgroundScanMode;

    tANI_U8              hiddenSsid;

    /* Number of SSIDs to scan */
    tANI_U8             numSsid;

    //channelList has to be the last member of this structure. Check tSirChannelList for the reason.
    /* This MUST be the last field of the structure */


    tANI_BOOLEAN         p2pSearch;
    tANI_U16             uIEFieldLen;
    tANI_U16             uIEFieldOffset;

    //channelList MUST be the last field of this structure
    tSirChannelList channelList;
    /*-----------------------------
      tSirSmeScanReq....
      -----------------------------
      uIEFiledLen
      -----------------------------
      uIEFiledOffset               ----+
      -----------------------------    |
      channelList.numChannels          |
      -----------------------------    |
      ... variable size up to          |
      channelNumber[numChannels-1]     |
      This can be zero, if             |
      numChannel is zero.              |
      ----------------------------- <--+
      ... variable size uIEFiled
      up to uIEFieldLen (can be 0)
      -----------------------------*/
} tSirSmeScanReq, *tpSirSmeScanReq;

typedef struct sSirSmeScanAbortReq
{
    tANI_U16        type;
    tANI_U16        msgLen;
    tANI_U8         sessionId;
} tSirSmeScanAbortReq, *tpSirSmeScanAbortReq;

typedef struct sSirSmeScanChanReq
{
    tANI_U16        type;
    tANI_U16        msgLen;
    tANI_U8         sessionId;
    tANI_U16        transcationId;
} tSirSmeGetScanChanReq, *tpSirSmeGetScanChanReq;

#ifdef FEATURE_OEM_DATA_SUPPORT

#ifndef OEM_DATA_REQ_SIZE
#define OEM_DATA_REQ_SIZE 280
#endif
#ifndef OEM_DATA_RSP_SIZE
#define OEM_DATA_RSP_SIZE 1724
#endif

typedef struct sSirOemDataReq
{
    tANI_U16              messageType; /* eWNI_SME_OEM_DATA_REQ */
    tANI_U16              messageLen;
    tSirMacAddr           selfMacAddr;
    uint8_t               data_len;
    uint8_t               *data;
} tSirOemDataReq, *tpSirOemDataReq;

typedef struct sSirOemDataRsp
{
    tANI_U16             messageType;
    tANI_U16             length;
    tANI_U8              oemDataRsp[OEM_DATA_RSP_SIZE];
} tSirOemDataRsp, *tpSirOemDataRsp;

#endif //FEATURE_OEM_DATA_SUPPORT

/// Definition for response message to previously issued scan request
typedef struct sSirSmeScanRsp
{
    tANI_U16           messageType; // eWNI_SME_SCAN_RSP
    tANI_U16           length;
    tANI_U8            sessionId;
    tSirResultCodes    statusCode;
    tANI_U16           transcationId;
    tSirBssDescription bssDescription[1];
} tSirSmeScanRsp, *tpSirSmeScanRsp;

/// Sme Req message to set the Background Scan mode
typedef struct sSirSmeBackgroundScanModeReq
{
    tANI_U16                      messageType; // eWNI_SME_BACKGROUND_SCAN_MODE_REQ
    tANI_U16                      length;
    tSirBackgroundScanMode   mode;
} tSirSmeBackgroundScanModeReq, *tpSirSmeBackgroundScanModeReq;

/* Background Scan Statistics */
typedef struct sSirBackgroundScanInfo {
    tANI_U32        numOfScanSuccess;
    tANI_U32        numOfScanFailure;
    tANI_U32        reserved;
} tSirBackgroundScanInfo, *tpSirBackgroundScanInfo;

#define SIR_BACKGROUND_SCAN_INFO_SIZE        (3 * sizeof(tANI_U32))

/* Definition for Join/Reassoc info */
typedef struct sJoinReassocInfo
{
    tAniBool            spectrumMgtIndicator;
    tSirMacPowerCapInfo powerCap;
    tSirSupChnl         supportedChannels;
} tJoinReassocInfo, *tpJoinReassocInfo;

/// Definition for join request
/// ---> MAC
/// WARNING! If you add a field in JOIN REQ.
///         Make sure to add it in REASSOC REQ
/// The Serdes function is the same and its
/// shared with REASSOC. So if we add a field
//  here and dont add it in REASSOC REQ. It will BREAK!!! REASSOC.
typedef struct sSirSmeJoinReq
{
    tANI_U16            messageType;            // eWNI_SME_JOIN_REQ
    tANI_U16            length;
    tANI_U8             sessionId;
    tANI_U16            transactionId;
    tSirMacSSid         ssId;
    tSirMacAddr         selfMacAddr;            // self Mac address
    tSirBssType         bsstype;                // add new type for BT -AMP STA and AP Modules
    tANI_U8             dot11mode;              // to support BT-AMP
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
    tANI_U8             cc_switch_mode;
#endif
    tVOS_CON_MODE       staPersona;             //Persona
    ePhyChanBondState   cbMode;                 // Pass CB mode value in Join.

    /*This contains the UAPSD Flag for all 4 AC
     * B0: AC_VO UAPSD FLAG
     * B1: AC_VI UAPSD FLAG
     * B2: AC_BK UAPSD FLAG
     * B3: AC_BE UASPD FLAG
     */
    tANI_U8                 uapsdPerAcBitmask;

    tSirMacRateSet      operationalRateSet;// Has 11a or 11b rates
    tSirMacRateSet      extendedRateSet;    // Has 11g rates
    tSirRSNie           rsnIE;                  // RSN IE to be sent in
                                                // (Re) Association Request
#ifdef FEATURE_WLAN_ESE
    tSirCCKMie          cckmIE;             // CCMK IE to be included as handler for join and reassoc is
                                            // the same. The join will never carry cckm, but will be set to
                                            // 0.
#endif

    tSirAddie           addIEScan;              // Additional IE to be sent in
                                                // (unicast) Probe Request at the time of join

    tSirAddie           addIEAssoc;             // Additional IE to be sent in
                                                // (Re) Association Request

    tAniEdType          UCEncryptionType;

    tAniEdType          MCEncryptionType;

#ifdef WLAN_FEATURE_11W
    tAniEdType          MgmtEncryptionType;
#endif

#ifdef WLAN_FEATURE_VOWIFI_11R
    tAniBool            is11Rconnection;
#endif
#ifdef FEATURE_WLAN_ESE
    tAniBool            isESEFeatureIniEnabled;
    tAniBool            isESEconnection;
    tESETspecInfo       eseTspecInfo;
#endif

#if defined WLAN_FEATURE_VOWIFI_11R || defined FEATURE_WLAN_ESE || defined(FEATURE_WLAN_LFR)
    tAniBool            isFastTransitionEnabled;
#endif
#ifdef FEATURE_WLAN_LFR
    tAniBool            isFastRoamIniFeatureEnabled;
#endif

    tANI_U8             txLdpcIniFeatureEnabled;
    tSirHTConfig        htConfig;
#ifdef WLAN_FEATURE_11AC
    tANI_U8             txBFIniFeatureEnabled;
    tANI_U8             txBFCsnValue;
    tANI_U8             txMuBformee;
    tANI_U8             enableVhtpAid;
    tANI_U8             enableVhtGid;
#endif
    tANI_U8             enableAmpduPs;
    tANI_U8             enableHtSmps;
    tANI_U8             htSmps;

    tANI_U8             max_amsdu_num;
    tAniBool            isWMEenabled;
    tAniBool            isQosEnabled;
    tAniBool            isOSENConnection;
    struct rrm_config_param rrm_config;
    tAniBool            spectrumMgtIndicator;
    tSirMacPowerCapInfo powerCap;
    tSirSupChnl         supportedChannels;
    tSirBssDescription  bssDescription;

} tSirSmeJoinReq, *tpSirSmeJoinReq;

/* Definition for response message to previously issued join request */
typedef struct sSirSmeJoinRsp
{
    tANI_U16                messageType; // eWNI_SME_JOIN_RSP
    tANI_U16                length;
    tANI_U8                 sessionId;         // Session ID
    tANI_U16                transactionId;     // Transaction ID for cmd
    tSirResultCodes         statusCode;
    tAniAuthType            authType;
    tANI_U32                vht_channel_width;
    tANI_U16        protStatusCode; //It holds reasonCode when join fails due to deauth/disassoc frame.
                                    //Otherwise it holds status code.
    tANI_U16        aid;
    tANI_U32        beaconLength;
    tANI_U32        assocReqLength;
    tANI_U32        assocRspLength;
#ifdef WLAN_FEATURE_VOWIFI_11R
    tANI_U32        parsedRicRspLen;
#endif
#ifdef FEATURE_WLAN_ESE
    tANI_U32        tspecIeLen;
#endif
    tANI_U32        staId;//Station ID for peer

    /*The DPU signatures will be sent eventually to TL to help it determine the
      association to which a packet belongs to*/
    /*Unicast DPU signature*/
    tANI_U8            ucastSig;

    /*Broadcast DPU signature*/
    tANI_U8            bcastSig;

    /* Timing and fine Timing measurement capability clubbed together */
    tANI_U8            timingMeasCap;

#ifdef FEATURE_WLAN_TDLS
    /* TDLS prohibited and TDLS channel switch prohibited are set as
     * per ExtCap IE in received assoc/re-assoc response from AP
     */
    bool tdls_prohibited;
    bool tdls_chan_swit_prohibited;
#endif

    uint8_t         nss;
    uint32_t        max_rate_flags;

#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
    tSirSmeHTProfile    HTProfile;
#endif

    bool supported_nss_1x1;
    /* Add new members before 'frames' to avoid memory corruption of 'frames' */
    tANI_U8         frames[ 1 ];
} tSirSmeJoinRsp, *tpSirSmeJoinRsp;

/// Definition for Authentication indication from peer
typedef struct sSirSmeAuthInd
{
    tANI_U16           messageType; // eWNI_SME_AUTH_IND
    tANI_U16           length;
    tANI_U8            sessionId;
    tSirMacAddr        bssId;             // Self BSSID
    tSirMacAddr        peerMacAddr;
    tAniAuthType       authType;
} tSirSmeAuthInd, *tpSirSmeAuthInd;

/// probereq from peer, when wsc is enabled
typedef struct sSirSmeProbereq
{
    tANI_U16           messageType; // eWNI_SME_PROBE_REQ
    tANI_U16           length;
    tANI_U8            sessionId;
    tSirMacAddr        peerMacAddr;
    tANI_U16           devicePasswdId;
} tSirSmeProbeReq, *tpSirSmeProbeReq;

typedef struct sSirSmeChanInfo
{
    /* channel id */
    tANI_U8 chan_id;
    /* primary 20 MHz channel frequency in mhz */
    tANI_U32 mhz;
    /* Center frequency 1 in MHz */
    tANI_U32 band_center_freq1;
    /* Center frequency 2 in MHz - valid only for 11acvht 80plus80 mode */
    tANI_U32 band_center_freq2;
    /* channel info described below */
    tANI_U32 info;
    /* contains min power, max power, reg power and reg class id */
    tANI_U32 reg_info_1;
    /* contains antennamax */
    tANI_U32 reg_info_2;
    /* number of spatial streams */
    uint8_t  nss;
    /* rate flags */
    uint32_t rate_flags;
} tSirSmeChanInfo, *tpSirSmeChanInfo;
/// Definition for Association indication from peer
/// MAC --->
typedef struct sSirSmeAssocInd
{
    tANI_U16             messageType; // eWNI_SME_ASSOC_IND
    tANI_U16             length;
    tANI_U8              sessionId;
    tSirMacAddr          peerMacAddr;
    tANI_U16             aid;
    tSirMacAddr          bssId; // Self BSSID
    tANI_U16             staId; // Station ID for peer
    tANI_U8              uniSig;  // DPU signature for unicast packets
    tANI_U8              bcastSig; // DPU signature for broadcast packets
    tAniAuthType         authType;
    tAniSSID             ssId; // SSID used by STA to associate
    tSirWAPIie           wapiIE;//WAPI IE received from peer
    tSirRSNie            rsnIE;// RSN IE received from peer
    tSirAddie            addIE;// Additional IE received from peer, which possibly include WSC IE and/or P2P IE

    // powerCap & supportedChannels are present only when
    // spectrumMgtIndicator flag is set
    tAniBool                spectrumMgtIndicator;
    tSirMacPowerCapInfo     powerCap;
    tSirSupChnl             supportedChannels;
    tAniBool             wmmEnabledSta; /* if present - STA is WMM enabled */
    tAniBool             reassocReq;
    // Required for indicating the frames to upper layer
    tANI_U32             beaconLength;
    tANI_U8*             beaconPtr;
    tANI_U32             assocReqLength;
    tANI_U8*             assocReqPtr;

    /* Timing and fine Timing measurement capability clubbed together */
    tANI_U8              timingMeasCap;
    tSirSmeChanInfo      chan_info;
} tSirSmeAssocInd, *tpSirSmeAssocInd;


/// Definition for Association confirm
/// ---> MAC
typedef struct sSirSmeAssocCnf
{
    tANI_U16             messageType; // eWNI_SME_ASSOC_CNF
    tANI_U16             length;
    tSirResultCodes      statusCode;
    tSirMacAddr          bssId;      // Self BSSID
    tSirMacAddr          peerMacAddr;
    tANI_U16             aid;
    tSirMacAddr          alternateBssId;
    tANI_U8              alternateChannelId;
} tSirSmeAssocCnf, *tpSirSmeAssocCnf;

/// Definition for Reassociation indication from peer
typedef struct sSirSmeReassocInd
{
    tANI_U16            messageType; // eWNI_SME_REASSOC_IND
    tANI_U16            length;
    tANI_U8             sessionId;         // Session ID
    tSirMacAddr         peerMacAddr;
    tSirMacAddr         oldMacAddr;
    tANI_U16            aid;
    tSirMacAddr         bssId;           // Self BSSID
    tANI_U16            staId;           // Station ID for peer
    tAniAuthType        authType;
    tAniSSID            ssId;   // SSID used by STA to reassociate
    tSirRSNie           rsnIE;  // RSN IE received from peer

    tSirAddie           addIE;  // Additional IE received from peer

    // powerCap & supportedChannels are present only when
    // spectrumMgtIndicator flag is set
    tAniBool                spectrumMgtIndicator;
    tSirMacPowerCapInfo     powerCap;
    tSirSupChnl             supportedChannels;
    // Required for indicating the frames to upper layer
    // TODO: use the appropriate names to distinguish between the other similar names used above for station mode of operation
    tANI_U32             beaconLength;
    tANI_U8*             beaconPtr;
    tANI_U32             assocReqLength;
    tANI_U8*             assocReqPtr;
} tSirSmeReassocInd, *tpSirSmeReassocInd;

/// Definition for Reassociation confirm
/// ---> MAC
typedef struct sSirSmeReassocCnf
{
    tANI_U16                  messageType; // eWNI_SME_REASSOC_CNF
    tANI_U16                  length;
    tSirResultCodes      statusCode;
    tSirMacAddr          bssId;             // Self BSSID
    tSirMacAddr          peerMacAddr;
    tANI_U16                  aid;
    tSirMacAddr          alternateBssId;
    tANI_U8                   alternateChannelId;
} tSirSmeReassocCnf, *tpSirSmeReassocCnf;


/// Enum definition for  Wireless medium status change codes
typedef enum eSirSmeStatusChangeCode
{
    eSIR_SME_DEAUTH_FROM_PEER,
    eSIR_SME_DISASSOC_FROM_PEER,
    eSIR_SME_LOST_LINK_WITH_PEER,
    eSIR_SME_CHANNEL_SWITCH,
    eSIR_SME_JOINED_NEW_BSS,
    eSIR_SME_LEAVING_BSS,
    eSIR_SME_IBSS_ACTIVE,
    eSIR_SME_IBSS_INACTIVE,
    eSIR_SME_IBSS_PEER_DEPARTED,
    eSIR_SME_RADAR_DETECTED,
    eSIR_SME_IBSS_NEW_PEER,
    eSIR_SME_AP_CAPS_CHANGED,
    eSIR_SME_BACKGROUND_SCAN_FAIL,
    eSIR_SME_CB_LEGACY_BSS_FOUND_BY_AP,
    eSIR_SME_CB_LEGACY_BSS_FOUND_BY_STA
} tSirSmeStatusChangeCode;

typedef struct sSirSmeNewBssInfo
{
    tSirMacAddr   bssId;
    tANI_U8            channelNumber;
    tANI_U8            reserved;
    tSirMacSSid   ssId;
} tSirSmeNewBssInfo, *tpSirSmeNewBssInfo;

typedef struct sSirSmeApNewCaps
{
    tANI_U16           capabilityInfo;
    tSirMacAddr   bssId;
    tANI_U8            channelId;
    tANI_U8            reserved[3];
    tSirMacSSid   ssId;
} tSirSmeApNewCaps, *tpSirSmeApNewCaps;

/**
 * Table below indicates what information is passed for each of
 * the Wireless Media status change notifications:
 *
 * Status Change code           Status change info
 * ----------------------------------------------------------------------
 * eSIR_SME_DEAUTH_FROM_PEER        Reason code received in DEAUTH frame
 * eSIR_SME_DISASSOC_FROM_PEER      Reason code received in DISASSOC frame
 * eSIR_SME_LOST_LINK_WITH_PEER     None
 * eSIR_SME_CHANNEL_SWITCH          New channel number
 * eSIR_SME_JOINED_NEW_BSS          BSSID, SSID and channel number
 * eSIR_SME_LEAVING_BSS             None
 * eSIR_SME_IBSS_ACTIVE             Indicates that another STA joined
 *                                  IBSS apart from this STA that
 *                                  started IBSS
 * eSIR_SME_IBSS_INACTIVE           Indicates that only this STA is left
 *                                  in IBSS
 * eSIR_SME_RADAR_DETECTED          Indicates that radar is detected
 * eSIR_SME_IBSS_NEW_PEER           Indicates that a new peer is detected
 * eSIR_SME_AP_CAPS_CHANGED         Indicates that capabilities of the AP
 *                                  that STA is currently associated with
 *                                  have changed.
 * eSIR_SME_BACKGROUND_SCAN_FAIL    Indicates background scan failure
 */

/// Definition for Wireless medium status change notification
typedef struct sSirSmeWmStatusChangeNtf
{
    tANI_U16                     messageType; // eWNI_SME_WM_STATUS_CHANGE_NTF
    tANI_U16                     length;
    tANI_U8                      sessionId;         // Session ID
    tSirSmeStatusChangeCode statusChangeCode;
    tSirMacAddr             bssId;             // Self BSSID
    union
    {
        tANI_U16                 deAuthReasonCode; // eSIR_SME_DEAUTH_FROM_PEER
        tANI_U16                 disassocReasonCode; // eSIR_SME_DISASSOC_FROM_PEER
        // none for eSIR_SME_LOST_LINK_WITH_PEER
        tANI_U8                  newChannelId;   // eSIR_SME_CHANNEL_SWITCH
        tSirSmeNewBssInfo   newBssInfo;     // eSIR_SME_JOINED_NEW_BSS
        // none for eSIR_SME_LEAVING_BSS
        // none for eSIR_SME_IBSS_ACTIVE
        // none for eSIR_SME_IBSS_INACTIVE
        tSirNewIbssPeerInfo     newIbssPeerInfo;  // eSIR_SME_IBSS_NEW_PEER
        tSirSmeApNewCaps        apNewCaps;        // eSIR_SME_AP_CAPS_CHANGED
        tSirBackgroundScanInfo  bkgndScanInfo;    // eSIR_SME_BACKGROUND_SCAN_FAIL
    } statusChangeInfo;
} tSirSmeWmStatusChangeNtf, *tpSirSmeWmStatusChangeNtf;

/// Definition for Disassociation request
typedef
__ani_attr_pre_packed
struct sSirSmeDisassocReq
{
    tANI_U16            messageType; // eWNI_SME_DISASSOC_REQ
    tANI_U16            length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirMacAddr         bssId;             // Peer BSSID
    tSirMacAddr peerMacAddr;
    tANI_U16         reasonCode;
    tANI_U8          doNotSendOverTheAir;  //This flag tells LIM whether to send the disassoc OTA or not
                                           //This will be set in while handing off from one AP to other
}
__ani_attr_packed
tSirSmeDisassocReq, *tpSirSmeDisassocReq;

/// Definition for Tkip countermeasures request
typedef __ani_attr_pre_packed struct sSirSmeTkipCntrMeasReq
{
    tANI_U16            messageType;    // eWNI_SME_DISASSOC_REQ
    tANI_U16            length;
    tANI_U8             sessionId;      // Session ID
    tANI_U16            transactionId;  // Transaction ID for cmd
    tSirMacAddr         bssId;          // Peer BSSID
    tANI_BOOLEAN        bEnable;        // Start/stop countermeasures
} __ani_attr_packed tSirSmeTkipCntrMeasReq, *tpSirSmeTkipCntrMeasReq;

typedef struct sAni64BitCounters
{
    tANI_U32 Hi;
    tANI_U32 Lo;
}tAni64BitCounters, *tpAni64BitCounters;

typedef struct sAniSecurityStat
{
    tAni64BitCounters txBlks;
    tAni64BitCounters rxBlks;
    tAni64BitCounters formatErrorCnt;
    tAni64BitCounters decryptErr;
    tAni64BitCounters protExclCnt;
    tAni64BitCounters unDecryptableCnt;
    tAni64BitCounters decryptOkCnt;

}tAniSecurityStat, *tpAniSecurityStat;

typedef struct sAniTxRxCounters
{
    tANI_U32 txFrames; // Incremented for every packet tx
    tANI_U32 rxFrames;
    tANI_U32 nRcvBytes;
    tANI_U32 nXmitBytes;
}tAniTxRxCounters, *tpAniTxRxCounters;

typedef struct sAniTxRxStats
{
    tAni64BitCounters txFrames;
    tAni64BitCounters rxFrames;
    tAni64BitCounters nRcvBytes;
    tAni64BitCounters nXmitBytes;

}tAniTxRxStats,*tpAniTxRxStats;

typedef struct sAniSecStats
{
    tAniSecurityStat aes;
    tAni64BitCounters aesReplays;
    tAniSecurityStat tkip;
    tAni64BitCounters tkipReplays;
    tAni64BitCounters tkipMicError;

    tAniSecurityStat wep;
#if defined(FEATURE_WLAN_WAPI) && !defined(LIBRA_WAPI_SUPPORT)
    tAniSecurityStat wpi;
    tAni64BitCounters wpiReplays;
    tAni64BitCounters wpiMicError;
#endif
}tAniSecStats, *tpAniSecStats;

#define SIR_MAX_RX_CHAINS 3

typedef struct sAniStaStatStruct
{
    /* following statistic elements till expandPktRxCntLo are not filled with valid data.
     * These are kept as it is, since WSM is using this structure.
     * These elements can be removed whenever WSM is updated.
     * Phystats is used to hold phystats from BD.
     */
    tANI_U32 sentAesBlksUcastHi;
    tANI_U32 sentAesBlksUcastLo;
    tANI_U32 recvAesBlksUcastHi;
    tANI_U32 recvAesBlksUcastLo;
    tANI_U32 aesFormatErrorUcastCnts;
    tANI_U32 aesReplaysUcast;
    tANI_U32 aesDecryptErrUcast;
    tANI_U32 singleRetryPkts;
    tANI_U32 failedTxPkts;
    tANI_U32 ackTimeouts;
    tANI_U32 multiRetryPkts;
    tANI_U32 fragTxCntsHi;
    tANI_U32 fragTxCntsLo;
    tANI_U32 transmittedPktsHi;
    tANI_U32 transmittedPktsLo;
    tANI_U32 phyStatHi; //These are used to fill in the phystats.
    tANI_U32 phyStatLo; //This is only for private use.

    tANI_U32 uplinkRssi;
    tANI_U32 uplinkSinr;
    tANI_U32 uplinkRate;
    tANI_U32 downlinkRssi;
    tANI_U32 downlinkSinr;
    tANI_U32 downlinkRate;
    tANI_U32 nRcvBytes;
    tANI_U32 nXmitBytes;

    /* Following elements are valid and filled in correctly. They have valid values.
     */

    //Unicast frames and bytes.
    tAniTxRxStats ucStats;

    //Broadcast frames and bytes.
    tAniTxRxStats bcStats;

    //Multicast frames and bytes.
    tAniTxRxStats mcStats;

    tANI_U32      currentTxRate;
    tANI_U32      currentRxRate; //Rate in 100Kbps

    tANI_U32      maxTxRate;
    tANI_U32      maxRxRate;

    tANI_S8       rssi[SIR_MAX_RX_CHAINS];


    tAniSecStats   securityStats;

    tANI_U8       currentRxRateIdx; //This the softmac rate Index.
    tANI_U8       currentTxRateIdx;

} tAniStaStatStruct, *tpAniStaStatStruct;

//Statistics that are not maintained per stations.
typedef struct sAniGlobalStatStruct
{
  tAni64BitCounters txError;
  tAni64BitCounters rxError;
  tAni64BitCounters rxDropNoBuffer;
  tAni64BitCounters rxDropDup;
  tAni64BitCounters rxCRCError;

  tAni64BitCounters singleRetryPkts;
  tAni64BitCounters failedTxPkts;
  tAni64BitCounters ackTimeouts;
  tAni64BitCounters multiRetryPkts;
  tAni64BitCounters fragTxCnts;
  tAni64BitCounters fragRxCnts;

  tAni64BitCounters txRTSSuccess;
  tAni64BitCounters txCTSSuccess;
  tAni64BitCounters rxRTSSuccess;
  tAni64BitCounters rxCTSSuccess;

  tAniSecStats      securityStats;

  tAniTxRxStats     mcStats;
  tAniTxRxStats     bcStats;

}tAniGlobalStatStruct,*tpAniGlobalStatStruct;

typedef enum sPacketType
{
    ePACKET_TYPE_UNKNOWN,
    ePACKET_TYPE_11A,
    ePACKET_TYPE_11G,
    ePACKET_TYPE_11B,
    ePACKET_TYPE_11N

}tPacketType, *tpPacketType;

typedef struct sAniStatSummaryStruct
{
    tAniTxRxStats uc; //Unicast counters.
    tAniTxRxStats bc; //Broadcast counters.
    tAniTxRxStats mc; //Multicast counters.
    tAni64BitCounters txError;
    tAni64BitCounters rxError;
    tANI_S8     rssi[SIR_MAX_RX_CHAINS]; //For each chain.
    tANI_U32    rxRate; // Rx rate of the last received packet.
    tANI_U32    txRate;
    tANI_U16    rxMCSId; //MCS index is valid only when packet type is ePACKET_TYPE_11N
    tANI_U16    txMCSId;
    tPacketType rxPacketType;
    tPacketType txPacketType;
    tSirMacAddr macAddr; //Mac Address of the station from which above RSSI and rate is from.
}tAniStatSummaryStruct,*tpAniStatSummaryStruct;

//structure for stats that may be reset, like the ones in sta descriptor
//The stats are saved into here before reset. It should be tANI_U32 aligned.
typedef struct _sPermStaStats
{
    tANI_U32 aesFormatErrorUcastCnts;
    tANI_U32 aesReplaysUcast;
    tANI_U32 aesDecryptErrUcast;
    tANI_U32 singleRetryPkts;
    tANI_U32 failedTxPkts;
    tANI_U32 ackTimeouts;
    tANI_U32 multiRetryPkts;
    tANI_U32 fragTxCntsHi;
    tANI_U32 fragTxCntsLo;
    tANI_U32 transmittedPktsHi;
    tANI_U32 transmittedPktsLo;
}tPermanentStaStats;


/// Definition for Disassociation response
typedef struct sSirSmeDisassocRsp
{
    tANI_U16           messageType; // eWNI_SME_DISASSOC_RSP
    tANI_U16           length;
    tANI_U8            sessionId;         // Session ID
    tANI_U16           transactionId;     // Transaction ID for cmd
    tSirResultCodes    statusCode;
    tSirMacAddr        peerMacAddr;
    tAniStaStatStruct  perStaStats; // STA stats
    tANI_U16           staId;
}
__ani_attr_packed
 tSirSmeDisassocRsp, *tpSirSmeDisassocRsp;

/// Definition for Disassociation indication from peer
typedef struct sSirSmeDisassocInd
{
    tANI_U16            messageType; // eWNI_SME_DISASSOC_IND
    tANI_U16            length;
    tANI_U8             sessionId;  // Session Identifier
    tANI_U16            transactionId;   // Transaction Identifier with PE
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;
    tSirMacAddr         peerMacAddr;
    tAniStaStatStruct  perStaStats; // STA stats
    tANI_U16            staId;
    tANI_U32            reasonCode;
} tSirSmeDisassocInd, *tpSirSmeDisassocInd;

/// Definition for Disassociation confirm
/// MAC --->
typedef struct sSirSmeDisassocCnf
{
    tANI_U16            messageType; // eWNI_SME_DISASSOC_CNF
    tANI_U16            length;
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;
    tSirMacAddr         peerMacAddr;
} tSirSmeDisassocCnf, *tpSirSmeDisassocCnf;

/* Definition for Deauthentication request */
typedef struct sSirSmeDeauthReq
{
    tANI_U16            messageType;   // eWNI_SME_DEAUTH_REQ
    tANI_U16            length;
    tANI_U8             sessionId;     // Session ID
    tANI_U16            transactionId; // Transaction ID for cmd
    tSirMacAddr         bssId;         // AP BSSID
    tSirMacAddr         peerMacAddr;
    tANI_U16            reasonCode;
} tSirSmeDeauthReq, *tpSirSmeDeauthReq;

/* Definition for Deauthentication response */
typedef struct sSirSmeDeauthRsp
{
    tANI_U16                messageType; // eWNI_SME_DEAUTH_RSP
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirMacAddr        peerMacAddr;
} tSirSmeDeauthRsp, *tpSirSmeDeauthRsp;

/* Definition for Deauthentication indication from peer */
typedef struct sSirSmeDeauthInd
{
    tANI_U16            messageType; // eWNI_SME_DEAUTH_IND
    tANI_U16            length;
    tANI_U8            sessionId;       //Added for BT-AMP
    tANI_U16            transactionId;  //Added for BT-AMP
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;// AP BSSID
    tSirMacAddr         peerMacAddr;

    tANI_U16            staId;
    tANI_U32            reasonCode;
    tANI_S8             rssi;
} tSirSmeDeauthInd, *tpSirSmeDeauthInd;

/// Definition for Deauthentication confirm
/// MAC --->
typedef struct sSirSmeDeauthCnf
{
    tANI_U16                messageType; // eWNI_SME_DEAUTH_CNF
    tANI_U16                length;
    tSirResultCodes     statusCode;
    tSirMacAddr         bssId;             // AP BSSID
    tSirMacAddr        peerMacAddr;
} tSirSmeDeauthCnf, *tpSirSmeDeauthCnf;

/// Definition for stop BSS request message
typedef struct sSirSmeStopBssReq
{
    tANI_U16                messageType;    // eWNI_SME_STOP_BSS_REQ
    tANI_U16                length;
    tANI_U8             sessionId;      //Session ID
    tANI_U16            transactionId;  //tranSaction ID for cmd
    tSirResultCodes         reasonCode;
    tSirMacAddr             bssId;          //Self BSSID
} tSirSmeStopBssReq, *tpSirSmeStopBssReq;

/// Definition for stop BSS response message
typedef struct sSirSmeStopBssRsp
{
    tANI_U16             messageType; // eWNI_SME_STOP_BSS_RSP
    tANI_U16             length;
    tSirResultCodes statusCode;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeStopBssRsp, *tpSirSmeStopBssRsp;



/// Definition for Channel Switch indication for station
/// MAC --->
typedef struct sSirSmeSwitchChannelInd
{
    tANI_U16                messageType; // eWNI_SME_SWITCH_CHL_REQ
    tANI_U16                length;
    tANI_U8                 sessionId;
    tANI_U16    newChannelId;
    tSirMacAddr        bssId;      // BSSID
} tSirSmeSwitchChannelInd, *tpSirSmeSwitchChannelInd;

/// Definition for ULA complete indication message
typedef struct sirUlaCompleteInd
{
    tANI_U16                messageType; // eWNI_ULA_COMPLETE_IND
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tSirMacAddr        peerMacAddr;
} tSirUlaCompleteInd, *tpSirUlaCompleteInd;

/// Definition for ULA complete confirmation message
typedef struct sirUlaCompleteCnf
{
    tANI_U16                messageType; // eWNI_ULA_COMPLETE_CNF
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tSirMacAddr        peerMacAddr;
} tSirUlaCompleteCnf, *tpSirUlaCompleteCnf;

/*
 * Definition for Neighbor BSS indication
 * MAC --->
 * MAC reports this each time a new I/BSS is detected
 */
typedef struct sSirSmeNeighborBssInd
{
    tANI_U16                    messageType; // eWNI_SME_NEIGHBOR_BSS_IND
    tANI_U16                    length;
    tANI_U8                     sessionId;
    tSirBssDescription     bssDescription[1];
} tSirSmeNeighborBssInd, *tpSirSmeNeighborBssInd;

/*
 * Definition for MIC failure indication
 * MAC --->
 * MAC reports this each time a MIC failure occurs on Rx TKIP packet
 */
typedef struct sSirSmeMicFailureInd
{
    tANI_U16                    messageType; // eWNI_SME_MIC_FAILURE_IND
    tANI_U16                    length;
    tANI_U8                     sessionId;
    tSirMacAddr         bssId;             // BSSID
    tSirMicFailureInfo     info;
} tSirSmeMicFailureInd, *tpSirSmeMicFailureInd;

typedef struct sSirSmeMissedBeaconInd
{
    tANI_U16                    messageType; // eWNI_SME_MISSED_BEACON_IND
    tANI_U16                    length;
    tANI_U8                     bssIdx;
} tSirSmeMissedBeaconInd, *tpSirSmeMissedBeaconInd;

/// Definition for Set Context request
/// ---> MAC
typedef struct sSirSmeSetContextReq
{
    tANI_U16           messageType; // eWNI_SME_SET_CONTEXT_REQ
    tANI_U16          length;
    tANI_U8            sessionId;  //Session ID
    tANI_U16           transactionId; //Transaction ID for cmd
    tSirMacAddr        peerMacAddr;
    tSirMacAddr        bssId;      // BSSID
    tSirKeyMaterial    keyMaterial;
} tSirSmeSetContextReq, *tpSirSmeSetContextReq;

/// Definition for Set Context response
/// MAC --->
typedef struct sSirSmeSetContextRsp
{
    tANI_U16                messageType; // eWNI_SME_SET_CONTEXT_RSP
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirMacAddr             peerMacAddr;
} tSirSmeSetContextRsp, *tpSirSmeSetContextRsp;

/// Definition for Remove Key Context request
/// ---> MAC
typedef struct sSirSmeRemoveKeyReq
{
    tANI_U16                messageType;    // eWNI_SME_REMOVE_KEY_REQ
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirMacAddr         bssId;             // BSSID
    tSirMacAddr             peerMacAddr;
    tANI_U8    edType;
    tANI_U8    wepType;
    tANI_U8    keyId;
    tANI_BOOLEAN unicast;
} tSirSmeRemoveKeyReq, *tpSirSmeRemoveKeyReq;

/// Definition for Remove Key Context response
/// MAC --->
typedef struct sSirSmeRemoveKeyRsp
{
    tANI_U16                messageType; // eWNI_SME_REMOVE_KEY_RSP
    tANI_U16                length;
    tANI_U8             sessionId;         // Session ID
    tANI_U16            transactionId;     // Transaction ID for cmd
    tSirResultCodes     statusCode;
    tSirMacAddr             peerMacAddr;
} tSirSmeRemoveKeyRsp, *tpSirSmeRemoveKeyRsp;

/// Definition for Set Power request
/// ---> MAC
typedef struct sSirSmeSetPowerReq
{
    tANI_U16                messageType; // eWNI_SME_SET_POWER_REQ
    tANI_U16                length;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tANI_S8                 powerLevel;
} tSirSmeSetPowerReq, *tpSirSmeSetPowerReq;

/// Definition for Set Power response
/// MAC --->
typedef struct sSirSmeSetPowerRsp
{
    tANI_U16                messageType; // eWNI_SME_SET_POWER_RSP
    tANI_U16                length;
    tSirResultCodes    statusCode;
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeSetPowerRsp, *tpSirSmeSetPowerRsp;


/// Definition for Link Test Start response
/// MAC --->
typedef struct sSirSmeLinkTestStartRsp
{
    tANI_U16                messageType; // eWNI_SME_LINK_TEST_START_RSP
    tANI_U16                length;
    tSirMacAddr        peerMacAddr;
    tSirResultCodes    statusCode;
} tSirSmeLinkTestStartRsp, *tpSirSmeLinkTestStartRsp;

/// Definition for Link Test Stop response
/// WSM ---> MAC
typedef struct sSirSmeLinkTestStopRsp
{
    tANI_U16                messageType; // eWNI_SME_LINK_TEST_STOP_RSP
    tANI_U16                length;
    tSirMacAddr        peerMacAddr;
    tSirResultCodes    statusCode;
} tSirSmeLinkTestStopRsp, *tpSirSmeLinkTestStopRsp;

/// Definition for kick starting DFS measurements
typedef struct sSirSmeDFSreq
{
    tANI_U16             messageType; // eWNI_SME_DFS_REQ
    tANI_U16             length;
    tANI_U16            transactionId;     // Transaction ID for cmd
} tSirSmeDFSrequest, *tpSirSmeDFSrequest;

/// Definition for response message to previously
/// issued DFS request
typedef struct sSirSmeDFSrsp
{
    tANI_U16             messageType; // eWNI_SME_DFS_RSP
    tANI_U16             length;
    tSirResultCodes statusCode;
    tANI_U16            transactionId;     // Transaction ID for cmd
    tANI_U32             dfsReport[1];
} tSirSmeDFSrsp, *tpSirSmeDFSrsp;

/// Statistic definitions
//=============================================================
// Per STA statistic structure; This same struct will be used for Aggregate
// STA stats as well.

// Clear radio stats and clear per sta stats
typedef enum
{
    eANI_CLEAR_ALL_STATS, // Clears all stats
    eANI_CLEAR_RX_STATS,  // Clears RX statistics of the radio interface
    eANI_CLEAR_TX_STATS,  // Clears TX statistics of the radio interface
    eANI_CLEAR_RADIO_STATS,   // Clears all the radio stats
    eANI_CLEAR_PER_STA_STATS, // Clears Per STA stats
    eANI_CLEAR_AGGR_PER_STA_STATS, // Clears aggregate stats

    // Used to distinguish between per sta to security stats.
    // Used only by AP, FW just returns the same parameter as it received.
    eANI_LINK_STATS,     // Get Per STA stats
    eANI_SECURITY_STATS, // Get Per STA security stats

    eANI_CLEAR_STAT_TYPES_END
} tAniStatSubTypes;

typedef struct sAniTxCtrs
{
    // add the rate counters here
    tANI_U32 tx1Mbps;
    tANI_U32 tx2Mbps;
    tANI_U32 tx5_5Mbps;
    tANI_U32 tx6Mbps;
    tANI_U32 tx9Mbps;
    tANI_U32 tx11Mbps;
    tANI_U32 tx12Mbps;
    tANI_U32 tx18Mbps;
    tANI_U32 tx24Mbps;
    tANI_U32 tx36Mbps;
    tANI_U32 tx48Mbps;
    tANI_U32 tx54Mbps;
    tANI_U32 tx72Mbps;
    tANI_U32 tx96Mbps;
    tANI_U32 tx108Mbps;

    // tx path radio counts
    tANI_U32 txFragHi;
    tANI_U32 txFragLo;
    tANI_U32 txFrameHi;
    tANI_U32 txFrameLo;
    tANI_U32 txMulticastFrameHi;
    tANI_U32 txMulticastFrameLo;
    tANI_U32 txFailedHi;
    tANI_U32 txFailedLo;
    tANI_U32 multipleRetryHi;
    tANI_U32 multipleRetryLo;
    tANI_U32 singleRetryHi;
    tANI_U32 singleRetryLo;
    tANI_U32 ackFailureHi;
    tANI_U32 ackFailureLo;
    tANI_U32 xmitBeacons;
} tAniTxCtrs, *tpAniTxCtrs;

typedef struct sAniRxCtrs
{
    // receive frame rate counters
    tANI_U32 rx1Mbps;
    tANI_U32 rx2Mbps;
    tANI_U32 rx5_5Mbps;
    tANI_U32 rx6Mbps;
    tANI_U32 rx9Mbps;
    tANI_U32 rx11Mbps;
    tANI_U32 rx12Mbps;
    tANI_U32 rx18Mbps;
    tANI_U32 rx24Mbps;
    tANI_U32 rx36Mbps;
    tANI_U32 rx48Mbps;
    tANI_U32 rx54Mbps;
    tANI_U32 rx72Mbps;
    tANI_U32 rx96Mbps;
    tANI_U32 rx108Mbps;

    // receive size counters; 'Lte' = Less than or equal to
    tANI_U32 rxLte64;
    tANI_U32 rxLte128Gt64;
    tANI_U32 rxLte256Gt128;
    tANI_U32 rxLte512Gt256;
    tANI_U32 rxLte1kGt512;
    tANI_U32 rxLte1518Gt1k;
    tANI_U32 rxLte2kGt1518;
    tANI_U32 rxLte4kGt2k;

    // rx radio stats
    tANI_U32 rxFrag;
    tANI_U32 rxFrame;
    tANI_U32 fcsError;
    tANI_U32 rxMulticast;
    tANI_U32 duplicate;
    tANI_U32 rtsSuccess;
    tANI_U32 rtsFailed;
    tANI_U32 wepUndecryptables;
    tANI_U32 drops;
    tANI_U32 aesFormatErrorUcastCnts;
    tANI_U32 aesReplaysUcast;
    tANI_U32 aesDecryptErrUcast;
} tAniRxCtrs, *tpAniRxCtrs;

// Radio stats
typedef struct sAniRadioStats
{
    tAniTxCtrs tx;
    tAniRxCtrs rx;
} tAniRadioStats, *tpAniRadioStats;

// Get Radio Stats request structure
// This structure shall be used for both Radio stats and Aggregate stats
// A valid request must contain entire structure with/without valid fields.
// Based on the request type, the valid fields will be checked.
typedef struct sAniGetStatsReq
{
    // Common for all types are requests
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;     // length of the entire request
    tANI_U8                 sessionId;  //Session ID
    tANI_U16                transactionId;
    tSirMacAddr             bssId;      //BSSID
    // only used for clear stats and per sta stats clear
    tAniStatSubTypes        stat;   // Clears the stats of the described types.
    tANI_U32                staId;  // Per STA stats request must contain valid
                               // values
    tANI_U8                 macAddr[6];
} tAniGetStatsReq, *tpAniGetStatsReq;

// Get Radio Stats response struct
typedef struct sAniGetRadioStatsRsp
{
    tANI_U16            type;   // message type is same as the request type
    tANI_U16            msgLen; // length of the entire request
    tANI_U32            rc;
    tANI_U16            transactionId;
    tAniRadioStats radio;
} tAniGetRadioStatsRsp, *tpAniGetRadioStatsRsp;

// Per Sta stats response struct
typedef struct sAniGetPerStaStatsRsp
{
    tANI_U16               type;   // message type is same as the request type
    tANI_U16               msgLen; // length of the entire request
    tANI_U32               rc;
    tANI_U16               transactionId;
    tAniStatSubTypes  stat;   // Sub type needed by AP. Returns the same value
    tAniStaStatStruct sta;
    tANI_U32               staId;
    tANI_U8                macAddr[6];
} tAniGetPerStaStatsRsp, *tpAniGetPerStaStatsRsp;

// Get Aggregate stats
typedef struct sAniGetAggrStaStatsRsp
{
    tANI_U16               type;   // message type is same as the request type
    tANI_U16               msgLen; // length of the entire request
    tANI_U32               rc;
    tANI_U16               transactionId;
    tAniStaStatStruct sta;
} tAniGetAggrStaStatsRsp, *tpAniGetAggrStaStatsRsp;

// Clear stats request and response structure. 'rc' field is unused in
// request and this field is used in response field.
typedef struct sAniClearStatsRsp
{
    tANI_U16                type;   // message type is same as the request type
    tANI_U16                msgLen; // length of the entire request
    tANI_U32                rc;     // return code - will be filled by FW on
                               // response.
                       // Same transaction ID will be returned by the FW
    tANI_U16                transactionId;
    tAniStatSubTypes   stat;       // Clears the stats of the described types.
    tANI_U32                staId;      // Applicable only to PER STA stats clearing
    tANI_U8                 macAddr[6]; // Applicable only to PER STA stats clearing
} tAniClearStatsRsp, *tpAniClearStatsRsp;

typedef struct sAniGetGlobalStatsRsp
{
    tANI_U16            type;   // message type is same as the request type
    tANI_U16            msgLen; // length of the entire request
    tANI_U32            rc;
    tANI_U16            transactionId;
    tAniGlobalStatStruct global;
} tAniGetGlobalStatsRsp, *tpAniGetGlobalStatsRsp;

typedef struct sAniGetStatSummaryRsp
{
    tANI_U16               type;   // message type is same as the request type
    tANI_U16               msgLen; // length of the entire request --Why?
    tANI_U32               rc;
    tANI_U16               transactionId;
    tAniStatSummaryStruct stat;
} tAniGetStatSummaryRsp, *tpAniGetStatSummaryRsp;

//***************************************************************


/*******************PE Statistics*************************/
typedef enum
{
    PE_SUMMARY_STATS_INFO           = 0x00000001,
    PE_GLOBAL_CLASS_A_STATS_INFO    = 0x00000002,
    PE_GLOBAL_CLASS_B_STATS_INFO    = 0x00000004,
    PE_GLOBAL_CLASS_C_STATS_INFO    = 0x00000008,
    PE_GLOBAL_CLASS_D_STATS_INFO    = 0x00000010,
    PE_PER_STA_STATS_INFO           = 0x00000020
}ePEStatsMask;

/*
 * tpAniGetPEStatsReq is tied to
 * for SME ==> PE eWNI_SME_GET_STATISTICS_REQ msgId  and
 * for PE ==> HAL SIR_HAL_GET_STATISTICS_REQ msgId
 */
typedef struct sAniGetPEStatsReq
{
    // Common for all types are requests
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;  // length of the entire request
    tANI_U32                staId;  // Per STA stats request must contain valid
    tANI_U32                statsMask;  // categories of stats requested. look at ePEStatsMask
    tANI_U8                 sessionId;
} tAniGetPEStatsReq, *tpAniGetPEStatsReq;

/*
 * tpAniGetPEStatsRsp is tied to
 * for PE ==> SME eWNI_SME_GET_STATISTICS_RSP msgId  and
 * for HAL ==> PE SIR_HAL_GET_STATISTICS_RSP msgId
 */
typedef struct sAniGetPEStatsRsp
{
    // Common for all types are responses
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;  // length of the entire request, includes the pStatsBuf length too
    tANI_U8                  sessionId;
    tANI_U32                rc;         //success/failure
    tANI_U32                staId;  // Per STA stats request must contain valid
    tANI_U32                statsMask;  // categories of stats requested. look at ePEStatsMask
/**********************************************************************************************
    //void                  *pStatsBuf;
    The Stats buffer starts here and can be an aggregate of more than one statistics
    structure depending on statsMask.The void pointer "pStatsBuf" is commented out
    intentionally and the src code that uses this structure should take that into account.
**********************************************************************************************/
} tAniGetPEStatsRsp, *tpAniGetPEStatsRsp;

typedef struct sAniGetRssiReq
{
    // Common for all types are requests
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;     // length of the entire request
    tANI_U8                 sessionId;
    tANI_U8                 staId;
    tANI_S8                 lastRSSI;   // in case of error, return last RSSI
    void                    *rssiCallback;
    void                    *pDevContext; //device context
    void                    *pVosContext; //voss context

} tAniGetRssiReq, *tpAniGetRssiReq;

typedef struct sAniGetSnrReq
{
    // Common for all types are requests
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;  // length of the entire request
    tANI_U8                 sessionId;
    tANI_U8                 staId;
    void                    *snrCallback;
    void                    *pDevContext; //device context
    tANI_S8                 snr;
} tAniGetSnrReq, *tpAniGetSnrReq;

#if defined(FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_ESE_UPLOAD)
typedef struct sSirTsmIE
{
    tANI_U8      tsid;
    tANI_U8      state;
    tANI_U16     msmt_interval;
} tSirTsmIE, *tpSirTsmIE;
typedef struct sSirSmeTsmIEInd
{
    tSirTsmIE tsmIe;
    tANI_U8   sessionId;
} tSirSmeTsmIEInd, *tpSirSmeTsmIEInd;
typedef struct sAniTrafStrmMetrics
{
    tANI_U16      UplinkPktQueueDly;
    tANI_U16      UplinkPktQueueDlyHist[4];
    tANI_U32      UplinkPktTxDly;
    tANI_U16      UplinkPktLoss;
    tANI_U16      UplinkPktCount;
    tANI_U8       RoamingCount;
    tANI_U16      RoamingDly;
} tAniTrafStrmMetrics, *tpAniTrafStrmMetrics;
typedef struct sAniGetTsmStatsReq
{
    // Common for all types are requests
    tANI_U16       msgType;  // message type is same as the request type
    tANI_U16       msgLen;   // length of the entire request
    tANI_U8        staId;
    tANI_U8        tid;      // traffic id
    tSirMacAddr    bssId;
    void           *tsmStatsCallback;
    void           *pDevContext;       //device context
    void           *pVosContext;       //voss context
} tAniGetTsmStatsReq, *tpAniGetTsmStatsReq;
typedef struct sAniGetTsmStatsRsp
{
    // Common for all types are responses
    tANI_U16            msgType;      /*
                                       * message type is same as
                                       * the request type
                                       */
    tANI_U16            msgLen;       /*
                                       * length of the entire request,
                                       * includes the pStatsBuf length too
                                       */
    tANI_U8             sessionId;
    tANI_U32            rc;           /* success/failure */
    tANI_U32            staId;        /*
                                       * Per STA stats request must
                                       * contain valid
                                       */
    tAniTrafStrmMetrics tsmMetrics;
    void               *tsmStatsReq;  /* tsm stats request backup */
} tAniGetTsmStatsRsp, *tpAniGetTsmStatsRsp;

typedef struct sSirEseBcnReportBssInfo
{
    tBcnReportFields  bcnReportFields;
    tANI_U8           ieLen;
    tANI_U8           *pBuf;
} tSirEseBcnReportBssInfo, *tpSirEseBcnReportBssInfo;

typedef struct sSirEseBcnReportRsp
{
    tANI_U16    measurementToken;
    tANI_U8     flag;     /* Flag to report measurement done and more data */
    tANI_U8     numBss;
    tSirEseBcnReportBssInfo bcnRepBssInfo[SIR_BCN_REPORT_MAX_BSS_DESC];
} tSirEseBcnReportRsp, *tpSirEseBcnReportRsp;

#endif /* FEATURE_WLAN_ESE || FEATURE_WLAN_ESE_UPLOAD */

/* Change country code request MSG structure */
typedef struct sAniChangeCountryCodeReq
{
    // Common for all types are requests
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;     // length of the entire request
    tANI_U8                 countryCode[WNI_CFG_COUNTRY_CODE_LEN];   //3 char country code
    tAniBool                countryFromUserSpace;
    tAniBool                sendRegHint;  //TRUE if we want to send hint to NL80211
    void                    *changeCCCallback;
    void                    *pDevContext; //device context
    void                    *pVosContext; //voss context

} tAniChangeCountryCodeReq, *tpAniChangeCountryCodeReq;

/* generic country code change request MSG structure */
typedef struct sAniGenericChangeCountryCodeReq
{
    // Common for all types are requests
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;     // length of the entire request
    tANI_U8                 countryCode[WNI_CFG_COUNTRY_CODE_LEN];   //3 char country code
    tANI_U16                domain_index;
} tAniGenericChangeCountryCodeReq, *tpAniGenericChangeCountryCodeReq;

typedef struct sAniDHCPStopInd
{
    tANI_U16                msgType;      // message type is same as the request type
    tANI_U16                msgLen;       // length of the entire request
    tANI_U8                 device_mode;  // Mode of the device(ex:STA, AP)
    tSirMacAddr             adapterMacAddr; // MAC address of the adapter
    tSirMacAddr             peerMacAddr; // MAC address of the connected peer

} tAniDHCPInd, *tpAniDHCPInd;

typedef struct sAniTXFailMonitorInd
{
    tANI_U16                msgType; // message type is same as the request type
    tANI_U16                msgLen;  // length of the entire request
    tANI_U8                 tx_fail_count;
    void                    *txFailIndCallback;
} tAniTXFailMonitorInd, *tpAniTXFailMonitorInd;

typedef struct sAniSummaryStatsInfo
{
    tANI_U32 retry_cnt[4];         //Total number of packets(per AC) that were successfully transmitted with retries
    tANI_U32 multiple_retry_cnt[4];//The number of MSDU packets and MMPDU frames per AC that the 802.11
    // station successfully transmitted after more than one retransmission attempt

    tANI_U32 tx_frm_cnt[4];        //Total number of packets(per AC) that were successfully transmitted
                                   //(with and without retries, including multi-cast, broadcast)
    //tANI_U32 tx_fail_cnt;
    //tANI_U32 num_rx_frm_crc_err;   //Total number of received frames with CRC Error
    //tANI_U32 num_rx_frm_crc_ok;    //Total number of successfully received frames with out CRC Error
    tANI_U32 rx_frm_cnt;           //Total number of packets that were successfully received
                                   //(after appropriate filter rules including multi-cast, broadcast)
    tANI_U32 frm_dup_cnt;          //Total number of duplicate frames received successfully
    tANI_U32 fail_cnt[4];          //Total number packets(per AC) failed to transmit
    tANI_U32 rts_fail_cnt;         //Total number of RTS/CTS sequence failures for transmission of a packet
    tANI_U32 ack_fail_cnt;         //Total number packets failed transmit because of no ACK from the remote entity
    tANI_U32 rts_succ_cnt;         //Total number of RTS/CTS sequence success for transmission of a packet
    tANI_U32 rx_discard_cnt;       //The sum of the receive error count and dropped-receive-buffer error count.
                                   //HAL will provide this as a sum of (FCS error) + (Fail get BD/PDU in HW)
    tANI_U32 rx_error_cnt;         //The receive error count. HAL will provide the RxP FCS error global counter.
    tANI_U32 tx_byte_cnt;          //The sum of the transmit-directed byte count, transmit-multicast byte count
                                   //and transmit-broadcast byte count. HAL will sum TPE UC/MC/BCAST global counters
                                   //to provide this.
}tAniSummaryStatsInfo, *tpAniSummaryStatsInfo;

typedef enum eTxRateInfo
{
   eHAL_TX_RATE_LEGACY = 0x1,    /* Legacy rates */
   eHAL_TX_RATE_HT20   = 0x2,    /* HT20 rates */
   eHAL_TX_RATE_HT40   = 0x4,    /* HT40 rates */
   eHAL_TX_RATE_SGI    = 0x8,    /* Rate with Short guard interval */
   eHAL_TX_RATE_LGI    = 0x10,   /* Rate with Long guard interval */
   eHAL_TX_RATE_VHT20  = 0x20,   /* VHT 20 rates */
   eHAL_TX_RATE_VHT40  = 0x40,   /* VHT 40 rates */
   eHAL_TX_RATE_VHT80  = 0x80    /* VHT 80 rates */
} tTxrateinfoflags;

typedef struct sAniGlobalClassAStatsInfo
{
    tANI_U32 rx_frag_cnt;             //The number of MPDU frames received by the 802.11 station for MSDU packets
                                     //or MMPDU frames
    tANI_U32 promiscuous_rx_frag_cnt; //The number of MPDU frames received by the 802.11 station for MSDU packets
                                     //or MMPDU frames when a promiscuous packet filter was enabled
    tANI_U32 rx_input_sensitivity;    //The receiver input sensitivity referenced to a FER of 8% at an MPDU length
                                     //of 1024 bytes at the antenna connector. Each element of the array shall correspond
                                     //to a supported rate and the order shall be the same as the supporteRates parameter.

    /* The maximum transmit power in dBm up-to one decimal
       for eg: if it is 10.5dBm, the value would be 105 */
    tANI_U32 max_pwr;

    tANI_U32 sync_fail_cnt;           //Number of times the receiver failed to synchronize with the incoming signal
                                     //after detecting the sync in the preamble of the transmitted PLCP protocol data unit.
    tANI_U32 tx_rate;                //Legacy transmit rate, in units of
                                     //500 kbit/sec, for the most
                                     //recently transmitted frame
    tANI_U32  mcs_index;             //mcs index for HT20 and HT40 rates
    tANI_U32  tx_rate_flags;         //to differentiate between HT20 and
                                     //HT40 rates;  short and long guard interval

}tAniGlobalClassAStatsInfo, *tpAniGlobalClassAStatsInfo;


typedef struct sAniGlobalSecurityStats
{
    tANI_U32 rx_wep_unencrypted_frm_cnt; //The number of unencrypted received MPDU frames that the MAC layer discarded when
                                        //the IEEE 802.11 dot11ExcludeUnencrypted management information base (MIB) object
                                        //is enabled
    tANI_U32 rx_mic_fail_cnt;            //The number of received MSDU packets that that the 802.11 station discarded
                                        //because of MIC failures
    tANI_U32 tkip_icv_err;               //The number of encrypted MPDU frames that the 802.11 station failed to decrypt
                                        //because of a TKIP ICV error
    tANI_U32 aes_ccmp_format_err;        //The number of received MPDU frames that the 802.11 discarded because of an
                                        //invalid AES-CCMP format
    tANI_U32 aes_ccmp_replay_cnt;        //The number of received MPDU frames that the 802.11 station discarded because of
                                        //the AES-CCMP replay protection procedure
    tANI_U32 aes_ccmp_decrpt_err;        //The number of received MPDU frames that the 802.11 station discarded because of
                                        //errors detected by the AES-CCMP decryption algorithm
    tANI_U32 wep_undecryptable_cnt;      //The number of encrypted MPDU frames received for which a WEP decryption key was
                                        //not available on the 802.11 station
    tANI_U32 wep_icv_err;                //The number of encrypted MPDU frames that the 802.11 station failed to decrypt
                                        //because of a WEP ICV error
    tANI_U32 rx_decrypt_succ_cnt;        //The number of received encrypted packets that the 802.11 station successfully
                                        //decrypted
    tANI_U32 rx_decrypt_fail_cnt;        //The number of encrypted packets that the 802.11 station failed to decrypt

}tAniGlobalSecurityStats, *tpAniGlobalSecurityStats;

typedef struct sAniGlobalClassBStatsInfo
{
    tAniGlobalSecurityStats ucStats;
    tAniGlobalSecurityStats mcbcStats;
}tAniGlobalClassBStatsInfo, *tpAniGlobalClassBStatsInfo;

typedef struct sAniGlobalClassCStatsInfo
{
    tANI_U32 rx_amsdu_cnt;           //This counter shall be incremented for a received A-MSDU frame with the stations
                                    //MAC address in the address 1 field or an A-MSDU frame with a group address in the
                                    //address 1 field
    tANI_U32 rx_ampdu_cnt;           //This counter shall be incremented when the MAC receives an AMPDU from the PHY
    tANI_U32 tx_20_frm_cnt;          //This counter shall be incremented when a Frame is transmitted only on the
                                    //primary channel
    tANI_U32 rx_20_frm_cnt;          //This counter shall be incremented when a Frame is received only on the primary channel
    tANI_U32 rx_mpdu_in_ampdu_cnt;   //This counter shall be incremented by the number of MPDUs received in the A-MPDU
                                    //when an A-MPDU is received
    tANI_U32 ampdu_delimiter_crc_err;//This counter shall be incremented when an MPDU delimiter has a CRC error when this
                                    //is the first CRC error in the received AMPDU or when the previous delimiter has been
                                    //decoded correctly

}tAniGlobalClassCStatsInfo, *tpAniGlobalClassCStatsInfo;

typedef struct sAniPerStaStatsInfo
{
    tANI_U32 tx_frag_cnt[4];       //The number of MPDU frames that the 802.11 station transmitted and acknowledged
                                  //through a received 802.11 ACK frame
    tANI_U32 tx_ampdu_cnt;         //This counter shall be incremented when an A-MPDU is transmitted
    tANI_U32 tx_mpdu_in_ampdu_cnt; //This counter shall increment by the number of MPDUs in the AMPDU when an A-MPDU
                                  //is transmitted

}tAniPerStaStatsInfo, *tpAniPerStaStatsInfo;

/**********************PE Statistics end*************************/



typedef struct sSirRSSIThresholds
{
#ifdef ANI_BIG_BYTE_ENDIAN
    tANI_S8   ucRssiThreshold1     : 8;
    tANI_S8   ucRssiThreshold2     : 8;
    tANI_S8   ucRssiThreshold3     : 8;
    tANI_U8   bRssiThres1PosNotify : 1;
    tANI_U8   bRssiThres1NegNotify : 1;
    tANI_U8   bRssiThres2PosNotify : 1;
    tANI_U8   bRssiThres2NegNotify : 1;
    tANI_U8   bRssiThres3PosNotify : 1;
    tANI_U8   bRssiThres3NegNotify : 1;
    tANI_U8   bReserved10          : 2;
#else
    tANI_U8   bReserved10          : 2;
    tANI_U8   bRssiThres3NegNotify : 1;
    tANI_U8   bRssiThres3PosNotify : 1;
    tANI_U8   bRssiThres2NegNotify : 1;
    tANI_U8   bRssiThres2PosNotify : 1;
    tANI_U8   bRssiThres1NegNotify : 1;
    tANI_U8   bRssiThres1PosNotify : 1;
    tANI_S8   ucRssiThreshold3     : 8;
    tANI_S8   ucRssiThreshold2     : 8;
    tANI_S8   ucRssiThreshold1     : 8;
#endif

}tSirRSSIThresholds, *tpSirRSSIThresholds;

typedef struct sSirRSSINotification
{
#ifdef ANI_BIG_BYTE_ENDIAN
    tANI_U32             bRssiThres1PosCross : 1;
    tANI_U32             bRssiThres1NegCross : 1;
    tANI_U32             bRssiThres2PosCross : 1;
    tANI_U32             bRssiThres2NegCross : 1;
    tANI_U32             bRssiThres3PosCross : 1;
    tANI_U32             bRssiThres3NegCross : 1;
    v_S7_t               avgRssi             : 8;
    tANI_U32             bReserved           : 18;
#else
    tANI_U32             bReserved           : 18;
    v_S7_t               avgRssi             : 8;
    tANI_U32             bRssiThres3NegCross : 1;
    tANI_U32             bRssiThres3PosCross : 1;
    tANI_U32             bRssiThres2NegCross : 1;
    tANI_U32             bRssiThres2PosCross : 1;
    tANI_U32             bRssiThres1NegCross : 1;
    tANI_U32             bRssiThres1PosCross : 1;
#endif

}tSirRSSINotification, *tpSirRSSINotification;


typedef struct sSirP2PNoaStart
{
   tANI_U32      status;
   tANI_U32      bssIdx;
} tSirP2PNoaStart, *tpSirP2PNoaStart;

typedef struct sSirTdlsInd
{
   tANI_U16      status;
   tANI_U16      assocId;
   tANI_U16      staIdx;
   tANI_U16      reasonCode;
} tSirTdlsInd, *tpSirTdlsInd;

typedef struct sSirP2PNoaAttr
{
#ifdef ANI_BIG_BYTE_ENDIAN
   tANI_U32      index :8;
   tANI_U32      oppPsFlag :1;
   tANI_U32      ctWin     :7;
   tANI_U32      rsvd1: 16;
#else
   tANI_U32      rsvd1: 16;
   tANI_U32      ctWin     :7;
   tANI_U32      oppPsFlag :1;
   tANI_U32      index :8;
#endif

#ifdef ANI_BIG_BYTE_ENDIAN
   tANI_U32       uNoa1IntervalCnt:8;
   tANI_U32       rsvd2:24;
#else
   tANI_U32       rsvd2:24;
   tANI_U32       uNoa1IntervalCnt:8;
#endif
   tANI_U32       uNoa1Duration;
   tANI_U32       uNoa1Interval;
   tANI_U32       uNoa1StartTime;

#ifdef ANI_BIG_BYTE_ENDIAN
   tANI_U32       uNoa2IntervalCnt:8;
   tANI_U32       rsvd3:24;
#else
   tANI_U32       rsvd3:24;
   tANI_U32       uNoa2IntervalCnt:8;
#endif
   tANI_U32       uNoa2Duration;
   tANI_U32       uNoa2Interval;
   tANI_U32       uNoa2StartTime;
} tSirP2PNoaAttr, *tpSirP2PNoaAttr;

typedef __ani_attr_pre_packed struct sSirTclasInfo
{
    tSirMacTclasIE   tclas;
    tANI_U8               version; // applies only for classifier type ip
    __ani_attr_pre_packed union {
        tSirMacTclasParamEthernet eth;
        tSirMacTclasParamIPv4     ipv4;
        tSirMacTclasParamIPv6     ipv6;
        tSirMacTclasParam8021dq   t8021dq;
    }__ani_attr_packed tclasParams;
} __ani_attr_packed tSirTclasInfo;

#if defined(FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_ESE_UPLOAD)
#define TSRS_11AG_RATE_6MBPS   0xC
#define TSRS_11B_RATE_5_5MBPS  0xB
typedef struct sSirMacESETSRSIE
{
    tANI_U8      tsid;
    tANI_U8      rates[8];
} tSirMacESETSRSIE;
typedef struct sSirMacESETSMIE
{
    tANI_U8      tsid;
    tANI_U8      state;
    tANI_U16     msmt_interval;
} tSirMacESETSMIE;
typedef struct sTSMStats
{
    tANI_U8           tid;
    tSirMacAddr       bssId;
    tTrafStrmMetrics  tsmMetrics;
} tTSMStats, *tpTSMStats;
typedef struct sEseTSMContext
{
   tANI_U8           tid;
   tSirMacESETSMIE   tsmInfo;
   tTrafStrmMetrics  tsmMetrics;
} tEseTSMContext, *tpEseTSMContext;
typedef struct sEsePEContext
{
#if defined(FEATURE_WLAN_ESE) && !defined(FEATURE_WLAN_ESE_UPLOAD)
   tEseMeasReq       curMeasReq;
#endif
   tEseTSMContext    tsm;
} tEsePEContext, *tpEsePEContext;
#endif /* FEATURE_WLAN_ESE && FEATURE_WLAN_ESE_UPLOAD */

typedef struct sSirAddtsReqInfo
{
    tANI_U8               dialogToken;
    tSirMacTspecIE   tspec;

    tANI_U8               numTclas; // number of Tclas elements
    tSirTclasInfo    tclasInfo[SIR_MAC_TCLASIE_MAXNUM];
    tANI_U8               tclasProc;
#if defined(FEATURE_WLAN_ESE)
    tSirMacESETSRSIE      tsrsIE;
    tANI_U8               tsrsPresent:1;
#endif
    tANI_U8               wmeTspecPresent:1;
    tANI_U8               wsmTspecPresent:1;
    tANI_U8               lleTspecPresent:1;
    tANI_U8               tclasProcPresent:1;
} tSirAddtsReqInfo, *tpSirAddtsReqInfo;

typedef struct sSirAddtsRspInfo
{
    tANI_U8                 dialogToken;
    tSirMacStatusCodes status;
    tSirMacTsDelayIE   delay;

    tSirMacTspecIE     tspec;
    tANI_U8                 numTclas; // number of Tclas elements
    tSirTclasInfo      tclasInfo[SIR_MAC_TCLASIE_MAXNUM];
    tANI_U8                 tclasProc;
    tSirMacScheduleIE  schedule;
#if defined(FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_ESE_UPLOAD)
    tSirMacESETSMIE    tsmIE;
    tANI_U8                 tsmPresent:1;
#endif
    tANI_U8                 wmeTspecPresent:1;
    tANI_U8                 wsmTspecPresent:1;
    tANI_U8                 lleTspecPresent:1;
    tANI_U8                 tclasProcPresent:1;
    tANI_U8                 schedulePresent:1;
} tSirAddtsRspInfo, *tpSirAddtsRspInfo;

typedef struct sSirDeltsReqInfo
{
    tSirMacTSInfo      tsinfo;
    tSirMacTspecIE     tspec;
    tANI_U8                 wmeTspecPresent:1;
    tANI_U8                 wsmTspecPresent:1;
    tANI_U8                 lleTspecPresent:1;
} tSirDeltsReqInfo, *tpSirDeltsReqInfo;

/// Add a tspec as defined
typedef struct sSirAddtsReq
{
    tANI_U16                messageType; // eWNI_SME_ADDTS_REQ
    tANI_U16                length;
    tANI_U8                 sessionId;  //Session ID
    tANI_U16                transactionId;
    tSirMacAddr             bssId;      //BSSID
    tANI_U32                timeout; // in ms
    tANI_U8                 rspReqd;
    tSirAddtsReqInfo        req;
} tSirAddtsReq, *tpSirAddtsReq;

typedef struct sSirAddtsRsp
{
    tANI_U16                messageType; // eWNI_SME_ADDTS_RSP
    tANI_U16                length;
    tANI_U8                 sessionId;  // sme sessionId  Added for BT-AMP support
    tANI_U16                transactionId; //sme transaction Id Added for BT-AMP Support
    tANI_U32                rc;          // return code
    tSirAddtsRspInfo        rsp;
} tSirAddtsRsp, *tpSirAddtsRsp;

typedef struct sSirDeltsReq
{
    tANI_U16                messageType; // eWNI_SME_DELTS_REQ
    tANI_U16                length;
    tANI_U8                 sessionId;//Session ID
    tANI_U16                transactionId;
    tSirMacAddr             bssId;  //BSSID
    tANI_U16                aid;  // use 0 if macAddr is being specified
    tANI_U8                 macAddr[6]; // only on AP to specify the STA
    tANI_U8                 rspReqd;
    tSirDeltsReqInfo        req;
} tSirDeltsReq, *tpSirDeltsReq;

typedef struct sSirDeltsRsp
{
    tANI_U16                messageType; // eWNI_SME_DELTS_RSP
    tANI_U16                length;
    tANI_U8                 sessionId;  // sme sessionId  Added for BT-AMP support
    tANI_U16                transactionId; //sme transaction Id Added for BT-AMP Support
    tANI_U32                rc;
    tANI_U16                aid;  // use 0 if macAddr is being specified
    tANI_U8                 macAddr[6]; // only on AP to specify the STA
    tSirDeltsReqInfo        rsp;
} tSirDeltsRsp, *tpSirDeltsRsp;

#if defined(FEATURE_WLAN_ESE) && defined(FEATURE_WLAN_ESE_UPLOAD)
typedef struct sSirPlmReq
{
    tANI_U16                diag_token; // Dialog token
    tANI_U16                meas_token; // measurement token
    tANI_U16                numBursts; // total number of bursts
    tANI_U16                burstInt; // burst interval in seconds
    tANI_U16                measDuration; // in TU's,STA goes off-ch
    /* no of times the STA should cycle through PLM ch list */
    tANI_U8                 burstLen;
    tPowerdBm               desiredTxPwr; // desired tx power
    tSirMacAddr             macAddr; // MC dest addr
    /* no of channels */
    tANI_U8                 plmNumCh;
    /* channel numbers */
    tANI_U8                 plmChList[WNI_CFG_VALID_CHANNEL_LIST_LEN];
    tANI_U8                 sessionId;
    eAniBoolean             enable;
} tSirPlmReq, *tpSirPlmReq;
#endif

#if defined WLAN_FEATURE_VOWIFI_11R || defined FEATURE_WLAN_ESE || defined(FEATURE_WLAN_LFR)

#define SIR_QOS_NUM_AC_MAX 4

typedef struct sSirAggrQosReqInfo
{
    tANI_U16 tspecIdx;
    tSirAddtsReqInfo aggrAddTsInfo[SIR_QOS_NUM_AC_MAX];
}tSirAggrQosReqInfo, *tpSirAggrQosReqInfo;

typedef struct sSirAggrQosReq
{
    tANI_U16                messageType; // eWNI_SME_ADDTS_REQ
    tANI_U16                length;
    tANI_U8                 sessionId;  //Session ID
    tANI_U16                transactionId;
    tSirMacAddr             bssId;      //BSSID
    tANI_U32                timeout; // in ms
    tANI_U8                 rspReqd;
    tSirAggrQosReqInfo      aggrInfo;
}tSirAggrQosReq, *tpSirAggrQosReq;

typedef struct sSirAggrQosRspInfo
{
    tANI_U16                tspecIdx;
    tSirAddtsRspInfo        aggrRsp[SIR_QOS_NUM_AC_MAX];
} tSirAggrQosRspInfo, *tpSirAggrQosRspInfo;

typedef struct sSirAggrQosRsp
{
    tANI_U16                messageType;
    tANI_U16                length;
    tANI_U8                 sessionId;
    tSirAggrQosRspInfo      aggrInfo;
} tSirAggrQosRsp, *tpSirAggrQosRsp;

#endif/*WLAN_FEATURE_VOWIFI_11R || FEATURE_WLAN_ESE*/

typedef struct sSirSetTxPowerReq
{
    tANI_U16       messageType;
    tANI_U16       length;
    tSirMacAddr    bssId;
    tANI_U8        mwPower;
    tANI_U8        bssIdx;
} tSirSetTxPowerReq, *tpSirSetTxPowerReq;

typedef struct sSirSetTxPowerRsp
{
    tANI_U16            messageType;
    tANI_U16        length;
    tANI_U32        status;
} tSirSetTxPowerRsp, *tpSirSetTxPowerRsp;

typedef struct sSirGetTxPowerReq
{
    tANI_U16    messageType;
    tANI_U16    length;
    tANI_U16    staid;
} tSirGetTxPowerReq, *tpSirGetTxPowerReq;

typedef struct sSirGetTxPowerRsp
{
    tANI_U16            messageType;
    tANI_U16            length; // length of the entire request
    tANI_U32            power;  // units of milliwatts
    tANI_U32            status;
} tSirGetTxPowerRsp, *tpSirGetTxPowerRsp;


typedef tANI_U32 tSirMacNoise[3];

typedef struct sSirGetNoiseRsp
{
    tANI_U16            messageType;
    tANI_U16            length;
    tSirMacNoise        noise;
} tSirGetNoiseRsp, *tpSirGetNoiseRsp;

typedef struct sSirQosMapSet
{
    tANI_U8      present;
    tANI_U8      num_dscp_exceptions;
    tANI_U8      dscp_exceptions[21][2];
    tANI_U8      dscp_range[8][2];
} tSirQosMapSet, *tpSirQosMapSet;

//
// PMC --> PE --> HAL
// Power save configuration parameters
//
typedef struct sSirPowerSaveCfg
{
    tANI_U16    listenInterval;

    /* Number of consecutive missed beacons before
     * hardware generates an interrupt to wake up
     * the host. In units of listen interval.
     */
    tANI_U32 HeartBeatCount;

    /* specifies which beacons are to be forwarded
     * to host when beacon filtering is enabled.
     * In units of listen interval.
     */
    tANI_U32    nthBeaconFilter;

    /* Maximum number of PS-Poll send before
     * firmware sends data null with PM set to 0.
     */
    tANI_U32    maxPsPoll;

    /* If the average RSSI value falls below the
     * minRssiThreshold, then FW will send an
     * interrupt to wake up the host.
     */
    tANI_U32    minRssiThreshold;

    /* Number of beacons for which firmware will
     * collect the RSSI values and compute the average.
     */
    tANI_U8     numBeaconPerRssiAverage;

    /* FW collects the RSSI stats for this period
     * in BMPS mode.
     */
    tANI_U8     rssiFilterPeriod;

    // Enabling/disabling broadcast frame filter feature
    tANI_U8     broadcastFrameFilter;

    // Enabling/disabling the ignore DTIM feature
    tANI_U8     ignoreDtim;

    /* The following configuration parameters are kept
     * in order to be backward compatible for Gen5.
     * These will NOT be used for Gen6 Libra chip
     */
    tBeaconForwarding beaconFwd;
    tANI_U16 nthBeaconFwd;
    tANI_U8 fEnablePwrSaveImmediately;
    tANI_U8 fPSPoll;

    // Enabling/disabling Beacon Early Termination feature
    tANI_U8     fEnableBeaconEarlyTermination;
    tANI_U8     bcnEarlyTermWakeInterval;

}tSirPowerSaveCfg, *tpSirPowerSaveCfg;

/* Reason code for requesting Full Power. This reason code is used by
   any module requesting full power from PMC and also by PE when it
   sends the eWNI_PMC_EXIT_BMPS_IND to PMC*/
typedef enum eRequestFullPowerReason
{
   eSME_MISSED_BEACON_IND_RCVD,    /* PE received a MAX_MISSED_BEACON_IND */
   eSME_BMPS_STATUS_IND_RCVD,      /* PE received a SIR_HAL_BMPS_STATUS_IND */
   eSME_BMPS_MODE_DISABLED,        /* BMPS mode was disabled by HDD in SME */
   eSME_LINK_DISCONNECTED_BY_HDD,  /* Link has been disconnected requested by HDD */

   /* Disconnect due to link lost or requested by peer */
   eSME_LINK_DISCONNECTED_BY_OTHER,

   eSME_FULL_PWR_NEEDED_BY_HDD,    /* HDD request full power for some reason */
   eSME_FULL_PWR_NEEDED_BY_BAP,    /* BAP request full power for BT_AMP */
   eSME_FULL_PWR_NEEDED_BY_CSR,    /* CSR requests full power */
   eSME_FULL_PWR_NEEDED_BY_QOS,    /* QOS requests full power */
   eSME_FULL_PWR_NEEDED_BY_CHANNEL_SWITCH, /* channel switch request full power*/
#ifdef FEATURE_WLAN_TDLS
   eSME_FULL_PWR_NEEDED_BY_TDLS_PEER_SETUP, /* TDLS peer setup*/
#endif
   eSME_REASON_OTHER               /* No specific reason. General reason code */
} tRequestFullPowerReason, tExitBmpsReason;



/* This is sent along with eWNI_PMC_EXIT_BMPS_REQ message */
typedef struct sExitBmpsInfo
{
   tExitBmpsReason exitBmpsReason;  /*Reason for exiting BMPS */
}tExitBmpsInfo, *tpExitBmpsInfo;


// MAC SW --> SME
// Message indicating to SME to exit BMPS sleep mode
typedef struct sSirSmeExitBmpsInd
{
    tANI_U16  mesgType;               /* eWNI_PMC_EXIT_BMPS_IND */
    tANI_U16  mesgLen;
    tSirResultCodes  statusCode;
    tExitBmpsReason  exitBmpsReason;  /*Reason for exiting BMPS */
    tANI_U32 smeSessionId;
} tSirSmeExitBmpsInd, *tpSirSmeExitBmpsInd;

// MAC SW --> SME
// Message indicating to SME for channel switch
typedef struct sSirSmePreSwitchChannelInd
{
    tANI_U16  mesgType;               /* eWNI_SME_PRE_SWITCH_CHL_IND */
    tANI_U16  mesgLen;
    tANI_U8   sessionId;
} tSirSmePreSwitchChannelInd, *tpSirSmePreSwitchChannelInd;


//
// HDD -> LIM
// tSirMsgQ.type = eWNI_SME_DEL_BA_PEER_IND
// tSirMsgQ.reserved = 0
// tSirMsgQ.body = instance of tDelBAParams
//
typedef struct sSmeDelBAPeerInd
{
    // Message Type
    tANI_U16 mesgType;

    tSirMacAddr bssId;//BSSID

    // Message Length
    tANI_U16 mesgLen;

    // Station Index
    tANI_U16 staIdx;

    // TID for which the BA session is being deleted
    tANI_U8 baTID;

    // DELBA direction
    // eBA_INITIATOR - Originator
    // eBA_RECEIPIENT - Recipient
    tANI_U8 baDirection;
} tSmeDelBAPeerInd, *tpSmeDelBAPeerInd;

typedef struct sSmeIbssPeerInd
{
    tANI_U16    mesgType;
    tANI_U16    mesgLen;
    tANI_U8     sessionId;

    tSirMacAddr peerAddr;
    tANI_U16    staId;

    /*The DPU signatures will be sent eventually to TL to help it determine the
      association to which a packet belongs to*/
    /*Unicast DPU signature*/
    tANI_U8            ucastSig;

    /*Broadcast DPU signature*/
    tANI_U8            bcastSig;

    //Beacon will be appended for new Peer indication.
}tSmeIbssPeerInd, *tpSmeIbssPeerInd;

typedef struct sSirIbssPeerInactivityInd
{
   tANI_U8       bssIdx;
   tANI_U8       staIdx;
   tSirMacAddr   peerAddr;
}tSirIbssPeerInactivityInd, *tpSirIbssPeerInactivityInd;


typedef struct sLimScanChn
{
    tANI_U16 numTimeScan;   //how many time this channel is scan
    tANI_U8 channelId;
}tLimScanChn;

typedef struct sSmeGetScanChnRsp
{
    // Message Type
    tANI_U16 mesgType;
    // Message Length
    tANI_U16 mesgLen;
    tANI_U8   sessionId;
    tANI_U8 numChn;
    tLimScanChn scanChn[1];
} tSmeGetScanChnRsp, *tpSmeGetScanChnRsp;

typedef struct sLimScanChnInfo
{
    tANI_U8 numChnInfo;     //number of channels in scanChn
    tLimScanChn scanChn[SIR_MAX_SUPPORTED_CHANNEL_LIST];
}tLimScanChnInfo;

typedef struct sSirSmeGetAssocSTAsReq
{
    tANI_U16    messageType;    // eWNI_SME_GET_ASSOC_STAS_REQ
    tANI_U16    length;
    tSirMacAddr bssId;          // BSSID
    tANI_U16    modId;
    void        *pUsrContext;
    void        *pSapEventCallback;
    void        *pAssocStasArray;// Pointer to allocated memory passed in WLANSAP_GetAssocStations API
} tSirSmeGetAssocSTAsReq, *tpSirSmeGetAssocSTAsReq;

typedef struct sSmeMaxAssocInd
{
    tANI_U16    mesgType;    // eWNI_SME_MAX_ASSOC_EXCEEDED
    tANI_U16    mesgLen;
    tANI_U8     sessionId;
    tSirMacAddr peerMac;     // the new peer that got rejected due to softap max assoc limit reached
} tSmeMaxAssocInd, *tpSmeMaxAssocInd;

typedef struct sSmeCsaOffloadInd
{
    tANI_U16    mesgType;    // eWNI_SME_CSA_OFFLOAD_EVENT
    tANI_U16    mesgLen;
    tSirMacAddr bssId;       // BSSID
} tSmeCsaOffloadInd, *tpSmeCsaOffloadInd;

/// WOW related structures
// SME -> PE <-> HAL
#define SIR_WOWL_BCAST_PATTERN_MAX_SIZE 146
#define SIR_WOWL_BCAST_MAX_NUM_PATTERNS 19
// SME -> PE -> HAL - This is to add WOWL BCAST wake-up pattern.
// SME/HDD maintains the list of the BCAST wake-up patterns.
// This is a pass through message for PE
typedef struct sSirWowlAddBcastPtrn
{
    tANI_U8  ucPatternId;           // Pattern ID
    // Pattern byte offset from beginning of the 802.11 packet to start of the
    // wake-up pattern
    tANI_U8  ucPatternByteOffset;
    tANI_U8  ucPatternSize;         // Non-Zero Pattern size
    tANI_U8  ucPattern[SIR_WOWL_BCAST_PATTERN_MAX_SIZE]; // Pattern
    tANI_U8  ucPatternMaskSize;     // Non-zero pattern mask size
    tANI_U8  ucPatternMask[SIR_WOWL_BCAST_PATTERN_MAX_SIZE]; // Pattern mask
    // Extra pattern data beyond 128 bytes
    tANI_U8  ucPatternExt[SIR_WOWL_BCAST_PATTERN_MAX_SIZE]; // Extra Pattern
    tANI_U8  ucPatternMaskExt[SIR_WOWL_BCAST_PATTERN_MAX_SIZE]; // Extra Pattern mask
    tSirMacAddr    bssId;           // BSSID
    tANI_U8    sessionId;
} tSirWowlAddBcastPtrn, *tpSirWowlAddBcastPtrn;


// SME -> PE -> HAL - This is to delete WOWL BCAST wake-up pattern.
// SME/HDD maintains the list of the BCAST wake-up patterns.
// This is a pass through message for PE
typedef struct sSirWowlDelBcastPtrn
{
    /* Pattern ID of the wakeup pattern to be deleted */
    tANI_U8  ucPatternId;
    tSirMacAddr    bssId;           // BSSID
}tSirWowlDelBcastPtrn, *tpSirWowlDelBcastPtrn;


// SME->PE: Enter WOWLAN parameters
typedef struct sSirSmeWowlEnterParams
{
    tANI_U8  sessionId;

    /* Enables/disables magic packet filtering */
    tANI_U8   ucMagicPktEnable;

    /* Magic pattern */
    tSirMacAddr magicPtrn;

    /* Enables/disables packet pattern filtering */
    tANI_U8   ucPatternFilteringEnable;

#ifdef WLAN_WAKEUP_EVENTS
    /* This configuration directs the WoW packet filtering to look for EAP-ID
     * requests embedded in EAPOL frames and use this as a wake source.
     */
    tANI_U8   ucWoWEAPIDRequestEnable;

    /* This configuration directs the WoW packet filtering to look for EAPOL-4WAY
     * requests and use this as a wake source.
     */
    tANI_U8   ucWoWEAPOL4WayEnable;

    /* This configuration allows a host wakeup on an network scan offload match.
     */
    tANI_U8   ucWowNetScanOffloadMatch;

    /* This configuration allows a host wakeup on any GTK re-keying error.
     */
    tANI_U8   ucWowGTKRekeyError;

    /* This configuration allows a host wakeup on BSS connection loss.
     */
    tANI_U8   ucWoWBSSConnLoss;
#endif // WLAN_WAKEUP_EVENTS

    tSirMacAddr bssId;
} tSirSmeWowlEnterParams, *tpSirSmeWowlEnterParams;


// PE<->HAL: Enter WOWLAN parameters
typedef struct sSirHalWowlEnterParams
{
    tANI_U8  sessionId;

    /* Enables/disables magic packet filtering */
    tANI_U8   ucMagicPktEnable;

    /* Magic pattern */
    tSirMacAddr magicPtrn;

    /* Enables/disables packet pattern filtering in firmware.
       Enabling this flag enables broadcast pattern matching
       in Firmware. If unicast pattern matching is also desired,
       ucUcastPatternFilteringEnable flag must be set tot true
       as well
    */
    tANI_U8   ucPatternFilteringEnable;

    /* Enables/disables unicast packet pattern filtering.
       This flag specifies whether we want to do pattern match
       on unicast packets as well and not just broadcast packets.
       This flag has no effect if the ucPatternFilteringEnable
       (main controlling flag) is set to false
    */
    tANI_U8   ucUcastPatternFilteringEnable;

    /* This configuration is valid only when magicPktEnable=1.
     * It requests hardware to wake up when it receives the
     * Channel Switch Action Frame.
     */
    tANI_U8   ucWowChnlSwitchRcv;

    /* This configuration is valid only when magicPktEnable=1.
     * It requests hardware to wake up when it receives the
     * Deauthentication Frame.
     */
    tANI_U8   ucWowDeauthRcv;

    /* This configuration is valid only when magicPktEnable=1.
     * It requests hardware to wake up when it receives the
     * Disassociation Frame.
     */
    tANI_U8   ucWowDisassocRcv;

    /* This configuration is valid only when magicPktEnable=1.
     * It requests hardware to wake up when it has missed
     * consecutive beacons. This is a hardware register
     * configuration (NOT a firmware configuration).
     */
    tANI_U8   ucWowMaxMissedBeacons;

    /* This configuration is valid only when magicPktEnable=1.
     * This is a timeout value in units of microsec. It requests
     * hardware to unconditionally wake up after it has stayed
     * in WoWLAN mode for some time. Set 0 to disable this feature.
     */
    tANI_U8   ucWowMaxSleepUsec;

#ifdef WLAN_WAKEUP_EVENTS
    /* This configuration directs the WoW packet filtering to look for EAP-ID
     * requests embedded in EAPOL frames and use this as a wake source.
     */
    tANI_U8   ucWoWEAPIDRequestEnable;

    /* This configuration directs the WoW packet filtering to look for EAPOL-4WAY
     * requests and use this as a wake source.
     */
    tANI_U8   ucWoWEAPOL4WayEnable;

    /* This configuration allows a host wakeup on an network scan offload match.
     */
    tANI_U8   ucWowNetScanOffloadMatch;

    /* This configuration allows a host wakeup on any GTK rekeying error.
     */
    tANI_U8   ucWowGTKRekeyError;

    /* This configuration allows a host wakeup on BSS connection loss.
     */
    tANI_U8   ucWoWBSSConnLoss;
#endif // WLAN_WAKEUP_EVENTS

    /* Status code to be filled by HAL when it sends
     * SIR_HAL_WOWL_ENTER_RSP to PE.
     */
    eHalStatus  status;

   /*BSSID to find the current session
      */
    tANI_U8  bssIdx;
} tSirHalWowlEnterParams, *tpSirHalWowlEnterParams;

// SME->PE: Exit WOWLAN parameters
typedef struct sSirSmeWowlExitParams
{
    tANI_U8  sessionId;

} tSirSmeWowlExitParams, *tpSirSmeWowlExitParams;

// PE<->HAL: Exit WOWLAN parameters
typedef struct sSirHalWowlExitParams
{
    tANI_U8  sessionId;

    /* Status code to be filled by HAL when it sends
     * SIR_HAL_WOWL_EXIT_RSP to PE.
     */
    eHalStatus  status;

   /*BSSIDX to find the current session
      */
    tANI_U8  bssIdx;
} tSirHalWowlExitParams, *tpSirHalWowlExitParams;


#define SIR_MAX_NAME_SIZE 64
#define SIR_MAX_TEXT_SIZE 32

typedef struct sSirName {
    v_U8_t num_name;
    v_U8_t name[SIR_MAX_NAME_SIZE];
} tSirName;

typedef struct sSirText {
    v_U8_t num_text;
    v_U8_t text[SIR_MAX_TEXT_SIZE];
} tSirText;


#define SIR_WPS_PROBRSP_VER_PRESENT    0x00000001
#define SIR_WPS_PROBRSP_STATE_PRESENT    0x00000002
#define SIR_WPS_PROBRSP_APSETUPLOCK_PRESENT    0x00000004
#define SIR_WPS_PROBRSP_SELECTEDREGISTRA_PRESENT    0x00000008
#define SIR_WPS_PROBRSP_DEVICEPASSWORDID_PRESENT    0x00000010
#define SIR_WPS_PROBRSP_SELECTEDREGISTRACFGMETHOD_PRESENT    0x00000020
#define SIR_WPS_PROBRSP_RESPONSETYPE_PRESENT    0x00000040
#define SIR_WPS_PROBRSP_UUIDE_PRESENT    0x00000080
#define SIR_WPS_PROBRSP_MANUFACTURE_PRESENT    0x00000100
#define SIR_WPS_PROBRSP_MODELNAME_PRESENT    0x00000200
#define SIR_WPS_PROBRSP_MODELNUMBER_PRESENT    0x00000400
#define SIR_WPS_PROBRSP_SERIALNUMBER_PRESENT    0x00000800
#define SIR_WPS_PROBRSP_PRIMARYDEVICETYPE_PRESENT    0x00001000
#define SIR_WPS_PROBRSP_DEVICENAME_PRESENT    0x00002000
#define SIR_WPS_PROBRSP_CONFIGMETHODS_PRESENT    0x00004000
#define SIR_WPS_PROBRSP_RF_BANDS_PRESENT    0x00008000


typedef struct sSirWPSProbeRspIE {
   v_U32_t  FieldPresent;
   v_U32_t  Version;           // Version. 0x10 = version 1.0, 0x11 = etc.
   v_U32_t  wpsState;          // 1 = unconfigured, 2 = configured.
   v_BOOL_t APSetupLocked;     // Must be included if value is TRUE
   v_BOOL_t SelectedRegistra;  //BOOL:  indicates if the user has recently activated a Registrar to add an Enrollee.
   v_U16_t  DevicePasswordID;  // Device Password ID
   v_U16_t  SelectedRegistraCfgMethod; // Selected Registrar config method
   v_U8_t   ResponseType;      // Response type
   v_U8_t   UUID_E[16];         // Unique identifier of the AP.
   tSirName   Manufacture;
   tSirText   ModelName;
   tSirText   ModelNumber;
   tSirText  SerialNumber;
   v_U32_t  PrimaryDeviceCategory ; // Device Category ID: 1Computer, 2Input Device, ...
   v_U8_t   PrimaryDeviceOUI[4] ; // Vendor specific OUI for Device Sub Category
   v_U32_t  DeviceSubCategory ; // Device Sub Category ID: 1-PC, 2-Server if Device Category ID is computer
   tSirText DeviceName;
   v_U16_t  ConfigMethod;     /* Configuration method */
   v_U8_t   RFBand;           // RF bands available on the AP
} tSirWPSProbeRspIE;

#define SIR_WPS_BEACON_VER_PRESENT    0x00000001
#define SIR_WPS_BEACON_STATE_PRESENT    0x00000002
#define SIR_WPS_BEACON_APSETUPLOCK_PRESENT    0x00000004
#define SIR_WPS_BEACON_SELECTEDREGISTRA_PRESENT    0x00000008
#define SIR_WPS_BEACON_DEVICEPASSWORDID_PRESENT    0x00000010
#define SIR_WPS_BEACON_SELECTEDREGISTRACFGMETHOD_PRESENT    0x00000020
#define SIR_WPS_BEACON_UUIDE_PRESENT    0x00000080
#define SIR_WPS_BEACON_RF_BANDS_PRESENT    0x00000100
#define SIR_WPS_UUID_LEN 16

typedef struct sSirWPSBeaconIE {
   v_U32_t  FieldPresent;
   v_U32_t  Version;           // Version. 0x10 = version 1.0, 0x11 = etc.
   v_U32_t  wpsState;          // 1 = unconfigured, 2 = configured.
   v_BOOL_t APSetupLocked;     // Must be included if value is TRUE
   v_BOOL_t SelectedRegistra;  //BOOL:  indicates if the user has recently activated a Registrar to add an Enrollee.
   v_U16_t  DevicePasswordID;  // Device Password ID
   v_U16_t  SelectedRegistraCfgMethod; // Selected Registrar config method
   v_U8_t   UUID_E[SIR_WPS_UUID_LEN];   /* Unique identifier of the AP */
   v_U8_t   RFBand;           // RF bands available on the AP
} tSirWPSBeaconIE;

#define SIR_WPS_ASSOCRSP_VER_PRESENT    0x00000001
#define SIR_WPS_ASSOCRSP_RESPONSETYPE_PRESENT    0x00000002

typedef struct sSirWPSAssocRspIE {
   v_U32_t FieldPresent;
   v_U32_t Version;
   v_U8_t ResposeType;
} tSirWPSAssocRspIE;

typedef struct sSirAPWPSIEs {
   tSirWPSProbeRspIE  SirWPSProbeRspIE;    /* WPS Set Probe Response IE */
   tSirWPSBeaconIE    SirWPSBeaconIE;      /*WPS Set Beacon IE*/
   tSirWPSAssocRspIE  SirWPSAssocRspIE;    /*WPS Set Assoc Response IE*/
} tSirAPWPSIEs, *tpSiriAPWPSIEs;

typedef struct sSirUpdateAPWPSIEsReq
{
    tANI_U16       messageType;     // eWNI_SME_UPDATE_APWPSIE_REQ
    tANI_U16       length;
    tANI_U16       transactionId;   //Transaction ID for cmd
    tSirMacAddr    bssId;           // BSSID
    tANI_U8        sessionId;       //Session ID
    tSirAPWPSIEs   APWPSIEs;
} tSirUpdateAPWPSIEsReq, *tpSirUpdateAPWPSIEsReq;

typedef struct sSirUpdateParams
{
    tANI_U16       messageType;
    tANI_U16       length;
    tANI_U8        sessionId;      // Session ID
    tANI_U8        ssidHidden;     // Hide SSID
} tSirUpdateParams, *tpSirUpdateParams;

//Beacon Interval
typedef struct sSirChangeBIParams
{
    tANI_U16       messageType;
    tANI_U16       length;
    tANI_U16       beaconInterval; // Beacon Interval
    tSirMacAddr    bssId;
    tANI_U8        sessionId;      // Session ID
} tSirChangeBIParams, *tpSirChangeBIParams;

#ifdef QCA_HT_2040_COEX
typedef struct sSirSetHT2040Mode
{
    tANI_U16       messageType;
    tANI_U16       length;
    tANI_U8        cbMode;
    tANI_BOOLEAN   obssEnabled;
    tSirMacAddr    bssId;
    tANI_U8        sessionId;      // Session ID
} tSirSetHT2040Mode, *tpSirSetHT2040Mode;
#endif

#define SIR_WPS_PBC_WALK_TIME   120  // 120 Second

typedef struct sSirWPSPBCSession {
    struct sSirWPSPBCSession *next;
    tSirMacAddr              addr;
    tANI_U8                  uuid_e[SIR_WPS_UUID_LEN];
    tANI_TIMESTAMP           timestamp;
} tSirWPSPBCSession;

typedef struct sSirSmeGetWPSPBCSessionsReq
{
    tANI_U16        messageType;    // eWNI_SME_GET_WPSPBC_SESSION_REQ
    tANI_U16        length;
    void            *pUsrContext;
    void            *pSapEventCallback;
    tSirMacAddr     bssId;          // BSSID
    tSirMacAddr     pRemoveMac;      // MAC Address of STA in WPS Session to be removed
}  tSirSmeGetWPSPBCSessionsReq, *tpSirSmeGetWPSPBCSessionsReq;

typedef struct sSirWPSPBCProbeReq
{
    tSirMacAddr        peerMacAddr;
    tANI_U16           probeReqIELen;
    tANI_U8            probeReqIE[512];
} tSirWPSPBCProbeReq, *tpSirWPSPBCProbeReq;

// probereq from peer, when wsc is enabled
typedef struct sSirSmeProbeReqInd
{
    tANI_U16           messageType; //  eWNI_SME_WPS_PBC_PROBE_REQ_IND
    tANI_U16           length;
    tANI_U8            sessionId;
    tSirMacAddr        bssId;
    tSirWPSPBCProbeReq WPSPBCProbeReq;
} tSirSmeProbeReqInd, *tpSirSmeProbeReqInd;

typedef struct sSirUpdateAPWPARSNIEsReq
{
    tANI_U16       messageType;      // eWNI_SME_SET_APWPARSNIEs_REQ
    tANI_U16       length;
    tANI_U16       transactionId; //Transaction ID for cmd
    tSirMacAddr    bssId;      // BSSID
    tANI_U8        sessionId;  //Session ID
    tSirRSNie      APWPARSNIEs;
} tSirUpdateAPWPARSNIEsReq, *tpSirUpdateAPWPARSNIEsReq;

#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
#define SIR_ROAM_MAX_CHANNELS            80
#define SIR_ROAM_SCAN_MAX_PB_REQ_SIZE    450
#define CHANNEL_LIST_STATIC                   1 /* Occupied channel list remains static */
#define CHANNEL_LIST_DYNAMIC_INIT             2 /* Occupied channel list can be learnt after init */
#define CHANNEL_LIST_DYNAMIC_FLUSH            3 /* Occupied channel list can be learnt after flush */
#define CHANNEL_LIST_DYNAMIC_UPDATE           4 /* Occupied channel list can be learnt after update */
#define SIR_ROAM_SCAN_24G_DEFAULT_CH     1
#define SIR_ROAM_SCAN_5G_DEFAULT_CH      36
#define SIR_ROAM_SCAN_RESERVED_BYTES     61
#endif //WLAN_FEATURE_ROAM_SCAN_OFFLOAD

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
#define SIR_ROAM_SCAN_PSK_SIZE    32
#define SIR_ROAM_R0KH_ID_MAX_LEN  48
#endif
// SME -> HAL - This is the host offload request.
#define SIR_IPV4_ARP_REPLY_OFFLOAD                  0
#define SIR_IPV6_NEIGHBOR_DISCOVERY_OFFLOAD         1
#define SIR_IPV6_NS_OFFLOAD                         2
#define SIR_OFFLOAD_DISABLE                         0
#define SIR_OFFLOAD_ENABLE                          1
#define SIR_OFFLOAD_BCAST_FILTER_ENABLE             0x2
#define SIR_OFFLOAD_MCAST_FILTER_ENABLE             0x4
#define SIR_OFFLOAD_ARP_AND_BCAST_FILTER_ENABLE     (SIR_OFFLOAD_ENABLE|SIR_OFFLOAD_BCAST_FILTER_ENABLE)
#define SIR_OFFLOAD_NS_AND_MCAST_FILTER_ENABLE      (SIR_OFFLOAD_ENABLE|SIR_OFFLOAD_MCAST_FILTER_ENABLE)

#ifdef WLAN_NS_OFFLOAD
typedef struct sSirNsOffloadReq
{
    tANI_U8 srcIPv6Addr[16];
    tANI_U8 selfIPv6Addr[SIR_MAC_NUM_TARGET_IPV6_NS_OFFLOAD_NA][SIR_MAC_IPV6_ADDR_LEN];
    tANI_U8 targetIPv6Addr[SIR_MAC_NUM_TARGET_IPV6_NS_OFFLOAD_NA][SIR_MAC_IPV6_ADDR_LEN];
    tANI_U8 selfMacAddr[6];
    tANI_U8 srcIPv6AddrValid;
    tANI_U8 targetIPv6AddrValid[SIR_MAC_NUM_TARGET_IPV6_NS_OFFLOAD_NA];
    tANI_U8 slotIdx;
} tSirNsOffloadReq, *tpSirNsOffloadReq;
#endif //WLAN_NS_OFFLOAD

typedef struct sSirHostOffloadReq
{
    tANI_U8 offloadType;
    tANI_U8 enableOrDisable;
    uint32_t num_ns_offload_count;
    union
    {
        tANI_U8 hostIpv4Addr [4];
        tANI_U8 hostIpv6Addr [16];
    } params;
#ifdef WLAN_NS_OFFLOAD
    tSirNsOffloadReq nsOffloadInfo;
#endif //WLAN_NS_OFFLOAD
    tSirMacAddr  bssId;
} tSirHostOffloadReq, *tpSirHostOffloadReq;

/* Packet Types. */
#define SIR_KEEP_ALIVE_NULL_PKT              1
#define SIR_KEEP_ALIVE_UNSOLICIT_ARP_RSP     2

/* Keep Alive request. */
typedef struct sSirKeepAliveReq
{
    v_U8_t          packetType;
    v_U32_t         timePeriod;
    tSirIpv4Addr    hostIpv4Addr;
    tSirIpv4Addr    destIpv4Addr;
    tSirMacAddr     destMacAddr;
    tSirMacAddr     bssId;
    tANI_U8         sessionId;
} tSirKeepAliveReq, *tpSirKeepAliveReq;

typedef struct sSirSmeAddStaSelfReq
{
    tANI_U16        mesgType;
    tANI_U16        mesgLen;
    tSirMacAddr     selfMacAddr;
    tVOS_CON_MODE   currDeviceMode;
    tANI_U32        type;
    tANI_U32        subType;
    tANI_U8         sessionId;
    tANI_U16        pkt_err_disconn_th;
    uint8_t         nss_2g;
    uint8_t         nss_5g;
}tSirSmeAddStaSelfReq, *tpSirSmeAddStaSelfReq;

typedef struct sSirSmeDelStaSelfReq
{
    tANI_U16        mesgType;
    tANI_U16        mesgLen;
    tSirMacAddr     selfMacAddr;
    tANI_U8         sessionId;
}tSirSmeDelStaSelfReq, *tpSirSmeDelStaSelfReq;

typedef struct sSirSmeAddStaSelfRsp
{
    tANI_U16        mesgType;
    tANI_U16        mesgLen;
    tANI_U16        status;
    tSirMacAddr     selfMacAddr;
}tSirSmeAddStaSelfRsp, *tpSirSmeAddStaSelfRsp;

typedef struct sSirSmeDelStaSelfRsp
{
    tANI_U16        mesgType;
    tANI_U16        mesgLen;
    tANI_U16        status;
    tSirMacAddr     selfMacAddr;
}tSirSmeDelStaSelfRsp, *tpSirSmeDelStaSelfRsp;

/* Coex Indication defines -
   should match WLAN_COEX_IND_DATA_SIZE
   should match WLAN_COEX_IND_TYPE_DISABLE_HB_MONITOR
   should match WLAN_COEX_IND_TYPE_ENABLE_HB_MONITOR */
#define SIR_COEX_IND_DATA_SIZE (4)
#define SIR_COEX_IND_TYPE_DISABLE_HB_MONITOR (0)
#define SIR_COEX_IND_TYPE_ENABLE_HB_MONITOR (1)
#define SIR_COEX_IND_TYPE_SCAN_COMPROMISED (2)
#define SIR_COEX_IND_TYPE_SCAN_NOT_COMPROMISED (3)
#define SIR_COEX_IND_TYPE_DISABLE_AGGREGATION_IN_2p4 (4)
#define SIR_COEX_IND_TYPE_ENABLE_AGGREGATION_IN_2p4 (5)

typedef struct sSirSmeCoexInd
{
    tANI_U16        mesgType;
    tANI_U16        mesgLen;
    tANI_U32        coexIndType;
    tANI_U32        coexIndData[SIR_COEX_IND_DATA_SIZE];
}tSirSmeCoexInd, *tpSirSmeCoexInd;

typedef struct sSirSmeMgmtFrameInd
{
    uint16_t        frame_len;
    tANI_U32        rxChan;
    tANI_U8        sessionId;
    tANI_U8         frameType;
    tANI_S8         rxRssi;
    tANI_U8  frameBuf[1]; //variable
}tSirSmeMgmtFrameInd, *tpSirSmeMgmtFrameInd;


typedef void (*sir_mgmt_frame_ind_callback)(tSirSmeMgmtFrameInd *frame_ind);
/**
 * struct sir_sme_mgmt_frame_cb_req - Register a
 * management frame callback req
 * @message_type: message id
 * @length: msg length
 * @callback: callback for management frame indication
 */
struct sir_sme_mgmt_frame_cb_req {
	uint16_t message_type;
	uint16_t length;
	sir_mgmt_frame_ind_callback callback;
};

typedef void (*sir_p2p_ack_ind_callback)(uint32_t session_id,
					bool tx_completion_status);

/**
 * struct sir_p2p_ack_ind_cb_req - Register a p2p ack ind callback req
 * @message_type: message id
 * @length: msg length
 * @callback: callback for p2p ack indication
 */
struct sir_sme_p2p_ack_ind_cb_req {
	uint16_t message_type;
	uint16_t length;
	sir_p2p_ack_ind_callback callback;
};

#ifdef WLAN_FEATURE_11W
typedef struct sSirSmeUnprotMgmtFrameInd
{
    tANI_U8         sessionId;
    tANI_U8         frameType;
    tANI_U8         frameLen;
    tANI_U8         frameBuf[1]; //variable
}tSirSmeUnprotMgmtFrameInd, *tpSirSmeUnprotMgmtFrameInd;
#endif

#define SIR_IS_FULL_POWER_REASON_DISCONNECTED(eReason) \
    ( ( eSME_LINK_DISCONNECTED_BY_HDD == (eReason) ) || \
      ( eSME_LINK_DISCONNECTED_BY_OTHER == (eReason) ) || \
      (eSME_FULL_PWR_NEEDED_BY_CHANNEL_SWITCH == (eReason)))
#define SIR_IS_FULL_POWER_NEEDED_BY_HDD(eReason) \
    ( ( eSME_LINK_DISCONNECTED_BY_HDD == (eReason) ) || ( eSME_FULL_PWR_NEEDED_BY_HDD == (eReason) ) )

/* P2P Power Save Related */
typedef struct sSirNoAParam
{
    tANI_U8 ctWindow:7;
    tANI_U8 OppPS:1;
    tANI_U8 count;
    tANI_U32 duration;
    tANI_U32 interval;
    tANI_U32 singleNoADuration;
    tANI_U8   psSelection;
}tSirNoAParam, *tpSirNoAParam;

typedef struct sSirWlanSuspendParam
{
    tANI_U8 configuredMcstBcstFilterSetting;
    tANI_U8 sessionId;
    tANI_U8 connectedState;
}tSirWlanSuspendParam,*tpSirWlanSuspendParam;

typedef struct sSirWlanResumeParam
{
    tANI_U8 configuredMcstBcstFilterSetting;
}tSirWlanResumeParam,*tpSirWlanResumeParam;

#ifdef WLAN_FEATURE_EXTWOW_SUPPORT

typedef enum ext_wow_type
{
    EXT_WOW_TYPE_APP_TYPE1, /* wow type: only enable wakeup for app type1 */
    EXT_WOW_TYPE_APP_TYPE2, /* wow type: only enable wakeup for app type2 */
    EXT_WOW_TYPE_APP_TYPE1_2, /* wow type: enable wakeup for app type1&2 */

}EXT_WOW_TYPE;

typedef struct
{
    tANI_U8 vdev_id;
    EXT_WOW_TYPE type;
    tANI_U32 wakeup_pin_num;
} tSirExtWoWParams, *tpSirExtWoWParams;

typedef struct
{
    tANI_U8 vdev_id;
    tSirMacAddr wakee_mac_addr;
    tANI_U8 identification_id[8];
    tANI_U8 password[16];
    tANI_U32 id_length;
    tANI_U32 pass_length;
} tSirAppType1Params, *tpSirAppType1Params;

typedef struct
{
    tANI_U8 vdev_id;

    tANI_U8 rc4_key[16];
    tANI_U32 rc4_key_len;

    /** ip header parameter */
    tANI_U32 ip_id; /* NC id */
    tANI_U32 ip_device_ip; /* NC IP address */
    tANI_U32 ip_server_ip; /* Push server IP address */

    /** tcp header parameter */
    tANI_U16 tcp_src_port; /* NC TCP port */
    tANI_U16 tcp_dst_port; /* Push server TCP port */
    tANI_U32 tcp_seq;
    tANI_U32 tcp_ack_seq;

    tANI_U32 keepalive_init; /* Initial ping interval */
    tANI_U32 keepalive_min; /* Minimum ping interval */
    tANI_U32 keepalive_max; /* Maximum ping interval */
    tANI_U32 keepalive_inc; /* Increment of ping interval */

    tSirMacAddr gateway_mac;
    tANI_U32 tcp_tx_timeout_val;
    tANI_U32 tcp_rx_timeout_val;
} tSirAppType2Params, *tpSirAppType2Params;
#endif

typedef struct sSirWlanSetRxpFilters
{
    tANI_U8 configuredMcstBcstFilterSetting;
    tANI_U8 setMcstBcstFilter;
}tSirWlanSetRxpFilters,*tpSirWlanSetRxpFilters;

#ifdef FEATURE_WLAN_SCAN_PNO
//
// PNO Messages
//

// Set PNO
#define SIR_PNO_MAX_NETW_CHANNELS  26
#define SIR_PNO_MAX_NETW_CHANNELS_EX  60
#define SIR_PNO_MAX_SUPP_NETWORKS  16

/*size based of dot11 declaration without extra IEs as we will not carry those for PNO*/
#define SIR_PNO_MAX_PB_REQ_SIZE    450

#define SIR_PNO_24G_DEFAULT_CH     1
#define SIR_PNO_5G_DEFAULT_CH      36

typedef enum
{
   SIR_PNO_MODE_IMMEDIATE,
   SIR_PNO_MODE_ON_SUSPEND,
   SIR_PNO_MODE_ON_RESUME,
   SIR_PNO_MODE_MAX
} eSirPNOMode;

typedef struct
{
  tSirMacSSid ssId;
  tANI_U32    authentication;
  tANI_U32    encryption;
  tANI_U32    bcastNetwType;
  tANI_U8     ucChannelCount;
  tANI_U8     aChannels[SIR_PNO_MAX_NETW_CHANNELS_EX];
  tANI_S32    rssiThreshold;
} tSirNetworkType;

/**
 * struct sSirPNOScanReq - PNO Scan request structure
 * @enable: flag to enable or disable
 * @modePNO: PNO Mode
 * @ucNetworksCount: Number of networks
 * @do_passive_scan: Flag to request passive scan to fw
 * @aNetworks: Preferred network list
 * @sessionId: Session identifier
 * @fast_scan_period: Fast Scan period
 * @slow_scan_period: Slow scan period
 * @fast_scan_max_cycles: Fast scan max cycles
 * @active_min_time: Min value of dwell time for active scan
 * @active_max_time: Max value of dwell time for active scan
 * @passive_min_time: Min value of dwell time for passive scan
 * @passive_max_time: Max value of dwell time for passive scan
 * @us24GProbeTemplateLen: 2.4G probe template length
 * @p24GProbeTemplate: 2.4G probe template
 * @us5GProbeTemplateLen: 5G probe template length
 * @p5GProbeTemplate: 5G probe template
 */
typedef struct sSirPNOScanReq {
	uint8_t         enable;
	eSirPNOMode     modePNO;
	bool            do_passive_scan;
	uint8_t         ucNetworksCount;
	tSirNetworkType aNetworks[SIR_PNO_MAX_SUPP_NETWORKS];
	uint8_t         sessionId;
	uint32_t        fast_scan_period;
	uint32_t        slow_scan_period;
	uint8_t         fast_scan_max_cycles;

	uint32_t        active_min_time;
	uint32_t        active_max_time;
	uint32_t        passive_min_time;
	uint32_t        passive_max_time;

	uint16_t        us24GProbeTemplateLen;
	uint8_t         p24GProbeTemplate[SIR_PNO_MAX_PB_REQ_SIZE];
	uint16_t        us5GProbeTemplateLen;
	uint8_t         p5GProbeTemplate[SIR_PNO_MAX_PB_REQ_SIZE];
} tSirPNOScanReq, *tpSirPNOScanReq;

typedef struct sSirSetRSSIFilterReq
{
  tANI_U8     rssiThreshold;
} tSirSetRSSIFilterReq, *tpSirSetRSSIFilterReq;


// Update Scan Params
typedef struct {
  tANI_U8   b11dEnabled;
  tANI_U8   b11dResolved;
  tANI_U8   ucChannelCount;
  tANI_U8   aChannels[SIR_PNO_MAX_NETW_CHANNELS_EX];
  tANI_U16  usPassiveMinChTime;
  tANI_U16  usPassiveMaxChTime;
  tANI_U16  usActiveMinChTime;
  tANI_U16  usActiveMaxChTime;
  tANI_U8   ucCBState;
} tSirUpdateScanParams, * tpSirUpdateScanParams;

// Preferred Network Found Indication
typedef struct
{
  tANI_U16      mesgType;
  tANI_U16      mesgLen;
  /* Network that was found with the highest RSSI*/
  tSirMacSSid   ssId;
  /* Indicates the RSSI */
  tANI_U8       rssi;
  /* Length of the beacon or probe response
   * corresponding to the candidate found by PNO */
  tANI_U32      frameLength;
  tANI_U8       sessionId;
  /* Index to memory location where the contents of
   * beacon or probe response frame will be copied */
  tANI_U8       data[1];
} tSirPrefNetworkFoundInd, *tpSirPrefNetworkFoundInd;
#endif //FEATURE_WLAN_SCAN_PNO

#ifdef WLAN_FEATURE_ROAM_OFFLOAD
typedef struct {
  tANI_U8    acvo_uapsd: 1;
  tANI_U8    acvi_uapsd: 1;
  tANI_U8    acbk_uapsd: 1;
  tANI_U8    acbe_uapsd: 1;
  tANI_U8    reserved: 4;
} tSirAcUapsd, *tpSirAcUapsd;
#endif

#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
typedef struct
{
  tSirMacSSid ssId;
  tANI_U8     currAPbssid[VOS_MAC_ADDR_SIZE];
  tANI_U32    authentication;
  tANI_U8     encryption;
  tANI_U8     mcencryption;
  tANI_U8     ChannelCount;
  tANI_U8     ChannelCache[SIR_ROAM_MAX_CHANNELS];
#ifdef WLAN_FEATURE_11W
  tANI_BOOLEAN MFPEnabled;
#endif

} tSirRoamNetworkType;

typedef struct SirMobilityDomainInfo
{
  tANI_U8 mdiePresent;
  tANI_U16 mobilityDomain;
} tSirMobilityDomainInfo;

typedef enum {
        SIR_ROAMING_DFS_CHANNEL_DISABLED = 0,
        SIR_ROAMING_DFS_CHANNEL_ENABLED_NORMAL = 1,
        SIR_ROAMING_DFS_CHANNEL_ENABLED_ACTIVE = 2
} eSirDFSRoamScanMode;
#define MAX_SSID_ALLOWED_LIST 4
#define MAX_BSSID_AVOID_LIST  16
#define MAX_BSSID_FAVORED     16
struct roam_ext_params {
	uint8_t num_bssid_avoid_list;
	uint8_t num_ssid_allowed_list;
	uint8_t num_bssid_favored;
	tSirMacSSid ssid_allowed_list[MAX_SSID_ALLOWED_LIST];
	tSirMacAddr bssid_avoid_list[MAX_BSSID_AVOID_LIST];
	tSirMacAddr bssid_favored[MAX_BSSID_FAVORED];
	uint8_t bssid_favored_factor[MAX_BSSID_FAVORED];
	int raise_rssi_thresh_5g;
	int drop_rssi_thresh_5g;
	uint8_t raise_rssi_type_5g;
	uint8_t raise_factor_5g;
	uint8_t drop_rssi_type_5g;
	uint8_t drop_factor_5g;
	int max_raise_rssi_5g;
	int max_drop_rssi_5g;
	int alert_rssi_threshold;
	int rssi_diff;
	int good_rssi_roam;
	bool is_5g_pref_enabled;
};

typedef struct sSirRoamOffloadScanReq
{
  uint16_t    message_type;
  uint16_t    length;
  eAniBoolean RoamScanOffloadEnabled;
  eAniBoolean MAWCEnabled;
  tANI_S8     LookupThreshold;
  tANI_U8     delay_before_vdev_stop;
  tANI_U8     OpportunisticScanThresholdDiff;
  tANI_U8     RoamRescanRssiDiff;
  tANI_U8     RoamRssiDiff;
  tANI_U8     ChannelCacheType;
  tANI_U8     Command;
  tANI_U8     reason;
  tANI_U16    NeighborScanTimerPeriod;
  tANI_U16    NeighborRoamScanRefreshPeriod;
  tANI_U16    NeighborScanChannelMinTime;
  tANI_U16    NeighborScanChannelMaxTime;
  tANI_U16    EmptyRefreshScanPeriod;
  tANI_U8     ValidChannelCount;
  tANI_U8     ValidChannelList[SIR_ROAM_MAX_CHANNELS];
  eAniBoolean IsESEAssoc;
  tANI_U16  us24GProbeTemplateLen;
  tANI_U8   p24GProbeTemplate[SIR_ROAM_SCAN_MAX_PB_REQ_SIZE];
  tANI_U16  us5GProbeTemplateLen;
  tANI_U8   p5GProbeTemplate[SIR_ROAM_SCAN_MAX_PB_REQ_SIZE];
  tANI_U8     ReservedBytes[SIR_ROAM_SCAN_RESERVED_BYTES];
  /*ReservedBytes is to add any further params in future
    without changing the interface params on Host
    and firmware.The firmware right now checks
    if the size of this structure matches and then
    proceeds with the processing of the command.
    So, in future, if there is any need to add
    more params, pick the memory from reserved
    bytes and keep deducting the reserved bytes
    by the amount of bytes picked.*/
  tANI_U8   nProbes;
  tANI_U16  HomeAwayTime;
  tSirRoamNetworkType ConnectedNetwork;
  tSirMobilityDomainInfo MDID;
  tANI_U8 sessionId;
  tANI_U8   RoamBmissFirstBcnt;
  tANI_U8   RoamBmissFinalBcnt;
  tANI_U8   RoamBeaconRssiWeight;
  eSirDFSRoamScanMode  allowDFSChannelRoam;
#ifdef WLAN_FEATURE_ROAM_OFFLOAD
  tANI_U8   RoamOffloadEnabled;
  tANI_U8   PSK_PMK[SIR_ROAM_SCAN_PSK_SIZE];
  tANI_U32  pmk_len;
  tANI_U8   Prefer5GHz;
  tANI_U8   RoamRssiCatGap;
  tANI_U8   Select5GHzMargin;
  tANI_U8   KRK[SIR_KRK_KEY_LEN];
  tANI_U8   BTK[SIR_BTK_KEY_LEN];
  tANI_U32   ReassocFailureTimeout;
  tSirAcUapsd AcUapsd;
  tANI_U8   R0KH_ID[SIR_ROAM_R0KH_ID_MAX_LEN];
  tANI_U32  R0KH_ID_Length;
  tANI_U8   RoamKeyMgmtOffloadEnabled;
#endif
  struct roam_ext_params roam_params;
  uint32_t  hi_rssi_scan_max_count;
  uint32_t  hi_rssi_scan_rssi_delta;
  uint32_t  hi_rssi_scan_delay;
  int32_t   hi_rssi_scan_rssi_ub;
  uint8_t  middle_of_roaming;

} tSirRoamOffloadScanReq, *tpSirRoamOffloadScanReq;

typedef struct sSirRoamOffloadScanRsp
{
  tANI_U8  sessionId;
  tANI_U32 reason;
} tSirRoamOffloadScanRsp, *tpSirRoamOffloadScanRsp;

#endif //WLAN_FEATURE_ROAM_SCAN_OFFLOAD

#define SIR_NOCHANGE_POWER_VALUE  0xFFFFFFFF

//Power Parameters Type
typedef enum
{
   eSIR_IGNORE_DTIM        = 1,
   eSIR_LISTEN_INTERVAL    = 2,
   eSIR_MCAST_BCAST_FILTER = 3,
   eSIR_ENABLE_BET         = 4,
   eSIR_BET_INTERVAL       = 5
}tPowerParamType;

//Power Parameters Value s
typedef struct
{
  /*  Ignore DTIM */
  tANI_U32 uIgnoreDTIM;

  /* DTIM Period */
  tANI_U32 uDTIMPeriod;

  /* Listen Interval */
  tANI_U32 uListenInterval;

  /* Broadcast Multicast Filter */
  tANI_U32 uBcastMcastFilter;

  /* Beacon Early Termination */
  tANI_U32 uEnableBET;

  /* Beacon Early Termination Interval */
  tANI_U32 uBETInterval;

  /* MAX LI for modulated DTIM */
  tANI_U32 uMaxLIModulatedDTIM;

}tSirSetPowerParamsReq, *tpSirSetPowerParamsReq;

typedef struct sSirTxPerTrackingParam
{
    tANI_U8  ucTxPerTrackingEnable;           /* 0: disable, 1:enable */
    tANI_U8  ucTxPerTrackingPeriod;              /* Check period, unit is sec. Once tx_stat_chk enable, firmware will check PER in this period periodically */
    tANI_U8  ucTxPerTrackingRatio;            /* (Fail TX packet)/(Total TX packet) ratio, the unit is 10%. for example, 5 means 50% TX failed rate, default is 5. If current TX packet failed rate bigger than this ratio then firmware send WLC_E_TX_STAT_ERROR event to driver */
    tANI_U32 uTxPerTrackingWatermark;               /* A watermark of check number, once the tx packet exceed this number, we do the check, default is 5 */
}tSirTxPerTrackingParam, *tpSirTxPerTrackingParam;

#ifdef WLAN_FEATURE_PACKET_FILTERING
/*---------------------------------------------------------------------------
  Packet Filtering Parameters
---------------------------------------------------------------------------*/
#define    SIR_IPV4_ADDR_LEN                 4
#define    SIR_MAC_ADDR_LEN                  6
#define    SIR_MAX_FILTER_TEST_DATA_LEN       8
#define    SIR_MAX_NUM_MULTICAST_ADDRESS    240
#define    SIR_MAX_NUM_FILTERS               20
#define    SIR_MAX_NUM_TESTS_PER_FILTER      10

//
// Receive Filter Parameters
//
typedef enum
{
  SIR_RCV_FILTER_TYPE_INVALID,
  SIR_RCV_FILTER_TYPE_FILTER_PKT,
  SIR_RCV_FILTER_TYPE_BUFFER_PKT,
  SIR_RCV_FILTER_TYPE_MAX_ENUM_SIZE
}eSirReceivePacketFilterType;

typedef enum
{
  SIR_FILTER_HDR_TYPE_INVALID,
  SIR_FILTER_HDR_TYPE_MAC,
  SIR_FILTER_HDR_TYPE_ARP,
  SIR_FILTER_HDR_TYPE_IPV4,
  SIR_FILTER_HDR_TYPE_IPV6,
  SIR_FILTER_HDR_TYPE_UDP,
  SIR_FILTER_HDR_TYPE_MAX
}eSirRcvPktFltProtocolType;

typedef enum
{
  SIR_FILTER_CMP_TYPE_INVALID,
  SIR_FILTER_CMP_TYPE_EQUAL,
  SIR_FILTER_CMP_TYPE_MASK_EQUAL,
  SIR_FILTER_CMP_TYPE_NOT_EQUAL,
  SIR_FILTER_CMP_TYPE_MASK_NOT_EQUAL,
  SIR_FILTER_CMP_TYPE_MAX
}eSirRcvPktFltCmpFlagType;

typedef struct sSirRcvPktFilterFieldParams
{
  eSirRcvPktFltProtocolType        protocolLayer;
  eSirRcvPktFltCmpFlagType         cmpFlag;
  /* Length of the data to compare */
  tANI_U16                         dataLength;
  /* from start of the respective frame header */
  tANI_U8                          dataOffset;
  /* Reserved field */
  tANI_U8                          reserved;
  /* Data to compare */
  tANI_U8                          compareData[SIR_MAX_FILTER_TEST_DATA_LEN];
  /* Mask to be applied on the received packet data before compare */
  tANI_U8                          dataMask[SIR_MAX_FILTER_TEST_DATA_LEN];
}tSirRcvPktFilterFieldParams, *tpSirRcvPktFilterFieldParams;

typedef struct sSirRcvPktFilterCfg
{
  tANI_U8                         filterId;
  eSirReceivePacketFilterType     filterType;
  tANI_U32                        numFieldParams;
  tANI_U32                        coalesceTime;
  tSirMacAddr                     selfMacAddr;
  tSirMacAddr                     bssId; //Bssid of the connected AP
  tSirRcvPktFilterFieldParams     paramsData[SIR_MAX_NUM_TESTS_PER_FILTER];
}tSirRcvPktFilterCfgType, *tpSirRcvPktFilterCfgType;

//
// Filter Packet Match Count Parameters
//
typedef struct sSirRcvFltPktMatchCnt
{
  tANI_U8    filterId;
  tANI_U32   matchCnt;
} tSirRcvFltPktMatchCnt, tpSirRcvFltPktMatchCnt;

typedef struct sSirRcvFltPktMatchRsp
{
  tANI_U16        mesgType;
  tANI_U16        mesgLen;

  /* Success or Failure */
  tANI_U32                 status;
  tSirRcvFltPktMatchCnt    filterMatchCnt[SIR_MAX_NUM_FILTERS];
  tSirMacAddr      bssId;
} tSirRcvFltPktMatchRsp, *tpSirRcvFltPktMatchRsp;

//
// Receive Filter Clear Parameters
//
typedef struct sSirRcvFltPktClearParam
{
  tANI_U32   status;  /* only valid for response message */
  tANI_U8    filterId;
  tSirMacAddr selfMacAddr;
  tSirMacAddr bssId;
}tSirRcvFltPktClearParam, *tpSirRcvFltPktClearParam;

//
// Multicast Address List Parameters
//
typedef struct sSirRcvFltMcAddrList
{
  tANI_U32       ulMulticastAddrCnt;
  tSirMacAddr    multicastAddr[SIR_MAX_NUM_MULTICAST_ADDRESS];
  tSirMacAddr    selfMacAddr;
  tSirMacAddr    bssId;
  tANI_U8        action;
} tSirRcvFltMcAddrList, *tpSirRcvFltMcAddrList;
#endif // WLAN_FEATURE_PACKET_FILTERING

//
// Generic version information
//
typedef struct
{
  tANI_U8    revision;
  tANI_U8    version;
  tANI_U8    minor;
  tANI_U8    major;
} tSirVersionType;

typedef struct sAniBtAmpLogLinkReq
{
    // Common for all types are requests
    tANI_U16                msgType;    // message type is same as the request type
    tANI_U16                msgLen;  // length of the entire request
    tANI_U8                 sessionId; //sme Session Id
    void                   *btampHandle; //AMP context

} tAniBtAmpLogLinkReq, *tpAniBtAmpLogLinkReq;

#ifdef WLAN_FEATURE_GTK_OFFLOAD
/*---------------------------------------------------------------------------
* WDA_GTK_OFFLOAD_REQ
*--------------------------------------------------------------------------*/
typedef struct
{
  tANI_U32     ulFlags;             /* optional flags */
  tANI_U8      aKCK[16];            /* Key confirmation key */
  tANI_U8      aKEK[16];            /* key encryption key */
  tANI_U64     ullKeyReplayCounter; /* replay counter */
  tSirMacAddr  bssId;
} tSirGtkOffloadParams, *tpSirGtkOffloadParams;

/**
 * struct sir_wifi_start_log - Structure to store the params sent to start/
 * stop logging
 * @name:          Attribute which indicates the type of logging like per packet
 *                 statistics, connectivity etc.
 * @verbose_level: Verbose level which can be 0,1,2,3
 * @flag:          Flag field for future use
 */
struct sir_wifi_start_log {
	uint32_t ring_id;
	uint32_t verbose_level;
	uint32_t flag;
};

/*---------------------------------------------------------------------------
* WDA_GTK_OFFLOAD_GETINFO_REQ
*--------------------------------------------------------------------------*/
typedef struct
{
   tANI_U16   mesgType;
   tANI_U16   mesgLen;

   tANI_U32   ulStatus;             /* success or failure */
   tANI_U64   ullKeyReplayCounter;  /* current replay counter value */
   tANI_U32   ulTotalRekeyCount;    /* total rekey attempts */
   tANI_U32   ulGTKRekeyCount;      /* successful GTK rekeys */
   tANI_U32   ulIGTKRekeyCount;     /* successful iGTK rekeys */
   tSirMacAddr bssId;
} tSirGtkOffloadGetInfoRspParams, *tpSirGtkOffloadGetInfoRspParams;
#endif // WLAN_FEATURE_GTK_OFFLOAD

#ifdef WLAN_WAKEUP_EVENTS
/*---------------------------------------------------------------------------
  tSirWakeReasonInd
---------------------------------------------------------------------------*/
typedef struct
{
    tANI_U16      mesgType;
    tANI_U16      mesgLen;
    tANI_U32      ulReason;        /* see tWakeReasonType */
    tANI_U32      ulReasonArg;     /* argument specific to the reason type */
    tANI_U32      ulStoredDataLen; /* length of optional data stored in this message, in case
                              HAL truncates the data (i.e. data packets) this length
                              will be less than the actual length */
    tANI_U32      ulActualDataLen; /* actual length of data */
    tANI_U8       aDataStart[1];  /* variable length start of data (length == storedDataLen)
                             see specific wake type */
} tSirWakeReasonInd, *tpSirWakeReasonInd;
#endif // WLAN_WAKEUP_EVENTS

/*---------------------------------------------------------------------------
  sAniSetTmLevelReq
---------------------------------------------------------------------------*/
typedef struct sAniSetTmLevelReq
{
    tANI_U16                tmMode;
    tANI_U16                newTmLevel;
} tAniSetTmLevelReq, *tpAniSetTmLevelReq;

#ifdef FEATURE_WLAN_TDLS
/* TDLS Request struct SME-->PE */
typedef struct sSirTdlsSendMgmtReq
{
    tANI_U16            messageType;   // eWNI_SME_TDLS_DISCOVERY_START_REQ
    tANI_U16            length;
    tANI_U8             sessionId;     // Session ID
    tANI_U16            transactionId; // Transaction ID for cmd
    tANI_U8             reqType;
    tANI_U8             dialog;
    tANI_U16            statusCode;
    tANI_U8             responder;
    tANI_U32            peerCapability;
    tSirMacAddr         bssid;         // For multi-session, for PE to locate peSession ID
    tSirMacAddr         peerMac;

    /* Variable length. Dont add any field after this. */
    tANI_U8             addIe[1];
} tSirTdlsSendMgmtReq, *tpSirSmeTdlsSendMgmtReq ;

typedef enum TdlsAddOper
{
    TDLS_OPER_NONE,
    TDLS_OPER_ADD,
    TDLS_OPER_UPDATE
} eTdlsAddOper;

/* TDLS Request struct SME-->PE */
typedef struct sSirTdlsAddStaReq
{
    tANI_U16            messageType;   // eWNI_SME_TDLS_DISCOVERY_START_REQ
    tANI_U16            length;
    tANI_U8             sessionId;     // Session ID
    tANI_U16            transactionId; // Transaction ID for cmd
    tSirMacAddr         bssid;         // For multi-session, for PE to locate peSession ID
    eTdlsAddOper        tdlsAddOper;
    tSirMacAddr         peerMac;
    tANI_U16            capability;
    tANI_U8             extn_capability[SIR_MAC_MAX_EXTN_CAP];
    tANI_U8             supported_rates_length;
    tANI_U8             supported_rates[SIR_MAC_MAX_SUPP_RATES];
    tANI_U8             htcap_present;
    tSirHTCap           htCap;
    tANI_U8             vhtcap_present;
    tSirVHTCap          vhtCap;
    tANI_U8             uapsd_queues;
    tANI_U8             max_sp;
} tSirTdlsAddStaReq, *tpSirSmeTdlsAddStaReq ;

/* TDLS Response struct PE-->SME */
typedef struct sSirTdlsAddStaRsp
{
    tANI_U16               messageType;
    tANI_U16               length;
    tSirResultCodes        statusCode;
    tSirMacAddr            peerMac;
    tANI_U8                sessionId;     // Session ID
    tANI_U16               staId ;
    tANI_U16               staType ;
    tANI_U8                ucastSig;
    tANI_U8                bcastSig;
    eTdlsAddOper           tdlsAddOper;
} tSirTdlsAddStaRsp ;

/* TDLS Request struct SME-->PE */
typedef struct
{
    tANI_U16            messageType;   // eWNI_SME_TDLS_LINK_ESTABLISH_REQ
    tANI_U16            length;
    tANI_U8             sessionId;     // Session ID
    tANI_U16            transactionId; // Transaction ID for cmd
    tANI_U8             uapsdQueues;   // Peer's uapsd Queues Information
    tANI_U8             maxSp;         // Peer's Supported Maximum Service Period
    tANI_U8             isBufSta;      // Does Peer Support as Buffer Station.
    tANI_U8             isOffChannelSupported;    // Does Peer Support as TDLS Off Channel.
    tANI_U8             isResponder;   // Is Peer a responder.
    tSirMacAddr         bssid;         // For multi-session, for PE to locate peSession ID
    tSirMacAddr         peerMac;
    tANI_U8             supportedChannelsLen;
    tANI_U8             supportedChannels[SIR_MAC_MAX_SUPP_CHANNELS];
    tANI_U8             supportedOperClassesLen;
    tANI_U8             supportedOperClasses[SIR_MAC_MAX_SUPP_OPER_CLASSES];
}tSirTdlsLinkEstablishReq, *tpSirTdlsLinkEstablishReq;

/* TDLS Request struct SME-->PE */
typedef struct
{
    tANI_U16            messageType;   // eWNI_SME_TDLS_LINK_ESTABLISH_RSP
    tANI_U16            length;
    tANI_U8             sessionId;     // Session ID
    tANI_U16            transactionId; // Transaction ID for cmd
    tSirResultCodes        statusCode;
    tSirMacAddr            peerMac;
}tSirTdlsLinkEstablishReqRsp, *tpSirTdlsLinkEstablishReqRsp;

/* TDLS Request struct SME-->PE */
typedef struct sSirTdlsDelStaReq
{
    tANI_U16            messageType;   // eWNI_SME_TDLS_DISCOVERY_START_REQ
    tANI_U16            length;
    tANI_U8             sessionId;     // Session ID
    tANI_U16            transactionId; // Transaction ID for cmd
    tSirMacAddr         bssid;         // For multi-session, for PE to locate peSession ID
    tSirMacAddr         peerMac;
} tSirTdlsDelStaReq, *tpSirSmeTdlsDelStaReq ;
/* TDLS Response struct PE-->SME */
typedef struct sSirTdlsDelStaRsp
{
   tANI_U16               messageType;
   tANI_U16               length;
   tANI_U8                sessionId;     // Session ID
   tSirResultCodes        statusCode;
   tSirMacAddr            peerMac;
   tANI_U16               staId;
} tSirTdlsDelStaRsp, *tpSirTdlsDelStaRsp;
/* TDLS Delete Indication struct PE-->SME */
typedef struct sSirTdlsDelStaInd
{
   tANI_U16               messageType;
   tANI_U16               length;
   tANI_U8                sessionId;     // Session ID
   tSirMacAddr            peerMac;
   tANI_U16               staId;
   tANI_U16               reasonCode;
} tSirTdlsDelStaInd, *tpSirTdlsDelStaInd;
typedef struct sSirTdlsDelAllPeerInd
{
   tANI_U16               messageType;
   tANI_U16               length;
   tANI_U8                sessionId;     // Session ID
} tSirTdlsDelAllPeerInd, *tpSirTdlsDelAllPeerInd;
#ifdef FEATURE_WLAN_TDLS_DISAPPEAR_AP
typedef struct sSirTdlsDisappearAPInd
{
   tANI_U16               messageType;
   tANI_U16               length;
   tANI_U8                sessionId;     // Session ID
   tANI_U16               staId;
   tSirMacAddr            staAddr;
} tSirTdlsDisappearAPInd, *tpSirTdlsDisappearAPInd;
#endif
typedef struct sSirMgmtTxCompletionInd
{
   tANI_U16               messageType;
   tANI_U16               length;
   tANI_U8                sessionId;     // Session ID
   tANI_U32               txCompleteStatus;
} tSirMgmtTxCompletionInd, *tpSirMgmtTxCompletionInd;

typedef struct sSirTdlsEventNotify
{
   tANI_U8         sessionId;
   tSirMacAddr     peerMac;
   tANI_U16        messageType;
   tANI_U32        peer_reason;
} tSirTdlsEventNotify;
#endif /* FEATURE_WLAN_TDLS */


typedef struct sSirActiveModeSetBcnFilterReq
{
   tANI_U16               messageType;
   tANI_U16               length;
   tANI_U8                seesionId;
} tSirSetActiveModeSetBncFilterReq, *tpSirSetActiveModeSetBncFilterReq;

//Reset AP Caps Changed
typedef struct sSirResetAPCapsChange
{
    tANI_U16       messageType;
    tANI_U16       length;
    tSirMacAddr    bssId;
} tSirResetAPCapsChange, *tpSirResetAPCapsChange;

/// Definition for Candidate found indication from FW
typedef struct sSirSmeCandidateFoundInd
{
    tANI_U16            messageType; // eWNI_SME_CANDIDATE_FOUND_IND
    tANI_U16            length;
    tANI_U8             sessionId;  // Session Identifier
} tSirSmeCandidateFoundInd, *tpSirSmeCandidateFoundInd;

#ifdef WLAN_FEATURE_11W
typedef struct sSirWlanExcludeUnencryptParam
{
    tANI_BOOLEAN    excludeUnencrypt;
    tSirMacAddr     bssId;
}tSirWlanExcludeUnencryptParam,*tpSirWlanExcludeUnencryptParam;
#endif

typedef enum {
    P2P_SCAN_TYPE_SEARCH = 1,    /* P2P Search */
    P2P_SCAN_TYPE_LISTEN     /* P2P Listen */
}tSirP2pScanType;

typedef struct sAniHandoffReq
{
    // Common for all types are requests
    tANI_U16  msgType; // message type is same as the request type
    tANI_U16  msgLen;  // length of the entire request
    tANI_U8   sessionId;
    tANI_U8   bssid[VOS_MAC_ADDR_SIZE];
    tANI_U8   channel;
    tANI_U8   handoff_src;
} tAniHandoffReq, *tpAniHandoffReq;

typedef struct sSirScanOffloadReq {
    tANI_U8 sessionId;
    tSirMacAddr bssId;
    tANI_U8 numSsid;
    tSirMacSSid ssId[SIR_SCAN_MAX_NUM_SSID];
    tANI_U8 hiddenSsid;
    tSirMacAddr selfMacAddr;
    tSirBssType bssType;
    tANI_U8 dot11mode;
    tSirScanType scanType;
    tANI_U32 minChannelTime;
    tANI_U32 maxChannelTime;
    /*in units of milliseconds, ignored when not connected*/
    uint32_t restTime;
    /*in units of milliseconds, ignored when not connected*/
    uint32_t min_rest_time;
    /*in units of milliseconds, ignored when not connected*/
    uint32_t idle_time;
    tSirP2pScanType p2pScanType;
    tANI_U16 uIEFieldLen;
    tANI_U16 uIEFieldOffset;
    tSirChannelList channelList;
    /*-----------------------------
      sSirScanOffloadReq....
      -----------------------------
      uIEFieldLen
      -----------------------------
      uIEFieldOffset               ----+
      -----------------------------    |
      channelList.numChannels          |
      -----------------------------    |
      ... variable size up to          |
      channelNumber[numChannels-1]     |
      This can be zero, if             |
      numChannel is zero.              |
      ----------------------------- <--+
      ... variable size uIEField
      up to uIEFieldLen (can be 0)
      -----------------------------*/
} tSirScanOffloadReq, *tpSirScanOffloadReq;

typedef enum sSirScanEventType {
    SCAN_EVENT_STARTED=0x1,          /* Scan command accepted by FW */
    SCAN_EVENT_COMPLETED=0x2,        /* Scan has been completed by FW */
    SCAN_EVENT_BSS_CHANNEL=0x4,      /* FW is going to move to HOME channel */
    SCAN_EVENT_FOREIGN_CHANNEL = 0x8,/* FW is going to move to FORIEGN channel */
    SCAN_EVENT_DEQUEUED=0x10,       /* scan request got dequeued */
    SCAN_EVENT_PREEMPTED=0x20,      /* preempted by other high priority scan */
    SCAN_EVENT_START_FAILED=0x40,   /* scan start failed */
    SCAN_EVENT_RESTARTED=0x80,      /*scan restarted*/
    SCAN_EVENT_MAX=0x8000
} tSirScanEventType;

typedef struct sSirScanOffloadEvent{
    tSirScanEventType event;
    tSirResultCodes reasonCode;
    tANI_U32 chanFreq;
    tANI_U32 requestor;
    tANI_U32 scanId;
    tSirP2pScanType p2pScanType;
    tANI_U8 sessionId;
} tSirScanOffloadEvent, *tpSirScanOffloadEvent;

typedef struct sSirUpdateChanParam
{
    tANI_U8 chanId;
    tANI_U8 pwr;
    tANI_BOOLEAN dfsSet;
    bool half_rate;
    bool quarter_rate;
} tSirUpdateChanParam, *tpSirUpdateChanParam;

typedef struct sSirUpdateChan
{
    tANI_U8 numChan;
    uint8_t ht_en;
    uint8_t vht_en;
    uint8_t vht_24_en;
    tSirUpdateChanParam chanParam[1];
} tSirUpdateChanList, *tpSirUpdateChanList;

typedef enum eSirAddonPsReq
{
    eSIR_ADDON_NOTHING,
    eSIR_ADDON_ENABLE_UAPSD,
    eSIR_ADDON_DISABLE_UAPSD
}tSirAddonPsReq;

/* Powersave Offload data */
typedef struct sSirPsReqData
{
    /* BSSID */
    tSirMacAddr bssId;

    /* Additional Info */
    tSirAddonPsReq addOnReq;
} tSirPsReqData,*tpSirPsReqData;

#ifdef FEATURE_WLAN_LPHB
#define SIR_LPHB_FILTER_LEN   64

typedef enum
{
   LPHB_SET_EN_PARAMS_INDID,
   LPHB_SET_TCP_PARAMS_INDID,
   LPHB_SET_TCP_PKT_FILTER_INDID,
   LPHB_SET_UDP_PARAMS_INDID,
   LPHB_SET_UDP_PKT_FILTER_INDID,
   LPHB_SET_NETWORK_INFO_INDID,
} LPHBIndType;

typedef struct sSirLPHBEnableStruct
{
   v_U8_t enable;
   v_U8_t item;
   v_U8_t session;
} tSirLPHBEnableStruct;

typedef struct sSirLPHBTcpParamStruct
{
   v_U32_t      srv_ip;
   v_U32_t      dev_ip;
   v_U16_t      src_port;
   v_U16_t      dst_port;
   v_U16_t      timeout;
   v_U8_t       session;
   tSirMacAddr  gateway_mac;
   uint16       timePeriodSec; // in seconds
   uint32       tcpSn;
} tSirLPHBTcpParamStruct;

typedef struct sSirLPHBTcpFilterStruct
{
   v_U16_t length;
   v_U8_t  offset;
   v_U8_t  session;
   v_U8_t  filter[SIR_LPHB_FILTER_LEN];
} tSirLPHBTcpFilterStruct;

typedef struct sSirLPHBUdpParamStruct
{
   v_U32_t      srv_ip;
   v_U32_t      dev_ip;
   v_U16_t      src_port;
   v_U16_t      dst_port;
   v_U16_t      interval;
   v_U16_t      timeout;
   v_U8_t       session;
   tSirMacAddr  gateway_mac;
} tSirLPHBUdpParamStruct;

typedef struct sSirLPHBUdpFilterStruct
{
   v_U16_t length;
   v_U8_t  offset;
   v_U8_t  session;
   v_U8_t  filter[SIR_LPHB_FILTER_LEN];
} tSirLPHBUdpFilterStruct;

typedef struct sSirLPHBReq
{
   v_U16_t cmd;
   v_U16_t dummy;
   union
   {
      tSirLPHBEnableStruct     lphbEnableReq;
      tSirLPHBTcpParamStruct   lphbTcpParamReq;
      tSirLPHBTcpFilterStruct  lphbTcpFilterReq;
      tSirLPHBUdpParamStruct   lphbUdpParamReq;
      tSirLPHBUdpFilterStruct  lphbUdpFilterReq;
   } params;
} tSirLPHBReq;

typedef struct sSirLPHBInd
{
   v_U8_t sessionIdx;
   v_U8_t protocolType; /*TCP or UDP*/
   v_U8_t eventReason;
} tSirLPHBInd;
#endif /* FEATURE_WLAN_LPHB */

#ifdef FEATURE_WLAN_CH_AVOID
typedef struct sSirChAvoidUpdateReq
{
   tANI_U32 reserved_param;
} tSirChAvoidUpdateReq;
#endif /* FEATURE_WLAN_CH_AVOID */

typedef struct sSirLinkSpeedInfo
{
  /* MAC Address for the peer */
  tSirMacAddr peer_macaddr;
  tANI_U32 estLinkSpeed;     //Linkspeed from firmware
} tSirLinkSpeedInfo, *tpSirLinkSpeedInfo;


/*
 * struct sir_rssi_req - rssi request struct
 * @peer_macaddr: MAC address
 * @sessionId: vdev id
 *
 * rssi request message's struct
 */
struct sir_rssi_req {
	v_MACADDR_t peer_macaddr;
	uint8_t sessionId;
};


/*
 * struct sir_rssi_info - rssi information struct
 * @peer_macaddr: MAC address
 * @rssi: rssi
 *
 * a station's rssi information
 */
struct sir_rssi_info {
	tSirMacAddr peer_macaddr;
	int8_t rssi;
};

/*
 * struct sir_rssi_info - all peers rssi information struct
 * @count: peer's number
 * @info: rssi information
 *
 * all station's rssi information
 */
struct sir_rssi_resp {
	uint8_t count;
	struct sir_rssi_info info[0];
};


typedef struct sSirAddPeriodicTxPtrn
{
   /* MAC Address for the adapter */
   tSirMacAddr macAddress;
   tANI_U8  ucPtrnId;           // Pattern ID
   tANI_U16 ucPtrnSize;         // Pattern size
   tANI_U32 usPtrnIntervalMs;   // In msec
   tANI_U8  ucPattern[PERIODIC_TX_PTRN_MAX_SIZE]; // Pattern buffer
} tSirAddPeriodicTxPtrn, *tpSirAddPeriodicTxPtrn;

typedef struct sSirDelPeriodicTxPtrn
{
   /* MAC Address for the adapter */
   tSirMacAddr macAddress;
   /* Bitmap of pattern IDs that need to be deleted */
   tANI_U32 ucPatternIdBitmap;
   tANI_U8  ucPtrnId;           // Pattern ID
} tSirDelPeriodicTxPtrn, *tpSirDelPeriodicTxPtrn;

/*---------------------------------------------------------------------------
* tSirIbssGetPeerInfoReqParams
*--------------------------------------------------------------------------*/
typedef struct
{
    tANI_BOOLEAN    allPeerInfoReqd; // If set, all IBSS peers stats are reported
    tANI_U8         staIdx;          // If allPeerInfoReqd is not set, only stats
                                     // of peer with staIdx is reported
}tSirIbssGetPeerInfoReqParams, *tpSirIbssGetPeerInfoReqParams;

/**
 * typedef struct - tSirIbssGetPeerInfoParams
 * @mac_addr: mac address received from target
 * @txRate: TX rate
 * @mcsIndex: MCS index
 * @txRateFlags: TX rate flags
 * @rssi: RSSI
 */
typedef struct {
   uint8_t  mac_addr[VOS_MAC_ADDR_SIZE];
   uint32_t txRate;
   uint32_t mcsIndex;
   uint32_t txRateFlags;
   int8_t  rssi;
}tSirIbssPeerInfoParams;

typedef struct
{
   tANI_U32   status;
   tANI_U8    numPeers;
   tSirIbssPeerInfoParams  peerInfoParams[32];
}tSirPeerInfoRspParams, *tpSirIbssPeerInfoRspParams;

/*---------------------------------------------------------------------------
* tSirIbssGetPeerInfoRspParams
*--------------------------------------------------------------------------*/
typedef struct
{
   tANI_U16   mesgType;
   tANI_U16   mesgLen;
   tSirPeerInfoRspParams ibssPeerInfoRspParams;
} tSirIbssGetPeerInfoRspParams, *tpSirIbssGetPeerInfoRspParams;

typedef struct
{
    tANI_U16      mesgType;
    tANI_U16      mesgLen;
    tANI_BOOLEAN  suspended;
}  tSirReadyToSuspendInd, *tpSirReadyToSuspendInd;
#ifdef WLAN_FEATURE_EXTWOW_SUPPORT
typedef struct
{
    tANI_U16      mesgType;
    tANI_U16      mesgLen;
    tANI_BOOLEAN  status;
}  tSirReadyToExtWoWInd, *tpSirReadyToExtWoWInd;
#endif
typedef struct sSirRateUpdateInd
{
    tANI_U8 nss; /* 0: 1x1, 1: 2x2 */
    tSirMacAddr bssid;
    tVOS_CON_MODE dev_mode;
    tANI_S32 bcastDataRate; /* bcast rate unit Mbpsx10, -1 : not used */
    /* 0 implies RA, positive value implies fixed rate, -1 implies ignore this
     * param.
     */
    tANI_S32 ucastDataRate;

    /* TX flag to differentiate between HT20, HT40 etc */
    tTxrateinfoflags ucastDataRateTxFlag;

    /*
     * 0 implies MCAST RA, positive value implies fixed rate,
     * -1 implies ignore this param
     */
    tANI_S32 reliableMcastDataRate;//unit Mbpsx10

    /* TX flag to differentiate between HT20, HT40 etc */
    tTxrateinfoflags reliableMcastDataRateTxFlag;

    /*
     * MCAST(or BCAST) fixed data rate in 2.4 GHz, unit Mbpsx10,
     * 0 implies ignore
     */
    tANI_U32 mcastDataRate24GHz;

    /* TX flag to differentiate between HT20, HT40 etc */
    tTxrateinfoflags mcastDataRate24GHzTxFlag;

    /*
     * MCAST(or BCAST) fixed data rate in 5 GHz,
     * unit Mbpsx10, 0 implies ignore
     */
    tANI_U32 mcastDataRate5GHz;

    /* TX flag to differentiate between HT20, HT40 etc */
    tTxrateinfoflags mcastDataRate5GHzTxFlag;

} tSirRateUpdateInd, *tpSirRateUpdateInd;

#if defined(FEATURE_WLAN_CH_AVOID) || defined(FEATURE_WLAN_FORCE_SAP_SCC)
#define SIR_CH_AVOID_MAX_RANGE   4

typedef struct sSirChAvoidFreqType
{
	tANI_U32 start_freq;
	tANI_U32 end_freq;
} tSirChAvoidFreqType;

typedef struct sSirChAvoidIndType
{
	tANI_U32	avoid_range_count;
	tSirChAvoidFreqType	avoid_freq_range[SIR_CH_AVOID_MAX_RANGE];
} tSirChAvoidIndType;
#endif /* FEATURE_WLAN_CH_AVOID || FEATURE_WLAN_FORCE_SAP_SCC */

#define SIR_DFS_MAX_20M_SUB_CH 8

typedef struct sSirSmeDfsChannelList
{
    tANI_U32    nchannels;
    /*Channel number including bonded channels on which the RADAR is present */
    tANI_U8     channels[SIR_DFS_MAX_20M_SUB_CH];
}tSirSmeDfsChannelList, *tpSirSmeDfsChannelList;

typedef struct sSirSmeDfsEventInd
{
    tANI_U32                   sessionId;
    tSirSmeDfsChannelList      chan_list;
    tANI_U32                   dfs_radar_status;
    int                        use_nol;
}tSirSmeDfsEventInd, *tpSirSmeDfsEventInd;

typedef struct sSirChanChangeRequest
{
    tANI_U16     messageType;
    tANI_U16     messageLen;
    tANI_U8      targetChannel;
    tANI_U8      cbMode;
    tANI_U8      vht_channel_width;
    tANI_U8      bssid[VOS_MAC_ADDR_SIZE];
    tANI_U32     dot11mode;
    tSirMacRateSet      operational_rateset;
    tSirMacRateSet      extended_rateset;
}tSirChanChangeRequest, *tpSirChanChangeRequest;

typedef struct sSirChanChangeResponse
{
    tANI_U8                  sessionId;
    tANI_U8                  newChannelNumber;
    tANI_U8                  channelChangeStatus;
    ePhyChanBondState        secondaryChannelOffset;
}tSirChanChangeResponse, *tpSirChanChangeResponse;

typedef struct sSirStartBeaconIndication
{
    tANI_U16     messageType;
    tANI_U16     messageLen;
    tANI_U8      beaconStartStatus;
    tANI_U8      bssid[VOS_MAC_ADDR_SIZE];
}tSirStartBeaconIndication, *tpSirStartBeaconIndication;

/* additional IE type */
typedef enum tUpdateIEsType
{
    eUPDATE_IE_NONE,
    eUPDATE_IE_PROBE_BCN,
    eUPDATE_IE_PROBE_RESP,
    eUPDATE_IE_ASSOC_RESP,

    /* Add type above this line*/
    /* this is used to reset all buffer */
    eUPDATE_IE_ALL,
    eUPDATE_IE_MAX
} eUpdateIEsType;


/* Modify particular IE in addition IE for probe resp Bcn */
typedef struct sSirModifyIE
{
   tSirMacAddr      bssid;
   tANI_U16         smeSessionId;
   boolean          notify;
   tANI_U8          ieID;
   tANI_U8          ieIDLen;   /*ie length as per spec*/
   tANI_U16         ieBufferlength;
   tANI_U8         *pIEBuffer;
   int32_t          oui_length;

}tSirModifyIE,    *tpSirModifyIE;


/* Message format for Update IE message sent to PE  */
typedef struct sSirModifyIEsInd
{
   tANI_U16         msgType;
   tANI_U16         msgLen;
   tSirModifyIE     modifyIE;
   eUpdateIEsType   updateType;
}tSirModifyIEsInd,    *tpSirModifyIEsInd;


/* Message format for Update IE message sent to PE  */
typedef struct sSirUpdateIE
{
   tSirMacAddr      bssid;
   tANI_U16         smeSessionId;
   boolean          append;
   boolean          notify;
   tANI_U16         ieBufferlength;
   tANI_U8         *pAdditionIEBuffer;
}tSirUpdateIE,  *tpSirUpdateIE;


/* Message format for Update IE message sent to PE  */
typedef struct sSirUpdateIEsInd
{
   tANI_U16         msgType;
   tANI_U16         msgLen;
   tSirUpdateIE     updateIE;
   eUpdateIEsType   updateType;
}tSirUpdateIEsInd,   *tpSirUpdateIEsInd;

/* Message format for requesting channel switch announcement to lower layers */
typedef struct sSirDfsCsaIeRequest
{
    tANI_U16 msgType;
    tANI_U16 msgLen;
    tANI_U8  targetChannel;
    tANI_U8  csaIeRequired;
    tANI_U8  bssid[VOS_MAC_ADDR_SIZE];
    u_int8_t  ch_bandwidth;
}tSirDfsCsaIeRequest, *tpSirDfsCsaIeRequest;

/* Indication from lower layer indicating the completion of first beacon send
 * after the beacon template update
 */
typedef struct sSirFirstBeaconTxCompleteInd
{
   tANI_U16 messageType; // eWNI_SME_MISSED_BEACON_IND
   tANI_U16 length;
   tANI_U8  bssIdx;
}tSirFirstBeaconTxCompleteInd, *tpSirFirstBeaconTxCompleteInd;

typedef struct sSirSmeCSAIeTxCompleteRsp
{
    tANI_U8  sessionId;
    tANI_U8  chanSwIeTxStatus;
}tSirSmeCSAIeTxCompleteRsp, *tpSirSmeCSAIeTxCompleteRsp;

/* Thermal Mitigation*/

typedef struct{
    u_int16_t minTempThreshold;
    u_int16_t maxTempThreshold;
} t_thermal_level_info, *tp_thermal_level_info;

typedef enum
{
    WLAN_WMA_THERMAL_LEVEL_0,
    WLAN_WMA_THERMAL_LEVEL_1,
    WLAN_WMA_THERMAL_LEVEL_2,
    WLAN_WMA_THERMAL_LEVEL_3,
    WLAN_WMA_MAX_THERMAL_LEVELS
} t_thermal_level;

typedef struct{
    /* Array of thermal levels */
    t_thermal_level_info thermalLevels[WLAN_WMA_MAX_THERMAL_LEVELS];
    u_int8_t thermalCurrLevel;
    u_int8_t thermalMgmtEnabled;
    u_int32_t throttlePeriod;
} t_thermal_mgmt, *tp_thermal_mgmt;

typedef struct sSirTxPowerLimit
{
    /* Thermal limits for 2g and 5g */
    u_int32_t txPower2g;
    u_int32_t txPower5g;
} tSirTxPowerLimit;


enum bad_peer_thresh_levels{
	WLAN_WMA_IEEE80211_B_LEVEL = 0,
	WLAN_WMA_IEEE80211_AG_LEVEL,
	WLAN_WMA_IEEE80211_N_LEVEL,
	WLAN_WMA_IEEE80211_AC_LEVEL,
	WLAN_WMA_IEEE80211_AX_LEVEL,
	WLAN_WMA_IEEE80211_MAX_LEVEL,
};

#define NUM_OF_RATE_THRESH_MAX    (4)
struct t_bad_peer_info{
	uint32_t cond;
	uint32_t delta;
	uint32_t percentage;
	uint32_t thresh[NUM_OF_RATE_THRESH_MAX];
	uint32_t txlimit;
};

struct t_bad_peer_txtcl_config{
	/* Array of thermal levels */
	struct t_bad_peer_info threshold[WLAN_WMA_IEEE80211_MAX_LEVEL];
	uint32_t enable;
	uint32_t period;
	uint32_t txq_limit;
	uint32_t tgt_backoff;
	uint32_t tgt_report_prd;
};


// Notify MODEM power state to FW
typedef struct
{
    tANI_U32 param;
} tSirModemPowerStateInd, *tpSirModemPowerStateInd;

#ifdef WLAN_FEATURE_STATS_EXT
typedef struct
{
    tANI_U32 vdev_id;
    tANI_U32 event_data_len;
    u_int8_t event_data[];
} tSirStatsExtEvent, *tpSirStatsExtEvent;
#endif

#ifdef WLAN_FEATURE_NAN
typedef struct
{
    tANI_U32 event_data_len;
    tANI_U8  event_data[];
} tSirNanEvent, *tpSirNanEvent;
#endif
#ifdef WLAN_FEATURE_ROAM_OFFLOAD
typedef struct sSirSmeRoamOffloadSynchInd
{
    tANI_U16    messageType; /*eWNI_SME_ROAM_OFFLOAD_SYNCH_IND*/
    tANI_U16    length;
    tANI_U16    beaconProbeRespOffset;
    tANI_U16    beaconProbeRespLength;
    tANI_U16    reassocRespOffset;
    tANI_U16    reassocRespLength;
    tANI_U8     isBeacon;
    tANI_U8     roamedVdevId;
    tSirMacAddr bssId;
    tANI_S8     txMgmtPower;
    tANI_U32    authStatus;
    tANI_U8     rssi;
    tANI_U8     roamReason;
    tANI_U32    chan_freq;
    tANI_U8     kck[SIR_KCK_KEY_LEN];
    tANI_U8     kek[SIR_KEK_KEY_LEN];
    tANI_U8     replay_ctr[SIR_REPLAY_CTR_LEN];
    tpSirBssDescription  pbssDescription;
} tSirRoamOffloadSynchInd, *tpSirRoamOffloadSynchInd;

typedef struct sSirSmeRoamOffloadSynchCnf
{
    tANI_U8 sessionId;
} tSirSmeRoamOffloadSynchCnf, *tpSirSmeRoamOffloadSynchCnf;

typedef struct sSirSmeHOFailureInd
{
    tANI_U8  sessionId;
} tSirSmeHOFailureInd, *tpSirSmeHOFailureInd;

typedef struct sSirRoamOffloadSynchFail
{
    tANI_U8 sessionId;
} tSirRoamOffloadSynchFail, *tpSirRoamOffloadSynchFail;
#endif

#ifdef FEATURE_WLAN_EXTSCAN

/**
 * typedef enum wifi_scan_flags - wifi scan flags
 * @WIFI_SCAN_FLAG_INTERRUPTED: Indicates that scan results are not complete
 *				because probes were not sent on some channels
 */
typedef enum {
	WIFI_SCAN_FLAG_INTERRUPTED = 1,
} wifi_scan_flags;

typedef enum
{
    WIFI_BAND_UNSPECIFIED,
    WIFI_BAND_BG             = 1,    /* 2.4 GHz */
    WIFI_BAND_A              = 2,    /* 5 GHz without DFS */
    WIFI_BAND_ABG            = 3,    /* 2.4 GHz + 5 GHz; no DFS */
    WIFI_BAND_A_DFS_ONLY     = 4,    /* 5 GHz DFS only */
    /* 5 is reserved */
    WIFI_BAND_A_WITH_DFS     = 6,    /* 5 GHz with DFS */
    WIFI_BAND_ABG_WITH_DFS   = 7,    /* 2.4 GHz + 5 GHz with DFS */

    /* Keep it last */
    WIFI_BAND_MAX
} tWifiBand;

/* wifi scan related events */
typedef enum
{
   WIFI_SCAN_BUFFER_FULL,
   WIFI_SCAN_COMPLETE,
} tWifiScanEventType;

/**
 * enum extscan_configuration_flags - extscan config flags
 * @EXTSCAN_LP_EXTENDED_BATCHING: extended batching
 */
enum extscan_configuration_flags {
	EXTSCAN_LP_EXTENDED_BATCHING = 0x00000001,
};

typedef struct
{
   tSirMacAddr    bssid;

   /* Low threshold */
   tANI_S32       low;

   /* High threshold */
   tANI_S32       high;

} tSirAPThresholdParam, *tpSirAPThresholdParam;

typedef struct
{
    tANI_U32    requestId;
    tANI_U8     sessionId;
} tSirGetExtScanCapabilitiesReqParams, *tpSirGetExtScanCapabilitiesReqParams;

/**
 * struct ext_scan_capabilities_response - extscan capabilities response data
 *					   from target
 * @requestId: request identifier
 * @status:    status
 * @max_scan_cache_size: total space allocated for scan (in bytes)
 * @max_scan_buckets: maximum number of channel buckets
 * @max_ap_cache_per_scan: maximum number of APs that can be stored per scan
 * @max_rssi_sample_size: number of RSSI samples used for averaging RSSI
 * @ax_scan_reporting_threshold: max possible report_threshold
 * @max_hotlist_bssids: maximum number of entries for hotlist APs
 * @max_significant_wifi_change_aps: maximum number of entries for
 *				significant wifi change APs
 * @max_bssid_history_entries: number of BSSID/RSSI entries that device can hold
 * @max_hotlist_ssids: maximum number of entries for hotlist SSIDs
 * @max_number_epno_networks: max number of epno entries
 * @max_number_epno_networks_by_ssid: max number of epno entries
 *			if ssid is specified, that is, epno entries for
 *			which an exact match is required,
 *			or entries corresponding to hidden ssids
 * @max_number_of_white_listed_ssid: max number of white listed SSIDs
 */
struct ext_scan_capabilities_response
{
	uint32_t    requestId;
	uint32_t    status;

	uint32_t    max_scan_cache_size;
	uint32_t    max_scan_buckets;
	uint32_t    max_ap_cache_per_scan;
	uint32_t    max_rssi_sample_size;
	uint32_t    max_scan_reporting_threshold;

	uint32_t    max_hotlist_bssids;
	uint32_t    max_significant_wifi_change_aps;

	uint32_t    max_bssid_history_entries;
	uint32_t    max_hotlist_ssids;
	uint32_t    max_number_epno_networks;
	uint32_t    max_number_epno_networks_by_ssid;
	uint32_t    max_number_of_white_listed_ssid;
};


typedef struct
{
    tANI_U32      requestId;
    tANI_U8       sessionId;

    /*
     * 1 - return cached results and flush it
     * 0 - return cached results and do not flush
     */
    tANI_BOOLEAN  flush;
} tSirExtScanGetCachedResultsReqParams, *tpSirExtScanGetCachedResultsReqParams;

typedef struct
{
    tANI_U32    requestId;
    tANI_U32    status;
} tSirExtScanGetCachedResultsRspParams, *tpSirExtScanGetCachedResultsRspParams;

typedef struct
{
    /* Time of discovery (in micro second) */
    tANI_U64      ts;

    /* Null terminated SSID */
    tANI_U8       ssid[SIR_MAC_MAX_SSID_LENGTH+1];

    tSirMacAddr   bssid;

    /* Frequency in MHz */
    tANI_U32      channel;

    /* RSSI in dBm */
    tANI_S32      rssi;

    /* RTT in nanoseconds */
    tANI_U32      rtt;

    /* Standard deviation in rtt */
    tANI_U32      rtt_sd;

    /* Period advertised in the beacon */
    tANI_U16      beaconPeriod;

    /* Capabilities advertised in the beacon */
    tANI_U16      capability;

    tANI_U16      ieLength;

    tANI_U8       ieData[];
} tSirWifiScanResult, *tpSirWifiScanResult;

/**
 * struct extscan_hotlist_match - extscan hotlist match
 * @requestId: request identifier
 * @numOfAps: number of bssids retrieved by the scan
 * @moreData: 0 - for last fragment
 *	      1 - still more fragment(s) coming
 * @ap: wifi scan result
 */
struct extscan_hotlist_match
{
	uint32_t    requestId;
	bool        moreData;
	bool        ap_found;
	uint32_t    numOfAps;
	tSirWifiScanResult   ap[];
};

/**
 * struct extscan_cached_scan_result - extscan cached scan result
 * @scan_id: a unique identifier for the scan unit
 * @flags: a bitmask with additional information about scan
 * @num_results: number of bssids retrieved by the scan
 * @ap: wifi scan bssid results info
 */
struct extscan_cached_scan_result
{
	uint32_t    scan_id;
	uint32_t    flags;
	uint32_t    num_results;
	tSirWifiScanResult *ap;
};

/**
 * struct tSirWifiScanResultEvent - wifi scan result event
 * @requestId: request identifier
 * @ap_found: flag to indicate ap found or not
 *		true: AP was found
 *		false: AP was lost
 * @numOfAps: number of aps
 * @moreData: more data
 * @ap: bssid information
 *
 */
typedef struct
{
	uint32_t     requestId;
	bool         ap_found;
	uint32_t     numOfAps;
	bool         moreData;
	tSirWifiScanResult   ap[];
} tSirWifiScanResultEvent, *tpSirWifiScanResultEvent;

/**
 * struct extscan_cached_scan_results - extscan cached scan results
 * @request_id: request identifier
 * @more_data: 0 - for last fragment
 *	       1 - still more fragment(s) coming
 * @num_scan_ids: number of scan ids
 * @result: wifi scan result
 */
struct extscan_cached_scan_results
{
	uint32_t    request_id;
	bool        more_data;
	uint32_t    num_scan_ids;
	struct extscan_cached_scan_result  *result;
};


/**
 * struct tSirWifiFullScanResultEvent - extscan full scan event
 * @request_id: request identifier
 * @moreData: 0 - for last fragment
 *             1 - still more fragment(s) coming
 * @ap: bssid info
 *
 * Reported when each probe response is received, if reportEvents
 * enabled in tSirWifiScanCmdReqParams
 */
typedef struct
{
	uint32_t            requestId;
	bool                moreData;
	tSirWifiScanResult  ap;
} tSirWifiFullScanResultEvent, *tpSirWifiFullScanResultEvent;

/**
 * struct pno_match_found - epno match found
 * @request_id: request identifier
 * @moreData: 0 - for last fragment
     * 1 - still more fragment(s) coming
 * @num_results: number of bssids, driver sends this event to upper layer
 *		 for every beacon, hence %num_results is always set to 1.
 * @ap: bssid info
 *
 * Reported when each beacon probe response is received with
 * epno match found tag.
     */
struct pno_match_found
{
	uint32_t            request_id;
	bool                more_data;
	uint32_t            num_results;
	tSirWifiScanResult  ap[];
};


typedef struct
{
    /* Frequency in MHz*/
    tANI_U32      channel;

    tANI_U32      dwellTimeMs;

    /* 0 => active
       1 => passive scan; ignored for DFS */
    tANI_BOOLEAN  passive;

    tANI_U8       chnlClass;
} tSirWifiScanChannelSpec, *tpSirWifiScanChannelSpec;

/**
 * struct tSirWifiScanBucketSpec - wifi scan bucket spec
 * @bucket: bucket identifier
 * @band: wifi band
 * @period: Desired period, in millisecond; if this is too
 *		low, the firmware should choose to generate results as fast as
 *		it can instead of failing the command byte
 *		for exponential backoff bucket this is the min_period
 * @reportEvents: 0 => normal reporting (reporting rssi history
 *		only, when rssi history buffer is % full)
 *		1 => same as 0 + report a scan completion event after scanning
 *		this bucket
 *		2 => same as 1 + forward scan results
 *		(beacons/probe responses + IEs) in real time to HAL
 * @max_period: if max_period is non zero or different than period,
 *		then this bucket is an exponential backoff bucket and
 *		the scan period will grow exponentially as per formula:
 *		actual_period(N) = period ^ (N/(step_count+1)) to a
 *		maximum period of max_period
 * @exponent: for exponential back off bucket: multiplier:
 *		new_period = old_period * exponent
 * @step_count: for exponential back off bucket, number of scans performed
 *		at a given period and until the exponent is applied
 * @numChannels: channels to scan; these may include DFS channels
 *		Note that a given channel may appear in multiple buckets
 * @min_dwell_time_active: per bucket minimum active dwell time
 * @max_dwell_time_active: per bucket maximum active dwell time
 * @min_dwell_time_passive: per bucket minimum passive dwell time
 * @max_dwell_time_passive: per bucket maximum passive dwell time
 * @channels: Channel list
 */
typedef struct
{
	uint8_t         bucket;
	tWifiBand       band;
	uint32_t        period;
	uint32_t        reportEvents;
	uint32_t        max_period;
	uint32_t        exponent;
	uint32_t        step_count;
	uint32_t        numChannels;

	uint32_t        min_dwell_time_active;
	uint32_t        max_dwell_time_active;
	uint32_t        min_dwell_time_passive;
	uint32_t        max_dwell_time_passive;
	tSirWifiScanChannelSpec channels[WLAN_EXTSCAN_MAX_CHANNELS];
} tSirWifiScanBucketSpec, *tpSirWifiScanBucketSpec;

/**
 * struct tSirWifiScanCmdReqParams - extscan request parameters
 * @basePeriod: base period
 * @maxAPperScan: Max number of APs to report per scan
 * @report_threshold_percent: threshold at which framework should be informed
 * @report_threshold_num_scans: number of scans after which wake up the host
 * @requestId: Request id
 * @sessionId: Session id
 * @numBuckets: number of buckets to scan
 * @min_dwell_time_active: per bucket minimum active dwell time
 * @max_dwell_time_active: per bucket maximum active dwell time
 * @min_dwell_time_passive: per bucket minimum passive dwell time
 * @max_dwell_time_passive: per bucket maximum passive dwell time
 * @configuration_flags: configuration flags
 * @buckets: bucket list
 */
typedef struct
{
	tANI_U32                basePeriod;
	tANI_U32                maxAPperScan;
	uint32_t                report_threshold_percent;
	uint32_t                report_threshold_num_scans;

	tANI_U32                requestId;
	tANI_U8                 sessionId;
	tANI_U32                numBuckets;

	uint32_t                min_dwell_time_active;
	uint32_t                max_dwell_time_active;
	uint32_t                min_dwell_time_passive;
	uint32_t                max_dwell_time_passive;
	uint32_t                configuration_flags;
	tSirWifiScanBucketSpec  buckets[WLAN_EXTSCAN_MAX_BUCKETS];
} tSirWifiScanCmdReqParams, *tpSirWifiScanCmdReqParams;

/**
 * struct sir_extscan_generic_response -
 *	Generic ExtScan Response structure
 * @request_id: ID of the request
 * @status: operation status returned by firmware
 */
struct sir_extscan_generic_response {
	uint32_t request_id;
	int32_t status;
};

typedef struct
{
    tANI_U32    requestId;
    tANI_U8     sessionId;
} tSirExtScanStopReqParams, *tpSirExtScanStopReqParams;

/**
 * struct tSirExtScanSetBssidHotListReqParams - set hotlist request
 * @requestId: request identifier
 * @sessionId: session identifier
 * @lost_ap_sample_size: number of samples to confirm AP loss
 * @numAp: Number of hotlist APs
 * @ap: hotlist APs
 */
typedef struct
{
	uint32_t    requestId;
	uint8_t     sessionId;
	uint32_t    lost_ap_sample_size;
	uint32_t    numAp;
	tSirAPThresholdParam   ap[WLAN_EXTSCAN_MAX_HOTLIST_APS];
} tSirExtScanSetBssidHotListReqParams, *tpSirExtScanSetBssidHotListReqParams;

typedef struct
{
    tANI_U32    requestId;
    tANI_U8     sessionId;
} tSirExtScanResetBssidHotlistReqParams,
  *tpSirExtScanResetBssidHotlistReqParams;

/**
 * struct sir_ssid_hotlist_param - param for SSID Hotlist
 * @ssid: SSID which is being hotlisted
 * @band: Band in which the given SSID should be scanned
 * @rssi_low: Low bound on RSSI
 * @rssi_high: High bound on RSSI
 */
struct sir_ssid_hotlist_param {
	tSirMacSSid ssid;
	uint8_t band;
	int32_t rssi_low;
	int32_t rssi_high;
};

/**
 * struct sir_set_ssid_hotlist_request - set SSID hotlist request struct
 * @request_id: ID of the request
 * @session_id: ID of the session
 * @lost_ssid_sample_size: Number of consecutive scans in which the SSID
 *	must not be seen in order to consider the SSID "lost"
 * @ssid_count: Number of valid entries in the @ssids array
 * @ssids: Array that defines the SSIDs that are in the hotlist
 */
struct sir_set_ssid_hotlist_request {
	uint32_t request_id;
	uint8_t session_id;
	uint32_t lost_ssid_sample_size;
	uint32_t ssid_count;
	struct sir_ssid_hotlist_param ssids[WLAN_EXTSCAN_MAX_HOTLIST_SSIDS];
};

typedef struct
{
    tANI_U32              requestId;
    tANI_U8               sessionId;

    /* Number of samples for averaging RSSI */
    tANI_U32              rssiSampleSize;

    /* Number of missed samples to confirm AP loss */
    tANI_U32              lostApSampleSize;

    /* Number of APs breaching threshold required for firmware
     * to generate event
     */
    tANI_U32              minBreaching;

    tANI_U32              numAp;
    tSirAPThresholdParam  ap[WLAN_EXTSCAN_MAX_SIGNIFICANT_CHANGE_APS];
} tSirExtScanSetSigChangeReqParams,
 *tpSirExtScanSetSigChangeReqParams;

typedef struct
{
    tSirMacAddr  bssid;
    tANI_U32     channel;
    tANI_U32     numOfRssi;

    /* Rssi history in db */
    tANI_S32     rssi[];
} tSirWifiSignificantChange, *tpSirWifiSignificantChange;

typedef struct
{
    tANI_U32                      requestId;

    tANI_BOOLEAN                  moreData;
    tANI_U32                      numResults;
    tSirWifiSignificantChange     ap[];
} tSirWifiSignificantChangeEvent, *tpSirWifiSignificantChangeEvent;

typedef struct
{
    tANI_U32    requestId;
    tANI_U8     sessionId;
} tSirExtScanResetSignificantChangeReqParams,
  *tpSirExtScanResetSignificantChangeReqParams;

typedef struct
{
    tANI_U32    requestId;
    tANI_U32    numResultsAvailable;
} tSirExtScanResultsAvailableIndParams,
  *tpSirExtScanResultsAvailableIndParams;

typedef struct
{
    tANI_U32   requestId;
    tANI_U32   status;
    tANI_U8    scanEventType;
} tSirExtScanOnScanEventIndParams,
  *tpSirExtScanOnScanEventIndParams;

/**
 * struct wifi_epno_network - enhanced pno network block
 * @ssid: ssid
 * @rssi_threshold: threshold for considering this SSID as found, required
 *		    granularity for this threshold is 4dBm to 8dBm
 * @flags: WIFI_PNO_FLAG_XXX
 * @auth_bit_field: auth bit field for matching WPA IE
 */
struct wifi_epno_network
{
	tSirMacSSid  ssid;
	int8_t       rssi_threshold;
	uint8_t      flags;
	uint8_t      auth_bit_field;
};

/**
 * struct wifi_epno_params - enhanced pno network params
 * @num_networks: number of ssids
 * @networks: PNO networks
 */
struct wifi_epno_params
{
	uint32_t    request_id;
	uint32_t    session_id;
	uint32_t    num_networks;
	struct wifi_epno_network networks[];
};

#define SIR_PASSPOINT_REALM_LEN 256
#define SIR_PASSPOINT_ROAMING_CONSORTIUM_ID_NUM 16
#define SIR_PASSPOINT_PLMN_LEN 3
/**
 * struct wifi_passpoint_network - passpoint network block
 * @id: identifier of this network block
 * @realm: null terminated UTF8 encoded realm, 0 if unspecified
 * @roaming_consortium_ids: roaming consortium ids to match, 0s if unspecified
 * @plmn: mcc/mnc combination as per rules, 0s if unspecified
 */
struct wifi_passpoint_network
{
	uint32_t id;
	uint8_t  realm[SIR_PASSPOINT_REALM_LEN];
	int64_t  roaming_consortium_ids[SIR_PASSPOINT_ROAMING_CONSORTIUM_ID_NUM];
	uint8_t  plmn[SIR_PASSPOINT_PLMN_LEN];
};

/**
 * struct wifi_passpoint_req - passpoint request
 * @request_id: request identifier
 * @num_networks: number of networks
 * @networks: passpoint networks
 */
struct wifi_passpoint_req
{
	uint32_t request_id;
	uint32_t session_id;
	uint32_t num_networks;
	struct wifi_passpoint_network networks[];
};

/**
 * struct wifi_passpoint_match - wifi passpoint network match
 * @id: network block identifier for the matched network
 * @anqp_len: length of ANQP blob
 * @ap: scan result, with channel and beacon information
 * @anqp: ANQP data, in the information_element format
 */
struct wifi_passpoint_match
{
	uint32_t  request_id;
	uint32_t  id;
	uint32_t  anqp_len;
	tSirWifiScanResult ap;
	uint8_t   anqp[];
};

#endif /* FEATURE_WLAN_EXTSCAN */

#ifdef FEATURE_WLAN_AUTO_SHUTDOWN
typedef struct
{
    tANI_U32    timer_val;
} tSirAutoShutdownCmdParams;

typedef struct
{
    tANI_U32    shutdown_reason;
} tSirAutoShutdownEvtParams;
#endif

#ifdef WLAN_FEATURE_LINK_LAYER_STATS

typedef struct
{
  tANI_U32 reqId;
  tANI_U8  staId;
  tANI_U32 mpduSizeThreshold;
  tANI_U32 aggressiveStatisticsGathering;
} tSirLLStatsSetReq, *tpSirLLStatsSetReq;

typedef struct
{
  tANI_U32 reqId;
  tANI_U8  staId;
  tANI_U32 paramIdMask;
} tSirLLStatsGetReq, *tpSirLLStatsGetReq;

typedef struct
{
  tANI_U32  reqId;
  tANI_U8   staId;
  tANI_U32  statsClearReqMask;
  tANI_U8   stopReq;
} tSirLLStatsClearReq, *tpSirLLStatsClearReq;

typedef struct
{
    tANI_U8 oui[WIFI_SCANNING_MAC_OUI_LENGTH];
} tSirScanMacOui, *tpSirScanMacOui;

enum {
    SIR_AP_RX_DATA_OFFLOAD             = 0x00,
    SIR_STA_RX_DATA_OFFLOAD            = 0x01,
};

struct sir_ipa_offload_enable_disable
{
    uint32_t offload_type;
    uint32_t vdev_id;
    uint32_t enable;
};

/**
 * struct sir_set_ht_vht_cfg - ht, vht IE config
 *
 * @msg_type: message type
 * @len: message length
 * @pdev_id: pdev id
 * @nss: Nss value
 * @dot11mode: Dot11 mode.
 *
 * Message wrapper structure for set HT/VHT IE req.
 */
struct sir_set_ht_vht_cfg {
    uint16_t msg_type;
    uint16_t len;
    uint32_t pdev_id;
    uint32_t nss;
    uint32_t dot11mode;
};

/*---------------------------------------------------------------------------
  WLAN_HAL_LL_NOTIFY_STATS
---------------------------------------------------------------------------*/


/******************************LINK LAYER Statistics**********************/

typedef int tSirWifiRadio;
typedef int tSirWifiChannel;
typedef int tSirwifiTxRate;

/* channel operating width */
typedef enum
{
    WIFI_CHAN_WIDTH_20    = 0,
    WIFI_CHAN_WIDTH_40    = 1,
    WIFI_CHAN_WIDTH_80    = 2,
    WIFI_CHAN_WIDTH_160   = 3,
    WIFI_CHAN_WIDTH_80P80 = 4,
    WIFI_CHAN_WIDTH_5     = 5,
    WIFI_CHAN_WIDTH_10    = 6,
} tSirWifiChannelWidth;

typedef enum
{
    WIFI_DISCONNECTED     = 0,
    WIFI_AUTHENTICATING   = 1,
    WIFI_ASSOCIATING      = 2,
    WIFI_ASSOCIATED       = 3,
    WIFI_EAPOL_STARTED    = 4, /* if done by firmware/driver */
    WIFI_EAPOL_COMPLETED  = 5, /* if done by firmware/driver */
} tSirWifiConnectionState;

typedef enum
{
    WIFI_ROAMING_IDLE     = 0,
    WIFI_ROAMING_ACTIVE   = 1,
} tSirWifiRoamState;

typedef enum
{
    WIFI_INTERFACE_STA        = 0,
    WIFI_INTERFACE_SOFTAP     = 1,
    WIFI_INTERFACE_IBSS       = 2,
    WIFI_INTERFACE_P2P_CLIENT = 3,
    WIFI_INTERFACE_P2P_GO     = 4,
    WIFI_INTERFACE_NAN        = 5,
    WIFI_INTERFACE_MESH       = 6,
 } tSirWifiInterfaceMode;

/* set for QOS association */
#define WIFI_CAPABILITY_QOS          0x00000001
/* set for protected association (802.11 beacon frame control protected bit set) */
#define WIFI_CAPABILITY_PROTECTED    0x00000002
/* set if 802.11 Extended Capabilities element interworking bit is set */
#define WIFI_CAPABILITY_INTERWORKING 0x00000004
/* set for HS20 association */
#define WIFI_CAPABILITY_HS20         0x00000008
/* set is 802.11 Extended Capabilities element UTF-8 SSID bit is set */
#define WIFI_CAPABILITY_SSID_UTF8    0x00000010
/* set is 802.11 Country Element is present */
#define WIFI_CAPABILITY_COUNTRY      0x00000020

typedef struct
{
    /* tSirWifiInterfaceMode */
    /* interface mode */
    tANI_U8                  mode;
    /* interface mac address (self) */
    tSirMacAddr              macAddr;
    /* tSirWifiConnectionState */
    /* connection state (valid for STA, CLI only) */
    tANI_U8                  state;
    /* tSirWifiRoamState */
    /* roaming state */
    tANI_U32                  roaming;
    /* WIFI_CAPABILITY_XXX (self) */
    tANI_U32                 capabilities;
    /* null terminated SSID */
    tANI_U8                  ssid[33];
    /* bssid */
    tSirMacAddr              bssid;
    /* country string advertised by AP */
    tANI_U8                  apCountryStr[WNI_CFG_COUNTRY_CODE_LEN];
    /* country string for this association */
    tANI_U8                  countryStr[WNI_CFG_COUNTRY_CODE_LEN];
} tSirWifiInterfaceInfo, *tpSirWifiInterfaceInfo;

/* channel information */
typedef struct
{
    /* channel width (20, 40, 80, 80+80, 160) */
    tSirWifiChannelWidth      width;
    /* primary 20 MHz channel */
    tSirWifiChannel           centerFreq;
    /* center frequency (MHz) first segment */
    tSirWifiChannel           centerFreq0;
    /* center frequency (MHz) second segment */
    tSirWifiChannel           centerFreq1;
} tSirWifiChannelInfo, *tpSirWifiChannelInfo;

/* wifi rate info */
typedef struct
{
    /* 0: OFDM, 1:CCK, 2:HT 3:VHT 4..7 reserved */
    tANI_U32 preamble   :3;
    /* 0:1x1, 1:2x2, 3:3x3, 4:4x4 */
    tANI_U32 nss        :2;
    /* 0:20MHz, 1:40Mhz, 2:80Mhz, 3:160Mhz */
    tANI_U32 bw         :3;
    /* OFDM/CCK rate code would be as per ieee std in the units of 0.5mbps */
    /* HT/VHT it would be mcs index */
    tANI_U32 rateMcsIdx :8;
    /* reserved */
    tANI_U32 reserved  :16;
    /* units of 100 Kbps */
    tANI_U32 bitrate;
} tSirWifiRate, *tpSirWifiRate;

/* channel statistics */
typedef struct
{
    /* channel */
    tSirWifiChannelInfo channel;
    /* msecs the radio is awake (32 bits number accruing over time) */
    tANI_U32          onTime;
    /* msecs the CCA register is busy (32 bits number accruing over time) */
    tANI_U32          ccaBusyTime;
} tSirWifiChannelStats, *tpSirWifiChannelStats;

/* radio statistics */
typedef struct
{
    /* wifi radio (if multiple radio supported) */
    tSirWifiRadio   radio;
    /* msecs the radio is awake (32 bits number accruing over time) */
    tANI_U32        onTime;
    /* msecs the radio is transmitting
     * (32 bits number accruing over time)
     */
    tANI_U32        txTime;
    /* msecs the radio is in active receive
     *(32 bits number accruing over time)
     */
    tANI_U32        rxTime;
    /* msecs the radio is awake due to all scan
     * (32 bits number accruing over time)
     */
    tANI_U32        onTimeScan;
    /* msecs the radio is awake due to NAN
     * (32 bits number accruing over time)
     */
    tANI_U32        onTimeNbd;
    /* msecs the radio is awake due to Gscan
     * (32 bits number accruing over time)
     */
    tANI_U32        onTimeGscan;
    /* msecs the radio is awake due to roam?scan
     * (32 bits number accruing over time)
     */
    tANI_U32        onTimeRoamScan;
    /* msecs the radio is awake due to PNO scan
     * (32 bits number accruing over time)
     */
    tANI_U32        onTimePnoScan;
    /* msecs the radio is awake due to HS2.0 scans and GAS exchange
     * (32 bits number accruing over time)
     */
    tANI_U32        onTimeHs20;
    /* number of channels */
    tANI_U32        numChannels;
    /* channel statistics tSirWifiChannelStats */
    tSirWifiChannelStats channels[0];
} tSirWifiRadioStat, *tpSirWifiRadioStat;

/* per rate statistics */
typedef struct
{
    /* rate information */
    tSirWifiRate rate;
    /* number of successfully transmitted data pkts (ACK rcvd) */
    tANI_U32 txMpdu;
    /* number of received data pkts */
    tANI_U32 rxMpdu;
    /* number of data packet losses (no ACK) */
    tANI_U32 mpduLost;
    /* total number of data pkt retries * */
    tANI_U32 retries;
    /* number of short data pkt retries */
    tANI_U32 retriesShort;
    /* number of long data pkt retries */
    tANI_U32 retriesLong;
} tSirWifiRateStat, *tpSirWifiRateStat;

/* access categories */
typedef enum
{
    WIFI_AC_VO  = 0,
    WIFI_AC_VI  = 1,
    WIFI_AC_BE  = 2,
    WIFI_AC_BK  = 3,
    WIFI_AC_MAX = 4,
} tSirWifiTrafficAc;

/* wifi peer type */
typedef enum
{
    WIFI_PEER_STA,
    WIFI_PEER_AP,
    WIFI_PEER_P2P_GO,
    WIFI_PEER_P2P_CLIENT,
    WIFI_PEER_NAN,
    WIFI_PEER_TDLS,
    WIFI_PEER_INVALID,
} tSirWifiPeerType;

/* per peer statistics */
typedef struct
{
    /* peer type (AP, TDLS, GO etc.) */
    tSirWifiPeerType type;
    /* mac address */
    tSirMacAddr    peerMacAddress;
    /* peer WIFI_CAPABILITY_XXX */
    tANI_U32       capabilities;
    /* number of rates */
    tANI_U32       numRate;
    /* per rate statistics, number of entries  = num_rate */
    tSirWifiRateStat rateStats[0];
} tSirWifiPeerInfo, *tpSirWifiPeerInfo;

/* per access category statistics */
typedef struct
{
    /* tSirWifiTrafficAc */
    /* access category (VI, VO, BE, BK) */
    tANI_U32 ac;
    /* number of successfully transmitted unicast data pkts (ACK rcvd) */
    tANI_U32 txMpdu;
    /* number of received unicast mpdus */
    tANI_U32 rxMpdu;
    /* Number of successfully transmitted multicast data packets */
    /* STA case: implies ACK received from AP for the unicast */
    /* packet in which mcast pkt was sent */
    tANI_U32 txMcast;
    /* number of received multicast data packets */
    tANI_U32 rxMcast;
    /* number of received unicast a-mpdus */
    tANI_U32 rxAmpdu;
    /* number of transmitted unicast a-mpdus */
    tANI_U32 txAmpdu;
    /* number of data pkt losses (no ACK) */
    tANI_U32 mpduLost;
    /* total number of data pkt retries */
    tANI_U32 retries;
    /* number of short data pkt retries */
    tANI_U32 retriesShort;
    /* number of long data pkt retries */
    tANI_U32 retriesLong;
    /* data pkt min contention time (usecs) */
    tANI_U32 contentionTimeMin;
    /* data pkt max contention time (usecs) */
    tANI_U32 contentionTimeMax;
    /* data pkt avg contention time (usecs) */
    tANI_U32 contentionTimeAvg;
    /* num of data pkts used for contention statistics */
    tANI_U32 contentionNumSamples;
} tSirWifiWmmAcStat, *tpSirWifiWmmAcStat;

/* Interface statistics - corresponding to 2nd most
 * LSB in wifi statistics bitmap  for getting statistics
 */
typedef struct
{
    /* current state of the interface */
    tSirWifiInterfaceInfo info;
    /* access point beacon received count from connected AP */
    tANI_U32            beaconRx;
    /* access point mgmt frames received count from */
    /* connected AP (including Beacon) */
    tANI_U32            mgmtRx;
    /* action frames received count */
    tANI_U32            mgmtActionRx;
    /* action frames transmit count */
    tANI_U32            mgmtActionTx;
    /* access Point Beacon and Management frames RSSI (averaged) */
    tANI_U32            rssiMgmt;
    /* access Point Data Frames RSSI (averaged) from connected AP */
    tANI_U32            rssiData;
    /* access Point ACK RSSI (averaged) from connected AP */
    tANI_U32            rssiAck;
    /** number of peers */
    tANI_U32 num_peers;
    /** Indicates how many peer_stats events will be sent depending on the num_peers. */
    tANI_U32 num_peer_events;
    /** number of ac */
    tANI_U32 num_ac;
    /** Roaming Stat */
    tANI_U32 roam_state;
    /** Average Beacon spread offset is the averaged time delay between TBTT and beacon TSF */
    /** Upper 32 bits of averaged 64 bit beacon spread offset */
    tANI_U32 avg_bcn_spread_offset_high;
    /** Lower 32 bits of averaged 64 bit beacon spread offset */
    tANI_U32 avg_bcn_spread_offset_low;
    /** Takes value of 1 if AP leaks packets after sending an ACK for PM=1 otherwise 0 */
    tANI_U32 is_leaky_ap;
    /** Average number of frames received from AP after receiving the ACK for a frame with PM=1 */
    tANI_U32 avg_rx_frms_leaked;
    /** Rx leak watch window currently in force to minimize data loss because of leaky AP. Rx leak window is the
        time driver waits before shutting down the radio or switching the channel and after receiving an ACK for
        a data frame with PM bit set) */
    tANI_U32 rx_leak_window;
    /* per ac data packet statistics */
    tSirWifiWmmAcStat    AccessclassStats[WIFI_AC_MAX];
} tSirWifiIfaceStat, *tpSirWifiIfaceStat;

/* Peer statistics - corresponding to 3rd most LSB in
 * wifi statistics bitmap  for getting statistics
 */
typedef struct
{
    /* number of peers */
    tANI_U32       numPeers;
    /* per peer statistics */
    tSirWifiPeerInfo peerInfo[0];
} tSirWifiPeerStat, *tpSirWifiPeerStat;

/* wifi statistics bitmap  for getting statistics */
#define WMI_LINK_STATS_RADIO          0x00000001
#define WMI_LINK_STATS_IFACE          0x00000002
#define WMI_LINK_STATS_ALL_PEER       0x00000004
#define WMI_LINK_STATS_PER_PEER       0x00000008

/* wifi statistics bitmap  for clearing statistics */
/* all radio statistics */
#define WIFI_STATS_RADIO              0x00000001
/* cca_busy_time (within radio statistics) */
#define WIFI_STATS_RADIO_CCA          0x00000002
/* all channel statistics (within radio statistics) */
#define WIFI_STATS_RADIO_CHANNELS     0x00000004
/* all scan statistics (within radio statistics) */
#define WIFI_STATS_RADIO_SCAN         0x00000008
/* all interface statistics */
#define WIFI_STATS_IFACE              0x00000010
/* all tx rate statistics (within interface statistics) */
#define WIFI_STATS_IFACE_TXRATE       0x00000020
/* all ac statistics (within interface statistics) */
#define WIFI_STATS_IFACE_AC           0x00000040
/* all contention (min, max, avg) statistics (within ac statistics) */
#define WIFI_STATS_IFACE_CONTENTION   0x00000080

typedef struct
{
    tANI_U32 paramId;
    tANI_U8 ifaceId;
    tANI_U32 rspId;
    tANI_U32 moreResultToFollow;
    union
    {
        tANI_U32 num_peers;
        tANI_U32 num_radio;
    };

    tANI_U32 peer_event_number ;
    /* Variable  length field - Do not add anything after this */
    tANI_U8 results[0];
} tSirLLStatsResults, *tpSirLLStatsResults;

#endif /* WLAN_FEATURE_LINK_LAYER_STATS */

typedef struct sAniGetLinkStatus
{
    tANI_U16 msgType;      /* message type is same as the request type */
    tANI_U16 msgLen;       /* length of the entire request */
    tANI_U8 linkStatus;
    tANI_U8 sessionId;
} tAniGetLinkStatus, *tpAniGetLinkStatus;

#ifdef DHCP_SERVER_OFFLOAD
typedef struct
{
    tANI_U32 vdev_id;
    tANI_U32 dhcpSrvOffloadEnabled;
    tANI_U32 dhcpClientNum;
    tANI_U32 dhcpSrvIP;
    tANI_U32 dhcp_client_start_ip;
} tSirDhcpSrvOffloadInfo, *tpSirDhcpSrvOffloadInfo;
#endif /* DHCP_SERVER_OFFLOAD */

#ifdef WLAN_FEATURE_GPIO_LED_FLASHING
typedef struct
{
  tANI_U32 reqId;
  tANI_U32 pattern_id; /* pattern identifier. 0: disconnected 1: connected*/
  tANI_U32 led_x0; /* led flashing parameter0 */
  tANI_U32 led_x1; /* led flashing parameter1 */
  tANI_U32 gpio_num; /* GPIO number */
} tSirLedFlashingReq, *tpSirLedFlashingReq;
#endif

#ifdef MDNS_OFFLOAD
#define MAX_MDNS_FQDN_LEN                         64
#define MAX_MDNS_RESP_LEN                         512

typedef struct
{
    tANI_U32 vdev_id;
    tANI_U32 mDNSOffloadEnabled;
} tSirMDNSOffloadInfo, *tpSirMDNSOffloadInfo;

typedef struct
{
    tANI_U32 vdev_id;
    tANI_U32 fqdn_type;
    tANI_U32 fqdn_len;
    tANI_U8 fqdn_data[MAX_MDNS_FQDN_LEN];
} tSirMDNSFqdnInfo, *tpSirMDNSFqdnInfo;

typedef struct
{
    tANI_U32 vdev_id;
    tANI_U32 resourceRecord_count;
    tANI_U32 resp_len;
    tANI_U8 resp_data[MAX_MDNS_RESP_LEN];
} tSirMDNSResponseInfo, *tpSirMDNSResponseInfo;
#endif /* MDNS_OFFLOAD */

/**
 * struct sir_lost_link_info - lost link information structure.
 *
 * @vdev_id: vdev_id from WMA. some modules call sessionId.
 * @rssi: rssi at disconnection time.
 *
 * driver uses this structure to communicate information collected at
 * disconnection time.
 */
struct sir_lost_link_info
{
	uint32_t vdev_id;
	int8_t rssi;
};

/* find the size of given member within a structure */
#ifndef member_size
#define member_size(type, member) (sizeof(((type *)0)->member))
#endif

#define RTT_INVALID                     0x00
#define RTT_TIMING_MEAS_CAPABILITY      0x01
#define RTT_FINE_TIME_MEAS_INITIATOR_CAPABILITY    0x02
#define RTT_FINE_TIME_MEAS_RESPONDER_CAPABILITY    0x03

/**
 * enum fine_time_meas_mask - bit mask to identify device's
 *                            fine timing measurement capability
 * @FINE_TIME_MEAS_STA_INITIATOR - STA role, Initiator capability is supported
 * @FINE_TIME_MEAS_STA_RESPONDER - STA role, Responder capability is supported
 * @FINE_TIME_MEAS_P2PCLI_INITIATOR - P2P-CLI supports initiator capability
 * @FINE_TIME_MEAS_P2PCLI_RESPONDER - P2P-CLI supports responder capability
 * @FINE_TIME_MEAS_P2PGO_INITIATOR - P2P-GO supports initiator capability
 * @FINE_TIME_MEAS_P2PGO_RESPONDER - P2P-GO supports responder capability
 * @FINE_TIME_MEAS_SAP_INITIATOR - SAP role, Initiator capability is supported
 * @FINE_TIME_MEAS_SAP_RESPONDER - SAP role, Responder capability is supported
 */
enum fine_time_meas_mask {
	FINE_TIME_MEAS_STA_INITIATOR    = (1 << (0)),
	FINE_TIME_MEAS_STA_RESPONDER    = (1 << (1)),
	FINE_TIME_MEAS_P2PCLI_INITIATOR = (1 << (2)),
	FINE_TIME_MEAS_P2PCLI_RESPONDER = (1 << (3)),
	FINE_TIME_MEAS_P2PGO_INITIATOR  = (1 << (4)),
	FINE_TIME_MEAS_P2PGO_RESPONDER  = (1 << (5)),
	FINE_TIME_MEAS_SAP_INITIATOR    = (1 << (6)),
	FINE_TIME_MEAS_SAP_RESPONDER    = (1 << (7)),
};

/* number of neighbor reports that we can handle in Neighbor Report Response */
#define MAX_SUPPORTED_NEIGHBOR_RPT 15

#ifdef SAP_AUTH_OFFLOAD
/* 80211 Pre-Share Key length */
#define SIR_PSK_MAX_LEN   64

/* Definition for Software AP Auth Offload Security Type */
enum tSirSecurityType
{
    eSIR_OFFLOAD_NONE,
    eSIR_OFFLOAD_WPA2PSK_CCMP,
};

/* Structure for Software AP Auth Offload feature */
struct tSirSapOffloadInfo
{
    uint32_t vdev_id;
    bool sap_auth_offload_enable;
    uint32_t sap_auth_offload_sec_type;
    uint32_t key_len;
    uint8_t key[SIR_PSK_MAX_LEN];
};
#endif /* SAP_AUTH_OFFLOAD */

/**
 * struct sblock_info - the basic block info structure
 *
 * @vdev_id: vdev id
 * @reconnect_cnt: allowed connection fail count in duration
 * @con_fail_duration: duration in which connection failure will be recorded
 * @block_duration: duration in whcih client will be rejected to connect
 *
 * driver use this structure to configure fw client connect block info
 */
struct sblock_info {
	uint32_t vdev_id;
	uint32_t reconnect_cnt;
	uint32_t con_fail_duration;
	uint32_t block_duration;
};

/**
 * struct stsf - the basic stsf structure
 *
 * @vdev_id: vdev id
 * @tsf_low: low 32bits of tsf
 * @tsf_high: high 32bits of tsf
 *
 * driver use this struct to store the tsf info
 */
struct stsf {
	uint32_t vdev_id;
	uint32_t tsf_low;
	uint32_t tsf_high;
};

/**
 * OCB structures
 */

#define NUM_AC			(4)
#define OCB_CHANNEL_MAX	(5)

typedef struct sir_qos_params {
	uint8_t aifsn;
	uint8_t cwmin;
	uint8_t cwmax;
} sir_qos_params_t;

/**
 * struct sir_ocb_set_config_response
 * @status: response status
 */
struct sir_ocb_set_config_response {
	uint8_t status;
};

/** Callback for the dcc_stats_event */
typedef void (*dcc_stats_event_callback_t)(void *hdd_ctx, uint32_t vdev_id,
	uint32_t num_channels, uint32_t stats_per_channel_array_len,
	const void *stats_per_channel_array);

/**
 * struct sir_ocb_config_channel
 * @chan_freq: frequency of the channel
 * @bandwidth: bandwidth of the channel, either 10 or 20 MHz
 * @mac_address: MAC address assigned to this channel
 * @qos_params: QoS parameters
 * @max_pwr: maximum transmit power of the channel (dBm)
 * @min_pwr: minimum transmit power of the channel (dBm)
 * @reg_pwr: maximum transmit power specified by the regulatory domain (dBm)
 * @antenna_max: maximum antenna gain specified by the regulatory domain (dB)
 */
struct sir_ocb_config_channel {
	uint32_t chan_freq;
	uint32_t bandwidth;
	tSirMacAddr mac_address;
	sir_qos_params_t qos_params[MAX_NUM_AC];
	uint32_t max_pwr;
	uint32_t min_pwr;
	uint8_t reg_pwr;
	uint8_t antenna_max;
	uint16_t flags;
};

/**
 * OCB_CHANNEL_FLAG_NO_RX_HDR - Don't add the RX stats header to packets
 *      received on this channel.
 */
#define OCB_CHANNEL_FLAG_DISABLE_RX_STATS_HDR	(1 << 0)

/**
 * struct sir_ocb_config_sched
 * @chan_freq: frequency of the channel
 * @total_duration: duration of the schedule
 * @guard_interval: guard interval on the start of the schedule
 */
struct sir_ocb_config_sched {
	uint32_t chan_freq;
	uint32_t total_duration;
	uint32_t guard_interval;
};

/**
 * struct sir_ocb_config
 * @session_id: session id
 * @channel_count: number of channels
 * @schedule_size: size of the channel schedule
 * @flags: reserved
 * @channels: array of OCB channels
 * @schedule: array of OCB schedule elements
 * @dcc_ndl_chan_list_len: size of the ndl_chan array
 * @dcc_ndl_chan_list: array of dcc channel info
 * @dcc_ndl_active_state_list_len: size of the active state array
 * @dcc_ndl_active_state_list: array of active states
 * @adapter: the OCB adapter
 * @dcc_stats_callback: callback for the response event
 */
struct sir_ocb_config {
	uint8_t session_id;
	uint32_t channel_count;
	uint32_t schedule_size;
	uint32_t flags;
	struct sir_ocb_config_channel *channels;
	struct sir_ocb_config_sched *schedule;
	uint32_t dcc_ndl_chan_list_len;
	void *dcc_ndl_chan_list;
	uint32_t dcc_ndl_active_state_list_len;
	void *dcc_ndl_active_state_list;
};

/* The size of the utc time in bytes. */
#define SIZE_UTC_TIME (10)
/* The size of the utc time error in bytes. */
#define SIZE_UTC_TIME_ERROR (5)

/**
 * struct sir_ocb_utc
 * @vdev_id: session id
 * @utc_time: number of nanoseconds from Jan 1st 1958
 * @time_error: the error in the UTC time. All 1's for unknown
 */
struct sir_ocb_utc {
	uint32_t vdev_id;
	uint8_t utc_time[SIZE_UTC_TIME];
	uint8_t time_error[SIZE_UTC_TIME_ERROR];
};

/**
 * struct sir_ocb_timing_advert
 * @vdev_id: session id
 * @chan_freq: frequency on which to advertise
 * @repeat_rate: the number of times it will send TA in 5 seconds
 * @timestamp_offset: offset of the timestamp field in the TA frame
 * @time_value_offset: offset of the time_value field in the TA frame
 * @template_length: size in bytes of the TA frame
 * @template_value: the TA frame
 */
struct sir_ocb_timing_advert {
	uint32_t vdev_id;
	uint32_t chan_freq;
	uint32_t repeat_rate;
	uint32_t timestamp_offset;
	uint32_t time_value_offset;
	uint32_t template_length;
	uint8_t *template_value;
};

/**
 * struct sir_ocb_get_tsf_timer_response
 * @vdev_id: session id
 * @timer_high: higher 32-bits of the timer
 * @timer_low: lower 32-bits of the timer
 */
struct sir_ocb_get_tsf_timer_response {
	uint32_t vdev_id;
	uint32_t timer_high;
	uint32_t timer_low;
};

/**
 * struct sir_ocb_get_tsf_timer
 * @vdev_id: session id
 */
struct sir_ocb_get_tsf_timer {
	uint32_t vdev_id;
};

/**
 * struct sir_dcc_get_stats_response
 * @vdev_id: session id
 * @num_channels: number of dcc channels
 * @channel_stats_array_len: size in bytes of the stats array
 * @channel_stats_array: the stats array
 */
struct sir_dcc_get_stats_response {
	uint32_t vdev_id;
	uint32_t num_channels;
	uint32_t channel_stats_array_len;
	void *channel_stats_array;
};

/**
 * struct sir_dcc_get_stats
 * @vdev_id: session id
 * @channel_count: number of dcc channels
 * @request_array_len: size in bytes of the request array
 * @request_array: the request array
 */
struct sir_dcc_get_stats {
	uint32_t vdev_id;
	uint32_t channel_count;
	uint32_t request_array_len;
	void *request_array;
};

/**
 * struct sir_dcc_clear_stats
 * @vdev_id: session id
 * @dcc_stats_bitmap: bitmap of clear option
 */
struct sir_dcc_clear_stats {
	uint32_t vdev_id;
	uint32_t dcc_stats_bitmap;
};

/**
 * struct sir_dcc_update_ndl_response
 * @vdev_id: session id
 * @status: response status
 */
struct sir_dcc_update_ndl_response {
	uint32_t vdev_id;
	uint32_t status;
};

/**
 * struct sir_dcc_update_ndl
 * @vdev_id: session id
 * @channel_count: number of channels to be updated
 * @dcc_ndl_chan_list_len: size in bytes of the ndl_chan array
 * @dcc_ndl_chan_list: the ndl_chan array
 * @dcc_ndl_active_state_list_len: size in bytes of the active_state array
 * @dcc_ndl_active_state_list: the active state array
 */
struct sir_dcc_update_ndl {
	uint32_t vdev_id;
	uint32_t channel_count;
	uint32_t dcc_ndl_chan_list_len;
	void *dcc_ndl_chan_list;
	uint32_t dcc_ndl_active_state_list_len;
	void *dcc_ndl_active_state_list;
};

/**
 * struct sir_stats_avg_factor
 * @vdev_id: session id
 * @stats_avg_factor: average factor
 */
struct sir_stats_avg_factor {
	uint8_t vdev_id;
	uint16_t stats_avg_factor;
};

/**
 * struct sir_guard_time_request
 * @vdev_id: session id
 * @guard_time: guard time
 */
struct sir_guard_time_request {
	uint8_t vdev_id;
	uint32_t guard_time;
};

/* Max number of rates allowed in Supported Rates IE */
#define MAX_NUM_SUPPORTED_RATES (8)

#define MAX_NUM_FW_SEGMENTS 4

/**
 * struct fw_dump_seg_req - individual segment details
 * @seg_id - segment id.
 * @seg_start_addr_lo - lower address of the segment.
 * @seg_start_addr_hi - higher address of the segment.
 * @seg_length - length of the segment.
 * @dst_addr_lo - lower address of the destination buffer.
 * @dst_addr_hi - higher address of the destination buffer.
 *
 * This structure carries the information to firmware about the
 * individual segments. This structure is part of firmware memory
 * dump request.
 */
struct fw_dump_seg_req
{
	uint8_t seg_id;
	uint32_t seg_start_addr_lo;
	uint32_t seg_start_addr_hi;
	uint32_t seg_length;
	uint32_t dst_addr_lo;
	uint32_t dst_addr_hi;
};

/**
 * struct fw_dump_req - firmware memory dump request details.
 * @request_id - request id.
 * @num_seg - requested number of segments.
 * @fw_dump_seg_req - individual segment information.
 *
 * This structure carries information about the firmware
 * memory dump request.
 */
struct fw_dump_req
{
	uint32_t request_id;
	uint32_t num_seg;
	struct fw_dump_seg_req segment[MAX_NUM_FW_SEGMENTS];
};

/**
 * struct fw_dump_rsp - firmware dump response details.
 * @request_id - request id.
 * @dump_complete - copy completion status.
 *
 * This structure is used to store the firmware dump copy complete
 * response from the firmware.
 */
struct fw_dump_rsp
{
	uint32_t request_id;
	uint32_t dump_complete;
};

/**
 * struct vdev_ie_info - IE info
 * @vdev_i - vdev for which the IE is being sent
 * @ie_id - ID of the IE
 * @length - length of the IE data
 * @data - IE data
 *
 * This structure is used to store the IE information.
 */
struct vdev_ie_info
{
	uint32_t vdev_id;
	uint32_t ie_id;
	uint32_t length;
	uint8_t *data;
};

/*
 * struct rssi_monitor_req - rssi monitoring
 * @request_id: request id
 * @session_id: session id
 * @min_rssi: minimum rssi
 * @max_rssi: maximum rssi
 * @control: flag to indicate start or stop
 */
struct rssi_monitor_req {
	uint32_t request_id;
	uint32_t session_id;
	int8_t   min_rssi;
	int8_t   max_rssi;
	bool     control;
};

/**
 * struct rssi_breach_event - rssi breached event structure
 * @request_id: request id
 * @session_id: session id
 * @curr_rssi: current rssi
 * @curr_bssid: current bssid
 */
struct rssi_breach_event {
	uint32_t     request_id;
	uint32_t     session_id;
	int8_t       curr_rssi;
	v_MACADDR_t  curr_bssid;
};

/**
 * struct sir_sme_ext_change_chan_req - channel change request
 * @message_type: message id
 * @length: msg length
 * @new_channel: new channel
 * @session_id: session id
 */
struct sir_sme_ext_cng_chan_req
{
	uint16_t  message_type; /* eWNI_SME_EXT_CHANGE_CHANNEL */
	uint16_t  length;
	uint32_t  new_channel;
	uint8_t   session_id;
};

/**
 * struct sir_sme_ext_change_chan_ind.
 * @session_id: session id
 * @new_channel: new channel to change
 */
struct sir_sme_ext_cng_chan_ind
{
	uint8_t  session_id;
	uint8_t  new_channel;
};

/**
 * enum powersave_qpower_mode: QPOWER modes
 * @QPOWER_DISABLED: Qpower is disabled
 * @QPOWER_ENABLED: Qpower is enabled
 * @QPOWER_DUTY_CYCLING: Qpower is enabled with duty cycling
 */
enum powersave_qpower_mode {
	QPOWER_DISABLED = 0,
	QPOWER_ENABLED = 1,
	QPOWER_DUTY_CYCLING = 2
};

/**
 * enum powersave_qpower_mode: powersave_mode
 * @PS_NOT_SUPPORTED: Power save is not supported
 * @PS_LEGACY_NODEEPSLEEP: Legacy power save enabled and deep sleep disabled
 * @PS_QPOWER_NODEEPSLEEP: QPOWER enabled and deep sleep disabled
 * @PS_LEGACY_DEEPSLEEP: Legacy power save enabled and deep sleep enabled
 * @PS_QPOWER_DEEPSLEEP: QPOWER enabled and deep sleep enabled
 * @PS_DUTY_CYCLING_QPOWER: QPOWER enabled in duty cycling mode
 */
enum powersave_mode {
	PS_NOT_SUPPORTED = 0,
	PS_LEGACY_NODEEPSLEEP = 1,
	PS_QPOWER_NODEEPSLEEP = 2,
	PS_LEGACY_DEEPSLEEP = 3,
	PS_QPOWER_DEEPSLEEP = 4,
	PS_DUTY_CYCLING_QPOWER = 5
};


/*
 * struct udp_resp_offload - the basic info structure
 *
 * @vdev_id: vdev id
 * @dest_port: specific UDP dest port
 * @udp_payload_filter: specific UDP payload filter
 * @udp_response_payload: response UDP oayload
 *
 * driver use this structure to configure fw specific
 * udp offload filter and response udp info
 */
struct udp_resp_offload {
	uint32_t   vdev_id;
	uint32_t   enable;
	uint32_t   dest_port;
	char       udp_payload_filter[MAX_LEN_UDP_RESP_OFFLOAD];
	char       udp_response_payload[MAX_LEN_UDP_RESP_OFFLOAD];

};

/**
 * struct beacon_filter_param - parameters for beacon filtering
 * @vdev_id: vdev id
 * @ie_map: bitwise map of IEs that needs to be filtered
 *
 */
struct beacon_filter_param {
	uint32_t   vdev_id;
	uint32_t   ie_map[8];
};

/**
 * struct smps_force_mode_event - smps force mode event param
 * @message_type: Type of message
 * @length: length
 * @vdev_id: vdev id
 * @status: status of the SMPS force mode command
 *
 */
struct sir_smps_force_mode_event {
	uint16_t   message_type;
	uint16_t   length;
	uint8_t    vdev_id;
	uint8_t    status;
};


/**
 * struct sir_del_all_tdls_peers - delete all tdls peers
 * @msg_type: type of message
 * @msg_len: length of message
 * bssid: bssid of peer device
 */
struct sir_del_all_tdls_peers {
	uint16_t msg_type;
	uint16_t msg_len;
	tSirMacAddr bssid;
};

#endif /* __SIR_API_H */
