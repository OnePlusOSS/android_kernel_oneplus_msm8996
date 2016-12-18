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
 * This file limProcessSmeReqMessages.cc contains the code
 * for processing SME request messages.
 * Author:        Chandra Modumudi
 * Date:          02/11/02
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 *
 */

#include "palTypes.h"
#include "wniApi.h"
#include "wniCfgSta.h"
#include "cfgApi.h"
#include "sirApi.h"
#include "schApi.h"
#include "utilsApi.h"
#include "limTypes.h"
#include "limUtils.h"
#include "limAssocUtils.h"
#include "limSecurityUtils.h"
#include "limSerDesUtils.h"
#include "limSmeReqUtils.h"
#include "limIbssPeerMgmt.h"
#include "limAdmitControl.h"
#include "dphHashTable.h"
#include "limSendMessages.h"
#include "limApi.h"
#include "wmmApsd.h"
#include "sirMacProtDef.h"
#include "regdomain_common.h"
#include "rrmApi.h"


#include "sapApi.h"

#if defined(FEATURE_WLAN_ESE) && !defined(FEATURE_WLAN_ESE_UPLOAD)
#include "eseApi.h"
#endif

#if defined WLAN_FEATURE_VOWIFI_11R
#include <limFT.h>
#endif

/* This overhead is time for sending NOA start to host in case of GO/sending NULL data & receiving ACK
 * in case of P2P Client and starting actual scanning with init scan req/rsp plus in case of concurrency,
 * taking care of sending null data and receiving ACK to/from AP/Also SetChannel with calibration is taking
 * around 7ms .
 */
#define SCAN_MESSAGING_OVERHEAD             20      // in msecs
#define JOIN_NOA_DURATION                   2000    // in msecs
#define OEM_DATA_NOA_DURATION               60      // in msecs
#define DEFAULT_PASSIVE_MAX_CHANNEL_TIME    110     // in msecs

#define CONV_MS_TO_US 1024 //conversion factor from ms to us
// SME REQ processing function templates
static void __limProcessSmeStartReq(tpAniSirGlobal, tANI_U32 *);
static tANI_BOOLEAN __limProcessSmeSysReadyInd(tpAniSirGlobal, tANI_U32 *);
static tANI_BOOLEAN __limProcessSmeStartBssReq(tpAniSirGlobal, tpSirMsgQ pMsg);
static void __limProcessSmeScanReq(tpAniSirGlobal, tANI_U32 *);
static void __limProcessSmeJoinReq(tpAniSirGlobal, tANI_U32 *);
static void __limProcessSmeReassocReq(tpAniSirGlobal, tANI_U32 *);
static void __limProcessSmeDisassocReq(tpAniSirGlobal, tANI_U32 *);
static void __limProcessSmeDisassocCnf(tpAniSirGlobal, tANI_U32 *);
static void __limProcessSmeDeauthReq(tpAniSirGlobal, tANI_U32 *);
static void __limProcessSmeSetContextReq(tpAniSirGlobal, tANI_U32 *);
static tANI_BOOLEAN __limProcessSmeStopBssReq(tpAniSirGlobal, tpSirMsgQ pMsg);
static void limProcessSmeChannelChangeRequest(tpAniSirGlobal pMac,
                                                      tANI_U32 *pMsg);
static void limProcessSmeStartBeaconReq(tpAniSirGlobal pMac,
                                                 tANI_U32 *pMsg);
static void limProcessSmeDfsCsaIeRequest(tpAniSirGlobal pMac, tANI_U32 *pMsg);

static void limStartBssUpdateAddIEBuffer(tpAniSirGlobal pMac,
                             tANI_U8 **pDstData_buff,
                             tANI_U16 *pDstDataLen,
                             tANI_U8 *pSrcData_buff,
                             tANI_U16 srcDataLen);

static void limUpdateAddIEBuffer(tpAniSirGlobal pMac,
                             tANI_U8 **pDstData_buff,
                             tANI_U16 *pDstDataLen,
                             tANI_U8 *pSrcData_buff,
                             tANI_U16 srcDataLen);
static tANI_BOOLEAN limUpdateIBssPropAddIEs(tpAniSirGlobal pMac,
                                            tANI_U8 **pDstData_buff,
                                            tANI_U16 *pDstDataLen,
                                            tSirModifyIE *pModifyIE);
static void limProcessModifyAddIEs(tpAniSirGlobal pMac, tANI_U32 *pMsg);

static void limProcessUpdateAddIEs(tpAniSirGlobal pMac, tANI_U32 *pMsg);

static void lim_process_ext_change_channel(tpAniSirGlobal mac_ctx,
						uint32_t *msg);

void __limProcessSmeAssocCnfNew(tpAniSirGlobal, tANI_U32, tANI_U32 *);

extern void peRegisterTLHandle(tpAniSirGlobal pMac);

static void lim_process_set_pdev_IEs(tpAniSirGlobal pMac, tANI_U32 *msg_buf);
static void lim_set_pdev_ht_ie(tpAniSirGlobal mac_ctx, tANI_U8 pdev_id,
		tANI_U8 nss);
static void lim_set_pdev_vht_ie(tpAniSirGlobal mac_ctx, tANI_U8 pdev_id,
		tANI_U8 nss);
#ifdef BACKGROUND_SCAN_ENABLED

// start the background scan timers if it hasn't already started
static void
__limBackgroundScanInitiate(tpAniSirGlobal pMac)
{
    if (pMac->lim.gLimBackgroundScanStarted)
        return;

    //make sure timer is created first
    if (TX_TIMER_VALID(pMac->lim.limTimers.gLimBackgroundScanTimer))
    {
        limDeactivateAndChangeTimer(pMac, eLIM_BACKGROUND_SCAN_TIMER);
     MTRACE(macTrace(pMac, TRACE_CODE_TIMER_ACTIVATE, NO_SESSION, eLIM_BACKGROUND_SCAN_TIMER));
        if (tx_timer_activate(&pMac->lim.limTimers.gLimBackgroundScanTimer) != TX_SUCCESS)
            limLog(pMac, LOGP, FL("could not activate background scan timer"));
        pMac->lim.gLimBackgroundScanStarted   = true;
        pMac->lim.gLimBackgroundScanChannelId = 0;
    }
}

#endif // BACKGROUND_SCAN_ENABLED

// determine if a fresh scan request must be issued or not
/*
* PE will do fresh scan, if all of the active sessions are in good state (Link Est or BSS Started)
* If one of the sessions is not in one of the above states, then PE does not do fresh scan
* If no session exists (scanning very first time), then PE will always do fresh scan if SME
* asks it to do that.
*/
static tANI_U8
__limFreshScanReqd(tpAniSirGlobal pMac, tANI_U8 returnFreshResults)
{

    tANI_U8 validState = TRUE;
    int i;

    limLog(pMac, LOG1, FL("gLimSmeState: %d, returnFreshResults 0x%x"),
        pMac->lim.gLimSmeState, returnFreshResults);
    if(pMac->lim.gLimSmeState != eLIM_SME_IDLE_STATE)
    {
        limLog(pMac, LOG1, FL("return FALSE"));
        return FALSE;
    }
    for(i =0; i < pMac->lim.maxBssId; i++)
    {

        if(pMac->lim.gpSession[i].valid == TRUE)
        {
            limLog(pMac, LOG1,
               FL("session %d, bsstype %d, limSystemRole %d, limSmeState %d"),
               i,
               pMac->lim.gpSession[i].bssType,
               pMac->lim.gpSession[i].limSystemRole,
               pMac->lim.gpSession[i].limSmeState);
            if(!( ( (  (pMac->lim.gpSession[i].bssType == eSIR_INFRASTRUCTURE_MODE) ||
                        (pMac->lim.gpSession[i].limSystemRole == eLIM_BT_AMP_STA_ROLE))&&
                       (pMac->lim.gpSession[i].limSmeState == eLIM_SME_LINK_EST_STATE) )||

                  (    ( (pMac->lim.gpSession[i].bssType == eSIR_IBSS_MODE)||
                           (pMac->lim.gpSession[i].limSystemRole == eLIM_BT_AMP_AP_ROLE)||
                           (pMac->lim.gpSession[i].limSystemRole == eLIM_BT_AMP_STA_ROLE) )&&
                       (pMac->lim.gpSession[i].limSmeState == eLIM_SME_NORMAL_STATE) )
               ||  ( ( ( (pMac->lim.gpSession[i].bssType == eSIR_INFRA_AP_MODE)
                      && ( pMac->lim.gpSession[i].pePersona == VOS_P2P_GO_MODE) )
                    || (pMac->lim.gpSession[i].limSystemRole == eLIM_AP_ROLE) )
                  && (pMac->lim.gpSession[i].limSmeState == eLIM_SME_NORMAL_STATE) )
             ))
                {
                validState = FALSE;
                break;
              }

        }
    }


    if((validState) &&
       (returnFreshResults & SIR_BG_SCAN_RETURN_FRESH_RESULTS)) {
        limLog(pMac, LOG1, FL("validState: %d, return TRUE"), validState);
        return TRUE;
    } else {
        limLog(pMac, LOG1, FL("validState: %d, return FALSE"), validState);
        return FALSE;
    }
}



/**
 * __limIsSmeAssocCnfValid()
 *
 *FUNCTION:
 * This function is called by limProcessLmmMessages() upon
 * receiving SME_ASSOC_CNF.
 *
 *LOGIC:
 * Message validity checks are performed in this function
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMeasReq  Pointer to Received ASSOC_CNF message
 * @return true      When received SME_ASSOC_CNF is formatted
 *                   correctly
 *         false     otherwise
 */

inline static tANI_U8
__limIsSmeAssocCnfValid(tpSirSmeAssocCnf pAssocCnf)
{
    if (limIsGroupAddr(pAssocCnf->peerMacAddr))
        return false;
    else
        return true;
} /*** end __limIsSmeAssocCnfValid() ***/


/**
 * __limGetSmeJoinReqSizeForAlloc()
 *
 *FUNCTION:
 * This function is called in various places to get IE length
 * from tSirBssDescription structure
 * number being scanned.
 *
 *PARAMS:
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 * NA
 *
 *NOTE:
 * NA
 *
 * @param     pBssDescr
 * @return    Total IE length
 */

static tANI_U16
__limGetSmeJoinReqSizeForAlloc(tANI_U8 *pBuf)
{
    tANI_U16 len = 0;

    if (!pBuf)
        return len;

    pBuf += sizeof(tANI_U16);
    len = limGetU16( pBuf );
    return (len + sizeof( tANI_U16 ));
} /*** end __limGetSmeJoinReqSizeForAlloc() ***/


/**----------------------------------------------------------------
\fn     __limIsDeferedMsgForLearn

\brief  Has role only if 11h is enabled. Not used on STA side.
        Defers the message if SME is in learn state and brings
        the LIM back to normal mode.

\param  pMac
\param  pMsg - Pointer to message posted from SME to LIM.
\return TRUE - If defered
        FALSE - Otherwise
------------------------------------------------------------------*/
static tANI_BOOLEAN
__limIsDeferedMsgForLearn(tpAniSirGlobal pMac, tpSirMsgQ pMsg)
{
    if (limIsSystemInScanState(pMac))
    {
        if (limDeferMsg(pMac, pMsg) != TX_SUCCESS)
        {
            PELOGE(limLog(pMac, LOGE, FL("Could not defer Msg = %d"), pMsg->type);)
            return eANI_BOOLEAN_FALSE;
        }
        PELOG1(limLog(pMac, LOG1, FL("Defer the message, in learn mode type = %d"),
                                                                 pMsg->type);)

        /** Send finish scan req to HAL only if LIM is not waiting for any response
         * from HAL like init scan rsp, start scan rsp etc.
         */
        if (GET_LIM_PROCESS_DEFD_MESGS(pMac))
        {
            //Set the resume channel to Any valid channel (invalid).
            //This will instruct HAL to set it to any previous valid channel.
            peSetResumeChannel(pMac, 0, 0);
            limSendHalFinishScanReq(pMac, eLIM_HAL_FINISH_LEARN_WAIT_STATE);
        }

        return eANI_BOOLEAN_TRUE;
    }
    return eANI_BOOLEAN_FALSE;
}

/**----------------------------------------------------------------
\fn     __limIsDeferedMsgForRadar

\brief  Has role only if 11h is enabled. Not used on STA side.
        Defers the message if radar is detected.

\param  pMac
\param  pMsg - Pointer to message posted from SME to LIM.
\return TRUE - If defered
        FALSE - Otherwise
------------------------------------------------------------------*/
static tANI_BOOLEAN
__limIsDeferedMsgForRadar(tpAniSirGlobal pMac, tpSirMsgQ pMsg)
{
    /** fRadarDetCurOperChan will be set only if we detect radar in current
     * operating channel and System Role == AP ROLE */
    //TODO: Need to take care radar detection.
    //if (LIM_IS_RADAR_DETECTED(pMac))
    if( 0 )
    {
        if (limDeferMsg(pMac, pMsg) != TX_SUCCESS)
        {
            PELOGE(limLog(pMac, LOGE, FL("Could not defer Msg = %d"), pMsg->type);)
            return eANI_BOOLEAN_FALSE;
        }
        PELOG1(limLog(pMac, LOG1, FL("Defer the message, in learn mode type = %d"),
                                                                 pMsg->type);)
        return eANI_BOOLEAN_TRUE;
    }
    return eANI_BOOLEAN_FALSE;
}


/**
 * __limProcessSmeStartReq()
 *
 *FUNCTION:
 * This function is called to process SME_START_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeStartReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirResultCodes  retCode = eSIR_SME_SUCCESS;
    tANI_U8          smesessionId;
    tANI_U16         smetransactionId;


   PELOG1(limLog(pMac, LOG1, FL("Received START_REQ"));)

    limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&smesessionId,&smetransactionId);

    if (pMac->lim.gLimSmeState == eLIM_SME_OFFLINE_STATE)
    {
        pMac->lim.gLimSmeState = eLIM_SME_IDLE_STATE;

        MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, NO_SESSION, pMac->lim.gLimSmeState));

        /// By default do not return after first scan match
        pMac->lim.gLimReturnAfterFirstMatch = 0;

        /// By default return unique scan results
        pMac->lim.gLimReturnUniqueResults = true;
        pMac->lim.gLimSmeScanResultLength = 0;
#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
        pMac->lim.gLimSmeLfrScanResultLength = 0;
#endif

        if (((tSirSmeStartReq *) pMsgBuf)->sendNewBssInd)
        {
            /*
                     * Need to indicate new BSSs found during background scanning to
                     * host. Update this parameter at CFG
                     */
            if (cfgSetInt(pMac, WNI_CFG_NEW_BSS_FOUND_IND, ((tSirSmeStartReq *) pMsgBuf)->sendNewBssInd)
                != eSIR_SUCCESS)
            {
                limLog(pMac, LOGP, FL("could not set NEIGHBOR_BSS_IND at CFG"));
                retCode = eSIR_SME_UNEXPECTED_REQ_RESULT_CODE;
            }
        }
    }
    else
    {
        /**
         * Should not have received eWNI_SME_START_REQ in states
         * other than OFFLINE. Return response to host and
         * log error
         */
        limLog(pMac, LOGE, FL("Invalid SME_START_REQ received in SME state %X"),pMac->lim.gLimSmeState );
        retCode = eSIR_SME_UNEXPECTED_REQ_RESULT_CODE;
    }
    limSendSmeRsp(pMac, eWNI_SME_START_RSP, retCode,smesessionId,smetransactionId);
} /*** end __limProcessSmeStartReq() ***/


/** -------------------------------------------------------------
\fn __limProcessSmeSysReadyInd
\brief handles the notification from HDD. PE just forwards this message to HAL.
\param   tpAniSirGlobal pMac
\param   tANI_U32* pMsgBuf
\return  TRUE-Posting to HAL failed, so PE will consume the buffer.
\        FALSE-Posting to HAL successful, so HAL will consume the buffer.
  -------------------------------------------------------------*/
static tANI_BOOLEAN
__limProcessSmeSysReadyInd(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirMsgQ msg;

    msg.type = WDA_SYS_READY_IND;
    msg.reserved = 0;
    msg.bodyptr =  pMsgBuf;
    msg.bodyval = 0;

    if (ANI_DRIVER_TYPE(pMac) != eDRIVER_TYPE_MFG) {
        peRegisterTLHandle(pMac);
    }
    PELOGW(limLog(pMac, LOGW, FL("sending WDA_SYS_READY_IND msg to HAL"));)
    MTRACE(macTraceMsgTx(pMac, NO_SESSION, msg.type));

    if (eSIR_SUCCESS != wdaPostCtrlMsg(pMac, &msg))
    {
        limLog(pMac, LOGP, FL("wdaPostCtrlMsg failed"));
        return eANI_BOOLEAN_TRUE;
    }
    return eANI_BOOLEAN_FALSE;
}

#ifdef WLAN_FEATURE_11AC

tANI_U32 limGetCenterChannel(tpAniSirGlobal pMac,tANI_U8 primarychanNum,ePhyChanBondState secondaryChanOffset, tANI_U8 chanWidth)
{
    if (chanWidth == WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ)
    {
        switch(secondaryChanOffset)
        {
            case PHY_QUADRUPLE_CHANNEL_20MHZ_CENTERED_40MHZ_CENTERED:
                return primarychanNum;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_CENTERED:
               return primarychanNum + 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_CENTERED:
               return primarychanNum - 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_LOW:
               return primarychanNum + 6;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_LOW:
               return primarychanNum + 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_HIGH:
               return primarychanNum - 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_HIGH:
               return primarychanNum - 6;
            default :
               return eSIR_CFG_INVALID_ID;
        }
    }
    else if (chanWidth == WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ)
    {
        switch(secondaryChanOffset)
        {
            case PHY_DOUBLE_CHANNEL_LOW_PRIMARY:
                return primarychanNum + 2;
            case PHY_DOUBLE_CHANNEL_HIGH_PRIMARY:
                return primarychanNum - 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_CENTERED_40MHZ_CENTERED:
                return primarychanNum;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_CENTERED:
               return primarychanNum + 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_CENTERED:
               return primarychanNum - 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_LOW:
               return primarychanNum + 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_LOW:
               return primarychanNum - 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_LOW_40MHZ_HIGH:
               return primarychanNum + 2;
            case PHY_QUADRUPLE_CHANNEL_20MHZ_HIGH_40MHZ_HIGH:
               return primarychanNum - 2;
            default :
               return eSIR_CFG_INVALID_ID;
        }
    }
    return primarychanNum;
}

#endif
/**
 * __limHandleSmeStartBssRequest()
 *
 *FUNCTION:
 * This function is called to process SME_START_BSS_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limHandleSmeStartBssRequest(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tANI_U16                size;
    tANI_U32                val = 0;
    tSirRetStatus           retStatus;
    tSirMacChanNum          channelNumber;
    tLimMlmStartReq         *pMlmStartReq = NULL;
    tpSirSmeStartBssReq     pSmeStartBssReq = NULL;
    tSirResultCodes         retCode = eSIR_SME_SUCCESS;
    tANI_U32                autoGenBssId = FALSE;           //Flag Used in case of IBSS to Auto generate BSSID.
    tANI_U8                 sessionId;
    tpPESession             psessionEntry = NULL;
    tANI_U8                 smesessionId;
    tANI_U16                smetransactionId;
    struct vdev_type_nss    *vdev_type_nss;

#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    //Since the session is not created yet, sending NULL. The response should have the correct state.
    limDiagEventReport(pMac, WLAN_PE_DIAG_START_BSS_REQ_EVENT, NULL, 0, 0);
#endif //FEATURE_WLAN_DIAG_SUPPORT

    PELOG1(limLog(pMac, LOG1, FL("Received START_BSS_REQ"));)

    /* Global Sme state and mlm states are not defined yet, for BT-AMP Suppoprt . TO BE DONE */
    if ( (pMac->lim.gLimSmeState == eLIM_SME_OFFLINE_STATE) ||
         (pMac->lim.gLimSmeState == eLIM_SME_IDLE_STATE))
    {
        size = sizeof(tSirSmeStartBssReq) + SIR_MAC_MAX_IE_LENGTH;

        pSmeStartBssReq = vos_mem_malloc(size);
        if ( NULL == pSmeStartBssReq )
        {
            PELOGE(limLog(pMac, LOGE, FL("AllocateMemory failed for pMac->lim.gpLimStartBssReq"));)
            /// Send failure response to host
            retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
            goto end;
        }

        vos_mem_set((void *)pSmeStartBssReq, size, 0);

        if ((limStartBssReqSerDes(pMac, pSmeStartBssReq, (tANI_U8 *) pMsgBuf) == eSIR_FAILURE) ||
                (!limIsSmeStartBssReqValid(pMac, pSmeStartBssReq)))
        {
            PELOGW(limLog(pMac, LOGW, FL("Received invalid eWNI_SME_START_BSS_REQ"));)
            retCode = eSIR_SME_INVALID_PARAMETERS;
            goto free;
        }

        /* This is the place where PE is going to create a session.
         * If session is not existed, then create a new session */
        if((psessionEntry = peFindSessionByBssid(pMac,pSmeStartBssReq->bssId,&sessionId)) != NULL)
        {
            limLog(pMac, LOGW, FL("Session Already exists for given BSSID"));
            retCode = eSIR_SME_BSS_ALREADY_STARTED_OR_JOINED;
            psessionEntry = NULL;
            goto free;
        }
        else
        {
            if((psessionEntry = peCreateSession(pMac,
                                                pSmeStartBssReq->bssId,
                                                &sessionId,
                                                pMac->lim.maxStation,
                                                pSmeStartBssReq->bssType)) == NULL)
            {
                limLog(pMac, LOGW, FL("Session Can not be created "));
                retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
                goto free;
            }
        }

        /* Probe resp add ie */
        limStartBssUpdateAddIEBuffer(pMac,
                             &psessionEntry->addIeParams.probeRespData_buff,
                             &psessionEntry->addIeParams.probeRespDataLen,
                             pSmeStartBssReq->addIeParams.probeRespData_buff,
                             pSmeStartBssReq->addIeParams.probeRespDataLen);

        /* Probe Beacon add ie */
        limStartBssUpdateAddIEBuffer(pMac,
                         &psessionEntry->addIeParams.probeRespBCNData_buff,
                         &psessionEntry->addIeParams.probeRespBCNDataLen,
                         pSmeStartBssReq->addIeParams.probeRespBCNData_buff,
                         pSmeStartBssReq->addIeParams.probeRespBCNDataLen);

        /* Assoc resp IE */
        limStartBssUpdateAddIEBuffer(pMac,
                         &psessionEntry->addIeParams.assocRespData_buff,
                         &psessionEntry->addIeParams.assocRespDataLen,
                         pSmeStartBssReq->addIeParams.assocRespData_buff,
                         pSmeStartBssReq->addIeParams.assocRespDataLen);

        /* Store the session related parameters in newly created session */
        psessionEntry->pLimStartBssReq = pSmeStartBssReq;

        /* Store PE sessionId in session Table  */
        psessionEntry->peSessionId = sessionId;

        /* Store SME session Id in sessionTable */
        psessionEntry->smeSessionId = pSmeStartBssReq->sessionId;

        psessionEntry->transactionId = pSmeStartBssReq->transactionId;

        vos_mem_copy(&(psessionEntry->htConfig), &(pSmeStartBssReq->htConfig),
                     sizeof(psessionEntry->htConfig));

        sirCopyMacAddr(psessionEntry->selfMacAddr,pSmeStartBssReq->selfMacAddr);

        /* Copy SSID to session table */
        vos_mem_copy( (tANI_U8 *)&psessionEntry->ssId,
                     (tANI_U8 *)&pSmeStartBssReq->ssId,
                      (pSmeStartBssReq->ssId.length + 1));

        psessionEntry->bssType = pSmeStartBssReq->bssType;

        psessionEntry->nwType = pSmeStartBssReq->nwType;

        psessionEntry->beaconParams.beaconInterval = pSmeStartBssReq->beaconInterval;

        /* Store the channel number in session Table */
        psessionEntry->currentOperChannel = pSmeStartBssReq->channelId;

        /*Store Persona */
        psessionEntry->pePersona = pSmeStartBssReq->bssPersona;
        VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO,FL("PE PERSONA=%d"),
            psessionEntry->pePersona);

        /*Update the phymode*/
        psessionEntry->gLimPhyMode = pSmeStartBssReq->nwType;

        psessionEntry->maxTxPower = cfgGetRegulatoryMaxTransmitPower( pMac,
            psessionEntry->currentOperChannel );
        /* Store the dot 11 mode in to the session Table*/
        psessionEntry->dot11mode = pSmeStartBssReq->dot11mode;
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
        psessionEntry->cc_switch_mode = pSmeStartBssReq->cc_switch_mode;
#endif
        psessionEntry->htCapability = IS_DOT11_MODE_HT(psessionEntry->dot11mode);
#ifdef WLAN_FEATURE_11AC
        psessionEntry->vhtCapability = IS_DOT11_MODE_VHT(psessionEntry->dot11mode);
        VOS_TRACE(VOS_MODULE_ID_PE,VOS_TRACE_LEVEL_INFO,
            FL("*****psessionEntry->vhtCapability = %d"),psessionEntry->vhtCapability);
#endif

        psessionEntry->txLdpcIniFeatureEnabled =
                                    pSmeStartBssReq->txLdpcIniFeatureEnabled;

#ifdef WLAN_FEATURE_11W
        psessionEntry->limRmfEnabled = pSmeStartBssReq->pmfCapable ? 1 : 0;
        limLog(pMac, LOG1, FL("Session RMF enabled: %d"), psessionEntry->limRmfEnabled);
#endif

        vos_mem_copy((void*)&psessionEntry->rateSet,
            (void*)&pSmeStartBssReq->operationalRateSet,
            sizeof(tSirMacRateSet));
        vos_mem_copy((void*)&psessionEntry->extRateSet,
            (void*)&pSmeStartBssReq->extendedRateSet,
            sizeof(tSirMacRateSet));

        if (IS_5G_CH(psessionEntry->currentOperChannel))
                vdev_type_nss = &pMac->vdev_type_nss_5g;
        else
                vdev_type_nss = &pMac->vdev_type_nss_2g;
        switch(pSmeStartBssReq->bssType)
        {
            case eSIR_INFRA_AP_MODE:
                 psessionEntry->limSystemRole = eLIM_AP_ROLE;
                 psessionEntry->privacy = pSmeStartBssReq->privacy;
                 psessionEntry->fwdWPSPBCProbeReq = pSmeStartBssReq->fwdWPSPBCProbeReq;
                 psessionEntry->authType = pSmeStartBssReq->authType;
                 /* Store the DTIM period */
                 psessionEntry->dtimPeriod = (tANI_U8)pSmeStartBssReq->dtimPeriod;
                 /*Enable/disable UAPSD*/
                 psessionEntry->apUapsdEnable = pSmeStartBssReq->apUapsdEnable;
                 if (psessionEntry->pePersona == VOS_P2P_GO_MODE)
                 {
                     psessionEntry->proxyProbeRspEn = 0;
                     psessionEntry->vdev_nss = vdev_type_nss->p2p_go;
                 }
                 else
                 {
                     /* To detect PBC overlap in SAP WPS mode, Host handles
                      * Probe Requests.
                      */
                     if(SAP_WPS_DISABLED == pSmeStartBssReq->wps_state)
                     {
                         psessionEntry->proxyProbeRspEn = 1;
                     }
                     else
                     {
                         psessionEntry->proxyProbeRspEn = 0;
                     }
                     psessionEntry->vdev_nss = vdev_type_nss->sap;
                 }
                 psessionEntry->ssidHidden = pSmeStartBssReq->ssidHidden;
                 psessionEntry->wps_state = pSmeStartBssReq->wps_state;
                 psessionEntry->sap_dot11mc = pSmeStartBssReq->sap_dot11mc;
                 limGetShortSlotFromPhyMode(pMac, psessionEntry,
                                            psessionEntry->gLimPhyMode,
                                            &psessionEntry->shortSlotTimeSupported);
                 psessionEntry->isCoalesingInIBSSAllowed =
                                pSmeStartBssReq->isCoalesingInIBSSAllowed;
                 break;
            case eSIR_IBSS_MODE:
                 psessionEntry->limSystemRole = eLIM_STA_IN_IBSS_ROLE;
                 limGetShortSlotFromPhyMode(pMac, psessionEntry,
                                            psessionEntry->gLimPhyMode,
                                            &psessionEntry->shortSlotTimeSupported);

                 // initialize to "OPEN". will be updated upon key installation
                 psessionEntry->encryptType = eSIR_ED_NONE;
                 psessionEntry->vdev_nss = vdev_type_nss->ibss;
                 break;

            case eSIR_BTAMP_AP_MODE:
                 psessionEntry->limSystemRole = eLIM_BT_AMP_AP_ROLE;
                 break;

            case eSIR_BTAMP_STA_MODE:
                 psessionEntry->limSystemRole = eLIM_BT_AMP_STA_ROLE;
                 break;

                 /* There is one more mode called auto mode. which is used no where */

                 //FORBUILD -TEMPFIX.. HOW TO use AUTO MODE?????


            default:
                                   //not used anywhere...used in scan function
                 break;
        }
        limLog(pMac, LOG1, FL("persona - %d, nss - %d"),
                        psessionEntry->pePersona, psessionEntry->vdev_nss);
        // BT-AMP: Allocate memory for the array of parsed (Re)Assoc request structure
        if ( (pSmeStartBssReq->bssType == eSIR_BTAMP_AP_MODE)
        || (pSmeStartBssReq->bssType == eSIR_INFRA_AP_MODE)
        )
        {
            psessionEntry->parsedAssocReq = vos_mem_malloc(
                     psessionEntry->dph.dphHashTable.size * sizeof(tpSirAssocReq));
            if ( NULL == psessionEntry->parsedAssocReq )
            {
                limLog(pMac, LOGW, FL("AllocateMemory() failed"));
                retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
                goto free;
            }
            vos_mem_set(psessionEntry->parsedAssocReq,
                    (psessionEntry->dph.dphHashTable.size * sizeof(tpSirAssocReq)),
                     0 );
        }

        /* Channel Bonding is not addressd yet for BT-AMP Support.. sunit will address channel bonding   */
        if (pSmeStartBssReq->channelId)
        {
            channelNumber = pSmeStartBssReq->channelId;
#ifdef QCA_HT_2040_COEX
            if (pSmeStartBssReq->obssEnabled)
                psessionEntry->htSupportedChannelWidthSet =
                          IS_DOT11_MODE_HT(psessionEntry->dot11mode)  ? 1 : 0;
            else
#endif
                psessionEntry->htSupportedChannelWidthSet =
                                 (pSmeStartBssReq->cbMode)?1:0;
            psessionEntry->htSecondaryChannelOffset = pSmeStartBssReq->cbMode;
            psessionEntry->htRecommendedTxWidthSet =
                                 (psessionEntry->htSecondaryChannelOffset)? 1:0;
            VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO,
                      FL("cbMode %u"), pSmeStartBssReq->cbMode);
#ifdef WLAN_FEATURE_11AC
            if(psessionEntry->vhtCapability)
            {
                tANI_U32 centerChan;
                tANI_U32 chanWidth;

                chanWidth = pSmeStartBssReq->vht_channel_width;

                VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO,
                                FL("vht_channel_width %u"),
                                pSmeStartBssReq->vht_channel_width);

                if(channelNumber <= RF_CHAN_14 &&
                                chanWidth != eHT_CHANNEL_WIDTH_20MHZ)
                {
                     chanWidth = eHT_CHANNEL_WIDTH_20MHZ;
                     limLog(pMac, LOG1, FL("Setting chanWidth to 20Mhz for"
                                                " channel %d"),channelNumber);
                }

                /*
                 * For Sta+p2p-Go concurrency
                 * vhtTxChannelWidthSet is used for storing p2p-GO channel width
                 *  apChanWidth is used for storing the AP channel width that
                 * the Sta is going to associate.
                 * Initialize the apChanWidth same as p2p-GO channel width this
                 * gets over written once the station joins the AP
                 */
                if(chanWidth == eHT_CHANNEL_WIDTH_20MHZ ||
                                chanWidth == eHT_CHANNEL_WIDTH_40MHZ)
                {
                    psessionEntry->vhtTxChannelWidthSet =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ;
                    psessionEntry->apChanWidth =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ;
                }
                if (chanWidth == eHT_CHANNEL_WIDTH_80MHZ)
                {
                    psessionEntry->vhtTxChannelWidthSet =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ;
                    psessionEntry->apChanWidth =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ;

                    centerChan = limGetCenterChannel( pMac, channelNumber,
                                         pSmeStartBssReq->cbMode,
                                         WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ);
                    if(centerChan != eSIR_CFG_INVALID_ID)
                    {
                        limLog(pMac, LOGW, FL("***Center Channel for "
                                     "80MHZ channel width = %d"),centerChan);
                        psessionEntry->apCenterChan = centerChan;
                        if (cfgSetInt(pMac,
                                      WNI_CFG_VHT_CHANNEL_CENTER_FREQ_SEGMENT1,
                                      centerChan) != eSIR_SUCCESS)
                        {
                            limLog(pMac, LOGP, FL("could not set  "
                                      "WNI_CFG_CHANNEL_BONDING_MODE at CFG"));
                            retCode = eSIR_LOGP_EXCEPTION;
                            goto free;
                        }
                    }
                }

            }
            psessionEntry->htSecondaryChannelOffset = limGetHTCBState(pSmeStartBssReq->cbMode);
#endif
        }
        else
        {
            PELOGW(limLog(pMac, LOGW, FL("Received invalid eWNI_SME_START_BSS_REQ"));)
            retCode = eSIR_SME_INVALID_PARAMETERS;
            goto free;
        }

        // Delete pre-auth list if any
        limDeletePreAuthList(pMac);

        psessionEntry->htCapability = IS_DOT11_MODE_HT(pSmeStartBssReq->dot11mode);

            /* keep the RSN/WPA IE information in PE Session Entry
             * later will be using this to check when received (Re)Assoc req
             * */
        limSetRSNieWPAiefromSmeStartBSSReqMessage(pMac,&pSmeStartBssReq->rsnIE,psessionEntry);

        if (LIM_IS_AP_ROLE(psessionEntry) || LIM_IS_IBSS_ROLE(psessionEntry)) {
            psessionEntry->gLimProtectionControl =  pSmeStartBssReq->protEnabled;
            /*each byte will have the following info
             *bit7       bit6    bit5   bit4 bit3   bit2  bit1 bit0
             *reserved reserved RIFS Lsig n-GF ht20 11g 11b*/
            vos_mem_copy( (void *) &psessionEntry->cfgProtection,
                          (void *) &pSmeStartBssReq->ht_capab,
                          sizeof( tANI_U16 ));
            psessionEntry->pAPWPSPBCSession = NULL; // Initialize WPS PBC session link list
        }

        // Prepare and Issue LIM_MLM_START_REQ to MLM
        pMlmStartReq = vos_mem_malloc(sizeof(tLimMlmStartReq));
        if ( NULL == pMlmStartReq )
        {
            limLog(pMac, LOGP, FL("call to AllocateMemory failed for mlmStartReq"));
            retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
            goto free;
        }

        vos_mem_set((void *) pMlmStartReq, sizeof(tLimMlmStartReq), 0);

        /* Copy SSID to the MLM start structure */
        vos_mem_copy( (tANI_U8 *) &pMlmStartReq->ssId,
                          (tANI_U8 *) &pSmeStartBssReq->ssId,
                          pSmeStartBssReq->ssId.length + 1);
        pMlmStartReq->ssidHidden = pSmeStartBssReq->ssidHidden;
        pMlmStartReq->obssProtEnabled = pSmeStartBssReq->obssProtEnabled;


        pMlmStartReq->bssType = psessionEntry->bssType;

        /* Fill PE session Id from the session Table */
        pMlmStartReq->sessionId = psessionEntry->peSessionId;

        if( (pMlmStartReq->bssType == eSIR_BTAMP_STA_MODE) || (pMlmStartReq->bssType == eSIR_BTAMP_AP_MODE )
            || (pMlmStartReq->bssType == eSIR_INFRA_AP_MODE)
        )
        {
            /* Copy the BSSId from sessionTable to mlmStartReq struct */
            sirCopyMacAddr(pMlmStartReq->bssId,psessionEntry->bssId);
        }

        else // ibss mode
        {
            pMac->lim.gLimIbssCoalescingHappened = false;

            if((retStatus = wlan_cfgGetInt(pMac, WNI_CFG_IBSS_AUTO_BSSID, &autoGenBssId)) != eSIR_SUCCESS)
            {
                limLog(pMac, LOGP, FL("Could not retrieve Auto Gen BSSID, retStatus=%d"), retStatus);
                retCode = eSIR_LOGP_EXCEPTION;
                goto free;
            }

            if(!autoGenBssId)
            {
                // We're not auto generating BSSID. Instead, get it from session entry
                sirCopyMacAddr(pMlmStartReq->bssId,psessionEntry->bssId);

                if(pMlmStartReq->bssId[0] & 0x01)
                {
                   PELOGE(limLog(pMac, LOGE, FL("Request to start IBSS with group BSSID\n Autogenerating the BSSID"));)
                   autoGenBssId = TRUE;
                }
            }

            if( autoGenBssId )
            {   //if BSSID is not any uc id. then use locally generated BSSID.
                //Autogenerate the BSSID
                limGetRandomBssid( pMac, pMlmStartReq->bssId);
                pMlmStartReq->bssId[0]= 0x02;

                /* Copy randomly generated BSSID to the session Table */
                sirCopyMacAddr(psessionEntry->bssId,pMlmStartReq->bssId);
            }
        }
        /* store the channel num in mlmstart req structure */
        pMlmStartReq->channelNumber = psessionEntry->currentOperChannel;
        pMlmStartReq->cbMode = pSmeStartBssReq->cbMode;
        pMlmStartReq->beaconPeriod = psessionEntry->beaconParams.beaconInterval;

        if (LIM_IS_AP_ROLE(psessionEntry)) {
            pMlmStartReq->dtimPeriod = psessionEntry->dtimPeriod;
            pMlmStartReq->wps_state = psessionEntry->wps_state;
        } else {
            if (wlan_cfgGetInt(pMac, WNI_CFG_DTIM_PERIOD, &val) != eSIR_SUCCESS)
                limLog(pMac, LOGP, FL("could not retrieve DTIM Period"));
            pMlmStartReq->dtimPeriod = (tANI_U8)val;
        }

        if (wlan_cfgGetInt(pMac, WNI_CFG_CFP_PERIOD, &val) != eSIR_SUCCESS)
            limLog(pMac, LOGP, FL("could not retrieve Beacon interval"));
        pMlmStartReq->cfParamSet.cfpPeriod = (tANI_U8)val;

        if (wlan_cfgGetInt(pMac, WNI_CFG_CFP_MAX_DURATION, &val) != eSIR_SUCCESS)
            limLog(pMac, LOGP, FL("could not retrieve CFPMaxDuration"));
        pMlmStartReq->cfParamSet.cfpMaxDuration = (tANI_U16) val;

        //this may not be needed anymore now, as rateSet is now included in the session entry and MLM has session context.
        vos_mem_copy((void*)&pMlmStartReq->rateSet, (void*)&psessionEntry->rateSet,
                       sizeof(tSirMacRateSet));

        // Now populate the 11n related parameters
        pMlmStartReq->nwType    = psessionEntry->nwType;
        pMlmStartReq->htCapable = psessionEntry->htCapability;
        //
        // FIXME_GEN4 - Determine the appropriate defaults...
        //
        pMlmStartReq->htOperMode        = pMac->lim.gHTOperMode;
        pMlmStartReq->dualCTSProtection = pMac->lim.gHTDualCTSProtection; // Unused
        pMlmStartReq->txChannelWidthSet = psessionEntry->htRecommendedTxWidthSet;

        psessionEntry->limRFBand = limGetRFBand(channelNumber);

        // Initialize 11h Enable Flag
        psessionEntry->lim11hEnable = 0;
        if((pMlmStartReq->bssType != eSIR_IBSS_MODE) &&
            (SIR_BAND_5_GHZ == psessionEntry->limRFBand) )
        {
            if (wlan_cfgGetInt(pMac, WNI_CFG_11H_ENABLED, &val) != eSIR_SUCCESS)
                limLog(pMac, LOGP, FL("Fail to get WNI_CFG_11H_ENABLED "));
            psessionEntry->lim11hEnable = val;

            if (psessionEntry->lim11hEnable &&
                (eSIR_INFRA_AP_MODE == pMlmStartReq->bssType))
            {
                if (wlan_cfgGetInt(pMac, WNI_CFG_DFS_MASTER_ENABLED, &val) !=
                                eSIR_SUCCESS)
                {
                    limLog(pMac, LOGE,
                            FL("Fail to get WNI_CFG_DFS_MASTER_ENABLED"));
                }
                psessionEntry->lim11hEnable = val;
            }
        }

        if (!psessionEntry->lim11hEnable)
        {
            if (cfgSetInt(pMac, WNI_CFG_LOCAL_POWER_CONSTRAINT, 0) != eSIR_SUCCESS)
                limLog(pMac, LOGE, FL
                      ("Fail to set value for WNI_CFG_LOCAL_POWER_CONSTRAINT"));
        }

        psessionEntry ->limPrevSmeState = psessionEntry->limSmeState;
        psessionEntry ->limSmeState     =  eLIM_SME_WT_START_BSS_STATE;
        MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry ->limSmeState));

        limPostMlmMessage(pMac, LIM_MLM_START_REQ, (tANI_U32 *) pMlmStartReq);
        return;
    }
    else
    {

        limLog(pMac, LOGE, FL("Received unexpected START_BSS_REQ, in state %X"),pMac->lim.gLimSmeState);
        retCode = eSIR_SME_BSS_ALREADY_STARTED_OR_JOINED;
        goto end;
    } // if (pMac->lim.gLimSmeState == eLIM_SME_OFFLINE_STATE)

free:
    if ((psessionEntry != NULL) &&
        (psessionEntry->pLimStartBssReq == pSmeStartBssReq))
    {
        psessionEntry->pLimStartBssReq = NULL;
    }
    vos_mem_free( pSmeStartBssReq);
    vos_mem_free( pMlmStartReq);

end:

    /* This routine should return the sme sessionId and SME transaction Id */
    limGetSessionInfo(pMac,(tANI_U8*)pMsgBuf,&smesessionId,&smetransactionId);

    if(NULL != psessionEntry)
    {
        peDeleteSession(pMac,psessionEntry);
        psessionEntry = NULL;
    }
    limSendSmeStartBssRsp(pMac, eWNI_SME_START_BSS_RSP, retCode,psessionEntry,smesessionId,smetransactionId);
} /*** end __limHandleSmeStartBssRequest() ***/


/**--------------------------------------------------------------
\fn     __limProcessSmeStartBssReq

\brief  Wrapper for the function __limHandleSmeStartBssRequest
        This message will be defered until softmac come out of
        scan mode or if we have detected radar on the current
        operating channel.
\param  pMac
\param  pMsg

\return TRUE - If we consumed the buffer
        FALSE - If have defered the message.
 ---------------------------------------------------------------*/
static tANI_BOOLEAN
__limProcessSmeStartBssReq(tpAniSirGlobal pMac, tpSirMsgQ pMsg)
{
    if (__limIsDeferedMsgForLearn(pMac, pMsg) ||
            __limIsDeferedMsgForRadar(pMac, pMsg))
    {
        /**
         * If message defered, buffer is not consumed yet.
         * So return false
         */
        return eANI_BOOLEAN_FALSE;
    }

    __limHandleSmeStartBssRequest(pMac, (tANI_U32 *) pMsg->bodyptr);
    return eANI_BOOLEAN_TRUE;
}


/**
 *  limGetRandomBssid()
 *
 *  FUNCTION:This function is called to process generate the random number for bssid
 *  This function is called to process SME_SCAN_REQ message
 *  from HDD or upper layer application.
 *
 * LOGIC:
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 * 1. geneartes the unique random number for bssid in ibss
 *
 *  @param  pMac      Pointer to Global MAC structure
 *  @param  *data      Pointer to  bssid  buffer
 *  @return None
 */
void limGetRandomBssid(tpAniSirGlobal pMac, tANI_U8 *data)
{
     tANI_U32 random[2] ;
     random[0] = tx_time_get();
     random[0] |= (random[0] << 15) ;
     random[1] = random[0] >> 1;
     vos_mem_copy( data, (tANI_U8*)random, sizeof(tSirMacAddr));
}

static eHalStatus limSendHalStartScanOffloadReq(tpAniSirGlobal pMac,
        tpSirSmeScanReq pScanReq)
{
    tSirScanOffloadReq *pScanOffloadReq;
    tANI_U8 *p;
    tANI_U8 *ht_cap_ie;
    tSirMsgQ msg;
    tANI_U16 i, len;
    tANI_U16  ht_cap_len = 0, addn_ie_len = 0;
#ifdef WLAN_FEATURE_11AC
    tANI_U8 *vht_cap_ie;
    tANI_U16 vht_cap_len = 0;
#endif /* WLAN_FEATURE_11AC */
    tSirRetStatus status, rc = eSIR_SUCCESS;
    tDot11fIEExtCap extracted_extcap = {0};
    bool extcap_present = true;

    pMac->lim.fOffloadScanPending = 0;
    pMac->lim.fOffloadScanP2PSearch = 0;

    if (pScanReq->uIEFieldLen) {
        status = lim_strip_extcap_update_struct(pMac,
                     (uint8_t *) pScanReq + pScanReq->uIEFieldOffset,
                     &pScanReq->uIEFieldLen, &extracted_extcap);

        if (eSIR_SUCCESS != status) {
            extcap_present = false;
            limLog(pMac, LOG1, FL("Unable to Strip ExtCap IE from Scan Req"));
        }

        if (extcap_present) {
            limLog(pMac, LOG1, FL("Extcap was part of SCAN IE - Updating FW"));
            lim_send_ext_cap_ie(pMac, pScanReq->sessionId,
                                &extracted_extcap, true);
        }
    } else {
        limLog(pMac, LOG1, FL("No IEs in the scan request from supplicant"));
    }

    /* The tSirScanOffloadReq will reserve the space for first channel,
       so allocate the memory for (numChannels - 1) and uIEFieldLen */
    len = sizeof(tSirScanOffloadReq) + (pScanReq->channelList.numChannels - 1) +
        pScanReq->uIEFieldLen;

    if (!pMac->per_band_chainmask_supp) {
        if (IS_DOT11_MODE_HT(pScanReq->dot11mode)) {
            limLog(pMac, LOG1,
                   FL("Adding HT Caps IE since dot11mode=%d"),
                   pScanReq->dot11mode);
            ht_cap_len = 2 + sizeof(tHtCaps); /* 2 bytes for EID and Length */
            len += ht_cap_len;
            addn_ie_len += ht_cap_len;
        }

#ifdef WLAN_FEATURE_11AC
        if (IS_DOT11_MODE_VHT(pScanReq->dot11mode)) {
            limLog(pMac, LOG1,
                   FL("Adding VHT Caps IE since dot11mode=%d"),
                pScanReq->dot11mode);
            /* 2 bytes for EID and Length */
            vht_cap_len = 2 + sizeof(tSirMacVHTCapabilityInfo) +
                              sizeof(tSirVhtMcsInfo);
            len += vht_cap_len;
            addn_ie_len += vht_cap_len;
        }
#endif /* WLAN_FEATURE_11AC */
    }

    pScanOffloadReq = vos_mem_malloc(len);
    if ( NULL == pScanOffloadReq )
    {
        limLog(pMac, LOGE,
                FL("AllocateMemory failed for pScanOffloadReq"));
        return eHAL_STATUS_FAILURE;
    }

    vos_mem_set( (tANI_U8 *) pScanOffloadReq, len, 0);

    msg.type = WDA_START_SCAN_OFFLOAD_REQ;
    msg.bodyptr = pScanOffloadReq;
    msg.bodyval = 0;

    vos_mem_copy((tANI_U8 *) pScanOffloadReq->bssId,
            (tANI_U8*) pScanReq->bssId,
            sizeof(tSirMacAddr));

    if (pScanReq->numSsid > SIR_SCAN_MAX_NUM_SSID)
    {
        limLog(pMac, LOGE,
                FL("Invalid value (%d) for numSsid"), SIR_SCAN_MAX_NUM_SSID);
        vos_mem_free (pScanOffloadReq);
        return eHAL_STATUS_FAILURE;
    }

    pScanOffloadReq->numSsid = pScanReq->numSsid;
    for (i = 0; i < pScanOffloadReq->numSsid; i++)
    {
        pScanOffloadReq->ssId[i].length = pScanReq->ssId[i].length;
        vos_mem_copy((tANI_U8 *) pScanOffloadReq->ssId[i].ssId,
                (tANI_U8 *) pScanReq->ssId[i].ssId,
                pScanOffloadReq->ssId[i].length);
    }

    pScanOffloadReq->hiddenSsid = pScanReq->hiddenSsid;
    vos_mem_copy((tANI_U8 *) pScanOffloadReq->selfMacAddr,
            (tANI_U8 *) pScanReq->selfMacAddr,
            sizeof(tSirMacAddr));
    pScanOffloadReq->bssType = pScanReq->bssType;
    pScanOffloadReq->dot11mode = pScanReq->dot11mode;
    pScanOffloadReq->scanType = pScanReq->scanType;
    pScanOffloadReq->minChannelTime = pScanReq->minChannelTime;
    pScanOffloadReq->maxChannelTime = pScanReq->maxChannelTime;
    pScanOffloadReq->restTime= pScanReq->restTime;
    pScanOffloadReq->min_rest_time= pScanReq->min_rest_time;
    pScanOffloadReq->idle_time= pScanReq->idle_time;


    /* for normal scan, the value for p2pScanType should be 0
       always */
    if (pScanReq->p2pSearch)
        pScanOffloadReq->p2pScanType = P2P_SCAN_TYPE_SEARCH;

    pScanOffloadReq->sessionId = pScanReq->sessionId;

    if (pScanOffloadReq->sessionId >= pMac->lim.maxBssId)
        limLog(pMac, LOGE, FL("Invalid pe sessionID : %d"),
                           pScanOffloadReq->sessionId);

    pScanOffloadReq->channelList.numChannels =
        pScanReq->channelList.numChannels;
    p = &(pScanOffloadReq->channelList.channelNumber[0]);
    for (i = 0; i < pScanOffloadReq->channelList.numChannels; i++)
        p[i] = pScanReq->channelList.channelNumber[i];

    pScanOffloadReq->uIEFieldLen = pScanReq->uIEFieldLen;
    pScanOffloadReq->uIEFieldOffset = len - addn_ie_len -
                                      pScanOffloadReq->uIEFieldLen;
    vos_mem_copy(
            (tANI_U8 *) pScanOffloadReq + pScanOffloadReq->uIEFieldOffset,
            (tANI_U8 *) pScanReq + pScanReq->uIEFieldOffset,
            pScanReq->uIEFieldLen);

    if (!pMac->per_band_chainmask_supp) {
        /* Copy HT Capability info if dot11mode is HT */
        if (IS_DOT11_MODE_HT(pScanReq->dot11mode)) {
            /* Populate EID and Length field here */
            ht_cap_ie = (tANI_U8 *) pScanOffloadReq +
                                pScanOffloadReq->uIEFieldOffset +
                                pScanOffloadReq->uIEFieldLen;
            vos_mem_set(ht_cap_ie, ht_cap_len, 0);
            *ht_cap_ie = SIR_MAC_HT_CAPABILITIES_EID;
            *(ht_cap_ie + 1) =  ht_cap_len - 2;
            lim_set_ht_caps(pMac, NULL, ht_cap_ie, ht_cap_len);
            pScanOffloadReq->uIEFieldLen += ht_cap_len;
        }

#ifdef WLAN_FEATURE_11AC
        /* Copy VHT Capability info if dot11mode is VHT Capable */
        if (IS_DOT11_MODE_VHT(pScanReq->dot11mode)) {
            /* Populate EID and Length field here */
            vht_cap_ie = (tANI_U8 *) pScanOffloadReq +
                                 pScanOffloadReq->uIEFieldOffset +
                                 pScanOffloadReq->uIEFieldLen;
            vos_mem_set(vht_cap_ie, vht_cap_len, 0);
            *vht_cap_ie = SIR_MAC_VHT_CAPABILITIES_EID;
            *(vht_cap_ie + 1) =  vht_cap_len - 2;
            lim_set_vht_caps(pMac, NULL, vht_cap_ie, vht_cap_len);
            pScanOffloadReq->uIEFieldLen += vht_cap_len;
        }
#endif /* WLAN_FEATURE_11AC */
    }

    rc = wdaPostCtrlMsg(pMac, &msg);
    if (rc != eSIR_SUCCESS)
    {
        limLog(pMac, LOGE, FL("wdaPostCtrlMsg() return failure"));
        vos_mem_free(pScanOffloadReq);
        return eHAL_STATUS_FAILURE;
    }

    pMac->lim.fOffloadScanPending = 1;
    if (pScanReq->p2pSearch)
        pMac->lim.fOffloadScanP2PSearch = 1;

    limLog(pMac, LOG1, FL("Processed Offload Scan Request Successfully"));

    return eHAL_STATUS_SUCCESS;
}

/**
 * __limProcessSmeScanReq()
 *
 *FUNCTION:
 * This function is called to process SME_SCAN_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 * 1. Periodic scanning should be requesting to return unique
 *    scan results.
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeScanReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tANI_U32            len;
    tLimMlmScanReq      *pMlmScanReq;
    tpSirSmeScanReq     pScanReq;
    tANI_U8             i = 0;

#ifdef FEATURE_WLAN_DIAG_SUPPORT
    limDiagEventReport(pMac, WLAN_PE_DIAG_SCAN_REQ_EVENT, NULL,
                       eSIR_SUCCESS, eSIR_SUCCESS);
#endif

    pScanReq = (tpSirSmeScanReq) pMsgBuf;
    limLog(pMac, LOG1,FL("SME SCAN REQ numChan %d min %d max %d IELen %d"
                         "first %d fresh %d unique %d type %s (%d)"
                         " mode %s (%d)rsp %d"),
           pScanReq->channelList.numChannels,
           pScanReq->minChannelTime,
           pScanReq->maxChannelTime,
           pScanReq->uIEFieldLen,
           pScanReq->returnAfterFirstMatch,
           pScanReq->returnFreshResults,
           pScanReq->returnUniqueResults,
           lim_ScanTypetoString(pScanReq->scanType),
           pScanReq->scanType,
           lim_BackgroundScanModetoString(pScanReq->backgroundScanMode),
           pScanReq->backgroundScanMode, pMac->lim.gLimRspReqd ? 1 : 0);


    /* Since scan req always requires a response, we will overwrite response required here.
     * This is added esp to take care of the condition where in p2p go case, we hold the scan req and
     * insert single NOA. We send the held scan request to FW later on getting start NOA ind from FW so
     * we lose state of the gLimRspReqd flag for the scan req if any other request comes by then.
     * e.g. While unit testing, we found when insert single NOA is done, we see a get stats request which turns the flag
     * gLimRspReqd to FALSE; now when we actually start the saved scan req for init scan after getting
     * NOA started, the gLimRspReqd being a global flag is showing FALSE instead of TRUE value for
     * this saved scan req. Since all scan reqs coming to lim require a response, there is no harm in setting
     * the global flag gLimRspReqd to TRUE here.
     */
     pMac->lim.gLimRspReqd = TRUE;

    /*copy the Self MAC address from SmeReq to the globalplace, used for sending probe req*/
    sirCopyMacAddr(pMac->lim.gSelfMacAddr,  pScanReq->selfMacAddr);

   /* This routine should return the sme sessionId and SME transaction Id */

    if (!limIsSmeScanReqValid(pMac, pScanReq))
    {
        limLog(pMac, LOGE, FL("Received SME_SCAN_REQ with invalid parameters"));

        if (pMac->lim.gLimRspReqd)
        {
            pMac->lim.gLimRspReqd = false;

            limSendSmeScanRsp(pMac, sizeof(tSirSmeScanRsp), eSIR_SME_INVALID_PARAMETERS, pScanReq->sessionId, pScanReq->transactionId);

        } // if (pMac->lim.gLimRspReqd)

        return;
    }

    /*
     * if scan is disabled then return as invalid scan request.
     * if scan in power save is disabled, and system is in power save mode,
     * then ignore scan request.
     */
    if((pMac->lim.fScanDisabled) ||
       (!pMac->psOffloadEnabled &&
       !pMac->lim.gScanInPowersave &&
       !limIsSystemInActiveState(pMac)))
    {
        limLog(pMac, LOGE, FL("SCAN is disabled or SCAN in power save"
                           " is disabled and system is in power save."));

        limSendSmeScanRsp(pMac, offsetof(tSirSmeScanRsp,bssDescription[0]), eSIR_SME_INVALID_PARAMETERS, pScanReq->sessionId, pScanReq->transactionId);
        return;
    }

    /* Clear P2P scan entries before starting any scan */
    if (pMac->fScanOffload)
        limFlushp2pScanResults(pMac);

    /**
     * If scan request is received in idle, joinFailed
     * states or in link established state (in STA role)
     * or in normal state (in STA-in-IBSS/AP role) with
     * 'return fresh scan results' request from HDD or
     * it is periodic background scanning request,
     * trigger fresh scan request to MLM
     */
  if (__limFreshScanReqd(pMac, pScanReq->returnFreshResults))
  {
      limLog(pMac, LOG1, FL("Fresh scan is required"));

      if (pScanReq->returnFreshResults & SIR_BG_SCAN_PURGE_RESUTLS)
      {
          // Discard previously cached scan results
          limReInitScanResults(pMac);
      }

      pMac->lim.gLim24Band11dScanDone     = 0;
      pMac->lim.gLim50Band11dScanDone     = 0;
      pMac->lim.gLimReturnAfterFirstMatch =
          pScanReq->returnAfterFirstMatch;
      pMac->lim.gLimBackgroundScanMode =
          pScanReq->backgroundScanMode;

      pMac->lim.gLimReturnUniqueResults   =
          ((pScanReq->returnUniqueResults) > 0 ? true : false);

      if (pMac->psOffloadEnabled)
      {
         if ((pMac->lim.gLimBackgroundScanMode != eSIR_ROAMING_SCAN) &&
            (!IS_ACTIVEMODE_OFFLOAD_FEATURE_ENABLE))
         {
            for (i=0;i<pMac->lim.maxBssId;i++)
            {
               tpPESession psessionEntry = peFindSessionBySessionId(pMac,i);
               if (psessionEntry && psessionEntry->valid &&
                  (eLIM_MLM_LINK_ESTABLISHED_STATE ==
                          psessionEntry->limMlmState) &&
                  (psessionEntry->pmmOffloadInfo.psstate == PMM_FULL_POWER))
               {
                  limHeartBeatDeactivateAndChangeTimer(pMac, psessionEntry);
               }
            }
         }
      }
      /* De-activate Heartbeat timers for connected sessions while
       * scan is in progress if the system is in Active mode *
       * AND it is not a ROAMING ("background") scan */
      else if (((ePMM_STATE_BMPS_WAKEUP == pMac->pmm.gPmmState) ||
                  (ePMM_STATE_READY == pMac->pmm.gPmmState)) &&
              (pScanReq->backgroundScanMode != eSIR_ROAMING_SCAN ) &&
              (!IS_ACTIVEMODE_OFFLOAD_FEATURE_ENABLE))
      {
          for(i=0;i<pMac->lim.maxBssId;i++)
          {
              if((peFindSessionBySessionId(pMac,i) != NULL) &&
                      (pMac->lim.gpSession[i].valid == TRUE) &&
                      (eLIM_MLM_LINK_ESTABLISHED_STATE == pMac->lim.gpSession[i].limMlmState))
              {
                  limHeartBeatDeactivateAndChangeTimer(pMac, peFindSessionBySessionId(pMac,i));
              }
          }
      }

      if (pMac->fScanOffload)
      {
          if (eHAL_STATUS_SUCCESS !=
                  limSendHalStartScanOffloadReq(pMac, pScanReq))
          {
              limLog(pMac, LOGE, FL("Couldn't send Offload scan request"));
              limSendSmeScanRsp(pMac,
                      offsetof(tSirSmeScanRsp, bssDescription[0]),
                      eSIR_SME_INVALID_PARAMETERS,
                      pScanReq->sessionId,
                      pScanReq->transactionId);
              return;
          }
      }
      else
      {

          /*Change Global SME state  */
          /* Store the previous SME state */
          limLog(pMac, LOG1, FL("Non Offload SCAN request "));
          pMac->lim.gLimPrevSmeState = pMac->lim.gLimSmeState;
          pMac->lim.gLimSmeState = eLIM_SME_WT_SCAN_STATE;
          MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, pScanReq->sessionId, pMac->lim.gLimSmeState));

          if (pScanReq->channelList.numChannels == 0)
          {
              tANI_U32            cfg_len;

              limLog(pMac, LOG1,
                    FL("Scan all channels as Number of channels is 0"));

              // Scan all channels
              len = sizeof(tLimMlmScanReq) +
                  (sizeof( pScanReq->channelList.channelNumber ) * (WNI_CFG_VALID_CHANNEL_LIST_LEN - 1)) +
                  pScanReq->uIEFieldLen;
              pMlmScanReq = vos_mem_malloc(len);
              if ( NULL == pMlmScanReq )
              {
                // Log error
                limLog(pMac, LOGP,
                       FL("call to AllocateMemory failed for mlmScanReq (%d)"), len);

                  return;
               }

              // Initialize this buffer
              vos_mem_set( (tANI_U8 *) pMlmScanReq, len, 0 );

              cfg_len = WNI_CFG_VALID_CHANNEL_LIST_LEN;
              if (wlan_cfgGetStr(pMac, WNI_CFG_VALID_CHANNEL_LIST,
                          pMlmScanReq->channelList.channelNumber,
                          &cfg_len) != eSIR_SUCCESS)
              {
                  /**
                   * Could not get Valid channel list from CFG.
                   * Log error.
                   */
                  limLog(pMac, LOGP,
                          FL("could not retrieve Valid channel list"));
              }
              pMlmScanReq->channelList.numChannels = (tANI_U8) cfg_len;
          }
          else
          {
              len = sizeof( tLimMlmScanReq ) - sizeof( pScanReq->channelList.channelNumber ) +
                  (sizeof( pScanReq->channelList.channelNumber ) * pScanReq->channelList.numChannels ) +
                  pScanReq->uIEFieldLen;

              pMlmScanReq = vos_mem_malloc(len);
              if ( NULL == pMlmScanReq )
              {
                // Log error
                limLog(pMac, LOGP,
                    FL("call to AllocateMemory failed for mlmScanReq(%d)"), len);

                  return;
               }

              // Initialize this buffer
              vos_mem_set( (tANI_U8 *) pMlmScanReq, len, 0);
              if (pScanReq->channelList.numChannels <= SIR_ESE_MAX_MEAS_IE_REQS)
              {
                  pMlmScanReq->channelList.numChannels =
                           pScanReq->channelList.numChannels;
              }
              else
              {
                  limLog(pMac, LOGE,
                    FL("numChannels is more than the size(%d)"),
                    pScanReq->channelList.numChannels);
                  pMlmScanReq->channelList.numChannels =
                      SIR_ESE_MAX_MEAS_IE_REQS;
              }

              vos_mem_copy( pMlmScanReq->channelList.channelNumber,
                          pScanReq->channelList.channelNumber,
                          pMlmScanReq->channelList.numChannels);
        }

         pMlmScanReq->uIEFieldLen = pScanReq->uIEFieldLen;
         pMlmScanReq->uIEFieldOffset = len - pScanReq->uIEFieldLen;
         if(pScanReq->uIEFieldLen)
         {
            vos_mem_copy( (tANI_U8 *)pMlmScanReq+ pMlmScanReq->uIEFieldOffset,
                          (tANI_U8 *)pScanReq+(pScanReq->uIEFieldOffset),
                          pScanReq->uIEFieldLen);
         }

         pMlmScanReq->bssType = pScanReq->bssType;
         vos_mem_copy( pMlmScanReq->bssId,
                      pScanReq->bssId,
                      sizeof(tSirMacAddr));
         pMlmScanReq->numSsid = pScanReq->numSsid;
         pMlmScanReq->sessionId = pScanReq->sessionId;

         i = 0;
         while (i < pMlmScanReq->numSsid)
         {
            vos_mem_copy( (tANI_U8 *) &pMlmScanReq->ssId[i],
                      (tANI_U8 *) &pScanReq->ssId[i],
                      pScanReq->ssId[i].length + 1);

              i++;
          }


          pMlmScanReq->scanType = pScanReq->scanType;
          pMlmScanReq->backgroundScanMode = pScanReq->backgroundScanMode;
          pMlmScanReq->minChannelTime = pScanReq->minChannelTime;
          pMlmScanReq->maxChannelTime = pScanReq->maxChannelTime;
          pMlmScanReq->minChannelTimeBtc = pScanReq->minChannelTimeBtc;
          pMlmScanReq->maxChannelTimeBtc = pScanReq->maxChannelTimeBtc;
          pMlmScanReq->dot11mode = pScanReq->dot11mode;
          pMlmScanReq->p2pSearch = pScanReq->p2pSearch;

          //Store the smeSessionID and transaction ID for later use.
          pMac->lim.gSmeSessionId = pScanReq->sessionId;
          pMac->lim.gTransactionId = pScanReq->transactionId;

          // Issue LIM_MLM_SCAN_REQ to MLM
          limLog(pMac, LOG1, FL("Issue Scan request command to MLM "));
          limPostMlmMessage(pMac, LIM_MLM_SCAN_REQ, (tANI_U32 *) pMlmScanReq);
      }
  } // if ((pMac->lim.gLimSmeState == eLIM_SME_IDLE_STATE) || ...

    else
    {
        /// In all other cases return 'cached' scan results
        if ((pMac->lim.gLimRspReqd) || pMac->lim.gLimReportBackgroundScanResults)
        {
            tANI_U16    scanRspLen = sizeof(tSirSmeScanRsp);

            pMac->lim.gLimRspReqd = false;
#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
            if (pScanReq->returnFreshResults & SIR_BG_SCAN_RETURN_LFR_CACHED_RESULTS)
            {
                pMac->lim.gLimSmeLfrScanResultLength = pMac->lim.gLimMlmLfrScanResultLength;
                limLog(pMac, LOG1,
                   FL("Returned scan results from LFR cache, length = %d"),
                   pMac->lim.gLimSmeLfrScanResultLength);

                if (pMac->lim.gLimSmeLfrScanResultLength == 0)
                {
                   limSendSmeLfrScanRsp(pMac, scanRspLen,
                                         eSIR_SME_SUCCESS,
                                         pScanReq->sessionId,
                                         pScanReq->transactionId);
                }
                else
                {
                    scanRspLen = sizeof(tSirSmeScanRsp) +
                                 pMac->lim.gLimSmeLfrScanResultLength -
                                 sizeof(tSirBssDescription);
                    limSendSmeLfrScanRsp(pMac, scanRspLen, eSIR_SME_SUCCESS,
                               pScanReq->sessionId, pScanReq->transactionId);
                }
            }
            else
            {
#endif
                limLog(pMac, LOG1,
                   FL("Returned scan results from normal cache, length = %d"),
                   pMac->lim.gLimSmeScanResultLength);
               if (pMac->lim.gLimSmeScanResultLength == 0)
               {
                  limSendSmeScanRsp(pMac, scanRspLen, eSIR_SME_SUCCESS,
                          pScanReq->sessionId, pScanReq->transactionId);
               }
               else
               {
                  scanRspLen = sizeof(tSirSmeScanRsp) +
                               pMac->lim.gLimSmeScanResultLength -
                               sizeof(tSirBssDescription);
                  limSendSmeScanRsp(pMac, scanRspLen, eSIR_SME_SUCCESS,
                                  pScanReq->sessionId, pScanReq->transactionId);
               }
#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
            }
#endif
            limLog(pMac, LOG1, FL("Cached scan results are returned "));

            if (pScanReq->returnFreshResults & SIR_BG_SCAN_PURGE_RESUTLS)
            {
                // Discard previously cached scan results
                limReInitScanResults(pMac);
            }
#ifdef WLAN_FEATURE_ROAM_SCAN_OFFLOAD
            if (pScanReq->returnFreshResults & SIR_BG_SCAN_PURGE_LFR_RESULTS)
            {
                // Discard previously cached scan results
                limReInitLfrScanResults(pMac);
            }
#endif

        } // if (pMac->lim.gLimRspReqd)
    } // else ((pMac->lim.gLimSmeState == eLIM_SME_IDLE_STATE) || ...

#ifdef BACKGROUND_SCAN_ENABLED
    // start background scans if needed
    // There is a bug opened against softmac. Need to enable when the bug is fixed.
    __limBackgroundScanInitiate(pMac);
#endif

} /*** end __limProcessSmeScanReq() ***/

#ifdef FEATURE_OEM_DATA_SUPPORT

static void __limProcessSmeOemDataReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirOemDataReq    pOemDataReq;
    tLimMlmOemDataReq* pMlmOemDataReq;

    pOemDataReq = (tpSirOemDataReq) pMsgBuf;

    //post the lim mlm message now
    pMlmOemDataReq = vos_mem_malloc(sizeof(*pMlmOemDataReq));
    if ( NULL == pMlmOemDataReq )
    {
        limLog(pMac, LOGP, FL("AllocateMemory failed for mlmOemDataReq"));
        return;
    }

    pMlmOemDataReq->data = vos_mem_malloc(pOemDataReq->data_len);
    if (!pMlmOemDataReq->data) {
        limLog(pMac, LOGP, FL("memory allocation failed"));
        vos_mem_free(pMlmOemDataReq);
        return;
    }

    vos_mem_copy( pMlmOemDataReq->selfMacAddr, pOemDataReq->selfMacAddr,
                  sizeof(tSirMacAddr));

    pMlmOemDataReq->data_len = pOemDataReq->data_len;
    vos_mem_copy(pMlmOemDataReq->data, pOemDataReq->data,
                 pOemDataReq->data_len);
    /* buffer from SME copied, free it now */
    vos_mem_free(pOemDataReq->data);

    //Issue LIM_MLM_OEM_DATA_REQ to MLM
    limPostMlmMessage(pMac, LIM_MLM_OEM_DATA_REQ, (tANI_U32*)pMlmOemDataReq);

    return;

} /*** end __limProcessSmeOemDataReq() ***/

#endif //FEATURE_OEM_DATA_SUPPORT

/**
 * __limProcessClearDfsChannelList()
 *
 *FUNCTION:
 *Clear DFS channel list  when country is changed/aquired.
.*This message is sent from SME.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */
static void __limProcessClearDfsChannelList(tpAniSirGlobal pMac,
                                                           tpSirMsgQ pMsg)
{
    vos_mem_set( &pMac->lim.dfschannelList,
                  sizeof(tSirDFSChannelList), 0);
}

/**
 * __limProcessSmeJoinReq()
 *
 *FUNCTION:
 * This function is called to process SME_JOIN_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */
static void
__limProcessSmeJoinReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
  //  tANI_U8             *pBuf;
    //tANI_U32            len;
//    tSirMacAddr         currentBssId;
    tpSirSmeJoinReq     pSmeJoinReq = NULL;
    tLimMlmJoinReq      *pMlmJoinReq;
    tSirResultCodes     retCode = eSIR_SME_SUCCESS;
    tANI_U32            val = 0;
    tANI_U16            nSize;
    tANI_U8             sessionId;
    tpPESession         psessionEntry = NULL;
    tANI_U8             smesessionId;
    tANI_U16            smetransactionId;
    tPowerdBm           localPowerConstraint = 0, regMax = 0;
    tANI_U16            ieLen;
    v_U8_t              *vendorIE;
    struct vdev_type_nss *vdev_type_nss;

#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    //Not sending any session, since it is not created yet. The response whould have correct state.
    limDiagEventReport(pMac, WLAN_PE_DIAG_JOIN_REQ_EVENT, NULL, 0, 0);
#endif //FEATURE_WLAN_DIAG_SUPPORT

    PELOG1(limLog(pMac, LOG1, FL("Received SME_JOIN_REQ"));)

   /**
     * Expect Join request in idle state.
     * Reassociate request is expected in link established state.
     */

    /* Global SME and LIM states are not defined yet for BT-AMP Support */
    if(pMac->lim.gLimSmeState == eLIM_SME_IDLE_STATE)
    {
        nSize = __limGetSmeJoinReqSizeForAlloc((tANI_U8*) pMsgBuf);

        pSmeJoinReq = vos_mem_malloc(nSize);
        if ( NULL == pSmeJoinReq )
        {
            limLog(pMac, LOGP, FL("call to AllocateMemory failed for "
                                  "pSmeJoinReq"));
            retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
            goto end;
        }
        (void) vos_mem_set((void *) pSmeJoinReq, nSize, 0);

        if ((limJoinReqSerDes(pMac, pSmeJoinReq, (tANI_U8 *)pMsgBuf) == eSIR_FAILURE) ||
                (!limIsSmeJoinReqValid(pMac, pSmeJoinReq)))
        {
            /// Received invalid eWNI_SME_JOIN_REQ
            // Log the event
            limLog(pMac, LOGW, FL("SessionId:%d Received SME_JOIN_REQ with invalid data"),
                   pSmeJoinReq->sessionId);
            retCode = eSIR_SME_INVALID_PARAMETERS;
            goto end;
        }

        /*
         * Update the capability here itself as this is used in
         * limExtractAPCapability() below. If not updated issues like not
         * honoring power constraint on 1st association after driver loading
         * might occur.
         */
        lim_update_rrm_capability(pMac, pSmeJoinReq);

        /* check for the existence of start BSS session  */

        if((psessionEntry = peFindSessionByBssid(pMac,pSmeJoinReq->bssDescription.bssId,&sessionId)) != NULL)
        {
            limLog(pMac, LOGE, FL("Session(%d) Already exists for BSSID: "
            MAC_ADDRESS_STR" in limSmeState = %X"),sessionId,
            MAC_ADDR_ARRAY(pSmeJoinReq->bssDescription.bssId),
            psessionEntry->limSmeState);

            if(psessionEntry->limSmeState == eLIM_SME_LINK_EST_STATE &&
               psessionEntry->smeSessionId == pSmeJoinReq->sessionId)
            {
                // Received eWNI_SME_JOIN_REQ for same
                // BSS as currently associated.
                // Log the event and send success
                PELOGW(limLog(pMac, LOGW, FL("SessionId:%d Received SME_JOIN_REQ for currently joined BSS"),
                       sessionId);)
                /// Send Join success response to host
                retCode = eSIR_SME_ALREADY_JOINED_A_BSS;
                psessionEntry = NULL;
                goto end;
            }
            else
            {
                PELOGE(limLog(pMac, LOGE, FL("SME_JOIN_REQ not for"
                                          "currently joined BSS"));)
                retCode = eSIR_SME_REFUSED;
                psessionEntry = NULL;
                goto end;
            }
        }
        else       /* Session Entry does not exist for given BSSId */
        {
            /* Try to Create a new session */
            if((psessionEntry = peCreateSession(pMac,
                    pSmeJoinReq->bssDescription.bssId,
                    &sessionId,
                    pMac->lim.maxStation,
                    eSIR_INFRASTRUCTURE_MODE )) == NULL)
            {
                limLog(pMac, LOGE, FL("Session Can not be created "));
                retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
                goto end;
            }
            else
              limLog(pMac,LOG1,FL("SessionId:%d New session created"),
                     sessionId);
        }
        psessionEntry->max_amsdu_num = pSmeJoinReq->max_amsdu_num;

        /* Store Session related parameters */
        /* Store PE session Id in session Table */
        psessionEntry->peSessionId = sessionId;

        /* store the smejoin req handle in session table */
        psessionEntry->pLimJoinReq = pSmeJoinReq;

        /* Store SME session Id in sessionTable */
        psessionEntry->smeSessionId = pSmeJoinReq->sessionId;

        /* Store SME transaction Id in session Table */
        psessionEntry->transactionId = pSmeJoinReq->transactionId;

        /* Store beaconInterval */
        psessionEntry->beaconParams.beaconInterval = pSmeJoinReq->bssDescription.beaconInterval;

        vos_mem_copy(&(psessionEntry->htConfig), &(pSmeJoinReq->htConfig),
                     sizeof(psessionEntry->htConfig));

        /* Copying of bssId is already done, while creating session */
        sirCopyMacAddr(psessionEntry->selfMacAddr,pSmeJoinReq->selfMacAddr);
        psessionEntry->bssType = pSmeJoinReq->bsstype;

        psessionEntry->statypeForBss = STA_ENTRY_PEER;
        psessionEntry->limWmeEnabled = pSmeJoinReq->isWMEenabled;
        psessionEntry->limQosEnabled = pSmeJoinReq->isQosEnabled;

        /* Store vendor specfic IE for CISCO AP */
        ieLen = (pSmeJoinReq->bssDescription.length +
                  sizeof( pSmeJoinReq->bssDescription.length ) -
                  GET_FIELD_OFFSET( tSirBssDescription, ieFields ));

        vendorIE = cfg_get_vendor_ie_ptr_from_oui(pMac, SIR_MAC_CISCO_OUI,
                    SIR_MAC_CISCO_OUI_SIZE,
                   ((tANI_U8 *)&pSmeJoinReq->bssDescription.ieFields) , ieLen);

        if (NULL != vendorIE) {
            limLog(pMac, LOG1, FL("Cisco vendor OUI present"));
            psessionEntry->isCiscoVendorAP = TRUE;
        } else {
            psessionEntry->isCiscoVendorAP = FALSE;
        }

        /* Copy the dot 11 mode in to the session table */

        psessionEntry->dot11mode  = pSmeJoinReq->dot11mode;
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
        psessionEntry->cc_switch_mode = pSmeJoinReq->cc_switch_mode;
#endif
        psessionEntry->nwType = pSmeJoinReq->bssDescription.nwType;
        psessionEntry->enableAmpduPs = pSmeJoinReq->enableAmpduPs;
        psessionEntry->enableHtSmps = pSmeJoinReq->enableHtSmps;
        psessionEntry->htSmpsvalue = pSmeJoinReq->htSmps;
        /*
         * By default supported NSS 1x1 is set to true
         * and later on updated while determining session
         * supported rates which is the intersection of
         * self and peer rates
         */
        psessionEntry->supported_nss_1x1 = true;

        limLog(pMac, LOG1,
               FL("enableHtSmps: %d htSmps: %d supported NSS 1x1: %d"),
               psessionEntry->enableHtSmps,
               psessionEntry->htSmpsvalue,
               psessionEntry->supported_nss_1x1);

        /*Store Persona */
        psessionEntry->pePersona = pSmeJoinReq->staPersona;
        VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO,
                  FL("PE PERSONA=%d cbMode %u"), psessionEntry->pePersona,
                      pSmeJoinReq->cbMode);
        /* Copy The channel Id to the session Table */
        psessionEntry->currentOperChannel =
                pSmeJoinReq->bssDescription.channelId;
        if (IS_5G_CH(psessionEntry->currentOperChannel))
                vdev_type_nss = &pMac->vdev_type_nss_5g;
        else
                vdev_type_nss = &pMac->vdev_type_nss_2g;
        if (psessionEntry->pePersona == VOS_P2P_CLIENT_MODE)
            psessionEntry->vdev_nss = vdev_type_nss->p2p_cli;
        else
            psessionEntry->vdev_nss = vdev_type_nss->sta;

        limLog(pMac, LOG1, FL("persona - %d, nss - %d"),
                        psessionEntry->pePersona, psessionEntry->vdev_nss);
#ifdef WLAN_FEATURE_11AC
        psessionEntry->vhtCapability = IS_DOT11_MODE_VHT(psessionEntry->dot11mode);
        VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO_MED,
            "***__limProcessSmeJoinReq: vhtCapability=%d****",psessionEntry->vhtCapability);
        if (psessionEntry->vhtCapability )
        {
            if (psessionEntry->pePersona == VOS_STA_MODE)
            {
                    psessionEntry->txBFIniFeatureEnabled = pSmeJoinReq->txBFIniFeatureEnabled;
            }
            else
            {
                    psessionEntry->txBFIniFeatureEnabled = 0;
            }
            psessionEntry->txMuBformee = pSmeJoinReq->txMuBformee;
            psessionEntry->enableVhtpAid = pSmeJoinReq->enableVhtpAid;
            psessionEntry->enableVhtGid = pSmeJoinReq->enableVhtGid;

            VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO_MED,
                "***__limProcessSmeJoinReq: txBFIniFeatureEnabled=%d****",
                psessionEntry->txBFIniFeatureEnabled);

            if( psessionEntry->txBFIniFeatureEnabled )
            {
                if (cfgSetInt(pMac, WNI_CFG_VHT_SU_BEAMFORMEE_CAP, psessionEntry->txBFIniFeatureEnabled)
                                                             != eSIR_SUCCESS)
                {
                    limLog(pMac, LOGP, FL("could not set  "
                                  "WNI_CFG_VHT_SU_BEAMFORMEE_CAP at CFG"));
                    retCode = eSIR_LOGP_EXCEPTION;
                    goto end;
                }
                VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO_MED,
                    "***__limProcessSmeJoinReq: txBFCsnValue=%d****",
                    pSmeJoinReq->txBFCsnValue);

                if (cfgSetInt(pMac, WNI_CFG_VHT_CSN_BEAMFORMEE_ANT_SUPPORTED, pSmeJoinReq->txBFCsnValue)
                                                             != eSIR_SUCCESS)
                {
                    limLog(pMac, LOGP, FL("could not set "
                     "WNI_CFG_VHT_CSN_BEAMFORMEE_ANT_SUPPORTED at CFG"));
                    retCode = eSIR_LOGP_EXCEPTION;
                    goto end;
                }
            }
        }

#endif

        /*Phy mode*/
        psessionEntry->gLimPhyMode = pSmeJoinReq->bssDescription.nwType;
        handleHTCapabilityandHTInfo(pMac, psessionEntry);
        psessionEntry->htSupportedChannelWidthSet = (pSmeJoinReq->cbMode)?1:0; // This is already merged value of peer and self - done by csr in csrGetCBModeFromIes
        psessionEntry->htRecommendedTxWidthSet = psessionEntry->htSupportedChannelWidthSet;
        psessionEntry->htSecondaryChannelOffset = pSmeJoinReq->cbMode;

        /* Record if management frames need to be protected */
#ifdef WLAN_FEATURE_11W
        if(eSIR_ED_AES_128_CMAC == pSmeJoinReq->MgmtEncryptionType)
        {
            VOS_STATUS vosStatus;
            psessionEntry->limRmfEnabled = 1;
            /*
             * For STA profile only:
             * init pmf comeback timer and info struct only if PMF connection
             */
            psessionEntry->pmfComebackTimerInfo.pMac = pMac;
            psessionEntry->pmfComebackTimerInfo.sessionID = sessionId;
            vosStatus = vos_timer_init(&psessionEntry->pmfComebackTimer,
                                   VOS_TIMER_TYPE_SW,
                                   limPmfComebackTimerCallback,
                                   (void*)&psessionEntry->pmfComebackTimerInfo);
            if (VOS_STATUS_SUCCESS != vosStatus) {
                limLog(pMac, LOGP,
                       FL("cannot init pmf comeback timer."));
                retCode = eSIR_LOGP_EXCEPTION;
                goto end;
            }
        }
        else
        {
            psessionEntry->limRmfEnabled = 0;
        }
#endif

#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM
        psessionEntry->rssi =  pSmeJoinReq->bssDescription.rssi;
#endif


        /* Copy the SSID from smejoinreq to session entry  */
        psessionEntry->ssId.length = pSmeJoinReq->ssId.length;
        vos_mem_copy( psessionEntry->ssId.ssId,
                      pSmeJoinReq->ssId.ssId, psessionEntry->ssId.length);

        // Determin 11r or ESE connection based on input from SME
        // which inturn is dependent on the profile the user wants to connect
        // to, So input is coming from supplicant
#ifdef WLAN_FEATURE_VOWIFI_11R
        psessionEntry->is11Rconnection = pSmeJoinReq->is11Rconnection;
#endif
#ifdef FEATURE_WLAN_ESE
        psessionEntry->isESEconnection = pSmeJoinReq->isESEconnection;
#endif
#if defined WLAN_FEATURE_VOWIFI_11R || defined FEATURE_WLAN_ESE || defined(FEATURE_WLAN_LFR)
        psessionEntry->isFastTransitionEnabled = pSmeJoinReq->isFastTransitionEnabled;
#endif

#ifdef FEATURE_WLAN_LFR
        psessionEntry->isFastRoamIniFeatureEnabled = pSmeJoinReq->isFastRoamIniFeatureEnabled;
#endif
        psessionEntry->txLdpcIniFeatureEnabled = pSmeJoinReq->txLdpcIniFeatureEnabled;

        if (psessionEntry->bssType == eSIR_INFRASTRUCTURE_MODE)
        {
            psessionEntry->limSystemRole = eLIM_STA_ROLE;
        }
        else if (psessionEntry->bssType == eSIR_BTAMP_AP_MODE)
        {
            psessionEntry->limSystemRole = eLIM_BT_AMP_STA_ROLE;
        }
        else
        {
            /* Throw an error and return and make sure to delete the session.*/
            limLog(pMac, LOGE, FL("received SME_JOIN_REQ with invalid"
                                 " bss type %d"), psessionEntry->bssType);
            retCode = eSIR_SME_INVALID_PARAMETERS;
            goto end;
        }

        if (pSmeJoinReq->addIEScan.length)
        {
            vos_mem_copy( &psessionEntry->pLimJoinReq->addIEScan,
                          &pSmeJoinReq->addIEScan, sizeof(tSirAddie));
        }

        if (pSmeJoinReq->addIEAssoc.length)
        {
            vos_mem_copy( &psessionEntry->pLimJoinReq->addIEAssoc,
                          &pSmeJoinReq->addIEAssoc, sizeof(tSirAddie));
        }

        val = sizeof(tLimMlmJoinReq) + psessionEntry->pLimJoinReq->bssDescription.length + 2;
        pMlmJoinReq = vos_mem_malloc(val);
        if ( NULL == pMlmJoinReq )
        {
            limLog(pMac, LOGP, FL("call to AllocateMemory "
                                "failed for mlmJoinReq"));
            return;
        }
        (void) vos_mem_set((void *) pMlmJoinReq, val, 0);

        /* PE SessionId is stored as a part of JoinReq*/
        pMlmJoinReq->sessionId = psessionEntry->peSessionId;

        if (wlan_cfgGetInt(pMac, WNI_CFG_JOIN_FAILURE_TIMEOUT, (tANI_U32 *) &pMlmJoinReq->joinFailureTimeout)
            != eSIR_SUCCESS)
        {
            limLog(pMac, LOGP, FL("could not retrieve JoinFailureTimer value"
                    " setting to default value"));
            pMlmJoinReq->joinFailureTimeout =
                                WNI_CFG_JOIN_FAILURE_TIMEOUT_STADEF;
        }
        /* copy operational rate from psessionEntry*/
        vos_mem_copy((void*)&psessionEntry->rateSet, (void*)&pSmeJoinReq->operationalRateSet,
                            sizeof(tSirMacRateSet));
        vos_mem_copy((void*)&psessionEntry->extRateSet, (void*)&pSmeJoinReq->extendedRateSet,
                            sizeof(tSirMacRateSet));
        //this may not be needed anymore now, as rateSet is now included in the session entry and MLM has session context.
        vos_mem_copy((void*)&pMlmJoinReq->operationalRateSet, (void*)&psessionEntry->rateSet,
                           sizeof(tSirMacRateSet));

        psessionEntry->encryptType = pSmeJoinReq->UCEncryptionType;

        pMlmJoinReq->bssDescription.length = psessionEntry->pLimJoinReq->bssDescription.length;

        vos_mem_copy((tANI_U8 *) &pMlmJoinReq->bssDescription.bssId,
           (tANI_U8 *) &psessionEntry->pLimJoinReq->bssDescription.bssId,
           psessionEntry->pLimJoinReq->bssDescription.length + 2);

        psessionEntry->limCurrentBssCaps =
           psessionEntry->pLimJoinReq->bssDescription.capabilityInfo;

        regMax = cfgGetRegulatoryMaxTransmitPower( pMac, psessionEntry->currentOperChannel );

        if(!pMac->psOffloadEnabled)
        {
           limExtractApCapability( pMac,
              (tANI_U8 *) psessionEntry->pLimJoinReq->bssDescription.ieFields,
              limGetIElenFromBssDescription(
              &psessionEntry->pLimJoinReq->bssDescription),
              &psessionEntry->limCurrentBssQosCaps,
              &psessionEntry->limCurrentBssPropCap,
              &pMac->lim.gLimCurrentBssUapsd
              , &localPowerConstraint,
              psessionEntry
              );
        }
        else
        {
           limExtractApCapability( pMac,
              (tANI_U8 *) psessionEntry->pLimJoinReq->bssDescription.ieFields,
              limGetIElenFromBssDescription(
              &psessionEntry->pLimJoinReq->bssDescription),
              &psessionEntry->limCurrentBssQosCaps,
              &psessionEntry->limCurrentBssPropCap,
              &psessionEntry->gLimCurrentBssUapsd,
              &localPowerConstraint,
              psessionEntry
              );
        }

        /* If power constraint is zero then update it with Region max.
         * MaxTxpower will be the MIN of regmax and power constraint */
        if (localPowerConstraint == 0)
            localPowerConstraint = regMax;

#ifdef FEATURE_WLAN_ESE
            psessionEntry->maxTxPower = limGetMaxTxPower(regMax, localPowerConstraint, pMac->roam.configParam.nTxPowerCap);
#else
            psessionEntry->maxTxPower = VOS_MIN( regMax, (localPowerConstraint) );
#endif
        VOS_TRACE( VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO,
                        "Regulatory max = %d, local power constraint = %d,"
                        " max tx = %d", regMax, localPowerConstraint,
                        psessionEntry->maxTxPower );

        if(!pMac->psOffloadEnabled)
        {
            if (pMac->lim.gLimCurrentBssUapsd)
            {
                pMac->lim.gUapsdPerAcBitmask =
                          psessionEntry->pLimJoinReq->uapsdPerAcBitmask;
                limLog( pMac, LOG1,
                        FL("UAPSD flag for all AC - 0x%2x"),
                        pMac->lim.gUapsdPerAcBitmask);

                // resetting the dynamic uapsd mask
                pMac->lim.gUapsdPerAcDeliveryEnableMask = 0;
                pMac->lim.gUapsdPerAcTriggerEnableMask = 0;
            }
        }
        else
        {
            if (psessionEntry->gLimCurrentBssUapsd)
            {
                psessionEntry->gUapsdPerAcBitmask =
                               psessionEntry->pLimJoinReq->uapsdPerAcBitmask;
                limLog( pMac, LOG1,
                        FL("UAPSD flag for all AC - 0x%2x"),
                        psessionEntry->gUapsdPerAcBitmask);

                /* resetting the dynamic uapsd mask  */
                psessionEntry->gUapsdPerAcDeliveryEnableMask = 0;
                psessionEntry->gUapsdPerAcTriggerEnableMask = 0;
             }
        }

        psessionEntry->limRFBand = limGetRFBand(psessionEntry->currentOperChannel);

        // Initialize 11h Enable Flag
        if(SIR_BAND_5_GHZ == psessionEntry->limRFBand)
        {
            if (wlan_cfgGetInt(pMac, WNI_CFG_11H_ENABLED, &val) != eSIR_SUCCESS)
            {
                limLog(pMac, LOGP, FL("Fail to get WNI_CFG_11H_ENABLED "));
                psessionEntry->lim11hEnable = WNI_CFG_11H_ENABLED_STADEF;
            }
            else {
                psessionEntry->lim11hEnable = val;
            }
        }
        else
            psessionEntry->lim11hEnable = 0;

        //To care of the scenario when STA transitions from IBSS to Infrastructure mode.
        pMac->lim.gLimIbssCoalescingHappened = false;

            psessionEntry->limPrevSmeState = psessionEntry->limSmeState;
            psessionEntry->limSmeState = eLIM_SME_WT_JOIN_STATE;
            MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry->limSmeState));

        limLog(pMac, LOG1, FL("SME JoinReq:Sessionid %d SSID len %d SSID : %s "
        "Channel %d, BSSID "MAC_ADDRESS_STR), pMlmJoinReq->sessionId,
        psessionEntry->ssId.length,psessionEntry->ssId.ssId,
        psessionEntry->currentOperChannel,
        MAC_ADDR_ARRAY(psessionEntry->bssId));

        /* Indicate whether spectrum management is enabled*/
        psessionEntry->spectrumMgtEnabled =
           pSmeJoinReq->spectrumMgtIndicator;

        /* Enable the spectrum management if this is a DFS channel */
        if (psessionEntry->countryInfoPresent &&
             limIsconnectedOnDFSChannel(psessionEntry->currentOperChannel))
             psessionEntry->spectrumMgtEnabled = TRUE;

        psessionEntry->isOSENConnection =
           pSmeJoinReq->isOSENConnection;

           PELOG1(limLog(pMac,LOG1,FL("SessionId:%d MLM_JOIN_REQ is posted to MLM SM"),
                         pMlmJoinReq->sessionId));
        /* Issue LIM_MLM_JOIN_REQ to MLM */
        limPostMlmMessage(pMac, LIM_MLM_JOIN_REQ, (tANI_U32 *) pMlmJoinReq);
        return;

    }
    else
    {
        /* Received eWNI_SME_JOIN_REQ un expected state */
        limLog(pMac, LOGE, FL("received unexpected SME_JOIN_REQ "
                             "in state %X"), pMac->lim.gLimSmeState);
        retCode = eSIR_SME_UNEXPECTED_REQ_RESULT_CODE;
        psessionEntry = NULL;
        goto end;

    }

end:
    limGetSessionInfo(pMac,(tANI_U8*)pMsgBuf,&smesessionId,&smetransactionId);

    if(pSmeJoinReq)
    {
        vos_mem_free(pSmeJoinReq);
        pSmeJoinReq = NULL;
        if (NULL != psessionEntry)
        {
            psessionEntry->pLimJoinReq = NULL;
        }
    }

    if(retCode != eSIR_SME_SUCCESS)
    {
        if(NULL != psessionEntry)
        {
            peDeleteSession(pMac,psessionEntry);
            psessionEntry = NULL;
        }
    }
    limLog(pMac, LOG1, FL("Sending failure status limSendSmeJoinReassocRsp"
                       "on sessionid: %d with retCode = %d"),smesessionId, retCode);
    limSendSmeJoinReassocRsp(pMac, eWNI_SME_JOIN_RSP, retCode, eSIR_MAC_UNSPEC_FAILURE_STATUS,psessionEntry,smesessionId,smetransactionId);
} /*** end __limProcessSmeJoinReq() ***/


#if defined FEATURE_WLAN_ESE || defined WLAN_FEATURE_VOWIFI
tANI_U8 limGetMaxTxPower(tPowerdBm regMax, tPowerdBm apTxPower, tANI_U8 iniTxPower)
{
    tANI_U8 maxTxPower = 0;
    tANI_U8 txPower = VOS_MIN( regMax, (apTxPower) );
    txPower = VOS_MIN(txPower, iniTxPower);
    if((txPower >= MIN_TX_PWR_CAP) && (txPower <= MAX_TX_PWR_CAP))
        maxTxPower =  txPower;
    else if (txPower < MIN_TX_PWR_CAP)
        maxTxPower = MIN_TX_PWR_CAP;
    else
        maxTxPower = MAX_TX_PWR_CAP;

    return (maxTxPower);
}
#endif

/**
 * __limProcessSmeReassocReq()
 *
 *FUNCTION:
 * This function is called to process SME_REASSOC_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeReassocReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tANI_U16                caps;
    tANI_U32                val;
    tpSirSmeJoinReq    pReassocReq = NULL;
    tLimMlmReassocReq  *pMlmReassocReq;
    tSirResultCodes    retCode = eSIR_SME_SUCCESS;
    tpPESession        psessionEntry = NULL;
    tANI_U8            sessionId;
    tANI_U8            smeSessionId;
    tANI_U16           transactionId;
    tPowerdBm            localPowerConstraint = 0, regMax = 0;
    tANI_U32           teleBcnEn = 0;
    tANI_U16            nSize;


    PELOG3(limLog(pMac, LOG3, FL("Received REASSOC_REQ"));)

    nSize = __limGetSmeJoinReqSizeForAlloc((tANI_U8 *) pMsgBuf);
    pReassocReq = vos_mem_malloc(nSize);
    if ( NULL == pReassocReq )
    {
        // Log error
        limLog(pMac, LOGP,
               FL("call to AllocateMemory failed for pReassocReq"));

        retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
        goto end;
    }
    (void) vos_mem_set((void *) pReassocReq, nSize, 0);
    if ((limJoinReqSerDes(pMac, (tpSirSmeJoinReq) pReassocReq,
                          (tANI_U8 *) pMsgBuf) == eSIR_FAILURE) ||
        (!limIsSmeJoinReqValid(pMac,
                               (tpSirSmeJoinReq) pReassocReq)))
    {
        /// Received invalid eWNI_SME_REASSOC_REQ
        // Log the event
        limLog(pMac, LOGW,
               FL("received SME_REASSOC_REQ with invalid data"));

        retCode = eSIR_SME_INVALID_PARAMETERS;
        goto end;
    }

   if((psessionEntry = peFindSessionByBssid(pMac,pReassocReq->bssDescription.bssId,&sessionId))==NULL)
    {
        limLog(pMac, LOGE, FL("Session does not exist for given bssId"));
        limPrintMacAddr(pMac, pReassocReq->bssDescription.bssId, LOGE);
        retCode = eSIR_SME_INVALID_PARAMETERS;

       limGetSessionInfo(pMac,(tANI_U8*)pMsgBuf, &smeSessionId, &transactionId);
       psessionEntry = pe_find_session_by_sme_session_id(pMac, smeSessionId);

       if (psessionEntry != NULL)
                   limHandleSmeJoinResult(pMac, eSIR_SME_INVALID_PARAMETERS,
                                 eSIR_MAC_UNSPEC_FAILURE_STATUS, psessionEntry);
       goto end;
    }

#ifdef FEATURE_WLAN_DIAG_SUPPORT
    limDiagEventReport(pMac, WLAN_PE_DIAG_REASSOC_REQ_EVENT, psessionEntry,
                       eSIR_SUCCESS, eSIR_SUCCESS);
#endif
    //pMac->lim.gpLimReassocReq = pReassocReq;//TO SUPPORT BT-AMP

    /* Store the reassoc handle in the session Table.. 23rd sep review */
    psessionEntry->pLimReAssocReq = pReassocReq;

    psessionEntry->dot11mode = pReassocReq->dot11mode;
    psessionEntry->vhtCapability = IS_DOT11_MODE_VHT(pReassocReq->dot11mode);

    psessionEntry->enableHtSmps = pReassocReq->enableHtSmps;
    psessionEntry->htSmpsvalue = pReassocReq->htSmps;
    limLog(pMac, LOG1, FL("enableHtSmps: %d htSmps: %d supported nss 1x1: %d"),
           psessionEntry->enableHtSmps,
           psessionEntry->htSmpsvalue,
           psessionEntry->supported_nss_1x1);

    /**
     * Reassociate request is expected
     * in link established state only.
     */

    if (psessionEntry->limSmeState != eLIM_SME_LINK_EST_STATE)
    {
#if defined(WLAN_FEATURE_VOWIFI_11R) || defined(FEATURE_WLAN_ESE) || defined(FEATURE_WLAN_LFR)
        if (psessionEntry->limSmeState == eLIM_SME_WT_REASSOC_STATE)
        {
            // May be from 11r FT pre-auth. So lets check it before we bail out
            limLog(pMac, LOG1, FL("Session in reassoc state is %d"),
                psessionEntry->peSessionId);

            // Make sure its our preauth bssid
            if (!vos_mem_compare( pReassocReq->bssDescription.bssId,
                psessionEntry->limReAssocbssId, 6))
            {
                limPrintMacAddr(pMac, pReassocReq->bssDescription.bssId, LOGE);
                limLog(pMac, LOGP, FL("Unknown bssId in reassoc state"));
                retCode = eSIR_SME_INVALID_PARAMETERS;
                goto end;
            }

            limProcessMlmFTReassocReq(pMac, pMsgBuf, psessionEntry);
            return;
        }
#endif
        /// Should not have received eWNI_SME_REASSOC_REQ
        // Log the event
        limLog(pMac, LOGE,
               FL("received unexpected SME_REASSOC_REQ in state %X"),
               psessionEntry->limSmeState);

        retCode = eSIR_SME_UNEXPECTED_REQ_RESULT_CODE;
        goto end;
    }

    vos_mem_copy( psessionEntry->limReAssocbssId,
             psessionEntry->pLimReAssocReq->bssDescription.bssId,
             sizeof(tSirMacAddr));

    psessionEntry->limReassocChannelId =
         psessionEntry->pLimReAssocReq->bssDescription.channelId;

    psessionEntry->reAssocHtSupportedChannelWidthSet =
         (psessionEntry->pLimReAssocReq->cbMode)?1:0;
    psessionEntry->reAssocHtRecommendedTxWidthSet =
         psessionEntry->reAssocHtSupportedChannelWidthSet;
    psessionEntry->reAssocHtSecondaryChannelOffset =
         psessionEntry->pLimReAssocReq->cbMode;

    psessionEntry->limReassocBssCaps =
                psessionEntry->pLimReAssocReq->bssDescription.capabilityInfo;
    regMax = cfgGetRegulatoryMaxTransmitPower( pMac, psessionEntry->currentOperChannel );
    localPowerConstraint = regMax;

    if(!pMac->psOffloadEnabled)
    {
        limExtractApCapability( pMac,
            (tANI_U8 *) psessionEntry->pLimReAssocReq->bssDescription.ieFields,
            limGetIElenFromBssDescription(
                     &psessionEntry->pLimReAssocReq->bssDescription),
            &psessionEntry->limReassocBssQosCaps,
            &psessionEntry->limReassocBssPropCap,
            &pMac->lim.gLimCurrentBssUapsd
            , &localPowerConstraint,
            psessionEntry
            );
    }
    else
    {
        limExtractApCapability(pMac,
            (tANI_U8 *) psessionEntry->pLimReAssocReq->bssDescription.ieFields,
            limGetIElenFromBssDescription(
                     &psessionEntry->pLimReAssocReq->bssDescription),
            &psessionEntry->limReassocBssQosCaps,
            &psessionEntry->limReassocBssPropCap,
            &psessionEntry->gLimCurrentBssUapsd,
            &localPowerConstraint,
            psessionEntry);
    }

    psessionEntry->maxTxPower = VOS_MIN( regMax, (localPowerConstraint) );
#if defined WLAN_VOWIFI_DEBUG
            limLog( pMac, LOGE, "Regulatory max = %d, local power constraint "
                        "= %d, max tx = %d", regMax, localPowerConstraint,
                          psessionEntry->maxTxPower );
#endif
    {

    /* Copy the SSID from session entry to local variable */
    psessionEntry->limReassocSSID.length = pReassocReq->ssId.length;
    vos_mem_copy(psessionEntry->limReassocSSID.ssId,
                 pReassocReq->ssId.ssId, psessionEntry->limReassocSSID.length);

    }

    if(!pMac->psOffloadEnabled)
    {
        if (pMac->lim.gLimCurrentBssUapsd)
        {
            pMac->lim.gUapsdPerAcBitmask =
                      psessionEntry->pLimReAssocReq->uapsdPerAcBitmask;
            limLog( pMac, LOG1,
                    FL("UAPSD flag for all AC - 0x%2x"),
                    pMac->lim.gUapsdPerAcBitmask);
        }
    }
    else
    {
        if(psessionEntry->gLimCurrentBssUapsd)
        {
            psessionEntry->gUapsdPerAcBitmask =
                           psessionEntry->pLimReAssocReq->uapsdPerAcBitmask;
            limLog( pMac, LOG1,
                    FL("UAPSD flag for all AC - 0x%2x"),
                    psessionEntry->gUapsdPerAcBitmask);
        }
    }

    pMlmReassocReq = vos_mem_malloc(sizeof(tLimMlmReassocReq));
    if ( NULL == pMlmReassocReq )
    {
        // Log error
        limLog(pMac, LOGP,
               FL("call to AllocateMemory failed for mlmReassocReq"));

        retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
        goto end;
    }

    vos_mem_copy( pMlmReassocReq->peerMacAddr,
                  psessionEntry->limReAssocbssId,
                  sizeof(tSirMacAddr));

    if (wlan_cfgGetInt(pMac, WNI_CFG_REASSOCIATION_FAILURE_TIMEOUT,
                  (tANI_U32 *) &pMlmReassocReq->reassocFailureTimeout)
                           != eSIR_SUCCESS)
    {
        /**
         * Could not get ReassocFailureTimeout value
         * from CFG. Log error.
         */
        limLog(pMac, LOGP,
               FL("could not retrieve ReassocFailureTimeout value"));
    }

    if (cfgGetCapabilityInfo(pMac, &caps,psessionEntry) != eSIR_SUCCESS)
    {
        /**
         * Could not get Capabilities value
         * from CFG. Log error.
         */
        limLog(pMac, LOGP,
               FL("could not retrieve Capabilities value"));
    }

    lim_update_caps_info_for_bss(pMac, &caps,
                        pReassocReq->bssDescription.capabilityInfo);

    limLog(pMac, LOG1, FL("Capabilities info Reassoc: 0x%X"), caps);

    pMlmReassocReq->capabilityInfo = caps;

    /* Update PE sessionId*/
    pMlmReassocReq->sessionId = sessionId;

   /* If telescopic beaconing is enabled, set listen interval to
     WNI_CFG_TELE_BCN_MAX_LI */
    if(wlan_cfgGetInt(pMac, WNI_CFG_TELE_BCN_WAKEUP_EN, &teleBcnEn) !=
       eSIR_SUCCESS)
       limLog(pMac, LOGP, FL("Couldn't get WNI_CFG_TELE_BCN_WAKEUP_EN"));

    val = WNI_CFG_LISTEN_INTERVAL_STADEF;

    if(teleBcnEn)
    {
       if(wlan_cfgGetInt(pMac, WNI_CFG_TELE_BCN_MAX_LI, &val) !=
          eSIR_SUCCESS)
       {
            /**
            * Could not get ListenInterval value
            * from CFG. Log error.
          */
          limLog(pMac, LOGP, FL("could not retrieve ListenInterval"));
       }
    }
    else
    {
       if (wlan_cfgGetInt(pMac, WNI_CFG_LISTEN_INTERVAL, &val) != eSIR_SUCCESS)
       {
         /**
            * Could not get ListenInterval value
            * from CFG. Log error.
          */
          limLog(pMac, LOGP, FL("could not retrieve ListenInterval"));
       }
    }

    /* Delete all BA sessions before Re-Assoc.
     *  BA frames are class 3 frames and the session
     *  is lost upon disassociation and reassociation.
     */

    limDeleteBASessions(pMac, psessionEntry, BA_BOTH_DIRECTIONS);

    pMlmReassocReq->listenInterval = (tANI_U16) val;

    /* Indicate whether spectrum management is enabled*/
    psessionEntry->spectrumMgtEnabled = pReassocReq->spectrumMgtIndicator;

    /* Enable the spectrum management if this is a DFS channel */
    if (psessionEntry->countryInfoPresent &&
             limIsconnectedOnDFSChannel(psessionEntry->currentOperChannel))
             psessionEntry->spectrumMgtEnabled = TRUE;

    psessionEntry->limPrevSmeState = psessionEntry->limSmeState;
    psessionEntry->limSmeState    = eLIM_SME_WT_REASSOC_STATE;

    MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry->limSmeState));

    limPostMlmMessage(pMac,
                      LIM_MLM_REASSOC_REQ,
                      (tANI_U32 *) pMlmReassocReq);
    return;

end:
    if (pReassocReq) {
        vos_mem_free( pReassocReq);
        if (psessionEntry)
            psessionEntry->pLimReAssocReq = NULL;
    }

    if (psessionEntry)
    {
       // error occurred after we determined the session so extract
       // session and transaction info from there
       smeSessionId = psessionEntry->smeSessionId;
       transactionId = psessionEntry->transactionId;
    }
    else
    {
       // error occurred before or during the time we determined the session
       // so extract the session and transaction info from the message
       limGetSessionInfo(pMac,(tANI_U8*)pMsgBuf, &smeSessionId, &transactionId);
    }

    /// Send Reassoc failure response to host
    /// (note psessionEntry may be NULL, but that's OK)
    limSendSmeJoinReassocRsp(pMac, eWNI_SME_REASSOC_RSP,
                             retCode, eSIR_MAC_UNSPEC_FAILURE_STATUS,
                             psessionEntry, smeSessionId, transactionId);

} /*** end __limProcessSmeReassocReq() ***/


tANI_BOOLEAN sendDisassocFrame = 1;
/**
 * __limProcessSmeDisassocReq()
 *
 *FUNCTION:
 * This function is called to process SME_DISASSOC_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeDisassocReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tANI_U16                disassocTrigger, reasonCode;
    tLimMlmDisassocReq      *pMlmDisassocReq;
    tSirResultCodes         retCode = eSIR_SME_SUCCESS;
    tSirRetStatus           status;
    tSirSmeDisassocReq      smeDisassocReq;
    tpPESession             psessionEntry = NULL;
    tANI_U8                 sessionId;
    tANI_U8                 smesessionId;
    tANI_U16                smetransactionId;

    if (pMsgBuf == NULL)
    {
        limLog(pMac, LOGE, FL("Buffer is Pointing to NULL"));
        return;
    }

    limGetSessionInfo(pMac, (tANI_U8 *)pMsgBuf,&smesessionId, &smetransactionId);

    status = limDisassocReqSerDes(pMac, &smeDisassocReq, (tANI_U8 *) pMsgBuf);

    if ( (eSIR_FAILURE == status) ||
         (!limIsSmeDisassocReqValid(pMac, &smeDisassocReq, psessionEntry)) )
    {
        PELOGE(limLog(pMac, LOGE,
               FL("received invalid SME_DISASSOC_REQ message"));)

        if (pMac->lim.gLimRspReqd)
        {
            pMac->lim.gLimRspReqd = false;

            retCode         = eSIR_SME_INVALID_PARAMETERS;
            disassocTrigger = eLIM_HOST_DISASSOC;
            goto sendDisassoc;
        }

        return;
    }

    if((psessionEntry = peFindSessionByBssid(pMac,smeDisassocReq.bssId,&sessionId))== NULL)
    {
        limLog(pMac, LOGE,FL("session does not exist for given bssId "MAC_ADDRESS_STR),
                          MAC_ADDR_ARRAY(smeDisassocReq.bssId));
        retCode = eSIR_SME_INVALID_PARAMETERS;
        disassocTrigger = eLIM_HOST_DISASSOC;
        goto sendDisassoc;

    }
    limLog(pMac, LOG1, FL("received DISASSOC_REQ message on sessionid %d "
          "Systemrole %d Reason: %u SmeState: %d from: "MAC_ADDRESS_STR),
          smesessionId, GET_LIM_SYSTEM_ROLE(psessionEntry),
          smeDisassocReq.reasonCode, pMac->lim.gLimSmeState,
          MAC_ADDR_ARRAY(smeDisassocReq.peerMacAddr));

#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
   limDiagEventReport(pMac, WLAN_PE_DIAG_DISASSOC_REQ_EVENT, psessionEntry, 0, smeDisassocReq.reasonCode);
#endif //FEATURE_WLAN_DIAG_SUPPORT

    /* Update SME session Id and SME transaction ID*/

    psessionEntry->smeSessionId = smesessionId;
    psessionEntry->transactionId = smetransactionId;

    switch (GET_LIM_SYSTEM_ROLE(psessionEntry))
    {
        case eLIM_STA_ROLE:
        case eLIM_BT_AMP_STA_ROLE:
            switch (psessionEntry->limSmeState)
            {
                case eLIM_SME_ASSOCIATED_STATE:
                case eLIM_SME_LINK_EST_STATE:
                    limLog(pMac, LOG1, FL("Rcvd SME_DISASSOC_REQ in limSmeState: %d"),
                                       psessionEntry->limSmeState);
                    psessionEntry->limPrevSmeState = psessionEntry->limSmeState;
                    psessionEntry->limSmeState= eLIM_SME_WT_DISASSOC_STATE;
#ifdef FEATURE_WLAN_TDLS
                    /* Delete all TDLS peers connected before leaving BSS*/
                    limDeleteTDLSPeers(pMac, psessionEntry);
#endif
                    MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry->limSmeState));
                    break;

                case eLIM_SME_WT_DEAUTH_STATE:
                    /* PE shall still process the DISASSOC_REQ and proceed with
                     * link tear down even if it had already sent a DEAUTH_IND to
                     * to SME. pMac->lim.gLimPrevSmeState shall remain the same as
                     * its been set when PE entered WT_DEAUTH_STATE.
                     */
                    psessionEntry->limSmeState= eLIM_SME_WT_DISASSOC_STATE;
                    MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry->limSmeState));
                    limLog(pMac, LOG1, FL("Rcvd SME_DISASSOC_REQ while in "
                       "SME_WT_DEAUTH_STATE. "));
                    break;

                case eLIM_SME_WT_DISASSOC_STATE:
                    /* PE Recieved a Disassoc frame. Normally it gets DISASSOC_CNF but it
                     * received DISASSOC_REQ. Which means host is also trying to disconnect.
                     * PE can continue processing DISASSOC_REQ and send the response instead
                     * of failing the request. SME will anyway ignore DEAUTH_IND that was sent
                     * for disassoc frame.
                     *
                     * It will send a disassoc, which is ok. However, we can use the global flag
                     * sendDisassoc to not send disassoc frame.
                     */
                    limLog(pMac, LOG1, FL("Rcvd SME_DISASSOC_REQ while in "
                       "SME_WT_DISASSOC_STATE. "));
                    break;

                case eLIM_SME_JOIN_FAILURE_STATE: {
                    /** Return Success as we are already in Disconnected State*/
                    limLog(pMac, LOG1, FL("Rcvd SME_DISASSOC_REQ while in "
                       "eLIM_SME_JOIN_FAILURE_STATE. "));
                     if (pMac->lim.gLimRspReqd) {
                        retCode = eSIR_SME_SUCCESS;
                        disassocTrigger = eLIM_HOST_DISASSOC;
                        goto sendDisassoc;
                    }
                }break;
                default:
                    /**
                     * STA is not currently associated.
                     * Log error and send response to host
                     */
                    limLog(pMac, LOGE,
                       FL("received unexpected SME_DISASSOC_REQ in state %X"),
                       psessionEntry->limSmeState);

                    if (pMac->lim.gLimRspReqd)
                    {
                        if (psessionEntry->limSmeState !=
                                                eLIM_SME_WT_ASSOC_STATE)
                                    pMac->lim.gLimRspReqd = false;

                        retCode = eSIR_SME_UNEXPECTED_REQ_RESULT_CODE;
                        disassocTrigger = eLIM_HOST_DISASSOC;
                        goto sendDisassoc;
                    }

                    return;
            }

            break;

        case eLIM_AP_ROLE:
    case eLIM_BT_AMP_AP_ROLE:
            // Fall through
            break;

        case eLIM_STA_IN_IBSS_ROLE:
        default: // eLIM_UNKNOWN_ROLE
            limLog(pMac, LOGE,
               FL("received unexpected SME_DISASSOC_REQ for role %d"),
               GET_LIM_SYSTEM_ROLE(psessionEntry));

            retCode = eSIR_SME_UNEXPECTED_REQ_RESULT_CODE;
            disassocTrigger = eLIM_HOST_DISASSOC;
            goto sendDisassoc;
    } // end switch (pMac->lim.gLimSystemRole)

    disassocTrigger = eLIM_HOST_DISASSOC;
    reasonCode      = smeDisassocReq.reasonCode;

    if (smeDisassocReq.doNotSendOverTheAir)
    {
        limLog(pMac, LOG1, FL("do not send dissoc over the air"));
        sendDisassocFrame = 0;
    }
    // Trigger Disassociation frame to peer MAC entity
    limLog(pMac, LOG1, FL("Sending Disasscoc with disassoc Trigger"
                          " : %d, reasonCode : %d"),
                          disassocTrigger, reasonCode);

    pMlmDisassocReq = vos_mem_malloc(sizeof(tLimMlmDisassocReq));
    if ( NULL == pMlmDisassocReq )
    {
        // Log error
        limLog(pMac, LOGP,
               FL("call to AllocateMemory failed for mlmDisassocReq"));

        return;
    }

    vos_mem_copy( (tANI_U8 *) &pMlmDisassocReq->peerMacAddr,
                  (tANI_U8 *) &smeDisassocReq.peerMacAddr,
                  sizeof(tSirMacAddr));

    pMlmDisassocReq->reasonCode      = reasonCode;
    pMlmDisassocReq->disassocTrigger = disassocTrigger;

    /* Update PE session ID*/
    pMlmDisassocReq->sessionId = sessionId;

    limPostMlmMessage(pMac,
                      LIM_MLM_DISASSOC_REQ,
                      (tANI_U32 *) pMlmDisassocReq);
    return;

sendDisassoc:
    if (psessionEntry)
        limSendSmeDisassocNtf(pMac, smeDisassocReq.peerMacAddr,
                          retCode,
                          disassocTrigger,
                          1,smesessionId,smetransactionId,psessionEntry);
    else
        limSendSmeDisassocNtf(pMac, smeDisassocReq.peerMacAddr,
                retCode,
                disassocTrigger,
                1, smesessionId, smetransactionId, NULL);


} /*** end __limProcessSmeDisassocReq() ***/


/** -----------------------------------------------------------------
  \brief __limProcessSmeDisassocCnf() - Process SME_DISASSOC_CNF

  This function is called to process SME_DISASSOC_CNF message
  from HDD or upper layer application.

  \param pMac - global mac structure
  \param pStaDs - station dph hash node
  \return none
  \sa
  ----------------------------------------------------------------- */
static void
__limProcessSmeDisassocCnf(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirSmeDisassocCnf  smeDisassocCnf;
    tANI_U16  aid;
    tpDphHashNode  pStaDs;
    tSirRetStatus  status = eSIR_SUCCESS;
    tpPESession         psessionEntry;
    tANI_U8             sessionId;


    PELOG1(limLog(pMac, LOG1, FL("received DISASSOC_CNF message"));)

    status = limDisassocCnfSerDes(pMac, &smeDisassocCnf,(tANI_U8 *) pMsgBuf);

    if (status == eSIR_FAILURE)
    {
        PELOGE(limLog(pMac, LOGE, FL("invalid SME_DISASSOC_CNF message"));)
        return;
    }

    if((psessionEntry = peFindSessionByBssid(pMac, smeDisassocCnf.bssId, &sessionId))== NULL)
    {
         limLog(pMac, LOGE,FL("session does not exist for given bssId"));
         return;
    }

    if (!limIsSmeDisassocCnfValid(pMac, &smeDisassocCnf, psessionEntry))
    {
        limLog(pMac, LOGE, FL("received invalid SME_DISASSOC_CNF message"));
        return;
    }

#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    if (smeDisassocCnf.messageType == eWNI_SME_DISASSOC_CNF)
        limDiagEventReport(pMac, WLAN_PE_DIAG_DISASSOC_CNF_EVENT, psessionEntry, (tANI_U16)smeDisassocCnf.statusCode, 0);
    else if (smeDisassocCnf.messageType ==  eWNI_SME_DEAUTH_CNF)
        limDiagEventReport(pMac, WLAN_PE_DIAG_DEAUTH_CNF_EVENT, psessionEntry, (tANI_U16)smeDisassocCnf.statusCode, 0);
#endif //FEATURE_WLAN_DIAG_SUPPORT

    switch (GET_LIM_SYSTEM_ROLE(psessionEntry))
    {
        case eLIM_STA_ROLE:
        case eLIM_BT_AMP_STA_ROLE:  //To test reconn
            if ((psessionEntry->limSmeState != eLIM_SME_IDLE_STATE) &&
                (psessionEntry->limSmeState != eLIM_SME_WT_DISASSOC_STATE) &&
                (psessionEntry->limSmeState != eLIM_SME_WT_DEAUTH_STATE))
            {
                limLog(pMac, LOGE,
                   FL("received unexp SME_DISASSOC_CNF in state %X"),
                  psessionEntry->limSmeState);
                return;
            }
            break;

        case eLIM_AP_ROLE:
            // Fall through
            break;

        case eLIM_STA_IN_IBSS_ROLE:
        default: // eLIM_UNKNOWN_ROLE
            limLog(pMac, LOGE,
               FL("received unexpected SME_DISASSOC_CNF role %d"),
               GET_LIM_SYSTEM_ROLE(psessionEntry));

            return;
    }

    if ((psessionEntry->limSmeState == eLIM_SME_WT_DISASSOC_STATE) ||
        (psessionEntry->limSmeState == eLIM_SME_WT_DEAUTH_STATE) ||
        LIM_IS_AP_ROLE(psessionEntry)) {
        pStaDs = dphLookupHashEntry(pMac, smeDisassocCnf.peerMacAddr, &aid, &psessionEntry->dph.dphHashTable);
        if (pStaDs == NULL)
        {
            PELOGE(limLog(pMac, LOGE, FL("received DISASSOC_CNF for a STA that "
               "does not have context, addr= "MAC_ADDRESS_STR),
                     MAC_ADDR_ARRAY(smeDisassocCnf.peerMacAddr));)
            return;
        }

        /*
         * If MlM state is either of del_sta or del_bss state, then no need to
         * go ahead and clean up further as there must be some cleanup in
         * progress from upper layer disassoc/deauth request.
         */
        if((pStaDs->mlmStaContext.mlmState == eLIM_MLM_WT_DEL_STA_RSP_STATE) ||
           (pStaDs->mlmStaContext.mlmState == eLIM_MLM_WT_DEL_BSS_RSP_STATE)) {
            limLog(pMac, LOGE, FL("No need to cleanup for addr:"MAC_ADDRESS_STR
                   "as Mlm state is %d"),
                   MAC_ADDR_ARRAY(smeDisassocCnf.peerMacAddr),
                   pStaDs->mlmStaContext.mlmState);
           return;
        }

#if defined WLAN_FEATURE_VOWIFI_11R
        /* Delete FT session if there exists one */
        limFTCleanupPreAuthInfo(pMac, psessionEntry);
#endif
        limCleanupRxPath(pMac, pStaDs, psessionEntry);

        limCleanUpDisassocDeauthReq(pMac, (char*)&smeDisassocCnf.peerMacAddr, 0);
    }

    return;
}


/**
 * __limProcessSmeDeauthReq()
 *
 *FUNCTION:
 * This function is called to process SME_DEAUTH_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeDeauthReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tANI_U16                deauthTrigger, reasonCode;
    tLimMlmDeauthReq        *pMlmDeauthReq;
    tSirSmeDeauthReq        smeDeauthReq;
    tSirResultCodes         retCode = eSIR_SME_SUCCESS;
    tSirRetStatus           status = eSIR_SUCCESS;
    tpPESession             psessionEntry;
    tANI_U8                 sessionId; //PE sessionId
    tANI_U8                 smesessionId;
    tANI_U16                smetransactionId;

    PELOG1(limLog(pMac, LOG1,FL("received DEAUTH_REQ message"));)

    status = limDeauthReqSerDes(pMac, &smeDeauthReq,(tANI_U8 *) pMsgBuf);

    limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&smesessionId,&smetransactionId);

    //We need to get a session first but we don't even know if the message is correct.
    if((psessionEntry = peFindSessionByBssid(pMac, smeDeauthReq.bssId, &sessionId)) == NULL)
    {
       limLog(pMac, LOGE,FL("session does not exist for given bssId"));
       retCode = eSIR_SME_INVALID_PARAMETERS;
       deauthTrigger = eLIM_HOST_DEAUTH;
       goto sendDeauth;

    }

    if ((status == eSIR_FAILURE) || (!limIsSmeDeauthReqValid(pMac, &smeDeauthReq, psessionEntry)))
    {
        PELOGE(limLog(pMac, LOGE,FL
                   ("received invalid SME_DEAUTH_REQ message"));)
        pMac->lim.gLimRspReqd = false;

        retCode       = eSIR_SME_INVALID_PARAMETERS;
        deauthTrigger = eLIM_HOST_DEAUTH;
        goto sendDeauth;
    }
    limLog(pMac, LOG1,FL("received DEAUTH_REQ message on sessionid %d "
      "Systemrole %d with reasoncode %u in limSmestate %d from "
      MAC_ADDRESS_STR), smesessionId, GET_LIM_SYSTEM_ROLE(psessionEntry),
      smeDeauthReq.reasonCode, psessionEntry->limSmeState,
      MAC_ADDR_ARRAY(smeDeauthReq.peerMacAddr));
#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    limDiagEventReport(pMac, WLAN_PE_DIAG_DEAUTH_REQ_EVENT, psessionEntry, 0, smeDeauthReq.reasonCode);
#endif //FEATURE_WLAN_DIAG_SUPPORT

    /* Update SME session ID and Transaction ID */
    psessionEntry->smeSessionId = smesessionId;
    psessionEntry->transactionId = smetransactionId;


    switch (GET_LIM_SYSTEM_ROLE(psessionEntry))
    {
        case eLIM_STA_ROLE:
        case eLIM_BT_AMP_STA_ROLE:

            switch (psessionEntry->limSmeState)
            {
                case eLIM_SME_ASSOCIATED_STATE:
                case eLIM_SME_LINK_EST_STATE:
                case eLIM_SME_WT_ASSOC_STATE:
                case eLIM_SME_JOIN_FAILURE_STATE:
                case eLIM_SME_IDLE_STATE:
                    psessionEntry->limPrevSmeState = psessionEntry->limSmeState;
                    psessionEntry->limSmeState = eLIM_SME_WT_DEAUTH_STATE;
                    MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry->limSmeState));

                    // Send Deauthentication request to MLM below

                    break;
                case eLIM_SME_WT_DEAUTH_STATE:
                case eLIM_SME_WT_DISASSOC_STATE:
                    /*
                     * PE Recieved a Deauth/Disassoc frame. Normally it gets
                     * DEAUTH_CNF/DISASSOC_CNF but it received DEAUTH_REQ. Which
                     * means host is also trying to disconnect.
                     * PE can continue processing DEAUTH_REQ and send
                     * the response instead of failing the request.
                     * SME will anyway ignore DEAUTH_IND/DISASSOC_IND that
                     * was sent for deauth/disassoc frame.
                     */
                    psessionEntry->limSmeState = eLIM_SME_WT_DEAUTH_STATE;
                    limLog(pMac, LOG1, FL("Rcvd SME_DEAUTH_REQ while in "
                       "SME_WT_DEAUTH_STATE. "));
                    break;
                default:
                    /**
                     * STA is not in a state to deauthenticate with
                     * peer. Log error and send response to host.
                     */
                    limLog(pMac, LOGE,
                    FL("received unexp SME_DEAUTH_REQ in state %X"),
                    psessionEntry->limSmeState);

                    if (pMac->lim.gLimRspReqd)
                    {
                        pMac->lim.gLimRspReqd = false;

                        retCode       = eSIR_SME_STA_NOT_AUTHENTICATED;
                        deauthTrigger = eLIM_HOST_DEAUTH;

/*
               here we received deauth request from AP so sme state is
               eLIM_SME_WT_DEAUTH_STATE.if we have ISSUED delSta then
               mlm state should be eLIM_MLM_WT_DEL_STA_RSP_STATE and if
               we got delBSS rsp then mlm state should be eLIM_MLM_IDLE_STATE
               so the below condition captures the state where delSta
               not done and firmware still in connected state.
*/

                        if (psessionEntry->limSmeState == eLIM_SME_WT_DEAUTH_STATE &&
                            psessionEntry->limMlmState != eLIM_MLM_IDLE_STATE &&
                            psessionEntry->limMlmState != eLIM_MLM_WT_DEL_STA_RSP_STATE)
                        {
                            retCode = eSIR_SME_DEAUTH_STATUS;
                        }
                        goto sendDeauth;
                    }

                    return;
            }

            break;

        case eLIM_STA_IN_IBSS_ROLE:
            limLog(pMac, LOGE,FL("Deauth not allowed in IBSS"));
            if (pMac->lim.gLimRspReqd)
            {
                   pMac->lim.gLimRspReqd = false;
                   retCode = eSIR_SME_INVALID_PARAMETERS;
                   deauthTrigger = eLIM_HOST_DEAUTH;
                   goto sendDeauth;
            }
            return;

        case eLIM_AP_ROLE:
            // Fall through

            break;

        default:
            limLog(pMac, LOGE,
               FL("received unexpected SME_DEAUTH_REQ for role %X"),
               GET_LIM_SYSTEM_ROLE(psessionEntry));
            if (pMac->lim.gLimRspReqd)
            {
                   pMac->lim.gLimRspReqd = false;
                   retCode = eSIR_SME_INVALID_PARAMETERS;
                   deauthTrigger = eLIM_HOST_DEAUTH;
                   goto sendDeauth;
            }
            return;
    } // end switch (pMac->lim.gLimSystemRole)

    if (smeDeauthReq.reasonCode == eLIM_LINK_MONITORING_DEAUTH)
    {
        /// Deauthentication is triggered by Link Monitoring
        PELOG1(limLog(pMac, LOG1, FL("**** Lost link with AP ****"));)
        deauthTrigger = eLIM_LINK_MONITORING_DEAUTH;
        reasonCode    = eSIR_MAC_UNSPEC_FAILURE_REASON;
    }
    else
    {
        deauthTrigger = eLIM_HOST_DEAUTH;
        reasonCode    = smeDeauthReq.reasonCode;
    }

    // Trigger Deauthentication frame to peer MAC entity
    pMlmDeauthReq = vos_mem_malloc(sizeof(tLimMlmDeauthReq));
    if ( NULL == pMlmDeauthReq )
    {
        // Log error
        limLog(pMac, LOGE,
               FL("call to AllocateMemory failed for mlmDeauthReq"));
        if (pMac->lim.gLimRspReqd)
        {
            pMac->lim.gLimRspReqd = false;
            retCode = eSIR_SME_RESOURCES_UNAVAILABLE;
            deauthTrigger = eLIM_HOST_DEAUTH;
            goto sendDeauth;
        }
        return;
    }

    vos_mem_copy( (tANI_U8 *) &pMlmDeauthReq->peerMacAddr,
                  (tANI_U8 *) &smeDeauthReq.peerMacAddr,
                  sizeof(tSirMacAddr));

    pMlmDeauthReq->reasonCode = reasonCode;
    pMlmDeauthReq->deauthTrigger = deauthTrigger;

    /* Update PE session Id*/
    pMlmDeauthReq->sessionId = sessionId;

    limPostMlmMessage(pMac,
                      LIM_MLM_DEAUTH_REQ,
                      (tANI_U32 *) pMlmDeauthReq);
    return;

sendDeauth:
    limSendSmeDeauthNtf(pMac, smeDeauthReq.peerMacAddr,
                        retCode,
                        deauthTrigger,
                        1,
                        smesessionId, smetransactionId);
} /*** end __limProcessSmeDeauthReq() ***/



/**
 * __limProcessSmeSetContextReq()
 *
 *FUNCTION:
 * This function is called to process SME_SETCONTEXT_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeSetContextReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirSmeSetContextReq  pSetContextReq;
    tLimMlmSetKeysReq      *pMlmSetKeysReq;
    tpPESession             psessionEntry;
    tANI_U8                 sessionId;  //PE sessionID
    tANI_U8                 smesessionId;
    tANI_U16                smetransactionId;


    PELOG1(limLog(pMac, LOG1,
           FL("received SETCONTEXT_REQ message")););


    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&smesessionId,&smetransactionId);

    pSetContextReq = vos_mem_malloc(sizeof(tSirKeys) * SIR_MAC_MAX_NUM_OF_DEFAULT_KEYS);
    if ( NULL == pSetContextReq )
    {
        limLog(pMac, LOGP, FL("call to AllocateMemory failed for pSetContextReq"));
        return;
    }

    if ((limSetContextReqSerDes(pMac, pSetContextReq, (tANI_U8 *) pMsgBuf) == eSIR_FAILURE) ||
        (!limIsSmeSetContextReqValid(pMac, pSetContextReq)))
    {
        limLog(pMac, LOGW, FL("received invalid SME_SETCONTEXT_REQ message"));
        goto end;
    }

    if(pSetContextReq->keyMaterial.numKeys > SIR_MAC_MAX_NUM_OF_DEFAULT_KEYS)
    {
        PELOGE(limLog(pMac, LOGE, FL("numKeys:%d is more than SIR_MAC_MAX_NUM_OF_DEFAULT_KEYS"), pSetContextReq->keyMaterial.numKeys);)
        limSendSmeSetContextRsp(pMac,
                                pSetContextReq->peerMacAddr,
                                1,
                                eSIR_SME_INVALID_PARAMETERS,NULL,
                                smesessionId,smetransactionId);

        goto end;
    }


    if((psessionEntry = peFindSessionByBssid(pMac, pSetContextReq->bssId, &sessionId)) == NULL)
    {
        limLog(pMac, LOGW, FL("Session does not exist for given BSSID"));
        limSendSmeSetContextRsp(pMac,
                                pSetContextReq->peerMacAddr,
                                1,
                                eSIR_SME_INVALID_PARAMETERS,NULL,
                                smesessionId,smetransactionId);

        goto end;
    }

#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    limDiagEventReport(pMac, WLAN_PE_DIAG_SETCONTEXT_REQ_EVENT, psessionEntry, 0, 0);
#endif //FEATURE_WLAN_DIAG_SUPPORT


    if (((LIM_IS_STA_ROLE(psessionEntry) ||
          LIM_IS_BT_AMP_STA_ROLE(psessionEntry)) &&
         (psessionEntry->limSmeState == eLIM_SME_LINK_EST_STATE)) ||
        ((LIM_IS_IBSS_ROLE(psessionEntry) ||
          LIM_IS_AP_ROLE(psessionEntry) ||
          LIM_IS_BT_AMP_AP_ROLE(psessionEntry)) &&
         (psessionEntry->limSmeState == eLIM_SME_NORMAL_STATE))) {
        // Trigger MLM_SETKEYS_REQ
        pMlmSetKeysReq = vos_mem_malloc(sizeof(tLimMlmSetKeysReq));
        if ( NULL == pMlmSetKeysReq )
        {
            // Log error
            limLog(pMac, LOGP, FL("call to AllocateMemory failed for mlmSetKeysReq"));
            goto end;
        }

        pMlmSetKeysReq->edType  = pSetContextReq->keyMaterial.edType;
        pMlmSetKeysReq->numKeys = pSetContextReq->keyMaterial.numKeys;
        if(pMlmSetKeysReq->numKeys > SIR_MAC_MAX_NUM_OF_DEFAULT_KEYS)
        {
            limLog(pMac, LOGP, FL("Num of keys exceeded max num of default keys limit"));
            goto end;
        }
        vos_mem_copy( (tANI_U8 *) &pMlmSetKeysReq->peerMacAddr,
                      (tANI_U8 *) &pSetContextReq->peerMacAddr,
                      sizeof(tSirMacAddr));


        vos_mem_copy( (tANI_U8 *) &pMlmSetKeysReq->key,
                      (tANI_U8 *) &pSetContextReq->keyMaterial.key,
                      sizeof(tSirKeys) * (pMlmSetKeysReq->numKeys ? pMlmSetKeysReq->numKeys : 1));

        pMlmSetKeysReq->sessionId = sessionId;
        pMlmSetKeysReq->smesessionId = smesessionId;
#ifdef WLAN_FEATURE_VOWIFI_11R_DEBUG
        PELOG1(limLog(pMac, LOG1,
           FL("received SETCONTEXT_REQ message sessionId=%d"), pMlmSetKeysReq->sessionId););
#endif

        if (((pSetContextReq->keyMaterial.edType == eSIR_ED_WEP40) ||
            (pSetContextReq->keyMaterial.edType == eSIR_ED_WEP104)) &&
            LIM_IS_AP_ROLE(psessionEntry)) {
            if(pSetContextReq->keyMaterial.key[0].keyLength)
            {
                tANI_U8 keyId;
                keyId = pSetContextReq->keyMaterial.key[0].keyId;
                vos_mem_copy( (tANI_U8 *)&psessionEntry->WEPKeyMaterial[keyId],
                   (tANI_U8 *) &pSetContextReq->keyMaterial, sizeof(tSirKeyMaterial));
            }
            else {
                tANI_U32 i;
                for( i = 0; i < SIR_MAC_MAX_NUM_OF_DEFAULT_KEYS; i++)
                {
                    vos_mem_copy( (tANI_U8 *) &pMlmSetKeysReq->key[i],
                        (tANI_U8 *)psessionEntry->WEPKeyMaterial[i].key, sizeof(tSirKeys));
                }
            }
        }

        limPostMlmMessage(pMac, LIM_MLM_SETKEYS_REQ, (tANI_U32 *) pMlmSetKeysReq);
    }
    else
    {
        limLog(pMac, LOGE,
           FL("received unexpected SME_SETCONTEXT_REQ for role %d, state=%X"),
           GET_LIM_SYSTEM_ROLE(psessionEntry),
           psessionEntry->limSmeState);

        limSendSmeSetContextRsp(pMac, pSetContextReq->peerMacAddr,
                                1,
                                eSIR_SME_UNEXPECTED_REQ_RESULT_CODE,psessionEntry,
                                smesessionId,
                                smetransactionId);
    }

end:
    vos_mem_free( pSetContextReq);
    return;
} /*** end __limProcessSmeSetContextReq() ***/

/**
 * __limProcessSmeRemoveKeyReq()
 *
 *FUNCTION:
 * This function is called to process SME_REMOVEKEY_REQ message
 * from HDD or upper layer application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeRemoveKeyReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirSmeRemoveKeyReq    pRemoveKeyReq;
    tLimMlmRemoveKeyReq     *pMlmRemoveKeyReq;
    tpPESession             psessionEntry;
    tANI_U8                 sessionId;  //PE sessionID
    tANI_U8                 smesessionId;
    tANI_U16                smetransactionId;

    PELOG1(limLog(pMac, LOG1,
           FL("received REMOVEKEY_REQ message"));)

    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
           return;
    }


    limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&smesessionId,&smetransactionId);

    pRemoveKeyReq = vos_mem_malloc(sizeof(*pRemoveKeyReq));
    if ( NULL == pRemoveKeyReq )
    {
        //Log error
        limLog(pMac, LOGP,
               FL("call to AllocateMemory failed for pRemoveKeyReq"));

        return;
     }

    if ((limRemoveKeyReqSerDes(pMac,
                                pRemoveKeyReq,
                                (tANI_U8 *) pMsgBuf) == eSIR_FAILURE))
    {
        limLog(pMac, LOGW,
               FL("received invalid SME_REMOVECONTEXT_REQ message"));

        /* extra look up is needed since, session entry to be passed il limsendremovekey response */

        if((psessionEntry = peFindSessionByBssid(pMac,pRemoveKeyReq->bssId,&sessionId))== NULL)
        {
            limLog(pMac, LOGE,FL("session does not exist for given bssId"));
            //goto end;
        }

        limSendSmeRemoveKeyRsp(pMac,
                                pRemoveKeyReq->peerMacAddr,
                                eSIR_SME_INVALID_PARAMETERS,psessionEntry,
                                smesessionId,smetransactionId);

        goto end;
    }

    if((psessionEntry = peFindSessionByBssid(pMac,pRemoveKeyReq->bssId, &sessionId))== NULL)
    {
        limLog(pMac, LOGE,
                      FL("session does not exist for given bssId"));
        limSendSmeRemoveKeyRsp(pMac,
                                pRemoveKeyReq->peerMacAddr,
                                eSIR_SME_UNEXPECTED_REQ_RESULT_CODE, NULL,
                                smesessionId, smetransactionId);
        goto end;
    }


    if (((LIM_IS_STA_ROLE(psessionEntry) ||
          LIM_IS_BT_AMP_STA_ROLE(psessionEntry)) &&
         (psessionEntry->limSmeState == eLIM_SME_LINK_EST_STATE)) ||
        ((LIM_IS_IBSS_ROLE(psessionEntry) ||
          LIM_IS_AP_ROLE(psessionEntry) ||
          LIM_IS_BT_AMP_AP_ROLE(psessionEntry)) &&
         (psessionEntry->limSmeState == eLIM_SME_NORMAL_STATE))) {
        // Trigger MLM_REMOVEKEYS_REQ
        pMlmRemoveKeyReq = vos_mem_malloc(sizeof(tLimMlmRemoveKeyReq));
        if ( NULL == pMlmRemoveKeyReq )
        {
            // Log error
            limLog(pMac, LOGP,
                   FL("call to AllocateMemory failed for mlmRemoveKeysReq"));

            goto end;
        }

        pMlmRemoveKeyReq->edType  = (tAniEdType)pRemoveKeyReq->edType;
        pMlmRemoveKeyReq->keyId = pRemoveKeyReq->keyId;
        pMlmRemoveKeyReq->wepType = pRemoveKeyReq->wepType;
        pMlmRemoveKeyReq->unicast = pRemoveKeyReq->unicast;

        /* Update PE session Id */
        pMlmRemoveKeyReq->sessionId = sessionId;

        vos_mem_copy( (tANI_U8 *) &pMlmRemoveKeyReq->peerMacAddr,
                      (tANI_U8 *) &pRemoveKeyReq->peerMacAddr,
                      sizeof(tSirMacAddr));


        limPostMlmMessage(pMac,
                          LIM_MLM_REMOVEKEY_REQ,
                          (tANI_U32 *) pMlmRemoveKeyReq);
    }
    else
    {
        limLog(pMac, LOGE,
           FL("received unexpected SME_REMOVEKEY_REQ for role %d, state=%X"),
           GET_LIM_SYSTEM_ROLE(psessionEntry),
           psessionEntry->limSmeState);

        limSendSmeRemoveKeyRsp(pMac,
                                pRemoveKeyReq->peerMacAddr,
                                eSIR_SME_UNEXPECTED_REQ_RESULT_CODE,psessionEntry,
                                smesessionId,smetransactionId);
    }

end:
    vos_mem_free( pRemoveKeyReq);
} /*** end __limProcessSmeRemoveKeyReq() ***/

void limProcessSmeGetScanChannelInfo(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirMsgQ         mmhMsg;
    tpSmeGetScanChnRsp  pSirSmeRsp;
    tANI_U16 len = 0;
    tANI_U8 sessionId;
    tANI_U16 transactionId;

    if(pMac->lim.scanChnInfo.numChnInfo > SIR_MAX_SUPPORTED_CHANNEL_LIST)
    {
        limLog(pMac, LOGW, FL("numChn is out of bounds %d"),
                pMac->lim.scanChnInfo.numChnInfo);
        pMac->lim.scanChnInfo.numChnInfo = SIR_MAX_SUPPORTED_CHANNEL_LIST;
    }

    len = sizeof(tSmeGetScanChnRsp) + (pMac->lim.scanChnInfo.numChnInfo - 1) * sizeof(tLimScanChn);
    pSirSmeRsp = vos_mem_malloc(len);
    if ( NULL == pSirSmeRsp )
    {
        /// Buffer not available. Log error
        limLog(pMac, LOGP,
               FL("call to AllocateMemory failed for JOIN/REASSOC_RSP"));

        return;
    }
    vos_mem_set(pSirSmeRsp, len, 0);

    pSirSmeRsp->mesgType = eWNI_SME_GET_SCANNED_CHANNEL_RSP;
    pSirSmeRsp->mesgLen = len;

    if (pMac->fScanOffload)
    {
        limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&sessionId,&transactionId);
        pSirSmeRsp->sessionId = sessionId;
    }
    else
        pSirSmeRsp->sessionId = 0;

    if(pMac->lim.scanChnInfo.numChnInfo)
    {
        pSirSmeRsp->numChn = pMac->lim.scanChnInfo.numChnInfo;
        vos_mem_copy( pSirSmeRsp->scanChn, pMac->lim.scanChnInfo.scanChn,
                      sizeof(tLimScanChn) * pSirSmeRsp->numChn);
    }
    //Clear the list
    limRessetScanChannelInfo(pMac);

    mmhMsg.type = eWNI_SME_GET_SCANNED_CHANNEL_RSP;
    mmhMsg.bodyptr = pSirSmeRsp;
    mmhMsg.bodyval = 0;

    pMac->lim.gLimRspReqd = false;
    MTRACE(macTraceMsgTx(pMac, NO_SESSION, mmhMsg.type));
    limSysProcessMmhMsgApi(pMac, &mmhMsg,  ePROT);
}


void limProcessSmeGetAssocSTAsInfo(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirSmeGetAssocSTAsReq  getAssocSTAsReq;
    tpDphHashNode           pStaDs = NULL;
    tpPESession             psessionEntry = NULL;
    tSap_Event              sapEvent;
    tpWLAN_SAPEventCB       pSapEventCallback = NULL;
    tpSap_AssocMacAddr      pAssocStasTemp = NULL;// #include "sapApi.h"
    tANI_U8                 sessionId = CSR_SESSION_ID_INVALID;
    tANI_U8                 assocId = 0;
    tANI_U8                 staCount = 0;

    if (!limIsSmeGetAssocSTAsReqValid(pMac, &getAssocSTAsReq, (tANI_U8 *) pMsgBuf))
    {
        limLog(pMac, LOGE,
                        FL("received invalid eWNI_SME_GET_ASSOC_STAS_REQ message"));
        return;
    }

    switch (getAssocSTAsReq.modId)
    {
        case VOS_MODULE_ID_PE:
        default:
            break;
    }

    // Get Associated stations from PE
    // Find PE session Entry
    if ((psessionEntry = peFindSessionByBssid(pMac, getAssocSTAsReq.bssId, &sessionId)) == NULL)
    {
        limLog(pMac, LOGE,
                        FL("session does not exist for given bssId"));
        goto limAssocStaEnd;
    }

    if (!LIM_IS_AP_ROLE(psessionEntry)) {
        limLog(pMac, LOGE,
               FL("Received unexpected message in state %X, in role %X"),
               psessionEntry->limSmeState, GET_LIM_SYSTEM_ROLE(psessionEntry));
        goto limAssocStaEnd;
    }

    // Retrieve values obtained in the request message
    pSapEventCallback   = (tpWLAN_SAPEventCB)getAssocSTAsReq.pSapEventCallback;
    pAssocStasTemp      = (tpSap_AssocMacAddr)getAssocSTAsReq.pAssocStasArray;

    if (NULL == pAssocStasTemp)
        goto limAssocStaEnd;

    for (assocId = 0; assocId < psessionEntry->dph.dphHashTable.size; assocId++)// Softap dphHashTable.size = 8
    {
        pStaDs = dphGetHashEntry(pMac, assocId, &psessionEntry->dph.dphHashTable);

        if (NULL == pStaDs)
            continue;

        if (pStaDs->valid)
        {
            vos_mem_copy((tANI_U8 *)&pAssocStasTemp->staMac,
                         (tANI_U8 *)&pStaDs->staAddr,
                         sizeof(v_MACADDR_t));  // Mac address
            pAssocStasTemp->assocId = (v_U8_t)pStaDs->assocId;         // Association Id
            pAssocStasTemp->staId   = (v_U8_t)pStaDs->staIndex;        // Station Id

            vos_mem_copy((tANI_U8 *)&pAssocStasTemp->supportedRates,
                                      (tANI_U8 *)&pStaDs->supportedRates,
                                      sizeof(tSirSupportedRates));
            pAssocStasTemp->ShortGI40Mhz = pStaDs->htShortGI40Mhz;
            pAssocStasTemp->ShortGI20Mhz = pStaDs->htShortGI20Mhz;
            pAssocStasTemp->Support40Mhz = pStaDs->htDsssCckRate40MHzSupport;

            limLog(pMac, LOG1, FL("dph Station Number = %d"), staCount+1);
            limLog(pMac, LOG1, FL("MAC = " MAC_ADDRESS_STR),
                                        MAC_ADDR_ARRAY(pStaDs->staAddr));
            limLog(pMac, LOG1, FL("Association Id = %d"),pStaDs->assocId);
            limLog(pMac, LOG1, FL("Station Index = %d"),pStaDs->staIndex);

            pAssocStasTemp++;
            staCount++;
        }
    }

limAssocStaEnd:
    // Call hdd callback with sap event to send the list of associated stations from PE
    if (pSapEventCallback != NULL)
    {
        sapEvent.sapHddEventCode = eSAP_ASSOC_STA_CALLBACK_EVENT;
        sapEvent.sapevt.sapAssocStaListEvent.module = VOS_MODULE_ID_PE;
        sapEvent.sapevt.sapAssocStaListEvent.noOfAssocSta = staCount;
        sapEvent.sapevt.sapAssocStaListEvent.pAssocStas = (tpSap_AssocMacAddr)getAssocSTAsReq.pAssocStasArray;
        pSapEventCallback(&sapEvent, getAssocSTAsReq.pUsrContext);
    }
}


/**
 * limProcessSmeGetWPSPBCSessions
 *
 *FUNCTION:
 * This function is called when query the WPS PBC overlap message is received
 *
 *LOGIC:
 * This function parses get WPS PBC overlap information message and call callback to pass
 * WPS PBC overlap information back to hdd.
 *ASSUMPTIONS:
 *
 *
 *NOTE:
 *
 * @param  pMac     Pointer to Global MAC structure
 * @param  pMsgBuf  A pointer to WPS PBC overlap query message
*
 * @return None
 */
void limProcessSmeGetWPSPBCSessions(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirSmeGetWPSPBCSessionsReq  GetWPSPBCSessionsReq;
    tpPESession                  psessionEntry = NULL;
    tSap_Event                   sapEvent;
    tpWLAN_SAPEventCB            pSapEventCallback = NULL;
    tANI_U8                      sessionId = CSR_SESSION_ID_INVALID;
    tSirMacAddr                  zeroMac = {0,0,0,0,0,0};

    sapEvent.sapevt.sapGetWPSPBCSessionEvent.status = VOS_STATUS_E_FAULT;

    if (limIsSmeGetWPSPBCSessionsReqValid(pMac,  &GetWPSPBCSessionsReq, (tANI_U8 *) pMsgBuf) != eSIR_SUCCESS)
    {
        limLog(pMac, LOGE,
                        FL("received invalid eWNI_SME_GET_ASSOC_STAS_REQ message"));
        return;
    }

    // Get Associated stations from PE
    // Find PE session Entry
    if ((psessionEntry = peFindSessionByBssid(pMac, GetWPSPBCSessionsReq.bssId, &sessionId)) == NULL)
    {
        limLog(pMac, LOGE,
                        FL("session does not exist for given bssId"));
        goto limGetWPSPBCSessionsEnd;
    }

    if (!LIM_IS_AP_ROLE(psessionEntry)) {
        limLog(pMac, LOGE,
               FL("Received unexpected message in role %X"),
               GET_LIM_SYSTEM_ROLE(psessionEntry));
        goto limGetWPSPBCSessionsEnd;
    }

    // Call hdd callback with sap event to send the WPS PBC overlap information
    sapEvent.sapHddEventCode =  eSAP_GET_WPSPBC_SESSION_EVENT;
    sapEvent.sapevt.sapGetWPSPBCSessionEvent.module = VOS_MODULE_ID_PE;

    if (vos_mem_compare( zeroMac, GetWPSPBCSessionsReq.pRemoveMac, sizeof(tSirMacAddr)))
    { //This is GetWpsSession call

      limGetWPSPBCSessions(pMac,
              sapEvent.sapevt.sapGetWPSPBCSessionEvent.addr.bytes, sapEvent.sapevt.sapGetWPSPBCSessionEvent.UUID_E,
              &sapEvent.sapevt.sapGetWPSPBCSessionEvent.wpsPBCOverlap, psessionEntry);
    }
    else
    {
      limRemovePBCSessions(pMac, GetWPSPBCSessionsReq.pRemoveMac,psessionEntry);
      /* don't have to inform the HDD/Host */
      return;
    }

    PELOG4(limLog(pMac, LOGE, FL("wpsPBCOverlap %d"), sapEvent.sapevt.sapGetWPSPBCSessionEvent.wpsPBCOverlap);)
    PELOG4(limPrintMacAddr(pMac, sapEvent.sapevt.sapGetWPSPBCSessionEvent.addr.bytes, LOG4);)

    sapEvent.sapevt.sapGetWPSPBCSessionEvent.status = VOS_STATUS_SUCCESS;

limGetWPSPBCSessionsEnd:
    pSapEventCallback   = (tpWLAN_SAPEventCB)GetWPSPBCSessionsReq.pSapEventCallback;
    if (NULL != pSapEventCallback)
       pSapEventCallback(&sapEvent, GetWPSPBCSessionsReq.pUsrContext);
}



/**
 * __limCounterMeasures()
 *
 * FUNCTION:
 * This function is called to "implement" MIC counter measure
 * and is *temporary* only
 *
 * LOGIC: on AP, disassoc all STA associated thru TKIP,
 * we don't do the proper STA disassoc sequence since the
 * BSS will be stoped anyway
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @return None
 */

static void
__limCounterMeasures(tpAniSirGlobal pMac, tpPESession psessionEntry)
{
    tSirMacAddr mac = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (LIM_IS_AP_ROLE(psessionEntry) || LIM_IS_BT_AMP_AP_ROLE(psessionEntry) ||
        LIM_IS_BT_AMP_STA_ROLE(psessionEntry))
        limSendDisassocMgmtFrame(pMac, eSIR_MAC_MIC_FAILURE_REASON, mac, psessionEntry, FALSE);
}


void
limProcessTkipCounterMeasures(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirSmeTkipCntrMeasReq  tkipCntrMeasReq;
    tpPESession             psessionEntry;
    tANI_U8                 sessionId;      //PE sessionId

    if ( limTkipCntrMeasReqSerDes( pMac, &tkipCntrMeasReq, (tANI_U8 *) pMsgBuf ) != eSIR_SUCCESS )
    {
        limLog(pMac, LOGE,
                        FL("received invalid eWNI_SME_TKIP_CNTR_MEAS_REQ message"));
        return;
    }

    if ( NULL == (psessionEntry = peFindSessionByBssid( pMac, tkipCntrMeasReq.bssId, &sessionId )) )
    {
        limLog(pMac, LOGE, FL("session does not exist for given BSSID "));
        return;
    }

    if ( tkipCntrMeasReq.bEnable )
    {
        __limCounterMeasures( pMac, psessionEntry );
    }

    psessionEntry->bTkipCntrMeasActive = tkipCntrMeasReq.bEnable;
}


static void
__limHandleSmeStopBssRequest(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirSmeStopBssReq  stopBssReq;
    tSirRetStatus      status;
    tLimSmeStates      prevState;
    tANI_U8            sessionId;  //PE sessionId
    tpPESession        psessionEntry;
    tANI_U8            smesessionId;
    tANI_U16           smetransactionId;
    tANI_U8 i = 0;
    tpDphHashNode      pStaDs = NULL;
    limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&smesessionId,&smetransactionId);



    if ((limStopBssReqSerDes(pMac, &stopBssReq, (tANI_U8 *) pMsgBuf) != eSIR_SUCCESS) ||
        !limIsSmeStopBssReqValid(pMsgBuf))
    {
        PELOGW(limLog(pMac, LOGW, FL("received invalid SME_STOP_BSS_REQ message"));)
        /// Send Stop BSS response to host
        limSendSmeRsp(pMac, eWNI_SME_STOP_BSS_RSP, eSIR_SME_INVALID_PARAMETERS,smesessionId,smetransactionId);
        return;
    }


    if((psessionEntry = peFindSessionByBssid(pMac,stopBssReq.bssId,&sessionId)) == NULL)
    {
        limLog(pMac, LOGW, FL("session does not exist for given BSSID "));
        limSendSmeRsp(pMac, eWNI_SME_STOP_BSS_RSP, eSIR_SME_INVALID_PARAMETERS,smesessionId,smetransactionId);
        return;
    }

#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    limDiagEventReport(pMac, WLAN_PE_DIAG_STOP_BSS_REQ_EVENT, psessionEntry, 0, 0);
#endif //FEATURE_WLAN_DIAG_SUPPORT


    if ((psessionEntry->limSmeState != eLIM_SME_NORMAL_STATE) ||    /* Added For BT -AMP Support */
        LIM_IS_STA_ROLE(psessionEntry)) {
        /**
         * Should not have received STOP_BSS_REQ in states
         * other than 'normal' state or on STA in Infrastructure
         * mode. Log error and return response to host.
         */
        limLog(pMac, LOGE,
           FL("received unexpected SME_STOP_BSS_REQ in state %X, for role %d"),
           psessionEntry->limSmeState, GET_LIM_SYSTEM_ROLE(psessionEntry));
        /// Send Stop BSS response to host
        limSendSmeRsp(pMac, eWNI_SME_STOP_BSS_RSP, eSIR_SME_UNEXPECTED_REQ_RESULT_CODE,smesessionId,smetransactionId);
        return;
    }

    if (LIM_IS_AP_ROLE(psessionEntry)) {
        limWPSPBCClose(pMac, psessionEntry);
    }
    PELOGW(limLog(pMac, LOGW, FL("RECEIVED STOP_BSS_REQ with reason code=%d"), stopBssReq.reasonCode);)

    prevState = psessionEntry->limSmeState;

    psessionEntry->limSmeState = eLIM_SME_IDLE_STATE;
    MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry->limSmeState));

    /* Update SME session Id and Transaction Id */
    psessionEntry->smeSessionId = smesessionId;
    psessionEntry->transactionId = smetransactionId;

    /* BTAMP_STA and STA_IN_IBSS should NOT send Disassoc frame */
    if (!LIM_IS_IBSS_ROLE(psessionEntry) &&
        !LIM_IS_BT_AMP_STA_ROLE(psessionEntry)) {
        tSirMacAddr   bcAddr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        if ((stopBssReq.reasonCode == eSIR_SME_MIC_COUNTER_MEASURES))
            // Send disassoc all stations associated thru TKIP
            __limCounterMeasures(pMac,psessionEntry);
        else
            limSendDisassocMgmtFrame(pMac, eSIR_MAC_DEAUTH_LEAVING_BSS_REASON, bcAddr, psessionEntry, FALSE);
    }

    /* Free the buffer allocated in START_BSS_REQ */
    vos_mem_free(psessionEntry->addIeParams.probeRespData_buff);
    psessionEntry->addIeParams.probeRespDataLen = 0;
    psessionEntry->addIeParams.probeRespData_buff = NULL;

    vos_mem_free(psessionEntry->addIeParams.assocRespData_buff);
    psessionEntry->addIeParams.assocRespDataLen = 0;
    psessionEntry->addIeParams.assocRespData_buff = NULL;

    vos_mem_free(psessionEntry->addIeParams.probeRespBCNData_buff);
    psessionEntry->addIeParams.probeRespBCNDataLen = 0;
    psessionEntry->addIeParams.probeRespBCNData_buff = NULL;

    //limDelBss is also called as part of coalescing, when we send DEL BSS followed by Add Bss msg.
    pMac->lim.gLimIbssCoalescingHappened = false;

    for(i = 1 ; i < pMac->lim.gLimAssocStaLimit ; i++)
    {
        pStaDs = dphGetHashEntry(pMac, i, &psessionEntry->dph.dphHashTable);
        if (NULL == pStaDs)
            continue;
        status = limDelSta(pMac, pStaDs, false, psessionEntry) ;
        if(eSIR_SUCCESS == status)
        {
            limDeleteDphHashEntry(pMac, pStaDs->staAddr, pStaDs->assocId, psessionEntry) ;
            limReleasePeerIdx(pMac, pStaDs->assocId, psessionEntry) ;
        }
        else
        {
            limLog(pMac, LOGE, FL("limDelSta failed with Status : %d"), status);
            VOS_ASSERT(0) ;
        }
    }
    /* send a delBss to HAL and wait for a response */
    status = limDelBss(pMac, NULL,psessionEntry->bssIdx,psessionEntry);

    if (status != eSIR_SUCCESS)
    {
        PELOGE(limLog(pMac, LOGE, FL("delBss failed for bss %d"), psessionEntry->bssIdx);)
        psessionEntry->limSmeState= prevState;

        MTRACE(macTrace(pMac, TRACE_CODE_SME_STATE, psessionEntry->peSessionId, psessionEntry->limSmeState));

        limSendSmeRsp(pMac, eWNI_SME_STOP_BSS_RSP, eSIR_SME_STOP_BSS_FAILURE,smesessionId,smetransactionId);
    }
}


/**--------------------------------------------------------------
\fn     __limProcessSmeStopBssReq

\brief  Wrapper for the function __limHandleSmeStopBssRequest
        This message will be defered until softmac come out of
        scan mode. Message should be handled even if we have
        detected radar in the current operating channel.
\param  pMac
\param  pMsg

\return TRUE - If we consumed the buffer
        FALSE - If have defered the message.
 ---------------------------------------------------------------*/
static tANI_BOOLEAN
__limProcessSmeStopBssReq(tpAniSirGlobal pMac, tpSirMsgQ pMsg)
{
    if (__limIsDeferedMsgForLearn(pMac, pMsg))
    {
        /**
         * If message defered, buffer is not consumed yet.
         * So return false
         */
        return eANI_BOOLEAN_FALSE;
    }
    __limHandleSmeStopBssRequest(pMac, (tANI_U32 *) pMsg->bodyptr);
    return eANI_BOOLEAN_TRUE;
} /*** end __limProcessSmeStopBssReq() ***/


void limProcessSmeDelBssRsp(
    tpAniSirGlobal  pMac,
    tANI_U32        body,tpPESession psessionEntry)
{

    (void) body;
    SET_LIM_PROCESS_DEFD_MESGS(pMac, true);
    limIbssDelete(pMac,psessionEntry);
    dphHashTableClassInit(pMac, &psessionEntry->dph.dphHashTable);
    limDeletePreAuthList(pMac);
    limSendSmeRsp(pMac, eWNI_SME_STOP_BSS_RSP, eSIR_SME_SUCCESS,psessionEntry->smeSessionId,psessionEntry->transactionId);
    return;
}


/**---------------------------------------------------------------
\fn     __limProcessSmeAssocCnfNew
\brief  This function handles SME_ASSOC_CNF/SME_REASSOC_CNF
\       in BTAMP AP.
\
\param  pMac
\param  msgType - message type
\param  pMsgBuf - a pointer to the SME message buffer
\return None
------------------------------------------------------------------*/

  void
__limProcessSmeAssocCnfNew(tpAniSirGlobal pMac, tANI_U32 msgType, tANI_U32 *pMsgBuf)
{
    tSirSmeAssocCnf    assocCnf;
    tpDphHashNode      pStaDs = NULL;
    tpPESession        psessionEntry= NULL;
    tANI_U8            sessionId;


    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE, FL("pMsgBuf is NULL "));
        goto end;
    }

    if ((limAssocCnfSerDes(pMac, &assocCnf, (tANI_U8 *) pMsgBuf) == eSIR_FAILURE) ||
        !__limIsSmeAssocCnfValid(&assocCnf))
    {
        limLog(pMac, LOGE, FL("Received invalid SME_RE(ASSOC)_CNF message "));
        goto end;
    }

    if((psessionEntry = peFindSessionByBssid(pMac, assocCnf.bssId, &sessionId))== NULL)
    {
        limLog(pMac, LOGE, FL("session does not exist for given bssId"));
        goto end;
    }

    if ((!LIM_IS_AP_ROLE(psessionEntry) &&
         !LIM_IS_BT_AMP_AP_ROLE(psessionEntry)) ||
        ((psessionEntry->limSmeState != eLIM_SME_NORMAL_STATE) &&
        (psessionEntry->limSmeState != eLIM_SME_NORMAL_CHANNEL_SCAN_STATE))) {
        limLog(pMac, LOGE,
               FL("Received unexpected message %X in state %X, in role %X"),
               msgType, psessionEntry->limSmeState,
               GET_LIM_SYSTEM_ROLE(psessionEntry));
        goto end;
    }

    pStaDs = dphGetHashEntry(pMac, assocCnf.aid, &psessionEntry->dph.dphHashTable);

    if (pStaDs == NULL)
    {
        limLog(pMac, LOGE,
            FL("Received invalid message %X due to no STA context"
               "for aid %d, peer "),
               msgType, assocCnf.aid);
        limPrintMacAddr(pMac, assocCnf.peerMacAddr, LOG1);

        /*
        ** send a DISASSOC_IND message to WSM to make sure
        ** the state in WSM and LIM is the same
        **/
       limSendSmeDisassocNtf( pMac, assocCnf.peerMacAddr, eSIR_SME_STA_NOT_ASSOCIATED,
                              eLIM_PEER_ENTITY_DISASSOC, assocCnf.aid,psessionEntry->smeSessionId,psessionEntry->transactionId,psessionEntry);
       goto end;
    }
    if ((pStaDs &&
         (( !vos_mem_compare( (tANI_U8 *) pStaDs->staAddr,
                     (tANI_U8 *) assocCnf.peerMacAddr,
                     sizeof(tSirMacAddr)) ) ||
          (pStaDs->mlmStaContext.mlmState != eLIM_MLM_WT_ASSOC_CNF_STATE) ||
          ((pStaDs->mlmStaContext.subType == LIM_ASSOC) &&
           (msgType != eWNI_SME_ASSOC_CNF)) ||
          ((pStaDs->mlmStaContext.subType == LIM_REASSOC) &&
           (msgType != eWNI_SME_ASSOC_CNF))))) // since softap is passing this as ASSOC_CNF and subtype differs
    {
        limLog(pMac, LOG1,
           FL("Received invalid message %X due to peerMacAddr mismatched"
              "or not in eLIM_MLM_WT_ASSOC_CNF_STATE state, for aid %d, peer "
              "StaD mlmState : %d"),
              msgType, assocCnf.aid, pStaDs->mlmStaContext.mlmState);
        limPrintMacAddr(pMac, assocCnf.peerMacAddr, LOG1);
        goto end;
    }

    /*
    ** Deactivate/delet CNF_WAIT timer since ASSOC_CNF
    ** has been received
    **/
    limLog(pMac, LOG1, FL("Received SME_ASSOC_CNF. Delete Timer"));
    limDeactivateAndChangePerStaIdTimer(pMac, eLIM_CNF_WAIT_TIMER, pStaDs->assocId);

    if (assocCnf.statusCode == eSIR_SME_SUCCESS)
    {
        /* In BTAMP-AP, PE already finished the WDA_ADD_STA sequence
         * when it had received Assoc Request frame. Now, PE just needs to send
         * Association Response frame to the requesting BTAMP-STA.
         */
        pStaDs->mlmStaContext.mlmState = eLIM_MLM_LINK_ESTABLISHED_STATE;
        limLog(pMac, LOG1, FL("sending Assoc Rsp frame to STA (assoc id=%d) "), pStaDs->assocId);
        limSendAssocRspMgmtFrame( pMac, eSIR_SUCCESS, pStaDs->assocId, pStaDs->staAddr,
                                  pStaDs->mlmStaContext.subType, pStaDs, psessionEntry);
        goto end;
    } // (assocCnf.statusCode == eSIR_SME_SUCCESS)
    else
    {
        // SME_ASSOC_CNF status is non-success, so STA is not allowed to be associated
        /*Since the HAL sta entry is created for denied STA we need to remove this HAL entry.So to do that set updateContext to 1*/
        if(!pStaDs->mlmStaContext.updateContext)
           pStaDs->mlmStaContext.updateContext = 1;
        limLog(pMac, LOG1, FL("Receive Assoc Cnf with status Code : %d(assoc id=%d) "),
                           assocCnf.statusCode, pStaDs->assocId);
        limRejectAssociation(pMac, pStaDs->staAddr,
                             pStaDs->mlmStaContext.subType,
                             true, pStaDs->mlmStaContext.authType,
                             pStaDs->assocId, true,
                             eSIR_MAC_UNSPEC_FAILURE_STATUS, psessionEntry);
    }

end:
    if((psessionEntry != NULL) && (pStaDs != NULL))
    {
        if ( psessionEntry->parsedAssocReq[pStaDs->assocId] != NULL )
        {
            if ( ((tpSirAssocReq)(psessionEntry->parsedAssocReq[pStaDs->assocId]))->assocReqFrame)
            {
                vos_mem_free(((tpSirAssocReq)
                   (psessionEntry->parsedAssocReq[pStaDs->assocId]))->assocReqFrame);
                ((tpSirAssocReq)(psessionEntry->parsedAssocReq[pStaDs->assocId]))->assocReqFrame = NULL;
            }

            vos_mem_free(psessionEntry->parsedAssocReq[pStaDs->assocId]);
            psessionEntry->parsedAssocReq[pStaDs->assocId] = NULL;
        }
    }

} /*** end __limProcessSmeAssocCnfNew() ***/

#ifdef SAP_AUTH_OFFLOAD
/**
 * __lim_process_sme_assoc_offload_cnf() station connect confirm
 * @pMac: SirGlobal handler
 * @msgType: message type
 * @pMsgBuf: message body
 *
 * This function handles the station connect confirm of
 * Software AP authentication offload feature
 *
 * Return: None
 */
static void
__lim_process_sme_assoc_offload_cnf(tpAniSirGlobal pmac,
                               tANI_U32 msg_type,
                               tANI_U32 *pmsg_buf)
{
    tSirSmeAssocCnf assoc_cnf;
    tpDphHashNode sta_ds = NULL;
    tpPESession psession_entry= NULL;
    tANI_U8 session_id;
    tANI_U16 aid=0;

    if(pmsg_buf == NULL) {
        limLog(pmac, LOGE, FL("pmsg_buf is NULL "));
        goto end;
    }

    if ((limAssocCnfSerDes(pmac, &assoc_cnf, (tANI_U8 *) pmsg_buf) ==
        eSIR_FAILURE) || !__limIsSmeAssocCnfValid(&assoc_cnf)) {
        limLog(pmac, LOGE, FL("Received invalid SME_RE(ASSOC)_CNF message "));
        goto end;
    }

    if((psession_entry =
        peFindSessionByBssid(pmac, assoc_cnf.bssId, &session_id))== NULL) {
        limLog(pmac, LOGE, FL("session does not exist for given bssId"));
        goto end;
    }

    if ((!LIM_IS_AP_ROLE(psession_entry) &&
         !LIM_IS_BT_AMP_AP_ROLE(psession_entry)) ||
        ((psession_entry->limSmeState != eLIM_SME_NORMAL_STATE) &&
        (psession_entry->limSmeState != eLIM_SME_NORMAL_CHANNEL_SCAN_STATE))) {
        limLog(pmac, LOGE,
               FL("Received unexpected message %X in state %X, in role %X"),
               msg_type, psession_entry->limSmeState,
               GET_LIM_SYSTEM_ROLE(psession_entry));
        goto end;
    }

    sta_ds = dphGetHashEntry(pmac,
                             assoc_cnf.aid,
                             &psession_entry->dph.dphHashTable);
    if (sta_ds != NULL) {
        aid = sta_ds->assocId;
        limDeactivateAndChangePerStaIdTimer(pmac,
                                            eLIM_CNF_WAIT_TIMER,
                                            aid);
    }

end:
    if((psession_entry != NULL) && (sta_ds != NULL)) {
        if ( psession_entry->parsedAssocReq[aid] != NULL ) {
            if ( ((tpSirAssocReq)
                (psession_entry->parsedAssocReq[aid]))->assocReqFrame) {
                vos_mem_free(((tpSirAssocReq)
                    (psession_entry->parsedAssocReq[aid]))->assocReqFrame);
                ((tpSirAssocReq)
                    (psession_entry->parsedAssocReq[aid]))->assocReqFrame =
                    NULL;
            }
            vos_mem_free(psession_entry->parsedAssocReq[aid]);
            psession_entry->parsedAssocReq[aid] = NULL;
        }
    }

} /*** end __lim_process_sme_assoc_offload_cnf() ***/
#endif /* SAP_AUTH_OFFLOAD */

static void
__limProcessSmeAddtsReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpDphHashNode   pStaDs;
    tSirMacAddr     peerMac;
    tpSirAddtsReq   pSirAddts;
    tANI_U32        timeout;
    tpPESession     psessionEntry;
    tANI_U8         sessionId;  //PE sessionId
    tANI_U8         smesessionId;
    tANI_U16        smetransactionId;


    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
           return;
    }

    limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&smesessionId,&smetransactionId);

    pSirAddts = (tpSirAddtsReq) pMsgBuf;

    if((psessionEntry = peFindSessionByBssid(pMac, pSirAddts->bssId,&sessionId))== NULL)
    {
        limLog(pMac, LOGE, "Session Does not exist for given bssId");
        return;
    }
#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    limDiagEventReport(pMac, WLAN_PE_DIAG_ADDTS_REQ_EVENT, psessionEntry, 0, 0);
#endif //FEATURE_WLAN_DIAG_SUPPORT



    /* if sta
     *  - verify assoc state
     *  - send addts request to ap
     *  - wait for addts response from ap
     * if ap, just ignore with error log
     */
    PELOG1(limLog(pMac, LOG1,
           FL("Received SME_ADDTS_REQ (TSid %d, UP %d)"),
           pSirAddts->req.tspec.tsinfo.traffic.tsid,
           pSirAddts->req.tspec.tsinfo.traffic.userPrio);)

    if (!LIM_IS_STA_ROLE(psessionEntry) &&
        !LIM_IS_BT_AMP_STA_ROLE(psessionEntry)) {
        PELOGE(limLog(pMac, LOGE, "AddTs received on AP - ignoring");)
        limSendSmeAddtsRsp(pMac, pSirAddts->rspReqd, eSIR_FAILURE, psessionEntry, pSirAddts->req.tspec,
                smesessionId,smetransactionId);
        return;
    }

    pStaDs = dphGetHashEntry(pMac, DPH_STA_HASH_INDEX_PEER, &psessionEntry->dph.dphHashTable);

    if(pStaDs == NULL)
    {
        PELOGE(limLog(pMac, LOGE, "Cannot find AP context for addts req");)
        limSendSmeAddtsRsp(pMac, pSirAddts->rspReqd, eSIR_FAILURE, psessionEntry, pSirAddts->req.tspec,
            smesessionId,smetransactionId);
        return;
    }

    if ((! pStaDs->valid) ||
        (pStaDs->mlmStaContext.mlmState != eLIM_MLM_LINK_ESTABLISHED_STATE))
    {
        PELOGE(limLog(pMac, LOGE, "AddTs received in invalid MLM state");)
        limSendSmeAddtsRsp(pMac, pSirAddts->rspReqd, eSIR_FAILURE, psessionEntry, pSirAddts->req.tspec,
            smesessionId,smetransactionId);
        return;
    }

    pSirAddts->req.wsmTspecPresent = 0;
    pSirAddts->req.wmeTspecPresent = 0;
    pSirAddts->req.lleTspecPresent = 0;

    if ((pStaDs->wsmEnabled) &&
        (pSirAddts->req.tspec.tsinfo.traffic.accessPolicy != SIR_MAC_ACCESSPOLICY_EDCA))
        pSirAddts->req.wsmTspecPresent = 1;
    else if (pStaDs->wmeEnabled)
        pSirAddts->req.wmeTspecPresent = 1;
    else if (pStaDs->lleEnabled)
        pSirAddts->req.lleTspecPresent = 1;
    else
    {
        PELOGW(limLog(pMac, LOGW, FL("ADDTS_REQ ignore - qos is disabled"));)
        limSendSmeAddtsRsp(pMac, pSirAddts->rspReqd, eSIR_FAILURE, psessionEntry, pSirAddts->req.tspec,
            smesessionId,smetransactionId);
        return;
    }

    if ((psessionEntry->limSmeState != eLIM_SME_ASSOCIATED_STATE) &&
        (psessionEntry->limSmeState != eLIM_SME_LINK_EST_STATE))
    {
        limLog(pMac, LOGE, "AddTs received in invalid LIMsme state (%d)",
              psessionEntry->limSmeState);
        limSendSmeAddtsRsp(pMac, pSirAddts->rspReqd, eSIR_FAILURE, psessionEntry, pSirAddts->req.tspec,
            smesessionId,smetransactionId);
        return;
    }

    if (pMac->lim.gLimAddtsSent)
    {
        limLog(pMac, LOGE, "Addts (token %d, tsid %d, up %d) is still pending",
               pMac->lim.gLimAddtsReq.req.dialogToken,
               pMac->lim.gLimAddtsReq.req.tspec.tsinfo.traffic.tsid,
               pMac->lim.gLimAddtsReq.req.tspec.tsinfo.traffic.userPrio);
        limSendSmeAddtsRsp(pMac, pSirAddts->rspReqd, eSIR_FAILURE, psessionEntry, pSirAddts->req.tspec,
            smesessionId,smetransactionId);
        return;
    }

    sirCopyMacAddr(peerMac,psessionEntry->bssId);

    // save the addts request
    pMac->lim.gLimAddtsSent = true;
    vos_mem_copy( (tANI_U8 *) &pMac->lim.gLimAddtsReq, (tANI_U8 *) pSirAddts, sizeof(tSirAddtsReq));

    // ship out the message now
    limSendAddtsReqActionFrame(pMac, peerMac, &pSirAddts->req,
            psessionEntry);
    PELOG1(limLog(pMac, LOG1, "Sent ADDTS request");)

    // start a timer to wait for the response
    if (pSirAddts->timeout)
        timeout = pSirAddts->timeout;
    else if (wlan_cfgGetInt(pMac, WNI_CFG_ADDTS_RSP_TIMEOUT, &timeout) != eSIR_SUCCESS)
    {
        limLog(pMac, LOGP, FL("Unable to get Cfg param %d (Addts Rsp Timeout)"),
               WNI_CFG_ADDTS_RSP_TIMEOUT);
        return;
    }

    timeout = SYS_MS_TO_TICKS(timeout);
    if (tx_timer_change(&pMac->lim.limTimers.gLimAddtsRspTimer, timeout, 0) != TX_SUCCESS)
    {
        limLog(pMac, LOGP, FL("AddtsRsp timer change failed!"));
        return;
    }
    pMac->lim.gLimAddtsRspTimerCount++;
    if (tx_timer_change_context(&pMac->lim.limTimers.gLimAddtsRspTimer,
                                pMac->lim.gLimAddtsRspTimerCount) != TX_SUCCESS)
    {
        limLog(pMac, LOGP, FL("AddtsRsp timer change failed!"));
        return;
    }
    MTRACE(macTrace(pMac, TRACE_CODE_TIMER_ACTIVATE, psessionEntry->peSessionId, eLIM_ADDTS_RSP_TIMER));

    //add the sessionId to the timer object
    pMac->lim.limTimers.gLimAddtsRspTimer.sessionId = sessionId;
    if (tx_timer_activate(&pMac->lim.limTimers.gLimAddtsRspTimer) != TX_SUCCESS)
    {
        limLog(pMac, LOGP, FL("AddtsRsp timer activation failed!"));
        return;
    }
    return;
}


static void
__limProcessSmeDeltsReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirMacAddr     peerMacAddr;
    tANI_U8         ac;
    tSirMacTSInfo   *pTsinfo;
    tpSirDeltsReq   pDeltsReq = (tpSirDeltsReq) pMsgBuf;
    tpDphHashNode   pStaDs = NULL;
    tpPESession     psessionEntry;
    tANI_U8         sessionId;
    tANI_U32        status = eSIR_SUCCESS;
    tANI_U8         smesessionId;
    tANI_U16        smetransactionId;

    limGetSessionInfo(pMac,(tANI_U8 *)pMsgBuf,&smesessionId,&smetransactionId);

    if((psessionEntry = peFindSessionByBssid(pMac, pDeltsReq->bssId, &sessionId))== NULL)
    {
        limLog(pMac, LOGE, "Session Does not exist for given bssId");
        status = eSIR_FAILURE;
        goto end;
    }
#ifdef FEATURE_WLAN_DIAG_SUPPORT_LIM //FEATURE_WLAN_DIAG_SUPPORT
    limDiagEventReport(pMac, WLAN_PE_DIAG_DELTS_REQ_EVENT, psessionEntry, 0, 0);
#endif //FEATURE_WLAN_DIAG_SUPPORT


    if (eSIR_SUCCESS != limValidateDeltsReq(pMac, pDeltsReq, peerMacAddr,psessionEntry))
    {
        PELOGE(limLog(pMac, LOGE, FL("limValidateDeltsReq failed"));)
        status = eSIR_FAILURE;
        limSendSmeDeltsRsp(pMac, pDeltsReq, eSIR_FAILURE,psessionEntry,smesessionId,smetransactionId);
        return;
    }

    PELOG1(limLog(pMac, LOG1, FL("Sent DELTS request to station with "
                  "assocId = %d MacAddr = "MAC_ADDRESS_STR),
                  pDeltsReq->aid, MAC_ADDR_ARRAY(peerMacAddr));)

    limSendDeltsReqActionFrame(pMac, peerMacAddr, pDeltsReq->req.wmeTspecPresent, &pDeltsReq->req.tsinfo, &pDeltsReq->req.tspec,
              psessionEntry);

    pTsinfo = pDeltsReq->req.wmeTspecPresent ? &pDeltsReq->req.tspec.tsinfo : &pDeltsReq->req.tsinfo;

    /* We've successfully send DELTS frame to AP. Update the
     * dynamic UAPSD mask. The AC for this TSPEC to be deleted
     * is no longer trigger enabled or delivery enabled
     */
    if(!pMac->psOffloadEnabled)
    {
        limSetTspecUapsdMask(pMac, pTsinfo, CLEAR_UAPSD_MASK);

        /* We're deleting the TSPEC, so this particular AC is no longer
         * admitted.  PE needs to downgrade the EDCA
         * parameters(for the AC for which TS is being deleted) to the
         * next best AC for which ACM is not enabled, and send the
         * updated values to HAL.
         */
        ac = upToAc(pTsinfo->traffic.userPrio);

        if(pTsinfo->traffic.direction == SIR_MAC_DIRECTION_UPLINK)
        {
            pMac->lim.gAcAdmitMask[SIR_MAC_DIRECTION_UPLINK] &= ~(1 << ac);
        }
        else if(pTsinfo->traffic.direction == SIR_MAC_DIRECTION_DNLINK)
        {
            pMac->lim.gAcAdmitMask[SIR_MAC_DIRECTION_DNLINK] &= ~(1 << ac);
        }
        else if(pTsinfo->traffic.direction == SIR_MAC_DIRECTION_BIDIR)
        {
            pMac->lim.gAcAdmitMask[SIR_MAC_DIRECTION_UPLINK] &= ~(1 << ac);
            pMac->lim.gAcAdmitMask[SIR_MAC_DIRECTION_DNLINK] &= ~(1 << ac);
        }
    }
    else
    {
        limSetTspecUapsdMaskPerSession(pMac, psessionEntry,
                                       pTsinfo, CLEAR_UAPSD_MASK);

        /* We're deleting the TSPEC, so this particular AC is no longer
         * admitted.  PE needs to downgrade the EDCA
         * parameters(for the AC for which TS is being deleted) to the
         * next best AC for which ACM is not enabled, and send the
         * updated values to HAL.
         */
        ac = upToAc(pTsinfo->traffic.userPrio);

        if(pTsinfo->traffic.direction == SIR_MAC_DIRECTION_UPLINK)
        {
            psessionEntry->gAcAdmitMask[SIR_MAC_DIRECTION_UPLINK] &= ~(1 << ac);
        }
        else if(pTsinfo->traffic.direction == SIR_MAC_DIRECTION_DNLINK)
        {
            psessionEntry->gAcAdmitMask[SIR_MAC_DIRECTION_DNLINK] &= ~(1 << ac);
        }
        else if(pTsinfo->traffic.direction == SIR_MAC_DIRECTION_BIDIR)
        {
            psessionEntry->gAcAdmitMask[SIR_MAC_DIRECTION_UPLINK] &= ~(1 << ac);
            psessionEntry->gAcAdmitMask[SIR_MAC_DIRECTION_DNLINK] &= ~(1 << ac);
        }
    }

    limSetActiveEdcaParams(pMac, psessionEntry->gLimEdcaParams, psessionEntry);

    pStaDs = dphGetHashEntry(pMac, DPH_STA_HASH_INDEX_PEER, &psessionEntry->dph.dphHashTable);
    if (pStaDs != NULL)
    {
        limSendEdcaParams(pMac, psessionEntry->gLimEdcaParamsActive,
                          pStaDs->bssId);
        status = eSIR_SUCCESS;
    }
    else
    {
        limLog(pMac, LOGE, FL("Self entry missing in Hash Table "));
     status = eSIR_FAILURE;
    }
#ifdef FEATURE_WLAN_ESE
#ifdef FEATURE_WLAN_ESE_UPLOAD
    limSendSmeTsmIEInd(pMac, psessionEntry, 0, 0, 0);
#else
    limDeactivateAndChangeTimer(pMac,eLIM_TSM_TIMER);
#endif /* FEATURE_WLAN_ESE_UPLOAD */
#endif

    // send an sme response back
    end:
         limSendSmeDeltsRsp(pMac, pDeltsReq, eSIR_SUCCESS,psessionEntry,smesessionId,smetransactionId);
}


void
limProcessSmeAddtsRspTimeout(tpAniSirGlobal pMac, tANI_U32 param)
{
    //fetch the sessionEntry based on the sessionId
    tpPESession psessionEntry;
    if((psessionEntry = peFindSessionBySessionId(pMac, pMac->lim.limTimers.gLimAddtsRspTimer.sessionId))== NULL)
    {
        limLog(pMac, LOGP,FL("Session Does not exist for given sessionID"));
        return;
    }

    if (!LIM_IS_STA_ROLE(psessionEntry) &&
        !LIM_IS_BT_AMP_STA_ROLE(psessionEntry)) {
        limLog(pMac, LOGW, "AddtsRspTimeout in non-Sta role (%d)",
               GET_LIM_SYSTEM_ROLE(psessionEntry));
        pMac->lim.gLimAddtsSent = false;
        return;
    }

    if (! pMac->lim.gLimAddtsSent)
    {
        PELOGW(limLog(pMac, LOGW, "AddtsRspTimeout but no AddtsSent");)
        return;
    }

    if (param != pMac->lim.gLimAddtsRspTimerCount)
    {
        limLog(pMac, LOGE, FL("Invalid AddtsRsp Timer count %d (exp %d)"),
               param, pMac->lim.gLimAddtsRspTimerCount);
        return;
    }

    // this a real response timeout
    pMac->lim.gLimAddtsSent = false;
    pMac->lim.gLimAddtsRspTimerCount++;

    limSendSmeAddtsRsp(pMac, true, eSIR_SME_ADDTS_RSP_TIMEOUT, psessionEntry, pMac->lim.gLimAddtsReq.req.tspec,
            psessionEntry->smeSessionId, psessionEntry->transactionId);
}


/**
 * __limProcessSmeStatsRequest()
 *
 *FUNCTION:
 *
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */
static void
__limProcessSmeStatsRequest(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpAniGetStatsReq    pStatsReq;
    tSirMsgQ msgQ;
    tpPESession psessionEntry;
    tANI_U8     sessionId;


    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
           return;
    }

    pStatsReq = (tpAniGetStatsReq) pMsgBuf;

    if((psessionEntry = peFindSessionByBssid(pMac,pStatsReq->bssId,&sessionId))== NULL)
    {
        limLog(pMac, LOGE, FL("session does not exist for given bssId"));
        vos_mem_free( pMsgBuf );
        pMsgBuf = NULL;
        return;
    }



    switch(pStatsReq->msgType)
    {
        //Add Lim stats here. and send reqsponse.

        //HAL maintained Stats.
        case eWNI_SME_STA_STAT_REQ:
            msgQ.type = WDA_STA_STAT_REQ;
            break;
        case eWNI_SME_AGGR_STAT_REQ:
            msgQ.type = WDA_AGGR_STAT_REQ;
            break;
        case eWNI_SME_GLOBAL_STAT_REQ:
            msgQ.type = WDA_GLOBAL_STAT_REQ;
            break;
        case eWNI_SME_STAT_SUMM_REQ:
            msgQ.type = WDA_STAT_SUMM_REQ;
            break;
        default: //Unknown request.
            PELOGE(limLog(pMac, LOGE, "Unknown Statistics request");)
            vos_mem_free( pMsgBuf );
            pMsgBuf = NULL;
            return;
    }

    msgQ.reserved = 0;
    msgQ.bodyptr = pMsgBuf;
    msgQ.bodyval = 0;
    if(NULL == psessionEntry)
    {
        MTRACE(macTraceMsgTx(pMac, NO_SESSION, msgQ.type));
    }
    else
    {
        MTRACE(macTraceMsgTx(pMac, psessionEntry->peSessionId, msgQ.type));
    }
    if( eSIR_SUCCESS != (wdaPostCtrlMsg( pMac, &msgQ ))){
        limLog(pMac, LOGP, "Unable to forward request");
        vos_mem_free( pMsgBuf );
        pMsgBuf = NULL;
        return;
    }

    return;
}


/**
 * __limProcessSmeGetStatisticsRequest()
 *
 *FUNCTION:
 *
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */
static void
__limProcessSmeGetStatisticsRequest(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpAniGetPEStatsReq    pPEStatsReq;
    tSirMsgQ msgQ;

    pPEStatsReq = (tpAniGetPEStatsReq) pMsgBuf;

    msgQ.type = WDA_GET_STATISTICS_REQ;

    msgQ.reserved = 0;
    msgQ.bodyptr = pMsgBuf;
    msgQ.bodyval = 0;
    MTRACE(macTraceMsgTx(pMac, NO_SESSION, msgQ.type));

    if( eSIR_SUCCESS != (wdaPostCtrlMsg( pMac, &msgQ ))){
        vos_mem_free( pMsgBuf );
        pMsgBuf = NULL;
        limLog(pMac, LOGP, "Unable to forward request");
        return;
    }

    return;
}

#if defined(FEATURE_WLAN_ESE) && defined(FEATURE_WLAN_ESE_UPLOAD)
/**
 *FUNCTION: __limProcessSmeGetTsmStatsRequest()
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */
static void
__limProcessSmeGetTsmStatsRequest(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tSirMsgQ               msgQ;

    msgQ.type = WDA_TSM_STATS_REQ;
    msgQ.reserved = 0;
    msgQ.bodyptr = pMsgBuf;
    msgQ.bodyval = 0;
    MTRACE(macTraceMsgTx(pMac, NO_SESSION, msgQ.type));

    if( eSIR_SUCCESS != (wdaPostCtrlMsg( pMac, &msgQ ))){
        vos_mem_free( pMsgBuf );
        pMsgBuf = NULL;
        limLog(pMac, LOGP, "Unable to forward request");
        return;
    }
}
#endif /* FEATURE_WLAN_ESE && FEATURE_WLAN_ESE_UPLOAD */

static void
__limProcessSmeUpdateAPWPSIEs(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirUpdateAPWPSIEsReq  pUpdateAPWPSIEsReq;
    tpPESession             psessionEntry;
    tANI_U8                 sessionId;  //PE sessionID

    PELOG1(limLog(pMac, LOG1,
           FL("received UPDATE_APWPSIEs_REQ message")););

    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    pUpdateAPWPSIEsReq = vos_mem_malloc(sizeof(tSirUpdateAPWPSIEsReq));
    if ( NULL == pUpdateAPWPSIEsReq )
    {
        limLog(pMac, LOGP, FL("call to AllocateMemory failed for pUpdateAPWPSIEsReq"));
        return;
    }

    if ((limUpdateAPWPSIEsReqSerDes(pMac, pUpdateAPWPSIEsReq, (tANI_U8 *) pMsgBuf) == eSIR_FAILURE))
    {
        limLog(pMac, LOGW, FL("received invalid SME_SETCONTEXT_REQ message"));
        goto end;
    }

    if((psessionEntry = peFindSessionByBssid(pMac, pUpdateAPWPSIEsReq->bssId, &sessionId)) == NULL)
    {
        limLog(pMac, LOGW, FL("Session does not exist for given BSSID"));
        goto end;
    }

    vos_mem_copy( &psessionEntry->APWPSIEs, &pUpdateAPWPSIEsReq->APWPSIEs, sizeof(tSirAPWPSIEs));

    schSetFixedBeaconFields(pMac, psessionEntry);
    limSendBeaconInd(pMac, psessionEntry);

end:
    vos_mem_free( pUpdateAPWPSIEsReq);
    return;
} /*** end __limProcessSmeUpdateAPWPSIEs(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf) ***/

void
limSendVdevRestart(tpAniSirGlobal pMac,
                   tpPESession psessionEntry,
                   tANI_U8 sessionId)
{
    tpHalHiddenSsidVdevRestart pHalHiddenSsidVdevRestart = NULL;
    tSirMsgQ       msgQ;
    tSirRetStatus  retCode = eSIR_SUCCESS;

    if ( psessionEntry == NULL )
    {
        PELOGE(limLog(pMac, LOGE, "%s:%d: Invalid parameters", __func__, __LINE__ );)
        return;
    }

    pHalHiddenSsidVdevRestart = vos_mem_malloc(sizeof(tHalHiddenSsidVdevRestart));
    if (NULL == pHalHiddenSsidVdevRestart)
    {
        PELOGE(limLog(pMac, LOGE, "%s:%d: Unable to allocate memory", __func__, __LINE__ );)
        return;
    }

    pHalHiddenSsidVdevRestart->ssidHidden = psessionEntry->ssidHidden;
    pHalHiddenSsidVdevRestart->sessionId = sessionId;

    msgQ.type = WDA_HIDDEN_SSID_VDEV_RESTART;
    msgQ.bodyptr = pHalHiddenSsidVdevRestart;
    msgQ.bodyval = 0;

    retCode = wdaPostCtrlMsg(pMac, &msgQ);
    if (eSIR_SUCCESS != retCode)
    {
        PELOGE(limLog(pMac, LOGE, "%s:%d: wdaPostCtrlMsg() failed", __func__, __LINE__ );)
        vos_mem_free(pHalHiddenSsidVdevRestart);
    }
}

static void __lim_process_roam_scan_offload_req(tpAniSirGlobal mac_ctx,
	tANI_U32 *msg_buf)
{
	tpPESession pe_session;
	tSirMsgQ       wma_msg;
	tSirRetStatus  status;
	tSirRoamOffloadScanReq *msg, *req_buffer;

	msg = (tSirRoamOffloadScanReq *)msg_buf;
	pe_session = pe_find_session_by_sme_session_id(mac_ctx,
                       msg->sessionId);

	/* Set roaming_in_progress flag according to the command */
	if ( pe_session && (msg->Command == ROAM_SCAN_OFFLOAD_START ||
			msg->Command == ROAM_SCAN_OFFLOAD_RESTART ||
			msg->Command == ROAM_SCAN_OFFLOAD_STOP))
	pe_session->roaming_in_progress = false;

	req_buffer = vos_mem_malloc(sizeof(tSirRoamOffloadScanReq));
	if (NULL == req_buffer) {
		limLog(mac_ctx, LOGE,
			FL("Mem Alloc failed for req buffer"));
		return;
	}

	*req_buffer = *msg;

	vos_mem_zero(&wma_msg, sizeof(tSirMsgQ));
	wma_msg.type = WDA_ROAM_SCAN_OFFLOAD_REQ;
	wma_msg.bodyptr = req_buffer;

	status = wdaPostCtrlMsg(mac_ctx, &wma_msg);
	if (eSIR_SUCCESS != status)
	{
		limLog(mac_ctx, LOGE,
			FL("Posting WDA_ROAM_SCAN_OFFLOAD_REQ failed"));
			vos_mem_free(req_buffer);
	}
}
static void
__limProcessSmeHideSSID(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirUpdateParams       pUpdateParams;
    tpPESession             psessionEntry;

    PELOG1(limLog(pMac, LOG1,
           FL("received HIDE_SSID message")););

    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    pUpdateParams = (tpSirUpdateParams)pMsgBuf;

    if((psessionEntry = peFindSessionBySessionId(pMac, pUpdateParams->sessionId)) == NULL)
    {
        limLog(pMac, LOGW, "Session does not exist for given sessionId %d",
                      pUpdateParams->sessionId);
        return;
    }

    /* Update the session entry */
    psessionEntry->ssidHidden = pUpdateParams->ssidHidden;

    /* Send vdev restart */
    limSendVdevRestart(pMac, psessionEntry, pUpdateParams->sessionId);

    /* Update beacon */
    schSetFixedBeaconFields(pMac, psessionEntry);
    limSendBeaconInd(pMac, psessionEntry);

    return;
} /*** end __limProcessSmeHideSSID(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf) ***/

static void
__limProcessSmeSetWPARSNIEs(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirUpdateAPWPARSNIEsReq  pUpdateAPWPARSNIEsReq;
    tpPESession             psessionEntry;
    tANI_U8                 sessionId;  //PE sessionID

    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    pUpdateAPWPARSNIEsReq = vos_mem_malloc(sizeof(tSirUpdateAPWPSIEsReq));
    if ( NULL == pUpdateAPWPARSNIEsReq )
    {
        limLog(pMac, LOGP, FL("call to AllocateMemory failed for pUpdateAPWPARSNIEsReq"));
        return;
    }

    if ((limUpdateAPWPARSNIEsReqSerDes(pMac, pUpdateAPWPARSNIEsReq, (tANI_U8 *) pMsgBuf) == eSIR_FAILURE))
    {
        limLog(pMac, LOGW, FL("received invalid SME_SETCONTEXT_REQ message"));
        goto end;
    }

    if((psessionEntry = peFindSessionByBssid(pMac, pUpdateAPWPARSNIEsReq->bssId, &sessionId)) == NULL)
    {
        limLog(pMac, LOGW, FL("Session does not exist for given BSSID"));
        goto end;
    }

    vos_mem_copy(&psessionEntry->pLimStartBssReq->rsnIE,
                 &pUpdateAPWPARSNIEsReq->APWPARSNIEs, sizeof(tSirRSNie));

    limSetRSNieWPAiefromSmeStartBSSReqMessage(pMac, &psessionEntry->pLimStartBssReq->rsnIE, psessionEntry);

    psessionEntry->pLimStartBssReq->privacy = 1;
    psessionEntry->privacy = 1;

    schSetFixedBeaconFields(pMac, psessionEntry);
    limSendBeaconInd(pMac, psessionEntry);

end:
    vos_mem_free(pUpdateAPWPARSNIEsReq);
    return;
} /*** end __limProcessSmeSetWPARSNIEs(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf) ***/

/*
Update the beacon Interval dynamically if beaconInterval is different in MCC
*/
static void
__limProcessSmeChangeBI(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirChangeBIParams     pChangeBIParams;
    tpPESession             psessionEntry;
    tANI_U8  sessionId = 0;
    tUpdateBeaconParams beaconParams;

    PELOG1(limLog(pMac, LOG1,
           FL("received Update Beacon Interval message")););

    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    vos_mem_zero(&beaconParams, sizeof(tUpdateBeaconParams));
    pChangeBIParams = (tpSirChangeBIParams)pMsgBuf;

    if((psessionEntry = peFindSessionByBssid(pMac, pChangeBIParams->bssId, &sessionId)) == NULL)
    {
        limLog(pMac, LOGE, FL("Session does not exist for given BSSID"));
        return;
    }

    /*Update sessionEntry Beacon Interval*/
    if(psessionEntry->beaconParams.beaconInterval !=
                                        pChangeBIParams->beaconInterval )
    {
       psessionEntry->beaconParams.beaconInterval = pChangeBIParams->beaconInterval;
    }

    /*Update sch beaconInterval*/
    if(pMac->sch.schObject.gSchBeaconInterval !=
                                        pChangeBIParams->beaconInterval )
    {
        pMac->sch.schObject.gSchBeaconInterval = pChangeBIParams->beaconInterval;

        PELOG1(limLog(pMac, LOG1,
               FL("LIM send update BeaconInterval Indication : %d"),pChangeBIParams->beaconInterval););

        if (VOS_FALSE == pMac->sap.SapDfsInfo.is_dfs_cac_timer_running)
        {
            /* Update beacon */
            schSetFixedBeaconFields(pMac, psessionEntry);

            beaconParams.bssIdx = psessionEntry->bssIdx;
            //Set change in beacon Interval
            beaconParams.beaconInterval = pChangeBIParams->beaconInterval;
            beaconParams.paramChangeBitmap = PARAM_BCN_INTERVAL_CHANGED;
            limSendBeaconParams(pMac, &beaconParams, psessionEntry);
        }
    }

    return;
} /*** end __limProcessSmeChangeBI(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf) ***/

#ifdef QCA_HT_2040_COEX
static void __limProcessSmeSetHT2040Mode(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirSetHT2040Mode     pSetHT2040Mode;
    tpPESession             psessionEntry;
    tANI_U8  sessionId = 0;
    vos_msg_t msg;
    tUpdateVHTOpMode *pHtOpMode = NULL;
    tANI_U16                staId = 0;
    tpDphHashNode pStaDs = NULL;

    PELOG1(limLog(pMac, LOG1,
           FL("received Set HT 20/40 mode message")););
    if(pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    pSetHT2040Mode = (tpSirSetHT2040Mode)pMsgBuf;

    if((psessionEntry = peFindSessionByBssid(pMac, pSetHT2040Mode->bssId,
                                                   &sessionId)) == NULL)
    {
        limLog(pMac, LOG1, FL("Session does not exist for given BSSID "));
        limPrintMacAddr(pMac, pSetHT2040Mode->bssId, LOG1);
        return;
    }

    limLog(pMac, LOG1, FL("Update session entry for cbMod=%d"),
                           pSetHT2040Mode->cbMode);
    /*Update sessionEntry HT related fields*/
    switch(pSetHT2040Mode->cbMode)
    {
    case PHY_SINGLE_CHANNEL_CENTERED:
        psessionEntry->htSecondaryChannelOffset = PHY_SINGLE_CHANNEL_CENTERED;
        psessionEntry->htRecommendedTxWidthSet = 0;
        if (pSetHT2040Mode->obssEnabled)
            psessionEntry->htSupportedChannelWidthSet = eHT_CHANNEL_WIDTH_40MHZ;
        else
            psessionEntry->htSupportedChannelWidthSet = eHT_CHANNEL_WIDTH_20MHZ;
        break;
    case PHY_DOUBLE_CHANNEL_LOW_PRIMARY:
        psessionEntry->htSecondaryChannelOffset = PHY_DOUBLE_CHANNEL_LOW_PRIMARY;
        psessionEntry->htRecommendedTxWidthSet = 1;
        break;
    case PHY_DOUBLE_CHANNEL_HIGH_PRIMARY:
        psessionEntry->htSecondaryChannelOffset = PHY_DOUBLE_CHANNEL_HIGH_PRIMARY;
        psessionEntry->htRecommendedTxWidthSet = 1;
        break;
    default:
        limLog(pMac, LOGE,FL("Invalid cbMode"));
        return;
    }

    /* Update beacon */
    schSetFixedBeaconFields(pMac, psessionEntry);
    limSendBeaconInd(pMac, psessionEntry);

    /* update OP Mode for each associated peer */
    for (staId = 0; staId < psessionEntry->dph.dphHashTable.size; staId++)
    {
        pStaDs = dphGetHashEntry(pMac, staId, &psessionEntry->dph.dphHashTable);
        if (NULL == pStaDs)
            continue;

        if (pStaDs->valid && pStaDs->htSupportedChannelWidthSet)
        {
            pHtOpMode = vos_mem_malloc(sizeof(tUpdateVHTOpMode));
            if ( NULL == pHtOpMode )
            {
                limLog(pMac, LOGE,
                      FL("%s: Not able to allocate memory for setting OP mode"),
                      __func__);
                return;
            }
            pHtOpMode->opMode = (psessionEntry->htSecondaryChannelOffset ==
                          PHY_SINGLE_CHANNEL_CENTERED)?
                          eHT_CHANNEL_WIDTH_20MHZ:eHT_CHANNEL_WIDTH_40MHZ;
            pHtOpMode->staId = staId;
            vos_mem_copy(pHtOpMode->peer_mac, &pStaDs->staAddr,
                 sizeof(tSirMacAddr));
            pHtOpMode->smesessionId = sessionId;

            msg.type     = WDA_UPDATE_OP_MODE;
            msg.reserved = 0;
            msg.bodyptr  = pHtOpMode;
            if (!VOS_IS_STATUS_SUCCESS(
                     vos_mq_post_message(VOS_MODULE_ID_WDA, &msg)))
            {
                 limLog(pMac, LOGE,
                   FL("%s: Not able to post WDA_UPDATE_OP_MODE message to WDA"),
                   __func__);
                 vos_mem_free(pHtOpMode);
                 return;
            }
            limLog(pMac, LOG1,
                        FL("%s: Notifed FW about OP mode: %d for staId=%d"),
                        __func__, pHtOpMode->opMode, staId);

         }
         else
            limLog(pMac, LOG1, FL("%s: station %d does not support HT40\n"),
                        __func__, staId);
    }

    return;
}
#endif

/** -------------------------------------------------------------
\fn limProcessSmeDelBaPeerInd
\brief handles indication message from HDD to send delete BA request
\param   tpAniSirGlobal pMac
\param   tANI_U32 pMsgBuf
\return None
-------------------------------------------------------------*/
void
limProcessSmeDelBaPeerInd(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tANI_U16            assocId =0;
    tpSmeDelBAPeerInd   pSmeDelBAPeerInd = (tpSmeDelBAPeerInd)pMsgBuf;
    tpDphHashNode       pSta;
    tpPESession         psessionEntry;
    tANI_U8             sessionId;



    if(NULL == pSmeDelBAPeerInd)
        return;

    if ((psessionEntry = peFindSessionByBssid(pMac,pSmeDelBAPeerInd->bssId,&sessionId))==NULL)
    {
        limLog(pMac, LOGE,FL("session does not exist for given bssId"));
        return;
    }
    limLog(pMac, LOGW, FL("called with staId = %d, tid = %d, baDirection = %d"),
              pSmeDelBAPeerInd->staIdx, pSmeDelBAPeerInd->baTID, pSmeDelBAPeerInd->baDirection);

    pSta = dphLookupAssocId(pMac, pSmeDelBAPeerInd->staIdx, &assocId, &psessionEntry->dph.dphHashTable);
    if( eSIR_SUCCESS != limPostMlmDelBAReq( pMac,
          pSta,
          pSmeDelBAPeerInd->baDirection,
          pSmeDelBAPeerInd->baTID,
          eSIR_MAC_UNSPEC_FAILURE_REASON,psessionEntry))
    {
      limLog( pMac, LOGW,
          FL( "Failed to post LIM_MLM_DELBA_REQ to " ));
      if (pSta)
          limPrintMacAddr(pMac, pSta->staAddr, LOGW);
    }
}

// --------------------------------------------------------------------
/**
 * __limProcessReportMessage
 *
 * FUNCTION:  Processes the next received Radio Resource Management message
 *
 * LOGIC:
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 *
 * @param None
 * @return None
 */

void __limProcessReportMessage(tpAniSirGlobal pMac, tpSirMsgQ pMsg)
{
#ifdef WLAN_FEATURE_VOWIFI
   switch (pMsg->type)
   {
      case eWNI_SME_NEIGHBOR_REPORT_REQ_IND:
         rrmProcessNeighborReportReq( pMac, pMsg->bodyptr );
         break;
      case eWNI_SME_BEACON_REPORT_RESP_XMIT_IND:
        {
#if defined(FEATURE_WLAN_ESE) && !defined(FEATURE_WLAN_ESE_UPLOAD)
         tpSirBeaconReportXmitInd pBcnReport=NULL;
         tpPESession psessionEntry=NULL;
         tANI_U8 sessionId;

         if(pMsg->bodyptr == NULL)
         {
            limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
            return;
         }
         pBcnReport = (tpSirBeaconReportXmitInd )pMsg->bodyptr;
         if((psessionEntry = peFindSessionByBssid(pMac, pBcnReport->bssId,&sessionId))== NULL)
         {
            limLog(pMac, LOGE, "Session Does not exist for given bssId");
            return;
         }
         if (psessionEntry->isESEconnection)
             eseProcessBeaconReportXmit( pMac, pMsg->bodyptr);
         else
#endif
             rrmProcessBeaconReportXmit( pMac, pMsg->bodyptr );
        }
        break;
      default:
        limLog(pMac, LOGE, FL("Invalid msg type:%d"), pMsg->type);
   }
#endif
}

#if defined(FEATURE_WLAN_ESE) || defined(WLAN_FEATURE_VOWIFI)
// --------------------------------------------------------------------
/**
 * limSendSetMaxTxPowerReq
 *
 * FUNCTION:  Send SIR_HAL_SET_MAX_TX_POWER_REQ message to change the max tx power.
 *
 * LOGIC:
 *
 * ASSUMPTIONS:
 *
 * NOTE:
 *
 * @param txPower txPower to be set.
 * @param pSessionEntry session entry.
 * @return None
 */
tSirRetStatus
limSendSetMaxTxPowerReq ( tpAniSirGlobal pMac, tPowerdBm txPower, tpPESession pSessionEntry )
{
   tpMaxTxPowerParams pMaxTxParams = NULL;
   tSirRetStatus  retCode = eSIR_SUCCESS;
   tSirMsgQ       msgQ;

   if( pSessionEntry == NULL )
   {
      PELOGE(limLog(pMac, LOGE, "%s:%d: Inavalid parameters", __func__, __LINE__ );)
      return eSIR_FAILURE;
   }

   pMaxTxParams = vos_mem_malloc(sizeof(tMaxTxPowerParams));
   if ( NULL == pMaxTxParams )
   {
      limLog( pMac, LOGP, "%s:%d:Unable to allocate memory for pMaxTxParams ", __func__, __LINE__);
      return eSIR_MEM_ALLOC_FAILED;

   }
#if defined(WLAN_VOWIFI_DEBUG) || defined(FEATURE_WLAN_ESE)
   PELOG1(limLog( pMac, LOG1, "%s:%d: Allocated memory for pMaxTxParams...will be freed in other module", __func__, __LINE__ );)
#endif
   if( pMaxTxParams == NULL )
   {
      limLog( pMac, LOGE, "%s:%d: pMaxTxParams is NULL", __func__, __LINE__);
      return eSIR_FAILURE;
   }
   pMaxTxParams->power = txPower;
   vos_mem_copy( pMaxTxParams->bssId, pSessionEntry->bssId, sizeof(tSirMacAddr) );
   vos_mem_copy( pMaxTxParams->selfStaMacAddr, pSessionEntry->selfMacAddr, sizeof(tSirMacAddr) );

   msgQ.type = WDA_SET_MAX_TX_POWER_REQ;
   msgQ.bodyptr = pMaxTxParams;
   msgQ.bodyval = 0;
   PELOG1(limLog(pMac, LOG1, FL("Posting WDA_SET_MAX_TX_POWER_REQ to WDA"));)
   MTRACE(macTraceMsgTx(pMac, pSessionEntry->peSessionId, msgQ.type));
   retCode = wdaPostCtrlMsg(pMac, &msgQ);
   if (eSIR_SUCCESS != retCode)
   {
      PELOGE(limLog(pMac, LOGE, FL("wdaPostCtrlMsg() failed"));)
      vos_mem_free(pMaxTxParams);
   }
   return retCode;
}
#endif

/**
 * __limProcessSmeAddStaSelfReq()
 *
 *FUNCTION:
 * This function is called to process SME_ADD_STA_SELF_REQ message
 * from SME. It sends a SIR_HAL_ADD_STA_SELF_REQ message to HAL.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeAddStaSelfReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
   tSirMsgQ msg;
   tpAddStaSelfParams pAddStaSelfParams;
   tpSirSmeAddStaSelfReq pSmeReq = (tpSirSmeAddStaSelfReq) pMsgBuf;

   pAddStaSelfParams = vos_mem_malloc(sizeof(tAddStaSelfParams));
   if ( NULL == pAddStaSelfParams )
   {
      limLog( pMac, LOGP, FL("Unable to allocate memory for tAddSelfStaParams") );
      return;
   }

   vos_mem_copy( pAddStaSelfParams->selfMacAddr, pSmeReq->selfMacAddr, sizeof(tSirMacAddr) );
   pAddStaSelfParams->currDeviceMode = pSmeReq->currDeviceMode;
   pAddStaSelfParams->sessionId = pSmeReq->sessionId;
   pAddStaSelfParams->type = pSmeReq->type;
   pAddStaSelfParams->subType = pSmeReq->subType;
   pAddStaSelfParams->pkt_err_disconn_th = pSmeReq->pkt_err_disconn_th;
   pAddStaSelfParams->nss_2g = pSmeReq->nss_2g;
   pAddStaSelfParams->nss_5g = pSmeReq->nss_5g;

   msg.type = SIR_HAL_ADD_STA_SELF_REQ;
   msg.reserved = 0;
   msg.bodyptr =  pAddStaSelfParams;
   msg.bodyval = 0;

   PELOGW(limLog(pMac, LOG1, FL("sending SIR_HAL_ADD_STA_SELF_REQ msg to HAL"));)
      MTRACE(macTraceMsgTx(pMac, NO_SESSION, msg.type));

   if(eSIR_SUCCESS != wdaPostCtrlMsg(pMac, &msg))
   {
      limLog(pMac, LOGP, FL("wdaPostCtrlMsg failed"));
   }
   return;
} /*** end __limProcessAddStaSelfReq() ***/


/**
 * __limProcessSmeDelStaSelfReq()
 *
 *FUNCTION:
 * This function is called to process SME_DEL_STA_SELF_REQ message
 * from SME. It sends a SIR_HAL_DEL_STA_SELF_REQ message to HAL.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */

static void
__limProcessSmeDelStaSelfReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
   tSirMsgQ msg;
   tpDelStaSelfParams pDelStaSelfParams;
   tpSirSmeDelStaSelfReq pSmeReq = (tpSirSmeDelStaSelfReq) pMsgBuf;

   pDelStaSelfParams = vos_mem_malloc(sizeof( tDelStaSelfParams));
   if ( NULL == pDelStaSelfParams )
   {
      limLog( pMac, LOGP, FL("Unable to allocate memory for tDelStaSelfParams") );
      return;
   }

   vos_mem_copy( pDelStaSelfParams->selfMacAddr, pSmeReq->selfMacAddr, sizeof(tSirMacAddr) );

   pDelStaSelfParams->sessionId = pSmeReq->sessionId;
   msg.type = SIR_HAL_DEL_STA_SELF_REQ;
   msg.reserved = 0;
   msg.bodyptr =  pDelStaSelfParams;
   msg.bodyval = 0;

   PELOGW(limLog(pMac, LOG1,
          FL("sending SIR_HAL_DEL_STA_SELF_REQ msg to HAL"));)
   MTRACE(macTraceMsgTx(pMac, NO_SESSION, msg.type));

   if(eSIR_SUCCESS != wdaPostCtrlMsg(pMac, &msg))
   {
      limLog(pMac, LOGP, FL("wdaPostCtrlMsg failed"));
      vos_mem_free(pDelStaSelfParams);
   }
   return;
} /*** end __limProcessSmeDelStaSelfReq() ***/


/**
 * __limProcessSmeRegisterMgmtFrameReq()
 *
 *FUNCTION:
 * This function is called to process eWNI_SME_REGISTER_MGMT_FRAME_REQ message
 * from SME. It Register this information within PE.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return None
 */
static void
__limProcessSmeRegisterMgmtFrameReq(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    VOS_STATUS vosStatus;
    tpSirRegisterMgmtFrame pSmeReq = (tpSirRegisterMgmtFrame)pMsgBuf;
    tpLimMgmtFrameRegistration pLimMgmtRegistration = NULL, pNext = NULL;
    tANI_BOOLEAN match = VOS_FALSE;
    PELOG1(limLog(pMac, LOG1,
           FL("registerFrame %d, frameType %d, matchLen %d"),
            pSmeReq->registerFrame, pSmeReq->frameType, pSmeReq->matchLen);)

    /* First check whether entry exists already*/

    vos_list_peek_front(&pMac->lim.gLimMgmtFrameRegistratinQueue,
            (vos_list_node_t**)&pLimMgmtRegistration);

    while(pLimMgmtRegistration != NULL)
    {
        if (pLimMgmtRegistration->frameType == pSmeReq->frameType)
        {
            if(pSmeReq->matchLen)
            {
                if (pLimMgmtRegistration->matchLen == pSmeReq->matchLen)
                {
                    if (vos_mem_compare( pLimMgmtRegistration->matchData,
                                pSmeReq->matchData, pLimMgmtRegistration->matchLen))
                    {
                        /* found match! */
                        match = VOS_TRUE;
                        break;
                    }
                }
            }
            else
            {
                /* found match! */
                match = VOS_TRUE;
                break;
            }
        }
        vosStatus = vos_list_peek_next (
                &pMac->lim.gLimMgmtFrameRegistratinQueue,
                (vos_list_node_t*) pLimMgmtRegistration,
                (vos_list_node_t**) &pNext );

        pLimMgmtRegistration = pNext;
        pNext = NULL;

    }

    if (match)
    {
        vos_list_remove_node(&pMac->lim.gLimMgmtFrameRegistratinQueue,
                (vos_list_node_t*)pLimMgmtRegistration);
        vos_mem_free(pLimMgmtRegistration);
    }

    if(pSmeReq->registerFrame)
    {
        pLimMgmtRegistration = vos_mem_malloc(sizeof(tLimMgmtFrameRegistration) + pSmeReq->matchLen);
        if ( pLimMgmtRegistration != NULL)
        {
            vos_mem_set((void*)pLimMgmtRegistration,
                         sizeof(tLimMgmtFrameRegistration) + pSmeReq->matchLen, 0 );
            pLimMgmtRegistration->frameType = pSmeReq->frameType;
            pLimMgmtRegistration->matchLen  = pSmeReq->matchLen;
            pLimMgmtRegistration->sessionId = pSmeReq->sessionId;
            if(pSmeReq->matchLen)
            {
                vos_mem_copy(pLimMgmtRegistration->matchData,
                             pSmeReq->matchData, pSmeReq->matchLen);
            }
            vos_list_insert_front(&pMac->lim.gLimMgmtFrameRegistratinQueue,
                              &pLimMgmtRegistration->node);
        }
    }

    return;
} /*** end __limProcessSmeRegisterMgmtFrameReq() ***/

static tANI_BOOLEAN
__limInsertSingleShotNOAForScan(tpAniSirGlobal pMac, tANI_U32 noaDuration)
{
    tpP2pPsParams pMsgNoA;
    tSirMsgQ msg;

    pMsgNoA = vos_mem_malloc(sizeof( tP2pPsConfig ));
    if ( NULL == pMsgNoA )
    {
        limLog( pMac, LOGP,
                     FL( "Unable to allocate memory during NoA Update" ));
        goto error;
    }

    vos_mem_set((tANI_U8 *)pMsgNoA, sizeof(tP2pPsConfig), 0);
    /* Below params used for opp PS/periodic NOA and are don't care in this case - so initialized to 0 */
    pMsgNoA->opp_ps = 0;
    pMsgNoA->ctWindow = 0;
    pMsgNoA->duration = 0;
    pMsgNoA->interval = 0;
    pMsgNoA->count = 0;

    /* Below params used for Single Shot NOA - so assign proper values */
    pMsgNoA->psSelection = P2P_SINGLE_NOA;
    pMsgNoA->single_noa_duration = noaDuration;

    /* Start Insert NOA timer
     * If insert NOA req fails or NOA rsp fails or start NOA indication doesn't come from FW due to GO session deletion
     * or any other failure or reason, we still need to process the deferred SME req. The insert NOA
     * timer of 500 ms will ensure the stored SME req always gets processed
     */
    if (tx_timer_activate(&pMac->lim.limTimers.gLimP2pSingleShotNoaInsertTimer)
                                      == TX_TIMER_ERROR)
    {
        /// Could not activate Insert NOA timer.
        // Log error
        limLog(pMac, LOGP, FL("could not activate Insert Single Shot NOA during scan timer"));

        // send the scan response back with status failure and do not even call insert NOA
        limSendSmeScanRsp(pMac, sizeof(tSirSmeScanRsp), eSIR_SME_SCAN_FAILED, pMac->lim.gSmeSessionId, pMac->lim.gTransactionId);
        vos_mem_free(pMsgNoA);
        goto error;
    }

    MTRACE(macTrace(pMac, TRACE_CODE_TIMER_ACTIVATE, NO_SESSION, eLIM_INSERT_SINGLESHOT_NOA_TIMER));

    msg.type = WDA_SET_P2P_GO_NOA_REQ;
    msg.reserved = 0;
    msg.bodyptr = pMsgNoA;
    msg.bodyval = 0;

    if(eSIR_SUCCESS != wdaPostCtrlMsg(pMac, &msg))
    {
        /* In this failure case, timer is already started, so its expiration will take care of sending scan response */
        limLog(pMac, LOGP, FL("wdaPost Msg failed"));
        /* Deactivate the NOA timer in failure case */
        limDeactivateAndChangeTimer(pMac, eLIM_INSERT_SINGLESHOT_NOA_TIMER);
        goto error;
    }
    return FALSE;

error:
    /* In any of the failure cases, just go ahead with the processing of registered deferred SME request without
     * worrying about the NOA
     */
    limProcessRegdDefdSmeReqAfterNOAStart(pMac);
    // msg buffer is consumed and freed in above function so return FALSE
    return FALSE;

}

static void __limRegisterDeferredSmeReqForNOAStart(tpAniSirGlobal pMac, tANI_U16 msgType, tANI_U32 *pMsgBuf)
{
    limLog(pMac, LOG1, FL("Reg msgType %d"), msgType) ;
    pMac->lim.gDeferMsgTypeForNOA = msgType;
    pMac->lim.gpDefdSmeMsgForNOA = pMsgBuf;
}

static void __limDeregisterDeferredSmeReqAfterNOAStart(tpAniSirGlobal pMac)
{
    limLog(pMac, LOG1, FL("Dereg msgType %d"), pMac->lim.gDeferMsgTypeForNOA) ;
    pMac->lim.gDeferMsgTypeForNOA = 0;
    if (pMac->lim.gpDefdSmeMsgForNOA != NULL)
    {
        /* __limProcessSmeScanReq consumed the buffer. We can free it. */
        vos_mem_free(pMac->lim.gpDefdSmeMsgForNOA);
        pMac->lim.gpDefdSmeMsgForNOA = NULL;
    }
}

static
tANI_U32 limCalculateNOADuration(tpAniSirGlobal pMac, tANI_U16 msgType, tANI_U32 *pMsgBuf)
{
    tANI_U32 noaDuration = 0;

    switch (msgType)
    {
        case eWNI_SME_SCAN_REQ:
        {
            tANI_U32 val;
            tANI_U8 i;
            tpSirSmeScanReq pScanReq = (tpSirSmeScanReq) pMsgBuf;
            if (wlan_cfgGetInt(pMac, WNI_CFG_PASSIVE_MAXIMUM_CHANNEL_TIME, &val) != eSIR_SUCCESS)
            {
                /*
                 * Could not get max channel value
                 * from CFG. Log error.
                 */
                limLog(pMac, LOGP, FL("could not retrieve passive max channel value"));

                /* use a default value of 110ms */
                val = DEFAULT_PASSIVE_MAX_CHANNEL_TIME;
            }

            for (i = 0; i < pScanReq->channelList.numChannels; i++) {
                tANI_U8 channelNum = pScanReq->channelList.channelNumber[i];

                if (limActiveScanAllowed(pMac, channelNum)) {
                    /* Use min + max channel time to calculate the total duration of scan */
                    noaDuration += pScanReq->minChannelTime + pScanReq->maxChannelTime;
                } else {
                    /* using the value from WNI_CFG_PASSIVE_MINIMUM_CHANNEL_TIME as is done in
                     * void limContinuePostChannelScan(tpAniSirGlobal pMac)
                     */
                    noaDuration += val;
                }
            }

            /* Adding an overhead of 20ms to account for the scan messaging delays */
            noaDuration += SCAN_MESSAGING_OVERHEAD;
            noaDuration *= CONV_MS_TO_US;

            break;
        }

#ifdef FEATURE_OEM_DATA_SUPPORT
        case eWNI_SME_OEM_DATA_REQ:
            noaDuration = OEM_DATA_NOA_DURATION*CONV_MS_TO_US; // use 60 msec as default
            break;
#endif

        case eWNI_SME_REMAIN_ON_CHANNEL_REQ:
        {
            tSirRemainOnChnReq *pRemainOnChnReq = (tSirRemainOnChnReq *) pMsgBuf;
            noaDuration = (pRemainOnChnReq->duration)*CONV_MS_TO_US;
            break;
        }

        case eWNI_SME_JOIN_REQ:
            noaDuration = JOIN_NOA_DURATION*CONV_MS_TO_US;
            break;

        default:
            noaDuration = 0;
            break;

    }
    limLog(pMac, LOGW, FL("msgType %d noa %d"), msgType, noaDuration);
    return noaDuration;
}

void limProcessRegdDefdSmeReqAfterNOAStart(tpAniSirGlobal pMac)
{
    tANI_BOOLEAN bufConsumed = TRUE;

    limLog(pMac, LOG1, FL("Process defd sme req %d"), pMac->lim.gDeferMsgTypeForNOA);
    if ( (pMac->lim.gDeferMsgTypeForNOA != 0) &&
         (pMac->lim.gpDefdSmeMsgForNOA != NULL) )
    {
        switch (pMac->lim.gDeferMsgTypeForNOA)
        {
            case eWNI_SME_SCAN_REQ:
                __limProcessSmeScanReq(pMac, pMac->lim.gpDefdSmeMsgForNOA);
                break;
#ifdef FEATURE_OEM_DATA_SUPPORT
            case eWNI_SME_OEM_DATA_REQ:
                __limProcessSmeOemDataReq(pMac, pMac->lim.gpDefdSmeMsgForNOA);
                break;
#endif
            case eWNI_SME_REMAIN_ON_CHANNEL_REQ:
                bufConsumed = limProcessRemainOnChnlReq(pMac, pMac->lim.gpDefdSmeMsgForNOA);
                /* limProcessRemainOnChnlReq doesnt want us to free the buffer since
                 * it is freed in limRemainOnChnRsp. this change is to avoid "double free"
                 */
                if (FALSE == bufConsumed)
                {
                    pMac->lim.gpDefdSmeMsgForNOA = NULL;
                }
                break;
            case eWNI_SME_JOIN_REQ:
                __limProcessSmeJoinReq(pMac, pMac->lim.gpDefdSmeMsgForNOA);
                break;
            default:
                limLog(pMac, LOGE, FL("Unknown deferred msg type %d"), pMac->lim.gDeferMsgTypeForNOA);
                break;
        }
        __limDeregisterDeferredSmeReqAfterNOAStart(pMac);
    }
    else
    {
        limLog( pMac, LOGW, FL("start received from FW when no sme deferred msg pending. Do nothing."
            "It might happen sometime when NOA start ind and timeout happen at the same time"));
    }
}


static void
__limProcessSmeResetApCapsChange(tpAniSirGlobal pMac, tANI_U32 *pMsgBuf)
{
    tpSirResetAPCapsChange pResetCapsChange;
    tpPESession             psessionEntry;
    tANI_U8  sessionId = 0;
    if (pMsgBuf == NULL)
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    pResetCapsChange = (tpSirResetAPCapsChange)pMsgBuf;
    psessionEntry = peFindSessionByBssid(pMac, pResetCapsChange->bssId, &sessionId);
    if (psessionEntry == NULL)
    {
        limLog(pMac, LOGE, FL("Session does not exist for given BSSID"));
        return;
    }

    psessionEntry->limSentCapsChangeNtf = false;
    return;
}

/* lim_register_p2p_ack_ind_cb() - Save the p2p ack indication callback.
 * @mac_ctx: Mac pointer
 * @msg_buf: Msg pointer containing the callback
 *
 * This function is used to save the p2p ack indication callback in PE.
 *
 * Return: None
 */
static void lim_register_p2p_ack_ind_cb(tpAniSirGlobal mac_ctx,
				uint32_t *msg_buf)
{
	struct sir_sme_p2p_ack_ind_cb_req *sme_req =
		(struct sir_sme_p2p_ack_ind_cb_req *)msg_buf;

	if (NULL == msg_buf) {
		limLog(mac_ctx, LOGE, FL("msg_buf is null"));
		return;
	}
	if (sme_req->callback)
		mac_ctx->p2p_ack_ind_cb =
				sme_req->callback;
	else
		limLog(mac_ctx, LOGE, FL("sme_req->callback is null"));
}

/**
 * lim_register_mgmt_frame_ind_cb() - Save the Management frame
 * indication callback in PE.
 * @mac_ctx: Mac pointer
 * @msg_buf: Msg pointer containing the callback
 *
 * This function is used save the Management frame
 * indication callback in PE.
 *
 * Return: None
 */
static void lim_register_mgmt_frame_ind_cb(tpAniSirGlobal mac_ctx,
							uint32_t *msg_buf)
{
	struct sir_sme_mgmt_frame_cb_req *sme_req =
		(struct sir_sme_mgmt_frame_cb_req *)msg_buf;

	if (NULL == msg_buf) {
		limLog(mac_ctx, LOGE, FL("msg_buf is null"));
		return;
	}
	if (sme_req->callback)
		mac_ctx->mgmt_frame_ind_cb =
				sme_req->callback;
	else
		limLog(mac_ctx, LOGE, FL("sme_req->callback is null"));
}

/**
 * limProcessSmeReqMessages()
 *
 *FUNCTION:
 * This function is called by limProcessMessageQueue(). This
 * function processes SME request messages from HDD or upper layer
 * application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  msgType   Indicates the SME message type
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return Boolean - TRUE - if pMsgBuf is consumed and can be freed.
 *                   FALSE - if pMsgBuf is not to be freed.
 */

tANI_BOOLEAN
limProcessSmeReqMessages(tpAniSirGlobal pMac, tpSirMsgQ pMsg)
{
    tANI_BOOLEAN bufConsumed = TRUE; //Set this flag to false within case block of any following message, that doesnt want pMsgBuf to be freed.
    tANI_U32 *pMsgBuf = pMsg->bodyptr;
    tpSirSmeScanReq     pScanReq;
    PELOG1(limLog(pMac, LOG1, FL("LIM Received Global LimMlmState: %s(%d)"),
         limMlmStateStr(pMac->lim.gLimMlmState), pMac->lim.gLimMlmState );)

    pScanReq = (tpSirSmeScanReq) pMsgBuf;
    /* Special handling of some SME Req msgs where we have an existing GO session and
     * want to insert NOA before processing those msgs. These msgs will be processed later when
     * start event happens
     */
    MTRACE(macTraceMsgRx(pMac, NO_SESSION, pMsg->type));
    switch (pMsg->type)
    {
        case eWNI_SME_SCAN_REQ:
        case eWNI_SME_REMAIN_ON_CHANNEL_REQ:

            /* If scan is disabled return from here
             */
            if (pMac->lim.fScanDisabled)
            {
                if (pMsg->type == eWNI_SME_SCAN_REQ)
                {
                   limSendSmeScanRsp(pMac,
                                     offsetof(tSirSmeScanRsp,bssDescription[0]),
                                     eSIR_SME_INVALID_PARAMETERS,
                                     pScanReq->sessionId,
                                     pScanReq->transactionId);

                   bufConsumed = TRUE;
                }
                else if (pMsg->type == eWNI_SME_REMAIN_ON_CHANNEL_REQ)
                {
                    pMac->lim.gpDefdSmeMsgForNOA = NULL;
                    pMac->lim.gpLimRemainOnChanReq = (tpSirRemainOnChnReq )pMsgBuf;
                    limRemainOnChnRsp(pMac,eHAL_STATUS_FAILURE, NULL);

                    /*
                     * limRemainOnChnRsp will free the buffer this change is to
                     * avoid "double free"
                     */
                    bufConsumed = FALSE;
                }

                limLog(pMac, LOGE,
                      FL("Error: Scan Disabled."
                      " Return with error status for SME Message type(%d)"),
                      pMsg->type);

                return bufConsumed;
            }
            /*
             * Do not add BREAK here
             */
#ifdef FEATURE_OEM_DATA_SUPPORT
        case eWNI_SME_OEM_DATA_REQ:
#endif
        case eWNI_SME_JOIN_REQ:
            /* If we have an existing P2P GO session we need to insert NOA before actually process this SME Req */
            if (!pMac->fScanOffload && (limIsNOAInsertReqd(pMac) == TRUE) &&
                IS_FEATURE_SUPPORTED_BY_FW(P2P_GO_NOA_DECOUPLE_INIT_SCAN))
            {
                tANI_U32 noaDuration;
                __limRegisterDeferredSmeReqForNOAStart(pMac, pMsg->type, pMsgBuf);
                noaDuration = limCalculateNOADuration(pMac, pMsg->type, pMsgBuf);
                bufConsumed = __limInsertSingleShotNOAForScan(pMac, noaDuration);
                return bufConsumed;
            }
    }
    /* If no insert NOA required then execute the code below */

    switch (pMsg->type)
    {
        case eWNI_SME_START_REQ:
            __limProcessSmeStartReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_SYS_READY_IND:
            bufConsumed = __limProcessSmeSysReadyInd(pMac, pMsgBuf);
            break;

        case eWNI_SME_START_BSS_REQ:
            bufConsumed = __limProcessSmeStartBssReq(pMac, pMsg);
            break;

        case eWNI_SME_SCAN_REQ:
            __limProcessSmeScanReq(pMac, pMsgBuf);
            break;

#ifdef FEATURE_OEM_DATA_SUPPORT
        case eWNI_SME_OEM_DATA_REQ:
            __limProcessSmeOemDataReq(pMac, pMsgBuf);
            break;
#endif
        case eWNI_SME_REMAIN_ON_CHANNEL_REQ:
            bufConsumed = limProcessRemainOnChnlReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_UPDATE_NOA:
            __limProcessSmeNoAUpdate(pMac, pMsgBuf);
            break;
        case eWNI_SME_CLEAR_DFS_CHANNEL_LIST:
            __limProcessClearDfsChannelList(pMac, pMsg);
            break;
        case eWNI_SME_CLEAR_LIM_SCAN_CACHE:
            limReInitScanResults(pMac);
            break;
        case eWNI_SME_JOIN_REQ:
            __limProcessSmeJoinReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_REASSOC_REQ:
            __limProcessSmeReassocReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_DISASSOC_REQ:
            __limProcessSmeDisassocReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_DISASSOC_CNF:
        case eWNI_SME_DEAUTH_CNF:
            __limProcessSmeDisassocCnf(pMac, pMsgBuf);
            break;

        case eWNI_SME_DEAUTH_REQ:
            __limProcessSmeDeauthReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_SETCONTEXT_REQ:
            __limProcessSmeSetContextReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_REMOVEKEY_REQ:
            __limProcessSmeRemoveKeyReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_STOP_BSS_REQ:
            bufConsumed = __limProcessSmeStopBssReq(pMac, pMsg);
            break;

        case eWNI_SME_ASSOC_CNF:
        case eWNI_SME_REASSOC_CNF:
            if (pMsg->type == eWNI_SME_ASSOC_CNF)
                PELOG1(limLog(pMac, LOG1, FL("Received ASSOC_CNF message"));)
            else
                PELOG1(limLog(pMac, LOG1, FL("Received REASSOC_CNF message"));)

#ifdef SAP_AUTH_OFFLOAD
            if (pMac->sap_auth_offload) {
                __lim_process_sme_assoc_offload_cnf(pMac, pMsg->type, pMsgBuf);
            } else {
                __limProcessSmeAssocCnfNew(pMac, pMsg->type, pMsgBuf);
            }
#else
            __limProcessSmeAssocCnfNew(pMac, pMsg->type, pMsgBuf);
#endif /* SAP_AUTH_OFFLOAD */
            break;

        case eWNI_SME_ADDTS_REQ:
            PELOG1(limLog(pMac, LOG1, FL("Received ADDTS_REQ message"));)
            __limProcessSmeAddtsReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_DELTS_REQ:
            PELOG1(limLog(pMac, LOG1, FL("Received DELTS_REQ message"));)
            __limProcessSmeDeltsReq(pMac, pMsgBuf);
            break;

        case SIR_LIM_ADDTS_RSP_TIMEOUT:
            PELOG1(limLog(pMac, LOG1, FL("Received SIR_LIM_ADDTS_RSP_TIMEOUT message "));)
            limProcessSmeAddtsRspTimeout(pMac, pMsg->bodyval);
            break;

        case eWNI_SME_STA_STAT_REQ:
        case eWNI_SME_AGGR_STAT_REQ:
        case eWNI_SME_GLOBAL_STAT_REQ:
        case eWNI_SME_STAT_SUMM_REQ:
            __limProcessSmeStatsRequest( pMac, pMsgBuf);
            //HAL consumes pMsgBuf. It will be freed there. Set bufConsumed to false.
            bufConsumed = FALSE;
            break;
        case eWNI_SME_GET_STATISTICS_REQ:
            __limProcessSmeGetStatisticsRequest( pMac, pMsgBuf);
            //HAL consumes pMsgBuf. It will be freed there. Set bufConsumed to false.
            bufConsumed = FALSE;
            break;
#if defined(FEATURE_WLAN_ESE) && defined(FEATURE_WLAN_ESE_UPLOAD)
        case eWNI_SME_GET_TSM_STATS_REQ:
            __limProcessSmeGetTsmStatsRequest( pMac, pMsgBuf);
            bufConsumed = FALSE;
            break;
#endif /* FEATURE_WLAN_ESE && FEATURE_WLAN_ESE_UPLOAD */
        case eWNI_SME_DEL_BA_PEER_IND:
            limProcessSmeDelBaPeerInd(pMac, pMsgBuf);
            break;
        case eWNI_SME_GET_SCANNED_CHANNEL_REQ:
            limProcessSmeGetScanChannelInfo(pMac, pMsgBuf);
            break;
        case eWNI_SME_GET_ASSOC_STAS_REQ:
            limProcessSmeGetAssocSTAsInfo(pMac, pMsgBuf);
            break;
        case eWNI_SME_TKIP_CNTR_MEAS_REQ:
            limProcessTkipCounterMeasures(pMac, pMsgBuf);
            break;
        case eWNI_SME_EXT_CHANGE_CHANNEL:
            lim_process_ext_change_channel(pMac, pMsgBuf);
            break;
       case eWNI_SME_HIDE_SSID_REQ:
            __limProcessSmeHideSSID(pMac, pMsgBuf);
            break;
       case eWNI_SME_ROAM_SCAN_OFFLOAD_REQ:
            __lim_process_roam_scan_offload_req(pMac, pMsgBuf);
            break;
       case eWNI_SME_UPDATE_APWPSIE_REQ:
            __limProcessSmeUpdateAPWPSIEs(pMac, pMsgBuf);
            break;
        case eWNI_SME_GET_WPSPBC_SESSION_REQ:
             limProcessSmeGetWPSPBCSessions(pMac, pMsgBuf);
             break;

        case eWNI_SME_SET_APWPARSNIEs_REQ:
              __limProcessSmeSetWPARSNIEs(pMac, pMsgBuf);
              break;

        case eWNI_SME_CHNG_MCC_BEACON_INTERVAL:
             //Update the beaconInterval
             __limProcessSmeChangeBI(pMac, pMsgBuf );
             break;

#ifdef QCA_HT_2040_COEX
        case eWNI_SME_SET_HT_2040_MODE:
             __limProcessSmeSetHT2040Mode(pMac, pMsgBuf);
             break;
#endif

#if defined WLAN_FEATURE_VOWIFI
        case eWNI_SME_NEIGHBOR_REPORT_REQ_IND:
        case eWNI_SME_BEACON_REPORT_RESP_XMIT_IND:
            __limProcessReportMessage(pMac, pMsg);
            break;
#endif

#if defined WLAN_FEATURE_VOWIFI_11R
       case eWNI_SME_FT_PRE_AUTH_REQ:
            bufConsumed = (tANI_BOOLEAN)limProcessFTPreAuthReq(pMac, pMsg);
            break;
       case eWNI_SME_FT_UPDATE_KEY:
            limProcessFTUpdateKey(pMac, pMsgBuf);
            break;

       case eWNI_SME_FT_AGGR_QOS_REQ:
            limProcessFTAggrQosReq(pMac, pMsgBuf);
            break;
#endif

#if defined(FEATURE_WLAN_ESE) && !defined(FEATURE_WLAN_ESE_UPLOAD)
       case eWNI_SME_ESE_ADJACENT_AP_REPORT:
            limProcessAdjacentAPRepMsg ( pMac, pMsgBuf );
            break;
#endif
       case eWNI_SME_ADD_STA_SELF_REQ:
            __limProcessSmeAddStaSelfReq( pMac, pMsgBuf );
            break;
        case eWNI_SME_DEL_STA_SELF_REQ:
            __limProcessSmeDelStaSelfReq( pMac, pMsgBuf );
            break;

        case eWNI_SME_REGISTER_MGMT_FRAME_REQ:
            __limProcessSmeRegisterMgmtFrameReq( pMac, pMsgBuf );
            break;
#ifdef FEATURE_WLAN_TDLS
        case eWNI_SME_TDLS_SEND_MGMT_REQ:
            limProcessSmeTdlsMgmtSendReq(pMac, pMsgBuf);
            break;
        case eWNI_SME_TDLS_ADD_STA_REQ:
            limProcessSmeTdlsAddStaReq(pMac, pMsgBuf);
            break;
        case eWNI_SME_TDLS_DEL_STA_REQ:
            limProcessSmeTdlsDelStaReq(pMac, pMsgBuf);
            break;
        case eWNI_SME_TDLS_LINK_ESTABLISH_REQ:
            limProcesSmeTdlsLinkEstablishReq(pMac, pMsgBuf);
            break;
#endif
        case eWNI_SME_RESET_AP_CAPS_CHANGED:
            __limProcessSmeResetApCapsChange(pMac, pMsgBuf);
            break;

        case eWNI_SME_SET_TX_POWER_REQ:
            limSendSetTxPowerReq(pMac,  pMsgBuf);
            break ;

        case eWNI_SME_CHANNEL_CHANGE_REQ:
            limProcessSmeChannelChangeRequest(pMac, pMsgBuf);
            break;

        case eWNI_SME_START_BEACON_REQ:
            limProcessSmeStartBeaconReq(pMac, pMsgBuf);
            break;

        case eWNI_SME_DFS_BEACON_CHAN_SW_IE_REQ:
            limProcessSmeDfsCsaIeRequest(pMac, pMsgBuf);
            break;

        case eWNI_SME_UPDATE_ADDITIONAL_IES:
            limProcessUpdateAddIEs(pMac, pMsgBuf);
            break;

        case eWNI_SME_MODIFY_ADDITIONAL_IES:
            limProcessModifyAddIEs(pMac, pMsgBuf);
            break;

        case eWNI_SME_PDEV_SET_HT_VHT_IE:
            lim_process_set_pdev_IEs(pMac, pMsgBuf);
            break;
        case eWNI_SME_REGISTER_MGMT_FRAME_CB:
            lim_register_mgmt_frame_ind_cb(pMac, pMsgBuf);
            break;
        case eWNI_SME_REGISTER_P2P_ACK_CB:
            lim_register_p2p_ack_ind_cb(pMac, pMsgBuf);
            break;
        default:
            vos_mem_free((v_VOID_t*)pMsg->bodyptr);
            pMsg->bodyptr = NULL;
            break;
    } // switch (msgType)

    return bufConsumed;
} /*** end limProcessSmeReqMessages() ***/

/**
 * limProcessSmeStartBeaconReq()
 *
 *FUNCTION:
 * This function is called by limProcessMessageQueue(). This
 * function processes SME request messages from HDD or upper layer
 * application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  msgType   Indicates the SME message type
 * @param  *pMsgBuf  A pointer to the SME message buffer
 * @return Boolean - TRUE - if pMsgBuf is consumed and can be freed.
 *                   FALSE - if pMsgBuf is not to be freed.
 */
static void
limProcessSmeStartBeaconReq(tpAniSirGlobal pMac, tANI_U32 * pMsg)
{
    tpSirStartBeaconIndication pBeaconStartInd;
    tpPESession                psessionEntry;
    tANI_U8                    sessionId;  //PE sessionID

    if( pMsg == NULL )
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    pBeaconStartInd = (tpSirStartBeaconIndication)pMsg;
    if((psessionEntry =
              peFindSessionByBssid(pMac, pBeaconStartInd->bssid, &sessionId))
                  == NULL)
    {
        limPrintMacAddr(pMac,  pBeaconStartInd->bssid, LOGE);
        PELOGE(limLog(pMac, LOGE,
               "%s[%d]: Session does not exist for given bssId",
               __func__, __LINE__ );)
        return;
    }

    if (pBeaconStartInd->beaconStartStatus == VOS_TRUE)
    {
        /*
         * Currently this Indication comes from SAP
         * to start Beacon Tx on a DFS channel
         * since beaconing has to be done on DFS
         * channel only after CAC WAIT is completed.
         * On a DFS Channel LIM does not start beacon
         * Tx right after the WDA_ADD_BSS_RSP.
         */
        limApplyConfiguration(pMac,psessionEntry);
        VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_INFO,
               FL("Start Beacon with ssid %s Ch %d"),
               psessionEntry->ssId.ssId,
               psessionEntry->currentOperChannel);
        limSendBeaconInd(pMac, psessionEntry);
    }
    else
    {
        limLog(pMac, LOGE,FL("Invalid Beacon Start Indication"));
        return;
    }
}

static void
limProcessSmeChannelChangeRequest(tpAniSirGlobal pMac, tANI_U32 *pMsg)
{
    tpSirChanChangeRequest pChannelChangeReq;
    tpPESession             psessionEntry;
    tANI_U8                 sessionId;  //PE sessionID
    tPowerdBm               maxTxPwr;
    u_int32_t val = 0;
#ifdef WLAN_FEATURE_11AC
    tANI_U32 centerChan;
    tANI_U32 chanWidth;
#endif
    if( pMsg == NULL )
    {
        limLog(pMac, LOGE,FL("pMsg is NULL"));
        return;
    }
    pChannelChangeReq = (tpSirChanChangeRequest)pMsg;

    if((psessionEntry =
              peFindSessionByBssid(pMac, pChannelChangeReq->bssid, &sessionId))
                  == NULL)
    {
        limPrintMacAddr(pMac,  pChannelChangeReq->bssid, LOGE);
        PELOGE(limLog(pMac, LOGE,
               "%s[%d]: Session does not exist for given bssId",
               __func__, __LINE__ );)
        return;
    }

    if (LIM_IS_AP_ROLE(psessionEntry))
       psessionEntry->channelChangeReasonCode = LIM_SWITCH_CHANNEL_SAP_DFS;
    else
       psessionEntry->channelChangeReasonCode = LIM_SWITCH_CHANNEL_OPERATION;

    maxTxPwr = cfgGetRegulatoryMaxTransmitPower( pMac,
                            pChannelChangeReq->targetChannel );

    if (pChannelChangeReq->messageType == eWNI_SME_CHANNEL_CHANGE_REQ
                                              &&
                                     maxTxPwr != WDA_MAX_TXPOWER_INVALID)
    {

        /* Store the New Channel Params in psessionEntry */
        if (psessionEntry->currentOperChannel !=
                              pChannelChangeReq->targetChannel)
        {
            limLog(pMac, LOGE,
                   FL("switch old chnl %d --> new chnl %d and CH width - %d"),
                                 psessionEntry->currentOperChannel,
                                 pChannelChangeReq->targetChannel,
                                 pChannelChangeReq->vht_channel_width);


#ifdef WLAN_FEATURE_11AC
            if(psessionEntry->vhtCapability)
            {

                chanWidth = pChannelChangeReq->vht_channel_width;

                if(chanWidth == eHT_CHANNEL_WIDTH_20MHZ || chanWidth == eHT_CHANNEL_WIDTH_40MHZ)
                {
                    psessionEntry->vhtTxChannelWidthSet =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ;
                    psessionEntry->apChanWidth =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ;
                    /*
                     * In case of DFS operation, If AP falls back to lower
                     * bandwidth [< 80Mhz] then there is no need of
                     * Center freq segment. So reset it to zero.
                     */
                    if (cfgSetInt(pMac,
                                  WNI_CFG_VHT_CHANNEL_CENTER_FREQ_SEGMENT1,
                                  0)
                                  != eSIR_SUCCESS)
                    {
                        limLog(pMac, LOGP,
                        FL("couldn't reset center freq seg 0 in beacon"));
                    }
                    psessionEntry->apCenterChan = 0;
                }
                if (chanWidth == eHT_CHANNEL_WIDTH_80MHZ)
                {
                    psessionEntry->vhtTxChannelWidthSet =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ;
                    psessionEntry->apChanWidth =
                                           WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ;

                    centerChan = limGetCenterChannel(pMac, pChannelChangeReq->targetChannel,
                                    pChannelChangeReq->cbMode,WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ);
                    if(centerChan != eSIR_CFG_INVALID_ID)
                    {
                        limLog(pMac, LOGW, FL("***Center Channel for 80MHZ channel width = %d"),centerChan);
                        psessionEntry->apCenterChan = centerChan;
                        if (cfgSetInt(pMac, WNI_CFG_VHT_CHANNEL_CENTER_FREQ_SEGMENT1, centerChan)
                                                                     != eSIR_SUCCESS)
                        {
                            limLog(pMac, LOGP, FL("could not set  WNI_CFG_CHANNEL_BONDING_MODE at CFG"));
                        }
                    }
                }

            }
            psessionEntry->htSecondaryChannelOffset = limGetHTCBState(pChannelChangeReq->cbMode);
            psessionEntry->htSupportedChannelWidthSet = (pChannelChangeReq->cbMode ? 1 : 0);

            psessionEntry->htRecommendedTxWidthSet =
                                  psessionEntry->htSupportedChannelWidthSet;
            psessionEntry->currentOperChannel =
                                  pChannelChangeReq->targetChannel;
            psessionEntry->limRFBand =
                                limGetRFBand(psessionEntry->currentOperChannel);
            // Initialize 11h Enable Flag
            if (SIR_BAND_5_GHZ == psessionEntry->limRFBand)
            {
                if (wlan_cfgGetInt(pMac, WNI_CFG_11H_ENABLED, &val) !=
                                               eSIR_SUCCESS)
                    limLog(pMac, LOGP, FL("Fail to get WNI_CFG_11H_ENABLED "));
            }

            psessionEntry->lim11hEnable = val;
            psessionEntry->dot11mode = pChannelChangeReq->dot11mode;

            vos_mem_copy((void*)&psessionEntry->rateSet,
                                (void*)&pChannelChangeReq->operational_rateset,
                                sizeof(tSirMacRateSet));
            vos_mem_copy((void*)&psessionEntry->extRateSet,
                                (void*)&pChannelChangeReq->extended_rateset,
                                sizeof(tSirMacRateSet));

            limSetChannel(pMac, pChannelChangeReq->targetChannel,
                          psessionEntry->htSecondaryChannelOffset,
                          maxTxPwr,
                          psessionEntry->peSessionId);
#endif
        }

    }
    else
    {
        limLog(pMac, LOGE,FL("Invalid Request/maxTxPwr"));
    }
}

/******************************************************************************
 * limStartBssUpdateAddIEBuffer()
 *
 *FUNCTION:
 * This function checks the src buffer and its length and then malloc for
 * dst buffer update the same
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  **pDstData_buff  A pointer to pointer of  tANI_U8 dst buffer
 * @param  *pDstDataLen  A pointer to pointer of  tANI_U16 dst buffer length
 * @param  *pSrcData_buff  A pointer of  tANI_U8  src buffer
 * @param  srcDataLen  src buffer length
******************************************************************************/

static void
limStartBssUpdateAddIEBuffer(tpAniSirGlobal pMac,
                             tANI_U8 **pDstData_buff,
                             tANI_U16 *pDstDataLen,
                             tANI_U8 *pSrcData_buff,
                             tANI_U16 srcDataLen)
{

    if (srcDataLen > 0 && pSrcData_buff != NULL)
    {
        *pDstDataLen = srcDataLen;

        *pDstData_buff = vos_mem_malloc(*pDstDataLen);

       if (NULL == *pDstData_buff)
       {
            PELOGE(limLog(pMac, LOGE, FL("AllocateMemory failed for "
                "pDstData_buff"));)
            return;
       }
       vos_mem_copy(*pDstData_buff, pSrcData_buff, *pDstDataLen);
    }
    else
    {
         *pDstData_buff = NULL;
         *pDstDataLen = 0;
    }
}
/******************************************************************************
 * limUpdateAddIEBuffer()
 *
 *FUNCTION:
 * This function checks the src buffer and length if src buffer length more
 * than dst buffer length then free the dst buffer and malloc for the new src
 * length, and update the dst buffer and length. But if dst buffer is bigger
 * than src buffer length then it just update the dst buffer and length
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  **pDstData_buff  A pointer to pointer of  tANI_U8 dst buffer
 * @param  *pDstDataLen  A pointer to pointer of  tANI_U16 dst buffer length
 * @param  *pSrcData_buff  A pointer of  tANI_U8  src buffer
 * @param  srcDataLen  src buffer length
******************************************************************************/

static void
limUpdateAddIEBuffer(tpAniSirGlobal pMac,
                             tANI_U8 **pDstData_buff,
                             tANI_U16 *pDstDataLen,
                             tANI_U8 *pSrcData_buff,
                             tANI_U16 srcDataLen)
{

    if (NULL == pSrcData_buff)
    {
        limLog(pMac, LOGE, FL("src buffer is null."));
        return;
    }

    if (srcDataLen > *pDstDataLen)
    {
        *pDstDataLen = srcDataLen;
        /* free old buffer */
        vos_mem_free(*pDstData_buff);
        /* allocate a new */
        *pDstData_buff = vos_mem_malloc(*pDstDataLen);

        if (NULL == *pDstData_buff)
        {
            limLog(pMac, LOGE, FL("Memory allocation failed."));
            *pDstDataLen = 0;
            return;
        }
    }

    /* copy the content of buffer into dst buffer
    */
    *pDstDataLen = srcDataLen;
    vos_mem_copy(*pDstData_buff, pSrcData_buff, *pDstDataLen);

}

/**
 * limUpdateIBssPropAddIEs() - update IBSS prop IE
 * @pMac          : Pointer to Global MAC structure
 * @pDstData_buff : A pointer to pointer of  tANI_U8 dst buffer
 * @pDstDataLen  :  A pointer to pointer of  tANI_U16 dst buffer length
 * @pModifyIE    :  A pointer to tSirModifyIE
 *
 * This function replaces previous ibss prop_ie with new ibss prop_ie.
 *
 * Return:
 *  True or false depending upon whether IE is updated or not
 */
static tANI_BOOLEAN
limUpdateIBssPropAddIEs(tpAniSirGlobal pMac, tANI_U8 **pDstData_buff,
                        tANI_U16 *pDstDataLen, tSirModifyIE *pModifyIE)
{
    int32_t  oui_length;
    uint8_t  *ibss_ie = NULL;

    ibss_ie = pModifyIE->pIEBuffer;
    oui_length = pModifyIE->oui_length;

    if ((0 == oui_length) || (NULL == ibss_ie)) {
        VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_ERROR,
                  FL("Invalid set IBSS vendor IE comamnd length %d ibss_ie %p"),
                  oui_length, ibss_ie);
        return FALSE;
    }

    limUpdateAddIEBuffer(pMac,
                pDstData_buff,
                pDstDataLen,
                pModifyIE->pIEBuffer,
                pModifyIE->ieBufferlength);

    return TRUE;
}

/******************************************************************************
 * limProcessModifyAddIEs()
 *
 *FUNCTION:
 * This function is called by limProcessMessageQueue(). This
 * function update the PE buffers for additional IEs.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
******************************************************************************/
static void
limProcessModifyAddIEs(tpAniSirGlobal pMac, tANI_U32 *pMsg)
{
    tpSirModifyIEsInd pModifyAddIEs = (tpSirModifyIEsInd)pMsg;
    tANI_U8           sessionId;
    tANI_BOOLEAN      ret = FALSE;

    /* Incoming message has smeSession, use BSSID to find PE session*/
    tpPESession  psessionEntry = peFindSessionByBssid(pMac,
                                           pModifyAddIEs->modifyIE.bssid,
                                           &sessionId);

    if (NULL != psessionEntry)
    {
        if ((0 != pModifyAddIEs->modifyIE.ieBufferlength) &&
            (0 != pModifyAddIEs->modifyIE.ieIDLen) &&
            (NULL != pModifyAddIEs->modifyIE.pIEBuffer))
        {

            switch (pModifyAddIEs->updateType)
            {
            case eUPDATE_IE_PROBE_RESP:
            {
                /* Probe resp */
                if (LIM_IS_IBSS_ROLE(psessionEntry)) {
                    limUpdateIBssPropAddIEs(pMac,
                        &psessionEntry->addIeParams.probeRespData_buff,
                        &psessionEntry->addIeParams.probeRespDataLen,
                        &pModifyAddIEs->modifyIE);
                }
                break;
            }
            case eUPDATE_IE_ASSOC_RESP:
                /* assoc resp IE */
                if (psessionEntry->addIeParams.assocRespDataLen == 0)
                {
                    VOS_TRACE(VOS_MODULE_ID_PE, VOS_TRACE_LEVEL_ERROR,
                              FL("assoc resp add ie not present %d"),
                              psessionEntry->addIeParams.assocRespDataLen);
                }
                /* search through the buffer and modify the IE */
                break;
            case eUPDATE_IE_PROBE_BCN:
            {
                /*probe beacon IE*/
                if (LIM_IS_IBSS_ROLE(psessionEntry)) {
                    ret = limUpdateIBssPropAddIEs(pMac,
                        &psessionEntry->addIeParams.probeRespBCNData_buff,
                        &psessionEntry->addIeParams.probeRespBCNDataLen,
                        &pModifyAddIEs->modifyIE);
                }
                if (ret == TRUE && pModifyAddIEs->modifyIE.notify)
                {
                    limHandleParamUpdate(pMac, pModifyAddIEs->updateType);
                }
                break;
            }
            default:
                limLog(pMac, LOGE, FL("unhandled buffer type %d."),
                    pModifyAddIEs->updateType);
                break;
            }
        }
        else
        {
            limLog(pMac, LOGE, FL("Invalid request pIEBuffer %p ieBufferlength"
                            " %d ieIDLen %d ieID %d. update Type %d"),
                            pModifyAddIEs->modifyIE.pIEBuffer,
                            pModifyAddIEs->modifyIE.ieBufferlength,
                            pModifyAddIEs->modifyIE.ieID,
                            pModifyAddIEs->modifyIE.ieIDLen,
                            pModifyAddIEs->updateType);
        }
    }
    else
    {
        limLog(pMac, LOGE, FL("Session not found for given bssid. "
            MAC_ADDRESS_STR), MAC_ADDR_ARRAY(pModifyAddIEs->modifyIE.bssid));
    }
    vos_mem_free(pModifyAddIEs->modifyIE.pIEBuffer);
    pModifyAddIEs->modifyIE.pIEBuffer = NULL;

}

/**
 * lim_process_set_pdev_IEs() - process the set pdev IE req
 *
 * @mac_ctx: Pointer to Global MAC structure
 * @msg_buf: Pointer to the SME message buffer
 *
 * This function is called by limProcessMessageQueue(). This
 * function sets the PDEV IEs to the FW.
 *
 * Return: None
 */
static void lim_process_set_pdev_IEs(tpAniSirGlobal mac_ctx, tANI_U32 *msg_buf)
{
	struct sir_set_ht_vht_cfg *ht_vht_cfg;

	ht_vht_cfg = (struct sir_set_ht_vht_cfg*)msg_buf;

	if (NULL == ht_vht_cfg) {
		limLog(mac_ctx, LOGE, FL("NULL ht_vht_cfg"));
		return;
	}

	limLog(mac_ctx, LOG1, FL("rcvd set pdev ht vht ie req with nss = %d"),
				ht_vht_cfg->nss);
	lim_set_pdev_ht_ie(mac_ctx, ht_vht_cfg->pdev_id, ht_vht_cfg->nss);

	if (IS_DOT11_MODE_VHT(ht_vht_cfg->dot11mode))
		lim_set_pdev_vht_ie(mac_ctx, ht_vht_cfg->pdev_id,
				ht_vht_cfg->nss);
}

/**
 * lim_set_pdev_ht_ie() - sends the set HT IE req to FW
 *
 * @mac_ctx: Pointer to Global MAC structure
 * @pdev_id: pdev id to set the IE.
 * @nss: Nss values to prepare the HT IE.
 *
 * Prepares the HT IE with self capabilities for different
 * Nss values and sends the set HT IE req to FW.
 *
 * Return: None
 */
static void lim_set_pdev_ht_ie(tpAniSirGlobal mac_ctx, tANI_U8 pdev_id,
		tANI_U8 nss)
{
	struct set_ie_param *ie_params;
	tSirMsgQ msg;
	tSirRetStatus rc = eSIR_SUCCESS;
	v_U8_t *p_ie = NULL;
	tHtCaps *p_ht_cap;
	int i;

	for (i = nss; i > 0; i--) {
		ie_params = vos_mem_malloc(sizeof(*ie_params));
		if (NULL == ie_params) {
			limLog(mac_ctx, LOGE, FL("mem alloc failed"));
			return;
		}
		ie_params->nss = i;
		ie_params->pdev_id = pdev_id;
		ie_params->ie_type = DOT11_HT_IE;
		/* 2 for IE len and EID */
		ie_params->ie_len = 2 + sizeof(tHtCaps);
		ie_params->ie_ptr = vos_mem_malloc(ie_params->ie_len);
		if (NULL == ie_params->ie_ptr) {
			vos_mem_free(ie_params);
			limLog(mac_ctx, LOGE, FL("mem alloc failed"));
			return;
		}
		*ie_params->ie_ptr = SIR_MAC_HT_CAPABILITIES_EID;
		*(ie_params->ie_ptr + 1) = ie_params->ie_len - 2;
		lim_set_ht_caps(mac_ctx, NULL, ie_params->ie_ptr,
				ie_params->ie_len);

		if (1 == i) {
			p_ie = limGetIEPtr(mac_ctx, ie_params->ie_ptr,
					ie_params->ie_len,
					DOT11F_EID_HTCAPS, ONE_BYTE);
			if (NULL == p_ie) {
				vos_mem_free(ie_params->ie_ptr);
				vos_mem_free(ie_params);
				limLog(mac_ctx, LOGE, FL(
						"failed to get IE ptr"));
				return;
			}
			p_ht_cap = (tHtCaps *)&p_ie[2];
			p_ht_cap->supportedMCSSet[1] = 0;
			p_ht_cap->txSTBC = 0;
		}

		msg.type = WDA_SET_PDEV_IE_REQ;
		msg.bodyptr = ie_params;
		msg.bodyval = 0;

		rc = wdaPostCtrlMsg(mac_ctx, &msg);
		if (rc != eSIR_SUCCESS) {
			limLog(mac_ctx, LOGE, FL(
				"wdaPostCtrlMsg() return failure"));
			vos_mem_free(ie_params->ie_ptr);
			vos_mem_free(ie_params);
			return;
		}
	}
}

/**
 * lim_set_pdev_vht_ie() - sends the set VHT IE to req FW
 *
 * @mac_ctx: Pointer to Global MAC structure
 * @pdev_id: pdev id to set the IE.
 * @nss: Nss values to prepare the VHT IE.
 *
 * Prepares the VHT IE with self capabilities for different
 * Nss values and sends the set VHT IE req to FW.
 *
 * Return: None
 */
static void lim_set_pdev_vht_ie(tpAniSirGlobal mac_ctx, tANI_U8 pdev_id,
		tANI_U8 nss)
{
	struct set_ie_param *ie_params;
	tSirMsgQ msg;
	tSirRetStatus rc = eSIR_SUCCESS;
	v_U8_t *p_ie = NULL;
	tSirMacVHTCapabilityInfo *vht_cap;
	int i;
	tSirVhtMcsInfo *vht_mcs;

	for (i = nss; i > 0; i--) {
		ie_params = vos_mem_malloc(sizeof(*ie_params));
		if (NULL == ie_params) {
			limLog(mac_ctx, LOGE, FL("mem alloc failed"));
			return;
		}
		ie_params->nss = i;
		ie_params->pdev_id = pdev_id;
		ie_params->ie_type = DOT11_VHT_IE;
		/* 2 for IE len and EID */
		ie_params->ie_len = 2 + sizeof(tSirMacVHTCapabilityInfo) +
			sizeof(tSirVhtMcsInfo);
		ie_params->ie_ptr = vos_mem_malloc(ie_params->ie_len);
		if (NULL == ie_params->ie_ptr) {
			vos_mem_free(ie_params);
			limLog(mac_ctx, LOGE, FL("mem alloc failed"));
			return;
		}
		*ie_params->ie_ptr = SIR_MAC_VHT_CAPABILITIES_EID;
		*(ie_params->ie_ptr + 1) = ie_params->ie_len - 2;
		lim_set_vht_caps(mac_ctx, NULL, ie_params->ie_ptr,
				ie_params->ie_len);

		if (1 == i) {
			p_ie = limGetIEPtr(mac_ctx, ie_params->ie_ptr,
					ie_params->ie_len,
					DOT11F_EID_VHTCAPS, ONE_BYTE);
			if (NULL == p_ie) {
				vos_mem_free(ie_params->ie_ptr);
				vos_mem_free(ie_params);
				limLog(mac_ctx, LOGE, FL(
						"failed to get IE ptr"));
				return;
			}
			vht_cap = (tSirMacVHTCapabilityInfo *)&p_ie[2];
			vht_cap->txSTBC = 0;
			vht_mcs =
				(tSirVhtMcsInfo *)&p_ie[2 +
				sizeof(tSirMacVHTCapabilityInfo)];
			vht_mcs->rxMcsMap |= DISABLE_NSS2_MCS;
			vht_mcs->rxHighest =
				VHT_RX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
			vht_mcs->txMcsMap |= DISABLE_NSS2_MCS;
			vht_mcs->txHighest =
				VHT_TX_HIGHEST_SUPPORTED_DATA_RATE_1_1;
		}
		msg.type = WDA_SET_PDEV_IE_REQ;
		msg.bodyptr = ie_params;
		msg.bodyval = 0;

		rc = wdaPostCtrlMsg(mac_ctx, &msg);
		if (rc != eSIR_SUCCESS) {
			limLog(mac_ctx, LOGE, FL(
					"wdaPostCtrlMsg() return failure"));
			vos_mem_free(ie_params->ie_ptr);
			vos_mem_free(ie_params);
			return;
		}
	}
}
/******************************************************************************
 * limProcessUpdateAddIEs()
 *
 *FUNCTION:
 * This function is called by limProcessMessageQueue(). This
 * function update the PE buffers for additional IEs.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
******************************************************************************/
static void
limProcessUpdateAddIEs(tpAniSirGlobal pMac, tANI_U32 *pMsg)
{
    tpSirUpdateIEsInd pUpdateAddIEs = (tpSirUpdateIEsInd)pMsg;
    tANI_U8      sessionId;
    /* incoming message has smeSession, use BSSID to find PE session*/
    tpPESession  psessionEntry = peFindSessionByBssid(pMac,
                                     pUpdateAddIEs->updateIE.bssid,
                                     &sessionId);

    if (NULL != psessionEntry)
    {
        /* if len is 0, upper layer requested freeing of buffer */
        if (0 == pUpdateAddIEs->updateIE.ieBufferlength)
        {
            switch (pUpdateAddIEs->updateType)
            {
            case eUPDATE_IE_PROBE_RESP:
                vos_mem_free(psessionEntry->addIeParams.probeRespData_buff);
                psessionEntry->addIeParams.probeRespData_buff = NULL;
                psessionEntry->addIeParams.probeRespDataLen = 0;
                break;
            case eUPDATE_IE_ASSOC_RESP:
                vos_mem_free(psessionEntry->addIeParams.assocRespData_buff);
                psessionEntry->addIeParams.assocRespData_buff = NULL;
                psessionEntry->addIeParams.assocRespDataLen = 0;
                break;
            case eUPDATE_IE_PROBE_BCN:
                vos_mem_free(psessionEntry->addIeParams.probeRespBCNData_buff);
                psessionEntry->addIeParams.probeRespBCNData_buff = NULL;
                psessionEntry->addIeParams.probeRespBCNDataLen = 0;

                if (pUpdateAddIEs->updateIE.notify)
                {
                    limHandleParamUpdate(pMac, pUpdateAddIEs->updateType);
                }
                break;
            default:
               break;
            }
            return;
        }

        switch (pUpdateAddIEs->updateType)
        {
        case eUPDATE_IE_PROBE_RESP:
        {
            if (pUpdateAddIEs->updateIE.append)
            {
                /* In case of append, allocate new memory with combined length */
                tANI_U16 new_length = pUpdateAddIEs->updateIE.ieBufferlength +
                                psessionEntry->addIeParams.probeRespDataLen;
                tANI_U8 *new_ptr = vos_mem_malloc(new_length);
                if (NULL == new_ptr)
                {
                    limLog(pMac, LOGE, FL("Memory allocation failed."));
                    /* free incoming buffer in message */
                    vos_mem_free(pUpdateAddIEs->updateIE.pAdditionIEBuffer);
                    return;
                }
                /* append buffer to end of local buffers */
                vos_mem_copy(new_ptr,
                             psessionEntry->addIeParams.probeRespData_buff,
                             psessionEntry->addIeParams.probeRespDataLen);
                vos_mem_copy(&new_ptr[psessionEntry->addIeParams.probeRespDataLen],
                             pUpdateAddIEs->updateIE.pAdditionIEBuffer,
                             pUpdateAddIEs->updateIE.ieBufferlength);
                /* free old memory*/
                vos_mem_free(psessionEntry->addIeParams.probeRespData_buff);
                /* adjust length accordingly */
                psessionEntry->addIeParams.probeRespDataLen = new_length;
                /* save refernece of local buffer in PE session */
                psessionEntry->addIeParams.probeRespData_buff = new_ptr;
                /* free incoming buffer in message */
                vos_mem_free(pUpdateAddIEs->updateIE.pAdditionIEBuffer);
                return;
            }
            limUpdateAddIEBuffer(pMac,
                &psessionEntry->addIeParams.probeRespData_buff,
                &psessionEntry->addIeParams.probeRespDataLen,
                pUpdateAddIEs->updateIE.pAdditionIEBuffer,
                pUpdateAddIEs->updateIE.ieBufferlength);
            break;
        }
        case eUPDATE_IE_ASSOC_RESP:
              /*assoc resp IE*/
            limUpdateAddIEBuffer(pMac,
                &psessionEntry->addIeParams.assocRespData_buff,
                &psessionEntry->addIeParams.assocRespDataLen,
                pUpdateAddIEs->updateIE.pAdditionIEBuffer,
                pUpdateAddIEs->updateIE.ieBufferlength);
            break;
        case eUPDATE_IE_PROBE_BCN:
              /*probe resp Bcn IE*/
            limUpdateAddIEBuffer(pMac,
                &psessionEntry->addIeParams.probeRespBCNData_buff,
                &psessionEntry->addIeParams.probeRespBCNDataLen,
                pUpdateAddIEs->updateIE.pAdditionIEBuffer,
                pUpdateAddIEs->updateIE.ieBufferlength);
            if (pUpdateAddIEs->updateIE.notify)
            {
                limHandleParamUpdate(pMac, pUpdateAddIEs->updateType);
            }
            break;
        default:
            limLog(pMac, LOGE, FL("unhandled buffer type %d."),
                pUpdateAddIEs->updateType);
            break;
        }
    }
    else
    {
        limLog(pMac, LOGE, FL("Session not found for given bssid. "
            MAC_ADDRESS_STR), MAC_ADDR_ARRAY(pUpdateAddIEs->updateIE.bssid));
    }
    vos_mem_free(pUpdateAddIEs->updateIE.pAdditionIEBuffer);
    pUpdateAddIEs->updateIE.pAdditionIEBuffer = NULL;

}

/**
 * send_extended_chan_switch_action_frame()- function to send ECSA
 * action frame for each sta connected to SAP/GO and AP in case of
 * STA .
 * @mac_ctx: pointer to global mac structure
 * @new_channel: new channel to switch to.
 * @ch_bandwidth: BW of channel to calculate op_class
 * @session_entry: pe session
 *
 * This function is called to send ECSA frame for STA/CLI and SAP/GO.
 *
 * Return: void
 */

static void send_extended_chan_switch_action_frame(tpAniSirGlobal mac_ctx,
				uint16_t new_channel, uint8_t ch_bandwidth,
						tpPESession session_entry)
{
	uint16_t op_class;
	uint8_t switch_mode = 0, i;
	tpDphHashNode psta;


	op_class = regdm_get_opclass_from_channel(
				mac_ctx->scan.countryCodeCurrent,
				new_channel,
				ch_bandwidth);

	if (LIM_IS_AP_ROLE(session_entry) &&
		(mac_ctx->sap.SapDfsInfo.disable_dfs_ch_switch == VOS_FALSE))
		switch_mode = 1;

	if (LIM_IS_AP_ROLE(session_entry)) {
		for (i = 0; i < mac_ctx->lim.maxStation; i++) {
			psta =
			  session_entry->dph.dphHashTable.pDphNodeArray + i;
			if (psta && psta->added) {
				lim_send_extended_chan_switch_action_frame(
					mac_ctx,
					psta->staAddr,
					switch_mode, op_class, new_channel,
					LIM_MAX_CSA_IE_UPDATES, session_entry);
			}
		}
	} else if (LIM_IS_STA_ROLE(session_entry)) {
		lim_send_extended_chan_switch_action_frame(mac_ctx,
					session_entry->bssId,
					switch_mode, op_class, new_channel,
					LIM_MAX_CSA_IE_UPDATES, session_entry);
	}

}

/**
 * limProcessSmeDfsCsaIeRequest()
 *
 *FUNCTION:
 * This function is called by limProcessMessageQueue(). This
 * function processes SME request messages from HDD or upper layer
 * application.
 *
 *LOGIC:
 *
 *ASSUMPTIONS:
 *
 *NOTE:
 *
 * @param  pMac      Pointer to Global MAC structure
 * @param  *pMsgBuf  A pointer to the SME message buffer
 */
static void
limProcessSmeDfsCsaIeRequest(tpAniSirGlobal pMac, tANI_U32 *pMsg)
{

    tpSirDfsCsaIeRequest  pDfsCsaIeRequest = (tSirDfsCsaIeRequest *)pMsg;
    tpPESession           psessionEntry = NULL;
    tANI_U32 chanWidth = 0;
    tANI_U8               sessionId;

    if ( pMsg == NULL )
    {
        limLog(pMac, LOGE,FL("Buffer is Pointing to NULL"));
        return;
    }

    if ((psessionEntry =
         peFindSessionByBssid(pMac,
                              pDfsCsaIeRequest->bssid,
                              &sessionId)) == NULL)
    {
        limLog(pMac, LOGE,
               FL("Session not found for given BSSID" MAC_ADDRESS_STR),
               MAC_ADDR_ARRAY(pDfsCsaIeRequest->bssid));
        return;
    }

    if (psessionEntry->valid && !LIM_IS_AP_ROLE(psessionEntry)) {
        limLog(pMac, LOGE, FL("Invalid SystemRole %d"),
               GET_LIM_SYSTEM_ROLE(psessionEntry));
        return;
    }

    if ( psessionEntry )
    {
        /* target channel */
        psessionEntry->gLimChannelSwitch.primaryChannel =
                                    pDfsCsaIeRequest->targetChannel;

        /* Channel switch announcement needs to be included in beacon */
        psessionEntry->dfsIncludeChanSwIe = VOS_TRUE;
        psessionEntry->gLimChannelSwitch.switchCount = LIM_MAX_CSA_IE_UPDATES;
        if (pMac->sap.SapDfsInfo.disable_dfs_ch_switch == VOS_FALSE)
            psessionEntry->gLimChannelSwitch.switchMode = 1;
        psessionEntry->gLimChannelSwitch.secondarySubBand =
                                         pDfsCsaIeRequest->ch_bandwidth;

        /* Validate if SAP is operating HT or VHT
         * mode and set the Channel Switch Wrapper
         * element with the Wide Band Switch
         * subelement..
         */
#ifdef WLAN_FEATURE_11AC
        if (VOS_TRUE == psessionEntry->vhtCapability)
        {
            if (WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ ==
                                        psessionEntry->vhtTxChannelWidthSet)
            {
                chanWidth = eHT_CHANNEL_WIDTH_80MHZ;
            }
            else if (WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ ==
                                        psessionEntry->vhtTxChannelWidthSet)
            {
                chanWidth = psessionEntry->htSupportedChannelWidthSet;
            }

            /*
             * Now encode the Wider Channel BW element
             * depending on the chanWidth.
             */
            switch(chanWidth)
            {
                case eHT_CHANNEL_WIDTH_20MHZ:
                    /*
                     * Wide channel BW sublement in channel
                     * wrapper element is not required in case
                     * of 20 Mhz operation. Currently It is set
                     * only set in case of 40/80 Mhz Operation.
                     */
                    psessionEntry->dfsIncludeChanWrapperIe = VOS_FALSE;
                    psessionEntry->gLimWiderBWChannelSwitch.newChanWidth =
                                            WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ;
                    break;
                case eHT_CHANNEL_WIDTH_40MHZ:
                    psessionEntry->dfsIncludeChanWrapperIe = VOS_TRUE;
                    psessionEntry->gLimWiderBWChannelSwitch.newChanWidth =
                                            WNI_CFG_VHT_CHANNEL_WIDTH_20_40MHZ;
                    break;
                case eHT_CHANNEL_WIDTH_80MHZ:
                    psessionEntry->dfsIncludeChanWrapperIe = VOS_TRUE;
                    psessionEntry->gLimWiderBWChannelSwitch.newChanWidth =
                                            WNI_CFG_VHT_CHANNEL_WIDTH_80MHZ;
                    break;
                case eHT_CHANNEL_WIDTH_160MHZ:
                    psessionEntry->dfsIncludeChanWrapperIe = VOS_TRUE;
                    psessionEntry->gLimWiderBWChannelSwitch.newChanWidth =
                                            WNI_CFG_VHT_CHANNEL_WIDTH_160MHZ;
                    break;
                default:
                    psessionEntry->dfsIncludeChanWrapperIe = VOS_FALSE;
                    /* Need to handle 80+80 Mhz Scenario
                     * When 80+80 is supported set the
                     * gLimWiderBWChannelSwitch.newChanWidth
                     * to 3
                     */
                    PELOGE(limLog(pMac, LOGE, FL("Invalid Channel Width"));)
                    break;
            }
            /*
             * Fetch the center channel based on the channel width
             */
            psessionEntry->gLimWiderBWChannelSwitch.newCenterChanFreq0 =
                           limGetCenterChannel(pMac,
                             pDfsCsaIeRequest->targetChannel,
                             psessionEntry->htSecondaryChannelOffset,
                             psessionEntry->gLimWiderBWChannelSwitch.newChanWidth);
            /*
             * This is not applicable for 20/40/80 Mhz.
             * Only used when we support 80+80 Mhz
             * operation. In case of 80+80 Mhz, this
             * parameter indicates center channel
             * frequency index of 80 Mhz channel
             * of frequency segment 1.
             */
            psessionEntry->gLimWiderBWChannelSwitch.newCenterChanFreq1 = 0;
        }
#endif
        /* Send CSA IE request from here */
        if (schSetFixedBeaconFields(pMac, psessionEntry) != eSIR_SUCCESS)
        {
            PELOGE(limLog(pMac, LOGE, FL("Unable to set CSA IE in beacon"));)
            return;
        }

        /* First beacon update request is sent here, the remaining updates are
         * done when the FW responds back after sending the first beacon after
         * the template update
         */
        limSendBeaconInd(pMac, psessionEntry);
        limLog(pMac, LOG1,
                   FL(" Updated CSA IE, IE COUNT = %d"),
                       psessionEntry->gLimChannelSwitch.switchCount );
        /* Send ECSA Action frame after updating the beacon */
        send_extended_chan_switch_action_frame(pMac,
          psessionEntry->gLimChannelSwitch.primaryChannel,
            psessionEntry->gLimChannelSwitch.secondarySubBand,
                                                  psessionEntry);
        psessionEntry->gLimChannelSwitch.switchCount--;
    }
    return;
}

/**
 * lim_process_ext_change_channel()- function to send ECSA
 * action frame for STA/CLI .
 * @mac_ctx: pointer to global mac structure
 * @msg: params from sme for new channel.
 *
 * This function is called to send ECSA frame for STA/CLI.
 *
 * Return: void
 */

static void lim_process_ext_change_channel(tpAniSirGlobal mac_ctx,
							uint32_t *msg)
{
	struct sir_sme_ext_cng_chan_req *ext_chng_channel =
				(struct sir_sme_ext_cng_chan_req *) msg;
	tpPESession session_entry = NULL;

	if (NULL == msg) {
		limLog(mac_ctx, LOGE, FL("Buffer is Pointing to NULL"));
		return;
	}
	session_entry =
		pe_find_session_by_sme_session_id(mac_ctx,
						ext_chng_channel->session_id);
	if (NULL == session_entry) {
		limLog(mac_ctx, LOGE,
			FL("Session not found for given session %d"),
			ext_chng_channel->session_id);
		return;
	}
	if (LIM_IS_AP_ROLE(session_entry)) {
		limLog(mac_ctx, LOGE,
			FL("not an STA/CLI session"));
		return;
	}
	send_extended_chan_switch_action_frame(mac_ctx,
			ext_chng_channel->new_channel,
				0, session_entry);
}
