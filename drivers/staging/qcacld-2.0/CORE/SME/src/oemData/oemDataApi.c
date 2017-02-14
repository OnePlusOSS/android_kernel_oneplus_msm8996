/*
 * Copyright (c) 2012-2014, 2016 The Linux Foundation. All rights reserved.
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

#ifdef FEATURE_OEM_DATA_SUPPORT
/** ------------------------------------------------------------------------- *
    ------------------------------------------------------------------------- *


    \file oemDataApi.c

    Implementation for the OEM DATA REQ/RSP interfaces.
========================================================================== */
#include "aniGlobal.h"
#include "oemDataApi.h"
#include "palApi.h"
#include "smeInside.h"
#include "smsDebug.h"

#include "csrSupport.h"
#include "wlan_qct_tl.h"

#include "vos_diag_core_log.h"
#include "vos_diag_core_event.h"

/* ---------------------------------------------------------------------------
    \fn oemData_OemDataReqOpen
    \brief This function must be called before any API call to (OEM DATA REQ/RSP module)
    \return eHalStatus
  -------------------------------------------------------------------------------*/

eHalStatus oemData_OemDataReqOpen(tHalHandle hHal)
{
    eHalStatus status = eHAL_STATUS_SUCCESS;
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);

    do
    {
        //initialize all the variables to null
        vos_mem_set(&(pMac->oemData), sizeof(tOemDataStruct), 0);
        if(!HAL_STATUS_SUCCESS(status))
        {
            smsLog(pMac, LOGE, "oemData_OemDataReqOpen: Cannot allocate memory for the timer function");
            break;
        }
    } while(0);

    return status;
}

/* ---------------------------------------------------------------------------
    \fn oemData_OemDataReqClose
    \brief This function must be called before closing the csr module
    \return eHalStatus
  -------------------------------------------------------------------------------*/

eHalStatus oemData_OemDataReqClose(tHalHandle hHal)
{
    eHalStatus status = eHAL_STATUS_SUCCESS;
    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);

    do
    {
        if(!HAL_STATUS_SUCCESS(status))
        {
            smsLog(pMac, LOGE, "oemData_OemDataReqClose: Failed in oemData_OemDataReqClose at StopTimers");
            break;
        }

        //initialize all the variables to null
        vos_mem_set(&(pMac->oemData), sizeof(tOemDataStruct), 0);
    } while(0);

    return eHAL_STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------
    \fn oemData_ReleaseOemDataReqCommand
    \brief This function removes the oemDataCommand from the active list and
           and frees up any memory occupied by this
    \return eHalStatus
  -------------------------------------------------------------------------------*/
void oemData_ReleaseOemDataReqCommand(tpAniSirGlobal pMac, tSmeCmd *pOemDataCmd, eOemDataReqStatus oemDataReqStatus)
{

    //First take this command out of the active list
    if(csrLLRemoveEntry(&pMac->sme.smeCmdActiveList, &pOemDataCmd->Link, LL_ACCESS_LOCK))
    {
        if (pOemDataCmd->u.oemDataCmd.oemDataReq.data) {
            vos_mem_free(pOemDataCmd->u.oemDataCmd.oemDataReq.data);
            pOemDataCmd->u.oemDataCmd.oemDataReq.data = NULL;
        }
        vos_mem_zero(&(pOemDataCmd->u.oemDataCmd), sizeof(tOemDataCmd));
        /* Now put this command back on the available command list */
        smeReleaseCommand(pMac, pOemDataCmd);
    }
    else
    {
        smsLog(pMac, LOGE, "OEM_DATA: **************** oemData_ReleaseOemDataReqCommand cannot release the command");
    }
}

/* ---------------------------------------------------------------------------
    \fn oemData_OemDataReq
    \brief Request an OEM DATA RSP
    \param sessionId - Id of session to be used
    \param pOemDataReqID - pointer to an object to get back the request ID
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus oemData_OemDataReq(tHalHandle hHal,
                                tANI_U8 sessionId,
                                tOemDataReqConfig *oemDataReqConfig,
                                tANI_U32 *pOemDataReqID)
{
    eHalStatus status = eHAL_STATUS_SUCCESS;
    tpAniSirGlobal pMac = PMAC_STRUCT( hHal );
    tSmeCmd *pOemDataCmd = NULL;
    tOemDataReq *cmd_req;

    do
    {
        if( !CSR_IS_SESSION_VALID( pMac, sessionId ) )
        {
           status = eHAL_STATUS_FAILURE;
           break;
        }

        pMac->oemData.oemDataReqID = *(pOemDataReqID);

        pMac->oemData.oemDataReqActive = eANI_BOOLEAN_FALSE;

        pOemDataCmd = smeGetCommandBuffer(pMac);

        //fill up the command before posting it.
        if(pOemDataCmd)
        {
            pOemDataCmd->command = eSmeCommandOemDataReq;
            pOemDataCmd->u.oemDataCmd.oemDataReqID = pMac->oemData.oemDataReqID;

            cmd_req = &(pOemDataCmd->u.oemDataCmd.oemDataReq);
            /* set the oem data request */
            cmd_req->sessionId = sessionId;
            cmd_req->data_len =  oemDataReqConfig->data_len;
            cmd_req->data = vos_mem_malloc(cmd_req->data_len);

            if (!cmd_req->data) {
                smsLog(pMac, LOGE, FL("memory alloc failed"));
                status = eHAL_STATUS_FAILED_ALLOC;
                break;
            }

            vos_mem_copy(cmd_req->data, oemDataReqConfig->data,
                         cmd_req->data_len);
        }
        else
        {
            status = eHAL_STATUS_FAILURE;
            break;
        }

        //now queue this command in the sme command queue
        //Here since this is not interacting with the csr just push the command
        //into the sme queue. Also push this command with the normal priority
        smePushCommand(pMac, pOemDataCmd, eANI_BOOLEAN_FALSE);

    } while(0);

    if(!HAL_STATUS_SUCCESS(status) && pOemDataCmd)
    {
        oemData_ReleaseOemDataReqCommand(pMac, pOemDataCmd, eOEM_DATA_REQ_FAILURE);
        pMac->oemData.oemDataReqActive = eANI_BOOLEAN_FALSE;
    }

    return status;
}

/* ---------------------------------------------------------------------------
    \fn oemData_SendMBOemDataReq
    \brief Request an OEM DATA REQ to be passed down to PE
    \param pMac:
    \param pOemDataReq: Pointer to the oem data request
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus oemData_SendMBOemDataReq(tpAniSirGlobal pMac, tOemDataReq *pOemDataReq)
{
    eHalStatus status = eHAL_STATUS_SUCCESS;
    tSirOemDataReq* pMsg;
    tANI_U16 msgLen;
    tCsrRoamSession *pSession;

    smsLog(pMac, LOGW, "OEM_DATA: entering Function %s", __func__);

    if (!pOemDataReq) {
        smsLog(pMac, LOGE, FL("oem data req is NULL"));
        return eHAL_STATUS_INVALID_PARAMETER;
    }

    pSession = CSR_GET_SESSION(pMac, pOemDataReq->sessionId);
    pMsg = vos_mem_malloc(sizeof(*pMsg));
    if (NULL == pMsg) {
        smsLog(pMac, LOGE, "Memory Allocation failed. %s", __func__);
        return eHAL_STATUS_FAILURE;
    }

    msgLen = (uint16_t) (sizeof(*pMsg) + pOemDataReq->data_len);
    pMsg->messageType = pal_cpu_to_be16((tANI_U16)eWNI_SME_OEM_DATA_REQ);
    pMsg->messageLen = pal_cpu_to_be16(msgLen);
    vos_mem_copy(pMsg->selfMacAddr, pSession->selfMacAddr, sizeof(tSirMacAddr) );
    pMsg->data_len = pOemDataReq->data_len;
    /* Incoming buffer ptr saved, set to null to avoid free by caller */
    pMsg->data = pOemDataReq->data;
    pOemDataReq->data = NULL;
    smsLog(pMac, LOGW, "OEM_DATA: sending message to pe%s", __func__);
    status = palSendMBMessage(pMac->hHdd, pMsg);

    smsLog(pMac, LOGW, "OEM_DATA: exiting Function %s", __func__);
    return status;
}

/* ---------------------------------------------------------------------------
    \fn oemData_ProcessOemDataReqCommand
    \brief This function is called by the smeProcessCommand when the case hits
           eSmeCommandOemDataReq
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus oemData_ProcessOemDataReqCommand(tpAniSirGlobal pMac, tSmeCmd *pOemDataReqCmd)
{
    eHalStatus status = eHAL_STATUS_SUCCESS;

    //check if the system is in proper mode of operation for
    //oem data req/rsp to be functional. Currently, concurrency is not
    //supported and the driver must be operational only as
    //STA for oem data req/rsp to be functional. We return an invalid
    //mode flag if it is operational as any one of the following
    //in any of the active sessions
    //1. AP Mode
    //2. IBSS Mode
    //3. BTAMP Mode ...

    if(eHAL_STATUS_SUCCESS == oemData_IsOemDataReqAllowed(pMac))
    {
        smsLog(pMac, LOG1, "%s: OEM_DATA REQ allowed in the current mode", __func__);
        pMac->oemData.oemDataReqActive = eANI_BOOLEAN_TRUE;
        status = oemData_SendMBOemDataReq(pMac, &(pOemDataReqCmd->u.oemDataCmd.oemDataReq));
    }
    else
    {
        smsLog(pMac, LOG1, "%s: OEM_DATA REQ not allowed in the current mode", __func__);
        status = eHAL_STATUS_FAILURE;
    }

    if (!HAL_STATUS_SUCCESS(status)) {
        smsLog(pMac, LOG1, FL("OEM_DATA Failure, Release command"));
        oemData_ReleaseOemDataReqCommand(pMac, pOemDataReqCmd, eOEM_DATA_REQ_INVALID_MODE);
        pMac->oemData.oemDataReqActive = eANI_BOOLEAN_FALSE;
    }

    return status;
}

/* ---------------------------------------------------------------------------
    \fn sme_HandleOemDataRsp
    \brief This function processes the oem data response obtained from the PE
    \param pMsg - Pointer to the pSirOemDataRsp
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus sme_HandleOemDataRsp(tHalHandle hHal, tANI_U8* pMsg)
{
    eHalStatus                         status = eHAL_STATUS_SUCCESS;
    tpAniSirGlobal                     pMac;
    tListElem                          *pEntry = NULL;
    tSmeCmd                            *pCommand = NULL;
    tSirOemDataRsp*                    pOemDataRsp = NULL;
    tOemDataReq                        *req;
    tANI_U32                           *msgSubType;

    pMac = PMAC_STRUCT(hHal);

    smsLog(pMac, LOG1, "%s: OEM_DATA Entering", __func__);

    do
    {
        if(pMsg == NULL)
        {
            smsLog(pMac, LOGE, "in %s msg ptr is NULL", __func__);
            status = eHAL_STATUS_FAILURE;
            break;
        }

        /* In this case, there can be multiple OEM Data Responses for one
         * OEM Data request, SME does not peek into data response so SME
         * can not know which response is the last one. So SME clears active
         * request command on receiving first response and thereafter SME
         * passes each subsequent response to upper user layer.
         */
        pEntry = csrLLPeekHead(&pMac->sme.smeCmdActiveList, LL_ACCESS_LOCK);
        if (pEntry)
        {
            pCommand = GET_BASE_ADDR(pEntry, tSmeCmd, Link);
            if (eSmeCommandOemDataReq == pCommand->command)
            {
                if (csrLLRemoveEntry(&pMac->sme.smeCmdActiveList,
                                     &pCommand->Link, LL_ACCESS_LOCK))
                {
                    vos_mem_set(&(pCommand->u.oemDataCmd),
                                sizeof(tOemDataCmd), 0);
                    req = &(pCommand->u.oemDataCmd.oemDataReq);
                    vos_mem_free(req->data);
                    smeReleaseCommand(pMac, pCommand);
                }
            }
        }

        pOemDataRsp = (tSirOemDataRsp *)pMsg;

        /* check if message is to be forwarded to oem application or not */
        msgSubType = (tANI_U32 *) (&pOemDataRsp->oemDataRsp[0]);
        if (*msgSubType != OEM_MESSAGE_SUBTYPE_INTERNAL)
        {
            VOS_TRACE(VOS_MODULE_ID_SME, VOS_TRACE_LEVEL_INFO,
                      "%s: calling send_oem_data_rsp_msg, msgSubType(0x%x)",
                      __func__, *msgSubType);
            send_oem_data_rsp_msg(sizeof(tOemDataRsp),
                                  &pOemDataRsp->oemDataRsp[0]);
        }
        else
            VOS_TRACE(VOS_MODULE_ID_SME, VOS_TRACE_LEVEL_INFO,
                      "%s: received internal oem data resp, msgSubType (0x%x)",
                      __func__, *msgSubType);
    } while(0);

    return status;
}

/* ---------------------------------------------------------------------------
    \fn oemData_IsOemDataReqAllowed
    \brief This function checks if OEM DATA REQs can be performed in the
           current driver state
    \return eHalStatus
  -------------------------------------------------------------------------------*/
eHalStatus oemData_IsOemDataReqAllowed(tHalHandle hHal)
{
    eHalStatus status = eHAL_STATUS_SUCCESS;
    tANI_U32 sessionId;

    tpAniSirGlobal pMac = PMAC_STRUCT(hHal);

    for(sessionId = 0; sessionId < CSR_ROAM_SESSION_MAX; sessionId++)
    {
        if(CSR_IS_SESSION_VALID(pMac, sessionId))
        {
            //co-exist with IBSS or BT-AMP mode is not supported
            if(csrIsConnStateIbss(pMac, sessionId) || csrIsBTAMP(pMac, sessionId) )
            {
                //co-exist with IBSS or BT-AMP mode is not supported
                smsLog(pMac, LOGW, "OEM DATA REQ is not allowed due to IBSS|BTAMP exist in session %d", sessionId);
                status = eHAL_STATUS_CSR_WRONG_STATE;
                break;
            }
        }
    }

    smsLog(pMac, LOG1, "Exiting oemData_IsOemDataReqAllowed with status %d", status);

    return (status);
}

#endif /*FEATURE_OEM_DATA_SUPPORT*/
