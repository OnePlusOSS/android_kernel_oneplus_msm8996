/*
 * Copyright (c) 2013 The Linux Foundation. All rights reserved.
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

/*===========================================================================

                       wlan_qct_WMA_legacy.c

  OVERVIEW:

  This software unit holds the implementation of the WLAN Device Adaptation
  Layer for the legacy functionalities that were part of the old HAL.

  The functions externalized by this module are to be called ONLY by other
  WLAN modules that properly register with the Transport Layer initially.

  DEPENDENCIES:

  Are listed for each API below.
===========================================================================*/

/* Standard include files */
/* Application Specific include files */
#include "limApi.h"
#include "pmmApi.h"
#include "cfgApi.h"

/* Locally used Defines */

#define HAL_MMH_MB_MSG_TYPE_MASK    0xFF00
// -------------------------------------------------------------
/**
 * WMAPostCtrlMsg
 *
 * FUNCTION:
 *     Posts WMA messages to MC thread
 *
 * LOGIC:
 *
 * ASSUMPTIONS:pl
 *
 *
 * NOTE:
 *
 * @param tpAniSirGlobal MAC parameters structure
 * @param pMsg pointer with message
 * @return Success or Failure
 */

tSirRetStatus
wmaPostCtrlMsg(tpAniSirGlobal pMac, tSirMsgQ *pMsg)
{
   if(VOS_STATUS_SUCCESS != vos_mq_post_message(VOS_MQ_ID_WMA, (vos_msg_t *) pMsg))
      return eSIR_FAILURE;
   else
      return eSIR_SUCCESS;
} // halPostMsg()

/**
 * WMAPostCfgMsg
 *
 * FUNCTION:
 *     Posts MNT messages to gSirMntMsgQ
 *
 * LOGIC:
 *
 * ASSUMPTIONS:
 *
 *
 * NOTE:
 *
 * @param tpAniSirGlobal MAC parameters structure
 * @param pMsg A pointer to the msg
 * @return Success or Failure
 */

tSirRetStatus
wmaPostCfgMsg(tpAniSirGlobal pMac, tSirMsgQ *pMsg)
{
   tSirRetStatus rc = eSIR_SUCCESS;

   do
   {
      // For Windows based MAC, instead of posting message to different
      // queues we will call the handler routines directly

      cfgProcessMbMsg(pMac, (tSirMbMsg*)pMsg->bodyptr);
      rc = eSIR_SUCCESS;
   } while (0);

   return rc;
} // halMntPostMsg()

// -------------------------------------------------------------
/**
 * uMacPostCtrlMsg
 *
 * FUNCTION:
 *     Forwards the completely received message to the respective
 *    modules for further processing.
 *
 * LOGIC:
 *
 * ASSUMPTIONS:
 *    Freeing up of the message buffer is left to the destination module.
 *
 * NOTE:
 *  This function has been moved to the API file because for MAC running
 *  on Windows host, the host module will call this routine directly to
 *  send any mailbox messages. Making this function an API makes sure that
 *  outside world (any module outside MMH) only calls APIs to use MMH
 *  services and not an internal function.
 *
 * @param pMb A pointer to the maibox message
 * @return NONE
 */

tSirRetStatus uMacPostCtrlMsg(void* pSirGlobal, tSirMbMsg* pMb)
{
   tSirMsgQ msg;
   tpAniSirGlobal pMac = (tpAniSirGlobal)pSirGlobal;


   tSirMbMsg* pMbLocal;
   msg.type = pMb->type;
   msg.bodyval = 0;
//TODO:FIXME
//   WMA_LOGD("msgType %d, msgLen %d\n" ,pMb->type, pMb->msgLen);

   // copy the message from host buffer to firmware buffer
   // this will make sure that firmware allocates, uses and frees
   // it's own buffers for mailbox message instead of working on
   // host buffer

   // second parameter, 'wait option', to palAllocateMemory is ignored on Windows
   if( eHAL_STATUS_SUCCESS != palAllocateMemory( pMac->hHdd, (void **)&pMbLocal, pMb->msgLen))
   {
//TODO:FIXME
//      WMA_LOGE("Buffer Allocation failed!\n");
      return eSIR_FAILURE;
   }

   palCopyMemory(pMac, (void *)pMbLocal, (void *)pMb, pMb->msgLen);
   msg.bodyptr = pMbLocal;

   switch (msg.type & HAL_MMH_MB_MSG_TYPE_MASK)
   {
   case WMA_MSG_TYPES_BEGIN:    // Posts a message to the HAL MsgQ
      wmaPostCtrlMsg(pMac, &msg);
      break;

   case SIR_LIM_MSG_TYPES_BEGIN:    // Posts a message to the LIM MsgQ
      limPostMsgApi(pMac, &msg);
      break;

   case SIR_CFG_MSG_TYPES_BEGIN:    // Posts a message to the CFG MsgQ
      wmaPostCfgMsg(pMac, &msg);
      break;

   case SIR_PMM_MSG_TYPES_BEGIN:    // Posts a message to the PMM MsgQ
      pmmPostMessage(pMac, &msg);
      break;

   case SIR_PTT_MSG_TYPES_BEGIN:
      break;


   default:
//TODO:FIXME
//WMA_LOGD("Unknown message type = 0x%X\n", msg.type);

      // Release the memory.
      if (palFreeMemory( pMac->hHdd, (void*)(msg.bodyptr))
            != eHAL_STATUS_SUCCESS)
      {
//TODO:FIXME
//        WMA_LOGE("Buffer Allocation failed!\n");
         return eSIR_FAILURE;
      }
      break;
   }

   return eSIR_SUCCESS;

} // uMacPostCtrlMsg()

//TODO: FIXME
#if 0
/* ---------------------------------------------------------
 * FUNCTION:  WMAGetGlobalSystemRole()
 *
 * Get the global HAL system role.
 * ---------------------------------------------------------
 */
tBssSystemRole WMAGetGlobalSystemRole(tpAniSirGlobal pMac)
{
   v_VOID_t * pVosContext = vos_get_global_context(VOS_MODULE_ID_WMA, NULL);
   t_wma_handle *wmaContext =
                       vos_get_context(VOS_MODULE_ID_WMA, pVosContext);
   if(NULL == wmaContext)
   {
      VOS_TRACE( VOS_MODULE_ID_WMA, VOS_TRACE_LEVEL_ERROR,
                           "%s:WMA context is NULL", __func__);
      VOS_ASSERT(0);
      return eSYSTEM_UNKNOWN_ROLE;
   }
#ifdef FEATURE_WLAN_INTEGRATED_SOC
   WMA_LOGD(" returning  %d role\n",wmaContext->wmaGlobalSystemRole);
#endif /* #ifdef FEATURE_WLAN_INTEGRATED_SOC */
   return  wmaContext->wmaGlobalSystemRole;
}
#endif
