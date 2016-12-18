/*
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
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

#if !defined( __I_VOS_LOCK_H )
#define __I_VOS_LOCK_H

/**=========================================================================

  \file  i_vos_lock.h

  \brief Linux-specific definitions for vOSS Locks

  ========================================================================*/

/* $Header$ */

/*--------------------------------------------------------------------------
  Include Files
  ------------------------------------------------------------------------*/
#include <vos_types.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/device.h>
#if defined(WLAN_OPEN_SOURCE) && defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif

/*--------------------------------------------------------------------------
  Preprocessor definitions and constants
  ------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/*--------------------------------------------------------------------------
  Type declarations
  ------------------------------------------------------------------------*/
typedef struct vos_lock_s
{
   struct mutex m_lock;
   v_U32_t cookie;
   int processID;
   v_U32_t state;
   v_U8_t  refcount;
} vos_lock_t;

typedef spinlock_t vos_spin_lock_t;

typedef struct wlan_wake_lock {
#if defined CONFIG_CNSS
	struct wakeup_source lock;
#elif defined(WLAN_OPEN_SOURCE) && defined(CONFIG_HAS_WAKELOCK)
	struct wake_lock lock;
#else
	int lock;
#endif
	bool is_initialized;
} vos_wake_lock_t;

typedef void *runtime_pm_context_t;
/*-------------------------------------------------------------------------
  Function declarations and documenation
  ------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __I_VOS_LOCK_H
