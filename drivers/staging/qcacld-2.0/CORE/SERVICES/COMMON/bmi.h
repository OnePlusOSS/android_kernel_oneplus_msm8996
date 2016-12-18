/*
 * Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
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

//==============================================================================
// BMI declarations and prototypes
//
// Author(s): ="Atheros"
//==============================================================================
#ifndef _BMI_H_
#define _BMI_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Header files */
#include "athdefs.h"
#include "a_types.h"
#include "hif.h"
#include "osapi_linux.h"
#include "bmi_msg.h"
#include "ol_if_athvar.h"

A_STATUS bmi_download_firmware(struct ol_softc *scn);
void bmi_target_ready(struct ol_softc *scn, void *cfg_ctx);
void
BMICleanup(struct ol_softc *scn);

A_STATUS
bmi_done(struct ol_softc *scn);

A_STATUS
BMIReadMemory(HIF_DEVICE *device,
              A_UINT32 address,
              A_UCHAR *buffer,
              A_UINT32 length,
              struct ol_softc *scn);

A_STATUS
BMIWriteMemory(HIF_DEVICE *device,
               A_UINT32 address,
               A_UCHAR *buffer,
               A_UINT32 length,
               struct ol_softc *scn);

A_STATUS
BMIExecute(HIF_DEVICE *device,
           A_UINT32 address,
           A_UINT32 *param,
           struct ol_softc *scn);

A_STATUS
BMISetAppStart(HIF_DEVICE *device,
               A_UINT32 address,
               struct ol_softc *scn);

A_STATUS
BMIReadSOCRegister(HIF_DEVICE *device,
                   A_UINT32 address,
                   A_UINT32 *param,
                   struct ol_softc *scn);

A_STATUS
BMIWriteSOCRegister(HIF_DEVICE *device,
                    A_UINT32 address,
                    A_UINT32 param,
                    struct ol_softc *scn);

A_STATUS
BMIrompatchInstall(HIF_DEVICE *device,
                   A_UINT32 ROM_addr,
                   A_UINT32 RAM_addr,
                   A_UINT32 nbytes,
                   A_UINT32 do_activate,
                   A_UINT32 *patch_id,
                   struct ol_softc *scn);

A_STATUS
BMIrompatchUninstall(HIF_DEVICE *device,
                     A_UINT32 rompatch_id,
                     struct ol_softc *scn);

A_STATUS
BMIrompatchActivate(HIF_DEVICE *device,
                    A_UINT32 rompatch_count,
                    A_UINT32 *rompatch_list,
                    struct ol_softc *scn);

A_STATUS
BMIrompatchDeactivate(HIF_DEVICE *device,
                      A_UINT32 rompatch_count,
                      A_UINT32 *rompatch_list,
                      struct ol_softc *scn);

A_STATUS
BMISignStreamStart(HIF_DEVICE *device,
                   A_UINT32 address,
                   A_UCHAR *buffer,
                   A_UINT32 length,
                   struct ol_softc *scn);

A_STATUS
BMILZStreamStart(HIF_DEVICE *device,
                 A_UINT32 address,
                 struct ol_softc *scn);

A_STATUS
BMILZData(HIF_DEVICE *device,
          A_UCHAR *buffer,
          A_UINT32 length,
          struct ol_softc *scn);

A_STATUS
BMIFastDownload(HIF_DEVICE *device,
                A_UINT32 address,
                A_UCHAR *buffer,
                A_UINT32 length,
                struct ol_softc *scn);

A_STATUS
BMInvramProcess(HIF_DEVICE *device,
                A_UCHAR *seg_name,
                A_UINT32 *retval,
                struct ol_softc *scn);

A_STATUS
BMIRawWrite(HIF_DEVICE *device,
            A_UCHAR *buffer,
            A_UINT32 length);

A_STATUS
BMIRawRead(HIF_DEVICE *device,
           A_UCHAR *buffer,
           A_UINT32 length,
           A_BOOL want_timeout);

#ifdef __cplusplus
}
#endif

#endif /* _BMI_H_ */
