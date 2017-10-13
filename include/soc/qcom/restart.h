/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ASM_ARCH_MSM_RESTART_H_
#define _ASM_ARCH_MSM_RESTART_H_

#define RESTART_NORMAL 0x0
#define RESTART_DLOAD  0x1

void msm_set_restart_mode(int mode);
extern int pmic_reset_irq;


#define SHARED_IMEM_BOOT_BASE	0x066BF000

#define uint32 uint32_t
#define uint64 uint64_t

/*
 * Following structure defines all the cookies that have been placed
 * in boot's shared imem space. We use it to share variable between XBL/Kernel.
 */
struct boot_shared_imem_cookie_type
{
  /* Magic number which indicates boot shared imem has been initialized
     and the content is valid.*/
  uint32 shared_imem_magic;

  /* Number to indicate what version of this structure is being used */
  uint32 shared_imem_version;

  /* Pointer that points to etb ram dump buffer, should only be set by HLOS */
  uint64 etb_buf_addr;

  /* Region where HLOS will write the l2 cache dump buffer start address */
  uint64 l2_cache_dump_buff_addr;

  /* When SBL which is A32 allocates the 64bit pointer above it will only
     consume 4 bytes.  When HLOS running in A64 mode access this it will over
     flow into the member below it.  Adding this padding will ensure 8 bytes
     are consumed so A32 and A64 have the same view of the remaining members. */
  uint32 a64_pointer_padding;

  /* Magic number for UEFI ram dump, if this cookie is set along with dload magic numbers,
     we don't enter dload mode but continue to boot. This cookie should only be set by UEFI*/
  uint32 uefi_ram_dump_magic;

  uint32 ddr_training_cookie;

  /* Abnormal reset cookie used by UEFI */
  uint32 abnormal_reset_occurred;

  /* Reset Status Register */
  uint32 reset_status_register;

  /* Cookie that will be used to sync with RPM */
  uint32 rpm_sync_cookie;

  /* Debug config used by UEFI */
  uint32 debug_config;

  /* Boot Log Location Pointer to be accessed in UEFI */
  uint64 boot_log_addr;

  /* Boot Log Size */
  uint32 boot_log_size;

  /* Please add new cookie here, do NOT modify or rearrange the existing cookies*/
  /* Kernel Log Start Address */
  uint32 kernel_log_addr;

  /* Kernel Log Size */
  uint32 kernel_log_size;

  /* Device Info Start Address */
  uint32 device_info_addr;

  /* Device Info Size */
  uint32 device_info_size;
};

#endif

