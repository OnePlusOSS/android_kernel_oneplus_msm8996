/*
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "ufshcd.h"
#include "ufs_quirks.h"

#include <linux/project_info.h>
char oem_ufs_manufacture_info[16];
char oem_ufs_fw_version[3];

static struct ufs_card_fix ufs_fixups[] = {
	/* UFS cards deviations table */
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL, UFS_DEVICE_NO_VCCQ),
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_RECOVERY_FROM_DL_NAC_ERRORS),
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL,
		UFS_DEVICE_NO_FASTAUTO),
	UFS_FIX(UFS_VENDOR_TOSHIBA, "THGLF2G9C8KBADG",
		UFS_DEVICE_QUIRK_PA_TACTIVATE),
	UFS_FIX(UFS_VENDOR_TOSHIBA, "THGLF2G9D8KBADG",
		UFS_DEVICE_QUIRK_PA_TACTIVATE),
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_HOST_PA_TACTIVATE),
	UFS_FIX(UFS_VENDOR_HYNIX, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_HOST_PA_SAVECONFIGTIME),

	END_FIX
};

static int ufs_get_device_info(struct ufs_hba *hba,
				struct ufs_card_info *card_data)
{
	int err;
	u8 model_index;
	u8 str_desc_buf[QUERY_DESC_STRING_MAX_SIZE + 1];
	u8 desc_buf[QUERY_DESC_DEVICE_MAX_SIZE];

	err = ufshcd_read_device_desc(hba, desc_buf,
					QUERY_DESC_DEVICE_MAX_SIZE);
	if (err)
		goto out;

	/*
	 * getting vendor (manufacturerID) and Bank Index in big endian
	 * format
	 */
	card_data->wmanufacturerid = desc_buf[DEVICE_DESC_PARAM_MANF_ID] << 8 |
				     desc_buf[DEVICE_DESC_PARAM_MANF_ID + 1];

	model_index = desc_buf[DEVICE_DESC_PARAM_PRDCT_NAME];

	memset(str_desc_buf, 0, QUERY_DESC_STRING_MAX_SIZE);
	err = ufshcd_read_string_desc(hba, model_index, str_desc_buf,
					QUERY_DESC_STRING_MAX_SIZE, ASCII_STD);
	if (err)
		goto out;

	str_desc_buf[QUERY_DESC_STRING_MAX_SIZE] = '\0';
	strlcpy(card_data->model, (str_desc_buf + QUERY_DESC_HDR_SIZE),
		min_t(u8, str_desc_buf[QUERY_DESC_LENGTH_OFFSET],
		      MAX_MODEL_LEN));
	/* Null terminate the model string */
	card_data->model[MAX_MODEL_LEN] = '\0';

out:
	return err;
}

static int ufs_get_capacity_info(struct ufs_hba *hba,	u64 *pcapacity)
{
	int err;
	u8 geometry_buf[QUERY_DESC_GEOMETRY_MAZ_SIZE];

	err = ufshcd_read_geometry_desc(hba, geometry_buf,
					QUERY_DESC_GEOMETRY_MAZ_SIZE);
	if (err)
		goto out;

	*pcapacity = (u64)geometry_buf[0x04] << 56 |
				 (u64)geometry_buf[0x04 + 1] << 48 |
				 (u64)geometry_buf[0x04 + 2] << 40 |
				 (u64)geometry_buf[0x04 + 3] << 32 |
				 (u64)geometry_buf[0x04 + 4] << 24 |
				 (u64)geometry_buf[0x04 + 5] << 16 |
				 (u64)geometry_buf[0x04 + 6] << 8 |
				 (u64)geometry_buf[0x04 + 7];

	printk("ufs_get_capacity_info size = 0x%llx", *pcapacity);

out:
	return err;
}

static char* ufs_get_capacity_size(u64 capacity)
{
	if (capacity == 0x1D62000){ //16G
		return "16G";
	} else if (capacity == 0x3B9E000){ //32G
		return "32G";
	} else if (capacity == 0x7734000){ //64G
		return "64G";
	} else if (capacity == 0xEE60000){ //128G
		return "128G";
	} else {
		return "0G";
	}
}

char ufs_vendor_and_rev[32]={'\0'};
int ufs_fill_info(struct ufs_hba *hba)
{
	int err=0;
	u64 ufs_capacity = 0;
	char ufs_vendor[9]={'\0'};
	char ufs_rev[5]={'\0'};

	/* Error Handle: Before filling ufs info, we must confirm sdev_ufs_device structure is not NULL*/
	if(!hba->sdev_ufs_device) {
		dev_err(hba->dev, "%s:hba->sdev_ufs_device is NULL!\n", __func__);
		goto out;
	}

	/* Copy UFS info from host controller structure (ex:vendor name, firmware revision) */
	if(!hba->sdev_ufs_device->vendor) {
		dev_err(hba->dev, "%s: UFS vendor info is NULL\n", __func__);
		strncpy(ufs_vendor, "UNKNOWN", 7);
	} else {
		strncpy(ufs_vendor, hba->sdev_ufs_device->vendor, sizeof(ufs_vendor)-1);
	}

	if(!hba->sdev_ufs_device->rev) {
		dev_err(hba->dev, "%s: UFS firmware info is NULL\n", __func__);
		strncpy(ufs_rev, "UNKNOWN", 7);
	} else {
		strncpy(ufs_rev, hba->sdev_ufs_device->rev, sizeof(ufs_rev)-1);
	}

	/* Get UFS storage size*/
	err = ufs_get_capacity_info(hba, &ufs_capacity);
	if (err) {
		dev_err(hba->dev, "%s: Failed getting capacity info\n", __func__);
		goto out;
	}

	strcpy(oem_ufs_manufacture_info, ufs_vendor);
	strcpy(oem_ufs_fw_version, ufs_rev);

	/* Combine vendor name with firmware revision */
	strcat(ufs_vendor_and_rev, ufs_vendor);
	strcat(ufs_vendor_and_rev, " FW_");
	strcat(ufs_vendor_and_rev, ufs_rev);

	ufs_get_capacity_size(ufs_capacity);

	push_component_info(UFS, ufs_get_capacity_size(ufs_capacity), ufs_vendor_and_rev);
out:
	return err;

}

void ufs_advertise_fixup_device(struct ufs_hba *hba)
{
	int err;
	struct ufs_card_fix *f;
	struct ufs_card_info card_data;

	card_data.wmanufacturerid = 0;
	card_data.model = kmalloc(MAX_MODEL_LEN + 1, GFP_KERNEL);
	if (!card_data.model)
		goto out;

	/* get device data*/
	err = ufs_get_device_info(hba, &card_data);
	if (err) {
		dev_err(hba->dev, "%s: Failed getting device info\n", __func__);
		goto out;
	}

	for (f = ufs_fixups; f->quirk; f++) {
		/* if same wmanufacturerid */
		if (((f->card.wmanufacturerid == card_data.wmanufacturerid) ||
		     (f->card.wmanufacturerid == UFS_ANY_VENDOR)) &&
		    /* and same model */
		    (STR_PRFX_EQUAL(f->card.model, card_data.model) ||
		     !strcmp(f->card.model, UFS_ANY_MODEL)))
			/* update quirks */
			hba->dev_quirks |= f->quirk;
	}
out:
	kfree(card_data.model);
}
