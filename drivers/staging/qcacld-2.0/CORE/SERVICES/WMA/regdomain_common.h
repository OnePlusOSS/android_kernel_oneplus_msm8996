/*
 * Copyright (c) 2011, 2014-2015 The Linux Foundation. All rights reserved.
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
* Copyright (c) 2002-2009 Sam Leffler, Errno Consulting
* Copyright (c) 2005-2011 Atheros Communications, Inc.
* All rights reserved.
*
* Permission to use, copy, modify, and/or distribute this software for any
* purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
* ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
* OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*
* $FreeBSD: release/9.0.0/sys/dev/ath/ath_hal/ah_regdomain/ah_rd_regenum.h 224226 2011-07-20 12:46:58Z adrian $
*/
 /*
 * This module contains the common regulatory domain database tables:
 *
 *	- reg domain enum constants
 *	- reg domain enum to reg domain pair mappings
 *	- country to regdomain mappings
 *	- channel tag enums and the frequency-to-frequency band mappings
 *	  for all the modes
 *
 * "The country table and respective Regulatory Domain channel and power
 * settings are based on available knowledge as of software release. The
 * underlying global regulatory and spectrum rules change on a regular basis,
 * therefore, no warranty is given that the channel and power information
 * herein is complete, accurate or up to date.  Developers are responsible
 * for regulatory compliance of end-products developed using the enclosed
 * data per all applicable national requirements.  Furthermore, data in this
 * table does not guarantee that spectrum is available and that regulatory
 * approval is possible in every case. Knowldegable regulatory compliance
 * or government contacts should be consulted by the manufacturer to ensure
 * that the most current and accurate settings are used in each end-product.
 * This table was designed so that developers are able to update the country
 * table mappings as well as the Regulatory Domain definitions in order to
 * incorporate the most current channel and power settings in the end-product."
 *
 */

/* Enumerated Regulatory Domain Information 8 bit values indicate that
 * the regdomain is really a pair of unitary regdomains.  12 bit values
 * are the real unitary regdomains and are the only ones which have the
 * frequency bitmasks and flags set.
 */

#include "_ieee80211_common.h"
#include <a_types.h>
#include "wlan_defs.h"

#define MAX_CHANNELS_PER_OPERATING_CLASS  25

enum EnumRd {
    /*
     * The following regulatory domain definitions are
     * found in the EEPROM. Each regulatory domain
     * can operate in either a 5GHz or 2.4GHz wireless mode or
     * both 5GHz and 2.4GHz wireless modes.
     * In general, the value holds no special
     * meaning and is used to decode into either specific
     * 2.4GHz or 5GHz wireless mode for that particular
     * regulatory domain.
     */
    NO_ENUMRD   = 0x00,
    NULL1_WORLD = 0x03,     /* For 11b-only countries (no 11a allowed) */
    NULL1_ETSIB = 0x07,     /* Israel */
    NULL1_ETSIC = 0x08,
    FCC1_FCCA   = 0x10,     /* USA */
    FCC1_WORLD  = 0x11,     /* Hong Kong */
    FCC4_FCCA   = 0x12,     /* USA - Public Safety */
    FCC5_FCCA   = 0x13,     /* US with no DFS (UNII-1 + UNII-3 Only)*/
    FCC6_FCCA   = 0x14,     /* Canada for AP only*/

    FCC2_FCCA   = 0x20,     /* Canada */
    FCC2_WORLD  = 0x21,     /* Australia & HK */
    FCC2_ETSIC  = 0x22,
    FCC6_WORLD  = 0x23,     /* Australia for AP only*/
    FRANCE_RES  = 0x31,     /* Legacy France for OEM */
    FCC3_FCCA   = 0x3A,     /* USA & Canada w/5470 band, 11h, DFS enabled */
    FCC3_WORLD  = 0x3B,     /* USA & Canada w/5470 band, 11h, DFS enabled */
    FCC3_ETSIC  = 0x3F,     /* New Zealand, DFS enabled */

    ETSI1_WORLD = 0x37,
    ETSI3_ETSIA = 0x32,     /* France (optional) */
    ETSI2_WORLD = 0x35,     /* Hungary & others */
    ETSI3_WORLD = 0x36,     /* France & others */
    ETSI4_WORLD = 0x30,
    ETSI4_ETSIC = 0x38,
    ETSI5_WORLD = 0x39,
    ETSI6_WORLD = 0x34,     /* Bulgaria */
    ETSI8_WORLD = 0x3D,     /* Russia */
    ETSI9_WORLD = 0x3E,     /* Ukraine */
    ETSI_RESERVED   = 0x33,     /* Reserved (Do not used) */

    MKK1_MKKA   = 0x40,     /* Japan (JP1) */
    MKK1_MKKB   = 0x41,     /* Japan (JP0) */
    APL4_WORLD  = 0x42,     /* Singapore and Morocco */
    MKK2_MKKA   = 0x43,     /* Japan with 4.9G channels */
    APL_RESERVED    = 0x44,     /* Reserved (Do not used)  */
    APL2_WORLD  = 0x45,     /* Korea */
    APL2_APLC   = 0x46,
    APL3_WORLD  = 0x47,
    MKK1_FCCA   = 0x48,     /* Japan (JP1-1) */
    APL2_APLD   = 0x49,     /* Korea with 2.3G channels */
    MKK1_MKKA1  = 0x4A,     /* Japan (JE1) */
    MKK1_MKKA2  = 0x4B,     /* Japan (JE2) */
    MKK1_MKKC   = 0x4C,     /* Japan (MKK1_MKKA,except Ch14) */
    APL2_FCCA   = 0x4D,     /* Mobile customer */
    APL11_FCCA  = 0x4F,         /* Specific AP Customer 5GHz, For APs Only */

    APL3_FCCA   = 0x50,
    APL12_WORLD = 0x51,
    APL1_WORLD  = 0x52,     /* Latin America */
    APL1_FCCA   = 0x53,
    APL1_APLA   = 0x54,
    APL1_ETSIC  = 0x55,
    APL2_ETSIC  = 0x56,     /* Venezuela */
    APL5_WORLD  = 0x58,     /* Chile */
    APL13_WORLD = 0x5A,     /* Algeria */
    APL6_WORLD  = 0x5B,     /* Singapore */
    APL7_FCCA   = 0x5C,     /* Taiwan 5.47 Band */
    APL8_WORLD  = 0x5D,     /* Malaysia 5GHz */
    APL9_MKKC   = 0x5E,     /* Korea 5GHz, Before 11/2007. Now used only by APs */
    APL10_MKKC  = 0x5F,     /* Korea 5GHz, After 11/2007. For STAs only */

    /*
     * World mode SKUs
     */
    WOR0_WORLD  = 0x60,     /* World0 (WO0 SKU) */
    WOR1_WORLD  = 0x61,     /* World1 (WO1 SKU) */
    WOR2_WORLD  = 0x62,     /* World2 (WO2 SKU) */
    WOR3_WORLD  = 0x63,     /* World3 (WO3 SKU) */
    WOR4_WORLD  = 0x64,     /* World4 (WO4 SKU) */
    WOR5_ETSIC  = 0x65,     /* World5 (WO5 SKU) */

    WOR01_WORLD = 0x66,     /* World0-1 (WW0-1 SKU) */
    WOR02_WORLD = 0x67,     /* World0-2 (WW0-2 SKU) */
    EU1_WORLD   = 0x68,     /* Same as World0-2 (WW0-2 SKU), except active scan ch1-13. No ch14 */

    WOR9_WORLD  = 0x69,     /* World9 (WO9 SKU) */
    WORA_WORLD  = 0x6A,     /* WorldA (WOA SKU) */
    WORB_WORLD  = 0x6B,     /* WorldB (WOB SKU) */
    WORC_WORLD  = 0x6C,     /* WorldC (WOC SKU) */

    MKK3_MKKB   = 0x80,     /* Japan UNI-1 even + MKKB */
    MKK3_MKKA2  = 0x81,     /* Japan UNI-1 even + MKKA2 */
    MKK3_MKKC   = 0x82,     /* Japan UNI-1 even + MKKC */

    MKK4_MKKB   = 0x83,     /* Japan UNI-1 even + UNI-2 + MKKB */
    MKK4_MKKA2  = 0x84,     /* Japan UNI-1 even + UNI-2 + MKKA2 */
    MKK4_MKKC   = 0x85,     /* Japan UNI-1 even + UNI-2 + MKKC */

    MKK5_MKKB   = 0x86,     /* Japan UNI-1 even + UNI-2 + mid-band + MKKB */
    MKK5_MKKA2  = 0x87,     /* Japan UNI-1 even + UNI-2 + mid-band + MKKA2 */
    MKK5_MKKC   = 0x88,     /* Japan UNI-1 even + UNI-2 + mid-band + MKKC */
    MKK5_FCCA   = 0x9A,

    MKK6_MKKB   = 0x89,     /* Japan UNI-1 even + UNI-1 odd MKKB */
    MKK6_MKKA2  = 0x8A,     /* Japan UNI-1 even + UNI-1 odd + MKKA2 */
    MKK6_MKKC   = 0x8B,     /* Japan UNI-1 even + UNI-1 odd + MKKC */

    MKK7_MKKB   = 0x8C,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + MKKB */
    MKK7_MKKA2  = 0x8D,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + MKKA2 */
    MKK7_MKKC   = 0x8E,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + MKKC */

    MKK8_MKKB   = 0x8F,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + MKKB */
    MKK8_MKKA2  = 0x90,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + MKKA2 */
    MKK8_MKKC   = 0x91,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + MKKC */

    MKK14_MKKA1  = 0x92,     /* Japan UNI-1 even + UNI-1 odd + 4.9GHz + MKKA1 */
    MKK15_MKKA1  = 0x93,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + 4.9GHz + MKKA1 */

    MKK10_FCCA  = 0xD0,     /* Japan UNI-1 even + UNI-2 + 4.9GHz + FCCA */
    MKK10_MKKA1 = 0xD1,     /* Japan UNI-1 even + UNI-2 + 4.9GHz + MKKA1 */
    MKK10_MKKC  = 0xD2,     /* Japan UNI-1 even + UNI-2 + 4.9GHz + MKKC */
    MKK10_MKKA2 = 0xD3,     /* Japan UNI-1 even + UNI-2 + 4.9GHz + MKKA2 */

    MKK11_MKKA  = 0xD4,     /* Japan UNI-1 even + UNI-2 + mid-band + 4.9GHz + MKKA */
    MKK11_FCCA  = 0xD5,     /* Japan UNI-1 even + UNI-2 + mid-band + 4.9GHz + FCCA */
    MKK11_MKKA1 = 0xD6,     /* Japan UNI-1 even + UNI-2 + mid-band + 4.9GHz + MKKA1 */
    MKK11_MKKC  = 0xD7,     /* Japan UNI-1 even + UNI-2 + mid-band + 4.9GHz + MKKC */
    MKK11_MKKA2 = 0xD8,     /* Japan UNI-1 even + UNI-2 + mid-band + 4.9GHz + MKKA2 */

    MKK12_MKKA  = 0xD9,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + 4.9GHz + MKKA */
    MKK12_FCCA  = 0xDA,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + 4.9GHz + FCCA */
    MKK12_MKKA1 = 0xDB,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + 4.9GHz + MKKA1 */
    MKK12_MKKC  = 0xDC,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + 4.9GHz + MKKC */
    MKK12_MKKA2 = 0xDD,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + 4.9GHz + MKKA2 */

    MKK13_MKKB  = 0xDE,     /* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + MKKB + All passive + no adhoc */

    /* Following definitions are used only by s/w to map old
     * Japan SKUs.
     */
    MKK3_MKKA       = 0xF0,         /* Japan UNI-1 even + MKKA */
    MKK3_MKKA1      = 0xF1,         /* Japan UNI-1 even + MKKA1 */
    MKK3_FCCA       = 0xF2,         /* Japan UNI-1 even + FCCA */
    MKK4_MKKA       = 0xF3,         /* Japan UNI-1 even + UNI-2 + MKKA */
    MKK4_MKKA1      = 0xF4,         /* Japan UNI-1 even + UNI-2 + MKKA1 */
    MKK4_FCCA       = 0xF5,         /* Japan UNI-1 even + UNI-2 + FCCA */
    MKK9_MKKA       = 0xF6,         /* Japan UNI-1 even + 4.9GHz */
    MKK10_MKKA      = 0xF7,         /* Japan UNI-1 even + UNI-2 + 4.9GHz */
    MKK6_MKKA1      = 0xF8,         /* Japan UNI-1 even + UNI-1 odd + UNI-2 + MKKA1 */
        MKK6_FCCA       = 0xF9,         /* Japan UNI-1 even + UNI-1 odd + UNI-2 + FCCA */
    MKK7_MKKA1      = 0xFA,         /* Japan UNI-1 even + UNI-1 odd + UNI-2 + MKKA1 */
    MKK7_FCCA       = 0xFB,         /* Japan UNI-1 even + UNI-1 odd + UNI-2 + FCCA */
    MKK9_FCCA       = 0xFC,         /* Japan UNI-1 even + 4.9GHz + FCCA */
    MKK9_MKKA1      = 0xFD,         /* Japan UNI-1 even + 4.9GHz + MKKA1 */
    MKK9_MKKC       = 0xFE,         /* Japan UNI-1 even + 4.9GHz + MKKC */
    MKK9_MKKA2      = 0xFF,         /* Japan UNI-1 even + 4.9GHz + MKKA2 */

    /*
     * Regulator domains ending in a number (e.g. APL1,
     * MK1, ETSI4, etc) apply to 5GHz channel and power
     * information.  Regulator domains ending in a letter
     * (e.g. APLA, FCCA, etc) apply to 2.4GHz channel and
     * power information.
     */
    APL1        = 0x0150,   /* LAT & Asia */
    APL2        = 0x0250,   /* LAT & Asia */
    APL3        = 0x0350,   /* Taiwan */
    APL4        = 0x0450,   /* Jordan */
    APL5        = 0x0550,   /* Chile */
    APL6        = 0x0650,   /* Singapore */
    APL7        = 0x0750,   /* Taiwan, disable ch52 */
    APL8        = 0x0850,   /* Malaysia */
    APL9        = 0x0950,   /* Korea. Before 11/2007. Now used only by APs */
    APL10       = 0x1050,   /* Korea. After 11/2007. For STAs only */
    APL11       = 0x1150,   /* Specific AP Customer 5GHz, For APs Only */
    APL12       = 0x1160,   /* Kenya */
    APL13       = 0x1170,   /* Algeria */

    ETSI1       = 0x0130,   /* Europe & others */
    ETSI2       = 0x0230,   /* Europe & others */
    ETSI3       = 0x0330,   /* Europe & others */
    ETSI4       = 0x0430,   /* Europe & others */
    ETSI5       = 0x0530,   /* Europe & others */
    ETSI6       = 0x0630,   /* Europe & others */
    ETSI8       = 0x0830,   /* Russia */
    ETSI9       = 0x0930,   /* Ukraine */
    ETSIA       = 0x0A30,   /* France */
    ETSIB       = 0x0B30,   /* Israel */
    ETSIC       = 0x0C30,   /* Latin America */

    FCC1        = 0x0110,   /* US & others */
    FCC2        = 0x0120,   /* Canada, Australia & New Zealand */
    FCC3        = 0x0160,   /* US w/new middle band & DFS */
    FCC4        = 0x0165,   /* US Public Safety */
    FCC5        = 0x0510,
    FCC6        = 0x0610,   /* Canada & Australia */
    FCCA        = 0x0A10,

    APLD        = 0x0D50,   /* South Korea */

    MKK1        = 0x0140,   /* Japan (UNI-1 odd)*/
    MKK2        = 0x0240,   /* Japan (4.9 GHz + UNI-1 odd) */
    MKK3        = 0x0340,   /* Japan (UNI-1 even) */
    MKK4        = 0x0440,   /* Japan (UNI-1 even + UNI-2) */
    MKK5        = 0x0540,   /* Japan (UNI-1 even + UNI-2 + mid-band) */
    MKK6        = 0x0640,   /* Japan (UNI-1 odd + UNI-1 even) */
    MKK7        = 0x0740,   /* Japan (UNI-1 odd + UNI-1 even + UNI-2 */
    MKK8        = 0x0840,   /* Japan (UNI-1 odd + UNI-1 even + UNI-2 + mid-band) */
    MKK9        = 0x0940,   /* Japan (UNI-1 even + 4.9 GHZ) */
    MKK10       = 0x0B40,   /* Japan (UNI-1 even + UNI-2 + 4.9 GHZ) */
    MKK11       = 0x1140,   /* Japan (UNI-1 even + UNI-2 + 4.9 GHZ) */
    MKK12       = 0x1240,   /* Japan (UNI-1 even + UNI-2 + 4.9 GHZ) */
    MKK13       = 0x0C40,   /* Same as MKK8 but all passive and no adhoc 11a */
    MKK14       = 0x1440,   /* Japan UNI-1 even + UNI-1 odd + 4.9GHz */
    MKK15       = 0x1540,   /* Japan UNI-1 even + UNI-1 odd + UNI-2 + 4.9GHz */
    MKKA        = 0x0A40,   /* Japan */
    MKKC        = 0x0A50,

    NULL1       = 0x0198,
    WORLD       = 0x0199,
    DEBUG_REG_DMN   = 0x01ff,
};

enum {					/* conformance test limits */
	FCC	= 0x10,
	MKK	= 0x40,
	ETSI	= 0x30,
};
/*
 * The following are flags for different requirements per reg domain.
 * These requirements are either inhereted from the reg domain pair or
 * from the unitary reg domain if the reg domain pair flags value is
 * 0
 */

enum {
	NO_REQ			= 0x00000000,
	DISALLOW_ADHOC_11A	= 0x00000001,
	DISALLOW_ADHOC_11A_TURB	= 0x00000002,
	NEED_NFC		= 0x00000004,

	ADHOC_PER_11D		= 0x00000008,  /* Start Ad-Hoc mode */
	ADHOC_NO_11A		= 0x00000010,

	PUBLIC_SAFETY_DOMAIN	= 0x00000020,	/* public safety domain */
	LIMIT_FRAME_4MS		= 0x00000040,	/* 4msec limit on the frame length */

	NO_HOSTAP		= 0x00000080,	/* No HOSTAP mode opereation */

	REQ_MASK		= 0x000000FF,   /* Requirements bit mask */
};

static const REG_DMN_PAIR_MAPPING ahCmnRegDomainPairs[] = {
	{NO_ENUMRD,	DEBUG_REG_DMN,	DEBUG_REG_DMN, NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{NULL1_WORLD,	NULL1,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{NULL1_ETSIB,	NULL1,		ETSIB,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{NULL1_ETSIC,	NULL1,		ETSIC,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },

	{FCC2_FCCA,     FCC2,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC2_WORLD,    FCC2,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC2_ETSIC,    FCC2,		ETSIC,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC3_FCCA,     FCC3,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC3_WORLD,    FCC3,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC3_ETSIC,    FCC3,		ETSIC,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC4_FCCA,     FCC4,		FCCA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{FCC5_FCCA,     FCC5,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC6_FCCA,     FCC6,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC6_WORLD,    FCC6,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },

	{ETSI1_WORLD,	ETSI1,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{ETSI2_WORLD,	ETSI2,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{ETSI3_WORLD,	ETSI3,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{ETSI4_WORLD,	ETSI4,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{ETSI5_WORLD,	ETSI5,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{ETSI6_WORLD,	ETSI6,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
    {ETSI8_WORLD,	ETSI8,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{ETSI9_WORLD,	ETSI9,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },

	{ETSI3_ETSIA,	ETSI3,		WORLD,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{FRANCE_RES,	ETSI3,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },

	{FCC1_WORLD,	FCC1,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{FCC1_FCCA,	    FCC1,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL1_WORLD,	APL1,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL2_WORLD,	APL2,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL2_FCCA,	    APL2,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL3_WORLD,	APL3,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL4_WORLD,	APL4,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL5_WORLD,	APL5,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL13_WORLD,	APL13,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL6_WORLD,	APL6,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL7_FCCA,	    APL7,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL8_WORLD,	APL8,		WORLD,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL9_MKKC,	APL9,		MKKC,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL10_MKKC,	APL10,		MKKC,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL3_FCCA,	APL3,		FCCA,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL1_ETSIC,	APL1,		ETSIC,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{APL2_ETSIC,	APL2,		ETSIC,		NO_REQ, NO_REQ, PSCAN_DEFER, 0 },

	{MKK1_MKKA,	MKK1,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA, CTRY_JAPAN },
	{MKK1_MKKB,	MKK1,		MKKA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC| LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN1 },
	{MKK1_FCCA,	MKK1,		FCCA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC| LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1, CTRY_JAPAN2 },
	{MKK1_MKKA1,	MKK1,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC| LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN4 },
	{MKK1_MKKA2,	MKK1,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC| LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN5 },
	{MKK1_MKKC,	MKK1,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC| LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1, CTRY_JAPAN6 },

	/* MKK2 */
	{MKK2_MKKA,	MKK2,		MKKA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC| LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK2 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN3 },

	/* MKK3 */
	{MKK3_MKKA,     MKK3,           MKKA,           DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC , PSCAN_MKKA, CTRY_JAPAN25 },
	{MKK3_MKKB,	MKK3,		MKKA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN7 },
	{MKK3_MKKA1,    MKK3,           MKKA,           DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN26 },
	{MKK3_MKKA2,MKK3,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN8 },
	{MKK3_MKKC,	MKK3,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, NO_PSCAN, CTRY_JAPAN9 },
	{MKK3_FCCA,     MKK3,           FCCA,           DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, NO_PSCAN, CTRY_JAPAN27 },

	/* MKK4 */
	{MKK4_MKKA,	MKK4,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN36 },
	{MKK4_MKKB,	MKK4,		MKKA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN10 },
	{MKK4_MKKA1,    MKK4,           MKKA,           DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN28 },
	{MKK4_MKKA2,	MKK4,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 |PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN11 },
	{MKK4_MKKC,	MKK4,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN12 },
	{MKK4_FCCA,     MKK4,           FCCA,           DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN29 },

	/* MKK5 */
/*	{MKK5_MKKA,     MKK5,           MKKA,           DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN56 },*/
	{MKK5_MKKB,	MKK5,		MKKA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN13 },
	{MKK5_MKKA2,MKK5,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN14 },
	{MKK5_MKKC,	MKK5,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN15 },
	{MKK5_FCCA,     MKK5,       FCCA,       DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN56 },

	/* MKK6 */
	{MKK6_MKKB,	MKK6,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN16 },
	{MKK6_MKKA1,MKK6,		MKKA,	    DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN30 },
	{MKK6_MKKA2,	MKK6,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN17 },
	{MKK6_MKKC,	MKK6,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1, CTRY_JAPAN18 },
	{MKK6_FCCA, MKK6,       FCCA,   DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1, CTRY_JAPAN31 },

	/* MKK7 */
	{MKK7_MKKB,	MKK7,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN19 },
	{MKK7_MKKA1,MKK7,		MKKA,	    DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN32 },
	{MKK7_MKKA2, MKK7,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN20 },
	{MKK7_MKKC,	MKK7,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3, CTRY_JAPAN21 },
	{MKK7_FCCA, MKK7,       FCCA,       DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3, CTRY_JAPAN33 },

	/* MKK8 */
	{MKK8_MKKB,	MKK8,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN22 },
	{MKK8_MKKA2,MKK8,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN23 },
	{MKK8_MKKC,	MKK8,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 , CTRY_JAPAN24 },

	{MKK9_MKKA,     MKK9,           MKKA,           DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN34 },
	{MKK9_FCCA,     MKK9,		FCCA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, NO_PSCAN, CTRY_JAPAN37 },
	{MKK9_MKKA1,    MKK9,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN38 },
	{MKK9_MKKA2,    MKK9,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN40 },
	{MKK9_MKKC,     MKK9,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, NO_PSCAN, CTRY_JAPAN39 },

	{MKK10_MKKA,    MKK10,      MKKA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN35 },
	{MKK10_FCCA,    MKK10,		FCCA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 , CTRY_JAPAN41 },
	{MKK10_MKKA1,   MKK10,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN42 },
	{MKK10_MKKA2,   MKK10,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN44 },
	{MKK10_MKKC,    MKK10,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN43 },

	{MKK11_MKKA,    MKK11,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN45 },
	{MKK11_FCCA,    MKK11,		FCCA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN46 },
	{MKK11_MKKA1,   MKK11,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN47 },
	{MKK11_MKKA2,   MKK11,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN49 },
	{MKK11_MKKC,    MKK11,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK3, CTRY_JAPAN48 },

	{MKK12_MKKA,    MKK12,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3, CTRY_JAPAN50 },
	{MKK12_FCCA,    MKK12,		FCCA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3, CTRY_JAPAN51 },
	{MKK12_MKKA1,   MKK12,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN52 },
	{MKK12_MKKA2,   MKK12,		MKKA,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA2 | PSCAN_MKKA2_G, CTRY_JAPAN54 },
	{MKK12_MKKC,    MKK12,		MKKC,		DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3, CTRY_JAPAN53 },

	{MKK13_MKKB,    MKK13,		MKKA,		DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA | PSCAN_MKKA_G, CTRY_JAPAN57 },

	{MKK14_MKKA1,   MKK14,      MKKA,       DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN58 },
	{MKK15_MKKA1,   MKK15,      MKKA,       DISALLOW_ADHOC_11A_TURB | NEED_NFC | LIMIT_FRAME_4MS, NEED_NFC, PSCAN_MKK1 | PSCAN_MKK3 | PSCAN_MKKA1 | PSCAN_MKKA1_G, CTRY_JAPAN59 },

		/* These are super domains */
	{WOR0_WORLD,	WOR0_WORLD,	WOR0_WORLD,	NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{WOR1_WORLD,	WOR1_WORLD,	WOR1_WORLD,	DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{WOR2_WORLD,	WOR2_WORLD,	WOR2_WORLD,	DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{WOR3_WORLD,	WOR3_WORLD,	WOR3_WORLD,	NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{WOR4_WORLD,	WOR4_WORLD,	WOR4_WORLD,	DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{WOR5_ETSIC,	WOR5_ETSIC,	WOR5_ETSIC,	DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{WOR01_WORLD,	WOR01_WORLD,	WOR01_WORLD,	NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{WOR02_WORLD,	WOR02_WORLD,	WOR02_WORLD,	NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{EU1_WORLD,	EU1_WORLD,	EU1_WORLD,	NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
	{WOR9_WORLD,	WOR9_WORLD,	WOR9_WORLD,	DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{WORA_WORLD,	WORA_WORLD,	WORA_WORLD,	DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{WORB_WORLD,	WORB_WORLD,	WORB_WORLD,	DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB, NO_REQ, PSCAN_DEFER, 0 },
	{WORC_WORLD,	WORC_WORLD,	WORC_WORLD,	NO_REQ, NO_REQ, PSCAN_DEFER, 0 },
};

static const COUNTRY_CODE_TO_ENUM_RD ahCmnAllCountries[] = {
    {CTRY_DEBUG,       NO_ENUMRD,     "DB", "DEBUG",          YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_DEFAULT,     DEF_REGDMN,    "NA", "NO_COUNTRY_SET", YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_ALBANIA,     ETSI1_WORLD,   "AL", "ALBANIA",        YES,  NO, YES, YES, YES,  NO,  NO, NO, 7000 },
    {CTRY_ALGERIA,     APL13_WORLD,   "DZ", "ALGERIA",        YES,  NO, YES, YES, YES,  NO,  NO, NO, 7000 },
    {CTRY_ARGENTINA,   FCC3_WORLD,    "AR", "ARGENTINA",      YES,  NO,  NO, YES, YES, YES, YES, YES, 7000 },
    {CTRY_ARMENIA,     ETSI4_WORLD,   "AM", "ARMENIA",        YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_ARUBA,       ETSI1_WORLD,   "AW", "ARUBA",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_AUSTRALIA,   FCC3_WORLD,    "AU", "AUSTRALIA",      YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_AUSTRALIA2,  FCC6_WORLD,    "AU", "AUSTRALIA2",     YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_AUSTRIA,     ETSI1_WORLD,   "AT", "AUSTRIA",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_AZERBAIJAN,  ETSI4_WORLD,   "AZ", "AZERBAIJAN",     YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BAHAMAS,     FCC3_WORLD,    "BS", "BAHAMAS",        YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BAHRAIN,     APL6_WORLD,    "BH", "BAHRAIN",        YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_BANGLADESH,  APL1_WORLD,    "BD", "BANGLADESH",     YES,  NO, YES, YES, YES,  NO,  NO, NO, 7000 },
    {CTRY_BARBADOS,    FCC2_WORLD,    "BB", "BARBADOS",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BELARUS,     ETSI1_WORLD,   "BY", "BELARUS",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BELGIUM,     ETSI1_WORLD,   "BE", "BELGIUM",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BELGIUM2,    ETSI4_WORLD,   "BE", "BELGIUM2",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BELIZE,      APL1_ETSIC,    "BZ", "BELIZE",         YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BERMUDA,     FCC3_FCCA,     "BM", "BERMUDA",        YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BOLIVIA,     APL1_ETSIC,    "BO", "BOLIVIA",        YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BOSNIA_HERZ, ETSI1_WORLD,   "BA", "BOSNIA AND HERZEGOVINA", YES,  NO, YES, YES, YES, YES,  YES, YES, 7000 },
    {CTRY_BRAZIL,      FCC3_WORLD,    "BR", "BRAZIL",         YES,  NO,  NO, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BRUNEI_DARUSSALAM, APL6_WORLD, "BN", "BRUNEI DARUSSALAM", YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_BULGARIA,    ETSI1_WORLD,   "BG", "BULGARIA",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_CAMBODIA,    ETSI1_WORLD,   "KH", "CAMBODIA",       YES, NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_CANADA,      FCC3_FCCA,     "CA", "CANADA",         YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_CANADA2,     FCC6_FCCA,     "CA", "CANADA2",        YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_CHILE,       APL6_WORLD,    "CL", "CHILE",          YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_CHINA,       APL1_WORLD,    "CN", "CHINA",          YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_COLOMBIA,    FCC3_WORLD,    "CO", "COLOMBIA",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_COSTA_RICA,  FCC1_WORLD,    "CR", "COSTA RICA",     YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_CROATIA,     ETSI1_WORLD,   "HR", "CROATIA",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_CYPRUS,      ETSI1_WORLD,   "CY", "CYPRUS",         YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_CZECH,       ETSI1_WORLD,   "CZ", "CZECH REPUBLIC", YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_DENMARK,     ETSI1_WORLD,   "DK", "DENMARK",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_DOMINICAN_REPUBLIC, FCC1_FCCA, "DO", "DOMINICAN REPUBLIC", YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_ECUADOR,     FCC1_WORLD,    "EC", "ECUADOR",        YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_EGYPT,       ETSI3_WORLD,   "EG", "EGYPT",          YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_EL_SALVADOR, FCC1_WORLD,    "SV", "EL SALVADOR",    YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_ESTONIA,     ETSI1_WORLD,   "EE", "ESTONIA",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_FINLAND,     ETSI1_WORLD,   "FI", "FINLAND",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_FRANCE,      ETSI1_WORLD,   "FR", "FRANCE",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_GEORGIA,     ETSI4_WORLD,   "GE", "GEORGIA",        YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_GERMANY,     ETSI1_WORLD,   "DE", "GERMANY",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_GREECE,      ETSI1_WORLD,   "GR", "GREECE",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_GREENLAND,   ETSI1_WORLD,   "GL", "GREENLAND",      YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_GRENADA,     FCC3_FCCA,     "GD", "GRENADA",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_GUAM,        FCC1_FCCA,     "GU", "GUAM",           YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_GUATEMALA,   FCC1_FCCA,     "GT", "GUATEMALA",      YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_HAITI,       ETSI1_WORLD,   "HT", "HAITI",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_HONDURAS,    FCC3_WORLD,    "HN", "HONDURAS",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_HONG_KONG,   FCC3_WORLD,    "HK", "HONG KONG",      YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_HUNGARY,     ETSI1_WORLD,   "HU", "HUNGARY",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_ICELAND,     ETSI1_WORLD,   "IS", "ICELAND",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_INDIA,       APL6_WORLD,    "IN", "INDIA",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_INDONESIA,   APL2_WORLD,    "ID", "INDONESIA",      YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_IRAN,        APL1_WORLD,    "IR", "IRAN",           YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_IRELAND,     ETSI1_WORLD,   "IE", "IRELAND",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_ISRAEL,      ETSI3_WORLD,   "IL", "ISRAEL",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_ITALY,       ETSI1_WORLD,   "IT", "ITALY",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_JAMAICA,     FCC3_WORLD,    "JM", "JAMAICA",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_JAPAN,       MKK5_MKKA2,    "JP", "JAPAN",          YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_JAPAN1,      MKK1_MKKB,     "JP", "JAPAN1",         YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_JAPAN2,      MKK1_FCCA,     "JP", "JAPAN2",         YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_JAPAN3,      MKK2_MKKA,     "JP", "JAPAN3",         YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_JAPAN4,      MKK1_MKKA1,    "JP", "JAPAN4",         YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_JAPAN5,      MKK1_MKKA2,    "JP", "JAPAN5",         YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_JAPAN6,      MKK1_MKKC,     "JP", "JAPAN6",         YES,  NO,  NO, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_JAPAN7,      MKK3_MKKB,     "JP", "JAPAN7",         YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN8,      MKK3_MKKA2,    "JP", "JAPAN8",         YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN9,      MKK3_MKKC,     "JP", "JAPAN9",         YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN10,     MKK4_MKKB,     "JP", "JAPAN10",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN11,     MKK4_MKKA2,    "JP", "JAPAN11",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN12,     MKK4_MKKC,     "JP", "JAPAN12",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN13,     MKK5_MKKB,     "JP", "JAPAN13",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN14,     MKK5_MKKA2,    "JP", "JAPAN14",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN15,     MKK5_MKKC,     "JP", "JAPAN15",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN16,     MKK6_MKKB,     "JP", "JAPAN16",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN17,     MKK6_MKKA2,    "JP", "JAPAN17",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN18,     MKK6_MKKC,     "JP", "JAPAN18",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN19,     MKK7_MKKB,     "JP", "JAPAN19",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN20,     MKK7_MKKA2,    "JP", "JAPAN20",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN21,     MKK7_MKKC,     "JP", "JAPAN21",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN22,     MKK8_MKKB,     "JP", "JAPAN22",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN23,     MKK8_MKKA2,    "JP", "JAPAN23",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN24,     MKK8_MKKC,     "JP", "JAPAN24",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN25,     MKK3_MKKA,     "JP", "JAPAN25",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN26,     MKK3_MKKA1,    "JP", "JAPAN26",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN27,     MKK3_FCCA,     "JP", "JAPAN27",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN28,     MKK4_MKKA1,    "JP", "JAPAN28",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN29,     MKK4_FCCA,     "JP", "JAPAN29",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN30,     MKK6_MKKA1,    "JP", "JAPAN30",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN31,     MKK6_FCCA,     "JP", "JAPAN31",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN32,     MKK7_MKKA1,    "JP", "JAPAN32",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN33,     MKK7_FCCA,     "JP", "JAPAN33",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN34,     MKK9_MKKA,     "JP", "JAPAN34",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN35,     MKK10_MKKA,    "JP", "JAPAN35",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN36,     MKK4_MKKA,     "JP", "JAPAN36",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN37,     MKK9_FCCA,     "JP", "JAPAN37",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN38,     MKK9_MKKA1,    "JP", "JAPAN38",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN39,     MKK9_MKKC,     "JP", "JAPAN39",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN40,     MKK9_MKKA2,    "JP", "JAPAN40",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN41,     MKK10_FCCA,    "JP", "JAPAN41",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN42,     MKK10_MKKA1,   "JP", "JAPAN42",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN43,     MKK10_MKKC,    "JP", "JAPAN43",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN44,     MKK10_MKKA2,   "JP", "JAPAN44",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN45,     MKK11_MKKA,    "JP", "JAPAN45",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN46,     MKK11_FCCA,    "JP", "JAPAN46",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN47,     MKK11_MKKA1,   "JP", "JAPAN47",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN48,     MKK11_MKKC,    "JP", "JAPAN48",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN49,     MKK11_MKKA2,   "JP", "JAPAN49",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN50,     MKK12_MKKA,    "JP", "JAPAN50",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN51,     MKK12_FCCA,    "JP", "JAPAN51",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN52,     MKK12_MKKA1,   "JP", "JAPAN52",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN53,     MKK12_MKKC,    "JP", "JAPAN53",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN54,     MKK12_MKKA2,   "JP", "JAPAN54",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
/*    {CTRY_JAPAN55,     MKK5_MKKA,     "JP", "JAPAN55",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },*/
    {CTRY_JAPAN56,     MKK5_FCCA,     "JP", "JAPAN56",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN57,     MKK13_MKKB,    "JP", "JAPAN57",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN58,     MKK14_MKKA1,   "JP", "JAPAN58",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JAPAN59,     MKK15_MKKA1,   "JP", "JAPAN59",        YES,  NO,  NO, YES, YES, YES, YES, NO, 7000 },
    {CTRY_JORDAN,      ETSI2_WORLD,   "JO", "JORDAN",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_KAZAKHSTAN,  NULL1_WORLD,   "KZ", "KAZAKHSTAN",     YES,  NO, YES, YES, YES,  NO,  NO, NO, 7000 },
    {CTRY_KENYA,       APL1_WORLD,    "KE", "KENYA",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_KOREA_NORTH, APL9_MKKC,     "KP", "NORTH KOREA",    YES,  NO,  NO, YES,  NO, YES,  NO, NO, 7000 },
    {CTRY_KOREA_ROC,   APL10_MKKC,    "KR", "KOREA REPUBLIC", YES,  NO,  NO, YES, YES, YES, YES, YES, 7000 },
    {CTRY_KOREA_ROC3,  APL9_MKKC,     "KR", "KOREA REPUBLIC3", YES,  NO,  NO, YES,  NO, YES,  NO, NO, 7000 },
    {CTRY_KUWAIT,      ETSI3_WORLD,   "KW", "KUWAIT",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_LATVIA,      ETSI1_WORLD,   "LV", "LATVIA",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_LEBANON,     APL1_WORLD,    "LB", "LEBANON",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_LIECHTENSTEIN, ETSI1_WORLD, "LI", "LIECHTENSTEIN",  YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_LITHUANIA,   ETSI1_WORLD,   "LT", "LITHUANIA",      YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_LUXEMBOURG,  ETSI1_WORLD,   "LU", "LUXEMBOURG",     YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MACAU,       FCC2_WORLD,    "MO", "MACAU SAR",      YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MACEDONIA,   ETSI1_WORLD,   "MK", "MACEDONIA, FYRO", YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MALAYSIA,    FCC1_WORLD,    "MY", "MALAYSIA",       YES,  NO,  NO, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MALTA,       ETSI1_WORLD,   "MT", "MALTA",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MAURITIUS,   ETSI1_WORLD,   "MU", "MAURITIUS",      YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MEXICO,      FCC1_WORLD,    "MX", "MEXICO",         YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MONACO,      ETSI4_WORLD,   "MC", "MONACO",         YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MOROCCO,     APL4_WORLD,    "MA", "MOROCCO",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_NEPAL,       APL1_WORLD,    "NP", "NEPAL",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_NETHERLANDS, ETSI1_WORLD,   "NL", "NETHERLANDS",    YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_NETHERLANDS_ANTILLES, ETSI1_WORLD, "AN", "NETHERLANDS ANTILLES", YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_NEW_ZEALAND, FCC3_ETSIC,    "NZ", "NEW ZEALAND",    YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_NICARAGUA,   FCC3_FCCA,     "NI", "NICARAGUA",      YES,  YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_NORWAY,      ETSI1_WORLD,   "NO", "NORWAY",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_OMAN,        FCC3_WORLD,    "OM", "OMAN",           YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PAKISTAN,    APL1_WORLD,    "PK", "PAKISTAN",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PANAMA,      FCC1_FCCA,     "PA", "PANAMA",         YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PAPUA_NEW_GUINEA, FCC1_WORLD, "PG", "PAPUA NEW GUINEA", YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PARAGUAY,    FCC3_WORLD,    "PY", "PARAGUAY",       YES,  YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PERU,        FCC3_WORLD,    "PE", "PERU",           YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PHILIPPINES, FCC3_WORLD,    "PH", "PHILIPPINES",    YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_POLAND,      ETSI1_WORLD,   "PL", "POLAND",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PORTUGAL,    ETSI1_WORLD,   "PT", "PORTUGAL",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_PUERTO_RICO, FCC1_FCCA,     "PR", "PUERTO RICO",    YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_QATAR,       APL1_WORLD,    "QA", "QATAR",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_ROMANIA,     ETSI1_WORLD,   "RO", "ROMANIA",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_RUSSIA,      ETSI8_WORLD,   "RU", "RUSSIA",         YES,  NO, NO,  YES, YES, YES, YES, NO, 7000 },
    {CTRY_RWANDA,      APL1_WORLD,    "RW", "RWANDA",         YES,  NO, YES, YES, YES, YES,  YES, YES, 7000 },
    {CTRY_SAUDI_ARABIA, FCC2_WORLD,   "SA", "SAUDI ARABIA",   YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_SERBIA,      ETSI1_WORLD,   "RS", "REPUBLIC OF SERBIA", YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_MONTENEGRO,  ETSI1_WORLD,   "ME", "MONTENEGRO",     YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SINGAPORE,   FCC3_WORLD,    "SG", "SINGAPORE",      YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SLOVAKIA,    ETSI1_WORLD,   "SK", "SLOVAKIA",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SLOVENIA,    ETSI1_WORLD,   "SI", "SLOVENIA",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SOUTH_AFRICA, FCC3_WORLD,   "ZA", "SOUTH AFRICA",   YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SPAIN,       ETSI1_WORLD,   "ES", "SPAIN",          YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SRI_LANKA,   FCC3_WORLD,    "LK", "SRI LANKA",      YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_SWEDEN,      ETSI1_WORLD,   "SE", "SWEDEN",         YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SWITZERLAND, ETSI1_WORLD,   "CH", "SWITZERLAND",    YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_SYRIA,       NULL1_WORLD,   "SY", "SYRIAN ARAB REPUBLIC", YES,  NO, YES, YES, YES,  NO,  NO, NO, 7000 },
    {CTRY_TAIWAN,      APL7_FCCA,     "TW", "TAIWAN",         YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_TANZANIA,    APL1_WORLD,    "TZ", "TANZANIA",       YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_THAILAND,    FCC3_WORLD,    "TH", "THAILAND",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_TRINIDAD_Y_TOBAGO, FCC3_WORLD, "TT", "TRINIDAD AND TOBAGO", YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_TUNISIA,     ETSI3_WORLD,   "TN", "TUNISIA",        YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_TURKEY,      ETSI3_WORLD,   "TR", "TURKEY",         YES,  NO, YES, YES, YES, YES,  NO, NO, 7000 },
    {CTRY_UGANDA,      FCC3_WORLD,    "UG", "UGANDA",         YES,  NO, NO, YES, YES, YES,  YES, YES, 7000 },
    {CTRY_UKRAINE,     ETSI9_WORLD,   "UA", "UKRAINE",        YES,  NO, NO,  YES, YES, YES,  YES, NO, 7000 },
    {CTRY_UAE,         ETSI1_WORLD, "AE", "UNITED ARAB EMIRATES", YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_UNITED_KINGDOM, ETSI1_WORLD, "GB", "UNITED KINGDOM", YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_UNITED_STATES, FCC3_FCCA,   "US", "UNITED STATES",  YES, YES, YES, YES, YES, YES, YES, YES, 5825 },
    {CTRY_UNITED_STATES2, FCC6_FCCA,  "US", "UNITED STATES2",  YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_UNITED_STATES_FCC49, FCC4_FCCA,   "PS", "UNITED STATES (PUBLIC SAFETY)",  YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_URUGUAY,     FCC3_WORLD,    "UY", "URUGUAY",        YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_UZBEKISTAN,  FCC3_FCCA,     "UZ", "UZBEKISTAN",     YES, YES, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_VENEZUELA,   FCC1_WORLD,    "VE", "VENEZUELA",      YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_VIET_NAM,    ETSI3_WORLD,   "VN", "VIET NAM",       YES,  NO, YES, YES, YES, YES, YES, YES, 7000 },
    {CTRY_YEMEN,       NULL1_WORLD,   "YE", "YEMEN",          YES,  NO, YES, YES, YES,  NO,  NO, NO, 7000 },
    {CTRY_ZIMBABWE,    ETSI1_WORLD,   "ZW", "ZIMBABWE",       YES,  NO, YES, YES, YES,  NO,  NO, NO, 7000 }
};


/* Bit masks for DFS per regdomain */

enum {
	NO_DFS   = 0x0000000000000000ULL,
	DFS_FCC3 = 0x0000000000000001ULL,
	DFS_ETSI = 0x0000000000000002ULL,
	DFS_MKK4 = 0x0000000000000004ULL,
};


/* The table of frequency bands is indexed by a bitmask.  The ordering
 * must be consistent with the enum below.  When adding a new
 * frequency band, be sure to match the location in the enum with the
 * comments
 */

/*
 * 5GHz 11A channel tags
 */
enum {
    F1_4912_4947,
    F1_4915_4925,
    F2_4915_4925,
    F1_4935_4945,
    F2_4935_4945,
    F1_4920_4980,
    F2_4920_4980,
    F1_4942_4987,
    F1_4945_4985,
    F1_4950_4980,
    F1_5032_5057,
    F1_5035_5040,
    F2_5035_5040,
    F1_5035_5045,
    F1_5040_5040,
    F1_5040_5080,
    F2_5040_5080,
    F1_5055_5055,
    F2_5055_5055,

    F1_5120_5240,

    F1_5170_5230,
    F2_5170_5230,

    F1_5180_5240,
    F2_5180_5240,
    F3_5180_5240,
    F4_5180_5240,
    F5_5180_5240,
    F6_5180_5240,
    F7_5180_5240,
    F8_5180_5240,
    F9_5180_5240,
    F10_5180_5240,

    F1_5240_5280,

    F1_5260_5280,

    F1_5260_5320,
    F2_5260_5320,
    F3_5260_5320,
    F4_5260_5320,
    F5_5260_5320,
    F6_5260_5320,
    F7_5260_5320,

    F1_5260_5700,

    F1_5280_5320,
    F2_5280_5320,
    F1_5500_5560,

    F1_5500_5580,
    F2_5500_5580,

    F1_5500_5620,

    F1_5500_5660,

    F1_5500_5720,
    F2_5500_5700,
    F3_5500_5700,
    F4_5500_5700,
    F5_5500_5700,
    F6_5500_5700,

    F1_5660_5700,
    F2_5660_5720,
    F3_5660_5720,

    F1_5745_5765,

    F1_5745_5805,
    F2_5745_5805,
    F3_5745_5805,
    F4_5745_5805,

    F1_5745_5825,
    F2_5745_5825,
    F3_5745_5825,
    F4_5745_5825,
    F5_5745_5825,
    F6_5745_5825,
    F7_5745_5825,
    F8_5745_5825,
    F9_5745_5825,

    F1_5845_5865,

    W1_4920_4980,
    W1_5040_5080,
    W1_5170_5230,
    W1_5180_5240,
    W1_5260_5320,
    W1_5745_5825,
    W1_5500_5700,
    A_DEMO_ALL_CHANNELS
};

static const REG_DMN_FREQ_BAND regDmn5GhzFreq[] = {
	{ 4915, 4925, 20, 0, 10, 5, NO_DFS, PSCAN_MKK2, 16 },				/* F1_4915_4925 */
	{ 4915, 4925, 23, 0, 10, 5, NO_DFS, PSCAN_MKK2, 16 },				/* F2_4915_4925 */
	{ 4935, 4945, 20, 0, 10, 5, NO_DFS, PSCAN_MKK2, 16 },				/* F1_4935_4945 */
	{ 4935, 4945, 23, 0, 10, 5, NO_DFS, PSCAN_MKK2, 16 },				/* F2_4935_4945 */
	{ 4920, 4980, 23, 0, 20, 20, NO_DFS, PSCAN_MKK2, 7 },				/* F1_4920_4980 */
	{ 4920, 4980, 20, 0, 20, 20, NO_DFS, PSCAN_MKK2, 7 },				/* F2_4920_4980 */
	{ 4942, 4987, 27, 6, 5,  5, NO_DFS, PSCAN_FCC, 0 },				/* F1_4942_4987 */
	{ 4945, 4985, 30, 6, 10, 5, NO_DFS, PSCAN_FCC, 0 },				/* F1_4945_4985 */
	{ 4950, 4980, 33, 6, 20, 5, NO_DFS, PSCAN_FCC, 0 },				/* F1_4950_4980 */
	{ 5035, 5040, 23, 0, 10, 5, NO_DFS, PSCAN_MKK2, 12 },				/* F1_5035_5040 */
	{ 5035, 5040, 23, 0, 10, 5, NO_DFS, PSCAN_MKK2, 12 },				/* F2_5035_5040 */
	{ 5040, 5040, 20, 0, 10, 5, NO_DFS, PSCAN_MKK2, 12 },				/* F1_5040_5040 */
	{ 5040, 5080, 23, 0, 20, 20, NO_DFS, PSCAN_MKK2, 2 },				/* F1_5040_5080 */
	{ 5040, 5080, 20, 0, 20, 20, NO_DFS, NO_PSCAN, 6 },					/* F2_5040_5080 */
	{ 5055, 5055, 20, 0, 10, 5, NO_DFS, PSCAN_MKK2, 12 },				/* F1_5055_5055 */
	{ 5055, 5055, 23, 0, 10, 5, NO_DFS, PSCAN_MKK2, 12 },				/* F2_5055_5055 */

	{ 5120, 5240, 5,  6, 20, 20, NO_DFS, NO_PSCAN, 0 },				/* F1_5120_5240 */

	{ 5170, 5230, 23, 0, 20, 20, NO_DFS, PSCAN_MKK1 | PSCAN_MKK2, 1 },		/* F1_5170_5230 */
	{ 5170, 5230, 20, 0, 20, 20, NO_DFS, PSCAN_MKK1 | PSCAN_MKK2, 1 },		/* F2_5170_5230 */

	{ 5180, 5240, 15, 0, 20, 20, NO_DFS, PSCAN_FCC | PSCAN_ETSI, 1 },		/* F1_5180_5240 */
	{ 5180, 5240, 17, 6, 20, 20, NO_DFS, NO_PSCAN, 1 },				/* F2_5180_5240 */
	{ 5180, 5240, 18, 0, 20, 20, NO_DFS, PSCAN_FCC | PSCAN_ETSI, 1 },		/* F3_5180_5240 */
	{ 5180, 5240, 20, 0, 20, 20, NO_DFS, PSCAN_FCC | PSCAN_ETSI, 1 },		/* F4_5180_5240 */
	{ 5180, 5240, 23, 0, 20, 20, NO_DFS, PSCAN_FCC | PSCAN_ETSI, 1 },		/* F5_5180_5240 */
	{ 5180, 5240, 23, 6, 20, 20, NO_DFS, PSCAN_FCC, 1 },				/* F6_5180_5240 */
	{ 5180, 5240, 20, 0, 20, 20, NO_DFS, PSCAN_MKK1 | PSCAN_MKK3, 0 },		/* F7_5180_5240 */
	{ 5180, 5240, 23, 6, 20, 20, NO_DFS, NO_PSCAN, 1 },                             /* F8_5180_5240 */
	{ 5180, 5240, 20, 6, 20, 20, NO_DFS, PSCAN_ETSI, 0 },				/* F9_5180_5240 */
	{ 5180, 5240, 23, 0, 20, 20, NO_DFS, PSCAN_FCC | PSCAN_ETSI, 1 },		/* F10_5180_5240 */

	{ 5240, 5280, 23, 0, 20, 20, DFS_FCC3, PSCAN_FCC | PSCAN_ETSI, 0 },		/* F1_5240_5280 */

	{ 5260, 5280, 23, 0, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 2 },	/* F1_5260_5280 */

	{ 5260, 5320, 18, 0, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 2 },	/* F1_5260_5320 */

	{ 5260, 5320, 20, 0, 20, 20, DFS_FCC3 | DFS_ETSI | DFS_MKK4, PSCAN_FCC | PSCAN_ETSI | PSCAN_MKK3 , 0 },
	/* F2_5260_5320 */

	{ 5260, 5320, 24, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 2 },/* F3_5260_5320 */
	{ 5260, 5320, 23, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC, 2 },		/* F4_5260_5320 */
	{ 5260, 5320, 23, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC, 2 },		/* F5_5260_5320 */
	{ 5260, 5320, 30, 0, 20, 20, NO_DFS, NO_PSCAN, 2 },				/* F6_5260_5320 */
	{ 5260, 5320, 23, 0, 20, 20, DFS_FCC3 | DFS_ETSI | DFS_MKK4, PSCAN_FCC | PSCAN_ETSI | PSCAN_MKK3 , 0 },
	/* F7_5260_5320 */

	{ 5260, 5700, 5,  6, 20, 20, DFS_FCC3 | DFS_ETSI, NO_PSCAN, 0 },		/* F1_5260_5700 */

	{ 5280, 5320, 17, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC, 2 },		/* F1_5280_5320 */

	{ 5500, 5580, 23, 6, 20, 20, DFS_FCC3, PSCAN_FCC, 4},                           /* F1_5500_5580 */
	{ 5500, 5580, 30, 6, 20, 20, DFS_FCC3, PSCAN_FCC, 4},                           /* F2_5500_5580 */

	{ 5500, 5620, 30, 6, 20, 20, DFS_ETSI, PSCAN_ETSI, 3 },				/* F1_5500_5620 */

    { 5500, 5660, 20, 0, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 0 },   /* F1_5500_5660 */

	{ 5500, 5720, 24, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC, 4 },		/* F1_5500_5720 */
	{ 5500, 5700, 27, 0, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 3 },	/* F2_5500_5700 */
	{ 5500, 5700, 30, 0, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 3 },	/* F3_5500_5700 */
	{ 5500, 5700, 23, 0, 20, 20, DFS_FCC3 | DFS_ETSI | DFS_MKK4, PSCAN_MKK3 | PSCAN_FCC, 0 },/* F4_5500_5700 */
	{ 5500, 5700, 30, 6, 20, 20, DFS_ETSI, PSCAN_ETSI, 0 },					/* F5_5500_5700 */
	{ 5500, 5700, 20, 0, 20, 20, DFS_FCC3 | DFS_ETSI | DFS_MKK4, PSCAN_MKK3 | PSCAN_FCC, 0 },/* F6_5500_5700 */

	{ 5660, 5700, 20, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 4},           /* F1_5660_5700 */
	{ 5660, 5700, 23, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 4},           /* F2_5660_5700 */
	{ 5660, 5700, 30, 6, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI, 4},           /* F3_5660_5700 */

	{ 5745, 5805, 23, 0, 20, 20, NO_DFS, NO_PSCAN, 3 },				/* F1_5745_5805 */
	{ 5745, 5805, 30, 6, 20, 20, NO_DFS, NO_PSCAN, 3 },				/* F2_5745_5805 */
	{ 5745, 5805, 30, 6, 20, 20, NO_DFS, PSCAN_ETSI, 0 },			/* F3_5745_5805 */
    { 5745, 5805, 20, 0, 20, 20, NO_DFS, NO_PSCAN, 0 },			    /* F4_5745_5805 */

	{ 5745, 5825, 5,  6, 20, 20, NO_DFS, NO_PSCAN, 5 },				/* F1_5745_5825 */
	{ 5745, 5825, 17, 0, 20, 20, NO_DFS, NO_PSCAN, 5 },				/* F2_5745_5825 */
	{ 5745, 5825, 20, 0, 20, 20, NO_DFS, NO_PSCAN, 0 },				/* F3_5745_5825 */
	{ 5745, 5825, 30, 0, 20, 20, NO_DFS, NO_PSCAN, 0 },				/* F4_5745_5825 */
	{ 5745, 5825, 30, 6, 20, 20, NO_DFS, NO_PSCAN, 5 },				/* F5_5745_5825 */
	{ 5745, 5825, 30, 6, 20, 20, NO_DFS, NO_PSCAN, 5 },				/* F6_5745_5825 */
    { 5745, 5825, 30, 6, 20, 20, NO_DFS, PSCAN_ETSI, 0 },			/* F7_5745_5825 */
        { 5745, 5825, 20, 6, 20, 20, NO_DFS, PSCAN_ETSI, 0 },			/* F8_5745_5825 */

	/*
	 * Below are the world roaming channels
	 * All WWR domains have no power limit, instead use the card's CTL
	 * or max power settings.
	 */
	{ 4920, 4980, 30, 0, 20, 20, NO_DFS, PSCAN_WWR, 0 },				/* W1_4920_4980 */
	{ 5040, 5080, 30, 0, 20, 20, NO_DFS, PSCAN_WWR, 0 },				/* W1_5040_5080 */
	{ 5170, 5230, 30, 0, 20, 20, NO_DFS, PSCAN_WWR, 0 },				/* W1_5170_5230 */
	{ 5180, 5240, 30, 0, 20, 20, NO_DFS, PSCAN_WWR, 0 },				/* W1_5180_5240 */
	{ 5260, 5320, 30, 0, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, 0 },		/* W1_5260_5320 */
	{ 5745, 5825, 30, 0, 20, 20, NO_DFS, PSCAN_WWR, 0 },				/* W1_5745_5825 */
	{ 5500, 5700, 30, 0, 20, 20, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, 0 },		/* W1_5500_5700 */
	{ 4920, 6100, 30, 6, 20, 20, NO_DFS, NO_PSCAN, 0 },				/* A_DEMO_ALL_CHANNELS */
};

/*
 * 2GHz 11b channel tags
 */
enum {
	F1_2312_2372,
	F2_2312_2372,

	F1_2412_2472,
	F2_2412_2472,
	F3_2412_2472,
	F4_2412_2472,

	F1_2412_2462,
	F2_2412_2462,

	F1_2432_2442,

	F1_2457_2472,

	F1_2467_2472,

	F1_2484_2484,
	F2_2484_2484,

	F1_2512_2732,

	W1_2312_2372,
	W1_2412_2412,
	W1_2417_2432,
	W1_2437_2442,
	W1_2447_2457,
	W1_2462_2462,
	W1_2467_2467,
	W2_2467_2467,
	W1_2472_2472,
	W2_2472_2472,
	W1_2484_2484,
	W2_2484_2484,
};

static const REG_DMN_FREQ_BAND regDmn2GhzFreq[] = {
	{ 2312, 2372, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* F1_2312_2372 */
	{ 2312, 2372, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* F2_2312_2372 */

	{ 2412, 2472, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* F1_2412_2472 */
	{ 2412, 2472, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA, 30},/* F2_2412_2472 */
	{ 2412, 2472, 30, 0, 20, 5, NO_DFS, NO_PSCAN, 4},	/* F3_2412_2472 */
	{ 2412, 2472, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA, 0},	/* F4_2412_2472 */

	{ 2412, 2462, 30, 6, 20, 5, NO_DFS, NO_PSCAN, 12},	/* F1_2412_2462 */
	{ 2412, 2462, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA, 30},	/* F2_2412_2462 */

	{ 2432, 2442, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 4},	/* F1_2432_2442 */

	{ 2457, 2472, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* F1_2457_2472 */

	{ 2467, 2472, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA2 | PSCAN_MKKA, 30}, /* F1_2467_2472 */

	{ 2484, 2484, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* F1_2484_2484 */
	{ 2484, 2484, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA | PSCAN_MKKA1 | PSCAN_MKKA2, 31},	/* F2_2484_2484 */

	{ 2512, 2732, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* F1_2512_2732 */

	/*
	 * WWR have powers opened up to 20dBm.  Limits should often come from CTL/Max powers
	 */

	{ 2312, 2372, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* W1_2312_2372 */
	{ 2412, 2412, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* W1_2412_2412 */
	{ 2417, 2432, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* W1_2417_2432 */
	{ 2437, 2442, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* W1_2437_2442 */
	{ 2447, 2457, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* W1_2447_2457 */
	{ 2462, 2462, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* W1_2462_2462 */
	{ 2467, 2467, 20, 0, 20, 5, NO_DFS, PSCAN_WWR | IS_ECM_CHAN, 0}, /* W1_2467_2467 */
	{ 2467, 2467, 20, 0, 20, 5, NO_DFS, NO_PSCAN | IS_ECM_CHAN, 0},	/* W2_2467_2467 */
	{ 2472, 2472, 20, 0, 20, 5, NO_DFS, PSCAN_WWR | IS_ECM_CHAN, 0}, /* W1_2472_2472 */
	{ 2472, 2472, 20, 0, 20, 5, NO_DFS, NO_PSCAN | IS_ECM_CHAN, 0},	/* W2_2472_2472 */
	{ 2484, 2484, 20, 0, 20, 5, NO_DFS, PSCAN_WWR | IS_ECM_CHAN, 0}, /* W1_2484_2484 */
	{ 2484, 2484, 20, 0, 20, 5, NO_DFS, NO_PSCAN | IS_ECM_CHAN, 0},	/* W2_2484_2484 */
};

/*
 * 2GHz 11g channel tags
 */

enum {
	G1_2312_2372,
	G2_2312_2372,

	G1_2412_2472,
	G2_2412_2472,
	G3_2412_2472,
	G4_2412_2472,

	G1_2412_2462,
	G2_2412_2462,

	G1_2432_2442,

	G1_2457_2472,

	G1_2512_2732,

	G1_2467_2472,
	G2_2467_2472,

	G1_2484_2484,

	WG1_2312_2372,
	WG1_2412_2462,
	WG1_2412_2472,
	WG2_2412_2472,
	G_DEMO_ALMOST_ALL_CHANNELS,
	G_DEMO_ALL_CHANNELS,
};

static const REG_DMN_FREQ_BAND regDmn2Ghz11gFreq[] = {
	{ 2312, 2372, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* G1_2312_2372 */
	{ 2312, 2372, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* G2_2312_2372 */

	{ 2412, 2472, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* G1_2412_2472 */
	{ 2412, 2472, 20, 0, 20, 5,  NO_DFS, PSCAN_MKKA_G | PSCAN_MKKA2 | PSCAN_MKKA | PSCAN_EXT_CHAN, 30},	/* G2_2412_2472 */
	{ 2412, 2472, 30, 0, 20, 5, NO_DFS, NO_PSCAN, 4},	/* G3_2412_2472 */
	{ 2412, 2472, 20, 0, 20, 5,  NO_DFS, PSCAN_MKKA_G | PSCAN_MKKA2 | PSCAN_MKKA | PSCAN_EXT_CHAN, 0},	/* G4_2412_2472 */

	{ 2412, 2462, 30, 6, 20, 5, NO_DFS, NO_PSCAN, 12},	/* G1_2412_2462 */
	{ 2412, 2462, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA_G, 30},	/* G2_2412_2462 */

	{ 2432, 2442, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 4},	/* G1_2432_2442 */

	{ 2457, 2472, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* G1_2457_2472 */

	{ 2512, 2732, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* G1_2512_2732 */

	{ 2467, 2472, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA2 | PSCAN_MKKA, 30 }, /* G1_2467_2472 */
	{ 2467, 2472, 20, 0, 20, 5, NO_DFS, PSCAN_MKKA_G | PSCAN_MKKA2, 0 }, /* G2_2467_2472 */

	{ 2484, 2484, 5,  6, 20, 5, NO_DFS, NO_PSCAN, 0},	/* G1_2484_2484 */
	/*
	 * WWR open up the power to 20dBm
	 */

	{ 2312, 2372, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* WG1_2312_2372 */
	{ 2412, 2462, 20, 0, 20, 5, NO_DFS, NO_PSCAN, 0},	/* WG1_2412_2462 */
	{ 2412, 2472, 20, 0, 20, 5, NO_DFS, PSCAN_WWR | IS_ECM_CHAN | PSCAN_EXT_CHAN, 0}, /* WG1_2412_2472 */
	{ 2412, 2472, 20, 0, 20, 5, NO_DFS, NO_PSCAN | IS_ECM_CHAN, 0}, /* WG2_2412_2472 */
	{ 2312, 2532, 27, 6, 20, 5, NO_DFS, NO_PSCAN, 0},		/* G_DEMO_ALMOST_ALL_CHANNELS */
	{ 2312, 2732, 27, 6, 20, 5, NO_DFS, NO_PSCAN, 0},		/* G_DEMO_ALL_CHANNELS */
};

/* regulatory capabilities */
#define REGDMN_EEPROM_EEREGCAP_EN_KK_U1_EVEN    0x0080
#define REGDMN_EEPROM_EEREGCAP_EN_KK_U2         0x0100
#define REGDMN_EEPROM_EEREGCAP_EN_KK_MIDBAND    0x0200
#define REGDMN_EEPROM_EEREGCAP_EN_KK_U1_ODD     0x0400

static const JAPAN_BANDCHECK j_bandcheck[] = {
	{F1_5170_5230, REGDMN_EEPROM_EEREGCAP_EN_KK_U1_ODD},
	{F4_5180_5240, REGDMN_EEPROM_EEREGCAP_EN_KK_U1_EVEN},
	{F2_5260_5320, REGDMN_EEPROM_EEREGCAP_EN_KK_U2},
	{F4_5500_5700, REGDMN_EEPROM_EEREGCAP_EN_KK_MIDBAND}
};

static const COMMON_MODE_POWER common_mode_pwrtbl[] = {
	{ 4900, 5000, 17 },
	{ 5000, 5100, 17 },
	{ 5150, 5250, 17 }, /* ETSI & MKK */
	{ 5250, 5350, 18 }, /* ETSI */
	{ 5470, 5725, 20 }, /* ETSI */
	{ 5725, 5825, 20 }, /* Singapore */
	{ 5825, 5850, 23 }  /* Korea */
};

/*
 * 5GHz Turbo (dynamic & static) tags
 */

enum {
    T1_5130_5650,
    T1_5150_5670,

    T1_5200_5200,
    T2_5200_5200,
    T3_5200_5200,
    T4_5200_5200,
    T5_5200_5200,
    T6_5200_5200,
    T7_5200_5200,
    T8_5200_5200,

    T1_5200_5280,
    T2_5200_5280,
    T3_5200_5280,
    T4_5200_5280,
    T5_5200_5280,
    T6_5200_5280,

    T1_5200_5240,
    T1_5210_5210,
    T2_5210_5210,
    T3_5210_5210,
    T4_5210_5210,
    T5_5210_5210,
    T6_5210_5210,
    T7_5210_5210,
    T8_5210_5210,
    T9_5210_5210,
    T10_5210_5210,
    T1_5240_5240,

    T1_5210_5250,
    T1_5210_5290,
    T2_5210_5290,
    T3_5210_5290,

    T1_5280_5280,
    T2_5280_5280,
    T1_5290_5290,
    T2_5290_5290,
    T3_5290_5290,
    T1_5250_5290,
    T2_5250_5290,
    T3_5250_5290,
    T4_5250_5290,

    T1_5540_5660,
    T2_5540_5660,
    T3_5540_5660,
    T1_5760_5800,
    T2_5760_5800,
    T3_5760_5800,
    T4_5760_5800,
    T5_5760_5800,
    T6_5760_5800,
    T7_5760_5800,

    T1_5765_5805,
    T2_5765_5805,
    T3_5765_5805,
    T4_5765_5805,
    T5_5765_5805,
    T6_5765_5805,
    T7_5765_5805,
    T8_5765_5805,
    T9_5765_5805,

    WT1_5210_5250,
    WT1_5290_5290,
    WT1_5540_5660,
    WT1_5760_5800,
};

/*
 * 2GHz Dynamic turbo tags
 */
#ifndef ATH_REMOVE_2G_TURBO_RD_TABLE
enum {
    T1_2312_2372,
    T1_2437_2437,
    T2_2437_2437,
    T3_2437_2437,
    T1_2512_2732
};

static const REG_DMN_FREQ_BAND regDmn2Ghz11gTurboFreq[] = {
    { 2312, 2372, 5,  6, 40, 40, NO_DFS, NO_PSCAN, 0},  /* T1_2312_2372 */
    { 2437, 2437, 5,  6, 40, 40, NO_DFS, NO_PSCAN, 0},  /* T1_2437_2437 */
    { 2437, 2437, 20, 6, 40, 40, NO_DFS, NO_PSCAN, 0},  /* T2_2437_2437 */
    { 2437, 2437, 18, 6, 40, 40, NO_DFS, PSCAN_WWR, 0}, /* T3_2437_2437 */
    { 2512, 2732, 5,  6, 40, 40, NO_DFS, NO_PSCAN, 0},  /* T1_2512_2732 */
};
#endif /* ATH_REMOVE_2G_TURBO_RD_TABLE */

static const REG_DOMAIN ahCmnRegDomains[] = {

	{DEBUG_REG_DMN, FCC, DFS_FCC3, NO_PSCAN, NO_REQ,
		CHAN_11A_BM(A_DEMO_ALL_CHANNELS, F6_5745_5825,
				-1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
		CHAN_11A_BM(T1_5130_5650, T1_5150_5670, F6_5745_5825,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
		CHAN_11A_BM(T1_5200_5240, T1_5280_5280, T1_5540_5660, T1_5765_5805,
					-1, -1, -1, -1, -1, -1, -1, -1)
		BM(F1_2312_2372, F1_2412_2472, F1_2484_2484, F1_2512_2732,
					-1, -1, -1, -1, -1, -1, -1, -1),
		BM(G_DEMO_ALMOST_ALL_CHANNELS,
				G1_2484_2484, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T1_2312_2372, T1_2437_2437, T1_2512_2732,
				-1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{APL1, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		BM(F4_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL2, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		BM(F1_5745_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL3, FCC, DFS_FCC3, PSCAN_FCC, NO_REQ,
		BM(F1_5280_5320, F6_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5290_5290, T1_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL4, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		BM(F5_5180_5240,  F9_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5210_5210, T3_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5200, T3_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL5, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		BM(F2_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T4_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T4_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL6, ETSI, DFS_ETSI, PSCAN_FCC_T | PSCAN_FCC , NO_REQ,
		BM(F9_5180_5240, F2_5260_5320, F3_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5210_5210, T1_5250_5290, T1_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5280, T5_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL7, FCC, DFS_FCC3 | DFS_ETSI, PSCAN_FCC | PSCAN_ETSI , NO_REQ,
		BM(F2_5280_5320, F2_5500_5580, F3_5660_5720, F7_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5290_5290, T5_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5540_5660, T6_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL8, ETSI, NO_DFS, NO_PSCAN, DISALLOW_ADHOC_11A|DISALLOW_ADHOC_11A_TURB,
		BM(F6_5260_5320, F4_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5290_5290, T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5280_5280, T1_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL9, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A|DISALLOW_ADHOC_11A_TURB,
		BM(F9_5180_5240, F2_5260_5320, F1_5500_5620, F3_5745_5805, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5290_5290, T5_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5540_5660, T6_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL10, ETSI, DFS_ETSI, PSCAN_ETSI , DISALLOW_ADHOC_11A|DISALLOW_ADHOC_11A_TURB,
		BM(F9_5180_5240, F2_5260_5320, F5_5500_5700, F3_5745_5805, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5290_5290, T5_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5540_5660, T6_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL11, ETSI, DFS_ETSI, PSCAN_ETSI , DISALLOW_ADHOC_11A|DISALLOW_ADHOC_11A_TURB,
		BM(F9_5180_5240, F2_5260_5320, F5_5500_5700, F7_5745_5825, F1_5845_5865, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5290_5290, T5_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5540_5660, T6_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL12, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A|DISALLOW_ADHOC_11A_TURB,
		BM(F5_5180_5240, F1_5500_5560, F1_5745_5765, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{APL13, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A|DISALLOW_ADHOC_11A_TURB,
		BM(F5_5180_5240, F1_5500_5560, F1_5745_5765, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{ETSI1, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F2_5180_5240, F2_5260_5320, F2_5500_5700, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5200_5280, T2_5540_5660, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{ETSI2, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F3_5180_5240, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{ETSI3, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F4_5180_5240, F2_5260_5320, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5200_5280, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{ETSI4, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F3_5180_5240, F1_5260_5320, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5200_5280, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{ETSI5, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F1_5180_5240, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T4_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{ETSI6, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F5_5180_5240, F1_5260_5280, F3_5500_5700, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5210_5250, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T4_5200_5280, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{ETSI8, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F4_5180_5240, F2_5260_5320, F1_5660_5700, F4_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5200_5280, T2_5540_5660, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		BMZERO
	},

	{ETSI9, ETSI, DFS_ETSI, PSCAN_ETSI, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F4_5180_5240, F2_5260_5320, F1_5500_5660, F8_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T2_5200_5280, T2_5540_5660, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		BMZERO
	},

	{FCC1, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		BM(F2_5180_5240, F4_5260_5320, F5_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T6_5210_5210, T2_5250_5290, T6_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5240, T2_5280_5280, T7_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{FCC2, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		BM(F6_5180_5240, F5_5260_5320, F6_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T7_5210_5210, T3_5250_5290, T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T7_5200_5200, T1_5240_5240, T2_5280_5280, T1_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{FCC3, FCC, DFS_FCC3, PSCAN_FCC | PSCAN_FCC_T, NO_REQ,
		BM(F2_5180_5240, F3_5260_5320, F1_5500_5720, F5_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T6_5210_5210, T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T4_5200_5200, T8_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},
	/*

		Bug Fix: EV 98583 Public Safety channel
		Exclude the following channel in FCC Public safety domain
		Uni-1: 5180, 5200, 5220, 5240
		Uni-2: 5260, 5280, 5300, 5320
		Uni-3: 5745, 5765, 5785, 5805, 5825
		*/
	{FCC4, FCC, DFS_FCC3, PSCAN_FCC | PSCAN_FCC_T, NO_REQ,
		BM(F1_4942_4987, F1_4945_4985, F1_4950_4980, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T8_5210_5210, T4_5250_5290, T7_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5240, T1_5280_5280, T9_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{FCC5, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		BM(F2_5180_5240, F6_5745_5825, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T6_5210_5210, T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T8_5200_5200, T7_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{FCC6, FCC, DFS_FCC3, PSCAN_FCC, NO_REQ,
		BM(F8_5180_5240, F5_5260_5320, F1_5500_5580, F2_5660_5720, F6_5745_5825, -1, -1, -1, -1, -1, -1, -1),
		BM(T7_5210_5210, T3_5250_5290, T2_5760_5800, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T7_5200_5200, T1_5240_5240, T2_5280_5280, T1_5765_5805, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{MKK1, MKK, DFS_MKK4, PSCAN_MKK1 | PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F1_5170_5230, F10_5180_5240, F7_5260_5320, F4_5500_5700, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T7_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T5_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	{MKK2, MKK, DFS_MKK4, PSCAN_MKK2 | PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F2_4915_4925, F2_4935_4945, F1_4920_4980, F1_5035_5040, F2_5055_5055, F1_5040_5080, F1_5170_5230, F10_5180_5240, -1, -1, -1, -1),
		BM(T7_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T5_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 even */
	{MKK3, MKK, NO_DFS, PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F4_5180_5240, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T9_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 even + UNI-2 */
	{MKK4, MKK, DFS_MKK4, PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F4_5180_5240, F2_5260_5320, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T10_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T6_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 even + UNI-2 + mid-band */
	{MKK5, MKK, DFS_MKK4, PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F4_5180_5240, F2_5260_5320, F6_5500_5700, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T5_5200_5280, T3_5540_5660, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 odd + even */
	{MKK6, MKK, NO_DFS, PSCAN_MKK1, DISALLOW_ADHOC_11A_TURB,
		BM(F2_5170_5230, F4_5180_5240, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T6_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 odd + UNI-1 even + UNI-2 */
	{MKK7, MKK, DFS_MKK4, PSCAN_MKK1 | PSCAN_MKK3 , DISALLOW_ADHOC_11A_TURB,
		BM(F2_5170_5230, F4_5180_5240, F2_5260_5320, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T5_5200_5280, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 odd + UNI-1 even + UNI-2 + mid-band */
	{MKK8, MKK, DFS_MKK4, PSCAN_MKK1 | PSCAN_MKK3 , DISALLOW_ADHOC_11A_TURB,
		BM(F2_5170_5230, F4_5180_5240, F2_5260_5320, F6_5500_5700, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T3_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T5_5200_5280, T3_5540_5660, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 even + 4.9 GHZ */
	{MKK9, MKK, NO_DFS, PSCAN_MKK2 | PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F1_4912_4947, F1_5032_5057, F1_4915_4925, F1_4935_4945, F2_4920_4980, F1_5035_5045, F1_5055_5055, F2_5040_5080, F4_5180_5240, -1, -1, -1),
		BM(T9_5210_5210, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5200, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 even + UNI-2 + 4.9 GHZ */
	{MKK10, MKK, DFS_MKK4, PSCAN_MKK2 | PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F1_4912_4947, F1_5032_5057, F1_4915_4925, F1_4935_4945, F2_4920_4980, F1_5035_5045, F1_5055_5055, F2_5040_5080, F4_5180_5240, F2_5260_5320, -1, -1),
		BM(T3_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5280, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* Japan UNI-1 even + UNI-2 + mid-band + 4.9GHz */
	{MKK11, MKK, DFS_MKK4, PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F1_4912_4947, F1_5032_5057, F1_4915_4925, F1_4935_4945, F2_4920_4980, F1_5035_5045, F1_5055_5055, F2_5040_5080, F4_5180_5240, F2_5260_5320, F6_5500_5700, -1),
		BM(T3_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5280, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* Japan UNI-1 even + UNI-1 odd + UNI-2 + mid-band + 4.9GHz */
	{MKK12, MKK, DFS_MKK4, PSCAN_MKK1 | PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F1_4915_4925, F1_4935_4945, F2_4920_4980, F1_5040_5040, F1_5055_5055, F2_5040_5080, F2_5170_5230, F4_5180_5240, F2_5260_5320, F6_5500_5700, -1, -1),
		BM(T3_5210_5290, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(T1_5200_5280, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 odd + UNI-1 even + UNI-2 + mid-band */
	{MKK13, MKK, DFS_MKK4, PSCAN_MKK1 | PSCAN_MKK3 , DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		BM(F2_5170_5230, F7_5180_5240, F2_5260_5320, F6_5500_5700, -1, -1, -1, -1, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 odd + UNI-1 even + 4.9GHz */
	{MKK14, MKK, DFS_MKK4, PSCAN_MKK1, DISALLOW_ADHOC_11A_TURB,
		BM(F1_4915_4925, F1_4935_4945, F2_4920_4980, F1_5040_5040, F2_5040_5080, F1_5055_5055, F2_5170_5230, F4_5180_5240, -1, -1, -1, -1),
		BMZERO,
		BMZERO,
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/* UNI-1 odd + UNI-1 even + UNI-2 + 4.9GHz */
	{MKK15, MKK, DFS_MKK4, PSCAN_MKK1 | PSCAN_MKK3, DISALLOW_ADHOC_11A_TURB,
		BM(F1_4915_4925, F1_4935_4945, F2_4920_4980, F1_5040_5040, F2_5040_5080, F1_5055_5055, F2_5170_5230, F4_5180_5240, F2_5260_5320, -1, -1, -1),
		BMZERO,
		BMZERO,
		BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},

	/*=== 2 GHz ===*/

	/* Defined here to use when 2G channels are authorised for country K2 */
	{APLD, NO_CTL, NO_DFS, NO_PSCAN, NO_REQ,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F2_2312_2372, F4_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G2_2312_2372,G4_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BMZERO
	},

	{ETSIA, NO_CTL, NO_DFS, PSCAN_ETSIA, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F1_2457_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G1_2457_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T2_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{ETSIB, ETSI, NO_DFS, PSCAN_ETSIB, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F1_2432_2442, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G1_2432_2442, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T2_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{ETSIC, ETSI, NO_DFS, PSCAN_ETSIC, DISALLOW_ADHOC_11A | DISALLOW_ADHOC_11A_TURB,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F3_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G3_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T2_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{FCCA, FCC, NO_DFS, NO_PSCAN, NO_REQ,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F1_2412_2462, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G1_2412_2462, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T2_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{MKKA, MKK, NO_DFS, PSCAN_MKKA | PSCAN_MKKA_G | PSCAN_MKKA1 | PSCAN_MKKA1_G | PSCAN_MKKA2 | PSCAN_MKKA2_G, DISALLOW_ADHOC_11A_TURB,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F2_2412_2462, F1_2467_2472, F2_2484_2484,
					-1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G2_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T2_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{MKKC, MKK, NO_DFS, NO_PSCAN, NO_REQ,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F2_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G2_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T2_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WORLD, ETSI, NO_DFS, NO_PSCAN, NO_REQ,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(F4_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		BM(G4_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T2_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR0_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_PER_11D,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5745_5825, W1_5500_5700,
				-1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, W1_2484_2484, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR01_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_PER_11D,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5170_5230, W1_5745_5825,
				W1_5500_5700, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2417_2432, W1_2447_2457,
					-1, -1, -1, -1, -1, -1, -1),
		BM(WG1_2412_2462, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR02_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_PER_11D,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5170_5230, W1_5745_5825,
				W1_5500_5700, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, -1, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{EU1_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_PER_11D,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5170_5230, W1_5745_5825,
				W1_5500_5700, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W2_2472_2472, W1_2417_2432,
					W1_2447_2457, W2_2467_2467, -1, -1, -1, -1, -1),
		BM(WG2_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR1_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_NO_11A,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5170_5230, W1_5745_5825,
				W1_5500_5700, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, W1_2484_2484, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR2_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_NO_11A,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5170_5230, W1_5745_5825,
				W1_5500_5700, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, W1_2484_2484, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR3_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_PER_11D,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5170_5230, W1_5745_5825,
				-1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, -1, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR4_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_NO_11A,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5745_5825,
				-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2417_2432, W1_2447_2457,
					-1, -1, -1, -1, -1, -1, -1),
		BM(WG1_2412_2462, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR5_ETSIC, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_NO_11A,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5745_5825,
				-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, -1, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WOR9_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_NO_11A,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5745_5825, W1_5500_5700,
				-1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BM(WT1_5210_5250, WT1_5290_5290, WT1_5760_5800,
					-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2417_2432, W1_2447_2457,
					-1, -1, -1, -1, -1, -1, -1),
		BM(WG1_2412_2462, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WORA_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_NO_11A,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5745_5825, W1_5500_5700,
				-1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, -1, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WORB_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_NO_11A,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5500_5700,
				-1, -1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, -1, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{WORC_WORLD, NO_CTL, DFS_FCC3 | DFS_ETSI, PSCAN_WWR, ADHOC_PER_11D,
		CHAN_11A_BM(W1_5260_5320, W1_5180_5240, W1_5500_5700, W1_5745_5825,
				-1, -1, -1, -1, -1, -1, -1, -1)
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BM(W1_2412_2412, W1_2437_2442, W1_2462_2462, W1_2472_2472, W1_2417_2432,
					W1_2447_2457, W1_2467_2467, -1, -1, -1, -1, -1),
		BM(WG1_2412_2472, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1),
		CHAN_TURBO_G_BM(T3_2437_2437, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
	},

	{NULL1, NO_CTL, NO_DFS, NO_PSCAN, NO_REQ,
		CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			CHAN_11A_BMZERO
			BMZERO,
		BMZERO,
		CHAN_TURBO_G_BMZERO
	},
};

static const struct cmode modes[] = {
	{ REGDMN_MODE_TURBO,               IEEE80211_CHAN_ST}, /* TURBO means 11a Static Turbo */
	{ REGDMN_MODE_11A,                 IEEE80211_CHAN_A},
	{ REGDMN_MODE_11B,                 IEEE80211_CHAN_B},
	{ REGDMN_MODE_11G,                 IEEE80211_CHAN_PUREG},
	{ REGDMN_MODE_11G_TURBO,           IEEE80211_CHAN_108G},
	{ REGDMN_MODE_11A_TURBO,           IEEE80211_CHAN_108A},
	{ REGDMN_MODE_11NG_HT20,           IEEE80211_CHAN_11NG_HT20},
	{ REGDMN_MODE_11NG_HT40PLUS,       IEEE80211_CHAN_11NG_HT40PLUS},
	{ REGDMN_MODE_11NG_HT40MINUS,      IEEE80211_CHAN_11NG_HT40MINUS},
	{ REGDMN_MODE_11NA_HT20,           IEEE80211_CHAN_11NA_HT20},
	{ REGDMN_MODE_11NA_HT40PLUS,       IEEE80211_CHAN_11NA_HT40PLUS},
	{ REGDMN_MODE_11NA_HT40MINUS,      IEEE80211_CHAN_11NA_HT40MINUS},
	{ REGDMN_MODE_11AC_VHT20,          IEEE80211_CHAN_11AC_VHT20},
	{ REGDMN_MODE_11AC_VHT40PLUS,      IEEE80211_CHAN_11AC_VHT40PLUS},
	{ REGDMN_MODE_11AC_VHT40MINUS,     IEEE80211_CHAN_11AC_VHT40MINUS},
	{ REGDMN_MODE_11AC_VHT80,          IEEE80211_CHAN_11AC_VHT80},
	{ REGDMN_MODE_11AC_VHT20_2G,       IEEE80211_CHAN_11AC_VHT20_2G},
	{ REGDMN_MODE_11AC_VHT40_2G,       IEEE80211_CHAN_11AC_VHT40_2G},
	{ REGDMN_MODE_11AC_VHT80_2G,       IEEE80211_CHAN_11AC_VHT80_2G},
};

typedef enum offset
{
	BW20 = 0,
	BW40_LOW_PRIMARY = 1,
	BW40_HIGH_PRIMARY = 3,
	BW80,
	BWALL
} offset_t;

typedef struct _regdm_op_class_map
{
	u_int8_t op_class;
	u_int8_t ch_spacing;
	offset_t offset;
	u_int8_t channels[MAX_CHANNELS_PER_OPERATING_CLASS];
} regdm_op_class_map_t;

typedef struct _regdm_supp_op_classes {
	u_int8_t num_classes;
	u_int8_t classes[SIR_MAC_MAX_SUPP_OPER_CLASSES];
} regdm_supp_op_classes;

u_int16_t regdm_get_opclass_from_channel(u_int8_t *country, u_int8_t channel,
	u_int8_t offset);
u_int16_t regdm_set_curr_opclasses(u_int8_t num_classes, u_int8_t *class);
u_int16_t regdm_get_curr_opclasses(u_int8_t *num_classes, u_int8_t *class);

