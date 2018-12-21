/*****************************************************************************
 * Copyright (c) 2009 Broadcom Corporation.  All rights reserved.
 *
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 * 1. This program, including its structure, sequence and organization,
 *    constitutes the valuable trade secrets of Broadcom, and you shall use
 *    all reasonable efforts to protect the confidentiality thereof, and to
 *    use this information only in connection with your use of Broadcom
 *    integrated circuit products.
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
 *    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
 *    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 *    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
 *    IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS
 *    FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS,
 *    QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU
 *    ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS
 *    LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT,
 *    OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY RELATING TO
 *    YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN
 *    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS
 *    OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER
 *    IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF
 *    ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 *****************************************************************************/
#define BSCD_USE_POLLING 1
#undef BSCD_USE_POLLING

#include "bstd.h"
#include "bkni.h"
#include "bchp_sca.h"
#include "bscd.h"
#include "bscd_priv.h"
#include "bscd_isopriv.h"

BDBG_MODULE(BSCD);

#define BSCD_INTERRUPT_DEBUG
//#define SCI_DEBUG 0  /* define this in Makefile */
#undef  BDBG_MSG
#undef  BDBG_ERR
#undef  BDBG_ALERT
/* To enable debug message, use this:*/
#ifndef SCI_DEBUG
#undef BDBG_ENTER
#undef BDBG_LEAVE
#define BDBG_ENTER(...)
#define BDBG_LEAVE(...)
#define BDBG_MSG(...)
#else
#define BDBG_MSG     BKNI_Printf
#endif
#define BDBG_ALERT   BDBG_MSG
#define BDBG_ERR     BDBG_MSG

extern BERR_Code NXP8026_Activate(void);
extern BERR_Code NXP8026_Deactivate(void);
extern int NXP8026_Get_Card_Presence(void);
extern int NXP8026_Get_Card_ATR_Error(void);


/* Population count of 1's in a byte */
static const unsigned char BSCD_P_Popcount[] = {
 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

/*
    FI          0    1   10   11   100   101   110  111  1000  1001  1010  1011  1100  1101  1110  1111
    Fi        372  372  558  744  1116  1488  1860  RFU   RFU   512   768  1024  1536  2048   RFU   RFU
    f(max)  4    5    6    8    12    16    20    -     -     5   7.5    10    15    20     -     -

    DI     0  1  10  11  100  101  110  111  1000  1001  1010  1011  1100  1101  1110  1111
    Di   RFU  1   2   4    8   16   32  RFU    12    20  RFU    RFU   RFU   RFU   RFU   RFU

Rule1:
	(sc_prescale+1) * sc_bauddiv / sc_clkdiv =  F/D   =======> (P2+1) * P3 / P1 =  F/D
Rule2:
	sc_clkdiv * etu_clkdiv  >= f / f(max)                      =======>   P1 * P4 >= 40/f(max)
Rule3:
	sc_clkdiv(P1) = 1,2,3,4,5,8,10,16,32
	bauddiv(P3) = 10,20,25,31,32
	etu_clkdiv(P4) = 1,2,3,4,5,6,7,8
	sc_prescale = 1,2,...,255
*/
static const BSCD_P_DFSmartCardStruct BSCD_P_DF[10][14] = {

#ifdef BSCD_EMV_TEST
   /* There is an issue with old Integri that we have to go with
      lower frequency
   */
    /* D = 0 */
   {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}},

    /* D = 1 */
   {{0x01,0x0B,0x1F,0x08}, {0x02,0x17,0x1F,0x08}, {0x01,0x11,0x1F,0x08},
    {0x01,0x17,0x1F,0x08}, {0x01,0x23,0x1F,0x08}, {0x01,0x2F,0x1F,0x08},
    {0x01,0x3B,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x0F,0x20,0x08}, {0x01,0x17,0x20,0x08}, {0x01,0x1F,0x20,0x08},
    {0x01,0x2F,0x20,0x08}, {0x01,0x3F,0x20,0x08}},

    /* D = 2 */
   {{0x01,0x05,0x1F,0x08}, {0x02,0x0B,0x1F,0x08}, {0x01,0x08,0x1F,0x08},
    {0x01,0x0B,0x1F,0x08}, {0x01,0x11,0x1F,0x08}, {0x01,0x17,0x1F,0x08},
    {0x01,0x1D,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x07,0x20,0x08}, {0x01,0x0B,0x20,0x08}, {0x01,0x0F,0x20,0x08},
    {0x01,0x17,0x20,0x08}, {0x01,0x1F,0x20,0x08}},

    /* D = 3 */
   {{0x01,0x02,0x1F,0x08}, {0x02,0x05,0x1F,0x08}, {0x02,0x08,0x1F,0x08},
    {0x01,0x05,0x1F,0x08}, {0x01,0x08,0x1F,0x08}, {0x01,0x0B,0x1F,0x08},
    {0x01,0x0E,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x03,0x20,0x08}, {0x01,0x05,0x20,0x08}, {0x01,0x07,0x20,0x08},
    {0x01,0x0B,0x20,0x08}, {0x01,0x0F,0x20,0x08}},

    /* D = 4 */
   {{0x02,0x02,0x1F,0x08}, {0x02,0x02,0x1F,0x08}, {0x04,0x08,0x1F,0x04},
    {0x01,0x02,0x1F,0x08}, {0x02,0x08,0x1F,0x08}, {0x01,0x05,0x1F,0x08},
    {0x02,0x0E,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x01,0x20,0x08}, {0x01,0x02,0x20,0x08}, {0x01,0x03,0x20,0x08},
    {0x01,0x05,0x20,0x08}, {0x01,0x07,0x20,0x08}},

    /* D = 5 */
   {{0x04,0x02,0x1F,0x04}, {0x04,0x02,0x1F,0x04}, {0x08,0x08,0x1F,0x02},
    {0x02,0x02,0x1F,0x08}, {0x04,0x08,0x1F,0x04}, {0x01,0x02,0x1F,0x08},
    {0x04,0x0E,0x1F,0x04}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x02,0x01,0x20,0x08}, {0x02,0x02,0x20,0x08}, {0x01,0x01,0x20,0x08},
    {0x01,0x02,0x20,0x08}, {0x01,0x03,0x20,0x08}},

    /* D = 6 */
   {{0x08,0x02,0x1F,0x02}, {0x08,0x02,0x1F,0x02}, {0x10,0x08,0x1F,0x01},
    {0x04,0x02,0x1F,0x04}, {0x08,0x08,0x1F,0x02}, {0x02,0x02,0x1F,0x08},
    {0x08,0x0E,0x1F,0x02}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x04,0x01,0x20,0x04}, {0x04,0x02,0x20,0x04}, {0x02,0x01,0x20,0x08},
    {0x02,0x02,0x20,0x08}, {0x01,0x01,0x20,0x08}},

    /* D = 7 */
   {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}},

    /* D = 8 */
   {{0x02,0x01,0x1F,0x08}, {0x02,0x01,0x1F,0x08}, {0x02,0x02,0x1F,0x08},
    {0x02,0x03,0x1F,0x08}, {0x02,0x05,0x1F,0x08}, {0x02,0x07,0x1F,0x08},
    {0x02,0x09,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x03,0x03,0x20,0x08}, {0x02,0x03,0x20,0x08}, {0x03,0x07,0x20,0x08},
    {0x02,0x07,0x20,0x08}, {0x03,0x0F,0x20,0x08}},

    /* D = 9 */
   {{0x05,0x02,0x1F,0x04}, {0x05,0x02,0x1F,0x04}, {0x0A,0x08,0x1F,0x02},
    {0x05,0x05,0x1F,0x04}, {0x05,0x08,0x1F,0x04}, {0x05,0x0B,0x1F,0x04},
    {0x02,0x05,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x05,0x03,0x20,0x04}, {0x05,0x05,0x20,0x04}, {0x05,0x07,0x20,0x04},
    {0x05,0x0B,0x20,0x04}, {0x05,0x0F,0x20,0x04}}

#elif defined(BSCD_DSS_ICAM)

    /* D = 0 */
   {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}},

    /* D = 1 */
   {{0x01,0x0B,0x1F,0x08}, {0x01,0x0B,0x1F,0x08}, {0x01,0x11,0x1F,0x08},
    {0x01,0x17,0x1F,0x04}, {0x01,0x23,0x1F,0x08}, {0x01,0x2F,0x1F,0x02},
    {0x01,0x3B,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x0F,0x20,0x08}, {0x01,0x17,0x20,0x08}, {0x01,0x1F,0x20,0x08},
    {0x01,0x2F,0x20,0x08}, {0x01,0x3F,0x20,0x08}},

    /* D = 2 */
   {{0x01,0x05,0x1F,0x08}, {0x01,0x05,0x1F,0x08}, {0x01,0x08,0x1F,0x08},
    {0x01,0x0B,0x1F,0x04}, {0x01,0x11,0x1F,0x08}, {0x01,0x17,0x1F,0x02},
    {0x01,0x1D,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x07,0x20,0x08}, {0x01,0x0B,0x20,0x08}, {0x01,0x0F,0x20,0x08},
    {0x01,0x17,0x20,0x08}, {0x01,0x1F,0x20,0x08}},

    /* D = 3 */
   {{0x01,0x02,0x1F,0x08}, {0x01,0x02,0x1F,0x08}, {0x02,0x08,0x1F,0x08},
    {0x01,0x05,0x1F,0x04}, {0x01,0x08,0x1F,0x08}, {0x01,0x0B,0x1F,0x02},
    {0x01,0x0E,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x03,0x20,0x08}, {0x01,0x05,0x20,0x08}, {0x01,0x07,0x20,0x08},
    {0x01,0x0B,0x20,0x08}, {0x01,0x0F,0x20,0x08}},

    /* D = 4 */
   {{0x02,0x02,0x1F,0x04}, {0x02,0x02,0x1F,0x04}, {0x04,0x08,0x1F,0x04},
    {0x01,0x02,0x1F,0x04}, {0x02,0x08,0x1F,0x08}, {0x01,0x05,0x1F,0x02},
    {0x02,0x0E,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x01,0x20,0x08}, {0x01,0x02,0x20,0x08}, {0x01,0x03,0x20,0x08},
    {0x01,0x05,0x20,0x08}, {0x01,0x07,0x20,0x08}},

    /* D = 5 */
   {{0x04,0x02,0x1F,0x02}, {0x04,0x02,0x1F,0x02}, {0x08,0x08,0x1F,0x02},
    {0x02,0x02,0x1F,0x02}, {0x04,0x08,0x1F,0x04}, {0x01,0x02,0x1F,0x02},
    {0x04,0x0E,0x1F,0x04}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x02,0x01,0x20,0x08}, {0x02,0x02,0x20,0x08}, {0x01,0x01,0x20,0x08},
    {0x01,0x02,0x20,0x08}, {0x01,0x03,0x20,0x08}},

    /* D = 6 */
   {{0x08,0x02,0x1F,0x02}, {0x08,0x02,0x1F,0x02}, {0x10,0x08,0x1F,0x01},
    {0x04,0x02,0x1F,0x04}, {0x08,0x08,0x1F,0x02}, {0x02,0x02,0x1F,0x08},
    {0x08,0x0E,0x1F,0x02}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x04,0x01,0x20,0x04}, {0x04,0x02,0x20,0x04}, {0x02,0x01,0x20,0x08},
    {0x02,0x02,0x20,0x08}, {0x01,0x01,0x20,0x08}},

    /* D = 7 */
   {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}},

    /* D = 8 */
   {{0x02,0x01,0x1F,0x08}, {0x02,0x01,0x1F,0x08}, {0x02,0x02,0x1F,0x08},
    {0x02,0x03,0x1F,0x08}, {0x02,0x05,0x1F,0x08}, {0x02,0x07,0x1F,0x08},
    {0x02,0x09,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x03,0x03,0x20,0x08}, {0x02,0x03,0x20,0x08}, {0x03,0x07,0x20,0x08},
    {0x02,0x07,0x20,0x08}, {0x03,0x0F,0x20,0x08}},

    /* D = 9 */
   {{0x05,0x02,0x1F,0x04}, {0x05,0x02,0x1F,0x04}, {0x0A,0x08,0x1F,0x02},
    {0x05,0x05,0x1F,0x04}, {0x05,0x08,0x1F,0x04}, {0x05,0x0B,0x1F,0x04},
    {0x02,0x05,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x05,0x03,0x20,0x04}, {0x05,0x05,0x20,0x04}, {0x05,0x07,0x20,0x04},
    {0x05,0x0B,0x20,0x04}, {0x05,0x0F,0x20,0x04}}

//#elif defined(CONFIG_MACH_IPROC)
// sc_clkdiv, sc_prescale, bauddiv, etu_clkdiv
#elif (BSCD_INTERNAL_CLOCK_FREQ == BSCD_INTERNAL_CLOCK_FREQ_40MHz)

	 /* DI = 0, Reserved for Future Use */
	{{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
	 {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
	 {0,0,0,0}, {0,0,0,0}},

	 /* DI = 1, D=1 */
	{{0x0a,0x77,0x1F,0x01}, {0x0a,0x77,0x1F,0x01}, {0x01,0x11,0x1F,0x08},//F=372, 372, 558
	 {0x01,0x17,0x1F,0x08}, {0x01,0x23,0x1F,0x08}, {0x01,0x2F,0x1F,0x08},//F=744, 1116, 1488
	 {0x01,0x3B,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x02,0x1F,0x20,0x08}, {0x01,0x17,0x20,0x08}, {0x01,0x1F,0x20,0x08},//F=512, 768, 1024
	 {0x01,0x2F,0x20,0x08}, {0x01,0x3F,0x20,0x08}},//F=1536, 2048

	 /* DI = 2, D=2 */
	{{0x02,0x0B,0x1F,0x08}, {0x02,0x0B,0x1F,0x08}, {0x01,0x08,0x1F,0x08},//F=372, 372, 558
	 {0x01,0x0B,0x1F,0x08}, {0x01,0x11,0x1F,0x08}, {0x01,0x17,0x1F,0x08},//F=744, 1116, 1488
	 {0x01,0x1D,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x02,0x0F,0x20,0x08}, {0x01,0x0B,0x20,0x08}, {0x01,0x0F,0x20,0x08},//F=512, 768, 1024
	 {0x01,0x17,0x20,0x08}, {0x01,0x1F,0x20,0x08}},//F=1536, 2048

	 /* DI = 3, D=4 */
	{{0x02,0x05,0x1F,0x08}, {0x02,0x05,0x1F,0x08}, {0x02,0x08,0x1F,0x05},//F=372, 372, 558
	 {0x01,0x05,0x1F,0x08}, {0x01,0x08,0x1F,0x08}, {0x01,0x0B,0x1F,0x08},//F=744, 1116, 1488
	 {0x01,0x0E,0x1F,0x08}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x02,0x07,0x20,0x08}, {0x01,0x05,0x20,0x08}, {0x01,0x07,0x20,0x08},//F=512, 768, 1024
	 {0x01,0x0B,0x20,0x08}, {0x01,0x0F,0x20,0x08}},//F=1536, 2048

	 /* DI = 4, D=8 */
	{{0x02,0x02,0x1F,0x06}, {0x02,0x02,0x1F,0x05}, {0x04,0x08,0x1F,0x02},//F=372, 372, 558
	 {0x01,0x02,0x1F,0x08}, {0x02,0x08,0x1F,0x05}, {0x01,0x05,0x1F,0x08},//F=744, 1116, 1488
	 {0x02,0x0E,0x1F,0x05}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x02,0x03,0x20,0x08}, {0x01,0x02,0x20,0x08}, {0x01,0x03,0x20,0x08},//F=512, 768, 1024
	 {0x01,0x05,0x20,0x08}, {0x01,0x07,0x20,0x08}},//F=1536, 2048

	 /* DI = 5, D=16 */
	{{0x04,0x02,0x1F,0x04}, {0x04,0x02,0x1F,0x04}, {0x08,0x08,0x1F,0x01},//F=372, 372, 558
	 {0x02,0x02,0x1F,0x04}, {0x04,0x08,0x1F,0x02}, {0x01,0x02,0x1F,0x08},//F=744, 1116, 1488
	 {0x04,0x0E,0x1F,0x02}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x02,0x01,0x20,0x05}, {0x02,0x02,0x20,0x04}, {0x01,0x01,0x20,0x08},//F=512, 768, 1024
	 {0x01,0x02,0x20,0x08}, {0x01,0x03,0x20,0x08}},//F=1536, 2048

	 /* DI = 6, D=32 */
	{{0x08,0x02,0x1F,0x02}, {0x08,0x02,0x1F,0x02}, {0x10,0x08,0x1F,0x01},//F=372, 372, 558
	 {0x04,0x02,0x1F,0x02}, {0x08,0x08,0x1F,0x01}, {0x02,0x02,0x1F,0x03},//F=744, 1116, 1488
	 {0x08,0x0E,0x1F,0x01}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x04,0x01,0x20,0x03}, {0x04,0x02,0x20,0x02}, {0x02,0x01,0x20,0x03},//F=512, 768, 1024
	 {0x02,0x02,0x20,0x03}, {0x01,0x01,0x20,0x06}},//F=1536, 2048

	 /* DI = 7, RFU */
	{{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
	 {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
	 {0,0,0,0}, {0,0,0,0}},

	 /* DI = 8, D=12 */
	{{0x02,0x01,0x1F,0x06}, {0x02,0x01,0x1F,0x06}, {0x02,0x02,0x1F,0x04},//F=372, 372, 558
	 {0x02,0x03,0x1F,0x04}, {0x02,0x05,0x1F,0x04}, {0x02,0x07,0x1F,0x04},//F=744, 1116, 1488
	 {0x02,0x09,0x1F,0x04}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x03,0x03,0x20,0x03}, {0x02,0x03,0x20,0x04}, {0x03,0x07,0x20,0x03},//F=512, 768, 1024
	 {0x02,0x07,0x20,0x04}, {0x03,0x0F,0x20,0x03}},//F=1536, 2048

	 /* DI = 9, D=20 */
	{{0x05,0x02,0x1F,0x03}, {0x05,0x02,0x1F,0x03}, {0x0A,0x08,0x1F,0x01},//F=372, 372, 558
	 {0x05,0x05,0x1F,0x02}, {0x05,0x08,0x1F,0x02}, {0x05,0x0B,0x1F,0x02},//F=744, 1116, 1488
	 {0x02,0x05,0x1F,0x04}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},//F=1860, -, -
	 {0x05,0x03,0x20,0x02}, {0x05,0x05,0x20,0x02}, {0x05,0x07,0x20,0x02},//F=512, 768, 1024
	 {0x05,0x0B,0x20,0x02}, {0x05,0x0F,0x20,0x02}}//F=1536, 2048

#elif (BSCD_INTERNAL_CLOCK_FREQ == BSCD_INTERNAL_CLOCK_FREQ_27MHz)

    /* DI = 0, Reserved for Future Use */
   {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}},

    /* DI = 1, D=1 */
   {{0x01,0x0B,0x1F,0x06}, {0x01,0x0B,0x1F,0x06}, {0x01,0x11,0x1F,0x06},
    {0x01,0x17,0x1F,0x06}, {0x01,0x23,0x1F,0x06}, {0x01,0x2F,0x1F,0x06},
    {0x01,0x3B,0x1F,0x06}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x0F,0x20,0x06}, {0x01,0x17,0x20,0x06}, {0x01,0x1F,0x20,0x06},
    {0x01,0x2F,0x20,0x06}, {0x01,0x3F,0x20,0x06}},

    /* DI = 2, D=2 */
   {{0x01,0x05,0x1F,0x06}, {0x01,0x05,0x1F,0x06}, {0x01,0x08,0x1F,0x06},
    {0x01,0x0B,0x1F,0x06}, {0x01,0x11,0x1F,0x06}, {0x01,0x17,0x1F,0x06},
    {0x01,0x1D,0x1F,0x06}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x07,0x20,0x06}, {0x01,0x0B,0x20,0x06}, {0x01,0x0F,0x20,0x06},
    {0x01,0x17,0x20,0x06}, {0x01,0x1F,0x20,0x06}},

    /* DI = 3, D=4 */
   {{0x01,0x02,0x1F,0x06}, {0x01,0x02,0x1F,0x06}, {0x02,0x08,0x1F,0x03},
    {0x01,0x05,0x1F,0x06}, {0x01,0x08,0x1F,0x06}, {0x01,0x0B,0x1F,0x06},
    {0x01,0x0E,0x1F,0x06}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x03,0x20,0x06}, {0x01,0x05,0x20,0x06}, {0x01,0x07,0x20,0x06},
    {0x01,0x0B,0x20,0x06}, {0x01,0x0F,0x20,0x06}},

    /* DI = 4, D=8 */
   {{0x02,0x02,0x1F,0x03}, {0x02,0x02,0x1F,0x03}, {0x04,0x08,0x1F,0x02},
    {0x01,0x02,0x1F,0x06}, {0x02,0x08,0x1F,0x03}, {0x01,0x05,0x1F,0x06},
    {0x02,0x0E,0x1F,0x03}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x01,0x01,0x20,0x06}, {0x01,0x02,0x20,0x06}, {0x01,0x03,0x20,0x06},
    {0x01,0x05,0x20,0x06}, {0x01,0x07,0x20,0x06}},

    /* DI = 5, D=16 */
   {{0x04,0x02,0x1F,0x02}, {0x04,0x02,0x1F,0x02}, {0x08,0x08,0x1F,0x01},
    {0x02,0x02,0x1F,0x03}, {0x04,0x08,0x1F,0x02}, {0x01,0x02,0x1F,0x06},
    {0x04,0x0E,0x1F,0x02}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x02,0x01,0x20,0x03}, {0x02,0x02,0x20,0x03}, {0x01,0x01,0x20,0x06},
    {0x01,0x02,0x20,0x06}, {0x01,0x03,0x20,0x06}},

    /* DI = 6, D=32 */
   {{0x08,0x02,0x1F,0x01}, {0x08,0x02,0x1F,0x01}, {0x10,0x08,0x1F,0x01},
    {0x04,0x02,0x1F,0x01}, {0x08,0x08,0x1F,0x01}, {0x02,0x02,0x1F,0x03},
    {0x08,0x0E,0x1F,0x01}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x04,0x01,0x20,0x02}, {0x04,0x02,0x20,0x02}, {0x02,0x01,0x20,0x03},
    {0x02,0x02,0x20,0x03}, {0x01,0x01,0x20,0x06}},

    /* DI = 7, RFU */
   {{0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0},
    {0,0,0,0}, {0,0,0,0}},

    /* DI = 8, D=12 */
   {{0x02,0x01,0x1F,0x03}, {0x02,0x01,0x1F,0x03}, {0x02,0x02,0x1F,0x03},
    {0x02,0x03,0x1F,0x03}, {0x02,0x05,0x1F,0x03}, {0x02,0x07,0x1F,0x03},
    {0x02,0x09,0x1F,0x03}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x03,0x03,0x20,0x02}, {0x02,0x03,0x20,0x03}, {0x03,0x07,0x20,0x02},
    {0x02,0x07,0x20,0x03}, {0x03,0x0F,0x20,0x02}},

    /* DI = 9, D=20 */
   {{0x05,0x02,0x1F,0x02}, {0x05,0x02,0x1F,0x02}, {0x0A,0x08,0x1F,0x01},
    {0x05,0x05,0x1F,0x02}, {0x05,0x08,0x1F,0x02}, {0x05,0x0B,0x1F,0x02},
    {0x02,0x05,0x1F,0x03}, {0x00,0x00,0x00,0x00}, {0x00,0x00,0x00,0x00},
    {0x05,0x03,0x20,0x02}, {0x05,0x05,0x20,0x02}, {0x05,0x07,0x20,0x02},
    {0x05,0x0B,0x20,0x02}, {0x05,0x0F,0x20,0x02}}
#else

error "SCI input clock unknow!"

#endif
};


void BSCD_Channel_P_ReadStatus(
        void *inp_param1,               /* Device channel handle */
        int in_param2                   /* reserved */
);


static const int BSCD_P_aunFFactor[14] = {372, 372, 558, 744, 1116, 1488, 1860,
                                 -1,  -1, 512, 768, 1024, 1536, 2048};


static const signed char BSCD_P_aucDFactor[10] = {-1, 1, 2, 4, 8, 16, 32, -1, 12, 20};

//The value of SCA_SC_CLK_CMD_1 of sc_clk_div
#ifdef CYGNUS_SCI_DRV
static const unsigned char BSCD_P_aucSCClkDiv[33] = {0x00, 0x00, 0x10, 0x20,  0x30, 0x40, 0x00, 0x00,
                                                     0x50, 0x00, 0x60, 0x00,  0x00, 0x00, 0x00, 0x00, 
                                                     0x70, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 
                                                     0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 
                                                     0x00};
#else
static const unsigned char BSCD_P_aucSCClkDiv[16] = {0x00, 0x10, 0x20, 0x30, 0x40, 0x00, 0x00, 0x50,
                                                     0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70};
#endif

unsigned char BSCD_P_GetClkDiv(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
)
{
   return BSCD_P_DF[in_ucDFactor][in_ucFFactor].sc_clkdiv;
}


unsigned char BSCD_P_GetETUClkDiv(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
)
{
   return BSCD_P_DF[in_ucDFactor][in_ucFFactor].sc_etuclkdiv;
}

unsigned char BSCD_P_GetISOBaudRateAdjustor(
      unsigned char in_ucDFactor
)
{
   return  BSCD_P_aucDFactor[in_ucDFactor];
}

unsigned int BSCD_P_GetISOClockRateConversionFactor(
      unsigned char in_ucFFactor
)
{
   return  BSCD_P_aunFFactor[in_ucFFactor];
}

unsigned char BSCD_P_MapScClkDivToMaskValue(
      unsigned char in_ucClkDiv
)
{
#ifdef CYGNUS_SCI_DRV
	/*
	 0000 - SC_CLK divider = 1.
	 0001 - SC_CLK divider = 2.
	 0010 - SC_CLK divider = 3.
	 0011 - SC_CLK divider = 4.
	 0100 - SC_CLK divider = 5.
	 0101 - SC_CLK divider = 8.
	 0110 - SC_CLK divider = 10.
	 0111 - SC_CLK divider = 16.
	 1000 - SC_CLK divider = 32.
	 */
	if(in_ucClkDiv==1 || in_ucClkDiv==2  || in_ucClkDiv==3  || in_ucClkDiv==4  || 
	   in_ucClkDiv==5 || in_ucClkDiv==8  || in_ucClkDiv==10 || in_ucClkDiv==16 || in_ucClkDiv==32
	)
		return	BSCD_P_aucSCClkDiv[in_ucClkDiv];
	else
	{
		BDBG_ERR("Invalid SC_CLK divider = %d\n", in_ucClkDiv);
		return 0;
	}
#else
	return	BSCD_P_aucSCClkDiv[in_ucClkDiv-1];
#endif
}

unsigned char BSCD_P_GetPrescale(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
)
{
   return BSCD_P_DF[in_ucDFactor][in_ucFFactor].sc_prescale;
}

unsigned char BSCD_P_GetBaudDiv(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
)
{
   return BSCD_P_DF[in_ucDFactor][in_ucFFactor].sc_bauddiv;
}


/* BSYT???: Change this function name to Adjust WWT */
BERR_Code BSCD_P_AdjustWWT(
		BSCD_ChannelSettings	*pChannelSettings,
		unsigned char       	 in_ucWorkWaitTImeInteger
)
{
	BERR_Code errCode = BERR_SUCCESS;
	unsigned char         ucBaudRateAdjustor;

	//BDBG_ENTER((BSCD_P_AdjustWWT));
	BDBG_ENTER(("BSCD_P_AdjustWWT\n"));

	BDBG_MSG("ucDFactor = %d\n", pChannelSettings->ucDFactor);

	BDBG_MSG("baudrate = %lu\n", pChannelSettings->currentBaudRate);

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(pChannelSettings->currentBaudRate == 0) );

	BDBG_MSG ("etu in us= %d\n",
	   	1000000/pChannelSettings->currentBaudRate);

	ucBaudRateAdjustor = BSCD_P_GetISOBaudRateAdjustor(pChannelSettings->ucDFactor);

/*EMV2000*/
	if (pChannelSettings->scStandard == BSCD_Standard_eEMV2000)
		pChannelSettings->workWaitTime.ulValue =
			BSCD_ISO_WORK_WAIT_TIME_DEFAULT_FACTOR * ucBaudRateAdjustor *
			in_ucWorkWaitTImeInteger + ucBaudRateAdjustor *
			BSCD_DEFAULT_EXTRA_WORK_WAITING_TIME_EMV2000 +
			BSCD_EMV2000_WORK_WAIT_TIME_DELTA;
	else
		pChannelSettings->workWaitTime.ulValue =
			BSCD_ISO_WORK_WAIT_TIME_DEFAULT_FACTOR * ucBaudRateAdjustor *
			in_ucWorkWaitTImeInteger;

	pChannelSettings->workWaitTime.unit = BSCD_TimerUnit_eETU;


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_P_AdjustWWT);
	return( errCode );
}


/* This modify registers */
BERR_Code BSCD_P_FDAdjust(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char 		in_ucFFactor,
		unsigned char		in_ucDFactor
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulClkCmd;

	BDBG_ENTER(("BSCD_P_FDAdjust\n"));

	/* Set BCM to adjust the clock and bit rate  */
	in_channelHandle->currentChannelSettings.unPrescale =
				BSCD_P_GetPrescale(in_ucDFactor, in_ucFFactor) *
				in_channelHandle->currentChannelSettings.ucExternalClockDivisor +
			(in_channelHandle->currentChannelSettings.ucExternalClockDivisor - 1);

   	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PRESCALE), in_channelHandle->currentChannelSettings.unPrescale);
	BDBG_ENTER(("SC_Prescale = 0x%x\n", in_channelHandle->currentChannelSettings.unPrescale));

	in_channelHandle->currentChannelSettings.ucBaudDiv = BSCD_P_GetBaudDiv(in_ucDFactor, in_ucFFactor);


	BDBG_ENTER(("sc_bauddiv = 0x%2x\n", in_channelHandle->currentChannelSettings.ucBaudDiv));

	in_channelHandle->currentChannelSettings.ucScClkDiv =
		BSCD_P_GetClkDiv(in_ucDFactor, in_ucFFactor) ;

	if (in_channelHandle->currentChannelSettings.ucBaudDiv == BSCD_DEFAULT_BAUD_DIV) {
		ulClkCmd = BCHP_SCA_SC_CLK_CMD_clk_en_MASK |
				BSCD_P_MapScClkDivToMaskValue(in_channelHandle->currentChannelSettings.ucScClkDiv ) |
				((in_channelHandle->currentChannelSettings.ucEtuClkDiv - 1) << 1)  ;
	}
	else {
		ulClkCmd = BCHP_SCA_SC_CLK_CMD_clk_en_MASK |
				BSCD_P_MapScClkDivToMaskValue(in_channelHandle->currentChannelSettings.ucScClkDiv ) |
				((in_channelHandle->currentChannelSettings.ucEtuClkDiv - 1) << 1)  |
				BCHP_SCA_SC_CLK_CMD_bauddiv_MASK;
	}

   	BKNI_RegWrite8(
		in_channelHandle,
		(in_channelHandle->ulRegStartAddr + BSCD_P_CLK_CMD),
		ulClkCmd);
	BDBG_ENTER(("New SC_CLK_CMD = 0x%x\n", ulClkCmd));

	BDBG_LEAVE(BSCD_P_FDAdjust);
	return( errCode );

}

/* This does NOT modify registers */
BERR_Code BSCD_P_FDAdjust_WithoutRegisterUpdate(
		BSCD_ChannelSettings     *pChannelSettings,
		unsigned char            in_ucFFactor,
		unsigned char            in_ucDFactor
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_P_FDAdjust_WithoutRegisterUpdate\n"));

	/* Set BCM to adjust the clock and bit rate  */

	if ((in_ucDFactor >= 1 ) && (in_ucDFactor <= 9)) {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(BSCD_P_GetISOBaudRateAdjustor(in_ucDFactor) ==  ((unsigned char ) -1) ) );
		pChannelSettings->ucDFactor   = in_ucDFactor;
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}

	if ((in_ucFFactor >= 1 ) && (in_ucFFactor <= 13)){
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(BSCD_P_GetISOClockRateConversionFactor(in_ucFFactor) ==  ((unsigned char ) -1) ) );
		pChannelSettings->ucFFactor   = in_ucFFactor;
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}

	pChannelSettings->ucEtuClkDiv = BSCD_P_GetETUClkDiv(in_ucDFactor, in_ucFFactor);

	pChannelSettings->ucScClkDiv  = BSCD_P_GetClkDiv(in_ucDFactor, in_ucFFactor) ;

	pChannelSettings->unPrescale  = BSCD_P_GetPrescale(in_ucDFactor, in_ucFFactor) *	pChannelSettings->ucExternalClockDivisor +
								(pChannelSettings->ucExternalClockDivisor - 1);

	pChannelSettings->ucBaudDiv   = BSCD_P_GetBaudDiv(in_ucDFactor, in_ucFFactor);


	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                BSCD_P_AdjustWWT(pChannelSettings, BSCD_ISO_DEFAULT_WORK_WAIT_TIME_INTEGER));

BSCD_P_DONE_LABEL:
	BDBG_LEAVE(BSCD_P_FDAdjust_WithoutRegisterUpdate);
	return( errCode );

}


/* Default ISR Callback Functions */
void BSCD_Channel_P_CardInsertCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_CardInsertCB_isr\n"));
	BDBG_MSG("default  BSCD_Channel_P_CardInsertCB_isr \n");
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.cardWait);
	}
}

void  BSCD_Channel_P_CardRemoveCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_CardRemoveCB_isr\n"));
	BDBG_MSG("default  BSCD_Channel_P_CardRemoveCB_isr \n");
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.cardWait);
		/* 09/20/05,	Allen.C,  set rcv event and let BKNI_WaitForEvent() exit after card is removed*/
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.tdoneWait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.timerWait);
	}
}


void BSCD_Channel_P_RcvCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_RcvCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}

void BSCD_Channel_P_ATRCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_ATRCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.atrStart);
	}
}

void BSCD_Channel_P_WaitCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_WaitCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.tdoneWait);
	}

}

void BSCD_Channel_P_RetryCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_RetryCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.tdoneWait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}

void BSCD_Channel_P_TimerCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_TimerCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.atrStart);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.timerWait);
	}
	BDBG_LEAVE(BSCD_Channel_P_TimerCB_isr);
 }

void BSCD_Channel_P_RParityCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_RParityCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}

void BSCD_Channel_P_TParityCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_TParityCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.tdoneWait);
	}
}

void BSCD_Channel_P_CWTCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_CWTCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}

void BSCD_Channel_P_BGTCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_BGTCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.tdoneWait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}

void BSCD_Channel_P_RLenCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_RLenCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}

void BSCD_Channel_P_RReadyCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_RReadyCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}

void BSCD_Channel_P_TDoneCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_TDoneCB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.tdoneWait);
	}
}

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
void BSCD_Channel_P_Event1CB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_Event1CB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.event1Wait);
	}
}
#endif

void BSCD_Channel_P_Event2CB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void                      *inp_data
)
{
	BDBG_ENTER(("BSCD_Channel_P_Event2CB_isr\n"));
	BSTD_UNUSED(inp_data);
	if (in_channelHandle->bIsOpen == true) {
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.event2Wait);
		BKNI_SetEvent( in_channelHandle->channelWaitEvent.rcvWait);
	}
}



BERR_Code BSCD_Channel_P_WaitForCardInsertion(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulStatus1;

	BDBG_ENTER(("BSCD_Channel_P_WaitForCardInsertion\n"));

	BDBG_MSG("Ready to receive card insertion pres_intr interrupt\n");


	BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
	BSCD_Channel_P_ReadStatus(in_channelHandle, 0);
#endif
	ulStatus1 = in_channelHandle->ulStatus1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK;
	BKNI_LeaveCriticalSection();

	do {

		if ( ulStatus1 != BCHP_SCA_SC_STATUS_1_card_pres_MASK) {

			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BKNI_WaitForEvent(
#ifdef BSCD_USE_POLLING
                                                  in_channelHandle,
#endif
                                                  in_channelHandle->channelWaitEvent.cardWait, BKNI_INFINITE));

		}

		BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
		BSCD_Channel_P_ReadStatus(in_channelHandle, 0);
#endif
		ulStatus1 = in_channelHandle->ulStatus1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK;
		BKNI_LeaveCriticalSection();

	} while  (ulStatus1 != BCHP_SCA_SC_STATUS_1_card_pres_MASK);

	BDBG_MSG("Received card insertion pres_intr interrupt\n");

	if ( ulStatus1 == BCHP_SCA_SC_STATUS_1_card_pres_MASK) {
		in_channelHandle->channelStatus.bCardPresent = true;
		in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
		BDBG_MSG("Smart Card Inserted\n");
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_WaitForCardInsertion);
	return errCode;
}

BERR_Code BSCD_Channel_P_WaitForCardRemove(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulStatus1;

	BDBG_ENTER(("BSCD_Channel_P_WaitForCardRemove\n"));

	BDBG_MSG("Ready to receive card removal pres_intr interrupt\n");


	BKNI_EnterCriticalSection();
	ulStatus1 = in_channelHandle->ulStatus1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK;
	BKNI_LeaveCriticalSection();

	do {

		if (ulStatus1 == BCHP_SCA_SC_STATUS_1_card_pres_MASK) {

		      BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			     BKNI_WaitForEvent(
#ifdef BSCD_USE_POLLING
                                               in_channelHandle,
#endif
                                               in_channelHandle->channelWaitEvent.cardWait, BKNI_INFINITE));

		}

		BKNI_EnterCriticalSection();
		ulStatus1 = in_channelHandle->ulStatus1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK;
		BKNI_LeaveCriticalSection();

	} while  (ulStatus1 == BCHP_SCA_SC_STATUS_1_card_pres_MASK);

	BDBG_MSG("Received card removal pres_intr interrupt\n");

	if ( ulStatus1 != BCHP_SCA_SC_STATUS_1_card_pres_MASK) {
		in_channelHandle->channelStatus.bCardPresent = false;
		in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
		BDBG_MSG("Smart Card Removed\n");
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_WaitForCardRemove);
	return errCode;
}

#ifdef BSCD_USE_POLLING
void BSCD_Channel_P_ReadStatus(
	void *inp_param1,		/* Device channel handle */
	int in_param2			/* reserved */
)
{
	uint32_t    unIntrEn1 = 0, unIntrEn2 = 0;
	BSCD_ChannelHandle 	channelHandle = (BSCD_ChannelHandle) inp_param1;

	/* Read Smartcard Interrupt Status & Mask Register */
	unIntrEn1 = BKNI_RegRead8(
		channelHandle,
		(channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1));

	channelHandle->ulIntrStatus1 = unIntrEn1 & BKNI_RegRead8(
		channelHandle,
		(channelHandle->ulRegStartAddr + BSCD_P_INTR_STAT_1));

	unIntrEn2 = BKNI_RegRead8(
		channelHandle,
		(channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_2));

	channelHandle->ulIntrStatus2  = unIntrEn2 & BKNI_RegRead8(
		channelHandle,
		(channelHandle->ulRegStartAddr + BSCD_P_INTR_STAT_2));

	return;
}
#endif


BERR_Code BSCD_Channel_P_WaitForTimerEvent(
		BSCD_ChannelHandle	in_channelHandle,
		uint32_t            in_nCheckCardRemoval
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulIntrStatus1;

	BDBG_ENTER(("BSCD_Channel_P_WaitForTimerEvent\n"));

	BDBG_MSG("Ready to receive scard_timer_wait interrupt\n");
	BDBG_MSG("SCA_SC_TIMER_CMD = 0x%x\n", BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD)));
	BDBG_MSG("SCA_SC_INTR_EN_1 = 0x%x\n", BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1)));

	BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
	BSCD_Channel_P_ReadStatus(in_channelHandle, 0);
#endif
	ulIntrStatus1 = in_channelHandle->ulIntrStatus1;
	BKNI_LeaveCriticalSection();

	BDBG_MSG("ulIntrStatus1 = 0x%x\n", ulIntrStatus1);

	do {
		/*09/20/05,Allen.C, check if the Card is removed  */
		if ( in_nCheckCardRemoval && (in_channelHandle-> bIsCardRemoved == true) &&
			(( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK ) == BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK)) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RESET_CHANNEL_REQUIRED;
			in_channelHandle-> bIsCardRemoved = false ;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceTimerEvent: SC_CARD_REMOVED error \n");
				errCode = BSCD_STATUS_FAILED ;
				goto BSCD_P_DONE_LABEL;
		}
		else if (( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK) != BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK) {
			BDBG_MSG("No timer intr, wait %d ms\n", in_channelHandle->currentChannelSettings.timeOut.ulValue);
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BKNI_WaitForEvent(
#ifdef BSCD_USE_POLLING
                                                  in_channelHandle,
#endif
                                                  in_channelHandle->channelWaitEvent.timerWait,
						  in_channelHandle->currentChannelSettings.timeOut.ulValue));
		}
	
		BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
		BSCD_Channel_P_ReadStatus(in_channelHandle, 0);
#endif
		ulIntrStatus1 = in_channelHandle->ulIntrStatus1;
		BKNI_LeaveCriticalSection();
		
		BDBG_MSG("Re-read SCA_SC_INTR_STAT_1=0x%x\n", ulIntrStatus1);
	} while  ((ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK )!=
					BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK);

	BKNI_EnterCriticalSection();
	in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK;
	BKNI_LeaveCriticalSection();

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_WaitForTimerEvent);

	return errCode;
}

BERR_Code BSCD_Channel_P_WaitForATRStart(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulIntrStatus1, ulIntrStatus2;

	BDBG_ENTER(("BSCD_Channel_P_WaitForATRStart\n"));

	BDBG_MSG("Ready to receive scard_atrStart interrupt, ucSlot = %d\n", in_channelHandle->ucChannelNumber);

	BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
	BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
	ulIntrStatus1 = in_channelHandle->ulIntrStatus1;
	ulIntrStatus2 = in_channelHandle->ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK;
	BKNI_LeaveCriticalSection();

	BDBG_MSG("ulIntrStatus1 = 0x%x, ulIntrStatus2 = 0x%x\n", ulIntrStatus1, ulIntrStatus2);
	BDBG_MSG("in_channelHandle->ulIntrStatus1 = 0x%x\n", in_channelHandle->ulIntrStatus1);

	do {
		if ( (( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK) == BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK) &&
		      (ulIntrStatus2 == BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK) ) {
		         goto BSCD_P_SUCCESS_LABEL;
		}
		else if (( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK)  == BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK) {
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK;
			 BDBG_ERR("ScardDeviceWaitForATRStart: SC_TIMER_INTR error \n");
			 errCode = BSCD_STATUS_TIME_OUT;
			 goto BSCD_P_DONE_LABEL;
		}
		else
		/*04/11/06,Allen.C, check if the Card is removed  */
		if ( (in_channelHandle-> bIsCardRemoved == true) &&
			(( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK ) == BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK)) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RESET_CHANNEL_REQUIRED;
			in_channelHandle-> bIsCardRemoved = false ;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForTDone: SC_CARD_REMOVED error \n");
	 		errCode = BSCD_STATUS_FAILED ;
		         goto BSCD_P_DONE_LABEL;
		}
		else if (ulIntrStatus2 != BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK) {


			if ( (errCode = BERR_TRACE(BKNI_WaitForEvent(
#ifdef BSCD_USE_POLLING
                                                                     in_channelHandle,
#endif
                                                                     in_channelHandle->channelWaitEvent.atrStart,
						in_channelHandle->currentChannelSettings.timeOut.ulValue))) != BERR_SUCCESS ) {
				in_channelHandle->channelStatus.ulStatus1 |= BSCD_RX_TIMEOUT;
				errCode = BSCD_STATUS_TIME_OUT;
				goto BSCD_P_DONE_LABEL;
			}
		}

		BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
		BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
		ulIntrStatus1 = in_channelHandle->ulIntrStatus1;
		ulIntrStatus2 = in_channelHandle->ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK;
		BKNI_LeaveCriticalSection();

	}while  (ulIntrStatus2 != BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK);

BSCD_P_SUCCESS_LABEL:

	BKNI_EnterCriticalSection();
	in_channelHandle->ulIntrStatus2  &= ~BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK;
	BKNI_LeaveCriticalSection();
	BDBG_MSG("scard_atrStart interrupt received\n");

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_WaitForATRStart);
	return errCode;
}


BERR_Code BSCD_Channel_P_WaitForTDone(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulIntrStatus1, ulIntrStatus2;

	BDBG_ENTER(("BSCD_Channel_P_WaitForTDone\n"));

	BDBG_MSG("Ready to receive scard_tDone interrupt, ucSlot = %d\n", in_channelHandle->ucChannelNumber);

	BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
	BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
	ulIntrStatus1 = in_channelHandle->ulIntrStatus1 ;
	ulIntrStatus2 = in_channelHandle->ulIntrStatus2;
	BKNI_LeaveCriticalSection();

	do {
 		if ( (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) &&
         				((ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK) ==
         					BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK) ) {
			BKNI_EnterCriticalSection();
        		 in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK;
			BKNI_LeaveCriticalSection();
         		BDBG_ERR("ScardDeviceWaitForTDone: SC_BGT_INTR error \n");
			 errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
      		}

		else if ( (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0) &&
				(( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK) ==
				BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK) ) {

			BKNI_EnterCriticalSection();
        		in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForTDone: RETRY_INTR error \n");

			in_channelHandle->channelStatus.ulStatus1 |= BSCD_TX_PARITY;
			 errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
		else if (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK;
			BKNI_LeaveCriticalSection();
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_TX_TIMEOUT;
			BDBG_ERR("ScardDeviceWaitForTDone: SC_WAIT_INTR error \n");
			 errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
		/*09/20/05,Allen.C, check if the Card is removed  */
		else if ( (in_channelHandle-> bIsCardRemoved == true) &&
			(( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK ) == BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK)) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RESET_CHANNEL_REQUIRED;
			in_channelHandle-> bIsCardRemoved = false ;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForTDone: SC_CARD_REMOVED error \n");
	 		errCode = BSCD_STATUS_FAILED ;
		         goto BSCD_P_DONE_LABEL;
		}
		else if (( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK) != BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK) {
			if ( (errCode = BERR_TRACE(BKNI_WaitForEvent(
#ifdef BSCD_USE_POLLING
                                                                     in_channelHandle,
#endif
                                                                     in_channelHandle->channelWaitEvent.tdoneWait,
						in_channelHandle->currentChannelSettings.timeOut.ulValue))) != BERR_SUCCESS ) {
				BKNI_EnterCriticalSection();
				in_channelHandle->channelStatus.ulStatus1 |= BSCD_TX_TIMEOUT;
				BKNI_LeaveCriticalSection();
				errCode = BSCD_STATUS_TIME_OUT;
				goto BSCD_P_DONE_LABEL;
			}
		}

		BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
		BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
		ulIntrStatus1 = in_channelHandle->ulIntrStatus1;
		ulIntrStatus2 = in_channelHandle->ulIntrStatus2;
		BKNI_LeaveCriticalSection();
	}while  ((ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK )!=
					BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK);

	BKNI_EnterCriticalSection();
	in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK;
	in_channelHandle->ulStatus1 &= ~BCHP_SCA_SC_STATUS_1_tdone_MASK;
	BKNI_LeaveCriticalSection();
	BDBG_MSG("tdone_intr interrupt received\n");


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_WaitForTDone);
	return errCode;
}


BERR_Code BSCD_Channel_P_WaitForRcv(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulIntrStatus1, ulIntrStatus2, ulStatus2;

	BDBG_ENTER(("BSCD_Channel_P_WaitForRcv\n"));

	/* BDBG_MSG("Ready to receive rcv interrupt\n");  */

	BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
	BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
	ulIntrStatus1 = in_channelHandle->ulIntrStatus1 ;
	ulIntrStatus2 = in_channelHandle->ulIntrStatus2;
	ulStatus2 =  in_channelHandle->ulStatus2;
	BKNI_LeaveCriticalSection();

	do {


		if ((ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK) ==  BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK ) {
			BKNI_EnterCriticalSection();
     			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			/* This could be a good error if the caller specify a length larger than that of  the actual one. */
         	BDBG_MSG("ScardDeviceWaitForRcv: SC_TIMER_INTR error \n");
			errCode = BSCD_STATUS_TIME_OUT;
		        goto BSCD_P_DONE_LABEL;
      	}

		/* BSYT???:  Obsolete ??? */
		else if ( (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) &&
			      ((ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK) == BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK) ) {
			BKNI_EnterCriticalSection();
     			in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_BGT_INTR error \n");
			errCode = BSCD_STATUS_READ_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
		else if ( ( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK)  {

			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK;
			in_channelHandle->channelStatus.ulStatus2  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_EVENT2_INTR error \n");
			errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
		else if ((in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0 ) &&
			(in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eIrdeto) &&
			   (( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK) == BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK)) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_RETRY_INTR error \n");
			in_channelHandle->channelStatus.ulStatus1 |= BSCD_RX_PARITY;
			errCode = BSCD_STATUS_PARITY_EDC_ERR;
			goto BSCD_P_DONE_LABEL;
		}

		else if ( (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) &&
			((ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK) ) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_RLEN_INTR error \n");
			errCode = BSCD_STATUS_READ_FAILED;
			goto BSCD_P_DONE_LABEL;
		}
		else if (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_WAIT_INTR error \n");
	 		errCode = BSCD_STATUS_TIME_OUT;
	        goto BSCD_P_DONE_LABEL;
		}
		else if (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_CWT_INTR error \n");
	 		errCode = BSCD_STATUS_TIME_OUT;
		         goto BSCD_P_DONE_LABEL;
		}
		else if (( ulStatus2 & BCHP_SCA_SC_STATUS_2_roverflow_MASK) == BCHP_SCA_SC_STATUS_2_roverflow_MASK) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulStatus2 &= ~BCHP_SCA_SC_STATUS_2_roverflow_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_ROVERFLOW error \n");
			errCode = BSCD_STATUS_READ_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
		/*09/20/05,Allen.C, check if the Card is removed  */
		else if ( (in_channelHandle-> bIsCardRemoved == true) &&
			(( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK ) == BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK)) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RESET_CHANNEL_REQUIRED;
			in_channelHandle-> bIsCardRemoved = false ;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRcv: SC_CARD_REMOVED error \n");
	 		errCode = BSCD_STATUS_FAILED ;
		         goto BSCD_P_DONE_LABEL;
		}
		else if (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK) != BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK) {
			if ( (errCode = BERR_TRACE(BKNI_WaitForEvent(
#ifdef BSCD_USE_POLLING
                                                                     in_channelHandle,
#endif
                                                                     in_channelHandle->channelWaitEvent.rcvWait,
						in_channelHandle->currentChannelSettings.timeOut.ulValue))) != BERR_SUCCESS ) {
				BKNI_EnterCriticalSection();
				in_channelHandle->channelStatus.ulStatus1 |= BSCD_RX_TIMEOUT;
				BKNI_LeaveCriticalSection();
				errCode = BSCD_STATUS_TIME_OUT;
				goto BSCD_P_DONE_LABEL;
			}
		}

		BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
		BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
		ulIntrStatus1 = in_channelHandle->ulIntrStatus1;
		ulIntrStatus2 = in_channelHandle->ulIntrStatus2;
		ulStatus2 =  in_channelHandle->ulStatus2;
		BKNI_LeaveCriticalSection();

	}while  ((ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK ) !=
					BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK);


	BKNI_EnterCriticalSection();
	in_channelHandle->ulIntrStatus2  &= ~BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK;
	in_channelHandle->ulStatus2 |= BCHP_SCA_SC_STATUS_2_rempty_MASK;
	BKNI_LeaveCriticalSection();
	BDBG_MSG("rcv interrupt received\n");

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_WaitForRcv);
	BDBG_MSG("LeaveWaitForRcv errCode = 0x%x \n", errCode);
	return errCode;
}

BERR_Code BSCD_Channel_P_WaitForRReady(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulIntrStatus1, ulIntrStatus2, ulStatus2;
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	uint32_t ulVal;
#endif

	BDBG_ENTER(("BSCD_Channel_P_WaitForRReady\n"));

	BDBG_MSG("Ready to receive rready interrupt\n");

	BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
        BKNI_Delay(100);
	BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
	ulIntrStatus1 = in_channelHandle->ulIntrStatus1 ;
        ulIntrStatus2 = in_channelHandle->ulIntrStatus2;
        ulStatus2 =  in_channelHandle->ulStatus2;
	BKNI_LeaveCriticalSection();


	do {


		BDBG_MSG("ulIntrStatus1 = 0x%x, ulIntrStatus2 = 0x%x, ulStatus2 = 0x%x \n", ulIntrStatus1, ulIntrStatus2, ulStatus2);

		if ((ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK) == BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_BGT_INTR error \n");
			errCode = BSCD_STATUS_FAILED;
			goto BSCD_P_DONE_LABEL;
		}




#ifdef BSCD_EMV2000_CWT_PLUS_4
		else if ( (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK) &&
			(( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) != BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) &&
			(in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV2000)) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_CWT_INTR error \n");
			errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
#elif defined(BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR)
		else if ( (( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK) == BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK) &&
			(( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) != BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) ) {

			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_EVENT1_INTR error \n");
			errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
#else
		else if ( (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK) &&
			(( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) != BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) ) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_CWT_INTR error \n");
			errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
#endif

		else if ( (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) &&
			((ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK) ) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_RLEN_INTR error \n");
			errCode = BSCD_STATUS_FAILED;
			goto BSCD_P_DONE_LABEL;
		}

		else if (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK) == BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RX_TIMEOUT;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_WAIT_INTR error \n");
	 		errCode = BSCD_STATUS_TIME_OUT;
		         goto BSCD_P_DONE_LABEL;
		}

		else if (( ulStatus2 & BCHP_SCA_SC_STATUS_2_roverflow_MASK) == BCHP_SCA_SC_STATUS_2_roverflow_MASK) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulStatus2 &= ~BCHP_SCA_SC_STATUS_2_roverflow_MASK;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_ROVERFLOW error \n");
			errCode = BSCD_STATUS_FAILED;
		         goto BSCD_P_DONE_LABEL;
		}
		/*09/20/05,Allen.C, check if the Card is removed  */
		else if ( (in_channelHandle-> bIsCardRemoved == true) &&
			(( ulIntrStatus1 & BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK ) == BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK)) {
			BKNI_EnterCriticalSection();
			in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			in_channelHandle->channelStatus.ulStatus1  |= BSCD_RESET_CHANNEL_REQUIRED;
			in_channelHandle-> bIsCardRemoved = false ;
			BKNI_LeaveCriticalSection();
			BDBG_ERR("ScardDeviceWaitForRReady: SC_CARD_REMOVED error \n");
	 		errCode = BSCD_STATUS_FAILED ;
		         goto BSCD_P_DONE_LABEL;
		}

		else if (( ulIntrStatus2 & BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) != BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) {
			if ( (errCode = BERR_TRACE(BKNI_WaitForEvent(
#ifdef BSCD_USE_POLLING
                                                                     in_channelHandle,
#endif
                                                                     in_channelHandle->channelWaitEvent.rcvWait,
						in_channelHandle->currentChannelSettings.timeOut.ulValue))) != BERR_SUCCESS ) {

				BKNI_EnterCriticalSection();
				in_channelHandle->channelStatus.ulStatus1 |= BSCD_RX_TIMEOUT;
				BKNI_LeaveCriticalSection();
				BDBG_ERR("ScardDeviceWaitForRReady: BKNI_WaitForEvent timeout error %d\n", in_channelHandle->currentChannelSettings.timeOut.ulValue);
				errCode = BSCD_STATUS_TIME_OUT;
				goto BSCD_P_DONE_LABEL;
			}
		}

		BKNI_EnterCriticalSection();
#ifdef BSCD_USE_POLLING
		BSCD_Channel_P_IntHandler_isr(in_channelHandle, 0);
#endif
		ulIntrStatus1 = in_channelHandle->ulIntrStatus1;
		ulIntrStatus2 = in_channelHandle->ulIntrStatus2;
		ulStatus2 =  in_channelHandle->ulStatus2;
		BKNI_LeaveCriticalSection();

	}while  ((ulStatus2 & BCHP_SCA_SC_STATUS_2_rready_MASK ) !=
					BCHP_SCA_SC_STATUS_2_rready_MASK);

	BKNI_EnterCriticalSection();
	in_channelHandle->ulIntrStatus2  &= ~BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK;
	in_channelHandle->ulStatus2  &= ~BCHP_SCA_SC_STATUS_2_rready_MASK;
	BKNI_LeaveCriticalSection();
	BDBG_MSG("rready interrupt received\n");


BSCD_P_DONE_LABEL:

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			/* Disable event1 */
			ulVal = BKNI_RegRead8(
				in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT1_CMD_4));
			ulVal &= ~(BCHP_SCA_SC_EVENT1_CMD_4_event_en_MASK);
			BKNI_RegWrite8(
				in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT1_CMD_4),
				 ulVal);
#endif

	BDBG_LEAVE(BSCD_Channel_P_WaitForRReady);
	BDBG_MSG("BSCD_Channel_P_WaitForRReady errCode = 0x%x\n", errCode);

#ifdef BSCD_EMV2000_CWT_PLUS_4
	in_channelHandle->bIsReceive = false;
#endif

	return errCode;
}

BERR_Code BSCD_Channel_P_Activating(
		BSCD_ChannelHandle	in_channelHandle
)
{

	BERR_Code	errCode = BERR_SUCCESS;
	uint32_t        ulIFCmdVal, ulVal;
	BSCD_Timer 	timer = {BSCD_TimerType_eGPTimer, {BSCD_GPTimerMode_eIMMEDIATE}, true, true};
	BSCD_TimerValue timeValue= {BSCD_MAX_RESET_IN_CLK_CYCLES, BSCD_TimerUnit_eCLK};

	BSCD_Timer 	wwtTimer = {BSCD_TimerType_eWaitTimer, {BSCD_GPTimerMode_eIMMEDIATE}, true, true};
	BSCD_TimerValue wwtTimeValue= {BSCD_MAX_ETU_PER_ATR_BYTE_EMV2000, BSCD_TimerUnit_eETU};


	unsigned char   i;
	uint32_t        ulTimerCntVal1, ulTimerCntVal2;
	uint32_t        ulTimerCntVal;
	uint32_t        ulPrevTimerCntVal = 0;

	BDBG_ENTER(("BSCD_Channel_P_Activating\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle == NULL ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	/* We disable interrupt here, since if the card removal interrupt comes ISR will set the vcc bit */
	BKNI_EnterCriticalSection();

	/* Now check card presence */
	if (in_channelHandle->channelStatus.bCardPresent == false)
	{
		BKNI_LeaveCriticalSection();
		errCode = BERR_TRACE(BSCD_STATUS_DEACTIVATE); /* this is the only error code which is similar */
		goto BSCD_P_DONE_LABEL;
	}

	/* Use Auto Deactivation instead of TDA8004 */
	if (in_channelHandle->currentChannelSettings.bAutoDeactiveReq == true) {
		ulIFCmdVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;
		ulIFCmdVal |= BCHP_SCA_SC_IF_CMD_1_auto_vcc_MASK;
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulIFCmdVal);
	}

	/* Set SC_RST low = RSTIN low */
	BSCD_Channel_ResetSignal(in_channelHandle, 0);

	/* Turn on Vcc (Set SC_VCC low = CMDVCC low) */
	BSCD_Channel_PowerICC(in_channelHandle, BSCD_PowerICC_ePowerUp);

	BKNI_LeaveCriticalSection();

	BDBG_MSG("Activating: SC_RST low \n");

	/* wait for 42,000 clk cycles. */
	for (i=0; i<in_channelHandle->currentChannelSettings.ucExternalClockDivisor; i++) {

		timer.bIsTimerInterruptEnable = true;
		timer.bIsTimerEnable = true;
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_P_WaitForTimerEvent(in_channelHandle, 1));

		/* Disable timer */
		timer.bIsTimerInterruptEnable = false;
		timer.bIsTimerEnable = false;
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer));
	}

	/*****************************************************
	**
	**  Set all required registers before we receive ATR
	**
	******************************************************/
	/* Set this to 0 temporarily during ATR session.  For EMV,
		we will set it back in BSCD_Channel_P_EMVATRReceiveAndDecode.
		For the rest, the application should set it back */
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_2), 0);

	/* Enable 2 interrupts with callback */
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
 				BSCD_Channel_EnableIntrCallback_isr (in_channelHandle, BSCD_IntType_eATRInt, BSCD_Channel_P_ATRCB_isr));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_EnableIntrCallback_isr (in_channelHandle, BSCD_IntType_eRcvInt, BSCD_Channel_P_RcvCB_isr));
	/* Enable WWT to ensure the max interval between 2 consecutive ATR chars of 10080 ETU */
	BDBG_MSG("Activating: Set WWT timer \n");
	if (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000)
		wwtTimeValue.ulValue = BSCD_MAX_ETU_PER_ATR_BYTE_EMV2000;
	else /* EMV 96 or the rest */
		wwtTimeValue.ulValue = BSCD_MAX_ETU_PER_ATR_BYTE;
	wwtTimer.timerMode.eWaitTimerMode = BSCD_WaitTimerMode_eWorkWaitTime;
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_ConfigTimer(in_channelHandle, &wwtTimer, &wwtTimeValue));

	/* Set BCM to get ATR packet.       */
	BDBG_MSG("Activating: Set BCM to get ATR packet\n");
	ulVal =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1));
	ulVal |= (BCHP_SCA_SC_UART_CMD_1_get_atr_MASK | BCHP_SCA_SC_UART_CMD_1_io_en_MASK);
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1), ulVal);

	/* Set RST */
	/* Use Auto Deactivation instead of TDA8004 */
	if (in_channelHandle->currentChannelSettings.bAutoDeactiveReq == true) {
		ulIFCmdVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;
		ulIFCmdVal |= BCHP_SCA_SC_IF_CMD_1_auto_rst_MASK;
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulIFCmdVal);
	}
	BSCD_Channel_ResetSignal(in_channelHandle, 1);

	if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
	{
		/* No timer wait needed since NXP8026 has ATR counter embedded */
		/* if ATR counter expires there will be PRES interrupt */

		NXP8026_Activate();

		errCode = BSCD_Channel_P_WaitForATRStart(in_channelHandle);
	}
	else
	{ /* start of non-NXP8026 activation */
		/* wait for 40,000 clk cycles for EMV96 and 42000 for EMV2000 */
		for (i=0; i<in_channelHandle->currentChannelSettings.ucExternalClockDivisor ; i++)  {

			/* Set Timer */
			timer.bIsTimerInterruptEnable = true;
			timer.bIsTimerEnable = true;
			timer.eTimerType = BSCD_TimerType_eGPTimer;
			timer.timerMode.eGPTimerMode = BSCD_GPTimerMode_eIMMEDIATE;
			if (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000)
				timeValue.ulValue = BSCD_EMV2000_MAX_ATR_START_IN_CLK_CYCLES + BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES;
			else
				timeValue.ulValue = BSCD_MAX_ATR_START_IN_CLK_CYCLES + BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES;
			timeValue.unit  = BSCD_TimerUnit_eCLK;
			BDBG_MSG("Activating: Set GP timer \n");
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

			if ((errCode = BSCD_Channel_P_WaitForATRStart(in_channelHandle)) != BERR_SUCCESS) {
				BDBG_MSG("Activating: WaitForATRStart failed\n");
				/* Disable timer */
				timer.bIsTimerInterruptEnable = false;
				timer.bIsTimerEnable = false;

				BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer); /* remove the check err since that will set the errCode */

				if (errCode == BSCD_STATUS_TIME_OUT) {

					if (i == (in_channelHandle->currentChannelSettings.ucExternalClockDivisor -1)) {

						/* if this is the last loop and we still timeout, major error */
						/* Need to return deactivate for EMV2000 test 1719 xy=30 */
						BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_DEACTIVATE, true);
					}
					else {
					/* If this is not the last loop, continue */
						ulPrevTimerCntVal += BSCD_MAX_ATR_START_IN_CLK_CYCLES;
						continue;
					}
				}
				else {
					/* If the error is not scTimeOut, major error */
					BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);

				}

			}


			/* Disable timer */
			timer.bIsTimerInterruptEnable = false;
			timer.bIsTimerEnable = false;
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer));
			BDBG_MSG("Activating: Disable GP timer \n");

			/* Read timer counter, the ATR shall be received after 400 clock cycles */
			ulTimerCntVal2 =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CNT_2)) ;
			ulTimerCntVal1 =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CNT_1)) ;

			ulTimerCntVal =  ((( ulTimerCntVal2) << 8) | ulTimerCntVal1) + ulPrevTimerCntVal;

			if ((ulTimerCntVal < (uint32_t)(BSCD_MIN_ATR_START_IN_CLK_CYCLES * in_channelHandle->currentChannelSettings.ucExternalClockDivisor) ) ||
				(ulTimerCntVal > timeValue.ulValue) ) {

				BDBG_MSG("PreATRREceive: ulTimerCmdVal = %lu, timerValue.vlValue=%d\n", ulTimerCntVal, timeValue.ulValue);
				/* Need to return deactivate for EMV2000 test 1719 xy=30 */
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_DEACTIVATE, true);
			}

			/*
				Enable WWT to ensure all ATR bytes are received within certain time
			*/
			if (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000)
				timeValue.ulValue = BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES_EMV2000;
			else /* EMV 96 or the rest */
				timeValue.ulValue = BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES;
			timeValue.unit = BSCD_TimerUnit_eETU;
			timer.bIsTimerInterruptEnable = true;
			timer.bIsTimerEnable = true;
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

			BDBG_MSG("ulTimerCntVal = %lu, MIN_ATR_START_IN_CLK_CYCLES = %d\n", ulTimerCntVal, BSCD_MIN_ATR_START_IN_CLK_CYCLES);

			if (errCode == BERR_SUCCESS) {
				in_channelHandle->channelStatus.bCardActivate = true;
				break;
			}

		}
	} /* End of non-NXP8026 activation */


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_Activating);
	return( errCode );
}




void BSCD_P_HexDump(
      char          *inp_cTitle,
      unsigned char *inp_ucBuf,
      unsigned int  in_unLen
)
{
	size_t   i;

	BDBG_MSG("\n%s (%u bytes):", inp_cTitle, in_unLen);

	for(i=0; i<in_unLen; i++) {

		if(!(i%20)) {
			BDBG_MSG("\n");
		}

		BDBG_MSG("%02X  ",*(inp_ucBuf+i));
	}

	BDBG_MSG("\n");

	BSTD_UNUSED(inp_cTitle);
	BSTD_UNUSED(inp_ucBuf);

}





BERR_Code BSCD_Channel_P_T0ReadData(
		BSCD_ChannelHandle       in_channelHandle,
		uint8_t                  *outp_ucRcvData,
		unsigned long            *outp_ulNumRcvBytes,
		unsigned long            in_ulMaxReadBytes
)
{
	BERR_Code 		errCode = BERR_SUCCESS;
	uint32_t               	ulLen = 0;
	uint32_t		ulStatus2;
#ifndef BSCD_DSS_ICAM
	BSCD_Timer 		timer = {BSCD_TimerType_eWaitTimer, {BSCD_WaitTimerMode_eWorkWaitTime}, false, false};
/*  to use WWT instead of GT
    BSCD_Timer 		timer = {BSCD_TimerType_eGPTimer, {BSCD_GPTimerMode_eIMMEDIATE}, true, true};
 */
	BSCD_TimerValue    timeValue= {BSCD_DEFAULT_WORK_WAITING_TIME, BSCD_TimerUnit_eETU};
	uint32_t        bEnabledHere = 0;
#endif

	BDBG_ENTER(("BSCD_Channel_P_T0ReadData\n"));

	*outp_ulNumRcvBytes = 0;

	BKNI_EnterCriticalSection();
	in_channelHandle->ulStatus2 = ulStatus2 = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_STATUS_2));
	BKNI_LeaveCriticalSection();

	BDBG_MSG("BSCD_Channel_P_T0ReadData: in_ulMaxReadBytes = %d\n", in_ulMaxReadBytes);

	while (ulLen < in_ulMaxReadBytes ) {

#ifndef BSCD_DSS_ICAM
		/*
			This is a backup time out for non EMV standard.
			Just in case, we do not read all the byte in one shot but
			WWT was disable in BSCD_Channel_Receive
		*/
		if ((in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV1996) &&
			(in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV2000) ) {
			/* Cannot enable GT if WWT is disabled, since WWT can be disabled when reach here, but GT is too small  */
			/* we now reenable WWT here, instead of using GT */
			if (!BSCD_Channel_IsTimerEnabled(in_channelHandle, BSCD_TimerType_eWaitTimer))
			{
				timeValue.ulValue = in_channelHandle->currentChannelSettings.workWaitTime.ulValue;

				BSCD_P_CHECK_ERR_CODE_FUNC2(errCode, BSCD_STATUS_READ_FAILED,
						BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

				bEnabledHere = 1;
			}
		}
#endif


		BDBG_MSG("\nSmartCardReadCmd: After SmartCardSetGPTimer\n");

		BKNI_EnterCriticalSection();
		ulStatus2 = in_channelHandle->ulStatus2;
		BKNI_LeaveCriticalSection();

	 	if ( (( ulStatus2 & BCHP_SCA_SC_STATUS_2_rempty_MASK) == BCHP_SCA_SC_STATUS_2_rempty_MASK)  &&
	   	     ((errCode = BSCD_Channel_P_WaitForRcv(in_channelHandle)) != BERR_SUCCESS) ) {

			errCode = BERR_TRACE(errCode);
			BDBG_MSG("After  BSCD_Channel_P_WaitForRcv in BSCD_Channel_P_T0ReadData errCode = 0x%x\n", errCode);
#ifndef BSCD_DSS_ICAM
			/* Disable timer */
			if ((in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV1996) &&
				(in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV2000) ) {
				if (bEnabledHere)
				{
					timer.bIsTimerInterruptEnable = false;
					timer.bIsTimerEnable = false;
					BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer);
				}
			}
#endif

		         if (errCode == BSCD_STATUS_PARITY_EDC_ERR ) {
				;   /* No op in software , hardware will retry */
			}
			else if (errCode == BSCD_STATUS_TIME_OUT)
				break;
			else {
				return BSCD_STATUS_READ_FAILED;
			}
		}

		/* BDBG_MSG("\nSmartCardReadCmd: After ScardDeviceWaitForRcv\n"); */

#ifndef BSCD_DSS_ICAM
		/* Disable timer */
		if ((in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV1996) &&
			(in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV2000) ) {
			if (bEnabledHere)
			{
				timer.bIsTimerInterruptEnable = false;
				timer.bIsTimerEnable = false;
				BSCD_P_CHECK_ERR_CODE_FUNC2(errCode, BSCD_STATUS_READ_FAILED,
					BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer));
			}
		}
#endif

		while (ulLen < in_ulMaxReadBytes ) {
			BDBG_MSG("In  ulLen < in_ulMaxReadBytes\n");

			if ((errCode = BSCD_Channel_P_ByteRead(in_channelHandle, &outp_ucRcvData[ulLen])) == BERR_SUCCESS) {

				in_channelHandle->channelStatus.ulStatus1 &= ~BSCD_RX_PARITY;
				if ((outp_ucRcvData[ulLen] == 0x60) &&
				     (in_channelHandle->currentChannelSettings.bNullFilter == true) ) {
					BDBG_MSG("Ignore 0x60 == %2X ", outp_ucRcvData[ulLen]);
					continue;

				}
				else {
					BDBG_MSG("%2X ", outp_ucRcvData[ulLen]);
					ulLen++;
				}

			}

			else if (errCode == BSCD_STATUS_PARITY_EDC_ERR) {
				BDBG_MSG("errCode == BSCD_STATUS_PARITY_EDC_ERR\n");
				continue;
			}
			else {
				break;
			}
		}
	}

#ifndef BSCD_DSS_ICAM
BSCD_P_DONE_LABEL:
#endif

	/* 09/28/2006 QX: return data even it's a partial read
	if (errCode != BERR_SUCCESS)
		ulLen = 0;
	*/
	*outp_ulNumRcvBytes = ulLen;

	BDBG_LEAVE(BSCD_Channel_P_T0ReadData);
	BDBG_MSG("Leave BSCD_Channel_P_T0ReadData errCode = 0x%x\n", errCode);
	return errCode;
}


BERR_Code BSCD_Channel_P_ByteRead(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char           *outp_ucData
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulStatus2;

	BDBG_ENTER(("BSCD_Channel_P_ByteRead\n"));

	BKNI_EnterCriticalSection();
	in_channelHandle->ulStatus2 =  ulStatus2 = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_STATUS_2));
	BKNI_LeaveCriticalSection();

	if (( ulStatus2 & BCHP_SCA_SC_STATUS_2_rempty_MASK) != BCHP_SCA_SC_STATUS_2_rempty_MASK) {

		*outp_ucData = (unsigned char) BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_RECEIVE));

		BKNI_EnterCriticalSection();
		in_channelHandle->ulStatus2 =  ulStatus2 = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_STATUS_2));
		BKNI_LeaveCriticalSection();

	      if ( (( ulStatus2 & BCHP_SCA_SC_STATUS_2_rpar_err_MASK) == BCHP_SCA_SC_STATUS_2_rpar_err_MASK) &&
		      (in_channelHandle->currentChannelSettings.scStandard  != BSCD_Standard_eIrdeto)) {
			BDBG_MSG("Receive a parity error byte\n");
			BKNI_EnterCriticalSection();
			in_channelHandle->channelStatus.ulStatus1 |= BSCD_RX_PARITY;
			BKNI_LeaveCriticalSection();
			return BSCD_STATUS_PARITY_EDC_ERR;
	      }
	}
	else
		return (BSCD_STATUS_FAILED);

	BDBG_LEAVE(BSCD_Channel_P_ByteRead);
	return errCode;
}


BERR_Code BSCD_Channel_P_T1ReadData(
		BSCD_ChannelHandle    in_channelHandle,
		uint8_t               *outp_ucRcvData,
		unsigned long         *outp_ulNumRcvBytes,
		unsigned long         in_ulMaxReadBytes
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulVal, ulLen1, ulLen2;
	uint32_t ulLen = 0, i;
	BSCD_Timer timer = {BSCD_TimerType_eWaitTimer, {BSCD_GPTimerMode_eIMMEDIATE}, false, false};

	BDBG_ENTER(("BSCD_Channel_P_T1ReadData\n"));

	BSTD_UNUSED(in_ulMaxReadBytes);
	*outp_ulNumRcvBytes	 = 0;

	ulVal =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD));

		ulVal |= BCHP_SCA_SC_PROTO_CMD_tbuf_rst_MASK;
	if ((in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eES) ||
		(in_channelHandle->currentChannelSettings.bTPDU == true) ) /* This condition added for WHQL card 5 test */
	{
		/* application computes its own LRC or CRC and appends it as the last byte */
		ulVal &= ~BCHP_SCA_SC_PROTO_CMD_edc_en_MASK;
	}
	else {
		ulVal |= BCHP_SCA_SC_PROTO_CMD_edc_en_MASK; /* for APDU or other standards, hw checks EDC/LRC */
	}

	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD), ulVal);

	if ((errCode = BSCD_Channel_P_WaitForRReady(in_channelHandle)) != BERR_SUCCESS)  {
		/* If parity error, continue reading all the bytes */
		errCode = BERR_TRACE(errCode);
		return BSCD_STATUS_NO_SC_RESPONSE;
	}

	/* Disable block wait timer */
	timer.eTimerType = BSCD_TimerType_eWaitTimer;
	timer.timerMode.eWaitTimerMode = BSCD_WaitTimerMode_eBlockWaitTime;
	BSCD_P_CHECK_ERR_CODE_FUNC2(errCode, BSCD_STATUS_READ_FAILED,
		BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer));

	/* Disable cwt since we already receive all the bytes */
	ulVal =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD));
	ulVal &= ~BCHP_SCA_SC_TIMER_CMD_cwt_en_MASK;
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD), ulVal);

	/* Clear cwt_intr so that it won't show up next time */
	BKNI_EnterCriticalSection();
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK;
#else
	in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK;
        BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_STAT_2));
#endif
	BKNI_LeaveCriticalSection();

	ulLen1 =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_RLEN_1));
	ulLen2 =  BKNI_RegRead8(in_channelHandle,	(in_channelHandle->ulRegStartAddr + BSCD_P_RLEN_2));

	/* RLEN_9_BIT_MASK = 0x01ff */
	ulLen = ((((unsigned short) ulLen2) << 8) | ulLen1) & BSCD_RLEN_9_BIT_MASK;
	BDBG_MSG("SmartCardBlockRead: rlen = %d\n", ulLen);

	if (ulLen) {
		for (i = 0; i < ulLen; i++) {

			outp_ucRcvData[i] =  (uint8_t) BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_RECEIVE));
			ulVal =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_STATUS_2));

			if ((ulVal & BCHP_SCA_SC_STATUS_2_rpar_err_MASK) == BCHP_SCA_SC_STATUS_2_rpar_err_MASK) {
				BDBG_MSG("SmartCardBlockRead: parity error\n");
				errCode = BSCD_STATUS_PARITY_EDC_ERR;
			}
			else if ((ulVal & BCHP_SCA_SC_STATUS_2_edc_err_MASK) == BCHP_SCA_SC_STATUS_2_edc_err_MASK) {
				BDBG_MSG("SmartCardBlockRead: EDC error\n");
				errCode = BSCD_STATUS_PARITY_EDC_ERR;
			}

			if ((i % 16) == 0) {
				BDBG_MSG("\n");
				/* Need to this to fix the warning if BDBG_MSG is defined to nothing */
				;
			}

			BDBG_MSG("%02x ", outp_ucRcvData[i]);

		}
	}

BSCD_P_DONE_LABEL:

	if (errCode != BERR_SUCCESS)
		ulLen = 0;

	*outp_ulNumRcvBytes = ulLen;

	BDBG_LEAVE(BSCD_Channel_P_T1ReadData);
	return errCode;
}


BERR_Code BSCD_Channel_P_ReceiveAndDecode(
		BSCD_ChannelHandle	in_channelHandle
)
{

	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_ReceiveAndDecode\n"));

	if (in_channelHandle->currentChannelSettings.resetCardAction == BSCD_ResetCardAction_eNoAction) {
		BDBG_MSG("In BSCD_Channel_P_ReceiveAndDecode BSCD_ResetCardAction_eNoAction\n");
		return BERR_SUCCESS;
	}
	else if (in_channelHandle->currentChannelSettings.resetCardAction == BSCD_ResetCardAction_eReceiveAndDecode) {
		BDBG_MSG("In BSCD_Channel_P_ReceiveAndDecode BSCD_ResetCardAction_eReceiveAndDecode standard = %d\n",
			in_channelHandle->currentChannelSettings.scStandard);
		switch (in_channelHandle->currentChannelSettings.scStandard) {
			case BSCD_Standard_eEMV1996:
			case BSCD_Standard_eEMV2000:
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
				/**** no EMV
				if ( (errCode = BSCD_Channel_P_EMVATRReceiveAndDecode(in_channelHandle)) != BERR_SUCCESS) {
					errCode = BERR_TRACE(errCode);
					goto BSCD_P_DONE_LABEL;
				}
				****/
				break;

			case BSCD_Standard_eISO:
				if ( (errCode = BSCD_Channel_P_ISOATRReceiveAndDecode(in_channelHandle)) != BERR_SUCCESS) {
					errCode = BERR_TRACE(errCode);
					goto BSCD_P_DONE_LABEL;
				}
				break;

			default:
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
		}

	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ReceiveAndDecode);
	return( errCode );
}

/* interrupt handler bottom half
   Currently it only handles card insertion/removal bh */
void BSCD_Channel_P_IntHandler_bh(BSCD_ChannelHandle channelHandle, int in_channelNumber)
{
	uint32_t   unStaReg1 = 0, unPrevStaReg1 = 0;

	/* Store status_1 to determine if hardware failure */
	unPrevStaReg1 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_STATUS_1));
	if (channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
	{	/* for this coupler, presence status needs to be read from coupler */
		unPrevStaReg1 |= (NXP8026_Get_Card_Presence() ? BCHP_SCA_SC_STATUS_1_card_pres_MASK : 0);
	}

#ifdef BSCD_INTERRUPT_DEBUG
	BDBG_MSG("unPrevStaReg1 = 0x%2x\n", unPrevStaReg1);
#endif

#ifndef CYGNUS_SCI_DRV   //On cygnus, if SCA_SC_IF_CMD_1.vcc = 0, then all present irq can not occur after re-inserting card
	/* In case this is an emergency deactivation, we have to set
	   IF_CMD_1[VCC]=1 to detect card pres again. */
	BSCD_Channel_PowerICC(channelHandle, BSCD_PowerICC_ePowerUp);
#endif

	/* TDA8004 suggests we to wait until debounce stabilizes.  NDS suggests to
		sleep for 10 milli seconds.  This may hold the system for 10ms but it is
		okay since the system should not continue without the card. */
	/* All customers should use TDA8024 now */
	/* BKNI_Delay(10000);*/

	unStaReg1 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_STATUS_1));
	if (channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
	{	/* for this coupler, presence status needs to be read from coupler */
		unStaReg1 |= (NXP8026_Get_Card_Presence() ? BCHP_SCA_SC_STATUS_1_card_pres_MASK : 0);

		/* Also check for other error status */
		if (NXP8026_Get_Card_ATR_Error())
			BKNI_SetEvent(channelHandle->channelWaitEvent.atrStart);
	}

	/* According TDA 8004 Application note, this is how to determine card presence, card removal and hardware failure. */
	if ( (unStaReg1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK) &&  (!(unPrevStaReg1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK)) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("hardware failure, unStaReg1=0x%2x\n", unStaReg1);
#endif
		channelHandle->channelStatus.bCardPresent = true;
		channelHandle->channelStatus.ulStatus1 |= BSCD_HARDWARE_FAILURE | BSCD_RESET_CHANNEL_REQUIRED;
	}
	else if ( (unStaReg1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK) &&  (unPrevStaReg1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("SC Inserted\n");
#endif
		channelHandle->channelStatus.bCardPresent  = true;
		channelHandle->channelStatus.bCardActivate = false; /* we have not reset the card yet */
		channelHandle->channelStatus.bPPSDone      = false;
	}
	else if (!(unStaReg1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK) && !(unPrevStaReg1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK)) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("SC Removed\n");
#endif

		/* Disable all interrupt but pres_intr to support auto-deactivation.
		     Auto Deactvation will cause a parity_intr and retry_intr to loop forever
		*/
		BKNI_RegWrite8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1), 0);
		BKNI_RegWrite8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_2), 0);

		/* 09/20/05,Allen.C, remember Card was removed */
		channelHandle->bIsCardRemoved = true;

		channelHandle->channelStatus.bCardPresent  = false;
		channelHandle->channelStatus.bCardActivate = false;
		channelHandle->channelStatus.bPPSDone      = false;
		channelHandle->channelStatus.ulStatus1 |= BSCD_RESET_CHANNEL_REQUIRED;
	}

	if (channelHandle->channelStatus.bCardPresent == true) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("SC %d Insertion Interrupt\n", channelHandle->ucChannelNumber);

#endif
	}
	else {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("SC %d Removal Interrupt\n", channelHandle->ucChannelNumber);
#endif
	}

	/* re-enable pres intr */
	BSCD_Channel_EnableIntrCallback_isr (channelHandle, BSCD_IntType_eCardInsertInt, BSCD_Channel_P_CardInsertCB_isr);

	/* The bottom half has done */
	BKNI_SetEvent( channelHandle->channelWaitEvent.bhPres);
	
#ifdef BSCD_INTERRUPT_DEBUG
	BDBG_MSG("Intr BH done\n", channelHandle->ucChannelNumber);
#endif
}

/* ISR handler.
   Return value: if a bottom half needed or not
*/
int BSCD_Channel_P_IntHandler_isr(BSCD_ChannelHandle channelHandle, int in_channelNumber)
{
	uint32_t   unStaReg1 = 0, unStaReg2 = 0, unProtoCmdReg = 0;
	uint32_t   unIntrEn1 = 0, unIntrEn2 = 0, unIntrStaReg1 = 0, unIntrStaReg2 = 0;
	uint32_t   ulVal;
	BSCD_IntrType event;
	BERR_Code errCode = BERR_SUCCESS;
	BSCD_Timer 		timer = {BSCD_TimerType_eGPTimer, {BSCD_GPTimerMode_eIMMEDIATE}, true, true};
	int 		i;
	int32_t    unBH = 0; /* if bottom half needed */

#ifdef BSCD_EMV2000_CWT_PLUS_4
	BSCD_Timer 		cwtTimer = {BSCD_TimerType_eWaitTimer, {BSCD_WaitTimerMode_eWorkWaitTime}, true, true};
	BSCD_TimerValue    cwtTimeValue= {16, BSCD_TimerUnit_eETU};
#endif

	BDBG_ENTER(("BSCD_Channel_P_IntHandler_isr\n"));

#ifdef BSCD_USE_POLLING
	BSTD_UNUSED(in_param2);
#endif


	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,  (channelHandle == NULL) );


	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(channelHandle->bIsOpen ==  false) );


	/* Read Smartcard Interrupt Status & Mask Register */
	unProtoCmdReg = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD));
	unStaReg1 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_STATUS_1));
	unStaReg2 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_STATUS_2));

	unIntrEn1 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1));
	unIntrStaReg1 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_STAT_1));
	unIntrEn2 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_2));
	unIntrStaReg2 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_STAT_2));

#ifdef BSCD_INTERRUPT_DEBUG
	BDBG_MSG("ChannelNumber = %d\n", channelHandle->ucChannelNumber);
	BDBG_MSG("BSCD_P_INTR_EN_1   = 0x%2x, BSCD_P_INTR_EN_2   = 0x%2x, \n", unIntrEn1, unIntrEn2);
	BDBG_MSG("BSCD_P_INTR_STAT_1 = 0x%2x, BSCD_P_INTR_STAT_2 = 0x%2x\n", unIntrStaReg1, unIntrStaReg2);
	BDBG_MSG("BSCD_P_STATUS_1    = 0x%2x, BSCD_P_STATUS_2    = 0x%2x\n", unStaReg1, unStaReg2);
#endif

	/* Process interrupt */
	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_pres_ien_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("PRES_INTR , IntrEn1 = %x, IntrStaReg1 = %x, unStaReg1 = %x, ucSlot = %d\n", unIntrEn1, unIntrStaReg1, unStaReg1, channelHandle->ucChannelNumber);
#endif

		channelHandle->ulIntrStatus1 = unIntrStaReg1;
		channelHandle->ulStatus1  = unStaReg1;

		/* Disable pres intr to debounce the card pres */
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,	BSCD_Channel_DisableIntrCallback_isr (channelHandle, BSCD_IntType_eCardInsertInt));

		unBH = 1;
		BKNI_ResetEvent(channelHandle->channelWaitEvent.bhPres); /* clear the event so any process waiting would wait for this bottom half */
	}

	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_tpar_ien_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_tpar_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("TPAR_INTR , IntrEn1 = %x, IntrStaReg1 = %x, ucSlot = %d\n", unIntrEn1, unIntrStaReg1, channelHandle->ucChannelNumber);
#endif

		channelHandle->channelStatus.ulStatus1 |= BSCD_TX_PARITY;

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eTParityInt;
#ifdef BSCD_EMV2000_FIME
                channelHandle->parity_error_cnt++;
#endif
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.tParityIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.tParityIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_timer_ien_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("TIMER_INTR , IntrEn1 = %x, IntrStaReg1 = %x, ucSlot = %d\n", unIntrEn1, unIntrStaReg1, channelHandle->ucChannelNumber);
#endif

		timer.bIsTimerInterruptEnable = false;
		timer.bIsTimerEnable = false;
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_EnableDisableTimer_isr(channelHandle, &timer));

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		/* We need to signal different events to take care of different scenarioes */
		event = BSCD_IntType_eTimerInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.timerIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.timerIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_bgt_ien7_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("BGT_INTR , IntrEn1 = %x, IntrStaReg1 = %x, ucSlot = %d\n", unIntrEn1, unIntrStaReg1, channelHandle->ucChannelNumber);
#endif

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		/* We need to signal different events to take care of different scenarioes */
		event = BSCD_IntType_eBGTInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.bgtIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.bgtIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_tdone_ien_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("TDONE_INTR ,IntrEn1 = %x, IntrStaReg1 = %x, ucSlot = %d\n", unIntrEn1, unIntrStaReg1, channelHandle->ucChannelNumber);
#endif

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		/* We need to signal different events to take care of different scenarioes */
		event = BSCD_IntType_eTDoneInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.tDoneIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.tDoneIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_retry_ien_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("RETRY_INTR IntrEn1 = %x, IntrStaReg1 = %x, ucSlot = %d\n", unIntrEn1, unIntrStaReg1, channelHandle->ucChannelNumber);
#endif

		/* If parity tx or rx retrial failes, we should reset uart and NOT to continue tx any more data */
		ulVal =  BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1)) ;
		ulVal |= (BCHP_SCA_SC_UART_CMD_1_uart_rst_MASK);
		BKNI_RegWrite8(channelHandle,	(channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1), ulVal);

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		/* We need to signal different events to take care of different scenarioes */
		event = BSCD_IntType_eRetryInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.retryIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.retryIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_tempty_ien_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_tempty_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("TEMPTY_INTR ,IntrEn1 = %x, IntrStaReg1 = %x, ucSlot = %d\n",	unIntrEn1, unIntrStaReg1, channelHandle->ucChannelNumber);
#endif

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		/* Currently we do not need this.  No signal needed */
		event = BSCD_IntType_eTEmptyInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.tEmptyIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.tEmptyIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_rpar_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_rpar_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("RPAR_INTR , IntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n", unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);
#endif

		channelHandle->channelStatus.ulStatus1 |= BSCD_RX_PARITY;
		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eRParityInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.rParityIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.rParityIsrCBFunc[i])) (channelHandle, &event);
		}

	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_atrs_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK) ) {
		BDBG_MSG("ATRS_INTR  unIntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n", unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);

		/*
			We need this interrupt to measure the period of time we received leading edge of
			the start bit of the first ATR byte.  As soon as we receive this interrupt, we should
			stop the timer so that we could get more accurate timing
		*/

		timer.bIsTimerInterruptEnable = false;
		timer.bIsTimerEnable = false;
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_EnableDisableTimer_isr(channelHandle, &timer));

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eATRInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.atrIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.atrIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_cwt_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("CWT_INTR IntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n", unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);
#endif

		/* If cwt_intr comes in after rready_intr, it is considered normal */
		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eCWTInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.cwtIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.cwtIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_rlen_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("RLEN_INTR , IntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n", unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);
#endif

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eRLenInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.rLenIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.rLenIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_wait_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("WAIT_INTR , IntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n", unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);
#endif

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		/* We need to signal different events to take care of different scenarioes */
		event = BSCD_IntType_eWaitInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.waitIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.waitIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_rcv_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
	BDBG_MSG("RCV_INTR IntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n",	unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);
#endif

/* Enable RCV_INTR only in T=1, EMV 2000 to resolve CWT+4 issue */
#ifdef BSCD_EMV2000_CWT_PLUS_4
		if ( (channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000) &&
		  	(channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) &&
		  	(channelHandle->bIsReceive == true))  {

			/* Disable BWT timer */
			cwtTimer.bIsTimerInterruptEnable = false;
			cwtTimer.bIsTimerEnable = false;
			BSCD_P_CHECK_ERR_CODE_FUNC2(errCode, BSCD_STATUS_READ_FAILED,
				BSCD_Channel_EnableDisableTimer_isr(channelHandle, &cwtTimer));

			/* Enable WWT in lieu of CWT */
			cwtTimer.bIsTimerInterruptEnable = true;
			cwtTimer.bIsTimerEnable = true;
			if (channelHandle->currentChannelSettings.ulCharacterWaitTimeInteger != 0)
				cwtTimeValue.ulValue = (2<<(channelHandle->currentChannelSettings.ulCharacterWaitTimeInteger-1))
					+ 15 + BSCD_CHARACTER_WAIT_TIME_GRACE_PERIOD;
			BSCD_P_CHECK_ERR_CODE_FUNC2(errCode, BSCD_STATUS_READ_FAILED,
					BSCD_Channel_ConfigTimer(channelHandle, &cwtTimer, &cwtTimeValue));

			BDBG_MSG("RCV_INTR  cwt = %d\n", cwtTimeValue.ulValue);

			channelHandle->ulStatus2 |= unStaReg2;
			unIntrStaReg2 &= ~BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK;

		}

#endif
/* Enable RCV_INTR only in T=1, EMV 2000 to resolve CWT+4 issue */

		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eRcvInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.rcvIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.rcvIsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_rready_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("RREADY_INTR IntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n", unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);
#endif

#ifdef BSCD_EMV2000_CWT_PLUS_4
		if ( (channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000) &&
		  	(channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1)) {
			/* Disable WWT timer, which is used as CWT + 4  */
			cwtTimer.bIsTimerInterruptEnable = false;
			cwtTimer.bIsTimerEnable = false;
			BSCD_P_CHECK_ERR_CODE_FUNC2(errCode, BSCD_STATUS_READ_FAILED,
				BSCD_Channel_EnableDisableTimer_isr(channelHandle, &cwtTimer));
			BDBG_MSG("RREADY_INTR  cwt disable\n");
		}
#endif
		if ((unStaReg2 & BCHP_SCA_SC_STATUS_2_rready_MASK) == 0) /* fix for RReady hang, where it waits for Status2 but Status2 does not contain RcvRdy */
		{
			unStaReg2 = BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_STATUS_2));
		}

		channelHandle->ulStatus2 |= unStaReg2;
		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eRReadyInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.rReadyIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.rReadyIsrCBFunc[i])) (channelHandle,  &event);
		}
	}

	if ( (unProtoCmdReg & BCHP_SCA_SC_PROTO_CMD_edc_en_MASK) && (unStaReg2 & BCHP_SCA_SC_STATUS_2_edc_err_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("EDC_ERR , ProtoCmdReg = %x, unStaReg2 = %x, ucSlot = %d\n", unProtoCmdReg, unStaReg2, channelHandle->ucChannelNumber);
#endif

		channelHandle->channelStatus.ulStatus1 |= BSCD_TX_PARITY;
		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eEDCInt;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.edcIsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.edcIsrCBFunc[i])) (channelHandle, &event);
		}
	}

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	if ( (unIntrEn1 & BCHP_SCA_SC_INTR_EN_1_event1_ien_MASK) && (unIntrStaReg1 & BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("EVENT1_INTR IntrEn1 = %x, IntrStaReg1 = %x, ucSlot = %d\n", unIntrEn1, unIntrStaReg1, channelHandle->ucChannelNumber);
#endif

		/* If cwt_intr comes in after rready_intr, it is considered normal */
		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eEvent1Int;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.event1IsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.event1IsrCBFunc[i])) (channelHandle, &event);
		}
	}

	if ( (unIntrEn2 & BCHP_SCA_SC_INTR_EN_2_event2_ien_MASK) && (unIntrStaReg2 & BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK) ) {
#ifdef BSCD_INTERRUPT_DEBUG
		BDBG_MSG("EVENT2_INTR IntrEn2 = %x, IntrStaReg2 = %x, ucSlot = %d\n", unIntrEn2, unIntrStaReg2, channelHandle->ucChannelNumber);
#endif

		/* If cwt_intr comes in after rready_intr, it is considered normal */
		channelHandle->ulIntrStatus1 |= unIntrStaReg1;
		channelHandle->ulIntrStatus2 |= unIntrStaReg2;

		event = BSCD_IntType_eEvent2Int;
		for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (channelHandle->callBack.event2IsrCBFunc[i] != NULL)
				(*(channelHandle->callBack.event2IsrCBFunc[i])) (channelHandle, &event);
		}
	}

#endif

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_IntHandler_isr);

	return unBH;
}

/* For T=0 and T=1 only */
BERR_Code BSCD_Channel_P_T0T1Transmit(
		BSCD_ChannelHandle          in_channelHandle,
		uint8_t                     *inp_ucXmitData,
		unsigned long               in_ulNumXmitBytes
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t        ulVal;
	unsigned int    i;
	BSCD_Timer 	timer = {BSCD_TimerType_eGPTimer, {BSCD_GPTimerMode_eIMMEDIATE}, true, true};
	BSCD_TimerValue timeValue= {BSCD_MIN_DELAY_BEFORE_TZERO_SEND, BSCD_TimerUnit_eETU};

	BDBG_ENTER(("BSCD_Channel_P_T0T1Transmit\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	/* BSCD_P_HexDump("Send",inp_ucXmitData, in_ulNumXmitBytes); */

	BKNI_EnterCriticalSection();
	in_channelHandle->ulIntrStatus1 &=
		~BCHP_SCA_SC_INTR_STAT_1_tpar_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_1_tempty_intr_MASK
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
		 & ~BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK
#endif
		;

	in_channelHandle->ulIntrStatus2 &=
		~BCHP_SCA_SC_INTR_STAT_2_rpar_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK &
		~BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK ;
	BKNI_LeaveCriticalSection();



	/* Reset the Transmit and Receive buffer */
	ulVal =  BCHP_SCA_SC_PROTO_CMD_tbuf_rst_MASK | BCHP_SCA_SC_PROTO_CMD_rbuf_rst_MASK |
	BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD)) ;
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD), ulVal);

	/*
	  Enable cwt here for only T=1. We will disable cwt in
	  SmartCardTOneReceive() after we receive RREADY_INTR
	*/
	if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) {

		/* Clear the possible previous cwt_intr */
		in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK;
		BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_STAT_2));

		ulVal =  BCHP_SCA_SC_TIMER_CMD_cwt_en_MASK |
				BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD)) ;
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD), ulVal);

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR

		/* Clear the possible previous event1 intr */
		in_channelHandle->ulIntrStatus1 &= ~BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK;

		if  (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000) {
			/* 4 ETU after CWT */
			BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT1_CMP), 5);

			/* start event src */
			BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT1_CMD_3), BSCD_P_CWT_INTR_EVENT_SRC);

			/* increment event src */
			BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT1_CMD_2), BSCD_P_RX_ETU_TICK_EVENT_SRC);

			/* reset event src */
			BKNI_RegWrite8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_EVENT1_CMD_1), BSCD_P_RX_START_BIT_EVENT_SRC);

			/* event_en, intr_mode, run_after_reset and run_after_compare*/
			ulVal = BCHP_SCA_SC_EVENT1_CMD_4_event_en_MASK |
					BCHP_SCA_SC_EVENT1_CMD_4_intr_after_compare_MASK |
					BCHP_SCA_SC_EVENT1_CMD_4_run_after_reset_MASK;

			ulVal &= ~(BCHP_SCA_SC_EVENT1_CMD_4_intr_after_reset_MASK |
						BCHP_SCA_SC_EVENT1_CMD_4_run_after_compare_MASK);

			BKNI_RegWrite8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_EVENT1_CMD_4), ulVal);

		}
#endif

	}


	/*
	  For EMV T=0 only, the minimum interval btw the leading
	  edges of the start bits of 2 consecutive characters sent
	  in opposite directions shall be 16.  For EMV and ISO T=1,
	  the minimum interval btw the leading edges of the start bits of 2
	  consecutive characters sent in opposite directions shall be 22.
	*/

	if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0) {


		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

	}

	else {

		/* Set Timer */
		/* 	timer.bIsTimerInterruptEnable = true;
		     	timer.bIsTimerEnable = true;
		     	timer.eTimerType = BSCD_TimerType_eGPTimer;
		     	timer.timerMode.eGPTimerMode = BSCD_GPTimerMode_eIMMEDIATE;
			timeValue.ulValue = BSCD_BLOCK_GUARD_TIME;
			timeValue.unit  = BSCD_TimerUnit_eETU;
		*/

		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

	}

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_P_WaitForTimerEvent(in_channelHandle, 1));


   	/* Disable timer */
	timer.bIsTimerInterruptEnable = false;
	timer.bIsTimerEnable = false;
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer));


	/* For T=1, we have to check the Block wait time */
	/* For T=0, we have to check the Work Wait Time.  */
	/* BSYT Issue: RC0 WWT timer could only check the interval
	  btw the leading edge of 2 consecutive characters sent
	  by the ICC.  We will use GP timer to check the interval
	  btw the leading edge of characters in opposite directions
	*/

	if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0) {

			/* Restore the original WWT */
			timer.bIsTimerInterruptEnable = true;
			timer.bIsTimerEnable = true;
			timer.eTimerType = BSCD_TimerType_eWaitTimer;
			timer.timerMode.eWaitTimerMode = BSCD_WaitTimerMode_eWorkWaitTime;
			timeValue.ulValue = in_channelHandle->currentChannelSettings.workWaitTime.ulValue;
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

	}

	else if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) {
			timer.bIsTimerInterruptEnable = true;
			timer.bIsTimerEnable = true;
			timer.eTimerType = BSCD_TimerType_eWaitTimer;
			timer.timerMode.eWaitTimerMode = BSCD_WaitTimerMode_eBlockWaitTime;

			if (in_channelHandle->currentChannelSettings.blockWaitTimeExt.ulValue == 0)
				timeValue.ulValue = in_channelHandle->currentChannelSettings.blockWaitTime.ulValue ;
			else
				timeValue.ulValue = in_channelHandle->currentChannelSettings.blockWaitTimeExt.ulValue;
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));
	}

	/*
	* Fill BCM FIFO with the request message.
	*/
	for (i = 0; i < in_ulNumXmitBytes; i++) {

		BKNI_RegWrite8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_TRANSMIT), (uint32_t) inp_ucXmitData[i]);
		BDBG_MSG("%02x ", inp_ucXmitData[i]);
	}
	BDBG_MSG("\n");

	/* Enable EDC */
	ulVal = BKNI_RegRead8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD));

	if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0)  {

		ulVal |= BCHP_SCA_SC_PROTO_CMD_rbuf_rst_MASK;
	}
	else {
		ulVal |= BCHP_SCA_SC_PROTO_CMD_rbuf_rst_MASK;
		if ((in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eES) ||
			(in_channelHandle->currentChannelSettings.bTPDU == true) ) /* This condition added for WHQL card 5 test */
		{
			/*
			 application computes its own LRC or CRC and appends it as the last byte
			*/
			ulVal &= ~BCHP_SCA_SC_PROTO_CMD_edc_en_MASK;
		}
		else {
			ulVal |= BCHP_SCA_SC_PROTO_CMD_edc_en_MASK;
			ulVal &= ~BCHP_SCA_SC_PROTO_CMD_crc_lrc_MASK;
		}
	}

	BKNI_RegWrite8(
			in_channelHandle,
			(in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD),
			ulVal);


	/* Set flow cmd */
	ulVal = BKNI_RegRead8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_FLOW_CMD));

	if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0)  {
		/*
		Only NDS support smartcard flow control. We MUST NOT set SC_FLOW_EN to 1 for
		other standards.
		*/
		if (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eNDS) {
			ulVal |= BCHP_SCA_SC_FLOW_CMD_flow_en_MASK;
		}
		else {
			ulVal &= ~BCHP_SCA_SC_FLOW_CMD_flow_en_MASK;
		}
	}
	else {
		/* No flow control for T=1 protocol or T=14. */
		ulVal &= ~BCHP_SCA_SC_FLOW_CMD_flow_en_MASK;
	}
	BKNI_RegWrite8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_FLOW_CMD),	ulVal);

	/* Ready to transmit */
	ulVal = BKNI_RegRead8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1));

	/* Always set auto receive */
	ulVal = BCHP_SCA_SC_UART_CMD_1_t_r_MASK | BCHP_SCA_SC_UART_CMD_1_xmit_go_MASK |
			 BCHP_SCA_SC_UART_CMD_1_io_en_MASK |BCHP_SCA_SC_UART_CMD_1_auto_rcv_MASK ;

	BKNI_RegWrite8(in_channelHandle,(in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1), ulVal);


#ifdef BSCD_EMV2000_CWT_PLUS_4
	in_channelHandle->bIsReceive = true;
#endif

	/*
	* Wait until the BCM sent all the data.
	*/
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_P_WaitForTDone(in_channelHandle));

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_T0T1Transmit);
	return( errCode );
}


BERR_Code BSCD_Channel_P_EnableInterrupts_isr(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_EnableInterrupts_isr\n"));

	if (in_channelHandle->bIsOpen == true) {
		/* Update BSCD_P_INTR_EN_1 and BSCD_P_INTR_EN_2 */
		if  ((in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0 ) &&
			(in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eIrdeto))	{

			/* Enable parity error re-transmission only in T=0 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eRetryInt,
				               		BSCD_Channel_P_RetryCB_isr));

			/* Enable RCV_INTR only in T=0 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eRcvInt,
				               		BSCD_Channel_P_RcvCB_isr));

			/* Enable RPAR_INTR only in T=0 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eRParityInt,
				               		BSCD_Channel_P_RParityCB_isr));

			/* Enable TPAR_INTR only in T=0 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eTParityInt,
				               		BSCD_Channel_P_TParityCB_isr));


		}


		else if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1 ) {  /* T=1 protocol */

			/* Enable cwt only in T=1 */
#ifdef BSCD_EMV2000_CWT_PLUS_4
			if  ( (in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV2000) ||
			  	(in_channelHandle->currentChannelSettings.eProtocolType != BSCD_AsyncProtocolType_e1)) {
#endif
				BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
							BSCD_Channel_EnableIntrCallback_isr (
								in_channelHandle, BSCD_IntType_eCWTInt,
					               		BSCD_Channel_P_CWTCB_isr));

#ifdef BSCD_EMV2000_CWT_PLUS_4
			}
#endif


#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			/* Enable BGT only in T=1 */
			if  (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000) {
				BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eEvent1Int,
				               		BSCD_Channel_P_Event1CB_isr));
			}

#endif

			/* Enable BGT only in T=1 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eBGTInt,
				               		BSCD_Channel_P_BGTCB_isr));

			/* Enable rlen only in T=1 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eRLenInt,
				               		BSCD_Channel_P_RLenCB_isr));

			/* Enable rreadyonly in T=1 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eRReadyInt,
				               		BSCD_Channel_P_RReadyCB_isr));

			/* set block guard time for T=1 only */
			/* Update the BSCD_P_BGT */
			BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_BGT),
				BCHP_SCA_SC_BGT_r2t_MASK| in_channelHandle->currentChannelSettings.blockGuardTime.ulValue);

/* Enable RCV_INTR only in T=1, EMV 2000 to resolve CWT+4 issue */
#ifdef BSCD_EMV2000_CWT_PLUS_4
			if ( (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eEMV2000) &&
		  		(in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1)) {

				BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eRcvInt,
				               		NULL));
			}
#endif
/* Enable RCV_INTR only in T=1, EMV 2000 to resolve CWT+4 issue */


		}
		else if (in_channelHandle->currentChannelSettings.scStandard ==  BSCD_Standard_eIrdeto) {  /* T=14 Irdeto  protocol */
			/* Enable RCV_INTR only in T=0 */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eRcvInt,
				               		BSCD_Channel_P_RcvCB_isr));
		}

		/* Keep the card insertion and removal interrrupt */
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eCardInsertInt,
				               		BSCD_Channel_P_CardInsertCB_isr));

		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eCardRemoveInt,
				               		BSCD_Channel_P_CardRemoveCB_isr));

		/* Enable tdone for T=0 and  T=1 */
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eTDoneInt,
				               		BSCD_Channel_P_TDoneCB_isr));

	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_EnableInterrupts_isr);
	return( errCode );

}


BERR_Code BSCD_Channel_P_SetStandard(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
)
{

	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_SetStandard\n"));

	/* Asynchronous Protocol Types. */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		((inp_sSettings->eProtocolType <= BSCD_AsyncProtocolType_eUnknown)  ||
		(inp_sSettings->eProtocolType > BSCD_AsyncProtocolType_e14_IRDETO)) );
	switch(in_channelHandle->currentChannelSettings.scStandard) {
		case BSCD_Standard_eNDS:  		/* NDS. T=0 with flow control. */
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							(inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;

		case BSCD_Standard_eISO:      		/* ISO 7816. T_0 or T=1*/
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							( (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ) &&
							 (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e1 )));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;

		case BSCD_Standard_eEMV1996:  		/* EMV. T=0 or T=1 */
		case BSCD_Standard_eEMV2000:  		/* EMV. T=0 or T=1 */
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							((inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ) &&
							 (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e1 )));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;

		case BSCD_Standard_eARIB:		/* ARIB. T=1 */
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							( (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ) &&
							 (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e1 )));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;

		case BSCD_Standard_eIrdeto:		/* Irdeto. T=14.  Need Major software workarouond to support this */
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							( (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ) &&
							 (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e14_IRDETO )));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;


		case BSCD_Standard_eES:             	/* ES, T=1.  Obsolete. Use ISO */
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							( (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ) &&
							 (inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e1 )));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;

		case BSCD_Standard_eMT:             	/* MT, T=0.  Obsolete. Use ISO */
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							((inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ) ));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;

		case BSCD_Standard_eConax:             	/* Conax, T=0.  Obsolete. Use ISO */
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
							((inp_sSettings->eProtocolType != BSCD_AsyncProtocolType_e0 ) ));
			in_channelHandle->currentChannelSettings.eProtocolType = inp_sSettings->eProtocolType;
			break;

		default:
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,  true);
			break;

	}
	BDBG_MSG("eProtocolType = %d\n", in_channelHandle->currentChannelSettings.eProtocolType);


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_SetStandard);
	return( errCode );
}


BERR_Code BSCD_Channel_P_SetFreq(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
)
{

	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_SetFreq\n"));

	/* Set F, 	Clock Rate Conversion Factor */
	if (inp_sSettings->ucFFactor == 0) {
		in_channelHandle->currentChannelSettings.ucFFactor = BSCD_DEFAULT_F;
	}
	else if ((inp_sSettings->ucFFactor >= 1 ) && (inp_sSettings->ucFFactor <= 13)){
		in_channelHandle->currentChannelSettings.ucFFactor = inp_sSettings->ucFFactor;
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	BDBG_MSG("ucFFactor = %d \n", in_channelHandle->currentChannelSettings.ucFFactor);

	/* Set D, 	Baud Rate Adjustor */
	if (inp_sSettings->ucDFactor == 0) {
		in_channelHandle->currentChannelSettings.ucDFactor = BSCD_DEFAULT_D;
	}
	if ((inp_sSettings->ucDFactor >= 1 ) && (inp_sSettings->ucDFactor <= 9)) {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(BSCD_P_GetISOBaudRateAdjustor(inp_sSettings->ucDFactor) ==  ((unsigned char ) -1) ) );
		in_channelHandle->currentChannelSettings.ucDFactor = inp_sSettings->ucDFactor;
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	BDBG_MSG("ucDFactor = %d \n", in_channelHandle->currentChannelSettings.ucDFactor);

	/* Set ETU Clock Divider */
	if (inp_sSettings->ucEtuClkDiv == 0 ) {
		in_channelHandle->currentChannelSettings.ucEtuClkDiv =
				BSCD_P_GetETUClkDiv(in_channelHandle->currentChannelSettings.ucDFactor,
									  in_channelHandle->currentChannelSettings.ucFFactor);
	}
	else if ( (inp_sSettings->ucEtuClkDiv == 1) || (inp_sSettings->ucEtuClkDiv == 2) ||
			(inp_sSettings->ucEtuClkDiv == 3) || (inp_sSettings->ucEtuClkDiv == 4) ||
			(inp_sSettings->ucEtuClkDiv == 5) || (inp_sSettings->ucEtuClkDiv == 6) ||
			(inp_sSettings->ucEtuClkDiv == 7) || (inp_sSettings->ucEtuClkDiv == 8)  ) {

		in_channelHandle->currentChannelSettings.ucEtuClkDiv = inp_sSettings->ucEtuClkDiv;
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	BDBG_MSG("ucEtuClkDiv = %d \n", in_channelHandle->currentChannelSettings.ucEtuClkDiv);

	/* Set SC Clock Divider */
	if (inp_sSettings->ucScClkDiv == 0 ) {
		in_channelHandle->currentChannelSettings.ucScClkDiv =
						BSCD_P_GetClkDiv(in_channelHandle->currentChannelSettings.ucDFactor,
										  in_channelHandle->currentChannelSettings.ucFFactor) ;
	}
	else if ( (inp_sSettings->ucScClkDiv == 1) || (inp_sSettings->ucScClkDiv == 2) ||
			(inp_sSettings->ucScClkDiv == 3) || (inp_sSettings->ucScClkDiv == 4) ||
			(inp_sSettings->ucScClkDiv == 5) || (inp_sSettings->ucScClkDiv == 8) ||
			(inp_sSettings->ucScClkDiv == 10) || (inp_sSettings->ucScClkDiv == 16)  ) {

		in_channelHandle->currentChannelSettings.ucScClkDiv = inp_sSettings->ucScClkDiv;
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	BDBG_MSG("ucScClkDiv = %d \n", in_channelHandle->currentChannelSettings.ucScClkDiv);

	/* Set external Clock Divisor.  For TDA only 1, 2,4,8 are valid value. */
	if (inp_sSettings->ucExternalClockDivisor == 0 ) {
		in_channelHandle->currentChannelSettings.ucExternalClockDivisor = BSCD_DEFAULT_EXTERNAL_CLOCK_DIVISOR;
	}
	else {
		in_channelHandle->currentChannelSettings.ucExternalClockDivisor = inp_sSettings->ucExternalClockDivisor;
	}
	BDBG_MSG("ucExternalClockDivisor = %d \n", in_channelHandle->currentChannelSettings.ucExternalClockDivisor);

	/* Set Prescale */
	if (inp_sSettings->unPrescale == 0 ) {
		in_channelHandle->currentChannelSettings.unPrescale =
		                   BSCD_P_GetPrescale(in_channelHandle->currentChannelSettings.ucDFactor,
                                              in_channelHandle->currentChannelSettings.ucFFactor) *
			in_channelHandle->currentChannelSettings.ucExternalClockDivisor +
			(in_channelHandle->currentChannelSettings.ucExternalClockDivisor - 1);
	}
	else if ( (inp_sSettings->unPrescale <= BSCD_MAX_PRESCALE)) {
		in_channelHandle->currentChannelSettings.unPrescale = inp_sSettings->unPrescale *
			in_channelHandle->currentChannelSettings.ucExternalClockDivisor +
			(in_channelHandle->currentChannelSettings.ucExternalClockDivisor - 1);
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	BDBG_MSG("unPrescale = %d \n", in_channelHandle->currentChannelSettings.unPrescale);

	/* Set baud divisor */
	if (inp_sSettings->ucBaudDiv == 0 ) {
		in_channelHandle->currentChannelSettings.ucBaudDiv = BSCD_DEFAULT_BAUD_DIV ;
	}
	else if ( (inp_sSettings->ucBaudDiv == 31) || (inp_sSettings->ucBaudDiv == 32) ||(inp_sSettings->ucBaudDiv == 25) ) {

		in_channelHandle->currentChannelSettings.ucBaudDiv = inp_sSettings->ucBaudDiv;
	}
	else {
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	BDBG_MSG("ucBaudDiv = %d \n", in_channelHandle->currentChannelSettings.ucBaudDiv);

	/* Set ICC CLK Freq */
	in_channelHandle->currentChannelSettings.currentICCClkFreq =
			in_channelHandle->moduleHandle->currentSettings.moduleClkFreq.ulClkFreq  /
					in_channelHandle->currentChannelSettings.ucScClkDiv /
					in_channelHandle->currentChannelSettings.ucEtuClkDiv/
					in_channelHandle->currentChannelSettings.ucExternalClockDivisor;
	BDBG_MSG("currentICCClkFreq = %d \n", in_channelHandle->currentChannelSettings.currentICCClkFreq);

	in_channelHandle->currentChannelSettings.currentBaudRate =
			in_channelHandle->moduleHandle->currentSettings.moduleClkFreq.ulClkFreq /
					in_channelHandle->currentChannelSettings.ucEtuClkDiv/
					(in_channelHandle->currentChannelSettings.unPrescale+1)/
					in_channelHandle->currentChannelSettings.ucBaudDiv;
	BDBG_MSG("currentBaudRate = %d \n", in_channelHandle->currentChannelSettings.currentBaudRate);

	if (in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eIrdeto) {
		BDBG_MSG("ISO currentBaudRate = %ld \n", in_channelHandle->currentChannelSettings.currentICCClkFreq *
			BSCD_P_GetISOBaudRateAdjustor(in_channelHandle->currentChannelSettings.ucDFactor) /
			BSCD_P_GetISOClockRateConversionFactor(in_channelHandle->currentChannelSettings.ucFFactor) );

		BDBG_MSG("ISOBaudRateAdjustor = %d \n", BSCD_P_GetISOBaudRateAdjustor(in_channelHandle->currentChannelSettings.ucDFactor ) );

		BDBG_MSG("ISOClockRateConversionFactor = %d \n", BSCD_P_GetISOClockRateConversionFactor(in_channelHandle->currentChannelSettings.ucFFactor) );


		/* If the final ISO baudrate is not equal to the final BRCM baudrate, there is a potential mismatch */
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
			(in_channelHandle->currentChannelSettings.currentBaudRate !=
			(in_channelHandle->currentChannelSettings.currentICCClkFreq *
			BSCD_P_GetISOBaudRateAdjustor(in_channelHandle->currentChannelSettings.ucDFactor) /
			BSCD_P_GetISOClockRateConversionFactor(in_channelHandle->currentChannelSettings.ucFFactor)) ));

	}
	else {
		/* For T=14 Irdeto */
		BDBG_MSG("ISO currentBaudRate = %d \n", in_channelHandle->currentChannelSettings.currentICCClkFreq / BSCD_T14_IRDETO_CONSTANT_CLOCK_RATE_CONV_FACTOR);

		/* If the final ISO baudrate is not equal to the final BRCM baudrate, there is a potential mismatch */
#ifndef A582_HAWK
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
			(in_channelHandle->currentChannelSettings.currentBaudRate !=
			(in_channelHandle->currentChannelSettings.currentICCClkFreq  /
			BSCD_T14_IRDETO_CONSTANT_CLOCK_RATE_CONV_FACTOR) ));
#endif
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_SetFreq);
	return( errCode );
}


BERR_Code BSCD_Channel_P_SetWaitTime(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
)
{

	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_SetWaitTime\n"));

	/* Set work waiting time */
	if (inp_sSettings->workWaitTime.ulValue == 0) {
		in_channelHandle->currentChannelSettings.workWaitTime.ulValue =  BSCD_DEFAULT_WORK_WAITING_TIME ;
		in_channelHandle->currentChannelSettings.workWaitTime.unit = BSCD_TimerUnit_eETU;
	}
	else {
		switch (inp_sSettings->workWaitTime.unit) {
			case BSCD_TimerUnit_eETU:
				in_channelHandle->currentChannelSettings.workWaitTime.ulValue =  inp_sSettings->workWaitTime.ulValue ;
				break;
			case BSCD_TimerUnit_eCLK:
				in_channelHandle->currentChannelSettings.workWaitTime.ulValue =
					inp_sSettings->workWaitTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/
					in_channelHandle->currentChannelSettings.currentICCClkFreq;
				break;
			case BSCD_TimerUnit_eMilliSec:
				in_channelHandle->currentChannelSettings.workWaitTime.ulValue =
					inp_sSettings->workWaitTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/1000;
				break;
			default:
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
				break;
		}
		in_channelHandle->currentChannelSettings.workWaitTime.unit =  BSCD_TimerUnit_eETU;
	}
	BDBG_MSG("workWaitTime.ulValue in ETU = %d \n", in_channelHandle->currentChannelSettings.workWaitTime.ulValue);
	BDBG_MSG("workWaitTime.unit = %d \n", in_channelHandle->currentChannelSettings.workWaitTime.unit);

	/* Set block Wait time */
	if (inp_sSettings->blockWaitTime.ulValue == 0) {
		in_channelHandle->currentChannelSettings.blockWaitTime.ulValue =  BSCD_DEFAULT_BLOCK_WAITING_TIME ;
		in_channelHandle->currentChannelSettings.blockWaitTime.unit = BSCD_TimerUnit_eETU;
	}
	else {
		switch (inp_sSettings->blockWaitTime.unit) {
			case BSCD_TimerUnit_eETU:
				in_channelHandle->currentChannelSettings.blockWaitTime.ulValue =  inp_sSettings->blockWaitTime.ulValue ;
				break;
			case BSCD_TimerUnit_eCLK:
				in_channelHandle->currentChannelSettings.blockWaitTime.ulValue =
					inp_sSettings->blockWaitTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/
					in_channelHandle->currentChannelSettings.currentICCClkFreq;
				break;
			case BSCD_TimerUnit_eMilliSec:
				in_channelHandle->currentChannelSettings.blockWaitTime.ulValue =
					inp_sSettings->blockWaitTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/
					1000;
				break;
			default:
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
				break;
		}
		in_channelHandle->currentChannelSettings.blockWaitTime.unit =  BSCD_TimerUnit_eETU;
	}
	BDBG_MSG("blockWaitTime.ulValue in ETU = %d \n", in_channelHandle->currentChannelSettings.blockWaitTime.ulValue);
	BDBG_MSG("blockWaitTime.unit = %d \n", in_channelHandle->currentChannelSettings.blockWaitTime.unit);

	/* Set block Wait time extension */
	in_channelHandle->currentChannelSettings.blockWaitTimeExt.ulValue =  inp_sSettings->blockWaitTimeExt.ulValue ;
	in_channelHandle->currentChannelSettings.blockWaitTimeExt.unit =  BSCD_TimerUnit_eETU;

	/* Set Character Waiting Time Integer */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(inp_sSettings->ulCharacterWaitTimeInteger > BSCD_MAX_CHARACTER_WAIT_TIME_INTEGER));
	in_channelHandle->currentChannelSettings.ulCharacterWaitTimeInteger =  inp_sSettings->ulCharacterWaitTimeInteger ;
	BDBG_MSG("ulCharacterWaitTimeInteger = %d \n", in_channelHandle->currentChannelSettings.ulCharacterWaitTimeInteger);



BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_SetWaitTime);
	return( errCode );
}


BERR_Code BSCD_Channel_P_SetGuardTime(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
)
{

	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_SetGuardTime\n"));

	/* Set Extra Guard Time  */
	switch (inp_sSettings->extraGuardTime.unit) {
		case BSCD_TimerUnit_eETU:
			in_channelHandle->currentChannelSettings.extraGuardTime.ulValue =  inp_sSettings->extraGuardTime.ulValue ;
			break;
		case BSCD_TimerUnit_eCLK:
			in_channelHandle->currentChannelSettings.extraGuardTime.ulValue =
				inp_sSettings->extraGuardTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/
				in_channelHandle->currentChannelSettings.currentICCClkFreq;
			break;
		case BSCD_TimerUnit_eMilliSec:
			in_channelHandle->currentChannelSettings.extraGuardTime.ulValue =
				inp_sSettings->extraGuardTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/
				1000;
			break;
		default:
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
			break;
	}
	in_channelHandle->currentChannelSettings.extraGuardTime.unit =  BSCD_TimerUnit_eETU;

	BDBG_MSG("extraGuardTime.ulValue in ETU = %d \n", in_channelHandle->currentChannelSettings.extraGuardTime.ulValue);
	BDBG_MSG("extraGuardTime.unit = %d \n", in_channelHandle->currentChannelSettings.extraGuardTime.unit);

	/* Set block Guard time */
	if (inp_sSettings->blockGuardTime.ulValue == 0) {
		in_channelHandle->currentChannelSettings.blockGuardTime.ulValue =  BSCD_DEFAULT_BLOCK_GUARD_TIME ;
		in_channelHandle->currentChannelSettings.blockGuardTime.unit = BSCD_TimerUnit_eETU;
	}
	else {
		switch (inp_sSettings->blockGuardTime.unit) {
			case BSCD_TimerUnit_eETU:
				in_channelHandle->currentChannelSettings.blockGuardTime.ulValue =  inp_sSettings->blockGuardTime.ulValue ;
				break;
			case BSCD_TimerUnit_eCLK:
				in_channelHandle->currentChannelSettings.blockGuardTime.ulValue =
					inp_sSettings->blockGuardTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/
					in_channelHandle->currentChannelSettings.currentICCClkFreq;
				break;
			case BSCD_TimerUnit_eMilliSec:
				in_channelHandle->currentChannelSettings.blockGuardTime.ulValue =
					inp_sSettings->blockGuardTime.ulValue*in_channelHandle->currentChannelSettings.currentBaudRate/
					1000;
				break;
			default:
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
				break;
		}
		in_channelHandle->currentChannelSettings.blockGuardTime.unit =  BSCD_TimerUnit_eETU;
	}
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
			((in_channelHandle->currentChannelSettings.blockGuardTime.ulValue > BSCD_MAX_BLOCK_GUARD_TIME) ||
			(in_channelHandle->currentChannelSettings.blockGuardTime.ulValue < BSCD_MIN_BLOCK_GUARD_TIME)) );
	BDBG_MSG("blockGuardTime.ulValue in ETU = %d \n", in_channelHandle->currentChannelSettings.blockGuardTime.ulValue);
	BDBG_MSG("blockGuardTime.unit = %d \n", in_channelHandle->currentChannelSettings.blockGuardTime.unit);


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_SetGuardTime);
	return( errCode );
}

BERR_Code BSCD_Channel_P_SetTransactionTimeout(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
)
{

	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_SetTransactionTimeout\n"));

	if (inp_sSettings->timeOut.ulValue == 0) {
		in_channelHandle->currentChannelSettings.timeOut.ulValue =  BSCD_DEFAULT_TIME_OUT ;
		in_channelHandle->currentChannelSettings.timeOut.unit = BSCD_TimerUnit_eMilliSec;
	}
	else {
		switch (inp_sSettings->timeOut.unit) {
			case BSCD_TimerUnit_eETU:
				in_channelHandle->currentChannelSettings.timeOut.ulValue =  inp_sSettings->timeOut.ulValue * 1000000 /
					in_channelHandle->currentChannelSettings.currentBaudRate;
				break;
			case BSCD_TimerUnit_eCLK:
				in_channelHandle->currentChannelSettings.timeOut.ulValue =
					inp_sSettings->timeOut.ulValue * 1000000 /
					in_channelHandle->currentChannelSettings.currentICCClkFreq ;
				break;
			case BSCD_TimerUnit_eMilliSec:
				in_channelHandle->currentChannelSettings.timeOut.ulValue =  inp_sSettings->timeOut.ulValue ;
				break;
			default:
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
				break;
		}
		in_channelHandle->currentChannelSettings.timeOut.unit =  BSCD_TimerUnit_eMilliSec;
	}
	BDBG_MSG("timeOut.ulValue in milliseconds = %d \n", in_channelHandle->currentChannelSettings.timeOut.ulValue);
	BDBG_MSG("timeOut.unit = %d \n", in_channelHandle->currentChannelSettings.timeOut.unit);


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_SetTransactionTimeout);
	return( errCode );
}

BERR_Code BSCD_Channel_P_SetEdcParity(
		BSCD_ChannelHandle	    in_channelHandle,
		const BSCD_ChannelSettings  *inp_sSettings
)
{

	BERR_Code	errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_P_SetEdcParity\n"));

	/* Set Number of transmit parity retries */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(inp_sSettings->ucTxRetries > BSCD_MAX_TX_PARITY_RETRIES));
	in_channelHandle->currentChannelSettings.ucTxRetries =  inp_sSettings->ucTxRetries ;
	BDBG_MSG("ucTxRetries = %d\n", in_channelHandle->currentChannelSettings.ucTxRetries);

	/* Set Number of receive parity retries */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(inp_sSettings->ucRxRetries > BSCD_MAX_TX_PARITY_RETRIES));
	in_channelHandle->currentChannelSettings.ucRxRetries =  inp_sSettings->ucRxRetries ;
	BDBG_MSG("ucRxRetries = %d\n", in_channelHandle->currentChannelSettings.ucRxRetries);

	/* Set EDC encoding */
	in_channelHandle->currentChannelSettings.edcSetting.bIsEnabled =  inp_sSettings->edcSetting.bIsEnabled;
	in_channelHandle->currentChannelSettings.edcSetting.edcEncode =  inp_sSettings->edcSetting.edcEncode;

	BDBG_MSG("edcSetting.bIsEnabled = %d\n", in_channelHandle->currentChannelSettings.edcSetting.bIsEnabled );
	BDBG_MSG("edcSetting.edcEncode = %d\n", in_channelHandle->currentChannelSettings.edcSetting.edcEncode);


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_SetEdcParity);
	return( errCode );
}
