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

#ifndef BERR_ID_H__
#define BERR_ID_H__

/* The standard ID. This ID is present when the standard error codes are
   being used without a module specific ID */
#define BERR_STD_ID     0x00

/* The application ID. This ID is reserved for use by top level applications
   so they can extend our error codes if they wish. This ID cannot be used
   in any module or library */
#define BERR_APP_ID     0x01

/* porting interfaces */
#define BERR_AUD_ID     0x02
#define BERR_HDM_ID     0x03
#define BERR_ICM_ID     0x04
#define BERR_IFD_ID     0x05
#define BERR_MVD_ID     0x06
#define BERR_QDS_ID     0x07
#define BERR_QOB_ID     0x08
#define BERR_QUS_ID     0x09
#define BERR_RDC_ID     0x0A
#define BERR_RFM_ID     0x0B
#define BERR_SCD_ID     0x0C
#define BERR_TNR_ID     0x0D
#define BERR_VBI_ID     0x0E
#define BERR_VDC_ID     0x0F
#define BERR_XPT_ID     0x10
#define BERR_I2C_ID     0x11
#define BERR_SPI_ID     0x12
#define BERR_ICP_ID     0x13
#define BERR_IRB_ID     0x14
#define BERR_KIR_ID     0x15
#define BERR_KPD_ID     0x16
#define BERR_LED_ID     0x17
#define BERR_PWM_ID     0x18
#define BERR_URT_ID     0x19
#define BERR_SDS_ID     0x1A
#define BERR_VSB_ID     0x1B
#define BERR_ENC_ID     0x1C
#define BERR_DMA_ID     0x1D
#define BERR_GIO_ID     0x1E
#define BERR_LNA_ID     0x1F
#define BERR_GRC_ID     0x20
#define BERR_P3D_ID     0x21
#define BERR_XVD_ID     0x22
#define BERR_ARC_ID     0x23
#define BERR_RAP_ID     0x24
#define BERR_MRC_ID     0x25
#define BERR_AST_ID     0x26
#define BERR_TMR_ID     0x27
#define BERR_RPC_ID     0x28
#define BERR_MEM_ID     0x29
#define BERR_INT_ID     0x30
#define BERR_ADS_ID     0x31
#define BERR_AOB_ID     0x32
#define BERR_AUS_ID     0x33

/* syslibs */
#define BERR_TNRlib_ID  0x100
#define BERR_VBIlib_ID  0x101
#define BERR_BTSlib_ID  0x102

#endif /* #ifndef BERR_ID_H__ */

/* end of file */
