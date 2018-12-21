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


/* ---- Include Files ---------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/i2c.h>

#include <bstd.h>
#include "bkni.h"
#include "bchp_sca.h"
#include "bscd.h"
#include "bscd_priv.h"
#include "bscd_isopriv.h"

BDBG_MODULE(BSCD);

/* Debug trace */
#define I2C_ENABLE_LOG           0

#if I2C_ENABLE_LOG
#define I2CLOG(f, ...)           printk(KERN_INFO f, ##__VA_ARGS__)
#else
#define I2CLOG(f, ...)
#endif

extern int nxp8026_i2c_init(void);
extern int nxp8026_i2c_remove(void);

BERR_Code NXP8026_Init(void);
BERR_Code NXP8026_Remove(void);
int NXP8026_Get_Product_Version(void);
int NXP8026_Get_Card_Status(void);
int NXP8026_Get_Card_Presence(void);
int NXP8026_Get_Card_ATR_Error(void);
BERR_Code NXP8026_Set_Vcc(bool bPower);
BERR_Code NXP8026_Set_Vcc_Level(BSCD_VccLevel in_vccLevel);
BERR_Code NXP8026_Activate(void);
BERR_Code NXP8026_Deactivate(void);


extern struct i2c_client *coupler_clients[3];
#define Addr_CSb coupler_clients[0]
#define Addr_Reg0 coupler_clients[1]
#define Addr_Reg1 coupler_clients[2]

/* register bit definition */
#define REG0_WR_VCC1V8    0x80
#define REG0_WR_IOEN      0x40
#define REG0_WR_REG01_00  0x00
#define REG0_WR_VCC3V     0x00
#define REG0_WR_VCC5V     0x04
#define REG0_WR_WARM      0x02
#define REG0_WR_START     0x01

#define REG0_RD_ACTIVE 0x80
#define REG0_RD_EARLY  0x40
#define REG0_RD_MUTE   0x20
#define REG0_RD_PRESL  0x02
#define REG0_RD_PRES   0x01

#define REG1_RESTIN           0x40 /* this bit indicates async or sync during activation sequence */
#define REG1_CLKPD_STOPPED0   0x00
#define REG1_CLKPD_STOPPED1   0x04
#define REG1_CLKPD_FINT_DIV2  0x08
#define REG1_CLKPD_CLKIN_DIV  0x0C
#define REG1_CLKDIV_1         0x00
#define REG1_CLKDIV_2         0x01
#define REG1_CLKDIV_4         0x02
#define REG1_CLKDIV_5         0x03


/* global */
BSCD_VccLevel   g_vccLevel;

/****************************************************************************
*
*  NXP8026_Init
*
***************************************************************************/
BERR_Code NXP8026_Init(void)
{
	uint i = 0;

	nxp8026_i2c_init();

	/* Set CLKDIV values */
	i2c_smbus_write_byte(Addr_CSb, 1); /* select slot 1 */
	i2c_smbus_write_byte(Addr_Reg0, g_vccLevel | REG0_WR_REG01_00); /* REG01 = 0b00 */
	i2c_smbus_write_byte(Addr_Reg1, REG1_RESTIN | REG1_CLKPD_CLKIN_DIV | REG1_CLKDIV_1); /* Set to async mode and clk div 1 */
	i = i2c_smbus_read_byte(Addr_Reg1);
	I2CLOG("NXP8026 REG1 read after write: %x\n", i);

//	NXP8026_Get_Card_Status();

	/* do it again to make sure SUPL bit is cleared */
//	NXP8026_Get_Card_Status();

	return BERR_SUCCESS;
}

/****************************************************************************
*
*  NXP8026_Remove
*
***************************************************************************/
BERR_Code NXP8026_Remove(void)
{
	nxp8026_i2c_remove();

	return BERR_SUCCESS;
}

/****************************************************************************
*
*  NXP8026_Get_Product_Version
*
***************************************************************************/
int NXP8026_Get_Product_Version(void)
{
	uint8_t  i = 0;

	/* get product version */
	i2c_smbus_write_byte(Addr_CSb, 0);
	i = i2c_smbus_read_byte(Addr_Reg0);

	return i;
}

/****************************************************************************
*
*  NXP8026_Get_Card_Status
*
***************************************************************************/
int NXP8026_Get_Card_Status(void)
{
	uint8_t  i;

	i2c_smbus_write_byte(Addr_CSb, 1); /* set for slot 1 */
	i = i2c_smbus_read_byte(Addr_Reg0);

	I2CLOG("Card status read: %x\n", i);

	return i;
}

/****************************************************************************
*
*  NXP8026_Get_Card_Presence
*
***************************************************************************/
int NXP8026_Get_Card_Presence(void)
{
	uint8_t  i;

	i = NXP8026_Get_Card_Status();

	return (i & REG0_RD_PRES);
}

/****************************************************************************
*
*  NXP8026_Get_Card_ATR_Error
*
***************************************************************************/
int NXP8026_Get_Card_ATR_Error(void)
{
	uint8_t  i;

	i = NXP8026_Get_Card_Status();

	return (i & (REG0_RD_EARLY | REG0_RD_MUTE));
}

/****************************************************************************
*
*  NXP8026_Set_Vcc
*
***************************************************************************/
/* slot 1 only for now */
BERR_Code NXP8026_Set_Vcc(bool bPower)
{
	/*
	don't do anything here since Vcc is auto controlled by Act/Deact
	*/

	return BERR_SUCCESS;
}


/****************************************************************************
*
*  NXP8026_Set_Vcc_Level
*
***************************************************************************/
BERR_Code NXP8026_Set_Vcc_Level(BSCD_VccLevel in_vccLevel)
{
	BERR_Code errCode = BERR_SUCCESS;

	/* We only remember it, it will be set together with other field */
	switch (in_vccLevel) {
		case BSCD_VccLevel_e3V:
			g_vccLevel = REG0_WR_VCC3V;
			break;

		case BSCD_VccLevel_e5V:
			g_vccLevel = REG0_WR_VCC5V;
			break;

		case BSCD_VccLevel_e18V:
			g_vccLevel = REG0_WR_VCC1V8;
			break;

		default:
			errCode = BSCD_STATUS_FAILED;
			BDBG_ERR(("BSCD_Channel_SetVccLevel: Do not support VCC Level switch = 0x%x, \n", in_vccLevel));
	}

	return errCode;
}

/****************************************************************************
*
*  NXP8026_Activate
*
***************************************************************************/
BERR_Code NXP8026_Activate(void)
{
	i2c_smbus_write_byte(Addr_CSb, 1);
	i2c_smbus_write_byte(Addr_Reg0, g_vccLevel | REG0_WR_IOEN | REG0_WR_START);

	return BERR_SUCCESS;
}

/****************************************************************************
*
*  NXP8026_Deactivate
*
***************************************************************************/
BERR_Code NXP8026_Deactivate(void)
{
	i2c_smbus_write_byte(Addr_CSb, 1);
	i2c_smbus_write_byte(Addr_Reg0, g_vccLevel );

	return BERR_SUCCESS;
}


/****************************************************************************
*
*  NXP8026_WarmReset
*
***************************************************************************/
BERR_Code NXP8026_WarmReset(void)
{
	i2c_smbus_write_byte(Addr_CSb, 1);
	i2c_smbus_write_byte(Addr_Reg0, g_vccLevel | REG0_WR_IOEN | REG0_WR_WARM);

	return BERR_SUCCESS;
}
