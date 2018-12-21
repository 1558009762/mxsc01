/*****************************************************************************
* Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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
#include <linux/kernel.h>
#include <bstd.h>
#include "bkni.h"
#include "bchp_sca.h"
#include "bscd.h"
#include "bscd_priv.h"
#include "bstd_defs.h"
#include "bint.h"
#include "iproc_sci_debug.h"

void iproc_sci_regdump(unsigned long reg_base, char *heading)
{
    uint32_t ulValue;

    printk(KERN_ERR "\n********** SC at 0x%lx:%s **********\n",reg_base,heading);

    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_UART_CMD_1); 
    printk(KERN_ERR "SMART CARD UART COMMAND REGISTER 1 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_UART_CMD_2); 
    printk(KERN_ERR "SMART CARD UART COMMAND REGISTER 2 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_PROTO_CMD); 
    printk(KERN_ERR "SMART CARD PROTOCOL COMMAND REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_FLOW_CMD); 
    printk(KERN_ERR "SMART CARD FLOW CONTROL COMMAND REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_IF_CMD_1); 
    printk(KERN_ERR "SMART CARD INTERFACE COMMAND REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_IF_CMD_2);
    printk(KERN_ERR "SMART CARD INTERFACE COMMAND REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_INTR_STAT_1);
    printk(KERN_ERR "SMART CARD INTERRUPT STATUS REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_INTR_STAT_2);
    printk(KERN_ERR "SMART CARD INTERRUPT STATUS REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_INTR_EN_1);
    printk(KERN_ERR "SMART CARD INTERRUPT ENABLE REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_INTR_EN_2);
    printk(KERN_ERR "SMART CARD INTERRUPT ENABLE REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_CLK_CMD);
    printk(KERN_ERR "SMART CARD CLOCK COMMAND 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_PRESCALE);
    printk(KERN_ERR "SMART CARD CLOCK PRESCALE 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TIMER_CMD);
    printk(KERN_ERR "SMART CARD TIMER COMMAND REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_BGT);
    printk(KERN_ERR "SMART CARD BLOCK GUARD TIME REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TIMER_CNT_1);
    printk(KERN_ERR "SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TIMER_CNT_2);
    printk(KERN_ERR "SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TIMER_CMP_1);
    printk(KERN_ERR "SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TIMER_CMP_2);
    printk(KERN_ERR "SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_WAIT_1);
    printk(KERN_ERR "SMART CARD WAITING TIMER REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_WAIT_2);
    printk(KERN_ERR "SMART CARD WAITING TIMER REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_WAIT_3);
    printk(KERN_ERR "SMART CARD WAITING TIMER REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TGUARD);
    printk(KERN_ERR "SMART CARD TRANSMIT GUARD TIME REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TRANSMIT);
    printk(KERN_ERR "SMART CARD TRANSMIT REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_RECEIVE);
    printk(KERN_ERR "SMART CARD RECEIVE REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_STATUS_1);
    printk(KERN_ERR "SMART CARD STATUS 1 REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_STATUS_2);
    printk(KERN_ERR "SMART CARD STATUS 2 REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TLEN_2);
    printk(KERN_ERR "SMART CARD TRANSMIT LENGTH REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_TLEN_1);
    printk(KERN_ERR "SMART CARD TRANSMIT LENGTH REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_RLEN_2);
    printk(KERN_ERR "SMART CARD RECEIVE LENGTH REGISTER 0x%02X\n",ulValue & 0xFF);
    ulValue = BKNI_RegRead8(0, reg_base + BSCD_P_RLEN_1);
    printk(KERN_ERR "SMART CARD RECEIVE LENGTH REGISTER 0x%02X\n",ulValue & 0xFF);
    printk(KERN_ERR "***********************************************\n");
}


static char *BDBG_GetFileNameFromPath(const char *path)
{
	char *ptr = (char *)path;
	
	while (*ptr != '\0')
		ptr++;
	
	while (*ptr != '/')
	{
		if(ptr==path)
			return ptr;
		
		 ptr--;
	}
	
	return (ptr+1);
}

void BDBG_EnterFunction(BDBG_pDebugModuleFile dbg_module, const char *function, const char *filename, unsigned LineNo)
{
	printk("%s,%u:%s()\n", BDBG_GetFileNameFromPath(filename), LineNo, function);
}

void BDBG_LeaveFunction(BDBG_pDebugModuleFile dbg_module, const char *function, const char *filename, unsigned LineNo)
{
	printk("%s,%u:%s()\n", BDBG_GetFileNameFromPath(filename), LineNo, function);
}

BERR_Code BDBG_P_PrintError(const char *file, unsigned lineno, const char *error, BERR_Code error_no)
{
	printk("%s,%u,err=%u:%s\n", BDBG_GetFileNameFromPath(file), lineno, error_no, error);
	return BERR_SUCCESS;
}

void BDBG_P_PrintString(const char *fmt, ...)
{
	printk("%s\n", fmt);
}

int BDBG_P_TestAndPrintHeader(BDBG_Level level, BDBG_pDebugModuleFile dbg_module)
{
	return BERR_SUCCESS;
}

void BDBG_P_AssertFailed(const char *expr, const char *file, unsigned line)
{
	printk("%s,%u:%s\n", BDBG_GetFileNameFromPath(file), line, expr);
}
