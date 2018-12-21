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

#include <linux/kernel.h>
#include <bstd.h>
#include "bkni.h"
#include "bchp_sca.h"
#include "bscd.h"
#include "bscd_priv.h"
#include "bint.h"
#include "iproc_sci_debug.h"

BDBG_MODULE(BSCD);

//#define SCI_DEBUG 0 /* define this in Makefile */

#undef  BDBG_MSG
#undef  BDBG_ALERT
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

extern BERR_Code NXP8026_Init(void);
extern BERR_Code NXP8026_Remove(void);
extern BERR_Code NXP8026_Set_Vcc(bool bPower);
extern BERR_Code NXP8026_Set_Vcc_Level(BSCD_VccLevel in_vccLevel);
extern BERR_Code NXP8026_Activate(void);
extern BERR_Code NXP8026_Deactivate(void);
extern BERR_Code NXP8026_WarmReset(void);
extern int NXP8026_Get_Card_Presence(void);


/*******************************************************************************
*	Default Module and Channel Settings.  Note that we could only modify
*	Module settings during BSCD_Open.
*******************************************************************************/
static const BSCD_Settings BSCD_defScdSettings =
{
	/* This attribute indicates the source of clock and the value */
	{BSCD_ClockFreqSrc_eInternalClock, BSCD_INTERNAL_CLOCK_FREQ},   /* Change from 27000000 for Cygnus */

	/* maximum SCD channels supported */
	BSCD_MAX_SUPPOTED_CHANNELS,

};

static const BSCD_ChannelSettings BSCD_defScdChannelSettings =
{
		/* Smart Card Standard */
		.scStandard = BSCD_Standard_eISO,

		/* Asynchronous Protocol Types. */
		.eProtocolType = BSCD_AsyncProtocolType_e0,

		/* Smart Card Types. */
		.ctxCardType = {BSCD_CardType_eUnknown, BSCD_VccLevel_e5V},

		/* This read-only attribute specifies the default
			source clock frequency in Hz. */
		.srcClkFreqInHz = BSCD_INTERNAL_CLOCK_FREQ,

		/* ICC CLK frequency in Hz which is
			source freq / SC_CLK_CMD[etu_clkdiv] / SC_CLK_CMD[sc_clkdiv] */
		.currentICCClkFreq = BSCD_INTERNAL_CLOCK_FREQ/BSCD_DEFAULT_ETU_CLKDIV/BSCD_DEFAULT_SC_CLKDIV,

		/* ETU in microseconds which is source freq / SC_CLK_CMD[etu_clkdiv] */
			/* (SC_PRESCALE * external_clock_div + (external_clock_div - 1))  */
		.currentBaudRate = BSCD_INTERNAL_CLOCK_FREQ/BSCD_DEFAULT_ETU_CLKDIV/(BSCD_DEFAULT_PRESCALE+1)/BSCD_DEFAULT_BAUD_DIV,

		/* This read-only attribute specifies the maximum IFSD.
			Should be 264. */
		.unMaxIFSD = BSCD_MAX_TX_SIZE,

		/* This attribute indicates the current IFSD */
		.unCurrentIFSD = BSCD_DEFAULT_EMV_INFORMATION_FIELD_SIZE,

		/* Clock Rate Conversion Factor,
			F in 1,2,3,4,5,6,9, 10, 11, 12 or 13.
			Default is 1. */
		.ucFFactor = BSCD_DEFAULT_F,

		/* Baud Rate Adjustment Factor,
			D in 1,2,3,4,5,6,8 or 9.
			Default is 1. */
		.ucDFactor = BSCD_DEFAULT_D,

		/* 	ETU Clock Divider in
			SC_CLK_CMD register. Valid value is
			from 1 to 8. Default is 6. */
		.ucEtuClkDiv = BSCD_DEFAULT_ETU_CLKDIV,

		/* 	SC Clock Divider in
			SC_CLK_CMD register. Valid value is
			1,2,3,4,5,8,10,16. Default is 1. */
		.ucScClkDiv = BSCD_DEFAULT_SC_CLKDIV,

		/* Prescale Value */
		.unPrescale = BSCD_DEFAULT_PRESCALE,

		/* external clock divisor */
		.ucExternalClockDivisor = BSCD_DEFAULT_EXTERNAL_CLOCK_DIVISOR,

		/* Baud Divisor	*/
		.ucBaudDiv = BSCD_DEFAULT_BAUD_DIV,

		/* Number of transmit parity retries per character in
			SC_UART_CMD_2 register.	Default is 4 and max is 6.
			7 indicates infinite retries */
		.ucTxRetries = BSCD_DEFAULT_TX_PARITY_RETRIES,

		/* Number of receive parity retries per character in
			SC_UART_CMD_2 register. Default is 4 and max is 6.
			7 indicates infinite retries */
		.ucRxRetries = BSCD_DEFAULT_RX_PARITY_RETRIES,

		/* work waiting time in SC_TIME_CMD register. Other than EMV
			standard, only valid if current protocol is T=0. */
		.workWaitTime = {BSCD_DEFAULT_WORK_WAITING_TIME,   BSCD_TimerUnit_eETU},

		/* block Wait time in SC_TIME_CMD register. Only valid if
			current protocol is T=1. */
		.blockWaitTime = {BSCD_DEFAULT_BLOCK_WAITING_TIME,   BSCD_TimerUnit_eETU},

		/* Extra Guard Time in SC_TGUARD register. */
		.extraGuardTime = { BSCD_DEFAULT_EXTRA_GUARD_TIME,   BSCD_TimerUnit_eETU},

		/*  block Guard time in SC_BGT register.Other than EMV
			standard, only valid if current protocol is T=1.  */
		.blockGuardTime = {BSCD_DEFAULT_BLOCK_GUARD_TIME,   BSCD_TimerUnit_eETU},

		/* character Wait time in SC_PROTO_CMD register. Only valid
			if current protocol is T=1. */
		.ulCharacterWaitTimeInteger = BSCD_DEFAULT_CHARACTER_WAIT_TIME_INTEGER,

		/* EDC encoding. Only valid if current protocol is T=1. */
		.edcSetting = {BSCD_EDCEncode_eLRC,   false},

		/* arbitrary Time Out value for any synchronous transaction. */
		.timeOut = {BSCD_DEFAULT_TIME_OUT,   BSCD_TimerUnit_eMilliSec},

		/* Specify if we need auto deactivation sequence */
		.bAutoDeactiveReq = false,

		/* True if we receive 0x60 in T=0, we will ignore it.  Otherwise, we treat 0x60 as a valid data byte */
		.bNullFilter = false,

		/* Debounce info for IF_CMD_2 */
		.scPresDbInfo = {BSCD_ScPresMode_eMask, true, BSCD_DEFAULT_DB_WIDTH},

		.resetCardAction = BSCD_ResetCardAction_eReceiveAndDecode, /* Tell driver whether to read/decode/program registers or not*/

		.blockWaitTimeExt = {0,   BSCD_TimerUnit_eETU},  /* block wait time extension */

		.bTPDU = false,                       /* IS packet in TPDU? */

		.unCurrentIFSC = BSCD_DEFAULT_IFSC

};

/*******************************************************************************
*	Default historical bytes for various card types
*******************************************************************************/
#define COUNTOF(ary)   ((int) (sizeof (ary) / sizeof ((ary)[0])))

static const BSCD_CardType_Settings BSCD_defCardTypeSettings[] =
{
    {BSCD_CardType_eJAVA, 5,
     {0xae, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    {BSCD_CardType_eJAVA, 13,
     {0x80, 0x31, 0x80, 0x65, 0xb0, 0x83, 0x02, 0x04, 0x7e, 0x83, 0x00, 0x90, 0x00, 0x00, 0x00}
    },

    {BSCD_CardType_ePKI, 10,
     {0x4a, 0x43, 0x4f, 0x50, 0x34, 0x31, 0x56, 0x32, 0x32, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    {BSCD_CardType_eACOS, 3,
    /*
     *  T1    T2    T3    T4    T5    T6    T7    T8    T9   T10   T11   T12   T13   T14   T15
     *
     * ACOS |Ver. |Rev. |Option Registers|Personalization File bytes | Stage | --  | --  | N/A
     */
     {0x41, 0x01, 0x38, 0x01, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x90, 0x00, 0x00}
    }
};

/*******************************************************************************
*	Public Module Functions
*******************************************************************************/
BERR_Code BSCD_Channel_APDU_Transceive(
                BSCD_ChannelHandle         in_channelHandle,
                uint8_t                    *inp_ucXmitData,
                unsigned long              in_ulNumXmitBytes,
                uint8_t                    *outp_ucRcvData,
                unsigned long              *outp_ulNumRcvBytes,
                unsigned long              in_ulMaxReadBytes
)
{
       BERR_Code errCode = BERR_SUCCESS;
       apdu_t apdu;
       uint8_t *data = inp_ucXmitData;
       uint16_t cse;
       uint8_t cmd[5], ack;
       unsigned long ulActualRxLen;
       uint8_t wtx = 0;
       uint8_t ucTxRxBuf[T1_MAX_BUF_SIZE];
       uint32_t len = 0;
       uint32_t bZero256 = 0; /* flag to indicate if this is Le=0 => 256 case */
       BSCD_TimerValue timeValue= {BSCD_DEFAULT_BLOCK_WAITING_TIME, BSCD_TimerUnit_eETU};

#ifdef SCI_DEBUG
	uint32_t ii;

	printk(KERN_ERR "BSCD TX %ld bytes: ", in_ulNumXmitBytes);
	for (ii=0; ii < in_ulNumXmitBytes; ii++)
		printk(KERN_ERR "%x ", inp_ucXmitData[ii]);
	printk(KERN_ERR "\n");
#endif

       BDBG_ENTER(("BSCD_Channel_APDU_Transceive\n"));

       BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
            (in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER));

       *outp_ulNumRcvBytes = 0;

       /* Check minium APDU command/response size */
       if ((in_ulNumXmitBytes < 4) || (in_ulNumXmitBytes < 2))
           return (BERR_INVALID_PARAMETER);

       if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0 ) { /* APDU throught T= 0 */

           /* Parse APDU command buffer */
           BKNI_Memset(&apdu, 0, sizeof(apdu_t));
           apdu.data = outp_ucRcvData;
           apdu.cla = *data++;
           apdu.ins = *data++;
           apdu.p1 = *data++;
           apdu.p2 = *data++;

           /* Check the APDU command cases */
           if (in_ulNumXmitBytes < 5) { /* case 1: Header only */
               cse = 1;
           } else if (in_ulNumXmitBytes == 5) { /* case 2: Header + Le */
               cse = 2;
               apdu.le = (inp_ucXmitData[4]);
               if (apdu.le == 0) {
                   bZero256 = 1; /* jTOP PIV T=0 card sends "0 c0 0 0 0", we will fail this frame */
			   }
           } else {
               apdu.lc = inp_ucXmitData[4];
               if (in_ulNumXmitBytes == (apdu.lc + 5)) { /* case 3: Header + Lc + data */
                   cse = 3;
               } else if (in_ulNumXmitBytes == (apdu.lc + 6)) { /* case 4: Header + Lc + data + Le */
                   cse = 4;
                   apdu.le = inp_ucXmitData[in_ulNumXmitBytes - 1];
                   if (apdu.le == 0)
                       bZero256 = 1;
               } else {
                   return (BERR_INVALID_PARAMETER); /* invalid APDU command */
               }
           }

           /* Check if receive buffer is big enough */
           if (in_ulMaxReadBytes < apdu.le + 2) {
               return (BERR_INVALID_PARAMETER);
           }

           /* clear return data (le bytes) */
           /* in case of card reponse with last command result, in case it didn't finish last command (due to us timed out) */
           BKNI_Memset(apdu.data, 0, apdu.le);

           /* Send 5-byte command header */
           if (cse == 1) BKNI_Memset(cmd, 0, 5);
           BKNI_Memcpy(cmd, inp_ucXmitData,  (cse == 1) ? 4 : 5);
           BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                BSCD_Channel_Transmit(in_channelHandle, cmd, 5));

           while (in_channelHandle->channelStatus.bCardPresent == true) {

             BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                  BSCD_Channel_Receive(in_channelHandle, &ack, &ulActualRxLen, 1));

             /* If receive null value '60' - wait a little */
             if (ack == 0x60) {
                 /* Send WTX indication to pc host */
                 /* Do not need this now since there's no PC in 5892
                 ccidctSendWTXIndication();
                 */
                 continue;
             }

             /* Check if it's SW1 || SW2 */
             if ((ack & 0xf0) == 0x60 || (ack & 0xf0) == 0x90) {
                 apdu.sw[0] = ack;
                 BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                      BSCD_Channel_Receive(in_channelHandle, &ack, &ulActualRxLen, 1)); /* get SW2 */
                 apdu.sw[1] = ack;

                 BKNI_Memcpy(apdu.data + len, apdu.sw, 2);
                 len += 2;

                 break;
             }

             /* Receive ACK (PB) - go ahead and send data */
             if ((ack == apdu.ins) && (apdu.lc)) {
                 BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                      BSCD_Channel_Transmit(in_channelHandle, ++data, apdu.lc));
                 continue;
             }

             /* Receive ~ACK, send single byte only */
             if ((ack == (uint8_t)~apdu.ins) && (apdu.lc)) {
                 BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                      BSCD_Channel_Transmit(in_channelHandle, ++data, 1));
                 apdu.lc--;
                 continue;
             }

             /* 0 means 256 for this case */
             if (bZero256 && (apdu.le == 0))
             	apdu.le = 256;

             BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                  BSCD_Channel_Receive(in_channelHandle, (apdu.data + len), &ulActualRxLen, apdu.le));

             len += ulActualRxLen;
           }

           *outp_ulNumRcvBytes = len;

       } else if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1 ) { /* APDU throught T= 1 */

          if (in_channelHandle->currentChannelSettings.bTPDU == true) {
              len = inp_ucXmitData[2];
              BKNI_Memcpy(ucTxRxBuf, inp_ucXmitData, len+4);
              if ((inp_ucXmitData[1] & 0xc0) == T1_S_BLOCK) {
                  switch (inp_ucXmitData[1] & 0x0f) {
                      case T1_S_WTX:
                           wtx = inp_ucXmitData[3];
                           timeValue.ulValue = wtx * in_channelHandle->currentChannelSettings.blockWaitTime.ulValue;
                           errCode = BSCD_Channel_SetBlockWaitTimeExt(in_channelHandle, timeValue.ulValue);
                           break;

                      case T1_S_RESYNC:
                      case T1_S_ABORT:
                      case T1_S_IFS:
                      default:
                           break;
                  }
              }

              BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                     BSCD_Channel_Transmit(in_channelHandle, ucTxRxBuf, len+4));
              BSCD_Channel_Receive(in_channelHandle, ucTxRxBuf, &ulActualRxLen, T1_MAX_BUF_SIZE);
              if (ulActualRxLen==0) {
                  BERR_Code err;
                  BKNI_Delay(1);
                  err = BSCD_Channel_Receive(in_channelHandle, ucTxRxBuf, &ulActualRxLen, T1_MAX_BUF_SIZE);
                  if ((err == BSCD_STATUS_NO_SC_RESPONSE) && (ulActualRxLen==0))
                      return err;
              }
              *outp_ulNumRcvBytes = ulActualRxLen;
              BKNI_Memcpy(outp_ucRcvData, ucTxRxBuf, ulActualRxLen);
              return (errCode);
          }

        } else {
           errCode = BERR_NOT_SUPPORTED;
        }

BSCD_P_DONE_LABEL:

#ifdef SCI_DEBUG
	printk(KERN_ERR "BSCD RX %ld bytes: ", *outp_ulNumRcvBytes);
	for (ii=0; ii < *outp_ulNumRcvBytes; ii++)
		printk(KERN_ERR "%x ", outp_ucRcvData[ii]);
	printk(KERN_ERR "\n");
#endif

       BDBG_LEAVE(BSCD_Channel_APDU_Transceive);
       return (errCode);
}

BERR_Code BSCD_GetDefaultSettings(
		BSCD_Settings	*outp_sSettings
)
{
        BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_GetDefaultSettings\n"));

	*outp_sSettings = BSCD_defScdSettings;

	BDBG_LEAVE(BSCD_GetDefaultSettings);
	return( errCode );
}


BERR_Code BSCD_Open(
		BSCD_Handle	        *outp_handle,
		const BSCD_Settings	*inp_sSettings,
		BSCD_COUPLER_TYPE    couplerType
)
{
	BERR_Code errCode = BERR_SUCCESS;
 	BSCD_Handle moduleHandle;
	unsigned int channelNum;

	BDBG_ENTER(("BSCD_Open\n"));

	/* Alloc memory from the system heap */
	if ((moduleHandle = (BSCD_Handle) BKNI_Malloc( sizeof(struct BSCD_P_Handle))) == NULL) {
		/* wrap initially detected error code */
		printk(KERN_ERR "%s: BSCD_Open FAIL no memory\n",__func__);
		errCode = BERR_TRACE(BERR_OUT_OF_SYSTEM_MEMORY);
		goto BSCD_P_DONE_LABEL;
	}
	BKNI_Memset(moduleHandle, 0, sizeof(struct BSCD_P_Handle ));

	moduleHandle->ulMagicNumber = BSCD_P_HANDLE_MAGIC_NUMBER;

	if (inp_sSettings == NULL)
		moduleHandle->currentSettings = BSCD_defScdSettings;
	else
		moduleHandle->currentSettings = *inp_sSettings;

	moduleHandle->couplerType = couplerType;


	/* Set ICC CLK Freq */
	if ((moduleHandle->currentSettings.moduleClkFreq.FreqSrc == BSCD_ClockFreqSrc_eInternalClock) ||
		(moduleHandle->currentSettings.moduleClkFreq.FreqSrc == BSCD_ClockFreqSrc_eUnknown) )
	{
		moduleHandle->currentSettings.moduleClkFreq.FreqSrc = BSCD_ClockFreqSrc_eInternalClock;
		moduleHandle->currentSettings.moduleClkFreq.ulClkFreq = BSCD_INTERNAL_CLOCK_FREQ;
	}
	else if ( (moduleHandle->currentSettings.moduleClkFreq.FreqSrc > BSCD_ClockFreqSrc_eExternalClock) ||
		  ((moduleHandle->currentSettings.moduleClkFreq.FreqSrc == BSCD_ClockFreqSrc_eExternalClock) &&
		  (moduleHandle->currentSettings.moduleClkFreq.ulClkFreq == 0) )  ) 
	{
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	else if ((moduleHandle->currentSettings.moduleClkFreq.FreqSrc == BSCD_ClockFreqSrc_eExternalClock) &&
		  (moduleHandle->currentSettings.moduleClkFreq.ulClkFreq != 0) ) 
	{
		/* removed lots of chip specific code here */

	}

	/*
		If inp_sSettings->maxChannels == 0, set it to BSCD_MAX_SUPPOTED_CHANNELS
	*/
	if (moduleHandle->currentSettings.ucMaxChannels == 0)
		moduleHandle->currentSettings.ucMaxChannels = BSCD_MAX_SUPPOTED_CHANNELS;

	for( channelNum = 0; channelNum < moduleHandle->currentSettings.ucMaxChannels; channelNum++ )
		moduleHandle->channelHandles[channelNum] = NULL;

	*outp_handle = moduleHandle;

BSCD_P_DONE_LABEL:
	BDBG_LEAVE(BSCD_Open);
	return( errCode );

}


BERR_Code BSCD_Close(
		BSCD_Handle inout_handle
)
{
	BERR_Code errCode = BERR_SUCCESS;


	BDBG_ENTER(("BSCD_Close\n"));
	BDBG_ASSERT( inout_handle );

	BKNI_EnterCriticalSection();
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, (inout_handle ==  NULL) );
	BKNI_LeaveCriticalSection();

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(inout_handle->ulMagicNumber != BSCD_P_HANDLE_MAGIC_NUMBER ) );

	if (inout_handle->couplerType == COUPLER_NXP8026)
		NXP8026_Remove();

BSCD_P_DONE_LABEL:

	BKNI_EnterCriticalSection();
	inout_handle->ulMagicNumber = 0;
	BKNI_Free(  inout_handle );
	inout_handle = NULL;
	BKNI_LeaveCriticalSection();

	BDBG_LEAVE(BSCD_Close);
	return( errCode );
}


BERR_Code BSCD_GetChannelDefaultSettings(
		BSCD_Handle		in_handle,
		unsigned int		in_channelNo,
		BSCD_ChannelSettings	*outp_sSettings
)
{
	BERR_Code errCode = BERR_SUCCESS;


	BDBG_ENTER(("BSCD_GetChannelDefaultSettings\n"));

	*outp_sSettings = BSCD_defScdChannelSettings;

	BDBG_LEAVE(BSCD_GetChannelDefaultSettings);
	return( errCode );
}

void BSCD_Remove_Reset(BSCD_ChannelHandle channelHandle)
{
		uint32_t  ulValue;

		ulValue = BCHP_SCA_SC_IF_CMD_1_rst_MASK | BKNI_RegRead8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;

		
		 BDBG_MSG("Enable sc rst signal to enable sci coupler writing 0x%x to 0x%lx\n",ulValue,
		 	(channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1));
		 
		BKNI_RegWrite8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulValue);

		/* Wait 20ms (according to NXP) for it to stabilize */
		BKNI_Sleep(20);
}


BERR_Code BSCD_Channel_Open(
		BSCD_Handle			in_handle,
		BSCD_ChannelHandle		*outp_channelHandle,
		unsigned int			in_channelNo,
		const BSCD_ChannelSettings	*inp_channelDefSettings,
		unsigned int            in_regBase
)
{
	BERR_Code errCode = BERR_SUCCESS;
 	BSCD_ChannelHandle channelHandle = NULL;

	BDBG_ENTER(("BSCD_Channel_Open\n"));
	BDBG_ASSERT( in_handle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_handle->ulMagicNumber != BSCD_P_HANDLE_MAGIC_NUMBER ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BERR_INVALID_PARAMETER,
		(in_channelNo >= in_handle->currentSettings.ucMaxChannels) );

	/* channel handle must be NULL.  */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_handle->channelHandles[in_channelNo]  != NULL) );

	*outp_channelHandle = NULL;

	/* Alloc memory from the system heap */
	if ((channelHandle = (BSCD_ChannelHandle) BKNI_Malloc(sizeof(BSCD_P_ChannelHandle))) == NULL) {
		/* wrap initially detected error code */
		errCode = BERR_TRACE(BERR_OUT_OF_SYSTEM_MEMORY);
		goto BSCD_P_DONE_LABEL;
	}

	BKNI_Memset(channelHandle, 0, sizeof( BSCD_P_ChannelHandle ));

	channelHandle->ulMagicNumber = BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER;
	channelHandle->moduleHandle = in_handle;

	channelHandle->ucChannelNumber = in_channelNo;

	channelHandle->ulRegStartAddr  = in_regBase;

#ifdef BSCD_EMV2000_CWT_PLUS_4
	channelHandle->bIsReceive = false;
#endif

	/* Enable sci interface, also put to async mode */
	BKNI_RegWrite8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_MODE_REGISTER), 1);

	BKNI_EnterCriticalSection();
	BKNI_RegWrite8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1), 0);
	BKNI_RegWrite8(channelHandle, (channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_2), 0);
	channelHandle->ulStatus1 = 0x00;
	channelHandle->ulStatus2 = 0x00;
	channelHandle->ulIntrStatus1 = 0x00;
	channelHandle->ulIntrStatus2 = 0x00;
	BKNI_LeaveCriticalSection();

	BDBG_MSG("in_channelNo = %d\n", in_channelNo);
	BDBG_MSG("channelHandle->ulRegStartAddr = 0x%lx\n", channelHandle->ulRegStartAddr);

	/* We put coupler init here instead of BSCD_Open() is because at that time registers are not enabled yet */
	if (in_handle->couplerType == COUPLER_NXP8026)
	{
		/* Need to enable RST signal, since this is tied to NXP8026 SDWNN */
		BSCD_Remove_Reset(channelHandle);
		NXP8026_Init();
	}


	if (inp_channelDefSettings != NULL)
	{
		BDBG_MSG("Using input settings for this channel\n");
		BSCD_Channel_SetParameters(channelHandle, inp_channelDefSettings);
	}
	else 
	{
		BDBG_MSG("Using BSCD_defScdChannelSettings for this channel\n");
		BSCD_Channel_SetParameters(channelHandle, &BSCD_defScdChannelSettings);
	}

	/* Set VCC level */
	BSCD_Channel_SetVccLevel(channelHandle, channelHandle->currentChannelSettings.ctxCardType.eVccLevel);

	/* we don't need to call BSCD_Channel_P_EnableInterrupts_isr() here since bIsOpen is not set */

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.cardWait)));
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.tdoneWait)));
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.rcvWait)));
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.atrStart)));
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.timerWait)));

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.event1Wait)));
#endif

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.event2Wait)));
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BKNI_CreateEvent( &(channelHandle->channelWaitEvent.bhPres)));

	in_handle->channelHandles[in_channelNo] = channelHandle;

	*outp_channelHandle = channelHandle;
	BKNI_EnterCriticalSection();
	channelHandle->bIsOpen = true;
	BKNI_LeaveCriticalSection();


BSCD_P_DONE_LABEL:
	if( errCode != BERR_SUCCESS )
	{
		if( channelHandle != NULL )
		{
			if (channelHandle->channelWaitEvent.cardWait != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.cardWait );
			if (channelHandle->channelWaitEvent.tdoneWait != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.tdoneWait );
			if (channelHandle->channelWaitEvent.rcvWait != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.rcvWait );
			if (channelHandle->channelWaitEvent.atrStart != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.atrStart );
			if (channelHandle->channelWaitEvent.timerWait != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.timerWait );
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			if (channelHandle->channelWaitEvent.atrStart != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.event1Wait );
#endif
			if (channelHandle->channelWaitEvent.timerWait != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.event2Wait );
			if (channelHandle->channelWaitEvent.bhPres != NULL)
				BKNI_DestroyEvent( channelHandle->channelWaitEvent.bhPres );

			BKNI_Free( channelHandle );

		}
	}

	BDBG_LEAVE(BSCD_Channel_Open);

//	iproc_sci_regdump(channelHandle->ulRegStartAddr, "Exiting BSCD_Channel_Open()");
	iproc_sci_regs_dump(0, 0, NULL);

	return( errCode );
}

BERR_Code BSCD_Channel_Close(
		BSCD_Handle			in_handle,
		unsigned int		in_channelNo
)
{
	BSCD_ChannelHandle inout_channelHandle = in_handle->channelHandles[in_channelNo];
	BERR_Code errCode = BERR_SUCCESS;
	BSCD_Handle moduleHandle;

	BDBG_ENTER(("BSCD_Channel_Close\n"));
	BDBG_ASSERT( inout_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(inout_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(inout_channelHandle->bIsOpen ==  false) );


BSCD_P_DONE_LABEL:

	inout_channelHandle->bIsOpen = false;



	BDBG_MSG("DestroyEvents...\n");
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.cardWait );
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.tdoneWait );
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.rcvWait );
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.atrStart );
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.timerWait );
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.event1Wait );
#endif
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.event2Wait );
	BKNI_DestroyEvent( inout_channelHandle->channelWaitEvent.bhPres );

	BDBG_MSG("Channel_Deactivate...\n");
	BSCD_Channel_Deactivate(inout_channelHandle);

	moduleHandle = inout_channelHandle->moduleHandle;
	moduleHandle->channelHandles[inout_channelHandle->ucChannelNumber] = NULL;

	BDBG_MSG("Free inout_channelHandle...\n");
	BKNI_EnterCriticalSection();
	inout_channelHandle->ulMagicNumber = 0;
	BKNI_Free( inout_channelHandle );
	inout_channelHandle = NULL;
	BKNI_LeaveCriticalSection();


	BDBG_LEAVE(BSCD_Channel_Close);
	return( errCode );
}

BERR_Code BSCD_GetChannel(
		BSCD_Handle		in_handle,
		unsigned int		in_channelNo,
		BSCD_ChannelHandle	*outp_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
 	BSCD_ChannelHandle channelHandle = NULL;

	BDBG_ENTER(("BSCD_GetChannel\n"));
	BDBG_ASSERT( in_handle );

	*outp_channelHandle = NULL;
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_handle->ulMagicNumber != BSCD_P_HANDLE_MAGIC_NUMBER ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BERR_INVALID_PARAMETER,
		(in_channelNo >= in_handle->currentSettings.ucMaxChannels) );

	channelHandle = in_handle->channelHandles[in_channelNo];

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(channelHandle == NULL ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(channelHandle->bIsOpen ==  false) );

	*outp_channelHandle = channelHandle;

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_GetChannel);
	return( errCode );
}


bool BSCD_Channel_IsCardActivated(
                BSCD_ChannelHandle      in_channelHandle
)
{
        BERR_Code errCode = BERR_SUCCESS;

        BDBG_ENTER(("BSCD_Channel_IsCardActivated\n"));
        BDBG_ASSERT( in_channelHandle );

        BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
                (in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

        BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
                (in_channelHandle->bIsOpen ==  false) );

        return (in_channelHandle->channelStatus.bCardActivate);

BSCD_P_DONE_LABEL:

        BDBG_LEAVE(BSCD_Channel_IsCardActivated);
        return( false );

}


bool BSCD_Channel_IsPPSDone(
                BSCD_ChannelHandle      in_channelHandle
)
{
        BERR_Code errCode = BERR_SUCCESS;

        BDBG_ENTER(("BSCD_Channel_IsPPSDone\n"));
        BDBG_ASSERT( in_channelHandle );

        BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
                (in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

		if (BSCD_Channel_IsCardActivated(in_channelHandle))
	        return (in_channelHandle->channelStatus.bPPSDone);

BSCD_P_DONE_LABEL:

        BDBG_LEAVE(BSCD_Channel_IsPPSDone);
        return( false );

}




BERR_Code BSCD_Channel_DetectCardNonBlock(
                BSCD_ChannelHandle      in_channelHandle,
                BSCD_CardPresent        in_eCardPresent
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_DetectCardNonBlock\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->bIsOpen ==  false) );

	in_channelHandle->ulStatus1 = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_STATUS_1));
	
	if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
	{
		/* for this coupler, presence status needs to be read from coupler */
		if(NXP8026_Get_Card_Presence()) 
			in_channelHandle->ulStatus1 |=  BCHP_SCA_SC_STATUS_1_card_pres_MASK;
		else
			in_channelHandle->ulStatus1 &= ~BCHP_SCA_SC_STATUS_1_card_pres_MASK;
	}
	
	BDBG_MSG("BSCD_Channel_DetectCardNonBlock, BSCD_P_IF_CMD_1=0x%x\n", BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)));
	BDBG_MSG("BSCD_Channel_DetectCardNonBlock, BSCD_P_STATUS_1=0x%x\n", in_channelHandle->ulStatus1);
	BDBG_MSG("WaitEvent 50ms for bhPres\n");
	BKNI_WaitForEvent(in_channelHandle->channelWaitEvent.bhPres, 50);
	BDBG_MSG("End of Wait\n");

	switch (in_eCardPresent)
	{
		case BSCD_CardPresent_eInserted:
		{
			BKNI_EnterCriticalSection();
		
			if ( in_channelHandle->ulStatus1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK) {
				in_channelHandle->channelStatus.bCardPresent = true;
				BKNI_LeaveCriticalSection();
				goto BSCD_P_DONE_LABEL;
			}
			else
			{
				errCode = BERR_UNKNOWN;
				BDBG_MSG("SmartCard Not Present, Please insert the SmartCard\n");
			}
			BKNI_LeaveCriticalSection();
		}
		break;
	
		case BSCD_CardPresent_eRemoved:
		{
			BKNI_EnterCriticalSection();
		
			if ( !(in_channelHandle->ulStatus1 & BCHP_SCA_SC_STATUS_1_card_pres_MASK)) {
                                in_channelHandle->channelStatus.bCardPresent = false;
				BKNI_LeaveCriticalSection();
				goto BSCD_P_DONE_LABEL;
			}
			else
			{
				errCode = BERR_UNKNOWN;
				BDBG_MSG("SmartCard Present, Please remove the SmartCard\n");
			}
			BKNI_LeaveCriticalSection();
		}
		break;
	}

BSCD_P_DONE_LABEL:
	BDBG_LEAVE(BSCD_Channel_DetectCardNonBlock);
	return(errCode);

}


BERR_Code BSCD_Channel_SetDetectCardCB(
                BSCD_ChannelHandle      in_channelHandle,
                BSCD_CardPresent        in_eCardPresent,
                BSCD_IsrCallbackFunc    in_callback
)
{
        BERR_Code errCode = BERR_SUCCESS;

        BDBG_ENTER(("BSCD_Channel_SetDetectCardCB\n"));
        BDBG_ASSERT( in_channelHandle );

        BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
                (in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

        BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
                (in_channelHandle->bIsOpen ==  false) );

        switch (in_eCardPresent) {
                case BSCD_CardPresent_eInserted:
                {
                     BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                            BSCD_Channel_EnableIntrCallback_isr(in_channelHandle,
                                 BSCD_IntType_eCardInsertInt, in_callback));
                }
                break;

                case BSCD_CardPresent_eRemoved:
                {
                     BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                            BSCD_Channel_EnableIntrCallback_isr(in_channelHandle,
                                 BSCD_IntType_eCardRemoveInt, in_callback));
                }
                break;
        }


BSCD_P_DONE_LABEL:

        BDBG_LEAVE(BSCD_Channel_SetDetectCardCB);
        return( errCode );
}


BERR_Code BSCD_Channel_SetParameters(
		BSCD_ChannelHandle		in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t  ulValue = 0;

	BDBG_ENTER(("BSCD_Channel_SetParameters\n"));
	BDBG_ASSERT( in_channelHandle );
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	/*  Smart Card Standard */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		((inp_sSettings->scStandard <= BSCD_Standard_eUnknown)  || (inp_sSettings->scStandard > BSCD_Standard_eES)) );
	in_channelHandle->currentChannelSettings.scStandard = 	inp_sSettings->scStandard;
	BDBG_MSG("scStandard = %d\n", in_channelHandle->currentChannelSettings.scStandard);

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode, BSCD_Channel_P_SetStandard(
		in_channelHandle, inp_sSettings));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode, BSCD_Channel_P_SetFreq(
		in_channelHandle, inp_sSettings));

	/* Set Vcc level used */
	in_channelHandle->currentChannelSettings.ctxCardType.eVccLevel = inp_sSettings->ctxCardType.eVccLevel;
	in_channelHandle->currentChannelSettings.ctxCardType.inited = inp_sSettings->ctxCardType.inited;

	/* Set maximum IFSD */
	in_channelHandle->currentChannelSettings.unMaxIFSD =  BSCD_MAX_TX_SIZE ;
	BDBG_MSG("unMaxIFSD = %ld\n", in_channelHandle->currentChannelSettings.unMaxIFSD);

	/* Set current IFSD */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(inp_sSettings->unCurrentIFSD > BSCD_MAX_TX_SIZE));
	if (inp_sSettings->unMaxIFSD == 0) {
		in_channelHandle->currentChannelSettings.unCurrentIFSD =  BSCD_MAX_TX_SIZE ;
	}
	else {
		in_channelHandle->currentChannelSettings.unCurrentIFSD =  inp_sSettings->unCurrentIFSD ;
	}
	BDBG_MSG("unCurrentIFSD = %ld\n", in_channelHandle->currentChannelSettings.unCurrentIFSD);

	/* Set current IFSC */
	in_channelHandle->currentChannelSettings.unCurrentIFSC =  inp_sSettings->unCurrentIFSC;
	BDBG_MSG("unCurrentIFSC = %d\n", in_channelHandle->currentChannelSettings.unCurrentIFSC);

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode, BSCD_Channel_P_SetEdcParity(
		in_channelHandle, inp_sSettings));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode, BSCD_Channel_P_SetWaitTime(
		in_channelHandle, inp_sSettings));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode, BSCD_Channel_P_SetGuardTime(
		in_channelHandle, inp_sSettings));

	/* Set transaction time out */
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode, BSCD_Channel_P_SetTransactionTimeout(
		in_channelHandle, inp_sSettings));

	/* auto deactivation sequence */
	in_channelHandle->currentChannelSettings.bAutoDeactiveReq =  inp_sSettings->bAutoDeactiveReq;
	BDBG_MSG("bAutoDeactiveReq = %d\n", in_channelHandle->currentChannelSettings.bAutoDeactiveReq);

	/* nullFilter */
	in_channelHandle->currentChannelSettings.bNullFilter =  inp_sSettings->bNullFilter;
	BDBG_MSG("bNullFilter = %d\n", in_channelHandle->currentChannelSettings.bNullFilter);

	/* debounce info */
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
				(inp_sSettings->scPresDbInfo.ucDbWidth > BSCD_MAX_DB_WIDTH ));
	in_channelHandle->currentChannelSettings.scPresDbInfo =  inp_sSettings->scPresDbInfo;
	BDBG_MSG("scPresDbInfo.bIsEnabled = %d\n", in_channelHandle->currentChannelSettings.scPresDbInfo.bIsEnabled);
	BDBG_MSG("scPresDbInfo.ucDbWidth = %d\n", in_channelHandle->currentChannelSettings.scPresDbInfo.ucDbWidth);
	BDBG_MSG("scPresDbInfo.scPresMode = %d\n", in_channelHandle->currentChannelSettings.scPresDbInfo.scPresMode);

	/* Specify if we want the driver to read, decode and program registers */
	in_channelHandle->currentChannelSettings.resetCardAction = inp_sSettings->resetCardAction;
	BDBG_MSG("resetCardAction = %d\n", in_channelHandle->currentChannelSettings.resetCardAction);


	/* Update the BSCD_P_PRESCALE */
	ulValue = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PRESCALE));
	BDBG_MSG("orig BSCD_P_PRESCALE = 0x%x\n", ulValue);


	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PRESCALE),
				in_channelHandle->currentChannelSettings.unPrescale);
	BDBG_MSG("New BSCD_P_PRESCALE = 0x%lx\n", in_channelHandle->currentChannelSettings.unPrescale);

	/* Don't enable clock here since auto_clk need to be set first in ResetIFD before
	     clock enabling for auto_deactivation */
	ulValue = BKNI_RegRead8(in_channelHandle,	(in_channelHandle->ulRegStartAddr + BSCD_P_CLK_CMD));
	BDBG_MSG("orig ucClkCmd = 0x%x\n",  ulValue);

	/* If enabled before, change the the value.  Otherwise leave it intact. */
	ulValue = ulValue & BCHP_SCA_SC_CLK_CMD_clk_en_MASK;
	if (ulValue == BCHP_SCA_SC_CLK_CMD_clk_en_MASK) {

		ulValue = ulValue | (BSCD_P_MapScClkDivToMaskValue(in_channelHandle->currentChannelSettings.ucScClkDiv))  |
				((in_channelHandle->currentChannelSettings.ucEtuClkDiv - 1) << 1)  |
				((in_channelHandle->currentChannelSettings.ucBaudDiv == 31) ? 0 : 1);

	   	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_CLK_CMD), ulValue);
		if(in_channelHandle->currentChannelSettings.ucBaudDiv == 25){
				ulValue = BKNI_RegRead8(in_channelHandle,	(in_channelHandle->ulRegStartAddr + BSCD_P_FLOW_CMD));
				ulValue = 0x80 |ulValue;
				BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_FLOW_CMD), ulValue);
		}
		BDBG_MSG("New SC_CLK_CMD = 0x%x\n", ulValue);
	}

	BDBG_MSG("ulRegStartAddr = 0x%lx\n", in_channelHandle->ulRegStartAddr);
	BDBG_MSG("BSCD_P_UART_CMD_2 address = 0x%lx\n", (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_2));
	/* Update the BSCD_P_UART_CMD_2 */
	ulValue = BKNI_RegRead8(in_channelHandle,	(in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_2));
	BDBG_MSG("orig BSCD_P_UART_CMD_2 = 0x%x\n", 	ulValue);

	ulValue  &=  (BCHP_SCA_SC_UART_CMD_2_convention_MASK);

	if (inp_sSettings->eProtocolType == BSCD_AsyncProtocolType_e0 ) {

		ulValue |= (in_channelHandle->currentChannelSettings.ucRxRetries << BCHP_SCA_SC_UART_CMD_2_rpar_retry_SHIFT) |
				(in_channelHandle->currentChannelSettings.ucTxRetries);
	}
	else if ( (inp_sSettings->eProtocolType == BSCD_AsyncProtocolType_e1 )  ||
		  (inp_sSettings->eProtocolType == BSCD_AsyncProtocolType_e14_IRDETO ) ) {
		/* No OP */ ;
	}
   	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_2), ulValue);

	BDBG_MSG("BSCD_P_UART_CMD_2 = 0x%x\n", 	ulValue);

	/* Update the BSCD_P_PROTO_CMD */
	ulValue =  BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD));
	if ((inp_sSettings->eProtocolType == BSCD_AsyncProtocolType_e1 ) &&
			(in_channelHandle->currentChannelSettings.edcSetting.bIsEnabled))  {
		ulValue =  BCHP_SCA_SC_PROTO_CMD_edc_en_MASK;

		if (in_channelHandle->currentChannelSettings.edcSetting.edcEncode == BSCD_EDCEncode_eLRC ) {
			ulValue &=  ~BCHP_SCA_SC_PROTO_CMD_crc_lrc_MASK;
		}
		else if (in_channelHandle->currentChannelSettings.edcSetting.edcEncode == BSCD_EDCEncode_eCRC) {
			ulValue |=  BCHP_SCA_SC_PROTO_CMD_crc_lrc_MASK;
		}
	}
	else {
		ulValue &=  ~BCHP_SCA_SC_PROTO_CMD_edc_en_MASK;
	}

	ulValue |= in_channelHandle->currentChannelSettings.ulCharacterWaitTimeInteger;
   	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD), ulValue);

	/* Update the BSCD_P_FLOW_CMD */
	ulValue = 0;
	if (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eNDS) {
		ulValue =  BCHP_SCA_SC_FLOW_CMD_flow_en_MASK;
	}
	else {
		ulValue &=  ~BCHP_SCA_SC_FLOW_CMD_flow_en_MASK;
	}
   	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_FLOW_CMD), ulValue);


	/* Update the BSCD_P_IF_CMD_2 */
	ulValue = 0;
	if (in_channelHandle->currentChannelSettings.scPresDbInfo.bIsEnabled == true) {
		ulValue =  BCHP_SCA_SC_IF_CMD_2_db_en_MASK;
	}
	else {
		ulValue &=  ~BCHP_SCA_SC_IF_CMD_2_db_en_MASK;
	}

	if (in_channelHandle->currentChannelSettings.scPresDbInfo.scPresMode == BSCD_ScPresMode_eMask) {
		ulValue |= BCHP_SCA_SC_IF_CMD_2_db_mask_MASK;
	}
	else if (in_channelHandle->currentChannelSettings.scPresDbInfo.scPresMode == BSCD_ScPresMode_eDebounce) {
		ulValue &= ~BCHP_SCA_SC_IF_CMD_2_db_mask_MASK;
	}
	ulValue |= in_channelHandle->currentChannelSettings.scPresDbInfo.ucDbWidth;
   	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_2), ulValue);

	/* Update the BSCD_P_TGUARD */
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TGUARD),
				in_channelHandle->currentChannelSettings.extraGuardTime.ulValue);

	/* for T=1 we always use TPDU, e.g. we do not build T=1 APDU */
	in_channelHandle->currentChannelSettings.bTPDU = true;

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_SetParameters);
	return( errCode );
}


BERR_Code BSCD_Channel_GetParameters(
		BSCD_ChannelHandle	in_channelHandle,
		BSCD_ChannelSettings	*outp_sSettings
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_GetParameters\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	*outp_sSettings = in_channelHandle->currentChannelSettings;

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_GetParameters);
	return( errCode );
}



BERR_Code BSCD_Channel_GetNegotiateParametersPointer(
		BSCD_ChannelHandle	in_channelHandle,
		BSCD_ChannelSettings	**outp_sSettings
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_GetNegotiateParametersPointer\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	*outp_sSettings = &(in_channelHandle->negotiateChannelSettings);

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_GetNegotiateParametersPointer);
	return( errCode );
}


int BSCD_Channel_GetChannelNumber(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_GetChannelNumber\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_GetChannelNumber);
	if (errCode == BERR_SUCCESS)
		return( in_channelHandle->ucChannelNumber );
	else
		return  -1;
}


BERR_Code BSCD_Channel_Deactivate(
		BSCD_ChannelHandle          in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t	ulValue;

	BDBG_ENTER(("BSCD_Channel_Deactivate\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	/* Disable all interrupts */
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1), 0);
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_2), 0);

	if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
		NXP8026_Deactivate();

	/* Turn off VCC */
	BSCD_Channel_PowerICC(in_channelHandle, BSCD_PowerICC_ePowerDown);

	/* Set RST = 0.     */
	BSCD_Channel_ResetSignal(in_channelHandle, 0);

	/* Set CLK = 0.      */
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_CLK_CMD), 0);

	/* Set IO = 0.      */
	ulValue =  ~(BCHP_SCA_SC_IF_CMD_1_io_MASK) & BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulValue);

	/* Reset Tx & Rx buffers.   */
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1), ~BCHP_SCA_SC_UART_CMD_1_io_en_MASK );
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD), BCHP_SCA_SC_PROTO_CMD_rbuf_rst_MASK | BCHP_SCA_SC_PROTO_CMD_tbuf_rst_MASK);


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_Deactivate);
	return( errCode );
}


BERR_Code BSCD_Channel_ResetIFD(
		BSCD_ChannelHandle	in_channelHandle,
		BSCD_ResetType		in_resetType

)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t	ulIFCmdVal = 0, ulValue;
	BSCD_Timer 	timer = {BSCD_TimerType_eGPTimer, {BSCD_GPTimerMode_eIMMEDIATE}, true, true};
	BSCD_TimerValue timeValue= {2, BSCD_TimerUnit_eETU};


	BDBG_ENTER(("BSCD_Channel_ResetIFD\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	/* Reset all status */
	in_channelHandle->ulStatus1 = 0;
	in_channelHandle->ulStatus2 = 0;
	in_channelHandle->ulIntrStatus1= 0;
	in_channelHandle->ulIntrStatus2= 0;

	in_channelHandle->channelStatus.ulStatus1 = 0;

	if (in_resetType == BSCD_ResetType_eCold) {
		in_channelHandle->channelStatus.bCardPresent = false;

		/* 09/20/05,Allen.C, reset bIsCardRemoved after card removed and reinitialize*/
		in_channelHandle->bIsCardRemoved = false;
	}
	
	/* Reset some critical registers */
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD), 0);
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1), 0);
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_2), 0);
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1), 0);
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_2), 0);

	/* Set up debounce filter */
	if (in_channelHandle->currentChannelSettings.scPresDbInfo.bIsEnabled == true) {

		ulValue = BCHP_SCA_SC_IF_CMD_2_db_en_MASK;

		if (in_channelHandle->currentChannelSettings.scPresDbInfo.scPresMode == BSCD_ScPresMode_eMask) {
			ulValue |= BCHP_SCA_SC_IF_CMD_2_db_mask_MASK;
		}

		ulValue |= in_channelHandle->currentChannelSettings.scPresDbInfo.ucDbWidth;

		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_2), ulValue);
	}
	else {
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_2), 0);
	}
	BDBG_MSG("Inside ResetIFD: debounce info BSCD_P_IF_CMD_2 = 0x%x\n", BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_2)));

	/* Cold Reset or Warm Reset */
	ulIFCmdVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1));
	BDBG_MSG("Inside ResetIFD: Before Cold Reset BSCD_P_IF_CMD_1 = 0x%x\n", ulIFCmdVal);

	if (in_resetType == BSCD_ResetType_eCold) {
		BDBG_MSG("Cold Reset\n");
		in_channelHandle->resetType = BSCD_ResetType_eCold;  /* Cold Reset */

		BSCD_Channel_PowerICC(in_channelHandle, BSCD_PowerICC_ePowerDown);

	 	ulIFCmdVal |= BCHP_SCA_SC_IF_CMD_1_pres_pol_MASK;
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulIFCmdVal);

		if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
		{
			NXP8026_Deactivate();
			BKNI_Sleep(20); /* wait some time */
		}
	}
	else {
		BDBG_MSG("Warm Reset\n");
		in_channelHandle->resetType = BSCD_ResetType_eWarm;  /* Warm Reset */

		if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
		{
			NXP8026_WarmReset();
			BKNI_Sleep(20); /* wait some time */
		}
 	}
	BDBG_MSG("Inside ResetIFD: After Cold Reset BSCD_P_IF_CMD_1 = 0x%x\n", ulIFCmdVal);

	/* Use Auto Deactivation instead of TDA8004 */
	if (in_channelHandle->currentChannelSettings.bAutoDeactiveReq == true) {
		BDBG_MSG("Inside ResetIFD: Before auto clk  BSCD_P_CLK_CMD = 0x%x\n",
				BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_CLK_CMD)));

		ulIFCmdVal |= BCHP_SCA_SC_IF_CMD_1_auto_clk_MASK;
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulIFCmdVal);
	}

	/* Set Clk cmd */
	ulValue = BCHP_SCA_SC_CLK_CMD_clk_en_MASK |
				(BSCD_P_MapScClkDivToMaskValue(in_channelHandle->currentChannelSettings.ucScClkDiv))  |
				((in_channelHandle->currentChannelSettings.ucEtuClkDiv - 1) << 1)  |
				((in_channelHandle->currentChannelSettings.ucBaudDiv == 31) ? 0 : 1);

	BDBG_MSG("Reset: BCM_SC_CLK_CMD = 0x%x\n", (unsigned)ulValue);

	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_CLK_CMD), ulValue);
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PRESCALE), in_channelHandle->currentChannelSettings.unPrescale);

	BDBG_MSG("Reset: BSCD_P_PRESCALE = 0x%lx\n", in_channelHandle->currentChannelSettings.unPrescale);

	/* Use Auto Deactivation instead of TDA8004 */
	if (in_channelHandle->currentChannelSettings.bAutoDeactiveReq == true) {

		BDBG_MSG("Inside ResetIFD: Before auto io ulIFCmdVal = 0x%x\n", ulIFCmdVal);
		ulIFCmdVal |= BCHP_SCA_SC_IF_CMD_1_auto_io_MASK;
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulIFCmdVal);
		BDBG_MSG("Inside ResetIFD: after auto io ulIFCmdVal = 0x%x\n", ulIFCmdVal);
	}

	BSCD_Channel_ResetSignal(in_channelHandle, 1);

	ulValue = 0;
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1), ulValue);

	BDBG_MSG("Inside ResetIFD: Before SmartCardEnableInt\n");


	/* Enable 2 interrupts with callback */
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
 				BSCD_Channel_EnableIntrCallback_isr (in_channelHandle, BSCD_IntType_eCardInsertInt, BSCD_Channel_P_CardInsertCB_isr));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_EnableIntrCallback_isr (in_channelHandle, BSCD_IntType_eCardRemoveInt, BSCD_Channel_P_CardRemoveCB_isr));

	ulValue = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_INTR_EN_1));
	BDBG_MSG("Inside ResetIFD: after enable insert/remove interrupt, BSCD_P_INTR_EN_1 = 0x%x\n", ulValue);
	
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1), BCHP_SCA_SC_UART_CMD_1_uart_rst_MASK);
	BKNI_Delay_tagged(100, __FILE__, __LINE__);
	
	/******************************************************************
	**
	** UART Reset should be set within 1 ETU (however, we are generous
	** to give it 2 etus.
	**
	*****************************************************************/
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		BSCD_Channel_P_WaitForTimerEvent(in_channelHandle, 0)); /* do not check for card removal, since it has nothing to do with the card */
	//this timer is setup to wait for UART reset done, after UART reset done, disable timer ?????
	//if time out, isr event is called
	
	/* Disable timer */
	timer.bIsTimerInterruptEnable = false;
	timer.bIsTimerEnable = false;
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode, BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer));

	ulValue = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_UART_CMD_1));

	/* If equal to zero, then UART reset has gone low, so return success */
	if ((ulValue & BCHP_SCA_SC_UART_CMD_1_uart_rst_MASK) == 0) {
		BDBG_MSG("Reset UART Success\n");

		/*
		**   INITIAL_CWI_SC_PROTO_CMD = 0x0f is required so that
		**   CWI does not remain equal to zero, which causes an
		**   erroneous timeout, the CWI is set correctly in the
		**   SmartCardEMVATRDecode procedure
		*/
		BDBG_MSG("Reset tx/rx buffer\n");
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD), BCHP_SCA_SC_PROTO_CMD_tbuf_rst_MASK | BCHP_SCA_SC_PROTO_CMD_rbuf_rst_MASK);
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_ResetIFD);
	return( errCode );
}


BERR_Code BSCD_Channel_PowerICC(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_PowerICC               in_iccAction
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulValue;

	BDBG_ENTER(("BSCD_Channel_PowerICC\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	switch (in_iccAction) {
		case BSCD_PowerICC_ePowerUp:
			BDBG_MSG("To power up coupler ");
			if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
			{
				BDBG_MSG("NXP8026 \n");
				NXP8026_Set_Vcc(true);
			}
			else
			{
				BDBG_MSG("NXP8024 \n");
				ulValue = ~BCHP_SCA_SC_IF_CMD_1_vcc_MASK & BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;
				BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulValue);
			}
			break;

		case BSCD_PowerICC_ePowerDown:
			BDBG_MSG("To power down coupler ");
			if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
			{
				BDBG_MSG("NXP8026\n");
				NXP8026_Set_Vcc(false);
			}
			else
			{
				BDBG_MSG("NXP8024\n");
				ulValue = BCHP_SCA_SC_IF_CMD_1_vcc_MASK | BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;
				BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulValue);
			}
			break;

		default:
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
			break;
	}

	BDBG_MSG("After PowerICC, BSCD_P_IF_CMD_1 = 0x%x\n", BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)));

	in_channelHandle->channelStatus.bCardActivate = false; /* both powerOn and powerOff ccid message should lead to reactivate card */
	in_channelHandle->channelStatus.bPPSDone = false;

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_PowerICC);
	return( errCode );
}


/* This handles the RST signal (high/low, e.g. true/false) to the ICC */
BERR_Code BSCD_Channel_ResetSignal(
		BSCD_ChannelHandle          in_channelHandle,
		bool                        in_bReset
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulValue;

	BDBG_ENTER(("BSCD_Channel_ResetSignal\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	if (in_bReset) {
		if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
		{
			/* NXP8026 RST is controlled automatically by the coupler */
		}
		else
		{
			ulValue = BCHP_SCA_SC_IF_CMD_1_rst_MASK | BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;
			BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulValue);
		}
	}
	else
	{
		if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
		{
			/* NXP8026 RST is controlled automatically by the coupler */
		}
		else
		{
			ulValue = ~BCHP_SCA_SC_IF_CMD_1_rst_MASK & BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1)) ;
			BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_IF_CMD_1), ulValue);
		}
	}


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_ResetSignal);
	return( errCode );
}

/* This sets the vcc level */
BERR_Code BSCD_Channel_SetVccLevel(
		BSCD_ChannelHandle in_channelHandle,
		BSCD_VccLevel      in_vccLevel
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_SetVccLevel\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	if (in_channelHandle->moduleHandle->couplerType == COUPLER_NXP8026)
	{
		NXP8026_Set_Vcc_Level(in_vccLevel);
	}
	else
	{
		switch (in_vccLevel) {
			case BSCD_VccLevel_e3V:
				break;

			case BSCD_VccLevel_e5V:
				/* For NXP8024:
				   GPB13 for SC0 (select 5V/3V)
				   GPB15 for SC1 (select 5V/3V)
				*/
				/* Set GPB13 (Group1 pin 13) high for 5V  --- fye: removed temporary
				reg_gpio_iotr_set_pin_type(HW_GPIO0_PIN_MAX+13, GPIO_PIN_TYPE_OUTPUT);
				*/
				break;

			case BSCD_VccLevel_e18V:
				break;

			default:
				errCode = BSCD_STATUS_FAILED;
				BDBG_ERR(("BSCD_Channel_SetVccLevel: Do not support VCC Level switch = 0x%x, \n", in_vccLevel));
				goto BSCD_P_DONE_LABEL;
		}
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_SetVccLevel);
	return( errCode );
}

BERR_Code BSCD_Channel_ResetCard(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_ResetCardAction        in_iccAction
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_ResetCard\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );


	switch (in_iccAction) {
		case BSCD_ResetCardAction_eNoAction:
			BSCD_Channel_P_Activating(in_channelHandle);
			break;
		case BSCD_ResetCardAction_eReceiveAndDecode:
			if ((errCode = BSCD_Channel_P_Activating(in_channelHandle)) != BERR_SUCCESS)
			{
				errCode = BERR_TRACE(BSCD_STATUS_DEACTIVATE);
				goto BSCD_P_DONE_LABEL;
			}
			BDBG_MSG("Activating OK\n");
			
			if ((errCode = BSCD_Channel_P_ReceiveAndDecode(in_channelHandle)) != BERR_SUCCESS)
			{
  				errCode = BERR_TRACE(errCode);
				goto BSCD_P_DONE_LABEL;
			}
			BDBG_MSG("ReceiveAndDecode OK\n");
			
			break;
		default:
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
			break;
	}


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_ResetCard);
	BDBG_MSG("Leave ResetCard erroCode = 0x%x\n", errCode);
	return( errCode );
}

BERR_Code  BSCD_Channel_get_atr(
	BSCD_ChannelHandle  in_channelHandle,
	unsigned char	*outp_ucRcvData,
	unsigned long	*outp_ulNumRcvBytes
	)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint8_t bindex;

	for (bindex = 0; bindex < in_channelHandle->ulRxLen; bindex++)
		*outp_ucRcvData++ = in_channelHandle->aucRxBuf[bindex];

	*outp_ulNumRcvBytes = in_channelHandle->ulRxLen;
	return errCode;
}


BERR_Code BSCD_Channel_PPS(
		BSCD_ChannelHandle          in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint8_t pTxBuf[4]; /* size of minimum PPS packet */
	uint8_t pRxBuf[4] = {0}; /* size of minimum PPS packet */
	uint32_t dwRetLen=0;

	BDBG_ENTER(("BSCD_Channel_PPS\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	if (in_channelHandle->bIsPPSNeeded == false)
	{
		BDBG_ALERT("sc: PPS not needed\n");
		in_channelHandle->channelStatus.bPPSDone = true;
		goto BSCD_P_DONE_LABEL;
	}

	{ /* ACOS5 32G card workaroud: skip PPS */
		char stACOS5_32g_ATR[] = {0x3B, 0xBE, 0x18, 0x0, 0x0, 0x41, 0x5, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x90, 0x0};

		if (in_channelHandle->ulRxLen == sizeof(stACOS5_32g_ATR))
		{
			if (BKNI_Memcmp(in_channelHandle->aucRxBuf, stACOS5_32g_ATR,  sizeof(stACOS5_32g_ATR)) == 0)
			{
				BDBG_ALERT("This is a ACOS 32G card.\n");

				errCode = BERR_STATUS_FAKE_FAILED;
				goto BSCD_P_DONE_LABEL;
			}
		}
	}

	/* assign PPS */
	pTxBuf[0] = 0xFF; /* PPSS */
	pTxBuf[1] = 0x10 | (in_channelHandle->negotiateChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1? 1: 0); /* PPS0: PPS1 exist, plus T */
	pTxBuf[2] = (in_channelHandle->negotiateChannelSettings.ucFFactor << 4) | (in_channelHandle->negotiateChannelSettings.ucDFactor & 0x0f); /* PPS1 */
	pTxBuf[3] = (pTxBuf[0] ^ pTxBuf[1] ^ pTxBuf[2]); /* PCK */

	/* send and receive PPS */
	errCode = BSCD_Channel_Transmit(in_channelHandle, pTxBuf, sizeof(pTxBuf));
	if (errCode == BERR_SUCCESS)
		errCode = BSCD_Channel_Receive(in_channelHandle, pRxBuf, (unsigned long *)&dwRetLen, sizeof(pRxBuf));

	in_channelHandle->channelStatus.bPPSDone = true;

	if (errCode)
	{
		char stSetCos43ATR[] = {0x3B, 0x9F, 0x94, 0x40, 0x1E, 0x00, 0x67, 0x11, 0x43, 0x46, 0x49, 0x53, 0x45, 0x10, 0x52, 0x66, 0xFF, 0x81, 0x90, 0x00};

		BDBG_ALERT("ct: PPS tx/rx error\n");

		if (in_channelHandle->ulRxLen == sizeof(stSetCos43ATR))
		{
			if (BKNI_Memcmp(in_channelHandle->aucRxBuf, stSetCos43ATR,  sizeof(stSetCos43ATR)) == 0)
			{
				BDBG_ALERT("This is a SetCos4.3 card.\n");

				errCode = BERR_SUCCESS;
				goto BSCD_P_DONE_LABEL;
			}
		}

		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}
	else
	{
		/* check if response is same as request */
		if (BKNI_Memcmp(pTxBuf, pRxBuf, sizeof(pRxBuf)))
		{
			/* We treat all non-matching cases as failure, although the spec says we need to look byte by byte.
			   From spec:
			   PPSS should be echoed
			   PPS0 b1-b4 should be echoed
			   PPS0 b5 should be either echoed or not
			     When echoed, PPS1 must echoed;
			     When not echoed, PPS1(e.g. F/D) should not be used, we should regard it as PPS fail so our parameter is not changed
			   So in all cases, the response must exactly same as request.
			 */

			BDBG_ALERT("ct: PPS response error\n");
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
		}
	}


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_PPS);
	BDBG_MSG("Leave BSCD_Channel_PPS erroCode = 0x%x\n", errCode);
	return( errCode );
}



BERR_Code BSCD_Channel_GetStatus(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_Status                 *outp_status
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_GetStatus\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	*outp_status = in_channelHandle->channelStatus;

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_GetStatus);
	return( errCode );
}

BERR_Code BSCD_Channel_Transmit(
		BSCD_ChannelHandle          in_channelHandle,
		uint8_t                     *inp_ucXmitData,
		unsigned long               in_ulNumXmitBytes
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_Transmit\n"));
	BDBG_ASSERT( in_channelHandle );


	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_EnableInterrupts(in_channelHandle));

	if (in_channelHandle->currentChannelSettings.scStandard == BSCD_Standard_eIrdeto) {
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true); /* No T=14 support */
	} else if ((in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0_SYNC) ||
		(in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1_SYNC) ) {
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true); /* No sync support */
	} else {
		return (BSCD_Channel_P_T0T1Transmit(
				in_channelHandle,
				inp_ucXmitData,
				in_ulNumXmitBytes));
	}
BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_Transmit);
	return( errCode );
}


BERR_Code BSCD_Channel_Receive(
		BSCD_ChannelHandle      in_channelHandle,
		uint8_t              	*outp_ucRcvData,
		unsigned long           *outp_ulNumRcvBytes,
		unsigned long          	in_ulMaxReadBytes
)
{
	BERR_Code 		errCode = BSCD_STATUS_READ_SUCCESS;
#ifndef BSCD_DSS_ICAM
	BSCD_Timer 		timer = {BSCD_TimerType_eWaitTimer, {BSCD_WaitTimerMode_eWorkWaitTime}, false, false};
#endif

	BDBG_ENTER(("BSCD_Channel_Receive\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );


        BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                         BSCD_Channel_EnableInterrupts(in_channelHandle));


	*outp_ulNumRcvBytes = 0;

	if ((in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0) ||
		(in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e14_IRDETO) ) {

		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				 BSCD_Channel_P_T0ReadData(in_channelHandle, outp_ucRcvData,
                                                           outp_ulNumRcvBytes, in_ulMaxReadBytes));

		/*
			The Work Wait Timer is enabled in BSCD_Channel_P_T0T1Transmit. We cannot disable
			it in BSCD_Channel_P_T0ReadData since BSCD_Channel_P_T0ReadData is also used
			by reading ATR, which is one byte at a time.
		*/

#ifndef BSCD_DSS_ICAM 	  /* BSYT leave this WWT enabled. We only disable WWT in \
	transmit. */
		/*
			I assume all standards, other than EMV, will read all the bytes in BSCD_Channel_P_T0ReadData,
			therefore we couold safely disable the WWT here.  EMV only read 1 bytes at a time, therefore
			we have to disable WWT in the application
		*/
		/* fye: this channel receive can be called multiple times without tx in between, in the case of rx 0x60 only.
		        this will result the WWT being disabled here for the 2nd rx and so on
		*/
		if ((in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV1996) &&
			(in_channelHandle->currentChannelSettings.scStandard != BSCD_Standard_eEMV2000) )
		BSCD_P_CHECK_ERR_CODE_FUNC2(errCode, BSCD_STATUS_READ_FAILED,
			BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer));
#endif


	}/* BSCD_AsyncProtocolType_e0 */

	else if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) {

		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				 BSCD_Channel_P_T1ReadData(in_channelHandle, outp_ucRcvData,
                                                           outp_ulNumRcvBytes, in_ulMaxReadBytes));

	} /* BSCD_AsyncProtocolType_e1 */

	if (*outp_ulNumRcvBytes > 0) {

		/* Ignore the ReadTimeOut error returned by SmartCardByteRead */
		/* printk(KERN_ERR "success in SmartCardReadCmd\n"); */
	}

	else {
		BDBG_MSG("No Response detected...deactivating, scerr = %02x\n",errCode);
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}



BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_Receive);
	/* printk(KERN_ERR "Leave BSCD_Channel_Receive = 0x%x\n", errCode); */
	return( errCode );
}

BERR_Code BSCD_Channel_ReceiveATR(
		BSCD_ChannelHandle      in_channelHandle,
		uint8_t           	*outp_ucRcvData,
		unsigned long           *outp_ulNumRcvBytes,
		unsigned long           in_ulMaxReadBytes
)
{
	BERR_Code 		errCode = BSCD_STATUS_READ_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_ReceiveATR\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	*outp_ulNumRcvBytes = 0;

	if ((in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0_SYNC) ||
		(in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1_SYNC) ) {
				BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true); /* No sync support */
        } else {
	    BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			     BSCD_Channel_P_T0ReadData(in_channelHandle, outp_ucRcvData,
                                                       outp_ulNumRcvBytes, in_ulMaxReadBytes));
        }

	if (*outp_ulNumRcvBytes > 0) {

		/*
		For T=0, we depend on timeout to
		identify that there is no more byte to be received
		*/

		/* Ignore the ReadTimeOut error returned by SmartCardByteRead */
		/* printk(KERN_ERR "success in SmartCardReadCmd\n"); */
	}

	else {
		BDBG_MSG("No Response detected...deactivating, scerr = %02x\n", errCode);
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_ReceiveATR);
	return( errCode );
}

BERR_Code BSCD_Channel_ConfigTimer(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_Timer 		    *inp_timer,
		BSCD_TimerValue             *inp_unCount

)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t			ulTimerCmdVal, ulTimerCmpVal;

	BDBG_ENTER(("BSCD_Channel_ConfigTimer\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	if (inp_timer->eTimerType == BSCD_TimerType_eGPTimer) {

		/* add check */
		if (inp_unCount->ulValue >> 16)
		{
			BDBG_ALERT("Warning: SC set GT timer more than 16 bits, change it back to 0xFFFF\n");
			inp_unCount->ulValue = 0xFFFF;
		}

		/* Always disbale timer first before we change timer_cmd */
		ulTimerCmdVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD));
		ulTimerCmdVal &= (~BCHP_SCA_SC_TIMER_CMD_timer_en_MASK);
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD), ulTimerCmdVal);
		BDBG_MSG("before we change timer_cmd: BSCD_P_TIMER_CMD = 0x%x\n", BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD)));
	
		BKNI_EnterCriticalSection();
		in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK;
		BKNI_LeaveCriticalSection();

		/* Set timer_cmp registers */

		ulTimerCmpVal = ((inp_unCount->ulValue & 0xFF00) >> 8);
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMP_2),	ulTimerCmpVal);

		ulTimerCmpVal = inp_unCount->ulValue & 0x00FF;
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMP_1),	ulTimerCmpVal);


		/* Set the timer unit and mode */
		if ( inp_unCount->unit == BSCD_TimerUnit_eCLK) {
			ulTimerCmdVal |= BCHP_SCA_SC_TIMER_CMD_timer_src_MASK;
		}
		else if (inp_unCount->unit  == BSCD_TimerUnit_eETU) {
			ulTimerCmdVal &= (~BCHP_SCA_SC_TIMER_CMD_timer_src_MASK);
		}
		else {
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
		}

		if (inp_timer->timerMode.eGPTimerMode == BSCD_GPTimerMode_eNEXT_START_BIT ) {
			ulTimerCmdVal |= BCHP_SCA_SC_TIMER_CMD_timer_mode_MASK;
		}
		else {  /* BSCD_GPTimerMode_eIMMEDIATE */
			ulTimerCmdVal &= (~BCHP_SCA_SC_TIMER_CMD_timer_mode_MASK);
		}

		/* Check if we need to invoke an interrupt when the time expires */
		if (inp_timer->bIsTimerInterruptEnable == true) {  /* inp_timer->bIsTimerInterruptEnable == true && BSCD_TimerType_eGPTimer */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_EnableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eTimerInt,
			               		BSCD_Channel_P_TimerCB_isr));
		}
		else { /* inp_timer->bIsTimerInterruptEnable == false && BSCD_TimerType_eGPTimer */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_DisableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eTimerInt));
		}

	 	if (inp_timer->bIsTimerEnable == true) {
			ulTimerCmdVal	 |= BCHP_SCA_SC_TIMER_CMD_timer_en_MASK;
	 	} /* inp_timer->bIsTimerEnable == true && BSCD_TimerType_eGPTimer */

		else { /* inp_timer->bIsTimerEnable == false && BSCD_TimerType_eGPTimer */
			ulTimerCmdVal	 &= ~BCHP_SCA_SC_TIMER_CMD_timer_en_MASK;

		}

	}  /* if (inp_timer->eTimerType == BSCD_TimerType_eGPTimer) */

	else {  /* BSCD_TimerType_eWaitTimer */

		/* add check */
		if (inp_unCount->ulValue >> 24)
			BDBG_ALERT("Err: SC set GT timer more than 16 bits\n");

		/* Always disable timer first before we change timer_cmd */
		ulTimerCmdVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD));
		ulTimerCmdVal &= (~BCHP_SCA_SC_TIMER_CMD_wait_en_MASK);
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD), ulTimerCmdVal);

		BKNI_EnterCriticalSection();
		in_channelHandle->ulIntrStatus2  &= ~BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK;
		BKNI_LeaveCriticalSection();

		/* Set sc_wait registers */
		ulTimerCmpVal = ((inp_unCount->ulValue  & 0xFF0000) >> 16);
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_WAIT_3), ulTimerCmpVal);

		ulTimerCmpVal = ((inp_unCount->ulValue & 0x00FF00) >> 8);
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_WAIT_2), ulTimerCmpVal);

		ulTimerCmpVal = (inp_unCount->ulValue & 0x0000FF);
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_WAIT_1), ulTimerCmpVal);

		/* Check if we need to invoke an interrupt when the time expires */
		if (inp_timer->bIsTimerInterruptEnable == true) {
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_EnableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eWaitInt,
			               		BSCD_Channel_P_WaitCB_isr));
		}
		else {
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_DisableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eWaitInt));
		}

	 	if (inp_timer->bIsTimerEnable == true) {

			/* Set the wait mode */
			if (inp_timer->eTimerType == BSCD_TimerType_eWaitTimer) {
				if (inp_timer->timerMode.eWaitTimerMode == BSCD_WaitTimerMode_eBlockWaitTime) {
					ulTimerCmdVal |= BCHP_SCA_SC_TIMER_CMD_wait_mode_MASK;
				}
				else { /* BSCD_WaitTimerMode_eWorkWaitTime */
					ulTimerCmdVal &= ~ BCHP_SCA_SC_TIMER_CMD_wait_mode_MASK;
				}

				ulTimerCmdVal |= BCHP_SCA_SC_TIMER_CMD_wait_en_MASK;
			}

	 	}/* BSCD_TimerType_eWaitTimer && inp_timer->bIsTimerEnable == true */

		else { /* inp_timer->bIsTimerEnable == false && BSCD_TimerType_eWaitTimer */
    			ulTimerCmdVal &= ~BCHP_SCA_SC_TIMER_CMD_wait_en_MASK;
		}


	}  /* else  BSCD_TimerType_eWaitTimer */
	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD), ulTimerCmdVal);
	BKNI_Delay(150);
	BDBG_MSG("*** BSCD_Channel_ConfigTimer: BSCD_P_TIMER_CMD = 0x%x\n", ulTimerCmdVal);

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_ConfigTimer);
	return( errCode );
}


bool BSCD_Channel_IsTimerEnabled(
		BSCD_ChannelHandle   in_channelHandle,
		BSCD_TimerType       eTimerType
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t ulTimerCmdVal;

	BDBG_ENTER(("BSCD_Channel_IsTimerEnabled\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	ulTimerCmdVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD));

	if (eTimerType == BSCD_TimerType_eGPTimer) {
		return (ulTimerCmdVal & BCHP_SCA_SC_TIMER_CMD_timer_en_MASK);
	}
	else {  /* BSCD_TimerType_eWaitTimer */
		return (ulTimerCmdVal & BCHP_SCA_SC_TIMER_CMD_wait_en_MASK);
	}

BSCD_P_DONE_LABEL:
	return 0;
}




BERR_Code BSCD_Channel_EnableDisableTimer_isr(
		BSCD_ChannelHandle   in_channelHandle,
		BSCD_Timer           *inp_timer
)
{
	BERR_Code errCode = BERR_SUCCESS;
	uint32_t         ulTimerCmdVal;

	BDBG_ENTER(("BSCD_Channel_EnableDisableTimer_isr\n"));
	BDBG_ASSERT( in_channelHandle );

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );


	ulTimerCmdVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD));

	if (inp_timer->eTimerType == BSCD_TimerType_eGPTimer) {

		in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK;

		/* Check if we need to invoke an interrupt when the time expires */
		if (inp_timer->bIsTimerInterruptEnable == true) {  /* inp_timer->bIsTimerInterruptEnable == true && BSCD_TimerType_eGPTimer */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_EnableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eTimerInt,
			               		BSCD_Channel_P_TimerCB_isr));
		}
		else { /* inp_timer->bIsTimerInterruptEnable == false && BSCD_TimerType_eGPTimer */
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_DisableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eTimerInt));
		}

	 	if (inp_timer->bIsTimerEnable == true) {
			ulTimerCmdVal	 |= BCHP_SCA_SC_TIMER_CMD_timer_en_MASK;
	 	} /* inp_timer->bIsTimerEnable == true && BSCD_TimerType_eGPTimer */

		else { /* inp_timer->bIsTimerEnable == false && BSCD_TimerType_eGPTimer */
			ulTimerCmdVal	 &= ~BCHP_SCA_SC_TIMER_CMD_timer_en_MASK;
		}


	}  /* if (inp_timer->eTimerType == BSCD_TimerType_eGPTimer) */

	else {  /* BSCD_TimerType_eWaitTimer */

		in_channelHandle->ulIntrStatus1  &= ~BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK;

		/* Check if we need to invoke an interrupt when the time expires */
		if (inp_timer->bIsTimerInterruptEnable == true) {
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_EnableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eWaitInt,
			               		BSCD_Channel_P_WaitCB_isr));
		}
		else {
			BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
					BSCD_Channel_DisableIntrCallback_isr (
						in_channelHandle, BSCD_IntType_eWaitInt));
		}

	 	if (inp_timer->bIsTimerEnable == true) {

			/* Set the wait mode */
			if (inp_timer->eTimerType == BSCD_TimerType_eWaitTimer) {
				if (inp_timer->timerMode.eWaitTimerMode == BSCD_WaitTimerMode_eBlockWaitTime) {
					ulTimerCmdVal |= BCHP_SCA_SC_TIMER_CMD_wait_mode_MASK;
				}
				else { /* BSCD_WaitTimerMode_eWorkWaitTime */
					ulTimerCmdVal &= ~ BCHP_SCA_SC_TIMER_CMD_wait_mode_MASK;
				}

				ulTimerCmdVal |= BCHP_SCA_SC_TIMER_CMD_wait_en_MASK;
			}

	 	}/* BSCD_TimerType_eWaitTimer && inp_timer->bIsTimerEnable == true */

		else { /* inp_timer->bIsTimerEnable == false && BSCD_TimerType_eWaitTimer */
    			ulTimerCmdVal &= ~BCHP_SCA_SC_TIMER_CMD_wait_en_MASK;
		}


	}  /* else  BSCD_TimerType_eWaitTimer */

	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CMD), ulTimerCmdVal);

	BDBG_MSG("*** BSCD_Channel_EnableDisableTimer_isr: Timer cmd = 0x%08x\n", ulTimerCmdVal);

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_EnableDisableTimer_isr);
	return( errCode );
}

BERR_Code BSCD_Channel_EnableIntrCallback_isr(
	BSCD_ChannelHandle	in_channelHandle,
	BSCD_IntrType		in_eIntType,
	BSCD_IsrCallbackFunc    in_callback
)
{
	uint32_t  ulVal, old_ulVal;
	unsigned int  unReg = BSCD_P_INTR_EN_1, i;
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_MSG("in_eIntType=%d\n", in_eIntType);
	
	if ( (in_eIntType == BSCD_IntType_eTParityInt)    ||
	    (in_eIntType == BSCD_IntType_eTimerInt)      ||
	    (in_eIntType == BSCD_IntType_eCardInsertInt) ||
	    (in_eIntType == BSCD_IntType_eCardRemoveInt) ||
	    (in_eIntType == BSCD_IntType_eBGTInt)        ||
	    (in_eIntType == BSCD_IntType_eTDoneInt)      ||
	    (in_eIntType == BSCD_IntType_eRetryInt)      ||
	    (in_eIntType == BSCD_IntType_eTEmptyInt) ||
	    (in_eIntType == BSCD_IntType_eEvent1Int)) {
		unReg = BSCD_P_INTR_EN_1;
		/* printk(KERN_ERR "BSCD_P_INTR_EN_1: "); */
	}
	else if ( (in_eIntType == BSCD_IntType_eRParityInt) ||
	    (in_eIntType == BSCD_IntType_eATRInt)          ||
	    (in_eIntType == BSCD_IntType_eCWTInt)          ||
	    (in_eIntType == BSCD_IntType_eRLenInt)         ||
	    (in_eIntType == BSCD_IntType_eWaitInt)         ||
	    (in_eIntType == BSCD_IntType_eRcvInt)          ||
	    (in_eIntType == BSCD_IntType_eRReadyInt) ||
	    (in_eIntType == BSCD_IntType_eEvent2Int)) {
		unReg = BSCD_P_INTR_EN_2;
		/* printk(KERN_ERR "BSCD_P_INTR_EN_2: "); */
	}
	else if (in_eIntType == BSCD_IntType_eEDCInt) {
		unReg = BSCD_P_PROTO_CMD;
	}
	else {
		BDBG_ERR(("Interrupt not supported, in_eIntType = %d\n", in_eIntType));
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,  true);
	}

	ulVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + unReg));
	old_ulVal = ulVal;

	/* printk(KERN_ERR "ulVal = 0x%x", ulVal); */

	switch (in_eIntType) {

		case BSCD_IntType_eTParityInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.tParityIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.tParityIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.tParityIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.tParityIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_tpar_intr_MASK;
			break;

		case BSCD_IntType_eTimerInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.timerIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.timerIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.timerIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.timerIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK;
			break;

		case BSCD_IntType_eCardInsertInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.cardInsertIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.cardInsertIsrCBFunc[i] = in_callback;
					BDBG_MSG("new BSCD_IntType_eCardInsertInt  callback \n");
					break;
				}
				else if ((in_channelHandle->callBack.cardInsertIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.cardInsertIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			break;

		case BSCD_IntType_eCardRemoveInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.cardRemoveIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.cardRemoveIsrCBFunc[i] = in_callback;
					BDBG_MSG("new BSCD_IntType_eCardRemoveInt  callback \n");
					break;
				}
				else if ((in_channelHandle->callBack.cardRemoveIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.cardRemoveIsrCBFunc[i] == in_callback) ) {
					BDBG_MSG("BSCD_IntType_eCardRemoveInt same callback \n");
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			break;

		case BSCD_IntType_eBGTInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.bgtIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.bgtIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.bgtIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.bgtIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK;
			break;

		case BSCD_IntType_eTDoneInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.tDoneIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.tDoneIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.tDoneIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.tDoneIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK;
			break;

		case BSCD_IntType_eRetryInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.retryIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.retryIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.retryIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.retryIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK;
			break;

		case BSCD_IntType_eTEmptyInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.tEmptyIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.tEmptyIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.tEmptyIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.tEmptyIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_tempty_intr_MASK;
			break;

		case BSCD_IntType_eRParityInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.rParityIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.rParityIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.rParityIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.rParityIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_rpar_intr_MASK;
			break;

		case BSCD_IntType_eATRInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.atrIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.atrIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.atrIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.atrIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK;
			break;

		case BSCD_IntType_eCWTInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.cwtIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.cwtIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.cwtIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.cwtIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK;
			break;

		case BSCD_IntType_eRLenInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.rLenIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.rLenIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.rLenIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.rLenIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK;
			break;

		case BSCD_IntType_eWaitInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.waitIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.waitIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.waitIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.waitIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK;
			break;

		case BSCD_IntType_eRcvInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.rcvIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.rcvIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.rcvIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.rcvIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK;
			break;

		case BSCD_IntType_eRReadyInt:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.rReadyIsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.rReadyIsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.rReadyIsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.rReadyIsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK;
			break;

		case BSCD_IntType_eEDCInt:
			if (in_channelHandle->currentChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0 ) {
				for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
					if (in_channelHandle->callBack.edcIsrCBFunc[i] == NULL) {
						in_channelHandle->callBack.edcIsrCBFunc[i] = in_callback;
						break;
					}
					else if ((in_channelHandle->callBack.edcIsrCBFunc[i] != NULL) &&
						(in_channelHandle->callBack.edcIsrCBFunc[i] == in_callback) ) {
						break;
					}
				}
				ulVal |=  BCHP_SCA_SC_PROTO_CMD_edc_en_MASK;
			}
			break;

		case BSCD_IntType_eEvent1Int:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.event1IsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.event1IsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.event1IsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.event1IsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK;
			break;

		case BSCD_IntType_eEvent2Int:
			for (i=0; i< BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
				if (in_channelHandle->callBack.event2IsrCBFunc[i] == NULL) {
					in_channelHandle->callBack.event2IsrCBFunc[i] = in_callback;
					break;
				}
				else if ((in_channelHandle->callBack.event2IsrCBFunc[i] != NULL) &&
					(in_channelHandle->callBack.event2IsrCBFunc[i] == in_callback) ) {
					break;
				}
			}
			ulVal |=  BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK;
			break;

		default:
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}

	if (ulVal != old_ulVal) {
		BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + unReg), ulVal);
	}


	/*printk(KERN_ERR ", final ulVal = 0x%x\n ", ulVal); */

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_EnableIntrCallback_isr);
	return( errCode );

}

BERR_Code BSCD_Channel_DisableIntrCallback_isr(
	BSCD_ChannelHandle	in_channelHandle,
	BSCD_IntrType           in_eIntType
)
{
	uint32_t ulVal;
	unsigned int  unReg = BSCD_P_INTR_EN_1;
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_MSG("BSCD_Channel_DisableIntrCallback_isr(): in_eIntType=%d\n", in_eIntType);

	if ( (in_eIntType == BSCD_IntType_eTParityInt)    ||
		(in_eIntType == BSCD_IntType_eTimerInt)      ||
		(in_eIntType == BSCD_IntType_eCardInsertInt) ||
		(in_eIntType == BSCD_IntType_eCardRemoveInt) ||
		(in_eIntType == BSCD_IntType_eBGTInt)        ||
		(in_eIntType == BSCD_IntType_eTDoneInt)      ||
		(in_eIntType == BSCD_IntType_eRetryInt)      ||
		(in_eIntType == BSCD_IntType_eTEmptyInt) ||
	    	(in_eIntType == BSCD_IntType_eEvent1Int)) {
		unReg = BSCD_P_INTR_EN_1;
		/* printk(KERN_ERR "BSCD_P_INTR_EN_1: ");		 */
	}
	else if ( (in_eIntType == BSCD_IntType_eRParityInt) ||
		(in_eIntType == BSCD_IntType_eATRInt)          ||
		(in_eIntType == BSCD_IntType_eCWTInt)          ||
		(in_eIntType == BSCD_IntType_eRLenInt)         ||
		(in_eIntType == BSCD_IntType_eWaitInt)         ||
		(in_eIntType == BSCD_IntType_eRcvInt)          ||
		(in_eIntType == BSCD_IntType_eRReadyInt) ||
	   	 (in_eIntType == BSCD_IntType_eEvent2Int)) {
		unReg = BSCD_P_INTR_EN_2;
		/* printk(KERN_ERR "BSCD_P_INTR_EN_2: ");		 */
	}
	else if (in_eIntType == BSCD_IntType_eEDCInt) {
		unReg = BSCD_P_PROTO_CMD;
	}
	else {
		BDBG_MSG("Interrupt not supported\n");
		BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,  true);
	}

	ulVal = BKNI_RegRead8(in_channelHandle, (in_channelHandle->ulRegStartAddr + unReg));

	/* printk(KERN_ERR "ulVal = 0x%x", ulVal); */

	switch (in_eIntType) {


		case BSCD_IntType_eTParityInt:
			 ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_tpar_intr_MASK;
			 break;

		case BSCD_IntType_eTimerInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_timer_intr_MASK;
			break;

		case BSCD_IntType_eCardInsertInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			break;

		case BSCD_IntType_eCardRemoveInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_pres_intr_MASK;
			break;

		case BSCD_IntType_eBGTInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_bgt_intr_MASK;
			break;

		case BSCD_IntType_eTDoneInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_tdone_intr_MASK;
			break;

		case BSCD_IntType_eRetryInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_retry_intr_MASK;
			break;

		case BSCD_IntType_eTEmptyInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_tempty_intr_MASK;
			break;

		case BSCD_IntType_eRParityInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_2_rpar_intr_MASK;
			break;

		case BSCD_IntType_eATRInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_2_atrs_intr_MASK;
			break;

		case BSCD_IntType_eCWTInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_2_cwt_intr_MASK;
			break;

		case BSCD_IntType_eRLenInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_2_rlen_intr_MASK;
			break;

		case BSCD_IntType_eWaitInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_2_wait_intr_MASK;
			break;

		case BSCD_IntType_eRcvInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK;
			break;

		case BSCD_IntType_eRReadyInt:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_2_rready_intr_MASK;
			break;

		case BSCD_IntType_eEDCInt:
			ulVal &= ~BCHP_SCA_SC_PROTO_CMD_edc_en_MASK;
			break;

		case BSCD_IntType_eEvent1Int:
			ulVal &= ~BCHP_SCA_SC_INTR_STAT_1_event1_intr_MASK;
			break;

		case BSCD_IntType_eEvent2Int:
			ulVal &=  ~BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK;
			break;

		default:
			BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED, true);
	}

	BKNI_RegWrite8(in_channelHandle, (in_channelHandle->ulRegStartAddr + unReg), ulVal);

	/* printk(KERN_ERR ", final ulVal = 0x%x\n ", ulVal); */
BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_DisableIntrCallback_isr);
	return( errCode );
}

BERR_Code BSCD_Channel_EnableInterrupts(
	BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_EnableInterrupts\n"));
	BDBG_ASSERT( in_channelHandle );

	BKNI_EnterCriticalSection();
	if ( (errCode = BSCD_Channel_P_EnableInterrupts_isr(in_channelHandle)) != BERR_SUCCESS) {
		errCode = BERR_TRACE(errCode);
		BKNI_LeaveCriticalSection();
		goto BSCD_P_DONE_LABEL;
	}
	BKNI_LeaveCriticalSection();

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_EnableInterrupts);
	return( errCode );
}

BERR_Code BSCD_Channel_ResetBlockWaitTimer(
		BSCD_ChannelHandle          in_channelHandle
)
{
	BERR_Code errCode = BERR_SUCCESS;
	BSCD_Timer 		timer = {BSCD_TimerType_eWaitTimer, {BSCD_GPTimerMode_eIMMEDIATE},
							false, true};
	BSCD_TimerValue    timeValue= {BSCD_DEFAULT_BLOCK_WAITING_TIME, BSCD_TimerUnit_eETU};

	BDBG_ENTER(("BSCD_Channel_ResetBlockWaitTimer\n"));
	BDBG_ASSERT( in_channelHandle );

	/* Need this for MetroWerks */
	timer.eTimerType = BSCD_TimerType_eWaitTimer;
	timer.timerMode.eWaitTimerMode = BSCD_WaitTimerMode_eBlockWaitTime;

	timeValue.ulValue = in_channelHandle->currentChannelSettings.blockWaitTime.ulValue ;
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_Channel_ConfigTimer(in_channelHandle, &timer, &timeValue));

	in_channelHandle->currentChannelSettings.blockWaitTimeExt.ulValue = 0;

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_ResetBlockWaitTimer);
	return( errCode );
}


/* Get BWT (Note: Not extension) */
BERR_Code BSCD_Channel_GetBlockWaitTime(
		BSCD_ChannelHandle        in_channelHandle,
		uint32_t                 *pin_ulBlockWaitTimeInETU
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_GetBlockWaitTime\n"));
	BDBG_ASSERT( in_channelHandle );

	*pin_ulBlockWaitTimeInETU = in_channelHandle->currentChannelSettings.blockWaitTime.ulValue;


	BDBG_LEAVE(BSCD_Channel_GetBlockWaitTime);
	return( errCode );

}

/* Set BWT Extension */
BERR_Code BSCD_Channel_SetBlockWaitTimeExt(
		BSCD_ChannelHandle          in_channelHandle,
		uint32_t 		    in_ulBlockWaitTimeExtInETU
)
{
	BERR_Code errCode = BERR_SUCCESS;

	BDBG_ENTER(("BSCD_Channel_SetBlockWaitTimeExt\n"));
	BDBG_ASSERT( in_channelHandle );

	in_channelHandle->currentChannelSettings.blockWaitTimeExt.ulValue = in_ulBlockWaitTimeExtInETU;

	BDBG_LEAVE(BSCD_Channel_SetBlockWaitTimeExt);
	return( errCode );
}


/* Set BWI */
BERR_Code BSCD_Channel_SetBlockWaitTimeInteger(
		BSCD_ChannelSettings     *pChannelSettings,
		unsigned char             ucBWIVal
)
{
	BERR_Code errCode = BERR_SUCCESS;
	unsigned char ucBaudDiv, ucClkDiv;
	uint32_t      ulBlockWaitTime;

	BDBG_ENTER(("BSCD_Channel_SetBlockWaitTimeInt\n"));

	/*
		The block waiting time is encoded as described in ISO 7816-3,
		repeated here in the following equation:

		BWT = [11 + 2 bwi x 960 x 372 x D/F] etu

		e.g If bwi = 4 and F/D = 372 then BWT = 15,371 etu.
		The minimum and maximum BWT are ~186 and 15,728,651 etu.
	*/
	pChannelSettings->unPrescale = BSCD_P_GetPrescale(pChannelSettings->ucDFactor, pChannelSettings->ucFFactor) * pChannelSettings->ucExternalClockDivisor +
					(pChannelSettings->ucExternalClockDivisor - 1);

	ucBaudDiv = BSCD_P_GetBaudDiv(pChannelSettings->ucDFactor, pChannelSettings->ucFFactor);
	ucClkDiv = BSCD_P_GetClkDiv(pChannelSettings->ucDFactor, pChannelSettings->ucFFactor);

	if (ucBWIVal == 0x00) {
		ulBlockWaitTime = 960 * 372 * ucClkDiv /(pChannelSettings->unPrescale+1) / ucBaudDiv + 11;
	}
	else {
		ulBlockWaitTime = (2<<(ucBWIVal-1)) * 960 *  372 * ucClkDiv / (pChannelSettings->unPrescale+1) / ucBaudDiv  + 11;
	}


	/* Change timer to equal calculated BWT */
	pChannelSettings->blockWaitTime.ulValue =  ulBlockWaitTime;
	pChannelSettings->blockWaitTime.unit    =  BSCD_TimerUnit_eETU;


	BDBG_LEAVE(BSCD_P_Set_BWT);
	return( errCode );

}
