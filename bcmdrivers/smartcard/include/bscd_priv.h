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

#ifndef BSCD_PRIV_H__
#define BSCD_PRIV_H__

#include "berr_ids.h"
#include "bscd.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Definitions */

/**
  Smart card D and F Table according to SC_PRESCALE Table, where F is Clock Rate
  Conversion Factor and D is the Baud Rate Adjustment Factor.
  This table contains the recommended set of values for the SC_PRESCALE and SC_CLK_CMD based
  on the ATR TA1 byte. The resulting clock rate and baud rate will comply to ISO 7816-3.
**/
typedef struct
{
      unsigned char sc_clkdiv;      /* clk divider in SC_CLK_CMD register */
      unsigned char sc_prescale;    /* prescale in SC_PRESCALE register */
      unsigned char sc_bauddiv;     /* baud rate divider in SC_CLK_CMD register */
      unsigned char sc_etuclkdiv;   /* etu clk divider in SC_CLK_CMD register */
} BSCD_P_DFSmartCardStruct;


#define BSCD_P_UART_CMD_1		0x00  /* SMART CARD UART COMMAND REGISTER */
#define BSCD_P_UART_CMD_2		0x40  /* SMART CARD UART COMMAND REGISTER */
#define BSCD_P_PROTO_CMD		0x0c  /* SMART CARD PROTOCOL COMMAND REGISTER */
#define BSCD_P_FLOW_CMD		        0x28  /* SMART CARD FLOW CONTROL COMMAND REGISTER */
#define BSCD_P_IF_CMD_1			0x04  /* SMART CARD INTERFACE COMMAND REGISTER */
#define BSCD_P_IF_CMD_2            	0x4c  /* SMART CARD INTERFACE COMMAND REGISTER */
#define BSCD_P_INTR_STAT_1		0x58  /* SMART CARD INTERRUPT STATUS REGISTER */
#define BSCD_P_INTR_STAT_2		0x5c  /* SMART CARD INTERRUPT STATUS REGISTER */
#define BSCD_P_INTR_EN_1		0x50  /* SMART CARD INTERRUPT ENABLE REGISTER */
#define BSCD_P_INTR_EN_2		0x54  /* SMART CARD INTERRUPT ENABLE REGISTER */
#define BSCD_P_CLK_CMD			0x08  /* SMART CARD CLOCK COMMAND */
#define BSCD_P_PRESCALE			0x10  /* SMART CARD CLOCK PRESCALE */
#define BSCD_P_TIMER_CMD		0x48  /* SMART CARD TIMER COMMAND REGISTER */
#define BSCD_P_BGT			0x44  /* SMART CARD BLOCK GUARD TIME REGISTER */
#define BSCD_P_TIMER_CNT_1		0x68  /* SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER */
#define BSCD_P_TIMER_CNT_2		0x6c  /* SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER */
#define BSCD_P_TIMER_CMP_1		0x60  /* SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER */
#define BSCD_P_TIMER_CMP_2		0x64  /* SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER */
#define BSCD_P_WAIT_1			0x70  /* SMART CARD WAITING TIMER REGISTER */
#define BSCD_P_WAIT_2			0x74  /* SMART CARD WAITING TIMER REGISTER */
#define BSCD_P_WAIT_3			0x78  /* SMART CARD WAITING TIMER REGISTER */
#define BSCD_P_TGUARD			0x14  /* SMART CARD TRANSMIT GUARD TIME REGISTER */
#define BSCD_P_TRANSMIT			0x18  /* SMART CARD TRANSMIT REGISTER */
#define BSCD_P_RECEIVE			0x1c  /* SMART CARD RECEIVE REGISTER */
#define BSCD_P_STATUS_1			0x34  /* SMART CARD STATUS 1 REGISTER */
#define BSCD_P_STATUS_2			0x38  /* SMART CARD STATUS 2 REGISTER */
#define BSCD_P_TLEN_2			0x20  /* SMART CARD TRANSMIT LENGTH REGISTER */
#define BSCD_P_TLEN_1			0x24  /* SMART CARD TRANSMIT LENGTH REGISTER */
#define BSCD_P_RLEN_2			0x2c  /* SMART CARD RECEIVE LENGTH REGISTER */
#define BSCD_P_RLEN_1			0x30  /* SMART CARD RECEIVE LENGTH REGISTER */
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
#define BSCD_P_EVENT1_CNT		0x80 /* SMART CARD EVENT 1 COUNT REGISTER */
#define BSCD_P_EVENT1_CMP 		0x88 /* SMART CARD EVENT 1 COMPARE REGISTER  */
#define BSCD_P_EVENT1_CMD_1 	        0x90 /* SMART CARD EVENT 1 COMMAND 1 REGISTER */
#define BSCD_P_EVENT1_CMD_2 	        0x94 /* SMART CARD EVENT 1 COMMAND 2 REGISTER  */
#define BSCD_P_EVENT1_CMD_3	        0x98/* SMART CARD EVENT 1 COMMAND 3 REGISTER  */
#define BSCD_P_EVENT1_CMD_4	        0x9c /* SMART CARD EVENT 1 COMMAND 4 REGISTER  */
#define BSCD_P_EVENT2_CMP		0xa0 /* SMART CARD EVENT 2 COMPARE REGISTER  */
#define BSCD_P_EVENT2_CNT		0xa8 /* SMART CARD EVENT 2 COUNT REGISTER  */
#define BSCD_P_EVENT2_CMD_1	        0xb0 /* SMART CARD EVENT 2 COMMAND 1 REGISTER */
#define BSCD_P_EVENT2_CMD_2	        0xb4 /* SMART CARD EVENT 2 COMMAND 2 REGISTER  */
#define BSCD_P_EVENT2_CMD_3	        0xb8 /* SMART CARD EVENT 2 COMMAND 3 REGISTER  */
#define BSCD_P_EVENT2_CMD_4	        0xbc /* SMART CARD EVENT 2 COMMAND 4 REGISTER  */
#endif
/* BCM5880 specific registers */
#define BSCD_P_DMA_RX_INTF              (0x030 << 2)
#define BSCD_P_DMA_TX_INTF              (0x031 << 2)
#define BSCD_P_MODE_REGISTER            (0x032 << 2)
#define BSCD_P_DB_WIDTH                 (0x033 << 2)
#define BSCD_P_SYNCCLK_DIVIDER1         (0x034 << 2)
#define BSCD_P_SYNCCLK_DIVIDER2         (0x035 << 2)
#define BSCD_P_SYNC_XMIT                (0x036 << 2)
#define BSCD_P_SYNC_RCV                 (0x037 << 2)
#define BSCD_P_LDO_PARAMETERS           (0x038 << 2)



/* Smart Card Module Event source for Event interrupt */
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
#define  BSCD_P_EVENT1_INTR_EVENT_SRC		  0x00
#define  BSCD_P_TEMPTY_INTR_EVENT_SRC		  0x01
#define  BSCD_P_RETRY_INTR_EVENT_SRC		  0x02
#define  BSCD_P_TDONE_INTR_EVENT_SRC		  0x03
#define  BSCD_P_BGT_INTR_EVENT_SRC		  0x04
#define  BSCD_P_PRES_INTR_EVENT_SRC		  0x05
#define  BSCD_P_TIMER_INTR_EVENT_SRC		  0x06
#define  BSCD_P_TPAR_INTR_EVENT_SRC		  0x07
#define  BSCD_P_RREADY_INTR_EVENT_SRC		  0x08
#define  BSCD_P_RCV_INTR_EVENT_SRC		  0x09
#define  BSCD_P_EVENT2_INTR_EVENT_SRC		  0x0a
#define  BSCD_P_WAIT_INTR_EVENT_SRC		  0x0b
#define  BSCD_P_RLEN_INTR_EVENT_SRC		  0x0c
#define  BSCD_P_CWT_INTR_EVENT_SRC		  0x0d
#define  BSCD_P_ATRS_INTR_EVENT_SRC		  0x0e
#define  BSCD_P_RPAR_INTR_EVENT_SRC		  0x0f
#define  BSCD_P_RX_ETU_TICK_EVENT_SRC		  0x10
#define  BSCD_P_TX_ETU_TICK_EVENT_SRC		  0x11
#define  BSCD_P_HALF_TX_ETU_TICK_EVENT_SRC	  0x12
#define  BSCD_P_RX_START_BIT_EVENT_SRC		  0x13
#define  BSCD_P_TX_START_BIT_EVENT_SRC		  0x14
#define  BSCD_P_LAST_TX_START_BIT_BLOCK_EVENT_SRC 0x15
#define  BSCD_P_ICC_FLOW_CTRL_ASSERTED_EVENT_SRC  0x16
#define  BSCD_P_ICC_FLOW_CTRL_DONE_EVENT_SRC	  0x17
#define  BSCD_P_START_IMMEDIATE_EVENT_SRC	  0x1f
#define  BSCD_P_DISABLE_COUNTING_EVENT_SRC	  0x1f
#define  BSCD_P_NO_EVENT_EVENT_SRC	          0x1f
#endif

/* Smart Card Module magic number used to check if opaque handle is corrupt */
#define BSCD_P_HANDLE_MAGIC_NUMBER           0xdeadbeef

/* Smart Card Channel magic number used to check if opaque handle is corrupt */
#define BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER   0xfeedbabe



/* End of Definitions */


/* Enum Types */





/* End of Enum Types */


/* Smart Card Private Data Structures */

/***************************************************************************
Summary:
Smart card event handle.

Description:
Upon receiving an interrupt, ISR shall signal this event to wake up the thread/task
that is waiting for this event.

See Also:
BSCD_ChannelSettings, BSCD_Open()

****************************************************************************/
typedef struct BSCD_P_WaitEvent {

    BKNI_EventHandle cardWait;   /* card detection */
    BKNI_EventHandle tdoneWait;  /* transmit done */
    BKNI_EventHandle rcvWait;    /*receive at least one byte */
    BKNI_EventHandle atrStart;   /* receive start bit of the ATR */
    BKNI_EventHandle timerWait;  /* timer expired */
    BKNI_EventHandle event1Wait; /* timer expired */
    BKNI_EventHandle event2Wait; /* timer expired */

    BKNI_EventHandle bhPres;     /* card insert/remove bottom half finished or not */
} BSCD_P_WaitEvent;


/***************************************************************************
Summary:
Structure that defines Smart card Baud Rate and Clock divisor.

Description:
Smart card D and F Table according to SC_PRESCALE Table, where F is Clock Rate
Conversion Factor and D is the Baud Rate Adjustment Factor.
This table contains the recommended set of values for the SC_PRESCALE and SC_CLK_CMD
based  on the ATR TA1 byte. The resulting clock rate and baud rate will comply to
ISO 7816-3.

See Also:
BSCD_OpenChannel()

****************************************************************************/
typedef struct
{
      uint8_t clkDiv;      /* clk divider in SC_CLK_CMD register */
      uint8_t prescale;    /* prescale in SC_PRESCALE register */
      uint8_t baudDiv;     /* baud rate divider in SC_CLK_CMD register */
      uint8_t etuClkDiv;   /* etu clk divider in SC_CLK_CMD register */
} BSCD_P_DFStruct;


/***************************************************************************
Summary:
Structure that defines Smart card calback function.

Description:
Structure that defines Smart card calback function.

See Also:


****************************************************************************/
typedef struct BSCD_P_IsrCallBackFunc
{
        /* CBFunc[0] is always for default CB function.  CBFunc[1] could be used for customized CB function */
	BSCD_IsrCallbackFunc	tParityIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];      /* Interrupt Callback Function for Transmit Parity */
	BSCD_IsrCallbackFunc	timerIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];        /* Interrupt Callback Function for eneral Purpose Timer */
	BSCD_IsrCallbackFunc  	cardInsertIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];   /* Interrupt Callback Function for Card Insertion */
	BSCD_IsrCallbackFunc  	cardRemoveIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];   /* Interrupt Callback Function for Card Removal */
	BSCD_IsrCallbackFunc  	bgtIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];          /* Interrupt Callback Function for Block Guard Time */
	BSCD_IsrCallbackFunc  	tDoneIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];        /* Interrupt Callback Function for Transmit Done */
	BSCD_IsrCallbackFunc  	retryIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];        /* Interrupt Callback Function for Transmit or Receive Retry Overflow */
	BSCD_IsrCallbackFunc  	tEmptyIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];       /* Interrupt Callback Function for Transmit Empty */
	BSCD_IsrCallbackFunc  	rParityIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];      /* Interrupt Callback Function for Receive Parity */
	BSCD_IsrCallbackFunc  	atrIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];          /* Interrupt Callback Function for ATR Start */
	BSCD_IsrCallbackFunc  	cwtIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];          /* Interrupt Callback Function for CWT */
	BSCD_IsrCallbackFunc  	rLenIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];         /* Interrupt Callback Function for Receive Length Error */
	BSCD_IsrCallbackFunc  	waitIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];         /* Interrupt Callback Function for Block or Work Waiting Time */
	BSCD_IsrCallbackFunc  	rcvIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];          /* Interrupt Callback Function for Receive Character */
	BSCD_IsrCallbackFunc  	rReadyIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];       /* Interrupt Callback Function for Receive Ready */
	BSCD_IsrCallbackFunc  	edcIsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];          /* Callback Function for EDC Error */
	BSCD_IsrCallbackFunc  	event1IsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];       /* Callback Function for Event1 inerrupt */
	BSCD_IsrCallbackFunc  	event2IsrCBFunc[BSCD_MAX_NUM_CALLBACK_FUNC];       /* Callback Function for Event2 inerrupt */

} BSCD_P_IsrCallBackFunc;

/***************************************************************************
Summary:
Structure that defines Smart card channel  handle.

Description:
Structure that defines Smart card channel  handle.
This P_Handle is actually BSCD_ChannelHandle, defined in bscd.h as follows:
typedef struct BSCD_P_ChannelHandle     *BSCD_ChannelHandle;

See Also:
BSCD_OpenChannel()

****************************************************************************/
typedef struct BSCD_P_ChannelHandle
{
	unsigned long		ulMagicNumber; /* Must be  BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER */

	BSCD_Handle    		moduleHandle;   /* Module handle */

	BSCD_ChannelSettings	currentChannelSettings;   /* current channel settings */

	BSCD_ChannelSettings	negotiateChannelSettings;   /* negotiating channel settings from ATR */

	BINT_CallbackHandle	channelIntCallback;  /* Channel Interrupt Callback */

	unsigned char		ucChannelNumber;     /* channel number */

	uint32_t		ulStatus1;           /* value of SC_STATUS_1 */

	uint32_t		ulStatus2;           /* value of SC_STATUS_2 */

	uint32_t		ulIntrStatus1;       /* value of SC_INTR_STATUS_1 */

	uint32_t		ulIntrStatus2;       /* value of SC_INTR_STATUS_2 */

	BSCD_P_WaitEvent	channelWaitEvent;    /* Wait event */

	BSCD_ResetType		resetType; 	/* Need this for EMV ATR to determine if this is warmReset or coldReset */

	BSCD_Status		channelStatus;         /* Channel status that returned by BSCD_Channel_GetStatus */

	bool			bIsOpen;    /* Is channel opened */

	unsigned char		aucRxBuf[BSCD_MAX_RX_SIZE]; /*receiving bytes */

	unsigned long		ulRxLen;                   /* number receiving bytes */

	unsigned long		ulRegStartAddr;            /* Start Address of this smart card  channel, got from channel open */

	BSCD_P_IsrCallBackFunc	callBack;			/* interrupt Callback functions */

#ifdef BSCD_EMV2000_CWT_PLUS_4
	bool			bIsReceive;    /* Is channel opened */
#endif

	bool			bIsCardRemoved;    /* Is the Card removed ? */
	bool            bIsPPSNeeded;      /* Is PPS needed */

#ifdef BSCD_EMV2000_FIME
        unsigned char           parity_error_cnt;  /* FIME EMV2000 for 1732 xy=20/21/22 patch */
#endif

} BSCD_P_ChannelHandle;


/* End of Smart Card Private Data Structures */


/* Private Function */

unsigned char BSCD_P_GetClkDiv(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
);

unsigned char BSCD_P_GetETUClkDiv(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
);

unsigned char BSCD_P_GetISOBaudRateAdjustor(
      unsigned char in_ucDFactor
);

unsigned int BSCD_P_GetISOClockRateConversionFactor(
      unsigned char in_ucFFactor
);

unsigned char BSCD_P_MapScClkDivToMaskValue(
      unsigned char in_ucClkDiv
);

unsigned char BSCD_P_GetPrescale(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
);

unsigned char BSCD_P_GetBaudDiv(
      unsigned char in_ucDFactor,
      unsigned char in_ucFFactor
);

BERR_Code BSCD_P_FDAdjust(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char 		in_ucFFactor,
		unsigned char		in_ucDFactor
);

BERR_Code BSCD_P_FDAdjust_WithoutRegisterUpdate(
		BSCD_ChannelSettings     *pChannelSettings,
		unsigned char            in_ucFFactor,
		unsigned char            in_ucDFactor
);

BERR_Code BSCD_P_AdjustWWT(
		BSCD_ChannelSettings	*pChannelSettings,
		unsigned char       	 in_ucWorkWaitTImeInteger
);

void BSCD_Channel_P_CardInsertCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
) ;


void  BSCD_Channel_P_CardRemoveCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
) ;

void BSCD_Channel_P_RcvCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
);

void BSCD_Channel_P_ATRCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       		*inp_data
);

void BSCD_Channel_P_WaitCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
) ;

void BSCD_Channel_P_TimerCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
) ;

void BSCD_Channel_P_RetryCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_TimerCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_RParityCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_TParityCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_CWTCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_BGTCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_RLenCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_RReadyCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

void BSCD_Channel_P_TDoneCB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
void BSCD_Channel_P_Event1CB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);
#endif

void BSCD_Channel_P_Event2CB_isr(
      BSCD_ChannelHandle	in_channelHandle,
      void       *inp_data
);

BERR_Code BSCD_Channel_P_WaitForCardInsertion(
		BSCD_ChannelHandle	in_channelHandle
);

BERR_Code BSCD_Channel_P_WaitForCardRemove(
		BSCD_ChannelHandle	in_channelHandle
);

BERR_Code BSCD_Channel_P_WaitForTimerEvent(
		BSCD_ChannelHandle	in_channelHandle,
		uint32_t            in_nCheckCardRemoval
);

BERR_Code BSCD_Channel_P_WaitForATRStart(
		BSCD_ChannelHandle	in_channelHandle
);

BERR_Code BSCD_Channel_P_WaitForTDone(
		BSCD_ChannelHandle	in_channelHandle
);


BERR_Code BSCD_Channel_P_WaitForRcv(
		BSCD_ChannelHandle	in_channelHandle
);

BERR_Code BSCD_Channel_P_WaitForRReady(
		BSCD_ChannelHandle	in_channelHandle
);


BERR_Code BSCD_Channel_P_Activating(
		BSCD_ChannelHandle	in_channelHandle
);


void BSCD_P_HexDump(
      char          *inp_cTitle,
      unsigned char *inp_ucBuf,
      unsigned int  in_unLen
);

BERR_Code BSCD_Channel_P_T0ReadData(
		BSCD_ChannelHandle      in_channelHandle,
		uint8_t         	*outp_ucRcvData,
		unsigned long           *outp_unNumRcvBytes,
		unsigned long           in_unMaxReadBytes
);

BERR_Code BSCD_Channel_P_T1ReadData(
		BSCD_ChannelHandle       in_channelHandle,
		uint8_t                  *outp_ucRcvData,
		unsigned long            *outp_ulNumRcvBytes,
		unsigned long            in_ulMaxReadBytes
);

BERR_Code BSCD_Channel_P_ByteRead(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char *outp_ucData
);

BERR_Code BSCD_Channel_P_EnableInterrupts_isr(
		BSCD_ChannelHandle	in_channelHandle
);

int BSCD_Channel_P_IntHandler_isr(BSCD_ChannelHandle channelHandle, int in_channelNumber);
void BSCD_Channel_P_IntHandler_bh(BSCD_ChannelHandle channelHandle, int in_channelNumber);

BERR_Code BSCD_Channel_P_T0T1Transmit(
		BSCD_ChannelHandle          in_channelHandle,
		uint8_t                     *inp_ucXmitData,
		unsigned long               in_ulNumXmitBytes
);


/*****************************************************************************
Summary:
This function creates the events that associated with the channel.

Description:
This function creates the events that associated with the channel.

Calling Context:
The function shall be called in BSCD_ChannelOpen .

Performance and Timing:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a smart card
					channel handle.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:


******************************************************************************/
BERR_Code BSCD_P_CreateWaitEvent(
      BSCD_ChannelHandle in_channelHandle
);

/*****************************************************************************
Summary:
This function destroys the events that associated with the channel.

Description:
This function destroys the events that associated with the channel.

Calling Context:
The function shall be called in BSCD_ChannelClose .

Performance and Timing:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a smart card
					channel handle.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:


******************************************************************************/
BERR_Code BSCD_P_DestroyWaitEvent(
      BSCD_ChannelHandle in_channelHandle
);

/*****************************************************************************
Summary:
This function control the state transition of smart card channel.

Description:
This function control the state transition of smart card channel.

Calling Context:


Performance and Timing:
This is a synchronous function that will return when it is done.

Input:
in_event - BSCD_P_Event, state transition event.

Input/Output:
in_channelHandle  - BSCD_ChannelHandle, a smart card
					channel handle.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:


******************************************************************************/


BERR_Code BSCD_Channel_P_ReceiveAndDecode(
		BSCD_ChannelHandle	in_channelHandle
);

BERR_Code BSCD_Channel_P_SetStandard(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
);


BERR_Code BSCD_Channel_P_SetFreq(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
);

BERR_Code BSCD_Channel_P_SetWaitTime(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
);

BERR_Code BSCD_Channel_P_SetGuardTime(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
);

BERR_Code BSCD_Channel_P_SetTransactionTimeout(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
);

BERR_Code BSCD_Channel_P_SetEdcParity(
		BSCD_ChannelHandle	in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
);

/* End of Private Function */

#ifdef __cplusplus
}
#endif

#endif /* BSCD_PRIV_H__ */
