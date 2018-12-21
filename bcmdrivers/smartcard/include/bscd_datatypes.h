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

#ifndef BSCD_DATATYPES_H__
#define BSCD_DATATYPES_H__

#ifdef __cplusplus
extern "C" {
#endif


#define BSCD_P_CHECK_ERR_CODE_FUNC( errCode, function )		\
	if( (errCode = (function)) != BERR_SUCCESS )	\
	{							\
		errCode = BERR_TRACE(errCode);	\
		goto BSCD_P_DONE_LABEL;	\
	}


/* Don't change this without testing EMV 1704 */
#define BSCD_P_CHECK_ERR_CODE_FUNC2( errCode, errCodeValue, function )			\
	if ( ( errCode = (function)) != BERR_SUCCESS ) \
	{										\
		errCode = BERR_TRACE(errCodeValue);	\
		goto BSCD_P_DONE_LABEL;							\
	}


#define BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, errCodeValue, condition )		\
	if( (condition) ) \
	{ \
		errCode = BERR_TRACE((errCodeValue));		\
		goto BSCD_P_DONE_LABEL;				\
	} else {}


/***************************************************************************
Summary:
This enum represents all the Smartcard Interrupt Types.

Description:
This enumeration defines all the Smartcard Interrupt Types.

See Also:


****************************************************************************/
typedef enum  BSCD_IntrType {
      BSCD_IntType_eTParityInt,     /* Transmit Parity Interrupt. Only for T=0 */
      BSCD_IntType_eTimerInt,       /* General-purpose Timer Interrupt */
      BSCD_IntType_eCardInsertInt,  /* Card Insertion Interrupt */
      BSCD_IntType_eCardRemoveInt,  /* Card Removal Interrupt */
      BSCD_IntType_eBGTInt,         /* Block Guard Time Interrupt */
      BSCD_IntType_eTDoneInt,       /* Transmit Done Interrupt */
      BSCD_IntType_eRetryInt,       /* Transmit or Receive Retry Overflow Interrupt */
      BSCD_IntType_eTEmptyInt,      /* Transmit Empty Interrupt. Only used for debugging */
      BSCD_IntType_eRParityInt,     /* Receive Parity Interrupt. Only for T=0 */
      BSCD_IntType_eATRInt,         /* ATR Start Interrupt */
      BSCD_IntType_eCWTInt,         /* CWT Interrupt */
      BSCD_IntType_eRLenInt,        /* Receive Length Error Interrupt. Only for T=1 */
      BSCD_IntType_eWaitInt,        /* Block or Work Waiting Time Interrupt */
      BSCD_IntType_eRcvInt,         /* Receive Character Interrupt */
      BSCD_IntType_eRReadyInt,      /* Receive Ready Interrupt. Only for T=1 */
      BSCD_IntType_eEDCInt,         /* EDC Error. Yet to be implemented. */
      BSCD_IntType_eEvent1Int,      /* Event1 Interrupt.  */
      BSCD_IntType_eEvent2Int       /* Event2 Interrupt. */
} BSCD_IntrType;

/* Definitions */

/**
   Define these to temporarily fixing RDB issue
**/
#ifndef BCHP_SCA_SC_INTR_STAT_1_event1_intr_SHIFT
#define BCHP_SCA_SC_INTR_STAT_1_event1_intr_SHIFT BCHP_SCA_SC_INTR_STAT_1_reserved1_SHIFT
#endif

#ifndef BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK
#define BCHP_SCA_SC_INTR_STAT_2_event2_intr_MASK   BCHP_SCA_SC_INTR_STAT_2_reserved1_MASK
#endif

#ifndef BCHP_SCA_SC_INTR_STAT_2_event2_intr_SHIFT
#define BCHP_SCA_SC_INTR_STAT_2_event2_intr_SHIFT BCHP_SCA_SC_INTR_STAT_2_reserved1_SHIFT
#endif

#ifndef BCHP_SCA_SC_INTR_EN_1_event1_ien_MASK
#define BCHP_SCA_SC_INTR_EN_1_event1_ien_MASK BCHP_SCA_SC_INTR_EN_1_RESERVED_MASK
#endif

#ifndef BCHP_SCA_SC_INTR_EN_1_event1_ien_SHIFT
#define BCHP_SCA_SC_INTR_EN_1_event1_ien_SHIFT BCHP_SCA_SC_INTR_EN_1_RESERVED_SHIFT
#endif

#ifndef BCHP_SCA_SC_INTR_EN_2_event2_ien_MASK
#define BCHP_SCA_SC_INTR_EN_2_event2_ien_MASK  BCHP_SCA_SC_INTR_EN_2_RESERVED_MASK
#endif

#ifndef BCHP_SCA_SC_INTR_EN_2_event2_ien_SHIFT
#define BCHP_SCA_SC_INTR_EN_2_event2_ien_SHIFT BCHP_SCA_SC_INTR_EN_2_RESERVED_SHIFT
#endif



/**
  ISO Work Waiting Time factor
**/
#define  BSCD_ISO_WORK_WAIT_TIME_DEFAULT_FACTOR			960
/* #define  BSCD_EMV2000_EXTRA_WORK_WAIT_TIME_DEFAULT_FACTOR	480 */

/**
  EMV 2000 Work Waiting Time factor in etu
**/
#define  BSCD_EMV2000_WORK_WAIT_TIME_DELTA 			1

/**
	DEfault ISO Work Waiting Time Integer (WI)
**/
#define  BSCD_ISO_DEFAULT_WORK_WAIT_TIME_INTEGER		10

/**
  Mask to combine SC_RLEN1 and SC_RLEN1 into rlen.  But
  we only want the 9 bits of those 2 registers
**/
#define  BSCD_RLEN_9_BIT_MASK                        0x01ff

/**
  For EMV and ISO T=1, the minimum interval btw the leading
  edges of the start bits of 2 consecutive characters sent
  in opposite directions shall be 22.
**/
#define BSCD_BLOCK_GUARD_TIME                     22

/**
  For EMV T=0 only, the minimum interval btw the leading
  edges of the start bits of 2 consecutive characters sent
  in opposite directions shall be 16.
**/
#define BSCD_MIN_DELAY_BEFORE_TZERO_SEND          16

/**
  Maximum ETU allowed per ATR byte
**/
#define BSCD_MAX_ETU_PER_ATR_BYTE                    9600
#define BSCD_MAX_ETU_PER_ATR_BYTE_EMV2000            10081

/**
  Maximum ETU allowed for whole EMV ATR package
**/
#define BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES               19200
#define BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES_EMV2000       20160

/**
  Default EMV Information field size for T=1, used for IFSD defalut
**/
#define BSCD_DEFAULT_EMV_INFORMATION_FIELD_SIZE		0x20

/**
  Default IFSC
**/
#define BSCD_DEFAULT_IFSC		0x20


/**
  Reset Period
**/
#define  BSCD_MAX_RESET_IN_CLK_CYCLES                42000

#define BSCD_MAX_ATR_SIZE 				33
#define BSCD_MAX_ATR_HISTORICAL_SIZE 		        15
#define BSCD_MAX_APDU_SIZE 				500

/**
  ATR timing
**/
#define  BSCD_MAX_ATR_START_IN_CLK_CYCLES            40000
#define  BSCD_EMV2000_MAX_ATR_START_IN_CLK_CYCLES    42000
#define  BSCD_MIN_ATR_START_IN_CLK_CYCLES            400

/**
  Delay between when ATR is sent from SmartCard
  and when ATR_START bit get set. In micro secs
**/

#ifdef BESPVR
#define  BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES      200 /* TDA8004 clkdiv =1 */
#else
#define  BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES      250
#endif

#ifdef FPGA_CM3
#define BSCD_DELAY_MSEC_COUNTS                       525
#else
//#define BSCD_DELAY_MSEC_COUNTS                      2098  /* internal RAM */
#define BSCD_DELAY_MSEC_COUNTS                       230    /* external RAM */
#endif


 /* Internal Clock Frequency in Hz */
#define BSCD_DEFAULT_EXTERNAL_CLOCK_DIVISOR		1

 /* Internal Clock Frequency in Hz */
#define BSCD_INTERNAL_CLOCK_FREQ_27MHz	27000000
#define BSCD_INTERNAL_CLOCK_FREQ_40MHz	40000000
#define BSCD_INTERNAL_CLOCK_FREQ		BSCD_INTERNAL_CLOCK_FREQ_40MHz /*Cygnus use fixed 40MHz clk source*/

 /* Default SC_CLK_CMD[etu_clkdiv]  */
#ifdef CYGNUS_SCI_DRV
#define BSCD_DEFAULT_ETU_CLKDIV		1
#else
#define BSCD_DEFAULT_ETU_CLKDIV		6
#endif

 /* Default SC_CLK_CMD[sc_clkdiv]  */
#ifdef CYGNUS_SCI_DRV
#define BSCD_DEFAULT_SC_CLKDIV		10
#else
#define BSCD_DEFAULT_SC_CLKDIV		1
#endif

/* Default SC_PRESCALE */
#ifdef CYGNUS_SCI_DRV
#define BSCD_DEFAULT_PRESCALE		119
#else
#define BSCD_DEFAULT_PRESCALE		11
#endif

/* Max SC_PRESCALE */
#define BSCD_MAX_PRESCALE		0xFF

/* Default Baud Divisor */
#define BSCD_DEFAULT_BAUD_DIV		31

 /* Default Clock Rate Conversion Factor  */
#define BSCD_DEFAULT_F		1

 /* Default Baud Rate Adjustment Factor  */
#define BSCD_DEFAULT_D		1

 /* Default number of transmit parity */
#define BSCD_DEFAULT_TX_PARITY_RETRIES		5

 /* Default EMV 2000 number of transmit parity */
#define BSCD_EMV2000_DEFAULT_TX_PARITY_RETRIES		4

 /*  maximum number of transmit parity */
#define BSCD_MAX_TX_PARITY_RETRIES		7

 /* Default number of receiving parity */
#define BSCD_DEFAULT_RX_PARITY_RETRIES		5

 /* Default EMV 2000 number of receiving parity */
#define BSCD_EMV2000_DEFAULT_RX_PARITY_RETRIES		4

 /* maximum number of receiving parity */
#define BSCD_MAX_RX_PARITY_RETRIES		7

 /* Default work waiting time in ETUs */
#define BSCD_DEFAULT_WORK_WAITING_TIME		9600
#define BSCD_DEFAULT_EXTRA_WORK_WAITING_TIME_EMV2000		480

 /*
 	Default block waiting time in ETUs = 11 + 960 etus, where bwi = 0
*/
#define BSCD_DEFAULT_BLOCK_WAITING_TIME		971
#define BSCD_DEFAULT_EXTRA_BLOCK_WAITING_TIME_EMV2000  		960

 /* Default Extra Guard Time in ETUs */
#define BSCD_DEFAULT_EXTRA_GUARD_TIME		2

/**
For EMV and ISO T=1, the minimum interval btw the leading
edges of the start bits of 2 consecutive characters sent
in opposite directions shall be 22 ETUs.
**/
#define BSCD_DEFAULT_BLOCK_GUARD_TIME          22

/**
Min Block Guard Time
**/
#define BSCD_MIN_BLOCK_GUARD_TIME              12


/**
Max Block Guard Time
**/
#define BSCD_MAX_BLOCK_GUARD_TIME              31



/**
Default character wait time in ETUs
**/
#define BSCD_DEFAULT_CHARACTER_WAIT_TIME_INTEGER            0
#define BSCD_CHARACTER_WAIT_TIME_GRACE_PERIOD               1

/**
Maximum character wait time in ETUs
**/
#define BSCD_MAX_CHARACTER_WAIT_TIME_INTEGER               15

/* Default timeout period for each transaction in milli seconds. */
//#ifdef BSCD_EMV_TEST
/* EMV2000 test 1775, xy=04 requires 6000, test 1795 requires 13 seconds */
#define BSCD_DEFAULT_TIME_OUT			13000
//#else
//#define BSCD_DEFAULT_TIME_OUT			5000
//#endif


/* T14 Irdeto clock Rate Conversion factor.  Spec says 625, but card prefers 600 */
#define BSCD_T14_IRDETO_CONSTANT_CLOCK_RATE_CONV_FACTOR		600

/* Irdeto needs to wait for minimum of 1250 from last RX to the next TX  */
#define BSCD_T14_IRDETO_MIN_DELAY_RX2TX		1250

/**
Maximum supported Channels in BSCD module.
**/
#define BSCD_MAX_SUPPOTED_CHANNELS    1  /* support only one channel for 5892 */


/**
Maximum supported callback functions per interrupt.
**/
#define BSCD_MAX_NUM_CALLBACK_FUNC    2


/**
The maximum number of bytes that system can write into the hardware transmit
buffer.
**/
#define BSCD_MAX_TX_SIZE          264

/**
The maximum number of bytes that system can write into the hardware receive
buffer.
**/
#define BSCD_MAX_RX_SIZE          264


/**
The maximum number of bytes that EMV package
**/
#define BSCD_MAX_EMV_BUFFER_SIZE          258

/**
The default db width in IF_CMD_2
**/
#define BSCD_DEFAULT_DB_WIDTH 		0x07   /*  around 10 ms */



/**
The maximum db width in IF_CMD_2
**/
#define BSCD_MAX_DB_WIDTH 		0x3F

/**
Definition to be used in member ulStatus1 of struct BSCD_Status
Either one of the following defined values shall be set to ulStatus1.
Use OR if you have more than one.
**/
#define BSCD_RX_PARITY                0x0800
#define BSCD_TX_PARITY                0x0400
#define BSCD_RX_TIMEOUT               0x0200
#define BSCD_TX_TIMEOUT               0x0100
#define BSCD_EDC_ERROR                0x0008
#define BSCD_HARDWARE_FAILURE         0x0004
#define BSCD_RESET_CHANNEL_REQUIRED   0x0002




/* Smart card module status codes */
#define BSCD_STATUS_ATR_SUCCESS      BERR_MAKE_CODE(BERR_SCD_ID, 0)  /* Succeed to Receive ATR */
#define BSCD_STATUS_ATR_FAILED       BERR_MAKE_CODE(BERR_SCD_ID, 1)  /* Failed to Receive ATR  */
#define BSCD_STATUS_READ_SUCCESS     BERR_MAKE_CODE(BERR_SCD_ID, 2)  /* Succeed to Receive data from the smartcard */
#define BSCD_STATUS_READ_FAILED      BERR_MAKE_CODE(BERR_SCD_ID, 3)  /* Failed to Receive data from the smartcard */
#define BSCD_STATUS_SEND_SUCCESS     BERR_MAKE_CODE(BERR_SCD_ID, 4)  /* Succeed to Send data to the smartcard  */
#define BSCD_STATUS_SEND_FAILED      BERR_MAKE_CODE(BERR_SCD_ID, 5)  /* Return code for general success. */
#define BSCD_STATUS_WARM_RESET       BERR_MAKE_CODE(BERR_SCD_ID, 6)  /* Warm Reset is expected after the cold reset failed */
#define BSCD_STATUS_DEACTIVATE       BERR_MAKE_CODE(BERR_SCD_ID, 7)  /* Need to deactivate the device as soon as possible */
#define BSCD_STATUS_OTHER_ERROR      BERR_MAKE_CODE(BERR_SCD_ID, 8)  /* Receive other error in T=1 */
#define BSCD_STATUS_LOOPBACK_DONE    BERR_MAKE_CODE(BERR_SCD_ID, 9)  /* Done with the EMV Loopback test */
#define BSCD_STATUS_TIME_OUT         BERR_MAKE_CODE(BERR_SCD_ID, 10) /* General Time out */
#define BSCD_STATUS_PARITY_EDC_ERR   BERR_MAKE_CODE(BERR_SCD_ID, 11) /* Receive a parity or EDC error */
#define BSCD_STATUS_NO_SC_RESPONSE   BERR_MAKE_CODE(BERR_SCD_ID, 12) /* SmartCard does not response */
#define BSCD_STATUS_FAILED           BERR_MAKE_CODE(BERR_SCD_ID, 13) /* Return code for general failure. */


/* End of Definitions */


/* Enum Types */

/***************************************************************************
Summary:
This enum represents all the Smartcard Interrupt Types.

Description:
This enumeration defines all the Smartcard Interrupt Types.

See Also:


****************************************************************************/
typedef enum BICM_IntrType {
      BICM_IntrType_eTParityInt,     /* Transmit Parity Interrupt. Only for T=0 */
      BICM_IntrType_eTimerInt,       /* General-purpose Timer Interrupt */
      BICM_IntrType_eCardInsertInt,  /* Card Insertion Interrupt */
      BICM_IntrType_eRemoveInt,      /* Card Removal Interrupt */
      BICM_IntrType_eBGTInt,         /* Block Guard Time Interrupt */
      BICM_IntrType_eTDoneInt,       /* Transmit Done Interrupt */
      BICM_IntrType_eRetryInt,       /* Transmit or Receive Retry Overflow Interrupt */
      BICM_IntrType_eTEmptyInt,      /* Transmit Empty Interrupt. Only used for debugging */
      BICM_IntrType_eRParityInt,     /* Receive Parity Interrupt. Only for T=0 */
      BICM_IntrType_eATRInt,         /* ATR Start Interrupt */
      BICM_IntrType_eCWTInt,         /* CWT Interrupt */
      BICM_IntrType_eRLenInt,        /* Receive Length Error Interrupt. Only for T=1 */
      BICM_IntrType_eWaitInt,        /* Block or Work Waiting Time Interrupt */
      BICM_IntrType_eRcvInt,         /* Receive Character Interrupt */
      BICM_IntrType_eRReadyInt,      /* Receive Ready Interrupt. Only for T=1 */
      BICM_IntrType_eEDCInt          /* EDC Error. Yet to be implemented. */
} BICM_IntrType;

/***************************************************************************
Summary:
This enum is to identify the smart card interface.

Description:
This enumeration defines the smart card channels.  System should call BCHP_GetFeature
to discover the maximum number of supported channels.

See Also:


****************************************************************************/
typedef enum BSCD_ChannelNumber {
		BSCD_ChannelNumber_e0 = 0,  /* first Smart Card Interface */
		BSCD_ChannelNumber_e1,      /* second Smart Card Interface */
		BSCD_ChannelNumber_e2       /* third Smart Card Interface */
} BSCD_ChannelNumber;


/***************************************************************************
Summary:
This enum is to identify the smart card T=0 or T=1 protocol.

Description:
This enumeration defines the smart card asynchronous and synchronous
transmission protocol.

See Also:


****************************************************************************/
typedef enum BSCD_AsyncProtocolType {
		BSCD_AsyncProtocolType_eUnknown = 0, /* Initial value */
		BSCD_AsyncProtocolType_e0,           /* Character-oriented asynchronous transmission protocol */
		BSCD_AsyncProtocolType_e1,           /* Block-oriented asynchronous transmission protocol */
		BSCD_AsyncProtocolType_e0_SYNC,      /* Type 1 synchronous transmission protocol */
		BSCD_AsyncProtocolType_e1_SYNC,      /* Type 2 synchronous transmission protocol */
		BSCD_AsyncProtocolType_e14_IRDETO    /* Irdeto T=14 proprietary transmission protocol */
} BSCD_AsyncProtocolType;


/***************************************************************************
Summary:
This enum is to identify the smart card type.

Description:
This enumeration defines the smart card type supported.

See Also:


****************************************************************************/
typedef enum BSCD_CardType {
                BSCD_CardType_eUnknown = 0, /* Initial value */
                BSCD_CardType_eACOS,        /* ACOS smart card by Advanced Card Systems */
                BSCD_CardType_eJAVA,        /* Java card */
                BSCD_CardType_ePKI          /* PKI card */

} BSCD_CardType;

/***************************************************************************
Summary:
This enum is to identify the source of the smartcard clock frequency.

Description:
This enumeration defines the smart card external or internal clock frequency.

See Also:


****************************************************************************/
typedef enum BSCD_ClockFreqSrc {
		BSCD_ClockFreqSrc_eUnknown = 0,       /* Initial Value */
		BSCD_ClockFreqSrc_eInternalClock = 1, /* Internal Clock. Should be 27MHz on BCM5892, 40MHz on Cygnus*/
		BSCD_ClockFreqSrc_eExternalClock = 2  /* External Clock.  Must further
							 specify the clock frequency in Hz */
} BSCD_ClockFreqSrc;


/***************************************************************************
Summary:
This enum is to identify the smart card standards.

Description:
This enumeration defines the supported smart card standards.

See Also:


****************************************************************************/
typedef enum BSCD_Standard {
		BSCD_Standard_eUnknown = 0, 	/* Initial Value */
		BSCD_Standard_eNDS,  		/* NDS. T=0 with flow control. */
		BSCD_Standard_eISO,      	/* ISO 7816. T=0 or T=1*/
		BSCD_Standard_eEMV1996,  	/* EMV. T=0 or T=1 */
		BSCD_Standard_eEMV2000,  	/* EMV. T=0 or T=1 */
		BSCD_Standard_eIrdeto,		/* Irdeto. T=14.  Need Major software workarouond to support this */
		BSCD_Standard_eARIB,		/* ARIB. T=1, Obsolete. Use ISO */
		BSCD_Standard_eMT,             	/* MT, T=??.  Obsolete. Use ISO */
		BSCD_Standard_eConax,          	/* Conax, T=??.  Obsolete. Use ISO */
		BSCD_Standard_eES             	/* ES, T=1.  Obsolete. Use ISO */
} BSCD_Standard;


/***************************************************************************
Summary:
This enum is to identify error detect code (EDC) encoding.

Description:
This enumeration defines the supported error detect code (EDC) encoding .

See Also:


****************************************************************************/
typedef enum BSCD_EDCEncode {
		BSCD_EDCEncode_eLRC = 0,   	/* 1-byte LRC */
		BSCD_EDCEncode_eCRC        	/* 2-byte CRC */
} BSCD_EDCEncode;

/***************************************************************************
Summary:
This enum is to identify action for BSCD_Channel_ResetIFD function.

Description:
This enumeration defines the supported action for  BSCD_Channel_ResetIFD function.

See Also:
BSCD_Channel_PowerICC

****************************************************************************/
typedef enum BSCD_ResetType {
 	BSCD_ResetType_eCold = 0,     /* Cold Reset */
	BSCD_ResetType_eWarm = 1      /* Warm Reset */

} BSCD_ResetType;

/***************************************************************************
Summary:
This enum is to identify action for BSCD_Channel_PowerICC function.

Description:
This enumeration defines the supported action for  BSCD_Channel_PowerICC function.

See Also:
BSCD_Channel_PowerICC

****************************************************************************/
typedef enum BSCD_PowerICC {
	BSCD_PowerICC_ePowerUp = 0,    /* power up the ICC and request
      					  activation of the contact */
	BSCD_PowerICC_ePowerDown = 1   /* power down the ICC and request
      					  deactivation of the contact */
} BSCD_PowerICC;

/***************************************************************************
Summary:
This enum is to identify voltage level for BSCD_Channel_SetVccLevel function.

Description:
This enumeration defines the supported action for  BSCD_Channel_SetVccLevel function.

See Also:
BSCD_Channel_SetVccLevel

****************************************************************************/
typedef enum BSCD_VccLevel {
	BSCD_VccLevel_e5V  = 0,  /* 5v is default value */
	BSCD_VccLevel_e3V  = 1,  /* 3v  */
	BSCD_VccLevel_e18V = 2   /* 1.8v */
} BSCD_VccLevel;

/***************************************************************************
Summary:
This enum is to identify action for BSCD_Channel_ResetCard function.

Description:
This enumeration defines the supported action for  BSCD_Channel_ResetCard function.

See Also:
BSCD_Channel_ResetCard

****************************************************************************/
typedef enum BSCD_ResetCardAction{
	BSCD_ResetCardAction_eNoAction = 0,    	   /* Only Reset the card, do not return ATR data.
						      Use BSCD_Channel_Receive to read the ATR data */
	BSCD_ResetCardAction_eReceiveAndDecode = 1 /* Reset the card , read and decode ATR and
						      program the registers accordingly. Caller still has
                                                      to call  BSCD_Channel_Receive to read the ATR data */
} BSCD_ResetCardAction;


/***************************************************************************
Summary:
This enum is to identify the unit of timer value.

Description:
This enumeration defines the supported unit of timer value.

See Also:


****************************************************************************/
typedef enum BSCD_TimerUnit {
		BSCD_TimerUnit_eETU = 0,  	/* in Elementary Time Units */
		BSCD_TimerUnit_eCLK,      	/* in raw clock cycles that smart card receives */
		BSCD_TimerUnit_eMilliSec 	/* in milli seconds */
} BSCD_TimerUnit;


/***************************************************************************
Summary:
This enum is to identify which timer register we are accessing.

Description:
This enumeration defines the supported timer register we are accessing.

See Also:


****************************************************************************/
typedef enum BSCD_TimerType {
		BSCD_TimerType_eGPTimer = 0,  /* General Purpose Timer */
		BSCD_TimerType_eWaitTimer     /* Wait Timer */
} BSCD_TimerType;


/***************************************************************************
Summary:
This enum is to identify the General-purpose Timer Start Mode in SC_TIMER_CMD
register.

Description:
This enumeration defines the General-purpose Timer Start Mode in SC_TIMER_CMD
register.

See Also:


****************************************************************************/
typedef enum BSCD_GPTimerMode {
		BSCD_GPTimerMode_eIMMEDIATE = 0,   /* Start Timer Immediate */
		BSCD_GPTimerMode_eNEXT_START_BIT   /* Start Timer on Next Start Bit */
} BSCD_GPTimerMode;


/**
  This enum is used to identify
**/
/***************************************************************************
Summary:
This enum is to identify the Waiting Timer Mode in SC_TIMER_CMD register.

Description:
This enumeration defines the Waiting Timer Mode in SC_TIMER_CMD register.

See Also:


****************************************************************************/
typedef enum BSCD_WaitTimerMode {
		BSCD_WaitTimerMode_eWorkWaitTime = 0,   /* Work Waiting Time Mode */
		BSCD_WaitTimerMode_eBlockWaitTime       /* Block Waiting Time Mode */
} BSCD_WaitTimerMode;


/***************************************************************************
Summary:
This enum is to identify the card is inserted or removed.

Description:
This enumeration defines the insertion or removal of the card.

See Also:


****************************************************************************/
typedef enum BSCD_CardPresent {
		BSCD_CardPresent_eRemoved = 0,   /* card removed */
		BSCD_CardPresent_eInserted       /* card inserted */
}  BSCD_CardPresent;


/***************************************************************************
Summary:
This enum is to identify read or write a specific register.

Description:
This enumeration defines read or write of a smart card register.

See Also:


****************************************************************************/
typedef enum BSCD_ScPresMode {
		BSCD_ScPresMode_eDebounce = 0,
		BSCD_ScPresMode_eMask
}  BSCD_ScPresMode;


/***************************************************************************
Summary:
This enum is to identify read or write a specific register.

Description:
This enumeration defines read or write of a smart card register.

See Also:


****************************************************************************/
typedef enum BSCD_RegAcessAction {
		BSCD_RegAcessAction_eRead = 0,
		BSCD_RegAcessAction_eWrite
}  BSCD_RegAcessAction;



/* End of Enum Types */


/* Smart Card Public Data Structures */

/***************************************************************************
Summary:
This struct specifies if we are intereseted in General-Purpose timer
or Wait Timer.

Description:
This structure defines if we are intereseted in General-Purpose timer
or Wait Timer.

See Also:


****************************************************************************/
typedef struct BSCD_Timer
{
		BSCD_TimerType        eTimerType;   /* timer type */
		union
		{
			BSCD_GPTimerMode         eGPTimerMode;  /* General-purpose
								   Timer Start Mode */
			BSCD_WaitTimerMode       eWaitTimerMode;  /* Waiting Timer Mode */
		} timerMode;   /* timer mode */

		bool	bIsTimerEnable;            /* enable or disable */
		bool    bIsTimerInterruptEnable;   /* enbale or disable timer interrupt */
} BSCD_Timer;


/***************************************************************************
Summary:
The timer value that application set to or get from the device.

Description:
This structure defines timer value that application set to or get from the device.

See Also:


****************************************************************************/
typedef struct BSCD_TimerValue
{
		uint32_t      ulValue;   /* timer value */
		BSCD_TimerUnit   unit;   /* units */
} BSCD_TimerValue;


/***************************************************************************
Summary:
The register address and value that application reads from or write to
the device

Description:
This structure defines value that application reads from or write to a
smart card.

See Also:


****************************************************************************/
typedef struct BSCD_RegAccessInfo
{
		BSCD_RegAcessAction action;  /* Read or Write */
		uint32_t       ulAddr; 	     /* Register Address */
		uint32_t       ulData; 	     /* Value in the above specified register address */
} BSCD_RegAccessInfo;


/***************************************************************************
Summary:
The source of smartcard clock frequency and number of clock
frequency in Hz if external clock is used.

Description:
The source of smartcard clock frequency and number of clock
frequency in Hz if external clock is used.

See Also:


****************************************************************************/
typedef struct BSCD_ClockFreq
{
		BSCD_ClockFreqSrc   FreqSrc;   /* source of clock frequency */
		uint32_t           ulClkFreq;  /* Clock frequency in Hz. Only
						  speficied this when external
						  clock is used. For Internal clock,
						  this is always 27000000 Hz. */
} BSCD_ClockFreq;


/***************************************************************************
Summary:
The configuration of EDC setting for T=1 protocol only.

Description:
This structure specifies what EDC enconding shall be used and whether
hardware shall handle the EDC automatically.

See Also:


****************************************************************************/
typedef struct BSCD_EDCSetting
{
		BSCD_EDCEncode   edcEncode;   /* EDC encoding */

		bool             bIsEnabled;  /* True if enabled and
						 hardware shall compute
						 the EDC byte(s) and append
						 it as the last byte when transmitting.
						 Hardware shall verify EDC byte(s)
						 when receiving.
						 Otherwise, it is false and application
						 shall compute and append the EDC
						 byte(s) when transmitting and verify
						 EDC byte(s) when receiving. */
} BSCD_EDCSetting;


/***************************************************************************
Summary:
The configuration of SC_Pres Debounce.

Description:
This structure specifies how to configure SC_Pres Debounce in IF_CMD_2.
See Also:


****************************************************************************/
typedef struct BSCD_ScPresDbInfo
{
		BSCD_ScPresMode	scPresMode;   /* EDC encoding */

		bool   	        bIsEnabled;   /* True if enabled. Otherwise it should be false*/

		unsigned char	ucDbWidth;    /* db_width[5:0] */

} BSCD_ScPresDbInfo;


/***************************************************************************
Summary:
The status of the device that driver will return  to the application.

Description:
This structure defines the status of the device that driver will return
to the application.

See Also:


****************************************************************************/
typedef struct BSCD_Status {
		bool      bCardPresent;  /* true if card present. Otherwise it is false. */
		bool      bCardActivate; /* Has card ever been reset (e.g. get ATR) */
		bool      bPPSDone;      /* if PPS done or not */
		uint32_t  ulStatus1;     /* status reported back to application */
		uint32_t  ulStatus2;   	 /* reserved */

} BSCD_Status;


/***************************************************************************
Summary:
This struct specifies the card characterics.

Description:
This structure defines various card characterics supported.

See Also:


****************************************************************************/
/* This is JAVA card specific */
typedef struct BSCD_JAVA_CTX
{
        uint16_t privateDataSize;   /* Private data size */
        uint16_t publicDataSize;    /* Public data size */
} BSCD_JAVA_CTX;

typedef struct BSCD_ACOS_userFile
{
     uint8_t  RecordLen;
     uint8_t  RecordNo;
     uint8_t  ReadAttr;
     uint8_t  WriteAttr;
     uint16_t FileID;
} BSCD_ACOS_userFile;

/* This is ACOS card specific */
typedef enum {SC_ACOS2, SC_ACOS3, SC_NON_ACOS} PHX_SC_CARD;
typedef struct BSCD_ACOS_CTX
{
        PHX_SC_CARD   nCard;
        uint32_t nCardStage;
        uint8_t  regOption;
        uint8_t  regSecurity;
        uint8_t  nNumOfFiles;
        uint8_t  bPersonalized;         /* if the card has personalization done or not */
        uint8_t  serialNumString[8];    /* card serial number */
        uint32_t nRecordMaxLen;         /* user file max record length */
        uint32_t nRecordMaxNum;         /* user file max record number */
        uint8_t  CodeNo;                /* Code reference */
        BSCD_ACOS_userFile userFile;    /* User data file */
} BSCD_ACOS_CTX;

#define BSCD_MAX_OBJ_ID_STRING  20
typedef struct BSCD_CardObjectID
{
        uint8_t  idNum;
        uint8_t  idString[BSCD_MAX_OBJ_ID_STRING];
} BSCD_CardObjectID;

#define BSCD_MAX_OBJ_ID_SIZE  5
typedef struct BSCD_CardType_Ctx
{
        BSCD_CardType eCardType;   /* Type of card */
        BSCD_VccLevel eVccLevel;   /* Vcc voltage level */
        uint32_t inited;
                union
                {
                   BSCD_JAVA_CTX        JAVACtx;
                   BSCD_ACOS_CTX        ACOSCtx;
                } CardChar;        /* Card characterics */
        uint8_t currentID;
        BSCD_CardObjectID objID[BSCD_MAX_OBJ_ID_SIZE];
        uint8_t HistBytes[BSCD_MAX_ATR_HISTORICAL_SIZE];
} BSCD_Cardtype_Ctx;

/*
   ToDo:
   1) For every structure member, we need another structure member to
		indicate the value is being modified
   2) Supported attributes should be read-only.
*/

/***************************************************************************
Summary:
Required default settings structure for smart card module.

Description:
The default setting structure defines the default configure of
smart card.  Since BSCD has multiple channels, it also has
default settings for a  channel.

See Also:
BSCD_ChannelSettings, BSCD_Open()

****************************************************************************/
struct BSCD_Settings
{
		BSCD_ClockFreq	moduleClkFreq;   /* This attribute indicates
						the source of clock and the value */

		unsigned char	ucMaxChannels;   /* maximum SCD channels supported */

} ;

/***************************************************************************
Summary:
Required default settings structure for smart card type.

Description:

See Also:

****************************************************************************/
struct BSCD_CardType_Settings
{
        BSCD_CardType eCardType;       /* Type of card */
        unsigned int  CardIDLength;    /* Length used to identify card */
        unsigned char HistBytes[BSCD_MAX_ATR_HISTORICAL_SIZE]; /* Historical bytes */

} ;

/***************************************************************************
Summary:
Required default settings structure for smart card channel.

Description:
The default setting structure defines the default configure of
smart card.

See Also:
BSCD_OpenChannel()

****************************************************************************/
struct BSCD_ChannelSettings
{
	BSCD_Standard           scStandard;      /* Smart Card Standard */
	BSCD_AsyncProtocolType  eProtocolType;   /* Asynchronous Protocol Types. */
	BSCD_Cardtype_Ctx       ctxCardType;     /* Card characteristics */
	unsigned long           srcClkFreqInHz;  /* This read-only attribute specifies the source clock
					            frequency in Hz. ??? May not need this */
	unsigned long           currentICCClkFreq; /* This read-only current ICC CLK frequency in Hz which is
						      source freq / SC_CLK_CMD[etu_clkdiv] /
						      SC_CLK_CMD[sc_clkdiv]/ external_clock_div */
	unsigned long           currentBaudRate; /* This read-only current baudrate which is source freq / SC_CLK_CMD[etu_clkdiv] /
						  / (SC_PRESCALE * external_clock_div + (external_clock_div - 1)) /SC_CLK_CMD[bauddiv] */
	unsigned long           unMaxIFSD;     /* This read-only attribute specifies the maximum IFSD. Should be 264. */
	unsigned long           unCurrentIFSD; /* This attribute indicates the current IFSD */
	unsigned char           ucFFactor;     /* Clock Rate Conversion Factor,
					      	  F in 1,2,3,4,5,6,9, 10, 11, 12 or 13. Default is 1. */
	unsigned char           ucDFactor;     /* Baud Rate Adjustment Factor,
						  D in 1,2,3,4,5,6,8 or 9. Default is 1. */
	unsigned char           ucEtuClkDiv;   /* ETU Clock Divider in SC_CLK_CMD register. Valid value is
						  from 1 to 8. Default is 6. */
	unsigned char           ucScClkDiv;    /* SC Clock Divider in SC_CLK_CMD register. Valid value is
                                                  1,2,3,4,5,8,10,16. Default is 1. */
	unsigned long           unPrescale;    /* Prescale Value.  This value is the final value that already
                                                  takes external clock divisor into account */
	unsigned char           ucExternalClockDivisor;   /* external clock divisor, could 1, 2, 4, 8 */
	unsigned char           ucBaudDiv;    /* baud divisor , could be 31 or 32 */
	unsigned char           ucTxRetries;  /* Number of transmit parity retries per character in SC_UART_CMD_2 register.
                                                 Default is 4 and max is 6. 7 indicates infinite retries */
	unsigned char           ucRxRetries;  /* Number of receive parity retries per character in SC_UART_CMD_2 register.
						 Default is 4 and max is 6. 7 indicates infinite retries */
	BSCD_TimerValue         workWaitTime; /* work waiting time in SC_TIME_CMD register.
						 Other than EMV standard, only valid if current protocol is T=0. */
	BSCD_TimerValue         blockWaitTime;  /* block Wait time in SC_TIME_CMD register. Only valid if current protocol is T=1. */
	BSCD_TimerValue         extraGuardTime;	/* Extra Guard Time in SC_TGUARD register. */
	BSCD_TimerValue         blockGuardTime; /* block Guard time in SC_BGT register. Other than EMV standard,
                                                   only valid if current protocol is T=1.  */
	unsigned long          	ulCharacterWaitTimeInteger;  /* character Wait time Integer in SC_PROTO_CMD register.
                                                                 Only valid if current protocol is T=1. */
	BSCD_EDCSetting	        edcSetting;        /* EDC encoding. Only valid if current protocol is T=1. */
	BSCD_TimerValue         timeOut;   	   /* arbitrary Time Out value in milli seconds for any synchronous transaction.  */
	bool                   	bAutoDeactiveReq;  /* Specify if we need auto deactivation sequence */
	bool                    bNullFilter;	   /* True if we receive 0x60 in T=0, we will ignore it.
                                                      Otherwise, we treat 0x60 as a valid data byte */
	BSCD_ScPresDbInfo       scPresDbInfo;      /* Debounce info for IF_CMD_2 */
	BSCD_ResetCardAction	resetCardAction;   /* Specify if we want the driver to read, decode and program registers */
	BSCD_TimerValue	        blockWaitTimeExt;  /* Specify the block wait time extension */
	bool                    bTPDU;	           /* True if packet is in TPDU rather than APDU */
	uint8_t                 unCurrentIFSC;     /* current IFSC size (T=1 only) */
};


/* End of Smart Card Public Data Structures */


#ifdef __cplusplus
}
#endif

#endif /* BSCD_DATATYPES_H__ */
