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

#include "bstd.h"
#include "bkni.h"
#include "bchp_sca.h"
#include "bscd.h"
#include "bscd_priv.h"
#include "bscd_isopriv.h"


BDBG_MODULE(BSCD);

//#define SCI_DEBUG 0     /* define this in Makefile */

#ifndef SCI_DEBUG
#undef BDBG_ENTER
#undef BDBG_LEAVE
#define BDBG_ENTER(...)
#define BDBG_LEAVE(...)
#endif

BERR_Code BSCD_Channel_P_ISOATRReadNextByte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char *outp_ucData
)
{
	BERR_Code		errCode = BERR_SUCCESS;
        unsigned long       ulRxLen = 0;

	BDBG_ENTER(BSCD_Channel_P_ISOATRReadNextByte);

	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulRxLen  ==  BSCD_MAX_ATR_SIZE ) );

        BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
                                BSCD_Channel_ReceiveATR(in_channelHandle,
                                                        outp_ucData, &ulRxLen, 1));

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOATRReadNextByte);
	BDBG_MSG(("Leave  ISOATRReadNextByte errCode = 0x%x\n", errCode));
	return( errCode );
}


BERR_Code BSCD_Channel_P_ISOATRCheckForAdditionalATRBytes(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code		errCode = BERR_SUCCESS;
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	uint32_t		ulVal;
#endif
	unsigned char       bval;
	unsigned long       ulTotalAtrByteTimeInETU = 0;
	unsigned int        unAdditionalByteCount = 0;

	BDBG_ASSERT( in_channelHandle );
	BSCD_P_CHECK_ERR_CODE_CONDITION( errCode, BSCD_STATUS_FAILED,
		(in_channelHandle->ulMagicNumber != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER ) );

	BDBG_MSG(("In  BSCD_Channel_P_ISOATRCheckForAdditionalATRBytes\n"));
	while (errCode == BERR_SUCCESS) {

		/*
			Since GP is used for total ATR time and WWT is used for WWT, we use
			event2_intr for this 200 ETU checking
		*/

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
		/* 200 ETU */
		BKNI_RegWrite32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT2_CMP), 200);

		/* start event src */
		BKNI_RegWrite32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT2_CMD_3), BSCD_P_RX_ETU_TICK_EVENT_SRC);

		/* increment event src */
		BKNI_RegWrite32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT2_CMD_2), BSCD_P_RX_ETU_TICK_EVENT_SRC);

		/* reset event src */
		BKNI_RegWrite32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT2_CMD_1), BSCD_P_NO_EVENT_EVENT_SRC);

		/* event_en, intr_mode, run_after_reset and run_after_compare*/
		ulVal = BCHP_SCA_SC_EVENT2_CMD_4_event_en_MASK |
				BCHP_SCA_SC_EVENT2_CMD_4_intr_after_compare_MASK |
				BCHP_SCA_SC_EVENT2_CMD_4_run_after_reset_MASK |
				BCHP_SCA_SC_EVENT2_CMD_4_run_after_compare_MASK;

		ulVal &= ~(BCHP_SCA_SC_EVENT2_CMD_4_intr_after_reset_MASK);
		BKNI_RegWrite32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT2_CMD_4), ulVal);


		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
						BSCD_Channel_EnableIntrCallback_isr (
							in_channelHandle, BSCD_IntType_eEvent2Int,
				               		BSCD_Channel_P_Event2CB_isr));

#endif

		errCode = BSCD_Channel_P_ISOATRByteRead(in_channelHandle, &bval, 200,
					BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES,
					&ulTotalAtrByteTimeInETU);

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			/* Disable event2 */
			ulVal = BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT2_CMD_4));
			ulVal &= ~(BCHP_SCA_SC_EVENT2_CMD_4_event_en_MASK);
			BKNI_RegWrite32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_EVENT2_CMD_4), ulVal);
#endif

		if (errCode == BERR_SUCCESS) {
			BDBG_MSG(("In  SmartCardATRCheckForAdditionalATRBytes: Extra Byte Detected\n"));
			unAdditionalByteCount++;
		}
	}


	if (unAdditionalByteCount)
		errCode = BSCD_STATUS_FAILED;


BSCD_P_DONE_LABEL:

	return errCode;
}



BERR_Code BSCD_Channel_P_ISOATRReceiveAndDecode(
		BSCD_ChannelHandle	in_channelHandle
)
{
	BERR_Code		errCode = BERR_SUCCESS;
	unsigned int          unNumHistoricalBytes = 0;
	unsigned char         ucT0Byte = 0, ucTD1Byte = 0, ucTD2Byte = 0, ucTD3Byte = 0, ucTC1Byte = 0;
	unsigned int          i;
	bool        bTCKRequired = false;


	BDBG_ENTER(BSCD_Channel_P_ISOATRReceiveAndDecode);

	/* assign negotiate settings to default */
	BSCD_GetChannelDefaultSettings(NULL, 0, &(in_channelHandle->negotiateChannelSettings));

	/* TS */
	in_channelHandle->ulRxLen = 0;
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
	                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_P_ISOValidateTSByte(
					in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));

	/* T0 */
	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
	                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
					&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

	BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
	                        BSCD_Channel_P_ISOValidateT0Byte(
					in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen],
					&unNumHistoricalBytes));

	ucT0Byte = in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++];

	in_channelHandle->bIsPPSNeeded = false; /* default to NOT need PPS */

	if (ucT0Byte  & 0x10 ) { /* TA1 is present */

		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));
		BDBG_MSG(("After T0 ISOATRReadNextByte errCode = 0x%x\n", errCode));


		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTA1Byte(in_channelHandle,
		                                in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));

	}
	/* if TA1 is absent, no need to do since already set to default in BSCD_GetChannelDefaultSettings() */


	if (ucT0Byte & 0x20) {  /* TB1 is present */

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTB1Byte(in_channelHandle,
		                                   in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));
	}

	if (ucT0Byte & 0x40) {  /* TC1 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));
		ucTC1Byte =  in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++];
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTC1Byte(in_channelHandle, ucTC1Byte));

	}
	/* if TC1 is absent from the ATR, default is 2etu (BSCD_DEFAULT_EXTRA_GUARD_TIME). Before we assign to 0 here */


	if (ucT0Byte & 0x80) {  /* TD1 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
	                                BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

		ucTD1Byte = in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++];  /* Store ucTD1Byte */

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTD1Byte(in_channelHandle,
		                                   ucTD1Byte, &bTCKRequired));
	}

	if (ucTD1Byte & 0x10) {  /* TA2 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTA2Byte(in_channelHandle,
		                                in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));
	}

	if (ucTD1Byte & 0x20) {  /* TB2 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTB2Byte(
		                                in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));
	}

		/* do not check err return, otherwise we would fail for the OberthurPIVv1.08 card which used RFU value of D=7 */
        BSCD_P_FDAdjust_WithoutRegisterUpdate(&(in_channelHandle->negotiateChannelSettings),
                            in_channelHandle->negotiateChannelSettings.ucFFactor, in_channelHandle->negotiateChannelSettings.ucDFactor);

        in_channelHandle->negotiateChannelSettings.currentBaudRate =
                        in_channelHandle->moduleHandle->currentSettings.moduleClkFreq.ulClkFreq /
                                        in_channelHandle->negotiateChannelSettings.ucEtuClkDiv/
                                        (in_channelHandle->negotiateChannelSettings.unPrescale+1)/
                                        in_channelHandle->negotiateChannelSettings.ucBaudDiv;


	if (ucTD1Byte & 0x40) { /* TC2 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTC2Byte(in_channelHandle,
		                                 in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));
	}

	if (ucTD1Byte & 0x80) {  /* TD2 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));
		ucTD2Byte = in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++];  /* Store ucTD1Byte */

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTD2Byte(in_channelHandle, ucTD2Byte,
		                                 ucTD1Byte, &bTCKRequired));
	}

	if (ucTD2Byte & 0x10) {  /* TA3 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTA3Byte(in_channelHandle,
		                                in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));
	}

	if (ucTD2Byte & 0x20) { /* TB3 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
	                        	BSCD_Channel_P_ISOValidateTB3Byte(in_channelHandle,
		                                in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++],
				                in_channelHandle->negotiateChannelSettings.ucFFactor,
				                in_channelHandle->negotiateChannelSettings.ucDFactor));

	}

	if (ucTD2Byte & 0x40) { /* TC3 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOValidateTC3Byte(
		                                 in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]));
	}


	if (ucTD2Byte & 0x80) {  /* TD3 is present */

                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

		ucTD3Byte = in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++];  /* Store ucTD1Byte */

		/*
		  BSCD_Channel_P_ISOValidateTD3Byte(in_channelHandle, ucTD2Byte,
		           ucTD1Byte, &bTCKRequired);
		*/
	}

	if (ucTD3Byte & 0x10) {  /* TA4 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++])));

		/*
		  BSCD_Channel_P_ISOValidateTA4Byte(in_channelHandle,
		           in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]);
		*/
	}

	if (ucTD3Byte & 0x20) { /* TB4 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++])));

		/*
		  BSCD_Channel_P_ISOValidateTB4Byte(in_channelHandle,
		           in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++],
				in_channelHandle->negotiateChannelSettings.ucFFactor,
				in_channelHandle->negotiateChannelSettings.ucDFactor);
		*/

	}

	if (ucTD3Byte & 0x40) { /* TC4 is present */
                BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                        BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++])));

		/*
		  BSCD_Channel_P_ISOValidateTC3Byte(
		           in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++]);
		*/
	}


	if (unNumHistoricalBytes) {
                unsigned char   ucHistoricalBytes[50]; /* 15 was very risky, there are cards with history bytes > 15 */

		for (i = 0; i < unNumHistoricalBytes; i++) {

                     BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
		                             BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
						&(in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)])));
                     ucHistoricalBytes[i] = in_channelHandle->aucRxBuf[(in_channelHandle->ulRxLen)++];
		}
	}

	if (bTCKRequired) {
		BDBG_MSG (("Checking for TCK Byte"));
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_P_ISOATRReadNextByte(in_channelHandle,
				&(in_channelHandle->aucRxBuf[in_channelHandle->ulRxLen])));

		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
				BSCD_Channel_P_ISOValidateTCKByte(in_channelHandle,
				in_channelHandle->aucRxBuf,  (in_channelHandle->ulRxLen)++));
	}


	/* Print ATR message. */
//	BSCD_CLI_HexDump("ATR Data", in_channelHandle->aucRxBuf, in_channelHandle->ulRxLen);

BSCD_P_DONE_LABEL:

	BDBG_MSG(("Leave ISOATRReceiveAndDecode  errCode = 0x%x\n", errCode));
	BDBG_LEAVE(BSCD_Channel_P_ISOATRReceiveAndDecode);
	return( errCode );

}

BERR_Code BSCD_Channel_P_ISOValidateTSByte(
		unsigned char in_ucTSByte
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	if (in_ucTSByte == 0x3f) {
		BDBG_MSG(("TS = %02x, Inverse Convention\n", in_ucTSByte));
		return errCode;
	}

	else if (in_ucTSByte == 0x3b) {
		BDBG_MSG(("TS = %02x, Direct Convention\n", in_ucTSByte));
		return errCode;
	}

	else {
		BDBG_ERR(("TS = %02x, Unknown Convention\n", in_ucTSByte));
		errCode = BERR_TRACE(BSCD_STATUS_FAILED);
		return BSCD_STATUS_FAILED;
	}
}

BERR_Code BSCD_Channel_P_ISOValidateT0Byte(
      unsigned char ucT0Byte,
      unsigned int  *unNumHistoricalBytes
)
{
	*unNumHistoricalBytes = (unsigned int)(ucT0Byte & 0x0F);
	return BERR_SUCCESS;
}

BERR_Code BSCD_Channel_P_ISOValidateTA1Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTA1Byte
)
{
	BERR_Code		errCode = BERR_SUCCESS;
	unsigned char       ucFfactor, ucDFactor;

	BDBG_ENTER(BSCD_Channel_P_ISOValidateTA1Byte);

	BDBG_MSG(("in_ucTA1Byte = %d\n", in_ucTA1Byte));

	/* Decode TA1 (F and D adjustment). */
	ucFfactor = (in_ucTA1Byte >> 4) & 0x0f;
	ucDFactor = in_ucTA1Byte & 0x0f;

	in_channelHandle->negotiateChannelSettings.ucFFactor = ucFfactor;
	in_channelHandle->negotiateChannelSettings.ucDFactor = ucDFactor;

	/* For Negotiable mode, if F/D are not default values, need to do PPS
	   For Specific mode there will be TA2 byte, bIsPPSNeeded can be overwritten there depends on TA2
	*/
	if ((ucFfactor != 1) || (ucDFactor != 1))
		in_channelHandle->bIsPPSNeeded = true;

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTA1Byte);
	return( errCode );

}

BERR_Code BSCD_Channel_P_ISOValidateTB1Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTB1Byte
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER((SmartCardValidateTB1Byte));

	/* Decode TB1 (programming voltage) */
	if (in_ucTB1Byte == 0x00) {
		/*
		BDBG_MSG(("VPP is not connected in the ICC\n"));
		BDBG_MSG(("No programming current\n"));
		*/
	}
	else {
		/*
		According to EMV ATR spec, in response to a warm reset,
		the terminal shall accept an ATR containing TB1 of any value
		*/
		if (in_channelHandle->resetType == BSCD_ResetType_eWarm) {
			/*
			BDBG_ERR(("VPP is not connected in the ICC\n"));
			BDBG_ERR(("No programming current\n"));
			*/
		}

		else {
			BDBG_ERR(("Non-Zero TB1 = %02x during cold reset.  Not acceptable for EMV.\n",in_ucTB1Byte));
			BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
		}
	}


BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTB1Byte);
	return( errCode );

}


BERR_Code BSCD_Channel_P_ISOValidateTC1Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTC1Byte
)
{
	BDBG_MSG(("In  SmartCardValidateTC1Byte\n"));

	/* Decode TC1 (guard time) */
	if ( (in_channelHandle->negotiateChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0) &&
		(in_ucTC1Byte == 0xff))  {

		/*
			When in T = 0 mode and ucTC1Byte == 0xff, simply set
			additional guardtime to zero ETUs.  This is to pass the test and it is different from EMV 96.
		*/
		in_channelHandle->negotiateChannelSettings.extraGuardTime.ulValue = 0;
	}
	else {
		/*
			use value of ucTC1Byte for additional guardtime,
			regardless of T = 0 or T = 1 mode
		*/
		in_channelHandle->negotiateChannelSettings.extraGuardTime.ulValue = in_ucTC1Byte;
	}

	in_channelHandle->negotiateChannelSettings.extraGuardTime.unit = BSCD_TimerUnit_eETU;
	BDBG_MSG(("\nSmartCardValidateTC1Byte: ulGuardTime = 0x%x \n",
			in_channelHandle->negotiateChannelSettings.extraGuardTime.ulValue));

	return BERR_SUCCESS;
}



BERR_Code BSCD_Channel_P_ISOValidateTD1Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTD1Byte,
		bool *outp_bTCKRequired
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER((BSCD_Channel_P_ISOValidateTD1Byte));

	if  ((in_ucTD1Byte & 0x0f) > 0x01) {

		/*
		If the lower nibble of ucTD1Byte is not equal to either 0
		or 1, then return fail, otherwise return success
		*/
		BDBG_ERR(("Erroneous TD1 l.s. nibble = %02x, should be either 0x00 or 0x01\n",in_ucTD1Byte));

		*outp_bTCKRequired = true;
		BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
	}

	else if ((in_ucTD1Byte & 0x0f) == 0x01) {

		/* set protocol type to T = 1 */
		in_channelHandle->negotiateChannelSettings.eProtocolType = BSCD_AsyncProtocolType_e1; // added by fye
		*outp_bTCKRequired = true;
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTD1Byte);
	return( errCode );
}

BERR_Code BSCD_Channel_P_ISOValidateTA2Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTA2Byte
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER((SmartCardValidateTA2Byte));

	/* check bit5, e.g. 0x10 */
	if (in_ucTA2Byte & 0x10) {
		BDBG_ERR(("Invalid TA2 = %02x, this uses implicit values \n",in_ucTA2Byte));
		BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
	}
	else {
		BDBG_MSG(("TA2 present with b5=0\n"));
		in_channelHandle->bIsPPSNeeded = false;
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTA2Byte);
	return( errCode );
}

BERR_Code BSCD_Channel_P_ISOValidateTB2Byte(
		unsigned char in_ucTB2Byte
)
{
	BSTD_UNUSED(in_ucTB2Byte);
	BDBG_MSG(("In  SmartCardValidateTB2Byte\n"));

	/* TB2 is not supported by EMV, therefore return failed */
	BDBG_ERR(("TB2 is present, but not required for Europay standard.  Invalid ATR\n"));

	return BSCD_STATUS_FAILED;
}

BERR_Code BSCD_Channel_P_ISOValidateTC2Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTC2Byte
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER((SmartCardValidateTC2Byte));
	BDBG_MSG(("In  BSCD_Channel_P_ISOValidateTC2Byte\n"));

	/* Decode TC2.  NOTE: TC2 is specific to protocol type T = 0 */
	if ((in_ucTC2Byte == 0x00)) { //fye: we take bigger values || (in_ucTC2Byte > 0x0A)

		/* Reject ATR if TC2 is equal to '0x00' or greater than '0x0A' */
		BDBG_ERR(("Invalid TC2 = %02x \n",in_ucTC2Byte));
		BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
	}

	else {
		/* Reset work waiting time, using valid TC2 Value */
		/* Specify work wait time used for T = 0 protocol */
		BSCD_P_CHECK_ERR_CODE_FUNC(errCode,
			BSCD_P_AdjustWWT(&(in_channelHandle->negotiateChannelSettings),
				in_ucTC2Byte));

	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTC2Byte);
	return( errCode );
}

BERR_Code BSCD_Channel_P_ISOValidateTD2Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTD2Byte,
		unsigned char in_ucTD1Byte,
		bool * outp_bTCKRequired
)
{
	BERR_Code		errCode = BERR_SUCCESS;
	unsigned char   ucT1, ucT2;

	BDBG_ENTER((BSCD_Channel_P_ISOValidateTD2Byte));

	BSTD_UNUSED(in_channelHandle);

	ucT1 = in_ucTD1Byte & 0x0F;
	ucT2 = in_ucTD2Byte & 0x0F;

	if (ucT1 == 0x00) {
		/* fye: we removed the error check here for TD2. If TD1 is T=0, TD2 can be any value */
		/* this is to fix the Oberthur ID One V5.2 used by DoD */

		if (ucT2 != 0x00) {
			/* If TD1 and TD2 are not same, TCK is needed */
			*outp_bTCKRequired = true;
		}
	}
	else if (ucT1 == 0x01) {

		if (ucT2 == 0x00) {
			/*
			If l.s. nibble of TD1 is '1', then l.s. nibble of TD2 must not be 0 according to standard
			*/
			BDBG_MSG(("Failing in SmartCardValidateTD2Byte - TD2 == 0x00 \n"));
			BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
		}
	}

	/* if two protocols offered (T=0 to 14), then need PPS */
	/* No need to compare >=0 since ucT1/2 are unsigned char */
	if ((ucT1 != ucT2) && (ucT1 <= 14) && (ucT2 <= 14))
		in_channelHandle->bIsPPSNeeded = true;

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTD2Byte);
	return( errCode );
}


BERR_Code BSCD_Channel_P_ISOValidateTA3Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTA3Byte
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER((BSCD_Channel_P_ISOValidateTA3Byte));

	/* Decode TA3 according to the protocol */
	if (in_channelHandle->negotiateChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e0) {
		/* TA3 codes UI(card supports clock stop or not) and XI(class indicator) */
		/* We dont hanlding this case for now */
	}
	else if (in_channelHandle->negotiateChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) {
			/* TA3 codes IFSC */
		if ((in_ucTA3Byte <= 0x0f) || (in_ucTA3Byte == 0xff))  {
			BDBG_ERR(("Invalid ISO TA3 = %02x \n",in_ucTA3Byte));
			BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
		}
		else {
				in_channelHandle->negotiateChannelSettings.unCurrentIFSC = in_ucTA3Byte;
		}
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTA3Byte);
	return( errCode );

}

BERR_Code BSCD_Channel_P_ISOValidateTB3Byte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char in_ucTB3Byte,
		unsigned char in_ucFFactor,
		unsigned char in_ucDFactor
)
{
	BERR_Code		errCode = BERR_SUCCESS;
	uint32_t  /* ulGuardTime,  ulGuardTimePlusOne,*/ ulCWTValue;
	unsigned char ucCWIVal, ucBWIVal;

	BDBG_ENTER((BSCD_Channel_P_ISOValidateTB3Byte));

	if (in_channelHandle->negotiateChannelSettings.eProtocolType == BSCD_AsyncProtocolType_e1) {
		/* Decode TB3. */
		ucCWIVal = in_ucTB3Byte & 0x0f;
		ucBWIVal = (in_ucTB3Byte >> 4) & 0x0f;

		/*
			Obtain the power(2,CWI) factor from the value of
			CWI - see TB3 in EMV'96 spec for more
		*/

                {

			/* Set CWT */
			ulCWTValue = BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_PROTO_CMD));

			/*
			and with 0xf0 to remove the original 0x0f inserted
			into this register
			*/
			ulCWTValue &= 0xf0;
			ulCWTValue |= ucCWIVal;
			in_channelHandle->negotiateChannelSettings.ulCharacterWaitTimeInteger = ucCWIVal;

			BDBG_MSG(("ulCWTValue = 0x%x\n", ulCWTValue));


			/* set BWT */
			BSCD_Channel_SetBlockWaitTimeInteger(&(in_channelHandle->negotiateChannelSettings), ucBWIVal);

			BDBG_MSG (("TB3, blockWaitTime = %ld\n",
				in_channelHandle->negotiateChannelSettings.blockWaitTime.ulValue));

		}
	} /* eProtocolType == BSCD_AsyncProtocolType_e1 */

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTB3Byte);
	return( errCode );
}

BERR_Code BSCD_Channel_P_ISOValidateTC3Byte(
		unsigned char in_ucTC3Byte
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	BDBG_ENTER((BSCD_Channel_P_ISOValidateTC3Byte));
	BDBG_MSG(("In BSCD_Channel_P_EMVValidateTC3Byte = 0x%02x\n", in_ucTC3Byte));

	/* Terminal shall reject ATR containing non-zero TC3 Byte */
	if (in_ucTC3Byte != 0) {
		BDBG_ERR(("Failing in SmartCardValidateTC3Byte \n"));
		BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
	}

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTC3Byte);
	return( errCode );

}

BERR_Code BSCD_Channel_P_ISOValidateTCKByte(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char *inp_ucATR,
		unsigned int in_unATRLength
)
{
	BERR_Code		errCode = BERR_SUCCESS;

	unsigned char ucTCKCompare = 0;
	unsigned int i;

	BDBG_ENTER((BSCD_Channel_P_ISOValidateTCKByte));
	BSTD_UNUSED(in_channelHandle);

	/* Start from T0 to TCK.  Including historical bytes if they exist */
	for (i=1; i<=in_unATRLength; i++) {
		ucTCKCompare = ucTCKCompare ^ inp_ucATR[i];
		BDBG_MSG(("In  SmartCardValidateTCKByte inp_ucATR[%d] = %02x\n", i, inp_ucATR[i]));
	}

	if (ucTCKCompare != 0) {
		BDBG_ERR(("Invalid TCK. \n"));
		BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_DEACTIVATE, true);
	}

	/* TCK validated successfully */
BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOValidateTCKByte);
	return( errCode );
}

BERR_Code BSCD_Channel_P_ISOATRByteRead(
		BSCD_ChannelHandle	in_channelHandle,
		unsigned char *inoutp_ucData,
		unsigned long in_ulMaxAtrByteTimeInETU,
		long in_lMaxTotalAtrByteTimeInETU,
		unsigned long *inoutp_ultotalAtrByteTimeInETU
)
{
	BERR_Code		errCode = BERR_SUCCESS;
	uint32_t        ulTimerCntVal1, ulTimerCntVal2, ulTimerCntVal;
	uint32_t 			ulStatus;
	BSCD_Timer 		timer = {BSCD_TimerType_eGPTimer, {BSCD_GPTimerMode_eIMMEDIATE}, false, false};
	/* BSCD_TimerValue    timeValue= {BSCD_MAX_ETU_PER_ATR_BYTE, BSCD_TimerUnit_eETU}; */

	BSCD_Timer 		wwtTimer = {BSCD_TimerType_eWaitTimer, {BSCD_GPTimerMode_eIMMEDIATE}, false, false};

	BDBG_ENTER(BSCD_Channel_P_ISOATRByteRead);

	BSTD_UNUSED(in_ulMaxAtrByteTimeInETU);

	BKNI_EnterCriticalSection();
	ulStatus = in_channelHandle->ulStatus2 = BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_STATUS_2));
	BDBG_MSG(("Status2 = 0x%x\n", ulStatus));
	BKNI_LeaveCriticalSection();

	if ((ulStatus & BCHP_SCA_SC_STATUS_2_rempty_MASK) == BCHP_SCA_SC_STATUS_2_rempty_MASK) {

		/* Do not have any byte in SC_RECEIVE */

		if ( (errCode = BSCD_Channel_P_WaitForRcv(in_channelHandle)) != BERR_SUCCESS) {

			/* disable the timer, always return the  previous error */

			/* Disable timer, which was enable upon receiving atr_intr */
			BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &timer);

			/* disable WWT.  This was enabled in activating time */
			wwtTimer.timerMode.eWaitTimerMode = BSCD_WaitTimerMode_eWorkWaitTime;
			BSCD_Channel_EnableDisableTimer_isr(in_channelHandle, &wwtTimer);

			/* Read timer counter and accumulate it to *inoutp_ultotalAtrByteTimeInETU */
			ulTimerCntVal2 =  BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CNT_2));

			ulTimerCntVal1=  BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_TIMER_CNT_1));

			ulTimerCntVal = (((unsigned int) ulTimerCntVal2) << 8) | ulTimerCntVal1;

			*inoutp_ultotalAtrByteTimeInETU += ulTimerCntVal;

			if (*inoutp_ultotalAtrByteTimeInETU > (unsigned long) in_lMaxTotalAtrByteTimeInETU) {
				BDBG_MSG(("SmartCardATRByteRead: inoutp_ultotalAtrByteTimeInETU = %lu , in_lMaxTotalAtrByteTimeInETU = %d\n",
						*inoutp_ultotalAtrByteTimeInETU, in_lMaxTotalAtrByteTimeInETU));
				BSCD_P_CHECK_ERR_CODE_CONDITION(errCode, BSCD_STATUS_FAILED, true);
			}

			BDBG_MSG(("After  WaitForRcv in ISOATRByteRead errCode = 0x%x\n", errCode));
			return errCode;
		}

	}

	else {
		BKNI_EnterCriticalSection();
		in_channelHandle->ulIntrStatus2 &= ~BCHP_SCA_SC_INTR_STAT_2_rcv_intr_MASK;
		BKNI_LeaveCriticalSection();
		BDBG_MSG(("Cancel out RCV_INTR, pSc_intrStatus2 = 0x%x\n",
			in_channelHandle->ulIntrStatus2));
	}

	*inoutp_ucData = (unsigned char) BKNI_RegRead32(in_channelHandle, (in_channelHandle->ulRegStartAddr + BSCD_P_RECEIVE));

	BDBG_MSG(("atr = 0x%x\n", *inoutp_ucData));

BSCD_P_DONE_LABEL:

	BDBG_LEAVE(BSCD_Channel_P_ISOATRByteRead);
	BDBG_MSG(("Leave ISOATRByteRead errCode = 0x%x\n", errCode));
	return( errCode );
}
