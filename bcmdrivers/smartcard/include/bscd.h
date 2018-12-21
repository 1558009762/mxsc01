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



/*================== Module Overview =====================================
<verbatim>

Module Overview
The Smart Card (SCD) API is a porting interface(PI) module which
allows status/control of smart card communication.



Usage
Since most of the chips and reference designs can support multiple identical and
completely indepedent smart card interfaces, SCD shall control multiple
smartcard channels with different settings concurrently. However, the whole system
should have only one module handle.

During system initialization, the system may first call BSCD_GetDefaultSettings
to acquire the default settings for the smart card module.  The sytem shall then
call BSCD_Open to create a module handle. It may call
BSCD_GetChannelDefaultSettings to retrieve the default settings for the
specific smart card channel and BSCD_Channel_Open to create a channel handle.
The sytem shall reset and configure the channel and  detect the smart card.
Upon receiving ATR (Answer To Reset) data after the sytem reset the smart card,
the system can then communcate with smart card associated with this
channel.  All the I/O data shall be automically processed according to the inverse
or direct  convention.   During system shutdown, the system shall call
BSCD_Channel_Close to close this channel and call BSCD_Close to close the
smart card module.

Each channel can support ISO 7816 asynchronous T=0 and T=1 modes with
264-byte UART receive and transmit buffers. Each channel can have different baud
rate but all channels have to use either internal clock or external clock.  SCD
could defines various smart card timing and error management registers to conform
to different standards with minimal CPU overhead and interrupt latency. SCD should
be able to support NDS, EMV 1996, ARIB and ISO smart card standards.

SCD is expected to be able to support various smart card standards, therefore it
is designed to be as much standard independent as possible.  Application is expected
to be standard awareness and to handle all the standard comformance requirements.

The caller usually first calls SCD to setup the related registers to receive ATR.
The caller then interprets ATR and setup the registers again for transmitting
and receiving data.  Since all these scenarioes occur sequentially and
synchronously, there is no reason for the caller to call more than one
SCD functions to access one channel concurrently with multiple threads.


Sample Code:
Please refer to the functions BSCD_CLI_T0Test( ) or BSCD_CLI_T1Test( ) in
bscd_cli_infra.c.


</verbatim>
***************************************************************************/


#ifndef BSCD_H__
#define BSCD_H__

#include "bint.h"
#include "bscd_datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************
Summary:
SCD channel context handle.

Description:
Opaque handle that is created in BSCD_Channel_Open.
BSCD_ChannelHandle holds the context of the smart card channel.  The system
could have more than one BSCD_ChannelHandle if the chip can support multiple
smartcard interfaces/channels.  Caller of BSCD_Channel_Open is responsible
to store this BSCD_ChannelHandle and uses it for the future function call after
BSCD_Channel_Open function returns successfully.

See Also:
BSCD_Open, BSCD_Channel_Open

****************************************************************************/
typedef struct BSCD_P_ChannelHandle     *BSCD_ChannelHandle;


/***************************************************************************
Summary:
Required default settings structure for smart card module.

Description:
The default setting structure that defines the default configure of
smart card module when the module is initialized.

See Also:
BSCD_GetDefaultSettings, BSCD_Open.

****************************************************************************/
typedef struct BSCD_Settings          BSCD_Settings;


/***************************************************************************
Summary:
Smart card coupler enum.

Description:
The supported couplers

See Also:

****************************************************************************/
typedef enum phx_rfid_system {
	COUPLER_NXP8024,
	COUPLER_NXP8026
} BSCD_COUPLER_TYPE;


/***************************************************************************
Summary:
Smart Card (SCD) module context handle.

Description:
Opaque handle that is created in BSCD_Open.
BSCD_Handle holds the context of the smart card module.  The system
should have only one BSCD_Handle. Caller of BSCD_Open is responsible  to store
this BSCD_Handle and uses it for the future function call after BSCD_Open function
returns successfully.

See Also:
BSCD_Open, BSCD_Channel_Open

****************************************************************************/
typedef struct BSCD_P_Handle
{
	unsigned long		ulMagicNumber; /* Must be  BSCD_P_HANDLE_MAGIC_NUMBER */

	BSCD_ChannelHandle	channelHandles[BSCD_MAX_SUPPOTED_CHANNELS];

	BSCD_Settings		currentSettings;   /* current settings */

	BSCD_COUPLER_TYPE   couplerType;
} *BSCD_Handle;


typedef struct BSCD_CardType_Settings BSCD_CardType_Settings;

/**
  Smart Card Interrupt Callback function.
**/
typedef void (*BSCD_IsrCallbackFunc)( BSCD_ChannelHandle in_channelHandle, void * inp_data );

/***************************************************************************
Summary:
Required default settings structure for smart card channel.

Description:
The default setting structure defines the default configure of
smart card interface/channel when the interface is open.  Since SCD
could support multiple smart card interfaces/channels, system may have
more than one default channel settings that each channel may have
different default channel settings.

See Also:
BSCD_GetChannelDefaultSettings, BSCD_Channel_Open

****************************************************************************/
typedef struct BSCD_ChannelSettings   BSCD_ChannelSettings;


/***************************************************************************
Summary:
Smart Card Interrupt Callback function.

Description:
Caller of BSCD shall call BSCD_Channel_EnableIntrCallback_isr
to register and enable the callback.  BSCD shall call

See Also:
BSCD_Channel_EnableIntrCallback_isr
****************************************************************************/
typedef void (* BICM_CallbackFunc)( void * inp_parm1,  int in_parm2 );


/* Basic Module Functions */

/*****************************************************************************
Summary:
This function shall return a recommended default settings for SCD module.

Description:
This function shall return a recommended default settings for SCD module.
This function shall be called before BSCD_Open
and the caller can then over-ride any of the default settings
required for the build and configuration by calling BSCD_Open.

These default settings are always the same regardless of how
many times this function is called or what other functions have
been called in the porting interface.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_GET_DEFAULT_SETTINGS or during insmod)

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:

Output:
outp_sSettings - BSCD_Settings, a ref/pointer to the default setting.

Returns:
BERR_SUCCESS - Always return success.

See Also:
BSCD_Open

*****************************************************************************/
BERR_Code BSCD_GetDefaultSettings(
		BSCD_Settings	*outp_sSettings
);


/*****************************************************************************
Summary:
This function creates the smart card module handle.

Description:
This function shall create the smart card module handle.
It also initializes the smart card module
and hardware using settings stored in the p_Settings pointer.
All the associated channels are not ready to be access until
BSCD_ChannelOpen is called and returns successfully.

The caller can pass a NULL pointer for inp_sSettings. If the
p_Settings pointer is NULL, default settings should be used.

It is the caller responsibility to store the outp_handle and uses
it for the future function call after this function returns
successfully. If this function returns successfully,  outp_handle shall
not be NULL.

Before calling this function, the only function that the caller
can call is BSCD_GetDefaultSettings. System shall not call
any other smart card functions prior to this function.

System shall not call this function more than once without calling BSCD_Close
previously.

If illegal settings are passed in an error should be
returned and the hardware state should not be modified.

The BINT_Handle is only required if this module needs to
associate ISR callback routines with L2 interrupts.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
during insmod )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
inp_sSettings - BSCD_Settings, the settings that apply to smart card module
					If NULL, a default setting shall be used.
BSCD_COUPLER_TYPE    couplerType

Output:
outp_handle - BSCD_Handle, a ref/pointer to the smart card module handle.
			It shall not be NULL if this function returns successfully.

Returns:
BERR_SUCCESS - success
BERR_OUT_OF_SYSTEM_MEMORY - out of system memory

See Also:
BSCD_GetDefaultSettings
BSCD_Close

*****************************************************************************/
BERR_Code BSCD_Open(
		BSCD_Handle		*outp_handle,
		const BSCD_Settings	*inp_sSettings,
		BSCD_COUPLER_TYPE    couplerType
);


/*****************************************************************************
Summary:
This function frees the main handle and any resources contained
in the main handle.

Description:
This function shall free the main handle and any resources contained
in the main handle. This function shall try to free any resources associated
with sub handles created from the main handle. However, this function does not
free any resources associated with channel handle.

Regardless of the return value, this function always attempts to free all
the allocated resources and inout_handle shall be NULL.

Other than BSCD_GetDefaultSettings, system shall not call any other smart
card functions after this function returns, regardless of the return result.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
during rmmod)

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input/Output:
inout_handle  - BSCD_Handle,  smart card module handle.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.

See Also:
BSCD_Open
BSCD_GetDefaultSettings

******************************************************************************/
BERR_Code BSCD_Close(
		BSCD_Handle inout_handle
);


/*****************************************************************************
Summary:
This function shall return a recommended default settings for SCD channel.

Description:
This function shall return a recommended default settings for SCD channel.
This function shall be called before BSCD_Channel_Open
and the caller can then over-ride any of the default settings
required for the build and configuration by calling BSCD_Channel_Open
or BSCD_Channel_ResetIFD.

The caller shall pass in_channelNo that is smaller outp_totalChannels in
BSCD_GetTotalChannels. The in_channelNo for the first channel shall be zero.

These default settings are always the same regardless of how
many times this function is called or what other functions have
been called in the porting interface.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_GET_CHANNEL_DEFAULT_SETTINGS
or device open )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_handle - BSCD_Handle,  smart card module handle.
in_channelNo  - unsigned int, an index that indicates which channel or smart
					card interface that the caller want to access.
Output:
outp_sSettings - BSCD_ChannelSettings, a ref/pointer to the default channel
					setting.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.
BERR_INVALID_PARAMETER - in_channelNo is invalid.

See Also:
BSCD_Open
BSCD_GetTotalChannels
BSCD_Channel_Open

*****************************************************************************/
BERR_Code BSCD_GetChannelDefaultSettings(
		BSCD_Handle		in_handle,
		unsigned int		in_channelNo,
		BSCD_ChannelSettings	*outp_sSettings
);


/*****************************************************************************
Summary:
This function creates the smart card channel handle.

Description:
This function shall create the smart card channel handle.
It also initializes the specified smart card interface, all the associated
channels and hardware using settings stored in the inp_channelDefSettings
pointer.

The caller shall pass in_channelNo that is smaller outp_totalChannels in
BSCD_GetTotalChannels. The in_channelNo for the first channel shall be zero.

The caller can pass a NULL pointer for inp_channelDefSettings. If the
inp_channelDefSettings pointer is NULL, default settings should be used.

It is the caller responsibility to store the outp_channelHandle and uses
it for the future function call after this function returns
successfully. If this function returns successfully,  outp_handle shall
not be NULL.

Before calling this function, the only channel related functions that
the system can call are BSCD_GetTotalChannels and
BSCD_GetChannelDefaultSettings. System shall not call any other channel
related functions prior to this function.

System shall not call this function more than once without calling
BSCD_Channel_Close previously.

If illegal settings are passed in an error should be
returned and the hardware state should not be modified.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
device open)

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_handle  - BSCD_Handle, smart card module handle.
in_channelNo  - unsigned int, an index that indicates which channel or smart
					card interface that the caller want to access.
inp_channelDefSettings - BSCD_ChannelSettings, the channel settings that
					apply to this  specific channel.  If NULL, a default
					channel setting shall be used.

Output:
outp_channelHandle - BSCD_ChannelHandle, a ref/pointer to the smart
							card channel handle.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.
BERR_INVALID_PARAMETER - in_channelNo or part of inp_channelDefSettings
							is invalid.

See Also:
BSCD_Open
BSCD_GetTotalChannels
BSCD_Channel_Close

*****************************************************************************/
BERR_Code BSCD_Channel_Open(
		BSCD_Handle			in_handle,
		BSCD_ChannelHandle		*outp_channelHandle,
		unsigned int			in_channelNo,
		const BSCD_ChannelSettings	*inp_channelDefSettings,
		unsigned int            in_regBase
);

/*****************************************************************************
Summary:
This function frees the channel handle and any resources contained
in the channel handle.

Description:
This function shall free the channel handle and any resources contained
in the channel handle.

Regardless of the return value, this function always attempts to free all
the allocated resources and inout_channelHandle shall be NULL.

Other than BSCD_GetTotalChannels and BSCD_GetChannelDefaultSettings, system
shall not call any other channel related functions after this function returns,
regardless of the return result.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
device close)

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.

See Also:
BSCD_Channel_Open

******************************************************************************/
BERR_Code BSCD_Channel_Close(
		BSCD_Handle			in_handle,
		unsigned int		in_channelNo
);


/* End of Basic Module Functions */


/* Module Specific Functions */


/*****************************************************************************
Summary:
This function returns a specific smart card channel handle.

Description:
This function returns a specific smart card channel handle.
The caller shall pass in_channelNo that is smaller outp_totalChannels in
BSCD_GetTotalChannels. The in_channelNo for the first channel shall be zero.
If this function returns successfully, outp_channelHandle shall not be NULL.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_GET_CHANNEL )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_handle  - BSCD_Handle, smart card module handle.
in_channelNo -  unsigned int, an index that indicate which channel or smart card
			inerface that the caller want to access.

Output:
outp_channelHandle - BSCD_ChannelHandle, a ref/pointer to the smart card
					 channel handle.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.
BERR_INVALID_PARAMETER - in_channelNo is invalid.

See Also:
BSCD_Open
BSCD_GetTotalChannels
BSCD_Channel_Open

******************************************************************************/
BERR_Code BSCD_GetChannel(
		BSCD_Handle		in_handle,
		unsigned int		in_channelNo,
		BSCD_ChannelHandle	*outp_channelHandle
);


/*****************************************************************************
Summary:
This function shall wait until the card is inserted or removed.

Description:
This function shall wait until the card is inserted or removed.

This function shall returns immediately either
1) the card is removed and in_eCardPresent is BSCD_CardPresent_eRemoved, or
2) the card is inserted and in_eCardPresent is BSCD_CardPresent_eInserted.

This function shall be blocked until the card is inserted if the card is
currently removed and in_eCardPresent is BSCD_CardPresent_eInserted.

This function shall be blocked until the card is removed if the card is
currently inserted and in_eCardPresent is BSCD_CardPresent_eRemoved.

If the caller does not want to be blocked, it could use
BSCD_Channel_GetStatus to check the card presence status.

Note:
The application may seem to be hang since this function may be blocked until
user inserting or removing the card.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_DETECT_CARD )

Performance and Synchronization:
This is a synchronous function that will return when it is done.
This function may be
blocked until user inserts or removes the card. Since TDA8004 does not have
debounce feature for the presence switches, this function may
wait for extra 10ms for the presence switches to be stabilized.

Input:
in_channelHandle  - BSCD_ChannelHandle, a specific smart
					card channel handle.
in_eCardPresent -  BSCD_CardPresent, indicate if the caller wants to wait
				   until the card is inserted or removed.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.

See Also:
BSCD_Channel_GetStatus

******************************************************************************/
BERR_Code BSCD_Channel_DetectCard(
		BSCD_ChannelHandle	in_channelHandle,
		BSCD_CardPresent	in_eCardPresent
);



/*****************************************************************************
Summary:
This function shall modify the current smart card channel setting.

Description:
This function shall modify the current smart card channel setting. For
better performance that this function can modify only a small set of changes,
it is strongly recommended that the caller should call
BSCD_Channel_GetParameters to retrieve the current channel setting.
The caller should only update the modified members in BSCD_Settings and call
BSCD_Channel_SetParameters to set the current setting.


Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_SET_PARAMETERS )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a specific smart
					card channel handle.
inp_sSettings - BSCD_Settings, the settings that apply to a specific
				channel.  If NULL, the function shall return an error.

Returns:
BERR_SUCCESS - success
bxerr_InvalidArgument - inp_sSettings is NULL.
BSCD_STATUS_FAILED - failed.

See Also:
BSCD_Channel_GetParameters

******************************************************************************/
BERR_Code BSCD_Channel_SetParameters(
		BSCD_ChannelHandle		in_channelHandle,
		const BSCD_ChannelSettings	*inp_sSettings
);

/*****************************************************************************
Summary:
This function retrieves the current smart card channel setting.

Description:
This function shall retrieve the current smart card channel setting. If
necessary, the caller can call BSCD_Channel_SetParameters to modify the
current channel setting.


Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_GET_PARAMETERS )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to a specific smart
					card channel handle.

Output:
outp_sSettings - BSCD_Settings, the settings that apply to a specific
				 channel.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.

See Also:
BSCD_Channel_SetParameters

******************************************************************************/
BERR_Code BSCD_Channel_GetParameters(
		BSCD_ChannelHandle	in_channelHandle,
		BSCD_ChannelSettings	*outp_sSettings
);

/*****************************************************************************
Summary:
This function retrieves the negotiate smart card channel setting pointer.

Returns:
BERR_SUCCESS - success
BSCD_STATUS_FAILED - failed.

See Also:
BSCD_Channel_SetParameters

******************************************************************************/
BERR_Code BSCD_Channel_GetNegotiateParametersPointer(
		BSCD_ChannelHandle	in_channelHandle,
		BSCD_ChannelSettings	**outp_sSettings
);

/*****************************************************************************
Summary:
This function retrieves the current smart card channel number.

Description:
This function shall retrieve the current smart card channel number.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_GET_CHANNEL_NUMBER )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to a specific smart
					card channel handle.

Output:
outp_ucChannelNumber - unsigned char, the settings that apply to a specific
				.

Returns:
Channel number, where 0 is the first channel. If it fails, the return values is -1.

See Also:

******************************************************************************/
int	BSCD_Channel_GetChannelNumber(
		BSCD_ChannelHandle	in_channelHandle
);

/*****************************************************************************
Summary:
This function deativates the specific smart card interface.

Description:
This function shall deativate the specific smart card interface.  The
deactivation sequence shall be:

o Set SC_VCC high
o Set SC_RST low
o Disable SC_CLK
o IO is unconditionally driver low.

The caller shall call BSCD_Channel_ResetIFD to  reset a specific smart card
channel.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_DEACTIVATE )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a specific smart
					card channel handle.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

See Also:
BSCD_Channel_PowerICC
BSCD_Channel_ResetIFD

******************************************************************************/
BERR_Code BSCD_Channel_Deactivate(
		BSCD_ChannelHandle          in_channelHandle
);


/*****************************************************************************
Summary:
This function reset a specific smart card interface (IFD).

Description:
This function shall reset a specific smart card interface (IFD).  It shall
not reset the smart card (ICC). This function shall reset the channel and
apply the settings stored in the p_Settings pointer to the channel and
hardware.

This function shall reset the smart card interface in the following sequence:
	Set SC_VCC high  (Only Cold Reset)
	Enable SC_CLK
	Set SC_RST high
	Reset UART transmit and receive buffer.

No ATR data shall be received after this call since IO is yet to be
enabled.  BSCD_Channel_PowerICC with ResetICC option shall activate
the card so that the interface is ready to receive ATR.

For TDA8004 emergency deactivation, we could use this function or
BSCD_Channel_PowerICC with BSCD_ICCAction_ePowerUp
to set SC_VCC high after we realize the smart card is no longer present.
This will set the SC_VCC high and the next BSCD_Channel_GetStatus will show the
correct presence of the card or next BSCD_Channel_DetectCard will response
correctly.  The presence of card is unknown if this function is not called
after TDA8004 emergency deactivation.

The caller shall call BSCD_Channel_Deactivate to  deactivate a specific smart
card channel. The caller shall call BSCD_Reset to reset all the channels.


Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_RESET_CHANNEL )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:
BSCD_Channel_PowerICC
BSCD_Channel_Deactivate

******************************************************************************/
BERR_Code BSCD_Channel_ResetIFD(
		BSCD_ChannelHandle	in_channelHandle,
		BSCD_ResetType		in_resetType

);


/*****************************************************************************
Summary:
This function shall set SC_VCC high, low or reset the smart card.

Description:
There are 3 options to be selected in this function.

If in_iccAction is BSCD_ICCAction_ePowerUp, this function shall set SC_VCC
high.  System should call this function so that the next BSCD_Channel_GetStatus
will show the correct presence of the card or next BSCD_Channel_DetectCard
will response correctly after TDA8004 emergency deactivation.

If in_iccAction is BSCD_ICCAction_ePowerDown, this function shall set SC_VCC
low.  The next BSCD_Channel_GetStatus may not show the correct presence of the
card or next BSCD_Channel_DetectCard may not response correctly after
TDA8004 emergency deactivation.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_Power_ICC )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle,  smart card
					channel handle.
in_iccAction - BSCD_ICCAction, the settings that apply to a specific
				channel.  If NULL, the interface shall be reset with the
				current setting.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:
BSCD_Channel_PowerICC
BSCD_Channel_Deactivate
BSCD_Channel_Receive

******************************************************************************/
BERR_Code BSCD_Channel_PowerICC(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_PowerICC               in_iccAction
);

/*****************************************************************************
Summary:
This function shall set SC_RST high or low
******************************************************************************/
BERR_Code BSCD_Channel_ResetSignal(
		BSCD_ChannelHandle          in_channelHandle,
		bool                        in_bReset
);

/*****************************************************************************
Summary:
This function set voltage level for smart card interface.

Description:
We have to modify the board to use VPP pin of smartcard and connect it to pin 3 (3V/5V) of TDA chip
and run this function.  Make sure to disconnect your QAM or QPSK connection before calling this function
or your smartcard will be damaged.

We also have to use proper voltage card for test when we change smart card interface voltage.
( 3v card for 3v Smart Card  interface, 5v card for 5v Smart Card interface ).
By default the Smart Card iterface is set 5V.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle,  smart card channel handle.
in_vccLevel - Smart Card interface voltage level ( 5V and 3V )

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:

******************************************************************************/
BERR_Code BSCD_Channel_SetVccLevel(
		BSCD_ChannelHandle      in_channelHandle,
		BSCD_VccLevel           in_vccLevel
);

/*****************************************************************************
Summary:
This function enable or disable "Insert Card Hardware Reset" feature in smart card.

Description:
When "Insert Card Hardware Reset" is enabled, the hardware will reset when smart card is inserted.
This is triggered by SC_STATUS_1 bit card_pres  = 0 -> 1
Make sure the SC_IF_CMD_1 bit pres_pol = 1, otherwise this trigger can not happen.
After hardware reset, by default this feature is dabled.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle,  smart card channel handle.
in_enableHwRst - true: Enable this feature
				false: disable this feature

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:

******************************************************************************/
BERR_Code BSCD_Channel_InsCardHwReset(
		BSCD_ChannelHandle          in_channelHandle,
		bool 		            in_enableHwRst
);

/*****************************************************************************
Summary:
This function shall reset the smart card.  Depends on the option, this function could
further read and interpret the ATR data and program the registers accordingly.

Description:
There are 2 options to be selected in this function.

If in_iccAction is BSCD_ResetCardAction_eNoAction, this function shall only reset the card.  It
shall not read ATR data.  Caller has to call  BSCD_Channel_Receive to receive the ATR
data, decode it and then call BSCD_Channel_SetParameters to program the registers.

If in_iccAction is BSCD_ResetCardAction_eReceiveAndDecode, this function shall reset the card.  It
then reads and decodes ATR data and programs the registers accordingly. This option is
required to support smart card standard, like EMV that has stringent timing requirements
on ATR error handling. Caller still has to call  BSCD_Channel_Receive to receive the ATR
data.  The caller has the option if it wants to decode it and then call
BSCD_Channel_SetParameters to program the registers.

This function shall reset the smart card (ICC) in the following sequence:

o SC_VCC low
o SC_RST high
o Wait for 42000 clock cycles.
o IO is ready to receive ATR.
o SC_RST low
o ICC must send ATR per standard requirements (For example between 400 and
   40,000 clock cycles).


Unless specify, the rest of this section describes the scenarioes if
in_iccAction is BSCD_ResetCardAction_eReceiveAndDecode:
This function shall be blocked until either

1) All ATR data is received.
2) one of the timer expired.

After ATR data are received correctly, the hardware shall determine if this
card support direct (TS, first character is 0x3B) or inverse convention (TS is
0x3F). The caller shall call BSCD_Channel_Receive to receive the ATR data.


This function shall parse the ATR data and modify the following settings
according to the received ATR data:

o ProtocolType (T=0 or T=1)
o Clock Rate Conversion Factor
o Baud Rate Adjustment Factor
o Extra Guard Time
o Work Waiting time (For T=0 only.  For EMV standard, it applies to T=1 too)
o Block Wait time (For T=1 only)
o Character Wait time (For T=1 only)

This function shall enable certain interrupts for T=0 or T=1 and set the parity
retrial number.


Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_RESET_CARD )

Performance and Synchronization:
This is a synchronous function that will return when it is done. The maximum
number of ATR data is 32 and the default work waiting time is 9600 ETUs,
therefore the application can wait for 5 seconds, if each ETU is 165us,
before all the ATR bytes are received.

Input:
in_channelHandle  - BSCD_ChannelHandle,  smart card
					channel handle.
in_iccAction - BSCD_ResetCardAction, the settings that apply to a specific
				channel.  If NULL, the interface shall be reset with the
				current setting.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:
BSCD_Channel_PowerICC
BSCD_Channel_Deactivate
BSCD_Channel_Receive

******************************************************************************/
BERR_Code BSCD_Channel_ResetCard(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_ResetCardAction        in_iccAction
);


/*****************************************************************************
Summary:
This function retrieves the smart card channel status and current software
state.

Description:
This function shall retrieve the smart card status and current software
state.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_GET_STATUS )

Performance and Synchronization:
This is a synchronous function that will return when it is done. Since
TDA8004 does not have debounce feature for the presence switches, this
function may wait for extra 10ms for the presence switches to be stabilized.

Input:
in_channelHandle  - BSCD_ChannelHandle, smart card
					channel handle.

Output:
outp_status - BSCD_Status, a ref/pointer that indicates the currect status
			  and software state of the smart card module.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:
BSCD_Channel_DetectCard

******************************************************************************/
BERR_Code BSCD_Channel_GetStatus(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_Status                 *outp_status
);


/*****************************************************************************
Summary:
This function transmits data to the smart card.

Description:
This function shall be blocked until it has transmitted in_unNumXmitBytes
number of  bytes in inp_ucXmitData to the card.

This function shall be blocked until either all the bytes has been transmitted.

For NDS standard, this function shall set the SC_FLOW_CMD[flow_en] to 1 and
the hardware transmitter shall waits until the NDS flow control is deasserted
before transmitting the next byte.

This function shall not interpret the transmitting data.

For any parity error, the hardware shall retry the transmission for the
number of times specify in the setting.  If ICC still reports transmission parity
error after that, SC_INTR_STAT_1[retry_intr] will set to 1 and caller can call
BSCD_Channel_GetStatus to check if this is a transmission parity error.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
driver write)

Performance and Synchronization:
This is a synchronous function that will return when it is done. This function
is blocked until it has transmitted in_unNumXmitBytes number of  bytes in
inp_ucXmitData.

Input:
in_channelHandle  - BSCD_ChannelHandle, smart card
					channel handle.
inp_ucXmitData - uint8_t, a ref/pointer to the buffer for transmitting data
in_unNumXmitBytes - unsigned long, a ref/pointer to the number of transmitting
					bytes.

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:
BSCD_Channel_Transmit
BSCD_Channel_Receive
BSCD_Channel_GetStatus

******************************************************************************/
BERR_Code BSCD_Channel_Transmit(
		BSCD_ChannelHandle          in_channelHandle,
		uint8_t                     *inp_ucXmitData,
		unsigned long               in_ulNumXmitBytes
);


/*****************************************************************************
Summary:
This function receives data from the smart card after IFD transmits data to the smart card.

Description:
This function shall receive data in the outp_ucRcvData from the smart card, after IFD
transmits data to the smart card.

This function is NOT recommended to be used to read ATR data.  Use
BSCD_Channel_ReceiveATR to read ATR data since it handles the time out more
accurately. If the system does not care about the accurate timeout, this function can
be used to read the ATR data.

For better performance, the caller is recommended to set in_unMaxReadBytes
equal to the number of expected receiving bytes. If the caller set
in_unMaxReadBytes to a number that is greater than the outp_unNumRcvBytes
(for example, MAX_ATR_SIZE), this function shall return outp_unNumRcvBytes
number of receiving bytes with an error. It is the responsibility of the
application to determine if this operation is succeed or not.

This function shall be blocked until either

1) The number of receiving data is equal to in_unMaxReadBytes.
2) An error occurs.

This function shall return an error either

1) The number of receiving bytes is greater than in_unMaxReadBytes.
2) One of the timer expired before in_unMaxReadBytes of bytes
   has received.

This function shall not interpret receiving data.

For any parity error, the hardware shall retry the receiving for the
number of times specify in the setting. If IFD still reports receiving parity
error after that, SC_INTR_STAT_1[retry_intr] will set to 1 and caller can call
BSCD_Channel_GetStatus to check if this is a receiving parity error.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
driver read )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.
in_unMaxReadBytes - unsigned long, a ref/pointer to the number of maximum receiving
					bytes

Output:
outp_ucRcvData - uint8_t, a ref/pointer to the buffer for receive data
outp_unNumRcvBytes - unsigned long a ref/pointer to the number of receiving bytes

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:
BSCD_Channel_Transmit
BSCD_Channel_ReceiveATR
BSCD_Channel_GetStatus

******************************************************************************/
BERR_Code BSCD_Channel_Receive(
		BSCD_ChannelHandle       in_channelHandle,
		uint8_t                  *outp_ucRcvData,
		unsigned long            *outp_ulNumRcvBytes,
		unsigned long            in_ulMaxReadBytes
);

/*****************************************************************************
Summary:
This function receives Answer To Reset (ATR) data from the smart card.

Description:
This function shall receive ATR data in the outp_ucRcvData from the smart card. ATR data
shall actually be received after calling BSCD_Channel_PowerICC. This function only
retrieve ATR data from the hardware receiving buffer.

This function is recommended to be used to return ATR data since the time out period
is more accurate. Use BSCD_Channel_Receive
to read the data from the smart card after IFD transmits data to the smart card.

For better performance, the caller is recommended to set in_unMaxReadBytes
equal to the number of expected receiving bytes. If the caller set
in_unMaxReadBytes to a number that is greater than the outp_unNumRcvBytes
(for example, MAX_ATR_SIZE), this function shall return outp_unNumRcvBytes
number of receiving bytes with an error. It is the responsibility of the
application to determine if this operation is succeed or not.

This function shall be blocked until either

1) The number of receiving data is equal to in_unMaxReadBytes.
2) An error occurs.

This function shall return an error either

1) The number of receiving bytes is greater than in_unMaxReadBytes.
2) One of the timer expired before in_unMaxReadBytes of bytes
   has received.

This function shall not interpret receiving data.

For any parity error, the hardware shall retry the receiving for the
number of times specify in the setting. If IFD still reports receiving parity
error after that, SC_INTR_STAT_1[retry_intr] will set to 1 and caller can call
BSCD_Channel_GetStatus to check if this is a receiving parity error.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
driver read )

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.
in_unMaxReadBytes - unsigned long, a ref/pointer to the number of maximum receiving
					bytes

Output:
outp_ucRcvData - uint8_t, a ref/pointer to the buffer for receive data
outp_unNumRcvBytes - unsigned long a ref/pointer to the number of receiving bytes

Returns:
BERR_SUCCESS - success
To Do: Need more error code


See Also:
BSCD_Channel_Transmit
BSCD_Channel_Receive
BSCD_Channel_GetStatus

******************************************************************************/
BERR_Code BSCD_Channel_ReceiveATR(
		BSCD_ChannelHandle      in_channelHandle,
		uint8_t              	*outp_ucRcvData,
		unsigned long           *outp_ulNumRcvBytes,
		unsigned long           in_ulMaxReadBytes
);

/*****************************************************************************
Summary:
This function configures the Waiting Timer or General Purpose Timer.

Description:
This function shall configure the Waiting Timer or General Purpose Timer.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_CONFIG_TIMER).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, smart card
					channel handle.
inp_timer - BSCD_Timer, a ref/pointer to the smart card timer structure.
inp_unCount - BSCD_TimerValue, a ref/pointer to the smart card timer value
			  and unit.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_ConfigTimer(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_Timer 	            *inp_timer,
		BSCD_TimerValue             *inp_unCount

);


/*****************************************************************************
Summary:
This function check to see if the Waiting Timer or General Purpose Timer is enabled or not

Input:
in_channelHandle  - BSCD_ChannelHandle, smart card
					channel handle.
eTimerType - BSCD_TimerType

Returns:
bool
******************************************************************************/
bool BSCD_Channel_IsTimerEnabled(
		BSCD_ChannelHandle   in_channelHandle,
		BSCD_TimerType       eTimerType
);


/*****************************************************************************
Summary:
This function enables or disables the Waiting Timer or General Purpose Timer.

Description:
This function shall either

1) Disable Waiting Timer or General Purpose Timer.
2) Enable Waiting Timer in Work Waiting Time Mode or Block Waiting Time Mode.
3) Enable General Purppose Timer in Start Timer Immediate mode or Start Timer
   on Next Start Bit mode.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_ENABLE_DISABLE_TIMER).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.
inp_timer - BSCD_Timer, a ref/pointer to the smart card timer structure.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_EnableDisableTimer_isr(
		BSCD_ChannelHandle          in_channelHandle,
		BSCD_Timer                  *inp_timer
);


/*****************************************************************************
Summary:
This function enables a specific smart card interrupt.

Description:
This function enables a specific smart card interrupt.

There are 2 callback functions that can be registered with a specific interrupt.
One of them must be reserved for default callback function provided by this module.
The other callback function is opt to use for customized callback function.
Therefore this function can only register 2 different callback functions
per specific interrupt.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_ENABLE_INTR_CALLBACK).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.
in_eIntrType - BICM_IntrType, Interrupt type.
in_callbackhandle - BICM_CallbackFunc, callback function.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_EnableIntrCallback_isr(
		BSCD_ChannelHandle in_channelHandle,
        	BSCD_IntrType      in_eIntrType,
		BSCD_IsrCallbackFunc  in_callbackhandle
);


/*****************************************************************************
Summary:
This function disables a specific smart card interrupt.

Description:
This function disables a specific smart card interrupt.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_DISABLE_INTR_CALLBACK).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.
in_eIntrType - BICM_IntrType, Interrupt type.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code  BSCD_Channel_DisableIntrCallback_isr(
		BSCD_ChannelHandle in_channelHandle,
        	BSCD_IntrType      in_eIntrType
);

/*****************************************************************************
Summary:
This function enable set of related smart card interrupts for T=0, T=1 or T=14.

Description:
This function enable set of related smart card interrupts for T=0, T=1 or T=14.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_ENABLE_INTERRUPTS).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_EnableInterrupts(
	BSCD_ChannelHandle	in_channelHandle
);


/*****************************************************************************
Summary:
This function will reset the Block wait time back to whatever current channel settings which was
either set through ATR or BSCD_Channel_SetParameters.

Description:
This function will reset the Block wait time back to whatever current channel settings which was
either set through ATR or BSCD_Channel_SetParameters.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_RESET_BLOCK_WAIT_TIMER).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_ResetBlockWaitTimer(
		BSCD_ChannelHandle          in_channelHandle
);


/*****************************************************************************
Summary:
This function will set the Block wait time extension.

Description:
This function will set the Block wait time extension.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_SET_BLOCK_WAIT_TIME_EXT).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.
in_ulBlockWaitTimeExtInETU - uint32_t, block wait time extention in ETU.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_SetBlockWaitTimeExt(
		BSCD_ChannelHandle          in_channelHandle,
		uint32_t 		    in_ulBlockWaitTimeExtInETU
);



/*****************************************************************************
Summary:
This function will get the Block wait time.

Description:
This function will get the Block wait time.

Calling Context:
The function shall be called from application level (for example in
VxWorks or no-os) or from driver level (for example in Linux,
recommended ioctl: BSCD_IOCTL_GET_BLOCK_WAIT_TIME).

Performance and Synchronization:
This is a synchronous function that will return when it is done.

Input:
in_channelHandle  - BSCD_ChannelHandle, a ref/pointer to the smart card
					channel handle.
pin_ulBlockWaitTimeInETU - uint32_t *, place to store block wait time in ETU.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_GetBlockWaitTime(
		BSCD_ChannelHandle          in_channelHandle,
		uint32_t 		    *pin_ulBlockWaitTimeInETU
);

/*****************************************************************************
Summary:
This function will set the Block wait time through the passed in BWI.

Input:
pChannelSettings  - BSCD_ChannelSettings, a ref/pointer to the smart card settings handle
ucBWIVal - unsigned char, BWI value.

Returns:
BERR_SUCCESS - success
To Do: Need more error code

******************************************************************************/
BERR_Code BSCD_Channel_SetBlockWaitTimeInteger(
		BSCD_ChannelSettings     *pChannelSettings,
		unsigned char             ucBWIVal
);



BERR_Code BSCD_Channel_SetCardTypeCharacter(
                BSCD_ChannelHandle          in_channelHandle,
                unsigned char               *ucHistoricalBytes,
                unsigned int                unNumHistoricalByte
);

/*****************************************************************************
******************************************************************************/
#define T1_MAX_BUF_SIZE         258 /* 3 (prologue) + 255 (max len) = 258 */

/* PCB block identifier */
#define T1_I_BLOCK              0x00
#define T1_R_BLOCK              0x80
#define T1_S_BLOCK              0xC0
#define T1_I_MORE_BLOCKS        0x20

/* Block sequence number */
#define T1_I_SEQ_SHIFT          6
#define T1_R_SEQ_SHIFT          4

/* R-block defines */
#define T1_R_IS_ERROR(pcb)      ((pcb) & 0x0F)
#define T1_R_EDC_ERROR          0x01
#define T1_R_OTHER_ERROR        0x02

/* S-block defines */
#define T1_S_RESPONSE           0x20
#define T1_S_RESYNC             0x00
#define T1_S_IFS                0x01
#define T1_S_ABORT              0x02
#define T1_S_WTX                0x03

enum {TX, RX, RESYNC};

#define BSCD_IS_APDU_RESPONSE_STATUS_FAIL(_status) \
        (((_status) == 0x90) ? 0:1)

#define BSCD_APDU_PIN_VERIFY        0x0001
#define BSCD_APDU_GET_CHALLENGE     0x0002
#define BSCD_APDU_GET_RESPONSE      0x0004
#define BSCD_APDU_SELECT_FILE       0x0008
#define BSCD_APDU_READ_DATA         0x0010
#define BSCD_APDU_WRITE_DATA        0x0020
#define BSCD_APDU_DELETE_FILE       0x0040

typedef struct apdu_s {
    uint8_t   cla;          /* Instruction class */
    uint8_t   ins;          /* Instruction code */
    uint8_t   p1;           /* Parameter 1 */
    uint8_t   p2;           /* Parameter 2 */
    uint16_t  lc;           /* Length of command data */
    uint16_t  le;           /* Length of response data */
    uint8_t   *data;        /* Response data buffer */
    uint8_t   sw[2];        /* Execution status of the command */
} apdu_t;


BERR_Code BSCD_Channel_APDU_Transceive(
                BSCD_ChannelHandle         in_channelHandle,
                uint8_t                    *inp_ucXmitData,
                unsigned long              in_ulNumXmitBytes,
                uint8_t                    *outp_ucRcvData,
                unsigned long              *outp_ulNumRcvBytes,
                unsigned long              in_ulMaxReadBytes
);


BERR_Code BSCD_Channel_DetectCardNonBlock(
                BSCD_ChannelHandle      in_channelHandle,
                BSCD_CardPresent        in_eCardPresent
);

BERR_Code BSCD_Channel_SetDetectCardCB(
                BSCD_ChannelHandle      in_channelHandle,
                BSCD_CardPresent        in_eCardPresent,
                BSCD_IsrCallbackFunc    in_callback
);


/* perform PPS transaction */
BERR_Code BSCD_Channel_PPS(
		BSCD_ChannelHandle          in_channelHandle
);

BERR_Code  BSCD_Channel_get_atr(
	BSCD_ChannelHandle  in_channelHandle,
	unsigned char   	*outp_ucRcvData,
	unsigned long	*outp_ulNumRcvBytes
);



#ifdef __cplusplus
}
#endif

#endif /* BSCD_H__ */
