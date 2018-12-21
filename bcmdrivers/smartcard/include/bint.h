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


/*= Module Overview *********************************************************

The InterruptInterface is used to provide an abstraction that can be
used by software modules to provide the following functionality for
various L2 interrupts found in the system:

	1 Assign ISR callback routines to specific interrupts.
		* Multiple callbacks can be installed for a single interrupt.
	2 Enable and disable callbacks.
		* The InterruptInterface will only manage L2 interrupt registers and
	      will not touch bits that are not enabled through the exposed interface.
		  This allows platform specific code to "share" L1 interrupt bits between
		  platform specific code and the interrupt interface.
		* The implementation of the InterruptInterface will automatically
	      mask the L2 hardware interrupt when all callbacks using that specific
		  interrupt bit have been disabled.  Likewise it will un-mask an L2
		  interrupt when one or more callbacks are enabled for a specific interrupt.
	3 Trigger an interrupt (software controlled and if hardware supports it).
	4 Uses interrupt names to specify specific interrupt (calling modules do
      not need to know the layout of each interrupt bit, only the standard
	  name of the interrupt).
		* Strings are used to identify interrupts which allows code to select
	      the proper interrupt when multiple instances of an interrupt exist.
		  This is easily be accomplished through [[KernelInterface][BKNI_Snprintf]]
		  that allows a number value to be inserted into a string.
	5 To support various L2 implementations that may appear on a system
	  (ie, Bcm7028 and Bcm3250 chip in one system), the board/chip-specific code
	  must be provide functions to set/clear interrupts,
	  set/clear interrupt masks,  and process L2 interrupt.

*Interrupt Naming Convention*

The following naming convention should be used when defining interrupt
names:

BXXX_INTERRUPT_NAME

Where BXXX is the PortingInterface module that owns the interrupt.

***************************************************************************/

#ifndef BINT_H
#define BINT_H

#include "bstd.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
This typedef is used to specify the ID an interrupt.
*/
typedef uint32_t BINT_Id;

/*
This structure defines an opaque handle that is used for InterruptInterface function calls.
*/
typedef struct BINT_P_Context *BINT_Handle;

/*
This structure defines an opaque handle that is used for InterruptInterface function calls.
*/
typedef struct BINT_P_Callback *BINT_CallbackHandle;

/*
This defines the function prototype that is used for InterruptInterface callbacks.
*/
typedef void (*BINT_CallbackFunc)( void * pParm1, int parm2 );

/*
Summary:
This function creates a callback.

Description:
After this function is used to create a callback, the BINT_EnableCallback()
function must be used before the callback will actually be enabled.  When
the specified interrupt triggers, the func specified will be called with the
pParm passed into this function.

The first time an callback is created for a specific interrupt, the interrupt
status will be cleared.
*/
BERR_Code BINT_CreateCallback(
							  BINT_CallbackHandle *pCbHandle, /* [out] Returns handle */
							  BINT_Handle handle, /* [in] InterruptInterface handle */
							  BINT_Id intId, /* [in] Id of the interrupt to associate with callback function */
							  BINT_CallbackFunc func, /* [in] Callback function that should be called when the specified interrupt triggers */
							  void * pParm1, /* [in] Parameter that is returned to callback function when interrupt triggers */
							  int parm2 /* [in] Parameter that is returned to callback function when interrupt triggers */
							  );

/*
Summary:
This function destroys a callback and frees any resources associated
with the callback.

Description:
If the specified callback is enabled when this function is called, the
callback may be called at any time until this function returns.
*/
BERR_Code BINT_DestroyCallback(
							   BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
							   );

/*
Summary:
This functions enable a callback.

Description:
The callback may be called any number of times before this function returns!
*/
BERR_Code BINT_EnableCallback(
							  BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
							  );

/*
Summary:
This functions enable a callback.

Description:
The callback may be called any number of times before this function returns!
*/
BERR_Code BINT_EnableCallback_isr(
								  BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
								  );

/*
Summary:
This function disables a callback.

Description:
The callback may be called at any time until the function returns.
*/
BERR_Code BINT_DisableCallback(
							   BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
							   );


/*
Summary:
This function disables a callback.

Description:
The callback may be called at any time until the function returns.
*/
BERR_Code BINT_DisableCallback_isr(
								   BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
								   );

/*
Summary:
This functions clears a pending interrupt specified by the callback.

Description:
The interrupt will only be cleared if there are no callbacks currently
enabled for the interrupt associated with the specified callback.
*/
BERR_Code BINT_ClearCallback(
							  BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
							  );

/*
Summary:
This functions clears a pending interrupt specified by the callback.

Description:
The interrupt will only be cleared if there are no callbacks currently
enabled for the interrupt associated with the specified callback.
*/
BERR_Code BINT_ClearCallback_isr(
							  BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
							  );

/*
Summary:
This function triggers a hardware interrupt.

Description:
The interrupt triggers the hardware specified by the BINT_CallbackHandle.
The callback handle is used directly trigger the interrupt.

Interrupts must be flagged as CPU trigger-able before this function will allow the interrupt
to be triggered (this flag exists in the BINT_P_intDef structure).  This allows specific
platforms and environments to prevent software from triggering interrupts they should not be.
*/
BERR_Code BINT_TriggerInterruptByHandle(
								BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
								);

/*
Summary:
This function triggers a hardware interrupt.

Description:
The interrupt triggers the hardware specified by the BINT_CallbackHandle.
The callback handle is used directly trigger the interrupt.

Interrupts must be flagged as CPU trigger-able before this function will allow the interrupt
to be triggered (this flag exists in the BINT_P_intDef structure).  This allows specific
platforms and environments to prevent software from triggering interrupts they should not be.
*/
BERR_Code BINT_TriggerInterruptByHandle_isr(
									BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
									);

/*
Summary:
Gets the first registered callback.

Description:
Gets the first callback handle stored in the INT module. used with BINT_GetCallbackNext
to traverse through all installed callbacks.
*/
BINT_CallbackHandle BINT_GetCallbackFirst(
										  BINT_Handle intHandle /* [in] Interrupt handle */
										  );

/*
Summary:
Gets the next registered callback.

Description:
Returns the next callback handle after cbHandle. Order of callbacks returned is in
order of callback creation.
*/
BINT_CallbackHandle BINT_GetCallbackNext(
										 BINT_CallbackHandle cbHandle /* [in] Callback handle returned by BINT_CreateCallback() */
										 );
/*
Summary:
Gets the interrupt ID for this callback.

Description:
Returns the interrupt ID for the specific callback in pIntId. Used with the above
traversal functions to identify a callback hooked to a specific interrupt.
*/
BERR_Code BINT_GetInterruptId(
							  BINT_CallbackHandle cbHandle, /* [in] Callback handle returned by BINT_CreateCallback() */
							  BINT_Id *pIntId /* [out] Pointer to storage for the interrupt ID */
							  );

/*
Summary:
Gets callback's active status.

Description:
Gets the status of a callback and returns whether it is active or not in pbEnabled.
*/
BERR_Code BINT_GetCallbackStatus(
								 BINT_CallbackHandle cbHandle,  /* [in] Callback handle returned by BINT_CreateCallback() */
								 bool *pbEnabled /* [out] Pointer to storage for the callback's active status */
								 );

/*
Summary:
This function outputs statistics of the interrupt interface.

Description:
Prints out statistics, using message level debug output and then resets accumulated information.
*/
void BINT_DumpInfo(BINT_Handle intHandle);
#ifdef BDBG_DEBUG_BUILD
/* {secret} */
BERR_Code BINT_P_CreateCallback_Tag(
							  BINT_CallbackHandle *pCbHandle,
							  BINT_Handle handle,
							  BINT_Id intId,
							  BINT_CallbackFunc func,
							  void * pParm1,
							  int parm2,
							  const char *callbackName
							  );
#define BINT_CreateCallback(cbh, h, id, f, p1, p2) BINT_P_CreateCallback_Tag((cbh), (h), (id), (f), (p1), (p2), #f)
#endif

#ifdef __cplusplus
}
#endif

#endif
/* End of File */
