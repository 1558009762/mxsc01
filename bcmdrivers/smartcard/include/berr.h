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

#ifndef BERR_H__
#define BERR_H__

/*=Module Overview: ********************************************************
The purpose of this module is to define error codes returned by all
modules, including the porting interface and syslib modules.

It is recommended that all functions should return a BERR_Code.
It is required that APIs must propogate unhandled error codes. Therefore,
if a function calls another function that returns and error code, it must
be able to return an error code.

This module includes support for two different kinds of error codes:
standard error codes and module specific error codes. Both of which can
be used simultaneously within the same module.

Standard error codes are included as part of the BERR module header file.
Module specific error codes are defined in the module specific header
files using BERR_MAKE_CODE.
****************************************************************************/

/***************************************************************************
Summary:
	Standard error code type.

Description:
	This error code may be a module specific error code created with
	BERR_MAKE_CODE or be one of the standard error codes:
	o BERR_SUCCESS
	o BERR_NOT_INITIALIZED
	o BERR_INVALID_PARAMETER
	o BERR_OUT_OF_SYSTEM_MEMORY
	o BERR_OUT_OF_DEVICE_MEMORY
	o BERR_TIMEOUT
	o BERR_OS_ERROR
	o BERR_LEAKED_RESOURCE
	o BERR_NOT_SUPPORTED
	o BERR_UNKNOWN
	o BERR_STATUS_FAILED
****************************************************************************/
typedef uint32_t BERR_Code;

/* standard error codes */

#define BERR_SUCCESS              0  /* success (always zero) */
#define BERR_NOT_INITIALIZED      1  /* parameter not initialized */
#define BERR_INVALID_PARAMETER    2  /* parameter is invalid */
#define BERR_OUT_OF_SYSTEM_MEMORY 3  /* out of KNI module memory */
#define BERR_OUT_OF_DEVICE_MEMORY 4  /* out of MEM module memory */
#define BERR_TIMEOUT              5  /* reached timeout limit */
#define BERR_OS_ERROR             6  /* generic OS error */
#define BERR_LEAKED_RESOURCE      7  /* resource being freed has attached
                                        resources that haven't been freed */
#define BERR_NOT_SUPPORTED 		  8  /* requested feature is not supported */
#define BERR_UNKNOWN              9  /* unknown */
#define BERR_STATUS_FAILED        10  /* operation failed */
#define BERR_STATUS_FAKE_FAILED   11  /* special fake failed, treat as success, added for PPS */

/* {private} error code masks */
#define BERR_P_ID_MASK  UINT32_C(0xFFFF0000)   /* {private} */
#define BERR_P_ID_SHIFT  16                    /* {private} */
#define BERR_P_NUM_MASK  UINT32_C(0x0000FFFF)  /* {private} */

/***************************************************************************
Summary:
	Trace macro.

Description:
	Whenever an error code is stored (not BERR_SUCCESS), it must be wrapped
	with the debugging macro BERR_TRACE(). This wrapping will occur when an
	error code is initially detected and also occurs when a called function
	returns a BERR_Code.

	When debugging is off, this macro does nothing. When debugging is on,
	this macro allows logging of the initial location of the error with the
	actual defined name (not just the hex number). By wrapping functions
	that return error codes, this macro also allows logging of the actual
	stack directly to the error itself.

Input:
	code - Specific error code or function capable of returning an error
	       code.

Returns:
	The error code specified is returned without modification.
****************************************************************************/
#ifdef BDBG_DEBUG_BUILD
#define BERR_TRACE(code) (BDBG_P_PrintError(__FILE__, __LINE__, #code, code))
#else
#define BERR_TRACE(code) (code)
#endif

/***************************************************************************
Summary:
	Extracts the module ID from a given error code.

Description:
	If BERR_MAKE_CODE was used to create the error code, this macro returns
	the ID used to create the error code.

Input:
	code - Source error code.

Returns:
	ID stored in error code.
****************************************************************************/
#define BERR_GET_ID(code) \
    ((((BERR_Code)(code)) & BERR_P_ID_MASK) >> BERR_P_ID_SHIFT)

/***************************************************************************
Summary:
	Extracts the unique number from a given error code.

Description:
	If BERR_MAKE_CODE was used to create the error code, this macro returns
	the unique number used to create the error code.

Input:
	code - Source error code.

Returns:
	Unique number stored in error code.
****************************************************************************/
#define BERR_GET_NUM(code) \
    (((BERR_Code)(code)) & BERR_P_NUM_MASK)

/***************************************************************************
Summary:
	Module specific error code generator.

Description:
	If the module needs error codes that aren't standard, the module may
	use this mechanism to create module specific error codes. These codes
	are guaranteed not to conflict with another module's specific error
	codes.

	If this mechanism is used, a unique ID must be assigned to the module
	and placed in the berr_ids.h file. The module uses this ID along with
	a unique number to create a unique error code.

Input:
	id - Module specific ID from berr_ids.h
	num - Unique number assigned by the module.

Returns:
	Module specific error code.
****************************************************************************/
#define BERR_MAKE_CODE(id, num) \
    (((((BERR_Code)(id)) << BERR_P_ID_SHIFT) & BERR_P_ID_MASK) | \
     (((BERR_Code)(num)) & BERR_P_NUM_MASK))

#endif /* #ifndef BERR_H__ */

/* end of file */
