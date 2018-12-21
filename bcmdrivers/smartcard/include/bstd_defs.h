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

#ifndef BSTD_DEFS_H__
#define BSTD_DEFS_H__

#ifndef _LINUX_TYPES_H    /* linux has defines for below so don't define it again */

#include <stdarg.h>
#include <stddef.h>

/* added since our compiler seems does not have those */
#define TRUE 1
#define FALSE 0

typedef char int8_t;
typedef unsigned char uint8_t;

typedef short int16_t;
typedef unsigned short uint16_t;

typedef int int32_t;
typedef unsigned int uint32_t;
/* end of add */

typedef long long		int64_t;
typedef unsigned long long uint64_t;

typedef uint32_t		uintptr_t;

#endif /* linux */

/*  Minimun-width integer types  */
typedef int8_t			int_least8_t;
typedef uint8_t			uint_least8_t;

typedef int16_t			int_least16_t;
typedef uint16_t		uint_least16_t;

typedef int32_t			int_least32_t;
typedef uint32_t		uint_least32_t;

/* Greatest-width integer types */

typedef int32_t			intmax_t;
typedef uint32_t		uintmax_t;

/* Boolean */
#ifndef __cplusplus
#define bool 			unsigned int
#endif

#define true 			1
#define false 			0



#define  INT8_MIN (-128)
#define  INT8_MAX ( 127)
#define UINT8_MAX ( 255)

#define  INT16_MIN (-32768)
#define  INT16_MAX ( 32767)
#define UINT16_MAX ( 65535)

#define  INT32_MIN (-2147483648L)
#define  INT32_MAX ( 2147483647L)
#define UINT32_MAX ( 4294967295UL)

/* 7.18.2.2 Limits of minimum-width integer types */

#define  INT_LEAST8_MIN  INT8_MIN
#define  INT_LEAST8_MAX  INT8_MAX
#define UINT_LEAST8_MAX UINT8_MAX

#define  INT_LEAST16_MIN  INT16_MIN
#define  INT_LEAST16_MAX  INT16_MAX
#define UINT_LEAST16_MAX UINT16_MAX

#define  INT_LEAST32_MIN  INT32_MIN
#define  INT_LEAST32_MAX  INT32_MAX
#define UINT_LEAST32_MAX UINT32_MAX

/* 7.18.2.3 Limits of fastest minimum-width integer types */

#define  INT_FAST8_MIN  INT8_MIN
#define  INT_FAST8_MAX  INT8_MAX
#define UINT_FAST8_MAX UINT8_MAX

#define  INT_FAST16_MIN  INT16_MIN
#define  INT_FAST16_MAX  INT16_MAX
#define UINT_FAST16_MAX UINT16_MAX

#define  INT_FAST32_MIN  INT32_MIN
#define  INT_FAST32_MAX  INT32_MAX
#define UINT_FAST32_MAX UINT32_MAX

/* 7.18.2.4 Limits of integer types capable of holding object pointers */

#define  INT_PTR_MIN  INT32_MIN
#define  INT_PTR_MAX  INT32_MAX
#define UINT_PTR_MAX UINT32_MAX

/* 7.18.2.5 Limits of greatest-width integer types */

#define  INTMAX_MIN  INT32_MIN
#define  INTMAX_MAX  INT32_MAX
#define UINTMAX_MAX UINT32_MAX

/* Macros for integer constants */
/* Macros for minimum-width integer constants */
#define INT8_C(value)	((int_least8_t)(value))
#define UINT8_C(value)	((uint_least8_t)(value))

#define INT16_C(value)	((int_least16_t)(value))
#define UINT16_C(value)	((uint_least16_t)(value))

#define INT32_C(value)	((int_least32_t)(value ## L))
#define UINT32_C(value)	((uint_least32_t)(value ## UL))

/* Macros for greatest-width integer constants */

#define INTMAX_C(value)	((intmax_t)(value ## L))
#define UINTMAX_C(value)	((uintmax_t)(value ## UL))

//#define BSTD_UNUSED(x)  { volatile void *bstd_unused; bstd_unused = (void *)&(x); }
#define BSTD_UNUSED(x)


/*****************************************************************************/

#endif /* BRCM_DEFS_H__ */

/* end of file */
