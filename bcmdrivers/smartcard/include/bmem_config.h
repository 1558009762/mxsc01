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

#ifndef BMEM_CONFIG_H__
#define BMEM_CONFIG_H__

#ifndef BMEM_SAFETY_CONFIG
/***************************************************************************
Summary:
	Used to choose the overall policy the memory manager will follow.

Description:
	Selecting one of these policies applies a number of configuration
	parameters. You can see exactly which ones by looking at the policy
	definitions themselves. They can be one of BMEM_CONFIG_FASTEST,
	BMEM_CONFIG_NORMAL, BMEM_CONFIG_TRACK, BMEM_CONFIG_SAFE, or
	BMEM_CONFIG_SAFEST. The default is to use BMEM_CONFIG_SAFE.

See Also:
	BMEM_CONFIG_FASTEST,
	BMEM_CONFIG_NORMAL,
	BMEM_CONFIG_TRACK,
	BMEM_CONFIG_SAFE,
	BMEM_CONFIG_SAFEST
****************************************************************************/
#define BMEM_SAFETY_CONFIG BMEM_CONFIG_SAFE
#endif


#ifndef BMEM_BOOKKEEPING_CONFIG
/***************************************************************************
Summary:
	Used to choose the location of the bookkeeping information.

Description:
	This information can be located either right with the allocated memory
	(typical for a UMA system) or in OS-allocated memory (typical for a
	slave or client system).

	If there is no time penalty for accessing the managed memory with the
	CPU, then it is recommended to use BMEM_BOOKKEEPING_LOCAL. Using OS-
	allocated memory with BMEM_BOOKKEEPING_SYSTEM is significantly slower
	since it uses a less efficient method of correlating bookkeeping to
	addresses.

	The default is to use BMEM_BOOKKEEPING_LOCAL.

See Also:
	BMEM_BOOKKEEPING_LOCAL,
	BMEM_BOOKKEEPING_SYSTEM
****************************************************************************/
#define BMEM_BOOKKEEPING_CONFIG BMEM_BOOKKEEPING_LOCAL
#endif


#ifndef BMEM_REENTRANT_CONFIG
/***************************************************************************
Summary:
	Used to determine whether the memory manager operates in a
	reentrant manner.

Description:
	In most multithreaded situations this must be defined to ensure safe
	operation.  In guaranteed non-reentrant single-threaded operation this
	may be undefined, which will result in a modest speed increase.

	Possible values are BMEM_REENTRANT or BMEM_NOT_REEENTRANT. The default
	is to use BMEM_NOT_REENTRANT.

See Also:
	BMEM_REENTRANT,
	BMEM_NOT_REEENTRANT
****************************************************************************/
#define BMEM_REENTRANT_CONFIG  BMEM_NOT_REENTRANT
#endif


/***************************************************************************
 * There is rarely any need to modify anything below this line.
 ***************************************************************************/

/***************************************************************************
Summary:
	Optimizes the settings for performance, but does no safety checks.

Description:
	The following settings are used:

	o No guard bytes
	o No guard byte checking
	o Don't track file and line number of allocations
	o Don't check if a block is allocated before freeing
	o Don't clear out free blocks
	o Don't check for overlapping blocks before freeing

See Also:
	BMEM_SAFETY_CONFIG
****************************************************************************/
#define BMEM_CONFIG_FASTEST 0


/***************************************************************************
Summary:
	Balances between performance and safety checks.

Description:
	The following settings are used:

	o Short guard byte series
	o Check guard bytes of the block when freed
	o Don't track file and line number of allocations
	o Don't check if a block is allocated before freeing
	o Don't clear out free blocks
	o Don't Check for overlapping blocks before freeing

See Also:
	BMEM_SAFETY_CONFIG
****************************************************************************/
#define BMEM_CONFIG_NORMAL 1


/***************************************************************************
Summary:
	Contains safety checks and tracks allocations.

Description:
	The following settings are used:

	o Short guard byte series
	o Check guard bytes of the block when freed
	o Track file and line number of allocations
	o Don't check if a block is allocated before freeing
	o Don't clear out free blocks
	o Don't Check for overlapping blocks before freeing

See Also:
	BMEM_SAFETY_CONFIG
****************************************************************************/
#define BMEM_CONFIG_TRACK 2


/***************************************************************************
Summary:
	Safe but slow.

Description:
	The following settings are used:

	o Medium guard byte series
	o Check all guards on allocation and free
	o Track file and line number of allocations
	o Check if a block is allocated before freeing
	o Don't clear out free blocks
	o Don't Check for overlapping blocks before freeing

See Also:
	BMEM_SAFETY_CONFIG
****************************************************************************/
#define BMEM_CONFIG_SAFE 3


/***************************************************************************
Summary:
	Safest configuration possible and very slow.

Description:
	The following settings are used:

	o Long guard byte series
	o Check all guards on allocation and free
	o Track file and line number of allocations
	o Check if a block is allocated before freeing.
	o Clear out free blocks
	o Check for overlapping blocks on free.

See Also:
	BMEM_SAFETY_CONFIG
****************************************************************************/
#define BMEM_CONFIG_SAFEST 4


/***************************************************************************
Summary:
	Use the managed memory to store bookkeeping information.

See Also:
	BMEM_BOOKKEEPING_CONFIG
****************************************************************************/
#define BMEM_BOOKKEEPING_LOCAL  0

/***************************************************************************
Summary:
	Use memory allocated by the OS to store bookkeeping information.

See Also:
	BMEM_BOOKKEEPING_CONFIG
****************************************************************************/
#define BMEM_BOOKKEEPING_SYSTEM  1


/***************************************************************************
Summary:
	This module will use semaphores to protect against rentrancy.

See Also:
	BMEM_REENTRANT_CONFIG
****************************************************************************/
#define BMEM_REENTRANT       0


/***************************************************************************
Summary:
	User guaranteed that reentrant calls won't be made into this module.
	Semaphores will not be used.

See Also:
	BMEM_REENTRANT_CONFIG
****************************************************************************/
#define BMEM_NOT_REENTRANT   1


#endif /* #ifndef BMEM_CONFIG_H__ */

/* End of File */
