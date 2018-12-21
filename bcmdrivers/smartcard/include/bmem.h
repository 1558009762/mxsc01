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

#ifndef BMEM_H__
#define BMEM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bmem_config.h"

/*=Module Overview: ********************************************************
The purpose of this module is to provide functions and controls for the
management of memory heaps. Users are allowed to create heaps, allocate
from those heaps, free memory back to the heap, and convert betweeen
device offsets and software addresses.

In addition, this module contains several controls which may be configured
at compile time. These features allow you to customize the implementation
of the memory manager to your specific needs.

Configuration Controls
----------------------

o BMEM_SAFETY_CONFIG - overall policy the memory manager will follow.
o BMEM_BOOKKEEPING_CONFIG - location of bookkeeping information.
o BMEM_REENTRANT_CONFIG - determine whether the memory manager operates
                          in a reentrant manner.

See Also:
	BMEM_Heap_Create,
	BMEM_SAFETY_CONFIG,
	BMEM_BOOKKEEPING_CONFIG,
	BMEM_REENTRANT_CONFIG
****************************************************************************/

/* main MEM module handle */
typedef struct BMEM_P_Mem *BMEM_ModuleHandle;

/***************************************************************************
Summary:
	A handle representing one heap of the memory manager.

Description:
	Memory described by this heap is considered to be device memory. Memory
	contained within this heap can then be used by additional modules to
	contain frame buffers, surfaces, or any other memory that needs to be
	provided directly to the hardware.

See Also:
	BMEM_Heap_Create,
****************************************************************************/
typedef struct BMEM_P_Heap *BMEM_Heap_Handle;

typedef enum BMEM_SafetyConfig
{
    BMEM_SafetyConfig_eFastest = BMEM_CONFIG_FASTEST,
    BMEM_SafetyConfig_eNormal  = BMEM_CONFIG_NORMAL,
    BMEM_SafetyConfig_eTrack   = BMEM_CONFIG_TRACK,
    BMEM_SafetyConfig_eSafe    = BMEM_CONFIG_SAFE,
    BMEM_SafetyConfig_eSafest  = BMEM_CONFIG_SAFEST
} BMEM_SafetyConfig;


typedef enum BMEM_Bookkeeping
{
    BMEM_BookKeeping_eLocal  = BMEM_BOOKKEEPING_LOCAL,
    BMEM_BookKeeping_eSystem = BMEM_BOOKKEEPING_SYSTEM
} BMEM_BookKeeping;


/***************************************************************************
Description:
	Callback called to flush the cache.
	Its implementation is platform-specific.

See Also:
	BMEM_Heap_InstallMonitor
****************************************************************************/
typedef void (*BMEM_FlushCallback)( const void *pvCachedAddress, size_t size );


/***************************************************************************
Summary:
	Holds interesting information about the heap.

Description:
	This is the public structure used to hold information about a heap. This
	information can be retrieved with the BMEM_Heap_GetInfo function.

See Also:
	BMEM_Heap_GetInfo
****************************************************************************/
typedef struct BMEM_HeapInfo
{
	void         *pvAddress;            /* Start address of the heap. */
	uint32_t      ulOffset;             /* Device offset of the heap. */
	size_t        zSize;                /* Size of the heap in bytes. */
	unsigned int  uiAlignment;          /* Heap alignment (power of 2). */

	size_t        ulLargestFree;        /* Size of the largest free block
	                                       in bytes */
	size_t        ulSmallestFree;       /* Size of the smallest free block
	                                       in bytes */
	unsigned int  ulNumFree;            /* Number of free blocks */
	size_t        ulLargestAllocated;   /* Size of the largest allocated
	                                       block in bytes */
	size_t        ulSmallestAllocated;  /* Size of the smallest allocated
	                                       block in bytes */
	unsigned int  ulNumAllocated;       /* Number of  allocated block */
	unsigned int  ulNumErrors;          /* Number of errors detected in
	                                       the heap since creation */
} BMEM_HeapInfo;


/* default settings for module (currently empty) */
typedef void * BMEM_Settings;


/***************************************************************************
Summary:
	Settings structure for heap creation.

Description:
	This is the public structure used to configure a heap upon
	its creation.

	Alignment is used to specify the minimum alignment of all block
	allocations within the heap, and is specified as a power of 2,
	measured in bytes in device offset space. Thus, 0 is unaligned,
	1 is 2-byte aligned, 2 is 4-byte aliged, 16 is 64K-aligned, 24
	is 16MB-aligned, etc.

See Also:
	BMEM_Heap_Create
****************************************************************************/
typedef struct BMEM_Heap_Settings
{
    unsigned int        uiAlignment;    /* enforced byte alignment on allocated memory (default 0) */

    BMEM_SafetyConfig   eSafetyConfig;  /* safety configuration for this heap (default: compile flag) */
    BMEM_BookKeeping    eBookKeeping;   /* bookkeeping configuration for this heap (default: compile flag) */

    void               *pCachedAddress; /* default: 0 (if flush is NULL then this value is not used) */
    BMEM_FlushCallback  flush;          /* Callback used to flush cache at task time (default: NULL) */
    BMEM_FlushCallback  flush_isr;      /* Callback used to flush cache at isr time (default: NULL) */

} BMEM_Heap_Settings;


/***************************************************************************
Summary:
	Fills in a BMEM_Settings structure with default settings.

Description:

Returns:
	BERR_SUCCESS - Default settings obtained.

See Also:
****************************************************************************/
BERR_Code BMEM_GetDefaultSettings
(
	BMEM_Settings *pDefSettings
);

/***************************************************************************
Summary:
	Opens the MEM module.

Description:

Returns:
	BERR_SUCCESS - MEM module successfully opened.

See Also:
****************************************************************************/
BERR_Code BMEM_Open
(
    BMEM_ModuleHandle   *phMem,
    const BMEM_Settings *pDefSettings
);

/***************************************************************************
Summary:
	Closes the MEM module.

Description:

Returns:
	BERR_SUCCESS - MEM module successfully closed.

See Also:
****************************************************************************/
void BMEM_Close
(
    BMEM_ModuleHandle hMem
);

/***************************************************************************
Summary:
	Fills in a BMEM_Heap_Settings structure with default settings.

Description:

Returns:
	BERR_SUCCESS - Default settings obtained.

See Also:
****************************************************************************/
BERR_Code BMEM_Heap_GetDefaultSettings
(
    BMEM_Heap_Settings *pHeapSettings
);

/***************************************************************************
Summary:
	Creates a memory heap.

Description:
	The arguments given to this function describe where the heap is
	located for software and hardware usage, the size of the heap, and
	additional configuration settings for the heap.

	Configuration settings include alignment restrictions placed on
	allocated blocks, local or system bookeeping, the safety configuration,
	in additional to cache settings.

	In order to create a heap, you must first determine where the
	physical memory has been mapped by the OS. This mapping allows
	you to provide the mapped location of the heap accessible
	by software (pvAddress) along with the starting physical
	address of the memory that is being mapped (ulOffset). This
	address range must correspond to a contiguous device address
	range.

	Heaps provided to other software modules must refer to
	uncached address space unless a module's documentation states
	otherwise.

	For example, if you had 64 megabytes in the system, and wanted
	to map the last 16 megabytes to software address 0xF0000000 to
	0xF1000000, you would create a heap where pvAddress = 0xF0000000,
	ulOffset = 0x03000000 (48 megabyte offset), and
	zSize = 0x01000000 (16 megabyte heap).

	When blocks are allocated from the heap, the mapped software
	address will always be returned. If that block address needs to be
	provided to a hardware register, it can be converted by using
	the function BMEM_ConvertAddressToOffset. To go back to a
	mapped software address, use BMEM_ConvertOffsetToAddress.

	For example, using the previously created heap, If I allocated
	a block from the heap, I would get a returned block address
	ranging from 0xF0000000 to 0xF1000000. If I got a block address
	of 0xF0000000 and then converted it to offset, I would get
	the value 0x03000000. Converting an offset of 0x03000000 to
	an address would return 0xF0000000.

Returns:
	BERR_SUCCESS - Heap was created.

See Also:
	BMEM_Heap_Destroy,
	BMEM_Heap_Validate,
	BMEM_Heap_Report,
	BMEM_Heap_ReportVerbose,
	BMEM_Heap_Alloc,
	BMEM_Heap_AllocAligned,
	BMEM_Heap_Free,
	BMEM_Heap_ConvertOffsetToAddress,
	BMEM_Heap_ConvertAddressToOffset,
	BMEM_Heap_GetLargestAvailableBlockSize,
	BMEM_Heap_GetInfo
****************************************************************************/
BERR_Code BMEM_Heap_Create
(
    BMEM_ModuleHandle   hMem,          /* main handle from BMEM_Open() - NULL is possible for older chipsets (7038/3560 only) */
    void               *pvAddress,     /* Pointer to beginning of memory chunk to manage (uncached) */
    uint32_t            ulOffset,      /* Device offset of initial location */
    size_t              zSize,         /* Size of chunk to manage in bytes */
    BMEM_Heap_Settings *pHeapSettings, /* default settings */
    BMEM_Heap_Handle   *phHeap         /* returned heap */
);

/***************************************************************************
Summary:
	Destroys the provided heap.

Description:
	Heaps are destroyed by calling BMEM_Heap_Destroy. This normally only
	happens when the system is being shut down.

	If the heap is not empty (contains allocations), then
	BERR_LEAKED_RESOURCE is returned and the heap is not destroyed.

See Also:
	BMEM_Heap_Create
****************************************************************************/
void BMEM_Heap_Destroy
(
	BMEM_Heap_Handle Heap   /* Heap to destroy. */
);

/***************************************************************************
Summary:
	Checks heap for errors.

Description:
	When debug features are enabled, such as guard banding, this routine
	checks all allocated blocks to insure that they are still valid.

Returns:
	BERR_SUCCESS - Heap was valid.

See Also:
	BMEM_SAFETY_CONFIG
****************************************************************************/
BERR_Code BMEM_Heap_Validate
(
	BMEM_Heap_Handle Heap  /* Heap to validate. */
);

/*{private}*****************************************************************
Summary:
	Private allocation function.

Description:
	This is the main workhorse allocation function. The allocation
	functions BMEM_Alloc and BMEM_AllocAligned are implemented as macros
	around this function. This function implements features for those
	functions while also allowing the file and line number to be tagged
	to the allocated memory (for debugging purposes).

	Tagging can be enabled or disabled by setting BMEM_SAFETY_CONFIG
	appropriately.

	This function should never be called directly. Its arguments may
	change over time when new methods of allocation are made available.

Returns:
	Pointer to allocated address or NULL on failure.

See Also:
	BMEM_Heap_Alloc,
	BMEM_Heap_AllocAligned,
	BMEM_SAFETY_CONFIG
****************************************************************************/
void *BMEM_P_Heap_TagAllocAligned
(
	BMEM_Heap_Handle  pheap,       /* Heap to allocate from */
	size_t            ulSize,      /* size in bytes of block to allocate */
	unsigned int      ucAlignBits, /* alignment for the block */
	unsigned int      Boundary,    /* boundry restricting allocated value */
	const char*       pchFile,     /* source filename where block is
			   				          allocated from */
	int               iLine        /* line number in file where allocation
							          occurs */
);

/***************************************************************************
Summary:
	Allocates memory from a heap.

Description:
	Returns a software address to an allocated piece of memory. Allocated
	memory will use the Heap's native alignment and contain no Boundary
	restrictions.

	Equivalent to calling BMEM_Heap_AllocAligned(Heap, Size, 0, 0).

Input:
	Heap - Heap to allocate from.
	Size - The number of bytes to allocate.

	Note, if the size is zero, will return NULL pointer.

Returns:
	Pointer to allocated address or NULL on failure.

See Also:
	BMEM_Heap_AllocAligned
****************************************************************************/
#define BMEM_Heap_Alloc(Heap, Size) \
	BMEM_P_Heap_TagAllocAligned(Heap, Size, 0, 0, __FILE__, __LINE__)

/***************************************************************************
Summary:
	Allocates memory from a heap.

Description:
	This function is similar to BMEM_Heap_Alloc except that it allows the
	user additional Alignment and Boundary controls.

	Alignment is used to specify the required alignment and is specified
	as a power of 2, measured in bytes in device offset space. Thus, 0 is
	unaligned, 1 is 2-byte aligned, 2 is 4-byte aliged, 16 is 64K-aligned,
	24 is 16MB-aligned, etc.

	If the Alignment value is less than the Heap alignement (provided to
	BMEM_HeapCreate) then the Heap's alignment will be used.

	Boundary specifies a boundary that an allocation must not cross in
	device offset space. This is measured in the same way as Alignment.
	A value of zero specifies no boundary. This value must not correspond
	to a size smaller than AllocationSize.

Input:
	Heap - Heap to allocate from.
	Size - The number of bytes to allocate.
	Alignment - Required alignment of the returned memory.
	Boundary - Prevents allocations across this boundry.

Returns:
	Pointer to allocated address or NULL on failure.

See Also:
	BMEM_Heap_Alloc
****************************************************************************/
#define BMEM_Heap_AllocAligned(Heap, Size, Alignment, Boundry) \
	BMEM_P_Heap_TagAllocAligned(Heap, Size, Alignment, Boundry, \
		__FILE__, __LINE__)

/***************************************************************************
Summary:
	Free memory allocated from heap.

Description:
	Any memory allocated with a call to BMEM_Heap_Alloc or
	BMEM_Heap_AllocAligned may be returned to the heap by calling this
	function. Attempting to free a pointer which was not allocated with
	those functions may cause errors in future allocations.

	If the supplied address is NULL, it is ignored and the function
	will successfully return.

	After the block has been freed, it is coalesced with any adjacent free
	blocks in order to maximize the size of free regions in the heap.

Returns:
	BERR_SUCCESS - Memory was returned to the heap.

See Also:
	BMEM_Heap_Alloc,
	BMEM_Heap_AllocAligned
****************************************************************************/
BERR_Code BMEM_Heap_Free
(
	BMEM_Heap_Handle  Heap,     /* Heap from which the block was allocated. */
	void             *Address   /* Allocated block address. */
);

/***************************************************************************
Summary:
	Converts device offset to address.

Description:
	This function must only be called for regions allocated through
	BMEM_Heap_Alloc or BMEM_Heap_AllocAligned. If other addresses are
	provided, this function may return BERR_INVALID_PARAMETER.

Returns:
	BERR_SUCCESS - Conversion was successful.
	BERR_INVALID_PARAMETER - Offset was invalid for this heap.

See Also:
	BMEM_Heap_ConvertOffsetToAddress
****************************************************************************/
BERR_Code BMEM_Heap_ConvertOffsetToAddress
(
	BMEM_Heap_Handle   Heap,      /* Heap that contains the memory block */
	uint32_t           ulOffset,  /* Device offset within the heap. */
	void             **ppvAddress /* [out] Returned address. */
);

/***************************************************************************
Summary:
	Converts address to device offset.

Description:
	This function must only be called for regions allocated through
	BMEM_Heap_Alloc or BMEM_Heap_AllocAligned. If other addresses are
	provided, this function may return BERR_INVALID_PARAMETER.

Returns:
	BERR_SUCCESS - Conversion was successful.
	BERR_INVALID_PARAMETER - Address was invalid for this heap.

See Also:
	BMEM_Heap_ConvertOffsetToAddress
****************************************************************************/
BERR_Code BMEM_Heap_ConvertAddressToOffset
(
	BMEM_Heap_Handle  Heap,      /* Heap that contains the memory block. */
	void             *pvAddress, /* Address of the memory block */
	uint32_t         *pulOffset  /* [out] Returned device offset. */
);

/***************************************************************************
Summary:
	Converts address to device offset at isr.

Description:
	This function must only be called for regions allocated through
	BMEM_Heap_Alloc or BMEM_Heap_AllocAligned. If other addresses are
	provided, this function may return BERR_INVALID_PARAMETER.

Returns:
	BERR_SUCCESS - Conversion was successful.
	BERR_INVALID_PARAMETER - Address was invalid for this heap.

See Also:
	BMEM_Heap_ConvertOffsetToAddress
****************************************************************************/
#define BMEM_Heap_ConvertAddressToOffset_isr(Heap, pvAddress, pulOffset) \
	BMEM_Heap_ConvertAddressToOffset(Heap, pvAddress, pulOffset)


/***************************************************************************
Summary:
	Gets the largest size block which can be successfully be allocated.

Description:
	This function assumes that the allocated block will be aligned to the
	heap alignment and have no boundry restrictions.

Returns:
	Maximum size of a block (in bytes) which can be allocated.
****************************************************************************/
size_t BMEM_Heap_GetLargestAvailableBlockSize(
	BMEM_Heap_Handle  pheap           /* Heap to check */
);

/***************************************************************************
Summary:
	Gets information about the heap.

Description:
	This function fills in a structure with a variety of interesting
	information about the heap. This information is mainly useful for
	debugging and statistic gathering. See the BMEM_Heap_HeapInfo structure
	definition for the set of information returned.

Returns:
	Maximum size of a block (in bytes) which can be allocated.
****************************************************************************/
void BMEM_Heap_GetInfo
(
	BMEM_Heap_Handle pheap,   /* Heap to get information from. */
	BMEM_HeapInfo      *phi   /* [out] Returned information. */
);

/***************************************************************************
Description:
This structure is used to describe interface used to monitor memory  allocations and deallocations in a heap. This interface provides
alloc and free hooks which will be called after when block is allocated or freed from the heap.
Single interface could be used to monitor several heaps.

****************************************************************************/
typedef struct BMEM_MonitorInterface
{
	void *cnxt; /*  User specified context */
	void (*alloc)(void *cnxt, uint32_t addr, size_t size, const char *fname); /* callback function called when new block was allocated */
	void (*free)(void *cnxt, uint32_t addr); /* callback function called when block was deallocated */
} BMEM_MonitorInterface;

/***************************************************************************
Summary:
	Install monitor to control memory allocation;

Description:
	This function is used to install application defined monitor to control for the memory allocation.
	This is application responsibility to keep monitor valid until it's removed or memory heap destroyed.

Returns:
	BERR_SUCCESS - Monitor was installed


See Also:
	BMEM_MonitorInterface
	BMEM_Heap_RemoveMonitor
****************************************************************************/
BERR_Code BMEM_Heap_InstallMonitor(BMEM_Heap_Handle heap, BMEM_MonitorInterface *monitor);

/***************************************************************************
Summary:
	Removes memory allocation monitor.

Description:
	This function is used to remove application defined monitor.

Returns:
	N/A

See Also:
	BMEM_Heap_InstallMonitor
****************************************************************************/
void BMEM_Heap_RemoveMonitor(BMEM_Heap_Handle heap, BMEM_MonitorInterface *monitor);

/***************************************************************************
Summary:
	Converts software address to cached memory address.

Description:
	This function is used to take pointer from BMEM_Heap_Alloc or BMEM_Heap_AllocAligned
	and turn into cached address. If other addresses are used, it may return
	BERR_INVALID_PARAMETER.

Returns:
	BERR_SUCCESS - Conversion was successful.

See Also:
	BMEM_Heap_FlushCache
	BMEM_Heap_FlushCache_isr
****************************************************************************/
/*  */
BERR_Code BMEM_Heap_ConvertAddressToCached
(
   BMEM_Heap_Handle  Heap,             /* Heap that contains the memory block. */
   void            *pvAddress,        /* Address of the memory block */
   void           **ppvCachedAddress  /* [out] Returned cache address. */
);

/***************************************************************************
Summary:
	Converts software address to cached memory address.

Description:
	The isr version of BMEM_Heap_ConvertAddressToCached.

Returns:
	BERR_SUCCESS - Conversion was successful.

See Also:
	BMEM_Heap_ConvertAddressToCached
****************************************************************************/
#define BMEM_Heap_ConvertAddressToCached_isr(Heap, pvAddress, ppvCachedAddress) \
	BMEM_Heap_ConvertAddressToCached(Heap, pvAddress, ppvCachedAddress)

/***************************************************************************
Summary:
	Flush cached data to the device memory, or read back data from memory to
	cache.

Description:
	This routine will be required if and only if the user is accessing memory
	through a cached address. If you are modifying cached memory, you must
	call BMEM_Heap_FluchCache before the memory can be passed to hardware.
	If you are reading from cached memory that was written by hardware, you
	must also call BMEM_Heap_Flush_Cache on the memory required.

Returns:
	BERR_SUCCESS - flushn was successful.

See Also:
	BMEM_Heap_FlushCache_isr
****************************************************************************/
BERR_Code BMEM_Heap_FlushCache
(
   BMEM_Heap_Handle  Heap,             /* Heap containing the cached memory. */
   void             *pvCachedAddress,  /* Start address to flush */
   size_t            size              /* Size in bytes of the block to flush */
);

/***************************************************************************
Summary:
	The isr version of BMEM_Heap_FlushCache.

Description:
	The difference between these two routines is that the isr routine will
	call the isr callback (cb_isr) and the non-isr routine will call the non-isr
	callback (cb). In both cases, if the function pointer is NULL (hasn't been
	set) this function will return without error. This represents a function
	calling BMEM_Heap_FlushCache without cache parameters enabled.

Returns:
	BERR_SUCCESS - flushn was successful.

See Also:
	BMEM_Heap_FlushCache
****************************************************************************/
BERR_Code BMEM_Heap_FlushCache_isr
(
   BMEM_Heap_Handle  Heap,             /* Heap containing the cached memory. */
   void             *pvCachedAddress,  /* Start address to flush */
   size_t            size              /* Size in bytes of the block to flush */
);


/***************************************************************************
   For backwards compatibility
 ****************************************************************************/

typedef BMEM_Heap_Handle BMEM_Handle;

/***************************************************************************
Summary:
	Creates a memory heap.

Description:
	The arguments given to this function describe where the heap is
	located for software and hardware usage, the size of the heap,
	and any restrictions to alignment placed upon allocated blocks
	returned from the heap.

Returns:
	BERR_SUCCESS - Heap was created.

See Also:
	BMEM_DestroyHeap,
	BMEM_ValidateHeap,
	BMEM_Report,
	BMEM_ReportVerbose,
	BMEM_Alloc,
	BMEM_AllocAligned,
	BMEM_Free,
	BMEM_ConvertOffsetToAddress,
	BMEM_ConvertAddressToOffset,
	BMEM_GetLargestAvailableBlockSize,
	BMEM_GetHeapInfo
****************************************************************************/
BERR_Code BMEM_CreateHeap
(
	BMEM_Handle *ppHeap,      /* Heap to be created. */
	void        *pvAddress,   /* Start address of the heap. */
	uint32_t     ulOffset,    /* Device offset of the heap. */
	size_t       zSize,       /* Size of the heap in bytes. */
	unsigned int uiAlignment  /* Heap alignment (power of 2). */
);

/**********************************************************************func*
 * BMEM_CreateHeapSystem - Initializes the heap with system bookkeeping.
 *
 * This function inititalizes a heap at a given location and size.
 * Any previous allocations in the chunk of memory handed over to this
 * function are lost. Every heap has a base minimum alignment for all of
 * the allocations within that heap. (However, you can specify a greater
 * alignment when actually doing an allocation.)
 *
 * In this implementation, memory in the CPU's system heap is used to store
 * the bookkeeping information for the heap.
 *
 * Returns:
 *   Returns true if heap is initialized, or false if the size of the heap
 *   is too small to manage or there isn't enough system memory to allocate
 *   bookkeeping information.
 *
 */
BERR_Code BMEM_CreateHeapSystem
(
	BMEM_Handle *ppHeap,      /* Heap to be created. */
	void        *pvAddress,   /* Start address of the heap. */
	uint32_t     ulOffset,    /* Device offset of the heap. */
	size_t       zSize,       /* Size of the heap in bytes. */
	unsigned int uiAlignment  /* Heap alignment (power of 2). */
);

/**********************************************************************func*
 * BMEM_CreateHeapLocal - Initializes the heap with local bookkeeping.
 *
 * This function inititalizes a heap at a given location and size.
 * Any previous allocations in the chunk of memory handed over to this
 * function are lost. Every heap has a base minimum alignment for all of
 * the allocations within that heap. (However, you can specify a greater
 * alignment when actually doing an allocation.)
 *
 * In this implementation, the first bit of the heap is used to store heap
 * information such as pointers to the free and used list, as well as other
 * bookkeeping information.
 *
 * Returns:
 *   Returns true if heap is initialized, or false if the given memory
 *   chunk is too small to be a heap.
 *
 */
BERR_Code BMEM_CreateHeapLocal
(
	BMEM_Handle *ppHeap,      /* Heap to be created. */
	void        *pvAddress,   /* Start address of the heap. */
	uint32_t     ulOffset,    /* Device offset of the heap. */
	size_t       zSize,       /* Size of the heap in bytes. */
	unsigned int uiAlignment  /* Heap alignment (power of 2). */
);

/***************************************************************************
Summary:
	Set the cache parameters for a given heap.

Description:
	This function is used to provide cache information to a given heap.
	This function should only be called if the cache is enabled.

Returns:
	BERR_SUCCESS - cache setup was successful.

See Also:
	BMEM_FlushCallback
****************************************************************************/
BERR_Code BMEM_SetCache
(
   BMEM_Handle Heap,          /* Heap handle. */
   void *pvCachedAddress,     /* Cached start address of the heap. */
   BMEM_FlushCallback cb,     /* Callback used to flush cache at task time */
   BMEM_FlushCallback cb_isr  /* Callback used to flush cache at isr time*/
);

#define BMEM_DestroyHeap BMEM_Heap_Destroy
#define BMEM_ValidateHeap BMEM_Heap_Validate
#define BMEM_Alloc BMEM_Heap_Alloc
#define BMEM_AllocAligned BMEM_Heap_AllocAligned
#define BMEM_Free BMEM_Heap_Free
#define BMEM_ConvertOffsetToAddress BMEM_Heap_ConvertOffsetToAddress
#define BMEM_ConvertAddressToOffset BMEM_Heap_ConvertAddressToOffset
#define BMEM_ConvertAddressToOffset_isr BMEM_Heap_ConvertAddressToOffset_isr
#define BMEM_GetLargestAvailableBlockSize BMEM_Heap_GetLargestAvailableBlockSize
#define BMEM_GetHeapInfo BMEM_Heap_GetInfo
#define BMEM_InstallMonitor BMEM_Heap_InstallMonitor
#define BMEM_RemoveMonitor BMEM_Heap_RemoveMonitor
#define BMEM_ConvertAddressToCached BMEM_Heap_ConvertAddressToCached
#define BMEM_ConvertAddressToCached_isr BMEM_Heap_ConvertAddressToCached_isr
#define BMEM_FlushCache BMEM_Heap_FlushCache
#define BMEM_FlushCache_isr BMEM_Heap_FlushCache_isr

#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* #ifndef BMEM_H__ */

/* End of File */
