/*****************************************************************************
* Copyright 2001 - 2009 Broadcom Corporation.  All rights reserved.
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
#ifndef IPROC_SCI_H
#define IPROC_SCI_H

#define DRIVER_NAME        "smartcard"
#define DRIVER_DESCRIPTION "Broadcom Smart Card driver support for Cygnus"
#define DRIVER_VERSION     "0.1"


#undef PDEBUG             /* undef it, just in case */
#undef IPROC_DEBUG
#ifdef IPROC_DEBUG
#  ifdef __KERNEL__
	/* This one if debugging is on, and kernel space */
#	define PDEBUG(fmt, args...) printk( KERN_INFO "IPROC_SCI: " fmt, ## args)
#  else
	/* This one for user space */
#	define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

/* IOCTL number for setting of LCD buffer address */
/* this is used for external allocation, like using VDEC buffer for LCD display */
#define FBIO_SET_BUFFER_ADDR	_IOW('S', 0x55, __u32)    /* 'S' is for smartcard, 0x55 is a number >= 0x20 */


/**********************************************************************
 * SCI driver software context
 * Note this is only for linux related. Inside sci itself it has its own context
 **********************************************************************/
/*
struct sci_regs_bases{
	uint32_t *iomux;
	uint32_t *sci0;
	uint32_t *sci1;
	uint32_t *dmu;
	uint32_t *sciirq;
};
*/
typedef struct IPROC_Resources {
	struct resource *res;
	uint32_t * __iomem vaddr;
}IPROC_Resources_t;


struct iproc_sci_ctx {

	BSCD_Handle  moduleHandle; /* A pointer, contains array of channel handles, allocated and initialized in BSCD_Open() */

	dev_t        sci_devID;   /* allocated device number */
	struct cdev  sci_cdev;

	/* below might be channel specific, if so need to expand into an array */
	uint32_t    *sci_reg_base; /* pointer to current SCI's iomapped register memory region */

	uint32_t     irq;

	/* Used for power management */
	uint32_t	busy;

	struct mutex sc_mutex;
};


#endif //IPROC_SCI_H
