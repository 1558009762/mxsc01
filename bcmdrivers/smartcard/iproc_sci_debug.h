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


#ifndef IPROC_SCI_DEBUG_H
#define IPROC_SCI_DEBUG_H

/*Debug functions*/
#define IPROC_PRINT_PREFIX "SCI:"
uint iproc_sci_get_msglevel(void);

#ifdef __BASE_FILE_NAME__
#define iproc_get_basename(x)   __BASE_FILE_NAME__  
#else
static char *iproc_get_basename(char *path)
{
	char *p1=path, *p2=p1;

	while (*p1 != '\0')
	{
		if(*p1 == '/')
			p2=p1+1;
		
		p1++;
	}
	return (p2);
}
#endif

#define iproc_prt(format, arg...) \
	do{ \
		int msglevel = iproc_sci_get_msglevel();\
		if(msglevel>0) \
			printk(KERN_INFO "[%s%4d@%12s] "format, IPROC_PRINT_PREFIX, __LINE__, iproc_get_basename(__FILE__), ##arg);	\
		else if(msglevel==0) \
			printk(KERN_INFO format, ##arg); \
	}while(0)

#define iproc_err(format, arg...) \
	do{ \
		int msglevel = iproc_sci_get_msglevel();\
		if(msglevel>0) \
			printk(KERN_ERR "[%s%4d@%12s] ***ERROR*** "format, IPROC_PRINT_PREFIX, __LINE__, iproc_get_basename(__FILE__), ##arg);	\
		else if(msglevel==0) \
			printk(KERN_ERR format, ##arg); \
	}while(0)

#define iproc_dbg(format, arg...) \
	do{ \
		if(iproc_sci_get_msglevel()>0) \
			printk(KERN_DEBUG "[%s%4d@%12s] "format, IPROC_PRINT_PREFIX, __LINE__, iproc_get_basename(__FILE__), ##arg);	\
	}while(0)

#define SHOW_LINE_NUMBER(index) do{iproc_dbg("%s() %d\n", __FUNCTION__, index);}while(0)

///////////////////////////////////////////////////////////////////////////////////////////////
void iproc_sci_regdump(unsigned long reg_base, char *heading);
void iproc_sci_regs_dump(uint32_t reg, uint32_t cnt, char *des);

#endif /* IPROC_SCI_DEBUG_H */
