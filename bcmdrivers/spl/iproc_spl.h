/*
 * Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef _IPROC_SPL_H
#define _IPROC_SPL_H

/*Module info*/
#define DRIVER_NAME           "spl"
#define DRIVER_VERSION        "1.0"
#define DRIVER_DESCRIPTION    "Broadcom SPL(Secure Protection Logic) driver for Cygnus" 

#define BUILD_DATE  __DATE__
#define BUILD_TIME  __TIME__

typedef enum SPL_IRQS{
	SPL_IRQ_WDOG=166,
	SPL_IRQ_RSTMON=167,
	SPL_IRQ_PVTMON=168,
	SPL_IRQ_FMON=169,
}SPL_IRQS_e;


/*Debug functions*/
uint iproc_spl_get_msglevel(void);
#define iproc_prt(format, arg...) \
	do{ \
		if(iproc_spl_get_msglevel()) \
			printk(KERN_INFO "[SPL:%s() %d] "format, __FUNCTION__, __LINE__, ##arg); \
		else \
			printk(KERN_INFO format, ##arg); \
	}while(0)

#define iproc_err(format, arg...) \
	do{ \
		if(iproc_spl_get_msglevel()) \
			printk(KERN_ERR "[SPL:%s() %d] "format, __FUNCTION__, __LINE__, ##arg); \
		else \
			printk(KERN_ERR format, ##arg); \
	}while(0)


#define iproc_dbg(format, arg...) \
	do{																	\
		if(iproc_spl_get_msglevel())										\
		  printk("[SPL:%s() %d] "format, __FUNCTION__, __LINE__, ##arg);	\
	}while(0)


struct iproc_spl_ctx{
	struct platform_device *pdev;		/*platform device pointer*/
	spinlock_t spl_lock;				/*reg access lock*/
	uint32_t __iomem *spl_ctl_addr;		/*mapped ctrl reg base*/
	uint32_t __iomem *spl_base_addr;	/*mapped SPL block reg base*/
	uint32_t *spl_regs_pm_val;			/*Array start addr to save spl regs for PM*/
};

extern struct iproc_spl_ctx *spl_ctx;

uint32_t spl_reg32_read(const uint32_t addr);
void spl_reg32_write(const uint32_t addr, const uint32_t value);

#endif//_IPROC_SPL_H
