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
#ifndef _IPROC_SRA_H
#define _IPROC_SRA_H

#define POLL_TIMEOUT 1000
#define IF_TIMEOUT 10000 
#define IPROC_CHIPCB_SRAB	0x18007000	/* iproc Common B SRAB base */
#define CORE_SIZE    	0x1000		/* each core gets 4Kbytes for registers */


#ifndef PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif	/* PAD */

#define MALLOC(x) kmalloc(x, GFP_KERNEL)
#define MFREE(x) kfree(x)

#ifdef BCMDBG
#define	SRA_ERROR(args)	if (!(sra_msg_level & 1)) ; else printf args
#define	SRA_TRACE(args)	if (!(sra_msg_level & 2)) ; else printf args
#else	/* BCMDBG */
#define	SRA_ERROR(args)
#define	SRA_TRACE(args)
#endif	/* BCMDBG */


#define	REG_MAP(pa, size)	ioremap((unsigned long)(pa), (unsigned long)(size))
#define	REG_UNMAP(va)		iounmap((va))

#define	R_REG(osh, r)	((sizeof *(r) == sizeof (uint32)) ? readl((uint32 *)(r)) : readw((uint16*)(r)))
#define	W_REG(osh, r, v)							\
	do {								\
		if (sizeof *(r) == sizeof (uint32))			\
			writel(v, (uint32*)r);					\
		else							\
			writew((uint16)(v), (uint16*)(r));	\
	} while (0)
#define	AND_REG(osh, r, v)	W_REG(osh, (r), R_REG(osh, r) & (v))
#define	OR_REG(osh, r, v)	W_REG((osh, r), R_REG(osh, r) | (v))


/*  chipcommonb_srab_cmdstat offset0x702c  */
#define CMDSTAT_SRA_GORDYN_SHIFT	0
#define CMDSTAT_SRA_GORDYN_MASK	0x1       
#define CMDSTAT_SRA_WRITE_SHIFT	1
#define CMDSTAT_SRA_WRITE_MASK	0x2       
#define CMDSTAT_SRA_RST_SHIFT	2
#define CMDSTAT_SRA_RST_MASK	0x4       
#define CMDSTAT_RESERVED_SHIFT	3
#define CMDSTAT_RESERVED_MASK	0xfff8    
#define CMDSTAT_SRA_OFFSET_SHIFT	16
#define CMDSTAT_SRA_OFFSET_MASK	0xff0000  
#define CMDSTAT_SRA_PAGE_SHIFT	24
#define CMDSTAT_SRA_PAGE_MASK	0xff000000

/*  chipcommonb_srab_wdh offset0x7030  */
#define WD_H_SR_WDATA_H_SHIFT	0
#define WD_H_SR_WDATA_H_MASK	0xffffffff
        
/*  chipcommonb_srab_wdl offset0x7034  */
#define WD_L_SR_WDATA_L_SHIFT	0
#define WD_L_SR_WDATA_L_MASK	0xffffffff
        
/*  chipcommonb_srab_rdh offset0x7038  */
#define RD_H_SR_RDATA_H_SHIFT	0
#define RD_H_SR_RDATA_H_MASK	0xffffffff
        
/*  chipcommonb_srab_rdl offset0x703c  */
#define RD_L_SR_RDATA_L_SHIFT	0
#define RD_L_SR_RDATA_L_MASK	0xffffffff
          
/*  chipcommonb_srab_sw_if offset0x7040  */
#define CTRL_IF_SPIMUX_ARM_SEL_SHIFT	0
#define CTRL_IF_SPIMUX_ARM_SEL_MASK	0x1       
#define CTRL_IF_HOST_INTR_SHIFT	1
#define CTRL_IF_HOST_INTR_MASK	0x2       
#define CTRL_IF_SPI_BUSY_SHIFT	2
#define CTRL_IF_SPI_BUSY_MASK	0x4       
#define CTRL_IF_RCAREQ_SHIFT	3
#define CTRL_IF_RCAREQ_MASK	0x8       
#define CTRL_IF_RCAGNT_SHIFT	4
#define CTRL_IF_RCAGNT_MASK	0x10      
#define CTRL_IF_SOC_BOOT_DONE_SHIFT	5
#define CTRL_IF_SOC_BOOT_DONE_MASK	0x20      
#define CTRL_IF_SW_INIT_DONE_SHIFT	6
#define CTRL_IF_SW_INIT_DONE_MASK	0x40      
#define CTRL_IF_RESERVED_1_SHIFT	7
#define CTRL_IF_RESERVED_1_MASK	0x80      
#define CTRL_IF_OTP_CTRL_SHIFT	8
#define CTRL_IF_OTP_CTRL_MASK	0xff00    
#define CTRL_IF_RESERVED_SHIFT	16
#define CTRL_IF_RESERVED_MASK	0xffff0000

/*  chipcommonb_srab_sw_intr offset0x7044  */
#define INTR_SWITCH_INTR_CLR_SHIFT	0
#define INTR_SWITCH_INTR_CLR_MASK	0xffffffff
        
typedef volatile struct{
	uint32	PAD[11];
	uint32	cmdstat;	/* 0x2c, command and status register of the SRAB */
	uint32	wd_h;		/* 0x30, high order word of write data to switch register */
	uint32	wd_l;		/* 0x34, low order word of write data to switch register */
	uint32	rd_h;		/* 0x38, high order word of read data from switch register */
	uint32	rd_l;		/* 0x3c, low order word of read data from switch register */
	uint32	ctrl_if;	/* 0x40, switch interface controls */
	uint32	intr;		/* 0x44, 	the register captures interrupt pulses from the switch */
} srabregs_t;

typedef struct _sra_info_s {
    void	*h;			/* dev handle */
    uint16	devid;			/* Device id for the switch */
    uint32	devid32;		/* Device id for the switch (32bits) */
    /* SRAB */
    srabregs_t *srabregs;
    spinlock_t lock;    
} sra_info_t;


#endif
