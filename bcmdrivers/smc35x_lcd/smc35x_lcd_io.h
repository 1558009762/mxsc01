/*****************************************************************************
* Copyright 2001 - 2011 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/
#include <linux/module.h>
#include <linux/io.h>
#include <mach/hardware.h>
#include <linux/slab.h>
#include <mach/socregs-cygnus.h>

/*
 */

#ifndef __IPROC_SMC35x_LCD_IO_H
#define __IPROC_SMC35x_LCD_IO_H

#define __KDB_TRACE

#ifdef __KDB_TRACE
#define _kdb_trace(fmt, args...) printk("IPROC_SMC35X_LCD: " fmt, ## args)
#else
#define _kdb_trace(fmt, args...) do {} while(0)
#endif

/*
 * Utilities to access registers.
 */
static inline void
Wr_Reg32(uint32_t d, void *addr)
{
	*(volatile uint32_t *) addr = d;
}

static inline void
Rd_Reg32(uint32_t * d, void *addr)
{
	*d = *(volatile uint32_t *) addr;
}

/*
 * context
 */
typedef struct smc35x_lcd_ctx {
	uint32_t maxsz;		/* maximal size allocated for the data buffer */
	uint8_t *dbuf;
	void *iomux_base;	/* memory page for IO Mux */
	void *sys_base;		/* SYS base address (CTRL) */
	void *smc_base;		/* SMC controller       */
	void *mem_base;		/* LCD MEM base address */
	void *cmd_addr;
	void *data_addr;
} smc35x_lcd_ctx_t;

/*
 *  Registers to be programmed to enable SMC35x controller for SRAM/LCD application
 */

/*
 * IO mux register.   We need to select the SRAM.
 *
 * Under Broadcom eval board, to provision for large-size SRAM, there are two groups of I/O pins needed
 * to be configured.  One controls via CRMU_IOMUX_CTRL4, anothe via CRMU_IOMUX_CTRL7.
 * 
 * For customer's design, if the SRAM interface is used for LCD display, it is ok to select the first group
 * of I/O via CRMU_IOMUX_CTRL4.   The second group of I/O can be left for other purposes.
 */

#define CRMU_IOMUX_BASE        0x0301d000
#define REG_CRMU_IOMUX_CTRL4   0x0d8	// 0x0301d0d8
#define REG_CRMU_IOMUX_CTRL7   0x0e4

/*
 * The I/O registers control the SMC behavior
 * In this example code, we will use default chip configuration, where
 * Register 0x18000a50 shall contain value 0x0000FEE8
 * Register 0x18000a54 shall contain value 0x0000FEEA
 * Register 0x18000a58 shall contain value 0x0000FCEC 
 * It means:
 * CS0 is asserted for address range 0xE8000000 - 0xE9FFFFFF
 * CS1 is asserted for address range 0xEA000000 - 0xEBFFFFFF
 * CS2 is asserted for address range 0xEC000000 - 0xEFFFFFFF
 *
 *   Set timing parameter such that, SMC uses asynchronous read and write operations
 *      A. tRC is 3 cycles
 *      B. tCEOE is 1 cycle  (OE is asserted 1 cycle after OE is asserted)
 *      C. tWC is 4 cycles
 *      D. tWP is 2 cycles (write period)
 *
 *      For (C, D), we can try (3, 1)-cycle, respectvely.
 */

#define OFFSET(d)            (d&0x00000FFF)

#define SMC35x_SYS_CFG_BASE  (ICFG_PNOR_CONFIG_CS_0 & 0xFFFFF000)	// 0x18000000=(0x18000a50&0xfffff000)

#define SMC35x_CFG_CS0       OFFSET(ICFG_PNOR_CONFIG_CS_0)	// offset 0xa50 to 0x18000000
#define SMC35x_CFG_CS1       OFFSET(ICFG_PNOR_CONFIG_CS_1)	// offset 0xa54
#define SMC35x_CFG_CS2       OFFSET(ICFG_PNOR_CONFIG_CS_2)	// offset 0xa58

#define SMC35x_LCD_ADDR_MASK0   0xFE
#define SMC35x_LCD_ADDR_MATCH0  0xE8
#define SMC35x_LCD_SPACE0       ((SMC35x_LCD_ADDR_MASK0 <<8) | SMC35x_LCD_ADDR_MATCH0)

/* see socregs-cygnus.h for the SMC controller */

#define SMC35x_CTRL_BASE     (PNOR_memc_status & 0xFFFFF000)	// 0x18045000
#define SMC35x_MEMC_STATUS   OFFSET(PNOR_memc_status)	// offset 000
#define SMC35x_MEMIF_CFG     OFFSET(PNOR_memif_cfg)	// offset 004
#define SMC35x_MEM_CFG_SET   OFFSET(PNOR_mem_cfg_set)	// 008
#define SMC35x_MEM_CFG_CLR   OFFSET(PNOR_mem_cfg_clr)	// 00c
#define SMC35x_DIRECT_CMD    OFFSET(PNOR_direct_cmd)	// 010
#define SMC35x_SET_CYCLE     OFFSET(PNOR_set_cycles)	// 014
#define SMC35x_SET_OPMODE    OFFSET(PNOR_set_opmode)	// 018

#define SMC35x_CYCLES0_0     OFFSET(PNOR_sram_cycles0_0)	// 100
#define SMC35x_OPMODE0_0     OFFSET(PNOR_opmode0_0)	// 104

#define tRC    0x03		// 0011  -- read cycle  = 3
#define tWC    0x04		// 0100  -- write cycle = 4
#define tCEOE  0x01		//  001  -- assert OE 1 cycle after CE is asserted
#define tWP    0x02		//  010  -- write enable
#define tPC    0x01		//  001  -- not used
#define tTR    0x01		//  001  -- not used
#define tWE    0x0		//    0  -- not used

#define SMC35x_opmode 0x0	// we only use asynchronous mode

#define SMC35x_UpdateRegs  0x00400000

/*
 * We allocate 64MB space, iomap only 4KB, and importantly, only use
 * two addresss.
 *      Command uses address SMC35x_MEM_BASE
 *      Data/Param uses address SMC35x_MEM_BASE+3
 *      (+3 in order for our testing using SRAM device for 16b data bus width)
 *      For customer, only address 0 and 1 are needed -- 
 *      (bit Addr Bus[0] to distinguish CMD and DATA)
 */
#define SMC35x_MM_BASE       (SMC35x_LCD_ADDR_MATCH0 << 24)	// 0xE8000000  /* where the SRAM/PNOR space starts in BCM5830X */
#define SMC35x_CMD_ADDR      0
#define SMC35x_DATA_ADDR     1

#endif
