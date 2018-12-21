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

#include <linux/interrupt.h>
#include <asm/hardware/gic.h>

#include <linux/irq.h>
#include <mach/iproc.h>
#include <asm/io.h>
#include <mach/io_map.h>

#define IHOST_L2C_INT_MASK_VA HW_IO_PHYS_TO_VIRT(IHOST_L2C_INT_MASK)
#define IHOST_GICDIST_enable_set2_VA HW_IO_PHYS_TO_VIRT(IHOST_GICDIST_enable_set2)

#define IHOST_S1_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(IHOST_S1_IDM_ERROR_LOG_CONTROL)
#define IHOST_S1_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(IHOST_S1_IDM_ERROR_LOG_COMPLETE)
#define IHOST_S1_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(IHOST_S1_IDM_ERROR_LOG_STATUS)
#define IHOST_S1_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(IHOST_S1_IDM_ERROR_LOG_ADDR_LSB)
#define IHOST_S1_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(IHOST_S1_IDM_ERROR_LOG_ID)
#define IHOST_S1_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(IHOST_S1_IDM_ERROR_LOG_FLAGS)

#define IHOST_S0_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(IHOST_S0_IDM_ERROR_LOG_CONTROL)
#define IHOST_S0_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(IHOST_S0_IDM_ERROR_LOG_COMPLETE)
#define IHOST_S0_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(IHOST_S0_IDM_ERROR_LOG_STATUS)
#define IHOST_S0_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(IHOST_S0_IDM_ERROR_LOG_ADDR_LSB)
#define IHOST_S0_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(IHOST_S0_IDM_ERROR_LOG_ID)
#define IHOST_S0_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(IHOST_S0_IDM_ERROR_LOG_FLAGS)

#define DDR_S1_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(DDR_S1_IDM_ERROR_LOG_CONTROL)
#define DDR_S1_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(DDR_S1_IDM_ERROR_LOG_COMPLETE)
#define DDR_S1_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(DDR_S1_IDM_ERROR_LOG_STATUS)
#define DDR_S1_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(DDR_S1_IDM_ERROR_LOG_ADDR_LSB)
#define DDR_S1_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(DDR_S1_IDM_ERROR_LOG_ID)
#define DDR_S1_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(DDR_S1_IDM_ERROR_LOG_FLAGS)

#define DDR_S2_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(DDR_S2_IDM_ERROR_LOG_CONTROL)
#define DDR_S2_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(DDR_S2_IDM_ERROR_LOG_COMPLETE)
#define DDR_S2_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(DDR_S2_IDM_ERROR_LOG_STATUS)
#define DDR_S2_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(DDR_S2_IDM_ERROR_LOG_ADDR_LSB)
#define DDR_S2_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(DDR_S2_IDM_ERROR_LOG_ID)
#define DDR_S2_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(DDR_S2_IDM_ERROR_LOG_FLAGS)

#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_CONTROL)
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_COMPLETE)
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_STATUS)
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ID)
#define AXI_PCIE_S0_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_FLAGS)

#define CMICD_S0_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(CMICD_S0_IDM_IDM_ERROR_LOG_CONTROL)
#define CMICD_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(CMICD_S0_IDM_IDM_ERROR_LOG_COMPLETE)
#define CMICD_S0_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(CMICD_S0_IDM_IDM_ERROR_LOG_STATUS)
#define CMICD_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(CMICD_S0_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define CMICD_S0_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(CMICD_S0_IDM_IDM_ERROR_LOG_ID)
#define CMICD_S0_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(CMICD_S0_IDM_IDM_ERROR_LOG_FLAGS)

#define APBY_S0_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(APBY_S0_IDM_IDM_ERROR_LOG_CONTROL)
#define APBY_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(APBY_S0_IDM_IDM_ERROR_LOG_COMPLETE)
#define APBY_S0_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(APBY_S0_IDM_IDM_ERROR_LOG_STATUS)
#define APBY_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(APBY_S0_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define APBY_S0_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(APBY_S0_IDM_IDM_ERROR_LOG_ID)
#define APBY_S0_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(APBY_S0_IDM_IDM_ERROR_LOG_FLAGS)

#define ROM_S0_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(ROM_S0_IDM_ERROR_LOG_CONTROL)
#define ROM_S0_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(ROM_S0_IDM_ERROR_LOG_COMPLETE)
#define ROM_S0_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(ROM_S0_IDM_ERROR_LOG_STATUS)
#define ROM_S0_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(ROM_S0_IDM_ERROR_LOG_ADDR_LSB)
#define ROM_S0_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(ROM_S0_IDM_ERROR_LOG_ID)
#define ROM_S0_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(ROM_S0_IDM_ERROR_LOG_FLAGS)

#define NAND_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(NAND_IDM_IDM_ERROR_LOG_CONTROL)
#define NAND_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(NAND_IDM_IDM_ERROR_LOG_COMPLETE)
#define NAND_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(NAND_IDM_IDM_ERROR_LOG_STATUS)
#define NAND_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(NAND_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define NAND_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(NAND_IDM_IDM_ERROR_LOG_ID)
#define NAND_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(NAND_IDM_IDM_ERROR_LOG_FLAGS)

#define QSPI_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(QSPI_IDM_IDM_ERROR_LOG_CONTROL)
#define QSPI_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(QSPI_IDM_IDM_ERROR_LOG_COMPLETE)
#define QSPI_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(QSPI_IDM_IDM_ERROR_LOG_STATUS)
#define QSPI_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(QSPI_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define QSPI_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(QSPI_IDM_IDM_ERROR_LOG_ID)
#define QSPI_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(QSPI_IDM_IDM_ERROR_LOG_FLAGS)

#define A9JTAG_S0_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(A9JTAG_S0_IDM_IDM_ERROR_LOG_CONTROL)
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(A9JTAG_S0_IDM_IDM_ERROR_LOG_COMPLETE)
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(A9JTAG_S0_IDM_IDM_ERROR_LOG_STATUS)
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(A9JTAG_S0_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(A9JTAG_S0_IDM_IDM_ERROR_LOG_ID)
#define A9JTAG_S0_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(A9JTAG_S0_IDM_IDM_ERROR_LOG_FLAGS)

#define SRAM_S0_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(SRAM_S0_IDM_ERROR_LOG_CONTROL)
#define SRAM_S0_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(SRAM_S0_IDM_ERROR_LOG_COMPLETE)
#define SRAM_S0_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(SRAM_S0_IDM_ERROR_LOG_STATUS)
#define SRAM_S0_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(SRAM_S0_IDM_ERROR_LOG_ADDR_LSB)
#define SRAM_S0_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(SRAM_S0_IDM_ERROR_LOG_ID)
#define SRAM_S0_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(SRAM_S0_IDM_ERROR_LOG_FLAGS)

#define APBZ_S0_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(APBZ_S0_IDM_IDM_ERROR_LOG_CONTROL)
#define APBZ_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(APBZ_S0_IDM_IDM_ERROR_LOG_COMPLETE)
#define APBZ_S0_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(APBZ_S0_IDM_IDM_ERROR_LOG_STATUS)
#define APBZ_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(APBZ_S0_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define APBZ_S0_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(APBZ_S0_IDM_IDM_ERROR_LOG_ID)
#define APBZ_S0_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(APBZ_S0_IDM_IDM_ERROR_LOG_FLAGS)

#define AXIIC_DS_3_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_3_IDM_ERROR_LOG_CONTROL)
#define AXIIC_DS_3_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_3_IDM_ERROR_LOG_COMPLETE)
#define AXIIC_DS_3_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_3_IDM_ERROR_LOG_STATUS)
#define AXIIC_DS_3_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_3_IDM_ERROR_LOG_ADDR_LSB)
#define AXIIC_DS_3_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_3_IDM_ERROR_LOG_ID)
#define AXIIC_DS_3_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_3_IDM_ERROR_LOG_FLAGS)

#define APBW_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(APBW_IDM_IDM_ERROR_LOG_CONTROL)
#define APBW_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(APBW_IDM_IDM_ERROR_LOG_COMPLETE)
#define APBW_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(APBW_IDM_IDM_ERROR_LOG_STATUS)
#define APBW_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(APBW_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define APBW_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(APBW_IDM_IDM_ERROR_LOG_ID)
#define APBW_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(APBW_IDM_IDM_ERROR_LOG_FLAGS)

#define APBX_IDM_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(APBX_IDM_IDM_ERROR_LOG_CONTROL)
#define APBX_IDM_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(APBX_IDM_IDM_ERROR_LOG_COMPLETE)
#define APBX_IDM_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(APBX_IDM_IDM_ERROR_LOG_STATUS)
#define APBX_IDM_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(APBX_IDM_IDM_ERROR_LOG_ADDR_LSB)
#define APBX_IDM_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(APBX_IDM_IDM_ERROR_LOG_ID)
#define APBX_IDM_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(APBX_IDM_IDM_ERROR_LOG_FLAGS)

#define AXIIC_DS_0_IDM_ERROR_LOG_CONTROL_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_0_IDM_ERROR_LOG_CONTROL)
#define AXIIC_DS_0_IDM_ERROR_LOG_COMPLETE_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_0_IDM_ERROR_LOG_COMPLETE)
#define AXIIC_DS_0_IDM_ERROR_LOG_STATUS_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_0_IDM_ERROR_LOG_STATUS)
#define AXIIC_DS_0_IDM_ERROR_LOG_ADDR_LSB_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_0_IDM_ERROR_LOG_ADDR_LSB)
#define AXIIC_DS_0_IDM_ERROR_LOG_ID_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_0_IDM_ERROR_LOG_ID)
#define AXIIC_DS_0_IDM_ERROR_LOG_FLAGS_VA HW_IO_PHYS_TO_VIRT(AXIIC_DS_0_IDM_ERROR_LOG_FLAGS)

#define IDM_ERROR_LOG_ENABLE 0x33A
#define IDM_ERROR_LOG_CLEAR 0x3

#ifdef CONFIG_MACH_IPROC_P7

#if defined(CONFIG_MACH_HR3)
#define IHOST_S0_IDM_IRQ 52
#define DDR_S1_IDM_IRQ 53
#define DDR_S2_IDM_IRQ 54
#define AXI_PCIE_S0_IDM_IRQ 55
//#define AXI_PCIE_S1_IDM_IRQ 57
#define ROM_S0_IDM_IRQ 56
#define NAND_IDM_IRQ 57
#define QSPI_IDM_IRQ 58
#define PNOR_IDM_IRQ 59
#define SRAM_S0_IDM_IRQ 60
#define A9JTAG_S0_IDM_IRQ 61
#define APX_IDM_IRQ 64
#define CMICD_S0_IDM_IRQ 67
#define AXIIC_DS_0_IDM_IRQ 68
#define AXIIC_DS_1_IDM_IRQ 69
#define AXIIC_DS_2_IDM_IRQ 70
//#define AXIIC_DS_3_IDM_IRQ 81
//#define AXIIC_DS_4_IDM_IRQ 83
#else
#define IHOST_S0_IDM_IRQ 52
#define DDR_S1_IDM_IRQ 54
#define DDR_S2_IDM_IRQ 55
#define AXI_PCIE_S0_IDM_IRQ 56
#define AXI_PCIE_S1_IDM_IRQ 57
#define ROM_S0_IDM_IRQ 58
#define NAND_IDM_IRQ 59
#define QSPI_IDM_IRQ 60
#define SRAM_S0_IDM_IRQ 62
#define A9JTAG_S0_IDM_IRQ 64
#define APX_IDM_IRQ 68
#define CMICD_S0_IDM_IRQ 71
#define AXIIC_DS_0_IDM_IRQ 78
#define AXIIC_DS_1_IDM_IRQ 79
#define AXIIC_DS_2_IDM_IRQ 80
#define AXIIC_DS_3_IDM_IRQ 81
#define AXIIC_DS_4_IDM_IRQ 83
#endif

#else
#define IHOST_S1_IDM_IRQ 62
#define IHOST_S0_IDM_IRQ 63
#define DDR_S1_IDM_IRQ 64
#define DDR_S2_IDM_IRQ 65
#define AXI_PCIE_S0_IDM_IRQ 66
#define AXI_PCIE_S1_IDM_IRQ 67
#define CMICD_S0_IDM_IRQ 68
#define ROM_S0_IDM_IRQ 69
#define NAND_IDM_IRQ 70
#define QSPI_IDM_IRQ 71
#define SATA_IDM_IRQ 72
#define A9JTAG_S0_IDM_IRQ 73
#define SRAM_S0_IDM_IRQ 74
#define APW_IDM_IRQ 75
#define APX_IDM_IRQ 76
#define APBY_S0_IDM_IRQ 77
#define APBZ_S0_IDM_IRQ 78
#define AXIIC_DS_0_IDM_IRQ 79
#define AXIIC_DS_1_IDM_IRQ 80
#define AXIIC_DS_2_IDM_IRQ 81
#define AXIIC_DS_3_IDM_IRQ 82
#define AXIIC_DS_4_IDM_IRQ 83
#endif

static irqreturn_t idm_timeout_handler(int val, void *ptr)
{
	u32 errStat;
//	printk("%s: %d, %d entry\n", __FUNCTION__, __LINE__, val);
	errStat = __raw_readl(IHOST_S1_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(IHOST_S1_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(IHOST_S1_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(IHOST_S1_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, IHOST_S1_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(IHOST_S1_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(IHOST_S0_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(IHOST_S0_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(IHOST_S0_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(IHOST_S0_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, IHOST_S0_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(IHOST_S0_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(DDR_S1_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(DDR_S1_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(DDR_S1_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(DDR_S1_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, DDR_S1_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(DDR_S1_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(DDR_S2_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(DDR_S2_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(DDR_S2_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(DDR_S2_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, DDR_S2_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(DDR_S2_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, AXI_PCIE_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(AXI_PCIE_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(CMICD_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(CMICD_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(CMICD_S0_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(CMICD_S0_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, CMICD_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(CMICD_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
#if !defined(CONFIG_MACH_IPROC_P7)
	errStat = __raw_readl(APBY_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBY_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBY_S0_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBY_S0_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, APBY_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(APBY_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
#endif
	errStat = __raw_readl(ROM_S0_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(ROM_S0_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(ROM_S0_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(ROM_S0_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, ROM_S0_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(ROM_S0_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(NAND_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(NAND_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(NAND_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(NAND_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, NAND_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(NAND_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(QSPI_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(QSPI_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(QSPI_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(QSPI_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, QSPI_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(QSPI_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(A9JTAG_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(A9JTAG_S0_IDM_IDM_ERROR_LOG_ADDR_LSB);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(A9JTAG_S0_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(A9JTAG_S0_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, A9JTAG_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(A9JTAG_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
#if !defined(CONFIG_MACH_IPROC_P7)
	errStat = __raw_readl(SRAM_S0_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(SRAM_S0_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(SRAM_S0_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(SRAM_S0_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, SRAM_S0_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(SRAM_S0_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(APBZ_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBZ_S0_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBZ_S0_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBZ_S0_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, APBZ_S0_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(APBZ_S0_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
#endif

#if !defined(CONFIG_MACH_HR3)
	errStat = __raw_readl(AXIIC_DS_3_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXIIC_DS_3_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXIIC_DS_3_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXIIC_DS_3_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, AXIIC_DS_3_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(AXIIC_DS_3_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
#endif /* !CONFIG_MACH_HR3) */

#if !defined(CONFIG_MACH_IPROC_P7)
	errStat = __raw_readl(APBW_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBW_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBW_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBW_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, APBW_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(APBW_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
#endif
	errStat = __raw_readl(APBX_IDM_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBX_IDM_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBX_IDM_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(APBX_IDM_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, APBX_IDM_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(APBX_IDM_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
	errStat = __raw_readl(AXIIC_DS_0_IDM_ERROR_LOG_STATUS_VA);
	if (errStat > 0)
	{
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXIIC_DS_0_IDM_ERROR_LOG_ADDR_LSB_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXIIC_DS_0_IDM_ERROR_LOG_ID_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		errStat = __raw_readl(AXIIC_DS_0_IDM_ERROR_LOG_FLAGS_VA);
//		printk("%s: %d, %08x\n", __FUNCTION__, __LINE__, errStat);
		__raw_writel(IDM_ERROR_LOG_CLEAR, AXIIC_DS_0_IDM_ERROR_LOG_COMPLETE_VA);
		errStat = __raw_readl(AXIIC_DS_0_IDM_ERROR_LOG_STATUS_VA);
//		printk("%s: %d, %d\n", __FUNCTION__, __LINE__, errStat);
	}
//	printk("%s: %d exit\n", __FUNCTION__, __LINE__);
	return IRQ_HANDLED;
}

int l2cc_interrupt_error_handler(int val, void *ptr)
{
	printk("%s: %d, %d entry\n", __FUNCTION__, __LINE__, val);
	printk("%s: %d exit\n", __FUNCTION__, __LINE__);
	return 0;
}

void request_idm_timeout_interrupts(void)
{
	u32 l2cc_mask;
	int ret = 0;

//	printk("%s: %d entry\n", __FUNCTION__, __LINE__);

	/* clear all pending idm interrupts */
	idm_timeout_handler(0, NULL);
    
	/* enable idm error log for all slaves */

	__raw_writel(IDM_ERROR_LOG_ENABLE, IHOST_S1_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, IHOST_S0_IDM_ERROR_LOG_COMPLETE_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, DDR_S1_IDM_ERROR_LOG_COMPLETE_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, DDR_S2_IDM_ERROR_LOG_COMPLETE_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, AXI_PCIE_S0_IDM_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, CMICD_S0_IDM_IDM_ERROR_LOG_CONTROL_VA);

#if !defined(CONFIG_MACH_HR3)
	__raw_writel(IDM_ERROR_LOG_ENABLE, SRAM_S0_IDM_ERROR_LOG_CONTROL_VA);
#endif /* !CONFIG_MACH_HR3 */

#ifndef CONFIG_MACH_IPROC_P7
	__raw_writel(IDM_ERROR_LOG_ENABLE, APBY_S0_IDM_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, APBZ_S0_IDM_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, APBW_IDM_IDM_ERROR_LOG_CONTROL_VA);
#endif

	__raw_writel(IDM_ERROR_LOG_ENABLE, ROM_S0_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, NAND_IDM_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, QSPI_IDM_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, A9JTAG_S0_IDM_IDM_ERROR_LOG_CONTROL_VA);
#if !defined(CONFIG_MACH_HR3)
	__raw_writel(IDM_ERROR_LOG_ENABLE, AXIIC_DS_3_IDM_ERROR_LOG_COMPLETE_VA);
#endif /* !CONFIG_MACH_HR3 */
	__raw_writel(IDM_ERROR_LOG_ENABLE, APBX_IDM_IDM_ERROR_LOG_CONTROL_VA);
	__raw_writel(IDM_ERROR_LOG_ENABLE, AXIIC_DS_0_IDM_ERROR_LOG_CONTROL_VA);

	/* now enable the idm interrupts */

#ifndef CONFIG_MACH_IPROC_P7
	ret = request_irq(IHOST_S1_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#endif /* !CONFIG_MACH_IPROC_P7 */
	ret = request_irq(IHOST_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(DDR_S1_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(DDR_S2_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(AXI_PCIE_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);

#if !defined(CONFIG_MACH_HR3)
	ret = request_irq(AXI_PCIE_S1_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#endif /* !defined(CONFIG_MACH_HR3) */

	ret = request_irq(CMICD_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(ROM_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(NAND_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(QSPI_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#ifndef CONFIG_MACH_IPROC_P7
	ret = request_irq(SATA_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#endif /* !CONFIG_MACH_IPROC_P7 */
	ret = request_irq(A9JTAG_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(SRAM_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(APX_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#ifndef CONFIG_MACH_IPROC_P7
	ret = request_irq(APW_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(APBY_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(APBZ_S0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#endif /* !CONFIG_MACH_IPROC_P7 */
	ret = request_irq(AXIIC_DS_0_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(AXIIC_DS_1_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
	ret = request_irq(AXIIC_DS_2_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);

#if !defined(CONFIG_MACH_HR3)
	ret = request_irq(AXIIC_DS_3_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#endif /* !defined(CONFIG_MACH_HR3) */

#ifndef CONFIG_MACH_IPROC_P7
	ret = request_irq(AXIIC_DS_4_IDM_IRQ, (irq_handler_t)idm_timeout_handler, IRQF_DISABLED | IRQF_PERCPU, "IDM", NULL);
	if (ret != 0)
		printk("%s: %d request_irq return = %d\n", __FUNCTION__, __LINE__, ret);
#endif /* !CONFIG_MACH_IPROC_P7 */
//	printk("%s: %d exit\n", __FUNCTION__, __LINE__);
}
