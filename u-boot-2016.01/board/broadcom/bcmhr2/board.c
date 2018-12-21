/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <config.h>
#include <netdev.h>
#include <pci.h>
#include <asm/system.h>
#include <asm/iproc-common/armpll.h>
#include <asm/iproc-common/reg_utils.h>
#include <asm/arch/socregs.h>

DECLARE_GLOBAL_DATA_PTR;

extern void iproc_clk_enum(void);
extern void ddr_init2(void);
extern void bench_screen_test1(void);
extern void iproc_save_shmoo_values(void);
extern uint32_t iproc_config_genpll(uint32_t mode);

static void init_timeouts(void) 
{
#define DDR_IDM_ERROR_LOG_CONTROL_VAL                       0x33a //0x330

    reg32_write((volatile uint32_t *)IHOST_S0_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)IHOST_S1_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);

    reg32_write((volatile uint32_t *)DDR_S1_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)DDR_S2_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);

    reg32_write((volatile uint32_t *)AXI_PCIE_S0_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);

    reg32_write((volatile uint32_t *)CMICD_S0_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)APBY_S0_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)ROM_S0_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)NAND_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)QSPI_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)A9JTAG_S0_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)SRAM_S0_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)APBZ_S0_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)AXIIC_DS_3_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)APBW_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)APBX_IDM_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
    reg32_write((volatile uint32_t *)AXIIC_DS_0_IDM_ERROR_LOG_CONTROL, DDR_IDM_ERROR_LOG_CONTROL_VAL);
}

static void init_axitrace(void)
{
	reg32_write((volatile uint32_t *)IHOST_AXITRACE_M1_ATM_CONFIG, 0x5551);
	reg32_write((volatile uint32_t *)IHOST_AXITRACE_M1_ATM_CMD, 2);
	reg32_write((volatile uint32_t *)IHOST_AXITRACE_M0_ATM_CONFIG, 0x5551);
	reg32_write((volatile uint32_t *)IHOST_AXITRACE_M0_ATM_CMD, 2);
	reg32_write((volatile uint32_t *)IHOST_AXITRACE_ACP_ATM_CONFIG, 0x5551);
	reg32_write((volatile uint32_t *)IHOST_AXITRACE_ACP_ATM_CMD, 2);
}
#if defined(CONFIG_L2C_AS_RAM)
extern ulong mmu_table_addr;
extern void v7_outer_cache_inval_all(void);
extern void v7_outer_cache_unlock_all(void);
#endif
/*****************************************
 * board_init -early hardware init
 *****************************************/
int board_init (void)
{
	uint32_t nor_enable;
	/* Enable flash decode lock */
	nor_enable = reg32_read((volatile uint32_t *)0x1803fc3c);
#ifndef CONFIG_SYS_NO_FLASH
	nor_enable |= 0xC;
	reg32_write((volatile uint32_t *)0x1803fc3c, nor_enable);
#endif
	printf("PNOR enable reg [0x1803fc3c] set to: 0x%x\n", nor_enable);

	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;      /* board id for linux */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR; /* adress of boot parameters */
	iproc_clk_enum();
#if defined(CONFIG_L2C_AS_RAM)
	v7_outer_cache_inval_all();

	printf("Unlocking L2 Cache ...");
	v7_outer_cache_unlock_all();
	printf("Done\n");

	/* MMU on flash fix: 16KB aligned */
	asm volatile ("mcr p15, 0, %0, c2, c0, 0"::"r"(mmu_table_addr)); /*update TTBR0 */
	asm volatile ("mcr p15, 0, r1, c8, c7, 0");  /*invalidate TLB*/
	asm volatile ("mcr p15, 0, r1, c7, c10, 4"); /* DSB */
	asm volatile ("mcr p15, 0, r0, c7, c5, 4"); /* ISB */
#endif
	return 0;
}

/**********************************************
 * dram_init - sets uboots idea of sdram size
 **********************************************/
int dram_init (void)
{
	uint32_t sku_id;

	bench_screen_test1();

	ddr_init2();

	sku_id = (reg32_read((volatile uint32_t *)CMIC_DEV_REV_ID)) & 0x0000ffff;
	if(sku_id == 0xb151  || sku_id == 0x8342 || sku_id == 0x8343 ||
	   sku_id == 0x8344 || sku_id == 0x8346 || sku_id == 0x8347 || 
	   sku_id == 0x8393 || sku_id == 0x8394) {
		/* DeerHound SVK */
		gd->ram_size = DEERHOUND_SVK_SDRAM_SIZE - CONFIG_PHYS_SDRAM_RSVD_SIZE;
	}
	else {
		gd->ram_size = CONFIG_PHYS_SDRAM_1_SIZE - CONFIG_PHYS_SDRAM_RSVD_SIZE;
	}

    return 0;
}

int board_early_init_f (void)
{
	int status = 0;
	uint32_t sku_id, val;

#ifdef CONFIG_HURRICANE2_EMULATION
    /* Set the UART clock to APB clock so we get baudrate 115200. 50000000/16/27 =  115740 */
	val = reg32_read((volatile uint32_t *)ChipcommonA_CoreCtrl);
	val &= 0xFFFFFFF7;
	reg32_write((volatile uint32_t *)ChipcommonA_CoreCtrl, val); /* Disable UART clock */
	val |= 0x9;  
	reg32_write((volatile uint32_t *)ChipcommonA_CoreCtrl, val); /* Enable and set it to APB clock(bit 0) */
#else

	sku_id = (reg32_read((volatile uint32_t *)CMIC_DEV_REV_ID)) & 0x0000ffff;
	if(sku_id == 0x8342 || sku_id == 0x8343 || sku_id == 0x8344 || 
	   sku_id == 0x8346 || sku_id == 0x8347 || sku_id == 0x8393 || 
	   sku_id == 0x8394) {
	    /* Wolfhound/Deerhound */
		/* Configure ARM PLL to 400MHz */
		armpll_config(400);

		status = iproc_config_genpll(4); /* change AXI clock to 200MHz */
	}
	else {
		/* default SKU = b15x, ARM Clock is 1GHz, AXI clock is 400MHz, APB clock is 100MHz */
		/* Configure ARM PLL to 1GHz */
		armpll_config(1000);

		//status = iproc_config_genpll(3); /* 0: 400, 2: 285, 3: 250, 4: 200, 5: 100 */
	}
	/* Set the UART clock to APB clock */
	/* APB clock is axiclk/4 */
	val = reg32_read((volatile uint32_t *)ChipcommonA_CoreCtrl);
	val &= 0xFFFFFFF7;
	reg32_write((volatile uint32_t *)ChipcommonA_CoreCtrl, val); /* Disable UART clock */
	val |= 0x9;  
	reg32_write((volatile uint32_t *)ChipcommonA_CoreCtrl, val); /* Enable and set it to APB clock(bit 0) */
#endif
	return(status);
}

int board_late_init (void) 
{
	int status = 0;

	disable_interrupts();

#ifndef CONFIG_HURRICANE2_EMULATION
	init_timeouts();
	init_axitrace();
#endif

#if defined(CONFIG_RUN_DDR_SHMOO2) && defined(CONFIG_SHMOO_REUSE)
	/* Save DDR PHY parameters into flash if Shmoo was performed */
	iproc_save_shmoo_values();
#endif
	return status;
}

#ifdef CONFIG_BCM_XGS_ETH
extern int bcm_xgs_eth_register(bd_t *bis, u8 dev_num);
int board_eth_init(bd_t *bis)
{
	int rc = -1;
	printf("Registering BCM xgs eth\n");
	rc = bcm_xgs_eth_register(bis, 0);
	return rc;
}
#endif
