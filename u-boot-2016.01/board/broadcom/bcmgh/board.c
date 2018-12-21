/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <config.h>
#include <netdev.h>
#include <asm/system.h>
#include <asm/iproc-common/armpll.h>
#include <asm/iproc-common/reg_utils.h>
#include <asm/arch/socregs.h>

DECLARE_GLOBAL_DATA_PTR;

extern void iproc_clk_enum(void);
extern void ddr_init2(void);
extern void save_shmoo_to_flash(void);
extern uint32_t iproc_config_genpll(uint32_t mode);

static
u32 cmicd_schan_read_top (u32 addr) {
    u32 read = 0x0;

    reg32_write((volatile u32 *)CMIC_COMMON_SCHAN_MESSAGE0, 0x2c800200);
    reg32_write((volatile u32 *)CMIC_COMMON_SCHAN_MESSAGE1, addr);

    reg32_write((volatile u32 *)CMIC_COMMON_SCHAN_CTRL, 0x1);

    while (read != 0x2) {
       read = reg32_read((volatile u32 *)CMIC_COMMON_SCHAN_CTRL);
    }
    read = reg32_read((volatile u32 *)CMIC_COMMON_SCHAN_MESSAGE1);
    return read;
}

static
u32 cmicd_schan_write_top (u32 addr, u32 val) {
    u32 read = 0x0;

    reg32_write((volatile u32 *)CMIC_COMMON_SCHAN_MESSAGE0, 0x34800200);
    reg32_write((volatile u32 *)CMIC_COMMON_SCHAN_MESSAGE1, addr);
    reg32_write((volatile u32 *)CMIC_COMMON_SCHAN_MESSAGE2, val);

    reg32_write((volatile u32 *)CMIC_COMMON_SCHAN_CTRL, 0x1);

    while (read != 0x2) {
       read = reg32_read((volatile u32 *)CMIC_COMMON_SCHAN_CTRL);
    }
    return read;
}

static
void cmicd_init_soc (void) {
    /* Configure SBUS Ring Map for TOP, block id = 16, ring number = 4 */
    reg32_write((volatile u32 *)CMIC_SBUS_RING_MAP_0_7, 0x11112200);
    reg32_write((volatile u32 *)CMIC_SBUS_RING_MAP_8_15, 0x430001);
    reg32_write((volatile u32 *)CMIC_SBUS_RING_MAP_16_23, 0x5064);
    reg32_write((volatile u32 *)CMIC_SBUS_RING_MAP_24_31, 0x0);
}
#if defined(CONFIG_L2C_AS_RAM)
extern ulong mmu_table_addr;
extern void v7_outer_cache_inval_all(void);
extern void v7_outer_cache_unlock_all(void);
#endif
/*
 * board_init - early hardware init
 */
int board_init(void)
{
#if !defined(CONFIG_SYS_NO_FLASH) && \
    !defined(CONFIG_NOR_IPROC_BOOT) && \
    !defined(CONFIG_NAND_IPROC_BOOT)
    
    /* If booting from PNOR, PNOR is already selected, no override required.
     * If booting from NAND, we shouldn't switch to PNOR.
     */
    *(volatile int *)ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL |= cpu_to_le32(
        (1 << ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel) |
        (1 << ICFG_IPROC_IOPAD_SW_OVERRIDE_CTRL__iproc_pnor_sel_sw_ovwr));
        
    /* Configure controller memory width based on mw strap */
    if (*(volatile int *)ICFG_PNOR_STRAPS & 
        cpu_to_le32(1 << ICFG_PNOR_STRAPS__PNOR_SRAM_MW_R)) {
        /* 16-bit */
        *(volatile int *)PNOR_set_opmode |= 
                cpu_to_le32(1 << PNOR_set_opmode__set_mw_R);
    } else {
        /* 8-bit */
        *(volatile int *)PNOR_set_opmode &= 
                cpu_to_le32(~(1 << PNOR_set_opmode__set_mw_R));
    }
    *(volatile int *)PNOR_direct_cmd |= 
            cpu_to_le32(2 << PNOR_direct_cmd__cmd_type_R);
    
#endif /* !CONFIG_SYS_NO_FLASH && !CONFIG_NOR_IPROC_BOOT */
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;      /* board id for linux */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR; /* adress of boot parameters */

	iproc_clk_enum();

#if defined(CONFIG_L2C_AS_RAM)
	v7_outer_cache_inval_all();

	printf("Unlocking L2 Cache ...");
	v7_outer_cache_unlock_all();
	printf("Done\n");

	/*
	 * Relocate MMU table from flash to DDR since flash may not be always accessable.
	 * eg. When QSPI controller is in MSPI mode.
	 */
	asm volatile ("mcr p15, 0, %0, c2, c0, 0"::"r"(mmu_table_addr)); /* Update TTBR0 */
	asm volatile ("mcr p15, 0, r1, c8, c7, 0");  /* Invalidate TLB*/
	asm volatile ("mcr p15, 0, r1, c7, c10, 4"); /* DSB */
	asm volatile ("mcr p15, 0, r0, c7, c5, 4"); /* ISB */
#endif
    return 0;
}

/*
 * dram_init - sets u-boot's idea of sdram size
 */
int dram_init(void)
{
#if !defined(CONFIG_IPROC_NO_DDR) && !defined(CONFIG_NO_CODE_RELOC)
    uint32_t val;

    printf("Reset XGPLL\n");
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x0060c000);
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x1060c000);
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_3, 0x00000120);
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_5, 0x005e0083);
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_7, 0x00806204);
    __udelay(2000);
    printf("Release reset\n");
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x1460c000);
    printf("Polling\n");
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_7, 0x00806204);
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_7, 0x00806604);
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_7, 0x00806204);

    val = reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_STATUS);
    while(1) {
         if (val & (0x1 << 31)) {
             break;
         }
         val = reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_STATUS);
    }
    printf("Locked\n");
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x1c60c000);
    reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x0c60c000);

    ddr_init2();
#endif
    gd->ram_size = CONFIG_PHYS_SDRAM_1_SIZE - CONFIG_PHYS_SDRAM_RSVD_SIZE;
	return 0;
}

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = gd->ram_size;
}

/*
 *  Clear ihost BISR content through top registers
 */
void board_early_fix (void)
{
    u32 val;

    cmicd_init_soc();
    /* Enable SW_TAP_DIS[Bit 7] = 0 */
    val = cmicd_schan_read_top(TOP_MISC_CONTROL_3);
    val &= ~(0x1 << 7);
    cmicd_schan_write_top(TOP_MISC_CONTROL_3, val);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2e);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2e);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x3ff7d000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0xbfbfffff);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x3ff7d000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0xbfbbffff);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x3ff7d000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0xbfbbffff);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    val |= (0x1 << 7);
    cmicd_schan_write_top(TOP_MISC_CONTROL_3, val);
    return;
}

int board_early_init_f(void)
{
    uint32_t val;
    val = reg32_read((volatile uint32_t *)ICFG_CHIP_ID_REG);
    if ((val & 0xfff0) == 0xb060) {
        /* Ranger2 - 5606x: CPU 1250MHz, AXI 400MHz, DDR 800MHz */
        armpll_config_p7(1250);
        iproc_config_genpll(0);
    } else {
        /* Greyhound - 5341x: CPU 600MHz, AXI 200MHz, DDR 667MHz */
        armpll_config_p7(600);
        iproc_config_genpll(1);
    }

#if defined(CONFIG_I2C_MULTI_BUS) && (CONFIG_SYS_MAX_I2C_BUS > 2)
    /* Enable CMICD I2C master mode */
    val = reg32_read((volatile uint32_t *)CMIC_OVERRIDE_STRAP);
    val |= ((0x1 << CMIC_OVERRIDE_STRAP__ENABLE_OVERRIDE_I2C_MASTER_SLAVE_MODE) |
            (0x1 << CMIC_OVERRIDE_STRAP__I2C_MASTER_SLAVE_MODE));
    reg32_write((volatile uint32_t *)CMIC_OVERRIDE_STRAP, val);
#endif

    /* Assign two 64MB address regions for PNOR */
    val = 0xfce8;
    reg32_write((volatile uint32_t *)ICFG_PNOR_CONFIG_CS_0, val);
    val = 0xfcec;
    reg32_write((volatile uint32_t *)ICFG_PNOR_CONFIG_CS_1, val);

#ifndef CONFIG_IPROC_NO_DDR
    /* Initialize ATCM and BTCM for SHMOO (clear to avoid ECC error) */
    {
        int i;
        volatile int *p;
        p = (volatile int *)0x01000000;
        for(i=0; i<64*1024/4; i++, p++)
            *p = 0;
        p = (volatile int *)0x01080000;
        for(i=0; i<128*1024/4; i++, p++)
            *p = 0;
    }
#endif
    return 0;
}

int board_late_init(void) 
{
    disable_interrupts();

#if defined(CONFIG_SHMOO_AND28_REUSE)
	save_shmoo_to_flash();
#endif
    return 0;
}

#ifdef CONFIG_ARMV7_NONSEC
void smp_set_core_boot_addr(unsigned long addr, int corenr)
{
}

void smp_kick_all_cpus(void)
{
}

void smp_waitloop(unsigned previous_address)
{
}
#endif

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
