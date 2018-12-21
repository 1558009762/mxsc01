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

#define SABER2_DEVID_56268 0xb268
#define SABER2_A0_REV_ID      0x1
#define SABER2_B0_REV_ID      0x11
#define SOC_IS_A0_REV() ((reg32_read((volatile u32 *)CMIC_DEV_REV_ID) >> 16) == SABER2_A0_REV_ID)
#define SOC_IS_B0_REV() ((reg32_read((volatile u32 *)CMIC_DEV_REV_ID) >> 16) == SABER2_B0_REV_ID)
#define SOC_IS_56268() ((reg32_read((volatile u32 *)CMIC_DEV_REV_ID) & 0xFFFF) == SABER2_DEVID_56268)

extern void iproc_clk_enum(void);
extern void ddr_init2(void);
extern void save_shmoo_to_flash(void);

static
u32 cmicd_schan_read_top (u32 addr) {
    u32 read = 0x0;

    reg32_write((volatile u32 *)CMIC_CMC0_SCHAN_MESSAGE0, 0x2c680200);
    reg32_write((volatile u32 *)CMIC_CMC0_SCHAN_MESSAGE1, addr);

    reg32_write((volatile u32 *)CMIC_CMC0_SCHAN_CTRL, 0x1);

    while (read != 0x2) {
       read = reg32_read((volatile u32 *)CMIC_CMC0_SCHAN_CTRL);
    }
    read = reg32_read((volatile u32 *)CMIC_CMC0_SCHAN_MESSAGE1);
    return read;
}

static
u32 cmicd_schan_write_top (u32 addr, u32 val) {
    u32 read = 0x0;

    reg32_write((volatile u32 *)CMIC_CMC0_SCHAN_MESSAGE0, 0x34680200);
    reg32_write((volatile u32 *)CMIC_CMC0_SCHAN_MESSAGE1, addr);
    reg32_write((volatile u32 *)CMIC_CMC0_SCHAN_MESSAGE2, val);

    reg32_write((volatile u32 *)CMIC_CMC0_SCHAN_CTRL, 0x1);

    while (read != 0x2) {
       read = reg32_read((volatile u32 *)CMIC_CMC0_SCHAN_CTRL);
    }
    return read;
}

static
void cmicd_init_soc (void) {
    /* Configure SBUS Ring Map for TOP, block id = 13, ring number = 3 */
    reg32_write((volatile u32 *)CMIC_SBUS_RING_MAP_0_7, 0x66034000);
    reg32_write((volatile u32 *)CMIC_SBUS_RING_MAP_8_15, 0x55312222);
    reg32_write((volatile u32 *)CMIC_SBUS_RING_MAP_16_23, 0x03103775);
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
#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
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
#ifndef CONFIG_NAND_IPROC_BOOT
static
void board_early_fix(void)
{
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2e);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2e);
    
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x1fd00000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0xbffffdfe);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40100000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x80000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x1fd00000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0xbffffdfe);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40100000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40100000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x40000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    cmicd_schan_write_top(TOP_UC_TAP_WRITE_DATA, 0x80000000);
    cmicd_schan_write_top(TOP_UC_TAP_CONTROL, 0x2f);
    
    return;
}
#endif
int board_early_init_f(void)
{
    u32 val;
#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
	u32 pio_endian;

	pio_endian = reg32_read((volatile u32 *)CMIC_COMMON_UC0_PIO_ENDIANESS);
	reg32_write((volatile u32 *)CMIC_COMMON_UC0_PIO_ENDIANESS, 0x0);

    cmicd_init_soc();

    if (!SOC_IS_56268() && SOC_IS_A0_REV()) {
#ifndef CONFIG_NAND_IPROC_BOOT
        /* Fix boot failure that CCU access being blocked by TAP controller */
        board_early_fix();
#endif
    }

	if (SOC_IS_A0_REV()) {
		/* Enable JTAG, ENABLE[Bit 1] = 1 */
		val = cmicd_schan_read_top(TOP_UC_TAP_CONTROL);
		val |= (0x1 << 1);
		cmicd_schan_write_top(TOP_UC_TAP_CONTROL, val);

		val = reg32_read((volatile u32 *)PAXB_0_STRAP_STATUS);
		if ((val & (1 << PAXB_0_STRAP_STATUS__STRAP_IPROC_PCIE_USER_RC_MODE)) != 0) {
			/* PCIE in RC mode, select 100MHz clock from iProc DDR LCPLL */
			/* Selected by strap pin in SB2 B0 */
			val = reg32_read((volatile u32 *)IPROC_WRAP_PCIE_SERDES_CONTROL);
			val &= ~(0x7 << IPROC_WRAP_PCIE_SERDES_CONTROL_REFCLK_IN_SEL);
			reg32_write((volatile u32 *)IPROC_WRAP_PCIE_SERDES_CONTROL, val);
		}
	}

	/* Stabilize IPROC DDR PLL */
	reg32_write((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_2, 0x00805004);
	val = reg32_read((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_6);
	val &= ~(0xffff);
	val |= 0x22;
	reg32_write((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_6, val);
	reg32_write((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_3, 0x3c414d40);

	/* Reset DDR PLL */
	val = reg32_read((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_0);
	val |= (0x1 << IPROC_DDR_PLL_SW_OVWR);
	val &= ~((0x1 << IPROC_DDR_PLL_RESETB) | (0x1 << IPROC_DDR_PLL_POST_RESETB));
	reg32_write((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_0, val);
	val |= (0x1 << IPROC_DDR_PLL_RESETB);
	reg32_write((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_0, val);

	val = reg32_read((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_STATUS);
	/* Wait untill PLL is locked */
	while (1) {
		if (val & (0x1 << IPROC_DDR_PLL_LOCK)) {
			break;
		}
		val = reg32_read((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_STATUS);
	}

	val = reg32_read((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_0);
	val |= (0x1 << IPROC_DDR_PLL_POST_RESETB);
	reg32_write((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_0, val);

	val &= ~(0x1 << IPROC_DDR_PLL_SW_OVWR);
	reg32_write((volatile u32 *)IPROC_WRAP_IPROC_DDR_PLL_CTRL_0, val);

	/* Master LCPLL */
	cmicd_schan_write_top(TOP_XGXS0_PLL_CONTROL_3, 0x00805004);
	val = cmicd_schan_read_top(TOP_XGXS0_PLL_CONTROL_7);
	val &= ~(0xffff);
	val |= 0x22;
	cmicd_schan_write_top(TOP_XGXS0_PLL_CONTROL_7, val);
	cmicd_schan_write_top(TOP_XGXS0_PLL_CONTROL_4, 0x3c5caf40);
	/* Serdes LCPLL */
	val = cmicd_schan_read_top(TOP_SERDES_LCPLL_FBDIV_CTRL_1);
	val &= ~(0xffff);
	val |= 0x7d0;
	cmicd_schan_write_top(TOP_SERDES_LCPLL_FBDIV_CTRL_1, val);
	cmicd_schan_write_top(TOP_XGXS1_PLL_CONTROL_1, 0x197d0714);
	cmicd_schan_write_top(TOP_XGXS1_PLL_CONTROL_2, 0x40004015);
	cmicd_schan_write_top(TOP_XGXS1_PLL_CONTROL_3, 0x897004);
	val = cmicd_schan_read_top(TOP_XGXS1_PLL_CONTROL_7);
	val &= ~(0xffff);
	val |= 0x22;
	cmicd_schan_write_top(TOP_XGXS1_PLL_CONTROL_7, val);
	cmicd_schan_write_top(TOP_XGXS1_PLL_CONTROL_4, 0x3c0caf40);
    /* BS0 LCPLL */
    cmicd_schan_write_top(TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_3, 0x00805004);
    val = cmicd_schan_read_top(TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_6);
    val &= ~(0xffff);
    val |= 0x22;
    cmicd_schan_write_top(TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_6, val);
    cmicd_schan_write_top(TOP_BROAD_SYNC0_PLL_CTRL_REGISTER_4, 0x2f172bd);
    /* BS1 LCPLL */
    cmicd_schan_write_top(TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_3, 0x00805004);
    val = cmicd_schan_read_top(TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_6);
    val &= ~(0xffff);
    val |= 0x22;
    cmicd_schan_write_top(TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_6, val);
    cmicd_schan_write_top(TOP_BROAD_SYNC1_PLL_CTRL_REGISTER_4, 0x2f172bd);

    /* Toggle LCPLL soft reset. */
    cmicd_schan_write_top(TOP_LCPLL_SOFT_RESET_REG, 0x1);
    cmicd_schan_write_top(TOP_LCPLL_SOFT_RESET_REG, 0x0);

    /* 5626x: CPU 1000MHz, AXI 400MHz, DDR 800MHz
     * AXI: 400/200 MHz for high/low sku controlled by otp
     */
    armpll_config_p7(1000);
#endif

#ifndef CONFIG_SGMII_REFCLCK_SEL_EXT
    /* Select SGMII reference clock from Master LCPLL by default */
    val = reg32_read((volatile u32 *)IPROC_WRAP_MISC_CONTROL);
    if ((val & (0x7 << IPROC_WRAP_MISC_CONTROL__SGMII_REFCLK_SEL)) != 0) {
        val &= ~(0x7 << IPROC_WRAP_MISC_CONTROL__SGMII_REFCLK_SEL);
        reg32_write((volatile u32 *)IPROC_WRAP_MISC_CONTROL, val);
    }
#endif

#if defined(CONFIG_I2C_MULTI_BUS) && (CONFIG_SYS_MAX_I2C_BUS > 2)
    /* Enable CMICD I2C master mode */
    val = reg32_read((volatile uint32_t *)CMIC_OVERRIDE_STRAP);
    val |= ((0x1 << CMIC_OVERRIDE_STRAP__ENABLE_OVERRIDE_I2C_MASTER_SLAVE_MODE) |
            (0x1 << CMIC_OVERRIDE_STRAP__I2C_MASTER_SLAVE_MODE));
    reg32_write((volatile uint32_t *)CMIC_OVERRIDE_STRAP, val);
#endif

    val = cmicd_schan_read_top(TOP_SWITCH_FEATURE_ENABLE_3);
    if (!(val & (0x1 << USB_DEV_MODE_STRAP))) {
        /* Enable this bit to indicate usb in host mode for linux driver */
        val = reg32_read((volatile u32 *)IPROC_WRAP_IPROC_STRAP_CTRL);
        val |= (0x1 << IPROC_WRAP_IPROC_STRAP_CTRL__DISABLE_USB2D);
        reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_STRAP_CTRL, val);
    }

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
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
	if (pio_endian) {
		reg32_write((volatile u32 *)CMIC_COMMON_UC0_PIO_ENDIANESS, 0x01010101);
	}
#endif
	return 0;
}
#if defined(CONFIG_L2C_AS_RAM)
extern ulong mmu_table_addr;
#endif
int board_late_init(void) 
{
    int status = 0;
	u32 pio_endian;

    disable_interrupts();

#if defined(CONFIG_SHMOO_AND28_REUSE)
     save_shmoo_to_flash();
#endif

	pio_endian = reg32_read((volatile u32 *)CMIC_COMMON_UC0_PIO_ENDIANESS);
	reg32_write((volatile u32 *)CMIC_COMMON_UC0_PIO_ENDIANESS, 0x0);

   if (SOC_IS_A0_REV()) {
        printf("TOP_UC_TAP_CONTROL = 0x%x\n", cmicd_schan_read_top(TOP_UC_TAP_CONTROL));
    }
	printf("TOP_XGXS0_PLL_CONTROL_3 = 0x%x\n", cmicd_schan_read_top(TOP_XGXS0_PLL_CONTROL_3));
	printf("TOP_SWITCH_FEATURE_ENABLE_3 = 0x%x\n", cmicd_schan_read_top(TOP_SWITCH_FEATURE_ENABLE_3));

	if (pio_endian) {
		reg32_write((volatile u32 *)CMIC_COMMON_UC0_PIO_ENDIANESS, 0x01010101);
	}

    return status;
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
