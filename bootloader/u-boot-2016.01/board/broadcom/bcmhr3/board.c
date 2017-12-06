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

#define HR3_B0_REV_ID	0x11
#define SOC_IS_B0_REV() ((reg32_read((volatile u32 *)CMIC_DEV_REV_ID) >> 16) == HR3_B0_REV_ID)

extern void iproc_clk_enum(void);
extern void ddr_init2(void);
extern void save_shmoo_to_flash(void);
extern uint32_t iproc_config_genpll(uint32_t mode);

#if defined(CONFIG_MMC) || defined(CONFIG_BOARD_UPI_ICID)
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
#endif /* defined(CONFIG_MMC) || defined(CONFIG_BOARD_UPI_ICID) */

#ifdef CONFIG_MMC
static void board_sdio_init(void)
{
    u32 val;

    cmicd_init_soc();

    /* Enable SDIO 8 bit mode */
    val = cmicd_schan_read_top(TOP_SDIO_MISC_CONTROL);
    val |= (0x1 << TOP_SDIO_MISC_CONTROL__TOP_SDIO_8B_INF);
    val |= (0xf << TOP_SDIO_MISC_CONTROL__TOP_SDIO_GPIO_INF_SEL_R);
    cmicd_schan_write_top(TOP_SDIO_MISC_CONTROL, val);
}
#endif

#ifdef CONFIG_BOARD_UPI_ICID
/*
 * GPIO[4] -> CLK_int
 * GPIO[5] -> ENA_int
 * GPIO[6] -> ENB_int
 * GPIO[7] -> SW_EN
 * GPIO[8] -> OUT_int
 */
void board_read_icid(void)
{
	volatile int i;
	u32 val, gp_out_en, gp_data_out;

	gp_out_en = reg32_read((volatile uint32_t *)ChipcommonG_GP_OUT_EN);
	gp_data_out = reg32_read((volatile uint32_t *)ChipcommonG_GP_DATA_OUT);

	cmicd_init_soc();

    /* Enable GPIO for UPI mode */
	val = cmicd_schan_read_top(TOP_MISC_CONTROL_3);
	val |= (1 << TOP_MISC_CONTROL_3__GPIO_FOR_UPI_EN);
	cmicd_schan_write_top(TOP_MISC_CONTROL_3, val);

	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0x0);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_OUT_EN, 0xf0);
	/* Toggle clock */
	for (i = 0; i < 5;i++) {
		reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0x10);
		reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0x0);
	}

	/* Enable SW_EN */
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0x80);
	/* Enable ENA_int */
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xa0);
	/* Toggle clock */
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xb0);
	/* First ENB_int */
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xf0);
	/* Toggle clock */
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xe0);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xf0);
	/* ENB_int low */
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xb0);
	for (i = 0; i < 5;i++) {
		reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xb0);
		reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xa0);
	}
	/* Second ENB_int */
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xe0);
	for (i = 0; i < 10000; i++);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xf0);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xe0);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xa0);

	reg32_read((volatile uint32_t *)ChipcommonG_GP_DATA_IN);
	for (i = 0; i < 256; i++) {
		reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xb0);
		reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xa0);
		printf("%d", (reg32_read((volatile uint32_t *)ChipcommonG_GP_DATA_IN) & 0x100) >> 8);
	}
	printf("\n");
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xb0);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0xa0);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0x80);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, 0x0);

	reg32_write((volatile uint32_t *)ChipcommonG_GP_DATA_OUT, gp_data_out);
	reg32_write((volatile uint32_t *)ChipcommonG_GP_OUT_EN, gp_out_en);
	/* Disable GPIO for UPI mode */
	val = cmicd_schan_read_top(TOP_MISC_CONTROL_3);
	val &= ~(1 << TOP_MISC_CONTROL_3__GPIO_FOR_UPI_EN);
	cmicd_schan_write_top(TOP_MISC_CONTROL_3, val);
}
#endif /* CONFIG_BOARD_UPI_ICID */

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
	/* Switch flash PAD mux to PNOR so it can be probed in board_r.c */
	uint32_t sku;
	sku = reg32_read((volatile uint32_t *)ICFG_CHIP_ID_REG);
	if ((sku & 0xf) != 0x4) {
		/* NOR/NAND are disabled on Hurricane3-lite */
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
    }
#endif /* !CONFIG_SYS_NO_FLASH && !CONFIG_NOR_IPROC_BOOT */
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE;      /* board id for linux */
	gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR; /* adress of boot parameters */

#ifdef CONFIG_MMC
	board_sdio_init();
#endif /* CONFIG_MMC */

	iproc_clk_enum();

#if defined(CONFIG_L2C_AS_RAM)
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
	uint32_t val;
	int is25Mhz = 1;

	if (reg32_read((volatile uint32_t *)IPROC_WRAP_TOP_STRAP_STATUS_1) & 0x01) {
		/* 50 MHz */
		is25Mhz = 0;
	}

	printf("Reset XGPLL\n");
	reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x1060c000);

    if (SOC_IS_B0_REV()) {
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_3, 0x00000020);
		if (is25Mhz) {
			reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_4, 0x84);
			reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_5, 0x000d406f);
		} else {
			reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_4, 0x42);
			reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_5, 0x005d402f);
		}
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_7, 0xC0804204);
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_2, 0x0c000042);
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_0, 0x21116e21);
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_6, 0x002200a2);
	} else {
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_3, 0x00000120);
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_5, 0x005e0083);
		reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_7, 0xC0806204);
	}
	__udelay(2000);
	printf("Release reset\n");
	reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x1460c000);
	printf("Polling\n");

	val = reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_STATUS);
	while(1) {
		if (val & (0x1 << 31)) {
			break;
		}
		val = reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_STATUS);
	}
	printf("Locked\n");
	reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, 0x1c60c000);

#if !defined(CONFIG_IPROC_NO_DDR)
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

int board_early_init_f(void)
{
    uint32_t val;
    val = reg32_read((volatile uint32_t *)ICFG_CHIP_ID_REG);
    val &= 0xffff;
    if ((val >= 0xb163) && (val <= 0xb166)) {
        /* Hurricane3-lite - 56163/4/6: CPU 600MHz, AXI 200MHz, DDR 667MHz */
        armpll_config_p7(600);
        iproc_config_genpll(1);
    } else {
        /* Hurricane3/Buckhound - 56160/2,53440/2/3: CPU 1250MHz, AXI 400MHz, DDR 800MHz */
        armpll_config_p7(1250);
        iproc_config_genpll(0);
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
    return 0;
}

int board_late_init(void) 
{
    int status = 0;

    disable_interrupts();

#if defined(CONFIG_SHMOO_AND28_REUSE)
     save_shmoo_to_flash();
#endif

#if defined(CONFIG_L2C_AS_RAM)
    v7_outer_cache_inval_all();

    printf("Unlocking L2 Cache ...");
    v7_outer_cache_unlock_all();
    printf("Done\n");
#endif
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
