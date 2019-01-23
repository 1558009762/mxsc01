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
extern void iproc_save_shmoo_values(void);
extern int linux_usbh_init(void);
extern void ethHw_serdesEarlyInit(int idx);


#define IPROC_GPIO_CCA_BASE 0x18000060
#define CMIC_GPIO_CCB_BASE  0x48002000

/* 
 * Chip level GPIO 0-3 from CMICD, 
 * GPIO 4-11 from ChipcommonA gpio pin 0 - 7
 * Hence the base is 0 and the number is 11.
 */

#define REGOFFSET_GPIO_DIN          0x000 /* GPIO Data in register */
#define REGOFFSET_GPIO_DOUT         0x004 /* GPIO Data out register */
#define REGOFFSET_GPIO_EN           0x008 /* GPIO driver enable register */


#if 1
void bcmgpio_directory_output(int gpio, unsigned char val)
{
	u32 read = 0;
	u32 gpiobase = 0;

	if(gpio <= 3)//cmic gpio
	{
		gpiobase = CMIC_GPIO_CCB_BASE;
		reg32_write((volatile u32 *)(gpiobase + 0x28), 0); // set CMIC_GP_AUX_SEL 
	}
	else //chipcommon A gpio
	{
		gpiobase = IPROC_GPIO_CCA_BASE;
		gpio = gpio - 4;
	}

	read = reg32_read((volatile u32 *)(gpiobase + REGOFFSET_GPIO_EN));
	read |= 1 << gpio;
	reg32_write((volatile u32 *)(gpiobase + REGOFFSET_GPIO_EN), read);
		
	read = 0;
	read = reg32_read((volatile u32 *)(gpiobase + REGOFFSET_GPIO_DOUT));
	read = val ? (read |(1<<gpio)):(read &(~(1<<gpio)));
	reg32_write((volatile u32 *)(gpiobase + REGOFFSET_GPIO_DOUT), read);
}
void bcmgpio_directory_input(int gpio)
{
	u32 read = 0;
	u32 gpiobase = 0;

	if(gpio <= 3)//cmic gpio
	{
		gpiobase = CMIC_GPIO_CCB_BASE;
		reg32_write((volatile u32 *)(gpiobase + 0x28), 0); // set CMIC_GP_AUX_SEL
	}
	else //chipcommon A gpio
	{
		gpiobase = IPROC_GPIO_CCA_BASE;
		gpio = gpio - 4;
	}

	read = reg32_read((volatile u32 *)(gpiobase + REGOFFSET_GPIO_EN));
	read &= ~(1 << gpio);
	reg32_write((volatile u32 *)(gpiobase + REGOFFSET_GPIO_EN), read);
}	
unsigned int bcmgpio_get_value(int gpio)
{
	u32 read = 0;
	u32 gpiobase = 0;

	if(gpio <= 3)//cmic gpio
	{
		gpiobase = CMIC_GPIO_CCB_BASE;
		reg32_write((volatile u32 *)(gpiobase + 0x28), 0); // set CMIC_GP_AUX_SEL
	}
	else //chipcommon A gpio
	{
		gpiobase = IPROC_GPIO_CCA_BASE;
		gpio = gpio - 4;
	}
	read = reg32_read((volatile u32 *)(gpiobase + REGOFFSET_GPIO_DIN));
	printf("read=0x%08x\n",read);
	return read & (1 << gpio)?1:0;
}
#endif

 /*add by lihz 2018-12-20*/
 static void reset_by_gpio()
 {
	/* init cpu extern interupt from gpio5/6 */
 	bcmgpio_directory_output(5, 1);
	bcmgpio_directory_output(6, 1);

#if 0	
 	/* enable cpu read/write flash */
	bcmgpio_directory_output(0, 1);
	/* disable watch dog */
	bcmgpio_directory_output(1, 1);
	/* enable 7A75T read/write flash */
	bcmgpio_directory_output(4, 1);
	/* enable XCKU60 read/write flash */
	bcmgpio_directory_output(9, 1);
#endif	
	/* reset 7A75T */
	//bcmgpio_directory_output(3, 0);
	//udelay(20000);
	//bcmgpio_directory_output(3, 1);
	/* reset XCKU60 */
	bcmgpio_directory_output(8, 0);
	udelay(20000);
	bcmgpio_directory_output(8, 1);
	/* reset DM8606C */
	bcmgpio_directory_output(10, 0);
	udelay(20000);
	bcmgpio_directory_output(10, 1);
	/* reset 88E1512 */
	bcmgpio_directory_output(11, 0);
	udelay(20000);
	bcmgpio_directory_output(11, 1);

 }

/*****************************************
 * board_init -early hardware init
 *****************************************/
int board_init (void)
{
    gd->bd->bi_arch_number = CONFIG_MACH_TYPE;      /* board id for linux */
    gd->bd->bi_boot_params = LINUX_BOOT_PARAM_ADDR; /* adress of boot parameters */
    iproc_clk_enum();
    return 0;
}

/**********************************************
 * dram_init - sets uboots idea of sdram size
 **********************************************/
int dram_init (void)
{
	uint32_t sku_id;
#if !defined(CONFIG_SPL_BUILD)
	/* init serdes early */
	ethHw_serdesEarlyInit(0);
#endif

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
	ddr_init2();
#endif
	sku_id = (reg32_read((volatile uint32_t *)CMIC_DEV_REV_ID)) & 0x0000ffff;
	if(sku_id == 0xb040) {
		/* Ranger SVK board */
		gd->ram_size = ENDURO3_SVK_SDRAM_SIZE - CONFIG_PHYS_SDRAM_RSVD_SIZE;
	}
	else {
		/* default Katana2 */
		gd->ram_size = CONFIG_PHYS_SDRAM_1_SIZE - CONFIG_PHYS_SDRAM_RSVD_SIZE;
	}

    return 0;
}

int board_early_init_f (void)
{
#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
	armpll_config(1000);
#endif
	return 0;
}

int board_late_init (void) 
{
	int status = 0;

	disable_interrupts();

#if !defined(CONFIG_NO_CODE_RELOC)
	extern ulong mmu_table_addr;
	/* 
	 * Relocate MMU table from flash to DDR since flash may not be always accessable.
	 * eg. When QSPI controller is in MSPI mode.
	 */
	asm volatile ("mcr p15, 0, %0, c2, c0, 0"::"r"(mmu_table_addr)); /* Update TTBR0 */
	asm volatile ("mcr p15, 0, r1, c7, c10, 4"); /* DSB */
	asm volatile ("mcr p15, 0, r0, c7, c5, 4"); /* ISB */
#endif

#if defined(CONFIG_RUN_DDR_SHMOO2) && defined(CONFIG_SHMOO_REUSE)
	/* Save DDR PHY parameters into flash if Shmoo was performed */
	iproc_save_shmoo_values();
#endif
	linux_usbh_init();

	/* init gpio, add by lihz - 2018.12.20 */
	reset_by_gpio();

	return status;
}

#ifdef CONFIG_BCM_XGS_ETH
extern int bcm_xgs_eth_register(bd_t *bis, u8 dev_num);
int board_eth_init(bd_t *bis)
{
	int idx, rc = -1;
	printf("Registering BCM xgs eth\n");
    for (idx = 0; idx < CONFIG_GMAC_NUM; idx++) {
		rc = bcm_xgs_eth_register(bis, idx);
    }
	return rc;
}
#endif
