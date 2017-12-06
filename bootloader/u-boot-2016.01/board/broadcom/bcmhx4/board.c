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
extern int linux_usbh_init(void);
extern void ethHw_serdesEarlyInit(int idx);

/* Returns core voltage in milli volts mV */
/* Applicable only to core voltage ADC on Helix4 SVK */
static
int get_core_voltage_adc(uint32_t *coreV) {
    uint32_t val, i;
	/* Enable SMBus */
	reg32_set_bits((volatile uint32_t *)CMIC_I2CM_SMBus_Config, 1 << CMIC_I2CM_SMBus_Config__SMB_EN);
	/* Write the ADC device address (0x28 on SVK), bit 7-1 is address and bit 0 is R/~W */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, 0x00000050);

	/* Write ADC data aquisition command with channel0, 
	   input volatage range 0 - 5V */ /* Indicate this is the end of write */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, 0x80000080);

	/* Issue "Send Byte" command */

	val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command, 0x80000200);

	i = 10000;
	do {
		val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	} while(--i && (val & 0x80000000));

	/* Clear FIFOs */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_FIFO_control, 0xc0000000);

	if(i == 0 || (val & (0x7 << 25))) {
		printf("i=%d, val = %x\n", i, val);
		return (1); /* Command is not successful, return error */
	}
	/* Add some delay, just in case */
	for(i = 0; i < 10000; i++)
		val = i * 2;

	/* Now read two bytes from ADC */
	/* Write the ADC device address (0x28 on SVK), bit 7-1 is address and bit 0 is R/~W */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, 0x80000051);

	/* Issue "Read Word" command */

	val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command, 0x80000c00);

	i = 10000;
	do {
		val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	} while(--i && (val & 0x80000000));

	if(i == 0 || (val & (0x7 << 25))) {
		/* Clear FIFOs */
		reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_FIFO_control, 0xc0000000);
		printf("i=%d, val = %x\n", i, val);
		return (1); /* Command is not successful, return error */
	}
	/* Get 12 bit value */
	*coreV = (reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Read) & 0xFF) << 4; /* MSB 8 bits */
	//printf("coreV: %x\n", *coreV);
	*coreV |= (reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Read) >> 4) & 0xF; /* LSB 4 bits */
	//printf("coreV: %x\n", *coreV);
	/* convert coreV into mV */
	*coreV = (*coreV * 5000)/4096;
	//printf("coreV: %x\n", *coreV);
	 return(0);
}



/* Applicable only to core voltage ADC on Helix4 SVK */
/* Pass coreV in milli volts, mV */
static
int set_core_voltage_dac(uint32_t coreV) {
    uint32_t val, i, steps;
	if(coreV > 1050 || coreV < 900) {
		printf("Error: Unsupported core voltage: %d mV\n", coreV);
		return(1);
	}

	/* 1024 steps cover the range of 200mV, from 900mV to 1100mV */
	steps = ((1100 - coreV) * 1024) / 200;
	if(steps == 1024)
		steps -= 1; /* to prevent rollover */

	/* Enable SMBus */
	reg32_set_bits((volatile uint32_t *)CMIC_I2CM_SMBus_Config, 1 << CMIC_I2CM_SMBus_Config__SMB_EN);
	/* Clear FIFOs */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_FIFO_control, 0xc0000000);
	
	/* Write the DAC device address (0x2C on SVK), bit 7-1 is address and bit 0 is R/~W */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, 0x00000058);

	/* 0x3FF -> 0.9v, 0x300 -> 0.95v, 0x200 -> 1.00v, 0x100 -> 1.05v, 0x0 -> 1.1v */
	/* Write BIT D9 - D8 */ 
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, (steps >> 8) & 0xFF);

	/* Write BIT D7 - D0 */ 
	/* Indicate this is the end of write */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, 0x80000000 | (steps & 0xFF));

	/* Issue "Write Byte" command */
	val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command, 0x80000600);

	i = 10000;
	do {
		val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	} while(--i && (val & 0x80000000));


	if(i == 0 || (val & (0x7 << 25))) {
		printf("i=%d, val = %x\n", i, val);
		return (1); /* Command is not successful, return error */
	}
	 return(0);
}

/* Applicable only to core voltage ADC on Helix4 SVK */
/* Pass coreV in milli volts, mV */
static
int set_smbus_mux(uint32_t channel) {
    uint32_t val, i;

	/* Enable SMBus */
	reg32_set_bits((volatile uint32_t *)CMIC_I2CM_SMBus_Config, 1 << CMIC_I2CM_SMBus_Config__SMB_EN);
	/* Clear FIFOs */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_FIFO_control, 0xc0000000);
	
	/* Write the MUX device address (0x76 on SVK), bit 7-1 is address and bit 0 is R/~W */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, 0x000000E6);


	/* Indicate this is the end of write */
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Data_Write, 0x80000000 | 1 << channel);

	/* Issue "Send Byte" command */
	val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	reg32_write((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command, 0x80000200);

	i = 10000;
	do {
		val  = reg32_read((volatile uint32_t *)CMIC_I2CM_SMBus_Master_Command);
	} while(--i && (val & 0x80000000));


	if(i == 0 || (val & (0x7 << 25))) {
		printf("i=%d, val = %x\n", i, val);
		return (1); /* Command is not successful, return error */
	}
	 return(0);
}

static
uint32_t cmicd_schan_read_top(uint32_t addr) {
    uint32_t read;
    read=0x0;

    reg32_write((volatile uint32_t *)CMIC_CMC0_SCHAN_MESSAGE0, 0x2c600200);
    reg32_write((volatile uint32_t *)CMIC_CMC0_SCHAN_MESSAGE1, addr);

    reg32_write((volatile uint32_t *)CMIC_CMC0_SCHAN_CTRL, 0x1);

    while (read != 0x2) {
       read = reg32_read((volatile uint32_t *)CMIC_CMC0_SCHAN_CTRL); 
    }
    read = reg32_read((volatile uint32_t *)CMIC_CMC0_SCHAN_MESSAGE1);
    return read;
}

static
void cmicd_init_soc () {
    /* Configure SBUS Ring Map */
    reg32_write((volatile uint32_t *)CMIC_SBUS_RING_MAP_0_7, 0x7752100);
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

/*****************************************************************
 * misc_init_r - miscellaneous platform dependent initializations
 ******************************************************************/
int misc_init_r (void)
{
    return(0);
}

/*****************Helix4 SKUs *************
Device	Part Number	Frequency
FireScout	56548	315MHz
			56547	315MHz
		
Helix4	56344	315MHz
		56342	215MHz
		56340	250MHz
		
Spiral	56049	250MHz
		56048	315MHz
		56047	315MHz

Ranger	56042	250MHz
		56041	250MHz
		56040	315MHz

******************************************/

/**********************************************
 * dram_init - sets uboots idea of sdram size
 **********************************************/
int dram_init (void)
{
	uint32_t sku_id, avs, coreV_req, coreV;
	int status;

#if !defined(CONFIG_SPL_BUILD)
	/* init serdes early */
	ethHw_serdesEarlyInit(0);
#endif

#if !defined(CONFIG_SPL) || defined(CONFIG_SPL_BUILD)
	bench_screen_test1();

	/* Get the required core voltage from TOP_AVS_SEL register, 
	 * and set the VRM (DAC) accordingly 
	 */
	cmicd_init_soc();
	avs = cmicd_schan_read_top(TOP_AVS_SEL_REG) & 0x07;
	/* AVS 0 -> 1.00V, 1 -> 0.95V, 2 -> 0.90V, 3 -> 0.85V, 4 -> 0.80V */
	coreV_req = 1000 - (avs * 50); 
	if(coreV_req < 900)
		coreV_req = 900;
	printf("AVS: 0x%x\n", avs);
	status = set_smbus_mux(0);
	if(status != 0) {
		printf("Error: in set_smbus_mux\n");
		return(status);
	}

	status = set_core_voltage_dac(coreV_req);
	if(status != 0) {
		printf("Error: in set_core_voltage_dac\n");
		return(status);
	}

	status  = get_core_voltage_adc(&coreV);
	if(status != 0) {
		printf("Error: in get_core_voltage_adc\n");
		return(status);
	}
	printf("Core Voltage set to: %d mV\n", coreV);

	ddr_init2();
#endif

	sku_id = (reg32_read((volatile uint32_t *)CMIC_DEV_REV_ID)) & 0x0000ffff;
	if(sku_id == 0xb040 || sku_id == 0xb041 || sku_id == 0xb042) {
		/* Ranger SVK board */
		gd->ram_size = RANGER_SVK_SDRAM_SIZE - CONFIG_PHYS_SDRAM_RSVD_SIZE;
	}
	else {
		/* default Helix4, Firescout and Spiral */
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
