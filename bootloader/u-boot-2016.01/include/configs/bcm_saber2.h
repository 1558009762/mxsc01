/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __BCM_SABER2_H
#define __BCM_SABER2_H

#include <asm/arch/configs.h>

#define CONFIG_IPROC_P7                     1
#define CONFIG_SABER2                       1

/* DDR: MT41K512M8RH-125E */
#define CONFIG_DDR_ROW_BITS                 16
#define CONFIG_DDR_COL_BITS                 10
#define CONFIG_DDR_BANK_BITS                3

#ifdef CONFIG_DDR32
#define CONFIG_DDR_BYTES_PER_COL            4
#else
#define CONFIG_DDR_BYTES_PER_COL            2
#endif

#define CONFIG_DDR_TOTAL_COLS               ((1 << CONFIG_DDR_ROW_BITS) * (1 << CONFIG_DDR_COL_BITS) * (1 << CONFIG_DDR_BANK_BITS))

#define CONFIG_PHYS_SDRAM_1_SIZE            (CONFIG_DDR_TOTAL_COLS * CONFIG_DDR_BYTES_PER_COL)

#define CONFIG_PHYS_SDRAM_1                 0x60000000
#define CONFIG_PHYS_SDRAM_RSVD_SIZE         0x0 /* bytes reserved from CONFIG_PHYS_SDRAM_1 for custom use */
#define CONFIG_SYS_SDRAM_BASE               (CONFIG_PHYS_SDRAM_1 + CONFIG_PHYS_SDRAM_RSVD_SIZE)

#define CONFIG_SYS_L2_PL310
#define CONFIG_SYS_PL310_BASE               0x19022000

#define CONFIG_SPL_MAX_FOOTPRINT			0x20000		/* 128KB */

/* SPL Features */
#ifdef CONFIG_SPL_BUILD

/*
 * SPL resides in sram
 */

#define CONFIG_SPL_FRAMEWORK
#define CONFIG_SPL_TEXT_BASE            0x2020000
#define CONFIG_SPL_STACK                CONFIG_SYS_INIT_SP_ADDR

#define CONFIG_SYS_SPL_MALLOC_START     0x64100000
#define CONFIG_SYS_SPL_MALLOC_SIZE      0x100000        /* 1 MB */

#define CONFIG_SPL_LIBCOMMON_SUPPORT

#ifdef CONFIG_NAND_IPROC_BOOT
#define CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_NAND_SIMPLE
#define CONFIG_SPL_NAND_BASE
#define CONFIG_SPL_NAND_ECC
#define CONFIG_SPL_MTD_SUPPORT
#define CONFIG_SYS_NAND_U_BOOT_OFFS     0x20000
#define CONFIG_SYS_NAND_PAGE_SIZE       4096
#define CONFIG_SPL_NAND_SIMPLE_BBT
#define CONFIG_SPL_UBOOT_START          0xE0000000
#else
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SPL_SPI_LOAD
#define CONFIG_SPL_SPI_BUS 0
#define CONFIG_SPL_SPI_CS 0
#define CONFIG_SYS_SPI_U_BOOT_OFFS       0x20000
#define CONFIG_SYS_SPI_U_BOOT_SIZE       0xD0000
#define CONFIG_SPL_UBOOT_START           0xF0000000
#endif /* CONFIG_NAND_IPROC_BOOT */

#define CONFIG_SPL_UBOOT_END            (CONFIG_SPL_UBOOT_START + CONFIG_SPL_MAX_FOOTPRINT)
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_SERIAL_SUPPORT

#endif /* CONFIG_SPL_BUILD */

/* Clocks */
#define CONFIG_SYS_REF_CLK                  (25000000)
#define CONFIG_IPROC_PLL_REF_CLK            (50000000)
#define CONFIG_SYS_REF2_CLK                 (200000000)
#define IPROC_ARM_CLK                       (1000000000)
#define IPROC_AXI_CLK                       (400000000)
#define IPROC_APB_CLK                       (100000000)

#define PLL_AXI_CLK				             400000000
#define PLL_APB_CLK				             100000000

#define CONFIG_BOARD_EARLY_INIT_F            1
#define CONFIG_BOARD_LATE_INIT               1

/*
 * Memory configuration
 */
#define CONFIG_NR_DRAM_BANKS		1

#define CONFIG_SYS_MALLOC_LEN	(4 * 1024 * 1024)
#define CONFIG_STACKSIZE		(256 * 1024)

/* Some commands use this as the default load address */
#define CONFIG_SYS_LOAD_ADDR                0x70000000
#define CONFIG_MACH_TYPE                          4735
#define LINUX_BOOT_PARAM_ADDR               0x60200000

/*
 * This is the initial SP which is used only briefly for relocating the u-boot
 * image to the top of SDRAM. After relocation u-boot moves the stack to the
 * proper place. Saber2 uses internal SRAM.
 */
/* SRAM configuration */
#define CONFIG_IPROC_SRAM_BASE              (0x02000000)
#define CONFIG_IPROC_SRAM_SIZE              (0x80000)

#define CONFIG_SYS_INIT_SP_ADDR    (CONFIG_IPROC_SRAM_BASE + CONFIG_IPROC_SRAM_SIZE - 0x2000 - 16)

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE

/* Serial Info */
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_BAUDRATE			115200

/* Enable generic u-boot SPI flash drivers and commands */
#define CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_BAR
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_SPI_FLASH_SST
#define CONFIG_SPI_FLASH_WINBOND
#define CONFIG_SPI_FLASH_ATMEL

/* SPI flash configurations */
#define CONFIG_IPROC_QSPI
#define CONFIG_IPROC_QSPI_BUS                   0
#define CONFIG_IPROC_QSPI_CS                    0

/* SPI flash configuration - flash specific */
#define CONFIG_IPROC_BSPI_DATA_LANES            1
#define CONFIG_IPROC_BSPI_ADDR_LANES            1
#define CONFIG_IPROC_BSPI_READ_CMD              0x0b
#define CONFIG_IPROC_BSPI_READ_DUMMY_CYCLES     8
#define CONFIG_SF_DEFAULT_SPEED                 50000000
#define CONFIG_SF_DEFAULT_MODE                  SPI_MODE_3

/* NAND flash configurations */
#define CONFIG_CMD_NAND 
#define CONFIG_IPROC_NAND 
#define CONFIG_SYS_MAX_NAND_DEVICE			1
#define CONFIG_SYS_NAND_BASE		        0xdeadbeef
#define CONFIG_SYS_NAND_ONFI_DETECTION

/* I2C */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_IPROC
#define CONFIG_SYS_IPROC_I2C_SPEED    100000  /* Default on 100KHz */
#define CONFIG_SYS_IPROC_I2C_SLAVE    0xff    /* No slave address */
#define CONFIG_I2C_MULTI_BUS
#define CONFIG_SYS_MAX_I2C_BUS  3

/* No PNOR flash */
#define CONFIG_SYS_NO_FLASH

/* USB */
#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_IPROC
#define CONFIG_USB_STORAGE
#define CONFIG_LEGACY_USB_INIT_SEQ
#endif /* CONFIG_CMD_USB */

/* Environment variables */
#ifdef CONFIG_NAND_IPROC_BOOT
#define CONFIG_ENV_IS_IN_NAND                   1
#define CONFIG_ENV_OFFSET                       0x200000
#define CONFIG_ENV_RANGE                        0x200000
#elif defined(CONFIG_SPI_FLASH)
#define CONFIG_ENV_IS_IN_SPI_FLASH              1
#define CONFIG_ENV_OFFSET                       0xc0000
#define CONFIG_ENV_SPI_MAX_HZ                   10000000
#define CONFIG_ENV_SPI_MODE                     SPI_MODE_3
#define CONFIG_ENV_SPI_BUS                      CONFIG_IPROC_QSPI_BUS
#define CONFIG_ENV_SPI_CS                       CONFIG_IPROC_QSPI_CS
#define CONFIG_ENV_SECT_SIZE                    0x10000     /* 64KB */
#else /* No flash defined */
#define CONFIG_ENV_IS_NOWHERE
#endif

#define CONFIG_ENV_SIZE                         0x10000     /* 64KB */

/* 
 * SHMOO and28 needs about 64K storage for calibration process 
 * The buffter will be released after trace calibration is done
 * and final config saved into flash 
 * For Saber2, we use the second BTCM as this buffer
 */
#define CONFIG_SHMOO_SRAM_BUF                   0x01080000
#if !defined(CONFIG_ENV_IS_NOWHERE)
/* SHMOO values reuse */
#define CONFIG_SHMOO_AND28_REUSE                1
#endif
/* Memory test address to verify stored SHMOO values */
#define CONFIG_SHMOO_REUSE_MEMTEST_START        0x61000000
/* Memory test address to verify stored SHMOO values */
#define CONFIG_SHMOO_REUSE_MEMTEST_LENGTH       0x1000
/* Offset of SPI flash to save Shmoo values */
#define CONFIG_SHMOO_REUSE_QSPI_OFFSET          0x000E0000
/* Offset of NOR flash to save Shmoo values */
#define CONFIG_SHMOO_REUSE_NOR_OFFSET           0x000E0000
/* Offset of NAND flash to save Shmoo values */
#define CONFIG_SHMOO_REUSE_NAND_OFFSET          0x00400000
/* Range for the partition to support NAND bad blocks replacement */
#define CONFIG_SHMOO_REUSE_NAND_RANGE           0x00200000
/* Delay to wait for the magic character to force Shmoo; 0 to disable delay */
#define CONFIG_SHMOO_REUSE_DELAY_MSECS          50

/* console configuration */

#define CONFIG_SYS_CBSIZE		1024	/* Console buffer size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
			sizeof(CONFIG_SYS_PROMPT) + 16)	/* Printbuffer size */
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

/*
 * One partition type must be defined for part.c
 * This is necessary for the fatls command to work on an SD card
 * for example.
 */
#define CONFIG_DOS_PARTITION

/* version string, parser, etc */
#define CONFIG_VERSION_VARIABLE
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_CMDLINE_EDITING
#define CONFIG_COMMAND_HISTORY
#define CONFIG_SYS_LONGHELP

#define CONFIG_CRC32_VERIFY
#define CONFIG_MX_CYCLIC

/* Commands */
#define CONFIG_CMD_FAT
#define CONFIG_CMD_EXT2
#define CONFIG_FAT_WRITE
#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_MEMTEST_START            CONFIG_PHYS_SDRAM_1
#define CONFIG_SYS_MEMTEST_END              (CONFIG_PHYS_SDRAM_1 + CONFIG_PHYS_SDRAM_1_SIZE)
/* Flatten Device Tree support */
#define CONFIG_OF_LIBFDT
/* Flatten Image Tree support */
#define CONFIG_FIT

#define CONFIG_EXTRA_ENV_SETTINGS \
	"loadaddr=0x64000000\0" \
	"fdt_high=0xffffffff\0"

/* SHA hashing */
#define CONFIG_CMD_HASH
#define CONFIG_HASH_VERIFY
#define CONFIG_SHA1
#define CONFIG_SHA256

/* Enable Time Command */
#define CONFIG_CMD_TIME
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_TFTPPUT

/* Misc utility code */
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CRC32_VERIFY
#define CONFIG_BOOTDELAY			3	/* User can hit a key to abort kernel boot and stay in uboot cmdline */

/* For booting Linux */
#define CONFIG_CMDLINE_TAG              /* ATAG_CMDLINE setup */
#define CONFIG_SETUP_MEMORY_TAGS        /* ATAG_MEM setup */

#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_DEVICE
#define MTDIDS_DEFAULT      "nand0=nand_iproc.0"
#define MTDPARTS_DEFAULT    "mtdparts=nand_iproc.0:2M(nboot),4M(nenv),10M(nsystem),48M(nrootfs),-(ncustfs)"

#endif /* __BCM_SABER2_H */
