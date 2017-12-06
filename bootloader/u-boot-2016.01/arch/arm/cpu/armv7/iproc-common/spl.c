/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <config.h>
#include <spl.h>
#include <image.h>
#include <linux/compiler.h>

#ifndef CONFIG_SPL_DM
/* Pointer to as well as the global data structure for SPL */
DECLARE_GLOBAL_DATA_PTR;
#endif

/*
 * In the context of SPL, board_init_f must ensure that any clocks/etc for
 * DDR are enabled, ensure that the stack pointer is valid, clear the BSS
 * and call board_init_r.
 */
void board_init_f(ulong dummy)
{
	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

#ifndef CONFIG_SPL_DM
	/* TODO: Remove settings of the global data pointer here */
	gd = &gdata;
    memset((void *)gd, 0, sizeof(gd_t));
#endif
	/* Console initialization */
	preloader_console_init();
	board_early_init_f();
	timer_init();
	dram_init();
	board_init_r(NULL, 0);
}

uint32_t spl_boot_device(void)
{
#if defined(CONFIG_SPL_NAND_SUPPORT)
	return BOOT_DEVICE_NAND;
#elif defined(CONFIG_SPL_SPI_FLASH_SUPPORT)
	return BOOT_DEVICE_SPI;
#else
	puts("Unknown boot device\n");
	hang();
	return 0;
#endif
}
