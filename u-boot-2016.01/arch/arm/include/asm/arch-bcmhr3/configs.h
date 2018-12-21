/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __ARCH_CONFIGS_H
#define __ARCH_CONFIGS_H

#include <asm/iproc-common/configs.h>

/* uArchitecture specifics */

/* Serial Info */
/* Post pad 3 bytes after each reg addr */
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_MEM32

#define CONFIG_SYS_NS16550_CLK		iproc_get_uart_clk(0)
#define CONFIG_CONS_INDEX           1
#define CONFIG_SYS_NS16550_COM1     0x18020000

/* Ethernet */
#define CONFIG_BCM_XGS_ETH
#define CONFIG_BCM_XGS_ETH_GMAC
#define CONFIG_GMAC_NUM				1
#define CONFIG_ETHPRIME				"bcm_xgs_gmac-0"
#if (defined(CONFIG_IPROC_NO_DDR) && defined(CONFIG_L2C_AS_RAM))
#define IPROC_ETH_MALLOC_BASE		0x50010000
#else
#define IPROC_ETH_MALLOC_BASE		0x00D00000
#endif /* (defined(CONFIG_IPROC_NO_DDR) && defined(CONFIG_L2C_AS_RAM)) */

#ifdef CONFIG_HURRICANE3_LITE
#define CONFIG_EXTERNAL_PHY_BUS_ID   0x1
#define CONFIG_EXTERNAL_PHY_DEV_ID   0x18
#else
#define CONFIG_EXTERNAL_PHY_BUS_ID   0x2
#define CONFIG_EXTERNAL_PHY_DEV_ID   0x18
#endif /* CONFIG_HURRICANE3_LITE */

#define CONFIG_PHYLIB
#define CONFIG_PHY_BROADCOM
#define CONFIG_PHY_RESET_DELAY 10000 /* PHY reset delay in us*/

#define CONFIG_CMD_PING
#define CONFIG_CMD_MII

#endif /* __ARCH_CONFIGS_H */
