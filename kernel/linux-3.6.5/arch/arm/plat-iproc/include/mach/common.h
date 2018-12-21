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

/*
 *
 * Core functions for Broadcom Northstar SoC Chip
 */

#ifndef __ARCH_IPROC_COMMON_H
#define __ARCH_IPROC_COMMON_H

struct mv643xx_eth_platform_data;
struct mv_sata_platform_data;

extern struct sys_timer IPROC_timer;
extern struct mbus_dram_target_info iproc_mbus_dram_info;

/*
 * Basic IPROC init functions used early by machine-setup.
 */
void IPROC_map_io(void);
void IPROC_init(void);
void IPROC_init_irq(void);
void IPROC_setup_cpu_mbus(void);
void IPROC_ge00_init(struct mv643xx_eth_platform_data *eth_data);
void IPROC_sata_init(struct mv_sata_platform_data *sata_data);
void IPROC_pcie_init(int init_port0, int init_port1);
void IPROC_ehci0_init(void);
void IPROC_ehci1_init(void);
void IPROC_uart0_init(void);
void IPROC_uart1_init(void);
void IPROC_uart2_init(void);
void IPROC_uart3_init(void);
void IPROC_spi0_init(void);
void IPROC_spi1_init(void);
void IPROC_i2c_init(void);

#endif
