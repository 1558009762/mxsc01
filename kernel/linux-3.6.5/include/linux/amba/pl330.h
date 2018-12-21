/* linux/include/linux/amba/pl330.h
 *
 * Copyright (C) 2010 Samsung Electronics Co. Ltd.
 *	Jaswinder Singh <jassi.brar@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef	__AMBA_PL330_H_
#define	__AMBA_PL330_H_

#include <linux/dmaengine.h>

struct dma_pl330_platdata {
	/*
	 * Number of valid peripherals connected to DMAC.
	 * This may be different from the value read from
	 * CR0, as the PL330 implementation might have 'holes'
	 * in the peri list or the peri could also be reached
	 * from another DMAC which the platform prefers.
	 */
	u8 nr_valid_peri;
	/* Array of valid peripherals */
	u8 *peri_id;
	/* Operational capabilities */
	dma_cap_mask_t cap_mask;
	/* Bytes to allocate for MC buffer */
	unsigned mcbuf_sz;
};

enum dma_ch {
	DMACH_DT_PROP = -1,
	DMACH_CRYPTO_IN = 0,
	DMACH_CRYPTO_OUT,
	DMACH_SPI0_RX,
	DMACH_SPI0_TX,
	DMACH_SPI1_RX,
	DMACH_SPI1_TX,
	DMACH_SPI2_RX,
	DMACH_SPI2_TX,
	DMACH_SPI3_RX,
	DMACH_SPI3_TX,
	DMACH_SPI4_RX,
	DMACH_SPI4_TX,
	DMACH_SPI5_RX,
	DMACH_SPI5_TX,
	DMACH_UART0_RX = 14,
	DMACH_UART0_TX,
	DMACH_UART1_RX,
	DMACH_UART1_TX,
	DMACH_UART2_RX,
	DMACH_UART2_TX,
	DMACH_UART3_RX,
	DMACH_UART3_TX,
	DMACH_UART4_RX,
	DMACH_UART4_TX,
	DMACH_MIPI = 24,
	DMACH_MTOM_0,
	DMACH_MTOM_1,
	DMACH_MTOM_2,
	DMACH_MTOM_3,
	DMACH_MTOM_4,
	DMACH_MTOM_5,
	DMACH_MTOM_6,
	/* END Marker, also used to denote a reserved channel */
	DMACH_MAX,
};

extern bool pl330_filter(struct dma_chan *chan, void *param);
#endif	/* __AMBA_PL330_H_ */
