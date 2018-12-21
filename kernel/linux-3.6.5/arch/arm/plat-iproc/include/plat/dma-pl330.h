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
#ifndef __PLAT_DMA_H
#define __PLAT_DMA_H

#include <asm/scatterlist.h>

#define MAX_CHAN_NAME_LENGTH	32

/* DMA direction control */
enum dma_direction {
	DMA_DIRECTION_MEM_TO_MEM = 0,
	DMA_DIRECTION_MEM_TO_DEV_FLOW_CTRL_DMAC = 1,
	DMA_DIRECTION_MEM_TO_DEV_FLOW_CTRL_PERI = 2,
	DMA_DIRECTION_DEV_TO_MEM_FLOW_CTRL_DMAC = 3,
	DMA_DIRECTION_DEV_TO_MEM_FLOW_CTRL_PERI = 4,
	DMA_DIRECTION_DEV_TO_DEV = 5	/* Invalid, unsupported */
};
#define DMA_DIRECTION_MASK	0x7

/* Channel configurations definition */
#define DMA_CFG_SRC_ADDR_FIXED				(0x0 << 0)
#define DMA_CFG_SRC_ADDR_INCREMENT			(0x1 << 0)
#define DMA_CFG_DST_ADDR_FIXED				(0x0 << 14)
#define DMA_CFG_DST_ADDR_INCREMENT			(0x1 << 14)

#define DMA_CFG_BURST_SIZE_MASK         (0x7 << 1)
#define DMA_CFG_BURST_SIZE_1            (0x0 << 1)
#define DMA_CFG_BURST_SIZE_2            (0x1 << 1)
#define DMA_CFG_BURST_SIZE_4            (0x2 << 1)
#define DMA_CFG_BURST_SIZE_8            (0x3 << 1)
#define DMA_CFG_BURST_SIZE_16           (0x4 << 1)
#define DMA_CFG_BURST_SIZE_32           (0x5 << 1)
#define DMA_CFG_BURST_SIZE_64           (0x6 << 1)
#define DMA_CFG_BURST_SIZE_128          (0x7 << 1)

#define DMA_CFG_BURST_LENGTH_MASK       (0xF << 4)
#define DMA_CFG_BURST_LENGTH_1           (0x0 << 4)
#define DMA_CFG_BURST_LENGTH_2           (0x1 << 4)
#define DMA_CFG_BURST_LENGTH_3           (0x2 << 4)
#define DMA_CFG_BURST_LENGTH_4           (0x3 << 4)
#define DMA_CFG_BURST_LENGTH_5           (0x4 << 4)
#define DMA_CFG_BURST_LENGTH_6           (0x5 << 4)
#define DMA_CFG_BURST_LENGTH_7           (0x6 << 4)
#define DMA_CFG_BURST_LENGTH_8           (0x7 << 4)
#define DMA_CFG_BURST_LENGTH_9           (0x8 << 4)
#define DMA_CFG_BURST_LENGTH_10          (0x9 << 4)
#define DMA_CFG_BURST_LENGTH_11          (0xA << 4)
#define DMA_CFG_BURST_LENGTH_12          (0xB << 4)
#define DMA_CFG_BURST_LENGTH_13          (0xC << 4)
#define DMA_CFG_BURST_LENGTH_14          (0xD << 4)
#define DMA_CFG_BURST_LENGTH_15          (0xE << 4)
#define DMA_CFG_BURST_LENGTH_16          (0xF << 4)

#define DMA_CFG_BURST_LEN(x)			(((x - 1) & 0xF) << 4)

/* src and dest burst size and burst length are assumed to be same */

enum pl330_xfer_status {
	DMA_PL330_XFER_OK,
	DMA_PL330_XFER_ERR,
	DMA_PL330_XFER_ABORT,
};

struct dma_transfer_list {
	dma_addr_t srcaddr;	/* src address */
	dma_addr_t dstaddr;	/* dst address */
	unsigned int xfer_size;	/* In bytes */
	struct list_head next;	/* Next item */
};

typedef void (*pl330_xfer_callback_t) (void *private_data,
				       enum pl330_xfer_status status);

int dma_request_chan(unsigned int *chan, const char *name);
int dma_free_chan(unsigned int chan);
int dma_map_peripheral(unsigned int chan, const char *peri_name);
int dma_unmap_peripheral(unsigned int chan);
int dma_setup_transfer(unsigned int chan, dma_addr_t s, dma_addr_t d,
		       unsigned int xfer_size, int ctrl, int cfg);
int dma_setup_transfer_list(unsigned int chan, struct list_head *head,
			    int ctrl, int cfg);
int dma_start_transfer(unsigned int chan);
int dma_stop_transfer(unsigned int chan);
int dma_shutdown_all_chan(void);
int dma_register_callback(unsigned int chan,
			  pl330_xfer_callback_t cb, void *pri);
int dma_free_callback(unsigned int chan);

#endif /* __PLAT_DMA_H */
