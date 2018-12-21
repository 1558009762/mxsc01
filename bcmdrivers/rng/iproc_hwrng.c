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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <mach/brcm_rdb_rng.h>
#include <mach/io_map.h>
#include "iproc_hwrng.h"

static void * baseAddr;

static void ioclrbit32(void *addr, unsigned int bits)
{
    iowrite32(ioread32(addr) & ~bits, addr);
}

static void iosetbit32(void *addr, unsigned int bits)
{
    iowrite32(ioread32(addr) | bits,  addr);
}

/**
*  @brief  set warmup cycle
*  @param  none
*  @return none
*  @note
*****************************************************************************/
static inline void chal_rng_inline_warmup(uint32_t cycles)
{
    iowrite32(RNG_STATUS_RNG_WARM_CNT_MASK - (cycles & RNG_STATUS_RNG_WARM_CNT_MASK), baseAddr + RNG_STATUS_OFFSET);
    while ( ( ioread32(baseAddr + RNG_STATUS_OFFSET) & RNG_STATUS_RNG_WARM_CNT_MASK ) != RNG_STATUS_RNG_WARM_CNT_MASK );
}

/**
*  @brief  Stop RNG block
*  @param  none
*  @return none
*  @note
*****************************************************************************/
void bcm5301x_rng_enable(void)
{
     iosetbit32(baseAddr + RNG_CTRL_OFFSET, RNG_CTRL_RNG_RBGEN_MASK);
}
/**
*  @brief  Stop RNG block
*  @param  none
*  @return none
*  @note
*****************************************************************************/
void bcm5301x_rng_disable(void)
{
    ioclrbit32(baseAddr + RNG_CTRL_OFFSET, RNG_CTRL_RNG_RBGEN_MASK);
}
/**
*  @brief  Enable/Disable RNG RBG2X
*  @param  enable   (in) 1 to enable, 0 to disable
*  @return none
*  @note
*****************************************************************************/
static inline void chal_rng_inline_rbg2x_enable(void)
{
        iosetbit32(baseAddr + RNG_CTRL_OFFSET, RNG_CTRL_RNG_RBG2X_MASK);
}
static inline void chal_rng_inline_rbg2x_disable(void)
{
        ioclrbit32(baseAddr + RNG_CTRL_OFFSET, RNG_CTRL_RNG_RBG2X_MASK);
}
/**
*  @brief  Configure RNG FF THRESH
*  @param  ff_thresh   (in) ff threshold
*  @return none
*  @note
*****************************************************************************/
static inline void chal_rng_inline_ff_thresh(uint8_t ff_thresh)
{
        uint32_t val = ioread32(baseAddr + RNG_FF_THRES_OFFSET);
        val &= ~RNG_FF_THRES_RNG_FF_THRESH_MASK;
        iowrite32(val | ff_thresh, baseAddr + RNG_FF_THRES_OFFSET);
}
/**
*  @brief  Enable/Disable RNG INTERRUPT
*  @param  enable   (in) 1 to enable, 0 to disable
*  @return none
*  @note
*****************************************************************************/
void bcm5301x_rng_int_enable(void)
{
        ioclrbit32(baseAddr + RNG_INT_MASK_OFFSET, RNG_INT_MASK_RNG_INT_OFF_MASK);
}
void bcm5301x_rng_int_disable(void)
{
        iosetbit32(baseAddr + RNG_INT_MASK_OFFSET, RNG_INT_MASK_RNG_INT_OFF_MASK);
}

/**
*  @brief  RNG get number of valid words available
*  @return Number of words available
*  @note
*****************************************************************************/
uint32_t bcm5301x_rng_get_valid_words(void)
{
    return ((ioread32(baseAddr + RNG_STATUS_OFFSET) & RNG_STATUS_RND_VAL_MASK) >> RNG_STATUS_RND_VAL_SHIFT);
}

/**
*  @brief  RNG get value
*  @param  pBuffer (in) Buffer to read data into
*  @param  len (in) bytes of the data to be read
*  @return Number of bytes actually read
*  @note
*****************************************************************************/
uint32_t bcm5301x_rng_get_random_number( void )
{
   while ( bcm5301x_rng_get_valid_words() == 0 );

   return ioread32(baseAddr + RNG_DATA_OFFSET);
}

/**
 *  @brief  Start RNG block
 *  @param  none
 *  @return none
 *  @note
 *****************************************************************************/
void bcm5301x_rng_start(void)
{
	baseAddr = ioremap(IPROC_CCB_RNG_REG_BASE, 0x1000);
	
    bcm5301x_rng_enable();
    chal_rng_inline_rbg2x_enable();
    bcm5301x_rng_int_disable();
    chal_rng_inline_warmup(0xfff);
}

void bcm5301x_rng_exit(void)
{
    bcm5301x_rng_disable();
    chal_rng_inline_rbg2x_disable();

    if (baseAddr) {
	    iounmap(baseAddr);
		baseAddr = NULL;
    }
	
}

EXPORT_SYMBOL(bcm5301x_rng_get_random_number);
EXPORT_SYMBOL(bcm5301x_rng_start);
EXPORT_SYMBOL(bcm5301x_rng_exit);
EXPORT_SYMBOL(bcm5301x_rng_get_valid_words);
EXPORT_SYMBOL(bcm5301x_rng_disable);
EXPORT_SYMBOL(bcm5301x_rng_enable);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM5301X RNG Device Driver");
MODULE_LICENSE("GPL");
