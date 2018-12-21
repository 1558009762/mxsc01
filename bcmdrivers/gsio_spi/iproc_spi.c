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
#include <linux/string.h>

#include <plat/types.h>

#include <mach/io_map.h>

#include <linux/io.h>
#include <asm/memory.h>

#include "iproc_gsio.h"

static int spi_init = 0;

static void * baseAddr;

#define R_REG(reg)       ioread32(baseAddr + (reg&0xffff))
#define W_REG(reg, val)        iowrite32(val, baseAddr + (reg&0xffff))

#define GSIO_OPCODE   		 0x4
#define GSIO_CORE_INSERT_CSB 0
#define GSIO_ENDIANESS       0
#define GSIO_START           1

#define SPI_ERR_VAL	0x0001
#define SPI_MSG_VAL	0x0002
#define SPI_DBG_VAL	0x0004
//static u32 spi_msg_level = SPI_ERR_VAL;

#if defined(BCMDBG) || defined(BCMDBG_ERR)
#define SPI_ERR(args)	do {if (spi_msg_level & SPI_ERR_VAL) printk args;} while (0)
#else
#define SPI_ERR(args)
#endif

#ifdef BCMDBG
#define SPI_MSG(args)	do {if (spi_msg_level & SPI_MSG_VAL) printk args;} while (0)
#define SPI_DBG(args)	do {if (spi_msg_level & SPI_DBG_VAL) printk args;} while (0)
#else
#define SPI_MSG(args)
#define SPI_DBG(args)
#endif

#define SPI_INIT_CHK  \
    if (!spi_init) {\
         SPI_MSG(("%s,SPI device not init yet!\n", __func__)); \
         return SPI_ERR_INTERNAL; \
    }

#define QT_GSIO_VERIFICATION 0

#if QT_GSIO_VERIFICATION
#define SPI_TRIES 10000
#else
#define SPI_TRIES 100000
#endif

#define CC_ID_MASK		0xf
#define CC_ID_SHIFT	    20
#define CC_CHIPID_MASK		0xffff
#define CC_CHIPID_SHIFT	    0
#define PKG_HIGH_SKU 0
#define PKG_MEDIUM_SKU 2
#define PKG_LOW_SKU 1

static uint32_t apb_clk = 0;

/* Function : cca_spi_cs_release
 *  - Release CS signal : 
 */
int
cca_spi_cs_release(void)
{
    SPI_DBG(("cca_spi_cs_release\n"));

    SPI_INIT_CHK;

    W_REG(GSIO_CTRL, 0);

    return SPI_ERR_NONE;
}

/* Function : cca_spi_read
 *  - Read operation.
 * Return :
 * Note : 
 */
int
cca_spi_read(uint8_t *buf, int len)
{
    int i;
    uint32_t value;
	uint32_t ctrl = 0;
	uint32_t nod = 0;
	int buf_len = len;
    uint quot = 0, remain = 0;

    SPI_DBG(("cca_spi_read: start\n"));

    SPI_INIT_CHK;

    while (buf_len > 0) {
        /* NSTAR-547, SW WAR for set BigEndian bit here */
    	ctrl = ((4 << GSIO_CTRL_GC_SHIFT) & GSIO_CTRL_GC_MASK) |
	        ((0 << GSIO_CTRL_ENDIAN_SHIFT) & GSIO_CTRL_ENDIAN_MASK) |
	        ((1 << GSIO_CTRL_GG_SHIFT) & GSIO_CTRL_GG_MASK) |
    	    ((1 << GSIO_CTRL_SB_SHIFT) & GSIO_CTRL_SB_MASK); 

        quot = (buf_len / 4);
        remain = (buf_len % 4);
        if (quot != 0) { /* buf_len >= 4 bytes */
		    nod = 3;
            ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
            SPI_DBG(("cca_spi_read: write %x, %x\n",GSIO_CTRL, ctrl));
            W_REG(GSIO_CTRL, ctrl);

        	for (i = 0; i < SPI_TRIES; i++) {
                ctrl = R_REG(GSIO_CTRL);
        		if (!(ctrl & GSIO_CTRL_SB_MASK)) {
        		    break;
        		}
        	}
        	if (i >= SPI_TRIES) {
        		SPI_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
        		return -1;
        	}

        	value = R_REG(GSIO_DATA);
            SPI_DBG(("cca_spi_read: done: value=%x\n",value));

        	for (i = 0; i < 4; i++) {
        	    *buf = (value >> (8 * i)) & 0xff;
        		buf ++;
        	}

			buf_len -= 4;
        } else { /* buf_len < 4 bytes */
		    nod = remain - 1;
            ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
            SPI_DBG(("cca_spi_read: write %x, %x\n",GSIO_CTRL, ctrl));
            W_REG(GSIO_CTRL, ctrl);

	        for (i = 0; i < SPI_TRIES; i++) {
                ctrl = R_REG(GSIO_CTRL);
		        if (!(ctrl & GSIO_CTRL_SB_MASK)) {
		            break;
		        }
	        }
        	if (i >= SPI_TRIES) {
        		SPI_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
    		    return -1;
	        }

	        value = R_REG(GSIO_DATA);
            SPI_DBG(("cca_spi_read: done: value=%x\n",value));

        	for (i = 0; i < remain; i++) {
	            *buf = (value >> (8 * i)) & 0xff;
		        buf ++;
        	}

			buf_len -= remain;
        }

    }

    return SPI_ERR_NONE;
}

/* Function : cca_spi_write 
 *  - Write operation.
 * Return :
 * Note : 
 *     DO NOT do de-assert CS here, there is still possiblity of shift-in data . 
 */
int
cca_spi_write(uint8_t *buf, int len)
{
    int i = 0;
	int buf_len = len;
	int process_len = 0;
	uint8_t *process_buf = buf;
    uint quot = 0, remain = 0;
	uint32_t data = 0;
	uint32_t ctrl = 0;
	uint32_t nod = 0;

    SPI_DBG(("cca_spi_write: start\n"));

    SPI_INIT_CHK;

    while (buf_len > 0) {
        /* NSTAR-547, SW WAR for set BigEndian bit here */
	    ctrl = ((4 << GSIO_CTRL_GC_SHIFT) & GSIO_CTRL_GC_MASK) |
	        ((1 << GSIO_CTRL_ENDIAN_SHIFT) & GSIO_CTRL_ENDIAN_MASK) |
    	    ((1 << GSIO_CTRL_GG_SHIFT) & GSIO_CTRL_GG_MASK) |
	        ((1 << GSIO_CTRL_SB_SHIFT) & GSIO_CTRL_SB_MASK);
        quot = (buf_len / 4);
        remain = (buf_len % 4);
		data = 0;
        if (quot != 0) { /* buf_len >= 4 bytes */
            process_len = 4;
			data |= (*process_buf);
			process_buf++;
			data |= (*process_buf << 8);
			process_buf++;
			data |= (*process_buf << 16);
			process_buf++;
			data |= (*process_buf << 24);
			process_buf++;

            nod = 3;
        	ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
        } else { /* buf_len < 4 bytes */
            process_len = remain;

			for (i = remain; i > 0; i--) {
    			data |= (*process_buf << (32 - 8*i));
    			process_buf++;
			}

            nod = remain-1;
        	ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
        }

        SPI_DBG(("cca_spi_write: write %x, %x\n",GSIO_DATA, data));
        SPI_DBG(("cca_spi_write: write %x, %x\n",GSIO_CTRL, ctrl));

        W_REG(GSIO_DATA, data);
        W_REG(GSIO_CTRL, ctrl);

    	for (i = 0; i < SPI_TRIES; i++) {
            ctrl = R_REG(GSIO_CTRL);
    		if (!(ctrl & GSIO_CTRL_SB_MASK)) {
    		    break;
    		}
    	}
    	if (i >= SPI_TRIES) {
    		SPI_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
    		return -1;
    	}

		buf_len -= process_len;
    }

    /* DO NOT do de-assert CS here, there is still possiblity of shift-in data */

    return SPI_ERR_NONE;
}

/* Function : cca_spi_freq_set
 *  - Open Northstar CCA SPI device.
 * Return :
 * Note : 
 *     Set the SPI frequency (offset 0xf0 in chipcommonA).
 *     
 */
int
cca_spi_freq_set(int speed_hz)
{
    int rv = SPI_ERR_NONE;
	uint32_t divider = 0;
	uint32_t val;
	int speed_khz;
    
    SPI_INIT_CHK;

	if (speed_hz > 20971520) {
    	SPI_ERR(("\n%s: exceed maximum 20MHz\n", __FUNCTION__));
	    return SPI_ERR_PARAM;
	}

    speed_khz = (speed_hz / 1024);

	divider = ((apb_clk * 1024) / speed_khz);
	val = R_REG(CLKDIV2);
	val &= ~CLKDIV2_GD_MASK;
	val |= ((divider << CLKDIV2_GD_SHIFT) & CLKDIV2_GD_MASK);
	W_REG(CLKDIV2, val);

    SPI_DBG(("bcm5301x_cca_spi set SPI freq(%dHz) done\n", speed_hz));
    return rv;
}

/* Function : cca_spi_init
 *  - Init Northstar CCA SPI.
 * Return :
 * Note : 
 *     Set the default SPI frequency (offset 0xf0 in chipcommonA).
 *     
 */
int
cca_spi_init(void)
{
    int rv = SPI_ERR_NONE;
//	uint32_t divider = 0;
//	uint32_t val;
	uint32_t pkg;
	uint32_t cid;
    
    spi_init = 1;

    /* Get register base address */
	baseAddr = ioremap(IPROC_CCA_REG_BASE, 0x1000);

    /* Get cpu/system clock */
    pkg = R_REG(0x18000000);
    cid = ((pkg >> CC_CHIPID_SHIFT) & CC_CHIPID_MASK);
    pkg = ((pkg >> CC_ID_SHIFT) & CC_ID_MASK);
    /* AXI bus clock = CPU / 2 */
    /* APB clock = AXI bus / 4 */
    if (cid == 0xcf1a) { /* Costar */
        /* CPU 1GHz, AXI 500MHz */
        apb_clk = 125;
    } else {
        if (pkg == PKG_HIGH_SKU) {
            /* CPU 1GHz, AXI 500MHz */
            apb_clk = 125;
        } else {
            /* CPU 800MHz, AXI 400MHz */
            apb_clk = 100;
        }
    }

    SPI_DBG(("bcm5301x_cca_spi module build %s %s. APB Clock @ %dMHz\n", __DATE__, __TIME__, apb_clk));

    /* Set default frequency 2MHz */
    cca_spi_freq_set(2097152);

    return rv;
}

int
cca_spi_exit(void)
{
    spi_init = 1;

    /* Get register base address */
	if (baseAddr) {
	    iounmap(baseAddr);
		baseAddr = NULL;
	}

    spi_init = 0;

    return 0;
}

/* Function : cca_spi_init_codec
 *  - Init Northstar CCA SPI.
 * Return :
 * Note : 
 *     Set the default SPI frequency (offset 0xf0 in chipcommonA).
 *     
 */
int
cca_spi_init_codec(void)
{
    int rv = SPI_ERR_NONE;
//	uint32_t divider = 0;
//	uint32_t val;
	uint32_t pkg;
	uint32_t cid;
    
    spi_init = 1;

    /* Get register base address */
	baseAddr = ioremap(IPROC_CCA_REG_BASE, 0x1000);

    /* Get cpu/system clock */
    pkg = R_REG(0x18000000);
    cid = ((pkg >> CC_CHIPID_SHIFT) & CC_CHIPID_MASK);
    pkg = ((pkg >> CC_ID_SHIFT) & CC_ID_MASK);
    /* AXI bus clock = CPU / 2 */
    /* APB clock = AXI bus / 4 */
    if (cid == 0xcf1a) { /* Costar */
        /* CPU 1GHz, AXI 500MHz */
        apb_clk = 125;
    } else {
        if (pkg == PKG_HIGH_SKU) {
            /* CPU 1GHz, AXI 500MHz */
            apb_clk = 125;
        } else {
            /* CPU 800MHz, AXI 400MHz */
            apb_clk = 100;
        }
    }

    SPI_DBG(("bcm5301x_cca_spi module build %s %s. APB Clock @ %dMHz\n", __DATE__, __TIME__, apb_clk));

    /* Set default frequency 8MHz */
	cca_spi_freq_set(8000000);

    return rv;
}

/* Function : cca_spi_write_codec 
 *  - Write operation.
 * Return :
 * Note : 
 *     DO NOT do de-assert CS here, there is still possiblity of shift-in data . 
 */
int
cca_spi_write_codec(uint8_t *buf, int len)
{
    int i = 0;
	int buf_len = len;
	int process_len = 0;
	uint8_t *process_buf = buf;
    uint quot = 0, remain = 0;
	uint32_t data = 0;
	uint32_t ctrl = 0;
	uint32_t nod = 0;

    SPI_DBG(("cca_spi_write: start\n"));

    SPI_INIT_CHK;

    while (buf_len > 0) {
	    ctrl = ((GSIO_OPCODE << GSIO_CTRL_GC_SHIFT) & GSIO_CTRL_GC_MASK) |
	        ((GSIO_ENDIANESS << GSIO_CTRL_ENDIAN_SHIFT) & GSIO_CTRL_ENDIAN_MASK) |
    	    ((GSIO_CORE_INSERT_CSB<< GSIO_CTRL_GG_SHIFT) & GSIO_CTRL_GG_MASK) |
	        ((GSIO_START << GSIO_CTRL_SB_SHIFT) & GSIO_CTRL_SB_MASK);				
        quot = (buf_len / 4);
        remain = (buf_len % 4);
		data = 0;
        if (quot != 0) { /* buf_len >= 4 bytes */
            process_len = 4;
			data |= (*process_buf);
			process_buf++;
			data |= (*process_buf << 8);
			process_buf++;
			data |= (*process_buf << 16);
			process_buf++;
			data |= (*process_buf << 24);
			process_buf++;

            nod = 3;
        	ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
        } else { /* buf_len < 4 bytes */
            process_len = remain;
			
			for (i = 0; i < remain; i++) {
    			data |= (*process_buf << (8*i));
    			process_buf++;
			}

            nod = remain-1;
        	ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
        }

        SPI_DBG(("cca_spi_write: write %x, %x\n",GSIO_DATA, data));
        SPI_DBG(("cca_spi_write: write %x, %x\n",GSIO_CTRL, ctrl));

        W_REG(GSIO_DATA, data);
        W_REG(GSIO_CTRL, ctrl);

    	for (i = 0; i < SPI_TRIES; i++) {
            ctrl = R_REG(GSIO_CTRL);
    		if (!(ctrl & GSIO_CTRL_SB_MASK)) {
    		    break;
    		}
    	}
    	if (i >= SPI_TRIES) {
    		SPI_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
    		return -1;
    	}

		buf_len -= process_len;
    }

    /* DO NOT do de-assert CS here, there is still possiblity of shift-in data */

    return SPI_ERR_NONE;
}

/* Function : cca_spi_slic
 *  - Writes 8-bit value and subsequently reads 8-bit value
 * Return :
 * Note : 
 *     DO NOT do de-assert CS here, there is still possiblity of shift-in data . 
 */
int
cca_spi_slic_rw(uint8_t *outbuf, uint8_t *inbuf)
{
    int i = 0;
	int len = 1;
	int buf_len = len;
	int process_len = 0;
	uint8_t *process_buf = outbuf;
    uint quot = 0, remain = 0;
	uint32_t data = 0;
	uint32_t ctrl = 0;
	uint32_t nod = 0;
	uint32_t value;

    SPI_DBG(("cca_spi_write: start\n"));

    SPI_INIT_CHK;

    while (buf_len > 0) {
	    ctrl = ((GSIO_OPCODE << GSIO_CTRL_GC_SHIFT) & GSIO_CTRL_GC_MASK) |
	        ((GSIO_ENDIANESS << GSIO_CTRL_ENDIAN_SHIFT) & GSIO_CTRL_ENDIAN_MASK) |
    	    ((GSIO_CORE_INSERT_CSB<< GSIO_CTRL_GG_SHIFT) & GSIO_CTRL_GG_MASK) |
	        ((GSIO_START << GSIO_CTRL_SB_SHIFT) & GSIO_CTRL_SB_MASK);				
        quot = (buf_len / 4);
        remain = (buf_len % 4);
		data = 0;
        if (quot != 0) { /* buf_len >= 4 bytes */
            process_len = 4;
			data |= (*process_buf);
			process_buf++;
			data |= (*process_buf << 8);
			process_buf++;
			data |= (*process_buf << 16);
			process_buf++;
			data |= (*process_buf << 24);
			process_buf++;

            nod = 3;
        	ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
        } else { /* buf_len < 4 bytes */
            process_len = remain;
			
			for (i = 0; i < remain; i++) {
    			data |= (*process_buf << (8*i));
    			process_buf++;
			}

            nod = remain-1;
        	ctrl |= ((nod << GSIO_CTRL_NOD_SHIFT) & GSIO_CTRL_NOD_MASK);
        }

        SPI_DBG(("cca_spi_write: write %x, %x\n",GSIO_DATA, data));
        SPI_DBG(("cca_spi_write: write %x, %x\n",GSIO_CTRL, ctrl));

        W_REG(GSIO_DATA, data);
        W_REG(GSIO_CTRL, ctrl);

    	for (i = 0; i < SPI_TRIES; i++) {
            ctrl = R_REG(GSIO_CTRL);
    		if (!(ctrl & GSIO_CTRL_SB_MASK)) {
    		    break;
    		}
    	}
    	if (i >= SPI_TRIES) {
    		SPI_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
    		return -1;
    	}

		buf_len -= process_len;
    }
	
	// Read the data
    value = R_REG(GSIO_DATA);
	*inbuf = (uint8_t)value;
	//DBG("%x\n", value);

    /* DO NOT do de-assert CS here, there is still possiblity of shift-in data */

    return SPI_ERR_NONE;
}

EXPORT_SYMBOL(cca_spi_cs_release);
EXPORT_SYMBOL(cca_spi_read);
EXPORT_SYMBOL(cca_spi_write);
EXPORT_SYMBOL(cca_spi_freq_set);
EXPORT_SYMBOL(cca_spi_init);
EXPORT_SYMBOL(cca_spi_exit);
EXPORT_SYMBOL(cca_spi_init_codec);
EXPORT_SYMBOL(cca_spi_write_codec);
EXPORT_SYMBOL(cca_spi_slic_rw);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM5301X SPI Device Driver");
MODULE_LICENSE("GPL");
