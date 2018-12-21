/*
 * smc35x_lcd_io.c
 *
 * Copyright 2008 - 2014 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2, available at
 * http://www.broadcom.com/licenses/GPLv2.php (the "GPL").
 *
 * Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a
 * license other than the GPL, without Broadcom's express prior written
 * consent.
 *
 *
 * This code is only meant to show how to set up the SRAM controller, and select the
 * proper I/O muxing and etc....  While it is presented in a driver form, it is not
 * necessarily how customers will use it.
 *
 */

#include "smc35x_lcd.h"		/* SMC register definition */
#include "smc35x_lcd_io.h"	/* SMC register definition */

/*
 *  Module Parameters - Set default 
 *       8b data Bus at 8b
 *       240 x320 LCD 
 */

static int smc35x_lcd_data_bus_width = 16;	/* LCD Bus Width */

static int smc35x_lcd_width = 240;	/* LCD Width */

static int smc35x_lcd_height = 320;	/* LCD Height */

static int smc35x_lcd_dw = 8;	/* 8b data port */

/*
 *  Function Prototypes
 */
static smc35x_lcd_ctx_t *drvctx;

/*
 * CMD and Data
 */
static inline void
smc35x_lcd_wrcmd8(uint8_t d)
{
	*(volatile uint8_t *) drvctx->cmd_addr = d;
}

static inline void
smc35x_lcd_wrcmd16(uint16_t d)
{
	*(volatile uint16_t *) drvctx->cmd_addr = d;
}

static inline void
smc35x_lcd_wrdata8(uint8_t d)
{
	*(volatile uint8_t *) drvctx->data_addr = d;
}

static inline void
smc35x_lcd_wrdata16(uint16_t d)
{
	*(volatile uint16_t *) drvctx->data_addr = d;
}

static inline void
smc35x_lcd_rddata8(uint8_t * d)
{
	*d = (uint8_t) * ((volatile uint8_t *) drvctx->data_addr);
}

static inline void
smc35x_lcd_rddata16(uint16_t * d)
{
	*d = (uint16_t) * ((volatile uint16_t *) drvctx->data_addr);
}


ssize_t
smc35x_lcd_io_write(const char *buf, size_t len)
{
	uint32_t i, n;
	uint8_t *d8;
	uint16_t *d16;
	n = ((drvctx->maxsz > len) ? len : drvctx->maxsz);
	if (smc35x_lcd_data_bus_width <= 8) {
		d8 = drvctx->dbuf;
		smc35x_lcd_wrcmd8(*d8++);
		i++;

		while (i < n) {
			smc35x_lcd_wrdata8(*d8++);
			i++;
		}
	} else {
		d16 = (uint16_t *) drvctx->dbuf;
		smc35x_lcd_wrcmd16(*d16++);
		i += 2;

		while (i < n) {
			smc35x_lcd_wrdata16(*d16++);
			i += 2;
		}
	}
	return len;
}

EXPORT_SYMBOL(smc35x_lcd_io_write);

/*
 * Read data from LCD
 */

ssize_t
smc35x_lcd_io_read(char *buf, size_t len)
{
	uint8_t *d8;
	uint16_t *d16;
	uint8_t wrcmd8 = 0x2e;	// read GRAM -- 8b data bus
	uint16_t wrcmd16 = 0x002e;	// read GRAM -- 16b data bus
	int i = 0;
	int maxsz = drvctx->maxsz;
	//n = ((drvctx->maxsz > len) ? len : drvctx->maxsz);
	if (len < maxsz - 1) {
		if (smc35x_lcd_data_bus_width <= 8) {
			d8 = drvctx->dbuf;
			smc35x_lcd_wrcmd8(wrcmd8);	// output a command
			while (i < len) {
				smc35x_lcd_rddata8(d8++);
				i++;
			}
		} else {
			d16 = (uint16_t *) drvctx->dbuf;
			smc35x_lcd_wrcmd16(wrcmd16);	// output a command

			while (i < len) {
				smc35x_lcd_rddata16(d16++);
				i += 2;
			}
		}

	}
	return len;
}

EXPORT_SYMBOL(smc35x_lcd_io_read);

/*
 * Initialize the SMC controller -
 * 
 * I/O map system control,  SMC control and memory space for LCD
 * While we allocate 64MB in [E8000000, EBFF_FFFF] for SRAM, we only map 4KB because
 * it is all we need.
 *
 * Set the timing parameter of SMC
 */

static int
smc35x_lcd_probe(void)
{
	uint32_t v;

	if ((drvctx->sys_base =
	     ioremap_nocache(SMC35x_SYS_CFG_BASE, 4096)) == NULL)
		return (-1);	// map 4K page

	if ((drvctx->smc_base =
	     ioremap_nocache(SMC35x_CTRL_BASE, 4096)) == NULL)
		return (-1);	// map 4K page

	if ((drvctx->mem_base =
	     ioremap_nocache(SMC35x_MM_BASE, 4096)) == NULL)
		return (-1);	// map 4K page

	/* if we have 8b memory width or 16b memory width */
	if (smc35x_lcd_data_bus_width == 8) {
		drvctx->cmd_addr = drvctx->mem_base + SMC35x_CMD_ADDR;	// 0 - addr[0]
		drvctx->data_addr = drvctx->mem_base + SMC35x_DATA_ADDR;	// 1 - addr[0]
	} else {		// 16b memory width
		drvctx->cmd_addr = drvctx->mem_base + SMC35x_CMD_ADDR;	// 0 - addr[1]
		drvctx->data_addr = drvctx->mem_base + SMC35x_DATA_ADDR + 1;	// 2 - addr[1]
	}

	if ((drvctx->iomux_base =
	     ioremap_nocache(CRMU_IOMUX_BASE, 4096)) == NULL)
		return (-1);	// map 4K page

	/* 
	 * IO muxing, select the SRAM 
	 * Register 0x0301d0d8 shall contain value, such that bit[22:20] has a value of 1 
	 * Register 0x0301
	 */

	// IOMUX_CTRL4
	Rd_Reg32(&v, drvctx->iomux_base + REG_CRMU_IOMUX_CTRL4);
	v = (v & 0xFF8FFFFF) | 0x00100000;
	Wr_Reg32(v, drvctx->iomux_base + REG_CRMU_IOMUX_CTRL4);

	// IO selection only for data bus > 8
	if (smc35x_lcd_data_bus_width > 8) {
		// IOMUX_CTRL7
		Rd_Reg32(&v, drvctx->iomux_base + REG_CRMU_IOMUX_CTRL7);
		v = (v & 0xFFFFF8FF) | 0x00000100;
		Wr_Reg32(v, drvctx->iomux_base + REG_CRMU_IOMUX_CTRL7);
	}

	/* 
	 * Set up access region for SRAM CS0#
	 * Use default chip setting
	 * Register 0x18000a50 shall contain value 0x0000FEE8
	 * Register 0x18000a54 shall contain value 0x0000FEEA
	 * Register 0x18000a58 shall contain value 0x0000FCEC 
	 */
	//Wr_Reg32(SMC35x_LCD_SPACE0, drvctx->sys_base+SMC35x_CFG_CS0);
	//Wr_Reg32(0, drvctx->sys_base+SMC35x_CFG_CS1);
	//Wr_Reg32(0, drvctx->sys_base+SMC35x_CFG_CS2);

	/*
	 * Set timing parameters 
	 * Register 0x18045014 shall contain value 0x00025143 afterwards
	 * Register 0x18045018 shall contain value 0x0 afterwards
	 *
	 * Set direct command register to activate
	 */

	v = tRC | (tWC << 4) | (tCEOE << 8) | (tWP << 11) |
	    (tPC << 14) | (tTR << 17) | (tWE << 20);

	Wr_Reg32(v, drvctx->smc_base + SMC35x_SET_CYCLE);

	/* if we have 8b memory width or 16b memory width */
	if (smc35x_lcd_data_bus_width == 8)
		Wr_Reg32(SMC35x_opmode, drvctx->smc_base + SMC35x_SET_OPMODE);	// 8b width, mw=2'b00
	else
		Wr_Reg32(SMC35x_opmode + 1, drvctx->smc_base + SMC35x_SET_OPMODE);	//16b width, mw=2'b01

	// Update Reg
	Wr_Reg32(SMC35x_UpdateRegs, drvctx->smc_base + SMC35x_DIRECT_CMD);


	/* 
	 * Additional code for setting drive strength
	 */

	/* 
	 */
	return (0);
}

/*
 * main initialization and cleanup section
 */

void
smc35x_lcd_io_cleanup(void)
{


	if (drvctx->dbuf)
		kfree(drvctx->dbuf);

	if (drvctx->sys_base)
		iounmap(drvctx->sys_base);

	if (drvctx->smc_base)
		iounmap(drvctx->smc_base);

	if (drvctx->mem_base)
		iounmap(drvctx->mem_base);

	if (drvctx->iomux_base)
		iounmap(drvctx->iomux_base);

	if (drvctx)
		kfree(drvctx);
}

EXPORT_SYMBOL(smc35x_lcd_io_cleanup);

int
smc35x_lcd_io_init(void)
{
	int ret = 0;
	int maxsz;


	/*
	 * Allocating driver context
	 */
	drvctx = kzalloc(sizeof (*drvctx), GFP_KERNEL);
	if (!drvctx) {
		printk("smc35x_data_init failed\n");
		goto exit;
	}

	/* 
	 * Allocate data buffer.  Note - This is an example, over-simplified code. 
	 * The real code needs to adjust the data buf, accounting the command field, 
	 * and bit-per-pixel (color mode), accordingly to the data bus width.  
	 */

	if (smc35x_lcd_data_bus_width <= 8)
		maxsz = smc35x_lcd_width;	// LCD data bus width is 8b
	else
		maxsz = smc35x_lcd_width * 2;	// LCD data bus width is >8b

	drvctx->maxsz = maxsz;
	drvctx->dbuf = kzalloc(maxsz, GFP_KERNEL | __GFP_ZERO);

	if (!drvctx->dbuf) {
		ret = -ENOMEM;
		goto exit;
	}

	/*
	 * Map memory for the SMC controller
	 */
	ret = smc35x_lcd_probe();

	/*
	 */
      exit:

	if (ret < 0) {
		_kdb_trace
		    ("inside lcd_cleanup - func<%s>\n,line<%d>\n",
		     __func__, __LINE__);
		smc35x_lcd_io_cleanup();
	}

	_kdb_trace("func<%s>\n,line<%d>\n", __func__, __LINE__);
	return ret;
}

EXPORT_SYMBOL(smc35x_lcd_io_init);
