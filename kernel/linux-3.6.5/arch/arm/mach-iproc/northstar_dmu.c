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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/clkdev.h>

#include <asm/io.h>
#include <asm/pgtable.h>

#include <mach/clkdev.h>
#include <mach/io_map.h>
#include <mach/iproc_regs.h>
#include <mach/hardware.h>

#define IPROC_DMU_BASE_PA	IPROC_DMU_BASE_REG
//#define IPROC_DMU_BASE_VA	HW_IO_PHYS_TO_VIRT(IPROC_DMU_BASE_PA)

static struct resource dmu_regs = {
	.name = "dmu_regs",
	.start = (resource_size_t) IOMEM(IPROC_DMU_BASE_VA),
	.end = (resource_size_t) (IOMEM(IPROC_DMU_BASE_VA) + SZ_4K - 1),
    .flags = IORESOURCE_MEM,
};

/*
 * Clock management scheme is a provisional implementation
 * only intended to retreive the pre-set frequencies for each
 * of the clocks.
 * Better handling of post-dividers and fractional part of
 * feedbeck dividers need to be added.
 * Need to understand what diagnostics from CRU registers could
 * be handy, and export that via a sysfs interface.
 */

/* 
 * The CRU contains two similar PLLs: LCPLL and GENPLL,
 * both with several output channels divided from the PLL
 * output
 */

/*
 * Get PLL running status and update output frequency
 */
static int lcpll_status(struct clk * clk)
{
	u32 reg;
	u64 x;
	unsigned pdiv, ndiv_int, ndiv_frac;

	if (clk->type != CLK_PLL)
		return -EINVAL;

	/* read status register */
	reg = readl(clk->regs_base + 0x10);

	/* bit 12 is "lock" signal, has to be "1" for proper PLL operation */
	if ((reg & (1 << 12)) == 0) {
		clk->rate = 0;
	}

	/* Update PLL frequency */

	/* control1 register */
	reg = readl(clk->regs_base + 0x04);

	/* feedback divider integer and fraction parts */
	pdiv = (reg >> 28) & 7 ;
	ndiv_int = (reg >> 20) & 0xff;
	ndiv_frac = reg & ((1<<20)-1);

	if (pdiv == 0)
		return -EIO;

	x = clk->parent->rate / pdiv ;

	x = x * ((u64) ndiv_int << 20 | ndiv_frac) ;

	clk->rate = x >> 20 ;

	return 0;
}

static const struct clk_ops lcpll_ops = {
	.status = lcpll_status,
};

static int lcpll_chan_status(struct clk * clk)
{
	void * __iomem base;
	u32 reg;
	unsigned enable;
	unsigned mdiv;

	if (clk->parent == NULL || clk->type != CLK_DIV)
		return -EINVAL;

	/* Register address is only stored in PLL structure */
	base = clk->parent->regs_base;
	BUG_ON(base == NULL);

	/* enable bit is in enableb_ch[] inversed */
	enable = ((readl(base + 0) >> 6) & 7) ^ 7;

	if (0 == (enable & (1 << clk->chan))) {
		clk->rate = 0;
		return -EIO;
	}

	/* get divider */
	reg = readl(base + 0x08);

	mdiv = 0xff & (reg >> ((0x3^clk->chan) << 3));

	/* when divisor is 0, it behaves as max+1 */
	if (mdiv == 0)
		mdiv = 1 << 8;

	printk("LCPLL[%d] mdiv=%u rate=%lu\n", clk->chan, mdiv, clk->parent->rate);

	clk->rate = (clk->parent->rate / mdiv);
	return 0;
}


static const struct clk_ops lcpll_chan_ops = {
	.status = lcpll_chan_status,
};

/*
 * LCPLL has 4 output channels
 */
static struct clk clk_lcpll = {
	.ops 	= &lcpll_ops,
	.name 	= "LCPLL",
	.type	= CLK_PLL,
	.chan	=	4,
};

/*
 * LCPLL output clocks -
 * chan 0 - PCIe ref clock, should be 1 GHz,
 * chan 1 - SDIO clock, e.g. 200 MHz,
 * chan 2 - DDR clock, typical 166.667 MHz for DDR667,
 * chan 3 - Unknown
 */

static struct clk clk_lcpll_ch[4] = {
	{
		.ops	= &lcpll_chan_ops,
		.parent = &clk_lcpll,
		.type = CLK_DIV,
	  	.name	= "lcpll_ch0",
		.chan	= 0,
	},
	{
		.ops	= &lcpll_chan_ops,
		.parent = &clk_lcpll,
		.type = CLK_DIV,
	  	.name	= "lcpll_ch1",
		.chan	= 1,
	},
	{
		.ops	= &lcpll_chan_ops,
		.parent = &clk_lcpll,
		.type = CLK_DIV,
	  	.name	= "lcpll_ch2",
		.chan	= 2,
	},
	{
		.ops	= &lcpll_chan_ops,
		.parent = &clk_lcpll,
		.type = CLK_DIV,
	  	.name	= "lcpll_ch3",
		.chan	= 3,
	},
};

/*
 * Get PLL running status and update output frequency
 */
#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_HR2) || defined(CONFIG_MACH_GH) || \
	defined(CONFIG_MACH_HR3)
static int genpll_status(struct clk * clk)
{
	u32 reg;
	u64 x;
	unsigned pdiv;
	unsigned ndiv_int;

	if (clk->type != CLK_PLL)
		return -EINVAL;

	/* Offset of the PLL status register */
	reg = readl(clk->regs_base + 0x18);

	/* bit 12 is "lock" signal, has to be "1" for proper PLL operation */
	if((reg & (1 << IPROC_WRAP_GEN_PLL_STATUS__GEN_PLL_LOCK)) == 0) {
		clk->rate = 0;
		return -EIO;
	}

	/* Update PLL frequency */

	/* get PLL feedback divider values from control5 */
	reg = readl(clk->regs_base + 0x04);

	/* feedback divider integer and fraction parts */
	ndiv_int = (reg >> IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_R) & ((1 << IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_WIDTH) -1);

	/* get pdiv */
	pdiv = (reg >> IPROC_WRAP_GEN_PLL_CTRL1__PDIV_R) & ((1 << IPROC_WRAP_GEN_PLL_CTRL1__PDIV_WIDTH) -1);

	if (pdiv == 0)
		return -EIO;

	x = clk->parent->rate / pdiv;

	x = x * ((u64) ndiv_int);

	clk->rate = x;

	return 0;
}
#endif

#if defined(CONFIG_MACH_KT2)
static int genpll_status(struct clk * clk)
{
	clk->rate = 2475000000;

	return 0;
}
#endif

#if defined(CONFIG_MACH_SB2)
static int genpll_status(struct clk * clk)
{
    clk->rate = 4000000000;
	return 0;
}
#endif

static const struct clk_ops genpll_ops = {
	.status = genpll_status,
};

#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_HR2) || defined(CONFIG_MACH_GH) || \
	defined(CONFIG_MACH_HR3)
static int genpll_chan_status(struct clk * clk)
{
	void * __iomem base;
	u32 reg;
	unsigned enable;
	unsigned mdiv = 0;
	unsigned off, shift;

	if (clk->parent == NULL || clk->type != CLK_DIV)
		return -EINVAL;

	/* Register address is only stored in PLL structure */
	base = clk->parent->regs_base;

	BUG_ON (base == NULL);
	/* GENPLL has the 6 channels spread over two regs */
	switch (clk->chan) {
		case 0:
			off = 0x04; shift = IPROC_WRAP_GEN_PLL_CTRL1__CH0_MDIV_R;
			break;

		case 1:
			off = 0x04; shift = IPROC_WRAP_GEN_PLL_CTRL1__CH1_MDIV_R;
			break;

		case 2:
			off = 0x08; shift = IPROC_WRAP_GEN_PLL_CTRL2__CH2_MDIV_R;
			break;

		case 3:
			off = 0x08; shift = IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_R;
			break;

		case 4:
			off = 0x08; shift = IPROC_WRAP_GEN_PLL_CTRL2__CH4_MDIV_R;
			break;

		case 5:
			off = 0x08; shift = IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_R;
			break;

		default:
			BUG_ON(clk->chan);
			off = shift = 0;	/* fend off warnings */
	}

	reg = readl(base + off);

	mdiv = 0xff & (reg >> shift);
	if(clk->chan == 5)
		mdiv *= 4;

	/* when divisor is 0, it behaves as max+1 */
	if (mdiv == 0)
		mdiv = 1 << 8;

	printk("GENPLL[%d] mdiv=%u rate=%lu\n",
		clk->chan, mdiv, clk->parent->rate);

	clk->rate = clk->parent->rate / mdiv;
	return 0;
}
#endif

#if defined(CONFIG_MACH_KT2)
static int genpll_chan_status(struct clk * clk)
{
	unsigned mdiv = 0;


	if (clk->parent == NULL || clk->type != CLK_DIV)
		return -EINVAL;

	/* GENPLL has the 6 channels spread over two regs */
	switch (clk->chan) {
		case 0:
			mdiv = 10;
			break;

		case 3:
			mdiv = 5;
			break;

		case 4:
			mdiv = 10;
			break;

		case 5:
			mdiv = 5;
			break;

		default:
			BUG_ON(clk->chan);
	}

	if(clk->chan == 5)
		mdiv *= 4;

	/* when divisor is 0, it behaves as max+1 */
	if (mdiv == 0)
		mdiv = 1 << 8;

	printk("GENPLL[%d] mdiv=%u rate=%lu\n",
		clk->chan, mdiv, clk->parent->rate);

	clk->rate = clk->parent->rate / mdiv;
	return 0;
}
#endif

#if defined(CONFIG_MACH_SB2)
static int genpll_chan_status(struct clk * clk)
{
	void * __iomem base;
	u32 reg;
	unsigned enable;
	unsigned mdiv = 0;
	unsigned off, shift;

	if (clk->parent == NULL || clk->type != CLK_DIV)
		return -EINVAL;

	/* Register address is only stored in PLL structure */
	base = clk->parent->regs_base;

	BUG_ON (base == NULL);
	/* IPROC_PLL has the 6 channels spread over two regs */
	switch (clk->chan) {
		case 0:
			off = 0x14; shift = IPROC_WRAP_IPROC_PLL_CTRL_5__CH0_MDIV_R;
			break;

		case 1:
            /* AXI clock */
			off = 0x14; shift = IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_R;
			break;

		case 2:
			off = 0x14; shift = IPROC_WRAP_IPROC_PLL_CTRL_5__CH2_MDIV_R;
			break;

		case 3:
			off = 0x14; shift = IPROC_WRAP_IPROC_PLL_CTRL_5__CH3_MDIV_R;
			break;

		case 4:
			off = 0x18; shift = IPROC_WRAP_IPROC_PLL_CTRL_6__CH4_MDIV_R;
			break;

		case 5:
            /* Used as APB clock, which is AXI clock/4 */
			off = 0x14; shift = IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_R;
			break;

		default:
			BUG_ON(clk->chan);
			off = shift = 0;	/* fend off warnings */
	}

    reg = readl(base + off);

	mdiv = 0xff & (reg >> shift);
	if(clk->chan == 5)
		mdiv *= 4;

	/* when divisor is 0, it behaves as max+1 */
	if (mdiv == 0)
		mdiv = 1 << 8;

	printk("GENPLL[%d] mdiv=%u rate=%lu\n",
		clk->chan, mdiv, clk->parent->rate);

	clk->rate = clk->parent->rate / mdiv;
	return 0;
}
#endif

static const struct clk_ops genpll_chan_ops = {
	.status = genpll_chan_status,
};


/*
 * GENPLL has 6 output channels
 */
static struct clk clk_genpll = {
	.ops 	= &genpll_ops,
	.name 	= "GENPLL",
	.type	= CLK_PLL,
	.chan	=	6,
};

/*
 * chan 0 - Ethernet switch and MAC, RGMII, need 250 MHz
 * chan 1 - Ethernet switch slow clock, 150 Mhz
 * chan 2 - USB PHY clock, need 30 MHz
 * chan 3 - iProc N MHz clock, set from OTP
 * chan 4 - iProc N/2 MHz clock, set from OTP
 * chan 5 - iProc N/4 MHz clock, set from OTP
 *
 * To Do: which clock goes to MPCORE PERIPHCLOCK?
 */
static struct clk clk_genpll_ch[6] = {
	{
		.ops	= &genpll_chan_ops,
		.parent = &clk_genpll,
		.type = CLK_DIV,
	  	.name	= "genpll_ch0",
		.chan	= 0,
	},
	{
		.ops	= &genpll_chan_ops,
		.parent = &clk_genpll,
		.type = CLK_DIV,
	  	.name	= "genpll_ch1",
		.chan	= 1,
	},
	{
		.ops	= &genpll_chan_ops,
		.parent = &clk_genpll,
		.type = CLK_DIV,
	  	.name	= "genpll_ch2",
		.chan	= 2,
	},
	{
		.ops	= &genpll_chan_ops,
		.parent = &clk_genpll,
		.type = CLK_DIV,
	  	.name	= "genpll_ch3",
		.chan	= 3,
	},
	{
		.ops	= &genpll_chan_ops,
		.parent = &clk_genpll,
		.type = CLK_DIV,
	  	.name	= "genpll_ch4",
		.chan	= 4,
	},
	{
		.ops	= &genpll_chan_ops,
		.parent = &clk_genpll,
		.type = CLK_DIV,
	  	.name	= "genpll_ch5",
		.chan	= 5,
	},
};

/*
 * This table is used to locate clock sources
 * from device drivers
 */

static struct clk_lookup ns_clk_lookups[] = {
	{
	.dev_id         = "pcie",
	.con_id         = "c_clk100",
	.clk            = &clk_lcpll_ch[0],
	},{
	.dev_id         = "sdio",
	.con_id         = "c_clk200",
	.clk            = &clk_lcpll_ch[1],
	},{
	.dev_id         = "ddr",
	.con_id         = "c_clk400",
	.clk            = &clk_lcpll_ch[2],
	},{
	.dev_id         = "tbd",
	.con_id         = "c_clk120",
	.clk            = &clk_lcpll_ch[3],
	},{
	.dev_id		= "en_phy",
	.con_id         = "c_clk250",
	.clk            = &clk_genpll_ch[0],
	},{
	.dev_id		= "en",
	.con_id         = "c_clk150",
	.clk            = &clk_genpll_ch[1],
	},{
	.dev_id         = "usb_phy",
	.con_id         = "c_clk30",
	.clk            = &clk_genpll_ch[2],
	},{
	.dev_id         = "iproc_fast",
	.con_id         = "c_clk500",
	.clk            = &clk_genpll_ch[3],
	},{
	.dev_id         = "iproc_med",
	.con_id         = "c_clk250",
	.clk            = &clk_genpll_ch[4],
	},{
	.dev_id         = "iproc_slow",
	.con_id         = "c_clk125",
	.clk            = &clk_genpll_ch[5],
	}
#ifdef CONFIG_ARM_AMBA
	,{
	.con_id		= "apb_pclk",
	.clk		= &clk_genpll_ch[5],
	}
#ifdef CONFIG_IPROC_SP805_WDT
	,{
	.dev_id		= "sp805-wdt",
	.clk		= &clk_genpll_ch[5],
	}
#endif
#endif
};

/* 
 * Install above clocks into clock lookup table 
 * and initialize the register base address for each
*/
static void __init northstar_clocks_init(void *__iomem cru_regs_base,
				struct clk * clk_ref)
{
	/*
	 * Registers are already mapped with the rest of DMU block
	 * Update register base address
	 */
#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_HR2) || defined(CONFIG_MACH_GH) || \
	defined(CONFIG_MACH_HR3)
	clk_lcpll.regs_base =	cru_regs_base + 0x1c ;
	clk_genpll.regs_base =	cru_regs_base + 0x00 ;
#elif defined(CONFIG_MACH_SB2)
    clk_genpll.regs_base =  cru_regs_base + 0x00 ;
#endif

	/* Set parent as reference ckock */
	clk_lcpll.parent	= clk_ref;
	clk_genpll.parent	= clk_ref;

	/* Install clock sources into the lookup table */
	clkdev_add_table(ns_clk_lookups, 
			ARRAY_SIZE(ns_clk_lookups));
}

void __init northstar_dmu_init(struct clk *clk_ref)
{
	void * __iomem 	reg_base;

	if (IS_ERR_OR_NULL(clk_ref )) {
		printk(KERN_ERR "CRU no clock source - skip init\n");
		return;
	}

	BUG_ON (request_resource(&iomem_resource, &dmu_regs));

	/* DMU regs are mapped as part of the fixed mapping with CCA+CCB */
	reg_base = (void * __iomem) dmu_regs.start;

	BUG_ON (IS_ERR_OR_NULL(reg_base));

	/* Initialize clocks */
#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_HR2) || defined(CONFIG_MACH_GH) || \
	defined(CONFIG_MACH_HR3)
	northstar_clocks_init(reg_base + 0xc00, clk_ref); /* IPROC_WRAP_GEN_PLL_CTRL0 */
#elif defined(CONFIG_MACH_KT2)
	northstar_clocks_init(NULL, clk_ref);
#elif defined(CONFIG_MACH_SB2)
	northstar_clocks_init(reg_base + 0xc50, clk_ref); /* IPROC_WRAP_IPROC_PLL_CTRL_0 */
#endif
}

/*
 * Reset the system
 */
void northstar_restart(char mode, const char *cmd)
{
	void * __iomem reg_addr;
	u32 reg;

	/* CRU_RESET register */
#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_HR2) || defined(CONFIG_MACH_KT2) || \
	defined(CONFIG_MACH_GH)  || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3)
	reg_addr = (void * __iomem) dmu_regs.start + DMU_CRU_RESET_BASE ;
#endif

	/* set iproc_reset_n to 0, it may come back or not ... TBD */
	reg = readl_relaxed(reg_addr);
	reg &= ~((u32) 1 << 1);
#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3)
    /* Reset switch as well */
	reg &= ~((u32) 1 << 0);
#endif /* CONFIG_MACH_GH */
	writel_relaxed(reg, reg_addr);
}







void northstar_clocks_show( void )
{
	unsigned i;
//	struct clk * clk ;

	printk("=========== CLOCKS =================\n");

	printk( "DMU Clocks:\n" );
	for (i = 0; i < ARRAY_SIZE( ns_clk_lookups); i++) {
		printk("%s, %s: (%s) %lu\n",
			ns_clk_lookups[i].con_id,
			ns_clk_lookups[i].dev_id,
			ns_clk_lookups[i].clk->name,
			clk_get_rate( ns_clk_lookups[i].clk));
	}
	printk( "DMU Clocks# %u\n", i );
}
