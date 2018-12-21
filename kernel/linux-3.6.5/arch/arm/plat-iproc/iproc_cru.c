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
 * iProc Clock Control Unit
 * The software model repsresents the hardware clock hierarchy
 *
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

#include <mach/clkdev.h>
#include <mach/iproc_regs.h>
#include <mach/io_map.h>
#include <mach/memory.h>

#include <asm/clkdev.h>
#include <asm/pgtable.h>

static struct resource ccu_regs = {
	.name  = "cru_regs",
	.start = IPROC_ROOT_CLK_VA,
	.end   = ((IPROC_ROOT_CLK_VA) + 0x0fff),
        .flags = IORESOURCE_MEM,
};

/*
 * Clock management scheme is a provisional implementation
 * only intended to retreive the pre-set frequencies for each
 * of the clocks.
 */

/*
 * Get PLL running status and update output frequency
 * for ARMPLL channel 0
 */
static int a9pll0_status (struct clk * clk)
{
	u32 regA;
	u32 regB;
	u32 regC;
	u32 pdiv;
	u32 ndiv_int;
	u32 ndiv_frac;
	u32 mdiv;
	u32 ref_clk;
	u64 x;

	printk(KERN_DEBUG "a9pll0_status: clk 0x%x\n", (unsigned int)clk);
	if (clk->type != CLK_PLL)
		return -EINVAL;

	BUG_ON (!clk->regs_base);
	BUG_ON (!clk->parent);
	ref_clk = clk->parent->rate;

#if defined(CONFIG_MACH_GH)
  if (readl_relaxed(HW_IO_PHYS_TO_VIRT(IPROC_WRAP_TOP_STRAP_STATUS_1)) & 0x01)  /* 50 MHz */
	  ref_clk = 50000000;
#endif

	/* read status register */
	regA = readl(clk->regs_base + 0x0c00);/* IHOST_PROC_CLK_PLLARMA */
	regB = readl(clk->regs_base + 0x0c04);/* IHOST_PROC_CLK_PLLARMB */
	regC = readl(clk->regs_base + 0x0c08);/* IHOST_PROC_CLK_PLLARMB */

	/* reg C bit 8 is bypass mode - input frequency to output */
	if ((regC & (1 << 8)) == 1) {
		clk->rate = ref_clk;
		return 0;
	}
	/* Update PLL frequency */


	/* pdiv in bits 24..27 (4 bits) */
	// pdiv = (regA >> 24 ) & 0xf; pavan 05092012
	pdiv = (regA >> 24 ) & 0x7;
	if (pdiv == 0)
		pdiv = 0x10;

	/* feedback divider (int)  in bits 8..17 (10 bits) */
	ndiv_int = (regA >> 8) & ((1<<10) - 1);
	if (ndiv_int == 0)
		ndiv_int = 1 << 10;

	/* feedback divider fraction in reg B, bits 0..19 */
	ndiv_frac = regB & ((1<<20) - 1);

	x = ((u64) ndiv_int << 20) | ndiv_frac;
	printk(KERN_INFO "parent rate %lu, x: %llu\n", ref_clk, x);
	x = (x * ref_clk);
 printk(KERN_DEBUG "multi rate - current x: %llu\n", x);

        x = x >> 20;
       printk(KERN_INFO "after shitf - current x: %llu\n", x);
	(void) do_div(x, pdiv);

        printk(KERN_DEBUG "after do_div - current x: %llu\n", x);

	/* post-divider is in reg C bits 0..7 */
	mdiv = regC & 0xff ;
	if (mdiv == 0)
		mdiv = 0x100;

	/*
	 * Here we need to divide the resulting clock by mdiv which we
	 * are not doing now?
	 */
	clk->rate = (u32)(x);

        printk("ARMPLL0: pdev: %d, ndev: %d, ndiv_frac: %d, mdiv: %d, rate: %lu\n", pdiv, ndiv_int, ndiv_frac, mdiv, clk->rate);

	return 0;
}


/*
 * Get PLL running status and update output frequency
 * for ARMPLL channel 1
 */
static int a9pll1_status(struct clk * clk)
{
	u32 regA;
	u32 regB;
	u32 regC;
	u32 regD;
	unsigned pdiv;
	unsigned ndiv_int;
	unsigned ndiv_frac;
	unsigned mdiv;
	u32 ref_clk;
	u64 x;

	if (clk->type != CLK_PLL)
		return -EINVAL;

	BUG_ON(!clk->regs_base);
	BUG_ON(!clk->parent);
	ref_clk = clk->parent->rate;

#if defined(CONFIG_MACH_GH)
  if (readl_relaxed(HW_IO_PHYS_TO_VIRT(IPROC_WRAP_TOP_STRAP_STATUS_1)) & 0x01)  /* 50 MHz */
	  ref_clk = 50000000;
#endif

	/* read status register */
	regA = readl(clk->regs_base+0xc00);/* IHOST_PROC_CLK_PLLARMB */
	regB = readl(clk->regs_base+0xc04);/* IHOST_PROC_CLK_PLLARMB */
	regC = readl(clk->regs_base+0xc20);/* IHOST_PROC_CLK_PLLARMCTRL5 */
	regD = readl(clk->regs_base+0xc24);/* IHOST_PROC_CLK_PLLARM_OFFSET*/
	
	/* reg C bit 8 is bypass mode - input frequency to output */
	if ((regC & (1 << 8)) == 1) {
		clk->rate = ref_clk;
		return 0;
	}

	/* reg A bit 28 is "lock" signal, has to be "1" for proper operation */
	if ((regA & (1 << 28)) == 0) {
#if defined(CONFIG_MACH_IPROC_P7) && defined(CONFIG_MACH_IPROC_EMULATION)
		printk(KERN_WARNING "a9pll1_status: ARM PLL not locked\n");
#else
		clk->rate = 0;
		return -EIO;
#endif
	}

	/* Update PLL frequency */


	/* pdiv in bits 24..27 (4 bits) */
	pdiv = (regA >> 24 ) & 0xf;
	if (pdiv == 0)
		pdiv = 0x10;

	/* Check if offset mode is active */
	if (regD & (1 << 29)) {
		/* pllarm_ndiv_int_offset bits 27:20 */
		ndiv_int = (regD >> 20 ) & 0xff;
		if (ndiv_int == 0)
			ndiv_int = 1 << 8;

		/* pllarm_ndiv_frac_offset bits 19:0 */
		ndiv_frac = regD & ((1 << 20) - 1);
	} else {
		/* If offset not active, channel 0 parameters are used */
		/* feedback divider (int)  in bits 8..17 (10 bits) */
		ndiv_int = (regA >> 8) & ((1 << 10) - 1);
		if (ndiv_int == 0)
			ndiv_int = 1 << 10;

		/* feedback divider fraction in reg B, bits 0..19 */
		ndiv_frac = regB & ((1 << 20) - 1);
	}

	x = ((u64) ndiv_int << 20) | ndiv_frac;
	x = (x * ref_clk) >> 20;
	(void) do_div(x, pdiv);

	/* post-divider is in reg C bits 0..7 */
	mdiv = regC & 0xff ;
	if (mdiv == 0)
		mdiv = 0x100;

	(void) do_div(x, mdiv);
	clk->rate = (u32)(x);
printk("ARMPLL1:  pdev: %d, ndev: %d, ndiv_frac: %d, mdiv: %d, rate: %d\n", pdiv, ndiv_int, ndiv_frac, mdiv, clk->rate);
	return 0;
}


static const struct clk_ops a9pll0_ops = {
	.status = a9pll0_status,
};

static const struct clk_ops a9pll1_ops = {
	.status = a9pll1_status,
};


/*
 * iProc A9 PLL
 * could be used as source for generated clocks
 */
static struct clk clk_a9pll[2] = {
	{
		.ops 	= &a9pll0_ops,
		.name 	= "A9_PLL",
		.type	= CLK_PLL,
		.chan	= 0xa,
	},
	{
		.ops 	= &a9pll1_ops,
		.name 	= "A9_PLL",
		.type	= CLK_PLL,
		.chan	= 0xb,
	},
};

/*
 * Decode the Frequency ID setting for arm_clk
 */
static int iproc_cru_arm_freq_id(void * __iomem regs_base)
{
	u32 reg_f, reg;
	unsigned policy;
	unsigned fid;
	unsigned i;
	u8 arm_clk_policy_mask = 0;
	u8 apb0_clk_policy_mask = 0;

	/*
	 * bits 0..2 freq# for policy0, 8..10 for policy1,
	 * 16..18 policy2, 24..26 policy 3
	 */
	reg_f = readl(regs_base + 0x008);/*IHOST_PROC_CLK_POLICY_FREQ*/

	for(i = 0; i < 4; i++) {
		/*
		 * Reg IHOST_PROC_CLK_POLICY<i>_MASK
		 * bit 20 arm policy mask, bit 21 apb0 policy mask
		 */
		reg = readl(regs_base + 0x010 + i*4);
		arm_clk_policy_mask |= (1 & ( reg >> 20)) << i;
		apb0_clk_policy_mask |=  (1 & ( reg >> 21)) << i;
	}

	/* How to obtain hardware policy setting ? */
	policy = 0;

	/* Check for PLL policy software override */
	reg = readl(regs_base + 0xe00);/* IHOST_PROC_CLK_ARM_DIV */
	if (reg & (1 << 4 ))
		policy = reg & 0xf;

	fid = (reg_f >> (8 * policy)) & 0xf;

	/* Verify freq_id from debug register */
	reg = readl( regs_base+0xec0 );/* IHOST_PROC_CLK_POLICY_DBG */
	/* Bits 12..14 contain active frequency_id */
	i = 0x7 & (reg >> 12);

	if (fid != i) {
		printk(KERN_WARNING
			"IPROC CRU clock frequency id override %d->%d\n",
			fid, i);
		fid = i;
	}

	printk(KERN_DEBUG "Active frequency ID %d\n", fid);

	return fid;
}

/*
 * Get status of any of the ARMPLL output channels
 */
static int a9pll_chan_status(struct clk * clk)
{
	u32 reg, regC;
	unsigned div;
	unsigned freq_id;

	if(clk->type != CLK_DIV)
		return -EINVAL;

	BUG_ON(!clk->regs_base);

	reg = readl(clk->regs_base + 0xe00); /* IHOST_PROC_CLK_ARM_DIV */
	regC = readl(clk->regs_base + 0xc20); /* IHOST_PROC_CLK_PLLARMCTRL5 */

	/* arm_pll_select 3:0 */
	printk(KERN_DEBUG "Clock Div = %#x\n", reg);

	freq_id = iproc_cru_arm_freq_id(clk->regs_base);


        printk(KERN_INFO "arm-freq-id: %d\n", freq_id);

	/* clk->parent = & clk_a9pll[0]; */

	switch (clk->chan) {
		case 0x0a:
			/* apb0_free_div bits 10:8 */
			div = (reg >> 8) & 0x7;
			if (div == 0)
				div = 8;
			break;

		case 0x0b:
			/* arm_switch_div bits 6:5 */
			div = (reg >> 5) & 0x3;
			if (div == 0)
				div = 4;
			break;

		case 0x1a:
			/* IHOST_PROC_CLK_APB_DIV apb_clk_div bits 1:0 */
			reg = readl(clk->regs_base + 0xa10);
			div = reg & 0x3;
			if(div == 0)
				div = 4;
			break;

		case 0x3a:      /* arm_clk */
			if( freq_id == 7 ) {
				//clk->parent = &clk_a9pll[1]; /* arm_clk_h */
				clk->parent = &clk_a9pll[0]; /* arm_clk_h */
				div = regC & 0xff; /* mdiv_h for frequency ID 7 */
			} else if( freq_id == 6 ) {
				clk->parent = &clk_a9pll[0]; /* arm_clk */
				div = 4;
			} else if( freq_id == 2 ) {
                                clk->parent = &clk_a9pll[0]; /* arm_clk */
                                div = 2;
                                clk->parent->rate = 200000000;
                                printk(KERN_DEBUG "freq_id parent clock %d\n", clk->parent->rate);
			} else if (freq_id == 0) {
				clk->parent = &clk_a9pll[0];
				div = 1;
			} else if (freq_id == 1) {
				struct clk * clk_lcpll_200;
				clk_lcpll_200 =
				clk_get_sys( NULL, "sdio_clk");
				BUG_ON( ! clk_lcpll_200 );
				clk->parent = clk_lcpll_200;
				div = 1;
			} else {
				clk->parent = &clk_a9pll[0];
				div = 2;
			}
			break;

                case 0x0f:      /* periph_clk */
                        div = 2;
                        break;

		default:
			return -EINVAL;

	}

	BUG_ON(!clk->parent);
	printk(KERN_DEBUG "Clock divisor %d\n", div);
	// clk->rate = clk->parent->rate / div ;
	clk->rate = clk->parent->rate / div ;
	printk(KERN_DEBUG "Clock rate %lu\n", clk->rate);
 
	return 0;
}


static const struct clk_ops a9pll_chan_ops = {
	.status = a9pll_chan_status,
};

/*
 * iProc A9 PLL output clocks
 */
static struct clk clk_a9chan[] = {
	{
		.ops = &a9pll_chan_ops,
		.type = CLK_DIV,
		.parent = &clk_a9chan[0],
		.name = "arm_clk",
		.chan = 0x3a
	},
	{
		.ops = &a9pll_chan_ops,
		.type = CLK_DIV,
		.parent = &clk_a9chan[0],
		.name = "periph_clk",
		.chan = 0x0f
	},
	{
		.ops = &a9pll_chan_ops,
		.type = CLK_DIV,
		.parent = &clk_a9chan[0],
		.name = "apb0_free",
		.chan = 0x0a
	},
	{
		.ops = &a9pll_chan_ops,
		.type = CLK_DIV,
		.parent = &clk_a9chan[0],
		.name = "arm_switch",
		.chan = 0x0b
	},
	{
		.ops = &a9pll_chan_ops,
		.type = CLK_DIV,
		.parent = &clk_a9chan[0],
		.name = "apb_clk",
		.chan = 0x1a
	},
};

static struct clk_lookup cru_clk_lookups[] = {
	{
		.con_id= "a9pll0",
		.clk= &clk_a9pll[0],
	},
	{
		.con_id= "a9pll1",
		.clk= &clk_a9pll[1],
	},
	{
		.con_id= "arm_clk",
		.clk= &clk_a9chan[0],
	},
	{
		.con_id= "periph_clk",
		.clk= &clk_a9chan[1],
	},
	{
		.con_id= "apb0_free",
		.clk= &clk_a9chan[2],
	},
	{
		.con_id= "axi_clk",
		.clk= &clk_a9chan[3],
	},
	{
		.con_id= "apb_clk",
		.clk= &clk_a9chan[4],
	},
};

void cru_clocks_show(void)
{
        unsigned i;

        printk( "CRU Clocks:\n" );
        for (i = 0; i < ARRAY_SIZE( cru_clk_lookups); i++) {
                printk( "%s: (%s) %lu\n",
                        cru_clk_lookups[i].con_id,
                        cru_clk_lookups[i].clk->name,
                        clk_get_rate( cru_clk_lookups[i].clk));
        }
        printk( "CRU Clocks# %u\n", i );

}

void __init iproc_cru_init(struct clk * src_clk)
{
	void * __iomem reg_base;
	unsigned i;

	BUG_ON(request_resource( &iomem_resource, &ccu_regs));

	reg_base = IOMEM(ccu_regs.start);

	BUG_ON(IS_ERR_OR_NULL(reg_base));

	/* Initialize clocks */

	for (i = 0; i < ARRAY_SIZE(clk_a9pll); i++) {
		clk_a9pll[i].regs_base = reg_base ;
		clk_a9pll[i].parent = src_clk ;
	}

	clk_a9chan[0].parent = src_clk ;	/* tentative */
	for (i = 0; i < ARRAY_SIZE(clk_a9chan); i++) {
		clk_a9chan[i].regs_base = reg_base ;
	}

	/* Install clock sources into the lookup table */
	clkdev_add_table(cru_clk_lookups, 
			ARRAY_SIZE(cru_clk_lookups));

//        cru_clocks_show();
}
