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

#ifndef __MACH_CLKDEV_H
#define __MACH_CLKDEV_H

#include <asm/atomic.h>
#include <mach/clock.h>

struct clk {
	const struct clk_ops 	*ops;
	const char 		*name;
	atomic_t		ena_cnt;
	atomic_t		use_cnt;
	unsigned long		rate;
	unsigned		gated :1;
	unsigned		fixed :1;
	unsigned		chan  :6;
	void __iomem 		*regs_base;
	struct clk 		*parent;
	/* TBD: could it have multiple parents to select from ? */
	enum {
		CLK_XTAL, CLK_GATE, CLK_PLL, CLK_DIV, CLK_PHA
	} type;
};

extern int __clk_get(struct clk *);
extern void __clk_put(struct clk *);

#endif
