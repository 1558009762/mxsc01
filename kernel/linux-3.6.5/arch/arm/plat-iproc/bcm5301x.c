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

#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <asm/signal.h>
#include <asm/mach/map.h>
#include <asm/pgtable.h>
#include <asm/bug.h>
#include <mach/iproc_regs.h>
#include <mach/io_map.h>
#include <mach/memory.h>

static int iproc_data_abort_handler(unsigned long addr,	unsigned int fsr,
				struct pt_regs *regs)
{
	/*
	 * These happen for no good reason
	 */
//	printk(KERN_WARNING	"Data abort at addr=%#lx, fsr=%#x ignored.\n", addr, fsr);
	return 0;
}

void __init iproc_enable_data_prefetch_aborts(void)
{
	u32 x;

	/* Install our hook */
	hook_fault_code(16 + 6, iproc_data_abort_handler, SIGBUS, 0,
			"imprecise external data abort");

	/* Enable external aborts - clear "A" bit in CPSR */

	/* Read CPSR */
//	asm("mrs       %0,cpsr": "=&r" (x) : : );
	asm("mrs       %0,cpsr": "=&r" (x) );

	x &= ~ PSR_A_BIT;

	/* Update CPSR, affect bits 8-15 */
	asm("msr       cpsr_x,%0; nop; nop": : "r" (x) : "cc");

}
