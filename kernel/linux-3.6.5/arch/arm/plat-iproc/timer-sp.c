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


#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/io_map.h>

#include <asm/clkdev.h>
//#include <mach/clkdev.h>
#include <asm/hardware/arm_timer.h>

#include <plat/timer-sp.h>

/*
 * These timers are currently setup to be clocked at 50MHz.
 */
#define ONE_MHZ			(1000000)
#define TIMER_FREQ_HZ		(ONE_MHZ * 500)
#define TIMER_FREQ_KHZ		(TIMER_FREQ_HZ/1000)
#define TIMER_MIN_RANGE		4

#define TIM_COUNT_LO		0x00            /* ACVR rw */
#define TIM_COUNT_HI		0x04            /* ACVR ro */
#define TIMER_CTRL		0x08            /* ACVR rw */
#define TIMER_INT_STAT		0x0C
#define TIMER_COMP_LO		0x10
#define TIMER_COMP_HI		0x14
#define TIMER_RELOAD		0x18
#define TIMER_CTRL_PRESC_SHFT   (8)             /* ACVR */

#define TIMER_ENABLE		(1 << 0)        /* ACVR */
#define TIMER_CMP		(1 << 1)
#define TIMER_IRQ		(1 << 2)
#define TIMER_AUTO		(1 << 3)

extern unsigned long clk_get_rate(struct clk *clk);

static void __iomem *clksrc_base;
static u32 ticks_per_jiffy;
static u32 timer_ints = 0;
static unsigned long cpu_clk_freq = 0;

static cycle_t iproc_read(struct clocksource *cs)
{
	u32 hi;
	u32 lo;
	u32 ho;
	u64 count;

	/*
	 * Read the upper half to detect a roll-over count
	 */
	do {
		hi = readl(clksrc_base + TIM_COUNT_HI);
		lo = readl(clksrc_base + TIM_COUNT_LO);
		ho = readl(clksrc_base + TIM_COUNT_HI);
	} while(hi != ho);

	count = (u64) hi << 32 | lo;
	return count;

}
static void iproc_set_mode(enum clock_event_mode mode,
        struct clock_event_device *evt);

static int iproc_set_next_event(unsigned long next,
        struct clock_event_device *evt);
static struct clock_event_device iproc_clockevent = {
        .name           = "iproc_gtimer",
        .shift          = 20,
        .features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
        .set_mode       = iproc_set_mode,
        .set_next_event = iproc_set_next_event,
        .rating         = 300,
        .cpumask        = cpu_all_mask,
};


static struct clocksource clocksource_iproc = {
	.name		= "iproc_gtimer",
	.rating		= 300,
	.read		= iproc_read,
	.mask		= CLOCKSOURCE_MASK(64),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};
extern void setup_sched_clock(u32 (*read)(void), int bits, unsigned long rate);

void __init iproc_clocksource_init(void __iomem *base)
{
	struct clocksource *cs = &clocksource_iproc;
	struct clk *clk;

	clksrc_base = base;

        /*
	 * setup global CPU timer as free-running clocksource
	 * obtain CPU clock frequency from clock module configuration
	 */
	clk = clk_get_sys(NULL, "periph_clk");
	BUG_ON(IS_ERR_OR_NULL(clk));
        clk_prepare_enable(clk);
	cpu_clk_freq = clk_get_rate(clk);
	BUG_ON(!cpu_clk_freq);


	printk(KERN_DEBUG
		"iproc_clocksource_init: CPU global timer freq %lu\n",
		cpu_clk_freq);

	/* ref - arch/arcm/mach-u300/timer.c (2.6.37 vs 2.6.38) */
	clocksource_register_hz(cs, cpu_clk_freq);
}


static void __iomem *clkevt_base;

/*
 * IRQ handler for the timer
 */
 irqreturn_t iproc_timer_interrupt(int irq, void *dev_id)
{
       struct clock_event_device *evt = &iproc_clockevent;
#warning "iproc_timer_interrupt: Fix this code to receive clock_event handler correctly"

	/* clear the interrupt */
	writel(1, clkevt_base + TIMER_INT_STAT);

	timer_ints++;
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static void iproc_set_mode(enum clock_event_mode mode,
	struct clock_event_device *evt)
{
	u32 ctrl;
	u32 period;
	u64 count;

	ctrl = readl(clkevt_base + TIMER_CTRL);

	/* Clear mode bits */
	ctrl &= ~(TIMER_CMP | TIMER_IRQ | TIMER_AUTO);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		writel(ctrl, clkevt_base + TIMER_CTRL);
		period = ticks_per_jiffy;
		count = iproc_read(NULL);
		count += period;
		writel(ctrl, clkevt_base + TIMER_CTRL);
		writel(period, clkevt_base + TIMER_RELOAD);
		ctrl =  (TIMER_CMP  |
			 TIMER_IRQ  |
			 TIMER_AUTO |
			 TIMER_ENABLE);
		break;

	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and timer enabled in 'next_event' hook */
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	default:
		break;
	}

	writel(ctrl, clkevt_base + TIMER_CTRL);
}

static int iproc_set_next_event(unsigned long next,
	struct clock_event_device *evt)
{
	u64 count = iproc_read(NULL);
	u32 ctrl  = readl(clkevt_base + TIMER_CTRL);

	ctrl &= ~TIMER_CMP;
	writel(ctrl, clkevt_base + TIMER_CTRL);

	count += next;
	writel(count & 0xffffffffUL,    clkevt_base + TIMER_COMP_LO);
	//writel(count, clkevt_base + TIM_COUNT_LO);
	writel(count >> 32, clkevt_base + TIMER_COMP_HI);

//	ctrl |= (TIMER_CMP | TIMER_IRQ);
	ctrl |= (TIMER_CMP | TIMER_IRQ | TIMER_AUTO);
	writel(ctrl, clkevt_base + TIMER_CTRL);

	return 0;
}

static struct irqaction iproc_timer_irq = {
	.name		= "iproc_gtimer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_PERCPU,
	.handler	= iproc_timer_interrupt,
	.dev_id		= &iproc_clockevent,
        .irq = BCM_INT_ID_PPI11,
};

void __init iproc_clockevents_init(void __iomem *base, unsigned int timer_irq)
{
        int ret;
	clkevt_base = base;
	ticks_per_jiffy = (cpu_clk_freq/HZ);

        clockevents_calc_mult_shift(&iproc_clockevent, cpu_clk_freq, TIMER_MIN_RANGE);
        iproc_clockevent.max_delta_ns =
                clockevent_delta2ns(0x1fffffff, &iproc_clockevent);
        iproc_clockevent.min_delta_ns =
                clockevent_delta2ns(0xf, &iproc_clockevent);
        iproc_clockevent.cpumask = cpumask_of(0);
        iproc_clockevent.irq = timer_irq;
        clockevents_register_device(&iproc_clockevent);
#warning "iproc_clockevents_init: Fix this code to enable timer irq and pass clock_event handler correctly"
        ret = setup_percpu_irq(timer_irq, &iproc_timer_irq);
        if (ret) {
                printk(KERN_ERR "Failed to register timer IRQ: %d\n", ret);
                BUG();
        }

        printk(KERN_DEBUG "cpu_clk_freq: %lu\n", cpu_clk_freq);
        printk(KERN_DEBUG "HZ: %d, ticks_per_jiffy: %u\n", HZ, ticks_per_jiffy);

        enable_percpu_irq(timer_irq, 0);

}
