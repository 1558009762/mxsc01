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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cpumask.h>
#include <linux/reboot.h>
#include <linux/pm.h>

#include <asm/io.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/system_misc.h>
#include <asm/pgtable.h>
#include <asm/localtimer.h>
#include <asm/smp_twd.h>

#include <mach/clkdev.h>
#include <mach/timer.h>
#include <mach/iproc.h>
#include <mach/io_map.h>
#include <mach/irqs.h>
#include <mach/memory.h>





#define TIMER_LOAD      	0x00
#define TIMER_VALUE     	0x04
#define TIMER_CTRL      	0x08
#define TIMER_CTRL_PRESC_SHFT   (8)
#define TIMER_CTRL_IE           (1 << 2)
#define TIMER_CTRL_PERIODIC     (1 << 1)
#define TIMER_CTRL_ENABLE       (1 << 0)

#define TIMER_INTCLR    	0x0c
#define IPROC_L2CC_REG_BASE_VA  HW_IO_PHYS_TO_VIRT(IPROC_L2CC_REG_BASE)

extern void __iomem *twd_base;
extern void iproc_clocksource_init(void __iomem *);
extern void iproc_clockevents_init(void __iomem *, unsigned int);
extern void __init northstar_dmu_init(struct clk *clk_ref);
extern void __init iproc_cru_init(struct clk *clk_ref);
extern void iproc_enable_data_prefetch_aborts(void);
extern void northstar_restart(char mode, const char *cmd);

static void
northstar_poweroff(void)
{
	while(1)
	;
}


#ifdef CONFIG_CACHE_L2X0
static void __init northstar_l2x0_init(void)
{
	void __iomem *l2cache_base = IOMEM(IPROC_L2CC_REG_VA);
	void __iomem *cca = IOMEM(IPROC_CCA_CORE_REG_VA);
	unsigned int chipid = (readl(cca) & 0x0000ffff);

	/*
	 * 16KB way size, 16-way associativity
	 */
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
		l2x0_init(l2cache_base, 0x0A150000, ~(0x000F0000));
#elif defined(CONFIG_MACH_HR2)
		l2x0_init(l2cache_base, 0x0A120000, ~(0x000F0000));
#elif defined(CONFIG_MACH_IPROC_P7)
		l2x0_init(l2cache_base, 0x0A130000, ~(0x000F0000));
#endif
}
#endif

static int __init northstar_init(void)
{
	pm_power_off = northstar_poweroff;
	arm_pm_restart = northstar_restart;

#ifdef CONFIG_CACHE_L2X0
	northstar_l2x0_init();
#endif

	return 0;
}
early_initcall(northstar_init);

/*
 * CPU global and MPCORE Per CPU local timer
 */
#define GLB_TIMER IOMEM(IPROC_PERIPH_GLB_TIM_REG_VA);
#define PVT_TIMER IOMEM(IPROC_PERIPH_PVT_TIM_REG_VA);

void __iomem *gtimer_va_base    = GLB_TIMER;
void __iomem *ptimer_va_base    = PVT_TIMER;

#define IRQ_LOCALTIMER BCM_INT_ID_PPI13
/*
 * Setup the local clock events for a CPU.
 */

static DEFINE_TWD_LOCAL_TIMER(twd_local_timer,
			      HW_IO_VIRT_TO_PHYS(IPROC_PERIPH_PVT_TIM_REG_VA),
			      IRQ_LOCALTIMER);


/*
 * Set up the clock source and clock events devices
 */
void __init northstar_timer_init(struct clk *clk_ref)
{
	int err;

	/*
	 * Setup DMU and CRU early
	 */
	northstar_dmu_init(clk_ref);
	iproc_cru_init(clk_ref);

	/*
	 * Initialise to a known state (all timers off)
	 */
	writel(0, ptimer_va_base + TIMER_CTRL);
	writel(0, gtimer_va_base + TIMER_CTRL);

#ifdef CONFIG_LOCAL_TIMERS
	err = twd_local_timer_register(&twd_local_timer);
	if (err)
		pr_err("iproc: twd_local_timer_register failed %d\n", err);
#endif

	iproc_clocksource_init(gtimer_va_base);
	iproc_clockevents_init(gtimer_va_base, BCM_INT_ID_PPI11);

	iproc_enable_data_prefetch_aborts();
}
