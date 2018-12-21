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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/clockchips.h>
#include <linux/types.h>

#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <mach/io.h>
#include <mach/io_map.h>
#include <mach/iproc_timer.h>
#include <mach/timer.h>
#include <mach/iproc_regs.h>

#ifdef CONFIG_LOCAL_TIMERS
#include <asm/smp_twd.h>
#endif

static struct iproc_timer *gpt_evt = NULL;
static struct iproc_timer *gpt_src = NULL;
static void __iomem*    proftmr_regbase = IOMEM(IPROC_CCU_PROF_CTL_REG_VA);

static int gptimer_set_next_event(unsigned long clc,
        struct clock_event_device *unused)
{
    /* gptimer (0) is disabled by the timer interrupt already
     * so, here we reload the next event value and re-enable
     * the timer
     *
     * This way, we are potentially losing the time between
     * timer-interrupt->set_next_event. CPU local timers, when
     * they come in should get rid of skew 
     */
    iproc_timer_set_match_start(gpt_evt,clc);
    return 0;
}

static void gptimer_set_mode(enum clock_event_mode mode,
                   struct clock_event_device *unused)
{
    switch (mode) {
    case CLOCK_EVT_MODE_ONESHOT:
        /* by default mode is one shot don't do any thing */
        break;
    case CLOCK_EVT_MODE_UNUSED:
    case CLOCK_EVT_MODE_SHUTDOWN:
    default:
        iproc_timer_disable_and_clear(gpt_evt);
    }
}

static cycle_t gptimer_clksrc_read (struct clocksource *cs)
{
    unsigned long msw, lsw;
    cycle_t    count = 0;

    iproc_timer_get_counter (gpt_src, &msw, &lsw);
    count = ((cycle_t)msw << 32) | (cycle_t)lsw;
    return count;
}

static struct clock_event_device clockevent_gptimer = {
    .name            = "gpt_event_1",
    .features        = CLOCK_EVT_FEAT_ONESHOT,
    .shift           = 32,
    .set_next_event  = gptimer_set_next_event,
    .set_mode        = gptimer_set_mode
};

static struct clocksource clksrc_gptimer = {
    .name    = "gpt_source_2",
    .rating  = 200,
    .read    = gptimer_clksrc_read,
    .mask    = CLOCKSOURCE_MASK(64),
    .shift   = 16,
    .flags   = CLOCK_SOURCE_IS_CONTINUOUS,
};

static void __init gptimer_clockevents_init(void)
{
    clockevent_gptimer.mult = div_sc(CLOCK_TICK_RATE, NSEC_PER_SEC,
                        clockevent_gptimer.shift);

    clockevent_gptimer.max_delta_ns =
        clockevent_delta2ns(0xffffffff, &clockevent_gptimer);

    clockevent_gptimer.min_delta_ns =
        clockevent_delta2ns(6, &clockevent_gptimer);

    clockevent_gptimer.cpumask = cpumask_of(0);
    clockevents_register_device(&clockevent_gptimer);
}

static void __init gptimer_clocksource_init(void)
{
    clksrc_gptimer.mult = clocksource_hz2mult(CLOCK_TICK_RATE, 
        clksrc_gptimer.shift);
    clocksource_register(&clksrc_gptimer);
    return;
}

static int gptimer_interrupt_cb(void *dev)
{
    struct clock_event_device *evt = (struct clock_event_device *)dev;
    evt->event_handler(evt);
    return 0;
}

static void profile_timer_init(void __iomem *base)
{
    uint32_t reg;

    /* Reset profile/global timer */
    writel(0, base + IPROC_GTIM_GLB_CTL);

    /* Clear pending interrupts */
    reg = readl(base + IPROC_GTIM_GLB_STS);
    reg &= ~(IPROC_GLB_TIM_CTRL_PRESC_MASK);
    reg |= (1 << GLBTMR_GLOB_STATUS_EVENT_G_SHIFT);
    writel(reg, base + IPROC_GTIM_GLB_STS);

    /* Enable profile timer now with
     * prescaler = 0, so timer freq = A9 PERIPHCLK 
     * IRQ disabled
     * Comapre disabled
     */

    reg = readl(base + IPROC_GTIM_GLB_CTL);
    reg &= ~(IPROC_GLB_TIM_CTRL_PRESC_MASK);
    reg |= (1 << GLBTMR_GLOB_CTRL_TIMER_EN_G_SHIFT);
    writel(reg, base + IPROC_GTIM_GLB_CTL);
}

static void
profile_timer_get_counter(void __iomem *base, uint32_t *msw, uint32_t *lsw)
{
    /* Read 64-bit free running counter
     * 1. Read hi-word
     * 2. Read low-word
     * 3. Read hi-word again
     * 4.1 
     *     if new hi-word is not equal to previously read hi-word, then
     *     start from #1
     * 4.2
     *     if new hi-word is equal to previously read hi-word then stop.
     */

    while (1) {
        *msw = readl(base + IPROC_GTIM_GLB_HI);
        *lsw = readl(base + IPROC_GTIM_GLB_LO);
        if (*msw == readl(base + IPROC_GTIM_GLB_HI))
            break;
    }

    return;
}

static void __init timers_init(struct gp_timer_setup *gpt_setup)
{
    struct timer_ch_cfg evt_tm_cfg;

    iproc_timer_modules_init ();
    iproc_timer_module_set_rate(gpt_setup->name, gpt_setup->rate);
    
    /* Initialize Event timer */
    gpt_evt = iproc_timer_request(gpt_setup->name, gpt_setup->ch_num);
    if (gpt_evt == NULL) {
        pr_err("timers_init: Unable to get GPT timer for event\r\n");
    }

    pr_info("timers_init: === SYSTEM TIMER NAME: %s CHANNEL NUMBER %d \
    RATE (0-32KHz, 1-1MHz) %d \r\n",gpt_setup->name, 
    gpt_setup->ch_num, gpt_setup->rate);

    evt_tm_cfg.mode =  MODE_PERIODIC;
    evt_tm_cfg.arg = &clockevent_gptimer;
    evt_tm_cfg.cb = gptimer_interrupt_cb;

    iproc_timer_config(gpt_evt, &evt_tm_cfg);

    gptimer_set_next_event((CLOCK_TICK_RATE / HZ), NULL);

    /* 
     * IMPORTANT
     * Note that we don't want to waste a channel for clock source. In iProc
     * timer module by default there is a counter that keeps counting
     * irrespective of the channels. So instead of implementing a periodic
     * timer using a channel (which in the HW is not peridoic) we can
     * simply read the counters of the timer that is used for event and
     * send it for source. The only catch is that this timer should not be
     * stopped by PM or any other sub-systems.
     */
     gpt_src = gpt_evt;

    /* Initialize the profile timer */

    return ;
}

void __init iproc_timer_init(struct gp_timer_setup *gpt_setup)
{
    timers_init(gpt_setup);
    gptimer_clocksource_init();
    gptimer_clockevents_init();
    gptimer_set_next_event((CLOCK_TICK_RATE / HZ), NULL);
#ifdef CONFIG_LOCAL_TIMERS
        twd_base = IOMEM(IPROC_PERIPH_PVT_TIM_REG_VA);
#endif

}


/* Profile timer implementations */

/* 
 * TODO: The below profile timer code is retained as it is.
 * The clock manager is not up yet, once its ready read the 
 * correct frequency from it.  
 *
 * Right now Global timer runs at 5000000 on FPGA (A9 PERIPHCLK)
 * Ideally, this should be derived by timer.prof_clk and
 * prescaler.
 */

#define GLOBAL_TIMER_FREQ_HZ    (351875) /* For FPGA only, (temp)*/
//#define GLOBAL_TIMER_FREQ_HZ    (500000) /* For FPGA only, (temp)*/
timer_tick_rate_t timer_get_tick_rate(void)
{
    uint32_t prescaler;

    prescaler = readl(IPROC_PERIPH_GLB_TIM_REG_BASE);
    prescaler &= IPROC_GLB_TIM_CTRL_PRESC_MASK;
    //prescaler >>= IPROC_GLB_TIM_CTRL_PRESC_SHIFT;

    return (GLOBAL_TIMER_FREQ_HZ / (1 + prescaler)); 
}

timer_tick_count_t timer_get_tick_count(void)
{
    uint32_t msw, lsw;
    uint64_t tick;

    profile_timer_get_counter(proftmr_regbase, &msw, &lsw);

    tick = (((uint64_t)msw << 32) | ((uint64_t)lsw));

    return (*(uint32_t *)(&tick));
}

timer_msec_t timer_ticks_to_msec(timer_tick_count_t ticks)
{
    return (ticks / (timer_get_tick_rate() / 1000));
}

timer_msec_t timer_get_msec(void)
{
    return timer_ticks_to_msec(timer_get_tick_count());
}

EXPORT_SYMBOL(timer_get_tick_count);
EXPORT_SYMBOL(timer_ticks_to_msec);
EXPORT_SYMBOL(timer_get_tick_rate);
EXPORT_SYMBOL(timer_get_msec);
