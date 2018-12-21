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
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/mm.h>
#include <linux/clk.h>
#include "iproc_timer.h"

/* 
 * Configurations
 */
#define DRV_NAME                            "iproc_ccb_timer"
#define REFCLK_SOURCE                       "c_clk125"
#define REFCLK_SOURCE_DEVID                 "iproc_slow"
#define MAX_NUMBER_OF_TIMERS                (4)

/*
 * Register offset
 */
#define TIMER_LOAD_OFFSET                   0x00
#define TIMER_VALUE_OFFSET                  0x04
#define TIMER_CONTROL_OFFSET                0x08
#define TIMER_INTCLR_OFFSET                 0x0c
#define TIMER_RIS_OFFSET                    0x10
#define TIMER_MIS_OFFSET                    0x14

/*
 * Timer Control Register Bits
 */
#define TIMER_CTRL_16BIT                    (0 << 1) /* 16-bit counter mode */
#define TIMER_CTRL_32BIT                    (1 << 1) /* 32-bit counter mode */
#define TIMER_CTRL_IE                       (1 << 5) /* Interrupt enable */
#define TIMER_CTRL_PERIODIC                 (1 << 6) /* Periodic mode */
#define TIMER_CTRL_EN                       (1 << 7) /* Timer enable */        
#define TIMER_CTRL_ONESHOTMODE              (1 << 0)
#define TIMER_CTRL_DIV1                     (0 << 2)
#define TIMER_CTRL_PREBY16                  (1 << 2)
#define TIMER_CTRL_PREBY256                 (2 << 2)

/* In case we use physical addresses */
#define IO_ADDRESS(x)                       (x)

/* Timer instance */
typedef struct {
    int32_t             id;         /* id for this timer */
    uint32_t            base;       /* timer base address */
    uint32_t            vec;        /* interrupt vector */
    int                 started;    /* Whether it has started */
    uint64_t            interval;   /* Interval in ticks; 0 if not inited */
    int                 periodic;   /* 1 for periodic; 0 for one-shot */
    uint32_t            load;       /* Actual value to load */
    iproc_timer_isr_t   isr;        /* User interrupt handler */
    void *              cookie;     /* Cookie for user isr */
    uint32_t            prescale;   /* Prescale bits */
} timer_map_t;

/* Actual number of timers */
static int timer_count;

/* Mapping from logical timer to physical timer */
static timer_map_t timers_map[MAX_NUMBER_OF_TIMERS];

/* Reference clock */
static uint32_t timer_refclk;

/* 
 * Disable timer 
 */
static inline void 
hw_timer_disable(timer_map_t *ptimer)
{
    register uint32_t timer_base;

    timer_base = ptimer->base;
    writel(
        readl(IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET)) & ~TIMER_CTRL_EN, 
        IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET)
        );
    
    /* Clear pending interrupt */
    writel(1, IO_ADDRESS(timer_base + TIMER_INTCLR_OFFSET));
}

/*
 * Configure timer
 */
int 
iproc_timer_configure(int timer_id, int periodic, uint64_t interval, 
                  iproc_timer_isr_t isr, void *cookie)
{
    timer_map_t *ptimer;
    uint32_t ctrl = TIMER_CTRL_32BIT;

    if (timer_id < 0 || timer_id >= timer_count || 
        interval == 0 || interval > iproc_timer_get_max_interval()) {
        return -EINVAL;
    }
    ptimer = &timers_map[timer_id];
    
    /* Cannot configure while it's started */
    if (ptimer->started) {
        return -EBUSY;
    }
    
    /* Check how much we need to prescale */
    if ((interval >> 32) == 0) {
        ptimer->prescale = 0;
        ctrl |= TIMER_CTRL_DIV1;
    } else {
        if ((interval >> 36) == 0) {
            ptimer->prescale = 4;
            ctrl |= TIMER_CTRL_PREBY16;
        } else {
            ptimer->prescale = 8;
            ctrl |= TIMER_CTRL_PREBY256;
        }
    }
    
    /* Actual value to load */
    ptimer->load = (uint32_t)(interval >> ptimer->prescale);
    
    /* Configure periodic/one-shot mode */
    ptimer->periodic = periodic;
    if (periodic) {
    
        /* 
         * For periodic mode, don't enable interrupt if user isr is not set 
         * This is to avoid frequent IRQs and degrade system performance.
         */
        if (isr) {
            ctrl |= TIMER_CTRL_IE;
        }
        ctrl |= TIMER_CTRL_PERIODIC;
        
    } else {
        /* For one-shot mode, interrupt must be enabled (to mark it stopped) */
        ctrl |= TIMER_CTRL_ONESHOTMODE | TIMER_CTRL_IE;
    }
    
    /* Write to control register */
    writel(ctrl, IO_ADDRESS(ptimer->base + TIMER_CONTROL_OFFSET));
    
    /* Record user specified arguments */
    ptimer->interval = interval;
    ptimer->isr = isr;
    ptimer->cookie = cookie;
    
    return 0;
}

/* 
 * Start timer
 */ 
int 
iproc_timer_start(int timer_id)
{
    timer_map_t *ptimer;
    register uint32_t timer_base;

    if (timer_id < 0 || timer_id >= timer_count) {
        return -EINVAL;
    }
    ptimer = &timers_map[timer_id];
    
    if (ptimer->interval == 0) {
        return -EPERM;
    }
    
    if (ptimer->started) {
        return -EBUSY;
    }
    
    ptimer->started = 1;
    timer_base = ptimer->base;
    writel(ptimer->load, IO_ADDRESS(timer_base + TIMER_LOAD_OFFSET));
    writel(
        readl(IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET)) | TIMER_CTRL_EN, 
        IO_ADDRESS(timer_base + TIMER_CONTROL_OFFSET)
        );
    
    return 0;
}

/* 
 * Stop timer
 */ 
int 
iproc_timer_stop(int timer_id)
{
    timer_map_t *ptimer;

    if (timer_id < 0 || timer_id >= timer_count) {
        return -EINVAL;
    }
    ptimer = &timers_map[timer_id];
    
    if (ptimer->started == 0) {
        return 0;
    }
    
    ptimer->started = 0;
    hw_timer_disable(ptimer);
    
    return 0;
}

/* 
 * Returns timer's counter 
 */
uint64_t 
iproc_timer_get_current_ticks(int timer_id)
{
    timer_map_t *ptimer;
    uint64_t ticks;
    
    if (timer_id < 0 || timer_id >= timer_count) {
        return -EINVAL;
    }
    ptimer = &timers_map[timer_id];
    if (!ptimer->interval || !ptimer->started) {
        return 0;
    }
    
    /* To return the elapsed ticks, not remaining ticks */
    ticks = 
        ptimer->load - readl(IO_ADDRESS(ptimer->base  + TIMER_VALUE_OFFSET));

    /* Scale back */
    return ticks << ptimer->prescale;
}

/*
 * Timer info
 */
int 
iproc_timer_get_info(int timer_id, iproc_timer_info_t *info)
{
    timer_map_t *ptimer;
    
    if (timer_id < 0 || timer_id >= timer_count || info == NULL) {
        return -EINVAL;
    }
    ptimer = &timers_map[timer_id];
    info->configured = ptimer->interval? 1 : 0;
    info->started = ptimer->started;
    info->periodic = ptimer->periodic;
    info->interval = ptimer->interval;
    info->isr = ptimer->isr;
    info->cookie = ptimer->cookie;
    
    return 0;
}

/*
 * Ticking rate (reference clock frequency): ticks per second
 */
uint32_t
iproc_timer_get_ticking_rate(void)
{
    return timer_refclk;
}

/*
 * Get max interval in ticks
 */
uint64_t
iproc_timer_get_max_interval(void)
{
    return (uint64_t)0xFFFFFFFFULL << 8;
}

/*
 * Get number of timers
 */
uint32_t
iproc_timer_count(void)
{
    return timer_count;
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t 
hw_timer_interrupt(int irq, void *dev_id)
{
    timer_map_t *ptimer = (timer_map_t *)dev_id;
    
    /* Check if it's for us */
    if (!(readl(IO_ADDRESS(ptimer->base + TIMER_MIS_OFFSET)) & 1)) {
        return IRQ_NONE;
    } 

    /* clear the interrupt */
    writel(1, IO_ADDRESS(ptimer->base + TIMER_INTCLR_OFFSET));
    
    /* Double confirm if it's enabled by user */
    if (ptimer->interval && ptimer->started) {
    
        /* If it's one-shot, mark it 'stopped' first */
        if (ptimer->periodic == 0) {
            ptimer->started = 0;
            
            /* It doesn't clear the EN bit automatically */
            writel(
                readl(
                    IO_ADDRESS(ptimer->base + TIMER_CONTROL_OFFSET)) & 
                    ~TIMER_CTRL_EN, 
                IO_ADDRESS(ptimer->base + TIMER_CONTROL_OFFSET)
                );
        }
    
        /* Call user specified ISR */
        if (ptimer->isr) {
            (*ptimer->isr)(ptimer->id, ptimer->cookie);
        }
    }

    return IRQ_HANDLED;
}

/***********************************************************************
 * Platform driver setup
 ***********************************************************************/

static int __devinit 
iproc_timer_probe(struct platform_device *pdev)
{
    struct resource *res;
    int i;
    int ret = 0;
    struct clk *clk;

    /* Retrieve reference clock frequency */
    clk = clk_get_sys(REFCLK_SOURCE_DEVID, REFCLK_SOURCE);
    if (!clk) {
        dev_err(&pdev->dev, "can't get reference clock frequency by %s\n", 
            REFCLK_SOURCE);
        ret = -EIO;
        goto err2;
    }
    timer_refclk = (uint32_t)clk_get_rate(clk);

    /* Retrieve IRQ from resources (also determine number of timers) */
    timer_count = 0;
    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res) {
        dev_err(&pdev->dev, "no IRQ defined\n");
        ret = -ENODEV;
        goto err2;
    }
    for(i = (int)res->start; i <= (int)res->end; i++) {
        timers_map[timer_count].vec = i;
        timer_count++;
    }

    /* Retrieve register space (in virtual addresses) from resources */
    for(i=0; i<timer_count; i++) {
        res = platform_get_resource(pdev, IORESOURCE_MEM, i);
        if (!res) {
            dev_err(&pdev->dev, "can't get resource for register space\n");
            ret = -EIO;
            goto err2;
        }
        timers_map[i].base = (uint32_t)res->start;
    }
    
    /* Perform basic initialization */
    for(i=0; i<timer_count; i++) {
        timers_map[i].id = i;
        timers_map[i].interval = 0;
        timers_map[i].started = 0;
        hw_timer_disable(&timers_map[i]);
    }
    
    /* Attach IRQ handlers */
    for(i=0; i<timer_count; i++) {
        ret = request_irq(timers_map[i].vec, hw_timer_interrupt, 0,
            DRV_NAME, &timers_map[i]);
        if (ret < 0) {
            dev_err(&pdev->dev, "unable to allocate IRQ\n");
            goto err1;
        }
    }

    printk(KERN_INFO "iProc Timer driver: %u timers running at %uHz\n", 
        timer_count, timer_refclk);

    return 0;

err1:
    for(i=0; i<timer_count; i++) {
        free_irq(timers_map[i].vec, &timers_map[i]);
    }
err2:
    return ret;
}

static int __devexit 
iproc_timer_remove(struct platform_device *pdev)
{
    int i;
    for(i=0; i<timer_count; i++) {
        hw_timer_disable(&timers_map[i]);
        free_irq(timers_map[i].vec, &timers_map[i]);
    }

    return 0;
}

static struct platform_driver iproc_timer_driver = {
    .probe            = iproc_timer_probe,
    .remove            = __devexit_p(iproc_timer_remove),
    .driver = {
        .name        = DRV_NAME,
        .owner        = THIS_MODULE,
    },
};

static int __init 
iproc_timer_init(void)
{
    int err;
    
    err = platform_driver_register(&iproc_timer_driver);
    if (err < 0) {
        printk(KERN_ERR "%s: can't register platform driver "
            "(error %d)\n", __func__, err);
        return err;
    }

    return 0;
}

static void __exit 
iproc_timer_exit(void)
{
    platform_driver_unregister(&iproc_timer_driver);
}

EXPORT_SYMBOL(iproc_timer_count);
EXPORT_SYMBOL(iproc_timer_get_ticking_rate);
EXPORT_SYMBOL(iproc_timer_get_max_interval);
EXPORT_SYMBOL(iproc_timer_configure);
EXPORT_SYMBOL(iproc_timer_start);
EXPORT_SYMBOL(iproc_timer_get_current_ticks);
EXPORT_SYMBOL(iproc_timer_stop);
EXPORT_SYMBOL(iproc_timer_get_info);

module_init(iproc_timer_init);
module_exit(iproc_timer_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("iProc Chipcommon Timer Driver");
