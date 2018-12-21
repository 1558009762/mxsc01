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
#ifndef _OSDAL_OS_H_
#define _OSDAL_OS_H_

#include <linux/kernel.h>
#include "asm/posix_types.h"
#include "asm/string.h"
/* Heap memory */
#define OSDAL_ALLOCHEAPMEM(s)	kzalloc((s), GFP_KERNEL)

#define OSDAL_FREEHEAPMEM(a)	kfree((a))


/* IRQ */
#define OSDAL_IRQ_Enable(irq)    enable_irq((irq))
#define OSDAL_IRQ_Disable(irq)   disable_irq((irq))
#define OSDAL_IRQ_Clear(irq)
#define OSDAL_IRQ_IsEnabled(irq)


/*  Synchronization */
#define OSDAL_SENDEVENT(e) complete((struct completion *)&(e))

#define OSDAL_WAITEVENT(e)	\
wait_for_completion((struct completion *)&(e))

#define OSDAL_WAITEVENT_TIMEOUT(e, t) \
wait_for_completion_timeout((struct completion *)&(e), (t))

#define OSDAL_CLEAREVENT(e)


/* Time stamp in ms */
#define  OSDAL_TIMEVAL()


/* Delays */
#define OSDAL_MDELAY(x)	mdelay(x)

#define OSDAL_UDELAY(x)	udelay(x)

/* Debug Print */
//#define dprintf(prio, format, args...)	pr_info("%s:%s"" format", __FILE__, __FUNCTION__)
#define dprintf(prio, fmt, ...) printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)

#endif /*_OSDAL_OS_H_*/
