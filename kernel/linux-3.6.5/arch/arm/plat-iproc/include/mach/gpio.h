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
#ifndef __PLAT_GPIO_H
#define __PLAT_GPIO_H

#include <mach/iproc_regs.h>
#include <mach/irqs.h>

#include <asm-generic/gpio.h>


#define gpio_to_irq(gpio)           __gpio_to_irq(gpio)
//#define gpio_get_value(gpio)		__gpio_get_value(gpio)
//#define gpio_set_value(gpio,value)  __gpio_set_value(gpio,value)
//#define gpio_cansleep(gpio)		   __gpio_cansleep(gpio)

#endif
