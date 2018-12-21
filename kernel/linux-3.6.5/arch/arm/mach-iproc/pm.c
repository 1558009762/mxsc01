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

#include <linux/pm.h>
#include <linux/suspend.h>
#include <asm/suspend.h>
#include <asm/proc-fns.h>
#include <linux/module.h>

int iproc_arm_cpu_do_idle(void)
{
	return cpu_do_idle();
}
EXPORT_SYMBOL(iproc_arm_cpu_do_idle);

int iproc_arm_cpu_suspend(unsigned long arg, int (*fn)(unsigned long))
{
	return cpu_suspend(arg, fn);
}
EXPORT_SYMBOL(iproc_arm_cpu_suspend);
