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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/serial_8250.h>

#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/page.h>

#include <asm/mach/map.h>
#include <asm/pgalloc.h>

#include <linux/serial.h>
#include <linux/serial_core.h>

#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/io_map.h>

#define IO_DESC(va, sz) {                           \
    .virtual = va,                                  \
    .pfn = __phys_to_pfn(HW_IO_VIRT_TO_PHYS(va)),   \
    .length = sz,                                   \
    .type = MT_DEVICE                               \
}

static struct map_desc iproc_io_desc[] __initdata =
{
	IO_DESC(IO_CORE_IDM_VA,IO_CORE_IDM_SIZE),
	IO_DESC(IO_ARMCORE_VA, IO_ARMCORE_SIZE),
#if defined(CONFIG_MACH_IPROC_P7)
	IO_DESC(IO_SMAU_IDM_VA, IO_SMAU_IDM_SIZE),
#endif /* !CONFIG_MACH_IPROC_P7 */
};

void __init iproc_map_io(void)
{
	iotable_init(iproc_io_desc, ARRAY_SIZE(iproc_io_desc));
}
