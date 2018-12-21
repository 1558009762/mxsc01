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
 
/*
 * PMU device description to Iproc
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <typedefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <mach/reg_utils.h>
#include <asm/pmu.h>
#include <mach/io_map.h>

static struct resource iproc_pmu_resource = {
    .start  = BCM_INT_ID_IHOST_PMU,
    .end    = BCM_INT_ID_IHOST_PMU+1,
    .flags  = IORESOURCE_IRQ,
};


static struct platform_device iproc_pmu_device = {
	.name		=	"arm-pmu",
	.id		    =	ARM_PMU_DEVICE_CPU,
	.dev =  {
		.init_name = "arm-pmu",
	},
        .num_resources  = 1,
        .resource = &iproc_pmu_resource,
};


static int __init iproc_pmu_init(void)
{
    int ret;
    printk(KERN_INFO "Registering iproc_pmu_device\n");
    ret = platform_device_register(&iproc_pmu_device);
    return ret;
}
module_init(iproc_pmu_init);


/* Module information */
MODULE_DESCRIPTION("IPROC PMU Driver");
MODULE_LICENSE("GPL");
