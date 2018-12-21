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
 * DESCRIPTION: The BCM IPROC random number generator (RNG) driver
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/hw_random.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include "iproc_hwrng.h"

DEFINE_MUTEX(lock); /* lock for data access */

static atomic_t bus_is_probed;

static int rng_data_present(struct hwrng *rng, int wait)
{
    int data, i;

    for (i = 0; i < 20; i++) {
        data = bcm5301x_rng_get_valid_words() ? 1 : 0;
        if (data || !wait)
            break;
        /*
         * RNG produces data fast enough.  We *could* use the RNG IRQ, but
         * that'd be higher overhead ... so why bother?
         */
        udelay(10);
    }

    return data;
}

int rng_data_read(struct hwrng *rng, u32 *data)
{
    if (!atomic_read(&bus_is_probed))
        return -ENODEV;

    /* lock it here since other kernel drivers can access it */
    mutex_lock(&lock);
    *data = bcm5301x_rng_get_random_number();
    mutex_unlock(&lock);
    return 4;
}
EXPORT_SYMBOL(rng_data_read);

static struct hwrng rng_ops = {
    .name = "bcmns",
    .data_present = rng_data_present,
    .data_read = rng_data_read,
};

static int __init rng_probe(struct platform_device *pdev)
{
//    struct resource *res;
    int ret;

    /* We only accept one device, and it must have an id of -1 */
    if (pdev->id != -1)
        return -ENODEV;

    atomic_set(&bus_is_probed, 0);

    bcm5301x_rng_start();

    /* register to the Linux RNG framework */
    ret = hwrng_register(&rng_ops);
    if (ret)
        goto err_out;

    atomic_set(&bus_is_probed, 1);

    return 0;

err_out:
    return ret;
}

static int __devexit rng_remove(struct platform_device *pdev)
{
    atomic_set(&bus_is_probed, 0);
    hwrng_unregister(&rng_ops);
    return 0;
}

static int rng_suspend(struct platform_device *dev, pm_message_t msg)
{
    bcm5301x_rng_disable();
    return 0;
}

static int rng_resume(struct platform_device *dev)
{
    bcm5301x_rng_enable();
    return 0;
}

static struct platform_driver rng_driver = {
    .driver = {
        .name = "bcmns-rng",
        .owner = THIS_MODULE,
    },
    .suspend   = rng_suspend,
    .resume    = rng_resume,
    .remove    = __devexit_p(rng_remove),
    .probe     = rng_probe,
};

static struct platform_device board_rng_device = {
	.name		=	"bcmns-rng",
	.id		    =	-1,
};

static char banner[] __initdata = KERN_INFO "Broadcom IPROC RNG Driver\n";

static int __init rng_init(void)
{
	int r;

    printk(banner);

    r = platform_driver_register(&rng_driver);
    if (r) {
        return r;
    }

    r = platform_device_register(&board_rng_device);
    if (r) {
        platform_driver_unregister(&rng_driver);
    }
    return r;
}

static void __exit rng_exit(void)
{
    bcm5301x_rng_exit();
	
    platform_device_unregister(&board_rng_device);

    platform_driver_unregister(&rng_driver);
}

module_init(rng_init);
module_exit(rng_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("RNG Device Driver");
MODULE_LICENSE("GPL");
