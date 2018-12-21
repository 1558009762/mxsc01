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
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
#include <linux/export.h>
#include <linux/module.h>
#endif

#include "iproc_gsio.h"

struct bcm5301x_spi {
    struct workqueue_struct *workqueue;
    struct work_struct work;
    spinlock_t lock;	/* protect 'queue' */

    struct platform_device *pdev;

    u8 stopping;
    struct list_head queue;
    struct spi_master	*master;
};
/* the spi->mode bits understood by this driver: */
#define MODEBITS (0) /* support SPI mode 0 */

/*
 * Transfer the spi message data to be written into Keystone's SPI FIFO.
 */
static int bcm5301x_spi_write(uint8_t *tx_buf, int len)
{

    return cca_spi_write(tx_buf, len);
}

/*
 * Transfer the spi message data to be read from Keystone's SPI FIFO.
 */
static int bcm5301x_spi_read(uint8_t *rx_buf, int len)
{

    return cca_spi_read(rx_buf, len);
}



static int bcm5301x_spi_setup_transfer(struct spi_device *spi,
		struct spi_transfer *t)
{

    struct bcm5301x_spi *bcm5301x_spi;
    unsigned long bus_hz;
    int ret;

    bcm5301x_spi = spi_master_get_devdata(spi->master);

    if (bcm5301x_spi->stopping)
        return -ESHUTDOWN;

    if (spi->chip_select > spi->master->num_chipselect) {
        dev_dbg(&spi->dev, "setup: invalid chipselect %u (%u defined)\n",
            spi->chip_select, spi->master->num_chipselect);
        return -EINVAL;
    }

    if (spi->mode & ~MODEBITS) {
        dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
            spi->mode & ~MODEBITS);
        return -EINVAL;
    }


    if (t != NULL) {
        bus_hz = t->speed_hz;
    } else {
        bus_hz = spi->max_speed_hz;
    }
    ret = cca_spi_freq_set(bus_hz);
    if (ret < 0) {
        dev_err(&spi->dev, "setup - cannot setup spi_device\n");
        return -EINVAL;
    }
       
    dev_dbg(&spi->dev,
        "setup: %lu Hz mode 0x%x -> %d\n",
        bus_hz, spi->mode, spi->chip_select);

    return 0;

}

static void bcm5301x_spi_work(struct work_struct *work)
{

    struct bcm5301x_spi *bcm5301x_spi =
    	container_of(work, struct bcm5301x_spi, work);

    spin_lock_irq(&bcm5301x_spi->lock);
    while (!list_empty(&bcm5301x_spi->queue)) {
        struct spi_message *msg;
        struct spi_device *spi;
        struct spi_transfer *t;
        unsigned count;
        int status, rv;
        u32 len;
        
        msg = list_entry(bcm5301x_spi->queue.prev, struct spi_message, queue);

        list_del_init(&msg->queue);
        spin_unlock_irq(&bcm5301x_spi->lock);

        spi = msg->spi;
        count=0;
        status = 0;

        status = bcm5301x_spi_setup_transfer(spi, NULL);
        if (status < 0){
            status = -EINVAL;            
            goto msg_done;
        }

        /* CS is asserted in cca_spi_enable() in each transfer */

        list_for_each_entry(t, &msg->transfers, transfer_list) {
            if (t->tx_buf == NULL && t->rx_buf == NULL && t->len) {
            	status = -EINVAL;
            	break;
            }

            if (t->speed_hz){

                status = bcm5301x_spi_setup_transfer(spi, t);
                if (status < 0) {
                    break;
                }
            }            
            len = t->len;
            /* Do the write operation through Keystone's SPI bus */
            if (t->tx_buf) {
                rv = bcm5301x_spi_write((uint8_t *)t->tx_buf, (int)len);
                if (rv < 0) {
                    status = -EINVAL;
					cca_spi_cs_release(); /* CS */
                    break;
                }
                count += len;
            }

            /* Do the read operation through Keystone's SPI bus */
            if (t->rx_buf) {
                rv = bcm5301x_spi_read((uint8_t *)t->rx_buf, (int)len);
                if (rv < 0) {
                    status = -EINVAL;
					cca_spi_cs_release(); /* CS */
                    break;
                }
                count += len;
            }

            if (t->delay_usecs) {
                udelay(t->delay_usecs);
            }
            msg->actual_length += count;      
        }

        rv = cca_spi_cs_release(); /* CS */
        if (rv < 0) {
            status = -EINVAL;           
        }
msg_done:
        msg->status = status;
        msg->complete(msg->context);

        spin_lock_irq(&bcm5301x_spi->lock);
    }

    spin_unlock_irq(&bcm5301x_spi->lock);
}


static int bcm5301x_spi_setup(struct spi_device *spi)
{
    struct bcm5301x_spi *bcm5301x_spi;
    unsigned long bus_hz;
    int ret;

    bcm5301x_spi = spi_master_get_devdata(spi->master);

    if (bcm5301x_spi->stopping)
        return -ESHUTDOWN;

    if (spi->chip_select > spi->master->num_chipselect) {
        dev_dbg(&spi->dev, "setup: invalid chipselect %u (%u defined)\n",
            spi->chip_select, spi->master->num_chipselect);
        return -EINVAL;
    }

    if (spi->mode & ~MODEBITS) {
        dev_dbg(&spi->dev, "setup: unsupported mode bits %x\n",
            spi->mode & ~MODEBITS);
        return -EINVAL;
    }

    bus_hz = spi->max_speed_hz;

    ret = cca_spi_freq_set(bus_hz);
    if (ret < 0) {
        dev_err(&spi->dev, "setup - cannot setup spi_device\n");
        return -EINVAL;
    }
       
    dev_dbg(&spi->dev,
        "setup: %lu Hz\n",
        bus_hz);

    return 0;
}

static int bcm5301x_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
    struct bcm5301x_spi *bcm5301x_spi;
    struct spi_transfer *xfer;
    unsigned long flags;


    bcm5301x_spi = spi_master_get_devdata(spi->master);

    if (unlikely(list_empty(&msg->transfers) || !spi->max_speed_hz)) {
        return -EINVAL;
    }

    if (bcm5301x_spi->stopping) {
        return -ESHUTDOWN;
    }

    /* check each transfer's parameters */
    list_for_each_entry(xfer, &msg->transfers, transfer_list) {
        if (!(xfer->tx_buf || xfer->rx_buf) && xfer->len) {
            dev_dbg(&spi->dev, "missing rx or tx buf\n");
            goto msg_rejected;
        }

    }

    spin_lock_irqsave(&bcm5301x_spi->lock, flags);
    list_add_tail(&msg->queue, &bcm5301x_spi->queue);
    queue_work(bcm5301x_spi->workqueue, &bcm5301x_spi->work);
    spin_unlock_irqrestore(&bcm5301x_spi->lock, flags);
    return 0;

msg_rejected:
    /* Message rejected and not queued */
    msg->status = -EINVAL;
    if (msg->complete)
        msg->complete(msg->context);
    return -EINVAL;
}

static void bcm5301x_spi_cleanup(struct spi_device *spi)
{
    kfree(spi_get_ctldata(spi));
}

/*-------------------------------------------------------------------------*/

static int __init bcm5301x_spi_probe(struct platform_device *pdev)
{
/*    struct resource *regs; */
    struct spi_master *master;
    struct bcm5301x_spi *bcm5301x_spi;
    int ret;


    /* setup spi core then bcm5301x-specific driver state */
    ret = -ENOMEM;
    master = spi_alloc_master(&pdev->dev, sizeof(*bcm5301x_spi));
    if (!master) {
        dev_err(&pdev->dev, "probe - cannot alloc spi_master\n");
        goto out_free;
    }

    master->bus_num = 0;

    master->num_chipselect = CCA_SPI_NUM_DEV;
    master->setup = bcm5301x_spi_setup;
    master->transfer = bcm5301x_spi_transfer;
    master->cleanup = bcm5301x_spi_cleanup;
    platform_set_drvdata(pdev, master);

    bcm5301x_spi = spi_master_get_devdata(master);

    bcm5301x_spi->master = master;
    INIT_WORK(&bcm5301x_spi->work, bcm5301x_spi_work);
    bcm5301x_spi->workqueue = create_singlethread_workqueue(
        pdev->name);

    if (bcm5301x_spi->workqueue == NULL) {
        ret = -EBUSY;
        goto out_free;
    }

    spin_lock_init(&bcm5301x_spi->lock);
    INIT_LIST_HEAD(&bcm5301x_spi->queue);
    bcm5301x_spi->pdev = pdev;

    /* Initialize the hardware */
    ret = cca_spi_init();
    if (ret < 0) {
        ret = -EINVAL;
        goto out_free_queue;
    }


    ret = spi_register_master(master);
    if (ret) {
        dev_err(&pdev->dev, "Failed to register SPI master\n");
        goto out_free_queue;
    }

    return 0;
out_free_queue:
    destroy_workqueue(bcm5301x_spi->workqueue);
out_free:
    spi_master_put(master);
    return ret;
}

static int __devexit bcm5301x_spi_remove(struct platform_device *pdev)
{
    struct spi_master *master = platform_get_drvdata(pdev);
    struct bcm5301x_spi *bcm5301x_spi = spi_master_get_devdata(master);

    /* reset the hardware and block queue progress */
    spin_lock_irq(&bcm5301x_spi->lock);
    bcm5301x_spi->stopping = 1;
    spin_unlock_irq(&bcm5301x_spi->lock);
    flush_workqueue(bcm5301x_spi->workqueue);
    destroy_workqueue(bcm5301x_spi->workqueue);

    spi_unregister_master(master);

    return 0;
}

#define bcm5301x_spi_suspend NULL
#define bcm5301x_spi_resume NULL

static struct platform_driver bcm5301x_spi_driver = {
    .driver = {
        .name = "bcm5301x_spi",
        .owner = THIS_MODULE,
    },
    .probe = bcm5301x_spi_probe,
    .remove = __devexit_p(bcm5301x_spi_remove),
    .suspend	= bcm5301x_spi_suspend,
    .resume = bcm5301x_spi_resume,
};

static struct platform_device board_gspi_device = {
	.name		=	"bcm5301x_spi",
	.id		    =	-1,
};

static char banner[] __initdata = KERN_INFO "Broadcom IPROC GSIO-SPI Driver\n";
static int __init bcm5301x_spi_init(void)
{
	int r;

    printk(banner);
    r = platform_driver_register(&bcm5301x_spi_driver);
	if (r) {
	    return r;
	}

    r = platform_device_register(&board_gspi_device);
	if (r) {
	    platform_driver_unregister(&bcm5301x_spi_driver);
	}

    return r;
}
module_init(bcm5301x_spi_init);

static void __exit bcm5301x_spi_exit(void)
{
    cca_spi_exit();
	
    platform_device_unregister(&board_gspi_device);

    platform_driver_unregister(&bcm5301x_spi_driver);
}
module_exit( bcm5301x_spi_exit);

MODULE_DESCRIPTION("Bcm5301x SPI Controller driver");
MODULE_ALIAS("platform:bcm5301x_spi");
MODULE_LICENSE("GPL");
