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
 * This serves as the SDHCI platform driver (for Northstar Soc) that
 * talks to the lower level SDHCI driver
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/io.h>
#include <mach/memory.h>
#include <mach/iproc_regs.h>
#include <mach/io_map.h>
#include "sdhci.h"

#define DRIVER_NAME "iproc_sdhci"

#define SDHCI_MAX_DIV_SPEC_300	2046
#define SDHCI_DIVIDER_SHIFT	8
#define SDHCI_DIVIDER_HI_SHIFT	6
#define SDHCI_DIV_MASK	0xFF
#define SDHCI_DIV_MASK_LEN	8
#define SDHCI_DIV_HI_MASK	0x300
#define SDHCI_CLOCK_V3_BASE_MASK	0x0000FF00

struct sdhci_platform_data {
    struct sdhci_host *host;
    struct clk *clk;
    unsigned host_num;
};

u32 sdhci_iproc_readl(struct sdhci_host *host, int reg)
{
	u32 l = readl(host->ioaddr + reg);

#ifdef LOG_REGISTERS
	printk(KERN_ERR "%s: readl from 0x%02x, value 0x%08x\n",
               mmc_hostname(host->mmc), reg, l);
#endif

	return l;
}

u16 sdhci_iproc_readw(struct sdhci_host *host, int reg)
{
	u32 l = readl(host->ioaddr + (reg & ~3));
	u32 w = l >> (reg << 3 & 0x18) & 0xffff; 

#ifdef LOG_REGISTERS
	printk(KERN_ERR "%s: readw from 0x%02x, value 0x%04x\n",
               mmc_hostname(host->mmc), reg, w);
#endif

	return (u16)w;
}

u8 sdhci_iproc_readb(struct sdhci_host *host, int reg)
{
	u32 l = readl(host->ioaddr + (reg & ~3));
	u32 b = l >> (reg << 3 & 0x18) & 0xff;

#ifdef LOG_REGISTERS
	printk(KERN_ERR "%s: readb from 0x%02x, value 0x%02x\n",
               mmc_hostname(host->mmc), reg, b);
#endif

	return (u8)b;
}

void sdhci_iproc_writel(struct sdhci_host *host, u32 val, int reg)
{
#ifdef LOG_REGISTERS
	printk(KERN_ERR "%s: writel to 0x%02x, value 0x%08x\n",
               mmc_hostname(host->mmc), reg, val);
#endif

	writel(val, host->ioaddr + reg);
}

void sdhci_iproc_writew(struct sdhci_host *host, u16 val, int reg)
{
	static u32 shadow = 0;

	u32 p = reg == SDHCI_COMMAND ? shadow : readl(host->ioaddr + (reg & ~3));
	u32 s = reg << 3 & 0x18;
	u32 l = val << s;
	u32 m = 0xffff << s;

#ifdef LOG_REGISTERS
	printk(KERN_ERR "%s: writew to 0x%02x, value 0x%04x\n",
               mmc_hostname(host->mmc), reg, val);
#endif

	if (reg == SDHCI_TRANSFER_MODE) 
		shadow = (p & ~m) | l;
	else{
		writel((p & ~m) | l, host->ioaddr + (reg & ~3));
    }		
}

void sdhci_iproc_writeb(struct sdhci_host *host, u8 val, int reg)
{
	u32 p = readl(host->ioaddr + (reg & ~3));
	u32 s = reg << 3 & 0x18;
	u32 l = val << s;
	u32 m = 0xff << s;

#ifdef LOG_REGISTERS
	printk(KERN_ERR "%s: writeb to 0x%02x, value 0x%02x\n",
               mmc_hostname(host->mmc), reg, val);
#endif

	writel((p & ~m) | l, host->ioaddr + (reg & ~3));
}

static void sdhci_iproc_set_clock(struct sdhci_host *host, unsigned int clock)
{
    int div;
    u16 clk;
    unsigned long timeout;
    
	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;

	if(clock > 100000000){
	    printk(KERN_INFO "%s :could work @ max 100MHz down the clock %d to 100MHz\n",
               mmc_hostname(host->mmc), clock);
            clock = 100000000;               
               
	}

	/* Version 3.00 divisors must be a multiple of 2. */
	if (host->max_clk <= clock)
		div = 1;
	else {
		for (div = 2; div < SDHCI_MAX_DIV_SPEC_300; div += 2) {
			if ((host->max_clk / div) <= clock)
				break;
		}
	}
	
	div >>= 1;

	clk = (div & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= ((div & SDHCI_DIV_HI_MASK) >> SDHCI_DIV_MASK_LEN)
		<< SDHCI_DIVIDER_HI_SHIFT;

	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Internal clock never "
				"stabilised.\n", mmc_hostname(host->mmc));
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

out:
	host->clock = clock;

}

/*
 * Get the base clock
 */
unsigned int sdhci_iproc_get_max_clock(struct sdhci_host *host)
{
	unsigned int caps;    
    unsigned long max_clock;
    caps = sdhci_readl(host, SDHCI_CAPABILITIES);
	max_clock = (caps & SDHCI_CLOCK_V3_BASE_MASK)
			>> SDHCI_CLOCK_BASE_SHIFT;
    max_clock *= 1000000;
    return max_clock;
}

unsigned int sdhci_iproc_get_timeout_clock(struct sdhci_host *host)
{
	return 100000;		// this value is in kHz
}

unsigned int sdhci_iproc_get_min_clock(struct sdhci_host *host)
{   
    return (host->max_clk/SDHCI_MAX_DIV_SPEC_300);
}
    
static struct sdhci_ops sdhci_platform_ops = {
#ifdef CONFIG_MMC_SDHCI_IO_ACCESSORS
	.read_l = sdhci_iproc_readl,
	.read_w = sdhci_iproc_readw,
	.read_b = sdhci_iproc_readb,
	.write_l = sdhci_iproc_writel,
	.write_w = sdhci_iproc_writew,
	.write_b = sdhci_iproc_writeb,
#else
#error The iproc SDHCI driver needs CONFIG_MMC_SDHCI_IO_ACCESSORS to be set
#endif
    .enable_dma = NULL,
    .set_clock = sdhci_iproc_set_clock,
    .get_max_clock = sdhci_iproc_get_max_clock,
    .get_min_clock = sdhci_iproc_get_min_clock,

};

static int sdhci_platform_probe(struct platform_device *pdev)
{
    struct resource *res;
    struct sdhci_host *host;
    struct sdio_platform_cfg *cfg;
    struct sdhci_platform_data *data;
    int ret = 0;
    int irq;
    u32 __maybe_unused val;
    void __iomem __maybe_unused *iomem_base;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    irq = platform_get_irq(pdev, 0);
    if (res == NULL || irq < 0) {
        printk(KERN_ERR "SDIO%d: Unable to get platform resource or IRQ\n",
          pdev->id);
        return -ENXIO;
    }

    res = request_mem_region(res->start, res->end - res->start + 1,
         pdev->name);
    if (res == NULL) {
        printk(KERN_ERR "SDIO%d: request_mem_region failed\n", pdev->id);
        return -EBUSY;
    }

    cfg = pdev->dev.platform_data;
    if (cfg == NULL) {
        printk(KERN_ERR "SDIO%d: Unable to get platform data\n",
          pdev->id);
        ret = -ENODEV;
        goto err_free_mem_region;
    }

    /* allocate SDHCI host + platform data memory */
    host = sdhci_alloc_host(&pdev->dev, sizeof(struct sdhci_platform_data));
    if (IS_ERR(host)) {
        ret = PTR_ERR(host);
        printk(KERN_ERR "SDIO%d: Unable to allocate SDHCI host\n",
          pdev->id);
        goto err_free_mem_region;
    }

    /* set up data structure */
    data = sdhci_priv(host);
    data->host = host;
    data->host_num = pdev->id;
    host->hw_name = "BCM-SDIO";
    host->quirks = 0;
    host->ops = &sdhci_platform_ops;
    host->irq = irq;

    /* map registers */
    host->ioaddr = ioremap_nocache(res->start, (res->end - res->start) + 1);
    if (!host->ioaddr) {
        printk(KERN_ERR "SDIO%d: Unable to iomap SDIO registers\n",
               pdev->id);
        ret = -ENXIO;
        goto err_free_host;
    }    

    platform_set_drvdata(pdev, data);
    
	host->quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
	               SDHCI_QUIRK_NONSTANDARD_CLOCK |
	               SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;

    ret = sdhci_add_host(host);
    if (ret) {
        printk(KERN_ERR "SDIO%d: Failed to add SDHCI host\n",
               pdev->id);
        goto err_iounmap;
    }

    return 0;

err_iounmap:
    iounmap(host->ioaddr);

err_free_host:
    sdhci_free_host(host);

err_free_mem_region:
    release_mem_region(res->start, res->end - res->start + 1);
    return ret;
}

static int __devexit sdhci_platform_remove(struct platform_device *pdev)
{
    int dead = 0;
    struct sdhci_platform_data *data = platform_get_drvdata(pdev);

    u32 scratch = readl(data->host->ioaddr + SDHCI_INT_STATUS);
    struct sdhci_host *host = data->host;


    if (scratch == (u32)-1)
      dead = 1;

    sdhci_remove_host(host, 0);
    platform_set_drvdata(pdev, NULL);
    iounmap(host->ioaddr);
    sdhci_free_host(host);
    release_mem_region(pdev->resource[0].start,
         pdev->resource[0].end - pdev->resource[0].start + 1);
    return 0;
}

#ifdef CONFIG_PM

static int sdhci_platform_suspend(struct platform_device *pdev,
      pm_message_t state)
{
   int ret=0;
   struct sdhci_platform_data *data = platform_get_drvdata(pdev);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
   ret = sdhci_suspend_host(data->host);
#else
   ret = sdhci_suspend_host(data->host, state);
#endif
   if (ret < 0) {
	    printk("%s: %d\n", __FILE__, __LINE__);
      return ret;
   }

   return 0;
}

static int sdhci_platform_resume(struct platform_device *pdev)
{
    int ret =0;
    struct sdhci_platform_data *data = platform_get_drvdata(pdev);

    ret = sdhci_resume_host(data->host);
    if (ret < 0) {
        printk("%s: %d\n", __FILE__, __LINE__);
        return ret;
    }
    return 0;
}

#else /* CONFIG_PM */

#define sdhci_platform_suspend NULL
#define sdhci_platform_resume NULL

#endif /* CONFIG_PM */

static struct platform_driver sdhci_platform_driver = {
    .probe = sdhci_platform_probe,
    .remove = __devexit_p(sdhci_platform_remove),
    .suspend = sdhci_platform_suspend,
    .resume = sdhci_platform_resume,
    .driver = {
      .name = "bcm-sdio",
      .owner = THIS_MODULE,
    },
};

static int __init sdhci_platform_init(void)
{
    int ret;

    ret = platform_driver_register(&sdhci_platform_driver);
    if (ret) {
        printk(KERN_ERR DRIVER_NAME
            ": Unable to register the SDHCI Platform driver\n");
        return ret;
    }

    return 0;
}

static void __exit sdhci_platform_exit(void)
{
    platform_driver_unregister(&sdhci_platform_driver);
}


module_init(sdhci_platform_init);
module_exit(sdhci_platform_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("SDHCI Platform driver");
MODULE_LICENSE("GPL");
