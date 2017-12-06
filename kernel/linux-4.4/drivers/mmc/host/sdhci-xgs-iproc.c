/*
 * drivers/mmc/host/sdhci-iproc.c - Broadcom IPROC SDHCI Platform driver
 *
 * Copyright (C) 2014-2016, Broadcom Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include "sdhci.h"

struct sdhci_xgs_iproc_data {
    struct sdhci_host *host;
    struct clk *clk;
    unsigned host_num;
};

struct xgs_iproc_sdhci_host {
    struct sdhci_host host;
    u32 shadow_cmd;
    u32 shadow_blk;
};

static inline void
iproc_sdhci_writel(struct sdhci_host *host, u32 val, int reg)
{
    writel(val, host->ioaddr + reg);
}

static inline u32
iproc_sdhci_readl(struct sdhci_host *host, int reg)
{
    return readl(host->ioaddr + reg);
}

static void
iproc_sdhci_writew(struct sdhci_host *host, u16 val, int reg)
{
    struct xgs_iproc_sdhci_host *iproc_host = (struct xgs_iproc_sdhci_host *)host;
    u32 oldval, newval;
    u32 word_num = (reg >> 1) & 1;
    u32 word_shift = word_num * 16;
    u32 mask = 0xffff << word_shift;

    if (reg == SDHCI_COMMAND) {
        if (iproc_host->shadow_blk != 0) {
            iproc_sdhci_writel(host, iproc_host->shadow_blk, SDHCI_BLOCK_SIZE);
            iproc_host->shadow_blk = 0;
        }
        oldval = iproc_host->shadow_cmd;
    } else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
        oldval = iproc_host->shadow_blk;
    } else {
        oldval = iproc_sdhci_readl(host, reg & ~3);
    }
    newval = (oldval & ~mask) | (val << word_shift);

    if (reg == SDHCI_TRANSFER_MODE) {
        iproc_host->shadow_cmd = newval;
    } else if (reg == SDHCI_BLOCK_SIZE || reg == SDHCI_BLOCK_COUNT) {
        iproc_host->shadow_blk = newval;
    } else {
        iproc_sdhci_writel(host, newval, reg & ~3);
    }
}

static u16
iproc_sdhci_readw(struct sdhci_host *host, int reg)
{
    u32 val, word;
    u32 word_num = (reg >> 1) & 1;
    u32 word_shift = word_num * 16;

    val = iproc_sdhci_readl(host, (reg & ~3));
	word = (val >> word_shift) & 0xffff;
    return word;
}


static void
iproc_sdhci_writeb(struct sdhci_host *host, u8 val, int reg)
{
    u32 oldval, newval;
    u32 byte_num = reg & 3;
    u32 byte_shift = byte_num * 8;
    u32 mask = 0xff << byte_shift;

    oldval = iproc_sdhci_readl(host, reg & ~3);
    newval = (oldval & ~mask) | (val << byte_shift);

    iproc_sdhci_writel(host, newval, reg & ~3);
}

static u8
iproc_sdhci_readb(struct sdhci_host *host, int reg)
{
    u32 val, byte;
    u32 byte_num = reg & 3;
    u32 byte_shift = byte_num * 8;

    val = iproc_sdhci_readl(host, (reg & ~3));
    byte = (val >> byte_shift) & 0xff;
    return byte;
}

static u32
iproc_sdhci_get_max_clock(struct sdhci_host *host)
{
    unsigned long max_clock;

    max_clock = (host->caps & SDHCI_CLOCK_V3_BASE_MASK)
                >> SDHCI_CLOCK_BASE_SHIFT;
    max_clock *= 1000000;

    return max_clock;
}

static u32
iproc_sdhci_get_min_clock(struct sdhci_host *host)
{
    return (host->max_clk / SDHCI_MAX_DIV_SPEC_300);
}

static int
iproc_sdhci_execute_tuning(struct sdhci_host *host, u32 opcode)
{
    /*
     * Tuning is unnecessary for SDR50 and DDR50; moreover, the IPROC platform
     * doesn't support SDR104, HS200 and Hs400 cards. So, we needn't do anything
     * for tuning.
     */
    return 0;
}

static void
iproc_sdhci_set_clock(struct sdhci_host *host, unsigned int clock)
{
    /*
     * WAR that IPROC SD/MMC host need to set the driver strength
     * to TYPE_A in 3.3v DS/HS mode even if the driver strength is
     * meaningless for 3.3V signaling.
     */
    if ((host->timing == MMC_TIMING_LEGACY) ||
        (host->timing == MMC_TIMING_MMC_HS) ||
        (host->timing == MMC_TIMING_SD_HS)) {
        host->mmc->ios.drv_type = MMC_SET_DRIVER_TYPE_A;
    }

    sdhci_set_clock(host, clock);
}

static struct sdhci_ops sdhci_xgs_iproc_ops = {
#ifdef CONFIG_MMC_SDHCI_IO_ACCESSORS
    .write_l = iproc_sdhci_writel,
    .write_w = iproc_sdhci_writew,
    .write_b = iproc_sdhci_writeb,
    .read_l = iproc_sdhci_readl,
    .read_w = iproc_sdhci_readw,
    .read_b = iproc_sdhci_readb,
#else
#error The iproc SDHCI driver needs CONFIG_MMC_SDHCI_IO_ACCESSORS to be set
#endif
    .reset = sdhci_reset,
	.set_bus_width = sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
    .set_clock = iproc_sdhci_set_clock,
    .get_max_clock = iproc_sdhci_get_max_clock,
    .get_min_clock = iproc_sdhci_get_min_clock,
	.platform_execute_tuning = iproc_sdhci_execute_tuning,
};

static int
sdhci_xgs_iproc_probe(struct platform_device *pdev)
{
    struct sdhci_host *host;
    struct sdhci_xgs_iproc_data *data;
    struct device_node *np = pdev->dev.of_node;
    int ret = 0;

    /* allocate SDHCI host + platform data memory */
    host = sdhci_alloc_host(&pdev->dev, sizeof(struct sdhci_xgs_iproc_data));
    if (IS_ERR(host)) {
        printk(KERN_ERR "SDIO%d: Unable to allocate SDHCI host\n", pdev->id);
        return PTR_ERR(host);
    }

    /* set up data structure */
    data = sdhci_priv(host);
    data->host = host;
    data->host_num = pdev->id;
    host->hw_name = "IPROC-SDIO";
    host->ops = &sdhci_xgs_iproc_ops;
    host->mmc->caps = MMC_CAP_8_BIT_DATA;
    host->quirks = SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
                   SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12;

    host->irq = (unsigned int)irq_of_parse_and_map(np, 0);
    host->ioaddr = (void *)of_iomap(np, 0);
    if (!host->ioaddr) {
        printk(KERN_ERR "SDIO%d: Unable to iomap SDIO registers\n", pdev->id);
        ret = -ENXIO;
        goto err_free_host;
    }

    platform_set_drvdata(pdev, data);

    ret = sdhci_add_host(host);
    if (ret) {
        printk(KERN_ERR "SDIO%d: Failed to add SDHCI host\n", pdev->id);
        goto err_iounmap;
    }

    return ret;

err_iounmap:
    iounmap(host->ioaddr);

err_free_host:
    sdhci_free_host(host);

    return ret;
}

static int __exit
sdhci_xgs_iproc_remove(struct platform_device *pdev)
{
    struct sdhci_xgs_iproc_data *data = platform_get_drvdata(pdev);
    struct sdhci_host *host = data->host;

    sdhci_remove_host(host, 0);
    platform_set_drvdata(pdev, NULL);
    iounmap(host->ioaddr);
    sdhci_free_host(host);
    release_mem_region(pdev->resource[0].start,
                       pdev->resource[0].end - pdev->resource[0].start + 1);
    return 0;
}

#ifdef CONFIG_PM
static int
sdhci_xgs_iproc_suspend(struct platform_device *pdev, pm_message_t state)
{
    int ret = 0;
    struct sdhci_xgs_iproc_data *data = platform_get_drvdata(pdev);

    ret = sdhci_suspend_host(data->host);
    if (ret < 0) {
        printk("%s: %d\n", __FILE__, __LINE__);
        return ret;
    }

    return 0;
}

static int
sdhci_xgs_iproc_resume(struct platform_device *pdev)
{
    int ret = 0;
    struct sdhci_xgs_iproc_data *data = platform_get_drvdata(pdev);

    ret = sdhci_resume_host(data->host);
    if (ret < 0) {
        printk("%s: %d\n", __FILE__, __LINE__);
        return ret;
    }
    return 0;
}
#else /* CONFIG_PM */

#define sdhci_xgs_iproc_suspend NULL
#define sdhci_xgs_iproc_resume NULL

#endif /* CONFIG_PM */


static const struct of_device_id brcm_iproc_dt_ids[] = {
	{ .compatible = "brcm,iproc-sdio"},
	{ }
};
MODULE_DEVICE_TABLE(of, brcm_iproc_dt_ids);

static struct platform_driver sdhci_xgs_iproc_driver = {
    .probe = sdhci_xgs_iproc_probe,
    .remove = __exit_p(sdhci_xgs_iproc_remove),
    .suspend = sdhci_xgs_iproc_suspend,
    .resume = sdhci_xgs_iproc_resume,
    .driver = {
        .name = "iproc-sdio",
        .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(brcm_iproc_dt_ids),
    },
};

module_platform_driver(sdhci_xgs_iproc_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("SDHCI XGS IPROC driver");
MODULE_LICENSE("GPL");
