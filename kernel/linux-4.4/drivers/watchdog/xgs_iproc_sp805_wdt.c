/*
 * Copyright (C) 2015, Broadcom Corporation. All Rights Reserved.
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

#include <linux/device.h>
#include <linux/resource.h>
#include <linux/amba/bus.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <asm/pgtable.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

/* default timeout in seconds */
#define DEFAULT_TIMEOUT		60

#define MODULE_NAME		"iproc-sp805-wdt"

/* watchdog register offsets and masks */
#define WDTLOAD			0x000
	#define LOAD_MIN	0x00000001
	#define LOAD_MAX	0xFFFFFFFF
#define WDTVALUE		0x004
#define WDTCONTROL		0x008
	/* control register masks */
	#define	INT_ENABLE	(1 << 0)
	#define	RESET_ENABLE	(1 << 1)
#define WDTINTCLR		0x00C
#define WDTRIS			0x010
#define WDTMIS			0x014
	#define INT_MASK	(1 << 0)
#define WDTLOCK			0xC00
	#define	UNLOCK		0x1ACCE551
	#define	LOCK		0x00000001

/**
 * struct sp805_wdt: sp805 wdt device structure
 * @wdd: instance of struct watchdog_device
 * @lock: spin lock protecting dev structure and io access
 * @base: base address of wdt
 * @clk: clock structure of wdt
 * @adev: amba device structure of wdt
 * @load_val: load value to be set for current timeout
 */
struct sp805_wdt {
	struct watchdog_device		wdd;
	spinlock_t			lock;
	void __iomem			*base;
	struct clk			*clk;
	struct amba_device		*adev;
	unsigned int			load_val;
};

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
		"Set to 1 to keep watchdog running after device release");

/* This routine get boot status to indicate if the last boot is from WDT */
static unsigned int wdt_get_clear_bootstatus(
			void __iomem *wdt_bootstatus,
			unsigned int wdt_bootstatus_bit)
{
	unsigned int reg;
	unsigned int bootstatus = 0;

	reg = readl_relaxed(wdt_bootstatus);
	bootstatus = reg & (1 << wdt_bootstatus_bit);

	if (bootstatus) {
		/* write 1 to clear boot status bit */
		writel_relaxed(reg, wdt_bootstatus);
#if defined(CONFIG_ARCH_BCM_NSP)
		writel_relaxed(reg, wdt_bootstatus);
#endif
	}

	return bootstatus;
}

/* This routine finds load value that will reset system in required timout */
static int wdt_setload(struct watchdog_device *wdd, unsigned int timeout)
{
	struct sp805_wdt *wdt = watchdog_get_drvdata(wdd);
	u64 load, rate;

	rate = clk_get_rate(wdt->clk);

	/*
	 * sp805 runs counter with given value twice, after the end of first
	 * counter it gives an interrupt and then starts counter again. If
	 * interrupt already occurred then it resets the system. This is why
	 * load is half of what should be required.
	 */
	load = div_u64(rate, 2) * timeout - 1;

	load = (load > LOAD_MAX) ? LOAD_MAX : load;
	load = (load < LOAD_MIN) ? LOAD_MIN : load;

	spin_lock(&wdt->lock);
	wdt->load_val = load;
	/* roundup timeout to closest positive integer value */
	wdd->timeout = div_u64((load + 1) * 2 + (rate / 2), rate);
	spin_unlock(&wdt->lock);

	return 0;
}

/* returns number of seconds left for reset to occur */
static unsigned int wdt_timeleft(struct watchdog_device *wdd)
{
	struct sp805_wdt *wdt = watchdog_get_drvdata(wdd);
	u64 load, rate;

	rate = clk_get_rate(wdt->clk);

	spin_lock(&wdt->lock);
	load = readl_relaxed(wdt->base + WDTVALUE);

	/*If the interrupt is inactive then time left is WDTValue + WDTLoad. */
	if (!(readl_relaxed(wdt->base + WDTRIS) & INT_MASK))
		load += wdt->load_val + 1;
	spin_unlock(&wdt->lock);

	return div_u64(load, rate);
}

static int wdt_config(struct watchdog_device *wdd, bool ping)
{
	struct sp805_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	if (!ping) {
		ret = clk_prepare(wdt->clk);
		if (ret) {
			dev_err(&wdt->adev->dev, "clock prepare fail");
			return ret;
		}

		ret = clk_enable(wdt->clk);
		if (ret) {
			dev_err(&wdt->adev->dev, "clock enable fail");
			clk_unprepare(wdt->clk);
			return ret;
		}
	}

	spin_lock(&wdt->lock);

	writel_relaxed(UNLOCK, wdt->base + WDTLOCK);
	writel_relaxed(wdt->load_val, wdt->base + WDTLOAD);

	if (!ping) {
		writel_relaxed(INT_MASK, wdt->base + WDTINTCLR);
		writel_relaxed(INT_ENABLE | RESET_ENABLE, wdt->base +
				WDTCONTROL);
	}

	writel_relaxed(LOCK, wdt->base + WDTLOCK);

	/* Flush posted writes. */
	readl_relaxed(wdt->base + WDTLOCK);
	spin_unlock(&wdt->lock);

	return 0;
}

static int wdt_ping(struct watchdog_device *wdd)
{
	return wdt_config(wdd, true);
}

/* enables watchdog timers reset */
static int wdt_enable(struct watchdog_device *wdd)
{
	return wdt_config(wdd, false);
}

/* disables watchdog timers reset */
static int wdt_disable(struct watchdog_device *wdd)
{
	struct sp805_wdt *wdt = watchdog_get_drvdata(wdd);

	spin_lock(&wdt->lock);

	writel_relaxed(UNLOCK, wdt->base + WDTLOCK);
	writel_relaxed(0, wdt->base + WDTCONTROL);
	writel_relaxed(LOCK, wdt->base + WDTLOCK);

	/* Flush posted writes. */
	readl_relaxed(wdt->base + WDTLOCK);
	spin_unlock(&wdt->lock);

	clk_disable(wdt->clk);
	clk_unprepare(wdt->clk);

	return 0;
}

static const struct watchdog_info wdt_info = {
	.options = WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = MODULE_NAME,
};

static const struct watchdog_ops wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= wdt_enable,
	.stop		= wdt_disable,
	.ping		= wdt_ping,
	.set_timeout	= wdt_setload,
	.get_timeleft	= wdt_timeleft,
};

static const struct of_device_id iproc_wdt_match[] = {
	{
		.compatible = "brcm,iproc-wdt",
	},
	{},
};
MODULE_DEVICE_TABLE(of, iproc_wdt_match);

static int sp805_wdt_probe(struct amba_device *adev, const struct amba_id *id)
{
	struct sp805_wdt *wdt;
	int ret;
	struct device_node *dnode = adev->dev.of_node;
	void __iomem *wdt_bootstatus = NULL;
	unsigned int bootstatus_bit = 0;

/* May get here with ARM ID match : 0x00141805, so the following check should not be run */
/*
	dnode = of_find_compatible_node(NULL, NULL, "brcm,iproc-wdt");
	if (!dnode) {
		dev_err(&adev->dev, "can't find DT configuration\n");
		ret = -ENODEV;
		goto error1;
	}
*/
	wdt = devm_kzalloc(&adev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt) {
		dev_err(&adev->dev, "Kzalloc failed\n");
		ret = -ENOMEM;
		goto error1;
	}

	wdt->base = of_iomap(dnode, 0);
	if (!wdt->base) {
		ret = -ENOMEM;
		dev_err(&adev->dev, "of_iomap failed\n");
		goto error2;
	}

	wdt_bootstatus = of_iomap(dnode, 1);
	if (!wdt_bootstatus) {
		ret = -ENOMEM;
		dev_err(&adev->dev, "of_iomap failed\n");
		goto error2;
	}

	wdt->clk = of_clk_get(dnode, 0);
	if (IS_ERR(wdt->clk)) {
		dev_err(&adev->dev, "Clock not found\n");
		ret = PTR_ERR(wdt->clk);
		goto error2;
	}

	wdt->adev = adev;
	wdt->wdd.info = &wdt_info;
	wdt->wdd.ops = &wdt_ops;

	spin_lock_init(&wdt->lock);
	watchdog_set_nowayout(&wdt->wdd, nowayout);
	watchdog_set_drvdata(&wdt->wdd, wdt);
	wdt_setload(&wdt->wdd, DEFAULT_TIMEOUT);

	ret = watchdog_register_device(&wdt->wdd);
	if (ret) {
		dev_err(&adev->dev, "watchdog_register_device() failed: %d\n",
				ret);
		goto error3;
	}

	amba_set_drvdata(adev, wdt);

	ret = of_property_read_u32(dnode, "wdt_boot_status_bit",
						&bootstatus_bit);
	if (ret) {
		dev_err(&adev->dev, "failed getting DT bootstatus bit\n");
		goto error3;
	}

	wdt->wdd.bootstatus = wdt_get_clear_bootstatus(
					wdt_bootstatus,
					bootstatus_bit);

	dev_info(&adev->dev, "registration successful\n");
	dev_info(&adev->dev, "timeout=%d sec, nowayout=%d\n",
		DEFAULT_TIMEOUT, nowayout);

	return 0;

error3:
	clk_put(wdt->clk);

error2:
	if (wdt->base)
		iounmap(wdt->base);
	if (wdt_bootstatus)
		iounmap(wdt_bootstatus);
	kfree(wdt);

error1:
	dev_err(&adev->dev, "Probe Failed!!!\n");

	return ret;
}

static int sp805_wdt_remove(struct amba_device *adev)
{
	struct sp805_wdt *wdt = amba_get_drvdata(adev);

	watchdog_unregister_device(&wdt->wdd);
	amba_set_drvdata(adev, NULL);
	watchdog_set_drvdata(&wdt->wdd, NULL);
	clk_put(wdt->clk);

	return 0;
}

#ifdef CONFIG_PM
static int sp805_wdt_suspend(struct device *dev)
{
	struct sp805_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return wdt_disable(&wdt->wdd);

	return 0;
}

static int sp805_wdt_resume(struct device *dev)
{
	struct sp805_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd))
		return wdt_enable(&wdt->wdd);

	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(sp805_wdt_dev_pm_ops, sp805_wdt_suspend,
		sp805_wdt_resume);

static struct amba_id sp805_wdt_ids[] = {
	{
		.id	= 0x00141805,
		.mask	= 0x00ffffff,
	},
	{ 0, 0 },
};
MODULE_DEVICE_TABLE(amba, sp805_wdt_ids);

static struct amba_driver sp805_wdt_driver = {
	.drv = {
		.name	= MODULE_NAME,
		.pm	= &sp805_wdt_dev_pm_ops,
		.of_match_table	= iproc_wdt_match,
	},
	.id_table	= sp805_wdt_ids,
	.probe		= sp805_wdt_probe,
	.remove		= sp805_wdt_remove,
};

module_amba_driver(sp805_wdt_driver);

MODULE_AUTHOR("Viresh Kumar <viresh.linux@gmail.com>");
MODULE_DESCRIPTION("ARM SP805 Watchdog Driver");
MODULE_LICENSE("GPL");
