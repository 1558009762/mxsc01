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
#include <linux/module.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <linux/interrupt.h>

#include "iproc_spl.h"
#include "iproc_spl_sysfs.h"
#include "socregs.h"
#include "../../kernel/linux-3.6.5/arch/arm/plat-iproc/include/plat/shm.h"
#ifdef CONFIG_PM
#include "../pm/iproc_pm_device.h"
#endif


#ifdef CONFIG_IPROC_SPL_DEBUG
static uint iproc_spl_debug=1;
module_param_named(debug, iproc_spl_debug, uint, 0644);
MODULE_PARM_DESC(debug, "enable/disable debug info (default: 0)");
#endif

/**********************************************************************
 *  Function Prototypes
 **********************************************************************/
static int __devinit iproc_spl_probe(struct platform_device *pldev);
static int __devexit iproc_spl_remove(struct platform_device *pldev);

#ifdef CONFIG_PM
static int iproc_spl_suspend(struct device *dev);
static int iproc_spl_resume(struct device *dev);
static SIMPLE_DEV_PM_OPS(iproc_spl_pm, iproc_spl_suspend, iproc_spl_resume);
#endif

/*Intialization & structures*/
static char const iproc_spl_name[] = DRIVER_NAME;
struct iproc_spl_ctx *spl_ctx = NULL;

/**********************************************************************
 *  This structure defines the methods to be called by a platform device
 *  during the lifecycle of a device
**********************************************************************/
#define SPL_RESOURCE_NAME__spl          "spl_regs"
#define SPL_RESOURCE_NAME__spl_ctl      "spl_ctl_regs"
#define SPL_RESOURCE_NAME__spl_wdog     "spl_wdog"
#define SPL_RESOURCE_NAME__spl_rstmon   "spl_rstmon"
#define SPL_RESOURCE_NAME__spl_pvtmon   "spl_pvtmon"
#define SPL_RESOURCE_NAME__spl_fmon     "spl_fmon"


static struct resource iproc_spl_resources[] = {
	[0] = {
		.name   = SPL_RESOURCE_NAME__spl,
		.flags  = IORESOURCE_MEM,
		.start  = SPL_WDOG,
		.end	= OSC_EN+3,
	},
	[1] = {
		.name	= SPL_RESOURCE_NAME__spl_ctl,
		.flags	= IORESOURCE_MEM,
		.start	= CRMU_SPL_RESET_CONTROL,
		.end	= CRMU_SPL_EVENT_LOG+3,
	},
	[2] = {
		.name	= SPL_RESOURCE_NAME__spl_wdog,
		.flags	= IORESOURCE_IRQ,
		.start	= SPL_IRQ_WDOG,
	},
	[3] = {
		.name	= SPL_RESOURCE_NAME__spl_rstmon,
		.flags	= IORESOURCE_IRQ,
		.start	= SPL_IRQ_RSTMON,
	},
	[4] = {
		.name	= SPL_RESOURCE_NAME__spl_fmon,
		.flags	= IORESOURCE_IRQ,
		.start	= SPL_IRQ_FMON,
	},
};

uint iproc_spl_get_msglevel(void)
{
#ifdef CONFIG_IPROC_SPL_DEBUG
	return iproc_spl_debug;
#else
	return 0;
#endif
}

////////////////////////////////////////////////////////////////////////////
uint32_t spl_reg32_read(const uint32_t addr)
{
	uint32_t val;
	uint32_t address;
	unsigned long flags;

	if (addr >= SPL_WDOG && addr <= OSC_EN)
	{
		address = (uint32_t)(spl_ctx->spl_base_addr) + addr - SPL_WDOG;
	}
	else if (addr >= CRMU_SPL_RESET_CONTROL && addr <= CRMU_SPL_EVENT_LOG)
	{
		address = (uint32_t)(spl_ctx->spl_ctl_addr) + addr - CRMU_SPL_RESET_CONTROL;
	}
	else
	{
		iproc_err("Invalid reg address: 0x%08x\n", addr);
		return 0;
	}

	spin_lock_irqsave(&spl_ctx->spl_lock, flags);
	val = ioread32(address);
	spin_unlock_irqrestore(&spl_ctx->spl_lock, flags);


//	iproc_dbg("Read reg addr=0x%08x, vaddr=0x%08x, result=0x%08x.\n", addr, address,val);

	return val;
}

void spl_reg32_write(const uint32_t addr, const uint32_t val )
{
	uint32_t address;
	unsigned long flags;

	if (addr >= SPL_WDOG && addr <= OSC_EN)
	{
		address = (uint32_t)(spl_ctx->spl_base_addr) + addr - SPL_WDOG;
	}
	else if (addr >= CRMU_SPL_RESET_CONTROL && addr <= CRMU_SPL_EVENT_LOG)
	{
		address = (uint32_t)(spl_ctx->spl_ctl_addr) + addr - CRMU_SPL_RESET_CONTROL;
	}
	else
	{
		iproc_err("Invalid reg address: 0x%08x\n", addr);
		return ;
	}

//	iproc_dbg("Write reg addr=0x%08x, vaddr=0x%08x, val=0x%08x.\n", addr, address,val);
	
	spin_lock_irqsave(&spl_ctx->spl_lock, flags);
	iowrite32(val, address);
	spin_unlock_irqrestore(&spl_ctx->spl_lock, flags);
}

///////////////////////////////////////////////////////////////////////////////////

/* This routine keeps the SPL block reseted, so all regs are to be default values, and any access to these regs returns 0 */
static void spl_regs_reset(void)
{
	spl_reg32_write(CRMU_SPL_RESET_CONTROL, 0);
}

/* This routine releases the reset of SPL block */
static void spl_regs_reset_release(void)
{
	spl_reg32_write(CRMU_SPL_RESET_CONTROL, 1);
}


/*
 *	Initialize one IORESOURCE_MEM resource, return the iomapped vaddr.
 */
static uint32_t *iproc_resource_init_one(struct platform_device *pldev, char *name)
{
	void * __iomem reg_base;
	struct resource *res = NULL;

	res = iproc_platform_get_resource_byname(pldev, IORESOURCE_MEM, name);
	if(res == NULL)
	{
		iproc_err("Failed to get resource by name `%s'!\n", name);
		return NULL;
	}

	iproc_dbg("Get resource `%s', range [0x%08x~0x%08x].\n", res->name, res->start, res->end);

	reg_base = ioremap_nocache(res->start, resource_size(res));
	if(reg_base == NULL)
	{
		iproc_err("Failed to do ioremap for resource `%s'!\n", name);
		return NULL;
	}

	iproc_dbg("resourse `%s' mapped to 0x%p\n", name, reg_base);

	return (uint32_t *)reg_base;
}

static int iproc_spl_resource_init(struct platform_device *pldev)
{
	if((spl_ctx->spl_base_addr = iproc_resource_init_one(pldev, SPL_RESOURCE_NAME__spl))==NULL)
		return -ENXIO;

	if((spl_ctx->spl_ctl_addr = iproc_resource_init_one(pldev, SPL_RESOURCE_NAME__spl_ctl))==NULL)
		return -ENXIO;
	
	return 0;
}

static int iproc_spl_resource_exit(struct platform_device *pldev)
{

	/* disable SPL reg access, this routine must be called before spl_ctx unmapped */
	spl_regs_reset();

	iounmap(spl_ctx->spl_base_addr);
	iounmap(spl_ctx->spl_ctl_addr);
	
	return 0;
}


/////////////////////////////////////////////////////////////////////////////
#ifdef CONFIG_PM
static struct iproc_pm_reg iproc_pm_regs_spl[] = {
	IPROC_SAVEREG(SPL_WDOG),
	IPROC_SAVEREG(SPL_CALIB_0),
	IPROC_SAVEREG(SPL_CALIB_1),
	IPROC_SAVEREG(SPL_CALIB_2),
	IPROC_SAVEREG(SPL_FREQ_MON_EN_0),
	IPROC_SAVEREG(SPL_MONITOR_POINT_0),
	IPROC_SAVEREG(SPL_LOW_THRESH_0),
	IPROC_SAVEREG(SPL_HIGH_THRESH_0),
	IPROC_SAVEREG(SPL_FREQ_MON_EN_1),
	IPROC_SAVEREG(SPL_MONITOR_POINT_1),
	IPROC_SAVEREG(SPL_LOW_THRESH_1),
	IPROC_SAVEREG(SPL_HIGH_THRESH_1),
	IPROC_SAVEREG(SPL_FREQ_MON_EN_2),
	IPROC_SAVEREG(SPL_MONITOR_POINT_2),
	IPROC_SAVEREG(SPL_LOW_THRESH_2),
	IPROC_SAVEREG(SPL_HIGH_THRESH_2),
	IPROC_SAVEREG(SPL_RST_MON_EN_0),
	IPROC_SAVEREG(SPL_RST_CNT_THRESH_0),
	IPROC_SAVEREG(SPL_WIN_CNT_THRESH_0),
//	IPROC_SAVEREG(SPL_RST_CNT_VAL_0),
//	IPROC_SAVEREG(SPL_WIN_CNT_VAL_0),
	IPROC_SAVEREG(SPL_RST_MON_EN_1),
	IPROC_SAVEREG(SPL_RST_CNT_THRESH_1),
	IPROC_SAVEREG(SPL_WIN_CNT_THRESH_1),
//	IPROC_SAVEREG(SPL_RST_CNT_VAL_1),
//	IPROC_SAVEREG(SPL_WIN_CNT_VAL_1),
	IPROC_SAVEREG(SPL_RST_MON_EN_2),
	IPROC_SAVEREG(SPL_RST_CNT_THRESH_2),
	IPROC_SAVEREG(SPL_WIN_CNT_THRESH_2),
//	IPROC_SAVEREG(SPL_RST_CNT_VAL_2),
//	IPROC_SAVEREG(SPL_WIN_CNT_VAL_2),
	IPROC_SAVEREG(SPL_RST_MON_EN_3),
	IPROC_SAVEREG(SPL_RST_CNT_THRESH_3),
	IPROC_SAVEREG(SPL_WIN_CNT_THRESH_3),
//	IPROC_SAVEREG(SPL_RST_CNT_VAL_3),
//	IPROC_SAVEREG(SPL_WIN_CNT_VAL_3),
	IPROC_SAVEREG(OSC_EN),
};
	
static struct iproc_pm_device_regs iproc_pm_dev_spl = {
	.devname  = "SPL",
	.regs	  = &iproc_pm_regs_spl,
	.regs_num = ARRAY_SIZE(iproc_pm_regs_spl),
	.read     = spl_reg32_read,
	.write	  = spl_reg32_write,
};

static int iproc_spl_suspend(struct device *dev)
{
	iproc_dbg("SPL suspending!\n");
	iproc_pm_save_device_regs(&iproc_pm_dev_spl);
	iproc_dbg("SPL suspending done!\n");
	
	return 0;
}

static int iproc_spl_resume(struct device *dev)
{
	iproc_dbg("SPL resume!\n");
	spl_regs_reset_release();
	iproc_pm_restore_device_regs(&iproc_pm_dev_spl);
	iproc_dbg("SPL resume done!\n");
	
	return 0;
}
#endif

static irqreturn_t iproc_spl_isr(int irq, void *data)
{
	iproc_dbg("irq%d occurs.\n", irq);

	/*Clear valid irq*/
	if (irq==SPL_IRQ_WDOG)
	{
		iproc_dbg("Clear irq of wdog.\n");
		spl_reg32_write(SPL_WDOG, 0x0);
	}
	else if (irq==SPL_IRQ_FMON)
	{
		iproc_dbg("Clear irq of low freq.\n");
		spl_reg32_write(SPL_FREQ_MON_EN_0, spl_reg32_read(SPL_FREQ_MON_EN_0) & ~1);
		spl_reg32_write(SPL_FREQ_MON_EN_1, spl_reg32_read(SPL_FREQ_MON_EN_1) & ~1);
		spl_reg32_write(SPL_FREQ_MON_EN_2, spl_reg32_read(SPL_FREQ_MON_EN_2) & ~1);
	}
	else if (irq==SPL_IRQ_PVTMON)
	{
		iproc_dbg("Clear irq of pvt monitor.\n");
	}
	else if (irq==SPL_IRQ_RSTMON)
	{
		iproc_dbg("Clear irq of reset monitor.\n");
	}
	else
	{
		iproc_err("Unknow irq %d!\n", irq);
		return IRQ_NONE;
	}

	/*Take some actions...*/
//	do_reset_cygnus();
	
	return IRQ_HANDLED;
}

static int iproc_spl_request_irqs(struct platform_device *pldev)
{
	int i,j;
	int irqno;
	int ret;
	struct resource *res=NULL;

	for(i=0; i<16; i++)
	{
		res = iproc_platform_get_resource(pldev, IORESOURCE_IRQ, i);
		if(!res)
		{
			iproc_dbg("Cannot get irq resource %d!\n", i);
			break;// get end
		}

		iproc_dbg("Get irq resource %d!\n", i);
		
		ret = request_irq(res->start, iproc_spl_isr, IRQF_DISABLED | IRQF_SHARED, res->name, pldev);
		if(ret)
		{
			iproc_err("Failed to request irq %d for `%s'!\n", res->start, res->name);
			goto err_free_irq;
		}

		iproc_dbg("Request irq %d for `%s'!\n", res->start, res->name);
	}
	
	return 0;

err_free_irq:
	for(j=0; j<i; j++)
	{
		irqno = iproc_platform_get_irq(pldev, j);
		free_irq(irqno, pldev);
	}
	return ret;
}

static void iproc_spl_free_irqs(struct platform_device *pldev)
{
	int i;
	struct resource *res=NULL;

	for(i=0; i<16; i++)
	{
		res = iproc_platform_get_resource(pldev, IORESOURCE_IRQ, i);
		if(!res)
			break;

		free_irq(res->start, pldev);

		iproc_dbg("irq %d for `%s' freed!\n", res->start, res->name);
	}
}

static int __devinit iproc_spl_probe(struct platform_device *pldev)
{
	int ret=0;
	struct resource *res=NULL;

	iproc_dbg("\n");
	
	if(!pldev) 
	{
		iproc_err("platfrom device is NULL!\n");
		return -EINVAL;
	}

	/* setup spl_ctx */
	if ((spl_ctx = kzalloc(sizeof(struct iproc_spl_ctx), GFP_KERNEL)) == NULL)
	{
		iproc_err("Failed to alloc SPL context!\n");
		return -ENOMEM;
	}

	spl_ctx->pdev = pldev;

	if((res=iproc_platform_get_resource_byname(spl_ctx->pdev, IORESOURCE_MEM, SPL_RESOURCE_NAME__spl))==NULL)
	{
		iproc_err("Failed to get resource `%s'!\n", SPL_RESOURCE_NAME__spl);
		ret = -ENXIO;
		goto err_free_ctx;
	}
	spin_lock_init(&spl_ctx->spl_lock);

	platform_set_drvdata(pldev, spl_ctx);

	/* resources init */
	ret = iproc_spl_resource_init(pldev);
	if (ret)
	{
		iproc_err("Fail to init resource, err=%d.\n", ret);
		goto err_free_pm_res;
	}

	/* add sysfs files */
	ret = iproc_spl_sysfs_register(pldev);
	if (ret)
	{
		iproc_err("Fail to create sysfs files, err=%d.\n", ret);
		goto err_free_res;
	}

	ret = iproc_spl_request_irqs(pldev);
	if(ret)
	{
		iproc_err("Failed to request irqs for spl!\n");
		goto err_remove_sysfiles;
	}

	/* enable SPL reg access */
	spl_regs_reset_release();

	iproc_prt("%s (version %s) installed.\n", DRIVER_DESCRIPTION, DRIVER_VERSION);
	iproc_dbg("Build time:%s, %s.\n", BUILD_DATE, BUILD_TIME);

	return 0;

err_remove_sysfiles:
	iproc_spl_sysfs_unregister(pldev);
err_free_res:
	iproc_spl_resource_exit(pldev);
err_free_pm_res:
	kfree(spl_ctx->spl_regs_pm_val);
err_free_ctx:
	kfree(spl_ctx);
	
	return ret;
}

static int __devexit iproc_spl_remove(struct platform_device *pldev)
{
	iproc_dbg("\n");

	iproc_spl_free_irqs(pldev);
	iproc_spl_sysfs_unregister(pldev);
	iproc_spl_resource_exit(pldev);
	
	/* free kmalloc mem */
	kfree(spl_ctx->spl_regs_pm_val);
	kfree(spl_ctx);

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
static struct platform_device *iproc_spl_dev;

static struct platform_driver iproc_spl_driver = {
	.probe   = iproc_spl_probe,
	.remove  = __devexit_p(iproc_spl_remove),
	.driver  = {
		.name  = iproc_spl_name,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &iproc_spl_pm,
#endif
	},
};

static int __init iproc_spl_init(void)
{
	int err = -1;

	iproc_dbg("\n");

	err = iproc_platform_driver_register(&iproc_spl_driver);
	if(err)
	{
		iproc_err("Fail to register platform driver, err=%d.\n", err);
		goto err_out;
	}

	iproc_spl_dev = iproc_platform_device_alloc(iproc_spl_name, -1);
	if (!iproc_spl_dev)
	{
		iproc_err("Fail to alloc platform device `%s', err=%d.\n", iproc_spl_name, err);
		err = -ENOMEM;
		goto err_unregister_driver;
	}

	err = iproc_platform_device_add_resources(iproc_spl_dev, iproc_spl_resources, ARRAY_SIZE(iproc_spl_resources));
	if (err) {
		iproc_err("Failed to add device resource on `%s', err=%d.\n", iproc_spl_name, err);
		goto err_free_device;
	}

	err = iproc_platform_device_add(iproc_spl_dev);
	if (err)
	{
		iproc_err("Fail to add platform device `%s', err=%d.\n", iproc_spl_name, err);
		goto err_free_device;
	}	

	return 0;

err_free_device:
	iproc_platform_device_put(iproc_spl_dev);
err_unregister_driver:
	iproc_platform_driver_unregister(&iproc_spl_driver);
err_out:
	return err;
}

static void __exit iproc_spl_exit(void)
{
	iproc_dbg("\n");

	iproc_platform_device_unregister(iproc_spl_dev);
	iproc_platform_driver_unregister(&iproc_spl_driver);

	iproc_prt("%s (version %s) removed.\n", DRIVER_DESCRIPTION, DRIVER_VERSION);
	
	return;
}

module_init(iproc_spl_init);
module_exit(iproc_spl_exit);


MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_AUTHOR("an.luo@broadcom.com");
MODULE_LICENSE("Proprietary");
