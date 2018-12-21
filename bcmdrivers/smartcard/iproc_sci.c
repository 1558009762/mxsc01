/*****************************************************************************
 * Copyright (c) 2009 Broadcom Corporation.  All rights reserved.
 *
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 * 1. This program, including its structure, sequence and organization,
 *    constitutes the valuable trade secrets of Broadcom, and you shall use
 *    all reasonable efforts to protect the confidentiality thereof, and to
 *    use this information only in connection with your use of Broadcom
 *    integrated circuit products.
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
 *    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
 *    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 *    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
 *    IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS
 *    FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS,
 *    QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU
 *    ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS
 *    LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT,
 *    OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY RELATING TO
 *    YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN
 *    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS
 *    OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER
 *    IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF
 *    ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 *****************************************************************************/


#include <linux/init.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>	/* for copy_from_user */
#include <asm/processor.h>	/* Processor type for cache alignment. */
#include <linux/platform_device.h>

#include "bscd.h"
#include "bkni.h"
#include "iproc_sci.h"
#include "iproc_sci_debug.h"
#include "iproc_sci_ioctl.h"
#include "socregs.h"
#include "plat/shm.h"
#ifdef CONFIG_PM
#include "../pm/iproc_pm_device.h"
#endif

#define IPROC_SCI_CHECK_HW_INIT
#undef IPROC_SCI_CHECK_HW_INIT

/////////////////////////////////////////////////////////////////////////
/* Module parameters */

#ifdef CONFIG_IPROC_SMARTCARD_DEBUG
static uint iproc_sci_debug=1;
module_param_named(debug, iproc_sci_debug, uint, 0644);
MODULE_PARM_DESC(debug, "debug info enable or not (default: 0)");
#endif

static int iproc_sci_major=0;
module_param_named(major, iproc_sci_major, int, 0444);
MODULE_PARM_DESC(major, "Major device number");

static int iproc_sci_coupler=0;
module_param_named(coupler, iproc_sci_coupler, int, 0444);
MODULE_PARM_DESC(coupler, "Coupler: 0=NXP8024, 1=NXP8026(with i2c1)");

static int iproc_sci_numbers=1;
module_param_named(scinum, iproc_sci_numbers, int, 0444);
MODULE_PARM_DESC(scinum, "smart card interfaces number to support, max 2, 1 by default");

/////////////////////////////////////////////////////////////////////////
static struct iproc_sci_ctx  *pCTX;
BSCD_ChannelHandle channelHandle;
static struct work_struct iproc_sci_wq;

extern int  BSCD_Channel_P_IntHandler_isr(BSCD_ChannelHandle channelHandle, int in_channelNumber);
extern void BSCD_Channel_P_IntHandler_bh(BSCD_ChannelHandle channelHandle, int in_channelNumber);
extern int NXP8026_Get_Product_Version(void);


/**********************************************************************
 *  Function Prototypes
 **********************************************************************/
static int __devinit iproc_sci_probe(struct platform_device *pldev);
static int __devexit iproc_sci_remove(struct platform_device *pldev);
static irqreturn_t iproc_sci_irq(int irq, void *dev_id);
static int iproc_sci_init_sci(struct iproc_sci_ctx *pCTX);
static int iproc_sci_fops_open(struct inode *inode, struct file *filep);
static int iproc_sci_fops_release(struct inode *inode, struct file *filep);
static long iproc_sci_fops_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
static void iproc_sci_dev_release(struct device *);

#ifdef CONFIG_PM
static int iproc_sci_suspend(struct device *dev);
static int iproc_sci_resume(struct device *dev);
static SIMPLE_DEV_PM_OPS(iproc_sci_pm, iproc_sci_suspend, iproc_sci_resume);
#endif

/*Intialization & structures*/
char const iproc_sci_name[] = DRIVER_NAME;

static const struct file_operations iproc_sci_dev_fops = {
	.owner   = THIS_MODULE,
	.open    = iproc_sci_fops_open,
	.release = iproc_sci_fops_release,
	.unlocked_ioctl = iproc_sci_fops_ioctl,
};


/**********************************************************************
 *  This structure defines the methods to be called by a platform device
 *  during the lifecycle of a device
**********************************************************************/
#define ASIU_SMART_CARD_INTR_BASE       193
#define ASIU_SMART_CARD_INTR_NUM        3
#define ASIU_SMART_CARD_INTR(id)        (ASIU_SMART_CARD_INTR_BASE + id)

#define SCI_RESOURCE_NAME__sci_irq(id)  "sci_irq"#id
#define SCI_RESOURCE_NAME__sci_iomux    "sci_iomux"
#define SCI_RESOURCE_NAME__sci_regs     "sci_regs"
#define SCI_RESOURCE_NAME__sci_dmu      "sci_dmu"
#define SCI_RESOURCE_NAME__sci_clkctl   "sci_clkctl"
#define SCI_RESOURCE_NAME__sci_gpio     "sci_gpio"

static struct resource iproc_sci_resources[] = {
	/* IRQ Resource */
	[0] = {
		.name   = SCI_RESOURCE_NAME__sci_irq(0),
		.flags  = IORESOURCE_IRQ,
		.start  = ASIU_SMART_CARD_INTR(0),
	},
	[1] = {
		.name	= SCI_RESOURCE_NAME__sci_irq(1),
		.flags	= IORESOURCE_IRQ,
		.start	=ASIU_SMART_CARD_INTR(1),
	},
	[2] = {
		.name	= SCI_RESOURCE_NAME__sci_irq(2),
		.flags	= IORESOURCE_IRQ,
		.start	= ASIU_SMART_CARD_INTR(2),
	},
	/* Mem Reg Resource */
	[3] = {
		.name   = SCI_RESOURCE_NAME__sci_iomux,
		.flags  = IORESOURCE_MEM,
		.start  = CRMU_CHIP_IO_PAD_CONTROL,
		.end	= SMART_CARD_FCB_SEL+3,
	},
	[4] = {
		.name   = SCI_RESOURCE_NAME__sci_regs,
		.flags  = IORESOURCE_MEM,
		.start  = SCA_SC_UART_CMD_1,
		.end	= SCIRQ_SCPU_SCIRQSTAT+3,
	},
	[5] = {
		.name	= SCI_RESOURCE_NAME__sci_dmu,
		.flags	= IORESOURCE_MEM,
		.start	= ASIU_INTR_STATUS,
		.end	= ASIU_TOP_SW_RESET_CTRL+3,
	},
	[6] = {
		.name	= SCI_RESOURCE_NAME__sci_clkctl,
		.flags	= IORESOURCE_MEM,
		.start	= ASIU_TOP_CLK_GATING_CTRL,
		.end	= ASIU_TOP_CLK_GATING_CTRL+3,
	},
	[7] = {
		.name	= SCI_RESOURCE_NAME__sci_gpio,
		.flags	= IORESOURCE_MEM,
		.start	= GP_DATA_OUT,
		.end	= GP_INT_CLR+3,
	},
};
#define IPROC_RESOURCES_NUM  ARRAY_SIZE(iproc_sci_resources)

static IPROC_Resources_t IPROC_SCI_RES[IPROC_RESOURCES_NUM];

////////////////////////////////////////////////////////////////////
static struct platform_device iproc_sci_pdev = {
	.name   = iproc_sci_name,
	.id	    = -1,
	.dev	= {
		.release = iproc_sci_dev_release,
	},
	.resource		 = iproc_sci_resources,
	.num_resources	 = IPROC_RESOURCES_NUM,
};

static struct platform_driver iproc_sci_driver = {
	.probe   = iproc_sci_probe,
	.remove  = __devexit_p(iproc_sci_remove),
	.driver  =
	{
		.name  = iproc_sci_name,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &iproc_sci_pm,
#endif
	},
};


/////////////////////////////////////////////////////////////
uint iproc_sci_get_msglevel(void)
{
#ifdef CONFIG_IPROC_SMARTCARD_DEBUG
	return iproc_sci_debug;
#else
	return 0;
#endif
}

static DEFINE_SPINLOCK(reg_lock);

/* Get the mapped address for a physical reg */
static uint32_t sci_reg32_get_mapped_addr(const uint32_t reg)
{
	uint32_t i;
	uint32_t address;
	IPROC_Resources_t *Res=NULL;

	for(i=0; i<IPROC_RESOURCES_NUM; i++)
	{
		Res = &IPROC_SCI_RES[i];
	
		if(Res->res->flags != IORESOURCE_MEM)
			continue;

		if (reg >= Res->res->start && reg <= Res->res->end)
		{
			if(Res->vaddr == NULL)
			{
				iproc_err("Resource `%s' not initialized!\n", Res->res->name);
				return 0;
			}

			address = ((uint32_t)Res->vaddr + (reg - Res->res->start));
//			iproc_dbg("Reg=0x%08x, vaddr=0x%08x\n", reg, address);
			
			return address;
		}
	}

	return 0;
}

uint32_t sci_reg32_read(const uint32_t addr)
{
	uint32_t val;
	uint32_t address = sci_reg32_get_mapped_addr(addr);
	
	if(!address)
	{
		iproc_err("Invalid reg addr: 0x%08x, caller: [%pS]\n", addr, __builtin_return_address(0));
		return 0;
	}
	
	spin_lock(&reg_lock);
	val = ioread32(address);
	spin_unlock(&reg_lock);

//	iproc_dbg("address=0x%08x, val=0x%08x.\n", address,val);

	return val;
}

void sci_reg32_write(const uint32_t addr, const uint32_t value)
{
	uint32_t address = sci_reg32_get_mapped_addr(addr);
	
	if(!address)
	{
		iproc_err("Invalid reg addr: 0x%08x, caller: [%pS]\n", addr, __builtin_return_address(0));
		return ;
	}
	
	spin_lock(&reg_lock);
	iowrite32(value, address);
	spin_unlock(&reg_lock);
}

void iproc_sci_regs_dump(uint32_t reg, uint32_t cnt, char *des)
{
	int i;
	int count = 1 + (SCA_SC_EVENT2_CMD_4 - SCA_SC_UART_CMD_1)/4;
	uint32_t base, addr;

	if(iproc_sci_get_msglevel()==0)
	{
		return ;
	}
	
	if(cnt!=0)
		count = cnt;

	if(reg == 0)
	{
		base = SCA_SC_UART_CMD_1;
		iproc_dbg("SCI 0:\n");
	}
	else if(reg == 1)
	{
		base = SCB_SC_UART_CMD_1;
		iproc_dbg("SCI 1:\n");
	}
	else
	{
		base = reg;
		iproc_dbg("%s\n", des?des:"");
	}
	
	for(i=0; i<count; i++)
	{
		addr = base+i*4;
		iproc_dbg("  [0x%08x] = 0x%08x\n", addr, sci_reg32_read(addr));
	}

	iproc_dbg("SCI IRQ:\n");
	for(addr=SCIRQ1_SCIRQEN; addr<=SCIRQ_SCPU_SCIRQSTAT; addr+=4)
	{
		iproc_dbg("  [0x%08x] = 0x%08x\n", addr, sci_reg32_read(addr));
	}

	iproc_prt("\n");
}

/////////////////////////////////////////////////////////////
static ssize_t regs_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	iproc_sci_regs_dump(0, 0, NULL);
	return 0;
}

static ssize_t regs_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t n)
{
	uint32_t reg=0, val=0;
	int err;

	if ((err=sscanf(buf, "0x%x 0x%x", &reg, &val)) != 2)
		return -EINVAL;

	iproc_dbg("orig [0x%08x]=0x%08x\n", reg, sci_reg32_read(reg));
	sci_reg32_write(reg, val);
	iproc_dbg("new  [0x%08x]=0x%08x\n", reg, sci_reg32_read(reg));
	
	return n;
}

#define IPROC_DEV_ATTR_RW(_name) 	DEVICE_ATTR(_name, 0644, _name##_show, _name##_store);

static IPROC_DEV_ATTR_RW(regs);

static struct attribute *sci_dev_attrs[] = {
	&dev_attr_regs.attr,
	NULL
};

static struct attribute_group sci_attr_group = {
	.name = DRIVER_NAME,
	.attrs = sci_dev_attrs,
};

int iproc_sci_sysfs_register(struct platform_device *pldev)
{
	return iproc_sysfs_create_group(&pldev->dev.kobj, &sci_attr_group);
}

void iproc_sci_sysfs_unregister(struct platform_device *pldev)
{
	iproc_sysfs_remove_group(&pldev->dev.kobj, &sci_attr_group);
}
////////////////////////////////////////////////////////////////////////

/*
 * Initialize one IORESOURCE_MEM resource, return the iomapped vaddr.
 */
static uint32_t *iproc_resource_init_one(struct platform_device *pldev, const char *name)
{
	void * __iomem reg_base;
	struct resource *res = NULL;

	if(!pldev || !name)
	{
		iproc_err("Invalid arg!\n");
		return NULL;
	}		
	
	res = iproc_platform_get_resource_byname(pldev, IORESOURCE_MEM, name);
	if(res == NULL)
	{
		iproc_err("Failed to get resource by name `%s'!\n", name);
		return NULL;
	}

	reg_base = ioremap_nocache(res->start, resource_size(res));
	if(reg_base == NULL)
	{
		iproc_err("Failed to map resource [%10s: 0x%08x~0x%08x/0x%08x]\n", name, res->start, res->end, resource_size(res));
		return NULL;
	}
	iproc_dbg("resource [%10s: 0x%08x~0x%08x/0x%08x] mapped to 0x%p\n", name, res->start, res->end, resource_size(res), reg_base);

	return (uint32_t *)reg_base;
}

static int iproc_sci_resource_init(struct platform_device *pldev)
{
	int i=0, j=0;
	uint32_t * __iomem mapped_addr;

	for(i=0; i<IPROC_RESOURCES_NUM; i++)
	{
		IPROC_SCI_RES[i].res = &iproc_sci_resources[i];

		if(IPROC_SCI_RES[i].res->flags != IORESOURCE_MEM)
			continue;
		
		mapped_addr = iproc_resource_init_one(pldev, IPROC_SCI_RES[i].res->name);

		if(!mapped_addr)
			return -ENXIO;
		else
			IPROC_SCI_RES[i].vaddr= mapped_addr;

		j++;
	}

	iproc_dbg("SCI all MEM resources initialized\n");

	return 0;
}

static int iproc_sci_resource_exit(struct platform_device *pldev)
{
	int i=0;

	for(i=0; i<IPROC_RESOURCES_NUM; i++)
	{
		if(IPROC_SCI_RES[i].res->flags==IORESOURCE_MEM && IPROC_SCI_RES[i].vaddr != NULL)
			iounmap(IPROC_SCI_RES[i].vaddr);
	}
	
	return 0;
}

static int iproc_sci_ctx_init(uint32_t intf)
{
	int i;
	int ret = -ENXIO;

	if(intf>=iproc_sci_numbers)
	{
		iproc_err("Not supported SCI number: %d\n", intf);
		return -EINVAL;
	}

	for(i=0; i<IPROC_RESOURCES_NUM; i++)
	{
		if(!strcmp(IPROC_SCI_RES[i].res->name, SCI_RESOURCE_NAME__sci_regs))
		{
			if(IPROC_SCI_RES[i].vaddr != NULL)
			{
				pCTX->sci_reg_base = (uint32_t*)((uint32_t)IPROC_SCI_RES[i].vaddr + intf*(SCB_SC_UART_CMD_1 - SCA_SC_UART_CMD_1));
				ret = 0;
			}
			else
			{
				iproc_err("resource `%s' not initialized!\n", SCI_RESOURCE_NAME__sci_regs);
			}

			break;
		}
	}

	iproc_dbg("SCI%d reg base is set to 0x%p\n", intf, pCTX->sci_reg_base);
	
	return ret;
}

/* Enable/Disable SCI IO PAD */
static void iproc_sci_set_io_pad(int en)
{
	uint32_t io = CRMU_CHIP_IO_PAD_CONTROL;

	en = en ? 1 : 0;
	
	iproc_dbg("%s SCI block IO PAD\n", en?"Enable":"Disable");
	
	sci_reg32_write(io, en);

	iproc_dbg("CRMU_CHIP_IO_PAD_CONTROL [0x%08x]=0x%08x\n", io, sci_reg32_read(io));
}

/* Reset SCI block, not smart card */
static void iprco_sci_reset(void)
{
	int i=0;
	uint32_t sw_rst=0;

	iproc_dbg("Reset SCI block\n");
	
	sw_rst = sci_reg32_read(ASIU_TOP_SW_RESET_CTRL);

	//put sci into reset
	sw_rst &= ~(1 << ASIU_TOP_SW_RESET_CTRL__SMARTCARD_SW_RESET_N);
	sci_reg32_write(ASIU_TOP_SW_RESET_CTRL, sw_rst);

	for(i=0;i<1000;i++)
		udelay(1000);

	//release sci from reset
	sw_rst |= (1 << ASIU_TOP_SW_RESET_CTRL__SMARTCARD_SW_RESET_N);
	sci_reg32_write(ASIU_TOP_SW_RESET_CTRL, sw_rst);
}

/* Enable/Disable SCI block clock */
static void iproc_sci_set_sci_clock(int en)
{
	uint32_t clkctr;

	iproc_dbg("%s SCI block clock\n", en?"Enable":"Disable");
	
	clkctr = sci_reg32_read(ASIU_TOP_CLK_GATING_CTRL);
	
	if(en) //enable SCI clock
		clkctr |= (1<<ASIU_TOP_CLK_GATING_CTRL__SMARTCARD_CLK_GATE_EN);
	else //clock gate SCI
		clkctr &= ~(1<<ASIU_TOP_CLK_GATING_CTRL__SMARTCARD_CLK_GATE_EN);	

	sci_reg32_write(ASIU_TOP_CLK_GATING_CTRL, clkctr);
}

#ifdef IPROC_SCI_CHECK_HW_INIT
/* Set IO mux to enable SCI block */
static int iproc_sci_set_sci_iomux(void)
{
	uint32_t iomux3=CRMU_IOMUX_CTRL3;

	iproc_dbg("Set SCI block IOMUX\n");

	sci_reg32_write(iomux3, sci_reg32_read(iomux3) & ~(0x7<<CRMU_IOMUX_CTRL3__CORE_TO_IOMUX_SMART_CARD0_SEL_R));

	if(iproc_sci_numbers==2)
		sci_reg32_write(iomux3, sci_reg32_read(iomux3) & ~(0x7<<CRMU_IOMUX_CTRL3__CORE_TO_IOMUX_SMART_CARD1_SEL_R));

	iproc_dbg("CRMU_IOMUX_CTRL3 [0x%08x]=0x%08x\n", iomux3, sci_reg32_read(iomux3));
	
	return 0;
}

/* Set SCI FCB type */
static int iproc_sci_set_sci_fcb(int sync)
{
	uint32_t fcb = SMART_CARD_FCB_SEL;
	uint32_t cur = 0;

	sync &= 0xf;

	iproc_dbg("Set SCI block FCB\n");

	cur = sci_reg32_read(fcb) & ~(0xf<<SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD0_FCB_SEL_R);
	sci_reg32_write(fcb, cur | (sync<<SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD0_FCB_SEL_R));

	if(iproc_sci_numbers==2)
	{
		cur = sci_reg32_read(fcb) & ~(0xf<<SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD1_FCB_SEL_R);
		sci_reg32_write(fcb, cur | (sync<<SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD1_FCB_SEL_R));
	}

	iproc_dbg("SMART_CARD_FCB_SEL [0x%08x]=0x%08x\n", fcb, sci_reg32_read(fcb));
	
	return 0;
}

/* Set GPIO as 3.3v program pin 
    To Program SC0_DC_GPIO3_3P3, following AON GPIO configuration is needed
    1.	Program AON_GPIO1 to 1.
    2.	Program AON_GPIO0 to 0.
*/
static int iproc_sci_set_gpio_program_pin(void)
{
	iproc_dbg("Set SCI GPIO pin\n");
	
	sci_reg32_write(GP_OUT_EN , sci_reg32_read(GP_OUT_EN)|0x3);

	sci_reg32_write(GP_DATA_OUT, sci_reg32_read(GP_DATA_OUT)|0x2);

	if (iproc_sci_numbers==2)
		sci_reg32_write(GP_DATA_OUT, sci_reg32_read(GP_DATA_OUT)|0x1);

	iproc_dbg("GP_OUT_EN [0x%08x]=0x%08x\n", GP_OUT_EN, sci_reg32_read(GP_OUT_EN));
	iproc_dbg("GP_DATA_OUT [0x%08x]=0x%08x\n", GP_DATA_OUT, sci_reg32_read(GP_DATA_OUT));

	return 0;
}
#endif

void iproc_sci_set_sci_irq(int en)
{
	int irqen = 0x0;
	
	if(en)
	{
		if(iproc_sci_numbers==2)
			irqen = 0x3;
		else
			irqen = 0x1;
	}
	
	iproc_dbg("%s SCI IRQs\n", en ? "Enable" : "Disable");
	
	sci_reg32_write(SCIRQ0_SCIRQEN, irqen);
	sci_reg32_write(SCIRQ1_SCIRQEN, irqen);
	sci_reg32_write(SCIRQ_SCPU_SCIRQEN, irqen);
}

/**********************************************************************
 *	iproc_sci_init_sci():
 *	Initialize the smart card stack
 *
 *  The SCI block power should have been enabled (DMU_SCI_PWR_ENABLE | DMU_SCICLK_PWR_ENABLE)
 *
 *  Input parameters:
 *         pCTX: struct iproc_sci_ctx *
 *
 *  Return value:
 *	       0
 *
 **********************************************************************/
static int iproc_sci_init_sci(struct iproc_sci_ctx *pCTX)
{
	int ret;

	iproc_dbg("\n");

#ifndef CYGNUS_SCI_DRV   //bcm5892
	/* Enable smart card 0 Alternative functions */
	reg_gpio_iotr_set_pin_type( GPIO_AUX_SCI0, GPIO_PIN_TYPE_ALTERNATIVE_FUNC0 );
	reg_gpio_iotr_set_pin_type( 14, GPIO_PIN_TYPE_ALTERNATIVE_FUNC1 );
	reg_gpio_iotr_set_pin_type( 15, GPIO_PIN_TYPE_ALTERNATIVE_FUNC1 );
#endif

	/* Next calls smart card stack functions */
	if(0==iproc_sci_coupler)
		iproc_dbg("Use coupler NXP8024\n");
	else
		iproc_dbg("Use coupler NXP8026\n");
	
	ret = BSCD_Open(&pCTX->moduleHandle, NULL, ((iproc_sci_coupler == 0) ? COUPLER_NXP8024 : COUPLER_NXP8026));
	if(ret != BERR_SUCCESS)
		iproc_dbg("BSCD_Open failed\n");


	return ret;
}

static int iproc_sci_fops_open(struct inode *inode, struct file *filep)
{
	int ret;
	BSCD_ChannelHandle handle;
	
#ifdef CYGNUS_SCI_DRV
	int sci_no=0;

	sci_no = iminor(inode);
	iproc_dbg("Open SCI %d\n", sci_no);

	if(sci_no>=iproc_sci_numbers)
	{
		iproc_err("Not supported sci number, max is %d!\n", iproc_sci_numbers-1);
		return -EINVAL;
	}
	
	iproc_sci_ctx_init(sci_no);
#endif

	iproc_dbg("\n");

	mutex_lock(&pCTX->sc_mutex);
	
	ret = BSCD_Channel_Open(pCTX->moduleHandle, &handle, 0 /* first channel */,
				NULL, (unsigned int)pCTX->sci_reg_base);	
	if(ret != BERR_SUCCESS) {
		iproc_err("OPEN channel failed\n");
		ret = -EPERM;
		goto out;
	}
	
	ret = BSCD_Channel_ResetIFD(handle, BSCD_ResetType_eCold);	
	if(ret != BERR_SUCCESS) {
		iproc_err("RESET IFD failed\n");
		ret = -EPERM;
		goto out;
	}

	ret = 0;
	filep->private_data = handle;

	iproc_dbg("Open channel success\n");
	
 out:
 	mutex_unlock(&pCTX->sc_mutex);
	return ret;
}

static int iproc_sci_fops_release(struct inode *inode, struct file *filep)
{
	int ret ;
		
	iproc_dbg("\n");

	mutex_lock(&pCTX->sc_mutex);
	ret = BSCD_Channel_Close(pCTX->moduleHandle, 0);
	if(ret != BERR_SUCCESS) {
		iproc_err("CLOSE failed\n");
		ret = -1;
		goto out;
	}

	ret = 0;

 out:
 	mutex_unlock(&pCTX->sc_mutex);
	return ret;
}

int iproc_sc_reset_get_atr(BSCD_ChannelHandle handle, struct atr_parm __user *up)
{
	int ret;
	unsigned char *rbuf;
	unsigned long rsize;
	struct atr_parm atr_info;

	iproc_dbg("\n");

	ret = BSCD_Channel_ResetIFD(handle, BSCD_ResetType_eCold);
	if(ret != BERR_SUCCESS) {
		iproc_err("RESET IFD failed\n");
		return -EFAULT;
	}
	iproc_dbg("RESET IFD OK\n");

	ret = BSCD_Channel_DetectCardNonBlock(handle, BSCD_CardPresent_eInserted);
	if(ret != BERR_SUCCESS) {
		iproc_err("DETECT CARD failed\n");
		return -EFAULT;
	}
	iproc_dbg("DETECT CARD OK\n");

	ret = BSCD_Channel_ResetCard(handle, BSCD_ResetCardAction_eReceiveAndDecode);
	if(ret != BERR_SUCCESS) {
		iproc_err("RESET CARD failed\n");
		return -EFAULT;
	}
	iproc_dbg("RESET CARD OK\n");

	rbuf = (unsigned char *) kzalloc(BSCD_MAX_ATR_SIZE, GFP_KERNEL);
	if(!rbuf)
	{
		iproc_err("KZALLOC failed, SIZE=%d\n", BSCD_MAX_ATR_SIZE);
		return -ENOMEM;
	}
	
	ret = BSCD_Channel_get_atr(handle, rbuf, &rsize);
	if(ret != BERR_SUCCESS) {
		iproc_err("RECEIVE failed, err:%d\n",ret);
		kfree(rbuf);
		return -EFAULT;
	}

	memcpy(atr_info.atr, rbuf, rsize);
	atr_info.atr_len = rsize;
	kfree(rbuf);

	if(copy_to_user(up, &atr_info, sizeof(struct atr_parm))) {
		iproc_err("copy_to_user failed for GET_ATR\n");
		return -EFAULT;
	}

	return ret;
}

int iproc_sc_transmit(BSCD_ChannelHandle handle, struct apdu_args __user *up)
{
	struct apdu_args ap;
	int ret;

	iproc_dbg("\n");
	
	if (copy_from_user(&ap, up, sizeof(*up))) {
		iproc_err("copy_from_user failed for TRANSMIT\n");
		return -EFAULT;
	}
	ret = BSCD_Channel_APDU_Transceive(handle, 
					   ap.txbuf, ap.txlen,
					   ap.rxbuf, (unsigned long *)ap.rxlen,
					   BSCD_MAX_APDU_SIZE);
	if(ret != BERR_SUCCESS) {
		iproc_err("Transceive failed\n");
		return -EFAULT;
	}
	if(copy_to_user(up, &ap, sizeof(ap))) {
		iproc_err("copy_to_user failed for TRANSMIT\n");
		return -EFAULT;
	}
	return ret;
}

#ifdef CONFIG_PM
static struct iproc_pm_reg iproc_pm_regs_sci[] = {
	IPROC_SAVEREG(SCA_SC_UART_CMD_1),
	IPROC_SAVEREG(SCA_SC_IF_CMD_1),
	IPROC_SAVEREG(SCA_SC_CLK_CMD_1),
	IPROC_SAVEREG(SCA_SC_PROTO_CMD),
	IPROC_SAVEREG(SCA_SC_PRESCALE),
	IPROC_SAVEREG(SCA_SC_TGUARD),
	IPROC_SAVEREG(SCA_SC_TRANSMIT),
	IPROC_SAVEREG(SCA_SC_RECEIVE),
	IPROC_SAVEREG(SCA_SC_TLEN_2),
	IPROC_SAVEREG(SCA_SC_TLEN_1),
	IPROC_SAVEREG(SCA_SC_FLOW_CMD),
	IPROC_SAVEREG(SCA_SC_RLEN_2),
	IPROC_SAVEREG(SCA_SC_RLEN_1),
//	IPROC_SAVEREG(SCA_SC_STATUS_1),
//	IPROC_SAVEREG(SCA_SC_STATUS_2),
	IPROC_SAVEREG(SCA_SC_CLK_CMD_2),
	IPROC_SAVEREG(SCA_SC_UART_CMD_2),
	IPROC_SAVEREG(SCA_SC_BGT),
	IPROC_SAVEREG(SCA_SC_TIMER_CMD),
	IPROC_SAVEREG(SCA_SC_IF_CMD_2),
	IPROC_SAVEREG(SCA_SC_INTR_EN_1),
	IPROC_SAVEREG(SCA_SC_INTR_EN_2),
//	IPROC_SAVEREG(SCA_SC_INTR_STAT_1),
//	IPROC_SAVEREG(SCA_SC_INTR_STAT_2),
	IPROC_SAVEREG(SCA_SC_TIMER_CMP_1),
	IPROC_SAVEREG(SCA_SC_TIMER_CMP_2),
	IPROC_SAVEREG(SCA_SC_TIMER_CNT_1),
	IPROC_SAVEREG(SCA_SC_TIMER_CNT_2),
	IPROC_SAVEREG(SCA_SC_WAIT_1),
	IPROC_SAVEREG(SCA_SC_WAIT_2),
	IPROC_SAVEREG(SCA_SC_WAIT_3),
	IPROC_SAVEREG(SCA_SC_IF_CMD_3),
//	IPROC_SAVEREG(SCA_SC_EVENT1_CNT),
//	IPROC_SAVEREG(SCA_SC_EVENT1_CMP),
//	IPROC_SAVEREG(SCA_SC_EVENT1_CMD_1),
//	IPROC_SAVEREG(SCA_SC_EVENT1_CMD_2),
//	IPROC_SAVEREG(SCA_SC_EVENT1_CMD_3),
//	IPROC_SAVEREG(SCA_SC_EVENT1_CMD_4),
//	IPROC_SAVEREG(SCA_SC_EVENT2_CMP),
//	IPROC_SAVEREG(SCA_SC_EVENT2_CNT),
//	IPROC_SAVEREG(SCA_SC_EVENT2_CMD_1),
//	IPROC_SAVEREG(SCA_SC_EVENT2_CMD_2),
//	IPROC_SAVEREG(SCA_SC_EVENT2_CMD_3),
//	IPROC_SAVEREG(SCA_SC_EVENT2_CMD_4),
};

static struct iproc_pm_device_regs iproc_pm_dev_sci = {
	.devname  = "SCI",
	.regs     = &iproc_pm_regs_sci,
	.regs_num = ARRAY_SIZE(iproc_pm_regs_sci),
	.read	  = sci_reg32_read,
	.write	  = sci_reg32_write,
};

static int iproc_sci_suspend(struct device *dev)
{
	struct iproc_sci_ctx *sci_CTX = dev_get_drvdata(dev);

	iproc_dbg("Enter SCI suspend\n");
	
 	if (sci_CTX->busy) 
	{
		iproc_err("driver busy\n");
 		return -EAGAIN;                           // PM core will call this routine again, until not busy
	}

	iproc_pm_save_device_regs(&iproc_pm_dev_sci); // save current settings

	iproc_sci_set_sci_irq(0);                     // disable SCI irqs
	
//	iproc_sci_set_sci_clock(0);                                            //controlled by PM core under user defined policy

	return 0;
}

static int iproc_sci_resume(struct device *dev)
{
	struct iproc_sci_ctx *sci_CTX = dev_get_drvdata(dev);

	iproc_dbg("Enter SCI resume\n");
	
 	if (sci_CTX->busy)
	{
		iproc_err("driver busy\n");
 		return -EAGAIN;
	}
	
	iproc_sci_set_io_pad(0);
	iprco_sci_reset();
	iproc_sci_set_sci_clock(1);
	iproc_sci_set_sci_irq(1);
	iproc_pm_restore_device_regs(&iproc_pm_dev_sci);

	return 0;
}
#endif


/**********************************************************************
 *  iproc_sci_fops_ioctl
 *
 *  The IOCTL handling routine in fp ops
 *
 *  Return value:
 *     IRQ_RETVAL(1): Interrupt handling is Succesful
 *     IRQ_RETVAL(0): Interrupt handling is Not Succesful
 **********************************************************************/
static long iproc_sci_fops_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	int ret = BERR_SUCCESS;
	BSCD_ChannelHandle channelHandle;
	void __user *p = (void __user *)arg;

	iproc_dbg("\n");
	
	channelHandle = (BSCD_ChannelHandle) filep->private_data;
	mutex_lock(&pCTX->sc_mutex);
	pCTX->busy = 1;

	/* Don't return from this switch statement without clearing pCTX->busy */
	switch (cmd) {
		case SC_IOCTL_INIT:
			iproc_dbg("Get IOCTL INIT\n");
			break;
		case SC_IOCTL_POWERUP:
			iproc_dbg("Get IOCTL POWERUP\n");
			ret = iproc_sc_reset_get_atr(channelHandle,p);
			if(ret != BERR_SUCCESS) {
				iproc_err("POWERUP failed\n");
				ret = -EFAULT;
				goto out;
			}
			break;
		case SC_IOCTL_POWERDOWN:
			iproc_dbg("Get IOCTL POWERDOWN\n");
			ret = BSCD_Channel_PowerICC(channelHandle, BSCD_PowerICC_ePowerDown);
			if(ret != BERR_SUCCESS) {
				iproc_err("POWERICC DOWN failed\n");
				ret = -EFAULT;
				goto out;
			}
			break;
		case SC_IOCTL_TRANSMIT:
			iproc_dbg("Get IOCTL TRANSMIT\n");
			ret = iproc_sc_transmit(channelHandle,p);
			break;
		case SC_IOCTL_RESET:
			iproc_dbg("Get IOCTL RESET\n");
			ret = iproc_sc_reset_get_atr(channelHandle,p);
			if(ret != BERR_SUCCESS) {
				iproc_err("RESET failed\n");
				ret = -EFAULT;
				goto out;
			}
			break;
		case SC_IOCTL_ICCSTATUS:
			iproc_dbg("Get IOCTL ICCSTATUS\n");
			ret = BSCD_Channel_DetectCardNonBlock(channelHandle, BSCD_CardPresent_eInserted);
			if(ret == BERR_UNKNOWN) /* no card*/
				put_user(0, (int __user *)arg);
			else if(ret == BERR_SUCCESS)
				put_user(1,(int __user *)arg);			  
			else {
				iproc_err("GETSTATUS failed\n");
				put_user(-1, (int __user *)arg);
				ret = -EFAULT;
				goto out;
			}
			ret = BERR_SUCCESS;
			break;
		default:
			iproc_err("Unrecognized ioctl:0x%x\n", cmd);
			ret = -ENOTTY;
			break;
	}

out:
	pCTX->busy = 0;
	mutex_unlock(&pCTX->sc_mutex);

	return ret;
}


/**********************************************************************
 *  iproc_sci_irq
 *
 *  The interrupt handling routine
 *
 *  Input parameters:
 *     irq - The Interrupt Number dedicated to us.
 *     drv_ctx - The driver context
 *     r -
 *
 *  Return value:
 *     IRQ_RETVAL(1): Interrupt handling is Succesful
 *     IRQ_RETVAL(0): Interrupt handling is Not Succesful
 **********************************************************************/
static irqreturn_t iproc_sci_irq(int irq, void *drv_ctx)
{
	struct iproc_sci_ctx  *pCTX = drv_ctx;
	int    bh;
	uint32_t irqstat0=0, irqstat1=0, irqstat=0;

	iproc_dbg("SCI IRQ %d occured\n", irq);
	irqstat0 = sci_reg32_read(SCIRQ0_SCIRQEN);
	irqstat1 = sci_reg32_read(SCIRQ1_SCIRQEN);
	irqstat  = sci_reg32_read(SCIRQ_SCPU_SCIRQEN);
	iproc_dbg("  irqstat=0x%x, irqstat0=0x%x, irqstat1=0x%x.\n",  irqstat, irqstat0, irqstat);

	bh = BSCD_Channel_P_IntHandler_isr(pCTX->moduleHandle->channelHandles[0], 0);

	if (bh)
		schedule_work(&iproc_sci_wq);

	return IRQ_HANDLED;
}

void iproc_sci_wq_irqHandling(struct work_struct *work)
{
	BSCD_Channel_P_IntHandler_bh(pCTX->moduleHandle->channelHandles[0], 0);
}

static int iproc_sci_irq_init(struct platform_device *pldev)
{
	int i, irqno, ret;

	for(i=0; i<ASIU_SMART_CARD_INTR_NUM; i++)
	{
		irqno = iproc_platform_get_irq(pldev, i);

#ifndef CYGNUS_SCI_DRV
		pCTX->irq = irqno; //this seems not used
#endif

		ret = request_irq(irqno, iproc_sci_irq, IRQF_DISABLED | IRQF_SHARED, pldev->name, pCTX);
		if(ret)
		{
			iproc_err("Failed to request irq %d\n", irqno);
			return -EINVAL;
		}
		iproc_dbg("Success to request irq %d\n", irqno);
	}

	return 0;
}

static void iproc_sci_irq_free(struct platform_device *pldev)
{
	int i, irqno;

	for(i=0; i<ASIU_SMART_CARD_INTR_NUM; i++)
	{
		irqno = iproc_platform_get_irq(pldev, i);
		free_irq(irqno, pCTX);
	}
}



/**********************************************************************
 *  iproc_sci_probe
 *
 *  The Platform Driver Probe function.
 *
 *  Input parameters:
 *         device: The Device Context
 *
 *  Return value:
 *		    0: Driver Probe is Succesful
 *		not 0: ERROR
 **********************************************************************/
int __devinit iproc_sci_probe(struct platform_device *pldev)
{
//	struct device *dev = &(pldev->dev);
	int ret;

	/* Validation of platform device structure */
	if(!pldev) {
		iproc_err("platfrom_device pointer is NULL!\n");
		return -EINVAL;
	}

	/*---------- Context -----------*/
	pCTX = kzalloc(sizeof(struct iproc_sci_ctx), GFP_KERNEL);
	if (!pCTX) {
		iproc_err("Failed to malloc mem for SCI context!\n");
		ret = -ENOMEM;
		goto err_out;
	}
	mutex_init(&pCTX->sc_mutex);
	platform_set_drvdata(pldev, pCTX);

	if((ret = iproc_sci_resource_init(pldev)))
	{
		iproc_err("Failed to init SCI resources!\n");
		goto err_free_ctx;
	}
	
	if((ret = iproc_sci_ctx_init(0)))
	{
		iproc_err("Failed to set default SCI reg base!\n");
		goto err_free_res;
	}

	/*---------------- IRQ -----------------*/
	ret = iproc_sci_irq_init(pldev);
	if (ret)
	{
		iproc_err("Failed to init irqs!\n");
		goto err_free_res;
	}



	INIT_WORK(&iproc_sci_wq, iproc_sci_wq_irqHandling);

	/*---------- Char Device -----------*/
	if (iproc_sci_major)
	{
		pCTX->sci_devID = MKDEV(iproc_sci_major, 0);
		ret = register_chrdev_region(pCTX->sci_devID, iproc_sci_numbers, iproc_sci_name);
	}
	else
	{
		ret = alloc_chrdev_region(&pCTX->sci_devID, 0, iproc_sci_numbers, iproc_sci_name);
		iproc_sci_major = MAJOR(pCTX->sci_devID);
	}
	
	if (ret) {
		iproc_err("Failed to get chrdev major number, err:%d\n", ret);
		goto err_free_irq;
	}
	iproc_dbg("Register %d %s devices, major number is %d.\n", iproc_sci_numbers, iproc_sci_name, iproc_sci_major);

	cdev_init(&pCTX->sci_cdev, &iproc_sci_dev_fops);

	ret = cdev_add(&pCTX->sci_cdev, pCTX->sci_devID, iproc_sci_numbers);
	if (ret)
	{
		iproc_err("cannot get add chrdev, err:%d\n", ret);
		goto err_free_cdev;
	}	
	iproc_dbg("%d devices added.\n", iproc_sci_numbers);

	iproc_sci_sysfs_register(pldev);

	/*---------------- HW init -----------------*/
	// these init routines must be called after all resources initialized
	iproc_sci_set_io_pad(0);
	iprco_sci_reset();
	iproc_sci_set_sci_clock(1);
	iproc_sci_set_sci_irq(1);
#ifdef IPROC_SCI_CHECK_HW_INIT
	iproc_sci_set_gpio_program_pin();
	iproc_sci_set_sci_iomux();
	iproc_sci_set_sci_fcb(0);
#endif

	iproc_dbg("\n");

	ret = iproc_sci_init_sci(pCTX);
	if (ret)
	{
		iproc_err("cannot init sci, err:%d\n", ret);
		goto err_del_cdev;
	}
	
	iproc_prt("%s (version %s) installed.\n", DRIVER_DESCRIPTION, DRIVER_VERSION);
	iproc_dbg("Build time:%s, %s.\n", __DATE__, __TIME__);

	return 0;

err_del_cdev:
	cdev_del(&pCTX->sci_cdev);
err_free_cdev:
	unregister_chrdev_region(pCTX->sci_devID, iproc_sci_numbers);
err_free_irq:
	iproc_sci_irq_free(pldev);
	iproc_sci_set_sci_irq(0);
err_free_res:
	iproc_sci_resource_exit(pldev);
err_free_ctx:
	kfree(pCTX);
err_out:
	iproc_err("sci probe failed, err:%d\n", ret);
	return ret;
}


/**********************************************************************
 *  iproc_sci_remove
 *
 *  The Platform Driver Remove function.
 *
 *  Input parameters:
 *         device: The Device Context
 *
 *  Return value:
 *		    0: Driver Probe is Succesful
 *		not 0: ERROR
 **********************************************************************/
int __devexit iproc_sci_remove(struct platform_device *pldev)
{
	iproc_sci_sysfs_unregister(pldev);

	if (pCTX) {
		cdev_del(&pCTX->sci_cdev);
		unregister_chrdev_region(pCTX->sci_devID, iproc_sci_numbers);
		if(pCTX->irq)
			free_irq(pCTX->irq, pCTX);
	}

	iproc_sci_irq_free(pldev);
	iproc_sci_set_sci_irq(0);
	iproc_sci_resource_exit(pldev);
	kfree(pCTX);
	
	return 0;
}

static void iproc_sci_dev_release(struct device *dev)
{
}

/**********************************************************************/
static int __init iproc_sci_init(void)
{
	int err = -1;

	iproc_dbg("Loading smart card driver...\n");
	
	if(iproc_sci_numbers>MAX_INTERFACES || iproc_sci_numbers<=0)
	{
		iproc_err("Invalid `scinum', max %d SCI supported!\n", MAX_INTERFACES);
		return -EINVAL;
	}
	
	if(iproc_sci_coupler!=0 && iproc_sci_coupler!=1)
	{
		iproc_err("Invalid `coupler', 0=NXP8024, 1=NXP8026!\n");
		return -EINVAL;
	}

	err = iproc_platform_device_register(&iproc_sci_pdev);
	if(err)
	{
		iproc_err("Failed to do platform_device_register, err:%d\n", err);
		return err;
	}

	err = iproc_platform_driver_register(&iproc_sci_driver);
	if(err)
	{
		iproc_err("Failed to do platform_driver_register, err:%d\n", err);
		goto err_unregister_dev;
	}
	
	return 0;

err_unregister_dev:
	iproc_platform_device_unregister(&iproc_sci_pdev);
	iproc_dbg("Exiting Module Init\n");
	return err;
}

static void __exit iproc_sci_exit(void)
{
	iproc_dbg("\n");
	
	iproc_platform_driver_unregister(&iproc_sci_driver);
	iproc_platform_device_unregister(&iproc_sci_pdev);

	iproc_prt("%s (version %s) removed.\n", DRIVER_DESCRIPTION, DRIVER_VERSION);

	return;
}


module_init(iproc_sci_init);
module_exit(iproc_sci_exit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_AUTHOR("an.luo@broadcom.com");
MODULE_LICENSE("Proprietary");
