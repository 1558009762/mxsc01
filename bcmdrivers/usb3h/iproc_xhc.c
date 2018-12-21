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
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <mach/io_map.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/of.h>


struct bcm_xhci {
	struct platform_device  *xhci;
	void			*mem;
	struct device		*dev;
};


void Configure_MDIO_for_USB(void)
{
/* Configure for MII needed for usb3, has to be done before usb2 */
#define MII_CTRL_INIT_VAL     0x9a
#define MII_MGMT_CMD_DATA_VAL 0x587e8000
#define MII_MGMT_CMD_DATA_VAL1  0x582a6400
#define MII_MGMT_CMD_DATA_VAL2  0x58061000
#define MII_MGMT_CMD_DATA_VAL3  0x582EC000
#define MII_MGMT_CMD_DATA_VAL4  0x582E8000
#define MII_MGMT_CMD_DATA_VAL5  0x58069000
#define MII_MGMT_CMD_DATA_VAL6	0x587e80e0
#define MII_MGMT_CMD_DATA_VAL7	0x580a009c
#define MII_MGMT_CMD_DATA_VAL8	0x587e8040
#define MII_MGMT_CMD_DATA_VAL9	0x580a21d3
#define MII_MGMT_CMD_DATA_VAL10	0x58061003
#define MII_MGMT_CMD_DATA_VAL11	0x587e8060
#define MII_MGMT_CMD_DATA_VAL12	0x580af30d
#define MII_MGMT_CMD_DATA_VAL13	0x580e6302
#define IPROC_IDM_USB2_RESET_CONTROL	 (0x18115800)
#define IPROC_IDM_USB2_RESET_CONTROL_VA   HW_IO_PHYS_TO_VIRT(IPROC_IDM_USB2_RESET_CONTROL)
#define IPROC_IDM_USB3_RESET_CONTROL	 (0x18105800)
#define IPROC_IDM_USB3_RESET_CONTROL_VA   HW_IO_PHYS_TO_VIRT(IPROC_IDM_USB3_RESET_CONTROL)


	unsigned int chipID, revID;

	chipID = readl_relaxed(IPROC_CCA_CORE_REG_VA);
	revID = ((chipID & 0x000F0000) >> 16);
	chipID &= 0x0000FFFF;

	writel_relaxed(0x1, IPROC_IDM_USB2_RESET_CONTROL_VA);
	mdelay(10);	
	writel_relaxed(0x1, IPROC_IDM_USB3_RESET_CONTROL_VA);


	if ((chipID >= 0xCF17) || ((chipID == 0xCF12) && (revID >= 0x4)))
	{
		writel_relaxed(MII_CTRL_INIT_VAL, IPROC_CCB_MDIO_REG_VA);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL2, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL1, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL3, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL4, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(0x0, IPROC_IDM_USB2_RESET_CONTROL_VA);
		mdelay(10);	
		writel_relaxed(0x0, IPROC_IDM_USB3_RESET_CONTROL_VA);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL5, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);
		if (chipID >=  0xCF17)
		{
			writel_relaxed(MII_MGMT_CMD_DATA_VAL11, IPROC_CCB_MDIO_REG_VA+4);
			mdelay(10);	
			writel_relaxed(MII_MGMT_CMD_DATA_VAL12, IPROC_CCB_MDIO_REG_VA+4);
			mdelay(10);	
			writel_relaxed(MII_MGMT_CMD_DATA_VAL13, IPROC_CCB_MDIO_REG_VA+4);
			mdelay(10);	
		}
		writel_relaxed(MII_MGMT_CMD_DATA_VAL8, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL10, IPROC_CCB_MDIO_REG_VA+4);
	}
	else if (chipID < 0xCF17)
	{
		writel_relaxed(MII_CTRL_INIT_VAL, IPROC_CCB_MDIO_REG_VA);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL1, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL6, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL7, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL8, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL9, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL10, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);
		writel_relaxed(0x0, IPROC_IDM_USB2_RESET_CONTROL_VA);
		mdelay(10);	
		writel_relaxed(0x0, IPROC_IDM_USB3_RESET_CONTROL_VA);
	}
	else
	{
		writel_relaxed(MII_CTRL_INIT_VAL, IPROC_CCB_MDIO_REG_VA);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL2, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL1, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL3, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL4, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(0x0, IPROC_IDM_USB2_RESET_CONTROL_VA);
		mdelay(10);	
		writel_relaxed(0x0, IPROC_IDM_USB3_RESET_CONTROL_VA);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL5, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL11, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL12, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL13, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL8, IPROC_CCB_MDIO_REG_VA+4);
		mdelay(10);	
		writel_relaxed(MII_MGMT_CMD_DATA_VAL10, IPROC_CCB_MDIO_REG_VA+4);
	}
}



      
static int __devinit bcm_xhc_probe(struct platform_device *pdev)
{
        struct bcm_xhci          *bcm_xhci;
        struct device           *dev = &pdev->dev;
	struct platform_device  *xhci;
	int                     ret = -ENOMEM;

        bcm_xhci  = devm_kzalloc(dev, sizeof(*bcm_xhci), GFP_KERNEL);
        if (!bcm_xhci) {
                dev_err(dev, "not enough memory\n");
                return -ENOMEM;
        }
                
        platform_set_drvdata(pdev, bcm_xhci);

        xhci = platform_device_alloc("xhci-hcd", -1);
 	if (!xhci) {
                dev_err(bcm_xhci->dev, "couldn't allocate xHCI device\n");
                ret = -ENOMEM;
                goto err0;
        }


	dma_set_coherent_mask(&xhci->dev, dev->coherent_dma_mask);

	xhci->dev.parent        = dev;
        xhci->dev.dma_mask      = dev->dma_mask; 
        xhci->dev.dma_parms     = dev->dma_parms;
        bcm_xhci->xhci = xhci;
	bcm_xhci->dev = dev;
   
        ret = platform_device_add_resources(xhci, pdev->resource,pdev->num_resources);
        if (ret) {
                dev_err(bcm_xhci->dev, "couldn't add resources to xhci device\n");
                goto err1;
        }

        ret = platform_device_add(xhci);
        if (ret) {
                dev_err(bcm_xhci->dev, "failed to register xHCI device\n");
                goto err1;
        }

	Configure_MDIO_for_USB();
	return 0;
err1:
        platform_device_put(xhci);

err0:
	kfree(bcm_xhci);
        return ret;
}

static int __devexit bcm_xhc_remove(struct platform_device *pdev)
{
        struct bcm_xhci   *bcm_xhci = platform_get_drvdata(pdev);

        platform_device_unregister(bcm_xhci->xhci);
	kfree(bcm_xhci);
        return 0;
}

struct platform_driver bcm_xhc_driver = {
        .probe          = bcm_xhc_probe,
        .remove         = __devexit_p(bcm_xhc_remove),
        .driver         = {
                .name   = "bcm-xhci",
        },
};

module_platform_driver(bcm_xhc_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom xHC glue layer");
MODULE_LICENSE("GPL");
