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
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <plat/types.h>
#include <mach/io_map.h>
#include <linux/io.h>
#include <asm/memory.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <mach/iproc_regs.h>
#include <asm/clkdev.h>
#include "iproc_mdio.h"

#define CCG_MDIO_BASE_ADDRESS		CCB_MII_MGMT_CTL

#define MGMT_CTL_REG			0x000
#define MGMT_CTL__BYP_SHIFT		10
#define MGMT_CTL__BYP_WIDTH		1
#define MGMT_CTL__BYP_MASK		((1 << MGMT_CTL__BYP_WIDTH) - 1)
#define MGMT_CTL__EXT_SHIFT		9
#define MGMT_CTL__EXT_WIDTH		1
#define MGMT_CTL__EXT_MASK		((1 << MGMT_CTL__EXT_WIDTH) - 1)
#define MGMT_CTL__BSY_SHIFT		8
#define MGMT_CTL__BSY_WIDTH		1
#define MGMT_CTL__BSY_MASK		((1 << MGMT_CTL__BSY_WIDTH) - 1)
#define MGMT_CTL__PRE_SHIFT		7
#define MGMT_CTL__PRE_WIDTH		1
#define MGMT_CTL__PRE_MASK		((1 << MGMT_CTL__BSY_WIDTH) - 1)
#define MGMT_CTL__MDCDIV_SHIFT		0
#define MGMT_CTL__MDCDIV_WIDTH		7
#define MGMT_CTL__MDCDIV_MASK		((1 << MGMT_CTL__MDCDIV_WIDTH) - 1)

#define MGMT_CMD_DATA_REG		0x004
#define MGMT_CMD_DATA__SB_SHIFT		30
#define MGMT_CMD_DATA__SB_WIDTH		2
#define MGMT_CMD_DATA__SB_MASK		((1 << MGMT_CMD_DATA__SB_WIDTH) - 1)
#define MGMT_CMD_DATA__OP_SHIFT		28
#define MGMT_CMD_DATA__OP_WIDTH		2
#define MGMT_CMD_DATA__OP_MASK		((1 << MGMT_CMD_DATA__OP_WIDTH) - 1)
#define MGMT_CMD_DATA__PA_SHIFT		23
#define MGMT_CMD_DATA__PA_WIDTH		5
#define MGMT_CMD_DATA__PA_MASK		((1 << MGMT_CMD_DATA__PA_WIDTH) - 1)
#define MGMT_CMD_DATA__RA_SHIFT		18
#define MGMT_CMD_DATA__RA_WIDTH		5
#define MGMT_CMD_DATA__RA_MASK		((1 << MGMT_CMD_DATA__RA_WIDTH) - 1)
#define MGMT_CMD_DATA__TA_SHIFT		16
#define MGMT_CMD_DATA__TA_WIDTH		2
#define MGMT_CMD_DATA__TA_MASK		((1 << MGMT_CMD_DATA__TA_WIDTH) - 1)
#define MGMT_CMD_DATA__DATA_SHIFT	0
#define MGMT_CMD_DATA__DATA_WIDTH	16
#define MGMT_CMD_DATA__DATA_MASK	((1 << MGMT_CMD_DATA__DATA_WIDTH) - 1)


#define SET_REG_FIELD(reg_value, fshift, fmask, fvalue)	\
	(reg_value) = ((reg_value) & ~((fmask) << (fshift))) |  \
			(((fvalue) & (fmask)) << (fshift))
#define ISET_REG_FIELD(reg_value, fshift, fmask, fvalue) \
		(reg_value) = (reg_value) | (((fvalue) & (fmask)) << (fshift))			
#define GET_REG_FIELD(reg_value, fshift, fmask)	\
	(((reg_value) & ((fmask) << (fshift))) >> (fshift))

#define MII_OP_MAX_HALT_USEC	500
#define MII_OP_HALT_USEC	10

enum {
	MII_OP_MODE_READ,
	MII_OP_MODE_WRITE,
	MII_OP_MODE_MAX	
};

/**
 * struct cmicd_mdio: cmicd mdio structure
 * @resource: resource of cmicd cmc2
 * @base: base address of cmicd cmc2
 * @lock: spin lock protecting io access
 */
struct ccg_mdio_ctrl {
	void __iomem *base;
	/* Use spinlock to co-operate that the caller might be in interrupt context */
	/* struct mutex lock; */
	spinlock_t lock;    
	int ref_cnt;
};

struct ccg_mdiobus_private {
	/* iproc_mdiobus_data field have to  be placed at the beginning of 
	 *  mdiobus private data */
	struct iproc_mdiobus_data bus_data;
	struct ccg_mdio_ctrl *hw_ctrl;
};

struct ccg_mii_cmd {
	int bus_id;
	int ext_sel;
	int phy_id;
	int regnum;
	u16 op_mode;
	u16 val;
};

static struct ccg_mdio_ctrl *ccg_mdio = NULL;

static struct resource ccg_mdio_mem = {
	.name	= "ccg_mdio",
	.start 	= CCG_MDIO_BASE_ADDRESS,
	.end 	= CCG_MDIO_BASE_ADDRESS + 4096 - 1,
	.flags	= IORESOURCE_MEM,
};

static void __maybe_unused ccg_mdiobus_test(struct mii_bus *mii_bus)
{
	int i, nRet1, nRet2;
	u16 data1 = 0, data2 = 0;
	struct phy_device *phy_dev;
	struct ccg_mdiobus_private *bus_priv = mii_bus->priv;
	struct iproc_mdiobus_data *bus_data = &bus_priv->bus_data;

	dev_info(mii_bus->parent, "%s : %s phy bus num[%d], type[%d]\n", 
			__func__, mii_bus->id, bus_data->phybus_num, bus_data->phybus_type);

	/* Check if mdiobus_read works fine */
	for (i=0; i<PHY_MAX_ADDR; i++)
	{
		phy_dev = mii_bus->phy_map[i];
		if (phy_dev)
			dev_info(mii_bus->parent, "phy[%d] id=0x%08x, addr = %d\n",
				i, phy_dev->phy_id, phy_dev->addr);	
	}

	/* Check if general interface function for mdiobus read works fine */
	for (i=0; i<PHY_MAX_ADDR; i++)
	{
		nRet1 = iproc_mii_read(bus_data->logbus_num, bus_data->logbus_type, i, 2, &data1);
		nRet2 = iproc_mii_read(bus_data->logbus_num, bus_data->logbus_type, i, 3, &data2);
		if ((nRet1 < 0) || (nRet2 < 0))
			dev_info(mii_bus->parent, 
				"iproc_mdiobus_read failed!, %s phy bus num[%d], type[%d], phyaddr = %d, nRet1 = %d, nRet2 = %d\n",
				mii_bus->id, bus_data->logbus_num, bus_data->logbus_type, i, nRet1, nRet2);
		else
			dev_info(mii_bus->parent, 
				"read %s phy bus num[%d] type[%d] phyaddr[%d], reg2 = 0x%x, reg3 = 0x%x\n",
				mii_bus->id, bus_data->phybus_num, bus_data->phybus_type, i, data1, data2);
	}

}

static inline u32 ccg_mii_reg_read(struct ccg_mdio_ctrl *ccg_mii, u32 reg)
{
	return readl(ccg_mii->base + reg);
}

static inline void ccg_mii_reg_write(struct ccg_mdio_ctrl *ccg_mii, u32 reg, u32 data)
{
	writel(data, ccg_mii->base + reg);
}

static inline int ccg_mii_busy(struct ccg_mdio_ctrl *ccg_mii, int to_usec)
{
	do {
		if(!GET_REG_FIELD(ccg_mii_reg_read(ccg_mii, MGMT_CTL_REG), 
			MGMT_CTL__BSY_SHIFT, MGMT_CTL__BSY_MASK))
			return 0;
		udelay(MII_OP_HALT_USEC);
		to_usec -= MII_OP_HALT_USEC;
	} while (to_usec > 0);

	return 1;
}

static int do_ccg_mii_op(struct ccg_mdio_ctrl *ccg_mii, struct ccg_mii_cmd *cmd)
{
	u32 cmd_data = 0, mgt_ctrl;
	unsigned long flags;
	int ret = 0;

	if (MII_OP_MODE_WRITE == cmd->op_mode) {
		ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__OP_SHIFT, 
				MGMT_CMD_DATA__OP_MASK, 1);
		ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__DATA_SHIFT,
				MGMT_CMD_DATA__DATA_MASK, cmd->val);
	}
	else if (MII_OP_MODE_READ == cmd->op_mode) {
		ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__OP_SHIFT, 
				MGMT_CMD_DATA__OP_MASK, 2);
	}
	else {
		printk(KERN_ERR "%s : invald operation %d\n", __func__, cmd->op_mode);
		return -EINVAL;
	}

	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__PA_SHIFT, 
			MGMT_CMD_DATA__PA_MASK, cmd->phy_id);
	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__RA_SHIFT,
			MGMT_CMD_DATA__RA_MASK, cmd->regnum);
	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__TA_SHIFT,
			MGMT_CMD_DATA__TA_MASK, 2);
	ISET_REG_FIELD(cmd_data, MGMT_CMD_DATA__SB_SHIFT,
			MGMT_CMD_DATA__SB_MASK, 1);

	/* mutex_lock(&ccg_mii->lock); */	
	spin_lock_irqsave(&ccg_mii->lock, flags);

	if (ccg_mii_busy(ccg_mii, MII_OP_MAX_HALT_USEC)) {
		ret = -EBUSY;
		printk(KERN_ERR "%s : bus busy (1)\n", __func__);
		goto err_exit_unlock;
	}

	mgt_ctrl = ccg_mii_reg_read(ccg_mii, MGMT_CTL_REG);
	if (cmd->ext_sel != GET_REG_FIELD(mgt_ctrl, MGMT_CTL__EXT_SHIFT, 
						MGMT_CTL__EXT_MASK)) {
		SET_REG_FIELD(mgt_ctrl, MGMT_CTL__EXT_SHIFT, MGMT_CTL__EXT_MASK, cmd->ext_sel);
		ccg_mii_reg_write(ccg_mii, MGMT_CTL_REG, mgt_ctrl);
	}

	ccg_mii_reg_write(ccg_mii, MGMT_CMD_DATA_REG, cmd_data);

	if (ccg_mii_busy(ccg_mii, MII_OP_MAX_HALT_USEC)) {
		ret = -EBUSY;
		printk(KERN_ERR "%s : bus busy (2)\n", __func__);
		goto err_exit_unlock;
	}

	if (MII_OP_MODE_READ == cmd->op_mode) {
		ret = GET_REG_FIELD(ccg_mii_reg_read(ccg_mii, MGMT_CMD_DATA_REG), 
			MGMT_CMD_DATA__DATA_SHIFT, MGMT_CMD_DATA__DATA_MASK);
	}
		
	/* mutex_unlock(&ccg_mii->lock); */
        spin_unlock_irqrestore(&ccg_mii->lock, flags);

	return ret;

err_exit_unlock:
	/* mutex_unlock(&ccg_mii->lock); */
        spin_unlock_irqrestore(&ccg_mii->lock, flags);
	return ret;
}

static int ccg_mdiobus_read(struct mii_bus *bus, int phy_id, int regnum)
{
	struct ccg_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = &bus_priv->bus_data;
	struct ccg_mii_cmd cmd = {0};

	cmd.bus_id = bus_data->phybus_num;
	if (IPROC_MDIOBUS_TYPE_EXTERNAL == bus_data->phybus_type)
		cmd.ext_sel = 1;
	cmd.phy_id = phy_id;
	cmd.regnum = regnum;
	cmd.op_mode = MII_OP_MODE_READ;

	return do_ccg_mii_op(bus_priv->hw_ctrl, &cmd);
}

static int ccg_mdiobus_write(struct mii_bus *bus, int phy_id,
				int regnum, u16 val)
{
	struct ccg_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = &bus_priv->bus_data;
	struct ccg_mii_cmd cmd = {0};

	cmd.bus_id = bus_data->phybus_num;
	if (IPROC_MDIOBUS_TYPE_EXTERNAL == bus_data->phybus_type)
		cmd.ext_sel = 1;
	cmd.phy_id = phy_id;
	cmd.regnum = regnum;
	cmd.op_mode = MII_OP_MODE_WRITE;
	cmd.val = val;
	
	return do_ccg_mii_op(bus_priv->hw_ctrl, &cmd);
}

static struct ccg_mdio_ctrl * ccg_mdio_res_alloc(void)
{
	if (!ccg_mdio) {
		ccg_mdio = kzalloc(sizeof(*ccg_mdio), GFP_KERNEL);
		if (!ccg_mdio)
			return NULL;
		
		ccg_mdio->base = ioremap(ccg_mdio_mem.start, 
					resource_size(&ccg_mdio_mem));
		if (!ccg_mdio->base) {
			kfree(ccg_mdio);
			ccg_mdio = NULL;
			return NULL;
		}

		/* mutex_init(&ccg_mdio->lock); */		
		spin_lock_init(&ccg_mdio->lock);
		ccg_mdio->ref_cnt = 1;		
	}
	else
		ccg_mdio->ref_cnt ++;

	return ccg_mdio;
}

static void ccg_mdio_res_free(struct ccg_mdio_ctrl *ctrl)
{
	if (ctrl) {
		ctrl->ref_cnt --;
		if (ctrl->ref_cnt == 0) {
			iounmap(ctrl->base);
			kfree(ctrl);
			ccg_mdio = NULL;
		}
	}
}

static void ccg_mii_init(struct ccg_mdio_ctrl *ccg_mii)
{
	struct clk *clk;
	u32 clk_rate, val = 0;
	
	if(ccg_mii->ref_cnt == 1) {
		/* Set preamble enabled */
		ISET_REG_FIELD(val, MGMT_CTL__PRE_SHIFT, MGMT_CTL__PRE_MASK, 1);
		
		/* Set the MII default clock to 1MHz */
		clk = clk_get_sys("iproc_slow", "c_clk125");
		if(clk)
			clk_rate = clk_get_rate(clk) ;
		else {
			printk(KERN_ERR "%s : clk_get_sys failed\n", __func__);
			return;
		}
		/*
		 * MII Mgt Clock (MDC) Divisor. 0x0: Disable output of the MDC 
		 * Non-zero: Output the MDC with a frequency that is 
		 * PCLK/(2* the value of this field).
		 */
		ISET_REG_FIELD(val, MGMT_CTL__MDCDIV_SHIFT, MGMT_CTL__MDCDIV_MASK, 
				(clk_rate >> 21));

		ccg_mii_reg_write(ccg_mii, MGMT_CTL_REG, val);
	}
}

static int __devinit ccg_mdiobus_probe(struct platform_device *pdev)
{
	struct mii_bus *mii_bus;
	struct ccg_mdiobus_private *bus_priv;
	struct iproc_mdiobus_data *bus_data = pdev->dev.platform_data;
	struct ccg_mdio_ctrl *ccg_ctrl;
	int ret;

	ccg_ctrl = ccg_mdio_res_alloc();
	if (!ccg_ctrl) {
		dev_err(&pdev->dev, "ccg mdio res alloc failed\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	
	ccg_mii_init(ccg_ctrl);
	
	mii_bus = mdiobus_alloc_size(sizeof(*bus_priv));
	if (!mii_bus) {
		dev_err(&pdev->dev, "mdiobus alloc filed\n");
		ret = -ENOMEM;
		goto err_free_ctrl;
	}
	
	mii_bus->name = "iproc_ccg_mdiobus";	
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, IPROC_MDIO_ID_FMT, 
		bus_data->logbus_num, bus_data->logbus_type);
	mii_bus->parent = &pdev->dev;
	mii_bus->read = ccg_mdiobus_read;
	mii_bus->write = ccg_mdiobus_write;

	bus_priv = mii_bus->priv;
	memcpy(&bus_priv->bus_data, bus_data, sizeof(struct iproc_mdiobus_data));
	bus_priv->hw_ctrl = ccg_ctrl;

	ret = mdiobus_register(mii_bus);
	if (ret) {
		dev_err(&pdev->dev, "mdiobus_register failed\n");
		goto err_free_bus;
	}

/*		
	ret = iproc_mdiobus_register(mii_bus);
	if (ret) {
		dev_err(&pdev->dev, "iproc_mdiobus_register failed\n");
		goto err_unregister_bus;
	}
*/	

	platform_set_drvdata(pdev, mii_bus);


	return 0;

/*
err_unregister_bus:
	mdiobus_unregister(mii_bus);
*/	
err_free_bus:
	kfree(mii_bus);
err_free_ctrl:
	ccg_mdio_res_free(ccg_ctrl);
err_exit:
	return ret;
}

static int __devexit ccg_mdiobus_remove(struct platform_device *pdev)
{
	struct mii_bus *mii_bus = platform_get_drvdata(pdev);
	struct ccg_mdiobus_private *bus_priv;

	if (mii_bus)
	{	
		bus_priv = mii_bus->priv;
/*		
		iproc_mdiobus_unregister(mii_bus);
*/		
		mdiobus_unregister(mii_bus);
		if (bus_priv)
			ccg_mdio_res_free(bus_priv->hw_ctrl);
		mdiobus_free(mii_bus);
	}

	return 0;
}


static struct platform_driver iproc_ccg_mdiobus_driver = 
{
	.driver = {
		.name = "iproc_ccg_mdio",
		.owner = THIS_MODULE,
	},
	.probe   = ccg_mdiobus_probe,
	.remove  = ccg_mdiobus_remove,
};

static int __init ccg_mdio_init(void)
{
	return platform_driver_register(&iproc_ccg_mdiobus_driver);
}

static void __exit ccg_mdio_exit(void)
{
	platform_driver_unregister(&iproc_ccg_mdiobus_driver);
}

module_init(ccg_mdio_init);
module_exit(ccg_mdio_exit);
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("iProc CCG mdio driver");
MODULE_LICENSE("GPL");

