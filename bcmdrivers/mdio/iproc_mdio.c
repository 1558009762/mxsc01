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

#include "iproc_mdio.h"

#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>

#include "iproc_mdio_dev.h"

static void * baseAddr;

#define R_REG(reg)       ioread32(baseAddr + (reg&0x0fff))
#define W_REG(reg, val)        iowrite32(val, baseAddr + (reg&0x0fff))

#define MII_ERR_VAL	0x0001
#define MII_MSG_VAL	0x0002
#define MII_DBG_VAL	0x0004
//static u32 mii_msg_level = MII_ERR_VAL;

#if defined(BCMDBG) || defined(BCMDBG_ERR)
#define MII_ERR(args)	do {if (mii_msg_level & MII_ERR_VAL) printk args;} while (0)
#else
#define MII_ERR(args)
#endif

#ifdef BCMDBG
#define MII_MSG(args)	do {if (mii_msg_level & MII_MSG_VAL) printk args;} while (0)
#define MII_DBG(args)	do {if (mii_msg_level & MII_DBG_VAL) printk args;} while (0)
#else
#define MII_MSG(args)
#define MII_DBG(args)
#endif

#define MII_EN_CHK  \
    {\
         if (!baseAddr) { \
             return MII_ERR_INIT; \
		 } \
         if (!(R_REG(MII_MGMT) & 0x7f)) { \
             return MII_ERR_INTERNAL; \
		 } \
    }

#define MII_TRIES 100000
#define MII_POLL_USEC                     20

struct mdio_device_data {
    mdio_info_t *mdio;
    int init;
};

static struct mdio_device_data mdio_devices={0};

#define DRIVER_VERSION          "0.01"
#define DRIVER_NAME             "iproc mdio"

static int mdio_major;
static struct cdev mdio_cdev;

#define MDIO_IOC_OP_EXTERNAL_READ     0
#define MDIO_IOC_OP_EXTERNAL_WRITE    1
#define MDIO_IOC_OP_LOCAL_READ        2
#define MDIO_IOC_OP_LOCAL_WRITE       3


struct ccb_mdiobus_private {
	/* iproc_mdiobus_data field have to  be placed at the beginning of 
	 *  mdiobus private data */
	struct iproc_mdiobus_data bus_data;
};


/* Function : ccb_mii_read
 *  - Read operation.
 * Return :
 * Note : 
 */
int
__ccb_mii_read(int dev_type, int phy_addr, int reg_off, uint16_t *data)
{
    int i;
	uint32_t ctrl = 0;
    unsigned long flags;
    mdio_info_t *mdio = NULL;

    MII_EN_CHK;

    mdio = mdio_devices.mdio;

    spin_lock_irqsave(&mdio->lock, flags);

    ctrl = R_REG(MII_MGMT);
    if (dev_type == MII_DEV_LOCAL) {
        ctrl &= ~MII_MGMT_EXP_MASK;
    } else {
        ctrl |= MII_MGMT_EXP_MASK;
    }
    W_REG(MII_MGMT, ctrl);
    MII_DBG(("MII READ: write(0x%x)=0x%x\n",MII_MGMT, ctrl));
    
	for (i = 0; i < MII_TRIES; i++) {
        ctrl = R_REG(MII_MGMT);
		if (!(ctrl & MII_MGMT_BSY_MASK)) {
		    break;
		}
		udelay(MII_POLL_USEC);
	}
	if (i >= MII_TRIES) {
		MII_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
        spin_unlock_irqrestore(&mdio->lock, flags);
		return -1;
	}

	ctrl = ((1 << MII_CMD_DATA_SB_SHIFT) & MII_CMD_DATA_SB_MASK) |
	    ((2 << MII_CMD_DATA_OP_SHIFT) & MII_CMD_DATA_OP_MASK) |
	    ((phy_addr << MII_CMD_DATA_PA_SHIFT) & MII_CMD_DATA_PA_MASK) |
	    ((reg_off << MII_CMD_DATA_RA_SHIFT) & MII_CMD_DATA_RA_MASK) |
	    ((2 << MII_CMD_DATA_TA_SHIFT) & MII_CMD_DATA_TA_MASK);
    W_REG(MII_CMD_DATA, ctrl);
    MII_DBG(("MII READ: write(0x%x)=0x%x\n",MII_CMD_DATA, ctrl));


	for (i = 0; i < MII_TRIES; i++) {
        ctrl = R_REG(MII_MGMT);
		if (!(ctrl & MII_MGMT_BSY_MASK)) {
		    break;
		}
		udelay(MII_POLL_USEC);
	}
	if (i >= MII_TRIES) {
		MII_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
        spin_unlock_irqrestore(&mdio->lock, flags);
		return -1;
	}

	ctrl = R_REG(MII_CMD_DATA);

    MII_DBG(("MDIO READ: addr=%x off=%x value=%x\n", phy_addr, reg_off, ctrl));

    spin_unlock_irqrestore(&mdio->lock, flags);

    *data = (ctrl & 0xffff);
	return 0;
}

/* Function : ccb_mii_write 
 *  - Write operation.
 * Return :
 * Note : 
 */
int
__ccb_mii_write(int dev_type, int phy_addr, int reg_off, uint16_t data)
{
    int i;
	uint32_t ctrl = 0;
    unsigned long flags;
    mdio_info_t *mdio = NULL;

    MII_DBG(("MDIO WRITE: addr=%x off=%x\n", phy_addr, reg_off));

    MII_EN_CHK;

    mdio = mdio_devices.mdio;

    spin_lock_irqsave(&mdio->lock, flags);

    ctrl = R_REG(MII_MGMT);
    if (dev_type == MII_DEV_LOCAL) {
        ctrl &= ~MII_MGMT_EXP_MASK;
    } else {
        ctrl |= MII_MGMT_EXP_MASK;
    }
    W_REG(MII_MGMT, ctrl);
    MII_DBG(("MII WRITE: write(0x%x)=0x%x\n",MII_MGMT, ctrl));

	for (i = 0; i < MII_TRIES; i++) {
        ctrl = R_REG(MII_MGMT);
		if (!(ctrl & MII_MGMT_BSY_MASK)) {
		    break;
		}
		udelay(MII_POLL_USEC);
	}
	if (i >= MII_TRIES) {
		MII_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
		spin_unlock_irqrestore(&mdio->lock, flags);
		return -1;
	}

	ctrl = ((1 << MII_CMD_DATA_SB_SHIFT) & MII_CMD_DATA_SB_MASK) |
	    ((1 << MII_CMD_DATA_OP_SHIFT) & MII_CMD_DATA_OP_MASK) |
	    ((phy_addr << MII_CMD_DATA_PA_SHIFT) & MII_CMD_DATA_PA_MASK) |
	    ((reg_off << MII_CMD_DATA_RA_SHIFT) & MII_CMD_DATA_RA_MASK) |
	    ((2 << MII_CMD_DATA_TA_SHIFT) & MII_CMD_DATA_TA_MASK) |
	    ((data << MII_CMD_DATA_DATA_SHIFT) & MII_CMD_DATA_DATA_MASK);
    W_REG(MII_CMD_DATA, ctrl);
    MII_DBG(("MII WRITE: write(0x%x)=0x%x\n",MII_CMD_DATA, ctrl));


	for (i = 0; i < MII_TRIES; i++) {
        ctrl = R_REG(MII_MGMT);
		if (!(ctrl & MII_MGMT_BSY_MASK)) {
		    break;
		}
		udelay(MII_POLL_USEC);
	}
	if (i >= MII_TRIES) {
		MII_ERR(("\n%s: BUSY stuck: ctrl=0x%x, count=%d\n", __FUNCTION__, ctrl, i));
		spin_unlock_irqrestore(&mdio->lock, flags);
		return -1;
	}

    spin_unlock_irqrestore(&mdio->lock, flags);

	return MII_ERR_NONE;
}



/* Function : ccb_mii_read
 *  - Read operation.
 * Return :
 * Note : 
 */
int
ccb_mii_read(int dev_type, int phy_addr, int reg_off, uint16_t *data)
{

#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3)

	int bustype = 0;

	if (MII_DEV_LOCAL == dev_type)
		bustype = IPROC_MDIOBUS_TYPE_INTERNAL;
	else if (MII_DEV_EXT == dev_type)
		bustype = IPROC_MDIOBUS_TYPE_EXTERNAL;
	else
		return -EINVAL;

	return iproc_mii_read(0, bustype, phy_addr, reg_off, data);

#else

	return __ccb_mii_read(dev_type, phy_addr, reg_off, data);

#endif

}


/* Function : ccb_mii_write 
 *  - Write operation.
 * Return :
 * Note : 
 */
int
ccb_mii_write(int dev_type, int phy_addr, int reg_off, uint16_t data)
{
	
#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3)
	
	int bustype = 0;
	
	if (MII_DEV_LOCAL == dev_type)
		bustype = IPROC_MDIOBUS_TYPE_INTERNAL;
	else if (MII_DEV_EXT == dev_type)
		bustype = IPROC_MDIOBUS_TYPE_EXTERNAL;
	else
		return -EINVAL;
	
	return iproc_mii_write(0, bustype, phy_addr, reg_off, data);

#else

	return __ccb_mii_write(dev_type, phy_addr, reg_off, data);

#endif

}


/* Function : ccb_mii_freq_set
 *  - Set MII management interface frequency.
 * Return :
 * Note : 
 *     
 */
int
ccb_mii_freq_set(int speed_khz)
{
    int rv = MII_ERR_NONE;
	uint32_t divider = 0;
	uint32_t mgmt = 0;
    
    MII_DBG(("MDIO FREQ SET: %d KHz\n", speed_khz));

	/* host clock 66MHz device value the MDCDIV field */
	/* resultant MDIO clock should not exceed 2.5MHz */

	if (speed_khz > 2560) {
    	MII_ERR(("\n%s: Maximum MDIO frequency is 2.5MHz\n", __FUNCTION__));
	    return MII_ERR_PARAM;
	}

	divider = 67584 / speed_khz;
	divider = (divider & MII_MGMT_MDCDIV_MASK);
	if (divider > 0x7f) {
	    /* make sure the minimum configurable frequency */
	    divider = 0x7f;
	}
    mgmt = R_REG(MII_MGMT);
	mgmt &= ~MII_MGMT_MDCDIV_MASK;
	mgmt |= divider;

    W_REG(MII_MGMT, mgmt);
    MII_DBG(("MII FREQ(%d KHz): write(0x%x)=0x%x\n",speed_khz, MII_MGMT, mgmt));
	
    return rv;
}

static void __maybe_unused
_dump_devs(void)
{
//    int r;
//    int addr, off;
    int addr;
    int phyid1, phyid2;
    int cnt = 0;
    int found = 0;

    for (addr = 0; addr <= 0x1f; addr++) {
        ccb_mii_read(MII_DEV_LOCAL, addr, 2, (uint16_t *)&phyid1);
        ccb_mii_read(MII_DEV_LOCAL, addr, 3, (uint16_t *)&phyid2);
        found = 0;
        if (phyid1 == 0xffff) {
            continue;
        }
        
        if ((phyid1) && (phyid2)) {
        	cnt ++;
        	found = 1;
        }
        if (cnt == 1) {
            printk("Found LOCAL device(s) on MDC/MDIO interface:\n");
        }
        if (found) {
            printk("PHY address=%2d, IDs = 0x%4x 0x%4x\n", addr, phyid1, phyid2);
        }
    }

    cnt = 0;
    found = 0;
    for (addr = 0; addr <= 0x1f; addr++) {
        ccb_mii_read(MII_DEV_EXT, addr, 2, (uint16_t *)&phyid1);
        ccb_mii_read(MII_DEV_EXT, addr, 3, (uint16_t *)&phyid2);
        found = 0;
        if (phyid1 == 0xffff) {
            continue;
        }
        
        if ((phyid1) && (phyid2)) {
        	cnt ++;
        	found = 1;
        }
        if (cnt == 1) {
            printk("Found EXTERNAL device(s) on MDC/MDIO interface:\n");
        }
        if (found) {
            printk("PHY address=%2d, IDs = 0x%4x 0x%4x\n", addr, phyid1, phyid2);
        }
    }
}

static int 
mdio_open(struct inode *inode, struct file *filp)
{
    filp->private_data = mdio_devices.mdio;
    return 0;
}

static int 
mdio_release(struct inode *inode, struct file *filp)
{

    return 0;
}    

static int mdio_message(mdio_info_t *mdio,
		struct mdio_ioc_transfer *u_xfers, unsigned n_xfers, int op)
{

    uint8_t pa, ra;
    uint16_t regval;

    pa = u_xfers->pa;
    ra = u_xfers->ra;

    MII_DBG(("mdio_message: op = %d\n", op));

    if(op == MDIO_IOC_OP_LOCAL_READ) {
        ccb_mii_read(MII_DEV_LOCAL, pa, ra, &regval);
        u_xfers->rx_buf = regval;
    }

    if(op == MDIO_IOC_OP_LOCAL_WRITE) {
        ccb_mii_write(MII_DEV_LOCAL, pa, ra, u_xfers->tx_buf);
    }

    if(op == MDIO_IOC_OP_EXTERNAL_READ) {
        ccb_mii_read(MII_DEV_EXT, pa, ra, &regval);
        u_xfers->rx_buf = regval;
    }

    if(op == MDIO_IOC_OP_EXTERNAL_WRITE) {
        ccb_mii_write(MII_DEV_EXT, pa, ra, u_xfers->tx_buf);
    }
    return 0;
}

static long
mdio_ioctl(struct file *filp,
                  unsigned int cmd, unsigned long arg)
{
    int         err = 0;
    int         retval = 0;
    int         ioc_op = 0;
    uint32_t      tmp;
    unsigned    n_ioc;
    struct mdio_ioc_transfer	*ioc, *uf;    
    mdio_info_t *mdio;

    MII_DBG(("mdio_ioctl: cmd = %d\n", cmd));

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != MDIO_IOC_MAGIC){
        return -ENOTTY;
    }
    
    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
            (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
            (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    mdio = (mdio_info_t *)filp->private_data;

	switch (cmd) {

    case MDIO_IOC_EXTERNAL_R_REG:
        ioc_op = MDIO_IOC_OP_EXTERNAL_READ;
        break;
    case MDIO_IOC_EXTERNAL_W_REG:        
        ioc_op = MDIO_IOC_OP_EXTERNAL_WRITE;
        break;
    case MDIO_IOC_LOCAL_R_REG:
        ioc_op = MDIO_IOC_OP_LOCAL_READ;
        break;
    case MDIO_IOC_LOCAL_W_REG:        
        ioc_op = MDIO_IOC_OP_LOCAL_WRITE;
        break;
    }

    tmp = _IOC_SIZE(cmd);
    if ((tmp % sizeof(struct mdio_ioc_transfer)) != 0) {
        retval = -EINVAL;
        return retval;
    }
    n_ioc = tmp / sizeof(struct mdio_ioc_transfer);
    if (n_ioc == 0)
        return 0;

    /* copy into scratch area */
    ioc = kmalloc(tmp, GFP_KERNEL);
    if (!ioc) {
        retval = -ENOMEM;
        return retval;
    }
    if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
        kfree(ioc);
        retval = -EFAULT;
        return retval;
    }    
    /* translate to mdio_message, execute */
    retval = mdio_message(mdio, ioc, n_ioc, ioc_op);   

    if ((ioc_op == MDIO_IOC_OP_EXTERNAL_READ) || (ioc_op == MDIO_IOC_OP_LOCAL_READ)) {
    
        uf = (struct mdio_ioc_transfer *)arg;
        if (__copy_to_user((u8 __user *)&uf->rx_buf, (uint8_t  *)&ioc->rx_buf, 2)) {
            kfree(ioc);
            retval = -EFAULT;
            return retval;
        }
    }
    kfree(ioc);

    return 0;
}

static const struct file_operations mdio_fops = {
    .open       = mdio_open,
    .release    = mdio_release,
    .unlocked_ioctl = mdio_ioctl,	    
    .owner      = THIS_MODULE,
};

static int _mdio_handler_init(mdio_info_t **mdio)
{
    *mdio = kmalloc(sizeof(mdio_info_t), GFP_KERNEL);
    if (*mdio == NULL) {
        MII_ERR(("mdio_init: out of memory\n"));
        return -ENOMEM;
    }
    memset(*mdio, 0, sizeof(mdio_info_t));

	/* Initialize lock */
	spin_lock_init(&(*mdio)->lock);
    
    
    mdio_devices.mdio = *mdio;
    mdio_devices.init = 1;

    return 0;
}


static int ccb_mdiobus_read(struct mii_bus *bus, int phy_id, int regnum)
{
	struct ccb_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = &bus_priv->bus_data;
	uint16_t data;
	int err;

	if (IPROC_MDIOBUS_TYPE_INTERNAL == bus_data->phybus_type)
		err = __ccb_mii_read(MII_DEV_LOCAL, phy_id, regnum, &data);
	else if (IPROC_MDIOBUS_TYPE_EXTERNAL == bus_data->phybus_type)
		err = __ccb_mii_read(MII_DEV_EXT, phy_id, regnum, &data);
	else
		err = -EINVAL;

	if (err < 0)
		return err;
	else 
		return data;
}

static int ccb_mdiobus_write(struct mii_bus *bus, int phy_id,
				int regnum, u16 val)
{
	struct ccb_mdiobus_private *bus_priv = bus->priv;
	struct iproc_mdiobus_data *bus_data = &bus_priv->bus_data;
	int err;

	if (IPROC_MDIOBUS_TYPE_INTERNAL == bus_data->phybus_type)
		err = __ccb_mii_write(MII_DEV_LOCAL, phy_id, regnum, val);
	else if (IPROC_MDIOBUS_TYPE_EXTERNAL == bus_data->phybus_type)
		err = __ccb_mii_write(MII_DEV_EXT, phy_id, regnum, val);
	else
		err = -EINVAL;

	return err;
}

static void __maybe_unused ccb_mdiobus_test(struct mii_bus *mii_bus)
{
	int i, nRet1, nRet2;
	u16 data1 = 0, data2 = 0;
	struct phy_device *phy_dev;
	struct ccb_mdiobus_private *bus_priv = mii_bus->priv;
	struct iproc_mdiobus_data *bus_data = &bus_priv->bus_data;

	dev_info(mii_bus->parent, "%s : %s phy bus num[%d], type[%d]\n", 
		__func__, mii_bus->id, bus_data->phybus_num, bus_data->phybus_type);

	/* Check if mdiobus_read works fine */
	for (i=0; i<PHY_MAX_ADDR; i++)
	{
		phy_dev = mii_bus->phy_map[i];
		if (phy_dev) {
			dev_info(mii_bus->parent, "phy[%d] id=0x%08x, addr = %d\n",
				i, phy_dev->phy_id, phy_dev->addr);
			nRet1 = phy_read(phy_dev, 2);
			nRet2 = phy_read(phy_dev, 3);
			if ((nRet1 < 0) || (nRet2 < 0))
				dev_info(mii_bus->parent, 
					"phy_read failed!, %s, nRet1 = %d, nRet2 = %d\n",
					dev_name(&phy_dev->dev), nRet1, nRet2);
			else
				dev_info(mii_bus->parent,
					"%s: reg2 = 0x%x, reg3 = 0x%x\n",
					dev_name(&phy_dev->dev), nRet1, nRet2);
		}
	}

	/* Check if general interface function for mdiobus read works fine */
	for (i=0; i<PHY_MAX_ADDR; i++)
	{
		nRet1 = iproc_mii_read(bus_data->logbus_num, bus_data->logbus_type, i, 2, &data1);
		nRet2 = iproc_mii_read(bus_data->logbus_num, bus_data->logbus_type, i, 3, &data2);
		if ((nRet1 < 0) || (nRet2 < 0))
			dev_info(mii_bus->parent, 
				"iproc_mdiobus_read failed!, %s phy bus num[%d], type[%d], phyaddr = %d, nRet1 = %d, nRet2 = %d\n",
				mii_bus->id, bus_data->phybus_num, bus_data->phybus_type, i, nRet1, nRet2);
		else
			dev_info(mii_bus->parent,
				"read %s phy bus num[%d] type[%d] phyaddr[%d], reg2 = 0x%x, reg3 = 0x%x\n",
				mii_bus->id, bus_data->logbus_num, bus_data->logbus_type, i, data1, data2);
	}
}

static int __devinit ccb_mdiobus_probe(struct platform_device *pdev)
{
	struct mii_bus *mii_bus;
	struct ccb_mdiobus_private *bus_priv;
	struct iproc_mdiobus_data *bus_data = pdev->dev.platform_data;
	int ret;

	mii_bus = mdiobus_alloc_size(sizeof(*bus_priv));
	if (!mii_bus)
		return -ENOMEM;

	mii_bus->name = "iproc_ccb_mdiobus";
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, IPROC_MDIO_ID_FMT, 
		bus_data->logbus_num, bus_data->logbus_type);
	mii_bus->parent = &pdev->dev;
	mii_bus->read = ccb_mdiobus_read;
	mii_bus->write = ccb_mdiobus_write;

	bus_priv = mii_bus->priv;
	memcpy(&bus_priv->bus_data, bus_data, sizeof(struct iproc_mdiobus_data));

	ret = mdiobus_register(mii_bus);
	if (ret) {
		dev_err(&pdev->dev, "mdiobus_register failed\n");
		goto err_exit;
	}

/*	
	ret = iproc_mdiobus_register(mii_bus);
	if (ret) {
		dev_err(&pdev->dev, "iproc_mdiobus_register failed\n");
		goto err_exit_unregister;
	}
*/

	platform_set_drvdata(pdev, mii_bus);


	/* ccb_mdiobus_test(mii_bus); */
	

	return 0;
/*
err_exit_unregister:
	mdiobus_unregister(mii_bus);
*/	
err_exit:
	kfree(mii_bus);
	return ret;
}

static int __devexit ccb_mdiobus_remove(struct platform_device *pdev)
{
	struct mii_bus *mii_bus = platform_get_drvdata(pdev);

	if (mii_bus)
	{
/*	
		iproc_mdiobus_unregister(mii_bus);
*/
		mdiobus_unregister(mii_bus);
		mdiobus_free(mii_bus);
	}

	return 0;
}

static struct platform_driver iproc_ccb_mdiobus_driver = 
{
	.driver = {
		.name = "iproc_ccb_mdio",
		.owner = THIS_MODULE,
	},
	.probe   = ccb_mdiobus_probe,
	.remove  = ccb_mdiobus_remove,
};


/* Function : ccb_mii_init
 *  - Init Northstar CCB MII management interface.
 * Return :
 * Note : 
 *     
 */
int
ccb_mii_init(void)
{
    int ret = -ENODEV;
    dev_t mdio_dev;
    mdio_info_t *mdio=NULL;

    _mdio_handler_init(&mdio);
    
    /* Get register base address */
	baseAddr = ioremap(IPROC_CCB_MDIO_REG_BASE, 0x1000);
    MII_DBG(("MDIO INIT: baseAddr %x\n",baseAddr));

    /* Set preamble */
    W_REG(MII_MGMT, MII_MGMT_PRE_MASK);
    /* Set the MII default clock 1MHz */
    ccb_mii_freq_set(1024);

//    _dump_devs();
    
    if(mdio_devices.init != 1) {
        return -ENOMEM;
    }
    mdio = mdio_devices.mdio;

    if (mdio_major) {
        mdio_dev = MKDEV(mdio_major, 0);
        ret = register_chrdev_region(mdio_dev, 
                        1, "mdio");
    } else {        
        ret = alloc_chrdev_region(&mdio_dev, 0, 
                        1, "mdio");
        mdio_major = MAJOR(mdio_dev);
    }

    if (ret) {
        goto error;
    }
    cdev_init(&mdio_cdev, &mdio_fops);    
    ret = cdev_add(&mdio_cdev, mdio_dev, 1);
    if (ret) {
        printk(KERN_ERR "Fail to add mdio char dev!\n");
        goto error_region;
    }

	platform_driver_register(&iproc_ccb_mdiobus_driver);
    
    return 0;

error_region:
    unregister_chrdev_region(mdio_dev, 1);  
error:
    kfree(mdio);
    return ret;
}

void
ccb_mii_exit(void)
{
    mdio_info_t *mdio=NULL;
    
    /* Get register base address */
	if (baseAddr) {
	    iounmap(baseAddr);
		baseAddr = NULL;
	}

    mdio = mdio_devices.mdio;
   	kfree(mdio);

    mdio_devices.mdio = NULL;
    mdio_devices.init = 0;
    unregister_chrdev_region(MKDEV(mdio_major, 0), 1);

	platform_driver_unregister(&iproc_ccb_mdiobus_driver);

}

module_init(ccb_mii_init);
module_exit(ccb_mii_exit);

EXPORT_SYMBOL(ccb_mii_init);
EXPORT_SYMBOL(ccb_mii_freq_set);
EXPORT_SYMBOL(ccb_mii_read);
EXPORT_SYMBOL(ccb_mii_write);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("BCM5301X MDIO Device Driver");
MODULE_LICENSE("GPL");
