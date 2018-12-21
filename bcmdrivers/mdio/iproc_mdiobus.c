#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
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
#include "iproc_mdio.h"

static struct mii_bus *iproc_mdiobus[IPROC_MDIOBUS_NUM_MAX][IPROC_MDIOBUS_TYPE_MAX] = {{0}};


static struct mii_bus* get_iproc_mdiobus(int busnum, int bustype, int phyaddr)
{
	struct device *d;
	char bus_id[MII_BUS_ID_SIZE];
	char phy_id[20];
	struct phy_device *phy_dev;

	if ((busnum < IPROC_MDIOBUS_NUM_MAX) && 
		(bustype < IPROC_MDIOBUS_TYPE_MAX))
	{
		if (NULL == iproc_mdiobus[busnum][bustype]) {
			snprintf(bus_id, MII_BUS_ID_SIZE, IPROC_MDIO_ID_FMT, busnum, bustype);
			snprintf(phy_id, 20, PHY_ID_FMT, bus_id, phyaddr);
			d = bus_find_device_by_name(&mdio_bus_type, NULL, phy_id);
			if (!d) {
				pr_err("%s : BUS[%s] PHY[%s] not found\n", __func__, bus_id, phy_id);
				return NULL;
			}
			phy_dev = to_phy_device(d);
			iproc_mdiobus[busnum][bustype] = phy_dev->bus;
		}
		
		return iproc_mdiobus[busnum][bustype];
	}

	pr_err("%s : bus num[%d], type[%d] is unregistered!\n", __func__, busnum, bustype);
	
	return NULL;
}


/**
 * iproc_mii_read - General iProc interface function for reading a given PHY register 
 			if not registered PHY interface by phy_driver_register
 * @busnum: currently we're using busnum value 0
 * @bustype: the mdio bus type, coud be IPROC_MDIOBUS_TYPE_INTERNAL or IPROC_MDIOBUS_TYPE_EXTERNAL
 * @phyaddr: the phy address
 * @regnum: register number to read, if MII_ADDR_C45 == (@regnum & MII_ADDR_C45), means a C45 request
 * @val: the address to store read value if the read operation is successful
 * 
 * Returns 0 on success, or a negative value on error.
 */
int iproc_mii_read(int busnum, int bustype, int phyaddr, u32 regnum, u16 *val)
{
	struct mii_bus *mii_bus = get_iproc_mdiobus(busnum, bustype, phyaddr);
	int err = -1;

	if (mii_bus) {
		//err = mdiobus_read(mii_bus, phyaddr, regnum);
		err = mii_bus->read(mii_bus, phyaddr, regnum);
		if (err >= 0)
			*val = err;
	}
	else
		pr_err("%s : mdioubs:%d:%d is invalid!\n", __func__, busnum, bustype);
		
	return err;
}
EXPORT_SYMBOL(iproc_mii_read);

/**
 * iproc_mii_write - General iProc interface function for writing a given PHY register
 			if not registered PHY interface by phy_driver_register
 * @busnum: currently we're using busnum value 0
 * @bustype: the mdio bus type, coud be IPROC_MDIOBUS_TYPE_INTERNAL or IPROC_MDIOBUS_TYPE_EXTERNAL
 * @phyaddr: the phy address
 * @regnum: register number to write, if MII_ADDR_C45 == (@regnum & MII_ADDR_C45), means a C45 request
 * @val: value to write to @regnum
 *
 * Returns 0 on success, or a negative value on error.
 */
int iproc_mii_write(int busnum, int bustype, int phyaddr, u32 regnum, u16 val)
{
	struct mii_bus *mii_bus = get_iproc_mdiobus(busnum, bustype, phyaddr);
	int err = -1;
	
	if (mii_bus) {
		//err = mdiobus_write(mii_bus, phyaddr, regnum, val);
		err = mii_bus->write(mii_bus, phyaddr, regnum, val);
	}
	else
		pr_err("%s : mdioubs:%d:%d is invalid!\n", __func__, busnum, bustype);

	return err;
}
EXPORT_SYMBOL(iproc_mii_write);

