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


#ifndef	_bcm5301x_ccb_mii_h_
#define	_bcm5301x_ccb_mii_h_

#include <linux/phy.h>

typedef struct _mdio_info_s {
    void	*h;			/* dev handle */
    spinlock_t lock;    
} mdio_info_t;

/* reutrn value for MII driver */
#define MII_ERR_NONE          0
#define MII_ERR_TIMEOUT       -1
#define MII_ERR_INTERNAL      -2
#define MII_ERR_PARAM         -3
#define MII_ERR_UNAVAIL       -4
#define MII_ERR_UNKNOW        -5
#define MII_ERR_INIT          -6

/* device type */
#define MII_DEV_LOCAL 0
#define MII_DEV_EXT   1

/* MII register definition */
#define  MII_MGMT                                                     0x18003000
#define  MII_MGMT_BASE                                                     0x000
#define  MII_MGMT_DATAMASK                                            0x000007ff
#define  MII_CMD_DATA                                                 0x18003004
#define  MII_CMD_DATA_BASE                                                 0x004
#define  MII_CMD_DATA_DATAMASK                                        0xffffffff

/* fields in MII_MGMT */
#define MII_MGMT_BYP_MASK		0x00000400
#define MII_MGMT_BYP_SHIFT	    10
#define MII_MGMT_EXP_MASK		0x00000200
#define MII_MGMT_EXP_SHIFT	    9
#define MII_MGMT_BSY_MASK		0x00000100
#define MII_MGMT_BSY_SHIFT	    8
#define MII_MGMT_PRE_MASK		0x00000080
#define MII_MGMT_PRE_SHIFT	    7
#define MII_MGMT_MDCDIV_MASK	0x0000007f
#define MII_MGMT_MDCDIV_SHIFT	0
/* fields in MII_CMD_DATA */
#define MII_CMD_DATA_SB_MASK		0xc0000000
#define MII_CMD_DATA_SB_SHIFT	    30
#define MII_CMD_DATA_OP_MASK		0x30000000
#define MII_CMD_DATA_OP_SHIFT	    28
#define MII_CMD_DATA_PA_MASK		0x0f800000
#define MII_CMD_DATA_PA_SHIFT	    23
#define MII_CMD_DATA_RA_MASK		0x007c0000
#define MII_CMD_DATA_RA_SHIFT	    18
#define MII_CMD_DATA_TA_MASK		0x00030000
#define MII_CMD_DATA_TA_SHIFT	    16
#define MII_CMD_DATA_DATA_MASK		0x0000ffff
#define MII_CMD_DATA_DATA_SHIFT	    0


/* external functions for SPI driver */
extern int ccb_mii_read(int dev_type, int phy_addr, int reg_off, uint16_t *data);
extern int ccb_mii_write(int dev_type, int phy_addr, int reg_off, uint16_t data);

extern int ccb_mii_freq_set(int speed_khz);
extern int ccb_mii_init(void);

/****** iProc General Interface for mdio bus support ******/
struct iproc_mdiobus_data {
	/* the mdio bus num and type from chip view */
	u8 logbus_num;
	u8 logbus_type;
	/* the actual bus num and type that mdio bus comes from */
	u8 phybus_num;
	u8 phybus_type;
	/* Note : 
	* Usually the logbus_num and logbus_type are the same as phybus_num and
	* phybus_type, but they may be different on some special cases. For example, 
	* we may use cmicd mdio external bus 2 for the iProc mdio external bus 0, 
	* this configuration could be described as phybus_num=2, phybus_type=external, 
	* logbus_num=0, logbus_type=external. From iProc's view, the Phy devices
	* for iProc AMAC should use mdiobus by logbus_num and logbus_type. But internally
	*  we'll configure the mdio core by phybus_num and phybus_type.
	*/
};

#define IPROC_MDIOBUS_TYPE_INTERNAL	0
#define IPROC_MDIOBUS_TYPE_EXTERNAL	1

#define IPROC_MDIOBUS_NUM_MAX		8
#define IPROC_MDIOBUS_TYPE_MAX		2

#define IPROC_MDIO_ID_FMT		"iproc_mii:%01x:%01x"	
					/* iproc_mii:[bus_num]:[bus_type] */

/* General interface for iProc mdio bus read/write function */
extern int iproc_mii_read(int busnum, int bustype, int phyaddr, u32 regnum, u16 *val);
extern int iproc_mii_write(int busnum, int bustype, int phyaddr, u32 regnum, u16 val);
/****** iProc General Interface for mdio bus support ******/

#endif /* _bcm5301x_ccb_mii_h_ */
