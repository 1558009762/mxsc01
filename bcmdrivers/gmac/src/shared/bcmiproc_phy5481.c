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
 *
 * These routines provide access to the external phy
 *
 */

/* ---- Include Files ---------------------------------------------------- */
#include <bcmutils.h>
#include <bcmenetphy.h>
#include "../../../mdio/iproc_mdio.h"
#include "bcmiproc_phy.h"
#include "bcmiproc_phy5481.h"

/* ---- External Variable Declarations ----------------------------------- */
/* ---- External Function Prototypes ------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */
/* ---- Private Variables ------------------------------------------------ */

/* debug/trace */
//#define BCMDBG
//#define BCMDBG_ERR
#ifdef BCMDBG
#define	NET_ERROR(args) printf args
#define	NET_TRACE(args) printf args
#elif defined(BCMDBG_ERR)
#define	NET_ERROR(args) printf args
#define NET_TRACE(args)
#else
#define	NET_ERROR(args)
#define	NET_TRACE(args)
#endif /* BCMDBG */
#define	NET_REG_TRACE(args)


#ifndef ASSERT
#define ASSERT(exp)
#endif


/* ==== Public Functions ================================================= */

int
phy5481_wr_reg(uint eth_num, uint phyaddr, uint16 reg_bank,
                uint8 reg_addr, uint16 *data)
{
	uint16  wr_data=*data;
	uint16  test_reg;

	NET_TRACE(("%s enter\n", __FUNCTION__));

	NET_REG_TRACE(("%s going to write phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
		 __FUNCTION__, phyaddr, reg_bank, reg_addr, wr_data));
	//printf("%s going to write phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
	//	 __FUNCTION__, phyaddr, reg_bank, reg_addr, wr_data);

	if (reg_bank) {
		ccb_mii_read(MII_DEV_EXT, phyaddr, 0x1f, &test_reg);
		ccb_mii_write(MII_DEV_EXT, phyaddr, 0x1f, (test_reg | 0x0080));

		ccb_mii_write(MII_DEV_EXT, phyaddr, reg_addr, wr_data);

		ccb_mii_write(MII_DEV_EXT, phyaddr, 0x1f, test_reg);
    } else {
		ccb_mii_write(MII_DEV_EXT, phyaddr, reg_addr, wr_data);
	}
	return SOC_E_NONE;
}


int
phy5481_rd_reg(uint eth_num, uint phyaddr, uint16 reg_bank,
			uint8 reg_addr, uint16 *data)
{
	uint16  test_reg;

	NET_TRACE(("%s enter\n", __FUNCTION__));

	NET_REG_TRACE(("%s going to read phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x)\n",
			 __FUNCTION__, phyaddr, reg_bank, reg_addr));

	if (reg_bank) {
		ccb_mii_read(MII_DEV_EXT, phyaddr, 0x1f, &test_reg);
		ccb_mii_write(MII_DEV_EXT, phyaddr, 0x1f, (test_reg | 0x0080));

		ccb_mii_read(MII_DEV_EXT, phyaddr, reg_addr, data);

		ccb_mii_write(MII_DEV_EXT, phyaddr, 0x1f, test_reg);
	} else {
		ccb_mii_read(MII_DEV_EXT, phyaddr, reg_addr, data);
	}
	NET_REG_TRACE(("%s rd phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
			 __FUNCTION__, phyaddr, reg_bank, reg_addr, *data));
	//printf("%s rd phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
	//		 __FUNCTION__, phyaddr, reg_bank, reg_addr, *data);
	return SOC_E_NONE;
}


int
phy5481_mod_reg(uint eth_num, uint phyaddr, uint16 reg_bank,
			uint8 reg_addr, uint16 data, uint16 mask)
{
	uint16 test_reg;
	uint16  org_data, rd_data;

	NET_TRACE(("%s enter\n", __FUNCTION__));

	NET_REG_TRACE(("%s going to modify phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x) mask(0x%x)\n",
		 __FUNCTION__, phyaddr, reg_bank, reg_addr, data, mask));

	if (reg_bank) {
		ccb_mii_read(MII_DEV_EXT, phyaddr, 0x1f, &test_reg);
		ccb_mii_write(MII_DEV_EXT, phyaddr, 0x1f, (test_reg | 0x0080));

		ccb_mii_read(MII_DEV_EXT, phyaddr, reg_addr, &rd_data);
		NET_REG_TRACE(("%s rd phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
				 __FUNCTION__, phyaddr, reg_bank, reg_addr, rd_data));
		org_data = rd_data;
		rd_data &= ~(mask);
		rd_data |= data;
		ccb_mii_write(MII_DEV_EXT, phyaddr, reg_addr, rd_data);
		NET_REG_TRACE(("%s wrt phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
				 __FUNCTION__, phyaddr, reg_bank, reg_addr, rd_data));

		ccb_mii_write(MII_DEV_EXT, phyaddr, 0x1f, test_reg);
    } else { 
		ccb_mii_read(MII_DEV_EXT, phyaddr, reg_addr, &rd_data); 
		NET_REG_TRACE(("%s rd phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
				 __FUNCTION__, phyaddr, reg_bank, reg_addr, rd_data));
		org_data = rd_data;
		rd_data &= ~(mask);
		rd_data |= data; 
		ccb_mii_write(MII_DEV_EXT, phyaddr, reg_addr, rd_data); 
		NET_REG_TRACE(("%s wrt phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x) data(0x%x)\n",
				 __FUNCTION__, phyaddr, reg_bank, reg_addr, rd_data));
	}
	//printf("%s modified(0x%x to 0x%x) at phyaddr(0x%x) reg_bank(0x%x) reg_addr(0x%x)\n",
	//		 __FUNCTION__, org_data, rd_data, phyaddr, reg_bank, reg_addr);

	return SOC_E_NONE;
}


void
phy5481_ge_reset(uint eth_num, uint phyaddr)
{
	uint16 ctrl;

	NET_TRACE(("et%d: %s: phyaddr %d\n", eth_num, __FUNCTION__, phyaddr));

	/* set reset flag */
	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, &ctrl);
	ctrl |= MII_CTRL_RESET;
	phy5481_wr_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, &ctrl);

	SPINWAIT( (!phy5481_rd_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, &ctrl)
	 			&& (ctrl & MII_CTRL_RESET)), 100000);
	/* check if out of reset */
	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, &ctrl);
	if (ctrl & MII_CTRL_RESET) {
		/* timeout */
		NET_ERROR(("et%d: %s reset not complete\n", eth_num, __FUNCTION__));
	} else {
		NET_ERROR(("et%d: %s reset complete\n", eth_num, __FUNCTION__));
	}

	return;
}


/*
 * Function:    
 *  phy5481_ge_init
 * Purpose: 
 *  Initialize the PHY (MII mode) to a known good state.
 * Parameters:
 *  unit - StrataSwitch unit #.
 *  port - StrataSwitch port #. 
 * Returns: 
 *  SOC_E_XXX

 * Notes: 
 *  No synchronization performed at this level.
 */
int
phy5481_ge_init(uint eth_num, uint phyaddr)
{
    uint16  mii_ana, mii_ctrl, mii_gb_ctrl;

    /* Reset PHY */
	phy5481_ge_reset(eth_num, phyaddr);

   /* set advertized bits */
	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_ANAr_BANK, PHY_MII_ANAr_ADDR, &mii_ana);
	mii_ana |= MII_ANA_FD_100 | MII_ANA_FD_10;
	mii_ana |= MII_ANA_HD_100 | MII_ANA_HD_10;
	phy5481_wr_reg(eth_num, phyaddr, PHY_MII_ANAr_BANK, PHY_MII_ANAr_ADDR, &mii_ana);


    mii_ctrl = MII_CTRL_FD | MII_CTRL_SS_1000 | MII_CTRL_AE | MII_CTRL_RAN;
    mii_gb_ctrl = MII_GB_CTRL_ADV_1000FD | MII_GB_CTRL_PT;
    
    phy5481_wr_reg(eth_num, phyaddr, PHY_MII_GB_CTRLr_BANK, PHY_MII_GB_CTRLr_ADDR, &mii_gb_ctrl);
    phy5481_wr_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, &mii_ctrl);
    

	return SOC_E_NONE;
}



void
phy5481_reset_setup(uint eth_num, uint phyaddr)
{

	NET_TRACE(("%s enter\n", __FUNCTION__));

    phy5481_ge_init(eth_num, phyaddr);

    /* copper regs */
    /* remove power down */
	phy5481_mod_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, 0, MII_CTRL_PD);
    /* Disable super-isolate */
	phy5481_mod_reg(eth_num, phyaddr, PHY_MII_POWER_CTRLr_BANK, PHY_MII_POWER_CTRLr_ADDR, 0, PHY5481_SUPER_ISOLATE_MODE);
    /* Enable extended packet length */
	phy5481_mod_reg(eth_num, phyaddr, PHY_MII_AUX_CTRLr_BANK, PHY_MII_AUX_CTRLr_ADDR, 0x4000, 0x4000);

    return;
}


/*
 * Function:
 *      phy5481_init
 * Purpose:
 *      Initialize xgxs6 phys
 * Parameters:
 *      eth_num - ethernet data
 *      phyaddr - physical address
 * Returns:
 *      0
 */
int
phy5481_init(uint eth_num, uint phyaddr)
{
	uint16	phyid0, phyid1;

	NET_TRACE(("et%d: %s: phyaddr %d\n", eth_num, __FUNCTION__, phyaddr));

	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_PHY_ID0r_BANK, PHY_MII_PHY_ID0r_ADDR, &phyid0);
	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_PHY_ID1r_BANK, PHY_MII_PHY_ID1r_ADDR, &phyid1);

	printf("%s phyaddr(0x%x) Phy ChipID: 0x%04x:0x%04x\n", __FUNCTION__, phyaddr, phyid1, phyid0);

    phy5481_reset_setup(eth_num, phyaddr);

	return 0;
}


/*
 * Function:    
 *  phy5481_link_get
 * Purpose: 
 *  Determine the current link up/down status
 * Parameters:
 *  unit - StrataSwitch unit #.
 *  port - StrataSwitch port #. 
 *  link - (OUT) Boolean, true indicates link established.
 * Returns:
 *  SOC_E_XXX
 * Notes: 
 *  No synchronization performed at this level.
 */
int
phy5481_link_get(uint eth_num, uint phyaddr, int *link)
{
	uint16      mii_ctrl, mii_stat;
	uint32		wait;

    *link = FALSE;      /* Default */

	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_STATr_BANK, PHY_MII_STATr_ADDR, &mii_stat);
	/* the first read of status register will not show link up, second read will show link up */
    if (!(mii_stat & MII_STAT_LA) ) {
		phy5481_rd_reg(eth_num, phyaddr, PHY_MII_STATr_BANK, PHY_MII_STATr_ADDR, &mii_stat);
	}

    if (!(mii_stat & MII_STAT_LA) || (mii_stat == 0xffff)) {
    /* mii_stat == 0xffff check is to handle removable PHY daughter cards */
        return SOC_E_NONE;
    }

    /* Link appears to be up; we are done if autoneg is off. */

	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, &mii_ctrl);

    if (!(mii_ctrl & MII_CTRL_AE)) {
		*link = TRUE;
        return SOC_E_NONE;
    }

    /*
     * If link appears to be up but autonegotiation is still in
     * progress, wait for it to complete.  For BCM5228, autoneg can
     * still be busy up to about 200 usec after link is indicated.  Also
     * continue to check link state in case it goes back down.
	 * wait 500ms (500000us/10us = 50000 )
     */
    for (wait=0; wait<50000; wait++) {

		phy5481_rd_reg(eth_num, phyaddr, PHY_MII_STATr_BANK, PHY_MII_STATr_ADDR, &mii_stat);

	    if (!(mii_stat & MII_STAT_LA)) {
			/* link is down */
	        return SOC_E_NONE;
	    }

	    if (mii_stat & MII_STAT_AN_DONE) {
			/* AutoNegotiation done */
	        break;
	    }

		OSL_DELAY(10);
    }
    if (wait>=50000) {
		/* timeout */
	    return SOC_E_BUSY;
	}

    /* Return link state at end of polling */
    *link = ((mii_stat & MII_STAT_LA) != 0);

    return SOC_E_NONE;
}

/*
 * Function:
 *      phy5481_enable_set
 * Purpose:
 *      Enable/Disable phy
 * Parameters:
 *      eth_num - ethernet data
 *      phyaddr - physical address
 *      enable - on/off state to set
 * Returns:
 *      0
 */
int
phy5481_enable_set(uint eth_num, uint phyaddr, int enable)
{
	uint16	power_down;

	NET_TRACE(("et%d: %s: phyaddr %d\n", eth_num, __FUNCTION__, phyaddr));
    
    power_down = (enable) ? 0 : MII_CTRL_PD;
    
    phy5481_mod_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, power_down, MII_CTRL_PD);

    return SOC_E_NONE;
}




/*
 * Function:     
 *    phy5481_auto_negotiate_gcd (greatest common denominator).
 * Purpose:    
 *    Determine the current greatest common denominator between 
 *    two ends of a link
 * Parameters:
 *    unit - StrataSwitch unit #.
 *    port - StrataSwitch port #. 
 *    speed - (OUT) greatest common speed.
 *    duplex - (OUT) greatest common duplex.
 *    link - (OUT) Boolean, true indicates link established.
 * Returns:    
 *    SOC_E_XXX
 * Notes: 
 *    No synchronization performed at this level.
 */
static int
phy5481_auto_negotiate_gcd(uint eth_num, uint phyaddr, int *speed, int *duplex)
{
    int        t_speed, t_duplex;
    uint16     mii_ana, mii_anp, mii_stat;
    uint16     mii_gb_stat, mii_esr, mii_gb_ctrl;

    mii_gb_stat = 0;            /* Start off 0 */
    mii_gb_ctrl = 0;            /* Start off 0 */

	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_ANAr_BANK, PHY_MII_ANAr_ADDR, &mii_ana);
	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_ANPr_BANK, PHY_MII_ANPr_ADDR, &mii_anp);
	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_STATr_BANK, PHY_MII_STATr_ADDR, &mii_stat);

    if (mii_stat & MII_STAT_ES) {    /* Supports extended status */
        /*
         * If the PHY supports extended status, check if it is 1000MB
         * capable.  If it is, check the 1000Base status register to see
         * if 1000MB negotiated.
         */
		phy5481_rd_reg(eth_num, phyaddr, PHY_MII_ESRr_BANK, PHY_MII_ESRr_ADDR, &mii_esr);

        if (mii_esr & (MII_ESR_1000_X_FD | MII_ESR_1000_X_HD | 
                       MII_ESR_1000_T_FD | MII_ESR_1000_T_HD)) {
			phy5481_rd_reg(eth_num, phyaddr, PHY_MII_GB_STATr_BANK, PHY_MII_GB_STATr_ADDR, &mii_gb_stat);
			phy5481_rd_reg(eth_num, phyaddr, PHY_MII_GB_CTRLr_BANK, PHY_MII_GB_CTRLr_ADDR, &mii_gb_ctrl);
        }
    }

    /*
     * At this point, if we did not see Gig status, one of mii_gb_stat or 
     * mii_gb_ctrl will be 0. This will cause the first 2 cases below to 
     * fail and fall into the default 10/100 cases.
     */

    mii_ana &= mii_anp;

    if ((mii_gb_ctrl & MII_GB_CTRL_ADV_1000FD) &&
        (mii_gb_stat & MII_GB_STAT_LP_1000FD)) {
        t_speed  = 1000;
        t_duplex = 1;
    } else if ((mii_gb_ctrl & MII_GB_CTRL_ADV_1000HD) &&
               (mii_gb_stat & MII_GB_STAT_LP_1000HD)) {
        t_speed  = 1000;
        t_duplex = 0;
    } else if (mii_ana & MII_ANA_FD_100) {         /* [a] */
        t_speed = 100;
        t_duplex = 1;
    } else if (mii_ana & MII_ANA_T4) {            /* [b] */
        t_speed = 100;
        t_duplex = 0;
    } else if (mii_ana & MII_ANA_HD_100) {        /* [c] */
        t_speed = 100;
        t_duplex = 0;
    } else if (mii_ana & MII_ANA_FD_10) {        /* [d] */
        t_speed = 10;
        t_duplex = 1 ;
    } else if (mii_ana & MII_ANA_HD_10) {        /* [e] */
        t_speed = 10;
        t_duplex = 0;
    } else {
        return(SOC_E_FAIL);
    }

    if (speed)  *speed  = t_speed;
    if (duplex)    *duplex = t_duplex;

    return(SOC_E_NONE);
}


/*
 * Function:
 *      phy5481_speed_get
 * Purpose:
 *      Get PHY speed
 * Parameters:
 *      eth_num - ethernet data
 *      phyaddr - physical address
 *      speed - current link speed in Mbps
 * Returns:
 *      0
 */
int
phy5481_speed_get(uint eth_num, uint phyaddr, int *speed, int *duplex)
{
    int     rv;
    uint16  mii_ctrl, mii_stat;

	NET_TRACE(("et%d: %s: phyaddr %d\n", eth_num, __FUNCTION__, phyaddr));

	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_CTRLr_BANK, PHY_MII_CTRLr_ADDR, &mii_ctrl);
	phy5481_rd_reg(eth_num, phyaddr, PHY_MII_STATr_BANK, PHY_MII_STATr_ADDR, &mii_stat);

    *speed = 0;
    *duplex = 0;
    if (mii_ctrl & MII_CTRL_AE) {   /* Auto-negotiation enabled */
        if (!(mii_stat & MII_STAT_AN_DONE)) { /* Auto-neg NOT complete */
            rv = SOC_E_NONE;
        } else {
	        rv = phy5481_auto_negotiate_gcd(eth_num, phyaddr, speed, duplex);
		}
    } else {                /* Auto-negotiation disabled */
	    /*
	     * Simply pick up the values we force in CTRL register.
	     */
		if (mii_ctrl & MII_CTRL_FD)
			*duplex = 1;

	    switch(MII_CTRL_SS(mii_ctrl)) {
	    case MII_CTRL_SS_10:
	        *speed = 10;
	        break;
	    case MII_CTRL_SS_100:
	        *speed = 100;
	        break;
	    case MII_CTRL_SS_1000:
	        *speed = 1000;
	        break;
	    default:            /* Just pass error back */
	        return(SOC_E_UNAVAIL);
	    }
    	rv = SOC_E_NONE;
    }

    return(rv);
}
