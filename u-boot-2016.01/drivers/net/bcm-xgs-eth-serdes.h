/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _bcm_iproc_serdes_h_
#define _bcm_iproc_serdes_h_


/* ---- Include Files ---------------------------------------------------- */
#include <common.h>
#include <asm/iproc-common/reg_utils.h>

#define PHY_REG_BLK_ADDR        0x001f

/*
 * MII Link Advertisment (Clause 37) 
 */
#define MII_ANA_C37_NP          (1 << 15)  /* Next Page */
#define MII_ANA_C37_RF_OK       (0 << 12)  /* No error, link OK */
#define MII_ANA_C37_RF_LINK_FAIL (1 << 12)  /* Offline */
#define MII_ANA_C37_RF_OFFLINE  (2 << 12)  /* Link failure */
#define MII_ANA_C37_RF_AN_ERR   (3 << 12)  /* Auto-Negotiation Error */
#define MII_ANA_C37_PAUSE       (1 << 7)   /* Symmetric Pause */
#define MII_ANA_C37_ASYM_PAUSE  (1 << 8)   /* Asymmetric Pause */
#define MII_ANA_C37_HD          (1 << 6)   /* Half duplex */
#define MII_ANA_C37_FD          (1 << 5)   /* Full duplex */ 

/* MII Control Register: bit definitions */

#define MII_CTRL_FS_2500        (1 << 5) /* Force speed to 2500 Mbps */
#define MII_CTRL_SS_MSB         (1 << 6) /* Speed select, MSb */
#define MII_CTRL_CST            (1 << 7) /* Collision Signal test */
#define MII_CTRL_FD             (1 << 8) /* Full Duplex */
#define MII_CTRL_RAN            (1 << 9) /* Restart Autonegotiation */
#define MII_CTRL_IP             (1 << 10) /* Isolate Phy */
#define MII_CTRL_PD             (1 << 11) /* Power Down */
#define MII_CTRL_AE             (1 << 12) /* Autonegotiation enable */
#define MII_CTRL_SS_LSB         (1 << 13) /* Speed select, LSb */
#define MII_CTRL_LE             (1 << 14) /* Loopback enable */
#define MII_CTRL_RESET          (1 << 15) /* PHY reset */

#define MII_CTRL_SS(_x)         ((_x) & (MII_CTRL_SS_LSB|MII_CTRL_SS_MSB))
#define MII_CTRL_SS_10          0
#define MII_CTRL_SS_100         (MII_CTRL_SS_LSB)
#define MII_CTRL_SS_1000        (MII_CTRL_SS_MSB)
#define MII_CTRL_SS_INVALID     (MII_CTRL_SS_LSB | MII_CTRL_SS_MSB)
#define MII_CTRL_SS_MASK        (MII_CTRL_SS_LSB | MII_CTRL_SS_MSB)

/* ---- External Function Prototypes ------------------------------------- */

extern void serdes_set_blk(uint phyaddr, uint blk);
extern void serdes_wr_reg(uint phyaddr, uint32_t reg, uint16_t data);
extern uint16_t serdes_rd_reg(uint phyaddr, uint32_t reg);
extern uint16_t serdes_get_id(uint phyaddr, uint off);
extern void serdes_reset(uint phyaddr);
extern int serdes_reset_core(uint phyaddr);
extern int serdes_start_pll(uint phyaddr);
extern int serdes_init(uint phyaddr);
#ifdef BCMINTERNAL
extern int serdes_enable_set(uint phyaddr, int enable);
extern int serdes_speed_set(uint phyaddr, int speed);
extern int serdes_speed_get(uint phyaddr, int *speed);
extern int serdes_lb_set(uint phyaddr, int enable);
extern void serdes_disp_status(uint phyaddr);
#endif /* BCMINTERNAL */

#endif	/* _bcm_iproc_serdes_h_ */
