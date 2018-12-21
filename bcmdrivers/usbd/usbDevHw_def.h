/*****************************************************************************
* Copyright 2003 - 2009 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*****************************************************************************/

#ifndef USBDEVHW_DEF_H
#define USBDEVHW_DEF_H

#ifndef USBDEVHW_H
#error *** Do not include usbDevHw_def.h directly. Use usbDevHw.h instead. ***
#endif

#ifdef __cplusplus
extern "C"
{
#endif
//#define DBG_USBDEVHW_DEF 1
/* ---- Include Files ---------------------------------------------------- */

#include <mach/io_map.h>
#include "usbDevHw_reg_dwc_d20ahb.h"

/* ---- Public Constants and Types --------------------------------------- */

#define usbDevHw_DEVICE_IRQ_REMOTEWAKEUP_DELTA	  usbDevHw_REG_DEV_INTR_REMOTE_WAKEUP_DELTA
#define usbDevHw_DEVICE_IRQ_SPEED_ENUM_DONE		 usbDevHw_REG_DEV_INTR_SPD_ENUM_DONE
#define usbDevHw_DEVICE_IRQ_SOF_DETECTED			usbDevHw_REG_DEV_INTR_SOF_RX
#define usbDevHw_DEVICE_IRQ_BUS_SUSPEND			 usbDevHw_REG_DEV_INTR_BUS_SUSPEND
#define usbDevHw_DEVICE_IRQ_BUS_RESET			   usbDevHw_REG_DEV_INTR_BUS_RESET
#define usbDevHw_DEVICE_IRQ_BUS_IDLE				usbDevHw_REG_DEV_INTR_BUS_IDLE
#define usbDevHw_DEVICE_IRQ_SET_INTF				usbDevHw_REG_DEV_INTR_SET_INTF_RX
#define usbDevHw_DEVICE_IRQ_SET_CFG				 usbDevHw_REG_DEV_INTR_SET_CFG_RX

#define usbDevHw_ENDPT_STATUS_DMA_ERROR			 usbDevHw_REG_ENDPT_FIFO_STATUS_AHB_BUS_ERROR
#define usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL	 usbDevHw_REG_ENDPT_FIFO_STATUS_DMA_BUF_NOT_AVAIL
#define usbDevHw_ENDPT_STATUS_IN_TOKEN_RX		   usbDevHw_REG_ENDPT_FIFO_STATUS_IN_TOKEN_RX
#define usbDevHw_ENDPT_STATUS_IN_DMA_DONE		   usbDevHw_REG_ENDPT_FIFO_STATUS_IN_DMA_DONE
#define	usbDevHw_ENDPT_STATUS_IN_FIFO_EMPTY        usbDevHw_REG_ENDPT_FIFO_STATUS_IN_FIFO_EMPTY
#define usbDevHw_ENDPT_STATUS_IN_XFER_DONE		  usbDevHw_REG_ENDPT_FIFO_STATUS_IN_XFER_DONE
#define usbDevHw_ENDPT_STATUS_OUT_DMA_DATA_DONE	 usbDevHw_REG_ENDPT_FIFO_STATUS_OUT_DMA_DATA_DONE
#define usbDevHw_ENDPT_STATUS_OUT_DMA_SETUP_DONE	usbDevHw_REG_ENDPT_FIFO_STATUS_OUT_DMA_SETUP_DONE

/* Number of endpoints actually configured (supported) in RTL. This may be less than usbDevHw_REG_ENDPT_CNT. */
#define usbDevHw_ENDPT_CFG_CNT					  10

#define usbDevHw_REG_P			  ((volatile usbDevHw_REG_t *)(IPROC_USB2D_REG_VA))
#define START_USB1_P			  ((volatile usbDevHw_REG_t *)(IPROC_USB2D_REG_VA))
/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

/* ---- Inline Function Definitions -------------------------------------- */

#ifdef usbDevHw_DEBUG_REG

#define REG_WR(reg, val)			  reg32_write(&reg, val)
#define REG_MOD_OR(reg, val)		  reg32_modify_or(&reg, val)
#define REG_MOD_AND(reg, val)		 reg32_modify_and(&reg, val)
#define REG_MOD_MASK(reg, mask, val)  reg32_modify_mask(&reg, mask, val)

#else

#define REG_WR(reg, val)			  reg = val
#define REG_MOD_OR(reg, val)		  reg |= val
#define REG_MOD_AND(reg, val)		 reg &= val
#define REG_MOD_MASK(reg, mask, val)  reg = (reg & mask) | val

#endif


/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_OpsFinis(void)
{
	register unsigned endptNum;

	usbDevHw_DeviceDmaDisable();
	usbDevHw_DeviceIrqDisable(usbDevHw_DEVICE_IRQ_ALL);
	usbDevHw_DeviceIrqClear(usbDevHw_DEVICE_IRQ_ALL);

	/* @todo Create and use usbDevHw_EndptIrqListDisable?? */
	for (endptNum = 0; endptNum < usbDevHw_ENDPT_CFG_CNT; endptNum++) {
		usbDevHw_EndptIrqDisable(endptNum, usbDevHw_ENDPT_DIRN_IN);
		usbDevHw_EndptIrqClear(endptNum, usbDevHw_ENDPT_DIRN_IN);
		usbDevHw_EndptStatusClear(endptNum, usbDevHw_ENDPT_DIRN_IN, usbDevHw_EndptStatusActive(endptNum, usbDevHw_ENDPT_DIRN_IN));

		usbDevHw_EndptIrqDisable(endptNum, usbDevHw_ENDPT_DIRN_OUT);
		usbDevHw_EndptIrqClear(endptNum, usbDevHw_ENDPT_DIRN_OUT);
		usbDevHw_EndptStatusClear(endptNum, usbDevHw_ENDPT_DIRN_OUT, usbDevHw_EndptStatusActive(endptNum, usbDevHw_ENDPT_DIRN_OUT));
	}
}

static inline void usbDevHw_OpsInit(void)
{
	/* Just in case the default state is not with interrupts, etc. disabled and cleared. */
	usbDevHw_OpsFinis();
	REG_WR(usbDevHw_REG_P->devCfg, 
		usbDevHw_REG_DEV_CFG_SET_DESCRIPTOR_ENABLE 
		| usbDevHw_REG_DEV_CFG_UTMI_8BIT_ENABLE
		| usbDevHw_REG_DEV_CFG_CSR_PROGRAM_ENABLE
		| usbDevHw_REG_DEV_CFG_SPD_HS);
		//| usbDevHw_REG_DEV_CFG_SPD_FS);

	//usbDevHw_REG_DEV_CFG_SPD_HS | usbDevHw_REG_DEV_CFG_SELF_PWR_ENABLE);

	REG_WR(usbDevHw_REG_P->devCtrl, usbDevHw_REG_DEV_CTRL_LE_ENABLE 
			/*^ usbDevHw_REG_DEV_CTRL_DISCONNECT_ENABLE */
			| usbDevHw_REG_DEV_CTRL_DISCONNECT_ENABLE
			| usbDevHw_REG_DEV_CTRL_DMA_MODE_ENABLE
			| usbDevHw_REG_DEV_CTRL_DMA_IN_ENABLE
			| usbDevHw_REG_DEV_CTRL_DMA_OUT_ENABLE
			| usbDevHw_REG_DEV_CTRL_DMA_DESC_UPDATE_ENABLE
			/*
			 *^			 | usbDevHw_REG_DEV_CTRL_DMA_DESC_UPDATE_ENABLE
			 *^			 | usbDevHw_REG_DEV_CTRL_CSR_DONE
			 */
			| usbDevHw_REG_DEV_CTRL_OUT_NAK_ALL_ENABLE
			| usbDevHw_REG_DEV_CTRL_DMA_OUT_THRESHOLD_LEN_MASK
			| usbDevHw_REG_DEV_CTRL_DMA_BURST_LEN_MASK
			| usbDevHw_REG_DEV_CTRL_DMA_BURST_ENABLE
#if !usbDevHw_REG_MULTI_RX_FIFO
			| usbDevHw_REG_DEV_CTRL_OUT_FIFO_FLUSH_ENABLE
#endif
			);
	REG_WR(usbDevHw_REG_P->devIntrMask, usbDevHw_DEVICE_IRQ_BUS_IDLE
					  |usbDevHw_DEVICE_IRQ_SOF_DETECTED); 
	REG_WR(usbDevHw_REG_P->eptIntrMask,0);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_DeviceBusConnect(void)
{
#ifdef DBG_USBDEVHW_DEF
	printk("usbDevHw_DeviceBusConnect: enter\n");
#endif
	REG_MOD_AND(usbDevHw_REG_P->devCtrl, ~usbDevHw_REG_DEV_CTRL_DISCONNECT_ENABLE);
}

static inline void usbDevHw_DeviceBusDisconnect(void)
{
#ifdef DBG_USBDEVHW_DEF
	printk("usbDevHw_DeviceBusDisconnect: enter\n");
#endif
	REG_MOD_OR(usbDevHw_REG_P->devCtrl, usbDevHw_REG_DEV_CTRL_DISCONNECT_ENABLE);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline bool usbDevHw_DeviceBusSuspended(void)
{
	return(usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_BUS_SUSPENDED ? true : false);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline unsigned usbDevHw_DeviceAltNum(void)
{
	return((usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_ALT_NUM_MASK) >>  usbDevHw_REG_DEV_STATUS_ALT_NUM_SHIFT);
}

static inline unsigned usbDevHw_DeviceCfgNum(void)
{
	return((usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_CFG_NUM_MASK) >>  usbDevHw_REG_DEV_STATUS_CFG_NUM_SHIFT);
}

static inline unsigned usbDevHw_DeviceIntfNum(void)
{
	return((usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_INTF_NUM_MASK) >>  usbDevHw_REG_DEV_STATUS_INTF_NUM_SHIFT);
}


/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_DeviceDmaDisable(void)
{
	REG_MOD_AND(usbDevHw_REG_P->devCtrl, ~(usbDevHw_REG_DEV_CTRL_DMA_IN_ENABLE | usbDevHw_REG_DEV_CTRL_DMA_OUT_ENABLE));
}

static inline void usbDevHw_DeviceDmaEnable(void)
{
#if usbDevHw_REG_MULTI_RX_FIFO
	REG_MOD_OR(usbDevHw_REG_P->devCtrl, (usbDevHw_REG_DEV_CTRL_DMA_IN_ENABLE | usbDevHw_REG_DEV_CTRL_DMA_OUT_ENABLE));
#else
	/*printk(KERN_ERR "udc: RX DMA ENABLE (global)\n"); */
	REG_MOD_OR(usbDevHw_REG_P->devCtrl, (usbDevHw_REG_DEV_CTRL_DMA_IN_ENABLE | usbDevHw_REG_DEV_CTRL_DMA_OUT_ENABLE));
#endif
}

static inline bool usbDevHw_DeviceDmaEnabled(void)
{
	return (usbDevHw_REG_P->devCtrl & usbDevHw_REG_DEV_CTRL_DMA_OUT_ENABLE ? true : false);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline unsigned usbDevHw_DeviceFrameNumLastRx(void)
{
	return((usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_SOF_FRAME_NUM_MASK) >> usbDevHw_REG_DEV_STATUS_SOF_FRAME_NUM_SHIFT);
}

/*****************************************************************************
* See bcm5892_reg.h for GPIO addresses
*****************************************************************************/
/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline uint32_t usbDevHw_DeviceIrqActive(void)
{
	return(usbDevHw_REG_P->devIntrStatus);
}

static inline void usbDevHw_DeviceIrqClear(uint32_t mask)
{
	REG_WR(usbDevHw_REG_P->devIntrStatus, mask);
}

static inline void usbDevHw_DeviceIrqDisable(uint32_t mask)
{
	REG_MOD_OR(usbDevHw_REG_P->devIntrMask, mask);
}

static inline void usbDevHw_DeviceIrqEnable(uint32_t mask)
{
	REG_MOD_AND(usbDevHw_REG_P->devIntrMask, ~mask);
}
static inline uint32_t usbDevHw_DeviceIrqMask(void)
{
	return((~usbDevHw_REG_P->devIntrMask) & usbDevHw_DEVICE_IRQ_ALL);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_DeviceNakAllOutEptDisable(void)
{
	/*printk(KERN_ERR "BCM5892:- %s\n",__func__); */
	REG_MOD_AND(usbDevHw_REG_P->devCtrl, ~usbDevHw_REG_DEV_CTRL_OUT_NAK_ALL_ENABLE);
}

static inline void usbDevHw_DeviceNakAllOutEptEnable(void)
{
	REG_MOD_OR(usbDevHw_REG_P->devCtrl, usbDevHw_REG_DEV_CTRL_OUT_NAK_ALL_ENABLE);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline bool usbDevHw_DevicePhyErrorDetected(void)
{
	return(usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_PHY_ERROR ? true : false);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline bool usbDevHw_DeviceRemoteWakeupAllowed(void)
{
	return(usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_REMOTE_WAKEUP_ALLOWED ? true : false);
}

static inline void usbDevHw_DeviceRemoteWakeupDisable(void)
{
	REG_MOD_AND(usbDevHw_REG_P->devCfg, ~usbDevHw_REG_DEV_CFG_REMOTE_WAKEUP_ENABLE);
}

static inline void usbDevHw_DeviceRemoteWakeupEnable(void)
{
	REG_MOD_OR(usbDevHw_REG_P->devCfg, usbDevHw_REG_DEV_CFG_REMOTE_WAKEUP_ENABLE);
}

static inline void usbDevHw_DeviceRemoteWakeupStart(void)
{
	REG_MOD_OR(usbDevHw_REG_P->devCtrl, usbDevHw_REG_DEV_CTRL_RESUME_SIGNAL_ENABLE);
}

static inline void usbDevHw_DeviceRemoteWakeupStop(void)
{
	REG_MOD_AND(usbDevHw_REG_P->devCtrl, ~usbDevHw_REG_DEV_CTRL_RESUME_SIGNAL_ENABLE);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_DeviceSelfPwrDisable(void)
{
	REG_MOD_AND(usbDevHw_REG_P->devCfg, ~usbDevHw_REG_DEV_CFG_SELF_PWR_ENABLE);
}

static inline void usbDevHw_DeviceSelfPwrEnable(void)
{
	REG_MOD_OR(usbDevHw_REG_P->devCfg, usbDevHw_REG_DEV_CFG_SELF_PWR_ENABLE);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_DeviceSetDescriptorDisable(void)
{
	REG_MOD_AND(usbDevHw_REG_P->devCfg, ~usbDevHw_REG_DEV_CFG_SET_DESCRIPTOR_ENABLE);
}

static inline void usbDevHw_DeviceSetDescriptorEnable(void)
{
	REG_MOD_OR(usbDevHw_REG_P->devCfg, usbDevHw_REG_DEV_CFG_SET_DESCRIPTOR_ENABLE);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_DeviceSetupDone(void)
{
	REG_MOD_OR(usbDevHw_REG_P->devCtrl, usbDevHw_REG_DEV_CTRL_CSR_DONE);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline unsigned usbDevHw_DeviceSpeedEnumerated(void)
{
	switch(usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_SPD_MASK)
	{
	case usbDevHw_REG_DEV_STATUS_SPD_LS:
		return(usbDevHw_DEVICE_SPEED_LOW);

	case usbDevHw_REG_DEV_STATUS_SPD_HS:
		return(usbDevHw_DEVICE_SPEED_HIGH);

	case usbDevHw_REG_DEV_STATUS_SPD_FS:
	case usbDevHw_REG_DEV_STATUS_SPD_FS_48MHZ:
		return(usbDevHw_DEVICE_SPEED_FULL);
	}

	/* This is only here to remove compiler warning. Should never get to this point. */
	return(usbDevHw_DEVICE_SPEED_FULL);
}

static inline void usbDevHw_DeviceSpeedRequested(unsigned speed)
{
	REG_MOD_AND(usbDevHw_REG_P->devCfg, ~usbDevHw_REG_DEV_CFG_SPD_MASK);

	switch(speed)
	{
	case usbDevHw_DEVICE_SPEED_LOW:
		REG_MOD_OR(usbDevHw_REG_P->devCfg, usbDevHw_REG_DEV_CFG_SPD_LS);
		break;

	case usbDevHw_DEVICE_SPEED_HIGH:
		REG_MOD_OR(usbDevHw_REG_P->devCfg, usbDevHw_REG_DEV_CFG_SPD_HS);
		break;

	case usbDevHw_DEVICE_SPEED_FULL:
	default:
		REG_MOD_OR(usbDevHw_REG_P->devCfg, usbDevHw_REG_DEV_CFG_SPD_FS);
		break;
	}
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_EndptOpsFinis(unsigned num)
{
}

#define usbDevHw_FIFO_SIZE_UINT32(maxPktSize)   (((maxPktSize) + 3) / sizeof(uint32_t))

#define usbDevHw_FIFO_SIZE_UINT8(maxPktSize)	(usbDevHw_FIFO_SIZE_UINT32(maxPktSize) * sizeof(uint32_t))

static inline void usbDevHw_EndptOpsInit(unsigned num, unsigned type, unsigned dirn, unsigned maxPktSize)
{
#ifdef DBG_USBDEVHW_DEF
	printk("%s: EP Num= %x:: EP Type= %x ::EP Dirn = %x ::Max Pkt Size=%x\n", __func__,num, type,dirn,maxPktSize);
#endif
	if ((type == usbDevHw_ENDPT_TYPE_CTRL) || (dirn == usbDevHw_ENDPT_DIRN_OUT)) {
		REG_WR(usbDevHw_REG_P->eptFifoOut[num].ctrl,   (type << usbDevHw_REG_ENDPT_FIFO_CTRL_TYPE_SHIFT)); /*| */
								  /*
								   *^usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET |
								   *^usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_FLUSH_ENABLE);
								   */
		REG_WR(usbDevHw_REG_P->eptFifoOut[num].status, usbDevHw_REG_P->eptFifoOut[num].status);
		REG_WR(usbDevHw_REG_P->eptFifoOut[num].size1,  0);
		/*^REG_WR(usbDevHw_REG_P->eptFifoOut[num].size2,  (maxPktSize << usbDevHw_REG_ENDPT_FIFO_SIZE2_PKT_MAX_SHIFT)); */
		REG_WR(usbDevHw_REG_P->eptFifoOut[num].size2,  ((maxPktSize >> 2)<< 16) | maxPktSize);
#if usbDevHw_REG_MULTI_RX_FIFO
		REG_MOD_OR(usbDevHw_REG_P->eptFifoOut[num].size2, (usbDevHw_FIFO_SIZE_UINT32(maxPktSize) << usbDevHw_REG_ENDPT_FIFO_SIZE2_OUT_DEPTH_SHIFT));
#endif
	}
	if ((type == usbDevHw_ENDPT_TYPE_CTRL) || (dirn == usbDevHw_ENDPT_DIRN_IN)) {
		REG_WR(usbDevHw_REG_P->eptFifoIn[num].ctrl,  (type << usbDevHw_REG_ENDPT_FIFO_CTRL_TYPE_SHIFT));/* | */
						/*
						 * ^usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET |
						 *^ usbDevHw_REG_ENDPT_FIFO_CTRL_IN_FLUSH_ENABLE);
						 */
		/*^ REG_WR(usbDevHw_REG_P->eptFifoIn[num].status, usbDevHw_REG_P->eptFifoIn[num].status); */
		REG_WR(usbDevHw_REG_P->eptFifoIn[num].size2,  (maxPktSize << usbDevHw_REG_ENDPT_FIFO_SIZE2_PKT_MAX_SHIFT));
		REG_WR(usbDevHw_REG_P->eptFifoIn[num].size1,  (maxPktSize >> 2));
		REG_MOD_OR(usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_IN_FLUSH_ENABLE); 
		REG_MOD_AND(usbDevHw_REG_P->eptFifoIn[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_IN_FLUSH_ENABLE); 
		REG_MOD_AND(usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET); 
	}
#ifdef DBG_USBDEVHW_DEF
	printk("%s: EP Num= %x:: &usbDevHw_REG_P->eptCfg[num]=%x\n", __func__,num,&usbDevHw_REG_P->eptCfg[num]);
#endif
	REG_WR(usbDevHw_REG_P->eptCfg[num], (num << usbDevHw_REG_ENDPT_CFG_FIFO_NUM_SHIFT) |
						   (type << usbDevHw_REG_ENDPT_CFG_TYPE_SHIFT) |
						   (maxPktSize << usbDevHw_REG_ENDPT_CFG_PKT_MAX_SHIFT) |
						   (dirn == usbDevHw_ENDPT_DIRN_OUT ? usbDevHw_REG_ENDPT_CFG_DIRN_OUT : usbDevHw_REG_ENDPT_CFG_DIRN_IN));
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_EndptAltSet(unsigned num, unsigned alt)
{
	REG_MOD_MASK(usbDevHw_REG_P->eptCfg[num], ~usbDevHw_REG_ENDPT_CFG_ALT_NUM_MASK, (alt << usbDevHw_REG_ENDPT_CFG_ALT_NUM_SHIFT));
}

static inline void usbDevHw_EndptCfgSet(unsigned num, unsigned cfg)
{
	REG_MOD_MASK(usbDevHw_REG_P->eptCfg[num], ~usbDevHw_REG_ENDPT_CFG_CFG_NUM_MASK, (cfg << usbDevHw_REG_ENDPT_CFG_CFG_NUM_SHIFT));
}

static inline void usbDevHw_EndptIntfSet(unsigned num, unsigned intf)
{
	REG_MOD_MASK(usbDevHw_REG_P->eptCfg[num], ~usbDevHw_REG_ENDPT_CFG_INTF_NUM_MASK, (intf << usbDevHw_REG_ENDPT_CFG_INTF_NUM_SHIFT));
}


/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_EndptDmaDisable(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
#if usbDevHw_REG_MULTI_RX_FIFO
		REG_MOD_AND(usbDevHw_REG_P->eptFifoOut[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_DMA_ENABLE);
#else
		/*
		 * With a single RX FIFO, do not want to do anything, as there might be another OUT capable
		 * endpoint still active and wanting DMA enabled. If theory this should be OK, as long as
		 * the DMA descriptor buffer status fields are the last thing updated before being set to
		 * HOST ready, or the first thing updated when being set to HOST busy. Hopefully no
		 * situations arise such that there's contention with the hardware with doing this.
		 */
#endif
	} else {
		REG_MOD_AND(usbDevHw_REG_P->eptFifoIn[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_IN_DMA_ENABLE);
	}
}

static inline void usbDevHw_EndptDmaEnable(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		/*printk(KERN_ERR "%s:: BCM5892:ENABLE OUT DMA\n",__func__); */
#if usbDevHw_REG_MULTI_RX_FIFO
		REG_MOD_OR(usbDevHw_REG_P->eptFifoOut[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_DMA_ENABLE);
#else
		/*printk(KERN_ERR "udc: RX DMA ENABLE (ept)\n"); */
		REG_MOD_OR(usbDevHw_REG_P->devCtrl, usbDevHw_REG_DEV_CTRL_DMA_OUT_ENABLE);
#ifdef DBG_USBDEVHW_DEF
		printk("usbDevHw_EndptDmaEnable: devCtrl: addr: 0x%x, val: 0x%x\n", &usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devCtrl);
#endif
#endif
	} else {
		/* Set the Poll bit in the control register */
		REG_MOD_OR(usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_IN_DMA_ENABLE);
	}
}

static inline void usbDevHw_EndptDmaSetupBufAddrSet(unsigned num, unsigned dirn, void *addr)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
#ifdef DBG_USBDEVHW_DEF
		printk("usbDevHw_EndptDmaSetupBufAddrSet: setupBufAddr: 0x%x, val: 0x%x\n", &usbDevHw_REG_P->eptFifoOut[num].setupBufAddr, addr);
#endif
		REG_WR(usbDevHw_REG_P->eptFifoOut[num].setupBufAddr, (uint32_t)addr);
	}
}

static inline void usbDevHw_EndptDmaDataDescAddrSet(unsigned num, unsigned dirn, void *addr)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		REG_WR(usbDevHw_REG_P->eptFifoOut[num].dataDescAddr, (uint32_t)addr);
	}
	else {
		REG_WR(usbDevHw_REG_P->eptFifoIn[num].dataDescAddr, (uint32_t)addr);
	}
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline bool usbDevHw_EndptFifoEmpty(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
#if usbDevHw_REG_MULTI_RX_FIFO
		return(usbDevHw_REG_P->eptFifoOut[num].status & usbDevHw_REG_ENDPT_FIFO_STATUS_OUT_FIFO_EMPTY ? true : false);
#else
		return(usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_OUT_FIFO_EMPTY ? true : false);
#endif
	}
	return(usbDevHw_REG_P->eptFifoIn[num].status & usbDevHw_REG_ENDPT_FIFO_STATUS_IN_FIFO_EMPTY ? true : false);
}

static inline void usbDevHw_EndptFifoFlushDisable(unsigned num, unsigned dirn)
{
#ifdef DBG_USBDEVHW_DEF
	printk("enter: %s\n", __func__);
#endif
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
#if usbDevHw_REG_MULTI_RX_FIFO
		REG_MOD_AND(usbDevHw_REG_P->eptFifoOut[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_FLUSH_ENABLE);
#else
		REG_MOD_AND(usbDevHw_REG_P->devCtrl, ~usbDevHw_REG_DEV_CTRL_OUT_FIFO_FLUSH_ENABLE);
#endif
	}
	else {
		REG_MOD_AND(usbDevHw_REG_P->eptFifoIn[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_IN_FLUSH_ENABLE);
	}
}

static inline void usbDevHw_EndptFifoFlushEnable(unsigned num, unsigned dirn)
{
#ifdef DBG_USBDEVHW_DEF
	printk("enter: %s\n", __func__);
#endif
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
#if usbDevHw_REG_MULTI_RX_FIFO
		REG_MOD_OR(usbDevHw_REG_P->eptFifoOut[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_FLUSH_ENABLE);
#else
		REG_MOD_OR(usbDevHw_REG_P->devCtrl, usbDevHw_REG_DEV_CTRL_OUT_FIFO_FLUSH_ENABLE);
#endif
	} else {
		REG_MOD_OR(usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_IN_FLUSH_ENABLE);
	}
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline unsigned usbDevHw_EndptFrameNumGet(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		return((usbDevHw_REG_P->eptFifoOut[num].size1 & usbDevHw_REG_ENDPT_FIFO_SIZE1_OUT_FRAME_NUM_MASK) >> usbDevHw_REG_ENDPT_FIFO_SIZE1_OUT_FRAME_NUM_SHIFT);
	}
	return(0);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_EndptIrqClear(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		/*printk(KERN_ERR "%s BCM5892:-  Initial Value of  OUT Ep_Intr_Mask %x\n", __func__, usbDevHw_REG_P->eptIntrStatus); */
		REG_WR(usbDevHw_REG_P->eptIntrStatus, (1 << num) << usbDevHw_REG_ENDPT_INTR_OUT_SHIFT);
		/*printk(KERN_ERR "%s BCM5892:-  Final Value of  OUT Ep_Intr_Mask %x\n", __func__, usbDevHw_REG_P->eptIntrStatus); */
	}
	else {
		/*printk(KERN_ERR "%s BCM5892:-  Initial Value of  IN Ep_Intr_Mask %x\n", __func__, usbDevHw_REG_P->eptIntrStatus); */
		REG_WR(usbDevHw_REG_P->eptIntrStatus, (1 << num) << usbDevHw_REG_ENDPT_INTR_IN_SHIFT);
		/*printk(KERN_ERR "%s BCM5892:-  Final Value of  IN Ep_Intr_Mask %x\n", __func__, usbDevHw_REG_P->eptIntrStatus); */
	}
}

static inline void usbDevHw_EndptIrqDisable(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
#ifdef DBG_USBDEVHW_DEF
		printk("udc: ep%d OUT disable\n", num);
#endif
		REG_MOD_OR(usbDevHw_REG_P->eptIntrMask, ((1 << num) << usbDevHw_REG_ENDPT_INTR_OUT_SHIFT));
	}
	else {
#ifdef DBG_USBDEVHW_DEF
		printk("udc: ep%d IN disable\n", num);
#endif
		REG_MOD_OR(usbDevHw_REG_P->eptIntrMask, ((1 << num) << usbDevHw_REG_ENDPT_INTR_IN_SHIFT));
	}
}

static inline void usbDevHw_EndptIrqEnable(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
#ifdef DBG_USBDEVHW_DEF
		printk("udc: ep%d OUT enable\n", num); 
#endif
		REG_MOD_AND(usbDevHw_REG_P->eptIntrMask, ~((1 << num) << usbDevHw_REG_ENDPT_INTR_OUT_SHIFT));
   	}
	else {
#ifdef DBG_USBDEVHW_DEF
		printk("udc: ep%d IN enable\n", num);
#endif
		REG_MOD_AND(usbDevHw_REG_P->eptIntrMask, ~((1 << num) << usbDevHw_REG_ENDPT_INTR_IN_SHIFT));
	}
}

static inline uint32_t usbDevHw_EndptIrqListActive(unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		return((usbDevHw_REG_P->eptIntrStatus & usbDevHw_REG_ENDPT_INTR_OUT_MASK) >> usbDevHw_REG_ENDPT_INTR_OUT_SHIFT);
	}
	return((usbDevHw_REG_P->eptIntrStatus & usbDevHw_REG_ENDPT_INTR_IN_MASK) >> usbDevHw_REG_ENDPT_INTR_IN_SHIFT);
}

static inline void usbDevHw_EndptIrqListClear(unsigned dirn, uint32_t mask)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		/*printk(KERN_ERR "%s::BCM5892 : Clearing OUT Interrupt Initial value=%x\n",__func__,usbDevHw_REG_P->eptIntrStatus); */
   		REG_WR(usbDevHw_REG_P->eptIntrStatus, (mask << usbDevHw_REG_ENDPT_INTR_OUT_SHIFT)); /*strat from bit 16 */
		/*printk(KERN_ERR "%s::BCM5892 : Cleared OUT Interrupt Final value=%x\n",__func__,usbDevHw_REG_P->eptIntrStatus); */
	}
	else {
		/*printk(KERN_ERR "%s::BCM5892 : Clearing IN Interrupt Initial value=%x\n",__func__,usbDevHw_REG_P->eptIntrStatus); */
		REG_WR(usbDevHw_REG_P->eptIntrStatus, (mask << usbDevHw_REG_ENDPT_INTR_IN_SHIFT)); /* start from bit 0 */
		/*printk(KERN_ERR "%s::BCM5892 : Clearing IN Interrupt Initial value=%x\n",__func__,usbDevHw_REG_P->eptIntrStatus); */
	}
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline uint32_t usbDevHw_EndptStatusActive(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		/*printk(KERN_ERR "%s::BCM5892 : EndPoint OUT Status:- =0x%08x\n",__func__,usbDevHw_REG_P->eptFifoOut[num].status); */
		return(usbDevHw_REG_P->eptFifoOut[num].status);  /* End Point Status register */
	}
	return(usbDevHw_REG_P->eptFifoIn[num].status);
}

static inline void usbDevHw_EndptStatusClear(unsigned num, unsigned dirn, uint32_t mask)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		/*printk(KERN_ERR "%s::BCM5892 : Clearing OUT EndPoint Status:-  Initial value=0x%08x\n",__func__,usbDevHw_REG_P->eptFifoOut[num].status); */
		REG_WR(usbDevHw_REG_P->eptFifoOut[num].status, mask);
		/*printk(KERN_ERR "%s::BCM5892 : Clearing OUT EndPoint Status:-  Final value=0x%08x\n",__func__,usbDevHw_REG_P->eptFifoOut[num].status); */
	}
	else {
		/*printk(KERN_ERR "%s::BCM5892 : Clearing IN EndPoint Status:-  Initial value=0x%08x\n",__func__,usbDevHw_REG_P->eptFifoIn[num].status); */
		REG_WR(usbDevHw_REG_P->eptFifoIn[num].status, mask);
		/*printk(KERN_ERR "%s::BCM5892 : Clearing IN EndPoint Status:-  Final value=0x%08x\n",__func__,usbDevHw_REG_P->eptFifoIn[num].status); */
	}
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_EndptNakClear(unsigned num, unsigned dirn)
{
	/*printk(KERN_ERR "udc: ep%d-%s: NAK CLEAR\n", num, (dirn == usbDevHw_ENDPT_DIRN_OUT) ? "out" : "in"); */
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		REG_MOD_OR(usbDevHw_REG_P->eptFifoOut[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_CLEAR);
	} else {
		REG_MOD_OR(usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_CLEAR);
	}
}

static inline void usbDevHw_EndptNakEnable(unsigned num, unsigned dirn)
{
	/*printk(KERN_ERR "ep%d-%s: NAK SET\n", num, (dirn == usbDevHw_ENDPT_DIRN_OUT) ? "out" : "in"); */
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		REG_MOD_OR(usbDevHw_REG_P->eptFifoOut[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET);
	} else {
		REG_MOD_OR(usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET);
	}
}

static inline void usbDevHw_EndptNakDisable(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		REG_MOD_AND(usbDevHw_REG_P->eptFifoOut[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET);
	} else {
		REG_MOD_AND(usbDevHw_REG_P->eptFifoIn[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET);
	}
}

static inline bool usbDevHw_EndptNakInProgress(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		return (usbDevHw_REG_P->eptFifoOut[num].ctrl & usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_IN_PROGRESS ? true : false);
	}
	return (usbDevHw_REG_P->eptFifoIn[num].ctrl & usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_IN_PROGRESS ? true : false);
}

/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_EndptStallDisable(unsigned num, unsigned dirn)
{
	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		REG_MOD_AND(usbDevHw_REG_P->eptFifoOut[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_STALL_ENABLE);
	}
	else {
		REG_MOD_AND(usbDevHw_REG_P->eptFifoIn[num].ctrl, ~usbDevHw_REG_ENDPT_FIFO_CTRL_STALL_ENABLE);
	}
}

static inline void usbDevHw_EndptStallEnable(unsigned num, unsigned dirn)
{
#if usbDevHw_REG_MULTI_RX_FIFO
	if (!(usbDevHw_REG_P->eptFifoOut[num].status & usbDevHw_REG_ENDPT_FIFO_STATUS_OUT_FIFO_EMPTY))
#else
	if (!(usbDevHw_REG_P->devStatus & usbDevHw_REG_DEV_STATUS_OUT_FIFO_EMPTY))
#endif
		return;

	if (dirn == usbDevHw_ENDPT_DIRN_OUT) {
		REG_MOD_OR(usbDevHw_REG_P->eptFifoOut[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_STALL_ENABLE);
	}
	else {
		REG_MOD_OR(usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_ENDPT_FIFO_CTRL_STALL_ENABLE);
	}
}


/*****************************************************************************
* See usbDevHw.h for API documentation
*****************************************************************************/
static inline void usbDevHw_PrintInfo(int (*printFP) (const char *, ...))
{
	unsigned num;


	printFP("\nDevice	 cfg=0x%08x	ctrl=0x%08x   stat=0x%08x\n",
			  usbDevHw_REG_P->devCfg, usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus);
	printFP("Intrpt devStat=0x%08x devMask=0x%08x epStat=0x%08x epMask=0x%08x\n",
			  usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrStatus, usbDevHw_REG_P->eptIntrMask);

	for (num = 0; num < usbDevHw_ENDPT_CFG_CNT; num++) {
		printFP("\nep%d:  cfg=0x%08x\n", num, usbDevHw_REG_P->eptCfg[num]);
		printFP("  i: ctrl=0x%08x stat=0x%08x size1=0x%08x size2=0x%08x setup=0x%08x data=0x%08x\n",
					   usbDevHw_REG_P->eptFifoIn[num].ctrl, usbDevHw_REG_P->eptFifoIn[num].status, usbDevHw_REG_P->eptFifoIn[num].size1,
					   usbDevHw_REG_P->eptFifoIn[num].size2, usbDevHw_REG_P->eptFifoIn[num].setupBufAddr, usbDevHw_REG_P->eptFifoIn[num].dataDescAddr);
		printFP("  o: ctrl=0x%08x stat=0x%08x size1=0x%08x size2=0x%08x setup=0x%08x data=0x%08x\n",
					   usbDevHw_REG_P->eptFifoOut[num].ctrl, usbDevHw_REG_P->eptFifoOut[num].status, usbDevHw_REG_P->eptFifoOut[num].size1,
					   usbDevHw_REG_P->eptFifoOut[num].size2, usbDevHw_REG_P->eptFifoOut[num].setupBufAddr, usbDevHw_REG_P->eptFifoOut[num].dataDescAddr);
	}
}

#ifdef __cplusplus
extern "C"
}
#endif

#endif /* USBDEVHW_DEF_H */
