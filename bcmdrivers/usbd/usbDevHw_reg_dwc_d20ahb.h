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

#ifndef USBDEVHW_REG_DWC_D20AHB_H
#define USBDEVHW_REG_DWC_D20AHB_H

/* ---- Include Files ---------------------------------------------------- */

#include "reg.h"
#include "stdint.h"
//#include "usbDevHw_reg_dwc_d20ahb.h"
/* ---- Public Constants and Types --------------------------------------- */

#define usbDevHw_REG_MULTI_RX_FIFO                          0

/* ---- Endpoint FIFO register definitions ------------------------ */

/*
 * The endpoint type field in the FIFO control register has the same enumeration
 * as the USB protocol. Not going to define it here.
 */
#define usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_FLUSH_ENABLE       (1 << 12)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_CLOSE_DESC         (1 << 11)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_IN_SEND_NULL           (1 << 10)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_OUT_DMA_ENABLE         (1 << 9)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_CLEAR              (1 << 8)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_SET                (1 << 7)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_NAK_IN_PROGRESS        (1 << 6)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_TYPE_SHIFT             4
#define usbDevHw_REG_ENDPT_FIFO_CTRL_TYPE_MASK              (3 << usbDevHw_REG_ENDPT_FIFO_CTRL_TYPE_SHIFT)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_IN_DMA_ENABLE          (1 << 3)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_SNOOP_ENABLE           (1 << 2)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_IN_FLUSH_ENABLE        (1 << 1)
#define usbDevHw_REG_ENDPT_FIFO_CTRL_STALL_ENABLE           (1 << 0)


#define usbDevHw_REG_ENDPT_FIFO_STATUS_CLOSE_DESC_CLEAR     (1 << 28)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_IN_XFER_DONE         (1 << 27)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_STALL_SET_RX         (1 << 26)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_STALL_CLEAR_RX       (1 << 25)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_IN_FIFO_EMPTY        (1 << 24)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_IN_DMA_DONE          (1 << 10)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_AHB_BUS_ERROR        (1 << 9)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_OUT_FIFO_EMPTY       (1 << 8)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_DMA_BUF_NOT_AVAIL    (1 << 7)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_IN_TOKEN_RX          (1 << 6)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_OUT_DMA_SETUP_DONE   (1 << 5)
#define usbDevHw_REG_ENDPT_FIFO_STATUS_OUT_DMA_DATA_DONE    (1 << 4)


#define usbDevHw_REG_ENDPT_FIFO_SIZE1_OUT_ISOC_PID_SHIFT    16
#define usbDevHw_REG_ENDPT_FIFO_SIZE1_OUT_ISOC_PID_MASK     (3 << usbDevHw_REG_ENDPT_FIFO_SIZE1_OUT_ISOC_PID_SHIFT)
#define usbDevHw_REG_ENDPT_FIFO_SIZE1_IN_DEPTH_SHIFT        0
#define usbDevHw_REG_ENDPT_FIFO_SIZE1_IN_DEPTH_MASK         (0xffff << usbDevHw_REG_ENDPT_FIFO_SIZE1_IN_DEPTH_SHIFT)
#define usbDevHw_REG_ENDPT_FIFO_SIZE1_OUT_FRAME_NUM_SHIFT   usbDevHw_REG_ENDPT_FIFO_SIZE1_IN_DEPTH_SHIFT
#define usbDevHw_REG_ENDPT_FIFO_SIZE1_OUT_FRAME_NUM_MASK    usbDevHw_REG_ENDPT_FIFO_SIZE1_IN_DEPTH_MASK


#define usbDevHw_REG_ENDPT_FIFO_SIZE2_OUT_DEPTH_SHIFT       16
#define usbDevHw_REG_ENDPT_FIFO_SIZE2_OUT_DEPTH_MASK        (0xffff << usbDevHw_REG_ENDPT_FIFO_SIZE2_OUT_DEPTH_SHIFT)
#define usbDevHw_REG_ENDPT_FIFO_SIZE2_PKT_MAX_SHIFT         0
#define usbDevHw_REG_ENDPT_FIFO_SIZE2_PKT_MAX_MASK          (0xffff << usbDevHw_REG_ENDPT_FIFO_SIZE2_PKT_MAX_SHIFT)


/* ---- Endpoint Configuration register definitions --------------- */

/*
 * The endpoint type field in the config register has the same enumeration
 * as the USB protocol. Not going to define it here.
 */
#define usbDevHw_REG_ENDPT_CFG_PKT_MAX_SHIFT                19
#define usbDevHw_REG_ENDPT_CFG_PKT_MAX_MASK                 (0x7ff << usbDevHw_REG_ENDPT_CFG_PKT_MAX_SHIFT)
#define usbDevHw_REG_ENDPT_CFG_ALT_NUM_SHIFT                15
#define usbDevHw_REG_ENDPT_CFG_ALT_NUM_MASK                 (0xf << usbDevHw_REG_ENDPT_CFG_ALT_NUM_SHIFT)
#define usbDevHw_REG_ENDPT_CFG_INTF_NUM_SHIFT               11
#define usbDevHw_REG_ENDPT_CFG_INTF_NUM_MASK                (0xf << usbDevHw_REG_ENDPT_CFG_INTF_NUM_SHIFT)
#define usbDevHw_REG_ENDPT_CFG_CFG_NUM_SHIFT                7
#define usbDevHw_REG_ENDPT_CFG_CFG_NUM_MASK                 (0xf << usbDevHw_REG_ENDPT_CFG_CFG_NUM_SHIFT)
#define usbDevHw_REG_ENDPT_CFG_TYPE_SHIFT                   5
#define usbDevHw_REG_ENDPT_CFG_TYPE_MASK                    (0x3 << usbDevHw_REG_ENDPT_CFG_TYPE_SHIFT)
#define usbDevHw_REG_ENDPT_CFG_DIRN_IN                      (1 << 4)
#define usbDevHw_REG_ENDPT_CFG_DIRN_OUT                     0
#define usbDevHw_REG_ENDPT_CFG_FIFO_NUM_SHIFT               0
#define usbDevHw_REG_ENDPT_CFG_FIFO_NUM_MASK                (0xf << usbDevHw_REG_ENDPT_CFG_FIFO_NUM_SHIFT)


/* ---- Endpoint Interrupt register definitions ------------------ */

#define usbDevHw_REG_ENDPT_INTR_OUT_SHIFT                   16
#define usbDevHw_REG_ENDPT_INTR_OUT_MASK                    (0xffff << usbDevHw_REG_ENDPT_INTR_OUT_SHIFT)
#define usbDevHw_REG_ENDPT_INTR_IN_SHIFT                    0
#define usbDevHw_REG_ENDPT_INTR_IN_MASK                     (0xffff << usbDevHw_REG_ENDPT_INTR_IN_SHIFT)


/* ---- Device Controller register definitions ------------------- */

#define usbDevHw_REG_DEV_CFG_ULPI_DDR_ENABLE                (1 << 19)
#define usbDevHw_REG_DEV_CFG_SET_DESCRIPTOR_ENABLE          (1 << 18)
#define usbDevHw_REG_DEV_CFG_CSR_PROGRAM_ENABLE             (1 << 17)
#define usbDevHw_REG_DEV_CFG_HALT_STALL_ENABLE              (1 << 16)
#define usbDevHw_REG_DEV_CFG_HS_TIMEOUT_CALIB_SHIFT         13
#define usbDevHw_REG_DEV_CFG_HS_TIMEOUT_CALIB_MASK          (7 << usbDevHw_REG_DEV_CFG_HS_TIMEOUT_CALIB_SHIFT)
#define usbDevHw_REG_DEV_CFG_FS_TIMEOUT_CALIB_SHIFT         10
#define usbDevHw_REG_DEV_CFG_FS_TIMEOUT_CALIB_MASK          (7 << usbDevHw_REG_DEV_CFG_FS_TIMEOUT_CALIB_SHIFT)
#define usbDevHw_REG_DEV_CFG_STATUS_1_ENABLE                (1 << 8)
#define usbDevHw_REG_DEV_CFG_STATUS_ENABLE                  (1 << 7)
#define usbDevHw_REG_DEV_CFG_UTMI_BI_DIRN_ENABLE            (1 << 6)
#define usbDevHw_REG_DEV_CFG_UTMI_8BIT_ENABLE               (1 << 5)
#define usbDevHw_REG_DEV_CFG_SYNC_FRAME_ENABLE              (1 << 4)
#define usbDevHw_REG_DEV_CFG_SELF_PWR_ENABLE                (1 << 3)
#define usbDevHw_REG_DEV_CFG_REMOTE_WAKEUP_ENABLE           (1 << 2)
#define usbDevHw_REG_DEV_CFG_SPD_SHIFT                      0
#define usbDevHw_REG_DEV_CFG_SPD_MASK                       (3 << usbDevHw_REG_DEV_CFG_SPD_SHIFT)
#define usbDevHw_REG_DEV_CFG_SPD_HS                         (0 << usbDevHw_REG_DEV_CFG_SPD_SHIFT)
#define usbDevHw_REG_DEV_CFG_SPD_FS                         (1 << usbDevHw_REG_DEV_CFG_SPD_SHIFT)
#define usbDevHw_REG_DEV_CFG_SPD_LS                         (2 << usbDevHw_REG_DEV_CFG_SPD_SHIFT)
#define usbDevHw_REG_DEV_CFG_SPD_FS_48MHZ                   (3 << usbDevHw_REG_DEV_CFG_SPD_SHIFT)


#define usbDevHw_REG_DEV_CTRL_DMA_OUT_THRESHOLD_LEN_SHIFT   24
#define usbDevHw_REG_DEV_CTRL_DMA_OUT_THRESHOLD_LEN_MASK    (0xff << usbDevHw_REG_DEV_CTRL_DMA_OUT_THRESHOLD_LEN_SHIFT)
#define usbDevHw_REG_DEV_CTRL_DMA_BURST_LEN_SHIFT           16
#define usbDevHw_REG_DEV_CTRL_DMA_BURST_LEN_MASK            (0xff << usbDevHw_REG_DEV_CTRL_DMA_BURST_LEN_SHIFT)
#define usbDevHw_REG_DEV_CTRL_OUT_FIFO_FLUSH_ENABLE         (1 << 14)
#define usbDevHw_REG_DEV_CTRL_CSR_DONE                      (1 << 13)
#define usbDevHw_REG_DEV_CTRL_OUT_NAK_ALL_ENABLE            (1 << 12)
#define usbDevHw_REG_DEV_CTRL_DISCONNECT_ENABLE             (1 << 10)
#define usbDevHw_REG_DEV_CTRL_DMA_MODE_ENABLE               (1 << 9)
#define usbDevHw_REG_DEV_CTRL_DMA_BURST_ENABLE              (1 << 8)
#define usbDevHw_REG_DEV_CTRL_DMA_OUT_THRESHOLD_ENABLE      (1 << 7)
#define usbDevHw_REG_DEV_CTRL_DMA_BUFF_FILL_MODE_ENABLE     (1 << 6)
#define usbDevHw_REG_DEV_CTRL_ENDIAN_BIG_ENABLE             (1 << 5)
#define usbDevHw_REG_DEV_CTRL_DMA_DESC_UPDATE_ENABLE        (1 << 4)
#define usbDevHw_REG_DEV_CTRL_DMA_IN_ENABLE                 (1 << 3)  /*TX DMA Enable */
#define usbDevHw_REG_DEV_CTRL_DMA_OUT_ENABLE                (1 << 2)  /*RX DMA Enable */
#define usbDevHw_REG_DEV_CTRL_RESUME_SIGNAL_ENABLE          (1 << 0)
#define usbDevHw_REG_DEV_CTRL_LE_ENABLE                      0           /*^BCM5892 */


#define usbDevHw_REG_DEV_STATUS_SOF_FRAME_NUM_SHIFT         18
#define usbDevHw_REG_DEV_STATUS_SOF_FRAME_NUM_MASK          (0x3ffff << usbDevHw_REG_DEV_STATUS_SOF_FRAME_NUM_SHIFT)
#define usbDevHw_REG_DEV_STATUS_REMOTE_WAKEUP_ALLOWED       (1 << 17)
#define usbDevHw_REG_DEV_STATUS_PHY_ERROR                   (1 << 16)
#define usbDevHw_REG_DEV_STATUS_OUT_FIFO_EMPTY              (1 << 15)
#define usbDevHw_REG_DEV_STATUS_SPD_SHIFT                   13
#define usbDevHw_REG_DEV_STATUS_SPD_MASK                    (3 << usbDevHw_REG_DEV_STATUS_SPD_SHIFT)
#define usbDevHw_REG_DEV_STATUS_SPD_HS                      (0 << usbDevHw_REG_DEV_STATUS_SPD_SHIFT)
#define usbDevHw_REG_DEV_STATUS_SPD_FS                      (1 << usbDevHw_REG_DEV_STATUS_SPD_SHIFT)
#define usbDevHw_REG_DEV_STATUS_SPD_LS                      (2 << usbDevHw_REG_DEV_STATUS_SPD_SHIFT)
#define usbDevHw_REG_DEV_STATUS_SPD_FS_48MHZ                (3 << usbDevHw_REG_DEV_STATUS_SPD_SHIFT)
#define usbDevHw_REG_DEV_STATUS_BUS_SUSPENDED               (1 << 12)
#define usbDevHw_REG_DEV_STATUS_ALT_NUM_SHIFT               8
#define usbDevHw_REG_DEV_STATUS_ALT_NUM_MASK                (0xf << usbDevHw_REG_DEV_STATUS_ALT_NUM_SHIFT)
#define usbDevHw_REG_DEV_STATUS_INTF_NUM_SHIFT              4
#define usbDevHw_REG_DEV_STATUS_INTF_NUM_MASK               (0xf << usbDevHw_REG_DEV_STATUS_INTF_NUM_SHIFT)
#define usbDevHw_REG_DEV_STATUS_CFG_NUM_SHIFT               0
#define usbDevHw_REG_DEV_STATUS_CFG_NUM_MASK                (0xf << usbDevHw_REG_DEV_STATUS_CFG_NUM_SHIFT)


#define usbDevHw_REG_DEV_INTR_REMOTE_WAKEUP_DELTA           (1 << 7) /*Remote Wakeup Delta*/ 
#define usbDevHw_REG_DEV_INTR_SPD_ENUM_DONE                 (1 << 6) /*ENUM Speed Completed*/
#define usbDevHw_REG_DEV_INTR_SOF_RX                        (1 << 5) /*SOF Token Detected */
#define usbDevHw_REG_DEV_INTR_BUS_SUSPEND                   (1 << 4) /*SUSPEND State Detected*/
#define usbDevHw_REG_DEV_INTR_BUS_RESET                     (1 << 3) /*RESET State Detected */
#define usbDevHw_REG_DEV_INTR_BUS_IDLE                      (1 << 2) /*IDLE State Detected*/
#define usbDevHw_REG_DEV_INTR_SET_INTF_RX                   (1 << 1) /*Received SET_INTERFACE CMD*/
#define usbDevHw_REG_DEV_INTR_SET_CFG_RX                    (1 << 0) /*Received SET_CONFIG CMD*/


/* ---- DMA Descriptor definitions ------------------------------- */

#define usbDevHw_REG_DMA_STATUS_BUF_SHIFT                   30
#define usbDevHw_REG_DMA_STATUS_BUF_HOST_READY              (0 << usbDevHw_REG_DMA_STATUS_BUF_SHIFT)
#define usbDevHw_REG_DMA_STATUS_BUF_DMA_BUSY                (1 << usbDevHw_REG_DMA_STATUS_BUF_SHIFT)
#define usbDevHw_REG_DMA_STATUS_BUF_DMA_DONE                (2 << usbDevHw_REG_DMA_STATUS_BUF_SHIFT)
#define usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY               (3 << usbDevHw_REG_DMA_STATUS_BUF_SHIFT)
#define usbDevHw_REG_DMA_STATUS_BUF_MASK                    (3 << usbDevHw_REG_DMA_STATUS_BUF_SHIFT)

#define usbDevHw_REG_DMA_STATUS_RX_SHIFT                    28
#define usbDevHw_REG_DMA_STATUS_RX_SUCCESS                  (0 << usbDevHw_REG_DMA_STATUS_RX_SHIFT)
#define usbDevHw_REG_DMA_STATUS_RX_ERR_DESC                 (1 << usbDevHw_REG_DMA_STATUS_RX_SHIFT)
#define usbDevHw_REG_DMA_STATUS_RX_ERR_BUF                  (3 << usbDevHw_REG_DMA_STATUS_RX_SHIFT)
#define usbDevHw_REG_DMA_STATUS_RX_MASK                     (3 << usbDevHw_REG_DMA_STATUS_RX_SHIFT)

#define usbDevHw_REG_DMA_STATUS_CFG_NUM_SHIFT               24
#define usbDevHw_REG_DMA_STATUS_CFG_NUM_MASK                (0xf << usbDevHw_REG_DMA_STATUS_CFG_NUM_SHIFT)

#define usbDevHw_REG_DMA_STATUS_INTF_NUM_SHIFT              20
#define usbDevHw_REG_DMA_STATUS_INTF_NUM_MASK               (0xf << usbDevHw_REG_DMA_STATUS_INTF_NUM_SHIFT)

#define usbDevHw_REG_DMA_STATUS_ALT_NUM_SHIFT               16
#define usbDevHw_REG_DMA_STATUS_ALT_NUM_MASK                (0xf << usbDevHw_REG_DMA_STATUS_ALT_NUM_SHIFT)

#define usbDevHw_REG_DMA_STATUS_LAST_DESC                   (1 << 27)

#define usbDevHw_REG_DMA_STATUS_FRAME_NUM_SHIFT             16
#define usbDevHw_REG_DMA_STATUS_FRAME_NUM_MASK              (0x7ff << usbDevHw_REG_DMA_STATUS_FRAME_NUM_SHIFT)

#define usbDevHw_REG_DMA_STATUS_BYTE_CNT_SHIFT              0

#define usbDevHw_REG_DMA_STATUS_ISO_PID_SHIFT               14
#define usbDevHw_REG_DMA_STATUS_ISO_PID_MASK                (0x3 << usbDevHw_REG_DMA_STATUS_ISO_PID_SHIFT)

#define usbDevHw_REG_DMA_STATUS_ISO_BYTE_CNT_SHIFT          usbDevHw_REG_DMA_STATUS_BYTE_CNT_SHIFT
#define usbDevHw_REG_DMA_STATUS_ISO_BYTE_CNT_MASK           (0x3fff << usbDevHw_REG_DMA_STATUS_ISO_BYTE_CNT_SHIFT)

#define usbDevHw_REG_DMA_STATUS_NON_ISO_BYTE_CNT_SHIFT      usbDevHw_REG_DMA_STATUS_BYTE_CNT_SHIFT
#define usbDevHw_REG_DMA_STATUS_NON_ISO_BYTE_CNT_MASK       (0xffff << usbDevHw_REG_DMA_STATUS_NON_ISO_BYTE_CNT_SHIFT)


/* ---- Block level strap option register definitions ----------- */

#define usbDevHw_REG_STRAP_SNAK_ENH                         (1 << 0)
#define usbDevHw_REG_STRAP_STALL_SET_ENH                    (1 << 2)
#define usbDevHw_REG_STRAP_CNAK_CLR_ENH                     (1 << 3)

/* ---- Endpoint Configuration register definitions --------------- */

/*
 * usbDevHw_REG_ENDPT_CNT is just for register layout purposes. There may not be this
 * many endpoints actually enabled (activated, configured) in the RTL.
 */
#define usbDevHw_REG_ENDPT_CNT      16

typedef struct
{
    uint32_t ctrl;
    uint32_t status;
    uint32_t size1;
    uint32_t size2; /* Buf Size OUT/Max PKT SIZE */
    uint32_t setupBufAddr;
    uint32_t dataDescAddr;
    REG32_RSVD(0x18, 0x20);
}
usbDevHw_REG_ENDPT_FIFO_t;

/* maps to page # 97 of UDC data sheet */

typedef struct
{
    usbDevHw_REG_ENDPT_FIFO_t eptFifoIn[usbDevHw_REG_ENDPT_CNT];
    usbDevHw_REG_ENDPT_FIFO_t eptFifoOut[usbDevHw_REG_ENDPT_CNT];

    uint32_t devCfg;
    uint32_t devCtrl;
    uint32_t devStatus;
    uint32_t devIntrStatus;
    uint32_t devIntrMask;
    uint32_t eptIntrStatus;
    uint32_t eptIntrMask;
    uint32_t testMode;
    uint32_t releaseNum;
    REG32_RSVD(0x424, 0x500);

    REG32_RSVD(0x500, 0x504);
    uint32_t eptCfg[usbDevHw_REG_ENDPT_CNT];
/*    REG32_RSVD(0x544, 0x908); */
	REG32_RSVD(0x544, 0x800);
	uint32_t RxFIFO[256];
	uint32_t TxFIFO[256];
    uint32_t strap;
}
usbDevHw_REG_t;

/* structure required for initial pull up pull down to generate d+ d- potential difference */


typedef struct 
{
uint32_t GPIO1_enable;
uint32_t GPIO1_data;
}
usbDevPull_REG_t;

/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */


#endif /* USBDEVHW_REG_DWC_D20AHB_H */
