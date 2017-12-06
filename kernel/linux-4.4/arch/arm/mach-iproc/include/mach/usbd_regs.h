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
#ifndef _USBD_REGS_H_
#define _USBD_REGS_H_

#include <linux/delay.h>
#include <mach/iproc_regs.h>

#define IPROC_USBD_MULTI_RX_FIFO                0

#define IPROC_USBD_EP_CFG_CNT                   10
#define IPROC_USBD_REG_EP_CNT                   16

#define IPROC_USBD_SPEED_UNKNOWN                0
#define IPROC_USBD_SPEED_LOW                    1
#define IPROC_USBD_SPEED_FULL                   2
#define IPROC_USBD_SPEED_HIGH                   3

#define IPROC_USBD_EP_DIR_IN                    0x80
#define IPROC_USBD_EP_DIR_OUT                   0x00

#define IPROC_USBD_EP_TYPE_CTRL                 0
#define IPROC_USBD_EP_TYPE_ISOC                 1
#define IPROC_USBD_EP_TYPE_BULK                 2
#define IPROC_USBD_EP_TYPE_INTR                 3

#define IPROC_USBD_IRQ_REMOTEWAKEUP_DELTA       IPROC_USBD_REG_INTR_REMOTE_WAKEUP_DELTA
#define IPROC_USBD_IRQ_SPEED_ENUM_DONE          IPROC_USBD_REG_INTR_SPD_ENUM_DONE
#define IPROC_USBD_IRQ_SOF_DETECTED             IPROC_USBD_REG_INTR_SOF_RX
#define IPROC_USBD_IRQ_BUS_SUSPEND              IPROC_USBD_REG_INTR_BUS_SUSPEND
#define IPROC_USBD_IRQ_BUS_RESET                IPROC_USBD_REG_INTR_BUS_RESET
#define IPROC_USBD_IRQ_BUS_IDLE                 IPROC_USBD_REG_INTR_BUS_IDLE
#define IPROC_USBD_IRQ_SET_INTF                 IPROC_USBD_REG_INTR_SET_INTF_RX
#define IPROC_USBD_IRQ_SET_CFG                  IPROC_USBD_REG_INTR_SET_CFG_RX
#define IPROC_USBD_IRQ_ALL                      (IPROC_USBD_IRQ_REMOTEWAKEUP_DELTA | \
                                                 IPROC_USBD_IRQ_SPEED_ENUM_DONE | \
                                                 IPROC_USBD_IRQ_SOF_DETECTED | \
                                                 IPROC_USBD_IRQ_BUS_SUSPEND | \
                                                 IPROC_USBD_IRQ_BUS_RESET | \
                                                 IPROC_USBD_IRQ_BUS_IDLE | \
                                                 IPROC_USBD_IRQ_SET_INTF | \
                                                 IPROC_USBD_IRQ_SET_CFG)

#define IPROC_USBD_EP_STAT_DMA_ERROR            IPROC_USBD_REG_EP_FIFO_STATUS_AHB_BUS_ERROR
#define IPROC_USBD_EP_STAT_DMA_BUF_UNAVAIL      IPROC_USBD_REG_EP_FIFO_STATUS_DMA_BUF_NOT_AVAIL
#define IPROC_USBD_EP_STAT_IN_TOKEN_RX          IPROC_USBD_REG_EP_FIFO_STATUS_IN_TOKEN_RX
#define IPROC_USBD_EP_STAT_IN_DMA_DONE          IPROC_USBD_REG_EP_FIFO_STATUS_IN_DMA_DONE
#define IPROC_USBD_EP_STAT_IN_FIFO_EMPTY        IPROC_USBD_REG_EP_FIFO_STATUS_IN_FIFO_EMPTY
#define IPROC_USBD_EP_STAT_IN_XFER_DONE         IPROC_USBD_REG_EP_FIFO_STATUS_IN_XFER_DONE
#define IPROC_USBD_EP_STAT_OUT_DMA_DATA_DONE    IPROC_USBD_REG_EP_FIFO_STATUS_OUT_DMA_DATA_DONE
#define IPROC_USBD_EP_STAT_OUT_DMA_SETUP_DONE   IPROC_USBD_REG_EP_FIFO_STATUS_OUT_DMA_SETUP_DONE
#define IPROC_USBD_EP_STAT_ALL                  (IPROC_USBD_EP_STAT_DMA_ERROR | \
                                                 IPROC_USBD_EP_STAT_DMA_BUF_UNAVAIL | \
                                                 IPROC_USBD_EP_STAT_IN_TOKEN_RX | \
                                                 IPROC_USBD_EP_STAT_IN_DMA_DONE | \
                                                 IPROC_USBD_EP_STAT_IN_XFER_DONE | \
                                                 IPROC_USBD_EP_STAT_OUT_DMA_DATA_DONE | \
                                                 IPROC_USBD_EP_STAT_OUT_DMA_SETUP_DONE)


#define REG8_RSVD(start, end)   uint8_t rsvd_##start[(end - start) / sizeof(uint8_t)]
#define REG16_RSVD(start, end)  uint16_t rsvd_##start[(end - start) / sizeof(uint16_t)]
#define REG32_RSVD(start, end)  uint rsvd_##start[(end - start) / sizeof(uint)]

struct iproc_usbd_ep_fifo_regs {
    uint ctrl;
    uint status;
    uint size1;
    uint size2; /* Buf Size OUT/Max PKT SIZE */
    uint buf_addr;
    uint desc_addr;
    REG32_RSVD(0x18, 0x20);
};

struct iproc_usbd_regs {
    struct iproc_usbd_ep_fifo_regs ep_fifo_in[IPROC_USBD_REG_EP_CNT];
    struct iproc_usbd_ep_fifo_regs ep_fifo_out[IPROC_USBD_REG_EP_CNT];
    uint dev_cfg;
    uint dev_ctrl;
    uint dev_status;
    uint dev_irq_status;
    uint dev_irq_mask;
    uint ep_irq_status;
    uint ep_irq_mask;
    uint test_mode;
    uint rel_num;
    REG32_RSVD(0x424, 0x500);
    REG32_RSVD(0x500, 0x504);
    uint ep_cfg[IPROC_USBD_REG_EP_CNT];
    REG32_RSVD(0x544, 0x800);
    uint rx_fifo[256];
    uint tx_fifo[256];
    uint strap;
};


struct iproc_usbd_idm_regs {
    REG32_RSVD(0x000, 0x408);
    uint io_ctrl;
    REG32_RSVD(0x40C, 0x500);
    uint io_status;
    REG32_RSVD(0x504, 0x800);
    uint reset_ctrl;
    uint reset_status;
    REG32_RSVD(0x808, 0xA00);
    uint irq_status;
};

/*
 * The endpoint type field in the FIFO control register has the same enumeration
 * as the USB protocol. Not going to define it here.
 */
#define IPROC_USBD_REG_EP_FIFO_CTRL_OUT_FLUSH_ENABLE        (1 << 12)
#define IPROC_USBD_REG_EP_FIFO_CTRL_OUT_CLOSE_DESC          (1 << 11)
#define IPROC_USBD_REG_EP_FIFO_CTRL_IN_SEND_NULL            (1 << 10)
#define IPROC_USBD_REG_EP_FIFO_CTRL_OUT_DMA_ENABLE          (1 << 9)
#define IPROC_USBD_REG_EP_FIFO_CTRL_NAK_CLEAR               (1 << 8)
#define IPROC_USBD_REG_EP_FIFO_CTRL_NAK_SET                 (1 << 7)
#define IPROC_USBD_REG_EP_FIFO_CTRL_NAK_IN_PROGRESS         (1 << 6)
#define IPROC_USBD_REG_EP_FIFO_CTRL_TYPE_SHIFT              4
#define IPROC_USBD_REG_EP_FIFO_CTRL_TYPE_MASK               (3 << IPROC_USBD_REG_EP_FIFO_CTRL_TYPE_SHIFT)
#define IPROC_USBD_REG_EP_FIFO_CTRL_IN_DMA_ENABLE           (1 << 3)
#define IPROC_USBD_REG_EP_FIFO_CTRL_SNOOP_ENABLE            (1 << 2)
#define IPROC_USBD_REG_EP_FIFO_CTRL_IN_FLUSH_ENABLE         (1 << 1)
#define IPROC_USBD_REG_EP_FIFO_CTRL_STALL_ENABLE            (1 << 0)

#define IPROC_USBD_REG_EP_FIFO_STATUS_CLOSE_DESC_CLEAR      (1 << 28)
#define IPROC_USBD_REG_EP_FIFO_STATUS_IN_XFER_DONE          (1 << 27)
#define IPROC_USBD_REG_EP_FIFO_STATUS_STALL_SET_RX          (1 << 26)
#define IPROC_USBD_REG_EP_FIFO_STATUS_STALL_CLEAR_RX        (1 << 25)
#define IPROC_USBD_REG_EP_FIFO_STATUS_IN_FIFO_EMPTY         (1 << 24)
#define IPROC_USBD_REG_EP_FIFO_STATUS_IN_DMA_DONE           (1 << 10)
#define IPROC_USBD_REG_EP_FIFO_STATUS_AHB_BUS_ERROR         (1 << 9)
#define IPROC_USBD_REG_EP_FIFO_STATUS_OUT_FIFO_EMPTY        (1 << 8)
#define IPROC_USBD_REG_EP_FIFO_STATUS_DMA_BUF_NOT_AVAIL     (1 << 7)
#define IPROC_USBD_REG_EP_FIFO_STATUS_IN_TOKEN_RX           (1 << 6)
#define IPROC_USBD_REG_EP_FIFO_STATUS_OUT_DMA_SETUP_DONE    (1 << 5)
#define IPROC_USBD_REG_EP_FIFO_STATUS_OUT_DMA_DATA_DONE     (1 << 4)

#define IPROC_USBD_REG_EP_FIFO_SIZE1_OUT_ISOC_PID_SHIFT     16
#define IPROC_USBD_REG_EP_FIFO_SIZE1_OUT_ISOC_PID_MASK      (3 << IPROC_USBD_REG_EP_FIFO_SIZE1_OUT_ISOC_PID_SHIFT)
#define IPROC_USBD_REG_EP_FIFO_SIZE1_IN_DEPTH_SHIFT         0
#define IPROC_USBD_REG_EP_FIFO_SIZE1_IN_DEPTH_MASK          (0xffff << IPROC_USBD_REG_EP_FIFO_SIZE1_IN_DEPTH_SHIFT)
#define IPROC_USBD_REG_EP_FIFO_SIZE1_OUT_FRAME_NUM_SHIFT    IPROC_USBD_REG_EP_FIFO_SIZE1_IN_DEPTH_SHIFT
#define IPROC_USBD_REG_EP_FIFO_SIZE1_OUT_FRAME_NUM_MASK     IPROC_USBD_REG_EP_FIFO_SIZE1_IN_DEPTH_MASK

#define IPROC_USBD_REG_EP_FIFO_SIZE2_OUT_DEPTH_SHIFT        16
#define IPROC_USBD_REG_EP_FIFO_SIZE2_OUT_DEPTH_MASK         (0xffff << IPROC_USBD_REG_EP_FIFO_SIZE2_OUT_DEPTH_SHIFT)
#define IPROC_USBD_REG_EP_FIFO_SIZE2_PKT_MAX_SHIFT          0
#define IPROC_USBD_REG_EP_FIFO_SIZE2_PKT_MAX_MASK           (0xffff << IPROC_USBD_REG_EP_FIFO_SIZE2_PKT_MAX_SHIFT)

/*
 * The endpoint type field in the config register has the same enumeration
 * as the USB protocol. Not going to define it here.
 */
#define IPROC_USBD_REG_EP_CFG_PKT_MAX_SHIFT                 19
#define IPROC_USBD_REG_EP_CFG_PKT_MAX_MASK                  (0x7ff << IPROC_USBD_REG_EP_CFG_PKT_MAX_SHIFT)
#define IPROC_USBD_REG_EP_CFG_ALT_NUM_SHIFT                 15
#define IPROC_USBD_REG_EP_CFG_ALT_NUM_MASK                  (0xf << IPROC_USBD_REG_EP_CFG_ALT_NUM_SHIFT)
#define IPROC_USBD_REG_EP_CFG_INTF_NUM_SHIFT                11
#define IPROC_USBD_REG_EP_CFG_INTF_NUM_MASK                 (0xf << IPROC_USBD_REG_EP_CFG_INTF_NUM_SHIFT)
#define IPROC_USBD_REG_EP_CFG_CFG_NUM_SHIFT                 7
#define IPROC_USBD_REG_EP_CFG_CFG_NUM_MASK                  (0xf << IPROC_USBD_REG_EP_CFG_CFG_NUM_SHIFT)
#define IPROC_USBD_REG_EP_CFG_TYPE_SHIFT                    5
#define IPROC_USBD_REG_EP_CFG_TYPE_MASK                     (0x3 << IPROC_USBD_REG_EP_CFG_TYPE_SHIFT)
#define IPROC_USBD_REG_EP_CFG_DIRN_IN                       (1 << 4)
#define IPROC_USBD_REG_EP_CFG_DIRN_OUT                      0
#define IPROC_USBD_REG_EP_CFG_FIFO_NUM_SHIFT                0
#define IPROC_USBD_REG_EP_CFG_FIFO_NUM_MASK                 (0xf << IPROC_USBD_REG_EP_CFG_FIFO_NUM_SHIFT)

/* Endpoint Interrupt register definitions */
#define IPROC_USBD_REG_EP_INTR_OUT_SHIFT                    16
#define IPROC_USBD_REG_EP_INTR_OUT_MASK                     (0xffff << IPROC_USBD_REG_EP_INTR_OUT_SHIFT)
#define IPROC_USBD_REG_EP_INTR_IN_SHIFT                     0
#define IPROC_USBD_REG_EP_INTR_IN_MASK                      (0xffff << IPROC_USBD_REG_EP_INTR_IN_SHIFT)

/* Device Controller register definitions */
#define IPROC_USBD_REG_CFG_ULPI_DDR_ENABLE                  (1 << 19)
#define IPROC_USBD_REG_CFG_SET_DESCRIPTOR_ENABLE            (1 << 18)
#define IPROC_USBD_REG_CFG_CSR_PROGRAM_ENABLE               (1 << 17)
#define IPROC_USBD_REG_CFG_HALT_STALL_ENABLE                (1 << 16)
#define IPROC_USBD_REG_CFG_HS_TIMEOUT_CALIB_SHIFT           13
#define IPROC_USBD_REG_CFG_HS_TIMEOUT_CALIB_MASK            (7 << IPROC_USBD_REG_CFG_HS_TIMEOUT_CALIB_SHIFT)
#define IPROC_USBD_REG_CFG_FS_TIMEOUT_CALIB_SHIFT           10
#define IPROC_USBD_REG_CFG_FS_TIMEOUT_CALIB_MASK            (7 << IPROC_USBD_REG_CFG_FS_TIMEOUT_CALIB_SHIFT)
#define IPROC_USBD_REG_CFG_STATUS_1_ENABLE                  (1 << 8)
#define IPROC_USBD_REG_CFG_STATUS_ENABLE                    (1 << 7)
#define IPROC_USBD_REG_CFG_UTMI_BI_DIRN_ENABLE              (1 << 6)
#define IPROC_USBD_REG_CFG_UTMI_8BIT_ENABLE                 (1 << 5)
#define IPROC_USBD_REG_CFG_SYNC_FRAME_ENABLE                (1 << 4)
#define IPROC_USBD_REG_CFG_SELF_PWR_ENABLE                  (1 << 3)
#define IPROC_USBD_REG_CFG_REMOTE_WAKEUP_ENABLE             (1 << 2)
#define IPROC_USBD_REG_CFG_SPD_SHIFT                        0
#define IPROC_USBD_REG_CFG_SPD_MASK                         (3 << IPROC_USBD_REG_CFG_SPD_SHIFT)
#define IPROC_USBD_REG_CFG_SPD_HS                           (0 << IPROC_USBD_REG_CFG_SPD_SHIFT)
#define IPROC_USBD_REG_CFG_SPD_FS                           (1 << IPROC_USBD_REG_CFG_SPD_SHIFT)
#define IPROC_USBD_REG_CFG_SPD_LS                           (2 << IPROC_USBD_REG_CFG_SPD_SHIFT)
#define IPROC_USBD_REG_CFG_SPD_FS_48MHZ                     (3 << IPROC_USBD_REG_CFG_SPD_SHIFT)

#define IPROC_USBD_REG_CTRL_DMA_OUT_THRESHOLD_LEN_SHIFT     24
#define IPROC_USBD_REG_CTRL_DMA_OUT_THRESHOLD_LEN_MASK      (0xff << IPROC_USBD_REG_CTRL_DMA_OUT_THRESHOLD_LEN_SHIFT)
#define IPROC_USBD_REG_CTRL_DMA_BURST_LEN_SHIFT             16
#define IPROC_USBD_REG_CTRL_DMA_BURST_LEN_MASK              (0xff << IPROC_USBD_REG_CTRL_DMA_BURST_LEN_SHIFT)
#define IPROC_USBD_REG_CTRL_OUT_FIFO_FLUSH_ENABLE           (1 << 14)
#define IPROC_USBD_REG_CTRL_CSR_DONE                        (1 << 13)
#define IPROC_USBD_REG_CTRL_OUT_NAK_ALL_ENABLE              (1 << 12)
#define IPROC_USBD_REG_CTRL_DISCONNECT_ENABLE               (1 << 10)
#define IPROC_USBD_REG_CTRL_DMA_MODE_ENABLE                 (1 << 9)
#define IPROC_USBD_REG_CTRL_DMA_BURST_ENABLE                (1 << 8)
#define IPROC_USBD_REG_CTRL_DMA_OUT_THRESHOLD_ENABLE        (1 << 7)
#define IPROC_USBD_REG_CTRL_DMA_BUFF_FILL_MODE_ENABLE       (1 << 6)
#define IPROC_USBD_REG_CTRL_ENDIAN_BIG_ENABLE               (1 << 5)
#define IPROC_USBD_REG_CTRL_DMA_DESC_UPDATE_ENABLE          (1 << 4)
#define IPROC_USBD_REG_CTRL_DMA_IN_ENABLE                   (1 << 3)  /*TX DMA Enable */
#define IPROC_USBD_REG_CTRL_DMA_OUT_ENABLE                  (1 << 2)  /*RX DMA Enable */
#define IPROC_USBD_REG_CTRL_RESUME_SIGNAL_ENABLE            (1 << 0)
#define IPROC_USBD_REG_CTRL_LE_ENABLE                        0           /*^BCM5892 */

#define IPROC_USBD_REG_STAT_SOF_FRAME_NUM_SHIFT             18
#define IPROC_USBD_REG_STAT_SOF_FRAME_NUM_MASK              (0x3ffff << IPROC_USBD_REG_STAT_SOF_FRAME_NUM_SHIFT)
#define IPROC_USBD_REG_STAT_REMOTE_WAKEUP_ALLOWED           (1 << 17)
#define IPROC_USBD_REG_STAT_PHY_ERROR                       (1 << 16)
#define IPROC_USBD_REG_STAT_OUT_FIFO_EMPTY                  (1 << 15)
#define IPROC_USBD_REG_STAT_SPD_SHIFT                       13
#define IPROC_USBD_REG_STAT_SPD_MASK                        (3 << IPROC_USBD_REG_STAT_SPD_SHIFT)
#define IPROC_USBD_REG_STAT_SPD_HS                          (0 << IPROC_USBD_REG_STAT_SPD_SHIFT)
#define IPROC_USBD_REG_STAT_SPD_FS                          (1 << IPROC_USBD_REG_STAT_SPD_SHIFT)
#define IPROC_USBD_REG_STAT_SPD_LS                          (2 << IPROC_USBD_REG_STAT_SPD_SHIFT)
#define IPROC_USBD_REG_STAT_SPD_FS_48MHZ                    (3 << IPROC_USBD_REG_STAT_SPD_SHIFT)
#define IPROC_USBD_REG_STAT_BUS_SUSPENDED                   (1 << 12)
#define IPROC_USBD_REG_STAT_ALT_NUM_SHIFT                   8
#define IPROC_USBD_REG_STAT_ALT_NUM_MASK                    (0xf << IPROC_USBD_REG_STAT_ALT_NUM_SHIFT)
#define IPROC_USBD_REG_STAT_INTF_NUM_SHIFT                  4
#define IPROC_USBD_REG_STAT_INTF_NUM_MASK                   (0xf << IPROC_USBD_REG_STAT_INTF_NUM_SHIFT)
#define IPROC_USBD_REG_STAT_CFG_NUM_SHIFT                   0
#define IPROC_USBD_REG_STAT_CFG_NUM_MASK                    (0xf << IPROC_USBD_REG_STAT_CFG_NUM_SHIFT)

#define IPROC_USBD_REG_INTR_REMOTE_WAKEUP_DELTA             (1 << 7) /*Remote Wakeup Delta*/
#define IPROC_USBD_REG_INTR_SPD_ENUM_DONE                   (1 << 6) /*ENUM Speed Completed*/
#define IPROC_USBD_REG_INTR_SOF_RX                          (1 << 5) /*SOF Token Detected */
#define IPROC_USBD_REG_INTR_BUS_SUSPEND                     (1 << 4) /*SUSPEND State Detected*/
#define IPROC_USBD_REG_INTR_BUS_RESET                       (1 << 3) /*RESET State Detected */
#define IPROC_USBD_REG_INTR_BUS_IDLE                        (1 << 2) /*IDLE State Detected*/
#define IPROC_USBD_REG_INTR_SET_INTF_RX                     (1 << 1) /*Received SET_INTERFACE CMD*/
#define IPROC_USBD_REG_INTR_SET_CFG_RX                      (1 << 0) /*Received SET_CONFIG CMD*/

/* DMA Descriptor definitions */
#define IPROC_USBD_REG_DMA_STAT_BUF_SHIFT                   30
#define IPROC_USBD_REG_DMA_STAT_BUF_HOST_READY              (0 << IPROC_USBD_REG_DMA_STAT_BUF_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_BUF_DMA_BUSY                (1 << IPROC_USBD_REG_DMA_STAT_BUF_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_BUF_DMA_DONE                (2 << IPROC_USBD_REG_DMA_STAT_BUF_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_BUF_HOST_BUSY               (3 << IPROC_USBD_REG_DMA_STAT_BUF_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_BUF_MASK                    (3 << IPROC_USBD_REG_DMA_STAT_BUF_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_RX_SHIFT                    28
#define IPROC_USBD_REG_DMA_STAT_RX_SUCCESS                  (0 << IPROC_USBD_REG_DMA_STAT_RX_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_RX_ERR_DESC                 (1 << IPROC_USBD_REG_DMA_STAT_RX_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_RX_ERR_BUF                  (3 << IPROC_USBD_REG_DMA_STAT_RX_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_RX_MASK                     (3 << IPROC_USBD_REG_DMA_STAT_RX_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_CFG_NUM_SHIFT               24
#define IPROC_USBD_REG_DMA_STAT_CFG_NUM_MASK                (0xf << IPROC_USBD_REG_DMA_STAT_CFG_NUM_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_INTF_NUM_SHIFT              20
#define IPROC_USBD_REG_DMA_STAT_INTF_NUM_MASK               (0xf << IPROC_USBD_REG_DMA_STAT_INTF_NUM_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_ALT_NUM_SHIFT               16
#define IPROC_USBD_REG_DMA_STAT_ALT_NUM_MASK                (0xf << IPROC_USBD_REG_DMA_STAT_ALT_NUM_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_LAST_DESC                   (1 << 27)
#define IPROC_USBD_REG_DMA_STAT_FRAME_NUM_SHIFT             16
#define IPROC_USBD_REG_DMA_STAT_FRAME_NUM_MASK              (0x7ff << IPROC_USBD_REG_DMA_STAT_FRAME_NUM_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_BYTE_CNT_SHIFT              0
#define IPROC_USBD_REG_DMA_STAT_ISO_PID_SHIFT               14
#define IPROC_USBD_REG_DMA_STAT_ISO_PID_MASK                (0x3 << IPROC_USBD_REG_DMA_STAT_ISO_PID_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_ISO_BYTE_CNT_SHIFT          IPROC_USBD_REG_DMA_STAT_BYTE_CNT_SHIFT
#define IPROC_USBD_REG_DMA_STAT_ISO_BYTE_CNT_MASK           (0x3fff << IPROC_USBD_REG_DMA_STAT_ISO_BYTE_CNT_SHIFT)
#define IPROC_USBD_REG_DMA_STAT_NON_ISO_BYTE_CNT_SHIFT      IPROC_USBD_REG_DMA_STAT_BYTE_CNT_SHIFT
#define IPROC_USBD_REG_DMA_STAT_NON_ISO_BYTE_CNT_MASK       (0xffff << IPROC_USBD_REG_DMA_STAT_NON_ISO_BYTE_CNT_SHIFT)

/* USB2D IDM definitions */
#define IPROC_USB2D_IDM_REG_IO_CTRL_DIRECT_CLK_ENABLE       (1 << 0)
#define IPROC_USB2D_IDM_REG_RESET_CTRL_RESET                (1 << 0)

/* Inline Function Definitions */
static inline uint
usbd_reg32_read(volatile uint *reg)
{
    return (le32_to_cpu(*reg));
}

static inline void
usbd_reg32_write(volatile uint *reg, uint value)
{
    *reg = cpu_to_le32(value);
}

static inline void
usbd_reg32_bits_set(volatile uint *reg, uint bits)
{
    uint tmp;
    tmp = usbd_reg32_read(reg);
    tmp |= bits;
    usbd_reg32_write(reg, tmp);
}

static inline void
usbd_reg32_bits_clear(volatile uint *reg, uint bits)
{
    uint tmp;
    tmp = usbd_reg32_read(reg);
    tmp &= ~bits;
    usbd_reg32_write(reg, tmp);
}

static inline void
usbd_reg32_bits_modify(volatile uint *reg, uint mask, uint value)
{
    uint tmp;
    tmp = usbd_reg32_read(reg);
    tmp &= ~mask;
    tmp |= value;
    usbd_reg32_write(reg, tmp);
}

#define IPROC_USBD_READ(_r)                       usbd_reg32_read(&_r)
#define IPROC_USBD_WRITE(_r, _v)                  usbd_reg32_write(&_r, _v)
#define IPROC_USBD_BITS_SET(_r, _b)               usbd_reg32_bits_set(&_r, _b)
#define IPROC_USBD_BITS_CLEAR(_r, _b)             usbd_reg32_bits_clear(&_r, _b)
#define IPROC_USBD_BITS_MODIFY(_r, _m, _v)        usbd_reg32_bits_modify(&_r, _m, _v)

/*****************************************************************************
*  @brief   Connect / Disconnect to USB BUS
*****************************************************************************/
static inline void iproc_usbd_bus_conn(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_CLEAR(base->dev_ctrl, IPROC_USBD_REG_CTRL_DISCONNECT_ENABLE);
}

static inline void iproc_usbd_bus_disconn(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_ctrl, IPROC_USBD_REG_CTRL_DISCONNECT_ENABLE);
}

/*****************************************************************************
*  @brief   USB BUS suspend status
*  @return
*   true  : BUS is in suspend state
*   false : BUS is not in suspend state
*****************************************************************************/
static inline bool iproc_usbd_bus_suspend(struct iproc_usbd_regs *base)
{
    return (IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_BUS_SUSPENDED) ? true : false;
}

/*****************************************************************************
*  @brief   Retrieve setting numbers from last Rx'd SET_CONFIGURATION or
*           SET_INTERFACE request
*  @return
*   Setting Number
*****************************************************************************/
static inline uint iproc_usbd_alt_num(struct iproc_usbd_regs *base)
{
    return ((IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_ALT_NUM_MASK) >> IPROC_USBD_REG_STAT_ALT_NUM_SHIFT);
}

static inline uint iproc_usbd_cfg_num(struct iproc_usbd_regs *base)
{
    return ((IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_CFG_NUM_MASK) >> IPROC_USBD_REG_STAT_CFG_NUM_SHIFT);
}

static inline uint iproc_usbd_intf_num(struct iproc_usbd_regs *base)
{
    return ((IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_INTF_NUM_MASK) >> IPROC_USBD_REG_STAT_INTF_NUM_SHIFT);
}


/*****************************************************************************
*  @brief   Disable / Enable DMA operations at the device level (all endpoints)
*****************************************************************************/
static inline void iproc_usbd_dma_dis(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_CLEAR(base->dev_ctrl, (IPROC_USBD_REG_CTRL_DMA_IN_ENABLE | IPROC_USBD_REG_CTRL_DMA_OUT_ENABLE));
}

static inline void iproc_usbd_dma_en(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_ctrl, (IPROC_USBD_REG_CTRL_DMA_IN_ENABLE | IPROC_USBD_REG_CTRL_DMA_OUT_ENABLE));
}

static inline bool iproc_usbd_dma_status(struct iproc_usbd_regs *base)
{
    return (IPROC_USBD_READ(base->dev_ctrl) & IPROC_USBD_REG_CTRL_DMA_OUT_ENABLE ? true : false);
}

/*****************************************************************************
*  @brief   Retrieve Frame number contained in last Rx'd SOF packet
*  @return
*   Frame Number in the following format.
*       bits[13:3] milli-second frame number
*       bits[2:0] micro-frame number
*  @note
*   For full and low speed connections, the microframe number will be zero.
*****************************************************************************/
static inline uint iproc_usbd_last_rx_frame_num(struct iproc_usbd_regs *base)
{
    return((IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_SOF_FRAME_NUM_MASK) >> IPROC_USBD_REG_STAT_SOF_FRAME_NUM_SHIFT);
}

/*****************************************************************************
*  @brief   Device level interrupt operations
*  @note
*       Use the IPROC_USBD_IRQ_xxx definitions with these routines. These
*       definitions are bit-wise, and allow operations on multiple interrupts
*       by OR'ing the definitions together.
*       DeviceIrqClear(), DeviceIrqDisable(), DeviceIrqEnable() use their mask
*       parameter to operate only on the interrupts set in the mask. E.g.
*           DeviceIrqEnable( DEVICE_IRQ_SET_INTF );
*           DeviceIrqEnable( DEVICE_IRQ_SET_CFG );
*       and
*           DeviceIrqEnable( DEVICE_IRQ_SET_INTF | DEVICE_IRQ_SET_CFG );
*       are equivalent.
*       DeviceIrqMask() returns a mask of all the interrupts that are enabled.
*       DeviceIrqStatus() returns a mask of all the interrupts that have an active status.
*****************************************************************************/
static inline uint iproc_usbd_irq_active(struct iproc_usbd_regs *base)
{
    return(IPROC_USBD_READ(base->dev_irq_status));
}

static inline void iproc_usbd_irq_clear(struct iproc_usbd_regs *base, uint mask)
{
    IPROC_USBD_WRITE(base->dev_irq_status, mask);
}

static inline void iproc_usbd_irq_dis(struct iproc_usbd_regs *base, uint mask)
{
    IPROC_USBD_BITS_SET(base->dev_irq_mask, mask);
}

static inline void iproc_usbd_irq_en(struct iproc_usbd_regs *base, uint mask)
{
    IPROC_USBD_BITS_CLEAR(base->dev_irq_mask, mask);
}
static inline uint iproc_usbd_irq_mask(struct iproc_usbd_regs *base)
{
    return((~IPROC_USBD_READ(base->dev_irq_mask)) & IPROC_USBD_IRQ_ALL);
}

/*****************************************************************************
*  @brief   Disable / Enable NAK responses for all OUT endpoints.
*****************************************************************************/
static inline void iproc_usbd_nak_response_dis(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_CLEAR(base->dev_ctrl, IPROC_USBD_REG_CTRL_OUT_NAK_ALL_ENABLE);
}

static inline void iproc_usbd_nak_response_en(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_ctrl, IPROC_USBD_REG_CTRL_OUT_NAK_ALL_ENABLE);
}

/*****************************************************************************
*  @brief   PHY error detected
*****************************************************************************/
static inline bool iproc_usbd_phy_err_detect(struct iproc_usbd_regs *base)
{
    return(IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_PHY_ERROR ? true : false);
}

/*****************************************************************************
*  @brief   Remote Wakeup operations.
*       DeviceRemoteWakeupEnable() and DeviceRemoteWakeupDisable() are used to
*       specify device if is going to attempt this.
*       DeviceRemoteWakeupAllowed() indicates if host has enabled this feature.
*       The associated DEVICE_IRQ_REMOTEWAKEUP_DELTA can be used to determine
*       changes to the status of this feature.
*       DeviceRemoteWakeupStart(); delayMsec(1); DeviceRemoteWakeupStop(); is
*       used for controlling the wakeup signalling.
*****************************************************************************/
static inline bool iproc_usbd_wakeup_allow(struct iproc_usbd_regs *base)
{
    return(IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_REMOTE_WAKEUP_ALLOWED ? true : false);
}

static inline void iproc_usbd_wakeup_dis(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_CLEAR(base->dev_cfg, IPROC_USBD_REG_CFG_REMOTE_WAKEUP_ENABLE);
}

static inline void iproc_usbd_wakeup_en(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_cfg, IPROC_USBD_REG_CFG_REMOTE_WAKEUP_ENABLE);
}

static inline void iproc_usbd_wakeup_start(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_ctrl, IPROC_USBD_REG_CTRL_RESUME_SIGNAL_ENABLE);
}

static inline void iproc_usbd_wakeup_stop(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_CLEAR(base->dev_ctrl, IPROC_USBD_REG_CTRL_RESUME_SIGNAL_ENABLE);
}

/*****************************************************************************
*  @brief   Control whether or not device advertises itself as self-powered.
*****************************************************************************/
static inline void iproc_usbd_self_pwr_dis(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_CLEAR(base->dev_cfg, IPROC_USBD_REG_CFG_SELF_PWR_ENABLE);
}

static inline void iproc_usbd_self_pwr_en(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_cfg, IPROC_USBD_REG_CFG_SELF_PWR_ENABLE);
}

/*****************************************************************************
*  @brief   Control whether or not device SET DESCRIPTOR support is enabled.
*       If disabled, STALL will be issued upon receipt of a SET DESCRIPTOR request.
*****************************************************************************/
static inline void iproc_usbd_set_desc_dis(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_CLEAR(base->dev_cfg, IPROC_USBD_REG_CFG_SET_DESCRIPTOR_ENABLE);
}

static inline void iproc_usbd_set_desc_en(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_cfg, IPROC_USBD_REG_CFG_SET_DESCRIPTOR_ENABLE);
}

/*****************************************************************************
*  @brief   Device SET configuration or SET interface has completed.
*       If disabled, STALL will be issued upon receipt of a SET DESCRIPTOR request.
*****************************************************************************/
static inline void iproc_usbd_setup_done(struct iproc_usbd_regs *base)
{
    IPROC_USBD_BITS_SET(base->dev_ctrl, IPROC_USBD_REG_CTRL_CSR_DONE);
}

/*****************************************************************************
*  @brief   Link speed routines.
*       Use the usbDevHw_DEVICE_SPEED_xxx definitions with these routines. These
*       DeviceSpeedRequested() indicates the desired link speed.
*       DeviceSpeedEnumerated() returns the speed negotiated with the host.
*       The associated DEVICE_IRQ_SPEED_ENUM_DONE can be used to determine
*       when speed negotiation has completed.
*****************************************************************************/
static inline uint iproc_usbd_speed_get(struct iproc_usbd_regs *base)
{
    switch(IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_SPD_MASK) {
        case IPROC_USBD_REG_STAT_SPD_LS:
            return(IPROC_USBD_SPEED_LOW);

        case IPROC_USBD_REG_STAT_SPD_HS:
            return(IPROC_USBD_SPEED_HIGH);

        case IPROC_USBD_REG_STAT_SPD_FS:
        case IPROC_USBD_REG_STAT_SPD_FS_48MHZ:
            return(IPROC_USBD_SPEED_FULL);
    }

    return IPROC_USBD_SPEED_FULL;
}

static inline void iproc_usbd_speed_req(struct iproc_usbd_regs *base, uint speed)
{
    IPROC_USBD_BITS_CLEAR(base->dev_cfg, IPROC_USBD_REG_CFG_SPD_MASK);

    switch(speed) {
        case IPROC_USBD_SPEED_LOW:
            IPROC_USBD_BITS_SET(base->dev_cfg, IPROC_USBD_REG_CFG_SPD_LS);
            break;

        case IPROC_USBD_SPEED_HIGH:
            IPROC_USBD_BITS_SET(base->dev_cfg, IPROC_USBD_REG_CFG_SPD_HS);
            break;

        case IPROC_USBD_SPEED_FULL:
        default:
            IPROC_USBD_BITS_SET(base->dev_cfg, IPROC_USBD_REG_CFG_SPD_FS);
            break;
    }
}

/*****************************************************************************
*  @brief   Finalize (terminate) / Initialize Endpoint operations
*  @param   num - Endpoint number
*  @param   dirn - Endpoint direction. See ENDPT_DIRN_xxx definitions
*  @param   dirn - Endpoint type. See ENDPT_TYPE_xxx definitions
*  @param   dirn - Endpoint max packet size.
*****************************************************************************/
static inline void iproc_usbd_ep_ops_finish(struct iproc_usbd_regs *base, uint num)
{
}

static inline void iproc_usbd_ep_ops_init(struct iproc_usbd_regs *base, uint num, uint type, uint dirn, uint maxPktSize)
{
    if ((type == IPROC_USBD_EP_TYPE_CTRL) || (dirn == IPROC_USBD_EP_DIR_OUT)) {
        IPROC_USBD_WRITE(base->ep_fifo_out[num].ctrl,   (type << IPROC_USBD_REG_EP_FIFO_CTRL_TYPE_SHIFT));
        IPROC_USBD_WRITE(base->ep_fifo_out[num].status, IPROC_USBD_READ(base->ep_fifo_out[num].status));
        IPROC_USBD_WRITE(base->ep_fifo_out[num].size1,  0);
        IPROC_USBD_WRITE(base->ep_fifo_out[num].size2,  ((maxPktSize >> 2) << 16) | maxPktSize);
#if IPROC_USBD_MULTI_RX_FIFO
        IPROC_USBD_BITS_SET(base->ep_fifo_out[num].size2, ((maxPktSize + 3) >> 2) << IPROC_USBD_REG_EP_FIFO_SIZE2_OUT_DEPTH_SHIFT));
#endif
    }
    if ((type == IPROC_USBD_EP_TYPE_CTRL) || (dirn == IPROC_USBD_EP_DIR_IN)) {
        IPROC_USBD_WRITE(base->ep_fifo_in[num].ctrl, (type << IPROC_USBD_REG_EP_FIFO_CTRL_TYPE_SHIFT));
        IPROC_USBD_WRITE(base->ep_fifo_in[num].size2, (maxPktSize << IPROC_USBD_REG_EP_FIFO_SIZE2_PKT_MAX_SHIFT));
        IPROC_USBD_WRITE(base->ep_fifo_in[num].size1, (maxPktSize >> 2));
        IPROC_USBD_BITS_SET(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_IN_FLUSH_ENABLE);
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_in[num].ctrl, (IPROC_USBD_REG_EP_FIFO_CTRL_NAK_SET | IPROC_USBD_REG_EP_FIFO_CTRL_IN_FLUSH_ENABLE));
    }
    IPROC_USBD_WRITE(base->ep_cfg[num], (num << IPROC_USBD_REG_EP_CFG_FIFO_NUM_SHIFT) |
                           (type << IPROC_USBD_REG_EP_CFG_TYPE_SHIFT) |
                           (maxPktSize << IPROC_USBD_REG_EP_CFG_PKT_MAX_SHIFT) |
                           (dirn == IPROC_USBD_EP_DIR_OUT ? IPROC_USBD_REG_EP_CFG_DIRN_OUT : IPROC_USBD_REG_EP_CFG_DIRN_IN));
}

/*****************************************************************************
*  @brief   Endpoint Configuration / Interface / Alternate number operations
*  @param   num - Endpoint number
*  @param   cfg - Configuration number
*  @param   intf - Interface number
*  @param   alt - Alternate number
*****************************************************************************/
static inline void iproc_usbd_ep_alt_set(struct iproc_usbd_regs *base, uint num, uint alt)
{
    IPROC_USBD_BITS_MODIFY(base->ep_cfg[num], IPROC_USBD_REG_EP_CFG_ALT_NUM_MASK, (alt << IPROC_USBD_REG_EP_CFG_ALT_NUM_SHIFT));
}

static inline void iproc_usbd_ep_cfg_set(struct iproc_usbd_regs *base, uint num, uint cfg)
{
    IPROC_USBD_BITS_MODIFY(base->ep_cfg[num], IPROC_USBD_REG_EP_CFG_CFG_NUM_MASK, (cfg << IPROC_USBD_REG_EP_CFG_CFG_NUM_SHIFT));
}

static inline void iproc_usbd_ep_intf_set(struct iproc_usbd_regs *base, uint num, uint intf)
{
    IPROC_USBD_BITS_MODIFY(base->ep_cfg[num], IPROC_USBD_REG_EP_CFG_INTF_NUM_MASK, (intf << IPROC_USBD_REG_EP_CFG_INTF_NUM_SHIFT));
}


/*****************************************************************************
*  @brief   Endpoint DMA routines
*  @param   num - Endpoint number
*  @param   addr - physical address of buffer or descriptor
*****************************************************************************/
static inline void iproc_usbd_ep_dma_dis(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
#if IPROC_USBD_MULTI_RX_FIFO
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_OUT_DMA_ENABLE);
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
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_IN_DMA_ENABLE);
    }
}

static inline void iproc_usbd_ep_dma_en(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
#if IPROC_USBD_MULTI_RX_FIFO
        IPROC_USBD_BITS_SET(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_OUT_DMA_ENABLE);
#else
        IPROC_USBD_BITS_SET(base->dev_ctrl, IPROC_USBD_REG_CTRL_DMA_OUT_ENABLE);
#endif
    } else {
        /* Set the Poll bit in the control register */
        IPROC_USBD_BITS_SET(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_IN_DMA_ENABLE);
    }
}

static inline void iproc_usbd_ep_dma_buf_addr_set(struct iproc_usbd_regs *base, uint num, uint dirn, void *addr)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_WRITE(base->ep_fifo_out[num].buf_addr, (uint)addr);
    }
}

static inline void iproc_usbd_ep_dma_desc_addr_set(struct iproc_usbd_regs *base, uint num, uint dirn, void *addr)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_WRITE(base->ep_fifo_out[num].desc_addr, (uint)addr);
    }
    else {
        IPROC_USBD_WRITE(base->ep_fifo_in[num].desc_addr, (uint)addr);
    }
}

/*****************************************************************************
*  @brief   Endpoint FIFO routines
*  @param   num - Endpoint number
*  @note    The flush operation is a state. Once enabled, FIFO contents are discared
*           until disabled. Usually enable upon endpoint termination or error, and
*           then disable once operations are to resume normally.
*****************************************************************************/
static inline bool iproc_usbd_ep_fifo_empty(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
#if IPROC_USBD_MULTI_RX_FIFO
        return(base->ep_fifo_out[num].status & IPROC_USBD_REG_EP_FIFO_STATUS_OUT_FIFO_EMPTY ? true : false);
#else
        return(base->dev_status & IPROC_USBD_REG_STAT_OUT_FIFO_EMPTY ? true : false);
#endif
    }
    return(base->ep_fifo_in[num].status & IPROC_USBD_REG_EP_FIFO_STATUS_IN_FIFO_EMPTY ? true : false);
}

static inline void iproc_usbd_ep_fifo_flush_dis(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
#if IPROC_USBD_MULTI_RX_FIFO
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_OUT_FLUSH_ENABLE);
#else
        IPROC_USBD_BITS_CLEAR(base->dev_ctrl, IPROC_USBD_REG_CTRL_OUT_FIFO_FLUSH_ENABLE);
#endif
    }
    else {
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_IN_FLUSH_ENABLE);
    }
}

static inline void iproc_usbd_ep_fifo_flush_en(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
#if IPROC_USBD_MULTI_RX_FIFO
        IPROC_USBD_BITS_SET(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_OUT_FLUSH_ENABLE);
#else
        IPROC_USBD_BITS_SET(base->dev_ctrl, IPROC_USBD_REG_CTRL_OUT_FIFO_FLUSH_ENABLE);
#endif
    } else {
        IPROC_USBD_BITS_SET(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_IN_FLUSH_ENABLE);
    }
}

/*****************************************************************************
*  @brief   Endpoint Frame Number routines
*  @param   num - Endpoint number
*  @return  Frame number of last packet received on the endpoint, and in the following format.
*               bits[13:3] milli-second frame number
*               bits[2:0] micro-frame number
*  @note    Really only applicable to OUT endpoints. IN will always return 0.
*****************************************************************************/
static inline uint iproc_usbd_ep_frame_num(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        return((IPROC_USBD_READ(base->ep_fifo_out[num].size1) & IPROC_USBD_REG_EP_FIFO_SIZE1_OUT_FRAME_NUM_MASK) >> IPROC_USBD_REG_EP_FIFO_SIZE1_OUT_FRAME_NUM_SHIFT);
    }
    return(0);
}

/*****************************************************************************
*  @brief   Endpoint IRQ / status routines
*  @param   num - Endpoint number
*  @note
*       Cannot set specific status for Endpoint interrupts. Can only do operations
*       in a global sense. Once an interrupt occurs for an endpoint, the endpoint
*       status has to be checked for the particular type of interrupt that occurred.
*
*       The iproc_usbd_ep_irq_en() and iproc_usbd_ep_irq_dis() are used for
*       operations on a specific endpoint. These routines may or may not be used in
*       the context of interrupt processing.
*
*       Use the usbDevHw_EndptIrqListXxx() routines for operations using a bit-wise
*       list of endpoints (bit 0 for endpoint 0, etc.). Typical use would be for
*       interrupt processing.
*
*       Use the IPROC_USBD_EP_STAT_xxx definitions with the status routines. These
*       definitions are bit-wise, and allow operations on multiple conditions
*       by OR'ing the definitions together.
*****************************************************************************/
static inline void iproc_usbd_ep_irq_clear(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_WRITE(base->ep_irq_status, (1 << num) << IPROC_USBD_REG_EP_INTR_OUT_SHIFT);
    } else {
        IPROC_USBD_WRITE(base->ep_irq_status, (1 << num) << IPROC_USBD_REG_EP_INTR_IN_SHIFT);
    }
}

static inline void iproc_usbd_ep_irq_dis(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_BITS_SET(base->ep_irq_mask, ((1 << num) << IPROC_USBD_REG_EP_INTR_OUT_SHIFT));
    } else {
        IPROC_USBD_BITS_SET(base->ep_irq_mask, ((1 << num) << IPROC_USBD_REG_EP_INTR_IN_SHIFT));
    }
}

static inline void iproc_usbd_ep_irq_en(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_BITS_CLEAR(base->ep_irq_mask, ((1 << num) << IPROC_USBD_REG_EP_INTR_OUT_SHIFT));
    } else {
        IPROC_USBD_BITS_CLEAR(base->ep_irq_mask, ((1 << num) << IPROC_USBD_REG_EP_INTR_IN_SHIFT));
    }
}

static inline uint iproc_usbd_ep_irq_list_active(struct iproc_usbd_regs *base, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        return((IPROC_USBD_READ(base->ep_irq_status) & IPROC_USBD_REG_EP_INTR_OUT_MASK) >> IPROC_USBD_REG_EP_INTR_OUT_SHIFT);
    }
    return((IPROC_USBD_READ(base->ep_irq_status) & IPROC_USBD_REG_EP_INTR_IN_MASK) >> IPROC_USBD_REG_EP_INTR_IN_SHIFT);
}

static inline void iproc_usbd_ep_irq_list_clear(struct iproc_usbd_regs *base, uint dirn, uint mask)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_WRITE(base->ep_irq_status, (mask << IPROC_USBD_REG_EP_INTR_OUT_SHIFT)); /*strat from bit 16 */
    } else {
        IPROC_USBD_WRITE(base->ep_irq_status, (mask << IPROC_USBD_REG_EP_INTR_IN_SHIFT)); /* start from bit 0 */
    }
}

static inline uint iproc_usbd_ep_stat_active(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        return(IPROC_USBD_READ(base->ep_fifo_out[num].status));  /* End Point Status register */
    }
    return(IPROC_USBD_READ(base->ep_fifo_in[num].status));
}

static inline void iproc_usbd_ep_stat_clear(struct iproc_usbd_regs *base, uint num, uint dirn, uint mask)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_WRITE(base->ep_fifo_out[num].status, mask);
    } else {
        IPROC_USBD_WRITE(base->ep_fifo_in[num].status, mask);
    }
}

/*****************************************************************************
*  @brief   Endpoint NAK routines
*  @param   num - Endpoint number
*  @note    A NAK response can be enabled by the application by the EndptNakEnable().
*           The EndptNakInProgress() is used to determine if the controller is
*           currently actively sending NAKs. This may have been a result of the
*           EndptNakEnable() or automatically by the controller under certain
*           conditions. The EndptNakClear() must be used to terminate the NAKs.
*****************************************************************************/
static inline void iproc_usbd_ep_nak_clear(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_BITS_SET(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_NAK_CLEAR);
    } else {
        IPROC_USBD_BITS_SET(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_NAK_CLEAR);
    }
}

static inline void iproc_usbd_ep_nak_en(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_BITS_SET(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_NAK_SET);
    } else {
        IPROC_USBD_BITS_SET(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_NAK_SET);
    }
}

static inline void iproc_usbd_ep_nak_dis(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_NAK_SET);
    } else {
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_NAK_SET);
    }
}

static inline bool iproc_usbd_ep_nak_progress(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        return (IPROC_USBD_READ(base->ep_fifo_out[num].ctrl) & IPROC_USBD_REG_EP_FIFO_CTRL_NAK_IN_PROGRESS) ? true : false;
    }
    return (IPROC_USBD_READ(base->ep_fifo_in[num].ctrl) & IPROC_USBD_REG_EP_FIFO_CTRL_NAK_IN_PROGRESS) ? true : false;
}

/*****************************************************************************
*  @brief   Endpoint Stall routines
*           Disable / Enable STALL responses (halt feature) on a given endpoint.
*  @param   num - Endpoint number
*****************************************************************************/
static inline void iproc_usbd_ep_stall_dis(struct iproc_usbd_regs *base, uint num, uint dirn)
{
    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_STALL_ENABLE);
    } else {
        IPROC_USBD_BITS_CLEAR(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_STALL_ENABLE);
    }
}

static inline void iproc_usbd_ep_stall_en(struct iproc_usbd_regs *base, uint num, uint dirn)
{
#if IPROC_USBD_MULTI_RX_FIFO
    if (!(IPROC_USBD_READ(base->ep_fifo_out[num].status) & IPROC_USBD_REG_EP_FIFO_STATUS_OUT_FIFO_EMPTY))
#else
    if (!(IPROC_USBD_READ(base->dev_status) & IPROC_USBD_REG_STAT_OUT_FIFO_EMPTY))
#endif
        return;

    if (dirn == IPROC_USBD_EP_DIR_OUT) {
        IPROC_USBD_BITS_SET(base->ep_fifo_out[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_STALL_ENABLE);
    } else {
        IPROC_USBD_BITS_SET(base->ep_fifo_in[num].ctrl, IPROC_USBD_REG_EP_FIFO_CTRL_STALL_ENABLE);
    }
}


/*****************************************************************************
*  @brief   Initialize device controller operations
*****************************************************************************/
static inline void iproc_usbd_ops_init(struct iproc_usbd_regs *base)
{
    int idx;

    iproc_usbd_dma_dis(base);
    iproc_usbd_irq_dis(base, IPROC_USBD_IRQ_ALL);
    iproc_usbd_irq_clear(base, IPROC_USBD_IRQ_ALL);

    /* @todo Create and use usbDevHw_EndptIrqListDisable?? */
    for (idx = 0; idx < IPROC_USBD_EP_CFG_CNT; idx++) {
        iproc_usbd_ep_irq_dis(base, idx, IPROC_USBD_EP_DIR_IN);
        iproc_usbd_ep_irq_clear(base, idx, IPROC_USBD_EP_DIR_IN);
        iproc_usbd_ep_stat_clear(base, idx, IPROC_USBD_EP_DIR_IN, iproc_usbd_ep_stat_active(base, idx, IPROC_USBD_EP_DIR_IN));

        iproc_usbd_ep_irq_dis(base, idx, IPROC_USBD_EP_DIR_OUT);
        iproc_usbd_ep_irq_clear(base, idx, IPROC_USBD_EP_DIR_OUT);
        iproc_usbd_ep_stat_clear(base, idx, IPROC_USBD_EP_DIR_OUT, iproc_usbd_ep_stat_active(base, idx, IPROC_USBD_EP_DIR_OUT));
    }

    IPROC_USBD_WRITE(base->dev_cfg, (IPROC_USBD_REG_CFG_SET_DESCRIPTOR_ENABLE |
                           IPROC_USBD_REG_CFG_UTMI_8BIT_ENABLE |
                           IPROC_USBD_REG_CFG_CSR_PROGRAM_ENABLE |
                           IPROC_USBD_REG_CFG_SPD_HS));

    IPROC_USBD_WRITE(base->dev_ctrl, (IPROC_USBD_REG_CTRL_LE_ENABLE |
                            IPROC_USBD_REG_CTRL_DISCONNECT_ENABLE |
                            IPROC_USBD_REG_CTRL_DMA_MODE_ENABLE |
                            IPROC_USBD_REG_CTRL_DMA_IN_ENABLE |
                            IPROC_USBD_REG_CTRL_DMA_OUT_ENABLE |
                            IPROC_USBD_REG_CTRL_DMA_DESC_UPDATE_ENABLE |
                            IPROC_USBD_REG_CTRL_OUT_NAK_ALL_ENABLE |
                            IPROC_USBD_REG_CTRL_DMA_OUT_THRESHOLD_LEN_MASK |
                            IPROC_USBD_REG_CTRL_DMA_BURST_LEN_MASK |
#if !IPROC_USBD_MULTI_RX_FIFO
                            IPROC_USBD_REG_CTRL_OUT_FIFO_FLUSH_ENABLE |
#endif
                            IPROC_USBD_REG_CTRL_DMA_BURST_ENABLE));

    IPROC_USBD_WRITE(base->dev_irq_mask, (IPROC_USBD_REG_INTR_BUS_IDLE |
                                         IPROC_USBD_REG_INTR_SOF_RX));
    IPROC_USBD_WRITE(base->ep_irq_mask,0);
}

/*****************************************************************************
*  @brief   Disable / Enable USB device
*****************************************************************************/
static inline void iproc_usbd_dis(struct iproc_usbd_idm_regs *idm_base)
{
    /* reset usb device */
    IPROC_USBD_BITS_SET(idm_base->reset_ctrl, IPROC_USB2D_IDM_REG_RESET_CTRL_RESET);

    /* disable usb device clock */
    IPROC_USBD_BITS_CLEAR(idm_base->io_ctrl, IPROC_USB2D_IDM_REG_IO_CTRL_DIRECT_CLK_ENABLE);
    mdelay(10);
}

static inline void iproc_usbd_en(struct iproc_usbd_idm_regs *idm_base)
{
    /* enable usb device clock */
    IPROC_USBD_BITS_SET(idm_base->io_ctrl, IPROC_USB2D_IDM_REG_IO_CTRL_DIRECT_CLK_ENABLE);
    mdelay(10);

    /* get usb device out of reset */
    IPROC_USBD_BITS_CLEAR(idm_base->reset_ctrl, IPROC_USB2D_IDM_REG_RESET_CTRL_RESET);
    mdelay(100);
}



#endif /* _USBD_REGS_H_ */
