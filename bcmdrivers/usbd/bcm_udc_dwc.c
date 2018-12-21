/*****************************************************************************
* Copyright 2006 - 2010 Broadcom Corporation.  All rights reserved.
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


/****************************************************************************/
/**
*  @file    bcm_dwc_udc.c
*
*  @brief   Broadcom Linux driver for DWC USB 2.0 Device Controller (UDC)
*
*  This driver implements the Linux Gadget driver API as defined in usb_gadget.h
*
*  @note
*
*  This driver was written with the intent of being able to support any
*  variations on how this block is integrated into different Broadcom chips.
*
*  There is a requirement on how the DWC UDC is configured. In particular, this
*  driver requires that the following options be defined and enabled in the
*  UDC core.
*
*       UDC20AHB_CNAK_CLR_ENH_CC
*       UDC20AHB_STALL_SET_ENH_CC
*       UDC20AHB_SNAK_ENH_CC
*
*  Some other UDC attributes can be supported by setting compile time options
*  or with some minor modifications to the source code. Ideally these would
*  be run-time info that is provided by the device instance to the driver.
*  These attributes include the following.
*
*       BCM_UDC_EP_CNT
*       BCM_UDC_EP_MAX_PKT_SIZE
*       BCM_UDC_OUT_RX_FIFO_MEM_SIZE
*       BCM_UDC_IN_TX_FIFO_MEM_SIZE
*       BCM_UDC_IRQ
*       Type of each endpoint: Control, IN, OUT, or Bidirectional
*/
/****************************************************************************/

#define BCM_UDC_EP_CNT                  10
#define BCM_UDC_EP_MAX_PKT_SIZE         512
#define BCM_UDC_OUT_RX_FIFO_MEM_SIZE    1024
#define BCM_UDC_IN_TX_FIFO_MEM_SIZE     4096

#ifdef CONFIG_ARCH_IPROC
#define BCM_UDC_IN_TX_MAX        (512)
#else
#error "No valid ARCH selected"
#endif


/* ---- Include Files ---------------------------------------------------- */

#include <linux/version.h>

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>

//#include <linux/broadcom/knllog.h>

#include "usbDevHw.h"

#ifdef CONFIG_PM_USBD
#include "../pm/iproc_pm_device.h"
#endif

#ifdef CONFIG_ARCH_BCMRING
static int gVbusDetectGpio = -1;
#include <linux/broadcom/gpio.h> /* gpio_get_value */
#include <linux/broadcom/bcmring/gpio_defs.h>
#include <mach/csp/gpiomux.h>
#include <mach/csp/chipcHw_inline.h>
#include <mach/csp/chipcHw_def.h>
#include <mach/csp/cap.h>
#include <csp/gpioHw.h>
#endif

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif // CONFIG_HAS_WAKELOCK

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK) && \
	defined(CONFIG_USB_WAKESOURCE_BCMRING_UDC)
static struct wake_lock wakelock;
const char   *wake_lock_name = "bcm_udc_dwc";
#define WAKE_LOCK_ACTIVE_TIMEOUT_DURATION 10*HZ
#define WAKE_LOCK_INACTIVE_TIMEOUT_DURATION 5*HZ
#endif 

/*
 * Name definitions for use with driver and devices. Device names will have
 * a digit appended. Note the correlation to the BCM_USBEHCI_HCD_MAX value
 * (max value is a single digit, hence the sizeof() + 1). Since this name
 * has to be the same in the device and the driver, really should have it
 * defined in some common include file used by both device and driver.
 */
#define BCM_UDC_NAME                "iproc-udc"

#ifndef BCM_UDC_KMARKER
#define BCM_UDC_KMARKER             BCM_UDC_NAME": "
#endif

#define BCM_KLOG(lvl, fmt, args...)     printk( lvl BCM_UDC_KMARKER fmt, ## args )

static unsigned debug = 0x0;

/* #define DEBUG */

#define DEBUG_DEV                   0x00000001
#define DEBUG_EP                    0x00000002
#define DEBUG_MEM                   0x00000004
#define DEBUG_REQ                   0x00000008
#define DEBUG_IRQ                   0x00000010
#define DEBUG_DMA                   0x00000020
#define DEBUG_ISOC                  0x00000040
#define DEBUG_CTRL                  0x00000080
#define DEBUG_SOF                   0x00000100

#define DEBUG_EVENT                 0x80000000
#define DEBUG_TRACE                 0x40000000
#define DEBUG_CNT                   0x20000000


#ifdef DEBUG

    /*
     * Add any normal log messages (syslogd) to the kernel log as well. Makes things easier
     * to interpret if the two are interleaved.
     */

    #define BCM_KERROR(fmt, args...)    KNLLOG(fmt, ## args); BCM_KLOG( KERN_ERR, fmt, ## args )
    #define BCM_KWARN(fmt, args...)     KNLLOG(fmt, ## args); BCM_KLOG( KERN_WARNING, fmt, ## args )
    #define BCM_KNOTICE(fmt, args...)   KNLLOG(fmt, ## args); BCM_KLOG( KERN_NOTICE, fmt, ## args )
    #define BCM_KINFO(fmt, args...)     KNLLOG(fmt, ## args); BCM_KLOG( KERN_INFO, fmt, ## args )

    /*
     * Debug output is controlled by areas as opposed to levels. It makes things more flexible
     * wrt the amount and type of output.
     *
     * NOTE: The kernel log utility only allows 6 parameters for a format string. If you have
     * a BCM_DEBUG_xxx() that uses more than this, you will need to change the appropriate
     * knllog parameter before you start this driver. See include/linux/broadcom/knllog.h
     * and drivers/char/broadcom/knllog.c for more details on the appropriate /proc/sys/knllog
     * entry to use. Currently this is maxargs. Probably easier / safer to use more than one
     * BCM_DEBUG_xxx() statement if need to output more than 6 parameters.
     */

    #define BCM_DEBUG_DEV               if (debug & DEBUG_DEV)  KNLLOG
    #define BCM_DEBUG_EP                if (debug & DEBUG_EP)   KNLLOG
    #define BCM_DEBUG_MEM               if (debug & DEBUG_MEM)  KNLLOG
    #define BCM_DEBUG_REQ               if (debug & DEBUG_MEM)  KNLLOG
    #define BCM_DEBUG_IRQ               if (debug & DEBUG_IRQ)  KNLLOG
    #define BCM_DEBUG_DMA               if (debug & DEBUG_DMA)  KNLLOG
    #define BCM_DEBUG_ISOC              if (debug & DEBUG_ISOC) KNLLOG
    #define BCM_DEBUG_CTRL              if (debug & DEBUG_CTRL) KNLLOG
    #define BCM_DEBUG_SOF               if (debug & DEBUG_SOF)  KNLLOG

    #ifdef DEBUG_VERBOSE
        #define BCM_DEBUG_TRACE         if (debug & DEBUG_TRACE) KNLLOG
    #else
        #define BCM_DEBUG_TRACE(fmt...)
    #endif

#else

    #define BCM_KERROR(fmt, args...)    BCM_KLOG( KERN_ERR, fmt, ## args )
    #define BCM_KWARN(fmt, args...)     BCM_KLOG( KERN_WARNING, fmt, ## args )
    #define BCM_KNOTICE(fmt, args...)   BCM_KLOG( KERN_NOTICE, fmt, ## args )
    #define BCM_KINFO(fmt, args...)     BCM_KLOG( KERN_INFO, fmt, ## args )
    #define BCM_DEBUG_DEV(fmt...)
    #define BCM_DEBUG_EP(fmt...)
    #define BCM_DEBUG_MEM(fmt...)
    #define BCM_DEBUG_REQ(fmt...)
    #define BCM_DEBUG_IRQ(fmt...)
    #define BCM_DEBUG_DMA(fmt...)
    #define BCM_DEBUG_ISOC(fmt...)
    #define BCM_DEBUG_CTRL(fmt...)
    #define BCM_DEBUG_SOF(fmt...)
    #define BCM_DEBUG_TRACE(fmt...)
#endif

#ifndef CONFIG_ARCH_IPROC
#include <mach/io_map.h>
#endif

/*
 * Uncomment the following REG_DEBUG and REG_DEBUG_PRINT if it is desired to have
 * the register modification routines defined in usbDevHw_def.h and usbPhyHw_def.h
 * output debug info.
 *
 * #define usbDevHw_DEBUG_REG
 * #define usbDevHw_DEBUG_REG_PRINT         printk
 * #define usbPhyHw_DEBUG_REG
 * #define usbPhyHw_DEBUG_REG_PRINT         printk
 */

/*
 * Define and set DMA_BURST_LEN_32BIT if it is desired to use a DMA burst length
 * other than the default which is 16 (INCR16). Set to 0 to disable burst mode
 * and use INCR.
 *
 * #define usbDevHw_DMA_BURST_LEN_32BIT     0
 */

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) || \
	defined(CONFIG_MACH_IPROC_P7))
#define BCM_UDC_IRQ            IPROC_USB2D_IRQ
#else
#error "No valid USBD IRQ selected"
#endif

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
#define IPROC_XGPLL							0x1803fc2c
#define IPROC_XGPLL_VA						HW_IO_PHYS_TO_VIRT(IPROC_XGPLL)
#define IPROC_USB_PHY_CTRL					IPROC_WRAP_USBPHY_CTRL
#define IPROC_USB_PHY_CTRL_VA				HW_IO_PHYS_TO_VIRT(IPROC_USB_PHY_CTRL)
#define IPROC_CLK_NDIV_40					0x80
#define IPROC_CLK_NDIV_20					0x8C
#define USB_CLK_NDIV_MASK					0xFE7FFE00
#define USB_CLK_PLL_RESET_MASK				0xFF7FFE00
#define USB_CLK_PHY_RESET_MASK				0xFFFFFE00
#define USB_CLK_NDIV_40						0x30
#define USB_CLK_NDIV_20						0x60
#define ChipcommonA_GPIOInput_VA			HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOInput)
#define ChipcommonA_GPIOOut_VA				HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOOut)
#define ChipcommonA_GPIOOutEn_VA			HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOOutEn)
#define SUPPLY_USBD_POWER					0xfffffffd
#define USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA  HW_IO_PHYS_TO_VIRT(USB2D_IDM_IDM_IO_CONTROL_DIRECT)
#define USB2D_IDM_IDM_RESET_CONTROL_VA      HW_IO_PHYS_TO_VIRT(USB2D_IDM_IDM_RESET_CONTROL)
#define ChipcommonB_MII_Management_Control_VA HW_IO_PHYS_TO_VIRT(ChipcommonB_MII_Management_Control)
#define ChipcommonB_MII_Management_Command_Data_VA HW_IO_PHYS_TO_VIRT(ChipcommonB_MII_Management_Command_Data)
#elif defined(CONFIG_MACH_IPROC_P7)
#define USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA  HW_IO_PHYS_TO_VIRT(USB2D_IDM_IDM_IO_CONTROL_DIRECT)
#define USB2D_IDM_IDM_RESET_CONTROL_VA      HW_IO_PHYS_TO_VIRT(USB2D_IDM_IDM_RESET_CONTROL)
#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
#define IPROC_WRAP_USBPHY_CTRL_0			0x1800fc44
#define IPROC_WRAP_USBPHY_CTRL_0_VA			HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_0)
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ		26 /* Port 0 */
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB		25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB		24
#define IPROC_WRAP_USBPHY_CTRL_2			0x1800fc4c
#define IPROC_WRAP_USBPHY_CTRL_2_VA			HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_2)
#define IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO		17
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0		0
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11		11
#define IPROC_WRAP_MISC_STATUS				0x1800fc58
#define IPROC_WRAP_MISC_STATUS_VA			HW_IO_PHYS_TO_VIRT(IPROC_WRAP_MISC_STATUS)
#define IPROC_WRAP_MISC_STATUS__USBPHY_LDO_ON_FLAG	2
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK		1
#define USBD_VBUS_GPIO					10
#elif defined(CONFIG_MACH_SB2)
#define IPROC_WRAP_USBPHY_CTRL_0			0x1800fc28
#define IPROC_WRAP_USBPHY_CTRL_0_VA			HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_0)
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO		27
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB		25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB		24
#define IPROC_WRAP_USBPHY_CTRL_2			0x1800fc30
#define IPROC_WRAP_USBPHY_CTRL_2_VA			HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_2)
#define IPROC_WRAP_USBPHY_CTRL_5			0x1800fc3c
#define IPROC_WRAP_USBPHY_CTRL_5_VA			HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_5)
#define IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B0		0
#define IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B11		11
#define IPROC_WRAP_MISC_STATUS				0x1800fc44
#define IPROC_WRAP_MISC_STATUS_VA			HW_IO_PHYS_TO_VIRT(IPROC_WRAP_MISC_STATUS)
/* #define IPROC_WRAP_MISC_STATUS__USBPHY_LDO_ON_FLAG	2 */
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK		1
#define USBD_VBUS_GPIO					1
#define IPROC_WRAP_IPROC_STRAP_CTRL   0x1800fc70

#define USBH_Utmi_p0Ctl               0x18049510
#define USBH_Utmi_p0Ctl_VA            HW_IO_PHYS_TO_VIRT(USBH_Utmi_p0Ctl)
#define ICFG_USB2D_CONFIG_0           0x18000370
#define ICFG_USB2D_CONFIG_0_VA        HW_IO_PHYS_TO_VIRT(ICFG_USB2D_CONFIG_0)
#define ICFG_USB2D_CONFIG_1           0x18000374
#define ICFG_USB2D_CONFIG_1_VA        HW_IO_PHYS_TO_VIRT(ICFG_USB2D_CONFIG_1)
#define ICFG_USB2D_CONFIG_2           0x18000378
#define ICFG_USB2D_CONFIG_2_VA        HW_IO_PHYS_TO_VIRT(ICFG_USB2D_CONFIG_2)
#define USB2D_DEVCONFIG               0x1804c400
#define USB2D_DEVCONFIG_VA            HW_IO_PHYS_TO_VIRT(USB2D_DEVCONFIG)
#define USB2D_DEVCTRL                 0x1804c404
#define USB2D_DEVCTRL_VA              HW_IO_PHYS_TO_VIRT(USB2D_DEVCTRL)

#endif   /* defined(CONFIG_MACH_SB2) */
#endif


/* Would be nice if DMA_ADDR_INVALID or similar was defined in dma-mapping.h */
#define DMA_ADDR_INVALID                (~(dma_addr_t)0)

/* Would be nice if ENOERROR or similar was defined in errno.h */
#define ENOERROR                        0

/* Would be nice if USB_DIR_MASK was defined in usb/ch9.h */
#define USB_DIR_MASK                    USB_ENDPOINT_DIR_MASK
#define USB_DIR_UNKNOWN                 ~USB_DIR_MASK
#define USB_CONTROL_MAX_PACKET_SIZE     64

/* ---- External Variable Declarations ----------------------------------- */
/* ---- External Function Prototypes ------------------------------------- */
/* ---- Public Variables ------------------------------------------------- */
/* ---- Private Constants and Types -------------------------------------- */

#define BCM_UDC_MODULE_DESCRIPTION      "Broadcom USB Device Controller (UDC) driver"
#define BCM_UDC_MODULE_VERSION          "1.0.0"

#include "bcm_udc_dwc.h"

#define BCM_DRIVER_DESC                 BCM_UDC_MODULE_DESCRIPTION
#define BCM_DRIVER_VERSION              BCM_UDC_MODULE_VERSION

/*
 * Definitions for the number of USB Device Controllers (UDC's) supported. Usually there's
 * just 1. Note that numbering is 0 .. (BCM_UDC_CNT_MAX - 1)
 */

#define BCM_UDC_CNT_MAX                 9

/*
 * FRAME_NUM_INVALID is used for ISOC IN transfers for frame alignment.
 * The device specifies the interval at which it wants to do transfers,
 * but the host initiates all transfers. If the interval is some multiple
 * number of frames, the device has no idea which frame in an interval
 * window the host is going to start transfers. This could even be at a
 * point many frames beyond the current window, as the starting point
 * can be very application dependant and subject to an indeterminate
 * amount of latency.
 */

#define FRAME_NUM_INVALID               (~(unsigned)0)

/*
 * Normally ISOC IN DMA transfers are disabled until such time that
 * an IN token is received from the host so that proper frame number
 * alignment is achieved. See previous comments regarding FRAME_NUM_INVALID.
 * If it desired that frames start being DMA'd w/o this alignment, then
 * define ISOC_IN_XFER_DELAY_DISABLED. If things are not aligned, they
 * should correct themselves when an IN token interrupt is received.
 * See IrqEpInStatusCheck() processing.
 *
 * #define ISOC_IN_XFER_DELAY_DISABLED
 */


/*
 * For IN DMA transfers, problems have been noticed in the UDC h/w when
 * buffer fill mode is used (descriptor length is set to buffer length
 * as opposed to packet length. Only applicable to cases where buffer
 * length is larger than max packet). To enable buffer fill mode for
 * IN transfers, define IN_DMA_BUFFER_FILL_ENABLED.
 */
//#define IN_DMA_BUFFER_FILL_ENABLED


/* ---- Private Function Prototypes -------------------------------------- */

/** @todo Rename dirn to dir to match USB_DIR_xxx def'ns */
#define DIRN_STR(dirn)                  ((dirn) == USB_DIR_IN ? "IN" : "OUT")

static void CtrlEpSetupInit( BCM_UDC_EP_t *udcEpP, int status );
static void CtrlEpSetupRx( BCM_UDC_EP_t *udcEpP, struct usb_ctrlrequest *setup );


static void DmaEpInit( BCM_UDC_EP_t *udcEpP );

#ifdef DEBUG
static void DmaDump( BCM_UDC_t *udcP );
#endif
static void DmaDumpDesc( char *label, BCM_UDC_DMA_DESC_t *virt, BCM_UDC_DMA_DESC_t *phys );
static void DmaDumpEp( BCM_UDC_EP_t *udcEpP );

static void DmaDataInit( BCM_UDC_EP_t *udcEpP );
static void DmaDataFinis( BCM_UDC_EP_t *udcEpP );
static void DmaDataAddReady( BCM_UDC_EP_t *udcEpP );
static void DmaDataRemoveDone( BCM_UDC_EP_t *udcEpP );

static inline BCM_UDC_DMA_DESC_t *DmaDescChainAlloc( BCM_UDC_EP_t *udcEpP );
static inline int DmaDescChainEmpty( BCM_UDC_EP_t *udcEpP );
static inline void DmaDescChainFree( BCM_UDC_EP_t *udcEpP );
static inline int DmaDescChainFull( BCM_UDC_EP_t *udcEpP );
static inline BCM_UDC_DMA_DESC_t *DmaDescChainHead( BCM_UDC_EP_t *udcEpP );
static inline void DmaDescChainReset( BCM_UDC_EP_t *udcEpP );


static int GadgetDevAdd( BCM_UDC_t *udcP );
static int GadgetDevInit( struct platform_device *devP, BCM_UDC_t *udcP );
static void GadgetDevRelease( struct device *dev );
static int GadgetDevRemove( BCM_UDC_t *udcP );

#ifdef CONFIG_ARCH_BCMRING
static int GadgetDevWakeup( struct usb_gadget *gadget );
static int GadgetDevSetSelfpowered( struct usb_gadget *gadget, int is_selfpowered );
static int GadgetDevVbusSession( struct usb_gadget *gadget, int is_active );
static int GadgetDevPullup( struct usb_gadget *gadget, int is_on );
#endif

static int GadgetEpEnable(struct usb_ep *ep, const struct usb_endpoint_descriptor *desc);
static int GadgetEpDisable(struct usb_ep *ep);
static struct usb_request *GadgetEpRequestAlloc(struct usb_ep *ep, unsigned gfp_flags);
static void GadgetEpRequestFree(struct usb_ep *ep, struct usb_request *req);
static int GadgetEpRequestQueue(struct usb_ep *ep, struct usb_request *req, unsigned gfp_flags);
static int GadgetEpRequestDequeue(struct usb_ep *ep, struct usb_request *req);
static int GadgetEpSetHalt(struct usb_ep *ep, int value);
static int GadgetEpFifoStatus(struct usb_ep *ep);
static void GadgetEpFifoFlush(struct usb_ep *ep);

static irqreturn_t  IrqUdc( int irq, void *context );

static void IrqDev( BCM_UDC_t *udcP, uint32_t irq );
static void IrqDevCfgSet( BCM_UDC_t *udcP );
static void IrqDevIntfSet( BCM_UDC_t *udcP );
static void IrqDevSpeedEnum( BCM_UDC_t *udcP );

static void IrqEp( BCM_UDC_t *udcP, uint32_t irqIn, uint32_t irqOut );
static void IrqEpInStatusCheck( BCM_UDC_EP_t *udcEpP );
static void IrqEpOutStatusCheck( BCM_UDC_EP_t *udcEpP );
static void IrqEpOutSetup( BCM_UDC_EP_t *udcEpP );


static int PlatformDriverProbe( struct platform_device *devP );
static int PlatformDriverRemove( struct platform_device *devP );
static int PlatformDmaAlloc( struct platform_device *platformDevP, BCM_UDC_t *udcP );
static void PlatformDmaFree( struct platform_device *platformDevP, BCM_UDC_t *udcP );

static void ProcFileCreate(void);
static void ProcFileRemove(void);


static void ReqQueueFlush( BCM_UDC_EP_t *udcEpP, int status );
static void ReqXferError( BCM_UDC_EP_t *udcEpP, int status );
static void ReqXferDone( BCM_UDC_EP_t *udcEpP, BCM_UDC_EP_REQ_t *udcEpReqP, int status );
static void ReqXferProcess( BCM_UDC_EP_t *udcEpP );
static void ReqXferAdd( BCM_UDC_EP_t *udcEpP, BCM_UDC_EP_REQ_t *udcEpReqP );


static void UdcOpsFinis( BCM_UDC_t *udcP );
static void UdcOpsInit( BCM_UDC_t *udcP );
static void UdcOpsShutdown( BCM_UDC_t *udcP );
static void UdcOpsStartup( BCM_UDC_t *udcP );
static void UdcOpsDisconnect( BCM_UDC_t *udcP );
#ifdef CONFIG_PM_USBD
static int  UdcOpsResume(BCM_UDC_t *udcP);
static int  UdcOpsSuspend(BCM_UDC_t *udcP);
static int  UdcPmSuspend(struct device *dev);
static int  UdcPmResume(struct device *dev);
static SIMPLE_DEV_PM_OPS(udc_pm, UdcPmSuspend, UdcPmResume);
#endif

static void UdcEpInit( BCM_UDC_t *udcP, unsigned num, const char *name, unsigned dirn );
static int UdcEpCfg( BCM_UDC_EP_t *udcEpP, unsigned type, unsigned maxPktSize );


static void UdcFifoRamInit( BCM_UDC_t *udcP );
static int UdcFifoRamAlloc( BCM_UDC_EP_t *udcEpP, unsigned maxPktSize );
static void UdcFifoRamFree( BCM_UDC_EP_t *udcEpP );

/* ---- Private Variables ------------------------------------------------ */

/** @todo Can this not be a global?? Part of platform device?? */

BCM_UDC_t *bcmUdcP = NULL;

#ifdef    CONFIG_PM_USBD
static int PlatformDriverSuspend(struct platform_device *dev, pm_message_t state);
static int PlatformDriverResume(struct platform_device *dev);
#ifdef CONFIG_ARCH_IPROC
extern int iproc_pm_udc_wakeup(void);
#endif
#else
#define    PlatformDriverSuspend    NULL
#define    PlatformDriverResume    NULL
#endif

/*
 * Generic platform device driver definition.
 */
static struct platform_driver bcm_udc_PlatformDriver =
{
    .probe      = PlatformDriverProbe,
    .remove     = PlatformDriverRemove,
    .suspend =    PlatformDriverSuspend,
    .resume =     PlatformDriverResume,

    .driver =
    {
        .name   = BCM_UDC_NAME,
        .owner  = THIS_MODULE,
#ifdef CONFIG_PM_USBD
		.pm    = &udc_pm,
#endif
    },
};

#ifdef CONFIG_ARCH_IPROC
/*Using SRAM as DMA for USB */

static struct resource iproc_udc_resource[] =
{
        [0] =
                {
                .flags  = IORESOURCE_IRQ,
                .start  =  BCM_UDC_IRQ,
                },

        [1] =
                {
                .flags  = IORESOURCE_MEM,
                .start  = IPROC_USB2D_REG_BASE,
                .end    = IPROC_USB2D_REG_BASE + IPROC_USB2D_REG_SIZE - 1,
                },
};

static void iproc_udc_release (struct device *dev) {}

static struct platform_device iproc_udc_pdev = {

        .name   = BCM_UDC_NAME,
        .id     = 0,
        .dev    = {
                        .coherent_dma_mask = DMA_BIT_MASK(32),
                        .dma_mask = &iproc_udc_pdev.dev.coherent_dma_mask,
                        .release = iproc_udc_release,
                },
        .resource =iproc_udc_resource,
        .num_resources=ARRAY_SIZE(iproc_udc_resource),

};
#endif

static int bcm_udc_start(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *));
static int bcm_udc_stop(struct usb_gadget_driver *driver);


static struct usb_gadget_ops bcm_udc_gadgetDevOps =
{
    .get_frame          = NULL,
#ifdef CONFIG_ARCH_BCMRING
             .wakeup             = GadgetDevWakeup,
             .set_selfpowered    = GadgetDevSetSelfpowered,
             .vbus_session       = GadgetDevVbusSession,
           .pullup             = GadgetDevPullup,
#else
             .wakeup             = NULL,
             .set_selfpowered    = NULL,
             .vbus_session       = NULL,
           .pullup             = NULL,
#endif

    .vbus_draw          = NULL,
    .ioctl              = NULL,
	.start			= bcm_udc_start,
	.stop			= bcm_udc_stop,

};

static struct usb_ep_ops bcm_udc_gadgetEpOps =
{
    .enable         = GadgetEpEnable,
    .disable        = GadgetEpDisable,

    .alloc_request  = GadgetEpRequestAlloc,
    .free_request   = GadgetEpRequestFree,
    .queue          = GadgetEpRequestQueue,
    .dequeue        = GadgetEpRequestDequeue,

    #ifdef CONFIG_USB_GADGET_USE_DMA_MAP

    /** @todo Something does not seem right with how this config item is being handled in other
     * device controller drivers. This is all that had to be done for things to work here.
     * However, it is required that this config item be defined, otherwise there may be DMA
     * alignment issues. This config item has no real effect within this driver, but it
     * does within some gadget drivers like the ethernet gadget.
     */

    .queue_dma      = GadgetEpRequestQueue,
    #endif

    .set_halt       = GadgetEpSetHalt,

    .fifo_status    = GadgetEpFifoStatus,
    .fifo_flush     = GadgetEpFifoFlush,
};

/* Returns USB PHY PLL ref clock in MHz */
static unsigned int get_usbref_clk(void)
{
	unsigned int ndiv = 0, mdiv = 1, usbrefclk;
#if defined(CONFIG_MACH_HX4)

#define IPROC_WRAP_IPROC_XGPLL_CTRL_4_VA	HW_IO_PHYS_TO_VIRT(IPROC_WRAP_IPROC_XGPLL_CTRL_4)
#define IPROC_WRAP_IPROC_XGPLL_CTRL_0_VA	HW_IO_PHYS_TO_VIRT(IPROC_WRAP_IPROC_XGPLL_CTRL_0)

	ndiv = readl_relaxed(IPROC_WRAP_IPROC_XGPLL_CTRL_4_VA) & 0xff;
	BCM_DEBUG_DEV("ctrl4 = %x\n", readl_relaxed(IPROC_WRAP_IPROC_XGPLL_CTRL_4_VA));
	BCM_DEBUG_DEV("ctrl0 = %x\n", readl_relaxed(IPROC_WRAP_IPROC_XGPLL_CTRL_0_VA));
	mdiv = (readl_relaxed(IPROC_WRAP_IPROC_XGPLL_CTRL_0_VA) >> IPROC_WRAP_IPROC_XGPLL_CTRL_0__CH3_MDIV_R) & 0xff; /* Ch3 MDIV */

#elif defined(CONFIG_MACH_KT2)

#define IPROC_DDR_PLL_CTRL_REGISTER_3_VA HW_IO_PHYS_TO_VIRT(IPROC_DDR_PLL_CTRL_REGISTER_3)
#define IPROC_DDR_PLL_CTRL_REGISTER_5_VA HW_IO_PHYS_TO_VIRT(IPROC_DDR_PLL_CTRL_REGISTER_5)

	ndiv = (readl_relaxed(IPROC_DDR_PLL_CTRL_REGISTER_3_VA) >> 
		    IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_R) & 
			((1 << IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_WIDTH) - 1);
	
	/* read channel 1 mdiv */
	mdiv = (readl_relaxed(IPROC_DDR_PLL_CTRL_REGISTER_5_VA) >> 
		IPROC_DDR_PLL_CTRL_REGISTER_5__CH1_MDIV_R) & 
		((1 << IPROC_DDR_PLL_CTRL_REGISTER_5__CH1_MDIV_WIDTH) - 1);
#endif

	BCM_DEBUG_DEV("XGPLL ndiv = %d, mdiv = %d\n", ndiv, mdiv);

	usbrefclk = (25 * ndiv) / mdiv;

	BCM_DEBUG_DEV("USB refclk = %d\n", usbrefclk);

	return(usbrefclk);
}
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
static void config_usb_phy(void)
{
	unsigned int usbrefclk, ndiv, precmd, miicmd, miidata;
#define START_SEQ 0x9A
#define MII_WRITE 0x1
#define MII_READ  0x2
	/* Get the USB PHY ref clock and configure USB PLL for 60MHz USB Clock */

	usbrefclk = get_usbref_clk();

	ndiv = 1920 / usbrefclk;

	/* Construct precmd with Start Bit, PHY address and turnaround time */
	/* SB | PA | TA */
	precmd = 1 << 30 | 6 << 23 | 2 << 16;
	
	/* Connect MDIO interface to onchip PHY */
	writel_relaxed(START_SEQ, ChipcommonB_MII_Management_Control_VA);
	mdelay(10);

	/* Program NDIV and PDIV into 0x1C register */
	miicmd = precmd | MII_WRITE << 28 | 0x1C << 18;
	miidata = 1 << 12 | ndiv;
	/* 0x53721040 */
	writel_relaxed(miicmd | miidata, ChipcommonB_MII_Management_Command_Data_VA);
	mdelay(10);

	/* Program other PLL parameters into 0x1D register, disable suspend and put PHY into reset */
	miicmd = precmd | MII_WRITE << 28 | 0x1D << 18;
	miidata = 1 << 13 | 3 << 8 | 3 << 4 | 0xa;
	/* 0x5376233a  */
	writel_relaxed(miicmd | miidata, ChipcommonB_MII_Management_Command_Data_VA);
	mdelay(10);

	/* Program register 0x15, USB device mode set and get PHY out of reset */
	miicmd = precmd | MII_WRITE << 28 | 0x15 << 18;
	miidata = 1 << 2 | 1 << 1;
	/* 0x53560006 */
	writel_relaxed(miicmd | miidata, ChipcommonB_MII_Management_Command_Data_VA);
	mdelay(10);

	/* Program register 0x19, set mdio mode */
	miicmd = precmd | MII_WRITE << 28 | 0x19 << 18;
	miidata = 1 << 7;
	/* 0x53660080 */
	writel_relaxed(miicmd | miidata, ChipcommonB_MII_Management_Command_Data_VA);
	mdelay(10);

	/* get the PLL out of reset */
	miicmd = precmd | MII_READ << 28 | 0x1D << 18;
	miidata = 0;
	writel_relaxed(miicmd | miidata, ChipcommonB_MII_Management_Command_Data_VA);
	mdelay(10);
	miidata = readl_relaxed(ChipcommonB_MII_Management_Command_Data_VA);
	miicmd = precmd | MII_WRITE << 28 | 0x1D << 18;
	miidata |= 1 << 12;
	/* 0x5376333a  */
	writel_relaxed(miicmd | miidata, ChipcommonB_MII_Management_Command_Data_VA);
	mdelay(10);
}
#elif defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) || defined(CONFIG_MACH_SB2)
static void config_usb_phy(void) 
{

	unsigned long val, mask;
	int count = 0;
	/* set phy_iso to 1, phy_iddq to 1, non_driving to 1 */
#if defined(CONFIG_MACH_SB2)
  
  val = readl_relaxed(USBH_Utmi_p0Ctl_VA);
  val &= ~0x00000800;     /* 11:dfe_powerup_fsm = 0 */
	writel_relaxed(val, USBH_Utmi_p0Ctl_VA);
	val |= 0x00000001;      /* 0:afe_non_driving = 1 */
	writel_relaxed(val, USBH_Utmi_p0Ctl_VA);
  
  val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= 0x0c000000;      /* 27:PHY_ISO & 26:PLL_SUSPEND_EN = 1 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
  val &= ~0x03000000;     /* 25:PLL_RESETB & 24:RESETB = 0 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
  
  val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
  val &= ~0x03000000;     /* 25:AFE_BG_PWRDWNB & 24:AFE_LDO_PWRDWNB = 0 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
  udelay(10);
  val |= 0x02000000;      /* 25:AFE_BG_PWRDWNB = 1 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
  udelay(150);
  val |= 0x01000000;      /* 24:AFE_LDO_PWRDWNB = 1 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
  udelay(160);
  
  val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~0x08000000;     /* 27:PHY_ISO = 0 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
	udelay(20);
  val |= 0x02000000;      /* 25:PLL_RESETB = 1 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
  
  mdelay(20);

	/* check pll_lock */
	mask = (1 << IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK);
	do {
		val = readl_relaxed(IPROC_WRAP_MISC_STATUS_VA);
		if ((val & mask) == mask)
			break;
		else
		{
			udelay(10);
			count ++;
		}
	} while(count <= 10);

	if (count > 10)
	{
		printk(KERN_WARNING "%s : PLL not lock! IPROC_WRAP_MISC_STATUS = 0x%08lx\n", 
					__FUNCTION__, val);
	}
  
  val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= 0x01000000;      /* 24:RESETB = 1 */
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
	udelay(2);
  
  val = readl_relaxed(USBH_Utmi_p0Ctl_VA);
	val &= ~0x00000001;     /* 0:afe_non_driving = 0 */
	writel_relaxed(val, USBH_Utmi_p0Ctl_VA);
  
#else  /* #if defined(CONFIG_MACH_GH) && defined(CONFIG_MACH_HR3) */

	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);

	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);

	/* set phy_resetb to 0, pll_resetb to 0 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	/* set p1ctl[11] to 0 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);

	/* set phy_iso to 0 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);

	/* set phy_iddq to 0 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	mdelay(1);

	/* set pll_resetb to 1, phy_resetb to 1 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	mdelay(20);

	/* check pll_lock */
	mask = (1 << IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK);
	do {
		val = readl_relaxed(IPROC_WRAP_MISC_STATUS_VA);
		if ((val & mask) == mask)
			break;
		else
		{
			udelay(10);
			count ++;
		}
	} while(count <= 10);

	if (count > 10)
	{
		printk(KERN_WARNING "%s : PLL not lock! IPROC_WRAP_MISC_STATUS = 0x%08lx\n", 
					__FUNCTION__, val);
	}

	/* set non_drving to 0 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);

	/* set p1ctl[11] to 1 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
#endif  /* #if defined(CONFIG_MACH_SB2) */

}
#endif


/*-------------------------------------------------------------------------*/

/* ==== Public Functions ================================================= */

/****************************************************************************
 * Module level definitions use to load / unload the UDC driver
 ***************************************************************************/

module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "Debug area bit mask");

static int __init bcm_udc_ModuleInit( void );
static void __exit bcm_udc_ModuleExit( void );

MODULE_DESCRIPTION( BCM_UDC_MODULE_DESCRIPTION );
MODULE_LICENSE( "GPL" );
MODULE_VERSION( BCM_UDC_MODULE_VERSION );

module_init (bcm_udc_ModuleInit);
module_exit (bcm_udc_ModuleExit);

static int __init bcm_udc_ModuleInit( void )
{
    int err;
	int i;

    BCM_KINFO( BCM_DRIVER_DESC " for Broadcom SOC\n" );
    #ifdef DEBUG
    BCM_KINFO( "debug=0x%x\n", debug );
    #endif

#ifdef CONFIG_ARCH_IPROC

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) || defined(CONFIG_MACH_GH) || \
	defined(CONFIG_MACH_HR3) || defined(CONFIG_MACH_SB2))
	volatile unsigned int regval, usbdgpiopwr;

	/* Check whether USB port is configured as Host/Device, return error if Host */
#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
	if (gpio_request(USBD_VBUS_GPIO, "usbd") == 0) {
		gpio_direction_input(USBD_VBUS_GPIO);
		if (__gpio_get_value(USBD_VBUS_GPIO) == 0) {
			BCM_KERROR( "USB VBUS input value is 0\n");
			return(-ENODEV);
		}	
	} else {
		return (-ENODEV);
	}
#elif defined(CONFIG_MACH_SB2)
    /* u-boot enable this bit to indicate usb in host mode */
    if (readl_relaxed(HW_IO_PHYS_TO_VIRT(IPROC_WRAP_IPROC_STRAP_CTRL)) & (1 << 10)) {
		return(-ENODEV);
    }
#else
	/* Configure GPIO0 as input and read the GPIO0 status */
	regval = readl_relaxed(ChipcommonA_GPIOOutEn_VA);
#if defined(CONFIG_MACH_HX4)
	regval &= ~(0x01);
#elif defined(CONFIG_MACH_KT2)
	regval &= ~(1 << 4);
#endif

	writel_relaxed(regval, ChipcommonA_GPIOOutEn_VA);

#if defined(CONFIG_MACH_HX4)

	if((readl_relaxed(ChipcommonA_GPIOInput_VA) & 0x01) == 0) {
#elif defined(CONFIG_MACH_KT2)
	if((readl_relaxed(ChipcommonA_GPIOInput_VA) & (1 << 4)) == 0) {
#endif
		BCM_KERROR( "USB port is configured for Host mode\n");
		return(-ENODEV);
	}

	/* Do not drive USB VBUS power */
	usbdgpiopwr = readl_relaxed(ChipcommonA_GPIOOut_VA);
	regval = readl_relaxed(ChipcommonA_GPIOOutEn_VA);
#if defined(CONFIG_MACH_HX4)
	usbdgpiopwr &= ~0x2;
	regval |= (1 << 1); /* GPIO 1 */
#elif defined(CONFIG_MACH_KT2)
	usbdgpiopwr &= ~0x1;
	regval |= (1 << 0); /* GPIO 0 */
#endif
	writel_relaxed(usbdgpiopwr, ChipcommonA_GPIOOut_VA);
	writel_relaxed(regval, ChipcommonA_GPIOOutEn_VA);
#endif
	/* Put USBD controller into reset state and disable clock via IDM registers */

#if defined(CONFIG_MACH_SB2)
  
  /* Enable clock to USBD */
	regval = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	regval |= (1 << USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel_relaxed(regval, USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	mdelay(10);
	
	/* Get USBD out of reset  */
	regval = readl_relaxed(USB2D_IDM_IDM_RESET_CONTROL_VA);
	regval &= ~(1 << USB2D_IDM_IDM_RESET_CONTROL__RESET);
	writel_relaxed(regval, USB2D_IDM_IDM_RESET_CONTROL_VA);
	mdelay(100);
	
	/* Configure USB PHY and PHY PLL to drive 60MHz USB clock*/
	config_usb_phy();
	
	/* Dev configuration */
	regval = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	regval |= 0x07ff0000;
	writel_relaxed(regval, USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	mdelay(10);
	regval &= ~0x07ff0000;
	writel_relaxed(regval, USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	mdelay(10);
	
	/* AXI related configuration */
	regval = readl_relaxed(ICFG_USB2D_CONFIG_0_VA);
	regval &= ~0x1f1f0fff;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_0_VA);
	mdelay(10);
	
	regval = readl_relaxed(ICFG_USB2D_CONFIG_0_VA);
	regval |= 0x0f0f0fff;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_0_VA);
	regval = readl_relaxed(ICFG_USB2D_CONFIG_0_VA);
	regval &= ~0x10100000;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_0_VA);
	mdelay(10);
	
	regval = readl_relaxed(ICFG_USB2D_CONFIG_0_VA);
	regval &= ~0x00000fff;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_0_VA);
	mdelay(10);
	
	regval = readl_relaxed(ICFG_USB2D_CONFIG_1_VA);
	regval |= 0x00000fff;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_1_VA);
	mdelay(10);
	
	regval = readl_relaxed(ICFG_USB2D_CONFIG_1_VA);
	regval &= ~0x00000fff;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_1_VA);
	mdelay(10);
	
	regval = readl_relaxed(ICFG_USB2D_CONFIG_2_VA);
	regval |= 0x00000fff;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_2_VA);
	mdelay(10);
	
	regval = readl_relaxed(ICFG_USB2D_CONFIG_2_VA);
	regval &= ~0x00000fff;
	writel_relaxed(regval, ICFG_USB2D_CONFIG_2_VA);
	mdelay(10);
	
	/* Dev configuration */
	regval = readl_relaxed(USB2D_DEVCONFIG_VA);
	regval |= 0x00040028;     /* 18:SET_DESC & 5:PI & 3:SP = 1 */
	writel_relaxed(regval, USB2D_DEVCONFIG_VA);
	regval &= ~0x00000040;    /* 6:DIR = 0 */
	writel_relaxed(regval, USB2D_DEVCONFIG_VA);
	regval &= ~0x00000011;    /* 0-1:SPD = 00 */
	writel_relaxed(regval, USB2D_DEVCONFIG_VA);
	mdelay(10);
	
	/* Dev control */
	regval = readl_relaxed(USB2D_DEVCTRL_VA);
	regval |= 0x00000350;     /* 4:DU & 6:BF & 8:BREN & 9:MODE = 1 */
	writel_relaxed(regval, USB2D_DEVCTRL_VA);
	//regval |= 0x00002000;   /* 13:CSR_DONE = 1 */
	//writel_relaxed(regval, USB2D_DEVCTRL_VA);
	regval |= 0x0a0a0000;     /* 16-23:BRLEN & 24-31:THLEN = random(3,15), give 10 */
	writel_relaxed(regval, USB2D_DEVCTRL_VA);
	
#else
	regval = readl_relaxed(USB2D_IDM_IDM_RESET_CONTROL_VA);
	regval |= (1 << USB2D_IDM_IDM_RESET_CONTROL__RESET);
	writel_relaxed(regval, USB2D_IDM_IDM_RESET_CONTROL_VA);

	regval = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	regval &= ~(1 << USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel_relaxed(regval, USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);

	/* Configure USB PHY and PHY PLL to drive 60MHz USB clock*/
	config_usb_phy();

	/* Enable clock to USBD and get the USBD out of reset  */
	regval = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	regval |= (1 << USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel_relaxed(regval, USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	regval = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	BCM_DEBUG_DEV("USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA: addr: 0x%08x, val: 0x%08x\n", USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA, regval);

	mdelay(10);
	regval = readl_relaxed(USB2D_IDM_IDM_RESET_CONTROL_VA);
	BCM_DEBUG_DEV("USB2D_IDM_IDM_RESET_CONTROL_VA: addr: 0x%08x, val: 0x%08x\n", USB2D_IDM_IDM_RESET_CONTROL_VA, regval);
	regval &= ~(1 << USB2D_IDM_IDM_RESET_CONTROL__RESET);
	writel_relaxed(regval, USB2D_IDM_IDM_RESET_CONTROL_VA);
	mdelay(100);
	regval = readl_relaxed(USB2D_IDM_IDM_RESET_CONTROL_VA);
	BCM_DEBUG_DEV("USB2D_IDM_IDM_RESET_CONTROL_VA: addr: 0x%08x, val: 0x%08x\n", USB2D_IDM_IDM_RESET_CONTROL_VA, regval);
#endif

	BCM_DEBUG_DEV("usbDevHw_REG_P: addr: 0x%08x, val: 0x%08x\n", usbDevHw_REG_P, usbDevHw_REG_P->devCfg);

	usbDevHw_DeviceIrqDisable(usbDevHw_DEVICE_IRQ_ALL);
	usbDevHw_DeviceIrqClear(usbDevHw_DEVICE_IRQ_ALL);
#endif

    err = platform_driver_register( &bcm_udc_PlatformDriver );
    if (err)
    {
        BCM_KERROR( "platform_driver_register failed, err=%d\n", err );
    }
    else
    {
        err = platform_device_register( &iproc_udc_pdev );
        if (err)
        {
            BCM_KERROR( "platform_device_register failed, err=%d\n", err );
            platform_driver_unregister( &bcm_udc_PlatformDriver );
        }
    }
#else
    err = platform_driver_register( &bcm_udc_PlatformDriver );
#endif
    return( err );
}

static void __exit bcm_udc_ModuleExit( void )
{
#ifdef CONFIG_ARCH_IPROC
	volatile unsigned int regval;
    platform_device_unregister( &iproc_udc_pdev );
#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
	gpio_free(USBD_VBUS_GPIO);
#endif	

	/* Put USBD controller into reset state and disable clock via IDM registers */
	regval = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);
	regval &= ~(1 << USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
	writel_relaxed(regval, USB2D_IDM_IDM_IO_CONTROL_DIRECT_VA);

	regval = readl_relaxed(USB2D_IDM_IDM_RESET_CONTROL_VA);
	regval |= (1 << USB2D_IDM_IDM_RESET_CONTROL__RESET);
	writel_relaxed(regval, USB2D_IDM_IDM_RESET_CONTROL_VA);
#endif
    platform_driver_unregister( &bcm_udc_PlatformDriver );
}

/****************************************************************************
 * APIs used by a Gadget driver to attach / detach from the UDC driver.
 ***************************************************************************/
int bcm_udc_start( struct usb_gadget_driver *gadget_driver,
		int (*bind)(struct usb_gadget *))
{
    unsigned long flags;
    int err;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    if ( bcmUdcP == NULL )
    {
        BCM_KERROR( "UDC driver not initialized\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -ENODEV );
    }

    if ( !bind || !gadget_driver || !gadget_driver->setup || (gadget_driver->max_speed < USB_SPEED_FULL) )
    {
        BCM_KERROR( "invalid gadget driver\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -EINVAL );
    }

    spin_lock_irqsave( &bcmUdcP->lock, flags );

    /** @todo somehow get rid of bcmUdcP global variable? */
    if ( bcmUdcP->gadget_driver )
    {
        spin_unlock_irqrestore( &bcmUdcP->lock, flags );
        BCM_KNOTICE( "UDC driver busy\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -EBUSY );
    }

    /* Hook up the gadget driver to the UDC controller driver */

    gadget_driver->driver.bus = NULL;
    bcmUdcP->gadget_driver = gadget_driver;
    bcmUdcP->gadget.dev.driver = &gadget_driver->driver;
    bcmUdcP->pullupOn = 1;

    spin_unlock_irqrestore( &bcmUdcP->lock, flags  );


	/* Bind the driver */
	if ((err = device_add(&bcmUdcP->gadget.dev)) != 0) {
		BCM_KERROR("Error in device_add() : %d\n",err);
		return(err);
	}

	BCM_DEBUG_DEV("binding gadget driver '%s'\n",
		gadget_driver->driver.name);


    err = bind( &bcmUdcP->gadget );

    if ( err)
    {
		device_del(&bcmUdcP->gadget.dev);
        BCM_KERROR( "%s gadget_driver->bind() failed, err=%d\n", gadget_driver->driver.name, err );
        bcmUdcP->gadget_driver = NULL;
    }
    else
    {
        BCM_KINFO( "%s gadget_driver->bind() done\n", gadget_driver->driver.name );

        // create wake lock for handling suspend/resume case
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK) && \
	defined(CONFIG_USB_WAKESOURCE_BCMRING_UDC)
        wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, wake_lock_name);
#endif


        /** @todo Move the BUS connect/disconnect to CltrStartup/Shutdown?? */
        spin_lock_irqsave( &bcmUdcP->lock, flags );
        UdcOpsStartup( bcmUdcP );
#ifdef CONFIG_ARCH_IPROC
        /* un-stop the control endpoint */
        bcmUdcP->ep[0].stopped = 0;
#endif
        if ( bcmUdcP->pullupOn )
        {
            usbDevHw_DeviceBusConnect();
        }
	usbDevHw_DeviceSetupDone();
	usbDevHw_DeviceDmaEnable();
        spin_unlock_irqrestore( &bcmUdcP->lock, flags );
    }

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( err );
}

static void UdcOpsShutdownDev(void)
{ 
    BCM_UDC_EP_t *udcEpP;
    unsigned long flags;

    spin_lock_irqsave( &bcmUdcP->lock, flags );
	
    bcmUdcP->ep[0].desc = NULL;
    list_for_each_entry( udcEpP, &bcmUdcP->gadget.ep_list, usb_ep.ep_list )
    {
        udcEpP->desc = NULL;
    }
    bcmUdcP->gadget.dev.driver = NULL;
    bcmUdcP->gadget_driver = NULL;
	
    spin_unlock_irqrestore( &bcmUdcP->lock, flags );
}

int bcm_udc_stop( struct usb_gadget_driver *gadget_driver )
{
    unsigned long flags;

    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /** @todo somehow get rid of bcmUdcP global variable? */
    if ( bcmUdcP == NULL )
    {
        BCM_KERROR( "UDC driver not initialized\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -ENODEV );
    }

    if ( !gadget_driver || (gadget_driver != bcmUdcP->gadget_driver) || !gadget_driver->unbind )
    {
        BCM_KERROR( "invalid gadget driver\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EINVAL );
    }

    /** @todo Is this delay needed? If so, state why. */
    usbDevHw_DeviceBusDisconnect();
    bcmUdcP->pullupOn = 0;
    udelay(20);

    spin_lock_irqsave( &bcmUdcP->lock, flags );
    UdcOpsShutdown( bcmUdcP );

    spin_unlock_irqrestore( &bcmUdcP->lock, flags );

    BCM_DEBUG_DEV( "%s gadget_driver->disconnect()\n", gadget_driver->driver.name );
    bcmUdcP->gadget_driver->disconnect( &bcmUdcP->gadget );

    BCM_DEBUG_DEV( "%s gadget_driver->unbind()\n", gadget_driver->driver.name );
    gadget_driver->unbind( &bcmUdcP->gadget );

	device_del(&bcmUdcP->gadget.dev);

    UdcOpsShutdownDev();

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK) && \
	defined(CONFIG_USB_WAKESOURCE_BCMRING_UDC)
    wake_lock_destroy(&wakelock);
#endif

    BCM_KINFO( "%s gadget_driver->unbind() done\n", gadget_driver->driver.name );
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

#ifdef CONFIG_ARCH_BCMRING
static irqreturn_t udc_vbus_irq(int irq, void *_dev)
{
    int is_active;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK)
    int timeout;
#endif
    BCM_UDC_t *udcP = _dev;
    (void)irq;

    /* debounce */
    udelay(20);

    is_active = ( gpio_get_value(gVbusDetectGpio) != 0 );

    // We need to maintain a wake lock for few seconds so allowing the system to wake up and
    // let the mount service to dismount or mount the device
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)) && defined(CONFIG_HAS_WAKELOCK) && \
	defined(CONFIG_USB_WAKESOURCE_BCMRING_UDC)
    if (is_active)
        timeout = WAKE_LOCK_ACTIVE_TIMEOUT_DURATION;
    else
        timeout = WAKE_LOCK_INACTIVE_TIMEOUT_DURATION;
    BCM_DEBUG_IRQ("USB gadget irq wake lock for %d sec\n", timeout/HZ);
    wake_lock_timeout(&wakelock, timeout);
#endif


    BCM_DEBUG_IRQ( "is_active=%d\n", is_active );

    GadgetDevVbusSession( &udcP->gadget, is_active );

    return IRQ_HANDLED;
}
#endif

/****************************************************************************
 * APIs used to bind / unbind UDC devices to the UDC driver.
 ***************************************************************************/

int PlatformDriverProbe( struct platform_device *platformDevP )
{
    int err;
#ifdef CONFIG_ARCH_BCMRING
    struct usb_udc_cfg *cfg;
#endif

    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

#ifdef CONFIG_ARCH_BCMRING
    err = bcmring_usb_init(1, 1);
    if (err)
       return err;

    cfg = platformDevP->dev.platform_data;
    if (cfg == NULL)
        gVbusDetectGpio = -1;
    else
        gVbusDetectGpio = cfg->vbus_detect_gpio;
    BCM_DEBUG_TRACE(KERN_INFO "USB UDC Vbus detect gpio = %d\n", gVbusDetectGpio);
#endif

    /** @todo somehow get rid of bcmUdcP global variable? e.g. reference in platformDevP? */
    if ( bcmUdcP != NULL )
    {
        BCM_KERROR( "device already attached\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EBUSY );
    }

    if ( (bcmUdcP = kmalloc( sizeof(*bcmUdcP), GFP_KERNEL)) == NULL )
    {
        BCM_KERROR( "kmalloc() failed\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -ENOMEM );
    }

    /** @todo put these in UdcOpsInit()?? */
    memset( bcmUdcP, 0, sizeof(*bcmUdcP) );
    spin_lock_init( &bcmUdcP->lock );

    /** @todo Have to make sure all interrupts are off at the lower layers before requesting (enabling) at the top level. */
    if ( (err = request_irq(BCM_UDC_IRQ, IrqUdc, 0, BCM_UDC_NAME, (void *)bcmUdcP)) < 0 )
    {
        BCM_KERROR( "request_irq() failed\n" );
        goto err1;
    }

    if ( (err = PlatformDmaAlloc(platformDevP, bcmUdcP)) < 0 )
    {
        BCM_KERROR( "PlatformDmaAlloc() failed\n" );
        goto err2;
    }

    /** @todo remove UdcOpsInit() ?? or is this called something else?? */
    UdcOpsInit( bcmUdcP );
	if (err = GadgetDevInit( platformDevP, bcmUdcP ) < 0 )
    {
        BCM_KERROR( "GadgetDevInit() failed\n" );
        goto err3;
    }
#ifdef CONFIG_ARCH_BCMRING
    if (gVbusDetectGpio >= 0) {
    /* request VBUS GPIO control */
    if ( gpiomux_request( gVbusDetectGpio, chipcHw_GPIO_FUNCTION_GPIO, "udc_vbus_detect" ) == gpiomux_rc_SUCCESS )
    {
        /* Set the current VBUS state. We will not get an interrupt for this,
         * only changes in the state.
         */
        BCM_KINFO( "GPIO Request gVbusDetectGpio (%d) OK\n", gVbusDetectGpio );
        GadgetDevVbusSession( &bcmUdcP->gadget, (gpio_get_value(gVbusDetectGpio) != 0) );

        if ( !request_irq( gpio_to_irq(gVbusDetectGpio),
                           udc_vbus_irq,
                           IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                           BCM_UDC_NAME,
                           bcmUdcP ) )
        {
            BCM_KINFO( "VBUS Detection IRQ installed OK\n" );
        }
        else
        {
            gpiomux_free( gVbusDetectGpio );
            /* Just log errors for now */
            BCM_KERROR( "VBUS detection IRQ installation failed\n" );
            }
        }
       else
       {
            /* Just log errors for now */
            BCM_KERROR( "gpio_request( gVbusDetectGpio ) failed\n" );
        }
    }
#endif

    ProcFileCreate();

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );

err3:
    PlatformDmaFree( platformDevP, bcmUdcP );
err2:
    free_irq( BCM_UDC_IRQ, bcmUdcP );
err1:
    kfree( bcmUdcP );
    bcmUdcP = NULL;

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( err );
}

int PlatformDriverRemove( struct platform_device *platformDevP )
{
    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    if ( !bcmUdcP )
    {
        return( -ENODEV );
    }

    ProcFileRemove();
    GadgetDevRemove( bcmUdcP );
	platform_set_drvdata(platformDevP, NULL);
    UdcOpsFinis( bcmUdcP );

    PlatformDmaFree( platformDevP, bcmUdcP );
    free_irq( BCM_UDC_IRQ, bcmUdcP );

#ifdef CONFIG_ARCH_BCMRING
    bcmring_usb_term(1);
    if (gVbusDetectGpio >= 0)
    {
        free_irq( gpio_to_irq(gVbusDetectGpio), bcmUdcP );
        gpiomux_free( gVbusDetectGpio );
    }
#endif

    kfree( bcmUdcP );
    bcmUdcP = NULL;

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

#ifdef CONFIG_PM_USBD
static int PlatformDriverSuspend(struct platform_device *dev, pm_message_t state)
{    
//#ifdef CONFIG_ARCH_IPROC
    int rc = 0;
    rc = UdcOpsSuspend( bcmUdcP );
    if (rc)
        return rc;    
    return 0;
}

static int PlatformDriverResume(struct platform_device *dev)
{    
//#ifdef CONFIG_ARCH_IPROC
    int rc;
    rc = UdcOpsResume( bcmUdcP );
    if (rc)
        return rc;
    return 0;
}
#endif

/****************************************************************************
 *
 * Platform device level alloc / free of memory used for DMA descriptors.
 * A single block of memory static in size is used for DMA descriptors.
 * Each endpoint has a small number of descriptors for its exclusive use.
 * These are chained in a loop. See bcm_udc_dwc.h and DmaEpInit() for more
 * details.
 *
 ***************************************************************************/

int PlatformDmaAlloc( struct platform_device *platformDevP, BCM_UDC_t *udcP )
{
    udcP->dma.virtualAddr = dma_alloc_coherent( &platformDevP->dev, sizeof(BCM_UDC_DMA_t),
                                                (dma_addr_t *)&udcP->dma.physicalAddr, GFP_KERNEL );

    if ( !udcP->dma.virtualAddr )
    {
        BCM_KERROR("dma_alloc_coherent() failed\n");
        return( -ENOMEM );
    }

    return( ENOERROR );
}

void PlatformDmaFree( struct platform_device *platformDevP, BCM_UDC_t *udcP )
{
    unsigned num;


    dma_free_coherent( &platformDevP->dev, sizeof(BCM_UDC_DMA_t), udcP->dma.virtualAddr,
                        (dma_addr_t)udcP->dma.physicalAddr );

    for ( num = 0; num < BCM_UDC_EP_CNT; num ++ )
    {
        if ( udcP->ep[num].dma.alignedBuf )
        {
            dma_free_coherent( NULL, udcP->ep[num].dma.alignedLen, udcP->ep[num].dma.alignedBuf, udcP->ep[num].dma.alignedAddr );
            udcP->ep[num].dma.alignedBuf = NULL;
        }
    }
}

/****************************************************************************
 * Linux Gadget device operations.
 ***************************************************************************/
int GadgetDevInit( struct platform_device *platformDevP, BCM_UDC_t *udcP )
{
    int err;
	unsigned num;

    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
    udcP->gadget.name = BCM_UDC_NAME;
    udcP->gadget.speed = USB_SPEED_UNKNOWN;
	udcP->gadget.max_speed = USB_SPEED_HIGH;
    udcP->gadget.ops = &bcm_udc_gadgetDevOps;

    udcP->gadget.ep0 = &udcP->ep[0].usb_ep;
    INIT_LIST_HEAD(&udcP->gadget.ep_list);
    for ( num = 1; num < BCM_UDC_EP_CNT; num ++ )
    {
        list_add_tail( &udcP->ep[num].usb_ep.ep_list, &udcP->gadget.ep_list );
    }

	device_initialize(&udcP->gadget.dev);
    dev_set_name(&(udcP->gadget.dev), "gadget");
	udcP->gadget.dev.parent = &platformDevP->dev;
	udcP->gadget.dev.dma_mask = platformDevP->dev.dma_mask;

	platform_set_drvdata(platformDevP, udcP);


	err = usb_add_gadget_udc(&platformDevP->dev, &udcP->gadget);
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
	return(err);
}

void GadgetDevRelease( struct device *dev )
{
    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /** @todo Is this required here?? Anything else need to be done?? */
    /* Shutdown the hardware operations */
    usbDevHw_OpsFinis();

    /** @todo remove global variable reference somehow? */
    complete( bcmUdcP->devRelease );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

int GadgetDevRemove( BCM_UDC_t *udcP )
{
    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

	usb_del_gadget_udc(&udcP->gadget);
	if (udcP->gadget_driver)
		return -EBUSY;

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

#ifdef CONFIG_ARCH_BCMRING
static int GadgetDevWakeup( struct usb_gadget *gadget )
{
    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
    if ( gadget == NULL )
    {
        BCM_KERROR( "DevWakeup: gadget NULL\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -EINVAL );
    }
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return 0;
}

static int GadgetDevSetSelfpowered( struct usb_gadget *gadget, int is_selfpowered )
{
    BCM_UDC_t      *udcP;
    unsigned long  flags;

    BCM_DEBUG_TRACE( "enter (%d)\n", is_selfpowered );

    if ( gadget == NULL )
    {
        BCM_KERROR( "DevSetSelfpowered: gadget NULL\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -EINVAL );
    }

    udcP = container_of( gadget, BCM_UDC_t, gadget );

    spin_lock_irqsave( &udcP->lock, flags );

    if ( is_selfpowered )
    {
        usbDevHw_DeviceSelfPwrEnable();
    }
    else
    {
        usbDevHw_DeviceSelfPwrDisable();
    }

    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

static int GadgetDevVbusSession( struct usb_gadget *gadget, int is_active )
{
    BCM_UDC_t      *udcP;
    unsigned long  flags;
    int            previously_active;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
    BCM_DEBUG_DEV( "is_active=%d\n", is_active );

    if ( gadget == NULL )
    {
        BCM_KERROR( "DevVbusSession: gadget NULL\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -EINVAL );
    }

    udcP = container_of( gadget, BCM_UDC_t, gadget );

    spin_lock_irqsave( &udcP->lock, flags );

    /** @todo For some reason will see a few interrupts when a cable is removed
     * and sometimes when the cable is inserted. This occurs even though some
     * debouncing is done. All the interrupts will have the same state though.
     * E.g. when the cable is removed will see several interrupts indicating
     * "not is_active", but none that indicate "is_active". To avoid excess
     * processing, etc. will only handle the case where there is a change in
     * state detected.
     */
    previously_active = udcP->vbusActive;
    udcP->vbusActive = is_active;

    if ( is_active && !previously_active )
    {
        BCM_KINFO( "VBUS connect\n" );
        /*
         * Set the desired link speed before enabling the USB connection. Also
         * only enable the USB connection if there's a gadget driver already attached.
         */
        /** @todo Set the requested speed based on a module parameter */
        usbDevHw_DeviceSpeedRequested( usbDevHw_DEVICE_SPEED_HIGH );

         /* Connect only if the application wants to connect. This is indicated
         * by the pullupOn flag. */
        if ( udcP->gadget_driver && udcP->pullupOn )
        {
            usbDevHw_DeviceBusConnect();
        }
    }
    else if ( !is_active && previously_active )
    {
        BCM_KINFO( "VBUS disconnect\n" );
        usbDevHw_DeviceBusDisconnect();
        usbDevHw_DeviceSpeedRequested( usbDevHw_DEVICE_SPEED_FULL );
        udcP->gadget.speed = USB_SPEED_UNKNOWN;

        if ( udcP->gadget_driver && udcP->gadget_driver->disconnect )
        {
            BCM_DEBUG_DEV( "%s gadget_driver->disconnect()\n", udcP->gadget_driver->driver.name );

            spin_unlock(&udcP->lock);
            udcP->gadget_driver->disconnect( &udcP->gadget );
            spin_lock( &udcP->lock );
        }
    }

    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

/**
 * GadgetDevPullup - Given the connection state, performs
 * a soft (dis)connect.
 * @gadget: the peripheral being (dis)connected
 * @is_on: indicates whether the peripheral needs to be
 * (dis)connected
 *
 * Returns ENOERROR on success and a negative value
 * upon failure.
 */
int GadgetDevPullup( struct usb_gadget *gadget, int is_on )
{
    BCM_UDC_t      *udcP;
    unsigned long  flags;

    BCM_DEBUG_TRACE( "%s enter (%d)\n", __func__, is_on );

    if ( gadget == NULL )
    {
        BCM_KERROR( "DevPullup: gadget NULL\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -EINVAL );
    }

    udcP = container_of( gadget, BCM_UDC_t, gadget );

    spin_lock_irqsave( &udcP->lock, flags );

    udcP->pullupOn = is_on;
    if ( is_on && udcP->vbusActive )
    {
        BCM_DEBUG_TRACE( "Connecting gadget\n" );
        usbDevHw_DeviceBusConnect();
    }
    else if ( !is_on )
    {
        BCM_DEBUG_TRACE( "Disconnecting gadget\n" );
        usbDevHw_DeviceBusDisconnect();
    }
    else
    {   /* nothing to do. Simply print a message */
        BCM_DEBUG_TRACE( "DevPullup: gadget in unexpected connected state vbusActive:%d, is_on:%d\n",
                     udcP->vbusActive, is_on );
    }

    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}
#endif

/****************************************************************************
 * Linux Gadget endpoint operations. See usb_ep_ops in usb_gadget.h.
 ***************************************************************************/

int GadgetEpEnable( struct usb_ep *usb_ep, const struct usb_endpoint_descriptor *desc )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_t *udcP;
    unsigned long flags;
    unsigned maxPktSize;
    unsigned xferType;



    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );


    if ( !usb_ep || (udcEpP->bEndpointAddress != desc->bEndpointAddress) )
    {
        BCM_KERROR( "invalid endpoint (%p)\n", usb_ep );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EINVAL );
    }

    if ( !desc || (desc->bDescriptorType != USB_DT_ENDPOINT) )
    {
        BCM_KERROR( "ep%d: invalid descriptor=%p type=%d\n", udcEpP->num, desc, desc ? desc->bDescriptorType : -1 );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EINVAL );
    }

    if ( desc == udcEpP->desc )
    {
        BCM_KWARN( "ep%d: already enabled with same descriptor\n", udcEpP->num );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EEXIST );
    }

    if ( udcEpP->desc )
    {
        BCM_KWARN( "ep%d: already enabled with another descriptor\n", udcEpP->num );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EBUSY );
    }

    udcP = udcEpP->udcP;

    if ( !udcP->gadget_driver
#ifndef CONFIG_ARCH_IPROC
         || (udcP->gadget.speed == USB_SPEED_UNKNOWN)
#endif
        )
    {
        BCM_KWARN( "%s: invalid device state\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -ESHUTDOWN );
    }

    xferType = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
    maxPktSize = le16_to_cpu (desc->wMaxPacketSize) & 0x7FF;

    if ( !maxPktSize || (maxPktSize > udcEpP->maxPktSize) )
    {
        BCM_KERROR( "%s: invalid max pkt size: ep=%d desc=%d\n", udcEpP->usb_ep.name, udcEpP->maxPktSize, maxPktSize );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -ERANGE );
    }

    if ( (udcEpP->dirn == USB_DIR_IN) && (xferType == USB_ENDPOINT_XFER_ISOC) )
    {
        if ((desc->bInterval < 1) || (desc->bInterval > 16))
        {
            BCM_KERROR( "%s: invalid ISOC bInterval=%u\n", udcEpP->usb_ep.name, desc->bInterval );
            BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
            return( -ERANGE );
        }
        /*
         * We don't know when the host will send the first ISO IN request, so we need to set up
         * to capture that event so we can align subsequent transfers to that particular frame
         * number. Also set the frame number increment. The endpoint descriptor specifies this
         * as a power of 2 (2**(n-1)). Translate this into a specific number of frames.
         */
        udcEpP->dma.frameNum = FRAME_NUM_INVALID;
        udcEpP->dma.frameIncr = 1 << (desc->bInterval - 1);
        BCM_DEBUG_ISOC("%s: frameIncr=%d\n", udcEpP->usb_ep.name, udcEpP->dma.frameIncr );
    }

    spin_lock_irqsave( &udcP->lock, flags );

    if ( UdcEpCfg( udcEpP, xferType, maxPktSize ) != ENOERROR )
    {
        spin_unlock_irqrestore( &udcP->lock, flags );
        BCM_KERROR( "%s: not enough FIFO space\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -ENOSPC );
    }

    /** @todo Rework the UdcEpCfg() so it includes usbDevHw_EndptCfgSet() ... */
    usbDevHw_EndptCfgSet( udcEpP->num, usbDevHw_DeviceCfgNum() );

    udcEpP->desc = desc;
    udcEpP->stopped = 0;
    udcEpP->usb_ep.maxpacket = maxPktSize;

    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_DEBUG_EP( "%s: enabled: type=0x%x, maxPktSize=%d\n", udcEpP->usb_ep.name, xferType, maxPktSize );
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

int GadgetEpDisable( struct usb_ep *usb_ep )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_t *udcP;
    unsigned long flags;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    if ( !usb_ep )
    {
        BCM_KERROR( "invalid endpoint\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EINVAL );
    }

    if ( !udcEpP->desc )
    {
        BCM_DEBUG_EP( "%s: already disabled\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( ENOERROR );
    }

    udcP = udcEpP->udcP;

    /** @todo Really need to do this around udcEpP->desc check */
    spin_lock_irqsave(&udcP->lock, flags);

    ReqQueueFlush( udcEpP, -ESHUTDOWN );

#ifndef CONFIG_ARCH_IPROC
    usbDevHw_EndptIrqDisable( udcEpP->num, udcEpP->dirn );
#endif
    udcEpP->desc = NULL;
    udcEpP->usb_ep.maxpacket = udcEpP->maxPktSize;
    UdcFifoRamFree( udcEpP );

    spin_unlock_irqrestore(&udcP->lock, flags);

    BCM_DEBUG_EP("%s: disabled\n", udcEpP->usb_ep.name );
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

struct usb_request * GadgetEpRequestAlloc( struct usb_ep *usb_ep, unsigned gfp_flags )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

	if (!usb_ep)
		return NULL;

    if ( (udcEpReqP = kzalloc( sizeof(*udcEpReqP), gfp_flags )) != NULL )
    {
        /*
         * Set the usb_req.dma to DMA_ADDR_INVALID so it can be determined if the usb_req.buf needs
         * to be mapped when the request is subsequently queued.
         */
        INIT_LIST_HEAD( &udcEpReqP->listNode );
        udcEpReqP->usb_req.dma = DMA_ADDR_INVALID;

        BCM_DEBUG_MEM( "%s: req=0x%p flags=0x%x\n", usb_ep->name, udcEpReqP, gfp_flags );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( &udcEpReqP->usb_req );
    }

    BCM_KERROR( "kmalloc() failed\n" );
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( NULL );
}


void  GadgetEpRequestFree( struct usb_ep *usb_ep, struct usb_request *usb_req )
{
    BCM_UDC_EP_REQ_t *udcEpReqP = container_of(usb_req, BCM_UDC_EP_REQ_t, usb_req);


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /** @todo Use usb_ep for sanity check?? */
    (void)usb_ep;

    if ( usb_req )
    {
        BCM_DEBUG_MEM( "%s: req=0x%p\n", usb_ep->name, udcEpReqP );
        kfree( udcEpReqP );
    }

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

int GadgetEpRequestQueue( struct usb_ep *usb_ep, struct usb_request *usb_req, unsigned gfp_flags )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_EP_REQ_t *udcEpReqP = container_of(usb_req, BCM_UDC_EP_REQ_t, usb_req);
    BCM_UDC_t *udcP;
    unsigned long flags;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    if ( !usb_ep || !usb_req || !udcEpReqP->usb_req.complete || !udcEpReqP->usb_req.buf || !list_empty(&udcEpReqP->listNode) )
    {
        BCM_KERROR( "invalid request\n" );
        BCM_DEBUG_REQ( "usb_ep=0x%p udc_req=0x%p usb_req=0x%p usb_req.complete=0x%p usb_req.buf=0x%p\n",
                        usb_ep, udcEpReqP, usb_req, udcEpReqP->usb_req.complete, udcEpReqP->usb_req.buf );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EINVAL );
    }

    if ( !udcEpP->desc && (udcEpP->num != 0) )
    {
        BCM_KERROR( "%s: invalid EP state\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EFAULT );
    }

    if ( (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) && !list_empty(&udcEpP->listQueue) )
    {
        BCM_KERROR( "%s: CTRL EP queue not empty\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EPERM );
    }

    if ( usb_req->length > CONFIG_USB_GADGET_STORAGE_BUFLEN )
    {
        /** @todo DMA descriptors have a 16-bit length field. Only really applicable if doing
         * buffer fill mode. Cannot really do this for OUT transfers (too many issues with
         * validating buffer sizes), and things seem to be broken for IN transfers.
         */
        BCM_KERROR( "%s: request too big, length=%u\n", udcEpP->usb_ep.name, usb_req->length );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -E2BIG );
    }

    /*
     * Restrict ISOC IN requests to the max packet size. Assumption is that it does not make
     * much sense to have more than one interval's (scheduled bandwidth's) worth of data.
     */
    /** @todo Support for multiple packets per frame (high speed/bandwidth). These could have up to 3
     * packets of max length per uframe.
     */
    if ( (udcEpP->type == USB_ENDPOINT_XFER_ISOC) && (udcEpP->dirn == USB_DIR_IN) && (usb_req->length > udcEpP->usb_ep.maxpacket) )
    {
        BCM_KERROR( "%s: request > scheduled bandwidth, length=%u\n", udcEpP->usb_ep.name, usb_req->length );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EFBIG );
    }

    udcP = udcEpP->udcP;

    if ( !udcP->gadget_driver
#ifndef CONFIG_ARCH_IPROC
         || (udcP->gadget.speed == USB_SPEED_UNKNOWN)
#endif
        )
    {
        BCM_KWARN( "%s: invalid device state\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -ESHUTDOWN );
    }

    if ( ((unsigned long)udcEpReqP->usb_req.buf) & 0x3UL )
    {
        BCM_DEBUG_MEM( "%s: invalid buffer alignment: addr=0x%p\n", udcEpP->usb_ep.name, udcEpReqP->usb_req.buf );

        /*
         * The DMA buffer does not have the alignment required by the hardware. We keep an endpoint level
         * buffer available to handle this situation if it arises. If we don't currently have one available
         * for this purpose, or if the current one is not large enough, then allocate a new one. Since
         * we only have one buffer, we won't copy into the buffer until we are ready to do the DMA transfer.
         * Mark the request as needing this alignment (copy).
         */
        if ( (udcEpP->dma.alignedBuf != NULL) && (udcEpP->dma.alignedLen < udcEpReqP->usb_req.length) )
        {
            BCM_DEBUG_MEM( "%s: dma_free_coherent(): addr=0x%x length=%u\n", udcEpP->usb_ep.name, udcEpP->dma.alignedAddr, udcEpP->dma.alignedLen );
            dma_free_coherent( NULL, udcEpP->dma.alignedLen, udcEpP->dma.alignedBuf, udcEpP->dma.alignedAddr );
            udcEpP->dma.alignedBuf = NULL;
        }

        if ( udcEpP->dma.alignedBuf == NULL )
        {
            udcEpP->dma.alignedLen = udcEpReqP->usb_req.length;
            udcEpP->dma.alignedBuf = dma_alloc_coherent( NULL, udcEpP->dma.alignedLen, &(udcEpP->dma.alignedAddr), GFP_KERNEL );
            BCM_DEBUG_MEM( "%s: dma_alloc_coherent(): addr=0x%x length=%u\n", udcEpP->usb_ep.name, udcEpP->dma.alignedAddr, udcEpP->dma.alignedLen );
        }

        if ( udcEpP->dma.alignedBuf == NULL )
        {
            BCM_KERROR( "%s: dma_alloc_coherent() failed, length=%u\n", udcEpP->usb_ep.name, usb_req->length );
            BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
            return( -ENOMEM );
        }

        udcEpReqP->dmaAligned = 1;
    }
    /** @todo Should not really have dma == 0. Unfortunately the ether_262317.c has been hacked to do this. See Trac #1318. */
    else if ( (udcEpReqP->usb_req.dma == DMA_ADDR_INVALID) || (udcEpReqP->usb_req.dma == 0) )
    {
        /* A physical address was not provided for the DMA buffer, so request it.
         */
        udcEpReqP->dmaMapped = 1;
        udcEpReqP->usb_req.dma = dma_map_single( udcEpP->udcP->gadget.dev.parent,
                                                    udcEpReqP->usb_req.buf,
                                                    udcEpReqP->usb_req.length,
                                                    (udcEpP->dirn == USB_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE) );
    }

    spin_lock_irqsave(&udcP->lock, flags);

    udcEpReqP->usb_req.status = -EINPROGRESS;
    udcEpReqP->usb_req.actual = 0;

    BCM_DEBUG_REQ( "%s: req=0x%p buf=0x%p dma=0x%x len=%d\n", udcEpP->usb_ep.name, usb_req, usb_req->buf, udcEpReqP->usb_req.dma, usb_req->length );

    if ( (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) && (udcEpP->dirn == USB_DIR_OUT) && (udcEpReqP->usb_req.length == 0) )
    {
        /*
         * This might happen if gadget driver decides to send zero length packet (ZLP) during STATUS phase
         * of a control transfer. This may happen for the cases where there is not a DATA phase. Just consider
         * things complete. ZLP will be issued by hardware. See the handling of SETUP packets for more details
         * on control transfer processing.
         */
        ReqXferDone( udcEpP, udcEpReqP, ENOERROR );
    }
    else
    {
#ifdef CONFIG_ARCH_IPROC
        if (udcEpReqP->usb_req.length == 0)
            udcEpReqP->usb_req.zero = 1;
#endif
        ReqXferAdd( udcEpP, udcEpReqP );
    }

    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

int GadgetEpRequestDequeue( struct usb_ep *usb_ep, struct usb_request *usb_req )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    BCM_UDC_EP_REQ_t *udcEpReqP = container_of(usb_req, BCM_UDC_EP_REQ_t, usb_req);
    unsigned long flags;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    if ( !usb_ep || !usb_req )
    {
        BCM_KERROR( "invalid request\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -EINVAL );
    }

    spin_lock_irqsave(&udcEpP->udcP->lock, flags);

    /* Make sure it's actually queued on this endpoint */

    list_for_each_entry( udcEpReqP, &udcEpP->listQueue, listNode )
    {
        if (&udcEpReqP->usb_req == usb_req)
            break;
    }

    if (&udcEpReqP->usb_req != usb_req)
    {
        spin_unlock_irqrestore(&udcEpP->udcP->lock, flags);
        BCM_KNOTICE( "%s: request not queued\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( -ENOLINK );
    }

    /** @todo Handle case where the request is in progress, or completed but not dequeued */

    ReqXferDone(udcEpP, udcEpReqP, -ECONNRESET);
    spin_unlock_irqrestore(&udcEpP->udcP->lock, flags);

    BCM_DEBUG_REQ( "%s: req=0x%p\n", udcEpP->usb_ep.name, usb_req );
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}


int GadgetEpSetHalt( struct usb_ep *usb_ep, int haltEnable )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    unsigned long flags;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    if ( !usb_ep )
    {
        BCM_KERROR( "invalid request\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EINVAL );
    }

    if (udcEpP->type == USB_ENDPOINT_XFER_ISOC)
    {
        BCM_KWARN( "%s: ISO HALT operations not supported\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EOPNOTSUPP );
    }

    if ( haltEnable && (udcEpP->dirn == USB_DIR_IN) && !list_empty(&udcEpP->listQueue) )
    {
        /* Only allow halt on an IN EP if its queue is empty */

        BCM_KNOTICE( "%s: IN queue not empty\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EAGAIN );
    }

    if ( !haltEnable && (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) )
    {
        /*
         * Halt clear for a control EP should only be handled as part of the subsequent SETUP
         * exchange that occurs after the Halt was set.
         */
        BCM_KWARN( "%s: CTRL HALT clear\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return( -EPROTO );
    }

    spin_lock_irqsave( &udcEpP->udcP->lock, flags );

    if ( !haltEnable )
    {
        usbDevHw_EndptStallDisable( udcEpP->num, udcEpP->dirn );
    }
    else if ( udcEpP->type != USB_ENDPOINT_XFER_CONTROL )
    {
        usbDevHw_EndptStallEnable( udcEpP->num, udcEpP->dirn );
    }
    else
    {
        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_IN );
        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_OUT );
    }

    spin_unlock_irqrestore( &udcEpP->udcP->lock, flags );
#ifdef CONFIG_ARCH_IPROC
    mdelay(2);
#endif

    BCM_DEBUG_EP( "%s: HALT %s done\n", udcEpP->usb_ep.name, haltEnable ? "SET" : "CLR" );
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

int GadgetEpFifoStatus( struct usb_ep *usb_ep )
{
    BCM_DEBUG_TRACE( "enter/exit\n" );

    /*
     * The DWC UDC core doesn't have a mechanism for determining the number of bytes
     * currently in a FIFO. The best that can be done is determine whether or not a
     * FIFO is empty. However, for the situation where a single Rx FIFO is being
     * used for all endpoints, if cannot be determined which OUT and CTRL EP's are
     * affected if the Rx FIFO is not empty.
     */
    return( -EOPNOTSUPP );
}


void GadgetEpFifoFlush( struct usb_ep *usb_ep )
{
    BCM_UDC_EP_t *udcEpP = container_of(usb_ep, BCM_UDC_EP_t, usb_ep);
    unsigned long flags;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
    if ( !usb_ep )
    {
        BCM_KERROR( "invalid request\n" );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return;
    }

    if ( udcEpP->type == USB_ENDPOINT_XFER_CONTROL )
    {
        /*
         * FIFO flush for a control EP does not make any sense. The SETUP protocol
         * should eliminate the need to flush.
         */
        BCM_KWARN( "%s: CTRL FIFO flush\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return;
    }

    if ( usbDevHw_EndptFifoEmpty( udcEpP->num, udcEpP->dirn ) )
    {
        BCM_DEBUG_EP( "%s: FIFO empty\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
        return;
    }

    spin_lock_irqsave( &udcEpP->udcP->lock, flags );

    /** @todo There may be some issues for single Rx FIFO and subsequent EP0 operations */
    /** @todo The UDC doc'n also mentions having to set DEVNAK bit and clearing it later. */
    /* FIFO flush will need to be disabled later on. E.g. when a EP request is queued.
     */
    usbDevHw_EndptFifoFlushEnable( udcEpP->num, udcEpP->dirn );

    spin_unlock_irqrestore( &udcEpP->udcP->lock, flags );

    BCM_DEBUG_EP( "%s: FIFO flush enabled\n", udcEpP->usb_ep.name );
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

/***************************************************************************
 * Routines for debug dump of DMA descriptors
 **************************************************************************/

void DmaDump( BCM_UDC_t *udcP )
{
    unsigned i;


    for ( i = 0; i < BCM_UDC_EP_CNT; i++ )
    {
        DmaDumpEp( &udcP->ep[i] );
    }
}

void DmaDumpDesc( char *label, BCM_UDC_DMA_DESC_t *virt, BCM_UDC_DMA_DESC_t *phys )
{
    BCM_DEBUG_DMA( "%s virt=0x%p phys=0x%p: 0x%08x 0x%08x 0x%08x", label, virt, phys, virt->status, virt->reserved, virt->bufAddr );
}

void DmaDumpEp( BCM_UDC_EP_t *udcEpP )
{
    unsigned i;

    BCM_DEBUG_DMA(      "EP %d DMA\n", udcEpP->num );
    BCM_DEBUG_DMA(      "   setup\n" );
    DmaDumpDesc( "       ", (BCM_UDC_DMA_DESC_t *)&udcEpP->dma.virtualAddr->setup, (BCM_UDC_DMA_DESC_t *)&udcEpP->dma.physicalAddr->setup );
    BCM_DEBUG_DMA(      "   desc\n" );

    for ( i = 0; i < BCM_UDC_EP_DMA_DESC_CNT; i++ )
    {
        DmaDumpDesc( "       ", &udcEpP->dma.virtualAddr->desc[i], &udcEpP->dma.physicalAddr->desc[i] );
#ifdef CONFIG_ARCH_IPROC
        /* Don't bother displaying entries beyond the last. */
        if ( udcEpP->dma.virtualAddr->desc[i].status & usbDevHw_REG_DMA_STATUS_LAST_DESC )
            break;

#endif
    }
}

/****************************************************************************
 * Initialization of DMA descriptors at the endpoint level.
 ***************************************************************************/

void DmaEpInit( BCM_UDC_EP_t *udcEpP )
{
    unsigned i;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
    BCM_DEBUG_DMA( "%s: num=%u\n", udcEpP->usb_ep.name, udcEpP->num );

    /** @todo shorten names to virtAddr physAddr?? */
    udcEpP->dma.virtualAddr = &udcEpP->udcP->dma.virtualAddr->ep[ udcEpP->num ];
    udcEpP->dma.physicalAddr = &udcEpP->udcP->dma.physicalAddr->ep[ udcEpP->num ];

    /*
     * Control endpoints only do setup in the OUT direction, so only need to set the
     * buffer address for that direction. The buffer is set, even if not a control
     * endpoint, just to simplify things. There's no harm with this.
     */

    udcEpP->dma.virtualAddr->setup.status = usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
    wmb();
    usbDevHw_EndptDmaSetupBufAddrSet( udcEpP->num, USB_DIR_OUT, &udcEpP->dma.physicalAddr->setup );

    /*
     * Take ownership of the DMA descriptors, and chain them in a loop. This allows a small number
     * descriptors to be used for requests. Need to have the DWC DMA Descriptor Update option enabled
     * in the device control register in order to do this. When a transfer for a descriptor completes,
     * the descriptor will get re-used if there's still data left in a request to transfer. See the
     * DmaDataRemoveDone() and DmaDataAddReady() routines.
     */
     /** @todo Put these in endpoint context?? */

    for ( i = 0; i < BCM_UDC_EP_DMA_DESC_CNT; i++ )
    {
        udcEpP->dma.virtualAddr->desc[i].status = usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
        wmb();
        udcEpP->dma.virtualAddr->desc[i].nextDescAddr = (uint32_t)&udcEpP->dma.physicalAddr->desc[i+1];
    }
    udcEpP->dma.virtualAddr->desc[(BCM_UDC_EP_DMA_DESC_CNT - 1)].nextDescAddr = (uint32_t)&udcEpP->dma.physicalAddr->desc[0];

    /*
     * To simplify things, register the descriptor chain in both directions. Control endpoints are the
     * only type that will be transferring in both directions, but they will only be transferring in one
     * direction at a time, so should not be any issues with using the same descriptor set for both directions.
     * For single direction endpoints, the other direction will not be used.
     */

    usbDevHw_EndptDmaDataDescAddrSet( udcEpP->num, USB_DIR_OUT, &udcEpP->dma.physicalAddr->desc[0] );
    usbDevHw_EndptDmaDataDescAddrSet( udcEpP->num, USB_DIR_IN,  &udcEpP->dma.physicalAddr->desc[0] );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

/****************************************************************************
 * DMA descriptor chain routines.
 *
 *  DmaDescChainReset - Initialize chain in preparation for transfer
 *  DmaDescChainFull - Indicates if no descriptors in chain for available for use.
 *  DmaDescChainAlloc - Get next free descriptor for use. Have to check if chain not full first.
 *  DmaDescChainEmpty - Indicates if no descriptors in the chain are being used.
 *  DmaDescChainHead - Pointer to 1st entry in chain. Have to check if chain not empty first.
 *  DmaDescChainFree - Frees up 1st entry for use. Only do this if DMA for this descriptor has completed.
 *
 ***************************************************************************/

inline BCM_UDC_DMA_DESC_t *DmaDescChainAlloc( BCM_UDC_EP_t *udcEpP )
{
    unsigned idx;

    idx = udcEpP->dma.addIndex++;

    return( &udcEpP->dma.virtualAddr->desc[ BCM_UDC_EP_DMA_DESC_IDX(idx) ] );
}

inline int DmaDescChainEmpty( BCM_UDC_EP_t *udcEpP )
{
    return( udcEpP->dma.addIndex == udcEpP->dma.removeIndex );
}

inline void DmaDescChainFree( BCM_UDC_EP_t *udcEpP )
{
    udcEpP->dma.removeIndex++;
}

inline int DmaDescChainFull( BCM_UDC_EP_t *udcEpP )
{
    return( !DmaDescChainEmpty(udcEpP) && (BCM_UDC_EP_DMA_DESC_IDX(udcEpP->dma.addIndex) == BCM_UDC_EP_DMA_DESC_IDX(udcEpP->dma.removeIndex)) );
}

inline BCM_UDC_DMA_DESC_t *DmaDescChainHead( BCM_UDC_EP_t *udcEpP )
{
    return( &udcEpP->dma.virtualAddr->desc[ BCM_UDC_EP_DMA_DESC_IDX(udcEpP->dma.removeIndex) ] );
}

inline void DmaDescChainReset( BCM_UDC_EP_t *udcEpP )
{
    udcEpP->dma.addIndex = 0;
    udcEpP->dma.removeIndex = 0;
}

/****************************************************************************
 * DMA data routines.
 *
 * A gadget usb_request buf is used for the data. The entire buf contents may
 * or may not fit into the descriptor chain at once. When the DMA transfer
 * associated with a descriptor completes, the descriptor is re-used to add
 * more segments of the usb_request to the chain as necessary.
 *
 *  DmaDataInit - Initialization in preparation for DMA of usb_request.
 *  DmaDataAddReady - Adds usb_request segments into DMA chain until full or no segments left
 *  DmaDataRemoveDone - Removes usb_request segments from DMA chain that have completed transfer
 *  DmaDataFinis - Final stage of DMA of the usb_request
 *
 ***************************************************************************/

void DmaDataInit( BCM_UDC_EP_t *udcEpP )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;

    BCM_DEBUG_TRACE( "enter: %s\n", __func__ );


    udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );

    if ( udcEpReqP->dmaAligned )
    {
        /*
         * This buffer needs to be aligned in order to DMA. We do this by copying into a special buffer we
         * have for this purpose. Save the original DMA physical address so it can be restored later.
         * This may not be used, but we'll do it anyways. Then set the DMA address to the aligned buffer
         * address. Only the DMA physical address is used for the transfers, so the original buffer virtual
         * address does not need to be changed. Then copy the data into the aligned buffer.
         */
        /** @todo Really only need to do the memcpy for IN data */

        udcEpReqP->dmaAddrOrig = udcEpReqP->usb_req.dma;
        udcEpReqP->usb_req.dma = udcEpP->dma.alignedAddr;
        memcpy( udcEpP->dma.alignedBuf, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.length );
    }

    udcEpP->dma.done = 0;
    udcEpP->dma.lengthDone = 0;
    udcEpP->dma.lengthToDo = udcEpP->dma.usb_req->length;
    udcEpP->dma.bufAddr = udcEpP->dma.usb_req->dma;
    udcEpP->dma.status = usbDevHw_REG_DMA_STATUS_RX_SUCCESS;

    if ( (udcEpP->dirn == USB_DIR_IN) && (udcEpP->type != USB_ENDPOINT_XFER_ISOC) )
    {
        /*
         * For IN transfers, do not need to segment the buffer into max packet portions
         * for the DMA descriptors. The hardware will automatically segment into max
         * packet sizes as necessary.
         */

        #ifdef IN_DMA_BUFFER_FILL_ENABLED
            udcEpP->dma.lengthBufMax = udcEpP->dma.usb_req->length;
        #else
            udcEpP->dma.lengthBufMax = udcEpP->usb_ep.maxpacket;
        #endif
#ifdef BCM_UDC_IN_TX_MAX
        udcEpP->dma.lengthBufMax = BCM_UDC_IN_TX_MAX;
#endif
#ifndef CONFIG_ARCH_IPROC
        /*
         * If the request is of zero length, then force the zero flag so DmaDataAddReady()
         * will queue the request. Conversely, if the gadget has set the zero flag, leave
         * it set only if it is needed (request length is a multiple of maxpacket)
         */
        if ( udcEpP->dma.usb_req->length == 0 )
        {
            udcEpP->dma.usb_req->zero = 1;
            udcEpP->dma.lengthBufMax = udcEpP->usb_ep.maxpacket;
        }
        else if ( udcEpP->dma.usb_req->zero )
        {
            udcEpP->dma.usb_req->zero = (udcEpP->dma.usb_req->length % udcEpP->usb_ep.maxpacket) ? 0 : 1;
        }
#endif
    }
    else
    {
        udcEpP->dma.lengthBufMax = udcEpP->usb_ep.maxpacket;
    }

    DmaDescChainReset( udcEpP );

    BCM_DEBUG_DMA( "%s: todo=%d bufMax=%d buf=0x%x add=0x%x remove=0x%x\n",
                    udcEpP->usb_ep.name, udcEpP->dma.lengthToDo,
                    udcEpP->dma.lengthBufMax, udcEpP->dma.bufAddr, udcEpP->dma.addIndex, udcEpP->dma.removeIndex );

    usbDevHw_EndptIrqEnable( udcEpP->num, udcEpP->dirn );

    BCM_DEBUG_TRACE( "exit: %s\n", __func__);
}

void DmaDataFinis( BCM_UDC_EP_t *udcEpP )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;


    BCM_DEBUG_TRACE( "enter: %s\n", __func__ );

    usbDevHw_EndptIrqDisable( udcEpP->num, udcEpP->dirn );
    usbDevHw_EndptDmaDisable( udcEpP->num, udcEpP->dirn );

    udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );

    if ( udcEpReqP->dmaAligned )
    {
        /*
         * The original request buffer was not aligned properly, so a special buffer was used
         * for the transfer. Copy the aligned buffer contents into the original. Also restore
         * the original dma physical address.
         */
        /** @todo Really only need to do the memcpy for OUT setup/data */
        memcpy( udcEpReqP->usb_req.buf, udcEpP->dma.alignedBuf, udcEpReqP->usb_req.length );
        udcEpReqP->usb_req.dma = udcEpReqP->dmaAddrOrig;
    }

    BCM_DEBUG_DMA( "%s: udcEpReqP=0x%x buf=0x%x dma=0x%x actual=%u\n",
                    udcEpP->usb_ep.name, udcEpReqP, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.dma, udcEpReqP->usb_req.actual);

    BCM_DEBUG_TRACE( "%s: exit\n", __func__ );
}

void DmaDataAddReady( BCM_UDC_EP_t *udcEpP )
{
    volatile BCM_UDC_DMA_DESC_t *dmaDescP = NULL;
    uint32_t status;
    unsigned len;
#ifdef CONFIG_ARCH_IPROC
    int enable_dma = 0;
#endif


    BCM_DEBUG_TRACE( "enter: %s\n", __func__ );
#ifdef CONFIG_ARCH_IPROC
    /*
     * DMA must be disabled while this loop is running, as multi-descriptor transfers
     * will have the descriptor chain in an intermediate state until the last descriptor
     * is written and the chain terminated.
     */
    if (usbDevHw_DeviceDmaEnabled())
    {
        enable_dma = 1;
        usbDevHw_DeviceDmaDisable();
    }

    if (!udcEpP->dma.lengthToDo)
        udcEpP->dma.usb_req->zero = 1;
#endif
    /*
     * Will only have one request in the chain at a time. Add request segments to the
     * chain until all parts of the request have been put in the chain or the chain
     * has no more room.
     */
    while ( !DmaDescChainFull( udcEpP ) && (udcEpP->dma.lengthToDo || udcEpP->dma.usb_req->zero) )
	{
        /*
         * Get the next descriptor in the chain, and then fill the descriptor contents as needed.
         * Do not set the descriptor buffer status to ready until last to ensure there's no
         * contention with the hardware.
         */
        dmaDescP = DmaDescChainAlloc( udcEpP );

        len = udcEpP->dma.lengthToDo < udcEpP->dma.lengthBufMax ? udcEpP->dma.lengthToDo : udcEpP->dma.lengthBufMax;
        udcEpP->dma.lengthToDo -= len;

        status = 0;

        if ( len < udcEpP->dma.lengthBufMax )
        {
            /*
             * If this segment is less than the max, then it is the last segment. There's no need to
             * send a closing ZLP, although this segment might be a ZLP. Regardless, clear the ZLP flag
             * to ensure that the processing of this request finishes. Also set the end of the descriptor
             * chain.
             */
            udcEpP->dma.usb_req->zero = 0;
#ifndef CONFIG_ARCH_IPROC
            status |= usbDevHw_REG_DMA_STATUS_LAST_DESC;
#endif
        }
        else if ( (udcEpP->dma.lengthToDo == 0) && !udcEpP->dma.usb_req->zero )
        {
            /*
             * Segment is of the max packet length. Since there's nothing left, it has to also be the last
             * last segment. No closing ZLP segment requested, just set the end of the descriptor chain.
             */
#ifndef CONFIG_ARCH_IPROC
            status |= usbDevHw_REG_DMA_STATUS_LAST_DESC;
#endif
        }

        if ( (udcEpP->dirn == USB_DIR_IN) && (udcEpP->type == USB_ENDPOINT_XFER_ISOC) )
        {
            /*
             * Increment the frame number for transmit, then use it for the next packet. The frame number
             * may get larger than its 13-bit size, but the mask will handle the wrap-around so we don't
             * need to add checks for this condition. E.g. 0x7ff + 1 = 0x800. 0x800 & 0x7ff = 0 which
             * is the next number in the sequence.
             */
            /** @todo Handle ISOC PIDs and frame numbers used with HS high bandwidth transfers */
            /** @todo Might not need to set the last descriptor status. Currently restricting
             * IN ISOC transfers to the max packet size.
             */
#ifndef CONFIG_ARCH_IPROC
            status |= usbDevHw_REG_DMA_STATUS_LAST_DESC;
#endif
            udcEpP->dma.frameNum += udcEpP->dma.frameIncr;
            BCM_DEBUG_ISOC("%s: DMA start: frameNum=%d.%d\n", udcEpP->usb_ep.name, (udcEpP->dma.frameNum >> 3), (udcEpP->dma.frameNum & 0x7) );
            status |= ((udcEpP->dma.frameNum << usbDevHw_REG_DMA_STATUS_FRAME_NUM_SHIFT) & usbDevHw_REG_DMA_STATUS_FRAME_NUM_MASK);
        }

        dmaDescP->bufAddr = udcEpP->dma.bufAddr;
        status |= (len << usbDevHw_REG_DMA_STATUS_BYTE_CNT_SHIFT);
        dmaDescP->status = status | usbDevHw_REG_DMA_STATUS_BUF_HOST_READY;
        wmb();
        BCM_DEBUG_DMA( "%s: desc=0x%p status=0x%08x bufAddr=0x%08x len=%d add=0x%x\n",
                        udcEpP->usb_ep.name, dmaDescP, dmaDescP->status, dmaDescP->bufAddr, len, udcEpP->dma.addIndex );
		udcEpP->dma.bufAddr += len;

        if ( (udcEpP->dirn == USB_DIR_IN) && (udcEpP->type == USB_ENDPOINT_XFER_ISOC) )
        {
            /* With ISOC transfers, only enable one DMA descriptors at a time.
             */
            /** @todo Determine if FIFO will overflow. If it does not, then can remove this check.
             * This may not even be an issue if the buffer size is restricted to the max packet size
             * when a request is submitted to the endpoint.
             */
            break;
        }
    }
#ifdef CONFIG_ARCH_IPROC
    /* Set LAST bit on last descriptor we've configured */
    if (dmaDescP)
        dmaDescP->status |= usbDevHw_REG_DMA_STATUS_LAST_DESC;

    if (enable_dma)
        usbDevHw_DeviceDmaEnable();
#endif

    BCM_DEBUG_TRACE( "exit: %s\n", __func__ );
}

void DmaDataRemoveDone( BCM_UDC_EP_t *udcEpP )
{
    volatile BCM_UDC_DMA_DESC_t *dmaDescP;
    uint32_t status;
    unsigned len;


    BCM_DEBUG_TRACE( "enter: %s\n", __func__ );

    /*
     * Will only have one request in the chain at a time. Remove any completed
     * request segments from the chain so any segments awaiting transfer can
     * be put in the chain.
     */
    while ( !DmaDescChainEmpty( udcEpP ) )
    {
        /*
         * Examine the first entry in the chain. If its status is not done, then there's
         * nothing to remove.
         */
        dmaDescP = DmaDescChainHead( udcEpP );
#ifndef CONFIG_ARCH_IPROC
        if ( (dmaDescP->status & usbDevHw_REG_DMA_STATUS_BUF_MASK) != usbDevHw_REG_DMA_STATUS_BUF_DMA_DONE )
        {
            BCM_DEBUG_DMA( "%s: not done: desc=0x%p status=0x%x bufAddr=0x%08x add=%d remove=0x%x\n",
                            udcEpP->usb_ep.name, dmaDescP, dmaDescP->status, dmaDescP->bufAddr, udcEpP->dma.addIndex, udcEpP->dma.removeIndex );
            break;
        }
#endif

        /*
         * The transfer of this request segment has completed. Save the status info and then
         * take ownership of the descriptor. It is simpler to do this than modifying parts of
         * the descriptor in order to take ownership. Don't put the descriptor back in the chain
         * until all info affected by the status has been updated, just to be safe.
         */
        status = dmaDescP->status;
        dmaDescP->status = usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
        wmb();

        len = (status & usbDevHw_REG_DMA_STATUS_NON_ISO_BYTE_CNT_MASK) >> usbDevHw_REG_DMA_STATUS_NON_ISO_BYTE_CNT_SHIFT;
#ifdef CONFIG_ARCH_IPROC
        /* RX: For multiple descriptors, len is cumulative, not absolute.
         * RX: So only adjust the dma fields when we get to the last descriptor
         * TX: Each descriptor entry is absolute, count them all
         */
        if ((udcEpP->dirn == USB_DIR_IN) || (status & usbDevHw_REG_DMA_STATUS_LAST_DESC))
        {
            udcEpP->dma.lengthDone += len;
            udcEpP->dma.usb_req->actual += len;
        }
#else
        udcEpP->dma.lengthDone += len;
        udcEpP->dma.usb_req->actual += len;
#endif
        if ( (status & usbDevHw_REG_DMA_STATUS_RX_MASK) != usbDevHw_REG_DMA_STATUS_RX_SUCCESS )
        {
            udcEpP->dma.status = status & usbDevHw_REG_DMA_STATUS_RX_MASK;
            udcEpP->dma.usb_req->status = -EIO;
            BCM_KWARN( "%s: DMA error: desc=0x%p status=0x%x len=%d add=0x%x remove=0x%x\n",
                        udcEpP->usb_ep.name, dmaDescP, status, len, udcEpP->dma.addIndex, udcEpP->dma.removeIndex );
        }

        if ( (udcEpP->dirn == USB_DIR_IN) && (udcEpP->type == USB_ENDPOINT_XFER_ISOC) )
        {
            /** @todo Determine if this special processing needs to be done. May not to do this if the
             * buffer size is restricted to the max packet size when a request is submitted to the endpoint.
             */
            if ( udcEpP->dma.usb_req->actual == udcEpP->dma.usb_req->length )
            {
                udcEpP->dma.usb_req->status = ENOERROR;
            }
            DmaDescChainReset( udcEpP );
        }
        else
        {
#ifndef CONFIG_ARCH_IPROC
            if ( (status & usbDevHw_REG_DMA_STATUS_LAST_DESC) && (udcEpP->dma.usb_req->status == -EINPROGRESS) )
            {
                udcEpP->dma.usb_req->status = ENOERROR;
            }
#endif
            DmaDescChainFree( udcEpP );
        }

        BCM_DEBUG_DMA( "%s: desc=0x%p status=0x%x bufAddr=0x%08x len=%d remove=0x%x\n",
                        udcEpP->usb_ep.name, dmaDescP, status, dmaDescP->bufAddr, len, udcEpP->dma.removeIndex );
    }
#ifdef CONFIG_ARCH_IPROC
    /* When last segment processed, update status if there has not been an error */
    if (!udcEpP->dma.lengthToDo && (udcEpP->dma.usb_req->status == -EINPROGRESS))
    {
        udcEpP->dma.usb_req->status = ENOERROR;
    }
#endif

    BCM_DEBUG_TRACE( "exit: %s\n", __func__ );
}

/***************************************************************************
 * UDC Operations routines.
 *
 *  UdcOpsInit - Initialization of the UDC in preparation for use by Gadget driver.
 *  UdcOpsStartup - Start UDC operations. Happens after a Gadget driver attaches.
 *  UdcOpsShutdown - Stop UDC operations. Happens after a Gadget driver detaches.
 *  UdcOpsFinis - Finish / terminate all UDC operations
 *
 ***************************************************************************/
static void UdcOpsFinis( BCM_UDC_t *udcP )
{
    /** @todo Anything need to be done here?? */
}

static void UdcOpsInit( BCM_UDC_t *udcP )
{
    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
    BCM_DEBUG_DEV( "enter: dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x ep=0x%x\n",
                    usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                    usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask,
                    usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus );
    BCM_DEBUG_DEV( "enter: ep0: status=0x%x\n", usbDevHw_REG_P->eptFifoOut[0].status );

    usbDevHw_OpsInit();

    UdcFifoRamInit( udcP );

    /*
     * See usb/gadget/epautoconf.c for endpoint naming conventions.
     * Control endpoints are bi-directional, but initial transfer (SETUP stage) is always OUT.
     */
    /** @todo Really should make the non endpoint 0 init attributes configurable by the chip specific part
     * of the driver, i.e. the device instantiation. The settings below are for a chip specific DWG UDC
     * core configuration. Also should incorporate the DWG UDC endpoint type attribute as part of this,
     * which can be control, IN, OUT, or bidirectional.
     */

    UdcEpInit( udcP, 0, "ep0",    USB_DIR_OUT );
#ifdef CONFIG_ARCH_IPROC
	/* On the IPROC, all endpoints except ep0 are birectional and generic type */
    UdcEpInit(udcP, 1, "ep1in",  USB_DIR_IN);
    UdcEpInit(udcP, 2, "ep2out",  USB_DIR_OUT);
    UdcEpInit(udcP, 3, "ep3in", USB_DIR_IN);
    UdcEpInit(udcP, 4, "ep4out", USB_DIR_OUT);
    UdcEpInit(udcP, 5, "ep5in", USB_DIR_IN);
    UdcEpInit(udcP, 6, "ep6out", USB_DIR_OUT);
    UdcEpInit(udcP, 7, "ep7in", USB_DIR_IN);
    UdcEpInit(udcP, 8, "ep8out", USB_DIR_IN);
	UdcEpInit(udcP, 9, "ep9in", USB_DIR_IN);
#else
    UdcEpInit( udcP, 1, "ep1in",  USB_DIR_IN );
    UdcEpInit( udcP, 2, "ep2in",  USB_DIR_IN );
    UdcEpInit( udcP, 3, "ep3out", USB_DIR_OUT );
    UdcEpInit( udcP, 4, "ep4in",  USB_DIR_IN );
    UdcEpInit( udcP, 5, "ep5out", USB_DIR_OUT );
    UdcEpInit( udcP, 6, "ep6in",  USB_DIR_IN );
    UdcEpInit( udcP, 7, "ep7out", USB_DIR_OUT );
#endif

    UdcEpCfg( &udcP->ep[0], USB_ENDPOINT_XFER_CONTROL, USB_CONTROL_MAX_PACKET_SIZE );
    usbDevHw_DeviceSelfPwrEnable();
    /** @todo usbDevHw_DeviceRemoteWakeupEnable(); */
    /** @todo usbDevHw_DeviceDmaBurstEnable(); */
    //usbDevHw_DeviceSetDescriptorDisable();

    if ( debug & DEBUG_DMA )
    {
        DmaDump( udcP );
    }

	BCM_DEBUG_DEV( "exit: dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x ep=0x%x\n",
                    usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                    usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask,
                    usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus );
    BCM_DEBUG_DEV( "exit: ep0: status=0x%x\n", usbDevHw_REG_P->eptFifoOut[0].status );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

static void UdcOpsStartup( BCM_UDC_t *udcP )
{
#ifdef CONFIG_ARCH_IPROC
    unsigned num;
#endif
    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /*
     * Just enable interrupts for now. Endpoint 0 will get enabled once the speed enumeration
     * has completed. The Device DMA enable is global in scope. There's endpoint specific
     * DMA enables that will happen later.
     */
    usbDevHw_DeviceIrqEnable( usbDevHw_DEVICE_IRQ_SPEED_ENUM_DONE |
                              usbDevHw_DEVICE_IRQ_BUS_SUSPEND |
                              usbDevHw_DEVICE_IRQ_BUS_IDLE |
                              usbDevHw_DEVICE_IRQ_BUS_RESET |
                              usbDevHw_DEVICE_IRQ_SET_INTF |
                              usbDevHw_DEVICE_IRQ_SET_CFG
                            );
    usbDevHw_DeviceDmaEnable();

#ifdef CONFIG_ARCH_IPROC
    /* Enable interrupts for all configured endpoints */
    for (num = 0; num < BCM_UDC_EP_CNT; ++num)
    {
        if (udcP->ep[num].usb_ep.name)
        {
            usbDevHw_EndptIrqEnable(udcP->ep[num].num, USB_DIR_OUT);
            usbDevHw_EndptIrqEnable(udcP->ep[num].num, USB_DIR_IN);
        }
    }
    usbDevHw_DeviceNakAllOutEptDisable();
#endif

    BCM_DEBUG_DEV( "dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x ep=0x%x\n",
                    usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                    usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask,
                    usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus );
    BCM_DEBUG_DEV( "ep0: status=0x%x\n", usbDevHw_REG_P->eptFifoOut[0].status );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

void UdcOpsShutdown( BCM_UDC_t *udcP )
{
    BCM_UDC_EP_t *udcEpP;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    usbDevHw_DeviceDmaDisable();
    usbDevHw_DeviceIrqDisable( usbDevHw_DEVICE_IRQ_ALL );
    usbDevHw_DeviceIrqClear( usbDevHw_DEVICE_IRQ_ALL );

    udcP->gadget.speed = USB_SPEED_UNKNOWN;

    ReqQueueFlush( &udcP->ep[0], -ESHUTDOWN );
    list_for_each_entry( udcEpP, &udcP->gadget.ep_list, usb_ep.ep_list )
    {
        ReqQueueFlush( udcEpP, -ESHUTDOWN );
    }

    BCM_DEBUG_DEV( "dev: ctrl=0x%x stat=0x%x  irq mask: dev=0x%x ep=0x%x  irq status: dev=0x%x ep=0x%x\n",
                    usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                    usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask,
                    usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus );
    BCM_DEBUG_DEV( "ep0: status=0x%x\n", usbDevHw_REG_P->eptFifoOut[0].status );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

#ifdef CONFIG_PM_USBD
#ifdef CONFIG_ARCH_IPROC
static struct iproc_pm_reg iproc_pm_regs_usbd[((sizeof(usbDevHw_REG_ENDPT_FIFO_t)/sizeof(uint32_t)) * BCM_UDC_EP_CNT * 2) +  5 + BCM_UDC_EP_CNT];

uint32_t usbd_reg32_read(const uint32_t addr)
{
	uint32_t val;
	
	if(!addr)
	{
		BCM_KERROR("Invalid reg addr: 0x%08x, caller: [%pS]\n", addr, __builtin_return_address(0));
		return 0;
	}
	
	val = readl_relaxed((void *)addr);

	return val;
}

void usbd_reg32_write(const uint32_t addr, const uint32_t value)
{
	
	if(!addr)
	{
		BCM_KERROR("Invalid reg addr: 0x%08x, caller: [%pS]\n", addr, __builtin_return_address(0));
		return ;
	}
	writel_relaxed(value, (void *)addr);
}


static struct iproc_pm_device_regs iproc_pm_dev_usbd = {
	.devname  = "USBD",
	.regs     = &iproc_pm_regs_usbd,
	.regs_num = ARRAY_SIZE(iproc_pm_regs_usbd),
	.read	  = usbd_reg32_read,
	.write	  = usbd_reg32_write,
};


void UdcPmSaveRegs(struct iproc_pm_device_regs *pm_dev)
{
	int i=0;
	uint32_t cnt = 0;
	struct iproc_pm_reg *saveReg = NULL;

	saveReg = *(pm_dev->regs);
	for(i=0; i < BCM_UDC_EP_CNT; i++) {
		/* IN endpoint registers */
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoIn[i].ctrl;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoIn[i].status;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoIn[i].size1;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoIn[i].size2;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoIn[i].setupBufAddr;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoIn[i].dataDescAddr;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;

		/* OUT endpoint registers */
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoOut[i].ctrl;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoOut[i].status;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoOut[i].size1;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoOut[i].size2;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoOut[i].setupBufAddr;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
		saveReg->regaddr = &usbDevHw_REG_P->eptFifoOut[i].dataDescAddr;
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;

		/* Save Endpoint config registers */
		saveReg->regaddr = &usbDevHw_REG_P->eptCfg[i];
		saveReg->regval= readl_relaxed(saveReg->regaddr);
		saveReg++; cnt++;
	}

	saveReg->regaddr = &usbDevHw_REG_P->devCfg;
	saveReg->regval= readl_relaxed(saveReg->regaddr);
	saveReg++; cnt++;
	saveReg->regaddr = &usbDevHw_REG_P->devStatus;
	saveReg->regval= readl_relaxed(saveReg->regaddr);
	saveReg++; cnt++;
	saveReg->regaddr = &usbDevHw_REG_P->devIntrMask;
	saveReg->regval= readl_relaxed(saveReg->regaddr);
	saveReg++; cnt++;
	saveReg->regaddr = &usbDevHw_REG_P->eptIntrMask;
	saveReg->regval= readl_relaxed(saveReg->regaddr);
	saveReg++; cnt++;
	saveReg->regaddr = &usbDevHw_REG_P->devCtrl;
	saveReg->regval= readl_relaxed(saveReg->regaddr);
	saveReg++; cnt++;
	
	pm_dev->regs_num = cnt;
}

void UdcPmDumpRegs(struct iproc_pm_device_regs *pm_dev)
{
	int i=0;
	struct iproc_pm_reg *saveReg = NULL;


	for (i=0; i<pm_dev->regs_num; i++)
	{
		saveReg = *(pm_dev->regs) + i;

		printk("regaddr:0x%08x  val:0x%08x\n",(uint32_t)(saveReg->regaddr), (uint32_t)(saveReg->regval));
	}
}

void UdcPmRestoreRegs(struct iproc_pm_device_regs *pm_dev)
{
	int i=0;
	struct iproc_pm_reg *saveReg = NULL;


	for (i=0; i<pm_dev->regs_num; i++)
	{
		saveReg = *(pm_dev->regs) + i;

		pm_dev->write((uint32_t)(saveReg->regaddr), (uint32_t)(saveReg->regval));
	}
}

#endif //#ifdef CONFIG_ARCH_IPROC

int UdcOpsSuspend(BCM_UDC_t *udcP)
{
    unsigned long flags;
    int rc =0;
    BCM_DEBUG_TRACE("UdcOpsSuspend\n");
    usbDevHw_DeviceBusDisconnect();
    udcP->pullupOn = 0;
    udelay(20);

    disable_irq(BCM_UDC_IRQ);

#ifdef CONFIG_ARCH_BCMRING
    if (gVbusDetectGpio >= 0) {
        disable_irq(gpio_to_irq(gVbusDetectGpio));
#ifndef CONFIG_USB_WAKESOURCE_BCMRING_UDC
        GpioHw_IrqDisable(gVbusDetectGpio);
#endif
    }
#endif

    local_irq_save(flags );
    usbDevHw_DeviceDmaDisable();
    usbDevHw_DeviceIrqDisable( usbDevHw_DEVICE_IRQ_ALL );
    usbDevHw_DeviceIrqClear( usbDevHw_DEVICE_IRQ_ALL );

    udcP->gadget.speed = USB_SPEED_UNKNOWN;
    
    local_irq_restore(flags );
#ifdef CONFIG_ARCH_BCMRING
	rc = bcmring_usb_suspend(1);
#endif
#ifdef CONFIG_ARCH_IPROC
	usbDevHw_PrintInfo(&printk);
	UdcPmSaveRegs(&iproc_pm_dev_usbd); // save current settings
	UdcPmDumpRegs(&iproc_pm_dev_usbd);
#endif
    udelay(50);
    return rc;
}

int UdcOpsResume(BCM_UDC_t *udcP)
{
    unsigned long flags;
    int rc = 0;
    BCM_DEBUG_TRACE("UdcOpsResume\n");
#ifdef CONFIG_ARCH_BCMRING		
    rc = bcmring_usb_resume(1);
#endif
#ifdef CONFIG_ARCH_IPROC
	/* PHY was completely shutdown, reconfigure it */
	config_usb_phy();
	UdcPmRestoreRegs(&iproc_pm_dev_usbd);
	usbDevHw_PrintInfo(&printk);
#endif
    local_irq_save(flags);
    udcP->pullupOn = 1;
    UdcOpsStartup( udcP );
    usbDevHw_DeviceBusConnect();
    local_irq_restore(flags);

#ifdef CONFIG_ARCH_BCMRING
    if (gVbusDetectGpio >= 0) {
#ifndef CONFIG_USB_WAKESOURCE_BCMRING_UDC
        GpioHw_IrqEnable(gVbusDetectGpio);
#endif
        enable_irq(gpio_to_irq(gVbusDetectGpio));
    }
#endif
    enable_irq(BCM_UDC_IRQ);
    return rc;
    
}

static int  UdcPmSuspend(struct device *dev)
{
    int rc;
    rc = UdcOpsSuspend( bcmUdcP );
    return rc;
}
static int  UdcPmResume(struct device *dev)
{
    int rc = 0;
    //rc = UdcOpsResume( bcmUdcP );
    return rc;
}
#endif  //CONFIG_PM_USBD

/****************************************************************************
 * Control Endpoint SETUP related routines.
 *
 *  CtrlEpSetupInit - Prepares for next SETUP Rx. Status indicates if STALL req'd.
 *  CtrlEpSetupRx - Handle Rx of a SETUP.
 *
 ***************************************************************************/

void CtrlEpSetupInit( BCM_UDC_EP_t *udcEpP, int status )
{
    BCM_DEBUG_TRACE( "%s: enter\n", __func__ /*udcEpP->usb_ep.name */);

    /* Re-enable transfers to the SETUP buffer, clear IN and OUT NAKs, and re-enable OUT interrupts. */

    udcEpP->dma.virtualAddr->setup.status = usbDevHw_REG_DMA_STATUS_BUF_HOST_READY;
    udcEpP->dirn = USB_DIR_OUT;
    udcEpP->stopped = 0;

    if ( status == ENOERROR )
    {
        /* Handling of previous SETUP was OK. Just clear any NAKs. */

        usbDevHw_EndptNakClear( udcEpP->num, USB_DIR_OUT );
        usbDevHw_EndptNakClear( udcEpP->num, USB_DIR_IN );
    }
    else
    {
        /*
         * Handling of previous SETUP failed. Set the STALL. This will get cleared
         * when the next SETUP is rx'd.
         */

        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_IN );
        usbDevHw_EndptStallEnable( udcEpP->num, USB_DIR_OUT );
    }
#ifdef CONFIG_ARCH_IPROC
    /* This order is the correct one. Masked IRQs aren't posted */
    usbDevHw_EndptIrqEnable( udcEpP->num, USB_DIR_OUT );
    usbDevHw_EndptDmaEnable( udcEpP->num, USB_DIR_OUT );
#else
    usbDevHw_EndptDmaEnable( udcEpP->num, USB_DIR_OUT );
    usbDevHw_EndptIrqEnable( udcEpP->num, USB_DIR_OUT );
#endif

    BCM_DEBUG_CTRL( "%s: status=%d\n", udcEpP->usb_ep.name, status );
    BCM_DEBUG_TRACE( "%s: exit\n", __func__  /*udcEpP->usb_ep.name, status */);
}

/** @todo this only happens in the context of an irq. Might rename IrqCtrlEpSetupRx. */

void CtrlEpSetupRx( BCM_UDC_EP_t *udcEpP, struct usb_ctrlrequest *setup )
{
    BCM_UDC_t *udcP;
    unsigned value;
    unsigned index;
    unsigned length;
    int status;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /*
     * For a CTRL EP, the initial stage is a SETUP in the OUT direction. The direction of the subsequent DATA (optional)
     * and STATUS stages are dependant upon the SETUP. The DWC UDC will enable NAK for both directions upon the RX
     * of a SETUP. The order in which the NAK bits get cleared is dependant upon the direction of the subsequent
     * stages.
     *
     * To start, disable interrupts for the OUT direction. The appropriate direction interrupt will get set once
     * the subsequent transfer direction is determined.
     */
#ifndef CONFIG_ARCH_IPROC
    usbDevHw_EndptIrqDisable( udcEpP->num, USB_DIR_OUT );
#endif

    value = le16_to_cpu( setup->wValue );
    index = le16_to_cpu( setup->wIndex );
    length = le16_to_cpu( setup->wLength );

    /*
     * Any SETUP packets appearing here need to be handled by the gadget driver. Some SETUPs may have
     * already been silently handled and acknowledged by the DWC UDC. The exceptions to this rule are the
     * USB_REQ_SET_CONFIGURATION and USB_REQ_SET_INTERFACE, which have been only partially handled with
     * the expectation that some additional software processing is required in order to complete these requests.
     * Thus, they have not been acknowledged by the DWC UDC. There is no DATA stage for these requests.
     */

    /*
     * Set the direction of the subsequent DATA stage of a control transfer. This is an
     * optional stage. It may not exist for all control transfers. If there is a DATA
     * stage, this info is used for DMA operations for any requests received from the
     * Gadget driver.
     */

    udcEpP->dirn = setup->bRequestType & USB_DIR_MASK;
    udcP = udcEpP->udcP;

    if ( udcEpP->num != 0 )
    {
        /** @todo Make changes here if the Linux USB gadget ever supports a control endpoint other
         * than endpoint 0. The DWC UDC supports multiple control endpoints, and this driver has
         * been written with this in mind. To make things work, really need to change the Gadget
         * setup() callback parameters to provide an endpoint context, or add something similar
         * to the usb_ep structure, or possibly use a usb_request to hold a setup data packet.
         */

        BCM_KERROR( "%s: control transfer not supported\n", udcEpP->usb_ep.name );
        status = -EOPNOTSUPP;
    }
    else
    {
        /*
         * Forward the SETUP to the gadget driver for processing. The appropriate directional
         * interrupt and NAK clear will happen when the DATA stage request is queued.
         */

        BCM_DEBUG_CTRL( "%s: SETUP %02x.%02x value=%04x index=%04x len=%04x\n",
                        udcEpP->usb_ep.name, setup->bRequestType, setup->bRequest, value, index, length );

        spin_unlock(&udcP->lock);
        status = udcP->gadget_driver->setup (&udcP->gadget, setup);
        spin_lock(&udcP->lock);
    }

    if ( status < 0 )
    {
        /*
         * Error occurred during the processing of the SETUP, so enable STALL. This condition
         * can only be cleared with the RX of another SETUP, so prepare for that event.
         */

        BCM_KNOTICE( "%s: SETUP %02x.%02x STALL; status=%d\n",
                     udcEpP->usb_ep.name, setup->bRequestType, setup->bRequest, status );

        CtrlEpSetupInit( udcEpP, status );
    }
    else if ( length == 0 )
    {
        /* No DATA stage. Just need to prepare for the next SETUP. */

        CtrlEpSetupInit( udcEpP, ENOERROR );
    }
    else
    {
        /*
         * The SETUP stage processing has completed OK, and there may or may not be a request queued
         * for the DATA stage. When the DATA stage completes, preparation for the RX of the next
         * SETUP will be done.
         */
    }

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}


/****************************************************************************
 * IRQ routines.
 *
 *  IrqUdc - top level entry point.
 *  IrqDev - top level device related interrupt handler
 *  IrqDevCfgSet - device (endpoint 0) set config interrupt handler
 *  IrqDevIntfSet - device (endpoint 0) set interface interrupt handler
 *  IrqDevSpeedEnum - device speed enumeration done interrupt handler
 *  IrqEp - top level endpoint related interrupt handler
 *  IrqEpInStatusCheck - top level IN endpoint related interrupt handler
 *  IrqEpOutStatusCheck -  top level OUT endpoint related interrupt handler
 *  IrqEpOutSetup - Control endpoint SETUP Rx handler. This may get called
 *                  directly as the result of an endpoint OUT interrupt, or
 *                  indirectly as the result of device SET_CFG or SET_INTF.
 *
 ***************************************************************************/

irqreturn_t IrqUdc(int irq, void *context)
{
    BCM_UDC_t *udcP;
    unsigned long flags;
    uint32_t irqDev;
    uint32_t irqEpIn;
    uint32_t irqEpOut;
	int i;

    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /** @todo sanity check irq */
    (void)irq;

    udcP = (BCM_UDC_t *)context;

    spin_lock_irqsave( &udcP->lock, flags );

    if ( !udcP || !udcP->gadget_driver )
    {
        BCM_KERROR( "invalid context or no driver registered: irq dev=0x%x\n", usbDevHw_DeviceIrqActive() );

        usbDevHw_DeviceIrqClear( usbDevHw_DEVICE_IRQ_ALL );
        usbDevHw_EndptIrqListClear( USB_DIR_IN, ~0 );
        usbDevHw_EndptIrqListClear( USB_DIR_OUT, ~0 );

        spin_unlock_irqrestore( &udcP->lock, flags );
        BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

        return( IRQ_HANDLED );
    }

	BCM_DEBUG_IRQ( "enter: devCfg: 0x%x, devCtrl: 0x%x, devStatus: 0x%x, mask: dev=0x%x ep=0x%x  status: dev=0x%x ep=0x%x\n",
		usbDevHw_REG_P->devCfg, usbDevHw_REG_P->devCtrl, usbDevHw_REG_P->devStatus,
                    usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask,
                    usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus );

    /** @todo change Active to Pending?? */
    /** @todo merge usbDevHw EP IN/OUT routines?? Can only have 16 endpoints max due to a USB protocol restriction. */

    irqDev = usbDevHw_DeviceIrqActive();
    irqEpIn = usbDevHw_EndptIrqListActive( USB_DIR_IN );
    irqEpOut = usbDevHw_EndptIrqListActive( USB_DIR_OUT );

    BCM_DEBUG_IRQ( "enter: irqDev=0x%x irqEpIn=0x%x irqEpOut=0x%x\n", irqDev, irqEpIn, irqEpOut );

    usbDevHw_DeviceIrqClear( irqDev );
    usbDevHw_EndptIrqListClear( USB_DIR_IN, irqEpIn );
    usbDevHw_EndptIrqListClear( USB_DIR_OUT, irqEpOut );

    /*
     * Handle the SET_CFG and SET_INTF interrupts after the endpoint and other device interrupts.
     * There can be some race conditions where we have an endpoint 0 interrupt pending for the
     * completion of a previous endpoint 0 transfer (e.g. a GET config) when a SETUP arrives
     * corresponding to the SET_CFG and SET_INTF. Need to complete the processing of the previous
     * transfer before handling the next one, i.e. the SET_CFG or SET_INTF.
     */

    IrqDev( udcP, irqDev & ~(usbDevHw_DEVICE_IRQ_SET_CFG | usbDevHw_DEVICE_IRQ_SET_INTF) );
    IrqEp( udcP, irqEpIn, irqEpOut );
    IrqDev( udcP, irqDev & (usbDevHw_DEVICE_IRQ_SET_CFG | usbDevHw_DEVICE_IRQ_SET_INTF) );

    spin_unlock_irqrestore( &udcP->lock, flags );

    BCM_DEBUG_IRQ( " exit: mask: dev=0x%x ep=0x%x  status: dev=0x%x ep=0x%x\n",
                 usbDevHw_REG_P->devIntrMask, usbDevHw_REG_P->eptIntrMask,
                 usbDevHw_REG_P->devIntrStatus, usbDevHw_REG_P->eptIntrStatus );

	BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( (irqDev || irqEpIn || irqEpOut) ? IRQ_HANDLED : IRQ_NONE );
}

void IrqDev( BCM_UDC_t *udcP, uint32_t irq )
{
    if ( irq & usbDevHw_DEVICE_IRQ_BUS_RESET )
    {
        BCM_KINFO( "BUS reset\n" );
        /** @todo: add support for reset */
    }

    if ( irq & usbDevHw_DEVICE_IRQ_BUS_SUSPEND )
    {
        BCM_DEBUG_DEV( "BUS suspend\n" );
        /** @todo: add support for suspend */
    }

    if ( irq & usbDevHw_DEVICE_IRQ_BUS_IDLE )
    {
        BCM_DEBUG_DEV("BUS idle\n" );
        UdcOpsDisconnect( udcP );
    }

    if ( irq & usbDevHw_DEVICE_IRQ_SPEED_ENUM_DONE )
    {
        BCM_DEBUG_DEV( "BUS speed enum done\n" );
        IrqDevSpeedEnum( udcP );
    }

    if ( irq & usbDevHw_DEVICE_IRQ_SET_CFG )
    {
        BCM_DEBUG_DEV( "SET CFG\n" );
        IrqDevCfgSet( udcP );
    }

    if ( irq & usbDevHw_DEVICE_IRQ_SET_INTF )
    {
        BCM_DEBUG_DEV( "SET INTF\n" );
        IrqDevIntfSet( udcP );
    }
}

void IrqDevCfgSet( BCM_UDC_t *udcP )
{
    struct usb_ctrlrequest setup;
    unsigned epNum;
    uint16_t cfg;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /*
     * Device Configuration SETUP has been received. This is not placed in the SETUP
     * DMA buffer. The packet has to be re-created here so it can be forwarded to the
     * gadget driver to act upon.
     */

    cfg = (uint16_t) usbDevHw_DeviceCfgNum();

    setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    setup.bRequest = USB_REQ_SET_CONFIGURATION;
    setup.wValue = cpu_to_le16(cfg);
    setup.wIndex = 0;
    setup.wLength = 0;

    /*
     * Setting the configuration number before the gadget responds is a bit presumptious, but should
     * not be fatal.
     */
    /** @todo Do not set endpoint 0? Or is it a don't care? */

    for ( epNum = 0; epNum < BCM_UDC_EP_CNT; epNum++)
    {
        usbDevHw_EndptCfgSet( epNum, cfg );
    }

    BCM_KINFO( "SET CFG=%d\n", cfg );

    CtrlEpSetupRx( &udcP->ep[0], &setup );
    usbDevHw_DeviceSetupDone();
    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

void IrqDevIntfSet( BCM_UDC_t *udcP )
{
    struct usb_ctrlrequest setup;
    unsigned epNum;
    uint16_t intf;
    uint16_t alt;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    /*
     * Device Interface SETUP has been received. This is not placed in the SETUP
     * DMA buffer. The packet has to be re-created here so it can be forwarded to the
     * gadget driver to act upon.
     */

    intf = (uint16_t) usbDevHw_DeviceIntfNum();
    alt =  (uint16_t) usbDevHw_DeviceAltNum();

    setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE;
    setup.bRequest = USB_REQ_SET_INTERFACE;
    setup.wValue = cpu_to_le16(alt);
    setup.wIndex = cpu_to_le16(intf);
    setup.wLength = 0;

    /*
     * Setting the interface numbers before the gadget responds is a bit presumptious, but should
     * not be fatal.
     */
    /** @todo Do not set endpoint 0? Or is it a don't care? */

    for ( epNum = 0; epNum < BCM_UDC_EP_CNT; epNum++)
    {
        usbDevHw_EndptAltSet( epNum, alt );
        usbDevHw_EndptIntfSet( epNum, intf );
    }

    BCM_KINFO( "SET INTF=%d ALT=%d\n", intf, alt );

    CtrlEpSetupRx( &udcP->ep[0], &setup );
    usbDevHw_DeviceSetupDone();

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

void IrqDevSpeedEnum( BCM_UDC_t *udcP )
{
    unsigned prevSpeed;


    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );

    prevSpeed = udcP->gadget.speed;

    switch( usbDevHw_DeviceSpeedEnumerated() )
    {
        case usbDevHw_DEVICE_SPEED_HIGH:

            BCM_KINFO( "HIGH SPEED\n" );
            udcP->gadget.speed = USB_SPEED_HIGH;
            break;

        case usbDevHw_DEVICE_SPEED_FULL:

            BCM_KINFO( "FULL SPEED\n" );
            udcP->gadget.speed = USB_SPEED_FULL;
            break;

        case usbDevHw_DEVICE_SPEED_LOW:

            BCM_KWARN( "low speed not supported\n" );
            udcP->gadget.speed = USB_SPEED_LOW;
            break;

        default:

            BCM_KERROR( "unknown speed=0x%x\n", usbDevHw_DeviceSpeedEnumerated() );
            break;
    }

    if ( (prevSpeed == USB_SPEED_UNKNOWN) && (udcP->gadget.speed != USB_SPEED_UNKNOWN) )
    {
        /*
         * Speed has not been enumerated before, so now we can initialize transfers on endpoint 0.
         * Also have to disable the NAKs at a global level, which has been in place while waiting
         * for enumeration to complete.
         */

        BCM_DEBUG_DEV( "dev status=0x%08x: ep0 IN status=0x%08x OUT status=0x%08x\n",
                        usbDevHw_REG_P->devStatus, usbDevHw_REG_P->eptFifoIn[0].status, usbDevHw_REG_P->eptFifoOut[0].status );
        CtrlEpSetupInit( &udcP->ep[0], ENOERROR );
        usbDevHw_DeviceNakAllOutEptDisable();
    }

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );
}

void IrqEp( BCM_UDC_t *udcP, uint32_t irqIn, uint32_t irqOut )
{
    uint32_t mask;
    unsigned num;


    mask = 1;
    for ( num = 0; num < BCM_UDC_EP_CNT; num++ )
    {
        if ( irqIn & mask )
        {
            IrqEpInStatusCheck( &udcP->ep[num] );
        }
        if ( irqOut & mask )
        {
            IrqEpOutStatusCheck( &udcP->ep[num] );
        }
        mask <<= 1;
    }

}

void IrqEpInStatusCheck( BCM_UDC_EP_t *udcEpP )
{
    uint32_t status;


    status = usbDevHw_EndptStatusActive( udcEpP->num, USB_DIR_IN );
    usbDevHw_EndptStatusClear( udcEpP->num, USB_DIR_IN, status );

	BCM_DEBUG_IRQ( "enter: %s: %s: status=0x%x\n", __func__, udcEpP->usb_ep.name, status );
    if ( !status )
    {
        return;
    }

    /** @todo check might only be for direction... */
    if ( (udcEpP->dirn != USB_DIR_IN) && (udcEpP->type != USB_ENDPOINT_XFER_CONTROL) )
    {
        BCM_KERROR( "%s: unexpected IN interrupt\n", udcEpP->usb_ep.name );
        return;
    }

    if ( udcEpP->dirn != USB_DIR_IN )
    {
        /* This probably should not be happening */
        BCM_DEBUG_IRQ( "%s: CTRL dirn OUT\n", udcEpP->usb_ep.name );
    }

    if ( (udcEpP->type == USB_ENDPOINT_XFER_ISOC) &&
         (status & (usbDevHw_ENDPT_STATUS_IN_XFER_DONE | usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL)) )
    {
        BCM_KWARN("%s: ISOC IN unexpected status=0x%x\n", udcEpP->usb_ep.name, status );
    }

    if ( status & usbDevHw_ENDPT_STATUS_IN_TOKEN_RX )
    {
        /*
         * If there's any IN requests, the DMA should be setup and ready to go if
         * the endpoint is not an ISOC. Nothing to do in this case. However, if
         * this is an ISOC endpoint, then this interrupt implies there was no
         * data available for this frame number. This will happen if the gadget
         * does not have any data queued to send in this frame, or we have been
         * waiting for this event to occur so we can get alignment with the host
         * for the interval. This alignment is necessary when the interval is
         * greater than one frame / uframe. E.g. for an audio stream sending
         * samples @ 5ms intervals on a FS link, this corresponds to a period
         * of 5 frames. Samples with be queued for every 5th frame number after
         * the frame number in which this interrupt occurred.
         */

        status &= ~usbDevHw_ENDPT_STATUS_IN_TOKEN_RX;
#ifdef CONFIG_ARCH_IPROC
            usbDevHw_EndptNakClear(udcEpP->num, USB_DIR_IN);
#endif
        if ( (udcEpP->type == USB_ENDPOINT_XFER_ISOC) )
        {
            /* Always align to the current frame number for subsequent transfers. */

            udcEpP->dma.frameNum = usbDevHw_DeviceFrameNumLastRx();
            BCM_DEBUG_ISOC("%s: ISOC IN rx: align frameNum=%d.%d\n", udcEpP->usb_ep.name, (udcEpP->dma.frameNum >> 3), (udcEpP->dma.frameNum & 0x7) );
            if ( udcEpP->dma.usb_req != NULL )
            {
                /*
                 * Might have something queued when waiting for alignment. If something is queued,
                 * it is already too late for the current transfer point. It will also have been
                 * placed in the queue at some point before this interrupt, and it will be stale
                 * if we try to transmit at the next transfer point.
                 */

                udcEpP->dma.usb_req->status = -EREMOTEIO;
                ReqXferProcess( udcEpP );
            }
        }
    }

    if ( status & usbDevHw_ENDPT_STATUS_IN_DMA_DONE )
    {
        /*
         * DMA has completed, but cannot start next transfer until usbDevHw_ENDPT_STATUS_IN_XFER_DONE.
         * To avoid race conditions and other issues, do not release the current transfer until both
         * interrupts have arrived. Normally this interrupt will arrive at or before the IN_XFER_DONE,
         * but there have been situations when the system is under load that this interrupt might
         * arrive after the IN_XFER_DONE, in which case we will need to do the processing now.
         * The exception to this rule is for ISOC endpoints. They will only get this interrupt to
         * indicate that DMA has completed.
         */

        status &= ~usbDevHw_ENDPT_STATUS_IN_DMA_DONE;

        if ( (udcEpP->type == USB_ENDPOINT_XFER_ISOC) )
        {
            BCM_DEBUG_ISOC("%s: ISOC IN DMA done: frameNum=%d.%d\n", udcEpP->usb_ep.name, (udcEpP->dma.frameNum >> 3), (udcEpP->dma.frameNum & 0x7) );
            ReqXferProcess( udcEpP );
        }
        else if ( udcEpP->dma.done & usbDevHw_ENDPT_STATUS_IN_XFER_DONE )
        {
            /*
             * Did not receive the IN_DMA_DONE interrupt for this request before or
             * at the same time as the IN_XFER_DONE interrupt, so the request
             * processing was postponed until the IN_DMA_DONE interrupt arrived.
             * See handling of IN_XFER_DONE status below.
             */
            BCM_DEBUG_DMA("%s: late IN DMA done rec'd\n", udcEpP->usb_ep.name );
            ReqXferProcess( udcEpP );
        }
        else
        {
            /*
             * IN_DMA_DONE received. Save this info so request processing will be
             * done when the IN_XFER_DONE interrupt is received. This may happen
             * immediately, i.e. both IN_DMA_DONE and IN_XFER_DONE status are
             * set when the interrupt processing takes place.
             */
            udcEpP->dma.done = usbDevHw_ENDPT_STATUS_IN_DMA_DONE;
        }
    }

    if ( status & usbDevHw_ENDPT_STATUS_IN_XFER_DONE )
    {
        status &= ~(usbDevHw_ENDPT_STATUS_IN_XFER_DONE);
		status &= ~(usbDevHw_ENDPT_STATUS_IN_FIFO_EMPTY);

        if ( udcEpP->dma.done & usbDevHw_ENDPT_STATUS_IN_DMA_DONE )
        {
            /*
             * Have received both the IN_DMA_DONE and IN_XFER_DONE interrupts
             * for this request. OK to process the request (remove the request
             * and start the next one).
             */
            ReqXferProcess( udcEpP );
        }
        else
        {
            /*
             * Have not received the IN_DMA_DONE interrupt for this request.
             * Need to postpone processing of the request until the IN_DMA_DONE
             * interrupt occurs. See handling of IN_DMA_DONE status above.
             */
            udcEpP->dma.done = usbDevHw_ENDPT_STATUS_IN_XFER_DONE;
            BCM_DEBUG_DMA("%s: late IN DMA done pending\n", udcEpP->usb_ep.name );
        }
    }
	
	/* Clear the FIFO EMPTY bit, not to print error message */
	status &= ~(usbDevHw_ENDPT_STATUS_IN_FIFO_EMPTY);
    
	if ( status & usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL )
    {
        BCM_KERROR( "%s: DMA BUF NOT AVAIL\n", udcEpP->usb_ep.name );
        status &= ~(usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL);
        ReqXferProcess( udcEpP );
    }

    if ( status & usbDevHw_ENDPT_STATUS_DMA_ERROR )
    {
        status &= ~usbDevHw_ENDPT_STATUS_DMA_ERROR;
        BCM_KERROR( "%s: DMA ERROR\n", udcEpP->usb_ep.name );
        ReqXferError(udcEpP, -EIO);
    }

    if ( status )
    {
		BCM_KERROR( "exit: %s %s: unknown status=0x%x\n", __func__, udcEpP->usb_ep.name, status );
    }
}

void IrqEpOutStatusCheck( BCM_UDC_EP_t *udcEpP )
{
    uint32_t status;


    status = usbDevHw_EndptStatusActive( udcEpP->num, USB_DIR_OUT );
    usbDevHw_EndptStatusClear( udcEpP->num, USB_DIR_OUT, status );

	BCM_DEBUG_IRQ( "enter: %s: %s: status=0x%x\n", __func__, udcEpP->usb_ep.name, status );

    /*
     * Remove the Rx packet size field from the status. The datasheet states this field is not used
     * in DMA mode, but that is not true.
     */
    status &= usbDevHw_ENDPT_STATUS_ALL;

    if ( !status )
    {
        return;
    }

    if ( (udcEpP->dirn != USB_DIR_OUT) && (udcEpP->type != USB_ENDPOINT_XFER_CONTROL) )
    {
        BCM_KERROR( "%s: unexpected OUT interrupt\n", udcEpP->usb_ep.name );
        return;
    }

    if ( udcEpP->dirn != USB_DIR_OUT )
    {
        /* This probably should not be happening */
        BCM_KNOTICE( "%s: CTRL dirn IN\n", udcEpP->usb_ep.name );
    }

    if ( status & usbDevHw_ENDPT_STATUS_OUT_DMA_DATA_DONE )
    {
        status &= ~usbDevHw_ENDPT_STATUS_OUT_DMA_DATA_DONE;
        ReqXferProcess( udcEpP );
    }

    if ( status & usbDevHw_ENDPT_STATUS_OUT_DMA_SETUP_DONE )
    {
        status &= ~usbDevHw_ENDPT_STATUS_OUT_DMA_SETUP_DONE;
        IrqEpOutSetup(udcEpP);
    }

    if ( status & usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL )
    {
        /** @todo Verify under what situations this can happen. Should be when chain has emptied but last desc not reached  */
        /** @todo status for desc updates */

        status &= ~usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL;
        BCM_KERROR( "%s: DMA BUF NOT AVAIL\n", udcEpP->usb_ep.name );
        ReqXferProcess( udcEpP );
    }

    if ( status & usbDevHw_ENDPT_STATUS_DMA_ERROR )
    {
        status &= ~usbDevHw_ENDPT_STATUS_DMA_ERROR;
        BCM_KERROR( "%s: DMA ERROR\n", udcEpP->usb_ep.name );
        /** @todo merge XferError and XferProcess?? */
        ReqXferError(udcEpP, -EIO);
    }

    if ( status )
    {
        BCM_KERROR( "%s: unknown status=0x%x\n", udcEpP->usb_ep.name, status );
    }
}

void IrqEpOutSetup( BCM_UDC_EP_t *udcEpP )
{
    BCM_UDC_DMA_SETUP_t *dmaP;


    dmaP = &udcEpP->dma.virtualAddr->setup;

    if ( (dmaP->status & usbDevHw_REG_DMA_STATUS_BUF_MASK) != usbDevHw_REG_DMA_STATUS_BUF_DMA_DONE )
    {
        BCM_KERROR( "%s: unexpected DMA buf status=0x%x\n", udcEpP->usb_ep.name, (dmaP->status & usbDevHw_REG_DMA_STATUS_BUF_MASK) );

        /** @todo Make this use of DmaDumpEp() a BCM_DEBUG or DEBUG dependant?? */
        /* DmaDumpEp( udcEpP ); */
        CtrlEpSetupInit( udcEpP, ENOERROR );
    }
    else if ( (dmaP->status & usbDevHw_REG_DMA_STATUS_RX_MASK) != usbDevHw_REG_DMA_STATUS_RX_SUCCESS )
    {
        BCM_KERROR( "%s: unexpected DMA rx status=0x%x\n", udcEpP->usb_ep.name, (dmaP->status & usbDevHw_REG_DMA_STATUS_RX_MASK) );

        /** @todo Make this use of DmaDumpEp() a BCM_DEBUG or DEBUG dependant?? */
        /* DmaDumpEp( udcEpP ); */
        CtrlEpSetupInit( udcEpP, ENOERROR );
    }
    else
    {
        if ( udcEpP->num != 0 )
        {
            /** @todo Handle the cfg / intf / alt fields of the DMA status. This will only be any issue
             * once the Linux Gadget driver framework supports control transfers on an endpoint other
             * than 0.
             */

            BCM_KWARN( "%s: CTRL xfr support not complete\n", udcEpP->usb_ep.name );
        }
        /*
         * Take ownership of the descriptor while processing the request. Ownership will be released
         * when ready to Rx SETUP again.
         */

        dmaP->status = (dmaP->status & ~usbDevHw_REG_DMA_STATUS_BUF_MASK) | usbDevHw_REG_DMA_STATUS_BUF_HOST_BUSY;
        CtrlEpSetupRx( udcEpP, (struct usb_ctrlrequest *)&dmaP->data1 );
    }
}

/****************************************************************************
 * UDC Endpoint routines.
 *
 * UdcEpInit - Initialize endpoint structures
 * UdcEpCfg - Sets endpoint configuration in preparation for usage.
 *
 ***************************************************************************/

static int UdcEpCfg( BCM_UDC_EP_t *udcEpP, unsigned type, unsigned maxPktSize )
{
    BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
    BCM_DEBUG_EP( "%s: type=%u dirn=0x%x pkt=%u\n", udcEpP->usb_ep.name, type, udcEpP->dirn, maxPktSize );
    udcEpP->type = type;
    if ( UdcFifoRamAlloc( udcEpP, maxPktSize ) != ENOERROR )
    {
        return( -ENOSPC );
    }

    udcEpP->type = type;
    udcEpP->usb_ep.maxpacket = maxPktSize;
    usbDevHw_EndptOpsInit( udcEpP->num, udcEpP->type, udcEpP->dirn, maxPktSize );

    BCM_DEBUG_EP( "%s: type=%u maxPktSize=%u\n", udcEpP->usb_ep.name, type, maxPktSize );

    BCM_DEBUG_TRACE( "%s : exit\n", __func__ );

    return( ENOERROR );
}

static void UdcEpInit( BCM_UDC_t *udcP, unsigned num, const char *name, unsigned dirn )
{
    BCM_UDC_EP_t *udcEpP;

    BCM_DEBUG_TRACE( "%s: enter\n", name );
    BCM_DEBUG_EP( "%s: num=%u dir=%s\n", name, num, DIRN_STR(dirn) );

    if ((num >= BCM_UDC_EP_CNT) || (udcP == NULL)) 
    {
        BCM_KERROR( "Parameters error\n" );
        return;
    }

    udcEpP = &udcP->ep[num];

    /*
     * Initialize the endpoint attribute / control structure. Note that the UDC max packet
     * size is an indication of the hardware capabilities, not what is necessarily
     * configured and used by the endpoint. In order to provide the most flexibility on
     * how the endpoints are used, this is set to the maximum possible. When the Linux
     * Gadget usb_ep_autoconfig() looks for a suitable endpoint, it *may* check to ensure
     * the max size is adequate. There may or may not be enough FIFO RAM left to support an
     * endpoint configuration, even though the max size indicates otherwise, due to FIFO RAM
     * consumption by other endpoints. If this condition exists, an error will be returned
     * when the gadget driver tries to enable the endpoint. It is felt that doing things in
     * this manner is much easier than trying to predict and accomodate all the endpoint
     * usage scenarios by various gadget drivers, both existing and yet to be developed.
     */

    udcEpP->udcP = udcP;
    udcEpP->num = num;
    udcEpP->dirn = dirn;
    udcEpP->bEndpointAddress = num | dirn;
    udcEpP->maxPktSize = BCM_UDC_EP_MAX_PKT_SIZE;
    udcEpP->usb_ep.maxpacket = udcEpP->maxPktSize;
    udcEpP->stopped = 0;
    INIT_LIST_HEAD(&udcEpP->listQueue);

    udcEpP->usb_ep.name = name;
    udcEpP->usb_ep.ops = &bcm_udc_gadgetEpOps;
    INIT_LIST_HEAD(&udcEpP->usb_ep.ep_list);

    DmaEpInit( udcEpP );

    BCM_DEBUG_TRACE( "%s: exit\n", name );
}

/****************************************************************************
 * UDC FIFO RAM management routines.
 *
 *  The are two FIFO RAMs, one for IN and one for OUT. Each is shared amongst
 *  the endpoints and is dynamically allocated. In order to handle any excess
 *  allocation issues, we need to keep track of consumption. These are used
 *  as part of the Gadget endpoint enable / disable operations.
 *
 *  UdcFifoRamInit - Initializes the space available for allocation.
 *  UdcFifoRamAlloc - Allocates space for endpoint.
 *  UdcFifoRamFree - Fress space used by endpoint.
 *
 ***************************************************************************/

static void UdcFifoRamInit( BCM_UDC_t *udcP )
{
    udcP->rxFifoSpace = BCM_UDC_OUT_RX_FIFO_MEM_SIZE;
    udcP->txFifoSpace = BCM_UDC_IN_TX_FIFO_MEM_SIZE;
}

static int UdcFifoRamAlloc( BCM_UDC_EP_t *udcEpP, unsigned maxPktSize )
{
    unsigned rxCnt;
    unsigned txCnt;


	BCM_DEBUG_TRACE( "%s : enter\n", __func__ );
#define EP_DIRN_TYPE(d,t)   (((d) << 8) | (t))

    /** @todo Move this FIFO space requirement calculation to CSP? */
    switch ( EP_DIRN_TYPE(udcEpP->dirn, udcEpP->type) )
    {
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_BULK):
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_INT):
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_ISOC):
            rxCnt = usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            txCnt = 0;
            break;

        case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_BULK):
        case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_INT):
            rxCnt = 0;
            txCnt = usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            break;

        case EP_DIRN_TYPE(USB_DIR_IN, USB_ENDPOINT_XFER_ISOC):
            /* DWC UDC does double buffering for IN ISOC */
            rxCnt = 0;
            txCnt = 2 * usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            break;

        case EP_DIRN_TYPE(USB_DIR_IN,  USB_ENDPOINT_XFER_CONTROL):
        case EP_DIRN_TYPE(USB_DIR_OUT, USB_ENDPOINT_XFER_CONTROL):
            rxCnt = usbDevHw_FIFO_SIZE_UINT8(maxPktSize);
            txCnt = rxCnt;
            break;

        default:

            BCM_KERROR(  "%s: invalid EP attributes\n", udcEpP->usb_ep.name );
            BCM_DEBUG_TRACE( "exit: error\n" );
            return( -ENODEV );
    }

    BCM_DEBUG_EP( "rx req=%u free=%u: tx req=%u free=%u\n",
                    rxCnt, udcEpP->udcP->rxFifoSpace, txCnt, udcEpP->udcP->txFifoSpace );

    /** @todo change FifoSpace to uint32 units?? */
    if ( (udcEpP->udcP->rxFifoSpace < rxCnt) || (udcEpP->udcP->txFifoSpace < txCnt) )
    {
        BCM_DEBUG_TRACE( "exit: error\n" );
        return( -ENOSPC );
    }

    udcEpP->rxFifoSize = rxCnt;
    udcEpP->txFifoSize = txCnt;

#if usbDevHw_REG_MULTI_RX_FIFO
    udcEpP->udcP->rxFifoSpace -= rxCnt;
#endif
    udcEpP->udcP->txFifoSpace -= txCnt;

    BCM_DEBUG_TRACE( "exit: ok\n" );

    return( ENOERROR );
}

static void UdcFifoRamFree( BCM_UDC_EP_t *udcEpP )
{
#if usbDevHw_REG_MULTI_RX_FIFO
    udcEpP->udcP->rxFifoSpace += udcEpP->rxFifoSize;
#endif
    udcEpP->udcP->txFifoSpace += udcEpP->txFifoSize;

    udcEpP->rxFifoSize = 0;
    udcEpP->txFifoSize = 0;
}

static void UdcOpsDisconnect( BCM_UDC_t *udcP )
{
    BCM_UDC_EP_t    *udcEpP;
    int             num;

    for ( num = 0; num < BCM_UDC_EP_CNT; num++ ) {
        udcEpP=&udcP->ep[num];
        if ( udcEpP->dma.usb_req ) {
            // Flush DMA, reqeust still pending
            usbDevHw_EndptFifoFlushEnable( 0, usbDevHw_ENDPT_DIRN_IN );
            usbDevHw_EndptFifoFlushDisable( 0, usbDevHw_ENDPT_DIRN_IN );
            ReqXferProcess( udcEpP );
        }
    }
}

/***************************************************************************
* Endpoint request operations
***************************************************************************/

void ReqQueueFlush( BCM_UDC_EP_t *udcEpP, int status )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;


    BCM_DEBUG_TRACE( "%s: enter\n", udcEpP->usb_ep.name );
    BCM_DEBUG_REQ( "%s\n", udcEpP->usb_ep.name );

    udcEpP->stopped = 1;
    usbDevHw_EndptOpsFinis( udcEpP->num );

    while ( !list_empty( &udcEpP->listQueue ) )
    {
        udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );
        ReqXferDone( udcEpP, udcEpReqP, status );
    }
    udcEpP->dma.usb_req = NULL;

    BCM_DEBUG_TRACE( "%s: exit\n", udcEpP->usb_ep.name );
}


void ReqXferAdd( BCM_UDC_EP_t *udcEpP, BCM_UDC_EP_REQ_t *udcEpReqP )
{
	BCM_DEBUG_TRACE( "%s: enter\n", __func__);
    BCM_DEBUG_REQ( "%s: %s: stopped=%d\n", udcEpP->usb_ep.name, DIRN_STR(udcEpP->dirn), udcEpP->stopped );


    list_add_tail( &udcEpReqP->listNode, &udcEpP->listQueue );

    /** @todo Is this necessary?? Stopped happens as a result of a halt, complete(), dequeue(), nuke().
     * nuke() is called when ep disabled, during setup processing, and by udc_queisce(). The latter is
     * called during vbus state change (cable insert/remove), USB reset interrupt, and gadget deregister.
     */
    if ( udcEpP->stopped )
    {
        BCM_DEBUG_TRACE( "%s: exit\n", __func__ );
        return;
    }

    if ( (udcEpP->dirn == USB_DIR_IN) && (udcEpP->type == USB_ENDPOINT_XFER_ISOC) && udcEpP->dma.usb_req && (udcEpP->dma.frameNum == FRAME_NUM_INVALID) )
    {
        /*
         * Gadget has a request already queued, but still have not received an IN token from the host
         * and the interval window is not aligned. Queued packet is now very stale, so remove it.
         */

        DmaDataFinis( udcEpP );
        /** @todo Move set of udcEpP->dma.usb_req to DmaDataInit() and DmaDataFinis() routines. */
        udcEpP->dma.usb_req = NULL;
        ReqXferDone( udcEpP, list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode ), -EREMOTEIO );
    }

    /** @todo Current transfer is always the queue head. Do we need a separate pointer? Maybe just a pointer to usb_request
     * need to know if the queue head has already been loaded. Maybe that's the point of the "stopped".
     */
    if ( udcEpP->dma.usb_req )
    {
        BCM_DEBUG_REQ( "%s: busy\n", udcEpP->usb_ep.name );
    }

    #ifndef ISOC_IN_XFER_DELAY_DISABLED
    else if ( (udcEpP->dirn == USB_DIR_IN) && (udcEpP->type == USB_ENDPOINT_XFER_ISOC) && (udcEpP->dma.frameNum == FRAME_NUM_INVALID) )
    {
        /*
         * Delay any ISOC IN DMA operations until it is known what frame number the host
         * is going to start transfers with. Normally might just return requests until
         * this event occurs. However, the zero gadget does not submit requests based on
         * its own timer or similar, so if the request is returned right away things are
         * going to thrash, as another request will be immediately submitted.
         */

        BCM_DEBUG_ISOC( "%s: ISOC delay xfer start\n", udcEpP->usb_ep.name );
        udcEpP->dma.usb_req = &(list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode ))->usb_req;
        DmaDataInit( udcEpP );
        usbDevHw_EndptNakClear( udcEpP->num, udcEpP->dirn );
        usbDevHw_EndptIrqEnable( udcEpP->num, udcEpP->dirn );

    }
    #endif

    else
    {
        #ifdef ISOC_IN_XFER_DELAY_DISABLED
        if ( (udcEpP->dirn == USB_DIR_IN) && (udcEpP->type == USB_ENDPOINT_XFER_ISOC) && (udcEpP->dma.frameNum == FRAME_NUM_INVALID) )
        {
            /*
             * Try and start ISOC IN transfers without any regard to alignment to the
             * host. Unless the interval is set to its lowest possible value (a single
             * frame or uframe), transfers may not work until a IN token is received
             * from the host. See ENDPT_STATUS_IN_TOKEN_RX processing in IrqEpInStatusCheck().
             */

            udcEpP->dma.frameNum = usbDevHw_DeviceFrameNumLastRx();
            BCM_DEBUG_ISOC("%s: INIT: current frameNum=%d.%d\n", udcEpP->usb_ep.name, (udcEpP->dma.frameNum >> 3), (udcEpP->dma.frameNum & 0x7) );
        }
        #endif

        udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );
        udcEpP->dma.usb_req = &udcEpReqP->usb_req;
        DmaDataInit( udcEpP );
        BCM_DEBUG_REQ( "%s: begin: req=0x%p buf=0x%p len=%d actual=%d\n",
                        udcEpP->usb_ep.name, &udcEpReqP->usb_req, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.length, udcEpReqP->usb_req.actual );
		DmaDataAddReady( udcEpP );
        usbDevHw_EndptNakClear( udcEpP->num, udcEpP->dirn );
        usbDevHw_EndptDmaEnable( udcEpP->num, udcEpP->dirn );
#ifdef CONFIG_ARCH_IPROC
        /* needed for gadget commands to complete correctly - possible locking issue */
        mdelay(3);
#endif
    }

    BCM_DEBUG_TRACE( "%s: exit\n", __func__ );
}

void ReqXferDone( BCM_UDC_EP_t *udcEpP, BCM_UDC_EP_REQ_t *udcEpReqP, int status)
{
    unsigned stopped;


    BCM_DEBUG_TRACE( "%s: enter\n", __func__ );

    list_del_init( &udcEpReqP->listNode );

    if ( udcEpReqP->usb_req.status == -EINPROGRESS )
    {
        udcEpReqP->usb_req.status = status;
    }

    if ( udcEpReqP->dmaAligned )
    {
        udcEpReqP->dmaAligned = 0;
    }
    else if ( udcEpReqP->dmaMapped )
    {
        /*
         * A physical address was not provided for the DMA buffer. Release any resources
         * that were requested by the driver.
         */
	  BCM_DEBUG_DMA( "%s: udcEpReqP=0x%x buf=0x%x dma=0x%x actual=%u len=%u\n",
                    udcEpP->usb_ep.name, udcEpReqP, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.dma, udcEpReqP->usb_req.actual, udcEpReqP->usb_req.length);

        dma_unmap_single( udcEpP->udcP->gadget.dev.parent, udcEpReqP->usb_req.dma, udcEpReqP->usb_req.length,
                            (udcEpP->dirn == USB_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE) );

        udcEpReqP->dmaMapped = 0;
        udcEpReqP->usb_req.dma = DMA_ADDR_INVALID;
    }

    BCM_DEBUG_REQ( "%s: ready: req=0x%p buf=0x%p len=%d actual=%d\n",
                    udcEpP->usb_ep.name, &udcEpReqP->usb_req, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.length, udcEpReqP->usb_req.actual );

    /*
     * Disable DMA operations during completion callback. The callback may cause requests to be
     * added to the queue, but we don't want to change the state of the queue head.
     */

    stopped = udcEpP->stopped;
    udcEpP->stopped = 1;
    spin_unlock( &udcEpP->udcP->lock );
    udcEpReqP->usb_req.complete( &udcEpP->usb_ep, &udcEpReqP->usb_req );
    spin_lock( &udcEpP->udcP->lock );
    udcEpP->stopped = stopped;

    /** @todo May not have valid access to request any longer it has been freed... */
    BCM_DEBUG_REQ( "%s: complete: req=0x%p buf=0x%p\n", udcEpP->usb_ep.name, &udcEpReqP->usb_req, udcEpReqP->usb_req.buf );
    BCM_DEBUG_TRACE( "%s: exit\n", __func__ );
}

void ReqXferError( BCM_UDC_EP_t *udcEpP, int status )
{
    BCM_DEBUG_TRACE( "%s: enter\n", __func__ );
    BCM_DEBUG_REQ( "%s: status=%d\n", udcEpP->usb_ep.name, status );

    if ( !udcEpP->dma.usb_req )
    {
        BCM_KERROR( "%s: No request being transferred\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s: exit\n", __func__ );
        return;
    }

    /** @todo abort current DMA, start next transfer if there is one. */
    udcEpP->dma.usb_req->status = status;
    ReqXferProcess( udcEpP );

    BCM_DEBUG_TRACE( "%s: exit\n", __func__);
}

void ReqXferProcess( BCM_UDC_EP_t *udcEpP )
{
    BCM_UDC_EP_REQ_t *udcEpReqP;


    BCM_DEBUG_TRACE( "%s: enter\n", __func__ );
    BCM_DEBUG_REQ( "%s\n", udcEpP->usb_ep.name );


    /** @todo Current transfer is always the queue head. Do we need a separate pointer? Maybe just a pointer to usb_request */
    if ( !udcEpP->dma.usb_req )
    {
        BCM_KERROR( "%s: No request being transferred\n", udcEpP->usb_ep.name );
        BCM_DEBUG_TRACE( "%s: exit\n", __func__ );
        return;
    }

    usbDevHw_EndptDmaDisable( udcEpP->num, udcEpP->dirn );
    DmaDataRemoveDone( udcEpP );

    if ( udcEpP->dma.usb_req->status != -EINPROGRESS )
    {
        /*
         * Current transfer stage has finished. This may or may not be with error.
         * Complete the transfer as needed before starting the next one, if any.
         */

        DmaDataFinis( udcEpP );

        if ( (udcEpP->type == USB_ENDPOINT_XFER_CONTROL) && (udcEpP->dirn == USB_DIR_IN) && (udcEpP->dma.usb_req->status == ENOERROR) )
        {
            /*
             * For the status phase of control IN transfers, the hardware requires that an OUT DMA transfer
             * actually takes place. This should be just an OUT ZLP, and we will re-use the IN buffer that
             * just completed transfer for this purpose. There should be no harm in doing this, even if the
             * OUT status is more than a ZLP.
             */
            udcEpP->dirn = USB_DIR_OUT;
            DmaDataInit( udcEpP );
        }
        else
        {
            /*
             * All transfer stages have completed. Return the request to the gadget driver, and then
             * setup for the next transfer.
             */

            ReqXferDone( udcEpP, list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode ), ENOERROR );

            if ( udcEpP->type == USB_ENDPOINT_XFER_CONTROL )
            {
                CtrlEpSetupInit( udcEpP, ENOERROR );
            }

            if ( list_empty( &udcEpP->listQueue ) )
            {
                /** @todo Probably should more closely bind this to DmaDataFinis. */
                udcEpP->dma.usb_req = NULL;
            }
            else
            {
                udcEpReqP = list_first_entry( &udcEpP->listQueue, BCM_UDC_EP_REQ_t, listNode );
                udcEpP->dma.usb_req = &udcEpReqP->usb_req;
                DmaDataInit( udcEpP );
                BCM_DEBUG_REQ( "%s: begin: req=0x%p buf=0x%p len=%d actual=%d\n",
                                udcEpP->usb_ep.name, &udcEpReqP->usb_req, udcEpReqP->usb_req.buf, udcEpReqP->usb_req.length, udcEpReqP->usb_req.actual );
            }
        }
    }


    if ( udcEpP->dma.usb_req != NULL )
    {
        DmaDataAddReady( udcEpP );
        usbDevHw_EndptDmaEnable( udcEpP->num, udcEpP->dirn );
        usbDevHw_EndptNakClear( udcEpP->num, udcEpP->dirn );
    }

    BCM_DEBUG_TRACE( "%s: exit\n", __func__ );
}


/***************************************************************************
 * Linux proc file system functions
 ***************************************************************************/

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>

static const char bcm_udc_procFileName[] = "driver/" BCM_UDC_NAME;

static int ProcFileShow(struct seq_file *s, void *_)
{
    return( 0 );
}

static int ProcFileOpen(struct inode *inode, struct file *file)
{
    return( single_open(file, ProcFileShow, NULL) );
}

static struct file_operations bcm_udc_procFileOps =
{
    .open       = ProcFileOpen,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static void ProcFileCreate(void)
{
    struct proc_dir_entry *pde;

    pde = create_proc_entry (bcm_udc_procFileName, 0, NULL);
    if (pde)
    {
        pde->proc_fops = &bcm_udc_procFileOps;
    }
}

static void ProcFileRemove(void)
{
    remove_proc_entry(bcm_udc_procFileName, NULL);
}

#else

static void ProcFileCreate(void) {}
static void ProcFileRemove(void) {}

#endif
