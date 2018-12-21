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

/****************************************************************************/
/**
*  @file    usbDevHw.h
*
*  @brief   API definitions for low level USB device controller drivers.
*
*  @note
*/
/****************************************************************************/

#ifndef USBDEVHW_H
#define USBDEVHW_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ---- Include Files ---------------------------------------------------- */

#include "stdint.h"

/* ---- Public Constants and Types --------------------------------------- */
//#define usbDevHw_REG_MULTI_RX_FIFO 1
/*
 * The following are to be defined in usbDevHw_def.h, using bit flags. This
 * is to allow IRQs to be OR'd together if desired.
 */
/*
 * usbDevHw_DEVICE_IRQ_REMOTEWAKEUP_DELTA
 * usbDevHw_DEVICE_IRQ_SPEED_ENUM_DONE
 * usbDevHw_DEVICE_IRQ_SOF_DETECTED
 * usbDevHw_DEVICE_IRQ_BUS_SUSPEND
 * usbDevHw_DEVICE_IRQ_BUS_RESET
 * usbDevHw_DEVICE_IRQ_BUS_IDLE
 * usbDevHw_DEVICE_IRQ_SET_INTF
 * usbDevHw_DEVICE_IRQ_SET_CFG
 */
#define usbDevHw_DEVICE_IRQ_ALL            (usbDevHw_DEVICE_IRQ_REMOTEWAKEUP_DELTA | \
                                            usbDevHw_DEVICE_IRQ_SPEED_ENUM_DONE    | \
                                            usbDevHw_DEVICE_IRQ_SOF_DETECTED       | \
                                            usbDevHw_DEVICE_IRQ_BUS_SUSPEND        | \
                                            usbDevHw_DEVICE_IRQ_BUS_RESET          | \
                                            usbDevHw_DEVICE_IRQ_BUS_IDLE           | \
                                            usbDevHw_DEVICE_IRQ_SET_INTF           | \
                                            usbDevHw_DEVICE_IRQ_SET_CFG            )

#define usbDevHw_DEVICE_SPEED_UNKNOWN       0
#define usbDevHw_DEVICE_SPEED_LOW           1
#define usbDevHw_DEVICE_SPEED_FULL          2
#define usbDevHw_DEVICE_SPEED_HIGH          3

/*
 * The following are to be defined in usbDevHw_def.h, using bit flags. This
 * is to allow endpoint status to be OR'd together if desired.
 */
/*
 * usbDevHw_ENDPT_STATUS_DMA_ERROR
 * usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL
 * usbDevHw_ENDPT_STATUS_IN_TOKEN_RX
 * usbDevHw_ENDPT_STATUS_IN_DMA_DONE
 * usbDevHw_ENDPT_STATUS_IN_XFER_DONE
 * usbDevHw_ENDPT_STATUS_OUT_DMA_DATA_DONE
 * usbDevHw_ENDPT_STATUS_OUT_DMA_SETUP_DONE
 */
#define usbDevHw_ENDPT_STATUS_ALL          (usbDevHw_ENDPT_STATUS_DMA_ERROR         | \
                                            usbDevHw_ENDPT_STATUS_DMA_BUF_NOT_AVAIL | \
                                            usbDevHw_ENDPT_STATUS_IN_TOKEN_RX       | \
                                            usbDevHw_ENDPT_STATUS_IN_DMA_DONE       | \
                                            usbDevHw_ENDPT_STATUS_IN_XFER_DONE      | \
                                            usbDevHw_ENDPT_STATUS_OUT_DMA_DATA_DONE | \
                                            usbDevHw_ENDPT_STATUS_OUT_DMA_SETUP_DONE )

/*
 * The ENDPT_DIRN enumeration is the same as the USB protocol endpoint descriptors'
 * bEndpointAddress field and control requests bRequestType. This is intentional.
 */
#define usbDevHw_ENDPT_DIRN_IN              0x80
#define usbDevHw_ENDPT_DIRN_OUT             0x00
#define usbDevHw_ENDPT_DIRN_MASK            0x80

/*
 * The ENDPT_TYPE enumeration is the same as the USB protocol endpoint descriptors'
 * bmAttributes field. This is intentional.
 */
#define usbDevHw_ENDPT_TYPE_CTRL            0
#define usbDevHw_ENDPT_TYPE_ISOC            1
#define usbDevHw_ENDPT_TYPE_BULK            2
#define usbDevHw_ENDPT_TYPE_INTR            3
#define usbDevHw_ENDPT_TYPE_MASK            0x03


/* ---- Public Variable Externs ------------------------------------------ */
/* ---- Public Function Prototypes --------------------------------------- */

/****************************************************************************/
/**
*  @brief   Finalize (terminate) / Initialize device controller operations
*
*  @return
*   0  : success
*   -1 : error
*/
/****************************************************************************/
static inline void usbDevHw_OpsFinis( void );
static inline void usbDevHw_OpsInit( void );

/****************************************************************************/
/**
*  @brief   Connect / Disconnect to USB BUS
*/
/****************************************************************************/
static inline void usbDevHw_DeviceBusConnect( void );
static inline void usbDevHw_DeviceBusDisconnect( void );

/****************************************************************************/
/**
*  @brief   USB BUS suspend status
*
*  @return
*   true  : BUS is in suspend state
*   false : BUS is not in suspend state
*/
/****************************************************************************/
static inline bool usbDevHw_DeviceBusSuspended( void );

/****************************************************************************/
/**
*  @brief   Retrieve setting numbers from last Rx'd SET_CONFIGURATION or
*           SET_INTERFACE request
*
*  @return
*   Setting Number
*/
/****************************************************************************/
static inline unsigned usbDevHw_DeviceAltNum( void );
static inline unsigned usbDevHw_DeviceCfgNum( void );
static inline unsigned usbDevHw_DeviceIntfNum( void );

/****************************************************************************/
/**
*  @brief   Disable / Enable DMA operations at the device level (all endpoints)
*/
/****************************************************************************/
static inline void usbDevHw_DeviceDmaDisable( void );
static inline void usbDevHw_DeviceDmaEnable( void );

/****************************************************************************/
/**
*  @brief   Retrieve Frame number contained in last Rx'd SOF packet
*
*  @return
*
*   Frame Number in the following format.
*       bits[13:3] milli-second frame number
*       bits[2:0] micro-frame number
*
*  @note
*
*   For full and low speed connections, the microframe number will be zero.
*/
/****************************************************************************/
static inline unsigned usbDevHw_DeviceFrameNumLastRx( void );

/****************************************************************************/
/**
*  @brief   Device level interrupt operations
*
*  @note
*       Use the usbDevHw_DEVICE_IRQ_xxx definitions with these routines. These
*       definitions are bit-wise, and allow operations on multiple interrupts
*       by OR'ing the definitions together.
*
*       DeviceIrqClear(), DeviceIrqDisable(), DeviceIrqEnable() use their mask
*       parameter to operate only on the interrupts set in the mask. E.g.
*
*           DeviceIrqEnable( DEVICE_IRQ_SET_INTF );
*           DeviceIrqEnable( DEVICE_IRQ_SET_CFG );
*
*       and
*
*           DeviceIrqEnable( DEVICE_IRQ_SET_INTF | DEVICE_IRQ_SET_CFG );
*
*       are equivalent.
*
*       DeviceIrqMask() returns a mask of all the interrupts that are enabled.
*       DeviceIrqStatus() returns a mask of all the interrupts that have an active status.
*/
/****************************************************************************/
static inline uint32_t usbDevHw_DeviceIrqActive( void );
static inline void usbDevHw_DeviceIrqClear( uint32_t mask );
static inline void usbDevHw_DeviceIrqDisable( uint32_t mask );
static inline void usbDevHw_DeviceIrqEnable( uint32_t mask );
static inline uint32_t usbDevHw_DeviceIrqMask( void );

/****************************************************************************/
/**
*  @brief   Disable / Enable NAK responses for all OUT endpoints.
*
*  @todo TDB usefullness of this. Have Endpt specific method for this.
*/
/****************************************************************************/
static inline void usbDevHw_DeviceNakAllOutEptDisable( void );
static inline void usbDevHw_DeviceNakAllOutEptEnable( void );

/****************************************************************************/
/**
*  @brief   PHY error detected
*/
/****************************************************************************/
static inline bool usbDevHw_DevicePhyErrorDetected( void );

/****************************************************************************/
/**
*  @brief   Remote Wakeup operations.
*
*       DeviceRemoteWakeupEnable() and DeviceRemoteWakeupDisable() are used to
*       specify device if is going to attempt this.
*
*       DeviceRemoteWakeupAllowed() indicates if host has enabled this feature.
*       The associated DEVICE_IRQ_REMOTEWAKEUP_DELTA can be used to determine
*       changes to the status of this feature.
*
*       DeviceRemoteWakeupStart(); delayMsec(1); DeviceRemoteWakeupStop(); is
*       used for controlling the wakeup signalling.
*/
/****************************************************************************/
static inline bool usbDevHw_DeviceRemoteWakeupAllowed( void );
static inline void usbDevHw_DeviceRemoteWakeupDisable( void );
static inline void usbDevHw_DeviceRemoteWakeupEnable( void );
static inline void usbDevHw_DeviceRemoteWakeupStart( void );
static inline void usbDevHw_DeviceRemoteWakeupStop( void );

/****************************************************************************/
/**
*  @brief   Control whether or not device advertises itself as self-powered.
*/
/****************************************************************************/
static inline void usbDevHw_DeviceSelfPwrDisable( void );
static inline void usbDevHw_DeviceSelfPwrEnable( void );

/****************************************************************************/
/**
*  @brief   Control whether or not device SET DESCRIPTOR support is enabled.
*
*       If disabled, STALL will be issued upon receipt of a SET DESCRIPTOR request.
*/
/****************************************************************************/
static inline void usbDevHw_DeviceSetDescriptorDisable( void );
static inline void usbDevHw_DeviceSetDescriptorEnable( void );

/****************************************************************************/
/**
*  @brief   Device SET configuration or SET interface has completed.
*
*       If disabled, STALL will be issued upon receipt of a SET DESCRIPTOR request.
*/
/****************************************************************************/
static inline void usbDevHw_DeviceSetupDone( void );

/****************************************************************************/
/**
*  @brief   Link speed routines.
*
*       Use the usbDevHw_DEVICE_SPEED_xxx definitions with these routines. These
*
*       DeviceSpeedRequested() indicates the desired link speed.
*
*       DeviceSpeedEnumerated() returns the speed negotiated with the host.
*       The associated DEVICE_IRQ_SPEED_ENUM_DONE can be used to determine
*       when speed negotiation has completed.
*/
/****************************************************************************/
static inline unsigned usbDevHw_DeviceSpeedEnumerated( void );
static inline void usbDevHw_DeviceSpeedRequested( unsigned speed );

/****************************************************************************/
/**
*  @brief   Finalize (terminate) / Initialize Endpoint operations
*
*  @param   num - Endpoint number
*  @param   dirn - Endpoint direction. See ENDPT_DIRN_xxx definitions
*  @param   dirn - Endpoint type. See ENDPT_TYPE_xxx definitions
*  @param   dirn - Endpoint max packet size.
*/
/****************************************************************************/
static inline void usbDevHw_EndptOpsInit( unsigned num, unsigned type, unsigned dirn, unsigned maxPktSize );
static inline void usbDevHw_EndptOpsFinis( unsigned num );

/****************************************************************************/
/**
*  @brief   Endpoint Configuration / Interface / Alternate number operations
*
*  @param   num - Endpoint number
*  @param   cfg - Configuration number
*  @param   intf - Interface number
*  @param   alt - Alternate number
*/
/****************************************************************************/
static inline void usbDevHw_EndptAltSet( unsigned num, unsigned alt );
static inline void usbDevHw_EndptCfgSet( unsigned num, unsigned cfg );
static inline void usbDevHw_EndptIntfSet( unsigned num, unsigned intf );

/****************************************************************************/
/**
*  @brief   Endpoint DMA routines
*
*  @param   num - Endpoint number
*  @param   addr - physical address of buffer or descriptor
*/
/****************************************************************************/
static inline void usbDevHw_EndptDmaDisable( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptDmaEnable( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptDmaSetupBufAddrSet( unsigned num, unsigned dirn, void *addr );
static inline void usbDevHw_EndptDmaDataDescAddrSet( unsigned num, unsigned dirn, void *addr );

/****************************************************************************/
/**
*  @brief   Endpoint FIFO routines
*
*  @param   num - Endpoint number
*
*  @note    The flush operation is a state. Once enabled, FIFO contents are discared
*           until disabled. Usually enable upon endpoint termination or error, and
*           then disable once operations are to resume normally.
*/
/****************************************************************************/
static inline bool usbDevHw_EndptFifoEmpty( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptFifoFlushDisable( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptFifoFlushEnable( unsigned num, unsigned dirn );

/****************************************************************************/
/**
*  @brief   Endpoint Frame Number routines
*
*  @param   num - Endpoint number
*
*  @return  Frame number of last packet received on the endpoint, and in the following format.
*               bits[13:3] milli-second frame number
*               bits[2:0] micro-frame number
*
*  @note    Really only applicable to OUT endpoints. IN will always return 0.
*/
/****************************************************************************/
static inline unsigned usbDevHw_EndptFrameNumGet( unsigned num, unsigned dirn );

/****************************************************************************/
/**
*  @brief   Endpoint IRQ / status routines
*
*  @param   num - Endpoint number
*
*  @note
*
*       Cannot set specific status for Endpoint interrupts. Can only do operations
*       in a global sense. Once an interrupt occurs for an endpoint, the endpoint
*       status has to be checked for the particular type of interrupt that occurred.
*
*       The usbDevHw_EndptIrqEnable() and usbDevHw_EndptIrqDisable() are used for
*       operations on a specific endpoint. These routines may or may not be used in
*       the context of interrupt processing.
*
*       Use the usbDevHw_EndptIrqListXxx() routines for operations using a bit-wise
*       list of endpoints (bit 0 for endpoint 0, etc.). Typical use would be for
*       interrupt processing.
*
*       Use the usbDevHw_ENDPT_STATUS_xxx definitions with the status routines. These
*       definitions are bit-wise, and allow operations on multiple conditions
*       by OR'ing the definitions together.
*
*       Example abstract usage is the following.
*
*           list = usbDevHw_EndptIrqListActive( dirn );
*           usbDevHw_EndptIrqListClear( dirn, list );
*           for endpoint bit num set in list
*               status = usbDevHw_EndptStatusState( num, dirn );
*               usbDevHw_EndptStatusClear( num, dirn, status );
*               for status bit num set in status
*                   status specific processing
*/
/****************************************************************************/
static inline void usbDevHw_EndptIrqClear( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptIrqDisable( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptIrqEnable( unsigned num, unsigned dirn );

static inline void usbDevHw_EndptIrqListClear( unsigned dirn, uint32_t mask );
static inline uint32_t usbDevHw_EndptIrqListActive( unsigned dirn );

static inline void usbDevHw_EndptStatusClear( unsigned num, unsigned dirn, uint32_t mask );
static inline uint32_t usbDevHw_EndptStatusActive( unsigned num, unsigned dirn );

/****************************************************************************/
/**
*  @brief   Endpoint NAK routines
*
*  @param   num - Endpoint number
*
*  @note    A NAK response can be enabled by the application by the EndptNakEnable().
*           The EndptNakInProgress() is used to determine if the controller is
*           currently actively sending NAKs. This may have been a result of the
*           EndptNakEnable() or automatically by the controller under certain
*           conditions. The EndptNakClear() must be used to terminate the NAKs.
*/
/****************************************************************************/
static inline void usbDevHw_EndptNakClear( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptNakEnable( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptNakDisable( unsigned num, unsigned dirn );
static inline bool usbDevHw_EndptNakInProgress( unsigned num, unsigned dirn );

/****************************************************************************/
/**
*  @brief   Endpoint Stall routines
*
*           Disable / Enable STALL responses (halt feature) on a given endpoint.
*
*  @param   num - Endpoint number
*/
/****************************************************************************/
static inline void usbDevHw_EndptStallDisable( unsigned num, unsigned dirn );
static inline void usbDevHw_EndptStallEnable( unsigned num, unsigned dirn );

/****************************************************************************/
/**
*  @brief   Prints USB device controller configuration and register info
*
*/
/****************************************************************************/
static inline void usbDevHw_PrintInfo
(
    int (*printFP) (const char *, ...)  /* [IN] printf routine to use for output */
);

/* ---- Chip specific definitions ---------------------------------------- */

/*
 * The following include will provide any chip-specific definitions needed to
 * implement the API described in this header file, e.g. inline functions.
 * This has to be included last to ensure all prototypes have been defined.
 * Some inline function implementations may use other inline functions, and
 * the functions have to a prototype defined before they can be referenced.
 */
#include "usbDevHw_def.h"


#ifdef __cplusplus
extern "C"
}
#endif

#endif /* USBDEVHW_H */
