/*****************************************************************************
 * Copyright (c) 2009 Broadcom Corporation.  All rights reserved.
 *
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distributed
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 * 1. This program, including its structure, sequence and organization,
 *    constitutes the valuable trade secrets of Broadcom, and you shall use
 *    all reasonable efforts to protect the confidentiality thereof, and to
 *    use this information only in connection with your use of Broadcom
 *    integrated circuit products.
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
 *    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
 *    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
 *    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND ALL
 *    IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT, FITNESS
 *    FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR COMPLETENESS,
 *    QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE TO DESCRIPTION. YOU
 *    ASSUME THE ENTIRE RISK ARISING OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR ITS
 *    LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL, INDIRECT,
 *    OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY WAY RELATING TO
 *    YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN IF BROADCOM HAS BEEN
 *    ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii) ANY AMOUNT IN EXCESS
 *    OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF OR U.S. $1, WHICHEVER
 *    IS GREATER. THESE LIMITATIONS SHALL APPLY NOTWITHSTANDING ANY FAILURE OF
 *    ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 *****************************************************************************/

#ifndef BCMTIMER_H
#define BCMTIMER_H

//#include "bcmmips.h"
//#include "bchp_common.h"
#include "bchp_timer.h"


#if !defined _ASMLANGUAGE
#if __cplusplus
extern "C" {
#endif
#endif

#define BCM_PHYS_TO_K1(x)    ((x) | 0xa0000000)
#define BCHP_PHYSICAL_OFFSET 0x10000000

#define TIMR_ADR_BASE  BCM_PHYS_TO_K1(BCHP_PHYSICAL_OFFSET+BCHP_TIMER_TIMER_IS)

#define TIMER_TIMER_IS        0x00
#define TIMER_TIMER_IE0       0x04
#define TIMER_TIMER0_CTRL     0x08
#define TIMER_TIMER1_CTRL     0x0c
#define TIMER_TIMER2_CTRL     0x10
#define TIMER_TIMER3_CTRL     0x14
#define TIMER_TIMER0_STAT     0x18
#define TIMER_TIMER1_STAT     0x1c
#define TIMER_TIMER2_STAT     0x20
#define TIMER_TIMER3_STAT     0x24
#define TIMER_WDTIMEOUT       0x28
#define TIMER_WDCMD           0x2c
#define TIMER_WDCHIPRST_CNT   0x30
#define TIMER_WDCRS           0x34
#define TIMER_TIMER_IE1       0x38

/* Timer interrupt register */
#define TIMER_INT_TIMER0    0x01
#define TIMER_INT_TIMER1    0x02
#define TIMER_INT_TIMER2    0x04
#define TIMER_INT_TIMER3    0x08
#define TIMER_INT_WDINT     0x10

/* Timer control register */
#define TIMER_CTL_ENA               0x80000000
#define TIMER_CTL_MODE              0x40000000
#define TIMER_CTL_TIMEOUT_MASK      0x3FFFFFFF

/* Timer status register */
#define TIMER_STS_COUNTER_MASK      0x3FFFFFFF

#if !defined _ASMLANGUAGE



typedef struct Timer {
  unsigned long  TimerInts;			/* 0x00 */
#define TIMER0          0x01
#define TIMER1          0x02
#define TIMER2          0x04
#define TIMER3          0x08
#define WATCHDOG        0x10
  unsigned long  TimerMask;			/* 0x04 */
#define TIMER0EN        0x01
#define TIMER1EN        0x02
#define TIMER2EN        0x04
#define TIMER3EN        0x08
  unsigned long  TimerCtl0;			/* 0x08 */
  unsigned long  TimerCtl1;			/* 0x0c */
  unsigned long  TimerCtl2;			/* 0x10 */
  unsigned long  TimerCtl3;			/* 0x14 */
#define TIMERENABLE     0x80000000
#define RSTCNTCLR       0x40000000
  unsigned long  TimerCnt0;			/* 0x18 */
  unsigned long  TimerCnt1;			/* 0x1c */
  unsigned long  TimerCnt2;			/* 0x20 */
  unsigned long  TimerCnt3;			/* 0x24 */
  unsigned long  WatchDogDefCount;	/* 0x28 */

  /* Write 0xff00 0x00ff to Start timer
   * Write 0xee00 0x00ee to Stop and re-load default count
   * Read from this register returns current watch dog count
   */
  unsigned long  WatchDogCtl;		/* 0x2c */

  /* Number of 40-MHz ticks for WD Reset pulse to last */
  unsigned long  WDResetCount;		/* 0x30 */
  unsigned long  WDResetStatus;		/* 0x34 */
} Timer;

#define TIMER ((volatile Timer * const)(TIMR_ADR_BASE))




#if __cplusplus
}
#endif
#endif

#endif
