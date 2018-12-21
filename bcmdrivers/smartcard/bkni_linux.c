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


/* standard defines */
#define BSTD_ENDIAN_BIG 4321
#define BSTD_ENDIAN_LITTLE 1234


//fye #include <linux/config.h>
#include <linux/kernel.h> /* printk */
#include <linux/slab.h> /* memset and friends */


#include <linux/wait.h>
#include <linux/delay.h>
//fye #include <linux/config.h>
#include <linux/version.h>
#if defined(MODVERSIONS)
#include <linux/modversions.h>
#endif
#include <linux/signal.h>
#include <linux/semaphore.h>
#include <linux/sched.h>

#include <linux/hardirq.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include "berr.h"
#include "bkni.h"

static int g_init = 0;
static int BKNI_P_signalPending(void);
static void BKNI_P_BlockSignals(sigset_t *pPrevMask);
static void BKNI_P_RestoreSignals(sigset_t *pPrevMask);


/***********************************************/
/* Options that may be diabled */
/***********************************************/
#define BKNI_MASK_INTERRUPTS 1


static struct task_struct *g_csOwner;

#define SET_CRITICAL() do { g_csOwner = current; } while (0)
#define CLEAR_CRITICAL() do { g_csOwner = NULL; } while (0)

#ifdef  BKNI_MASK_INTERRUPTS
#define CHECK_CRITICAL() ( g_csOwner == current || in_irq() )
#else
#define CHECK_CRITICAL() ( g_csOwner == current || in_interrupt() )
#endif

#ifdef BDBG_DEBUG_BUILD
/* It's ok to have this #if, but KNI cannot actually use DBG. */

#define ASSERT_CRITICAL() do \
{\
	if ( !CHECK_CRITICAL() )\
	{\
		BKNI_Printf("Error, must be in critical section to call %s\n", __FUNCTION__);\
		BKNI_Fail();\
	}\
} while (0)

#define ASSERT_NOT_CRITICAL() do \
{\
	if ( CHECK_CRITICAL() )\
	{\
		BKNI_Printf("Error, must not be in critical section to call %s\n", __FUNCTION__);\
		BKNI_Fail();\
	}\
} while (0)

#else

#define ASSERT_CRITICAL() (void)0
#define ASSERT_NOT_CRITICAL() (void)0

#endif

/* The standard Magnum BERR_TRACE uses DBG. That can't be used here. */
#define BKNI_ERR_TRACE(code) (BKNI_P_PrintError(__FILE__, __LINE__, #code, code))

BERR_Code
BKNI_P_PrintError(const char *file, unsigned lineno, const char *error, BERR_Code error_no)
{
    if (error_no != BERR_SUCCESS) {
        printk("!!!Error %s(%#x) at %s:%d\n", error, error_no, file, lineno);
    }
    return error_no;
}

static struct tasklet_struct *g_pIsrTasklet;
static struct tasklet_struct *g_pPendingTasklet;
static sigset_t g_blockedSignals;


struct BKNI_EventObj {
	wait_queue_head_t wq;
	volatile int eventset;
};

struct BKNI_MutexObj {
	struct semaphore sem;
};

BERR_Code BKNI_Init(void)
{
	if (!g_init) {
		int i;

		/* A list of terminal signals.  Any of these signals will trigger a
		   BERR_OS_ERROR if received while waiting or sleeping */
		static const int terminal_signals[ ] ={
			SIGCONT, /* Continue if stopped */
			SIGSTOP, /* Stop Stop process */
			SIGTSTP, /* Stop typed at tty */
			SIGKILL,
			SIGTERM,
			SIGINT,
			SIGHUP,
			SIGQUIT,
			SIGABRT,
			SIGILL,
			SIGSEGV,
			SIGBUS
		};

		g_init = 1;

		/* By default, block all signals except for the ones listed above */
		sigfillset(&g_blockedSignals);
		for(i=0;i<sizeof(terminal_signals)/sizeof(*terminal_signals);i++) {
			sigdelset(&g_blockedSignals, terminal_signals[i]);
		}

		return BERR_SUCCESS;
	}
	else
		return BERR_OS_ERROR;
}

void BKNI_Uninit(void)
{
	g_init = 0;
}

void BKNI_Fail(void)
{
	volatile int i=0;
	volatile int j=0;
	printk("BKNI_Fail: forcing oops\n");
	i = *(int *)i;
	i /= j;
	panic("BKNI_Fail: panic...");
}

uint32_t BKNI_RegRead32(void *in_channelHandle, uint32_t reg)
{
	return readl_relaxed(reg);
}

void BKNI_RegWrite32(void * in_channelHandle, uint32_t reg, uint32_t data)
{
	writel_relaxed(data, reg);
}

uint8_t BKNI_RegRead8(void *in_channelHandle, uint32_t reg)
{
	return __raw_readb(reg);
}

void BKNI_RegWrite8(void * in_channelHandle, uint32_t reg, uint32_t data)
{
	__raw_writeb(data, reg);
}

void *BKNI_Memchr(const void *s, int c, size_t n)
{
	const unsigned char *p = s;
	while (n-- != 0) {
		if ((unsigned char)c == *p++) {
			return (void *)(p-1);
		}
	}
	return NULL;
}


/**
* WARNING: This implementation is unfortunate, but it is safe, simple,
* and memory is only used when it is called. For a release build
* it is no doubt unacceptable.
**/
int BKNI_Vprintf(const char *fmt, va_list ap)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	/* 2.6 has a proper vprintk */
	return vprintk(fmt,ap);
#else
/**
 * WARNING: This implementation is unfortunate, but it is safe, simple,
 * and memory is only used when it is called.  You must be careful about
 * Linux's limited kernel stack size, so it is better to use static data
 * w/semaphore instead of putting such a large buffer on the stack.
**/
#define VPRINTF_BUFSIZE 1024
	static DECLARE_MUTEX(vprintf_mutex);
	static char buf[VPRINTF_BUFSIZE];
	static char isrbuf[VPRINTF_BUFSIZE];
	int rc;

	if ( CHECK_CRITICAL() )
	{
		/* ISR's need a different buffer to avoid a semaphore lock */
		vsnprintf(isrbuf, VPRINTF_BUFSIZE, fmt, ap);
		rc = printk("isr: %s", isrbuf);
	}
	else
	{
		if ( down_interruptible(&vprintf_mutex) )
		{
			/* don't print a warning from here, just schedule() to allow termination and retry */
			schedule();
			down(&vprintf_mutex);
		}

		vsnprintf(buf, VPRINTF_BUFSIZE, fmt, ap);
		rc = printk(buf);

		up(&vprintf_mutex);
	}

	return rc;
#endif
}

void BKNI_Delay_tagged(int microsec, const char *file, int line)
{
	udelay(microsec);
}

BERR_Code BKNI_Sleep_tagged(int millisec, const char *file, int line)
{
	unsigned long ticks;
	long rc;
	sigset_t mask;
	BERR_Code retval;

	ASSERT_NOT_CRITICAL();

	ticks = (millisec * HZ) / 1000;

	/* Each tick is 1 or 10ms, so we must wait at least that long */
	if (ticks == 0) {
		ticks = 1;
	}

	/* Block all non-terminal signals while sleeping */
	BKNI_P_BlockSignals(&mask);

	for(;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		rc = schedule_timeout(ticks);
		if (rc==0) {
			retval = BERR_SUCCESS;
			break;
		}
    	if (BKNI_P_signalPending()) {
			retval = BKNI_ERR_TRACE(BERR_OS_ERROR);
			break;
		}
		ticks = rc; /* keep sleeping */
	}

	/* Restore original signal mask */
	BKNI_P_RestoreSignals(&mask);

	return retval;
}

BERR_Code BKNI_CreateEvent_tagged(BKNI_EventHandle *p_event, const char *file, int line)
{
	BKNI_EventHandle event = BKNI_Malloc(sizeof(*event));

	ASSERT_NOT_CRITICAL();

	if (!event)
		return BERR_OS_ERROR;
	event->eventset = 0;

	init_waitqueue_head(&event->wq);
	*p_event = event;
	return BERR_SUCCESS;
}

void BKNI_DestroyEvent_tagged(BKNI_EventHandle event, const char *file, int line)
{
	ASSERT_NOT_CRITICAL();

	BKNI_Free(event);
}

void BKNI_SetEvent_tagged(BKNI_EventHandle event, const char *file, int line)
{
	event->eventset = 1;

    wake_up_interruptible(&event->wq);

}

void BKNI_ResetEvent_tagged(BKNI_EventHandle event, const char *file, int line)
{
	/* Ignore possible BERR_OS_ERROR, because we're not blocking. */
	BKNI_WaitForEvent(event, 0);
}

/**
* This will modify the caller's signal mask to block all signals
* except for the terminal signals listed in BKNI_Init().  This prevents
* user signals from interrupting magnum code, but they will be safely
* dispatched at the next opportunity.
**/
static void BKNI_P_BlockSignals(sigset_t *pPrevMask)
{
	spinlock_t *pSignalLock;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	/* 2.6 signal lock */
	pSignalLock = &current->sighand->siglock;
#else
	/* 2.4 signal lock */
	pSignalLock = &current->sigmask_lock;
#endif

	/* Lock the signal structs */
	spin_lock(pSignalLock);

	/* Save current signals */
	memcpy(pPrevMask, &current->blocked, sizeof(sigset_t));
	/* Set to block all but the terminal signals */
	memcpy(&current->blocked, &g_blockedSignals, sizeof(sigset_t));
	/* Must be called after manipulating blocked signals */
	recalc_sigpending();

	/* Release the lock */
	spin_unlock(pSignalLock);
}

/**
* This will restore the original signal mask saved by BKNI_P_BlockSignals()
**/
static void BKNI_P_RestoreSignals(sigset_t *pPrevMask)
{
	spinlock_t *pSignalLock;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
	/* 2.6 signal lock */
	pSignalLock = &current->sighand->siglock;
#else
	/* 2.4 signal lock */
	pSignalLock = &current->sigmask_lock;
#endif

	/* Lock the signal structs */
	spin_lock(pSignalLock);

	/* Restore signals */
	memcpy(&current->blocked, pPrevMask, sizeof(sigset_t));
	/* Must be called after manipulating blocked signals */
	recalc_sigpending();

	/* Release the lock */
	spin_unlock(pSignalLock);
}

/**
* This will return 1 if a signal is pending.  Previous implementations
* of this function would dequeue all non-terminal signals to allow the
* caller to continue waiting.  That approach had the drawback of consuming
* any non-terminal signals and preventing the application from using them.
* The new implementation assumes that the caller has called BKNI_P_BlockSignals()
* prior to waiting on an event or timeout and therefore can only be interrupted by
* a terminal signal.  When the original signal mask is restored via BKNI_P_RestoreSignals,
* the user signals will be dispatched again.
**/
static int BKNI_P_signalPending(void)
{
	if (signal_pending(current)) {
		return 1;
	}
	return 0;
}


/**
* This WaitForEvent consumes all signals. This is allowed by KNI impl's, but not allowed
* for Magnum code to us.
**/
BERR_Code BKNI_WaitForEvent_tagged(BKNI_EventHandle event, int timeoutMsec, const char *file, int line)
{
	BERR_Code result = BERR_TIMEOUT;
	sigset_t mask;

	if ( timeoutMsec )
	{
		ASSERT_NOT_CRITICAL();
	}
	else if ( CHECK_CRITICAL() )
	{
		/* Don't mess with current or wait queues from an ISR */
		if ( event->eventset )
		{
			event->eventset = 0;
			return BERR_SUCCESS;
		}
		else
		{
			return BERR_TIMEOUT;
		}
	}

	/* This is used to achieve consistency between different OS's. */
	if (timeoutMsec>0 && timeoutMsec<30)
	{
		/* wait at least 30 msec */
		timeoutMsec = 30;
	}

	BKNI_P_BlockSignals(&mask);

	if (BKNI_P_signalPending())
		result = BERR_OS_ERROR;
	else
	{
		wait_queue_t wqe;
		unsigned long ticks;

		init_waitqueue_entry(&wqe, current);
		add_wait_queue(&event->wq, &wqe);

		if (timeoutMsec == BKNI_INFINITE)
			ticks = MAX_SCHEDULE_TIMEOUT;
		else if (timeoutMsec)
			ticks = (timeoutMsec * HZ) / 1000;
		else
			ticks = 0;

		/* Need to repeat the sleep until the entire timeout
		is consumed, or event occurs, or a true fatal signal is detected.
		It's possible to be signalled and yet keep going. */
		for ( ;; )
		{
			/* Be sure to go half asleep before checking condition. */
    		/* Otherwise we have a race condition between when we   */
    		/* check the condition and when we call schedule().     */
		 	set_current_state(TASK_INTERRUPTIBLE);

		 	if (event->eventset)
			{
				result = BERR_SUCCESS;
				event->eventset = 0;
				break;
			}
			else if (!ticks)
			{
	       	 	result = BERR_TIMEOUT;
				break;
			}
			else
			{
				/* When SetEvent is called, event process on event->wq is woken. */
				ticks = schedule_timeout(ticks);
		    	if (BKNI_P_signalPending())
				{
					result = BERR_OS_ERROR;
					break;
				}
			}
		}

    	set_current_state(TASK_RUNNING);
	    remove_wait_queue(&event->wq, &wqe);
	}

	BKNI_P_RestoreSignals(&mask);

	return result;
}

//static spinlock_t g_criticalSection = SPIN_LOCK_UNLOCKED;
static DEFINE_SPINLOCK(g_criticalSection);

#ifdef BKNI_MASK_INTERRUPTS
static unsigned long g_flags;
#endif

void BKNI_EnterCriticalSection_tagged(const char *file, int line)
{
	ASSERT_NOT_CRITICAL();

	if ( g_pIsrTasklet )
	{
		spin_lock(&g_criticalSection);
		tasklet_disable(g_pIsrTasklet);
	}
	else
	{
		#ifdef BKNI_MASK_INTERRUPTS
		spin_lock_irqsave(&g_criticalSection, g_flags);
		#else
		spin_lock_bh(&g_criticalSection);
		#endif
	}

	SET_CRITICAL();
}

void BKNI_LeaveCriticalSection_tagged(const char *file, int line)
{
	struct tasklet_struct *pIsrTasklet;

	ASSERT_CRITICAL();

	CLEAR_CRITICAL();

	/* Store tasklet and replace with any possible changes */
	pIsrTasklet = g_pIsrTasklet;
	g_pIsrTasklet = g_pPendingTasklet;

	/* Re-enable interrupts in the same way they were disabled */
	if ( pIsrTasklet )
	{
		tasklet_enable(pIsrTasklet);
		spin_unlock(&g_criticalSection);
	}
	else
	{
		#ifdef BKNI_MASK_INTERRUPTS
		spin_unlock_irqrestore(&g_criticalSection, g_flags);
		#else
		spin_unlock_bh(&g_criticalSection);
		#endif
	}
}

void *BKNI_Malloc_tagged(size_t size, const char *file, int line)
{
	ASSERT_NOT_CRITICAL();

	return kmalloc(size, GFP_KERNEL);
}

void BKNI_Free_tagged(void *mem, const char *file, int line)
{
	ASSERT_NOT_CRITICAL();

	if (mem) /* Magnum code isn't allowed to pass NULL, but the behavior is undefined */
		kfree(mem);
}


void *BKNI_Memset(void *mem, int ch, size_t n)
{
	return memset(mem, ch, n);
}

void *BKNI_Memcpy(void *dest, const void *src, size_t n)
{
	return memcpy(dest, src, n);
}

int BKNI_Memcmp(const void *s1, const void *s2, size_t n)
{
	return memcmp(s1, s2, n);
}

void *BKNI_Memmove(void *dest, const void *src, size_t n)
{
	return memmove(dest, src, n);
}

int BKNI_Printf(const char *fmt, ...)
{
	va_list ap;
	int rc;
	va_start(ap, fmt);
	rc = BKNI_Vprintf(fmt, ap);
	va_end(ap);
	return rc;
}

int BKNI_Snprintf(char *s, size_t n, const char *fmt, ...)
{
	va_list ap;
	int rc;
	va_start(ap, fmt);
	rc = vsnprintf(s, n, fmt, ap);
	va_end(ap);
	return rc;
}

int BKNI_Vsnprintf(char *s, size_t n, const char *fmt, va_list ap)
{
	return vsnprintf(s, n, fmt, ap);
}
