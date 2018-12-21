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
#ifndef __PLAT_IPROC_TIMER_H
#define __PLAT_IPROC_TIMER_H

/* Timer module specific data structures */
enum timer_rate {
    KHZ_32 = 0,
    MHZ_1,
    MHZ_19_5,
};

struct iproc_timer;

/* Channel specific data structures */
typedef int (*intr_callback)(void *data);

enum timer_mode {
    MODE_PERIODIC=0,
    MODE_ONESHOT,
};

struct timer_ch_cfg {
    void *arg;
    enum timer_mode mode;
    intr_callback cb;
    unsigned long reload;    /* Holds the reload value in 
                 * case of periodic timers 
                 */
};

/* Timer Module related APIs */

/*
 * USAGE OF THIS APIs
 * ------------------
 * From the board specific file, the iproc_timer_modules_init will be called 
 * After that it will call the init function of timer.c and will pass the
 * following information in a platform structure
 * 1) Timer name to be used as system timer
 * 2) Frequency to be configured for system timer
 * 3) The channel of the timer to use as clock source (optional)
 * 4) The channel of the timer to use as clock event (optional)
 *
 * from the init function of timer.c iproc_timer_modules_set_rate will be called
 * to set the system timer frequency. 
 * Then the appropriate channels would be setup for clock source/event by 
 * calling iproc_timer_request()
 */

/*
 *  iproc_timer_modules_init - Initialize the data structures
 *  that depcits the iProc timer modules
 */
void iproc_timer_modules_init (void);

/*
 * iproc_timer_module_set_rate - Set the speed in which a timer module should count
 * name - Name of the Timer to configure
 * rate - Speed 
 */
int iproc_timer_module_set_rate(char* name, enum timer_rate);

/* 
 * iproc_timer_module_get_rate - Get the speed in which a timer module is running
 * name - Name of the Timer module 
 */
int iproc_timer_module_get_rate (char* name);


/* Channel/Timer related APIs */
/*
 *  iproc_timer_request - Get access to a channel in the given timer
 *  name - Name of the Timer module
 *  channel - Channel number requested. If this is -1 then by default
 *            the next available channel will be returned
 */
struct iproc_timer* iproc_timer_request(char* name, int channel);

/*
 *  iproc_timer_config - Configure the following parameters of the timer
 *  1) mode of the timer - periodic/one shot
 *  2) call back function that will be called from the ISR context
 *  3) context to be passed back in the call back function
 *
 *  pit - iProc timer context (returned by iproc_timer_request())
 *  pcfg - pointer to the configuration structure
 */
int iproc_timer_config (struct iproc_timer *pit, struct timer_ch_cfg *pcfg);  

/*
 * iproc_timer_set_match_start - Set the match register for the timer and start
 * counting
 *
 *  pit - iProc timer context (returned by iproc_timer_request())
 *  load - The load value to be programmed. This function will internally
 *         add this value to the current counter and program the resultant in the
 *         match register. Once the timer is started when the counter 
 *         reaches this value an interrupt will be raised
 */
int iproc_timer_set_match_start (struct iproc_timer* pit, unsigned int load);

/*
 * iproc_timer_free - Read the counter register of the timer 
 *
 * pit - Timer context to be freed.
 * msw - pointer to the Most Significant Word (32 bits) 
 * lsw - pointer to the Leas Significant Word (32 bits) 
 */
int iproc_timer_get_counter(struct iproc_timer* pit, 
                            unsigned long *msw, unsigned long *lsw);
/*
 * iproc_timer_disable_and_clear - Disable the timer and clear the 
 * interrupt
 *
 * pit - Timer context to be freed.
 */
int iproc_timer_disable_and_clear(struct iproc_timer *pit);

/*
 * iproc_timer_stop  - Stop the timer.
 *
 * pit - The timer context to be stopped.
 */
int iproc_timer_stop (struct iproc_timer* pit);

/*
 * iproc_timer_free - Release the timer, after this call the timer can be used
 * again by others.
 *
 * pit - Timer context to be freed.
 */
int iproc_timer_free (struct iproc_timer* pit);

#endif /* __PLAT_IPROC_TIMER_H */
