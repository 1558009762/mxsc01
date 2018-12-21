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
#ifndef _IPROC_TIMER_H_
#define _IPROC_TIMER_H_

/*
 * Get number of timers
 */
extern uint32_t iproc_timer_count(void);

/*
 * Ticking rate (reference clock frequency): ticks per second
 */
extern uint32_t iproc_timer_get_ticking_rate(void);

/*
 * Conversion between ticks and microseconds
 */
#define IPROC_TIMER_USEC_TO_TICKS(usec)                                 \
    ((uint64_t)(usec) * (iproc_timer_get_ticking_rate() / 1000000UL ))
#define IPROC_TIMER_TICKS_TO_USEC(ticks)                                \
    ((uint64_t)(ticks) / (iproc_timer_get_ticking_rate() / 1000000UL))

/*
 * Get max interval in ticks
 */
extern uint64_t iproc_timer_get_max_interval(void);

/*
 * Prototype of user ISR
 *
 * Note: The user ISR will be called in interrrupt context.
 */
typedef void (*iproc_timer_isr_t)(int timer_id, void *cookie);

/*
 * Configure timer (before starting it)
 *
 * Parameters
 *      timer_id    - timer id
 *      periodic    - 1 if periodic; 0 if one-shot.
 *      interval    - interval in ticks
 *      isr         - ISR function to be called when timer expires
 *      cookie      - Pointer to supply when ISR is called
 */
extern int iproc_timer_configure(
                int timer_id, 
                int periodic, 
                uint64_t interval, 
                iproc_timer_isr_t isr, 
                void *cookie);

/* 
 * Start timer
 */ 
extern int iproc_timer_start(int timer_id);

/* 
 * Returns current ticks of the timer.
 */
extern uint64_t iproc_timer_get_current_ticks(int timer_id);

/* 
 * Stop timer
 */ 
extern int iproc_timer_stop(int timer_id);

/*
 * Timer info
 */
typedef struct {
    int configured;
    int started;
    int periodic;
    uint64_t interval;
    iproc_timer_isr_t isr;
    void *cookie;
} iproc_timer_info_t;
extern int iproc_timer_get_info(int timer_id, iproc_timer_info_t *info);

#endif /* _IPROC_TIMER_H_ */
