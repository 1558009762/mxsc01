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
#ifndef _SWRESET_REC_H_
#define _SWRESET_REC_H_

/* Handle to access Software Record Record */
typedef void *SWRR_HANDLE;

/* Get number of software reset records for the SoC; 0 if not suppprted */
extern unsigned int swreset_record_get_record_count(void);

/* Get number of bits per software reset record for the SoC */
extern unsigned int swreset_record_get_record_width(void);

/* Register to use one software reset record; return NULL if used out */
extern SWRR_HANDLE swreset_record_register(const char *name);

/* Unregister to return the record */
extern void swreset_record_unregister(SWRR_HANDLE handle);

/* Set value of the software reset record */
extern int swreset_record_set(SWRR_HANDLE handle, int value);

/* Get value of the software reset record */
extern int swreset_record_get(SWRR_HANDLE handle, int *value);
 
#endif /* _SWRESET_REC_H_ */
