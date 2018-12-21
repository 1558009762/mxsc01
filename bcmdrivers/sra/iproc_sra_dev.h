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

#ifndef _IPROC_SRA_DEV_H
#define _IPROC_SRA_DEV_H

/* IOCTL commands */

#define SRA_IOC_MAGIC       's'

struct sra_ioc_transfer {
    uint8_t   page;
    uint8_t   offset;
    int     len;
    uint64_t  tx_buf;
    uint64_t  rx_buf;
};

#define SRA_MSGSIZE(N) \
	((((N)*(sizeof (struct sra_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct sra_ioc_transfer))) : 0)

#define SRA_IOC_MESSAGE(N) _IOW(SRA_IOC_MAGIC, 0, char[SRA_MSGSIZE(N)])

#define SRA_IOC_R_REG _IOWR(SRA_IOC_MAGIC, 0, char[SRA_MSGSIZE(1)])
#define SRA_IOC_W_REG _IOW(SRA_IOC_MAGIC, 1, char[SRA_MSGSIZE(1)])


#endif
