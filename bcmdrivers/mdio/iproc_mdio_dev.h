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


#ifndef _IPROC_MDIO_DEV_H
#define _IPROC_MDIO_DEV_H

/* IOCTL commands */

#define MDIO_IOC_MAGIC       'm'

struct mdio_ioc_transfer {
    uint8_t   pa; /* phy address */
    uint8_t   ra; /* register address */
    uint16_t  tx_buf;
    uint16_t  rx_buf;
};

#define MDIO_MSGSIZE(N) \
	((((N)*(sizeof (struct mdio_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct mdio_ioc_transfer))) : 0)

#define MDIO_IOC_MESSAGE(N) _IOW(MDIO_IOC_MAGIC, 0, char[MDIO_MSGSIZE(N)])

#define MDIO_IOC_EXTERNAL_R_REG _IOWR(MDIO_IOC_MAGIC, 0, char[MDIO_MSGSIZE(1)])
#define MDIO_IOC_EXTERNAL_W_REG _IOW(MDIO_IOC_MAGIC, 1, char[MDIO_MSGSIZE(1)])
#define MDIO_IOC_LOCAL_R_REG _IOWR(MDIO_IOC_MAGIC, 2, char[MDIO_MSGSIZE(1)])
#define MDIO_IOC_LOCAL_W_REG _IOW(MDIO_IOC_MAGIC, 3, char[MDIO_MSGSIZE(1)])


#endif
