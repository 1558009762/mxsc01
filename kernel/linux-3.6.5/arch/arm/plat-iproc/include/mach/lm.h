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

#ifndef _LM_DEVICE_H_
#define _LM_DEVICE_H_

#include <linux/device.h>
#include <linux/ioport.h>


struct lm_device {
	struct device dev;
	struct resource resource;
	unsigned int irq;
	unsigned int id;
};

struct lm_driver {
	struct device_driver drv;
	int (*probe) (struct lm_device *);
	void (*remove) (struct lm_device *);
	int (*suspend) (struct lm_device *, pm_message_t);
	int (*resume) (struct lm_device *);
};

int lm_driver_register(struct lm_driver *drv);
void lm_driver_unregister(struct lm_driver *drv);

int lm_device_register(struct lm_device *dev);
void lm_device_unregister(struct lm_device *dev);

#define lm_get_drvdata(lm)	dev_get_drvdata(&(lm)->dev)
#define lm_set_drvdata(lm,d)	dev_set_drvdata(&(lm)->dev, d)

#endif /* _LM_DEVICE_H_ */
