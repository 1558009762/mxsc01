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


#ifndef __PLAT_GPIO_CFG_H
#define __PLAT_GPIO_CFG_H

typedef unsigned int __bitwise__ iproc_gpio_pull_t;
typedef unsigned int __bitwise__ iproc_gpio_drvstr_t;

struct iproc_gpio_chip;

struct iproc_gpio_cfg {
	iproc_gpio_pull_t	(*get_pull)(struct iproc_gpio_chip *chip, unsigned int offs);
	int		(*set_pull)(struct iproc_gpio_chip *chip, unsigned int offs,
				    iproc_gpio_pull_t pull);
	unsigned (*get_config)(struct iproc_gpio_chip *chip, unsigned int offs);
	int	 (*set_config)(struct iproc_gpio_chip *chip, unsigned int offs,
			       unsigned int config);
};


/**
 * iproc_gpio_cfgpin() - Change the GPIO function of a pin.
 * @pin pin The pin number to configure.
 * @to to The configuration (IPROC_GPIO_GENERAL/ IPROC_GPIO_AUX_FUN) for the pin's function.
 *
 */

extern int iproc_gpio_cfgpin(unsigned int pin, unsigned int to);


/**
 * iproc_gpio_getcfg - Read the current function for a GPIO pin
 * @pin: The pin to read the configuration value for.
 *
 * Read the configuration state of the given @pin, returning a value that
 * could be passed back to iproc_gpio_cfgpin().
 *
 * 
 */
extern unsigned iproc_gpio_getcfg(unsigned int pin);

#define IPROC_GPIO_GENERAL  0
#define IPROC_GPIO_AUX_FUN  1


/* Define values for the pull-{up,down} available for each gpio pin.
 *
 * These values control the state of the weak pull-{up,down} resistors.
 */
#define IPROC_GPIO_PULL_NONE	((__force iproc_gpio_pull_t)0x00)
#define IPROC_GPIO_PULL_DOWN	((__force iproc_gpio_pull_t)0x01)
#define IPROC_GPIO_PULL_UP	((__force iproc_gpio_pull_t)0x02)




/**
 * iproc_gpio_setpull() - set the state of a gpio pin pull resistor
 * @pin: The pin number to configure the pull resistor.
 * @pull: The configuration for the pull resistor.
 *
 * This function sets the state of the pull-{up,down} resistor for the
 * specified pin. It will return 0 if successfull, or a negative error
 * code if the pin cannot support the requested pull setting.
 *
 * @pull is one of IPROC_GPIO_PULL_NONE, IPROC_GPIO_PULL_DOWN or IPROC_GPIO_PULL_UP.
*/
extern int iproc_gpio_setpull(unsigned int pin, iproc_gpio_pull_t pull);


/**
 * iproc_gpio_getpull() - get the pull resistor state of a gpio pin
 * @pin: The pin number to get the settings for
 *
 * Read the pull resistor value for the specified pin.
*/
extern iproc_gpio_pull_t iproc_gpio_getpull(unsigned int pin);

/* internal gpio functions */
extern int iproc_gpio_setpull_updown(struct iproc_gpio_chip *chip,
			    unsigned int off, iproc_gpio_pull_t pull);

extern iproc_gpio_pull_t iproc_gpio_getpull_updown(struct iproc_gpio_chip *chip,
					unsigned int off);

extern int iproc_gpio_set_config(struct iproc_gpio_chip *chip,
			      unsigned int off, unsigned int cfg);
			      
unsigned iproc_gpio_get_config(struct iproc_gpio_chip *chip,
				   unsigned int off);

#endif				   
