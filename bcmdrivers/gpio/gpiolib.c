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


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>

#include <mach/io_map.h>
#include <mach/reg_utils.h>
#include <mach/memory.h>

#include <asm/pgtable.h>

#include "gpio.h"
#include "gpio_cfg.h"


#if defined(IPROC_GPIO_CCA)

extern struct iproc_gpio_irqcfg cca_gpio_irqcfg;

static struct resource iproc_gpio_cca_config_resource[] = {
    [0] = {
		.start	= IPROC_CCA_BASE,
		.end	= IPROC_CCA_BASE + IPROC_GPIO_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
		.name   = "intr",
    },
    [1] = {.name = "",},
};

#endif /* IPROC_GPIO_CCA */


#if defined(IPROC_GPIO_CCB) || defined(IPROC_GPIO_CCG)

extern struct iproc_gpio_irqcfg ccb_gpio_irqcfg;
extern struct iproc_gpio_cfg ccb_gpio_cfg;

#endif /* IPROC_GPIO_CCB || IPROC_GPIO_CCG */


#if defined(ASIU_GPIO)
extern struct iproc_gpio_irqcfg asiu_gpio_irqcfg;
extern struct iproc_gpio_cfg asiu_gpio_cfg;
#endif /* ASIU_GPIO */

#if defined(AON_GPIO)
extern struct iproc_gpio_irqcfg aon_gpio_irqcfg;
extern struct iproc_gpio_cfg aon_gpio_cfg;
static struct resource aon_gpio_config_resource[] = {
	[0] = {
		.start = CRMU_IOMUX_CONTROL & 0xfffff000,
		.end   = (CRMU_IOMUX_CONTROL & 0xfffff000) + 0x1000 - 1,
		.flags = IORESOURCE_MEM,
		.name  = "dmu",
	},
    [1] = {.name = "",},
};
#endif /* AON_GPIO */


#if defined(IPROC_GPIO_CCG)
static struct resource iproc_gpio_resources[] = {
	[0] = {
		.start	= IPROC_GPIO_CCG_BASE,
		.end	= IPROC_GPIO_CCG_BASE + IPROC_GPIO_REG_SIZE -1,
		.flags	= IORESOURCE_MEM,
	},
#if defined(ASIU_GPIO)
	[1] = {
		.start  = ASIU_GP_DATA_IN_0,
		.end    = ASIU_GP_DATA_IN_0 + 0x1000 - 1,
		.flags  = IORESOURCE_MEM,
	},
#endif
#if defined(AON_GPIO)
	[2] = {
		.start = GP_DATA_IN,
		.end   = GP_DATA_IN + IPROC_GPIO_REG_SIZE -1,
		.flags = IORESOURCE_MEM,
		.child = aon_gpio_config_resource,		
	},
#endif
};
#else
static struct resource iproc_gpio_resources[] = {
	[0] = {
		.start	= IPROC_GPIO_CCA_BASE,
		.end	= IPROC_GPIO_CCA_BASE + IPROC_GPIO_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
		.child = iproc_gpio_cca_config_resource,
	},
	[1] = {
		.start	= IPROC_GPIO_CCB_BASE,
		.end	= IPROC_GPIO_CCB_BASE + IPROC_GPIO_REG_SIZE -1,
		.flags	= IORESOURCE_MEM,
	}
};
#endif

#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) 
/* 
 * Chip level GPIO 0-3 from CMICD, 
 * GPIO 4-11 from ChipcommonA gpio pin 0 - 7
 * Hence the base is 4 and the number is 8.
 */
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id   = IPROC_GPIO_CCA_ID,
        .chip   = {
            .base           = 4,
            .label          = "GPIOA",
            .ngpio          = 8,
        },
        .irq_base = IPROC_GPIO_CCA_IRQ_BASE,
        .resource = &iproc_gpio_resources[0],
        .irq = IPROC_GPIO_CCA_INT,
        .irqcfg = &cca_gpio_irqcfg,
        .pin_offset = 0,
    },
};
#elif defined(CONFIG_MACH_HR2)  
/* 
 * Chip level GPIO 0-3 from CMICD, 
 * GPIO 4-15 are from ChipcommonA gpio pin 0 - 11
 * where GPIO 8-15 are shared with MII or LED depends on strap pin
 * Hence the base is 4 and the number is 12.
 */
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id   = IPROC_GPIO_CCA_ID,
        .chip   = {
            .base           = 4,
            .label          = "GPIOA",
            .ngpio          = 12,
        },
        .irq_base = IPROC_GPIO_CCA_IRQ_BASE,
        .resource = &iproc_gpio_resources[0],
        .irq = IPROC_GPIO_CCA_INT,
        .irqcfg = &cca_gpio_irqcfg,
        .pin_offset = 0,
    },
};
#elif defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
/*
* Chip level GPIO 0-3 from CMICD,
* GPIO 4-15 are from ChipcommonG gpio pin 4 - 15
*/
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id = IPROC_GPIO_CCG_ID,
	.config = &ccb_gpio_cfg,
        .chip = {
            .base           = 4,
            .label          = "GPIOG",
            .ngpio          = 12,
       	},
       	.irq_base = IPROC_GPIO_CCG_IRQ_BASE,
       	.resource = &iproc_gpio_resources[0],
       	.irq = IPROC_GPIO_CCG_INT,
       	.irqcfg = &ccb_gpio_irqcfg,
       	.pin_offset = 4,
    },
};
#elif defined(CONFIG_MACH_SB2)
/*
* Chip level GPIO 0-3 from CMICD but seperate with iProc GPIO,
* GPIO 0-15 are from ChipcommonG gpio pin 0 - 15
*/
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id = IPROC_GPIO_CCG_ID,
	.config = &ccb_gpio_cfg,
        .chip = {
            .base           = 0,
            .label          = "GPIOG",
            .ngpio          = 15,
       	},
       	.irq_base = IPROC_GPIO_CCG_IRQ_BASE,
       	.resource = &iproc_gpio_resources[0],
       	.irq = IPROC_GPIO_CCG_INT,
       	.irqcfg = &ccb_gpio_irqcfg,
       	.pin_offset = 0,
    },
};
#else
struct iproc_gpio_chip iproc_gpios_config[] = {
    [0] = {
        .id   = IPROC_GPIO_CCA_ID,
        .chip   = {
            .base           = 0,
            .label          = "GPIOA",
            .ngpio          = 32,
        },
        .irq_base = IPROC_GPIO_CCA_IRQ_BASE,
        .resource = &iproc_gpio_resources[0],
        .irq = IPROC_GPIO_CCA_INT,
        .irqcfg = &cca_gpio_irqcfg,
        .pin_offset = 0,
    },
    [1] = {
        .id   = IPROC_GPIO_CCB_ID,
        .chip   = {
            .base           = -EINVAL,
            .label          = "GPIOB",
            .ngpio          = 4,
        },
        .irq_base = IPROC_GPIO_CCB_IRQ_BASE,
        .resource = &iproc_gpio_resources[1],
        .irq = IPROC_GPIO_CCB_INT,
        .irqcfg = &ccb_gpio_irqcfg,
        .pin_offset = 0,
    },
};
#endif 

int __init iproc_gpiolib_init(void)
{
    struct iproc_gpio_chip *chip = iproc_gpios_config;
    int gpn;
    int temp_base;

    temp_base = 0;
    for (gpn = 0; gpn < ARRAY_SIZE(iproc_gpios_config); gpn++, chip++) {
        if (gpn >= MAX_NS_GPIO){
            printk("Unavailabe to add gpiolib\n");
            return -EINVAL;
        }
            
        if (chip->chip.base == -EINVAL) {
            chip->chip.base = temp_base;            
        }

        iproc_gpiolib_add(chip);
        temp_base = chip->chip.base + chip->chip.ngpio;
	}

	return 0;
}
