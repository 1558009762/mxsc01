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
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/io.h>

#include "gpio.h"
#include "gpio_cfg.h"

#if defined(IPROC_GPIO_CCB) || defined(IPROC_GPIO_CCG)
int iproc_gpio_set_config_ccb(struct iproc_gpio_chip *chip,
			      unsigned int off, unsigned int cfg)
{
    unsigned long aux_sel;
    unsigned int aux_sel_reg;
    unsigned int base, pin;

    base = 0;
    pin = 0;

    if ((chip->id == IPROC_GPIO_CCB_ID) || (chip->id == IPROC_GPIO_CCG_ID)) {
        aux_sel_reg = IPROC_GPIO_CCB_AUX_SEL;
        base = (unsigned int)chip->ioaddr;
        pin = 1 << off;
    }
    aux_sel = readl_relaxed(base + aux_sel_reg);

    switch (cfg) {
        case IPROC_GPIO_GENERAL:
            aux_sel &= ~(pin);
            break;            
        case IPROC_GPIO_AUX_FUN:
            aux_sel |= (pin);
            break;
        default:
            return -EINVAL;
    }
    writel_relaxed(aux_sel, base + aux_sel_reg);

    return 0;
}


unsigned iproc_gpio_get_config_ccb(struct iproc_gpio_chip *chip,
				   unsigned int off)
{
    unsigned long aux_sel;
    unsigned int aux_sel_reg;
    unsigned int base, pin;

    base = 0;
    pin = 0;

    if ((chip->id == IPROC_GPIO_CCB_ID) || (chip->id == IPROC_GPIO_CCG_ID)) {
        aux_sel_reg = IPROC_GPIO_CCB_AUX_SEL;
        base = (unsigned int)chip->ioaddr;
        pin = 1 << off;
    }

    aux_sel = readl_relaxed(base + aux_sel_reg);

    if (aux_sel & pin) {
        return IPROC_GPIO_AUX_FUN;
    } else {
        return IPROC_GPIO_GENERAL;
    }        
}

int iproc_gpio_setpull_updown_ccb(struct iproc_gpio_chip *chip,
			    unsigned int off, iproc_gpio_pull_t pull)
{
    unsigned int base;
    base = 0;

    if ((chip->id == IPROC_GPIO_CCB_ID) || (chip->id == IPROC_GPIO_CCG_ID)) {
        unsigned long pad_res, res_en;        

        base = (unsigned int)chip->ioaddr;

        pad_res = readl_relaxed(base + IPROC_GPIO_CCB_PAD_RES);
        res_en = readl_relaxed(base + IPROC_GPIO_CCB_RES_EN);
        switch (pull) {
            case IPROC_GPIO_PULL_UP:
                pad_res |= (1 << off);
                res_en |= (1 << off);
                break;
            case IPROC_GPIO_PULL_DOWN:
                pad_res &= ~(1 << off);
                res_en |= (1 << off);
                break;
            case IPROC_GPIO_PULL_NONE:           
                res_en &= ~(1 << off);
                break;                
            default:
                return -EINVAL;
        }
        writel_relaxed(pad_res, base + IPROC_GPIO_CCB_PAD_RES);
        writel_relaxed(res_en, base + IPROC_GPIO_CCB_RES_EN);
    }
    return 0;
}


iproc_gpio_pull_t iproc_gpio_getpull_updown_ccb(struct iproc_gpio_chip *chip,
					unsigned int off)
{

    unsigned int base;
    base = 0;

    if ((chip->id == IPROC_GPIO_CCB_ID) || (chip->id == IPROC_GPIO_CCG_ID)) {
        unsigned long pad_res, res_en;        

        base = (unsigned int)chip->ioaddr;

        pad_res = readl_relaxed(base + IPROC_GPIO_CCB_PAD_RES);
        res_en = readl_relaxed(base + IPROC_GPIO_CCB_RES_EN);
        pad_res &= 1 << off;
        res_en &= 1 << off;

        if (res_en) {
            if (pad_res) {
                return IPROC_GPIO_PULL_UP;
            } else {
                return IPROC_GPIO_PULL_DOWN;
            }
        } else {
            return IPROC_GPIO_PULL_NONE;
        }
    }    
    return IPROC_GPIO_PULL_NONE;
}

struct iproc_gpio_cfg ccb_gpio_cfg = {
    .get_pull = iproc_gpio_getpull_updown_ccb,
    .set_pull = iproc_gpio_setpull_updown_ccb,
    .get_config = iproc_gpio_get_config_ccb,
    .set_config = iproc_gpio_set_config_ccb,    
};
#endif /* IPROC_GPIO_CCB || IPROC_GPIO_CCG */

#if defined(ASIU_GPIO)
int asiu_gpio_set_config(struct iproc_gpio_chip *chip,
			      unsigned int off, unsigned int cfg)
{
    unsigned long aux_sel;
    unsigned int aux_sel_reg;
    unsigned int base, pin;

    base = 0;
    pin = 0;

    aux_sel_reg = ASIU_GP_AUX_SEL_0_BASE + 
                    ASIU_GPIO_REGIDX(off)*ASIU_GPIO_REGOFFSET;
    base = (unsigned int)chip->ioaddr;
    pin = 1 << ASIU_GPIO_REGBIT(off);

    aux_sel = readl_relaxed(base + aux_sel_reg);

    switch (cfg) {
    case IPROC_GPIO_GENERAL:
        aux_sel &= ~(pin);
        break;            
    case IPROC_GPIO_AUX_FUN:
        aux_sel |= (pin);
        break;
    default:
        return -EINVAL;
    }
    writel_relaxed(aux_sel, base + aux_sel_reg);

    return 0;
}


unsigned asiu_gpio_get_config(struct iproc_gpio_chip *chip,
				   unsigned int off)
{
    unsigned long aux_sel;
    unsigned int aux_sel_reg;
    unsigned int base, pin;

    base = 0;
    pin = 0;

    aux_sel_reg = ASIU_GP_AUX_SEL_0_BASE + 
                    ASIU_GPIO_REGIDX(off)*ASIU_GPIO_REGOFFSET;
    base = (unsigned int)chip->ioaddr;
    pin = 1 << ASIU_GPIO_REGBIT(off);

    aux_sel = readl_relaxed(base + aux_sel_reg);

    if (aux_sel & pin) {
        return IPROC_GPIO_AUX_FUN;
    } else {
        return IPROC_GPIO_GENERAL;
    }        
}


int asiu_gpio_setpull_updown(struct iproc_gpio_chip *chip,
			    unsigned int off, iproc_gpio_pull_t pull)
{
    unsigned int base;
    unsigned long pad_res, res_en, reg_offset, shft;	    

    base = (unsigned int)chip->ioaddr;

    reg_offset = ASIU_GPIO_REGIDX(off) * ASIU_GPIO_REGOFFSET;
    pad_res = readl_relaxed(base + ASIU_GP_PAD_RES_0_BASE + reg_offset);
    res_en = readl_relaxed(base + ASIU_GP_RES_EN_0_BASE + reg_offset);

    shft = ASIU_GPIO_REGBIT(off);
    switch (pull) {
    case IPROC_GPIO_PULL_UP:
        pad_res |= (1 << shft);
        res_en |= (1 << shft);
        break;
    case IPROC_GPIO_PULL_DOWN:
        pad_res &= ~(1 << shft);
        res_en |= (1 << shft);
        break;
    case IPROC_GPIO_PULL_NONE:           
        res_en &= ~(1 << shft);
        break;                
    default:
        return -EINVAL;
    }
    writel_relaxed(pad_res, base + ASIU_GP_PAD_RES_0_BASE + reg_offset);
    writel_relaxed(res_en, base + ASIU_GP_RES_EN_0_BASE + reg_offset);
    return 0;
}


iproc_gpio_pull_t asiu_gpio_getpull_updown(struct iproc_gpio_chip *chip,
					unsigned int off)
{

    unsigned int base;
    unsigned long pad_res, res_en, reg_offset, shft;        

    base = (unsigned int)chip->ioaddr;

    reg_offset = ASIU_GPIO_REGIDX(off) * ASIU_GPIO_REGOFFSET;
    pad_res = readl_relaxed(base + ASIU_GP_PAD_RES_0_BASE + reg_offset);
    res_en = readl_relaxed(base + ASIU_GP_RES_EN_0_BASE + reg_offset);

    shft = ASIU_GPIO_REGBIT(off);
    pad_res &= 1 << shft;
    res_en &= 1 << shft;

    if (res_en) {
        if (pad_res) {
            return IPROC_GPIO_PULL_UP;
        } else {
            return IPROC_GPIO_PULL_DOWN;
        }
    } else {
        return IPROC_GPIO_PULL_NONE;
    }
    return IPROC_GPIO_PULL_NONE;
}

struct iproc_gpio_cfg asiu_gpio_cfg = {
    .get_pull = asiu_gpio_getpull_updown,
    .set_pull = asiu_gpio_setpull_updown,
    .get_config = asiu_gpio_get_config,
    .set_config = asiu_gpio_set_config,    
};
#endif /* ASIU_GPIO */


#if defined(AON_GPIO)
int aon_gpio_set_config(struct iproc_gpio_chip *chip,
			      unsigned int off, unsigned int cfg)
{
    unsigned long aux_sel;
    unsigned int aux_sel_reg;
    unsigned int base, pin;

    aux_sel_reg = IPROC_GPIO_CCB_AUX_SEL;
    base = (unsigned int)chip->ioaddr;
    pin = 1 << off;

    aux_sel = readl_relaxed(base + aux_sel_reg);

    switch (cfg) {
        case IPROC_GPIO_GENERAL:
            aux_sel &= ~(pin);
            break;            
        case IPROC_GPIO_AUX_FUN:
            aux_sel |= (pin);
            break;
        default:
            return -EINVAL;
    }
    writel_relaxed(aux_sel, base + aux_sel_reg);

    return 0;
}


unsigned aon_gpio_get_config(struct iproc_gpio_chip *chip,
				   unsigned int off)
{
    unsigned long aux_sel;
    unsigned int aux_sel_reg;
    unsigned int base, pin;

    aux_sel_reg = IPROC_GPIO_CCB_AUX_SEL;
    base = (unsigned int)chip->ioaddr;
    pin = 1 << off;

    aux_sel = readl_relaxed(base + aux_sel_reg);

    if (aux_sel & pin) {
        return IPROC_GPIO_AUX_FUN;
    } else {
        return IPROC_GPIO_GENERAL;
    }        
}


int aon_gpio_setpull_updown(struct iproc_gpio_chip *chip,
			    unsigned int off, iproc_gpio_pull_t pull)
{
    unsigned int base;
    unsigned long pad_res, res_en;        

    base = (unsigned int)chip->ioaddr;

    pad_res = readl_relaxed(base + IPROC_GPIO_CCB_PAD_RES);
    res_en = readl_relaxed(base + IPROC_GPIO_CCB_RES_EN);

    switch (pull) {
        case IPROC_GPIO_PULL_UP:
            pad_res |= (1 << off);
            res_en |= (1 << off);
            break;
        case IPROC_GPIO_PULL_DOWN:
            pad_res &= ~(1 << off);
            res_en |= (1 << off);
            break;
        case IPROC_GPIO_PULL_NONE:           
            res_en &= ~(1 << off);
            break;                
        default:
            return -EINVAL;
    }
    writel_relaxed(pad_res, base + IPROC_GPIO_CCB_PAD_RES);
    writel_relaxed(res_en, base + IPROC_GPIO_CCB_RES_EN);

    return 0;
}


iproc_gpio_pull_t aon_gpio_getpull_updown(struct iproc_gpio_chip *chip,
					unsigned int off)
{

    unsigned int base;
    unsigned long pad_res, res_en;        

    base = (unsigned int)chip->ioaddr;

    pad_res = readl_relaxed(base + IPROC_GPIO_CCB_PAD_RES);
    res_en = readl_relaxed(base + IPROC_GPIO_CCB_RES_EN);

    pad_res &= 1 << off;
    res_en &= 1 << off;

    if (res_en) {
        if (pad_res) {
            return IPROC_GPIO_PULL_UP;
        } else {
            return IPROC_GPIO_PULL_DOWN;
        }
    } else {
        return IPROC_GPIO_PULL_NONE;
    }

    return IPROC_GPIO_PULL_NONE;
}

struct iproc_gpio_cfg aon_gpio_cfg = {
    .get_pull = aon_gpio_getpull_updown,
    .set_pull = aon_gpio_setpull_updown,
    .get_config = aon_gpio_get_config,
    .set_config = aon_gpio_set_config,    
};
#endif /* AON_GPIO */


iproc_gpio_pull_t iproc_gpio_getpull(unsigned int pin)
{
	struct iproc_gpio_chip *chip = iproc_gpiolib_getchip(pin);
	unsigned long flags;
	int offset, ret = -EINVAL;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;
	offset += chip->pin_offset;

	iproc_gpio_lock(chip, flags);
    if (chip->config){	
	    ret = (chip->config->get_pull)(chip, offset);
    }
	iproc_gpio_unlock(chip, flags);

	return ret;

}
EXPORT_SYMBOL(iproc_gpio_getpull);


int iproc_gpio_setpull(unsigned int pin, iproc_gpio_pull_t pull)
{
	struct iproc_gpio_chip *chip = iproc_gpiolib_getchip(pin);
	unsigned long flags;
	int offset, ret = -EINVAL;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;
	offset += chip->pin_offset;

	iproc_gpio_lock(chip, flags);
    if (chip->config){
	    ret = (chip->config->set_pull)(chip, offset, pull);
    }
	iproc_gpio_unlock(chip, flags);

	return ret;

}
EXPORT_SYMBOL(iproc_gpio_setpull);

unsigned iproc_gpio_getcfg(unsigned int pin)
{
    struct iproc_gpio_chip *chip = iproc_gpiolib_getchip(pin);
    unsigned long flags;
    int offset;
	unsigned ret = 0;

	if (!chip)
		return -EINVAL;

    offset = pin - chip->chip.base;
    offset += chip->pin_offset;
    iproc_gpio_lock(chip, flags);
    if (chip->config){
        ret = (chip->config->get_config)(chip, offset);
    }
    iproc_gpio_unlock(chip, flags);
        
    return ret;
}

EXPORT_SYMBOL(iproc_gpio_getcfg);

int iproc_gpio_cfgpin(unsigned int pin, unsigned int config)
{
	struct iproc_gpio_chip *chip = iproc_gpiolib_getchip(pin);
	unsigned long flags;
	int offset;
	int ret = 0;

	if (!chip)
		return -EINVAL;

	offset = pin - chip->chip.base;
	offset += chip->pin_offset;
	iproc_gpio_lock(chip, flags);
    if (chip->config){
        (chip->config->set_config)(chip, offset, config);
    }
	iproc_gpio_unlock(chip, flags);

	return ret;


}
EXPORT_SYMBOL(iproc_gpio_cfgpin);
