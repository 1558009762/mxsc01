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
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/version.h>

#include <mach/iproc_regs.h>
#include <mach/memory.h>


#include "gpio.h"

#if defined(CONFIG_MACH_SB2)
#define POWERDOWN_ALARM
#endif

#ifdef POWERDOWN_ALARM

#include <linux/netlink.h>
#include <net/sock.h>
#include <linux/time.h>
static struct sock *netlink_sock;
#ifndef GROUP_MASK 
#define GROUP_MASK 3
#endif

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
#define irq_get_chip_data get_irq_chip_data
#define irq_set_chip_data set_irq_chip_data
#define  irq_set_chip set_irq_chip
#define irq_set_handler set_irq_handler
#define status_use_accessors status
#endif


static struct iproc_gpio_chip *iproc_gpio_dev[MAX_NS_GPIO] = {};
static int dev = 0;

static unsigned int _iproc_gpio_readl(struct iproc_gpio_chip *chip, int reg)
{
    return readl(chip->ioaddr + reg);
}

static void _iproc_gpio_writel(struct iproc_gpio_chip *chip, unsigned int val, int reg)
{
	writel(val, chip->ioaddr + reg);
}


/*
@ pin : the actual pin number of the gpiochip
*/
static int iproc_gpio_to_irq(struct iproc_gpio_chip *chip, unsigned int pin) {
    return (chip->irq_base + pin - chip->pin_offset);
}

/*
returns the actual pin number of the gpiochip
*/
static int iproc_irq_to_gpio(struct iproc_gpio_chip *chip, unsigned int irq) {
    return (irq - chip->irq_base + chip->pin_offset);
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static void iproc_gpio_irq_ack(unsigned int irq)
{
#else
static void iproc_gpio_irq_ack(struct irq_data *d)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

    if (ourchip) {
    	struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
	if (irqcfg && irqcfg->ack)
	    irqcfg->ack(irq);
		
    }    
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static void iproc_gpio_irq_unmask(unsigned int irq)
{
#else
static void iproc_gpio_irq_unmask(struct irq_data *d)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

    if (ourchip) {
    	struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
	if (irqcfg && irqcfg->unmask)
	    irqcfg->unmask(irq);
    }
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static void iproc_gpio_irq_mask(unsigned int irq)
{
#else
static void iproc_gpio_irq_mask(struct irq_data *d)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

    if (ourchip) {
    	struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
	if (irqcfg && irqcfg->mask)
	    irqcfg->mask(irq);
    }
}
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
static int iproc_gpio_irq_set_type(unsigned int irq, unsigned int type)
{
#else
static int iproc_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
    unsigned int irq = d->irq;
#endif
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

    if (ourchip) {
    	struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
	if (irqcfg && irqcfg->set_type)
	    return irqcfg->set_type(irq, type);
    }
    return -EINVAL;
}

#if defined(IPROC_GPIO_CCA)
static irqreturn_t 
iproc_gpio_irq_handler_cca(int irq, void *dev)

{
    unsigned int  val, irq_type;
    unsigned int  int_mask, int_pol, in;
    unsigned int  event_mask, event, event_pol, tmp = 0;
    int iter, g_irq, max_pin;
    struct iproc_gpio_chip *ourchip = dev;


    val = readl(ourchip->intr_ioaddr + IPROC_CCA_INT_STS);
    
    if (val & IPROC_CCA_INT_F_GPIOINT) {
        int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL_MASK);
        int_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL);
        in = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_DIN);
        event_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT_MASK);
        event = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT);
        event_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EDGE);

		max_pin = ourchip->pin_offset + ourchip->chip.ngpio;
        for (iter = ourchip->pin_offset; iter < max_pin; iter ++) {
            g_irq = iproc_gpio_to_irq(ourchip, iter);  
            irq_type = irq_desc[g_irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;
            switch(irq_type) {
                case IRQ_TYPE_EDGE_RISING:
                    tmp = event_mask;
                    tmp &= event;
                    tmp &= ~event_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }
                    break;
                case IRQ_TYPE_EDGE_FALLING:
                    tmp = event_mask;
                    tmp &= event;
                    tmp &= event_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }                
                    break;
                case IRQ_TYPE_LEVEL_LOW:
                    tmp = in ^ int_pol;
                    tmp &= int_mask;
                    tmp &= int_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }                
                    break;
                case IRQ_TYPE_LEVEL_HIGH:
                    tmp = in ^ int_pol;
                    tmp &= int_mask;
                    tmp &= ~int_pol;
                    if (tmp & (1 << iter)) {
                        generic_handle_irq(g_irq);
                    }                
                    break;
                default:                        
                    break;
            }
        }
    }else {
        return IRQ_NONE;
    }

    return IRQ_HANDLED;
}

static void iproc_gpio_irq_ack_cca(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
	if (ourchip->id == IPROC_GPIO_CCA_ID) {
	    unsigned int  event_status, irq_type;
	
	    event_status = 0;
	    irq_type = irq_desc[irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;
	    if (irq_type & IRQ_TYPE_EDGE_BOTH) 
	    {
		event_status |= (1 << pin);	       
		_iproc_gpio_writel(ourchip, event_status,
		    IPROC_GPIO_CCA_INT_EVENT);
	    }
	
	}
}

static void iproc_gpio_irq_unmask_cca(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	unsigned int int_mask, irq_type;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	irq_type = irq_desc[irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;
	
	if (ourchip->id == IPROC_GPIO_CCA_ID) {
	    unsigned int  event_mask; 
	
	    event_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT_MASK);
	    int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL_MASK);
	
	    if (irq_type & IRQ_TYPE_EDGE_BOTH) {
		event_mask |= 1 << pin;
		_iproc_gpio_writel(ourchip, event_mask, 
		    IPROC_GPIO_CCA_INT_EVENT_MASK);
	    } else {
		int_mask |= 1 << pin;
		_iproc_gpio_writel(ourchip, int_mask, 
		    IPROC_GPIO_CCA_INT_LEVEL_MASK);
	    }
	}

}

static void iproc_gpio_irq_mask_cca(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	unsigned int irq_type, int_mask;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	irq_type = irq_desc[irq].status_use_accessors & IRQ_TYPE_SENSE_MASK;
	
	if (ourchip->id == IPROC_GPIO_CCA_ID) {
	    unsigned int  event_mask;
	    
	    event_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EVENT_MASK);
	    int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL_MASK);
	
	    if (irq_type & IRQ_TYPE_EDGE_BOTH) {
		event_mask &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, event_mask,
		    IPROC_GPIO_CCA_INT_EVENT_MASK);
	    } else {
		int_mask &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, int_mask, 
		    IPROC_GPIO_CCA_INT_LEVEL_MASK);
	    }
	}
}

static int iproc_gpio_irq_set_type_cca(unsigned int irq, unsigned int type)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;    
	
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
	if (ourchip->id == IPROC_GPIO_CCA_ID) {
	    unsigned int  event_pol, int_pol;
	
	    switch (type & IRQ_TYPE_SENSE_MASK) {
	    case IRQ_TYPE_EDGE_RISING:
		event_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EDGE);
		event_pol &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, event_pol, IPROC_GPIO_CCA_INT_EDGE);
		break;
	    case IRQ_TYPE_EDGE_FALLING:
		event_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_EDGE);
		event_pol |= (1 << pin);
		_iproc_gpio_writel(ourchip, event_pol, IPROC_GPIO_CCA_INT_EDGE);
		break;
	    case IRQ_TYPE_LEVEL_HIGH:
		int_pol = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCA_INT_LEVEL);
		int_pol &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, int_pol, IPROC_GPIO_CCA_INT_LEVEL);
		break;
	    case IRQ_TYPE_LEVEL_LOW:
		int_pol = _iproc_gpio_readl(ourchip,IPROC_GPIO_CCA_INT_LEVEL);
		int_pol |= (1 << pin);
		_iproc_gpio_writel(ourchip, int_pol, IPROC_GPIO_CCA_INT_LEVEL);
		break;
	    default:
		printk(KERN_ERR "unsupport irq type !\n");
		return -EINVAL;
	    }
	}
	
	return 0;
}

struct iproc_gpio_irqcfg cca_gpio_irqcfg = {
        .flags = IRQF_NO_SUSPEND|IRQF_SHARED,
	.handler = iproc_gpio_irq_handler_cca,
	.ack = iproc_gpio_irq_ack_cca,
	.mask = iproc_gpio_irq_mask_cca,
	.unmask = iproc_gpio_irq_unmask_cca,
	.set_type = iproc_gpio_irq_set_type_cca,
};
#endif /* IPROC_GPIO_CCA */

#ifdef POWERDOWN_ALARM
static void recv_sock_handler(struct sk_buff * sk)
{
  printk("start to recv_handler()....\n");
}

int poweroff_handle(unsigned char val)
{
        struct sk_buff * skb0 = NULL;
        struct nlmsghdr * nlhdr = NULL;

		if(0 == val)
		{
			printk("poweroff_handle val=%d\n",val);
			return -1;
		}
        skb0 = alloc_skb(64, GFP_KERNEL);
        if(skb0)
        {
                nlhdr = nlmsg_put(skb0, 0, 0, 0, 64-sizeof(*nlhdr), 0);

                memcpy(NLMSG_DATA(nlhdr), (unsigned char*)&val, 1);

                nlhdr->nlmsg_len = NLMSG_LENGTH(1);
                nlhdr->nlmsg_pid = 0;
                nlhdr->nlmsg_flags= 0;

                NETLINK_CB(skb0).pid = 0;
                NETLINK_CB(skb0).dst_group = GROUP_MASK;


                netlink_broadcast(netlink_sock, skb0, 0, GROUP_MASK, GFP_KERNEL);

        }
        else
        {
                printk("Error to malloc nlhdr...\n");
                return 0;
        }
        return 1;

        nlmsg_failure:
        if(skb0 != 0)
                        kfree_skb(skb0);
        return 0;
}

#endif
#if defined(IPROC_GPIO_CCB) || defined(IPROC_GPIO_CCG)
static irqreturn_t 
iproc_gpio_irq_handler_ccb(int irq, void *dev)
{
    struct iproc_gpio_chip *ourchip = dev;
    int iter, max_pin;
    unsigned int  val;
	struct timeval now;

//	do_gettimeofday(&now);
//	printk("gpio interrupt time sec:%d usec:%d\n",now.tv_sec,now.tv_usec);

    val = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MSTAT);
    if(!val){
        return IRQ_NONE;
    }
#ifdef POWERDOWN_ALARM	
	poweroff_handle(1);
#endif
	max_pin = ourchip->pin_offset + ourchip->chip.ngpio;
    for (iter = ourchip->pin_offset; iter < max_pin; iter ++) {
        if (val & (1 << iter)) {
            generic_handle_irq(iproc_gpio_to_irq(ourchip, iter));
        }
    }
    
    return IRQ_HANDLED;
}

static void iproc_gpio_irq_ack_ccb(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
	if ((ourchip->id == IPROC_GPIO_CCB_ID) || 
		(ourchip->id == IPROC_GPIO_CCG_ID)) {
	    unsigned int int_clear = 0;
	
	    int_clear |= (1 << pin);
	    _iproc_gpio_writel(ourchip, int_clear, IPROC_GPIO_CCB_INT_CLR);
	
	}
}

static void iproc_gpio_irq_unmask_ccb(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	unsigned int int_mask;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
	if ((ourchip->id == IPROC_GPIO_CCB_ID) ||
		(ourchip->id == IPROC_GPIO_CCG_ID)) {
	    int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MASK);
	    int_mask |= (1 << pin);
	    _iproc_gpio_writel(ourchip, int_mask, IPROC_GPIO_CCB_INT_MASK);
	}
	
}

static void iproc_gpio_irq_mask_ccb(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	unsigned int int_mask;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
	if ((ourchip->id == IPROC_GPIO_CCB_ID) ||
		(ourchip->id == IPROC_GPIO_CCG_ID)) {
	    int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MASK);
	    int_mask &= ~(1 << pin);
	    _iproc_gpio_writel(ourchip, int_mask,IPROC_GPIO_CCB_INT_MASK);
	}
}

static int iproc_gpio_irq_set_type_ccb(unsigned int irq, unsigned int type)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;    
	
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
	if ((ourchip->id == IPROC_GPIO_CCB_ID) ||
		(ourchip->id == IPROC_GPIO_CCG_ID)) {
	    unsigned int  int_type, int_de, int_edge;
	    int_type = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_TYPE);
	    int_edge = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_EDGE);
	    switch (type) {
		case IRQ_TYPE_EDGE_BOTH:
		    int_type &= ~(1 << pin); 
		    int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
		    int_de |= (1 << pin); 
		    _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
		    break;
		case IRQ_TYPE_EDGE_RISING:
		    int_type &= ~(1 << pin); 
		    int_edge |= (1 << pin); 
		    
		    int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
		    int_de  &= ~(1 << pin); 
		    _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
		    break;
		case IRQ_TYPE_EDGE_FALLING:
		    int_type &= ~(1 << pin);
		    int_edge &= ~(1 << pin);
		    
		    int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
		    int_de  &= ~(1 << pin); 
		    _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
		    break;
		case IRQ_TYPE_LEVEL_HIGH:
		    int_type |= (1 << pin); 
		    int_edge |= (1 << pin); 
		    break;
		case IRQ_TYPE_LEVEL_LOW:
		    int_type |= (1 << pin); 
		    int_edge &= ~(1 << pin); 
		    break;
		default:
		    printk(KERN_ERR "unsupport irq type !\n");
		    return -EINVAL;
	    }
	    _iproc_gpio_writel(ourchip, int_type, IPROC_GPIO_CCB_INT_TYPE);
	    _iproc_gpio_writel(ourchip, int_edge, IPROC_GPIO_CCB_INT_EDGE);
	}
	
	return 0;
}

struct iproc_gpio_irqcfg ccb_gpio_irqcfg = {
        .flags = IRQF_NO_SUSPEND,
	.handler = iproc_gpio_irq_handler_ccb,
	.ack = iproc_gpio_irq_ack_ccb,
	.mask = iproc_gpio_irq_mask_ccb,
	.unmask = iproc_gpio_irq_unmask_ccb,
	.set_type = iproc_gpio_irq_set_type_ccb,
};
#endif /* IPROC_GPIO_CCB || IPROC_GPIO_CCG*/


#if defined(ASIU_GPIO)
static irqreturn_t 
asiu_gpio_irq_handler(int irq, void *dev)
{
    struct iproc_gpio_chip *ourchip = dev;
    int iter, max_pin;
    int round, i, start_pin, end_pin;
    unsigned int  val;
    int irq_handled = 0;

    max_pin = ourchip->pin_offset + ourchip->chip.ngpio;
    round = (max_pin - 1)/32 + 1;
    for(i=0; i< round; i++) {
        val = _iproc_gpio_readl(ourchip, ASIU_GP_INT_MSTAT_0_BASE + i*ASIU_GPIO_REGOFFSET);
	if (val) {
	    start_pin = i * 32;
	    end_pin = start_pin + 32;
	    if (start_pin < ourchip->pin_offset)
	    	start_pin = ourchip->pin_offset;
	    if (end_pin > max_pin)
	    	end_pin = max_pin;
            for (iter = start_pin; iter < end_pin; iter ++) {
	    	if (val & (1 << ASIU_GPIO_REGBIT(iter)))
                    generic_handle_irq(iproc_gpio_to_irq(ourchip, iter));
            }
	    irq_handled = 1;
	}
    }

    if (!irq_handled)
    	return IRQ_NONE;
    else
     return IRQ_HANDLED;
}

static void asiu_gpio_irq_ack(unsigned int irq)
{
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin;
    unsigned int int_clear = 0;
	
    pin = iproc_irq_to_gpio(ourchip, irq);
	
    int_clear |= (1 << ASIU_GPIO_REGBIT(pin));
    _iproc_gpio_writel(ourchip, int_clear, 
                        ASIU_GP_INT_CLR_0_BASE + ASIU_GPIO_REGIDX(pin)*ASIU_GPIO_REGOFFSET);	
}

static void asiu_gpio_irq_unmask(unsigned int irq)
{
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin, reg_offset;
    unsigned int int_mask;
	
    pin = iproc_irq_to_gpio(ourchip, irq);

    reg_offset = ASIU_GP_INT_MSK_0_BASE	+ ASIU_GPIO_REGIDX(pin)*ASIU_GPIO_REGOFFSET;
    int_mask = _iproc_gpio_readl(ourchip, reg_offset);
    int_mask |= (1 << ASIU_GPIO_REGBIT(pin));
    _iproc_gpio_writel(ourchip, int_mask, reg_offset);
}

static void asiu_gpio_irq_mask(unsigned int irq)
{
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin, reg_offset;
    unsigned int int_mask;
	
    pin = iproc_irq_to_gpio(ourchip, irq);

    reg_offset = ASIU_GP_INT_MSK_0_BASE + ASIU_GPIO_REGIDX(pin)*ASIU_GPIO_REGOFFSET;
    int_mask = _iproc_gpio_readl(ourchip, reg_offset);
    int_mask &= ~(1 << ASIU_GPIO_REGBIT(pin));
    _iproc_gpio_writel(ourchip, int_mask, reg_offset);
}

static int asiu_gpio_irq_set_type(unsigned int irq, unsigned int type)
{
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin, reg_offset;    
    unsigned int  int_type, int_de, int_edge;
	
    pin = iproc_irq_to_gpio(ourchip, irq);
	
    reg_offset = ASIU_GPIO_REGIDX(pin)*ASIU_GPIO_REGOFFSET;
    
    int_type = _iproc_gpio_readl(ourchip, ASIU_GP_INT_TYPE_0_BASE + reg_offset);
    int_edge = _iproc_gpio_readl(ourchip, ASIU_GP_INT_EDGE_0_BASE + reg_offset);

    switch (type) {
    case IRQ_TYPE_EDGE_BOTH:
        int_type &= ~(1 << ASIU_GPIO_REGBIT(pin)); 
        int_de = _iproc_gpio_readl(ourchip, ASIU_GP_INT_DE_0_BASE + reg_offset);
        int_de |= (1 << ASIU_GPIO_REGBIT(pin)); 
        _iproc_gpio_writel(ourchip, int_de, ASIU_GP_INT_DE_0_BASE + reg_offset);
        break;
    case IRQ_TYPE_EDGE_RISING:
        int_type &= ~(1 << ASIU_GPIO_REGBIT(pin)); 
        int_edge |= (1 << ASIU_GPIO_REGBIT(pin)); 
		    
        int_de = _iproc_gpio_readl(ourchip, ASIU_GP_INT_DE_0_BASE + reg_offset);
        int_de  &= ~(1 << ASIU_GPIO_REGBIT(pin)); 
        _iproc_gpio_writel(ourchip, int_de, ASIU_GP_INT_DE_0_BASE + reg_offset);
        break;
    case IRQ_TYPE_EDGE_FALLING:
        int_type &= ~(1 << ASIU_GPIO_REGBIT(pin));
        int_edge &= ~(1 << ASIU_GPIO_REGBIT(pin));
		    
        int_de = _iproc_gpio_readl(ourchip, ASIU_GP_INT_DE_0_BASE + reg_offset);
        int_de  &= ~(1 << ASIU_GPIO_REGBIT(pin)); 
        _iproc_gpio_writel(ourchip, int_de, ASIU_GP_INT_DE_0_BASE + reg_offset);
        break;
    case IRQ_TYPE_LEVEL_HIGH:
        int_type |= (1 << ASIU_GPIO_REGBIT(pin)); 
        int_edge |= (1 << ASIU_GPIO_REGBIT(pin)); 
        break;
    case IRQ_TYPE_LEVEL_LOW:
        int_type |= (1 << ASIU_GPIO_REGBIT(pin)); 
        int_edge &= ~(1 << ASIU_GPIO_REGBIT(pin)); 
        break;
    default:
        printk(KERN_ERR "unsupport irq type !\n");
        return -EINVAL;
    }
    _iproc_gpio_writel(ourchip, int_type, ASIU_GP_INT_TYPE_0_BASE + reg_offset);
    _iproc_gpio_writel(ourchip, int_edge, ASIU_GP_INT_EDGE_0_BASE + reg_offset);
	
    return 0;
}

struct iproc_gpio_irqcfg asiu_gpio_irqcfg = {
    .flags = IRQF_NO_SUSPEND,
    .handler = asiu_gpio_irq_handler,
    .ack = asiu_gpio_irq_ack,
    .mask = asiu_gpio_irq_mask,
    .unmask = asiu_gpio_irq_unmask,
    .set_type = asiu_gpio_irq_set_type,
};
#endif /* ASIU_GPIO */


#if defined(AON_GPIO)
static irqreturn_t 
aon_gpio_irq_handler(int irq, void *dev)
{
    struct iproc_gpio_chip *ourchip = dev;
    int iter, max_pin;
    unsigned int  val;
    irqreturn_t rv;

/*
    if (readl(ourchip->dmu_ioaddr + CRMU_IPROC_MAIL_BOX0_BASE) != 0x4f495047)
    {
        return IRQ_NONE;
    }
*/

    val = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MSTAT);
    if(!val){
        rv = IRQ_NONE;
        goto clear_mailbox_irq;
    }

    max_pin = ourchip->pin_offset + ourchip->chip.ngpio;
    for (iter = ourchip->pin_offset; iter < max_pin; iter ++) {
        if (val & (1 << iter)) {
            generic_handle_irq(iproc_gpio_to_irq(ourchip, iter));
        }
    }
    
    rv = IRQ_HANDLED;

clear_mailbox_irq:
/*
    writel(0, ourchip->dmu_ioaddr + CRMU_IPROC_MAIL_BOX0_BASE);
    writel(0, ourchip->dmu_ioaddr + CRMU_IPROC_MAIL_BOX1_BASE);
*/
    writel(readl(ourchip->dmu_ioaddr + CRMU_IPROC_INTR_CLEAR_BASE) | (1 << CRMU_IPROC_INTR_CLEAR__IPROC_MAILBOX_INTR_CLR), 
            ourchip->dmu_ioaddr + CRMU_IPROC_INTR_CLEAR_BASE);		

    return rv;
}

static void aon_gpio_irq_ack(unsigned int irq)
{
    struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
    int pin;
    unsigned int int_clear = 0;

    pin = iproc_irq_to_gpio(ourchip, irq);


    int_clear |= (1 << pin);
    _iproc_gpio_writel(ourchip, int_clear, IPROC_GPIO_CCB_INT_CLR);

}

static void aon_gpio_irq_unmask(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	unsigned int int_mask;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
    int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MASK);
    int_mask |= (1 << pin);
    _iproc_gpio_writel(ourchip, int_mask, IPROC_GPIO_CCB_INT_MASK);
	
}

static void aon_gpio_irq_mask(unsigned int irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	unsigned int int_mask;
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
    int_mask = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_MASK);
    int_mask &= ~(1 << pin);
    _iproc_gpio_writel(ourchip, int_mask,IPROC_GPIO_CCB_INT_MASK);
}

static int aon_gpio_irq_set_type(unsigned int irq, unsigned int type)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;    
    unsigned int  int_type, int_de, int_edge;
	
	
	pin = iproc_irq_to_gpio(ourchip, irq);
	
    int_type = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_TYPE);
    int_edge = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_EDGE);
    switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
	    int_type &= ~(1 << pin); 
	    int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
	    int_de |= (1 << pin); 
	    _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
	    break;
	case IRQ_TYPE_EDGE_RISING:
	    int_type &= ~(1 << pin); 
	    int_edge |= (1 << pin); 
	    
	    int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
	    int_de  &= ~(1 << pin); 
	    _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
	    break;
	case IRQ_TYPE_EDGE_FALLING:
	    int_type &= ~(1 << pin);
	    int_edge &= ~(1 << pin);
		    
	    int_de = _iproc_gpio_readl(ourchip, IPROC_GPIO_CCB_INT_DE);
	    int_de  &= ~(1 << pin); 
	    _iproc_gpio_writel(ourchip, int_de, IPROC_GPIO_CCB_INT_DE);
	    break;
	case IRQ_TYPE_LEVEL_HIGH:
	    int_type |= (1 << pin); 
	    int_edge |= (1 << pin); 
	    break;
	case IRQ_TYPE_LEVEL_LOW:
	    int_type |= (1 << pin); 
	    int_edge &= ~(1 << pin); 
	    break;
	default:
	    printk(KERN_ERR "unsupport irq type !\n");
	    return -EINVAL;
    }
    _iproc_gpio_writel(ourchip, int_type, IPROC_GPIO_CCB_INT_TYPE);
    _iproc_gpio_writel(ourchip, int_edge, IPROC_GPIO_CCB_INT_EDGE);
	
	return 0;
}

struct iproc_gpio_irqcfg aon_gpio_irqcfg = {
    .flags = IRQF_NO_SUSPEND|IRQF_SHARED,
	.handler = aon_gpio_irq_handler,
	.ack = aon_gpio_irq_ack,
	.mask = aon_gpio_irq_mask,
	.unmask = aon_gpio_irq_unmask,
	.set_type = aon_gpio_irq_set_type,
};
#endif /* AON_GPIO */


static struct irq_chip iproc_gpio_irq_chip = {
    .name         = "IPROC-GPIO",
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 5)
    .ack      = (void *) iproc_gpio_irq_ack,
    .mask     = (void *) iproc_gpio_irq_mask,
    .unmask   = (void *) iproc_gpio_irq_unmask,
    .set_type = (void *) iproc_gpio_irq_set_type,
#else
    .irq_ack      = (void *) iproc_gpio_irq_ack,
    .irq_mask     = (void *) iproc_gpio_irq_mask,
    .irq_unmask   = (void *) iproc_gpio_irq_unmask,
    .irq_set_type = (void *) iproc_gpio_irq_set_type,
#endif
};

struct iproc_gpio_chip *iproc_gpios[IPROC_GPIO_END];

static __init void iproc_gpiolib_track(struct iproc_gpio_chip *chip)
{
    unsigned int gpn;
    int i;

    gpn = chip->chip.base;
    for (i = 0; i < chip->chip.ngpio; i++, gpn++) {
        BUG_ON(gpn >= ARRAY_SIZE(iproc_gpios));
        iproc_gpios[gpn] = chip;
    }
}

static int iproc_gpiolib_input(struct gpio_chip *chip, unsigned gpio)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags;
    unsigned int  val, nBitMask;
    int reg_offset;
    unsigned int pin_offset = gpio + ourchip->pin_offset;

    iproc_gpio_lock(ourchip, flags);

    if (ourchip->id == ASIU_GPIO_ID) {
    	nBitMask = 1 << ASIU_GPIO_REGBIT(pin_offset);
    	reg_offset = REGOFFSET_GPIO_EN + ASIU_GPIO_REGIDX(pin_offset)*ASIU_GPIO_REGOFFSET;
    }
    else {
        nBitMask = 1 << pin_offset;
	reg_offset = REGOFFSET_GPIO_EN;
    }

    val = _iproc_gpio_readl(ourchip, reg_offset);
    val &= ~nBitMask;
    _iproc_gpio_writel(ourchip, val, reg_offset);
    
    iproc_gpio_unlock(ourchip, flags);
    return 0;
}

static int iproc_gpiolib_output(struct gpio_chip *chip,
			      unsigned gpio, int value)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags, val;
    unsigned int nBitMask;
    int reg_offset;
    unsigned int pin_offset = gpio + ourchip->pin_offset;

    iproc_gpio_lock(ourchip, flags);

    if (ourchip->id == ASIU_GPIO_ID) {
    	nBitMask = 1 << ASIU_GPIO_REGBIT(pin_offset);
    	reg_offset = REGOFFSET_GPIO_EN + ASIU_GPIO_REGIDX(pin_offset)*ASIU_GPIO_REGOFFSET;
    }
    else {
        nBitMask = 1 << pin_offset;
	reg_offset = REGOFFSET_GPIO_EN;
    }

    val = _iproc_gpio_readl(ourchip, reg_offset);
    val |= nBitMask;
    _iproc_gpio_writel(ourchip, val, reg_offset);

    iproc_gpio_unlock(ourchip, flags);
    return 0;
}

static void iproc_gpiolib_set(struct gpio_chip *chip,
			    unsigned gpio, int value)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags, val;
    unsigned int nBitMask;
    int reg_offset = 0;
    unsigned int pin_offset = gpio + ourchip->pin_offset;

    iproc_gpio_lock(ourchip, flags);


    /* determine the GPIO pin direction 
     */ 
    if (ourchip->id == ASIU_GPIO_ID) {
	nBitMask = 1 << ASIU_GPIO_REGBIT(pin_offset);
	reg_offset = ASIU_GPIO_REGIDX(pin_offset)*ASIU_GPIO_REGOFFSET;
    } else {
	nBitMask = 1 << pin_offset;
    }
    val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_EN + reg_offset);
    val &= nBitMask;

    /* this function only applies to output pin
     */ 
    if (!val)
        return;

    val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DOUT + reg_offset);

    if ( value == 0 ){
        /* Set the pin to zero */
        val &= ~nBitMask;
    }else{
        /* Set the pin to 1 */
        val |= nBitMask;
    }    
    _iproc_gpio_writel(ourchip, val, REGOFFSET_GPIO_DOUT + reg_offset);

    iproc_gpio_unlock(ourchip, flags);

}


static int iproc_gpiolib_get(struct gpio_chip *chip, unsigned gpio)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    unsigned long flags;
    unsigned int val, offset, nBitMask;
    int reg_offset = 0;
    unsigned int pin_offset = gpio + ourchip->pin_offset;

    iproc_gpio_lock(ourchip, flags);

    if (ourchip->id == ASIU_GPIO_ID) {
	nBitMask = 1 << ASIU_GPIO_REGBIT(pin_offset);
    	reg_offset = ASIU_GPIO_REGIDX(pin_offset)*ASIU_GPIO_REGOFFSET;
    }
    else {
        nBitMask = 1 << pin_offset;
    }

    /* determine the GPIO pin direction 
     */ 
    offset = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_EN + reg_offset);
    offset &= nBitMask;

    if (offset){
        val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DOUT + reg_offset);
    } else {
        val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DIN + reg_offset);    
    }
    
    if (ourchip->id == ASIU_GPIO_ID) {
    	val >>= ASIU_GPIO_REGBIT(pin_offset);
    } else {	
        val >>= pin_offset;
    }

    val &= 1;
    
    iproc_gpio_unlock(ourchip, flags);

    return val;
}

/*
@offset : the gpio pin index number from gpiolib view (minus gpio base only)
*/
static int iproc_gpiolib_to_irq(struct gpio_chip *chip,
                unsigned offset)
{
    struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
    return iproc_gpio_to_irq(ourchip, offset + ourchip->pin_offset);
}

void __init  iproc_gpiolib_add(struct iproc_gpio_chip *chip)
{
    struct resource *res;
    struct gpio_chip *gc = &chip->chip;
    int ret, i;    

    BUG_ON(!gc->label);
    BUG_ON(!gc->ngpio);
    
    spin_lock_init(&chip->lock);
    
    if (!gc->direction_input)
        gc->direction_input = iproc_gpiolib_input;
    if (!gc->direction_output)
        gc->direction_output = iproc_gpiolib_output;
    if (!gc->set)
        gc->set = iproc_gpiolib_set;
    if (!gc->get)
        gc->get = iproc_gpiolib_get;
    if (!gc->to_irq)
        gc->to_irq = iproc_gpiolib_to_irq;
    if (!gc->request)
    	gc->request = 0;

    /* gpiochip_add() prints own failure message on error. */
    ret = gpiochip_add(gc);
    if (ret >= 0)
        iproc_gpiolib_track(chip);

    printk(KERN_INFO "iproc gpiochip add %s\n", gc->label);
    /* io remap */
    res = chip->resource;

    chip->ioaddr = ioremap_nocache(res->start, (res->end - res->start) + 1);
    printk(KERN_INFO "%s:ioaddr %p \n", gc->label, chip->ioaddr);
    chip->intr_ioaddr = NULL;
    chip->dmu_ioaddr = NULL;
    if(res->child){
        for (i=0; i< 2; i++){        
            if (!strcmp("intr", res->child[i].name)){
                chip->intr_ioaddr = 
                    ioremap_nocache(res->child[i].start, 
                    (res->child[i].end - res->child[i].start) + 1);
            }
            if (!strcmp("dmu", res->child[i].name)){
                chip->dmu_ioaddr = 
                    ioremap_nocache(res->child[i].start, 
                    (res->child[i].end - res->child[i].start) + 1);
            }
        }
        printk(KERN_INFO "%s:intr_ioaddr %p dmu_ioaddr %p\n",
            gc->label, chip->intr_ioaddr,chip->dmu_ioaddr);
    }


    if (chip->irq_base) {
        for (i = chip->irq_base; i < (chip->irq_base + gc->ngpio); i++) {
            irq_set_chip(i, &iproc_gpio_irq_chip);
            irq_set_chip_data(i,chip);
            irq_set_handler(i, handle_level_irq);
            set_irq_flags(i, IRQF_VALID);                            
    
        }
#if defined(IPROC_GPIO_CCA)	
        if (chip->id == IPROC_GPIO_CCA_ID ){
            unsigned int val;
            /* enable the GPIO in CCA interrupt mask */
            val = readl(chip->intr_ioaddr + IPROC_CCA_INT_MASK);
            val |= IPROC_CCA_INT_F_GPIOINT;
            writel(val, chip->intr_ioaddr + IPROC_CCA_INT_MASK);
        }
#endif	
	if (chip->irqcfg) {
	    struct iproc_gpio_irqcfg *irqcfg = chip->irqcfg;
	    if (irqcfg->handler) {
                ret = request_irq(chip->irq, irqcfg->handler, irqcfg->flags, 
		    gc->label, chip);
	        if (ret)
		    printk(KERN_ERR "Unable to request IRQ%d: %d\n",
		        chip->irq, ret);
	    }
	    else
	    	printk(KERN_ERR "%s is added without isr!\n", chip->chip.label);
	}
    }
    iproc_gpio_dev[dev] = chip;
    dev++;
	printk("\nset gpio7 irq\n");
	unsigned int irq_gpio7 = chip->irq_base+chip->pin_offset+7;
	iproc_gpio_dev[0]->irqcfg->set_type(irq_gpio7, IRQ_TYPE_EDGE_FALLING);
	iproc_gpio_dev[0]->irqcfg->ack(irq_gpio7);
	iproc_gpio_dev[0]->irqcfg->unmask(irq_gpio7);

}

#ifdef POWERDOWN_ALARM
void netlink_poweroff_init()
{
		struct netlink_kernel_cfg cfg = {
				.input	= recv_sock_handler,
		};

		netlink_sock = netlink_kernel_create(&init_net, NETLINK_POWEROFF, THIS_MODULE, &cfg);
		if (!netlink_sock) {
				printk("Fail to create netlink_sock socket.....\n");
		}
}
#endif
static int __init gpio_init(void)
{      
#ifdef POWERDOWN_ALARM
	netlink_poweroff_init();
#endif
    iproc_gpiolib_init();
    
    return 0;
}
static void __exit gpio_exit(void)
{
    int i=0;

    for (i = 0 ; i < MAX_NS_GPIO; i++) {        
        if(iproc_gpio_dev[i]){
            if(iproc_gpio_dev[i]->ioaddr){
                iounmap(iproc_gpio_dev[i]->ioaddr);
            }
            if(iproc_gpio_dev[i]->intr_ioaddr){
#if defined(IPROC_GPIO_CCA)	    	
                if (iproc_gpio_dev[i]->id == IPROC_GPIO_CCA_ID ){
                  unsigned int val;
                  val = readl(iproc_gpio_dev[i]->intr_ioaddr + IPROC_CCA_INT_MASK);
                  val &= ~(IPROC_CCA_INT_F_GPIOINT);
                  writel(val, iproc_gpio_dev[i]->intr_ioaddr + IPROC_CCA_INT_MASK);
                }
#endif		
                iounmap(iproc_gpio_dev[i]->intr_ioaddr);
            }
            if(iproc_gpio_dev[i]->dmu_ioaddr){
                iounmap(iproc_gpio_dev[i]->dmu_ioaddr);
            }                        
            if(iproc_gpio_dev[i]->irq_base) {
                free_irq(iproc_gpio_dev[i]->irq,iproc_gpio_dev[i]);
            }

            gpiochip_remove(&iproc_gpio_dev[i]->chip);
            iproc_gpio_dev[i] = NULL;
        }
    }
}

MODULE_DESCRIPTION("IPROC GPIO driver");
MODULE_LICENSE("GPL");

module_init(gpio_init);
module_exit(gpio_exit);
