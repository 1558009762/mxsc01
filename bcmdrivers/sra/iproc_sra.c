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
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>


#include <typedefs.h>
#include "iproc_sra.h"
#include "iproc_sra_dev.h"


#define SRA_IOC_OP_READ     0
#define SRA_IOC_OP_WRITE    1


uint32 sra_msg_level =
#ifdef BCMDBG
	1;
#else
	0;
#endif /* BCMDBG */


#define DEBUG_LEVEL		KERN_INFO
#define DRIVER_VERSION          "0.01"
#define DRIVER_NAME             "iproc sra"

static int sra_major;
static struct cdev sra_cdev;


struct sra_device_data {
    sra_info_t *sra;
    int init;
};

static struct sra_device_data sra_devices={0};


static
void _switch_interface_reset(void *rinfo)
{
    uint32 timeout;
    sra_info_t *sra = (sra_info_t *)rinfo;
    srabregs_t *regs;      /* pointer to chip registers */
   
    regs = sra->srabregs;

    /* Wait for switch initialization complete */
    timeout = IF_TIMEOUT;
    while (!(R_REG(sra, &regs->ctrl_if) & 
        CTRL_IF_SW_INIT_DONE_MASK)) {
        if (!timeout--) {
            SRA_ERROR(("srab reset switch interface: timeout"));
            break;
        }
    }

    /* Set the SRAU reset bit */
    W_REG(sra, &regs->cmdstat, 
        CMDSTAT_SRA_RST_MASK);
    /* Wait for it to auto-clear */
    timeout = IF_TIMEOUT;
    while (R_REG(sra, &regs->cmdstat) & 
        CMDSTAT_SRA_RST_MASK) {
        if (!timeout--) {
            SRA_ERROR(("srab reset switch interface: timeout sra_rst"));
            return;
        }
    }
}

void _switch_request_grant(void *rinfo)
{
    uint32 regval;
    uint32 timeout = IF_TIMEOUT;
    sra_info_t *sra = (sra_info_t *)rinfo;      
    srabregs_t *regs;      /* pointer to chip registers */

    regs = sra->srabregs;

	regval = R_REG(sra, &regs->ctrl_if);
    regval |= CTRL_IF_RCAREQ_MASK;

    W_REG(sra, &regs->ctrl_if, regval);
    while (!(R_REG(sra, &regs->ctrl_if) & 
        CTRL_IF_RCAGNT_MASK)) {
        if (!timeout--) {
            SRA_ERROR(("srab request grant: timeout"));
            return;
        }
    }
}

static
void _switch_release_grant(void *rinfo)
{
    uint32 regval;
    sra_info_t *sra = (sra_info_t *)rinfo;    
    srabregs_t *regs;      /* pointer to chip registers */

    regs = sra->srabregs;

	regval = R_REG(sra, &regs->ctrl_if);
    regval &= ~CTRL_IF_RCAREQ_MASK;
    W_REG(sra, &regs->ctrl_if, regval);
}


static
uint64 _switch_reg_read(void *rinfo, uint8 page, uint8 offset)
{
    uint64 value = ~(uint64)0;
    uint32 regval;
    uint32 timeout = POLL_TIMEOUT;
    sra_info_t *sra = (sra_info_t *)rinfo;       
    srabregs_t *regs;      /* pointer to chip registers */
    unsigned long flags;

    spin_lock_irqsave(&sra->lock, flags);


    regs = sra->srabregs;

    /* Assemble read command */
    _switch_request_grant(rinfo);
    regval = ((page << CMDSTAT_SRA_PAGE_SHIFT)
              | (offset << CMDSTAT_SRA_OFFSET_SHIFT)
              | CMDSTAT_SRA_GORDYN_MASK);
    W_REG(o, &regs->cmdstat, regval);

    /* Wait for command complete */
    while (R_REG(sra, &regs->cmdstat) & 
        CMDSTAT_SRA_GORDYN_MASK) {
        if (!--timeout) {
			SRA_ERROR(("robo_read: timeout"));
            _switch_interface_reset(rinfo);
            break;
        }
    }
    if (timeout) {
        /* Didn't time out, read and return the value */
        value = (((uint64)R_REG(sra, &regs->rd_h)) << 32)
                        | R_REG(sra, &regs->rd_l);
    }

    _switch_release_grant(rinfo);
    spin_unlock_irqrestore(&sra->lock, flags);

    return value;
}

static
void _switch_reg_write(void *rinfo, uint8 page, uint8 offset, uint64 value)
{
    uint32 regval;
    uint32 timeout = POLL_TIMEOUT;
    sra_info_t *sra = (sra_info_t *)rinfo;
    srabregs_t *regs;      /* pointer to chip registers */
    unsigned long flags;

    spin_lock_irqsave(&sra->lock, flags);


    regs = sra->srabregs;

    _switch_request_grant(rinfo);
    /* Load the value to write */
    W_REG(sra, &regs->wd_h, (uint32)(value >> 32));
    W_REG(sra, &regs->wd_l, (uint32)(value));

    /* Issue the write command */
    regval = ((page << CMDSTAT_SRA_PAGE_SHIFT)
              | (offset << CMDSTAT_SRA_OFFSET_SHIFT)
              | CMDSTAT_SRA_GORDYN_MASK
              | CMDSTAT_SRA_WRITE_MASK);
    W_REG(sra, &regs->cmdstat, regval);
    /* Wait for command complete */
    while (R_REG(sra, &regs->cmdstat) & 
        CMDSTAT_SRA_GORDYN_MASK) {
        if (!--timeout) {
			SRA_ERROR(("robo_write: timeout"));
            _switch_interface_reset(rinfo);
            break;
        }
    }
    _switch_release_grant(rinfo);

    spin_unlock_irqrestore(&sra->lock, flags);

}


static int sra_message(sra_info_t *sra,
		struct sra_ioc_transfer *u_xfers, unsigned n_xfers, int op)
{

    uint8   page, offset;
    int     len;
    uint64  regval;

    page = u_xfers->page;
    offset = u_xfers->offset;
    len = u_xfers->len;

    if(op == SRA_IOC_OP_READ) {

        regval = _switch_reg_read(sra, page, offset);

        u_xfers->rx_buf = regval;

        if (len > 4) {
            u_xfers->len = 8;
        } else {
            u_xfers->len = 4;
        }
    }

    if(op == SRA_IOC_OP_WRITE) {
        regval = u_xfers->tx_buf;
        _switch_reg_write(sra, page, offset, regval);

    }
    return 0;
}

static int
_srab_attach(sra_info_t *sra)
{
    sra->srabregs = (srabregs_t *)REG_MAP(IPROC_CHIPCB_SRAB, CORE_SIZE);

	/* Initialize lock */
	spin_lock_init(&sra->lock);
    return 0;
}

static int
_srab_deattach(sra_info_t *sra)
{
	if (sra->srabregs)
		REG_UNMAP(sra->srabregs);

	MFREE(sra);

    return 0;
}
static int 
sra_open(struct inode *inode, struct file *filp)
{
    filp->private_data = sra_devices.sra;
    return 0;
}

static int 
sra_release(struct inode *inode, struct file *filp)
{

    return 0;
}    

static long
sra_ioctl(struct file *filp,
		  unsigned int cmd, unsigned long arg)
{
    int         err = 0;
    int         retval = 0;
    int         ioc_op = 0;
    uint32      tmp;
    unsigned    n_ioc;
    struct sra_ioc_transfer	*ioc, *uf;    
    sra_info_t *sra;

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SRA_IOC_MAGIC){
        return -ENOTTY;
    }
    
    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
            (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
            (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    sra = (sra_info_t *)filp->private_data;

    ioc_op = SRA_IOC_OP_READ;

	switch (cmd) {

    case SRA_IOC_W_REG:
        ioc_op = SRA_IOC_OP_WRITE;
    case SRA_IOC_R_REG:        
        tmp = _IOC_SIZE(cmd);
        if ((tmp % sizeof(struct sra_ioc_transfer)) != 0) {
            retval = -EINVAL;
            break;
        }
        n_ioc = tmp / sizeof(struct sra_ioc_transfer);
        if (n_ioc == 0)
            break;

        /* copy into scratch area */
        ioc = kmalloc(tmp, GFP_KERNEL);
        if (!ioc) {
            retval = -ENOMEM;
            break;
        }
        if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
            kfree(ioc);
            retval = -EFAULT;
            break;
        }    
        /* translate to sra_message, execute */
        retval = sra_message(sra, ioc, n_ioc, ioc_op);    
        if(ioc_op == SRA_IOC_OP_READ) {
    
            uf = (struct sra_ioc_transfer *)arg;
            if (__copy_to_user((u8 __user *)&uf->rx_buf, (uint8  *)&ioc->rx_buf, ioc->len)) {
                kfree(ioc);
                retval = -EFAULT;
                break;
            }
        }
        kfree(ioc);
        break;
    }

    return 0;
}

static const struct file_operations sra_fops = {
    .open       = sra_open,
    .release    = sra_release,
    .unlocked_ioctl      = sra_ioctl,	    
    .owner      = THIS_MODULE,
};

static int _ns_switch_init(sra_info_t *sra)
{
    uint8 port;

    /* reset switch */
    _switch_reg_write(sra, 0, 0x79, 0x90);
    /* per port control: no stp */
    for( port = 0; port < 8; port++) {        
        _switch_reg_write(sra, 0, port, 0x00);
    }
    /* IMP port */
    _switch_reg_write(sra, 0, 0x8, 0x1c);
    /* IMP port force-link 1G*/
    _switch_reg_write(sra, 0, 0xe, 0xbb);
    /* port5 force-link 1G*/
    _switch_reg_write(sra, 0, 0x5d, 0x7b);
    /* port7 force-link 1G*/
    _switch_reg_write(sra, 0, 0x5f, 0x7b);
    /* management mode*/
    _switch_reg_write(sra, 0, 0xb, 0x7);
    /* disable BRCM tag */
    _switch_reg_write(sra, 0x2, 0x3, 0x0);
    /* enable IMP=port8 */
    _switch_reg_write(sra, 0x2, 0x0, 0x80);
    return 0;
}
static int __init robo_switch_init(void)
{
	sra_info_t *sra;
    
    sra = MALLOC(sizeof(sra_info_t));
    if (sra == NULL) {
        SRA_ERROR(("sra_init: out of memory\n"));
        return -ENOMEM;
    }
    memset(sra, 0, sizeof(sra_info_t));

    _srab_attach(sra);
    
    sra_devices.sra = sra;
    sra_devices.init = 1;

    _ns_switch_init(sra);
    return 0;
}

static int __init sra_init(void)
{
    dev_t sra_dev;
    sra_info_t *sra=NULL;
    int ret = -ENODEV;

    printk(DEBUG_LEVEL "%s: init version %s (built %s, %s)\n",
        DRIVER_NAME, DRIVER_VERSION, __DATE__, __TIME__);

#ifdef MODULE
    robo_switch_init();
#endif

    if(sra_devices.init != 1) {
        return -ENOMEM;
    }
    sra = sra_devices.sra;

    if (sra_major) {
        sra_dev = MKDEV(sra_major, 0);
        ret = register_chrdev_region(sra_dev, 
                        1, "sra");
    } else {        
        ret = alloc_chrdev_region(&sra_dev, 0, 
                        1, "sra");
        sra_major = MAJOR(sra_dev);
    }

    if (ret) {
        goto error;
    }
    cdev_init(&sra_cdev, &sra_fops);    
    ret = cdev_add(&sra_cdev, sra_dev, 1);
    if (ret) {
        printk(KERN_ERR "Fail to add sra char dev!\n");
        goto error_region;
    }

    return 0;

error_region:
    unregister_chrdev_region(sra_dev, 1);  
error:
    MFREE(sra);
    return ret;

}

static void __exit sra_exit(void)
{
    sra_info_t *sra=NULL;
    
    sra = sra_devices.sra;
    _srab_deattach(sra);
    sra_devices.sra = NULL;
    sra_devices.init = 0;
    unregister_chrdev_region(MKDEV(sra_major, 0), 1);

}

#ifndef MODULE
fs_initcall(robo_switch_init);
#endif

MODULE_DESCRIPTION("SRA (Switch Register Access) driver");
MODULE_LICENSE("Proprietary");


module_init(sra_init);
module_exit(sra_exit);
