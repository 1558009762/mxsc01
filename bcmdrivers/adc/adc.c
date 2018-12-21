#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/keyboard.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "mach/socregs-cygnus.h"
#include "mach/iproc_regs.h"
#include "adc.h"

typedef struct _ADC_reg
{
    unsigned int regCtl1;
    unsigned int regCtl2;
    unsigned int interrupt_Thres;
    unsigned int interrupt_Mask;
    unsigned int interrupt_Status;
    unsigned int controller_Status;
    unsigned int FIFO_Data;
    unsigned int analog_Control;
    unsigned int aux_Data;
    unsigned int debounce_Cntr_Stat;
    unsigned int scan_Cntr_Stat;
    unsigned int rem_Cntr_Stat;
    unsigned int settling_Timer_Stat;
    unsigned int spare_Reg;
    unsigned int soft_Bypass_Control;
    unsigned int soft_Bypass_Data;
} adc_reg_t;

typedef struct iproc_adc_priv {
    struct platform_device *pdev;
	struct cdev *cdev;
    struct class *adc_class;
	volatile adc_reg_t *regs;
    volatile unsigned int *clk_gating;
    spinlock_t lock;
    struct mutex mutex;
    int  irqno;
    dev_t devt;
    int cs; /*Channel select*/
    int cv; /*Channel value*/
    void *read_complete;
} iproc_adc_priv_t;

iproc_adc_priv_t *adc_priv = NULL;

static irqreturn_t iproc_adc_interrupt(int irq, void *data)
{
    /*struct platform_device *pdev = (struct platform_device *)data;*/
    unsigned long flags;
    unsigned int intr_status = 0;

    spin_lock_irqsave(&adc_priv->lock, flags);
    intr_status = adc_priv->regs->interrupt_Status;
    
    if(intr_status&ADC_AUXData_RDY_INTR)  /*ADC AUX data is ready*/
    {
        adc_priv->regs->interrupt_Status = ADC_AUXData_RDY_INTR; /*Clear all interrupt status bits, write-1-clear*/
        if(adc_priv->cs != -1)
        {
            adc_priv->cv = adc_priv->regs->aux_Data&0xFFFF;
            if(adc_priv->read_complete)
                complete(adc_priv->read_complete);
        }
    }

    spin_unlock_irqrestore(&adc_priv->lock, flags);
    
	return IRQ_HANDLED;
}

static int adc_read(struct file *fp, char *buff, size_t size, loff_t *offset)
{
    struct inode *inode;
    int minor;
    unsigned int analog_ctrl = 0;
    int read_len = 0;
    DECLARE_COMPLETION_ONSTACK(read_done);

    inode = fp->f_dentry->d_inode;
    minor = iminor(inode);
    adc_priv->read_complete = &read_done;
    mutex_lock(&adc_priv->mutex);
    adc_priv->cs = minor;
    adc_priv->cv = -1;
    analog_ctrl = adc_priv->regs->analog_Control;
    analog_ctrl &= ~(7<<11);
    analog_ctrl |= adc_priv->cs<<11;
    adc_priv->regs->interrupt_Status = ADC_AUXData_RDY_INTR;
    adc_priv->regs->interrupt_Mask |= ADC_AUXData_RDY_INTR; /*Interrupt is generated when the aux data is available on auxin_dout*/
    adc_priv->regs->analog_Control = analog_ctrl;
    wait_for_completion_timeout(&read_done, ADC_READ_TIMEOUT);
    if(adc_priv->cv != -1)
    {
        read_len = 4;
        copy_to_user(buff, &adc_priv->cv, 4);
    }
    adc_priv->regs->interrupt_Mask &= ~ADC_AUXData_RDY_INTR; /*Disable AUX interrupt*/
    mutex_unlock(&adc_priv->mutex);

	return read_len;
}

static int adc_open(struct inode *pnode, struct file *fp)
{
    return 0;
}

static int adc_release(struct inode *pnode, struct file *fp)
{
    mutex_lock(&adc_priv->mutex);
    adc_priv->read_complete = NULL;
    adc_priv->cs = -1;
    adc_priv->cv = -1;
    mutex_unlock(&adc_priv->mutex);

    return 0;
}

static void adc_reg_dump(void)
{
    printk("regCtl1             = 0x%08x\n", adc_priv->regs->regCtl1);
    printk("regCtl2             = 0x%08x\n", adc_priv->regs->regCtl2);
    printk("interrupt_Thres     = 0x%08x\n", adc_priv->regs->interrupt_Thres);
    printk("interrupt_Mask      = 0x%08x\n", adc_priv->regs->interrupt_Mask);
    printk("interrupt_Status    = 0x%08x\n", adc_priv->regs->interrupt_Status);
    printk("controller_Status   = 0x%08x\n", adc_priv->regs->controller_Status);
    printk("FIFO_Data           = 0x%08x\n", adc_priv->regs->FIFO_Data);
    printk("analog_Control      = 0x%08x\n", adc_priv->regs->analog_Control);
    printk("aux_Data            = 0x%08x\n", adc_priv->regs->aux_Data);
    printk("debounce_Cntr_Stat  = 0x%08x\n", adc_priv->regs->debounce_Cntr_Stat);
    printk("scan_Cntr_Stat      = 0x%08x\n", adc_priv->regs->scan_Cntr_Stat);
    printk("rem_Cntr_Stat       = 0x%08x\n", adc_priv->regs->rem_Cntr_Stat);
    printk("settling_Timer_Stat = 0x%08x\n", adc_priv->regs->settling_Timer_Stat);
    printk("spare_Reg           = 0x%08x\n", adc_priv->regs->spare_Reg);
    printk("soft_Bypass_Control = 0x%08x\n", adc_priv->regs->soft_Bypass_Control);
    printk("soft_Bypass_Data    = 0x%08x\n", adc_priv->regs->soft_Bypass_Data);
}

static const struct file_operations adc_fops = {
        .owner          = THIS_MODULE,
        .open           = adc_open,
        .release        = adc_release,
        .read           = adc_read,
};

static void iproc_adc_enable(void)
{
    unsigned long flags;

    spin_lock_irqsave(&adc_priv->lock, flags);
    *((volatile unsigned int *)adc_priv->clk_gating) |= 1<<9;
    adc_priv->regs->analog_Control &= ~(7<<11); /*Set i_amux = 3b'000, select channel 0*/
    adc_priv->cs = -1;
    adc_priv->cv = -1;
    adc_priv->regs->regCtl2 &= ~(ADC_PWR_LDO |  /*PWR up LDO*/
                            ADC_PWR_ADC |   /*PWR up ADC*/
                            ADC_PWR_BG);    /*PWR up Band Gap*/
    adc_priv->regs->regCtl2 |= ADC_CONTROLLER_EN;
    //adc_priv->regs->regCtl1 = 0xFFFF0BFF; /*Using default value 0x0528070a*/
    adc_priv->regs->interrupt_Status |= ADC_AUXData_RDY_INTR;/*Try to clear all interrupt status*/
    request_irq(adc_priv->irqno, iproc_adc_interrupt,
        			IRQF_DISABLED | IRQF_SHARED, "iproc-adc", adc_priv->pdev);
    spin_unlock_irqrestore(&adc_priv->lock, flags);

    return;
}

static int __init iproc_adc_probe(struct platform_device *pdev)
{
	struct cdev *cdev;
    struct resource *res = NULL;
	int ret = -ENOMEM;
    dev_t devt;
    struct class *adc_class;
    int minor;

    adc_priv = kzalloc(sizeof(iproc_adc_priv_t), GFP_KERNEL);
    if(!adc_priv)
    {
        goto fail;
    }

    mutex_init(&adc_priv->mutex);
    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if(!res)
    {
        goto fail0;
    }
    adc_priv->irqno = res->start;
    
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        ret = -ENXIO;
        goto fail0;
    }

    adc_priv->regs = (volatile adc_reg_t *)ioremap(res->start, resource_size(res));
    if(!adc_priv->regs)
    {
        ret = -ENXIO;
        goto fail0;
    }

    adc_priv->clk_gating = (volatile unsigned int *)ioremap(ASIU_TOP_CLK_GATING_CTRL, 0x4);
    if(!adc_priv->clk_gating)
    {
        ret = -ENXIO;
        goto fail1;
    }
    adc_reg_dump();

    cdev = (struct cdev *)cdev_alloc();
	if (!cdev)
		goto fail2;

    cdev->ops = &adc_fops;
    cdev->owner = THIS_MODULE;
    spin_lock_init(&adc_priv->lock);
	adc_priv->cdev = cdev;
    adc_priv->pdev = pdev;
    if(alloc_chrdev_region(&devt, 0, MAX_ADC_AUX_CHANNEL, ADC_CHANNEL_NAME))
    {
        printk(KERN_INFO"Alloc char device region failed\n");
        goto fail3;
    }
    if(cdev_add(cdev, devt, MAX_ADC_AUX_CHANNEL))
    {
        goto fail4;
    }
    adc_class = class_create(THIS_MODULE, ADC_CHANNEL_NAME);
    adc_priv->adc_class = adc_class;
    for(minor = 0; minor < MAX_ADC_AUX_CHANNEL; minor++)
        device_create(adc_class, NULL, MKDEV(MAJOR(devt), minor), NULL, ADC_CHANNEL_NAME"%d", minor); 
	platform_set_drvdata(pdev, adc_priv);
    adc_priv->devt = devt;
    iproc_adc_enable();
	return 0;

fail4:
    unregister_chrdev_region(devt, MAX_ADC_AUX_CHANNEL);
fail3:
	cdev_del(cdev);
fail2:
    iounmap(adc_priv->clk_gating);
fail1:
    iounmap(adc_priv->regs);
fail0:
    kfree(adc_priv);
fail:    
	return ret;
}

static int __devexit iproc_adc_remove(struct platform_device *pdev)
{
    int minor;
	for(minor = 0; minor < MAX_ADC_AUX_CHANNEL; minor++)
		device_destroy(adc_priv->adc_class, MKDEV(MAJOR(adc_priv->devt), minor));
	
	class_destroy(adc_priv->adc_class);
    unregister_chrdev_region(adc_priv->devt, MAX_ADC_AUX_CHANNEL);
	platform_set_drvdata(pdev, NULL);
	cdev_del(adc_priv->cdev);
    iounmap(adc_priv->clk_gating);
    iounmap(adc_priv->regs);
    free_irq(adc_priv->irqno, pdev);
	kfree(adc_priv);

	return 0;
}

#ifdef CONFIG_PM
void iproc_adc_shutdown(struct platform_device *pdev)
{
}


int iproc_adc_resume(struct platform_device *pdev)
{
    return 0;
}
#endif

static struct resource adc_resources[] = {
        [0] = {
                .start  = IPROC_ADC_REG_BASE,
                .end    = IPROC_ADC_REG_BASE+0x3F,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = IPROC_ADC_INTR,
                .end    = IPROC_ADC_INTR,
                .flags  = IORESOURCE_IRQ,
        }
};

void platform_adc_release(struct device *dev)
{
    return;
}

static struct platform_device adc_device = {
        .name           =       "iproc-adc",
        .id             =       -1,
        .dev            =       {
                .platform_data  = NULL,
                .release = platform_adc_release,
        },
        .num_resources  = ARRAY_SIZE(adc_resources),
        .resource       = adc_resources,
};

static struct platform_driver iproc_adc_driver = {
	.remove		= __devexit_p(iproc_adc_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "iproc-adc",
	},
#ifdef CONFIG_PM	
	.shutdown   = iproc_adc_shutdown,
	.resume     = iproc_adc_resume,
#endif	
};

static int __init iproc_adc_init(void)
{
    platform_device_register(&adc_device);
	return platform_driver_probe(&iproc_adc_driver, &iproc_adc_probe);
}

static void __exit iproc_adc_exit(void)
{
	platform_driver_unregister(&iproc_adc_driver);
    platform_device_unregister(&adc_device);
}

module_init(iproc_adc_init);
module_exit(iproc_adc_exit);
MODULE_DESCRIPTION("IPROC ADC driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");



