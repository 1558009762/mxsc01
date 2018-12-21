#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/keyboard.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "mach/socregs-cygnus.h"
#include "mach/iproc_regs.h"
#include "touch_screen.h"

#define IPROC_TS_NAME "iproc-ts"

static int ts_dbg = 1;
module_param_named(ts_debug, ts_dbg, int, 0644);

#define TS_DEBUG if(ts_dbg) printk

typedef struct _TSC_reg
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
} tsc_reg_t;

struct iproc_ts_priv {
    struct platform_device *pdev;
	struct input_dev *idev;
	volatile tsc_reg_t *regs;
    volatile unsigned int *clk_gating;
    spinlock_t lock;
    int  fifo_thes;
    int  irqno;
    int  pen_status;
};

static irqreturn_t iproc_touchscreen_interrupt(int irq, void *data)
{
    struct platform_device *pdev = (struct platform_device *)data;
	struct iproc_ts_priv *priv = (struct iproc_ts_priv *)platform_get_drvdata(pdev);
    unsigned long flags;
    unsigned int intr_status = 0;
    int coordinate = 0;
    int x = 0, y = 0;
    int i;

    spin_lock_irqsave(&priv->lock, flags);
    intr_status = priv->regs->interrupt_Status;
    intr_status &= TS_PEN_INTR_MASK|TS_FIFO_INTR_MASK;
    priv->regs->interrupt_Status = intr_status; /*Clear all interrupt status bits, write-1-clear*/

    if(intr_status&TS_PEN_INTR_MASK)  /*Pen up/down*/
    {
        if(priv->regs->controller_Status&TS_PEN_DOWN)
            priv->pen_status = PEN_DOWN_STATUS;
        else
            priv->pen_status = PEN_UP_STATUS;

        input_report_key(priv->idev, BTN_TOUCH, priv->pen_status);
        //input_report_abs(priv->idev, ABS_PRESSURE, 1);
        input_sync(priv->idev);
    }

    if(intr_status&TS_FIFO_INTR_MASK) /*coordinates in FIFO exceed the theshold*/
    {
        for(i = 0; i < priv->fifo_thes; i++)
        {
            coordinate = priv->regs->FIFO_Data;
            if(coordinate == 0xFFFFFFFF)
                continue;
            x = coordinate&0xFFFF;
            y = (coordinate)>>16&0xFFFF;
            input_report_abs(priv->idev, ABS_X, x);
            input_report_abs(priv->idev, ABS_Y, y);
            input_sync(priv->idev);
        }
    }
    
    spin_unlock_irqrestore(&priv->lock, flags);
    
	return IRQ_HANDLED;
}

static int iproc_ts_open(struct input_dev *dev)
{
	struct iproc_ts_priv *priv = input_get_drvdata(dev);
    unsigned long flags;
    unsigned int v;

	spin_lock_irqsave(&priv->lock, flags);
    *((unsigned int *)priv->clk_gating) |= 1<<9; /*Disable ADC Clock Gating*/
    priv->regs->interrupt_Mask |= TS_PEN_INTR_MASK|   /*Interrupt is generated when the FIFO reaches the int_th value*/
                                  TS_FIFO_INTR_MASK;  /*Pen event(up/down)*/

    priv->regs->interrupt_Thres = priv->fifo_thes;
    //priv->regs->regCtl1 = 0xFFFF0BFF;  /*Using default value 0x0528070a*/
    v = priv->regs->regCtl2;
    v |= TS_CONTROLLER_EN_BIT|       /*enables the touch screen controller*/
         TS_WIRE_MODE_BIT;           /*4 Wire operation*/
    v &= ~(TS_CONTROLLER_PWR_LDO |   /*PWR up LDO*/
           TS_CONTROLLER_PWR_ADC |   /*PWR up ADC*/
           TS_CONTROLLER_PWR_BGP |   /*PWR up BGP*/
           TS_CONTROLLER_PWR_TS);    /*PWR up TS*/
    priv->regs->regCtl2 = v;
    priv->regs->interrupt_Status |= TS_FIFO_INTR_MASK|TS_PEN_INTR_MASK; /*Try to clear all interrupt status*/
	spin_unlock_irqrestore(&priv->lock, flags);
    TS_DEBUG("Touch screen device opened\n");
	return 0;
}

static void iproc_ts_close(struct input_dev *dev)
{
    struct iproc_ts_priv *priv = input_get_drvdata(dev);
    unsigned long flags;

    spin_lock_irqsave(&priv->lock, flags);
    priv->regs->interrupt_Mask = 0; /*Disable all interrupts*/
    /*priv->regs->regCtl2 &= ~TS_CONTROLLER_EN_BIT;*/ /*ADC also need this bit set, so we can't disable it*/
    priv->regs->regCtl2 |= TS_CONTROLLER_PWR_TS; /*Only powerdown touch screen controller*/
    spin_unlock_irqrestore(&priv->lock, flags);
    TS_DEBUG("Touch screen device closed\n");
}

static void ts_reg_dump(struct iproc_ts_priv *priv)
{
    printk("regCtl1             = 0x%08x\n", priv->regs->regCtl1);
    printk("regCtl2             = 0x%08x\n", priv->regs->regCtl2);
    printk("interrupt_Thres     = 0x%08x\n", priv->regs->interrupt_Thres);
    printk("interrupt_Mask      = 0x%08x\n", priv->regs->interrupt_Mask);
    printk("interrupt_Status    = 0x%08x\n", priv->regs->interrupt_Status);
    printk("controller_Status   = 0x%08x\n", priv->regs->controller_Status);
    printk("FIFO_Data           = 0x%08x\n", priv->regs->FIFO_Data);
    printk("analog_Control      = 0x%08x\n", priv->regs->analog_Control);
    printk("aux_Data            = 0x%08x\n", priv->regs->aux_Data);
    printk("debounce_Cntr_Stat  = 0x%08x\n", priv->regs->debounce_Cntr_Stat);
    printk("scan_Cntr_Stat      = 0x%08x\n", priv->regs->scan_Cntr_Stat);
    printk("rem_Cntr_Stat       = 0x%08x\n", priv->regs->rem_Cntr_Stat);
    printk("settling_Timer_Stat = 0x%08x\n", priv->regs->settling_Timer_Stat);
    printk("spare_Reg           = 0x%08x\n", priv->regs->spare_Reg);
    printk("soft_Bypass_Control = 0x%08x\n", priv->regs->soft_Bypass_Control);
    printk("soft_Bypass_Data    = 0x%08x\n", priv->regs->soft_Bypass_Data);
}

static int __init iproc_ts_probe(struct platform_device *pdev)
{
	struct iproc_ts_priv *priv;
	struct input_dev *idev;
    struct resource *res = NULL;
	int ret = -ENOMEM;

    priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if(!priv)
    {
        TS_DEBUG("Allocate iproc_ts_priv structure failed\n");
        goto fail;
    }

    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if(!res)
    {
        goto fail0;
    }
    priv->irqno = res->start;
    
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        ret = -ENXIO;
        goto fail0;
    }

    priv->regs = (volatile tsc_reg_t *)ioremap(res->start, resource_size(res));
    if(!priv->regs)
    {
        TS_DEBUG("IO remap failed\n");
        ret = -ENXIO;
        goto fail0;
    }
    TS_DEBUG("ts_regs = 0x%08x\n", (unsigned int)priv->regs);
    
    priv->clk_gating = (volatile unsigned int *)ioremap(ASIU_TOP_CLK_GATING_CTRL, 0x4);
    if(!priv->clk_gating)
    {
        TS_DEBUG("IO remap failed\n");
        ret = -ENXIO;
        goto fail1;
    }
    TS_DEBUG("clk_gating = 0x%08x\n", (unsigned int)priv->clk_gating);
    
    if(ts_dbg)
        ts_reg_dump(priv);
    
	idev = input_allocate_device();
	if (!idev)
    {
        TS_DEBUG("Allocate input device failed\n");
		goto fail2;
    }

    spin_lock_init(&priv->lock);
	priv->idev = idev;
    priv->pdev = pdev;
    priv->pen_status = PEN_UP_STATUS;
    priv->fifo_thes = TS_DEFAULT_FIFO_THES;
	idev->name = IPROC_TS_NAME;
	idev->dev.parent = &pdev->dev;

	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	//idev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	set_bit(BTN_TOUCH, idev->keybit);
	input_set_abs_params(idev, ABS_X, X_MIN, X_MAX, 0, 0);
	input_set_abs_params(idev, ABS_Y, Y_MIN, Y_MAX, 0, 0);
    //input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0);

	idev->open = iproc_ts_open;
	idev->close = iproc_ts_close;

	input_set_drvdata(idev, priv);

	ret = input_register_device(priv->idev);
	if (ret) {
		dev_err(&pdev->dev, "register input device failed with %d\n", ret);
		goto fail3;
	}

    ret = request_irq(priv->irqno, iproc_touchscreen_interrupt,
        			IRQF_DISABLED | IRQF_SHARED, "iproc-tsc", pdev);
	platform_set_drvdata(pdev, priv);
	return 0;

fail3:
	input_free_device(idev);
fail2:
    iounmap(priv->clk_gating);
fail1:
    iounmap(priv->regs);
fail0:
    kfree(priv);
fail:    
	return ret;
}

static int __devexit iproc_ts_remove(struct platform_device *pdev)
{
	struct iproc_ts_priv *priv = platform_get_drvdata(pdev);

    free_irq(priv->irqno, pdev);
	platform_set_drvdata(pdev, NULL);
	input_unregister_device(priv->idev);
    iounmap(priv->clk_gating);
    iounmap(priv->regs);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
void iproc_ts_shutdown(struct platform_device *pdev)
{
    struct iproc_ts_priv *priv = platform_get_drvdata(pdev);
    unsigned long flags;

    spin_lock_irqsave(&priv->lock, flags);
    priv->regs->regCtl2 |= TS_CONTROLLER_PWR_TS; /*Only powerdown touch screen controller*/
    spin_unlock_irqrestore(&priv->lock, flags);
    TS_DEBUG("Touch screen power down\n");
}


int iproc_ts_resume(struct platform_device *pdev)
{
    struct iproc_ts_priv *priv = platform_get_drvdata(pdev);
    unsigned long flags;

    spin_lock_irqsave(&priv->lock, flags);
    priv->regs->regCtl2 &= ~TS_CONTROLLER_PWR_TS; /*Only powerdown touch screen controller*/
    spin_unlock_irqrestore(&priv->lock, flags);
    TS_DEBUG("Touch screen power up\n");
    return 0;
}
#endif

static struct resource tsc_resources[] = {
        [0] = {
                .start  = IPROC_TSC_REG_BASE,
                .end    = IPROC_TSC_REG_BASE+0x3F,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = IPROC_TSC_INTR,
                .end    = IPROC_TSC_INTR,
                .flags  = IORESOURCE_IRQ,
        }
};

void platform_tsc_release(struct device *pdev)
{
    return;
}

static struct platform_device tsc_device = {
        .name           =       "iproc-tsc",
        .id             =       -1,
        .dev            =       {
                .platform_data  = NULL,
                .release = platform_tsc_release
        },
        .num_resources  = ARRAY_SIZE(tsc_resources),
        .resource       = tsc_resources,
};

static struct platform_driver iproc_ts_driver = {
	.remove		= __devexit_p(iproc_ts_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "iproc-tsc",
	},
#ifdef CONFIG_PM	
	.shutdown   = iproc_ts_shutdown,
	.resume     = iproc_ts_resume,
#endif	
};

static int __init iproc_ts_init(void)
{
    platform_device_register(&tsc_device);
	return platform_driver_probe(&iproc_ts_driver, &iproc_ts_probe);
}

static void __exit iproc_ts_exit(void)
{
	platform_driver_unregister(&iproc_ts_driver);
    platform_device_unregister(&tsc_device);
}

module_init(iproc_ts_init);
module_exit(iproc_ts_exit);
MODULE_DESCRIPTION("IPROC Touchscreen driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL");


