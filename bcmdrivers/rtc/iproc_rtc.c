#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>

#include "iproc_rtc.h"

#define RTC_REG_ACC_DONE        REG_BIT(0)
#define RTC_REG_RTC_STOP        REG_BIT(0)
#define RTC_REG_PERIO_INTR      REG_BIT(0)
#define RTC_REG_ALARM_INTR      REG_BIT(1)
#define RTC_IND_SOFT_RST_N      REG_BIT(10)
#define RTC_REG_WR_CMD          REG_BIT(11)
#define RTC_REG_RD_CMD          REG_BIT(12)

typedef struct rtc_regs
{
    unsigned int reg_SPRU_BBL_WDATA;
    unsigned int reg_SPRU_BBL_CMD;
    unsigned int reg_SPRU_BBL_STATUS;
    unsigned int reg_SPRU_BBL_RDATA;
} rtc_reg_t;

typedef struct crmu_regs
{
    unsigned int reg_CRMU_PWR_GOOD_STATUS;
    unsigned int reg_CRMU_POWER_REQ_CFG;
    unsigned int reg_CRMU_POWER_POLL;
    unsigned int reg_CRMU_ISO_CELL_CONTROL;
    unsigned int rsvd;
    unsigned int reg_CRMU_SPRU_SOURCE_SEL_STAT;
} crmu_reg_t;

typedef struct auth_regs
{
    unsigned int reg_CRMU_BBL_AUTH_CODE;
    unsigned int reg_CRMU_BBL_AUTH_CHECK;
} bbl_auth_t;

typedef struct _iproc_rtc_t {
	struct rtc_device  *rtc;
	volatile rtc_reg_t *regs;
    volatile crmu_reg_t *crmu_regs;
    volatile bbl_auth_t *auth_regs;
	int				   peridic_irq;
    int                alarm_irq;
    spinlock_t         lock;
} iproc_rtc_t;

iproc_rtc_t iproc_rtc;

static inline int wait_acc_done(void)
{
    unsigned int v;

    v = readl(&iproc_rtc.regs->reg_SPRU_BBL_STATUS);
    while(!(v&RTC_REG_ACC_DONE))
    {
        udelay(1);
        v = readl(&iproc_rtc.regs->reg_SPRU_BBL_STATUS);
    }

    return 1;
}

static inline void rtc_reg_write(unsigned int reg_addr, unsigned int v)
{
    unsigned int cmd = 0;

    local_irq_disable();
    writel(v, &iproc_rtc.regs->reg_SPRU_BBL_WDATA);
    cmd = (reg_addr&0x3FF)|RTC_REG_WR_CMD|RTC_IND_SOFT_RST_N; /*Write command*/
    writel(cmd, &iproc_rtc.regs->reg_SPRU_BBL_CMD);
    wait_acc_done();
    local_irq_enable();
    
    return;
}

static inline unsigned int rtc_reg_read(unsigned int reg_addr)
{
    unsigned int cmd = 0, v;

    local_irq_disable();
    cmd = (reg_addr&0x3FF)|RTC_REG_RD_CMD|RTC_IND_SOFT_RST_N; /*Read command*/
    writel(cmd, &iproc_rtc.regs->reg_SPRU_BBL_CMD);
    wait_acc_done();
    v = readl(&iproc_rtc.regs->reg_SPRU_BBL_RDATA);
    local_irq_enable();
    
    return v;
}

static void bbl_init(void)
{
    unsigned int v;
    int read_cnt = 50;

    v = readl(&iproc_rtc.crmu_regs->reg_CRMU_SPRU_SOURCE_SEL_STAT);
    while(v != 0) /*SPRU_SOURCE_SELECT  AON*/
    {
        read_cnt--;
        if(read_cnt == 0)
        {
            printk("BBL: AON power is not available\n");
            return;
        }
        udelay(1);
        v = readl(&iproc_rtc.crmu_regs->reg_CRMU_SPRU_SOURCE_SEL_STAT);
    }
    
    /*Wait for reset cycle*/
    writel(0, &iproc_rtc.regs->reg_SPRU_BBL_CMD);
    udelay(200);
    writel(RTC_IND_SOFT_RST_N, &iproc_rtc.regs->reg_SPRU_BBL_CMD);

    /*- remove BBL related isolation from CRMU*/
    v = readl(&iproc_rtc.crmu_regs->reg_CRMU_ISO_CELL_CONTROL);
    v &= ~((1<<16)|(1<<24));
    writel(v, &iproc_rtc.crmu_regs->reg_CRMU_ISO_CELL_CONTROL);

    /* program CRMU auth_code resister*/
    writel(0x12345678, &iproc_rtc.auth_regs->reg_CRMU_BBL_AUTH_CODE);
    /* program CRMU auth_code_check register*/
    /* auth_code must equal to auth_code_check*/
    writel(0x12345678, &iproc_rtc.auth_regs->reg_CRMU_BBL_AUTH_CHECK);
}

static void iproc_rtc_enable(void)
{
    unsigned int v;

    rtc_reg_write(IREG_BBL_RTC_PER, BBL_PER_1s);    /*Set periodic timer as 1 second*/
    v = rtc_reg_read(IREG_BBL_INTERRUPT_EN);
    v &= ~(RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR);  /*Disable alarm&periodic interrupt*/
    rtc_reg_write(IREG_BBL_INTERRUPT_EN, v);
    rtc_reg_write(IREG_BBL_INTERRUPT_clr, RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR);
    v |= RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR;     /*Enable periodic interrupt*/
    rtc_reg_write(IREG_BBL_INTERRUPT_EN, v);        /*To enable RTC periodic interrupt*/
}

void iproc_rtc_disable(void)
{
    unsigned int v;

    v = rtc_reg_read(IREG_BBL_INTERRUPT_EN);
    v &= ~(RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR);
    rtc_reg_write(IREG_BBL_INTERRUPT_EN, v);        /*To disable RTC periodic interrupt*/
}

static irqreturn_t iproc_rtc_interrupt(int irq, void *class_dev)
{
	unsigned long events = 0;
	u32 irq_flg;
    unsigned long flags;
    unsigned int v;

    spin_lock_irqsave(&iproc_rtc.lock, flags);
	irq_flg = rtc_reg_read(IREG_BBL_INTERRUPT_stat);
    irq_flg &= RTC_REG_PERIO_INTR|RTC_REG_ALARM_INTR; /*periodic interrupt | alarm interrupt*/
    if(!irq_flg)
    {
        spin_unlock_irqrestore(&iproc_rtc.lock, flags);
        return IRQ_HANDLED;
    }
    
	if (irq_flg&RTC_REG_PERIO_INTR) {
        rtc_reg_write(IREG_BBL_INTERRUPT_clr, RTC_REG_PERIO_INTR); /*Clear periodic interrupt status*/
    	events |= RTC_IRQF | RTC_PF;
	}

    if (irq_flg&RTC_REG_ALARM_INTR) {
        rtc_reg_write(IREG_BBL_INTERRUPT_clr, RTC_REG_ALARM_INTR); /*Clear alarm interrupt status*/
    	events |= RTC_IRQF | RTC_AF;
        
        v = rtc_reg_read(IREG_BBL_INTERRUPT_EN);
        v &= ~RTC_REG_ALARM_INTR;
        rtc_reg_write(IREG_BBL_INTERRUPT_EN, v);  /*Disable Alarm interrupt*/
	}
    rtc_update_irq(iproc_rtc.rtc, 1, events);
    spin_unlock_irqrestore(&iproc_rtc.lock, flags);
	return IRQ_HANDLED;
}

static int iproc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags;
    unsigned int seconds;
    int ret = 0;

	spin_lock_irqsave(&iproc_rtc.lock, flags);

    seconds = rtc_reg_read(IREG_BBL_RTC_SECOND);
	rtc_time_to_tm(seconds, tm);

	ret = rtc_valid_tm(tm);

	spin_unlock_irqrestore(&iproc_rtc.lock, flags);

	return ret;
}

static int iproc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags, t;

    rtc_tm_to_time(tm, &t);
    spin_lock_irqsave(&iproc_rtc.lock, flags);
    rtc_reg_write(IREG_BBL_CONTROL, RTC_REG_RTC_STOP);  /*bbl_rtc_stop = 1, RTC halt*/
    rtc_reg_write(IREG_BBL_RTC_DIV, 0);                 /*Update DIV*/
    rtc_reg_write(IREG_BBL_RTC_SECOND, t);              /*Update second*/
    rtc_reg_write(IREG_BBL_CONTROL, ~RTC_REG_RTC_STOP); /*bbl_rtc_stop = 0, RTC release*/
	spin_unlock_irqrestore(&iproc_rtc.lock, flags);

	return 0;
}

static int iproc_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	unsigned long flags;
    unsigned int v;

	spin_lock_irqsave(&iproc_rtc.lock, flags);
    v = rtc_reg_read(IREG_BBL_INTERRUPT_EN);
	if (enabled)
		v |= RTC_REG_ALARM_INTR;
	else
		v &= ~RTC_REG_ALARM_INTR;

	rtc_reg_write(IREG_BBL_INTERRUPT_EN, v);
	spin_unlock_irqrestore(&iproc_rtc.lock, flags);

	return 0;
}

static int iproc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long flags;
    unsigned long seconds;
    unsigned int v;

	spin_lock_irqsave(&iproc_rtc.lock, flags);
    seconds = rtc_reg_read(IREG_BBL_RTC_MATCH);
    v = rtc_reg_read(IREG_BBL_RTC_SECOND);
    v &= ~0x3FFFFF;
    seconds = (seconds<<7);
    seconds |= v;
    rtc_time_to_tm(seconds, &alm->time);
    v = rtc_reg_read(IREG_BBL_INTERRUPT_EN);
    v &= RTC_REG_ALARM_INTR;
    alm->pending = !v;
    alm->enabled = alm->pending && device_may_wakeup(dev);
    spin_unlock_irqrestore(&iproc_rtc.lock, flags);

	return 0;
}

static int iproc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
    unsigned long flags;
    unsigned long seconds;
    
	spin_lock_irqsave(&iproc_rtc.lock, flags);
    rtc_tm_to_time(&alm->time, &seconds);
    seconds = ((seconds&(0xFFFF<<7))>>7)&0xFFFF;    
    rtc_reg_write(IREG_BBL_RTC_MATCH, seconds);
    spin_unlock_irqrestore(&iproc_rtc.lock, flags);

	return 0;
}

static struct rtc_class_ops iproc_rtc_ops = {
	.read_time		    = iproc_rtc_read_time,
	.set_time		    = iproc_rtc_set_time,
	.alarm_irq_enable	= iproc_rtc_alarm_irq_enable,
	.read_alarm		    = iproc_rtc_read_alarm,
	.set_alarm		    = iproc_rtc_set_alarm,
};

static int __init iproc_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret = 0;

    memset(&iproc_rtc, 0, sizeof(iproc_rtc_t));
    spin_lock_init(&iproc_rtc.lock);
	iproc_rtc.peridic_irq = platform_get_irq(pdev, 0);
	if (iproc_rtc.peridic_irq < 0) {
		dev_err(dev, "no RTC periodic irq\n");
		ret = -1;
		goto fail;
	}

    iproc_rtc.alarm_irq = platform_get_irq(pdev, 1);
    if (iproc_rtc.alarm_irq < 0) {
		dev_err(dev, "no RTC alarm irq\n");
		ret = -1;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no mem resource\n");
		ret = -EINVAL;
		goto fail;
	}
	iproc_rtc.regs = (volatile rtc_reg_t *)ioremap(res->start, resource_size(res));
	if (!iproc_rtc.regs) {
		dev_err(dev, "unable to ioremap MEM resource\n");
		ret = -ENOMEM;
		goto fail;
	}
    printk("rtc_reg = 0x%08x\n", (unsigned int)iproc_rtc.regs);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "no mem resource\n");
		ret = -EINVAL;
		goto fail0;
	}
	iproc_rtc.crmu_regs = (volatile crmu_reg_t *)ioremap(res->start, resource_size(res));
	if (!iproc_rtc.crmu_regs) {
		dev_err(dev, "unable to ioremap MEM resource\n");
		ret = -ENOMEM;
		goto fail0;
	}
    printk("crmu_regs = 0x%08x\n", (unsigned int)iproc_rtc.crmu_regs);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(dev, "no mem resource\n");
		ret = -EINVAL;
		goto fail1;
	}
	iproc_rtc.auth_regs = (volatile bbl_auth_t *)ioremap(res->start, resource_size(res));
	if (!iproc_rtc.auth_regs) {
		dev_err(dev, "unable to ioremap MEM resource\n");
		ret = -ENOMEM;
		goto fail1;
	}
    printk("auth_regs = 0x%08x\n", (unsigned int)iproc_rtc.auth_regs);

	platform_set_drvdata(pdev, &iproc_rtc);
    bbl_init();
    //iproc_rtc_disable();
	iproc_rtc.rtc = rtc_device_register(pdev->name, &pdev->dev,
				    &iproc_rtc_ops, THIS_MODULE);
	if (IS_ERR(iproc_rtc.rtc)) {
		dev_err(dev, "unable to register RTC device, err %ld\n",
				PTR_ERR(iproc_rtc.rtc));
		goto fail2;
	}

	ret = request_irq(iproc_rtc.peridic_irq, iproc_rtc_interrupt, IRQF_DISABLED | IRQF_SHARED, "iproc_rtc_peri", pdev);
	if (ret < 0) {
		dev_err(dev, "unable to register iproc RTC periodic interrupt\n");
		goto fail3;
	}

    ret = request_irq(iproc_rtc.alarm_irq, iproc_rtc_interrupt, 0, "iproc_rtc_alarm", pdev);
    if (ret < 0) {
		dev_err(dev, "unable to register iproc RTC alarm interrupt\n");
		goto fail4;
	}
    
    iproc_rtc_enable();
    device_init_wakeup(&pdev->dev, 1);
	return 0;
fail4:
    free_irq(iproc_rtc.peridic_irq, pdev);
fail3:
	rtc_device_unregister(iproc_rtc.rtc);
fail2:
    iounmap(iproc_rtc.auth_regs);
fail1:
    iounmap(iproc_rtc.crmu_regs);
fail0:
    iounmap(iproc_rtc.regs);
fail:
    platform_set_drvdata(pdev, NULL);
	return ret;
}

static int __devexit iproc_rtc_remove(struct platform_device *pdev)
{
    device_init_wakeup(&pdev->dev, 0);
    free_irq(iproc_rtc.alarm_irq, pdev);
	free_irq(iproc_rtc.peridic_irq, pdev);
	rtc_device_unregister(iproc_rtc.rtc);
    iounmap(iproc_rtc.auth_regs);
    iounmap(iproc_rtc.crmu_regs);
	iounmap(iproc_rtc.regs);
	platform_set_drvdata(pdev, NULL);
    
	return 0;
}

static struct resource iproc_rtc_resources[] = {
        [0] = {
            .start  = IPROC_BBL_REG_BASE,
            .end    = IPROC_BBL_REG_BASE+0xF,
            .flags  = IORESOURCE_MEM,
        },
        [1] = {
            .start  = IPROC_BBL_POWER_STS,
            .end    = IPROC_BBL_POWER_STS+0x17,
            .flags  = IORESOURCE_MEM,
        },
        [2] = {
            .start  = IPROC_BBL_AUTH_REG,
            .end    = IPROC_BBL_AUTH_REG+0x3,
            .flags  = IORESOURCE_MEM,
        },
        [3] = {
            .start  = IPROC_RTC_INTR_PERIODIC,
            .end    = IPROC_RTC_INTR_PERIODIC,
            .flags  = IORESOURCE_IRQ,
        },
        [4] = {
            .start  = IPROC_RTC_INTR_ALARM,
            .end    = IPROC_RTC_INTR_ALARM,
            .flags  = IORESOURCE_IRQ,
        }
};

void platform_rtc_release(struct device *pDev)
{
    return;
}

static struct platform_device iproc_rtc_device = {
        .name           =       "iproc-rtc",
        .id             =       -1,
        .dev            =       {
                .platform_data  = NULL,
                .release = platform_rtc_release
        },
        .num_resources  = ARRAY_SIZE(iproc_rtc_resources),
        .resource       = iproc_rtc_resources,
};

static struct platform_driver iproc_rtc_driver = {
	.probe		= iproc_rtc_probe,
	.remove		= __devexit_p(iproc_rtc_remove),
	.driver		= {
		.name = "iproc-rtc",
		.owner = THIS_MODULE,
	},
};

static int __init iproc_rtc_init(void)
{
    if(platform_device_register(&iproc_rtc_device))
    {
        printk("Register RTC device failed\n");
        return -1;
    }
	return platform_driver_register(&iproc_rtc_driver);
}
module_init(iproc_rtc_init);

static void __exit iproc_rtc_exit(void)
{
    platform_driver_unregister(&iproc_rtc_driver);
    platform_device_unregister(&iproc_rtc_device);
}
module_exit(iproc_rtc_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom IPROC PRTC Driver");
MODULE_LICENSE("GPL");

