#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/keyboard.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "mach/socregs-cygnus.h"
#include "mach/iproc_regs.h"
#include "keypad.h"

static int keypad_dbg = 0;
module_param_named(keypad_debug, keypad_dbg, int, 0644);
#define KP_DEBUG if(keypad_dbg) printk

#define KEY_STAR KEY_NUMERIC_STAR
#define KEY_POUND KEY_NUMERIC_POUND
unsigned int iproc_keymap[8][8] = {
/**********************************************************************************************************
                                              keypad
                                          ---------------
                                          | D  #  0  *  |
                                          | C  9  8  7  |
                                          | B  6  5  4  |
                                          | A  3  2  1  |
                                          ---------------
************************************************************************************************************/
/*Col    0             1             2             3             4             5             6             7            */
/*Row0*/{KEY_D,        KEY_POUND,    KEY_NUMERIC_0,KEY_STAR,     KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
/*Row1*/{KEY_C,        KEY_NUMERIC_9,KEY_NUMERIC_8,KEY_NUMERIC_7,KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
/*Row2*/{KEY_B,        KEY_NUMERIC_6,KEY_NUMERIC_5,KEY_NUMERIC_4,KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
/*Row3*/{KEY_A,        KEY_NUMERIC_3,KEY_NUMERIC_2,KEY_NUMERIC_1,KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
/*Row4*/{KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
/*Row5*/{KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
/*Row6*/{KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
/*Row7*/{KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED, KEY_RESERVED},
};

typedef struct _keypad_reg
{
    unsigned int kpcr;
    unsigned int kpior;
    unsigned int rsvd0[2];
    unsigned int kpemr0;
    unsigned int kpemr1;
    unsigned int kpemr2;
    unsigned int kpemr3;
    unsigned int kpssr0;
    unsigned int kpssr1;
    unsigned int rsvd1[2];
    unsigned int kpimr0;
    unsigned int kpimr1;
    unsigned int kpicr0;
    unsigned int kpicr1;
    unsigned int kpisr0;
    unsigned int kpisr1;
    unsigned int kpdbctr;
} iproc_keypad_reg_t;

typedef struct _iomux_ctrl_reg
{
    unsigned int io_mux1;
    unsigned int io_mux2;
} iproc_iomux_reg_t;

typedef struct _bcm_kp
{
    int irqno;
    iproc_keypad_reg_t *keypad_reg;
    unsigned int *clk_reg;
    iproc_iomux_reg_t *io_mux;
    iproc_keypad_t *pKeypad;
    unsigned int *clk_gate;
    spinlock_t kp_lock;
} iproc_bcm_kp_t;

iproc_bcm_kp_t bcm_kp;
static irqreturn_t iproc_keypad_interrupt(int irq, void *data)
{
    struct platform_device *pdev = (struct platform_device *)data;
	struct input_dev *in_dev = (struct input_dev *)platform_get_drvdata(pdev);
	unsigned int intr_status0, intr_status1,ssr0, ssr1;
    int i;
    unsigned long flags;
    int col, row;

    spin_lock_irqsave(&bcm_kp.kp_lock, flags);
    intr_status0 = bcm_kp.keypad_reg->kpisr0;
    intr_status1 = bcm_kp.keypad_reg->kpisr1;
    KP_DEBUG("keypad: intr_status %08x %08x\n", intr_status0, intr_status1);
    if(!intr_status0 && !intr_status1) /*No interrupt detected*/
    {
        spin_unlock_irqrestore(&bcm_kp.kp_lock, flags);
        return IRQ_HANDLED;
    }
    
    if(intr_status0)
    {
        bcm_kp.keypad_reg->kpimr0 &= ~intr_status0; /*Disable related interrupt*/
        bcm_kp.keypad_reg->kpicr0 |= intr_status0; /*Clear related interrupt*/
    }
    if(intr_status1)
    {
        bcm_kp.keypad_reg->kpimr1 &= ~intr_status1; /*Disable related interrupt*/
        bcm_kp.keypad_reg->kpicr1 |= intr_status1; /*Clear related interrupt*/
    }

    if(intr_status0)
    {
        ssr0 = bcm_kp.keypad_reg->kpssr0;
        for(i = 0; i < 32; i++)
        {
            if((intr_status0&(1<<i)) == 0)
                continue;

            row = i/8;
            col = i%8;
            if(iproc_keymap[row][col] == KEY_RESERVED)
                continue;
            if((ssr0&(1<<i)) == 0)
            {
                input_report_key(in_dev, iproc_keymap[row][col], 1);
                KP_DEBUG("keypad: key %d pressed\n", iproc_keymap[row][col]);
            }
            else
            {
                input_report_key(in_dev, iproc_keymap[row][col], 0);
                KP_DEBUG("keypad: key %d released\n", iproc_keymap[row][col]);
            }
            input_sync(in_dev);
            
        }
    }

    if(intr_status1)
    {
        ssr1 = bcm_kp.keypad_reg->kpssr1;
        for(i = 0; i < 32; i++)
        {
            if((intr_status0&(1<<i)) == 0)
                continue;

            col = (i+32)/8;
            row = (i+32)%8;
            if(iproc_keymap[row][col] == KEY_RESERVED)
                continue;
            if((ssr1&(1<<i)) == 0)
            {
                input_report_key(in_dev, iproc_keymap[row][col], 1);
                KP_DEBUG("keypad: key %d pressed\n", iproc_keymap[row][col]);
            }
            else
            {
                input_report_key(in_dev, iproc_keymap[row][col], 0);
                KP_DEBUG("keypad: key %d released\n", iproc_keymap[row][col]);
            }
            input_sync(in_dev);
        }
    }

    if(intr_status0)
    {
        bcm_kp.keypad_reg->kpimr0 |= intr_status0; /*Enable related interrupt*/
    }
    if(intr_status1)
    {
        bcm_kp.keypad_reg->kpimr1 |= intr_status1; /*Enable related interrupt*/
    }
    spin_unlock_irqrestore(&bcm_kp.kp_lock, flags);
    
	return IRQ_HANDLED;
}

static unsigned int bits_pattern_set
    (
    unsigned int base_value,
    int          offset,
    unsigned int bits_pattern,
    unsigned int bits_pattern_len,
    int          repeat
    )
{
    int total_offset = offset, i;
    unsigned int bits_mask = 0;

    for(i = 0; i < bits_pattern_len; i++)
        bits_mask |= 1<<i;

    bits_pattern &= bits_mask;
    for(i = 0; i < repeat; i++)
    {
        base_value &= ~(bits_mask<<total_offset);
        base_value |= bits_pattern<<total_offset;
        total_offset += bits_pattern_len;
    }

    return base_value;
}

static void iproc_keypad_enable(void)
{
    int i;
    iproc_keypad_t *pKeypad = (iproc_keypad_t *)bcm_kp.pKeypad;
    unsigned int kp_ctrl_reg = 0;
    unsigned int value = 0, pattern_offset = 0;
    volatile unsigned int *pv;

    value = bcm_kp.io_mux->io_mux1;
    /*IO MUX: 001 : keypad colume GPIO 8~15*/
    value = bits_pattern_set(value, 0, 0x1, 4, pKeypad->max_cols); /*Each col takes 4 bits*/
    bcm_kp.io_mux->io_mux1 = value;

    value = bcm_kp.io_mux->io_mux2;
    /*IO MUX: 001 : keypad row GPIO 16~23*/
    value = bits_pattern_set(value, 0, 0x1, 4, pKeypad->max_rows); /*Each row takes 4 bits*/
    bcm_kp.io_mux->io_mux2 = value;

     /**************************************
        Controls CLK_GATE_EN signal of Keypad
        Controller block
        1 = disableClockGating to this block
        0 = enableClockGating to this block(Default value)
        **************************************/
    *bcm_kp.clk_reg |= 0x80050005;
    *bcm_kp.clk_gate |= 1<<ASIU_TOP_CLK_GATING_CTRL__KEYPAD_CLK_GATE_EN;
    kp_ctrl_reg |= (pKeypad->max_rows-1)<<KEYPAD_TOP_REGS_KPCR__RowWidth_R;
    kp_ctrl_reg |= (pKeypad->max_cols-1)<<KEYPAD_TOP_REGS_KPCR__ColumnWidth_R;
    kp_ctrl_reg |= pKeypad->StatFilEn<<KEYPAD_TOP_REGS_KPCR__StatusFilterEnable;
    kp_ctrl_reg |= pKeypad->StatFilType<<KEYPAD_TOP_REGS_KPCR__StatusFilterType_R;
    kp_ctrl_reg |= pKeypad->ColFilEn<<KEYPAD_TOP_REGS_KPCR__ColFilterEnable;
    kp_ctrl_reg |= pKeypad->ColFilType<<KEYPAD_TOP_REGS_KPCR__ColumnFilterType_R;
    kp_ctrl_reg |= pKeypad->IoMode<<KEYPAD_TOP_REGS_KPCR__soft;
    kp_ctrl_reg |= pKeypad->SwapRc<<KEYPAD_TOP_REGS_KPCR__SwapRowColumn;
    kp_ctrl_reg |= pKeypad->ScanMode<<KEYPAD_TOP_REGS_KPCR__Mode;
    kp_ctrl_reg |= 1<<KEYPAD_TOP_REGS_KPCR__Enable;
    bcm_kp.keypad_reg->kpcr = kp_ctrl_reg;

    pattern_offset = KEYPAD_TOP_REGS_KPIOR__RowOContrl_R;
    value = bcm_kp.keypad_reg->kpior;
    value = bits_pattern_set(value, pattern_offset, 1, 1, pKeypad->max_rows); /*Rows: Output*/
    pattern_offset = KEYPAD_TOP_REGS_KPIOR__ColumnOContrl_R;
    value = bits_pattern_set(value, pattern_offset, 0, 1, pKeypad->max_cols); /*Cols: Input*/
    value = bits_pattern_set(value, 8, 1, 1, pKeypad->max_rows);
    value = bits_pattern_set(value, 0, 0, 1, pKeypad->max_cols);
    bcm_kp.keypad_reg->kpior = value;

    for(i = 0; i < pKeypad->max_rows; i++)
    {
        pattern_offset = 0;
        if(i%2) pattern_offset = 16;
        switch(i)
        {
        case 0:
        case 1:
            value = bcm_kp.keypad_reg->kpemr0;
            pv = (volatile unsigned int *)&bcm_kp.keypad_reg->kpemr0;
            break;
        case 2:
        case 3:
            value = bcm_kp.keypad_reg->kpemr1;
            pv = (volatile unsigned int *)&bcm_kp.keypad_reg->kpemr1;
            break;
        case 4:
        case 5:
            value = bcm_kp.keypad_reg->kpemr2;
            pv = (volatile unsigned int *)&bcm_kp.keypad_reg->kpemr2;
            break;
        case 6:
        case 7:
            value = bcm_kp.keypad_reg->kpemr3;
            pv = (volatile unsigned int *)&bcm_kp.keypad_reg->kpemr3;
            break;
        default:
            return;
        }
        value = bits_pattern_set(value, pattern_offset, 0x3, 2, pKeypad->max_cols); /*Falling Edge*/
        *pv = value;
    }

    bcm_kp.keypad_reg->kpimr0 = 0; /*Disable all key's interrupt*/
    bcm_kp.keypad_reg->kpimr1 = 0; /*Disable all key's interrupt*/
    for(i = 0; i < pKeypad->max_rows; i++)
    {
        pattern_offset = (i%4)*8;
        switch(i)
        {
        case 0:
        case 1:
        case 2:
        case 3:
            value = bcm_kp.keypad_reg->kpimr0;
            pv = (volatile unsigned int *)&bcm_kp.keypad_reg->kpimr0;
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            value = bcm_kp.keypad_reg->kpimr1;
            pv = (volatile unsigned int *)&bcm_kp.keypad_reg->kpimr1;
            break;
        default:
            return;
        }
        value = bits_pattern_set(value, pattern_offset, 0x1, 1, pKeypad->max_cols); /*Enable interrupt according to col and row*/
        *pv = value;
    }

    bcm_kp.keypad_reg->kpicr0 = 0xFFFFFFFF; /*Clear all interrupts*/
    bcm_kp.keypad_reg->kpicr1 = 0xFFFFFFFF; /*Clear all interrupts*/
}

static int __init iproc_keypad_probe(struct platform_device *pdev)
{
    struct input_dev *in_dev;
    iproc_keypad_t *pKeypad = (iproc_keypad_t *)pdev->dev.platform_data;
    struct resource *res = NULL;
    int err = 0, ret = -1;
    int i,j;

    spin_lock_init(&bcm_kp.kp_lock);
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		err = -ENXIO;
		goto fail0;
	}
    bcm_kp.pKeypad = pKeypad;
	bcm_kp.keypad_reg = (iproc_keypad_reg_t *)ioremap_nocache(res->start, resource_size(res));
	if (!bcm_kp.keypad_reg) {
		err = -ENOMEM;
		goto fail0;
	}
    KP_DEBUG("keypad_reg = 0x%08x\n", (unsigned int)bcm_kp.keypad_reg);

    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		err = -ENXIO;
		goto fail1;
	}
	bcm_kp.clk_reg = (unsigned int *)ioremap_nocache(res->start, resource_size(res));
	if (!bcm_kp.clk_reg) {
        KP_DEBUG("keypad: IO remap failed\n");
		err = -ENOMEM;
		goto fail1;
	}
    KP_DEBUG("clk_reg = 0x%08x\n", (unsigned int)bcm_kp.clk_reg);

    bcm_kp.io_mux = (iproc_iomux_reg_t *)ioremap_nocache(IPROC_IO_MUX_REG_BASE, 0x8);
	if (!bcm_kp.io_mux) {
		err = -ENOMEM;
		goto fail2;
	}
    KP_DEBUG("io_mux = 0x%08x\n", (unsigned int)bcm_kp.io_mux);

    bcm_kp.clk_gate = (unsigned int *)ioremap_nocache(IPROC_CLK_GATING_CTRL, 0x4);
	if (!bcm_kp.clk_gate) {
		err = -ENOMEM;
		goto fail3;
	}
    KP_DEBUG("clk_gate = 0x%08x\n", (unsigned int)bcm_kp.clk_gate);

    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res) {
		err = -ENXIO;
		goto fail4;
	}

    iproc_keypad_enable();
    
	in_dev = input_allocate_device();
	if (!in_dev) {
		dev_err(&pdev->dev, "Not enough memory for input device\n");
		err =  -ENOMEM;
        goto fail4;
	}

	in_dev->name = pdev->name;
	in_dev->phys = "bcm/input0";
	in_dev->id.bustype = BUS_HOST;
	in_dev->id.vendor = 0x0001;
	in_dev->id.product = 0x0001;
	in_dev->id.version = 0x0100;
	in_dev->dev.parent = &pdev->dev;
	in_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);

    for(i = 0; i < pKeypad->max_rows; i++)
        for(j = 0; j < pKeypad->max_cols; j++)
    {
        if(iproc_keymap[i][j] == KEY_RESERVED)
            continue;
        set_bit(iproc_keymap[i][j], in_dev->keybit);
    }

	err = input_register_device(in_dev);
	if (err)
    {   
        KP_DEBUG("keypad: input device register failed\n");
		goto fail5;
    }

    bcm_kp.irqno = res->start;
    ret = request_irq(bcm_kp.irqno, iproc_keypad_interrupt,
			IRQF_DISABLED | IRQF_SHARED, "iproc-keypad", pdev);
	if (ret)
    {   
        KP_DEBUG("keypad: interrupt register failed\n");
		goto fail5;
    }
    
	platform_set_drvdata(pdev, in_dev);

	return 0;

 fail5:	
    input_unregister_device(in_dev);
 fail4:
    iounmap(bcm_kp.clk_gate);
 fail3:
    iounmap(bcm_kp.io_mux);
 fail2:	
    iounmap(bcm_kp.clk_reg);
 fail1:
    iounmap(bcm_kp.keypad_reg);
 fail0:
    
	return err;
}

static int __exit iproc_keypad_remove(struct platform_device *pdev)
{
	struct input_dev *in_dev = platform_get_drvdata(pdev);

    iounmap(bcm_kp.io_mux);
    iounmap(bcm_kp.clk_reg);
    iounmap(bcm_kp.keypad_reg);
	platform_set_drvdata(pdev, NULL);
	free_irq(bcm_kp.irqno, pdev);
	input_unregister_device(in_dev);
	return 0;
}

#ifdef CONFIG_PM
static void iproc_keypad_shutdown(struct platform_device *pdev)
{
    //bcm_kp.clk_gate |= (1<<ASIU_TOP_CLK_GATING_CTRL__KEYPAD_CLK_GATE_EN);
}

static int iproc_keypad_resume(struct platform_device *pdev)
{
    //bcm_kp.clk_gate &= ~(1<<ASIU_TOP_CLK_GATING_CTRL__KEYPAD_CLK_GATE_EN);
    iproc_keypad_enable();
    return 0;
}

#endif

static iproc_keypad_t keypad_info = {
    .max_rows       = 4,
    .max_cols       = 4,
    .StatFilEn      = 1,
    .StatFilType    = 0x07,
    .ColFilEn       = 1,
    .ColFilType     = 0x07,
    .IoMode         = 0,
    .SwapRc         = 1,
    .ScanMode       = 1
};

static struct resource keypad_resources[] = {
        [0] = {
                .start  = IPROC_KEYPAD_REG_BASE,
                .end    = IPROC_KEYPAD_REG_BASE+0x4B,
                .flags  = IORESOURCE_MEM,
        },
        [1] = {
                .start  = IPROC_CRMU_ASIU_KEYPAD_CLK_DIV,
                .end    = IPROC_CRMU_ASIU_KEYPAD_CLK_DIV+0x03,
                .flags  = IORESOURCE_MEM,
        },
        [2] = {
                .start  = IPROC_KEYPAD_INTR,
                .end    = IPROC_KEYPAD_INTR,
                .flags  = IORESOURCE_IRQ,
        }
};

void platform_keypad_release(struct device *dev)
{
    return;
}

static struct platform_device keypad_device = {
        .name           =       "iproc-keypad",
        .id             =       -1,
        .dev            =       {
                .platform_data  = &keypad_info,
                .release = platform_keypad_release
        },
        .num_resources  = ARRAY_SIZE(keypad_resources),
        .resource       = keypad_resources,
};

static struct platform_driver iproc_keypad_driver = {
	.remove = __exit_p(iproc_keypad_remove),
	.driver   = {
		.name	= "iproc-keypad",
		.owner	= THIS_MODULE,
	},
#ifdef CONFIG_PM
	.shutdown = iproc_keypad_shutdown,
	.resume   = iproc_keypad_resume,
#endif
};

static int __init iproc_keypad_init(void)
{
    platform_device_register(&keypad_device);
	return platform_driver_probe(&iproc_keypad_driver, iproc_keypad_probe);
}

static void __exit iproc_keypad_exit(void)
{
    platform_driver_unregister(&iproc_keypad_driver);
    platform_device_unregister(&keypad_device);
}

module_init(iproc_keypad_init);
module_exit(iproc_keypad_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("IPROC keyboard driver");
MODULE_LICENSE("GPL");

