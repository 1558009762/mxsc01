/*
 * $Copyright Open Broadcom Corporation$
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <mach/iproc_regs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>
#include <linux/usb/phy.h>
#include <linux/usb/iproc_usb.h>
#include <linux/of_gpio.h>

/***************************************************************************
****************************************************************************
***************************************************************************/
#define USB2D_IDM_IDM_IO_CONTROL_DIRECT_ADDR(base)      (base + 0x408)
#define USB2D_IDM_IDM_RESET_CONTROL_ADDR(base)          (base + 0x800)

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
#define IPROC_CCB_MDIO_MII_CTRL_ADDR(base)              (base + 0x0)
#define IPROC_CCB_MDIO_MII_DATA_ADDR(base)              (base + 0x4)
#define IPROC_CCB_MDIO_COMPATIBLE                       "brcm,iproc-ccb-mdio"
#if defined(CONFIG_MACH_HX4)
#define IPROC_WRAP_IPROC_XGPLL_CTRL_0_ADDR(base)        (base + 0x1c)
#define IPROC_WRAP_IPROC_XGPLL_CTRL_4_ADDR(base)        (base + 0x2c)
#define IPROC_WRAP_USBPHY_CTRL_ADDR(base)               (base + 0x34)
#define IPROC_WRAP_MISC_STATUS_ADDR(base)               (base + 0x38)
#define IPROC_CLK_NDIV_40                               0x80
#define IPROC_CLK_NDIV_20                               0x8C
#define USB_CLK_NDIV_MASK                               0xFE7FFE00
#define USB_CLK_PLL_RESET_MASK                          0xFF7FFE00
#define USB_CLK_PHY_RESET_MASK                          0xFFFFFE00
#define USB_CLK_NDIV_40                                 0x30
#define USB_CLK_NDIV_20                                 0x60
#elif defined(CONFIG_MACH_KT2)
#define IPROC_DDR_PLL_CTRL_REGISTER_3_ADDR(base)        (base + 0x0c)
#define IPROC_DDR_PLL_CTRL_REGISTER_5_ADDR(base)        (base + 0x14)
#define IPROC_WRAP_USBPHY_CTRL_ADDR(base)               (base + 0x20)
#define IPROC_WRAP_MISC_STATUS_ADDR(base)               (base + 0x28)
#endif /* !defined(CONFIG_MACH_HX4) */
#endif /* (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2)) */

#if (defined(CONFIG_MACH_SB2))
#define IPROC_WRAP_USBPHY_CTRL_0_ADDR(base)             (base + 0x28)
#define IPROC_WRAP_USBPHY_CTRL_2_ADDR(base)             (base + 0x30)
#define IPROC_WRAP_MISC_STATUS_ADDR(base)               (base + 0x44)
#define IPROC_WRAP_TOP_STRAP_CTRL_ADDR(base)            (base + 0x70)
#endif /* (defined(CONFIG_MACH_SB2)) */

#if (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) || defined(CONFIG_MACH_GH2))
#define IPROC_WRAP_USBPHY_CTRL_0_ADDR(base)             (base + 0x44)
#define IPROC_WRAP_USBPHY_CTRL_2_ADDR(base)             (base + 0x4c)
#define IPROC_WRAP_MISC_STATUS_ADDR(base)               (base + 0x58)
#define IPROC_WRAP_TOP_STRAP_STATUS_ADDR(base)          (base + 0xa4)
#endif /* (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) || defined(CONFIG_MACH_GH2)) */

struct iproc_usb_priv {
	struct usb_phy phy;
    struct device *dev;
    struct device_node *dn;
    void __iomem *wrap_base;
    void __iomem *idm_base;
    uint usb_mode;
};

extern void __iomem *get_iproc_wrap_ctrl_base(void);

/***************************************************************************
****************************************************************************
***************************************************************************/
static const struct of_device_id xgs_iproc_usb_phy_dt_ids[] = {
    { .compatible = "brcm,usb-phy,hx4", },
    { .compatible = "brcm,usb-phy,kt2", },
    { .compatible = "brcm,usb-phy,gh", },
    { .compatible = "brcm,usb-phy,sb2", },
    { .compatible = "brcm,usb-phy,hr3", },
    { }
};
MODULE_DEVICE_TABLE(of, xgs_iproc_usb_phy_dt_ids);


static int xgs_iproc_usb_phy_mode(struct iproc_usb_priv *iproc_usb_data)
{
    void __iomem *wrap_base = iproc_usb_data->wrap_base;
    struct device *dev = iproc_usb_data->dev;
    int usb_mode = IPROC_USB_MODE_HOST;
    ulong val;
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
    int gpio_pin, ret;
#endif /* (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2)) */

    if (!wrap_base) {
        return -EINVAL;
    }

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
    /* gpio pin 4 to control host/device mode */
    gpio_pin = of_get_named_gpio(dev->of_node, "usbdev-gpio", 0);

    if (gpio_pin < 0) {
        dev_warn(dev, "No gpio pin set for USB device detection(default to 4)\n");
        gpio_pin = 4;
    }

    ret = gpio_request(gpio_pin, "usbdev-gpio");
    if (ret != 0) {
        dev_err(dev, "request gpio #%d error.\n", gpio_pin);
        return ret;
    }

    val = __gpio_get_value(gpio_pin);

    gpio_free(gpio_pin);

    if (val & 1) {
        usb_mode = IPROC_USB_MODE_DEVICE;
    }
#elif defined(CONFIG_MACH_SB2)
    /* u-boot enable this bit to indicate usb in host mode */
    val = readl_relaxed(IPROC_WRAP_TOP_STRAP_CTRL_ADDR(wrap_base));
    if (!(val & (1 << IPROC_WRAP_TOP_STRAP_CTRL__USB_DEVICE))) {
        usb_mode = IPROC_USB_MODE_DEVICE;
    }
#else
    /* u-boot enable this bit to indicate usb in host mode */
    val = readl_relaxed(IPROC_WRAP_TOP_STRAP_STATUS_ADDR(wrap_base));
    if (!(val & (1 << IPROC_WRAP_TOP_STRAP_STATUS__USB2_SEL))) {
        usb_mode = IPROC_USB_MODE_DEVICE;
    }
#endif

    dev_info(dev, "usb mode: %s\n", (usb_mode == IPROC_USB_MODE_DEVICE) ? "DEVICE" : "HOST");

    return usb_mode;
}

#if (defined (CONFIG_MACH_HX4) || defined (CONFIG_MACH_KT2))
/* Returns USB PHY PLL ref clock in MHz */
static uint _get_usb_clk(void __iomem *wrap_base)
{
    uint ndiv, mdiv, refclk;
    ulong val;

#if defined(CONFIG_MACH_HX4)
    val = readl_relaxed(IPROC_WRAP_IPROC_XGPLL_CTRL_4_ADDR(wrap_base));
    ndiv = ((val >> IPROC_WRAP_IPROC_XGPLL_CTRL_4__NDIV_INT_R) &
            ~(0xFFFFFFFF << IPROC_WRAP_IPROC_XGPLL_CTRL_4__NDIV_INT_WIDTH));

    val = readl_relaxed(IPROC_WRAP_IPROC_XGPLL_CTRL_0_ADDR(wrap_base));
    mdiv = ((val >> IPROC_WRAP_IPROC_XGPLL_CTRL_0__CH3_MDIV_R) &
            ~(0xFFFFFFFF << IPROC_WRAP_IPROC_XGPLL_CTRL_0__CH3_MDIV_WIDTH));
#else
    val = readl_relaxed(IPROC_DDR_PLL_CTRL_REGISTER_3_ADDR(wrap_base));
    ndiv = ((val >> IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_R) &
            ~(0xFFFFFFFF << IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_WIDTH));

    /* read channel 1 mdiv */
    val = readl_relaxed(IPROC_DDR_PLL_CTRL_REGISTER_5_ADDR(wrap_base));
    mdiv = ((val >> IPROC_DDR_PLL_CTRL_REGISTER_5__CH1_MDIV_R) &
            ~(0xFFFFFFFF << IPROC_DDR_PLL_CTRL_REGISTER_5__CH1_MDIV_WIDTH));
#endif

    refclk = (25 * ndiv) / mdiv;

    return refclk;
}
#endif /* (defined (CONFIG_MACH_HX4) || defined (CONFIG_MACH_KT2)) */

static int iproc_usb_phy_hx4_config(struct iproc_usb_priv *iproc_usb_data)
{
#if (defined (CONFIG_MACH_HX4) || defined (CONFIG_MACH_KT2))
    void __iomem *wrap_base = iproc_usb_data->wrap_base;
    void __iomem *ccb_mdio_base = NULL;
    struct device_node *np;
    ulong ndiv, precmd, miicmd, miidata;
    ulong val, mask;
    uint count = 0;

    if (!wrap_base) {
        return -EINVAL;
    }

    if (iproc_usb_data->usb_mode == IPROC_USB_MODE_DEVICE) {
        np = of_find_compatible_node(NULL, NULL, IPROC_CCB_MDIO_COMPATIBLE);
        if (!np) {
            printk(KERN_ERR "Failed to find CCB MDIO defined in DT\n");
            return -ENODEV;
        }

        ccb_mdio_base = of_iomap(np, 0);
        if (!ccb_mdio_base) {
            printk(KERN_ERR "Unable to iomap CCB MDIO base address\n");
            return -ENXIO;
        }

        ndiv = 1920 / _get_usb_clk(wrap_base);

        /* Construct precmd with Start Bit, PHY address and turnaround time */
        /* SB | PA | TA */
        precmd = 1 << 30 | 6 << 23 | 2 << 16;

        /* Connect MDIO interface to onchip PHY */
        writel_relaxed(0x9A, IPROC_CCB_MDIO_MII_CTRL_ADDR(ccb_mdio_base));
        mdelay(10);

        /* Program NDIV and PDIV into 0x1C register */
        miicmd = precmd | (0x1 << 28) | (0x1C << 18);
        miidata = 1 << 12 | ndiv;
        /* 0x53721040 */
        writel_relaxed(miicmd | miidata, IPROC_CCB_MDIO_MII_DATA_ADDR(ccb_mdio_base));
        mdelay(10);

        /* Program other PLL parameters into 0x1D register, disable suspend and put PHY into reset */
        miicmd = precmd | (0x1 << 28) | (0x1D << 18);
        miidata = 1 << 13 | 3 << 8 | 3 << 4 | 0xa;
        /* 0x5376233a  */
        writel_relaxed(miicmd | miidata, IPROC_CCB_MDIO_MII_DATA_ADDR(ccb_mdio_base));
        mdelay(10);

        /* Program register 0x15, USB device mode set and get PHY out of reset */
        miicmd = precmd | (0x1 << 28) | (0x15 << 18);
        miidata = 1 << 2 | 1 << 1;
        /* 0x53560006 */
        writel_relaxed(miicmd | miidata, IPROC_CCB_MDIO_MII_DATA_ADDR(ccb_mdio_base));
        mdelay(10);

        /* Program register 0x19, set mdio mode */
        miicmd = precmd | (0x1 << 28) | (0x19 << 18);
        miidata = 1 << 7;
        /* 0x53660080 */
        writel_relaxed(miicmd | miidata, IPROC_CCB_MDIO_MII_DATA_ADDR(ccb_mdio_base));
        mdelay(10);

        /* get the PLL out of reset */
        miicmd = precmd | (0x2 << 28) | (0x1D << 18);
        miidata = 0;
        writel_relaxed(miicmd | miidata, IPROC_CCB_MDIO_MII_DATA_ADDR(ccb_mdio_base));
        mdelay(10);
        miidata = readl_relaxed(IPROC_CCB_MDIO_MII_DATA_ADDR(ccb_mdio_base));
        miicmd = precmd | (0x1 << 28) | (0x1D << 18);
        miidata |= (1 << 12);
        /* 0x5376333a  */
        writel_relaxed(miicmd | miidata, IPROC_CCB_MDIO_MII_DATA_ADDR(ccb_mdio_base));
        mdelay(10);

        if (ccb_mdio_base) {
            iounmap(ccb_mdio_base);
            ccb_mdio_base = NULL;
        }
    } else {
        val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
        val |= 0x01000000;      /* 24:PLL_RESETB = 1 */
        writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));

        mdelay(20);

        /* check pll_lock */
        mask = (1 << IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK);
        do {
            val = readl_relaxed(IPROC_WRAP_MISC_STATUS_ADDR(wrap_base));
            if ((val & mask) == mask) {
                break;
            } else {
                udelay(10);
                count ++;
            }
        } while(count <= 10);
        if (count > 10) {
            printk(KERN_WARNING "%s : PLL not lock! IPROC_WRAP_MISC_STATUS = 0x%08lx\n",
                   __FUNCTION__, val);
        }

        val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
        val &= ~0x00800000;      /* 23:RESETB = 0 */
        writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
        mdelay(100);

#if defined(CONFIG_MACH_HX4)
        val = readl_relaxed(IPROC_WRAP_IPROC_XGPLL_CTRL_4_ADDR(wrap_base));
        ndiv = ((val >> IPROC_WRAP_IPROC_XGPLL_CTRL_4__NDIV_INT_R) &
               ~(0xFFFFFFFF << IPROC_WRAP_IPROC_XGPLL_CTRL_4__NDIV_INT_WIDTH));
        if (ndiv == IPROC_CLK_NDIV_40) {
            val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            val = (val & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_40;
            writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            udelay(10);
            val = (val & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_40;
            writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            udelay(10);
            val = (val & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_40;
            writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            udelay(10);
        } else if (ndiv == IPROC_CLK_NDIV_20) {
            val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            val = (val & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_20;
            writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            udelay(10);
            val = (val & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_20;
            writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            udelay(10);
            val = (val & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_20;
            writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
            udelay(10);
        }
#endif /* CONFIG_MACH_HX4 */

        val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
        val |= 0x00800000;      /* 23:RESETB = 1 */
        writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_ADDR(wrap_base));
        udelay(100);
    }
#endif /* (defined (CONFIG_MACH_HX4) || defined (CONFIG_MACH_KT2)) */

    return 0;
}

static int iproc_usb_phy_sb2_config(struct iproc_usb_priv *iproc_usb_data)
{
#if defined(CONFIG_MACH_SB2)
    void __iomem *wrap_base = iproc_usb_data->wrap_base;
    ulong val, mask, count = 0;

    if (!wrap_base) {
        return -EINVAL;
    }

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val |= 0x0c000000;      /* 27:PHY_ISO & 26:PLL_SUSPEND_EN = 1 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val &= ~0x03000000;     /* 25:PLL_RESETB & 24:RESETB = 0 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    val &= ~0x03000000;     /* 25:AFE_BG_PWRDWNB & 24:AFE_LDO_PWRDWNB = 0 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    udelay(10);
    val |= 0x02000000;      /* 25:AFE_BG_PWRDWNB = 1 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    udelay(150);
    val |= 0x01000000;      /* 24:AFE_LDO_PWRDWNB = 1 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    udelay(160);

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val &= ~0x08000000;     /* 27:PHY_ISO = 0 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    udelay(20);
    val |= 0x02000000;      /* 25:PLL_RESETB = 1 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    mdelay(20);

    /* check pll_lock */
    mask = (1 << IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK);
    do {
        val = readl_relaxed(IPROC_WRAP_MISC_STATUS_ADDR(wrap_base));
        if ((val & mask) == mask) {
            break;
        } else {
            udelay(10);
            count ++;
        }
    } while(count <= 10);

    if (count > 10) {
        printk(KERN_WARNING "%s : PLL not lock! IPROC_WRAP_MISC_STATUS = 0x%08lx\n",
               __FUNCTION__, val);
    }

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val |= 0x01000000;      /* 24:RESETB = 1 */
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    udelay(2);
#endif /* defined(CONFIG_MACH_SB2) */

    return 0;
}

static int iproc_usb_phy_gh_config(struct iproc_usb_priv *iproc_usb_data)
{
#if (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) || defined (CONFIG_MACH_GH2))
    void __iomem *wrap_base = iproc_usb_data->wrap_base;
    ulong val, mask, count = 0;

    if (!wrap_base) {
        return -EINVAL;
    }

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

    /* set phy_resetb to 0, pll_resetb to 0 */
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    /* set p1ctl[11] to 0 */
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

    /* set phy_iso to 0 */
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

    /* set phy_iddq to 0 */
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    mdelay(1);

    /* set pll_resetb to 1, phy_resetb to 1 */
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_ADDR(wrap_base));

    mdelay(20);

    /* check pll_lock */
    mask = (1 << IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK);
    do {
        val = readl_relaxed(IPROC_WRAP_MISC_STATUS_ADDR(wrap_base));
        if ((val & mask) == mask) {
            break;
        } else {
            udelay(10);
            count ++;
        }
    } while(count <= 10);

    if (count > 10) {
        printk(KERN_WARNING "%s : PLL not lock! IPROC_WRAP_MISC_STATUS = 0x%08lx\n",
                    __FUNCTION__, val);
    }

    /* set non_drving to 0 */
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));

    /* set p1ctl[11] to 1 */
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11);
    writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_ADDR(wrap_base));
#endif /* (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) || defined (CONFIG_MACH_GH2)) */

    return 0;
}

static int iproc_usb_phy_init(struct usb_phy *phy)
{
    struct iproc_usb_priv *iproc_usb_data = container_of(phy, struct iproc_usb_priv, phy);
    struct device *dev = iproc_usb_data->dev;
    const struct of_device_id *match;
    int ret = 0;
    uint val;

    if (!iproc_usb_data->wrap_base || !iproc_usb_data->idm_base) {
        return -EINVAL;
    }

    match = of_match_device(xgs_iproc_usb_phy_dt_ids, dev);
    if (!match) {
        dev_err(dev, "failed to find USB PHY in DT\n");
        return -ENODEV;
    }

    /* Put USBD controller into reset state and disable clock via IDM registers */
    val = readl_relaxed(USB2D_IDM_IDM_RESET_CONTROL_ADDR(iproc_usb_data->idm_base));
    val |= (1 << USB2D_IDM_IDM_RESET_CONTROL__RESET);
    writel_relaxed(val, USB2D_IDM_IDM_RESET_CONTROL_ADDR(iproc_usb_data->idm_base));

    val = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_ADDR(iproc_usb_data->idm_base));
    val &= ~(1 << USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
    writel_relaxed(val, USB2D_IDM_IDM_IO_CONTROL_DIRECT_ADDR(iproc_usb_data->idm_base));

	if (strstr(match->compatible, "hx4") ||
	    strstr(match->compatible, "kt2")) {
	    ret = iproc_usb_phy_hx4_config(iproc_usb_data);
    } else if (strstr(match->compatible, "sb2")) {
        ret = iproc_usb_phy_sb2_config(iproc_usb_data);
    } else {
        ret = iproc_usb_phy_gh_config(iproc_usb_data);
    }

    /* Enable clock to USBD and get the USBD out of reset  */
    val = readl_relaxed(USB2D_IDM_IDM_IO_CONTROL_DIRECT_ADDR(iproc_usb_data->idm_base));
    val |= (1 << USB2D_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
    writel_relaxed(val, USB2D_IDM_IDM_IO_CONTROL_DIRECT_ADDR(iproc_usb_data->idm_base));

    mdelay(10);
    val = readl_relaxed(USB2D_IDM_IDM_RESET_CONTROL_ADDR(iproc_usb_data->idm_base));
    val &= ~(1 << USB2D_IDM_IDM_RESET_CONTROL__RESET);
    writel_relaxed(val, USB2D_IDM_IDM_RESET_CONTROL_ADDR(iproc_usb_data->idm_base));

    return ret;
}

static int xgs_iproc_usb_phy_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct device_node *dn = pdev->dev.of_node;
    struct iproc_usb_priv *iproc_usb_data;
    int gpio_pin;
    enum of_gpio_flags flags;
    u32 gpio_active_low;
    int ret, usb_mode;

    if (!of_device_is_available(dn)) {
        return -ENODEV;
    }

    iproc_usb_data = devm_kzalloc(dev, sizeof(*iproc_usb_data), GFP_KERNEL);
    if (!iproc_usb_data) {
        dev_err(dev, "devm_kzalloc() failed\n" );
        return -ENOMEM;
    }
    memset(iproc_usb_data, 0, sizeof(*iproc_usb_data));
    platform_set_drvdata(pdev, iproc_usb_data);

    iproc_usb_data->dev = dev;
    iproc_usb_data->dn = dn;

    iproc_usb_data->wrap_base = get_iproc_wrap_ctrl_base();
    if (!iproc_usb_data->wrap_base) {
        dev_err(&pdev->dev, "can't iomap usb phy base address\n");
        ret = -ENOMEM;
        goto err1;
    }

    gpio_pin = of_get_named_gpio_flags(dn, "vbus-gpio", 0, &flags);
    
    if (gpio_pin < 0) {
        dev_err(&pdev->dev, "Error: no gpio pin set for USB power\n");
        return gpio_pin;
    }    
    
    gpio_active_low = flags & OF_GPIO_ACTIVE_LOW;

    ret = gpio_request(gpio_pin, "usbphy-vbus");
    if (ret != 0) {
        dev_err(dev, "request gpio #%d error.\n", gpio_pin);
        goto err1;
    }

    usb_mode = xgs_iproc_usb_phy_mode(iproc_usb_data);

    iproc_usb_data->usb_mode = usb_mode;
    iproc_usb_data->phy.dev = dev;
    iproc_usb_data->phy.flags = usb_mode;
    iproc_usb_data->phy.init = iproc_usb_phy_init;
    iproc_usb_data->phy.type = USB_PHY_TYPE_USB2;

    if (usb_mode == IPROC_USB_MODE_DEVICE) {
        iproc_usb_data->idm_base = (void *)of_iomap(dn, 1);
        if (!iproc_usb_data->idm_base) {
            dev_err(&pdev->dev, "can't iomap usb2d idm base address 1\n");
            ret = -ENOMEM;
            goto err2;
        }

        gpio_direction_input(gpio_pin);
    } else {
        iproc_usb_data->idm_base = (void *)of_iomap(dn, 0);
        if (!iproc_usb_data->idm_base) {
            dev_err(&pdev->dev, "can't iomap usb2h idm base address 1\n");
            ret = -ENOMEM;
            goto err2;
        }

        gpio_direction_output(gpio_pin, 1);

        /*turn off the power: if active low for power, then set 1 to turn off*/
        if (gpio_active_low)
            __gpio_set_value(gpio_pin, 1);
        else
            __gpio_set_value(gpio_pin, 0);

        /* 
           Initial usb phy for usb hose mode. For the device mode, 
           the iproc_usb_phy_init will be called when usb udc start.
        */
        ret = iproc_usb_phy_init(&iproc_usb_data->phy);
        if (ret < 0)
            goto err2;
    }

    ret = usb_add_phy_dev(&iproc_usb_data->phy);
    if (ret)
        goto err2;

    /* supply power for USB device connected to the host */
    if (usb_mode != IPROC_USB_MODE_DEVICE) {
        if (gpio_active_low)
            __gpio_set_value(gpio_pin, 0);
	else
            __gpio_set_value(gpio_pin, 1);
    }
    gpio_free(gpio_pin);

    return 0;

err2:
    gpio_free(gpio_pin);
err1:
    if (iproc_usb_data->idm_base) {
        iounmap(iproc_usb_data->idm_base);
    }
    if (iproc_usb_data) {
        iounmap(iproc_usb_data);
    }

    return ret;
}

static int xgs_iproc_usb_phy_remove(struct platform_device *pdev)
{
    struct iproc_usb_priv *iproc_usb_data = platform_get_drvdata(pdev);

    platform_set_drvdata(pdev, NULL);
    if (iproc_usb_data->idm_base) {
        iounmap(iproc_usb_data->idm_base);

	    usb_remove_phy(&iproc_usb_data->phy);
    }

    if (iproc_usb_data) {
        iounmap(iproc_usb_data);
    }

    return 0;
}

static struct platform_driver xgs_iproc_usb_phy_driver =
{
    .driver = {
        .name = "usb-phy",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(xgs_iproc_usb_phy_dt_ids),
    },
    .probe   = xgs_iproc_usb_phy_probe,
    .remove  = xgs_iproc_usb_phy_remove,
};

module_platform_driver(xgs_iproc_usb_phy_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB phy driver");
MODULE_LICENSE("GPL");
