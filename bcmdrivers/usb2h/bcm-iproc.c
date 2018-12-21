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
 *
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <mach/memory.h>
#include <linux/version.h>

#include "usbh_cfg.h"

#include "bcm_usbh.h"

#define DEBUG
#ifdef DEBUG
#define dbg_printk(fmt, args...) printk(KERN_INFO "%s: " fmt, __func__, ## args)
#else
#define dbg_printk(fmt, args...)
#endif

#define IPROC_USB2_CLK_CONTROL_ENABLE           (0x1800C180)
#define IPROC_USB2_CLK_CONTROL_ENABLE_VA        HW_IO_PHYS_TO_VIRT(IPROC_USB2_CLK_CONTROL_ENABLE)
#define IPROC_USB2_CLK_CONTROL_PLL              (0x1800C164)
#define IPROC_USB2_CLK_CONTROL_PLL_VA           HW_IO_PHYS_TO_VIRT(IPROC_USB2_CLK_CONTROL_PLL)
#define IPROC_STRAP_SKU_VECTOR                  (0x1810D500)
#define IPROC_STRAP_SKU_VECTOR_VA               HW_IO_PHYS_TO_VIRT(IPROC_STRAP_SKU_VECTOR)
#define IPROC_IDM_USB2_RESET_CONTROL            (0x18115800)
#define IPROC_IDM_USB2_RESET_CONTROL_VA         HW_IO_PHYS_TO_VIRT(IPROC_IDM_USB2_RESET_CONTROL)

#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) || defined(CONFIG_MACH_GH) || \
	defined(CONFIG_MACH_HR3) || defined(CONFIG_MACH_SB2)
#define IPROC_IDM_USB2_IO_CONTROL_DIRECT        USB2_IDM_IDM_IO_CONTROL_DIRECT
#define IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA     HW_IO_PHYS_TO_VIRT(IPROC_IDM_USB2_IO_CONTROL_DIRECT)
#endif
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
#define IPROC_XGPLL                             0x1803fc2c
#define IPROC_XGPLL_VA                          HW_IO_PHYS_TO_VIRT(IPROC_XGPLL)
#define IPROC_USB_PHY_CTRL                      IPROC_WRAP_USBPHY_CTRL
#define IPROC_USB_PHY_CTRL_VA                   HW_IO_PHYS_TO_VIRT(IPROC_USB_PHY_CTRL)
#define IPROC_WRAP_MISC_STATUS                  0x1803fc28
#define IPROC_WRAP_MISC_STATUS_VA               HW_IO_PHYS_TO_VIRT(IPROC_WRAP_MISC_STATUS)
#define IPROC_CLK_NDIV_40                       0x80
#define IPROC_CLK_NDIV_20                       0x8C
#define USB_CLK_NDIV_MASK                       0xFE7FFE00
#define USB_CLK_PLL_RESET_MASK                  0xFF7FFE00
#define USB_CLK_PHY_RESET_MASK                  0xFFFFFE00
#define USB_CLK_NDIV_40                         0x30
#define USB_CLK_NDIV_20                         0x60
#define ChipcommonA_GPIOIn_VA                   HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOInput)
#define ChipcommonA_GPIOOut_VA                  HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOOut)
#define ChipcommonA_GPIOOutEn_VA                HW_IO_PHYS_TO_VIRT(ChipcommonA_GPIOOutEn)
#define SUPPLY_USBD_POWER                       0xfffffffd
#elif defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
#define IPROC_WRAP_USBPHY_CTRL_0                0x1800fc44
#define IPROC_WRAP_USBPHY_CTRL_0_VA             HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_0)
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ      26 /* Port 0 */
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB    25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB        24
#define IPROC_WRAP_USBPHY_CTRL_2                0x1800fc4c
#define IPROC_WRAP_USBPHY_CTRL_2_VA             HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_2)
#define IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO       17
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0      0
#define IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11     11
#define IPROC_WRAP_MISC_STATUS                  0x1800fc58
#define IPROC_WRAP_MISC_STATUS_VA               HW_IO_PHYS_TO_VIRT(IPROC_WRAP_MISC_STATUS)
#define IPROC_WRAP_MISC_STATUS__USBPHY_LDO_ON_FLAG  2
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK     1
#define USBH_Phy_Ctrl_P0_VA                     HW_IO_PHYS_TO_VIRT(USBH_Phy_Ctrl_P0)
#define USBH_Phy_Ctrl_P1_VA                     HW_IO_PHYS_TO_VIRT(USBH_Phy_Ctrl_P1)
#define USBH_VBUS_GPIO                          10
#elif defined(CONFIG_MACH_SB2)
#define IPROC_WRAP_USBPHY_CTRL_0                0x1800fc28
#define IPROC_WRAP_USBPHY_CTRL_0_VA             HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_0)
#define IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO       27
#define IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB    25
#define IPROC_WRAP_USBPHY_CTRL_0__RESETB        24
#define IPROC_WRAP_USBPHY_CTRL_5                0x1800fc3c
#define IPROC_WRAP_USBPHY_CTRL_5_VA             HW_IO_PHYS_TO_VIRT(IPROC_WRAP_USBPHY_CTRL_5)
#define IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B0      0
#define IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B11     11
#define IPROC_WRAP_MISC_STATUS                  0x1800fc44
#define IPROC_WRAP_MISC_STATUS_VA               HW_IO_PHYS_TO_VIRT(IPROC_WRAP_MISC_STATUS)
/* #define IPROC_WRAP_MISC_STATUS__USBPHY_LDO_ON_FLAG	2 */
#define IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK 1
#define USBH_Phy_Ctrl_P0_VA                     HW_IO_PHYS_TO_VIRT(USBH_Phy_Ctrl_P0)
#define USBH_Phy_Ctrl_P1_VA                     HW_IO_PHYS_TO_VIRT(USBH_Phy_Ctrl_P1)
#define USBH_VBUS_GPIO                          1
#define IPROC_WRAP_IPROC_STRAP_CTRL             0x1800fc70
#endif

#define IPROC_SKU_STRAP_MASK                    0xC

struct usbh_ctrl_regs {
	u32 mode;
#define MODE_ULPI_TTL                       (1<<0)
#define MODE_ULPI_PHY                       (1<<1)
#define MODE_UTMI_TTL                       (1<<2)
#define MODE_UTMI_PHY                       (1<<3)
#define MODE_PORT_CFG(port, mode) ((mode) << (4 * port))

	u32 strap_q;
#define STRAP_PWR_STATE_VALID               (1 << 7)    /* ss_power_state_valid */
#define STRAP_SIM_MODE                      (1 << 6)    /* ss_simulation_mode */
#define STRAP_OHCI_CNTSEL_SIM               (1 << 5)    /* ohci_0_cntsel_i_n */
#define STRAP_PWR_STATE_NXT_VALID           (1 << 4)    /* ss_nxt_power_state_valid_i */
#define STRAP_PWR_STATE_NXT_SHIFT           2           /* ss_next_power_state_i */
#define STRAP_PWR_STATE_NXT_MASK            (3 << STRAP_PWR_STATE_NXT_SHIFT)
#define STRAP_PWR_STATE_SHIFT               0           /* ss_power_state_i */
#define STRAP_PWR_STATE_MASK                (3 << STRAP_PWR_STATE_SHIFT)

	u32 framelen_adj_q;
	u32 framelen_adj_qx[USBH_NUM_PORTS];       
	u32 misc;
#define MISC_RESUME_R23_ENABLE              (1 << 4) /* ss_utmi_backward_enb_i */
#define MISC_RESUME_R23_UTMI_PLUS_DISABLE   (1 << 3) /* ss_resume_utmi_pls_dis_i */
#define MISC_ULPI_BYPASS_ENABLE             (1 << 2) /* ulpi_bypass_en_i */
#define MISC_PORT_PWRDWN_OVERCURRENT        (1 << 1) /* ss_autoppd_on_overcur_en_i */
#define MISC_OHCI_CLK_RESTART               (1 << 0) /* app_start_clk_i */
};

struct usbh_priv {
	atomic_t probe_done;
	volatile int init_cnt;
	struct mutex lock;
	struct device *dev;
	struct usbh_cfg hw_cfg;
	struct clk *peri_clk;
	struct clk *ahb_clk;
	struct clk *opt_clk;
	struct usbh_ctrl_regs __iomem *ctrl_regs;
};

static struct usbh_priv usbh_data;

int bcm_usbh_suspend(unsigned int host_index)
{
	return 0;
}
EXPORT_SYMBOL(bcm_usbh_suspend);

int bcm_usbh_resume(unsigned int host_index)
{
	return 0;
}
EXPORT_SYMBOL(bcm_usbh_resume);

#if (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) || defined(CONFIG_MACH_SB2))
int bcm_gh_usbphy_init()
{
	unsigned long val, mask;
	int count = 0;
	/* set phy_iso to 1, phy_iddq to 1, non_driving to 1 */
#if defined(CONFIG_MACH_SB2)
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
#else
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
#endif

#if !defined(CONFIG_MACH_SB2)
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
#endif

#if defined(CONFIG_MACH_SB2)
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_5_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B0);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_5_VA);
#else
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
#endif

	/* set phy_resetb to 0, pll_resetb to 0 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
	
	/* set p1ctl[11] to 0 */
#if defined(CONFIG_MACH_SB2)
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_5_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B11);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_5_VA);
#else
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
#endif

    /* set phy_iso to 0 */
#if defined(CONFIG_MACH_SB2)
    val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
#else
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
#endif

#if !defined(CONFIG_MACH_SB2)	
	/* set phy_iddq to 0 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);
#endif
	mdelay(1);

	/* set pll_resetb to 1, phy_resetb to 1 */
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_0_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_0_VA);

	mdelay(20);
	
	/* check pll_lock */
	mask = (1 << IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK);
	do {
		val = readl_relaxed(IPROC_WRAP_MISC_STATUS_VA);
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
#if defined(CONFIG_MACH_SB2)
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_5_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B0);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_5_VA);
#else
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B0);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
#endif

    /* set p1ctl[11] to 1 */
#if defined(CONFIG_MACH_SB2)
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_5_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_B11);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_5_VA);
#else
	val = readl_relaxed(IPROC_WRAP_USBPHY_CTRL_2_VA);
	val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_B11);
	writel_relaxed(val, IPROC_WRAP_USBPHY_CTRL_2_VA);
#endif
	return 0;
}

int bcm_gh_usbh_init(unsigned int host_index)
{
	unsigned long val, mask;
	int count = 0;
	static int initialized = 0;

	if (initialized) {
		return 0;
	}
	
	if (gpio_request(USBH_VBUS_GPIO, "USB2H") == 0) {	
		gpio_direction_output(USBH_VBUS_GPIO, 1);
#if defined(CONFIG_MACH_SB2)
        /* Saber2 is low-active */
		__gpio_set_value(USBH_VBUS_GPIO, 0);
#else
		__gpio_set_value(USBH_VBUS_GPIO, 1);
#endif
		gpio_free(USBH_VBUS_GPIO);

		bcm_gh_usbphy_init();
	} else {
		printk(KERN_WARNING "%s : Failed to request gpio %d !\n", __FUNCTION__, USBH_VBUS_GPIO);
	}

	/* USB Host clock enable */
	val = readl_relaxed(IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	writel_relaxed(val | 1, IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);

	/* Bring USB Host out of reset */
	val = readl_relaxed(IPROC_IDM_USB2_RESET_CONTROL_VA);
	writel_relaxed(val | 1, IPROC_IDM_USB2_RESET_CONTROL_VA);
	writel_relaxed(val & ~1, IPROC_IDM_USB2_RESET_CONTROL_VA);

	writel_relaxed(0x3ff, USBH_Phy_Ctrl_P0_VA);
	mdelay(100);

	initialized = 1;
	
	return 0;
}
#endif

/*
 * Function to initialize USB host related low level hardware including PHY,
 * clocks, etc.
 *
 * TODO: expand support for more than one host in the future if needed
 */
int bcm_usbh_init(unsigned int host_index)
{
#if (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) || defined(CONFIG_MACH_SB2))
	bcm_gh_usbh_init(host_index);
#endif
	return 0;
}

EXPORT_SYMBOL(bcm_usbh_init);
	
/*
 * Function to terminate USB host related low level hardware including PHY,
 * clocks, etc.
 *
 * TODO: expand support for more than one host in the future if needed
 */
int bcm_usbh_term(unsigned int host_index)
{
	return 0;
}
EXPORT_SYMBOL(bcm_usbh_term);

int InUSBDMode(void)
{
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
	int usbd_detect;
	usbd_detect = readl_relaxed(ChipcommonA_GPIOIn_VA);
	if (usbd_detect & 1) {
		printk("%s: %d gpioin val %08x, ohci host mode will not be functional since in USBD mode\n", __FUNCTION__, __LINE__, usbd_detect);
		printk("%s: %d to make ohci host mode work, appropriate jumper is needed on the board. Please refer to board schematics.\n",
			__FUNCTION__, __LINE__);
	}

	return (usbd_detect & 1);
#elif defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
	if ((readl_relaxed(HW_IO_PHYS_TO_VIRT(IPROC_WRAP_TOP_STRAP_STATUS)) & (1 << 17)) == 0) {
		return 1;
	} else {
		return 0;
	}
#elif defined(CONFIG_MACH_SB2)
    /* u-boot enable this bit to indicate usb in host mode */
    if ((readl_relaxed(HW_IO_PHYS_TO_VIRT(IPROC_WRAP_IPROC_STRAP_CTRL)) & (1 << 10)) == 0) {
		return 1;
	} else {
		return 0;
	}
#else
	return 0;
#endif
}
	
static int __devinit usbh_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *iomem, *ioarea;

	memset(&usbh_data, 0, sizeof(usbh_data));

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "platform_data missing\n");
		ret = -EFAULT;
		goto err_exit;
	}
	memcpy(&usbh_data.hw_cfg, pdev->dev.platform_data, sizeof(usbh_data.hw_cfg));
	usbh_data.dev = &pdev->dev;
	
	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem) {
		dev_err(&pdev->dev, "no mem resource\n");
		ret = -ENODEV;
		goto err_exit;
	}

	/* mark the memory region as used */
	ioarea = request_mem_region(iomem->start, resource_size(iomem), pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "memory region already claimed\n");
		ret = -EBUSY;
		goto err_exit;
	}

	/* now map the I/O memory */
	usbh_data.ctrl_regs = (struct usbh_ctrl_regs __iomem *)ioremap(iomem->start, 
	                                                sizeof(usbh_data.ctrl_regs));
	if (!usbh_data.ctrl_regs) {
		dev_err(&pdev->dev, "failed to remap registers\n");
		ret = -ENOMEM;
		goto err_free_mem_region;
	}

	platform_set_drvdata(pdev, &usbh_data);
	mutex_init(&usbh_data.lock);
	usbh_data.init_cnt = 0;
	atomic_set(&usbh_data.probe_done, 1);

	return 0;

err_free_mem_region:
	release_mem_region(iomem->start, resource_size(iomem));

err_exit:
	memset(&usbh_data, 0, sizeof(usbh_data));
	return ret;
}

static int __devexit usbh_remove(struct platform_device *pdev)
{
	struct usbh_priv *drv_data = platform_get_drvdata(pdev);
	struct resource *iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	atomic_set(&drv_data->probe_done, 0);
	platform_set_drvdata(pdev, NULL);
	iounmap(drv_data->ctrl_regs);
	release_mem_region(iomem->start, resource_size(iomem));
	memset(&usbh_data, 0, sizeof(usbh_data));

	return 0;
}

static struct platform_driver usbh_driver = 
{
	.driver = {
		.name = "usbh",
		.owner = THIS_MODULE,
	},
	.probe   = usbh_probe,
	.remove  = usbh_remove,
};

static int __init usbh_init(void)
{
#if !(defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3))
	int usb2_reset_state;
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
	int clk_enable, k;
#if defined(CONFIG_MACH_HX4)
	unsigned int iClk;
#endif
	unsigned int USBClk, usbdgpiopwr, pllStatus;

	/* turn off power for USB device connected to the host */
	usbdgpiopwr = readl_relaxed(ChipcommonA_GPIOOut_VA);
	usbdgpiopwr |= 0x2;
	writel_relaxed(usbdgpiopwr, ChipcommonA_GPIOOut_VA);
	writel_relaxed(0x2, ChipcommonA_GPIOOutEn_VA);

	/* Do USB PHY reset */
	mdelay(100);
	USBClk = readl_relaxed(IPROC_USB_PHY_CTRL_VA);
	/* bring phy pll out of reset if not done already */
	if ((USBClk & 0x01000000) == 0 ) {
		USBClk |= 0x01000000;
		writel_relaxed(USBClk, IPROC_USB_PHY_CTRL_VA);
		pllStatus = readl_relaxed(IPROC_WRAP_MISC_STATUS_VA);
		for (k = 0; k < 100000; k++) {
			if ((pllStatus & 2) == 2) {
				printk("USB phy pll locked\n");
				break;
			}
			pllStatus = readl_relaxed(IPROC_WRAP_MISC_STATUS_VA);
		}
	}
	writel_relaxed(USBClk & (~(1<<23)), IPROC_USB_PHY_CTRL_VA);
	clk_enable = readl_relaxed(IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	printk("Initial usb2h clock is: %08x\n", clk_enable);
	clk_enable |= 1;
	writel_relaxed(clk_enable, IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	clk_enable = readl_relaxed(IPROC_IDM_USB2_IO_CONTROL_DIRECT_VA);
	printk("Initial usb2h clock now is: %08x\n", clk_enable);
#if defined(CONFIG_MACH_HX4)
	iClk = readl_relaxed(IPROC_XGPLL_VA);
	USBClk = readl_relaxed(IPROC_USB_PHY_CTRL_VA);
	printk("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
	if ((iClk & 0xff) == IPROC_CLK_NDIV_40) {
		writel_relaxed((USBClk & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_40, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		writel_relaxed((USBClk & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_40, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		writel_relaxed((USBClk & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_40, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		USBClk = readl_relaxed(IPROC_USB_PHY_CTRL_VA);
		printk("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
	} else if ((iClk & 0xff) == IPROC_CLK_NDIV_20) {
		writel_relaxed((USBClk & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_20, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		writel_relaxed((USBClk & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_20, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		writel_relaxed((USBClk & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_20, IPROC_USB_PHY_CTRL_VA);
		udelay(10);
		USBClk = readl_relaxed(IPROC_USB_PHY_CTRL_VA);
		printk("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
	}
#endif
	mdelay(100);
	writel_relaxed(USBClk | (1<<23), IPROC_USB_PHY_CTRL_VA);
	udelay(100);
#endif

	usb2_reset_state = readl_relaxed(IPROC_IDM_USB2_RESET_CONTROL_VA);
	printk("Initial usb2_reset_state is: %08x\n", usb2_reset_state);
	if ((usb2_reset_state & 1) == 1) {
		writel_relaxed(0x0, IPROC_IDM_USB2_RESET_CONTROL_VA);
		usb2_reset_state = readl_relaxed(IPROC_IDM_USB2_RESET_CONTROL_VA);
		printk("usb2_reset_state is set and now it is: %08x\n", usb2_reset_state);
	}
#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
	/* supply power for USB device connected to the host */
	mdelay(100);
	usbdgpiopwr = readl_relaxed(ChipcommonA_GPIOOut_VA);
	usbdgpiopwr &= SUPPLY_USBD_POWER;
	writel_relaxed(usbdgpiopwr, ChipcommonA_GPIOOut_VA);
	writel_relaxed(0x2, ChipcommonA_GPIOOutEn_VA);
#endif
#endif /* !(defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3)) */

	return platform_driver_register(&usbh_driver);
}

static void __exit usbh_exit(void)
{
	platform_driver_unregister(&usbh_driver);
}

module_init(usbh_init);
module_exit(usbh_exit);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB host low-level driver");
MODULE_LICENSE("GPL");
