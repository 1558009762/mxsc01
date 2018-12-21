/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <usb.h>
#include <asm/io.h>

#include <asm/arch/socregs.h>
#include "ehci.h"

#define BCM_USB_FIFO_THRESHOLD 0x00800040
#define USBH_INSNREG01         0x18048094

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
static uint32_t swap_u32(uint32_t i) {
    uint8_t c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return ((uint32_t)c1 << 24) + ((uint32_t)c2 << 16) + ((uint32_t)c3 << 8) + c4;
}

static void ehci_iol_w(uint32_t val, volatile uint32_t *addr)
{
    *addr = swap_u32(val);
}

static uint32_t ehci_iol_r(volatile uint32_t *addr)
{
    return(swap_u32(*addr));
}
#else
static void ehci_iol_w(uint32_t val, volatile uint32_t *addr)
{
    *addr = val;
}

static uint32_t ehci_iol_r(volatile uint32_t *addr)
{
    return(*addr);
}
#endif


#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_SABER2))

#define IPROC_EHCI                          USBH_HCCAPBASE
#if defined(CONFIG_GREYHOUND)
#define USBH_VBUS_GPIO                      10
#define USBH_ENABLE_VBUS(val, gpio)         val |= (1 << gpio)
#define USB_IN_DEVICE_MODE() ((ehci_iol_r((volatile uint32_t *)IPROC_WRAP_TOP_STRAP_STATUS) & (1 << 17)) == 0)
#elif defined(CONFIG_HURRICANE3)
#define USBH_VBUS_GPIO                      7
#define USBH_ENABLE_VBUS(val, gpio)         val |= (1 << gpio)
#define USB_IN_DEVICE_MODE() ((ehci_iol_r((volatile uint32_t *)IPROC_WRAP_TOP_STRAP_STATUS) & (1 << 17)) == 0)
#elif defined(CONFIG_SABER2)
#define USBH_VBUS_GPIO                      1
#define USBH_ENABLE_VBUS(val, gpio)         val &= ~(1 << gpio)
#define USB_IN_DEVICE_MODE() ((ehci_iol_r((volatile uint32_t *)IPROC_WRAP_IPROC_STRAP_CTRL) & (1 << 10)) == 0)
#endif /* defined(CONFIG_GREYHOUND) */

#elif ((defined(CONFIG_HELIX4)) || (defined(CONFIG_KATANA2)))

#define IPROC_USB2_CLK_CONTROL_ENABLE       (0x1803f180)
#define IPROC_USB2_CLK_CONTROL_PLL          (0x1803f164)
#define IPROC_STRAP_SKU_VECTOR              (0x1811A500)
#define IPROC_IDM_USB2_RESET_CONTROL        (0x18115800)
#define IPROC_EHCI                          (0x1802a000)
#define IPROC_SKU_STRAP_MASK                0xC
#define IPROC_XGPLL                         0x1803fc2c
#define IPROC_WRAP_MISC_STATUS              0x1803fc28
#define IPROC_USB_PHY_CTRL                  IPROC_WRAP_USBPHY_CTRL
#define IPROC_CLK_NDIV_40                   0x80
#define IPROC_CLK_NDIV_20                   0x8C
#define USB_CLK_NDIV_MASK                   0xFE7FFE00
#define USB_CLK_PLL_RESET_MASK              0xFF7FFE00
#define USB_CLK_PHY_RESET_MASK              0xFFFFFE00
#define USB_CLK_NDIV_40                     0x30
#define USB_CLK_NDIV_20                     0x60
#define SUPPLY_USBD_POWER                   0xfffffffd
#define ChipcommonA_GPIOOut                 0x18000064
#define ChipcommonA_GPIOOutEn               0x18000068
#ifdef CONFIG_HELIX4
#define IPROC_WRAP_USBPHY_CTRL              0x1803fc34
#else
#define IPROC_WRAP_USBPHY_CTRL              0x1803fc20
#endif /* CONFIG_HELIX4 */

#else

#define IPROC_USB2_CLK_CONTROL_ENABLE       (0x1800C180)
#define IPROC_USB2_CLK_CONTROL_PLL          (0x1800C164)
#define IPROC_STRAP_SKU_VECTOR              (0x1810D500)
#define IPROC_IDM_USB2_RESET_CONTROL        (0x18115800)
#define IPROC_EHCI                          (0x18021000)
#define IPROC_SKU_STRAP_MASK                0xC

#endif  /* defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_SABER2) */


#if ((defined(CONFIG_HELIX4)) || (defined(CONFIG_KATANA2)))
/*
 * Function to initialize USB host related low level hardware including PHY,
 * clocks, etc.
 *
 * TODO: expand support for more than one host in the future if needed
 */
int linux_usbh_init(void)
{
    int clk_enable, k;
    unsigned int iClk, USBClk, usbdgpiopwr, usb2_reset_state, pllStatus;

    /* Do USB PHY reset */
    USBClk = ehci_iol_r((volatile uint32_t *)IPROC_USB_PHY_CTRL);
    /* bring phy pll out of reset if not done already */
    printf("phy ctrl %08x\n", USBClk);
    if ((USBClk & 0x01000000) == 0) {
        USBClk |= 0x01000000;
        ehci_iol_w(USBClk, (volatile uint32_t *)IPROC_USB_PHY_CTRL);
        pllStatus = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_MISC_STATUS);
        for (k = 0; k < 100000; k++) {
            if ((pllStatus & 2) == 2) {
                printf("USB phy pll locked\n");
                break;
            }
            pllStatus = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_MISC_STATUS);
        }
    }
    udelay(1000);
    USBClk = ehci_iol_r((volatile uint32_t *)IPROC_USB_PHY_CTRL);
    ehci_iol_w(USBClk & (~(1<<23)), (volatile uint32_t *)IPROC_USB_PHY_CTRL);
    udelay(1000);

    /* enable clock */
    clk_enable = ehci_iol_r((volatile uint32_t *)USB2_IDM_IDM_IO_CONTROL_DIRECT);
    printf("Initial usb2h clock is: %08x\n", clk_enable);
    clk_enable |= 1;
    printf("Initial usb2h clock is: %08x\n", clk_enable);
    ehci_iol_w(clk_enable, (volatile uint32_t *)USB2_IDM_IDM_IO_CONTROL_DIRECT);

#if defined(CONFIG_HELIX4)
    /* set USB clock to configured */
    iClk = ehci_iol_r((volatile uint32_t *)IPROC_XGPLL);
    USBClk = ehci_iol_r((volatile uint32_t *)IPROC_USB_PHY_CTRL);
    printf("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
    if ((iClk & 0xff) == IPROC_CLK_NDIV_40) {
        ehci_iol_w((USBClk & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_40, (volatile uint32_t *)IPROC_USB_PHY_CTRL);
        udelay(10);
        ehci_iol_w((USBClk & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_40, (volatile uint32_t *)IPROC_USB_PHY_CTRL);
        udelay(10);
        ehci_iol_w((USBClk & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_40, (volatile uint32_t *)IPROC_USB_PHY_CTRL);
        udelay(10);
        USBClk = ehci_iol_r((volatile uint32_t *)IPROC_USB_PHY_CTRL);
        printf("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
    } else if ((iClk & 0xff) == IPROC_CLK_NDIV_20) {
        ehci_iol_w((USBClk & USB_CLK_NDIV_MASK) | USB_CLK_NDIV_20, (volatile uint32_t *)IPROC_USB_PHY_CTRL);
        udelay(10);
        ehci_iol_w((USBClk & USB_CLK_PLL_RESET_MASK) | USB_CLK_NDIV_20, (volatile uint32_t *)IPROC_USB_PHY_CTRL);
        udelay(10);
        ehci_iol_w((USBClk & USB_CLK_PHY_RESET_MASK) | USB_CLK_NDIV_20, (volatile uint32_t *)IPROC_USB_PHY_CTRL);
        udelay(10);
        USBClk = ehci_iol_r((volatile uint32_t *)IPROC_USB_PHY_CTRL);
        printf("iClk = %08x, USBClk = %08x\n", iClk, USBClk);
    }
#endif  /* defined(CONFIG_HELIX4) */

    /* bring USB PHY out of reset */
    ehci_iol_w(USBClk | (1<<23), (volatile uint32_t *)IPROC_USB_PHY_CTRL);
    udelay(100);

    printf("\nBring usb2h_out of reset.......\n");
    ehci_iol_w(0x0, (volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
    udelay(100000);
    usb2_reset_state = ehci_iol_r((volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
    printf("usb2_reset_state is set and now it is: %08x\n", usb2_reset_state);

    /* supply power for USB device connected to the host */
    usbdgpiopwr = ehci_iol_r((volatile uint32_t *)ChipcommonA_GPIOOut);
    usbdgpiopwr &= SUPPLY_USBD_POWER;
    ehci_iol_w(usbdgpiopwr, (volatile uint32_t *)ChipcommonA_GPIOOut);
    ehci_iol_w(0x2, (volatile uint32_t *)ChipcommonA_GPIOOutEn);
    return 0;
}
#endif  /* (defined(CONFIG_HELIX4) || (defined(CONFIG_KATANA2) */


#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_SABER2))
int usbphy_init(void)
{
    uint32_t val, mask;
    int count = 0;
    /* set phy_iso to 1, phy_iddq to 1, non_driving to 1 */
#if defined(CONFIG_SABER2)
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
#else
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
#endif

#if !defined(CONFIG_SABER2)
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
#endif

#if defined(CONFIG_SABER2)
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
#else
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
#endif

    /* set phy_resetb to 0, pll_resetb to 0 */
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);

    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);

    /* set p1ctl[11] to 0 */
#if defined(CONFIG_SABER2)
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
    val &= ~(0x0800 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
#else
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
    val &= ~(0x0800 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
#endif

    /* set phy_iso to 0 */
#if defined(CONFIG_SABER2)
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_ISO);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
#else
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__PHY_ISO);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
#endif

#if !defined(CONFIG_SABER2)
    /* set phy_iddq to 0 */
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_0__PHY_IDDQ_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
#endif
    mdelay(1);

    /* set pll_resetb to 1, phy_resetb to 1 */
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__PLL_RESETB);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);

    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);
    val |= (1 << IPROC_WRAP_USBPHY_CTRL_0__RESETB);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_0);

    mdelay(20);

    /* check pll_lock */
    mask = (1 << IPROC_WRAP_MISC_STATUS__USBPHY_PLL_LOCK);
    do {
        val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_MISC_STATUS);
        if ((val & mask) == mask) {
            break;
        } else {
            udelay(10);
            count ++;
        }
    } while(count <= 10);

    if (count > 10) {
        printf("%s : PLL not lock! IPROC_WRAP_MISC_STATUS = 0x%08x\n", __FUNCTION__, val);
    }

    /* set non_drving to 0 */
#if defined(CONFIG_SABER2)
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
#else
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
    val &= ~(1 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
#endif

    /* set p1ctl[11] to 1 */
#if defined(CONFIG_SABER2)
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
    val |= (0x0800 << IPROC_WRAP_USBPHY_CTRL_5__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_5);
#else
    val = ehci_iol_r((volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
    val |= (0x0800 << IPROC_WRAP_USBPHY_CTRL_2__P1CTL_R);
    ehci_iol_w(val, (volatile uint32_t *)IPROC_WRAP_USBPHY_CTRL_2);
#endif
    return 0;
}
#endif  /* defined(CONFIG_GREYHOUND) || defined(CONFIG_MACH_HURRICANE3) || defined(CONFIG_MACH_SABER2) */


/*
 * Function to initialize USB host related low level hardware including PHY,
 * clocks, etc.
 *
 * TODO: expand support for more than one host in the future if needed
 */
int ehci_hcd_init(int index, enum usb_init_type init,
		struct ehci_hccr **hccr, struct ehci_hcor **hcor)
{
#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_SABER2))
    uint32_t val;

    if (USB_IN_DEVICE_MODE()) {
        printf("Not in USB host mode !!");
        return 1;
    }

    usbphy_init();

    /* USB Host clock enable */
    val = ehci_iol_r((volatile uint32_t *)USB2_IDM_IDM_IO_CONTROL_DIRECT);
#if (defined(CONFIG_IPROC_NO_DDR) && defined(CONFIG_L2C_AS_RAM))
    val |= ((1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__Bypass_CT) |
            (1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__CT));
#endif /* (defined(CONFIG_IPROC_NO_DDR) && defined(CONFIG_L2C_AS_RAM)) */
    val |= (1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
    ehci_iol_w(val, (volatile uint32_t *)USB2_IDM_IDM_IO_CONTROL_DIRECT);

    debug("Enable usb2h clock. (I/O Ctrl Direct: 0x%08x)\n", 
                        ehci_iol_r((volatile uint32_t *)USB2_IDM_IDM_IO_CONTROL_DIRECT));

    /* Bring USB Host out of reset */
    printf("Bring usb2h_out of reset.......\n");
    val = ehci_iol_r((volatile uint32_t *)USB2_IDM_IDM_RESET_CONTROL);
    val |= (1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
    ehci_iol_w(val, (volatile uint32_t *)USB2_IDM_IDM_RESET_CONTROL);
    mdelay(1);
    val &= ~(1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
    ehci_iol_w(val, (volatile uint32_t *)USB2_IDM_IDM_RESET_CONTROL);

    debug("usb2_reset_state is set and now it is: %08x\n", 
                            ehci_iol_r((volatile uint32_t *)USB2_IDM_IDM_RESET_CONTROL));

    val = ehci_iol_r((volatile uint32_t *)USBH_Phy_Ctrl_P0);
    val |= ((0x3 << USBH_Phy_Ctrl_P0__PHY_PLL_Power_Down_R) |
            (0x3 << USBH_Phy_Ctrl_P0__PHY_Test_port_UTMI_Power_Down_R) |
            (0x3 << USBH_Phy_Ctrl_P0__PHY_Test_port_Power_Down_R) |
            (0x3 << USBH_Phy_Ctrl_P0__PHY_Soft_Reset_R) |
            (0x1 << USBH_Phy_Ctrl_P0__Core_Reset) |
            (0x1 << USBH_Phy_Ctrl_P0__Phy_Hard_Reset));
    ehci_iol_w(val, (volatile uint32_t *)USBH_Phy_Ctrl_P0);

    mdelay(100);

    ehci_iol_w(BCM_USB_FIFO_THRESHOLD, (volatile uint32_t *)USBH_INSNREG01);

    /* Turn on USB module */
    val = ehci_iol_r((volatile uint32_t *)ChipcommonG_GP_DATA_OUT);
    USBH_ENABLE_VBUS(val, USBH_VBUS_GPIO);
    ehci_iol_w(val, (volatile uint32_t *)ChipcommonG_GP_DATA_OUT);
    val = ehci_iol_r((volatile uint32_t *)ChipcommonG_GP_OUT_EN);
    val |= (1 << USBH_VBUS_GPIO);
    ehci_iol_w(val, (volatile uint32_t *)ChipcommonG_GP_OUT_EN);

#elif ((defined(CONFIG_HELIX4)) || (defined(CONFIG_KATANA2)))

    int usb2_clk_cntrl, usb2_clk_enable, sku_vect, usb2_reset_state;

    /* Start clock */
    sku_vect = ehci_iol_r((volatile uint32_t *)IPROC_STRAP_SKU_VECTOR);

    linux_usbh_init();

    /* Reset USBH 2.0 core */
    usb2_reset_state = ehci_iol_r((volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
    printf("usb2_reset_state is: %08x\n", usb2_reset_state);
    if ((usb2_reset_state & 1) == 1) {
        printf("\nBring usb2h_out of reset.......\n");
        ehci_iol_w(0x0, (volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
        udelay(1000);
        usb2_reset_state = ehci_iol_r((volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
        printf("usb2_reset_state is set and now it is: %08x\n", usb2_reset_state);
    }

#else
    int usb2_clk_cntrl, usb2_clk_enable, sku_vect, usb2_reset_state;

    /* Start clock */
    sku_vect = ehci_iol_r((volatile uint32_t *)IPROC_STRAP_SKU_VECTOR);
    if ((sku_vect & IPROC_SKU_STRAP_MASK) != 0x0) {
        /* enable clocks */
        ehci_iol_w(0xEA68, (volatile uint32_t *)IPROC_USB2_CLK_CONTROL_ENABLE);
        usb2_clk_cntrl = ehci_iol_r((volatile uint32_t *)IPROC_USB2_CLK_CONTROL_ENABLE);
        printf("USB clk control enable register is: %08x\n", usb2_clk_cntrl);

        ehci_iol_w(0xDD10F3, (volatile uint32_t *)IPROC_USB2_CLK_CONTROL_PLL);
        usb2_clk_enable = ehci_iol_r((volatile uint32_t *)IPROC_USB2_CLK_CONTROL_PLL);
        printf("USB clk enable register is: %08x\n", usb2_clk_enable);

        ehci_iol_w(0x0, (volatile uint32_t *)IPROC_USB2_CLK_CONTROL_ENABLE);
        usb2_clk_cntrl = ehci_iol_r((volatile uint32_t *)IPROC_USB2_CLK_CONTROL_ENABLE);
        printf("USB clk control enable register is: %08x\n", usb2_clk_cntrl);
    }

    usb2_clk_enable = ehci_iol_r((volatile uint32_t *)IPROC_USB2_CLK_CONTROL_PLL);
    debug("Before reset, USB clk enable register is: %08x\n", usb2_clk_enable);

    /* Reset USBH 2.0 core */
    ehci_iol_w(0x1, (volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
    printf("\nusb2h_reset.......\n");
    ehci_iol_w(0x0, (volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
    usb2_reset_state = ehci_iol_r((volatile uint32_t *)IPROC_IDM_USB2_RESET_CONTROL);
    printf("usb2_reset_state is set and now it is: %08x\n", usb2_reset_state);

    usb2_clk_enable = ehci_iol_r((volatile uint32_t *)IPROC_USB2_CLK_CONTROL_PLL);
    debug("After reset, USB clk enable register is: %08x\n", usb2_clk_enable);

#endif /* defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_SABER2) */

    /* map registers */
    *hccr = (struct ehci_hccr *)IPROC_EHCI;
	*hcor = (struct ehci_hcor *)((uint32_t)*hccr +
			HC_LENGTH(ehci_iol_r((volatile uint32_t *)&(*hccr)->cr_capbase)));

    return 0;
}

/*
 * Function to terminate USB host related low level hardware including PHY,
 * clocks, etc.
 *
 * TODO: expand support for more than one host in the future if needed
 */
int ehci_hcd_stop(int index)
{
#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_SABER2))
    uint32_t val;

    val = ehci_iol_r((volatile uint32_t *)ChipcommonG_GP_DATA_OUT);
#if defined(CONFIG_SABER2)
    /* Saber2 is low-active */
    val |= (1 << USBH_VBUS_GPIO);
#else
    val &= ~(1 << USBH_VBUS_GPIO);
#endif
    ehci_iol_w(val, (volatile uint32_t *)ChipcommonG_GP_DATA_OUT);

    val = ehci_iol_r((volatile uint32_t *)ChipcommonG_GP_OUT_EN);
    val |= (1 << USBH_VBUS_GPIO);
    ehci_iol_w(val, (volatile uint32_t *)ChipcommonG_GP_OUT_EN);
#endif
    return 0;
}
