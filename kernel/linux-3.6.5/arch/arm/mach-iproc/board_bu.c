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

#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <asm/hardware/gic.h>

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/i2c/tsc2007.h>
#include <linux/spi/spi.h>
#include <mach/hardware.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/sdio_platform.h>
#include <mach/iproc.h>
#include <asm/io.h>
#include <mach/io_map.h>
#include <mach/reg_utils.h>
#include <linux/pwm.h>
#include <linux/amba/bus.h>
#include <mach/watchdog_platform.h>
#include <linux/amba/pl330.h>

#include "northstar.h"
#include "common.h"

#include <linux/amba/pl022.h>

#ifdef CONFIG_MACH_SB2
#include "include/mach/iproc_regs.h"

struct pl022_config_chip spi_chip_info = {
	.iface = SSP_INTERFACE_MOTOROLA_SPI,	
	.hierarchy = SSP_MASTER,
	/* We can only act as master but SSP_SLAVE is possible in theory */
	/* 0 = drive TX even as slave, 1 = do not drive TX as slave */
	.slave_tx_disable = 0,
	.clk_freq = {	.cpsdvsr = 100,  .scr = 1,	},	
	.com_mode = DMA_TRANSFER,
	.rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
	.tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
	.ctrl_len = SSP_BITS_12,
	.wait_state = SSP_MWIRE_WAIT_ZERO,
	.duplex = SSP_MICROWIRE_CHANNEL_HALF_DUPLEX,
};

struct pl022_config_chip spi1_chip_info = {
    .iface = SSP_INTERFACE_MOTOROLA_SPI,
    .hierarchy = SSP_MASTER,
    /* We can only act as master but SSP_SLAVE is possible in theory */
    /* 0 = drive TX even as slave, 1 = do not drive TX as slave */
    .slave_tx_disable = 0,
    .clk_freq = {   .cpsdvsr = 100,  .scr = 99,  },
    .com_mode = DMA_TRANSFER,
    .rx_lev_trig = SSP_RX_4_OR_MORE_ELEM,
    .tx_lev_trig = SSP_TX_4_OR_MORE_EMPTY_LOC,
    .ctrl_len = SSP_BITS_8,
    .wait_state = SSP_MWIRE_WAIT_ZERO,
    .duplex = SSP_MICROWIRE_CHANNEL_HALF_DUPLEX,
};

static struct pl022_ssp_controller ssp_platform_data[] = {
	{
		/* If you have several SPI buses this varies, we have only bus 0 */
		.bus_id = 2,
		/*
		 * On the APP CPU GPIO 4, 5 and 6 are connected as generic
		 * chip selects for SPI. (Same on U330, U335 and U365.)
		 * TODO: make sure the GPIO driver can select these properly
		 * and do padmuxing accordingly too.
		 */
		.num_chipselect = 1,

	    .enable_dma = 0,

    },
	{
		/* If you have several SPI buses this varies, we have only bus 0 */
		.bus_id = 3,
		/*
		 * On the APP CPU GPIO 4, 5 and 6 are connected as generic
		 * chip selects for SPI. (Same on U330, U335 and U365.)
		 * TODO: make sure the GPIO driver can select these properly
		 * and do padmuxing accordingly too.
		 */
		.num_chipselect = 1,
		.enable_dma = 0,
    },
    {},
};

static struct spi_board_info iproc_spi_devices[] = {
	{
		/* A dummy chip used for loopback tests */
		.modalias       = "spidev",
		/* Really dummy, pass in additional chip config here */
		.platform_data  = NULL,
		/* This defines how the controller shall handle the device */
		.controller_data = &spi_chip_info,
		/* .irq - no external IRQ routed from this device */
		.max_speed_hz   = 500000,
		.bus_num        = 2, /* Only one bus on this chip */
		.chip_select    = 0,
		/* Means SPI_CS_HIGH, change if e.g low CS */
		.mode           = SPI_MODE_3| SPI_NO_CS | SPI_LOOP ,
	},	
	{
		.modalias       = "spi_msr",
		.platform_data  = NULL,
		/* This defines how the controller shall handle the device */
		.controller_data = &spi1_chip_info,
		/* .irq - no external IRQ routed from this device */
		.max_speed_hz   = 1000000,
		.bus_num        = 3, /* Only one bus on this chip */
		.chip_select    = 0,
		/* Means SPI_CS_HIGH, change if e.g low CS */
		.mode           = SPI_MODE_0| SPI_NO_CS,
	},	
    {},
};

static struct resource spi2_resources[] ={
	[0] = {
	    .start = ChipcommonG_SPI2_SSPCR0,
		.end   = ChipcommonG_SPI2_SSPCR0 + 0x1000 - 1,
		.name = "ssp-pl022",
		.flags  = IORESOURCE_MEM,
    },
    [1] = {
		.start	= 112, 
		.end	= 112,
		.flags	= IORESOURCE_IRQ,
    },
	{},
};

static struct platform_device board_pl022_devices[] = {
	[0] = {
		.name = "ssp-pl022",
		.id = 0,
		.dev = {
		    .platform_data = &ssp_platform_data[0],
		    },
		.num_resources = ARRAY_SIZE(spi2_resources),
		.resource = spi2_resources,
    },
    {},
};

void __init iproc_init_dev(void)
{
    /* SB2 uses spi2_resources */
    platform_device_register(&board_pl022_devices[0]);

    /* Register SPI bus */
    spi_register_board_info(iproc_spi_devices, 2 /* ARRAY_SIZE(iproc_spi_devices)*/);
}
#endif /* CONFIG_MACH_SB2 */



extern void request_idm_timeout_interrupts(void);
extern void __init iproc_map_io(void);
extern void __init northstar_timer_init(struct clk *clk_ref);

/* This is the main reference clock 25/50MHz from external crystal */
static struct clk clk_ref = {
    .name           = "Refclk",
    .rate           = 25 * 1000000,   /* run-time override */
    .fixed          = 1,
    .type           = 0,
};

static struct clk_lookup board_clk_lookups[] = {
	{
        .con_id     = "refclk",
        .clk        = &clk_ref,
	}
};


/********************************************************
 Struct definitions for platform resources and devices
********************************************************/
#if defined(CONFIG_IPROC_SD) || defined(CONFIG_IPROC_SD_MODULE)
/* sdio */
static struct sdio_platform_cfg sdio_platform_data = {
	.devtype = SDIO_DEV_TYPE_SDMMC,
};
static struct resource sdio_resources[] = {
	[0] = {
		.start	    = IPROC_SDIO0_REG_BASE,
		.end	    = IPROC_SDIO0_REG_BASE + SZ_4K - 1,
		.flags	    = IORESOURCE_MEM,
	},
	[1] = {
		.start	    = BCM_INT_ID_SDIO2CORE,
		.end	    = BCM_INT_ID_SDIO2CORE,
		.flags	    = IORESOURCE_IRQ,
	}
};
static struct platform_device board_sdio_device = {
	.name           = "bcm-sdio",
	.id             = 0,
	.dev = {
		.platform_data = &sdio_platform_data,
	},
  	.resource       = sdio_resources,
	.num_resources  = ARRAY_SIZE(sdio_resources),
};
#endif /* CONFIG_IPROC_SD || CONFIG_IPROC_SD_MODULE */


#if defined(CONFIG_IPROC_PWM) || defined(CONFIG_IPROC_PWM_MODULE)
static struct resource iproc_pwm_resources = {
	.start	        = IPROC_CCB_PWM_CTL,
	.end	        = IPROC_CCB_PWM_CTL + SZ_4K - 1,
	.flags	        = IORESOURCE_MEM,
};
static struct platform_device board_pwm_device = {
	.name           = "iproc_pwmc",
	.id             = -1,
  	.resource       = &iproc_pwm_resources,
  	.num_resources  = 1,
};
static struct pwm_lookup board_pwm_lookup[] = {
    PWM_LOOKUP("iproc_pwmc", 0,"iproc_pwmc","pwm-0"),
    PWM_LOOKUP("iproc_pwmc", 1,"iproc_pwmc","pwm-1"),        
    PWM_LOOKUP("iproc_pwmc", 2,"iproc_pwmc","pwm-2"),
    PWM_LOOKUP("iproc_pwmc", 3,"iproc_pwmc","pwm-3"),

};
#endif /* CONFIG_IPROC_PWM || CONFIG_IPROC_PWM_MODULE */


#if defined(CONFIG_ASIU_PWM) || defined(CONFIG_ASIU_PWM_MODULE)
static struct resource asiu_pwm_resources[] = {
	[0] = {
		.start	    = ASIU_PWM_CONTROL - ASIU_PWM_CONTROL_BASE,
		.end	    = (ASIU_PWM_CONTROL - ASIU_PWM_CONTROL_BASE) + SZ_4K - 1,
		.flags	    = IORESOURCE_MEM,
	},
	[1] = {
		.start	    = CRMU_GENPLL_CONTROL0,
		.end	    = CRMU_GENPLL_CONTROL0 + 0x100 - 1,
		.flags	    = IORESOURCE_MEM,
	}
};
static struct platform_device board_asiu_pwm_device = {
	.name           = "asiu_pwmc",
	.id             = -1,
  	.resource       = asiu_pwm_resources,
  	.num_resources  = ARRAY_SIZE(asiu_pwm_resources),
};
static struct pwm_lookup board_asiu_pwm_lookup[] = {
    PWM_LOOKUP("asiu_pwmc", 0,"asiu_pwmc","pwm-0"),
    PWM_LOOKUP("asiu_pwmc", 1,"asiu_pwmc","pwm-1"),        
    PWM_LOOKUP("asiu_pwmc", 2,"asiu_pwmc","pwm-2"),
    PWM_LOOKUP("asiu_pwmc", 3,"asiu_pwmc","pwm-3"),
    PWM_LOOKUP("asiu_pwmc", 4,"asiu_pwmc","pwm-4"),
    PWM_LOOKUP("asiu_pwmc", 5,"asiu_pwmc","pwm-5"),
};
#endif /* CONFIG_ASIU_PWM || CONFIG_ASIU_PWM_MODULE */


#if defined(CONFIG_IPROC_WDT) || defined(CONFIG_IPROC_WDT_MODULE)
static struct resource wdt_resources[] = {
	[0] = {
		.start	    = IPROC_CCA_REG_BASE,
		.end	    = IPROC_CCA_REG_BASE + 0x1000 - 1,
		.flags	    = IORESOURCE_MEM,
	},
};

static struct platform_device board_wdt_device = {
	.name           = "iproc_wdt",
	.id             = -1,
  	.resource       = wdt_resources,
	.num_resources  = ARRAY_SIZE(wdt_resources),
};
#endif /* CONFIG_IPROC_WDT || CONFIG_IPROC_WDT_MODULE */


#if defined(CONFIG_IPROC_CCB_TIMER) || defined(CONFIG_IPROC_CCB_TIMER_MODULE)
static struct resource ccb_timer_resources[] = {
	[0] = {
		.start	    = IPROC_CCB_TIMER_INT_START,
		.end	    = IPROC_CCB_TIMER_INT_START + IPROC_CCB_TIMER_INT_COUNT - 1,
		.flags	    = IORESOURCE_IRQ,
	},
	[1] = {
		.start	    = IPROC_CCB_TIMER0_REGS_VA,
		.end	    = IPROC_CCB_TIMER0_REGS_VA + 0x20 - 1,
		.flags	    = IORESOURCE_MEM,
	},
	[2] = {
		.start	    = IPROC_CCB_TIMER1_REGS_VA,
		.end	    = IPROC_CCB_TIMER1_REGS_VA + 0x20 - 1,
		.flags	    = IORESOURCE_MEM,
	},
	[3] = {
		.start	    = IPROC_CCB_TIMER2_REGS_VA,
		.end	    = IPROC_CCB_TIMER2_REGS_VA + 0x20 - 1,
		.flags	    = IORESOURCE_MEM,
	},
	[4] = {
		.start	    = IPROC_CCB_TIMER3_REGS_VA,
		.end	    = IPROC_CCB_TIMER3_REGS_VA + 0x20 - 1,
		.flags	    = IORESOURCE_MEM,
	},
};

static struct platform_device board_timer_device = {
	.name		    = "iproc_ccb_timer",
	.id		        = -1,
  	.resource	    = ccb_timer_resources,
	.num_resources  = ARRAY_SIZE(ccb_timer_resources),
};
#endif /* CONFIG_IPROC_CCB_TIMER || CONFIG_IPROC_CCB_TIMER_MODULE */



#ifdef CONFIG_IPROC_I2C   /* SMBus (I2C/BSC) block */
#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2)
static struct resource smbus_resources[] = {
    [0] = {
        .start      = ChipcommonB_SMBus_Config, /* Defined in socregs_ing_open.h */
        .end        = ChipcommonB_SMBus_Config + SZ_4K - 1,
        .flags      = IORESOURCE_MEM,
    },
    [1] = {
        .start      = BCM_INT_ID_CCB_SMBUS0, 
        .end        = BCM_INT_ID_CCB_SMBUS0,
        .flags      = IORESOURCE_IRQ,
    }
};
static struct resource smbus_resources1[] = {
    [0] = {
        .start      = ChipcommonB_SMBus1_SMBus_Config, /* Defined in socregs_ing_open.h */
        .end        = ChipcommonB_SMBus1_SMBus_Config + SZ_4K - 1,
        .flags      = IORESOURCE_MEM
    },
    [1] = {
        .start      = BCM_INT_ID_CCB_SMBUS1, /* Defined in irqs.h */
        .end        = BCM_INT_ID_CCB_SMBUS1,
        .flags      = IORESOURCE_IRQ,
    }
};
#elif (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3)) \
	
static struct resource smbus_resources[] = {
    [0] = {
        .start      = ChipcommonG_SMBus0_SMBus_Config,
        .end        = ChipcommonG_SMBus0_SMBus_Config + SZ_4K - 1,
        .flags      = IORESOURCE_MEM,
    },
    [1] = {
        .start      = BCM_INT_ID_CCG_SMBUS0, 
        .end        = BCM_INT_ID_CCG_SMBUS0,
        .flags      = IORESOURCE_IRQ,
    }
};
#if defined(CONFIG_MACH_SB2)
static struct resource smbus_resources1[] = {
    [0] = {
        .start      = ChipcommonG_SMBus1_SMBus_Config,
        .end        = ChipcommonG_SMBus1_SMBus_Config + SZ_4K - 1,
        .flags      = IORESOURCE_MEM
    },
    [1] = {
        .start      = BCM_INT_ID_CCG_SMBUS1, 
        .end        = BCM_INT_ID_CCG_SMBUS1,
        .flags      = IORESOURCE_IRQ,
    }
};
#endif /* defined(CONFIG_MACH_SB2) */
#endif /* (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2) || defined(CONFIG_MACH_HR3)) */

static struct platform_device board_smbus_device = {
    .name           = "iproc-smb",
    .id             = 0,
    .dev = {
        .platform_data  = NULL, /* Can be defined, if reqd */
    },
    .resource       = smbus_resources,
    .num_resources  = ARRAY_SIZE(smbus_resources),
};

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) || defined(CONFIG_MACH_SB2))
static struct platform_device board_smbus_device1 = {
    .name           = "iproc-smb",
    .id             = 1,
    .dev = {
        .platform_data  = NULL, /* Can be defined, if reqd */
    },
    .num_resources  = ARRAY_SIZE(smbus_resources1),
    .resource       = smbus_resources1,
};
#endif /* (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) || defined(CONFIG_MACH_SB2)) */
#endif /* CONFIG_IPROC_I2C */



#ifdef CONFIG_IPROC_USB3H
static struct resource bcm_xhci_resources[] = {
    [0] = {
        .start      = IPROC_USB30_REG_BASE,
        .end        = IPROC_USB30_REG_BASE + SZ_4K - 1,
        .flags      = IORESOURCE_MEM,
    },
    [1] = {
        .start      = BCM_INT_ID_USB3H2CORE_USB2_INT0,
        .end        = BCM_INT_ID_USB3H2CORE_USB2_INT0,
        .flags      = IORESOURCE_IRQ,
    },
};
static u64 xhci_dmamask = DMA_BIT_MASK(32);
static struct platform_device bcm_xhci_device = {
	.name           = "bcm-xhci",
	.id             = 0,
	.dev =	{
		 .dma_mask          = &xhci_dmamask,
		 .coherent_dma_mask = DMA_BIT_MASK(32),
	},
  	.resource       = bcm_xhci_resources,
	.num_resources  = ARRAY_SIZE(bcm_xhci_resources),
};
#endif /* CONFIG_IPROC_USB3H */


#ifdef CONFIG_USB_EHCI_BCM
static struct resource usbh_ehci_resource[] = {
	[0] = {
        .start      = IPROC_USB20_EHCI_REG_BASE,
        .end        = IPROC_USB20_EHCI_REG_BASE + IPROC_USB20_EHCI_REG_SIZE,
        .flags      = IORESOURCE_MEM,
	},
	[1] = {
		.start      = BCM_INT_ID_USB2H2CORE_USB2_INT,
		.end        = BCM_INT_ID_USB2H2CORE_USB2_INT,
		.flags      = IORESOURCE_IRQ,
	},
};

static u64 ehci_dmamask = DMA_BIT_MASK(32);
static struct platform_device usbh_ehci_device =
{
	.name           = "bcm-ehci",
	.id             = 0,
	.dev = {
		.dma_mask           = &ehci_dmamask,
		.coherent_dma_mask  = DMA_BIT_MASK(32),
	},
	.resource       = usbh_ehci_resource,
	.num_resources  = ARRAY_SIZE(usbh_ehci_resource),
};
#endif /* CONFIG_USB_EHCI_BCM */


#ifdef CONFIG_USB_OHCI_BCM
static struct resource usbh_ohci_resource[] = {
	[0] = {
        .start      = IPROC_USB20_OHCI_REG_BASE,
        .end        = IPROC_USB20_OHCI_REG_BASE + IPROC_USB20_OHCI_REG_SIZE,
		.flags      = IORESOURCE_MEM,
	},
	[1] = {
		.start      = BCM_INT_ID_USB2H2CORE_USB2_INT,
		.end        = BCM_INT_ID_USB2H2CORE_USB2_INT,
		.flags      = IORESOURCE_IRQ,
	},
};

static u64 ohci_dmamask = DMA_BIT_MASK(32);
static struct platform_device usbh_ohci_device =
{
	.name           = "bcm-ohci",
	.id             = 0,
	.dev = {
		.dma_mask           = &ohci_dmamask,
		.coherent_dma_mask  = DMA_BIT_MASK(32),
	},
	.resource       = usbh_ohci_resource,
	.num_resources  = ARRAY_SIZE(usbh_ohci_resource),
};
#endif /* CONFIG_USB_OHCI_BCM */


#if defined(CONFIG_BRCM_CE_28nm_OTP) || defined(CONFIG_BRCM_CE_28nm_OTP_MODULE)
static struct resource bcm_ce_28nm_otp_resources[] = {
    [0] = {
        .start      = CHIP_OTPC_REG_BASE,
        .end        = CHIP_OTPC_REG_BASE + SZ_4K - 1,
        .flags      = IORESOURCE_MEM,
    },
};
static struct platform_device bcm_ce_28nm_otp_device = {
	.name		    =	"brcm-ce-28nm-otp",
	.id		        =	-1,
  	.resource	    = bcm_ce_28nm_otp_resources,
	.num_resources  = ARRAY_SIZE(bcm_ce_28nm_otp_resources),
};
#endif /* defined(CONFIG_BRCM_CE_28nm_OTP) || defined(CONFIG_BRCM_CE_28nm_OTP_MODULE) */


#ifdef CONFIG_DMAC_PL330
#include "../../../../bcmdrivers/dma/pl330-pdata.h"
static struct iproc_pl330_data iproc_pl330_pdata =	{
	.dmac_ns_base       = IPROC_DMAC_REG_VA,    /* Non Secure DMAC virtual base address */
	.dmac_s_base        = IPROC_DMAC_REG_VA,    /* Secure DMAC virtual base address */
	.num_pl330_chans    = 8,                    /* # of PL330 dmac channels 'configurable' */
	.irq_base           = BCM_INT_ID_DMAC,      /* irq number to use */
	.irq_line_count     = 16,                   /* # of PL330 Interrupt lines connected to GIC */
};
static struct platform_device pl330_dmac_device = {
	.name           = "iproc-dmac-pl330",
	.id             = 0,
	.dev = {
		.platform_data      = &iproc_pl330_pdata,
		.coherent_dma_mask  = DMA_BIT_MASK(64),
	},
};
#endif /* CONFIG_DMAC_PL330 */


#ifdef CONFIG_PL330_DMA
static AMBA_AHB_DEVICE(pl330_driver, "dma-pl330", 0x00041330, 
                       DMAC_pl330_DS, {(BCM_INT_ID_DMAC - 1)}, NULL);
#endif /* CONFIG_PL330_DMA */


#if defined(CONFIG_IPROC_MDIO) || defined(CONFIG_IPROC_MDIO_MODULE)
#include "../../../../../bcmdrivers/mdio/iproc_mdio.h"
#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
static struct iproc_mdiobus_data iproc_mdiobus_data[] = {
	[0] = {
		.phybus_num     = 0,
		.phybus_type    = IPROC_MDIOBUS_TYPE_INTERNAL,
		.logbus_num     = 0,
		.logbus_type    = IPROC_MDIOBUS_TYPE_INTERNAL,
	},
	[1] = {
		.phybus_num     = 2,
		.phybus_type    = IPROC_MDIOBUS_TYPE_EXTERNAL,
		.logbus_num     = 0,
		.logbus_type    = IPROC_MDIOBUS_TYPE_EXTERNAL,
	},
};
#else
static struct iproc_mdiobus_data iproc_mdiobus_data[] = {
	[0] = {
		.phybus_num     = 0,
		.phybus_type    = IPROC_MDIOBUS_TYPE_INTERNAL,
		.logbus_num     = 0,
		.logbus_type    = IPROC_MDIOBUS_TYPE_INTERNAL,
	},
	[1] = {
		.phybus_num     = 0,
		.phybus_type    = IPROC_MDIOBUS_TYPE_EXTERNAL,
		.logbus_num     = 0,
		.logbus_type    = IPROC_MDIOBUS_TYPE_EXTERNAL,
	},
};
#endif /* defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3) */

#if defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_HR3)
static struct platform_device board_mdiobus_int0_device = {
	.name       = "iproc_ccg_mdio",
	.id         = 0,
	.dev = {
		.platform_data  = &iproc_mdiobus_data[0],
	},
};
static struct platform_device board_mdiobus_ext0_device = {
	.name       = "iproc_cmicd_mdio",
	.id         = 0,
	.dev = {
		.platform_data  = &iproc_mdiobus_data[1],
	},
};
#elif defined(CONFIG_MACH_SB2)
static struct platform_device board_mdiobus_int0_device = {
	.name       = "iproc_ccg_mdio",
	.id         = 0,
	.dev = {
		.platform_data  = &iproc_mdiobus_data[0],
	},
};
static struct platform_device board_mdiobus_ext0_device = {
	.name       = "iproc_ccg_mdio",
	.id         = 1,
	.dev = {
		.platform_data  = &iproc_mdiobus_data[1],
	},
};
#else
static struct platform_device board_mdiobus_int0_device = {
	.name       = "iproc_ccb_mdio",
	.id         = 0,
	.dev = {
		.platform_data = &iproc_mdiobus_data[0],
	},
};
static struct platform_device board_mdiobus_ext0_device = {
	.name       = "iproc_ccb_mdio",
	.id         = 1,
	.dev = {
		.platform_data = &iproc_mdiobus_data[1],
	},
};
#endif
#endif /* defined(CONFIG_IPROC_MDIO) || defined(CONFIG_IPROC_MDIO_MODULE) */


#if defined(CONFIG_ET)
static struct resource iproc_gmac0_resources[] = {
    [0] = {
        .flags          = IORESOURCE_IRQ,
        .start          = IPROC_GMAC0_INT,
    },
    [1] = {
        .flags          = IORESOURCE_MEM,
        .start          = IPROC_GMAC0_REG_BASE,
        .end            = IPROC_GMAC0_REG_BASE + 0xbff,
    },
};
static struct resource iproc_gmac1_resources[] = {
    [0] = {
        .flags          = IORESOURCE_IRQ,
        .start          = IPROC_GMAC1_INT,
    },
    [1] = {
        .flags          = IORESOURCE_MEM,
        .start          = IPROC_GMAC1_REG_BASE,
        .end            = IPROC_GMAC1_REG_BASE + 0xbff,
    },
};
static struct resource iproc_gmac2_resources[] = {
    [0] = {
        .flags          = IORESOURCE_IRQ,
        .start          = IPROC_GMAC2_INT,
    },
    [1] = {
        .flags          = IORESOURCE_MEM,
        .start          = IPROC_GMAC2_REG_BASE,
        .end            = IPROC_GMAC2_REG_BASE + 0xbff,
    },
};
static struct resource iproc_gmac3_resources[] = {
    [0] = {
        .flags          = IORESOURCE_IRQ,
        .start          = IPROC_GMAC3_INT,
    },
    [1] = {
        .flags          = IORESOURCE_MEM,
        .start          = IPROC_GMAC3_REG_BASE,
        .end            = IPROC_GMAC3_REG_BASE + 0xbff,
    },
};

static u64 gmac_dmamask = DMA_BIT_MASK(32);
static struct platform_device board_gmac_device[IPROC_MAX_GMAC_CORES] = {
    {
        .name           = "bcm-gmac0",
        .id             = 0,
        .dev = {
            .init_name          = "bcm-gmac0",
            .dma_mask           = &gmac_dmamask,
            .coherent_dma_mask  = DMA_BIT_MASK(32),
        },
        .resource       = iproc_gmac0_resources,
        .num_resources  = ARRAY_SIZE(iproc_gmac0_resources),
    },
    {
        .name           = "bcm-gmac1",
        .id             = 0,
        .dev = {
            .init_name          = "bcm-gmac1",
            .dma_mask           = &gmac_dmamask,
            .coherent_dma_mask  = DMA_BIT_MASK(32),
        },
        .resource       = iproc_gmac1_resources,
        .num_resources  = ARRAY_SIZE(iproc_gmac1_resources),
    },
    {
        .name           = "bcm-gmac2",
        .id             = 0,
        .dev = {
            .init_name          = "bcm-gmac2",
            .dma_mask           = &gmac_dmamask,
            .coherent_dma_mask  = DMA_BIT_MASK(32),
        },
        .resource       = iproc_gmac2_resources,
        .num_resources  = ARRAY_SIZE(iproc_gmac2_resources),
    },
    {
        .name           = "bcm-gmac3",
        .id             = 0,
        .dev = {
            .init_name          = "bcm-gmac3",
            .dma_mask           = &gmac_dmamask,
            .coherent_dma_mask  = DMA_BIT_MASK(32),
        },
        .resource       = iproc_gmac3_resources,
        .num_resources  = ARRAY_SIZE(iproc_gmac3_resources),
    }
};
#endif /* defined(CONFIG_ET) */


static struct platform_device *board_devices[] __initdata = {
#ifdef CONFIG_IPROC_I2C
    &board_smbus_device,
#if defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2) || defined(CONFIG_MACH_SB2)
    &board_smbus_device1,
#endif
#endif /* CONFIG_IPROC_I2C */
#if defined(CONFIG_IPROC_MDIO) || defined(CONFIG_IPROC_MDIO_MODULE)
	&board_mdiobus_int0_device,
	&board_mdiobus_ext0_device,
#endif
#if defined(CONFIG_IPROC_CCB_TIMER) || defined(CONFIG_IPROC_CCB_TIMER_MODULE)
    &board_timer_device,
#endif
#if defined(CONFIG_IPROC_WDT) || defined(CONFIG_IPROC_WDT_MODULE)
    &board_wdt_device,
#endif
#if defined(CONFIG_IPROC_PWM) || defined(CONFIG_IPROC_PWM_MODULE)
	&board_pwm_device,
#endif
#if defined(CONFIG_ASIU_PWM) || defined(CONFIG_ASIU_PWM_MODULE)
	&board_asiu_pwm_device,
#endif
#ifdef CONFIG_IPROC_USB3H
	&bcm_xhci_device,
#endif 
#ifdef CONFIG_USB_EHCI_BCM
	&usbh_ehci_device,
#endif
#ifdef CONFIG_USB_OHCI_BCM
	&usbh_ohci_device,
#endif
#ifdef CONFIG_DMAC_PL330
	&pl330_dmac_device,
#endif
#if defined(CONFIG_BRCM_CE_28nm_OTP) || defined(CONFIG_BRCM_CE_28nm_OTP_MODULE)
	&bcm_ce_28nm_otp_device,
#endif
};

/******************************************************
*******************************************************/
#if defined(CONFIG_IPROC_SP805_WDT) || defined(CONFIG_IPROC_SP805_WDT_MODULE)
static struct wdt_platform_data iproc_wdt_data = {
	.rsts = {
		.reg = IPROC_SP805_WDT_BOOTSTATUS,
		.bit = IPROC_SP805_WDT_BOOTSTATUS_BIT,
		.needs_clear = 1,
	},
};
static struct amba_device iproc_sp805_wdt_device = {
	.dev = {
		.init_name = "sp805-wdt",
		.id = 0,
		.platform_data = &iproc_wdt_data,
	},
	.res = DEFINE_RES_MEM(IPROC_SP805_WDT_REG_BASE, SZ_4K),
	.periphid = 0x00141805,
};
#endif /* defined(CONFIG_IPROC_SP805_WDT) || defined(CONFIG_IPROC_SP805_WDT_MODULE) */

static struct amba_device *amba_devs[] __initdata = {
#if defined(CONFIG_IPROC_SP805_WDT) || defined(CONFIG_IPROC_SP805_WDT_MODULE)
	&iproc_sp805_wdt_device,
#endif
};

/******************************************************
 SPI device info of GSIO(SPI) interface 
*******************************************************/
static struct spi_board_info bcm5301x_spi_device[] = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.controller_data = NULL,
		.max_speed_hz = 2 * 1000 * 1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
	},
};

/******************************************************
*******************************************************/
void __init board_timer_init(void);
struct sys_timer board_timer = {
	.init   = board_timer_init,
};


/******************************************************************************
******************************************************************************/
#if defined(CONFIG_IPROC_SD) || defined(CONFIG_IPROC_SD_MODULE)
static void setup_sdio(void)
{
    void __iomem *idm_base;
    
    idm_base = (void __iomem *)IPROC_IDM_REGISTER_VA;
    writel_relaxed(0, idm_base + IPROC_SDIO_IDM_RESET_CONTROL);
    
    platform_device_register(&board_sdio_device);
}
#endif /* CONFIG_IPROC_SD || CONFIG_IPROC_SD_MODULE */

static void __init board_add_devices(void)
{
	int idx;

	platform_add_devices(board_devices, ARRAY_SIZE(board_devices));
	
    /* Register platform devices for AMAC */
#if defined(CONFIG_ET)
    for (idx = 0; idx < IPROC_NUM_GMACS; idx++) {
        platform_device_register(&board_gmac_device[idx]);
    }
#endif /* defined(CONFIG_ET) */

	for (idx = 0; idx < ARRAY_SIZE(amba_devs); idx++) {
		amba_device_register(amba_devs[idx], &iomem_resource);
	}
}

void __init board_map_io(void)
{
	/*
	 * Install clock sources in the lookup table.
	 */
	clkdev_add_table(board_clk_lookups, ARRAY_SIZE(board_clk_lookups));

	/* Map machine specific iodesc here */
	iproc_map_io();
}

void __init iproc_init_early(void)
{
	/*
	 * SDK allocates coherent buffers from atomic
	 * context. Increase size of atomic coherent pool to make sure such
	 * the allocations won't fail.
	 */
#ifdef CONFIG_CMA 
	init_dma_coherent_pool_size(SZ_1M * 16);
#endif
}

void __init board_timer_init(void)
{
	northstar_timer_init(&clk_ref);
}

void __init board_init(void)
{
	printk(KERN_DEBUG "board_init: Enter\n");

	/*
	 * Add common platform devices that do not have board dependent HW
	 * configurations
	 */
	board_add_common_devices(&clk_ref);
	
	/* register IDM timeout interrupt handler */
	request_idm_timeout_interrupts();

    board_add_devices();

#if defined(CONFIG_IPROC_SD) || defined(CONFIG_IPROC_SD_MODULE)   
    setup_sdio();
#endif

#if defined(CONFIG_IPROC_PWM) || defined(CONFIG_IPROC_PWM_MODULE)
    writel_relaxed(0xf, IPROC_CCB_GPIO_REG_VA + IPROC_GPIO_CCB_AUX_SEL);
    pwm_add_table(board_pwm_lookup, ARRAY_SIZE(board_pwm_lookup));
#endif  

#if defined(CONFIG_ASIU_PWM) || defined(CONFIG_ASIU_PWM_MODULE)
    pwm_add_table(board_asiu_pwm_lookup, ARRAY_SIZE(board_asiu_pwm_lookup));
#endif

	/* Register SPI device info */
	spi_register_board_info(bcm5301x_spi_device, 
	ARRAY_SIZE(bcm5301x_spi_device)); 

	printk(KERN_DEBUG "board_init: Leave\n");
}


MACHINE_START(IPROC, "Broadcom iProc")
    /* Used micro9 as a reference.  Micro9 removed these two fields,
     * and replaced them with a call to ep93xx_map_io(), which in turn
     * calls iotable_init().  Northstar appears to have an equivalent
     * init (refer to northstar_io_desc[] array, in io_map.c
     */
    .map_io = board_map_io,
    .init_early	= iproc_init_early,
    .init_irq = iproc_init_irq,
    .handle_irq = gic_handle_irq,
    .timer = &board_timer,
    .init_machine = board_init,
MACHINE_END
