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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/io.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/clkdev.h>
//#include <mach/clkdev.h>
#include <mach/hardware.h>
#include <mach/iproc.h>
#include <mach/irqs.h>

#define IPROC_UART0_PA		IPROC_CCA_UART0_REG_BASE
#define IPROC_UART1_PA		IPROC_CCA_UART1_REG_BASE
#define IPROC_UART2_PA      IPROC_CCA_UART2_REG_BASE
#define IPROC_UART3_PA		IPROC_CCA_UART3_REG_BASE
#define IPROC_UART4_PA      IPROC_CCA_UART4_REG_BASE
#define IPROC_CCA_UART_CLK_PA	IPROC_CCA_UART_CLK_REG_BASE
#define IPROC_CCA_CCAP_PA	IPROC_CCA_CORE_CAP_REG_BASE
#define IPROC_CCA_CCTL_PA	IPROC_CCA_CORE_CTL_REG_BASE
#define IPROC_CCA_INTMASK_PA	IPROC_CCA_INTMASK_REG_BASE

#define IPROC_UART0_VA		HW_IO_PHYS_TO_VIRT(IPROC_UART0_PA)
#define IPROC_UART1_VA		HW_IO_PHYS_TO_VIRT(IPROC_UART1_PA)
#define IPROC_UART2_VA      HW_IO_PHYS_TO_VIRT(IPROC_UART2_PA)
#define IPROC_UART3_VA		HW_IO_PHYS_TO_VIRT(IPROC_UART3_PA)
#define IPROC_UART4_VA      HW_IO_PHYS_TO_VIRT(IPROC_UART4_PA)
#define IPROC_CCA_UART_CLK_VA	HW_IO_PHYS_TO_VIRT(IPROC_CCA_UART_CLK_PA)
#define IPROC_CCA_CCAP_VA	HW_IO_PHYS_TO_VIRT(IPROC_CCA_CCAP_PA)
#define IPROC_CCA_CCTL_VA	HW_IO_PHYS_TO_VIRT(IPROC_CCA_CCTL_PA)
#define IPROC_CCA_INTMASK_VA	HW_IO_PHYS_TO_VIRT(IPROC_CCA_INTMASK_PA)

#if (defined(CONFIG_MACH_HX4) || defined(CONFIG_MACH_KT2))
#define IRQ_IPROC_UART0  123
#elif defined(CONFIG_MACH_HR2)
#define IRQ_IPROC_UART0  123
#elif (defined(CONFIG_MACH_GH) || defined(CONFIG_MACH_SB2))
#define IRQ_IPROC_UART0  105
#elif defined(CONFIG_MACH_HR3)
#define IRQ_IPROC_UART0  93
#else
#error "No valid UART IRQ selected"
#endif

#if defined(CONFIG_MACH_HR2) && defined(CONFIG_MACH_IPROC_EMULATION)
#define IPROC_UART_CLK (76800)
#elif defined(CONFIG_MACH_KT2) && defined(CONFIG_MACH_IPROC_EMULATION)
#define IPROC_UART_CLK (65800)
#elif defined(CONFIG_MACH_IPROC_P7) && defined(CONFIG_MACH_IPROC_EMULATION)
#define IPROC_UART_CLK (76219)
#elif defined(CONFIG_MACH_IPROC_P7)
#define IPROC_UART_CLK (100000000)
#else
#define IPROC_UART_CLK (62500000)
#endif

#ifdef CONFIG_MACH_IPROC_P7
#define IPROC_8250PORT(name)						\
{									\
	.membase    = (void __iomem *)(IPROC_##name##_VA),		\
	.mapbase    = (resource_size_t)(IPROC_##name##_PA),		\
	.irq        = IRQ_IPROC_UART0,					\
	.uartclk    = IPROC_UART_CLK,					\
	.regshift   = 2,						\
	.iotype     = UPIO_MEM32,					\
	.type       = PORT_16550A,					\
	.flags      = UPF_FIXED_TYPE | UPF_SHARE_IRQ,			\
	.private_data = (void __iomem *)((IPROC_##name##_VA) + 0x00),	\
}
#else
#define IPROC_8250PORT(name)						\
{									\
	.membase    = (void __iomem *)(IPROC_##name##_VA),		\
	.mapbase    = (resource_size_t)(IPROC_##name##_PA),		\
	.irq        = IRQ_IPROC_UART0,					\
	.uartclk    = IPROC_UART_CLK,					\
	.regshift   = 0,						\
	.iotype     = UPIO_MEM,						\
	.type       = PORT_16550A,					\
	.flags      = UPF_FIXED_TYPE | UPF_SHARE_IRQ,			\
	.private_data = (void __iomem *)((IPROC_##name##_VA) + 0x00),	\
}
#endif /* CONFIG_MACH_IPROC_P7 */

static struct plat_serial8250_port uart_data[] = {
#if (defined(CONFIG_MACH_HR2) && !defined(CONFIG_MACH_IPROC_EMULATION))
	IPROC_8250PORT(UART1), /* Use UART2 as ttys0 */
	IPROC_8250PORT(UART0),
#else
	IPROC_8250PORT(UART0),
	IPROC_8250PORT(UART1),
#endif
	{ .flags = 0, },
};

static struct platform_device board_serial_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
	.platform_data = uart_data,
	},
};

#if defined(CONFIG_MPCORE_WATCHDOG)
static struct resource wdt_device_resource[] = {
	[0] = {
	.start = IPROC_PERIPH_PVT_TIM_REG_BASE,
	.end   = IPROC_PERIPH_PVT_TIM_REG_BASE + 0x34,
	.flags = IORESOURCE_MEM,
	},
	[1] = {
	.start = BCM_INT_ID_CCB_TIM1_INT2,
	.end   = BCM_INT_ID_CCB_TIM1_INT2,
	.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device wdt_device =
{
	.name          = "mpcore_wdt",
	.id            = -1,
	.resource      = wdt_device_resource,
	.num_resources = ARRAY_SIZE(wdt_device_resource),
};
#endif

/* Common devices among all Northstar boards */
//static struct platform_device *board_common_plat_devices[] __initdata = {
//	&board_serial_device,
//#if defined(CONFIG_MPCORE_WATCHDOG)
//	&wdt_device,
//#endif
//};

void __init iproc_config_boot_console(struct clk *ref_clk)
{
	u32 i;
	u32 clk_rate = IPROC_UART_CLK;
	struct clk * clk = NULL ;

#ifndef CONFIG_MACH_IPROC_P7
	u8 uart_clk_sel;
	u8 uart_clk_ovr;
	u16 uart_clk_div;

	/* Get Core Capabilities Register, and extract
	   UART Clock Select from bits 4..3 which show
	   the clock source.  Values are:
	    	0 = 25Mhz clock input
	    	1 = Internal clock
	    	2 = reserved
	    	3 = reserved
	*/
	uart_clk_sel = (readl(IPROC_CCA_CCAP_VA) >> 3) & 0x3;

	/* Get UARTClkOvr from bit 0 of the Core Control Register
	   If set, this bit indicates that the UART clock is supplied
	   from the internal ALP (APB) clock.  If clear then it indicates
	   that APBX_IDM_IO_CONTROL_DIRECT register bit UARTClkSel controls
	   the clock source
	*/
	uart_clk_ovr = readl(IPROC_CCA_CCTL_VA) & 0x01;

	/* uart_clk_div: ChipcommonA_ClkDiv bits 0..7 */
	uart_clk_div =  0xff & readl(IPROC_CCA_UART_CLK_VA);
	if( uart_clk_div == 0 )
		uart_clk_div = 0x100 ;

	if( uart_clk_sel == 0 ) {
		/* uart_clk_sel = 0 -> external reference clock source */
		clk = ref_clk ;
		BUG_ON( !clk );
		clk_rate = clk_get_rate(clk);
	} else if( uart_clk_sel == 1 ) {
		/* uart_clk_sel = 1 -> Internal clock optionally divided */
		clk = clk_get_sys( "iproc_slow", "c_clk125" );
		BUG_ON( !clk );
#if defined(CONFIG_MACH_IPROC_EMULATION)
		clk_rate = IPROC_UART_CLK ;
#else
		clk_rate = clk_get_rate(clk) ;
#endif


		if( ! uart_clk_ovr )
			clk_rate /= uart_clk_div;
	}

	printk( KERN_INFO "Sel=%d Ovr=%d Div=%d\n", uart_clk_sel, uart_clk_ovr, uart_clk_div );

#else /* CONFIG_MACH_IPROC_P7 */
#if !defined(CONFIG_MACH_IPROC_EMULATION)

	clk = clk_get_sys("iproc_slow", "c_clk125");
    if (clk != NULL) {
		clk_rate = clk_get_rate(clk);
    }

#endif /* !defined(CONFIG_MACH_IPROC_EMULATION) */
#endif /* CONFIG_MACH_IPROC_P7 */

	printk( KERN_INFO "UART clock rate %u\n", clk_rate );

	/* fixup UART port structure */
	for(i = 0; i < ARRAY_SIZE(uart_data); i++ ) {
		if( uart_data[i].flags == 0 )
			break;
		if( uart_data[i].irq == 0 )
			uart_data[i].flags |= UPF_AUTO_IRQ;

		uart_data[i].uartclk = clk_rate ;
	}

	/* Install SoC devices in the system: uarts */
	platform_device_register(&board_serial_device);

#ifndef CONFIG_MACH_IPROC_P7
	/* Enable UART interrupt in ChipcommonA */
	i = readl(IPROC_CCA_INTMASK_VA);
	i |= 1 << 6;
	writel(i, IPROC_CCA_INTMASK_VA);
#endif /* !CONFIG_MACH_IPROC_P7 */
}

void __init board_add_common_devices(struct clk *ref_clk)
{
	/*
	 * Configure boot console
	 */
	iproc_config_boot_console(ref_clk);
}
