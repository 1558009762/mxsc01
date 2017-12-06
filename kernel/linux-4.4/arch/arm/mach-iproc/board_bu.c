/*
 * $Copyright Open Broadcom Corporation$
 */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>

#include <linux/irqchip/arm-gic.h>
#include <mach/iproc_regs.h>
#include <mach/smp.h>
#include <asm/mach/map.h>

#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <mach/hardware.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <mach/reg_utils.h>
#include <linux/pwm.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>
#include <asm/smp_scu.h>

#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/clocksource.h>
#include <linux/clk-provider.h>

extern void iproc_enable_data_prefetch_aborts(void);
extern void request_idm_timeout_interrupts(struct device_node *);

#define IPROC_DMU_PCU_COMPATIBLE "brcm,iproc-dmu-pcu"
#define IPROC_WRAP_CTRL_COMPATIBLE "brcm,iproc-wrap-ctrl"
#define IPROC_IDM_COMPATIBLE "brcm,iproc-idm"
#define MAX_IDM_NUM	2
static void __iomem *iproc_dmu_pcu_base=NULL;
static void __iomem *iproc_wrap_ctrl_base=NULL;
static void __iomem *iproc_idm_base[MAX_IDM_NUM]={NULL};
static void __iomem *iproc_idm_base_phys[MAX_IDM_NUM]={NULL};

#ifdef CONFIG_PL330_DMA
static void __iomem *iproc_dmac_idm_base=NULL;
#if (defined (CONFIG_MACH_SB2) || defined (CONFIG_MACH_HR3))
#define IPROC_DMAC_IDM_RESET_OFFSET 0xf800
#elif (defined (CONFIG_MACH_HX4) || defined (CONFIG_MACH_KT2) || defined (CONFIG_MACH_HR2) || defined (CONFIG_MACH_GH))
#define IPROC_DMAC_IDM_RESET_OFFSET 0x14800
#endif
#endif /* CONFIG_PL330_DMA */

void __iomem *get_iproc_dmu_pcu_base(void) {
	return iproc_dmu_pcu_base;
}

void __iomem *get_iproc_wrap_ctrl_base(void) {
	return iproc_wrap_ctrl_base;
}

void __iomem *get_iproc_idm_base(int index) {
	return iproc_idm_base[index];
}

void __iomem *get_iproc_idm_base_phys(int index) {
	return iproc_idm_base_phys[index];
}


void __init bcm_iproc_init_early(void)
{   
	/*
	 * SDK allocates coherent buffers from atomic context.
	 * Increase size of atomic coherent pool to make sure such
	 * the allocations won't fail.
	 */
#ifdef CONFIG_DMA_CMA
	/*can be overrided by "coherent_pool" in bootargs */
	init_dma_coherent_pool_size(SZ_1M * 16);
#endif
}

static const char const *bcm_iproc_dt_compat[] = {
	"brcm,helix4",
	"brcm,katana2",
	"brcm,hurricane2",
	"brcm,greyhound",
	"brcm,saber2",
	"brcm,hurricane3",
	"brcm,xgs-iproc",
	NULL,
};

static void __init bcm_iproc_timer_init(void)
{
	iproc_enable_data_prefetch_aborts();
	/* Initialize all clocks declared in device tree */
	of_clk_init(NULL);

	clocksource_probe();
}

static void __init bcm_iproc_init(void)
{	
	struct device_node *np;
	struct platform_device *pdev=NULL;	
	struct resource *res_mem;

	/* l2x0_of_init(aux_value,aux_mask) update aux_value & aux_mask 
	 * in cache-l2x0.c::pl310_of_setup based on DT.
	 */
	l2x0_of_init(0,~0UL); 
   
	/* Get DMU PCU base addr */
	np = of_find_compatible_node(NULL, NULL, IPROC_DMU_PCU_COMPATIBLE);
	if (!np) {
		printk(KERN_ERR "No dmu pcu node defined in DT\n");
		return;
	}	
	iproc_dmu_pcu_base = of_iomap(np, 0);
	if (!iproc_dmu_pcu_base) {
		printk(KERN_ERR "DMU PCU ioremap eror\n");
		return;
	}
	
	/* Get WRAP CTRL base addr */
	np = of_find_compatible_node(NULL, NULL, IPROC_WRAP_CTRL_COMPATIBLE);
	if (!np) {
		printk(KERN_INFO "No wrap ctrl node defined in DT\n");
		return;
	}	
	iproc_wrap_ctrl_base = of_iomap(np, 0);
	if (!iproc_wrap_ctrl_base) {
		printk(KERN_ERR "Wrap ctrl ioremap eror\n");
		return;
	}
	
	/* Get IDM base addr */
	np = of_find_compatible_node(NULL, NULL, IPROC_IDM_COMPATIBLE);
	if (!np) {
		printk(KERN_INFO "No IDM node defined in DT\n");
		return;
	}	
	iproc_idm_base[0] = of_iomap(np, 0);
	if (!iproc_idm_base[0]) {
		printk(KERN_ERR "IDM ioremap eror\n");
		return;
	}
	/* GH/GH2 IDM needs this */
#if (defined (CONFIG_MACH_GH) || defined (CONFIG_MACH_SB2) || defined (CONFIG_MACH_GH2))	
	iproc_idm_base[1] = of_iomap(np, 1);
	if (!iproc_idm_base[1]) {
		printk(KERN_ERR "IDM 1 ioremap eror\n");
		return;
	}
#endif	
	   
#ifdef CONFIG_PL330_DMA
	/* Need to de-assert reset of DMAC before of_platform_populate */ 
	iproc_dmac_idm_base = get_iproc_idm_base(0) + IPROC_DMAC_IDM_RESET_OFFSET;
	writel_relaxed(readl_relaxed(iproc_dmac_idm_base) & 0xFFFFFFFE, iproc_dmac_idm_base);
#endif /* CONFIG_PL330_DMA */
     
    
	/* Populate platform devices based on DTS */    
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
    
    
	/* To get IDM phys addr */
	np = of_find_compatible_node(NULL, NULL, IPROC_IDM_COMPATIBLE);
	if (!np) {
		printk(KERN_INFO "No IDM node defined in DT\n");
		return;
	}	
	pdev = of_find_device_by_node(np);
	if (!pdev) {
		printk(KERN_INFO "No IDM platform device found\n");
		return;
	}	 	
	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iproc_idm_base_phys[0] = (void __iomem *)res_mem->start;
#if (defined (CONFIG_MACH_GH) || defined (CONFIG_MACH_SB2) || defined (CONFIG_MACH_GH2))		
	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	iproc_idm_base_phys[1] = (void __iomem *)res_mem->start;
#endif
    
	/* register IDM timeout interrupt handler */
	request_idm_timeout_interrupts(np);   
}


static void bcm_iproc_restart(enum reboot_mode mode, const char *cmd)
{
	void * __iomem reg_addr;
	u32 reg;
	
	/* CRU_RESET register */	
	reg_addr = (void * __iomem) (get_iproc_dmu_pcu_base() + DMU_CRU_RESET_BASE);

	/* set iproc_reset_n to 0, it may come back or not ... TBD */
	reg = readl_relaxed(reg_addr);
	reg &= ~((u32) 1 << 1);
			
	writel_relaxed(reg, reg_addr);
	
	/* Wait for reset */
	while (1)
		cpu_do_idle();	
}

DT_MACHINE_START(iProc_DT, "BRCM XGS iProc")
	.smp = smp_ops(iproc_smp_ops),
	.init_early = bcm_iproc_init_early,
	.init_machine = bcm_iproc_init,
	.init_time = bcm_iproc_timer_init,
	.dt_compat = bcm_iproc_dt_compat,
	.restart = bcm_iproc_restart,
MACHINE_END
