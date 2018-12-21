/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <asm/io.h>
#include <asm/iproc-common/sysmap.h>
#include <asm/iproc-common/reg_utils.h>
#include <asm/arch/socregs.h>

DECLARE_GLOBAL_DATA_PTR;

typedef struct iproc_clk_struct_t {
    uint32_t arm_clk; /* A9 core clock */
    uint32_t arm_periph_clk;
    uint32_t axi_clk;
    uint32_t apb_clk;
} iproc_clk_struct;

static iproc_clk_struct iproc_clk;

#if defined(CONFIG_HURRICANE2)
unsigned int genpll_clk_tab[] = {
	/*  mode,	ndiv,	MDIV
						0	1	2	3	4	5 */

		0,		80,		10,	0,	0,	5,	8,	0,		/* 400 MHz AXI */
		2,		80,		10,	0,	0,	7,	8,	0,		/* 285 MHz AXI */
		3,		80,		10,	0,	0,	8,	8,	0,		/* 250 MHz AXI */	
		4,		80,		10,	0,	0,	10,	8,	0,		/* 200 MHz AXI */	
		5,		80,		10,	0,	0,	20,	8,	0,		/* 100 MHz AXI */	
		0xffffffff
};
#endif
#if defined(CONFIG_HELIX4)
unsigned int genpll_clk_tab[] = {
	/*  mode,	ndiv,	MDIV
						0	1	2	3	4	5 */
		0,		60,		12,	0,	50,	3,	6,	0,		/* 500 MHz AXI(Ch3), ch0:125, ch2:30, ch4:250 */
		1,		80,		16,	0,	50,	5,	8,	0,		/* 400 MHz AXI */
		2,		60,		12,	0,	50,	5,	6,	0,		/* 300 MHz AXI */
		3,		60,		12,	0,	50,	6,	6,	0,		/* 250 MHz AXI */	
		4,		80,		16,	0,	50,	10,	8,	0,		/* 200 MHz AXI */	
		5,		60,		12,	0,	50,	15,	6,	0,		/* 100 MHz AXI */	
		0xffffffff
};
#endif
#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2))
unsigned int genpll_clk_tab[] = {
	/*  mode,	ndiv,	MDIV
						0	1	2	3	4	5 */
		0,		96,		12,	0,	80,	6,	5,	40,		/* 400 MHz AXI */
		1,		96,		12,	0,	80,	12,	5,	40,		/* 200 MHz AXI */
		0xffffffff
};
#endif

#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) ||\
     defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2))

uint32_t iproc_config_genpll(uint32_t mode)
{
   volatile uint32_t rdata;
   volatile uint32_t lock;
   int i = 0;

	while(1) {
		if(genpll_clk_tab[i] == mode)
			break;
		if(genpll_clk_tab[i] == 0xffffffff) {
			return(1);
		}
		i += 8;
	}

   // Clear Load_en Channel3 & Channel4
   rdata = reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL3);
   rdata &= 0xffc0ffff;
   reg32_write((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL3,rdata);

	// Write fast_lock =1
   rdata = reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL0);
   rdata |= (1<<IPROC_WRAP_GEN_PLL_CTRL0__FAST_LOCK);
   reg32_write((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL0,rdata);

   // Write NDIV
   rdata = reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL1);
   rdata &= 0xfffffc00;
   rdata |= (genpll_clk_tab[i+1] << IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_R);
   reg32_write((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL1,rdata);

   // Write MDIV
   rdata = reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL2);
   rdata &= 0xff0000ff;
   rdata |= ((genpll_clk_tab[i+5] <<IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_R)|
            (genpll_clk_tab[i+6]<<IPROC_WRAP_GEN_PLL_CTRL2__CH4_MDIV_R));
   reg32_write((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL2,rdata);

   // Write PLL_LOAD
   rdata = reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL3);
   rdata |= (1<<IPROC_WRAP_GEN_PLL_CTRL3__SW_TO_GEN_PLL_LOAD);
   reg32_write((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL3,rdata);

   // Load Channel3 & Channel4
   rdata &= 0xffc0ffff;
   rdata |= (0x18<<IPROC_WRAP_GEN_PLL_CTRL3__LOAD_EN_CH_R);
   reg32_write((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL3,rdata);

   // Wait for IPROC_WWRAP GENPLL lock
   do{
     rdata = reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_STATUS);
     lock = ((rdata>>IPROC_WRAP_GEN_PLL_STATUS__GEN_PLL_LOCK)&1);
   }while(!lock);

	return(0);
}

uint32_t iproc_get_axi_clk(uint32_t refclk)
{
#if defined(CONFIG_HURRICANE2_EMULATION) || defined(CONFIG_IPROC_EMULATION)
	return(IPROC_AXI_CLK); /* return the emulator clock defined in configuration file */
#else
   uint32_t ndiv, mdiv, pdiv;

   ndiv = (reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL1) >> IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_R) &
			((1 << IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_WIDTH) -1);
   if(ndiv == 0)
	   ndiv = 1 << IPROC_WRAP_GEN_PLL_CTRL1__NDIV_INT_WIDTH;

   pdiv = (reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL1) >> IPROC_WRAP_GEN_PLL_CTRL1__PDIV_R) &
			((1 << IPROC_WRAP_GEN_PLL_CTRL1__PDIV_WIDTH) -1);
   if(pdiv == 0)
	   pdiv = 1 << IPROC_WRAP_GEN_PLL_CTRL1__PDIV_WIDTH;

   mdiv = (reg32_read((volatile uint32_t *)IPROC_WRAP_GEN_PLL_CTRL2) >> IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_R) &
			((1 << IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_WIDTH) -1);
   if(mdiv == 0)
	   mdiv = 1 << IPROC_WRAP_GEN_PLL_CTRL2__CH3_MDIV_WIDTH;

	return refclk * ndiv / pdiv / mdiv;
#endif
}
#endif

#if defined(CONFIG_KATANA2)
uint32_t iproc_get_axi_clk(uint32_t refclk)
{
#if defined(CONFIG_KATANA2_EMULATION)
	return(IPROC_AXI_CLK); /* return the emulator clock defined in configuration file */
#else
	return(495000000);
#endif
}
#endif

#if defined(CONFIG_SABER2)
uint32_t iproc_get_axi_clk(uint32_t refclk)
{
#if defined(CONFIG_IPROC_EMULATION)
    return (IPROC_AXI_CLK); /* return clock defined in configuration file */
#else
    uint32_t ndiv, mdiv, pdiv;

    ndiv = (reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_PLL_CTRL_3) >> IPROC_WRAP_IPROC_PLL_CTRL_3__NDIV_INT_R) &
			((1 << IPROC_WRAP_IPROC_PLL_CTRL_3__NDIV_INT_WIDTH) -1);

    mdiv = (reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_PLL_CTRL_5) >> IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_R) &
			((1 << IPROC_WRAP_IPROC_PLL_CTRL_5__CH1_MDIV_WIDTH) -1);

    pdiv = (reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_PLL_CTRL_1) >> IPROC_WRAP_IPROC_PLL_CTRL_1__PDIV_R) &
			((1 << IPROC_WRAP_IPROC_PLL_CTRL_1__PDIV_WIDTH) -1);

    return refclk * ndiv / pdiv / mdiv;
#endif
}
#endif

uint32_t iproc_get_uart_clk(uint32_t uart)
{
	uint32_t uartclk; 
	
#if defined(CONFIG_SABER2)
    uartclk = iproc_get_axi_clk(CONFIG_IPROC_PLL_REF_CLK) / 4; /* APB clock */
#else
	uartclk = iproc_get_axi_clk(CONFIG_SYS_REF_CLK) / 4; /* APB clock */
#endif

#if !defined(CONFIG_IPROC_P7)
	if (uart < 2) {
        uint32_t uartclkovr, uartclksel;
		/* CCA UART */
		uartclkovr = (reg32_read((volatile uint32_t *)ChipcommonA_CoreCtrl) >> ChipcommonA_CoreCtrl__UARTClkOvr) & 0x01;
		uartclksel = (reg32_read((volatile uint32_t *)APBW_IDM_IDM_IO_CONTROL_DIRECT) >> APBW_IDM_IDM_IO_CONTROL_DIRECT__UARTClkSel) & 0x01;
		if(!uartclkovr) {
			if(uartclksel) {
				uartclk /= ((reg32_read((volatile uint32_t *)ChipcommonA_ClkDiv) >> ChipcommonA_ClkDiv__UartClkDiv_R) & 
					        ((1 << ChipcommonA_ClkDiv__UartClkDiv_WIDTH) - 1));
			}
			else{
				uartclk = CONFIG_SYS_REF_CLK; /* Reference clock */
			}
		}
	}
#endif /* !CONFIG_IPROC_P7 */

	return(uartclk);
}

uint32_t iproc_get_periph_clk(void)
{
	if ((gd->flags & GD_FLG_RELOC) && iproc_clk.arm_periph_clk) {
		return iproc_clk.arm_periph_clk;
	}
	return IPROC_ARM_CLK/2;
}

uint32_t iproc_get_apb_clk(void)
{
	if ((gd->flags & GD_FLG_RELOC) && iproc_clk.apb_clk) {
		return iproc_clk.arm_periph_clk;
	}
	return IPROC_APB_CLK;
}

void iproc_clk_enum(void)
{
    uint32_t arm_refclk = CONFIG_SYS_REF_CLK;
#if defined(CONFIG_IPROC_EMULATION)
    iproc_clk.arm_clk = IPROC_ARM_CLK;
    iproc_clk.axi_clk = IPROC_AXI_CLK;
    iproc_clk.apb_clk = IPROC_APB_CLK;
#else
	uint32_t pll_arma, pll_armb, ndiv_int,pdiv, freq_id, mdiv;
	unsigned long long vco_freq;
	
#if defined(CONFIG_GREYHOUND)
  if (reg32_read((volatile uint32_t *)IPROC_WRAP_TOP_STRAP_STATUS_1) & 0x01)  /* 50 MHz */
 	  arm_refclk = 50000000;
#endif

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	pll_arma = *((uint32_t *)IHOST_PROC_CLK_PLLARMA);
	pll_armb = *((uint32_t *)IHOST_PROC_CLK_PLLARMB);
#else
	pll_arma = swap_u32(*((uint32_t *)IHOST_PROC_CLK_PLLARMA));
	pll_armb = swap_u32(*((uint32_t *)IHOST_PROC_CLK_PLLARMB));
#endif
	ndiv_int = (pll_arma >> 8) & 0x3FF;
	if(!ndiv_int) 
		ndiv_int = 512;
	pdiv = (pll_arma >> IHOST_PROC_CLK_PLLARMA__pllarm_pdiv_R) & 0x07;
	if(!pdiv)
		pdiv = 0x0F;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	freq_id = (*((uint32_t *)IHOST_PROC_CLK_POLICY_FREQ)) & 0x07;
#else
	freq_id = swap_u32((*((uint32_t *)IHOST_PROC_CLK_POLICY_FREQ))) & 0x07;
#endif
	vco_freq = (ndiv_int << 20) + (pll_armb & 0xFFFFF);
	vco_freq = ((vco_freq * arm_refclk) >> 20) / pdiv;
	//printf("ndiv=%d, pdiv=%d, refclk=%d, vco=%d\n", ndiv_int, pdiv, arm_refclk ,vco_freq); 
	if(freq_id == 0) {
		iproc_clk.arm_clk = arm_refclk;
		iproc_clk.axi_clk = arm_refclk;
		iproc_clk.apb_clk = arm_refclk;
	}
	else if(freq_id == 2) {
		iproc_clk.arm_clk = CONFIG_SYS_REF2_CLK / 2;
		iproc_clk.axi_clk = CONFIG_SYS_REF2_CLK / 2;
		iproc_clk.apb_clk = CONFIG_SYS_REF2_CLK / 4;
	}
	else if(freq_id == 6) {
		iproc_clk.arm_clk = vco_freq / 4;
		iproc_clk.axi_clk = iproc_clk.arm_clk / 2;
		iproc_clk.apb_clk = iproc_clk.arm_clk / 4;
	}
	else if(freq_id == 7) {
		/* For freq_id 7, h_mdiv is the divider */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		mdiv = *((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xff;
#else
		mdiv = swap_u32(*((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5)) & 0xff;
#endif
		iproc_clk.arm_clk = vco_freq / mdiv;
		iproc_clk.axi_clk = iproc_clk.arm_clk / 2;
		iproc_clk.apb_clk = iproc_clk.arm_clk / 4;
	}
	else {
		/* TO BE REVIEWED */
		iproc_clk.arm_clk = vco_freq / 2;
		iproc_clk.axi_clk = iproc_clk.arm_clk / 2;
		iproc_clk.apb_clk = iproc_clk.arm_clk / 4;
	}
#endif

#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) \
    || defined(CONFIG_KATANA2) || defined(CONFIG_GREYHOUND) \
    || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2))
	iproc_clk.axi_clk = iproc_get_axi_clk(arm_refclk);
	iproc_clk.apb_clk = iproc_clk.axi_clk / 4;
#endif
#if defined(CONFIG_SABER2)
	iproc_clk.axi_clk = iproc_get_axi_clk(CONFIG_IPROC_PLL_REF_CLK);
	iproc_clk.apb_clk = iproc_clk.axi_clk / 4;
#endif
	iproc_clk.arm_periph_clk = iproc_clk.arm_clk / 2;
	if(iproc_clk.apb_clk < 1000000 || iproc_clk.arm_periph_clk < 1000000) {
		printf("arm_clk=%dHz, axi_clk=%dHz, apb_clk=%dHz, arm_periph_clk=%dHz\n", 
		iproc_clk.arm_clk, iproc_clk.axi_clk, 
		iproc_clk.apb_clk, iproc_clk.arm_periph_clk);
	}
	else {
		printf("arm_clk=%dMHz, axi_clk=%dMHz, apb_clk=%dMHz, arm_periph_clk=%dMHz\n", 
		iproc_clk.arm_clk/1000000, iproc_clk.axi_clk/1000000, 
		iproc_clk.apb_clk/1000000, iproc_clk.arm_periph_clk/1000000);
	}
}
