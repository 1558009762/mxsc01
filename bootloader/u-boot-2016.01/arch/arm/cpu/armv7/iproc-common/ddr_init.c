/*
 * Copyright 2016 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#if defined(CONFIG_RUN_DDR_SHMOO2) && defined(CONFIG_SHMOO_REUSE)
#include <malloc.h>
#include <linux/mtd/mtd.h>
#include <spi.h>
#include <spi_flash.h>
#include <nand.h>
#endif
#include <asm/arch/socregs.h>
#include <asm/iproc-common/reg_utils.h>
#include <asm/iproc-common/ddr40_phy_init.h>
#include <asm/iproc-common/shmoo_public.h>
#include "shmoo_sig.h"

extern unsigned int ddr_init_tab[];
extern unsigned int ddr2_init_tab[];
extern unsigned int ddr2_init_tab_400[];
extern unsigned int ddr2_init_tab_667[];
extern unsigned int ddr2_init_tab_800[];
extern unsigned int ddr2_init_tab_1066[];
extern unsigned int ddr3_init_tab[];
extern unsigned int ddr3_init_tab_667[];
extern unsigned int ddr3_init_tab_800[];
extern unsigned int ddr3_init_tab_933[];
extern unsigned int ddr3_init_tab_1066[];
extern unsigned int ddr3_init_tab_1333[];
extern unsigned int ddr3_init_tab_1600[];
extern unsigned int ddr2_mode_reg_tab[];
extern unsigned int ddr3_mode_reg_tab[];
extern unsigned int ddr_clk_tab[];
extern unsigned int ddr_phy_ctl_ovrd_tab[];
extern unsigned int ddr_phy_wl0_tab[];
extern unsigned int mt40a512m8_093_1600[];

#if defined(CONFIG_IPROC_DDR_ECC) && !defined(CONFIG_IPROC_P7)
void iproc_ddr_ovrd_ecc_lane(void)
{
	uint32_t val;

#define SET_OVR_STEP(v) ( 0x30000 | ( (v) & 0x3F ) )    /* OVR_FORCE = OVR_EN = 1, OVR_STEP = v */

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE_RD_EN);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_RD_EN, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_R_P);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_R_P, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_R_N);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_R_N, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT0_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT0_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT1_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT1_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT2_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT2_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT3_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT3_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_DM_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_DM_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT0_R_P);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT0_R_P, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT0_R_N);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT0_R_N, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT1_R_P);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT1_R_P, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT1_R_N);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT1_R_N, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT2_R_P);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT2_R_P, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT2_R_N);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT2_R_N, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT3_R_P);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT3_R_P, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT3_R_N);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT3_R_N, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);


		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT_RD_EN);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT_RD_EN, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

#if 0
		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_DYN_VDL_OVRIDE_BYTE0_R_P);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_DYN_VDL_OVRIDE_BYTE_R_P, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_DYN_VDL_OVRIDE_BYTE0_R_N);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_DYN_VDL_OVRIDE_BYTE_R_N, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_DYN_VDL_OVRIDE_BYTE0_BIT_R_P);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_DYN_VDL_OVRIDE_BYTE_BIT_R_P, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_DYN_VDL_OVRIDE_BYTE0_BIT_R_N);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_DYN_VDL_OVRIDE_BYTE_BIT_R_N, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_DYN_VDL_OVRIDE_BYTE0_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_DYN_VDL_OVRIDE_BYTE_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_DYN_VDL_OVRIDE_BYTE0_BIT_W);
		val = SET_OVR_STEP(val & 0xff);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_DYN_VDL_OVRIDE_BYTE_BIT_W, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif
		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_READ_DATA_DLY);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_READ_DATA_DLY, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_READ_CONTROL);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_READ_CONTROL, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);


		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_IDLE_PAD_CONTROL);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_IDLE_PAD_CONTROL, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_DRIVE_PAD_CTL);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_DRIVE_PAD_CTL, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#if 0
		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_CLOCK_PAD_DISABLE);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_CLOCK_PAD_DISABLE, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif	
		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_WR_PREAMBLE_MODE);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_WR_PREAMBLE_MODE, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#if 0
		val = reg32_read((volatile uint32_t *)DDR_PHY_WORD_LANE_0_PHYBIST_VDL_ADJ);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_PHYBIST_VDL_ADJ, val);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif
		__udelay(200);
		reg32_write((volatile uint32_t *)DDR_PHY_ECC_LANE_READ_FIFO_CLEAR, 0x1);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
		__udelay(200);
}

uint32_t iproc_read_ecc_syndrome(void)
{
	volatile uint32_t syndrome = 0;
	/* Place uncorrectible as bits 7:0, and correctible as 15:8 */
	syndrome = ((reg32_read((volatile uint32_t *)DDR_DENALI_CTL_89) >> 3) & 0x1) |
				(((reg32_read((volatile uint32_t *)DDR_DENALI_CTL_89) >> 5) & 0x1));
	return(syndrome);
}

void iproc_clear_ecc_syndrome(void)
{
	/* Clear the interrupts, bits 6:3 */
	reg32_write((volatile uint32_t *)DDR_DENALI_CTL_213, (1 << 5) | (1<< 3));
	__udelay(100);
}
#endif

#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2))
uint32_t iproc_get_ddr3_clock_mhz(uint32_t unit)
{
	uint32_t ndiv, mdiv, ddrclk;
	ndiv = reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_4) & 0xff;
	mdiv = reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_2) & 0xff; /* Ch1 MDIV */
	ddrclk = (25 * ndiv * 2) / mdiv;
	if(ddrclk == 466 || ddrclk == 666)
		ddrclk +=1;
	return(ddrclk);
}
#elif (defined(CONFIG_KATANA2))
uint32_t iproc_get_ddr3_clock_mhz(uint32_t unit)
{
	uint32_t ndiv, mdiv, ddrclk;
	
	/* read ndiv */
	ndiv = (reg32_read((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_3) >> 
		    IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_R) & 
			((1 << IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_WIDTH) - 1);
	
	/* read channel 0 mdiv */
	mdiv = (reg32_read((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_4) >> 
		IPROC_DDR_PLL_CTRL_REGISTER_4__CH0_MDIV_R) & 
		((1 << IPROC_DDR_PLL_CTRL_REGISTER_4__CH0_MDIV_WIDTH) - 1);
	
	
	ddrclk = (25 * ndiv * 2) / mdiv;
	if(ddrclk == 466 || ddrclk == 666)
		ddrclk +=1;
	printf("%s ndiv(0x%x) mdiv(0x%x) ddrclk(0x%x)\n", __FUNCTION__, ndiv, mdiv, ddrclk);
	return(ddrclk);
}
#elif (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2))
uint32_t iproc_get_ddr3_clock_mhz(uint32_t unit)
{
    uint32_t ndiv, mdiv, pdiv, ddrclk, data;

    data = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_PLL_DIVIDERS);
      
    ndiv = data >> DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__NDIV_INT_R;
    ndiv &= (2^DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__NDIV_INT_WIDTH) - 1;

    pdiv = data >> DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__PDIV_R;
    pdiv &= (2^DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__PDIV_WIDTH) - 1;

    mdiv = data >> DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__MDIV_R;
    mdiv &= (2^DDR_PHY_CONTROL_REGS_PLL_DIVIDERS__MDIV_WIDTH) - 1;

    /* read ndiv pdiv and mdiv */
    ddrclk = (25 * ndiv * 2 * pdiv) / mdiv;
    printf("%s DDR PHY PLL divisor: ndiv(0x%x) mdiv(0x%x) ddrclk(0x%x)\n", __FUNCTION__, ndiv, mdiv, ddrclk);

    return(ddrclk);
}
#endif

uint32_t change_ddr_clock(uint32_t clk)
{
#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) || defined(CONFIG_KATANA2))
    int i = 0;
	uint32_t val;
#endif /* (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) || defined(CONFIG_KATANA2)) */
#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2))
	while(1) {
		if(ddr_clk_tab[i] == clk)
			break;
		if(ddr_clk_tab[i] == 0xffffffff) {
			printf("Error in change_ddr_clock: Can't find clock configuration requested\n");
			return(1);
		}
		i += 8;
	}

	/* Program LCPLL NDIV */
	reg32_write_masked((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_4, 0xff, ddr_clk_tab[i + 1]);
	/* Program Channel dividers */
	reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_0, 
		ddr_clk_tab[i + 7] << 24 | ddr_clk_tab[i + 6] << 16 | 
		ddr_clk_tab[i + 5] << 8 | ddr_clk_tab[i + 2]); /*ch 5,4,3,0 */
	reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_2, 
		ddr_clk_tab[i + 4] << 20 | ddr_clk_tab[i + 3]); /*ch 2, 1 */

	/* Load new divider settings */
	val = ((IPROC_WRAP_IPROC_XGPLL_CTRL_1_RESETVALUE|
            (1<<IPROC_WRAP_IPROC_XGPLL_CTRL_1__SW_OVWR)) &
            (~(1<<IPROC_WRAP_IPROC_XGPLL_CTRL_1__POST_RESETB))&
            (~(1<<IPROC_WRAP_IPROC_XGPLL_CTRL_1__RESETB)));
	reg32_write((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, val);

	/* release the PLL reset */
	reg32_set_bits((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, (1<<IPROC_WRAP_IPROC_XGPLL_CTRL_1__RESETB));

   /* Wait for IPROC_XGPLL lock */
	do{
		val = reg32_read((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_STATUS);
	}while(!((val>>IPROC_WRAP_IPROC_XGPLL_STATUS__IPROC_WRAP_XGPLL_LOCK)&1));

	reg32_set_bits((volatile uint32_t *)IPROC_WRAP_IPROC_XGPLL_CTRL_1, (1<<IPROC_WRAP_IPROC_XGPLL_CTRL_1__POST_RESETB));

#elif defined(CONFIG_KATANA2)
	while(1) {
		if(ddr_clk_tab[i] == clk)
			break;
		if(ddr_clk_tab[i] == 0xffffffff) {
			printf("Error in change_ddr_clock: Can't find clock configuration requested\n");
			return(1);
		}
		i += 4;
	}

	reg32_clear_bits((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_5, 
	                 1 << IPROC_DDR_PLL_CTRL_REGISTER_5__IPROC_DDR_PLL_RESETB);	

	reg32_set_bits((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_5, 
		            (1<<IPROC_DDR_PLL_CTRL_REGISTER_5__IPROC_DDR_PLL_SW_OVWR));

	reg32_clear_bits((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_5, 
	            1 << IPROC_DDR_PLL_CTRL_REGISTER_5__IPROC_DDR_PLL_POST_RESETB);
	/* Program Ch1 (USB clock) divider */
	reg32_write_masked((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_5, 
		((1 << IPROC_DDR_PLL_CTRL_REGISTER_5__CH1_MDIV_WIDTH) - 1) << 
		IPROC_DDR_PLL_CTRL_REGISTER_5__CH1_MDIV_R, 
		ddr_clk_tab[i + 3] << IPROC_DDR_PLL_CTRL_REGISTER_5__CH1_MDIV_R);
	/* Program Ch0 (DDR clock) divider */
	reg32_write_masked((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_4, 
		((1 << IPROC_DDR_PLL_CTRL_REGISTER_4__CH0_MDIV_WIDTH) - 1) << 
		IPROC_DDR_PLL_CTRL_REGISTER_4__CH0_MDIV_R, 
		ddr_clk_tab[i + 2] << IPROC_DDR_PLL_CTRL_REGISTER_4__CH0_MDIV_R);

	/* Program ndiv */
	reg32_write_masked((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_3, 
		((1 << IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_WIDTH) - 1) << 
		IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_R, 
		ddr_clk_tab[i + 1] << IPROC_DDR_PLL_CTRL_REGISTER_3__NDIV_INT_R);
	
	/* release the PLL reset whih loads the new driver settings */
	reg32_set_bits((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_5, 
		            (1<<IPROC_DDR_PLL_CTRL_REGISTER_5__IPROC_DDR_PLL_RESETB));

   /* Wait for PLL lock */
	do{
		val = reg32_read((volatile uint32_t *)IPROC_DDR_PLL_STATUS);
	}while(!((val>>IPROC_DDR_PLL_STATUS__IPROC_DDR_PLL_LOCK)&1));

	reg32_set_bits((volatile uint32_t *)IPROC_DDR_PLL_CTRL_REGISTER_5, 
		       (1<<IPROC_DDR_PLL_CTRL_REGISTER_5__IPROC_DDR_PLL_POST_RESETB));
#endif
	return(0);
}

void dump_phy_regs(void)
{
#if 0	
	int i;
	printf("\n PHY register dump: Control registers\n");
	for(i = 0; i <= 0xc4; i+=4)
	{
		printf("0x%03x,\t0x%08x,\n", i, *(volatile uint32_t *)(DDR_PHY_CONTROL_REGS_REVISION + i));
	}

	printf("\n PHY register dump: Wordlane0 registers\n");
	for(i = 0; i <= 0x1b0; i+=4)
	{
		printf("0x%03x,\t0x%08x,\n", i, *(volatile uint32_t *)(DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE_RD_EN + i));
	}

#endif	
}
 
void ddr_init_regs(unsigned int * tblptr)
{
	unsigned int offset = *tblptr;
	unsigned int *addr = (unsigned int *)DDR_DENALI_CTL_00;

	while(offset != 0xffffffff) {
		++tblptr;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		addr[offset] = *tblptr;
#else
		addr[offset] = swap_u32(*tblptr);
#endif
		++tblptr;
		offset = *tblptr;
	}
}

void ddr_phy_ctl_regs_ovrd(unsigned int * tblptr)
{
	unsigned int offset = *tblptr;
	unsigned int *addr = (unsigned int *)DDR_PHY_CONTROL_REGS_REVISION;
	unsigned int val;

	while(offset != 0xffffffff) {
		++tblptr;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		addr[offset/4] = *tblptr;
#else
		addr[offset/4] = swap_u32(*tblptr);
#endif
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
		if (val) ;
		++tblptr;
		offset = *tblptr;
	}
}

void ddr_phy_wl_regs_ovrd(unsigned int * tblptr)
{
#if (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2) || defined(CONFIG_HURRICANE2))
	unsigned int offset = *tblptr;
	unsigned int *addr = (unsigned int *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE_RD_EN;
	unsigned int val;

	while(offset != 0xffffffff) {
		++tblptr;
		if(offset < 0x118) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
		    addr[offset/4] = *tblptr | (1 << 17);
#else
		    addr[offset/4] = swap_u32(*tblptr | (1 << 17));
#endif
		}
		else {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
			addr[offset/4] = *tblptr;
#else
			addr[offset/4] = swap_u32(*tblptr);
#endif
		}
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
		if (val) ;
		++tblptr;
		offset = *tblptr;
	}
#endif 
}

/*DDR_SHMOO_RELATED_CHANGE*/
#if 0
uint32_t ddr_write_mode_regs(uint32_t * tblptr) {
	int i, j;

	reg32_clear_bits((volatile uint32_t *)DDR_DENALI_CTL_89, 1 << 18);
	for(i = 0; i < 4; i++) {
		reg32_write_masked((volatile uint32_t *)DDR_DENALI_CTL_47, 0xffff << 16, tblptr[i] << 16); // CS0
		reg32_write_masked((volatile uint32_t *)DDR_DENALI_CTL_51, 0xffff << 16, tblptr[i] << 16); // CS1
		reg32_write_masked((volatile uint32_t *)DDR_DENALI_CTL_55, 0xffff << 16, tblptr[i] << 16); // CS2
		reg32_write_masked((volatile uint32_t *)DDR_DENALI_CTL_59, 0xffff << 16, tblptr[i] << 16); // CS3
		/* Set mode register for single write for all chip selects */
		reg32_write((volatile uint32_t *)DDR_DENALI_CTL_43, 1 << 23 | 1 << 24 | i); 
		/* Trigger Mode Register Write(MRW) sequence */
		reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_43, 1 << 25);
		j = 100;
		do {
			if(reg32_read((volatile uint32_t *)DDR_DENALI_CTL_89) & (1 << 18))
				break;
			--j;
		}while(j);
		if (j == 0 && (reg32_read((volatile uint32_t *)DDR_DENALI_CTL_89) & (1 << 18)) == 0) {
			printf("Error: DRAM mode register write failed\n");
			return(1);
		}
	}
	return(0);
}
#endif

#ifdef CONFIG_RUN_DDR_SHMOO
int ReWriteModeRegisters( void )
{
    /*DDR_SHMOO_RELATED_CHANGE: comment out calling ddr_write_mode_regs(), and add new implementation*/	
#if 0	
    unsigned int nRet = ddr_write_mode_regs( ddr3_mode_reg_tab );
    return nRet;
#endif    
    int nRet = 0;
    int j = 100;

    reg32_clear_bits( (volatile uint32_t *)DDR_DENALI_CTL_89 , 1 << 18 );

    /* Set mode register for MR0, MR1, MR2 and MR3 write for all chip selects */
    reg32_write( (volatile uint32_t *)DDR_DENALI_CTL_43 , (1 << 17) | (1 << 24) | (1 << 25) );

    /* Trigger Mode Register Write(MRW) sequence */
    reg32_set_bits( (volatile uint32_t *)DDR_DENALI_CTL_43 , 1 << 25 );

    do {
        if ( reg32_read( (volatile uint32_t *)DDR_DENALI_CTL_89) & (1 << 18) ) {
            break;
        }
        --j;
    } while( j );

    if ( j == 0 && (reg32_read( (volatile uint32_t *)DDR_DENALI_CTL_89) & (1 << 18) ) == 0 ) {
        printf("Error: DRAM mode registers write failed\n");
        nRet = 1;
    };

    return nRet;
}
#endif /* CONFIG_RUN_DDR_SHMOO */

int is_ddr_32bit(void)
{
    int ddr32 = 0;
#if (CONFIG_SHMOO_REUSE_DDR_32BIT)
    ddr32 = 1;
#endif /* (CONFIG_SHMOO_REUSE_DDR_32BIT) */

#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_SABER2) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2))
#ifdef CONFIG_DDR32
    ddr32 = 1;
#else
    ddr32 = 0;
#endif
#endif 
	return ddr32;
}

#if (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2) || defined(CONFIG_HURRICANE2))
void ddr_init(void)
{
	int i;
	volatile unsigned int val;
	int ddr_type;
	uint32_t params, connect, ovride, status, otp_status = 0, sku_id, ddr_clk, dev_id, rev_id;
	uint32_t wire_dly[4] = {0};

    /* Assert S1 and S2 reset, and wait for DDR power to be stabilized */
    reg32_write((volatile uint32_t *)DDR_S1_IDM_RESET_CONTROL, 1);
    reg32_write((volatile uint32_t *)DDR_S2_IDM_RESET_CONTROL, 1);
    printf("Wait.\n");
    __udelay(500*1000);
    printf("Done.\n");

	dev_id = (reg32_read((volatile uint32_t *)ChipcommonA_ChipID)) & 0x0000ffff;
	printf("DEV ID= %08x\n", dev_id);
	dev_id &= 0x0000ffff;

	rev_id = ((reg32_read((volatile uint32_t *)ChipcommonA_ChipID)) >> 16) & 0xf;
	printf("REV ID= %08x\n", rev_id);

	sku_id = (reg32_read((volatile uint32_t *)ROM_S0_IDM_IO_STATUS) >> 2) & 0x03;
	printf("SKU ID = %d\n", sku_id);

	ddr_type = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS) & 0x1; 

#if CONFIG_HELIX4
#ifdef CONFIG_DDRCLK
	ddr_clk = CONFIG_DDRCLK;
#else
	ddr_clk = 800;
#endif
#elif CONFIG_HURRICANE2
#ifdef CONFIG_DDRCLK
	ddr_clk = CONFIG_DDRCLK;
#else
	ddr_clk = 533;
#endif
#else
	ddr_clk = 400; /* default value */
#endif
	printf("MEMC 0 DDR speed = %dMHz\n", ddr_clk);
	status = change_ddr_clock(ddr_clk);
	if(status) {
		printf("CRU LCPLL configuratioin failed\n");
		return;
	}


	/* Get the DDR S1 and S2 out of reset */
	reg32_write((volatile uint32_t *)DDR_S1_IDM_RESET_CONTROL, 0);
	reg32_write((volatile uint32_t *)DDR_S2_IDM_RESET_CONTROL, 0);
	/* Set the ddr_ck to 400 MHz, 2x memc clock */
	reg32_write_masked((volatile uint32_t *)DDR_S1_IDM_IO_CONTROL_DIRECT, 0xfff << 16, /*ddr_clk*/ 0x190 << 16);     

	/* Wait for DDR PHY up */
	for(i=0; i < 0x19000; i++) {
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
		if( val != 0)
			break; /* DDR PHY is up */
	}
	
	if(i == 0x19000) {
		printf("DDR PHY is not up\n");
		return;
	}

	if(otp_status && (sku_id == 0)) {
		/* High SKU */
		reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_PLL_DIVIDERS, 0x0c10);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
	}
#ifdef CONFIG_RUN_DDR_SHMOO
    // PHY init parameters
#ifdef CONFIG_DDR_LONG_PREAMBLE    
    params = DDR40_PHY_PARAM_USE_VTT |
		/*
             DDR40_PHY_PARAM_DIS_ODT |
             DDR40_PHY_PARAM_DIS_DQS_ODT |
		*/
             DDR40_PHY_PARAM_ODT_LATE |
             DDR40_PHY_PARAM_ADDR_CTL_ADJUST_0 |
             DDR40_PHY_PARAM_ADDR_CTL_ADJUST_1 |
             DDR40_PHY_PARAM_MAX_ZQ |
             DDR40_PHY_PARAM_LONG_PREAMBLE;
#else
     params = DDR40_PHY_PARAM_USE_VTT |
		/*
             DDR40_PHY_PARAM_DIS_ODT |
             DDR40_PHY_PARAM_DIS_DQS_ODT |
		*/
             DDR40_PHY_PARAM_ODT_LATE |
             DDR40_PHY_PARAM_ADDR_CTL_ADJUST_0 |
             DDR40_PHY_PARAM_ADDR_CTL_ADJUST_1 |
             DDR40_PHY_PARAM_MAX_ZQ;
#endif             
	
	if(ddr_type) {
		/* DDR3, 1.5v */
		params |= DDR40_PHY_PARAM_VDDO_VOLT_0;
	}
	else {
		/* DDR2, 1.8v */
		params |= DDR40_PHY_PARAM_VDDO_VOLT_1;
	}
    connect = 0x01CF7FFF;
    ovride = 0x00077FFF;
/*DDR_SHMOO_RELATED_CHANGE: change clock from 400 to ddr_clk*/    
	status = ddr40_phy_init(ddr_clk, params, ddr_type, wire_dly, connect, ovride, (uint32_t)DDR_PHY_CONTROL_REGS_REVISION);
	if(status != DDR40_PHY_RETURN_OK) {
		printf("Error: ddr40_phy_init failed with error 0x%x\n", status);
		return;
	}

#else

#if defined(CONFIG_HELIX4)
	/* mhz = 0x190, 32 bit bus, 16 bit chip width, 1.35v vddq, 8Gb chip size, jedec type = 25 (DDR3-1866M) */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL, 0x0190c4f3);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif
#if defined(CONFIG_HURRICANE2)
	/* mhz = 0x190, 16 bit bus, 16 bit chip width, 1.35v vddq, 8Gb chip size, jedec type = 25 (DDR3-1866M) */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL, 0x0190d4f3);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif
#if defined(CONFIG_HELIX4)
	/* ECC =1, DRAM type = 1 i.e DDR3, AL = 3, write recovery = 0, write latency = 9, CAS read latency = 13 */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL2, 0x0036048d);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif
#if defined(CONFIG_HURRICANE2)
	/* DRAM type = 1 i.e DDR3, AL = 3, write recovery = 0, write latency = 9, CAS read latency = 13 */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL2, 0x0016048d);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif


	/* Set LDO output voltage control to 1.00 * VDDC, and enable PLL */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_PLL_CONFIG, 1 << 4);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

	/* Wait for PLL locked */
	for(i=0; i < 0x1400; i++) {
		if(reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_PLL_STATUS) & 0x1)
			break; /* PLL locked */
	}

	if(i == 0x1400) {
		printf("DDR PHY PLL lock failed\n");
		return;
	}

	/* Set Read Data delay to 2 clock cycles */
	if(otp_status && (sku_id == 0)) {
		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_READ_DATA_DLY, 3);
	}
	else {
		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_READ_DATA_DLY, 2);
	}
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#if defined(CONFIG_HELIX4)
	reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_READ_DATA_DLY, 2);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif
	
	/* Write 2 if ddr2, 3 if ddr3 */
	/* Set preamble mode according to DDR type, and length of write preamble to 1.5 DQs, 0.75 DQ  */
	val = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS);
#ifdef CONFIG_DDR_LONG_PREAMBLE	
	val = (val & 1) | 0x2;
#else
	val = (val & 1);
#endif	 
	reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_WR_PREAMBLE_MODE, val);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#if defined(CONFIG_HELIX4)
	reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_WR_PREAMBLE_MODE, val);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif

	/* Initiate a PVT calibration cycle */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_ZQ_PVT_COMP_CTL, 1 << 20);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

	/* Initiate auto calibration and apply the results to VDLs */
	if(otp_status && (sku_id == 0)) {
		reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_VDL_CALIBRATE, 0x08000101);
	}
	else {
		reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_VDL_CALIBRATE, 0x00000101);
	}
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

	/* Wait for Calibration lock done */
	for(i=0; i < 0x1400; i++) {
		if(reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_VDL_CALIB_STATUS) & 0x1)
			break; /* PLL locked */
	}

	if(i == 0x1400) {
		printf("DDR PHY auto calibration timed out\n");
		return;
	}

	if(reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_VDL_CALIB_STATUS) & 0x2) {
#if 0
		printf("PHY register dump after the auto calibration\n");
		dump_phy_regs();
#endif
		goto ddr_cntrl_prog;
	}
        
	/* Auto calibration failed, do the override */
calib_override:
		printf("Auto calibration failed, do the override\n");
		reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_VDL_OVRIDE_BIT_CTL, 0x0001003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT0_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT1_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT2_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT3_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT4_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT5_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT6_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_BIT7_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_DM_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT0_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT1_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT2_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT3_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT4_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT5_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT6_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_BIT7_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_DM_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_R_P, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE0_R_N, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_R_P, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE1_R_N, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#if defined(CONFIG_HELIX4)
		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT0_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT1_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT2_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT3_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT4_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT5_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT6_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_BIT7_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_DM_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT0_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT1_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT2_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT3_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT4_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT5_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT6_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_BIT7_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_DM_W, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_R_P, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE0_R_N, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_R_P, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE1_R_N, 0x0003003f);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif

#endif
ddr_cntrl_prog:
#ifdef CONFIG_RUN_DDR_SHMOO
	printf("PHY register dump after DDR PHY init\n");
	dump_phy_regs();
#endif

	ddr_init_regs(ddr_init_tab);
	if(ddr_type) {
		/* DDR3 */	
#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2))
		switch(ddr_clk) {
			case 333:
				ddr_init_regs(ddr3_init_tab_667);
				break;
			case 400:
				ddr_init_regs(ddr3_init_tab_800);
				break;
			case 467:
				ddr_init_regs(ddr3_init_tab_933);
				break;
			case 533:
				ddr_init_regs(ddr3_init_tab_1066);
				break;
			case 666:
				ddr_init_regs(ddr3_init_tab_1333);
				break;
			case 800:
				ddr_init_regs(ddr3_init_tab_1600);
				break;
		}

#else
		ddr_init_regs(ddr3_init_tab);
#endif
	}

	else {
		/* DDR2 */	
		ddr_init_regs(ddr2_init_tab);
	}

turnon:
	/* Start the DDR */
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_00, 0x01);

poll_ddr_ctrl:
	while(!(reg32_read((volatile uint32_t *)DDR_DENALI_CTL_89) & 0x100));
	if(otp_status && (sku_id == 0)) {
		reg32_write((volatile uint32_t *)DDR_PHY_WORD_LANE_0_READ_DATA_DLY, 0x03);
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);	
	}


#ifdef CONFIG_RUN_DDR_SHMOO

/*DDR_SHMOO_RELATED_CHANGE: mark out mode register update as it should already be programmed before "turnon"*/
#if 0
	/* Program DDR controller mode registers */
	if(ddr_type) {
		/* DDR3 */	
		if(ddr_write_mode_regs(ddr3_mode_reg_tab)) {
			printf("Error: DRAM mode register write failed\n");
			return;
		}
	}
	else {
		/* DDR2 */	
		if(ddr_write_mode_regs(ddr2_mode_reg_tab)) {
			printf("Error: DRAM mode register write failed\n");
			return;
		}
	}
#endif

	printf("PHY register dump after mode register write\n");
	dump_phy_regs();

	/* Run the SHMOO */
	/// Turn on all SHMOO options for NORTHSTAR - 16 bit PHY
/*DDR_SHMOO_RELATED_CHANGE: chanhe shmoo memory test base from 0x0 (write-through) to 0x1000000 (write-back)*/
	status = do_shmoo(/*(ddr40_addr_t)*/DDR_PHY_CONTROL_REGS_REVISION, 0, ((26 << 16) | (16 << 8) | DO_ALL_SHMOO), 0x1000000);
	if(status != SHMOO_NO_ERROR) {
		printf("Error: do_shmoo failed with error 0x%x\n", status);
		return;
	}


//DDR-EYE-SHMOO: added           
#ifdef CONFIG_MDE_BUILD_RW_EYE 
	{
		volatile unsigned int tmp=10000; //1000 is also OK
		extern unsigned mde_entry (void);

	    reg32_write((volatile uint32_t *)0x1800c180, 0x0000ea68); // unlock
	    reg32_write((volatile uint32_t *)0x1800c154, 0x83c00000); // write bit31=1 , sel_SW_setting
		while(tmp--);  //delay for correct read
		tmp = reg32_read((unsigned int *)0x1800c15c);
		if( tmp & (1<<4) ) { //bit 4 is set as 1 by "shmoo" command
			//restore to original value
	        reg32_write((volatile uint32_t *)0x1800c180, 0x0000ea68); // unlock
	        reg32_write((volatile uint32_t *)0x1800c15c, 0x0003060c); // bit[7:4]: reset indicator
	        reg32_write((volatile uint32_t *)0x1800c154, 0x83c00000); // write bit31=1 , sel_SW_setting
			#warning: MDE to run in non-relocated space in FLASH....
			printf ("Forcing to MDE entry at %08x\n", (unsigned) mde_entry );
			mde_entry ();
		}
	}
#endif /*CONFIG_MDE_BUILD_RW_EYE*/
#endif /* CONFIG_RUN_DDR_SHMOO */
}
#endif

static uint32_t get_ddr_clock(uint32_t sku_id, int ddr_type)
{
#ifdef CONFIG_DDRCLK
	return CONFIG_DDRCLK;
#else
    uint32_t ddr_clk;
#if defined(CONFIG_HELIX4)
	if(ddr_type) 
		ddr_clk = 750; //ddr3 
	else
		ddr_clk = 400; //ddr2 
#elif defined(CONFIG_HURRICANE2)
	/* Set the defaults according to SKU */
	if(sku_id == 0xb151  || sku_id == 0x8342 || sku_id == 0x8343 || sku_id == 0x8344 || 
	   sku_id == 0x8346 || sku_id == 0x8347 || sku_id == 0x8393 || 
	   sku_id == 0x8394) {
	   /* Wolfhound/Deerhound */
		ddr_clk = 400;
	}
	else {
		ddr_clk = 667; /* Hurricane2 - BCM5615x */
	}
#elif defined(CONFIG_KATANA2)
	if(ddr_type) 
		ddr_clk = 800; //ddr3 
	else
		ddr_clk = 400; //ddr2 
#elif defined(CONFIG_GREYHOUND)
    if ((sku_id & 0xfff0) == 0xb060) {
        /* Ranger2 - 5606x: CPU 1250MHz, AXI 400MHz, DDR 800MHz */
        ddr_clk = 800;
    } else {
        /* Greyhound - 5341x: CPU 600MHz, AXI 200MHz, DDR 667MHz */
        ddr_clk = 667; 
    }
#elif defined(CONFIG_HURRICANE3)
    if ((sku_id >= 0xb163) && (sku_id <= 0xb166)) {
        /* Hurricane3-lite - 56163/4/6: CPU 600MHz, AXI 200MHz, DDR 667MHz */
        ddr_clk = 667;
    } else {
        /* Hurricane3/Buckhound - 56160/2,53440/2/3: CPU 1250MHz, AXI 400MHz, DDR 800MHz */
        ddr_clk = 800;
    }
#elif defined(CONFIG_SABER2)
#if defined(CONFIG_IPROC_EMULATION)
    ddr_clk = 400;
#else
    ddr_clk = 800;
#endif  /* defined(CONFIG_IPROC_EMULATION) */
#elif defined(CONFIG_GREYHOUND2)
    if ((sku_id & 0xfff0) == 0xb070) {
        /* Greyhound2 - 5607x: CPU 1250MHz, AXI 400MHz, DDR 800MHz */
        ddr_clk = 800;
    } else {
        /* Ranger3    - 5357x: CPU 600MHz, AXI 200MHz, DDR 667MHz */
        ddr_clk = 667;
    }
#endif
	return ddr_clk;
#endif
}

#if defined(CONFIG_SHMOO_REUSE) || defined(CONFIG_SHMOO_AND28_REUSE)

#define RAND_MAGIC_1    0x0000444BUL
#define RAND_MAGIC_2    0x88740000UL
#define RAND_MAGIC_3    69069UL
#define RAND_SEED       0x5301beef
#define RAND_SEED_2     ((RAND_SEED << 21) + (RAND_SEED << 14) + (RAND_SEED << 7))
#define RAND_C_INIT     (((RAND_SEED_2 + RAND_MAGIC_1) << 1) + 1)
#define RAND_T_INIT     ((RAND_SEED_2 << (RAND_SEED_2 & 0xF)) + RAND_MAGIC_2)

static int simple_memory_test(void *start, uint32_t len)
{
    register uint32_t rand_c_value, rand_t_value, rand_value;
    register uint32_t i;
    register volatile uint32_t *paddr;
    
    len /= 4;
    paddr = (volatile uint32_t *)start;
    rand_c_value = RAND_C_INIT;
    rand_t_value = RAND_T_INIT;
    for(i=0; i<len; i++, paddr++) {
        rand_c_value *= RAND_MAGIC_3;
        rand_t_value ^= rand_t_value >> 15;
        rand_t_value ^= rand_t_value << 17;
        rand_value = rand_t_value ^ rand_c_value;
        *paddr = rand_value;
    }
    
    paddr = (volatile uint32_t *)start;
    rand_c_value = RAND_C_INIT;
    rand_t_value = RAND_T_INIT;
    for(i=0; i<len; i++, paddr++) {
        rand_c_value *= RAND_MAGIC_3;
        rand_t_value ^= rand_t_value >> 15;
        rand_t_value ^= rand_t_value << 17;
        rand_value = rand_t_value ^ rand_c_value;
        if (*paddr != rand_value) {
            return -1;
        }
    }

    return 0;
}

#endif /* CONFIG_SHMOO_REUSE || CONFIG_SHMOO_AND28_REUSE */

#if defined(CONFIG_RUN_DDR_SHMOO2) && defined(CONFIG_SHMOO_REUSE)

#define SHMOO_HEADER_MAGIC      "SHMO"
#define SHMOO_MIN_BLOCK_SIZE    0x10000

static const uint16_t ddr_phy_ctl_regs[] = {
    0x030,
    0x034,
    0x06c
};

static const uint16_t ddr_phy_wl_regs[] = {
    0x000,
    0x004,
    0x008,
    0x00c,
    0x010,
    0x014,
    0x018,
    0x01c,
    0x020,
    0x024,
    0x028,
    0x02c,
    0x030,
    0x034,
    0x038,
    0x03c,
    0x040,
    0x044,
    0x048,
    0x04c,
    0x050,
    0x054,
    0x058,
    0x05c,
    0x060,
    0x064,
    0x068,
    0x06c,
    0x070,
    0x074,
    0x0a4,
    0x0a8,
    0x0ac,
    0x0b0,
    0x0b4,
    0x0b8,
    0x0bc,
    0x0c0,
    0x0c4,
    0x0c8,
    0x0cc,
    0x0d0,
    0x0d4,
    0x0d8,
    0x0dc,
    0x0e0,
    0x0e4,
    0x0e8,
    0x0ec,
    0x0f0,
    0x0f4,
    0x0f8,
    0x0fc,
    0x100,
    0x104,
    0x108,
    0x10c,
    0x110,
    0x114,
    0x118,
    0x11c,
    0x120,
    0x124,
    0x128,
    0x12c,
    0x130,
    0x134,
    0x138,
    0x13c,
    0x140,
    0x144,
    0x148,
    0x14c,
    0x150,
    0x154,
    0x158,
    0x15c,
    0x160,
    0x164,
    0x168,
    0x16c,
    0x1a0,
    0x1a4,
    0x1a8,
    0x1ac,
    0x1b0
};
#if defined(CONFIG_IPROC_DDR_ECC) && !defined(CONFIG_IPROC_P7)
static const uint16_t ddr_phy_eccl_regs[] = {
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_RD_EN_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_W_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_R_P_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_R_N_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT0_W_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT1_W_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT2_W_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT3_W_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_DM_W_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT0_R_P_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT0_R_N_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT1_R_P_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT1_R_N_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT2_R_P_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT2_R_N_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT3_R_P_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT3_R_N_BASE,
DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT_RD_EN_BASE,
DDR_PHY_ECC_LANE_READ_DATA_DLY_BASE,
DDR_PHY_ECC_LANE_READ_CONTROL_BASE,
DDR_PHY_ECC_LANE_IDLE_PAD_CONTROL_BASE,
DDR_PHY_ECC_LANE_DRIVE_PAD_CTL_BASE,
DDR_PHY_ECC_LANE_WR_PREAMBLE_MODE_BASE,
};
#endif
#if defined(CONFIG_IPROC_NAND) && defined(CONFIG_ENV_IS_IN_NAND) && CONFIG_ENV_IS_IN_NAND

static int write_shmoo_to_flash(void *buf, int length)
{
    nand_info_t *nand;
    int ret = 0;
    uint32_t offset = CONFIG_SHMOO_REUSE_NAND_OFFSET;
    uint32_t end = offset + CONFIG_SHMOO_REUSE_NAND_RANGE;
    uint32_t blksize;
    
    /* Get flash handle */
    nand = &nand_info[0];
    if (nand->size < offset || nand->writesize == 0 || nand->erasesize == 0) {
        printf("Failed to initialize NAND flash for saving Shmoo values!\n");
        return -1;
    }

    /* For NAND with bad blocks, we always erase all blocks in the range */
    {
        nand_erase_options_t opts;
        memset(&opts, 0, sizeof(opts));
        opts.offset = offset / nand->erasesize * nand->erasesize;
        opts.length = (CONFIG_SHMOO_REUSE_NAND_RANGE - 1) / nand->erasesize * nand->erasesize + 1;
        opts.quiet  = 1;
        ret = nand_erase_opts(nand, &opts);
        if (ret) {
            printf("NAND flash erase failed, error=%d\n", ret);
            return ret;
        }
    }
    
    /* Write data */
    blksize = nand->erasesize > SHMOO_MIN_BLOCK_SIZE? 
        nand->erasesize : SHMOO_MIN_BLOCK_SIZE;
    while (offset < end) {
        if (nand_block_isbad(nand, offset)) {
            offset += blksize;
            continue;
        }
        ret = nand_write(nand, offset, (size_t *)&length, (u_char *)buf);
        if (ret) {
            printf("NAND flash write failed, error=%d\n", ret);
        }
        break;
    }

    return ret;
}

#elif defined (CONFIG_SPI_FLASH) && defined(CONFIG_ENV_IS_IN_SPI_FLASH) && CONFIG_ENV_IS_IN_SPI_FLASH

static int write_shmoo_to_flash(void *buf, int length)
{
    struct spi_flash *flash;
    int erase = 0;
    volatile uint32_t *flptr;
    int i, j, ret = 0;
    uint32_t offset = CONFIG_SHMOO_REUSE_QSPI_OFFSET;
    
    /* Check if erasing is required */
    flptr = (volatile uint32_t *)(IPROC_QSPI_MEM_BASE + offset / 4 * 4);    
    j = (length - 1) / 4 + 1;
    for(i=0; i<j; i++, flptr++) {
        if (*flptr != 0xFFFFFFFF) {
            erase = 1;
            break;
        }
    }
    
    /* Probe flash */
    flash = spi_flash_probe(
                CONFIG_ENV_SPI_BUS, 
                CONFIG_ENV_SPI_CS, 
                CONFIG_ENV_SPI_MAX_HZ, 
                CONFIG_ENV_SPI_MODE
                );
    if (!flash) {
        printf("Failed to initialize SPI flash for saving Shmoo values!\n");
        return -1;
    }
    
    /* Erase if necessary */
    if (erase) {
        ret = spi_flash_erase(
                flash, 
                offset / flash->sector_size * flash->sector_size, 
                flash->sector_size
                );
        if (ret) {
            printf("SPI flash erase failed, error=%d\n", ret);
            spi_flash_free(flash);
            return ret;
        }
    }
    
    /* Write data */
    ret = spi_flash_write(flash, offset, length, buf);
    if (ret) {
        printf("SPI flash write failed, error=%d\n", ret);
    }
   
    /* Free flash instance */
    spi_flash_free(flash);
    
    return ret;
}

#elif defined (CONFIG_ENV_IS_IN_FLASH)

static int write_shmoo_to_flash(void *buf, int length)
{
    int erase = 0;
    volatile uint32_t *flptr, shmoo_start;
    int i, j, ret = 0;
    uint32_t offset = CONFIG_SHMOO_REUSE_NOR_OFFSET;
    int sect_len;

    /* Check if erasing is required */
    flptr = (volatile uint32_t *)(IPROC_NOR_MEM_BASE + offset / 4 * 4);
    shmoo_start = flptr;
    j = (length - 1) / 4 + 1;
    for(i=0; i<j; i++, flptr++) {
        if (*flptr != 0xFFFFFFFF) {
            erase = 1;
            break;
        }
    }

    sect_len = (((length / 0x20000) + 1)*0x20000 - 1);   
    /* Erase if necessary */
    if (erase) {
        ret = flash_sect_erase((ulong)shmoo_start, (ulong)shmoo_start + sect_len);
        if (ret) {
            printf("NOR flash erase failed, error=%d, start addr: 0x%x, end addr: 0x%x\n", 
                            ret, (ulong)shmoo_start, (ulong)shmoo_start + sect_len);
            return ret;
        }
    }

    /* Write data */
    ret = flash_write((char *)buf, (ulong)shmoo_start, length);

    if (ret) {
        printf("NOR flash write failed, error=%d\n", ret);
    }


    return ret;

}
#else
 #error Flash (SPI or NAND) must be enabled 
#endif

/* Return flash pointer; or NULL if validation failed */
static volatile uint32_t *validate_flash_shmoo_values(struct shmoo_signature *psig, int *ppairs)
{
	uint32_t dev_id, sku_id, ddr_type, ddr_clk;
    volatile uint32_t *ptr;
    volatile uint32_t *flptr;
    struct shmoo_signature sig;
    uint32_t checksum, pairs, length;
    uint32_t chksum;
    int offset;
    int i;
	int numpairs = 1;

	if (is_ddr_32bit()) {
		numpairs = 2;
	}

    /* Calculate required length (register/value pair) */
    pairs = 
        sizeof(ddr_phy_ctl_regs) / sizeof(ddr_phy_ctl_regs[0]) + 
        sizeof(ddr_phy_wl_regs) / sizeof(ddr_phy_wl_regs[0]) * numpairs;
#ifdef CONFIG_IPROC_DDR_ECC
	pairs += sizeof(ddr_phy_eccl_regs) / sizeof(ddr_phy_eccl_regs[0]);
#endif
        
    if (ppairs != NULL) {
        *ppairs = pairs;
    }
    
#if defined(CONFIG_ENV_IS_IN_NAND) && CONFIG_ENV_IS_IN_NAND
    /* Read SHMOO data from NAND */
    flptr = (volatile uint32_t *)(IPROC_NAND_MEM_BASE + CONFIG_SHMOO_REUSE_NAND_OFFSET);
    offset = (CONFIG_SHMOO_REUSE_NAND_RANGE - 1) / SHMOO_MIN_BLOCK_SIZE * SHMOO_MIN_BLOCK_SIZE;
#elif defined (CONFIG_ENV_IS_IN_FLASH) 
    /* Read SHMOO data from NOR */
    flptr = (volatile uint32_t *)(IPROC_NOR_MEM_BASE + CONFIG_SHMOO_REUSE_NOR_OFFSET);
    offset = 0;
#else
    /* Read SHMOO data from SPI */
    flptr = (volatile uint32_t *)(IPROC_QSPI_MEM_BASE + CONFIG_SHMOO_REUSE_QSPI_OFFSET);
    offset = 0;
#endif

    /* Get chip type and DDR type/clock */
    dev_id = (reg32_read((volatile uint32_t *)ChipcommonA_ChipID)) & 0x0000ffff;
#ifdef CONFIG_HURRICANE2
    sku_id = (reg32_read((volatile uint32_t *)CMIC_DEV_REV_ID)) & 0x0000ffff;
#else
    sku_id = (reg32_read((volatile uint32_t *)ROM_S0_IDM_IO_STATUS) >> 2) & 0x03;
#endif
    ddr_type = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS) & 0x1; 
    ddr_clk = get_ddr_clock(sku_id, ddr_type);

    /* Construct signature */
    memcpy(sig.magic, SHMOO_HEADER_MAGIC, 4);
    sig.dev_id = dev_id;
    sig.sku_id = sku_id;
    sig.ddr_type = ddr_type;
    sig.ddr_clock = ddr_clk;
    
    /* Provide signature data to caller */
    if (psig) {
        memcpy(psig, &sig, sizeof(sig));
    }
    
    /* Check signature (in min-blocks from bottom) */
    while (offset >= 0) {
        ptr = flptr + offset;
        if (!shmoo_sigmemcmp(&sig,(void *)ptr)) {
            break;
        }
        offset -= SHMOO_MIN_BLOCK_SIZE;
    }
    if (offset < 0) {
        printf(" Signature mismatch ");
        return NULL;
    }
    ptr += 3;
    
    /* Verify checksum */
    checksum = *ptr++;
    length = *ptr++;
    if (pairs != length) {
        /* Pair count unmatched */
        printf(" Pair count mismatch pairs %x length %x",pairs, length);
        return NULL;
    }
    chksum = 0;
    for(i=0; i<length * 2; i++, ptr++) {
        chksum += *ptr;
    }
    if (chksum != checksum) {
        printf(" Checksum mismatch cksum: %x checksum:%x",chksum,checksum);
        return NULL;
    }
    
    return flptr + offset;
}

static int try_restore_shmoo(void)
{
    int invalid = 0;
    struct shmoo_signature sig;
    volatile uint32_t *flptr;
    volatile uint32_t *reg;
    uint32_t val;
    int pairs, i;
    
    /* Validate values in flash */
    printf("Validate Shmoo parameters stored in flash ..... ");
    flptr = validate_flash_shmoo_values(&sig, &pairs);
    if (flptr == NULL) {
        printf("failed\n");
        return 1;
    }
    printf("OK\n");

    /* Check if user wants to skip restoring and run Shmoo */
    if (CONFIG_SHMOO_REUSE_DELAY_MSECS > 0) {
        char c = 0;
        unsigned long start;
        printf("Press Ctrl-C to run Shmoo ..... ");
        start = get_timer(0);
        while(get_timer(start) <= CONFIG_SHMOO_REUSE_DELAY_MSECS) {
            if (tstc()) {
                c = getc();
                if (c == 0x03) {
                    printf("Pressed.\n");
                    printf("Do you want to run the Shmoo? [y/N] ");
                    for(;;) {
                        c = getc();
                        if (c == 'y' || c == 'Y') {
                            printf("Y\n");
                            invalid = 1;
                            break;
                        } else if (c == '\r' || c == 'n' || c == 'N') {
                            if (c != '\r')
                                printf("N\n");
                            break;
                        }
                    }
                    break;
                } else {
                    c = 0;
                }
            }
        }
        if (c == 0) 
            printf("skipped\n");
    }
    
    if (invalid) {
        return 1;
    }
    
    /* Restore values from flash */
    printf("Restoring Shmoo parameters from flash ..... ");
    flptr += 5;
    for(i=0; i<pairs; i++) {
        reg = (uint32_t *)(*flptr++);
        val = (uint32_t *)(*flptr++);
	if( (((uint32_t)reg >= DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE_RD_EN) && ((uint32_t)reg <= (DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE_RD_EN + 0x114)))
#if (CONFIG_SHMOO_REUSE_DDR_32BIT)
		|| (((uint32_t)reg >= DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE_RD_EN) && ((uint32_t)reg <= (DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE_RD_EN + 0x114)))
#endif
#ifdef CONFIG_IPROC_DDR_ECC
		|| (((uint32_t)reg >= (DDR_DENALI_CTL_00 + DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_RD_EN_BASE)) && ((uint32_t)reg <= (DDR_DENALI_CTL_00 + DDR_PHY_ECC_LANE_VDL_OVRIDE_BYTE_BIT_RD_EN_BASE)))
#endif
	) {
		val |= (1 << 17); /* Force Override */
	}
        // printf("Writing 0x%x to 0x%x\n",val,reg);
        reg32_write(reg,val);

        reg32_read(reg); /* Dummy read back */
    }
    printf("done\n");
    
    /* Perform memory test to see if the parameters work */
    if (CONFIG_SHMOO_REUSE_MEMTEST_LENGTH > 0 ) {
        printf("Running simple memory test ..... ");
        i = simple_memory_test(
            (void *)CONFIG_SHMOO_REUSE_MEMTEST_START,
            CONFIG_SHMOO_REUSE_MEMTEST_LENGTH);
        if (i) {
            printf("failed!\n");
            return 1;
        }
        printf("OK\n");
    }
    
    return 0;
}

void iproc_save_shmoo_values(void)
{
    uint32_t *buffer, *ptr;
    volatile uint32_t *flptr;
    uint32_t reg, val;
    struct shmoo_signature sig;
    int pairs, length;
    uint32_t chksum;
    int i;

    /* Check if flash already contains valid data  */
    flptr = validate_flash_shmoo_values(&sig, &pairs);
    if (flptr != NULL) {
        /* Check if the flash data are the same as current DDR PHY values */
        flptr += 5;
	for(i=0; i<pairs; i++) {
	    reg = *flptr++;
	    val = *flptr++;
            if (val != reg32_read(reg)) {
                break;
            }

	}
	if (i == pairs) {
		/* No difference found; Saving skipped */
	    return;
	}
    }
    
    /* Calculate size of buffer */
    length = 12 + 
        sizeof(uint32_t) * 2 + 
        sizeof(uint32_t) * pairs * 2;
        
    /* Allocate buffer */
    buffer = malloc(length);
    if (buffer == NULL) {
        printf("Error allocating memory for saving Shmoo values!\n");
        return;
    }
    ptr = buffer;
    
    /* Fill signature */
    shmoo_sig2mem(&sig,ptr);
    ptr += 5;
    
    /* Copy registers and values to buffer */
    chksum = 0;
    for(i=0; i<sizeof(ddr_phy_ctl_regs) / sizeof(ddr_phy_ctl_regs[0]); i++) {
        reg = (uint32_t)DDR_PHY_CONTROL_REGS_REVISION + ddr_phy_ctl_regs[i];
        *ptr++ = reg;
        chksum += reg;
        // val = *(volatile uint32_t *)reg;
        val = reg32_read((volatile uint32_t *)reg);
        *ptr++ = val;
        chksum += val;
    }
    for(i=0; i<sizeof(ddr_phy_wl_regs) / sizeof(ddr_phy_wl_regs[0]); i++) {
        reg = (uint32_t)DDR_PHY_WORD_LANE_0_VDL_OVRIDE_BYTE_RD_EN + ddr_phy_wl_regs[i];
        *ptr++ = reg;
        chksum += reg;
        // val = *(volatile uint32_t *)reg;
        val = reg32_read((volatile uint32_t *)reg);
	*ptr++ = val;
        chksum += val;
    }
#if (CONFIG_SHMOO_REUSE_DDR_32BIT)
	if (is_ddr_32bit()) {
	    for(i=0; i<sizeof(ddr_phy_wl_regs) / sizeof(ddr_phy_wl_regs[0]); i++) {
	        reg = (uint32_t)DDR_PHY_WORD_LANE_1_VDL_OVRIDE_BYTE_RD_EN + ddr_phy_wl_regs[i];
	        *ptr++ = reg;
	        chksum += reg;
	        // val = *(volatile uint32_t *)reg;
		val = reg32_read((volatile uint32_t *)reg);
		*ptr++ = val;
	        chksum += val;
	    }
	}
#endif /* (CONFIG_SHMOO_REUSE_DDR_32BIT) */
#ifdef CONFIG_IPROC_DDR_ECC
    for(i=0; i<sizeof(ddr_phy_eccl_regs) / sizeof(ddr_phy_eccl_regs[0]); i++) {
        reg = (uint32_t)DDR_DENALI_CTL_00 + ddr_phy_eccl_regs[i];
        *ptr++ = reg;
        chksum += reg;
        // val = *(volatile uint32_t *)reg;
        val = reg32_read((volatile uint32_t *)reg);
	*ptr++ = val;
        chksum += val;
    }
#endif

    /* Fill checksum and length */
    buffer[3] = chksum;
    buffer[4] = pairs;
    
    /* Write to flash */
    printf("Writing Shmoo values into flash .....\n");
    i = write_shmoo_to_flash(buffer, length);
    
    /* Free buffer */
    free(buffer);
}

#endif /* CONFIG_RUN_DDR_SHMOO2 && CONFIG_SHMOO_REUSE */

#ifdef CONFIG_RUN_DDR_SHMOO2
#include <asm/iproc-common/ddr40.h>
#include <asm/iproc-common/shmoo_ddr40.h>
#include <asm/iproc-common/ddr_bist.h>
#endif
#include "shmoo_and28/shmoo_and28.h"

#ifdef CONFIG_IPROC_DDR_ECC
static int clear_ddr(uint32_t offset, uint32_t size)
{
	unsigned long start;
	unsigned int i, val;

    reg32_write((uint32_t *)DDR_BistConfig,reg32_read((uint32_t *)DDR_BistConfig) & ~0x1);

    for( i = 0; i < 1000; i++);

#if !defined(CONFIG_IPROC_P7)
	reg32_write((volatile uint32_t *)DDR_DENALI_CTL_213, 0x00FFFFFF);
#endif

	reg32_write((volatile uint32_t *)DDR_BistConfig, 0x00000002);
	reg32_write((volatile uint32_t *)DDR_BistConfig, 0x00000003);
	reg32_write((volatile uint32_t *)DDR_BistConfig, 0x0000C003);
	reg32_write((volatile uint32_t *)DDR_BistGeneralConfigurations, 0x00000020);

	val =  255 << DDR_BistConfigurations__WriteWeight_R | 
		  0 << DDR_BistConfigurations__ReadWeight_R |
		  1 << DDR_BistConfigurations__ConsAddr8Banks;

	reg32_write((volatile uint32_t *)DDR_BistConfigurations, val);


	reg32_write((volatile uint32_t *)DDR_BistStartAddress, offset);
	reg32_write((volatile uint32_t *)DDR_BistEndAddress, (1 << DDR_BistEndAddress__BistEndAddress_WIDTH) - 1);
	reg32_write((volatile uint32_t *)DDR_BistNumberOfActions, (size + 31) / 32);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord0, 0);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord1, 0);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord2, 0);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord3, 0);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord4, 0);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord5, 0);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord6, 0);
	reg32_write((volatile uint32_t *)DDR_BistPatternWord7, 0);

	reg32_set_bits((volatile uint32_t *)DDR_BistConfigurations, 1 << DDR_BistConfigurations__IndWrRdAddrMode);
	
	reg32_set_bits((volatile uint32_t *)DDR_BistConfigurations, 1 << DDR_BistConfigurations__BistEn);

	start = get_timer(0);
	while(get_timer(start) <= 10000) {
		if(reg32_read((volatile uint32_t *)DDR_BistStatuses) & (1 << DDR_BistStatuses__BistFinished))
			break;
	}
	/* Clear BIST_EN bit */
	reg32_clear_bits((volatile uint32_t *)DDR_BistConfigurations, 1 << DDR_BistConfigurations__BistEn);

	if((get_timer(start) <= 10000)  &&
	   (!reg32_read((volatile uint32_t *)DDR_BistErrorOccurred)))
	{
		printf("clear_ddr: OK\n");
		return(0);
	}
	printf("clear_ddr: Failed: 0x%lx\n", get_timer(start));
	if(reg32_read((volatile uint32_t *)DDR_BistErrorOccurred))
		printf("clear_ddr: Error occured\n");
	return(1);
}
#endif /* CONFIG_IPROC_DDR_ECC */

#if defined(CONFIG_SHMOO_AND28_REUSE)
extern void restore_shmoo_config(and28_shmoo_config_param_t *shmoo_control_para);
#endif

void ddr_init2(void)
{
	int i;
	volatile unsigned int val;
	int ddr_type;
	uint32_t status, sku_id, ddr_clk, dev_id;
	uint32_t unit = 0;
	uint32_t skip_shmoo = 0;

#if (defined(CONFIG_GREYHOUND) || defined(CONFIG_SABER2) || defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2))
	and28_shmoo_dram_info_t sdi;
	and28_shmoo_config_param_t config_param;
#endif

	dev_id = (reg32_read((volatile uint32_t *)ICFG_CHIP_ID_REG)) & 0x0000ffff;
	sku_id = (reg32_read((volatile uint32_t *)CMIC_DEV_REV_ID)) & 0x0000ffff;
	/* See if it is KATANA2, KATANA2 doesn't have right chip ID in ChipcommonA_ChipID */
	if(((sku_id & 0xfff0) == 0xa450) || ((sku_id & 0xfff0) == 0xb450) || sku_id == 0xb248) {
		dev_id = 56450; /* KATANA2 */
	}

	printf("DEV ID = 0x%x\n", dev_id);
	printf("SKU ID = 0x%x\n", sku_id);

#if defined(CONFIG_IPROC_P7)

#if defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2)
  /* For HR3, ddr_type = 0: DDR3;  1: DDR4;  2: LPDDR2 (Actually LPDDR2 is not supported in HR3) */
  //val = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS) & 0x3;    
  val = (reg32_read((volatile uint32_t *)DMU_PCU_IPROC_STRAPS_CAPTURED) >>
                      DMU_PCU_IPROC_STRAPS_CAPTURED__strap_ddr_type_R) & 0x3;
  if ((val != 0) && (val != 1)) {
     printf("Unsupported DDR type: %d\n", val);
     return;
  }
  /* To comply with other chips, DDR3 : 1, DDR4 : 0 */
  ddr_type = (val == 1)? 0:1;
  
#else
	val = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS) & 0x3;
	if (val == 0) {
		ddr_type = 1;
	} else if (val == 1) {
		ddr_type = 2;
	} else {
		printf("Unsupported DDR type: %d\n", val);
		return;
	}
#endif  /* defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2) */

    printf("DDR type: DDR%d\n", (ddr_type == 1)? 3 : 4);
#else
	ddr_type = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS) & 0x1; 
	printf("DDR type: DDR%d\n", (ddr_type) ? 3 : 2);
#endif /* defined(CONFIG_IPROC_P7) */
    
	ddr_clk = get_ddr_clock(sku_id, ddr_type);
	printf("MEMC 0 DDR speed = %dMHz\n", ddr_clk);

	status = change_ddr_clock(ddr_clk);
	if(status) {
		printf("CRU LCPLL configuratioin failed\n");
		return;
	}

#if defined(CONFIG_IPROC_P7)
	val = reg32_read((volatile uint32_t *)CRU_ddrphy_pwr_ctrl);
	
	/* assert power ON */
	val |= 1 << CRU_ddrphy_pwr_ctrl__i_pwronin_phy;
	reg32_write((volatile uint32_t *)CRU_ddrphy_pwr_ctrl, val);
	
	/* assert power OK */
	__udelay(10);
	val |= 1 << CRU_ddrphy_pwr_ctrl__i_pwrokin_phy;
	reg32_write((volatile uint32_t *)CRU_ddrphy_pwr_ctrl, val);
	
	/* remove DFI isolation */
	__udelay(150);
	val &= ~(1 << CRU_ddrphy_pwr_ctrl__i_iso_phy_dfi);
	reg32_write((volatile uint32_t *)CRU_ddrphy_pwr_ctrl, val);

	/* remove PHY register isolation */
	val &= ~(1 << CRU_ddrphy_pwr_ctrl__i_iso_phy_regs);
	reg32_write((volatile uint32_t *)CRU_ddrphy_pwr_ctrl, val);
	
	/* remove PLL isolation */
	val &= ~(1 << CRU_ddrphy_pwr_ctrl__i_iso_phy_pll);
	reg32_write((volatile uint32_t *)CRU_ddrphy_pwr_ctrl, val);
	
	/* de-assert reset */
	__udelay(200);
	val |= 1 << CRU_ddrphy_pwr_ctrl__i_hw_reset_n;
	reg32_write((volatile uint32_t *)CRU_ddrphy_pwr_ctrl, val);

	/* Wait for PHY power up */
	for(i=0; i < 0x19000; i++) {
		val = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS);
		if((val & (1 << DDR_S1_IDM_IO_STATUS__o_phy_pwrup_rsb)))
			break;
	}
	if(i == 0x19000) {
		printf("DDR PHY not power up\n");
		return;
	}
#endif /* defined(CONFIG_IPROC_P7) */

#if defined(CONFIG_IPROC_P7)
	/* Get the DDR S1 and S2 out of reset */
	reg32_write((volatile uint32_t *)DDR_S1_IDM_RESET_CONTROL, 0);
	reg32_write((volatile uint32_t *)DDR_S2_IDM_RESET_CONTROL, 0);

	__udelay(1000);
#if !defined(CONFIG_HURRICANE3)
	reg32_write((volatile uint32_t *)DDR_S0_IDM_RESET_CONTROL, 0);
#endif
#if !(defined(CONFIG_SABER2) || defined(CONFIG_HURRICANE3))
	/* Set the ddr_ck to 400 MHz, 2x memc clock */
	reg32_write_masked((volatile uint32_t *)DDR_S1_IDM_IO_CONTROL_DIRECT, 0xfff << 16, /*ddr_clk*/ 0x190 << 16);
#endif
#else
    reg32_write((volatile uint32_t *)DDR_S1_IDM_RESET_CONTROL, 0);
    reg32_write((volatile uint32_t *)DDR_S2_IDM_RESET_CONTROL, 0);
    /* Set the ddr_ck to 400 MHz, 2x memc clock */
    reg32_write_masked((volatile uint32_t *)DDR_S1_IDM_IO_CONTROL_DIRECT, 0xfff << 16, /*ddr_clk*/ 0x190 << 16);
#endif /* defined(CONFIG_IPROC_P7) */

#if defined(CONFIG_IPROC_P7)
	if (is_ddr_32bit()) {
		reg32_write_masked(
			(volatile uint32_t *)DDR_S2_IDM_IO_CONTROL_DIRECT, 
				1 << DDR_S2_IDM_IO_CONTROL_DIRECT__mode_32b,
				1 << DDR_S2_IDM_IO_CONTROL_DIRECT__mode_32b
				);
	}

#if !defined(CONFIG_SABER2) && !defined(CONFIG_HURRICANE3) && !defined(CONFIG_GREYHOUND2)
	/* Wait for PHY ready */
    /* Saber2 has phy ready only after phy pll is locked */
	for(i=0; i < 0x19000; i++) {
		val = reg32_read((volatile uint32_t *)DDR_S1_IDM_IO_STATUS);
		if((val & (1 << DDR_S1_IDM_IO_STATUS__o_phy_ready)))
			break; /* DDR PHY is up */
	}

	if(i == 0x19000) {
		printf("DDR PLL not locked\n");
		return;
	}

	/* Get the DDR S0 out of reset */
	reg32_write((volatile uint32_t *)DDR_S0_IDM_RESET_CONTROL, 0);
#endif /* !defined(CONFIG_SABER2) */
#endif /* defined(CONFIG_IPROC_P7) */

	/* Wait for DDR PHY up */
	for(i=0; i < 0x19000; i++) {
		val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
		if( val != 0) {
            printf("PHY revision version: 0x%08x\n", val);
			break; /* DDR PHY is up */
        }
	}
	
	if(i == 0x19000) {
		printf("DDR PHY is not up\n");
		return;
	}
#if (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2))
	/* mhz = 0x190, 32 bit bus, 16 bit chip width, 1.35v vddq, 8Gb chip size, jedec type = 25 (DDR3-1866M) */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL, 0x0190c4f3);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

	/* ECC =1, DRAM type = 1 i.e DDR3, AL = 3, write recovery = 0, write latency = 9, CAS read latency = 13 */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL2, 0x0036048d);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif /* (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2)) */
#if (defined(CONFIG_HURRICANE2))
	/* mhz = 0x190, 16 bit bus, 16 bit chip width, 1.35v vddq, 8Gb chip size, jedec type = 25 (DDR3-1866M) */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL, 0x0190d4f3);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);

	/* DRAM type = 1 i.e DDR3, AL = 3, write recovery = 0, write latency = 9, CAS read latency = 13 */
	reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_STRAP_CONTROL2, 0x0016048d);
	val = reg32_read((volatile uint32_t *)DDR_PHY_CONTROL_REGS_REVISION);
#endif /* (defined(CONFIG_HURRICANE2)) */

#if (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2))
	printf("ddr_init2: Calling soc_ddr40_set_shmoo_dram_config\n");
	soc_ddr40_set_shmoo_dram_config(unit, ((uint32) 0xFFFFFFFF) >> (32 - 2));
#elif (defined(CONFIG_HURRICANE2))
	printf("ddr_init2: Calling soc_ddr40_set_shmoo_dram_config\n");
	soc_ddr40_set_shmoo_dram_config(unit, ((uint32) 0xFFFFFFFF) >> (32 - 1));
#elif (defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3) || defined(CONFIG_SABER2) || defined(CONFIG_GREYHOUND2))
#if defined(CONFIG_GREYHOUND)
    /* Ki/Kp = 2/3 is the recommended setting */
    reg32_write((volatile uint32_t *)DDR_PHY_CONTROL_REGS_PLL_CONTROL1, 0xD0);
#endif
	printf("ddr_init2: Calling soc_and28_shmoo_dram_info_set\n");
	sdi.data_rate_mbps = (ddr_clk == 667) ? 1333 : (ddr_clk * 2);
	sdi.ref_clk_mhz = 50;
	sdi.ctl_type = SHMOO_AND28_CTL_TYPE_1;
#ifdef CONFIG_DRAM_TYPE_DDR4
    sdi.dram_type = SHMOO_AND28_DRAM_TYPE_DDR4;
#else
	sdi.dram_type = SHMOO_AND28_DRAM_TYPE_DDR3;
#endif
	sdi.dram_bitmap = 0x00000001;

	if (is_ddr_32bit()) {
		sdi.interface_bitwidth = 32;
	} else {
  #if defined(CONFIG_IPROC_DDR_ECC)
		/* Initialize 32-bit PHY for 16+2 bits with ECC */
		sdi.interface_bitwidth = 32;
  #else
		sdi.interface_bitwidth = 16;
  #endif /* defined(CONFIG_IPROC_DDR_ECC) */
	}

	sdi.num_columns = (1 << CONFIG_DDR_COL_BITS);
	sdi.num_rows = (1 << CONFIG_DDR_ROW_BITS);
	sdi.num_banks = (1 << CONFIG_DDR_BANK_BITS);
#ifdef CONFIG_DRAM_TYPE_DDR4
    sdi.num_bank_groups = (1 << CONFIG_DDR_BANK_GROUP_BITS);
#else
    sdi.num_bank_groups = (1 << 1);
#endif
	sdi.refi = 7800;
	sdi.command_parity_latency = 0;
	sdi.sim_system_mode = 0;
    
	soc_and28_shmoo_dram_info_set(unit, &sdi);
#else
#error "DRAM config is not set"
#endif /* (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2)) */

#if (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2) || defined(CONFIG_HURRICANE2))
	printf("ddr_init2: Calling soc_ddr40_phy_calibrate\n");
	if(soc_ddr40_phy_calibrate(unit, 0, DDR_PHYTYPE_ENG, 0) != SOC_E_NONE) {

		printf("DDR phy calibration failed\n");
		return;
	} else {
		printf("DDR phy calibration passed\n");
    }
#else
	if(!skip_shmoo)
	{
		printf("ddr_init2: Calling soc_and28_shmoo_phy_init\n");
		if(soc_and28_shmoo_phy_init(unit, 0) != SOC_E_NONE) {

			printf("DDR PHY initialization failed\n");
			return;
		}
	}
	else
	{
		printf("DeepSleep wakeup: ddr init bypassed 2\n");
	}
#endif /* (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2) || defined(CONFIG_HURRICANE2)) */

#ifdef CONFIG_RUN_DDR_SHMOO
	printf("PHY register dump after DDR PHY init\n");
	dump_phy_regs();
#endif

	printf("Programming controller register\n");
	ddr_init_regs(ddr_init_tab);

	if(ddr_type) {
		/* DDR3 */	
#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) || defined(CONFIG_KATANA2) \
	|| defined(CONFIG_GREYHOUND) || defined(CONFIG_SABER2) || defined(CONFIG_HURRICANE3) \
    || defined(CONFIG_GREYHOUND2))
		switch(ddr_clk) {
			case 333:
				ddr_init_regs(ddr3_init_tab_667);
				break;
			case 400:
				ddr_init_regs(ddr3_init_tab_800);
				break;
/*
			case 467:
				ddr_init_regs(ddr3_init_tab_933);
				break;
*/
			case 533:
				ddr_init_regs(ddr3_init_tab_1066);
				break;
			case 667:
				ddr_init_regs(ddr3_init_tab_1333);
				break;
			case 750:
			case 800:
				ddr_init_regs(ddr3_init_tab_1600);
				break;
		}
#else
		ddr_init_regs(ddr3_init_tab_667);
#endif	/* (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) || defined(CONFIG_KATANA2) \ */
		/* 	|| defined(CONFIG_GREYHOUND))	                                                     */
	}
	else {
#if (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) || defined(CONFIG_KATANA2))
		/* DDR2 */	
		switch(ddr_clk) {
			case 333:
				ddr_init_regs(ddr2_init_tab_667);
				break;
			case 400:
				ddr_init_regs(ddr2_init_tab_800);
				break;
			case 533:
				ddr_init_regs(ddr2_init_tab_1066);
				break;
		}
#elif defined(CONFIG_HURRICANE3) || defined(CONFIG_GREYHOUND2)
    /* DDR4 */
    printf("Enabling mt40a512m8_093_1600\n");
    ddr_init_regs(mt40a512m8_093_1600);
#else
		ddr_init_regs(ddr2_init_tab);
#endif	/* (defined(CONFIG_HELIX4) || defined(CONFIG_HURRICANE2) || defined(CONFIG_KATANA2)) */
	}
#if defined(CONFIG_HELIX4)
	if(ddr_clk == 750) {
		val = reg32_read((volatile uint32_t *)DDR_DENALI_CTL_22) & 0x3FFF; //bits 13:0 TREF_F1
		val = val * 750 / 800;
		reg32_write_masked((volatile uint32_t *)DDR_DENALI_CTL_22, 0x3fff, val);     
	}
#endif /* defined(CONFIG_HELIX4) */

	/* Start the DDR */
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_00, 0x01);


#if (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2) || defined(CONFIG_HURRICANE2))

	while(!(reg32_read((volatile uint32_t *)DDR_DENALI_CTL_89) & 0x100));
	printf("ddr_init2: MemC initialization complete\n");
	
	/* Srini - Write to DDR controller registers */
	reg32_write((unsigned int *)0x18010354, 0x00FFFFFF);
	reg32_write((unsigned int *)0x18010400, 0x00000002);
	reg32_write((unsigned int *)0x18010400, 0x00000003);
	reg32_write((unsigned int *)0x18010400, 0x0000C003);
	reg32_write((unsigned int *)0x18010408, 0x00000020);
    
  #ifdef CONFIG_SHMOO_REUSE
	if (try_restore_shmoo()) 
  #endif /* CONFIG_SHMOO_REUSE */
	{
		printf("ddr_init2: Calling soc_ddr40_shmoo_ctl\n");
		soc_ddr40_shmoo_ctl(unit, 0, 2, 2, 0, 1);
	}
#else
	if(!skip_shmoo)
	{
		while(!(reg32_read((volatile uint32_t *)DDR_DENALI_CTL_175) & 0x100));
		printf("ddr_init2: MemC initialization complete\n");

		reg32_set_bits((unsigned int *)DDR_DENALI_CTL_177, 0x00100);
		reg32_write((unsigned int *)DDR_BistConfig, 0x00000002);
		reg32_write((unsigned int *)DDR_BistConfig, 0x00000003);
		reg32_write((unsigned int *)DDR_BistConfig, 0x0000C003);
		reg32_write((unsigned int *)DDR_BistGeneralConfigurations, 0x00000020);

		printf("ddr_init2: Calling soc_and28_shmoo_ctl\n");
  #if defined(CONFIG_SHMOO_AND28_REUSE)
		if (is_shmoo_data_valid()) {
			restore_shmoo_config(&config_param);
			soc_and28_shmoo_ctl(unit, 0, SHMOO_AND28_SHMOO_RSVP, 0, 1, SHMOO_AND28_ACTION_RESTORE, &config_param);
			
    #if defined(CONFIG_SHMOO_REUSE_MEMTEST_LENGTH)
			/* Perform memory test to see if the stored SHMMO values work */
			if (CONFIG_SHMOO_REUSE_MEMTEST_LENGTH > 0) {
				/* Release DDR to AXI for memory testing */
				reg32_clear_bits((volatile uint32_t *)DDR_BistConfig, 1 << DDR_BistConfig__axi_port_sel);

				printf("Running simple memory test ..... ");
				i = simple_memory_test(
					(void *)CONFIG_SHMOO_REUSE_MEMTEST_START,
					CONFIG_SHMOO_REUSE_MEMTEST_LENGTH);
				if (i) {
					printf("failed!\n");
					
					/* Connect DDR controller to BIST for SHMOO */
					reg32_set_bits((volatile uint32_t *)DDR_BistConfig, 1 << DDR_BistConfig__axi_port_sel);
					
					/* Perform full SHMOO since stored values don't work */
					soc_and28_shmoo_ctl(unit, 0, SHMOO_AND28_SHMOO_RSVP, 0, 1, SHMOO_AND28_ACTION_RUN, &config_param);
				} else {
					printf("OK\n");
				}
			}
    #endif /* defined(CONFIG_SHMOO_REUSE_MEMTEST_LENGTH) */
			
		} else {
        /* Connect DDR controller to BIST for SHMOO */
        reg32_set_bits((volatile uint32_t *)DDR_BistConfig, 1 << DDR_BistConfig__axi_port_sel);
			  soc_and28_shmoo_ctl(unit, 0, SHMOO_AND28_SHMOO_RSVP, 0, 1, SHMOO_AND28_ACTION_RUN, &config_param);
		}
  #else
		soc_and28_shmoo_ctl(unit, 0, SHMOO_AND28_SHMOO_RSVP, 0, 1, SHMOO_AND28_ACTION_RUN, &config_param);
  #endif /* CONFIG_SHMOO_AND28_REUSE */
	}
#endif /* (defined(CONFIG_HELIX4) || defined(CONFIG_KATANA2) || defined(CONFIG_HURRICANE2)) */
	else
	{
		printf("DeepSleep wakeup: ddr init bypassed 3\n");
	}

#if defined(CONFIG_IPROC_P7) && defined(CONFIG_IPROC_DDR_ECC)
	printf("Enabling DDR ECC correcting and reporting\n");
    
	/* Clear DDR ECC interrupts if any */
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_177, 
		DDR_DENALI_CTL_177_ECC_MASK);

	/* Disable auto corruption */
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_148, 
		1 << DDR_DENALI_CTL_148__ECC_DISABLE_W_UC_ERR);

	/* Enable ECC correction and reporting */
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_146,
		1 << DDR_DENALI_CTL_146__ECC_EN);

	/* Initialize DDR so that uninitialized reads won't report ecc error */
	clear_ddr(0, CONFIG_PHYS_SDRAM_1_SIZE);
#elif defined(CONFIG_IPROC_DDR_ECC)
	printf("Enabling DDR ECC reporting\n");
	/* Clear DDR interrupts if any */
	*(unsigned int *)(DDR_DENALI_CTL_213) = 0x00FFFFFF;
	__udelay(1000);
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_67, 0x01); /* Disable auto correction */
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_66, 0x01); /* Enable ECC */
    
	clear_ddr(0, CONFIG_PHYS_SDRAM_1_SIZE);
	printf("Enabling DDR ECC correction\n");
	reg32_set_bits((volatile uint32_t *)DDR_DENALI_CTL_66, 1 << 1); /* Enable ECC correction */
#endif /* defined(CONFIG_IPROC_P7) && defined(CONFIG_IPROC_DDR_ECC) */
	/* Release DDR slave port to AXI */
	reg32_clear_bits((volatile uint32_t *)DDR_BistConfig, 1 << DDR_BistConfig__axi_port_sel);
	printf("DDR Interface Ready\n");

	/* dump_phy_regs(); */
}
