/*
 * Copyright 2014 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/iproc-common/armpll.h>
#include <asm/iproc-common/sysmap.h>
#include <asm/iproc-common/reg_utils.h>
#include <asm/arch/socregs.h>

#define NELEMS(x)	(sizeof(x) / sizeof(x[0]))

struct armpll_parameters {
	unsigned int mode;
	unsigned int ndiv_int;
	unsigned int ndiv_frac;
	unsigned int pdiv;
	unsigned int freqid;
};

struct armpll_parameters armpll_clk_tab[] = {
	{   25, 64,      1, 1, 0},
	{  100, 64,      1, 1, 2},
	{  400, 64,      1, 1, 6},
	{  448, 71, 713050, 1, 6},
	{  500, 80,      1, 1, 6},
	{  560, 89, 629145, 1, 6},
	{  600, 96,      1, 1, 6},
	{  800, 64,      1, 1, 7},
	{  896, 71, 713050, 1, 7},
	{ 1000, 80,      1, 1, 7},
	{ 1100, 88,      1, 1, 7},
	{ 1120, 89, 629145, 1, 7},
	{ 1200, 96,      1, 1, 7},
};

uint32_t armpll_config(uint32_t clkmhz)
{
	uint32_t freqid;
	uint32_t ndiv_frac;
	uint32_t pll;
	uint32_t status = 1;
	uint32_t timeout_countdown;
	int i;

	for (i = 0; i < NELEMS(armpll_clk_tab); i++) {
		if (armpll_clk_tab[i].mode == clkmhz) {
			status = 0;
			break;
		}
	}

	if (status) {
		printf("Error: Clock configuration not supported\n");
		goto armpll_config_done;
	}

	/* Enable write access */
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_WR_ACCESS, IPROC_REG_WRITE_ACCESS);

	if (clkmhz == 25)
		freqid = 0;
	else
		freqid = 2;

	/* Bypass ARM clock and run on sysclk */
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_FREQ,
			1 << IHOST_PROC_CLK_POLICY_FREQ__PRIV_ACCESS_MODE |
	       freqid << IHOST_PROC_CLK_POLICY_FREQ__POLICY3_FREQ_R |
	       freqid << IHOST_PROC_CLK_POLICY_FREQ__POLICY2_FREQ_R |
	       freqid << IHOST_PROC_CLK_POLICY_FREQ__POLICY1_FREQ_R |
	       freqid << IHOST_PROC_CLK_POLICY_FREQ__POLICY0_FREQ_R);

	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL,
			1 << IHOST_PROC_CLK_POLICY_CTL__GO |
			1 << IHOST_PROC_CLK_POLICY_CTL__GO_AC);

	/* Poll CCU until operation complete */
	timeout_countdown = 0x100000;
	while (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL) &
	       (1 << IHOST_PROC_CLK_POLICY_CTL__GO)) {
		timeout_countdown--;
		if (timeout_countdown == 0) {
			printf("CCU polling timedout\n");
			status = 1;
			goto armpll_config_done;
		}
	}

	if (clkmhz == 25 || clkmhz == 100) {
		status = 0;
		goto armpll_config_done;
	}

	/* Now it is safe to program the PLL */
	pll = reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMB);
	pll &= ~((1 << IHOST_PROC_CLK_PLLARMB__PLLARM_NDIV_FRAC_WIDTH) - 1);
	ndiv_frac =
		((1 << IHOST_PROC_CLK_PLLARMB__PLLARM_NDIV_FRAC_WIDTH) - 1) &
		 (armpll_clk_tab[i].ndiv_frac <<
		 IHOST_PROC_CLK_PLLARMB__PLLARM_NDIV_FRAC_R);
	pll |= ndiv_frac;
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMB, pll);

	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA, 
	       armpll_clk_tab[i].ndiv_int <<
			IHOST_PROC_CLK_PLLARMA__PLLARM_NDIV_INT_R |
	       armpll_clk_tab[i].pdiv <<
			IHOST_PROC_CLK_PLLARMA__PLLARM_PDIV_R |
	       1 << IHOST_PROC_CLK_PLLARMA__PLLARM_SOFT_RESETB);

	/* Poll ARM PLL Lock until operation complete */
	timeout_countdown = 0x100000;
	while (!(reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA) &
	       (1 << IHOST_PROC_CLK_PLLARMA__PLLARM_LOCK))) {
		timeout_countdown--;
		if (timeout_countdown == 0) {
			printf("ARM PLL lock failed\n");
			status = 1;
			goto armpll_config_done;
		}
	}

	pll = reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA);
	pll |= (1 << IHOST_PROC_CLK_PLLARMA__PLLARM_SOFT_POST_RESETB);
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA, pll);

	/* Set the policy */
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_FREQ,
			1 << IHOST_PROC_CLK_POLICY_FREQ__PRIV_ACCESS_MODE |
	       armpll_clk_tab[i].freqid <<
			IHOST_PROC_CLK_POLICY_FREQ__POLICY3_FREQ_R |
	       armpll_clk_tab[i].freqid <<
			IHOST_PROC_CLK_POLICY_FREQ__POLICY2_FREQ_R |
	       armpll_clk_tab[i].freqid <<
			IHOST_PROC_CLK_POLICY_FREQ__POLICY1_FREQ_R |
	       armpll_clk_tab[i].freqid <<
			IHOST_PROC_CLK_POLICY_FREQ__POLICY0_FREQ_R);

	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_CORE0_CLKGATE, IPROC_CLKCT_HDELAY_SW_EN);
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_CORE1_CLKGATE, IPROC_CLKCT_HDELAY_SW_EN);
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_ARM_SWITCH_CLKGATE, IPROC_CLKCT_HDELAY_SW_EN);
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_ARM_PERIPH_CLKGATE, IPROC_CLKCT_HDELAY_SW_EN);
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_APB0_CLKGATE, IPROC_CLKCT_HDELAY_SW_EN);

    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL,
			1 << IHOST_PROC_CLK_POLICY_CTL__GO |
			1 << IHOST_PROC_CLK_POLICY_CTL__GO_AC);

	/* Poll CCU until operation complete */
	timeout_countdown = 0x100000;
	while (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL) &
	       (1 << IHOST_PROC_CLK_POLICY_CTL__GO)) {
		timeout_countdown--;
		if (timeout_countdown == 0) {
			printf("CCU polling failed\n");
			status = 1;
			goto armpll_config_done;
		}
	}

	status = 0;
armpll_config_done:
	/* Disable access to PLL registers */
	reg32_write((volatile uint32_t *)IHOST_PROC_CLK_WR_ACCESS, 0);

	return status;
}
#if defined(CONFIG_IPROC_P7)
#define ARM_FREQ_1500 1500
#define ARM_FREQ_1250 1250
#define ARM_FREQ_1000 1000
#define ARM_FREQ_800  800
#define ARM_FREQ_600  600
#define ARM_FREQ_400  400
#define ARM_FREQ_200  200

uint32_t armpll_config_p7(uint32_t mode)
{
    int i_loop  ;
    uint32_t pdiv = 1;

#if defined(CONFIG_GREYHOUND) || defined(CONFIG_HURRICANE3)
    if (reg32_read((volatile uint32_t *)IPROC_WRAP_TOP_STRAP_STATUS_1) & 0x01) {
        /* 50 MHz */
        pdiv = 2;
    }
#endif

    /* Before PLL locking change the Frequency ID to 2 'default' */

    mb();
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_WR_ACCESS, 0xA5A501);    /* Write KPROC_CLK_MGR_REG_WR_ACCESS = 32'h00A5A501 to enable clk manager access. */
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_FREQ, 0x82020202); /* Select the frequency ID =2 for all policies */
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL , (1 << IHOST_PROC_CLK_POLICY_CTL__GO) | ( 1 << IHOST_PROC_CLK_POLICY_CTL__GO_AC)); 
    /* Set the GO and GO_AC bit */
    while ((reg32_read((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL) & (1 << IHOST_PROC_CLK_POLICY_CTL__GO) ) != 0);   /* Wait for Go bit to get clear */

    if (mode ==  ARM_FREQ_1250) {
         /* Reset the PLL and post-divider */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0006400 | (pdiv << 24));
         for (i_loop =0 ; i_loop < 5 ; i_loop++);
         /* crystal_clk=25 MHz, NDIV=100(0x64), MDIV=4, H_MDIV=2, PDIV=1 therefore pll_h_clk= (((crystal_clk/pdiv)* ndiv)/h_mdiv) = 1250 */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5 , (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xffffff00) | 0x2);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC, (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC) & 0xffffff00) | 0x4);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0006401 | (pdiv << 24));
    } else if (mode == ARM_FREQ_1000) {
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x00005000 | (pdiv << 24));
         for (i_loop =0 ; i_loop < 5 ; i_loop++) ;
         /* crystal_clk=25 MHz , NDIV=80(0x50), MDIV=4 , H_MDIV=2, PDIV=1 therefore pll_h_clk= (((crystal_clk/pdiv)* ndiv)/h_mdiv) = 1000 */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5 , (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xffffff00) | 0x2);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC, (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC) & 0xffffff00) | 0x4);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0005001 | (pdiv << 24));
    } else if (mode == ARM_FREQ_800) {
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0004000 | (pdiv << 24));
         for (i_loop =0 ; i_loop < 5 ; i_loop++) ;
         /* crystal_clk=25 MHz, NDIV=64(0x40), MDIV=4, H_MDIV=2, PDIV=1 */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5 , (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xffffff00) | 0x2);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC, (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC) & 0xffffff00) | 0x4);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0004001 | (pdiv << 24));
    } else if (mode == ARM_FREQ_600) {
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0006000 | (pdiv << 24));
         for (i_loop =0 ; i_loop < 5 ; i_loop++) ;
         /* crystal_clk=25 MHz, NDIV=96(0x60), MDIV=8, H_MDIV=4, PDIV=1 */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5 , (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xffffff00) | 0x4);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC, (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC) & 0xffffff00) | 0x8);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0006001 | (pdiv << 24));
    } else if (mode == ARM_FREQ_400)  {
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA, 0x0004000 | (pdiv << 24));
         for (i_loop =0 ; i_loop < 5 ; i_loop++) ;
         /* crystal_clk=25 MHz, NDIV=64(0x40), MDIV=8, H_MDIV=4, PDIV=1 */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5 , (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xffffff00) | 0x4);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC, (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC) & 0xffffff00) | 0x8);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0004001| (pdiv << 24));
    } else if (mode == ARM_FREQ_200)   {
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0002000 | (pdiv << 24));
         for (i_loop =0 ; i_loop < 5 ; i_loop++) ;
         /* crystal_clk=25 MHz, NDIV=32(0x20), MDIV=8, H_MDIV=4, PDIV=1 */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5 , (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xffffff00) | 0x4);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC, (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC) & 0xffffff00) | 0x8);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0002001 | (pdiv << 24));
    } else if (mode == ARM_FREQ_1500) {
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0007800 | (pdiv << 24));
         for (i_loop=0; i_loop < 5; i_loop++) ;
         /* crystal_clk=25 MHz, NDIV=120(0x78), MDIV=4, H_MDIV=2, PDIV=1 */
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5 , (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMCTRL5) & 0xffffff00) | 0x2);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC, (reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMC) & 0xffffff00) | 0x4);
         reg32_write((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA , 0x0007801 | (pdiv << 24));
    } else {
         printf("mode is not correct\n");
         return(-1);
    }

    while ( !(reg32_read((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA) & (1 <<IHOST_PROC_CLK_PLLARMA__pllarm_lock)) ) {
           /* Wait for PLL lock to be set */
    }
    reg32_set_bits((volatile uint32_t *)IHOST_PROC_CLK_PLLARMA, 1 << IHOST_PROC_CLK_PLLARMA__pllarm_soft_post_resetb);
    /* Switch to frequency ID 7 */
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_FREQ, 0x87070707);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL , (1 << IHOST_PROC_CLK_POLICY_CTL__GO) | ( 1 << IHOST_PROC_CLK_POLICY_CTL__GO_AC));
    /* Set the GO and GO_AC bit */
    while ((reg32_read((volatile uint32_t *)IHOST_PROC_CLK_POLICY_CTL) & (1 << IHOST_PROC_CLK_POLICY_CTL__GO) ) != 0) {
          /* Wait for Go bit to get clear */
    }
#ifdef CONFIG_SABER2
    /* Disable dynamic clock gating [Bit 1] in addition */
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_CORE0_CLKGATE, 0x00000303);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_CORE1_CLKGATE, 0x00000303);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_ARM_SWITCH_CLKGATE, 0x00000303);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_ARM_PERIPH_CLKGATE, 0x00000303);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_APB0_CLKGATE, 0x00000303);
#else
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_CORE0_CLKGATE, 0x00000301);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_CORE1_CLKGATE, 0x00000301);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_ARM_SWITCH_CLKGATE, 0x00000301);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_ARM_PERIPH_CLKGATE, 0x00000301);
    reg32_write((volatile uint32_t *)IHOST_PROC_CLK_APB0_CLKGATE, 0x00000303);
#endif
    mb();
    return 0;
}
#endif /* CONFIG_IPROC_P7 */
