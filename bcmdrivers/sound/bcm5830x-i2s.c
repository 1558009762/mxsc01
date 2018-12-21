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
#include <linux/module.h>
#include <typedefs.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <osl.h>
#include <siutils.h>
#include <mach/reg_utils.h>
#include <linux/io.h>
#include <mach/io_map.h>

#include "bcm5830x-i2s.h"

static uint32_t a_debug = 0;
module_param_named(debug, a_debug, uint, 0644);
MODULE_PARM_DESC(mask, "debug flag");

int bcm5830x_i2s_port = 2;
module_param_named(port, bcm5830x_i2s_port, int, 0644);
MODULE_PARM_DESC(mask, "I2S port");
EXPORT_SYMBOL(bcm5830x_i2s_port);

static int audio_i2s_pll_poweron(struct bcm5830x_i2s_info *priv, int on)
{
	uint32_t value = 0;

	if (on)
	{
		value = readl(priv->asiutop + ASIU_TOP_CLK_GATING_CTRL_BASE);
		value |= 1 << ASIU_TOP_CLK_GATING_CTRL__AUD_CLK_GATE_EN;
		writel(value, priv->asiutop + ASIU_TOP_CLK_GATING_CTRL_BASE);
		udelay(1000);

		value = readl(priv->crmudru + CRMU_PLL_AON_CTRL_BASE);
		value |= 1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_BG;
		value |= 1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_LDO;
		value |= 1<< CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_PLL;
		value &= ~(1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_ISO_IN);
		writel(value, priv->crmudru + CRMU_PLL_AON_CTRL_BASE);
		udelay(1000);

	} else {

		value = readl(priv->asiutop + ASIU_TOP_CLK_GATING_CTRL_BASE);
		value &= ~(1 << ASIU_TOP_CLK_GATING_CTRL__AUD_CLK_GATE_EN);
		writel(value, priv->asiutop + ASIU_TOP_CLK_GATING_CTRL_BASE);
		udelay(1000);

		value = readl(priv->crmudru + CRMU_PLL_AON_CTRL_BASE);
		value &= ~(1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_BG);
		value &= ~(1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_LDO);
		value &= ~(1<< CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_PWRON_PLL);
		value |= 1 << CRMU_PLL_AON_CTRL__ASIU_AUDIO_GENPLL_ISO_IN;
		writel(value, priv->crmudru + CRMU_PLL_AON_CTRL_BASE);
		udelay(1000);
	}
	return 0;
}

static int audio_i2s_iomux(struct bcm5830x_i2s_info *priv)
{
	uint32_t value = 0;
	
	if (priv->port == 2) {
		/* i2s2 mux with gpio */
		value = readl(priv->chipdru + CRMU_IOMUX_CTRL0_BASE);
		value = (value & (~0xfffff)) | 0x22222; 
		writel(value, priv->chipdru + CRMU_IOMUX_CTRL0_BASE);
	} else if ((priv->port == 0) || (priv->port == 3)) {
		/* i2s0 mux with smartcard0 */
		value = readl(priv->chipdru + SMART_CARD_FCB_SEL_BASE);
		value &= ~(0x7 << SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD0_FCB_SEL_R); 
		value |= 1 << SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD0_FCB_SEL_R;
		writel(value, priv->chipdru + SMART_CARD_FCB_SEL_BASE);

		value = readl(priv->chipdru + CRMU_IOMUX_CTRL3_BASE);
		value &= ~(0x7 << CRMU_IOMUX_CTRL3__CORE_TO_IOMUX_SMART_CARD0_SEL_R); 
		value |= 1 << CRMU_IOMUX_CTRL3__CORE_TO_IOMUX_SMART_CARD0_SEL_R;
		writel(value, priv->chipdru + CRMU_IOMUX_CTRL3_BASE);
	} else if (priv->port == 1) {
		/* i2s1 mux with smartcard1 */
		value = readl(priv->chipdru + SMART_CARD_FCB_SEL_BASE);
		value &= ~(0x7 << SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD1_FCB_SEL_R); 
		value |= 1 << SMART_CARD_FCB_SEL__CORE_TO_IOMUX_SMART_CARD1_FCB_SEL_R;
		writel(value, priv->chipdru + SMART_CARD_FCB_SEL_BASE);

		value = readl(priv->chipdru + CRMU_IOMUX_CTRL3_BASE);
		value &= ~(0x7 << CRMU_IOMUX_CTRL3__CORE_TO_IOMUX_SMART_CARD1_SEL_R); 
		value |= 1 << CRMU_IOMUX_CTRL3__CORE_TO_IOMUX_SMART_CARD1_SEL_R;
		writel(value, priv->chipdru + CRMU_IOMUX_CTRL3_BASE);
	}

	return 0;
}

static int audio_i2s_reset(struct bcm5830x_i2s_info *priv)
{
	uint32_t value;

	value = readl(priv->asiutop + ASIU_TOP_SW_RESET_CTRL_BASE);
	value &= ~(1 << ASIU_TOP_SW_RESET_CTRL__AUD_SW_RESET_N);
	writel(value, priv->asiutop + ASIU_TOP_SW_RESET_CTRL_BASE);
	udelay(10);

	value |= 1 << ASIU_TOP_SW_RESET_CTRL__AUD_SW_RESET_N;
	writel(value, priv->asiutop + ASIU_TOP_SW_RESET_CTRL_BASE);
	udelay(100);

	return 0;
}

static int audio_pll0_init(struct bcm5830x_i2s_info *priv)
{
	writel(0x24, priv->audio + AUD_FMM_IOP_PLL_0_USER_MDIV_Ch0_REG_BASE);
	writel(0x24, priv->audio + AUD_FMM_IOP_PLL_0_USER_MDIV_Ch1_REG_BASE);
	writel(0x24, priv->audio + AUD_FMM_IOP_PLL_0_USER_MDIV_Ch2_REG_BASE);
	writel(0x3, priv->audio + AUD_FMM_IOP_PLL_0_RESET_REG_BASE);
	writel(0x0, priv->audio + AUD_FMM_IOP_PLL_0_RESET_REG_BASE);

	return 0;
}

static int audio_i2s_init(struct bcm5830x_i2s_info *priv, int force)
{
	static int inited = 0;
	uint32_t value;
	uint32_t n = priv->port;

	DBG("==> %s port %d \n", __FUNCTION__, priv->port);
	if ((!inited) || (force)) {
		inited = 1;
		audio_i2s_reset(priv);
		audio_i2s_iomux(priv);
		audio_i2s_pll_poweron(priv, 1);
		audio_pll0_init(priv);

		value = readl(priv->asiutop + ASIU_TOP_AUD_AXI_SB_CTRL_BASE);
		value &= 0xfc0fffff;
		writel(value, priv->asiutop + ASIU_TOP_AUD_AXI_SB_CTRL_BASE);

		/* group */
		writel(0, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_GRP0_REG_BASE);
		writel(1, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_GRP1_REG_BASE);
		writel(2, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_GRP2_REG_BASE);
		writel(3, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_GRP3_REG_BASE);
	}

	/* FCI_ID */
	value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG_BASE + 0x40 * n);
	value &= ~0x3ff;
	value |= n;
	if (n == 3)
		value |= 1 << AUD_FMM_IOP_OUT_SPDIF_0_STREAM_CFG_0_REG__ENA;
	writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG_BASE + 0x40 * n);

	value = readl(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);
	value |= 1 << AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG__NOT_PAUSE_WHEN_EMPTY; 
	value |= 1 << AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG__SOURCEFIFO_SIZE_DOUBLE; 
	value |= 1 << AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG__PROCESS_SEQ_ID_VALID;
	writel(value, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);

	if (priv->port == 3) {
		value = readl(priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_CTRL_REG_BASE);
		value |= 1 << AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_CTRL_REG__DITHER_ENA;
		writel(value, priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_CTRL_REG_BASE);
	} else {
		value = readl(priv->audio + AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG_BASE + 0x40 * n);
		value |= n << AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG__CAP_GROUP_ID_R; 
		writel(value, priv->audio + AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG_BASE + 0x40 * n);

		value = readl(priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + 4 * n);
		value |= 1 << AUD_FMM_BF_CTRL_DESTCH_CFG0_REG__DESTFIFO_SIZE_DOUBLE;
		value |= 1 << AUD_FMM_BF_CTRL_DESTCH_CFG0_REG__NOT_PAUSE_WHEN_FULL;
		value |= (0x180 + n) << AUD_FMM_BF_CTRL_DESTCH_CFG0_REG__FCI_CAP_ID_R; 
		value |= 1 << AUD_FMM_BF_CTRL_DESTCH_CFG0_REG__PROCESS_SEQ_ID_VALID; 
		writel(value, priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + 4 * n);
	}

	return 0;
}


static int audio_i2s_in_enable(struct bcm5830x_i2s_info *priv, int enable)
{
	uint32_t value;
	uint32_t n = priv->port;

	if (enable) {
		value = readl(priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + 4 * n);
		value |= 1 << AUD_FMM_BF_CTRL_DESTCH_CFG0_REG__CAPTURE_ENABLE; 
		writel(value, priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + 4 * n);

		writel(0x1, priv->audio + AUD_FMM_BF_CTRL_DESTCH_CTRL0_REG_BASE + 4 * n);

		value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);
		value |= 1 << AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG__CLOCK_ENABLE; 
		writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);

		value = readl(priv->audio + AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG_BASE + 0x40 * n);
		value |= 1 << AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG__CAP_ENA;
		writel(value, priv->audio + AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG_BASE + 0x40 * n);

	} else {
		value = readl(priv->audio + AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG_BASE + 0x40 * n);
		value &= ~(1 << AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG__CAP_ENA);
		writel(value, priv->audio + AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG_BASE+ 0x40 * n);

		value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);
		value &= ~(1 << AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG__CLOCK_ENABLE);
		writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE+ 0x40 * n);


		writel(0x0, priv->audio + AUD_FMM_BF_CTRL_DESTCH_CTRL0_REG_BASE + 4 * n);

		value = readl(priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + 4 * n);
		value &= ~(1 << AUD_FMM_BF_CTRL_DESTCH_CFG0_REG__CAPTURE_ENABLE); 
		writel(value, priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + 4 * n);
	}

	return 0;
}

static int audio_i2s_out_enable(struct bcm5830x_i2s_info *priv, int enable)
{
	uint32_t value;
	uint32_t n = priv->port;

	if (priv->port == 3) {
		if (enable) {
			value = readl(priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_FORMAT_CFG_REG_BASE);
			value |= 0x3;
			writel(value, priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_FORMAT_CFG_REG_BASE);
			writel(1, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CTRL3_REG_BASE);

			value = readl(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG3_REG_BASE);
			value |= 1 << AUD_FMM_BF_CTRL_SOURCECH_CFG2_REG__SOURCEFIFO_ENABLE; 
			writel(value, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG3_REG_BASE);
		} else {
			value = readl(priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_FORMAT_CFG_REG_BASE);
			value &= ~0x3;
			value = readl(priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_FORMAT_CFG_REG_BASE);
			writel(0, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CTRL3_REG_BASE);

			value = readl(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG3_REG_BASE);
			value &= ~(1 << AUD_FMM_BF_CTRL_SOURCECH_CFG2_REG__SOURCEFIFO_ENABLE); 
			writel(value, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG3_REG_BASE);
		}
		return 0;
	}


	if (enable) {
		value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG_BASE + 0x40 * n);
		value |= 1 << AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG__ENA;
		writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG_BASE + 0x40 * n);
		writel(1, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CTRL0_REG_BASE + 4 * n);

		value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);
		value |= 0x3;
		writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);      

		value = readl(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);
		value |= 1 << AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG__SOURCEFIFO_ENABLE; 
		writel(value, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);
	} else {
		value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG_BASE + 0x40 * n);
		value &= ~(1 << AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG__ENA);
		writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG_BASE + 0x40 * n);
		writel(0, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CTRL0_REG_BASE + 4 * n);

		value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);
		value &= ~0x3;
		writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);      

		value = readl(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);
		value &= ~(1 << AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG__SOURCEFIFO_ENABLE); 
		writel(value, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);
	}

	return 0;
}

static int audio_i2s_rate(struct bcm5830x_i2s_info *priv, int div, int macro, 
	int clksrc)
{
	uint32_t mask = 0xf;
	uint32_t shift = AUD_FMM_IOP_OUT_I2S_2_MCLK_CFG_0_REG__MCLK_RATE_R;
	uint32_t value = 0;
	uint32_t n = priv->port;

	if (priv->port == 3) {
		value = readl(priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_MCLK_CFG_0_REG_BASE);	
		value &= ~((mask << shift) | mask);
		value |= (div << shift) | clksrc;
		writel(value, priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_MCLK_CFG_0_REG_BASE);  
	} else {
		value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_MCLK_CFG_0_REG_BASE + 0x40 * n);	
		value &= ~((mask << shift) | mask);
		value |= (div << shift) | clksrc;
		writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_MCLK_CFG_0_REG_BASE + 0x40 * n);  
	}

	writel(macro, priv->audio + AUD_FMM_IOP_PLL_0_MACRO_REG_BASE);
	DBG("==> MCLK_CFG  %x div %d\n",  value, div);
	DBG("==> macro %d \n", macro);
	return 0;
}

static int audio_i2s_source_bitres(struct bcm5830x_i2s_info *priv, int bits)
{
	uint32_t mask = 0x1f;
	uint32_t value = 0;
	uint32_t shift = AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG__BIT_RESOLUTION_R;
	uint32_t n = priv->port;
	
	value = readl(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);
	value &= ~(mask << shift);
	value |= bits << shift;
	writel(value, priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);  
	
	DBG("==> SOURCECH_CFG value %x\n", value);
	return 0;
}

static int audio_i2s_dst_bitres(struct bcm5830x_i2s_info *priv, int bits)
{
	uint32_t value;
	uint32_t shift = AUD_FMM_BF_CTRL_DESTCH_CFG0_REG__CAPTURE_MODE;
	uint32_t n = priv->port;

	value = readl(priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + n * 4);
	if (bits == 16) {
		value |= 1 << shift;
		writel(value, priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + n * 4);  
	} else if (bits == 32) {
		value &= ~(1 << shift);
		writel(value, priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + n * 4);  
	}

	return 0;
}

static int audio_i2s_tdm(struct bcm5830x_i2s_info *priv, int en) 
{
	uint32_t value;
	uint32_t n = priv->port;

	value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);

	if (en) {
		value |= 1 << AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG__TDM_MODE;
	} else {
		value &= ~(1 << AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG__TDM_MODE);
	}
	writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);

	return 0;
}

static int audio_i2s_slave(struct bcm5830x_i2s_info *priv, int en) 
{
	uint32_t value;
	uint32_t n = priv->port;

	value = readl(priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);

	if (en) {
		value |= 1 << AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG__SLAVE_MODE;
	} else {
		value &= ~(1 << AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG__SLAVE_MODE);
	}
	writel(value, priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);

	return 0;

}

static int bcm5830x_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct bcm5830x_i2s_info *priv = snd_soc_dai_get_drvdata(dai);
	int rate;
	
	DBG("==> %s\n", __FUNCTION__);
	DBG("==> params_channels %d \n", params_channels(params));
	DBG("==> rate %d \n", params_rate(params));
	DBG("==> format %d \n", params_format(params));

	rate = params_rate(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S8:	
			audio_i2s_source_bitres(priv, 8);
			break;

		case SNDRV_PCM_FORMAT_S16_LE:
			audio_i2s_source_bitres(priv, 16);
			break;

		case SNDRV_PCM_FORMAT_S32_LE:
			audio_i2s_source_bitres(priv, 32);
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
		default:
			return -EINVAL;
		}
	} else {
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			audio_i2s_dst_bitres(priv, 16);
			break;

		case SNDRV_PCM_FORMAT_S32_LE:
			audio_i2s_dst_bitres(priv, 32);
			break;

		case SNDRV_PCM_FORMAT_S24_LE:
		default:
			return -EINVAL;
		}
	}

	switch (rate) {
		/* mclk 12.288M / 2 */
	case 8000:
		audio_i2s_rate(priv, 6 * 8000 / rate, 2, 0);
		break;
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
		/* mclk 12.288M */
		audio_i2s_rate(priv, 12 * 8000 / rate, 2, 1);
		break;

	case 11025:
	case 22050:
	case 44100:
		/* mclk 11.2896M */
		audio_i2s_rate(priv, 88200 / rate, 1, 1);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static int bcm5830x_i2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	DBG("==> %s\n", __FUNCTION__);

	return 0;
}

static int bcm5830x_i2s_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct bcm5830x_i2s_info *priv = snd_soc_dai_get_drvdata(dai);

	DBG("==> %s\n", __FUNCTION__);
	snd_soc_dai_set_dma_data(dai, substream, priv);

	return 0;
}

static void bcm5830x_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	DBG("==> %s\n", __FUNCTION__);
}


static int bcm5830x_i2s_dai_set_clkdiv(struct snd_soc_dai *cpu_dai,
				int div_id, int div)
{
	DBG("==> %s\n", __FUNCTION__);
	return 0;
}

static int bcm5830x_i2s_set_fmt(struct snd_soc_dai *cpu_dai,
				   unsigned int fmt)
{
	struct bcm5830x_i2s_info *priv = snd_soc_dai_get_drvdata(cpu_dai);

	DBG("==> %s: fmt %x\n", __FUNCTION__, fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		audio_i2s_slave(priv, 1);
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		audio_i2s_slave(priv, 0);
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		return -EINVAL;
	case SND_SOC_DAIFMT_I2S:
		audio_i2s_tdm(priv, 0);
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		audio_i2s_tdm(priv, 1);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bcm5830x_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct bcm5830x_i2s_info *priv = snd_soc_dai_get_drvdata(dai);
	struct snd_pcm_runtime *runtime = substream->runtime;

	DBG("==> %s cmd %d buffer_size %d \n", __FUNCTION__, cmd, (int)runtime->buffer_size);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			audio_i2s_out_enable(priv, 1);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			audio_i2s_out_enable(priv, 0);
			break;

		default:
			return -EINVAL;
		}
	} else {

		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			audio_i2s_in_enable(priv, 1);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			audio_i2s_in_enable(priv, 0);
			break;
		}
	}

	return 0;
}

static const struct snd_soc_dai_ops bcm5830x_i2s_dai_ops = {
	.startup	= bcm5830x_i2s_startup,
	.shutdown	= bcm5830x_i2s_shutdown,
	.prepare	= bcm5830x_i2s_prepare,
	.trigger	= bcm5830x_i2s_trigger,
	.hw_params	= bcm5830x_i2s_hw_params,
	.set_fmt	= bcm5830x_i2s_set_fmt,
	.set_clkdiv	= bcm5830x_i2s_dai_set_clkdiv,
};

#define BCM5830X_I2S_RATE \
		(SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_8000)

static struct snd_soc_dai_driver bcm5830x_i2s_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = BCM5830X_I2S_RATE,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates =BCM5830X_I2S_RATE ,
		.formats =  SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &bcm5830x_i2s_dai_ops,
};



static int bcm5830x_i2s_probe(struct platform_device *pdev)
{
	struct bcm5830x_i2s_info *priv;
	int err;
	
	DBG("==> %s pdev->id %d \n", __FUNCTION__, pdev->id);

	if (bcm5830x_i2s_port != pdev->id)
		return -ENXIO;

	priv = kzalloc(sizeof(struct bcm5830x_i2s_info), GFP_KERNEL);
	if (!priv) {
		dev_err(&pdev->dev, "allocation failed\n");
		err = -ENOMEM;
		goto error;
	}
	dev_set_drvdata(&pdev->dev, priv);
	priv->port = pdev->id;
	
	priv->audio = ioremap_nocache(AUD_MISC_REVISION_REG, SZ_4K - 1);
	if (!priv->audio) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -ENOMEM;
		goto err_alloc;
	}

	priv->audioeav = ioremap_nocache(0x180af000, SZ_4K - 1);
	if (!priv->audioeav) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -ENOMEM;
		goto err_map_audio;
	}

	priv->chipdru = ioremap_nocache(CRMU_GENPLL_CONTROL0, 0xbff);
	if (!priv->chipdru) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -ENOMEM;
		goto err_map_audioeav;
	}

	priv->crmudru = ioremap_nocache(CRMU_XTAL_CHANNEL_CONTROL, SZ_4K - 1);
	if (!priv->crmudru) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -ENOMEM;
		goto err_map_chipdru;
	}

	priv->asiutop = ioremap_nocache(ASIU_INTR_STATUS, SZ_4K - 1);
	if (!priv->asiutop) {
		dev_err(&pdev->dev, "ioremap failed\n");
		err = -ENOMEM;
		goto err_map_crmudru;
	}

	priv->irq = 175;

	err = snd_soc_register_dai(&pdev->dev, &bcm5830x_i2s_dai);
	if (err) {
		dev_err(&pdev->dev, "snd_soc_register_dai failed\n");
		goto err_map_asiutop;
	}
	
	audio_i2s_init(priv, 0);
	return 0;

err_map_asiutop:
	iounmap(priv->asiutop);
err_map_crmudru:
	iounmap(priv->crmudru);
err_map_chipdru:
	iounmap(priv->chipdru);
err_map_audioeav:
	iounmap(priv->audioeav);
err_map_audio:
	iounmap(priv->audio);
err_alloc:
	kfree(priv);
error:
	return err;
}

static int bcm5830x_i2s_remove(struct platform_device *pdev)
{
	struct bcm5830x_i2s_info *priv = dev_get_drvdata(&pdev->dev);

	snd_soc_unregister_dai(&pdev->dev);

	iounmap(priv->audio);
	iounmap(priv->audioeav);
	iounmap(priv->chipdru);
	iounmap(priv->crmudru);
	iounmap(priv->asiutop);

	kfree(priv);
	return 0;
}

#ifdef CONFIG_PM

#define SAVE_REG_TO_MEM(pr) \
	do { reg2men[i].reg = (pr); reg2men[i].value = readl(pr); i++;} while(0)

#define REG_MEM_SIZE 32 

struct reg_men_t{
	void __iomem *reg;
	uint32_t value;	
};

struct reg_men_t reg2men[REG_MEM_SIZE];

static int bcm5830x_i2s_suspend(struct device *dev)
{
	struct bcm5830x_i2s_info *priv = dev_get_drvdata(dev);
	int i = 0;
	int n = priv->port;

	DBG("==> %s \n", __FUNCTION__);

	SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CFG0_REG_BASE + 4 * n);
	SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_CTRL0_REG_BASE + 4 * n);
	if (n < 3) {
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_DESTCH_CFG0_REG_BASE + 4 * n);
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_DESTCH_CTRL0_REG_BASE + 4 * n);
	}
	if (n < 3) {
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_IOP_IN_I2S_0_CAP_STREAM_CFG_0_REG_BASE + 0x40 * n);
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_IOP_OUT_I2S_0_I2S_CFG_REG_BASE + 0x40 * n);
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_IOP_OUT_I2S_0_MCLK_CFG_0_REG_BASE + 0x40 * n);  
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_IOP_OUT_I2S_0_STREAM_CFG_0_REG_BASE + 0x40 * n);
	} else {
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_MCLK_CFG_0_REG_BASE);  
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_IOP_OUT_SPDIF_0_SPDIF_FORMAT_CFG_REG_BASE);
	}
	SAVE_REG_TO_MEM(priv->audio + AUD_FMM_IOP_PLL_0_MACRO_REG_BASE);

	/* stop for SLEEP mode */
	audio_i2s_in_enable(priv, 0);
	audio_i2s_out_enable(priv, 0);

	/* pcm window */
	if (n < 3) {
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_DESTCH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n);
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_DESTCH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n + 4);
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_DESTCH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n + 8);
		SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_DESTCH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n + 12);
	}
	SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n);
	SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n + 4);
	SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n + 8);
	SAVE_REG_TO_MEM(priv->audio + AUD_FMM_BF_CTRL_SOURCECH_RINGBUF_0_RDADDR_REG_BASE + 0x30 * n + 12);

	reg2men[i].reg = NULL; 
	
	if (i >= REG_MEM_SIZE) {
		dev_err(dev, "regs %d exceeding max %d \n", i, REG_MEM_SIZE);
	}

	return 0;
}

static int bcm5830x_i2s_resume(struct device *dev)
{
	struct bcm5830x_i2s_info *priv = dev_get_drvdata(dev);
	int i;

	DBG("==> %s \n", __FUNCTION__);
	audio_i2s_init(priv, 1);

	for (i = 0; i < REG_MEM_SIZE; i++) {
		if (!reg2men[i].reg)	
			break;
	}

	while (--i >= 0) {
		writel(reg2men[i].value, reg2men[i].reg);
	}

	return 0;
}
static SIMPLE_DEV_PM_OPS(bcm5830x_i2s_pm, bcm5830x_i2s_suspend, bcm5830x_i2s_resume);
#endif

static struct platform_driver bcm5830x_i2s_driver = {
	.probe		= bcm5830x_i2s_probe,
	.remove		= __devexit_p(bcm5830x_i2s_remove),
	.driver		= {
		.name	= "bcm5830x-i2s",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &bcm5830x_i2s_pm,
#endif
	},
};


module_platform_driver(bcm5830x_i2s_driver)

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("ALSA SoC bcm5830x I2S Interface");
MODULE_LICENSE("GPL");
