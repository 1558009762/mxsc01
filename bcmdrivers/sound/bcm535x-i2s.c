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
/*
 * ALSA I2S Interface for the Broadcom BCM535X family of I2S cores
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
#include <bcmutils.h>
#include <siutils.h>
#include <sbhnddma.h>
#include <hnddma.h>
#include <hndsoc.h>
#include "i2s_regs.h"
#include "bcm535x-i2s.h"
#include <mach/reg_utils.h>
#include <linux/io.h>
#include <mach/io_map.h>

void __iomem *baseAddr;
EXPORT_SYMBOL_GPL(baseAddr);

bcm535x_i2s_info_t *snd_bcm535x;
EXPORT_SYMBOL_GPL(snd_bcm535x);

void register_dump(void)
{
	uint32_t deviceControl, clkDivider, stxControl;
	
	//DBG("%s baseAddr: %x\n", __FUNCTION__, baseAddr);
	
	// Read the registers
	deviceControl = readl(baseAddr + I2S_DEVCONTROL_REG);
	clkDivider = readl(baseAddr + I2S_CLOCKDIVIDER_REG);
	stxControl = readl(baseAddr + I2S_STXCTRL_REG);

	DBG("%s: deviceControl 0x%x\n", __FUNCTION__, deviceControl);
	DBG("%s: clkDivider 0x%x\n", __FUNCTION__, clkDivider);
	DBG("%s: stxControl 0x%x\n", __FUNCTION__, stxControl);
	
	return;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
int bcm535x_i2s_probe(struct snd_soc_dai *dai)
{
	DBG("%s\n", __FUNCTION__);
	
	return 0;
}
#else
int bcm535x_i2s_probe(struct platform_device *pdev, struct snd_soc_dai *dai)
{

	DBG("%s\n", __FUNCTION__);


	return 0;
}
#endif

int bcm535x_i2s_set_format(struct snd_soc_dai *soc_dai, unsigned int fmt)
{
	uint32_t deviceControl;			// Device control register
	
	DBG("%s: format 0x%x\n", __FUNCTION__, fmt);
	
	// Read device control register
	deviceControl = readl(baseAddr + I2S_DEVCONTROL_REG);
	
	// Set the core to I2S mode
	deviceControl &= ~I2S_DC_TDM_SEL;
	
	/* See include/sound/soc.h for DAIFMT */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		/* Codec clk master and frame master */
		deviceControl |= I2S_DC_BCLKD_IN;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		/* Codec clk slave and frame master */
		/* BCM SOC is the master */
		deviceControl &= ~I2S_DC_BCLKD_IN;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		/* Codec clk master and frame slave */
		deviceControl |= I2S_DC_BCLKD_IN;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		/* Codec clk slave and frame slave */
		/* BCM SOC is the master */
		deviceControl &= ~I2S_DC_BCLKD_IN;
		break;
	default:
		DBG("%s: unsupported MASTER: 0x%x \n", __FUNCTION__,
		    fmt & SND_SOC_DAIFMT_MASTER_MASK );
		return -EINVAL;
	}
	
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* we only support I2S Format */
		break;
	default:
		DBG("%s: unsupported FORMAT: 0x%x \n", __FUNCTION__,
		    fmt & SND_SOC_DAIFMT_FORMAT_MASK );
		return -EINVAL;
	}

	// Write I2S Device control register
	writel(deviceControl, baseAddr + I2S_DEVCONTROL_REG);
	DBG("%s: deviceControl: 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_DEVCONTROL_REG));
					
#if defined CODEC_WM8955
	WM8955_set_fmt(fmt);
#elif defined CODEC_WM8750
	WM8750_set_dai_fmt(fmt);
#endif
	
	return 0;
}

int bcm535x_i2s_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{

	void __iomem *i2s_m0_idm_io_control_direct    = IOMEM(HW_IO_PHYS_TO_VIRT(I2S_M0_IDM_IO_CONTROL_DIRECT));

	uint32_t deviceControl, clkDivider, stxControl, idmControlDirect, i2sControl;
	uint32_t rate;
	uint32_t srate = 0;
	int channels;
	unsigned int access, format, subformat;
	int ii = 0;
	bool found = false;
	
	// Read the registers
	deviceControl = readl(baseAddr + I2S_DEVCONTROL_REG);
	clkDivider = readl(baseAddr + I2S_CLOCKDIVIDER_REG);
	stxControl = readl(baseAddr + I2S_STXCTRL_REG);
	idmControlDirect = readl(i2s_m0_idm_io_control_direct);
	i2sControl = readl(baseAddr + I2S_CONTROL_REG);
	
	rate = params_rate(params);
	channels = params_channels(params);
	
	DBG("%s: rate 0x%x\n", __FUNCTION__, rate);
	DBG("%s: channels 0x%x\n", __FUNCTION__, channels);
	
	// Make sure I2S block system clock is enabled
	idmControlDirect |= I2S_IDM_CD_CLK_ENABLE;
	
	// Setup codec	
#if defined CODEC_WM8955
	WM8955_hw_params(params);

	// For wm8955 codec, mclk frequency is 12.288 Mhz on the SVK Board since CLK/2 is configured on the codec, no need to divide
	snd_bcm535x->mclk = 12288000;
	idmControlDirect &= ~I2S_IDM_CD_CLK_DIV;
	DBG("%s Using WM8955 codec\n", __FUNCTION__);
	
#elif defined CODEC_WM8750
	WM8750_pcm_hw_params(params);
	
	// For wm8750 codecs mclk frequency is 12.288 Mhz on the SVK Board, no need to divide
	snd_bcm535x->mclk = 12288000;
	idmControlDirect &= ~I2S_IDM_CD_CLK_DIV;
	DBG("%s Using WM8750 codec\n", __FUNCTION__);	

#else
	// Setup mclk frequency	
	if ((rate == 44100) || (rate == 22050)) {
		snd_bcm535x->mclk = 11289600;
		idmControlDirect &= ~I2S_IDM_CD_CLK_DIV;
	}
	else if (rate > 96000) {
		snd_bcm535x->mclk = 24567000;
		idmControlDirect &= ~I2S_IDM_CD_CLK_DIV;		
	}
	else {
		snd_bcm535x->mclk = 12288000;	
		idmControlDirect |= 0x20;						// Documentation indicates Bit6 is clk_div, but Bit5 seems to work and not Bit6.
		//idmControlDirect |= I2S_IDM_CD_CLK_DIV;	
	}
#endif
	
	writel(idmControlDirect, i2s_m0_idm_io_control_direct);
	DBG("%s: idmControlDirect: 0x%x\n", __FUNCTION__, readl(i2s_m0_idm_io_control_direct));
	
	// Setup clock divider register
	/* Set up our ClockDivider register with audio sample rate */
	for (ii = 0; ii < ARRAY_SIZE(i2s_clkdiv_coeffs); ii++) {
		if ((i2s_clkdiv_coeffs[ii].rate == rate) &&
		    (i2s_clkdiv_coeffs[ii].mclk == snd_bcm535x->mclk)) {
			found = true;
			break;
		}
	}

	if (found != true) {
		printk(KERN_ERR "%s: unsupported audio sample rate %d Hz and mclk %d Hz "
		       "combination\n", __FUNCTION__, rate, snd_bcm535x->mclk);
		return -EINVAL;
	} else {
		/* Write the new SRATE into the clock divider register */
		srate = (i2s_clkdiv_coeffs[ii].srate << I2S_CLKDIV_SRATE_SHIFT);
		clkDivider &= ~I2S_CLKDIV_SRATE_MASK;
		writel(clkDivider | srate, baseAddr + I2S_CLOCKDIVIDER_REG);

		DBG("%s: i2s clkdivider 0x%x txplayth 0x%x\n", __FUNCTION__,
		    readl(baseAddr + I2S_CLOCKDIVIDER_REG),
		    readl(baseAddr + I2S_TXPLAYTH_REG));
		DBG("%s: audio sample rate %d Hz and mclk %d Hz\n",
		    __FUNCTION__, rate, snd_bcm535x->mclk);
	}

	// Program TxPlayTH to 1
	//writel(1, baseAddr + I2S_TXPLAYTH_REG);
	DBG("%s: txplayth 0x%x\n", __FUNCTION__,
		    readl(baseAddr + I2S_TXPLAYTH_REG));
	
	DBG("%s: %d channels in this stream\n", __FUNCTION__, channels);
	
	// Setup number of channels
	switch (channels) {
	case 2:
		deviceControl &= ~I2S_DC_OPCHSEL;
		break;
	case 6:
		deviceControl |= I2S_DC_OPCHSEL;
		break;
	default:
		DBG(KERN_ERR "%s: unsupported number of channels in stream - %d\n"
		       "combination\n", __FUNCTION__, channels);
		return -EINVAL;
	}

	access = params_access(params);
	format = params_format(params);
	subformat = params_subformat(params);
	
	DBG("%s: access 0x%x\n", __FUNCTION__, params_access(params));
	DBG("%s: format 0x%x\n", __FUNCTION__, params_format(params));
	DBG("%s: subformat 0x%x\n", __FUNCTION__, params_subformat(params));
	
	// Set to half-duplex TX only
	// Clear duplex field
	deviceControl &= ~(0x00000300);
	//deviceControl &= ~I2S_DC_DPX_MASK;		// Full duplex
	// Set to full-duplex
	deviceControl |= 0x300;						// Full duplex
	
	/* Set up core's SRAM for Half duplex Tx */
	//deviceControl |= I2S_DC_I2SCFG;
	
	// Clear TX/RX wordlength
	deviceControl &= ~I2S_DC_WL_TXRX_MASK;
	stxControl &= ~I2S_STXC_WL_MASK;
	
	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		deviceControl |= 0x0;				// TX Wordlength
		deviceControl |= 0x0;				// RX Wordlength
		stxControl |= 0x0;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		deviceControl |= 0x400;				// TX Wordlength
		deviceControl |= 0x1000;			// RX Wordlength
		stxControl |= 0x01;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		deviceControl |= 0x800;				// TX Wordlength
		deviceControl |= 0x2000;			// RX Wordlength
		stxControl |= 0x02;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		deviceControl |= 0xC00;				// TX Wordlength
		deviceControl |= 0x3000;			// RX Wordlength
		/* SPDIF doesn't support 32 bit samples */
		/* Should we just disable SPDIF rather than putting out garbage? */
		stxControl |= 0x03;
		break;
	default:
		DBG("unsupported format\n");
		break;
	}
	
	// Enable SPDIF TX
	stxControl |= I2S_STXC_STX_EN;
	
	//i2sControl |= I2S_CONTROL_CLSLM;
	//i2sControl |= I2S_CONTROL_MUTE;

	// Write DeviceControl, STX Control registers
	writel(deviceControl, baseAddr + I2S_DEVCONTROL_REG);
	writel(stxControl, baseAddr + I2S_STXCTRL_REG);

	writel(i2sControl, baseAddr + I2S_CONTROL_REG);
	
	DBG("%s: deviceControl: 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_DEVCONTROL_REG));
	DBG("%s: stxControl 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_STXCTRL_REG));
	DBG("%s: i2sControl 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_CONTROL_REG));
	
	return 0;
}

void bcm535x_i2s_snd_txctrl(int on)
{

	DBG("%s\n", __FUNCTION__);
	
	return;
}

void bcm535x_i2s_snd_rxctrl(int on)
{

	DBG("%s\n", __FUNCTION__);
	return;
}

int bcm535x_i2s_snd_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	uint32_t i2sControl, i2sIntrecelazyDMA0;
	int retValue = 0;
	uint32 temp;
	
	temp = readl(baseAddr + I2S_FIFOCOUNTER_REG);
	DBG("%s: i2s txcnt 0x%x rxcnt 0x%x\n", __FUNCTION__,
	    (temp & I2S_FC_TX_CNT_MASK)>> I2S_FC_TX_CNT_SHIFT,
	    (temp & I2S_FC_RX_CNT_MASK)>> I2S_FC_RX_CNT_SHIFT);

	DBG("%s: deviceStatus: 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_DEVSTATUS_REG));
	DBG("%s: intstatus 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_INTSTATUS_REG));
	
	i2sControl = readl(baseAddr + I2S_CONTROL_REG);
	i2sIntrecelazyDMA0 = readl(baseAddr + I2S_INTRECELAZY_DMA0_REG);
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
#if defined ENABLE_TX
		i2sControl |= I2S_CONTROL_PLAYEN;
#endif
#if defined ENABLE_RX		
		i2sControl |= I2S_CONTROL_RECEN;
		i2sIntrecelazyDMA0 |= I2S_INTRECE_LAZY_FC;
#endif		
#if defined CODEC_WM8750	
	// Set bias level to on
		//WM8750_set_bias_level(SND_SOC_BIAS_ON);
#endif		
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
#if defined ENABLE_TX
		i2sControl &= ~I2S_CONTROL_PLAYEN;
#endif
#if defined ENABLE_RX		
		i2sControl &= ~I2S_CONTROL_RECEN;
#endif		
#if defined CODEC_WM8750	
	// Set bias level to off
		//WM8750_set_bias_level(SND_SOC_BIAS_OFF);
#endif
		break;
	default:
		retValue = -EINVAL;
	}
	
	writel(i2sControl, baseAddr + I2S_CONTROL_REG);
	writel(i2sIntrecelazyDMA0, baseAddr + I2S_INTRECELAZY_DMA0_REG);
	
	DBG("%s: i2sControl 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_CONTROL_REG));
	DBG("%s: i2sIntrecelazyDMA0 0x%x\n", __FUNCTION__, readl(baseAddr + I2S_INTRECELAZY_DMA0_REG));
	
	return retValue;
}

int bcm535x_i2s_suspend(struct snd_soc_dai *dai)
{

	DBG("%s\n", __FUNCTION__);

	return 0;
}

int bcm535x_i2s_resume(struct snd_soc_dai *dai)
{

	DBG("%s\n", __FUNCTION__);

	return 0;
}

int bcm535x_i2s_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{

	DBG("%s\n", __FUNCTION__);

	return 0;
}

int bcm535x_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id, unsigned int freq, int dir)
{
	
	/* Store the MCLK rate that we're using, we can use it to help us to pick
	 * the right clkdiv settings later.
	 */
	snd_bcm535x->mclk = freq;
	DBG("%s: mclk %d Hz\n", __FUNCTION__, snd_bcm535x->mclk);

	return 0;
}

struct snd_soc_dai_ops bcm535x_i2s_dai_ops = {
		.set_sysclk = bcm535x_i2s_set_sysclk,
		.set_clkdiv = bcm535x_i2s_set_clkdiv,
		.set_fmt = bcm535x_i2s_set_format,	
		.hw_params = bcm535x_i2s_hw_params,
		.trigger = bcm535x_i2s_snd_trigger,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
struct snd_soc_dai_driver bcm535x_i2s_dai_driver = {
	.name = "bcm535x-i2s-driver",
	.id = 0,
	
	.probe = bcm535x_i2s_probe,
	.suspend = bcm535x_i2s_suspend,
	.resume = bcm535x_i2s_resume,
	
	.ops = &bcm535x_i2s_dai_ops,
	
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = BCM535X_I2S_RATES,
		.formats = BCM535X_I2S_FORMATS,
	},
	
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = BCM535X_I2S_RATES,
		.formats = BCM535X_I2S_FORMATS,
	},
};
#endif

struct snd_soc_dai bcm535x_i2s_dai = {
	.name = " bcm535x-i2s",
	.id = 0,
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	.driver = &bcm535x_i2s_dai_driver,
#else	
	.probe = bcm535x_i2s_probe,
	.suspend = bcm535x_i2s_suspend,
	.resume = bcm535x_i2s_resume,
	
	.ops = &bcm535x_i2s_dai_ops,
	
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = BCM535X_I2S_RATES,
		.formats = BCM535X_I2S_FORMATS,},
	
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = BCM535X_I2S_RATES,
		.formats = BCM535X_I2S_FORMATS,},
#endif		
};

#define DMAREG(a, direction, fifonum)	( \
	(direction == DMA_TX) ? \
                (void *)(uintptr)&(a->regs->dmaregs[fifonum].dmaxmt) : \
                (void *)(uintptr)&(a->regs->dmaregs[fifonum].dmarcv))


int bcm535x_i2s_attach(struct platform_device *pdev)
{

	uint32_t idm_i2s_reset_status_reg, sku_id;
	osl_t *osh = NULL;
	int dma_attach_err = 0;
	
	void __iomem *i2s_m0_idm_io_reset_control    = IOMEM(HW_IO_PHYS_TO_VIRT(I2S_M0_IDM_RESET_CONTROL));
	void __iomem *i2s_m0_idm_io_reset_control_status    = IOMEM(HW_IO_PHYS_TO_VIRT(I2S_M0_IDM_RESET_STATUS));
	void __iomem *rom_s0_idm_io_status    = IOMEM(HW_IO_PHYS_TO_VIRT(ROM_S0_IDM_IO_STATUS));
	
	// Register with soc
	DBG("%s\n", __FUNCTION__);
	
	printk("Checking platform type\n");
	sku_id = readl(rom_s0_idm_io_status);		// 1 - low sku, 2 - medium sku, 0 or 3 - high sku
	printk("sku_id: 0x%x\n", sku_id);
	
	// If platform is not equal to 12, bailing out
    if(sku_id == 1 || sku_id == 2) {
		printk("No I2S block on platform\n");
		return 0;
	}
	
	// Temporary patch to bring I2S block out of reset
	// Should go into kernel init area, later	
	printk("Resetting i2s block\n");
	
	// Reset the device
	writel(1, i2s_m0_idm_io_reset_control);
	
	idm_i2s_reset_status_reg = readl(i2s_m0_idm_io_reset_control_status);
	printk("idmResetStatusReg:  %x\n", idm_i2s_reset_status_reg);
	
	printk("Exiting reset\n");
	// Exit Reset the device
	writel(0, i2s_m0_idm_io_reset_control);
	// End temporary code
	
	baseAddr = IOMEM(IPROC_I2S_REG_VA);
	
	//retVal = snd_soc_register_dai(&bcm535x_i2s_dai);
	//DBG("%s: retVal = 0x%x\n", __FUNCTION__, retVal);
	
	register_dump();
	
	snd_bcm535x->irq = BCM5301X_I2S_INTERRUPT;

	DBG("%s: Attaching osl\n", __FUNCTION__);
	osh = osl_attach(pdev, PCI_BUS, FALSE);
	ASSERT(osh);

	snd_bcm535x->osh = osh;
	snd_bcm535x->cpu_dai = &bcm535x_i2s_dai;
	
	snd_bcm535x->sih = si_attach(0, snd_bcm535x->osh, snd_bcm535x->regsva, PCI_BUS, pdev,
	                        NULL, NULL);
	
	snd_bcm535x->regs = (i2sregs_t *)si_setcore(snd_bcm535x->sih, I2S_CORE_ID, 0);
	//si_core_reset(snd_bcm535x->sih, 0, 0);
	
	DBG("%s: Attaching DMA\n", __FUNCTION__);
	
	/* DMA attach not setting up Rx.  It hasn't been tested. */
	//snd_bcm535x->di[0] = dma_attach(snd_bcm535x->osh, "i2s_dma", snd_bcm535x->sih,
	//                            DMAREG(snd_bcm535x, DMA_TX, 0),
	//                            NULL, 64, 0,
	//                            0, -1, 0, 0, NULL);
								
	/* DMA attach setup TX and RX */
	//snd_bcm535x->di[0] = dma_attach(snd_bcm535x->osh, "i2s_dma", snd_bcm535x->sih,
	//                            DMAREG(snd_bcm535x, DMA_TX, 0),
	//                            DMAREG(snd_bcm535x, DMA_RX, 0),
	//                            64, 64, 512, -1, 0, 4, NULL);
								
	snd_bcm535x->di[0] = dma_attach(snd_bcm535x->osh, "i2s_dma", snd_bcm535x->sih,
	                            DMAREG(snd_bcm535x, DMA_TX, 0),
	                            DMAREG(snd_bcm535x, DMA_RX, 0),
	                            64, 64, 512, -1, 0, 0, NULL);

	if (snd_bcm535x->di[0] == NULL)
		DBG("%s: DMA Attach not successfull\n", __FUNCTION__);
	
	dma_attach_err |= (NULL == snd_bcm535x->di[0]);
	
	/* Tell DMA that we're not using framed/packet data */
	dma_ctrlflags(snd_bcm535x->di[0], DMA_CTRL_UNFRAMED /* mask */, DMA_CTRL_UNFRAMED /* value */);
	
	return 0;
}
