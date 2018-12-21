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

//#define TDM_MODE
//#define I2S_SLAVE_MODE

#if defined(TDM_MODE)
	#define AUDIO_FORMAT (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_DSP_B | \
		SND_SOC_DAIFMT_NB_NF)
#elif defined(I2S_SLAVE_MODE)
	#define AUDIO_FORMAT (SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S | \
		SND_SOC_DAIFMT_NB_NF)
#else
	#define AUDIO_FORMAT (SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_I2S | \
		SND_SOC_DAIFMT_NB_NF)
#endif

static uint32_t i2s_port = 2;
module_param_named(port, i2s_port, uint, 0644);
MODULE_PARM_DESC(mask, "I2S port");

static uint32_t a_debug = 0;
module_param_named(debug, a_debug, uint, 0644);
MODULE_PARM_DESC(mask, "debug flag");

static int bcm5830x_hw_params_spdif(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned sysclk = 12288000;

	DBG("==> %s rate %d \n", __FUNCTION__, params_rate(params));

	switch (params_rate(params)) {
	case 8000:
		sysclk = 12288000 / 2;
		break;

	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
		sysclk = 12288000;
		break;

	case 11025:
	case 22050:
	case 44100:
		sysclk = 11289600;
		break;

	default:
		return -EINVAL;
	}

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	return 0;
}
static int bcm5830x_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned sysclk = 12288000;

	DBG("==> %s: rate %d \n", __FUNCTION__, params_rate(params));

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
		sysclk = 12288000;
		break;

	case 11025:
	case 22050:
	case 44100:
		sysclk = 11289600;
		break;

	default:
		return -EINVAL;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}


static struct snd_soc_ops bcm5830x_ops = {
	.hw_params = bcm5830x_hw_params,
};


/* machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

/* machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},
};

static int bcm5830x_aic3x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	DBG("=>> %s \n", __FUNCTION__);

	/* Add specific widgets */
	snd_soc_dapm_new_controls(dapm, aic3x_dapm_widgets,
				  ARRAY_SIZE(aic3x_dapm_widgets));

	/* Set up specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(dapm, "MONO_LOUT");
	snd_soc_dapm_disable_pin(dapm, "HPLCOM");
	snd_soc_dapm_disable_pin(dapm, "HPRCOM");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Line Out");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Line In");

	return 0;
}

static struct snd_soc_dai_link bcm5830x_dai_aci3x = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name= "bcm5830x-i2s.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "bcm5830x-pcm",
	.init = bcm5830x_aic3x_init,
	.ops = &bcm5830x_ops,
};

static struct snd_soc_card snd_soc_machine_bcm5830x_aci3x = {
	.name = "bcm5830x SVK",
	.owner = THIS_MODULE,
	.dai_link = &bcm5830x_dai_aci3x,
	.num_links = 1,

	.dapm_widgets	= aic3x_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aic3x_dapm_widgets),
	.dapm_routes	= audio_map,
	.num_dapm_routes = ARRAY_SIZE(audio_map),
};

static const struct snd_soc_dapm_route route_wm8750[] = {
	{ "Headphone Jack", NULL, "LOUT1" },
	{ "Headphone Jack", NULL, "ROUT1" },
	{ "LINPUT1", NULL, "Mic Bias" },
	{ "RINPUT1", NULL, "Mic Bias" },
	{ "Mic Bias", NULL, "Mic Jack"},
};

static const struct snd_soc_dapm_widget wm8750_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static int bcm5830x_wm8750_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	DBG("=>> %s \n", __FUNCTION__);

	/* These endpoints are not being used. */
	snd_soc_dapm_nc_pin(dapm, "LINPUT2");
	snd_soc_dapm_nc_pin(dapm, "RINPUT2");
	snd_soc_dapm_nc_pin(dapm, "LINPUT3");
	snd_soc_dapm_nc_pin(dapm, "RINPUT3");
	snd_soc_dapm_nc_pin(dapm, "OUT3");
	snd_soc_dapm_nc_pin(dapm, "MONO1");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "LINPUT1");
	snd_soc_dapm_enable_pin(dapm, "RINPUT1");
	snd_soc_dapm_enable_pin(dapm, "LOUT1");
	snd_soc_dapm_enable_pin(dapm, "ROUT1");

	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Line In");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	return 0;
}


static struct snd_soc_dai_link bcm5830x_dai_wm8750 = {
	.name = "wm8750",
	.stream_name = "wm8750",
	.cpu_dai_name= "bcm5830x-i2s.2",
	.codec_dai_name = "wm8750-hifi",
	.codec_name = "wm8750.0-001a",
	.platform_name = "bcm5830x-pcm",
	.init = bcm5830x_wm8750_init,
	.ops = &bcm5830x_ops,
};


static struct snd_soc_card snd_soc_machine_bcm5830x_wm8750 = {
	.name = "bcm5830x wm8750",
	.owner = THIS_MODULE,
	.dai_link = &bcm5830x_dai_wm8750,
	.num_links = 1,

	.dapm_widgets	= wm8750_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(wm8750_dapm_widgets),
	.dapm_routes	= route_wm8750,
	.num_dapm_routes = ARRAY_SIZE(route_wm8750),
};

/* spdif */
static struct snd_soc_ops bcm5830x_spdif_ops = {
	.hw_params = bcm5830x_hw_params_spdif,
};

static struct snd_soc_dai_link bcm5830x_dai_spdif = {
	.name = "S/PDIF",
	.stream_name = "S/PDIF PCM Playback",
	.cpu_dai_name = "bcm5830x-i2s.3",
	.codec_dai_name = "dit-hifi",
	.codec_name = "spdif-dit",
	.platform_name = "bcm5830x-pcm",
	.ops = &bcm5830x_spdif_ops,
};

static struct snd_soc_card snd_soc_machine_bcm5830x_spdif ={
	 
	.name = "bcm5830x spdif",
	.owner = THIS_MODULE,
	.dai_link = &bcm5830x_dai_spdif,
	.num_links = 1,
};

/* select right signal used for I2SDC */
static int bcm5830x_audio_gpio_init(int port)
{
	void __iomem *crmugpio;
	void __iomem *asiugpio;
	uint32_t value;

	if ((port == 2) || (port == 3))
		return 0;
	
	crmugpio = ioremap_nocache(0x03024000, SZ_4K - 1);
	if (!crmugpio) {
		return -ENOMEM;
	}
	asiugpio = ioremap_nocache(ASIU_GP_DATA_IN_0, SZ_4K - 1);
	if (!asiugpio) {
		iounmap(crmugpio);
		return -ENOMEM;
	}

	value = readl(crmugpio + GP_OUT_EN_BASE);
	value |= 0x3;
	writel(value, crmugpio + GP_OUT_EN_BASE);
	if (port == 1) {
		/* To Select SC0_DC_GPIOx_3P3, set AON_GPIO1 AON_GPIO0 0 and 1 */
		value = readl(crmugpio + GP_DATA_OUT_BASE);
		value &= ~0x3;
		value |= 0x1;
		writel(value, crmugpio + GP_DATA_OUT_BASE);

	} else {
		/* To Select SC0_DC_GPIOx_3P3, set AON_GPIO1 AON_GPIO0 1 and 0 */
		value = readl(crmugpio + GP_DATA_OUT_BASE);
		value &= ~0x3;
		value |= 0x2;
		writel(value, crmugpio + GP_DATA_OUT_BASE);
	}
	
	/* reset */
	writel(0x400000, asiugpio + ASIU_GP_OUT_EN_3_BASE);
	writel(0, asiugpio + ASIU_GP_DATA_OUT_3_BASE);
	udelay(10);
	writel(0x400000, asiugpio + ASIU_GP_DATA_OUT_3_BASE);

	/* headset enable SC0_DC_GPIO0_3P3(ASIU_GPIO_117) */
	/* handsfree speaker enable SC0_DC_GPIO2_3P3(ASIU_GPIO_119) */
	value = readl(asiugpio + ASIU_GP_OUT_EN_3_BASE);
	value |= 0x200000 | 0x800000;
	writel(value, asiugpio + ASIU_GP_OUT_EN_3_BASE);
	value = readl(asiugpio + ASIU_GP_DATA_OUT_3_BASE);
	value |= 0x200000 | 0x800000;
	writel(value, asiugpio + ASIU_GP_DATA_OUT_3_BASE);

	iounmap(crmugpio);
	iounmap(asiugpio);
	return 0;

}

static int __devinit bcm5830x_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	int ret;

	DBG("=>> %s: bcm5830x_i2s_port %d \n", __FUNCTION__, bcm5830x_i2s_port);

	ret = bcm5830x_audio_gpio_init(bcm5830x_i2s_port);
	if (ret) {
		dev_err(&pdev->dev, "bcm5830x_audio_gpio_init() failed: %d\n", ret);
		return ret;
	}

	if (bcm5830x_i2s_port == 2)
		card = &snd_soc_machine_bcm5830x_wm8750;
	else if (bcm5830x_i2s_port == 3)
		card = &snd_soc_machine_bcm5830x_spdif;
	else if (bcm5830x_i2s_port == 1)
		card = &snd_soc_machine_bcm5830x_aci3x;
	else if (bcm5830x_i2s_port == 0) {
		bcm5830x_dai_aci3x.cpu_dai_name= "bcm5830x-i2s.0",
		card = &snd_soc_machine_bcm5830x_aci3x;
	} else {
		return -ENOENT;
	}

	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);

	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n", ret);
		
	return ret;
}

static int __devexit bcm5830x_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	DBG("=>> %s \n", __FUNCTION__);
	snd_soc_unregister_card(card);
	return 0;
}

#ifdef CONFIG_PM
static int bcm5830x_audio_suspend(struct device *dev)
{
	DBG("--> %s \n", __FUNCTION__);
	return 0;
}
static int bcm5830x_audio_resume(struct device *dev)
{
	DBG("--> %s \n", __FUNCTION__);
	bcm5830x_audio_gpio_init(bcm5830x_i2s_port);
	return 0;
}
static SIMPLE_DEV_PM_OPS(bcm5830x_audio_pm, bcm5830x_audio_suspend,
	bcm5830x_audio_resume);
#endif

static struct platform_driver bcm5830x_audio_driver = {
	.driver		= {
		.name	= "bcm5830x-audio",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &bcm5830x_audio_pm,
#endif

	},
	.probe		= bcm5830x_audio_probe,
	.remove		= __devexit_p(bcm5830x_audio_remove),
};

module_platform_driver(bcm5830x_audio_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("ALSA SoC bcm5830x Audio");
MODULE_LICENSE("GPL");
