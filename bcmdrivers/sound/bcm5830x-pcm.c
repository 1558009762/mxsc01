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
#include <asm/io.h>

#include "bcm5830x-i2s.h"
#include <mach/socregs-cygnus.h>
#include <linux/timer.h>

static void dma_timer(unsigned long data);

struct bcm5830x_runtime_data {
	spinlock_t lock;
	struct bcm5830x_i2s_info *snd_bcm5830x;
	struct snd_pcm_substream *play_stream;
	struct snd_pcm_substream *rec_stream;
};

static const struct snd_pcm_hardware bcm5830x_pcm_hw = {
	.info			= SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_PAUSE |
		SNDRV_PCM_INFO_RESUME,
	.formats		= 
		SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S32_LE,
	.rates =	  SNDRV_PCM_RATE_8000_192000,
	.rate_min =	  8000,
	.rate_max =	  192000,				  
	.channels_min		= 2,
	.channels_max		= 2,
	.period_bytes_min	= 0x2000,
	.period_bytes_max	= 0x2000,
	.periods_min		= 16,
	.periods_max		= 16,
	.buffer_bytes_max	= 16 * 0x2000,
	.fifo_size		= 0,
};

static uint32_t a_debug = 0;
module_param_named(debug, a_debug, uint, 0644);
MODULE_PARM_DESC(mask, "debug flag");

static DEFINE_TIMER(dma_irq_timer, dma_timer, 0, 0);
static u64 bcm5830x_dma_dmamask = DMA_BIT_MASK(32);

static void dma_timer(unsigned long data)
{
	struct bcm5830x_runtime_data *brtd = (struct bcm5830x_runtime_data *)data;

	if (!brtd)
		return; 

	if (brtd->play_stream) 
		snd_pcm_period_elapsed(brtd->play_stream);

	if (brtd->rec_stream) 
		snd_pcm_period_elapsed(brtd->rec_stream);

	mod_timer(&dma_irq_timer, jiffies + 2);
}

static int bcm5830x_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_platform *platform = soc_runtime->platform;
	struct bcm5830x_runtime_data *brtd;

	snd_soc_set_runtime_hwparams(substream, &bcm5830x_pcm_hw);
	brtd = snd_soc_platform_get_drvdata(platform);

	if (brtd == NULL) {
		brtd = kzalloc(sizeof(struct bcm5830x_runtime_data), GFP_KERNEL);
		if (brtd == NULL)
			return -ENOMEM;

		dma_irq_timer.expires = jiffies + 2;
		dma_irq_timer.data = (int)brtd;
		add_timer(&dma_irq_timer);
		snd_soc_platform_set_drvdata(platform, brtd);
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		brtd->play_stream = substream;
	} else {
		brtd->rec_stream = substream;
	}
	return 0;
}

static int bcm5830x_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_platform *platform = soc_runtime->platform;
	struct bcm5830x_runtime_data *brtd = snd_soc_platform_get_drvdata(platform); 
	
	if (!brtd)
		return 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		brtd->play_stream = NULL;
	else
		brtd->rec_stream = NULL;

	if (!brtd->play_stream && !brtd->rec_stream) {
		del_timer(&dma_irq_timer);
		kfree(brtd);
		snd_soc_platform_set_drvdata(platform, NULL);
		DBG("==> freed \n");
	}
	return 0;
}

static int bcm5830x_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	DBG("==> %s dma_bytes 0x%x \n", __FUNCTION__, runtime->dma_bytes);

	return ret;
}

static int bcm5830x_pcm_hw_free(struct snd_pcm_substream *substream)
{
	DBG("==> %s\n", __FUNCTION__);

	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int bcm5830x_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct bcm5830x_i2s_info *priv;
	uint32_t size, count;
	int ret = 0;
	uint32_t srcrb_base = AUD_FMM_BF_CTRL_SOURCECH_RINGBUF_0_RDADDR_REG_BASE; 
	uint32_t dstrb_base = AUD_FMM_BF_CTRL_DESTCH_RINGBUF_0_RDADDR_REG_BASE; 

	priv = snd_soc_dai_get_dma_data(cpu_dai, substream);
	
	size = frames_to_bytes(runtime, runtime->buffer_size);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		size &= ~0xff;
	count = snd_pcm_lib_period_bytes(substream);

	DBG("==> %s: size = %d count = %d addr %x\n", __FUNCTION__, size, count, 
		runtime->dma_addr);

	/* rd, wr, end, base, */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		writel(runtime->dma_addr,
			priv->audio + srcrb_base + 0x30 * priv->port);
		writel(runtime->dma_addr + size - 1,
			priv->audio + srcrb_base + 0x30 * priv->port + 4);
		writel(runtime->dma_addr,
			priv->audio + srcrb_base + 0x30 * priv->port + 8);
		writel(runtime->dma_addr + size - 1,
			priv->audio + srcrb_base + 0x30 * priv->port + 12);
	} else {
		writel(runtime->dma_addr,
			priv->audio + dstrb_base + 0x30 * priv->port);
		writel(runtime->dma_addr,
			priv->audio + dstrb_base + 0x30 * priv->port + 4);
		writel(runtime->dma_addr,
			priv->audio + dstrb_base + 0x30 * priv->port + 8);
		writel(runtime->dma_addr + size - 1,
			priv->audio + dstrb_base + 0x30 * priv->port + 12);
	}
	return ret;
}


static snd_pcm_uframes_t
bcm5830x_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *soc_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = soc_runtime->cpu_dai;
	struct bcm5830x_i2s_info *priv;
	int res;
	unsigned int rd, base;
	uint32_t srcrb_base = AUD_FMM_BF_CTRL_SOURCECH_RINGBUF_0_RDADDR_REG_BASE; 
	uint32_t dstrb_base = AUD_FMM_BF_CTRL_DESTCH_RINGBUF_0_RDADDR_REG_BASE; 

	priv = snd_soc_dai_get_dma_data(cpu_dai, substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) { 
		rd = readl(priv->audio + srcrb_base + 0x30 * priv->port);
		base = readl(priv->audio + srcrb_base + 0x30 * priv->port + 8);
		res = (rd & 0x7fffffff) - (base & 0x7fffffff);
		//DBG("%s rd %x base %x end %x res %x \n", __FUNCTION__, rd, base, 
		//	readl(priv->audio + srcrb_base + 0x30 * priv->port + 12) , res);
	} else {
		rd = readl(priv->audio + dstrb_base + 0x30 * priv->port + 4);
		base = readl(priv->audio + dstrb_base + 0x30 * priv->port + 8);
		res = (rd & 0x7fffffff) - (base & 0x7fffffff);
		//DBG("%s wr %x base %x end %x res %x \n", __FUNCTION__, rd, base, 
		//	readl(priv->audio + dstrb_base + 0x30 * priv->port + 12) , res);
	}
	return bytes_to_frames(substream->runtime, res);
}

static int bcm5830x_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = bcm5830x_pcm_hw.buffer_bytes_max;
	
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);
 
	DBG("==> %s: size 0x%x @ 0x%p\n", __FUNCTION__, size, buf->area);

	if (!buf->area) {
		DBG("==> %s: dma_alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}
	buf->bytes = size;
	

	return 0;
}
struct snd_pcm_ops bcm5830x_pcm_ops = {
	.open		= bcm5830x_pcm_open,
	.close		= bcm5830x_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= bcm5830x_pcm_hw_params,
	.hw_free	= bcm5830x_pcm_hw_free,
	.prepare	= bcm5830x_pcm_prepare,
	.pointer	= bcm5830x_pcm_pointer,
};



static int bcm5830x_dma_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret;

	DBG("==> Entered %s\n", __FUNCTION__);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &bcm5830x_dma_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = bcm5830x_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			return ret;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = bcm5830x_pcm_preallocate_dma_buffer(pcm,
				SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			return ret;
	}
	return 0;
}

static void bcm5830x_dma_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	DBG("==> Entered %s\n", __FUNCTION__);
	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(pcm->card->dev, buf->bytes,
				buf->area, buf->addr);
		buf->area = NULL;
	}
}
struct snd_soc_platform_driver bcm5830x_soc_platform = {
	.ops		= &bcm5830x_pcm_ops,
	.pcm_new	= bcm5830x_dma_new,
	.pcm_free	= bcm5830x_dma_free_dma_buffers,
};


static int __devinit bcm5830x_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &bcm5830x_soc_platform);
}

static int __devexit bcm5830x_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}
#ifdef CONFIG_PM
static int bcm5830x_pcm_suspend(struct device *dev)
{
	return 0;
}
static int bcm5830x_pcm_resume(struct device *dev)
{
	return 0;
}
static SIMPLE_DEV_PM_OPS(bcm5830x_pcm_pm, bcm5830x_pcm_suspend, bcm5830x_pcm_resume);
#endif

static struct platform_driver bcm5830x_pcm_driver = {
	.driver = {
		.name = "bcm5830x-pcm",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &bcm5830x_pcm_pm,
#endif
	},

	.probe = bcm5830x_soc_platform_probe,
	.remove = __devexit_p(bcm5830x_soc_platform_remove),
};

module_platform_driver(bcm5830x_pcm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("bcm5830x PCM module");
