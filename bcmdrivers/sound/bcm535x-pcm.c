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
 * SOC Audio for the Broadcom BCM535X family of I2S cores
 */
 
#include <linux/init.h>
#include <linux/module.h>
#include <typedefs.h>
#include <linux/platform_device.h>
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
#include "i2s_regs.h"
#include "bcm535x-i2s.h"

/* Be careful here... turning on prints can break everything, if you start seeing FIFO underflows
 * then it might be due to excessive printing
 */
#define BCM535X_PCM_DEBUG 1
//#if BCM535X_PCM_DEBUG
//#define DBG(x...) printk(KERN_ERR x)
//#else
//#define DBG(x...)
//#endif

#define BCM_I2S_RUNNING		(1<<0)

extern void *baseAddr;

static dma_addr_t lastRxPos;
static dma_addr_t lastlastRxPos;
	
static const struct snd_pcm_hardware bcm535x_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
	/*				  SNDRV_PCM_INFO_BLOCK_TRANSFER | */
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_U16_LE |
				  SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S20_3LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S24_3LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.rates =	  SNDRV_PCM_RATE_8000_192000,
	.rate_min =	  8000,
	.rate_max =	  192000,				  
	.channels_min		= 2,
	.channels_max		= 2,
	.period_bytes_min	= 32,
	.period_bytes_max	= 4096,
	//.periods_min		= 1,
	.periods_min		= 2,
	.periods_max		= 64,
	.buffer_bytes_max	= 256 * 1024,
	//.buffer_bytes_max	= 128 * 1024,
	.fifo_size		= 128,
};

struct bcm535x_runtime_data {
	spinlock_t lock;
	bcm535x_i2s_info_t *snd_bcm535x;
	unsigned int dma_loaded;
	unsigned int rxdma_loaded;
	unsigned int dma_limit;
	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	dma_addr_t dma_rxstart;
	dma_addr_t dma_rxpos;
	dma_addr_t dma_rxend;
	uint state;
	hnddma_t	*di[1];		/* hnddma handles, per fifo */
};

struct snd_pcm_substream *my_stream;

static void bcm535x_pcm_enqueue_tx(struct snd_pcm_substream *substream)
{
	struct bcm535x_runtime_data *brtd = substream->runtime->private_data;
	dma_addr_t pos = brtd->dma_pos;
	int i, ret = 0;
	int16_t *rx_buffer;
	int16_t *tx_buffer;	
		

	while (brtd->dma_loaded < brtd->dma_limit) {
		unsigned long len = brtd->dma_period;

		if ((pos & ~0xFFF) != (((pos+len - 1) & ~0xFFF))) {
			len = ((pos+len) & ~0xFFF) - pos;
		}

		if ((pos + len) > brtd->dma_end) {
			len  = brtd->dma_end - pos;
		}
	
		//DBG("%s: txunframed with pos: %p len: %d\n", __FUNCTION__, pos, len);
#if defined RX_LOOPBACK		
		// Loopback from RX buffer
		if (lastlastRxPos != 0) {
			tx_buffer = (int16_t *)pos;
			rx_buffer = (int16_t *)lastlastRxPos;
			for (i = 0; i < len; i++)
				tx_buffer[i] = rx_buffer[i+2];	// Header (32-bits) contains no of bytes received
		}
#endif
		//DBG("T:%d", len);
							
		ret = dma_txunframed(snd_bcm535x->di[0], (void *)pos, len, TRUE);
		//ret = dma_txunframed(snd_bcm535x->di[0], (void *)start, len, TRUE);
		
		//DBG("%s txunframed returns: %d\n", __FUNCTION__, ret);

		if (ret == 0) {
			pos += len;
			brtd->dma_loaded++;
			if (pos >= brtd->dma_end)
				pos = brtd->dma_start;
		} else
			break;
	}

	brtd->dma_pos = pos;
	
}

static void bcm535x_pcm_enqueue_rx(struct snd_pcm_substream *substream)
{
	struct bcm535x_runtime_data *brtd = substream->runtime->private_data;
	dma_addr_t rxpos = brtd->dma_rxpos;
	int ret = 0;
	

	while (brtd->rxdma_loaded < brtd->dma_limit) {
		unsigned long len = brtd->dma_period;

		if ((rxpos & ~0xFFF) != (((rxpos+len - 1) & ~0xFFF))) {
			len = ((rxpos+len) & ~0xFFF) - rxpos;
		}

		if ((rxpos + len) > brtd->dma_rxend) {
			len  = brtd->dma_rxend - rxpos;
		}
	
		//DBG("%s: txunframed with pos: %p len: %d\n", __FUNCTION__, pos, len);
		//DBG("R:%d", len);

		ret = dma_rxunframed(snd_bcm535x->di[0], (void *)rxpos, len, TRUE);
		//ret = dma_txunframed(snd_bcm535x->di[0], (void *)start, len, TRUE);
		
		//DBG("%s txunframed returns: %d\n", __FUNCTION__, ret);

		if (ret == 0) {
			rxpos += len;
			brtd->rxdma_loaded++;
			if (rxpos >= brtd->dma_rxend)
				rxpos = brtd->dma_rxstart;
		} else
			break;
	}

	brtd->dma_rxpos = rxpos;
	lastlastRxPos = lastRxPos;
	lastRxPos = rxpos;	
	
}


static void bcm535x_pcm_enqueue(struct snd_pcm_substream *substream)
{		
	//DBG("%s\n", __FUNCTION__);

#if defined ENABLE_TX
	bcm535x_pcm_enqueue_tx(substream);
#endif	

#if defined ENABLE_RX
	bcm535x_pcm_enqueue_rx(substream);
#endif	
}

irqreturn_t bcm535x_i2s_isr(int irq, void *devid)
{
	uint32 intstatus = readl(baseAddr + I2S_INTSTATUS_REG);
	uint32 intmask = readl(baseAddr + I2S_INTMASK_REG);
	uint32 intstatus_new = 0;
	uint32 int_errmask = I2S_INT_DESCERR | I2S_INT_DATAERR | I2S_INT_DESC_PROTO_ERR |
	        I2S_INT_RCVFIFO_OFLOW | I2S_INT_XMTFIFO_UFLOW | I2S_INT_SPDIF_PAR_ERR;
	struct bcm535x_runtime_data *brtd = my_stream->runtime->private_data;

	//DBG("Isr: 0x%x\n", intstatus);
	
	if (intstatus & I2S_INT_XMT_INT) {						// TX Interrupt
		//DBG("T");	

		/* reclaim descriptors that have been TX'd */
		dma_getnexttxp(snd_bcm535x->di[0], HNDDMA_RANGE_TRANSMITTED);

		/* clear this bit by writing a "1" back, we've serviced this */
		intstatus_new |= I2S_INT_XMT_INT;

		// Disable interrupts
		writel(readl(baseAddr + I2S_INTMASK_REG) & ~I2S_INT_XMT_INT, baseAddr + I2S_INTMASK_REG);
	
		spin_lock(&brtd->lock);
		brtd->dma_loaded--;
		//if (brtd->state & BCM_I2S_RUNNING) {
			bcm535x_pcm_enqueue_tx(my_stream);
		//}
		spin_unlock(&brtd->lock);
		
		writel(intstatus_new, baseAddr + I2S_INTSTATUS_REG);
	
		/* Enable Tx interrupt */
		writel(intmask | I2S_INT_XMT_INT, baseAddr + I2S_INTMASK_REG);	
	}
	
	if (intstatus & I2S_INT_RCV_INT) {						// RX Interrupt
		//DBG("R");	

		/* reclaim descriptors that have been RX'd */
		dma_getnextrxp(snd_bcm535x->di[0], 0);

		/* clear this bit by writing a "1" back, we've serviced this */
		intstatus_new |= I2S_INT_RCV_INT;

		// Disable interrupts
		writel(readl(baseAddr + I2S_INTMASK_REG) & ~I2S_INT_RCV_INT, baseAddr + I2S_INTMASK_REG);
	
		spin_lock(&brtd->lock);
		brtd->rxdma_loaded--;
		//if (brtd->state & BCM_I2S_RUNNING) {
			bcm535x_pcm_enqueue_rx(my_stream);
		//}
		spin_unlock(&brtd->lock);
		
		writel(intstatus_new, baseAddr + I2S_INTSTATUS_REG);
	
		/* Enable Rx interrupt */
		writel(intmask | I2S_INT_RCV_INT, baseAddr + I2S_INTMASK_REG);
	}
	
	if (intstatus & int_errmask) {
		DBG("\n\n%s: Turning off all interrupts due to error\n", __FUNCTION__);
		DBG("%s: intstatus 0x%x intmask 0x%x\n", __FUNCTION__, intstatus, intmask);

		writel(0, baseAddr + I2S_INTMASK_REG);
	}

	snd_pcm_period_elapsed(my_stream);
	
	return IRQ_RETVAL(intstatus);
}

static int bcm535x_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct bcm535x_runtime_data *brtd = substream->runtime->private_data;
	struct snd_soc_dai *cpu_dai;
	int ret = 0;

	DBG("%s w/cmd %d\n", __FUNCTION__, cmd);
	
	cpu_dai = snd_bcm535x->cpu_dai;

	spin_lock(&brtd->lock);
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	ret = cpu_dai->driver->ops->trigger(substream, cmd, cpu_dai);
#else
	ret = cpu_dai->ops->trigger(substream, cmd, cpu_dai);
#endif	
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:	
		brtd->state |= BCM_I2S_RUNNING;		
		bcm535x_pcm_enqueue(my_stream);	
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		brtd->state &= ~BCM_I2S_RUNNING;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	spin_unlock(&brtd->lock);

	return ret;
}

static int bcm535x_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm535x_runtime_data *brtd;

	DBG("%s\n", __FUNCTION__);

	//snd_soc_set_runtime_hwparams(substream, &bcm535x_pcm_hardware);

	brtd = kzalloc(sizeof(struct bcm535x_runtime_data), GFP_KERNEL);
	if (brtd == NULL) {
		return -ENOMEM;
	}
	brtd->snd_bcm535x = snd_bcm535x;

	spin_lock_init(&brtd->lock);

	runtime->private_data = brtd;
	runtime->hw = bcm535x_pcm_hardware;
	
#if defined ENABLE_TX
	/* probably should put this somewhere else, after setting up isr ??? */
	dma_txreset(snd_bcm535x->di[0]);
	dma_txinit(snd_bcm535x->di[0]);
#endif	
#if defined ENABLE_RX
	dma_rxreset(snd_bcm535x->di[0]);
	dma_rxinit(snd_bcm535x->di[0]);
	//dma_rxfill(snd_bcm535x->di[0]);
#endif
	
	DBG("%s: i2s devcontrol 0x%x devstatus 0x%x\n", __FUNCTION__,
	    readl(baseAddr + I2S_DEVCONTROL_REG),
	    readl(baseAddr + I2S_DEVSTATUS_REG));
	DBG("%s: i2s intstatus 0x%x intmask 0x%x\n", __FUNCTION__,
	    readl(baseAddr + I2S_INTSTATUS_REG),
	    readl(baseAddr + I2S_INTMASK_REG));
	DBG("%s: i2s control 0x%x\n", __FUNCTION__,
	    readl(baseAddr + I2S_CONTROL_REG));
	DBG("%s: i2s clkdivider 0x%x txplayth 0x%x\n", __FUNCTION__,
	    readl(baseAddr + I2S_CLOCKDIVIDER_REG),
	    readl(baseAddr + I2S_TXPLAYTH_REG));
	DBG("%s: i2s stxctrl 0x%x\n", __FUNCTION__,
	    readl(baseAddr + I2S_STXCTRL_REG));

	{
	uint32 temp;
	temp = readl(baseAddr + I2S_FIFOCOUNTER_REG);
	DBG("%s: i2s txcnt 0x%x rxcnt 0x%x\n", __FUNCTION__,
	    (temp & I2S_FC_TX_CNT_MASK)>> I2S_FC_TX_CNT_SHIFT,
	    (temp & I2S_FC_RX_CNT_MASK)>> I2S_FC_RX_CNT_SHIFT);
	}

	DBG("%s: i2s devcontrol 0x%x devstatus 0x%x\n", __FUNCTION__,
	    R_REG(snd_bcm535x->osh, &snd_bcm535x->regs->devcontrol),
	    R_REG(snd_bcm535x->osh, &snd_bcm535x->regs->devstatus));

	my_stream = substream;
	
	lastRxPos = 0;
	lastlastRxPos = 0;

	DBG("%s: i2s devcontrol 0x%x devstatus 0x%x fifocounter 0x%x stxctrl 0x%x stxstatus0 0x%x stxstatus1 0x%x\n", __FUNCTION__,
	    readl(baseAddr + I2S_DEVCONTROL_REG),
	    readl(baseAddr + I2S_DEVSTATUS_REG),
	    readl(baseAddr + I2S_FIFOCOUNTER_REG),	
		readl(baseAddr + I2S_STXCTRL_REG),
		readl(baseAddr + I2S_STXCHSTATUS0_REG),
		readl(baseAddr + I2S_STXCHSTATUS1_REG));

	return 0;
}

static int bcm535x_pcm_close(struct snd_pcm_substream *substream)
{
	struct bcm535x_runtime_data *brtd = substream->runtime->private_data;
	
	DBG("%s\n", __FUNCTION__);

	/* Turn off interrupts... */
	writel(readl(baseAddr + I2S_INTMASK_REG) & ~I2S_INT_RCV_INT, baseAddr + I2S_INTMASK_REG);
	writel(readl(baseAddr + I2S_INTMASK_REG) & ~I2S_INT_XMT_INT, baseAddr + I2S_INTMASK_REG);

	bcm535x_pcm_trigger(substream, SNDRV_PCM_TRIGGER_STOP);	


	/* reclaim all descriptors */
	dma_txreclaim(snd_bcm535x->di[0], HNDDMA_RANGE_ALL);

	if (brtd)
		kfree(brtd);
	else
		DBG("%s: called with brtd == NULL\n", __FUNCTION__);

	return 0;
}

static int bcm535x_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm535x_runtime_data *brtd = runtime->private_data;
	unsigned long totbytes = params_buffer_bytes(params);
	uint32_t channels, rate, bits;
	unsigned int fmt;
	struct snd_soc_dai *cpu_dai;
	
	int ret = 0;
	
	size_t buffer_size = params_buffer_size(params);
	size_t buffer_bytes = params_buffer_bytes(params);
	size_t period_size = params_period_size(params);
	size_t period_bytes = params_period_bytes(params);
	//size_t periods = params_periods(params);
	
	DBG("%s\n", __FUNCTION__);
	
	channels = params_channels(params);
	rate = params_rate(params);
	bits = snd_pcm_format_width(params_format(params));
		
	cpu_dai = snd_bcm535x->cpu_dai;
	
	/* set codec DAI configuration */
	DBG("%s: calling cpu_dai hw_params\n", __FUNCTION__);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	ret = cpu_dai->driver->ops->hw_params(substream, params, cpu_dai);
#else
	ret = cpu_dai->ops->hw_params(substream, params, snd_bcm535x);
#endif	
	
	if (ret < 0) {
		DBG("%s: dai_ops.hw_params returns 0x%x\n", __FUNCTION__, ret);
		return ret;
	}
	
#if defined I2S_MASTER	
	fmt = SND_SOC_DAIFMT_I2S |				/* I2S mode audio */
	      //SND_SOC_DAIFMT_IB_IF |			/* BCLK inverted and inverted LRCLK polarity */
	      //SND_SOC_DAIFMT_NB_IF |			/* BCLK not inverted and inverted LRCLK polarity */
	      //SND_SOC_DAIFMT_IB_NF |			/* BCLK inverted and normal LRCLK polarity */
	      SND_SOC_DAIFMT_NB_NF |			/* BCLK not inverted and normal LRCLK polarity */
	      SND_SOC_DAIFMT_CBS_CFS;			/* BCM is I2S Master */
	DBG("%s: I2S MASTER\n", __FUNCTION__);		  
#else
	fmt = SND_SOC_DAIFMT_I2S |				/* I2S mode audio */
	      SND_SOC_DAIFMT_NB_NF |			/* BCLK not inverted and normal LRCLK polarity */
	      SND_SOC_DAIFMT_CBM_CFM;			/* BCM is I2S Slave */
	DBG("%s: I2S SLAVE\n", __FUNCTION__);	
#endif	
	
	DBG("%s: calling set_fmt with fmt 0x%x\n", __FUNCTION__, fmt);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)		
	ret = cpu_dai->driver->ops->set_fmt(cpu_dai, fmt);
#else
	ret = cpu_dai->ops->set_fmt(snd_bcm535x, fmt);
#endif	
	if (ret < 0) {
		DBG("%s: dai_ops.set_fmt returns 0x%x\n", __FUNCTION__, ret);	
		return ret;
	}

	//my_stream = substream;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totbytes;

	spin_lock_irq(&brtd->lock);
	brtd->dma_limit = runtime->hw.periods_min;
	brtd->dma_period = params_period_bytes(params);
	/* Virtual address of our runtime buffer */
	brtd->dma_start = (dma_addr_t)runtime->dma_area;
	brtd->dma_loaded = 0;
	brtd->dma_pos = brtd->dma_start;
	brtd->dma_end = brtd->dma_start + totbytes;
	// RX
	brtd->dma_rxstart = brtd->dma_end;
	brtd->rxdma_loaded = 0;	
	brtd->dma_rxpos = brtd->dma_rxstart;	
	brtd->dma_rxend = brtd->dma_rxstart + totbytes;
	spin_unlock_irq(&brtd->lock);
	// RX same buffer as TX for loopback
	//brtd->dma_rxstart = brtd->dma_start;
	//brtd->dma_rxend = brtd->dma_rxstart + totbytes;
	//brtd->dma_rxpos = brtd->dma_rxstart;
	//spin_unlock_irq(&brtd->lock);
	//size_t tick_time = params_tick_time(params);

	DBG("%s: hw.periods_min %d dma_addr %p dma_bytes %d\n",
	    __FUNCTION__, runtime->hw.periods_min, (void *)runtime->dma_addr, runtime->dma_bytes);
	DBG("%s: buffer_size 0x%x buffer_bytes 0x%x\n", __FUNCTION__, buffer_size, buffer_bytes);
	DBG("%s: period_size 0x%x period_bytes 0x%x\n", __FUNCTION__, period_size, period_bytes);
	//DBG("%s: periods 0x%x tick_time0x%x\n", __FUNCTION__, periods, tick_time);

	DBG("%s: i2s devcontrol 0x%x devstatus 0x%x fifocounter 0x%x stxctrl 0x%x stxstatus0 0x%x stxstatus1 0x%x\n", __FUNCTION__,
	    readl(baseAddr + I2S_DEVCONTROL_REG),
	    readl(baseAddr + I2S_DEVSTATUS_REG),
	    readl(baseAddr + I2S_FIFOCOUNTER_REG),	
		readl(baseAddr + I2S_STXCTRL_REG),
		readl(baseAddr + I2S_STXCHSTATUS0_REG),
		readl(baseAddr + I2S_STXCHSTATUS1_REG));		

	return ret;
}

static int bcm535x_pcm_hw_free(struct snd_pcm_substream *substream)
{
	DBG("%s\n", __FUNCTION__);
	
	/* Turn off interrupts... */
	writel(readl(baseAddr + I2S_INTMASK_REG) & ~I2S_INT_XMT_INT, baseAddr + I2S_INTMASK_REG);
	writel(readl(baseAddr + I2S_INTMASK_REG) & ~I2S_INT_RCV_INT, baseAddr + I2S_INTMASK_REG);
	
	snd_pcm_set_runtime_buffer(substream, NULL);
	my_stream = NULL;

	return 0;
}

static int bcm535x_pcm_prepare(struct snd_pcm_substream *substream)
{
	uint32 intmask = readl(baseAddr + I2S_INTMASK_REG);
	int ret = 0;
	
	DBG("%s\n", __FUNCTION__);

	// Clear all interrupt masks
	writel(0, baseAddr + I2S_INTMASK_REG);
	
	// Temporary call to trigger.
	bcm535x_pcm_trigger(substream, SNDRV_PCM_TRIGGER_START);		// Test mode, output from local buffer, no data from alsa.

#if defined ENABLE_TX		
	/* Turn on Tx interrupt */
	writel(intmask | I2S_INT_XMT_INT, baseAddr + I2S_INTMASK_REG);
#endif	
#if defined ENABLE_RX
	/* Turn on RX interrupt */
	writel(intmask | I2S_INT_RCV_INT, baseAddr + I2S_INTMASK_REG);		
#endif

	/* Do not enqueue yet, trigger will enqueue */
	/* enqueue dma buffers */
	//bcm535x_pcm_enqueue(substream);

	return ret;
}


static snd_pcm_uframes_t
bcm535x_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct bcm535x_runtime_data *brtd = runtime->private_data;
	unsigned long res;
	
	DBG("%s\n", __FUNCTION__);

	spin_lock(&brtd->lock);

	res = brtd->dma_pos - brtd->dma_start;
	
	// Queue next frame
	brtd->dma_loaded--;
	//if (brtd->state & BCM_I2S_RUNNING) {
	//	bcm535x_pcm_enqueue(my_stream);
	//}

	spin_unlock(&brtd->lock);

	return bytes_to_frames(substream->runtime, res);
}

static int bcm535x_pcm_copy(struct snd_pcm_substream *substream, int channel,
        snd_pcm_uframes_t pos, void *src, snd_pcm_uframes_t count)
{
	
	DBG("%s\n", __FUNCTION__);
	
	return 0;
}

/* Currently unused... memory mapping is automatically done in the dma code */
static int bcm535x_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	DBG("Entered %s\n", __FUNCTION__);
	return 0;
}

struct snd_pcm_ops bcm535x_pcm_ops = {
	.open		= bcm535x_pcm_open,
	.close		= bcm535x_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= bcm535x_pcm_hw_params,
	.hw_free	= bcm535x_pcm_hw_free,
	.prepare	= bcm535x_pcm_prepare,
	.trigger	= bcm535x_pcm_trigger,
	.pointer	= bcm535x_pcm_pointer,
	.mmap		= bcm535x_pcm_mmap,
	.copy    	= bcm535x_pcm_copy,
};

static int bcm535x_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{

	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = bcm535x_pcm_hardware.buffer_bytes_max;
	
	// Test
	int i, j;
	//int16_t sin_1000[8] = {0x0, 0x5a81, 0x7fff, 0x5a81, 0x0, 0xa57f, 0x8000, 0xa57f};
#if defined TX_TEST_SINEWAVE	
	int16_t sin_500[16] = {0x0, 0x30fb, 0x5a81, 0x7640, 0x7fff, 0x7640, 0x5a81, 0x30fb, 0x0, 0xcf05, 0xa57f, 0x89c0, 0x8000, 0x89c0, 0xa57f, 0xcf05};	
	int16_t sin_500_shift[16] = {0x7fff, 0x7640, 0x5a81, 0x30fb, 0x0, 0xcf05, 0xa57f, 0x89c0, 0x8000, 0x89c0, 0xa57f, 0xcf05, 0x0, 0x30fb, 0x5a81, 0x7640};
#endif	

	int16_t *buffer;

	DBG("%s\n", __FUNCTION__);
	
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = kmalloc(size, GFP_ATOMIC);
	DBG("%s: size %d @ 0x%p\n", __FUNCTION__, size, buf->area);

	if (!buf->area) {
		DBG("%s: dma_alloc failed\n", __FUNCTION__);
		return -ENOMEM;
	}
	buf->bytes = size;
	
	buffer = (int16_t *)buf->area;

	// Test, populate buffer with 500Hz sine wave
	// Assume S16_LE format and 8000Hz sampling frequency
	i = 0;
	j = 0;
	
	while (i < size/4)  // No of words per channel
	{
		for (j = 0; j < 16; j++) {
#if defined TX_TEST_SINEWAVE		
			buffer[i++] = sin_500[j];				// Attenuate signal (x/8)
			buffer[i++] = sin_500_shift[j];			// Attenuate signal (x/8), Invert one channel
#else
			buffer[i++] = 0;						// Attenuate signal (x/8)
			buffer[i++] = 0;						// Attenuate signal (x/8), Invert one channel
#endif			
		}
	}

	return 0;
}


int bcm535x_pcm_new(struct snd_card *card)
{
	int ret = 0;
	struct snd_pcm *pcm;
	
	DBG("%s\n", __FUNCTION__);

	ret = snd_pcm_new(card, "bcm535x ALSA", 0, 8, 8, &pcm);
	if (ret < 0) { 
		DBG("%s: snd_pcm_new returned %d\n", __FUNCTION__, ret);
		return ret;
	}

	pcm->private_data = snd_bcm535x;
	strcpy(pcm->name, "bcm535x ALSA");
	snd_bcm535x->pcm = pcm;
	
	/* set operators */
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
	    &bcm535x_pcm_ops);
		
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
	    &bcm535x_pcm_ops);
	
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)	
	if (snd_bcm535x->cpu_dai->driver->playback.channels_min) {
#else
	if (snd_bcm535x->cpu_dai->playback.channels_min) {
#endif	
		ret = bcm535x_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if ((request_irq(snd_bcm535x->irq,
	                 bcm535x_i2s_isr, IRQF_SHARED, "i2s", snd_bcm535x)) < 0) {
		DBG("%s: request_irq failure\n", __FUNCTION__);
	}

 out:
	return ret;
}

struct snd_soc_platform bcm535x_soc_platform = {
	.name		= "bcm535x-audio",
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)		
	.pcm_ops 	= &bcm535x_pcm_ops,
	.pcm_new	= bcm535x_pcm_new,
	.pcm_free	= bcm535x_pcm_free,
#endif	
};

EXPORT_SYMBOL_GPL(bcm535x_soc_platform);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("bcm535x PCM module");
