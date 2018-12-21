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
#include "i2s_regs.h"
#include "bcm535x-i2s.h"
#include <mach/reg_utils.h>
#include <linux/io.h>
#include <mach/io_map.h>

/* MCLK in Hz - to bcm535xx & Wolfson 8955 */
#define BCM535X_MCLK_FREQ 20000000 /* 20 MHz */

//bcm535x_i2s_info_t *snd_bcm535x;
//EXPORT_SYMBOL_GPL(snd_bcm535x);


static int snd_bcm535x_free(struct bcm535x_i2s_info *snd_bcm)
{
	kfree(snd_bcm);
	return 0;
}

static int snd_bcm535x_dev_free(struct snd_device *device)
{
	return snd_bcm535x_free(device->device_data);
}


static int __devinit snd_bcm535x_create(struct snd_card *card,
        struct platform_device *pdev)
{
	int ret;
	static struct snd_device_ops ops = {
		.dev_free = snd_bcm535x_dev_free,
	};

	snd_bcm535x = kzalloc(sizeof(bcm535x_i2s_info_t), GFP_KERNEL);
	if (snd_bcm535x == NULL) {
		DBG("%s: Error allocating snd_bcm535x\n", __FUNCTION__);
		return -ENOMEM;
	}

	snd_bcm535x->card = card;

	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, snd_bcm535x, &ops);
	if (ret < 0) {
		snd_bcm535x_free(snd_bcm535x);
		return ret;
	}

	snd_card_set_dev(card, &pdev->dev);

	return 0;
}

static int __devinit snd_bcm535x_alsa_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_card *card;
		
	DBG("%s: ############ PROBING FOR bcm535x ALSA ###############\n", __FUNCTION__);
	
	ret = snd_card_create(0, "BCM5301x Audio",
                      THIS_MODULE, 0, &card);
	if (ret < 0) {
		DBG("%s: snd_card_create returned %d\n", __FUNCTION__, ret);
       return ret;
	}

	ret = snd_bcm535x_create(card, pdev);
	if (ret < 0) {
		DBG("Failed to create snd_bcm535x\n");
		snd_card_free(card);
		return ret;
	}
	
	snd_bcm535x->card = card;
	
	// Attach i2s interface
	ret = bcm535x_i2s_attach(pdev);

	strcpy(card->driver, "BRCM bcm585x ALSA Driver");
	strcpy(card->shortname, "bcm585x ALSA");
	sprintf(card->longname, "%s", card->shortname);

	ret = bcm535x_pcm_new(snd_bcm535x->card);
	if (ret < 0) {
		DBG("%s: Failed to create new BCM535x pcm device\n", __FUNCTION__);
	}

	ret = snd_card_register(card);
	if (ret < 0) {
		DBG("%s: snd_card_register returns: %d \n", __FUNCTION__, ret);
	}
	
	platform_set_drvdata(pdev, card);
	
	return 0;
}

static int snd_bcm535x_alsa_remove(struct platform_device *pdev)
{
	DBG("%s\n", __FUNCTION__);
	return 0;
}


static int snd_bcm535x_alsa_suspend(struct platform_device *pdev, pm_message_t state)
{
	DBG("%s\n", __FUNCTION__);
	return 0;
}

static int snd_bcm535x_alsa_resume(struct platform_device *pdev)
{
	DBG("%s\n", __FUNCTION__);
	return 0;
}


//static struct snd_device_ops ops = { NULL };

static u64 bcm535x_pcm_dmamask = DMA_BIT_MASK(32);

static struct platform_driver bcm535x_alsa_driver =
{
    .driver = {
        .name = "bcm535x_AUDIO",
        .owner = THIS_MODULE,
    },
	.probe      = snd_bcm535x_alsa_probe,
	.remove     = snd_bcm535x_alsa_remove,
	.suspend    = snd_bcm535x_alsa_suspend,
	.resume     = snd_bcm535x_alsa_resume,
};

static struct platform_device bcm535x_audio_device = {
	.name		=	"bcm535x_AUDIO",
	.id		    =	-1,
	.dev =  {
		//.release = bcm5301x_gmac_release,
		.init_name = "bcm535x_AUDIO",
		.dma_mask = &bcm535x_pcm_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},

};

static int __init bcm535x_init(void)
{
	int ret;
	
	DBG("%s\n", __FUNCTION__);

	DBG("%s  Registering platform driver\n", __FUNCTION__);	
	ret = platform_driver_register(&bcm535x_alsa_driver);
	if (ret) {
		DBG("Error registering bcm535x_alsa_driver %d .\n", ret);
		return ret;
	}

	DBG("%s  Registering platform device\n", __FUNCTION__);	
    ret = platform_device_register(&bcm535x_audio_device);
	if (ret) {
		DBG("Error registering bcm535x_audio_device %d .\n", ret);
	    platform_driver_unregister(&bcm535x_alsa_driver);
	}

	return ret;
}

static void __exit bcm535x_exit(void)
{
	DBG("%s\n", __FUNCTION__);
	platform_device_put(&bcm535x_audio_device);
	platform_driver_unregister(&bcm535x_alsa_driver);
	
	//if (card)
	//	snd_card_free(card);
}

module_init(bcm535x_init);
module_exit(bcm535x_exit);

/* Module information */
MODULE_DESCRIPTION("ALSA SoC bcm535x");
MODULE_LICENSE("GPL");
