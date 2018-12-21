#ifndef __BCM5830X_I2S_H__
#define __BCM5830X_I2S_H__

#ifndef DEBUG
#define DEBUG
#endif

#define DBG(fmt, ...) do { \
	if (a_debug) \
		printk(KERN_INFO fmt, ##__VA_ARGS__); \
} while(0)

struct bcm5830x_i2s_info {
	/* ALSA structs. */
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	int			irq;
	uint32_t 	mclk;
	uint32_t 	port;
	void __iomem *audio;
	void __iomem *audioeav;
	void __iomem *chipdru;
	void __iomem *crmudru;
	void __iomem *asiutop;
};

extern int bcm5830x_i2s_port;

#endif
