# bcm535x support

EXTRA_CFLAGS	+= -I$(BCMDRIVERS_DIR)/gmac/src/include
KBUILD_CFLAGS += -DBCMDRIVER -Dlinux

iproc_snd-objs := bcm535x.o bcm535x-i2s.o bcm535x-pcm.o

obj-$(CONFIG_IPROC_SND) += iproc_snd.o

snd-soc-bcm5830x-pcm-objs := bcm5830x-pcm.o
snd-soc-bcm5830x-i2s-objs := bcm5830x-i2s.o
snd-soc-bcm5830x-svk-objs := bcm5830x.o

obj-$(CONFIG_SND_BCM5830X_SOC) += snd-soc-bcm5830x-pcm.o
obj-$(CONFIG_SND_BCM5830X_SOC_I2S) += snd-soc-bcm5830x-i2s.o
obj-$(CONFIG_SND_BCM5830X_SVK) += snd-soc-bcm5830x-svk.o
