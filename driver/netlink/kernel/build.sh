#!/bin/bash

# build ht153
#export KPATH=/home/work/share/vtss-linux/ht153-linux/mscc-linux-605b09f
#export K_SRC_PATH=/home/work/share/vtss-linux/ht153-linux/mscc-linux-605b09f


# build ht157
#export KPATH=/home/work/share/vtss-linux/ht157-linux/mscc-linux-605b09f
#export K_SRC_PATH=/home/work/share/vtss-linux/ht157-linux/mscc-linux-605b09f

export PATH=/opt/arm/usr/bin:$PATH
TARGET_ARCH=arm

rm ./build -rf


make ARCH=arm CROSS_COMPILE=arm-linux-  KERNELDIR=../../../kernel/linux-3.6.5

#mipsel-linux-gcc extern-irq.c -o extern-irq

#cp drv_extern-irq.ko  /tftpboot/
#cp extern-irq  /tftpboot/


