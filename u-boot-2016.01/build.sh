#########################################################################
# File Name: build.sh
# Author: lihz
# mail: lihz@bjhuahuan.com 
# Created Time: 2018年09月06日 星期四 11时53分36秒
#########################################################################
#!/bin/bash


export export CROSS_COMPILE=/opt/arm/usr/bin/arm-linux-
export ARCH=arm

#make O=./output distclean
make O=./output bcm956450k_defconfig
make O=./output all

#cp ./output/u-boot.bin /home/work/tftpboot/
cp ./output/u-boot.bin /home/work/public/tftpd64/

