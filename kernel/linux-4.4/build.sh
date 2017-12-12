#!/bin/sh

export ARCH=arm

export PATH=/opt/arm/brcm-3.8.1/usr/bin:$PATH
export CROSS_COMPILE=arm-linux-

make Image
if [ $? -eq 0 ];then
cp ./arch/arm/boot/Image ../../mkubi/Image
echo cp ./arch/arm/boot/Image ../../mkubi/Image
cd ../../mkubi
./build.sh
else
echo "==========make Image error=========="

fi
#cp ./arch/powerpc/boot/p1020rdb-pc.dtb ../mkubi/bin/1604-dtb
#cp ./arch/powerpc/boot/uImage ../mkubi/bin/uImage

