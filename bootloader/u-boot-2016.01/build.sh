#!/bin/bash
#
#  COPYRIGHT NOTICE
#  Copyright (C) 2016 HuaHuan Electronics Corporation, Inc. All rights reserved
#
#  Author       	:fzs
#  File Name        	:/home/kevin/works/projects/XLDK-3.6.2/XLDK/bootloader/u-boot-2012.10-rc3/build.sh
#  Create Date        	:2017/01/19 15:22
#  Last Modified      	:2017/01/19 15:22
#  Description    	:
#
export PATH=$PATH:/opt/arm/brcm-3.8.1/usr/bin
cd ../../buildroot
#make saber2-uboot_defconfig
make brcm_katana2-uboot_defconfig
make clean
make
#cp output/images/u-boot.bin /mnt/hgfs/D/tmp/u-boot-saber2.bin-2
cp output/images/u-boot.bin /tftpboot/u-boot-katana2.bin
echo "cp u-boot-katana2.bin to tftpboot OK"

