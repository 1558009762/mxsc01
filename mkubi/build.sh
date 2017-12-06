#!/bin/bash
#
#  COPYRIGHT NOTICE
#  Copyright (C) 2016 HuaHuan Electronics Corporation, Inc. All rights reserved
#
#  Author       	:Kevin_fzs
#  File Name        	:/home/kevin/works/projects/H20PN-2000/mkubi\build.sh
#  Create Date        	:2016/07/13 10:11
#  Last Modified      	:2016/07/13 10:11
#  Description    	:
#
mkimage -A arm -O linux -T kernel -n Image -a 0x61008000 -C none -d ./Image ./uImage

cp uImage /tftpboot/
exit
read -p "Input kernel version:" KVER
./padmd5 bin/uImage $KVER 201 201

cp ../ramdisk/rootfs-ht201 ./bin/rootfs
mkfs.ubifs -r ./bin -m 2048 -e 126976  -c  6979 -o ht201_main_kernel 

cp ../ramdisk/rootfs-ht201e ./bin/rootfs
mkfs.ubifs -r ./bin -m 2048 -e 126976  -c  6979 -o ht201e_main_kernel 

cp ht201e_main_kernel  /tftpboot/
cp ht201_main_kernel  /tftpboot/

