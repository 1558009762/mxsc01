#!/bin/bash
#
#  COPYRIGHT NOTICE
#  Copyright (C) 2016 HuaHuan Electronics Corporation, Inc. All rights reserved
#
#  Author       	:fzs
#  File Name        	:/home/kevin/works/projects/XLDK-3.6.2/XLDK/buildroot/output/images/mkuimage.sh
#  Create Date        	:2017/01/17 17:31
#  Last Modified      	:2017/01/17 17:31
#  Description    	:
#

cp arch/arm/boot/Image .
mkimage -A arm -O linux -T kernel -n Image -a 0x61008000 -C none -d ./Image ./uImage
cp uImage ../../mkubi/bin

