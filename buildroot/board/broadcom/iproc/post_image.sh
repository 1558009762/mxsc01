#!/bin/bash

BINARIES_DIR=$1
BUILD_DIR=$BINARIES_DIR/../build
MKIMAGE=$BINARIES_DIR/../../host/usr/bin/mkimage

FDT_NAME=$2.its 

OUTPUT_NAME=uImage_$3.img
if [ -z $3 ]; then
	OUTPUT_NAME=uImage.img
fi

$MKIMAGE -f $BUILD_DIR/linux-custom/arch/arm/boot/dts/$FDT_NAME $BINARIES_DIR/$OUTPUT_NAME

