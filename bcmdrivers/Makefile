# File: bcmdrivers/Makefile
#
# Makefile for the Linux kernel modules.
#

-include $(KERNEL_DIR)/.config

export BCMDRIVERS_DIR:=$(src)

# The following must have config defined for each driver

obj-y += keypad/
obj-y += touch_screen/
obj-y += adc/
obj-y += rtc/
obj-y += timer/
obj-y += otp/
obj-y += bbl/
obj-y += rng/
obj-y += mdio/
obj-y += gsio_spi/
obj-y += sd/
obj-y += qspi/
obj-y += nand/
obj-y += gpio/
obj-y += usb2h/
obj-y += usb3h/
obj-m += usbd/
obj-y += pcie/
obj-$(CONFIG_ET) += gmac/et/
obj-y += gmac/hnd/
obj-y += sound/
obj-y += smbus/
obj-y += sra/
obj-y += ../bcmgpldrivers/

obj-y += pmu/
obj-y += smartcard/
obj-y += spl/
obj-y += dte/
obj-y += smc35x_lcd/
obj-y += video/

## kdrivers integrated at ../kernel/linux-3.6.5/drivers/mtd/Makefile
## requires early compile

clean: 
