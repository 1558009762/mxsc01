#If KERNELRELEASE is defined, we've been invoked from the

# kernel build system and can use its language.
#CC = /opt/cross/eldk42/usr/bin/ppc_82xx-gcc
#CCDEPMODE = depmode=none
#CFLAGS = -g -O2
#CPP = /opt/cross/eldk42/usr/bin/ppc_82xx-gcc -E
#CPPFLAGS = 
#CRLDP = 
#CRLDPINCLUDE = 
#CRLDPLIB = 
#CURSES = 
#2.6.18-8.el5
KERNELDIR ?= ../../../kernel/linux-3.6.5
ifneq ($(KERNELRELEASE),)

obj-m += netlink.o
obj-m += bcmiproc_phy5461s.o
#obj-m += $(KERNELDIR)/../../bcmdrivers
outband-objs := netlink.o

a:

	echo defined	 

# Otherwise we were called directly from the command

# line; invoke the kernel build system.

else

	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	EXTRA_CFLAGS += -I$(KERNELDIR)/../../bcmdrivers/gmac/src/include
	PWD := $(shell pwd) 

default:

	echo no defined

	$(MAKE) -C $(KERNELDIR) $(EXTRA_CFLAGS) M=$(PWD) modules

endif

clean: 
	rm *.ko *.mod.c *.o *.symvers *.order


