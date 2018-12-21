#
# Makefile for the Broadcom et driver
#
# Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
# 
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
# 
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
# SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
# OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
# CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# $Id: Makefile,v 1.5 2010-12-07 04:47:36 $
#


ETSRCDIR := ../src/et

et-objs := $(ETSRCDIR)/sys/et_linux.o $(ETSRCDIR)/sys/etc.o

## from linux dir ##########
export SRCBASE_et := $(src)/$(ETSRCDIR)/sys/../../
KBUILD_CFLAGS += -I$(SRCBASE_et)/include -DBCMDRIVER -Dlinux
KBUILD_AFLAGS += -I$(SRCBASE_et)/include
#################################
obj-$(CONFIG_ET) := et.o

et-objs         += $(ETSRCDIR)/sys/etcgmac.o
EXTRA_CFLAGS	+= -DDMA -Wno-error
EXTRA_CFLAGS    += -DGMAC_RATE_LIMITING -DBCMDMA32 -DBCMDBG_ERR

ifeq ($(CONFIG_BCM_IPROC_GMAC_SG),y)
EXTRA_CFLAGS    += -DBCMDMASGLISTOSL
endif

ifeq ($(CONFIG_ET_ALL_PASSIVE_ON),y)
EXTRA_CFLAGS	+= -DGMAC_ALL_PASSIVE
else
ifeq ($(CONFIG_ET_ALL_PASSIVE_RUNTIME),y)
EXTRA_CFLAGS	+= -DGMAC_ALL_PASSIVE
endif
endif

ifeq ($(CONFIG_ET_NAPI_POLL),y)
EXTRA_CFLAGS	+= -DGMAC_NAPI_POLL
else
ifeq ($(CONFIG_ET_NAPI2_POLL),y)
EXTRA_CFLAGS	+= -DGMAC_NAPI2_POLL
endif
endif

# Search for sources under src/et/sys or objects under src/et/linux
ifneq ($(wildcard $(src)/$(ETSRCDIR)/sys),)
EXTRA_CFLAGS	+= -I$(src)/$(ETSRCDIR)/sys
else
et-objs := $(ETSRCDIR)/linux/et.o
endif

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
else
# Normal makefile
KERNELDIR := ../../kernel/linux
all:
	$(MAKE) -C $(KERNELDIR) M=`pwd`

clean:
	$(MAKE) -C $(KERNELDIR) M=`pwd` clean
endif

clean-files += $(ETSRCDIR)/sys/*.o $(ETSRCDIR)/sys/.*.o.cmd
