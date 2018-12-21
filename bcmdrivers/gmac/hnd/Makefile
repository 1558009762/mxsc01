#
# Makefile for Broadcom Home Networking Division (HND) shared driver code
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
# $Id: Makefile,v 1.5 2008-05-02 22:49:54 $
#

SHARED   := ../src/shared

## from linux dir ##########
export SRCBASE_hnd := $(src)/$(SHARED)/../
KBUILD_CFLAGS += -I$(SRCBASE_hnd)/include -DBCMDRIVER -Dlinux
KBUILD_AFLAGS += -I$(SRCBASE_hnd)/include
#################################
obj-$(CONFIG_HND) := hnd.o

EXTRA_CFLAGS += -DBCMDBG_ERR -DBCMDMA32

ifeq ($(CONFIG_BCM_IPROC_GMAC_SG),y)
EXTRA_CFLAGS    += -DBCMDMASGLISTOSL
endif

HND_OBJS += $(src)/$(SHARED)/nvramstubs.o
hnd-objs += $(SHARED)/nvramstubs.o

HND_OBJS += $(src)/$(SHARED)/hnddma.o
hnd-objs += $(SHARED)/hnddma.o

HND_OBJS += $(src)/$(SHARED)/bcmutils.o
hnd-objs += $(SHARED)/bcmutils.o

HND_OBJS += $(src)/$(SHARED)/linux_osl.o
hnd-objs += $(SHARED)/linux_osl.o

HND_OBJS += $(src)/$(SHARED)/siutils.o
hnd-objs += $(SHARED)/siutils.o

HND_OBJS += $(src)/$(SHARED)/aiutils.o
hnd-objs += $(SHARED)/aiutils.o


ifeq ($(CONFIG_MACH_HX4),y)
HND_OBJS += $(src)/$(SHARED)/bcmiproc_serdes.o
hnd-objs += $(SHARED)/bcmiproc_serdes.o

HND_OBJS += $(src)/$(SHARED)/bcmiproc_phy5461s.o
hnd-objs += $(SHARED)/bcmiproc_phy5461s.o

HND_OBJS += $(src)/$(SHARED)/hx4_erom.o
hnd-objs += $(SHARED)/hx4_erom.o
endif

ifeq ($(CONFIG_MACH_SB2),y)
HND_OBJS += $(src)/$(SHARED)/bcmiproc_serdes.o
hnd-objs += $(SHARED)/bcmiproc_serdes.o

HND_OBJS += $(src)/$(SHARED)/bcmiproc_phy5461s.o
hnd-objs += $(SHARED)/bcmiproc_phy5461s.o

HND_OBJS += $(src)/$(SHARED)/sb2_erom.o
hnd-objs += $(SHARED)/sb2_erom.o
endif

ifeq ($(CONFIG_MACH_KT2),y)
HND_OBJS += $(src)/$(SHARED)/bcmiproc_serdes.o
hnd-objs += $(SHARED)/bcmiproc_serdes.o

HND_OBJS += $(src)/$(SHARED)/bcmiproc_phy5461s.o
hnd-objs += $(SHARED)/bcmiproc_phy5461s.o

HND_OBJS += $(src)/$(SHARED)/kt2_erom.o
hnd-objs += $(SHARED)/kt2_erom.o
endif

ifeq ($(CONFIG_MACH_HR2),y)
HND_OBJS += $(src)/$(SHARED)/bcmiproc_phy5221.o
hnd-objs += $(SHARED)/bcmiproc_phy5221.o

HND_OBJS += $(src)/$(SHARED)/hr2_erom.o
hnd-objs += $(SHARED)/hr2_erom.o
endif

ifeq ($(CONFIG_MACH_GH),y)
HND_OBJS += $(src)/$(SHARED)/bcmiproc_phy5481.o
hnd-objs += $(SHARED)/bcmiproc_phy5481.o

HND_OBJS += $(src)/$(SHARED)/gh_erom.o
hnd-objs += $(SHARED)/gh_erom.o
endif

ifeq ($(CONFIG_MACH_HR3),y)
HND_OBJS += $(src)/$(SHARED)/bcmiproc_phy5481.o
hnd-objs += $(SHARED)/bcmiproc_phy5481.o

HND_OBJS += $(src)/$(SHARED)/hr3_erom.o
hnd-objs += $(SHARED)/hr3_erom.o
endif

#$(src)/shared_ksyms.c: $(src)/shared_ksyms.sh $(HND_OBJS)
#	sh -e $< $(HND_OBJS) > $@

hnd-objs += shared_ksyms.o
