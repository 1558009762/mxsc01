#
# Copyright 2014 Broadcom Corporation.
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y	+= armpll.o
obj-y	+= hwinit-common.o
obj-y	+= timer.o
obj-y	+= genpll.o ddr_init.o benchscreen.o
ifdef CONFIG_SHMOO_AND28_REUSE
obj-y   += shmoo_and28_reuse.o
obj-y   += shmoo_and28/
endif
ifdef CONFIG_RUN_DDR_SHMOO2
obj-y   += shmoo_sig.o
obj-y   += shmoo2/
endif
ifdef CONFIG_SPL_BUILD
obj-y   += spl.o
endif
