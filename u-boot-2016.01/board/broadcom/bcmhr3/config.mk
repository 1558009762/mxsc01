#
# Copyright 2016 Broadcom Corporation.
#
# SPDX-License-Identifier:	GPL-2.0+
#
ifdef CONFIG_NAND_IPROC_BOOT
CONFIG_SYS_TEXT_BASE = 0xE0000000
else 
ifdef CONFIG_NOR_IPROC_BOOT
CONFIG_SYS_TEXT_BASE = 0xE8000000
else 
CONFIG_SYS_TEXT_BASE = 0xF0000000
endif 
endif

# Big endian
ifdef CONFIG_SYS_BIG_ENDIAN
LDSCRIPT := $(srctree)/board/$(BOARDDIR)/../bcm_xgs/u-boot-l2c-be.lds
else
LDSCRIPT := $(srctree)/board/$(BOARDDIR)/../bcm_xgs/u-boot-l2c.lds
endif

#PLATFORM_RELFLAGS += -DSVN_REVISION=' " $(SVN_REV)"'
