#
# Copyright 2016 Broadcom Corporation.
#
# SPDX-License-Identifier:	GPL-2.0+
#
ifdef CONFIG_NAND_IPROC_BOOT
CONFIG_SYS_TEXT_BASE = 0x28000000
else
ifdef CONFIG_NOR_IPROC_BOOT
CONFIG_SYS_TEXT_BASE = 0x20000000
else 
CONFIG_SYS_TEXT_BASE = 0x1C000000
endif 
endif

# Big endian
ifdef CONFIG_SYS_BIG_ENDIAN
LDSCRIPT := $(srctree)/board/$(BOARDDIR)/../bcm_xgs/u-boot-l2c-be.lds
else
LDSCRIPT := $(srctree)/board/$(BOARDDIR)/../bcm_xgs/u-boot-l2c.lds
endif

#PLATFORM_RELFLAGS += -DSVN_REVISION=' " $(SVN_REV)"'
