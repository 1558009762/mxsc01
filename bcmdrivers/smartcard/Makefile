ifneq ($(KERNELRELEASE),)
  KBUILD_CFLAGS += -DBCMDRIVER -DCYGNUS_SCI_DRV -Dlinux -DBSTD_CPU_ENDIAN=BSTD_ENDIAN_LITTLE -mlong-calls 
  KBUILD_CFLAGS += -D__BASE_FILE_NAME__=\"$*.c\"
  ifeq ($(CONFIG_IPROC_SMARTCARD_DEBUG),y)
#    KBUILD_CFLAGS += -DSCI_DEBUG=1 
  endif
  ifeq ($(CONFIG_IPROC_SMARTCARD_DEBUG),y)
#    KBUILD_CFLAGS += -DBDBG_DEBUG_BUILD=1
  endif
  EXTRA_CFLAGS := -I$(src)/include/
  MODULE_NAME = iProc_SmartCard
  $(MODULE_NAME)-objs := iproc_sci.o bkni_linux.o bscd.o bscd_isopriv.o bscd_priv.o nxp8026_i2c.o nxp8026_ops.o iproc_sci_debug.o
  obj-$(CONFIG_IPROC_SMARTCARD) += $(MODULE_NAME).o
else
all:
	make -C ../../kernel/linux-3.6.5 M=$(PWD)  modules

clean:
	rm -rf .*.cmd  *.o  *.mod.c  *.ko  .tmp_versions .*.o.d *.ko.dump
	rm -rf Module.markers  Module.symvers  modules.order   modules.builtin
	chmod -x *
	chmod +x include

endif
