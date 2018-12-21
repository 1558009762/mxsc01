ifneq ($(KERNELRELEASE),)
#  KBUILD_CFLAGS += -DBCMDRIVER -Dlinux -DBDBG_DEBUG_BUILD=0 -DBSTD_CPU_ENDIAN=BSTD_ENDIAN_LITTLE -mlong-calls 
  MODULE_NAME = iProc_SPL
  $(MODULE_NAME)-objs := iproc_spl.o iproc_spl_sysfs.o
  obj-$(CONFIG_IPROC_SPL) += $(MODULE_NAME).o
else
all:
	make -C ../../kernel/linux-3.6.5 M=$(PWD)  modules

clean:
	rm -rf .*.cmd  *.o  *.mod.c  *.ko  .tmp_versions .*.o.d *.ko.dump
	rm -rf Module.markers  Module.symvers  modules.order   modules.builtin
	chmod -x *

endif
