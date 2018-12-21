# iproc TDM support

EXTRA_CFLAGS	+= -I$(BCMDRIVERS_DIR)/gmac/src/include
KBUILD_CFLAGS += -DBCMDRIVER -Dlinux

iproc_pmu-objs := iproc-pmu.o

obj-$(CONFIG_IPROC_PMU) += iproc_pmu.o
