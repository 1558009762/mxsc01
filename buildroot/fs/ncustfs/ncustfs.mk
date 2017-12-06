################################################################################
#
# Build the ubifs root filesystem image
#
################################################################################

NCUSTFS_OPTS := -e $(BR2_TARGET_ROOTFS_NCUSTFS_LEBSIZE) -c $(BR2_TARGET_ROOTFS_NCUSTFS_MAXLEBCNT) -m $(BR2_TARGET_ROOTFS_NCUSTFS_MINIOSIZE)

ifeq ($(BR2_TARGET_ROOTFS_NCUSTFS_RT_ZLIB),y)
NCUSTFS_OPTS += -x zlib
endif
ifeq ($(BR2_TARGET_ROOTFS_NCUSTFS_RT_LZO),y)
NCUSTFS_OPTS += -x lzo
endif
ifeq ($(BR2_TARGET_ROOTFS_NCUSTFS_RT_NONE),y)
NCUSTFS_OPTS += -x none
endif

NCUSTFS_OPTS += $(call qstrip,$(BR2_TARGET_ROOTFS_NCUSTFS_OPTS))

ROOTFS_NCUSTFS_DEPENDENCIES = host-mtd

define ROOTFS_NCUSTFS_CMD
	$(HOST_DIR)/usr/sbin/mkfs.ubifs -d $(BR2_TARGET_ROOTFS_NCUSTFS_DIR) $(NCUSTFS_OPTS) -o $@
endef

$(eval $(call ROOTFS_TARGET,ncustfs))
