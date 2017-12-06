################################################################################
#
# Embed the ubifs image into an ubi image
#
################################################################################

NCUST_UBINIZE_OPTS := -m $(BR2_TARGET_ROOTFS_NCUSTFS_MINIOSIZE)
NCUST_UBINIZE_OPTS += -p $(BR2_TARGET_ROOTFS_NCUST_PEBSIZE)
ifneq ($(BR2_TARGET_ROOTFS_NCUST_SUBSIZE),0)
NCUST_UBINIZE_OPTS += -s $(BR2_TARGET_ROOTFS_NCUST_SUBSIZE)
endif

NCUST_UBINIZE_OPTS += $(call qstrip,$(BR2_TARGET_ROOTFS_NCUST_OPTS))

ROOTFS_NCUST_DEPENDENCIES = rootfs-ncustfs

ifeq ($(BR2_TARGET_ROOTFS_NCUST_USE_CUSTOM_CONFIG),y)
NCUST_UBINIZE_CONFIG_FILE_PATH = $(call qstrip,$(BR2_TARGET_ROOTFS_NCUST_CUSTOM_CONFIG_FILE))
else
NCUST_UBINIZE_CONFIG_FILE_PATH = fs/ncustfs/ubinize.cfg
endif

define ROOTFS_NCUST_CMD
	$(INSTALL) -m 0644 $(NCUST_UBINIZE_CONFIG_FILE_PATH) $(BUILD_DIR)/ubinize.cfg ;\
	$(SED) 's;BR2_ROOTFS_NCUSTFS_PATH;$@fs;' $(BUILD_DIR)/ubinize.cfg ;\
	$(HOST_DIR)/usr/sbin/ubinize -o $@ $(NCUST_UBINIZE_OPTS) $(BUILD_DIR)/ubinize.cfg ;\
	rm $(BUILD_DIR)/ubinize.cfg
endef

$(eval $(call ROOTFS_TARGET,ncust))
