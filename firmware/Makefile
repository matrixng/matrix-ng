PWD := $(shell pwd)
DESTDIR= 
PACKAGE_VERSION= 
DEST_OPT_DIR = $(DESTDIR)/opt/matrix-ng
DEST_FIRMWARE_DIR = $(DEST_OPT_DIR)/firmware/$(PACKAGE_VERSION)

deb_install:
	install -m 0755 -d $(DEST_FIRMWARE_DIR)
	install -m 0755 -d $(DEST_FIRMWARE_DIR)/creator
	install -m 0755 -d $(DEST_FIRMWARE_DIR)/voice
	install -m 0644 blobs/creator/*.firmware $(DEST_FIRMWARE_DIR)/creator
	install -m 0644 blobs/voice/*.bit $(DEST_FIRMWARE_DIR)/voice
	echo $(PACKAGE_VERSION) >> $(DEST_OPT_DIR)/firmware/latest_version