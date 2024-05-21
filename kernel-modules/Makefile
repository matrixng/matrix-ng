KVERSION := $(shell uname -r)
KDIR := /lib/modules/$(KVERSION)/build
DTC ?= dtc
PWD := $(shell pwd)
DESTDIR = 
PACKAGE_VERSION = 
DKMS_MODULES_NAME = matrix-ng-kernel-modules
DKMS_SOURCE_DIR = $(DESTDIR)/usr/src/$(DKMS_MODULES_NAME)-$(PACKAGE_VERSION)
OVERLAYS_DEST_DIR = $(DESTDIR)/boot/firmware/overlays

dkms_source_install:
	$(DTC) -W no-unit_address_vs_reg -@ -I dts -O dtb -o matrixio.dtbo src/matrixio.dts
	install -m 0755 -d $(DKMS_SOURCE_DIR)
	install -m 0755 -d $(OVERLAYS_DEST_DIR)
	install -m 0644 src/Makefile src/Kbuild src/*.[hc] $(DKMS_SOURCE_DIR)
	install -m 0644 matrixio.dtbo $(OVERLAYS_DEST_DIR)
	rm -rf matrixio.dtbo
	