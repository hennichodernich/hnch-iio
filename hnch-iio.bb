SUMMARY = "Recipe for external hnch-iio Linux kernel module"
SECTION = "PETALINUX/modules"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "file://Makefile \
           file://cf_axi_hnch_core.c \
           file://cf_axi_hnch_ring_stream.c \
           file://cf_axi_hnch.h \
           file://hnchboard_hw_config.h \
           file://si5351_defs.h \
	   file://COPYING \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
