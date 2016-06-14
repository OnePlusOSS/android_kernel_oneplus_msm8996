#
# Makefile for the Synaptics DSX touchscreen driver.
#

# Each configuration option enables a list of files.

obj-y += synaptics_dsx_i2c.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_SPI) += synaptics_dsx_spi.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI_HID_I2C) += synaptics_dsx_rmi_hid_i2c.o
obj-y += synaptics_dsx_core.o
obj-y += synaptics_dsx_rmi_dev.o
obj-y += synaptics_dsx_fw_update.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_TEST_REPORTING) += synaptics_dsx_test_reporting.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_PROXIMITY) += synaptics_dsx_proximity.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_ACTIVE_PEN) += synaptics_dsx_active_pen.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_VIDEO) += synaptics_dsx_video.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_DEBUG) += synaptics_dsx_debug.o
