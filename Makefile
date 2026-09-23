##############################################################################
# DH BLE Device - TLSR8258F512ET32
# Build with: make -C /root/tc_ble_sdk/tc_ble_sdk PROJECT_DIR=/root/workspace/tlsr/202609
##############################################################################

SDK_ROOT := /root/tc_ble_sdk/tc_ble_sdk
PROJ_DIR := $(shell pwd)
BUILD_DIR := $(SDK_ROOT)/build_output/dh_device
PROJECT := dh_device

.PHONY: all clean info

all:
	@echo "Building DH BLE Device firmware..."
	$(MAKE) -C $(SDK_ROOT) PROJECT_DIR=$(PROJ_DIR) PROJECT=$(PROJECT)

clean:
	rm -rf $(BUILD_DIR)
	@echo "Cleaned build output."

info:
	$(MAKE) -C $(SDK_ROOT) PROJECT_DIR=$(PROJ_DIR) PROJECT=$(PROJECT) info
