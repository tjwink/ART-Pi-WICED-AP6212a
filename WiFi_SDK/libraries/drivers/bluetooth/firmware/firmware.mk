#
# $ Copyright Broadcom Corporation $
#
NAME := Lib_WICED_Bluetooth_Firmware_Driver_for_BCM$(BT_CHIP)$(BT_CHIP_REVISION)

# By default,the 20706A2/43012C0 will use the firmware images that are part of the release.
# The firmware driver is picked depending on the BT chip, XTAL frequency and the mode it is configured with(controller/embedded).
# E.g. If 20706 A1 BT chip with XTAL 40Mhz is selected in controller mode,
# then the file path would be 20706A1/40Mhz/bt_firmware_controller.c.
# E.g. If 20706 A2 BT chip with XTAL 40Mhz is selected in embedded mode,
# then the file path would be 20706A2/40Mhz/bt_firmware_embedded_(EMBEDDED_APP_NAME).c.
# For embedded mode, the EMBEDDED_APP_NAME should be provided in the application MakeFile.
# The EMBEDDED_APP_NAME will be same as the application name used to generate firmware in the 20706-SDK.
# NOTE: Only 20706A2 and 43012 will support both controller/embedded Mode. Rest of the BT chips support controller mode only.

ifneq ($(EMBEDDED_APP_NAME),)
ifneq ( $(BT_CHIP_XTAL_FREQUENCY), )
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/$(BT_CHIP_XTAL_FREQUENCY)/bt_firmware_embedded_$(EMBEDDED_APP_NAME).c
else
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/40MHz/bt_firmware_embedded_$(EMBEDDED_APP_NAME).c
endif
else
ifeq ( $(BT_CHIP_XTAL_FREQUENCY), )
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/bt_firmware_controller.c
else
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/$(BT_CHIP_XTAL_FREQUENCY)/bt_firmware_controller.c
endif
endif