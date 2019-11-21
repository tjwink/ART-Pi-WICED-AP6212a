#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_wiced_hci_bt
BLUETOOTH_ROOT_DIR              := ../
BLUETOOTH_wiced_hci_bt_DIR      := ./
BLUETOOTH_BTE_DIR               := ../BTE/
BLUETOOTH_LIB_TYPE              := wiced_hci_bt
BLUETOOTH_PATH_TO_PREBUILT      := ../

GLOBAL_INCLUDES                 := $(BLUETOOTH_wiced_hci_bt_DIR) \
                                   $(BLUETOOTH_ROOT_DIR)include

GLOBAL_INCLUDES                 += $(BLUETOOTH_BTE_DIR)Components/gki/common \
                                   $(BLUETOOTH_BTE_DIR)Components/udrv/include \
                                   $(BLUETOOTH_BTE_DIR)Projects/bte/main \
                                   $(BLUETOOTH_BTE_DIR)WICED \
                                   $(BLUETOOTH_BTE_DIR)Components/stack/include \
                                   $(BLUETOOTH_ROOT_DIR)dual_mode

$(info $(CURDIR)$(BLUETOOTH_LIB_TYPE)_src.mk)
include $(CURDIR)$(BLUETOOTH_LIB_TYPE)_src.mk

# Include appropriate firmware as component
$(NAME)_COMPONENTS += libraries/drivers/bluetooth/firmware
