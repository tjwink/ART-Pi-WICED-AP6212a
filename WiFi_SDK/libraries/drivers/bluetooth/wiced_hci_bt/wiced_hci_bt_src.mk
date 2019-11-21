#
# $ Copyright Broadcom Corporation $
#

BLUETOOTH_WICED_HCI_DIR :=  $(BLUETOOTH_ROOT_DIR)wiced_hci_bt/
$(info $(BLUETOOTH_WICED_HCI_DIR))
################################################################################
# Supported variants                                                           #
################################################################################

SUPPORTED_BT_CHIPS           := 20702B0 43341B0 43438A1

################################################################################
# Default settings                                                             #
################################################################################

COMPLETE_BT_CHIP_NAME := BCM$(BT_CHIP)$(BT_CHIP_REVISION)

$(NAME)_INCLUDES   += $(SOURCE_ROOT)libraries/protocols/wiced_hci \
                      $(BLUETOOTH_BTE_DIR)WICED \
                      $(BLUETOOTH_ROOT_DIR)include \
                      $(BLUETOOTH_BTE_DIR)Components/stack/include \
                      $(BLUETOOTH_BTE_DIR)Components/gki/common \
                      $(BLUETOOTH_BTE_DIR)Projects/bte/main \
                      $(BLUETOOTH_WICED_HCI_DIR)internal


HCIBTSOURCES :=   $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_dm.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_ble.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_gatt.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_sdp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_a2dp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_hfp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_avrcp.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_rfcomm.c \
                  $(BLUETOOTH_WICED_HCI_DIR)wiced_hci_bt_mesh.c



$(NAME)_SOURCES    :=      $(HCIBTSOURCES)
$(NAME)_COMPONENTS := libraries/protocols/wiced_hci


ifneq ($(IAR),)
# IAR toolchain

# Disable "pointless integer comparison" because macros are
# used to test integer variable ranges.
BTE_IAR_DIAG_SUPPRESSIONS += --diag_suppress Pa084

# Disable "typedef name has already been declared" because
# multiply defined typedefs are used extensively.
BTE_IAR_DIAG_SUPPRESSIONS += --diag_suppress Pe301

$(NAME)_CFLAGS   += $(BTE_IAR_DIAG_SUPPRESSIONS)
$(NAME)_CXXFLAGS += $(BTE_IAR_DIAG_SUPPRESSIONS)

endif
