#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_Bluetooth_Manufacturing_Test_for_BCM$(BT_CHIP)$(BT_CHIP_REVISION)

################################################################################
# Default settings                                                             #
################################################################################

ifndef BT_TRANSPORT_BUS
BT_TRANSPORT_BUS := UART
endif

################################################################################
# Add defines for specific variants and check variants for validity            #
################################################################################

ifndef BT_CHIP
$(error ERROR: BT_CHIP is undefined!)
endif

ifndef BT_CHIP_REVISION
$(error ERROR: BT_CHIP_REVISION is undefined!)
endif

ifndef BT_TRANSPORT_BUS
$(error ERROR: BT_TRANSPORT_BUS is undefined!)
endif

################################################################################
# Specify global include directories                                           #
################################################################################

GLOBAL_INCLUDES  := . \
                    ../include \
                    internal/bus \
                    internal/firmware \
                    internal/packet \
                    internal/transport/driver \
                    internal/transport/HCI \
                    internal/transport/thread

ifeq ($(BT_CHIP_XTAL_FREQUENCY),)
BT_FIRMWARE_SOURCE = ../firmware/$(BT_CHIP)$(BT_CHIP_REVISION)/bt_firmware_controller.c
else
BT_FIRMWARE_SOURCE = ../firmware/$(BT_CHIP)$(BT_CHIP_REVISION)/$(BT_CHIP_XTAL_FREQUENCY)/bt_firmware_controller.c
endif

$(NAME)_SOURCES  += bt_mfg_test.c \
                    internal/bus/$(BT_TRANSPORT_BUS)/bt_bus.c \
                    internal/transport/driver/$(BT_TRANSPORT_BUS)/bt_transport_driver_receive.c \
                    internal/transport/driver/$(BT_TRANSPORT_BUS)/bt_transport_driver.c \
                    internal/transport/thread/bt_transport_thread.c \
                    internal/packet/bt_packet.c \
                    internal/firmware/bt_firmware.c \
                    $(BT_FIRMWARE_SOURCE)


$(NAME)_COMPONENTS += utilities/linked_list