#
# $ Copyright Broadcom Corporation $
#

NAME                        := Lib_Bluetooth_Embedded_Low_Energy_Stack_for_WICED
BLUETOOTH_low_energy_DIR    := ./
BLUETOOTH_ROOT_DIR          := ../
BLUETOOTH_BTE_DIR           := ../BTE/
BLUETOOTH_LIB_TYPE          := low_energy
BLUETOOTH_PATH_TO_PREBUILT  := ../

ifneq ($(HOST_MCU_FAMILY),BCM920739)
ifneq ($(wildcard $(CURDIR)$(BLUETOOTH_ROOT_DIR)bluetooth_$(BLUETOOTH_LIB_TYPE).$(RTOS).$(NETWORK).$(HOST_ARCH).release.a),)
$(NAME)_PREBUILT_LIBRARY := $(BLUETOOTH_PATH_TO_PREBUILT)bluetooth_$(BLUETOOTH_LIB_TYPE).$(RTOS).$(NETWORK).$(HOST_ARCH)$(DOT_TOOLCHAIN_TYPE).release.a

GLOBAL_INCLUDES :=      $(BLUETOOTH_low_energy_DIR)\
                        $(BLUETOOTH_ROOT_DIR)include \
                        $(BLUETOOTH_BTE_DIR)Components/gki/common \
                        $(BLUETOOTH_BTE_DIR)Components/udrv/include \
                        $(BLUETOOTH_BTE_DIR)Projects/bte/main \
                        $(BLUETOOTH_BTE_DIR)Components/stack/include \
                        $(BLUETOOTH_BTE_DIR)proto_disp/ \
                        $(BLUETOOTH_BTE_DIR)WICED
else
# Build from source (Broadcom internal)
include  $(CURDIR)$(BLUETOOTH_LIB_TYPE)_src.mk
endif

$(NAME)_SOURCES += ../BTE/bt_logmsg/wiced_bt_logmsg.c \
                   ../BTE/proto_disp/wiced_bt_protocol_print.c

# Include appropriate firmware as component
$(NAME)_COMPONENTS := libraries/drivers/bluetooth/firmware
endif #ifneq ($(PLATFORM),BCM920739)
