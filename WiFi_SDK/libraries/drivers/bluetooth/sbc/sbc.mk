#
# $ Copyright Broadcom Corporation $
#

NAME                        := Lib_sbc
BLUETOOTH_sbc_DIR           := ./
BLUETOOTH_ROOT_DIR          := ../
BLUETOOTH_BTE_DIR           := ../BTE/
BLUETOOTH_LIB_TYPE          := sbc
BLUETOOTH_PATH_TO_PREBUILT  := ../
BLUETOOTH_DUAL_MODE_DIR     := ../dual_mode/

ifneq ($(wildcard $(CURDIR)$(BLUETOOTH_ROOT_DIR)bluetooth_$(BLUETOOTH_LIB_TYPE).$(RTOS).$(NETWORK).$(HOST_ARCH)$(DOT_TOOLCHAIN_TYPE).release.a),)

$(NAME)_PREBUILT_LIBRARY := $(BLUETOOTH_PATH_TO_PREBUILT)bluetooth_$(BLUETOOTH_LIB_TYPE).$(RTOS).$(NETWORK).$(HOST_ARCH).release.a

GLOBAL_INCLUDES :=              $(BLUETOOTH_ROOT_DIR)include \
                                $(BLUETOOTH_BTE_DIR)Components/stack/include \
                                $(BLUETOOTH_BTE_DIR)Projects/bte/main \
                                $(BLUETOOTH_BTE_DIR)WICED
else
# Build from source (Broadcom internal)
include  $(CURDIR)$(BLUETOOTH_LIB_TYPE)_src.mk
endif

