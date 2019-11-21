#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_wlu_server

ifeq ($(GCC_VERSION_GREATER_THAN_5),1)
$(NAME)_CFLAGS += -Wno-error=misleading-indentation -Wno-error=pointer-compare -Wno-format-truncation -Wno-stringop-overflow \
                  -Wno-memset-transposed-args -Wno-format-overflow -Wno-unused-const-variable
endif

$(NAME)_SOURCES  := $(WLAN_CHIP)$(WLAN_CHIP_REVISION)/wl/exe/wlu_server_shared.c \
                   wlu_server.c \
                   wlu_wiced.c \
                   $(WLAN_CHIP)$(WLAN_CHIP_REVISION)/wl/exe/wlu_pipe.c \
                   $(WLAN_CHIP)$(WLAN_CHIP_REVISION)/wl/exe/wlu.c \
                   $(WLAN_CHIP)$(WLAN_CHIP_REVISION)/shared/miniopt.c \
                   $(WLAN_CHIP)$(WLAN_CHIP_REVISION)/shared/bcmutils.c \
                   $(WLAN_CHIP)$(WLAN_CHIP_REVISION)/shared/bcm_app_utils.c

$(NAME)_INCLUDES := ./$(WLAN_CHIP)$(WLAN_CHIP_REVISION)/include \
                    ./$(WLAN_CHIP)$(WLAN_CHIP_REVISION)/common/include \
                    ./$(WLAN_CHIP)$(WLAN_CHIP_REVISION)/wl/exe \
                    ./$(WLAN_CHIP)$(WLAN_CHIP_REVISION)/shared/bcmwifi/include \
                    ./$(WLAN_CHIP)$(WLAN_CHIP_REVISION)/wl/ppr/include


CHIP := $(WLAN_CHIP)$(WLAN_CHIP_REVISION)
include $(CURDIR)$(WLAN_CHIP)$(WLAN_CHIP_REVISION)/$(WLAN_CHIP)$(WLAN_CHIP_REVISION).mk
$(NAME)_SOURCES += $($(CHIP)_SOURCE_WL)

#Serial Support
$(NAME)_DEFINES  := RWL_SERIAL TARGET_wiced
#Ethernet Support
#$(NAME)_DEFINES  := RWL_SOCKET TARGET_wiced

#$(NAME)_COMPONENTS += test/malloc_debug

GLOBAL_INCLUDES  += .

#AVOID_GLOMMING_IOVAR AVOID_APSTA SET_COUNTRY_WITH_IOCTL_NOT_IOVAR

