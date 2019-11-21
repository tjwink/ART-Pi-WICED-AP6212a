#
# $ Copyright Broadcom Corporation $
#
SOURCE_ROOT :=../../../../

ifneq ($(MAKECMDGOALS),wipe_source_for_test)
ifeq ($(RTOS),)
$(error Must define RTOS)
endif
endif

NAME :=fdk_aac_dec

# General WICED includes
CFLAGS += -I$(SOURCE_ROOT)include
CFLAGS += -I$(SOURCE_ROOT)tools/ARM_GNU/include
CFLAGS += -I$(SOURCE_ROOT)WICED
CFLAGS += -I$(SOURCE_ROOT)WICED/WWD/include
CFLAGS += -I$(SOURCE_ROOT)WICED/platform/include
CFLAGS += -I$(SOURCE_ROOT)WICED/platform/MCU
#CFLAGS += -I$(SOURCE_ROOT)WICED/network/$(NETWORK)/WWD
#CFLAGS += -I$(SOURCE_ROOT)WICED/WWD/internal/chips/4390x
CFLAGS += -I$(SOURCE_ROOT)WICED/RTOS/$(RTOS)/WICED
CFLAGS += -I$(SOURCE_ROOT)WICED/RTOS/$(RTOS)/WWD
#CFLAGS += -I$(SOURCE_ROOT)WICED/WWD/internal/bus_protocols/SDIO
#CFLAGS += -I$(SOURCE_ROOT)WICED/WWD/include/network
#CFLAGS += -I$(SOURCE_ROOT)WICED/network/$(NETWORK)/WICED
#CFLAGS += -I$(SOURCE_ROOT)WICED/security/BESL/crypto
#CFLAGS += -I$(SOURCE_ROOT)WICED/security/BESL/crypto_open
#CFLAGS += -I$(SOURCE_ROOT)libraries/utilities/base64
#CFLAGS += -I$(SOURCE_ROOT)WICED/security/BESL/include
#CFLAGS += -I$(SOURCE_ROOT)libraries/utilities/ring_buffer
#CFLAGS += -I$(SOURCE_ROOT)libraries/utilities/linked_list
#CFLAGS += -I$(SOURCE_ROOT)libraries/filesystems/wicedfs
CFLAGS += $(addprefix -I$(SOURCE_ROOT)WICED/RTOS/$(RTOS)/WWD/, $(GLOBAL_INCLUDES))
CFLAGS += -I$(SOURCE_ROOT)WICED/platform/$(HOST_ARCH)
# Using prebuilt stubs for "platform_mcu_peripheral.h"
CFLAGS += -I$(SOURCE_ROOT)tools/prebuild_stubs

include fdk_aac_dec_src.mk

SOURCES := $($(NAME)_SOURCES)
CFLAGS += $(addprefix -I,$(GLOBAL_INCLUDES)) $(addprefix -D,$(GLOBAL_DEFINES)) $(addprefix -I,$($(NAME)_INCLUDES)) $(addprefix -D,$($(NAME)_DEFINES)) $($(NAME)_CFLAGS)

KEEP_LIST_DIR := $(SOURCE_ROOT)libraries/audio/codec/fdk_aac_dec


include $(SOURCE_ROOT)tools/release/wiced_library.mk
