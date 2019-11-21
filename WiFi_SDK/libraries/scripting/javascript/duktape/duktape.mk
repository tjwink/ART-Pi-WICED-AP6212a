#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME := Lib_duktape

DUKTAPE_VER := 2.0.0

$(NAME)_SOURCES := wiced_duktape.c \
                   src/ver$(DUKTAPE_VER)/extras/module-node/duk_module_node.c

GLOBAL_INCLUDES += . \
                   src/ver$(DUKTAPE_VER)/extras/module-node

# Use newer version of FATFS that fixes some bugs
$(NAME)_COMPONENTS += filesystems/FATFS
FATFS_VERSION := ver0.12b

# Used to compile some WICED-specific code in third-party source code
$(NAME)_DEFINES += WICED

#
# Optional compile-time features
#

# Optional low-memory build of Duktape
WICED_DUKTAPE_OPT_LOW_MEMORY_BUILD      ?= 0

ifeq ($(WICED_DUKTAPE_OPT_LOW_MEMORY_BUILD),1)
$(NAME)_SOURCES += src/ver$(DUKTAPE_VER)/src-WICED-low-memory/duktape.c
GLOBAL_INCLUDES += src/ver$(DUKTAPE_VER)/src-WICED-low-memory
else
$(NAME)_SOURCES += src/ver$(DUKTAPE_VER)/src-WICED/duktape.c
GLOBAL_INCLUDES += src/ver$(DUKTAPE_VER)/src-WICED
endif

# Optional modules for Pandora speaker
WICED_DUKTAPE_OPT_PANDORA_MODULES      ?= 0

ifeq ($(WICED_DUKTAPE_OPT_PANDORA_MODULES),1)
$(NAME)_SOURCES     += dukluv/refs.c \
                       dukluv/schema.c \
                       dukluv/utils.c \
                       modules/dpm.c \
                       modules/timer.c \
                       modules/wss.c \
                       modules/wiced_websocket.c \
                       modules/callback_loop.c \
                       modules/xhr.c \
                       modules/audio.c

GLOBAL_INCLUDES     += modules dukluv
GLOBAL_DEFINES      += PLATFORM_DDR_HEAP_SIZE_CONFIG=16777216
$(NAME)_DEFINES     += WICED_USE_DDR_HEAP WICED_DUKTAPE_PANDORA_MODULES
$(NAME)_COMPONENTS  += utilities/wiced_log \
                       protocols/HTTP_client \
                       audio/audio_client \
                       audio/audio_render \
                       utilities/linked_list \
                       libraries/protocols/websocket

WICED_DUKTAPE_OPT_MEMORY_POOL_ALLOC := 1
endif

# Optional Duktape tests
WICED_DUKTAPE_OPT_TESTS_API             ?= 0

ifeq ($(WICED_DUKTAPE_OPT_TESTS_API),1)
$(NAME)_INCLUDES += src/ver$(DUKTAPE_VER)/tests/api
$(NAME)_DEFINES += WICED_DUKTAPE_TESTS_API
DUKTAPE_TESTS_API_SRC := $(wildcard $(CURDIR)src/ver$(DUKTAPE_VER)/tests/api/*.c)
$(NAME)_SOURCES += $(DUKTAPE_TESTS_API_SRC:$(CURDIR)%=%)
endif

# Optional Duktape logger
# Note: This adds logger.trace(), logger.debug(), logger.info(), logger.warn(),
#       logger.error(), and logger.fatal() functions for use in JS
WICED_DUKTAPE_OPT_LOGGER                ?= 1

ifeq ($(WICED_DUKTAPE_OPT_LOGGER),1)
$(NAME)_INCLUDES += src/ver$(DUKTAPE_VER)/extras/logging
$(NAME)_DEFINES += WICED_DUKTAPE_LOGGER
$(NAME)_SOURCES += src/ver$(DUKTAPE_VER)/extras/logging/duk_logging.c
endif

# Optional Duktape fatal handler
WICED_DUKTAPE_OPT_FATAL_HANDLER         ?= 1

ifeq ($(WICED_DUKTAPE_OPT_FATAL_HANDLER),1)
$(NAME)_DEFINES += WICED_DUKTAPE_FATAL_HANDLER
endif

# Optional Duktape memory pool allocator
# Note: Duktape makes numerous malloc calls, which result in memory
#       fragmentation. By using a memory pool allocator, memory fragmentation
#       can be kept minimal. However, the memory pool sizes would likely need
#       to be tuned in the source code.
WICED_DUKTAPE_OPT_MEMORY_POOL_ALLOC     ?= 0

ifeq ($(WICED_DUKTAPE_OPT_MEMORY_POOL_ALLOC),1)
$(NAME)_INCLUDES += src/ver$(DUKTAPE_VER)/examples/alloc-hybrid
$(NAME)_DEFINES += WICED_DUKTAPE_MEMORY_POOL_ALLOC
$(NAME)_SOURCES += src/ver$(DUKTAPE_VER)/examples/alloc-hybrid/duk_alloc_hybrid.c
endif

# Optional C Modules
WICED_DUKTAPE_OPT_MODULE_TIME           ?= 0
WICED_DUKTAPE_OPT_MODULE_WIFI           ?= 0

ifeq ($(WICED_DUKTAPE_OPT_MODULE_TIME),1)
$(NAME)_DEFINES += WICED_DUKTAPE_MODULE_TIME
$(NAME)_SOURCES += modules/wiced_duktape_module_time.c
endif

ifeq ($(WICED_DUKTAPE_OPT_MODULE_WIFI),1)
$(NAME)_DEFINES += WICED_DUKTAPE_MODULE_WIFI
$(NAME)_SOURCES += modules/wiced_duktape_module_wifi.c
endif

# Optional C Objects
WICED_DUKTAPE_OPT_OBJECT_AUDIO          ?= 0
WICED_DUKTAPE_OPT_OBJECT_TIME           ?= 0
WICED_DUKTAPE_OPT_OBJECT_XMLHTTPREQUEST ?= 0

ifeq ($(WICED_DUKTAPE_OPT_OBJECT_AUDIO),1)
$(NAME)_COMPONENTS += audio/audio_client \
                      audio/audio_render
$(NAME)_DEFINES += WICED_DUKTAPE_OBJECT_AUDIO
$(NAME)_SOURCES += objects/wiced_duktape_object_audio.c
endif

ifeq ($(WICED_DUKTAPE_OPT_OBJECT_TIME),1)
$(NAME)_DEFINES += WICED_DUKTAPE_OBJECT_TIME
$(NAME)_SOURCES += objects/wiced_duktape_object_time.c
endif

ifeq ($(WICED_DUKTAPE_OPT_OBJECT_XMLHTTPREQUEST),1)
$(NAME)_COMPONENTS += protocols/HTTP_client
$(NAME)_DEFINES += WICED_DUKTAPE_OBJECT_XMLHTTPREQUEST
$(NAME)_SOURCES += objects/wiced_duktape_object_xmlhttprequest.c
endif

#
# Resource files
# Notes:
#   - To make easier to add resource files, some Makefile magic is done below
#     to automagically flesh out $(NAME)_RESOURCES and also add it into the
#     source code via the DUKTAPE_RESOURCES define via $(NAME)_CFLAGS
#   - To add resources under WICED_DUKTAPE_RESOURCE_TOPDIR, simply add the
#     relative path and filename to WICED_DUKTAPE_RESOURCE_LIST
#   - This only works for resource files under WICED_DUKTAPE_RESOURCE_TOPDIR
#   - The WICED_DUKTAPE_RESOURCE_LIST can be appended to by other modules as
#     well; just be mindful to use the += operator instead of := or =
#   - Due to limitation of wiced_resources, it is not possible to have files
#     that share the same filename, even though they reside in different
#     directories
#
WICED_DUKTAPE_RESOURCE_TOPDIR := apps/duktape/
WICED_DUKTAPE_RESOURCE_LIST +=

$(NAME)_RESOURCES := $(strip \
    $(foreach ENTRY, $(WICED_DUKTAPE_RESOURCE_LIST), \
    $(addprefix $(WICED_DUKTAPE_RESOURCE_TOPDIR), $(ENTRY))))

include $(MAKEFILES_PATH)/wiced_resources.mk
WICED_DUKTAPE_RESOURCES_ENTRY = \
    { &$(call RESOURCE_VARIABLE_NAME, $(1)), \
    $(patsubst $(WICED_DUKTAPE_RESOURCE_TOPDIR)%,"%",$(1)) },
WICED_DUKTAPE_RESOURCES_ARRAY = \
    $(foreach ENTRY, $($(NAME)_RESOURCES), \
    $(call WICED_DUKTAPE_RESOURCES_ENTRY, $(ENTRY)))

$(NAME)_CFLAGS := -DWICED_DUKTAPE_RESOURCES='$(call WICED_DUKTAPE_RESOURCES_ARRAY)'
$(NAME)_CFLAGS += -Wno-error=unused-value -Wno-unused-value -Wno-error=strict-aliasing
ifeq ($(GCC_VERSION_GREATER_THAN_5),1)
$(NAME)_CFLAGS += -Wno-error=format-truncation
endif
