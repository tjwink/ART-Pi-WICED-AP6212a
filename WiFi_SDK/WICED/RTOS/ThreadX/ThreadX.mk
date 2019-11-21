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

NAME := ThreadX

THREADX_VERSION := 5.8

ifeq (ROM, $(WICED_OSTYPE))
$(info WICED_OSTYPE ROM build)
THREADX_VERSION = 5.6
$(NAME)_COMPONENTS := WICED/platform/MCU/$(HOST_MCU_FAMILY)/RTOS/ThreadXROM
GLOBAL_INCLUDES := ../../platform/MCU/$(HOST_MCU_FAMILY)/RTOS/ThreadXROM/ver$(THREADX_VERSION)
# Define some macros to allow for some network-specific checks
GLOBAL_DEFINES += RTOS_$(NAME)=1
GLOBAL_DEFINES += $(NAME)_VERSION=$$(SLASH_QUOTE_START)v$(THREADX_VERSION)$$(SLASH_QUOTE_END)
ifneq ($(filter $(HOST_ARCH), ARM_CM3 ARM_CM4),)
THREADX_ARCH:=Cortex_M3_M4
GLOBAL_INCLUDES += ver$(THREADX_VERSION)/Cortex_M3_M4/$(TOOLCHAIN_NAME) \
                   WWD/CM3_CM4
else
ifneq ($(filter $(HOST_ARCH), ARM_CR4),)
THREADX_ARCH:=Cortex_R4
GLOBAL_DEFINES += __TARGET_ARCH_ARM=7 __THUMB_INTERWORK
GLOBAL_INCLUDES += ver$(THREADX_VERSION)/Cortex_R4/$(TOOLCHAIN_NAME) \
                   WWD/CR4
else
$(error No ThreadX port for architecture)
endif
endif
else #Non ROM build
$(NAME)_COMPONENTS := WICED/RTOS/ThreadX/WWD
ifeq (,$(APP_WWD_ONLY)$(NS_WWD_ONLY)$(RTOS_WWD_ONLY))
$(NAME)_COMPONENTS += WICED/RTOS/ThreadX/WICED
endif

# Define some macros to allow for some network-specific checks
GLOBAL_DEFINES += RTOS_$(NAME)=1
GLOBAL_DEFINES += $(NAME)_VERSION=$$(SLASH_QUOTE_START)v$(THREADX_VERSION)$$(SLASH_QUOTE_END)
GLOBAL_INCLUDES := ver$(THREADX_VERSION)

GLOBAL_DEFINES += TX_INCLUDE_USER_DEFINE_FILE
#GLOBAL_DEFINES += TX_ENABLE_FIQ_SUPPORT

ifneq ($(filter $(HOST_ARCH), ARM_CM3 ARM_CM4),)
THREADX_ARCH:=Cortex_M3_M4
GLOBAL_INCLUDES += ver$(THREADX_VERSION)/Cortex_M3_M4/$(TOOLCHAIN_NAME) \
                   WWD/CM3_CM4
else
ifneq ($(filter $(HOST_ARCH), ARM_CR4),)
THREADX_ARCH:=Cortex_R4
GLOBAL_DEFINES += __TARGET_ARCH_ARM=7 __THUMB_INTERWORK
GLOBAL_INCLUDES += ver$(THREADX_VERSION)/Cortex_R4/$(TOOLCHAIN_NAME) \
                   WWD/CR4
else
$(error No ThreadX port for architecture)
endif
endif


ifdef WICED_ENABLE_TRACEX
$(info TRACEX is ENABLED)
THREADX_LIBRARY_NAME :=ThreadX.TraceX.$(HOST_ARCH)$(DOT_TOOLCHAIN_TYPE).$(BUILD_TYPE).a
GLOBAL_DEFINES += TX_ENABLE_EVENT_TRACE
else
THREADX_LIBRARY_NAME :=ThreadX.$(HOST_ARCH)$(DOT_TOOLCHAIN_TYPE).$(BUILD_TYPE).a
endif

ifneq ($(wildcard $(CURDIR)$(THREADX_LIBRARY_NAME)),)
$(NAME)_PREBUILT_LIBRARY := $(THREADX_LIBRARY_NAME)
else
# Build from source (Broadcom internal)
-include $(CURDIR)ThreadX_src.mk
endif # ifneq ($(wildcard $(CURDIR)$(THREADX_LIBRARY_NAME)),)
endif #ifeq (ROM, $(WICED_OSTYPE))
