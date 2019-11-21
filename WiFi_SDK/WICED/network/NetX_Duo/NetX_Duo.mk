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

NAME := NetX_Duo

ifneq ($(WICED_NETTYPE),ROM)
NETX_DUO_VERSION := 5.10_sp3
else
NETX_DUO_VERSION := 5.7
endif

VALID_RTOS_LIST:= ThreadX

$(NAME)_COMPONENTS += WICED/network/NetX_Duo/WWD

ifeq (,$(APP_WWD_ONLY)$(NS_WWD_ONLY)$(RTOS_WWD_ONLY))
$(NAME)_COMPONENTS += WICED/network/NetX_Duo/WICED
endif

# Define some macros to allow for some network-specific checks
GLOBAL_DEFINES += NETWORK_$(NAME)=1
GLOBAL_DEFINES += $(NAME)_VERSION=$$(SLASH_QUOTE_START)v$(NETX_DUO_VERSION)$$(SLASH_QUOTE_END)
# prevent sending reset for non-blocking disconnect
GLOBAL_DEFINES += NX_INCLUDE_USER_DEFINE_FILE
GLOBAL_DEFINES += __fd_set_defined

ifneq ($(TOOLCHAIN_NAME),IAR)
GLOBAL_DEFINES += SYS_TIME_H_AVAILABLE
endif

ifneq ($(WICED_NETTYPE),ROM)
GLOBAL_INCLUDES := ver$(NETX_DUO_VERSION) \
                   ver$(NETX_DUO_VERSION)/netx_bsd_layer \
                   WICED
else
GLOBAL_INCLUDES := ../../platform/MCU/$(HOST_MCU_FAMILY)/network/Netx_Duo_$(NETX_DUO_VERSION) \
                    WICED
endif

ifneq ($(WICED_NETTYPE),ROM)

ifdef WICED_ENABLE_TRACEX
# Precompiled library with TraceX
NETX_DUO_LIBRARY_NAME :=NetX_Duo.TraceX.$(RTOS).$(HOST_ARCH)$(DOT_TOOLCHAIN_TYPE).$(BUILD_TYPE).a
else
# Precompiled library
NETX_DUO_LIBRARY_NAME :=NetX_Duo.$(RTOS).$(HOST_ARCH)$(DOT_TOOLCHAIN_TYPE).$(BUILD_TYPE).a
endif

ifneq ($(wildcard $(CURDIR)$(NETX_DUO_LIBRARY_NAME)),)
# Using a precompiled Library
$(NAME)_PREBUILT_LIBRARY := $(NETX_DUO_LIBRARY_NAME)
else
# Build from source (Cypress internal)
-include $(CURDIR)NetX_Duo_src.mk
endif #ifneq ($(wildcard $(CURDIR)$(NETX_DUO_LIBRARY_NAME)),)

$(NAME)_SOURCES += ver$(NETX_DUO_VERSION)/nxd_external_functions.c
endif #ifneq ($(WICED_NETTYPE),ROM)
