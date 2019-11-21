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
NAME := Lib_USBX

USBX_VERSION := 5.8

VALID_RTOS_LIST:= ThreadX

$(NAME)_COMPONENTS += libraries/drivers/USB/USBX/WICED

ifeq ($(RTOS),)
$(error Must define RTOS)
endif

# Define some macros to allow for some usb-specific checks
GLOBAL_DEFINES += USB_$(NAME)=1
GLOBAL_DEFINES += $(NAME)_VERSION=$$(SLASH_QUOTE_START)v$(USBX_VERSION)$$(SLASH_QUOTE_END)
GLOBAL_DEFINES += UX_INCLUDE_USER_DEFINE_FILE UX_ENABLE_DEBUG_LOG

# These define are for app-layer and library-layer correctly include of some files in
# WICED/platform/MCU/BCM4390x/peripherals
$(NAME)_DEFINES += BCMDRIVER BCM_WICED

# Now USBX only works with FileX
$(NAME)_COMPONENTS += filesystems/FileX
USING_FILEX_USBX := yes


GLOBAL_INCLUDES := ver$(USBX_VERSION)
GLOBAL_INCLUDES += ver$(USBX_VERSION)/usbx_host_classes
GLOBAL_INCLUDES += ver$(USBX_VERSION)/usbx_host_controllers
GLOBAL_INCLUDES += ver$(USBX_VERSION)/usbx_device_classes
GLOBAL_INCLUDES += ver$(USBX_VERSION)/usbx_device_controllers
GLOBAL_INCLUDES += ver$(USBX_VERSION)/usbx_device_controllers/bcm4390x


ifdef WICED_ENABLE_TRACEX
# Precompiled library with TraceX
USBX_LIBRARY_NAME :=USBX.TraceX.$(RTOS).$(HOST_ARCH).$(BUILD_TYPE).a
else
# Precompiled library
USBX_LIBRARY_NAME :=USBX.$(RTOS).$(HOST_ARCH).$(BUILD_TYPE).a
endif

ifneq ($(wildcard $(CURDIR)$(USBX_LIBRARY_NAME)),)
# Using a precompiled Library
$(NAME)_PREBUILT_LIBRARY := $(USBX_LIBRARY_NAME)
else
# Build from source (Broadcom internal)
include $(CURDIR)USBX_src.mk
endif #ifneq ($(wildcard $(CURDIR)USBX.$(HOST_ARCH).$(BUILD_TYPE).a),)

$(NAME)_SOURCES +=  ver$(USBX_VERSION)/ux_utility_wiced_all.c
