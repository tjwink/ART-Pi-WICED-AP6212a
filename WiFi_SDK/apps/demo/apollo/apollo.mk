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

NAME := App_apollo

APOLLO_TX_PACKET_POOL_SIZE ?= 10
APOLLO_RX_PACKET_POOL_SIZE ?= 10

# Make sure we pick up the Apollo-specific WLAN firmware
WLAN_CHIP_BIN_TYPE :=-apollo

# To include Apollo Ambient Light support in the Apollo  library, uncomment the
# USE_AMBILIGHT define or add USE_AMBILIGHT=1 to the target build string.
#USE_AMBILIGHT := 1
ifdef USE_AMBILIGHT
GLOBAL_DEFINES += USE_AMBILIGHT
endif

# To include Dolby Digital Decoder support in the Apollo streamer library, uncomment the
# USE_UDC define or add USE_UDC=1 to the target build string. For example: demo.apollo-BCM943909WCD1_3 USE_UDC=1
#USE_UDC := 1

#WICED_ENABLE_TRACEX := 1

# To build Apollo without Bluetooth, uncomment the APOLLO_NO_BT define.
#APOLLO_NO_BT := 1

# To buil Apollo with BT embedded firmware (BT stack running on BT chip) instead of BT host firmware (BT stack running on apps CPU)
#APOLLO_BT_EMBEDDED := 1

# To build with OLED display support
#USE_AUDIO_DISPLAY := 1

# To build with and enable UPnP AV rendering with Apollo rebroadcasting
#USE_UPNPAV := 1

# To build with Apollo 3.2 Wavpack compression enable this define
# USE_AUDIOPCM_COMPRESSION := 1
#
# NOTE: since we are memory limited on 43907 we need to make sure
#       that we do not compile audiopcm_compression when BOTH
#       Bluetooth AND DolbyDigital are enabled.
#
# Only one (DD or BT) can be enabled when using audiopcm_compression

ifdef USE_AUDIOPCM_COMPRESSION
ifdef USE_UDC
ifndef APOLLO_NO_BT
$(error Exceeding system memory: using BT+DD and AUDIOPCM_COMPRESSION is above 43907 memory limit.)
endif
endif
endif

#GLOBAL_DEFINES     += CONSOLE_ENABLE_WL
#GLOBAL_DEFINES     += APOLLO_NO_WIFI_COMMANDS

# Prevents inclusion of mbedtls_open library saving at least 110KB of memory
# (but disables TLS/DTLS support along with Enterprise WiFi Security)
GLOBAL_DEFINES     += WICED_DISABLE_TLS CONSOLE_DISABLE_ENTERPRISE_COMMANDS

GLOBAL_DEFINES     += APPLICATION_STACK_SIZE=8000

$(NAME)_SOURCES    := apollo.c
$(NAME)_SOURCES    += apollo_debug.c
$(NAME)_SOURCES    += apollo_config.c
$(NAME)_SOURCES    += apollo_console.c
$(NAME)_SOURCES    += apollo_keypad.c

$(NAME)_COMPONENTS := audio/audio_render \
                      audio/apollo/apollo_player \
                      audio/apollo/apollo_streamer \
                      audio/apollo/apollocore \
                      utilities/command_console \
                      utilities/command_console/wifi \
                      utilities/command_console/dct \
                      utilities/wiced_log \
                      inputs/button_manager \
                      drivers/power_management

ifdef USE_AMBILIGHT
$(NAME)_COMPONENTS += /ambilight/amblt_middle/amblt_source
$(NAME)_COMPONENTS += /ambilight/amblt_middle/amblt_sink
$(NAME)_COMPONENTS += /ambilight/amblt_back
$(NAME)_COMPONENTS += /ambilight/amblt_front
endif

ifdef USE_UPNPAV
APOLLO_NO_BT       := 1
GLOBAL_DEFINES     += USE_UPNPAV
GLOBAL_DEFINES     += APOLLO_NO_WIFI_COMMANDS
$(NAME)_SOURCES    += apollo_upnpavrender.c
$(NAME)_COMPONENTS += audio/upnp_av_render
endif

ifdef APOLLO_NO_BT
$(info apollo building without BT)
GLOBAL_DEFINES     += APOLLO_NO_BT
else # !APOLLO_NO_BT
PLATFORM_SUPPORTING_BT_EMBEDDED := CYW943907WAE*
$(eval PLATFORM_SUPPORTING_BT_EMBEDDED := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_SUPPORTING_BT_EMBEDDED)))
ifneq ($(filter $(PLATFORM),$(PLATFORM_SUPPORTING_BT_EMBEDDED)),)
$(info apollo building with BT EMBEDDED stack)
APOLLO_BT_EMBEDDED := 1
else
$(info apollo building with BT HOST stack)
endif
$(NAME)_COMPONENTS += audio/apollo/apollo_bt_service
endif

PLATFORM_SUPPORTING_POWER_MGMT := BCM943907WAE_1* BCM943907WAE2_1* CYW943907WAE*
$(eval PLATFORM_SUPPORTING_POWER_MGMT := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_SUPPORTING_POWER_MGMT)))
ifneq ($(filter $(PLATFORM),$(PLATFORM_SUPPORTING_POWER_MGMT)),)
    GLOBAL_DEFINES  += POWER_MANAGEMENT_ON_BCM943907WAE_1
endif

ifdef USE_AUDIO_DISPLAY
# include display
GLOBAL_DEFINES += USE_AUDIO_DISPLAY
$(NAME)_COMPONENTS += audio/display
endif

# Enable this for getting Bluetooth protocol traces
#GLOBAL_DEFINES     += ENABLE_BT_PROTOCOL_TRACES

ifneq (,$(findstring CONSOLE_ENABLE_WL,$(GLOBAL_DEFINES)))
$(NAME)_COMPONENTS += test/wl_tool
endif

APPLICATION_DCT    := apollo_dct.c

WIFI_CONFIG_DCT_H  := wifi_config_dct.h

# Bluetooth MAC address and name are configured via the DCT
BT_CONFIG_DCT_H    := bt_config_dct.h

ifdef WICED_ENABLE_TRACEX
$(info apollo using tracex lib)

#Only use DDR on WCD1 boards
ifneq ($(filter $(PLATFORM),BCM943909WCD1_3),)
GLOBAL_DEFINES     += WICED_TRACEX_BUFFER_DDR_OFFSET=0x0
GLOBAL_DEFINES     += WICED_TRACEX_BUFFER_SIZE=0x200000
else
GLOBAL_DEFINES     += WICED_TRACEX_BUFFER_SIZE=0x10000
endif

$(NAME)_COMPONENTS += test/TraceX
$(NAME)_SOURCES    += apollo_tracex.c
endif

GLOBAL_DEFINES     += TX_PACKET_POOL_SIZE=$(APOLLO_TX_PACKET_POOL_SIZE)
GLOBAL_DEFINES     += RX_PACKET_POOL_SIZE=$(APOLLO_RX_PACKET_POOL_SIZE)

GLOBAL_DEFINES     += WICED_USE_AUDIO

GLOBAL_DEFINES     += AUTO_IP_ENABLED

#GLOBAL_DEFINES     += WICED_DISABLE_WATCHDOG

#OTA2 Support
ifeq (1,$(OTA2_SUPPORT))
# undefine to skip Version checking of update while developing updates
# define (and provide version #) for production version testing
# default values (if not defined) are in apollo_dct.h
# You can provide version # during compilation by adding to build compile args
# <application>-<platform> ota2_image APP_VERSION_FOR_OTA2_MAJOR=x APP_VERSION_FOR_OTA2_MINOR=y
#GLOBAL_DEFINES      += CHECK_OTA2_UPDATE_VERSION=1

$(NAME)_SOURCES    += apollo_ota2_support.c

$(NAME)_COMPONENTS += daemons/ota2_service

OTA_APPLICATION	   := snip.ota2_extract-$(PLATFORM)
OTA_APP    		   := build/$(OTA_APPLICATION)/binary/$(OTA_APPLICATION).stripped.elf
endif

VALID_OSNS_COMBOS  := ThreadX-NetX_Duo ThreadX-NetX
VALID_PLATFORMS    :=
VALID_PLATFORMS    += BCM943909WCD*
VALID_PLATFORMS    += BCM943907*
VALID_PLATFORMS    += CYW943907WAE*
INVALID_PLATFORMS  += BCM943907AEVAL* BCM943907WCD2* CYW9MCU7X9N364
