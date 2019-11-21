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

NAME               := App_headset

$(NAME)_SOURCES    := headset.c \
                      headset_config.c \
                      headset_dct.c \
                      headset_button.c \
                      headset_wlan.c \
                      headset_upnpavrender.c \
                      hashtable.c

$(NAME)_INCLUDES   += .

$(NAME)_COMPONENTS += utilities/connection_manager \
                      audio/upnp_av_render \
                      inputs/button_manager

# Enable console. The MCU powersave can be a cause of poor response of console
#$(NAME)_DEFINES    += HEADSET_CONS_SUPPORT

# Enable MCU Sleep
#GLOBAL_DEFINES     += WICED_DISABLE_MCU_POWERSAVE

ifneq (,$(findstring HEADSET_CONS_SUPPORT,$($(NAME)_DEFINES)))
$(info "Headset console is enabled!")
$(NAME)_COMPONENTS += utilities/command_console \
                      utilities/command_console/dct \
                      utilities/command_console/wifi \
                      utilities/command_console/p2p \
                      utilities/command_console/wps \
                      utilities/command_console/platform

$(NAME)_DEFINES    += CONSOLE_INCLUDE_P2P
GLOBAL_DEFINES     += CONSOLE_THREAD_STACK_SIZE=8*1024
endif

PLATFORM_SUPPORTING_POWER_MGMT := BCM943907WAE_1* BCM943907WAE2_1* CYW943907WAE*
$(eval PLATFORM_SUPPORTING_POWER_MGMT := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_SUPPORTING_POWER_MGMT)))
ifneq ($(filter $(PLATFORM),$(PLATFORM_SUPPORTING_POWER_MGMT)),)
    GLOBAL_DEFINES  += POWER_MANAGEMENT_ON_BCM943907WAE_1
endif

#OTA2 Support
ifeq (1,$(OTA2_SUPPORT))
$(info "OTA2 Is Enabled!")
OTA_APPLICATION    := snip.ota2_extract-$(PLATFORM)
OTA_APP            := build/$(OTA_APPLICATION)/binary/$(OTA_APPLICATION).stripped.elf
endif

GLOBAL_DEFINES     += PLATFORM_POWERSAVE_DEFAULT=1
GLOBAL_DEFINES     += PLATFORM_WLAN_POWERSAVE_STATS=1
GLOBAL_DEFINES     += APPLICATION_STACK_SIZE=10*1024
GLOBAL_DEFINES     += RX_PACKET_POOL_SIZE=40
GLOBAL_DEFINES     += AUTO_IP_ENABLED
GLOBAL_DEFINES     += WICED_USE_AUDIO
GLOBAL_DEFINES     += P2P_IP_ALLOCATION
GLOBAL_DEFINES     += WICED_DISABLE_MATH_NEWLIB
GLOBAL_DEFINES     += WICED_TCP_RX_DEPTH_QUEUE=20
GLOBAL_DEFINES     += WICED_TCP_WINDOW_SIZE=20*1024

WIFI_CONFIG_DCT_H  := wifi_config_dct.h
APPLICATION_DCT    := headset_dct.c

VALID_OSNS_COMBOS  := ThreadX-NetX_Duo FreeRTOS-LwIP
VALID_PLATFORMS    += BCM943909WCD* BCM943907* CYW943907*
INVALID_PLATFORMS  += BCM943907AEVAL* BCM943907WCD2* CYW943907AEVAL* Quicksilver_EVL
