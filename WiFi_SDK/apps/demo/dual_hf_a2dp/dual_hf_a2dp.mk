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

NAME := App_dual_hf_a2dp

$(NAME)_SOURCES    := dual_a2dp_hfp_audio.c \
                      bluetooth_dm.c \
                      bluetooth_nv.c \
                      app_dct.c \
                      bluetooth_a2dp.c \
                      bluetooth_hfp.c \
                      bluetooth_audio.c \
                      bluetooth_audio_common.c \
                      hashtable.c \

$(NAME)_COMPONENTS += libraries/audio/tone_player

# Comment the below flag, if application needs to be executed on Host Stack.
USE_BT_EMBED_MODE := 1

ifeq ($(USE_BT_EMBED_MODE), 1)
$(info "Embedded Stack")
$(NAME)_COMPONENTS += libraries/drivers/bluetooth/wiced_hci_bt\
                      libraries/protocols/wiced_hci

GLOBAL_DEFINES     := USE_WICED_HCI
$(NAME)_SOURCES    += bluetooth_audio_common_wiced_hci.c
EMBEDDED_APP_NAME := headset

VALID_PLATFORMS    = CYW943907WAE*
INVALID_PLATFORMS  += BCM9WCD1AUDIO BCM943909* BCM943907*

else
$(info "Host Stack")
$(NAME)_SOURCES    += bluetooth_audio_decoder.c \
                      bluetooth_audio_player.c \
                      bluetooth_audio_recorder.c

$(NAME)_COMPONENTS += libraries/drivers/bluetooth/dual_mode \
                      libraries/audio/codec/codec_framework \
                      libraries/audio/codec/sbc_if

VALID_PLATFORMS    := BCM9WCD1AUDIO BCM943909* BCM943907*
INVALID_PLATFORMS  := BCM943909B0FCBU BCM943907AEVAL* BCM943907WCD2*
endif

# Add button management on supported platforms
PLATFORM_SUPPORTING_BUTTONS := BCM943909WCD1_3* BCM943907WAE_1* BCM943907APS* BCM943907WCD1* BCM943907WAE2_1* BCM943907WCD2 CYW943907* Quicksilver_EVL
$(eval PLATFORM_SUPPORTING_BUTTONS := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_SUPPORTING_BUTTONS)))
ifneq ($(filter $(PLATFORM),$(PLATFORM_SUPPORTING_BUTTONS)),)
$(NAME)_COMPONENTS += inputs/button_manager
$(NAME)_SOURCES    += app_keypad.c
endif

ifneq (,$(findstring BCM943907WAE_1,$(PLATFORM)))
# Option to use ssd1306 128x64 i2c display over WICED_I2C_2
$(info *** Dual A2DP_audio using SSD1306 display on WAE_1! ***)
GLOBAL_DEFINES     += USE_AUDIO_DISPLAY
GLOBAL_DEFINES     += USE_NO_WIFI
$(NAME)_COMPONENTS += audio/display
endif

BT_CONFIG_DCT_H  := bt_config_dct.h

APPLICATION_DCT    := app_dct.c

GLOBAL_DEFINES     += WICED_USE_AUDIO \
                      BUILDCFG \
                      APPLICATION_STACK_SIZE=14336 \

# Define ENABLE_BT_PROTOCOL_TRACES to enable Bluetooth Protocol/Profile level traces
#GLOBAL_DEFINES     += ENABLE_BT_PROTOCOL_TRACES

GLOBAL_DEFINES     += USE_MEM_POOL WICED_DCT_INCLUDE_BT_CONFIG

ifneq (,$(findstring USE_MEM_POOL,$(GLOBAL_DEFINES)))
$(NAME)_SOURCES   += mem_pool/mem_pool.c
$(NAME)_INCLUDES  += ./mem_pool
#GLOBAL_DEFINES    += MEM_POOL_DEBUG
endif

# Default stack sizez of Hardware worker IO thread is 512 bytes.
# Since the keypress events are handled in this thread, this is not
# enough (might overflow when debug prints are enabled).
# Making it 1K gives enough room in stack when debug prints are enabled
# and for future code addition.
GLOBAL_DEFINES     += HARDWARE_IO_WORKER_THREAD_STACK_SIZE=1024

VALID_OSNS_COMBOS  := ThreadX-NetX_Duo

