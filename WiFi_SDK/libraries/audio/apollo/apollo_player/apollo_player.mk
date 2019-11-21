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

NAME := Lib_apollo_player

APOLLO_PLAYER_LIBRARY_NAME :=apollo_player.$(RTOS).$(NETWORK).$(HOST_ARCH).$(BUILD_TYPE).a

ifneq ($(wildcard $(CURDIR)$(APOLLO_PLAYER_LIBRARY_NAME)),)
$(info Using PREBUILT:  $(APOLLO_PLAYER_LIBRARY_NAME))
$(NAME)_PREBUILT_LIBRARY :=$(APOLLO_PLAYER_LIBRARY_NAME)
else
# Build from source (Broadcom internal)
$(info Building SRC:  $(APOLLO_PLAYER_LIBRARY_NAME))
include $(CURDIR)apollo_player_src.mk
endif # ifneq ($(wildcard $(CURDIR)$(APOLLO_PLAYER_LIBRARY_NAME)),)

GLOBAL_INCLUDES += .

GLOBAL_DEFINES  += WICED_USE_AUDIO

$(NAME)_COMPONENTS += audio/apollo/apollocore
$(NAME)_COMPONENTS += audio/apollo/audio_pll_tuner

$(NAME)_COMPONENTS += audio/apollo/audiopcm2
$(NAME)_COMPONENTS += audio/apollo/audioplc2
$(NAME)_COMPONENTS += audio/codec/wavpack/shortblocks

$(NAME)_COMPONENTS += audio/audio_render

$(NAME)_CFLAGS  :=
