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

NAME := App_audio_loopback

$(NAME)_COMPONENTS := test/audio_loopback

$(NAME)_SOURCES    := audio_loopback.c

GLOBAL_DEFINES     += WICED_USE_AUDIO

# Usage of wiced_log_msg/printf will require more stack
GLOBAL_DEFINES     += LOOPBACK_WORKER_THREAD_STACK_SIZE=5000

ifneq ($(filter $(PLATFORM),CYW943907AEVAL1F CYW954907AEVAL1F),)
# Use CS47L24 audio front end
USE_CS47L24_AUDIO  := 1
GLOBAL_DEFINES     += USE_CS47L24_AUDIO
GLOBAL_DEFINES     += TEST_TX_AUDIO_DEVICE=AUDIO_DEVICE_ID_CS47L24_DAC_LINE
GLOBAL_DEFINES     += TEST_RX_AUDIO_DEVICE=AUDIO_DEVICE_ID_CS47L24_ADC_DIGITAL_MIC
GLOBAL_DEFINES     += SAMPLE_FREQUENCY_IN_HZ=16000
endif

VALID_OSNS_COMBOS  := ThreadX-NetX_Duo FreeRTOS-LwIP

VALID_PLATFORMS    := BCM943909WCD* BCM943907WAE_1* BCM943907APS* BCM943907WCD* BCM9WCD1AUDIO
VALID_PLATFORMS    += BCM943907WAE2_1 CYW943907WAE*
VALID_PLATFORMS    += CYW943907AEVAL1F CYW954907AEVAL1F
