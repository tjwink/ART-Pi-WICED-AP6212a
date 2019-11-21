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

NAME := App_Multi_Image_0

$(NAME)_SOURCES := app_0.c

WIFI_CONFIG_DCT_H  := wifi_config_dct.h

APPLICATION_DCT    := common_dct.c

#add second app to the build
APP1_FILE = snip.multi_image_1-$(PLATFORM)
APP1 = build/$(APP1_FILE)/binary/$(APP1_FILE).stripped.elf

#To add a third app to the build, create the application and enable these two lines
#APP2_FILE = snip.multi_image_2-$(PLATFORM)
#APP2 = build/$(APP2_FILE)/binary/$(APP2_FILE).stripped.elf

GLOBAL_DEFINES  := PLATFORM_POWERSAVE_DEFAULT=1
GLOBAL_DEFINES  += PLATFORM_WLAN_POWERSAVE_STATS=1
GLOBAL_DEFINES  += PLATFORM_MCU_POWERSAVE_MODE_INIT=PLATFORM_MCU_POWERSAVE_MODE_DEEP_SLEEP
GLOBAL_DEFINES  += WICED_DEEP_SLEEP_SAVE_PACKETS_NUM=4
GLOBAL_DEFINES  += PLATFORM_SUPPORTS_LOW_POWER_MODES=1

$(NAME)_COMPONENTS := utilities/command_console 	 \
                      utilities/command_console/wifi \
                      utilities/command_console/dct  \
                      utilities/mini_printf 		 \

VALID_OSNS_COMBOS := ThreadX-NetX_Duo ThreadX-NetX
VALID_PLATFORMS   := BCM943909* BCM943907* BCM943903* CYW943907* Quicksilver_EVL
INVALID_PLATFORMS := BCM943909QT
