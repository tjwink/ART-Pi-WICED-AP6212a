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

NAME := App_duktape

$(NAME)_SOURCES := duktape.c

WIFI_CONFIG_DCT_H := wifi_config_dct.h

$(NAME)_COMPONENTS := inputs/button_manager
$(NAME)_COMPONENTS += daemons/tftp
$(NAME)_COMPONENTS += scripting/javascript/duktape
$(NAME)_COMPONENTS += utilities/command_console
$(NAME)_COMPONENTS += utilities/command_console/wifi
$(NAME)_COMPONENTS += utilities/command_console/fs
$(NAME)_COMPONENTS += utilities/command_console/platform
$(NAME)_COMPONENTS += utilities/command_console/mallinfo
$(NAME)_COMPONENTS += utilities/command_console/duktape

# Use newer version of FATFS that fixes some bugs
$(NAME)_COMPONENTS += filesystems/FATFS
FATFS_VERSION := ver0.12b

GLOBAL_DEFINES += APPLICATION_STACK_SIZE=16*1024

WICED_DUKTAPE_OPT_MODULE_TIME = 1
WICED_DUKTAPE_OPT_MODULE_WIFI = 1
WICED_DUKTAPE_OPT_OBJECT_AUDIO = 1
WICED_DUKTAPE_OPT_OBJECT_XMLHTTPREQUEST = 1
WICED_DUKTAPE_OPT_OBJECT_TIME = 1
WICED_DUKTAPE_OPT_TESTS_API = 1

WICED_DUKTAPE_RESOURCE_LIST += tests/test_time.js \
                               tests/test_wifi.js \
                               tests/test_audio.js \
                               tests/test_xmlhttprequest.js \
                               tests/test_modules.js \
                               modules/greetings.js \
                               modules/nodejs_style/nodejs_style.js \
                               modules/nodejs_style/lib/goodbye.js \
                               modules/nodejs_style/lib/hello.js

# Only support platforms with DDR for now
VALID_PLATFORMS := BCM943909WCD1_3
VALID_OSNS_COMBOS := ThreadX-NetX ThreadX-NetX_Duo
