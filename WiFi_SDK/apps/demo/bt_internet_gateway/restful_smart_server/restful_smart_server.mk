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

NAME := RESTful_Smart_Server_Application

$(NAME)_SOURCES    := restful_smart_server_app.c \
                      wiced_bt_cfg.c

$(NAME)_COMPONENTS += daemons/bt_internet_gateway \
                      daemons/bt_internet_gateway/restful_smart_server

VALID_PLATFORMS    := BCM943340WCD1 \
                      BCM943909WCD* \
                      BCM9WCD1AUDIO \
                      BCM943438WLPTH_2 \
                      BCM94343WWCDA_ext \
                      BCM943907AEVAL2F* \
                      CY8CKIT_062 \
                      CYW9MCU7X9N364 \
                      CYW9WCD2REFAD2* \
                      CYW9WCD760PINSDAD2 \
                      BCM94343WWCD2 \
                      CYW943907WAE*

# To support Low memory platforms, disabling components which are not required
GLOBAL_DEFINES += WICED_CONFIG_DISABLE_SSL_SERVER \
                  WICED_CONFIG_DISABLE_DTLS \
                  WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY \
                  WICED_CONFIG_DISABLE_DES \
                  WICED_CONFIG_DISABLE_ADVANCED_SECURITY_CURVES

VALID_OSNS_COMBOS  := ThreadX-NetX_Duo FreeRTOS-LwIP
INVALID_PLATFORMS  += BCM943907WCD2*

WIFI_CONFIG_DCT_H  := wifi_config_dct.h

ifeq ($(PLATFORM),$(filter $(PLATFORM), CYW9MCU7X9N364))
USE_LIBC_PRINTF     := 0
endif
