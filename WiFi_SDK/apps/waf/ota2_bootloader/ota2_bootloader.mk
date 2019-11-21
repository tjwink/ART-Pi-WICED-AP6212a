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

NAME := App_OTA2_Bootloader_$(PLATFORM)

OTA2_SUPPORT := 1

$(NAME)_SOURCES    := ./ota2_bootloader.c

NoOS_FIQ_STACK     := 0
NoOS_IRQ_STACK     := 256
NoOS_SYS_STACK     := 0

APP_WWD_ONLY       := 1
NO_WIFI_FIRMWARE   := YES
NO_WIFI            := YES
NO_WICED_API       := 1

# stack needs to be big enough to handle the CRC32 calculation buffer
PLATFORMS_WITH_BIGGER_STACK := BCM943909WCD1_3* BCM943909B0FCBU BCM943907WAE_1* BCM943907APS* BCM943907AEVAL2F* BCM943907AEVAL1F* CYW943907WAE* CYW943907AEVAL*
$(eval PLATFORMS_WITH_BIGGER_STACK := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORMS_WITH_BIGGER_STACK)))
ifeq ($(filter $(PLATFORM),$(PLATFORMS_WITH_BIGGER_STACK)),)
NoOS_START_STACK   := 6800
GLOBAL_DEFINES     += DCT_CRC32_CALCULATION_SIZE_ON_STACK=256
else
NoOS_START_STACK   := 8000
GLOBAL_DEFINES     += DCT_CRC32_CALCULATION_SIZE_ON_STACK=2048
endif

GLOBAL_DEFINES     += WICED_NO_WIFI
GLOBAL_DEFINES     += WICED_DISABLE_MCU_POWERSAVE
GLOBAL_DEFINES     += WICED_DCACHE_WTHROUGH
GLOBAL_DEFINES     += NO_WIFI_FIRMWARE
GLOBAL_DEFINES     += BOOTLOADER
GLOBAL_DEFINES     += WICED_DISABLE_STDIO

GLOBAL_INCLUDES    += .
GLOBAL_INCLUDES    += ../../../libraries/utilities/linked_list

VALID_OSNS_COMBOS  := NoOS
VALID_BUILD_TYPES  := release

VALID_PLATFORMS    := BCM943909WCD1_3* BCM943907WAE_1* BCM943907WAE2_1* BCM943907AEVAL1_1 BCM943907AEVAL1F* BCM943907AEVAL2F* BCM943907WCD1 CYW943907WAE* CYW943907AEVAL*
VALID_PLATFORMS    += BCM943907WCD2 CYW954907AEVAL1F
INVALID_PLATFORMS  := BCM943909WCD1_3_SDIO CYW9MCU7X9N364
