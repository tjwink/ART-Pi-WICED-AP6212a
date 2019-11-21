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

NAME := App_OverTheAir2_example

# undefine to skip check for Version checking of update while developing updates
# define (and provide version #) for production version testing
# You can provide version # during compilation (default is 0) by adding to build compile args
# <application>-<platform> ota2_image APP_VERSION_FOR_OTA2_MAJOR=x APP_VERSION_FOR_OTA2_MINOR=y
#GLOBAL_DEFINES      += CHECK_OTA2_UPDATE_VERSION=1
#WICED_ENABLE_TRACEX := 1

APPLICATION_DCT    := ota2_test_dct.c

GLOBAL_DEFINES     += PLATFORM_NO_DDR=1
GLOBAL_DEFINES     += APPLICATION_STACK_SIZE=16000
# stack needs to be big enough to handle the CRC32 calculation buffer
GLOBAL_DEFINES     += DCT_CRC32_CALCULATION_SIZE_ON_STACK=2048

$(NAME)_SOURCES    := ota2_test.c				\
					  ota2_test_config.c		\
					  ota2_test_dct.c

$(NAME)_COMPONENTS := utilities/command_console 	 \
                      utilities/command_console/wifi \
                      utilities/command_console/dct  \
                      utilities/wiced_log 		 	 \
                      utilities/mini_printf 		 \
                      daemons/ota2_service		 	 \
                      filesystems/wicedfs			 \
					  protocols/DNS 				 \
					  protocols/HTTP

# Uncomment to add Version checking of timed update (see use in OTA2 callback in ota2_test.c)
# You can provide version # during compilation (default is 0) by adding to build compile args
# <application>-<platform> ota2_image APP_VERSION_FOR_OTA2_MAJOR=x APP_VERSION_FOR_OTA2_MINOR=y
#CHECK_OTA2_UPDATE_VERSION := 1

#OTA SoftAp application
OTA_APPLICATION	:= snip.ota2_extract-$(PLATFORM)
OTA_APP    := build/$(OTA_APPLICATION)/binary/$(OTA_APPLICATION).stripped.elf

#GLOBAL_DEFINES     += CONSOLE_ENABLE_WL
ifneq (,$(findstring CONSOLE_ENABLE_WL,$(GLOBAL_DEFINES)))
# wl commands which dump a lot of data require big buffers.
GLOBAL_DEFINES   += WICED_PAYLOAD_MTU=8320
$(NAME)_COMPONENTS += test/wl_tool
endif

WIFI_CONFIG_DCT_H  := wifi_config_dct.h

ifdef WICED_ENABLE_TRACEX
$(info using tracex lib)
GLOBAL_DEFINES     += WICED_TRACEX_BUFFER_DDR_OFFSET=0x0
GLOBAL_DEFINES     += WICED_TRACEX_BUFFER_SIZE=0x200000
$(NAME)_COMPONENTS += test/TraceX
endif

PLATFORMS_WITH_MORE_DEFINES := BCM943909WCD1 BCM943909WCD1_3* BCM943907AEVAL2F* BCM943907AEVAL1F* BCM943907AEVAL1_1* CYW943907AEVAL*
$(eval PLATFORMS_WITH_MORE_DEFINES := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORMS_WITH_MORE_DEFINES)))
ifeq ($(PLATFORM),$(filter $(PLATFORM),$(PLATFORMS_WITH_MORE_DEFINES)))
GLOBAL_DEFINES     += WWD_TEST_NVRAM_OVERRIDE
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=5 \
                  RX_PACKET_POOL_SIZE=20 \
                  PBUF_POOL_TX_SIZE=8 \
                  PBUF_POOL_RX_SIZE=8 \
                  WICED_ETHERNET_DESCNUM_TX=32 \
                  WICED_ETHERNET_DESCNUM_RX=8 \
                  WICED_ETHERNET_RX_PACKET_POOL_SIZE=32+WICED_ETHERNET_DESCNUM_RX
endif
#tcp window size is usually 7k, reduce here due to need for reduced host memory usage
#and the host needing the extra processing time
#increase this (and provide more memory) if faster download speeds are desired
GLOBAL_DEFINES += WICED_TCP_WINDOW_SIZE=3072

VALID_OSNS_COMBOS  := ThreadX-NetX ThreadX-NetX_Duo

VALID_PLATFORMS    := BCM943909WCD1_3* BCM943907WAE_1* BCM943907WAE2_1* BCM943907AEVAL1_1 BCM943907AEVAL1F* BCM943907AEVAL2F* BCM943907WCD1 CYW943907WAE*
VALID_PLATFORMS    += BCM943907WCD2 CYW943907AEVAL1F Quicksilver_EVL CYW954907AEVAL1F
INVALID_PLATFORMS  := BCM943909WCD1_3_SDIO CYW9MCU7X9N364
