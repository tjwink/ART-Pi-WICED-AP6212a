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
# The 20706A2/43012C0 BT embedded applications are capable of interacting with 43xxx_WiFi MCU applications
# The embedded applications serve to abstract the details of Bluetooth protocols and profiles while allowing MCU application to handle with the business logic.
# The apps processor is typically connected over UART and can send commands and receive notifications.

# Follow these instructions, in order for 43xxx_WiFi MCU applications to work with Embedded BT application (applies to 20706A2 and 43012C0 based platforms):
# 1.) Set the BLUETOOTH_AUDIO_APP_MODE flag to BT_EMBED_MODE in the application Makefile to run the application in embedded mode. Or set the
#     BLUETOOTH_AUDIO_APP_MODE flag to BT_HYBRID_MODE in the application Makefile to run the application in hybrid mode
# 2.) By default, the Makefile picks the default bt_firmware_embedded_(EMBEDDED_APP_NAME).c firmware file,
# that is present in libraries/drivers/bluetooth/firmware/. Refer to the firmware.mk file for details.
# 3.) The application running on MCU shall use WICED_HCI mode to communicate with the Bluetooth embedded application
#
# To build a custom BT embedded application, follow the below steps:
# 1.) Open the 20706/43012 embedded application present under the respective Chip project in the WICED Studio IDE
# 2.) Modify/enhance the embedded application as desired.
# 3.) Build application to produce a downloadable hcd file.  For example
#     demo.headset-CYW920706WCDEVAL DIRECT_LOAD=1 build
# 4.) Convert '.hcd' file generated above to byte-array in a 'C' file using hcd2c.pl script.
#     Use the hcd2c.pl present under libraries/drivers/bluetooth/firmware/tools to generate the C file.
#     Sample command: perl hcd2c.pl -n headset-CYW920706WCDEVAL_40Mhz-rom-ram-Wiced-release.hcd > firmware.c
# 5.) Make sure that 'C' file generated above is syntatically correct and uses 'brcm_patchram_buf' and brcm_patch_ram_length' names
#      to represent byte-array and byte-array length
#     For example :
#        const uint8_t brcm_patchram_buf[] = { << byte_array here >> };
#        const int brcm_patch_ram_length = sizeof(brcm_patchram_buf);
#     For reference, check this file: bt_firmware_embedded_headset.c file.
# 6.) Rename this C file to bt_firmware_embedded_(EMBEDDED_APP_NAME).c and place under the respective BT_CHIP folder in the firmware path mentioned above.
# 7.) The 43xx_Wifi_MCU application will link & compile the updated C file.
#
# To Automate the above mentioned steps from 2) to 6), make the following changes in the make file:
#     a) Enable REBUILD_BLUETOOTH_FIRMWARE flag to 1.
#     b) Enter the desired Embedded Application target make string to MAKE_STRING flag.
#     c) Enter the desired Embedded Application name to EMBEDDED_APP_NAME flag.
#     d) Enter the Bluetooth XTAL freq to BT_CHIP_XTAL.
#     e) The Bluetooth chip name will be taken by default from the Platform file.
#     NOTE: By doing this step, the existing firmware file will be over written by the new firmware file.
#           However,the existing firmware file would be backed up as bt_firmware_embedded_back_(app_name).c in the same folder structure.

NAME := App_Bluetooth_Audio

$(NAME)_SOURCES    := bluetooth_audio.c \
                      bluetooth_audio_nv.c

$(NAME)_COMPONENTS := utilities/command_console \
                      utilities/wiced_log

#Set BLUETOOTH_AUDIO_APP_MODE to BT_EMBED_MODE if the application needs to be executed in the embedded mode
BT_EMBED_MODE :=1
#Set BLUETOOTH_AUDIO_APP_MODE to HOST_MODE if the application needs to be executed in the host mode
HOST_MODE :=2
#Set BLUETOOTH_AUDIO_APP_MODE to BT_HYBRID_MODE if the application needs to be executed in the hybrid mode
BT_HYBRID_MODE :=3

BLUETOOTH_AUDIO_APP_MODE := $(BT_EMBED_MODE)

VALID_OSNS_COMBOS  := ThreadX-NetX_Duo ThreadX-NetX


ifeq ($(BLUETOOTH_AUDIO_APP_MODE), $(BT_EMBED_MODE))
$(info "Embedded Stack")
$(NAME)_SOURCES    += bluetooth_audio_common_wiced_hci.c
else ifeq ($(BLUETOOTH_AUDIO_APP_MODE), $(HOST_MODE))
$(info "Host Stack")
$(NAME)_COMPONENTS += libraries/drivers/bluetooth/dual_mode

VALID_PLATFORMS    := BCM9WCD1AUDIO BCM943909* BCM943907WAE* BCM943907APS* BCM943907WCD1* CYW943907WAE*
INVALID_PLATFORMS  := BCM943909QT BCM943907WCD2*
else
$(info "Hybrid Mode")
GLOBAL_DEFINES     += USE_HYBRID_MODE
endif

#Common for BT_EMBED_MODE and BT_HYBRID_MODE
ifneq ($(BLUETOOTH_AUDIO_APP_MODE), $(HOST_MODE))

$(NAME)_COMPONENTS += libraries/drivers/bluetooth/wiced_hci_bt\
                      libraries/protocols/wiced_hci

GLOBAL_INCLUDES    += libraries/drivers/bluetooth/wiced_hci_bt
GLOBAL_DEFINES     += USE_WICED_HCI

VALID_PLATFORMS    := CYW943907WAE*
INVALID_PLATFORMS  += BCM9WCD1AUDIO BCM943909* BCM943907* CYW9MCU7X9N364

EMBEDDED_APP_NAME := headset
PLATFORMS_FOR_AUDIO := CYW943907WAE3 CYW943907WAE4
REBUILD_BLUETOOTH_FIRMWARE := 0

ifneq ($(filter $(PLATFORM),$(PLATFORMS_FOR_AUDIO)),)
MAKE_STRING := "demo.audio.headset-CYW920706WCDEVAL"
BT_CHIP_XTAL := 40Mhz

ifeq ($(REBUILD_BLUETOOTH_FIRMWARE),1)
$(info "Generating a New Firmware ...")
#Perl script takes 4 Arguments:
# 1)make_string 2)bt_chip 3)bt_xtal_freq 4)EMBEDDED_APP_NAME

FETCH_FM_COMPONENT := $(PERL) $(TOOLS_ROOT)/BT/scripts/auto_make.pl $(MAKE_STRING) $(BT_CHIP) $(BT_CHIP_XTAL) $(EMBEDDED_APP_NAME)
EXEC := $(shell $(FETCH_FM_COMPONENT))

ERR ?=$(SOURCE_ROOT)temp.txt
ifneq ($(wildcard $(ERR)),)
RET_VAL := $(shell $(CAT) "$(SOURCE_ROOT)temp.txt" )
$(error Aborting due to errors in BT-SDK:$(RET_VAL))
endif

else
$(info "Building existing Firmware!!")
endif
endif
endif

#Common for HOST_MODE and BT_HYBRID_MODE
ifneq ($(BLUETOOTH_AUDIO_APP_MODE), $(BT_EMBED_MODE))

$(NAME)_COMPONENTS += libraries/audio/codec/codec_framework \
                      libraries/audio/codec/sbc_if
$(NAME)_SOURCES    += bluetooth_audio_player.c \
                      bluetooth_audio_decoder.c

PLATFORM_USING_AUDIO_PLL := BCM943909WCD1_3* BCM943909B0FCBU BCM943907WAE_1* BCM943907WAE2_1* BCM943907APS* BCM943907WCD1* CYW943907* Quicksilver_EVL
$(eval PLATFORM_USING_AUDIO_PLL := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_USING_AUDIO_PLL)))
ifneq ($(filter $(PLATFORM),$(PLATFORM_USING_AUDIO_PLL)),)
# Use h/w audio PLL to tweak the master clock going into the I2S engine
GLOBAL_DEFINES     += USE_AUDIO_PLL
$(NAME)_SOURCES    += bluetooth_audio_pll_tuning.c
$(NAME)_COMPONENTS += audio/apollo/audio_pll_tuner
endif
endif

BT_CONFIG_DCT_H := bt_config_dct.h

GLOBAL_DEFINES += BUILDCFG \
                  WICED_USE_AUDIO \
                  WICED_NO_WIFI \
                  NO_WIFI_FIRMWARE \
                  TX_PACKET_POOL_SIZE=1 \
                  RX_PACKET_POOL_SIZE=1 \
                  USE_MEM_POOL \
                  WICED_DCT_INCLUDE_BT_CONFIG

#GLOBAL_DEFINES += WICED_DISABLE_WATCHDOG

ifneq (,$(findstring USE_MEM_POOL,$(GLOBAL_DEFINES)))
$(NAME)_SOURCES   += mem_pool/mem_pool.c
$(NAME)_INCLUDES  += ./mem_pool
#GLOBAL_DEFINES    += MEM_POOL_DEBUG
endif

# Define ENABLE_BT_PROTOCOL_TRACES to enable Bluetooth protocol/profile level
# traces.
#GLOBAL_DEFINES     += ENABLE_BT_PROTOCOL_TRACES

NO_WIFI_FIRMWARE   := YES
