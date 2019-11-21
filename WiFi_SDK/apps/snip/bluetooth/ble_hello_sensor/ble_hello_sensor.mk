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
# In order to use with 43xxx_WiFi, the HCD file generated must be downloaded into 20706/43012C0 RAM.

# Follow these instructions, in order for 43xxx_WiFi MCU applications to work with Embedded BT application (applies to 20706A2 and 43012C0 based platforms):
# 1.) Set the USE_BT_EMBED_MODE flag to 1 in the application Makefile
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

NAME := Bluetooth_Low_Energy_Hello_Sensor_Application

$(NAME)_SOURCES    := ble_hello_sensor.c \
                      wiced_bt_cfg.c

$(NAME)_INCLUDES   := .
$(NAME)_COMPONENTS := utilities/command_console

# Set USE_BT_EMBED_MODE to 0 for all platforms other than CY943907WAE3
USE_BT_EMBED_MODE := 0
PLATFORMS_FOR_POWER_SAVE := CYW943907WAE3 CYW94343WWCD1_EVB CYW943907WAE4

#ENABLE_APP_POWERSAVE macro will be used only for platforms mentioned in PLATFORM_FOR_POWER_SAVE_FLAG.
ifneq ($(filter $(PLATFORM),$(PLATFORMS_FOR_POWER_SAVE)),)
$(info "Power Save enabled")
GLOBAL_DEFINES   := ENABLE_APP_POWERSAVE
GLOBAL_DEFINES   += PLATFORM_POWERSAVE_DEFAULT=1

ifeq ($(PLATFORM),$(filter $(PLATFORM),CYW943907WAE4 CYW943907WAE3))
GLOBAL_DEFINES   += PLATFORM_MCU_POWERSAVE_MODE_INIT=PLATFORM_MCU_POWERSAVE_MODE_DEEP_SLEEP
#GLOBAL_DEFINES   += PLATFORM_MCU_POWERSAVE_MODE_INIT=PLATFORM_MCU_POWERSAVE_MODE_SLEEP
# GLOBAL_DEFINES   += WICED_DISABLE_WATCHDOG
GLOBAL_DEFINES   += APPLICATION_WATCHDOG_TIMEOUT_SECONDS=60
GLOBAL_DEFINES   += WICED_NO_WIFI
GLOBAL_DEFINES   += NO_WIFI_FIRMWARE
USE_BT_EMBED_MODE := 1
endif
endif

ifeq ($(USE_BT_EMBED_MODE), 1)
$(info "Embedded Stack")
$(NAME)_COMPONENTS += libraries/drivers/bluetooth/wiced_hci_bt \
                      libraries/protocols/wiced_hci

GLOBAL_DEFINES     += USE_WICED_HCI
EMBEDDED_APP_NAME      := headset
# The below BT chip related config is required for Fly-wire setup only, as this is not defined in the BCM943907WCD2 platform file.
ifeq ($(PLATFORM), BCM943907WCD2)
BT_CHIP            := 20706
BT_CHIP_REVISION   := A2
BT_CHIP_XTAL_FREQUENCY := 20MHz
endif

VALID_PLATFORMS    = CYW943907WAE3 BCM943907WCD2 CYW9WCD2REFAD2.CYW943012A0FCREF_3 CYW9WCD760PINSDAD2 CYW943907WAE4
INVALID_PLATFORMS  += BCM9WCD1AUDIO BCM943909*

else
$(info "Host Stack")
$(NAME)_COMPONENTS += libraries/drivers/bluetooth/low_energy

VALID_PLATFORMS = BCM943909WCD* \
                   BCM943340WCD1 \
                   BCM9WCD1AUDIO \
                   BCM943438WLPTH_2 \
                   BCM943907WAE_1 \
                   BCM943340WCD1 \
                   CYW94343WWCD1_EVB \
                   NEB1DX* \
                   BCM943438WCD1 \
                   BCM920739B0_EVAL \
                   BCM920739B0_HB2 \
                   CYW943907WAE3 \
                   CYW943907WAE4 \
                   BCM943907AEVAL2F \
                   CYW9MCU7X9N364 \
                   CYW9WCD2REFAD2.CYW943012A0FCREF_3 \
                   CYW9WCD760PINSDAD2 \
                   BCM94343WWCD2 \
                   CY8CKIT_062 \
                   CYW943012EVB* \
                   CYW943455EVB*

INVALID_PLATFORMS  += BCM943907WCD2*
endif

VALID_OSNS_COMBOS  := ThreadX-NetX_Duo FreeRTOS-LwIP

# Enable this flag to get bluetooth protocol traces
#GLOBAL_DEFINES     += ENABLE_BT_PROTOCOL_TRACES

