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

NAME := Mesh_Application

$(NAME)_SOURCES    := mesh_network.c \
                      wiced_bt_cfg.c \
                      mesh_uri.c \
                      mesh_mqtt.c

WIFI_CONFIG_DCT_H := wifi_config_dct.h

APPLICATION_DCT := mesh_dct.c


$(NAME)_COMPONENTS += libraries/drivers/bluetooth/wiced_hci_bt \
                      libraries/protocols/wiced_hci \
                      protocols/MQTT \
                      libraries/utilities/JSON_parser \
                      libraries/daemons/HTTP_server

$(NAME)_RESOURCES  := apps/aws/iot/rootca.cer \
                      apps/aws/iot/mesh/client.cer \
                      apps/aws/iot/mesh/privkey.cer

#VALID_PLATFORMS    = CYW943907WAE*
#INVALID_PLATFORMS  += BCM943907AEVAL* BCM943907WCD2* CYW9MCU7X9N364

#EMBEDDED_APP_NAME      := mesh_gateway
USE_BT_EMBED_MODE := 1

ifeq ($(USE_BT_EMBED_MODE), 1)
$(info "Embedded Stack")
$(NAME)_COMPONENTS += libraries/drivers/bluetooth/wiced_hci_bt \
                      libraries/protocols/wiced_hci \
                      protocols/MQTT \
                      libraries/utilities/JSON_parser \
                      libraries/daemons/HTTP_server

GLOBAL_DEFINES     := USE_WICED_HCI
EMBEDDED_APP_NAME := mesh_gateway_proxy

VALID_PLATFORMS    = CYW943907WAE*
INVALID_PLATFORMS  += BCM943907AEVAL* BCM943907WCD2* CYW9MCU7X9N364
REBUILD_BLUETOOTH_FIRMWARE := 0

MAKE_STRING := "snip.mesh.mesh_gateway_proxy-CYW920706WCDEVAL"
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

#GLOBAL_DEFINES     += ENABLE_BT_PROTOCOL_TRACES


