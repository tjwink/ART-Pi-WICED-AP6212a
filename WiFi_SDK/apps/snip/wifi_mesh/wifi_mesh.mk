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

NAME := App_Wifi_Mesh

$(NAME)_INCLUDES := .

$(NAME)_SOURCES := wifi_mesh.c \
				   wifi_mesh_config.c

GLOBAL_DEFINES :=

WIFI_CONFIG_DCT_H := wifi_config_dct.h

APPLICATION_DCT    := wifi_mesh_dct.c


$(NAME)_COMPONENTS :=	utilities/command_console\
						utilities/command_console/wps \
						utilities/command_console/wifi \
						utilities/command_console/thread \
						utilities/command_console/ping \
						utilities/command_console/platform \
						utilities/command_console/tracex \
						utilities/command_console/mallinfo \
						utilities/command_console/dct \
						utilities/wiced_log \
						test/iperf
GLOBAL_DEFINES     += TX_PACKET_POOL_SIZE=40
GLOBAL_DEFINES     += RX_PACKET_POOL_SIZE=40
GLOBAL_DEFINES     += AUTO_IP_ENABLED
GLOBAL_DEFINES     += WICED_USE_WIFI_TWO_STA_INTERFACE
GLOBAL_DEFINES     += APPLICATION_STACK_SIZE=8000

VALID_PLATFORMS    :=
VALID_PLATFORMS    += BCM943909WCD*
VALID_PLATFORMS    += BCM943907*
VALID_PLATFORMS    += CYW943907WAE*
VALID_OSNS_COMBOS := ThreadX-NetX_Duo ThreadX-NetX
# Make sure we pick up the mesh-specific WLAN firmware & clm_blob files
WLAN_CHIP_BIN_TYPE :=-mesh
