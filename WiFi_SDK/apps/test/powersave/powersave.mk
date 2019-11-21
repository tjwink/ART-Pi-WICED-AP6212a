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

NAME := App_powersave

$(NAME)_SOURCES := powersave.c

$(NAME)_COMPONENTS += utilities/command_console \
                      utilities/command_console/wps \
                      utilities/command_console/wifi \
                      utilities/command_console/thread \
                      utilities/command_console/ping \
                      utilities/command_console/platform \
                      utilities/command_console/mallinfo \
                      utilities/command_console/ethernet \
                      utilities/command_console/p2p \
                      utilities/command_console/tracex \
                      test/iperf

GLOBAL_DEFINES += STDIO_BUFFER_SIZE=128
GLOBAL_DEFINES += PLATFORM_POWERSAVE_DEFAULT=1 PLATFORM_WLAN_POWERSAVE_STATS=1
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=10 \
                  RX_PACKET_POOL_SIZE=20 \
                  WICED_TCP_TX_DEPTH_QUEUE=8 \
                  WICED_ETHERNET_DESCNUM_TX=32 \
                  WICED_ETHERNET_DESCNUM_RX=8 \
                  WICED_ETHERNET_RX_PACKET_POOL_SIZE=32+WICED_ETHERNET_DESCNUM_RX \
                  WICED_TCP_WINDOW_SIZE=8192
GLOBAL_DEFINES += ADD_NETX_EAPOL_SUPPORT
GLOBAL_DEFINES += WICED_DCT_INCLUDE_P2P_CONFIG
GLOBAL_DEFINES += WPRINT_PLATFORM_PERMISSION

$(NAME)_DEFINES += CONSOLE_INCLUDE_ETHERNET \
                   CONSOLE_INCLUDE_P2P \
                   CONSOLE_ENABLE_IPERF \
                   CONSOLE_ENABLE_THREADS

# Define as 1 if want to include WL commands into application
CONSOLE_ENABLE_WL := 0

VALID_PLATFORMS   := BCM943909* BCM943907* BCM943903* CYW943907* Quicksilver_EVL
INVALID_PLATFORMS := BCM943909QT
