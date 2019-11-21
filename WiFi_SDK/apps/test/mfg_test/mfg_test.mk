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

NAME := App_mfg_test

$(NAME)_SOURCES := mfg_test_init.c

WIFI_FIRMWARE_BIN := firmware/$(WLAN_CHIP)/$(WLAN_CHIP)$(WLAN_CHIP_REVISION)-mfgtest.bin

$(NAME)_COMPONENTS += test/wl_tool \
                      utilities/wifi

# wl commands which dump a lot of data require big buffers.
# Added restriction only for Iwa platform due to memory limiation
ifneq ($(PLATFORM),$(filter $(PLATFORM), CYW9MCU7X9N364))
GLOBAL_DEFINES   += WICED_PAYLOAD_MTU=8320
endif

PLATFORM_WITH_LARGE_POOL_SIZE := BCM943909WCD1_3*
$(eval PLATFORM_WITH_LARGE_POOL_SIZE := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_WITH_LARGE_POOL_SIZE)))
ifeq ($(PLATFORM),$(filter $(PLATFORM),$(PLATFORM_WITH_LARGE_POOL_SIZE)))
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=10 \
                  RX_PACKET_POOL_SIZE=10 \
                  PBUF_POOL_TX_SIZE=8 \
                  PBUF_POOL_RX_SIZE=8 \
                  WICED_ETHERNET_DESCNUM_TX=32 \
                  WICED_ETHERNET_DESCNUM_RX=8 \
                  WICED_ETHERNET_RX_PACKET_POOL_SIZE=32+WICED_ETHERNET_DESCNUM_RX
else
ifneq ($(PLATFORM),$(filter $(PLATFORM), BCM943362WCD4_LPCX1769))
# Default values for all other platforms
# Values are higher for LwIP because it has sanity checks and will reject a config
# where the TCP window is larger then the total memory available for Rx buffers
GLOBAL_DEFINES   += TX_PACKET_POOL_SIZE=2 \
                    RX_PACKET_POOL_SIZE=2 \
                    PBUF_POOL_RX_SIZE=5
endif
endif

ifeq ($(PLATFORM),$(filter $(PLATFORM), CYW9MCU7X9N364))
USE_PLATFORM_ALLOCATED_POOL := 0
endif

# ENABLE for ethernet support
#$(NAME)_DEFINES   += MFG_TEST_ENABLE_ETHERNET_SUPPORT

INVALID_PLATFORMS := BCM943362WCD4_LPCX1769 \
                     BCM943362WCDA \
                     BCM9WCD2WLREFAD.BCM94334WLAGB \
                     BCM943909QT

ifeq ($(PLATFORM),$(filter $(PLATFORM),BCM943362WCD4_LPCX1769 BCM94334WLAGB BCM943909QT))
ifneq (yes,$(strip $(TESTER)))
$(error Platform not supported for Manufacturing test)
endif
endif

#==============================================================================
# iperf inclusion
#==============================================================================
#4390 does not have enough memory for iperf; enable iperf for all others
ifeq (,$(filter $(PLATFORM),BCM94390WCD1 BCM94390WCD2 BCM94390WCD3 BCM943364WCD1 CYW94343WWCD1_EVB BCM943438WCD1 BCM94343WWCD2))
MFG_TEST_ENABLE_IPERF :=1
GLOBAL_DEFINES   += MFG_TEST_ENABLE_IPERF
endif

# Uncomment the below line to Enable Iperf & TCP pool tunables for Low memory platforms: BCM943438WWCD1, BCM43364WCD1, CYW94343WWCD1_EVB, BCM94343WWCD2
#LOWMEM_ENABLE_IPERF := 1

# if LOWMEM_ENABLE_IPERF flag is enabled then enable Iperf and configure with limited network buffer pools for low memory platforms
ifeq ($(LOWMEM_ENABLE_IPERF),1)
ifeq ($(PLATFORM),$(filter $(PLATFORM), BCM943438WCD1))
# Enable iperf for low memory platforms such as: BCM943438WCD1 here at one place instead of touching the existing code below
MFG_TEST_ENABLE_IPERF :=1
GLOBAL_DEFINES   += MFG_TEST_ENABLE_IPERF
# Remove the older values to update new one
GLOBAL_DEFINES := $(filter-out TX_PACKET_POOL_SIZE=2 RX_PACKET_POOL_SIZE=2, $(GLOBAL_DEFINES))
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=4 \
                  RX_PACKET_POOL_SIZE=3
ifneq ($(NETWORK), LwIP)
# For NetX or NetX_Duo stack
GLOBAL_DEFINES += WICED_TCP_TX_DEPTH_QUEUE=2 \
                  WICED_TCP_WINDOW_SIZE=8192
endif # LwIP
endif # PLATFORM
endif # LOWMEM_ENABLE_IPERF

ifdef MFG_TEST_ENABLE_IPERF
$(NAME)_COMPONENTS += test/iperf
endif

#==============================================================================
# audio loopback inclusion
#==============================================================================
PLATFORM_SUPPORTING_AUDIO := BCM943909WCD1_3* BCM943907WAE_1* BCM943907APS* BCM943907WCD1* BCM943907WCD2*
$(eval PLATFORM_SUPPORTING_AUDIO := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORM_SUPPORTING_AUDIO)))
ifneq ($(filter $(PLATFORM),$(PLATFORM_SUPPORTING_AUDIO)),)
ifneq ($(filter $(RTOS)-$(NETWORK),ThreadX- ThreadX-NetX_Duo),)
MFG_TEST_ENABLE_AUDIO_LOOPBACK :=1
GLOBAL_DEFINES += MFG_TEST_ENABLE_AUDIO_LOOPBACK
endif
endif

ifdef MFG_TEST_ENABLE_AUDIO_LOOPBACK
$(NAME)_COMPONENTS += test/audio_loopback
GLOBAL_DEFINES     += WICED_AUDIO_LOOPBACK_LOG_ENABLED=0
endif

#==============================================================================
# Flag for app specific
#==============================================================================
GLOBAL_DEFINES   += MFG_TEST_APP
