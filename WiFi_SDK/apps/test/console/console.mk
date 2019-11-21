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

NAME := App_console

#==============================================================================
# global defines
#==============================================================================
WICED_USE_BSD_SOCKET ?= 0
WICED_PACKET_POOL_SIZE_128 ?= 0
WICED_COM_PACKET_POOL_SIZE ?= 0

ifeq ($(WICED_USE_BSD_SOCKET),1)
ifeq (NetX, $(NETWORK))
GLOBAL_DEFINES += ENABLE_NX_BSD_SOCKET
endif

ifeq (NetX_Duo, $(NETWORK))
GLOBAL_DEFINES += ENABLE_NXD_BSD_SOCKET
endif

# default network
ifeq ($(strip $(NETWORK)),)
GLOBAL_DEFINES += ENABLE_NXD_BSD_SOCKET
endif
endif


#==============================================================================
# Console specific files
#==============================================================================
$(NAME)_SOURCES := wiced_init.c

$(NAME)_COMPONENTS += utilities/command_console

#==============================================================================
# Additional command console modules
#==============================================================================
$(NAME)_COMPONENTS += utilities/command_console/wps \
                      utilities/command_console/wifi \
                      utilities/command_console/thread \
                      utilities/command_console/ping \
                      utilities/command_console/platform \
                      utilities/command_console/tracex \
                      utilities/command_console/mallinfo \
                      utilities/wiced_log \
                      utilities/wifi

ifeq ($(PLATFORM),$(filter $(PLATFORM), BCM943438WCD1 BCM943907WAE2_1 CYW943907WAE* CYW954907AEVAL1F* ))
$(NAME)_COMPONENTS += utilities/command_console/bt_hci \
                      drivers/bluetooth/mfg_test
GLOBAL_DEFINES += BT_BUS_RX_FIFO_SIZE=512
$(NAME)_DEFINES += CONSOLE_INCLUDE_BT
endif

# Disable P2P component for BCM9WCD2REFAD.BCM943012A0FCREF_3 platform due to limitation of APP_CODE lenth.
# To use P2P component, get rid of BCM9WCD2REFAD.BCM943012A0FCREF_3 and some unnecessary components.
ifeq ($(PLATFORM),$(filter $(PLATFORM), BCM94390WCD1 BCM94390WCD2 BCM94390WCD3 BCM943341WAE BCM943341WCD1 BCM943340WCD1 BCM9WCD2REFAD.BCM943012A0FCREF_3 ))
CONSOLE_NO_P2P :=1
endif

BCM94390x_ETHERNET_PLATFORMS := BCM943909* BCM943907* CYW943907AEVAL1F* CYW954907AEVAL1F*
$(eval BCM94390x_ETHERNET_PLATFORMS := $(call EXPAND_WILDCARD_PLATFORMS,$(BCM94390x_ETHERNET_PLATFORMS)))
ifeq ($(PLATFORM),$(filter $(PLATFORM), $(BCM94390x_ETHERNET_PLATFORMS)))
$(NAME)_COMPONENTS += utilities/command_console/ethernet
$(NAME)_DEFINES += CONSOLE_INCLUDE_ETHERNET
endif


PLATFORMS_WITH_AUDIO := BCM943909WCD* BCM943907WAE_1* BCM943907APS* BCM943907WCD* BCM9WCD1AUDIO
PLATFORMS_WITH_AUDIO += CYW943907WAE3 CYW943907WAE4 BCM943907WAE2_1
$(eval PLATFORMS_WITH_AUDIO := $(call EXPAND_WILDCARD_PLATFORMS,$(PLATFORMS_WITH_AUDIO)))
ifeq ($(PLATFORM),$(filter $(PLATFORM), $(PLATFORMS_WITH_AUDIO)))
$(NAME)_DEFINES    += CONSOLE_INCLUDE_AUDIO
$(NAME)_COMPONENTS += utilities/command_console/audio

# only 943907WAE platforms have support for i2s1 at the moment
ifneq (,$(filter $(PLATFORM),BCM943907WAE2_1))
#   By default, AK4961 codec is connected via I2S0. The "digital" audio loopback test requires I2S "in"
#   and "out" lines to be shorted together to form a return path for audio data.
#   However, on i.e. 943907WAE dev board, I2S0_SD_OUT_C and I2S0_SD_IN_C lines are not exposed on the PCB.
#   If WICED_AUDIO_ROUTE_RECONFIG_ENABLE is defined, then data port can be reconfigured to I2S1 on supported
#   platforms. This bus has test points on all the lines, thus making it possible to add a jumper.
#   If it is not defined, then the standard configuration is used (requires a loopback on the
#   analog side and a different approach to data verification)

GLOBAL_DEFINES   += WICED_AUDIO_ROUTE_RECONFIG_ENABLE
endif

endif

BCM943907AEVAL1F_PLATFORMS := BCM943907AEVAL1F* CYW943907AEVAL1F* CYW954907AEVAL1F*
$(eval BCM943907AEVAL1F_PLATFORMS := $(call EXPAND_WILDCARD_PLATFORMS,$(BCM943907AEVAL1F_PLATFORMS)))
ifeq ($(PLATFORM),$(filter $(PLATFORM), $(BCM943907AEVAL1F_PLATFORMS)))
# Only include FileX filesystem when we are building for ThreadX
ifeq ($(RTOS),ThreadX)
WICED_SDMMC_SUPPORT := yes
$(NAME)_COMPONENTS += utilities/command_console/fs
$(NAME)_COMPONENTS += filesystems/FileX
$(NAME)_DEFINES += CONSOLE_INCLUDE_FILESYSTEM
endif
ifeq ($(BUILD_TYPE),debug)
# GLOBAL_DEFINES += BCMDBG
# GLOBAL_DEFINES += WPRINT_ENABLE_WICED_DEBUG WPRINT_ENABLE_WICED_ERROR
endif
endif

#==============================================================================
# Includes
#==============================================================================
$(NAME)_INCLUDES := .

#==============================================================================
# Configuration
#==============================================================================

#==============================================================================
# Global defines
#==============================================================================


#Platforms & combinations with enough memory to fit WL tool, can declare CONSOLE_ENABLE_WL := 1
#CONSOLE_ENABLE_WL := 1

#Increase packet pool size for particular platforms
#To use Enterprise Security please use below commented settings
#GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=10 \
#                  RX_PACKET_POOL_SIZE=10
#This platform need a specific setting for passing Enterprise security testing. and remove some commands to free memory space,
#by uncommenting below line.
ifeq ($(PLATFORM),$(filter $(PLATFORM),  BCM943364WCD1 CYW94343WWCD1_EVB BCM94343WWCD2 NEB1DX_01 BCM943438WCD1 CY8CKIT_062 CYW9WCD2REFAD2.CYW943012A0FCREF_3 CYW9WCD760PINSDAD2 ))
# Disable components not needed due to application size limitation on this platform
CONSOLE_NO_P2P ?=1
CONSOLE_DISABLE_TRACEX_COMMANDS := 1

$(NAME)_DEFINES    += CONSOLE_DISABLE_TRACEX_COMMANDS
$(NAME)_DEFINES    += CONSOLE_DISABLE_MALLINFO_COMMANDS

#GLOBAL_DEFINES += CONSOLE_DISABLE_ENTERPRISE_COMMANDS
#GLOBAL_DEFINES += WICED_DISABLE_TLS

ifeq ($(WICED_DISABLE_COMMON_PKT_POOL),1)
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=14 \
                  RX_PACKET_POOL_SIZE=12 \
                  WICED_TCP_TX_DEPTH_QUEUE=10 \
                  WICED_TCP_WINDOW_SIZE=131072
else #WICED_DISABLE_COMMON_PKT_POOL
#This flag enables the NetX stack to use commong packet pool for both RX and TX direction
# Also this create a smaller packet pool of 128 bytes.
#To use Enterprise Security please use below commented settings
#GLOBAL_DEFINES += WICED_USE_COMMON_PKT_POOL \
#                  COM_PACKET_POOL_SIZE=15
ifeq ($(CONSOLE_NO_P2P),1)
ifeq ($(CONSOLE_ENABLE_WL),1)
#if the CONSOLE_ENABLE_WL is set to 1, there is no memory hence reduce the
#packet pool size to 5 from 10 and COM_PACKET_POOL_SIZE=10 from 26 for CYW943012C0 chip
#so the overall heap memory is increased for WL commands
#with this change IPERF throughput tests should not be run!!!!
ifeq ($(PLATFORM),$(filter $(PLATFORM), CYW9WCD2REFAD2.CYW943012A0FCREF_3 CYW9WCD760PINSDAD2 ))
WICED_PACKET_POOL_SIZE_128 := 5
WICED_COM_PACKET_POOL_SIZE := 10
else
WICED_PACKET_POOL_SIZE_128 := 10
WICED_COM_PACKET_POOL_SIZE := 26
endif
else
WICED_PACKET_POOL_SIZE_128 := 10
WICED_COM_PACKET_POOL_SIZE := 26
endif
GLOBAL_DEFINES += WICED_USE_COMMON_PKT_POOL \
                  PACKET_POOL_SIZE_128=$(WICED_PACKET_POOL_SIZE_128) \
                  COM_PACKET_POOL_SIZE=$(WICED_COM_PACKET_POOL_SIZE) \
                  WICED_TCP_TX_DEPTH_QUEUE=24 \
                  WICED_UDP_QUEUE_MAX=8 \
                  WICED_TCP_WINDOW_SIZE=131072
else
WICED_PACKET_POOL_SIZE_128 := 10
WICED_COM_PACKET_POOL_SIZE := 22
GLOBAL_DEFINES += WICED_USE_COMMON_PKT_POOL \
                  PACKET_POOL_SIZE_128=$(WICED_PACKET_POOL_SIZE_128) \
                  COM_PACKET_POOL_SIZE=$(WICED_COM_PACKET_POOL_SIZE) \
                  WICED_TCP_TX_DEPTH_QUEUE=20 \
                  WICED_UDP_QUEUE_MAX=8 \
                  WICED_TCP_WINDOW_SIZE=131072
endif

# This flag makes the netx stack to use smaller 128 byte pkt pool (PACKET_POOL_SIZE_128) for it's IP operation.
# Netx uses this pool for tcp ack sending, to fragment a big IP packet etc. For various operation 128 byte pool is suffice.
# But if you requires handing very big IP packet (like a ping with 10K ping), then DONOT SET THIS FLAG, as this big IP
# need to be fragmented, which can't be handled using smaller pool.
# As most of the appilication request IP packet of MTU size, and doesn't require such a big IP packet, it is enabled by default
GLOBAL_DEFINES += WICED_USE_128_POOL_FOR_IP_STACK
endif #WICED_DISABLE_COMMON_PKT_POOL
else
BCM943x_PLATFORMS := BCM9WCD2REFAD.*
$(eval BCM943x_PLATFORMS := $(call EXPAND_WILDCARD_PLATFORMS,$(BCM943x_PLATFORMS)))
ifeq ($(PLATFORM),$(filter $(PLATFORM), $(BCM943x_PLATFORMS)))
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=16 \
                  RX_PACKET_POOL_SIZE=12 \
                  WICED_TCP_TX_DEPTH_QUEUE=15 \
                  WICED_TCP_WINDOW_SIZE=16384
GLOBAL_DEFINES += CONSOLE_DISABLE_ENTERPRISE_COMMANDS

else
ifeq ($(PLATFORM),$(filter $(PLATFORM),  BCM94390WCD2 ))
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=15 \
                  RX_PACKET_POOL_SIZE=10 \
                  WICED_TCP_TX_DEPTH_QUEUE=8 \
                  WICED_TCP_WINDOW_SIZE=8192
else
# Set 43909 specific packet pool settings
BCM94390x_PLATFORMS := BCM943909* BCM943907* BCM943903* CYW943907* Quicksilver_EVL
$(eval BCM94390x_PLATFORMS := $(call EXPAND_WILDCARD_PLATFORMS,$(BCM94390x_PLATFORMS)))
ifeq ($(PLATFORM),$(filter $(PLATFORM), $(BCM94390x_PLATFORMS)))
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=40 \
                  RX_PACKET_POOL_SIZE=40 \
                  WICED_TCP_TX_DEPTH_QUEUE=32 \
                  WICED_ETHERNET_DESCNUM_TX=32 \
                  WICED_ETHERNET_DESCNUM_RX=8 \
                  WICED_ETHERNET_RX_PACKET_POOL_SIZE=40+WICED_ETHERNET_DESCNUM_RX \
                  WICED_TCP_WINDOW_SIZE=32768
# Enlarge stack size of test.console for solving WPA-Enterprise issue of BCM4390x.
GLOBAL_DEFINES += CONSOLE_THREAD_STACK_SIZE=8*1024
else
CYW943EVBx_PLATFORMS := CYW943012EVB* CYW943455EVB*
$(eval CYW943EVBx_PLATFORMS := $(call EXPAND_WILDCARD_PLATFORMS, $(CYW943EVBx_PLATFORMS)))
ifeq ($(PLATFORM), $(filter $(PLATFORM), $(CYW943EVBx_PLATFORMS)))
$(info <<<<< CYW943EVBx Platform BUFFER POOLS INITIALIZED >>>>>)
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=60 \
                  RX_PACKET_POOL_SIZE=30 \
                  WICED_TCP_TX_DEPTH_QUEUE=32 \
                  WICED_TCP_WINDOW_SIZE=131072

GLOBAL_DEFINES += CONSOLE_THREAD_STACK_SIZE=8*1024
else
#set 54907 specific settings to support 11AC throughput
CYW95490x_PLATFORMS := CYW954907*
$(eval CYW95490x_PLATFORMS := $(call EXPAND_WILDCARD_PLATFORMS,$(CYW95490x_PLATFORMS)))
ifeq ($(PLATFORM), $(filter $(PLATFORM), $(CYW95490x_PLATFORMS)))
ifneq (LwIP, $(NETWORK))
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=200 \
                  RX_PACKET_POOL_SIZE=200 \
                  WICED_TCP_TX_DEPTH_QUEUE=150 \
                  WICED_ETHERNET_DESCNUM_TX=32 \
                  WICED_ETHERNET_DESCNUM_RX=8 \
                  WICED_ETHERNET_RX_PACKET_POOL_SIZE=40+WICED_ETHERNET_DESCNUM_RX \
                  WICED_TCP_WINDOW_SIZE=65535
else
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=85 \
                  RX_PACKET_POOL_SIZE=85 \
                  WICED_ETHERNET_DESCNUM_TX=32 \
                  WICED_ETHERNET_DESCNUM_RX=8 \
                  WICED_ETHERNET_RX_PACKET_POOL_SIZE=40+WICED_ETHERNET_DESCNUM_RX
endif
# Enlarge stack size of test.console for solving WPA-Enterprise issue of BCM4390x.
GLOBAL_DEFINES += CONSOLE_THREAD_STACK_SIZE=8*1024
else

# Otherwise use default values
endif
endif
endif
endif
endif
endif
#==============================================================================
# IAR heap size config (Not used by GCC)
#==============================================================================
ifeq ($(PLATFORM),$(filter $(PLATFORM), BCM943438WCD1))
DEFAULT_GLOBAL_HEAPSIZE = 0x2000
endif

# some platform need a specific setting for passing tcp and udp testing. and remove some commands to free memory space.
ifeq ($(PLATFORM),$(filter $(PLATFORM),BCM943340WCD1 BCM943341WCD1 BCM943362WCD4 ))
# Enterprise security is disabled by default on these (i.e., the platforms listed in the above line) platforms.
# Comment out the CONSOLE_DISABLE_ENTERPRISE_COMMANDS to enable enterprise security support for these platforms.
# This would tune the system parameters to free up the RAM space required for supporting enterprise security feature.
CONSOLE_DISABLE_ENTERPRISE_COMMANDS:=1
CONSOLE_NO_P2P :=1
CONSOLE_DISABLE_WPS_COMMANDS := 1
ifeq ($(CONSOLE_DISABLE_WPS_COMMANDS),1)
$(NAME)_DEFINES    += CONSOLE_DISABLE_WPS_COMMANDS
endif
ifeq ($(CONSOLE_DISABLE_ENTERPRISE_COMMANDS),1)
GLOBAL_DEFINES += WWD_ENABLE_STATS
$(NAME)_DEFINES    += CONSOLE_DISABLE_ENTERPRISE_COMMANDS
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=16 \
                  RX_PACKET_POOL_SIZE=16 \
                  WICED_TCP_TX_DEPTH_QUEUE=5 \
                  WICED_TCP_RX_DEPTH_QUEUE=5 \
                  WICED_TCP_WINDOW_SIZE=16384
else
CONSOLE_DISABLE_TRACEX_COMMANDS := 1
$(NAME)_DEFINES    += CONSOLE_DISABLE_TRACEX_COMMANDS
GLOBAL_DEFINES += TX_PACKET_POOL_SIZE=8 \
                  RX_PACKET_POOL_SIZE=8 \
                  WICED_TCP_TX_DEPTH_QUEUE=5 \
                  WICED_TCP_RX_DEPTH_QUEUE=5 \
                  WICED_TCP_WINDOW_SIZE=16384
endif
endif

INVALID_PLATFORMS := BCM943362WCD4_LPCX1769 \
                     BCM943362WCDA \
                     STMDiscovery411_BCM43438

#==============================================================================
# Wl tool inclusion
#==============================================================================
# Platforms & combinations with enough memory to fit WL tool, can declare CONSOLE_ENABLE_WL := 1
CONSOLE_ENABLE_WL ?= 0
# Enable wl commands which require large data buffer. ex: wl curpower, wl dump ampdu, ...
# set WL_COMMAND_LARGE_BUFFER=1 to enable this. Default is disabled.
WL_COMMAND_LARGE_BUFFER ?= 0


ifeq ($(PLATFORM),$(filter $(PLATFORM), CYW9MCU7X9N364))
# Disable components not needed due to application size limitation on this platform
CONSOLE_NO_P2P :=1
CONSOLE_DISABLE_WPS_COMMANDS := 1
CONSOLE_DISABLE_TRACEX_COMMANDS := 1

GLOBAL_DEFINES     += CONSOLE_DISABLE_ENTERPRISE_COMMANDS
$(NAME)_DEFINES    += CONSOLE_DISABLE_WPS_COMMANDS
$(NAME)_DEFINES    += CONSOLE_DISABLE_TRACEX_COMMANDS
$(NAME)_DEFINES    += CONSOLE_DISABLE_MALLINFO_COMMANDS
$(NAME)_DEFINES    += CONSOLE_INCLUDE_WIFI_LIMITED_SET_COMMANDS
$(NAME)_DEFINES    += CONSOLE_INCLUDE_BT_THROUGHPUT_TEST_COMMANDS

GLOBAL_DEFINES += WICED_DISABLE_TLS

$(NAME)_COMPONENTS += utilities/command_console/bt

endif

# Disable related component for supporting wl command due to limitation of APP_CODE length on BCM9WCD2REFAD.BCM943455C0WLREF.
ifeq ($(PLATFORM),$(filter $(PLATFORM), BCM9WCD2REFAD.BCM943455C0WLREF ))
ifeq ($(CONSOLE_ENABLE_WL),1)
CONSOLE_NO_P2P :=1
CONSOLE_NO_IPERF :=1
CONSOLE_DISABLE_WIFI_COMMANDS ?= 1
CONSOLE_DISABLE_WPS_COMMANDS ?= 1
CONSOLE_DISABLE_TRACEX_COMMANDS ?= 1
$(NAME)_DEFINES    += CONSOLE_DISABLE_WIFI_COMMANDS
$(NAME)_DEFINES    += CONSOLE_DISABLE_WPS_COMMANDS
$(NAME)_DEFINES    += CONSOLE_DISABLE_TRACEX_COMMANDS
endif
endif

ifeq ($(CONSOLE_ENABLE_WL),1)
ifeq ($(WL_COMMAND_LARGE_BUFFER),1)
# Some wl command might still not work when WICED_PAYLOAD_MTU=2560 (ex: scanresults might fail when there're many AP)
# Increasing WICED_PAYLOAD_MTU will increase the total packet buffer pools size and might not fit into the available ram size.
# 2560 is chosen for 1) selected wl commands <ex: curpower, dump ampdu, ...>, 2) acceptable packet pools size.
GLOBAL_DEFINES += WICED_PAYLOAD_MTU=2560
endif
endif

#==============================================================================
# Provision to replace wlan production fw with mfg_test FW
#==============================================================================
CONSOLE_USE_MFG_TEST_FW ?= 0
ifeq ($(CONSOLE_USE_MFG_TEST_FW),1)
ifneq ($(PLATFORM),$(filter $(PLATFORM), BCM943364WCD1 CYW94343WWCD1_EVB BCM943438WCD1 BCM94343WWCD2))
WIFI_FIRMWARE_LOCATION := WIFI_FIRMWARE_IN_RESOURCES
WIFI_FIRMWARE_BIN := firmware/$(WLAN_CHIP)/$(WLAN_CHIP)$(WLAN_CHIP_REVISION)-mfgtest.bin
else
# Set the WIFI firmware in multi application file system to point to firmware
WIFI_FIRMWARE_LOCATION  := WIFI_FIRMWARE_IN_MULTI_APP
MULTI_APP_WIFI_FIRMWARE := resources/firmware/$(WLAN_CHIP)/$(WLAN_CHIP)$(WLAN_CHIP_REVISION)-mfgtest.bin
endif
endif

#==============================================================================
# Network stack-specific inclusion
#==============================================================================
ifeq ($(NETWORK),NetX)
#$(NAME)_SOURCES += NetX/netdb.c

ifdef CONSOLE_ENABLE_WPS
GLOBAL_DEFINES  += ADD_NETX_EAPOL_SUPPORT
endif
endif

ifeq ($(NETWORK),NetX_Duo)
#$(NAME)_SOURCES += NetX_Duo/netdb.c

ifdef CONSOLE_ENABLE_WPS
GLOBAL_DEFINES  += ADD_NETX_EAPOL_SUPPORT
endif
endif

GLOBAL_DEFINES += CONSOLE_ENABLE_THREADS

#==============================================================================
# p2p inclusion
#==============================================================================

ifneq ($(CONSOLE_NO_P2P),1)
$(NAME)_COMPONENTS           += utilities/command_console/p2p
WICED_USE_WIFI_P2P_INTERFACE := 1
$(NAME)_DEFINES              += CONSOLE_INCLUDE_P2P
GLOBAL_DEFINES               += WICED_DCT_INCLUDE_P2P_CONFIG
endif
#==============================================================================
# iperf inclusion
#==============================================================================
#4390 does not have enough memory for iperf
ifneq (,$(filter $(PLATFORM),BCM94390WCD1 BCM94390WCD2 BCM94390WCD3))
CONSOLE_NO_IPERF :=1
endif

ifndef CONSOLE_NO_IPERF
$(NAME)_COMPONENTS += test/iperf
$(NAME)_DEFINES    += CONSOLE_ENABLE_IPERF
endif

#==============================================================================
# Fortuna inclusion
#==============================================================================
#uncomment the below to enable use of the PostgreSQL Fortuna-like pseudo-random number generator
#GLOBAL_DEFINES    += WICED_SECURE_PRNG_FORTUNA_ENABLE

#==============================================================================
# Enabling WLAN IOCTL LOG
#==============================================================================
#uncomment the below line to enable WLAN IOCTL,IOVARS & events logging
#GLOBAL_DEFINES += WWD_IOCTL_LOG_ENABLE

#==============================================================================
# Traffic generation inclusion
#==============================================================================
#$(NAME)_COMPONENTS += utilities/command_console/traffic_generation

#Optional Phyrate Logging
ifeq ($(RVR_PHYRATE_LOGGING),1)
GLOBAL_DEFINES     += RVR_PHYRATE_LOGGING
endif
