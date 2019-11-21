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

NAME := LwIP

VERSION := 2.0.3

$(NAME)_COMPONENTS += WICED/network/LwIP/WWD
ifeq (,$(APP_WWD_ONLY)$(NS_WWD_ONLY)$(RTOS_WWD_ONLY))
$(NAME)_COMPONENTS += WICED/network/LwIP/WICED
endif

ifeq ($(BUILD_TYPE),debug)
GLOBAL_DEFINES := WICED_LWIP_DEBUG
endif

VALID_RTOS_LIST:= FreeRTOS

# Ethernet driver is not yet implemented for LwIP
PLATFORM_NO_GMAC := 1

# Define some macros to allow for some network-specific checks
GLOBAL_DEFINES += NETWORK_$(NAME)=1
GLOBAL_DEFINES += $(NAME)_VERSION=$$(SLASH_QUOTE_START)v$(VERSION)$$(SLASH_QUOTE_END)

# See GCC bug 48778. Some LwIP IP address macros fall over this
ifeq ($(TOOLCHAIN_TYPE),gcc)
GLOBAL_CFLAGS += -Wno-address
endif

GLOBAL_INCLUDES := ver$(VERSION) \
                   ver$(VERSION)/src/include \
                   WICED

$(NAME)_SOURCES :=  ver$(VERSION)/src/api/api_lib.c \
                    ver$(VERSION)/src/api/api_msg.c \
                    ver$(VERSION)/src/api/err.c \
                    ver$(VERSION)/src/api/netbuf.c \
                    ver$(VERSION)/src/api/netdb.c \
                    ver$(VERSION)/src/api/netifapi.c \
                    ver$(VERSION)/src/api/sockets.c \
                    ver$(VERSION)/src/api/tcpip.c \
                    ver$(VERSION)/src/core/ipv4/dhcp.c \
                    ver$(VERSION)/src/core/dns.c \
                    ver$(VERSION)/src/core/init.c \
                    ver$(VERSION)/src/core/ip.c \
                    ver$(VERSION)/src/core/ipv4/autoip.c \
                    ver$(VERSION)/src/core/ipv4/icmp.c \
                    ver$(VERSION)/src/core/ipv4/igmp.c \
                    ver$(VERSION)/src/core/inet_chksum.c \
                    ver$(VERSION)/src/core/ipv4/ip4.c \
                    ver$(VERSION)/src/core/ipv4/ip4_addr.c \
                    ver$(VERSION)/src/core/ipv4/ip4_frag.c \
                    ver$(VERSION)/src/core/def.c \
                    ver$(VERSION)/src/core/timeouts.c \
                    ver$(VERSION)/src/core/mem.c \
                    ver$(VERSION)/src/core/memp.c \
                    ver$(VERSION)/src/core/netif.c \
                    ver$(VERSION)/src/core/pbuf.c \
                    ver$(VERSION)/src/core/raw.c \
                    ver$(VERSION)/src/apps/snmp/snmp_asn1.c \
                    ver$(VERSION)/src/apps/snmp/snmp_mib2.c \
                    ver$(VERSION)/src/apps/snmp/snmp_msg.c \
                    ver$(VERSION)/src/core/stats.c \
                    ver$(VERSION)/src/core/sys.c \
                    ver$(VERSION)/src/core/tcp.c \
                    ver$(VERSION)/src/core/tcp_in.c \
                    ver$(VERSION)/src/core/tcp_out.c \
                    ver$(VERSION)/src/core/udp.c \
                    ver$(VERSION)/src/core/ipv4/etharp.c \
                    ver$(VERSION)/src/netif/ethernet.c \
                    ver$(VERSION)/src/core/ipv6/ethip6.c \
                    ver$(VERSION)/src/core/ipv6/dhcp6.c \
                    ver$(VERSION)/src/core/ipv6/icmp6.c \
                    ver$(VERSION)/src/core/ipv6/inet6.c \
                    ver$(VERSION)/src/core/ipv6/ip6.c \
                    ver$(VERSION)/src/core/ipv6/ip6_addr.c \
                    ver$(VERSION)/src/core/ipv6/ip6_frag.c \
                    ver$(VERSION)/src/core/ipv6/mld6.c \
                    ver$(VERSION)/src/core/ipv6/nd6.c
