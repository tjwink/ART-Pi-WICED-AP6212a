/**************************************************************************/
/*                                                                        */
/*            Copyright (c) 1996-2017 by Express Logic Inc.               */ 
/*                                                                        */
/*  This software is copyrighted by and is the sole property of Express   */
/*  Logic, Inc.  All rights, title, ownership, or other interests         */
/*  in the software remain the property of Express Logic, Inc.  This      */
/*  software may only be used in accordance with the corresponding        */
/*  license agreement.  Any unauthorized use, duplication, transmission,  */
/*  distribution, or disclosure of this software is expressly forbidden.  */
/*                                                                        */
/*  This Copyright notice may not be removed or modified without prior    */
/*  written consent of Express Logic, Inc.                                */
/*                                                                        */
/*  Express Logic, Inc. reserves the right to modify this software        */
/*  without notice.                                                       */
/*                                                                        */
/*  Express Logic, Inc.                     info@expresslogic.com         */
/*  11423 West Bernardo Court               http://www.expresslogic.com   */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** NetX Component                                                        */
/**                                                                       */
/**   User Specific                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  PORT SPECIFIC C INFORMATION                            RELEASE        */
/*                                                                        */
/*    nx_user.h                                           PORTABLE C      */
/*                                                           5.10 SP2     */
/*                                                                        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Express Logic, Inc.                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains user defines for configuring NetX in specific    */
/*    ways. This file will have an effect only if the application and     */
/*    NetX library are built with NX_INCLUDE_USER_DEFINE_FILE defined.    */
/*    Note that all the defines in this file may also be made on the      */
/*    command line when building NetX library and application objects.    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  12-12-2005     William E. Lamie         Initial Version 5.0           */
/*  08-09-2007     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.1    */
/*  12-30-2007     Yuxin Zhou               Modified comment(s), and      */
/*                                            added support for IPv6,     */
/*                                            resulting in version 5.2    */
/*  08-03-2009     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.3    */
/*  11-23-2009     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.4    */
/*  06-01-2010     Janet Christiansen       Addeded support for path MTU  */
/*                                            packet fragmentation logic, */
/*                                            added options for router    */
/*                                            solicitation, disabling RA, */
/*                                            ICMPv6 error message, and   */
/*                                            redirect, NetX Duo error    */
/*                                            checking and new features   */
/*                                            using stale cache entries,  */
/*                                            IPsec, static IP routing    */
/*                                            (IPv4), path MTU discovery, */
/*                                            resulting in version 5.5    */
/*  10-10-2011     Yuxin Zhou               Modified comment(s), added    */
/*                                            TCP window scaling and      */
/*                                            disable loopback interface  */
/*                                            options, added receive and  */
/*                                            transmit UDP packet checksum*/
/*                                            disable options, added      */
/*                                            options to disable ICMP     */
/*                                            checksum computation,       */
/*                                            resulting in version 5.6    */
/*  01-31-2013     Yuxin Zhou               Modified comment(s), added    */
/*                                            symbols to support IP raw   */
/*                                            recv queue max, raw filter, */
/*                                            IPv6 address change notify, */
/*                                            IPv6 autoconfig control,    */
/*                                            added new option            */
/*                                            NX_ENABLE_TCP_MSS_CHECK     */
/*                                            option to check connection  */
/*                                            requests for a minimum MSS  */
/*                                            value, added support for    */
/*                                            IPv6 join/leave operations, */
/*                                            added support for queue     */
/*                                            depth notify callback,      */
/*                                            resulting in version 5.7    */
/*  01-12-2015     Yuxin Zhou               Modified comment(s), added    */ 
/*                                            definition for IP reassembly*/
/*                                            time, added symbol that     */
/*                                            allows IP fragment          */
/*                                            reassemble immediately,     */
/*                                            updated time of ND cache to */
/*                                            RFC recommended values,     */
/*                                            added duo packet pool       */
/*                                            feature, added source       */
/*                                            address check for incoming  */
/*                                            packet, added a feature that*/
/*                                            responds to a conflict GARP */
/*                                            packet, supported to limit  */ 
/*                                            the number of packets that  */
/*                                            are out of order,           */
/*                                            supported NAT feature,      */
/*                                            removed deprecated macro    */
/*                                            for IPv6 path MTU,          */
/*                                            resulting in version 5.8    */
/*  02-22-2016     Yuxin Zhou               Modified comment(s), and      */
/*                                            supported packet alignment, */
/*                                            supported packet debugging, */
/*                                            removed deprecated or unused*/
/*                                            macros, added support to    */
/*                                            detect man-in-the-middle    */
/*                                            attack (NX_ENABLE_ARP_MAC_  */
/*                                            CHANGE_NOTIFICATION),       */
/*                                            corrected the reassembly    */
/*                                            timer, unified ticks per    */
/*                                            second, added ASSERT macros,*/
/*                                            cleaned up the macro list,  */
/*                                            supported packet pool low   */
/*                                            watermark, updated the value*/
/*                                            for maximum TCP receive     */
/*                                            queue length, renamed       */
/*                                            symbols, resulting in       */
/*                                            version 5.9                 */
/*  05-10-2016     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.10   */
/*  04-03-2017     Yuxin Zhou               Modified comment(s), and      */
/*                                            added symbol for checking   */
/*                                            address of ICMP packet,     */
/*                                            resulting in version 5.10SP2*/
/*                                                                        */
/**************************************************************************/

#ifndef NX_USER_H
#define NX_USER_H

#define WICED_CUSTOM_NX_USER_H

#define NX_DISABLE_INCLUDE_SOURCE_CODE

/* Define various build options for the NetX Duo port.  The application should either make changes
   here by commenting or un-commenting the conditional compilation defined OR supply the defines
   though the compiler's equivalent of the -D option.  */


/* Override various options with default values already assigned in nx_api.h or nx_port.h. Please
   also refer to nx_port.h for descriptions on each of these options.  */


/* Configuration options for Interface */

/* NX_MAX_PHYSICAL_INTERFACES defines the number physical network interfaces
   present to NetX Duo IP layer.  Physical interface does not include
   loopback interface. By default there is at least one physical interface
   in the system. */
#define NX_MAX_PHYSICAL_INTERFACES    (2)

/* Defined, this option disables NetX Duo support on the 127.0.0.1 loopback interface.
   127.0.0.1 loopback interface is enabled by default.  Uncomment out the follow code to disable
   the loopback interface. */
/*
#define NX_DISABLE_LOOPBACK_INTERFACE
*/

/* If defined, the link driver is able to specify extra capability, such as checksum offloading features. */
/*
#define NX_ENABLE_INTERFACE_CAPABILITY
*/


/* Configuration options for IP */

/* This defines specifies the number of ThreadX timer ticks in one second. The default value is based
   on ThreadX timer interrupt.  */
/*
#ifdef TX_TIMER_TICKS_PER_SECOND
#define NX_IP_PERIODIC_RATE         TX_TIMER_TICKS_PER_SECOND
#else
#define NX_IP_PERIODIC_RATE         100
#endif
*/
extern unsigned long host_rtos_get_tickrate( void );
#define NX_IP_PERIODIC_RATE         ( host_rtos_get_tickrate( ) )


/* Defined, NX_ENABLE_IP_RAW_PACKET_FILTER allows an application to install a filter
   for incoming raw packets. This feature is disabled by default. */
/*
#define NX_ENABLE_IP_RAW_PACKET_FILTER
*/

/* This define specifies the maximum number of RAW packets can be queued for receive.  The default
   value is 20.  */
/*
#define NX_IP_RAW_MAX_QUEUE_DEPTH 20
*/

/* Defined, this option enables IP static routing feature.  By default IP static routing
   feature is not compiled in. */
/*
#define NX_ENABLE_IP_STATIC_ROUTING
*/

/* This define specifies the size of IP routing table. The default value is 8. */
/* 
#define NX_IP_ROUTING_TABLE_SIZE 8 
*/

/* This define specifies the maximum number of multicast groups that can be joined. 
   The default value is 7.  */
/*
#define NX_MAX_MULTICAST_GROUPS     7
*/


/* Configuration options for IPv6 */

/* Disable IPv6 processing in NetX Duo.  */
/*
#define NX_DISABLE_IPV6
*/

/* Define the number of entries in IPv6 address pool. */
/*
#ifdef NX_MAX_PHYSICAL_INTERFACES
#define NX_MAX_IPV6_ADDRESSES (NX_MAX_PHYSICAL_INTERFACES * 3)
#endif 
*/

/* Do not process IPv6 ICMP Redirect Messages. */
/*
#define NX_DISABLE_ICMPV6_REDIRECT_PROCESS
*/

/* Do not process IPv6 Router Advertisement Messages. */
/*
#define NX_DISABLE_ICMPV6_ROUTER_ADVERTISEMENT_PROCESS
*/

/* Do not send IPv6 Router Solicitation Messages. */
/*
#define NX_DISABLE_ICMPV6_ROUTER_SOLICITATION
*/

/* Define the max number of router solicitations a host sends until a router response
   is received.  If no response is received, the host concludes no router is present. */
/*
#define NX_ICMPV6_MAX_RTR_SOLICITATIONS         3
*/

/* Define the interval between which the host sends router solicitations in seconds. */
/*
#define NX_ICMPV6_RTR_SOLICITATION_INTERVAL     4
*/

/* Do not send ICMPv6 Error Message. */
/*
#define NX_DISABLE_ICMPV6_ERROR_MESSAGE
*/

/* Disable the Duplicate Address Detection (DAD) protocol when configuring the host IP address. */
/*
#define NX_DISABLE_IPV6_DAD
*/

/* If defined, application is able to control whether or not to perform IPv6 stateless
   address autoconfiguration with nxd_ipv6_stateless_address_autoconfig_enable() or
   nxd_ipv6_stateless_address_autoconfig_disable() service.  If defined, the system starts
   with IPv6 stateless address autoconfiguration enabled.  This feature is disabled by default. */
/*
#define NX_IPV6_STATELESS_AUTOCONFIG_CONTROL
*/

/* If enabled, application is able to install a callback function to get notified
   when an interface IPv6 address is changed. By default this feature is disabled. */
#define NX_ENABLE_IPV6_ADDRESS_CHANGE_NOTIFY

/* Defined, this option prevents NetX Duo from removing stale (old) cache table entries
   whose timeout has not expired so are otherwise still valid) to make room for new entries
   when the table is full.  Static and router entries are not purged.  */
/*
#define NX_DISABLE_IPV6_PURGE_UNUSED_CACHE_ENTRIES
*/

/* This define enables simple IPv6 multicast group join/leave function.  By default
   the IPv6 multicast join/leave function is not enabled. */
#define NX_ENABLE_IPV6_MULTICAST

/* Defined, Minimum Path MTU Discovery feature is enabled.  */
/*
#define NX_ENABLE_IPV6_PATH_MTU_DISCOVERY
*/

/* Define wait interval in seconds to reset the path MTU for a destination 
   table entry after decreasing it in response to a packet too big error message.
   RFC 1981 Section 5.4 states the minimum time to wait is
   5 minutes and recommends 10 minutes.
*/
/*
#define NX_PATH_MTU_INCREASE_WAIT_INTERVAL               600 
*/


/* Configuration options for Neighbor Discovery.  */
/* Define values used for Neighbor Discovery protocol.
   The default values are suggested by RFC2461, chapter 10. */

/* Define the maximum number of multicast Neighbor Solicitation packets
   NetX Duo sends for a packet destination needing physical mapping
   to the IP address. */
/*
#define NX_MAX_MULTICAST_SOLICIT        3
*/

/* Define the maximum number of unicast Neighbor Solicitation packets
   NetX Duo sends for a cache entry whose reachable time has expired
   and gone "stale". */
/*
#define NX_MAX_UNICAST_SOLICIT          3
*/

/* Define the length of time, in seconds, that a Neighbor Cache table entry
   remains in the reachable state before it becomes state. */
/*
#define NX_REACHABLE_TIME               30
*/

/* Define the length of time, in milliseconds, between retransmitting
   Neighbor Solicitation (NS) packets. */
/*
#define NX_RETRANS_TIMER                1000
*/

/* Define the length of time, in seconds, for a Neighbor Cache entry
   to remain in the Delay state.  This is the Delay first probe timer. */
/*
#define NX_DELAY_FIRST_PROBE_TIME       5
*/

/* This defines specifies the maximum number of packets that can be queued while waiting for a
   Neighbor Discovery to resolve an IPv6 address. The default value is 4.  */
/*
#define NX_ND_MAX_QUEUE_DEPTH           4
*/

/* Define the maximum ICMPv6 Duplicate Address Detect Transmit .  */
/*
#define NX_IPV6_DAD_TRANSMITS           3
*/

/* Define the number of neighbor cache entries. */
#define NX_IPV6_NEIGHBOR_CACHE_SIZE     (1)

/* Define the size of the IPv6 destination table. */
/*
#define NX_IPV6_DESTINATION_TABLE_SIZE  8
*/

/* Define the size of the IPv6 prefix table. */
/*
#define NX_IPV6_PREFIX_LIST_TABLE_SIZE  8
*/


/* Configuration options for IPSEC */

/* This define enables IPSEC in NetX Duo.  */
/*
#define NX_IPSEC_ENABLE
*/


/* Configuration options for NAT */

/* This define enables NAT process in NetX Duo.  */
/*
#define NX_NAT_ENABLE
*/


/* Configuration options for IGMP */

/* Defined, IGMP v2 support is disabled.  By default NetX Duo 
   is built with IGMPv2 enabled .  By uncommenting this option, 
   NetX Duo reverts back to IGMPv1 only. */
/* 
#define NX_DISABLE_IGMPV2 
*/


/* Configuration options for ARP */

/* When defines, ARP reply is sent when address conflict occurs. */
/*
#define NX_ARP_DEFEND_BY_REPLY
*/

/* To use the ARP collision hander to check for invalid ARP messages
   matching existing entries in the table (man in the middle attack), 
   enable this feature.  */
/*
#define  NX_ENABLE_ARP_MAC_CHANGE_NOTIFICATION
*/

/* This define specifies the number of seconds ARP entries remain valid. The default value of 0 disables
   aging of ARP entries.  */
/*
#define NX_ARP_EXPIRATION_RATE      0
*/

/* This define specifies the number of seconds between ARP retries. The default value is 10, which represents
   10 seconds.  */
#define NX_ARP_UPDATE_RATE          (1)

/* This define specifies the maximum number of ARP retries made without an ARP response. The default
   value is 18.  */
/*
#define NX_ARP_MAXIMUM_RETRIES      18
*/

/* This defines specifies the maximum number of packets that can be queued while waiting for an ARP
   response. The default value is 4.  */
/*
#define NX_ARP_MAX_QUEUE_DEPTH      4
*/

/* Defined, this option disables entering ARP request information in the ARP cache.  */
#define NX_DISABLE_ARP_AUTO_ENTRY

/* Define the ARP defend interval. The default value is 10 seconds.  */
/*
#define NX_ARP_DEFEND_INTERVAL  10
*/


/* Configuration options for TCP */

/* This define specifies how the number of system ticks (NX_IP_PERIODIC_RATE) is divided to calculate the
   timer rate for the TCP delayed ACK processing. The default value is 5, which represents 200ms.  */
/*
#define NX_TCP_ACK_TIMER_RATE       5
*/

/* This define specifies how the number of system ticks (NX_IP_PERIODIC_RATE) is divided to calculate the
   fast TCP timer rate. The fast TCP timer is used to drive various TCP timers, including the delayed ACK
   timer. The default value is 10, which represents 100ms.  */
/*
#define NX_TCP_FAST_TIMER_RATE      10
*/

/* This define specifies how the number of system ticks (NX_IP_PERIODIC_RATE) is divided to calculate the
   timer rate for the TCP transmit retry processing. The default value is 1, which represents 1 second.  */
/*
#define NX_TCP_TRANSMIT_TIMER_RATE  1
*/

/* This define specifies how many seconds of inactivity before the keepalive timer activates. The default
   value is 7200, which represents 2 hours.   */
/*
#define NX_TCP_KEEPALIVE_INITIAL    7200
*/

/* This define specifies how many seconds between retries of the keepalive timer assuming the other side
   of the connection is not responding. The default value is 75, which represents 75 seconds between
   retries.  */
/*
#define NX_TCP_KEEPALIVE_RETRY      75
*/

/* This define specifies the maximum packets that are out of order. The default value is 8.  */
#define NX_TCP_MAX_OUT_OF_ORDER_PACKETS (8)

/* This define specifies the maximum number of TCP server listen requests. The default value is 10.  */
#define NX_MAX_LISTEN_REQUESTS      (18)

/* Defined, this option enables the optional TCP keepalive timer.  */
#define NX_ENABLE_TCP_KEEPALIVE

/* Defined, this option enables the optional TCP immediate ACK response processing.  */
/*
#define NX_TCP_IMMEDIATE_ACK
*/

/* This define specifies the number of TCP packets to receive before sending an ACK. */
/* The default value is 2: ack every 2 packets.                                      */
#define NX_TCP_ACK_EVERY_N_PACKETS  (2)

/* Automatically define NX_TCP_ACK_EVERY_N_PACKETS to 1 if NX_TCP_IMMEDIATE_ACK is defined.
   This is needed for backward compatibility. */
#if (defined(NX_TCP_IMMEDIATE_ACK) && !defined(NX_TCP_ACK_EVERY_N_PACKETS))
#define NX_TCP_ACK_EVERY_N_PACKETS 1
#endif

/* This define specifies how many transmit retires are allowed before the connection is deemed broken.
   The default value is 10.  */
/*
#define NX_TCP_MAXIMUM_RETRIES      10
*/

/* This define specifies the maximum depth of the TCP transmit queue before TCP send requests are
   suspended or rejected. The default value is 20, which means that a maximum of 20 packets can be in
   the transmit queue at any given time.  */
/*
#define NX_TCP_MAXIMUM_TX_QUEUE     20
*/

/* This define specifies how the retransmit timeout period changes between successive retries. If this
   value is 0, the initial retransmit timeout is the same as subsequent retransmit timeouts. If this
   value is 1, each successive retransmit is twice as long. The default value is 0.  */
/*
#define NX_TCP_RETRY_SHIFT          0
*/

/* This define specifies how many keepalive retries are allowed before the connection is deemed broken.
   The default value is 10.  */
/*
#define NX_TCP_KEEPALIVE_RETRIES    10
*/

/* Defined, this option enables the TCP window scaling feature. (RFC 1323). Default disabled. */
#define NX_ENABLE_TCP_WINDOW_SCALING

/* Defined, this option disables the reset processing during disconnect when the timeout value is
   specified as NX_NO_WAIT.  */
/*
#define NX_DISABLE_RESET_DISCONNECT
*/

/* If defined, the incoming SYN packet (connection request) is checked for a minimum acceptable
   MSS for the host to accept the connection. The default minimum should be based on the host
   application packet pool payload, socket transmit queue depth and relevant application specific parameters. */
/*
#define NX_ENABLE_TCP_MSS_CHECK
#define NX_TCP_MSS_MINIMUM              128
*/

/* If defined, NetX Duo has a notify callback for the transmit TCP socket queue decreased from
   the maximum queue depth.  */
/*
#define NX_ENABLE_TCP_QUEUE_DEPTH_UPDATE_NOTIFY
*/

/* Defined, feature of low watermark is enabled. */
/*
#define NX_ENABLE_LOW_WATERMARK
*/

/* Define the maximum receive queue for TCP socket. */
/*
#ifdef NX_ENABLE_LOW_WATERMARK
#define NX_TCP_MAXIMUM_RX_QUEUE    20
#endif 
*/

/* Configuration options for fragmentation */

/* Defined, this option disables both IPv4 and IPv6 fragmentation and reassembly logic.  */
/*
#define NX_DISABLE_FRAGMENTATION
*/

/* Defined, this option process IP fragmentation immediately.  */
/*
#define NX_FRAGMENT_IMMEDIATE_ASSEMBLY
*/
                  
/* This define specifies the maximum time of IP reassembly.  The default value is 60.  
   By default this option is not defined.  */
#define NX_IP_MAX_REASSEMBLY_TIME   (1)

/* This define specifies the maximum time of IPv4 reassembly.  The default value is 15.  
   Note that if NX_IP_MAX_REASSEMBLY_TIME is defined, this option is automatically defined as 60.
   By default this option is not defined.  */
/*
#define NX_IPV4_MAX_REASSEMBLY_TIME 15
*/
     
/* This define specifies the maximum time of IPv6 reassembly.  The default value is 60.   
   Note that if NX_IP_MAX_REASSEMBLY_TIME is defined, this option is automatically defined as 60. 
   By default this option is not defined.  */
/*
#define NX_IPV6_MAX_REASSEMBLY_TIME 60
*/

/* Configuration options for checksum */

/* Defiend, this option disables checksum logic on received ICMPv4 packets.
   Note that if NX_DISABLE_ICMP_RX_CHECKSUM is defined, this option is
   automatically defined. By default this option is not defined.*/
/*
#define NX_DISABLE_ICMPV4_RX_CHECKSUM
*/

/* Defiend, this option disables checksum logic on received ICMPv6 packets.
   Note that if NX_DISABLE_ICMP_RX_CHECKSUM is defined, this option is
   automatically defined. By default this option is not defined.*/
/*
#define NX_DISABLE_ICMPV6_RX_CHECKSUM
*/

/* Defined, this option disables checksum logic on received ICMPv4 or ICMPv6 packets.
   Note that if NX_DISABLE_ICMP_RX_CHECKSUM is defined, NX_DISABLE_ICMPV4_RX_CHECKSUM
   and NX_DISABLE_ICMPV6_RX_CHECKSUM are automatically defined. */
/*
#define NX_DISABLE_ICMP_RX_CHECKSUM
*/

/* Defiend, this option disables checksum logic on transmitted ICMPv4 packets.
   Note that if NX_DISABLE_ICMP_TX_CHECKSUM is defined, this option is
   automatically defined. By default this option is not defined.*/
/*
#define NX_DISABLE_ICMPV4_TX_CHECKSUM
*/

/* Defiend, this option disables checksum logic on transmitted ICMPv6 packets.
   Note that if NX_DISABLE_ICMP_TX_CHECKSUM is defined, this option is
   automatically defined. By default this option is not defined.*/
/*
#define NX_DISABLE_ICMPV6_TX_CHECKSUM
*/

/* Defined, this option disables checksum logic on transmitted ICMPv4 or ICMPv6 packets.
   Note that if NX_DISABLE_ICMP_TX_CHECKSUM is defined, NX_DISABLE_ICMPV4_TX_CHECKSUM
   and NX_DISABLE_ICMPV6_TX_CHECKSUM are automatically defined. */
/*
#define NX_DISABLE_ICMP_TX_CHECKSUM
*/

/* Defined, this option disables checksum logic on received IP packets. This is useful if the link-layer
   has reliable checksum or CRC logic.  */
/*
#define NX_DISABLE_IP_RX_CHECKSUM
*/

/* Defined, this option disables checksum logic on transmitted IP packets.  */
/*
#define NX_DISABLE_IP_TX_CHECKSUM
*/

/* Defined, this option disables checksum logic on received TCP packets.  */
/*
#define NX_DISABLE_TCP_RX_CHECKSUM
*/

/* Defined, this option disables checksum logic on transmitted TCP packets.  */
/*
#define NX_DISABLE_TCP_TX_CHECKSUM
*/

/* Defined, this option disables checksum logic on received UDP packets.  */

/*
#define NX_DISABLE_UDP_RX_CHECKSUM
*/

/* Defined, this option disables checksum logic on transmitted UDP packets.  Note that
   IPV6 requires the UDP checksum computed for outgoing packets.  If this option is
   defined, the IPv6 NetX Duo host must ensure the UDP checksum is computed elsewhere
   before the packet is transmitted. */
/*
#define NX_DISABLE_UDP_TX_CHECKSUM
*/


/* Configuration options for statistics.  */

/* Defined, ARP information gathering is disabled.  */
#define NX_DISABLE_ARP_INFO

/* Defined, IP information gathering is disabled.  */
#define NX_DISABLE_IP_INFO

/* Defined, ICMP information gathering is disabled.  */
#define NX_DISABLE_ICMP_INFO

/* Defined, IGMP information gathering is disabled.  */
#define NX_DISABLE_IGMP_INFO

/* Defined, packet information gathering is disabled.  */
#define NX_DISABLE_PACKET_INFO

/* Defined, RARP information gathering is disabled.  */
#define NX_DISABLE_RARP_INFO

/* Defined, TCP information gathering is disabled.  */
#define NX_DISABLE_TCP_INFO

/* Defined, UDP information gathering is disabled.  */
/*
#define NX_DISABLE_UDP_INFO
*/


/* Configuration options for Packet Pool */

/* This define specifies the size of the physical packet header. The default value is 16 (based on
   a typical 16-byte Ethernet header).  */
#define NX_PHYSICAL_HEADER          (14 + 12 + 18)

/* This define specifies the size of the physical packet trailer and is typically used to reserve storage
   for things like Ethernet CRCs, etc.  */
#define NX_PHYSICAL_TRAILER         (0)

/* Defined, this option disables the addition size checking on received packets.  */
/*
#define NX_DISABLE_RX_SIZE_CHECKING
*/

/* Defined, packet debug infromation is enabled.  */
/*
#define NX_ENABLE_PACKET_DEBUG_INFO
*/

/* Defined, NX_PACKET structure is padded for alignment purpose. The default is no padding. */
/*
#define NX_PACKET_HEADER_PAD
#define NX_PACKET_HEADER_PAD_SIZE   1
*/

/* Defined, packet header and payload are aligned automatically by the value. The default value is sizeof(ULONG). */
/*
#define NX_PACKET_ALIGNMENT sizeof(ULONG)
*/

/* If defined, the packet chain feature is removed. */
/*
#define NX_DISABLE_PACKET_CHAIN
*/

/* Defined, the IP instance manages two packet pools. */
/*
#define NX_ENABLE_DUAL_PACKET_POOL
*/

/* Configuration options for Others */

/* Defined, this option bypasses the basic NetX error checking. This define is typically used
   after the application is fully debugged.  */
/*
#define NX_DISABLE_ERROR_CHECKING
*/

/* Defined, this option enables deferred driver packet handling. This allows the driver to place a raw
   packet on the IP instance and have the driver's real processing routine called from the NetX internal
   IP helper thread.  */
/*
#define NX_DRIVER_DEFERRED_PROCESSING
*/

/* Defined, the source address of incoming packet is checked. The default is disabled. */
/*
#define NX_ENABLE_SOURCE_ADDRESS_CHECK
*/

/* Defined, the extended notify support is enabled.  This feature adds additional callback/notify services
   to NetX Duo API for notifying the application of socket events, such as TCP connection and disconnect
   completion.  These extended notify functions are mainly used by the BSD wrapper. The default is this
   feature is disabled.  */
#define NX_ENABLE_EXTENDED_NOTIFY_SUPPORT

/* Defined, ASSERT is disabled. The default is enabled. */
/*
#define NX_DISABLE_ASSERT
*/

/* Define the process when assert fails. */
/*
#define NX_ASSERT_FAIL while (1) tx_thread_sleep(NX_WAIT_FOREVER);
*/

/* Defined, the destination address of ICMP packet is checked. The default is disabled.
   An ICMP Echo Request destined to an IP broadcast or IP multicast address will be silently discarded.
*/
/*
#define NX_ENABLE_ICMP_ADDRESS_CHECK
*/

#define NX_DHCP_PACKET_PAYLOAD      (NX_BOOT_BUFFER_SIZE + NX_PHYSICAL_HEADER + 20 + 8 + 4 + 100)
#define NX_DHCP_THREAD_PRIORITY     (4)
#define NX_DNS_MAX_RETRIES          (1)
#define NX_FTP_NO_FILEX
#define NX_HTTP_NO_FILEX
#define NX_TFTP_NO_FILEX
#define NX_DHCP_THREAD_STACK_SIZE   (1152)
#define NX_DHCP_CLIENT_USER_CREATE_PACKET_POOL
#define NX_DHCP_CLIENT_RESTORE_STATE


#define PACKET_RELEASE_NOTIFY

extern void packet_release_notify( void* pool );

#define NX_RAND                         nx_rand16
extern UINT nx_rand16( void );

/* Disable parameter checking when building non-debug */
#ifndef DEBUG
#define NX_DISABLE_ERROR_CHECKING
#else
#undef NX_DISABLE_ERROR_CHECKING
#endif


/* These are all the defines that are used in NetX-Duo #if statements
 * This prevents users changing NetX values to become incompatible with
 * the prebuilt libraries.
 */

#undef OK
#undef SO_PASSCRED
#undef FD_SETSIZE
#undef ERROR
#undef fd_set
#undef FIONBIO
#undef FIONREAD
#undef htonl
#undef htons
#undef ntohl
#undef ntohs
#undef NX_DHCP_DEFAULT_LEASE_TIME
#undef NX_DHCPV6_DEFAULT_PREFERRED_TIME
#undef NX_DHCPV6_DEFAULT_T1_TIME
#undef NX_DHCPV6_DEFAULT_T2_TIME
#undef NX_DHCPV6_DEFAULT_VALID_TIME
#undef NX_EXTENDED_BSD_LINGER_AND_TIMED_WAIT
#undef NX_HTTP_DIGEST_ENABLE
#undef NX_TELNET_SERVER_PORT
#undef NX_DISABLE_LOOPBACK_INTERFACE
#undef NX_ENABLE_INTERFACE_CAPABILITY
#undef NX_TCP_IMMEDIATE_ACK
#undef NX_MAX_IPV6_ADDRESSES
#undef FEATURE_NX_IPV6
#undef NX_ARP_DEFEND_BY_REPLY
#undef NX_ENABLE_ARP_MAC_CHANGE_NOTIFICATION
#undef NX_ARP_EXPIRATION_RATE
#undef NX_ARP_MAX_QUEUE_DEPTH
#undef NX_ARP_MAXIMUM_RETRIES
#undef NX_ARP_DEFEND_INTERVAL
#undef NX_AUTO_IP_ANNOUNCE_INTERVAL
#undef NX_AUTO_IP_ANNOUNCE_NUM
#undef NX_AUTO_IP_ANNOUNCE_WAIT
#undef NX_AUTO_IP_DEFEND_INTERVAL
#undef NX_AUTO_IP_MAX_CONFLICTS
#undef NX_AUTO_IP_PROBE_MAX
#undef NX_AUTO_IP_PROBE_MIN
#undef NX_AUTO_IP_PROBE_NUM
#undef NX_AUTO_IP_PROBE_WAIT
#undef NX_AUTO_IP_RATE_LIMIT_INTERVAL
#undef NX_BSD_MAX_LISTEN_BACKLOG
#undef NX_BSD_MAX_SOCKETS
#undef NX_BSD_PRINT_ERRORS
#undef NX_BSD_SOCKFD_START
#undef NX_BSD_TCP_WINDOW
#undef NX_BSD_TIMED_WAIT_TIMEOUT
#undef NX_BSD_TIMEOUT
#undef NX_DELAY_FIRST_PROBE_TIME
#undef NX_DESTINATION_TABLE_SIZE
#undef NX_DHCP_CLIENT_HOSTNAME_MAX
#undef NX_DHCP_CLIENT_IDENTIFIER_MAX
#undef NX_DHCP_CLIENT_OPTIONS_MAX
#undef NX_DHCP_CLIENT_RECORD_TABLE_SIZE
#undef NX_DHCP_CLIENT_SESSION_TIMEOUT
#undef NX_DHCP_ENABLE_BOOTP
#undef NX_DHCP_FAST_PERIODIC_TIME_INTERVAL
#undef NX_DHCP_FRAGMENT_OPTION
#undef NX_DHCP_IP_ADDRESS_MAX_LIST_SIZE
#undef NX_DHCP_MAX_RETRANS_TIMEOUT
#undef NX_DHCP_MIN_RENEW_TIMEOUT
#undef NX_DHCP_MIN_RETRANS_TIMEOUT
#undef NX_DHCP_MINIMUM_PACKET_PAYLOAD
#undef NX_DHCP_OPTIONAL_SERVER_OPTION_LIST
#undef NX_DHCP_OPTIONAL_SERVER_OPTION_SIZE
#undef NX_DHCP_PACKET_ALLOCATE_TIMEOUT
#undef NX_DHCP_PACKET_POOL_SIZE
#undef NX_DHCP_QUEUE_DEPTH
#undef NX_DHCP_SERVER_HOSTNAME_MAX
#undef NX_DHCP_SERVER_NAME
#undef NX_DHCP_SERVER_THREAD_PRIORITY
#undef NX_DHCP_SERVER_THREAD_STACK_SIZE
#undef NX_DHCP_SLOW_PERIODIC_TIME_INTERVAL
#undef NX_DHCP_SUBNET_MASK
#undef NX_DHCP_TIME_INTERVAL
#undef NX_DHCP_TIME_TO_LIVE
#undef NX_DHCP_TYPE_OF_SERVICE
#undef NX_DHCPV6_DOMAIN_NAME_BUFFER_SIZE
#undef NX_DHCPV6_FRAGMENT_OPTION
#undef NX_DHCPV6_IP_LEASE_TIMER_INTERVAL
#undef NX_DHCPV6_IP_LIFETIME_TIMER_INTERVAL
#undef NX_DHCPV6_MAX_CLIENTS
#undef NX_DHCPV6_MAX_LEASES
#undef NX_DHCPV6_MAX_MESSAGE_SIZE
#undef NX_DHCPV6_MAX_OPTION_REQUEST_OPTIONS
#undef NX_DHCPV6_MUTEX_WAIT
#undef NX_DHCPV6_NUM_DNS_SERVERS
#undef NX_DHCPV6_NUM_TIME_SERVERS
#undef NX_DHCPV6_PACKET_POOL_SIZE
#undef NX_DHCPV6_PACKET_SIZE
#undef NX_DHCPV6_PACKET_TIME_OUT
#undef NX_DHCPV6_PACKET_WAIT_OPTION
#undef NX_DHCPV6_PREFERENCE_VALUE
#undef NX_DHCPV6_QUEUE_DEPTH
#undef NX_DHCPV6_SERVER_DUID_TYPE
#undef NX_DHCPV6_SERVER_DUID_VENDOR_ASSIGNED_ID
#undef NX_DHCPV6_SERVER_DUID_VENDOR_ASSIGNED_LENGTH
#undef NX_DHCPV6_SERVER_DUID_VENDOR_PRIVATE_ID
#undef NX_DHCPV6_SERVER_HW_TYPE
#undef NX_DHCPV6_SERVER_THREAD_PRIORITY
#undef NX_DHCPV6_SERVER_THREAD_STACK_SIZE
#undef NX_DHCPV6_SESSION_TIMEOUT
#undef NX_DHCPV6_SESSION_TIMER_INTERVAL
#undef NX_DHCPV6_STATUS_MESSAGE_MAX
#undef NX_DHCPV6_STATUS_MESSAGE_NO_BINDING
#undef NX_DHCPV6_STATUS_MESSAGE_NOT_ON_LINK
#undef NX_DHCPV6_STATUS_MESSAGE_SUCCESS
#undef NX_DHCPV6_STATUS_MESSAGE_UNSPECIFIED
#undef NX_DHCPV6_STATUS_MESSAGE_USE_MULTICAST
#undef NX_DHCPV6_STATUS_MESSAGE_NO_ADDRS_AVAILABLE
#undef NX_DHCPV6_THREAD_PRIORITY
#undef NX_DHCPV6_TICKS_PER_SECOND
#undef NX_DHCPV6_TIME_TO_LIVE
#undef NX_DHCPV6_TIME_ZONE_BUFFER_SIZE
#undef NX_DHCPV6_TYPE_OF_SERVICE
#undef NX_DISABLE_EXTENDED_NOTIFY_SUPPORT
#undef NX_DISABLE_IGMPV2
#undef NX_DISABLE_IPV6
#undef NX_DNS_CLIENT_USER_CREATE_PACKET_POOL
#undef NX_DNS_FRAGMENT_OPTION
#undef NX_DNS_MAX_SERVERS
#undef NX_DNS_MESSAGE_MAX
#undef NX_DNS_PACKET_ALLOCATE_TIMEOUT
#undef NX_DNS_PACKET_PAYLOAD_UNALIGNED
#undef NX_DNS_PACKET_POOL_SIZE
#undef NX_DNS_QUEUE_DEPTH
#undef NX_DNS_TIME_TO_LIVE
#undef NX_DNS_TYPE_OF_SERVICE
#undef NX_IPV6_DAD_TRANSMITS
#undef NX_ENABLE_IP_STATIC_ROUTING
#undef NX_ENABLE_IPV6_PATH_MTU_DISCOVERY
#undef NX_FTP_ACTIVITY_TIMEOUT
#undef NX_FTP_CLIENT_SOURCE_PORT
#undef NX_FTP_CONTROL_TOS
#undef NX_FTP_CONTROL_WINDOW_SIZE
#undef NX_FTP_DATA_TOS
#undef NX_FTP_DATA_WINDOW_SIZE
#undef NX_FTP_FRAGMENT_OPTION
#undef NX_FTP_MAX_CLIENTS
#undef NX_FTP_PASSWORD_SIZE
#undef NX_FTP_SERVER_PRIORITY
#undef NX_FTP_SERVER_RETRY_MAX
#undef NX_FTP_SERVER_RETRY_SECONDS
#undef NX_FTP_SERVER_RETRY_SHIFT
#undef NX_FTP_SERVER_TIME_SLICE
#undef NX_FTP_SERVER_TIMEOUT
#undef NX_FTP_SERVER_TRANSMIT_QUEUE_DEPTH
#undef NX_FTP_TIME_TO_LIVE
#undef NX_FTP_TIMEOUT_PERIOD
#undef NX_FTP_USERNAME_SIZE
#undef NX_HTTP_CLIENT_MIN_PACKET_SIZE
#undef NX_HTTP_CLIENT_TIMEOUT
#undef NX_HTTP_FRAGMENT_OPTION
#undef NX_HTTP_MAX_NAME
#undef NX_HTTP_MAX_PASSWORD
#undef NX_HTTP_MAX_RESOURCE
#undef NX_HTTP_SERVER_MAX_PENDING
#undef NX_HTTP_SERVER_MIN_PACKET_SIZE
#undef NX_HTTP_SERVER_PRIORITY
#undef NX_HTTP_SERVER_RETRY_MAX
#undef NX_HTTP_SERVER_RETRY_SECONDS
#undef NX_HTTP_SERVER_RETRY_SHIFT
#undef NX_HTTP_SERVER_THREAD_TIME_SLICE
#undef NX_HTTP_SERVER_TIMEOUT_ACCEPT
#undef NX_HTTP_SERVER_TIMEOUT_RECEIVE
#undef NX_HTTP_SERVER_TIMEOUT_SEND
#undef NX_HTTP_SERVER_TIMEOUT_DISCONNECT
#undef NX_HTTP_SERVER_TIMEOUT
#undef NX_HTTP_SERVER_TRANSMIT_QUEUE_DEPTH
#undef NX_HTTP_SERVER_WINDOW_SIZE
#undef NX_HTTP_TIME_TO_LIVE
#undef NX_HTTP_TYPE_OF_SERVICE
#undef NX_IP_FAST_TIMER_RATE
#undef NX_IP_RAW_MAX_QUEUE_DEPTH
#undef NX_IP_ROUTING_TABLE_SIZE
#undef NX_IP_STATUS_CHECK_WAIT_TIME
#undef NX_IPSEC_ENABLE
#undef NX_NAT_ENABLE
#undef NX_IPV6_DEFAULT_ROUTER_TABLE_SIZE
#undef NX_IPV6_HOST_ID_LENGTH
#undef NX_IPV6_PREFIX_LIST_TABLE_SIZE
#undef NX_IPV6_STATELESS_AUTOCONFIG_CONTROL
#undef NX_DISABLE_IPV6_PURGE_UNUSED_CACHE_ENTRIES
#undef NX_IPV6_UTIL_INLINE
#undef NX_MAX_MULTICAST_GROUPS
#undef NX_MAX_MULTICAST_SOLICIT
#undef NX_MAX_UNICAST_SOLICIT
#undef NX_MICROSECOND_PER_CPU_TICK
#undef NX_ND_MAX_QUEUE_DEPTH
#undef NX_PACKET_HEADER_PAD
#undef NX_PACKET_HEADER_PAD_SIZE
#undef NX_PACKET_OFFSET_ERROR
#undef NX_PATH_MTU_INCREASE_WAIT_INTERVAL
#undef NX_ENABLE_IP_RAW_PACKET_FILTER
#undef NX_REACHABLE_TIME
#undef NX_RETRANS_TIMER
#undef NX_SNMP_AGENT_PORT
#undef NX_SNMP_AGENT_PRIORITY
#undef NX_SNMP_AGENT_TIMEOUT
#undef NX_SNMP_DISABLE_V3
#undef NX_SNMP_FRAGMENT_OPTION
#undef NX_SNMP_MANAGER_TRAP_PORT
#undef NX_SNMP_MAX_CONTEXT_STRING
#undef NX_SNMP_MAX_OCTET_STRING
#undef NX_SNMP_MAX_SECURITY_KEY
#undef NX_SNMP_MAX_USER_NAME
#undef NX_SNMP_NO_SECURITY
#undef NX_SNMP_PACKET_SIZE
#undef NX_SNMP_TIME_TO_LIVE
#undef NX_SNMP_TYPE_OF_SERVICE
#undef NX_SNTP_CLIENT_EXP_BACKOFF_RATE
#undef NX_SNTP_CLIENT_IGNORE_MAX_ADJUST_STARTUP
#undef NX_SNTP_CLIENT_INVALID_UPDATE_LIMIT
#undef NX_SNTP_CLIENT_MAX_QUEUE_DEPTH
#undef NX_SNTP_CLIENT_MAX_ROOT_DISPERSION
#undef NX_SNTP_CLIENT_MAX_TIME_ADJUSTMENT
#undef NX_SNTP_CLIENT_MAX_TIME_LAPSE
#undef NX_SNTP_CLIENT_MIN_NTP_VERSION
#undef NX_SNTP_CLIENT_MIN_SERVER_STRATUM
#undef NX_SNTP_CLIENT_MIN_TIME_ADJUSTMENT
#undef NX_SNTP_CLIENT_NTP_VERSION
#undef NX_SNTP_CLIENT_PACKET_TIMEOUT
#undef NX_SNTP_CLIENT_PREEMPTION_THRESHOLD
#undef NX_SNTP_CLIENT_RANDOMIZE_ON_STARTUP
#undef NX_SNTP_CLIENT_RTT_REQUIRED
#undef NX_SNTP_CLIENT_SLEEP_INTERVAL
#undef NX_SNTP_CLIENT_THREAD_PRIORITY
#undef NX_SNTP_CLIENT_THREAD_STACK_SIZE
#undef NX_SNTP_CLIENT_THREAD_TIME_SLICE
#undef NX_SNTP_CLIENT_TIME_TO_LIVE
#undef NX_SNTP_CLIENT_UDP_PORT
#undef NX_SNTP_CLIENT_UDP_SOCKET_NAME
#undef NX_SNTP_CLIENT_UNICAST_POLL_INTERVAL
#undef NX_SNTP_DISABLE_ERROR_CHECKING
#undef NX_SNTP_SERVER_UDP_PORT
#undef NX_SNTP_UPDATE_TIMEOUT_INTERVAL
#undef NX_DISABLE_RESET_DISCONNECT
#undef NX_TCP_ACK_TIMER_RATE
#undef NX_TCP_FAST_TIMER_RATE
#undef NX_TCP_KEEPALIVE_INITIAL
#undef NX_TCP_KEEPALIVE_RETRIES
#undef NX_TCP_KEEPALIVE_RETRY
#undef NX_TCP_MAXIMUM_RETRIES
#undef NX_TCP_MAXIMUM_TX_QUEUE
#undef NX_TCP_RETRY_SHIFT
#undef NX_TCP_TRANSMIT_TIMER_RATE
#undef NX_ENABLE_TCP_MSS_CHECK
#undef NX_TCP_MSS_MINIMUM
#undef NX_ENABLE_TCP_QUEUE_DEPTH_UPDATE_NOTIFY
#undef NX_ENABLE_LOW_WATERMARK
#undef NX_TCP_MAXIMUM_RX_QUEUE
#undef NX_TELNET_ACTIVITY_TIMEOUT
#undef NX_TELNET_FRAGMENT_OPTION
#undef NX_TELNET_MAX_CLIENTS
#undef NX_TELNET_MAX_OPTION_SIZE
#undef NX_TELNET_SERVER_PRIORITY
#undef NX_TELNET_SERVER_TIMEOUT
#undef NX_TELNET_SERVER_WINDOW_SIZE
#undef NX_TELNET_TIME_TO_LIVE
#undef NX_TELNET_TIMEOUT_PERIOD
#undef NX_TELNET_TOS
#undef NX_TFTP_ERROR_STRING_MAX
#undef NX_TFTP_FRAGMENT_OPTION
#undef NX_TFTP_MAX_CLIENTS
#undef NX_TFTP_SERVER_PRIORITY
#undef NX_TFTP_SERVER_TIME_SLICE
#undef NX_TFTP_SOURCE_PORT
#undef NX_TFTP_TIME_TO_LIVE
#undef NX_TFTP_TYPE_OF_SERVICE
#undef NX_TRACE_INTERNAL_ARP_REQUEST_RECEIVE
#undef NX_TRACE_OBJECT_TYPE_IP
#undef NXD_TRACE_ICMP_ENABLE
#undef NX_IPV6_DESTINATION_TABLE_SIZE
#undef NX_DISABLE_IPV6_DAD
#undef NX_DISABLE_ICMPV6_ERROR_MESSAGE
#undef NX_DISABLE_ICMPV6_REDIRECT_PROCESS
#undef NX_DISABLE_ICMPV6_ROUTER_ADVERTISEMENT_PROCESS
#undef NX_DISABLE_ICMPV6_ROUTER_SOLICITATION
#undef NX_ICMPV6_MAX_RTR_SOLICITATIONS
#undef NX_ICMPV6_RTR_SOLICITATION_INTERVAL
#undef NX_DISABLE_FRAGMENTATION
#undef NX_FRAGMENT_IMMEDIATE_ASSEMBLY
#undef NX_DISABLE_ICMPV4_RX_CHECKSUM
#undef NX_DISABLE_ICMPV6_RX_CHECKSUM
#undef NX_DISABLE_ICMP_RX_CHECKSUM
#undef NX_DISABLE_ICMPV4_TX_CHECKSUM
#undef NX_DISABLE_ICMPV6_TX_CHECKSUM
#undef NX_DISABLE_ICMP_TX_CHECKSUM
#undef NX_DISABLE_IP_RX_CHECKSUM
#undef NX_DISABLE_IP_TX_CHECKSUM
#undef NX_DISABLE_TCP_RX_CHECKSUM
#undef NX_DISABLE_TCP_TX_CHECKSUM
#undef NX_DISABLE_UDP_RX_CHECKSUM
#undef NX_DISABLE_UDP_TX_CHECKSUM
#undef NX_IPV4_MAX_REASSEMBLY_TIME
#undef NX_IPV6_MAX_REASSEMBLY_TIME
#undef NX_DISABLE_UDP_INFO
#undef NX_DISABLE_RX_SIZE_CHECKING
#undef NX_ENABLE_PACKET_DEBUG_INFO
#undef NX_PACKET_ALIGNMENT
#undef NX_DISABLE_PACKET_CHAIN
#undef NX_ENABLE_DUAL_PACKET_POOL
#undef NX_DRIVER_DEFERRED_PROCESSING
#undef NX_ENABLE_SOURCE_ADDRESS_CHECK
#undef NX_DISABLE_ASSERT

#undef NTP_SECONDS_AT_01011999


#ifdef PLATFORM_L1_CACHE_SHIFT

#include "platform_cache_def.h"

#define NX_PACKET_HEADER_SIZE     60    /* Bytes of header not counting padding. Header can be changed by stack features enabling / disabling. Static assert is added somewhere to make sure header is aligned. */
#define NX_PACKET_HEADER_PAD_SIZE ((PLATFORM_L1_CACHE_ROUND_UP(NX_PACKET_HEADER_SIZE) - NX_PACKET_HEADER_SIZE) / 4)

#if NX_PACKET_HEADER_PAD_SIZE
#define NX_PACKET_HEADER_PAD
#else
#undef NX_PACKET_HEADER_PAD_SIZE
#endif

#endif /* PLATFORM_L1_CACHE_SHIFT */

#endif

