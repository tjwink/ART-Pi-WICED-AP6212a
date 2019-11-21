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
/*                                                           5.10 SP3     */
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
/*  07-04-2009     William E. Lamie         Modified comment(s),          */ 
/*                                            resulting in version 5.2    */ 
/*  12-31-2009     Yuxin Zhou               Modified comment(s), added    */
/*                                            options for enabling static */
/*                                            routing, loopback interface,*/
/*                                            multiple physical interface */
/*                                            support, and IGMPv2 support,*/
/*                                            removed internal debug logic*/ 
/*                                            added TCP window scaling,   */ 
/*                                            resulting in version 5.3    */
/*  06-30-2011     Yuxin Zhou               Modified comment(s), added    */ 
/*                                            receive and transmit UDP    */ 
/*                                            checksum disable options,   */
/*                                            and added option to disable */ 
/*                                            extended notify logic,      */ 
/*                                            resulting in version 5.4    */
/*  04-30-2013     Janet Christiansen       Modified comment(s), added    */ 
/*                                            NX_TCP_MSS_CHECKING_ENABLED */ 
/*                                            option to check connection  */
/*                                            requests for a minimum MSS  */
/*                                            value, added source address */ 
/*                                            check for incoming packet,  */ 
/*                                            resulting in version 5.5    */
/*  01-12-2015     Yuxin Zhou               Modified comment(s), added    */
/*                                            NX_ARP_DEFEND_BY_REPLY and  */
/*                                            NX_ARP_DEFEND_BY_INERVAL for*/
/*                                            a feature that responds     */
/*                                            to a conflict GARP packet,  */ 
/*                                            option for limiting packets */
/*                                            on TCP receive queue if     */
/*                                            received out of order, added*/
/*                                            ARP spoof/man in the middle */
/*                                            attack detection feature,   */
/*                                            added NX_TCP_MAX_OUT_OF_    */
/*                                            ORDER_PACKETS to limit #    */
/*                                            packets on receive queue if */
/*                                            one packet is missing,      */
/*                                            resulting in version 5.8    */
/*  02-22-2016     Yuxin Zhou               Modified comment(s), and      */
/*                                            unified ticks per second,   */ 
/*                                            removed unused code, renamed*/
/*                                            symbols, resulting in       */
/*                                            version 5.9                 */
/*  05-10-2016     Yuxin Zhou               Modified comment(s), and      */
/*                                            added symbols for disabling */
/*                                            checksum logic on received  */
/*                                            and transmitted ICMP packet,*/
/*                                            resulting in version 5.10   */
/*  04-03-2017     Yuxin Zhou               Modified comment(s), and      */
/*                                            added symbol for checking   */
/*                                            address of ICMP packet,     */
/*                                            resulting in version 5.10SP2*/
/*  06-30-2017     Yuxin Zhou               Modified comment(s), and      */
/*                                            corrected the comment for   */
/*                                            NX_DISABLE_IGMPV2,          */
/*                                            resulting in version 5.10SP3*/
/*                                                                        */
/**************************************************************************/

#ifndef NX_USER_H
#define NX_USER_H

#define WICED_CUSTOM_NX_USER_H

#define NX_DISABLE_INCLUDE_SOURCE_CODE

/* Define various build options for the NetX port.  The application should either make changes
   here by commenting or un-commenting the conditional compilation defined OR supply the defines 
   though the compiler's equivalent of the -D option.  */


/* Override various options with default values already assigned in nx_api.h or nx_port.h. Please 
   also refer to nx_port.h for descriptions on each of these options.  */

/* Defined, this option bypasses the basic NetX error checking. This define is typically used
   after the application is fully debugged.  */

#ifndef DEBUG
#define NX_DISABLE_ERROR_CHECKING
#else
#undef NX_DISABLE_ERROR_CHECKING
#endif

/* Defined, this option enables IP static routing feature.  By default IP static routing
   feature is not compiled in. */
/*
#define NX_ENABLE_IP_STATIC_ROUTING 
*/

/* This define specifies the size of the physical packet header. The default value is 16 (based on 
   a typical 16-byte Ethernet header).  */

#define NX_PHYSICAL_HEADER          (14 + 12 + 18)


/* This define specifies the size of the physical packet trailer and is typically used to reserve storage 
   for things like Ethernet CRCs, etc.  */

#define NX_PHYSICAL_TRAILER         (0)

/* This defines specifies the number of ThreadX timer ticks in one second. The default value is based
   on ThreadX timer interrupt.  */
extern unsigned long host_rtos_get_tickrate( void );
#define NX_IP_PERIODIC_RATE ( host_rtos_get_tickrate( ) )

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


/* This define specifies the maximum number of multicast groups that can be joined. The default value is
   7.  */

/*
#define NX_MAX_MULTICAST_GROUPS     7
*/


/* This define specifies the maximum number of TCP server listen requests. The default value is 10.  */

#define NX_MAX_LISTEN_REQUESTS      (18)


/* Defined, this option enables the optional TCP keepalive timer.  */

#define NX_ENABLE_TCP_KEEPALIVE


/* Defined, this option enables the TCP window scaling feature. (RFC 1323). Default disabled. */
#define NX_ENABLE_TCP_WINDOW_SCALING  



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


/* Defined, this option enables deferred driver packet handling. This allows the driver to place a raw
   packet on the IP instance and have the driver's real processing routine called from the NetX internal
   IP helper thread.  */

/*
#define NX_DRIVER_DEFERRED_PROCESSING
*/


/* Defined, this option disables NetX support on the 127.0.0.1 loopback interface. 
   127.0.0.1 loopback interface is enabled by default.  Uncomment out the follow code to disable
   the loopback interface. */
/* 
#define NX_DISABLE_LOOPBACK_INTERFACE
*/

/* This option defines the number of physical network interfaces to support.  Default is one*/
/*
#define NX_MAX_PHYSICAL_INTERFACES 1
*/

/* Defined, this option disables all IP fragmentation logic.  */

/*
#define NX_DISABLE_FRAGMENTATION
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


/* Defined, this option disables checksum logic on transmitted UDP packets.  */

#define NX_DISABLE_UDP_TX_CHECKSUM


/* Defined, this option disables checksum logic on received ICMP packets.  */
/*
#define NX_DISABLE_ICMP_RX_CHECKSUM
*/


/* Defined, this option disables checksum logic on transmitted ICMP packets.  */
/*
#define NX_DISABLE_ICMP_TX_CHECKSUM
*/


/* Defined, this option disables the reset processing during disconnect when the timeout value is 
   specified as NX_NO_WAIT.  */

/*
#define NX_DISABLE_RESET_DISCONNECT
*/

/* Defined, this option disables the addition size checking on received packets.  */

/*
#define NX_DISABLE_RX_SIZE_CHECKING
*/


/* Defined, ARP information gathering is disabled.  */

#define NX_DISABLE_ARP_INFO


/* Defined, IP information gathering is disabled.  */

#define NX_DISABLE_IP_INFO


/* Defined, ICMP information gathering is disabled.  */

#define NX_DISABLE_ICMP_INFO

/* Defined, IGMP v2 support is disabled.  By default NetX 
   is built with IGMPv2 enabled .  By uncommenting this option, 
   NetX reverts back to IGMPv1 only. */
/* 
#define NX_DISABLE_IGMPV2
*/

/* Defined, IGMP information gathering is disabled.  */

#define NX_DISABLE_IGMP_INFO


/* Defined, packet information gathering is disabled.  */

#define NX_DISABLE_PACKET_INFO


/* Defined, RARP information gathering is disabled.  */

#define NX_DISABLE_RARP_INFO


/* Defined, TCP information gathering is disabled.  */

#define NX_DISABLE_TCP_INFO


/* Defined, UDP information gathering is disabled.  */

#define NX_DISABLE_UDP_INFO


/* Defined, extended notify support is enabled.  This feature adds additional callback/notify services
   to NetX API for notifying the host of socket events, such as TCP connection and disconnect
   completion.  The default is that the extended notify feature is enabled.   */
#define NX_ENABLE_EXTENDED_NOTIFY_SUPPORT


/* Defined, NX_PACKET structure is padded for alignment purpose. The default is no padding. */
/*
#define NX_PACKET_HEADER_PAD 
#define NX_PACKET_HEADER_PAD_SIZE 1
*/

/* If defined, the incoming SYN packet (connection request) is checked for a minimum acceptable
   MSS for the host to accept the connection. The default minimum should be based on the host 
   application packet pool payload, socket transmit queue depth and relevant application specific parameters. 
#define NX_ENABLE_TCP_MSS_CHECKING
#define NX_TCP_MSS_MINIMUM              128
*/

/* Defined, the source address of incoming packet is checked. The default is disabled. */
/*
#define NX_ENABLE_SOURCE_ADDRESS_CHECK
*/

/* Define the ARP defend interval. The default value is 10 seconds.  */
/*
#define NX_ARP_DEFEND_INTERVAL  10
*/

/* To limit the number of out of order packets stored to the TCP receive queue and prevent
   possible packet pool depletion, define this to a non zero value: 
*/
#define NX_TCP_MAX_OUT_OF_ORDER_PACKETS 8

/* Defined, the destination address of ICMP packet is checked. The default is disabled.
   An ICMP Echo Request destined to an IP broadcast or IP multicast address will be silently discarded.
*/
/*
#define NX_ENABLE_ICMP_ADDRESS_CHECK
*/

#define NX_FTP_NO_FILEX
#define NX_HTTP_NO_FILEX
#define NX_TFTP_NO_FILEX
#define NX_DHCP_CLIENT_RESTORE_STATE

#undef NX_LINK_MTU
#define NX_DHCP_PACKET_PAYLOAD      (NX_BOOT_CLIENT_BUFFER_SIZE + NX_PHYSICAL_HEADER + 20 + 8 + 4 + 4)

#define PACKET_RELEASE_NOTIFY

extern void packet_release_notify( void* pool );

#define NX_RAND                         nx_rand16
extern UINT nx_rand16( void );


/* These are all the defines that are used in NetX #if statements
 * This prevents users changing NetX values to become incompatible with
 * the prebuilt libraries.
 *
 */


#undef __time_t_defined
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

#undef AUTHENTICATION_REQUIRED
#undef EL_PRINTF_ENABLE
#undef NX_ARP_DEFEND_BY_REPLY
#undef NX_ENABLE_ARP_MAC_CHANGE_NOTIFICATION
#undef FILEX_STUB_H

#undef PACKET_DUMP
#undef PRIVACY_REQUIRED
#undef REQUEST_CLIENT_IP
#undef SKIP_DISCOVER_MESSAGE
#undef TESTOUTPUT

#undef NX_ARP_DEFEND_INTERVAL
#undef NX_ARP_EXPIRATION_RATE
#undef NX_ARP_MAX_QUEUE_DEPTH
#undef NX_ARP_MAXIMUM_RETRIES
#undef NX_AUTO_IP_ANNOUNCE_INTERVAL
#undef NX_AUTO_IP_ANNOUNCE_NUM
#undef NX_AUTO_IP_ANNOUNCE_WAIT
#undef NX_AUTO_IP_DEBUG
#undef NX_AUTO_IP_DEFEND_INTERVAL
#undef NX_AUTO_IP_MAX_CONFLICTS
#undef NX_AUTO_IP_PROBE_MAX
#undef NX_AUTO_IP_PROBE_MIN
#undef NX_AUTO_IP_PROBE_NUM
#undef NX_AUTO_IP_PROBE_WAIT
#undef NX_AUTO_IP_RATE_LIMIT_INTERVAL
#undef NX_BSD_INCLUDE_DATA_EXTRACT_OFFSET
#undef NX_BSD_INHERIT_LISTENER_SOCKET_SETTINGS
#undef NX_BSD_MAX_LISTEN_BACKLOG
#undef NX_BSD_MAX_SOCKETS
#undef NX_BSD_PRINT_ERRORS
#undef NX_BSD_SOCKFD_START
#undef NX_BSD_TCP_WINDOW
#undef NX_BSD_TIMED_WAIT_TIMEOUT
#undef NX_BSD_TIMEOUT
#undef NX_DEBUG
#undef NX_DEBUG_PACKET
#undef NX_DHCP_CLIENT_HOSTNAME_MAX
#undef NX_DHCP_CLIENT_IDENTIFIER_MAX
#undef NX_DHCP_CLIENT_OPTIONS_MAX
#undef NX_DHCP_CLIENT_RECORD_TABLE_SIZE
#undef NX_DHCP_CLIENT_SESSION_TIMEOUT
#undef NX_DHCP_DEFAULT_LEASE_TIME
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
#undef NX_DHCP_THREAD_PRIORITY
#undef NX_DHCP_THREAD_STACK_SIZE
#undef NX_DHCP_TIME_INTERVAL
#undef NX_DHCP_TIME_TO_LIVE
#undef NX_DHCP_TYPE_OF_SERVICE
#undef NX_DIRECT_ISR_CALL
#undef NX_DISABLE_EXTENDED_NOTIFY_SUPPORT
#undef NX_DISABLE_FRAGMENTATION
#undef NX_DISABLE_IGMPV2
#undef NX_DISABLE_IP_RX_CHECKSUM
#undef NX_DISABLE_IP_TX_CHECKSUM
#undef NX_DISABLE_LOOPBACK_INTERFACE
#undef NX_DISABLE_RX_SIZE_CHECKING
#undef NX_DISABLE_TCP_RX_CHECKSUM
#undef NX_DISABLE_TCP_TX_CHECKSUM
#undef NX_DISABLE_UDP_RX_CHECKSUM
#undef NX_DISABLE_ICMP_RX_CHECKSUM
#undef NX_DISABLE_ICMP_TX_CHECKSUM
#undef NX_DISABLE_RESET_DISCONNECT
#undef NX_DNS_FRAGMENT_OPTION
#undef NX_DNS_MAX_SERVERS
#undef NX_DNS_TIME_TO_LIVE
#undef NX_DNS_TYPE_OF_SERVICE
#undef NX_DRIVER_DEFERRED_PROCESSING
#undef NX_ENABLE_IP_STATIC_ROUTING
#undef NX_ENABLE_SOURCE_ADDRESS_CHECK
#undef NX_ENABLE_TCP_MSS_CHECK
#undef NX_FTP_ACTIVITY_TIMEOUT
#undef NX_FTP_CONTROL_TOS
#undef NX_FTP_CONTROL_WINDOW_SIZE
#undef NX_FTP_DATA_TOS
#undef NX_FTP_DATA_WINDOW_SIZE
#undef NX_FTP_FAULT_TOLERANT
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
#undef NX_HTTP_DIGEST_ENABLE
#undef NX_HTTP_FRAGMENT_OPTION
#undef NX_HTTP_MAX_NAME
#undef NX_HTTP_MAX_PASSWORD
#undef NX_HTTP_MAX_RESOURCE
#undef NX_HTTP_SERVER_MAX_PENDING
#undef NX_HTTP_SERVER_MIN_PACKET_SIZE
#undef NX_HTTP_SERVER_OMIT_CONTENT_LENGTH
#undef NX_HTTP_SERVER_PRIORITY
#undef NX_HTTP_SERVER_RETRY_MAX
#undef NX_HTTP_SERVER_RETRY_SECONDS
#undef NX_HTTP_SERVER_RETRY_SHIFT
#undef NX_HTTP_SERVER_THREAD_TIME_SLICE
#undef NX_HTTP_SERVER_TIMEOUT
#undef NX_HTTP_SERVER_TRANSMIT_QUEUE_DEPTH
#undef NX_HTTP_SERVER_WINDOW_SIZE
#undef NX_HTTP_TIME_TO_LIVE
#undef NX_HTTP_TYPE_OF_SERVICE
#undef NX_IP_DEBUG_LOG_SIZE
#undef NX_IP_RAW
#undef NX_IP_ROUTING_TABLE_SIZE
#undef NX_IP_STATUS_CHECK_WAIT_TIME
#undef NX_MAX_MULTICAST_GROUPS
#undef NX_MAX_PHYSICAL_INTERFACES
#undef NX_MICROSECOND_PER_CPU_TICK
#undef NX_PACKET_HEADER_PAD
#undef NX_PACKET_HEADER_PAD_SIZE
#undef NX_PACKET_OFFSET_ERROR
#undef NX_POP3_CLIENT_CONNECTION_TIMEOUT
#undef NX_POP3_CLIENT_DISCONNECT_TIMEOUT
#undef NX_POP3_CLIENT_MAIL_BUFFER_SIZE
#undef NX_POP3_CLIENT_PACKET_TIMEOUT
#undef NX_POP3_MAX_PASSWORD
#undef NX_POP3_MAX_USERNAME
#undef NX_POP3_TCP_SOCKET_SEND_WAIT
#undef NX_PPP_DEBUG_FRAME_SIZE
#undef NX_PPP_DEBUG_LOG_ENABLE
#undef NX_PPP_DEBUG_LOG_PRINT_ENABLE
#undef NX_PPP_DEBUG_LOG_SIZE
#undef NX_PPP_DISABLE_CHAP
#undef NX_PPP_DISABLE_INFO
#undef NX_PPP_DISABLE_PAP
#undef NX_PPP_HASHED_VALUE_SIZE
#undef NX_PPP_MINIMUM_MRU
#undef NX_PPP_MRU
#undef NX_PPP_NAME_SIZE
#undef NX_PPP_PASSWORD_SIZE
#undef NX_PPP_RECEIVE_TIMEOUT
#undef NX_PPP_SERIAL_BUFFER_SIZE
#undef NX_PPP_THREAD_TIME_SLICE
#undef NX_PPP_TIMEOUT
#undef NX_PPP_VALUE_SIZE
#undef NX_SMTP_CLIENT_CONNECTION_TIMEOUT
#undef NX_SMTP_CLIENT_PACKET_TIMEOUT
#undef NX_SMTP_DISABLE_ERROR_CHECKING
#undef NX_SNMP_AGENT_PORT
#undef NX_SNMP_AGENT_PRIORITY
#undef NX_SNMP_AGENT_TIMEOUT
#undef NX_SNMP_DISABLE_V1
#undef NX_SNMP_DISABLE_V2
#undef NX_SNMP_DISABLE_V3
#undef NX_SNMP_FRAGMENT_OPTION
#undef NX_SNMP_MANAGER_TRAP_PORT
#undef NX_SNMP_MAX_CONTEXT_STRING
#undef NX_SNMP_MAX_OCTET_STRING
#undef NX_SNMP_MAX_SECURITY_KEY
#undef NX_SNMP_MAX_TRAP_KEY
#undef NX_SNMP_MAX_TRAP_NAME
#undef NX_SNMP_MAX_USER_NAME
#undef NX_SNMP_NO_SECURITY
#undef NX_SNMP_PACKET_SIZE
#undef NX_SNMP_TIME_TO_LIVE
#undef NX_SNMP_TYPE_OF_SERVICE
#undef NX_SNMP_V1_AUTHORIZATION_ERROR_RESPONSE
#undef NX_SNMP_V2C_ONLY
#undef NX_SNTP_CLIENT_EXP_BACKOFF_RATE
#undef NX_SNTP_CLIENT_IGNORE_MAX_ADJUST_STARTUP
#undef NX_SNTP_CLIENT_MAX_QUEUE_DEPTH
#undef NX_SNTP_CLIENT_MAX_ROOT_DISPERSION
#undef NX_SNTP_CLIENT_MAX_TIME_ADJUSTMENT
#undef NX_SNTP_CLIENT_MIN_NTP_VERSION
#undef NX_SNTP_CLIENT_MIN_TIME_ADJUSTMENT
#undef NX_SNTP_CLIENT_NTP_VERSION
#undef NX_SNTP_CLIENT_PACKET_TIMEOUT
#undef NX_SNTP_CLIENT_PREEMPTION_THRESHOLD
#undef NX_SNTP_CLIENT_THREAD_PRIORITY
#undef NX_SNTP_CLIENT_THREAD_STACK_SIZE
#undef NX_SNTP_CLIENT_THREAD_TIME_SLICE
#undef NX_SNTP_CLIENT_TIME_TO_LIVE
#undef NX_SNTP_CLIENT_UDP_PORT
#undef NX_SNTP_CLIENT_UDP_SOCKET_NAME
#undef NX_SNTP_CLIENT_UNICAST_POLL_INTERVAL
#undef NX_SNTP_DISABLE_ERROR_CHECKING
#undef NX_SNTP_UPDATE_TIMEOUT_INTERVAL
#undef NX_TCP_ACK_TIMER_RATE
#undef NX_TCP_ENABLE_DEBUG_LOG
#undef NX_TCP_FAST_TIMER_RATE
#undef NX_TCP_IMMEDIATE_ACK
#undef NX_TCP_KEEPALIVE_INITIAL
#undef NX_TCP_KEEPALIVE_RETRIES
#undef NX_TCP_KEEPALIVE_RETRY
#undef NX_TCP_MAXIMUM_RETRIES
#undef NX_TCP_MAXIMUM_TX_QUEUE
#undef NX_TCP_MSS_MINIMUM
#undef NX_TCP_RETRY_SHIFT
#undef NX_TCP_TRANSMIT_TIMER_RATE
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
#undef NX_TFTP_TIME_TO_LIVE
#undef NX_TFTP_TYPE_OF_SERVICE
#undef NX_TRACE_INTERNAL_ARP_REQUEST_RECEIVE
#undef NX_TRACE_OBJECT_TYPE_IP
#undef NX_UDP_DEBUG_LOG_SIZE

#ifdef PLATFORM_L1_CACHE_SHIFT

#include "platform_cache_def.h"

#define NX_PACKET_HEADER_SIZE     52 /* Bytes of header not counting padding. Header can be changed by stack features enabling / disabling. Static assert is added somewhere to make sure header is aligned. */
#define NX_PACKET_HEADER_PAD_SIZE ((PLATFORM_L1_CACHE_ROUND_UP(NX_PACKET_HEADER_SIZE) - NX_PACKET_HEADER_SIZE) / 4)

#if NX_PACKET_HEADER_PAD_SIZE
#define NX_PACKET_HEADER_PAD
#else
#undef NX_PACKET_HEADER_PAD_SIZE
#endif

#endif /* PLATFORM_L1_CACHE_SHIFT */


#endif

