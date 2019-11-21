/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#include "network/wwd_network_constants.h"

#include "platform_cache_def.h"

#ifdef CUSTOM_LWIPOPTS
#include "custom_lwipopts.h"
#else /* ifdef CUSTOM_LWIPOPTS */

#ifdef __cplusplus
extern "C" {
#endif



/**
 * LWIP_NETCONN==1: Enable Netconn API (require to use api_lib.c)
 */
#define LWIP_NETCONN 1
/**
 * LWIP_SOCKET==1: Enable Socket API (require to use sockets.c)
 */
#define LWIP_SOCKET 1

/**
 * MEM_ALIGNMENT: should be set to the alignment of the CPU
 *    4 byte alignment -> #define MEM_ALIGNMENT 4
 *    2 byte alignment -> #define MEM_ALIGNMENT 2
 */
#if defined(PLATFORM_L1_CACHE_SHIFT)
/* If we're using a dcache, we need to ensure we're cache aligned */
#define MEM_ALIGNMENT                  (PLATFORM_L1_CACHE_BYTES)
#else
#define MEM_ALIGNMENT                  (4)
#endif

/**
 * MEM_LIBC_MALLOC==1: Use malloc/free/realloc provided by your C-library
 * instead of the lwip internal allocator. Can save code size if you
 * already use it.
 */
#define MEM_LIBC_MALLOC                (1)

/**
 * MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
 * per active UDP "connection".
 * (requires the LWIP_UDP option)
 */
#define MEMP_NUM_UDP_PCB               10

/**
 * MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP segments.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_SEG               (TCP_SND_QUEUELEN+1)

/**
 * PBUF_POOL_TX_SIZE: the number of buffers in the Tx pbuf pool.
 */
#ifndef PBUF_POOL_TX_SIZE
#ifdef TX_PACKET_POOL_SIZE
#define PBUF_POOL_TX_SIZE                 TX_PACKET_POOL_SIZE
#else
#define PBUF_POOL_TX_SIZE                 (10)
#endif
#endif

/**
 * PBUF_POOL_RX_SIZE: the number of buffers in the Rx pbuf pool.
 */
#ifndef PBUF_POOL_RX_SIZE
# ifdef RX_PACKET_POOL_SIZE
#  define PBUF_POOL_RX_SIZE               RX_PACKET_POOL_SIZE
# else /* RX_PACKET_POOL_SIZE */
#  ifdef BCM43909
    /* Cortex-R4 uses many more buffers */
#   define PBUF_POOL_RX_SIZE              (45)
#  else /* BCM43909 */
#   define PBUF_POOL_RX_SIZE              (7)
#  endif /* BCM43909 */
# endif /* RX_PACKET_POOL_SIZE */
#endif /* PBUF_POOL_RX_SIZE */

/**
 * MEMP_NUM_NETBUF: the number of struct netbufs.
 * (only needed if you use the sequential API, like api_lib.c)
 *
 * A struct netbuf may be allocated for every transmit and
 * receive buffer in WICED. However, on 4390x, it is possible
 * a lot more netbuf structs are needed to accomodate for
 * user supplied pbufs.
 */
#ifdef BCM43909
#   define MEMP_NUM_NETBUF                 500
#else
#   define MEMP_NUM_NETBUF                 (PBUF_POOL_TX_SIZE + PBUF_POOL_RX_SIZE)
#endif /* BCM43909 */

/**
 * IP_REASS_MAX_PBUFS: Total maximum amount of pbufs waiting to be reassembled.
 * Since the received pbufs are enqueued, be sure to configure
 * PBUF_POOL_RX_SIZE > IP_REASS_MAX_PBUFS so that the stack is still able to receive
 * packets even if the maximum amount of fragments is enqueued for reassembly!
 */
#if PBUF_POOL_TX_SIZE > 2
#ifndef IP_REASS_MAX_PBUFS
#define IP_REASS_MAX_PBUFS              (PBUF_POOL_TX_SIZE - 2)
#endif
#else
#define IP_REASS_MAX_PBUFS              0
#define IP_REASSEMBLY                   0
#endif

/**
 * MEMP_NUM_REASSDATA: the number of IP packets simultaneously queued for
 * reassembly (whole packets, not fragments!)
 */
#if IP_REASS_MAX_PBUFS > 1
#ifndef MEMP_NUM_REASSDATA
#define MEMP_NUM_REASSDATA              (IP_REASS_MAX_PBUFS - 1)
#endif
#else
#define MEMP_NUM_REASSDATA              0
#endif

/**
 * PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. The default is
 * designed to accommodate single full size TCP frame in one pbuf, including
 * TCP_MSS, IP header, and link header.
 */
#define PBUF_POOL_BUFSIZE              (LWIP_MEM_ALIGN_SIZE(WICED_LINK_MTU) + LWIP_MEM_ALIGN_SIZE(sizeof(struct pbuf)) + 1)

/**
 * LWIP_TCPIP_CORE_LOCKING
 * Creates a global mutex that is held during TCPIP thread operations.
 * Can be locked by client code to perform lwIP operations without changing
 * into TCPIP thread using callbacks. See LOCK_TCPIP_CORE() and
 * UNLOCK_TCPIP_CORE().
 * Your system should provide mutexes supporting priority inversion to use this.
 *
 * The preference for WICED is to have TCP/IP processing done in it's own
 * thread to not put a strain on stack requirements of threads using TCP.
 */
#define LWIP_TCPIP_CORE_LOCKING         0

/**
 * TCP_MSS: TCP Maximum segment size. (default is 536, a conservative default,
 * you might want to increase this.)
 * For the receive side, this MSS is advertised to the remote side
 * when opening a connection. For the transmit size, this MSS sets
 * an upper limit on the MSS advertised by the remote host.
 */
/* Ethernet MTU - IP header size - TCP header size */
#define TCP_MSS                        (WICED_PAYLOAD_MTU-20-20)

/**
 * TCP_SND_BUF: TCP sender buffer space (bytes).
 * To achieve good performance, this should be at least 2 * TCP_MSS.
 */
#ifdef TX_PACKET_POOL_SIZE
#define TCP_SND_BUF                    LWIP_MAX((2 * TCP_MSS), ((TX_PACKET_POOL_SIZE/4) * TCP_MSS))
#else
#define TCP_SND_BUF                    (6 * TCP_MSS)
#endif

/**
 * TCP_WND: The size of a TCP window.  This must be at least
 * (2 * TCP_MSS) for things to work well.
 * ATTENTION: when using TCP_RCV_SCALE, TCP_WND is the total size
 * with scaling applied. Maximum window value in the TCP header
 * will be TCP_WND >> TCP_RCV_SCALE
 */
#ifdef RX_PACKET_POOL_SIZE
#define TCP_WND                        ((LWIP_MIN(RX_PACKET_POOL_SIZE, PBUF_POOL_RX_SIZE)/2) * TCP_MSS)
#endif

/**
 * PBUF_LINK_HLEN: the number of bytes that should be allocated for a
 * link level header. The default is 14, the standard value for
 * Ethernet.
 */
#define PBUF_LINK_HLEN                 (WICED_PHYSICAL_HEADER)

/**
 * LWIP_NETIF_TX_SINGLE_PBUF: if this is set to 1, lwIP tries to put all data
 * to be sent into one single pbuf. This is for compatibility with DMA-enabled
 * MACs that do not support scatter-gather.
 * Beware that this might involve CPU-memcpy before transmitting that would not
 * be needed without this flag! Use this only if you need to!
 *
 * @todo: TCP and IP-frag do not work with this, yet:
 */
/* TODO: remove this option once buffer chaining has been implemented */
#define LWIP_NETIF_TX_SINGLE_PBUF      (1)

/** Define LWIP_COMPAT_MUTEX if the port has no mutexes and binary semaphores
 *  should be used instead
 */
#define LWIP_COMPAT_MUTEX              (1)
#define LWIP_COMPAT_MUTEX_ALLOWED

/**
 * LWIP_RAW==1: Enable application layer to hook into the IP layer itself.
 */
#define LWIP_RAW                       (1)

/**
 * LWIP_IPV6==1: Enable IPv6
 */
#define LWIP_IPV6                       0

/**
 * TCPIP_THREAD_STACKSIZE: The stack size used by the main tcpip thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_STACKSIZE         (4 * 1024)

/**
 * TCPIP_THREAD_PRIO: The priority assigned to the main tcpip thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_PRIO              (7)

/**
 * TCP_LISTEN_BACKLOG: Enable the backlog option for tcp listen pcb.
 */
#define TCP_LISTEN_BACKLOG     (1)

/**
 * LWIP_DHCP==1: Enable DHCP module.
 */
#define LWIP_DHCP                      (1)

/* WICED likes to enable AUTOIP */
#ifdef AUTO_IP_ENABLED
/**
 * LWIP_AUTOIP==1: Enable AutoIP module.
 */
#   define LWIP_AUTOIP                 (1)

/* Note: Not enabling LWIP_DHCP_AUTOIP_COOP on purpose */
#endif /* AUTO_IP_ENABLED */

/**
 * LWIP_PROVIDE_ERRNO: System does not have errno defines - force LwIP to create them
 */
#define LWIP_PROVIDE_ERRNO             (1)

/** LWIP_TCPIP_TIMEOUT==1: Enable tcpip_timeout/tcpip_untimeout to create
 * timers running in tcpip_thread from another thread.
 */
#define LWIP_TCPIP_TIMEOUT             (1)

/**
 * DHCP_DOES_ARP_CHECK==1: Do an ARP check on the offered address.
 */
/* ARP before DHCP causes multi-second delay  - turn it off */
#define DHCP_DOES_ARP_CHECK            (0)

/**
 * ARP_QUEUEING==1: Multiple outgoing packets are queued during hardware address
 * resolution. By default, only the most recent packet is queued per IP address.
 * This is sufficient for most protocols and mainly reduces TCP connection
 * startup time. Set this to 1 if you know your application sends more than one
 * packet in a row to an IP address that is not in the ARP cache.
 */
/* ARP Queue size needs to be reduced to avoid using up all PBUFs when SoftAP is in use under load in busy environments */
#define ARP_QUEUEING                   (1)

/**
 * MEMP_NUM_ARP_QUEUE: the number of simultaneously queued outgoing
 * packets (pbufs) that are waiting for an ARP request (to resolve
 * their destination address) to finish.
 * (requires the ARP_QUEUEING option)
 */
#define MEMP_NUM_ARP_QUEUE              5

/**
 * MEMP_NUM_NETCONN: the number of struct netconns.
 * (only needed if you use the sequential API, like api_lib.c)
 */
#define MEMP_NUM_NETCONN               (18)

/**
 * LWIP_SO_RCVTIMEO==1: Enable receive timeout for sockets/netconns and
 * SO_RCVTIMEO processing.
 */
#define LWIP_SO_RCVTIMEO               (1)

/**
 * LWIP_IGMP==1: Turn on IGMP module.
 */
#define LWIP_IGMP                      (1)

/**
 * SO_REUSE==1: Enable SO_REUSEADDR option.
 */
/* Required by IGMP for reuse of multicast address and port by other sockets */
#define SO_REUSE                       (1)

/**
 * LWIP_RAND() needs to be defined to a function returning an u32_t random value
 */
#define LWIP_RAND()                    ((u32_t)sys_rand16())

/**
 * LWIP_TCP_KEEPALIVE==1: Enable TCP_KEEPIDLE, TCP_KEEPINTVL and TCP_KEEPCNT
 * options processing. Note that TCP_KEEPIDLE and TCP_KEEPINTVL have to be set
 * in seconds. (does not require sockets.c, and will affect tcp.c)
 */
#define LWIP_TCP_KEEPALIVE             (1)

/**
 * LWIP_DNS==1: Turn on DNS module. UDP must be available for DNS
 * transport.
 */
#define LWIP_DNS                        (1)

#ifdef LWIP_SO_RCVBUF
#if ( LWIP_SO_RCVBUF == 1 )
#include <limits.h>  /* Needed because RECV_BUFSIZE_DEFAULT is defined as INT_MAX */
#endif /* if ( LWIP_SO_RCVBUF == 1 ) */
#endif /* ifdef LWIP_SO_RCVBUF */

/**
 * LWIP_STATS==1: Enable statistics collection in lwip_stats.
 */
#ifdef WICED_LWIP_DEBUG
#define LWIP_STATS                     (1)
#else
#define LWIP_STATS                     (0)
#endif /* ifdef WICED_LWIP_DEBUG */

/**
 * LWIP_RANDOMIZE_INITIAL_LOCAL_PORTS==1: randomize the local port for the first
 * local TCP/UDP pcb (default==0). This can prevent creating predictable port
 * numbers after booting a device.
 */
#define LWIP_RANDOMIZE_INITIAL_LOCAL_PORTS 1

/**
 * LWIP_NETIF_STATUS_CALLBACK==1: Support a callback function whenever an interface
 * changes its up/down status (i.e., due to DHCP IP acquisition)
 */
#define LWIP_NETIF_STATUS_CALLBACK     (1)

/**
 * LWIP_NETIF_API==1: Support netif api (in netifapi.c)
 */
#define LWIP_NETIF_API                  1

/**
 * LWIP_NETIF_HOSTNAME==1: use DHCP_OPTION_HOSTNAME with netif's hostname
 * field.
 */
#define LWIP_NETIF_HOSTNAME             1

/** LWIP_SUPPORT_CUSTOM_PBUF==1: Custom pbufs behave much like their pbuf type
 * but they are allocated by external code (initialised by calling
 * pbuf_alloced_custom()) and when pbuf_free gives up their last reference, they
 * are freed by calling pbuf_custom->custom_free_function().
 * Currently, the pbuf_custom code is only needed for one specific configuration
 * of IP_FRAG, unless required by external driver/application code. */
#define LWIP_SUPPORT_CUSTOM_PBUF        1

/**
 * Debug printing
 * By default enable debug printing for debug build, but set level to off
 * This allows user to change any desired debug level to on.
 */

#ifdef WICED_LWIP_DEBUG
#define LWIP_DEBUG
#define MEMP_OVERFLOW_CHECK            ( 2 )
#define MEMP_SANITY_CHECK              ( 1 )

#define MEM_DEBUG                      (LWIP_DBG_OFF)
#define MEMP_DEBUG                     (LWIP_DBG_OFF)
#define PBUF_DEBUG                     (LWIP_DBG_OFF)
#define API_LIB_DEBUG                  (LWIP_DBG_OFF)
#define API_MSG_DEBUG                  (LWIP_DBG_OFF)
#define TCPIP_DEBUG                    (LWIP_DBG_OFF)
#define NETIF_DEBUG                    (LWIP_DBG_OFF)
#define SOCKETS_DEBUG                  (LWIP_DBG_OFF)
#define IP_DEBUG                       (LWIP_DBG_OFF)
#define IP_REASS_DEBUG                 (LWIP_DBG_OFF)
#define RAW_DEBUG                      (LWIP_DBG_OFF)
#define ICMP_DEBUG                     (LWIP_DBG_OFF)
#define UDP_DEBUG                      (LWIP_DBG_OFF)
#define TCP_DEBUG                      (LWIP_DBG_OFF)
#define TCP_INPUT_DEBUG                (LWIP_DBG_OFF)
#define TCP_OUTPUT_DEBUG               (LWIP_DBG_OFF)
#define TCP_RTO_DEBUG                  (LWIP_DBG_OFF)
#define TCP_CWND_DEBUG                 (LWIP_DBG_OFF)
#define TCP_WND_DEBUG                  (LWIP_DBG_OFF)
#define TCP_FR_DEBUG                   (LWIP_DBG_OFF)
#define TCP_QLEN_DEBUG                 (LWIP_DBG_OFF)
#define TCP_RST_DEBUG                  (LWIP_DBG_OFF)
#define PPP_DEBUG                      (LWIP_DBG_OFF)
#define ETHARP_DEBUG                   (LWIP_DBG_OFF)
#define IGMP_DEBUG                     (LWIP_DBG_OFF)
#define INET_DEBUG                     (LWIP_DBG_OFF)
#define SYS_DEBUG                      (LWIP_DBG_OFF)
#define TIMERS_DEBUG                   (LWIP_DBG_OFF)
#define SLIP_DEBUG                     (LWIP_DBG_OFF)
#define DHCP_DEBUG                     (LWIP_DBG_OFF)
#define AUTOIP_DEBUG                   (LWIP_DBG_OFF)
#define SNMP_DEBUG                     (LWIP_DBG_OFF)
#define SNMP_MIB_DEBUG                 (LWIP_DBG_OFF)
#define DNS_DEBUG                      (LWIP_DBG_OFF)

#define LWIP_DBG_TYPES_ON              (LWIP_DBG_OFF)   /* (LWIP_DBG_ON|LWIP_DBG_TRACE|LWIP_DBG_STATE|LWIP_DBG_FRESH|LWIP_DBG_HALT) */
#endif


#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifdef CUSTOM_LWIPOPTS */

#endif /* __LWIPOPTS_H__ */
