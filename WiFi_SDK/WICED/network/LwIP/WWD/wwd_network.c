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

/******************************************************************************
 Wiced Includes
 ******************************************************************************/

#include <string.h>
#include "lwip/tcpip.h"
#include "lwip/debug.h"
#include "lwip/netif.h"
#include "lwip/igmp.h"
#include "lwip/ethip6.h"
#include "netif/ppp/ppp_opts.h"
#include "netif/etharp.h"

#include "wwd_network.h"
#include "wwd_wifi.h"
#include "wwd_eapol.h"
#include "network/wwd_network_interface.h"
#include "network/wwd_buffer_interface.h"
#include "wwd_assert.h"
#include "wiced_constants.h"
#include <stdlib.h>
#include "wiced_low_power.h"
#include "wiced_management.h"

#ifdef ADD_LWIP_EAPOL_SUPPORT
#define ETHTYPE_EAPOL    0x888E
#endif

/*****************************************************************************
 * The following is based on the skeleton Ethernet driver in LwIP            *
 *****************************************************************************/


/**
 * @file
 * Ethernet Interface Skeleton
 *
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
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
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "netif/etharp.h"
#include "wwd_bus_protocol.h"

/* 定义它们以更好地描述您的网络接口。 */
#define IFNAME0 'w'
#define IFNAME1 'l'

#define MULTICAST_IP_TO_MAC(ip)       { (uint8_t) 0x01,             \
                                        (uint8_t) 0x00,             \
                                        (uint8_t) 0x5e,             \
                                        (uint8_t) ((ip)[1] & 0x7F), \
                                        (uint8_t) (ip)[2],          \
                                        (uint8_t) (ip)[3]           \
                                      }


/* Forward declarations. */
#if LWIP_IGMP
static err_t lwip_igmp_mac_filter(struct netif *netif,
       const ip4_addr_t *group, enum netif_mac_filter_action action);
#endif

/* 以太网类型的数据包过滤支持。 */
static uint16_t filter_ethertype;
static wwd_interface_t filter_interface;
static wwd_network_filter_ethernet_packet_t filter_ethernet_packet_callback;
static void* filter_userdata;

/**
 *在此功能中，应初始化硬件。
 * Called from ethernetif_init().
 *
 * @param netif此ethernetif的已初始化lwip网络接口结构
 */
static void low_level_init( /*@partial@*/ struct netif *netif )
{
    /*设置MAC硬件地址长度 */
    netif->hwaddr_len = (u8_t) ETHARP_HWADDR_LEN;

    /*设置此IP实例的物理地址。 */
    if ( wwd_wifi_get_mac_address( (wiced_mac_t*) ( netif->hwaddr ), (wwd_interface_t) netif->state ) != WWD_SUCCESS )
    {
        WPRINT_NETWORK_DEBUG(("Couldn't get MAC address\n"));
        return;
    }

    /*设置最大传输单位 */
    netif->mtu = (u16_t) WICED_PAYLOAD_MTU;

    /* 设置设备功能。 如果此设备不是以太网设备，请不要设置NETIF_FLAG_ETHARP*/
    netif->flags = (u8_t) ( NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP );

    /* 执行初始化接口所需的其他任何操作。 */
#if LWIP_IGMP
    netif->flags |= NETIF_FLAG_IGMP;
    netif_set_igmp_mac_filter(netif, lwip_igmp_mac_filter);
#endif
}

/**
*	此功能应完成数据包的实际传输。 
	数据包包含在传递给函数的pbuf中。 此pbuf可能已被链接。
 *
 * @param netif此ethernetif的lwip网络接口结构
 * @paramp	要发送的MAC数据包（例如包含MAC地址和类型的IP数据包）
 * @return ERR_OK如果可以发送数据包，则为err_t值，如果不能发送数据包
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output( struct netif *netif, /*@only@*/ struct pbuf *p )
{
    /*@-noeffect@*/
    UNUSED_PARAMETER( netif );
    /*@+noeffect@*/

    if ( ( (wiced_interface_t) netif->state  == WICED_ETHERNET_INTERFACE ) ||
         ( wwd_wifi_is_ready_to_transceive( (wwd_interface_t) netif->state ) == WWD_SUCCESS ) )
    {
        /* Take a reference to this packet */
        pbuf_ref( p );

        LWIP_ASSERT( "No chained buffers", ( ( p->next == NULL ) && ( ( p->tot_len == p->len ) ) ) );
        wwd_network_send_ethernet_data( p, (wwd_interface_t) netif->state );

        LINK_STATS_INC( link.xmit );

        return (err_t) ERR_OK;
    }
    else
    {
        /* Stop lint warning about packet not being freed - it is not being referenced */ /*@-mustfree@*/
        return (err_t) ERR_INPROGRESS; /* Note that signalling ERR_CLSD or ERR_CONN causes loss of connectivity on a roam */
        /*@+mustfree@*/
    }
}

/**
 * 当准备读取数据包时应调用此函数
 * 来自界面。 它使用函数low_level_input（）
 * 应该处理来自网络的实际字节接收
 * 界面。 然后确定接收到的数据包的类型，并
 * 调用适当的输入函数。
 *
 * @param p : 传入的网络的数据包
 */
void host_network_process_ethernet_data( /*@only@*/ wiced_buffer_t buffer, wwd_interface_t interface )
{
    struct eth_hdr* ethernet_header;
    struct netif*   tmp_netif;
    u8_t            result;
    uint16_t        ethertype;

    if ( buffer == NULL )
        return;

    /* 指向以以太网报头开头的数据包有效负载 */
    ethernet_header = (struct eth_hdr *) buffer->payload;

    ethertype = htons( ethernet_header->type );

    if (filter_ethernet_packet_callback != NULL && filter_ethertype == ethertype && filter_interface == interface)
    {
        filter_ethernet_packet_callback(buffer->payload, filter_userdata);
    }

    /*检查这是否是802.1Q VLAN标记的数据包 */
    if ( ethertype == WICED_ETHERTYPE_8021Q )
    {
        /* Need to remove the 4 octet VLAN Tag, by moving src and dest addresses 4 octets to the right,
         * and then read the actual ethertype. The VLAN ID and priority fields are currently ignored. */
        uint8_t temp_buffer[ 12 ];
        memcpy( temp_buffer, buffer->payload, 12 );
        memcpy( ( (uint8_t*) buffer->payload ) + 4, temp_buffer, 12 );

        buffer->payload = ( (uint8_t*) buffer->payload ) + 4;
        buffer->len = (u16_t) ( buffer->len - 4 );

        ethernet_header = (struct eth_hdr *) buffer->payload;
        ethertype = htons( ethernet_header->type );
    }

    if ( WICED_DEEP_SLEEP_IS_ENABLED() && ( WICED_DEEP_SLEEP_SAVE_PACKETS_NUM != 0 ) )
    {
        if ( wiced_deep_sleep_save_packet( buffer, interface ) )
        {
            return;
        }
    }

    switch ( ethertype )
    {
        case WICED_ETHERTYPE_IPv4:
        case WICED_ETHERTYPE_ARP:
#if PPPOE_SUPPORT
        /* PPPoE packet? */
        case ETHTYPE_PPPOEDISC:
        case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
            /* Find the netif object matching the provided interface */
            for ( tmp_netif = netif_list; ( tmp_netif != NULL ) && ( tmp_netif->state != (void*) interface ); tmp_netif = tmp_netif->next )
            {
            }

            if ( tmp_netif == NULL )
            {
                /* 收到的网络接口数据包未初始化无法对数据包执行任何操作-丢弃它. */
                result = pbuf_free( buffer );
                LWIP_ASSERT("Failed to release packet buffer", ( result != (u8_t)0 ) );
                buffer = NULL;
                return;
            }

            /* 发送到数据包到tcpip_thread进行处理 */
            if ( tcpip_input( buffer, tmp_netif ) != ERR_OK )
            {
                LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));

                /* Stop lint warning - packet has not been released in this case */ /*@-usereleased@*/
                result = pbuf_free( buffer );
                /*@+usereleased@*/
                LWIP_ASSERT("Failed to release packet buffer", ( result != (u8_t)0 ) );
                buffer = NULL;
            }
            break;

#ifdef ADD_LWIP_EAPOL_SUPPORT
        case WICED_ETHERTYPE_EAPOL:
            wwd_eapol_receive_eapol_packet( buffer, interface );
            break;
#endif
        default:
            result = pbuf_free( buffer );
            LWIP_ASSERT("Failed to release packet buffer", ( result != (u8_t)0 ) );
            buffer = NULL;
            break;
    }
}

void host_network_set_ethertype_filter( uint16_t ethertype, wwd_interface_t interface, wwd_network_filter_ethernet_packet_t ethernet_packet_callback, void* userdata )
{
    filter_ethertype                = ethertype;
    filter_interface                = interface;
    filter_ethernet_packet_callback = ethernet_packet_callback;
    filter_userdata                 = userdata;
}

/**
 * 应在程序开始时调用以设置
 * 网络接口。它调用函数low_level_init（）来执行
 * 硬件的实际设置。
 *
 * 该函数应作为参数传递给netif_add（）。
 *
 * @param  netif此ethernetif的lwip网络接口结构
 * @return ERR_OK，如果循环初始化
 *         ERR_MEM，如果在出错时无法为私有数据分配任何其他err_t
 */
err_t ethernetif_init( /*@partial@*/ struct netif *netif )
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

    /* Verify the netif is a valid interface */
    if ( (wwd_interface_t) netif->state > WWD_ETHERNET_INTERFACE )
    {
        return ERR_ARG;
    }

    /*
				在结构netif中初始化snmp变量和计数器。 
				最后一个参数应替换为链接速度，以每秒位数为单位。
     */
    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->name[0] = IFNAME0;
    netif->name[1] = IFNAME1;

    /* We directly use etharp_output() here to save a function call.
     * You can instead declare your own function an call etharp_output()
     * from it if you have to do some checks before sending (e.g. if link
     * is available...)
     */
    netif->output = etharp_output;
#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif
    netif->linkoutput = low_level_output;

    /* 初始化硬件 */
    low_level_init( netif );

    return ERR_OK;
}

#if LWIP_IGMP
/**
 *LwIP IGMP MAC过滤器和WICED MAC过滤器之间的接口
 */
static err_t lwip_igmp_mac_filter(struct netif *netif,
       const ip4_addr_t *group, enum netif_mac_filter_action action)
{
    wiced_mac_t mac = { MULTICAST_IP_TO_MAC((uint8_t*)group) };
    /*@-noeffect@*/
    UNUSED_PARAMETER(netif);
    /*@+noeffect@*/

    switch ( action )
    {
        case IGMP_ADD_MAC_FILTER:
            if ( wwd_wifi_register_multicast_address( &mac ) != WWD_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        case IGMP_DEL_MAC_FILTER:
            if ( wwd_wifi_unregister_multicast_address( &mac ) != WWD_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        default:
            return ERR_VAL;
    }

    return ERR_OK;
}
#endif

/********************************** End of file ******************************************/
