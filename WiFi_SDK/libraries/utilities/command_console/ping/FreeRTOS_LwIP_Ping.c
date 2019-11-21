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

#include "lwip/opt.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/inet.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "wiced_network.h"
#include "wiced_management.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "RTOS/wwd_rtos_interface.h"
#include "command_console.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "wiced_time.h"
#include "lwip/prot/ip.h"
#include <math.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define PING_RCV_TIMEO       (1000)    /* ping receive timeout - in milliseconds */
#define PING_ID              (0xAFAF)
#define PING_MAX_PAYLOAD_SIZE ( 10000 ) /* ping max size */

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len );
static err_t ping_recv( int socket_hnd );


/******************************************************
 *               Variable Definitions
 ******************************************************/
static uint16_t      ping_seq_num;


/******************************************************
 *               Function Definitions
 ******************************************************/

/*!
 ******************************************************************************
 * Sends an ICMP ping to the indicated host or IP address
 *
 * @return  0 for success, otherwise error
 */

int ping( int argc, char *argv[] )
{
    struct hostent * host;
    struct sockaddr_in host_addr;
    int socket_hnd;
    struct timeval recv_timeout;
    int tx_pkt = 0, pkt_loss = 0;

    if ( argc == 1 )
    {
        return ERR_INSUFFICENT_ARGS;
    }

    int i        = 0;
    int len      = 100;
    int num      = 1;
    int interval = 1000;
    wiced_bool_t continuous = WICED_FALSE;

    host = gethostbyname( argv[1] );

    if ( host == NULL )
    {
        WPRINT_APP_INFO(( "Could not find host %s\n", argv[1] ));
        return ERR_UNKNOWN;
    }

    host_addr.sin_addr.s_addr = *((uint32_t*) host->h_addr_list[0]);
    host_addr.sin_len = sizeof( host_addr );
    host_addr.sin_family = AF_INET;

    /* Open a local socket for pinging */
    if ( ( socket_hnd = lwip_socket( AF_INET, SOCK_RAW, IP_PROTO_ICMP ) ) < 0 )
    {
        WPRINT_APP_INFO(( "unable to create socket for Ping\r\n" ));
        return ERR_UNKNOWN;
    }

    /* Set the receive timeout on local socket so pings will time out. */
    recv_timeout.tv_sec = PING_RCV_TIMEO / 1000U;
    recv_timeout.tv_usec = (PING_RCV_TIMEO % 1000U) * 1000U;
    lwip_setsockopt( socket_hnd, SOL_SOCKET, SO_RCVTIMEO, &recv_timeout, sizeof( recv_timeout ) );

    WPRINT_APP_INFO (("PING %u.%u.%u.%u\r\n", (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 24 ) & 0xff ),
                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 16 ) & 0xff ),
                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  8 ) & 0xff ),
                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  0 ) & 0xff ) ));

    for (i = 2; i < argc; i++ )
    {
        switch (argv[i][1])
        {
            case 'i':
                interval = atoi(argv[i+1]);
                if ( interval < 0 )
                {
                    WPRINT_APP_INFO(("min interval 0\n\r"));
                    return ERR_CMD_OK;
                }
                WPRINT_APP_INFO(("interval: %d milliseconds\n\r", interval));
                i++;
                break;

            case 'l':
                len = atoi(argv[i+1]);
                if ( ( len > PING_MAX_PAYLOAD_SIZE ) || ( len < 0 ) )
                {
                    WPRINT_APP_INFO(("max ping length: %d, min: 0\n\r", PING_MAX_PAYLOAD_SIZE));
                    return ERR_CMD_OK;
                }
                WPRINT_APP_INFO(("length: %d\n\r", len));
                i++;
                break;

            case 'n':
                num = atoi(argv[i+1]);
                if ( num < 1 )
                {
                    WPRINT_APP_INFO(("min number of packets 1\n\r"));
                    return ERR_CMD_OK;
                }
                WPRINT_APP_INFO(("number : %d\n\r", num));
                i++;
                break;

            case 't':
                continuous = WICED_TRUE;
                WPRINT_APP_INFO(("continuous...\n\r"));
                break;

            default:
                WPRINT_APP_INFO(("Not supported, ignoring: %s\n\r", argv[i]));
            break;
        }
    }

    struct icmp_echo_hdr *iecho;
    size_t ping_size = sizeof(struct icmp_echo_hdr) + len;

    /* Allocate memory for packet */
    if ( !( iecho = mem_malloc( ping_size ) ) )
    {
        return ERR_MEM;
    }

    /* Construct ping request */
    ping_prepare_echo( iecho, ping_size );

    wiced_time_t send_time;
    wiced_time_t reply_time;
    wiced_time_t overall_time = 0;
    wiced_time_t overall_time2 = 0;
    wiced_time_t rtt_min = ~0, rtt_max = 0;

    while (( num > 0 ) || ( continuous == WICED_TRUE ) )
    {
        /* Send ping and wait for reply */
        send_time = host_rtos_get_time( );

        /* Set default netif so that direct ping packets will be routed correctly if tethered to an AP and also running SoftAP. */
        wiced_rtos_lock_mutex( &lwip_send_interface_mutex );
        netif_set_default( wiced_ip_handle[0]);

        if ( lwip_sendto( socket_hnd, iecho, ping_size, 0, (struct sockaddr*) &host_addr, host_addr.sin_len ) > 0 )
        {
            wiced_rtos_unlock_mutex( &lwip_send_interface_mutex );
            /* Wait for ping reply */
            err_t result = ping_recv( socket_hnd );
            reply_time = host_rtos_get_time( );
            if ( ERR_OK == result )
            {
                wiced_time_t ping_time = reply_time - send_time;
                WPRINT_APP_INFO( ("Ping Reply %dms\r\n", (int) ping_time ) );
                overall_time += ping_time;
                overall_time2 += ping_time * ping_time;
                if ( rtt_min > ping_time )
                {
                    rtt_min = ping_time;
                }
                if ( rtt_max < ping_time )
                {
                    rtt_max = ping_time;
                }

            }
            else
            {
                WPRINT_APP_INFO( ("Ping timeout\r\n") );
                pkt_loss++;
            }
        }
        else
        {
            wiced_rtos_unlock_mutex( &lwip_send_interface_mutex );
            reply_time = host_rtos_get_time( );
            WPRINT_APP_INFO(("Ping error\r\n"));
            pkt_loss++;
        }

        num--;
        tx_pkt++;
        if ( ( ( num > 0 ) || ( continuous == WICED_TRUE ) ) && ( ( send_time + interval ) > reply_time ) )
        {
            wiced_rtos_delay_milliseconds( ( send_time + interval ) - reply_time );
        }
    }

    // Free the packet
    mem_free( iecho );

    if ( 0 != lwip_close( socket_hnd ) )
    {
        WPRINT_APP_INFO( ("Could not close ping socket\r\n") );
        return ERR_UNKNOWN;
    }

    wiced_time_t rtt_mdev = 0;
    float rtt_avg = 0.0;
    int received = tx_pkt - pkt_loss;
    if( received )
    {
        rtt_mdev = sqrt(overall_time2/(received) - overall_time * overall_time/(received * received));
        rtt_avg = overall_time/received;
    }
    WPRINT_APP_INFO (("--- %u.%u.%u.%u ping statistics ---\r\n", (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 24 ) & 0xff ),
                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 16 ) & 0xff ),
                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  8 ) & 0xff ),
                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  0 ) & 0xff ) ));

    WPRINT_APP_INFO(("%u packets transmitted, %u received, %d%% packet loss, time %dms\r\n",
                     tx_pkt, received, (pkt_loss*100)/tx_pkt, (int)overall_time));
    WPRINT_APP_INFO(("rtt min/avg/max/mdev = %d/%.1f/%d/%d ms\r\n",
                     received?(int)rtt_min:0, rtt_avg,
                     (int)rtt_max, (int)rtt_mdev));

    return ERR_CMD_OK;
}



/**
 *  Prepare the contents of an echo ICMP request packet
 *
 *  @param iecho  : Pointer to an icmp_echo_hdr structure in which the ICMP packet will be constructed
 *  @param len    : The length in bytes of the packet buffer passed to the iecho parameter
 *
 */

static void ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len )
{
    int i, payload_size;

    payload_size = len - sizeof(struct icmp_echo_hdr);

    ICMPH_TYPE_SET( iecho, ICMP_ECHO );
    ICMPH_CODE_SET( iecho, 0 );
    iecho->chksum = 0;
    iecho->id = PING_ID;
    iecho->seqno = htons( ++ping_seq_num );

    /* fill the additional data buffer with some data */
    for ( i = 0; i < payload_size; i++ )
    {
        ( (char*) iecho )[sizeof(struct icmp_echo_hdr) + i] = i;
    }

    iecho->chksum = inet_chksum( iecho, len );
}


/**
 *  Receive a Ping reply
 *
 *  Waits for a ICMP echo reply (Ping reply) to be received using the specified socket. Compares the
 *  sequence number, and ID number to the last ping sent, and if they match, returns ERR_OK, indicating
 *  a valid ping response.
 *
 *  @param socket_hnd : The handle for the local socket through which the ping reply will be received
 *
 *  @return  ERR_OK if valid reply received, ERR_TIMEOUT otherwise
 */

static err_t ping_recv( int socket_hnd )
{
    char buf[64];
    int fromlen, len;
    struct sockaddr_in from;
    struct ip_hdr *iphdr;
    struct icmp_echo_hdr *iecho;

    while ( ( len = lwip_recvfrom( socket_hnd, buf, sizeof( buf ), 0, (struct sockaddr*) &from, (socklen_t*) &fromlen ) ) > 0 )
    {
        if ( len >= (int) ( sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr) ) )
        {
            iphdr = (struct ip_hdr *) buf;
            iecho = (struct icmp_echo_hdr *) ( buf + ( IPH_HL( iphdr ) * 4 ) );

            if ( ( iecho->id == PING_ID ) &&
                 ( iecho->seqno == htons( ping_seq_num ) ) &&
                 ( ICMPH_TYPE( iecho ) == ICMP_ER ) )
            {
                return ERR_OK; /* Echo reply received - return success */
            }
        }
    }

    return ERR_TIMEOUT; /* No valid echo reply received before timeout */
}

