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
#include "wiced_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "RTOS/wwd_rtos_interface.h"
#include "command_console.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include "wifi_cert_ping.h"
#include "wwd_constants.h"
#include <stdio.h>
#include <stdlib.h>
#include "wiced_p2p.h"


#define PING_ID              (0xAFAF)

static void wifi_ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len );
static err_t wifi_ping_recv( int socket_hnd );
//static err_t wifi_ping_send( int socket_hnd, struct sockaddr_in *addr, int frame_size );

#define test_print( a ) { printf a; }

static uint16_t      ping_seq_num;
static uint32_t      num_ping_requests = 0;
static uint32_t      num_ping_replies = 0;

/*!
 ******************************************************************************
 * Implementation of traffic send ping (with stripped down set of arguments)
 * Sends ICMP pings to the indicated host or IP address
 * argv[0]: traffic_send_ping
 * argv[1]: IP address of target
 * argv[2]: frame size
 * argv[3]: frame rate
 * argv[4]: duration
 * @return  0 for success, otherwise error
 */

wiced_result_t wifi_traffic_send_ping( void *arg )
{
    struct hostent * host;
    struct sockaddr_in host_addr;
    wwd_time_t send_time;
    wwd_time_t recv_time;
//    wiced_time_t end_time;
    int socket_hnd;
    char** argv = arg;

    num_ping_requests = 0; // Global count of the number of ping requests for the latest call of this function
    num_ping_replies = 0; // Global count of the number of ping replies for the latest call of this function

    host = gethostbyname( argv[2] );
    int frame_size = atoi(argv[4]);
    int frame_rate = atoi(argv[6]); // Frames per second

    // XXX Some scripts send a decimal frame rate even though the variable is defined as a short int. This fix may not work for all test plans.
    if ( frame_rate == 0 )
    {
        frame_rate = 1;
    }

    //printf("entering ping, frame rate %d\n", frame_rate);

    int duration = atoi(argv[8]);  // How long to ping for in seconds

    int frame_interval = 1000 / frame_rate;
    int wait_time = 0;
    int num_frames = frame_rate * duration;
    int sleep_time = 0;

    if ( host == NULL )
    {
        printf("host is null\n");
        return -1;
    }

    host_addr.sin_addr.s_addr = *((uint32_t*) host->h_addr_list[0]);
    host_addr.sin_len = sizeof( host_addr );
    host_addr.sin_family = AF_INET;

    /* Open a local socket for pinging */
    if ( ( socket_hnd = lwip_socket( AF_INET, SOCK_RAW, IP_PROTO_ICMP ) ) < 0 )
    {
        return -1;
    }

    /* Set the receive timeout on local socket so pings will time out. */
//    if ( ( num_frames < 5 ) && besl_p2p_group_owner_is_up() == WICED_TRUE )
//    {
//        wait_time = 15000; // To cope with STAs in power save
//    }
//    else
//    {
        wait_time = frame_interval - 5;
//    }
    lwip_setsockopt( socket_hnd, SOL_SOCKET, SO_RCVTIMEO, &wait_time, sizeof( wait_time ) );
    //sys_msleep( 10 );

//    end_time = host_rtos_get_time( ) + ( duration * 1000 );

    struct icmp_echo_hdr *iecho;
    size_t ping_size = sizeof(struct icmp_echo_hdr) + frame_size;

    /* Allocate memory for packet */
    if ( !( iecho = calloc( 1, ping_size ) ) )
    {
        printf("calloc failed\n");
        return ERR_MEM;
    }

    /* Construct ping request */
    wifi_ping_prepare_echo( iecho, ping_size );

    //printf("about to ping\n");

    /* Loop forever */
    while ( num_frames > 0 )
    {
//        printf ("Pinging: %u.%u.%u.%u\n", (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 24 ) & 0xff ),
//                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >> 16 ) & 0xff ),
//                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  8 ) & 0xff ),
//                                            (unsigned char) ( ( htonl( host_addr.sin_addr.s_addr ) >>  0 ) & 0xff ) );

        lwip_sendto( socket_hnd, iecho, ping_size, 0, (struct sockaddr*) &host_addr, host_addr.sin_len );

        ++num_ping_requests;

        /* Record time ping was sent */
        send_time = host_rtos_get_time( );

        /* Wait for ping reply */
        err_t result = wifi_ping_recv( socket_hnd );

        if ( ERR_OK == result )
        {
            //printf( "Ping Reply %dms\n", (int)( host_rtos_get_time( ) - send_time ) );
            ++num_ping_replies;
        }
        else
        {
            //printf( "Ping timeout\n" );
            ;
        }
        recv_time = host_rtos_get_time( );

        /* Sleep until time for next ping */
        sleep_time = frame_interval - ( recv_time - send_time );
        if ( sleep_time > 0 )
        {
            sys_msleep( sleep_time );
        }

        --num_frames;
    }

    if ( 0 != lwip_close( socket_hnd ) )
    {
        printf( "status,ERROR\n");
        return -1;
    }

    /* free packet */
    mem_free( iecho );

    return 0;
}

/*!
 ******************************************************************************
 * Return value of num_ping_requests and num_ping_replies
 * @return  0 for success, otherwise error
 */

int wifi_traffic_stop_ping( void )
{
    printf("status,COMPLETE,sent,%u,replies,%u\n", (unsigned int)num_ping_requests,(unsigned int)num_ping_replies);

    return ERR_CMD_OK;
}


/**
 *  Prepare the contents of an echo ICMP request packet
 *
 *  @param iecho  : Pointer to an icmp_echo_hdr structure in which the ICMP packet will be constructed
 *  @param len    : The length in bytes of the packet buffer passed to the iecho parameter
 *
 */

static void wifi_ping_prepare_echo( struct icmp_echo_hdr *iecho, uint16_t len )
{
    int i;

    ICMPH_TYPE_SET( iecho, ICMP_ECHO );
    ICMPH_CODE_SET( iecho, 0 );
    iecho->chksum = 0;
    iecho->id = PING_ID;
    iecho->seqno = htons( ++ping_seq_num );

    /* fill the additional data buffer with some data */
    for ( i = 0; i < (len - sizeof(struct icmp_echo_hdr)); i++ )
    {
        ( (char*) iecho )[sizeof(struct icmp_echo_hdr) + i] = i;
    }

    iecho->chksum = inet_chksum( iecho, len );
}



/**
 *  Send a Ping
 *
 *  Sends a ICMP echo request (Ping) to the specified IP address, using the specified socket.
 *
 *  @param socket_hnd : The handle for the local socket through which the ping request will be sent
 *  @param addr       : The IP address to which the ping request will be sent
 *
 *  @return  ERR_OK if successfully sent, ERR_MEM if out of memory or ERR_VAL otherwise
 */

//static err_t wifi_ping_send( int socket_hnd, struct sockaddr_in *addr, int frame_size )
//{
//    int err;
////    char buf[1500];
//    struct icmp_echo_hdr *iecho;
//    size_t ping_size = sizeof(struct icmp_echo_hdr) + frame_size;
//
//    /* Allocate memory for packet */
//    if ( !( iecho = calloc( 1, ping_size ) ) )
//    {
//        return ERR_MEM;
//    }
//
//    /* Construct ping request */
//    wifi_ping_prepare_echo( iecho, ping_size );
//
//    /* Send the ping request */
//
//    err = lwip_sendto( socket_hnd, iecho, ping_size, 0, (struct sockaddr*) addr, addr->sin_len );
//
//    /* free packet */
//    mem_free( iecho );
//
//    return ( err ? ERR_OK : ERR_VAL );
//}


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

static err_t wifi_ping_recv( int socket_hnd )
{
    char buf[1500];
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

