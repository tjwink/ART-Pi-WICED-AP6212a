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

#include <stdio.h>
#include <string.h>
#include "wifi_cert_traffic_stream.h"
#include "wiced_network.h"
#include "wiced_tcpip.h"
#include "compat.h"
#include "wwd_wlioctl.h"

/*
 * Rate-limit- the number of packets sent in a interval
 *
 */
#define RATE_CHECK_TIME_LIMIT ( 100 ) // Milliseconds
#define LOW_PRIOIRTY_TRAFFIC_SLEEP_TIME (5) // sleep time for lower priority threads


/* Extern Global variables highest Rx/Tx Type of Service and
 * ac_params
 */
extern edcf_acparam_t ac_params[AC_COUNT];

int udp_rx( traffic_stream_t* ts )
{
    int retval =    1;
    wiced_packet_t* packet = NULL;
    uint8_t*        udp_data = NULL;
    uint16_t        offset;
    uint16_t        data_length;
    uint16_t        available_data_length;
    uint32_t        timeout = 1; // Milliseconds
    wiced_ip_address_t target_ip_addr;

    if ( wiced_udp_create_socket( &ts->rx_socket, ts->dest_port, WICED_STA_INTERFACE ) != WICED_SUCCESS )
    {
        printf( "Could not create receive UDP socket\n" );
        return -7;
    }

    if ( str_to_ip( ts->dest_ipaddr, &target_ip_addr ) != WICED_SUCCESS )
    {
        target_ip_addr.version = WICED_IPV4; /* Force to IPv4 in case the dest_ipaddr was terminated with a space rather than a NULL */
    }

    if (ts->profile == PROFILE_MULTICAST)
    {
        if ( wiced_multicast_join( WICED_STA_INTERFACE, &target_ip_addr ) != WICED_SUCCESS )
        {
            printf( "Could not join multicast group\n\n" );
            return -7;
        }
    }

#if ( defined(NETWORK_LwIP) )
    lwip_setsockopt( ts->rx_socket.socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof( uint32_t ) );
#endif

    while ( ts->enabled )
    {
        data_length = 0;
        offset = 0;

        /* Wait for UDP packet */
        wiced_result_t result = wiced_udp_receive(&ts->rx_socket, &packet, timeout );
        if ((result == WICED_ERROR) || (result == WICED_TIMEOUT))
        {
            ;
        }
        else
        {
            do
            {
                wiced_packet_get_data(packet, offset, &udp_data, &data_length, &available_data_length);

                offset += data_length;
            } while (data_length < available_data_length);

            wiced_packet_delete(packet);   /* Delete packet, it is no longer needed */
        }

        if ( offset > 0  && ts->enabled) // Also check the enabled flag since another thread may have set it to 0
        {
            ++ts->frames_received;
            ts->bytes_received += offset;
        }
    }

    if ( wiced_udp_delete_socket( &ts->rx_socket ) != WICED_SUCCESS )
    {
        printf("Could not close UDP socket\n\n");
    }

    return retval;
}


int udp_tx( traffic_stream_t* ts )
{
    wiced_time_t       rate_check_time, current_time, rest_time;
    int                frames_sent_this_period = 0;
    int                frames_required_this_period = 0;
    wiced_packet_t*    packet = NULL;
    char*              udp_data = NULL;
    uint16_t           available_data_length;
    wiced_ip_address_t target_ip_addr;

    if ( wiced_udp_create_socket( &ts->tx_socket, ts->dest_port, WICED_STA_INTERFACE ) != WICED_SUCCESS )
    {
        printf( "Could not create transmit UDP socket\n\n" );
        return -7;
    }

    uint32_t tos_priority[4] = { 0x00, 0x40, 0xA0, 0xE0 }; // Best Effort, Background, Video, Voice

    wiced_udp_set_type_of_service( &ts->tx_socket, tos_priority[ts->ac] );

    if ( str_to_ip( ts->dest_ipaddr, &target_ip_addr ) != WICED_SUCCESS )
    {
        target_ip_addr.version = WICED_IPV4; /* Force to IPv4 in case the dest_ipaddr was terminated with a space rather than a NULL */
    }

    if (ts->profile == PROFILE_MULTICAST)
    {
        if ( wiced_multicast_join( WICED_STA_INTERFACE, &target_ip_addr ) != WICED_SUCCESS )
        {
            printf( "Could not join multicast group\n\n" );
            return -7;
        }
    }

    frames_sent_this_period = 0;
    if ( ts->frame_rate > 0 )
    {
        frames_required_this_period = ( ts->frame_rate * 1000 ) / ( 1000000 / RATE_CHECK_TIME_LIMIT );
    }

    rate_check_time = host_rtos_get_time( ) + RATE_CHECK_TIME_LIMIT; // Need to check periodically whether we have sent required frames and should rest
    while ( ts->enabled )
    {
        if ( host_rtos_get_time( ) > ts->stop_time )
        {
            break;
        }

        /* It is busy loop here. Try to allocate a packet. If available send it out, otherwise keep trying until enough frame is sent.*/
        if (wiced_packet_create_udp_no_wait(&ts->tx_socket, ts->payload_size, &packet, (uint8_t**)&udp_data, &available_data_length) == WICED_SUCCESS)
        {
            /* Set the end of the data portion */
            wiced_packet_set_data_end(packet, (uint8_t*)udp_data + ts->payload_size);

            /* Send the UDP packet */
            if (wiced_udp_send(&ts->tx_socket, &target_ip_addr, ts->dest_port, packet) != WICED_SUCCESS)
            {
                printf("UDP packet send failed\n");
                wiced_packet_delete(packet);  /* Delete packet, since the send failed */
            }
            else
            {
                ++ts->frames_sent;
                ts->bytes_sent += ts->payload_size;
                ++frames_sent_this_period;
            }
        }

        if ( ts->frame_rate > 0 )
        {
            if ( frames_sent_this_period >= frames_required_this_period )
            {
                frames_sent_this_period = 0;
                current_time = host_rtos_get_time( );
                if ( current_time < rate_check_time )
                {
                    rest_time = rate_check_time - current_time;
                }
                else
                {
                    rest_time = 0;
                }
                rate_check_time += RATE_CHECK_TIME_LIMIT;
                if ( ( rest_time > 0 ) && ( rest_time <= RATE_CHECK_TIME_LIMIT ) )
                {
                    host_rtos_delay_milliseconds( rest_time );
                }
            }
            else
            {
                if ( host_rtos_get_time( ) >= rate_check_time )
                {
                    rate_check_time += RATE_CHECK_TIME_LIMIT;
                    frames_sent_this_period = 0;
                }
            }
        }
    }

    if (ts->profile == PROFILE_MULTICAST)
    {
        if ( wiced_multicast_leave( WICED_STA_INTERFACE, &target_ip_addr ) != WICED_SUCCESS )
        {
            printf( "Could not leave multicast group\n\n" );
            return -7;
        }
    }

    return 0;
}

int udp_transactional( traffic_stream_t* ts )
{
    int retval =       1;
    wiced_packet_t*    packet = NULL;
    char*              udp_data = NULL;
    uint16_t           data_length;
    uint16_t           available_data_length;
    uint32_t timeout = 10; // Milliseconds
    wiced_ip_address_t target_ip_addr;

    if ( wiced_udp_create_socket( &ts->rx_socket, ts->dest_port, WICED_STA_INTERFACE ) != WICED_SUCCESS )
    {
        printf( "Could not create receive UDP socket\n" );
        return -7;
    }

    if ( str_to_ip( ts->src_ipaddr, &target_ip_addr ) != WICED_SUCCESS )
    {
        target_ip_addr.version = WICED_IPV4; /* Force to IPv4 in case the dest_ipaddr was terminated with a space rather than a NULL */
    }

    while ( ts->enabled )
    {
        data_length = 0;

        /* Wait for UDP packet */
        wiced_result_t result = wiced_udp_receive(&ts->rx_socket, &packet, timeout );
        if ((result == WICED_ERROR) || (result == WICED_TIMEOUT))
        {
            ;
        }
        else
        {
            wiced_packet_get_data(packet, 0, (uint8_t**)&udp_data, &data_length, &available_data_length);

            if (data_length != available_data_length)
            {
                printf("Fragmented packets not supported\n");
                retval = WICED_ERROR;
            }

            wiced_packet_delete(packet);   /* Delete packet, it is no longer needed */
        }

        if ( data_length > 0  && ts->enabled) // Also check the enabled flag since another thread may have set it to 0
        {
            ++ts->frames_received;
            ts->bytes_received += data_length;

            /* Create the UDP packet */
            if (wiced_packet_create_udp(&ts->rx_socket, data_length, &packet, (uint8_t**)&udp_data, &available_data_length) != WICED_SUCCESS)
            {
                printf("UDP tx packet creation failed\n\n");
                //return WICED_ERROR;
            }
            else
            {
                /* Set the end of the data portion */
                wiced_packet_set_data_end(packet, (uint8_t*)udp_data + data_length);

                /* Send the UDP packet */
                if (wiced_udp_send(&ts->rx_socket, &target_ip_addr, ts->dest_port, packet) != WICED_SUCCESS)
                {
                    printf("UDP packet send failed\n");
                    wiced_packet_delete(packet);  /* Delete packet, since the send failed */
                }
                else
                {
                    ++ts->frames_sent;
                    ts->bytes_sent += ts->payload_size;
                }
            }
        }
    }

    if ( wiced_udp_delete_socket( &ts->rx_socket ) != WICED_SUCCESS )
    {
        printf("Could not close UDP socket\n\n");
    }

    return retval;
}

