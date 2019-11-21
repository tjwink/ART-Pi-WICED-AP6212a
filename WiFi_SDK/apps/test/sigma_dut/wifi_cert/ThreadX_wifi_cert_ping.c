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

#include "compat.h"
#include "tx_thread.h"
#include "wiced_management.h"
#include "wiced_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "network/wwd_buffer_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_network_constants.h"
#include "wwd_network.h"
#include "wiced_network.h"
#include "command_console.h"
#include "wiced_p2p.h"
#include "wiced_time.h"

static uint32_t      num_ping_requests = 0;
static uint32_t      num_ping_replies = 0;


wiced_result_t wifi_traffic_send_ping( void *arg )
{
    wiced_ip_address_t ping_target;
    NX_PACKET *response_ptr;
    char ping_payload[WICED_PAYLOAD_MTU];
    wiced_time_t send_time;
    wiced_time_t recv_time;
    wiced_time_t finish_time;
    uint32_t interface = 0;
    UINT status;
    char** argv = arg;

    int frame_size = atoi(argv[4]);
    int frame_rate = atoi(argv[6]); // Frames per second

    // XXX Some scripts send a decimal frame rate even though the variable is defined as a short int. This fix may not work for all test plans.
    if ( frame_rate == 0 )
    {
        frame_rate = 1;
    }

    int duration = atoi(argv[8]);  // How long to ping for in seconds
    int frame_interval = 1000 / frame_rate;
    int num_frames = (frame_rate * duration);
    uint32_t wait_time = 0;

    num_ping_requests = 0; // Global count of the number of ping requests for the latest call of this function
    num_ping_replies = 0; // Global count of the number of ping replies for the latest call of this function
    if ( str_to_ip( argv[2], &ping_target ) != WICED_SUCCESS )
    {
        ping_target.version = WICED_IPV4; /* Force to IPv4 in case the IP address was terminated with a space rather than a NULL */
    }

    //printf("entering ping\n");
    // This is limited to one interface being up
    if (wwd_wifi_is_ready_to_transceive(WICED_STA_INTERFACE) == WWD_SUCCESS)
    {
        interface = WICED_STA_INTERFACE;
    }
    else if (wwd_wifi_is_ready_to_transceive(WICED_AP_INTERFACE) == WWD_SUCCESS)
    {
        interface = WICED_AP_INTERFACE;
    }
    else if (wwd_wifi_is_ready_to_transceive(WICED_P2P_INTERFACE) == WWD_SUCCESS)
    {
        interface = WICED_P2P_INTERFACE;
    }
    else
    {
        return 0;
    }

    if ( ( num_frames < 5 ) && besl_p2p_group_owner_is_up() == WICED_TRUE )
    {
        wait_time = 15000; // To cope with STAs in power save
    }
    else
    {
        wait_time = (uint32_t)(frame_interval - 5);
    }


    // Clear the NetX ARP pipe by sending a sacrificial ping of the required size first and then waiting 100 ms
    status = nx_icmp_ping( &IP_HANDLE(interface), ping_target.ip.v4, ping_payload, frame_size, &response_ptr, 100 );
    if ( status == NX_SUCCESS )
    {
        nx_packet_release( response_ptr );
    }
    host_rtos_delay_milliseconds( 100 );

    finish_time = host_rtos_get_time( ) + ( duration * 1000 );

    while ( (num_frames > 0 ) && ( host_rtos_get_time( ) < finish_time ) )
    {
        ++num_ping_requests;
        send_time = host_rtos_get_time( );
        status = nx_icmp_ping( &IP_HANDLE(interface), ping_target.ip.v4, ping_payload, frame_size, &response_ptr, wait_time * SYSTICK_FREQUENCY / 1000 );
        recv_time = host_rtos_get_time( );

        /* Print result */
        if ( status == NX_SUCCESS )
        {
            ++num_ping_replies;
            nx_packet_release( response_ptr );
        }
        else
        {
            ;//printf(" ping not ok 0x%x\n", (unsigned int) status);
        }

        /* Sleep until time for next ping */
        if ( ( recv_time - send_time ) < frame_interval )
        {
            host_rtos_delay_milliseconds( frame_interval - ( recv_time - send_time ) );
        }

        --num_frames; // Decrement frames to be sent
    }

    host_rtos_delay_milliseconds( 10 );

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

