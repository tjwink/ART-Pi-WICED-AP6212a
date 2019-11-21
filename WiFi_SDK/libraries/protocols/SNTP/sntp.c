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

/** @file
 *
 */
#include "sntp.h"
#include "wiced.h"
#include "wiced_crypto.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define NTP_EPOCH            (86400U * (365U * 70U + 17U))
#define NTP_PORT             123

#ifndef WICED_NTP_REPLY_TIMEOUT
#define WICED_NTP_REPLY_TIMEOUT 300
#endif

#define MAX_NTP_ATTEMPTS     3
#define TIME_BTW_ATTEMPTS    5000
/* RFC4330 recommends min 15s between polls */
#define MIN_POLL_INTERVAL    15 * 1000

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/*
 * Taken from RFC 1305
 * http://www.ietf.org/rfc/rfc1305.txt
 */
typedef struct
{
    unsigned int mode : 3;
    unsigned int vn   : 3;
    unsigned int li   : 2;
    uint8_t      stratum;
    int8_t       poll;
    uint32_t     root_delay;
    uint32_t     root_dispersion;
    uint32_t     reference_identifier;
    uint32_t     reference_timestamp_seconds;
    uint32_t     reference_timestamp_fraction;
    uint32_t     originate_timestamp_seconds;
    uint32_t     originate_timestamp_fraction;
    uint32_t     receive_timestamp_seconds;
    uint32_t     receive_timestamp_fraction;
    uint32_t     transmit_timestamp_seconds;
    uint32_t     transmit_timestamp_fraction;
} ntp_packet_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t sync_ntp_time( void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_timed_event_t sync_ntp_time_event;
/* Only support primary and secondary servers */
static wiced_ip_address_t ntp_server[2];

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t sntp_start_auto_time_sync( uint32_t interval_ms )
{
    wiced_result_t result;
    uint8_t random_initial;

    /* Synchronize time with NTP server and schedule for re-sync every one day */
    wiced_crypto_get_random(&random_initial, 1);
    /* prevent thundering herd scenarios by randomizing per RFC4330 */
    wiced_rtos_delay_milliseconds(300 * (unsigned int)random_initial);
    result = sync_ntp_time( NULL );
    if (interval_ms < MIN_POLL_INTERVAL)
        interval_ms = MIN_POLL_INTERVAL;
    wiced_rtos_register_timed_event( &sync_ntp_time_event, WICED_NETWORKING_WORKER_THREAD, sync_ntp_time, interval_ms, 0 );
    return result;
}

wiced_result_t sntp_set_server_ip_address(uint32_t index, wiced_ip_address_t ip_address)
{
    if ((index != 0) && (index != 1))
        return WICED_BADARG;
    if ((ip_address.version != 4) && (ip_address.version != 6))
        return WICED_BADARG;

    ntp_server[index] = ip_address;
    return WICED_SUCCESS;
}

wiced_result_t sntp_clr_server_ip_address(uint32_t index)
{
    if (index > 1)
        return WICED_BADARG;

    ntp_server[index].version = 0;
    return WICED_SUCCESS;
}

wiced_result_t sntp_stop_auto_time_sync( void )
{
    return wiced_rtos_deregister_timed_event( &sync_ntp_time_event );
}

wiced_result_t sntp_get_time( const wiced_ip_address_t* address, ntp_timestamp_t* timestamp)
{
    wiced_udp_socket_t socket;
    wiced_packet_t*    packet;
    ntp_packet_t*      data;
    uint16_t           data_length;
    uint16_t           available_data_length;
    wiced_utc_time_t   utc_time;
    wiced_result_t     result;
    uint32_t           client_sent_timestamp;

    /* Create the query packet */
    memset( &socket, 0, sizeof(wiced_udp_socket_t));
    result = wiced_packet_create_udp( &socket, sizeof(ntp_packet_t), &packet, (uint8_t**) &data, &available_data_length );
    if ( ( result != WICED_SUCCESS ) || ( available_data_length < sizeof(ntp_packet_t) ) )
    {
        return WICED_ERROR;
    }

    /* Fill packet contents */
    wiced_time_get_utc_time( &utc_time );
    memset( data, 0, sizeof(ntp_packet_t) );
    data->li                         = 3;
    data->vn                         = 3;
    data->mode                       = 3;
    data->poll                       = 17;
    data->transmit_timestamp_seconds = htobe32( utc_time + NTP_EPOCH );
    client_sent_timestamp            = data->transmit_timestamp_seconds;
    wiced_packet_set_data_end( packet, (uint8_t*) data + sizeof(ntp_packet_t) );

    /* Create the UDP socket and send request */
    if ( wiced_udp_create_socket( &socket, NTP_PORT, WICED_STA_INTERFACE ) != WICED_SUCCESS )
    {
        wiced_packet_delete( packet );
        return WICED_ERROR;
    }
    if ( wiced_udp_send( &socket, address, NTP_PORT, packet ) != WICED_SUCCESS )
    {
        wiced_packet_delete( packet );
        wiced_udp_delete_socket( &socket );
        return WICED_ERROR;
    }

    /* Wait for reply */
    result = wiced_udp_receive(&socket, &packet, WICED_NTP_REPLY_TIMEOUT);

    /* Close the socket */
    wiced_udp_delete_socket( &socket );

    /* Process received time stamp */
    if (result != WICED_SUCCESS)
    {
        return result;
    }
    result = wiced_packet_get_data( packet, 0, (uint8_t**) &data, &data_length, &available_data_length );
    if ((result == WICED_SUCCESS) && (data_length >= sizeof(ntp_packet_t)))
    {
        if (client_sent_timestamp != data->originate_timestamp_seconds)
        {
            WPRINT_LIB_INFO( (" Server Returned Bad Originate TimeStamp \n" ));
            wiced_packet_delete( packet );
            return WICED_ERROR;
        }
        if (data->li > 2 || data->vn != 3 || data->stratum == 0 || data->stratum > 15 || data->transmit_timestamp_seconds == 0 || data->mode != 4)
        {
            WPRINT_LIB_INFO( (" Invalid Protocol Parameters returned n"));
            wiced_packet_delete( packet );
            return WICED_ERROR;
        }
        timestamp->seconds      = htobe32( data->transmit_timestamp_seconds ) - NTP_EPOCH;
        timestamp->microseconds = htobe32( data->transmit_timestamp_fraction ) / 4295; /* 4295 = 2^32 / 10^6 */
    }
    else if ( data_length < available_data_length )
    {
        /* Fragmented packets not supported */
        wiced_packet_delete( packet );
        return WICED_ERROR;
    }
    else
    {
        wiced_packet_delete( packet );
        return WICED_PACKET_BUFFER_CORRUPT;
    }

    wiced_packet_delete( packet );
    return WICED_SUCCESS;
}

static wiced_result_t sync_ntp_time( void* arg )
{
    ntp_timestamp_t      current_time;
    wiced_ip_address_t   ntp_server_ip;
    wiced_iso8601_time_t iso8601_time;
    uint32_t             a;

    UNUSED_PARAMETER( arg );

    /* Get the time */
    WPRINT_LIB_DEBUG( ( "Getting NTP time... ") );

    for ( a = 0; a < MAX_NTP_ATTEMPTS; ++a )
    {
        wiced_result_t result = WICED_ERROR;

        /* First check if there are local servers to use */
        if (ntp_server[0].version != 0)
        {
            WPRINT_LIB_DEBUG( ( "Sending request primary ...") );
            result = sntp_get_time ( &ntp_server[0], &current_time );
        }
        if (result != WICED_SUCCESS && (ntp_server[1].version != 0))
        {
            WPRINT_LIB_DEBUG( ( "Sending request secondary ...") );
            result = sntp_get_time ( &ntp_server[1], &current_time );
        }
        /* only fall back to global servers if we can't get local */
        if (result != WICED_SUCCESS)
        {
            if ( wiced_hostname_lookup( "pool.ntp.org", &ntp_server_ip, 3 * SECONDS, WICED_STA_INTERFACE ) == WICED_SUCCESS )
            {
                WPRINT_LIB_DEBUG( ( "Sending global request ... ") );
                result = sntp_get_time( &ntp_server_ip, &current_time );
            }
        }

        if ( result == WICED_SUCCESS )
        {
            WPRINT_LIB_DEBUG( ( "success\n" ) );
            break;
        }
        else
        {
            /*
             * This is a bad thing to do here. This will put the network worker thread to
             * sleep for TIME_BTW_ATTEMPTS milliseconds (currently 5 seconds).
             */

            wiced_rtos_delay_milliseconds(TIME_BTW_ATTEMPTS);
            WPRINT_LIB_DEBUG( ( "\nfailed, trying again...\n" ) );
        }
    }

    if ( a >= MAX_NTP_ATTEMPTS )
    {
        WPRINT_LIB_DEBUG( ("Give up getting NTP time\n") );
        memset( &current_time, 0, sizeof( current_time ) );
        return WICED_TIMEOUT;
    }
    else
    {
        wiced_utc_time_ms_t utc_time_ms = (uint64_t)current_time.seconds * (uint64_t)1000 + ( current_time.microseconds / 1000 );

        /* Set & Print the time */
        wiced_time_set_utc_time_ms( &utc_time_ms );
    }

    wiced_time_get_iso8601_time( &iso8601_time );
    WPRINT_LIB_INFO( ("Current time is: %.27s\n", (char*)&iso8601_time) );
    return WICED_SUCCESS;
}
