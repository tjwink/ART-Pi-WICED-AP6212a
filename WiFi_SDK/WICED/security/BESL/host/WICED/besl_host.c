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

#include <string.h>

#include <stdarg.h>
#include "besl_host_interface.h"
#include "wwd_structures.h"
#include "wiced_utilities.h"
#include "wwd_wifi.h"
#include "wiced_crypto.h"
#include "wwd_network_constants.h"
#include "internal/wwd_sdpcm.h"
#include "besl_host_rtos_structures.h"
#include "RTOS/wwd_rtos_interface.h"
#include "network/wwd_network_interface.h"
#include "network/wwd_buffer_interface.h"
#include "wwd_assert.h"
#include "internal/wwd_internal.h"
#include "supplicant_structures.h"
#include "wwd_network_constants.h"
#include "besl_host_rtos_structures.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define CHECK_IOCTL_BUFFER( buff )  if ( buff == NULL ) {  wiced_assert("Allocation failed\n", 0 == 1); return BESL_BUFFER_ALLOC_FAIL; }

/******************************************************
 *                    Constants
 ******************************************************/

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void* besl_host_malloc( const char* name, uint32_t size )
{
    BESL_DEBUG(("besl_host_malloc: %s %u\n", name, (unsigned int)size));
    return malloc_named( name, size );
}

void* besl_host_calloc( const char* name, uint32_t num, uint32_t size )
{
    BESL_DEBUG(("besl_host_calloc: %s %u %u\n", name, (unsigned int)num, (unsigned int)size));
    return calloc_named(name, num, size );
}

void besl_host_free( void* p )
{
    free( p );
}

besl_result_t besl_host_get_mac_address(besl_mac_t* address, uint32_t interface )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wwd_result_t result;
    uint32_t*      data;

    data = wwd_sdpcm_get_iovar_buffer( &buffer, sizeof(wiced_mac_t) + sizeof(uint32_t), "bsscfg:" IOVAR_STR_CUR_ETHERADDR );
    CHECK_IOCTL_BUFFER( data );
    *data = wwd_get_bss_index( ( wwd_interface_t )interface );

    result = wwd_sdpcm_send_iovar( SDPCM_GET, buffer, &response, WWD_STA_INTERFACE );
    if ( result != WWD_SUCCESS )
    {
        memset( address->octet, 0, sizeof(wiced_mac_t) );
        return (besl_result_t) result;
    }
    memcpy( address, host_buffer_get_current_piece_data_pointer( response ), sizeof(wiced_mac_t) );
    host_buffer_release( response, WWD_NETWORK_RX );

    return BESL_SUCCESS;
}

besl_result_t besl_host_set_mac_address(besl_mac_t* address, uint32_t interface )
{
    wiced_buffer_t buffer;
    uint32_t*      data;

    data = wwd_sdpcm_get_iovar_buffer( &buffer, sizeof(wiced_mac_t) + sizeof(uint32_t), "bsscfg:" IOVAR_STR_CUR_ETHERADDR );
    CHECK_IOCTL_BUFFER( data );
    data[0] = wwd_get_bss_index( ( wwd_interface_t )interface );
    memcpy(&data[1], address, sizeof(wiced_mac_t));

    return (besl_result_t) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE );
}

void besl_host_random_bytes( uint8_t* buffer, uint16_t buffer_length )
{
    wiced_crypto_get_random( buffer, buffer_length );
}

void besl_host_get_time(besl_time_t* time)
{
    *time = (besl_time_t)host_rtos_get_time();
}

uint32_t besl_host_hton32(uint32_t intlong)
{
    return htobe32(intlong);
}

uint16_t besl_host_hton16(uint16_t intshort)
{
    return htobe16(intshort);
}


void besl_host_hex_bytes_to_chars( char* cptr, const uint8_t* bptr, uint32_t blen )
{
    int i,j;
    uint8_t temp;

    i = 0;
    j = 0;
    while( i < blen )
    {
        // Convert first nibble of byte to a hex character
        temp = bptr[i] / 16;
        if ( temp < 10 )
        {
            cptr[j] = temp + '0';
        }
        else
        {
            cptr[j] = (temp - 10) + 'A';
        }
        // Convert second nibble of byte to a hex character
        temp = bptr[i] % 16;
        if ( temp < 10 )
        {
            cptr[j+1] = temp + '0';
        }
        else
        {
            cptr[j+1] = (temp - 10) + 'A';
        }
        i++;
        j+=2;
    }
}

void besl_unit_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

besl_result_t besl_host_create_packet( besl_packet_t* packet, uint16_t size )
{
    wwd_result_t result;
    result = host_buffer_get( (wiced_buffer_t*) packet, WWD_NETWORK_TX, size + WICED_LINK_OVERHEAD_BELOW_ETHERNET_FRAME_MAX, WICED_TRUE );
    if ( result != WWD_SUCCESS )
    {
        *packet = 0;
        return (besl_result_t) result;
    }
    host_buffer_add_remove_at_front( (wiced_buffer_t*) packet, WICED_LINK_OVERHEAD_BELOW_ETHERNET_FRAME_MAX );

    return (besl_result_t) result;
}

void besl_host_consume_bytes( besl_packet_t* packet, int32_t number_of_bytes )
{
    host_buffer_add_remove_at_front( (wiced_buffer_t*) packet, number_of_bytes );
}

uint8_t* besl_host_get_data( besl_packet_t packet )
{
    return host_buffer_get_current_piece_data_pointer( packet );
}

besl_result_t besl_host_set_packet_size( besl_packet_t packet, uint16_t packet_length )
{
    return (besl_result_t) host_buffer_set_size( (wiced_buffer_t) packet, packet_length );
}

uint16_t besl_host_get_packet_size( besl_packet_t packet )
{
    return host_buffer_get_current_piece_size( packet );
}

void besl_host_free_packet( besl_packet_t packet )
{
    host_buffer_release( (wiced_buffer_t) packet, WWD_NETWORK_RX );
}

void besl_host_send_packet( void* workspace, besl_packet_t packet, uint16_t size )
{
    besl_host_workspace_t* host = (besl_host_workspace_t*) workspace;
    host_buffer_set_size( (wiced_buffer_t) packet, size );
    wwd_network_send_ethernet_data( packet, host->interface );
}

besl_result_t besl_host_leave( wwd_interface_t interface )
{
    wwd_wifi_leave( interface );
    return BESL_SUCCESS;
}

void besl_host_start_timer( void* workspace, uint32_t timeout )
{
    besl_host_workspace_t* host = (besl_host_workspace_t*) workspace;
    host->timer_reference = host_rtos_get_time( );
    host->timer_timeout = timeout;
}

void besl_host_stop_timer( void* workspace )
{
    besl_host_workspace_t* host = (besl_host_workspace_t*) workspace;
    host->timer_timeout = 0;
}

uint32_t besl_host_get_current_time( void )
{
    return host_rtos_get_time();
}

uint32_t besl_host_get_timer( void* workspace )
{
    besl_host_workspace_t* host = (besl_host_workspace_t*)workspace;
    return host->timer_timeout;
}

besl_result_t besl_queue_message_packet( void* workspace, besl_event_t type, besl_packet_t packet )
{
    besl_result_t result;
    supplicant_workspace_t* temp = (supplicant_workspace_t*)workspace;
    besl_host_workspace_t* host_workspace = (besl_host_workspace_t*)temp->supplicant_host_workspace;
    besl_event_message_t   message;
    message.event_type = type;
    message.data.packet = packet;
    result = (besl_result_t) host_rtos_push_to_queue( &host_workspace->event_queue, &message, WICED_NEVER_TIMEOUT );
    if ( result != BESL_SUCCESS )
    {
        host_buffer_release( (wiced_buffer_t) packet, WWD_NETWORK_RX );
    }
    return result;
}

besl_result_t besl_queue_message_uint( void* workspace, besl_event_t type, uint32_t value )
{
    besl_host_workspace_t* host_workspace = (besl_host_workspace_t*)workspace;
    besl_event_message_t   message;
    message.event_type = type;
    message.data.value = value;
    return (besl_result_t) host_rtos_push_to_queue( &host_workspace->event_queue, &message, WICED_NEVER_TIMEOUT );
}



void besl_get_bssid( besl_mac_t* mac )
{
    wwd_wifi_get_bssid( (wiced_mac_t*)mac );
}


besl_result_t besl_set_passphrase( const uint8_t* security_key, uint8_t key_length )
{
    return (besl_result_t)wwd_wifi_set_passphrase( security_key, key_length, WWD_STA_INTERFACE );
}
