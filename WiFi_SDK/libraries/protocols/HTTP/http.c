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
#include "http.h"
#include "wiced_tcpip.h"
#include "wiced_tls.h"
#include "wwd_debug.h"
#include "wiced_utilities.h"

/******************************************************
 *                      Macros
 ******************************************************/

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

wiced_result_t wiced_http_get( wiced_ip_address_t* address, const char* query, void* buffer, uint32_t buffer_length )
{
    wiced_tcp_socket_t socket;
    wiced_packet_t*    reply_packet;
    wiced_result_t     result = WICED_ERROR;
    wiced_result_t     rx_result;
    char *             buffer_ptr = (char*) buffer;

    if ( ( result = wiced_tcp_create_socket( &socket, WICED_STA_INTERFACE ) ) != WICED_SUCCESS )
    {
        return result;
    }
    result = wiced_tcp_connect( &socket, address, 80, 10000 );
    if ( result != WICED_SUCCESS )
    {
        wiced_tcp_delete_socket( &socket );
        return ( result );
    }

    if ( ( result = wiced_tcp_send_buffer( &socket, query, (uint16_t) strlen( query ) ) != WICED_SUCCESS ) )
    {
        wiced_tcp_disconnect( &socket );
        wiced_tcp_delete_socket( &socket );
        return ( result );
    }

    WPRINT_LIB_INFO( ("waiting for HTTP reply\n") );

    do
    {
        rx_result = wiced_tcp_receive( &socket, &reply_packet, 5000 );
        if ( rx_result == WICED_SUCCESS )
        {
            uint8_t* data;
            uint16_t offset;
            uint16_t data_length;
            uint16_t available;
            uint32_t data_to_copy;

            /* Record the fact we received a reply of some kind */
            result = WICED_SUCCESS;

            /* Copy data into provided buffer */
            offset = 0;
            do
            {
                wiced_packet_get_data( reply_packet, offset, &data, &data_length, &available );
                data_to_copy = MIN(data_length, buffer_length);
                memcpy( buffer_ptr, data, data_to_copy );
                buffer_ptr    += data_to_copy;
                buffer_length -= data_to_copy;
                offset  = (uint16_t)(offset + data_length);
            } while (data_length < available);
            wiced_packet_delete( reply_packet );
        }
    } while ( rx_result == WICED_SUCCESS );

    wiced_tcp_disconnect( &socket );
    wiced_tcp_delete_socket( &socket );

    return ( result );
}

wiced_result_t wiced_https_get( wiced_ip_address_t* address, const char* query, void* buffer, uint32_t buffer_length, const char* peer_cn )
{
    wiced_tcp_socket_t         socket;
    wiced_packet_t*            reply_packet;
    wiced_result_t             rx_result;
    wiced_tls_context_t        *context;
    wiced_result_t             result;
    char*                      buffer_ptr = (char*) buffer;
    wiced_tls_identity_t*      tls_identity = NULL;

    platform_dct_security_t* dct_security = NULL;

    /*
     * Certificates are loaded into library [and not configured from application]. The newer version of the HTTP client library implementation provides
     *  support for configuring certificates from application.
     */
    /* Lock the DCT to allow us to access the certificate and key */
    WPRINT_APP_INFO(( "Read the certificate Key from DCT\n" ));
    result = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
    if ( result != WICED_SUCCESS )
    {
         WPRINT_APP_INFO(("Unable to lock DCT to read certificate\n"));
         result = WICED_ERROR;
         goto cleanup;
    }

    if(dct_security->private_key[0] != '\0' && dct_security->certificate[0] != '\0')
    {
        tls_identity = ( wiced_tls_identity_t* ) malloc_named ("tls_identity", sizeof( wiced_tls_identity_t ));

        if ( tls_identity == NULL )
        {
            wiced_dct_read_unlock( dct_security, WICED_FALSE );
            result = WICED_OUT_OF_HEAP_SPACE;
            goto cleanup;
        }

        /* Setup TLS identity */
        result = wiced_tls_init_identity( tls_identity, dct_security->private_key, strlen( dct_security->private_key ), (uint8_t*) dct_security->certificate, strlen( dct_security->certificate ) );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(( "Unable to initialize TLS identity. Error = [%d]\n", result ));
            wiced_dct_read_unlock( dct_security, WICED_FALSE );
            result = WICED_ERROR;
            goto cleanup;
        }
    }

    /* Finished accessing the certificates */
    result = wiced_dct_read_unlock( dct_security, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "DCT Read Unlock Failed. Error = [%d]\n", result ));
        goto cleanup;
    }

    if ( ( result = wiced_tcp_create_socket( &socket, WICED_STA_INTERFACE ) ) != WICED_SUCCESS )
    {
        goto cleanup;
    }

    context = MALLOC_OBJECT( "HTTP Client TLS Context", wiced_tls_context_t );

    if ( context == NULL )
    {
        result = WICED_OUT_OF_HEAP_SPACE;
        wiced_tcp_delete_socket( &socket );
        goto cleanup;
    }

    socket.context_malloced = WICED_TRUE;

    wiced_tls_init_context( context, tls_identity, peer_cn );

    wiced_tcp_enable_tls( &socket, context );

    result = wiced_tcp_connect( &socket, address, 443, 20000 );
    if ( result != WICED_SUCCESS )
    {
        wiced_tcp_delete_socket( &socket );
        goto cleanup;
    }

    if ( ( result = wiced_tcp_send_buffer( &socket, query, (uint16_t) strlen( query ) ) != WICED_SUCCESS ) )
    {
        wiced_tcp_disconnect( &socket );
        wiced_tcp_delete_socket( &socket );
        goto cleanup;
    }

    WPRINT_LIB_INFO( ("waiting for HTTP reply\n") );

    do
    {
        rx_result = wiced_tcp_receive( &socket, &reply_packet, 5000 );
        if ( rx_result == WICED_SUCCESS )
        {
            uint8_t* data;
            uint16_t offset;
            uint16_t data_length;
            uint16_t available;
            uint32_t data_to_copy;

            /* Record the fact we received a reply of some kind */
            result = WICED_SUCCESS;

            /* Copy data into provided buffer */
            offset = 0;
            do
            {
                wiced_packet_get_data( reply_packet, offset, &data, &data_length, &available );
                data_to_copy = MIN(data_length, buffer_length);
                memcpy( buffer_ptr, data, data_to_copy );
                buffer_ptr    += data_to_copy;
                buffer_length -= data_to_copy;
                offset  = (uint16_t)(offset + data_length);
            } while (data_length < available);
        }
        if ( reply_packet != NULL )
        {
            wiced_packet_delete( reply_packet );
        }
    } while ( rx_result == WICED_SUCCESS );

    wiced_tcp_disconnect( &socket );
    wiced_tcp_delete_socket( &socket );

cleanup :
    if ( tls_identity != NULL )
    {
        wiced_tls_deinit_identity( tls_identity );
        free ( tls_identity );
    }

    return ( result );
}

wiced_result_t wiced_https_get_with_specified_port( wiced_ip_address_t* address, const char* query, void* buffer, uint32_t buffer_length, uint16_t port, const char* peer_cn )
{
    wiced_tcp_socket_t         socket;
    wiced_packet_t*            reply_packet;
    wiced_result_t             rx_result;
    wiced_tls_context_t        context;
    wiced_result_t             result     = WICED_ERROR;
    char*                      buffer_ptr = (char*) buffer;
    uint32_t                   byte_count = 0;
    uint32_t                   packet_count = 0;

    wiced_tls_init_context( &context, NULL, peer_cn );

    if ( ( result = wiced_tcp_create_socket( &socket, WICED_STA_INTERFACE ) ) != WICED_SUCCESS )
    {
        return result;
    }
    wiced_tcp_enable_tls( &socket, &context );

    result = wiced_tcp_connect( &socket, address, port, 20000 );
    if ( result != WICED_SUCCESS )
    {
        wiced_tcp_delete_socket( &socket );
        return ( result );
    }

    if ( ( result = wiced_tcp_send_buffer( &socket, query, (uint16_t) strlen( query ) ) != WICED_SUCCESS ) )
    {
        wiced_tcp_disconnect( &socket );
        wiced_tcp_delete_socket( &socket );
        return ( result );
    }

    WPRINT_LIB_INFO( ("waiting for HTTP reply\n") );

    do
    {
        rx_result = wiced_tcp_receive( &socket, &reply_packet, 5000 );
        if ( rx_result == WICED_SUCCESS )
        {
            uint8_t* data;
            uint16_t offset;
            uint16_t data_length;
            uint16_t available;
            uint32_t data_to_copy;

            /* Record the fact we received a reply of some kind */
            result = WICED_SUCCESS;

            /* Copy data into provided buffer */
            offset = 0;
            do
            {
                wiced_packet_get_data( reply_packet, offset, &data, &data_length, &available );
                byte_count += data_length;
                data_to_copy = MIN(data_length, buffer_length);
                memcpy( buffer_ptr, data, data_to_copy );
                buffer_ptr    += data_to_copy;
                buffer_length -= data_to_copy;
                offset  = (uint16_t)(offset + data_length);
            } while (data_length < available);
            packet_count++;
            wiced_packet_delete( reply_packet );
        }
    } while ( rx_result == WICED_SUCCESS );

    WPRINT_LIB_INFO( ("Packet count: %u\n", (unsigned int)packet_count) );
    WPRINT_LIB_INFO( ("Byte count  : %u\n", (unsigned int)byte_count) );

    wiced_tcp_disconnect( &socket );
    wiced_tcp_delete_socket( &socket );

    return ( result );
}
