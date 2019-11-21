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
#include "http2.h"
#include "http2_pvt.h"
#include "wiced_tcpip.h"
#include "wiced_tls.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HTTP2_WORKER_THREAD_PRIORITY            (WICED_NETWORK_WORKER_PRIORITY + 1)
#define HTTP2_WORKER_THREAD_NAME                "HTTP2 worker"

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    HTTP2_EVENT_SHUTDOWN    = (1 << 0),
    HTTP2_EVENT_RECEIVE     = (1 << 1),
    HTTP2_EVENT_DISCONNECT  = (1 << 2),
} HTTP2_EVENTS_T;

#define HTTP2_ALL_EVENTS    (0xFFFFFFFF)

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static wiced_result_t http_network_receive_callback     ( wiced_tcp_socket_t* socket, void* arg );
static wiced_result_t http_network_disconnect_callback  ( wiced_tcp_socket_t* socket, void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/
uint8_t alpn_protocols[] = "h2-16,h2-15,h2-14,h2,spdy/3.1,http/1.1";
/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t http_network_create_packet( http_socket_t* socket, http_packet_t* packet, uint16_t size, uint16_t *size_available )
{
    wiced_result_t result;

    /* Create the TCP packet. Memory for the tx_data is automatically allocated */
    HTTP_DEBUG(( "[%s : %s()] : [%d] : Requested data packet of size = [%d] \n", __FILE__, __FUNCTION__,__LINE__, size ));
    result =  wiced_packet_create_tcp( &socket->socket, size, &packet->packet, &packet->data, size_available );
    HTTP_DEBUG(( "[%s : %s()] : [%d] : Created data packet size = [%d] \n", __FILE__, __FUNCTION__,__LINE__, *size_available ));
    return result;
}

wiced_result_t http_network_delete_packet( http_packet_t* packet )
{
    /* Delete a created packet */
    return wiced_packet_delete( packet->packet );
}

wiced_result_t http_network_send_packet( http_socket_t* socket, http_packet_t* packet )
{
    wiced_result_t result = WICED_SUCCESS;

    /*Set the end of the data portion*/
    wiced_packet_set_data_end( packet->packet, packet->data );
    result = wiced_tcp_send_packet( &socket->socket, packet->packet );
    if ( result != WICED_SUCCESS )
    {
        /*Delete packet, since the send failed*/
        http_network_delete_packet( packet );
    }
    return result;
}

static wiced_result_t http_network_receive_event_handler( http_socket_t* socket )
{
    wiced_result_t result;
    uint8_t *data;
    wiced_packet_t *packet;

    /*Receive the query from the TCP client*/
    while ( ( result = wiced_tcp_receive( &socket->socket, &packet, WICED_NO_WAIT ) ) == WICED_SUCCESS )
    {
        uint16_t offset;
        uint16_t rx_data_length;
        uint16_t available_data_length;
        uint16_t current_size = 0;
        uint32_t size;

        /*Process the client request*/
        offset = 0;
        do
        {
            wiced_packet_get_data( packet, offset, &data, &rx_data_length, &available_data_length );
            size = rx_data_length;
            HTTP_DEBUG(( "[%s()] : [%d] : Received packet data size = [%d]\n", __FUNCTION__,__LINE__, rx_data_length ));
            while ( size )
            {
                if ( http_receive_callback( data + current_size, &size, socket->p_user ) != WICED_SUCCESS )
                {
                    break;
                }
                HTTP_DEBUG(( "[%s()] : [%d] : Bytes remaining after receive callback = [%u]\n", __FUNCTION__,__LINE__, (unsigned int)size ));
                current_size = (uint16_t) ( rx_data_length - size );
            }

            offset = (uint16_t)(offset + rx_data_length);
        } while (rx_data_length < available_data_length);

        /*Delete the packet, we're done with it*/
        wiced_packet_delete( packet );
    }

    return result;
}

static void http_network_thread(uint32_t context)
{
    http_connection_t* connect = (http_connection_t*)context;
    wiced_result_t result;
    uint32_t events;

    HTTP_DEBUG(( "[%s()] : [%d] : HTTP2 network thread begin\n", __FUNCTION__, __LINE__ ));

    while (WICED_TRUE)
    {
        events = 0;
        result = wiced_rtos_wait_for_event_flags(&connect->events, HTTP2_ALL_EVENTS, &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);

        if (result != WICED_SUCCESS || events & HTTP2_EVENT_SHUTDOWN)
        {
            break;
        }

        if (events & HTTP2_EVENT_RECEIVE)
        {
            http_network_receive_event_handler(connect->socket_ptr);
        }

        if (events & HTTP2_EVENT_DISCONNECT)
        {
            http_disconnect_callback(connect);
        }
    }

    HTTP_DEBUG(( "[%s()] : [%d] : HTTP2 network thread end\n", __FUNCTION__, __LINE__ ));
}

static wiced_result_t http_network_receive_callback( wiced_tcp_socket_t* socket, void* arg )
{
    http_socket_t* http_socket = container_of( socket, http_socket_t, socket );

    (void)arg;

    wiced_rtos_set_event_flags(&((http_connection_t*)http_socket->p_user)->events, HTTP2_EVENT_RECEIVE);

    return WICED_SUCCESS;
}

static wiced_result_t http_network_disconnect_callback( wiced_tcp_socket_t* socket, void* arg )
{
    http_socket_t* http_socket = container_of( socket, http_socket_t, socket );

    ( void )arg;

    wiced_rtos_set_event_flags(&((http_connection_t*)http_socket->p_user)->events, HTTP2_EVENT_DISCONNECT);

    return WICED_SUCCESS;
}

wiced_result_t http_network_tls_init( http_socket_t *socket, const http_security_info_t *security, http_client_config_info_t* config )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_tls_extension_t extension;

    socket->tls_context.identity = NULL;
    socket->security             = security;
    if ( security != NULL )
    {
        if ( ( security->ca_cert != NULL ) && ( result = wiced_tls_init_root_ca_certificates( security->ca_cert, security->ca_cert_len ) ) != WICED_SUCCESS )
        {
            if ( ( result = wiced_tls_init_root_ca_certificates( security->ca_cert, security->ca_cert_len ) ) != WICED_SUCCESS )
            {
                goto ERROR_CA_CERT_INIT;
            }
        }
        if ( ( security->cert != NULL ) && ( security->key != NULL ) )
        {
            result = wiced_tls_init_identity( &socket->tls_identity, (char*) security->key, security->key_len, (const uint8_t*) security->cert, security->cert_len );
            if ( result != WICED_SUCCESS )
            {
                goto ERROR_TLS_INIT;
            }
            wiced_tls_init_context( &socket->tls_context, &socket->tls_identity, NULL );
        }
        else
        {
            wiced_tls_init_context( &socket->tls_context, NULL, NULL );

        }

        HTTP_DEBUG(( "[%s()] : [%d] : Add ALPN TLS Extension\n", __FUNCTION__, __LINE__ ));
        /* Add "h2" extension for signaling http2 protocol */
        extension.type = TLS_EXTENSION_TYPE_APPLICATION_LAYER_PROTOCOL_NEGOTIATION;
        extension.extension_data.alpn_protocol_list = alpn_protocols;
        wiced_tls_set_extension( &socket->tls_context, &extension );

        /* set TLS extensions configured by client */
        if( config != NULL )
        {
            if( config->flag & HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME )
            {
                HTTP_DEBUG(( "[%s()] : [%d] : Add SNI TLS Extension\n", __FUNCTION__, __LINE__ ));
                /* Add "server_name" extension */
                extension.type = TLS_EXTENSION_TYPE_SERVER_NAME;
                extension.extension_data.server_name = (uint8_t*)config->server_name;
                wiced_tls_set_extension( &socket->tls_context, &extension );
            }
        }
    }
    return result;

ERROR_TLS_INIT:
    wiced_tls_deinit_root_ca_certificates( );
ERROR_CA_CERT_INIT:
    return result;
    }

wiced_result_t http_network_tls_deinit( http_socket_t *socket )
{
    if ( socket->security != NULL )
    {
        wiced_tls_deinit_identity(&socket->tls_identity);
        wiced_tls_reset_context( socket->socket.tls_context );
        wiced_tls_deinit_root_ca_certificates( );
    }
    return WICED_SUCCESS;
}

wiced_result_t http2_network_init( http_connection_t* connect )
{
    wiced_result_t result;

    connect->socket_ptr->p_user = connect;

    /*
     * Has the worker thread already been initialized?
     */

    if (connect->http2_thread_ptr != NULL)
    {
        return WICED_SUCCESS;
    }

    /*
     * Create our worker thread.
     */

    HTTP_DEBUG(( "[%s()] : [%d] : Create worker thread\n", __FUNCTION__, __LINE__ ));
    result = wiced_rtos_init_event_flags(&connect->events);
    if (result != WICED_SUCCESS)
    {
        HTTP_DEBUG(( "[%s()] : [%d] : Create event flags failed: %d\n", __FUNCTION__, __LINE__, result ));
        return result;
    }

    result = wiced_rtos_create_thread(&connect->http2_thread, HTTP2_WORKER_THREAD_PRIORITY, HTTP2_WORKER_THREAD_NAME, http_network_thread, connect->stack_size, connect);
    if (result != WICED_SUCCESS)
    {
        HTTP_DEBUG(( "[%s()] : [%d] : Create worker thread failed: %d\n", __FUNCTION__, __LINE__, result ));
        wiced_rtos_deinit_event_flags(&connect->events);
        return result;
    }
    connect->http2_thread_ptr = &connect->http2_thread;

    return WICED_SUCCESS;
}

wiced_result_t http2_network_deinit( http_connection_t* connect )
{
    if (connect->http2_thread_ptr == NULL)
    {
        return WICED_SUCCESS;
    }

    /*
     * Shutdown the worker thread.
     */

    wiced_rtos_set_event_flags(&connect->events, HTTP2_EVENT_SHUTDOWN);

    wiced_rtos_thread_force_awake(&connect->http2_thread);
    wiced_rtos_thread_join(&connect->http2_thread);
    wiced_rtos_delete_thread(&connect->http2_thread);

    connect->http2_thread_ptr = NULL;

    /*
     * And delete the event flags.
     */

    wiced_rtos_deinit_event_flags(&connect->events);

    return WICED_SUCCESS;
}

wiced_result_t http_network_connect( http_socket_t *socket, const wiced_ip_address_t *server_ip_address, uint16_t portnumber, wiced_interface_t interface )
{
    wiced_result_t result = WICED_SUCCESS;
    int connection_retries;

    /* Create a TCP socket */
    if ( ( result = wiced_tcp_create_socket( &socket->socket, interface ) ) != WICED_SUCCESS )
    {
        return result;
    }

    if ( socket->security != NULL )
    {
        wiced_tcp_enable_tls( &socket->socket, &socket->tls_context );
    }
    /* Connect to the remote TCP server, try several times */
    connection_retries = 0;
    do
    {
        result = wiced_tcp_connect( &socket->socket, server_ip_address, portnumber, HTTP_CLIENT_CONNECTION_TIMEOUT );
        connection_retries++ ;
    } while ( ( result != WICED_SUCCESS ) && ( connection_retries < HTTP_CLIENT_CONNECTION_NUMBER_OF_RETRIES ) );

    if ( result == WICED_SUCCESS )
    {
        result = wiced_tcp_register_callbacks( &socket->socket, NULL, http_network_receive_callback, http_network_disconnect_callback, socket->p_user );
    }
    else
    {
        wiced_tcp_delete_socket( &socket->socket );
    }
    return result;
}

wiced_result_t http_network_disconnect( http_socket_t *socket )
{
    wiced_tcp_unregister_callbacks( &socket->socket );
    return wiced_tcp_delete_socket( &socket->socket );
}

