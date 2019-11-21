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
 *  I/O functions
 *
 *  Provides functions for sending and receiving to the network for use by
 *  framing layer.
 */
#include "amqp_frame.h"
#include "amqp_network.h"
#include "amqp_manager.h"
#include "wiced_tls.h"
#include "amqp.h"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define TCP_SERVER_THREAD_PRIORITY          (WICED_DEFAULT_LIBRARY_PRIORITY)
#define TCP_SERVER_STACK_SIZE               (6200)
#define TCP_CLIENT_PORT                     (50007)
#define TCP_CLIENT_INTERVAL                 (2)
#define TCP_CLIENT_CONNECT_TIMEOUT          (500)
#define TCP_CONNECTION_NUMBER_OF_RETRIES    (3)
#define WICED_AMQP_QUEUE_SIZE               (10)

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum amqp_network_message_type_e
{
        AMQP_MSG_TYPE_SEND       = 0,
        AMQP_MSG_TYPE_RECEIVE    = 1,
        AMQP_MSG_TYPE_DISCONNECT = 2,
} amqp_network_message_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    wiced_packet_t* packet;
    void*           data;
    amqp_network_message_type_t msg_type;
} wiced_amqp_network_message_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t amqp_network_recv_callback( wiced_tcp_socket_t *socket, void *args );
static wiced_result_t amqp_network_disconnect_callback( wiced_tcp_socket_t *socket, void *args );
static void amqp_network_process_recv_packet(wiced_amqp_socket_t* socket);
static void amqp_network_queue_thread( uint32_t arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t amqp_network_init( const wiced_ip_address_t* server_ip_address, uint16_t portnumber, wiced_interface_t interface, void* p_user, wiced_amqp_socket_t* socket, const wiced_amqp_socket_security_t* security )
{
    wiced_result_t result;
    wiced_amqp_connection_instance *connection_instance = (wiced_amqp_connection_instance *) p_user;
    /* Create a TCP socket */
    result = wiced_tcp_create_socket( &socket->socket, interface);
    if ( result != WICED_SUCCESS)
    {
        goto ERROR_CREATE_SOCKET;
    }
    if ( security != NULL )
    {
        if ( ( result = wiced_tls_init_root_ca_certificates( security->ca_cert, security->ca_cert_len ) ) != WICED_SUCCESS )
        {
            goto ERROR_CA_CERT_INIT;
        }
        if ( ( security->cert != NULL ) && ( security->key != NULL ) )
        {
            result = wiced_tls_init_identity( &socket->tls_identity, (char*) security->key, security->key_len, (const uint8_t*) security->cert, security->cert_len );
            if ( result != WICED_SUCCESS )
            {
                goto ERROR_TLS_INIT;
            }
            wiced_tls_init_context( &socket->tls_context, &socket->tls_identity, (const char*) connection_instance->conn->peer_cn );
        }
        else
        {
            wiced_tls_init_context( &socket->tls_context, NULL,  (const char*) connection_instance->conn->peer_cn );

        }
        wiced_tcp_enable_tls( &socket->socket, &socket->tls_context );
    }

    /* Bind to the socket */
    wiced_tcp_bind( &socket->socket, TCP_CLIENT_PORT );

    result = wiced_rtos_init_semaphore( &socket->net_semaphore );
    if ( result != WICED_SUCCESS )
    {
        goto ERROR_SEM_INIT;
    }

    socket->quit   = WICED_FALSE;
    socket->p_user = p_user;
    result = wiced_rtos_init_queue( &socket->queue, "amqp", sizeof(wiced_amqp_network_message_t), WICED_AMQP_QUEUE_SIZE );
    if ( result != WICED_SUCCESS )
    {
        goto ERROR_QUE_INIT;
    }

    result = wiced_rtos_create_thread( &socket->queue_thread, WICED_DEFAULT_LIBRARY_PRIORITY, "amqp", amqp_network_queue_thread, TCP_SERVER_STACK_SIZE, socket );
    if ( result != WICED_SUCCESS )
    {
        goto ERROR_NET_QUE_THREAD;
    }

    result = wiced_tcp_register_callbacks( &socket->socket, NULL, amqp_network_recv_callback, amqp_network_disconnect_callback, socket );
    if ( result != WICED_SUCCESS )
    {
        goto ERROR_TCP_CB_REG;
    }

    socket->server_ip_address = *server_ip_address;
    socket->portnumber        = portnumber;

    return WICED_SUCCESS;

ERROR_TCP_CB_REG:
    wiced_rtos_thread_force_awake( &socket->queue_thread );
    wiced_rtos_thread_join( &socket->queue_thread );
    wiced_rtos_delete_thread( &socket->queue_thread );

ERROR_NET_QUE_THREAD:
    wiced_rtos_deinit_queue( &socket->queue );
    if ( security == NULL )
    {
        goto ERROR_CA_CERT_INIT;
    }

ERROR_QUE_INIT:
    wiced_rtos_deinit_semaphore( &socket->net_semaphore );

ERROR_SEM_INIT:
    wiced_tls_reset_context( socket->socket.tls_context );

ERROR_TLS_INIT:
    wiced_tls_deinit_root_ca_certificates( );

ERROR_CA_CERT_INIT:
    wiced_tcp_delete_socket( &socket->socket );

ERROR_CREATE_SOCKET:
    return result;
}

wiced_result_t amqp_network_connect( wiced_amqp_socket_t* socket )
{
    wiced_result_t result;
    int connection_retries;

    /* Connect to the remote TCP server, try several times */
    connection_retries = 0;
    do
    {
        result = wiced_tcp_connect( &socket->socket, &socket->server_ip_address, socket->portnumber, TCP_CLIENT_CONNECT_TIMEOUT );
        connection_retries++ ;
    } while ( ( result != WICED_SUCCESS ) && ( connection_retries < TCP_CONNECTION_NUMBER_OF_RETRIES ) );

    if ( result != WICED_SUCCESS )
    {
        return result;
    }
    wiced_rtos_set_semaphore( &socket->net_semaphore );

    return WICED_SUCCESS;
}

wiced_result_t amqp_network_disconnect( wiced_amqp_socket_t* socket )
{
    return wiced_tcp_disconnect( &socket->socket );
}

wiced_result_t amqp_network_deinit( wiced_amqp_socket_t* socket )
{
    socket->quit = WICED_TRUE;

    wiced_tcp_unregister_callbacks( &socket->socket );

    wiced_rtos_thread_force_awake( &socket->queue_thread );
    wiced_rtos_thread_join( &socket->queue_thread );
    wiced_rtos_delete_thread( &socket->queue_thread );

    wiced_rtos_deinit_queue( &socket->queue );

    wiced_rtos_deinit_semaphore( &socket->net_semaphore );

    wiced_tls_reset_context( socket->socket.tls_context );

    wiced_tls_deinit_identity( &socket->tls_identity );

    wiced_tls_deinit_root_ca_certificates( );

    wiced_tcp_delete_socket( &socket->socket );

    return WICED_SUCCESS;
}

wiced_result_t amqp_network_create_buffer( wiced_amqp_buffer_t* buffer, uint16_t size, wiced_amqp_socket_t* socket)
{
    uint16_t available_data_length;

    /* Create the TCP packet. Memory for the tx_data is automatically allocated */
    return wiced_packet_create_tcp( &socket->socket, size, &buffer->packet, &buffer->data, &available_data_length );
}

wiced_result_t amqp_network_delete_buffer( wiced_amqp_buffer_t* buffer)
{
    /* Create the TCP packet. Memory for the tx_data is automatically allocated */
    return wiced_packet_delete( buffer->packet );
}

wiced_result_t amqp_network_send_buffer( const wiced_amqp_buffer_t* buffer, wiced_amqp_socket_t* socket)
{
    wiced_amqp_network_message_t message;

    message.packet = buffer->packet;
    message.data   = buffer->data;
    message.msg_type = AMQP_MSG_TYPE_SEND;
    return wiced_rtos_push_to_queue( &socket->queue, &message, NEVER_TIMEOUT );
}

static wiced_result_t amqp_network_recv_callback( wiced_tcp_socket_t *socket, void *args )
{
    wiced_amqp_network_message_t message;
    wiced_amqp_socket_t *amqp_socket;

    UNUSED_PARAMETER(socket);
    amqp_socket = (wiced_amqp_socket_t *) args;
    message.packet = NULL;
    message.data   = NULL;
    message.msg_type = AMQP_MSG_TYPE_RECEIVE;
    return wiced_rtos_push_to_queue( &amqp_socket->queue, &message, WICED_NO_WAIT );
}

static wiced_result_t amqp_network_disconnect_callback( wiced_tcp_socket_t *socket, void *args )
{
    wiced_amqp_network_message_t message;
    wiced_amqp_socket_t *amqp_socket;

    UNUSED_PARAMETER(socket);
    amqp_socket = (wiced_amqp_socket_t *) args;
    message.packet = NULL;
    message.data   = NULL;
    message.msg_type = AMQP_MSG_TYPE_DISCONNECT;
    return wiced_rtos_push_to_queue( &amqp_socket->queue, &message, WICED_NO_WAIT );
}

static void amqp_network_process_recv_packet(wiced_amqp_socket_t* socket)
{
    wiced_result_t result;
    uint8_t* data;
    wiced_packet_t* packet;

    /* Receive the query from the TCP client */
    result = wiced_tcp_receive( &socket->socket, &packet, WICED_NO_WAIT );
    if ( result == WICED_SUCCESS )
    {
        uint16_t offset;
        uint16_t rx_data_length;
        uint16_t available_data_length;
        uint16_t current_size = 0;
        wiced_amqp_buffer_t buffer;

        /* Process the client request */
        offset = 0;
        do
        {
            wiced_packet_get_data( packet, offset, (uint8_t**) ( &data ), &rx_data_length, &available_data_length );
            while ( current_size < rx_data_length )
            {
                uint32_t size;
                buffer.packet = packet;
                buffer.data = data + current_size;
                if ( amqp_frame_recv( &buffer, socket->p_user, &size ) != WICED_SUCCESS )
                {
                    return;
                }
                current_size = (uint16_t) ( current_size + size );
            }

            offset = (uint16_t) ( offset + rx_data_length );
        } while (rx_data_length < available_data_length);

        /* Delete the packet, we're done with it */
        wiced_packet_delete( buffer.packet );
    }
}

/*
 * Serialize all AMQP events for connection and channels to avoid any asynchronus behaviour.
 * The thread waits on the amqp_queue which can be fed by the user or the network receiving thread
 */
static void amqp_network_queue_thread( uint32_t arg )
{
    wiced_amqp_network_message_t message;
    wiced_amqp_socket_t*         socket = (wiced_amqp_socket_t*)arg;
    wiced_queue_t*               queue  = &socket->queue;

    while ( socket->quit != WICED_TRUE )
    {
        wiced_result_t result = wiced_rtos_pop_from_queue( queue, &message, NEVER_TIMEOUT );
        if (result != WICED_SUCCESS)
        {
            break;
        }

        switch(message.msg_type)
        {
            case AMQP_MSG_TYPE_SEND:
                /* Set the end of the data portion */
                wiced_packet_set_data_end( message.packet, message.data );
                result = wiced_tcp_send_packet( &socket->socket,message.packet );
                if ( result != WICED_SUCCESS )
                {
                    /* Delete packet, since the send failed */
                    wiced_packet_delete( message.packet );
                }
                break;

            case AMQP_MSG_TYPE_RECEIVE:
                amqp_network_process_recv_packet(socket);
                break;

            case AMQP_MSG_TYPE_DISCONNECT:
            default:
                /* Disconnect or unknown message received, close the connection */
                if ( ((wiced_amqp_connection_instance *)(socket->p_user))->conn->is_open == WICED_FALSE )
                {
                    break;
                }

                if ( socket->socket.tls_context != NULL )
                {
                    wiced_tls_reset_context( socket->socket.tls_context );
                }

                /* Cleanup the current connection and related data */
                result = amqp_manager( WICED_AMQP_EVENT_CLOSE_RCVD, NULL, 0, socket->p_user );
                if ( result == WICED_SUCCESS )
                {
                    /* Update the APP about disconnection */
                    if ( ((wiced_amqp_connection_instance *)(socket->p_user))->conn->callbacks.connection_event != NULL )
                    {
                        ((wiced_amqp_connection_instance *)(socket->p_user))->conn->callbacks.connection_event( WICED_AMQP_EVENT_CLOSE_RCVD, NULL, ((wiced_amqp_connection_instance *)(socket->p_user)) );
                    }
                }
                break;
        }
    }
    wiced_rtos_deinit_queue( queue );

    WICED_END_OF_CURRENT_THREAD( );
}
