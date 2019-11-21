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
 * NetX_Duo TCP/IP library
 */

#include "wiced.h"
#include "wiced_network.h"
#include "nx_api.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wiced_tcpip.h"
#include "wwd_assert.h"
#include "wiced_crypto.h"
#include "dns.h"
#include "linked_list.h"
#include "internal/wiced_internal_api.h"
#ifndef WICED_DISABLE_TLS
#include "wiced_tcpip_tls_api.h"
#include "wiced_tls.h"
#endif /* ifndef WICED_DISABLE_TLS */

#ifndef WICED_CONFIG_DISABLE_DTLS
#include "wiced_udpip_dtls_api.h"
#include "wiced_dtls.h"
#endif /* ifndef WICED_CONFIG_DISABLE_DTLS */

#include "platform_ethernet.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define MIN(x,y)  ((x) < (y) ? (x) : (y))

#define NX_TIMEOUT(timeout_ms)   ((timeout_ms != WICED_NEVER_TIMEOUT) ? ((ULONG)(timeout_ms * SYSTICK_FREQUENCY / 1000)) : NX_WAIT_FOREVER )
/******************************************************
 *                    Constants
 ******************************************************/

#ifndef WICED_MAXIMUM_NUMBER_OF_SOCKETS_WITH_CALLBACKS
#define WICED_MAXIMUM_NUMBER_OF_SOCKETS_WITH_CALLBACKS    (5)
#endif

#ifndef WICED_MAXIMUM_NUMBER_OF_SERVER_SOCKETS
#define WICED_MAXIMUM_NUMBER_OF_SERVER_SOCKETS (WICED_MAXIMUM_NUMBER_OF_SOCKETS_WITH_CALLBACKS)
#endif

#ifndef WICED_TCP_TX_RETRIES
#define WICED_TCP_TX_RETRIES       WICED_DEFAULT_TCP_TX_RETRIES
#endif

#ifndef WICED_TCP_TX_DEPTH_QUEUE
#define WICED_TCP_TX_DEPTH_QUEUE    WICED_DEFAULT_TCP_TX_DEPTH_QUEUE
#endif

#ifndef WICED_TCP_WINDOW_SIZE
#define WICED_TCP_WINDOW_SIZE       WICED_DEFAULT_TCP_WINDOW_SIZE
#endif

/* If running persistent multiple socket connections, we can not queue on a socket */
#define PERSISTANT_TCP_SERVER_TCP_LISTEN_QUEUE_SIZE 0

#define PRIMARY_INTERFACE           0

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef VOID (*tcp_listen_callback_t)(NX_TCP_SOCKET *socket_ptr, UINT port);

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static void           internal_nx_tcp_listen_callback                ( NX_TCP_SOCKET *socket_ptr, UINT port );
static void           internal_nx_tcp_socket_disconnect_callback     ( NX_TCP_SOCKET *socket_ptr );
static void           internal_nx_tcp_socket_receive_callback        ( NX_TCP_SOCKET *socket_ptr );
static void           internal_nx_udp_socket_receive_callback        ( NX_UDP_SOCKET *socket_ptr );
static wiced_result_t internal_wiced_tcp_server_listen               ( wiced_tcp_server_t* tcp_server );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_result_t netx_returns[] =
{
    [NX_SUCCESS             ] = WICED_TCPIP_SUCCESS,
    [NX_NO_PACKET           ] = WICED_TCPIP_TIMEOUT,
    [NX_UNDERFLOW           ] = WICED_TCPIP_ERROR,
    [NX_OVERFLOW            ] = WICED_TCPIP_ERROR,
    [NX_NO_MAPPING          ] = WICED_TCPIP_ERROR,
    [NX_DELETED             ] = WICED_TCPIP_ERROR,
    [NX_POOL_ERROR          ] = WICED_TCPIP_ERROR,
    [NX_PTR_ERROR           ] = WICED_TCPIP_ERROR,
    [NX_WAIT_ERROR          ] = WICED_TCPIP_ERROR,
    [NX_SIZE_ERROR          ] = WICED_TCPIP_ERROR,
    [NX_OPTION_ERROR        ] = WICED_TCPIP_ERROR,
    [NX_DELETE_ERROR        ] = WICED_TCPIP_ERROR,
    [NX_CALLER_ERROR        ] = WICED_TCPIP_ERROR,
    [NX_INVALID_PACKET      ] = WICED_TCPIP_INVALID_PACKET,
    [NX_INVALID_SOCKET      ] = WICED_TCPIP_INVALID_SOCKET,
    [NX_NOT_ENABLED         ] = WICED_TCPIP_ERROR,
    [NX_ALREADY_ENABLED     ] = WICED_TCPIP_ERROR,
    [NX_ENTRY_NOT_FOUND     ] = WICED_TCPIP_ERROR,
    [NX_NO_MORE_ENTRIES     ] = WICED_TCPIP_ERROR,
    [NX_ARP_TIMER_ERROR     ] = WICED_TCPIP_ERROR,
    [NX_RESERVED_CODE0      ] = WICED_TCPIP_ERROR,
    [NX_WAIT_ABORTED        ] = WICED_TCPIP_WAIT_ABORTED,
    [NX_IP_INTERNAL_ERROR   ] = WICED_TCPIP_ERROR,
    [NX_IP_ADDRESS_ERROR    ] = WICED_TCPIP_ERROR,
    [NX_ALREADY_BOUND       ] = WICED_TCPIP_ERROR,
    [NX_PORT_UNAVAILABLE    ] = WICED_TCPIP_PORT_UNAVAILABLE,
    [NX_NOT_BOUND           ] = WICED_TCPIP_SOCKET_CLOSED,
    [NX_RESERVED_CODE1      ] = WICED_TCPIP_ERROR,
    [NX_SOCKET_UNBOUND      ] = WICED_TCPIP_ERROR,
    [NX_NOT_CREATED         ] = WICED_TCPIP_ERROR,
    [NX_SOCKETS_BOUND       ] = WICED_TCPIP_ERROR,
    [NX_NO_RESPONSE         ] = WICED_TCPIP_ERROR,
    [NX_POOL_DELETED        ] = WICED_TCPIP_ERROR,
    [NX_ALREADY_RELEASED    ] = WICED_TCPIP_ERROR,
    [NX_RESERVED_CODE2      ] = WICED_TCPIP_ERROR,
    [NX_MAX_LISTEN          ] = WICED_TCPIP_ERROR,
    [NX_DUPLICATE_LISTEN    ] = WICED_TCPIP_ERROR,
    [NX_NOT_CLOSED          ] = WICED_TCPIP_ERROR,
    [NX_NOT_LISTEN_STATE    ] = WICED_TCPIP_ERROR,
    [NX_IN_PROGRESS         ] = WICED_TCPIP_IN_PROGRESS,
    [NX_NOT_CONNECTED       ] = WICED_TCPIP_SOCKET_CLOSED,
    [NX_WINDOW_OVERFLOW     ] = WICED_TCPIP_ERROR,
    [NX_ALREADY_SUSPENDED   ] = WICED_TCPIP_ERROR,
    [NX_DISCONNECT_FAILED   ] = WICED_TCPIP_ERROR,
    [NX_STILL_BOUND         ] = WICED_TCPIP_ERROR,
    [NX_NOT_SUCCESSFUL      ] = WICED_TCPIP_ERROR,
    [NX_UNHANDLED_COMMAND   ] = WICED_TCPIP_ERROR,
    [NX_NO_FREE_PORTS       ] = WICED_TCPIP_ERROR,
    [NX_INVALID_PORT        ] = WICED_TCPIP_ERROR,
    [NX_INVALID_RELISTEN    ] = WICED_TCPIP_ERROR,
    [NX_CONNECTION_PENDING  ] = WICED_TCPIP_IN_PROGRESS,
    [NX_TX_QUEUE_DEPTH      ] = WICED_TCPIP_ERROR,
    [NX_NOT_IMPLEMENTED     ] = WICED_TCPIP_ERROR,
    [NX_NOT_SUPPORTED       ] = WICED_TCPIP_ERROR,
    [NX_INVALID_INTERFACE   ] = WICED_TCPIP_ERROR,
    [NX_INVALID_PARAMETERS  ] = WICED_TCPIP_ERROR,
    [NX_NOT_FOUND           ] = WICED_TCPIP_ERROR,
    [NX_CANNOT_START        ] = WICED_TCPIP_ERROR,
    [NX_NO_INTERFACE_ADDRESS] = WICED_TCPIP_ERROR,
    [NX_INVALID_MTU_DATA    ] = WICED_TCPIP_ERROR,
    [NX_DUPLICATED_ENTRY    ] = WICED_TCPIP_ERROR,
    [NX_PACKET_OFFSET_ERROR ] = WICED_TCPIP_ERROR,
    [NX_OPTION_HEADER_ERROR ] = WICED_TCPIP_ERROR,   /* Specific to NetX-Duo */
#ifdef WICED_NETTYPE_ROM
    [NX_PARAMETER_ERROR     ] = WICED_TCPIP_BADARG,
#else
    [NX_CONTINUE            ] = WICED_TCPIP_ERROR,   /* Specific to NetX-Duo */
#endif
};


/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_tcp_server_start( wiced_tcp_server_t* tcp_server, wiced_interface_t interface, uint16_t port, uint16_t max_sockets, wiced_tcp_socket_callback_t connect_callback, wiced_tcp_socket_callback_t receive_callback, wiced_tcp_socket_callback_t disconnect_callback, void* arg )
{
    int                        i;
    wiced_tcp_server_socket_t* tcp_socket = NULL;
    wiced_result_t             status;

    if ( max_sockets > WICED_MAXIMUM_NUMBER_OF_SERVER_SOCKETS )
    {
        return WICED_BADARG;
    }

    tcp_server->interface = interface;
    tcp_server->port      = port;

    linked_list_init( &tcp_server->socket_list );

    tcp_socket = malloc( sizeof(wiced_tcp_server_socket_t) * max_sockets );
    if ( tcp_socket == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    memset( tcp_socket, 0, sizeof(wiced_tcp_server_socket_t) * max_sockets );

    for ( i = 0; i < max_sockets; i++ )
    {
        status = wiced_tcp_create_socket ( &tcp_socket[i].socket, interface );
        if( status != WICED_SUCCESS )
        {
            goto clean_up;
        }

        status = wiced_tcp_register_callbacks( &tcp_socket[i].socket, connect_callback, receive_callback, disconnect_callback, arg );
        if( status != WICED_SUCCESS )
        {
            goto clean_up;
        }

        linked_list_set_node_data( &tcp_socket[i].socket_node, &tcp_socket[i] );
        linked_list_insert_node_at_rear( &tcp_server->socket_list, &tcp_socket[i].socket_node );
    }

    tcp_server->max_tcp_connections     = max_sockets;

    tcp_server->active_tcp_connections  = 0;

    /* start server listen */
    status = netx_returns[ nx_tcp_server_socket_listen( tcp_socket[0].socket.socket.nx_tcp_socket_ip_ptr, tcp_server->port,  &tcp_socket[0].socket.socket, PERSISTANT_TCP_SERVER_TCP_LISTEN_QUEUE_SIZE, internal_nx_tcp_listen_callback ) ];
    if( status != WICED_SUCCESS )
    {
        goto clean_up;
    }

    return WICED_SUCCESS;

clean_up:
    free( tcp_socket );

    return status;
}

wiced_result_t wiced_tcp_server_accept( wiced_tcp_server_t* tcp_server, wiced_tcp_socket_t* socket )
{
    wiced_result_t result;
    UINT           nx_result;

    result = wiced_tcp_accept( socket );

    if ( result == WICED_SUCCESS )
    {
        tcp_server->active_tcp_connections = tcp_server->active_tcp_connections + 1 ;

        if( tcp_server->active_tcp_connections < tcp_server->max_tcp_connections )
        {
            result = internal_wiced_tcp_server_listen( tcp_server );
        }
    }
    else
    {
        result = netx_returns[ nx_result = nx_tcp_server_socket_relisten( socket->socket.nx_tcp_socket_ip_ptr, tcp_server->port, &socket->socket ) ];
        wiced_assert("Re-Listen failed!", nx_result == NX_SUCCESS);
    }

    return result;
}

wiced_result_t wiced_tcp_server_disconnect_socket_with_timeout( wiced_tcp_server_t* tcp_server, wiced_tcp_socket_t* socket, uint32_t timeout_ms )
{
    wiced_result_t result;

    nx_tcp_socket_disconnect( &socket->socket, NX_TIMEOUT(timeout_ms) );

    nx_tcp_server_socket_unaccept( &socket->socket );

    /* FIXME: More checks on tcp_socket_disconnect & socket_unaccept above need to be added.
     * Based on those checks, we should decrement below */
    tcp_server->active_tcp_connections = tcp_server->active_tcp_connections - 1;

    result = internal_wiced_tcp_server_listen( tcp_server );

    return result;
}

wiced_result_t wiced_tcp_server_disconnect_socket( wiced_tcp_server_t* tcp_server, wiced_tcp_socket_t* socket )
{
    return wiced_tcp_server_disconnect_socket_with_timeout( tcp_server, socket, WICED_TCP_DISCONNECT_TIMEOUT );
}

wiced_result_t wiced_tcp_server_stop( wiced_tcp_server_t* tcp_server )
{
    void* memory_to_free;
    linked_list_node_t* last_node;

    /* This is the first node, and start of malloced heap for sockets */
    linked_list_get_front_node( &tcp_server->socket_list, (linked_list_node_t**)&memory_to_free );

    while( linked_list_remove_node_from_rear( &tcp_server->socket_list, &last_node ) == WICED_SUCCESS )
    {
        wiced_tcp_server_socket_t* socket = (wiced_tcp_server_socket_t*)last_node;

        if ( socket->socket.socket_magic_number == WICED_SOCKET_MAGIC_NUMBER )
        {
            wiced_tcp_delete_socket( &socket->socket );
        }
    }

    /* free malloced heap */
    free( memory_to_free );

    return WICED_SUCCESS;
}

wiced_result_t wiced_tcp_server_get_socket_at_index( wiced_tcp_server_t* tcp_server, wiced_tcp_socket_t** socket, uint8_t index )
{
    linked_list_t* socket_list = NULL;
    linked_list_node_t* node = NULL;
    uint32_t count_nodes = 0;

    wiced_assert("Error fetching Server's socket", (tcp_server != NULL) && (socket != NULL) );

    socket_list = &tcp_server->socket_list;

    *socket = NULL;

    if( index > socket_list->count )
    {
        return WICED_ERROR;
    }

    /* Get the front node */
    linked_list_get_front_node(socket_list, &node);

    while( (count_nodes < socket_list->count) && (node != NULL) )
    {
        if( count_nodes == index )
        {
            break;
        }
        else
        {
            node = node->next;
            count_nodes++;
        }
    }

    if( !node )
    {
        return WICED_ERROR;
    }

    *socket =  &( ((wiced_tcp_server_socket_t *)node->data)->socket);

    return WICED_SUCCESS;
}

wiced_result_t wiced_tcp_create_socket( wiced_tcp_socket_t* socket, wiced_interface_t interface )
{
    UINT result;

    memset( socket, 0, sizeof(wiced_tcp_socket_t) );

    /* Set magic number for validating wiced_tcp_socket_t object and differentiating with a native NX_TCP_SOCKET object */
    socket->socket_magic_number = WICED_SOCKET_MAGIC_NUMBER;

    result = nx_tcp_socket_create( &IP_HANDLE(interface), &socket->socket, (CHAR*)"", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, WICED_TCP_WINDOW_SIZE, NX_NULL, NX_NULL );
    wiced_assert("Error creating socket", result == NX_SUCCESS);
    nx_tcp_socket_transmit_configure(&socket->socket, WICED_TCP_TX_DEPTH_QUEUE, WICED_TCP_SEND_TIMEOUT/WICED_TCP_TX_RETRIES, WICED_TCP_TX_RETRIES, 0);

    return netx_returns[result];
}

wiced_result_t wiced_tcp_get_socket_state( wiced_tcp_socket_t* socket, wiced_socket_state_t* socket_state )
{

    if( socket == NULL )
    {
        *socket_state = WICED_SOCKET_ERROR;
        return WICED_BADARG;
    }

    switch( socket->socket.nx_tcp_socket_state )
    {
        case NX_TCP_CLOSED:
            *socket_state = WICED_SOCKET_CLOSED;
            break;
        case NX_TCP_CLOSING:
        case NX_TCP_CLOSE_WAIT:
        case NX_TCP_FIN_WAIT_1:
        case NX_TCP_FIN_WAIT_2:
        case NX_TCP_TIMED_WAIT:
        case NX_TCP_LAST_ACK  :
            *socket_state = WICED_SOCKET_CLOSING;
            break;
        case NX_TCP_SYN_SENT:
        case NX_TCP_SYN_RECEIVED:
            *socket_state = WICED_SOCKET_CONNECTING;
            break;
        case NX_TCP_ESTABLISHED:
            *socket_state = WICED_SOCKET_CONNECTED;
            break;
        case NX_TCP_LISTEN_STATE:
            *socket_state = WICED_SOCKET_LISTEN;
            break;
        default:
            *socket_state = WICED_SOCKET_ERROR;
            break;

    }

    return WICED_SUCCESS;

}

void wiced_tcp_set_type_of_service( wiced_tcp_socket_t* socket, uint32_t tos )
{
    socket->socket.nx_tcp_socket_type_of_service = tos << 16;
}

wiced_result_t wiced_tcp_accept( wiced_tcp_socket_t* socket )
{
    wiced_result_t result = WICED_TCPIP_SUCCESS;
    UINT net_result;

    WICED_LINK_CHECK_TCP_SOCKET( socket );

    if ( socket->callbacks.connect == NULL )
    {
        if ( socket->socket.nx_tcp_socket_state != NX_TCP_LISTEN_STATE )
        {
            nx_tcp_socket_disconnect( &socket->socket, NX_TIMEOUT(WICED_TCP_DISCONNECT_TIMEOUT) );
        }

        nx_tcp_server_socket_unaccept( &socket->socket );
        nx_tcp_server_socket_relisten( socket->socket.nx_tcp_socket_ip_ptr, socket->socket.nx_tcp_socket_port, &socket->socket );

#ifndef WICED_DISABLE_TLS
        if ( socket->tls_context != NULL )
        {
            wiced_tls_reset_context( socket->tls_context );
        }
#endif /* ifndef WICED_DISABLE_TLS */

        net_result = nx_tcp_server_socket_accept( &socket->socket, NX_TIMEOUT(WICED_TCP_ACCEPT_TIMEOUT) );
        if ( ( net_result == NX_NOT_CONNECTED ) || ( net_result == NX_WAIT_ABORTED ) )
        {
            return netx_returns[net_result];
        }
        else if ( net_result != NX_SUCCESS )
        {
            return netx_returns[net_result];
        }

#ifndef WICED_DISABLE_TLS
        if ( socket->tls_context != NULL )
        {
            net_result = wiced_tcp_start_tls( socket, WICED_TLS_AS_SERVER, WICED_TLS_DEFAULT_VERIFICATION );
            if ( net_result != NX_SUCCESS )
            {
                return netx_returns[net_result];
            }
        }
#endif /* ifndef WICED_DISABLE_TLS */
    }
    else
    {
        if ( ( socket->socket.nx_tcp_socket_state == NX_TCP_LISTEN_STATE ) || ( socket->socket.nx_tcp_socket_state == NX_TCP_SYN_RECEIVED ) )
        {

#ifndef WICED_DISABLE_TLS
            if ( socket->tls_context != NULL )
            {
                wiced_tls_reset_context( socket->tls_context );
            }
#endif /* ifndef WICED_DISABLE_TLS */

            net_result = nx_tcp_server_socket_accept( &socket->socket, NX_TIMEOUT(WICED_TCP_ACCEPT_TIMEOUT) );
            if ( net_result != NX_SUCCESS )
            {
                nx_tcp_server_socket_unaccept( &socket->socket );
                return netx_returns[net_result];
            }

#ifndef WICED_DISABLE_TLS
            /* Only start up if the socket is successfully accepted */
            if ( ( result == WICED_SUCCESS ) && socket->tls_context != NULL )
            {
                net_result = wiced_tcp_start_tls( socket, WICED_TLS_AS_SERVER, WICED_TLS_DEFAULT_VERIFICATION );
                if ( net_result != NX_SUCCESS )
                {
                    return netx_returns[net_result];
                }
            }
#endif /* ifndef WICED_DISABLE_TLS */
        }
        else if ( socket->socket.nx_tcp_socket_state == NX_TCP_ESTABLISHED )
        {
            /* socket is already connected: return success */
            result = WICED_TCPIP_SUCCESS;
        }
        else
        {
            result = WICED_TCPIP_ERROR;
        }
        return result;
    }

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_tcp_connect( wiced_tcp_socket_t* socket, const wiced_ip_address_t* address, uint16_t port, uint32_t timeout_ms )
{
#ifndef WICED_DISABLE_TLS
    wiced_result_t result;
#endif /* ifndef WICED_DISABLE_TLS */
    UINT net_result;

    WICED_LINK_CHECK_TCP_SOCKET( socket );

    /* Check if socket is not yet bound to a local port */
    if ( !socket->socket.nx_tcp_socket_bound_next )
    {
        net_result = nx_tcp_client_socket_bind( &socket->socket, NX_ANY_PORT, NX_TIMEOUT(WICED_TCP_BIND_TIMEOUT) );
        if ( net_result != NX_SUCCESS )
        {
            return netx_returns[net_result];
        }
    }

    net_result = nxd_tcp_client_socket_connect( &socket->socket, (NXD_ADDRESS*) address, port, NX_TIMEOUT(timeout_ms) );
    if ( net_result != NX_SUCCESS )
    {
        return netx_returns[net_result];
    }

#ifndef WICED_DISABLE_TLS
    if ( socket->tls_context != NULL )
    {
        result = wiced_tcp_start_tls(socket, WICED_TLS_AS_CLIENT, WICED_TLS_DEFAULT_VERIFICATION);
        if ( result != WICED_SUCCESS)
        {
            nx_tcp_socket_disconnect( &socket->socket, NX_TIMEOUT(WICED_TCP_DISCONNECT_TIMEOUT) );
            /* Unbind the port to ensure that all the resources are released */
            nx_tcp_client_socket_unbind( &socket->socket );
            return result;
        }
    }
#endif /* ifndef WICED_DISABLE_TLS */

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_tcp_bind( wiced_tcp_socket_t* socket, uint16_t port )
{
    UINT result;
    WICED_LINK_CHECK_TCP_SOCKET( socket );

    result = nx_tcp_client_socket_bind( &socket->socket, port, NX_TIMEOUT(WICED_TCP_BIND_TIMEOUT) );
    return netx_returns[result];
}

wiced_result_t wiced_tcp_register_callbacks( wiced_tcp_socket_t* socket, wiced_tcp_socket_callback_t connect_callback, wiced_tcp_socket_callback_t receive_callback, wiced_tcp_socket_callback_t disconnect_callback, void* arg )
{
    if ( disconnect_callback != NULL )
    {
        socket->callbacks.disconnect = disconnect_callback;
        socket->socket.nx_tcp_disconnect_callback =  internal_nx_tcp_socket_disconnect_callback;
    }

    if ( receive_callback != NULL )
    {
        socket->callbacks.receive = receive_callback;
        nx_tcp_socket_receive_notify( &socket->socket, internal_nx_tcp_socket_receive_callback );
    }

    if ( connect_callback != NULL )
    {
        socket->callbacks.connect = connect_callback;
        /* Note: Link to NetX_Duo callback mechanism is configured in wiced_tcp_listen() */
    }

    socket->callback_arg = arg;

    return WICED_SUCCESS;
}

wiced_result_t wiced_tcp_unregister_callbacks( wiced_tcp_socket_t* socket )
{
    memset( &socket->callbacks, 0, sizeof( socket->callbacks ) );
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_tcp_delete_socket( wiced_tcp_socket_t* socket )
{
    wiced_assert("Bad args", socket != NULL);

#ifndef WICED_DISABLE_TLS
    if ( socket->tls_context != NULL )
    {
        wiced_tls_close_notify( socket );

        wiced_tls_deinit_context( socket->tls_context );

        if ( socket->context_malloced == WICED_TRUE )
        {
            free( socket->tls_context );
            socket->tls_context = NULL;
            socket->context_malloced = WICED_FALSE;
        }
    }
#endif /* ifndef WICED_DISABLE_TLS */

    nx_tcp_socket_disconnect     ( &socket->socket, NX_TIMEOUT(WICED_TCP_DISCONNECT_TIMEOUT));
    nx_tcp_server_socket_unaccept( &socket->socket );
    nx_tcp_server_socket_unlisten( socket->socket.nx_tcp_socket_ip_ptr, socket->socket.nx_tcp_socket_port );
    nx_tcp_client_socket_unbind  ( &socket->socket );
    nx_tcp_socket_delete         ( &socket->socket );

    /* Invalidate WICED TCP socket */
    socket->socket_magic_number = 0;

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_tcp_listen( wiced_tcp_socket_t* socket, uint16_t port )
{
    UINT                         result;
    tcp_listen_callback_t        listen_callback = NULL;
    struct NX_TCP_LISTEN_STRUCT* listen_ptr;

    wiced_assert("Bad args", socket != NULL);

    WICED_LINK_CHECK_TCP_SOCKET( socket );

    /* Check if there is already another socket listening on the port */
    listen_ptr = socket->socket.nx_tcp_socket_ip_ptr->nx_ip_tcp_active_listen_requests;
    if ( listen_ptr != NULL )
    {
        /* Search the active listen requests for this port. */
        do
        {
            /* Determine if there is another listen request for the same port. */
            if ( listen_ptr->nx_tcp_listen_port == port )
            {
                /* Do a re-listen instead of a listen */
                result = nx_tcp_server_socket_relisten( socket->socket.nx_tcp_socket_ip_ptr, port, &socket->socket );
                return netx_returns[result];
            }
            listen_ptr = listen_ptr->nx_tcp_listen_next;
        } while ( listen_ptr != socket->socket.nx_tcp_socket_ip_ptr->nx_ip_tcp_active_listen_requests);
    }

    /* Check if this socket has an asynchronous connect callback */
    if (socket->callbacks.connect != NULL )
    {
        listen_callback = internal_nx_tcp_listen_callback;
    }

    if (socket->socket.nx_tcp_socket_state != NX_TCP_CLOSED)
    {
        nx_tcp_server_socket_unaccept( &socket->socket );
        result = nx_tcp_server_socket_relisten( socket->socket.nx_tcp_socket_ip_ptr, socket->socket.nx_tcp_socket_port, &socket->socket );
    }
    else
    {
        result = nx_tcp_server_socket_listen( socket->socket.nx_tcp_socket_ip_ptr, port, &socket->socket, WICED_DEFAULT_TCP_LISTEN_QUEUE_SIZE, listen_callback );
    }
    return netx_returns[result];
}

wiced_result_t network_tcp_send_packet( wiced_tcp_socket_t* socket, wiced_packet_t* packet )
{
    UINT result;

    WICED_LINK_CHECK_TCP_SOCKET( socket );

    result = nx_tcp_socket_send(&socket->socket, packet, NX_TIMEOUT(WICED_TCP_SEND_TIMEOUT) );
    return netx_returns[result];
}

wiced_result_t network_tcp_receive( wiced_tcp_socket_t* socket, wiced_packet_t** packet, uint32_t timeout )
{
    UINT result;

    if ( socket == NULL )
    {
        result = NX_NOT_CONNECTED;
    }
    else
    {
        result = nx_tcp_socket_receive( &socket->socket, packet, NX_TIMEOUT(timeout) );
    }

    return netx_returns[result];
}

wiced_result_t network_udp_receive( wiced_udp_socket_t* socket, wiced_packet_t** packet, uint32_t timeout )
{
    UINT result;

    if ( socket == NULL )
    {
        result = NX_NOT_CONNECTED;
    }
    else
    {
        result = nx_udp_socket_receive( &socket->socket, packet, NX_TIMEOUT(timeout) );
    }

    return netx_returns[result];
}

wiced_result_t wiced_tcp_disconnect_with_timeout( wiced_tcp_socket_t* socket, uint32_t timeout_ms )
{
    UINT result;
    WICED_LINK_CHECK_TCP_SOCKET( socket );

#ifndef WICED_DISABLE_TLS
    if ( socket->tls_context != NULL )
    {
        wiced_tls_close_notify( socket );
    }
#endif

    nx_tcp_socket_disconnect( &socket->socket, NX_TIMEOUT(timeout_ms) );
    if ( socket->socket.nx_tcp_socket_client_type == NX_TRUE)
    {
        /* Un-bind the socket, so the TCP port becomes available for other sockets which are suspended on bind requests. This will also flush the receive queue of the socket */
        /* We ignore the return of the unbind as there isn't much we can do */
        result = nx_tcp_client_socket_unbind( &socket->socket );
    }
    else
    {
        result = nx_tcp_server_socket_unaccept( &socket->socket );
    }

    return netx_returns[result];
}

wiced_result_t wiced_tcp_disconnect( wiced_tcp_socket_t* socket )
{
    return wiced_tcp_disconnect_with_timeout(socket, WICED_TCP_DISCONNECT_TIMEOUT);
}

wiced_result_t wiced_packet_create_tcp( wiced_tcp_socket_t* socket, uint16_t content_length, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space )
{
    UINT     result;
    uint16_t maximum_segment_size = (uint16_t)WICED_MAXIMUM_SEGMENT_SIZE(socket);

    UNUSED_PARAMETER( content_length );
    result = nx_packet_allocate( &wiced_packet_pools[0], packet, NX_TCP_PACKET, NX_TIMEOUT(WICED_ALLOCATE_PACKET_TIMEOUT) );
    if ( result != NX_SUCCESS )
    {
        *packet = NULL;
        *data   = NULL;
        *available_space = 0;
        wiced_assert( "Packet allocation error", result != NX_NO_PACKET );
        return netx_returns[result];;
    }

#ifndef WICED_DISABLE_TLS
    if ( socket->tls_context != NULL )
    {
        uint16_t header_space = 0;
        uint16_t footer_pad_space = 0;

        wiced_tls_calculate_overhead( &socket->tls_context->context, (uint16_t)((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr), &header_space, &footer_pad_space );

        ( *packet )->nx_packet_prepend_ptr += header_space;

        *available_space  = (uint16_t)( MIN((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr, maximum_segment_size) - footer_pad_space );
    }
    else
#endif /* ifndef WICED_DISABLE_TLS */
    {
        *available_space = (uint16_t) MIN((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr, maximum_segment_size);
    }

    ( *packet )->nx_packet_append_ptr = ( *packet )->nx_packet_prepend_ptr;
    *data = ( *packet )->nx_packet_prepend_ptr;
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_tcp_enable_keepalive( wiced_tcp_socket_t* socket, uint16_t interval, uint16_t probes, uint16_t time )
{
    UNUSED_PARAMETER( socket );
    UNUSED_PARAMETER( interval );
    UNUSED_PARAMETER( probes );
    UNUSED_PARAMETER( time );

    socket->socket.nx_tcp_socket_keepalive_enabled = WICED_TRUE;
    socket->socket.nx_tcp_socket_keepalive_retries = probes;
    socket->socket.nx_tcp_socket_keepalive_timeout = time;


    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_packet_create_udp_no_wait( wiced_udp_socket_t* socket, uint16_t content_length, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space )
{
    UINT result;
    UNUSED_PARAMETER( content_length );
    UNUSED_PARAMETER( socket );

    result = nx_packet_allocate( &wiced_packet_pools[0], packet, NX_UDP_PACKET, 0 );
    if ( result != NX_SUCCESS )
    {
        *packet = NULL;
        *data   = NULL;
        *available_space = 0;
        wiced_assert( "Packet allocation error", result == NX_NO_PACKET );
        return netx_returns[result];
    }

#ifndef WICED_CONFIG_DISABLE_DTLS
    if ( socket->dtls_context != NULL )
    {
        uint16_t header_space;
        uint16_t footer_pad_space = 0;

        wiced_dtls_calculate_overhead( &socket->dtls_context->context, (uint16_t)((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr), &header_space, &footer_pad_space );

        ( *packet )->nx_packet_prepend_ptr += header_space;

        *available_space  = (uint16_t) (((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr) - footer_pad_space);
    }
    else
#endif /* ifndef WICED_CONFIG_DISABLE_TLS */
    {
        *available_space = (uint16_t)((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr);
    }

    ( *packet )->nx_packet_append_ptr = ( *packet )->nx_packet_prepend_ptr;
    *data = ( *packet )->nx_packet_prepend_ptr;

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_packet_create_udp( wiced_udp_socket_t* socket, uint16_t content_length, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space )
{
    UINT result;
    UNUSED_PARAMETER( content_length );
    UNUSED_PARAMETER( socket );

    result = nx_packet_allocate( &wiced_packet_pools[0], packet, NX_UDP_PACKET, NX_TIMEOUT(WICED_ALLOCATE_PACKET_TIMEOUT) );
    if ( result != NX_SUCCESS )
    {
        *packet = NULL;
        *data   = NULL;
        *available_space = 0;
        wiced_assert( "Packet allocation error", result == NX_NO_PACKET );
        return netx_returns[result];
    }

#ifndef WICED_CONFIG_DISABLE_DTLS
    if ( socket->dtls_context != NULL )
    {
        uint16_t header_space;
        uint16_t footer_pad_space = 0;

        wiced_dtls_calculate_overhead( &socket->dtls_context->context, (uint16_t)((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr), &header_space, &footer_pad_space );

        ( *packet )->nx_packet_prepend_ptr += header_space;

        *available_space  = (uint16_t) (((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr) - footer_pad_space);
    }
    else
#endif /* ifndef WICED_CONFIG_DISABLE_TLS */
    {
        //*available_space = (uint16_t) MIN((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr, maximum_segment_size);
        *available_space = (uint16_t)((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr);
    }

    ( *packet )->nx_packet_append_ptr = ( *packet )->nx_packet_prepend_ptr;
    *data = ( *packet )->nx_packet_prepend_ptr;

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_packet_create( uint16_t content_length, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space )
{
    UINT result;
    UNUSED_PARAMETER( content_length );

    result = nx_packet_allocate( &wiced_packet_pools[0], packet, NX_IP_PACKET, NX_TIMEOUT(WICED_ALLOCATE_PACKET_TIMEOUT) );
    if ( result != NX_SUCCESS )
    {
        *packet = NULL;
        *data   = NULL;
        *available_space = 0;
        wiced_assert( "Packet allocation error", result == NX_NO_PACKET );
        return netx_returns[result];
    }

    *data = ( *packet )->nx_packet_prepend_ptr;
    *available_space = (uint16_t)((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr);
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_packet_delete( wiced_packet_t* packet )
{
    UINT nx_result;

    wiced_assert("Bad args", packet != NULL);
    nx_result = nx_packet_release( packet );
    return netx_returns[ nx_result ];
}

wiced_result_t wiced_packet_set_data_end( wiced_packet_t* packet, uint8_t* data_end )
{
    wiced_assert("Bad packet end\n", (data_end >= packet->nx_packet_prepend_ptr) && (data_end <= packet->nx_packet_data_end));
    packet->nx_packet_append_ptr = data_end;
    packet->nx_packet_length     = (ULONG)(packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr);
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_packet_set_data_start( wiced_packet_t* packet, uint8_t* data_start )
{
    wiced_assert("Bad packet end\n", (data_start >= packet->nx_packet_data_start) && (data_start <= packet->nx_packet_data_end));
    packet->nx_packet_prepend_ptr = data_start;
    packet->nx_packet_length      = (ULONG)(packet->nx_packet_append_ptr - packet->nx_packet_prepend_ptr);
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_udp_packet_get_info( wiced_packet_t* packet, wiced_ip_address_t* address, uint16_t* port )
{
    UINT result;
    UINT p;

    result = nxd_udp_source_extract( packet, (NXD_ADDRESS*) address, &p );
    if ( result == NX_SUCCESS )
    {
        *port = (uint16_t)p;
    }
    return netx_returns[result];
}

wiced_result_t wiced_packet_get_data( wiced_packet_t* packet, uint16_t offset, uint8_t** data, uint16_t* fragment_available_data_length, uint16_t *total_available_data_length )
{

    NX_PACKET* first_packet      = packet;
    NX_PACKET* iter              = packet;
    uint16_t   requested_offset  = offset;
    uint16_t   fragment_length;

    wiced_assert( "Bad args", (packet != NULL) && (data != NULL) && (fragment_available_data_length != NULL) && (total_available_data_length != NULL) );

    while ( iter != NULL )
    {
        /* It is more appropriate to use the difference between nx_packet_append_ptr and nx_packet_prepend_ptr rather
         * than nx_packet_length. If the packet was fragmented the nx_packet_length will reflect the sum of the data length of all
         * fragments, not the length of the fragment alone */
        fragment_length = (uint16_t)( iter->nx_packet_append_ptr - iter->nx_packet_prepend_ptr );
        if ( iter->nx_packet_length == 0 )
        {
            *data                           = NULL;
            *fragment_available_data_length = 0;
            *total_available_data_length    = 0;
            return WICED_TCPIP_SUCCESS;
        }
        else if ( offset < fragment_length )
        {
            *data = iter->nx_packet_prepend_ptr + offset;
            *fragment_available_data_length = (uint16_t) ( iter->nx_packet_append_ptr - *data );

            /* This will give number of bytes available after this offset including this packet and further packets in the chain */
            *total_available_data_length    = (uint16_t) ( first_packet->nx_packet_length - requested_offset );
            return WICED_TCPIP_SUCCESS;
        }
        else
        {
            offset = (uint16_t) ( offset - ( iter->nx_packet_append_ptr - iter->nx_packet_prepend_ptr ) );
        }
        iter = iter->nx_packet_next;
    }

    return WICED_TCPIP_ERROR;
}

wiced_result_t wiced_packet_pool_init( wiced_packet_pool_ref packet_pool, uint8_t* memory_pointer, uint32_t memory_size, char *pool_name )
{
    void* memory_pointer_aligned = PLATFORM_L1_CACHE_PTR_ROUND_UP( memory_pointer );
    uint32_t memory_size_aligned = PLATFORM_L1_CACHE_ROUND_DOWN  ( memory_size - ( (uint32_t)memory_pointer_aligned - (uint32_t)memory_pointer ) );
    UINT result;

    if (memory_pointer == NULL || packet_pool == NULL)
    {
        return WICED_TCPIP_ERROR;
    }

    result = nx_packet_pool_create(&packet_pool->pool, pool_name, WICED_LINK_MTU_ALIGNED, memory_pointer_aligned, memory_size_aligned);
    if (result != NX_SUCCESS)
    {
        return netx_returns[result];
    }

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_packet_pool_allocate_packet( wiced_packet_pool_ref packet_pool, wiced_packet_type_t packet_type, wiced_packet_t** packet, uint8_t** data, uint16_t* available_space, uint32_t timeout )
{
    ULONG type;
    UINT result;

    switch (packet_type)
    {
        case WICED_PACKET_TYPE_RAW: type = 0;             break;
        case WICED_PACKET_TYPE_IP:  type = NX_IP_PACKET;  break;
        case WICED_PACKET_TYPE_TCP: type = NX_TCP_PACKET; break;
        case WICED_PACKET_TYPE_UDP: type = NX_UDP_PACKET; break;
        default: return WICED_TCPIP_ERROR;
    }

    result = nx_packet_allocate(&packet_pool->pool, packet, type, NX_TIMEOUT(timeout));
    if (result != NX_SUCCESS)
    {
        *packet = NULL;
        *data   = NULL;
        *available_space = 0;
        return netx_returns[result];
    }

    *data = (*packet)->nx_packet_prepend_ptr;
    *available_space = (uint16_t)((*packet)->nx_packet_data_end - (*packet)->nx_packet_prepend_ptr);

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_packet_pool_deinit( wiced_packet_pool_ref packet_pool )
{
    UINT result;

    if (packet_pool == NULL)
    {
        return WICED_TCPIP_ERROR;
    }

    result = nx_packet_pool_delete(&packet_pool->pool);
    if (result != NX_SUCCESS)
    {
        return netx_returns[result];
    }

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_ip_get_ipv4_address( wiced_interface_t interface, wiced_ip_address_t* ipv4_address )
{
    ULONG temp;
    ipv4_address->version = WICED_IPV4;
    nx_ip_address_get( &IP_HANDLE(interface), (ULONG*)&ipv4_address->ip.v4, &temp );
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_ip_get_ipv6_address( wiced_interface_t interface, wiced_ip_address_t* ipv6_address, wiced_ipv6_address_type_t address_type )
{

    ULONG    ipv6_prefix;
    UINT     ipv6_interface_index;
    uint32_t i;
    UINT     result;

    for ( i = 0; i < NX_MAX_IPV6_ADDRESSES; i++ )
    {
        if ( IP_HANDLE(interface).nx_ipv6_address[i].nxd_ipv6_address_state == NX_IPV6_ADDR_STATE_VALID )
        {
            NXD_ADDRESS ip_address;

            result = nxd_ipv6_address_get( &IP_HANDLE(interface), i, &ip_address, &ipv6_prefix, &ipv6_interface_index );
            if ( result != NX_SUCCESS )
            {
                return netx_returns[result];
            }

            if ( address_type == IPv6_GLOBAL_ADDRESS )
            {
                if ( ( ip_address.nxd_ip_address.v6[0] & 0xf0000000 ) == 0x30000000 || ( ip_address.nxd_ip_address.v6[0] & 0xf0000000 ) == 0x20000000 )
                {
                    memcpy(ipv6_address, &ip_address, sizeof(NXD_ADDRESS));
                    return WICED_TCPIP_SUCCESS;
                }
            }
            else if ( address_type == IPv6_LINK_LOCAL_ADDRESS )
            {
                if ( ( ip_address.nxd_ip_address.v6[0] & 0xffff0000 ) == 0xfe800000 )
                {
                    memcpy(ipv6_address, &ip_address, sizeof(NXD_ADDRESS));
                    return WICED_TCPIP_SUCCESS;
                }
            }
            else
            {
                wiced_assert("Wrong address type", 0!=0 );
                return WICED_TCPIP_ERROR;
            }
        }
    }

    return WICED_TCPIP_ERROR;
}


wiced_result_t wiced_udp_create_socket( wiced_udp_socket_t* socket, uint16_t port, wiced_interface_t interface )
{
    UINT  result;

    WICED_LINK_CHECK( interface );

    memset( socket, 0, sizeof(wiced_udp_socket_t) );

    /* Set magic number for validating wiced_udp_socket_t object and differentiating with a native NX_UDP_SOCKET object */
    socket->socket_magic_number = WICED_SOCKET_MAGIC_NUMBER;

    result = nx_udp_socket_create( &IP_HANDLE(interface), &socket->socket, (char*)"WICEDsock", 0, NX_DONT_FRAGMENT, 255, WICED_DEFAULT_UDP_QUEUE_SIZE );
    if ( result != NX_SUCCESS )
    {
        wiced_assert( "Socket creation error", 0 != 0 );
        return netx_returns[ result ];
    }

    result = nx_udp_socket_bind( &socket->socket, ( port == WICED_ANY_PORT )? NX_ANY_PORT : (UINT) port, NX_TIMEOUT( WICED_UDP_BIND_TIMEOUT ) );
    if ( result != NX_SUCCESS )
    {
        nx_udp_socket_delete( &socket->socket );
        return netx_returns[ result ];
    }

    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_udp_update_socket_backlog( wiced_udp_socket_t* socket, uint32_t backlog )
{
    wiced_assert("Bad args", socket != NULL);
    socket->socket.nx_udp_socket_queue_maximum = backlog;
    return WICED_TCPIP_SUCCESS;
}

void wiced_udp_set_type_of_service( wiced_udp_socket_t* socket, uint32_t tos )
{
    socket->socket.nx_udp_socket_type_of_service = tos << 16;
}

wiced_result_t wiced_udp_send( wiced_udp_socket_t* socket, const wiced_ip_address_t* address, uint16_t port, wiced_packet_t* packet )
{
    UINT  result;

    WICED_LINK_CHECK_UDP_SOCKET( socket );

    result = nxd_udp_socket_send( &socket->socket, packet, (NXD_ADDRESS*) address, port );
    return netx_returns[result];
}

wiced_result_t wiced_udp_reply( wiced_udp_socket_t* socket, wiced_packet_t* in_packet, wiced_packet_t* out_packet )
{
    wiced_ip_address_t source_ip;
    UINT               source_port;

    wiced_assert("Bad args", (socket != NULL) && (in_packet != NULL) && (out_packet != NULL));

    WICED_LINK_CHECK_UDP_SOCKET( socket );

    if ((nxd_udp_source_extract( (NX_PACKET*) in_packet, (NXD_ADDRESS*) &source_ip, &source_port )) != NX_SUCCESS)
    {
        return WICED_TCPIP_ERROR;
    }

    return wiced_udp_send( socket, &source_ip, (uint16_t) source_port, out_packet );
}

wiced_result_t wiced_udp_receive( wiced_udp_socket_t* socket, wiced_packet_t** packet, uint32_t timeout )
{
    UINT result;

    WICED_LINK_CHECK_UDP_SOCKET( socket );

    result = nx_udp_socket_receive( &socket->socket, packet, NX_TIMEOUT(timeout) );

    if ( result == NX_SUCCESS )
    {
        NX_PACKET* temp_packet = *packet;
        if ( (uint16_t) ( temp_packet->nx_packet_append_ptr - temp_packet->nx_packet_prepend_ptr ) != (uint16_t) temp_packet->nx_packet_length )
        {
            uint32_t  diff = (uint32_t) temp_packet->nx_packet_length - (uint32_t) ( temp_packet->nx_packet_append_ptr - temp_packet->nx_packet_prepend_ptr );

            /* Packet has been chained due to IP fragmentation. If we have enough space left in the first packet in the chain */
            /* We will append data from the next chained packet to its current end and set a new end of the packet */
            if ( diff <= (uint32_t) ( temp_packet->nx_packet_data_end - temp_packet->nx_packet_append_ptr ) )
            {
                if ( temp_packet->nx_packet_next != NULL )
                {
                    uint16_t next_packet_chunk_length =(uint16_t)( temp_packet->nx_packet_next->nx_packet_append_ptr - temp_packet->nx_packet_next->nx_packet_prepend_ptr );

                    /* Copy data from the start of the chained packet to the end of the first packet in this chain */
                    memcpy( temp_packet->nx_packet_append_ptr, temp_packet->nx_packet_next->nx_packet_prepend_ptr, next_packet_chunk_length );

                    /* Set a new end of the packet */
                    temp_packet->nx_packet_append_ptr += next_packet_chunk_length;

                    /* Release a packet which was chained */
                    result = nx_packet_release( temp_packet->nx_packet_next );
                    temp_packet->nx_packet_next = NULL;
                }
            }
        }
    }

    return netx_returns[result];
}

wiced_result_t wiced_udp_delete_socket( wiced_udp_socket_t* socket )
{
    UINT result;
    UNUSED_PARAMETER(socket);

    /* Check if socket is still bound */
    if ( socket->socket.nx_udp_socket_bound_next != NULL )
    {
        nx_udp_socket_unbind( &socket->socket );
    }

#ifndef WICED_CONFIG_DISABLE_DTLS
    if ( socket->dtls_context != NULL )
    {
        wiced_dtls_close_notify( socket );

        wiced_dtls_deinit_context( socket->dtls_context );

        if ( socket->context_malloced == WICED_TRUE )
        {
            free( socket->dtls_context );
            socket->dtls_context = NULL;
            socket->context_malloced = WICED_FALSE;
        }
    }
#endif /* ifndef WICED_CONFIG_DISABLE_DTLS */

    result = nx_udp_socket_delete( &socket->socket );

    /* Invalidate UDP socket */
    socket->socket_magic_number = 0;

    return netx_returns[result];
}

wiced_result_t wiced_udp_register_callbacks( wiced_udp_socket_t* socket, wiced_udp_socket_callback_t receive_callback, void* arg )
{
    socket->receive_callback = receive_callback;
    socket->callback_arg     = arg;
    nx_udp_socket_receive_notify( &socket->socket, internal_nx_udp_socket_receive_callback );
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_udp_unregister_callbacks( wiced_udp_socket_t* socket )
{
   socket->receive_callback = 0;
   return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_multicast_join( wiced_interface_t interface, const wiced_ip_address_t* address )
{
    NXD_ADDRESS group_address;
    UINT result = NX_IP_ADDRESS_ERROR;
    WICED_LINK_CHECK( interface );

    if ( address->version == WICED_IPV4 )
    {
        result = nx_igmp_multicast_join( &IP_HANDLE( interface ), address->ip.v4 );
    }
    else if ( address->version == WICED_IPV6 )
    {
        group_address.nxd_ip_version       = NX_IP_VERSION_V6;
        group_address.nxd_ip_address.v6[0] = address->ip.v6[0];
        group_address.nxd_ip_address.v6[1] = address->ip.v6[1];
        group_address.nxd_ip_address.v6[2] = address->ip.v6[2];
        group_address.nxd_ip_address.v6[3] = address->ip.v6[3];

        result = nxd_ipv6_multicast_interface_join( &IP_HANDLE( interface ), &group_address, PRIMARY_INTERFACE );
    }
    return netx_returns[result];
}

wiced_result_t wiced_multicast_leave( wiced_interface_t interface, const wiced_ip_address_t* address )
{
    NXD_ADDRESS group_address;
    WICED_LINK_CHECK( interface );

    if ( address->version == WICED_IPV4 )
    {
        nx_igmp_multicast_leave( &IP_HANDLE( interface ), address->ip.v4 );
    }
    else if ( address->version == WICED_IPV6 )
    {
        group_address.nxd_ip_version       = NX_IP_VERSION_V6;
        group_address.nxd_ip_address.v6[0] = address->ip.v6[0];
        group_address.nxd_ip_address.v6[1] = address->ip.v6[1];
        group_address.nxd_ip_address.v6[2] = address->ip.v6[2];
        group_address.nxd_ip_address.v6[3] = address->ip.v6[3];

        nxd_ipv6_multicast_interface_leave( &IP_HANDLE( interface ), &group_address, PRIMARY_INTERFACE );
    }
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_ping( wiced_interface_t interface, const wiced_ip_address_t* address, uint32_t timeout_ms, uint32_t* elapsed_ms )
{
    UINT         result;
    wiced_time_t send_time;
    wiced_time_t reply_time;
    NX_PACKET*   response_ptr;

    WICED_LINK_CHECK( interface );

    if ( address->version != WICED_IPV4 )
    {
        wiced_assert("ping for ipv6 not implemented yet", 0!=0);
        return WICED_TCPIP_UNSUPPORTED;
    }

    send_time     = host_rtos_get_time( );
    result        = nxd_icmp_ping( &IP_HANDLE(interface), (NXD_ADDRESS*)address, NULL, 0, &response_ptr, NX_TIMEOUT(timeout_ms) );
    reply_time    = host_rtos_get_time( );
    if ( result == NX_SUCCESS )
    {
        *elapsed_ms = (reply_time - send_time)*1000/SYSTICK_FREQUENCY;
        result = nx_packet_release( response_ptr );
    }
    else if ( result == NX_NO_RESPONSE )
    {
        return WICED_TCPIP_TIMEOUT;
    }

    return netx_returns[result];
}

wiced_result_t wiced_ip_get_gateway_address( wiced_interface_t interface, wiced_ip_address_t* ipv4_address )
{
    SET_IPV4_ADDRESS( *ipv4_address, IP_HANDLE(interface).nx_ip_gateway_address );
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_ip_get_netmask( wiced_interface_t interface, wiced_ip_address_t* ipv4_address )
{
    SET_IPV4_ADDRESS( *ipv4_address, IP_HANDLE(interface).nx_ip_network_mask );
    return WICED_TCPIP_SUCCESS;
}

wiced_result_t wiced_tcp_server_peer( wiced_tcp_socket_t* socket, wiced_ip_address_t* address, uint16_t* port )
{
    UINT  result;
    ULONG peer_port;
    ULONG peer_address;

    if (socket->socket.nx_tcp_socket_connect_ip.nxd_ip_version == NX_IP_VERSION_V6)
    {
        address->version = WICED_IPV6;
        memcpy(address->ip.v6, socket->socket.nx_tcp_socket_connect_ip.nxd_ip_address.v6, 16);
        *port = (uint16_t)socket->socket.nx_tcp_socket_connect_port;
        return WICED_TCPIP_SUCCESS;
    }

    /* IPv4 */
    result = nx_tcp_socket_peer_info_get( &socket->socket, &peer_address, &peer_port );
    if ( result == NX_SUCCESS )
    {
        *port = (uint16_t)peer_port;
        SET_IPV4_ADDRESS( *address, peer_address );
        return WICED_TCPIP_SUCCESS;
    }

    return netx_returns[result];
}

wiced_result_t wiced_tcp_server_enable_tls( wiced_tcp_server_t* tcp_server, wiced_tls_identity_t* tls_identity )
{
    tcp_server->tls_identity = tls_identity;

    return WICED_TCPIP_SUCCESS;
}

/******************************************************
 *            Static Function Definitions
 ******************************************************/

static void internal_nx_tcp_socket_disconnect_callback( NX_TCP_SOCKET* socket_ptr )
{
    wiced_tcp_socket_t* socket = (wiced_tcp_socket_t*)socket_ptr;
    internal_defer_tcp_callback_to_wiced_network_thread( socket, socket->callbacks.disconnect );
}

static void internal_nx_tcp_socket_receive_callback( NX_TCP_SOCKET* socket_ptr )
{
    wiced_tcp_socket_t* socket = (wiced_tcp_socket_t*)socket_ptr;
    internal_defer_tcp_callback_to_wiced_network_thread( socket, socket->callbacks.receive );
}

static VOID internal_nx_tcp_listen_callback( NX_TCP_SOCKET* socket_ptr, UINT port )
{
    wiced_tcp_socket_t* socket = (wiced_tcp_socket_t*)socket_ptr;
    UNUSED_PARAMETER( port );

    internal_defer_tcp_callback_to_wiced_network_thread( socket, socket->callbacks.connect );
}

static void internal_nx_udp_socket_receive_callback( NX_UDP_SOCKET* socket_ptr )
{
    wiced_udp_socket_t* socket = (wiced_udp_socket_t*)socket_ptr;

    if ( socket->socket_magic_number == WICED_SOCKET_MAGIC_NUMBER )
    {
        internal_defer_udp_callback_to_wiced_network_thread( socket );
    }
}

static wiced_result_t internal_wiced_tcp_server_listen( wiced_tcp_server_t* tcp_server )
{
    wiced_tcp_socket_t* free_socket = NULL;
    linked_list_node_t* socket_node = NULL;
    wiced_tcp_socket_t* tcp_socket;
    wiced_tcp_server_socket_t* aligned_serv_sock;
    UINT                status;

    linked_list_get_front_node( &tcp_server->socket_list, &socket_node );
    /* Find a socket that is either listening or free to listen */
    while( socket_node != NULL )
    {
        aligned_serv_sock = (wiced_tcp_server_socket_t*)socket_node->data;
        tcp_socket = &aligned_serv_sock->socket;

        if ( tcp_socket->socket.nx_tcp_socket_state == NX_TCP_LISTEN_STATE )
        {
            return WICED_SUCCESS;
        }
        else if ( tcp_socket->socket.nx_tcp_socket_state == NX_TCP_CLOSED )
        {
            free_socket = tcp_socket;
        }
        else if ( tcp_socket->socket.nx_tcp_socket_state == NX_TCP_CLOSE_WAIT )
        {
            nx_tcp_socket_disconnect( &tcp_socket->socket, NX_NO_WAIT );
        }

        socket_node = socket_node->next;
    }

    if ( free_socket == NULL )
    {
        return WICED_ERROR;
    }

    status = nx_tcp_server_socket_relisten( free_socket->socket.nx_tcp_socket_ip_ptr, tcp_server->port, &free_socket->socket );

    if ( status == NX_ALREADY_BOUND )
    {
        nx_tcp_server_socket_unaccept( &free_socket->socket );
        status = nx_tcp_server_socket_relisten( free_socket->socket.nx_tcp_socket_ip_ptr, tcp_server->port, &free_socket->socket );
    }
    return netx_returns[ status ];
}

wiced_bool_t wiced_network_interface_is_up( NX_IP* ip_handle )
{
#ifdef WICED_USE_ETHERNET_INTERFACE
    if ( &IP_HANDLE( WICED_ETHERNET_INTERFACE ) == ip_handle )
    {
        return ip_handle->nx_ip_initialize_done ? WICED_TRUE : WICED_FALSE;
    }
    else
#endif
    {
        return ip_handle->nx_ip_driver_link_up ? WICED_TRUE : WICED_FALSE;
    }
}
