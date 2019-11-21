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
 *  COAP Server :
 *
 *  Functionality :
 *
 *   -> Can handle GET.POST,DELETE methods from coap client
 *   -> Can handle CON(with Ack) and NONCON(without Ack) requests.
 *   -> Can handle ping request and send response
 *   -> Can handle discovery of all the resources.
 *   -> Can subscribe for particular service and get notificaion when state of that service changes.
 *   -> Retransmission for unacknowledged notification.
 *   -> Can handle removing of observers by unregistering them.
 *
 *
 */

#include "coap_server.h"
#include "coap_server_internal.h"
#include "parser/coap_parser.h"

#include "wiced_dtls.h"
#include "wiced_udpip_dtls_api.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define COAP_TARGET_PORT                     (5683)
#define UDP_MAX_DATA_LENGTH                  (1024)
#define COAP_SERVER_THREAD_PRIORITY          (WICED_DEFAULT_LIBRARY_PRIORITY)
#define COAP_SERVER_STACK_SIZE               (6200)
#define COAP_SERVER_EVENT_QUEUE_DEPTH        (10)
#define COAP_DISCOVERY_SERVICE_FORMAT_LENGTH (8)
/* Pool size available to accommodate all available registered service. */
#define MAX_DISCOVERY_SERVICES_BUF_SIZE      (500)
#define COAP_RETRANSMISISON_TIMER            (1000)
#define UDP_QUEUE_SIZE                       (10)


#define DTLS_SERVER_IP MAKE_IPV4_ADDRESS(192,168,0,1)
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    wiced_udp_socket_t* socket;
    coap_server_event_t event_type;
    void                *data;
} server_event_message_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t coap_timed_event( void* arg );
static wiced_result_t coap_internal_send_response( void* context, wiced_coap_server_service_t * service, void* req_handle, wiced_coap_server_response_t * response, coap_responsecode_t response_code, wiced_coap_notification_type notification_type, wiced_bool_t from_api );
static wiced_bool_t compare_service( linked_list_node_t* node_to_compare, void* user_data );
static void list_init( wiced_coap_server_t* server );

/* TODO: Reuse some of these and move to parser.c based on need for client and server */
static wiced_result_t coap_server_receive_callback( wiced_udp_socket_t* socket, void* args );
static wiced_result_t coap_server_process_request( wiced_coap_server_t* server, wiced_packet_t* packet, uint16_t port );
static wiced_result_t coap_server_parse_packet( coap_packet_t* request, uint8_t* data, int data_length, coap_responsecode_t* response_code );
static wiced_result_t coap_send_response( coap_send_response_t *client_response, wiced_bool_t fromapi );
static wiced_result_t coap_delete_service( coap_service_t* service );
static wiced_result_t send_udp_response( wiced_udp_socket_t* udp_socket, uint8_t* data, int data_len, wiced_ip_address_t ip, uint16_t port );
static wiced_result_t handle_well_known_core( void* context, wiced_coap_server_service_t* service, wiced_coap_server_request_t* request );
static coap_observer_t* coap_add_observer( wiced_coap_server_service_t* service, coap_request_info_t* request );
static coap_transaction_t* coap_new_transaction( wiced_coap_server_t* server, coap_observer_t* observer, wiced_coap_server_response_t response );
static wiced_result_t coap_find_service( wiced_coap_server_t* server, coap_request_info_t* pkt, wiced_coap_server_service_t** service );
static wiced_result_t coap_notify( wiced_coap_server_t* server, coap_observer_t* observer, wiced_coap_server_service_t* service, wiced_coap_server_response_t response, wiced_coap_notification_type notification_type, coap_responsecode_t response_code );
static wiced_result_t coap_find_service_by_name( wiced_coap_server_t* server, uint8_t* service_name, wiced_coap_server_service_t** service_found );
static void coap_handle_methods( coap_request_info_t* request, coap_responsecode_t *response_code );
static void coap_handle_ack( wiced_coap_server_t* server, wiced_coap_server_service_t* service, coap_request_info_t* request_info );
static void coap_send_transaction( coap_transaction_t *t, wiced_coap_server_t* server );
static void coap_remove_observer( wiced_coap_server_service_t* service, wiced_ip_address_t client_ip, uint16_t port );
static void coap_free_transaction( wiced_coap_server_t* server, coap_transaction_t* transaction );
static void coap_server_thread_main( uint32_t arg );
static void coap_check_transactions( wiced_coap_server_t* server );
static void coap_clear_transaction( wiced_coap_server_t* server, coap_request_info_t* request );
static void coap_notify_observers( coap_send_response_t* client_response );
static void coap_add_service( coap_service_t* service );
static int coap_well_known_services( wiced_coap_server_service_t* known_services, char *buf );
static wiced_result_t coap_get_service_name( const coap_request_info_t* pkt, uint8_t num, char *data );

static int dtls_event_callback( void *socket, void *args );
/******************************************************
 *               Variables Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_coap_server_init( wiced_coap_server_t* server )
{
    wiced_result_t result;

    WICED_VERIFY( wiced_rtos_init_queue( &server->event_queue, NULL, sizeof(server_event_message_t), COAP_SERVER_EVENT_QUEUE_DEPTH ) );

    server->quit = WICED_FALSE;

    if ( ( result = wiced_rtos_create_thread( &server->event_thread, COAP_SERVER_THREAD_PRIORITY, "COAPserver", coap_server_thread_main, COAP_SERVER_STACK_SIZE, server ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Error in creation of CoAP thread\n"));
        return result;
    }

    /* Setup a regular timed event to check for retransmisison of packets after timeout. */
    return wiced_rtos_register_timed_event( &server->coap_timer_event, WICED_NETWORKING_WORKER_THREAD, &coap_timed_event, COAP_RETRANSMISISON_TIMER, server );
}

wiced_result_t wiced_coap_server_deinit( wiced_coap_server_t* server )
{
    server_event_message_t current_event;
    wiced_result_t result;

    current_event.event_type = COAP_SERVER_DEINIT;
    current_event.socket = &server->socket;
    current_event.data = NULL;

    wiced_rtos_deregister_timed_event( &server->coap_timer_event );

    result = wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Error in push to queue\n"));
        return result;
    }

    if ( wiced_rtos_is_current_thread( &server->event_thread ) != WICED_SUCCESS )
    {
        wiced_rtos_thread_force_awake( &server->event_thread );
    }

    wiced_rtos_thread_join( &server->event_thread );
    wiced_rtos_delete_thread( &server->event_thread );

    wiced_rtos_deinit_queue( &server->event_queue );

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_server_start( wiced_coap_server_t* server, wiced_interface_t interface, uint16_t port, wiced_coap_security_t *security )
{
    wiced_result_t result;
    const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( target_ip_addr, DTLS_SERVER_IP );

    UNUSED_PARAMETER(target_ip_addr);

    server->discovery_services_bytes_used = 0;

    if ( ( result = wiced_udp_create_socket( &server->socket, port, interface ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Socket creation failed\n"));
        return result;
    }

    /* Load key and enable DTLS */
    if ( security != NULL )
    {
        /* configure UDP socket to queue up the 10 packets. This is needed for DTLS server to handle multiple requests from multiple clients */
        if ( ( result = wiced_udp_update_socket_backlog ( &server->socket, UDP_QUEUE_SIZE ) != WICED_SUCCESS ) )
        {
            WPRINT_LIB_ERROR(( "Unable to update UDP socket backlog\n" ));
            return result;
        }

        if ( ( result = wiced_udp_enable_dtls( &server->socket, &server->context )) != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR(( "Failed to enable DTLS\n" ));
            return result;
        }

        if ( security->type == WICED_COAP_SECURITY_TYPE_PSK )
        {
            result = wiced_dtls_init_identity( &server->identity, WICED_DTLS_SECURITY_TYPE_PSK, security->args.psk_info );
            if ( result != WICED_SUCCESS )
            {
                WPRINT_LIB_ERROR(( "Unable to initialize DTLS NONPSK identity. Error = [%d]\n", result ));
                return result;
            }
        }
        else if ( security->type == WICED_COAP_SECURITY_TYPE_NONPSK )
        {
            result = wiced_dtls_init_identity( &server->identity, WICED_DTLS_SECURITY_TYPE_NONPSK, &security->args.cert_info );
            if ( result != WICED_SUCCESS )
            {
                WPRINT_LIB_ERROR(( "Unable to initialize DTLS NONPSK identity. Error = [%d]\n", result ));
                return result;
            }
        }
        else
        {
            return WICED_ERROR;
        }

        result = wiced_dtls_init_context (server->socket.dtls_context, &server->identity, NULL);
        if ( result != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR(( "Unable to initialize DTLS context. Error = [%d]\n", result ));
            return result;
        }

        /* Register callback to get application data from client once handshake has been completed */
        server->socket.dtls_context->callback = (wiced_dtls_callback_t) dtls_event_callback;
        server->socket.dtls_context->callback_arg = server;

        result = wiced_udp_start_dtls (&server->socket, target_ip_addr, WICED_DTLS_AS_SERVER, DTLS_NO_VERIFICATION);
        if ( result != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR(( "Unable to start DTLS Server. Error = [%d]\n", result ));
            return result;
        }
    }
    else
    {
        /* Register callback function for the UDP data. */
        if ( ( result = wiced_udp_register_callbacks( &server->socket, coap_server_receive_callback, server ) ) != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR(("Error in registering udp callback\n"));
            return result;
        }
    }

    /* Initialize linked list for service and transaction for server */
    list_init( server );

    /* Register well-known service with server */
    if ( ( result = wiced_coap_server_add_service( server, &server->discovery_service, ".well-known/core", handle_well_known_core, COAP_INTERNAL_CONTENTTYPE_APPLICATION_LINKFORMAT ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (("Error in adding service\n"));
        return result;
    }

    return WICED_SUCCESS;
}

void list_init( wiced_coap_server_t* server )
{
    linked_list_init( &server->transaction_list );
    linked_list_init( &server->service_list );
}

static void list_deinit( wiced_coap_server_t* server )
{
    linked_list_deinit( &server->transaction_list );
    linked_list_deinit( &server->service_list );
}

static void coap_server_thread_main( uint32_t arg )
{
    wiced_coap_server_t* server = (wiced_coap_server_t*) arg;
    coap_service_t* service = NULL;
    wiced_packet_t* packet = NULL;
    coap_send_response_t* response = NULL;
    server_event_message_t current_event;
    wiced_result_t result;
    dtls_peer_data* peer_data = NULL;
    wiced_coap_security_t* security;
    coap_server_stop_t* data = NULL;

    UNUSED_PARAMETER(security);

    while ( server->quit != WICED_TRUE )
    {
        result = wiced_rtos_pop_from_queue( &server->event_queue, &current_event, WICED_NEVER_TIMEOUT );

        wiced_assert("unable to pop from queue : coap_server_thread_main", result == WICED_SUCCESS);

        switch ( current_event.event_type )
        {
            case COAP_RECEIVE_EVENT:
                /* Receive Event */
                if ( server->socket.dtls_context != NULL )
                {
                    peer_data = (dtls_peer_data*) current_event.data;
                    packet = (wiced_packet_t*) peer_data->packet;
                    if ( coap_server_process_request( server, (wiced_packet_t*)packet, peer_data->session.port ) != WICED_SUCCESS )
                    {
                        WPRINT_LIB_ERROR (("Error in processing client request\n"));
                    }
                    free (peer_data);
                }
                else
                {
                    wiced_ip_address_t ip;
                    uint16_t port;

                    packet = (wiced_packet_t*) current_event.data;
                    wiced_udp_packet_get_info( packet, &ip, &port );

                    if ( coap_server_process_request( server, (wiced_packet_t*)packet, port ) != WICED_SUCCESS )
                    {
                        WPRINT_LIB_ERROR (("Error in processing client request\n"));
                    }
                }
                wiced_packet_delete( packet );
                break;

            case COAP_SEND_EVENT:
                /* Send Event */
                response = (coap_send_response_t*) current_event.data;
                if ( coap_send_response( response, WICED_TRUE ) != WICED_SUCCESS )
                {
                    WPRINT_LIB_ERROR (("Error in sending response back to client\n"));
                }
                break;

            case COAP_TIMER_EVENT:
                /* Retransmission Timer Event */
                coap_check_transactions( server );
                break;

            case COAP_ADD_SERVICE_EVENT:
                /* Add New Service Event */
                service = (coap_service_t*) current_event.data;
                coap_add_service( service );
                break;

            case COAP_DELETE_SERVICE_EVENT:
                /* Delete Service Event */
                service = (coap_service_t*) current_event.data;
                coap_delete_service( service );
                break;

            case COAP_SERVER_STOP:
                data = (coap_server_stop_t*) current_event.data;
                /* Stop COAP server Event */
                if ( data->security != NULL )
                {
                    security = (wiced_coap_security_t*) data->security;
                    wiced_dtls_deinit_identity(server->socket.dtls_context->identity, security->type );
                    wiced_dtls_deinit_context( server->socket.dtls_context );
                }
                wiced_udp_unregister_callbacks( &server->socket );
                wiced_udp_delete_socket( &server->socket );
                list_deinit( server );
                wiced_rtos_set_semaphore( &data->stop_semaphore );
                break;

            case COAP_SERVER_DEINIT:
                server->quit = WICED_TRUE;
                break;

            default:
                wiced_assert("Wrong Event : coap_server_thread_main",1);
                break;
        }
    }
}

static wiced_result_t coap_delete_service( coap_service_t* service )
{
    wiced_assert( "bad arg", ( service->service != NULL ) );

    coap_observer_t* current_observer;
    wiced_coap_server_t* server = service->server;
    wiced_coap_server_service_t* current_service = service->service;
    wiced_coap_server_response_t response;
    wiced_result_t result;

    UNUSED_PARAMETER(result);

    memset( &response, 0, sizeof(wiced_coap_server_response_t) );

    /* If any observers are associated with this service then before deleting
     * service from server list notify to all the registered observers and
     * then delete service.
     */

    linked_list_get_front_node( &current_service->observer_list, (linked_list_node_t**) &current_observer );

    while ( current_observer != NULL )
    {
        coap_observer_t* observer = (coap_observer_t*) current_observer;

        if ( coap_notify( server, observer, current_service, response, WICED_COAP_MSGTYPE_CON, COAP_RSPCODE_NOT_FOUND ) != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR(("Error in notifying to observer\n"));
        }

        current_observer = (coap_observer_t*) observer->this_node.next;

        result = linked_list_remove_node( &current_service->observer_list, &observer->this_node );

        result = coap_release_buffer( observer );
    }

    result = linked_list_remove_node( &server->service_list, &current_service->this_node );

    free( service );

    return WICED_SUCCESS;

}

static void coap_add_service( coap_service_t* new_service )
{
    wiced_coap_server_t* server = new_service->server;
    wiced_coap_server_service_t* service = new_service->service;

    linked_list_insert_node_at_rear( &server->service_list, &service->this_node );

}

static void coap_check_transactions( wiced_coap_server_t* server )
{
    coap_transaction_t* current_node;

    linked_list_get_front_node( &server->transaction_list, (linked_list_node_t**) &current_node );
    while ( current_node != NULL )
    {
        coap_transaction_t* transaction = (coap_transaction_t*) current_node;

        current_node = (coap_transaction_t*) transaction->this_node.next;
        if ( transaction->retrans_timer == 0 )
        {
            ++( transaction->retrans_counter );
            coap_send_transaction( transaction, server );
        }
        else
        {
            transaction->retrans_timer-- ;
        }
    }
}

static void coap_send_transaction( coap_transaction_t* transaction, wiced_coap_server_t* server )
{
    /* retransmission is only needed for response type of connection because we are
     * accepting ACK from client side if no ACK within timeout then retransmit again. but
     * there is no retransmission for NONCON packet so just delete transaction.
     */

    if ( WICED_COAP_MSGTYPE_CON == ( ( COAP_HEADER_TYPE_MASK & transaction->packet[ 0 ] ) >> COAP_HEADER_TYPE_POSITION ) )
    {
        if ( transaction->retrans_counter < COAP_MAX_RETRANSMIT )
        {
            if ( send_udp_response( &server->socket, transaction->packet, transaction->packet_len, transaction->addr, transaction->port ) != WICED_SUCCESS )
            {
                WPRINT_LIB_ERROR(("Error in sending transaction to client\n"));
            }

            transaction->retrans_timer = ( transaction->reset_timer * 2 ); /* double */
            transaction->reset_timer = transaction->retrans_timer;
        }
        else
        {
            /* remove observer from list*/
            coap_remove_observer( transaction->service, transaction->addr, transaction->port );

            coap_free_transaction( server, transaction );
        }
    }
    else
    {
        /* Free the transaction as this is for NONCON type and we are not
         * expecting any ACK from client any retransamission will not be there.
         */

        coap_free_transaction( server, transaction );
    }
}

static void coap_free_transaction( wiced_coap_server_t* server, coap_transaction_t* transaction )
{
    wiced_assert( "bad arg", ( transaction != NULL ) );

    linked_list_remove_node( &server->transaction_list, &transaction->this_node );

    /* Release value buffer */
    coap_release_buffer( (void*) transaction );

}

static wiced_result_t coap_server_process_request( wiced_coap_server_t *server, wiced_packet_t *packet, uint16_t port )
{
    uint8_t* udp_data = NULL;
    uint16_t data_length, available_data_length, udp_src_port;
    wiced_result_t result;
    wiced_ip_address_t udp_src_ip_addr;
    coap_request_info_t request_info;
    wiced_coap_server_response_t response;
    coap_responsecode_t response_code = COAP_RSPCODE_NONE;
    wiced_coap_option_t* option;
    wiced_coap_server_service_t* service = NULL;
    wiced_coap_server_request_t request;

    memset( &request_info, 0, sizeof(coap_request_info_t) );
    memset( &response, 0, sizeof(wiced_coap_server_response_t) );

    /* Get info about the received UDP packet */
    wiced_udp_packet_get_info( packet, &udp_src_ip_addr, &udp_src_port );

    result = wiced_packet_get_data( packet, 0, (uint8_t**) &udp_data, &data_length, &available_data_length );
    if ( data_length < available_data_length )
    {
        WPRINT_LIB_ERROR (("Fragmented packets not supported\n"));
        return WICED_ERROR;
    }

    /* skip zero length packets */
    if(data_length == 0)
    {
        WPRINT_LIB_INFO (("Received Zero Length packet\n"));
        return WICED_SUCCESS;
    }
    udp_data[ data_length ] = '\0'; /* Null terminate the received string */

    /* Save client IP and Port No which will be used to send response back */
    request_info.client_ip = udp_src_ip_addr;

    if (server->socket.dtls_context != NULL)
    {
        request_info.port = port;
    }
    else
    {
        request_info.port = udp_src_port;
    }

    WPRINT_LIB_DEBUG (("UDP Rx Application data from IP %u.%u.%u.%u:%d\n",
               (unsigned char) ( ( GET_IPV4_ADDRESS(request_info.client_ip) >> 24 ) & 0xff ),
               (unsigned char) ( ( GET_IPV4_ADDRESS(request_info.client_ip) >> 16 ) & 0xff ),
               (unsigned char) ( ( GET_IPV4_ADDRESS(request_info.client_ip) >> 8 ) & 0xff ),
               (unsigned char) ( ( GET_IPV4_ADDRESS(request_info.client_ip) >> 0 ) & 0xff ),
               request_info.port ));

    /* Parse COAP Packet (Including Header, options and Payload) */
    if ( ( result = ( coap_server_parse_packet( &request_info.packet, udp_data, data_length, &response_code ) ) != WICED_SUCCESS ) )
    {
        WPRINT_LIB_ERROR (("Error in parsing coap packet\n"));
        coap_internal_send_response( server, service, (void*) &request_info, &response, response_code, WICED_COAP_MSGTYPE_CON, WICED_FALSE );
        return WICED_SUCCESS;
    }

    /* If we receive ACK or RST from client means client has received particular
     * packet so delete it from transaction list and if RST then client wants to_notify
     * deregister its interest for particular service so delete that observer entry.
     */

    if ( request_info.packet.hdr.t == WICED_COAP_MSGTYPE_ACK || request_info.packet.hdr.t == WICED_COAP_MSGTYPE_RESET )
    {
        coap_handle_ack( server, service, &request_info );
        return WICED_SUCCESS;
    }

    /* This is for Ping request if Ping request comes then send response back */
    if ( request_info.packet.hdr.code == COAP_INTERNAL_METHOD_EMPTY )
    {
        response_code = COAP_RSPCODE_EMPTY;
        coap_internal_send_response( server, service, (void*) &request_info, &response, response_code, COAP_NOTIFICATIONTYPE_NONE, WICED_FALSE );
        return WICED_SUCCESS;
    }

    result = coap_find_service( server, &request_info, &service );
    if(result != WICED_SUCCESS)
    {
        response_code = COAP_NOT_FOUND;
        coap_internal_send_response( server, service, &request_info, &response, response_code, COAP_NOTIFICATIONTYPE_NONE, WICED_FALSE );
        return WICED_SUCCESS;
    }
    /* check here that request is observable then add client to list */
    if ( ( ( option = coap_find_options( &request_info.packet, COAP_OPTION_OBSERVE ) ) != NULL ) )
    {
        if ( option->buf.len > 0 )
        {
            coap_remove_observer( service, request_info.client_ip, request_info.port );
            coap_internal_send_response( server, service, &request_info, &response, response_code, COAP_NOTIFICATIONTYPE_NONE, WICED_FALSE );
            return WICED_SUCCESS;
        }
    }

    if ( service )
    {
        request.method = request_info.packet.hdr.code;
        request.payload = (wiced_coap_buffer_t) request_info.packet.payload;
        request.req_handle = (void*) &request_info;

        service ->callback( server, service, (void*) &request );
    }

    return WICED_SUCCESS;
}

static void coap_handle_ack( wiced_coap_server_t* server, wiced_coap_server_service_t* service, coap_request_info_t* request_info )
{
    /* Remove particular transaction as we received ACK from client */
    coap_clear_transaction( server, request_info );

    /* Remove observer from list if we receive RESET from client */
    if ( request_info->packet.hdr.t == WICED_COAP_MSGTYPE_RESET )
    {
        coap_remove_observer( service, request_info->client_ip, request_info->port );
    }
}

static wiced_result_t coap_server_parse_packet( coap_packet_t* request, uint8_t *data, int data_length, coap_responsecode_t *response_code )
{
    wiced_result_t result;

    /* Parse coap header */
    if ( ( result = coap_parse_header( &request->hdr, (uint8_t *) data, data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Error in parsing header"));
        *response_code = COAP_RSPCODE_BAD_REQUEST;
        return WICED_ERROR;
    }

    /* Parse coap token (token length and value) */
    if ( ( result = coap_parse_token( &request->token, request->hdr.tkl, (uint8_t *) data, data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(( "Error in parsing coap token\n" ));
        *response_code = COAP_RSPCODE_BAD_REQUEST;
        return WICED_ERROR;
    }

    /* Parse coap options (take all the options ) */
    if ( ( result = coap_parse_options_and_payload( &request, (uint8_t *) data, data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(( "Error in parsing coap option and payload\n" ));
        *response_code = COAP_BAD_OPTION;
        return WICED_ERROR;
    }

    return WICED_SUCCESS;

}

static void coap_handle_methods( coap_request_info_t* request, coap_responsecode_t* response_code )
{
    /* If COAP_METHOD is empty then its ping request so just give response back */
    switch ( request->packet.hdr.code )
    {
        case COAP_INTERNAL_METHOD_EMPTY:
            *response_code = COAP_RSPCODE_EMPTY;
            break;

        case COAP_INTERNAL_METHOD_GET:
            *response_code = COAP_RSPCODE_CONTENT;
            break;

        case COAP_INTERNAL_METHOD_POST:
            *response_code = COAP_RSPCODE_CHANGED;
            break;

        case COAP_INTERNAL_METHOD_PUT:
            *response_code = COAP_RSPCODE_CHANGED;
            break;

        case COAP_INTERNAL_METHOD_DELETE:
            *response_code = COAP_RSPCODE_DELETED;
            break;

        default:
            *response_code = COAP_METHOD_NOT_ALLOWED;
            break;
    }

}

static wiced_result_t coap_find_service( wiced_coap_server_t* server, coap_request_info_t* pkt, wiced_coap_server_service_t** service )
{
    char buffer[ WICED_COAP_MAX_SERVICE_LENGTH ];
    wiced_result_t result;

    memset( buffer, 0, sizeof( buffer ) );

    result = coap_get_service_name( pkt, COAP_OPTION_URI_PATH, buffer );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }
    return coap_find_service_by_name( server, (uint8_t*) buffer, service );
}

/* Receive packet and process the coap request received from client */
static wiced_result_t coap_server_receive_callback( wiced_udp_socket_t *socket, void *args )
{
    server_event_message_t current_event;
    wiced_coap_server_t* server = (wiced_coap_server_t*) args;
    wiced_packet_t* packet;
    wiced_result_t result;

    if ( ( result = wiced_udp_receive( &server->socket, &packet, WICED_NO_WAIT ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (("Error in receiving UDP packet\n"));
        return result;
    }

    current_event.event_type = COAP_RECEIVE_EVENT;
    current_event.socket = socket;
    current_event.data = packet;
    wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_server_add_service( wiced_coap_server_t* server, wiced_coap_server_service_t* service, char* service_name, wiced_coap_server_callback service_handler, wiced_coap_content_type_t content_type )
{
    server_event_message_t current_event;
    static coap_service_t new_service;
    wiced_result_t result;

    wiced_assert( "bad arg", ( service_name != NULL ) && ( service_handler != NULL ));

    /* Create observer list */
    result = linked_list_init( &service->observer_list );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    server->discovery_services_bytes_used += ( strlen( service_name ) + COAP_DISCOVERY_SERVICE_FORMAT_LENGTH );

    if ( server->discovery_services_bytes_used > MAX_DISCOVERY_SERVICES_BUF_SIZE )
    {
        WPRINT_LIB_ERROR(("No memory to accommodate new service. update MAX_DISCOVERY_SERVICES_BUF_SIZE pool size\n"));
        return WICED_ERROR;
    }

    /* Copy content to new_service */
    strcpy( service->service_name, service_name );
    service->callback = service_handler;
    service->content_type = content_type;

    new_service.server = server;
    new_service.service = service;

    current_event.event_type = COAP_ADD_SERVICE_EVENT;
    current_event.data = (void*) &new_service;

    wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_server_delete_service( wiced_coap_server_t* server, wiced_coap_server_service_t* service )
{
    server_event_message_t current_event;
    coap_service_t* new_service = malloc( sizeof(coap_service_t) );

    new_service->server = server;
    new_service->service = service;

    current_event.event_type = COAP_DELETE_SERVICE_EVENT;
    current_event.data = (void*) new_service;

    wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_server_stop( wiced_coap_server_t *server, wiced_coap_security_t *security )
{
    server_event_message_t current_event;
    coap_server_stop_t event_data;
    wiced_result_t result;
    wiced_coap_server_service_t* current_service, *next_service;

    linked_list_get_front_node( &server->service_list, (linked_list_node_t**) &current_service );
    while(current_service)
    {
        next_service = (wiced_coap_server_service_t*) current_service->this_node.next;
        wiced_coap_server_delete_service ( server, current_service );
        current_service = next_service;
    }
    /* delete well-known service */

    result = wiced_rtos_init_semaphore( &event_data.stop_semaphore );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (("Error in semaphore initialization\n"));
        return result;
    }

    current_event.event_type = COAP_SERVER_STOP;
    current_event.socket = &server->socket;
    event_data.security = security;
    current_event.data = &event_data;

    result = wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (("Error in push to queue\n"));
        wiced_rtos_deinit_semaphore( &event_data.stop_semaphore );
        return result;
    }

    result = wiced_rtos_get_semaphore( &event_data.stop_semaphore, WICED_NEVER_TIMEOUT );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(( "Error in get semahore \n" ));
        wiced_rtos_deinit_semaphore( &event_data.stop_semaphore );
        return result;
    }

    result = wiced_rtos_deinit_semaphore( &event_data.stop_semaphore );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(( "Error in deinitializing semaphore \n" ));
        return result;
    }

    return WICED_SUCCESS;
}

static wiced_result_t coap_internal_send_response( void* context, wiced_coap_server_service_t* service, void* req_handle, wiced_coap_server_response_t* response, coap_responsecode_t response_code, wiced_coap_notification_type notification_type, wiced_bool_t from_api )
{
    server_event_message_t current_event;
    coap_send_response_t* client_response;
    wiced_result_t result;

    result = coap_get_buffer( (void**) &client_response, sizeof(coap_send_response_t) );

    wiced_assert("unable to get memory for response : coap_internal_send_response", result == WICED_SUCCESS);

    wiced_coap_server_t* server = (wiced_coap_server_t*) context;
    coap_request_info_t* request = (coap_request_info_t*) req_handle;

    client_response->server = server;
    client_response->service = service;
    client_response->request = *request;
    client_response->response_code = response_code;

    client_response->response = *response;

    if ( from_api == WICED_TRUE )
    {
        result = coap_get_buffer( (void**) &client_response->response.payload.data, response->payload.len );

        wiced_assert("unable to get memory for payload : coap_internal_send_response", result == WICED_SUCCESS);

        memcpy( client_response->response.payload.data, response->payload.data, response->payload.len );

        client_response->notification_type = notification_type;

        current_event.event_type = COAP_SEND_EVENT;
        current_event.socket = &server->socket;
        current_event.data = (void*) client_response;

        wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );
    }
    else
    {
        client_response->notification_type = WICED_COAP_NOTIFICATION_TYPE_NONE;
        coap_send_response( client_response, WICED_FALSE );
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_server_send_response( void* context, wiced_coap_server_service_t* service, void* req_handle, wiced_coap_server_response_t* response, wiced_coap_notification_type notification_type )
{
    coap_responsecode_t response_code = COAP_RSPCODE_NONE;
    wiced_coap_option_t* option = NULL;
    coap_observer_t* observer = NULL;
    coap_request_info_t* request = (coap_request_info_t*) req_handle;

    if ( ( ( option = coap_find_options( &request->packet, COAP_OPTION_OBSERVE ) ) != NULL ) )
    {
        if ( option->buf.len == 0 )
        {

            if ( ( observer = coap_add_observer( service, request ) ) != NULL )
            {
                response_code = COAP_RSPCODE_CONTENT;
                coap_set_observer(  &response->options, ++( observer->obs_counter ) );
            }
            else
            {
                response_code = COAP_SERVICE_UNAVAILABLE;
                return WICED_SUCCESS;
            }
        }
    }

    coap_internal_send_response( context, service, req_handle, response, response_code, notification_type, WICED_TRUE );
    return WICED_SUCCESS;
}

static wiced_result_t coap_timed_event( void *arg )
{
    wiced_coap_server_t *server = (wiced_coap_server_t*) arg;
    server_event_message_t current_event;

    current_event.event_type = COAP_TIMER_EVENT;
    current_event.socket = &server->socket;
    wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );

    return WICED_SUCCESS;
}

static wiced_result_t coap_send_response( coap_send_response_t* client_response, wiced_bool_t fromapi )
{
    coap_request_info_t* client_request = (coap_request_info_t*) &client_response->request;
    wiced_coap_server_t* server = (wiced_coap_server_t*) client_response->server;
    wiced_coap_server_service_t* service = (wiced_coap_server_service_t*) client_response->service;
    wiced_coap_server_response_t* response = &client_response->response;
    coap_responsecode_t response_code;
    coap_response_info_t coap_response;
    wiced_packet_t *tx_packet;
    uint16_t available_data_length;
    uint8_t *tx_udp_data;
    wiced_result_t result = WICED_SUCCESS;
    uint16_t random;
    int payload_len;
    int data_len;

    payload_len = response->payload.len;

    memcpy(&coap_response.response.options, &client_response->response.options, sizeof(client_response->response.options));
    coap_response.response.payload.data = client_response->response.payload.data;
    coap_response.response.payload.len = payload_len;

    if ( ( result = wiced_packet_create_udp( &server->socket, UDP_MAX_DATA_LENGTH, &tx_packet, (uint8_t**) &tx_udp_data, &available_data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(( "UDP TX packet creation failed\n" ));
        goto Clean_Exit;
    }

    coap_response.response.hdr.ver = COAP_PROTOCOL_VER;

    /* If request has CON type message then send acknowledgement and if NONCON then send NONCON type
     * as response if its ping request then send RESET as a message type in response.
     */

    if ( client_request->packet.hdr.code == COAP_INTERNAL_METHOD_EMPTY )
    {
        coap_response.response.hdr.t = WICED_COAP_MSGTYPE_RESET;
        coap_response.response.hdr.id[ 0 ] = client_request->packet.hdr.id[ 0 ];
        coap_response.response.hdr.id[ 1 ] = client_request->packet.hdr.id[ 1 ];
    }
    else if ( client_request->packet.hdr.t == WICED_COAP_MSGTYPE_CON )
    {
        coap_response.response.hdr.t = WICED_COAP_MSGTYPE_ACK;
        coap_response.response.hdr.id[ 0 ] = client_request->packet.hdr.id[ 0 ];
        coap_response.response.hdr.id[ 1 ] = client_request->packet.hdr.id[ 1 ];
    }
    else
    {
        coap_response.response.hdr.t = WICED_COAP_MSGTYPE_NONCON;
        wiced_crypto_get_random( &random, sizeof( random ) );
        coap_response.response.hdr.id[ 0 ] = (uint8_t) ( random >> 8 );
        coap_response.response.hdr.id[ 1 ] = (uint8_t) random;
    }

    if ( client_response->response_code == COAP_RSPCODE_NONE )
    {
        coap_handle_methods( client_request, &response_code );
        client_response->response_code = response_code;
    }

    coap_response.response.hdr.code = client_response->response_code;
    coap_response.response.hdr.tkl = client_request->packet.hdr.tkl;
    memcpy(&coap_response.response.token, &client_request->packet.token, sizeof(client_request->packet.token));

    if ( client_request->packet.hdr.code == COAP_INTERNAL_METHOD_GET || client_request->packet.hdr.code == COAP_INTERNAL_METHOD_POST || client_request->packet.hdr.code == COAP_INTERNAL_METHOD_DELETE )
    {
        if ( client_response->response_code != COAP_NOT_FOUND )
        {
            wiced_coap_set_uri_path( &coap_response.response.options, service->service_name );
        }
    }

    if ( client_request->packet.hdr.code == COAP_INTERNAL_METHOD_GET )
    {
        if ( response->payload.len > 0 )
        {
            coap_set_content_type( &coap_response.response.options, client_response->response.payload_type );
        }
    }

    /* make sure if application gives response with payload on post we should not send payload */
    if ( client_request->packet.hdr.code == COAP_INTERNAL_METHOD_POST )
    {
        if ( response->payload.len > 0 )
        {
            response->payload.len = 0;
        }
    }

    data_len = coap_frame_create( &coap_response.response, tx_udp_data );

    if ( data_len == 0 )
    {
        wiced_packet_delete( tx_packet );
        result = WICED_ERROR;
        goto Clean_Exit;
    }

    response->payload.len = payload_len;

    wiced_packet_set_data_end( tx_packet, (uint8_t*) tx_udp_data + data_len );

    WPRINT_APP_INFO ( ("sending Response to : \"%s\" from IP %u.%u.%u.%u:%d\n", "coap",
                    (unsigned char) ( ( GET_IPV4_ADDRESS(client_request->client_ip) >> 24 ) & 0xff ),
                    (unsigned char) ( ( GET_IPV4_ADDRESS(client_request->client_ip) >> 16 ) & 0xff ),
                    (unsigned char) ( ( GET_IPV4_ADDRESS(client_request->client_ip) >> 8 ) & 0xff ),
                    (unsigned char) ( ( GET_IPV4_ADDRESS(client_request->client_ip) >> 0 ) & 0xff ),
                    client_request->port ) );


    if (server->socket.dtls_context != NULL)
    {
        if ( ( result = wiced_dtls_encrypt_packet( &server->socket.dtls_context->context, &client_request->client_ip, client_request->port, tx_packet ) ) != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR (( "DTLS encryption failed \n" ));
            wiced_packet_delete( tx_packet );
            result = WICED_ERROR;
            goto Clean_Exit;
        }
    }

    if ( ( result = wiced_udp_send( &server->socket, &client_request->client_ip, client_request->port, tx_packet ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (( "udp packet send failed\n" ));
        wiced_packet_delete( tx_packet );
        result = WICED_ERROR;
        goto Clean_Exit;
    }

    if ( client_request->packet.hdr.code != COAP_INTERNAL_METHOD_DELETE )
    {
        if ( ( client_response->notification_type != WICED_COAP_NOTIFICATION_TYPE_NONE ) )
        {
            coap_notify_observers( client_response );
        }
    }
Clean_Exit:
    if ( fromapi )
    {
        free( response->payload.data );
        response->payload.data = NULL;
    }

    free( client_response );

    return result;
}

static wiced_result_t send_udp_response( wiced_udp_socket_t *udp_socket, uint8_t *data, int data_len, wiced_ip_address_t ip, uint16_t port )
{
    wiced_packet_t *tx_packet;
    uint16_t available_data_length;
    wiced_result_t result;
    uint8_t *tx_udp_data;

    if ( ( result = wiced_packet_create_udp( udp_socket, data_len, &tx_packet, (uint8_t**) &tx_udp_data, &available_data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (( "UDP TX packet creation failed\n" ));
        return result;
    }

    memcpy( tx_udp_data, data, data_len );

    wiced_packet_set_data_end( tx_packet, (uint8_t*) tx_udp_data + data_len );

    WPRINT_APP_INFO ( ("sending to : \"%s\" from IP %u.%u.%u.%u:%d\n", "coap",
                    (unsigned char) ( ( GET_IPV4_ADDRESS(ip) >> 24 ) & 0xff ),
                    (unsigned char) ( ( GET_IPV4_ADDRESS(ip) >> 16 ) & 0xff ),
                    (unsigned char) ( ( GET_IPV4_ADDRESS(ip) >> 8 ) & 0xff ),
                    (unsigned char) ( ( GET_IPV4_ADDRESS(ip) >> 0 ) & 0xff ),
                    port ) );

    if (udp_socket->dtls_context != NULL)
    {
        if ( ( result = wiced_dtls_encrypt_packet( &udp_socket->dtls_context->context, &ip, port, tx_packet ) ) != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR (( "DTLS encryption failed \n" ));
            wiced_packet_delete( tx_packet );
            return result;
        }
    }

    if ( ( result = wiced_udp_send( udp_socket, &ip, port, tx_packet ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (( "udp packet send failed\n" ));
        wiced_packet_delete( tx_packet );
        return result;
    }

    return WICED_SUCCESS;
}

//int coap_frame_create( coap_response_info_t* pkt, uint8_t* udp_data )
//{
//    int buflen = 0;
//    size_t opts_len = 0;
//    size_t i;
//    uint8_t *p = NULL;
//    uint16_t running_delta = 0;
//
//    udp_data[ 0 ] = ( pkt->hdr.ver & COAP_HEADER_POSITION ) << COAP_HEADER_VERSION_POSITION;
//    udp_data[ 0 ] |= ( pkt->hdr.t & COAP_HEADER_POSITION ) << COAP_HEADER_TYPE_POSITION;
//    udp_data[ 0 ] |= ( pkt->hdr.tkl & COAP_HEADER_TOKEN_LEN_MASK );
//    udp_data[ 1 ] = pkt->hdr.code;
//    udp_data[ 2 ] = pkt->hdr.id[ 0 ];
//    udp_data[ 3 ] = pkt->hdr.id[ 1 ];
//
//    // Add Token
//    p = udp_data + COAP_HEADER_LENGTH;
//
//    if ( pkt->hdr.tkl > 0 )
//    {
//        memcpy( p, pkt->tok.data, pkt->hdr.tkl );
//    }
//
//    // Add Options
//    p += pkt->hdr.tkl;
//
//    if ( pkt->response->num_opts > WICED_COAP_MAX_OPTIONS )
//    {
//        WPRINT_LIB_ERROR(("Too many options\n"));
//        return buflen;
//    }
//
//    for ( i = 0; i < pkt->response->num_opts; i++ )
//    {
//        uint8_t options_delta;
//        uint8_t len;
//        uint8_t delta = 0;
//
//        options_delta = pkt->response->opts[ i ].num - running_delta;
//
//        coap_option_nibble( options_delta, &delta );
//        coap_option_nibble( pkt->response->opts[ i ].buf.len, &len );
//
//        *p++ = ( 0xFF & ( delta << 4 | len ) );
//
//        if ( delta == 13 )
//        {
//            *p++ = ( options_delta - 13 );
//        }
//        else if ( delta == 14 )
//        {
//            *p++ = ( ( options_delta - 269 ) >> 8 );
//            *p++ = ( 0xFF & ( options_delta - 269 ) );
//        }
//        if ( len == 13 )
//        {
//            *p++ = ( pkt->response->opts[ i ].buf.len - 13 );
//        }
//        else if ( len == 14 )
//        {
//            *p++ = ( pkt->response->opts[ i ].buf.len >> 8 );
//            *p++ = ( 0xFF & ( pkt->response->opts[ i ].buf.len - 269 ) );
//        }
//
//        memcpy( p, pkt->response->opts[ i ].buf.data, pkt->response->opts[ i ].buf.len );
//        p += pkt->response->opts[ i ].buf.len;
//        running_delta = pkt->response->opts[ i ].num;
//    }
//
//    opts_len = ( p - udp_data ) - COAP_HEADER_LENGTH; // number of bytes used by options
//
//    if ( pkt->response->payload.len > 0 )
//    {
//        udp_data[ COAP_HEADER_LENGTH + opts_len ] = PAYLOAD_MARKER;
//        memcpy( udp_data + COAP_HEADER_LENGTH + opts_len + PAYLOAD_MARKER_LENGTH, pkt->response->payload.data, pkt->response->payload.len );
//        buflen = opts_len + COAP_HEADER_LENGTH + PAYLOAD_MARKER_LENGTH + pkt->response->payload.len;
//    }
//    else
//    {
//        buflen = opts_len + COAP_HEADER_LENGTH;
//    }
//
//    return buflen;
//}

/* this function is used for resource discovery and send all the resources to client */

wiced_result_t handle_well_known_core( void* context, wiced_coap_server_service_t* service, wiced_coap_server_request_t* request )
{
    int count = 0;
    int first_node = 0;
    wiced_coap_server_service_t* current_node = NULL;
    wiced_coap_server_response_t response;
    char buf[ MAX_DISCOVERY_SERVICES_BUF_SIZE ];
    coap_responsecode_t response_code = COAP_RSPCODE_CONTENT;
    wiced_coap_server_t* server = (wiced_coap_server_t*) ( context );

    memset( &response, 0, sizeof(wiced_coap_server_response_t) );

    /* Traverse through all the available services server has and form that in
     * proper format so client can see all the available services with server and
     * take some action.
     */

    linked_list_get_front_node( &server->service_list, (linked_list_node_t**) &current_node );
    while ( current_node != NULL )
    {
        if ( first_node == 0 )
        {
            first_node = 1;
        }
        else
        {
            buf[ count++ ] = ',';
        }

        wiced_coap_server_service_t* service = (wiced_coap_server_service_t*) current_node;
        count = count + coap_well_known_services( service, buf + count );
        current_node = (wiced_coap_server_service_t*) service->this_node.next;
    }

    response.payload_type = COAP_INTERNAL_CONTENTTYPE_APPLICATION_LINKFORMAT;

    response.payload.len = count;
    response.payload.data = (uint8_t*) buf;

    coap_internal_send_response( context, service, request->req_handle, &response, response_code, WICED_COAP_MSGTYPE_CON, WICED_FALSE );

    return WICED_SUCCESS;
}

static coap_observer_t *coap_add_observer( wiced_coap_server_service_t* service, coap_request_info_t* request )
{
    coap_observer_t* new_observer = NULL;
    wiced_result_t result;

    /* Remove observer if its already present */
    coap_remove_observer( service, request->client_ip, request->port );

    /* Allocate memory space for new observer */
    result = coap_get_buffer( (void**) &new_observer, sizeof(coap_observer_t) );
    if ( result != WICED_SUCCESS )
    {
        return new_observer;
    }

    /* Copy parameters to observer */
    memcpy( &new_observer->addr, &request->client_ip, sizeof( request->client_ip ) );
    new_observer->port = request->port;
    new_observer->token_len = request->packet.hdr.tkl;
    memcpy( &new_observer->token, request->packet.token.data, request->packet.token.token_len );
    new_observer->obs_counter = 0;

    result = linked_list_insert_node_at_rear( &service->observer_list, &new_observer->this_node );

    return new_observer;
}

static void coap_remove_observer( wiced_coap_server_service_t* service, wiced_ip_address_t client_ip, uint16_t port )
{
    coap_observer_t* current_node;

    /*
     *  Traverse all the observers registered for this particular service
     *  and where addr and port matches delete that particular observer from
     *  list.
     */

    if ( service )
    {
        linked_list_get_front_node( &service->observer_list, (linked_list_node_t**) &current_node );

        while ( current_node != NULL )
        {
            coap_observer_t* observer = (coap_observer_t*) current_node;

            if ( ( memcmp( &observer->addr.ip.v4, &client_ip.ip.v4, sizeof( observer->addr.ip.v4 ) ) == 0 ) && ( observer->port == port ) )
            {
                linked_list_remove_node( &service->observer_list, &observer->this_node );

                /* Release observer */
                coap_release_buffer( (void*) observer );
                break;
            }

            current_node = (coap_observer_t*) observer->this_node.next;
        }
    }
}

static void coap_clear_transaction( wiced_coap_server_t* server, coap_request_info_t* request )
{
    coap_transaction_t* current_transaction = NULL;
    uint16_t id;

    /* Traverse all the transactions and delete particular transaction from list
     * which matched the message_id of request.
     */

    linked_list_get_front_node( &server->transaction_list, (linked_list_node_t**) &current_transaction );
    while ( current_transaction != NULL )
    {
        coap_transaction_t* transaction = (coap_transaction_t*) current_transaction;
        id = ( ( request->packet.hdr.id[ 0 ] << 8 ) | ( ( request->packet.hdr.id[ 1 ] ) ) );

        if ( id == transaction->mid )
        {
            /* Free the transaction. */
            coap_free_transaction( server, transaction );
            break;
        }

        current_transaction = (coap_transaction_t*) transaction->this_node.next;
    }
}

static void coap_notify_observers( coap_send_response_t* client_response )
{
    coap_observer_t *current_observer = NULL;
    wiced_coap_server_t* server = client_response->server;
    wiced_coap_server_service_t* service = client_response->service;

    /*
     * Traverse through the list of observers and find that any observers has
     * registered for this particular service if yes then notify to all that
     * registered observers.
     */

    linked_list_get_front_node( &service->observer_list, (linked_list_node_t**) &current_observer );

    while ( current_observer != NULL )
    {
        coap_observer_t* observer = (coap_observer_t*) current_observer;

        coap_notify( server, observer, service, client_response->response, client_response->notification_type, COAP_RSPCODE_CONTENT );

        current_observer = (coap_observer_t*) observer->this_node.next;
    }
}

static wiced_result_t coap_notify( wiced_coap_server_t* server, coap_observer_t* observer, wiced_coap_server_service_t* service, wiced_coap_server_response_t obs_response, wiced_coap_notification_type notification_type, coap_responsecode_t response_code )
{
    coap_transaction_t* transaction = NULL;
    coap_response_info_t notification_response;
    wiced_result_t result = WICED_SUCCESS;
    int data_length;

    transaction = coap_new_transaction( server, observer, obs_response );
    if(transaction == NULL)
    {
        WPRINT_LIB_ERROR(("Error in creating new transaction\n"));
        return WICED_ERROR;
    }

    transaction->packet = malloc( ( COAP_MAX_HEADER_SIZE + obs_response.payload.len ) * sizeof(char) );

    wiced_assert("No Heap Memory Available : coap_notify", transaction != NULL);
    if(transaction->packet == NULL)
    {
        WPRINT_LIB_ERROR(("Error No Heap Memory Available : coap_notify\n"));
        result = WICED_ERROR;
        goto Clean_Exit;
    }

    notification_response.response.payload.data = obs_response.payload.data;
    notification_response.response.payload.len = obs_response.payload.len;
    notification_response.response.hdr.ver = COAP_PROTOCOL_VER;
    notification_response.response.hdr.t = notification_type;
    transaction->service = service;
    notification_response.response.hdr.id[ 0 ] = (uint8_t) ( transaction->mid >> 8 );
    notification_response.response.hdr.id[ 1 ] = (uint8_t) transaction->mid;
    notification_response.response.hdr.code = response_code;
    notification_response.response.options.num_opts = 0;
    notification_response.response.hdr.tkl = observer->token_len;
    memcpy(&notification_response.response.token.data ,observer->token, observer->token_len);

    /*
     * If service is not deleted then only add observer count
     * to packet otherwise there is no need.
     */

    coap_set_observer( &notification_response.response.options, ++( observer->obs_counter ) );

    wiced_coap_set_uri_path( &notification_response.response.options, service->service_name );

    data_length = coap_frame_create( &notification_response.response, transaction->packet );

    if ( data_length == 0 )
    {
        result = WICED_ERROR;
        goto Clean_Exit;
    }

    transaction->packet_len = data_length;

    if ( ( result = send_udp_response( &server->socket, transaction->packet, data_length, observer->addr, observer->port ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Error in sending response to client\n"));
        result = WICED_ERROR;
        goto Clean_Exit;
    }
    return result;
Clean_Exit:
    if(transaction->packet)
        free(transaction->packet);

    coap_free_transaction(server, transaction);

    return result;

}

static coap_transaction_t* coap_new_transaction( wiced_coap_server_t* server, coap_observer_t* observer, wiced_coap_server_response_t response )
{
    uint16_t random;
    wiced_result_t result;
    coap_transaction_t* new_transaction = NULL;

    UNUSED_PARAMETER(result);

    /* Allocate memory space for new transaction */
    result = coap_get_buffer( (void**) &new_transaction, ( sizeof(coap_transaction_t) ) );

    wiced_assert("unable to pop from queue : coap_server_thread_main", result == WICED_SUCCESS);
    if(new_transaction == NULL)
    {
        WPRINT_LIB_ERROR(("Error in allocating memory for new transaction\n"));
        return NULL;
    }

    /* Create new transaction with the maximum response time and message_id and address and
     * port no of client this will be used for retransmission of unacknowledged packets.
     */

    wiced_crypto_get_random( &random, sizeof( random ) );

    /* Copy parameters to observer */
    new_transaction->mid = random;
    new_transaction->retrans_counter = 0;
    new_transaction->retrans_timer = COAP_RESPONSE_TIME;
    new_transaction->reset_timer = COAP_RESPONSE_TIME;
    memcpy( &new_transaction->addr, &observer->addr, sizeof( observer->addr ) );
    new_transaction->port = observer->port;

    result = linked_list_insert_node_at_rear( &server->transaction_list, &new_transaction->this_node );

    return new_transaction;
}

static wiced_result_t coap_get_service_name( const coap_request_info_t* request, uint8_t num, char *data )
{
    size_t i;
    wiced_bool_t first = WICED_TRUE;
    wiced_result_t result = WICED_ERROR;
    for ( i = 0; i < request->packet.options.num_opts; i++ )
    {
        if ( request->packet.options.option[ i ].num == num )
        {
            if ( first != WICED_TRUE )
            {
                strncat( data, "/", strlen( "/" ) );
            }

            if(request->packet.options.option[ i ].buf.len > WICED_COAP_MAX_SERVICE_LENGTH)
                break;

            strncat( data, (char*) request->packet.options.option[ i ].buf.data, request->packet.options.option[ i ].buf.len );
            result = WICED_SUCCESS;
            first = WICED_FALSE;
        }
    }
    return result;
}

static int coap_well_known_services( wiced_coap_server_service_t* known_services, char *buf )
{
    int i = 0;
    /* Add Resource URI */
    *buf++ = '<';
    *buf++ = '/';

    i = i + 2;

    memcpy( buf, (char*) known_services->service_name, strlen( known_services->service_name ) );
    buf = buf + strlen( known_services->service_name );
    i = i + strlen( known_services->service_name );

    *buf++ = '>';
    *buf++ = ';';

    i = i + 2;

    /* Add attribute of content type to service information */
    *buf++ = 'c';
    *buf++ = 't';
    *buf++ = '=';

    *buf++ = known_services->content_type;

    i = i + 4;

    return i;
}

static wiced_result_t coap_find_service_by_name( wiced_coap_server_t* server, uint8_t* service_name, wiced_coap_server_service_t** service_found )
{
    return linked_list_find_node( &server->service_list, compare_service, (void*) service_name, (linked_list_node_t**) ( service_found ) );
}

static wiced_bool_t compare_service( linked_list_node_t* node_to_compare, void* user_data )
{
    wiced_bool_t result;
    wiced_coap_server_service_t* current_service = (wiced_coap_server_service_t*) node_to_compare;
    char* service = (char*) user_data;

    if ( strcmp( current_service->service_name, service ) == 0 )
    {
        result = WICED_TRUE;
    }
    else
    {
        result = WICED_FALSE;
    }

    return result;
}

static int dtls_event_callback( void *socket, void *args )
{
    dtls_peer_data* peer_data;
    wiced_coap_server_t* server;

    peer_data = (dtls_peer_data*) malloc (sizeof (dtls_peer_data));
    memset (peer_data, 0, sizeof (dtls_peer_data));
    memcpy( peer_data, args, sizeof(dtls_peer_data));

    server = peer_data->callback_args;

    switch ( peer_data->event )
    {
        case DTLS_EVENT_TYPE_APP_DATA:
        {
            server_event_message_t current_event;

            current_event.event_type = COAP_RECEIVE_EVENT;
            current_event.socket = socket;
            current_event.data = peer_data;

            if ( wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT ) != WICED_SUCCESS )
            {
                WPRINT_LIB_ERROR(("Error in push to queue \n"));
                wiced_packet_delete((wiced_packet_t*)peer_data->packet);
                free (peer_data);
            }
        }
        break;
        case DTLS_EVENT_TYPE_DISCONNECTED:
        {
            free (peer_data);
            /* Disconnect Event */
            /* TODO : Remove client from observer list if client has observed for any service and still present in the list */
        }
        break;
        default:
        break;
    }

    return WICED_SUCCESS;
}
