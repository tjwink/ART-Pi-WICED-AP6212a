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

#include "wwd_constants.h"
#include "wiced_tcpip.h"
#include "wiced_dtls.h"
#include "wiced_utilities.h"
#include "dtls_host_api.h"
#include "wiced_crypto.h"
#include "wwd_buffer_interface.h"
#include "wiced_security_internal.h"
#include "crypto_constants.h"
#include "crypto_structures.h"
#include "wiced_time.h"
#include "wwd_assert.h"
#include "wwd_buffer_interface.h"
#include "besl_host_interface.h"
#include "internal/wiced_internal_api.h"
#include "wiced_udpip_dtls_api.h"

#include "wiced_network.h"
#include "dtls_types.h"

#include "mbedtls/ssl_cookie.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/cipher.h"
#include "mbedtls/ssl.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define DTLS_EVENT_QUEUE_DEPTH                      (20)
#define DTLS_THREAD_PRIORITY                        (WICED_DEFAULT_LIBRARY_PRIORITY)
#define DTLS_THREAD_STACK_SIZE                      (6200)
#define DTLS_TIMED_EVENT_TIMER_INTERVAL             (300)
#define DEFAULT_UDP_PACKET_SIZE                     (0)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

wiced_result_t  dtls_server_event_processing_thread (wiced_udp_socket_t* socket);
wiced_result_t dtls_timed_event_callback( void *arg );
wiced_result_t dtls_check_retransmission( void *arg );

/******************************************************
 *               Static Function Declarations
 ******************************************************/
int dtls_check_psk_identity( void* context, mbedtls_ssl_context* ssl, const unsigned char* PSK_identity, size_t PSK_identity_length );
static wiced_result_t dtls_receive_callback( wiced_udp_socket_t* socket, void *args );
static void dtls_event_thread( uint32_t arg );
static void dtls_remove_peer( dtls_context_t* dtls_context, dtls_peer_t* peer );
static void dtls_free_peer( dtls_peer_t* peer);
static dtls_peer_t* dtls_get_peer( dtls_context_t *context, const dtls_session_t *session );

static wiced_bool_t dtls_find_peer( linked_list_node_t* node_to_compare, void* user_data );
static wiced_bool_t dtls_find_psk_identity( linked_list_node_t* node_to_compare, void* user_data );

int dtls_network_send( void* socket, const uint8_t* buffer, size_t length );
int dtls_network_receive( void* socket, uint8_t** buffer, size_t length );
int mbedtls_timing_get_delay( void *data );
void mbedtls_timing_set_delay( void *data, uint32_t int_ms, uint32_t fin_ms );

void printMemoryInfo();
void dtls_internal_reset_context(mbedtls_ssl_context* ssl);
void dtls_internal_copy_context( mbedtls_ssl_context* new_context, mbedtls_ssl_context* old_context, dtls_session_t* peer_session_info);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static mbedtls_ssl_cookie_ctx cookie_ctx;
/* TODO : Need to make this as a local member to DTLS thread. */
static mbedtls_ssl_context ssl;
static mbedtls_ssl_config  conf;
static mbedtls_entropy_context entropy;
static mbedtls_ctr_drbg_context ctr_drbg;
static uint32_t pending_rcv_events = 0;


wiced_thread_t  dtls_thread;
wiced_queue_t   dtls_event_queue;
wiced_timed_event_t dtls_timed_event;
/******************************************************
 *               Function Definitions
 ******************************************************/

dtls_result_t dtls_host_get_packet_data( dtls_context_t* dtls, dtls_packet_t* packet, uint32_t offset, uint8_t** data, uint16_t* data_length, uint16_t* available_data_length )
{
    uint16_t temp_length;
    uint16_t temp_available_length;
    wiced_result_t result = wiced_packet_get_data( (wiced_packet_t*) packet, (uint16_t) offset, data, &temp_length, &temp_available_length );
    if ( result != WICED_SUCCESS )
    {
        return (dtls_result_t) result;
    }
    *data_length = temp_length;
    *available_data_length = temp_available_length;
    return DTLS_SUCCESS;
}

dtls_result_t dtls_host_packet_get_info( uint32_t* packet, dtls_session_t* session )
{
    wiced_udp_packet_get_info( (wiced_packet_t*) packet, (wiced_ip_address_t*) &session->ip, &session->port );
    return DTLS_SUCCESS;
}

wiced_result_t wiced_dtls_init_context( wiced_dtls_context_t* context, wiced_dtls_identity_t* identity, const char* peer_cn )
{
    memset( context, 0, sizeof(wiced_dtls_context_t) );

    /* peer_cn is not used. need to add support for passing peer_cn to DTLS library */
    UNUSED_PARAMETER( peer_cn );

    context->identity = identity;
    return WICED_SUCCESS;
}

wiced_result_t wiced_dtls_init_identity( wiced_dtls_identity_t* identity, wiced_dtls_security_type_t type, void* data )
{
    wiced_result_t result;

    memset( identity, 0, sizeof( *identity ) );

    if ( type == WICED_DTLS_SECURITY_TYPE_PSK )
    {
        wiced_dtls_psk_info_t* psk_info = (wiced_dtls_psk_info_t*) data;

        linked_list_init( &identity->psk_key.psk_identity_list );
        if ( ( result = wiced_dtls_add_psk_identity( identity, psk_info ) ) != WICED_SUCCESS )
        {
            linked_list_deinit( &identity->psk_key.psk_identity_list );
            return WICED_ERROR;
        }
    }
    else if ( type == WICED_DTLS_SECURITY_TYPE_NONPSK )
    {
        wiced_dtls_nonpsk_info_t* cert_info = (wiced_dtls_nonpsk_info_t*) data;

        wiced_assert( "Bad args", (identity != NULL) && (cert_info->private_key != NULL) && (cert_info->certificate_data != NULL) );

        mbedtls_x509_crt_init( &identity->certificate );
        result = mbedtls_x509_crt_parse( &identity->certificate, (const unsigned char *) cert_info->certificate_data, cert_info->certificate_length );
        if ( result != WICED_SUCCESS )
        {
            mbedtls_x509_crt_free( &identity->certificate );
            return WICED_ERROR;
        }

        mbedtls_pk_init( &identity->private_key );
        result = mbedtls_pk_parse_key( &identity->private_key, (const unsigned char *) cert_info->private_key, cert_info->key_length, NULL, 0 );
        if ( result != WICED_SUCCESS )
        {
            mbedtls_pk_free( &identity->private_key );
            mbedtls_x509_crt_free( &identity->certificate );
            return WICED_ERROR;
        }
    }
    else
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_dtls_add_psk_identity( wiced_dtls_identity_t* identity, wiced_dtls_psk_info_t* psk_identity )
{
    wiced_result_t result;
    result = linked_list_insert_node_at_rear( &identity->psk_key.psk_identity_list, &psk_identity->this_node );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(("Error in adding PSK identity-key pair in list\n"));
        return result;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_dtls_remove_psk_identity( wiced_dtls_identity_t* identity, wiced_dtls_psk_info_t* psk_identity )
{
    wiced_result_t result;
    wiced_dtls_psk_info_t* current_identity;

    /* Check if psk_identity is already present in list, If present then only try to remove identity from list */
    if ( linked_list_find_node( &identity->psk_key.psk_identity_list, dtls_find_psk_identity, (wiced_dtls_psk_info_t*) psk_identity, (linked_list_node_t**)&current_identity ) == WICED_SUCCESS )
    {
        result = linked_list_remove_node ( &identity->psk_key.psk_identity_list, &psk_identity->this_node );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_SECURITY_ERROR(("Error in removing PSK identity-key pair from list\n"));
            return result;
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_dtls_deinit_identity( wiced_dtls_identity_t* identity, wiced_dtls_security_type_t type )
{

    if ( type == WICED_DTLS_SECURITY_TYPE_PSK )
    {
        /* handle cleanup for PSK */
    }
    else if ( type == WICED_DTLS_SECURITY_TYPE_NONPSK )
    {
        mbedtls_x509_crt_free( &identity->certificate );
        mbedtls_pk_free( &identity->private_key );
    }
    else
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_dtls_reset_context( wiced_dtls_context_t* dtls_context )
{
    return WICED_SUCCESS;
}

wiced_result_t wiced_udp_enable_dtls( wiced_udp_socket_t* socket, void* context )
{
    socket->dtls_context = context;
    return WICED_SUCCESS;
}

wiced_result_t wiced_dtls_deinit_context( wiced_dtls_context_t* dtls_context )
{
    dtls_event_message_t current_event;

    current_event.event_type = DTLS_STOP_EVENT;
    current_event.data = NULL;

    wiced_rtos_push_to_queue( (wiced_queue_t*)dtls_context->event_queue, &current_event, WICED_NO_WAIT );

    if ( wiced_rtos_is_current_thread((wiced_thread_t*) dtls_context->event_thread ) != WICED_SUCCESS )
    {
        /* Wakeup DTLS event thread */
        wiced_rtos_thread_force_awake((wiced_thread_t*) dtls_context->event_thread );
    }

    /* Wait for the event to completely get processed. */
    wiced_rtos_thread_join((wiced_thread_t*) dtls_context->event_thread );

    /* Delete the threads */
    wiced_rtos_delete_thread((wiced_thread_t*) dtls_context->event_thread );

    memset (dtls_context, 0, sizeof (wiced_dtls_context_t));

    return WICED_SUCCESS;
}

wiced_result_t wiced_udp_start_dtls( wiced_udp_socket_t* socket, wiced_ip_address_t ip, wiced_dtls_endpoint_type_t type, wiced_dtls_certificate_verification_t verification )
{
    return wiced_generic_start_dtls_with_ciphers( socket->dtls_context, socket, ip, type, ( wiced_dtls_certificate_verification_t )verification, NULL, DTLS_UDP_TRANSPORT );
}

wiced_result_t dtls_send_pending_rcv_events( void *arg )
{
    wiced_udp_socket_t* socket = (wiced_udp_socket_t*) arg;
    dtls_event_message_t current_event;
    wiced_result_t result = WICED_SUCCESS;

    if (pending_rcv_events == 0)
    {
        /* No pending events */
        return result;
    }
    current_event.event_type = DTLS_RECEIVE_EVENT;
    current_event.data = socket;
    current_event.rcv_ev_count = pending_rcv_events;
    if ( ( result =  wiced_rtos_push_to_queue( (wiced_queue_t*)socket->dtls_context->event_queue, &current_event, WICED_NO_WAIT ) ) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_INFO(("Timed DTLS_RECEIVE_EVENT push failed. Pending Events: %u \n", (unsigned int) pending_rcv_events));
        return result;
    }

    pending_rcv_events = 0;
    return result;
}

static wiced_result_t dtls_receive_callback( wiced_udp_socket_t* socket, void *args )
{
    dtls_event_message_t current_event;
    wiced_result_t result;

    current_event.event_type = DTLS_RECEIVE_EVENT;
    current_event.data = socket;
    current_event.rcv_ev_count = ++pending_rcv_events;
    if ( ( result =  wiced_rtos_push_to_queue( (wiced_queue_t*)socket->dtls_context->event_queue, &current_event, WICED_NO_WAIT ) ) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_INFO(("DTLS_RECEIVE_EVENT push failed. Pending Events: %u \n", (unsigned int) pending_rcv_events));
        return result;
    }

    pending_rcv_events = 0;
    return WICED_SUCCESS;
}

wiced_result_t wiced_generic_start_dtls_with_ciphers( wiced_dtls_context_t* dtls_context, void* socket, wiced_ip_address_t ip, wiced_dtls_endpoint_type_t type, wiced_dtls_certificate_verification_t verification, const cipher_suite_t* cipher_list[ ], dtls_transport_protocol_t transport_protocol )
{
    wiced_result_t  result;
    int ret = 0;

    mbedtls_ssl_init( &ssl );
    memset( &dtls_context->context, 0, sizeof(wiced_dtls_workspace_t) );

    ssl.transport_protocol = TLS_UDP_TRANSPORT;
    ssl.receive_context = socket;

    /* initialize linked list for retransmission and peer list */
    linked_list_init (&dtls_context->context.peer_list);

    if ( ( result = wiced_rtos_init_queue( (wiced_queue_t*)&dtls_event_queue, NULL, sizeof(dtls_event_message_t), DTLS_EVENT_QUEUE_DEPTH ) != WICED_SUCCESS ) )
    {
        goto ERROR_QUEUE_INIT;
    }

    dtls_context->event_queue = &dtls_event_queue;

    if ( ( result = wiced_rtos_create_thread( (wiced_thread_t*)&dtls_thread, DTLS_THREAD_PRIORITY, "DTLS server", dtls_event_thread, DTLS_THREAD_STACK_SIZE, socket ) ) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(("Error in creation of DTLS thread\n"));
        goto ERROR_CREATE_THREAD;
    }

    dtls_context->event_thread = &dtls_thread;

    if ( ( result = wiced_udp_register_callbacks( (wiced_udp_socket_t*)socket, dtls_receive_callback, dtls_context->callback_arg ) ) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(("Error in registering udp callback\n"));
        goto ERROR_REGISTER_CALLBACK;
    }

    if ( ( result = wiced_rtos_register_timed_event( (wiced_timed_event_t*)&dtls_timed_event, WICED_NETWORKING_WORKER_THREAD, dtls_timed_event_callback, DTLS_TIMED_EVENT_TIMER_INTERVAL, socket ) ) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(("Error in registration of timed event \n"));
        goto ERROR_TIMED_EVENT;
    }

    dtls_context->context.timer_event = &dtls_timed_event;

    mbedtls_ssl_cookie_init( &cookie_ctx );
    mbedtls_ctr_drbg_init( &ctr_drbg );
    mbedtls_ssl_config_init( &conf );

    /* seeding entropy for random number generation */
    mbedtls_entropy_init( &entropy );
    if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy, NULL, 0) ) != 0 )
    {
        WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n", ret ));
        result = DTLS_ERROR;
        goto ERROR_CLEANUP_EXIT;
    }

    if( ( ret = mbedtls_ssl_config_defaults( &conf, MBEDTLS_SSL_IS_SERVER, MBEDTLS_SSL_TRANSPORT_DATAGRAM, MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
    {
        WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ssl_config_defaults returned %d\n\n", ret ));
        result = DTLS_ERROR;
        goto ERROR_CLEANUP_EXIT;
    }

    /* configure random number generation function */
    mbedtls_ssl_conf_rng( &conf, mbedtls_ctr_drbg_random, &ctr_drbg );

#if defined MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
    /* client authentication is disabled for now, we need to enable loading root CA chain when we add support for client authentication */
    //mbedtls_ssl_conf_ca_chain( &conf, srvcert.next, NULL );
    if( ( ret = mbedtls_ssl_conf_own_cert( &conf, &dtls_context->identity->certificate, &dtls_context->identity->private_key ) ) != 0 )
    {
        WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ssl_conf_own_cert returned %d\n\n", ret ));
        result = DTLS_ERROR;
        goto ERROR_CLEANUP_EXIT;
    }
#endif

   if( ( ret = mbedtls_ssl_cookie_setup( &cookie_ctx, mbedtls_ctr_drbg_random, &ctr_drbg ) ) != 0 )
   {
       WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ssl_cookie_setup returned %d\n\n", ret ));
       result = DTLS_ERROR;
       goto ERROR_CLEANUP_EXIT;
   }

#if defined MBEDTLS_SSL_DTLS_HELLO_VERIFY
   mbedtls_ssl_conf_dtls_cookies( &conf, mbedtls_ssl_cookie_write, mbedtls_ssl_cookie_check, &cookie_ctx );
#endif

   mbedtls_ssl_set_timer_cb( &ssl, &ssl, mbedtls_timing_set_delay, mbedtls_timing_get_delay );
   mbedtls_ssl_set_bio( &ssl, &ssl, dtls_network_send, dtls_network_receive, NULL );

#if defined MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
   mbedtls_ssl_conf_psk_cb (&conf, dtls_check_psk_identity, dtls_context->identity);
#endif

   if( ( ret = mbedtls_ssl_setup( &ssl, &conf ) ) != 0 )
   {
       WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ssl_setup returned %d\n\n", ret ));
       result = DTLS_ERROR;
       goto ERROR_CLEANUP_EXIT;
   }

   return result;

ERROR_CLEANUP_EXIT:
#if defined MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
  mbedtls_x509_crt_free( &dtls_context->identity->certificate );
  mbedtls_pk_free( &dtls_context->identity->private_key );
#endif
  mbedtls_ssl_free( &ssl );
  mbedtls_ssl_config_free( &conf );
  mbedtls_ssl_cookie_free( &cookie_ctx );
  mbedtls_ctr_drbg_free( &ctr_drbg );

ERROR_TIMED_EVENT:
    wiced_rtos_deinit_timer((wiced_timer_t*)dtls_context->context.timer_event);

ERROR_REGISTER_CALLBACK:
  wiced_udp_unregister_callbacks((wiced_udp_socket_t*)socket);

ERROR_CREATE_THREAD:
  wiced_rtos_delete_thread( (wiced_thread_t*) dtls_context->event_thread);

ERROR_QUEUE_INIT:
  wiced_rtos_deinit_queue( (wiced_queue_t*)dtls_context->event_queue );

return result;

}

static void dtls_event_thread( uint32_t arg )
{
    wiced_udp_socket_t* socket = (wiced_udp_socket_t*) arg;
    dtls_event_message_t current_event;
    wiced_result_t result;
    wiced_bool_t quit = WICED_FALSE;
    uint32_t rcv_count = 0;

    while ( quit != WICED_TRUE )
    {
        result = wiced_rtos_pop_from_queue( (wiced_queue_t*)socket->dtls_context->event_queue, &current_event, WICED_NEVER_TIMEOUT );
        wiced_assert("unable to pop from queue : dtls_server_thread_main", result == WICED_SUCCESS);

        switch ( current_event.event_type )
        {
            case DTLS_RECEIVE_EVENT:
            {
                rcv_count = current_event.rcv_ev_count;
                while ( rcv_count-- )
                {
                    result = dtls_server_event_processing_thread(socket);
                }
            }
            break;

            case DTLS_RETRANSMISSION_CHECK_EVENT:
            {
                dtls_check_retransmission( current_event.data );
            }
            break;

            case DTLS_STOP_EVENT:
            {
                dtls_peer_t *current_peer, *peer;

                wiced_udp_unregister_callbacks(socket);
                wiced_rtos_deinit_timer(socket->dtls_context->context.timer_event);

                /* Remove retransmission nodes and peers */
                linked_list_get_front_node( &socket->dtls_context->context.peer_list, (linked_list_node_t**) &peer );
                /* Remove all peers and free allocated resources related to peers */
                while ( peer != NULL )
                {
                    current_peer = peer;
                    peer = (dtls_peer_t*) peer->this_node.next;
                    dtls_free_peer( current_peer );
                    dtls_remove_peer( &socket->dtls_context->context, current_peer);
                }

                linked_list_deinit( &socket->dtls_context->context.peer_list );
                wiced_rtos_deinit_queue((wiced_queue_t*) socket->dtls_context->event_queue );
                mbedtls_ssl_free( &ssl );
                mbedtls_ssl_config_free( &conf );
                mbedtls_ssl_cookie_free( &cookie_ctx );
                mbedtls_ctr_drbg_free( &ctr_drbg );

                quit = WICED_TRUE;
            }
            break;

        default:
            wiced_assert("Wrong Event : DTLS_thread_main",1);
            break;
        }
    }

    WICED_END_OF_CURRENT_THREAD( );
}

wiced_result_t dtls_check_retransmission( void *arg )
{
#ifndef WICED_CONFIG_DISABLE_DTLS
    wiced_udp_socket_t* socket = (wiced_udp_socket_t*) arg;
    dtls_peer_t *current_peer, *next_peer;
    uint32_t current_time;
    int ret = 0;

    /* Go over all the peers and find out how many retransmission nodes has timer expired and need to retransmit */
    linked_list_get_front_node( &socket->dtls_context->context.peer_list, (linked_list_node_t**) &current_peer );
    while ( current_peer != NULL )
    {
        if ( current_peer->context.handshake->retransmit_timestamp != 0 )
        {
            next_peer = (dtls_peer_t*) current_peer->this_node.next;

            wiced_time_get_time( &current_time );

            if ( current_peer->context.handshake->retransmit_timestamp < current_time )
            {
                ret = mbedtls_ssl_resend_on_timeout ( &current_peer->context );
                if ( ret == MBEDTLS_ERR_SSL_TIMEOUT )
                {
                    dtls_free_peer(current_peer);
                    dtls_remove_peer ( &socket->dtls_context->context, current_peer );
                }
            }

            current_peer = next_peer;
        }
        else
        {
            next_peer = (dtls_peer_t*) current_peer->this_node.next;
            current_peer = next_peer;
        }
    }
#endif
    return WICED_SUCCESS;
}

wiced_result_t dtls_send_retransmission_events( void *arg )
{
    wiced_udp_socket_t* socket = (wiced_udp_socket_t*) arg;
    dtls_event_message_t current_event;
    wiced_result_t result = WICED_SUCCESS;

    current_event.event_type = DTLS_RETRANSMISSION_CHECK_EVENT;
    current_event.data = socket;

    if ( ( result = wiced_rtos_push_to_queue((wiced_queue_t*)socket->dtls_context->event_queue, &current_event, WICED_NO_WAIT ) != WICED_SUCCESS ) )
    {
        WPRINT_SECURITY_ERROR(( "Retransmission check event push to queue failed: %d \n", result ));
        return result;
    }

    return result;
}

wiced_result_t dtls_timed_event_callback( void *arg )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( ( result = dtls_send_retransmission_events(arg) ) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(( "Retransmission failed: %d \n", result ));
    }
    if ( ( result = dtls_send_pending_rcv_events(arg) ) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(( "Error :%d in sending pending events \n", result ));
    }

    return result;
}

wiced_result_t wiced_dtls_close_notify( wiced_udp_socket_t* socket )
{
    /* TODO : Notify peer with alert before closing connection */
    return WICED_SUCCESS;
}

#ifdef BESL
dtls_result_t dtls_host_free_packet( uint32_t* packet )
{
    wiced_packet_delete( (wiced_packet_t*) packet );
    return DTLS_SUCCESS;
}
#endif

dtls_result_t dtls_host_set_packet_start( tls_packet_t* packet, uint8_t* start )
{
    wiced_packet_set_data_start( (wiced_packet_t*) packet, start );
    return DTLS_SUCCESS;
}

/*
 * Calculates the maximium amount of payload that can fit in a given sized buffer
 */
wiced_result_t wiced_dtls_calculate_overhead( wiced_dtls_workspace_t* context, uint16_t available_space, uint16_t* header, uint16_t* footer )
{
    *header = 0;
    *footer = 0;

    /* Add DTLS record size */
    *header = sizeof(dtls_record_header_t);

    /* TODO : Add MAC size based on cipher suite currently only support AES CCM so
     * made it constant 8. will change once we add MAC and cipher driver. */
    *footer += MBEDTLS_CIPHERSUITE_SHORT_TAG;

    return WICED_SUCCESS;
}

wiced_result_t wiced_dtls_encrypt_packet( wiced_dtls_workspace_t* workspace, const wiced_ip_address_t* IP, uint16_t port, wiced_packet_t* packet )
{
    uint8_t* data;
    uint16_t length;
    uint16_t available;
    wiced_result_t result;
    dtls_session_t target;
    int ret;

    if ( ( workspace == NULL ) || ( packet == NULL ) )
    {
        return WICED_ERROR;
    }

    if ( wiced_packet_get_data( packet, 0, &data, &length, &available ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    data -= sizeof(dtls_record_header_t);
    result = (wiced_result_t) dtls_host_set_packet_start( (dtls_packet_t*) packet, data );

    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    memcpy( &target.ip, IP, sizeof(wiced_ip_address_t) );
    target.port = port;

    dtls_peer_t *peer = dtls_get_peer( workspace, &target );

    /* Check if peer connection already exists */
    if ( !peer )
    {
        WPRINT_SECURITY_INFO(( "peer not found : wiced_dtls_encrypt_packet\n" ));
        return WICED_ERROR;
    }
    else
    {
        if ( peer->context.state != MBEDTLS_SSL_HANDSHAKE_OVER )
        {
            return WICED_ERROR;
        }
        else
        {
           mbedtls_ssl_context* context = &peer->context;

           context->out_hdr = data;
           context->out_len = context->out_hdr + DTLS_LENGTH_OFFSET;
           context->out_msg = context->out_hdr + sizeof(dtls_record_header_t);
           context->out_iv = context->out_msg;
           context->out_msglen  = length;

           context->out_msgtype = MBEDTLS_SSL_MSG_APPLICATION_DATA;

           if( context->minor_ver >= MBEDTLS_SSL_MINOR_VERSION_2 )
           {
               memmove( context->out_msg + context->transform_out->ivlen - context->transform_out->fixed_ivlen, context->out_msg, context->out_msglen );
               context->out_msg = context->out_msg + context->transform_out->ivlen - context->transform_out->fixed_ivlen;
           }
           else
           {
               context->out_msg = context->out_iv;
           }

           if( ( ret = mbedtls_ssl_write_record( context ) ) != 0 )
           {
               WPRINT_SECURITY_ERROR( ( "Mbedtls_ssl_write_record failed \n" ) );
               return( ret );
           }

           wiced_packet_set_data_end(packet, data + context->out_left);
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t dtls_server_event_processing_thread (wiced_udp_socket_t* socket )
{
#ifndef WICED_CONFIG_DISABLE_DTLS
    wiced_packet_t*             packet = NULL;
    dtls_peer_t*                peer = NULL;
    uint8_t*                    data;
    uint8_t*                    request_string = NULL;
    uint16_t                    request_length;
    uint16_t                    available_data_length;
    dtls_session_t              peer_session_info;
    dtls_peer_data              peer_data;
    wiced_result_t              res;
    int ret;

    /* Receive DTLS packet from client on socket */
    res = wiced_udp_receive (socket, &packet, WICED_NO_WAIT);
    if ( res != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(("UDP Receive failed\n"));
        return res;
    }

    /* Extract data from packet
     * TODO: Need to handle multiple NETX packets when packet chaining is enabled */
    res = wiced_packet_get_data( (wiced_packet_t*) packet, 0, &request_string, &request_length, &available_data_length );
    if ( res != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR(("WICED_packet_get_data failed\n"));
        wiced_packet_delete( packet );
        return res;
    }

    memset ( &peer_session_info, 0, sizeof ( peer_session_info ) );

    /* Extract client IP address and port no from packet */
    wiced_udp_packet_get_info( (wiced_packet_t*) packet, (wiced_ip_address_t*) &peer_session_info.ip, &peer_session_info.port );

    WPRINT_SECURITY_DEBUG ( ("UDP Rx from IP %u.%u.%u.%u:%d\n",
                (unsigned char) ( ( GET_IPV4_ADDRESS(peer_session_info.ip) >> 24 ) & 0xff ),
                (unsigned char) ( ( GET_IPV4_ADDRESS(peer_session_info.ip) >> 16 ) & 0xff ),
                (unsigned char) ( ( GET_IPV4_ADDRESS(peer_session_info.ip) >> 8 ) & 0xff ),
                (unsigned char) ( ( GET_IPV4_ADDRESS(peer_session_info.ip) >> 0 ) & 0xff ),
                peer_session_info.port ));

    /* Check if Peer is already available in our Peer list */
    peer = dtls_get_peer(&socket->dtls_context->context, &peer_session_info);

    /* Peer is available, Process application data If handshake is over. If handshake is not over then process DTLS handshake packet */
    if ( peer )
    {
        if ( peer->context.state == MBEDTLS_SSL_HANDSHAKE_OVER )
        {
            peer->context.received_packet = (uint32_t*) packet;
            peer->context.received_packet_length = request_length;

            /* Read application data from client and call application registered callback for further processing
             * currently fragmentation and reassembly is not supported in DTLS.
             *  */
            ret = mbedtls_ssl_read ( &peer->context, &data, MBEDTLS_SSL_MAX_CONTENT_LEN);

            if ( ret == 0)
            {
                /* Decryption of application data successful, Pass it to application registered callback */
                wiced_packet_set_data_start ( (wiced_packet_t*)packet, data );
                wiced_packet_set_data_end( (wiced_packet_t*) packet, data + peer->context.in_msglen  );

                peer_data.event = DTLS_EVENT_TYPE_APP_DATA;
                peer_data.callback_args = socket->dtls_context->callback_arg;
                peer_data.packet = (dtls_packet_t*)packet;

                memcpy(&peer_data.session, &peer_session_info, sizeof(dtls_session_t));

                /* Call application registered event callback with DTLS application data, Its upto application to delete packet */
                socket->dtls_context->callback (socket, &peer_data);
            }
            else if (ret == MBEDTLS_ERR_SSL_UNEXPECTED_RECORD || ret == MBEDTLS_ERR_SSL_INVALID_RECORD )
            {
                wiced_packet_delete (( wiced_packet_t*) peer->context.received_packet);
                peer->context.received_packet = NULL;
                peer->context.received_packet_length = 0;
            }
            else if ( ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY )
            {
                /* Received Alert from peer, Free resources allocated for peer and remove from peer list */
                wiced_packet_delete ( (wiced_packet_t*) peer->context.received_packet);
                peer->context.received_packet = NULL;
                peer->context.received_packet_length = 0;
                dtls_free_peer( peer );
                dtls_remove_peer (&socket->dtls_context->context, peer);
            }
            else if ( ret == MBEDTLS_ERR_SSL_CONN_EOF || ret == MBEDTLS_ERR_SSL_WANT_READ )
            {
                /* no packet, so return and come back with packet */
                wiced_packet_delete(( wiced_packet_t*) peer->context.received_packet);
                return DTLS_SUCCESS;
            }
            else
            {
                /* discard any other errors */
                wiced_packet_delete(( wiced_packet_t*) peer->context.received_packet);
                dtls_free_peer( peer );
                dtls_remove_peer (&socket->dtls_context->context, peer);
            }
        }
        else
        {
            peer->context.received_packet = (uint32_t*) packet;
            peer->context.received_packet_length = request_length;

            /* Process handshake packets till there is no error */
            do
            {
                ret = mbedtls_ssl_handshake( &peer->context );
            } while( ret == 0 );

            /* Handshake is over, return */
            if ( peer->context.state ==  MBEDTLS_SSL_HANDSHAKE_OVER )
            {
                return DTLS_SUCCESS;
            }
            else if ( ret == MBEDTLS_ERR_SSL_CONN_EOF || ret == MBEDTLS_ERR_SSL_WANT_READ )
            {
                /* No packet available to process, receive new packet and come back */
                return DTLS_SUCCESS;
            }
            else
            {
                /* Error case, free peer resources and remove peer from list */
                if ( peer->context.received_packet != NULL )
                {
                    wiced_packet_delete( (wiced_packet_t*) peer->context.received_packet);
                    peer->context.received_packet = NULL;
                    peer->context.received_packet_length = 0;
                }

                /* remove peer and delete packet */
                dtls_free_peer( peer );
                dtls_remove_peer( &socket->dtls_context->context, peer );
                return DTLS_SUCCESS;
            }
        }
    }
    else
    {
        dtls_peer_t* new_peer = NULL;
        unsigned char client_ip[16] = { 0 };
        size_t cliip_len;
        mbedtls_ssl_context* ssl_context = NULL;

        UNUSED_PARAMETER(client_ip);
        UNUSED_PARAMETER(cliip_len);

        ssl_context = &ssl;

        /* For HelloVerifyRequest ( to generate cookie ), set IP address of current peer */
#if defined(MBEDTLS_SSL_DTLS_HELLO_VERIFY)
        memcpy(client_ip, &peer_session_info.ip.ip.v4, sizeof(peer_session_info.ip.ip.v4));
        cliip_len = sizeof(peer_session_info.ip.ip.v4);
        if( ( ret = mbedtls_ssl_set_client_transport_id( ssl_context, (unsigned char*)client_ip, cliip_len ) ) != 0 )
        {
            WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ssl_set_client_transport_id() returned -0x%x\n\n", ret ));
            wiced_packet_delete( (wiced_packet_t*) packet);
            return DTLS_ERROR;
        }
#endif

        /* set packet pointer to ssl context */
        ssl.received_packet = (uint32_t*) packet;
        ssl.received_packet_length = request_length;
        memcpy( &ssl.peer_session, &peer_session_info, sizeof(dtls_session_t) );

        /* Process handshake packets till we don't get any error, There are specific cases like after sending hello verify request ret value will be MBEDTLS_ERR_SSL_HELLO_VERIFY_REQUIRED
         * which indicates we should reset the session and start again. */

        do
        {
            ret = mbedtls_ssl_handshake( ssl_context );

            /* If client hello with cookie is processed successfully, and cookie is matching then add peer into list. change ssl_context pointer to use new peer context
             * so handshake and session related info will be stored to new peer context */
            if ( ret == 0 && ssl.handshake->cookie_verified == 1)
            {
                /* Create new peer, Allocate memory for new Peer */
                new_peer = dtls_new_peer();
                if ( new_peer == NULL )
                {
                    WPRINT_SECURITY_ERROR((" No memory available for peer \n"));
                    wiced_packet_delete( (wiced_packet_t*) ssl_context->received_packet );
                    dtls_internal_reset_context( ssl_context );
                    return DTLS_ERROR_OUT_OF_MEMORY;
                }

                /* Initialize new peer mbedtls_ssl_context */
                mbedtls_ssl_init(&new_peer->context);

                /* Setup & configure new peer context */
                if( ( ret = mbedtls_ssl_setup( &new_peer->context, &conf ) ) != 0 )
                {
                    WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ssl_setup returned %d\n\n", ret ));
                    wiced_packet_delete( (wiced_packet_t*) ssl_context->received_packet );
                    dtls_internal_reset_context( ssl_context );
                    free(new_peer);
                    return DTLS_ERROR;
                }

                /* Copy required information from old ssl context to new peer ssl context */
                dtls_internal_copy_context ( &new_peer->context, ssl_context, &peer_session_info );
                memcpy( &new_peer->session, &peer_session_info, sizeof(dtls_session_t) );

                /* Register timer function for new peer */
                mbedtls_ssl_set_timer_cb( &new_peer->context, &new_peer->context, mbedtls_timing_set_delay, mbedtls_timing_get_delay );

                /* Register callback functions to send & receive packet */
                mbedtls_ssl_set_bio( &new_peer->context, &new_peer->context, dtls_network_send, dtls_network_receive, NULL );

#if defined MBEDTLS_SSL_DTLS_HELLO_VERIFY
                memcpy(client_ip, &peer_session_info.ip.ip.v4, sizeof(peer_session_info.ip.ip.v4));
                cliip_len = sizeof(peer_session_info.ip.ip.v4);

                /* For hello verify request ( to generate cookies ), set client IP address */
                if( ( ret = mbedtls_ssl_set_client_transport_id( &new_peer->context, (unsigned char*)client_ip, cliip_len ) ) != 0 )
                {
                    WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ssl_set_client_transport_id() returned -0x%x\n\n", ret ));
                    wiced_packet_delete( (wiced_packet_t*) ssl_context->received_packet);
                    dtls_internal_reset_context( ssl_context );
                    dtls_free_peer( new_peer );
                    free( new_peer );
                    return DTLS_ERROR;
                }
#endif

                /* Add new peer into existing peer list */
                if ( linked_list_insert_node_at_rear( &socket->dtls_context->context.peer_list, &new_peer->this_node ) != WICED_SUCCESS )
                {
                    WPRINT_SECURITY_ERROR(("Error in adding new peer to list \n"));
                    wiced_packet_delete( (wiced_packet_t*) ssl_context->received_packet);
                    dtls_internal_reset_context( ssl_context );
                    dtls_free_peer( new_peer );
                    free (new_peer);
                    return DTLS_ERROR;
                }

                ssl_context = &new_peer->context;
                ssl.handshake->cookie_verified = 0;
          }

      } while( ret == 0 );

      /* Hello verify request sent successfully, now reset the ssl context  */
      if( ret == MBEDTLS_ERR_SSL_HELLO_VERIFY_REQUIRED )
      {
          /* Packet will be deleted before returning errror in mbedtls. But this is added for safe side */
          if ( ssl.received_packet != NULL )
          {
              wiced_packet_delete((wiced_packet_t*) ssl.received_packet);
          }
          mbedtls_ssl_session_reset(&ssl);
          return DTLS_SUCCESS;
      }
      else if ( ret == MBEDTLS_ERR_SSL_CONN_EOF || ret == MBEDTLS_ERR_SSL_WANT_READ )
      {
          dtls_internal_reset_context(&ssl);
          /* If return value is MBEDTLS_ERR_SSL_CONN_EOF or MBEDTLS_ERR_SSL_WANT_READ then, it is success case as it came out from loop because there was no packet to process, receive packet and come back */
          return DTLS_SUCCESS;
      }
      else
      {
          dtls_internal_reset_context(&ssl);
          wiced_packet_delete( (wiced_packet_t*) packet);
          if ( new_peer )
          {
              dtls_free_peer( new_peer );
              dtls_remove_peer( &socket->dtls_context->context, new_peer );
          }
      }
  }
#endif
  return DTLS_SUCCESS;
}

static dtls_peer_t* dtls_get_peer( dtls_context_t *context, const dtls_session_t *session )
{
    dtls_peer_t* current_peer = NULL;

    /* Check if Peer is present in our Peer list by comparing IP address and Port. Return peer if available else return NULL */
    linked_list_find_node( &context->peer_list, dtls_find_peer, (dtls_session_t*) session, (linked_list_node_t**)&current_peer );

    return current_peer;
}

int dtls_network_send( void* ctx, const uint8_t* buffer, size_t length )
{
    mbedtls_ssl_context* ssl = (mbedtls_ssl_context*) ctx;
    wiced_udp_socket_t* socket = (wiced_udp_socket_t*) ssl->receive_context;

    wiced_packet_set_data_end( (wiced_packet_t*) ssl->outgoing_packet, ((uint8_t*)buffer) + length );

    /* Send the UDP packet */
    if ( wiced_udp_send( socket, (wiced_ip_address_t*) &ssl->peer_session.ip, ssl->peer_session.port, (wiced_packet_t*) ssl->outgoing_packet) != WICED_SUCCESS )
    {
        wiced_packet_delete( (wiced_packet_t*) ssl->outgoing_packet ); /* Delete packet, since the send failed */
        return DTLS_ERROR;
    }

    ssl->outgoing_packet = NULL;
    ssl->out_buf = NULL;
    ssl->out_msg = NULL;
    ssl->out_hdr = NULL;
    ssl->out_len = 0;

    return length;
}

int dtls_network_receive( void *ctx, unsigned char **buf, size_t len )
{
    uint16_t                    available_data_length;
    mbedtls_ssl_context* ssl = (mbedtls_ssl_context*) ctx;

    /* If previously received packet is consumed, then go and read the next packet */
      if ( ssl->received_packet == NULL )
      {
          ssl->received_packet_length = 0;
      }
      else
      {
          ssl->received_packet_bytes_skipped = 0;
          wiced_packet_get_data( (wiced_packet_t*) ssl->received_packet, 0, (uint8_t**)buf, &ssl->received_packet_length, &available_data_length );

          return ssl->received_packet_length;
      }

    return DTLS_SUCCESS;
}

void mbedtls_timing_set_delay( void *data, uint32_t int_ms, uint32_t fin_ms )
{
#ifndef WICED_CONFIG_DISABLE_DTLS
    uint32_t current_time;
    mbedtls_ssl_context* context = (mbedtls_ssl_context*) data;

    if ( context->handshake != NULL )
    {
        if ( fin_ms == 0 )
        {
            context->handshake->retransmit_timestamp = 0;
        }

        if ( fin_ms > 0 )
        {
            wiced_time_get_time (&current_time);
            context->handshake->retransmit_timestamp = current_time + fin_ms;
        }
    }
#endif
}

/*
 * Get number of delays expired
 */
int mbedtls_timing_get_delay( void *data )
{
#ifndef WICED_CONFIG_DISABLE_DTLS
    mbedtls_ssl_context* context = (mbedtls_ssl_context*) data;
    uint32_t current_time;

    if ( context->handshake != NULL )
    {
        if ( context->handshake->retransmit_timestamp > 0 )
        {
            wiced_time_get_time( &current_time );
            if ( context->handshake->retransmit_timestamp < current_time )
            {
                return 2;
            }
        }
    }
#endif
    return 0;
}

void dtls_free_peer( dtls_peer_t* peer)
{
    mbedtls_ssl_free( &peer->context );
}

/* Search for peer if it is already in list then remove the peer from list */
void dtls_remove_peer( dtls_context_t* dtls_context, dtls_peer_t* peer )
{
    if ( linked_list_find_node( &dtls_context->peer_list, dtls_find_peer, &peer->session, (linked_list_node_t**)&peer ) == WICED_SUCCESS )
    {
        linked_list_remove_node( &dtls_context->peer_list, &peer->this_node );
    }

    free(peer);
}

static wiced_bool_t dtls_find_peer( linked_list_node_t* node_to_compare, void* user_data )
{
    dtls_peer_t* current_peer = (dtls_peer_t*) node_to_compare;
    dtls_session_t* peer_session = (dtls_session_t*) user_data;

    if ( current_peer->session.ip.ip.v4 == peer_session->ip.ip.v4 && current_peer->session.port == peer_session->port )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static wiced_bool_t dtls_find_psk_identity( linked_list_node_t* node_to_compare, void* user_data )
{
    wiced_dtls_psk_info_t* current_identity = (wiced_dtls_psk_info_t*) node_to_compare;
    wiced_dtls_psk_info_t* identity = (wiced_dtls_psk_info_t*) user_data;

    if ( current_identity == identity )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

#ifdef BESL
void printMemoryInfo()
{
   struct mallinfo mi;
   memset(&mi,0,sizeof(struct mallinfo));
   mi = mallinfo();
   printf(" Heap Blocks = %d\n",mi.uordblks);
   printf("Total free space (fordblks):           %d\n", mi.fordblks);
}
#endif

/* Search for matching PSK identity entry in psk_identity list by comparing with PSK_identity sent from client. If it matches then set secret key and secret key length
 * else return error if no entry found  */
int dtls_check_psk_identity( void* ctx, mbedtls_ssl_context* ssl, const unsigned char* PSK_identity, size_t PSK_identity_length )
{
    wiced_dtls_identity_t* identity = ( wiced_dtls_identity_t*) ctx;
    wiced_tls_psk_info_t* psk_info;

    UNUSED_PARAMETER(identity);
    UNUSED_PARAMETER(psk_info);

#if defined MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED
    linked_list_get_front_node( &identity->psk_key.psk_identity_list, (linked_list_node_t**) &psk_info );
    while ( psk_info != NULL )
    {
        if( PSK_identity_length == strlen( psk_info->identity ) && memcmp( PSK_identity, psk_info->identity, PSK_identity_length ) == 0 )
        {
            return( mbedtls_ssl_set_hs_psk( ssl, (const unsigned char*) psk_info->key, psk_info->key_length ) );
        }

        psk_info = (wiced_tls_psk_info_t*) psk_info->this_node.next;
    }
#endif
    /* no matching PSK entry found */
    return -1;
}
