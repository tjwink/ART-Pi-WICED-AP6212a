/*
 * WICED EAP host implementation
 *
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
 *
 */

#include <string.h>
#include "wiced_result.h"
#include "rtos.h"
#include "wwd_rtos_interface.h"
#include "besl_host_interface.h"
#include "besl_host_rtos_structures.h"
#include "network/wwd_buffer_interface.h"
#include "supplicant_structures.h"
#include "wiced_supplicant.h"

#include "wiced_tcpip_tls_api.h"
#include "wiced_utilities.h"
#include "wiced_tls.h"
#include "wwd_eapol.h"
#include "wwd_events.h"
#include "wwd_wifi.h"
#include "wiced_time.h"
#include "wiced_tcpip_tls_api.h"


/******************************************************
 *            Includes
 ******************************************************/

/******************************************************
 *             Constants
 ******************************************************/

#define SUPPLICANT_THREAD_STACK_SIZE         ( 4*1024 )
#define TLS_AGENT_THREAD_STACK_SIZE          ( 4*1024 )
#define SUPPLICANT_BUFFER_SIZE               ( 3500 )
#define SUPPLICANT_WORKSPACE_ARRAY_SIZE      ( 1 )
#define WLC_EVENT_MSG_LINK                   ( 0x01 )
#define EAPOL_PACKET_TIMEOUT                 ( 5000 )  /* Milliseconds */
#define SUPPLICANT_HANDSHAKE_ATTEMPT_TIMEOUT ( 30000 ) /* Milliseconds */
#define EAP_HANDSHAKE_TIMEOUT_IN_MSEC        ( 15000 ) /* Milliseconds */

/******************************************************
 *             Macros
 ******************************************************/

#define IF_TO_WORKSPACE( interface )   ( active_supplicant_workspaces[ WWD_STA_INTERFACE ] )     /* STA is the only currently supported interface. */

/******************************************************
 *             Local Structures
 ******************************************************/

typedef struct
{
    besl_host_workspace_t host_workspace;
    host_queue_type_t     outgoing_packet_queue;
    besl_event_message_t  outgoing_packet_buffer[10];
} supplicant_host_workspace_t;

struct supplicant_phase2_workspace_s
{
    supplicant_phase2_state_t state;
    host_thread_type_t      thread;
    void*                   thread_stack;
};
typedef struct supplicant_phase2_workspace_s supplicant_phase2_workspace_t;

/******************************************************
 *             Static Variables
 ******************************************************/

static const wwd_event_num_t         supplicant_events[] = { WLC_E_LINK, WLC_E_DEAUTH_IND, WLC_E_DISASSOC_IND, WLC_E_NONE };
static       supplicant_workspace_t* active_supplicant_workspaces[SUPPLICANT_WORKSPACE_ARRAY_SIZE] = { 0 };

/******************************************************
 *             Static Function Prototypes
 ******************************************************/

static void          supplicant_tls_agent_thread      ( wwd_thread_arg_t arg );
static void          wiced_supplicant_thread          ( wwd_thread_arg_t arg );
static void          supplicant_eapol_packet_handler  ( wiced_buffer_t buffer, wwd_interface_t interface );
static void*         supplicant_external_event_handler( const wwd_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data );
static void          wiced_supplicant_thread_main     ( wwd_thread_arg_t arg );
#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
static void          supplicant_phase2_thread           ( wwd_thread_arg_t arg );
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */

#ifdef WPRINT_ENABLE_WWD_DEBUG
const char*     wwd_event_to_string         (wwd_event_num_t var);
char*           wwd_ssid_to_string          (uint8_t *value, uint8_t length, char *ssid_buf, uint8_t ssid_buf_len);
const char*     wwd_status_to_string        (wwd_event_status_t status);
const char*     wwd_reason_to_string        (wwd_event_reason_t reason);
const char*     wwd_interface_to_string      (wwd_interface_t interface);
#endif /* ifdef WPRINT_ENABLE_WWD_DEBUG */

/******************************************************
 *             Function definitions
 ******************************************************/

besl_result_t supplicant_tls_agent_start( supplicant_workspace_t* workspace )
{
    besl_host_workspace_t* host_workspace = (besl_host_workspace_t*) workspace->tls_agent.tls_agent_host_workspace;

    workspace->have_packet = 0;
    if ( host_rtos_create_thread_with_arg( &host_workspace->thread, supplicant_tls_agent_thread, "tls_agent", host_workspace->thread_stack, TLS_AGENT_THREAD_STACK_SIZE, RTOS_HIGHER_PRIORTIY_THAN(RTOS_DEFAULT_THREAD_PRIORITY), (wwd_thread_arg_t) workspace ) == WWD_SUCCESS )
    {
        return SUPPLICANT_SUCCESS;
    }
    return SUPPLICANT_NOT_STARTED;
}

static void supplicant_tls_agent_thread( wwd_thread_arg_t arg )
{
    wiced_result_t res;
    supplicant_workspace_t* workspace = (supplicant_workspace_t*)arg;
    besl_host_workspace_t*  host      = (besl_host_workspace_t*)workspace->tls_agent.tls_agent_host_workspace;
    besl_event_message_t    message;

    SUPPLICANT_DEBUG( ( "\nTLS handshake start..\n" ) );
    if ( (res = wiced_supplicant_start_tls( workspace, WICED_TLS_AS_CLIENT, TLS_VERIFICATION_REQUIRED )) != WICED_SUCCESS )
    {
        SUPPLICANT_DEBUG( ( "TLS handshake failed with error = [%d]\n",  res) );
    }
    else
    {
        SUPPLICANT_DEBUG( ( "TLS Agent finish connect start..\n" ) );
        supplicant_tls_agent_finish_connect( workspace );
        SUPPLICANT_DEBUG( ( "TLS Agent finish connect completed\n" ) );
    }

    /* Clean up left over messages in the event queue */
    SUPPLICANT_DEBUG( ( "Cleanup TLS Agent event queue\n" ) );
    while ( host_rtos_pop_from_queue( &host->event_queue, &message, 0 ) == WWD_SUCCESS )
    {
        if (message.event_type == BESL_EVENT_EAPOL_PACKET_RECEIVED || message.event_type == SUPPLICANT_EVENT_PACKET_TO_SEND )
        {
            besl_host_free_packet(message.data.packet);
        }
    }
    SUPPLICANT_DEBUG( ( "Cleanup of TLS Agent event queue done!\n" ) );

    WICED_END_OF_CURRENT_THREAD( );
}

besl_result_t supplicant_tls_calculate_overhead( supplicant_workspace_t* workspace, uint16_t available_space, uint16_t* header, uint16_t* footer )
{
    return (besl_result_t)wiced_tls_calculate_overhead( &workspace->tls_context->context, available_space, header, footer );
}

besl_result_t supplicant_send_eap_tls_packet( supplicant_workspace_t* workspace, tls_agent_event_message_t* tls_agent_message, uint32_t timeout )
{
    besl_host_workspace_t* tls_agent_host_workspace = (besl_host_workspace_t*) workspace->tls_agent.tls_agent_host_workspace;
    wwd_result_t result = host_rtos_push_to_queue(  &tls_agent_host_workspace->event_queue, tls_agent_message, timeout);
    if ( result != WWD_SUCCESS )
    {
        SUPPLICANT_DEBUG( ( "Supplicant unable to push packet to tls agent queue\n" ) );
        besl_host_free_packet( tls_agent_message->data.packet );
    }
    return (besl_result_t) result;
}


besl_result_t supplicant_receive_eap_tls_packet( void* workspace_in, tls_packet_t** packet, uint32_t timeout )
{
    supplicant_workspace_t*     workspace = (supplicant_workspace_t*) workspace_in;
    tls_agent_event_message_t   message;
    besl_host_workspace_t* tls_agent_host_workspace = (besl_host_workspace_t*)workspace->tls_agent.tls_agent_host_workspace;
    wwd_result_t result = host_rtos_pop_from_queue(&tls_agent_host_workspace->event_queue, &message, timeout );
    if ( result != WWD_SUCCESS )
    {
        return (besl_result_t) result;
    }
    if ( message.event_type == TLS_AGENT_EVENT_EAPOL_PACKET )
    {
        *packet = message.data.packet;
    }
    else
    {
        host_rtos_delay_milliseconds( 10 );
    }
    return BESL_SUCCESS;
}


besl_result_t supplicant_tls_agent_init( tls_agent_workspace_t* workspace )
{
    besl_host_workspace_t* host_workspace;

    SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : MALLOC : Allocateing TLS Agent host workspace\n", __FUNCTION__, __LINE__ ));
    host_workspace = besl_host_malloc("tls_agent workspace", sizeof(besl_host_workspace_t));
    if (host_workspace == NULL)
    {
        SUPPLICANT_DEBUG((" ERROR : [%s()] : L%d : Failed to allocate workspace for TLS Agent\n", __FUNCTION__, __LINE__ ));
        return SUPPLICANT_OUT_OF_HEAP_SPACE;
    }
    memset(host_workspace, 0, sizeof(besl_host_workspace_t));
    workspace->tls_agent_host_workspace = host_workspace;

#ifdef RTOS_USE_STATIC_THREAD_STACK
    SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : MALLOC : Allocateing TLS Agent thread stack\n", __FUNCTION__, __LINE__ ));
    host_workspace->thread_stack = besl_host_malloc("tls agent stack", TLS_AGENT_THREAD_STACK_SIZE);
    if (host_workspace->thread_stack == NULL)
    {
        besl_host_free(host_workspace);
        host_workspace = NULL;
        return SUPPLICANT_OUT_OF_HEAP_SPACE;
    }
    memset( host_workspace->thread_stack, 0, TLS_AGENT_THREAD_STACK_SIZE );
#else
    host_workspace->thread_stack = NULL;
#endif

    SUPPLICANT_DEBUG((" [%s()] : L%d : Init event queue for TLS Agent\n", __FUNCTION__, __LINE__ ));
    host_rtos_init_queue( &host_workspace->event_queue, host_workspace->event_buffer, sizeof( host_workspace->event_buffer ), sizeof(tls_agent_event_message_t) );
    SUPPLICANT_DEBUG((" [%s()] : L%d : TLS Agent init done!\n", __FUNCTION__, __LINE__ ));

    return SUPPLICANT_SUCCESS;
}


besl_result_t supplicant_tls_agent_deinit( tls_agent_workspace_t* workspace )
{
    /* FIXME: Split me into stop and deinit APIs */
    besl_host_workspace_t* host_workspace = workspace->tls_agent_host_workspace;


    if ( host_workspace != NULL )
    {
        SUPPLICANT_DEBUG(("[%s()] : L%d : Waiting for TLS Agent thread to join\n", __FUNCTION__, __LINE__ ));
        host_rtos_join_thread(&host_workspace->thread);
        SUPPLICANT_DEBUG(("[%s()] : L%d : TLS Agent thread joined\n", __FUNCTION__, __LINE__ ));

        /* Delete the tls agent thread */
        host_rtos_delete_terminated_thread( &host_workspace->thread );
        SUPPLICANT_DEBUG(("[%s()] : L%d : Deleted TLS Agent thread\n", __FUNCTION__, __LINE__ ));

        SUPPLICANT_DEBUG(("[%s()] : L%d : Deinit TLS Agent thread queue\n", __FUNCTION__, __LINE__ ));
        host_rtos_deinit_queue( &host_workspace->event_queue );

        if ( host_workspace->thread_stack != NULL )
        {
            SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : FREE : Freeing TLS Agent thread stack\n", __FUNCTION__, __LINE__ ));
            besl_host_free( host_workspace->thread_stack );
            host_workspace->thread_stack = NULL;
        }

        SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : FREE : Freeing TLS Agent host workspace\n", __FUNCTION__, __LINE__ ));
        besl_host_free( workspace->tls_agent_host_workspace );
        workspace->tls_agent_host_workspace = NULL;
    }
    return BESL_SUCCESS;
}


besl_result_t besl_supplicant_start( supplicant_workspace_t* workspace )
{
    besl_result_t result = SUPPLICANT_ERROR_AT_THREAD_START;
    besl_host_workspace_t* host_workspace = &((supplicant_host_workspace_t*) workspace->supplicant_host_workspace )->host_workspace;

    result = (besl_result_t) host_rtos_create_thread_with_arg( &host_workspace->thread, wiced_supplicant_thread, "supplicant", host_workspace->thread_stack, SUPPLICANT_THREAD_STACK_SIZE, RTOS_HIGHER_PRIORTIY_THAN(RTOS_DEFAULT_THREAD_PRIORITY), (wwd_thread_arg_t) workspace );

    return result;
}


static void wiced_supplicant_thread( wwd_thread_arg_t arg )
{
    wiced_supplicant_thread_main( arg );

    WICED_END_OF_CURRENT_THREAD( );
}

besl_result_t supplicant_init_state(supplicant_workspace_t* workspace, eap_type_t eap_type )
{
    workspace->eap_type                 = eap_type;
    workspace->current_sub_stage        = SUPPLICANT_EAP_START;

    workspace->eap_handshake_start_time = 0;

    return BESL_SUCCESS;
}

besl_result_t besl_supplicant_init(supplicant_workspace_t* workspace, supplicant_connection_info_t *conn_info)
{
    supplicant_host_workspace_t* supplicant_host_workspace = NULL;
    besl_result_t result;

    /* validating ttls input parameters */
    if(conn_info->eap_type == EAP_TYPE_TTLS)
    {
        if(conn_info->tunnel_auth_type != TUNNEL_TYPE_EAP)
        {
            return BESL_ERROR;
        }
        if(!( conn_info->inner_eap_type == EAP_TYPE_LEAP || conn_info->inner_eap_type == EAP_TYPE_MSCHAPV2))
        {
            return BESL_ERROR;
        }
    }

    if ( conn_info->eap_type == EAP_TYPE_PEAP /*|| ( conn_info->eap_type == EAP_TYPE_TTLS && !(conn_info->is_client_cert_required) )*/)
    {
        wiced_tls_init_identity( conn_info->tls_identity, NULL, 0, NULL, 0 );
    }
    else if ( conn_info->eap_type == EAP_TYPE_TLS || ( conn_info->eap_type == EAP_TYPE_TTLS && conn_info->is_client_cert_required))
    {
        wiced_tls_init_identity( conn_info->tls_identity, conn_info->private_key, conn_info->key_length, conn_info->user_cert, conn_info->user_cert_length );
    }

    wiced_tls_init_context( conn_info->context, conn_info->tls_identity, NULL );

    conn_info->context->session = conn_info->tls_session;

    wiced_tls_init_root_ca_certificates( conn_info->trusted_ca_certificates, conn_info->root_ca_cert_length );

    memset( workspace, 0, sizeof( *workspace ) );

    /* Register the EAPOL packet receive handler */
    result = (besl_result_t) wwd_eapol_register_receive_handler( supplicant_eapol_packet_handler );
    if ( result != BESL_SUCCESS )
    {
        SUPPLICANT_DEBUG( ("ERROR : EAPOL hndlr registration failed with error = [%d]\n", result ) );
        return result;
    }

    /* Allocate memory for the supplicant host workspace */
    SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : MALLOC : Allocating supplicant host workspace\n", __FUNCTION__, __LINE__ ));
    supplicant_host_workspace = besl_host_calloc( "supplicant host", 1, sizeof(supplicant_host_workspace_t) );
    if ( supplicant_host_workspace == NULL )
    {
        result = SUPPLICANT_OUT_OF_HEAP_SPACE;
        SUPPLICANT_DEBUG( ("ERROR : Couldn't allocate supplicant host workspace\n") );
        goto exit;
    }
    workspace->supplicant_host_workspace = supplicant_host_workspace;

    /* Allocate memory for the EAP defragmentation buffer */
    SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : MALLOC : Allocating supplicant defragmentation buffer\n", __FUNCTION__, __LINE__ ));
    workspace->buffer = besl_host_calloc( "supplicant buffer", 1, SUPPLICANT_BUFFER_SIZE );
    if ( workspace->buffer == NULL )
    {
        result = SUPPLICANT_OUT_OF_HEAP_SPACE;
        SUPPLICANT_DEBUG( ("ERROR : Couldn't allocate EAPOL packet defragmentation buffer\n") );
        goto exit;
    }
    workspace->buffer_size = SUPPLICANT_BUFFER_SIZE;
    workspace->auth_type = conn_info->auth_type;

    /* Allocate memory for the supplicant host thread stack */
#ifdef RTOS_USE_STATIC_THREAD_STACK
    SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : MALLOC : Allocating supplicant thread stack\n", __FUNCTION__, __LINE__ ));
    supplicant_host_workspace->host_workspace.thread_stack = besl_host_calloc("supplicant thread stack", 1, SUPPLICANT_THREAD_STACK_SIZE);
    if (supplicant_host_workspace->host_workspace.thread_stack == NULL)
    {
        result = SUPPLICANT_ERROR_STACK_MALLOC_FAIL;
        goto exit;
    }
#else
    supplicant_host_workspace->host_workspace.thread_stack = NULL;
#endif

    supplicant_host_workspace->host_workspace.interface = conn_info->interface;
    workspace->interface = conn_info->interface;
    wwd_wifi_get_mac_address( (wiced_mac_t*) &workspace->supplicant_mac_address, (wwd_interface_t) workspace->interface );

    supplicant_init_state( workspace, conn_info->eap_type );
    workspace->current_main_stage = SUPPLICANT_INITIALISING;
    workspace->supplicant_result = SUPPLICANT_NOT_STARTED;

    if(workspace->eap_type == EAP_TYPE_TTLS)
    {
        workspace->inner_eap_type = conn_info->inner_eap_type;
        workspace->tunnel_auth_type = conn_info->tunnel_auth_type;
    }

    host_rtos_init_queue( &supplicant_host_workspace->host_workspace.event_queue, supplicant_host_workspace->host_workspace.event_buffer, sizeof( supplicant_host_workspace->host_workspace.event_buffer ), sizeof(besl_event_message_t) );
    host_rtos_init_queue( &supplicant_host_workspace->outgoing_packet_queue, supplicant_host_workspace->outgoing_packet_buffer, sizeof( supplicant_host_workspace->outgoing_packet_buffer ), sizeof(besl_event_message_t) );

    wiced_supplicant_enable_tls( workspace, conn_info->context );
    besl_supplicant_set_identity( workspace, conn_info->eap_identity, strlen( conn_info->eap_identity ) );

    if ( conn_info->eap_type == EAP_TYPE_PEAP  || conn_info->eap_type == EAP_TYPE_TTLS)
    {
        /* Preserve user name and password for PEAP and EAP-TTLS*/
        /* Convert the password from ASCII to UTF16 */
        int i;
        uint8_t* password = (uint8_t*) conn_info->password;
        uint8_t* unicode = (uint8_t*) workspace->inner_identity.password;

        SUPPLICANT_DEBUG( ( "Initialize inner identity and password\n" ) );

        for ( i = 0; i <= strlen( conn_info->password ); i++ )
        {
            *unicode++ = *password++ ;
            *unicode++ = '\0';
        }
        workspace->inner_identity.password_length = 2 * ( i - 1 );

        workspace->inner_identity.identity_length = MIN(sizeof(workspace->inner_identity.identity), strlen(conn_info->user_name));
        memcpy( workspace->inner_identity.identity, conn_info->user_name, workspace->inner_identity.identity_length );
    }

    return BESL_SUCCESS;

exit:
    wwd_eapol_unregister_receive_handler( );
    if ( workspace->buffer != NULL )
    {
        besl_host_free(workspace->buffer);
        workspace->buffer = NULL;
    }
    if ( workspace->supplicant_host_workspace != NULL ) /* This points at supplicant_host_workspace */
    {
        if ( supplicant_host_workspace->host_workspace.thread_stack != NULL )
        {
            besl_host_free( supplicant_host_workspace->host_workspace.thread_stack );
            supplicant_host_workspace->host_workspace.thread_stack = NULL;
        }

        SUPPLICANT_DEBUG( ("Deinit and free host queues\n" ) );
        host_rtos_deinit_queue( &supplicant_host_workspace->host_workspace.event_queue);
        host_rtos_deinit_queue( &supplicant_host_workspace->outgoing_packet_queue );
        besl_host_free( workspace->supplicant_host_workspace );
        workspace->supplicant_host_workspace = NULL;
    }

    SUPPLICANT_DEBUG( ("Deinitialize TLS context, cert and identity\n" ) );
    wiced_tls_deinit_root_ca_certificates();
    wiced_tls_deinit_identity( workspace->tls_context->identity );
    wiced_tls_deinit_context( workspace->tls_context );

    return result;
}


besl_result_t besl_supplicant_deinit( supplicant_workspace_t* workspace )
{
    supplicant_host_workspace_t* supplicant_host_workspace = (supplicant_host_workspace_t*) workspace->supplicant_host_workspace;
    besl_event_message_t message;

    /* Check if supplicant is stopped before de-initializing it */
    if ( workspace->supplicant_result != SUPPLICANT_ABORTED )
    {
        SUPPLICANT_DEBUG( ("Either supplicant is still not stopped or supplicant is not started\n" ) );
        return BESL_ERROR;
    }

    SUPPLICANT_DEBUG( ( "Unregister EAPOL handler\n" ) );
    wwd_eapol_unregister_receive_handler( );

    /** Queues clean up **/
    /* Clean up left over messages in the event and outgoing packet queues */SUPPLICANT_DEBUG( ( "Cleanup supplicant event queue\n" ) );
    while ( host_rtos_pop_from_queue( &supplicant_host_workspace->host_workspace.event_queue, &message, 0 ) == WWD_SUCCESS )
    {
        if ( message.event_type == BESL_EVENT_EAPOL_PACKET_RECEIVED || message.event_type == SUPPLICANT_EVENT_PACKET_TO_SEND )
        {
            besl_host_free_packet( message.data.packet );
        }
    }
    SUPPLICANT_DEBUG( ( "Cleanup supplicant outgoing packet queue\n" ) );
    while ( host_rtos_pop_from_queue( &( (supplicant_host_workspace_t*) workspace->supplicant_host_workspace )->outgoing_packet_queue, &message, 0 ) == WWD_SUCCESS )
    {
        if ( message.event_type == SUPPLICANT_EVENT_PACKET_TO_SEND )
        {
            besl_host_free_packet( message.data.packet );
        }
    }

    SUPPLICANT_DEBUG( ("Deinit and free host queues\n" ) );
    host_rtos_deinit_queue( &supplicant_host_workspace->host_workspace.event_queue );
    host_rtos_deinit_queue( &supplicant_host_workspace->outgoing_packet_queue );

    if ( supplicant_host_workspace != NULL )
    {
        IF_TO_WORKSPACE( supplicant_host_workspace->host_workspace.interface ) = NULL;

        /* Delete the supplicant thread */
        if ( workspace->supplicant_result != SUPPLICANT_NOT_STARTED )
        {
            SUPPLICANT_DEBUG( ("Delete supplicant thread\n" ) );
            host_rtos_delete_terminated_thread( &supplicant_host_workspace->host_workspace.thread );
        }

        if ( supplicant_host_workspace->host_workspace.thread_stack != NULL )
        {
            SUPPLICANT_DEBUG( ("MEM : FREE : Free supplicant thread stack\n" ) );
            besl_host_free( supplicant_host_workspace->host_workspace.thread_stack );
            supplicant_host_workspace->host_workspace.thread_stack = NULL;
        }

        SUPPLICANT_DEBUG( ("MEM : FREE : Free supplicant host workspace\n" ) );
        besl_host_free( supplicant_host_workspace );
        workspace->supplicant_host_workspace = NULL;
    }

    if ( workspace->buffer != NULL )
    {
        SUPPLICANT_DEBUG( ("MEM : FREE : Free supplicant defragmentation buffer\n" ) );
        besl_host_free( workspace->buffer );
        workspace->buffer = NULL;
    }

    SUPPLICANT_DEBUG( ("Deinitialize TLS context, cert and identity\n" ) );
    wiced_tls_deinit_root_ca_certificates();
    wiced_tls_deinit_identity( workspace->tls_context->identity );
    wiced_tls_deinit_context( workspace->tls_context );

    memset( workspace, 0, sizeof( *workspace ) );

    SUPPLICANT_DEBUG( ("Exit success!\n" ) );
    return BESL_SUCCESS;
}


/* This function is called by the WWD thread so do not print from here unless the WWD thread stack size has been increased by 4K to allow for printing. */
static void supplicant_eapol_packet_handler( wiced_buffer_t buffer, wwd_interface_t interface )
{
    supplicant_workspace_t* workspace;

    if ( interface == WWD_STA_INTERFACE )
    {
        workspace = IF_TO_WORKSPACE( interface );
        if ( workspace == NULL )
        {
            host_buffer_release( buffer, WWD_NETWORK_RX );
        }
        else
        {
            besl_queue_message_packet( workspace, BESL_EVENT_EAPOL_PACKET_RECEIVED, buffer );
        }
    }
    else
    {
        SUPPLICANT_DEBUG( ( "EAPOL packet arriving on incorrect interface %u\n", (unsigned int) interface ) );
    }
}

besl_result_t besl_supplicant_stop( supplicant_workspace_t* workspace )
{
    SUPPLICANT_DEBUG( ( "Supplicant result = [%d]\n", workspace->supplicant_result ) );
    if ( workspace->supplicant_result != SUPPLICANT_NOT_STARTED )
    {
        besl_host_workspace_t* host_workspace = &((supplicant_host_workspace_t*) workspace->supplicant_host_workspace)->host_workspace;

        SUPPLICANT_DEBUG( ( "Signal supplicant thread to exit\n" ) );
        besl_queue_message_packet( workspace, BESL_EVENT_ABORT_REQUESTED, NULL );

        SUPPLICANT_DEBUG( ( "Wait for supplicant thread to exit\n\n" ) );
        host_rtos_join_thread( &host_workspace->thread );
    }

    return BESL_SUCCESS;
}


besl_result_t besl_supplicant_management_set_event_handler( supplicant_workspace_t* workspace, wiced_bool_t enable )
{
    wwd_result_t        result;

    if ( enable == WICED_TRUE )
    {
        result = wwd_management_set_event_handler( supplicant_events, supplicant_external_event_handler, workspace, ( wwd_interface_t ) workspace->interface );
    }
    else
    {
        result = wwd_management_set_event_handler( supplicant_events, NULL, workspace, ( wwd_interface_t ) workspace->interface );
    }

    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error setting supplicant event handler %u\n", (unsigned int)result));
    }
    return (besl_result_t) result;
}


/* This function is called by the WWD thread so do not print from here unless the WWD thread stack size has been increased by 4K to allow for printing. */
static void* supplicant_external_event_handler( const wwd_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data )
{
    supplicant_workspace_t* workspace = (supplicant_workspace_t*) handler_user_data;

        SUPPLICANT_DEBUG(("[%s()] : L%d : Event type = [%s], Status = [%s], Reason = [%s]\n", __FUNCTION__, __LINE__, wwd_event_to_string(event_header->event_type),
                wwd_status_to_string(event_header->status), wwd_reason_to_string(event_header->reason) ));
    switch ( event_header->event_type )
    {
        case WLC_E_DEAUTH_IND:
        case WLC_E_DISASSOC_IND:
        {
            // TBD
            break;
        }

        case WLC_E_LINK:
            SUPPLICANT_DEBUG( ("Supplicant received link event\n" ) );
            if ( ( event_header->flags & WLC_EVENT_MSG_LINK ) != 0 )
            {
                if ( workspace->current_sub_stage == SUPPLICANT_EAP_START )
                {
                    workspace->current_main_stage = SUPPLICANT_INITIALISING;
                    workspace->start_time = host_rtos_get_time( );

                    SUPPLICANT_DEBUG(("[%s()] : L%d : Start timer to wait for EAPOL ID request\n", __FUNCTION__, __LINE__ ));
                    besl_host_start_timer( workspace->supplicant_host_workspace, EAPOL_PACKET_TIMEOUT ); /* Start a timer to wait for EAP ID Request */
                }

            }
            /* TBD handle link down and link up events */
            break;

        default:
            break;
    }

    return handler_user_data;
}


static void wiced_supplicant_thread_main( wwd_thread_arg_t arg )
{
    wiced_time_t                 current_time;
    besl_result_t                result;
    besl_event_message_t         message;
    supplicant_workspace_t*      workspace = (supplicant_workspace_t*)arg;
    besl_host_workspace_t*       host      = &((supplicant_host_workspace_t*)workspace->supplicant_host_workspace)->host_workspace;

    workspace->supplicant_result = SUPPLICANT_IN_PROGRESS;

    /* Now that our queue is initialized we can flag the workspace as active */
    IF_TO_WORKSPACE( workspace->interface ) = workspace;

    workspace->start_time = host_rtos_get_time( );

    if ( besl_supplicant_management_set_event_handler( workspace, WICED_TRUE ) != SUPPLICANT_SUCCESS )
    {
        SUPPLICANT_DEBUG( ( "Supplicant unable to set management event handler.\n" ) );
    }

    while ( workspace->supplicant_result != SUPPLICANT_ABORTED )
    {
        uint32_t time_to_wait;

        SUPPLICANT_DEBUG(("[%s()] : L%d : In processing loop...\n", __FUNCTION__, __LINE__));

        workspace->eap_handshake_current_time = host_rtos_get_time( );

        if ( ( workspace->eap_handshake_start_time != 0 ) && ( ( workspace->eap_handshake_current_time - workspace->eap_handshake_start_time ) >= EAP_HANDSHAKE_TIMEOUT_IN_MSEC ) )
        {
            SUPPLICANT_DEBUG(( "eap Handshake timed out...\n" ));
            supplicant_eap_handshake_cleanup( workspace );
        }

        /* Update EAP timeout values based on EAP handshake state */
        if ( workspace->current_main_stage != SUPPLICANT_INITIALISED )
        {
            current_time = host_rtos_get_time( );

            if (( current_time - workspace->start_time ) >= SUPPLICANT_HANDSHAKE_ATTEMPT_TIMEOUT )
            {
                SUPPLICANT_DEBUG( ( "Total handshake timedout expired. Quit processing\n" ) );
                workspace->supplicant_result = SUPPLICANT_ABORTED;
                continue;
            }

            SUPPLICANT_DEBUG( ( "Calculate the wait time\n" ) );
            time_to_wait = ( SUPPLICANT_HANDSHAKE_ATTEMPT_TIMEOUT ) - ( current_time - workspace->start_time );
            if ( host->timer_timeout != 0 )
            {
                SUPPLICANT_DEBUG( ( "Timer timeout is enabled. Find the min wait time again\n" ) );
                time_to_wait = MIN( time_to_wait, host->timer_timeout - (current_time - host->timer_reference));
            }
        }
        else
        {
            time_to_wait = EAPOL_PACKET_TIMEOUT;
        }

        SUPPLICANT_DEBUG( ( "Waiting for EAPOL packet\n" ) );
        if ( host_rtos_pop_from_queue( &host->event_queue, &message, time_to_wait ) != WWD_SUCCESS )
        {
            /* Create a timeout message to process */
            SUPPLICANT_DEBUG(( "EAPOL packet wait timed out\n" ));
            message.event_type = BESL_EVENT_TIMER_TIMEOUT;
            message.data.value = 0;
        }

        /* Process the message */
        SUPPLICANT_DEBUG( ( "Process EAPOL packet\n" ) );
        result = supplicant_process_event( workspace, &message );
        if ( result != SUPPLICANT_SUCCESS )
        {
            SUPPLICANT_DEBUG( ( "Supplicant error %u\n", (unsigned int)result ) );
        }
    }

    SUPPLICANT_DEBUG( ( "Terminate TLS agent and PEAP threads, if running\n" ) );
    supplicant_eap_handshake_cleanup( workspace );

    SUPPLICANT_DEBUG( ( "Reset supplicant event handlers\n" ) );
    if ( besl_supplicant_management_set_event_handler( workspace, WICED_FALSE ) != SUPPLICANT_SUCCESS )
    {
        SUPPLICANT_DEBUG( ( "Supplicant unable to remove management event handler.\n" ) );
    }
}


besl_result_t supplicant_outgoing_pop( void* workspace, besl_event_message_t* message )
{
    supplicant_host_workspace_t* supplicant_host = (supplicant_host_workspace_t*)workspace;

    return (besl_result_t) host_rtos_pop_from_queue( &supplicant_host->outgoing_packet_queue, message, WICED_NEVER_TIMEOUT );
}


besl_result_t supplicant_outgoing_push( void* workspace, besl_event_message_t* message )
{
    supplicant_host_workspace_t* supplicant_host = (supplicant_host_workspace_t*)workspace;
    SUPPLICANT_DEBUG( ( "Push outgoing packet to queue\n" ) );
    return (besl_result_t) host_rtos_push_to_queue( &supplicant_host->outgoing_packet_queue, message, WICED_NEVER_TIMEOUT );
}

#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY

/* Create a Phase2 workspace to run EAP in it. */
besl_result_t supplicant_phase2_init( supplicant_workspace_t* workspace, eap_type_t type )
{
    supplicant_phase2_workspace_t* phase2_workspace;

    SUPPLICANT_DEBUG( ( "Start supplicant_phase2_init\n" ) );
    SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : MALLOC : Allocating PEAP workspace\n", __FUNCTION__, __LINE__ ));
    phase2_workspace = besl_host_malloc( "supplicant phase2 workspace ", sizeof(supplicant_phase2_workspace_t) );
    if ( phase2_workspace == NULL )
    {
        SUPPLICANT_DEBUG( ( "Can't allocate memory for peap workspace, out of heap space!\n" ) );
        return SUPPLICANT_OUT_OF_HEAP_SPACE;
    }
    memset( phase2_workspace, 0, sizeof(supplicant_phase2_workspace_t) );
    workspace->ptr_phase2 = phase2_workspace;

#ifdef RTOS_USE_STATIC_THREAD_STACK
    SUPPLICANT_DEBUG(("[%s()] : L%d : MEM : MALLOC : Allocating Phase2 thread stack\n", __FUNCTION__, __LINE__ ));
    phase2_workspace->thread_stack = besl_host_malloc( "supplicant Phase2 stack", SUPPLICANT_THREAD_STACK_SIZE );
    if ( phase2_workspace->thread_stack == NULL )
    {
        besl_host_free( workspace->ptr_phase2 );
        workspace->ptr_phase2 = NULL;
        SUPPLICANT_DEBUG( ( "Can't allocate memory for Phase2 thread stack, out of heap space!\n" ) );
        return SUPPLICANT_ERROR_STACK_MALLOC_FAIL;
    }
    memset( phase2_workspace->thread_stack, 0, SUPPLICANT_THREAD_STACK_SIZE );
#else
    phase2_workspace->thread_stack = NULL;
#endif

    phase2_workspace->state.eap_type   = type;
    phase2_workspace->state.result     = SUPPLICANT_NOT_STARTED;
    phase2_workspace->state.main_stage = SUPPLICANT_INITIALISING;
    phase2_workspace->state.sub_stage  = SUPPLICANT_EAP_START;

    besl_supplicant_set_inner_identity( workspace, workspace->eap_type, &workspace->inner_identity );

    SUPPLICANT_DEBUG( ( "supplicant_phase2_init done\n" ) );
    return BESL_SUCCESS;
}

/* Free a previously create phase2 work space. */
besl_result_t supplicant_phase2_deinit( supplicant_workspace_t* workspace )
{
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;
    wwd_result_t res;

    (void) res;

    if ( phase2_workspace != NULL )
    {
        SUPPLICANT_DEBUG(( "Terminate PEAP thread\n" ));
        res = host_rtos_delete_terminated_thread( &phase2_workspace->thread );
        SUPPLICANT_DEBUG( ( "PEAP thread terminated. Result = [%d]\n", res ) );

        if ( phase2_workspace->thread_stack != NULL )
        {
            SUPPLICANT_DEBUG( ( "MEM : FREE : Free PEAP thread stack\n" ) );
            besl_host_free( phase2_workspace->thread_stack );
            phase2_workspace->thread_stack = NULL;
        }

        SUPPLICANT_DEBUG( ( "MEM : FREE : Free PEAP workspace\n" ) );
        besl_host_free( workspace->ptr_phase2 );
        workspace->ptr_phase2 = NULL;
    }

    return BESL_SUCCESS;
}

besl_result_t supplicant_phase2_start( supplicant_workspace_t* workspace )
{
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;

    /* validating input parameters */
    if(workspace->eap_type == EAP_TYPE_TTLS)
    {
        if(workspace->tunnel_auth_type != TUNNEL_TYPE_EAP)
        {
            return BESL_ERROR;
        }
        if(!( workspace->inner_eap_type == EAP_TYPE_LEAP || workspace->inner_eap_type == EAP_TYPE_MSCHAPV2))
        {
            return BESL_ERROR;
        }
    }
    else if(workspace->eap_type != EAP_TYPE_PEAP)
    {
        return BESL_ERROR;
    }

    phase2_workspace->state.result = SUPPLICANT_IN_PROGRESS;
    SUPPLICANT_DEBUG( ( "Start supplicant_phase2_start\n" ) );
    return (besl_result_t) host_rtos_create_thread_with_arg( &phase2_workspace->thread, supplicant_phase2_thread, "phase2", phase2_workspace->thread_stack, SUPPLICANT_THREAD_STACK_SIZE, RTOS_HIGHER_PRIORTIY_THAN(RTOS_DEFAULT_THREAD_PRIORITY), (wwd_thread_arg_t) workspace );
}

besl_result_t supplicant_phase2_stop( supplicant_workspace_t* workspace )
{
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;

    SUPPLICANT_DEBUG( ( "Stop supplicant_peap_thread\n" ));
    if ( (phase2_workspace != NULL) && (phase2_workspace->state.result != SUPPLICANT_NOT_STARTED) )
    {
        SUPPLICANT_DEBUG( ( "Signal abort to Phase2 thread\n" ));
        phase2_workspace->state.result = SUPPLICANT_ABORTED;
        SUPPLICANT_DEBUG( ( "Wait for PEAP thread to exit\n" ));
        host_rtos_join_thread( &phase2_workspace->thread );
        SUPPLICANT_DEBUG( ( "PEAP thread exited\n" ));
    }

    return BESL_SUCCESS;
}

besl_result_t supplicant_inner_packet_set_data( besl_packet_t* packet, int32_t size )
{
    return (besl_result_t) host_buffer_add_remove_at_front( (wiced_buffer_t *) packet, size );
}

static void supplicant_phase2_thread( wwd_thread_arg_t arg )
{
    supplicant_workspace_t*      workspace      = (supplicant_workspace_t*) arg;
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;
    besl_result_t (* phase2_event_handler ) (supplicant_workspace_t* , besl_packet_t);

    SUPPLICANT_DEBUG( ( "\nsupplicant_peap_thread started...\n" ) );

    if (workspace->eap_type == EAP_TYPE_PEAP)
    {
        phase2_event_handler = supplicant_process_peap_event;

    }
    else if (workspace->eap_type == EAP_TYPE_TTLS)
    {
        phase2_event_handler = supplicant_process_ttls_phase2_event;
    }
    else
    {
        return;
    }

    /* Wait until TLS is done before starting to receive packets */
    SUPPLICANT_DEBUG( ( "Wait for TLS handshake to finish\n" ) );

    while ( workspace->tls_context->context.state != MBEDTLS_SSL_HANDSHAKE_OVER )
    {
        host_rtos_delay_milliseconds( 10 );
    }

    SUPPLICANT_DEBUG( ( "TLS handshake finished. Start PEAP processing loop\n" ) );

    while ( phase2_workspace->state.result == SUPPLICANT_IN_PROGRESS )
    {
        besl_packet_t packet;
        SUPPLICANT_DEBUG( ( "Waiting to receive EAP packet\n" ) );
        if ( wiced_tls_receive_eap_packet( workspace, &packet, 100 ) == WICED_SUCCESS )
        {
            phase2_event_handler( workspace, packet );
            besl_host_free_packet( packet );
        }
    }
    SUPPLICANT_DEBUG( ( "supplicant_phase2_thread end\n" ) );
    WICED_END_OF_CURRENT_THREAD( );
}

uint32_t supplicant_eap_get_timer( supplicant_workspace_t* workspace )
{
    return host_rtos_get_time();
}
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */
