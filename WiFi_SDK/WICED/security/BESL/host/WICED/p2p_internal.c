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
#include "wiced.h"
#include "besl_host_rtos_structures.h"
#include "p2p_structures.h"
#include "wiced_p2p.h"
#include "p2p_host_interface.h"
#include "internal/wwd_sdpcm.h"
#include "wiced_wps.h"
#include "wps_host_interface.h"
#include "internal/wiced_internal_api.h"
#include "wps_p2p_interface.h"
#include "p2p_frame_writer.h"
#include "wwd_buffer_interface.h"
#include "wiced_utilities.h"
#include <string.h>

/******************************************************
 *                      Macros
 ******************************************************/

#define CHECK_IOCTL_BUFFER( buff )  if ( buff == NULL ) {  wiced_assert("Allocation failed\n", 0 == 1); return BESL_BUFFER_ALLOC_FAIL; }
#define CHECK_RETURN( expr )  { besl_result_t check_res = (expr); if ( check_res != BESL_SUCCESS ) { wiced_assert("Command failed\n", 0 == 1); return check_res; } }

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_NUMBER_OF_P2P_MESSAGES    (20)
#define P2P_THREAD_STACK_SIZE         (5*1024)
#define P2P_MAX_DISCOVERABLE_INTERVAL (3)
#define P2P_BEACON_INTERVAL_MS        (100)
#define P2P_LISTEN_MODE_DWELL_TIME    (300)     /* Milliseconds */
#define DOT11_CAP_PRIVACY             (0x0010)  /* d11 cap. privacy */
#define DOT11_PMK_LEN                 (32)
#define P2P_MAX_RESPONSE_WAIT_TIME    (350)
#define P2P_MIN_RESPONSE_WAIT_TIME    (100)

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
 *               Function Declarations
 ******************************************************/

static void           p2p_thread_main                 ( wwd_thread_arg_t arg );
static besl_result_t  p2p_scan                        ( p2p_workspace_t* workspace, uint16_t scan_action, wiced_bool_t all_channels );
static besl_result_t  p2p_set_discovery_state         ( p2p_discovery_state_t state, uint32_t channel );
static void           p2p_discover                    ( p2p_workspace_t* workspace );
static besl_result_t  p2p_start_client                ( p2p_workspace_t* workspace );
static besl_result_t  p2p_group_owner_stop            ( p2p_workspace_t* workspace );
static besl_result_t  p2p_group_client_stop           ( p2p_workspace_t* workspace );
static void           p2p_abort_scan                  ( p2p_workspace_t* workspace );
static besl_result_t  p2p_join_group_owner            ( p2p_workspace_t* workspace );
static void           p2p_start_timer                 ( void* workspace, uint32_t timeout );
void                  p2p_wps_internal_result_callback( wps_result_t* result );
#ifdef P2P_IP_ALLOCATION
static besl_result_t  p2p_get_ip_addr( wiced_ip_setting_t* ip_setting );
#endif

/******************************************************
 *               Variable Definitions
 ******************************************************/

static host_thread_type_t p2p_thread;
static host_queue_type_t  p2p_message_queue;
static p2p_message_t      p2p_message_queue_buffer        [ MAX_NUMBER_OF_P2P_MESSAGES ];
#ifdef RTOS_USE_STATIC_THREAD_STACK
static uint8_t            p2p_thread_stack                [ P2P_THREAD_STACK_SIZE ];
#else
#define p2p_thread_stack (NULL)
#endif /* ifdef RTOS_USE_STATIC_THREAD_STACK */

static const wwd_event_num_t p2p_group_client_events[] = { WLC_E_ESCAN_RESULT, WLC_E_PROBREQ_MSG, WLC_E_ACTION_FRAME, WLC_E_ACTION_FRAME_COMPLETE, WLC_E_DEAUTH, WLC_E_DEAUTH_IND, WLC_E_DISASSOC, WLC_E_DISASSOC_IND, WLC_E_NONE };
const        wwd_event_num_t p2p_discovery_events[]    = { WLC_E_ESCAN_RESULT, WLC_E_P2P_DISC_LISTEN_COMPLETE, WLC_E_PROBREQ_MSG, WLC_E_ACTION_FRAME, WLC_E_ACTION_FRAME_COMPLETE, WLC_E_NONE };
const        wwd_event_num_t p2p_group_owner_events[]  = { WLC_E_ACTION_FRAME, WLC_E_ASSOC_IND, WLC_E_REASSOC_IND, WLC_E_ACTION_FRAME_COMPLETE, WLC_E_DEAUTH, WLC_E_DEAUTH_IND, WLC_E_DISASSOC, WLC_E_DISASSOC_IND, WLC_E_PROBREQ_MSG, WLC_E_NONE };

/******************************************************
 *               Function Definitions
 ******************************************************/

void p2p_thread_start( p2p_workspace_t* workspace )
{
    /* Create the message queue */
    memset( p2p_message_queue_buffer , 0, sizeof(p2p_message_queue_buffer) );
    host_rtos_init_queue(&p2p_message_queue, p2p_message_queue_buffer, sizeof(p2p_message_queue_buffer), sizeof(p2p_message_t));

    /* Create the P2P thread */
    workspace->p2p_initialised = 1;
#ifdef RTOS_USE_STATIC_THREAD_STACK
    memset( p2p_thread_stack, 0, P2P_THREAD_STACK_SIZE );
#endif /* ifdef RTOS_USE_STATIC_THREAD_STACK */
    host_rtos_create_thread_with_arg( &p2p_thread, p2p_thread_main, "p2p_thread", p2p_thread_stack, P2P_THREAD_STACK_SIZE, RTOS_HIGHER_PRIORTIY_THAN(RTOS_DEFAULT_THREAD_PRIORITY), (wwd_thread_arg_t)workspace );
}

static void p2p_thread_main( wwd_thread_arg_t arg )
{
    p2p_workspace_t*              workspace = (p2p_workspace_t*)arg;
    wwd_result_t                  result;
    p2p_message_t                 message;
    wiced_time_t                  current_time;
    uint8_t                       tie_breaker[2];
    p2p_discovered_device_t       discovery_target;
    p2p_discovered_device_t       discovery_requestor;
    p2p_client_info_descriptor_t* associated_device;
    p2p_discovered_device_t*      peer_device;
    besl_mac_t*                   associated_legacy_device;
    wps_result_t*                 wps_result;

    if ( workspace->group_owner_is_up == 1 )
    {
        p2p_start_timer( workspace, 5000 );
    }
    else if ( workspace->group_client_is_up == 0 )
    {
        workspace->p2p_result = BESL_IN_PROGRESS;
    }
    workspace->p2p_thread_running = 1;

    while ( workspace->p2p_current_state != P2P_STATE_ABORTED )
    {
        uint32_t time_to_wait;

        /* Check for group formation timeout */
        if ( ( workspace->group_owner_is_up == 0 ) && ( workspace->group_client_is_up == 0 ) )
        {
            if ( host_rtos_get_time( ) > ( workspace->group_formation_start_time + workspace->group_formation_timeout ) )
            {
                if ( workspace->p2p_current_state == P2P_STATE_NEGOTIATING )
                {
                    if ( workspace->initiate_negotiation == 1 )
                    {
                        p2p_stop( workspace );
                    }
                    else
                    {
                        workspace->p2p_current_state = P2P_STATE_DISCOVERY; /* If we didn't initiate group formation then go back to discovery */
                    }
                }
                else if ( workspace->looking_for_group_owner == 1 )
                {
                    p2p_stop( workspace );
                }
            }
        }

        /* Check for failure to stop p2p */
        if ( workspace->p2p_current_state == P2P_STATE_FAILURE )
        {
            p2p_stop( workspace );
        }

        if ( workspace->timer_timeout > 0 )
        {
            current_time = host_rtos_get_time( );
            if ( ( workspace->timer_reference + workspace->timer_timeout ) <= current_time )
            {
                time_to_wait = 0;
            }
            else
            {
                time_to_wait = workspace->timer_timeout - (current_time - workspace->timer_reference);
            }
        }
        else
        {
            time_to_wait = WICED_NEVER_TIMEOUT;
        }

        if ( host_rtos_pop_from_queue(&p2p_message_queue, &message, time_to_wait ) != WWD_SUCCESS )
        {
            /* Create a timeout message */
            message.type = P2P_EVENT_TIMER_TIMEOUT;
            message.data = 0;
            workspace->timer_timeout = 0;
        }

        switch(message.type)
        {
            case P2P_EVENT_SCAN_COMPLETE:
                if ( workspace->p2p_current_state == P2P_STATE_DISCOVERY )
                    {
                    if ( workspace->looking_for_group_owner == 1 )
                    {
                        // XXX should check if device is actually a GO
                        workspace->candidate_device = besl_p2p_host_find_device( workspace, (besl_mac_t*)&workspace->group_candidate.p2p_device_address );
                        if ( workspace->candidate_device != NULL )
                        {
                            if ( !NULL_MAC( workspace->candidate_device->p2p_device_address.octet) )
                            {
                                memcpy(&workspace->group_candidate.operating_channel, &workspace->candidate_device->operating_channel, sizeof(p2p_channel_info_t));
                                memcpy(&workspace->group_candidate.bssid, &workspace->candidate_device->p2p_interface_address, sizeof(besl_mac_t));
                                besl_p2p_send_action_frame( workspace, workspace->candidate_device, p2p_write_provision_discovery_request, (uint32_t)workspace->candidate_device->operating_channel.channel, 500 );
                            }
                            else
                            {
                                p2p_scan( workspace, WL_SCAN_ACTION_START, WICED_TRUE ); // Scan all channels when looking for an existing group owner
                            }
                        }
                        else
                        {
                            p2p_scan( workspace, WL_SCAN_ACTION_START, WICED_TRUE ); // Scan all channels when looking for an existing group owner
                        }
                    }
                    else
                    {
                        if ( workspace->initiate_negotiation == 1 )
                        {
                            workspace->candidate_device = besl_p2p_host_find_device( workspace, (besl_mac_t*)&workspace->group_candidate.p2p_device_address );
                            if ( workspace->candidate_device != NULL )
                            {
                                if ( !NULL_MAC(workspace->candidate_device->p2p_device_address.octet) )
                                {
                                    BESL_DEBUG(("Found target device\n"));
                                    message.type = P2P_EVENT_FOUND_TARGET_DEVICE;
                                    message.data = NULL;
                                    host_rtos_push_to_queue(&p2p_message_queue, &message, WICED_NEVER_TIMEOUT);
                                }
                                else
                                {
                                    p2p_discover(workspace);
                                }
                            }
                        }
                        else
                        {
                            p2p_discover(workspace);
                        }
                    }
                }
                break;

            case P2P_EVENT_DISCOVERY_COMPLETE:
            case P2P_EVENT_START_REQUESTED:
                workspace->p2p_current_state = P2P_STATE_DISCOVERY;
                p2p_set_discovery_state( P2P_DISCOVERY_STATE_LISTEN, 0 );
                result = (wwd_result_t) p2p_scan( workspace, WL_SCAN_ACTION_START, WICED_TRUE ); /* Scan social channels or scan all channels? GOs may not be on social channels. */
                wiced_assert("scan failed", result == WWD_SUCCESS);
                REFERENCE_DEBUG_ONLY_VARIABLE( result );
                break;

            case P2P_EVENT_START_LISTEN_DISCOVERY:
                workspace->p2p_current_state = P2P_STATE_DISCOVERY;
                workspace->p2p_listen_only = WICED_TRUE;
                p2p_discover(workspace);
                break;

            case P2P_EVENT_START_NEGOTIATION:
                /* Create random tie breaker bit for the first negotiation request */
                besl_host_random_bytes(tie_breaker, 2); /* Have to ask for a minimum of two bytes */
                workspace->group_owner_tie_breaker = tie_breaker[0] & 0x01;
                workspace->group_formation_start_time = host_rtos_get_time( );
                p2p_discover(workspace);
                break;

            case P2P_EVENT_FIND_GROUP_OWNER:
                workspace->looking_for_group_owner = 1;
                workspace->group_formation_start_time = host_rtos_get_time( );
                break;

            case P2P_EVENT_PACKET_TO_BE_SENT:
                if ( ( workspace->sent_negotiation_request == 1 ) ||
                     ( workspace->sent_negotiation_confirm == 1 ) ||
                     ( workspace->sent_go_discoverability_request == 1 ) ||
                     ( workspace->sent_provision_discovery_request == 1 ) ||
                     ( workspace->sent_invitation_request == 1 ) )
                {
                    if ( workspace->sent_provision_discovery_request == 1 )
                    {
                        /* For testcase 5.1.4 of P2P WFA CERTIFIED
                         * - Target Testbed : Atheros_P2P
                         *
                         * We had reduced the time interval of sending PD_request(Provision Discovery Request)to Atheros_P2P.
                         *
                         * Due to Atheros_P2P always reply ONE PD_response packet when receiving PD_request.
                         * DUT need to wait for 350ms to send next PD_requst, if it missed PD_response of Atheros_P2P that time.
                         * 350ms was too long and easy to hit the timeout of PD process, if DUT always miss PD_response.
                         *
                         * Instead of using 100ms, we can improve the success chance of PD process.
                         *
                         *
                         * PS :
                         *  Quote from [Wi-Fi Peer-to-Peer (P2P) Technical Specification v1.5 (page.136)]
                         *  State: Listen
                         *  Actions:
                         *      - Pick Random Dwell Time
                         *          On entering the Listen state, the P2P Device chooses a random dwell
                         *          between minDiscoverableInterval and maxDiscoverableInterval
                         *          (the default values for these are 1 and 3 times 100TU respectively).
                         */
                        p2p_start_timer( workspace, P2P_MIN_RESPONSE_WAIT_TIME );
                    }
                    else
                    {
                        p2p_start_timer( workspace, P2P_MAX_RESPONSE_WAIT_TIME ); /* This value needs to be offset to the other devices listen interval and longer than our listen mode dwell time */
                    }
                }

                /* Abort scan to cancel previous action frame's dwell time and specifically set channel or our negotiation phase is unreliable */
                if ( workspace->p2p_current_state != P2P_STATE_GROUP_OWNER )
                {
                    p2p_abort_scan( workspace );
                    if ( workspace->current_channel != 0 )
                    {
                        result = wwd_wifi_set_channel( WWD_STA_INTERFACE, workspace->current_channel );
                        wiced_assert("set channel failed", result == WWD_SUCCESS);
                        REFERENCE_DEBUG_ONLY_VARIABLE( result );
                    }
                }
                /* Send the packet */
                result = wwd_sdpcm_send_iovar( SDPCM_SET, (wiced_buffer_t) message.data, NULL, WWD_STA_INTERFACE );
                if (result != WWD_SUCCESS)
                {
                    /*  Packet has been lost.. Maybe. Don't think we can recover it though */
                    BESL_DEBUG(("P2P negotiating and lost the packet\r\n"));
                }
                if ( workspace->p2p_current_state == P2P_STATE_DISCOVERY )
                {
                    p2p_discover(workspace);
                }
                break;

            case P2P_EVENT_NEGOTIATION_COMPLETE:
                BESL_INFO( ("P2P negotiation complete...\n") );

                workspace->looking_for_group_owner = 0;

                /*  Remove P2P event handler */
                result = wwd_management_set_event_handler( p2p_discovery_events, NULL, workspace, WWD_STA_INTERFACE );
                wiced_assert("set handler failed", result == WWD_SUCCESS);
                REFERENCE_DEBUG_ONLY_VARIABLE( result );

                p2p_abort_scan( workspace ); /* Abort scan or the GO may not be able to come up */
                p2p_remove_ies( workspace );

                /*  Disable discovery */
                wiced_buffer_t buffer;
                uint32_t* data;

                p2p_set_discovery_state( P2P_DISCOVERY_STATE_SCAN, 0 );
                data = wwd_sdpcm_get_iovar_buffer(&buffer, 4, IOVAR_STR_P2P_DISC);
                if ( data == NULL )
                {
                    wiced_assert("Allocation failed\n", 0 == 1);
                    return;
                }
                *data = 0;
                result = wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE); /* This IOVAR must be sent on STA interface */
                if (result != WWD_SUCCESS)
                {
                    BESL_DEBUG(("Unable to disable discovery\r\n"));
                }
                wiced_assert("", result == WWD_SUCCESS);

                if ( workspace->i_am_group_owner == 1 )
                {
                    if ( besl_p2p_group_owner_start( workspace ) != BESL_SUCCESS )
                    {
                        BESL_ERROR(("Failed to start group owner!\r\n"));
                        workspace->p2p_current_state = P2P_STATE_FAILURE;
                        break;
                    }

                    /* Start the registrar if we're not reinvoking a group via the invitation procedure */
                    if ( workspace->reinvoking_group == 0 )
                    {
                        besl_p2p_start_registrar();
                    }
                }
                else
                {
                    if ( workspace->group_candidate.configuration_timeout > 0 )
                    {
                        host_rtos_delay_milliseconds(workspace->group_candidate.configuration_timeout * 10); /* Run the delay here so we can still ACK negotiation confirmations */
                    }
                    workspace->p2p_current_state = P2P_STATE_NEGOTIATION_COMPLETE;
                    p2p_start_client( workspace );
                }
                break;

            case P2P_EVENT_STOP_REQUESTED:
                BESL_DEBUG(("Stopping P2P thread\n"));
                /* Free WPS agent if necessary */
                if ( workspace->p2p_wps_agent != NULL )
                {
                    if ( workspace->p2p_wps_agent->wps_result != WPS_NOT_STARTED )
                    {
                        if ( besl_wps_abort( workspace->p2p_wps_agent ) == BESL_SUCCESS )
                        {
                            BESL_DEBUG(("besl_p2p_deinit: P2P WPS agent stopped\r\n"));
                        }
                    }
                    host_rtos_delay_milliseconds( 10 ); /* Delay to allow the WPS thread to complete */
                    besl_wps_deinit( workspace->p2p_wps_agent );
                    besl_host_free( workspace->p2p_wps_agent );
                    workspace->p2p_wps_agent = NULL;
                }
                workspace->p2p_current_state = P2P_STATE_ABORTED;
                break;

            case P2P_EVENT_TIMER_TIMEOUT:
                if ( ( ( workspace->sent_negotiation_confirm == 1 ) || ( workspace->sent_negotiation_request == 1 ) ) && ( workspace->p2p_current_state == P2P_STATE_NEGOTIATING ) )
                {
                    workspace->sent_negotiation_confirm = 0;
                    if ( workspace->candidate_device != NULL )
                    {
                        besl_p2p_send_action_frame( workspace, workspace->candidate_device, p2p_write_negotiation_request, (uint32_t)workspace->candidate_device->listen_channel, 2 );
                    }
                }
                else if ( workspace->sent_go_discoverability_request == 1 )
                {
                    memcpy( &discovery_target.p2p_device_address, &workspace->discovery_target->p2p_interface_address, sizeof( besl_mac_t ) );
                    besl_p2p_send_action_frame( workspace, &discovery_target, p2p_write_go_discoverability_request, workspace->current_channel, 2 );
                }
                else if ( workspace->sent_provision_discovery_request == 1 )
                {
                    if ( workspace->candidate_device != NULL )
                    {
                        besl_p2p_send_action_frame( workspace, workspace->candidate_device, p2p_write_provision_discovery_request, (uint32_t)workspace->candidate_device->operating_channel.channel, 500 );
                    }
                }
                else if ( workspace->sent_invitation_request == 1)
                {
                    if ( workspace->candidate_device != NULL )
                    {
                        besl_p2p_send_action_frame( workspace, workspace->candidate_device, p2p_write_invitation_request, workspace->current_channel, 2 );
                    }
                    /* The timer is single threaded so kick off another 5 second interval in case we overwrote the last one */
                    if ( workspace->p2p_current_state == P2P_STATE_GROUP_OWNER )
                    {
                        p2p_start_timer( workspace, 5000 );
                    }
                }
                else if ( workspace->group_owner_is_up == 1 )
                {
                    p2p_update_devices( workspace );
                    p2p_start_timer( workspace, 5000 );
                }
                break;

            case P2P_EVENT_FOUND_TARGET_DEVICE:
                workspace->p2p_current_state = P2P_STATE_NEGOTIATING;
                if ( workspace->candidate_device != NULL )
                {
                    BESL_DEBUG(("Device listen channel %u\n", (unsigned int)workspace->candidate_device->listen_channel));
                    besl_p2p_send_action_frame( workspace, workspace->candidate_device, p2p_write_negotiation_request, (uint32_t)workspace->candidate_device->listen_channel, 2 );
                    workspace->candidate_device->status = P2P_DEVICE_WAS_INVITED_TO_FORM_GROUP;
                }
                break;

            case P2P_EVENT_START_REGISTRAR:
                    /* Cleanup the WPS workspace, advertise selected registrar and then kick off the registrar */
                if (workspace->p2p_wps_agent->wps_result == WPS_IN_PROGRESS )
                {
                    if ( workspace->p2p_wps_agent->wps_mode == WPS_PBC_MODE )
                    {
                        WPRINT_APP_INFO(("Restarting 2 minute window\n"));
                        besl_wps_restart( workspace->p2p_wps_agent );
                    }
                    else
                    {
                        WPRINT_APP_INFO(("P2P thread: WPS already running %u\n", (unsigned int)workspace->p2p_wps_agent->wps_result));
                    }
                    break;
                }
                else if ( workspace->p2p_wps_agent->wps_result != WPS_NOT_STARTED )
                {
                    besl_wps_deinit( workspace->p2p_wps_agent );
                    memset( workspace->p2p_wps_agent, 0, sizeof(wps_agent_t) );
                    workspace->p2p_wps_agent->is_p2p_registrar = 1;
                    workspace->p2p_wps_agent->wps_agent_owner = workspace;
                    besl_wps_init( workspace->p2p_wps_agent, workspace->wps_device_details, WPS_REGISTRAR_AGENT, WWD_P2P_INTERFACE );
                    wps_register_result_callback( workspace->p2p_wps_agent, workspace->p2p_wps_result_callback );
                }

                wps_internal_init( workspace->p2p_wps_agent, WWD_P2P_INTERFACE, (besl_wps_mode_t) workspace->p2p_wps_mode, workspace->p2p_wps_pin, &workspace->p2p_wps_credential, 1 );
                p2p_write_probe_response_ie( workspace ); // Re-add the p2p IE after the WPS IE
                workspace->p2p_wps_agent->device_password_id = workspace->p2p_wps_device_password_id;

                /* Run WPS state machine in its own thread */
                besl_p2p_wps_start( workspace->p2p_wps_agent );
                host_rtos_delay_milliseconds( 10 ); /* Delay required to allow WPS thread to run and initialize */
                break;

            case P2P_EVENT_DEVICE_AWAKE:
                if ( workspace->sent_go_discoverability_request == 1 )
                {
                    workspace->sent_go_discoverability_request = 0;
                    memcpy( &discovery_requestor.p2p_device_address, &workspace->discovery_requestor, sizeof( besl_mac_t ) );
                    discovery_requestor.dialog_token = workspace->discovery_dialog_token;
                    discovery_requestor.status = ( p2p_device_status_t ) 0; /* Success */
                    besl_p2p_send_action_frame( workspace, &discovery_requestor, p2p_write_device_discoverability_response, (uint32_t)workspace->operating_channel.channel, 0 );
                }
                break;

            case P2P_EVENT_P2P_DEVICE_ASSOCIATED:
                /* Rewrite the probe response to add P2P client descriptors */
                if ( ( workspace->p2p_wpa2_client_association_callback != NULL ) && ( message.data != NULL ) )
                {
                    workspace->p2p_wpa2_client_association_callback( message.data );
                }
                host_rtos_delay_milliseconds( 10 ); /* Delay required to allow WPS thread to finish its rewriting of the probe response ie */
                p2p_write_probe_response_ie( workspace );
                break;

            case P2P_EVENT_LEGACY_DEVICE_ASSOCIATED:
                if ( ( workspace->p2p_wpa2_client_association_callback != NULL ) && ( message.data != NULL ) )
                {
                    workspace->p2p_wpa2_client_association_callback( message.data );
                }
                break;

            case P2P_EVENT_WPS_ENROLLEE_ASSOCIATED:
                if ( ( workspace->p2p_wps_enrollee_association_callback != NULL ) && ( message.data != NULL ) )
                {
                    workspace->p2p_wps_enrollee_association_callback( message.data );
                }
                break;

            case P2P_EVENT_CONNECTION_REQUESTED:
                BESL_DEBUG( ("P2P connection or invitation request\r\n") );
                /* Inform the user interface that a device wants to connect */
                if ( ( workspace->p2p_connection_request_callback != NULL ) && ( message.data != NULL ) )
                {
                    BESL_DEBUG(("Calling connection handler\r\n"));
                    workspace->p2p_connection_request_callback( message.data );
                }
                if ( workspace->p2p_connection_request_callback == NULL )
                {
                    BESL_DEBUG(("Connection request handler null\r\n"));
                }
                if ( message.data == NULL )
                {
                    BESL_DEBUG(("Message data null\r\n"));
                }

                break;

            case P2P_EVENT_LEGACY_DEVICE_CONNECTION_REQUEST:
                if ( ( workspace->p2p_current_state == P2P_STATE_GROUP_OWNER ) && ( workspace->p2p_legacy_device_connection_request_callback != NULL ) && ( message.data != NULL ) )
                {
                    workspace->p2p_legacy_device_connection_request_callback( (p2p_legacy_device_t*)message.data );
                }
                break;

            case P2P_EVENT_DEVICE_DISASSOCIATED:
                if ( message.data != NULL )
                {
                    if ( workspace->p2p_current_state == P2P_STATE_GROUP_OWNER )
                    {
                        /* If the current enrollee is disassociating then reset the registrar state machine */
                        if ( workspace->p2p_wps_agent != NULL )
                        {
                            besl_wps_reset_registrar( workspace->p2p_wps_agent, (besl_mac_t*)message.data );
                        }
                        /* If it's an associated P2P device then clear the entry in the associated P2P device list and rewrite the P2P IE in probe responses */
                        associated_device = p2p_host_find_associated_p2p_device( workspace, message.data );
                        if ( associated_device != NULL )
                        {
                            memset( associated_device, 0, sizeof( p2p_client_info_descriptor_t ) );
                            if ( workspace->associated_p2p_device_count > 0 )
                            {
                                workspace->associated_p2p_device_count--;
                            }
                            p2p_write_probe_response_ie( workspace );
                            if ( workspace->p2p_device_disassociation_callback != NULL )
                            {
                                workspace->p2p_device_disassociation_callback( message.data );
                            }
                        }
                        else
                        {
                            associated_legacy_device = p2p_host_find_associated_legacy_device( workspace, message.data );
                            if ( associated_legacy_device != NULL )
                            {
                                memset( associated_legacy_device, 0, sizeof( besl_mac_t ) );
                                if ( workspace->p2p_legacy_device_disassociation_callback != NULL )
                                {
                                    workspace->p2p_legacy_device_disassociation_callback( message.data );
                                }
                            }
                        }
                    }
                    else if ( workspace->p2p_current_state == P2P_STATE_GROUP_CLIENT )
                    {
                        /* client callback is tbd */
                    }
                    /* If the device is in the p2p peer list clear the entry. */
                    peer_device = besl_p2p_host_find_device( workspace, message.data );
                    if ( peer_device != NULL )
                    {
                        peer_device->status = P2P_DEVICE_INVALID;
                    }
                }
                break;

            case P2P_EVENT_WPS_ENROLLEE_COMPLETED:
                if ( message.data != NULL )
                {
                    wps_result = (wps_result_t*)message.data;
                    if ( *wps_result != WPS_COMPLETE )
                    {
                        workspace->p2p_result = BESL_P2P_ERROR_FAIL;
                        p2p_stop( workspace );
                    }
                    else
                    {
                        BESL_INFO( ("WPS successful for P2P client\r\n") );
                        p2p_join_group_owner( workspace );
                    }
                }
                break;

            default:
                break;
        }
    }

    /* Remove P2P related information elements from management frames */
    p2p_remove_ies( workspace );

    /*  Remove P2P event handler */
    if ( workspace->group_owner_is_up == 1 )
    {
        result = wwd_management_set_event_handler( p2p_group_owner_events, NULL, workspace, WWD_P2P_INTERFACE );
        wiced_assert("set handler failed", result == WWD_SUCCESS);
        REFERENCE_DEBUG_ONLY_VARIABLE( result );
        p2p_group_owner_stop( workspace );
    }
    else if ( workspace->group_client_is_up == 1 )
    {
        result = wwd_management_set_event_handler( p2p_group_client_events, NULL, workspace, WWD_P2P_INTERFACE );
        wiced_assert("set handler failed", result == WWD_SUCCESS);
        REFERENCE_DEBUG_ONLY_VARIABLE( result );

        /* If the WPS handshake failed then we end up here */
        if ( workspace->p2p_result == BESL_P2P_ERROR_FAIL )
        {
            if ( workspace->p2p_group_formation_result_callback != NULL )
            {
                workspace->p2p_group_formation_result_callback( workspace );
            }
        }
        p2p_group_client_stop( workspace );
    }
    else
    {
        wiced_buffer_t buffer;
        uint32_t* data;

        /*  Disable discovery */
        wwd_wifi_abort_scan();

        p2p_set_discovery_state( P2P_DISCOVERY_STATE_SCAN, 0 );
        data = wwd_sdpcm_get_iovar_buffer( &buffer, 4, IOVAR_STR_P2P_DISC );
        if ( data == NULL )
        {
            wiced_assert("Allocation failed\n", 0 == 1);
            return;
        }
        *data = 0;
        result = wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE); /* This IOVAR must be sent on STA interface */
        if (result != WWD_SUCCESS)
        {
            BESL_DEBUG(("P2P: Unable to disable discovery\r\n"));
        }
        wiced_assert("", result == WWD_SUCCESS);
        if ( workspace->p2p_group_formation_result_callback != NULL )
        {
            workspace->p2p_group_formation_result_callback( workspace );
        }

        result = wwd_management_set_event_handler( p2p_discovery_events, NULL, workspace, WWD_STA_INTERFACE );
        wiced_assert("set handler failed", result == WWD_SUCCESS);
        REFERENCE_DEBUG_ONLY_VARIABLE( result );

        /* Turn on MPC */
        data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_MPC );
        if ( data != NULL )
        {
            *data = 1;
            if ( wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) != WWD_SUCCESS )
            {
                BESL_DEBUG(("P2P: Unable to enable MPC\r\n"));
            }
        }
    }
    wiced_assert("", result == WWD_SUCCESS);
    p2p_clear_event_queue();

    WICED_END_OF_CURRENT_THREAD( );
}

besl_result_t p2p_stop( p2p_workspace_t* workspace )
{
    p2p_message_t message;

    /* Stop timer and drain message queue so we don't block */
    p2p_stop_timer( workspace );
    p2p_clear_event_queue();

    message.type = P2P_EVENT_STOP_REQUESTED;
    message.data = NULL;
    host_rtos_push_to_queue( &p2p_message_queue, &message, WICED_NEVER_TIMEOUT );

    return BESL_SUCCESS;
}

static besl_result_t p2p_start_client( p2p_workspace_t* workspace )
{
    wwd_result_t             result;
    wiced_buffer_t           buffer;
    wiced_buffer_t           response;
    uint32_t*                data;
    uint32_t                 bss_index = 0;

    if ( workspace->reinvoking_group == 0 )
    {
        if ( workspace->p2p_wps_agent != NULL )
        {
            BESL_DEBUG(("wps agent not null\r\n"));
            besl_host_free( workspace->p2p_wps_agent );
            workspace->p2p_wps_agent = NULL;
        }
        workspace->p2p_wps_agent = besl_host_calloc( "p2p client wps agent", 1, sizeof(wps_agent_t) );
        if ( workspace->p2p_wps_agent == NULL )
        {
            workspace->p2p_current_state = P2P_STATE_FAILURE;
            workspace->p2p_result = BESL_ERROR_OUT_OF_MEMORY;
            return BESL_ERROR_OUT_OF_MEMORY;
        }
    }

    /* Clear the p2p event queue before bringing up the client */
    p2p_clear_event_queue();
    workspace->group_client_is_up = 1;

    /* Turn off STA interface */
    wiced_network_down( WICED_STA_INTERFACE );

    /* Create P2P interface */
    wl_p2p_if_t* p2p_if = wwd_sdpcm_get_iovar_buffer( &buffer, sizeof(wl_p2p_if_t), "p2p_ifadd" );
    CHECK_IOCTL_BUFFER( p2p_if );
    p2p_if->interface_type = P2P_CLIENT_MODE;
    p2p_if->chan_spec = workspace->group_candidate.operating_channel.channel | WL_CHANSPEC_BAND_2G | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE;
    memcpy( &p2p_if->mac_address, &workspace->p2p_interface_address, sizeof(besl_mac_t) );
    result = wwd_sdpcm_send_iovar( SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE );
    if ( result != WWD_SUCCESS )
    {
        BESL_DEBUG(("p2p_ifadd fail when creating client\r\n"));
    }

    /* Check that we can read the p2p interface. It comes up as a side effect of this call... */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) sizeof(besl_mac_t), "p2p_if" );
    CHECK_IOCTL_BUFFER( data );
    memcpy(data, &workspace->p2p_interface_address, sizeof(besl_mac_t));
    result = wwd_sdpcm_send_iovar( SDPCM_GET, buffer, &response, WWD_STA_INTERFACE );

    if ( result != WWD_SUCCESS )
    {
        BESL_DEBUG(("unable to read p2p interface\r\n"));
        return (besl_result_t) result;
    }

    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    wl_p2p_ifq_t* gc_if = (wl_p2p_ifq_t*)data;
    BESL_DEBUG(("p2p interface %u, %s\n", (unsigned int)gc_if->bsscfgidx, gc_if->ifname));
    bss_index = gc_if->bsscfgidx;
    wwd_update_host_interface_to_bss_index_mapping( WWD_P2P_INTERFACE, bss_index );
    host_buffer_release( response, WWD_NETWORK_RX );

    /* Get the P2P interface address */
    besl_host_get_mac_address(&workspace->device_info.p2p_device_address, WWD_P2P_INTERFACE);

    BESL_DEBUG(("STA MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", workspace->device_info.p2p_device_address.octet[0],
        workspace->device_info.p2p_device_address.octet[1],
        workspace->device_info.p2p_device_address.octet[2],
        workspace->device_info.p2p_device_address.octet[3],
        workspace->device_info.p2p_device_address.octet[4],
        workspace->device_info.p2p_device_address.octet[5]));

    wwd_wifi_set_block_ack_window_size( WWD_P2P_INTERFACE );

    if ( workspace->reinvoking_group == 0 )
    {
        /* Start WPS */
        workspace->p2p_wps_agent->device_password_id = workspace->p2p_wps_device_password_id;
        workspace->p2p_wps_agent->wps_mode = workspace->p2p_wps_mode;
        workspace->p2p_wps_agent->is_p2p_enrollee = 1;
        besl_wps_init( workspace->p2p_wps_agent, workspace->wps_device_details, WPS_ENROLLEE_AGENT, WWD_P2P_INTERFACE );
        BESL_DEBUG(("past wps init\r\n"));
        wps_internal_init(workspace->p2p_wps_agent, WWD_P2P_INTERFACE, (besl_wps_mode_t) workspace->p2p_wps_mode, workspace->p2p_wps_pin, &workspace->wps_credential, 1);
        BESL_DEBUG(("past wps internal init\r\n"));
        wps_register_result_callback( workspace->p2p_wps_agent, workspace->p2p_wps_result_callback );

        /*  Add the P2P IE to the probe request after WPS IE */
        p2p_write_probe_request_ie( workspace );

        /* Add the P2P IE to the association request after the WPS IE */
        p2p_write_association_request_ie( workspace );

        /*  Create the AP details */
        workspace->group_owner.scan_result.channel = workspace->group_candidate.operating_channel.channel;
        workspace->current_channel     = (uint32_t)workspace->group_candidate.operating_channel.channel;
        memcpy( &workspace->operating_channel, &workspace->group_candidate.operating_channel, sizeof( p2p_channel_info_t ) );
        BESL_DEBUG( ("GO operating channel %u\r\n", (unsigned int)workspace->group_candidate.operating_channel.channel) );
        memcpy(&workspace->group_owner.scan_result.BSSID, &workspace->group_candidate.bssid, sizeof(besl_mac_t));
        workspace->group_owner.scan_result.SSID.length = workspace->group_candidate.ssid_length;
        memcpy(workspace->group_owner.scan_result.SSID.value, workspace->group_candidate.ssid, workspace->group_candidate.ssid_length);
        workspace->group_owner.scan_result.security = WICED_SECURITY_WPS_SECURE;
        workspace->group_owner.scan_result.band = WICED_WIFI_CH_TO_BAND( workspace->group_candidate.operating_channel.channel );
        workspace->p2p_wps_agent->directed_wps_max_attempts = 0xFFFFFFFF;
        workspace->p2p_wps_agent->ap = &workspace->group_owner;

        /* Run the WPS state machine in its own thread */
        wps_register_internal_result_callback( workspace->p2p_wps_agent, p2p_wps_internal_result_callback );
        workspace->p2p_current_state = P2P_STATE_CONNECTION_WPS_ENROLLEE;
        besl_p2p_wps_start( workspace->p2p_wps_agent );
        host_rtos_delay_milliseconds( 10 ); // Delay required to allow WPS thread to run and initialize
    }
    else
    {
        /* Add the P2P IE to the association request */
        p2p_write_association_request_ie( workspace );

        workspace->group_owner.scan_result.channel = workspace->group_candidate.operating_channel.channel;
        workspace->current_channel = (uint32_t)workspace->group_candidate.operating_channel.channel;
        memcpy(&workspace->group_owner.scan_result.BSSID, &workspace->group_candidate.bssid, sizeof(besl_mac_t));
        workspace->group_owner.scan_result.SSID.length = workspace->group_candidate.ssid_length;
        memcpy(workspace->group_owner.scan_result.SSID.value, workspace->group_candidate.ssid, workspace->group_candidate.ssid_length);
        workspace->group_owner.scan_result.band = WICED_WIFI_CH_TO_BAND( workspace->current_channel );
        result = (wwd_result_t) p2p_join_group_owner( workspace );
    }
    return (besl_result_t) result;
}

static besl_result_t p2p_join_group_owner( p2p_workspace_t* workspace )
{
    wwd_result_t result;
#ifdef P2P_IP_ALLOCATION
    wiced_ip_setting_t device_init_ip_settings;
#endif

    if ( workspace->reinvoking_group == 0 )
    {
        workspace->p2p_passphrase_length = workspace->wps_credential.passphrase_length;
        memcpy( workspace->p2p_passphrase, workspace->wps_credential.passphrase, workspace->p2p_passphrase_length );
    }

    /* Rewrite the IEs that go into the probe request */
    workspace->p2p_wps_device_password_id = WPS_DEFAULT_DEVICEPWDID;
    p2p_write_wps_probe_request_ie( workspace);
    p2p_write_probe_request_ie( workspace );

    /* Try to join the AP with the credentials we've just received */
    workspace->group_owner.scan_result.security = WICED_SECURITY_WPA2_AES_PSK;
    result = WWD_PENDING;
    BESL_INFO( ("About to join group owner using join specific\r\n") );

    result = wwd_wifi_join_specific( &workspace->group_owner.scan_result, workspace->p2p_passphrase, workspace->p2p_passphrase_length, NULL, WWD_P2P_INTERFACE );

    if ( result != WWD_SUCCESS)
    {
        /* If join-specific failed, try scan and join AP */
        BESL_INFO( ("Full channel scan\n") );
        result = wwd_wifi_join( &workspace->group_owner.scan_result.SSID, workspace->group_owner.scan_result.security, (uint8_t*) workspace->p2p_passphrase, workspace->p2p_passphrase_length, NULL, WWD_P2P_INTERFACE );
    }

    if ( result == WWD_SUCCESS )
    {
        wiced_result_t ip_up_result = WICED_ERROR;
        BESL_INFO( ("P2P group formation complete as client device. Group ID %s %02X:%02X:%02X:%02X:%02X:%02X\n", workspace->group_candidate.ssid,
            workspace->group_candidate.bssid.octet[0],
            workspace->group_candidate.bssid.octet[1],
            workspace->group_candidate.bssid.octet[2],
            workspace->group_candidate.bssid.octet[3],
            workspace->group_candidate.bssid.octet[4],
            workspace->group_candidate.bssid.octet[5] ) );

        besl_host_set_mac_address(&workspace->p2p_interface_address, WWD_STA_INTERFACE ); // If this isn't done we don't respond to provision discovery

#ifdef P2P_IP_ALLOCATION
        /* Get ip retrieved from eapol frame */
        ip_up_result = p2p_get_ip_addr(&device_init_ip_settings);

        if (ip_up_result == WICED_SUCCESS)
        {
            ip_up_result = wiced_ip_up( ( wiced_interface_t ) WWD_P2P_INTERFACE, WICED_USE_STATIC_IP, &device_init_ip_settings );
        }
#endif
        if ( ip_up_result != WICED_SUCCESS &&
                wiced_ip_up( ( wiced_interface_t ) WWD_P2P_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL) != WICED_SUCCESS )

        {
            BESL_DEBUG( ("IP failed to get an address\r\n") );
            workspace->p2p_result = BESL_P2P_ERROR_FAIL;
            p2p_stop( workspace );
        }
        else
        {
            host_rtos_delay_milliseconds( 10 ); // Delay to allow the DHCP thread to run
            workspace->p2p_current_state = P2P_STATE_GROUP_CLIENT;

            /*  Add P2P event handler */
            result = wwd_management_set_event_handler( p2p_group_client_events, p2p_event_handler, workspace, WWD_P2P_INTERFACE );
            if ( result != WWD_SUCCESS )
            {
                BESL_DEBUG(("Unable to set group client event handler\r\n"));
            }
            workspace->p2p_result = BESL_SUCCESS;
        }
    }
    else
    {
        BESL_DEBUG( ("WPA2 handshake failed when joining GO\r\n") );
        workspace->p2p_result = BESL_P2P_ERROR_FAIL;
        p2p_stop( workspace );
    }

    if ( workspace->p2p_group_formation_result_callback != NULL )
    {
        workspace->p2p_group_formation_result_callback( workspace );
    }

    return workspace->p2p_result;
}

#ifdef P2P_IP_ALLOCATION
static besl_result_t p2p_get_ip_addr( wiced_ip_setting_t* ip_setting )
{
    uint32_t*      data;
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wwd_result_t   result;

    /* Get Ip addr */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, P2P_IP_ALLOC_LEN, IOVAR_STR_P2P_IP_ADDR );
    CHECK_IOCTL_BUFFER( data );
    result = wwd_sdpcm_send_iovar( SDPCM_GET, buffer, &response, WWD_P2P_INTERFACE );

    if ( result != WWD_SUCCESS )
    {
        BESL_DEBUG(("unable to read p2p interface\r\n"));
        return (besl_result_t) result;
    }

    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );

    if (data) {
        SET_IPV4_ADDRESS( ip_setting->ip_address, ntohl(*(data)) );
        SET_IPV4_ADDRESS( ip_setting->netmask, ntohl(*(data+1)));
        SET_IPV4_ADDRESS( ip_setting->gateway, ntohl(*(data+2)) );

        result = BESL_SUCCESS;
    }
    else
        result = WICED_ERROR;

    host_buffer_release( response, WWD_NETWORK_RX );

    return result;
}
#endif
static void p2p_discover( p2p_workspace_t* workspace )
{
    besl_result_t result;

    result = p2p_set_discovery_state( P2P_DISCOVERY_STATE_LISTEN, workspace->listen_channel.channel );
    wiced_assert("", result == BESL_SUCCESS);
    REFERENCE_DEBUG_ONLY_VARIABLE(result);
}

besl_result_t p2p_host_send_message( p2p_message_t* message, uint32_t timeout_ms )
{
    return (besl_result_t) host_rtos_push_to_queue( &p2p_message_queue, message, timeout_ms );
}

static besl_result_t p2p_scan( p2p_workspace_t* workspace, uint16_t scan_action, wiced_bool_t scan_all_channels )
{
    wiced_buffer_t  buffer;
    wl_p2p_escan_t* p2p_scan;
    besl_mac_t      bcast = {{255, 255, 255, 255, 255, 255}};
    uint32_t        number_of_channels = 3; /* Default number of channels to cover the three social channels (1, 6, 11) */
    uint16_t        social_channels[3] = { 1, 6, 11 };
    uint16_t        all_channels[11]   = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
    uint16_t*       channel_list_ptr   = social_channels;
    uint32_t        bss_index          = wwd_get_bss_index( WWD_P2P_INTERFACE );
    uint32_t*       data;
    uint32_t        data_size;

    if ( workspace->scan_in_progress == 1 )
    {
        BESL_DEBUG(("p2p_scan: scan already in progress\r\n"));
    }
    workspace->scan_in_progress = 1;

    /*  Begin p2p scan of the "escan" variety */
    if ( scan_all_channels == WICED_TRUE ) /* All channels means 2.4 GHz 1-11 for now */
    {
        number_of_channels = 11;
        channel_list_ptr   = all_channels;
    }
    data_size = 4 + sizeof(wl_p2p_escan_t) + ( sizeof( uint16_t ) * number_of_channels );
    data = wwd_sdpcm_get_iovar_buffer( &buffer, data_size, "bsscfg:p2p_scan" );
    CHECK_IOCTL_BUFFER( data );
    memset( data, 0, data_size );
    *data = bss_index;
    p2p_scan = (wl_p2p_escan_t*)&data[1];

    /* Fill in the appropriate details of the scan parameters structure */
    p2p_scan->type                      = 'E';
    besl_host_random_bytes((uint8_t*)&p2p_scan->escan.sync_id, sizeof(p2p_scan->escan.sync_id));
    p2p_scan->escan.version             = htod32(ESCAN_REQ_VERSION);
    p2p_scan->escan.action              = htod16(WL_SCAN_ACTION_START);
    p2p_scan->escan.params.scan_type    = (int8_t) WICED_SCAN_TYPE_ACTIVE;
    memcpy(&p2p_scan->escan.params.bssid, &bcast, sizeof(besl_mac_t));
    p2p_scan->escan.params.bss_type     = (int8_t) WICED_BSS_TYPE_ANY;
    p2p_scan->escan.params.nprobes      = htod32(2);
    p2p_scan->escan.params.active_time  = htod32(40);
    p2p_scan->escan.params.passive_time = (int32_t) -1;
    p2p_scan->escan.params.home_time    = htod32(60);

    p2p_scan->escan.params.channel_num  = number_of_channels;
    memcpy( p2p_scan->escan.params.channel_list, channel_list_ptr, sizeof( uint16_t ) * number_of_channels );

    p2p_scan->escan.params.ssid.SSID_len = sizeof( P2P_WILDCARD_SSID ) - 1;
    memcpy( p2p_scan->escan.params.ssid.SSID, P2P_WILDCARD_SSID, sizeof( P2P_WILDCARD_SSID ) - 1 );

    if ( wwd_sdpcm_send_iovar( SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE ) != WWD_SUCCESS )
    {
        BESL_ERROR(("Failed to start P2P scan\n"));
        return BESL_P2P_ERROR_FAIL;
    }

    return BESL_SUCCESS;
}


static besl_result_t p2p_set_discovery_state( p2p_discovery_state_t state, uint32_t channel )
{
    uint32_t*      data;
    wiced_buffer_t buffer;
    wl_p2p_disc_st_t discovery_mode;

    discovery_mode.state = state;
    if (state == P2P_DISCOVERY_STATE_LISTEN)
    {
        uint16_t listen_ms;
        besl_host_random_bytes( (uint8_t*)&listen_ms, 2 );
        listen_ms = ( 1 + (listen_ms % P2P_MAX_DISCOVERABLE_INTERVAL ) ) * P2P_BEACON_INTERVAL_MS;

        discovery_mode.chanspec      = channel | WL_CHANSPEC_BAND_2G | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE;
        discovery_mode.dwell_time_ms = 500;//listen_ms;
    }
    else
    {
        discovery_mode.chanspec      = channel | WL_CHANSPEC_BAND_2G | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE;
        discovery_mode.dwell_time_ms = 0;
    }

    data = wwd_sdpcm_get_iovar_buffer(&buffer, sizeof(wl_p2p_disc_st_t), "p2p_state");
    CHECK_IOCTL_BUFFER( data );
    memcpy( data, &discovery_mode, sizeof(wl_p2p_disc_st_t) );
    return (besl_result_t) wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE);
}

static besl_result_t p2p_group_owner_stop( p2p_workspace_t* workspace )
{
    uint32_t*      data;
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    wwd_result_t   result;

    wiced_network_down( WICED_P2P_INTERFACE );
    host_rtos_delay_milliseconds( 10 );

    /* Restore original MAC address */
    besl_host_set_mac_address(&workspace->original_mac_address, WICED_STA_INTERFACE );

    /* Query bss state (does it exist? if so is UP?) */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_BSS );
    CHECK_IOCTL_BUFFER( data );
    *data = wwd_get_bss_index( WWD_P2P_INTERFACE );
    result = wwd_sdpcm_send_iovar( SDPCM_GET, buffer, &response, WWD_STA_INTERFACE );
    if ( ( result != WWD_SUCCESS ) &&
         ( result != WWD_WLAN_NOTFOUND ) )
    {
        BESL_ERROR( ("p2p interface not found\n") );
        return (besl_result_t) result;
    }

    if ( result == WWD_WLAN_NOTFOUND )
    {
        /* P2P interface does not exist - i.e. it is down */
        return BESL_SUCCESS;
    }
    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    if ( data[0] != (uint32_t) 1 )
    {
        /* P2P interface indicates it is not up - i.e. it is down */
        host_buffer_release( response, WWD_NETWORK_RX );
        BESL_DEBUG( ("p2p interface is not up\n") );
        return BESL_SUCCESS;
    }

    host_buffer_release( response, WWD_NETWORK_RX );

    /* Turn on MPC or the radio chip keeps beaconing */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_MPC );
    CHECK_IOCTL_BUFFER( data );
    *data = 1;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) );

    data = wwd_sdpcm_get_iovar_buffer(&buffer, sizeof(wiced_mac_t), "p2p_ifdel" );
    CHECK_IOCTL_BUFFER( data );
    memcpy( data, &workspace->p2p_interface_address, sizeof(wiced_mac_t) );
    result = wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE);
    wiced_assert("", result == WWD_SUCCESS);

    if ( result != WWD_SUCCESS )
    {
        BESL_DEBUG( ("Couldn't delete p2p i/f\r\n") );
    }

    return BESL_SUCCESS;

}


static besl_result_t p2p_group_client_stop( p2p_workspace_t* workspace )
{
    uint32_t*      data;
    wiced_buffer_t buffer;
    wwd_result_t   result;

    wiced_network_down( WICED_P2P_INTERFACE );
    host_rtos_delay_milliseconds( 10 );

    /* Restore original MAC address */
    besl_host_set_mac_address(&workspace->original_mac_address, WICED_STA_INTERFACE );

    /* Turn on MPC */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_MPC );
    CHECK_IOCTL_BUFFER( data );
    *data = 1;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) );

    /* Delete the P2P interface */
    data = wwd_sdpcm_get_iovar_buffer(&buffer, sizeof(wiced_mac_t), "p2p_ifdel" );
    CHECK_IOCTL_BUFFER( data );
    memcpy( data, &workspace->p2p_interface_address, sizeof(wiced_mac_t) );
    result = wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE);
    wiced_assert("", result == WWD_SUCCESS);

    if ( result != WWD_SUCCESS )
    {
        BESL_DEBUG( ("Failed to delete p2p i/f\r\n") );
    }
    else
    {
        BESL_DEBUG( ("p2p_group_client_stop: deleted p2p i/f\r\n") );
    }

    return BESL_SUCCESS;
}


static void p2p_start_timer( void* workspace, uint32_t timeout )
{
    p2p_workspace_t* host = (p2p_workspace_t*)workspace;
    host->timer_reference = host_rtos_get_time( );
    host->timer_timeout   = timeout;
}

void p2p_stop_timer( void* workspace )
{
    p2p_workspace_t* host = (p2p_workspace_t*)workspace;
    host->timer_timeout = 0;
}

static void p2p_abort_scan( p2p_workspace_t* workspace )
{
    if ( workspace->scan_in_progress == 0 )
    {
        BESL_DEBUG(("p2p_abort_scan: scan not in progress\r\n"));
    }
    else
    {
        wwd_wifi_abort_scan();
        workspace->scan_aborted = 1;
    }
}

void p2p_wps_internal_result_callback( wps_result_t* result )
{
    p2p_message_t message;
    wps_result_t  *wps_result = result;

    message.type = P2P_EVENT_WPS_ENROLLEE_COMPLETED;
    message.data = wps_result;
    host_rtos_push_to_queue(&p2p_message_queue, &message, WICED_NEVER_TIMEOUT);
}

besl_result_t p2p_deinit( p2p_workspace_t* workspace )
{
    wiced_network_down( WICED_P2P_INTERFACE );
    if ( workspace->p2p_thread_running == 1 )
    {
        BESL_DEBUG(("besl_p2p_deinit: stopping p2p thread\r\n"));
        /* Stop and delete the P2P thread */
        workspace->p2p_thread_running = 0;
        p2p_stop(workspace);
        host_rtos_delay_milliseconds( 10 ); // Delay to allow the P2P thread to complete

        if ( host_rtos_join_thread(&p2p_thread) != WWD_SUCCESS )
        {
            BESL_DEBUG(("besl_p2p_deinit: failed to join thread\r\n"));
        }
        if ( host_rtos_delete_terminated_thread(&p2p_thread) != WWD_SUCCESS )
        {
            BESL_DEBUG(("besl_p2p_deinit: failed to delete thread\r\n"));
        }
    }

    /* Delete the message queue and clean up */
    host_rtos_deinit_queue( &p2p_message_queue );
    memset(p2p_message_queue_buffer, 0, sizeof (p2p_message_t) * MAX_NUMBER_OF_P2P_MESSAGES );
    memset(&p2p_thread, 0, sizeof(p2p_thread));
    memset(workspace, 0, sizeof(p2p_workspace_t));
    host_rtos_delay_milliseconds( 10 ); // Delay to allow the WPS and P2P threads to complete

    /* Restore mapping for AP and P2P interfaces to default in case we were using index 1 for P2P */
    wwd_update_host_interface_to_bss_index_mapping( WWD_AP_INTERFACE, WWD_AP_INTERFACE );
    wwd_update_host_interface_to_bss_index_mapping( WWD_P2P_INTERFACE, WWD_P2P_INTERFACE );

    return BESL_SUCCESS;
}

void p2p_clear_event_queue( void )
{
    p2p_message_t message;

    /* Clear the p2p event queue */
    while ( host_rtos_pop_from_queue( &p2p_message_queue, &message, 0 ) == WWD_SUCCESS )
    {
        ;
    }
}

besl_result_t p2p_set_ops( wl_p2p_ops_t* ops )
{
    uint32_t*      data;
    wiced_buffer_t buffer;

    if ( (ops->ops != 0) && ( (ops->ctw < 10) || (ops->ctw > P2P_BEACON_INTERVAL_MS) ) )
    {
        BESL_DEBUG(( "Invalid CTWindow value\n" ));
        BESL_DEBUG(( "CTWindow must be at least 10 TU and less than a beacon interval (100ms)\n" ));
        return ( -1 );
    }

    data = wwd_sdpcm_get_iovar_buffer(&buffer, sizeof(wl_p2p_ops_t), "p2p_ops");
    CHECK_IOCTL_BUFFER( data );
    memcpy( data, ops, sizeof(wl_p2p_ops_t) );
    return (besl_result_t) wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_P2P_INTERFACE);
}

besl_result_t p2p_set_noa( wl_p2p_sched_t* noa )
{
    uint32_t*      data = NULL;
    wiced_buffer_t buffer;

    if ( (noa->action != WL_P2P_SCHED_ACTION_RESET) &&
           ( (noa->desc[0].interval == 0) || (noa->desc[0].duration == 0) || (noa->desc[0].interval < noa->desc[0].duration) ) )
    {
        BESL_DEBUG(( "Invalid Duration or interval combination\n" ));
        return ( -1 );
    }

    if (noa->action == WL_P2P_SCHED_ACTION_RESET) {
        /* Send only FIXED LENGTH NOA packet for P2P NoA reset */
        data = wwd_sdpcm_get_iovar_buffer(&buffer,( sizeof(wl_p2p_sched_t) - sizeof(wl_p2p_sched_desc_t)), "p2p_noa");
        memcpy( data, noa, ( sizeof(wl_p2p_sched_t) - sizeof(wl_p2p_sched_desc_t)) );
    }
    else
    {
        data = wwd_sdpcm_get_iovar_buffer(&buffer, sizeof(wl_p2p_sched_t), "p2p_noa");
        memcpy( data, noa, sizeof(wl_p2p_sched_t) );
    }
    CHECK_IOCTL_BUFFER( data );

    return (besl_result_t) wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_P2P_INTERFACE);
}
