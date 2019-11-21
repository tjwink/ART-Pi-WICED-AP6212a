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

#include "besl_structures.h"
#include "wps_common.h"
#include "p2p_host_interface.h"
#include "p2p_structures.h"
#include "p2p_frame_writer.h"
#include "besl_host.h"
#include "wiced_p2p.h"
#include "wwd_events.h"
#include "wwd_wifi.h"
#include "p2p_constants.h"
#include "wiced_utilities.h"
#include <string.h>

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

static void                          p2p_process_disassociation( p2p_workspace_t* workspace, const uint8_t* data, const wwd_event_header_t* event_header );
static besl_result_t                 p2p_init_as_group_owner(p2p_workspace_t* workspace);
static p2p_client_info_descriptor_t* p2p_host_find_empty_associated_p2p_device_entry( p2p_workspace_t* workspace, besl_mac_t* p2p_device_address );
static p2p_legacy_device_t*          p2p_find_legacy_device( p2p_workspace_t* workspace, besl_mac_t* mac );
static besl_result_t                 p2p_defragment_wps_and_p2p_elements( p2p_workspace_t* workspace, const uint8_t* data_start, uint32_t length );
static besl_mac_t*                   p2p_host_find_empty_associated_legacy_device_entry( p2p_workspace_t* workspace, const besl_mac_t* p2p_device_address );
static besl_result_t                 p2p_process_scan_result( p2p_workspace_t* workspace, wl_escan_result_t* scan_result );
static p2p_tlv_data_t*               p2p_find_tlv(const uint8_t* message, uint16_t message_length, uint16_t type);
static besl_result_t                 p2p_host_notify_negotiation_request( p2p_workspace_t* workspace, p2p_discovered_device_t* device );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static p2p_discovered_device_t connecting_device; /* Used for message passing */

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_bool_t p2p_find_common_channel( p2p_channel_list_t* work_channel_list, p2p_channel_list_t* dev_channel_list, p2p_channel_list_t* comm_channel_list )
{
    int i, j;

    for( i = 0; i < work_channel_list->number_of_channels; i++ )
    {
        for( j = 0; j < dev_channel_list->number_of_channels; j++ )
        {
            if( work_channel_list->channel_list[i] == dev_channel_list->channel_list[j] )
            {
                comm_channel_list->channel_list[comm_channel_list->number_of_channels] = dev_channel_list->channel_list[j];
                comm_channel_list->number_of_channels++;
                break;
            }
        }
    }

    if( comm_channel_list->number_of_channels != 0 )
    {
        comm_channel_list->operating_class = work_channel_list->operating_class;
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

wiced_bool_t p2p_find_common_channel_list( p2p_workspace_t* workspace, p2p_discovered_device_t* device, p2p_tlv_data_t* p2p_tlv )
{
    int idx = 0, class_idx = 0, dev_class_idx = 0;
    size_t country_size;
    p2p_channel_list_t*      p2p_channel_list;
    p2p_channel_list_t*      work_channel_list;
    p2p_channel_list_t*      comm_channel_list;
    p2p_channel_list_info_t* comm_channel_list_info;
    p2p_channel_list_tlv_t*  p2p_channel_list_tlv;

    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_CHANNEL_LIST );
    if ( p2p_tlv == NULL )
    {
        return WICED_FALSE;
    }

    p2p_channel_list     = (p2p_channel_list_t*)&device->channel_list.p2p_channel_list;
    p2p_channel_list_tlv = &device->channel_list.p2p_channel_list_tlv;
    p2p_channel_list_tlv->length =
            ( ( p2p_tlv->length > sizeof( p2p_channel_list_tlv_data_t ) ) ? sizeof( p2p_channel_list_tlv_data_t ) : p2p_tlv->length );

    if ( p2p_channel_list_tlv->length == 0 )
    {
        BESL_INFO( ( "Can not find device channel list element \n" ) );
        return WICED_FALSE;
    }

    country_size = sizeof(workspace->channel_list.country_string);
    memcpy( &device->channel_list.country_string, p2p_tlv->data, country_size );
    memcpy( &p2p_channel_list_tlv->p2p_chan_list, p2p_tlv->data, p2p_channel_list_tlv->length );

    idx += country_size;
    while ( class_idx < P2P_MAX_OP_CLASSES && idx < p2p_channel_list_tlv->length )
    {
        /* For make sure the integrity of "Operating Class" set,
         * if the amount of "idx" + "current Operating Class set" exceeds p2p_channel_list_tlv->length,
         * then "current Operating Class set" will be ignored.
         * */
        BESL_INFO((" operating_class=%d, number_of_channels=%d\n", p2p_tlv->data[idx], p2p_tlv->data[idx+1]));
        if ( ( ( offsetof(p2p_channel_list_t, channel_list) +
                p2p_tlv->data[idx+1] ) + idx ) > p2p_channel_list_tlv->length )
        {
            break;
        }

        /* Copying each set of "Operating Class" of P2P_SEID_CHANNEL_LIST into p2p_channel_list.
         *
         * Operating Class :
         * {
         *   operating_class            //1 byte
         *   number_of_channels         //1 byte
         *   channel_list               //1~15 bytes
         * }
         * */
        p2p_channel_list[class_idx].operating_class = p2p_tlv->data[idx++];
        p2p_channel_list[class_idx].number_of_channels = p2p_tlv->data[idx++];
        memcpy( &p2p_channel_list[class_idx].channel_list, &p2p_tlv->data[idx], p2p_channel_list[class_idx].number_of_channels );
        idx += p2p_channel_list[class_idx].number_of_channels;

        p2p_channel_list_tlv->total_classes++;
        p2p_channel_list_tlv->total_channels += p2p_channel_list[class_idx].number_of_channels;
        class_idx++;
    }

    memset( &workspace->common_channel_list, 0, sizeof(workspace->common_channel_list) );
    comm_channel_list_info = &workspace->common_channel_list;
    comm_channel_list      = &comm_channel_list_info->p2p_channel_list[0];

    for ( class_idx = 0; class_idx < workspace->channel_list.p2p_channel_list_tlv.total_classes; class_idx++ )
    {
        work_channel_list = &workspace->channel_list.p2p_channel_list[class_idx];
        for ( dev_class_idx = 0; dev_class_idx < p2p_channel_list_tlv->total_classes; dev_class_idx++ )
        {
            p2p_channel_list = &device->channel_list.p2p_channel_list[dev_class_idx];
            if ( work_channel_list->operating_class == p2p_channel_list->operating_class )
            {
                if ( p2p_find_common_channel( work_channel_list, p2p_channel_list, comm_channel_list ) == WICED_TRUE )
                {
                    comm_channel_list_info->p2p_channel_list_tlv.total_classes++;
                    comm_channel_list_info->p2p_channel_list_tlv.total_channels += comm_channel_list->number_of_channels;
                    comm_channel_list++;
                }
                break;
            }
        }
    }

    if ( comm_channel_list_info->p2p_channel_list_tlv.total_channels != 0 )
    {
        p2p_channel_list_tlv_t*      comm_channel_list_tlv;
        p2p_channel_list_tlv_data_t* comm_channel_list_tlv_data;

        comm_channel_list_tlv         = &comm_channel_list_info->p2p_channel_list_tlv;
        comm_channel_list_tlv_data    = &comm_channel_list_info->p2p_channel_list_tlv.p2p_chan_list;
        comm_channel_list_tlv->length = country_size + comm_channel_list_tlv->total_classes * 2 + comm_channel_list_tlv->total_channels;
        memcpy( comm_channel_list_tlv_data->country_string, workspace->channel_list.country_string, country_size );

        for ( class_idx = 0, idx = 0; class_idx < comm_channel_list_tlv->total_classes; class_idx++ )
        {
            comm_channel_list_tlv_data->p2p_chan_list_tlv[idx++] = comm_channel_list_info->p2p_channel_list[class_idx].operating_class;
            comm_channel_list_tlv_data->p2p_chan_list_tlv[idx++] = comm_channel_list_info->p2p_channel_list[class_idx].number_of_channels;
            memcpy( &comm_channel_list_tlv_data->p2p_chan_list_tlv[idx], &comm_channel_list_info->p2p_channel_list[class_idx].channel_list, comm_channel_list_info->p2p_channel_list[class_idx].number_of_channels );
            idx += comm_channel_list_info->p2p_channel_list[class_idx].number_of_channels;
        }
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/* The event handler runs from the Wiced thread so no printing is allowed and it is better to post a message to the relevant thread handler to avoid length processing */
void* p2p_event_handler( const wwd_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data )
{
    p2p_workspace_t* workspace;
    p2p_message_t    message;
    //wl_af_params_t* action_frame = NULL;
    //uint32_t* packet_id = NULL;

    if ( ( event_header == NULL ) || ( event_data == NULL ) || ( handler_user_data == NULL ) )
    {
        return handler_user_data;
    }
    workspace = handler_user_data;

    switch ( event_header->event_type )
    {
        case WLC_E_ESCAN_RESULT:
            if ( workspace->p2p_current_state == P2P_STATE_DISCOVERY )
            {
                if ( event_header->status == WLC_E_STATUS_PARTIAL )
                {
                    p2p_process_scan_result( workspace, (wl_escan_result_t*)event_data );
                }
                if ( event_header->status == WLC_E_STATUS_SUCCESS )
                {
                    /*  Scan complete */
                    workspace->scan_in_progress = 0;
                    if ( workspace->scan_aborted == 1 )
                    {
                        workspace->scan_aborted = 0;
                    }
                    else
                    {
                        message.type = P2P_EVENT_SCAN_COMPLETE;
                        message.data = NULL;
                        p2p_host_send_message( &message, WICED_NO_WAIT );
                        if ( workspace->group_owner_is_up == 0 )
                        {
                            p2p_update_devices( workspace ); /* In discovery mode we update the discovered devices array here to avoid possible race conditions */
                        }
                    }
                }
            }
            else
            {
                if ( event_header->status == WLC_E_STATUS_SUCCESS )
                {
                    /*  Scan complete */
                    if ( workspace->scan_aborted == 1 )
                    {
                        workspace->scan_aborted = 0;
                    }
                }
            }
            break;

        case WLC_E_P2P_DISC_LISTEN_COMPLETE:
            if (workspace->p2p_listen_only == WICED_TRUE) {
                message.type = P2P_EVENT_START_LISTEN_DISCOVERY;
            } else {
                message.type = P2P_EVENT_DISCOVERY_COMPLETE;
            }
            message.data = NULL;
            workspace->p2p_listen_only = WICED_FALSE;
            p2p_host_send_message( &message, WICED_NO_WAIT );
            break;

        case WLC_E_PROBREQ_MSG:
            if ( ( workspace->p2p_current_state == P2P_STATE_DISCOVERY ) || ( workspace->group_owner_is_up == 1 ) )
            {
                p2p_process_probe_request( workspace, event_data, event_header->datalen );
            }
            break;

        case WLC_E_ACTION_FRAME:
        {
            p2p_process_action_frame( workspace, event_data, event_header );
            break;
        }

        case WLC_E_ACTION_FRAME_COMPLETE:
        {
            if ( event_header->status == 0 ) // Action frame received ACK
            {
                if ( workspace->sent_negotiation_confirm == 1 )
                {
                    p2p_stop_timer( workspace );
                    workspace->sent_negotiation_confirm = 0;
                    besl_p2p_host_negotiation_complete( workspace );
                }
                else if ( workspace->sent_go_discoverability_request == 1 )
                {
                    /* Note that for discoverability we don't zero the sent_go_discoverability_request here */
                    p2p_stop_timer( workspace );
                    message.type = P2P_EVENT_DEVICE_AWAKE;
                    message.data = NULL;
                    p2p_host_send_message( &message, WICED_NO_WAIT );
                }
            }
            break;
        }

        case WLC_E_ASSOC_IND:
        case WLC_E_REASSOC_IND:
        {
            p2p_process_association_request( workspace, event_data, event_header );
            break;
        }

        case WLC_E_DEAUTH:
        case WLC_E_DISASSOC:
        case WLC_E_DEAUTH_IND:
        case WLC_E_DISASSOC_IND:
        {
            if ( workspace->p2p_current_state == P2P_STATE_GROUP_OWNER )
            {
                p2p_process_disassociation( workspace, event_data, event_header );
            }
            else if ( workspace->p2p_current_state == P2P_STATE_GROUP_CLIENT )
            {
                ;//wiced_network_down(WWD_P2P_INTERFACE);
            }
            break;
        }

        default:
            break;
    }

    return handler_user_data;
}


// XXX restructure this into multiple functions
// XXX we can get action frames from associated devices as well as discovered devices
// XXX this function is called by the event handler which is running in the context of the Wiced thread so printing won't work at all unless the stack size is increased
// XXX Note that action frame dwell times will cause the next action frame not to be sent if the dwell time is not cancelled and the action frame is sent before the
// XXX dwell time expires.
void p2p_process_action_frame( p2p_workspace_t* workspace, const uint8_t* data, const wwd_event_header_t* event_header )
{
    besl_result_t                      result;
    p2p_tlv_data_t*                    p2p_tlv;
    p2p_public_action_frame_message_t* message              = (p2p_public_action_frame_message_t*)data;
    p2p_action_frame_message_t*        action_frame_message = (p2p_action_frame_message_t*)data;
    uint32_t                           channel              = 0;
    p2p_client_info_descriptor_t*      discovery_target     = NULL;
    p2p_message_t                      p2p_message;
    p2p_device_info_t*                 p2p_device_info      = NULL;
    tlv16_data_t*                      tlv16                = NULL;
    p2p_discovered_device_t*           device;
    p2p_discovered_device_t            temp_device;
    uint32_t                           data_length          = event_header->datalen;
    tlv16_data_t*                      config_method        = NULL;

    /* Check if it's a public action frame */
    if ( message->fixed.category_code == P2P_PUBLIC_ACTION_FRAME_CATEGORY_CODE )
    {
        if ( ( message->fixed.public_action == P2P_PUBLIC_ACTION_FRAME_TYPE ) &&
             ( message->fixed.oui_sub_type  == P2P_OUI_SUB_TYPE ) &&
             ( memcmp( &message->fixed.oui, P2P_OUI, 3 ) == 0 ) )
        {
            data_length -= sizeof(p2p_public_action_frame_fixed_parameter_t);
            if ( p2p_defragment_wps_and_p2p_elements( workspace, message->data, data_length ) != BESL_SUCCESS )
            {
                BESL_DEBUG( ( "p2p_process_action_frame: defragmentation failed\r\n" ) );
                return;
            }

            device = besl_p2p_host_find_device(workspace, (besl_mac_t*)&event_header->addr);
            if (device != NULL)
            {
                if (device->status == P2P_DEVICE_INVALID)
                {
                    memcpy( &device->p2p_device_address, &event_header->addr, sizeof(besl_mac_t) );
                    p2p_process_new_device_data( workspace, device );
                }
            }
            /* The space in the discovered devices array is limited so we proceed with a temporary variable */
            else
            {
                device = &temp_device;
                p2p_process_new_device_data( workspace, device );
                --workspace->discovered_device_count;
            }

            switch (message->fixed.p2p_sub_type)
            {
                case P2P_GO_NEGOTIATION_REQUEST:
                    if ( ( workspace->p2p_current_state != P2P_STATE_GROUP_OWNER ) && ( workspace->p2p_current_state != P2P_STATE_GROUP_CLIENT ) )
                    {
                        /*  Look for the group owner intent */
                        p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_GROUP_OWNER_INTENT );
                        if (p2p_tlv == NULL)
                        {
                            return;
                        }
                        device->tie_breaker = p2p_tlv->data[0] & 0x01;
                        device->group_owner_intent = (p2p_tlv->data[0] >> 1);

                        /* Determine group owner */
                        if ( device->group_owner_intent == 15 && workspace->group_owner_intent == 15 )
                        {
                            device->status = P2P_DEVICE_MUST_ALSO_BE_GROUP_OWNER;
                        }
                        else if ( ( device->group_owner_intent < workspace->group_owner_intent ) ||
                                  ( ( device->group_owner_intent == workspace->group_owner_intent ) &&  ( device->tie_breaker == 0 ) ) )
                        {
                            p2p_init_as_group_owner(workspace); // XXX review this
                        }
                        else
                        {
                            workspace->i_am_group_owner = 0;

                            /* Look for operating channel */
                            p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_OP_CHANNEL );
                            if (p2p_tlv == NULL)
                            {
                                return;
                            }
                            memcpy( &workspace->group_candidate.operating_channel, p2p_tlv->data, sizeof(p2p_channel_info_t) );

                            /* Look for intended interface */
                            p2p_tlv = p2p_find_tlv(workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_INTENDED_INTERFACE_ADDRESS);
                            if ( p2p_tlv != NULL )
                            {
                                memcpy(&workspace->group_candidate.bssid, &p2p_tlv->data[0], sizeof(besl_mac_t));
                            }

                            /* Look for capability attribute */
                            p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_P2P_CAPABILITY_INFO );
                            if ( p2p_tlv != NULL )
                            {
                                workspace->group_candidate.p2p_capability = *(uint16_t*)p2p_tlv->data;
                            }
                        }

                        /* Update the device status. If it was not recently discovered then we've invited it once before */
                        if (device->status == P2P_DEVICE_DISCOVERED)
                        {
                            ;//device->status = P2P_DEVICE_REQUESTED_TO_FORM_GROUP;
                        }
                        else if ( device->status != P2P_DEVICE_MUST_ALSO_BE_GROUP_OWNER )
                        {
                            device->status = P2P_DEVICE_ACCEPTED_GROUP_FORMATION;
                        }

                        if ( workspace->i_am_group_owner == 1 )
                        {
                            if ( p2p_find_common_channel_list( workspace, device, p2p_tlv ) != WICED_TRUE )
                            {
                                device->status = P2P_DEVICE_NO_COMMON_CHANNEL;
                            }
                        }

                        /* Copy the dialog token */
                        device->dialog_token = message->fixed.p2p_dialog_token;

                        if ( workspace->wps_data_length != 0 )
                        {
                            /* Read the device password id out of the WSC IE */
                            tlv16_data_t* device_password_id = tlv_find_tlv16(workspace->wps_data, workspace->wps_data_length, WPS_ID_DEVICE_PWD_ID);
                            if (device_password_id != NULL)
                            {
                                device->p2p_wps_device_password_id = ( wps_device_password_id_t ) BESL_READ_16_BE(device_password_id->data);
                            }
                        }

                        memcpy( &connecting_device, device, sizeof(p2p_discovered_device_t));
                        connecting_device.status = P2P_DEVICE_REQUESTED_TO_FORM_GROUP;

                        result = p2p_host_notify_negotiation_request( workspace, &connecting_device );

                        if (result == BESL_P2P_ACCEPT_CONNECTION)
                        {
                            workspace->p2p_current_state = P2P_STATE_NEGOTIATING;
                            workspace->sent_negotiation_confirm = 0;
                            workspace->sent_negotiation_request = 0;
                            besl_host_get_time( &workspace->group_formation_start_time );
                        }

                        /*  Send GO Response */
                        besl_p2p_send_action_frame( workspace, device, p2p_write_negotiation_response, workspace->listen_channel.channel, 100 );
                    }
                    break;

                case P2P_GO_NEGOTIATION_RESPONSE: /*  GO Response */
                    result = BESL_UNPROCESSED;

                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_STATUS );
                    if ( p2p_tlv == NULL )
                    {
                        if ( workspace->current_channel != workspace->listen_channel.channel )
                        {
                            BESL_INFO(("Return to the listen channel %d->%d\r\n", (int)workspace->current_channel, (int)workspace->listen_channel.channel));
                            workspace->current_channel = workspace->listen_channel.channel;
                            wwd_wifi_set_channel( WWD_P2P_INTERFACE, workspace->current_channel );
                        }
                        return;
                    }
                    workspace->sent_negotiation_request = 0;
                    if ( p2p_tlv->data[0] == P2P_STATUS_SUCCESS )
                    {
                        /*  Success! */
                        device->status = P2P_DEVICE_ACCEPTED_GROUP_FORMATION;
                        result = BESL_SUCCESS;
                    }
                    else if ( p2p_tlv->data[0] == P2P_STATUS_INFORMATION_UNAVAILABLE )
                    {
                        device->status = P2P_DEVICE_NOT_READY;
                    }
                    else if ( p2p_tlv->data[0] == P2P_STATUS_BOTH_DEVICES_MUST_BE_GO )
                    {
                        device->status = P2P_DEVICE_MUST_ALSO_BE_GROUP_OWNER;
                        result = BESL_SUCCESS;
                    }
                    else if ( p2p_tlv->data[0] == P2P_STATUS_INCOMPATIBLE_WPS_METHOD )
                    {
                        device->status = P2P_DEVICE_INCOMPATIBLE_WPS_METHOD;
                    }
                    else
                    {
                        device->status = P2P_DEVICE_DECLINED_INVITATION;
                    }

                    /* Return to the listen channel if the negitiation request is denied */
                    if ( result != BESL_SUCCESS )
                    {
                        BESL_INFO(("Found an error status in GO-NEGO-RESP: %02X\r\n", (int)p2p_tlv->data[0]));
                        if ( workspace->current_channel != workspace->listen_channel.channel )
                        {
                            BESL_INFO(("Return to the listen channel %d->%d\r\n", (int)workspace->current_channel, (int)workspace->listen_channel.channel));
                            workspace->current_channel = workspace->listen_channel.channel;
                            wwd_wifi_set_channel( WWD_P2P_INTERFACE, workspace->current_channel );
                        }
                        break;
                    }

                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_GROUP_OWNER_INTENT );
                    if ( p2p_tlv != NULL )
                    {
                        device->tie_breaker = p2p_tlv->data[0] & 0x01;
                        device->group_owner_intent = (p2p_tlv->data[0] >> 1);

                        if ( ( device->group_owner_intent < workspace->group_owner_intent ) ||
                             ( ( device->group_owner_intent == workspace->group_owner_intent ) &&  ( device->tie_breaker == 0 ) ) )
                        {
                            workspace->i_am_group_owner = 1;
                            memcpy( &workspace->group_candidate.bssid, &workspace->p2p_interface_address, sizeof(besl_mac_t) );

                            /* Change WPS device password ID as per Table 1 in P2P specification v1.2 */
                            if ( ( device->status == P2P_DEVICE_ACCEPTED_GROUP_FORMATION ) && ( workspace->p2p_wps_device_password_id == WPS_USER_SPEC_DEVICEPWDID ) )
                            {
                                workspace->p2p_wps_device_password_id = WPS_DEFAULT_DEVICEPWDID;
                            }
                        }
                        else
                        {
                            workspace->i_am_group_owner = 0;
                            p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_INTENDED_INTERFACE_ADDRESS );
                            if ( p2p_tlv != NULL )
                            {
                                memcpy( &workspace->group_candidate.bssid, &p2p_tlv->data[0], sizeof(besl_mac_t) );
                            }
                            p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_GROUP_ID );
                            if ( p2p_tlv != NULL )
                            {
                                memset( &workspace->group_candidate.ssid, 0, 32 );
                                workspace->group_candidate.ssid_length = MIN( p2p_tlv->length - 6, 32 );
                                memcpy( &workspace->group_candidate.ssid, &p2p_tlv->data[6], workspace->group_candidate.ssid_length );
                            }
                            /* Look for configuration timeout */
                            p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_CONFIGURATION_TIMEOUT );
                            if (p2p_tlv != NULL)
                            {
                                workspace->group_candidate.configuration_timeout = p2p_tlv->data[0];
                            }
                            /* Change WPS device password ID as per Table 1 in P2P specification v1.2 */
                            if ( ( device->status == P2P_DEVICE_ACCEPTED_GROUP_FORMATION ) && ( workspace->p2p_wps_device_password_id == WPS_DEVICEPWDID_REG_SPEC ) )
                            {
                                workspace->p2p_wps_device_password_id = WPS_DEFAULT_DEVICEPWDID;
                            }
                            /* Look for capability attribute */
                            p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_P2P_CAPABILITY_INFO );
                            if ( p2p_tlv != NULL )
                            {
                                workspace->group_candidate.p2p_capability = *(uint16_t*)p2p_tlv->data;
                            }
                        }
                    }

                    /* Look for operating channel */
                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_OP_CHANNEL );
                    if (p2p_tlv != NULL)
                    {
                        memcpy(&device->operating_channel, &p2p_tlv->data[0], sizeof(p2p_channel_info_t));
                        if (workspace->i_am_group_owner == 0)
                        {
                            memcpy(&workspace->group_candidate.operating_channel, &p2p_tlv->data[0], sizeof(p2p_channel_info_t));
                        }
                    }

                    /* Look for channel list */
                    if ( p2p_find_common_channel_list( workspace, device, p2p_tlv ) != WICED_TRUE )
                    {
                        device->status = P2P_DEVICE_NO_COMMON_CHANNEL;
                    }
                    else
                    {
                        /* If we're the group owner we have to select an acceptable operating channel. */
                        if ( workspace->i_am_group_owner == 1 )
                        {
                            wiced_bool_t        channel_selected = WICED_FALSE;
                            p2p_channel_list_t* p2p_chan_list;
                            int a;

                            p2p_chan_list = (p2p_channel_list_t*)&workspace->common_channel_list.p2p_channel_list;

                            while ( ( channel_selected == WICED_FALSE ) && ( p2p_chan_list->operating_class > 0 ) )
                            {
                                if ( workspace->operating_channel.operating_class == p2p_chan_list->operating_class )
                                {
                                    for ( a = 0; a < p2p_chan_list->number_of_channels; a++ )
                                    {
                                        if ( workspace->operating_channel.channel == p2p_chan_list->channel_list[a] )
                                        {
                                            channel_selected = WICED_TRUE;
                                            workspace->group_candidate.operating_channel.channel = p2p_chan_list->channel_list[a];
                                            workspace->group_candidate.operating_channel.operating_class = p2p_chan_list->operating_class;
                                            break;
                                        }
                                    }
                                }
                                p2p_chan_list++;
                            }

                            if ( channel_selected == WICED_FALSE )
                            {
                                // XXX need to implement preferred channel selection
                                p2p_chan_list = (p2p_channel_list_t*)&workspace->common_channel_list.p2p_channel_list;
                                workspace->group_candidate.operating_channel.channel = p2p_chan_list->channel_list[0];
                                workspace->group_candidate.operating_channel.operating_class = p2p_chan_list->operating_class;
                                channel_selected = WICED_TRUE;
                            }
                        }
                    }

                    if ( device->status == P2P_DEVICE_ACCEPTED_GROUP_FORMATION ) // XXX this check should probably be further up
                    {
                        /* Copy the dialog token */
                        device->dialog_token = message->fixed.p2p_dialog_token;
                        p2p_stop_timer( workspace );
                        if ( besl_p2p_send_action_frame( workspace, device, p2p_write_negotiation_confirmation, device->listen_channel, 2 ) != 0)
                        {
                            BESL_DEBUG(("P2P failed to send negotiation confirm\r\n"));
                        }
                        workspace->sent_negotiation_confirm = 1;
                    }
                    else
                    {
                        p2p_stop_timer( workspace );
                        if ( device->status == P2P_DEVICE_MUST_ALSO_BE_GROUP_OWNER )
                        {
                            p2p_stop( workspace );
                        }
                        // XXX need to signal that negotiation has failed depending on the status value
                    }
                    break;

                case P2P_GO_NEGOTIATION_CONFIRMATION: // XXX should check we are in negotiation with this device
                    /* Extract out operating channel and SSID */
                    p2p_tlv = p2p_find_tlv(workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_OP_CHANNEL);
                    if (p2p_tlv != NULL)
                    {
                        memcpy( &workspace->group_candidate.operating_channel, p2p_tlv->data, sizeof(p2p_channel_info_t) );
                    }

                    if (workspace->i_am_group_owner == 0)
                    {
                        p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_GROUP_ID );
                        if ( p2p_tlv != NULL )
                        {
                            memset( &workspace->group_candidate.ssid, 0, 32 );
                            workspace->group_candidate.ssid_length = MIN( p2p_tlv->length - 6, 32 );
                            memcpy( &workspace->group_candidate.ssid, &p2p_tlv->data[6], workspace->group_candidate.ssid_length );
                        }
                    }
                    besl_p2p_host_negotiation_complete(workspace);
                    break;

                case P2P_PROVISION_DISCOVERY_REQUEST:
                    /* Copy the dialog token */
                    device->dialog_token = message->fixed.p2p_dialog_token;

                    /* Read the Config Methods out of the WSC IE */
                    config_method = tlv_find_tlv16(workspace->wps_data, workspace->wps_data_length, WPS_ID_CONFIG_METHODS);
                    if (config_method != NULL)
                    {
                        device->preferred_config_method = BESL_READ_16_BE(config_method->data);
                    }
                    else
                    {
                        break;
                    }

                    /* Extract the Device Info attribute */
                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_DEVICE_INFO );
                    if ( p2p_tlv != NULL )
                    {
                        p2p_device_info = (p2p_device_info_t*)p2p_tlv->data;
                        /* Find the WPS device name attribute which is in the P2P device info attribute */
                        tlv16 = (tlv16_data_t*)((uint8_t*)p2p_device_info + sizeof(p2p_device_info_t));
                        if ( tlv16 != NULL )
                        {
                            memcpy( &device->device_name, tlv16->data, htobe16(tlv16->length) ); // XXX htobe16
                        }
                    }
                    else
                    {
                        break;
                    }

                    memcpy( &connecting_device, device, sizeof(p2p_discovered_device_t));
                    connecting_device.status = P2P_DEVICE_REQUESTED_TO_FORM_GROUP;
                    p2p_message.type = P2P_EVENT_CONNECTION_REQUESTED;
                    p2p_message.data = (void*)&connecting_device;
                    p2p_host_send_message( &p2p_message, WICED_NO_WAIT );

                    if ( workspace->group_owner_is_up == 0 )
                    {
                        channel = workspace->listen_channel.channel;
                    }
                    else
                    {
                        channel = workspace->operating_channel.channel;
                    }

                    besl_p2p_send_action_frame( workspace, device, p2p_write_provision_discovery_response, channel, 2 );
                    break;

                case P2P_PROVISION_DISCOVERY_RESPONSE:
                    if ( workspace->sent_provision_discovery_request == 1 ) // XXX should check mac address etc
                    {
                        /* Read the Config Methods out of the WSC IE */
                        config_method = tlv_find_tlv16(workspace->wps_data, workspace->wps_data_length, WPS_ID_CONFIG_METHODS);
                        if (config_method != NULL)
                        {
                            if ( workspace->provisioning_config_method == BESL_READ_16_BE(config_method->data) )
                            {
                                // XXX there's no status to check but we should check mac, dialog token
                                device->status = P2P_DEVICE_ACCEPTED_GROUP_FORMATION;
                                workspace->sent_provision_discovery_request = 0;
                                besl_p2p_host_negotiation_complete( workspace );
                                p2p_stop_timer( workspace );
                            }
                            else
                            {
                                BESL_DEBUG(("P2P provision discovery config method does not match\r\n"));
                            }
                        }
                    }
                    break;

                case P2P_INVITATION_REQUEST:
                    /* Copy the dialog token */
                    device->dialog_token = message->fixed.p2p_dialog_token;

                    /*  Also need operating channel, group BSSID and SSID etc */
                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_GROUP_BSSID );
                    if (p2p_tlv != NULL)
                    {
                        memcpy(&workspace->invitation_candidate.bssid, p2p_tlv->data, sizeof(besl_mac_t));
                    }

                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_GROUP_ID );
                    if ( p2p_tlv != NULL )
                    {
                        memcpy( &device->group_owner_device_address, &p2p_tlv->data[0], sizeof( besl_mac_t ) );
                        /* Note that the invitation candidate is a group, not a device */
                        memcpy( &workspace->invitation_candidate.p2p_device_address, &p2p_tlv->data[0], sizeof( besl_mac_t ) );
                        memset( &workspace->invitation_candidate.ssid, 0, 32 );
                        workspace->invitation_candidate.ssid_length = p2p_tlv->length - sizeof(besl_mac_t);
                        memcpy( &workspace->invitation_candidate.ssid, &p2p_tlv->data[6], workspace->invitation_candidate.ssid_length );
                    }

                    /* Extract the Device Info attribute */
                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_DEVICE_INFO );
                    if ( p2p_tlv != NULL )
                    {
                        p2p_device_info = (p2p_device_info_t*)p2p_tlv->data;
                        /* Find the WPS device name attribute which is in the P2P device info attribute */
                        tlv16 = (tlv16_data_t*)((uint8_t*)p2p_device_info + sizeof(p2p_device_info_t));
                        if ( tlv16 != NULL )
                        {
                            memcpy( &device->device_name, tlv16->data, htobe16(tlv16->length) ); // XXX htobe16
                        }
                    }

                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_OP_CHANNEL );
                    if (p2p_tlv != NULL)
                    {
                        p2p_channel_info_t *device = (p2p_channel_info_t*) p2p_tlv->data;
                        p2p_channel_list_t *p2p_channel_list;
                        wiced_bool_t channel_selected = WICED_FALSE;
                        int i, j;

                        p2p_channel_list = (p2p_channel_list_t*)workspace->channel_list.p2p_channel_list;
                        for ( i = 0; i < workspace->channel_list.p2p_channel_list_tlv.total_classes; i++ )
                        {
                            for( j = 0; j < p2p_channel_list->number_of_channels; j++ )
                            {
                                if ( device->channel == p2p_channel_list->channel_list[j] )
                                {
                                    channel_selected = WICED_TRUE;
                                    break;
                                }
                            }
                            p2p_channel_list++;
                        }
                        BESL_INFO(("%s device channel %d \n", __func__, device->channel));

                        /* Check if channel list includes operating channel. Otherwise, set default op channel */
                        if ( channel_selected == WICED_TRUE)
                        {
                            memcpy( &workspace->invitation_candidate.operating_channel, p2p_tlv->data, sizeof( p2p_channel_info_t ) );
                        }
                        else
                        {
                            memcpy( &workspace->invitation_candidate.operating_channel, &workspace->operating_channel, sizeof( p2p_channel_info_t ) );
                        }
                        BESL_INFO(("%s channel_selected %d, channel %d \n", __func__, channel_selected, workspace->invitation_candidate.operating_channel.channel));
                    }

                    if ( ( workspace->p2p_current_state == P2P_STATE_GROUP_OWNER) || ( workspace->p2p_current_state == P2P_STATE_GROUP_CLIENT ) )
                    {
                        channel = workspace->operating_channel.channel;
                    }
                    else
                    {
                        channel = workspace->listen_channel.channel;
                    }

                    if ( p2p_find_common_channel_list( workspace, device, p2p_tlv ) != WICED_TRUE )
                    {
                        device->status = P2P_DEVICE_NO_COMMON_CHANNEL;
                    }

                    p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_INVITATION_FLAGS );
                    if (p2p_tlv != NULL)
                    {
                        device->invitation_flags = p2p_tlv->data[4];
                    }

                    besl_p2p_send_action_frame( workspace, device, p2p_write_invitation_response, channel, 2 );

                    /* Order the message so it gets processed after the invitation response is sent */
                    memcpy( &connecting_device, device, sizeof(p2p_discovered_device_t));
                    connecting_device.status = P2P_DEVICE_INVITATION_REQUEST;
                    p2p_message.type = P2P_EVENT_CONNECTION_REQUESTED;
                    p2p_message.data = (void*)&connecting_device;
                    p2p_host_send_message( &p2p_message, WICED_NO_WAIT );
                    break;

                case P2P_INVITATION_RESPONSE:
                    if ( ( workspace->sent_invitation_request == 1 ) && ( workspace->candidate_device != NULL ) ) // XXX compare device address
                    {
                        if ( memcmp( &device->p2p_device_address, &workspace->candidate_device->p2p_device_address, sizeof( besl_mac_t )) == 0 )
                        {
                            workspace->sent_invitation_request = 0;
                            p2p_stop_timer( workspace );
                            if ( workspace->reinvoking_group == 1 )
                            {
                                /* Look for configuration timeout */
                                p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_CONFIGURATION_TIMEOUT );
                                if (p2p_tlv != NULL)
                                {
                                    workspace->group_candidate.configuration_timeout = p2p_tlv->data[0];
                                }
                                /* Look for operating channel */
                                p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_OP_CHANNEL );
                                if (p2p_tlv != NULL)
                                {
                                    memcpy( &workspace->group_candidate.operating_channel, p2p_tlv->data, sizeof(p2p_channel_info_t) );
                                }
                                // XXX need to review channel use in case the channel is zero or in a wrong band etc.
                                besl_p2p_host_negotiation_complete(workspace);
                            }
                        }
                    }
                    break;

                case P2P_DEVICE_DISCOVERABILITY_REQUEST:
                    /* Only handle one discovery request at a time */
                    if ( ( workspace->group_owner_is_up == 1 ) && ( workspace->sent_go_discoverability_request == 0 ) )
                    {
                        p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_DEVICE_ID );
                        if (p2p_tlv == NULL)
                        {
                            return;
                        }

                        discovery_target = p2p_host_find_associated_p2p_device( workspace, (besl_mac_t*)&p2p_tlv->data );
                        if ( discovery_target == NULL ) // XXX or device does not support discoverability
                        {
                            device->status = P2P_DEVICE_DISCOVERED;
                            besl_p2p_send_action_frame( workspace, device, p2p_write_device_discoverability_response, (uint32_t)workspace->operating_channel.channel, 2 );
                        }
                        else
                        {
                            /* We have to send a GO discoverability request to the client device to wake it up and then send a Device Discoverability Response to the
                             * device that sent us the Device Discoverability Request, so save the requesting device's details.
                             */
                            workspace->sent_go_discoverability_request = 1;
                            workspace->discovery_dialog_token = message->fixed.p2p_dialog_token;
                            workspace->discovery_target = discovery_target;
                            memcpy( &workspace->discovery_requestor, &device->p2p_device_address, sizeof( besl_mac_t ) );

                            // XXX hack
                            memcpy( &device->p2p_device_address, &discovery_target->p2p_interface_address, sizeof( besl_mac_t ) );
                            besl_p2p_send_action_frame( workspace, device, p2p_write_go_discoverability_request, (uint32_t)workspace->operating_channel.channel, 0 );
                        }
                    }
                    break;

                default:
                    break;
            }
        }
    }
    else
    /* Check if it's a non-public action frame */
    {
        if ( ( action_frame_message->fixed.category_code == P2P_ACTION_FRAME_CATEGORY_CODE ) &&
             ( action_frame_message->fixed.oui_sub_type  == P2P_OUI_SUB_TYPE ) &&
             ( memcmp( action_frame_message->fixed.oui, P2P_OUI, 3 ) == 0 ) )
        {
            p2p_client_info_descriptor_t* associated_device = p2p_host_find_associated_p2p_device( workspace, (besl_mac_t*)&event_header->addr );

            if ( associated_device != NULL )
            {
                /* Copy the dialog token */
                temp_device.dialog_token = action_frame_message->fixed.p2p_dialog_token;
                memcpy( &temp_device.p2p_device_address, &associated_device->p2p_interface_address, sizeof( besl_mac_t ) );

                switch (action_frame_message->fixed.p2p_sub_type)
                {
                    case P2P_PRESENCE_REQUEST:
                        besl_p2p_send_action_frame( workspace, &temp_device, p2p_write_presence_response, workspace->current_channel, 2 );
                        break;

                    default:
                        break;
                }
            }
            else
            {
                BESL_DEBUG( ( "P2P received action frame from non-associated STA\n" ) );
            }
        }
    }
}

static besl_result_t p2p_defragment_wps_and_p2p_elements( p2p_workspace_t* workspace, const uint8_t* data, uint32_t data_length )
{
    workspace->p2p_data_length = 0;
    workspace->wps_data_length = 0;

    tlv8_data_t* tlv = (tlv8_data_t*) data;
    while ( ( (uint8_t*) tlv - data ) < data_length )
    {
        if ( tlv->type == 221 )
        {
            /* Check the OUI and OUI type */
            if ( memcmp( tlv->data, P2P_OUI, 3 ) == 0 && tlv->data[3] == P2P_OUI_TYPE )
            {
                if ( ( workspace->p2p_data_length + tlv->length ) <= P2P_DEFRAGMENTATION_BUFFER_LENGTH )
                {
                    memcpy( workspace->p2p_data + workspace->p2p_data_length, &tlv->data[4], tlv->length - 4 );
                    workspace->p2p_data_length += (uint32_t)( tlv->length - 4 );
                }
            }
            else if ( memcmp( tlv->data, WPS_OUI, 3 ) == 0 && tlv->data[3] == WPS_OUI_TYPE )
            {
                if ( ( workspace->wps_data_length + tlv->length ) <= P2P_DEFRAGMENTATION_BUFFER_LENGTH )
                {
                    memcpy( workspace->wps_data + workspace->wps_data_length, &tlv->data[4], tlv->length - 4 );
                    workspace->wps_data_length += (uint32_t)( tlv->length - 4 );
                }
            }
        }
        tlv = (tlv8_data_t*)((uint8_t*)tlv + sizeof(tlv8_header_t) + tlv->length);
    }

    return BESL_SUCCESS;
}

// This is called by the event handler so don't print unless Wiced thread has at least 4 KBytes memory
besl_result_t p2p_process_probe_request( p2p_workspace_t* workspace, const uint8_t* data, uint32_t data_length )
{
    p2p_legacy_device_t*     legacy_device      = NULL;
    p2p_message_t            message;
    ieee80211_header_t*      header             = (ieee80211_header_t*) ( data );
    besl_time_t              current_time;
    tlv16_uint16_t*          wps_pwd_id         = NULL;
    wps_device_password_id_t device_password_id = WPS_DEFAULT_DEVICEPWDID;
    tlv16_data_t*            device_name_tlv    = NULL;
    besl_result_t            result             = BESL_P2P_ERROR_FAIL;
    p2p_discovered_device_t* device             = NULL;
    besl_time_t              aligned_besl_time;

    data = ( (uint8_t*) header ) + sizeof(ieee80211_header_t);
    data_length -= sizeof(ieee80211_header_t);

    besl_host_get_time( &current_time );

    if ( p2p_defragment_wps_and_p2p_elements( workspace, data, data_length ) != BESL_SUCCESS )
    {
        BESL_DEBUG( ( "p2p_process_probe_request: defragmentation failed\r\n" ) );
        goto exit;
    }

    if ( workspace->wps_data_length != 0 )
    {
        if ( workspace->group_owner_is_up == 1 )
        {
            /* Look for WPS password ID and check for PBC overlap - applies to P2P and non-P2P devices */
            wps_pwd_id = (tlv16_uint16_t*) tlv_find_tlv16( workspace->wps_data, workspace->wps_data_length, WPS_ID_DEVICE_PWD_ID );
            if ( wps_pwd_id != NULL && besl_host_hton16( wps_pwd_id->data ) == WPS_PUSH_BTN_DEVICEPWDID )
            {
                wps_update_pbc_overlap_array( workspace->p2p_wps_agent, (besl_mac_t*)(header->address2) );
                device_password_id = WPS_PUSH_BTN_DEVICEPWDID;
            }
            /* Find the device name. May only be present for WPS 2.0 devices. */
            device_name_tlv = tlv_find_tlv16( workspace->wps_data, workspace->wps_data_length, WPS_ID_DEVICE_NAME );
        }
    }

    /* If the P2P IE was not found then it's a non-P2P device */
    if ( workspace->p2p_data_length == 0 )
    {
        if ( ( wps_pwd_id != NULL ) && ( workspace->group_owner_is_up == 1 )  && ( device_password_id == WPS_PUSH_BTN_DEVICEPWDID ) )
        {
            legacy_device = p2p_find_legacy_device( workspace, (besl_mac_t*)(header->address2) );
            if ( legacy_device != NULL )
            {
                legacy_device->wps_device_password_id = WPS_PUSH_BTN_DEVICEPWDID; // Record this because later we may record other device password ids
                memcpy( &legacy_device->mac_address, header->address2, sizeof(besl_mac_t));
                if ( device_name_tlv != NULL )
                {
                    memset( legacy_device->device_name, 0, 32 );
                    memcpy( legacy_device->device_name, device_name_tlv->data, MIN( htobe16(device_name_tlv->length), 32 ) );
                }

                /* If this is the first time we have seen the device or it's 3 seconds since the last time we alerted the application then call the callback */
                if ( ( legacy_device->in_use == 0 ) || ( ( legacy_device->last_alert_time + 3000 ) < current_time ) )
                {
                    legacy_device->in_use = 1;
                    legacy_device->last_alert_time = current_time;
                    message.type = P2P_EVENT_LEGACY_DEVICE_CONNECTION_REQUEST;
                    message.data = legacy_device;
                    p2p_host_send_message( &message, WICED_NO_WAIT );
                }
            }
        }
        goto exit;
    }

    /* Check if we've already seen this device */
    device = besl_p2p_host_find_device( workspace, (besl_mac_t*) header->address2 );
    if ( device == NULL )
    {
        goto exit;
    }

    besl_host_get_time( &aligned_besl_time );
    device->last_seen_time = aligned_besl_time;
    if ( device->status != P2P_DEVICE_INVALID )
    {
        goto exit;
    }

    /* Copy the MAC */
    memcpy( &device->p2p_device_address, header->address2, sizeof(besl_mac_t) );
    result = p2p_process_new_device_data( workspace, device );

exit:
    return result;
}

static besl_result_t p2p_process_scan_result( p2p_workspace_t* workspace, wl_escan_result_t* scan_result )
{
    wl_bss_info_t*        bss_info = &scan_result->bss_info[0];
    uint8_t*              data;
    tlv16_data_t*         tlv16 = NULL;
    uint32_t              data_length;
    p2p_device_info_t*    p2p_device_info = NULL;
    p2p_capability_tlv_t* p2p_capability_info = NULL;
    besl_result_t         result = BESL_P2P_ERROR_FAIL;
    besl_time_t           aligned_besl_time;

    /* Make sure the Wi-Fi Direct SSID is present. Note that a group owner SSID will be longer than 7 octets */
    if ( memcmp(bss_info->SSID, P2P_SSID_PREFIX, 7) != 0 )
    {
        goto exit;
    }

    /* Find the P2P IE and copy the device address etc */
    data  = (uint8_t*) ( ( (uint8_t*) bss_info ) + BESL_READ_16(&bss_info->ie_offset) );
    data_length = BESL_READ_32(&bss_info->ie_length);

    if ( p2p_defragment_wps_and_p2p_elements( workspace, data, data_length ) != BESL_SUCCESS )
    {
        BESL_ERROR(( "p2p_process_scan_result: defragmentation failed\r\n" ));
        goto exit;
    }

    /* Find the P2P element if it's present */
    if ( workspace->p2p_data_length != 0 )
    {
        p2p_tlv_data_t* p2p_tlv;

        /* Extract the capability attribute */
        p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_P2P_CAPABILITY_INFO );
        if ( p2p_tlv != NULL )
        {
            p2p_capability_info = (p2p_capability_tlv_t*)p2p_tlv;
        }

        /* Extract the Device Info attribute */
        p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_DEVICE_INFO );
        if ( p2p_tlv != NULL )
        {
            p2p_device_info = (p2p_device_info_t*)p2p_tlv->data;
            /* Find the WPS device name attribute which is in the P2P device info attribute */
            tlv16 = (tlv16_data_t*)((uint8_t*)p2p_device_info + sizeof(p2p_device_info_t));
        }
    }

    if ( p2p_device_info != NULL )
    {
        p2p_discovered_device_t* device = besl_p2p_host_find_device( workspace, &p2p_device_info->p2p_device_address );
        if ( device == NULL )
        {
            goto exit;
        }
        /* Always update channel, time, SSID and capability information in case the device has changed role or moved channel */
        device->listen_channel = (uint8_t)bss_info->chanspec;
        device->operating_channel.channel = (uint8_t)bss_info->chanspec;

        /* Update time that the device was last seen */
        besl_host_get_time( &aligned_besl_time );
        device->last_seen_time = aligned_besl_time;

        memcpy( device->ssid, (char*)bss_info->SSID, 32 );
        if ( p2p_capability_info != NULL )
        {
            device->device_capability = p2p_capability_info->device_capability;
            device->group_owner_capability = p2p_capability_info->group_capability;
        }

        /* If we have not already discovered it then add the rest of the information */
        if ( device->status == P2P_DEVICE_INVALID )
        {
            /* Flag it as a new device */
            device->status = P2P_DEVICE_DISCOVERED;
            ++workspace->discovered_device_count;

            /* Copy the device interface */
            memcpy( &device->p2p_device_address, &p2p_device_info->p2p_device_address, sizeof(besl_mac_t) );
            memcpy( &device->p2p_interface_address, &bss_info->BSSID, sizeof(besl_mac_t) );
            if ( tlv16 != NULL )
            {
                memcpy( &device->device_name, tlv16->data, htobe16(tlv16->length) ); // XXX htobe16
            }
        }

        result = BESL_SUCCESS;
    }

exit:
    return result;
}

besl_result_t p2p_process_association_request( p2p_workspace_t* workspace, const uint8_t* data, const wwd_event_header_t* event_header )
{
    p2p_device_info_t* p2p_device_info = NULL;
    p2p_capability_tlv_t* p2p_device_capability = NULL;
    tlv8_data_t* tlv8 = NULL;
    tlv16_data_t* tlv16 = NULL;
    uint32_t data_length = event_header->datalen;
    p2p_message_t   message;
    besl_result_t result = BESL_P2P_ERROR_FAIL;

    /* Check that the device is associating using WPA2 */
    tlv8 = tlv_find_tlv8( data, data_length, 48 ); // Check for presence of RSN IE
    if ( tlv8 == NULL ) // It's not WPA2 but since WPS 1.0 clients aren't required to put the WPS IE in association requests we indicate that a potential WPS enrollee has associated
    {
        memcpy( &workspace->wps_enrollee_mac, &event_header->addr, sizeof( besl_mac_t ) );
        message.type = P2P_EVENT_WPS_ENROLLEE_ASSOCIATED;
        message.data = &workspace->wps_enrollee_mac;
        p2p_host_send_message( &message, WICED_NO_WAIT );
        goto exit;
    }

    if ( p2p_defragment_wps_and_p2p_elements( workspace, data, data_length ) != BESL_SUCCESS )
    {
        BESL_DEBUG( ( "p2p_process_scan_result: defragmentation failed\r\n" ) );
        goto exit;
    }

    /* Find the P2P element if it's present */
    if ( workspace->p2p_data_length != 0 )
    {
        /* Extract the Device Info attribute */
        p2p_tlv_data_t* p2p_tlv;

        /* Extract the Device Capability bitmap */
        p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_P2P_CAPABILITY_INFO );
        if ( p2p_tlv != NULL )
        {
            p2p_device_capability = (p2p_capability_tlv_t*)p2p_tlv;
        }

        p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_DEVICE_INFO );
        if ( p2p_tlv != NULL )
        {
            p2p_device_info = (p2p_device_info_t*)p2p_tlv->data;
            /* Find the WPS device name attribute which is in the P2P device info attribute */
            tlv16 = (tlv16_data_t*)((uint8_t*)p2p_device_info + sizeof(p2p_device_info_t));
        }
    }

    /* If it's a P2P device then add it to the client descriptor table or update the existing entry */
    if ( p2p_device_info != NULL )
    {
        p2p_client_info_descriptor_t* associated_device = p2p_host_find_associated_p2p_device( workspace, &p2p_device_info->p2p_device_address );

        if ( associated_device == NULL )
        {
            associated_device = p2p_host_find_empty_associated_p2p_device_entry( workspace, &p2p_device_info->p2p_device_address );
        }

        /* The only static piece of information about a P2P device is its device address so we err on the side of updating all fields */
        if ( associated_device != NULL ) // It should not be NULL but could be due to race conditions between radio firmware and host
        {
            memcpy( &associated_device->p2p_device_address, &p2p_device_info->p2p_device_address, 6 );
            memcpy( &associated_device->p2p_interface_address, &event_header->addr, 6 );
            if ( p2p_device_capability != NULL )
            {
                associated_device->device_capability = p2p_device_capability->device_capability;
            }
            associated_device->config_methods = p2p_device_info->config_methods;
            memcpy( (uint8_t*)&associated_device->primary_type, &p2p_device_info->primary_type, 8 );
            associated_device->number_of_secondary_devices = (uint8_t)0; // Optional and currently unsupported by Wiced
            memcpy( &associated_device->device_name, tlv16, 4 );
            memcpy( &associated_device->device_name_data, tlv16->data, htobe16(tlv16->length) ); // XXX htobe16
            associated_device->length = (uint8_t)(sizeof(p2p_client_info_descriptor_t) - 1 - 32 + htobe16(tlv16->length));

            /* Update the list of discovered devices */
            p2p_discovered_device_t* device = besl_p2p_host_find_device( workspace, &associated_device->p2p_device_address );
            if ( device != NULL )
            {
                device->status = P2P_DEVICE_ASSOCIATED_CLIENT;
            }
        }
        message.type = P2P_EVENT_P2P_DEVICE_ASSOCIATED;
    }
    else
    {
        besl_mac_t* legacy_device = p2p_host_find_associated_legacy_device( workspace, (besl_mac_t*)&event_header->addr );

        if ( legacy_device == NULL )
        {
            legacy_device = p2p_host_find_empty_associated_legacy_device_entry( workspace, (besl_mac_t*)&event_header->addr );
            if ( legacy_device != NULL )
            {
                memcpy( legacy_device, &event_header->addr, sizeof( besl_mac_t ) );
            }
        }
        message.type = P2P_EVENT_LEGACY_DEVICE_ASSOCIATED;
    }

    memcpy( &workspace->wpa2_client_mac, &event_header->addr, sizeof( besl_mac_t ) );
    message.data = &workspace->wpa2_client_mac;
    p2p_host_send_message( &message, WICED_NO_WAIT );

    result = BESL_SUCCESS;

exit:
    return result;
}

besl_result_t p2p_process_new_device_data( p2p_workspace_t* workspace, p2p_discovered_device_t* device )
{
    besl_time_t aligned_besl_time;

    /* Extract the listen channel */
    if ( workspace->p2p_data_length > 0 )
    {
        p2p_tlv_data_t* p2p_tlv = p2p_find_tlv( workspace->p2p_data, workspace->p2p_data_length, P2P_SEID_CHANNEL );
        if ( p2p_tlv != NULL )
        {
            device->listen_channel = p2p_tlv->data[4];
        }
    }
    if ( workspace->wps_data_length > 0 )
    {
        /* Copy out the device name */
        tlv16_data_t* tlv16 = tlv_find_tlv16( workspace->wps_data, workspace->wps_data_length, WPS_ID_DEVICE_NAME );
        if ( tlv16 != NULL )
        {
            memcpy( device->device_name, tlv16->data, MIN(sizeof(device->device_name), besl_host_hton16(tlv16->length)) );
        }
    }

    /* Flag it as a new device */
    device->status = P2P_DEVICE_DISCOVERED;
    besl_host_get_time( &aligned_besl_time );
    device->last_seen_time = aligned_besl_time;
    ++workspace->discovered_device_count;

    return BESL_SUCCESS;
}


// XXX this may be called by the P2P event handler so don't print unless Wiced thread has at least 4 KBytes memory
besl_result_t p2p_update_devices( p2p_workspace_t* workspace )
{
    int a = 0;
    besl_time_t current_time;

    if ( workspace->discovered_device_count > 0 )
    {
        besl_host_get_time( &current_time );

        while ( a < P2P_MAX_DISCOVERED_DEVICES )
        {
            if ( ( workspace->discovered_devices[a].status != P2P_DEVICE_INVALID ) &&
                 ( workspace->discovered_devices[a].status != P2P_DEVICE_ASSOCIATED_CLIENT ) &&
                 ( ( workspace->discovered_devices[a].last_seen_time + workspace->peer_device_timeout ) <= current_time ) )
            {
                memset( &workspace->discovered_devices[a], 0, sizeof( p2p_discovered_device_t ) );
                workspace->discovered_device_count--;
            }
            a++;
        }
    }

    return BESL_SUCCESS;
}


/******************************************************
 *            Static Function Definitions
 ******************************************************/

static void p2p_process_disassociation( p2p_workspace_t* workspace, const uint8_t* data, const wwd_event_header_t* event_header )
{
    p2p_message_t message;

    memcpy( &workspace->disassociating_client_mac, &event_header->addr, sizeof(besl_mac_t) );

    if ( !NULL_MAC( workspace->disassociating_client_mac.octet ) )
    {
        message.type = P2P_EVENT_DEVICE_DISASSOCIATED;
        message.data = &workspace->disassociating_client_mac;
        p2p_host_send_message( &message, WICED_NO_WAIT );
    }
}


static besl_result_t p2p_init_as_group_owner(p2p_workspace_t* workspace)
{
    workspace->i_am_group_owner = 1;
    workspace->group_candidate.operating_channel.channel = workspace->operating_channel.channel;

    if ( strlen( workspace->p2p_ap_suffix ) != 0 )
    {
        workspace->group_candidate.ssid[workspace->group_candidate.ssid_length++] = '-';
        memcpy(&workspace->group_candidate.ssid[workspace->group_candidate.ssid_length], workspace->p2p_ap_suffix, strlen(workspace->p2p_ap_suffix));
        workspace->group_candidate.ssid_length += strlen( workspace->p2p_ap_suffix );
    }

    return BESL_SUCCESS;
}


/* Use one function to search for either the device address or the interface address because some post-association p2p action frames only have the interface address */
p2p_client_info_descriptor_t* p2p_host_find_associated_p2p_device( p2p_workspace_t* workspace, const besl_mac_t* mac_address )
{
    int i = 0;

    for ( i = 0; i < P2P_MAX_ASSOCIATED_DEVICES; i++ )
    {
        if ( ( memcmp( &workspace->associated_p2p_devices[i].p2p_device_address, mac_address, sizeof(besl_mac_t) ) == 0 ) ||
             ( memcmp( &workspace->associated_p2p_devices[i].p2p_interface_address, mac_address, sizeof(besl_mac_t) ) == 0 ) )
        {
            return &workspace->associated_p2p_devices[i];
        }
    }
    return NULL;
}

besl_mac_t* p2p_host_find_associated_legacy_device( p2p_workspace_t* workspace, const besl_mac_t* mac_address )
{
    int i = 0;

    for ( i = 0; i < P2P_MAX_ASSOCIATED_DEVICES; i++ )
    {
        if ( memcmp( &workspace->associated_legacy_devices[i], mac_address, sizeof(besl_mac_t) ) == 0 )
        {
            return &workspace->associated_legacy_devices[i];
        }
    }
    return NULL;
}

static p2p_client_info_descriptor_t* p2p_host_find_empty_associated_p2p_device_entry( p2p_workspace_t* workspace, besl_mac_t* p2p_device_address )
{
    int i = 0;

    for ( i = 0; i < P2P_MAX_ASSOCIATED_DEVICES; i++ )
    {
        if ( NULL_MAC( workspace->associated_p2p_devices[i].p2p_device_address.octet ) )
        {
            workspace->associated_p2p_device_count++;
            return &workspace->associated_p2p_devices[i];
        }
    }
    return NULL;
}

static besl_mac_t* p2p_host_find_empty_associated_legacy_device_entry( p2p_workspace_t* workspace, const besl_mac_t* p2p_device_address )
{
    int i = 0;

    for ( i = 0; i < P2P_MAX_ASSOCIATED_DEVICES; i++ )
    {
        if ( NULL_MAC( workspace->associated_legacy_devices[i].octet ) )
        {
            return &workspace->associated_legacy_devices[i];
        }
    }
    return NULL;
}

/* This may be called by the P2P event handler so don't print unless Wiced thread has at least 4 KBytes memory */
static p2p_legacy_device_t* p2p_find_legacy_device( p2p_workspace_t* workspace, besl_mac_t* mac )
{
    int a = 0;
    int unused = 0;
    int oldest = 0;

    /* Return the entry that matches the MAC address or an unused entry or the oldest entry, in that order */
    while ( a < P2P_MAX_DISCOVERED_LEGACY_DEVICES )
    {
        /* Check if we have already seen this device */
        if ( memcmp( &workspace->legacy_devices[a].mac_address, mac, sizeof( besl_mac_t ) ) == 0 )
        {
            return &workspace->legacy_devices[a]; // Found a match
        }
        else if ( workspace->legacy_devices[a].in_use == 0 )
        {
            unused = a; // Record last unused entry in the array
        }
        else if ( workspace->legacy_devices[a].last_alert_time < workspace->legacy_devices[oldest].last_alert_time )
        {
            oldest = a;
        }
        a++;
    }

    if ( unused < P2P_MAX_DISCOVERED_LEGACY_DEVICES )
    {
        memset( &workspace->legacy_devices[unused], 0, sizeof( p2p_legacy_device_t ) );
        return &workspace->legacy_devices[unused];
    }

    memset( &workspace->legacy_devices[oldest], 0, sizeof( p2p_legacy_device_t ) );

    return &workspace->legacy_devices[oldest];
}

static p2p_tlv_data_t* p2p_find_tlv(const uint8_t* message, uint16_t message_length, uint16_t type)
{
    p2p_tlv_data_t* tlv = (p2p_tlv_data_t*) message;
    while ( ( (uint8_t*) tlv - message ) < message_length )
    {
        if ( tlv->type == type )
        {
            return tlv;
        }
        tlv = (p2p_tlv_data_t*)((uint8_t*)tlv + sizeof(p2p_tlv_header_t) + tlv->length);
    }
    return NULL;
}

static besl_result_t p2p_host_notify_negotiation_request( p2p_workspace_t* workspace, p2p_discovered_device_t* device )
{
    p2p_message_t message;

    /* Work out which WPS mode the device is using */
    device->p2p_wps_mode = WPS_NO_MODE;
    switch ( device->p2p_wps_device_password_id )
    {
        case WPS_DEFAULT_DEVICEPWDID:
        case WPS_USER_SPEC_DEVICEPWDID:
        case WPS_DEVICEPWDID_REG_SPEC:
            device->p2p_wps_mode = WPS_PIN_MODE;
            break;

        case WPS_PUSH_BTN_DEVICEPWDID:
            device->p2p_wps_mode = WPS_PBC_MODE;
            break;

        default:
            break;
    }

    if ( ( device->status == P2P_DEVICE_WAS_INVITED_TO_FORM_GROUP ) || ( device->status == P2P_DEVICE_ACCEPTED_GROUP_FORMATION ) )
    {
        if ( device->p2p_wps_mode == workspace->p2p_wps_mode )
        {
            return BESL_P2P_ACCEPT_CONNECTION;
        }
        else
        {
            ; // XXX inform user that there's a mismatch of wps method or password id
        }
    }
    else if ( ( device->p2p_wps_mode == workspace->p2p_wps_mode ) && ( workspace->ok_to_accept_negotiation == 1 ) ) // XXX also compare mac addresses
    {
        return BESL_P2P_ACCEPT_CONNECTION;
    }
    else
    {
        device->status = P2P_DEVICE_REQUESTED_TO_FORM_GROUP;
        message.type = P2P_EVENT_CONNECTION_REQUESTED;
        message.data = (void*)device;
        p2p_host_send_message( &message, WICED_NO_WAIT );
    }

    return BESL_IN_PROGRESS;
}
