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
#include "wiced_p2p.h"
#include "internal/wwd_sdpcm.h"
#include "wiced_wps.h"
#include "besl_host_rtos_structures.h"
#include "internal/wiced_internal_api.h"
#include "wiced_security_internal.h"
#include "p2p_constants.h"
#include "p2p_structures.h"
#include "p2p_frame_writer.h"
#include "wwd_buffer_interface.h"
#include <string.h>

/******************************************************
 *                      Macros
 ******************************************************/

#define CHECK_IOCTL_BUFFER( buff )  if ( buff == NULL ) {  wiced_assert("Allocation failed\n", 0 == 1); return BESL_BUFFER_ALLOC_FAIL; }
#define CHECK_RETURN( expr )  { besl_result_t check_res = (expr); if ( check_res != BESL_SUCCESS ) { wiced_assert("Command failed\n", 0 == 1); return check_res; } }


/******************************************************
 *                    Constants
 ******************************************************/

#define DOT11_PMK_LEN (32)
#define WIFI_BAND_B_LAST_CHANNEL (14)

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

static besl_result_t besl_p2p_check_soft_ap_interface  ( p2p_workspace_t* workspace );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t p2p_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};

/******************************************************
 *               Function Definitions
 ******************************************************/

besl_result_t besl_p2p_init( p2p_workspace_t* workspace, const besl_p2p_device_detail_t* device_details )
{
    wiced_buffer_t buffer;
    wiced_buffer_t response;
    uint32_t*      data;
    besl_result_t  result = BESL_SUCCESS;
    wwd_result_t wwd_result = WWD_SUCCESS;

    REFERENCE_DEBUG_ONLY_VARIABLE(result);

    memset( workspace, 0, sizeof(p2p_workspace_t) );

    workspace->group_owner_intent             = device_details->group_owner_intent;
    workspace->listen_channel.operating_class = device_details->listen_channel.operating_class;
    workspace->listen_channel.channel         = device_details->listen_channel.channel;
    workspace->group_formation_timeout        = device_details->group_formation_timeout;
    workspace->device_name_length             = strlen(device_details->wps_device_details.device_name);
    memcpy( workspace->listen_channel.country_string, &device_details->listen_channel.country_string,  3 );
    memcpy( workspace->device_name, device_details->wps_device_details.device_name, workspace->device_name_length );

    if ( ( wwd_result = ( wwd_result_t ) besl_p2p_check_soft_ap_interface( workspace ) ) != WWD_SUCCESS )
    {
        if ( wwd_result != WWD_WLAN_UNSUPPORTED )
        {
            return ( besl_result_t ) wwd_result;
        }
    }

    /* Save the original STA MAC address */
    if ( ( wwd_result = ( wwd_result_t ) besl_host_get_mac_address(&workspace->original_mac_address, WWD_STA_INTERFACE ) ) != WWD_SUCCESS )
    {
        return ( besl_result_t ) wwd_result;
    }

    /* Turn roaming off for P2P */
    data = wwd_sdpcm_get_iovar_buffer(&buffer, 4, "roam_off");
    CHECK_IOCTL_BUFFER( data );
    *data = 1;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE) );

    /* Turn off MPC or action frame sequence numbers are broken as well as other things */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_MPC );
    CHECK_IOCTL_BUFFER( data );
    *data = 0;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) );

    /*  Enable discovery. This sets the p2p device address used by the firmware to be the locally administered version of the STA MAC address */
    data = wwd_sdpcm_get_iovar_buffer(&buffer, 4, "p2p_disc");
    CHECK_IOCTL_BUFFER( data );
    *data = 1;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE) );

    /* Generate P2P device (used in negotiation) and P2P interface MAC addresses (used by group client or owner) */
    memcpy(&workspace->p2p_device_address, &workspace->original_mac_address, sizeof(besl_mac_t));
    workspace->p2p_device_address.octet[0] |= 0x02;
    memcpy(&workspace->p2p_interface_address, &workspace->p2p_device_address, sizeof(besl_mac_t));
//    workspace->p2p_interface_address.octet[4] ^= 0x80;

    /* Check that we can read the p2p interface. This also appears necessary to bring it up. */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) sizeof(besl_mac_t), "p2p_if" );
    CHECK_IOCTL_BUFFER( data );
    memcpy(data, &workspace->p2p_device_address, sizeof(besl_mac_t));
    wwd_result = wwd_sdpcm_send_iovar( SDPCM_GET, buffer, &response, WWD_STA_INTERFACE );

    if ( wwd_result != WWD_SUCCESS )
    {
        BESL_DEBUG(("Unable to read p2p interface\n"));
        return ( besl_result_t )wwd_result;
    }

    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    wl_p2p_ifq_t* go_if = (wl_p2p_ifq_t*)data;
    BESL_DEBUG(("p2p interface %u, %s\n", (unsigned int)go_if->bsscfgidx, go_if->ifname));
    wwd_update_host_interface_to_bss_index_mapping( WWD_P2P_INTERFACE, go_if->bsscfgidx );

    host_buffer_release( response, WWD_NETWORK_RX );

    /* Get the P2P interface address */
    besl_host_get_mac_address(&workspace->device_info.p2p_device_address, WWD_P2P_INTERFACE);

    BESL_DEBUG(("STA MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", workspace->device_info.p2p_device_address.octet[0],
        workspace->device_info.p2p_device_address.octet[1],
        workspace->device_info.p2p_device_address.octet[2],
        workspace->device_info.p2p_device_address.octet[3],
        workspace->device_info.p2p_device_address.octet[4],
        workspace->device_info.p2p_device_address.octet[5]));

    /* Set the device details */
    workspace->wps_device_details = &device_details->wps_device_details;

    /* Allow the P2P library to initialize */
    besl_p2p_init_common(workspace, device_details);

    /* Bring up P2P interface */
    CHECK_IOCTL_BUFFER( wwd_sdpcm_get_ioctl_buffer( &buffer, 0 ) );

    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_ioctl(SDPCM_SET, WLC_UP, buffer, NULL, WWD_P2P_INTERFACE) );

    /* Set wsec to WPA2-AES in the discovery bsscfg to ensure our P2P probe responses have the privacy bit set in the 802.11 WPA IE.
     * Some peer devices may not initiate WPS with us if this bit is not set. */
    data = wwd_sdpcm_get_iovar_buffer(&buffer, 8, "bsscfg:" IOVAR_STR_WSEC);
    CHECK_IOCTL_BUFFER( data );
    data[0] = wwd_get_bss_index( WWD_P2P_INTERFACE );
    data[1] = WICED_SECURITY_WPA2_AES_PSK;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE) );

    CHECK_RETURN( (besl_result_t) wwd_management_set_event_handler( p2p_discovery_events, p2p_event_handler, workspace, WWD_P2P_INTERFACE ) );

    /* Create the P2P thread */
    p2p_thread_start( workspace );

    return BESL_SUCCESS;
}

wiced_bool_t besl_p2p_check_valid_channel( uint8_t* band_a, uint8_t* band_b, uint8_t channel )
{
    int i;

    if ( channel > WIFI_BAND_B_LAST_CHANNEL )
    {
        for ( i = 0; band_a[i] != 0; i++ )
        {
            if ( band_a[i] == channel )
            {
                return WICED_TRUE;
            }
        }
    }
    else
    {
        for ( i = 0; band_b[i] != 0; i++ )
        {
            if( band_b[i] == channel )
            {
                return WICED_TRUE;
            }
        }
    }

    return WICED_FALSE;
}

wiced_bool_t besl_p2p_set_channel_list_tlv_info( p2p_workspace_t* workspace, p2p_channel_list_tlv_t* p2p_channel_list_tlv, p2p_channel_list_t* p2p_chan_list )
{
    uint8_t chan_info[P2P_MAX_CHANNEL_LIST_ELEMENT] = {0, };
    int p = 0, class_idx, length;

    length = ( p2p_channel_list_tlv->total_classes ) * 2 + p2p_channel_list_tlv->total_channels;

    /*  Cut off operating classes which exceeds P2P_MAX_CHANNEL_LIST_ELEMENT  */
    if ( length > P2P_MAX_CHANNEL_LIST_ELEMENT )
    {
        uint8_t tlv_size = 0;

        for ( class_idx = 0; tlv_size <= P2P_MAX_CHANNEL_LIST_ELEMENT && class_idx < p2p_channel_list_tlv->total_classes; class_idx++ )
        {
            tlv_size += p2p_chan_list[class_idx].number_of_channels + 2;
        }

        p2p_channel_list_tlv->total_classes  = class_idx - 1;
        p2p_channel_list_tlv->total_channels = tlv_size - p2p_chan_list[class_idx].number_of_channels - class_idx * 2;

        length = p2p_channel_list_tlv->total_classes * 2 + p2p_channel_list_tlv->total_channels;
        p2p_channel_list_tlv->length = length + sizeof(workspace->channel_list.country_string);
    }

    for ( int i = 0; i < p2p_channel_list_tlv->total_classes; i++ )
    {
        chan_info[p] = p2p_chan_list[i].operating_class;
        p++;
        chan_info[p] = p2p_chan_list[i].number_of_channels;
        p++;
        memcpy( &chan_info[p], p2p_chan_list[i].channel_list, p2p_chan_list[i].number_of_channels);
        p += p2p_chan_list[i].number_of_channels;
    }

    memcpy( p2p_channel_list_tlv->p2p_chan_list.p2p_chan_list_tlv, chan_info, length );

    return WICED_TRUE;
}

besl_result_t besl_p2p_set_channel_list( p2p_workspace_t* workspace, const besl_p2p_device_detail_t* device_details )
{
    uint8_t*                buffer = NULL;
    size_t                  size = 0;
    wl_uint32_list_t*       chan_list = NULL;
    wiced_bool_t            found_op_class = WICED_FALSE;
    uint8_t                 total_chan_num = 0, total_class_num = 0;
    uint8_t*                channel_list_element;
    p2p_channel_list_t      p2p_channel_list[P2P_MAX_OP_CLASSES];
    p2p_channel_list_tlv_t* p2p_channel_list_tlv;
    uint8_t                 band_b_chan[WICED_WIFI_MAX_CHANNELS] = {0, };
    uint8_t                 band_a_chan[WICED_WIFI_MAX_CHANNELS] = {0, };
    int                     a = 0, b = 0, j;

    /*  Read Channel List  */
    size = ( sizeof(uint32_t) * ( WICED_WIFI_MAX_CHANNELS + 1 ) );
    buffer = calloc( 1, size );
    if ( buffer == NULL )
    {
        WPRINT_APP_INFO( ( "Unable to allocate memory for associations list\n" ) );
        return BESL_ERROR_OUT_OF_MEMORY;
    }

    chan_list = (wl_uint32_list_t*)buffer;
    if ( wwd_wifi_get_channels( WWD_STA_INTERFACE, chan_list ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Unable to get channel list\n" ) );
        besl_host_free( buffer );
        return BESL_ABORTED;
    }

    /*  Separate channels as band  */
    for ( j = 0; j < (int)( chan_list->count ); j++ )
    {
        if( chan_list->element[j] > WIFI_BAND_B_LAST_CHANNEL )
        {
            band_a_chan[a] = chan_list->element[j];
            a++;
        }
        else
        {
            band_b_chan[b] = chan_list->element[j];
            b++;
        }
    }

    besl_host_free( buffer );

    /*  Copy p2p channel list table from device_details's p2p_channel_list_table  */
    channel_list_element = (uint8_t*)&workspace->channel_list.p2p_channel_list_table;
    size = sizeof( device_details->channel_list.p2p_channel_list_table );
    memset( &p2p_channel_list, 0, sizeof(p2p_channel_list) );
    memcpy( channel_list_element, &device_details->channel_list.p2p_channel_list_table, size );

    a = b = 0;
    while ( a < size )
    {
        if ( channel_list_element[a] == 0 )
        {
            if ( found_op_class == WICED_TRUE )
            {
                b = 0;
                total_class_num++;
                found_op_class = WICED_FALSE;
            }
            a++;
            continue;
        }

        if ( found_op_class == WICED_TRUE )
        {
            if( besl_p2p_check_valid_channel( band_a_chan, band_b_chan, channel_list_element[a] ) )
            {
                p2p_channel_list[total_class_num].channel_list[b++] = channel_list_element[a];
                p2p_channel_list[total_class_num].number_of_channels++;
                total_chan_num++;
            }
            a++;
            continue;
        }

        if ( found_op_class == WICED_FALSE )
        {
            p2p_channel_list[total_class_num].operating_class = channel_list_element[a++];
            found_op_class = WICED_TRUE;
        }
    }

    memcpy( workspace->channel_list.country_string, device_details->channel_list.country_string, sizeof(device_details->channel_list.country_string) );
    memcpy( &workspace->channel_list.p2p_channel_list, &p2p_channel_list, sizeof(p2p_channel_list) );

    p2p_channel_list_tlv = &workspace->channel_list.p2p_channel_list_tlv;
    p2p_channel_list_tlv->total_classes  = total_class_num;
    p2p_channel_list_tlv->total_channels = total_chan_num;
    p2p_channel_list_tlv->length         = total_class_num * 2 + total_chan_num + sizeof(workspace->channel_list.country_string);
    memcpy( p2p_channel_list_tlv->p2p_chan_list.country_string, workspace->channel_list.country_string, sizeof(workspace->channel_list.country_string) );

    /*  Make p2p_channel_list_tlv_data_t which is using in action frame  */
    if ( besl_p2p_set_channel_list_tlv_info( workspace, p2p_channel_list_tlv, (p2p_channel_list_t*)&p2p_channel_list ) != WICED_TRUE )
    {
        return BESL_P2P_ERROR_FAIL;
    }

    return BESL_SUCCESS;
}

besl_result_t besl_p2p_init_common( p2p_workspace_t* workspace, const besl_p2p_device_detail_t* device_details )
{
    uint8_t a;
    uint8_t random_suffix[2];

    workspace->p2p_interface                   = WWD_P2P_INTERFACE;
    workspace->device_info.config_methods      = htobe16( device_details->wps_device_details.config_methods );
    workspace->configuration_timeout           = ( device_details->client_configuration_timeout << 8 ) | device_details->go_configuration_timeout;
    workspace->p2p_capability                  = device_details->p2p_capability;
    workspace->allowed_configuration_methods   = device_details->wps_device_details.config_methods;
    memcpy(&workspace->operating_channel, &device_details->operating_channel, sizeof(p2p_channel_info_t));
    memcpy(&workspace->group_candidate.operating_channel, &device_details->operating_channel, sizeof(p2p_channel_info_t));

    workspace->device_info.number_of_secondary_devices = 0; // XXX device info should be included in device details
    workspace->device_info.primary_type.category       = 0x0a00;
    workspace->device_info.primary_type.sub_category   = 0x0500;
    workspace->device_info.primary_type.oui            = 0x04f25000;

    workspace->p2p_wps_device_password_id = device_details->device_password_id;
    workspace->peer_device_timeout = device_details->peer_device_timeout;

    if ( besl_p2p_set_channel_list( workspace, device_details ) != BESL_SUCCESS )
    {
        return BESL_P2P_ERROR_FAIL;
    }

    /* Prepare random GO SSID if necessary */
    if ( workspace->group_candidate.ssid_length == 0 )
    {
        workspace->group_candidate.ssid_length = P2P_SSID_PREFIX_LENGTH + 2;
        memcpy( workspace->group_candidate.ssid, P2P_SSID_PREFIX, P2P_SSID_PREFIX_LENGTH );
        besl_host_random_bytes(random_suffix, 2);
        for (a = 0; a < 2; ++a)
        {
            random_suffix[a] = '0' + (random_suffix[a] % 62);
            if (random_suffix[a] > '9')
            {
                random_suffix[a] += 'A' - '9' - 1;
            }
            if (random_suffix[a] > 'Z')
            {
                random_suffix[a] += 'a' - 'Z' - 1;
            }
        }
        memcpy( &workspace->group_candidate.ssid[P2P_SSID_PREFIX_LENGTH], random_suffix, 2 );
    }

    if (workspace->i_am_group_owner != 1)
    {
        /*  Add WPS IE into both the probe request and response */
        p2p_write_wps_probe_request_ie( workspace);
        p2p_write_wps_probe_response_ie( workspace);

        /*  Add P2P IE into the probe request after the WPS IE */
        p2p_write_probe_request_ie( workspace );

        /*  Add P2P IE into the probe response after the WPS IE */
        p2p_write_probe_response_ie( workspace );
    }

    return BESL_SUCCESS;
}

besl_result_t besl_p2p_deinit( p2p_workspace_t* workspace )
{
    return p2p_deinit( workspace );
}

besl_result_t besl_p2p_start( p2p_workspace_t* workspace )
{
    p2p_message_t message;

    message.type = P2P_EVENT_START_REQUESTED;
    message.data = NULL;
    p2p_host_send_message( &message, WICED_WAIT_FOREVER );
    /* Delay to allow the p2p thread to start and run */
    host_rtos_delay_milliseconds( 10 );

    return BESL_SUCCESS;
}


besl_result_t besl_p2p_listen_start( p2p_workspace_t* workspace )
{
    p2p_message_t message;
    uint8_t channel;

    channel = workspace->listen_channel.channel;

    if ( ( channel == 1 ) || ( channel == 6 ) || ( channel == 11 ) )
    {
        message.type = P2P_EVENT_START_LISTEN_DISCOVERY;
        message.data = NULL;
        p2p_host_send_message( &message, WICED_WAIT_FOREVER );
        /* Delay to allow the p2p thread to start and run */
        host_rtos_delay_milliseconds( 10 );
    }
    else
    {
        WPRINT_APP_INFO(( "Listen channel must be 1, 6 or 11\n"));
    }

    return BESL_SUCCESS;
}

p2p_discovered_device_t* besl_p2p_host_find_device(p2p_workspace_t* workspace, const besl_mac_t* mac)
{
    int a = 0;
    p2p_discovered_device_t* device = NULL;

    /* Return a pointer to an existing device, or an available entry in the table, or NULL */
    while ( a < P2P_MAX_DISCOVERED_DEVICES )
    {
        if ( ( workspace->discovered_devices[a].status != P2P_DEVICE_INVALID ) &&
             ( memcmp( &workspace->discovered_devices[a].p2p_device_address, mac, sizeof(besl_mac_t) ) == 0 ) )
        {
            return &workspace->discovered_devices[a];
        }
        if ( workspace->discovered_devices[a].status == P2P_DEVICE_INVALID )
        {
            device = &workspace->discovered_devices[a];
        }
        a++;
    }

    return device;
}


// XXX need to change this function so it accepts a destination MAC address as one of the arguments. May also need to know action frame type and sub-type.
besl_result_t besl_p2p_send_action_frame( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, p2p_action_frame_writer_t writer, uint32_t channel, uint32_t dwell_time )
{
    wiced_buffer_t  buffer;
    wl_af_params_t* frame;
    p2p_message_t   message;
    besl_result_t   result;

    workspace->current_channel = channel;

    uint32_t* a = wwd_sdpcm_get_iovar_buffer( &buffer, sizeof(wl_af_params_t) + 4, "bsscfg:" IOVAR_STR_ACTFRAME );
    CHECK_IOCTL_BUFFER( a );
    *a = wwd_get_bss_index( WWD_P2P_INTERFACE );
    frame = (wl_af_params_t*) ( a + 1 );
    frame->channel    = channel;
    frame->dwell_time = dwell_time;

    memcpy( &frame->action_frame.da, &device->p2p_device_address, sizeof(besl_mac_t) );

    /*
     * When communication is not within a P2P Group, e.g. during Service Discovery, P2P Invitation, GO Negotiation and Device Discoverability,
     * a P2P Device shall use the P2P Device Address of the intended destination as the BSSID in Request, or Confirmation frames and its own
     *  P2P Device Address as the BSSID in Response frames.
    */
    if ( workspace->group_owner_is_up == 0 )
    {
        memcpy( &frame->BSSID, &frame->action_frame.da, 6 );
    }
    else
    {
        memcpy( &frame->BSSID, &workspace->p2p_interface_address, 6 );
    }

    uint8_t* end_of_data = writer( workspace, device, frame->action_frame.data );

    frame->action_frame.len      = end_of_data - frame->action_frame.data;
    frame->action_frame.packetId = htobe32(workspace->p2p_action_frame_cookie); // It comes back in reverse order in the event that occurs when the packet is ACKed
    ++workspace->p2p_action_frame_cookie;

    message.type = P2P_EVENT_PACKET_TO_BE_SENT;
    message.data = buffer;
    result = p2p_host_send_message( &message, WICED_WAIT_FOREVER );

    if ( result != BESL_SUCCESS)
    {
        return result;
    }
    return BESL_SUCCESS;
}

besl_result_t besl_p2p_start_registrar( void )
{
    p2p_message_t   message;
    besl_result_t    result;

    message.type = P2P_EVENT_START_REGISTRAR;
    message.data = NULL;
    result = p2p_host_send_message( &message, WICED_WAIT_FOREVER );

    if ( result != BESL_SUCCESS)
    {
        return result;
    }

    return BESL_SUCCESS;
}

void besl_p2p_host_negotiation_complete( p2p_workspace_t* workspace )
{
    p2p_message_t message;
    message.type = P2P_EVENT_NEGOTIATION_COMPLETE;
    message.data = NULL;
    p2p_host_send_message( &message, WICED_WAIT_FOREVER );
}

besl_result_t besl_p2p_get_group_formation_progress( p2p_workspace_t* workspace )
{
    return workspace->p2p_result;
}

besl_result_t besl_p2p_go_get_client_wps_progress( p2p_workspace_t* workspace )
{
    if (workspace->p2p_current_state == P2P_STATE_GROUP_OWNER)
    {
        switch (workspace->p2p_wps_agent->wps_result)
        {
            case WPS_NOT_STARTED: return BESL_P2P_GROUP_OWNER_WAITING_FOR_CONNECTION;
            case WPS_IN_PROGRESS: return BESL_P2P_GROUP_OWNER_WPS_IN_PROGRESS;
            case WPS_COMPLETE:    return BESL_P2P_GROUP_OWNER_WPS_COMPLETE;
            case WPS_PBC_OVERLAP: return BESL_P2P_GROUP_OWNER_WPS_PBC_OVERLAP;
            case WPS_TIMEOUT:     return BESL_P2P_GROUP_OWNER_WPS_TIMED_OUT;
            case WPS_ABORTED:     return BESL_P2P_GROUP_OWNER_WPS_ABORTED;

            default:
                return BESL_P2P_UNKNOWN;
        }
    }

    return workspace->p2p_result;
}

besl_result_t besl_p2p_start_negotiation( p2p_workspace_t* workspace )
{
    besl_result_t  result = BESL_SUCCESS;
    p2p_message_t message;

    message.type = P2P_EVENT_START_NEGOTIATION;
    message.data = NULL;
    p2p_host_send_message( &message, WICED_NO_WAIT );

    return result;
}

besl_result_t besl_p2p_find_group_owner( p2p_workspace_t* workspace )
{
    besl_result_t  result = BESL_SUCCESS;
    p2p_message_t message;

    message.type = P2P_EVENT_FIND_GROUP_OWNER;
    message.data = NULL;
    p2p_host_send_message( &message, WICED_WAIT_FOREVER );

    return result;
}

besl_result_t besl_p2p_get_discovered_peers( p2p_workspace_t* workspace, p2p_discovered_device_t** devices, uint8_t* device_count )
{
    *devices = workspace->discovered_devices;
    *device_count = workspace->discovered_device_count;

    return BESL_SUCCESS;
}

besl_result_t besl_p2p_group_owner_start( p2p_workspace_t* workspace )
{
    uint32_t*             data;
    besl_result_t         result;
    wiced_buffer_t        buffer;
    wiced_buffer_t        response;
    uint8_t               pmk[DOT11_PMK_LEN + 8]; /* PMK storage must be 40 octets in length for use in the function that converts from passphrase to pmk */
    uint8_t               security_key[64];
    wsec_pmk_t*           psk;
    uint32_t              bss_index;

    BESL_DEBUG( ("Group owner start entry\n") );

    if ( ( result = besl_p2p_check_soft_ap_interface( workspace ) ) != BESL_SUCCESS )
    {
        if ( result != (besl_result_t)WWD_WLAN_UNSUPPORTED )
        {
            return result;
        }
    }

    /* Clear the p2p event queue before bringing up group owner */
    if ( workspace->p2p_thread_running == 1 )
    {
        p2p_clear_event_queue();
    }

    /* Re-init some parts of the p2p workspace since we may be coming from group negotiation */
    memset( workspace->discovered_devices, 0, sizeof(p2p_discovered_device_t) * P2P_MAX_DISCOVERED_DEVICES );
    workspace->discovered_device_count = 0;

    /* Generate a random passphrase if necessary */
    if ( workspace->p2p_passphrase_length == 0 )
    {
        workspace->p2p_passphrase_length = 8;
        besl_802_11_generate_random_passphrase( (char *)workspace->p2p_passphrase, workspace->p2p_passphrase_length );
    }

    if ( workspace->p2p_passphrase_length != 64 )
    {
        memset(pmk, 0, DOT11_PMK_LEN + 8);
        if ( besl_802_11_generate_pmk( (char *)workspace->p2p_passphrase, (unsigned char *)workspace->group_candidate.ssid, workspace->group_candidate.ssid_length, (unsigned char*)pmk ) != BESL_SUCCESS )
        {
            return BESL_P2P_ERROR_FAIL;
        }
        besl_host_hex_bytes_to_chars( (char*)security_key, pmk, DOT11_PMK_LEN );
    }
    else
    {
        memcpy( security_key, workspace->p2p_passphrase, 64 );
    }

    /* Save the original STA MAC address */
    if ( ( result = besl_host_get_mac_address(&workspace->original_mac_address, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        return result;
    }

    /* Create device address */
    memcpy(&workspace->p2p_device_address, &workspace->original_mac_address, sizeof(besl_mac_t));
    workspace->p2p_device_address.octet[0] |= 0x02;
    memcpy( &workspace->group_candidate.p2p_device_address, &workspace->p2p_device_address, sizeof(besl_mac_t) );
    memcpy( &workspace->group_candidate.bssid, &workspace->p2p_device_address, sizeof(besl_mac_t) );

    /* Create p2p interface address */
    memcpy(&workspace->p2p_interface_address, &workspace->p2p_device_address, sizeof(besl_mac_t));
    //workspace->p2p_interface_address.octet[4] ^= 0x80;

    /* Turn roaming off for P2P */
    data = wwd_sdpcm_get_iovar_buffer(&buffer, 4, "roam_off");
    CHECK_IOCTL_BUFFER( data );
    *data = 1;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar(SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE) );

    /* Turn off MPC or action frame sequence numbers are broken as well as other things */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, IOVAR_STR_MPC );
    CHECK_IOCTL_BUFFER( data );
    *data = 0;
    CHECK_RETURN( (besl_result_t) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) );

    /* If the group owner is created as a result of negotiation then set the operating channel from the negotiation phase,
     * otherwise use the pre-configured operating channel.
     */
    if ( workspace->p2p_thread_running == 1 )
    {
        memcpy( &workspace->operating_channel, &workspace->group_candidate.operating_channel, sizeof(p2p_channel_info_t) );
    }
    /* Make sure the current channel is the operating channel. XXX we may remove current channel later */
    workspace->current_channel = (uint32_t) workspace->operating_channel.channel;

    /* Create P2P interface */
    wl_p2p_if_t* p2p_if = wwd_sdpcm_get_iovar_buffer( &buffer, sizeof(wl_p2p_if_t), "p2p_ifadd" );
    CHECK_IOCTL_BUFFER( p2p_if );
    p2p_if->interface_type = P2P_GROUP_OWNER_MODE;
    p2p_if->chan_spec = workspace->operating_channel.channel | WL_CHANSPEC_BAND_2G | WL_CHANSPEC_BW_20 | WL_CHANSPEC_CTL_SB_NONE;

    memcpy( &p2p_if->mac_address, &workspace->p2p_interface_address, sizeof(besl_mac_t) );
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, NULL, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("p2p_ifadd fail\n"));
        return result;
    }

    /* Once past this point call p2p_deinit() before returning an error, unless it's not possible to get an IOCTL buffer, in which case just return an error. */
    workspace->group_owner_is_up = 1;

    /* Read the p2p interface so it comes up */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) sizeof(besl_mac_t), "p2p_if" );
    CHECK_IOCTL_BUFFER( data );
    memcpy(data, &workspace->p2p_interface_address, sizeof(besl_mac_t));
    if ( ( result = (besl_result_t) wwd_sdpcm_send_iovar( SDPCM_GET, buffer, &response, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("unable to read go p2p interface\n"));
        return result;
    }
    data = (uint32_t*) host_buffer_get_current_piece_data_pointer( response );
    if ( data == NULL )
    {
        // XXX assert something is very wrong if this happens
        return BESL_P2P_ERROR_FAIL;
    }
    wl_p2p_ifq_t* go_if = (wl_p2p_ifq_t*)data;
    BESL_DEBUG(("p2p bss index %u, %s\n", (unsigned int)go_if->bsscfgidx, go_if->ifname));
    bss_index = go_if->bsscfgidx;
    wwd_update_host_interface_to_bss_index_mapping( WWD_P2P_INTERFACE, bss_index );
    besl_host_get_mac_address(&workspace->p2p_interface_address, WWD_P2P_INTERFACE );
    BESL_DEBUG(("GO MAC: %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n", workspace->p2p_interface_address.octet[0],
            workspace->p2p_interface_address.octet[1],
            workspace->p2p_interface_address.octet[2],
            workspace->p2p_interface_address.octet[3],
            workspace->p2p_interface_address.octet[4],
            workspace->p2p_interface_address.octet[5]));

    host_buffer_release( response, WWD_NETWORK_RX );

    /* Bring up P2P interface */
    CHECK_IOCTL_BUFFER( wwd_sdpcm_get_ioctl_buffer(&buffer, 0 ) );
    if ( ( result = ( besl_result_t )wwd_sdpcm_send_ioctl(SDPCM_SET, WLC_UP, buffer, NULL, WWD_P2P_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to read p2p interface\n"));
        return result;
    }

    /* Set the SSID */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 40, "bsscfg:" IOVAR_STR_SSID );
    CHECK_IOCTL_BUFFER( data );
    data[0] = bss_index; /* Set the bsscfg index */
    data[1] = MIN( workspace->group_candidate.ssid_length, 32 ); /* Set the ssid length */
    memcpy( &data[2], (uint8_t*)&workspace->group_candidate.ssid, data[1] );
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to set P2P SSID\n"));
        return result;
    }

    /* Set the security type */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 8, "bsscfg:" IOVAR_STR_WSEC );
    CHECK_IOCTL_BUFFER( data );
    data[0] = bss_index;
    data[1] = (uint32_t) ( ( (WICED_SECURITY_WPA2_AES_PSK | WPS_ENABLED) & ( ~WPS_ENABLED ) ) | SES_OW_ENABLED );
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to set wsec\n"));
        return result;
    }

    /* Set the wpa auth */
    data = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 8, "bsscfg:" IOVAR_STR_WPA_AUTH );
    CHECK_IOCTL_BUFFER( data );
    data[0] = bss_index;
    data[1] = (uint32_t) WPA2_AUTH_PSK;
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to set wpa auth\n"));
        return result;
    }

    /* Set the security key */
    psk = (wsec_pmk_t*) wwd_sdpcm_get_ioctl_buffer( &buffer, sizeof(wsec_pmk_t) );
    CHECK_IOCTL_BUFFER( psk );
    memcpy( psk->key, security_key, 64 );
    psk->key_len = 64;
    psk->flags   = (uint16_t) WSEC_PASSPHRASE;
    host_rtos_delay_milliseconds( 1 ); // Delay required to allow radio firmware to be ready to receive PMK and avoid intermittent failure
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_ioctl( SDPCM_SET, WLC_SET_WSEC_PMK, buffer, 0, WWD_P2P_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to set passphrase\n"));
        return result;
    }

    /* Restrict the number of associated devices */
    data   = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, "bss_maxassoc" );
    CHECK_IOCTL_BUFFER( data );
    *data  = P2P_MAX_ASSOCIATED_DEVICES;
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_P2P_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to set max associated devices\n"));
        return result;
    }

    /* Set DTIM period */
    data   = wwd_sdpcm_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data  = (uint32_t) WICED_DEFAULT_SOFT_AP_DTIM_PERIOD;
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_ioctl( SDPCM_SET, WLC_SET_DTIMPRD, buffer, 0, WWD_P2P_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to set dtim period\n"));
        return result;
    }

    uint32_t scb_timeout = 20;
    data = (uint32_t*) wwd_sdpcm_get_ioctl_buffer( &buffer, (uint16_t) sizeof(uint32_t) );
    CHECK_IOCTL_BUFFER( data );
    *data = scb_timeout;
    if ( ( result = ( besl_result_t )wwd_sdpcm_send_ioctl( SDPCM_SET, WLC_SET_SCB_TIMEOUT, buffer, 0, WWD_P2P_INTERFACE ) ) != BESL_SUCCESS)
    {
        BESL_DEBUG(("Set scb timeout fail\n"));
        return result;
    }

    /* XXX check that this call does something */
    wwd_wifi_set_block_ack_window_size( WWD_P2P_INTERFACE );

    /* Restrict the BSS Rate Set to OFDM XXX is this still necessary */
    data   = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, "bss_rateset" );
    CHECK_IOCTL_BUFFER( data );
    *data  = (uint32_t)1;
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_P2P_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to set BSS rate set\n"));
        return result;
    }


    workspace->p2p_wps_agent = besl_host_calloc( "p2p go wps agent", 1, sizeof(wps_agent_t) );
    if ( workspace->p2p_wps_agent == NULL )
    {
        return BESL_ERROR_OUT_OF_MEMORY;
    }
    memset( workspace->p2p_wps_agent, 0, sizeof(wps_agent_t) );

    workspace->p2p_wps_agent->is_p2p_registrar = 1;
    workspace->p2p_wps_agent->wps_agent_owner = workspace;
    memset(&workspace->p2p_wps_credential, 0, sizeof(besl_wps_credential_t));
    workspace->p2p_wps_credential.security = WICED_SECURITY_WPA2_AES_PSK;
    workspace->p2p_wps_credential.ssid.length = workspace->group_candidate.ssid_length;
    memcpy( workspace->p2p_wps_credential.ssid.value, workspace->group_candidate.ssid, workspace->p2p_wps_credential.ssid.length );
    workspace->p2p_wps_credential.passphrase_length = workspace->p2p_passphrase_length;
    memcpy( workspace->p2p_wps_credential.passphrase, workspace->p2p_passphrase, workspace->p2p_wps_credential.passphrase_length );
    result = ( besl_result_t ) besl_wps_init( workspace->p2p_wps_agent, workspace->wps_device_details, WPS_REGISTRAR_AGENT, WWD_P2P_INTERFACE );
    if ( result != BESL_SUCCESS )
    {
        BESL_DEBUG(("besl_p2p_group_owner_start: error besl init %d\n", result));
        return result;
    }
    workspace->p2p_wps_agent->wps_result_callback = workspace->p2p_wps_result_callback;

    workspace->p2p_capability |= P2P_GROUP_CAPABILITY_P2P_GROUP_OWNER;
    /* Check if a persistent group is being formed and if it is save the details. We don't check if it's new or not. */
    if ( workspace->form_persistent_group == 1 )
    {
        workspace->p2p_capability |= P2P_GROUP_CAPABILITY_P2P_PERSISTENT_GROUP;
        workspace->persistent_group.channel             = workspace->operating_channel.channel;
        workspace->persistent_group.security            = WICED_SECURITY_WPA2_AES_PSK;
        workspace->persistent_group.security_key_length = WSEC_MAX_PSK_LEN;
        memcpy( workspace->persistent_group.security_key, security_key, WSEC_MAX_PSK_LEN );
        workspace->persistent_group.SSID.length = workspace->group_candidate.ssid_length;
        memcpy( workspace->persistent_group.SSID.value, workspace->group_candidate.ssid, workspace->persistent_group.SSID.length );
    }
    workspace->configuration_timeout = 0;
    memcpy( &workspace->device_info.p2p_device_address, &workspace->p2p_device_address, sizeof(besl_mac_t) );
    p2p_write_beacon_ie( workspace );
    p2p_write_probe_response_ie( workspace );

    /* Bring up the BSS */
    data    = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 8, IOVAR_STR_BSS );
    CHECK_IOCTL_BUFFER( data );
    data[0] = bss_index;
    data[1] = (uint32_t) 1;
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to bring up BSS\n"));
        return result;
    }

    wwd_wifi_p2p_set_go_is_up( WICED_TRUE );

    /* Bring up IP layer on P2P interface */
    result = ( besl_result_t ) wiced_ip_up( WICED_P2P_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &p2p_ip_settings );
    if ( result != BESL_SUCCESS )
    {
        BESL_DEBUG(("Unable to bring up IP layer\n"));
        return result;
    }

    host_rtos_delay_milliseconds( 10 ); // Delay to allow the DHCP thread to run or it may not be up when moving from negotiation to group owner

    workspace->p2p_current_state = P2P_STATE_GROUP_OWNER;
    workspace->i_am_group_owner  = 1;
    workspace->p2p_result        = BESL_SUCCESS;

    /*  Add P2P event handler */
    if ( ( result = ( besl_result_t ) wwd_management_set_event_handler( p2p_group_owner_events, p2p_event_handler, workspace, WWD_P2P_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_ERROR(("Unable to set group owner event handler\n"));
        return result;
    }

    /* If the p2p thread is already running then we are here as a result of group owner negotiation and we don't want to start another thread. If the p2p thread
     * is not running then we are here as a result of starting an autonomous group owner and we do need to start the p2p thread.
     */
    if ( workspace->p2p_thread_running == 0 )
    {
        p2p_thread_start( workspace );
    }

    BESL_INFO( ( "P2P Group Owner starting. Group ID %02X:%02X:%02X:%02X:%02X:%02X %s\n",
            workspace->p2p_device_address.octet[0],
            workspace->p2p_device_address.octet[1],
            workspace->p2p_device_address.octet[2],
            workspace->p2p_device_address.octet[3],
            workspace->p2p_device_address.octet[4],
            workspace->p2p_device_address.octet[5],
            workspace->group_candidate.ssid ) );

    if ( workspace->p2p_group_formation_result_callback != NULL )
    {
        workspace->p2p_group_formation_result_callback( workspace );
    }

    return BESL_SUCCESS;
}

besl_result_t besl_p2p_client_enable_powersave( p2p_workspace_t* workspace, uint32_t power_save_mode )
{
    wiced_buffer_t buffer;
    uint32_t* data;

    /* Set legacy powersave mode - PM1 */
    data = (uint32_t*) wwd_sdpcm_get_ioctl_buffer( &buffer, (uint16_t) 4 );
    CHECK_IOCTL_BUFFER( data );
    *data = power_save_mode;
    return (besl_result_t) wwd_sdpcm_send_ioctl( SDPCM_SET, WLC_SET_PM, buffer, NULL, WWD_P2P_INTERFACE );
}

wiced_bool_t besl_p2p_group_owner_is_up( void )
{
    return wwd_wifi_p2p_is_go_up( );
}

/* Callbacks */

void besl_p2p_register_p2p_device_connection_callback( p2p_workspace_t* workspace, void (*p2p_connection_request_callback)(p2p_discovered_device_t*) )
{
    workspace->p2p_connection_request_callback = p2p_connection_request_callback;
}

void besl_p2p_register_legacy_device_connection_callback( p2p_workspace_t* workspace, void (*p2p_legacy_device_connection_request_callback)(p2p_legacy_device_t*) )
{
    workspace->p2p_legacy_device_connection_request_callback = p2p_legacy_device_connection_request_callback;
}

void besl_p2p_register_group_formation_result_callback( p2p_workspace_t* workspace, void (*p2p_group_formation_result_callback)(void*) )
{
    workspace->p2p_group_formation_result_callback = p2p_group_formation_result_callback;
}

void besl_p2p_register_wpa2_client_association_callback( p2p_workspace_t* workspace, void (*p2p_wpa2_client_association_callback)(besl_mac_t*) )
{
    workspace->p2p_wpa2_client_association_callback = p2p_wpa2_client_association_callback;
}

void besl_p2p_register_wps_enrollee_association_callback( p2p_workspace_t* workspace, void (*p2p_wps_enrollee_association_callback)(besl_mac_t*) )
{
    workspace->p2p_wps_enrollee_association_callback = p2p_wps_enrollee_association_callback;
}

void besl_p2p_register_wps_result_callback( p2p_workspace_t* workspace, void (*p2p_wps_result_callback)(wps_result_t*) )
{
    workspace->p2p_wps_result_callback = p2p_wps_result_callback;
}

void besl_p2p_register_p2p_device_disassociation_callback( p2p_workspace_t* workspace, void (*p2p_device_disassociation_callback)(besl_mac_t*) )
{
    workspace->p2p_device_disassociation_callback = p2p_device_disassociation_callback;
}

void besl_p2p_register_legacy_device_disassociation_callback( p2p_workspace_t* workspace, void (*p2p_legacy_device_disassociation_callback)(besl_mac_t*) )
{
    workspace->p2p_legacy_device_disassociation_callback = p2p_legacy_device_disassociation_callback;
}


void p2p_host_add_vendor_ie( uint32_t interface, void* data, uint16_t data_length, uint32_t packet_mask )
{
    wwd_wifi_manage_custom_ie( ( wwd_interface_t ) interface, WICED_ADD_CUSTOM_IE, (uint8_t*) P2P_OUI, P2P_OUI_SUB_TYPE, data, data_length, packet_mask );
}

void p2p_host_remove_vendor_ie( uint32_t interface, void* data, uint16_t data_length, uint32_t packet_mask )
{
    wwd_wifi_manage_custom_ie( ( wwd_interface_t ) interface, WICED_REMOVE_CUSTOM_IE, (uint8_t*) P2P_OUI, P2P_OUI_SUB_TYPE, data, data_length, packet_mask );
}

/* If the AP interface is up then return an error. If it is in existence but down then remove it. */
static besl_result_t besl_p2p_check_soft_ap_interface( p2p_workspace_t* workspace )
{
    wiced_buffer_t buffer;
    uint32_t*      data;
    besl_result_t result = BESL_SUCCESS;

    if ( wiced_network_is_up( ( wiced_interface_t ) WWD_AP_INTERFACE ) == WICED_TRUE )
    {
        BESL_DEBUG(( "Error: Soft AP already up\n" ));
        return (besl_result_t)WWD_AP_ALREADY_UP;
    }

    /* Some platforms require removal of the Soft AP interface before the GO can be created. If the IOVAR fails because it is unsupported then it is non-fatal. */
    data   = (uint32_t*) wwd_sdpcm_get_iovar_buffer( &buffer, (uint16_t) 4, "bsscfg:" IOVAR_STR_INTERFACE_REMOVE );
    CHECK_IOCTL_BUFFER( data );
    *data  = wwd_get_bss_index( WWD_AP_INTERFACE );
    if ( ( result = ( besl_result_t ) wwd_sdpcm_send_iovar( SDPCM_SET, buffer, 0, WWD_STA_INTERFACE ) ) != BESL_SUCCESS )
    {
        BESL_DEBUG( ( "Error: Unable to remove Soft AP interface %u\n", (unsigned int)result) );
    }
    return result;
}
