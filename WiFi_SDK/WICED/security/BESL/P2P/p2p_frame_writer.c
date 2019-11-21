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

#include "wps_constants.h"
#include "wps_common.h"
#include "template_wps_packets.h"
#include "wps_host_interface.h"
#include "p2p_host_interface.h"
#include "p2p_constants.h"
#include <string.h>
#include <stddef.h>
#include "p2p_frame_writer.h"

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
 *               Function Declarations
 ******************************************************/

static uint8_t* p2p_write_tlv( uint8_t* iter, uint8_t type, uint16_t length, ptrdiff_t data, tlv_data_type_t data_type );
static uint8_t  p2p_get_new_dialog_token( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const uint8_t p2p_public_action_frame_fixed_parameters[]  = { 0x04, 0x09, 0x50, 0x6f, 0x9a, 0x09 };
static const uint8_t p2p_action_frame_fixed_parameters[]         = { 0x7f, 0x50, 0x6f, 0x9a, 0x09 };

static const p2p_vendor_specific_tag_header_t template_p2p_tag_header =
{
    .tag_number        = 0xdd,
    .length            = 0,
    .oui               = {0x50, 0x6f, 0x9a},
    .specific_oui_type = P2P_OUI_TYPE,
};

static const wps_vendor_specific_tag_header_t template_wps_tag_header =
{
    .tag_number        = 0xdd,
    .length            = 0,
    .oui               = {0x00, 0x50, 0xf2},
    .specific_oui_type = WPS_OUI_TYPE,
};

static uint8_t p2p_dialog_token = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* This function is used by the WPS Registrar code */
void p2p_rewrite_group_owner_management_ies( void* wps_agent_owner )
{
    p2p_workspace_t* workspace = (p2p_workspace_t*)wps_agent_owner;

    if ( workspace != NULL )
    {
        p2p_write_beacon_ie( workspace );
        p2p_write_probe_response_ie( workspace );
    }
}

void p2p_remove_ies( p2p_workspace_t* workspace )
{
    /* Free P2P and WPS information elements */
    if ( workspace->p2p_probe_request_ie != NULL )
    {
        p2p_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_probe_request_ie,  workspace->p2p_probe_request_ie_length,  VENDOR_IE_PROBE_REQUEST );
        besl_host_free( workspace->p2p_probe_request_ie );
        workspace->p2p_probe_request_ie = NULL;
    }
    if ( workspace->p2p_probe_response_ie != NULL )
    {
        p2p_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_probe_response_ie, workspace->p2p_probe_response_ie_length, VENDOR_IE_PROBE_RESPONSE );
        besl_host_free( workspace->p2p_probe_response_ie );
        workspace->p2p_probe_response_ie = NULL;
    }
    if ( workspace->p2p_association_request_ie != NULL )
    {
        p2p_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_association_request_ie,  workspace->p2p_association_request_ie_length,  VENDOR_IE_ASSOC_REQUEST );
        besl_host_free( workspace->p2p_association_request_ie );
        workspace->p2p_association_request_ie = NULL;
    }
    if ( workspace->p2p_beacon_ie != NULL )
    {
        p2p_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_beacon_ie, workspace->p2p_beacon_ie_length, VENDOR_IE_BEACON );
        besl_host_free( workspace->p2p_beacon_ie );
        workspace->p2p_beacon_ie = NULL;
    }
    if ( workspace->wps_probe_request_ie != NULL )
    {
        wps_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->wps_probe_request_ie,  workspace->wps_probe_request_ie_length,  VENDOR_IE_PROBE_REQUEST );
        besl_host_free( workspace->wps_probe_request_ie );
        workspace->wps_probe_request_ie = NULL;
    }
    if ( workspace->wps_probe_response_ie != NULL )
    {
        wps_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->wps_probe_response_ie, workspace->wps_probe_response_ie_length, VENDOR_IE_PROBE_RESPONSE );
        besl_host_free( workspace->wps_probe_response_ie );
        workspace->wps_probe_response_ie = NULL;
    }
}

/* This P2P IE writer is used by the negotiation phase and also by the p2p client when joining */
besl_result_t p2p_write_probe_request_ie( p2p_workspace_t* workspace )
{
    if (workspace->p2p_probe_request_ie == NULL)
    {
        workspace->p2p_probe_request_ie = besl_host_calloc( "p2p probe req", 1, sizeof(p2p_probe_request_ie_t));
    }
    else
    {
        p2p_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_probe_request_ie, workspace->p2p_probe_request_ie_length, VENDOR_IE_PROBE_REQUEST );
        memset( workspace->p2p_probe_request_ie, 0, sizeof(p2p_probe_request_ie_t) );
    }
    uint8_t* iter = workspace->p2p_probe_request_ie;
    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO, 2, workspace->p2p_capability,                             TLV_UINT16);
    iter = p2p_write_tlv(iter, P2P_SEID_CHANNEL,             5, (ptrdiff_t)&workspace->listen_channel, TLV_UINT8_PTR);
    workspace->p2p_probe_request_ie_length = iter - workspace->p2p_probe_request_ie;

    p2p_host_add_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_probe_request_ie, workspace->p2p_probe_request_ie_length, VENDOR_IE_PROBE_REQUEST );
    return BESL_SUCCESS;
}

/* This WPS IE writer is for use during the negotiation phase */
besl_result_t p2p_write_wps_probe_response_ie( p2p_workspace_t* workspace )
{
    uint16_t config_methods = htobe16( workspace->device_info.config_methods );
    uint16_t device_password_id = workspace->p2p_wps_device_password_id;
    uint16_t size_of_wps_probe_response_ie = sizeof(template_wps2_probe_response_ie_t) +
                                             sizeof( tlv16_header_t ) +
                                             sizeof( vendor_ext_t ) +
                                             strlen( workspace->wps_device_details->manufacturer )  +
                                             strlen( workspace->wps_device_details->model_name )    +
                                             strlen( workspace->wps_device_details->model_number )  +
                                             strlen( workspace->wps_device_details->device_name )   +
                                             strlen( workspace->wps_device_details->serial_number ) +
                                             sizeof( wps_uuid_t );

    if (workspace->wps_probe_response_ie == NULL)
    {
        workspace->wps_probe_response_ie = besl_host_calloc( "p2p wps probe resp", 1, size_of_wps_probe_response_ie);
    }
    else
    {
        wps_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->wps_probe_response_ie, workspace->wps_probe_response_ie_length, VENDOR_IE_PROBE_RESPONSE );
        memset( workspace->wps_probe_response_ie, 0, size_of_wps_probe_response_ie );
    }

    uint8_t  version = WPS_VERSION;
    uint8_t  wps_setup_state = WPS_SCSTATE_UNCONFIGURED;
    uint8_t  response_type = WPS_MSGTYPE_ENROLLEE_INFO_ONLY;
    primary_device_type_t primary_device;
    primary_device.category     = besl_host_hton16(workspace->wps_device_details->device_category);
    primary_device.oui          = besl_host_hton32(WIFI_ALLIANCE_OUI);
    primary_device.sub_category = besl_host_hton16(workspace->wps_device_details->sub_category);
    wps_uuid_t uuid;
    memcpy(&uuid, WPS_TEMPLATE_UUID, sizeof(WPS_TEMPLATE_UUID));
    memcpy(&uuid.octet[sizeof(wps_uuid_t) - sizeof(besl_mac_t)], &workspace->device_info.p2p_device_address, sizeof(besl_mac_t));


    uint8_t* iter = workspace->wps_probe_response_ie;

    iter = tlv_write_value( iter, WPS_ID_VERSION,        WPS_ID_VERSION_S,        &version,                  TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_SC_STATE,       WPS_ID_SC_STATE_S,       &wps_setup_state,          TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_DEVICE_PWD_ID,  WPS_ID_DEVICE_PWD_ID_S,  &device_password_id,       TLV_UINT16 );
    iter = tlv_write_value( iter, WPS_ID_RESP_TYPE,      WPS_ID_RESP_TYPE_S,      &response_type,            TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_UUID_E,         WPS_ID_UUID_S,           &uuid,                     TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_MANUFACTURER,   strlen( workspace->wps_device_details->manufacturer ),  workspace->wps_device_details->manufacturer,  TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_MODEL_NAME,     strlen( workspace->wps_device_details->model_name ),    workspace->wps_device_details->model_name,    TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_MODEL_NUMBER,   strlen( workspace->wps_device_details->model_number ),  workspace->wps_device_details->model_number,  TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_SERIAL_NUM,     strlen( workspace->wps_device_details->serial_number ), workspace->wps_device_details->serial_number, TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_PRIM_DEV_TYPE,  WPS_ID_PRIM_DEV_TYPE_S,  &primary_device,           TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_DEVICE_NAME,    strlen( workspace->wps_device_details->device_name ),   workspace->wps_device_details->device_name,   TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_CONFIG_METHODS, WPS_ID_CONFIG_METHODS_S, &config_methods,           TLV_UINT16 );


    /* Add WFA Vendor Extension */
    vendor_ext_t* vendor_ext = (vendor_ext_t*) tlv_write_header( iter, WPS_ID_VENDOR_EXT, sizeof(vendor_ext_t));
    memcpy( vendor_ext->vendor_id, WFA_VENDOR_EXT_ID, 3 );
    vendor_ext->subid_version2.type      = WPS_WFA_SUBID_VERSION2;
    vendor_ext->subid_version2.length    = 1;
    vendor_ext->subid_version2.data      = WPS_VERSION2;
    iter += sizeof(tlv16_header_t) + sizeof(vendor_ext_t);

    workspace->wps_probe_response_ie_length = (iter - workspace->wps_probe_response_ie);

    wps_host_add_vendor_ie( WWD_P2P_INTERFACE, workspace->wps_probe_response_ie, workspace->wps_probe_response_ie_length, VENDOR_IE_PROBE_RESPONSE );

    return BESL_SUCCESS;
}

/* This WPS IE writer is used by the negotiation phase and also by the p2p client when joining */
besl_result_t p2p_write_wps_probe_request_ie( p2p_workspace_t* workspace )
{
    uint16_t device_password_id = workspace->p2p_wps_device_password_id;
    uint16_t config_methods = htobe16( workspace->device_info.config_methods );
    uint16_t size_of_wps_probe_request_ie = sizeof(template_wps2_probe_ie_t) +
                                            strlen( workspace->wps_device_details->manufacturer ) +
                                            strlen( workspace->wps_device_details->model_name )   +
                                            strlen( workspace->wps_device_details->model_number ) +
                                            strlen( workspace->wps_device_details->device_name )  +
                                            sizeof( wps_uuid_t );

    if (workspace->wps_probe_request_ie == NULL)
    {
        workspace->wps_probe_request_ie = besl_host_calloc( "p2p wps probe req", 1, size_of_wps_probe_request_ie);
    }
    else
    {
        wps_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->wps_probe_request_ie, workspace->wps_probe_request_ie_length, VENDOR_IE_PROBE_REQUEST );
        memset( workspace->wps_probe_request_ie, 0, size_of_wps_probe_request_ie );
    }

    uint8_t  version = WPS_VERSION;
    uint8_t  request_type = WPS_MSGTYPE_ENROLLEE_INFO_ONLY;
    uint16_t config_error = 0;
    uint16_t assoc_state = 0;
    uint8_t  rf_band   = 1;
    primary_device_type_t primary_device;
    primary_device.category     = besl_host_hton16(workspace->wps_device_details->device_category);
    primary_device.oui          = besl_host_hton32(WIFI_ALLIANCE_OUI);
    primary_device.sub_category = besl_host_hton16(workspace->wps_device_details->sub_category);
    wps_uuid_t uuid;
    memcpy(&uuid, WPS_TEMPLATE_UUID, sizeof(WPS_TEMPLATE_UUID));
    memcpy(&uuid.octet[sizeof(wps_uuid_t) - sizeof(besl_mac_t)], &workspace->device_info.p2p_device_address, sizeof(besl_mac_t));


    uint8_t* iter = workspace->wps_probe_request_ie;

    iter = tlv_write_value( iter, WPS_ID_VERSION,        WPS_ID_VERSION_S,        &version,          TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_REQ_TYPE,       WPS_ID_REQ_TYPE_S,       &request_type,     TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_CONFIG_METHODS, WPS_ID_CONFIG_METHODS_S, &config_methods,   TLV_UINT16 );

    iter = tlv_write_value( iter, WPS_ID_UUID_E, WPS_ID_UUID_S, &uuid, TLV_UINT8_PTR );

    iter = tlv_write_value( iter, WPS_ID_PRIM_DEV_TYPE, WPS_ID_PRIM_DEV_TYPE_S, &primary_device,     TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_RF_BAND,       WPS_ID_RF_BAND_S,       &rf_band,            TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_ASSOC_STATE,   WPS_ID_ASSOC_STATE_S,   &assoc_state,        TLV_UINT16 );
    iter = tlv_write_value( iter, WPS_ID_CONFIG_ERROR,  WPS_ID_CONFIG_ERROR_S,  &config_error,       TLV_UINT16 );
    iter = tlv_write_value( iter, WPS_ID_DEVICE_PWD_ID, WPS_ID_DEVICE_PWD_ID_S, &device_password_id, TLV_UINT16 );

    /* Manufacturer, Model Name, Model Number, Device Name */
    iter = tlv_write_value( iter, WPS_ID_MANUFACTURER, strlen( workspace->wps_device_details->manufacturer ), workspace->wps_device_details->manufacturer, TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_MODEL_NAME,   strlen( workspace->wps_device_details->model_name ),   workspace->wps_device_details->model_name,   TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_MODEL_NUMBER, strlen( workspace->wps_device_details->model_number ), workspace->wps_device_details->model_number, TLV_UINT8_PTR );
    iter = tlv_write_value( iter, WPS_ID_DEVICE_NAME,  strlen( workspace->wps_device_details->device_name ),  workspace->wps_device_details->device_name,  TLV_UINT8_PTR );

    /* Add WFA Vendor Extension */
    vendor_ext_t* vendor_ext = (vendor_ext_t*) tlv_write_header( iter, WPS_ID_VENDOR_EXT, sizeof(vendor_ext_t));
    memcpy( vendor_ext->vendor_id, WFA_VENDOR_EXT_ID, 3 );
    vendor_ext->subid_version2.type      = WPS_WFA_SUBID_VERSION2;
    vendor_ext->subid_version2.length    = 1;
    vendor_ext->subid_version2.data      = WPS_VERSION2;
    iter += sizeof(tlv16_header_t) + sizeof(vendor_ext_t);

    workspace->wps_probe_request_ie_length = (iter - workspace->wps_probe_request_ie);

    wps_host_add_vendor_ie( WWD_P2P_INTERFACE, workspace->wps_probe_request_ie, workspace->wps_probe_request_ie_length, VENDOR_IE_PROBE_REQUEST );

    return BESL_SUCCESS;
}


/* This P2P IE writer is used by the negotiation phase and the group owner. */
besl_result_t p2p_write_probe_response_ie( p2p_workspace_t* workspace )
{
    int i = 0;
    uint16_t size_of_p2p_probe_ie = sizeof(p2p_probe_response_ie_t) +
                                    workspace->device_name_length +
                                    sizeof( p2p_group_info_t ) +
                                    P2P_MAX_ASSOCIATED_DEVICES * ( sizeof(p2p_client_info_descriptor_t ) + 1 );

    /* Allocate all the memory we will ever need for this */
    if ( workspace->p2p_probe_response_ie == NULL)
    {
        workspace->p2p_probe_response_ie = besl_host_calloc("p2p probe response", 1, size_of_p2p_probe_ie);
    }
    else
    {
        p2p_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_probe_response_ie, workspace->p2p_probe_response_ie_length, VENDOR_IE_PROBE_RESPONSE );
        memset( workspace->p2p_probe_response_ie, 0, size_of_p2p_probe_ie );
    }


    uint8_t* iter = workspace->p2p_probe_response_ie;
    uint16_t aligned_config_methods;

    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO, 2, workspace->p2p_capability,                            TLV_UINT16);

    p2p_tlv_data_t* temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_DEVICE_INFO,         sizeof(p2p_device_info_t),      (ptrdiff_t)&workspace->device_info, TLV_UINT8_PTR);
    iter = tlv_write_value(iter, WPS_ID_DEVICE_NAME,         strlen(workspace->device_name), &workspace->device_name, TLV_UINT8_PTR);
    temp_header->length = iter - temp_header->data;

    if ( workspace->associated_p2p_device_count > 0 )
    {
        temp_header = (p2p_tlv_data_t*)iter;

        p2p_group_info_t p2p_group_info;
        p2p_group_info.type = P2P_SEID_GROUP_INFO;
        iter = p2p_write_tlv(iter, P2P_SEID_GROUP_INFO,       0,      (ptrdiff_t)&p2p_group_info, TLV_UINT8_PTR);

        for ( i = 0; i < P2P_MAX_ASSOCIATED_DEVICES; i++ )
        {
            if ( workspace->associated_p2p_devices[i].length > 0 )
            {
                *iter = workspace->associated_p2p_devices[i].length;
                iter++;
                memcpy( iter, (uint8_t*)&workspace->associated_p2p_devices[i].p2p_device_address, 6 );
                iter += 6;
                memcpy( iter, (uint8_t*)&workspace->associated_p2p_devices[i].p2p_interface_address, 6 );
                iter += 6;
                *iter = workspace->associated_p2p_devices[i].device_capability;
                iter++;
                aligned_config_methods = workspace->associated_p2p_devices[i].config_methods;
                memcpy( iter, (uint8_t*)&aligned_config_methods, 2 );
                iter += 2;
                memcpy( iter, (uint8_t*)&workspace->associated_p2p_devices[i].primary_type, 8 );
                iter += 8;
                *iter = 0;
                iter++;
                memcpy( iter, (uint8_t*)&workspace->associated_p2p_devices[i].device_name, 4 );
                iter += 4;
                memcpy( iter, &workspace->associated_p2p_devices[i].device_name_data, htobe16(workspace->associated_p2p_devices[i].device_name.length) ); // XXX htobe16
                iter += htobe16( workspace->associated_p2p_devices[i].device_name.length ); // XXX htobe16
            }
        }
        temp_header->length = iter - temp_header->data;
    }

    workspace->p2p_probe_response_ie_length = ( iter - workspace->p2p_probe_response_ie ); // Need to bound this
    p2p_host_add_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_probe_response_ie, workspace->p2p_probe_response_ie_length, VENDOR_IE_PROBE_RESPONSE );

    return BESL_SUCCESS;
}

besl_result_t p2p_write_beacon_ie( p2p_workspace_t* workspace )
{
    if ( workspace->p2p_beacon_ie == NULL )
    {
        workspace->p2p_beacon_ie = besl_host_calloc( "p2p beacon", 1, sizeof(p2p_beacon_ie_t) );
    }
    else
    {
        p2p_host_remove_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_beacon_ie, workspace->p2p_beacon_ie_length, VENDOR_IE_BEACON );
        memset( workspace->p2p_beacon_ie, 0, sizeof(p2p_beacon_ie_t) );
    }
    uint8_t* iter = workspace->p2p_beacon_ie;

    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO, 2, workspace->p2p_capability,           TLV_UINT16);
    iter = p2p_write_tlv(iter, P2P_SEID_DEVICE_ID,           6, (ptrdiff_t)&workspace->device_info.p2p_device_address, TLV_UINT8_PTR);

    workspace->p2p_beacon_ie_length = iter - workspace->p2p_beacon_ie;

    p2p_host_add_vendor_ie(WWD_P2P_INTERFACE, workspace->p2p_beacon_ie, workspace->p2p_beacon_ie_length, VENDOR_IE_BEACON);

    return BESL_SUCCESS;
}

besl_result_t p2p_write_association_request_ie( p2p_workspace_t* workspace )
{
    if ( workspace->p2p_association_request_ie == NULL)
    {
        workspace->p2p_association_request_ie = besl_host_calloc( "p2p assoc req", 1, sizeof(p2p_probe_response_ie_t) + workspace->device_name_length);
    }
    else
    {
        p2p_host_remove_vendor_ie(WWD_P2P_INTERFACE, workspace->p2p_association_request_ie, workspace->p2p_association_request_ie_length, VENDOR_IE_ASSOC_REQUEST);
        memset( workspace->p2p_association_request_ie, 0, workspace->p2p_association_request_ie_length );
    }

    uint8_t* iter = workspace->p2p_association_request_ie;
    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO, 2, workspace->p2p_capability,                            TLV_UINT16);

    p2p_tlv_data_t* temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_DEVICE_INFO,         sizeof(p2p_device_info_t),      (ptrdiff_t)&workspace->device_info, TLV_UINT8_PTR);
    iter = tlv_write_value(iter, WPS_ID_DEVICE_NAME,         strlen(workspace->device_name), &workspace->device_name, TLV_UINT8_PTR);
    temp_header->length = iter - temp_header->data;
    workspace->p2p_association_request_ie_length = iter - workspace->p2p_association_request_ie;

    p2p_host_add_vendor_ie( WWD_P2P_INTERFACE, workspace->p2p_association_request_ie, workspace->p2p_association_request_ie_length, VENDOR_IE_ASSOC_REQUEST );

    return BESL_SUCCESS;
}

uint8_t* p2p_write_invitation_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    p2p_tlv_data_t* temp_header;
    uint8_t* iter                = buffer;
    uint16_t channel_list_length;

    BESL_DEBUG(("Sending Invitation Request\r\n"));

    p2p_channel_list_tlv_t *p2p_channel_list_tlv = &workspace->channel_list.p2p_channel_list_tlv;
    channel_list_length = p2p_channel_list_tlv->length;

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_INVITATION_REQUEST;
    fixed_parameters->p2p_dialog_token = p2p_get_new_dialog_token();
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);

    iter = p2p_write_tlv(iter, P2P_SEID_CONFIGURATION_TIMEOUT,  2,                               workspace->configuration_timeout,                         TLV_UINT16);
    iter = p2p_write_tlv(iter, P2P_SEID_OP_CHANNEL,             sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->operating_channel,                  TLV_UINT8_PTR);
    iter = p2p_write_tlv(iter, P2P_SEID_GROUP_BSSID,            6,                               (ptrdiff_t)&workspace->group_candidate.bssid,              TLV_UINT8_PTR);
    iter = p2p_write_tlv(iter, P2P_SEID_CHANNEL_LIST,           channel_list_length,             (ptrdiff_t)&p2p_channel_list_tlv->p2p_chan_list,          TLV_UINT8_PTR);

    temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_DEVICE_INFO,            sizeof(p2p_device_info_t),       (ptrdiff_t)&workspace->device_info,                        TLV_UINT8_PTR);
    iter = tlv_write_value(iter, WPS_ID_DEVICE_NAME,            strlen(workspace->device_name),  &workspace->device_name,                                  TLV_UINT8_PTR);
    temp_header->length = iter - temp_header->data;

    temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_GROUP_ID,               sizeof(besl_mac_t),              (ptrdiff_t)&workspace->group_candidate.p2p_device_address, TLV_UINT8_PTR);
    iter = MEMCAT(iter, workspace->group_candidate.ssid, workspace->group_candidate.ssid_length);
    temp_header->length = iter - temp_header->data;

    iter = p2p_write_tlv(iter, P2P_SEID_INVITATION_FLAGS,       1,                               workspace->invitation_flags,                              TLV_UINT8);

    p2p_header->length = (uint8_t)(iter - p2p_header->oui);

    workspace->sent_invitation_request = 1;

    return iter;
}

uint8_t* p2p_write_invitation_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    uint8_t* iter                = buffer;
    uint8_t  status              = P2P_STATUS_UNKNOWN_P2P_GROUP;
    uint16_t channel_list_length;
    p2p_channel_list_tlv_t* p2p_channel_list_tlv;

    BESL_DEBUG(("Sending Invitation response\r\n"));

    /* If invitation procedure is supported then reply with status successful. It is then up to the application to authorize joining the group. */
    if ( workspace->p2p_capability & P2P_DEVICE_CAPABILITY_INVITATION_PROCEDURE )
    {
        status = P2P_STATUS_SUCCESS;
    }

    if ( device->status == P2P_DEVICE_NO_COMMON_CHANNEL )
    {
        status = P2P_STATUS_NO_COMMON_CHANNEL;
        p2p_channel_list_tlv = &workspace->channel_list.p2p_channel_list_tlv;
    }
    else
    {
        p2p_channel_list_tlv = &workspace->common_channel_list.p2p_channel_list_tlv;
    }
    channel_list_length = p2p_channel_list_tlv->length;

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_INVITATION_RESPONSE;
    fixed_parameters->p2p_dialog_token = device->dialog_token;
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);

    iter = p2p_write_tlv(iter, P2P_SEID_STATUS,                     1,                               status,                                                       TLV_UINT8);
    /* Check if we are to be group owner or the invitation candidate is to be group owner */
    if ( memcmp( &workspace->invitation_candidate.p2p_device_address, &workspace->p2p_device_address, sizeof( besl_mac_t ) ) == 0 )
    {
        iter = p2p_write_tlv(iter, P2P_SEID_CONFIGURATION_TIMEOUT,  2,                               0,                                                            TLV_UINT16);
        /* If we already the group owner then use the current operating channel, otherwise use the candidate operating channel */
        if ( workspace->p2p_current_state == P2P_STATE_GROUP_OWNER )
        {
            iter = p2p_write_tlv(iter, P2P_SEID_OP_CHANNEL,             sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->operating_channel,                  TLV_UINT8_PTR);
        }
        else
        {
            iter = p2p_write_tlv(iter, P2P_SEID_OP_CHANNEL,             sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->group_candidate.operating_channel,      TLV_UINT8_PTR);
        }
        iter = p2p_write_tlv(iter, P2P_SEID_GROUP_BSSID,            6,                               (ptrdiff_t)&workspace->p2p_interface_address,                  TLV_UINT8_PTR);
    }
    else
    {
        iter = p2p_write_tlv(iter, P2P_SEID_CONFIGURATION_TIMEOUT,  2,                               workspace->configuration_timeout,                             TLV_UINT16);
        iter = p2p_write_tlv(iter, P2P_SEID_OP_CHANNEL,             sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->invitation_candidate.operating_channel, TLV_UINT8_PTR);
        iter = p2p_write_tlv(iter, P2P_SEID_GROUP_BSSID,            6,                               (ptrdiff_t)&workspace->invitation_candidate.bssid,             TLV_UINT8_PTR);
    }
    iter = p2p_write_tlv(iter, P2P_SEID_CHANNEL_LIST,               channel_list_length,             (ptrdiff_t)&p2p_channel_list_tlv->p2p_chan_list,            TLV_UINT8_PTR);

    p2p_header->length = (uint8_t)(iter - p2p_header->oui);
    return iter;
}

uint8_t* p2p_write_provision_discovery_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    p2p_tlv_data_t* temp_header;
    uint8_t         version = 0x10;
    uint8_t*        iter    = buffer;

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_PROVISION_DISCOVERY_REQUEST;
    fixed_parameters->p2p_dialog_token = p2p_get_new_dialog_token();
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);

    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO, 2,                              workspace->p2p_capability,         TLV_UINT16);

    temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_DEVICE_INFO,         sizeof(p2p_device_info_t),      (ptrdiff_t)&workspace->device_info, TLV_UINT8_PTR);
    iter = tlv_write_value(iter, WPS_ID_DEVICE_NAME,         strlen(workspace->device_name), &workspace->device_name,           TLV_UINT8_PTR);
    temp_header->length = iter - temp_header->data;

    temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_GROUP_ID, sizeof(besl_mac_t), (ptrdiff_t)&workspace->group_candidate.p2p_device_address,        TLV_UINT8_PTR);
    iter = MEMCAT(iter, workspace->group_candidate.ssid, workspace->group_candidate.ssid_length);
    temp_header->length = iter - temp_header->data;

    /*  End of P2P vendor specific tag */
    p2p_header->length  = (iter - p2p_header->oui);

    /*  Write WPS vendor specific tag */
    wps_vendor_specific_tag_header_t* wps_header = (wps_vendor_specific_tag_header_t*)iter;
    memcpy(wps_header, &template_wps_tag_header, sizeof(wps_vendor_specific_tag_header_t));
    iter += sizeof(wps_vendor_specific_tag_header_t);
    iter = tlv_write_value( iter, WPS_ID_VERSION,        WPS_ID_VERSION_S,        &version,                               TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_CONFIG_METHODS, WPS_ID_CONFIG_METHODS_S, &workspace->provisioning_config_method, TLV_UINT16 );
    wps_header->length = (iter - wps_header->oui);

    workspace->sent_provision_discovery_request = 1;

    return iter;
}

uint8_t* p2p_write_provision_discovery_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    uint8_t* iter                 = buffer;
    uint8_t  version              = 0x10;
    uint16_t configuration_method = 0;

    /* Determine whether we accept the request or not */
    if ( workspace->allowed_configuration_methods & device->preferred_config_method )
    {
        configuration_method = device->preferred_config_method;
    }

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_PROVISION_DISCOVERY_RESPONSE;
    fixed_parameters->p2p_dialog_token = device->dialog_token;
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write WPS vendor specific tag */
    wps_vendor_specific_tag_header_t* wps_header = (wps_vendor_specific_tag_header_t*)iter;
    memcpy(wps_header, &template_wps_tag_header, sizeof(wps_vendor_specific_tag_header_t));
    iter += sizeof(wps_vendor_specific_tag_header_t);
    iter = tlv_write_value( iter, WPS_ID_VERSION,        WPS_ID_VERSION_S,        &version,              TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_CONFIG_METHODS, WPS_ID_CONFIG_METHODS_S, &configuration_method, TLV_UINT16 );
    wps_header->length = (iter - wps_header->oui);

    return iter;
}

/* Public action frame writers */
uint8_t* p2p_write_negotiation_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    p2p_tlv_data_t* temp_header;
    uint8_t  version = 0x10;
    uint8_t* iter    = buffer;
    uint16_t device_password_id = workspace->p2p_wps_device_password_id;
    uint16_t channel_list_length;

    p2p_channel_list_tlv_t *p2p_channel_list_tlv = &workspace->channel_list.p2p_channel_list_tlv;
    channel_list_length = p2p_channel_list_tlv->length;

    /* Toggle tie breaker every time we send a negotiation request */
    workspace->group_owner_tie_breaker = (~workspace->group_owner_tie_breaker) & 1;

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_GO_NEGOTIATION_REQUEST;
    fixed_parameters->p2p_dialog_token = p2p_get_new_dialog_token();
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);
    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO,        2,                               workspace->p2p_capability,                     TLV_UINT16);
    iter = p2p_write_tlv(iter, P2P_SEID_GROUP_OWNER_INTENT,         1,                               (workspace->group_owner_intent << 1) | workspace->group_owner_tie_breaker, TLV_UINT8);
    iter = p2p_write_tlv(iter, P2P_SEID_CONFIGURATION_TIMEOUT,      2,                               workspace->configuration_timeout, TLV_UINT16);
    iter = p2p_write_tlv(iter, P2P_SEID_CHANNEL,                    sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->listen_channel,            TLV_UINT8_PTR);
    iter = p2p_write_tlv(iter, P2P_SEID_INTENDED_INTERFACE_ADDRESS, sizeof(besl_mac_t),              (ptrdiff_t)&workspace->p2p_interface_address, TLV_UINT8_PTR);
    iter = p2p_write_tlv(iter, P2P_SEID_CHANNEL_LIST,               channel_list_length,             (ptrdiff_t)&p2p_channel_list_tlv->p2p_chan_list,            TLV_UINT8_PTR);

    temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_DEVICE_INFO,                sizeof(p2p_device_info_t),       (ptrdiff_t)&workspace->device_info,             TLV_UINT8_PTR);
    iter = tlv_write_value(iter, WPS_ID_DEVICE_NAME,                strlen(workspace->device_name),  &workspace->device_name,                       TLV_UINT8_PTR);
    temp_header->length = iter - temp_header->data;// FIX: ALIGN
    iter = p2p_write_tlv(iter, P2P_SEID_OP_CHANNEL,                 sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->operating_channel,            TLV_UINT8_PTR);
    p2p_header->length = (iter - p2p_header->oui);// FIX: ALIGN

    /*  Write WPS vendor specific tag */
    wps_vendor_specific_tag_header_t* wps_header = (wps_vendor_specific_tag_header_t*)iter;
    memcpy(wps_header, &template_wps_tag_header, sizeof(wps_vendor_specific_tag_header_t));
    iter += sizeof(wps_vendor_specific_tag_header_t);
    iter = tlv_write_value( iter, WPS_ID_VERSION,       WPS_ID_VERSION_S,        &version,                       TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_DEVICE_PWD_ID, WPS_ID_DEVICE_PWD_ID_S,  &device_password_id, TLV_UINT16 );

    /* Add WFA Vendor Extension */
    m1_vendor_ext_t* vendor_ext = (m1_vendor_ext_t*) tlv_write_header( iter, WPS_ID_VENDOR_EXT, sizeof(m1_vendor_ext_t) - sizeof(tlv8_uint8_t) );
    memcpy( vendor_ext->vendor_id, WFA_VENDOR_EXT_ID, 3 );
    vendor_ext->subid_version2.type      = WPS_WFA_SUBID_VERSION2;
    vendor_ext->subid_version2.length    = 1;
    vendor_ext->subid_version2.data      = WPS_VERSION2;
    iter += sizeof(tlv16_header_t) + sizeof(m1_vendor_ext_t) - sizeof(tlv8_uint8_t); // Sans request to enroll

    wps_header->length = (iter - wps_header->oui);
    workspace->sent_negotiation_request = 1;

    return iter;
}

uint8_t* p2p_write_negotiation_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    p2p_tlv_data_t* temp_header;
    uint8_t         version = 0x10;
    uint8_t*        iter    = buffer;
    uint8_t         status  = (workspace->p2p_current_state == P2P_STATE_NEGOTIATING) ? 0 : 1;
    uint16_t        device_password_id = workspace->p2p_wps_device_password_id;
    uint8_t         tie_breaker = device->tie_breaker;
    uint16_t channel_list_length;
    p2p_channel_list_tlv_t* p2p_channel_list_tlv;

    /* Determine status */
    if ( device->status == P2P_DEVICE_MUST_ALSO_BE_GROUP_OWNER )
    {
        status = 0x09;
    }
    else if ( device->status == P2P_DEVICE_NO_COMMON_CHANNEL )
    {
        status = P2P_STATUS_NO_COMMON_CHANNEL;
    }

    if ( status != P2P_STATUS_NO_COMMON_CHANNEL && workspace->i_am_group_owner == 1 )
    {
        p2p_channel_list_tlv = &workspace->common_channel_list.p2p_channel_list_tlv;
    }
    else
    {
        p2p_channel_list_tlv = &workspace->channel_list.p2p_channel_list_tlv;
    }
    channel_list_length = p2p_channel_list_tlv->length;

    /* Toggle tie breaker */
    tie_breaker = (~tie_breaker) & 1;

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_GO_NEGOTIATION_RESPONSE;
    fixed_parameters->p2p_dialog_token = device->dialog_token;
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);

    iter = p2p_write_tlv(iter, P2P_SEID_STATUS,                     1,                               status,                                      TLV_UINT8);
    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO,        2,                               workspace->p2p_capability,                   TLV_UINT16);
    /* Send back our intent but the tie breaker is the one from the negotiation request, toggled */
    iter = p2p_write_tlv(iter, P2P_SEID_GROUP_OWNER_INTENT,         1,                               (workspace->group_owner_intent << 1) | tie_breaker,              TLV_UINT8);
    iter = p2p_write_tlv(iter, P2P_SEID_CONFIGURATION_TIMEOUT,      2,                               workspace->configuration_timeout,            TLV_UINT16);
    iter = p2p_write_tlv(iter, P2P_SEID_OP_CHANNEL,                 sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->group_candidate.operating_channel,     TLV_UINT8_PTR);
    iter = p2p_write_tlv(iter, P2P_SEID_INTENDED_INTERFACE_ADDRESS, sizeof(besl_mac_t),              (ptrdiff_t)&workspace->p2p_interface_address, TLV_UINT8_PTR);
//     XXX need to determine the size of the channel list info properly
    iter = p2p_write_tlv(iter, P2P_SEID_CHANNEL_LIST,               channel_list_length,             (ptrdiff_t)&p2p_channel_list_tlv->p2p_chan_list,            TLV_UINT8_PTR);

    temp_header = (p2p_tlv_data_t*)iter;
    iter = p2p_write_tlv(iter, P2P_SEID_DEVICE_INFO,                sizeof(p2p_device_info_t),      (ptrdiff_t)&workspace->device_info, TLV_UINT8_PTR);
    iter = tlv_write_value(iter, WPS_ID_DEVICE_NAME,                strlen(workspace->device_name), &workspace->device_name, TLV_UINT8_PTR);
    temp_header->length = iter - temp_header->data;

    if (workspace->i_am_group_owner != 0)
    {
        temp_header = (p2p_tlv_data_t*)iter;
        iter = p2p_write_tlv(iter, P2P_SEID_GROUP_ID, sizeof(besl_mac_t), (ptrdiff_t)&workspace->device_info.p2p_device_address,  TLV_UINT8_PTR);
        iter = MEMCAT(iter, workspace->group_candidate.ssid, workspace->group_candidate.ssid_length);
        temp_header->length = iter - temp_header->data;
    }

    /*  End of P2P vendor specific tag */
    p2p_header->length  = (iter - p2p_header->oui);

    /*  Write WPS vendor specific tag */
    wps_vendor_specific_tag_header_t* wps_header = (wps_vendor_specific_tag_header_t*)iter;
    memcpy(wps_header, &template_wps_tag_header, sizeof(wps_vendor_specific_tag_header_t));
    iter += sizeof(wps_vendor_specific_tag_header_t);
    iter = tlv_write_value( iter, WPS_ID_VERSION,       WPS_ID_VERSION_S,        &version,                       TLV_UINT8 );
    iter = tlv_write_value( iter, WPS_ID_DEVICE_PWD_ID, WPS_ID_DEVICE_PWD_ID_S,  &device_password_id, TLV_UINT16 );
    wps_header->length = (iter - wps_header->oui);

    return iter;
}

uint8_t* p2p_write_negotiation_confirmation( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    p2p_tlv_data_t* temp_header;
    uint8_t*        iter    = buffer;
    uint8_t         status  = P2P_STATUS_SUCCESS;
    uint16_t channel_list_length;
    p2p_channel_list_tlv_t* p2p_channel_list_tlv;

    if ( device->status == P2P_DEVICE_NO_COMMON_CHANNEL )
    {
        status = P2P_STATUS_NO_COMMON_CHANNEL;
        p2p_channel_list_tlv = &workspace->channel_list.p2p_channel_list_tlv;
    }
    else
    {
        p2p_channel_list_tlv = &workspace->common_channel_list.p2p_channel_list_tlv;
    }
    channel_list_length = p2p_channel_list_tlv->length;

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_GO_NEGOTIATION_CONFIRMATION;
    fixed_parameters->p2p_dialog_token = device->dialog_token; // XXX we should check this is the same as our original request
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);

    iter = p2p_write_tlv(iter, P2P_SEID_STATUS,                     1,                               status,                                          TLV_UINT8);
    iter = p2p_write_tlv(iter, P2P_SEID_P2P_CAPABILITY_INFO,        2,                               workspace->p2p_capability,                     TLV_UINT16);
    iter = p2p_write_tlv(iter, P2P_SEID_OP_CHANNEL,                 sizeof(p2p_channel_info_t),      (ptrdiff_t)&workspace->group_candidate.operating_channel,          TLV_UINT8_PTR);
    iter = p2p_write_tlv(iter, P2P_SEID_CHANNEL_LIST,               channel_list_length,             (ptrdiff_t)&p2p_channel_list_tlv->p2p_chan_list,            TLV_UINT8_PTR);

    if (workspace->i_am_group_owner != 0)
    {
        temp_header = (p2p_tlv_data_t*)iter;
        iter = p2p_write_tlv(iter, P2P_SEID_GROUP_ID, sizeof(besl_mac_t), (ptrdiff_t)&workspace->device_info.p2p_device_address,  TLV_UINT8_PTR);
        iter = MEMCAT(iter, workspace->group_candidate.ssid, workspace->group_candidate.ssid_length);
        temp_header->length = iter - temp_header->data;
    }

    /*  End of P2P vendor specific tag */
    p2p_header->length  = (iter - p2p_header->oui);

    return iter;
}

uint8_t* p2p_write_device_discoverability_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    uint8_t* iter = buffer;
    uint8_t         status = device->status;

    BESL_DEBUG(("Sending Discoverability response\r\n"));

    /*  Write Fixed Parameters */
    p2p_public_action_frame_fixed_parameter_t* fixed_parameters = (p2p_public_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_public_action_frame_fixed_parameters, sizeof(p2p_public_action_frame_fixed_parameters));
    fixed_parameters->p2p_sub_type     = P2P_DEVICE_DISCOVERABILITY_RESPONSE;
    fixed_parameters->p2p_dialog_token = device->dialog_token;
    iter += sizeof(p2p_public_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);

    iter = p2p_write_tlv(iter, P2P_SEID_STATUS,                     1,                               status,                                      TLV_UINT8);

    /*  End of P2P vendor specific tag */
    p2p_header->length  = (iter - p2p_header->oui);

    return iter;
}

/* Action frame writers */
uint8_t* p2p_write_presence_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    uint8_t* iter = buffer;
    p2p_notice_of_absence_t noa;

    BESL_DEBUG(("Sending Presence response\r\n"));

    /* Fill out notice of absence fields - not yet supporting GO power save so these are zeroed */
    noa.index     = 0;
    noa.ct_window = 0;

    /*  Write Fixed Parameters */
    p2p_action_frame_fixed_parameter_t* fixed_parameters = (p2p_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_action_frame_fixed_parameters, sizeof(p2p_action_frame_fixed_parameters ) );
    fixed_parameters->p2p_sub_type     = P2P_PRESENCE_RESPONSE;
    fixed_parameters->p2p_dialog_token = device->dialog_token;
    iter += sizeof(p2p_action_frame_fixed_parameter_t);

    /*  Write P2P vendor specific tag */
    p2p_vendor_specific_tag_header_t* p2p_header = (p2p_vendor_specific_tag_header_t*)iter;
    memcpy(p2p_header, &template_p2p_tag_header, sizeof(p2p_vendor_specific_tag_header_t));
    iter += sizeof(p2p_vendor_specific_tag_header_t);

    /* Reply with status as success" */
    iter = p2p_write_tlv(iter, P2P_SEID_STATUS,                 1,                                   0x00,           TLV_UINT8);
    iter = p2p_write_tlv(iter, P2P_SEID_NOTICE_OF_ABSENCE,      sizeof(p2p_notice_of_absence_t),     (ptrdiff_t)&noa, TLV_UINT8_PTR);

    p2p_header->length = (uint8_t)(iter - p2p_header->oui);
    return iter;
}

uint8_t* p2p_write_go_discoverability_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer )
{
    uint8_t* iter = buffer;

    BESL_DEBUG(("Sending GO discoverability request\r\n"));

    /*  Write Fixed Parameters */
    p2p_action_frame_fixed_parameter_t* fixed_parameters = (p2p_action_frame_fixed_parameter_t*)iter;
    memcpy(iter, p2p_action_frame_fixed_parameters, sizeof(p2p_action_frame_fixed_parameters ) );
    fixed_parameters->p2p_sub_type     = P2P_GO_DISCOVERABILITY_REQUEST;
    fixed_parameters->p2p_dialog_token = 0; // Dialog token is NULL in a GO DIscoverability Request
    iter += sizeof(p2p_action_frame_fixed_parameter_t);

    /* There is no elements field in a GO discoverability request */

    return iter;
}

static uint8_t* p2p_write_tlv(uint8_t* iter, uint8_t type, uint16_t length, ptrdiff_t data, tlv_data_type_t data_type)
{
    p2p_tlv_data_t* tlv = (p2p_tlv_data_t*) iter;
    uint16_t aligned_length;

    tlv->type   = type;
    BESL_WRITE_16(&aligned_length, length);
    tlv->length = aligned_length;
    switch ( data_type )
    {
        case TLV_UINT8:
            tlv->data[0] = (uint8_t)data;
            break;
        case TLV_UINT16:
            BESL_WRITE_16(tlv->data, data);
            break;
        case TLV_UINT32:
            BESL_WRITE_32(tlv->data, data);
            break;
        default:
            memcpy( tlv->data, (void*)data, length );
            break;
    }
    return iter + sizeof(p2p_tlv_header_t) + length;
}

static uint8_t p2p_get_new_dialog_token( void )
{
    /* Always increment the dialog token for use in a new request action frame. The dialog token must be non-zero except for special use */
    p2p_dialog_token++;
    if ( p2p_dialog_token == 0 )
    {
        p2p_dialog_token++;
    }
    return p2p_dialog_token;
}
