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
 * BLE RESTful API Demo Application
 *
 */

#include "string.h"
#include "wiced.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "restful_smart_ble.h"
#include "restful_smart_server.h"
#include "restful_smart_response.h"
#include "http_server.h"
#include "bt_types.h"
#include "big_stack_interface.h"

/******************************************************
 *                    Constants
 ******************************************************/

#define CCC_EVENT                   ( 0xCCCE4E47 )
#define CCC_UUID                    ( 0x2902 )
#define CACHED_VALUE_BUFFER_LENGTH  ( wiced_bt_cfg_settings.gatt_cfg.max_attr_len )

/******************************************************
 *                   Structures
 ******************************************************/

typedef union
{
    uint32_t              passkey;
    wiced_bt_smp_sc_oob_t secure_oob_data;
    uint8_t               error_reason;
} pairing_data_t;

typedef struct
{
    linked_list_node_t             pairing_info_node;
    smart_node_handle_t            node_handle;
    wiced_http_response_stream_t*  stream;
    rest_smart_pairing_status_t    expected_action;
    rest_smart_pairing_status_t    status;
    wiced_bt_dev_ble_io_caps_req_t io_caps;
    uint8_t                        pairing_id[16];
    pairing_data_t                 pairing_data;
} rest_smart_pairing_info_t;

typedef struct
{
    linked_list_node_t                 value_node;
    uint32_t                           magic_number;
    big_gatt_connection_t*             connection;
    smart_characteristic_handle_t      characteristic;
    wiced_http_response_stream_t*      sse_stream;
    wiced_bt_gatt_client_char_config_t config;
    wiced_bool_t                       is_subscribed;
    uint16_t                           ccc_handle;
    smart_value_handle_t               value;
} rest_smart_cached_value_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

extern void                       SMP_ConfirmReply                  ( BD_ADDR bd_addr, UINT8 res );
static wiced_result_t             restful_interface_callback        ( big_gatt_interface_t* interface, wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* data );
static wiced_bt_gatt_status_t     restful_smart_gatt_callback       ( big_gatt_connection_t* connection, const big_gatt_request_t* current_request, wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *data );
static void                       restful_smart_scan_result_callback( wiced_bt_ble_scan_results_t *result, uint8_t *data );
static rest_smart_pairing_info_t* find_pairing_node_info            ( wiced_bt_device_address_t* address );
static rest_smart_pairing_info_t* create_pairing_node_info          ( smart_node_handle_t* node );
static wiced_result_t             remove_pairing_node_info          ( rest_smart_pairing_info_t* pairing_node_info );
static rest_smart_cached_value_t* create_cached_value               ( big_gatt_connection_t* connection, const smart_characteristic_handle_t* characteristic );
static rest_smart_cached_value_t* find_cached_value_by_value_handle ( big_gatt_connection_t* connection, uint16_t value_handle );
static rest_smart_cached_value_t* find_cached_value_by_ccc_handle   ( big_gatt_connection_t* connection, uint16_t ccc_handle );
static wiced_result_t             remove_cached_value               ( rest_smart_cached_value_t* value );
static wiced_result_t             read_client_characteristic_config ( big_gatt_connection_t* connection, const smart_characteristic_handle_t* characteristic, void* arg );
static wiced_result_t             write_client_characteristic_config( big_gatt_connection_t* connection, uint16_t ccc_handle, wiced_bt_gatt_client_char_config_t config, void* arg );
static wiced_result_t             handle_gatt_read_response         ( big_gatt_connection_t* connection, const big_gatt_request_t* current_request, wiced_bt_gatt_operation_complete_t* data );
static wiced_result_t             handle_ccc_read_response          ( big_gatt_connection_t* connection, wiced_bt_gatt_operation_complete_t* data );
static wiced_result_t             handle_ccc_write_response         ( big_gatt_connection_t* connection, wiced_bt_gatt_operation_complete_t* data );

/******************************************************
 *               External Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* Temporary global variables for discovery */
static volatile uint32_t             scan_result_count = 0;
smart_characteristic_handle_t        current_characteristic_handle;
wiced_bt_gatt_char_declaration_t     current_characteristic;
wiced_bt_gatt_char_declaration_t     previous_characteristic;
static volatile uint32_t             current_procedure_count = 0;
static wiced_http_response_stream_t* current_stream = NULL;

extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
static big_gatt_interface_t    restful_app_interface;

static rest_smart_pairing_info_t* pairing_info_buffer = NULL;
static linked_list_t              active_pairing_node_list;   /* Pairing info list. Nodes in this list are in pairing status */
static linked_list_t              inactive_pairing_node_list; /* Unused bond info */

static rest_smart_cached_value_t* cached_value_buffer = NULL;
static linked_list_t              active_cached_value_list;
static linked_list_t              inactive_cached_value_list;

static wiced_mutex_t              list_mutex; /* Mutex for protecting access to pairing info and cache value lists */

static const wiced_bt_dev_ble_io_caps_req_t default_local_io_caps =
{
    .bd_addr      = { 0 },
    .local_io_cap = BTM_IO_CAPABILITIES_BLE_DISPLAY_AND_KEYBOARD_INPUT,
    .oob_data     = 0,
    .auth_req     = BTM_LE_AUTH_REQ_BOND|BTM_LE_AUTH_REQ_MITM, /* BTM_LE_AUTH_REQ_SC_MITM_BOND */
    .max_key_size = 16,
    .init_keys    = (BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK), // init_keys - Keys to be distributed, bit mask
    .resp_keys    = (BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK)  // resp_keys - Keys to be distributed, bit mask
};

static const uint8_t temp_pairing_id[] = { 0x30, 0x30, 0x30, 0x31, 0x30, 0x32, 0x30, 0x33, 0x30, 0x34, 0x30, 0x35, 0x30, 0x36, 0x30, 0x37, 0x30, 0x38, 0x30, 0x39, 0x30, 0x41, 0x30, 0x42, 0x30, 0x43, 0x30, 0x44, 0x30, 0x45, 0x30, 0x46, 0x0 };

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t restful_smart_server_start( big_peer_device_link_keys_callback_t callback )
{
    uint32_t a;
    uint32_t pairing_info_buffer_size = TOTAL_PAIRING_INFOS * sizeof( rest_smart_pairing_info_t );
    uint32_t cached_value_buffer_size = TOTAL_CACHED_VALUES * ( sizeof( rest_smart_cached_value_t) - 1 + wiced_bt_cfg_settings.gatt_cfg.max_attr_len );

    /* Initialise pairing info */
    pairing_info_buffer = (rest_smart_pairing_info_t*)malloc_named( "pairing info", pairing_info_buffer_size );
    if ( pairing_info_buffer == NULL )
    {
        return WICED_BT_OUT_OF_HEAP_SPACE;
    }
    memset( pairing_info_buffer, 0, pairing_info_buffer_size );
    linked_list_init( &active_pairing_node_list );
    linked_list_init( &inactive_pairing_node_list );
    for ( a = 0; a < TOTAL_PAIRING_INFOS; a++ )
    {
        linked_list_insert_node_at_rear( &inactive_pairing_node_list, &pairing_info_buffer[a].pairing_info_node );
    }

    /* Initialise cached values */
    cached_value_buffer = (rest_smart_cached_value_t*)malloc_named( "cached value", cached_value_buffer_size );
    if ( cached_value_buffer == NULL )
    {
        free( pairing_info_buffer );
        return WICED_BT_OUT_OF_HEAP_SPACE;
    }
    memset( cached_value_buffer, 0, cached_value_buffer_size );
    linked_list_init( &active_cached_value_list );
    linked_list_init( &inactive_cached_value_list );
    for ( a = 0; a < TOTAL_CACHED_VALUES; a++ )
    {
        linked_list_insert_node_at_rear( &inactive_cached_value_list, &cached_value_buffer[a].value_node );
    }

    wiced_rtos_init_mutex( &list_mutex );

    return big_add_gatt_client_interface( &restful_app_interface, restful_interface_callback, restful_smart_gatt_callback, callback );
}

wiced_result_t restful_smart_server_stop( void )
{
    big_remove_gatt_client_interface( &restful_app_interface );
    linked_list_deinit( &active_pairing_node_list );
    linked_list_deinit( &inactive_pairing_node_list );
    linked_list_deinit( &active_cached_value_list );
    linked_list_deinit( &inactive_cached_value_list );
    free( pairing_info_buffer );
    free( cached_value_buffer );
    wiced_rtos_deinit_mutex( &list_mutex );
    return WICED_SUCCESS;
}

wiced_result_t restful_smart_connect( wiced_http_response_stream_t* stream, const smart_node_handle_t* node )
{
    big_gatt_connection_t* connection;
    wiced_result_t result;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection != NULL )
    {
        return rest_smart_response_send_node_data( stream, node );
    }

    current_stream = stream;

    result = big_ble_gatt_client_connect( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( result != WICED_SUCCESS )
    {
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_discover_all_primary_services( wiced_http_response_stream_t* stream, const smart_node_handle_t* node )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    current_procedure_count              = 0;
    request->arg                         = (void*)stream;
    request->feature                     = BIG_GATT_DISCOVER_ALL_PRIMARY_SERVICES;
    request->parameter.discover.s_handle = 1;
    request->parameter.discover.e_handle = 0xffff;
    memset( &request->parameter.discover.uuid, 0, sizeof( wiced_bt_uuid_t ) );

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_discover_primary_services_by_uuid( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const wiced_bt_uuid_t* uuid )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    current_procedure_count              = 0;
    request->arg                         = (void*)stream;
    request->feature                     = BIG_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID;
    request->parameter.discover.s_handle = 1;
    request->parameter.discover.e_handle = 0xffff;
    memcpy( &request->parameter.discover.uuid, (void*)uuid, sizeof( wiced_bt_uuid_t ) );

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_discover_characteristics_of_a_service( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_service_handle_t* service )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    request->arg                         = (void*)stream;
    request->feature                     = BIG_GATT_DISCOVER_ALL_CHARACTERISTICS_OF_A_SERVICE;
    request->parameter.discover.s_handle = service->start_handle;
    request->parameter.discover.e_handle = service->end_handle;
    memset( &request->parameter.discover.uuid, 0, sizeof( wiced_bt_uuid_t ) );

    current_procedure_count    = 0;

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_discover_characteristics_by_uuid( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const wiced_bt_uuid_t* uuid )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    /* Fill out request parameters */
    request->arg                         = (void*)stream;
    request->feature                     = BIG_GATT_DISCOVER_CHARACTERISTICS_BY_UUID;
    request->parameter.discover.s_handle = 0x0001;
    request->parameter.discover.e_handle = 0xffff;
    memcpy( &request->parameter.discover.uuid, &uuid, sizeof( request->parameter.discover.uuid ) );

    /* Set global vars */
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_discover_characteristic_descriptors( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }


    /* Fill out request parameters */
    request->arg                         = (void*)stream;
    request->feature                     = BIG_GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS;
    request->parameter.discover.s_handle = characteristic->value_handle + 1;;
    request->parameter.discover.e_handle = characteristic->end_handle;
    memset( &request->parameter.discover.uuid, 0, sizeof( wiced_bt_uuid_t ) );

    /* Set global vars */
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_read_characteristic_value( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    /* Fill out request parameters */
    request->arg                               = (void*)stream;
    request->feature                           = BIG_GATT_READ_CHARACTERISTIC_VALUE;
    request->parameter.read.by_handle.auth_req = GATT_AUTH_REQ_NONE;
    request->parameter.read.by_handle.handle   = characteristic->value_handle;

    /* Set global vars */
    memcpy( &current_characteristic_handle, characteristic, sizeof( current_characteristic_handle ) );
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_read_characteristic_values_by_uuid( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic, const wiced_bt_uuid_t* uuid )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    /* Fill out request parameters */
    request->arg                               = (void*)stream;
    request->feature                           = BIG_GATT_READ_CHARACTERISTICS_BY_UUID;
    request->parameter.read.char_type.auth_req = GATT_AUTH_REQ_NONE;
    request->parameter.read.char_type.s_handle = 0x0001;
    request->parameter.read.char_type.e_handle = 0xffff;
    memcpy( &request->parameter.read.char_type.uuid, uuid, sizeof( request->parameter.read.char_type.uuid ) );

    /* Set global vars */
    memcpy( &current_characteristic_handle, characteristic, sizeof( current_characteristic_handle ) );
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_write_characteristic_value( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic, const smart_value_handle_t* value )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    /* Fill out request parameters */
    request->arg                      = (void*)stream;
    request->feature                  = BIG_GATT_WRITE_CHARACTERISTIC_VALUE;
    request->parameter.write.auth_req = GATT_AUTH_REQ_NONE;
    request->parameter.write.handle   = characteristic->value_handle;
    request->parameter.write.len      = value->length;
    request->parameter.write.offset   = 0;
    memcpy( request->parameter.write.value, value->value, value->length );

    /* Set global vars */
    memcpy( &current_characteristic_handle, characteristic, sizeof( current_characteristic_handle ) );
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_write_characteristic_value_without_response( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic, const smart_value_handle_t* value )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }


    /* Fill out request parameters */
    request->arg                      = (void*)stream;
    request->feature                  = BIG_GATT_WRITE_WITHOUT_RESPONSE;
    request->parameter.write.auth_req = GATT_AUTH_REQ_NONE;
    request->parameter.write.handle   = characteristic->value_handle;
    request->parameter.write.len      = value->length;
    request->parameter.write.offset   = 0;
    memcpy( request->parameter.write.value, value->value, value->length );

    /* Set global vars */
    memcpy( &current_characteristic_handle, characteristic, sizeof( current_characteristic_handle ) );
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );

    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_read_cached_value( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    rest_smart_cached_value_t* cached_value;
    wiced_result_t             result = WICED_NOT_FOUND;
    char node_string[NODE_BUFFER_LENGTH];
    char char_string[CHARACTERISTIC_BUFFER_LENGTH];

    /* Find connection */
    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    /* Find cached value */
    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_value_handle( connection, characteristic->value_handle );
    if ( cached_value == NULL )
    {
        wiced_rtos_unlock_mutex( &list_mutex );
        goto exit;
    }
    wiced_rtos_unlock_mutex( &list_mutex );

    memset( &node_string, 0, sizeof( node_string ) );
    memset( &char_string, 0, sizeof( char_string ) );
    format_node_string( node_string, &node->bda, node->type );
    format_characteristic_string( char_string, characteristic->start_handle, characteristic->end_handle, characteristic->value_handle );
    rest_smart_response_write_characteristic_value( stream, node_string, char_string, characteristic->value_handle, cached_value->value.value, cached_value->value.length );
    rest_smart_response_end_stream( stream );
    return WICED_SUCCESS;

    exit:
    if ( result != WICED_SUCCESS )
    {
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;

}

wiced_result_t restful_smart_enable_notification( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    rest_smart_cached_value_t* cached_value;
    wiced_result_t             result = WICED_NOT_FOUND;

    /* Find connection */
    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    /* Find cached value */
    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_value_handle( connection, characteristic->value_handle );
    if ( cached_value == NULL )
    {
        /* Not found. Create one */
        cached_value = create_cached_value( connection, characteristic );
        if ( cached_value == NULL )
        {
            /* Max cached value is reached */
            wiced_rtos_unlock_mutex( &list_mutex );
            goto exit;
        }
    }

    cached_value->config     = GATT_CLIENT_CONFIG_NOTIFICATION;
    cached_value->sse_stream = stream;
    wiced_rtos_unlock_mutex( &list_mutex );

    /* Find CCC in characteristic. Write in callback */
    result = read_client_characteristic_config( connection, characteristic, (void*)CCC_EVENT );

    exit:
    if ( result != WICED_SUCCESS )
    {
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_disable_notification( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    rest_smart_cached_value_t* cached_value;
    wiced_result_t             result = WICED_NOT_FOUND;

    /* Find connection */
    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    /* Find cached value */
    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_value_handle( connection, characteristic->value_handle );
    if ( cached_value == NULL )
    {
        /* Max cached value is reached */
        wiced_rtos_unlock_mutex( &list_mutex );
        goto exit;
    }

    cached_value->config     = GATT_CLIENT_CONFIG_NONE;
    cached_value->sse_stream = stream;
    wiced_rtos_unlock_mutex( &list_mutex );

    /* Find CCC in characteristic. Write in callback */
    result = read_client_characteristic_config( connection, characteristic, (void*)CCC_EVENT );

    exit:
    if ( result != WICED_SUCCESS )
    {
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_enable_indication( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    rest_smart_cached_value_t* cached_value;
    wiced_result_t             result = WICED_NOT_FOUND;

    /* Find connection */
    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    /* Find cached value */
    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_value_handle( connection, characteristic->value_handle );
    if ( cached_value == NULL )
    {
        /* Not found. Create one */
        cached_value = create_cached_value( connection, characteristic );
        if ( cached_value == NULL )
        {
            /* Max cached value is reached */
            wiced_rtos_unlock_mutex( &list_mutex );
            goto exit;
        }
    }

    cached_value->config     = GATT_CLIENT_CONFIG_INDICATION;
    cached_value->sse_stream = stream;
    wiced_rtos_unlock_mutex( &list_mutex );

    /* Find CCC in characteristic. Write in callback */
    result = read_client_characteristic_config( connection, characteristic, (void*)CCC_EVENT );

    exit:
    if ( result != WICED_SUCCESS )
    {
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_disable_indication( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    return restful_smart_disable_notification( stream, node, characteristic );
}

wiced_result_t restful_smart_subscribe_notification( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    rest_smart_cached_value_t* cached_value;
    wiced_result_t             result = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_value_handle( connection, characteristic->value_handle );
    if ( cached_value == NULL )
    {
        wiced_rtos_unlock_mutex( &list_mutex );
        goto exit;
    }

    cached_value->is_subscribed = WICED_TRUE;
    cached_value->sse_stream    = stream;
    result = WICED_SUCCESS;
    wiced_rtos_unlock_mutex( &list_mutex );

    wiced_http_response_stream_write_header( stream, HTTP_200_TYPE, CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_TEXT_EVENT_STREAM );
    return WICED_SUCCESS;

    exit:
    rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    return result;
}

wiced_result_t restful_smart_unsubscribe_notification( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    big_gatt_connection_t*     connection;
    rest_smart_cached_value_t* cached_value;
    wiced_result_t             result = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_value_handle( connection, characteristic->value_handle );
    if ( cached_value == NULL )
    {
        wiced_rtos_unlock_mutex( &list_mutex );
        goto exit;
    }

    cached_value->is_subscribed = WICED_FALSE;
    cached_value->sse_stream    = NULL;
    result = WICED_SUCCESS;
    wiced_rtos_unlock_mutex( &list_mutex );

    exit:
    rest_smart_response_send_error_message( stream, ( result == WICED_SUCCESS ) ? REST_SMART_STATUS_200 : REST_SMART_STATUS_404 );
    return result;
}

wiced_result_t restful_smart_subscribe_indication( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    return restful_smart_subscribe_notification( stream, node, characteristic );
}

wiced_result_t restful_smart_unsubscribe_indication( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, const smart_characteristic_handle_t* characteristic )
{
    return restful_smart_unsubscribe_notification( stream, node, characteristic );
}

wiced_result_t restful_smart_read_descriptor_value( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, uint16_t descriptor_handle )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    /* Fill out request parameters */
    request->arg                               = (void*)stream;
    request->feature                           = BIG_GATT_READ_CHARACTERISTIC_DESCRIPTOR;
    request->parameter.read.by_handle.auth_req = GATT_AUTH_REQ_NONE;
    request->parameter.read.by_handle.handle   = descriptor_handle;

    /* Set global vars */
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );
    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_write_descriptor_value( wiced_http_response_stream_t* stream, const smart_node_handle_t* node, uint16_t descriptor_handle, const smart_value_handle_t* value )
{
    big_gatt_connection_t*     connection;
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    connection = big_find_gatt_connection_by_ble_address( &restful_app_interface, (wiced_bt_ble_address_t*)node );
    if ( connection == NULL )
    {
        goto exit;
    }

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        goto exit;
    }

    /* Fill out request parameters */
    request->arg                      = (void*)stream;
    request->feature                  = BIG_GATT_WRITE_CHARACTERISTIC_DESCRIPTOR;
    request->parameter.write.auth_req = GATT_AUTH_REQ_NONE;
    request->parameter.write.handle   = descriptor_handle;
    request->parameter.write.len      = value->length;
    request->parameter.write.offset   = 0;
    memcpy( request->parameter.write.value, value->value, value->length );

    /* Set global vars */
    current_procedure_count = 0;

    result = big_send_gatt_request( connection, buffer );
    exit:
    if ( result != WICED_SUCCESS )
    {
        if ( buffer != NULL )
        {
            big_release_gatt_request_buffer( buffer );
        }
        rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    }
    return result;
}

wiced_result_t restful_smart_passive_scan( wiced_http_response_stream_t* stream )
{
    current_stream    = stream;
    scan_result_count = 0;

    wiced_bt_cfg_settings.ble_scan_cfg.scan_mode = BTM_BLE_SCAN_MODE_PASSIVE;

    rest_smart_response_write_status_code( current_stream, REST_SMART_STATUS_200 );
    rest_smart_response_write_node_array_start( current_stream );

    wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, restful_smart_scan_result_callback );

    return WICED_SUCCESS;
}

wiced_result_t restful_smart_active_scan( wiced_http_response_stream_t* stream )
{
    current_stream    = stream;
    scan_result_count = 0;

    wiced_bt_cfg_settings.ble_scan_cfg.scan_mode = BTM_BLE_SCAN_MODE_ACTIVE;

    rest_smart_response_write_status_code( current_stream, REST_SMART_STATUS_200 );
    rest_smart_response_write_node_array_start( current_stream );

    wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, restful_smart_scan_result_callback );

    return WICED_SUCCESS;
}

wiced_result_t restful_smart_start_pairing( wiced_http_response_stream_t* stream, smart_node_handle_t* node_handle, rest_smart_pairing_type_t pairing_type )
{
    big_gatt_connection_t*     connection = big_find_gatt_connection_by_device_address( &restful_app_interface, (wiced_bt_device_address_t*)&node_handle->bda );
    rest_smart_pairing_info_t* pairing_info  = NULL;

    if ( connection == NULL )
    {
        /* 404 - Connection not found */
        goto error;
    }

    /* Find device address in pairing list. If it's already pairing, return error */
    pairing_info = find_pairing_node_info( (wiced_bt_device_address_t*)&node_handle->bda );
    if ( pairing_info != NULL )
    {
        /* 400 - Bad request. Pairing is already in progress */
        goto error;
    }

    /* Add device to pairing list */
    pairing_info = create_pairing_node_info( node_handle );
    if ( pairing_info == NULL )
    {
        /* 404 - resources exhausted. Report as 404 */
        goto error;
    }

    memcpy( pairing_info->pairing_id, temp_pairing_id, sizeof( pairing_info->pairing_id ) );
    pairing_info->stream  = stream;
    pairing_info->io_caps = default_local_io_caps;

    switch ( pairing_type )
    {
        case REST_SMART_PAIRING_SECURE_CONNECTIONS_OOB:
        {
            /* Using Secure Connections OOB pairing. Need to generate local SC OOB data */
            pairing_info->io_caps.oob_data = 1;
            wiced_bt_smp_create_local_sc_oob_data( node_handle->bda, node_handle->type );
            break;
        }
        case REST_SMART_PAIRING_LEGACY_OOB:
        {
            pairing_info->io_caps.oob_data = 1;
            wiced_bt_dev_sec_bond( node_handle->bda, node_handle->type, BT_TRANSPORT_LE, 0, NULL );
            break;
        }
        case REST_SMART_PAIRING_DISPLAY_ONLY:
        case REST_SMART_PAIRING_DISPLAY_YES_NO:
        case REST_SMART_PAIRING_KEYBOARD_ONLY:
        case REST_SMART_PAIRING_NO_IO:
        case REST_SMART_PAIRING_KEYBOARD_DISPLAY:
        {
            /* For any other pairing type, use initiate bond API */
            pairing_info->io_caps.local_io_cap = (wiced_bt_dev_io_cap_t)pairing_type;
            wiced_bt_dev_sec_bond( node_handle->bda, node_handle->type, BT_TRANSPORT_LE, 0, NULL );
            break;
        }
        default:
            return WICED_BT_BADARG;
    }

    return WICED_BT_SUCCESS;

    error:
    if ( pairing_info != NULL )
    {
        remove_pairing_node_info( pairing_info );
    }
    rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    return WICED_ERROR;
}

wiced_result_t restful_smart_process_client_pairing_response( wiced_http_response_stream_t* stream, smart_node_handle_t* node_handle, rest_smart_client_pairing_response_t* client_response )
{
    big_gatt_connection_t*     connection = big_find_gatt_connection_by_device_address( &restful_app_interface, (wiced_bt_device_address_t*)&node_handle->bda );
    rest_smart_pairing_info_t* pairing_info  = NULL;
    wiced_result_t             result     = WICED_ERROR;

    if ( connection == NULL )
    {
        /* Connection not found */
        goto error;
    }

    pairing_info = find_pairing_node_info( (wiced_bt_device_address_t*)&node_handle->bda );
    if ( pairing_info == NULL )
    {
        /* Pairing info not found. Client hasn't initiated pairing before */
        goto error;
    }

    if ( client_response->pairing_id_available == WICED_FALSE )
    {
        /* Bad argument. Pairing ID isn't provided */
        goto error;
    }

    if ( memcmp( client_response->pairing_id, pairing_info->pairing_id, sizeof( client_response->pairing_id ) ) != 0 )
    {
        /* Non-matching pairing ID provided by the client */
        goto error;
    }

    pairing_info->stream = stream;

    switch ( pairing_info->expected_action )
    {
        case LE_LEGACY_OOB_EXPECTED:
            if ( client_response->tk_available == WICED_TRUE )
            {
                wiced_bt_smp_oob_data_reply( node_handle->bda, WICED_SUCCESS, sizeof( client_response->tk ), client_response->tk );
            }
            else
            {
                goto error;
            }
            break;

        case PASSKEY_INPUT_EXPECTED:
            if ( client_response->passkey_available == WICED_TRUE )
            {
                wiced_bt_dev_pass_key_req_reply( WICED_SUCCESS, node_handle->bda, client_response->passkey );
            }
            else
            {
                goto error;
            }
            break;

        case PASSKEY_DISPLAY_EXPECTED:
        {
            /* No more event is expected. Send response back to client */
            rest_smart_pairing_response_t response;
            memset( &response, 0, sizeof( response ));
            if ( pairing_info->status == PAIRING_SUCCESSFUL )
            {
                response.status_code         = REST_SMART_STATUS_200;
                response.pairing_status_code = PAIRING_SUCCESSFUL;
            }
            else if ( pairing_info->status == PAIRING_FAILED )
            {
                remove_pairing_node_info( pairing_info );

                response.status_code         = REST_SMART_STATUS_403;
                response.pairing_status_code = PAIRING_FAILED;
                response.reason_code         = pairing_info->pairing_data.error_reason;
            }
            result = rest_smart_response_send_pairing_data( stream, &response );
            remove_pairing_node_info( pairing_info );
            break;
        }

        case NUMERIC_COMPARISON_EXPECTED:
            if ( client_response->confirmed_available == WICED_TRUE )
            {
                SMP_ConfirmReply( node_handle->bda, ( ( client_response->confirmed == WICED_TRUE ) ? WICED_SUCCESS : WICED_ERROR ) );
            }
            else
            {
                goto error;
            }
            break;

        case LE_SECURE_OOB_EXPECTED:
        {
            wiced_bool_t all_params_available = ( ( client_response->bdaddrb_available == WICED_TRUE  ) && ( client_response->rb_available == WICED_TRUE  ) && ( client_response->cb_available == WICED_TRUE  ) ) ? WICED_TRUE : WICED_FALSE;
            wiced_bool_t no_params_available  = ( ( client_response->bdaddrb_available == WICED_FALSE ) && ( client_response->rb_available == WICED_FALSE ) && ( client_response->cb_available == WICED_FALSE ) ) ? WICED_TRUE : WICED_FALSE;

            if ( all_params_available == WICED_TRUE )
            {
                memcpy( pairing_info->pairing_data.secure_oob_data.peer_oob_data.commitment,          client_response->cb,      sizeof( pairing_info->pairing_data.secure_oob_data.peer_oob_data.commitment ) );
                memcpy( pairing_info->pairing_data.secure_oob_data.peer_oob_data.randomizer,          client_response->rb,      sizeof( pairing_info->pairing_data.secure_oob_data.peer_oob_data.randomizer ) );
                memcpy( &pairing_info->pairing_data.secure_oob_data.peer_oob_data.addr_received_from, &client_response->bdaddrb, sizeof( pairing_info->pairing_data.secure_oob_data.peer_oob_data.addr_received_from ) );
            }

            if ( ( pairing_info->pairing_data.secure_oob_data.local_oob_data.present == WICED_TRUE ) && ( ( all_params_available == WICED_TRUE ) || ( no_params_available == WICED_TRUE ) ) )
            {
                /* Initiator OOB is set with all params available or no params */
                wiced_bt_dev_sec_bond( node_handle->bda, node_handle->type, BT_TRANSPORT_LE, 0, NULL );
            }
            else if ( ( pairing_info->pairing_data.secure_oob_data.local_oob_data.present == WICED_FALSE ) && ( pairing_info->pairing_data.secure_oob_data.peer_oob_data.present == WICED_TRUE ) && ( all_params_available == WICED_TRUE ) )
            {
                /* responder OOB is set, initiator OOB is not set with all params available */
                wiced_bt_dev_sec_bond( node_handle->bda, node_handle->type, BT_TRANSPORT_LE, 0, NULL );
            }
            else
            {
                goto error;
            }
            break;
        }

        case PAIRING_FAILED:
        case PAIRING_ABORTED:
        case PAIRING_SUCCESSFUL:
        default:
        {
            goto error;
            break;
        }
    }

    /* Successfully processed pairing response from client */
    return result;

error:
    rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    return WICED_ERROR;
}

wiced_result_t restful_smart_cancel_pairing( wiced_http_response_stream_t* stream, smart_node_handle_t* node_handle )
{
    big_gatt_connection_t*     connection    = big_find_gatt_connection_by_device_address( &restful_app_interface, (wiced_bt_device_address_t*)&node_handle->bda );
    rest_smart_pairing_info_t* pairing_info  = NULL;
    wiced_result_t             result        = WICED_ERROR;

    if ( connection == NULL )
    {
        /* Connection not found */
        goto error;
    }

    pairing_info = find_pairing_node_info( (wiced_bt_device_address_t*)&node_handle->bda );
    if ( pairing_info == NULL )
    {
        /* Pairing info not found. Client hasn't initiated pairing before */
        goto error;
    }

    pairing_info->stream = stream;

    result = wiced_bt_dev_sec_bond_cancel( node_handle->bda );
    if ( result != WICED_BT_PENDING )
    {
        goto error;
    }
    return result;

error:
    rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
    return WICED_ERROR;
}

static wiced_bt_gatt_status_t restful_smart_gatt_callback( big_gatt_connection_t* connection, const big_gatt_request_t* current_request, wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
        {
            /* Write response only if connection status event is originated from app */
            if ( current_stream != NULL )
            {
                if ( data->connection_status.connected == WICED_TRUE )
                {
                    rest_smart_response_send_node_data( current_stream, &connection->address );
                }
                else
                {
                    rest_smart_response_send_error_message( current_stream, REST_SMART_STATUS_504 );
                }
            }
            break;
        }

        case GATT_DISCOVERY_RESULT_EVT:
        {
            wiced_http_response_stream_t* stream = (wiced_http_response_stream_t*)current_request->arg;
            char node[NODE_BUFFER_LENGTH] = { 0 };

            format_node_string( node, (const wiced_bt_device_address_t*)&connection->address.bda, connection->address.type );

            if ( data->discovery_result.discovery_type == GATT_DISCOVER_SERVICES_ALL || data->discovery_result.discovery_type == GATT_DISCOVER_SERVICES_BY_UUID )
            {
                char  service[SERVICE_BUFFER_LENGTH];
                char  uuid   [UUID_BUFFER_LENGTH];

                memset( &service, 0, sizeof( service ) );
                memset( &uuid, 0, sizeof( uuid ) );

                if ( current_procedure_count == 0 )
                {
                    rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
                    rest_smart_response_write_service_array_start( stream );
                }

                format_service_string( service, data->discovery_result.discovery_data.group_value.s_handle, data->discovery_result.discovery_data.group_value.e_handle );
                uuid_to_string( &data->discovery_result.discovery_data.group_value.service_type, uuid );
                rest_smart_response_write_service( stream, node, service, data->discovery_result.discovery_data.group_value.s_handle, uuid, WICED_TRUE, ( current_procedure_count == 0 ) ? WICED_TRUE : WICED_FALSE );
                current_procedure_count++;
            }
            else if ( data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTICS )
            {
                memcpy( &current_characteristic, &data->discovery_result.discovery_data.characteristic_declaration, sizeof( current_characteristic ) );

                /* Send previous discovered characteristic now because the end handle is calculated from the current characteristic handle range */
                if ( current_procedure_count > 0 )
                {
                    char  characteristic[CHARACTERISTIC_BUFFER_LENGTH];
                    char  uuid          [UUID_BUFFER_LENGTH];

                    format_characteristic_string( characteristic, previous_characteristic.handle, current_characteristic.handle - 1, previous_characteristic.val_handle );
                    uuid_to_string( &previous_characteristic.char_uuid, uuid );
                    rest_smart_response_write_characteristic( stream, node, characteristic, previous_characteristic.handle, uuid, previous_characteristic.characteristic_properties, ( current_procedure_count == 1 ) ? WICED_TRUE : WICED_FALSE );
                }
                else if ( current_procedure_count == 0 )
                {
                    rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
                    rest_smart_response_write_characteristic_array_start( stream );
                }

                memcpy( &previous_characteristic, &data->discovery_result.discovery_data.characteristic_declaration, sizeof( previous_characteristic ) );
                current_procedure_count++;
            }
            else if ( data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS )
            {
                char  descriptor[DESCRIPTOR_BUFFER_LENGTH];
                char  uuid      [UUID_BUFFER_LENGTH];

                if ( current_procedure_count == 0 )
                {
                    rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
                    rest_smart_response_write_descriptor_array_start( stream );
                }

                unsigned_to_hex_string( (uint32_t)data->discovery_result.discovery_data.char_descr_info.handle, descriptor, 4, 4 );
                uuid_to_string( &data->discovery_result.discovery_data.char_descr_info.type, uuid );
                rest_smart_response_write_descriptor( stream, node, descriptor, data->discovery_result.discovery_data.char_descr_info.handle, uuid, ( current_procedure_count == 0 ) ? WICED_TRUE : WICED_FALSE );
                current_procedure_count++;
            }

            wiced_http_response_stream_flush( stream );
            break;
        }

        case GATT_DISCOVERY_CPLT_EVT:
        {
            wiced_http_response_stream_t* stream = (wiced_http_response_stream_t*)current_request->arg;

            if ( ( data->discovery_complete.status == WICED_BT_GATT_SUCCESS ) && ( current_procedure_count > 0 ) )
            {
                if ( data->discovery_complete.disc_type == GATT_DISCOVER_CHARACTERISTICS )
                {
                    char  characteristic[CHARACTERISTIC_BUFFER_LENGTH];
                    char  node          [NODE_BUFFER_LENGTH];
                    char  uuid          [UUID_BUFFER_LENGTH];

                    memset( &characteristic, 0, sizeof ( characteristic ) );
                    memset( &node, 0, sizeof( node ) );
                    memset( &uuid, 0, sizeof( uuid ) );

                    format_node_string( node, (const wiced_bt_device_address_t*)&connection->address.bda, connection->address.type );
                    format_characteristic_string( characteristic, current_characteristic.handle, current_request->parameter.discover.e_handle, current_characteristic.val_handle );
                    uuid_to_string( &current_characteristic.char_uuid, uuid );
                    rest_smart_response_write_characteristic( stream, node, characteristic, current_characteristic.handle, uuid, current_characteristic.characteristic_properties, ( current_procedure_count == 1 ) ? WICED_TRUE : WICED_FALSE );
                }

                /* Services and descriptors discovery */
                rest_smart_response_write_array_end( stream );
                rest_smart_response_end_stream( stream );
            }
            else
            {
                rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
            }

            current_procedure_count = 0;
            break;
        }

        case GATT_OPERATION_CPLT_EVT:
        {
            switch ( data->operation_complete.op )
            {
                case GATTC_OPTYPE_READ:
                {
                    if ( (uint32_t) current_request->arg == CCC_EVENT )
                    {
                        /* CCC read. Handle response */
                        handle_ccc_read_response( connection, &data->operation_complete );
                    }
                    else
                    {
                        /* Normal GATT read. Handle response */
                        handle_gatt_read_response( connection, current_request, &data->operation_complete );
                    }
                    break;
                }
                case GATTC_OPTYPE_WRITE:
                {

                    if ( (uint32_t) current_request->arg == CCC_EVENT )
                    {
                        /* CCC write. Handle response */
                        handle_ccc_write_response( connection, &data->operation_complete );
                    }
                    else if ( data->operation_complete.status == WICED_BT_GATT_SUCCESS )
                    {
                        /* Normal GATT write. Handle response */
                        wiced_http_response_stream_t* stream = (wiced_http_response_stream_t*)current_request->arg;
                        rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
                        rest_smart_response_end_stream( stream );
                    }
                    else
                    {
                        rest_smart_response_send_error_message( (wiced_http_response_stream_t*)current_request->arg, REST_SMART_STATUS_200 );
                    }
                    break;
                }
                case GATTC_OPTYPE_NOTIFICATION:
                case GATTC_OPTYPE_INDICATION:
                {
                    wiced_bool_t                  sse_subscribed = WICED_FALSE;
                    wiced_http_response_stream_t* stream = NULL;
                    rest_smart_cached_value_t*    cached_value;

                    /* Search if cached value is available */
                    wiced_rtos_lock_mutex( &list_mutex );
                    cached_value = find_cached_value_by_value_handle( connection, data->operation_complete.response_data.att_value.handle );
                    if ( ( cached_value != NULL ) )
                    {
                        /* Copy new value to cached value */
                        memset( cached_value->value.value, 0, CACHED_VALUE_BUFFER_LENGTH );
                        memcpy( cached_value->value.value, data->operation_complete.response_data.att_value.p_data, MIN( CACHED_VALUE_BUFFER_LENGTH, data->operation_complete.response_data.att_value.len ) );
                        sse_subscribed = cached_value->is_subscribed;
                        stream         = cached_value->sse_stream;
                    }
                    wiced_rtos_unlock_mutex( &list_mutex );
                    // indication should be acknowledged irrespective of sse subscription
                    if( data->operation_complete.op == GATTC_OPTYPE_INDICATION)
                    {
                        wiced_bt_gatt_send_indication_confirm( connection->connection_id, data->operation_complete.response_data.att_value.handle );
                    }

                    if ( ( sse_subscribed == WICED_TRUE ) && ( stream != NULL ) )
                    {
                        //Application callback for notification and indication are overloaded
                        rest_smart_response_send_notification( stream, &data->operation_complete.response_data.att_value );
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }

        case GATT_ATTRIBUTE_REQUEST_EVT:
        default:
        {
            break;
        }
    }

    return (status);
}

static wiced_result_t restful_interface_callback( big_gatt_interface_t* interface, wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* data )
{
    rest_smart_pairing_info_t*    pairing_info = NULL;
    rest_smart_pairing_response_t response;

    memset( &response, 0, sizeof( response ) );

    switch ( event )
    {
        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /**< Updated remote device link keys (store device_link_keys to  NV memory). Event data: #wiced_bt_device_link_keys_t */
            WPRINT_LIB_INFO( ( "BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT\n" ) );
            return WICED_BT_ERROR;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /**< Request for stored remote device link keys (restore device_link_keys from NV memory). If successful, return WICED_BT_SUCCESS. Event data: #wiced_bt_device_link_keys_t */
            WPRINT_LIB_INFO( ( "BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT\n" ) );
            return WICED_BT_ERROR;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /**< Update local identity key (stored local_identity_keys NV memory). Event data: #wiced_bt_local_identity_keys_t */
            WPRINT_LIB_INFO( ( "BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT\n" ) );
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /**< Request local identity key (get local_identity_keys from NV memory). If successful, return WICED_BT_SUCCESS. Event data: #wiced_bt_local_identity_keys_t */
            WPRINT_LIB_INFO( ( "BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT\n" ) );
            break;

        case BTM_SECURITY_REQUEST_EVT:
        {
            WPRINT_LIB_INFO( ( "BTM_SECURITY_REQUEST_EVT\n" ) );

            /* Peer requested pairing. Grant request */
            wiced_bt_ble_security_grant( data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;
        }
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        {
            /* Peer requested for I/O capabilities. Copy local I/O caps to stack */
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( &data->pairing_io_capabilities_ble_request.bd_addr );
            if ( pairing_info != NULL )
            {
                WPRINT_LIB_INFO( ( "Getting local I/O capabilities\n" ) );

                data->pairing_io_capabilities_ble_request.local_io_cap = pairing_info->io_caps.local_io_cap;
                data->pairing_io_capabilities_ble_request.oob_data     = pairing_info->io_caps.oob_data;
                data->pairing_io_capabilities_ble_request.auth_req     = pairing_info->io_caps.auth_req;
                data->pairing_io_capabilities_ble_request.max_key_size = pairing_info->io_caps.max_key_size;
                data->pairing_io_capabilities_ble_request.init_keys    = pairing_info->io_caps.init_keys;
                data->pairing_io_capabilities_ble_request.resp_keys    = pairing_info->io_caps.resp_keys;
            }
            wiced_rtos_unlock_mutex( &list_mutex );
            break;
        }
        case BTM_PASSKEY_REQUEST_EVT:
        {
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( &data->user_passkey_request.bd_addr );
            if ( pairing_info != NULL )
            {
                /* Update local data */
                pairing_info->status          = PASSKEY_INPUT_EXPECTED;
                pairing_info->expected_action = PASSKEY_INPUT_EXPECTED;
            }
            wiced_rtos_unlock_mutex( &list_mutex );

            if ( pairing_info != NULL )
            {
                /* Send pairing response to client */
                response.status_code         = REST_SMART_STATUS_200;
                response.pairing_status_code = PASSKEY_INPUT_EXPECTED;
                memcpy( response.pairing_id, pairing_info->pairing_id, sizeof( response.pairing_id ) );
                rest_smart_response_send_pairing_data( pairing_info->stream, &response );
            }

            break;
        }
        case BTM_PASSKEY_NOTIFICATION_EVT:
        {
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( &data->user_passkey_notification.bd_addr );
            if ( pairing_info != NULL )
            {
                pairing_info->status               = PASSKEY_DISPLAY_EXPECTED;
                pairing_info->expected_action      = PASSKEY_DISPLAY_EXPECTED;
                pairing_info->pairing_data.passkey = data->user_passkey_notification.passkey;
            }
            wiced_rtos_unlock_mutex( &list_mutex );

            if ( pairing_info != NULL )
            {
                response.status_code         = REST_SMART_STATUS_200;
                response.pairing_status_code = PASSKEY_DISPLAY_EXPECTED;
                response.passkey             = data->user_passkey_notification.passkey;
                memcpy( response.pairing_id, pairing_info->pairing_id, sizeof( response.pairing_id ) );
                rest_smart_response_send_pairing_data( pairing_info->stream, &response );
            }
            break;
        }
        case BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT:
        {
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( &data->remote_oob_data_request.bd_addr );
            if ( pairing_info != NULL )
            {
                pairing_info->status          = LE_LEGACY_OOB_EXPECTED;
                pairing_info->expected_action = LE_LEGACY_OOB_EXPECTED;
            }
            wiced_rtos_unlock_mutex( &list_mutex );

            if ( pairing_info != NULL )
            {
                response.status_code         = REST_SMART_STATUS_200;
                response.pairing_status_code = LE_LEGACY_OOB_EXPECTED;
                memcpy( response.pairing_id, pairing_info->pairing_id, sizeof( response.pairing_id ) );
                rest_smart_response_send_pairing_data( pairing_info->stream, &response );
            }
            break;
        }
        case BTM_USER_CONFIRMATION_REQUEST_EVT:
        {
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( &data->user_confirmation_request.bd_addr );
            if ( pairing_info != NULL )
            {
                pairing_info->status               = NUMERIC_COMPARISON_EXPECTED;
                pairing_info->expected_action      = NUMERIC_COMPARISON_EXPECTED;
                pairing_info->pairing_data.passkey = data->user_confirmation_request.numeric_value;
            }
            wiced_rtos_unlock_mutex( &list_mutex );

            if ( pairing_info != NULL )
            {
                response.status_code         = REST_SMART_STATUS_200;
                response.pairing_status_code = NUMERIC_COMPARISON_EXPECTED;
                response.passkey             = data->user_confirmation_request.numeric_value;
                memcpy( response.pairing_id, pairing_info->pairing_id, sizeof( response.pairing_id ) );
                rest_smart_response_send_pairing_data( pairing_info->stream, &response );
            }
            break;
        }
        case BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT:
        {
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( &data->user_confirmation_request.bd_addr );
            if ( pairing_info != NULL )
            {
                WPRINT_LIB_DEBUG( "LE secure connections OOB expected\n" );

                pairing_info->status = LE_SECURE_OOB_EXPECTED;
                pairing_info->pairing_data.secure_oob_data.peer_oob_data.present  = WICED_FALSE;
                pairing_info->pairing_data.secure_oob_data.local_oob_data.present = WICED_TRUE;

                memcpy( &pairing_info->pairing_data.secure_oob_data.local_oob_data.randomizer,       &data->p_smp_sc_local_oob_data->randomizer,       sizeof( BT_OCTET16 ) );
                memcpy( &pairing_info->pairing_data.secure_oob_data.local_oob_data.commitment,       &data->p_smp_sc_local_oob_data->commitment,       sizeof( BT_OCTET16 ) );
                memcpy( &pairing_info->pairing_data.secure_oob_data.local_oob_data.addr_sent_to,     &data->p_smp_sc_local_oob_data->addr_sent_to,     sizeof( tBLE_BD_ADDR ) );
                memcpy( &pairing_info->pairing_data.secure_oob_data.local_oob_data.private_key_used, &data->p_smp_sc_local_oob_data->private_key_used, sizeof( BT_OCTET32 ) );
                memcpy( &pairing_info->pairing_data.secure_oob_data.local_oob_data.public_key_used,  &data->p_smp_sc_local_oob_data->public_key_used,  sizeof( wiced_bt_public_key_t ) );
            }
            wiced_rtos_unlock_mutex( &list_mutex );

            if ( pairing_info != NULL )
            {
                /* TODO: send response here */
            }

            break;
        }
        case BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT:
        {
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( &data->user_confirmation_request.bd_addr );
            if ( pairing_info != NULL )
            {
                switch ( data->smp_sc_remote_oob_data_request.oob_type )
                {
                    case BTM_OOB_PEER:
                        WPRINT_LIB_INFO(( "oob_type = BTM_OOB_PEER\n" ));
                        pairing_info->pairing_data.secure_oob_data.local_oob_data.present = WICED_FALSE;
                        pairing_info->pairing_data.secure_oob_data.peer_oob_data.present  = WICED_TRUE;
                        break;
                    case BTM_OOB_LOCAL:
                        WPRINT_LIB_INFO(( "oob_type = BTM_OOB_LOCAL\n" ));
                        pairing_info->pairing_data.secure_oob_data.local_oob_data.present = WICED_TRUE;
                        pairing_info->pairing_data.secure_oob_data.peer_oob_data.present  = WICED_FALSE;
                        break;
                    case BTM_OOB_BOTH:
                        WPRINT_LIB_INFO(( "oob_type = BTM_OOB_BOTH\n" ));
                        break;
                    default:
                        break;
                }
                wiced_rtos_lock_mutex( &list_mutex );
                wiced_bt_smp_sc_oob_reply( (uint8_t*)&pairing_info->pairing_data.secure_oob_data );
            }
            wiced_rtos_unlock_mutex( &list_mutex );
            break;
        }
        case BTM_PAIRING_COMPLETE_EVT:
        {
            /* Don't send if passkey display pairing is used. Send response only when client sends its response */
            wiced_bool_t send_response = WICED_FALSE;

            /* Pairing completed. Store bond info. Notify user interface */
            wiced_rtos_lock_mutex( &list_mutex );
            pairing_info = find_pairing_node_info( (wiced_bt_device_address_t*) data->pairing_complete.bd_addr );
            if ( pairing_info != NULL )
            {
                send_response = ( pairing_info->expected_action == PASSKEY_DISPLAY_EXPECTED ) ? WICED_FALSE : WICED_TRUE;

                if ( data->pairing_complete.pairing_complete_info.ble.status == WICED_SUCCESS )
                {
                    pairing_info->status = PAIRING_SUCCESSFUL;
                }
                else if ( data->pairing_complete.pairing_complete_info.ble.is_pair_cancel == WICED_TRUE )
                {
                    pairing_info->status = PAIRING_ABORTED;
                }
                else
                {
                    pairing_info->status = PAIRING_FAILED;
                    pairing_info->pairing_data.error_reason = data->pairing_complete.pairing_complete_info.ble.reason;
                }
            }
            wiced_rtos_unlock_mutex( &list_mutex );

            if ( ( pairing_info != NULL ) && ( send_response == WICED_TRUE ) )
            {
                if ( data->pairing_complete.pairing_complete_info.ble.status == WICED_SUCCESS )
                {
                    response.pairing_status_code = PAIRING_SUCCESSFUL;
                    response.status_code         = REST_SMART_STATUS_200;

                }
                else if ( data->pairing_complete.pairing_complete_info.ble.is_pair_cancel == WICED_TRUE )
                {
                    response.pairing_status_code = PAIRING_ABORTED;
                    response.status_code         = REST_SMART_STATUS_200;
                }
                else
                {
                    response.pairing_status_code = PAIRING_FAILED;
                    response.status_code         = REST_SMART_STATUS_403;
                    response.reason_code         = data->pairing_complete.pairing_complete_info.ble.reason;
                }

                rest_smart_response_send_pairing_data( pairing_info->stream, &response );

                wiced_rtos_lock_mutex( &list_mutex );
                remove_pairing_node_info( pairing_info );
                wiced_rtos_unlock_mutex( &list_mutex );
            }
            break;
        }
        default:
            break;
    }

    return WICED_BT_SUCCESS;
}

static void restful_smart_scan_result_callback( wiced_bt_ble_scan_results_t *result, uint8_t *data )
{
    if ( result != 0 )
    {
        char    node_handle[14]    = { 0 };
        char    bd_addr[13]        = { 0 };
        char    rssi[5]            = { 0 };
        char    bdaddrtype[2]      = { 0 };
        char    adv_type_string[2] = { 0 };
        uint8_t adv_type;
        uint8_t adv_data_length;
        char    adv_data[31*2] = {0};
        uint8_t *p = data;
        uint8_t i;
        uint32_t j;

        format_node_string( node_handle, (const wiced_bt_device_address_t*)&result->remote_bd_addr, (uint8_t)result->ble_addr_type );
        device_address_to_string( (const wiced_bt_device_address_t*)&result->remote_bd_addr, bd_addr );
        signed_to_decimal_string( (int32_t)result->rssi, rssi, 1, 5 );
        unsigned_to_hex_string( (uint32_t)result->ble_addr_type, bdaddrtype, 1, 2 );
        rest_smart_response_write_gap_node( current_stream, (const char*)node_handle, (const char*)bd_addr, (const char*)bdaddrtype, (const char*)rssi, ( scan_result_count == 0 ) ? WICED_TRUE : WICED_FALSE );
        rest_smart_response_write_adv_array_start( current_stream );

        STREAM_TO_UINT8( adv_data_length, p );

        j = 0;

        while( adv_data_length && (p - data <= 31 ) )
        {
            memset( adv_data, 0, sizeof( adv_data ) );

            STREAM_TO_UINT8( adv_type, p );
            unsigned_to_decimal_string( (uint32_t)adv_type, adv_type_string, 1, 2 );

            for( i = 0; i < ( adv_data_length-1 ); i++ )
            {
                unsigned_to_hex_string( (uint32_t)p[i], &adv_data[i*2], 2, 2 );
            }

            rest_smart_response_write_adv_data( current_stream, (const char*)adv_type_string, (const char*)adv_data, ( j == 0 ) ? WICED_TRUE : WICED_FALSE );

            p += adv_data_length - 1; /* skip the length of data */
            STREAM_TO_UINT8( adv_data_length, p );
            j++;
        }

        rest_smart_response_write_array_end( current_stream );
        scan_result_count++;
    }
    else
    {
        WPRINT_LIB_INFO(( "REST: LE scan completed.\n" ));
        rest_smart_response_write_array_end( current_stream );
        rest_smart_response_end_stream( current_stream );
    }
}

static wiced_bool_t compare_pairing_info_callback( linked_list_node_t* node_to_compare, void* user_data )
{
    rest_smart_pairing_info_t* current_pairing_info = (rest_smart_pairing_info_t*)node_to_compare;
    wiced_bt_device_address_t* address           = (wiced_bt_device_address_t*)user_data;

    if ( memcmp( &current_pairing_info->node_handle.bda, address, sizeof( wiced_bt_device_address_t ) ) == 0 )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static rest_smart_pairing_info_t* find_pairing_node_info( wiced_bt_device_address_t* address )
{
    rest_smart_pairing_info_t* pairing_info = NULL;

    linked_list_find_node( &active_pairing_node_list, compare_pairing_info_callback, (void*)address, (linked_list_node_t**)&pairing_info );

    return pairing_info;
}

static rest_smart_pairing_info_t* create_pairing_node_info( smart_node_handle_t* node )
{
    rest_smart_pairing_info_t* pairing_info = NULL;
    wiced_result_t result;

    result = linked_list_remove_node_from_front( &inactive_pairing_node_list, (linked_list_node_t**)&pairing_info );
    if ( result != WICED_SUCCESS )
    {
        return NULL;
    }

    memset( pairing_info, 0, sizeof( *pairing_info ) );

    memcpy( &pairing_info->node_handle, node, sizeof( smart_node_handle_t ) );
    pairing_info->status = PAIRING_ABORTED;

    result = linked_list_insert_node_at_rear( &active_pairing_node_list, &pairing_info->pairing_info_node );
    if ( result != WICED_SUCCESS )
    {
        return NULL;
    }

    return pairing_info;
}

static wiced_result_t remove_pairing_node_info( rest_smart_pairing_info_t* pairing_node_info )
{
    wiced_result_t result;

    result = linked_list_remove_node( &active_pairing_node_list, &pairing_node_info->pairing_info_node );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    return linked_list_insert_node_at_rear( &inactive_pairing_node_list, &pairing_node_info->pairing_info_node );
}

static rest_smart_cached_value_t* create_cached_value( big_gatt_connection_t* connection, const smart_characteristic_handle_t* characteristic )
{
    rest_smart_cached_value_t* cached_value = NULL;
    uint32_t size = sizeof( rest_smart_cached_value_t ) + wiced_bt_cfg_settings.gatt_cfg.max_attr_len;

    if ( linked_list_remove_node_from_front( &inactive_cached_value_list, (linked_list_node_t**)&cached_value ) != WICED_SUCCESS )
    {
        return NULL;
    }

    memset( cached_value, 0, size );

    cached_value->connection     = connection;
    cached_value->characteristic = *characteristic;

    if ( linked_list_insert_node_at_rear( &active_cached_value_list, &cached_value->value_node ) != WICED_SUCCESS )
    {
        return NULL;
    }

    return cached_value;
}

static wiced_bool_t compare_cached_value( linked_list_node_t* node_to_compare, void* arg )
{
    rest_smart_cached_value_t* current_cached_value = (rest_smart_cached_value_t*)node_to_compare;
    rest_smart_cached_value_t* value_to_find        = (rest_smart_cached_value_t*)arg;

    if ( ( current_cached_value->connection == value_to_find->connection ) && ( current_cached_value->characteristic.value_handle == value_to_find->characteristic.value_handle ) )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static rest_smart_cached_value_t* find_cached_value_by_value_handle( big_gatt_connection_t* connection, uint16_t value_handle )
{
    rest_smart_cached_value_t* cached_value_found = NULL;
    rest_smart_cached_value_t  value_to_compare;

    value_to_compare.connection                  = connection;
    value_to_compare.characteristic.value_handle = value_handle;

    linked_list_find_node( &active_cached_value_list, compare_cached_value, (void*)&value_to_compare, (linked_list_node_t**)&cached_value_found );

    return cached_value_found;
}

static wiced_bool_t compare_ccc_handle( linked_list_node_t* node_to_compare, void* arg )
{
    rest_smart_cached_value_t* current_cached_value = (rest_smart_cached_value_t*)node_to_compare;
    rest_smart_cached_value_t* value_to_find        = (rest_smart_cached_value_t*)arg;

    if ( ( current_cached_value->connection == value_to_find->connection ) &&
         ( value_to_find->ccc_handle >= current_cached_value->characteristic.value_handle + 1 ) &&
         ( value_to_find->ccc_handle <= current_cached_value->characteristic.end_handle ) )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static rest_smart_cached_value_t* find_cached_value_by_ccc_handle( big_gatt_connection_t* connection, uint16_t ccc_handle )
{
    rest_smart_cached_value_t* cached_value_found = NULL;
    rest_smart_cached_value_t  value_to_compare;

    value_to_compare.connection = connection;
    value_to_compare.ccc_handle = ccc_handle;

    linked_list_find_node( &active_cached_value_list, compare_ccc_handle, (void*)&value_to_compare, (linked_list_node_t**)&cached_value_found );

    return cached_value_found;
}

static wiced_result_t remove_cached_value( rest_smart_cached_value_t* value )
{
    wiced_result_t result;

    result = linked_list_remove_node( &active_cached_value_list, &value->value_node );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }
    return linked_list_insert_node_at_front( &inactive_cached_value_list, &value->value_node );
}

static wiced_result_t write_client_characteristic_config( big_gatt_connection_t* connection, uint16_t ccc_handle, wiced_bt_gatt_client_char_config_t config, void* arg )
{
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        return result;
    }

    /* Fill out request parameters */
    request->feature = BIG_GATT_WRITE_CHARACTERISTIC_VALUE;
    request->arg     = arg;
    request->parameter.write.auth_req = GATT_AUTH_REQ_NONE;
    request->parameter.write.handle   = ccc_handle;
    request->parameter.write.len      = sizeof( config );
    request->parameter.write.offset   = 0;
    memcpy( request->parameter.write.value, (void*)&config, sizeof( config ) );

    result = big_send_gatt_request( connection, buffer );
    if ( ( result != WICED_SUCCESS ) && ( buffer != NULL ) )
    {
        big_release_gatt_request_buffer( buffer );
    }
    return result;
}

static wiced_result_t read_client_characteristic_config( big_gatt_connection_t* connection, const smart_characteristic_handle_t* characteristic, void* arg )
{
    big_gatt_request_buffer_t* buffer  = NULL;
    big_gatt_request_t*        request = NULL;
    wiced_result_t             result  = WICED_NOT_FOUND;

    result = big_get_gatt_request_buffer( &buffer, &request );
    if ( result != WICED_BT_SUCCESS )
    {
        return result;
    }

    /* Find attribute with UUID = 0x2902 */
    request->feature = BIG_GATT_READ_CHARACTERISTICS_BY_UUID;
    request->arg     = arg;
    request->parameter.read.char_type.auth_req       = GATT_AUTH_REQ_NONE;
    request->parameter.read.char_type.s_handle       = characteristic->value_handle + 1;
    request->parameter.read.char_type.e_handle       = characteristic->end_handle;
    request->parameter.read.char_type.uuid.len       = LEN_UUID_16;
    request->parameter.read.char_type.uuid.uu.uuid16 = CCC_UUID;

    result = big_send_gatt_request( connection, buffer );
    if ( ( result != WICED_SUCCESS ) && ( buffer != NULL ) )
    {
        big_release_gatt_request_buffer( buffer );
    }
    return result;

}

static wiced_result_t handle_gatt_read_response( big_gatt_connection_t* connection, const big_gatt_request_t* current_request, wiced_bt_gatt_operation_complete_t* data )
{
    wiced_http_response_stream_t* stream = (wiced_http_response_stream_t*)current_request->arg;
    char node[NODE_BUFFER_LENGTH] = { 0 };

    if ( data->status != WICED_BT_GATT_SUCCESS )
    {
        if ( ( current_request->feature == BIG_GATT_READ_LONG_CHARACTERISTIC_VALUE ) && ( current_request->parameter.read.partial.offset > 0 ) )
        {
            /* Not the first partial read of a long value. Close the stream */
            rest_smart_response_write_long_characteristic_value_end( stream );
            rest_smart_response_end_stream( stream );
        }
        else
        {
            rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
        }
        return WICED_ERROR;
    }

    device_address_to_string( (const wiced_bt_device_address_t*)&connection->address.bda, node );

    switch ( current_request->feature )
    {
        case BIG_GATT_READ_CHARACTERISTIC_VALUE:
        {
            char characteristic[CHARACTERISTIC_BUFFER_LENGTH] = { 0 };

            format_characteristic_string( characteristic, current_characteristic_handle.start_handle, current_characteristic_handle.end_handle, current_characteristic_handle.value_handle );
            rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
            rest_smart_response_write_characteristic_value( stream, node, characteristic, data->response_data.att_value.handle, data->response_data.att_value.p_data, data->response_data.att_value.len );
            rest_smart_response_end_stream( stream );
            break;
        }
        case BIG_GATT_READ_CHARACTERISTICS_BY_UUID:
        {
            char characteristic[CHARACTERISTIC_BUFFER_LENGTH] = { 0 };

            format_characteristic_string( characteristic, current_characteristic_handle.start_handle, current_characteristic_handle.end_handle, current_characteristic_handle.value_handle );
            rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
            rest_smart_response_write_characteristic_value( stream, node, characteristic, data->response_data.att_value.handle, data->response_data.att_value.p_data, data->response_data.att_value.len );
            rest_smart_response_end_stream( stream );
            break;
        }

        case BIG_GATT_READ_CHARACTERISTIC_DESCRIPTOR:
        {
            char descriptor[DESCRIPTOR_BUFFER_LENGTH] = { 0 };

            unsigned_to_hex_string( data->response_data.att_value.handle, descriptor, 4, 4 );
            rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
            rest_smart_response_write_descriptor_value( stream, node, descriptor, data->response_data.att_value.p_data, data->response_data.att_value.len );
            rest_smart_response_end_stream( stream );
            break;
        }

        /* BIG_GATT_READ_LONG_CHARACTERISTIC_VALUE/DESCRIPTOR aren't used. For long read, BIG_GATT_READ_CHARACTERISTIC_VALUE/DESCRIPTOR are used */
        case BIG_GATT_READ_LONG_CHARACTERISTIC_VALUE:
        case BIG_GATT_READ_LONG_CHARACTERISTIC_DESCRIPTOR:
        case BIG_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES:
        {
            /* TODO: Currently unsupported */
            rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
            break;
        }

        default:
            break;
    }

    return WICED_BT_SUCCESS;
}

static wiced_result_t handle_ccc_read_response( big_gatt_connection_t* connection, wiced_bt_gatt_operation_complete_t* data )
{
    wiced_http_response_stream_t*      stream       = NULL;
    rest_smart_cached_value_t*         cached_value = NULL;
    wiced_bt_gatt_client_char_config_t config;

    /* Find cached value */
    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_ccc_handle( connection, data->response_data.att_value.handle );
    if ( cached_value != NULL )
    {
        cached_value->ccc_handle = data->response_data.att_value.handle;
        stream = cached_value->sse_stream;
        config = cached_value->config;
        wiced_rtos_unlock_mutex( &list_mutex );
        /* Write value to CCC */
        if ( write_client_characteristic_config( connection, data->response_data.att_value.handle, config, (void*)CCC_EVENT ) != WICED_SUCCESS )
        {
            rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
        }
    }
    else
    {
        wiced_rtos_unlock_mutex( &list_mutex );
    }

    return WICED_BT_SUCCESS;
}

static wiced_result_t handle_ccc_write_response( big_gatt_connection_t* connection, wiced_bt_gatt_operation_complete_t* data )
{
    wiced_http_response_stream_t*      stream       = NULL;
    rest_smart_cached_value_t*         cached_value = NULL;

    /* Find cached value */
    wiced_rtos_lock_mutex( &list_mutex );
    cached_value = find_cached_value_by_ccc_handle( connection, data->response_data.att_value.handle );
    if ( cached_value != NULL )
    {
        stream = cached_value->sse_stream;
        if ( cached_value->config == GATT_CLIENT_CONFIG_NONE )
        {
            remove_cached_value( cached_value );
        }
        wiced_rtos_unlock_mutex( &list_mutex );

        if ( data->status == WICED_BT_SUCCESS )
        {
            rest_smart_response_send_error_message( stream, REST_SMART_STATUS_200 );
        }
        else
        {
            rest_smart_response_send_error_message( stream, REST_SMART_STATUS_404 );
        }
    }
    else
    {
        wiced_rtos_unlock_mutex( &list_mutex );
    }

    return WICED_BT_SUCCESS;
}
