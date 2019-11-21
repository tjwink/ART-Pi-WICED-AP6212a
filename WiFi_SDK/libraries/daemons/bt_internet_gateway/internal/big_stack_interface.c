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
#include <string.h>
#include <stdlib.h>
#include "big_stack_interface.h"
#include "linked_list.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "bt_types.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define BIG_OBJECT_UNIQUE_ID          ( 0xFEEDBEEF )
#define STACK_INIT_DEINIT_TIMEOUT_MS  ( 30 * SECONDS )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * BIG GATT request buffer. Typedef declared in header file.
 */
struct big_gatt_request_buffer
{
   linked_list_node_t buffer_node;
   wiced_bool_t       free_buffer;
   big_gatt_request_t request;
};

typedef struct
{
    big_gatt_interface_t* interface;
    uint16_t              connection_id;
} compare_connection_id_t;

typedef struct
{
    big_gatt_interface_t*   interface;
    wiced_bt_ble_address_t* address;
} compare_ble_address_t;

typedef struct
{
    big_gatt_interface_t*      interface;
    wiced_bt_device_address_t* address;
} compare_bt_device_address_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t         connect_timeout_callback           ( void* arg );
static wiced_bt_gatt_status_t big_gatt_callback                  ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *data );
static wiced_bt_dev_status_t  big_stack_callback                 ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* data );
static wiced_bool_t           big_match_bt_device_addresses      ( wiced_bt_device_address_t* address1, wiced_bt_device_address_t* address2 );
static wiced_bool_t           big_match_ble_addresses            ( wiced_bt_ble_address_t* address1, wiced_bt_ble_address_t* address2 );
static wiced_result_t         send_gatt_request                  ( big_gatt_connection_t* connection, big_gatt_request_buffer_t* request_buffer );
static wiced_result_t         pop_next_gatt_request              ( big_gatt_connection_t* connection, big_gatt_request_buffer_t** request_buffer );
static wiced_bool_t           compare_connection_id              ( linked_list_node_t* node_to_compare, void* user_data );
static wiced_bool_t           compare_ble_address                ( linked_list_node_t* node_to_compare, void* user_data );
static wiced_bool_t           compare_bt_device_address          ( linked_list_node_t* node_to_compare, void* user_data );
static wiced_bool_t           internal_compare_connection_id     ( linked_list_node_t* node_to_compare, void* user_data );
static big_gatt_connection_t* internal_find_connection_by_id     ( uint16_t connection_id );
static wiced_bool_t           internal_compare_bt_device_address ( linked_list_node_t* node_to_compare, void* user_data );
static big_gatt_connection_t* internal_find_connection_by_address( wiced_bt_device_address_t* ble_address );
static big_gatt_connection_t* add_new_connection                 ( big_gatt_interface_t* interface, wiced_bt_ble_address_t* address, uint16_t connection_id );
static wiced_result_t         remove_exisiting_connection        ( big_gatt_connection_t* connection );

/******************************************************
 *               Variables Definitions
 ******************************************************/

/* Externed from application for stack config */
extern wiced_bt_cfg_settings_t       wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];

/* Initalisation globals */
static wiced_semaphore_t             init_semaphore;
static wiced_result_t                init_result = WICED_SUCCESS;

/* GATT app interface globals */
static wiced_mutex_t                 big_gatt_mutex;
static big_gatt_interface_t          gatt_server_interface;
static linked_list_t                 gatt_client_interface_list;

/* GATT connection globals */
static big_gatt_connection_t*        connection_buffer    = NULL;
static big_gatt_interface_t*         connecting_interface = NULL;
static linked_list_t                 active_connection_list;
static linked_list_t                 inactive_connection_list;
static wiced_bt_ble_address_t        connecting_peer_address;
static wiced_timed_event_t           connect_timeout_event;

/* GATT request globals */
static big_gatt_request_buffer_t*    current_gatt_request = NULL;
static big_gatt_request_buffer_t*    request_buffer_pool  = NULL;
static linked_list_t                 free_request_buffer_list;

static const uint8_t gatt_feature_to_type[] =
{
    [BIG_GATT_EXCHANGE_MTU                             ] = 0, /* Arbitrary value. Unused */
    [BIG_GATT_DISCOVER_ALL_PRIMARY_SERVICES            ] = GATT_DISCOVER_SERVICES_ALL,
    [BIG_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID        ] = GATT_DISCOVER_SERVICES_BY_UUID,
    [BIG_GATT_DISCOVER_INCLUDED_SERVICES               ] = GATT_DISCOVER_INCLUDED_SERVICES,
    [BIG_GATT_DISCOVER_ALL_CHARACTERISTICS_OF_A_SERVICE] = GATT_DISCOVER_CHARACTERISTICS,
    [BIG_GATT_DISCOVER_CHARACTERISTICS_BY_UUID         ] = GATT_DISCOVER_CHARACTERISTICS,
    [BIG_GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS      ] = GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,
    [BIG_GATT_READ_CHARACTERISTIC_VALUE                ] = GATT_READ_BY_HANDLE,
    [BIG_GATT_READ_CHARACTERISTICS_BY_UUID             ] = GATT_READ_CHAR_VALUE,
    [BIG_GATT_READ_LONG_CHARACTERISTIC_VALUE           ] = GATT_READ_PARTIAL,
    [BIG_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES      ] = GATT_READ_MULTIPLE,
    [BIG_GATT_READ_CHARACTERISTIC_DESCRIPTOR           ] = GATT_READ_BY_HANDLE,
    [BIG_GATT_READ_LONG_CHARACTERISTIC_DESCRIPTOR      ] = GATT_READ_PARTIAL,
    [BIG_GATT_WRITE_CHARACTERISTIC_VALUE               ] = GATT_WRITE,
    [BIG_GATT_WRITE_LONG_CHARACTERISTIC_VALUE          ] = GATT_WRITE,
    [BIG_GATT_WRITE_WITHOUT_RESPONSE                   ] = GATT_WRITE_NO_RSP,
    [BIG_GATT_WRITE_SIGNED_WITHOUT_RESPONSE            ] = GATT_WRITE_NO_RSP,
    [BIG_GATT_WRITE_CHARACTERISTIC_DESCRIPTOR          ] = GATT_WRITE,
    [BIG_GATT_WRITE_LONG_CHARACTERISTIC_DESCRIPTOR     ] = GATT_WRITE,
    [BIG_GATT_PREPARE_RELIABLE_WRITE                   ] = GATT_WRITE_PREPARE,
    [BIG_GATT_EXECUTE_RELIABLE_WRITE                   ] = 0, /* Arbitrary value. Unused */
    [BIG_GATT_CONFIRM_INDICATION                       ] = 0, /* Arbitrary value. Unused */
};

/* Security globals */
static big_local_keys_callback_t local_keys_callback = NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t big_init_stack_interface( big_local_keys_callback_t callback )
{
    uint32_t       request_buffer_size    = sizeof( big_gatt_request_buffer_t ) + wiced_bt_cfg_settings.gatt_cfg.max_attr_len;
    uint32_t       request_pool_size      = BIG_GATT_REQUEST_BUFFER_COUNT * request_buffer_size;
    uint32_t       max_connections        = wiced_bt_cfg_settings.gatt_cfg.client_max_links + wiced_bt_cfg_settings.gatt_cfg.server_max_links;
    uint32_t       connection_buffer_size = max_connections * sizeof( big_gatt_connection_t );
    uint32_t       a;
    wiced_result_t result;
    big_gatt_request_buffer_t* current_buffer = NULL;

    memset( &gatt_server_interface, 0, sizeof( gatt_server_interface ) );

    wiced_rtos_init_semaphore( &init_semaphore );
    wiced_rtos_init_mutex( &big_gatt_mutex );
    linked_list_init( &active_connection_list );
    linked_list_init( &inactive_connection_list );
    linked_list_init( &gatt_client_interface_list );
    linked_list_init( &free_request_buffer_list );

    /* Allocate request buffer */
    request_buffer_pool = (big_gatt_request_buffer_t*) malloc_named( "BIG request buffer", request_pool_size );
    if ( request_buffer_pool == NULL )
    {
        result = WICED_BT_OUT_OF_HEAP_SPACE;
        goto cleanup;
    }
    memset( request_buffer_pool, 0, request_pool_size );
    current_buffer = request_buffer_pool;
    for ( a = 0; a < BIG_GATT_REQUEST_BUFFER_COUNT; a++ )
    {
        current_buffer->free_buffer = WICED_TRUE;
        linked_list_insert_node_at_rear( &free_request_buffer_list, &current_buffer->buffer_node );
        current_buffer = (big_gatt_request_buffer_t*)( (uint8_t*)current_buffer + request_buffer_size );
    }

    /* Allocate buffer for connections */
    connection_buffer = (big_gatt_connection_t*) malloc_named( "BIG connection buffer", connection_buffer_size );
    if ( connection_buffer == NULL )
    {
        result = WICED_BT_OUT_OF_HEAP_SPACE;
        goto cleanup;
    }
    memset( connection_buffer, 0, connection_buffer_size );
    for ( a = 0; a < max_connections; a++ )
    {
        wiced_rtos_init_mutex( &connection_buffer[a].mutex );
        linked_list_insert_node_at_rear( &inactive_connection_list, &connection_buffer[a].connection_node );
    }

    local_keys_callback = callback;

    /* Initialize BT host stack and controller */
    result = wiced_bt_stack_init( big_stack_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
    if ( result != WICED_SUCCESS )
    {
        goto cleanup;
    }

    /* Wait for stack initialisation to complete */
    result = wiced_rtos_get_semaphore( &init_semaphore, STACK_INIT_DEINIT_TIMEOUT_MS );
    if ( result != WICED_SUCCESS )
    {
        goto cleanup;
    }

    /* Check init result from stack callback */
    if ( init_result != WICED_SUCCESS )
    {
        result = init_result;
        goto cleanup;
    }

    /* Register generic GATT callback */
    return wiced_bt_gatt_register( big_gatt_callback );

    cleanup:
    wiced_rtos_deinit_semaphore( &init_semaphore );
    wiced_rtos_deinit_mutex( &big_gatt_mutex );
    linked_list_deinit( &gatt_client_interface_list );
    linked_list_deinit( &active_connection_list );
    linked_list_deinit( &free_request_buffer_list );
    if ( connection_buffer != NULL )
    {
        free( connection_buffer );
    }
    if ( request_buffer_pool != NULL )
    {
        free( request_buffer_pool );
    }
    return result;
}

wiced_result_t big_deinit_stack_interface( void )
{
    /* TODO: Deinit BT stack */
    local_keys_callback = NULL;
    linked_list_deinit( &gatt_client_interface_list );
    linked_list_deinit( &inactive_connection_list );
    linked_list_deinit( &active_connection_list );
    linked_list_deinit( &free_request_buffer_list );
    wiced_rtos_deinit_semaphore( &init_semaphore );
    wiced_rtos_deinit_mutex( &big_gatt_mutex );
    free( connection_buffer );
    free( request_buffer_pool );
    return WICED_SUCCESS;
}

wiced_result_t big_init_gatt_server_interface( big_gatt_interface_callback_t interface_callback, big_gatt_connection_callback_t connection_callback, big_peer_device_link_keys_callback_t keys_callback )
{
    memset( &gatt_server_interface.interface_node, 0, sizeof( gatt_server_interface.interface_node ) );
    gatt_server_interface.interface_callback  = (uint32_t)interface_callback;
    gatt_server_interface.connection_callback = (uint32_t)connection_callback;
    gatt_server_interface.keys_callback       = (uint32_t)keys_callback;
    gatt_server_interface.object_id           = BIG_OBJECT_UNIQUE_ID;
    return WICED_SUCCESS;
}

wiced_result_t big_deinit_gatt_server_interface( void )
{
    memset( &gatt_server_interface.interface_node, 0, sizeof( gatt_server_interface.interface_node ) );
    return WICED_SUCCESS;
}

wiced_result_t big_add_gatt_client_interface( big_gatt_interface_t* interface, big_gatt_interface_callback_t interface_callback, big_gatt_connection_callback_t connection_callback, big_peer_device_link_keys_callback_t keys_callback )
{
    wiced_result_t result;

    memset( &interface->interface_node, 0, sizeof( interface->interface_node ) );
    interface->object_id           = BIG_OBJECT_UNIQUE_ID;
    interface->interface_callback  = (uint32_t)interface_callback;
    interface->connection_callback = (uint32_t)connection_callback;
    interface->keys_callback       = (uint32_t )keys_callback;

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_insert_node_at_front( &gatt_client_interface_list, &interface->interface_node );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    return result;
}

wiced_result_t big_remove_gatt_client_interface( big_gatt_interface_t* interface )
{
    wiced_result_t result;

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_remove_node( &gatt_client_interface_list, &interface->interface_node );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    return result;
}

wiced_result_t big_ble_gatt_client_connect( big_gatt_interface_t* interface, wiced_bt_ble_address_t* address )
{
    big_gatt_connection_t* connection;

    wiced_assert( "bad arg", ( interface != NULL ) && ( address != NULL ) );

    if ( connecting_interface != NULL )
    {
        return WICED_BT_CONNECT_IN_PROGRESS;
    }

    connection = big_find_gatt_connection_by_ble_address( interface, address );
    if ( connection != NULL )
    {
        return WICED_BT_SOCKET_IN_USE;
    }

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    connecting_interface = interface;
    memcpy( &connecting_peer_address, address, sizeof( connecting_peer_address ) );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    if ( wiced_bt_gatt_le_connect( address->bda, address->type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE ) == WICED_TRUE )
    {
        wiced_rtos_register_timed_event( &connect_timeout_event, WICED_NETWORKING_WORKER_THREAD, connect_timeout_callback, BIG_GATT_CONNECT_TIMEOUT_MS, NULL );
        return WICED_SUCCESS;
    }
    else
    {
        wiced_rtos_lock_mutex( &big_gatt_mutex );
        connecting_interface = NULL;
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
        return WICED_ERROR;
    }
}

wiced_result_t big_ble_gatt_client_cancel_connect( void )
{
    if ( connecting_interface == NULL )
    {
        return WICED_BT_BADARG;
    }

    return wiced_bt_gatt_cancel_connect( connecting_peer_address.bda, WICED_TRUE );
}

wiced_result_t big_ble_gatt_disconnect( big_gatt_connection_t* connection )
{
    wiced_assert( "bad arg", ( connection != NULL ) );

    return wiced_bt_gatt_disconnect( connection->connection_id );
}

static wiced_bool_t compare_connection_id( linked_list_node_t* node_to_compare, void* user_data )
{
    big_gatt_connection_t*   connection               = (big_gatt_connection_t*)node_to_compare;
    compare_connection_id_t* connection_id_to_compare = (compare_connection_id_t*)user_data;

    if ( ( connection->owner_interface == connection_id_to_compare->interface ) && ( connection->connection_id == connection_id_to_compare->connection_id ) )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

big_gatt_connection_t* big_find_gatt_connection_by_id( big_gatt_interface_t* interface, uint16_t connection_id )
{
    big_gatt_connection_t*  connection = NULL;
    compare_connection_id_t compare_param;
    wiced_result_t          result;

    compare_param.connection_id = connection_id;
    compare_param.interface     = interface;

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_find_node( &active_connection_list, compare_connection_id, (void*)&compare_param, (linked_list_node_t**) &connection );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    return ( result == WICED_SUCCESS ) ? connection : NULL;
}

static wiced_bool_t compare_ble_address( linked_list_node_t* node_to_compare, void* user_data )
{
    big_gatt_connection_t* connection               = (big_gatt_connection_t*)node_to_compare;
    compare_ble_address_t* connection_id_to_compare = (compare_ble_address_t*)user_data;

    if ( ( connection->owner_interface == connection_id_to_compare->interface ) && ( big_match_ble_addresses( &connection->address, connection_id_to_compare->address ) == WICED_TRUE ) )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

big_gatt_connection_t* big_find_gatt_connection_by_ble_address( big_gatt_interface_t* interface, wiced_bt_ble_address_t* ble_address )
{
    big_gatt_connection_t* connection = NULL;
    compare_ble_address_t  compare_param;
    wiced_result_t         result;

    compare_param.address   = ble_address;
    compare_param.interface = interface;

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_find_node( &active_connection_list, compare_ble_address, (void*)&compare_param, (linked_list_node_t**) &connection );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    return ( result == WICED_SUCCESS ) ? connection : NULL;
}

static wiced_bool_t compare_bt_device_address( linked_list_node_t* node_to_compare, void* user_data )
{
    big_gatt_connection_t*       connection         = (big_gatt_connection_t*)node_to_compare;
    compare_bt_device_address_t* address_to_compare = (compare_bt_device_address_t*)user_data;

    if ( ( connection->owner_interface == address_to_compare->interface ) && ( big_match_bt_device_addresses( &connection->address.bda, address_to_compare->address ) == WICED_TRUE ) )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

big_gatt_connection_t* big_find_gatt_connection_by_device_address( big_gatt_interface_t* interface, wiced_bt_device_address_t* device_address )
{
    big_gatt_connection_t*      connection = NULL;
    compare_bt_device_address_t compare_param;
    wiced_result_t              result;

    compare_param.address   = device_address;
    compare_param.interface = interface;

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_find_node( &active_connection_list, compare_bt_device_address, (void*)&compare_param, (linked_list_node_t**) &connection );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    return ( result == WICED_SUCCESS ) ? connection : NULL;
}

wiced_result_t big_get_gatt_request_buffer( big_gatt_request_buffer_t** buffer, big_gatt_request_t** request_to_fill )
{
    wiced_assert( "bad arg", ( buffer != NULL ) && ( request_to_fill != NULL ) );

    wiced_rtos_lock_mutex( &big_gatt_mutex );

    if ( linked_list_remove_node_from_front( &free_request_buffer_list, ( linked_list_node_t**)buffer ) == WICED_SUCCESS )
    {
        big_gatt_request_buffer_t* internal_buffer = (big_gatt_request_buffer_t*)(*buffer);
        internal_buffer->free_buffer = WICED_FALSE;
        *request_to_fill             = &internal_buffer->request;
//        memset( *request_to_fill, 0, ( sizeof( big_gatt_request_t ) + wiced_bt_cfg_settings.gatt_cfg.max_attr_len ) );
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
        return WICED_BT_SUCCESS;
    }
    else
    {
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
        return WICED_BT_PACKET_POOL_EXHAUSTED;
    }
}

wiced_result_t big_release_gatt_request_buffer( big_gatt_request_buffer_t* buffer )
{
    big_gatt_request_buffer_t* internal_buffer = (big_gatt_request_buffer_t*)buffer;
    wiced_result_t result;

    wiced_assert( "bad arg", ( buffer != NULL ) && ( internal_buffer->free_buffer == WICED_FALSE ) );

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_insert_node_at_rear( &free_request_buffer_list, &internal_buffer->buffer_node );
    if ( result == WICED_SUCCESS )
    {
        internal_buffer->free_buffer = WICED_TRUE;
    }
    wiced_rtos_unlock_mutex( &big_gatt_mutex );
    return result;
}

wiced_result_t big_send_gatt_request( big_gatt_connection_t* connection, big_gatt_request_buffer_t* buffer )
{
    wiced_result_t result;
    uint32_t       count;

    wiced_assert( "bad arg", ( connection != NULL ) && ( buffer != NULL ) && ( connection->object_id == BIG_OBJECT_UNIQUE_ID ) );

    wiced_rtos_lock_mutex( &connection->mutex );

    linked_list_get_count( &connection->queued_request_list, &count );
    if ( ( count == 0 ) && ( current_gatt_request == NULL ) )
    {
        /* No command queued. Execute command now */
        wiced_rtos_lock_mutex( &big_gatt_mutex );
        result = send_gatt_request( connection, (big_gatt_request_buffer_t*)buffer );
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
    }
    else
    {
        /* Queue command */
        result = linked_list_insert_node_at_rear( &connection->queued_request_list, (linked_list_node_t*)buffer );
    }

    wiced_rtos_unlock_mutex( &connection->mutex );
    return result;
}

static wiced_result_t send_gatt_request( big_gatt_connection_t* connection, big_gatt_request_buffer_t* request_buffer )
{
    wiced_bt_gatt_status_t result;

    wiced_assert( "bad arg", ( connection != NULL ) && ( request_buffer != NULL ) );

    current_gatt_request = request_buffer;

    switch ( request_buffer->request.feature )
    {
        case BIG_GATT_EXCHANGE_MTU:
            result = wiced_bt_gatt_configure_mtu( connection->connection_id, request_buffer->request.parameter.mtu );
            break;

        case BIG_GATT_DISCOVER_ALL_PRIMARY_SERVICES:
        case BIG_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID:
        case BIG_GATT_DISCOVER_INCLUDED_SERVICES:
        case BIG_GATT_DISCOVER_ALL_CHARACTERISTICS_OF_A_SERVICE:
        case BIG_GATT_DISCOVER_CHARACTERISTICS_BY_UUID:
        case BIG_GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS:
            result = wiced_bt_gatt_send_discover( connection->connection_id, gatt_feature_to_type[request_buffer->request.feature], &request_buffer->request.parameter.discover );
            break;

        case BIG_GATT_READ_CHARACTERISTIC_VALUE:
        case BIG_GATT_READ_CHARACTERISTICS_BY_UUID:
        case BIG_GATT_READ_LONG_CHARACTERISTIC_VALUE:
        case BIG_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES:
        case BIG_GATT_READ_CHARACTERISTIC_DESCRIPTOR:
        case BIG_GATT_READ_LONG_CHARACTERISTIC_DESCRIPTOR:
            result = wiced_bt_gatt_send_read( connection->connection_id, gatt_feature_to_type[request_buffer->request.feature], &request_buffer->request.parameter.read );
            break;

        case BIG_GATT_WRITE_CHARACTERISTIC_VALUE:
        case BIG_GATT_WRITE_LONG_CHARACTERISTIC_VALUE:
        case BIG_GATT_WRITE_WITHOUT_RESPONSE:
        case BIG_GATT_WRITE_SIGNED_WITHOUT_RESPONSE:
        case BIG_GATT_WRITE_CHARACTERISTIC_DESCRIPTOR:
        case BIG_GATT_WRITE_LONG_CHARACTERISTIC_DESCRIPTOR:
            result = wiced_bt_gatt_send_write( connection->connection_id, gatt_feature_to_type[request_buffer->request.feature], &request_buffer->request.parameter.write );
            break;

        case BIG_GATT_PREPARE_RELIABLE_WRITE:
            result = wiced_bt_gatt_send_write( connection->connection_id, gatt_feature_to_type[request_buffer->request.feature], &request_buffer->request.parameter.prepare_reliable_write );
            break;

        case BIG_GATT_EXECUTE_RELIABLE_WRITE:
            result = wiced_bt_gatt_send_execute_write( connection->connection_id, ( request_buffer->request.parameter.execute_reliable_write == BIG_GATT_SEND_EXECUTE_WRITE ) ? WICED_TRUE : WICED_FALSE );
            break;

        case BIG_GATT_CONFIRM_INDICATION:
            result = wiced_bt_gatt_send_indication_confirm( connection->connection_id, request_buffer->request.parameter.indication_handle );
            break;

        default:
            wiced_assert("bad request", 0!= 0 );
            return WICED_BADARG;
    }

    if ( result != WICED_BT_GATT_SUCCESS )
    {
        current_gatt_request = NULL;
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

static wiced_result_t pop_next_gatt_request( big_gatt_connection_t* connection, big_gatt_request_buffer_t** request_buffer )
{
    return linked_list_remove_node_from_front( &connection->queued_request_list, (linked_list_node_t**)request_buffer );
}

static wiced_bt_dev_status_t big_stack_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* data )
{
    big_gatt_connection_t* connection = NULL;
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t local_address;

    UNUSED_VARIABLE( local_address );

    switch ( event )
    {
        case BTM_ENABLED_EVT:
            /* Bluetooth controller and host stack enabled. Event data: wiced_bt_dev_enabled_t */
            wiced_assert( "BT stack init failed", ( data->enabled.status == WICED_SUCCESS ) );
            init_result = data->enabled.status;
            if ( data->enabled.status == WICED_BT_SUCCESS )
            {
                wiced_bt_dev_read_local_addr( (uint8_t*)&local_address );
                WPRINT_LIB_INFO( ( "Bluetooth Internet Gateway initialised. BD address : %02X:%02X:%02X:%02X:%02X:%02X\r\n", local_address[ 0 ], local_address[ 1 ], local_address[ 2 ], local_address[ 3 ], local_address[ 4 ], local_address[ 5 ] ) );
                wiced_rtos_set_semaphore( &init_semaphore );
            }
            else
            {
                WPRINT_LIB_INFO( ( "Bluetooth Internet Gateway stack error [%u]\n", (unsigned int)data->enabled.status ) );
            }
            break;

        case BTM_DISABLED_EVT:
            /**< Bluetooth controller and host stack disabled. Event data: NULL */
            wiced_rtos_set_semaphore( &init_semaphore );
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            /**< Power management status change. Event data: wiced_bt_power_mgmt_notification_t */
            connection = internal_find_connection_by_address( &data->power_mgmt_notification.bd_addr );
            break;

        case BTM_PIN_REQUEST_EVT:
            /**< PIN request (used only with legacy devices). Event data: #wiced_bt_dev_name_and_class_t */
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            /**< received USER_CONFIRMATION_REQUEST event (respond using #wiced_bt_dev_confirm_req_reply). Event data: #wiced_bt_dev_user_cfm_req_t */
            connection = internal_find_connection_by_address( &data->user_confirmation_request.bd_addr );
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            /**< received USER_PASSKEY_NOTIFY event. Event data: #wiced_bt_dev_user_key_notif_t */
            connection = internal_find_connection_by_address( &data->user_passkey_notification.bd_addr );
            break;

        case BTM_PASSKEY_REQUEST_EVT:
            /**< received USER_PASSKEY_REQUEST event (respond using #wiced_bt_dev_pass_key_req_reply). Event data: #wiced_bt_dev_user_key_req_t */
            connection = internal_find_connection_by_address( &data->user_passkey_request.bd_addr );
            break;

        case BTM_KEYPRESS_NOTIFICATION_EVT:
            /**< received KEYPRESS_NOTIFY event. Event data: #wiced_bt_dev_user_keypress_t */
            connection = internal_find_connection_by_address( &data->user_keypress_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /**< Requesting IO capabilities for BLE pairing. Event data: #wiced_bt_dev_ble_io_caps_req_t */
            connection = internal_find_connection_by_address( &data->pairing_io_capabilities_ble_request.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /**< Requesting IO capabilities for BR/EDR pairing. Event data: #wiced_bt_dev_bredr_io_caps_req_t */
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            /**< received SIMPLE_PAIRING_COMPLETE event. Event data: #wiced_bt_dev_pairing_cplt_t */
            connection = internal_find_connection_by_address( (wiced_bt_device_address_t*)data->pairing_complete.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            /**< Encryption status change. Event data: #wiced_bt_dev_encryption_status_t */
            connection = internal_find_connection_by_address( (wiced_bt_device_address_t*)data->encryption_status.bd_addr );
            break;

        case BTM_SECURITY_REQUEST_EVT:
            /**< Security request (respond using #wiced_bt_ble_security_grant). Event data: #wiced_bt_dev_security_request_t */
            connection = internal_find_connection_by_address( &data->security_request.bd_addr );
            break;

        case BTM_SECURITY_FAILED_EVT:
            /**< Check if the application wants to upgrade the link key. Event data: #wiced_bt_dev_security_upgrade_t */
            connection = internal_find_connection_by_address( &data->security_failed.bd_addr );
            break;

        case BTM_SECURITY_ABORTED_EVT:
            /**< Security procedure aborted locally: or unexpected link drop. Event data: #wiced_bt_dev_name_and_class_t */
            connection = internal_find_connection_by_address( data->security_aborted.bd_addr );
            break;

        case BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT:
            /**< Result of reading local OOB data (wiced_bt_dev_read_local_oob_data). Event data: #wiced_bt_dev_local_oob_t */
            /* TODO: */
            break;

        case BTM_REMOTE_OOB_DATA_REQUEST_EVT:
            /**< OOB data from remote device (respond using #wiced_bt_dev_remote_oob_data_reply). Event data: #wiced_bt_dev_remote_oob_t */
            connection = internal_find_connection_by_address( &data->remote_oob_data_request.bd_addr );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* Assume that result is error first */
            result = WICED_BT_ERROR;
            connection = internal_find_connection_by_address( &data->paired_device_link_keys_request.bd_addr );
            if ( connection != NULL )
            {
                result = ((big_peer_device_link_keys_callback_t)connection->owner_interface->keys_callback)( BIG_REQUEST_SECURITY_KEYS_EVENT, &data->paired_device_link_keys_request );
                connection = NULL;
            }
            else if ( connecting_interface != NULL )
            {
                /* When stack request for link keys, connection isn't up yet. Pass it to the connecting interface */
                result = ((big_peer_device_link_keys_callback_t)connecting_interface->keys_callback)( BIG_REQUEST_SECURITY_KEYS_EVENT, &data->paired_device_link_keys_request );
            }
            else if ( gatt_server_interface.keys_callback != 0 )
            {
                result = ((big_peer_device_link_keys_callback_t)gatt_server_interface.keys_callback)( BIG_REQUEST_SECURITY_KEYS_EVENT, &data->paired_device_link_keys_request );
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Called when pairing is complete */
            connection = internal_find_connection_by_address( &data->paired_device_link_keys_update.bd_addr );
            if ( connection != NULL )
            {
                result = ((big_peer_device_link_keys_callback_t)connection->owner_interface->keys_callback)( BIG_UPDATE_SECURITY_KEYS_EVENT, &data->paired_device_link_keys_update );
            }
            connection = NULL;
            result = WICED_BT_SUCCESS;
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /**< Update local identity key (stored local_identity_keys NV memory). Event data: #wiced_bt_local_identity_keys_t */
            if ( local_keys_callback != NULL )
            {
                result = local_keys_callback( BIG_UPDATE_SECURITY_KEYS_EVENT, &data->local_identity_keys_update );
            }
            else
            {
                /* Return error to inform the stack that local keys aren't available */
                result = WICED_BT_SUCCESS;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /**< Request local identity key (get local_identity_keys from NV memory). If successful, return WICED_BT_SUCCESS. Event data: #wiced_bt_local_identity_keys_t */
            if ( local_keys_callback != NULL )
            {
                result = local_keys_callback( BIG_REQUEST_SECURITY_KEYS_EVENT, &data->local_identity_keys_request );
            }
            else
            {
                /* Return error to inform the stack that local keys aren't available */
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            /**< BLE scan state change. Event data: #wiced_bt_ble_scan_type_t */
            /* TODO: */
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            /**< BLE advertisement state change. Event data: #wiced_bt_ble_advert_mode_t */
            /* TODO: */
            break;

        case BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT:
            /**< SMP remote oob data request. Reply using wiced_bt_smp_oob_data_reply. Event data: wiced_bt_smp_remote_oob_req_t  */
            connection = internal_find_connection_by_address( &data->smp_remote_oob_data_request.bd_addr );
            break;

        case BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT:
            /**< LE secure connection remote oob data request. Reply using wiced_bt_smp_sc_oob_reply. Event data: #wiced_bt_smp_sc_remote_oob_req_t */
            connection = internal_find_connection_by_address( &data->smp_sc_remote_oob_data_request.bd_addr );
            break;

        case BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT:
            /**< LE secure connection local OOB data (wiced_bt_smp_create_local_sc_oob_data). Event data: #wiced_bt_smp_sc_local_oob_t*/
            connection = internal_find_connection_by_address( &data->p_smp_sc_local_oob_data->addr_sent_to.bda );
            break;

        case BTM_SCO_CONNECTED_EVT:
            /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
            /* TODO: */
            break;

        case BTM_SCO_DISCONNECTED_EVT:
            /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:
            /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:
            /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
            break;

        default:
            break;
    }

    if ( connection != NULL )
    {
        result = ((big_gatt_interface_callback_t)connection->owner_interface->interface_callback)( connection->owner_interface, event, data );
    }
    return result;
}

static wiced_bt_gatt_status_t big_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *data )
{
    big_gatt_connection_t* connection = NULL;

    /* Find connection */
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            connection = internal_find_connection_by_id( data->connection_status.conn_id );
            break;

        case GATT_OPERATION_CPLT_EVT:
            connection = internal_find_connection_by_id( data->operation_complete.conn_id );
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            connection = internal_find_connection_by_id( data->discovery_result.conn_id );
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            connection = internal_find_connection_by_id( data->discovery_complete.conn_id );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            connection = internal_find_connection_by_id( data->attribute_request.conn_id );
            break;

        default:
            break;
    }

    /* Execute command */
    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
        {
            /**< GATT connection status change. Event data: #wiced_bt_gatt_connection_status_t */
            big_gatt_interface_t* interface = NULL;

            if ( ( connection != NULL ) && ( data->connection_status.connected == WICED_FALSE ) )
            {
                /* Remove existing connection */
                interface = connection->owner_interface;
                remove_exisiting_connection( connection );
            }
            else if ( connection == NULL )
            {
                /* Add new connection */
                wiced_bt_ble_address_t new_connection_address;

                memcpy( &new_connection_address.bda, data->connection_status.bd_addr, sizeof( new_connection_address.bda ) );
                new_connection_address.type = data->connection_status.addr_type;

                if ( ( connecting_interface != NULL ) && big_match_ble_addresses( &connecting_peer_address, &new_connection_address ) == WICED_TRUE )
                {
                    wiced_rtos_deregister_timed_event( &connect_timeout_event );

                    /* Connection initiated connection GATT client interface */
                    interface = connecting_interface;
                    connecting_interface = NULL;
                }
                else if ( gatt_server_interface.object_id == BIG_OBJECT_UNIQUE_ID )
                {
                    /* Connection initiated from peer device to GATT server */
                    interface = &gatt_server_interface;
                }

                if ( data->connection_status.connected == WICED_TRUE )
                {
                    connection = add_new_connection( interface, &new_connection_address, data->connection_status.conn_id );
                }
            }
            else
            {
                wiced_assert( "Connection not initiated by BIG", ( connection != NULL ) );
            }

            ( (big_gatt_connection_callback_t) ( interface->connection_callback ) )( connection, NULL, event, data );
            break;
        }

        case GATT_OPERATION_CPLT_EVT:
        case GATT_DISCOVERY_CPLT_EVT:
        case GATT_ATTRIBUTE_REQUEST_EVT:
        {
            /* GATT read or write is complete. Call the connection callback and then Execute the next queued command */
            if ( connection != NULL )
            {
                big_gatt_request_buffer_t* buffer;

                /* Call connection callback */
                ( (big_gatt_connection_callback_t) ( connection->owner_interface->connection_callback ) )( connection, &current_gatt_request->request, event, data );

                /* Release request buffer */
                if ( current_gatt_request != NULL )
                {
                    big_release_gatt_request_buffer( current_gatt_request );
                }

                /* Execute next commmand, if any */
                wiced_rtos_lock_mutex( &big_gatt_mutex );
                if ( pop_next_gatt_request( connection, &buffer ) == WICED_SUCCESS )
                {
                    send_gatt_request( connection, buffer );
                }
                else
                {
                    current_gatt_request = NULL;
                }
                wiced_rtos_unlock_mutex( &big_gatt_mutex );
            }

            break;
        }

        case GATT_DISCOVERY_RESULT_EVT:
        {
            big_gatt_connection_t* connection = internal_find_connection_by_id( data->discovery_result.conn_id );

            if ( connection != NULL )
            {
                /* Call connection callback */
                ( (big_gatt_connection_callback_t) ( connection->owner_interface->connection_callback ) )( connection, &current_gatt_request->request, event, data );
            }
            break;
        }

        default:
            break;
    }

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bool_t big_match_bt_device_addresses( wiced_bt_device_address_t* address1, wiced_bt_device_address_t* address2 )
{
    if ( memcmp( address1, address2, sizeof( *address1 ) ) == 0 )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static wiced_bool_t big_match_ble_addresses( wiced_bt_ble_address_t* address1, wiced_bt_ble_address_t* address2 )
{
    // TODO: Check why during disconnection timeout event, event data contains address type 0 instead of 1
    if ( ( memcmp( &address1->bda, &address2->bda, sizeof( address1->bda ) ) == 0 ) /* && ( address1->type == address2->type ) */ )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static wiced_bool_t internal_compare_connection_id( linked_list_node_t* node_to_compare, void* user_data )
{
    big_gatt_connection_t* connection = (big_gatt_connection_t*) node_to_compare;
    uint16_t connection_id = (uint16_t) ( (uint32_t) user_data & 0xffff );

    if ( connection->connection_id == connection_id )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static big_gatt_connection_t* internal_find_connection_by_id( uint16_t connection_id )
{
    big_gatt_connection_t* connection = NULL;
    wiced_result_t result;

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_find_node( &active_connection_list, internal_compare_connection_id, (void*) ( (uint32_t) connection_id ), (linked_list_node_t**) &connection );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    return ( result == WICED_SUCCESS ) ? connection : NULL;
}

static wiced_bool_t internal_compare_bt_device_address( linked_list_node_t* node_to_compare, void* user_data )
{
    big_gatt_connection_t*     connection = (big_gatt_connection_t*) node_to_compare;
    wiced_bt_device_address_t* address    = (wiced_bt_device_address_t*) user_data;

    return big_match_bt_device_addresses( &connection->address.bda, address );
}

static big_gatt_connection_t* internal_find_connection_by_address( wiced_bt_device_address_t* address )
{
    big_gatt_connection_t* connection = NULL;
    wiced_result_t         result;

    wiced_rtos_lock_mutex( &big_gatt_mutex );
    result = linked_list_find_node( &active_connection_list, internal_compare_bt_device_address, (void*)address, (linked_list_node_t**)&connection );
    wiced_rtos_unlock_mutex( &big_gatt_mutex );

    return ( result == WICED_SUCCESS ) ? connection : NULL;
}

static big_gatt_connection_t* add_new_connection( big_gatt_interface_t* interface, wiced_bt_ble_address_t* address, uint16_t connection_id )
{
    big_gatt_connection_t* new_connection;
    wiced_result_t         result;

    wiced_rtos_lock_mutex( &big_gatt_mutex );

    /* Move connection from inactive to active list */
    result = linked_list_remove_node_from_front( &inactive_connection_list, (linked_list_node_t**)&new_connection );
    if ( result != WICED_SUCCESS )
    {
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
        return NULL;
    }

    new_connection->connection_id   = connection_id;
    new_connection->object_id       = BIG_OBJECT_UNIQUE_ID;
    new_connection->owner_interface = interface;
    new_connection->is_connected    = WICED_TRUE;
    new_connection->address         = *address;
    linked_list_init( &new_connection->queued_request_list );

    result = linked_list_insert_node_at_rear( &active_connection_list, &new_connection->connection_node );
    if ( result != WICED_SUCCESS )
    {
        linked_list_insert_node_at_rear( &inactive_connection_list, &new_connection->connection_node );
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
        return NULL;
    }

    wiced_rtos_unlock_mutex( &big_gatt_mutex );
    return new_connection;
}

static wiced_result_t remove_exisiting_connection( big_gatt_connection_t* connection )
{
    wiced_result_t result;

    wiced_rtos_lock_mutex( &big_gatt_mutex );

    /* Move connection from active to inactive list */
    result = linked_list_remove_node( &active_connection_list, &connection->connection_node );
    if ( result != WICED_SUCCESS )
    {
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
        return result;
    }

    connection->connection_id   = 0;
    connection->object_id       = 0;
    connection->owner_interface = NULL;
    connection->is_connected    = WICED_FALSE;
    memset( &connection->address, 0, sizeof( connection->address ) );
    linked_list_deinit( &connection->queued_request_list );

    result = linked_list_insert_node_at_rear( &inactive_connection_list, &connection->connection_node );
    if ( result != WICED_SUCCESS )
    {
        wiced_rtos_unlock_mutex( &big_gatt_mutex );
        return result;
    }

    wiced_rtos_unlock_mutex( &big_gatt_mutex );
    return WICED_SUCCESS;
}

static wiced_result_t connect_timeout_callback( void* arg )
{
    wiced_rtos_deregister_timed_event( &connect_timeout_event );
    big_ble_gatt_client_cancel_connect( );
    return WICED_SUCCESS;
}

void wiced_mesh_get_semaphore()
{
    wiced_rtos_get_semaphore( &init_semaphore, WICED_WAIT_FOREVER );
}

void wiced_mesh_set_semaphore()
{
    wiced_rtos_set_semaphore( &init_semaphore );
}
