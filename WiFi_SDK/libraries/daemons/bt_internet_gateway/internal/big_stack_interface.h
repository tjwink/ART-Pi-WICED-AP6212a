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
#pragma once

#include "linked_list.h"
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* Default GATT request buffer count. Can be overridden through GLOBAL_DEFINES in makefile */
#ifndef BIG_GATT_REQUEST_BUFFER_COUNT
#define BIG_GATT_REQUEST_BUFFER_COUNT  ( 10 )
#endif

#ifndef BIG_GATT_CONNECT_TIMEOUT_MS
#define BIG_GATT_CONNECT_TIMEOUT_MS  ( 3000 )
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    BIG_GATT_EXCHANGE_MTU,
    BIG_GATT_DISCOVER_ALL_PRIMARY_SERVICES,
    BIG_GATT_DISCOVER_PRIMARY_SERVICES_BY_UUID,
    BIG_GATT_DISCOVER_INCLUDED_SERVICES,
    BIG_GATT_DISCOVER_ALL_CHARACTERISTICS_OF_A_SERVICE,
    BIG_GATT_DISCOVER_CHARACTERISTICS_BY_UUID,
    BIG_GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,
    BIG_GATT_READ_CHARACTERISTIC_VALUE,
    BIG_GATT_READ_CHARACTERISTICS_BY_UUID,
    BIG_GATT_READ_LONG_CHARACTERISTIC_VALUE,
    BIG_GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES,
    BIG_GATT_READ_CHARACTERISTIC_DESCRIPTOR,
    BIG_GATT_READ_LONG_CHARACTERISTIC_DESCRIPTOR,
    BIG_GATT_WRITE_CHARACTERISTIC_VALUE,
    BIG_GATT_WRITE_LONG_CHARACTERISTIC_VALUE,
    BIG_GATT_WRITE_WITHOUT_RESPONSE,
    BIG_GATT_WRITE_SIGNED_WITHOUT_RESPONSE,
    BIG_GATT_WRITE_CHARACTERISTIC_DESCRIPTOR,
    BIG_GATT_WRITE_LONG_CHARACTERISTIC_DESCRIPTOR,
    BIG_GATT_PREPARE_RELIABLE_WRITE,
    BIG_GATT_EXECUTE_RELIABLE_WRITE,
    BIG_GATT_CONFIRM_INDICATION,
} big_gatt_feature_t;

typedef enum
{
    BIG_GATT_SEND_EXECUTE_WRITE,
    BIG_GATT_CANCEL_EXECUTE_WRITE,
} big_gatt_execute_write_t;

typedef enum
{
    BIG_REQUEST_SECURITY_KEYS_EVENT,
    BIG_UPDATE_SECURITY_KEYS_EVENT,
} big_security_event_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/**
 * Declaration of BIG GATT request buffer
 */
typedef struct big_gatt_request_buffer big_gatt_request_buffer_t;

/* Type definition of callback function that will be called when the app needs to load or store link keys with a paired device */
typedef wiced_result_t (*big_peer_device_link_keys_callback_t)( big_security_event_t event, wiced_bt_device_link_keys_t* keys );

/* Type definition of callback function that will be called when the app needs to load or store updated local keys */
typedef wiced_result_t (*big_local_keys_callback_t)( big_security_event_t event, wiced_bt_local_identity_keys_t* keys );


/******************************************************
 *                    Structures
 ******************************************************/

/**
 * GATT request parameters
 */
typedef union
{
    uint16_t                        mtu;                     /* Parameters for BIG_GATT_EXCHANGE request */
    wiced_bt_gatt_discovery_param_t discover;                /* Parameters for BIG_GATT_DISCOVER_XXX requests */
    wiced_bt_gatt_read_param_t      read;                    /* Parameters for BIG_GATT_READ_XXX requests */
    wiced_bt_gatt_value_t           write;                   /* Parameters for BIG_GATT_WRITE_XXX requests */
    wiced_bt_gatt_value_t           prepare_reliable_write;  /* Parameters for BIG_GATT_PREPARE_RELIABLE_WRITE request */
    big_gatt_execute_write_t        execute_reliable_write;  /* Parameters for BIG_GATT_EXECUTE_RELIABLE_WRITE requests */
    uint16_t                        indication_handle;       /* Parameters for BIG_GATT_CONFIRM_INDICATION request */
} big_gatt_request_param_t;

/**
 * GATT request to fill
 * Warning: Do not change the order of the fields. Parameter is always the last one because for GATT write, the value buffer immediately follows the parameters.
 */
typedef struct
{
    big_gatt_feature_t       feature;   /* GATT feature */
    void*                    arg;       /* User defined argument to pass to the GATT callback */
    big_gatt_request_param_t parameter; /* Union of parameters. Fill out parameter according to the feature selected */
} big_gatt_request_t;

/**
 * Workspace structure for BIG application interface
 * Users should not access these values - they are provided here only
 * to provide the compiler with datatype size information allowing static declarations
 */
typedef struct
{
    linked_list_node_t interface_node;
    uint32_t           object_id;
    uint32_t           interface_callback;
    uint32_t           connection_callback;
    uint32_t           keys_callback;
} big_gatt_interface_t;

/**
 * Workspace structure for BIG GATT connection.
 * Users may access but must not modify these values.
 * Connections are managed internally by the BIG GATT interface module.
 */
typedef struct
{
    linked_list_node_t     connection_node;
    uint32_t               object_id;
    uint16_t               connection_id;
    wiced_bool_t           is_connected;
    wiced_bt_ble_address_t address;
    big_gatt_interface_t*  owner_interface;
    wiced_mutex_t          mutex;
    linked_list_t          queued_request_list;
} big_gatt_connection_t;

/**
 * BIG GATT interface callback
 */
typedef wiced_result_t (*big_gatt_interface_callback_t)( big_gatt_interface_t* interface, wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* data );

/**
 * BIG GATT connection callback
 */
typedef wiced_bt_gatt_status_t (*big_gatt_connection_callback_t)( big_gatt_connection_t* connection, const big_gatt_request_t* current_request, wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t* data );

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Init BIG internal infrastructure
 *
 * @param[in] callback : function that will be called when the apps need to load or store local keys
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_init_stack_interface( big_local_keys_callback_t callback );

/**
 * Deinit BIG internal infrastructure
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_deinit_stack_interface( void );

/**
 * Init GATT server interface. GATT server is singleton and is defined internally.
 *
 * @param[in] interface_callback  : function that will be called when @ref wiced_bt_management_evt_t for the GATT server interface occurs
 * @param[in] connection_callback : function that will be called when @ref wiced_bt_gatt_evt_t for the connections that belongs to the GATT server interface occurs
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_init_gatt_server_interface( big_gatt_interface_callback_t interface_callback, big_gatt_connection_callback_t connection_callback, big_peer_device_link_keys_callback_t keys_callback );

/**
 * Deinit GATT client interface
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_deinit_gatt_server_interface( void );

/**
 * Add a GATT client interface. Application may add as many clients as applicable.
 *
 * @param[in] interface           : GATT client application interface to add
 * @param[in] interface_callback  : function that will be called when @ref wiced_bt_management_evt_t for the given GATT client occurs
 * @param[in] connection_callback : function that will be called when @ref wiced_bt_gatt_evt_t for the connections that belongs to the GATT client interface occurs
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_add_gatt_client_interface( big_gatt_interface_t* interface, big_gatt_interface_callback_t interface_callback, big_gatt_connection_callback_t connection_callback, big_peer_device_link_keys_callback_t keys_callback );

/**
 * Remove a GATT client interface
 *
 * @param[in] interface : GATT client interface to remove
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_remove_gatt_client_interface( big_gatt_interface_t* interface );

/**
 * Create a BLE GATT connection
 *
 * @param[in] interface    : Application interface which will own the connection
 * @param[in] connection   : Pointer to connection structure
 * @param[in] address      : Address of the peer bluetooth device to connect
 * @param[in] address_type : Address type
 * @param[in] callback     : Connection callback function
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_ble_gatt_client_connect( big_gatt_interface_t* interface, wiced_bt_ble_address_t* address );

/**
 * Cancel current BLE GATT connection attempt
 *
 * @return @ref wiced_result_t
 */

wiced_result_t big_ble_gatt_client_cancel_connect( void );

/**
 * Disconnect a BLE GATT connection
 *
 * @param[in] connection : Connection to be disconnected
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_ble_gatt_disconnect( big_gatt_connection_t* connection );

/**
 * Find a connection by its connection ID
 *
 * @param[in] connection_id : ID of the connection to be found
 *
 * @return @ref big_gatt_connection_t
 */
big_gatt_connection_t* big_find_gatt_connection_by_id( big_gatt_interface_t* interface, uint16_t connection_id );

/**
 * Find a connection by its BLE address
 *
 * @param[in] address : Address of the connection to be found
 *
 * @return @ref big_gatt_connection_t
 */
big_gatt_connection_t* big_find_gatt_connection_by_ble_address( big_gatt_interface_t* interface, wiced_bt_ble_address_t* ble_address );

/**
 * Find a connection by its peer device address
 *
 * @param[in] address : Address of the connection to be found
 *
 * @return @ref big_gatt_connection_t
 */
big_gatt_connection_t* big_find_gatt_connection_by_device_address( big_gatt_interface_t* interface, wiced_bt_device_address_t* device_address );

/**
 * Allocate a GATT request buffer
 *
 * @param[in] buffer          : Pointer that will point to the buffer
 * @param[in] request_to_fill : Pointer that will point to the section in the buffer that will be filled with the request parameters
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_get_gatt_request_buffer( big_gatt_request_buffer_t** buffer, big_gatt_request_t** request_to_fill );

/**
 * Release a GATT request buffer back to the pool
 *
 * @return @ref big_gatt_request_t
 */
wiced_result_t big_release_gatt_request_buffer( big_gatt_request_buffer_t* buffer );

/**
 * Send a GATT procedure request to the BT stack.
 * Upon successful return, BIG GATT interface is responsible for releasing the buffer back to the pool.
 *
 * @param[in] connection : Connection where this request will be sent
 * @param[in] buffer     : Request buffer
 *
 * @return @ref wiced_result_t
 */
wiced_result_t big_send_gatt_request( big_gatt_connection_t* connection, big_gatt_request_buffer_t* buffer );

#ifdef __cplusplus
} /* extern "C" */
#endif
