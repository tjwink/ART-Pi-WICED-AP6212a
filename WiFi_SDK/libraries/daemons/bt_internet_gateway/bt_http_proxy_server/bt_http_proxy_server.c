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
#include "bt_http_proxy_server.h"
#include "wiced_result.h"
#include "wwd_constants.h"
#include <string.h>
#include <stdio.h>
#include "wiced.h"
#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "sdpdefs.h"  /* For GATT UUID definitions */
#include "gattdefs.h" /* For GATT UUID definitions */
#include "big_stack_interface.h"

/** @file
 *
 */

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HTTP_GET_STRING                              "GET "
#define HTTP_HEAD_STRING                             "HEAD "
#define HTTP_POST_STRING                             "POST "
#define HTTP_PUT_STRING                              "PUT "
#define HTTP_DELETE_STR                              "DELETE "
#define NEW_LINE_STR                                 "\r\n"

#define HTTP_PORT                                    (80)
#define HTTPS_PORT                                   (443)
#define UUID_SERVCLASS_HPS_LOW_BYTE                  (0x10)
#define UUID_SERVCLASS_HPS_HIGH_BYTE                 (0x7F)
#define HTTP_PROXY_SERVER_GATT_DB_SIZE               (sizeof( http_proxy_server_gatt_db ))
#define HTTP_SERVER_REQUEST_TIMEOUT_STATUS_CODE      (504)
#define LEGATTDB_ATTR_WRITE_QUEUE                    (30)
#define HOSTNAME_LOOKUP_TIMEOUT_MS                   (2000)
#define HPS_SERVER_TCP_CONNECTION_TIMEOUT            (3000)
#define MAX_HTTP_RESPONSE_RECEIVE_TIMEOUT            (5000)
#define INIT_STACK_TIMEOUT_MS                        (10000)

/* Data Status */
#define HPS_DATA_STATUS_HEADERS_RECEIVED             (0x01)
#define HPS_DATA_STATUS_HEADERS_TRUNCATED            (0x02)
#define HPS_DATA_STATUS_BODY_RECEIVED                (0x04)
#define HPS_DATA_STATUS_BODY_TRUNCATED               (0x08)

#define HTTP_SERVER_HIGH_DUTY_ADVERTISEMENT_WINDOW   (48)
#define HTTP_SERVER_HIGH_DUTY_ADVERTISEMENT_INTERVAL (96)

/******************************************************
 *                   Enumerations
 ******************************************************/

/* Enumeration of GATT database handles */
typedef enum
{
    /* GATT service handles */
    GATT_DB_HANDLE_GATT_SERVICE_CLASS                                    = 0x0001,

    /* GAP service handles */
    GATT_DB_HANDLE_GAP_SERVICE_CLASS                                     = 0x0010,
    GATT_DB_HANDLE_GAP_DEVICE_NAME_ATTRIBUTE                             = 0x0011,
    GATT_DB_HANDLE_GAP_DEVICE_NAME_VALUE                                 = 0x0012,
    GATT_DB_HANDLE_GAP_APPEARANCE_ATTRIBUTE                              = 0x0013,
    GATT_DB_HANDLE_GAP_APPEARANCE_NAME_VALUE                             = 0x0014,

    /* HTTP Proxy Service Handles */
    GATT_DB_HANDLE_HPS_SERVICE_CLASS                                     = 0x0030,
    GATT_DB_HANDLE_HPS_URI_ATTRIBUTE                                     = 0x0031,
    GATT_DB_HANDLE_HPS_URI_VALUE                                         = 0x0032,
    GATT_DB_HANDLE_HPS_URI_EXTENDED_PROPERTIES_DESCRIPTOR                = 0x0033,
    GATT_DB_HANDLE_HPS_HTTP_HEADERS_ATTRIBUTE                            = 0x0034,
    GATT_DB_HANDLE_HPS_HTTP_HEADERS_VALUE                                = 0x0035,
    GATT_DB_HANDLE_HPS_HTTP_HEADERS_EXTENDED_PROPERTIES_DESCRIPTOR       = 0x0036,
    GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_ATTRIBUTE                        = 0x0037,
    GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_VALUE                            = 0x0038,
    GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_EXTENDED_PROPERTIES_DESCRIPTOR   = 0x0039,
    GATT_DB_HANDLE_HPS_HTTP_CONTROL_POINT_ATTRIBUTE                      = 0x003a,
    GATT_DB_HANDLE_HPS_HTTP_CONTROL_POINT_VALUE                          = 0x003b,
    GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_ATTRIBUTE                        = 0x003c,
    GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_VALUE                            = 0x003d,
    GATT_DB_HANDLE_CLIENT_CHAR_CONFIG                                    = 0x003e,
    GATT_DB_HANDLE_HPS_HTTPS_SECURITY_ATTRIBUTE                          = 0x003f,
    GATT_DB_HANDLE_HPS_HTTPS_SECURITY_VALUE                              = 0x0040,

    GATT_DB_HDL_MAX
} gatt_database_handle_t;

typedef enum
{
    HTTP_GET_REQUEST     = 1,
    HTTP_HEAD_REQUEST    = 2,
    HTTP_POST_REQUEST    = 3,
    HTTP_PUT_REQUEST     = 4,
    HTTP_DELETE_REQUEST  = 5,
    HTTPS_GET_REQUEST    = 6,
    HTTPS_HEAD_REQUEST   = 7,
    HTTPS_POST_REQUEST   = 8,
    HTTPS_PUT_REQUEST    = 9,
    HTTPS_DELETE_REQUEST = 10,
    HTTP_REQUEST_CANCEL  = 11
} hps_http_request_t;

typedef enum
{
    HPS_EVENT_HTTP_CONNECTION_ESTABLISHED,
    HPS_EVENT_HTTP_CONNECTION_DISCONNECTED,
    HPS_EVENT_HTTP_DISCONNECT_TIMEOUT,
    HPS_EVENT_HTTP_CONNECT_TIMEOUT,
    HPS_EVENT_HTTP_RESPONSE_RECEIVED,
} hps_http_event_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    hps_connection_t*                      connections;
    uint8_t                                max_connections;
    wiced_interface_t                      interface;
    hps_server_ble_connect_callback_t      connect_callback;
    hps_sever_https_certificate_callback_t https_callback;
} hps_server_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/* GATT database */
static const uint8_t http_proxy_server_gatt_db[] =
{
    /* GATT service
     * Service change characteristic is optional and is not present
     */
    PRIMARY_SERVICE_UUID16( GATT_DB_HANDLE_GATT_SERVICE_CLASS,
                            UUID_SERVCLASS_GATT_SERVER ),

    /* GAP service
     * Device Name and Appearance are mandatory characteristics.  Peripheral
     * Privacy Flag only required if privacy feature is supported.  Reconnection
     * Address is optional and only when privacy feature is supported.
     * Peripheral Preferred Connection Parameters characteristic is optional
     * and not present.
     */
    PRIMARY_SERVICE_UUID16( GATT_DB_HANDLE_GAP_SERVICE_CLASS,
                            UUID_SERVCLASS_GAP_SERVER ),

    /* Characteristic Device Name, handle 0x16 characteristic value.
     * Any 16 byte string can be used to identify the sensor.  Just need to
     * replace the "Hello" string below.  Keep it short so that it fits in
     * advertisement data along with 16 byte UUID.
     */
    CHARACTERISTIC_UUID16( GATT_DB_HANDLE_GAP_DEVICE_NAME_ATTRIBUTE,
                           GATT_DB_HANDLE_GAP_DEVICE_NAME_VALUE,
                           GATT_UUID_GAP_DEVICE_NAME,
                           LEGATTDB_CHAR_PROP_READ,
                           LEGATTDB_PERM_READABLE ),

    /* Characteristic Appearance, handle 0x18 characteristic value.
     * List of approved appearances is available at bluetooth.org.  Current
     * value is set to 0x200 - Generic Tag
     */
    CHARACTERISTIC_UUID16( GATT_DB_HANDLE_GAP_APPEARANCE_ATTRIBUTE,
                           GATT_DB_HANDLE_GAP_APPEARANCE_NAME_VALUE,
                           GATT_UUID_GAP_ICON,
                           LEGATTDB_CHAR_PROP_READ,
                           LEGATTDB_PERM_READABLE ),

    /* HTTP Proxy Service (HPS) */
    PRIMARY_SERVICE_UUID16( GATT_DB_HANDLE_HPS_SERVICE_CLASS,
                            UUID_SERVCLASS_HPS ),

    /* URI Characteristic - used to configure the URI for a subsequent request */
    CHARACTERISTIC_UUID16_WRITABLE( GATT_DB_HANDLE_HPS_URI_ATTRIBUTE,
                                    GATT_DB_HANDLE_HPS_URI_VALUE,
                                    GATT_UUID_HPS_URI,
                                    ( LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_EXTENDED),
                                    ( LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH ) ),

    /* Extended Properties Descriptor */
    CHAR_DESCRIPTOR_UUID16( GATT_DB_HANDLE_HPS_URI_EXTENDED_PROPERTIES_DESCRIPTOR,
                            GATT_UUID_CHAR_EXT_PROP,
                            LEGATTDB_PERM_READABLE ),

    /* HTTP Headers Characteristic - used to hold the headers that would be sent with the
     * HTTP Request or the headers contained within an HTTP response message from the HTTP
     * server.
     */
    CHARACTERISTIC_UUID16_WRITABLE( GATT_DB_HANDLE_HPS_HTTP_HEADERS_ATTRIBUTE,
                                    GATT_DB_HANDLE_HPS_HTTP_HEADERS_VALUE,
                                    GATT_UUID_HPS_HEADERS,
                                    ( LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_EXTENDED),
                                    ( LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH ) ),

    /* Extended Properties Descriptor */
    CHAR_DESCRIPTOR_UUID16( GATT_DB_HANDLE_HPS_HTTP_HEADERS_EXTENDED_PROPERTIES_DESCRIPTOR,
                            GATT_UUID_CHAR_EXT_PROP,
                            LEGATTDB_PERM_READABLE ),

    /* HTTP Entity Body Characteristic - contains the contents of the message-body after
     * any Transfer-Encoding has been applied.
     */
    CHARACTERISTIC_UUID16_WRITABLE( GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_ATTRIBUTE,
                                    GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_VALUE,
                                    GATT_UUID_HPS_ENTITY_BODY,
                                    ( LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_EXTENDED),
                                    ( LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE | LEGATTDB_PERM_VARIABLE_LENGTH ) ),

    /* Extended Properties Descriptor */
    CHAR_DESCRIPTOR_UUID16( GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_EXTENDED_PROPERTIES_DESCRIPTOR,
                            GATT_UUID_CHAR_EXT_PROP,
                            LEGATTDB_PERM_READABLE ),

    /* HTTP Control Point Characteristic - used to initiate a request to send an HTTP
     * request from the device containing the HTTP Proxy Service, acting as a HTTP
     * Client, and a HTTP Server.
     */
    CHARACTERISTIC_UUID16_WRITABLE( GATT_DB_HANDLE_HPS_HTTP_CONTROL_POINT_ATTRIBUTE,
                                    GATT_DB_HANDLE_HPS_HTTP_CONTROL_POINT_VALUE,
                                    GATT_UUID_HPS_CONTROL_POINT,
                                    LEGATTDB_CHAR_PROP_WRITE,
                                    ( LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ) ),


    /* HTTP Status Code Characteristic - contains the Status-Code from the Status-
     * Line of the first line of the HTTP Response message, followed by one octet
     * indicating the Data Status bit field indicating the status of the data
     * received as defined by the HTTP Data Status Bit Field in the HTTP Proxy
     * Service Bluetooth specification.
     */
    CHARACTERISTIC_UUID16( GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_ATTRIBUTE,
                           GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_VALUE,
                           GATT_UUID_HPS_STATUS_CODE,
                           LEGATTDB_CHAR_PROP_NOTIFY,
                           LEGATTDB_PERM_NONE ),

    /* Client Characteristic Configuration Descriptor for Status Code Notifications */
    CHAR_DESCRIPTOR_UUID16_WRITABLE( GATT_DB_HANDLE_CLIENT_CHAR_CONFIG,
                                     GATT_UUID_CHAR_CLIENT_CONFIG,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ ),

    /* HTTPS Security Characteristic - contains the known authenticity of the
     * HTTP Server certificate for the URI.
     */
    CHARACTERISTIC_UUID16( GATT_DB_HANDLE_HPS_HTTPS_SECURITY_ATTRIBUTE,
                           GATT_DB_HANDLE_HPS_HTTPS_SECURITY_VALUE,
                           GATT_UUID_HPS_SERCURITY,
                           LEGATTDB_CHAR_PROP_READ,
                           LEGATTDB_PERM_READABLE )
};

/* HPS service advertisement details */
static const uint8_t hps_service_uuid[ 2 ] =
{
    UUID_SERVCLASS_HPS_LOW_BYTE,
    UUID_SERVCLASS_HPS_HIGH_BYTE
};


static hps_server_t                  hps_server;
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[];


/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t         hps_server_gatt_server_interface_callback          ( big_gatt_interface_t* interface, wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* data );
static wiced_bt_gatt_status_t hps_server_gatt_connection_callback                ( big_gatt_connection_t* connection, const big_gatt_request_t* current_request, wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t* data );
static wiced_bt_gatt_status_t hps_server_ble_connect                             ( uint8_t *bd_address, uint16_t connection_handle, wiced_bool_t is_connected );
static hps_connection_t*      hps_server_find_hps_connection                     ( uint16_t connection_handle );
static wiced_bt_gatt_status_t hps_server_gatt_write                              ( wiced_bt_gatt_request_data_t *buf, uint16_t connection_handle, wiced_bt_gatt_request_type_t request_type );
static wiced_bt_gatt_status_t hps_server_gatt_read                               ( wiced_bt_gatt_request_data_t *buf, uint16_t connection_handle );
static void                   hps_server_update_state                            ( hps_connection_t* connection, wiced_tcp_socket_t* tcp_socket, hps_http_event_t event );
static wiced_bt_gatt_status_t hps_server_write_characteristic                    ( hps_connection_t *connection, wiced_bt_gatt_write_t *write_req );
static wiced_bt_gatt_status_t hps_server_write_client_char_config_characteristic ( hps_connection_t *connection, wiced_bt_gatt_write_t *write_req );
static wiced_bt_gatt_status_t hps_server_write_http_control_point_characteristic ( hps_connection_t *connection, wiced_bt_gatt_write_t *write_req );
static wiced_bt_gatt_status_t hps_server_write_http_headers_characteristic       ( hps_connection_t *connection, wiced_bt_gatt_write_t *write_req );
static wiced_bt_gatt_status_t hps_server_write_http_body_characteristic          ( hps_connection_t *connection, wiced_bt_gatt_write_t *write_req );
static wiced_bt_gatt_status_t hps_server_write_http_uri_characteristic           ( hps_connection_t *connection, wiced_bt_gatt_write_t *write_req );
static void                   hps_server_execute_write_characteristic            ( hps_connection_t *connection, uint16_t handle );
static void                   hps_server_dns_lookup_complete                     ( hps_connection_t* connection, wiced_ip_address_t* address );
static void                   hps_server_parse_http_response                     ( uint8_t* response, uint16_t response_length, hps_connection_t* connection );
static wiced_result_t         hps_server_receive_timer_expired                   ( void* arg );
static void                   hps_server_send_https_request                      ( hps_connection_t *connection );
static void                   hps_server_send_http_request                       ( hps_connection_t *connection );
static wiced_result_t         hps_server_perform_dns_lookup                      ( hps_connection_t *connection );
static void                   hps_server_fill_http_request                       ( hps_connection_t* connection, uint8_t* data_ptr, uint16_t* http_request_length );
static wiced_result_t         hps_server_tcp_connect                             ( hps_connection_t* connection );
static wiced_result_t         hps_server_hostname_lookup_callback                ( void* arg );
static wiced_result_t         hps_server_tcp_connect_callback                    ( void* arg );
static wiced_result_t         hps_server_tcp_disconnect_callback                 ( wiced_tcp_socket_t* socket, void* arg );
static wiced_result_t         hps_server_tcp_receive_callback                    ( wiced_tcp_socket_t* socket, void* arg );

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t hps_server_start( hps_connection_t* connections_array, uint8_t connection_count, wiced_interface_t interface, big_peer_device_link_keys_callback_t keys_callback )
{
    wiced_result_t result;
    wiced_bt_ble_advert_elem_t adv_data[2];
    uint8_t ble_advertisement_flag_value = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0;

    memset( &hps_server, 0, sizeof( hps_server ) );
    hps_server.connections     = connections_array;
    hps_server.max_connections = connection_count;
    hps_server.interface       = interface;

    memset( hps_server.connections, 0x00, ( sizeof(hps_connection_t) * connection_count ) );

    /* Bring up the network interface */
    result = wiced_network_up( interface, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    result = big_init_gatt_server_interface( hps_server_gatt_server_interface_callback, hps_server_gatt_connection_callback, keys_callback );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Set max number of prepare write requests GATT server should support before executing them. Enable discoverability now */
    wiced_bt_dev_set_discoverability( BTM_GENERAL_DISCOVERABLE, HTTP_SERVER_HIGH_DUTY_ADVERTISEMENT_WINDOW,
    HTTP_SERVER_HIGH_DUTY_ADVERTISEMENT_INTERVAL );

    /* Initialize GATT profile */
    wiced_bt_gatt_db_init( (uint8_t *) http_proxy_server_gatt_db, HTTP_PROXY_SERVER_GATT_DB_SIZE );

    /* Enable BLE advertisements */
    adv_data[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_data[num_elem].len          = 1;
    adv_data[num_elem].p_data       = &ble_advertisement_flag_value;

    num_elem ++;

    adv_data[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_SOLICITATION_SRV_UUID;
    adv_data[num_elem].len          = sizeof(hps_service_uuid);
    adv_data[num_elem].p_data       = (uint8_t*)&hps_service_uuid[ 0 ];

    num_elem ++;

    wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_data );

    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

    return result;
}

wiced_result_t hps_server_stop( void )
{
    big_deinit_gatt_server_interface( );

    /* Bring up the network interface */
    wiced_network_down( hps_server.interface );

    return WICED_SUCCESS;
}

wiced_result_t hps_server_register_ble_connection_callback( hps_server_ble_connect_callback_t callback )
{
    hps_server.connect_callback = callback;
    return WICED_SUCCESS;
}

wiced_result_t hps_server_register_https_certificate_callback( hps_sever_https_certificate_callback_t callback )
{
    hps_server.https_callback = callback;
    return WICED_SUCCESS;
}

static wiced_result_t hps_server_receive_timer_expired( void* arg )
{
    hps_connection_t* connection = (hps_connection_t*)arg;

    wiced_tcp_disconnect( &connection->tcp_socket );
    wiced_rtos_deregister_timed_event( &connection->receive_timer );
    connection->receive_timer_expiry = WICED_TRUE;

    return WICED_SUCCESS;
}

static void hps_server_parse_http_response( uint8_t* response, uint16_t response_length, hps_connection_t* connection )
{
    const char* start_field_ptr;
    const char* end_field_ptr;
    uint16_t    response_code_length;
    uint16_t    headers_length;
    uint32_t    response_code_value;
    uint16_t    body_length;

    REFERENCE_DEBUG_ONLY_VARIABLE( response_code_length );

    /* Get the packet, parse it and release it, send notification on the bluetooth side */
    start_field_ptr = (const char*)response;

    /* Search for HTTP version part */
    start_field_ptr = strstr( (const char*)response, "HTTP/1.1 " );
    if ( strncmp( start_field_ptr, "HTTP/1.1 ", strlen( "HTTP/1.1 " ) ) != 0 )
    {
        if ( strncmp( start_field_ptr, "HTTP/1.0 ", strlen( "HTTP/1.0 " ) ) != 0 )
        {
            wiced_assert( "HTTP version is not found", 0 != 0 );
        }
    }

    /* Parse response code */
    start_field_ptr += strlen("HTTP/1.0 ");
    end_field_ptr    = strstr( start_field_ptr, " " );
    response_code_length = string_to_unsigned( start_field_ptr, ( end_field_ptr - start_field_ptr ), &response_code_value, 0 );
    wiced_assert( "Response code character length must be always 3", response_code_length == 3 );

    /* Copy response code value */
    connection->status_code_char_value[ 0 ] = (uint8_t) (( response_code_value & 0xFF00 ) >> 8 );
    connection->status_code_char_value[ 1 ] = (uint8_t) (response_code_value & 0x00FF );
    connection->status_code_char_value[ 2 ] = 0;

    start_field_ptr = end_field_ptr;

    /* Search where the headers start */
    end_field_ptr   = strstr( start_field_ptr, "\r" );
    end_field_ptr  += strlen( "\r\n" );
    start_field_ptr = end_field_ptr;

    /* Search the end of the headers */
    end_field_ptr   = strstr( start_field_ptr, "\r\n\r\n" );

    /* Check headers length */
    headers_length = end_field_ptr - start_field_ptr;
    if ( headers_length > HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH )
    {
        connection->status_code_char_value[ 2 ] |= HPS_DATA_STATUS_HEADERS_TRUNCATED;
    }
    else
    {
        connection->status_code_char_value[ 2 ] |= HPS_DATA_STATUS_HEADERS_RECEIVED;
    }

    /* Copy headers to the headers value  */
    memcpy( connection->headers_char_value, start_field_ptr, MIN( headers_length, HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH ) );
    connection->headers_length = MIN( headers_length, HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH );
    end_field_ptr  += strlen( "\r\n\r\n" );
    start_field_ptr = end_field_ptr;

    /* Copy body */
    body_length = ( response + response_length ) - (uint8_t*) start_field_ptr;
    if ( body_length == 0 )
    {
        return;
    }

    memcpy( connection->body_char_value, start_field_ptr, MIN( body_length, HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH ) );
    connection->body_length = MIN( body_length, HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH );
    if ( body_length > HPS_BODY_CHARACTERISTIC_VALUE_LENGTH )
    {
        connection->status_code_char_value[ 2 ] |= HPS_DATA_STATUS_BODY_TRUNCATED;
    }
    else
    {
        connection->status_code_char_value[ 2 ] |= HPS_DATA_STATUS_BODY_RECEIVED;
    }
}

static void hps_server_update_state( hps_connection_t* connection, wiced_tcp_socket_t* tcp_socket, hps_http_event_t event )
{
    wiced_result_t result = WICED_SUCCESS;

    UNUSED_VARIABLE( result );

    switch( connection->state )
    {
        case HPS_IDLE:
        case HPS_BLE_CONNECTION_ESTABLISHED:
            break;

        case HPS_TCP_CONNECTION_INITIATED:
            if ( event == HPS_EVENT_HTTP_CONNECTION_ESTABLISHED )
            {
                connection->state = HPS_TCP_CONNECTION_ESTABLISHED;
                /* Connection to the remote server is established we are ready to send an http request
                 * as long as control point characteristic value is written to any value other than 0 */
                if ( connection->control_point_char_value != 0 )
                {
                    if ( connection->https_enabled == WICED_TRUE )
                    {
                        hps_server_send_https_request( connection );
                    }
                    else
                    {
                        hps_server_send_http_request( connection );
                    }
                    connection->state = HPS_HTTP_REQUEST_SENT;
                }
                else
                {
                    /* Control point is 0 */
                    connection->state = HPS_TCP_CONNECTION_ESTABLISHED;
                }
            }
            else if ( event == HPS_EVENT_HTTP_CONNECT_TIMEOUT )
            {
                wiced_tcp_disconnect( &connection->tcp_socket );

                /* Move back to the state when there is no tcp connection */
                connection->state = HPS_BLE_CONNECTION_ESTABLISHED;
            }
            break;

        case HPS_TCP_CONNECTION_ESTABLISHED:
            if ( event == HPS_EVENT_HTTP_CONNECTION_DISCONNECTED )
            {
                /* Try to connect again */
                wiced_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD, hps_server_tcp_connect_callback, (void*)connection );
            }
            break;

        case HPS_HTTP_REQUEST_SENT:
            if ( event == HPS_EVENT_HTTP_RESPONSE_RECEIVED )
            {
                uint16_t available;
                uint16_t total_available;
                uint8_t* data_ptr;

                UNUSED_VARIABLE( total_available );

                /* Stop receive timer from kicking */
                wiced_rtos_deregister_timed_event( &connection->receive_timer );

                /* Get the packet, parse it and release it, send notification on the bluetooth side */
                result = wiced_tcp_receive( &connection->tcp_socket, &connection->tcp_rx_packet, 0 );
                if ( result == WICED_SUCCESS )
                {
                    wiced_packet_get_data( connection->tcp_rx_packet, 0, &data_ptr, &available, &total_available );
                    /* Fragmented packets not supported */
                    if ( available == total_available )
                    {
                        hps_server_parse_http_response( data_ptr, available, connection );
                    }
                    wiced_packet_delete( connection->tcp_rx_packet );
                }
                wiced_tcp_disconnect( &connection->tcp_socket );

                connection->state                    = HPS_HTTP_RESPONSE_RECEIVED;
                connection->control_point_char_value = 0;

                /* Send an indication with a new status code is notification mode is turned on for status characteristic */
                if ( connection->client_char_config == GATT_CLIENT_CONFIG_NOTIFICATION )
                {
                    wiced_bt_gatt_send_notification( connection->connection_handle, GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_VALUE, sizeof( connection->status_code_char_value ), connection->status_code_char_value );
                }
                else if ( connection->client_char_config == GATT_CLIENT_CONFIG_INDICATION )
                {
                    wiced_bt_gatt_send_indication( connection->connection_handle, GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_VALUE, sizeof( connection->status_code_char_value ), connection->status_code_char_value );
                }
            }
            else if ( event == HPS_EVENT_HTTP_CONNECTION_DISCONNECTED )
            {
                wiced_tcp_delete_socket( &connection->tcp_socket );

                connection->state     = HPS_BLE_CONNECTION_ESTABLISHED;
                connection->dns_state = HOSTNAME_UNRESOLVED;

                if ( connection->receive_timer_expiry == WICED_TRUE )
                {
                    connection->status_code_char_value[ 0 ] = (uint8_t) (( HTTP_SERVER_REQUEST_TIMEOUT_STATUS_CODE & 0xFF00 ) >> 8 );
                    connection->status_code_char_value[ 1 ] = (uint8_t) ( HTTP_SERVER_REQUEST_TIMEOUT_STATUS_CODE & 0x00FF );
                    connection->status_code_char_value[ 2 ] = 0;

                    connection->control_point_char_value    = 0;

                    /* Send an indication with a new status code is notification mode is turned on for status characteristic */
                    if ( connection->client_char_config == GATT_CLIENT_CONFIG_NOTIFICATION )
                    {
                        wiced_bt_gatt_send_notification( connection->connection_handle, GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_VALUE, sizeof( connection->status_code_char_value ), connection->status_code_char_value );
                    }
                    else if ( connection->client_char_config == GATT_CLIENT_CONFIG_INDICATION )
                    {
                        wiced_bt_gatt_send_indication( connection->connection_handle, GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_VALUE, sizeof( connection->status_code_char_value ), connection->status_code_char_value );
                    }
                    connection->receive_timer_expiry = WICED_FALSE;
                }

            }
            break;

        case HPS_HTTP_RESPONSE_RECEIVED:
            if ( event == HPS_EVENT_HTTP_CONNECTION_DISCONNECTED || event == HPS_EVENT_HTTP_DISCONNECT_TIMEOUT )
            {
                wiced_tcp_delete_socket( &connection->tcp_socket );
            }

            connection->state     = HPS_BLE_CONNECTION_ESTABLISHED;
            connection->dns_state = HOSTNAME_UNRESOLVED;
            break;
    }
}

static void hps_server_dns_lookup_complete( hps_connection_t* connection, wiced_ip_address_t* address )
{
    wiced_result_t result;

    if ( address->version != 0 )
    {
        memcpy( &connection->server_ip, address, sizeof(wiced_ip_address_t) );
        hps_server_tcp_connect( connection );
        connection->dns_state = HOSTNAME_RESOLVED;
        *connection->hostname_end = '/';
    }
    else
    {
        /* Try to do DNS lookup again */
        result = wiced_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD, hps_server_hostname_lookup_callback, (void*) connection );
        wiced_assert( "DNS lookup not initiated", result == WICED_SUCCESS );
    }
}

static wiced_result_t hps_server_tcp_connect( hps_connection_t* connection )
{
    wiced_tcp_create_socket( &connection->tcp_socket, hps_server.interface );
    wiced_tcp_register_callbacks( &connection->tcp_socket, NULL, hps_server_tcp_receive_callback, hps_server_tcp_disconnect_callback, (void*)connection );
    connection->state = HPS_TCP_CONNECTION_INITIATED;

    return wiced_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD, hps_server_tcp_connect_callback, (void*)connection );
}

static wiced_result_t hps_server_perform_dns_lookup( hps_connection_t* connection )
{
    wiced_result_t result;

    connection->https_enabled         = WICED_FALSE;
    connection->hostname_end          = NULL;
    connection->hostname_start        = NULL;
    connection->hostname_end          = NULL;
    connection->hostname_found_in_uri = WICED_FALSE;

    /* Check whether URI is given in the form of the ip address */
    if ( str_to_ip( (char*) connection->uri_char_value, &connection->server_ip ) != 0 )
    {
        /* Search for the beginning of URL, usually starts after "http://" or "https://" */
        if ( ( connection->hostname_start = (uint8_t*) strstr( (const char*) connection->uri_char_value, "http://" ) ) != NULL )
        {
            connection->hostname_start += 7;
        }
        else if ( ( connection->hostname_start = (uint8_t*) strstr( (const char*) connection->uri_char_value, "https://" ) ) != NULL )
        {
            connection->hostname_start += 8;
            connection->https_enabled   = WICED_TRUE;
        }
        else if ( ( connection->hostname_start = (uint8_t*) strstr( (const char*) connection->uri_char_value, "www." ) ) != NULL )
        {
            connection->hostname_start = &connection->uri_char_value[ 0 ];
        }

        if ( connection->hostname_start != NULL )
        {
            /* Find the end of the host name, first occurrence of the '/' character will mark the end of the host name */
            connection->hostname_end = (uint8_t*) strstr( (char*) connection->hostname_start, "/" );
            if ( connection->hostname_end != NULL )
            {
                *connection->hostname_end         = '\0';
                connection->hostname_found_in_uri = WICED_TRUE;
            }
            else
            {
                /* URI hasnt been received completely, do not do hostname lookup right now */
                connection->dns_state = HOSTNAME_UNRESOLVED;
                return WICED_SUCCESS;
            }
        }
        else
        {
            if ( connection->headers_length != 0 )
            {
                connection->hostname_start = (uint8_t*) strstr( (const char*) connection->uri_char_value, "Host: " );
                if ( connection->hostname_start != NULL )
                {
                    connection->hostname_start        += strlen("Host: ");
                    connection->hostname_end          = (uint8_t*) strstr( (char*) connection->hostname_start, " " );
                    *connection->hostname_end         = '\0';
                    connection->hostname_found_in_uri = WICED_FALSE;
                }
                else
                {
                    /* HTTP headers are not complete yet, try on the next write to http headers characteristic value */
                    connection->dns_state = HOSTNAME_UNRESOLVED;
                    return WICED_SUCCESS;
                }
            }
            else
            {
                /* Hostname is not found yet, it may be found later as one of the http headers when the http headers will be known */
                connection->dns_state = HOSTNAME_UNRESOLVED;
                return WICED_SUCCESS;
            }
        }

        result = wiced_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD, hps_server_hostname_lookup_callback, (void*)connection );
        if ( result != WICED_SUCCESS )
        {
            return result;
        }
        connection->dns_state = HOSTNAME_LOOKUP_PENDING;
    }
    else
    {
        connection->dns_state = HOSTNAME_RESOLVED;
    }

    return WICED_SUCCESS;
}

static hps_connection_t* hps_server_find_hps_connection( uint16_t connection_handle )
{
    uint32_t i;

    for ( i = 0; i < hps_server.max_connections; i++ )
    {
        if ( connection_handle == hps_server.connections[ i ].connection_handle )
        {
            return &hps_server.connections[ i ];
        }
    }
    return NULL;
}

static wiced_bt_gatt_status_t hps_server_ble_connect( uint8_t *bd_address, uint16_t connection_handle, wiced_bool_t is_connected )
{
    uint32_t i;
    hps_connection_t* connection = hps_server_find_hps_connection( connection_handle );

    if ( connection != NULL )
    {
        if ( is_connected == WICED_FALSE )
        {
            /* TODO: Do a proper disconnect, tcp may be connected, hostname lookup may have started but hasn't been completed */
            return WICED_BT_GATT_SUCCESS;
        }
        else
        {
            /* TODO: Received a connection attempt to the HPS connection which has been already established */
            return WICED_BT_GATT_BUSY;
        }
    }

    /* Search a connection entry which is in the idle state and allocate it for a requested connection */
    for ( i = 0; i < hps_server.max_connections; i++ )
    {
        if ( hps_server.connections[ i ].state == HPS_IDLE )
        {
            hps_server.connections[ i ].state              = HPS_BLE_CONNECTION_ESTABLISHED;
            hps_server.connections[ i ].client_char_config = 0;
            hps_server.connections[ i ].connection_handle  = connection_handle;
            memcpy( &hps_server.connections[ i ].bd_address[ 0 ], bd_address, BD_ADDR_LEN );

            WPRINT_LIB_INFO(( "HPS server: New BLE connection has been added at index [%u] with conn_id : %d\r\n", (unsigned int)i, connection_handle ));

            return WICED_BT_GATT_SUCCESS;
        }
    }

    WPRINT_LIB_INFO(( "HPS Server: Error: Max HPS Connections already reached\r\n"));
    return WICED_BT_GATT_BUSY;
}

static void hps_server_send_https_request( hps_connection_t *connection )
{
    wiced_assert( "HTTPS isn't supported", 0!=0 );
}

static void hps_server_fill_http_request( hps_connection_t* connection, uint8_t* data_ptr, uint16_t* http_request_length )
{
    uint8_t* start_of_data_ptr = data_ptr;

    /* Add HTTP method */
    memcpy( data_ptr, connection->http_method, strlen(connection->http_method) );
    data_ptr += strlen( connection->http_method );

    /* Add URI */

    /* Check if URI contains the HTTP protocol version, ie "HTTP/1.1" */
    if ( strstr( (const char*) connection->uri_char_value, "HTTP/" ) == NULL )
    {
        if ( ( connection->hostname_end != NULL ) && ( connection->hostname_found_in_uri == WICED_TRUE ) )
        {
            uint16_t uri_length = strlen((char*) connection->uri_char_value) - ( connection->hostname_end - connection->uri_char_value );
            memcpy( data_ptr, connection->hostname_end, uri_length );
            data_ptr += uri_length;
        }
        else
        {
            memcpy( data_ptr, connection->uri_char_value, strlen((const char*) connection->uri_char_value) );
            data_ptr += strlen( (const char*) connection->uri_char_value );
        }
        memcpy( data_ptr, " HTTP/1.1\r\n", strlen(" HTTP/1.1\r\n") );
        data_ptr += strlen( " HTTP/1.1\r\n" );
    }
    else
    {
        memcpy( data_ptr, connection->uri_char_value, strlen( (const char*) connection->uri_char_value) );
        data_ptr += strlen( (const char*) connection->uri_char_value );

        /* Make sure URI has new line character at end of string */
        if ( connection->uri_char_value[ connection->uri_length - 1 ] != '\n' )
        {
            memcpy( data_ptr, "\r\n", strlen("\r\n") );
            data_ptr += strlen("\r\n");
        }
    }

    /* If there are no headers and no body, then add \r\n at the end of the uri */
    /* If there are headers but no body add \r\n\ at the end of the headers */
    if ( connection->headers_length != 0 )
    {
        char* hostname_header_start;
        hostname_header_start = strstr( (char*) connection->headers_char_value, "Host:" );
        if ( hostname_header_start == NULL )
        {
            memcpy( data_ptr, "Host: ", strlen("Host: ") );
            data_ptr += strlen("Host: ");

            memcpy( data_ptr, connection->hostname_start, ( connection->hostname_end - connection->hostname_start ) );
            data_ptr += ( connection->hostname_end - connection->hostname_start );

            memcpy( data_ptr, "\r\n", strlen("\r\n") );
            data_ptr += strlen("\r\n");
        }
    }
    else
    {
        /* Add hostname, it is a mandatory header for many servers */
        memcpy( data_ptr, "Host: ", strlen("Host: ") );
        data_ptr += strlen("Host: ");

        memcpy( data_ptr, connection->hostname_start, ( connection->hostname_end - connection->hostname_start ) );
        data_ptr += ( connection->hostname_end - connection->hostname_start );

        memcpy( data_ptr, "\r\n", strlen("\r\n") );
        data_ptr += strlen("\r\n");
    }

    /* Add headers */
    if ( connection->headers_length != 0 )
    {
        memcpy( data_ptr, connection->headers_char_value, strlen( (const char*) connection->headers_char_value ) );
        data_ptr += strlen( (const char*) connection->headers_char_value );

        /* Make sure Header has a new line characters at end of string */
        if ( connection->headers_length != 0 )
        {
            if ( connection->headers_char_value[ connection->headers_length - 1 ] != '\n' )
            {
                memcpy( data_ptr, "\r\n", strlen("\r\n") );
                data_ptr += strlen( "\r\n" );
            }
        }
    }

    /* Add body */
    if ( connection->body_length != 0 )
    {
        /* Add one more new line character at the end of the headers */
        memcpy( data_ptr, "\r\n", strlen("\r\n") );
        data_ptr += strlen("\r\n");

        memcpy( data_ptr, connection->body_char_value, connection->body_length );
        data_ptr += connection->body_length;
    }
    else
    {
        /* If there is no body after the header then we will need to append one more CRLF */
        memcpy( data_ptr, "\r\n", strlen("\r\n") );
        data_ptr += strlen("\r\n");
    }
    *http_request_length = data_ptr - start_of_data_ptr;
}

static void hps_server_send_http_request( hps_connection_t *connection )
{
    wiced_assert( "Null connection pointer", connection != NULL );

    if ( connection->dns_state == HOSTNAME_LOOKUP_TIMEOUT || connection->dns_state == HOSTNAME_UNRESOLVED )
    {
        /* Update http status with 504 response code( please see the HTTP Proxy service specifications ) */
        WPRINT_APP_INFO( ("Unable to send HTTP request now. DNS lookup failed previously\r\n") );
    }
    else if ( connection->dns_state == HOSTNAME_LOOKUP_PENDING )
    {
        /* TODO: for asynchronous DNS lookup may be needed */
        WPRINT_APP_INFO(("Unable to send HTTP request now. DNS lookup hasnt been completed yet\r\n"));
    }
    else
    {
        /* DNS look up was successful, check whether TCP connection was established successfully */
        if ( connection->state == HPS_TCP_CONNECTION_ESTABLISHED )
        {
            /* In connect state we can send HTTP request */

            /* Clear control point */
            uint8_t*       data_ptr;
            uint16_t       available;
            uint16_t       http_request_length;
            wiced_result_t result = WICED_ERROR;

            result = wiced_packet_create_tcp( &connection->tcp_socket, 1500, &connection->tcp_tx_packet, &data_ptr, &available );
            if ( result != WICED_SUCCESS )
            {
                /* TODO: */
            }
            else
            {
                hps_server_fill_http_request( connection, data_ptr, &http_request_length );
                wiced_packet_set_data_end( connection->tcp_tx_packet, data_ptr + http_request_length );
                result = wiced_tcp_send_packet( &connection->tcp_socket, connection->tcp_tx_packet );
                if ( result != WICED_SUCCESS )
                {
                    /* What to do when we were unable to send a packet?  TODO */
                    wiced_packet_delete( connection->tcp_tx_packet );
                }
            }
            connection->state = HPS_HTTP_REQUEST_SENT;

            /* Load receive timer */
            connection->receive_timer_expiry = WICED_FALSE;
            wiced_rtos_register_timed_event( &connection->receive_timer, WICED_NETWORKING_WORKER_THREAD, hps_server_receive_timer_expired,  MAX_HTTP_RESPONSE_RECEIVE_TIMEOUT, connection );

            WPRINT_LIB_INFO( ("HTTP request was sent. Waiting for HTTP reply\r\n") );
        }
        else if ( connection->state == HPS_BLE_CONNECTION_ESTABLISHED )
        {
            /* We were unable to connect, we wont try to connect again, since every change in uri is followed by the DNS lookup tcp connection attempt */

            /* return 504 response code in HTTP status characteristic value */

            /* Clear control point */
        }
        else if ( connection->state == HPS_TCP_CONNECTION_INITIATED )
        {
            /* tcp connection is in progress, wait till it is established and in the established callback send http request if HTTP control point was written */
        }
    }
}

static wiced_bt_gatt_status_t hps_server_write_client_char_config_characteristic( hps_connection_t* connection, wiced_bt_gatt_write_t* write_req )
{
    WPRINT_LIB_INFO( ("HPS Server: CLIENT CHARACTERISTIC CONFIG value write request, offset[%d],length[%d]\r\n", write_req->offset, write_req->val_len ) );

    if ( ( write_req->offset + write_req->val_len ) > 2 )
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
    else if ( write_req->val_len == 2 )
    {
        memcpy( &connection->client_char_config, write_req->p_val, write_req->val_len );

        WPRINT_LIB_INFO( ("HPS Server: New CLIENT CHARACTERISTIC CONFIG value is 0x%4x\r\n", connection->client_char_config) );
        return WICED_BT_GATT_SUCCESS;
    }
    else
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
}

static wiced_bt_gatt_status_t hps_server_write_http_body_characteristic( hps_connection_t* connection, wiced_bt_gatt_write_t* write_req )
{
    int i;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    WPRINT_LIB_INFO(("HPS Server: HTTP BODY characteristic value write request, offset[%d],length[%d]\r\n", write_req->offset, write_req->val_len ));
    if ( ( write_req->offset + write_req->val_len ) > HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH )
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
    else if ( connection->state >= HPS_HTTP_REQUEST_SENT )
    {
        /* TODO: HTTP request was sent already, we must wait until the HTTP response is received and block writing to the
         * HTTP headers characteristic until that moment */
        return WICED_BT_GATT_WRITE_NOT_PERMIT;
    }

    memcpy( &connection->body_char_value[ write_req->offset ], write_req->p_val, write_req->val_len );
    if ( ( write_req->offset + write_req->val_len ) > connection->body_length )
    {
        connection->body_length = write_req->offset + write_req->val_len;
        connection->body_char_value[ connection->body_length ] = '\0';
    }

    WPRINT_LIB_INFO(("HPS Server: New HTTP BODY characteristic value is - "));
    for ( i = 0; i < connection->body_length; i++ )
    {
        WPRINT_LIB_INFO(("0x%02x ", connection->body_char_value[ i ] ));
    }
    WPRINT_LIB_INFO(("\r\n"));
    return gatt_status;
}

static wiced_bt_gatt_status_t hps_server_write_http_headers_characteristic( hps_connection_t* connection, wiced_bt_gatt_write_t* write_req )
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    WPRINT_LIB_INFO(("HPS Server: HTTP HEADERS characteristic value write request, offset[%d],length[%d]\r\n", write_req->offset, write_req->val_len ));
    if ( ( write_req->offset + write_req->val_len ) > HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH )
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
    else if ( connection->state >= HPS_HTTP_REQUEST_SENT )
    {
        /* TODO: HTTP request was sent already, we must wait until the HTTP response is received and block writing to the
         * HTTP headers characteristic until that moment */
        return WICED_BT_GATT_WRITE_NOT_PERMIT;
    }
    memcpy( &connection->headers_char_value[ write_req->offset ], write_req->p_val, write_req->val_len );

    if ( ( write_req->offset + write_req->val_len ) > connection->headers_length )
    {
        connection->headers_length = write_req->offset + write_req->val_len;
        connection->body_char_value[ connection->headers_length ] = '\0';
    }

    /* Null terminate headers, it may be incomplete still */
    connection->headers_char_value[ connection->headers_length ] = '\0';
    WPRINT_LIB_INFO(("HPS Server: New HTTP HEADERS characteristic value is %s\r\n", connection->headers_char_value));

    return gatt_status;
}

static wiced_bt_gatt_status_t hps_server_write_http_uri_characteristic( hps_connection_t* connection, wiced_bt_gatt_write_t* write_req )
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    WPRINT_LIB_INFO(("HPS Server: URI characteristic value write request, offset[%d],length[%d]\r\n", write_req->offset, write_req->val_len ));

    if ( ( write_req->offset + write_req->val_len ) > HPS_URI_CHARACTERISTIC_VALUE_LENGTH )
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
    else if ( connection->state >= HPS_TCP_CONNECTION_INITIATED )
    {
        /* TODO: Connection has already initiated a TCP connection, we are not allowed to write URI because it will lead to
         * unpredicted behavior */
        return WICED_BT_GATT_WRITE_NOT_PERMIT;
    }

    /* On every request to write http URI starting from offset 0, clear headers and body */
    if ( write_req->offset == 0 )
    {
        /* This shouldnt be done in this way TODO */
        memset( connection->headers_char_value, 0x00, sizeof(connection->headers_char_value) );
        connection->headers_length = 0;
        memset( connection->body_char_value, 0x00, sizeof(connection->body_char_value) );
        connection->body_length = 0;
    }

    memcpy( &connection->uri_char_value[ write_req->offset ], write_req->p_val, write_req->val_len );
    if ( ( write_req->offset + write_req->val_len ) > connection->uri_length )
    {
        connection->uri_length = write_req->offset + write_req->val_len;
        connection->uri_char_value[ connection->uri_length ] = '\0';
    }

    /* NULL terminate URI, it may be incomplete still */
    connection->uri_char_value[ connection->uri_length ] = '\0';

    WPRINT_LIB_INFO( ("HPS Server: New URI characteristic value is %s\r\n", connection->uri_char_value) );

    if ( write_req->is_prep == WICED_TRUE )
    {
        WPRINT_LIB_INFO(("HPS Server: Prepare write request for URI characteristic\r\n"));
    }
    else
    {
        WPRINT_LIB_INFO(("HPS Server: Write request for URI characteristic\r\n"));
    }

    if ( write_req->is_prep == WICED_FALSE )
    {
        /* Initiate dns lookup only when the characteristic value has been written completely */
        if ( connection->dns_state == HOSTNAME_UNRESOLVED || connection->dns_state == HOSTNAME_LOOKUP_TIMEOUT )
        {
            /* On every URI write we will launch DNS lookup on the hostname if it was unsuccessful before */
            connection->dns_state = HOSTNAME_LOOKUP_PENDING;
            hps_server_perform_dns_lookup( connection );
        }
        else if ( connection->dns_state == HOSTNAME_LOOKUP_PENDING )
        {
            /* DNS lookup is still in progress, we will have to wait until it is completed */
        }
    }
//#endif /* DEBUG_NETWORKING_SIMULATION_MODE */

    return gatt_status;
}

static wiced_bt_gatt_status_t hps_server_write_http_control_point_characteristic( hps_connection_t* connection, wiced_bt_gatt_write_t* write_req )
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    WPRINT_LIB_INFO( ("HPS Server: HTTP CONTROL POINT characteristic value write request, offset[%d],length[%d]\r\n", write_req->offset, write_req->val_len ) );

    if ( ( write_req->offset + write_req->val_len ) > 1 )
    {
        return WICED_BT_GATT_INVALID_ATTR_LEN;
    }
    else if ( ( write_req->p_val[ 0 ] < HTTP_GET_REQUEST ) || ( write_req->p_val[ 0 ] > HTTP_REQUEST_CANCEL ) )
    {
        return WICED_BT_GATT_OUT_OF_RANGE;
    }
    else if ( write_req->p_val[ 0 ] == HTTP_REQUEST_CANCEL )
    {
        /* TODO: Stop DNS lookup if it is in progress */
        /* Disconnect from the remote server if it is connected already */
        /* Reset http request characteristics values */
        return WICED_BT_GATT_SUCCESS;
    }
    else if ( connection->state == HPS_HTTP_REQUEST_SENT )
    {
        /* Client is not allowed to write to the control point if HTTP request was sent already and a response hasn't been received yet */
        return WICED_BT_GATT_WRITE_NOT_PERMIT;
    }
    else
    {
        /* Check whether at least a valid URI is present */
        if ( connection->uri_length == 0 )
        {
            return WICED_BT_GATT_WRITE_NOT_PERMIT;
        }

        /* Prepare an HTTP request and send it */
        if ( write_req->p_val[ 0 ] >= HTTP_REQUEST_CANCEL )
        {
            return WICED_BT_GATT_ILLEGAL_PARAMETER;
        }

        connection->control_point_char_value = write_req->p_val[ 0 ];
        WPRINT_LIB_INFO( ("HPS Server: New HTTP CONTROL POINT characteristic value is 0x%02x\r\n", connection->control_point_char_value) );

        switch ( connection->control_point_char_value )
        {
            case HTTP_GET_REQUEST:
            case HTTPS_GET_REQUEST:
                WPRINT_APP_INFO( ( "HPS Server: Writing control point to HTTP_GET_REQUEST\n" ) );
                connection->http_method = HTTP_GET_STRING;
                break;

            case HTTP_HEAD_REQUEST:
            case HTTPS_HEAD_REQUEST:
                WPRINT_APP_INFO( ( "HPS Server: Writing control point to HTTP_HEAD_REQUEST\n" ) );
                connection->http_method = HTTP_HEAD_STRING;
                break;

            case HTTP_POST_REQUEST:
            case HTTPS_POST_REQUEST:
                WPRINT_APP_INFO( ( "HPS Server: Writing control point to HTTP_POST_REQUEST\n" ) );
                connection->http_method = HTTP_POST_STRING;
                break;

            case HTTP_PUT_REQUEST:
            case HTTPS_PUT_REQUEST:
                WPRINT_APP_INFO( ( "HPS Server: Writing control point to HTTP_PUT_REQUEST\n" ) );
                connection->http_method = HTTP_PUT_STRING;
                break;

            case HTTP_DELETE_REQUEST:
            case HTTPS_DELETE_REQUEST:
                WPRINT_APP_INFO( ( "HPS Server: Writing control point to HTTP_DELETE_REQUEST\n" ) );
                connection->http_method = HTTP_DELETE_STR;
                break;

            default:
                break;
        }
        wiced_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD,
           (event_handler_t)hps_server_send_http_request, (void*) connection );

    }

    return gatt_status;
}

static void hps_server_execute_write_characteristic( hps_connection_t* connection, uint16_t handle )
{
    switch ( handle )
    {
        case GATT_DB_HANDLE_HPS_HTTP_CONTROL_POINT_VALUE:
            wiced_assert("Execute write request should never get to control point characteristic", 0!=0);
            break;

        case GATT_DB_HANDLE_HPS_URI_VALUE:
            /* Initiate dns lookup only when the characteristic value has been written completely */
            if ( connection->dns_state == HOSTNAME_UNRESOLVED || connection->dns_state == HOSTNAME_LOOKUP_TIMEOUT )
            {
                /* On every URI write we will launch DNS lookup on the hostname if it was unsuccessful before */
                connection->dns_state = HOSTNAME_LOOKUP_PENDING;
                hps_server_perform_dns_lookup( connection );
            }
            else if ( connection->dns_state == HOSTNAME_LOOKUP_PENDING )
            {
                /* DNS lookup is still in progress, we will have to wait until it is completed */
            }
            break;

        case GATT_DB_HANDLE_HPS_HTTP_HEADERS_VALUE:
            break;

        case GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_VALUE:
            break;

        case GATT_DB_HANDLE_CLIENT_CHAR_CONFIG:
            wiced_assert("Execute write request should never get to client char characteristic", 0!=0);
            break;

        default:
            break;
    }
}

static wiced_bt_gatt_status_t hps_server_write_characteristic( hps_connection_t* connection, wiced_bt_gatt_write_t* write_req )
{
    wiced_bt_gatt_status_t status;

    switch ( write_req->handle )
    {
        case GATT_DB_HANDLE_HPS_HTTP_CONTROL_POINT_VALUE:
            status = hps_server_write_http_control_point_characteristic ( connection, write_req );
            break;

        case GATT_DB_HANDLE_HPS_URI_VALUE:
            status = hps_server_write_http_uri_characteristic ( connection, write_req );
            break;

        case GATT_DB_HANDLE_HPS_HTTP_HEADERS_VALUE:
            status = hps_server_write_http_headers_characteristic ( connection, write_req );
            break;

        case GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_VALUE:
            status = hps_server_write_http_body_characteristic ( connection, write_req );
            break;

        case GATT_DB_HANDLE_CLIENT_CHAR_CONFIG:
            status = hps_server_write_client_char_config_characteristic ( connection, write_req );
            break;

        default:
            status = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }

    return status;
}

static wiced_bt_gatt_status_t hps_server_gatt_write( wiced_bt_gatt_request_data_t* buf, uint16_t connection_handle, wiced_bt_gatt_request_type_t request_type )
{
    wiced_bt_gatt_write_t* write_req;
    hps_connection_t*      connection_ptr;
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_SUCCESS;

    connection_ptr = hps_server_find_hps_connection( connection_handle );
    wiced_assert("", connection_ptr != NULL );

    if ( request_type == GATTS_REQ_TYPE_WRITE_EXEC )
    {
        /* TODO buf->handle is not set to the proper handle value */
        hps_server_execute_write_characteristic( connection_ptr, connection_ptr->write_handle_in_progress );
    }
    else
    {
        /* TODO: */
        write_req = &buf->write_req;
        if ( write_req->offset == 0 )
        {
            connection_ptr->write_handle_in_progress = write_req->handle;
        }
        gatt_status = hps_server_write_characteristic( connection_ptr, write_req );
    }

    return gatt_status;
}

static wiced_bt_gatt_status_t hps_server_gatt_read( wiced_bt_gatt_request_data_t *buf, uint16_t connection_handle )
{
    wiced_bt_gatt_status_t status           = WICED_BT_GATT_SUCCESS;
    wiced_bt_gatt_read_t*  read_request_ptr = &buf->read_req;
    hps_connection_t*      connection_ptr   = NULL;

    connection_ptr = hps_server_find_hps_connection( connection_handle );
    wiced_assert("", connection_ptr != NULL );

    WPRINT_LIB_INFO(("GATT READ request : conn_id=%d, handle=[0x%04X], offset=[%d]\r\n", connection_handle, read_request_ptr->handle, read_request_ptr->offset));
    switch ( read_request_ptr->handle )
    {
        case GATT_DB_HANDLE_HPS_HTTP_STATUS_CODE_VALUE:
            WPRINT_LIB_INFO(( "HPS Server: GATT Read request for HTTP status code value, conn_id = %d\r\n", connection_handle ));
            *read_request_ptr->p_val_len = 3;
            memcpy( read_request_ptr->p_val, connection_ptr->status_code_char_value, 3 );
            status = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_DB_HANDLE_HPS_URI_VALUE:
            WPRINT_LIB_INFO(( "HPS Server: GATT Read request for HTTP URI value, conn_id = [%d]\r\n", connection_handle ));

            if ( connection_ptr->uri_length == 0 )
            {
                status = WICED_BT_GATT_SUCCESS;
                *read_request_ptr->p_val_len = 0;
            }
            else if ( read_request_ptr->offset < connection_ptr->uri_length )
            {
                if ( ( read_request_ptr->offset + *read_request_ptr->p_val_len ) > connection_ptr->uri_length )
                {
                    *read_request_ptr->p_val_len = connection_ptr->uri_length - read_request_ptr->offset;
                }
                memcpy( read_request_ptr->p_val, ( connection_ptr->uri_char_value + read_request_ptr->offset ), *read_request_ptr->p_val_len );
                status = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                WPRINT_LIB_INFO(( "HPS Server: Error: offset great than URI characteristic value length\r\n" ));
                WPRINT_LIB_INFO(( "HPS Server: offset = %d bytes,  length = %d bytes\r\n", read_request_ptr->offset, connection_ptr->uri_length ));
                *read_request_ptr->p_val_len = 0;
                status = WICED_BT_GATT_INVALID_OFFSET;
            }
            break;

        case GATT_DB_HANDLE_HPS_HTTP_HEADERS_VALUE:
            WPRINT_LIB_INFO(( "HPS Server: GATT Read request for HTTP headers value, conn_id = [%d]\r\n", connection_handle ));

            if ( connection_ptr->headers_length == 0 )
            {
                status = WICED_BT_GATT_SUCCESS;
                *read_request_ptr->p_val_len = 0;
            }
            else if ( read_request_ptr->offset < connection_ptr->headers_length )
            {
                if ( ( read_request_ptr->offset + *read_request_ptr->p_val_len ) > connection_ptr->headers_length )
                {
                    *read_request_ptr->p_val_len = connection_ptr->headers_length - read_request_ptr->offset;
                }
                memcpy( read_request_ptr->p_val, ( connection_ptr->headers_char_value + read_request_ptr->offset ), *read_request_ptr->p_val_len );
                status = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                WPRINT_LIB_INFO(( "HPS Server: Error: offset great than HTTP headers characteristic value length\r\n" ));
                WPRINT_LIB_INFO(( "HPS Server: offset = %d bytes,  length = %d bytes\r\n", read_request_ptr->offset, connection_ptr->headers_length ));
                *read_request_ptr->p_val_len = 0;
                status = WICED_BT_GATT_INVALID_OFFSET;
            }
            break;

        case GATT_DB_HANDLE_HPS_HTTP_ENTITY_BODY_VALUE:
            WPRINT_LIB_INFO(( "HPS Server: GATT Read request for HTTP body value, conn_id = [%d]\r\n", connection_handle ));

            if ( connection_ptr->body_length == 0 )
            {
                status                       = WICED_BT_GATT_SUCCESS;
                *read_request_ptr->p_val_len = 0;

            }
            else if ( read_request_ptr->offset < connection_ptr->body_length )
            {
                if ( ( read_request_ptr->offset + *read_request_ptr->p_val_len ) > connection_ptr->body_length )
                {
                    *read_request_ptr->p_val_len = connection_ptr->body_length - read_request_ptr->offset;
                }
                memcpy( read_request_ptr->p_val, ( connection_ptr->body_char_value + read_request_ptr->offset ), *read_request_ptr->p_val_len );
                status = WICED_BT_GATT_SUCCESS;
            }
            else
            {
                WPRINT_LIB_INFO(( "HPS Server: Error: offset great than HTTP body characteristic value length\r\n" ));
                WPRINT_LIB_INFO(( "HPS Server: offset = %d bytes,  length = %d bytes\r\n", read_request_ptr->offset, connection_ptr->body_length ));
                *read_request_ptr->p_val_len = 0;
                status = WICED_BT_GATT_INVALID_OFFSET;
            }
            break;

        case GATT_DB_HANDLE_HPS_HTTPS_SECURITY_VALUE:
            WPRINT_LIB_INFO(( "HPS Server: GATT Read request for HTTPS security value, conn_id = [%d]\r\n", connection_handle ));

            *read_request_ptr->p_val_len = 1;
            read_request_ptr->p_val[ 0 ] = connection_ptr->security_char_value;
            status = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_DB_HANDLE_CLIENT_CHAR_CONFIG:
            WPRINT_LIB_INFO(( "HPS Server: GATT Read request for client char config , conn_id = [%d]\r\n", connection_handle ));

            *read_request_ptr->p_val_len = 2;
            memcpy( read_request_ptr->p_val, &connection_ptr->client_char_config, 2 );
            status = WICED_BT_GATT_SUCCESS;
            break;

        default:
            WPRINT_LIB_INFO(( "HPS Server: Received Invalid Characteristic Handle\r\n" ));
            status = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }

    return status;
}

static wiced_bt_gatt_status_t hps_server_gatt_connection_callback( big_gatt_connection_t* connection, const big_gatt_request_t* current_request, wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t* data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
        {
            wiced_bool_t                       is_allowed           = WICED_TRUE;
            wiced_bt_gatt_connection_status_t* connection_status    = &data->connection_status;

            WPRINT_LIB_INFO( ("HPS Server: GATT_CONNECTION_STATUS_EVT received, status[%d], reason[%d]\r\n", (int)data->connection_status.connected, (int)data->connection_status.reason ) );

            if ( hps_server.connect_callback != NULL )
            {
                hps_server.connect_callback( connection_status->bd_addr, &is_allowed );
            }

            if ( is_allowed == WICED_FALSE )
            {
                /* Cancel GATT connection */
                big_ble_gatt_disconnect( connection );
                status = WICED_BT_GATT_BUSY;
                return status;
            }
            status = hps_server_ble_connect( connection_status->bd_addr, connection_status->conn_id, connection_status->connected );
        }
        break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            if ( ( data->attribute_request.request_type == GATTS_REQ_TYPE_WRITE )      ||
                 ( data->attribute_request.request_type == GATTS_REQ_TYPE_PREP_WRITE ) ||
                 ( data->attribute_request.request_type == GATTS_REQ_TYPE_WRITE_EXEC ) )
            {
                status = hps_server_gatt_write( &data->attribute_request.data, data->attribute_request.conn_id, data->attribute_request.request_type );
            }
            else if ( data->attribute_request.request_type == GATTS_REQ_TYPE_READ )
            {
                status = hps_server_gatt_read( &data->attribute_request.data, data->attribute_request.conn_id );
            }
            else
            {
                WPRINT_LIB_INFO( ("HPS Server: GATT attribute Request type not found\r\n") );
            }
            break;

        default:
            WPRINT_LIB_INFO( ("Unhandled GATT event[%u]\n", (unsigned int)event ) );
            break;
    }

    return status;
}

static wiced_result_t hps_server_gatt_server_interface_callback( big_gatt_interface_t* interface, wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* data )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    switch ( event )
    {
        default:
            WPRINT_LIB_INFO( ("Unhandled stack event[%u]\n", (unsigned int)event ) );
            break;
    }

    return ( status );
}

static wiced_result_t hps_server_hostname_lookup_callback( void* arg )
{
    hps_connection_t*  connection = (hps_connection_t*)arg;
    wiced_ip_address_t address;
    wiced_result_t result;

    result = wiced_hostname_lookup( (char*) connection->hostname_start, &address, HOSTNAME_LOOKUP_TIMEOUT_MS, WICED_STA_INTERFACE );

    if ( result != WICED_SUCCESS )
    {
        address.version = 0;
    }

    hps_server_dns_lookup_complete( connection, &address );
    return WICED_SUCCESS;
}

static wiced_result_t hps_server_tcp_connect_callback( void* arg )
{
    hps_connection_t* connection = (hps_connection_t*)arg;
    wiced_result_t    result;

    result = wiced_tcp_connect( &connection->tcp_socket, &connection->server_ip, ( ( connection->https_enabled == WICED_TRUE ) ? HTTPS_PORT : HTTP_PORT ), HPS_SERVER_TCP_CONNECTION_TIMEOUT );
    if ( result == WICED_SUCCESS )
    {
        hps_server_update_state( connection, &connection->tcp_socket, HPS_EVENT_HTTP_CONNECTION_ESTABLISHED );
    }
    else
    {
        hps_server_update_state( connection, &connection->tcp_socket, HPS_EVENT_HTTP_CONNECT_TIMEOUT );

    }
    return WICED_SUCCESS;
}

static wiced_result_t hps_server_tcp_disconnect_callback( wiced_tcp_socket_t* socket, void* arg )
{
    hps_server_update_state( (hps_connection_t*)arg, socket, HPS_EVENT_HTTP_CONNECTION_DISCONNECTED );
    return WICED_SUCCESS;
}

static wiced_result_t hps_server_tcp_receive_callback( wiced_tcp_socket_t* socket, void* arg )
{
    hps_server_update_state( (hps_connection_t*)arg, socket, HPS_EVENT_HTTP_RESPONSE_RECEIVED );
    return WICED_SUCCESS;
}
