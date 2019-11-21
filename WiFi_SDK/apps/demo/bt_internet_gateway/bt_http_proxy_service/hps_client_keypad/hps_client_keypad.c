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
#include <string.h>
#include <stdio.h>
#include "hps_client_keypad.h"
#include "hps_client_keypad_dct.h"
#include "wiced.h"
#include "platform_button.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "sdpdefs.h"
#include "gattdefs.h"
#include "bt_types.h"
#include "bt_target.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define THINGSPEAK_URI                     "http://api.thingspeak.com/update?key="
#define HPS_SERVICE_LENGTH                 (2)
#define POST_BUTTON_STATE_INTERVAL         (1000)
#define START_HPS_CHARACTERISTIC_UUID      (0x7F11)
#define HTTP_PROXY_SERVICE_UUID            (0x7F10)
#define EVENT_PROCESSOR_THREAD_STACK_SIZE  (4096)
#define EVENT_PROCESSOR_THREAD_QUEUE_SIZE  (10)

/* Data Status */
#define HPS_DATA_STATUS_HEADERS_RECEIVED   (0x01)
#define HPS_DATA_STATUS_HEADERS_TRUNCATED  (0x02)
#define HPS_DATA_STATUS_BODY_RECEIVED      (0x04)
#define HPS_DATA_STATUS_BODY_TRUNCATED     (0x08)

#define BUTTON_STATE_UPDATE_PERIOD         (2500)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    HTTP_GET_REQUEST = 1,
    HTTP_HEAD_REQUEST,
    HTTP_POST_REQUEST,
    HTTP_PUT_REQUEST,
    HTTP_DELETE_REQUEST,
    HTTPS_GET_REQUEST,
    HTTPS_HEAD_REQUEST,
    HTTPS_POST_REQUEST,
    HTTPS_PUT_REQUEST,
    HTTPS_DELETE_REQUEST,
    HTTP_REQUEST_CANCEL
} ble_hps_method_type_t;

typedef enum
{
    HPSC_IDLE,
    HPSC_SCANNING,
    HPSC_CONNECTING,
    HPSC_SERVICE_DISCOVERY,
    HPSC_CHAR_DISCOVERY,
    HPSC_DISCOVERY_COMPLETE,
    HPSC_WRITE_HTTP_STATUS_CLIENT_CHAR_VALUE,
    HPSC_WRITE_HPS_URI,
    HPSC_WRITE_HPS_CONTROL_POINT,
    HPSC_WAITING_FOR_HTTP_STATUS_INDICATION,
    HPSC_READ_HEADERS,
    HPSC_READ_BODY
} hps_client_state_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

extern wiced_bt_cfg_settings_t         wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t   wiced_bt_cfg_buf_pools[];
static wiced_bt_gatt_discovery_param_t ble_parameters;
static hps_client_state_t              ble_hps_client_state;
static uint8_t                         ble_address_type;
static uint16_t                        hps_characteristic_values_handles[ 6 ];
static uint16_t                        hps_service_start_handle;
static uint16_t                        hps_service_end_handle;
static wiced_bt_device_address_t       hps_server_device_address;
static uint8_t                         http_uri_buffer[ 512 ];
static uint16_t                        http_status_client_config_handle = 0;
static uint16_t                        connection_handle;
static uint8_t                         gatt_write_value_buffer[ sizeof( wiced_bt_gatt_value_t ) + HPS_CLIENT_MAX_ATTRIBUTE_LENGTH - 1 ];
static wiced_timed_event_t             update_button_states_event;
static wiced_bool_t                    button_states[ 2 ];
static wiced_worker_thread_t           event_processor;

#if UNUSED_GLOBAL_VARIABLE
static const configuration_entry_t const app_config[] =
{
    {"ThingSpeak channel", DCT_OFFSET(user_dct_data_t, thingspeak_channel), CHANNEL_LENGTH + 1, CONFIG_STRING_DATA },
    {"ThingSpeak key",     DCT_OFFSET(user_dct_data_t, thingspeak_key),     KEY_LENGTH + 1,     CONFIG_STRING_DATA },
    {0,0,0,0}
};
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/

static wiced_result_t         ble_hps_client_write_handle          ( uint8_t* value_ptr, uint16_t length, uint16_t handle );
static wiced_result_t         ble_hps_client_http_request_write_url( uint8_t* uri );
static wiced_bt_dev_status_t  ble_hps_client_stack_init_callback   ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                   ble_hps_client_scan_results_callback ( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );
static wiced_bt_gatt_status_t ble_hps_client_gatt_event_callback   ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );
static wiced_result_t         ble_hps_client_http_request_write_url( uint8_t* uri );
static wiced_result_t         ble_hps_client_http_request_initiate ( ble_hps_method_type_t method );
static void                   ble_hps_client_init                  ( void );
static void                   button_state_change_handler          ( platform_button_t id, wiced_bool_t new_state );
static wiced_result_t         update_button_states                 ( void* arg );

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_init( );

    wiced_rtos_create_worker_thread( &event_processor, WICED_DEFAULT_WORKER_PRIORITY, EVENT_PROCESSOR_THREAD_STACK_SIZE, EVENT_PROCESSOR_THREAD_QUEUE_SIZE );

    /* Initialize BT host stack and controller */
    wiced_bt_stack_init( ble_hps_client_stack_init_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );

    /* Initialize the buttons */
    platform_button_init( PLATFORM_BUTTON_1 );
    platform_button_init( PLATFORM_BUTTON_2 );
    platform_button_register_state_change_callback( button_state_change_handler );

    /* Register for GATT event notifications */
    ble_hps_client_init();
}

static void button_state_change_handler( platform_button_t id, wiced_bool_t new_state )
{
    button_states[ id ] = new_state;
}

static wiced_result_t update_button_states( void* arg )
{
    uint8_t*         http_uri_buffer_ptr = http_uri_buffer;
    user_dct_data_t* dct;

    wiced_rtos_deregister_timed_event( &update_button_states_event );

    wiced_dct_read_lock( (void**) &dct, WICED_FALSE, DCT_APP_SECTION, 0, sizeof(user_dct_data_t) );

    if ( ( dct->thingspeak_channel[0]    == '\x00' ) ||
         ( dct->thingspeak_key[0] == '\x00' ) )
    {
        WPRINT_APP_INFO(("ThingSpeak details not in DCT\n"));
        wiced_dct_read_unlock( dct, WICED_FALSE );
        return WICED_ERROR;
    }

    /* Create Thingspeak URI */
    memcpy( http_uri_buffer_ptr, THINGSPEAK_URI, sizeof( THINGSPEAK_URI ) - 1 );
    http_uri_buffer_ptr += sizeof( THINGSPEAK_URI ) - 1;
    memcpy( http_uri_buffer_ptr, dct->thingspeak_key, strlen( dct->thingspeak_key ) );
    http_uri_buffer_ptr += strlen( dct->thingspeak_key );

    if ( button_states[ 0 ] == WICED_FALSE )
    {
        memcpy( http_uri_buffer_ptr, "&field1=0", strlen("&field1=0") );
        http_uri_buffer_ptr += strlen("&field1=0");
    }
    else
    {
        memcpy( http_uri_buffer_ptr, "&field1=1", strlen("&field1=1") );
        http_uri_buffer_ptr += strlen("&field1=1");
    }

    if ( button_states[ 1 ] == WICED_FALSE )
    {
        memcpy( http_uri_buffer_ptr, "&field2=0", strlen("&field2=0") );
        http_uri_buffer_ptr += strlen("&field2=0");
    }
    else
    {
        memcpy( http_uri_buffer_ptr, "&field2=1", strlen("&field2=1") );
        http_uri_buffer_ptr += strlen("&field2=1");
    }
    *http_uri_buffer_ptr = '\0';

    ble_hps_client_http_request_write_url( http_uri_buffer );

    wiced_dct_read_unlock( dct, WICED_FALSE );

    return WICED_SUCCESS;
}

static wiced_result_t ble_hps_client_write_handle( uint8_t* value_ptr, uint16_t length, uint16_t handle )
{
    wiced_bt_gatt_value_t* write_param;

    write_param         = (wiced_bt_gatt_value_t *) gatt_write_value_buffer;
    write_param->handle = handle;
    write_param->len    = length;

    memcpy( &write_param->value, value_ptr, length );

    WPRINT_APP_INFO(( "Sending Write Prepare\n" ));
    WPRINT_APP_INFO(( "write_param.handle = 0x%x\n", write_param->handle ));
    WPRINT_APP_INFO(( "write_param.len = %d\n", write_param->len ));

    wiced_bt_gatt_send_write ( connection_handle, GATT_WRITE, write_param );

    memset( write_param->value, 0, wiced_bt_cfg_settings.gatt_cfg.max_attr_len );

    return WICED_SUCCESS;
}

static wiced_result_t ble_hps_client_http_request_write_url( uint8_t* uri )
{
    ble_hps_client_write_handle( uri, (uint16_t) strlen( (char*) uri ), hps_characteristic_values_handles[ GATT_UUID_HPS_URI - START_HPS_CHARACTERISTIC_UUID ] );
    ble_hps_client_state = HPSC_WRITE_HPS_URI;
    return WICED_SUCCESS;
}

static wiced_result_t ble_hps_client_http_request_initiate( ble_hps_method_type_t method )
{
    ble_hps_client_write_handle( (uint8_t*) &method, 1, hps_characteristic_values_handles[ GATT_UUID_HPS_CONTROL_POINT - START_HPS_CHARACTERISTIC_UUID ] );
    ble_hps_client_state = HPSC_WRITE_HPS_CONTROL_POINT;
    return WICED_SUCCESS;
}

static void ble_hps_client_init( void )
{
    memset( &hps_characteristic_values_handles, 0, sizeof( hps_characteristic_values_handles ) );
    ble_hps_client_state = HPSC_IDLE;

}

static wiced_bt_gatt_status_t ble_hps_client_gatt_event_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t* p_event_data )
{
    wiced_bt_gatt_status_t    status = WICED_BT_GATT_SUCCESS;
    wiced_bt_device_address_t bda = {0};

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            if ( p_event_data->connection_status.connected == WICED_TRUE )
            {
                /* We will use this connection handle to post our data to the http proxy service device */
                connection_handle = p_event_data->connection_status.conn_id;

                /* Discover HPS service and its start and end handles */
                memset( &ble_parameters, 0, sizeof(wiced_bt_gatt_discovery_param_t) );

                ble_parameters.s_handle       = 1;
                ble_parameters.e_handle       = 0xFFFF;
                ble_parameters.uuid.len       = 2;
                ble_parameters.uuid.uu.uuid16 = UUID_SERVCLASS_HPS;
                wiced_bt_gatt_send_discover( connection_handle, GATT_DISCOVER_SERVICES_BY_UUID, &ble_parameters );
                ble_hps_client_state = HPSC_SERVICE_DISCOVERY;
            }
            else
            {
                /* Device was disconnected, unregister timed event and initiate scan again */
                connection_handle = 0;

                /* Initiate a new scan and find any device which advertises HPS service */
                wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, ble_hps_client_scan_results_callback );
                ble_hps_client_state = HPSC_SCANNING;
            }
            WPRINT_APP_INFO( ( "BLE HPS CLIENT: GATT connection to HPS server [%02X:%02X:%02X:%02X:%02X:%02X] %s, conn_id = 0x%x.\n", bda[ 0 ], bda[ 1 ], bda[ 2 ], bda[ 3 ], bda[ 4 ], bda[ 5 ], ( p_event_data->connection_status.connected ? "established" : "released/failed"), p_event_data->connection_status.conn_id ) );
            break;

        case GATT_DISCOVERY_RESULT_EVT:

            if ( ble_hps_client_state == HPSC_SERVICE_DISCOVERY )
            {
                hps_service_start_handle = GATT_DISCOVERY_RESULT_SERVICE_START_HANDLE( p_event_data );
                hps_service_end_handle = GATT_DISCOVERY_RESULT_SERVICE_END_HANDLE( p_event_data );

                /* Start searching for HPS characteristics and their handles properties and values */
                ble_parameters.s_handle = hps_service_start_handle;
                ble_parameters.e_handle = hps_service_end_handle;
            }
            else if ( ble_hps_client_state == HPSC_CHAR_DISCOVERY )
            {
                uint16_t value_handle;
                uint16_t characteristic_uuid;

                /* Extract characteristic UUID  */
                characteristic_uuid = GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID16( p_event_data );

                /* Extract characteristic value handle */
                value_handle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
                hps_characteristic_values_handles[ characteristic_uuid - START_HPS_CHARACTERISTIC_UUID ] = value_handle;
            }
            else if ( ble_hps_client_state == HPSC_WRITE_HTTP_STATUS_CLIENT_CHAR_VALUE )
            {
                http_status_client_config_handle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
            }

            break;

        case GATT_DISCOVERY_CPLT_EVT:
            WPRINT_APP_INFO( ( "Received GATT_DISCOVERY_CPLT_EVT\n" ) );
            if ( ble_hps_client_state == HPSC_CHAR_DISCOVERY )
            {
                int     i = 0;
                uint8_t client_char_value[ 2 ];

                /* TODO: Check whether all characteristic value handles have been found */
                for ( i = 0; i < 6; i++ )
                {
                    if ( hps_characteristic_values_handles[ i ] == 0 )
                    {
                        wiced_assert("", 0!=0);
                    }
                }
                /* Sign up for an indication when HTTP status value changes */
                client_char_value[ 0 ] = 0x02;
                client_char_value[ 1 ] = 0x00;
                ble_hps_client_state = HPSC_WRITE_HTTP_STATUS_CLIENT_CHAR_VALUE;
                ble_hps_client_write_handle( client_char_value, 2, (uint16_t) ( hps_characteristic_values_handles[ GATT_UUID_HPS_STATUS_CODE - START_HPS_CHARACTERISTIC_UUID ] + 1 ) );
            }
            else if ( ble_hps_client_state == HPSC_SERVICE_DISCOVERY )
            {
                ble_hps_client_state = HPSC_CHAR_DISCOVERY;

                /* Start searching characteristics, we will use discover all characteristics of the service discovery type. No uuid needs to be specified */
                memset( &ble_parameters.uuid, 0, sizeof( ble_parameters.uuid ) );
                wiced_bt_gatt_send_discover( connection_handle, GATT_DISCOVER_CHARACTERISTICS, &ble_parameters );
            }

            break;

        case GATT_OPERATION_CPLT_EVT:
            switch ( ble_hps_client_state )
            {
                case HPSC_WRITE_HTTP_STATUS_CLIENT_CHAR_VALUE:
                    WPRINT_APP_INFO( ("HTTP status client characteristic handle has been updated\r\n") );
                    update_button_states( NULL );
                    ble_hps_client_state = HPSC_WRITE_HPS_URI;
                    break;

                case HPSC_WRITE_HPS_URI:
                    if ( p_event_data->operation_complete.status == WICED_BT_GATT_SUCCESS )
                    {
                        /* Write http headers */
                        WPRINT_APP_INFO( ("URI characteristic value has been written successfully\r\n") );
                        ble_hps_client_http_request_initiate( HTTP_POST_REQUEST );
                    }
                    break;

                case HPSC_WRITE_HPS_CONTROL_POINT:
                    if ( p_event_data->operation_complete.status == WICED_BT_GATT_SUCCESS )
                    {
                        WPRINT_APP_INFO( ("HTTP Control point characteristic value has been written successfully\r\n") );
                        ble_hps_client_state = HPSC_WAITING_FOR_HTTP_STATUS_INDICATION;
                    }
                    break;

                case HPSC_WAITING_FOR_HTTP_STATUS_INDICATION:
                    if ( p_event_data->operation_complete.op == GATTC_OPTYPE_INDICATION )
                    {
                        uint8_t  http_status_buffer[ 3 ];
                        uint16_t http_response_code;

                        wiced_assert("", p_event_data->operation_complete.response_data.att_value.len == 3 );

                        /* Read HTTP status and check if it is OK ,
                         * TODO: schedule reading headers operation and reading body operation */
                        memcpy( http_status_buffer, p_event_data->operation_complete.response_data.att_value.p_data, 3 );
                        http_response_code = (uint16_t) ( (uint16_t) ( http_status_buffer[ 0 ] << 8 ) | (uint16_t) ( http_status_buffer[ 1 ] ) );
                        WPRINT_APP_INFO( ("Received http response with %d response code\r\n", http_response_code) );
                        if ( ( http_status_buffer[ 2 ] & HPS_DATA_STATUS_HEADERS_RECEIVED ) != 0 )
                        {
                            /* TODO: receive headers */
                        }
                        if ( ( http_status_buffer[ 2 ] & HPS_DATA_STATUS_BODY_RECEIVED ) != 0 )
                        {
                            /* TODO: receive body */
                        }

                        /* Send confirmation to the indication */
                        wiced_bt_gatt_send_indication_confirm( p_event_data->operation_complete.conn_id, p_event_data->operation_complete.response_data.handle );

                        /* Schedule a new status update in 5000 seconds */
                        WPRINT_APP_INFO( ("Scheduling a new button states update in 5 seconds\r\n") );
                        wiced_rtos_register_timed_event( &update_button_states_event, &event_processor, update_button_states, BUTTON_STATE_UPDATE_PERIOD, NULL );
                        ble_hps_client_state = HPSC_WRITE_HPS_URI;
                    }
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }

    return status;
}

/* Callback function for LE scan results */
static void ble_hps_client_scan_results_callback( wiced_bt_ble_scan_results_t* p_scan_result, uint8_t* p_adv_data )
{
    uint8_t  adv_data_len     = 0;
    uint8_t* p_adv_data_check = NULL;

    if ( p_scan_result != NULL )
    {
        p_adv_data_check = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_SOLICITATION_SRV_UUID, &adv_data_len );
        if ( p_adv_data_check != NULL)
        {
            uint8_t uuid_temp[ 2 ];

            uuid_temp[ 1 ] = (uint8_t) ( ( HTTP_PROXY_SERVICE_UUID & 0xFF00 ) >> 8 );
            uuid_temp[ 0 ] = (uint8_t) ( HTTP_PROXY_SERVICE_UUID & 0x00FF ) ;

            if ( ( adv_data_len == HPS_SERVICE_LENGTH ) && ( memcmp( p_adv_data_check, uuid_temp, HPS_SERVICE_LENGTH ) == 0 ) )
            {
                WPRINT_APP_INFO(( "Found server advertising UUID_SERVCLASS_HPS\r\n" ));

                /* Copy the address */
                memcpy( hps_server_device_address, p_scan_result->remote_bd_addr, BD_ADDR_LEN );

                memcpy( hps_server_device_address, p_scan_result->remote_bd_addr, BD_ADDR_LEN );
                ble_address_type = p_scan_result->ble_addr_type;

                /* Cancel search on first client found - for now do only 1 connection */
                wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, NULL );
            }
        }
    }
    else /* Scan completion event occurs when the callback gets called with p_scan_result set to NULL */
    {
        if ( ( hps_server_device_address[ 0 ] | hps_server_device_address[ 1 ] | hps_server_device_address[ 2 ] |
               hps_server_device_address[ 3 ] | hps_server_device_address[ 4 ] | hps_server_device_address[ 5 ] ) != 0 )
        {
            WPRINT_APP_INFO(( "Attempt to connect to HTTP proxy service server : [%0x:%0x:%0x:%0x:%0x:%0x]\n",
                                 hps_server_device_address[ 0 ],
                                 hps_server_device_address[ 1 ],
                                 hps_server_device_address[ 2 ],
                                 hps_server_device_address[ 3 ],
                                 hps_server_device_address[ 4 ],
                                 hps_server_device_address[ 5 ] ));

            /* Initiate GATT connection to the just found peripheral device with HPS service */
            wiced_bt_gatt_le_connect( hps_server_device_address, ble_address_type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE );
            ble_hps_client_state = HPSC_CONNECTING;
        }
        else
        {
            WPRINT_APP_INFO(("Did not find any devices advertising the HPS Service"));
            wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, ble_hps_client_scan_results_callback );
        }
    }
}

/* Bluetooth management event handler */
static wiced_bt_dev_status_t ble_hps_client_stack_init_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t* p_event_data )
{
    wiced_bt_dev_status_t bt_dev_status = p_event_data->enabled.status;

    switch ( event )
    {
        case BTM_ENABLED_EVT:
            /* Initialize GATT REST API Server once Bluetooth controller and host stack is enabled */
            if ( bt_dev_status == WICED_BT_SUCCESS )
            {
                WPRINT_APP_INFO( ( "Scanning for peripheral devices\n" ) );
                wiced_bt_gatt_register( ble_hps_client_gatt_event_callback );
                wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, ble_hps_client_scan_results_callback );
                ble_hps_client_state = HPSC_SCANNING;
            }
            else
            {
                bt_dev_status = WICED_BT_ERROR;
            }
            break;

        default:
            WPRINT_LIB_INFO( ( "Unhandled event[%u]\n", (unsigned int)event ) );
            break;
    }

    return bt_dev_status;
}

