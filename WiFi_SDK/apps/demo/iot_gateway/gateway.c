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
 * Home Gateway Application
 *
 * This application demonstrates the gateway capability between sensor devices and external cloud service.
 * IoT sensor devices can send data to cloud server via this gateway device.
 * Sensor devices connect and communicates with IoT Gateway over BLE. In other end IoT Gateway is configured to connect with WiFi Access Point.
 * IoT Gateway receives sensor data from sensor devices over BLE and sends to cloud server over WiFi.
 * Also sensor device can be controlled from external cloud server via this gateway.
 *
 * Features demonstrated
 *  - Acts as BLE server (Gateway between BLE sensor device and cloud server)
 *  - Registrations with BLE stack for various events
 *  - Advertises HTTP Proxy service and WiFi config service
 *  - Process HTTP write requests from the client BLE device
 *  - connects to WiFi Access point for internet access.
 *  - Sends data to cloud service using HTTP and CoAP protocol over WiFi interface
 *  - Sends acknowledgement and notification to client device over BLE
 *
 * To demonstrate the app, work through the following steps.
 *
 * Cloud configuration
 * *************************************
 * This app is validated with Exosite and Carriots cloud server.
 *
 * Cloud host Exosite
 * *************************************
 * 1. Login to https://portals.exosite.com and create ports for each of the sensor devices.
 * 2. Use the ALIAS and CIK in the sensor device to send the http proxy request.
 *
 * 3. In iot_gateway.mk
 * - Enable the CLOUDHOST as CLOUD_HOST_EXOSITE. (by default it is enabled. So no need to do again)
 * - Enable the CLOUD_PROTOCOL as CLOUD_PROTO_HTTP if wants send data over HTTP (by default it is enabled. So no need to do again)
 * - Enable the CLOUD_PROTOCOL as CLOUD_PROTO_COAP if wants send data over CoAP
 *
 *
 * Cloud host Carriots
 * **************************************
 * 1. login to https://www.carriots.com and create devices for each of the sensor devices.
 * 2. Use the Id.Developer and API key in the sensor device to send the http proxy request.
 *
 * 3. In iot_gateway.mk
 * - Enable the CLOUDHOST as CLOUD_HOST_CARRIOTS
 * - Enable the CLOUD_PROTOCOL as CLOUD_PROTO_HTTP if you want data to be sent over HTTP (by default it is enabled. So no need to do again)
 * - Enable the CLOUD_PROTOCOL as CLOUD_PROTO_HTTPS if you want data to be sent over HTTPS
 *
 *
 * Note:
 * - At a time one cloud host and one cloud protocol is allowed. Multiple host and multiple protocol is not supported.
 * - Use the same configuration in the sensor device.
 * - Carriots+CoAP and Exosite+HTTPS is not verified.
 *
 *
 * Usage:
 * **************************************
 * 1. Plug the CYW9MCU7X9N364 board into your computer
 * 2. Build and download the application to the CYW9MCU7X9N364 board
 * 3. Sample build string "demo.iot_gateway-CYW9MCU7X9N364 UART=COM6 download". Note: Put your correct UART value
 * 4. On application start, the device acts as a GATT server and advertises cloud config and http proxy service
 * 5. Connect to GATT server using phone/companion App (LE clients) to configure the Access point information.
 * 6. Configure the  WiFi Access point credentials using Phone/companion App. Then trigger connect command from Phone/companion App.
 * 7. Home Gateway will connects to specified Access point. Receives IP address through DHCP. It waits for sensor data from client device.
 * 8. Power on one sensor device (LE client) and connects to Home Gateway. Maximum four sensor devices can connect to Home Gateway at a time.
 * 9. Sensor device can sends sensor data to Home Gateway over BLE.
 * 10. Gateway receives the sensor data. Decodes the cloud server address and payload. Then sends the payload to cloud server.
 * 11. Gateway receives acknowledgement for last sent data from the cloud server and send back to respective sensor device.
 * 12. Gateway receives the notification/control command from cloud server and sends to Sensor device (LE client)
 *
 *
 * Note: Gateway app can be used without companion app. Change the gateway_dct.c with appropriate access point credentials and set the wifi_config_connect=49.
 */

#include <string.h>
#include "wiced.h"
#include "wiced_bt_stack.h"

#include "wiced_network.h"
#include "wiced_tcpip.h"
#include "wiced_management.h"
#include "wiced_rtos.h"
#include "gateway.h"
#include "httpproxy.h"
#include "cloudconfig.h"

#include "platform_dct.h"
#include "wiced_dct_common.h"
#include "wiced_gki.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define WIFI_JOIN_THREAD_STACK_SIZE         (3 * 1024)
#define PROXY_SERVICE_THREAD_STACK_SIZE     (6 * 1024)

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/

static void iot_gateway_application_init( void );
static wiced_bt_gatt_status_t iot_gateway_gatts_connection_status_handler( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t iot_gateway_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t iot_gateway_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t iot_gateway_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t iot_gateway_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static wiced_bt_gatt_status_t iot_gateway_gatt_server_read_request_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static wiced_bt_gatt_status_t iot_gateway_gatt_server_write_request_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void iot_gateway_set_advertisement_data( void );
static void iot_gateway_send_message( void );
static wiced_bt_gatt_status_t iot_gateway_gatt_server_write_and_execute_request_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_flag );

static void create_btinit_wifijoin_thread( void );
static void create_proxyservice_thread( void );
static void proxyservice_thread( uint32_t arg );
static void btinit_wifijoin_thread( uint32_t arg );

#ifdef CLOUD_PROTO_COAP
extern wiced_result_t coap_process_request(iot_gateway_device_t *device);
extern wiced_result_t init_coap_client(void);
#else
extern wiced_result_t http_process_request( iot_gateway_device_t *device );
#endif

/******************************************************************************
 *                                Structures
 ******************************************************************************/

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void* p_attr;
} attribute_t;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[ ];

iot_gateway_device_t iot_gateway_devices[ IOT_GATEWAY_GATTS_MAX_CONN ];
wifi_info_t iot_gateway_wifiinfo;
static wiced_bool_t is_bt_connected = FALSE;
wiced_timer_t tick_timer;

attribute_t gatt_wificfg_attributes[ ] =
{
{ HANDLE_WIFI_CONFIG_SERVICE_CHAR_SSID_VAL, WIFI_SSID_LEN, iot_gateway_wifiinfo.wifi_config_ssid },
{ HANDLE_WIFI_CONFIG_SERVICE_SEC_VAL, WIFI_SECURITY_LEN, &iot_gateway_wifiinfo.wifi_config_sec },
{ HANDLE_WIFI_CONFIG_SERVICE_PCODE_VAL, WIFI_PCODE_LEN, iot_gateway_wifiinfo.wifi_config_pcode },
{ HANDLE_WIFI_CONFIG_SERVICE_CON_VAL, 1, &iot_gateway_wifiinfo.wifi_config_connect } };

attribute_t gatt_httpproxy_attributes[ ] =
{
{ HANDLE_HTTP_PROXY_SERVICE_URI_VAL, HTTP_URI_LEN, NULL },
{ HANDLE_HTTP_PROXY_SERVICE_HTTP_HEADERS_VAL, HTTP_HEADER_LEN, NULL },
{ HANDLE_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY_VAL, HTTP_BODY_LEN, NULL },
{ HANDLE_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT_VAL, 1, NULL },
{ HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE_VAL, 1, NULL },
{ HANDLE_HTTP_PROXY_SERVICE_HTTPS_SCEURITY_VAL, 1, NULL }, };

wiced_bool_t netup = WICED_FALSE;
wiced_thread_t wifijoin_handler;
wiced_thread_t proxyservice_handler;
wiced_ip_address_t ip_address;
#ifndef LOOP_TEST
wiced_semaphore_t gw_join_semaphore;
#endif
uint32_t tsec_count = 0;

/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the IoT Gateway application.  It defines
 * services, characteristics and descriptors supported by the Gateway.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t iot_gateway_gatt_database[ ] =
{
// Declare WIFI CONFIG service
        PRIMARY_SERVICE_UUID128( HANDLE_WIFI_CONFIG_SERVICE, UUID_WIFI_CONFIG_SERVICE ),

        // Declare characteristics wifi config service SSID of AP. Value of the descriptor can be modified by the client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_WIFI_CONFIG_SERVICE_CHAR_SSID, HANDLE_WIFI_CONFIG_SERVICE_CHAR_SSID_VAL, UUID_WIFI_CONFIG_CHARACTERISTIC_SSID, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

        // Declare characteristic wifi config service security type. Value of the descriptor can be modified by the client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_WIFI_CONFIG_SERVICE_SEC, HANDLE_WIFI_CONFIG_SERVICE_SEC_VAL, UUID_WIFI_CONFIG_CHARACTERISTIC_SEC, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        // Declare characteristic wifi config service passcode. Value of the descriptor can be modified by the client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_WIFI_CONFIG_SERVICE_PCODE, HANDLE_WIFI_CONFIG_SERVICE_PCODE_VAL, UUID_WIFI_CONFIG_CHARACTERISTIC_PCODE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        // Declare characteristic wifi config service connect/disconnect. Value of the descriptor can be modified by the client
        // One bye length. connect write 1. disconnect write 0
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_WIFI_CONFIG_SERVICE_CON, HANDLE_WIFI_CONFIG_SERVICE_CON_VAL, UUID_WIFI_CONFIG_CHARACTERISTIC_CON, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        // Declare HTTP Proxy service
        PRIMARY_SERVICE_UUID128( HANDLE_HTTP_PROXY_SERVICE, UUID_HTTP_PROXY_SERVICE ),

        // Declare characteristic http proxy service URI. Value of the descriptor can be modified by the client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HTTP_PROXY_SERVICE_URI, HANDLE_HTTP_PROXY_SERVICE_URI_VAL, UUID_HTTP_PROXY_SERVICE_URI, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

        // Declare characteristic http proxy service header. Value of the descriptor can be modified by the client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HTTP_PROXY_SERVICE_HTTP_HEADERS, HANDLE_HTTP_PROXY_SERVICE_HTTP_HEADERS_VAL, UUID_HTTP_PROXY_SERVICE_HTTP_HEADER, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

        // Declare characteristic http proxy service body. Value of the descriptor can be modified by the client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY, HANDLE_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY_VAL, UUID_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

        // Declare characteristic http proxy service control. Value of the descriptor can be modified by the client
        // write 1 by the client to process http proxy packets
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT, HANDLE_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT_VAL, UUID_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE ),

        // Declare characteristic http proxy service status code. This is used to send status value to client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE, HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE_VAL, UUID_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

        // Declare characteristic http proxy service security. Value of the descriptor can be modified by the client
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HTTP_PROXY_SERVICE_HTTPS_SCEURITY, HANDLE_HTTP_PROXY_SERVICE_HTTPS_SCEURITY_VAL, UUID_HTTP_PROXY_SERVICE_HTTPS_SCEURITY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE, LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ), };

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */

void application_start( void )
{
    wiced_result_t result;
    WPRINT_APP_INFO( ( "IoT Gateway Starts\n" ) );
    result = wiced_init( );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("wiced_init failed %d\n", result) );
        return;
    }
    /* Register call back and configuration with stack */
    wiced_bt_stack_init( iot_gateway_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}

void gateway_gatt_send_notification( uint16_t conn_id, uint16_t code, uint16_t val_len, uint8_t *p_val )
{
    WPRINT_APP_DEBUG (("conn_id=%d code=%d value=%s\n",conn_id, code, p_val));
    wiced_bt_gatt_send_notification( conn_id, code, val_len, p_val );
}

static wiced_result_t iot_gateway_start_bt( )
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t result = WICED_SUCCESS;

    gatt_status = wiced_bt_gatt_register( iot_gateway_gatts_callback );
    gatt_status = wiced_bt_gatt_db_init( iot_gateway_gatt_database, sizeof( iot_gateway_gatt_database ) );
    WPRINT_APP_INFO( ( "wiced_bt_gatt_db_init %d\n", gatt_status ) );
    iot_gateway_set_advertisement_data( );
    result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WPRINT_APP_INFO( ( "wiced_bt_start_advertisements %d\n", result ) );

    return result;
}

static void wifi_config( char* ssid, uint8_t ssid_length, wiced_security_t auth_type, uint8_t* key, uint16_t key_length, char* ip, char* netmask, char* gateway )
{
    platform_dct_wifi_config_t* dct_wifi_config;
    wiced_result_t result;

    /* Read config */
    result = wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("DCT read failed for WiFi config section \n") );
        if ( dct_wifi_config != NULL )
            wiced_dct_read_unlock( dct_wifi_config, WICED_TRUE );
        return;
    }
    /* Modify config */
    dct_wifi_config->stored_ap_list[ 0 ].details.SSID.length = ssid_length;
    memcpy( (char*) dct_wifi_config->stored_ap_list[ 0 ].details.SSID.value, ssid, SSID_NAME_SIZE );
    dct_wifi_config->stored_ap_list[ 0 ].details.security = auth_type;
    if ( ( auth_type & ENTERPRISE_ENABLED ) == 0 )
    {
        /* Save credentials for non-enterprise AP */
        memcpy( (char*) dct_wifi_config->stored_ap_list[ 0 ].security_key, (char*) key, SECURITY_KEY_SIZE );
        dct_wifi_config->stored_ap_list[ 0 ].security_key_length = key_length;
    }

    /* Write config */
    result = wiced_dct_write( (const void*) dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );

    WPRINT_APP_DEBUG (("DCT WiFi config write : %d\n",result));

    wiced_dct_read_unlock( dct_wifi_config, WICED_TRUE );
}

/*!
 ******************************************************************************
 * Convert a security authentication type string to a wiced_security_t.
 *
 * @param[in] arg  The string containing the value.
 *
 * @return    The value represented by the string.
 */
static wiced_security_t str_to_authtype( char* arg )
{
    if ( strcmp( arg, "open" ) == 0 )
    {
        return WICED_SECURITY_OPEN;
    }
    else if ( strcmp( arg, "wep" ) == 0 )
    {
        return WICED_SECURITY_WEP_PSK;
    }
    else if ( strcmp( arg, "wep_shared" ) == 0 )
    {
        return WICED_SECURITY_WEP_SHARED;
    }
    else if ( strcmp( arg, "wpa2_tkip" ) == 0 )
    {
        return WICED_SECURITY_WPA2_TKIP_PSK;
    }
    else if ( strcmp( arg, "wpa2_aes" ) == 0 )
    {
        return WICED_SECURITY_WPA2_AES_PSK;
    }
    else if ( strcmp( arg, "wpa2" ) == 0 )
    {
        return WICED_SECURITY_WPA2_MIXED_PSK;
    }
    else if ( strcmp( arg, "wpa_aes" ) == 0 )
    {
        return WICED_SECURITY_WPA_AES_PSK;
    }
    else if ( strcmp( arg, "wpa_tkip" ) == 0 )
    {
        return WICED_SECURITY_WPA_TKIP_PSK;
    }
    else
    {
        WPRINT_APP_INFO( ("Bad auth type: '%s'. Returning SEC_OPEN \r\n", arg) );
        return WICED_SECURITY_OPEN;
    }
}

static int wifi_net_up( )
{
    wiced_result_t result;
    wiced_security_t auth_type = str_to_authtype( (char*) iot_gateway_wifiinfo.wifi_config_sec );
    uint32_t sta_channel;

    wifi_config( (char*) iot_gateway_wifiinfo.wifi_config_ssid, (uint8_t) strlen( (char*) iot_gateway_wifiinfo.wifi_config_ssid ), auth_type, iot_gateway_wifiinfo.wifi_config_pcode, (uint16_t) strlen( (char*) iot_gateway_wifiinfo.wifi_config_pcode ), NULL, NULL, NULL );

    result = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("wiced_network_up failed\r\n") );
        return result;
    }

    while ( wiced_network_is_ip_up( WICED_STA_INTERFACE ) != WICED_TRUE )
    {
        wiced_rtos_delay_milliseconds( 500 );
        WPRINT_APP_DEBUG (("Waiting for network to up\n"));
    }

    wiced_wifi_get_channel( &sta_channel );
    WPRINT_APP_INFO( ("wiced_network_up success. WiFi Channel: %d\r\n", sta_channel) );
    return result;
}

static void btinit_wifijoin_thread( uint32_t arg )
{
    wiced_result_t result;
    wifi_info_t * dct_wifi_info;
#ifdef LOOP_TEST
    char writeval[25];
    int val = 0;
#endif
    wiced_platform_mcu_enable_powersave();
    result = wiced_dct_read_lock( (void**) &dct_wifi_info, WICED_TRUE, DCT_APP_SECTION, 0, sizeof(wifi_info_t) );

    if ( result == WICED_SUCCESS )
    {
        memcpy( &iot_gateway_wifiinfo, dct_wifi_info, sizeof(wifi_info_t) );

        if ( dct_wifi_info->wifi_config_connect == '1' )
        {
            result = wifi_net_up( );
            if ( result == WICED_SUCCESS )
            {
                netup = WICED_TRUE;
            }
        }
    }
    else
    {
        WPRINT_APP_DEBUG (("DCT READ Result : %d :: value : %d \n", result, dct_wifi_info->wifi_config_connect));
    }
    wiced_dct_read_unlock( dct_wifi_info, WICED_TRUE );

    result = iot_gateway_start_bt( );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("BT init failed \n") );
        return;
    }

#ifndef LOOP_TEST
    while ( 1 )
    {
        if ( wiced_rtos_get_semaphore( &gw_join_semaphore, 30000 ) == WICED_SUCCESS )
        {
            netup = WICED_FALSE;

            if ( iot_gateway_wifiinfo.wifi_config_connect == 0x30 )
            {
                result = wiced_network_down( WICED_STA_INTERFACE );
                WPRINT_APP_DEBUG (("wiced_network_down result: %d\n",result));
            }
            else if ( iot_gateway_wifiinfo.wifi_config_connect == 0x31 )
            {
                result = wiced_network_down( WICED_STA_INTERFACE );
                WPRINT_APP_DEBUG (("wiced_network_down result: %d\n",result));

                result = wifi_net_up( );
                if ( result == WICED_SUCCESS )
                {
                    netup = WICED_TRUE;
                    WPRINT_APP_INFO( (" ############## REBOOT HGW ##############\n") );
                }
            }
            else
            {
                WPRINT_APP_INFO( ("Command [%d] not handled\n",iot_gateway_wifiinfo.wifi_config_connect) );
            }
        }
        else
        {
            WPRINT_APP_DEBUG (("Timeout: Waiting for wifi config\n"));
        }
    }
#endif

#ifdef LOOP_TEST
#ifndef CLOUD_HOST_CARRIOTS
    /*push dummy observe packet to Queue*/
    iot_gateway_devices[1].http_proxy_info.http_proxy_control = HTTP_REQ_OBSERVE;
    memcpy(iot_gateway_devices[1].http_proxy_info.http_proxy_uri,CLOUD_HTTP_URI, strlen(CLOUD_HTTP_URI));
    sprintf((char*)iot_gateway_devices[1].http_proxy_info.http_proxy_body,"{\"auth\":{\"cik\":\"%s\"},\"calls\":[{\"id\":%d,\"procedure\":\"wait\",\"arguments\":[{\"alias\":\"%s\"},{\"timeout\":%d}]}]}",
            CLOUD_CLIENT_KEY,
            111,
            CLOUD_DATAPORT_ALIAS_CONTROL,
            CLOUD_OBSERVE_TIMER);
    sprintf((char*)iot_gateway_devices[1].http_proxy_info.http_proxy_header,"Host:m2.exosite.com\r\nContent-Type:application/json; charset=utf-8\r\nContent-Length:%d",
            strlen((char*)iot_gateway_devices[1].http_proxy_info.http_proxy_body));

    if (wiced_rtos_push_to_queue( &httpservq, &(iot_gateway_devices[1]), WICED_NO_WAIT ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("PUSH FAILED !!!\n"));
    }
    WPRINT_APP_DEBUG (("pushed observe %d\n",iot_gateway_devices[1].http_proxy_info.http_proxy_control));
#endif//#ifndef CLOUD_HOST_CARRIOTS
    /* write dummy post data*/
    while (1)
    {
        wiced_rtos_delay_milliseconds(10000);
        val++;
        sprintf(writeval,"%d",val);
        iot_gateway_devices[0].http_proxy_info.http_proxy_control = HTTP_REQ_POST;
        sprintf((char *) iot_gateway_devices[0].http_proxy_info.http_proxy_uri, "%s", CLOUD_HTTP_URI);

#ifdef CLOUD_HOST_CARRIOTS
        sprintf((char*)iot_gateway_devices[0].http_proxy_info.http_proxy_body,"{\"protocol\":\"v2\",\"device\":\"%s\",\"at\":\"now\",\"data\":%s}",SENSOR_ID,SENSOR_DATA);
#else
        sprintf((char*)iot_gateway_devices[0].http_proxy_info.http_proxy_body,"{\"auth\":{\"cik\":\"%s\"},\"calls\":[{\"id\":%d,\"procedure\":\"write\",\"arguments\":[{\"alias\":\"%s\"},\"%s\"]}]}",
                CLOUD_CLIENT_KEY,
                222,
#ifdef CLOUD_PROTO_COAP
                CLOUD_DATAPORT_ALIAS_COAP,
#else
                CLOUD_DATAPORT_ALIAS_HTTP,
#endif//#ifdef CLOUD_PROTO_COAP
                writeval);
#endif//#ifdef CLOUD_HOST_CARRIOTS

#ifdef CLOUD_HOST_CARRIOTS
        sprintf((char*)iot_gateway_devices[0].http_proxy_info.http_proxy_header,"Host:api.carriots.com\r\nContent-Type:application/json; charset=utf-8\r\nCarriots.apiKey: %s\r\nContent-Length:%d",
                CLOUD_API_KEY,strlen((char*)iot_gateway_devices[0].http_proxy_info.http_proxy_body));
#else
        sprintf((char*)iot_gateway_devices[0].http_proxy_info.http_proxy_header,"Host:m2.exosite.com\r\nContent-Type:application/json; charset=utf-8\r\nContent-Length:%d",
                strlen((char*)iot_gateway_devices[0].http_proxy_info.http_proxy_body));
#endif//#ifdef CLOUD_HOST_CARRIOTS
        if (wiced_rtos_push_to_queue( &httpservq, &(iot_gateway_devices[0]), WICED_NO_WAIT ) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO (("PUSH FAILED !!!\n"));
        }

    }            //while

#endif//#ifdef LOOP_TEST
}

static void create_btinit_wifijoin_thread( void )
{
    uint32_t stack_size = WIFI_JOIN_THREAD_STACK_SIZE;
    wiced_result_t result;

    result = wiced_rtos_create_thread( &wifijoin_handler, 3, "wifi join Thrd", btinit_wifijoin_thread, stack_size, NULL );
    if ( result )
    {
        WPRINT_APP_INFO( ( "join thread init failed !!!!!!!!!\n" ) );
        return;
    }
}

/**
 * The purpose of this thread is to free application start
 *
 */
static void proxyservice_thread( uint32_t arg )
{
    wiced_result_t result;
    iot_gateway_device_t current_event;

    WPRINT_APP_DEBUG (( "Started httpservice_thread\n" ));

    do
    {
        /*wait for network up*/
        wiced_rtos_delay_milliseconds( 100 );
    } while ( wiced_network_is_ip_up( WICED_STA_INTERFACE ) != WICED_TRUE );

#ifdef CLOUD_PROTO_COAP
    result = init_coap_client();
    if(result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO (("coap init failed. Exiting Thread \n"));
        return;
    }
#else
    result = init_http_client();
    if(result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO (("http init failed. Exiting Thread \n"));
        return;
    }
#endif

    while ( 1 )
    {
        /* check for network link */
        if ( wiced_network_is_ip_up( WICED_STA_INTERFACE ) != WICED_TRUE )
        {
            wiced_rtos_delay_milliseconds( 100 );
            continue;
        }

        WPRINT_APP_DEBUG (("httpservice_thread: Waiting for event \n"));

        result = wiced_rtos_pop_from_queue( &httpservq, &current_event, WICED_NEVER_TIMEOUT );
        WPRINT_APP_DEBUG (("httpservice_thread: pop %d \n",result));
        if ( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("Error in popping\n") );
            continue;
        }

#ifdef CLOUD_PROTO_COAP
        result = coap_process_request(&current_event);
#else
        result = http_process_request( &current_event );
#endif
    }

}

static void create_proxyservice_thread( void )
{
    uint32_t stack_size = PROXY_SERVICE_THREAD_STACK_SIZE;
    wiced_result_t result;
    result = wiced_rtos_create_thread( &proxyservice_handler, 2, "proxy serv Thrd", proxyservice_thread, stack_size, NULL );
    if ( result )
    {
        WPRINT_APP_INFO( ( "Thread init failed !!!!!!!!!\n" ) );
        return;
    }

}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
static void iot_gateway_application_init( void )
{
    wiced_result_t result;
    int32_t i;

    WPRINT_APP_DEBUG (( "iot_gateway_application_init\n" )) ;

    netup = WICED_FALSE;

    result = wiced_rtos_init_queue( &httpservq, NULL, sizeof(http_proxy_info_t), EVENT_QUEUE_DEPTH );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("httpservq init failed\n") );
        return;
    }
#ifndef LOOP_TEST
    /* init semaphores */
    wiced_rtos_init_semaphore( &gw_join_semaphore );
#endif
    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    for ( i = 0; i < IOT_GATEWAY_GATTS_MAX_CONN; i++ )
    {
        iot_gateway_devices[ i ].conn_id = 0;
        iot_gateway_devices[ i ].flag_stay_connected = 1;
        memset( iot_gateway_devices[ i ].remote_addr, 0, sizeof(BD_ADDR) );
    }

    create_proxyservice_thread( );

    create_btinit_wifijoin_thread( );

}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void iot_gateway_set_advertisement_data( void )
{
    wiced_result_t result;
    wiced_bt_ble_advert_elem_t adv_elem[ 3 ];
    uint8_t ble_advertisement_flag_value = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0;
    uint8_t hp_service_uuid[ LEN_UUID_128 ] =
    { UUID_HTTP_PROXY_SERVICE };

    adv_elem[ num_elem ].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[ num_elem ].len = 1;
    adv_elem[ num_elem ].p_data = &ble_advertisement_flag_value;
    num_elem++;

    adv_elem[ num_elem ].advert_type = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[ num_elem ].len = LEN_UUID_128;
    adv_elem[ num_elem ].p_data = hp_service_uuid;
    num_elem++;

    adv_elem[ num_elem ].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[ num_elem ].len = strlen( (const char *) wiced_bt_cfg_settings.device_name );
    WPRINT_APP_DEBUG( ("wiced_bt_cfg_settings.device_name:%s\n", wiced_bt_cfg_settings.device_name));
    adv_elem[ num_elem ].p_data = (uint8_t *) wiced_bt_cfg_settings.device_name;

    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );

    WPRINT_APP_INFO( ( "wiced_bt_ble_set_advertisement_data %d\n", result ) );
}

/*
 * This function is invoked when advertisements stop.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 */
void iot_gateway_advertisement_stopped( void )
{
    wiced_result_t result;
    int i = 0;

    for ( i = 0; i < IOT_GATEWAY_GATTS_MAX_CONN; i++ )
    {
        if ( iot_gateway_devices[ i ].conn_id == 0 )
        {
            break; // still more connection can happens
        }
    }

    if ( i < IOT_GATEWAY_GATTS_MAX_CONN )
    {
        result = wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        WPRINT_APP_INFO( ( "wiced_bt_start_advertisements: %d\n", result ) );
    }
    else
    {
        WPRINT_APP_INFO( ( "ADV stop\n") );
    }
}

/*
 * IoT gateway bt/ble link management callback
 */
static wiced_result_t iot_gateway_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_dev_ble_pairing_info_t *p_info;
    wiced_bt_ble_advert_mode_t *p_mode;

    WPRINT_APP_DEBUG (("iot_gateway_management_callback: %x\n", event ));

    switch ( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            iot_gateway_application_init( );
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
            WPRINT_APP_INFO( ( "Pairing Complete: %d",p_info->reason) );
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WPRINT_APP_INFO( ( "Advertisement State Change: %d\n", *p_mode) );
            if ( *p_mode == BTM_BLE_ADVERT_OFF )
            {
                iot_gateway_advertisement_stopped( );
            }
            break;

        default:
            break;
    }

    return result;
}

/*
 * Check if client has registered for notification/indication
 * and send message if appropriate
 */
static void iot_gateway_send_message( void )
{
    WPRINT_APP_DEBUG (( "iot_gateway_send_message: CCC:%d\n", iot_gateway_devices[0].flag_indication_sent )) ;

    if ( !iot_gateway_devices[ 0 ].flag_indication_sent )
    {
        iot_gateway_devices[ 0 ].flag_indication_sent = TRUE;
    }
}

/*
 * Find attribute description by handle
 */
static attribute_t * iot_gateway_get_attribute( uint16_t handle, uint16_t conn_id )
{
    int i, j;
    for ( i = 0; i < sizeof( gatt_wificfg_attributes ) / sizeof( gatt_wificfg_attributes[ 0 ] ); i++ )
    {
        if ( gatt_wificfg_attributes[ i ].handle == handle )
        {
            return ( &gatt_wificfg_attributes[ i ] );
        }
    }
    for ( i = 0; i < sizeof( gatt_httpproxy_attributes ) / sizeof( gatt_httpproxy_attributes[ 0 ] ); i++ )
    {
        if ( gatt_httpproxy_attributes[ i ].handle == handle )
        {
            for ( j = 0; j < IOT_GATEWAY_GATTS_MAX_CONN; j++ )
                if ( iot_gateway_devices[ j ].conn_id == conn_id )
                {
                    switch ( handle )
                    {
                        case HANDLE_HTTP_PROXY_SERVICE_URI_VAL:
                            gatt_httpproxy_attributes[ i ].p_attr = iot_gateway_devices[ j ].http_proxy_info.http_proxy_uri;
                            break;
                        case HANDLE_HTTP_PROXY_SERVICE_HTTP_HEADERS_VAL:
                            gatt_httpproxy_attributes[ i ].p_attr = iot_gateway_devices[ j ].http_proxy_info.http_proxy_header;
                            break;
                        case HANDLE_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY_VAL:
                            gatt_httpproxy_attributes[ i ].p_attr = iot_gateway_devices[ j ].http_proxy_info.http_proxy_body;
                            break;
                        case HANDLE_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT_VAL:
                            gatt_httpproxy_attributes[ i ].p_attr = &iot_gateway_devices[ j ].http_proxy_info.http_proxy_control;
                            break;
                        case HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE_VAL:
                            gatt_httpproxy_attributes[ i ].p_attr = &iot_gateway_devices[ j ].http_proxy_info.http_proxy_status;
                            break;
                        case HANDLE_HTTP_PROXY_SERVICE_HTTPS_SCEURITY_VAL:
                            gatt_httpproxy_attributes[ i ].p_attr = &iot_gateway_devices[ j ].http_proxy_info.https_security;
                            break;
                    }
                    break;
                }
            return ( &gatt_httpproxy_attributes[ i ] );
        }
    }
    WPRINT_APP_INFO( ( "attribute not found:%x\n", handle ) );
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
static wiced_bt_gatt_status_t iot_gateway_gatt_server_read_request_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int attr_len_to_copy;

    if ( ( puAttribute = iot_gateway_get_attribute( p_read_data->handle, conn_id ) ) == NULL )
    {
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->attr_len;

    WPRINT_APP_DEBUG (("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy));

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int to_copy = attr_len_to_copy - p_read_data->offset;

        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ( (uint8_t *) puAttribute->p_attr ) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy );
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
static wiced_bt_gatt_status_t iot_gateway_gatt_server_write_request_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;
    attribute_t *puAttribute;
    int attr_len_to_copy;
#ifndef LOOP_TEST
    int j = 0;
#endif

    WPRINT_APP_DEBUG (("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len ));
    if ( ( puAttribute = iot_gateway_get_attribute( p_data->handle, conn_id ) ) == NULL )
    {
        WPRINT_APP_INFO( ("write_hndlr attr not found hdl:%x\n", p_data->handle ) );
        return WICED_BT_GATT_INVALID_HANDLE;
    }
    attr_len_to_copy = puAttribute->attr_len;
    if ( p_data->offset == 0 )
        memset( (uint8_t *) puAttribute->p_attr, 0, attr_len_to_copy );

    if ( p_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }
    if ( attr_len_to_copy != 0 )
    {
        uint8_t *to;
        int to_copy = attr_len_to_copy - p_data->offset;
        if ( to_copy > p_data->val_len )
        {
            to_copy = p_data->val_len;
        }
        to = ( (uint8_t *) puAttribute->p_attr ) + p_data->offset;
        memcpy( to, p_data->p_val, to_copy );
    }
    switch ( p_data->handle )
    {
        /* By writing into Characteristic Client Configuration descriptor
         * peer can enable or disable notification or indication */
        case HANDLE_WIFI_CONFIG_SERVICE_CHAR_SSID_VAL:
            WPRINT_APP_DEBUG (("### SSID ### \n %s \n",iot_gateway_wifiinfo.wifi_config_ssid));
            break;
        case HANDLE_WIFI_CONFIG_SERVICE_SEC_VAL:
            WPRINT_APP_DEBUG (("### SECURITY ### \n %s \n",iot_gateway_wifiinfo.wifi_config_sec));
            break;
        case HANDLE_WIFI_CONFIG_SERVICE_PCODE_VAL:
            WPRINT_APP_DEBUG (("### PASSCODE ### \n %s \n",iot_gateway_wifiinfo.wifi_config_pcode));
            break;
        case HANDLE_WIFI_CONFIG_SERVICE_CON_VAL:
        {
            if ( iot_gateway_wifiinfo.wifi_config_connect == '1' || iot_gateway_wifiinfo.wifi_config_connect == '0' )
            {
                WPRINT_APP_INFO( ("%s\n",(iot_gateway_wifiinfo.wifi_config_connect == '1')?"### CONNECT ###":"### DISCONNECT ###") );
                /* set dct */
                result = wiced_dct_write( (const void*) &iot_gateway_wifiinfo, DCT_APP_SECTION, 0, sizeof(wifi_info_t) );
                WPRINT_APP_DEBUG (("wiced_dct_write : %d\n",result));
            }
#ifndef LOOP_TEST
            WPRINT_APP_DEBUG (("setting semaphore \n"));
            wiced_rtos_set_semaphore( &gw_join_semaphore );
#endif
        }
            break;
        case HANDLE_HTTP_PROXY_SERVICE_URI_VAL:
            break;
        case HANDLE_HTTP_PROXY_SERVICE_HTTP_HEADERS_VAL:
            break;
        case HANDLE_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY_VAL:
            break;
        case HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE_VAL:
            break;
        case HANDLE_HTTP_PROXY_SERVICE_HTTPS_SCEURITY_VAL:
            WPRINT_APP_DEBUG (("### HTTPS :: %d ###\n", iot_gateway_devices[0].http_proxy_info.https_security));
            break;
        case HANDLE_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT_VAL:
#ifndef LOOP_TEST
            if ( netup == WICED_TRUE )
            {
                for ( j = 0; j < IOT_GATEWAY_GATTS_MAX_CONN; j++ )
                    if ( iot_gateway_devices[ j ].conn_id == conn_id )
                    {
                        if ( wiced_rtos_push_to_queue( &httpservq, &( iot_gateway_devices[ j ] ), WICED_NO_WAIT ) != WICED_SUCCESS )
                        {
                            WPRINT_APP_INFO( ("PUSH FAILED !!!\n") );
                        }
                        else
                        {

                            WPRINT_APP_DEBUG (("PUSH SUCC !!!\n"));
                        }
                        break;
                    }
            }
            else
            {
                WPRINT_APP_DEBUG (("network is not up \n"));
            }
#endif
            break;
        default:
            result = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }
    return result;
}

/*
 * Write Execute Procedure
 */
static wiced_bt_gatt_status_t iot_gateway_gatt_server_write_and_execute_request_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_flag )
{
    WPRINT_APP_DEBUG (("write exec: flag:%d\n", exec_flag));
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
static wiced_bt_gatt_status_t iot_gateway_gatt_server_mtu_request_handler( uint16_t conn_id, uint16_t mtu )
{
    WPRINT_APP_DEBUG (("req_mtu: %d\n", mtu));
    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t iot_gateway_gatt_server_confirmation_handler( uint16_t conn_id, uint16_t handle )
{
    WPRINT_APP_DEBUG (( "iot_gateway_indication_confirmation, conn %d hdl %d\n", conn_id, handle ));

    if ( !iot_gateway_devices[ 0 ].flag_indication_sent )
    {
        WPRINT_APP_INFO( ("iot_gateway: Wrong Confirmation!") );
        return WICED_BT_GATT_SUCCESS;
    }
    iot_gateway_devices[ 0 ].flag_indication_sent = 0;
    iot_gateway_send_message( );

    return WICED_BT_GATT_SUCCESS;
}
/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t iot_gateway_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    int i = 0;

    WPRINT_APP_INFO( ( "iot_gateway_conn_up %B id:%d\n:", p_status->bd_addr, p_status->conn_id) );

    /* Update the connection handler.  Save address of the connected device. */

    for ( i = 0; i < IOT_GATEWAY_GATTS_MAX_CONN; i++ )
    {
        if ( iot_gateway_devices[ i ].conn_id == 0 )
        {
            /* Update the connection handler.  Save address of the connected device. */
            iot_gateway_devices[ i ].conn_id = p_status->conn_id;
            memcpy( iot_gateway_devices[ i ].remote_addr, p_status->bd_addr, sizeof(BD_ADDR) );
            WPRINT_APP_INFO( ("Alloted new index in Gateway devices :: %d\n", i) );
            break;
        }

    }

    /* Stop advertising. Adv restarted in "advertisement_stopped" handler
     * depending upon number of active connection */
    i = wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
    WPRINT_APP_DEBUG (( "Stopping Advertisements : %d\n",i));

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
static wiced_bt_gatt_status_t iot_gateway_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    int i;

    for ( i = 0; i < IOT_GATEWAY_GATTS_MAX_CONN; i++ )
    {
        if ( p_status->conn_id == iot_gateway_devices[ i ].conn_id )
        {
            WPRINT_APP_INFO (( "connection_down %B conn_id:%d reason:%d\n", iot_gateway_devices[i].remote_addr, p_status->conn_id, p_status->reason ));
         /* Resetting the device info */
            memset( iot_gateway_devices[i].remote_addr, 0, 6 );
            iot_gateway_devices[i].conn_id = 0;
            WPRINT_APP_DEBUG (("Cleared from Gateway device list index :: %d\n", i));

            wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        }
    }

    return WICED_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
static wiced_bt_gatt_status_t iot_gateway_gatts_connection_status_handler( wiced_bt_gatt_connection_status_t *p_status )
{
    is_bt_connected = p_status->connected;
    if ( p_status->connected )
    {
        return iot_gateway_gatts_connection_up( p_status );
    }

    return iot_gateway_gatts_connection_down( p_status );
}

/*
 * Process GATT request from the peer
 */
static wiced_bt_gatt_status_t iot_gateway_gatt_server_request_handler( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WPRINT_APP_DEBUG (( "gatt_server_request_handler. conn %d, type %d\n", p_data->conn_id, p_data->request_type ));

    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = iot_gateway_gatt_server_read_request_handler( p_data->conn_id, &( p_data->data.read_req ) );
            break;

        case GATTS_REQ_TYPE_PREP_WRITE:
        case GATTS_REQ_TYPE_WRITE:
            result = iot_gateway_gatt_server_write_request_handler( p_data->conn_id, &( p_data->data.write_req ) );
            break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            result = iot_gateway_gatt_server_write_and_execute_request_handler( p_data->conn_id, p_data->data.exec_write );
            break;

        case GATTS_REQ_TYPE_MTU:
            result = iot_gateway_gatt_server_mtu_request_handler( p_data->conn_id, p_data->data.mtu );
            break;

        case GATTS_REQ_TYPE_CONF:
            result = iot_gateway_gatt_server_confirmation_handler( p_data->conn_id, p_data->data.handle );
            break;

        default:
            break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of
 * the events are ommitted.
 */
static wiced_bt_gatt_status_t iot_gateway_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = iot_gateway_gatts_connection_status_handler( &p_data->connection_status );
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = iot_gateway_gatt_server_request_handler( &p_data->attribute_request );
            break;

        default:
            break;
    }

    return result;
}
