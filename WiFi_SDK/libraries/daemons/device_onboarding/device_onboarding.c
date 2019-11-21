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
 * This file implements the APIs for Wi-Fi Device onboarding.
 *
 */

#include "wiced.h"
#include "http_server.h"
#include "wiced_resource.h"
#include "resources.h"
#include "wiced_tls.h"
#include "dns_redirect.h"
#include "JSON.h"
#include "device_onboarding.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define WIFI_ONBOARDING_THREAD_STACK_SIZE           (4000)
#define WIFI_ONBOARDING_WORKER_THREAD_PRIORITY      (WICED_APPLICATION_PRIORITY)
#define WIFI_ONBOARDING_WORKER_THREAD_QUEUE_SIZE    (10)

#define WIFI_ONBOARDING_LOG(fmt, ...)               printf( ("[Onboarding] " fmt "\r\n"), ##__VA_ARGS__)

#define SSID_NAME_KEY                               "ssid_name"
#define SSID_PASSPHRASE_KEY                         "ssid_passphrase"
#define FIND_AP_MAXIMUM_RETRY_COUNT                 (5)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    ONBOARDING_NOT_STARTED,
    ONBOARDING_SOFTAP_STARTED,
    ONBOARDING_SOFTAP_ONBOARDER_CONNECTED,
    ONBOARDING_SSID_NOT_FOUND,
    ONBOARDING_STA_STARTED,
    ONBOARDING_STA_CONNECTION_FAILED,
    ONBOARDING_STA_CONNECTED,
    ONBOARDED_SOFTAP_STOPPED,
} onboardee_device_state_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint32_t    event_type;
    wiced_mac_t mac;
    uint16_t    padding; /* Message size must be on a 4 byte boundary */
} ap_event_message_t;

typedef struct wifi_onboarding_context
{
    wiced_http_server_t                 https_server;
    wiced_tls_identity_t                tls_identity;
    onboardee_device_state_t            state;
    int                                 max_sockets;
    wiced_mutex_t                       lock;

    wiced_ssid_t                        onboarder_ssid;
    uint8_t                             onboarder_security_key_length;
    char                                onboarder_security_key[ SECURITY_KEY_SIZE ];
    dns_redirector_t                    dns_redirector;
    wiced_worker_thread_t               helper_thread;
    wiced_wifi_onboarding_callback_t    app_callback;
    wiced_http_request_type_t           cached_request;
    ap_event_message_t                  ap_event;

} wifi_onboarding_context_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static int32_t process_get_sta_network_config (const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream,
                                                        void* arg, wiced_http_message_body_t* http_message_body);

static int32_t process_onboarding_completion_status (const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream,
                                                        void* arg, wiced_http_message_body_t* http_message_body);

static int32_t process_check_onboarding_status (const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream,
                                                        void* arg, wiced_http_message_body_t* http_message_body);

static wiced_result_t initiate_sta_mode( void* args );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wifi_onboarding_context_t context =
{
    .max_sockets                = 4,
    .state                      = ONBOARDING_NOT_STARTED,
    .cached_request             = REQUEST_UNDEFINED,
    .app_callback               = NULL,
};

START_OF_HTTP_PAGE_DATABASE(onboarding_web_pages)
    { "/networkconfig",         "application/json",         WICED_RAW_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = { process_get_sta_network_config, 0 }, },
    { "/config-status",         "text/html",                WICED_RAW_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = { process_check_onboarding_status, 0 }, },
    { "/confirm-onboarding",    "text/html",                WICED_RAW_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = { process_onboarding_completion_status, 0 }, },
END_OF_HTTP_PAGE_DATABASE();

/******************************************************
 *               Function Definitions
 ******************************************************/

static uint8_t get_onboardee_state(void)
{
    uint8_t onboardee_state;

    wiced_rtos_lock_mutex(&context.lock);
    onboardee_state = (uint8_t)context.state;
    wiced_rtos_unlock_mutex(&context.lock);

    return onboardee_state;
}

static void set_onboardee_state( onboardee_device_state_t onboardee_state )
{
    wiced_rtos_lock_mutex(&context.lock);
    context.state = onboardee_state;
    wiced_rtos_unlock_mutex(&context.lock);

    return;
}

static wiced_result_t ssid_credential_json_parser_cb( wiced_json_object_t* ssid_object )
{
    if( strncmp( ssid_object->object_string, SSID_NAME_KEY, strlen(SSID_NAME_KEY) ) == 0 )
    {
        if( ssid_object->value_type == JSON_STRING_TYPE )
        {
            memcpy(context.onboarder_ssid.value, ssid_object->value, ssid_object->value_length > SSID_NAME_SIZE ? SSID_NAME_SIZE : ssid_object->value_length );
            context.onboarder_ssid.length = (uint8_t)ssid_object->value_length;

            WIFI_ONBOARDING_LOG ("Extracted SSID name: %s (len:%d)", context.onboarder_ssid.value, context.onboarder_ssid.length );
        }
    }
    else if( strncmp( ssid_object->object_string, SSID_PASSPHRASE_KEY, strlen(SSID_PASSPHRASE_KEY) ) == 0 )
    {
        if( ssid_object->value_type == JSON_STRING_TYPE )
        {
            memcpy(context.onboarder_security_key, ssid_object->value, ssid_object->value_length > SECURITY_KEY_SIZE ? SECURITY_KEY_SIZE : ssid_object->value_length );
            context.onboarder_security_key_length =  (uint8_t)ssid_object->value_length;
            WIFI_ONBOARDING_LOG ("Extracted SSID passphrase: %s (len:%d)", context.onboarder_security_key, context.onboarder_security_key_length );
        }
    }
    return WICED_SUCCESS;
}

static wiced_result_t handle_softap_event( void *args )
{
    if ( context.ap_event.event_type == WICED_AP_STA_JOINED_EVENT )
    {
        WPRINT_APP_INFO( ("STA joined: ") );
        print_mac_address( &context.ap_event.mac );
        WPRINT_APP_INFO( ("\n") );
    }
    else if ( context.ap_event.event_type == WICED_AP_STA_LEAVE_EVENT )
    {
        WPRINT_APP_INFO( ("STA left: ") );
        print_mac_address( &context.ap_event.mac );
        WPRINT_APP_INFO( ("\n") );
    }
    return WICED_SUCCESS;
}

static void onboarding_softap_client_callback( wiced_wifi_softap_event_t event, const wiced_mac_t* mac_address )
{
    context.ap_event.event_type = event;
    memcpy( &context.ap_event.mac, mac_address, sizeof(wiced_mac_t));

    wiced_rtos_send_asynchronous_event(&context.helper_thread, handle_softap_event, NULL );
}

wiced_result_t wiced_wifi_device_onboarding_start( const wiced_ip_setting_t* ap_ip_settings, wiced_wifi_onboarding_callback_t callback )
{
    wiced_result_t result = WICED_ERROR;
    platform_dct_security_t* dct_security = NULL;

    if( !ap_ip_settings )
    {
        WIFI_ONBOARDING_LOG( "Bad arguments. Failed to start onboarding." );
        return WICED_BADARG;
    }

    /* Bring-up the device in AP mode first */
    result = wiced_network_up(WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, ap_ip_settings);
    if( result != WICED_SUCCESS )
    {
        return result;
    }

    context.app_callback = callback;

    wiced_wifi_register_softap_event_handler(onboarding_softap_client_callback);

    /* Start a DNS redirect server to redirect wiced.com to the AP webserver database*/
    WICED_VERIFY( wiced_dns_redirector_start( &context.dns_redirector, WICED_AP_INTERFACE ) );

    /* Start a Worker thread to process time-consuming routines and hence unblocking http-server thread */
    WICED_VERIFY( wiced_rtos_create_worker_thread( &context.helper_thread, WIFI_ONBOARDING_WORKER_THREAD_PRIORITY, WIFI_ONBOARDING_THREAD_STACK_SIZE, WIFI_ONBOARDING_WORKER_THREAD_QUEUE_SIZE ) );

    WICED_VERIFY( wiced_rtos_init_mutex(&context.lock) );

    set_onboardee_state(ONBOARDING_SOFTAP_STARTED);

    /* Set-up TLS identity structure with some server certificate etc. */

    /* Lock the DCT to allow us to access the certificate and key */
    WIFI_ONBOARDING_LOG( "Read the certificate Key from DCT" );

    result = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
    if ( result != WICED_SUCCESS )
    {
        WIFI_ONBOARDING_LOG( "Unable to lock DCT to read certificate" );
        return result;
    }

        /* Setup TLS identity */
    result = wiced_tls_init_identity( &context.tls_identity, dct_security->private_key, strlen( dct_security->private_key ), (uint8_t*) dct_security->certificate, strlen( dct_security->certificate ) );
    if ( result != WICED_SUCCESS )
    {
        WIFI_ONBOARDING_LOG( "Unable to initialize TLS identity. Error = [%d]", result );
        return result;
    }

    /* Start a web server on the AP interface */
    wiced_https_server_start( &context.https_server, 443, context.max_sockets, onboarding_web_pages, &context.tls_identity,
                                WICED_AP_INTERFACE, DEFAULT_URL_PROCESSOR_STACK_SIZE );

    /* Finished accessing the certificates */
    result = wiced_dct_read_unlock( dct_security, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WIFI_ONBOARDING_LOG( "DCT Read Unlock Failed. Error = [%d]", result );
        return result;
    }

    WIFI_ONBOARDING_LOG( "SoftAP started successfully" );

    WICED_VERIFY( wiced_JSON_parser_register_callback(ssid_credential_json_parser_cb) );

    return WICED_SUCCESS;

}

static int32_t process_get_sta_network_config(const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body)
{
    wiced_result_t result = WICED_ERROR;
    http_status_codes_t response_status;

    if( context.state != ONBOARDING_SOFTAP_STARTED && context.state != ONBOARDING_STA_CONNECTION_FAILED )
    {
        response_status = HTTP_500_TYPE;
        goto exit;
    }

    if( (http_message_body->request_type == WICED_HTTP_POST_REQUEST) &&
        (http_message_body->data == NULL) && (http_message_body->total_message_data_remaining != 0) )
    {
        context.cached_request = WICED_HTTP_POST_REQUEST;
        return WICED_SUCCESS;
    }

    if( context.cached_request != REQUEST_UNDEFINED )
    {
        http_message_body->request_type = WICED_HTTP_POST_REQUEST;
    }

    if( http_message_body->request_type != WICED_HTTP_POST_REQUEST || http_message_body->data == NULL )
    {
        WIFI_ONBOARDING_LOG("Error request type or data invalid");
        response_status = HTTP_400_TYPE;
        goto exit;
    }

    WIFI_ONBOARDING_LOG( "/networkconfig URI post request. Data: %s", http_message_body->data );

    /* Parse the SSID credentials */
    result = wiced_JSON_parser( (const char *)http_message_body->data, http_message_body->message_data_length );

    if( result != WICED_SUCCESS )
    {
        response_status = HTTP_400_TYPE;
        goto exit;
    }

    set_onboardee_state(ONBOARDING_STA_STARTED);

    WIFI_ONBOARDING_LOG( "SSID credentials received...Initiating STA mode" );

    /* Offload the connection establishment to the worker thread - unblocks HTTP server thread to get further requests */
    wiced_rtos_send_asynchronous_event(&context.helper_thread, initiate_sta_mode, NULL );

    response_status = HTTP_200_TYPE;

exit:
    WICED_VERIFY( wiced_http_response_stream_disable_chunked_transfer( stream ) );

    WICED_VERIFY( wiced_http_response_stream_write_header( stream, response_status, 0, HTTP_CACHE_DISABLED, MIME_TYPE_ALL ) );

    WICED_VERIFY( wiced_http_response_stream_flush( stream ) );

    return result;
}


static wiced_result_t initiate_sta_mode( void* args )
{
    wiced_scan_result_t             ap_info;
    platform_dct_wifi_config_t*     dct_wifi_config;
    wiced_result_t                  result = WICED_ERROR;
    char ssid_value[SSID_NAME_SIZE + 1] = { 0 };
    int num_retries = 0;

    UNUSED_PARAMETER( args );

    if ( ( wwd_wifi_is_ready_to_transceive( 0 ) == WWD_SUCCESS ) )
    {
        WIFI_ONBOARDING_LOG( "STA already initialized?? Closing it." );
        result = wiced_network_down( WICED_STA_INTERFACE );
        if( result != WICED_SUCCESS )
        {
            WIFI_ONBOARDING_LOG( "Huh! Failed to close STA interface. Bailing out..." );
            return result;
        }
    }

    /* Need below memcpy to search AP using wifi_find_ap API */
    memcpy(ssid_value, context.onboarder_ssid.value, context.onboarder_ssid.length);
    ssid_value[context.onboarder_ssid.length] = '\0';

    do
    {
        /* Delay it a while before next attempt */
        wiced_rtos_delay_milliseconds(100);
        WIFI_ONBOARDING_LOG(" Trying to fetch AP details(attempt# %d)", num_retries );
        result = wiced_wifi_find_ap( (char*)context.onboarder_ssid.value, &ap_info, NULL );
    } while( (num_retries++) < FIND_AP_MAXIMUM_RETRY_COUNT && result != WICED_SUCCESS );

    if( result != WICED_SUCCESS )
    {
        WIFI_ONBOARDING_LOG( "Can't fetch SSID: \"%s\" Security-type. Bailing out...", context.onboarder_ssid.value );
        set_onboardee_state(ONBOARDING_SSID_NOT_FOUND);
        return result;
    }

    /* Read config */
    wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );

    /* Modify config */
    dct_wifi_config->stored_ap_list[0].details.SSID.length = context.onboarder_ssid.length;

    memset( dct_wifi_config->stored_ap_list[0].details.SSID.value, 0, sizeof(dct_wifi_config->stored_ap_list[0].details.SSID.value) );

    memcpy( (char*)dct_wifi_config->stored_ap_list[0].details.SSID.value, context.onboarder_ssid.value, context.onboarder_ssid.length );

    dct_wifi_config->stored_ap_list[0].details.security = ap_info.security;

    /* Copy the BSSID */
    memcpy( dct_wifi_config->stored_ap_list[0].details.BSSID.octet, ap_info.BSSID.octet, sizeof(ap_info.BSSID.octet) );

    /* copy bss-type as it is used internally */
    dct_wifi_config->stored_ap_list[0].details.bss_type       = ap_info.bss_type;

    /* copy channel */
    dct_wifi_config->stored_ap_list[0].details.channel        = ap_info.channel;

    /* Save credentials for non-enterprise AP ; assuming non-enterprise AP here */
    memcpy((char*)dct_wifi_config->stored_ap_list[0].security_key, (char*)context.onboarder_security_key, context.onboarder_security_key_length);

    dct_wifi_config->stored_ap_list[0].security_key_length = context.onboarder_security_key_length;

    dct_wifi_config->device_configured = WICED_TRUE;
    /* Write config */
    wiced_dct_write( (const void*) dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );


    /* Initialize the device in STA mode */
    result = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);

    if ( result != WICED_SUCCESS )
    {
        /* Reset the configuration as failed to join */
        dct_wifi_config->device_configured                      = WICED_FALSE;
        dct_wifi_config->stored_ap_list[0].details.SSID.length  = 0;
        dct_wifi_config->stored_ap_list[0].details.security     = 0;
        dct_wifi_config->stored_ap_list[0].security_key_length  = 0;
        dct_wifi_config->stored_ap_list[0].details.bss_type     = 0;
        dct_wifi_config->stored_ap_list[0].details.channel      = 0;

        memset( dct_wifi_config->stored_ap_list[0].details.SSID.value, 0, sizeof(dct_wifi_config->stored_ap_list[0].details.SSID.value) );
        memset( dct_wifi_config->stored_ap_list[0].security_key, 0, sizeof(dct_wifi_config->stored_ap_list[0].security_key) );
        memset( dct_wifi_config->stored_ap_list[0].details.BSSID.octet, 0, sizeof(dct_wifi_config->stored_ap_list[0].details.BSSID.octet) );

        wiced_dct_write( (const void*) dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );

        wiced_dct_read_unlock( (void*) dct_wifi_config, WICED_TRUE );

        WIFI_ONBOARDING_LOG( "Failed to join network: \"%s\"", context.onboarder_ssid.value );

        set_onboardee_state(ONBOARDING_STA_CONNECTION_FAILED);

        return WICED_ERROR;
    }

    WIFI_ONBOARDING_LOG( "Started STA successfully" );

    set_onboardee_state(ONBOARDING_STA_CONNECTED);

    wiced_dct_read_unlock( (void*) dct_wifi_config, WICED_TRUE );

    return WICED_SUCCESS;
}

static int32_t process_check_onboarding_status(const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body)
{
    http_status_codes_t response_status = HTTP_200_TYPE;

    uint8_t state = get_onboardee_state();

    WIFI_ONBOARDING_LOG( "Onboardee state: %d", state );

    if( http_message_body->request_type != WICED_HTTP_GET_REQUEST )
    {
        response_status = HTTP_400_TYPE;
    }

    WICED_VERIFY( wiced_http_response_stream_disable_chunked_transfer( stream ) );

    if( response_status == HTTP_200_TYPE )
    {
        WICED_VERIFY( wiced_http_response_stream_write_header(stream, HTTP_200_TYPE, sizeof(state), HTTP_CACHE_DISABLED, MIME_TYPE_TEXT_PLAIN ) );
        WICED_VERIFY( wiced_http_response_stream_write( stream, &state, sizeof(state) ) );
    }
    else
    {
        WICED_VERIFY( wiced_http_response_stream_write_header( stream, response_status, 0, HTTP_CACHE_DISABLED, MIME_TYPE_TEXT_PLAIN ) );
    }

    WICED_VERIFY( wiced_http_response_stream_flush( stream ) );

    return WICED_SUCCESS;
}

static int32_t process_onboarding_completion_status(const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body)
{
    http_status_codes_t response_status = HTTP_200_TYPE;

    uint8_t state = get_onboardee_state();

    WIFI_ONBOARDING_LOG( "On-boarding status received from Onboarder..." );

    if( http_message_body->request_type != WICED_HTTP_PUT_REQUEST )
    {
        response_status = HTTP_400_TYPE;
    }
    /* If the client has sent 'confirm-onboarding' and server's state is not in-sync, then something went wrong */
    else if( state != ONBOARDING_STA_CONNECTED )
    {
        response_status = HTTP_500_TYPE;
    }
    else
    {
        response_status = HTTP_200_TYPE;
    }

    WICED_VERIFY( wiced_http_response_stream_disable_chunked_transfer( stream ) );
    WICED_VERIFY( wiced_http_response_stream_write_header(stream, response_status, 0, HTTP_CACHE_DISABLED, MIME_TYPE_ALL ) );
    WICED_VERIFY( wiced_http_response_stream_flush( stream ) );

    if( response_status == HTTP_200_TYPE )
    {
        WICED_VERIFY( wiced_http_response_stream_disconnect( stream ) );

        if( context.app_callback )
        {
            context.app_callback( WICED_SUCCESS );
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_wifi_device_onboarding_stop(void)
{
    wiced_result_t result;

    result = wiced_http_server_stop(&context.https_server);
    if( result != WICED_SUCCESS )
    {
        WIFI_ONBOARDING_LOG( "Error stopping HTTP Server @ AP-interface" );
        return result;
    }

#if 0
    result = wiced_network_down(WICED_AP_INTERFACE);
    if( result != WICED_SUCCESS )
    {
        WIFI_ONBOARDING_LOG( "Error stopping AP-interface" );
        return result;
    }
#endif

    WIFI_ONBOARDING_LOG( "SoftAP stopped successfully" );

    WICED_VERIFY( wiced_rtos_delete_worker_thread(&context.helper_thread) );

    WICED_VERIFY( wiced_dns_redirector_stop(&context.dns_redirector) );

    set_onboardee_state(ONBOARDING_NOT_STARTED);

    WICED_VERIFY( wiced_rtos_deinit_mutex(&context.lock) );

    return WICED_SUCCESS;
}
