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

#include <stdlib.h>
#include <string.h>
#include "wiced.h"
#include "simple_http_server.h"
#include "wwd_constants.h"
#include <wiced_utilities.h>
#include <resources.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SSID_FIELD_NAME            "ssid"
#define SECURITY_FIELD_NAME        "at0"
#define CHANNEL_FIELD_NAME         "chan"
#define BSSID_FIELD_NAME           "bssid"
#define PASSPHRASE_FIELD_NAME      "ap0"
#define PIN_FIELD_NAME             "pin"

#ifdef USE_HTTPS
#define CONNECT_PROTOCOL           "https\n"
#else
#define CONNECT_PROTOCOL           "http\n"
#endif

#define APP_SCRIPT_PT1     "var elem_num = "
#define APP_SCRIPT_PT2     ";\n var labelname = \""
#define APP_SCRIPT_PT3     "\";\n var fieldname  = \"v"
#define APP_SCRIPT_PT4     "\";\n var fieldvalue = \""
#define APP_SCRIPT_PT5     "\";\n"


#define SCAN_SCRIPT_PT1    "var elem_num = "
#define SCAN_SCRIPT_PT2    ";\n var SSID = \""
#define SCAN_SCRIPT_PT3    "\";\n var RSSIstr  = \""
#define SCAN_SCRIPT_PT4    "\";\n var SEC = "
#define SCAN_SCRIPT_PT5    ";\n var CH  = "
#define SCAN_SCRIPT_PT6    ";\n var BSSID  = \""
#define SCAN_SCRIPT_PT7    "\";\n"

/* Signal strength defines (in dBm) */
#define RSSI_VERY_POOR             -85
#define RSSI_POOR                  -70
#define RSSI_GOOD                  -55
#define RSSI_VERY_GOOD             -40
#define RSSI_EXCELLENT             -25
#define RSSI_VERY_POOR_STR         "Very Poor"
#define RSSI_POOR_STR              "Poor"
#define RSSI_GOOD_STR              "Good"
#define RSSI_VERY_GOOD_STR         "Very good"
#define RSSI_EXCELLENT_STR         "Excellent"

#define CAPTIVE_PORTAL_REDIRECT_PAGE \
    "<html><head>" \
    "<meta http-equiv=\"refresh\" content=\"0; url=/config/device_settings.html\">" \
    "</head></html>"

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct
{
    wiced_tcp_stream_t* stream;
    wiced_semaphore_t           semaphore;
    uint32_t                    result_count;
} process_scan_data_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static int32_t        process_app_settings_page ( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg );
static int32_t        process_wps_go            ( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg );
static int32_t        process_scan              ( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg );
static int32_t        process_connect           ( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg );
static int32_t        process_config_save       ( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg );
static wiced_result_t scan_handler              ( wiced_scan_handler_result_t* malloced_scan_result );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/**
 * URL Handler List
 */
START_OF_HTTP_PAGE_DATABASE(config_http_page_database)
    ROOT_HTTP_PAGE_REDIRECT("/config/device_settings.html"),
    { "/images/cypresslogo.png",         "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cypresslogo_png,        },
    { "/images/cypresslogo_line.png",    "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cypresslogo_line_png,   },
    { "/wpad.dat",                       "application/x-ns-proxy-autoconfig", WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_scripts_DIR_wpad_dat,              },
    { "/scan_results.txt",               "text/plain",                        WICED_DYNAMIC_URL_CONTENT,    .url_content.dynamic_data   = {process_scan,                  0 }           },
    { "/images/64_0bars.png",            "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_64_0bars_png,           },
    { "/images/64_1bars.png",            "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_64_1bars_png,           },
    { "/images/64_2bars.png",            "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_64_2bars_png,           },
    { "/images/64_3bars.png",            "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_64_3bars_png,           },
    { "/images/64_4bars.png",            "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_64_4bars_png,           },
    { "/images/64_5bars.png",            "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_64_5bars_png,           },
    { "/images/tick.png",                "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_tick_png,               },
    { "/images/cross.png",               "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cross_png,              },
    { "/images/lock.png",                "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_lock_png,               },
    { "/images/progress.gif",            "image/gif",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_progress_gif,           },
    { "/config/device_settings.html",    "text/html",                         WICED_DYNAMIC_URL_CONTENT,    .url_content.dynamic_data   = {process_app_settings_page,     0 } },
    { "/config/scan_page_outer.html",    "text/html",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_config_DIR_scan_page_outer_html,   },
    { "/scripts/general_ajax_script.js", "application/javascript",            WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_scripts_DIR_general_ajax_script_js,},
    { "/images/wps_icon.png",            "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_wps_icon_png,           },
    { "/images/scan_icon.png",           "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_scan_icon_png,          },
    { "/images/favicon.ico",             "image/vnd.microsoft.icon",          WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_favicon_ico,            },
    { "/images/cypresslogo.png",         "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cypresslogo_png,        },
    { "/images/cypresslogo_line.png",    "image/png",                         WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_images_DIR_cypresslogo_line_png,   },
    { "/styles/buttons.css",             "text/css",                          WICED_RESOURCE_URL_CONTENT,   .url_content.resource_data  = &resources_styles_DIR_buttons_css,            },
    { "/connect",                        "text/html",                         WICED_DYNAMIC_URL_CONTENT,    .url_content.dynamic_data   = {process_connect,               0 }          },
    { "/wps_go",                         "text/html",                         WICED_DYNAMIC_URL_CONTENT,    .url_content.dynamic_data   = {process_wps_go,                0 }          },
    { "/config_save",                    "text/html",                         WICED_DYNAMIC_URL_CONTENT,    .url_content.dynamic_data   = {process_config_save,           0 }          },
    { IOS_CAPTIVE_PORTAL_ADDRESS,        "text/html",                         WICED_STATIC_URL_CONTENT,     .url_content.static_data  = {CAPTIVE_PORTAL_REDIRECT_PAGE, sizeof(CAPTIVE_PORTAL_REDIRECT_PAGE) } },
    /* Add more pages here */
END_OF_HTTP_PAGE_DATABASE();

extern const configuration_entry_t* app_configuration;
extern wiced_simple_http_server_t*  http_server;
extern wiced_bool_t                 config_use_wps;
extern char                         config_wps_pin[9];

/******************************************************
 *               Function Definitions
 ******************************************************/

int32_t process_app_settings_page( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg )
{
    const configuration_entry_t* config_entry;
    char                  temp_buf[11];
    const resource_hnd_t* end_str_res;
    uint8_t               string_size;
    char                  config_count[2] = {'0','0'};

    UNUSED_PARAMETER( url_parameters );
    UNUSED_PARAMETER( arg );

    wiced_tcp_stream_write_resource( stream, &resources_config_DIR_device_settings_html );

    /* Write the app configuration table */
    if( app_configuration != NULL )
    {
        for (config_entry = app_configuration; config_entry->name != NULL; ++config_entry)
        {

            /* Write the table entry start html direct from resource file */
            switch (config_entry->data_type)
            {
                case CONFIG_STRING_DATA:
                wiced_tcp_stream_write_resource( stream, &resources_config_DIR_device_settings_html_dev_settings_str );
                    break;
                case CONFIG_UINT8_DATA:
                case CONFIG_UINT16_DATA:
                case CONFIG_UINT32_DATA:
                    wiced_tcp_stream_write_resource( stream, &resources_config_DIR_device_settings_html_dev_settings_int );
                    break;
                default:
                    wiced_tcp_stream_write(stream, "error", 5);
                    break;
            }

            /* Output javascript to fill the table entry */

            wiced_tcp_stream_write( stream, APP_SCRIPT_PT1, sizeof(APP_SCRIPT_PT1)-1 );
            wiced_tcp_stream_write( stream, config_count, 2 );
            wiced_tcp_stream_write( stream, APP_SCRIPT_PT2, sizeof(APP_SCRIPT_PT2)-1 );
            wiced_tcp_stream_write( stream, config_entry->name, (uint16_t) strlen( config_entry->name ) );
            wiced_tcp_stream_write( stream, APP_SCRIPT_PT3, sizeof(APP_SCRIPT_PT3)-1 );
            wiced_tcp_stream_write( stream, config_count, 2 );
            wiced_tcp_stream_write( stream, APP_SCRIPT_PT4, sizeof(APP_SCRIPT_PT4)-1 );

            /* Fill in current value */
            switch (config_entry->data_type)
            {
                case CONFIG_STRING_DATA:
                    {
                        char* str_ptr = NULL;
                        wiced_dct_read_lock( (void**)&str_ptr, WICED_FALSE, DCT_APP_SECTION, config_entry->dct_offset, config_entry->data_size );
                        wiced_tcp_stream_write(stream, str_ptr, (uint16_t) strlen( str_ptr ) );
                        wiced_dct_read_unlock( str_ptr, WICED_FALSE );
                        end_str_res = &resources_config_DIR_device_settings_html_dev_settings_str_end;
                    }
                    break;
                case CONFIG_UINT8_DATA:
                    {
                        uint8_t * data;
                        wiced_dct_read_lock( (void**)&data, WICED_FALSE, DCT_APP_SECTION, config_entry->dct_offset, config_entry->data_size );
                        memset(temp_buf, ' ', 3);
                        string_size = unsigned_to_decimal_string(*data, (char*)temp_buf, 0, 3);
                        wiced_dct_read_unlock( data, WICED_FALSE );
                        wiced_tcp_stream_write(stream, temp_buf, (uint16_t) string_size);
                        end_str_res = &resources_config_DIR_device_settings_html_dev_settings_int_end;
                    }
                    break;
                case CONFIG_UINT16_DATA:
                    {
                        uint16_t * data;
                        wiced_dct_read_lock( (void**)&data, WICED_FALSE, DCT_APP_SECTION, config_entry->dct_offset, config_entry->data_size );
                        memset(temp_buf, ' ', 5);
                        string_size = unsigned_to_decimal_string(*data, (char*)temp_buf, 0, 5);
                        wiced_dct_read_unlock( data, WICED_FALSE );
                        wiced_tcp_stream_write(stream, temp_buf, (uint16_t) string_size);
                        end_str_res = &resources_config_DIR_device_settings_html_dev_settings_int_end;
                    }
                    break;
                case CONFIG_UINT32_DATA:
                    {
                        uint32_t * data;
                        wiced_dct_read_lock( (void**)&data, WICED_FALSE, DCT_APP_SECTION, config_entry->dct_offset, config_entry->data_size );
                        memset(temp_buf, ' ', 10);
                        string_size = unsigned_to_decimal_string(*data, (char*)temp_buf, 0, 10);
                        wiced_dct_read_unlock( data, WICED_FALSE );
                        wiced_tcp_stream_write(stream, temp_buf, (uint16_t) string_size);
                        end_str_res = &resources_config_DIR_device_settings_html_dev_settings_int_end;
                    }
                    break;
                default:
                    wiced_tcp_stream_write(stream, "error", 5);
                    end_str_res = NULL;
                    break;
            }

            wiced_tcp_stream_write(stream, APP_SCRIPT_PT5, sizeof(APP_SCRIPT_PT5)-1);
            wiced_tcp_stream_write_resource(stream, end_str_res);


            if (config_count[1] == '9')
            {
                ++config_count[0];
                config_count[1] = '0';
            }
            else
            {
                ++config_count[1];
            }
        }
    }
    wiced_tcp_stream_write_resource( stream, &resources_config_DIR_device_settings_html_dev_settings_bottom );

    return 0;
}


static wiced_result_t scan_handler( wiced_scan_handler_result_t* malloced_scan_result )
{
    process_scan_data_t* scan_data = (process_scan_data_t*)malloced_scan_result->user_data;

    malloc_transfer_to_curr_thread( malloced_scan_result );

    /* Check if scan is not finished */
    if ( malloced_scan_result->status == WICED_SCAN_INCOMPLETE )
    {
        char temp_buffer[70];
        char* temp_ptr;
        const char* const_temp_ptr;
        uint16_t temp_length;
        int i;

        wiced_tcp_stream_t* stream = scan_data->stream;

        /* Result ID */
        temp_length = unsigned_to_decimal_string( scan_data->result_count, temp_buffer, 1, 10 );
        temp_buffer[temp_length++] = '\n';
        scan_data->result_count++;
        wiced_tcp_stream_write( stream, temp_buffer, temp_length );

        /* SSID */
        temp_ptr = temp_buffer;
        for( i = 0; i < malloced_scan_result->ap_details.SSID.length; i++)
        {
            temp_ptr = string_append_two_digit_hex_byte( temp_ptr, malloced_scan_result->ap_details.SSID.value[i] );
        }
        *temp_ptr = '\n';
        temp_ptr++;

        wiced_tcp_stream_write(stream, temp_buffer, (uint32_t)( temp_ptr - temp_buffer ) );

        /* Security */
        temp_length = unsigned_to_decimal_string( malloced_scan_result->ap_details.security, temp_buffer, 1, 10 );
        temp_buffer[temp_length++] = '\n';
        wiced_tcp_stream_write( stream, temp_buffer, temp_length );

        const_temp_ptr = (   malloced_scan_result->ap_details.security == WICED_SECURITY_OPEN )? "OPEN\n" :
                         ( ( malloced_scan_result->ap_details.security & WEP_ENABLED   ) != 0 )? "WEP\n"  :
                         ( ( malloced_scan_result->ap_details.security & WPA_SECURITY  ) != 0 )? "WPA\n"  :
                         ( ( malloced_scan_result->ap_details.security & WPA2_SECURITY ) != 0 )? "WPA2\n" : "UNKNOWN\n";
        wiced_tcp_stream_write(stream, const_temp_ptr, strlen(const_temp_ptr) );

        /* RSSI */
        temp_length = signed_to_decimal_string( malloced_scan_result->ap_details.signal_strength, temp_buffer, 1, 11 );
        temp_buffer[temp_length++] = '\n';
        wiced_tcp_stream_write( stream, temp_buffer, temp_length );

        /* Channel */
        temp_length = unsigned_to_decimal_string( malloced_scan_result->ap_details.channel, temp_buffer, 1, 10 );
        temp_buffer[temp_length++] = '\n';
        wiced_tcp_stream_write( stream, temp_buffer, temp_length );

        /* BSSID */
        temp_ptr = temp_buffer;
        temp_ptr = string_append_two_digit_hex_byte( temp_ptr, malloced_scan_result->ap_details.BSSID.octet[0] );
        temp_ptr = string_append_two_digit_hex_byte( temp_ptr, malloced_scan_result->ap_details.BSSID.octet[1] );
        temp_ptr = string_append_two_digit_hex_byte( temp_ptr, malloced_scan_result->ap_details.BSSID.octet[2] );
        temp_ptr = string_append_two_digit_hex_byte( temp_ptr, malloced_scan_result->ap_details.BSSID.octet[3] );
        temp_ptr = string_append_two_digit_hex_byte( temp_ptr, malloced_scan_result->ap_details.BSSID.octet[4] );
        temp_ptr = string_append_two_digit_hex_byte( temp_ptr, malloced_scan_result->ap_details.BSSID.octet[5] );
        *temp_ptr = '\n';
        temp_ptr++;
        wiced_tcp_stream_write( stream, temp_buffer, (uint32_t)(temp_ptr - temp_buffer) );

        /* Remembered */
        temp_length = unsigned_to_decimal_string( 0, temp_buffer, 1, 10 );  /* TODO: add support for remembered APs */
        temp_buffer[temp_length++] = '\n';
        wiced_tcp_stream_write( stream, temp_buffer, temp_length );
    }
    else
    {
        wiced_rtos_set_semaphore( &scan_data->semaphore );
    }

    free(malloced_scan_result);

    return WICED_SUCCESS;
}


static int32_t process_scan( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg )
{
    process_scan_data_t scan_data;

    UNUSED_PARAMETER( url_parameters );
    UNUSED_PARAMETER( arg );

    scan_data.stream = stream;
    scan_data.result_count = 0;

    /* Initialise the semaphore that will tell us when the scan is complete */
    wiced_rtos_init_semaphore(&scan_data.semaphore);

    wiced_tcp_stream_write( stream, CONNECT_PROTOCOL, sizeof( CONNECT_PROTOCOL ) - 1 );

    /* Start the scan */
    wiced_wifi_scan_networks( scan_handler, &scan_data );

    /* Wait until scan is complete */
    wiced_rtos_get_semaphore(&scan_data.semaphore, WICED_WAIT_FOREVER);

    /* Clean up */
    wiced_rtos_deinit_semaphore(&scan_data.semaphore);

    return 0;
}


static int32_t process_wps_go( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg )
{
    unsigned int url_parameters_len;

    UNUSED_PARAMETER( stream );
    UNUSED_PARAMETER( arg );

    url_parameters_len = strlen(url_parameters);

    /* client has signalled to start client mode via WPS. */
    config_use_wps = WICED_TRUE;

    /* Check if config method is PIN */
    if ( ( strlen( PIN_FIELD_NAME ) + 1 < url_parameters_len ) &&
         ( 0 == strncmp( url_parameters, PIN_FIELD_NAME "=", strlen( PIN_FIELD_NAME ) + 1 ) ) )
    {
        unsigned int pinlen = 0;

        url_parameters += strlen( PIN_FIELD_NAME ) + 1;

        /* Find length of pin */
        while ( ( url_parameters[pinlen] != '&'    ) &&
                ( url_parameters[pinlen] != '\n'   ) &&
                ( url_parameters[pinlen] != '\x00' ) &&
                ( url_parameters_len > 0 ) )
        {
            pinlen++;
            url_parameters_len--;
        }
        memcpy( config_wps_pin, url_parameters, pinlen );
        config_wps_pin[pinlen] = '\x00';
    }
    else
    {
        config_wps_pin[0] = '\x00';
    }

    /* Config has been set. Turn off HTTP server */
    wiced_simple_http_server_stop( http_server );
    return 1;
}


/**
 * URL handler for signaling web server shutdown
 *
 * The reception of this web server request indicates that the client wants to
 * start the appliance, after shutting down the access point, DHCP server and web server
 * Decodes the URL parameters into the connection configuration buffer, then signals
 * for the web server to shut down
 *
 * @param  socket  : a handle for the TCP socket over which the data will be sent
 * @param  url_parameters     : a byte array containing any parameters included in the URL
 * @param  url_parameters_len : size of the url_parameters byte array in bytes
 */
static int32_t process_connect( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg )
{
    /* This is the first part of the platform_dct_wifi_config_t structure */
    struct
    {
        wiced_bool_t             device_configured;
        wiced_config_ap_entry_t  ap_entry;
    } temp_config;

    UNUSED_PARAMETER( stream );
    UNUSED_PARAMETER( arg );

    memset( &temp_config, 0, sizeof(temp_config) );

    /* First, parse AP details */
    while (url_parameters[0] == 'a' && url_parameters[3] == '=')
    {
        uint8_t ap_index;
        const char* end_of_value;

        /* Extract the AP index and check validity */
        ap_index = (uint8_t)( url_parameters[2] - '0' );
        if (ap_index >= CONFIG_AP_LIST_SIZE)
        {
            return -1;
        }

        /* Find the end of the value */
        end_of_value = &url_parameters[4];
        while( (*end_of_value != '&') && (*end_of_value != '\x00') && (*end_of_value != '\n') )
        {
            ++end_of_value;
        }

        /* Parse either the SSID or PSK*/
        if ( url_parameters[1] == 's' )
        {
            memcpy( temp_config.ap_entry.details.SSID.value, &url_parameters[4], (size_t) ( end_of_value - &url_parameters[4] ) );
            temp_config.ap_entry.details.SSID.length = (uint8_t) ( end_of_value - &url_parameters[4] );
            temp_config.ap_entry.details.SSID.value[temp_config.ap_entry.details.SSID.length] = 0;
        }
        else if (url_parameters[1] == 'p')
        {
            temp_config.ap_entry.security_key_length = (uint8_t) ( end_of_value - &url_parameters[4] );
            memcpy( temp_config.ap_entry.security_key, &url_parameters[4], temp_config.ap_entry.security_key_length);
            temp_config.ap_entry.security_key[temp_config.ap_entry.security_key_length] = 0;
        }
        else if (url_parameters[1] == 't')
        {
            temp_config.ap_entry.details.security = (wiced_security_t) atoi( &url_parameters[4] );
        }
        else
        {
            return -1;
        }
        url_parameters = end_of_value + 1;
    }

    /* Save updated config details */
    temp_config.device_configured = WICED_TRUE;
    wiced_dct_write( &temp_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof(temp_config) );

    /* Config has been set. Turn off HTTP server */
    wiced_simple_http_server_stop( http_server );
    return 0;
}

static int32_t process_config_save( const char* url_parameters, wiced_tcp_stream_t* stream, void* arg )
{
    UNUSED_PARAMETER( arg );

    if ( app_configuration != NULL )
    {
        uint32_t earliest_offset = 0xFFFFFFFF;
        uint32_t end_of_last_offset = 0x0;
        const configuration_entry_t* config_entry;
        uint8_t* app_dct;
        wiced_result_t result;

        /* Calculate how big the app config details are */
        for ( config_entry = app_configuration; config_entry->name != NULL; ++config_entry )
        {
            if ( config_entry->dct_offset < earliest_offset )
            {
                earliest_offset = config_entry->dct_offset;
            }
            if ( config_entry->dct_offset + config_entry->data_size > end_of_last_offset )
            {
                end_of_last_offset = config_entry->dct_offset + config_entry->data_size;
            }
        }

        if ( end_of_last_offset <= earliest_offset )
        {
            wiced_assert( "Invalid app configuration data", 1 == 0 );
            goto save_failed;
        }

        result = wiced_dct_read_lock( (void**)&app_dct, WICED_TRUE, DCT_APP_SECTION, earliest_offset, end_of_last_offset - earliest_offset );
        if ( result != WICED_SUCCESS )
        {
            wiced_assert( "DCT read failed", 1 == 0 );
            goto save_failed;
        }
        if ( app_dct != NULL )
        {
            while ( url_parameters[0] == 'v' && url_parameters[3] == '=' )
            {
                /* Extract the variable index and check validity */
                uint16_t variable_index = (uint16_t) ( ( ( url_parameters[1] - '0' ) << 8 ) | ( url_parameters[2] - '0' ) );

                /* Find the end of the value */
                const char* end_of_value = &url_parameters[4];
                while ( ( *end_of_value != '&' ) && ( *end_of_value != '\n' ) )
                {
                    ++end_of_value;
                }

                /* Parse param */
                config_entry = &app_configuration[variable_index];
                switch ( config_entry->data_type )
                {
                    case CONFIG_STRING_DATA:
                        memcpy( (uint8_t*) ( app_dct + config_entry->dct_offset ), &url_parameters[4], (size_t) ( end_of_value - &url_parameters[4] ) );
                        ( (uint8_t*) ( app_dct + config_entry->dct_offset ) )[end_of_value - &url_parameters[4]] = 0;
                        break;
                    case CONFIG_UINT8_DATA:
                        *(uint8_t*) ( app_dct + config_entry->dct_offset - earliest_offset ) = (uint8_t) atoi( &url_parameters[4] );
                        break;
                    case CONFIG_UINT16_DATA:
                        *(uint16_t*) ( app_dct + config_entry->dct_offset - earliest_offset ) = (uint16_t) atoi( &url_parameters[4] );
                        break;
                    case CONFIG_UINT32_DATA:
                        *(uint32_t*) ( app_dct + config_entry->dct_offset - earliest_offset ) = (uint32_t) atoi( &url_parameters[4] );
                        break;
                    default:
                        break;
                }

                url_parameters = end_of_value + 1;
            }

            /* Write the app DCT */
            result = wiced_dct_write( app_dct, DCT_APP_SECTION, earliest_offset, end_of_last_offset - earliest_offset );
            if ( result != WICED_SUCCESS )
            {
                wiced_assert( "DCT read failed", 1 == 0 );
                goto save_failed;
            }

            wiced_dct_read_unlock( app_dct, WICED_TRUE );
        }
    }

    #define CONFIG_SAVE_SUCCESS  "Config saved"
    wiced_tcp_stream_write(stream, CONFIG_SAVE_SUCCESS, sizeof(CONFIG_SAVE_SUCCESS)-1);

    return 0;

save_failed:
    #define CONFIG_SAVE_FAIL     "Config save failed"
    wiced_assert( "Save Failed", 1 == 0 );
    wiced_tcp_stream_write(stream, CONFIG_SAVE_FAIL, sizeof(CONFIG_SAVE_FAIL)-1);

    return 0;
}

