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
 * HTTP Server-Sent Event (SSE) Example Application
 *
 * This application snippet demonstrates how to send events from a HTTP server back to its clients.
 *
 * Features demonstrated
 *  - HTTP Server-Sent Event (SSE)
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  3. Plug the WICED eval board into your computer
 *  4. Open a terminal application and connect to the WICED eval board
 *  5. Build and download the application (to the WICED board)
 *
 * After the download completes, the terminal displays WICED startup
 * information and then :
 *  - Joins a Wi-Fi network and pings the gateway. Ping results are
 *    printed to the UART and appear on the terminal
 *  - Starts a softAP and a webserver on the AP interface
 *
 * Application Operation
 * This section provides a description of the application flow and usage.
 * The app runs in a thread, the entry point is application_start()
 *
 *    Startup
 *      - Initialise the device
 *      - Start the network interface to connect the device to the network
 *      - Set the local time from a time server on the internet
 *      - Setup a timer to get current time
 *      - Start a webserver to display timestamps
 *
 *    Usage *
 *        The current time is published to a webpage on the client by a local webserver using server-sent events (SSE).
 *        The webpage uses an EventSource object to receive the SSE notifications.
 *
 */

#include "wiced.h"
#include "http_server.h"
#include "resources.h"
#include "sntp.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SERVER_SENT_EVENT_INTERVAL_MS  ( 1 * SECONDS )
#define MAX_SOCKETS                    ( 5 )

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
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t send_event  ( void* arg );
static int32_t        process_page( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_http_server_t http_server;

START_OF_HTTP_PAGE_DATABASE(web_pages)
    ROOT_HTTP_PAGE_REDIRECT("/main.html"),
    { "/main.html",                     "text/html",                WICED_RESOURCE_URL_CONTENT,    .url_content.resource_data = &resources_apps_DIR_http_server_sent_events_DIR_main_html   },
    { "/images/favicon.ico",            "image/vnd.microsoft.icon", WICED_RESOURCE_URL_CONTENT,    .url_content.resource_data = &resources_images_DIR_favicon_ico,                          },
    { "/images/cypresslogo.png",        "image/png",                WICED_RESOURCE_URL_CONTENT,    .url_content.resource_data = &resources_images_DIR_cypresslogo_png,                      },
    { "/images/cypresslogo_line.png",   "image/png",                WICED_RESOURCE_URL_CONTENT,    .url_content.resource_data = &resources_images_DIR_cypresslogo_line_png,                 },
    { "/events",                        "text/event-stream",        WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data  = { process_page, 0 },                                        },

END_OF_HTTP_PAGE_DATABASE();

static wiced_http_response_stream_t* http_event_stream = NULL;
static wiced_timed_event_t           timed_event;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    /* Initialise the device */
    wiced_init();

    /* Bring up the STA (client) interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    /* Timestamp is needed for server-sent event (SSE) data.
     * Start automatic time synchronisation and synchronise once every day.
     */
    sntp_start_auto_time_sync( 1 * DAYS );

    wiced_rtos_register_timed_event( &timed_event, WICED_NETWORKING_WORKER_THREAD, send_event, SERVER_SENT_EVENT_INTERVAL_MS, NULL );

    /* Start a web server on the STA interface */
    wiced_http_server_start( &http_server, 80, MAX_SOCKETS, web_pages, WICED_STA_INTERFACE, DEFAULT_URL_PROCESSOR_STACK_SIZE );
}

static int32_t process_page( const char* url_path, const char* url_parameters, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    /* Grab HTTP response stream. This will be used by the application to send events back to the client */
    http_event_stream = stream;
    wiced_http_response_stream_enable_chunked_transfer( http_event_stream );
    wiced_http_response_stream_write_header( http_event_stream, HTTP_200_TYPE, CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_TEXT_EVENT_STREAM );
    return 0;
}

static wiced_result_t send_event( void* arg )
{
    wiced_iso8601_time_t current_time;

    UNUSED_PARAMETER( arg );

    if ( http_event_stream == NULL )
    {
        return WICED_ERROR;
    }

    /* SSE is prefixed with "data: " */
    WICED_VERIFY( wiced_http_response_stream_write( http_event_stream, (const void*)EVENT_STREAM_DATA, sizeof( EVENT_STREAM_DATA ) - 1 ) );

    /* Send current time back to the client */
    wiced_time_get_iso8601_time( &current_time );
    WICED_VERIFY( wiced_http_response_stream_write( http_event_stream, (const void*)&current_time, sizeof( current_time ) ) );

    /* SSE is ended with two line feeds */
    WICED_VERIFY( wiced_http_response_stream_write( http_event_stream, (const void*)LFLF, sizeof( LFLF ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_flush( http_event_stream ) );
    return WICED_SUCCESS;
}

