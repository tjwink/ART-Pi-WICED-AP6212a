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

/**
 * @file
 *
 * API for the HTTP / HTTPS Web server
 *
 * Web pages and other resources are provided via an array which gets passed as an argument when starting the web server.
 * The array is constructed using the START_OF_HTTP_PAGE_DATABASE() and END_OF_HTTP_PAGE_DATABASE() macros, and optionally the ROOT_HTTP_PAGE_REDIRECT() macro
 * Below is an example of a list of web pages (taken from one of the demo apps)
 *
 * START_OF_HTTP_PAGE_DATABASE(web_pages)
 *     ROOT_HTTP_PAGE_REDIRECT("/apps/temp_control/main.html"),
 *     { "/apps/temp_control/main.html",    "text/html",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_apps_DIR_temp_control_DIR_main_html, },
 *     { "/temp_report.html",               "text/html",                WICED_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = {process_temperature_update, 0 }, },
 *     { "/temp_up",                        "text/html",                WICED_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = {process_temperature_up, 0 }, },
 *     { "/temp_down",                      "text/html",                WICED_DYNAMIC_URL_CONTENT,   .url_content.dynamic_data   = {process_temperature_down, 0 }, },
 *     { "/images/favicon.ico",             "image/vnd.microsoft.icon", WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_favicon_ico, },
 *     { "/scripts/general_ajax_script.js", "application/javascript",   WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_scripts_DIR_general_ajax_script_js, },
 *     { "/images/cypresslogo.png",         "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_png, },
 *     { "/images/cypressogo_line.png",     "image/png",                WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_images_DIR_cypresslogo_line_png, },
 *     { "/styles/buttons.css",             "text/css",                 WICED_RESOURCE_URL_CONTENT,  .url_content.resource_data  = &resources_styles_DIR_buttons_css, },
 * END_OF_HTTP_PAGE_DATABASE();
 */



#pragma once

#include "wiced_defaults.h"
#include "wiced_tcpip.h"
#include "wiced_rtos.h"
#include "wiced_resource.h"
#include "linked_list.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/**
 * Macro which creates the start of a web-page list.
 */
#define START_OF_HTTP_PAGE_DATABASE(name) \
    const wiced_http_page_t name[] = {

/**
 * Macro which creates an entry in the web-page list which will redirect requests for the '/' page to another URL.
 */
#define ROOT_HTTP_PAGE_REDIRECT(url) \
    { "/", "text/html", WICED_RAW_STATIC_URL_CONTENT, .url_content.static_data  = {url, sizeof(url)-1 } }

/**
 * Macro which creates the end part of a web-page list.
 */
#define END_OF_HTTP_PAGE_DATABASE() \
    {0,0,0, .url_content.static_data  = {NULL, 0 } } \
    }

#define EXPAND_AS_ENUMERATION(a,b)   a,
#define EXPAND_AS_MIME_TABLE(a,b)    b,

/******************************************************
 *                    Constants
 ******************************************************/

#define HTTP_404 \
    "HTTP/1.1 404 Not Found\r\n" \
    "Content-Type: text/html\r\n\r\n" \
    "<!doctype html>\n" \
    "<html><head><title>404 - WICED Web Server</title></head><body>\n" \
    "<h1>Address not found on WICED Web Server</h1>\n" \
    "<p><a href=\"/\">Return to home page</a></p>\n" \
    "</body>\n</html>\n"

#define MIME_TABLE( ENTRY ) \
    ENTRY( MIME_TYPE_TLV = 0 ,                "application/x-tlv8"               ) \
    ENTRY( MIME_TYPE_APPLE_BINARY_PLIST,      "application/x-apple-binary-plist" ) \
    ENTRY( MIME_TYPE_APPLE_PROXY_AUTOCONFIG,  "application/x-ns-proxy-autoconfig") \
    ENTRY( MIME_TYPE_BINARY_DATA,             "application/octet-stream"         ) \
    ENTRY( MIME_TYPE_JAVASCRIPT,              "application/javascript"           ) \
    ENTRY( MIME_TYPE_JSON,                    "application/json"                 ) \
    ENTRY( MIME_TYPE_HAP_JSON,                "application/hap+json"             ) \
    ENTRY( MIME_TYPE_HAP_PAIRING,             "application/pairing+tlv8"         ) \
    ENTRY( MIME_TYPE_HAP_VERIFY,              "application/hap+verify"           ) \
    ENTRY( MIME_TYPE_TEXT_HTML,               "text/html"                        ) \
    ENTRY( MIME_TYPE_TEXT_PLAIN,              "text/plain"                       ) \
    ENTRY( MIME_TYPE_TEXT_EVENT_STREAM,       "text/event-stream"                ) \
    ENTRY( MIME_TYPE_TEXT_CSS,                "text/css"                         ) \
    ENTRY( MIME_TYPE_IMAGE_PNG,               "image/png"                        ) \
    ENTRY( MIME_TYPE_IMAGE_GIF,               "image/gif"                        ) \
    ENTRY( MIME_TYPE_IMAGE_MICROSOFT,         "image/vnd.microsoft.icon"         ) \
    ENTRY( MIME_TYPE_ALL,                     "*/*"                              ) /* This must always be the last mimne*/

/**
 * A string with the address which iOS searches during the captive-portal part of a Wi-Fi attachment.
 */
#define IOS_CAPTIVE_PORTAL_ADDRESS        "/library/test/success.html"

#define DEFAULT_URL_PROCESSOR_STACK_SIZE  5000

#define NO_CONTENT_LENGTH                 0
#define CHUNKED_CONTENT_LENGTH            NO_CONTENT_LENGTH

#define HTTP_HEADER_200                   "HTTP/1.1 200 OK"
#define HTTP_HEADER_204                   "HTTP/1.1 204 No Content"
#define HTTP_HEADER_207                   "HTTP/1.1 207 Multi-Status"
#define HTTP_HEADER_301                   "HTTP/1.1 301"
#define HTTP_HEADER_400                   "HTTP/1.1 400 Bad Request"
#define HTTP_HEADER_403                   "HTTP/1.1 403"
#define HTTP_HEADER_404                   "HTTP/1.1 404 Not Found"
#define HTTP_HEADER_405                   "HTTP/1.1 405 Method Not Allowed"
#define HTTP_HEADER_406                   "HTTP/1.1 406 Not Acceptable"
#define HTTP_HEADER_412                   "HTTP/1.1 412 Precondition Failed"
#define HTTP_HEADER_429                   "HTTP/1.1 429 Too Many Requests"
#define HTTP_HEADER_444                   "HTTP/1.1 444"
#define HTTP_HEADER_470                   "HTTP/1.1 470 Connection Authorization Required"
#define HTTP_HEADER_500                   "HTTP/1.1 500 Internal Server Error"
#define HTTP_HEADER_504                   "HTTP/1.1 504 Not Able to Connect"
#define HTTP_HEADER_CONTENT_LENGTH        "Content-Length: "
#define HTTP_HEADER_CONTENT_TYPE          "Content-Type: "
#define HTTP_HEADER_CHUNKED               "Transfer-Encoding: chunked"
#define HTTP_HEADER_LOCATION              "Location: "
#define HTTP_HEADER_ACCEPT                "Accept: "
#define HTTP_HEADER_KEEP_ALIVE            "Connection: Keep-Alive"
#define HTTP_HEADER_CLOSE                 "Connection: close"
#define NO_CACHE_HEADER                   "Cache-Control: no-store, no-cache, must-revalidate, post-check=0, pre-check=0\r\n"\
                                          "Pragma: no-cache"
#define CRLF                              "\r\n"
#define CRLF_CRLF                         "\r\n\r\n"
#define LFLF                              "\n\n"
#define EVENT_STREAM_DATA                 "data: "

/******************************************************
 *                   Enumerations
 ******************************************************/

/**
 * HTTP cache
 */
typedef enum
{
    HTTP_CACHE_DISABLED, /**< Do not cache previously fetched resources */
    HTTP_CACHE_ENABLED,  /**< Allow caching of previously fetched resources  */
} http_cache_t;

/**
 * HTTP MIME type
 */
typedef enum
{
    MIME_TABLE( EXPAND_AS_ENUMERATION )
    MIME_UNSUPPORTED
} wiced_packet_mime_type_t;

/**
 * HTTP status code
 */
typedef enum
{
    HTTP_200_TYPE,
    HTTP_204_TYPE,
    HTTP_207_TYPE,
    HTTP_301_TYPE,
    HTTP_400_TYPE,
    HTTP_403_TYPE,
    HTTP_404_TYPE,
    HTTP_405_TYPE,
    HTTP_406_TYPE,
    HTTP_412_TYPE,
    HTTP_415_TYPE,
    HTTP_429_TYPE,
    HTTP_444_TYPE,
    HTTP_470_TYPE,
    HTTP_500_TYPE,
    HTTP_504_TYPE
} http_status_codes_t;

/**
 * HTTP request type
 */
typedef enum
{
    WICED_HTTP_GET_REQUEST,
    WICED_HTTP_POST_REQUEST,
    WICED_HTTP_PUT_REQUEST,
    REQUEST_UNDEFINED
} wiced_http_request_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * HTTP message structure that gets passed to dynamic URL processor functions
 */
typedef struct
{
    const uint8_t*            data;                         /* packet data in message body      */
    uint16_t                  message_data_length;          /* data length in current packet    */
    uint32_t                  total_message_data_remaining; /* data yet to be consumed          */
    wiced_bool_t              chunked_transfer;             /* chunked data format              */
    wiced_packet_mime_type_t  mime_type;                    /* mime type                        */
    wiced_http_request_type_t request_type;                 /* GET, POST or PUT request         */
} wiced_http_message_body_t;

typedef struct wiced_http_page_s wiced_http_page_t;

typedef struct
{
   wiced_http_page_t*           page_found;
   uint32_t                     total_data_remaning;
   wiced_packet_mime_type_t     mime_type;
   wiced_http_request_type_t    request_type;
} wiced_http_request_info_t;

/**
 * Workspace structure for HTTP server stream
 * Users should not access these values - they are provided here only
 * to provide the compiler with datatype size information allowing static declarations
 */
typedef struct
{
    wiced_tcp_stream_t tcp_stream;
    wiced_bool_t       chunked_transfer_enabled;
} wiced_http_response_stream_t;

typedef struct
{
    wiced_http_response_stream_t response;
    wiced_http_request_info_t    request;
} wiced_http_stream_t;

/**
 * HTTP server receive data callback
 */
typedef wiced_result_t (*http_server_receive_callback_t)( wiced_http_response_stream_t* stream, uint8_t** data, uint16_t* data_length );

/**
 * HTTP server disconnect socket callback
 */
typedef wiced_result_t (*http_server_disconnect_callback_t)( wiced_http_response_stream_t* stream );

/**
 * Prototype for URL processor functions
 */
typedef int32_t (*url_processor_t)(  const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_data );

/**
 * HTTP page list structure
 * Request with content length more than MTU size is handled for RAW_DYNAMIC_URL_CONTENT and WICED_DYNAMIC_CONTENT type for now.
 */
struct wiced_http_page_s
{
    const char* const url;                     /** String containing the path part of the URL of this page/file */
    const char* const mime_type;               /** String containing the MIME type of this page/file */
    enum
    {
        WICED_STATIC_URL_CONTENT,              /** Page is constant data in memory addressable area */
        WICED_DYNAMIC_URL_CONTENT,             /** Page is dynamically generated by a @ref url_processor_t type function */
        WICED_RESOURCE_URL_CONTENT,            /** Page data is proivded by a Wiced Resource which may reside off-chip */
        WICED_RAW_STATIC_URL_CONTENT,          /** Same as @ref WICED_STATIC_URL_CONTENT but HTTP header must be supplied as part of the content */
        WICED_RAW_DYNAMIC_URL_CONTENT,         /** Same as @ref WICED_DYNAMIC_URL_CONTENT but HTTP header must be supplied as part of the content */
        WICED_RAW_RESOURCE_URL_CONTENT         /** Same as @ref WICED_RESOURCE_URL_CONTENT but HTTP header must be supplied as part of the content */
    } url_content_type;                        /** The page type - this selects which part of the @url_content union will be used - also see above */
    union
    {
        struct                                 /* Used for WICED_DYNAMIC_URL_CONTENT and WICED_RAW_DYNAMIC_URL_CONTENT */
        {
            const url_processor_t generator;   /* The function which will handle requests for this page */
            void*                 arg;         /* An argument to be passed to the generator function    */
        } dynamic_data;
        struct                                 /* Used for WICED_STATIC_URL_CONTENT and WICED_RAW_STATIC_URL_CONTENT */
        {
            const void* ptr;                   /* A pointer to the data for the page / file */
            uint32_t length;                   /* The length in bytes of the page / file */
        } static_data;
        const resource_hnd_t* resource_data;   /* A Wiced Resource containing the page / file - Used for WICED_RESOURCE_URL_CONTENT and WICED_RAW_RESOURCE_URL_CONTENT */
    } url_content;
};

/**
 * Workspace structure for HTTP server
 * Users should not access these values - they are provided here only
 * to provide the compiler with datatype size information allowing static declarations
 */
typedef struct
{
    wiced_tcp_server_t                  tcp_server;
    wiced_thread_t                      event_thread;
    wiced_queue_t                       event_queue;
    wiced_worker_thread_t               connect_thread;
    volatile wiced_bool_t               quit;
    const wiced_http_page_t*            page_database;
    uint8_t*                            streams;
    linked_list_t                       active_stream_list;
    linked_list_t                       inactive_stream_list;
    http_server_receive_callback_t      receive_callback;
    http_server_disconnect_callback_t   disconnect_callback;
} wiced_http_server_t;

typedef wiced_http_server_t wiced_https_server_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/**
 * @defgroup http       HTTP
 * @ingroup ipcoms
 *
 * @addtogroup http_server      HTTP Server
 * @ingroup http
 *
 * Communication functions for HTTP (Hypertext Transfer Protocol) Server
 *
 * HTTP functions as a request-response protocol in the client-server computing model. A web browser,
 * for example, may be the client and an application running on a computer hosting a website may be the
 * server. The client submits an HTTP request message to the server. The server, which provides
 * resources such as HTML files and other content, or performs other functions on behalf of the client,
 * returns a response message to the client. The response contains completion status information about
 * the request and may also contain requested content in its message body.
 *
 * The HTTP server library on WICED is capable of both secure [with TLS security] and
 * non-secure mode of connection. The library also provides support for various RESTful HTTP methods
 * such as GET, POST and PUT; and has support for various content types [e.g. HTML, Plain, JSON].
 * The HTTP library is capable of handling content payload that is greater than MTU size.
 *
 *  @{
 */
/*****************************************************************************/

/**
 * Start a HTTP server daemon (web server)
 *
 * The web server implements HTTP1.1 using a non-blocking architecture which allows
 * multiple sockets to be served simultaneously.
 * Web pages and other files can be served either dynamically from a function or
 * from static data in memory or internal/external flash resources
 *
 * @param[in] server         Structure workspace that will be used for this HTTP server instance - allocated by caller.
 * @param[in] port           TCP port number on which to listen - usually port 80 for normal HTTP
 * @param[in] max_sockets    Maximum number of sockets to be served simultaneously
 * @param[in] page_database  A list of web pages / files that will be served by the HTTP server. See @ref wiced_http_page_t for details and snippet apps for examples
 * @param[in] interface      Which network interface the HTTP server should listen on.
 * @param[in] interface      Thread stack size
 *
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_server_start( wiced_http_server_t* server, uint16_t port, uint16_t max_sockets, const wiced_http_page_t* page_database, wiced_interface_t interface, uint32_t http_thread_stack_size );

/**
 *  Stop a HTTP server daemon (web server)
 *
 * @param[in] server   The structure workspace that was used with @ref wiced_http_server_start
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_server_stop( wiced_http_server_t* server );

/**
 * Start a HTTPS server daemon (secure web server)
 *
 * This is identical to @ref wiced_http_server_start except that it uses TLS to provide
 * a secure HTTPS link
 *
 * @param[in] server         Structure workspace that will be used for this HTTP server instance - allocated by caller.
 * @param[in] port           TCP port number on which to listen - usually port 80 for normal HTTP
 * @param[in] max_sockets    Maximum number of sockets to be served simultaneously
 * @param[in] page_database  A list of web pages / files that will be served by the HTTP server. See @ref wiced_http_page_t for details and snippet apps for examples
 * @param[in] server_cert    A string containing the X.509 server certificate which is BASE64 DER encoded
 * @param[in] server_key     A string containing the key for the server_cert (BASE64 DER encoded)
 * @param[in] interface      Which network interface the HTTP server should listen on.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_https_server_start( wiced_https_server_t* server, uint16_t port, uint16_t max_sockets, const wiced_http_page_t* page_database, wiced_tls_identity_t* identity, wiced_interface_t interface, uint32_t url_processor_stack_size );

/**
 *  Stop a HTTPS server daemon (web server)
 *
 * @param[in] server   The structure workspace that was used with @ref wiced_https_server_start
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_https_server_stop( wiced_https_server_t* server );

/**
 * Register HTTP server callback(s)
 *
 * @param[in] server              : HTTP server
 * @param[in] receive_callback    : Callback function that will be called when a packet is received by the server
 * @param[in] disconnect_callback : Callback function that will be called when a disconnection event is received by the server
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_server_register_callbacks( wiced_http_server_t* server, http_server_receive_callback_t receive_callback, http_server_disconnect_callback_t disconnect_callback );

/**
 * Deregister HTTP server callback(s)
 *
 * @param[in] server : HTTP server
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_server_deregister_callbacks( wiced_http_server_t* server );

/**
 * Queue a disconnect request to the HTTP server
 *
 * @param[in] stream : stream to disconnect
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_disconnect( wiced_http_response_stream_t* stream );

/**
 * Disconnect all HTTP stream in a server
 *
 * @param[in] server   The structure workspace that was used with @ref wiced_http_server_start
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_disconnect_all_response_stream( wiced_https_server_t* server );

/**
 * Initialise HTTP server stream
 *
 * @param[in] stream : HTTP server stream
 * @param[in] socket : TCP socket for the stream to use
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_init( wiced_http_response_stream_t* stream, wiced_tcp_socket_t* socket );

/**
 * Deinitialise HTTP server stream
 *
 * @param[in] stream : HTTP server stream
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_deinit( wiced_http_response_stream_t* stream );

/**
 * Enable chunked transfer encoding on the HTTP stream
 *
 * @param[in] stream : HTTP stream
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_enable_chunked_transfer( wiced_http_response_stream_t* stream );

/**
 * Disable chunked transfer encoding on the HTTP stream
 *
 * @param[in] stream : HTTP stream
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_disable_chunked_transfer( wiced_http_response_stream_t* stream );

/**
 * Write HTTP header to the TCP stream provided
 *
 * @param[in] stream         : HTTP stream to write the header into
 * @param[in] status_code    : HTTP status code
 * @param[in] content_length : HTTP content length to follow in bytes
 * @param[in] cache_type     : HTTP cache type (enabled or disabled)
 * @param[in] mime_type      : HTTP MIME type
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_write_header( wiced_http_response_stream_t* stream, http_status_codes_t status_code, uint32_t content_length, http_cache_t cache_type, wiced_packet_mime_type_t mime_type );

/**
 * Write data to HTTP stream
 *
 * @param[in] stream : HTTP stream to write the data into
 * @param[in] data   : data to write
 * @param[in] length : data length in bytes
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_write( wiced_http_response_stream_t* stream, const void* data, uint32_t length );

/**
 * Write resource to HTTP stream
 *
 * @param[in] stream   : HTTP stream to write the resource into
 * @param[in] resource : Pointer to resource
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_write_resource( wiced_http_response_stream_t* stream, const resource_hnd_t* res_id );

/**
 * Flush HTTP stream
 *
 * @param[in] stream : HTTP stream to flush
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_http_response_stream_flush( wiced_http_response_stream_t* stream );

/**
 * Search for a parameter (key-value pair) in a URL query string and return a pointer to the value
 *
 * @param[in]  url_query       : URL query string
 * @param[in]  parameter_key   : Key or name of the parameter to find in the URL query string
 * @param[out] parameter_value : If the parameter with the given key is found, this pointer will point to the parameter value upon return; NULL otherwise
 * @param[out] value_length    : This variable will contain the length of the parameter value upon return; 0 otherwise
 *
 * @return @ref wiced_result_t WICED_SUCCESS if found; WICED_NOT_FOUND if not found
 */
wiced_result_t wiced_http_get_query_parameter_value( const char* url_query, const char* parameter_key, char** parameter_value, uint32_t* value_length );

/**
 * Return the number of parameters found in the URL query string
 *
 * @param[in] url_query : URL query string
 *
 * @return parameter count
 */
uint32_t wiced_http_get_query_parameter_count( const char* url_query );

/**
 * Match a URL query string contains a parameter with the given parameter key and value
 *
 * @param[in]  url_query       : URL query string
 * @param[in]  parameter_key   : NUL-terminated key or name of the parameter to find in the URL query string
 * @param[out] parameter_value : NUL-terminated value of the parameter to find in the URL query string
 *
 * @return @ref wiced_result_t WICED_SUCCESS if matched; WICED_NOT_FOUND if matching parameter is not found
 */
wiced_result_t wiced_http_match_query_parameter( const char* url_query, const char* parameter_key, const char* parameter_value );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
