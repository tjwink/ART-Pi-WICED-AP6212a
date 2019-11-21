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

#include "wiced_defaults.h"
#include "wiced_tcpip.h"
#include "wiced_rtos.h"
#include "wiced_resource.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define HTTP_REDIRECT(url) "HTTP/1.0 301\r\nLocation: " url "\r\n\r\n"

#define HTTP_404 \
    "HTTP/1.0 404 Not Found\r\n" \
    "Content-Type: text/html\r\n\r\n" \
    "<!doctype html>\n" \
    "<html><head><title>404 - WICED Web Server</title></head><body>\n" \
    "<h1>Address not found on WICED Web Server</h1>\n" \
    "<p><a href=\"/\">Return to home page</a></p>\n" \
    "</body>\n</html>\n"

#define START_OF_HTTP_PAGE_DATABASE(name) \
    const wiced_http_page_t name[] = {

#define ROOT_HTTP_PAGE_REDIRECT(url) \
    { "/", "", WICED_RAW_STATIC_URL_CONTENT, .url_content.static_data  = {HTTP_REDIRECT(url), sizeof(HTTP_REDIRECT(url))-1 } }

#define END_OF_HTTP_PAGE_DATABASE() \
    {0,0,0, .url_content.static_data  = {NULL, 0 } } \
    }

/******************************************************
 *                    Constants
 ******************************************************/

#define IOS_CAPTIVE_PORTAL_ADDRESS    "/library/test/success.html"

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int32_t (*url_processor_t)( const char* url, wiced_tcp_stream_t* stream, void* arg );

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    const char* const url;
    const char* const mime_type;
    enum
    {
        WICED_STATIC_URL_CONTENT,
        WICED_DYNAMIC_URL_CONTENT,
        WICED_RESOURCE_URL_CONTENT,
        WICED_RAW_STATIC_URL_CONTENT,
        WICED_RAW_DYNAMIC_URL_CONTENT,
        WICED_RAW_RESOURCE_URL_CONTENT
    } url_content_type;
    union
    {
        struct
        {
            const url_processor_t generator;
            void*                 arg;
        } dynamic_data;
        struct
        {
            const void* ptr;
            uint32_t length;
        } static_data;
        const resource_hnd_t* resource_data;
    } url_content;
} wiced_http_page_t;

typedef struct
{
    wiced_tcp_socket_t       socket;
    volatile wiced_bool_t    quit;
    wiced_thread_t           thread;
    const wiced_http_page_t* page_database;
} wiced_simple_http_server_t;

typedef struct
{
    wiced_tcp_socket_t         socket;
    volatile wiced_bool_t      quit;
    wiced_thread_t             thread;
    const wiced_http_page_t*   page_database;
    wiced_tls_context_t        tls_context;
    wiced_tls_identity_t       tls_identity;
} wiced_simple_https_server_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t wiced_simple_http_server_start       ( wiced_simple_http_server_t* server, uint16_t port, const wiced_http_page_t* page_database, wiced_interface_t interface );
wiced_result_t wiced_simple_http_server_stop        ( wiced_simple_http_server_t* server );

wiced_result_t wiced_simple_https_server_start      ( wiced_simple_https_server_t* server, uint16_t port, const wiced_http_page_t* page_database, const uint8_t* server_cert, const char* server_key, wiced_interface_t interface );
wiced_result_t wiced_simple_https_server_stop       ( wiced_simple_https_server_t* server );

wiced_result_t wiced_simple_http_write_reply_header ( wiced_tcp_stream_t* stream, const char* mime_type, wiced_bool_t nocache );

#ifdef __cplusplus
} /* extern "C" */
#endif
