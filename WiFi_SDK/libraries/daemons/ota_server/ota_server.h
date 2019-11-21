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

#define OTA_HTTP_REDIRECT(url) "HTTP/1.0 301\r\nLocation: " url "\r\n\r\n"

#define OTA_HTTP_404 \
    "HTTP/1.0 404 Not Found\r\n" \
    "Content-Type: text/html\r\n\r\n" \
    "<!doctype html>\n" \
    "<html><head><title>404 - WICED Web Server</title></head><body>\n" \
    "<h1>Address not found on WICED Web Server</h1>\n" \
    "<p><a href=\"/\">Return to home page</a></p>\n" \
    "</body>\n</html>\n"

#define OTA_START_OF_HTTP_PAGE_DATABASE(name) \
    const ota_http_page_t name[] = {

#define OTA_ROOT_HTTP_PAGE_REDIRECT(url) \
    { "/", "", OTA_RAW_STATIC_URL_CONTENT, .ota_url_content.ota_static_data  = {OTA_HTTP_REDIRECT(url), sizeof(OTA_HTTP_REDIRECT(url))-1 } }

#define OTA_END_OF_HTTP_PAGE_DATABASE() \
    {0,0,0, .ota_url_content.ota_static_data  = {NULL, 0 } } \
    }

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct
{
    uint8_t* data;
    uint16_t size;
} http_body_chunk_t;

typedef struct
{
    wiced_packet_t*           request_packets[3];
    http_body_chunk_t         body_chunks[3];
    uint16_t                  current_packet_index;
    uint8_t*                  request_type;
    uint16_t                  request_type_length;
    uint8_t*                  header_ptr;
    uint16_t                  header_size;
    uint8_t*                  protocol_type_ptr;
    uint16_t                  protocol_type_length;
    uint8_t*                  url_ptr;
    uint16_t                  url_length;
    uint8_t*                  query_ptr;
    uint16_t                  query_length;
    uint16_t                  content_length;
} ota_http_request_message_t;


/******************************************************
 *                    Structures
 ******************************************************/

typedef int (*ota_http_request_processor_t)( ota_http_request_message_t* , wiced_tcp_stream_t* stream, void* arg );

typedef struct
{
    const char* const ota_url;
    const char* const ota_mime_type;
    enum
    {
        OTA_STATIC_URL_CONTENT,
        OTA_DYNAMIC_URL_CONTENT,
        OTA_RESOURCE_URL_CONTENT,
        OTA_RAW_STATIC_URL_CONTENT,
        OTA_RAW_DYNAMIC_URL_CONTENT,
        OTA_RAW_RESOURCE_URL_CONTENT
    } ota_url_content_type;
    union
    {
        struct
        {
            const ota_http_request_processor_t generator;
            void*                 arg;
        } ota_dynamic_data;
        struct
        {
            const void* ptr;
            uint32_t length;
        } ota_static_data;
        const resource_hnd_t* ota_resource_data;
    } ota_url_content;
} ota_http_page_t;


typedef enum
{
    READING_HEADER,
    READING_BODY
} ota_server_connection_state_t;

typedef struct
{
    wiced_tcp_socket_t              socket;
    ota_server_connection_state_t   state;
    volatile wiced_bool_t           quit;
    wiced_thread_t                  thread;
    const ota_http_page_t*          page_database;
    ota_http_request_message_t      request;
    wiced_bool_t                    reboot_required;
} ota_server_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t ota_server_start       ( ota_server_t* server, uint16_t port, const ota_http_page_t* page_database, wiced_interface_t interface );
wiced_result_t ota_server_stop        ( ota_server_t* server );

#ifdef __cplusplus
} /* extern "C" */
#endif
