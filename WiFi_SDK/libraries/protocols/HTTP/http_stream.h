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

#include "wiced_result.h"
#include "wiced_tcpip.h"
#include "wiced_network.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_HTTP_HOST_NAME_SIZE         ( 40 )

#define HTTP_NEW_LINE                   "\r\n"
#define HTTP_HEADERS_BODY_SEPARATOR     "\r\n\r\n"

/******************************************************
 *                   Enumerations
 ******************************************************/

/* rfc2616 */
typedef enum
{
    HTTP_UNKNOWN  = -1,
    HTTP_OPTIONS  =  0,
    HTTP_GET      =  1,
    HTTP_HEAD     =  2,
    HTTP_POST     =  3,
    HTTP_PUT      =  4,
    HTTP_DELETE   =  5,
    HTTP_TRACE    =  6,
    HTTP_CONNECT  =  7,
    HTTP_NOTIFY   =  8,
    HTTP_M_SEARCH =  9,
    HTTP_END      = 10,

    HTTP_METHODS_MAX,   /* must be last! */
} http_request_t;

typedef enum
{
    HTTP_CONTINUE                        = 100,
    HTTP_SWITCHING_PROTOCOLS             = 101,
    HTTP_RESPONSE_OK                     = 200,
    HTTP_CREATED                         = 201,
    HTTP_ACCEPTED                        = 202,
    HTTP_NONAUTHORITATIVE                = 203,
    HTTP_NO_CONTENT                      = 204,
    HTTP_RESET_CONTENT                   = 205,
    HTTP_PARTIAL_CONTENT                 = 206,
    HTTP_MULTIPLE_CHOICES                = 300,
    HTTP_MOVED_PERMANENTLY               = 301,
    HTTP_FOUND                           = 302,
    HTTP_SEE_OTHER                       = 303,
    HTTP_NOT_MODIFIED                    = 304,
    HTTP_USEPROXY                        = 305,
    HTTP_TEMPORARY_REDIRECT              = 307,
    HTTP_BAD_REQUEST                     = 400,
    HTTP_UNAUTHORIZED                    = 401,
    HTTP_PAYMENT_REQUIRED                = 402,
    HTTP_FORBIDDEN                       = 403,
    HTTP_NOT_FOUND                       = 404,
    HTTP_METHOD_NOT_ALLOWED              = 405,
    HTTP_NOT_ACCEPTABLE                  = 406,
    HTTP_PROXY_AUTHENTICATION_REQUIRED   = 407,
    HTTP_REQUEST_TIMEOUT                 = 408,
    HTTP_CONFLICT                        = 409,
    HTTP_GONE                            = 410,
    HTTP_LENGTH_REQUIRED                 = 411,
    HTTP_PRECONDITION_FAILED             = 412,
    HTTP_REQUESTENTITYTOOLARGE           = 413,
    HTTP_REQUESTURITOOLONG               = 414,
    HTTP_UNSUPPORTEDMEDIATYPE            = 415,
    HTTP_REQUESTED_RANGE_NOT_SATISFIABLE = 416,
    HTTP_EXPECTATION_FAILED              = 417,
    HTTP_INTERNAL_SERVER_ERROR           = 500,
    HTTP_NOT_IMPLEMENTED                 = 501,
    HTTP_BAD_GATEWAY                     = 502,
    HTTP_SERVICE_UNAVAILABLE             = 503,
    HTTP_GATEWAY_TIMEOUT                 = 504,
    HTTP_VERSION_NOT_SUPPORTED           = 505,
} http_status_code_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_bool_t       is_connected;
    wiced_ip_address_t host_ip;
    wiced_tcp_socket_t socket;
    wiced_tcp_stream_t tcp_stream;
    wiced_packet_t*    packet;
    char               hostname[MAX_HTTP_HOST_NAME_SIZE];
} http_stream_t;

typedef struct
{
    const char* name;
    char* value;
} http_header_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* HTTP stream management functions */
wiced_result_t http_init_stream  ( http_stream_t* session, wiced_interface_t interface, const char* host_name, wiced_ip_address_t* ip_address, uint16_t port );
wiced_result_t http_deinit_stream( http_stream_t* session );

/* HTTP stream header functions */
wiced_result_t http_stream_start_headers    ( http_stream_t* session, http_request_t method, const char* url );
wiced_result_t http_stream_add_headers      ( http_stream_t* session, const http_header_t* headers, uint16_t number_of_headers );
wiced_result_t http_send_basic_authorization( http_stream_t* session, char* username_password );
wiced_result_t http_stream_end_headers      ( http_stream_t* session, wiced_bool_t keep_alive);

/* HTTP stream writing functions */
wiced_result_t http_stream_write( http_stream_t* session, uint8_t* data, uint16_t length );
wiced_result_t http_stream_flush( http_stream_t* session );

/* HTTP stream reading and processing functions */
wiced_result_t http_stream_read     ( http_stream_t* session, uint8_t* data, uint16_t length, uint32_t timeout, uint32_t* read_count );
wiced_result_t http_stream_receive  ( http_stream_t* session, wiced_packet_t** packet, uint32_t timeout );
wiced_result_t http_extract_headers ( wiced_packet_t* packet, http_header_t* headers, uint16_t number_of_headers );
wiced_result_t http_get_body        ( wiced_packet_t* packet, uint8_t** body, uint32_t* body_length );
http_request_t http_determine_method(wiced_packet_t* packet);
wiced_result_t http_process_response( wiced_packet_t* packet, http_status_code_t* response_code );


/** Process the packet headers in palce
 *
 * NOTE: Inserts null characters ('\0') at end of header names and header values IN PLACE
 *
 * @param packet                - [packet]
 * @param headers               - [in]      pointer to http_header_t structure array to be filled
 * @param number_of_headers     - [in]      number of headers in array
 *                                [out]     number of headers parsed into array
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t http_process_headers_in_place( wiced_packet_t* packet, http_header_t* headers_handle, uint16_t* number_of_headers );


#ifdef __cplusplus
} /* extern "C" */
#endif
