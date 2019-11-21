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
#include "wiced_rtos.h"
#include "linked_list.h"
#include "http.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 * @cond               Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 * HTTP security type
*/
typedef enum
{
    HTTP_NO_SECURITY, /* Standard HTTP over TCP */
    HTTP_USE_TLS,     /* HTTPS over secure TLS tunnel */
} http_security_t;

/**
 * HTTP events received from the HTTP library
*/
typedef enum
{
    HTTP_NO_EVENT,      /* Reserved. Not used currently */
    HTTP_CONNECTED,     /* Connection for HTTP transaction is established */
    HTTP_DISCONNECTED,  /* Connection for HTTP transaction is dropped */
    HTTP_DATA_RECEIVED, /* Marks beginning of the header or the beginning of a segment if the header is fragmented over several packets */
} http_event_t;

/**
 * HTTP client configuration flags
*/
typedef enum
{
    HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME      = (0x1),
    HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN = (0x1 << 1),
} http_client_configuration_flags_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
/**
 * HTTP client configuration settings if security is enabled. Depends on the http_client_configuration_flags_t field
*/
typedef struct
{
    http_client_configuration_flags_t   flag;                /* HTTP client configuration flags */
    uint8_t*                            server_name;         /* server name upto length 256 acceptable and should be null terminated string */
    wiced_tls_max_fragment_length_t     max_fragment_length; /* Maximum fragment length that the client can receive */
} http_client_configuration_info_t;

/**
 * HTTP client object
*/
typedef struct
{
    wiced_tcp_socket_t    socket;             /* TCP socket handle */
    wiced_tls_identity_t* tls_identity;       /* TLS object that encompasses the device's own certificate/key */
    wiced_tls_context_t*  tls_context;        /* TLS context object that has all the information to process a TLS message */
    uint32_t              event_handler;      /* HTTP event handler callback @ref http_event_handler_t */
    linked_list_t         request_list;       /* Linked list of outstanding HTTP requests from application */
    wiced_worker_thread_t thread;             /* HTTP worker thread to process upstream HTTP frames */
    http_client_configuration_info_t *config; /* HTTP client configuration settings (optional) */
    uint8_t*              peer_cn;            /* Peer Common Name (optional) */
    void*                 user_data;          /* Reserved */
} http_client_t;

/**
 * HTTP request
*/
typedef struct
{
    linked_list_node_t node;   /* Encloses a HTTP request */
    wiced_tcp_stream_t stream; /* TCP stream object */
    http_client_t*     owner;  /* HTTP client object associated with the request */
    void*              context; /* User data that will be passed along with the response */
}http_request_t;

/**
 * HTTP response
*/
typedef struct
{
    http_request_t* request;             /* This contains the pointer to the HTTP request */
    uint8_t*        response_hdr;        /* This contains HTTP response header including status line */
    uint16_t        response_hdr_length; /* Response_hdr_length is the length of HTTP header. */
    uint8_t*        payload;             /* This contains Payload received in response */
    uint16_t        payload_data_length; /* This length indicates only payload length */
    uint16_t        remaining_length;    /* Remaining data for this response. */
} http_response_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 * @endcond
 ******************************************************/

/*****************************************************************************/
/**
 * @defgroup http       HTTP
 * @ingroup ipcoms
 *
 * @addtogroup http_client      HTTP Client
 * @ingroup http
 *
 * Communication functions for HTTP (Hypertext Transfer Protocol) Client
 *
 * HTTP functions as a request-response protocol in the client-server computing model. A web browser,
 * for example, may be the client and an application running on a computer hosting a website may be the
 * server. The client submits an HTTP request message to the server. The server, which provides
 * resources such as HTML files and other content, or performs other functions on behalf of the client,
 * returns a response message to the client. The response contains completion status information about
 * the request and may also contain requested content in its message body.
 *
 * The HTTP client library on WICED is capable of both secure [with TLS security] and
 * non-secure mode of connection. The library also provides support for various RESTful HTTP methods
 * such as GET, POST and PUT; and has support for various content types [e.g. HTML, Plain, JSON].
 * The HTTP library is capable of handling content payload that is greater than MTU size.
 *
 *  @{
 */
/*****************************************************************************/

/**
 * HTTP client event handler callback
 *
 * Callback for HTTP client events that are part of http_event_t
 * Callback will be invoked when response is received by the library.
 * Registered using http_client_init()
 *
 * @param[out] client           : HTTP client
 * @param[out] event          : Event received from the HTTP library.
 * @param[out] response      : Event response data. The 'request' field shall point to the HTTP request that matches
 *                                       the HTTP response. The HTTP library shall automatically free the response and
 *                                       payload that is part of the response upon return of the callback. Application must
 *                                       copy the payload as needed. Application is expected to preserve data if content
 *                                       length exceeds MTU. The 'remaining_length' field in http_response_t indicates the
 *                                       data remaining; remaining length '0' indicates that the response is complete.
 *
 * @return none
 */
typedef void (*http_event_handler_t)( http_client_t* client, http_event_t event, http_response_t* response );

/**
 * Initialize HTTP client
 *
 * @param[in] client                : HTTP client
 * @param[in] interface             : WICED interface
 * @param[in] event_handler         : Event callback function
 * @param[in] optional_tls_identity : Pointer to client TLS identity, if available
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_client_init( http_client_t* client, wiced_interface_t interface, http_event_handler_t event_handler, wiced_tls_identity_t* optional_identity );

/**
 * Configure HTTP client connection related configuration
 *
 * This API needs to be called before http_client_connect API and is configured per connection.
 * API is optional and need not be called if default values are fine.
 *
 * @param[in] client                : HTTP client
 * @param[in] client_config         : pointer to client configuration info
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_client_configure(http_client_t* client, http_client_configuration_info_t* client_config);

/**
 * De-initialize HTTP client. This API should not be invoked from the http_event_handler.
 *
 * @param[in] client : HTTP client
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_client_deinit( http_client_t* client );

/**
 * Connect to a HTTP server
 *
 * @param[in] client     : HTTP client
 * @param[in] server_ip  : HTTP server IP address
 * @param[in] port       : TCP port
 * @param[in] security   : Security type i.e. HTTP or HTTPS
 * @param[in] timeout_ms : Connection timeout in milliseconds
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_client_connect( http_client_t* client, const wiced_ip_address_t* server_ip, uint16_t port, http_security_t security, uint32_t timeout_ms );

/**
 * Disconnect client from HTTP server
 *
 * @param[in] client : HTTP client
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_client_disconnect( http_client_t* client );

/**
 * Initialize a HTTP request
 *
 * @param[in] request : HTTP request
 * @param[in] client  : HTTP client
 * @param[in] method  : HTTP request method
 * @param[in] uri     : Universal Resource Identifier or URI (normally starts with '/')
 * @param[in] version : HTTP version
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_init( http_request_t* request, http_client_t* client, http_method_t method, const char* uri, http_version_t version );

/**
 * De-initialize a HTTP request
 *
 * @param[in] request : HTTP request
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_deinit( http_request_t* request );

/**
 * Write header to the HTTP request
 *
 * @param[in] request          : HTTP request
 * @param[in] header_fields    : Pointer to an array containing header fields and they values
 * @param[in] number_of_fields : Total number of the header fields i.e. the size of the array
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_write_header( http_request_t* request, const http_header_field_t* header_fields, uint32_t number_of_fields );

/**
 * Write end of header (blank line) to the HTTP request
 *
 * @param[in] request : HTTP request
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_write_end_header( http_request_t* request );

/**
 * Write data to the HTTP request
 *
 * @param[in] request : HTTP request
 * @param[in] data    : Data to write
 * @param[in] length  : Data length in bytes
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_write( http_request_t* request, const uint8_t* data, uint32_t length );

/**
 * Flush (send) the HTTP request to the server. The response will be received via the HTTP event handler specified in http_client_init()
 *
 * @param[in] request : HTTP request
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_flush( http_request_t* request );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
