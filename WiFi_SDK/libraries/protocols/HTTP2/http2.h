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

#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *            Enumerations
 ******************************************************/
typedef enum http_frame_type_e
{
    HTTP_FRAME_TYPE_HEADER = 0,
    HTTP_FRAME_TYPE_PING,
    HTTP_FRAME_TYPE_PUSH_PROMISE
} http_frame_type_t;

typedef enum http_frame_flags_e
{
    HTTP_FRAME_FLAGS_NONE           = 0x00,
    HTTP_FRAME_FLAGS_ACK            = 0x01,
    HTTP_FRAME_FLAGS_END_STREAM     = 0x01,
    HTTP_FRAME_FLAGS_END_HEADER     = 0x04,
    HTTP_FRAME_FLAGS_PADDED         = 0x08,
    HTTP_FRAME_FLAGS_PRIORITY       = 0x20,
} http_frame_flags_t;

/**
 * HTTP client configuration flags
*/
typedef enum
{
    HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME      = (0x1),
} http_client_config_flags_t;

/******************************************************
 *                    Constants
 ******************************************************/
/* WICED HTTP2 default settings. */
#define HTTP2_SETTINGS_HEADER_TABLE_SIZE       ( 0 )
#define HTTP2_SETTINGS_ENABLE_PUSH             ( 1 )
#define HTTP2_SETTINGS_MAX_CONCURRENT_STREAMS  ( 4 )
#define HTTP2_SETTINGS_MAX_FRAME_SIZE          ( 16 * 1024 )
#define HTTP2_SETTINGS_MAX_HEADER_LIST_SIZE    ( 16 * 1024 )
#define HTTP2_SETTINGS_INITIAL_WINDOW_SIZE     ( 16 * 1024 )

#define HTTP2_FRAME_FIXED_HEADER_SIZE          (9)
#define HTTP2_FRAME_SCRATCH_BUFF_SIZE          (HTTP2_SETTINGS_MAX_FRAME_SIZE + HTTP2_SETTINGS_HEADER_TABLE_SIZE + HTTP2_FRAME_FIXED_HEADER_SIZE)
#define HTTP2_PING_FRAME_DATA_LENGTH           (8)

#define HTTP2_CONNECTION_TIMEOUT_IN_MSEC       (5 * 1000)

/* Use the below flag to build the library for a server (Non tested ) */
//#define HTTP_IS_SERVER

/******************************************************
 *             Structures
 ******************************************************/
typedef struct http_security_info_s
{
    const char*                     ca_cert;                /* Root CA certificate */
    const char*                     cert;                   /* Client certificate in PEM format */
    const char*                     key;                    /* Client private key */
    uint32_t                        ca_cert_len;
    uint32_t                        cert_len;
    uint32_t                        key_len;
} http_security_info_t;

typedef struct http_socket_s
{
    void*                           p_user;
    wiced_tcp_socket_t              socket;
    wiced_tls_context_t             tls_context;
    wiced_tls_identity_t            tls_identity;
    const http_security_info_t*     security;
} http_socket_t;

typedef struct http_header_info_s
{
    uint8_t*                        name;
    uint8_t*                        value;
    uint32_t                        name_length;
    uint32_t                        value_length;
    /* The http_header_t is implemented as a linked list, because the header table
     * size in the setting frame in set to include the header name and value +
     * 32 bytes for any pointers. So this memory once allocated should be able
     * to old the http_header_t as well. If we take it out of this memory, then
     * we need to allocate a place to it and we don't guarantee the received size.
     */
    struct http_header_info_s*      next;
} http_header_info_t;

typedef struct http_connection_settings_s
{
    uint32_t                        header_table_size;      /** HPACK table size, in bytes */
    uint32_t                        enable_push;            /** Server push notification support */
    uint32_t                        max_concurrent_streams; /** Max number of concurrent streams supported for the connection */
    uint32_t                        initial_window_size;    /** Flow control window size, in bytes */
    uint32_t                        max_frame_size;         /** Maximum size of HTTP2 frame, in bytes */
    uint32_t                        max_header_list_size;   /** Maximum size of HTTP2 header list, in bytes */
} http_connection_settings_t;

/**
 * Client configuration settings if security is enabled. Depends on the http_client_configuration_flags_t field
*/
typedef struct
{
    http_client_config_flags_t   flag;                /* HTTP client configuration flags */
    uint8_t*                     server_name;         /* server name upto length 256 acceptable and should be null terminated string */
} http_client_config_info_t;

typedef uint32_t http_request_id_t;

typedef struct http_connection_s* http_connection_ptr_t;

typedef struct http_connection_callbacks_s
{
    /**
     * A call back when receiving header fields
     *
     * For new requests a dynamic request object is created.
     *
     * Clients should receive response headers ( for already created requests ).
     *
     * Servers should receive request headers ( along with new created request ).
     *
     * A client can receive a request header as a result of a server sending a
     * promise push request. This will most probably be followed by a response
     * header for the already mentioned request.
     *
     * If a client receives a request header while the HTTP_REQUEST_FLAGS_PROMISE_PUSH
     * is not set, this indicates that the request is legitimate and that the client
     * should respond with the appropriate header / data.
     *
     * The  HTTP_REQUEST_FLAGS_FINISH_REQUEST is set if the received header is
     * the last part of the request.
     *
     * @param[in] connection     Structure workspace that will be used for this HTTP connection.
     * @param[in] request        HTTP2 stream ID.
     * @param[in] headers        A pointer to a list of http headers
     * @param[in] type           Indicates the HTTP2 frame type.
     * @param[in] flags          Flags associated with HEADER frames.
     * @param[in] user_data      Pointer to the user data which is passed during connection init.
     *
     * @return @ref wiced_result_t
     */
    wiced_result_t (*http_request_recv_header_callback) ( http_connection_ptr_t connect, http_request_id_t request, http_header_info_t* headers, http_frame_type_t type, http_frame_flags_t flags, void* user_data );

    /**
     * A call back when receiving data
     *
     * Clients should receive response body ( for already created requests ).
     *
     * Servers should receive request body.
     *
     * The HTTP_REQUEST_FLAGS_FINISH_REQUEST is set if the received data is
     * the last part of the request.
     *
     * @param[in] connection     Structure workspace that will be used for this HTTP connection.
     * @param[in] request        HTTP2 stream ID.
     * @param[in] headers        A pointer to a list of http headers
     * @param[in] flags          Flags associated with DATA frame.
     * @param[in] user_data      Pointer to the user data which is passed during connection init.
     *
     * @return @ref wiced_result_t
     */
    wiced_result_t (*http_request_recv_data_callback) ( http_connection_ptr_t connect, http_request_id_t request, uint8_t* data, uint32_t length, http_frame_flags_t flags, void* user_data );

    /**
     * A call back when connection is closed
     *
     * A client and/or server receives the call back when receiving a close connection request
     * and when the connection is closed suddenly.
     *
     * @param[in] connect                   Structure workspace that will be used for this HTTP connection.
     * @param[in] last_processed_request    Last processed stream when receiving go away.
     * @param[in] error                     The error code received in a go away frame.
     * @param[in] user_data                 Pointer to the user data which is passed during connection init.
     *
     * @return @ref wiced_result_t
     */
    wiced_result_t (*http_disconnect_callback) ( http_connection_ptr_t connect, http_request_id_t last_processed_request, uint32_t error, void* user_data );
} http_connection_callbacks_t;

/*
 *     http_disconnect()      +---------------+
 *           (or)    +--------> NOT CONNECTED +---------+
 *          Server   |        +-------^-------+         | http_connect()
 *       Closes Conn |                |                 |
 *                   |                |                 |
 *               +---+----+           |           +-----v-----+
 *  GOAWAY Frame | GOAWAY |           |           | CONNECTED | TCP Socket connection established
 *      Sent     +---^----+           |           +-----+-----+
 *                   |        http_disconnect()         |
 *         Receive   |                |                 |
 *       GOAWAY from |           +----+----+            | Send SETTINGS Frame to server
 *         Server    +-----------+  READY  <------------+
 *                               +---------+
 *                             Ready to transact
 */

typedef enum http_network_connection_state_e
{
    HTTP_CONNECTION_NOT_CONNECTED,      /* Initial and final state of the TCP socket closed */
    HTTP_CONNECTION_CONNECTED,          /* TCP socket opened by no data exchange yet */
    HTTP_CONNECTION_READY_LOCAL,        /* Connection settings sent to remote */
    HTTP_CONNECTION_READY_REMOTE,       /* Connection settings received from remote */
    HTTP_CONNECTION_READY,              /* SETTINGS frame sent to server. Connection is up, can now send and receive frames */
    HTTP_CONNECTION_GOAWAY              /* In GOAWAY (i.e. shutdown) mode, either by a Disconnect API or Recv GOAWAY frame */
} http_connection_network_state_t;

typedef struct http_connection_state_s
{
    http_connection_network_state_t connection;
    uint32_t                        frame_fragment;             /* The remaining size from previous frame */
    uint32_t                        stream_id;                  /* The last successfully created stream ID, odd for client, even for server. */
    uint32_t                        last_valid_stream;          /* Last stream that was successful, used in GOAWAY messages. */
    uint32_t                        goaway_stream;              /* Last processed stream when receiving go away */
    uint32_t                        goaway_error;               /* The error code received in a go away */
} http_connection_state_t;

typedef struct http_connection_s
{
    http_connection_settings_t   local_settings;                /* Local SETTINGS Frame information */
    http_connection_settings_t   remote_settings;               /* Remote peer SETTINGS Frame information */
    http_connection_callbacks_t  callbacks;                     /* Callback functions */
    http_security_info_t*        security;                      /* Contains security certificates and key */
    http_socket_t                socket;                        /* Socket object used for establishing the connection */
    http_socket_t*               socket_ptr;
    uint8_t*                     frame_scratch_buf;             /* Pointer to HTTP2 header frame scratch buffer */
    uint32_t                     frame_scratch_buf_size;        /* Scratch buffer size */
    uint8_t*                     header_table;                  /* Pointer to HPACK table scratch buffer */
    wiced_semaphore_t            sem_reentrant;                 /* Semaphore used reentrancy */
    http_connection_state_t      state;                         /* Connection state */
    wiced_thread_t               http2_thread;                  /* Internal HTTP2 processing thread */
    wiced_thread_t*              http2_thread_ptr;
    uint32_t                     stack_size;                    /* HTTP2 worker thread stack size */
    wiced_event_flags_t          events;                        /* Internal event flags */
    void*                        user_data;                     /* Holds the user data associated with the connection */
} http_connection_t;

/******************************************************
 *             Function declarations
 ******************************************************/

/**
 * Initialize an http connection
 *
 * The function initiates an http connection. It allocate resources and sockets
 * if required. It doesn't connect nor send any requests.
 * Should be called by client and server before connecting or sending packets.
 * When called by a server, the server should have already set the socket
 * option of the connection. Once set the init function is not going to allocate
 * a new socket but use the existing one.
 *
 * @param[in] connection        Structure workspace that will be used for this HTTP
 *                              connection. Should be valid for the full life of the
 *                              connection.
 * @param[in] security          A pointer to security parameters passed to connection.
 *                              If set to NULL, indicates a clear connection (i.e. no
 *                              encryption). Should be valid for the full life of the
 *                              connection.
 *                              If ca_cert field is NULL, server certificate will
 *                              NOT be verified.
 * @param[in] callbacks         A pointer to a list of call backs for the requested
 *                              events. Should be valid for the full life of the
 *                              connection.
 * @param[in] scratch_buf       A buffer provided by the user for the HTTP2
 *                              module to use while working. The size of this buffer
 *                              should be >= HTTP2_SETTINGS_HEADER_TABLE_SIZE + HTTP2_SETTINGS_MAX_FRAME_SIZE

 * @param[in] scratch_buf_len   Length of scratch buffer. This should be
 *                              should be >= HTTP2_SETTINGS_HEADER_TABLE_SIZE + HTTP2_SETTINGS_MAX_FRAME_SIZE

 * @param[in] stack_size        HTTP thread stack size.
 * @param[in] user_data         Pointer to the user data associated with the connection, which needs to be passed back in the callbacks.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_connection_init( http_connection_t* connect, http_security_info_t* security, http_connection_callbacks_t* callbacks, void* scratch_buf, uint32_t scratch_buf_len, uint32_t stack_size, void* user_data );

/**
 * Deinitialize an http connection
 *
 * The function deinitializes an http connection. It frees any allocated resources.
 *
 * Should be called by client and server once the connection is closed.
 *
 * @param[in] connection     Structure workspace that will be used for this HTTP
 *                           connection. Should be valid for the full life of the
 *                           connection.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_connection_deinit( http_connection_t* connect );

/**
 * Connects to an http server
 *
 * Creates a connection with the http server on the socket level (along with any
 * security options set). No http packets are exchanged at this point.
 *
 * Should be called by clients (as servers are already connected to the client
 * once the connection is accepted ).
 *
 * @param[in] connection     Structure workspace that will be used for this HTTP connection.
 * @param[in] address        A pointer to IP address of an http server.
 * @param[in] portnumber     TCP port number for connection.
 * @param[in] interface      Network interface to use for connection.
 * @param[in] client_config  HTTP client configuration settings (optional)
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_connect( http_connection_t* connect, wiced_ip_address_t* address, uint16_t portnumber, wiced_interface_t interface, http_client_config_info_t* client_config );

/**
 * Disconnects from an http server or client
 *
 * Disconnects from server or client. The function will also try to achieve a decent disconnect
 * by sending the appropriate close messages for different requests.
 *
 * @param[in] connect     Structure workspace that will be used for this HTTP connection.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_disconnect( http_connection_t* connect );

/**
 * Creates a new request in case of client and sends the header
 *
 * A Client should fill this with a request header ( usually includes method,
 * host, authority, etc. ).
 * A server responding to a received request should include a response header
 * ( status, etc. )
 *
 * The HTTP_FRAME_FLAGS_END_STREAM should be set if the request being sent
 * doesn't have any data payload to send.
 *
 * @param[in] connection     Structure workspace that will be used for this HTTP connection.
 * @param[in] request        A pointer which contains the ID of the created request (stream ID).
 * @param[in] headers        A pointer to a list of HTTP headers
 * @param[in] flags          A bit masked flag to indicate request options.
 *                           - HTTP_FRAME_FLAGS_END_STREAM: Set by server or client when
 *                             the request/response doesn't have any data payload
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_put_headers ( http_connection_t* connect, http_request_id_t *request, http_header_info_t* headers, http_frame_flags_t flags );

/**
 * Adds/sends data fields on a created request
 *
 * Adds/sends data fields on a created request.
 *
 * A Client should fill this with a request data ( set length to 0 if not valid).
 * A server responding to a received request should include a response body
 *
 * The  HTTP_REQUEST_FLAGS_FINISH_REQUEST should be set if the sent data is
 * the last part of the request.
 *
 * @param[in] connection     Structure workspace that will be used for this HTTP connection.
 * @param[in] request        Created request (stream ID).
 * @param[in] data           A pointer to a data buffer
 * @param[in] data_len       Length of the data
 * @param[in] flags          Flags associated with the request.
 *                           - HTTP_FRAME_FLAGS_END_STREAM: Set by server or client when
 *                             the data sent is the last part of the request/response
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_request_put_data ( http_connection_t* connect, http_request_id_t request, uint8_t* data, uint32_t data_len, http_frame_flags_t flags );

/**
 * Sends HTTP/2 PING frame to server
 *
 * @param[in] connection     Structure workspace that will be used for this HTTP connection.
 * @param[in] data           A pointer to a ping data payload
 * @param[in] flags          A bit masked parameter indicates request options.
 *                           - HTTP_FRAME_FLAGS_NONE: To send PING request
 *                           - HTTP_FRAME_FLAGS_ACK: To send PING response
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_ping_request ( http_connection_t* connect, uint8_t data[HTTP2_PING_FRAME_DATA_LENGTH], http_frame_flags_t flags );

#ifdef __cplusplus
} /* extern "C" */
#endif
