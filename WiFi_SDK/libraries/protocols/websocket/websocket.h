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
#include "websocket_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * NOTE:
 *
 * Current Limitations:
 *  - This implementation can only support receiving single frames on packet boundaries
 */

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SUB_PROTOCOL_STRING_LENGTH  10
#define SUB_EXTENSION_STRING_LENGTH 10

#define WEBSOCKET_CLOSE_FRAME_STATUS_CODE_LENGTH    2
#define WEBSOCKET_CLOSE_FRAME_MAX_REASON_LENGTH     123
#define WEBSOCKET_CLOSE_FRAME_BODY_MAX_LENGTH       ( WEBSOCKET_CLOSE_FRAME_STATUS_CODE_LENGTH + WEBSOCKET_CLOSE_FRAME_MAX_REASON_LENGTH )

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WEBSOCKET_UNINITIALISED = 0, /* The WebSocket in uninitialised */
    WEBSOCKET_INITIALISED,       /* The WebSocket in initialised */
    WEBSOCKET_CONNECTING,        /* The connection has not yet been established */
    WEBSOCKET_OPEN,              /* The WebSocket connection is established and communication is possible */
    WEBSOCKET_CLOSING,           /* The connection is going through the closing handshake */
    WEBSOCKET_CLOSED,            /* The connection has been closed or could not be opened */
} wiced_websocket_state_t;

/* Websocket Frame type values as per RFC */
typedef enum
{
    WEBSOCKET_CONTINUATION_FRAME = 0,
    WEBSOCKET_TEXT_FRAME,
    WEBSOCKET_BINARY_FRAME,
    WEBSOCKET_RESERVED_3,
    WEBSOCKET_RESERVED_4,
    WEBSOCKET_RESERVED_5,
    WEBSOCKET_RESERVED_6,
    WEBSOCKET_RESERVED_7,
    WEBSOCKET_CONNECTION_CLOSE,
    WEBSOCKET_PING,
    WEBSOCKET_PONG,
    WEBSOCKET_RESERVED_B,
    WEBSOCKET_RESERVED_C,
    WEBSOCKET_RESERVED_D,
    WEBSOCKET_RESERVED_E,
    WEBSOCKET_RESERVED_F
} wiced_websocket_frame_type_t;

typedef enum
{
    WEBSOCKET_NO_ERROR                              = 0,
    WEBSOCKET_CLIENT_CONNECT_ERROR                  = 1,
    WEBSOCKET_NO_AVAILABLE_SOCKET                   = 2,
    WEBSOCKET_SERVER_HANDSHAKE_RESPONSE_INVALID     = 3,
    WEBSOCKET_CREATE_SOCKET_ERROR                   = 4,
    WEBSOCKET_FRAME_SEND_ERROR                      = 5,
    WEBSOCKET_HANDSHAKE_SEND_ERROR                  = 6,
    WEBSOCKET_PONG_SEND_ERROR                       = 7,
    WEBSOCKET_RECEIVE_ERROR                         = 8,
    WEBSOCKET_DNS_RESOLVE_ERROR                     = 9,
    WEBSOCKET_SUBPROTOCOL_NOT_SUPPORTED             = 10,
    WEBSOCKET_ACCEPT_ERROR                          = 11
} wiced_websocket_error_t;

typedef enum
{
    WEBSOCKET_CLOSE_STATUS_NO_CODE                      = 0,

    WEBSOCKET_CLOSE_STATUS_CODE_NORMAL                  = 1000,
    WEBSOCKET_CLOSE_STATUS_CODE_GOING_AWAY,
    WEBSOCKET_CLOSE_STATUS_CODE_PROTOCOL_ERROR,
    WEBSOCKET_CLOSE_STATUS_CODE_DATA_NOT_ACCEPTED,
    WEBSOCKET_CLOSE_STATUS_CODE_RESERVED,
    WEBSOCKET_CLOSE_STATUS_CODE_RESERVED_NO_STATUS_CODE, // MUST NOT be set as a status code in Close Control frame
    WEBSOCKET_CLOSE_STATUS_CODE_RESERVED_ABNORMAL_CLOSE, // MUST NOT be set as a status code in Close Control frame
    WEBSOCKET_CLOSE_STATUS_CODE_DATA_INCONSISTENT,
    WEBSOCKET_CLOSE_STATUS_CODE_POLICY_VIOLATION,
    WEBSOCKET_CLOSE_STATUS_CODE_DATA_TOO_BIG,
    WEBSOCKET_CLOSE_STATUS_CODE_EXTENSION_EXPECTED,
    WEBSOCKET_CLOSE_STATUS_CODE_UNEXPECTED_CONDITION,
} wiced_websocket_close_status_code_t;

typedef enum
{
    WEBSOCKET_FRAME_FLAG_UNFRAGMENTED,
    /* Helper enums for applications to send fragmented frames of a particular type */
    WEBSOCKET_FRAME_FLAG_FRAGMENTED_FIRST,
    WEBSOCKET_FRAME_FLAG_CONTINUED,
    WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL,
} wiced_websocket_frame_flags_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef wiced_result_t (*wiced_websocket_callback_t)( void* websocket );
typedef wiced_result_t (*wiced_websocket_message_callback_t)( void* websocket, uint8_t* message, uint32_t length,
                        wiced_websocket_frame_type_t frame_type, wiced_websocket_frame_flags_t flags );

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_websocket_callback_t          on_open;
    wiced_websocket_callback_t          on_error;
    wiced_websocket_callback_t          on_close;
    wiced_websocket_message_callback_t  on_message;
} wiced_websocket_callbacks_t;

typedef struct
{
    const char*   url;
    const char**  protocols;
} wiced_websocket_url_protocol_entry_t;

/**
 *  Mostly in-line with W3C APIs
 *  https://html.spec.whatwg.org/multipage/comms.html#websocket
 */
typedef struct wiced_websocket
{
    wiced_websocket_core_t                      core;                                      //!< Mainly an abstracted tcp_socket
    wiced_websocket_error_t                     error_type;                                //!< Error-type reported on websocket
    wiced_websocket_state_t                     state;                                     //!< Websocket State
    char                                        subprotocol[SUB_PROTOCOL_STRING_LENGTH];   //!< 'subprotocol in use' when websocket connection is established
    wiced_websocket_callbacks_t                 callbacks;                                 //!< callbacks for websocket
    wiced_websocket_url_protocol_entry_t*       url_protocol_ptr;                          //!< 'url'-'protocols' entry for this websocket
    wiced_tcp_stream_t                          stream;                                    //!< stream object for the websocket
} wiced_websocket_t;

/**
 * Keeping this structure for backward compatibility.
 */
typedef struct
{
    char*       request_uri;
    char*       host;
    char*       origin;
    char*       sec_websocket_protocol;
} wiced_websocket_client_url_protocol_t;

typedef struct wiced_websocket_server wiced_websocket_server_t;

typedef struct
{
    uint8_t                                 count;
    wiced_websocket_url_protocol_entry_t*   entries;
} wiced_websocket_url_protocol_table_t;

typedef struct wiced_websocket_server_config
{
    uint8_t                                 max_connections;                //!< Maximum simultaneous websocket connections
    uint16_t                                heartbeat_duration;             //!< Duration for heart-beat; if 0, 'heart-beat' is disabled
    wiced_websocket_url_protocol_table_t*   url_protocol_table;             //!< Table of all url-protocol pairs supported by this server
    uint8_t*                                rx_frame_buffer;                //!< Rx Frame buffer provided by application to receive frames
    uint32_t                                frame_buffer_length;            //!< Maximum Length of the Rx Frame buffer
} wiced_websocket_server_config_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/**
 *
 *  @defgroup websocket          WebSocket
 *  @ingroup  ipcoms
 *
 * Communication functions for WebSocket protocol(both Client & Server)
 *
 * The WebSocket Protocol enables two-way communication between a client running untrusted code in
 * a controlled environment to a remote host that has opted-in to communication from that code.
 * Websocket is designed to supersede existing bidirectional communication technologies that use
 * HTTP as a transport layer to benefit from existing infrastructure. Refer to RFC #6455 for more
 * details on Websockets.
 *
 * Wiced Websockets APIs , can be broadly classified into the following:
 * - Websocket Server APIs to configure, start and stop Websocket server
 * - Websocket Client APIs to create a Websocket and connect/disconnect to the server
 * - Common APIs both for Server & Clients to send & receive frames on a Websocket handle.
 *
 *  @{
 */
/*****************************************************************************/

/** Perform opening handshake on port 80 with server and establish a connection. Called by Client only.
 *
 * @param[in] websocket            Websocket object
 * @param[in] url                  Server URL to be used for connection
 * @param[in] port                 Port number to open the connection on
 * @param[in] interface            interface to open the connection on
 *
 * @return @ref wiced_result_t
 *
 * @note                           For additional error information, check the wiced_websocket_error_t field
 *                                 of the wiced_websocket_t structure
 */
wiced_result_t wiced_websocket_connect( wiced_websocket_t* websocket, const wiced_websocket_client_url_protocol_t* url, uint16_t port, wiced_interface_t interface );

/** Perform opening handshake on port 443 with server and establish a connection. Called by Client only.
 *
 * @param[in] websocket            Websocket object
 * @param[in] config               Server URL to be used for connection
 * @param[in] tls_identity         TLS identity object
 * @param[in] port                 Port number to open the connection on
 * @param[in] interface            interface to open the connection on
 *
 *
 * @return @ref wiced_result_t
 *
 * @note                           For additional error information, check the wiced_websocket_error_t field
 *                                 of the  wiced_websocket_t structure
 */
wiced_result_t wiced_websocket_secure_connect( wiced_websocket_t* websocket, const wiced_websocket_client_url_protocol_t* url, wiced_tls_identity_t* tls_identity, uint16_t port, wiced_interface_t interface );

/** Send data to websocket end-point. Called by Server & Client both.
 *
 * @param[in] websocket            Websocket object
 * @param[in] data                 pointer to application data to send.
 * @param[in] length               length of application data to send
 * @param[in] frame_type           denotes the frame type
 * @param[in] flags                denotes whether it is a fragmented websocket frame(continuation/finish/start frame) or
 *                                 normal websocket frame.
 *
 * @return @ref wiced_result_t     returns WICED_SUCCESS when sent to remote successfully
 */
wiced_result_t wiced_websocket_send ( wiced_websocket_t* websocket, uint8_t* data, uint32_t length,
                                    wiced_websocket_frame_type_t frame_type, wiced_websocket_frame_flags_t flags );

/** Close and clean up websocket, and send close message to websocket server. Called by Server & Client both.
 *
 * @param[in] websocket             Websocket to close
 * @param[in] code                  Closing status code
 * @param[in] reason                Closing reason
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_websocket_close( wiced_websocket_t* websocket, const uint16_t code, const char* reason );

/** Register the on_open, on_close, on_message and on_error callbacks. Called by Client.
 *
 * @param[in] websocket                 websocket on which to register the callbacks
 * @param[in] on_open_callback          called on open websocket  connection
 * @param[in] on_close_callback         called on close websocket connection
 * @param[in] on_message_callback       called on websocket receive data
 * @param[in] on_error_callback         called on websocket error
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_websocket_register_callbacks ( wiced_websocket_t* websocket, wiced_websocket_callback_t on_open_callback, wiced_websocket_callback_t on_close_callback, wiced_websocket_message_callback_t on_message_callback, wiced_websocket_callback_t on_error ) ;

/** Un-Register the on_open, on_close, on_message and on_error callbacks for a given websocket
 *  Called by Client.
 *
 * @param[in] websocket                 websocket on which to unregister the callbacks
 *
 */
void wiced_websocket_unregister_callbacks ( wiced_websocket_t* websocket );


/** Initialise the websocket. Called by Client only.
 *
 * @param[in] websocket            websocket we are initialising
 * @param[in] rx_frame_buffer      Rx Frame buffer provided by application(to receive Websocket frames).
 *                                 Actual received Websocket Frame length & data pointer will be given to application
 *                                 on_message() callback.
 *
 * @param[in] frame_buffer_length  Rx Frame buffer Maximum length.
 *
 * @return @ref wiced_result_t     returns WICED_SUCCESS if library initializes the websocket successfully.
 */
wiced_result_t wiced_websocket_initialise( wiced_websocket_t* websocket, uint8_t* rx_frame_buffer, uint32_t frame_buffer_length );

/** Un-initialise the websoket and free memory allocated in creating sending buffers. Called by Client only.
 *
 * @param[in] websocket            websocket we are un-initialising
 *
 *
 * @return @ref wiced_result_t     returns WICED_SUCCESS if library un-initializes the websocket successfully.
 */
wiced_result_t wiced_websocket_uninitialise( wiced_websocket_t* websocket );

/** Initialise and start a Websocket server. Called by Server.
 *
 * @param[in] server               Websocket server we are starting
 * @param[in] config               Configuration like max-connections, heart-beat, URL, list-of-protocols
 * @param[in] callbacks            Callbacks for events received on the websockets(under this webserver)
 * @param[in] tls_identity         TLS identity object;if NULL, TLS is disabled
 * @param[in] port                 Port number for the websocket connection(default is 443/80 for secure/non-secure)
 * @param[in] data                 Pointer to user-data
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_websocket_server_start( wiced_websocket_server_t* server, wiced_websocket_server_config_t* config,
                                    wiced_websocket_callbacks_t* callbacks, wiced_tls_identity_t* tls_identity, uint16_t port, void* data );

/** Stop and uninitialise Websocket server. Called by Server.
 *
 * @param[in] server               Websocket server we are stopping
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_websocket_server_stop( wiced_websocket_server_t* server );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
