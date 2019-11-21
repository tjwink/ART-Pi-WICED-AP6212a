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
#include "websocket.h"
#include "websocket_handshake.h"
#include "wwd_debug.h"
#include "wiced_crypto.h"
#include "wiced_utilities.h"
#include "wwd_assert.h"
#include "wiced_tls.h"

/*  Websocket packet format
 *
 *   (MSB)
 *    0                   1                   2                   3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-------+-+-------------+-------------------------------+
 *   |F|R|R|R| opcode|M| Payload len |    Extended payload length    |
 *   |I|S|S|S|  (4)  |A|     (7)     |             (16/63)           |
 *   |N|V|V|V|       |S|             |   (if payload len==126/127)   |
 *   | |1|2|3|       |K|             |                               |
 *   +-+-+-+-+-------+-+-------------+ - - - - - - - - - - - - - - - +
 *   |     Extended payload length continued, if payload len == 127  |
 *   + - - - - - - - - - - - - - - - +-------------------------------+
 *   |                               |         Masking-key           |
 *   +-------------------------------+ - - - - - - - - - - - - - - - +<---------
 *   :    Masking-key (cont.)        |         Payload Data          :
 *   +---------------------------------------------------------------+ AREA TO
 *   :                          Payload data                         : MASK
 *   +---------------------------------------------------------------+<---------
 */

/******************************************************
 *                      Macros
 ******************************************************/

#define IF_ERROR_RECORD_AND_EXIT( GENERIC_ERROR, WEBSOCKET_ERROR )            \
    {                                                                         \
        if( GENERIC_ERROR != WICED_SUCCESS )                                  \
        {                                                                     \
            websocket->error_type=WEBSOCKET_ERROR;                            \
            if( websocket->callbacks.on_error != NULL )                       \
            {                                                                 \
                websocket->callbacks.on_error(websocket);                     \
            }                                                                 \
            unlock_websocket_mutex();                                         \
            return GENERIC_ERROR;                                             \
        }                                                                     \
    }

/******************************************************
 *                    Constants
 ******************************************************/

#define DEFAULT_WEBSOCKET_FRAME_TIMEOUT                 (2000)  /* In milliseconds */
#define WEBSOCKET_CONNECTION_TIMEOUT                    (10000) /* In milliseconds */

/* FIN: 1 bit, RSV[1|2|3]: 3 bits, Opcode: 4 bits, Mask: 1 bit, Payload-length: 7 bits  == 16-bits */
#define WEBSOCKET_FRAME_HEADER_CONTROL_FIELD_LENGTH     (2)

/* Websocket Frame Header Control Field Byte-0 Masks */

#define WEBSOCKET_FRAME_FIN_MASK                        (0x80)
#define WEBSOCKET_FRAME_RSV1_MASK                       (0x40)
#define WEBSOCKET_FRAME_RSV2_MASK                       (0x20)
#define WEBSOCKET_FRAME_RSV3_MASK                       (0x10)
#define WEBSOCKET_FRAME_RSV_ALL_MASK                    ( WEBSOCKET_FRAME_RSV1_MASK | WEBSOCKET_FRAME_RSV2_MASK \
                                                        | WEBSOCKET_FRAME_RSV3_MASK)
#define WEBSOCKET_FRAME_OPCODE_MASK                     (0x0F)

/* Websocket Frame Header Control Field Byte-1 Masks */
#define WEBSOCKET_FRAME_MASKING_KEY_PRESENT_MASK        (0x80)
#define WEBSOCKET_FRAME_PAYLOAD_LENGTH_MASK             (0x7F)

/* Maximum extended payload length when 7-bit payload-length > 125(in bytes) */
#define WEBSOCKET_FRAME_EXTENDED_PAYLOAD_MAXIMUM_LENGTH (8)

/* If Masking-Key-present bit is set, length of the mask(in bytes) */
#define WEBSOCKET_FRAME_MASKING_KEY_LENGTH              (4)

/* Sum of above length fields */
#define WEBSOCKET_FRAME_MAXIMUM_HEADER_LENGTH            ( ( WEBSOCKET_FRAME_HEADER_CONTROL_FIELD_LENGTH \
                                                         + WEBSOCKET_FRAME_EXTENDED_PAYLOAD_MAXIMUM_LENGTH \
                                                         + WEBSOCKET_FRAME_MASKING_KEY_LENGTH ) )
/* Application payload is 0-125 bytes */
#define WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_7_BITS    (125)

/* Application payload is >125 bytes but less than 65535 bytes(uint16_t); Payload-len(7-bit) = 126 */
#define WEBSOCKET_FRAME_PAYLOAD_LENGTH_2_BYTES           (126)
#define WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_2_BYTES   ( UINT16_MAX - 1)

/* Application payload is >65536 bytes but less than 8 bytes(uint64_t); Payload-len(7-bit) = 127
 * However, we restrict this length to uint32_t i.e. 4294967295 bytes.
 */
#define WEBSOCKET_FRAME_PAYLOAD_LENGTH_8_BYTES           (127)
#define WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_8_BYTES   ( UINT32_MAX - 1 )

#define WEBSOCKET_FRAME_CONTROL_FRAME_MAXIMUM_LENGTH     (WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_7_BITS)


#define RSV_ERR_SHUTDOWN_MESSAGE              "rsv bits/extension negotiation is not supported"
#define CLOSE_REQUEST_SHUTDOWN_MESSAGE        "close requested from Server"
#define LENGTH_SHUTDOWN_MESSAGE               "max Frame length supported is 1024"
#define PAYLOAD_TYPE_ERROR_SHUTDOWN_MESSAGE   "unsupported payload type"

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    MUTEX_UNINITIALISED,
    MUTEX_LOCKED,
    MUTEX_UNLOCKED
}websocket_mutex_state_t;
/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum
{
    READ_CONTROL_BYTES,
    READ_LENGTH,
    READ_PAYLOAD,
    READ_COMPLETED_SUCCESSFULLY,
    READ_FRAME_ERROR
} wiced_websocket_read_frame_state_t;

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_mutex_t           handshake_lock;
    websocket_mutex_state_t mutex_state;
} wiced_websocket_mutex_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t websocket_common_connect     ( wiced_websocket_t* websocket, const wiced_websocket_client_url_protocol_t* url, wiced_tls_identity_t* tls_identity, uint16_t port, wiced_interface_t interface );
static wiced_result_t websocket_connect            ( wiced_websocket_t* websocket, wiced_ip_address_t* address, uint16_t port, wiced_interface_t interface );
static wiced_result_t websocket_tls_connect        ( wiced_websocket_t* websocket, wiced_ip_address_t* address, wiced_tls_identity_t* tls_identity, uint16_t port, wiced_interface_t interface );
static wiced_result_t mask_unmask_frame_data       ( uint8_t* data_in, uint8_t* data_out, uint32_t data_length, uint8_t* mask );
static wiced_result_t update_socket_state          ( wiced_websocket_t* websocket, wiced_websocket_state_t state);
static wiced_result_t on_websocket_close_callback  ( wiced_tcp_socket_t* socket, void* websocket );
static wiced_result_t on_websocket_message_callback( wiced_tcp_socket_t* socket, void* websocket );
static void           lock_websocket_mutex         ( void );
static void           unlock_websocket_mutex       ( void );
#if 0
static void           handle_frame_error           ( wiced_websocket_t* websocket, const char* close_string );
static void           handle_ping_frame            ( wiced_websocket_t* websocket, wiced_websocket_frame_t* rx_frame );
static void           handle_pong_frame            ( wiced_websocket_t* websocket, wiced_websocket_frame_t* rx_frame );
static void           handle_close_connection_frame( wiced_websocket_t* websocket, wiced_websocket_frame_t* rx_frame );
#endif
static wiced_result_t websocket_send_common        ( wiced_websocket_t* websocket, uint8_t* data, uint32_t length,
                                                        wiced_websocket_frame_type_t frame_type, wiced_websocket_frame_flags_t flags );

static wiced_result_t  websocket_send_stream        ( wiced_websocket_t* websocket, uint8_t* header, uint8_t header_length, uint8_t* data,
                                                        uint32_t length, uint8_t* mask );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_websocket_mutex_t     websocket_mutex =
{
    .mutex_state = MUTEX_UNINITIALISED
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static uint8_t check_reserved_flag( uint8_t header )
{
    return( header & ( WEBSOCKET_FRAME_RSV_ALL_MASK ) );
}

static wiced_bool_t verify_close_code( const uint16_t code )
{
    if( (code <= WEBSOCKET_CLOSE_STATUS_CODE_UNEXPECTED_CONDITION && code >= WEBSOCKET_CLOSE_STATUS_CODE_NORMAL)
        && (code != WEBSOCKET_CLOSE_STATUS_CODE_RESERVED_NO_STATUS_CODE)
        && (code != WEBSOCKET_CLOSE_STATUS_CODE_RESERVED_ABNORMAL_CLOSE) )
    {
        return WICED_TRUE;
    }

    /* code == 0 is a special value when applications just wants to send a connection close frame
     * without any body ==> 'reason' string will be discarded too
     */
    if( code == WEBSOCKET_CLOSE_STATUS_NO_CODE )
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

static wiced_bool_t verify_close_reason( const char* reason )
{
    if( strlen(reason ) > WEBSOCKET_CLOSE_FRAME_MAX_REASON_LENGTH )
    {
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

static wiced_result_t mask_unmask_frame_data( uint8_t* data_in, uint8_t* data_out, uint32_t data_length, uint8_t* mask )
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t       i;

    for ( i = 0; i < data_length; i++ )
    {
        data_out[ i ] = data_in[ i ] ^ mask[ i % 4 ];
    }

    return result;
}

static uint8_t is_valid_websocket_frame_type( wiced_websocket_frame_type_t type )
{
    if ( ( ( type >= WEBSOCKET_RESERVED_3 ) && ( type <= WEBSOCKET_RESERVED_7 ) ) || ( ( type >= WEBSOCKET_RESERVED_B ) ) )
    {
        return 0;
    }

    return 1;
}

static uint8_t match_websocket_state( wiced_websocket_t* websocket, wiced_websocket_state_t state )
{
    if( websocket->state != state )
    {
        return 0;
    }
    return 1;
}

static uint8_t is_control_frame( wiced_websocket_frame_type_t frame_type )
{
    if( frame_type == WEBSOCKET_PONG || frame_type == WEBSOCKET_PING || frame_type == WEBSOCKET_CONNECTION_CLOSE )
        return 1;
    return 0;
}

static wiced_result_t websocket_core_initialise( wiced_websocket_core_t* websocket, wiced_websocket_role_t role, uint8_t* rx_buffer, uint32_t length )
{
    if( !websocket || role > WEBSOCKET_ROLE_SERVER || !rx_buffer || !length )
    {
        return WICED_BADARG;
    }
    /* if socket is Non-NULL here; then probably the application has not uninitialized the websocket;
     * just report this erroneous state to the user */
    if( websocket->socket )
    {
        return WICED_ERROR;
    }

    websocket->socket               = malloc(sizeof(wiced_tcp_socket_t));
    if( !websocket->socket )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    websocket->role                 = role;
    websocket->receive.frame_buffer = rx_buffer;
    websocket->receive.length       = length;
    websocket->receive.continuation = 0;

    return WICED_SUCCESS;
}

static wiced_result_t websocket_core_uninitialise( wiced_websocket_core_t* websocket )
{
    if( !websocket )
    {
        return WICED_BADARG;
    }

    if( websocket->socket != NULL )
    {
        free(websocket->socket);
    }

    websocket->socket = NULL;

    return WICED_SUCCESS;
}

wiced_result_t wiced_websocket_initialise( wiced_websocket_t* websocket, uint8_t* rx_frame_buffer, uint32_t length )
{
    wiced_result_t result = WICED_SUCCESS;
    if( !websocket || !rx_frame_buffer || !length )
    {
        return WICED_BADARG;
    }

    result = websocket_core_initialise(&websocket->core, WEBSOCKET_ROLE_CLIENT, rx_frame_buffer, length );
    if( result!= WICED_SUCCESS )
    {
        return result;
    }

    websocket->state            = WEBSOCKET_INITIALISED;
    websocket->url_protocol_ptr = NULL;
    // websocket->subprotocol      = { 0x0 };
    return result;
}

wiced_result_t wiced_websocket_uninitialise( wiced_websocket_t* websocket )
{
    if( !websocket )
    {
        return WICED_BADARG;
    }

    websocket->state = WEBSOCKET_UNINITIALISED;

    return websocket_core_uninitialise(&websocket->core);
}

wiced_result_t wiced_websocket_connect( wiced_websocket_t* websocket, const wiced_websocket_client_url_protocol_t* url_entry, uint16_t port, wiced_interface_t interface )
{

    if ( websocket_common_connect( websocket, url_entry, NULL, port, interface ) != WICED_SUCCESS )
    {
        wiced_websocket_close( websocket, WEBSOCKET_CLOSE_STATUS_NO_CODE, NULL );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_websocket_secure_connect( wiced_websocket_t* websocket, const wiced_websocket_client_url_protocol_t* url_entry, wiced_tls_identity_t* tls_identity, uint16_t port, wiced_interface_t interface )
{
    if ( websocket_common_connect( websocket, url_entry, tls_identity, port, interface ) != WICED_SUCCESS )
    {
        wiced_websocket_close( websocket, WEBSOCKET_CLOSE_STATUS_NO_CODE, NULL );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

#if 0
static wiced_result_t parse_url( wiced_websocket_url_protocol_entry_t* url_entry, wiced_websocket_header_fields_t* header, wiced_websocket_role_t role )
{
    char* url_start_ptr;

    wiced_bool_t secure_uri = WICED_FALSE;
    uint8_t uri_header_length;
    uint8_t uri_host_length;
    uint8_t uri_port_length;
    uint8_t uri_path_length;
    uint8_t uri_query_length;

    char* host_start_ptr;
    char* port_start_ptr;
    char* path_start_ptr;
    char* query_start_ptr;


    /* FIXME: make it case-insensitive */
    char ws_uri[] = "ws://";
    char wss_uri[] = "wss://";

    if( !url_entry || !header || !(url_entry->url) )
    {
        return WICED_ERROR;
    }

    url_start_ptr = url_entry->url;

    if( memcmp( url_start_ptr, ws_uri, sizeof(ws_uri) ) != 0 )
    {
        if( memcmp( url_start_ptr, wss_uri, sizeof(wss_uri) ) != 0 )
        {
            return WICED_BADARG;
        }
        else
        {
            secure_uri = WICED_TRUE;
        }
    }

    if( secure_uri )
    {
        uri_header_length = sizeof(wss_uri);
    }
    else
    {
        uri_header_length = sizeof(ws_uri);
    }

    host_start_ptr = url_start_ptr + uri_header_length;

    /* first look for [":" port ] field */
    port_start_ptr = strstr( host_start_ptr, ':')

    if( port_start_ptr == NULL )
    {
        /* port is not provided; take default values for port and look for first '/' as end of 'host' */
        first_slash_ptr = strstr(host_start_ptr, '/');
        if( first_slash_ptr == NULL )
        {
            /* only |host| available, no path or query is provided */
            uri_host_length = strlen (url_start_ptr) - uri_header_length;

            header->host = malloc( uri_host_length + 1 );
            memset( &header->host, 0, uri_host_length + 1 );

            memcpy( &header->host, host_start_ptr, uri_host_length );

            uri_path_length = 2;
            /* Assumption here is that URI won't have query component if 'path' is empty */
            header->request_uri = malloc( uri_path_length );
            strncpy(&header->request_uri, "/", uri_path_length );
        }
        else
        {
            uri_host_length = first_slash_ptr - host_start_ptr;
            header->host = malloc( uri_host_length + 1 );
            memset( &header->host, 0, uri_host_length + 1 );

            memcpy( &header->host, host_start_ptr, uri_host_length );
        }
    }
    else
    {
        uri_host_length = port_start_ptr - host_start_ptr;
        /* port is provided; see if it matches default ports or not */
        first_slash_ptr = strstr(port_start_ptr, '/');
        if( first_slash_ptr == NULL )
        {
            /* 'path' is empty */
        }

    }

    if( url_entry->sec_websocket_protocols != NULL )
    {

    }

    if( role == WEBSOCKET_ROLE_CLIENT )
    {
        WPRINT_LIB_INFO( ("==== Websocket Header fields ====\n") );
        WPRINT_LIB_INFO( ("\tHost: %s\r\nRequest_uri: %s\r\n"))

    }

}

static void set_local_header_fields( wiced_websocket_t* websocket, const wiced_websocket_url_protocol_entry_t* url_entry )
{
    wiced_websocket_header_fields_t* local = &websocket->core.local;

    //parse_url( url_entry.url, local, websocket->core.role );
#if 1 // Hard-coding
    if( websocket->core.role == WEBSOCKET_ROLE_CLIENT )
    {
        local->host                     = "192.168.1.2";
        local->request_uri              = "?encoding=text";
        local->origin                   = client->origin;
        local->sec_websocket_protocols  = client->sec_websocket_protocols;
    }
    else
    {
        wiced_websocket_server_config_t* server = (wiced_websocket_server_config_t* )config;
        local->sec_websocket_protocols = server->sec_websocket_protocols;
        local->host                    = server->host;
    }
#endif
}
#endif

static void reset_client_handshake_fields( wiced_websocket_handshake_fields_t* client_handshake )
{
    websocket_client_handshake_fields_t* client = NULL;

    if( client_handshake == NULL )
    {
        WPRINT_LIB_INFO(("Handshake fields already NULL?\n"));
        return;
    }

    client = &client_handshake->client;

    if( client->host )
    {
        free(client->host);
    }

    if( client-> port )
    {
        free(client->port);
    }

    if( client->resource_name )
    {
        free(client->resource_name);
    }

    if( client->origin )
    {
        free(client->origin);
    }

    if( client->protocols )
    {
        free(client->protocols);
    }

    if( client->sec_websocket_key )
    {
        free(client->sec_websocket_key);
    }

    if( client->sec_websocket_version )
    {
        free(client->sec_websocket_version);
    }
    return;
}

static wiced_result_t set_client_handshake_fields( wiced_websocket_handshake_fields_t* client_handshake, const wiced_websocket_client_url_protocol_t* url_entry )
{
    uint8_t nr_bytes = 0;

    websocket_client_handshake_fields_t* client = &client_handshake->client;

    client->host                    = NULL;
    client->port                    = NULL;
    client->resource_name           = NULL;
    client->origin                  = NULL;
    client->protocols               = NULL;
    client->sec_websocket_key       = NULL;
    client->sec_websocket_version   = NULL;

    if( url_entry->host == NULL || url_entry->request_uri == NULL )
    {
        return WICED_BADARG;
    }

    /* extract 'host' field */
    nr_bytes = (uint8_t)(strlen(url_entry->host) + 1 );

    client->host = malloc( nr_bytes );

    if(client->host == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    memcpy(client->host, url_entry->host, nr_bytes );

    /* extract 'request_uri' field */
    nr_bytes = (uint8_t)(strlen(url_entry->request_uri) + 1 );

    client->resource_name = malloc( nr_bytes );

    if(client->resource_name == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    memcpy(client->resource_name, url_entry->request_uri, nr_bytes );

    /* extract 'origin' field if available */
    if( url_entry->origin != NULL )
    {
        nr_bytes = (uint8_t)(strlen(url_entry->origin) + 1);

        client->origin = malloc( nr_bytes );
        if(client->origin == NULL )
        {
            return WICED_OUT_OF_HEAP_SPACE;
        }

        memcpy(client->origin, url_entry->origin, nr_bytes );
    }
    /* extract 'sec_websocket_protocol' field if available */
    if( url_entry->sec_websocket_protocol != NULL )
    {
        nr_bytes = (uint8_t)(strlen(url_entry->sec_websocket_protocol) + 1);

        client->protocols = malloc( nr_bytes );
        if(client->protocols == NULL )
        {
            return WICED_OUT_OF_HEAP_SPACE;
        }

        memcpy(client->protocols, url_entry->sec_websocket_protocol, nr_bytes );
    }

    return WICED_SUCCESS;
}

static wiced_result_t websocket_common_connect( wiced_websocket_t* websocket, const wiced_websocket_client_url_protocol_t* url_entry, wiced_tls_identity_t* tls_identity, uint16_t port, wiced_interface_t interface )
{
    wiced_result_t     result = WICED_SUCCESS;
    wiced_ip_address_t address;
    wiced_tcp_socket_t* tcp_socket = NULL;
    wiced_websocket_handshake_fields_t handshake;

    if( websocket == NULL )
    {
        return result;
    }

    /* as per rfc 6455 specification, there can not be more then one socket in a connecting state*/
    lock_websocket_mutex();

    /* FIXME: First expect a Server IP address. If it fails, try hostname_lookup.
     * If none works, bail out. This logic need to be improved but as of now collision of a valid
     * str_to_ip(ip-address) and a valid hostname is minimal. And this approach
     * avoids adding another flag.
     */
    if( str_to_ip(url_entry->host, &address) != 0 )
    {
        WPRINT_LIB_INFO( ("Not a valid IP address?? Trying hostname lookup...\n") );
        result = wiced_hostname_lookup( url_entry->host, &address, WEBSOCKET_CONNECTION_TIMEOUT, interface );
        if( result != WICED_SUCCESS )
        {
            WPRINT_LIB_INFO( ("Host-name lookup failed as well ! bailing out...\n") );
        }
    }

    IF_ERROR_RECORD_AND_EXIT( result, WEBSOCKET_DNS_RESOLVE_ERROR);

    update_socket_state( websocket, WEBSOCKET_CONNECTING );

    /* Establish secure or non secure tcp connection to server */
    if ( tls_identity != NULL )
    {
        result = websocket_tls_connect( websocket, &address, tls_identity, port, interface );
    }
    else
    {
        result = websocket_connect( websocket, &address, port, interface );
    }

    IF_ERROR_RECORD_AND_EXIT( result, WEBSOCKET_CLIENT_CONNECT_ERROR );

    /* FIXME: Extract Client hadnshake fields from url_entry earlier as part of parse_url().
     * This need to be made better.
     */
    result = set_client_handshake_fields( &handshake, url_entry );

    IF_ERROR_RECORD_AND_EXIT( result, WEBSOCKET_CLIENT_CONNECT_ERROR );

    result = wiced_establish_websocket_handshake( websocket, &handshake );

    IF_ERROR_RECORD_AND_EXIT( result, WEBSOCKET_SERVER_HANDSHAKE_RESPONSE_INVALID );

    reset_client_handshake_fields(&handshake);

    /* If a sub protocol was requested, extract it from the handshake */
    if ( url_entry->sec_websocket_protocol != NULL )
    {
        result = wiced_get_websocket_subprotocol( websocket->subprotocol );
        IF_ERROR_RECORD_AND_EXIT( result, WEBSOCKET_SUBPROTOCOL_NOT_SUPPORTED );
    }

    update_socket_state( websocket, WEBSOCKET_OPEN );
    if( websocket->callbacks.on_open != NULL )
    {
        websocket->callbacks.on_open( websocket );
    }

    GET_TCP_SOCKET_FROM_WEBSOCKET((tcp_socket),(websocket))
    /*Once the communication with server is established, register for the message callback and close callback */
    wiced_tcp_register_callbacks( tcp_socket, NULL, on_websocket_message_callback, on_websocket_close_callback, websocket );

    unlock_websocket_mutex();

    return result;
}

static wiced_result_t websocket_tls_connect( wiced_websocket_t* websocket, wiced_ip_address_t* address, wiced_tls_identity_t*  tls_identity, uint16_t port, wiced_interface_t interface )
{
    wiced_result_t result;
    wiced_tcp_socket_t* tcp_socket = NULL;
    wiced_tls_context_t* context = NULL;

    GET_TCP_SOCKET_FROM_WEBSOCKET((tcp_socket), (websocket))
    if( tls_identity != NULL )
    {
        context = malloc_named("wss", sizeof(wiced_tls_context_t));

        if( context == NULL )
        {
            return WICED_OUT_OF_HEAP_SPACE;
        }

        result = wiced_tls_init_context( context, tls_identity, NULL );
        if( result != WICED_SUCCESS || context == NULL )
        {
            WPRINT_LIB_INFO( ("[%s] Error Initializing TLS context\n",__func__) );
            return result;
        }
    }

    WICED_VERIFY( wiced_tcp_create_socket( tcp_socket, interface ) );

    /* setup the stream object */
    wiced_tcp_stream_init(&websocket->stream, tcp_socket);

    result = wiced_tcp_enable_tls( tcp_socket, context );
    if( result != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO( ("[%s] Error Enabling TLS for TCP socket\n",__func__) );
        return result;
    }

    result = wiced_tcp_connect( tcp_socket, address, port, WEBSOCKET_CONNECTION_TIMEOUT );
    if ( result != WICED_SUCCESS )
    {
        websocket->error_type = WEBSOCKET_CLIENT_CONNECT_ERROR;
        if( websocket->callbacks.on_error != NULL )
        {
            websocket->callbacks.on_error( websocket );
        }
        WPRINT_LIB_INFO(("[%s] Error[%d] connecting @websocket: %p\n", __func__, result, (void *)websocket) );
        wiced_tcp_delete_socket( tcp_socket );
    }

    return result;
}

static wiced_result_t websocket_connect( wiced_websocket_t* websocket, wiced_ip_address_t* address, uint16_t port, wiced_interface_t interface )
{
    wiced_result_t result;
    wiced_tcp_socket_t* tcp_socket = NULL;

    GET_TCP_SOCKET_FROM_WEBSOCKET((tcp_socket), (websocket))

    if( tcp_socket == NULL || websocket->core.role == WEBSOCKET_ROLE_SERVER )
    {
        return WICED_ERROR;
    }

    WICED_VERIFY( wiced_tcp_create_socket( tcp_socket, interface ) );

    result = wiced_tcp_connect( tcp_socket, address, port, WEBSOCKET_CONNECTION_TIMEOUT );
    if ( result != WICED_SUCCESS )
    {
        websocket->error_type = WEBSOCKET_CLIENT_CONNECT_ERROR;
        if( websocket->callbacks.on_error != NULL )
        {
            websocket->callbacks.on_error( websocket );
        }
        WPRINT_LIB_INFO(("[%s] Error[%d] connecting @websocket: %p\n", __func__, result, (void *)websocket) );
        wiced_tcp_delete_socket( tcp_socket );
        return result;
    }

    wiced_tcp_stream_init( &websocket->stream, tcp_socket );

    return result;
}

static wiced_result_t websocket_send_common( wiced_websocket_t* websocket, uint8_t* data, uint32_t length,
                        wiced_websocket_frame_type_t frame_type, wiced_websocket_frame_flags_t flags )
{
    wiced_result_t result = WICED_ERROR;
    uint8_t additional_bytes_to_represent_length;
    uint8_t masking_key_offset;
    uint8_t header_length = 0;
    uint8_t mask[WEBSOCKET_FRAME_MASKING_KEY_LENGTH] = { 0 };
    uint8_t* mask_ptr = NULL;
    uint8_t do_masking = 0;
    uint8_t is_final = 0;

    /* header memory  -- see if this should be malloced instead */
    uint8_t websocket_frame_header[WEBSOCKET_FRAME_MAXIMUM_HEADER_LENGTH] = { 0 };
    wiced_websocket_frame_type_t final_frame_type = frame_type;

    if( websocket->core.role == WEBSOCKET_ROLE_CLIENT )
    {
        do_masking = 1;
    }

    /* Set FIN, RSV and opcode fields of Byte 0 */
    if( flags == WEBSOCKET_FRAME_FLAG_UNFRAGMENTED || flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL )
    {
        is_final = (uint8_t) WEBSOCKET_FRAME_FIN_MASK;
    }

    if( flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL || flags == WEBSOCKET_FRAME_FLAG_CONTINUED )
    {
        final_frame_type = WEBSOCKET_CONTINUATION_FRAME;
    }

    websocket_frame_header[ 0 ] =  (uint8_t) ( is_final | ( ( final_frame_type & WEBSOCKET_FRAME_OPCODE_MASK ) ) );

    /* 2 bytes of header-length is standard */
    header_length = (uint8_t) ( header_length + WEBSOCKET_FRAME_HEADER_CONTROL_FIELD_LENGTH );

    if( do_masking  )
    {
        websocket_frame_header[1] |= WEBSOCKET_FRAME_MASKING_KEY_PRESENT_MASK;
    }

    if ( length <= WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_7_BITS )
    {
        /* payload length is represented with 7-bits, and extended payload bytes are not in use */
        websocket_frame_header[ 1 ] = (uint8_t) ( websocket_frame_header[1] | ( (uint8_t) ( WEBSOCKET_FRAME_PAYLOAD_LENGTH_MASK & length ) ) );

        additional_bytes_to_represent_length = 0;
    }
    else if( length <= WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_2_BYTES )
    {
        websocket_frame_header[ 1 ] = (uint8_t) ( websocket_frame_header[1] | (uint8_t) ( WEBSOCKET_FRAME_PAYLOAD_LENGTH_MASK & WEBSOCKET_FRAME_PAYLOAD_LENGTH_2_BYTES ) );
        websocket_frame_header[ 2 ] = (uint8_t) ( length >> 8 );
        websocket_frame_header[ 3 ] = (uint8_t) ( length );
        additional_bytes_to_represent_length = 2;
    }
    else
    {
        websocket_frame_header[ 1 ] = (uint8_t)(websocket_frame_header[1] | (uint8_t) ( WEBSOCKET_FRAME_PAYLOAD_LENGTH_MASK & WEBSOCKET_FRAME_PAYLOAD_LENGTH_8_BYTES ) );
        /* Note: Any payload length higher than pow(2,32) is not supported..However it will still take 8 bytes  */
        websocket_frame_header[ 2 ] = 0;
        websocket_frame_header[ 3 ] = 0;
        websocket_frame_header[ 4 ] = 0;
        websocket_frame_header[ 5 ] = 0;
        websocket_frame_header[ 6 ] = (uint8_t) ( length >> 24 );
        websocket_frame_header[ 7 ] = (uint8_t) ( length >> 16 );
        websocket_frame_header[ 8 ] = (uint8_t) ( length >> 8 );
        websocket_frame_header[ 9 ] = (uint8_t) ( length );
        additional_bytes_to_represent_length = 8;
    }

    /* account for 'extended payload length' bytes  in header-length */
    header_length = (uint8_t) (header_length +  additional_bytes_to_represent_length );

    if( do_masking )
    {
        masking_key_offset = header_length;
        /* Generate random mask to be used in masking the frame data*/
        wiced_crypto_get_random( mask, WEBSOCKET_FRAME_MASKING_KEY_LENGTH );

        memcpy(&websocket_frame_header[masking_key_offset], mask, WEBSOCKET_FRAME_MASKING_KEY_LENGTH);
        /* account for masking bytes  in header-length */
        header_length = (uint8_t)( header_length + WEBSOCKET_FRAME_MASKING_KEY_LENGTH );
        mask_ptr = mask;
    }

    result = websocket_send_stream( websocket, websocket_frame_header, header_length, data, length, mask_ptr );

    IF_ERROR_RECORD_AND_EXIT( result, WEBSOCKET_FRAME_SEND_ERROR );

    return result;
}

static wiced_result_t websocket_send_stream( wiced_websocket_t* websocket, uint8_t* header, uint8_t header_length, uint8_t* data, uint32_t length, uint8_t* mask )
{
    wiced_tcp_stream_t* tcp_stream = &websocket->stream;

    /*  send the header first un-masked on the stream */
    wiced_tcp_stream_write(tcp_stream, header, header_length );

    if( mask )
    {
        /* if 'mask' is available, first invocation will mask the data in-place */
        mask_unmask_frame_data(data, data, length, mask);
    }

    /* now send the data */
    wiced_tcp_stream_write(tcp_stream, data, length );

    wiced_tcp_stream_flush(tcp_stream);

    if( mask )
    {
        /* now second invocation will unmask the data in-place; so that it returns to the application without any changes */
        mask_unmask_frame_data(data, data, length, mask);
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_websocket_send( wiced_websocket_t* websocket, uint8_t* data, uint32_t length,
                        wiced_websocket_frame_type_t frame_type, wiced_websocket_frame_flags_t flags )
{
    wiced_result_t result = WICED_ERROR;

    /* data can be NULL or length could be zero for frames like connection-close or ping/pong */
    if( websocket == NULL )
    {
        return WICED_BADARG;
    }

    /* check if websocket is in 'OPEN' state to send data or not */
    if( !match_websocket_state( websocket, WEBSOCKET_OPEN) )
    {
        return result;
    }

    /* check for erroneous payload type */
    if( !is_valid_websocket_frame_type(frame_type) )
    {
        return result;
    }

    /* Control Frames MUST not be fragmented and hence MUST be less than 125 bytes of payload */
    if( ( (flags != WEBSOCKET_FRAME_FLAG_UNFRAGMENTED) || (length > WEBSOCKET_FRAME_CONTROL_FRAME_MAXIMUM_LENGTH) ) && is_control_frame( frame_type ) )
    {
        return result;
    }

    return websocket_send_common( websocket, data, length, frame_type, flags );
}

wiced_result_t websocket_receive_frame( wiced_websocket_t* websocket )
{
    wiced_result_t result = WICED_SUCCESS;
    /* TODO: We can declare a header structure instead of a byte-array to parse headers */
    uint8_t header[WEBSOCKET_FRAME_MAXIMUM_HEADER_LENGTH] = { 0 };
    uint8_t final_frame = 0;
    wiced_websocket_frame_type_t type = 0;
    uint8_t payload_length = 0;
    uint8_t is_masked = 0;
    uint32_t data_length = 0;
    uint32_t received_bytes = 0;
    uint32_t offset = 0;
    uint8_t mask[4]  = { 0 };
    wiced_websocket_frame_type_t frame_type;
    wiced_websocket_frame_flags_t   flags;
    //wiced_result_t result;
    uint8_t bytes_to_read = 0;

    uint8_t* buffer = websocket->core.receive.frame_buffer;
    uint32_t max_buffer_length = websocket->core.receive.length;
    wiced_tcp_stream_t* stream = &websocket->stream;

    offset = 0;

    /* first fetch two bytes of common header from the stream */
    result = wiced_tcp_stream_read_with_count( stream, header, WEBSOCKET_FRAME_HEADER_CONTROL_FIELD_LENGTH, 0, &received_bytes );
    if( result != WICED_SUCCESS || received_bytes != WEBSOCKET_FRAME_HEADER_CONTROL_FIELD_LENGTH )
    {
        if( result == WICED_TIMEOUT && received_bytes == 0)
        {
            /* Receiving data using stream APIs will consume packets if they are available.
               However, each packet received enqueues an async notifcation. These notifications will be dummy if
               we read the new packets as part of ongoing receive_frame(). For example - Websocket Frame payload length
               is more than TCP segment size. Hence ignore the dummy receive_frame calls for these async notifications.
            */
            return WICED_SUCCESS;
        }
        WPRINT_LIB_INFO(("Error receiving data from stream(received:%d bytes) result:%d \n",(int)received_bytes, (int)result ) );
        return WICED_ERROR;
    }

    /* if RSVD bits are set; report the error */
    if( check_reserved_flag(header[0]) )
    {
        WPRINT_LIB_INFO(("Error: Reserved flag bit are set\n"));
        return WICED_ERROR;
    }

    type = (wiced_websocket_frame_type_t) (header[0] & WEBSOCKET_FRAME_OPCODE_MASK );
    if( !is_valid_websocket_frame_type(type) )
    {
        WPRINT_LIB_INFO(("Error: Invalid Payload type\n"));
        return WICED_ERROR;
    }

    is_masked = header[1] & WEBSOCKET_FRAME_MASKING_KEY_PRESENT_MASK;
    if( websocket->core.role == WEBSOCKET_ROLE_CLIENT && is_masked )
    {
        /* if we receive a 'masked' frame from a websocket server, don't process it */
        WPRINT_LIB_INFO(("Error: Invalid masking from Server data\n"));
        return WICED_ERROR;
    }

    /* check for final frame */
    final_frame = (header[0] & WEBSOCKET_FRAME_FIN_MASK) ? 1 : 0;

    /* if it is a CONTINUE frame(either FIN set or clear) and if we are not expecting to receive continuation frames
     * then report error
     */
    if( type == WEBSOCKET_CONTINUATION_FRAME && !websocket->core.receive.continuation )
    {
        WPRINT_LIB_INFO(("Error: A CONTINUE frame is received while socket is not expecting so\n"));
        return WICED_ERROR;
    }
    /* if we are expecting Continuation frames(i.e. First fragmented frame with valid text/binary opcode had been received)
     * and new frame is neither CONTINUE or CONTROL frames; report error
     */
    if( websocket->core.receive.continuation && !( is_control_frame(type) || type == WEBSOCKET_CONTINUATION_FRAME) )
    {
        WPRINT_LIB_INFO(("Error: Socket expecting CONTINUE frames and got type:%d\n", type ) );
        return WICED_ERROR;
    }

    /* account for payload-length variations */
    payload_length = ( header[1] & WEBSOCKET_FRAME_PAYLOAD_LENGTH_MASK );

    if( payload_length <= WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_7_BITS )
    {
        /* do nothing */
    }
    else if( payload_length == WEBSOCKET_FRAME_PAYLOAD_LENGTH_2_BYTES )
    {
        /* read the 2-bytes 'payload-length' */
        bytes_to_read = (uint8_t)( bytes_to_read + 2 );
    }
    else if( payload_length == WEBSOCKET_FRAME_PAYLOAD_LENGTH_8_BYTES )
    {
        /* read the 8-bytes 'payload-length' */
        bytes_to_read = (uint8_t)( bytes_to_read + 8 );
    }

    /* account for 'mask-key' length */
    if( is_masked )
    {
        bytes_to_read = (uint8_t)( bytes_to_read + WEBSOCKET_FRAME_MASKING_KEY_LENGTH );
    }

    /* increase the read offset */
    offset += WEBSOCKET_FRAME_HEADER_CONTROL_FIELD_LENGTH;

    /* it is possible that the frame has only 2-byte header, don't read further in such scenario */
    if( bytes_to_read != 0 )
    {
        received_bytes = 0;
        result = wiced_tcp_stream_read_with_count( stream, header + offset, bytes_to_read, 0, &received_bytes );
        if( received_bytes != bytes_to_read )
        {
            WPRINT_LIB_ERROR(("Error receiving data from stream(received:%d bytes) result:%d \n",(int)received_bytes, (int)result ) );
            return WICED_ERROR;
        }
    }

    /* copy the mask */
    if( payload_length <= WEBSOCKET_FRAME_PAYLOAD_LENGTH_MAXIMUM_7_BITS )
    {
        data_length = payload_length;
        if( is_masked )
        {
            memcpy(mask, &header[2], 4);
        }
        /* do nothing */
    }
    if( payload_length == WEBSOCKET_FRAME_PAYLOAD_LENGTH_2_BYTES )
    {
        data_length = (uint16_t) ( header[ 3 ] | ( header[ 2 ] << 8 ) );
        if( is_masked )
        {
            memcpy(mask, &header[4], 4);
        }
    }
    else if( payload_length == WEBSOCKET_FRAME_PAYLOAD_LENGTH_8_BYTES )
    {
        /* if payload length is greater than uint32_t; just bail out */
        if( !header[5] || !header[4] || !header[3] || !header[2] )
        {
            return WICED_ERROR;
        }
        data_length = (uint32_t) ( (header[9] ) | (header[8] << 8) | (header[7] << 16) | (header[6] << 24) );
        if( is_masked )
        {
            memcpy(mask, &header[10], 4);
        }
    }

    /* if received' frame payload is larger than frame-buffer size..bail out */
    if( data_length > max_buffer_length )
    {
        WPRINT_LIB_INFO(("Rx Frame buffer is smaller than Websocket Frame payload length\n"));
        return WICED_ERROR;
    }

    /* Now read the Message payload in the application buffer */
    received_bytes = 0;
    result = wiced_tcp_stream_read_with_count( stream, buffer, (uint16_t)data_length, DEFAULT_WEBSOCKET_FRAME_TIMEOUT, &received_bytes );
    if( result != WICED_SUCCESS || received_bytes != data_length )
    {
        WPRINT_LIB_INFO(("Received bytes from stream:%d result:%d \n",(int)received_bytes, (int)result ) );
        return WICED_ERROR;
    }

    if( received_bytes == data_length )
    {
        WPRINT_LIB_DEBUG(("Received Websocket Frame completely\n"));
    }

    if( !websocket->callbacks.on_message )
    {
        WPRINT_LIB_INFO(("No Application on-message callback available\n"));
        return WICED_SUCCESS;
    }

    /* Do in-place Unmasking */
    if( is_masked )
    {
        mask_unmask_frame_data(buffer, buffer, data_length, mask);
    }

    /* Not expecting any CONTINUATION Frame */
    if( !websocket->core.receive.continuation )
    {
        frame_type = type;
        if( final_frame )
        {
            /* A new frame with FIN bit set */
            flags = WEBSOCKET_FRAME_FLAG_UNFRAGMENTED;
        }
        else
        {
            /* A new frame with FIN bit cleared -- hence expect fragmented messages */
            flags = WEBSOCKET_FRAME_FLAG_FRAGMENTED_FIRST;
            /* update book-keeping */
            websocket->core.receive.continuation = 1;
            websocket->core.receive.first_fragment_opcode = type;
        }
    }
    else
    {
        frame_type = websocket->core.receive.first_fragment_opcode;
        if( final_frame )
        {
            flags = WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL;
            websocket->core.receive.continuation = 0;
            websocket->core.receive.first_fragment_opcode = 0;
        }
        else
        {
            flags = WEBSOCKET_FRAME_FLAG_CONTINUED;
        }
    }

    /* Give it to the application */
    websocket->callbacks.on_message( websocket, buffer, data_length, frame_type, flags );

    return WICED_SUCCESS;
}

#if 0
static void handle_frame_error( wiced_websocket_t* websocket, const char* close_string )
{
    /* If frame error, end-point should initiate Websocket-close connection */
    wiced_websocket_close( websocket, WEBSOCKET_CLOSE_STATUS_CODE_PROTOCOL_ERROR, close_string );
}

static void handle_ping_frame( wiced_websocket_t* websocket , wiced_websocket_frame_t* rx_frame )
{
    wiced_websocket_send( websocket, rx_frame->payload, rx_frame->payload_length, WEBSOCKET_PONG, UNFRAGMENTED_FRAME );
}

static void handle_pong_frame( wiced_websocket_t* websocket, wiced_websocket_frame_t* rx_frame )
{
    UNUSED_PARAMETER(websocket);
    UNUSED_PARAMETER(rx_frame);
    /* Do Nothing */
}

static void handle_close_connection_frame( wiced_websocket_t* websocket, wiced_websocket_frame_t* rx_frame )
{
    /* Do Nothing */
    UNUSED_PARAMETER(websocket);
    UNUSED_PARAMETER(rx_frame);
}
#endif

wiced_result_t wiced_websocket_close( wiced_websocket_t* websocket, const uint16_t code, const char* reason )
{
    wiced_result_t          result = WICED_SUCCESS;
    wiced_tcp_socket_t* tcp_socket = NULL;
    uint8_t payload_length = 0;
    uint8_t body_message[WEBSOCKET_CLOSE_FRAME_BODY_MAX_LENGTH] = { 0 };
    GET_TCP_SOCKET_FROM_WEBSOCKET((tcp_socket), (websocket))

    if( websocket->state == WEBSOCKET_CLOSED || websocket->state == WEBSOCKET_CLOSING )
    {
        /* Nothing to DO */
        return WICED_SUCCESS;
    }

    if( verify_close_code( code ) == WICED_FALSE )
    {
        WPRINT_LIB_ERROR(("[websocket] Invalid or Non-standard CLOSE status-code:%d\n", code ) );
        return WICED_ERROR;
    }

    if( verify_close_reason( reason ) == WICED_FALSE )
    {
        WPRINT_LIB_ERROR(("[websocket] Invalid CLOSE reason string\n" ) );
        return WICED_ERROR;
    }

    if( code == WEBSOCKET_CLOSE_STATUS_NO_CODE )
    {
        result = wiced_websocket_send( websocket, NULL, 0, WEBSOCKET_CONNECTION_CLOSE, WEBSOCKET_FRAME_FLAG_UNFRAGMENTED );
        if( result != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR(("Error sending CLOSE frame\n"));
        }
        return result;
    }

    body_message[0] = (uint8_t) (code >> 8);
    body_message[1] = (uint8_t) (code & 0xFF);
    payload_length = 2;

    if( reason != NULL )
    {
        uint8_t reason_length = (uint8_t)strlen(reason);
        memcpy( &body_message[2], reason, reason_length );
        payload_length = (uint8_t)(payload_length + reason_length );
    }

    WICED_VERIFY( wiced_websocket_send( websocket, body_message, payload_length, WEBSOCKET_CONNECTION_CLOSE, WEBSOCKET_FRAME_FLAG_UNFRAGMENTED ) );

    update_socket_state( websocket, WEBSOCKET_CLOSING );

    wiced_tcp_stream_deinit(&websocket->stream);

    if( websocket->core.role == WEBSOCKET_ROLE_CLIENT )
    {
        wiced_tcp_disconnect( tcp_socket );

        update_socket_state( websocket, WEBSOCKET_CLOSED );

        wiced_tcp_delete_socket( tcp_socket );
    }
    else
    {
        wiced_websocket_server_t* websocket_server = NULL;
        GET_SERVER_FROM_WEBSOCKET(websocket_server, websocket)
        if( websocket_server == NULL )
        {
            return WICED_BADARG;
        }

        update_socket_state(websocket, WEBSOCKET_CLOSED);
        result = wiced_tcp_server_disconnect_socket(&websocket_server->tcp_server, tcp_socket);
        if( result != WICED_SUCCESS )
        {
            WPRINT_LIB_INFO(("[websocket] Server disconnecting socket result:%d\n", result));
            return result;
        }
        update_socket_state(websocket, WEBSOCKET_INITIALISED);
    }

    return result;
}

static wiced_result_t update_socket_state( wiced_websocket_t* websocket, wiced_websocket_state_t state )
{
    websocket->state = state;
    return WICED_SUCCESS;
}

static wiced_result_t on_websocket_close_callback(  wiced_tcp_socket_t* socket, void* websocket )
{
    wiced_websocket_t* websocket_ptr = websocket;

    UNUSED_PARAMETER(socket);

    if( websocket_ptr->callbacks.on_close != NULL )
    {
        websocket_ptr->callbacks.on_close( websocket_ptr );
    }

    return WICED_SUCCESS;
}

/*  This Callback, for Websockets(Client only), will run in NETWORKING worker thread context.
 *  Applications which are using Websockets will receive the data from Networking worker thread.
 *  Idea is to have a lean n thin websocket, without creating additional thread to get the data.
 */
static wiced_result_t on_websocket_message_callback( wiced_tcp_socket_t* socket, void* websocket )
{
    wiced_result_t result = WICED_ERROR;

    wiced_websocket_t* websocket_ptr = (wiced_websocket_t*) websocket;
    if( !websocket_ptr || !socket )
    {
        return WICED_BADARG;
    }

    /* Make sure websocket is in 'Established Connection' state */
    if( !match_websocket_state(websocket_ptr, WEBSOCKET_OPEN) )
    {
        return result;
    }

    result = websocket_receive_frame( websocket_ptr );

    if( result != WICED_SUCCESS )
    {
        /* Handle Frame parse and timeout errors etc. */
        WPRINT_LIB_INFO(("Error Parsing Websocket Frame\n"));
        return result;
    }

    return WICED_SUCCESS;
}

static void lock_websocket_mutex( void )
{
    if ( websocket_mutex.mutex_state == MUTEX_UNINITIALISED )
    {
        /* create handhsake mutex. There can only be one connection in a connecting state*/
        wiced_rtos_init_mutex( &websocket_mutex.handshake_lock );
        websocket_mutex.mutex_state = MUTEX_UNLOCKED;
    }

    if ( websocket_mutex.mutex_state == MUTEX_UNLOCKED )
    {
        wiced_rtos_lock_mutex( &websocket_mutex.handshake_lock );
        websocket_mutex.mutex_state = MUTEX_LOCKED;
    }

}

static void unlock_websocket_mutex( void )
{
    if ( websocket_mutex.mutex_state == MUTEX_LOCKED )
    {
        wiced_rtos_unlock_mutex( &websocket_mutex.handshake_lock );
        websocket_mutex.mutex_state = MUTEX_UNLOCKED;
    }
}

wiced_result_t wiced_websocket_register_callbacks ( wiced_websocket_t* websocket, wiced_websocket_callback_t on_open_callback, wiced_websocket_callback_t on_close_callback, wiced_websocket_message_callback_t on_message_callback, wiced_websocket_callback_t on_error_callback )
{
    websocket->callbacks.on_open    = on_open_callback;
    websocket->callbacks.on_close   = on_close_callback;
    websocket->callbacks.on_error   = on_error_callback;
    websocket->callbacks.on_message = on_message_callback;

    return WICED_SUCCESS;
}

void wiced_websocket_unregister_callbacks ( wiced_websocket_t* websocket )
{
    websocket->callbacks.on_open    = NULL;
    websocket->callbacks.on_close   = NULL;
    websocket->callbacks.on_error   = NULL;
    websocket->callbacks.on_message = NULL;
}
