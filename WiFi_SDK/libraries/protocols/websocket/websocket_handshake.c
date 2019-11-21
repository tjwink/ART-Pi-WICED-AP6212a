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

#include "websocket_handshake.h"
#include "wiced_crypto.h"
#include <base64.h>
#include <ctype.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define STRING_LENGTH_GUID                               36
#define GUID                                             "258EAFA5-E914-47DA-95CA-C5AB0DC85B11" /* Globally Unique Identifier for Websocket Protocol defined in RFC6455 */
#define HTTP_RESPONSE_STATUS_TOKEN                       "HTTP/1.1 101"
#define HTTP_UPGRADE_CASE_SENSITIVE_TOKEN                "Upgrade: "
#define HTTP_UPGRADE_CASE_INSENSITIVE_TOKEN              "WEBSOCKET"
#define HTTP_CONNECTION_CASE_SENSITIVE_TOKEN             "Connection: "
#define HTTP_CONNECTION_CASE_INSENSITIVE_TOKEN           "UPGRADE"
#define HTTP_WEBSOCKET_ACCEPT_CASE_SENSITIVE_TOKEN       "Sec-WebSocket-Accept: "
#define HTTP_WEBSOCKET_SUB_PROTOCOL_CASE_SENSITIVE_TOKEN "Sec-WebSocket-Protocol: "
#define CR_LN                                            "\r\n"

#define GET_TOKEN                                   "GET "
#define HTTP_1_1_TOKEN                              " HTTP/1.1"

#define HTTP_HEADER_FIELD_HOST                      "Host: "
#define HTTP_HEADER_FIELD_CONNECTION                "Connection: "
#define HTTP_HEADER_FIELD_UPGRADE                   "Upgrade: "
#define HTTP_HEADER_FIELD_SEC_WEBSOCKET_KEY         "Sec-WebSocket-Key: "
#define HTTP_HEADER_FIELD_SEC_WEBSOCKET_VERSION     "Sec-WebSocket-Version: "
#define HTTP_HEADER_FIELD_ORIGIN                    "Origin: "
#define HTTP_HEADER_FIELD_SEC_WEBSOCKET_ACCEPT      "Sec-WebSocket-Accept: "
#define HTTP_HEADER_FIELD_SEC_WEBSOCKET_PROTOCOL    "Sec-WebSocket-Protocol: "
#define HTTP_HEADER_FIELD_SEC_WEBSOCKET_EXTENSIONS  "Sec-WebSocket-Extensions: "

#define HTTP_HEADER_FIELD_UPGRADE_VALUE                 "websocket"
#define HTTP_HEADER_FIELD_CONNECTION_VALUE              "UPGRADE"
#define HTTP_HEADER_FIELD_SEC_WEBSOCKET_VERSION_VALUE   "13"


#define HTTP_HEADER_404                   "HTTP/1.1 404 Not Found"

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

static wiced_result_t wiced_generate_client_websocket_key( uint8_t* websocket_key_base64_ptr);
static wiced_result_t wiced_generate_server_websocket_key( uint8_t* server_websocket_key,uint8_t* websocket_key_from_client, uint32_t websocket_key_length);
static wiced_result_t wiced_verify_server_handshake(char* server_response);

/* case insensitive string compare */
static char *stristr(const char *string, const char *pattern);

/******************************************************
 *               Variable Definitions
 ******************************************************/

static uint8_t websocket_key_base64[CLIENT_WEBSOCKET_BASE64_KEY_LENGTH];
static char    sub_protocol_supported[SUB_PROTOCOL_STRING_LENGTH];
void dump_raw_data(uint8_t* ptr, uint32_t length, const char* str);
/******************************************************
 *               Function Definitions
 ******************************************************/

void dump_raw_data(uint8_t* ptr, uint32_t length, const char* str)
{
    uint32_t i =0;
    WPRINT_LIB_INFO(("[%s Data Dump ]:\n", str));
    for ( i = 0; i< length; i++)
    {
        WPRINT_LIB_INFO(("%x\t", ptr[i]));
        if(((i+1) % 4) == 0)
            WPRINT_LIB_INFO(("\n"));
    }
    WPRINT_LIB_INFO(("\n"));
}

static wiced_result_t wiced_generate_client_websocket_key( uint8_t* websocket_key_base64_ptr)
{
    wiced_result_t result;
    uint8_t        client_websocket_key[ WEBSOCKET_KEY_LENGTH ];

    result = wiced_crypto_get_random( client_websocket_key, WEBSOCKET_KEY_LENGTH );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Now we base64 encode the 16 byte key, as required by the RFC6455 specification */
    if ( base64_encode( client_websocket_key, WEBSOCKET_KEY_LENGTH, websocket_key_base64_ptr, CLIENT_WEBSOCKET_BASE64_KEY_LENGTH, BASE64_STANDARD ) == 0 )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t wiced_verify_server_handshake(char* server_response)
{
    char*   received_server_key_start_pointer;
    char*   received_server_key_end_pointer;
    char*   start_of_match;
    uint8_t server_websocket_key[SERVER_WEBSOCKET_BASE64_SHA1_KEY_LENGTH];

    if ( strncmp( server_response, HTTP_RESPONSE_STATUS_TOKEN, strlen( HTTP_RESPONSE_STATUS_TOKEN ) ) != 0 )
    {
        return WICED_ERROR;
    }

    memset( sub_protocol_supported, 0x0, sizeof( sub_protocol_supported ) );

    /* Move to the http upgrade token */
    start_of_match = strstr( server_response, HTTP_UPGRADE_CASE_SENSITIVE_TOKEN );

    /* If response does not contain the upgrade token, it is invalid, so exit with error */
    if ( start_of_match == NULL )
    {
        return WICED_ERROR;
    }

    /* Move to the http upgrade token to point to its value*/
    start_of_match += strlen( HTTP_UPGRADE_CASE_SENSITIVE_TOKEN );

    /* According to the RFC6455, the header is case sensitive but the field is case insensitive*/
    if ( stristr( start_of_match, HTTP_UPGRADE_CASE_INSENSITIVE_TOKEN ) == 0 )
    {
        return WICED_ERROR;
    }
    start_of_match = strstr( server_response, HTTP_CONNECTION_CASE_SENSITIVE_TOKEN );

    /* If response does not contain the Connection token, it is invalid, so exit with error */
    if ( start_of_match == NULL )
    {
        return WICED_ERROR;
    }
    /* Move to the ccurrent pointer past onnection token to point to its value*/
    start_of_match += strlen( HTTP_CONNECTION_CASE_SENSITIVE_TOKEN );

    if ( stristr( start_of_match, HTTP_CONNECTION_CASE_INSENSITIVE_TOKEN ) == 0 )
    {
        return WICED_ERROR;
    }
    /* check for any subprotocols*/
    start_of_match = strstr( server_response, HTTP_WEBSOCKET_SUB_PROTOCOL_CASE_SENSITIVE_TOKEN );
    if ( start_of_match != NULL )
    {
        strncpy( sub_protocol_supported, start_of_match, SUB_PROTOCOL_STRING_LENGTH - 1 );
        sub_protocol_supported [ SUB_PROTOCOL_STRING_LENGTH - 1 ] = '\0';
    }

    received_server_key_start_pointer = strstr( server_response, HTTP_WEBSOCKET_ACCEPT_CASE_SENSITIVE_TOKEN );

    if ( received_server_key_start_pointer == NULL )
    {
        return WICED_ERROR;
    }
    received_server_key_start_pointer += strlen( HTTP_WEBSOCKET_ACCEPT_CASE_SENSITIVE_TOKEN );

    received_server_key_end_pointer = strstr( received_server_key_start_pointer, CR_LN );

    if ( ( received_server_key_end_pointer == NULL ) || ( received_server_key_end_pointer <= received_server_key_start_pointer ) )
    {
        return WICED_ERROR;
    }
    /* generate key that server sent in the handshake, and validate they are the same*/
    wiced_generate_server_websocket_key( server_websocket_key, websocket_key_base64, strlen( (char*) websocket_key_base64 ) );

    if ( strncmp( (char*) server_websocket_key, received_server_key_start_pointer, (uint8_t) ( received_server_key_end_pointer - received_server_key_start_pointer ) ) == 0 )
    {
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }
}

static wiced_result_t wiced_generate_server_websocket_key( uint8_t* server_websocket_key, uint8_t* websocket_key_from_client, uint32_t websocket_key_length)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t        websocket_concatenated_key[ CLIENT_WEBSOCKET_BASE64_KEY_LENGTH + STRING_LENGTH_GUID ] = { 0x0 };
    uint8_t        websocket_key_sha1[ SHA_LENGTH ];

    /* concatenate client key with GUI*/
    strncpy( (char*) websocket_concatenated_key, (char*) websocket_key_from_client, websocket_key_length );
    strncat( (char*) websocket_concatenated_key, GUID, strlen( GUID ) );

    /* Create SHA1 of the key concatenation above*/
    mbedtls_sha1( websocket_concatenated_key, strlen( (char*) websocket_concatenated_key ), websocket_key_sha1 );

    /* Base64 Encode the SHA1 value*/
    if ( base64_encode( websocket_key_sha1, SHA_LENGTH, server_websocket_key, SERVER_WEBSOCKET_BASE64_SHA1_KEY_LENGTH, BASE64_STANDARD ) == 0 )
    {
        return WICED_ERROR;
    }
    return result;
}

static wiced_result_t do_client_handshake( wiced_websocket_t* websocket, wiced_websocket_handshake_fields_t* websocket_header_fields )
{
    wiced_packet_t*     tcp_reply_packet;
    uint16_t            total_received_bytes;
    uint8_t*            received_handshake;
    uint16_t            tcp_data_available;
    wiced_tcp_socket_t* tcp_socket = NULL;
    wiced_tcp_stream_t* stream;
    wiced_result_t      result;
    websocket_client_handshake_fields_t* client = &(websocket_header_fields->client);
    if( !client )
    {
        WPRINT_LIB_ERROR(("Error doing client handshake\n"));
        return WICED_BADARG;
    }

    GET_TCP_SOCKET_FROM_WEBSOCKET(tcp_socket, websocket)

    stream = &websocket->stream;

    /* generate a unique websocket key to send to server as part of initial handshake*/
    WICED_VERIFY( wiced_generate_client_websocket_key( websocket_key_base64 ) );

    /* build the handshaking headers*/

    /* < GET /uri HTTP/1.1 >*/
    wiced_tcp_stream_write( stream, "GET ", (uint16_t) strlen( "GET " ) );
    wiced_tcp_stream_write( stream, client->resource_name, (uint16_t) strlen( client->resource_name ) );
    wiced_tcp_stream_write( stream, " HTTP/1.1\r\n", (uint16_t) strlen( " HTTP/1.1\r\n" ) );

    /* < Host: ip1.ip2.ip3.ip4 >*/
    wiced_tcp_stream_write( stream, "Host: ", (uint16_t) strlen( "Host: " ) );
    wiced_tcp_stream_write( stream, client->host, (uint16_t) strlen( client->host ) );
    wiced_tcp_stream_write( stream, "\r\n", (uint16_t) strlen( "\r\n" ) );

    /* < Upgrade: websocket>*/
    wiced_tcp_stream_write( stream, "Upgrade: websocket\r\n", (uint16_t) strlen( "Upgrade: websocket\r\n" ) );

    /* < Connection: Upgrade >*/
    wiced_tcp_stream_write( stream, "Connection: Upgrade\r\n", (uint16_t) strlen( "Connection: Upgrade\r\n" ) );

    /* < Sec-WebSocket-Key: random_base4_value >*/
    wiced_tcp_stream_write( stream, "Sec-WebSocket-Key: ", (uint16_t) strlen( "Sec-WebSocket-Key: " ) );
    wiced_tcp_stream_write( stream, websocket_key_base64, (uint16_t) strlen( (char*) websocket_key_base64 ) );
    wiced_tcp_stream_write( stream, "\r\n", (uint16_t) strlen( "\r\n" ) );

    /* < Origin: ip1.ip2.ip3.ip4 >*/
    wiced_tcp_stream_write( stream, "Origin: ", (uint16_t) strlen( "Origin: " ) );
    wiced_tcp_stream_write( stream, client->origin, (uint16_t) strlen( client->origin ) );
    wiced_tcp_stream_write( stream, "\r\n", (uint16_t) strlen( "\r\n" ) );

    /* The sec_websocket_protocol is optional, so check if it has been added, include in header if required*/
    if ( client->protocols != NULL )
    {
        /* < Sec-WebSocket-Protocol: server_understood_protocol >*/
        wiced_tcp_stream_write( stream, "Sec-WebSocket-Protocol: ", (uint16_t) strlen( "Sec-WebSocket-Protocol: " ) );
        wiced_tcp_stream_write( stream, client->protocols, (uint16_t) strlen( client->protocols ) );
        wiced_tcp_stream_write( stream, "\r\n", (uint16_t) strlen( "\r\n" ) );
    }

    /* < Sec-WebSocket-Version: 13 >*/
    wiced_tcp_stream_write( stream, "Sec-WebSocket-Version: 13\r\n\r\n", strlen( "Sec-WebSocket-Version: 13\r\n\r\n" ) );

    /* send the handshake to server*/
    wiced_tcp_stream_flush( stream );

    result = wiced_tcp_receive( tcp_socket, &tcp_reply_packet, WICED_WAIT_FOREVER );
    if( result != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO(("Error receiving packet\n"));
        return WICED_ERROR;
    }
    wiced_packet_get_data( tcp_reply_packet, 0, (uint8_t**) &received_handshake, &total_received_bytes, &tcp_data_available );
    if ( total_received_bytes < tcp_data_available )
    {
        WPRINT_LIB_INFO(("Fragmented packets not supported\n"));
        wiced_packet_delete( tcp_reply_packet );
        return WICED_ERROR;
    }

    WICED_VERIFY( wiced_verify_server_handshake( (char* )received_handshake ) );

    wiced_packet_delete( tcp_reply_packet );

    return WICED_SUCCESS;
}

static wiced_result_t wiced_verify_client_handshake( char* client_request, char* server_websocket_key )
{
    char client_request_uri[50]                                     = { 0x0 };
    char client_host_value[50]                                      = { 0x0 };
    char client_websocket_key[CLIENT_WEBSOCKET_BASE64_KEY_LENGTH]   = { 0x0 };

    char* field_value_start_ptr;
    char* field_value_end_ptr;
    char* start_of_field;

    /* if we don't get 'GET', bail out */
    if( strncmp((char*)client_request, GET_TOKEN, strlen(GET_TOKEN) ) != 0 )
    {
        return WICED_ERROR;
    }

    /* uri field */
    start_of_field = client_request + strlen(GET_TOKEN);
    field_value_start_ptr = start_of_field;

    start_of_field = strstr( client_request, HTTP_1_1_TOKEN);
    if( start_of_field == NULL )
    {
        return WICED_ERROR;
    }

    field_value_end_ptr = start_of_field;
    /* <request_uri> field from client */
    /* FIXME: correct size memcpy */
    memcpy(client_request_uri, field_value_start_ptr, (unsigned int)(field_value_end_ptr - field_value_start_ptr) );

    WPRINT_LIB_INFO(("client-uri: %s\n", client_request_uri));

    /* |Host| header field containing server's authority */
    start_of_field = strstr( client_request , HTTP_HEADER_FIELD_HOST );
    if( start_of_field == NULL )
    {
        return WICED_ERROR;
    }

    field_value_start_ptr  = start_of_field + strlen(HTTP_HEADER_FIELD_HOST);
    field_value_end_ptr    = strstr( start_of_field, CR_LN );
    /* FIXME: correct size memcpy */
    memcpy((void *)client_host_value, (void *)field_value_start_ptr, (uint32_t)(field_value_end_ptr - field_value_start_ptr) );
    WPRINT_LIB_INFO( ("client-host: %s\n", client_host_value) );

    /* |Connection| header field containing 'upgrade' as value */
    start_of_field = strstr( client_request, HTTP_HEADER_FIELD_CONNECTION );
    if( start_of_field == NULL )
    {
        WPRINT_LIB_INFO(("Could not find [%s] header\n", HTTP_HEADER_FIELD_CONNECTION ));
        return WICED_ERROR;
    }

    start_of_field += strlen(HTTP_HEADER_FIELD_CONNECTION);
    if ( stristr( (char *)start_of_field, HTTP_HEADER_FIELD_CONNECTION_VALUE ) == NULL )
    {
        WPRINT_LIB_INFO( ("Could not find connection value\n") );
        return WICED_ERROR;
    }

    /* |Upgrade| header field containing 'websocket' as value */
    start_of_field = strstr( client_request, HTTP_HEADER_FIELD_UPGRADE );
    if( start_of_field == NULL )
    {
        return WICED_ERROR;
    }

    start_of_field += strlen(HTTP_HEADER_FIELD_UPGRADE);

    if ( stristr( (char*)start_of_field, HTTP_HEADER_FIELD_UPGRADE_VALUE) == NULL )
    {
        WPRINT_LIB_INFO(("Could not find upgrade value\n") );
        return WICED_ERROR;
    }


    /* |Sec-WebSocket-Version| header field, with a value of 13 */
    start_of_field = strstr(client_request, HTTP_HEADER_FIELD_SEC_WEBSOCKET_VERSION );
    if( start_of_field == NULL )
    {
        return WICED_ERROR;
    }

    start_of_field += strlen(HTTP_HEADER_FIELD_SEC_WEBSOCKET_VERSION);
    if( strncmp(start_of_field, HTTP_HEADER_FIELD_SEC_WEBSOCKET_VERSION_VALUE, strlen(HTTP_HEADER_FIELD_SEC_WEBSOCKET_VERSION_VALUE)) != 0 )
    {
        return WICED_ERROR;
    }

    /* |Sec-WebSocket-Key| header field with a base64-encoded value */
    start_of_field = strstr(client_request, HTTP_HEADER_FIELD_SEC_WEBSOCKET_KEY);
    if( start_of_field == NULL )
    {
        return WICED_ERROR;
    }
    field_value_start_ptr   = start_of_field + strlen(HTTP_HEADER_FIELD_SEC_WEBSOCKET_KEY);
    field_value_end_ptr     = strstr( start_of_field, CR_LN);
    if( field_value_end_ptr == NULL || field_value_end_ptr <= field_value_start_ptr )
    {
        return WICED_ERROR;
    }
    memcpy(client_websocket_key, field_value_start_ptr, (uint32_t)(field_value_end_ptr - field_value_start_ptr) );
    if ( wiced_generate_server_websocket_key( (uint8_t*)server_websocket_key, (uint8_t*)client_websocket_key, strlen(client_websocket_key) ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static void wiced_abort_client_handshake( wiced_tcp_stream_t* stream )
{
    /* FIXME: different error codes need to be send depending on what went wrong with client handshake.
     * For now, just send the bad-request so that client is aware of server-side error.
     */
    wiced_tcp_stream_write(stream, HTTP_HEADER_404, (uint16_t) strlen(HTTP_HEADER_404) );
    wiced_tcp_stream_write(stream, CR_LN, (uint16_t) strlen(CR_LN) );

    /* send the abort-reason to client */
    wiced_tcp_stream_flush( stream );
}

static wiced_result_t do_server_handshake( wiced_websocket_t* websocket, const wiced_websocket_handshake_fields_t* handshake_fields )
{
    wiced_result_t      result = WICED_ERROR;
    wiced_tcp_stream_t* stream;
    wiced_packet_t*     tcp_reply_packet;
    uint16_t            total_received_bytes;
    uint8_t*            received_client_handshake;
    uint16_t            tcp_data_available;
    wiced_tcp_socket_t* tcp_socket = NULL;
    char server_websocket_key[SERVER_WEBSOCKET_BASE64_SHA1_KEY_LENGTH] = { 0x0 };

    UNUSED_PARAMETER(handshake_fields);

    GET_TCP_SOCKET_FROM_WEBSOCKET(tcp_socket, websocket)

    stream = &websocket->stream;

    /* Get Client Handshake data immediately as we have arrived here after receive_notification */
    result = wiced_tcp_receive( tcp_socket, &tcp_reply_packet, WICED_NO_WAIT );

    if( result != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO(("[%s] %d tcp-receive-result: %d\n", __func__, __LINE__, result));
        return result;
    }

    wiced_packet_get_data( tcp_reply_packet, 0, (uint8_t** )&received_client_handshake, &total_received_bytes, &tcp_data_available );
    if ( total_received_bytes < tcp_data_available )
    {
        WPRINT_LIB_INFO(("Fragmented packets not supported\n"));
        wiced_packet_delete( tcp_reply_packet );
        return WICED_ERROR;
    }

    //dump_raw_data(received_client_handshake, total_received_bytes, "verify-client");

    result = wiced_verify_client_handshake( (char *)received_client_handshake, server_websocket_key );
    if( result != WICED_SUCCESS )
    {
        wiced_abort_client_handshake( stream );
        return result;
    }

    wiced_packet_delete( tcp_reply_packet );

    /* Send Server response; build handshake headers */

    /* Leading line from server */
    wiced_tcp_stream_write( stream, "HTTP/1.1 101 Switching Protocols\r\n", (uint16_t)strlen("HTTP/1.1 101 Switching Protocols\r\n") );

    /* < Upgrade: websocket>*/
    wiced_tcp_stream_write( stream, "Upgrade: websocket\r\n", (uint16_t) strlen( "Upgrade: websocket\r\n" ) );

    /* < Connection: Upgrade >*/
    wiced_tcp_stream_write( stream, "Connection: Upgrade\r\n", (uint16_t) strlen( "Connection: Upgrade\r\n" ) );

    /* < Sec-WebSocket-Accept: accept-key > */
    wiced_tcp_stream_write( stream, HTTP_HEADER_FIELD_SEC_WEBSOCKET_ACCEPT, (uint16_t) strlen(HTTP_HEADER_FIELD_SEC_WEBSOCKET_ACCEPT) );

    wiced_tcp_stream_write( stream, server_websocket_key, strlen(server_websocket_key) );
    wiced_tcp_stream_write( stream, CR_LN, strlen(CR_LN) );
    wiced_tcp_stream_write( stream, CR_LN, strlen(CR_LN) );
    /* send the handshake to client */
    wiced_tcp_stream_flush( stream );


    return WICED_SUCCESS;
}

wiced_result_t wiced_establish_websocket_handshake( wiced_websocket_t* websocket, wiced_websocket_handshake_fields_t* handshake_fields )
{
    if( websocket->core.role == WEBSOCKET_ROLE_CLIENT )
    {
        return do_client_handshake(websocket, handshake_fields);
    }
    else if(websocket->core.role == WEBSOCKET_ROLE_SERVER)
    {
        return do_server_handshake(websocket, handshake_fields);
    }
    else
        return WICED_ERROR;
}

wiced_result_t wiced_get_websocket_subprotocol( char* subprotocol )
{
    if ( sub_protocol_supported[ 0 ] == 0x0 )
    {
        return WICED_ERROR;
    }

    strncpy( subprotocol, sub_protocol_supported, SUB_PROTOCOL_STRING_LENGTH - 1);
    subprotocol[SUB_PROTOCOL_STRING_LENGTH - 1] = '\0';
    return WICED_SUCCESS;
}

static char* stristr( const char* string, const char* pattern )
{
    char* pptr  = (char *) pattern;
    char* start = (char *) string;
    uint  slen  = strlen( string );
    uint  plen  = strlen( pattern );
    char* sptr;

    for ( ; slen >= plen; start++, slen-- )
    {
        /* find start of pattern in string */
        while ( toupper( (int) *start ) != toupper( (int) *pattern ) )
        {
            start++;
            slen--;

            /* if pattern longer than string */
            if ( slen < plen )
            {
                return NULL;
            }
        }

        sptr = start;
        pptr = (char *) pattern;

        while ( toupper( (int) *sptr ) == toupper( (int) *pptr ) )
        {
            sptr++;
            pptr++;

            /* if end of pattern then pattern was found */
            if ( '\0' == *pptr )
            {
                return start;
            }
        }
    }
    return NULL;
}
