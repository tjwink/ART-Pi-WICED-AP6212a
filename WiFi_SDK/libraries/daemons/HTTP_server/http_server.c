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
 *  Implements both HTTP and HTTPS servers
 *
 */

#include <string.h>
#include "http_server.h"
#include "wwd_assert.h"
#include "wiced.h"
#include "wiced_utilities.h"
#include "wiced_resource.h"
#include "strings.h"
#include "platform_resource.h"
#include "wiced_utilities.h"
#include "wiced_tls.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

//#define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
#define HTTP_DEBUG( X )     printf X
#else
#define HTTP_DEBUG( X )     //printf X
#endif

#define HTTP_SERVER_CONNECT_THREAD_STACK_SIZE  (1500)
#define HTTPS_SERVER_CONNECT_THREAD_STACK_SIZE (4000)

#define HTTP_SERVER_THREAD_PRIORITY    (WICED_DEFAULT_LIBRARY_PRIORITY)
#define HTTP_LISTEN_PORT               (80)
#define HTTP_SERVER_RECEIVE_TIMEOUT    (1000)

/* HTTP Tokens */
#define GET_TOKEN                      "GET "
#define POST_TOKEN                     "POST "
#define PUT_TOKEN                      "PUT "

#define HTTP_1_1_TOKEN                 " HTTP/1.1"
#define FINAL_CHUNKED_PACKET           "0\r\n\r\n"

/*
 * Request-Line =   Method    SP        Request-URI           SP       HTTP-Version      CRLFCRLF
 *              = <-3 char->  <-1 char->   <-1 char->      <-1 char->  <--8 char-->    <-4char->
 *              = 18
 */
#define MINIMUM_REQUEST_LINE_LENGTH    (18)
#define EVENT_QUEUE_DEPTH              (10)
#define COMPARE_MATCH                  (0)
#define MAX_URL_LENGTH                 (100)

/* Most of the webserver supports request length 2KB to 8KB.
 * We may thus assume that 8KB is the maximum possible length
 * and 2KB is a more affordable length to rely on at the server side */
#define MAXIMUM_CACHED_LENGTH          (8192)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    SOCKET_ERROR_EVENT,
    SOCKET_DISCONNECT_EVENT,
    SOCKET_PACKET_RECEIVED_EVENT,
    SERVER_STOP_EVENT,
} http_server_event_t;

typedef enum
{
    HTTP_HEADER_AND_DATA_FRAME_STATE,
    HTTP_DATA_ONLY_FRAME_STATE,
    HTTP_ERROR_STATE
} wiced_http_packet_state_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_tcp_socket_t* socket;
    http_server_event_t event_type;
} server_event_message_t;

typedef struct
{
    linked_list_node_t  node;
    wiced_http_stream_t stream;
} http_stream_node_t;

typedef struct
{
    wiced_tcp_socket_t*       socket;
    wiced_http_packet_state_t http_packet_state;
    char                      url[ MAX_URL_LENGTH ];
    uint16_t                  url_length;
    wiced_http_message_body_t message_body_descriptor;
} wiced_http_message_context_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t           http_server_deferred_connect_callback( void* arg );
static wiced_result_t           http_server_connect_callback         ( wiced_tcp_socket_t* socket, void* arg );
static wiced_result_t           http_server_disconnect_callback      ( wiced_tcp_socket_t* socket, void* arg );
static wiced_result_t           http_server_receive_callback         ( wiced_tcp_socket_t* socket, void* arg );
static void                     http_server_event_thread_main        ( uint32_t arg );
static wiced_result_t           http_server_parse_receive_packet     ( wiced_http_server_t* server, wiced_http_stream_t* stream, wiced_packet_t* packet );
static wiced_result_t           http_server_process_url_request      ( wiced_http_stream_t* stream, const wiced_http_page_t* page_database, char* url, uint32_t url_length, wiced_http_message_body_t* http_message_body );
static uint16_t                 http_server_remove_escaped_characters( char* output, uint16_t output_length, const char* input, uint16_t input_length );
static wiced_packet_mime_type_t http_server_get_mime_type            ( const char* request_data );
static wiced_result_t           http_server_get_request_type_and_url ( char* request, uint16_t request_length, wiced_http_request_type_t* type, char** url_start, uint16_t* url_length );
static wiced_result_t           http_server_find_url_in_page_database( char* url, uint32_t length, wiced_http_message_body_t* http_request, const wiced_http_page_t* page_database, wiced_http_page_t** page_found, wiced_packet_mime_type_t* mime_type );
static wiced_bool_t             http_server_compare_stream_socket    ( linked_list_node_t* node_to_compare, void* user_data );
static wiced_result_t           http_internal_server_start           (wiced_http_server_t* server, uint16_t port, uint16_t max_sockets, const wiced_http_page_t* page_database, wiced_interface_t interface, uint32_t http_thread_stack_size, uint32_t server_connect_thread_stack_size );

/******************************************************
 *                 Static Variables
 ******************************************************/

static const char* const http_mime_array[ MIME_UNSUPPORTED ] =
{
    MIME_TABLE( EXPAND_AS_MIME_TABLE )
};

static const char* const http_status_codes[ ] =
{
    [HTTP_200_TYPE] = HTTP_HEADER_200,
    [HTTP_204_TYPE] = HTTP_HEADER_204,
    [HTTP_207_TYPE] = HTTP_HEADER_207,
    [HTTP_301_TYPE] = HTTP_HEADER_301,
    [HTTP_400_TYPE] = HTTP_HEADER_400,
    [HTTP_403_TYPE] = HTTP_HEADER_403,
    [HTTP_404_TYPE] = HTTP_HEADER_404,
    [HTTP_405_TYPE] = HTTP_HEADER_405,
    [HTTP_406_TYPE] = HTTP_HEADER_406,
    [HTTP_412_TYPE] = HTTP_HEADER_412,
    [HTTP_415_TYPE] = HTTP_HEADER_406,
    [HTTP_429_TYPE] = HTTP_HEADER_429,
    [HTTP_444_TYPE] = HTTP_HEADER_444,
    [HTTP_470_TYPE] = HTTP_HEADER_470,
    [HTTP_500_TYPE] = HTTP_HEADER_500,
    [HTTP_504_TYPE] = HTTP_HEADER_504
};

static char*  cached_string = NULL;
static size_t cached_length = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_http_server_start( wiced_http_server_t* server, uint16_t port, uint16_t max_sockets, const wiced_http_page_t* page_database, wiced_interface_t interface, uint32_t http_thread_stack_size )
{
    wiced_assert( "bad arg", ( server != NULL ) && ( page_database != NULL ) );

    memset( server, 0, sizeof( *server ) );
    return http_internal_server_start (server,port,max_sockets,page_database,interface, http_thread_stack_size, HTTP_SERVER_CONNECT_THREAD_STACK_SIZE);
}

static wiced_result_t http_internal_server_start(wiced_http_server_t* server, uint16_t port, uint16_t max_sockets, const wiced_http_page_t* page_database, wiced_interface_t interface, uint32_t http_thread_stack_size, uint32_t server_connect_thread_stack_size )
{
    http_stream_node_t* stream_node;
    uint32_t a;

    /* Store the inputs database */
    server->page_database = page_database;

    wiced_http_server_deregister_callbacks( server );

    /* Allocate space for response streams and insert them into the inactive stream list */
    linked_list_init( &server->inactive_stream_list );
    server->streams = malloc_named( "HTTP streams", sizeof(http_stream_node_t) * max_sockets );
    if ( server->streams == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    memset( server->streams, 0, sizeof(http_stream_node_t) * max_sockets );
    stream_node = (http_stream_node_t*)server->streams;
    for ( a = 0; a < max_sockets; a++ )
    {
        linked_list_set_node_data( &stream_node[a].node, (void*)&stream_node[a] );
        linked_list_insert_node_at_rear( &server->inactive_stream_list, &stream_node[a].node );
    }

    /* Create linked-list for holding active response streams */
    linked_list_init( &server->active_stream_list );

    /* Create worker thread to process connect events */
    WICED_VERIFY( wiced_rtos_create_worker_thread( &server->connect_thread, HTTP_SERVER_THREAD_PRIORITY, server_connect_thread_stack_size, EVENT_QUEUE_DEPTH ) );

    /* Initialize HTTP server event queue */
    WICED_VERIFY( wiced_rtos_init_queue( &server->event_queue, NULL, sizeof(server_event_message_t), EVENT_QUEUE_DEPTH ) );

    /* Create HTTP server connect thread */
    WICED_VERIFY( wiced_rtos_create_thread( &server->event_thread, HTTP_SERVER_THREAD_PRIORITY, "HTTPserver", http_server_event_thread_main, http_thread_stack_size, server ) );

    /* Initialise the socket state for all sockets and return */
    return wiced_tcp_server_start( &server->tcp_server, interface, port, max_sockets, http_server_connect_callback, http_server_receive_callback, http_server_disconnect_callback, (void*) server );
}

wiced_result_t wiced_http_server_stop( wiced_http_server_t* server )
{
    server_event_message_t current_event;

    wiced_assert( "bad arg", ( server != NULL ) );

    current_event.event_type = SERVER_STOP_EVENT;
    current_event.socket     = 0;

    HTTP_DEBUG((" %s() : ----- ### Send STOP event\n", __FUNCTION__));
    wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );

    if ( wiced_rtos_is_current_thread( &server->event_thread ) != WICED_SUCCESS )
    {
        /* Wakeup HTTP event thread */
        HTTP_DEBUG((" %s() : ----- ### Wake up ev processor\n", __FUNCTION__));
        wiced_rtos_thread_force_awake( &server->event_thread );
    }

    /* Wait for the event to completely get processed for closing the server smoothly */
    HTTP_DEBUG((" %s() : ----- ### Wait for ev processing to complete\n", __FUNCTION__));
    wiced_rtos_thread_join( &server->event_thread );

    /* Delete the threads */
    HTTP_DEBUG((" %s() : ----- ### Delete ev processing\n", __FUNCTION__));
    wiced_rtos_delete_thread( &server->event_thread );
    wiced_rtos_delete_worker_thread( &server->connect_thread );

    HTTP_DEBUG((" %s() : ----- ### Delete stream mgmt lists\n", __FUNCTION__));
    linked_list_deinit( &server->inactive_stream_list );
    linked_list_deinit( &server->active_stream_list );
    free( server->streams );
    server->streams = NULL;

    HTTP_DEBUG((" %s() : ----- ### Stop even processed successfully\n", __FUNCTION__));
    return WICED_SUCCESS;
}

wiced_result_t wiced_https_server_start( wiced_https_server_t* server, uint16_t port, uint16_t max_sockets, const wiced_http_page_t* page_database, wiced_tls_identity_t* identity, wiced_interface_t interface, uint32_t url_processor_stack_size )
{
    wiced_assert( "bad arg", ( server != NULL ) && ( page_database != NULL ) );

    memset( server, 0, sizeof( *server ) );

    wiced_tcp_server_enable_tls( &server->tcp_server, identity );

    /* Start HTTP server */
    WICED_VERIFY( http_internal_server_start( server, port, max_sockets, page_database, interface, url_processor_stack_size, HTTPS_SERVER_CONNECT_THREAD_STACK_SIZE) );

    /* Enable TLS */
    return WICED_SUCCESS;
}

wiced_result_t wiced_https_server_stop( wiced_https_server_t* server )
{
    return wiced_http_server_stop( server );
}

wiced_result_t wiced_http_server_register_callbacks( wiced_http_server_t* server, http_server_receive_callback_t receive_callback, http_server_disconnect_callback_t disconnect_callback )
{
    wiced_assert( "bad arg", ( server != NULL ) );

    server->receive_callback = receive_callback;

    server->disconnect_callback = disconnect_callback;

    return WICED_SUCCESS;
}

wiced_result_t wiced_http_server_deregister_callbacks( wiced_http_server_t* server )
{
    wiced_assert( "bad arg", ( server != NULL ) );

    server->receive_callback = NULL;

    server->disconnect_callback = NULL;

    return WICED_SUCCESS;
}

wiced_result_t wiced_http_response_stream_enable_chunked_transfer( wiced_http_response_stream_t* stream )
{
    wiced_assert( "bad arg", ( stream !=  NULL ) );

    stream->chunked_transfer_enabled = WICED_TRUE;
    return WICED_SUCCESS;
}

wiced_result_t wiced_http_response_stream_disable_chunked_transfer( wiced_http_response_stream_t* stream )
{
    wiced_assert( "bad arg", ( stream !=  NULL ) );

    if ( stream->chunked_transfer_enabled == WICED_TRUE )
    {
        /* Send final chunked frame */
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, FINAL_CHUNKED_PACKET, sizeof( FINAL_CHUNKED_PACKET ) - 1 ) );
    }

    stream->chunked_transfer_enabled = WICED_FALSE;
    return WICED_SUCCESS;
}

wiced_result_t wiced_http_response_stream_write_header( wiced_http_response_stream_t* stream, http_status_codes_t status_code, uint32_t content_length, http_cache_t cache_type, wiced_packet_mime_type_t mime_type )
{
    char data_length_string[ 15 ];

    wiced_assert( "bad arg", ( stream != NULL ) );

    memset( data_length_string, 0x0, sizeof( data_length_string ) );

    /*HTTP/1.1 <status code>\r\n*/
    WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, http_status_codes[ status_code ], strlen( http_status_codes[ status_code ] ) ) );
    WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, sizeof( CRLF )-1 ) );

    /* Content-Type: xx/yy\r\n */
    WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, HTTP_HEADER_CONTENT_TYPE, strlen( HTTP_HEADER_CONTENT_TYPE ) ) );
    WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, http_mime_array[ mime_type ], strlen( http_mime_array[ mime_type ] ) ) );
    WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, strlen( CRLF ) ) );

    if ( cache_type == HTTP_CACHE_DISABLED )
    {
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, NO_CACHE_HEADER, strlen( NO_CACHE_HEADER ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, strlen( CRLF ) ) );
    }

    if ( status_code == HTTP_444_TYPE )
    {
        /* Connection: close */
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, HTTP_HEADER_CLOSE, strlen( HTTP_HEADER_CLOSE ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, strlen( CRLF ) ) );
    }
    else
    {
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, HTTP_HEADER_KEEP_ALIVE, strlen( HTTP_HEADER_KEEP_ALIVE ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, strlen( CRLF ) ) );
    }

    if ( stream->chunked_transfer_enabled == WICED_TRUE )
    {
        /* Chunked transfer encoding */
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, HTTP_HEADER_CHUNKED, strlen( HTTP_HEADER_CHUNKED ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, strlen( CRLF ) ) );
    }
    else
    {
        /* for EVENT Stream content length is  Zero*/
        if ( mime_type != MIME_TYPE_TEXT_EVENT_STREAM )
        {
            /* Content-Length: xx\r\n */
            sprintf( data_length_string, "%lu", (long) content_length );
            WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, HTTP_HEADER_CONTENT_LENGTH, strlen( HTTP_HEADER_CONTENT_LENGTH ) ) );
            WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, data_length_string, strlen( data_length_string ) ) );
            WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, strlen( CRLF ) ) );
        }
    }

    /* Closing sequence */
    WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, strlen( CRLF ) ) );

    return WICED_SUCCESS;
}

wiced_result_t wiced_http_response_stream_write( wiced_http_response_stream_t* stream, const void* data, uint32_t length )
{
    wiced_assert( "bad arg", ( stream !=  NULL ) );

    if ( length == 0 )
    {
        return WICED_BADARG;
    }

    if ( stream->chunked_transfer_enabled == WICED_TRUE )
    {
        char data_length_string[10];
        memset( data_length_string, 0x0, sizeof( data_length_string ) );
        sprintf( data_length_string, "%lx", (long unsigned int)length );
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, data_length_string, strlen( data_length_string ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, sizeof( CRLF ) - 1 ) );
    }

    WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, data, length ) );

    if ( stream->chunked_transfer_enabled == WICED_TRUE )
    {
        WICED_VERIFY( wiced_tcp_stream_write( &stream->tcp_stream, CRLF, sizeof( CRLF ) - 1 ) );
    }
    return WICED_SUCCESS;
}

wiced_result_t wiced_http_response_stream_write_resource( wiced_http_response_stream_t* stream, const resource_hnd_t* res_id )
{
    wiced_result_t result;
    const void*    data;
    uint32_t       res_size;
    uint32_t       pos = 0;

    do
    {
        resource_result_t resource_result = resource_get_readonly_buffer ( res_id, pos, 0x7fffffff, &res_size, &data );
        if ( resource_result != RESOURCE_SUCCESS )
        {
            return resource_result;
        }

        result = wiced_http_response_stream_write( stream, data, res_size );
        resource_free_readonly_buffer( res_id, data );
        if ( result != WICED_SUCCESS )
        {
            return result;
        }
        pos += res_size;
    } while ( res_size > 0 );

    return result;
}

wiced_result_t wiced_http_response_stream_flush( wiced_http_response_stream_t* stream )
{
    wiced_assert( "bad arg", ( stream != NULL ) );

    return wiced_tcp_stream_flush( &stream->tcp_stream );
}

wiced_result_t wiced_http_response_stream_disconnect( wiced_http_response_stream_t* stream )
{
    server_event_message_t current_event;
    wiced_http_server_t*   server;

    wiced_assert( "bad arg", ( stream != NULL ) && ( stream->tcp_stream.socket != NULL ) );
    if ((stream == NULL) || (stream->tcp_stream.socket == NULL ))
    {
        return WICED_BADARG;
    }

    HTTP_DEBUG((" %s() : ----- ### DBG : Stream = [0x%X]\n", __FUNCTION__, (unsigned int)stream));

    current_event.event_type = SOCKET_DISCONNECT_EVENT;
    current_event.socket     = stream->tcp_stream.socket;

    server = (wiced_http_server_t*)stream->tcp_stream.socket->callback_arg;

    return wiced_rtos_push_to_queue( &server->event_queue, &current_event, WICED_NO_WAIT );
}

wiced_result_t wiced_http_disconnect_all_response_stream( wiced_https_server_t* server )
{
    http_stream_node_t* stream;

    linked_list_get_front_node( &server->active_stream_list, (linked_list_node_t**)&stream );
    while ( stream != NULL )
    {
        wiced_http_response_stream_disconnect(&stream->stream.response);
        stream = (http_stream_node_t*)stream->node.next;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_http_response_stream_init( wiced_http_response_stream_t* stream, wiced_tcp_socket_t* socket )
{
    wiced_assert( "bad arg", ( stream != NULL ) );

    memset( stream, 0, sizeof( wiced_http_response_stream_t ) );

    stream->chunked_transfer_enabled = WICED_FALSE;

    return wiced_tcp_stream_init( &stream->tcp_stream, socket );
}

wiced_result_t wiced_http_response_stream_deinit( wiced_http_response_stream_t* stream )
{
    wiced_result_t  result;

    wiced_assert( "bad arg", ((stream != NULL) && (stream->tcp_stream.socket != NULL)));
    if ((stream == NULL) || (stream->tcp_stream.socket == NULL))
    {
        return WICED_BADARG;
    }

    HTTP_DEBUG((" %s() : ----- ### DBG : Stream = [0x%X]\n", __FUNCTION__, (unsigned int)stream));
    result = wiced_tcp_stream_deinit( &stream->tcp_stream );
    return result;
}

wiced_result_t wiced_http_get_query_parameter_value( const char* url_query, const char* parameter_key, char** parameter_value, uint32_t* value_length )
{
    char* iterator = (char*)url_query;

    while ( *iterator != '\0' )
    {
        char*    current_key = iterator;
        uint32_t current_key_length;

        while( ( *iterator != '\0' ) && ( *iterator != '=' ) && ( *iterator != '&' ) )
        {
            iterator++;
        }

        current_key_length = (uint32_t)( iterator - current_key );

        if ( match_string_with_wildcard_pattern( current_key, current_key_length, parameter_key ) != 0 )
        {
            if ( *iterator == '=' )
            {
                *parameter_value = iterator + 1;
                while( *iterator != '\0' && *iterator != '&' )
                {
                    iterator++;
                }
                *value_length = (uint32_t)( iterator - *parameter_value );
            }
            else
            {
                *parameter_value = NULL;
                *value_length    = 0;
            }
            return WICED_SUCCESS;
        }
        else
        {
            iterator++;
        }
    }

    *parameter_value = NULL;
    *value_length    = 0;
    return WICED_NOT_FOUND;
}

uint32_t wiced_http_get_query_parameter_count( const char* url_query )
{
    char*    current_query = (char*) url_query;
    uint32_t count;

    if ( current_query == NULL )
    {
        return 0;
    }

    /* Non-NULL URL query is considered 1 parameter */
    count = 1;

    while ( *current_query != '\0' )
    {
        /* Count up everytime '&' is found */
        if ( *current_query == '&' )
        {
            count++;
        }

        current_query++;
    }

    return count;
}

wiced_result_t wiced_http_match_query_parameter( const char* url_query, const char* parameter_key, const char* parameter_value )
{
    wiced_result_t result;
    char*          value_found;
    uint32_t       value_length;

    result = wiced_http_get_query_parameter_value( url_query, parameter_key, &value_found, &value_length );
    if ( result == WICED_SUCCESS )
    {
        if ( strncmp( parameter_value, value_found, value_length ) != 0 )
        {
            result = WICED_NOT_FOUND;
        }
    }

    return result;
}

static wiced_result_t http_server_connect_callback( wiced_tcp_socket_t* socket, void* arg )
{
    wiced_http_server_t* server = (wiced_http_server_t*)arg;

    return wiced_rtos_send_asynchronous_event( &server->connect_thread, http_server_deferred_connect_callback, (void*)socket );
}

static wiced_result_t http_server_disconnect_callback( wiced_tcp_socket_t* socket, void* arg )
{
    server_event_message_t current_event;

    HTTP_DEBUG((" %s() : ----- ### DBG : Socket = [0x%X]\n", __FUNCTION__, (unsigned int)socket));

    current_event.event_type = SOCKET_DISCONNECT_EVENT;
    current_event.socket     = socket;
    return wiced_rtos_push_to_queue( &((wiced_http_server_t*)arg)->event_queue, &current_event, WICED_NO_WAIT );
}

static wiced_result_t http_server_receive_callback( wiced_tcp_socket_t* socket, void* arg )
{
    server_event_message_t current_event;

    current_event.event_type = SOCKET_PACKET_RECEIVED_EVENT;
    current_event.socket     = socket;
    return wiced_rtos_push_to_queue( &((wiced_http_server_t*)arg)->event_queue, &current_event, WICED_NO_WAIT );
}

static wiced_result_t http_server_parse_receive_packet( wiced_http_server_t* server, wiced_http_stream_t* stream, wiced_packet_t* packet )
{
    wiced_result_t result                        = WICED_SUCCESS;
    wiced_bool_t   disconnect_current_connection = WICED_FALSE;
    char*          start_of_url                  = NULL; /* Suppress compiler warning */
    uint16_t       url_length                    = 0;    /* Suppress compiler warning */
    char*          request_string = NULL;
    uint16_t       request_length;
    uint16_t       new_url_length;
    uint16_t       available_data_length;
    char*          message_data_length_string;
    char*          mime;
    char*          cached_string_to_be_freed = NULL;

    wiced_http_message_body_t http_message_body =
    {
        .data                         = NULL,
        .message_data_length          = 0,
        .total_message_data_remaining = 0,
        .chunked_transfer             = WICED_FALSE,
        .mime_type                    = MIME_UNSUPPORTED,
        .request_type                 = REQUEST_UNDEFINED
    };

    if ( packet == NULL )
    {
        return WICED_ERROR;
    }

    /* Get URL request string from the receive packet */
    result = wiced_packet_get_data( packet, 0, (uint8_t**)&request_string, &request_length, &available_data_length );
    if ( result != WICED_SUCCESS )
    {
        disconnect_current_connection = WICED_TRUE;
        goto exit;
    }
    if ( request_length != available_data_length )
    {
        /* We don't support fragmented packets */
        goto exit;
    }

    /* If application registers a receive callback, call the callback before further processing */
    if ( server->receive_callback != NULL )
    {
        HTTP_DEBUG((" ####### HTTP Request Received : Payload-Len = [%d], Payload = [%.*s]\n", available_data_length, available_data_length, request_string));
#ifdef ENABLE_DEBUG
        {
            int i;

            HTTP_DEBUG(("Payload in Hex = ["));
            for (i = 0; i < available_data_length; i++)
            {
                HTTP_DEBUG(("%X", request_string[i]));
            }
            HTTP_DEBUG(("]\n"));
        }
#endif

        result = server->receive_callback( &stream->response, (uint8_t**)&request_string, &request_length );
        if ( result != WICED_SUCCESS )
        {
            if ( result != WICED_PARTIAL_RESULTS )
            {
                disconnect_current_connection = WICED_TRUE;
            }
            goto exit;
        }
    }

    /* Check if this is a close request */
    if ( strnstrn( request_string, request_length, HTTP_HEADER_CLOSE, sizeof( HTTP_HEADER_CLOSE ) - 1 ) != NULL )
    {
        disconnect_current_connection = WICED_TRUE;
    }

    /* Code allows to support if content length > MTU then send data to callback registered for particular page_found. */
    if(stream->request.page_found != NULL)
    {
        /* currently we only handle content length > MTU for RAW_DYNAMIC_URL_CONTENT and WICED_DYNAMIC_CONTENT */
        if(stream->request.page_found->url_content_type == WICED_RAW_DYNAMIC_URL_CONTENT || stream->request.page_found->url_content_type == WICED_DYNAMIC_URL_CONTENT)
        {
            if(stream->request.total_data_remaning > 0)
            {
                stream->request.total_data_remaning = (uint16_t)(stream->request.total_data_remaning -  request_length);

                http_message_body.data = (uint8_t*)request_string;
                http_message_body.message_data_length = request_length;
                http_message_body.total_message_data_remaining = stream->request.total_data_remaning;
                http_message_body.mime_type = stream->request.mime_type;
                http_message_body.request_type = stream->request.request_type;

                HTTP_DEBUG((" %s() : ----- ### DBG : Invoking the URL generator to process partial data\n", __FUNCTION__));
                stream->request.page_found->url_content.dynamic_data.generator(stream->request.page_found->url, NULL, &stream->response, stream->request.page_found->url_content.dynamic_data.arg, &http_message_body);

                /* We got all fragmented packets of http request [as total_data_remaining is 0 ] now send the response */
                if(stream->request.total_data_remaning == 0)
                {
                   /* Disable chunked transfer as it was enabled previously in library itself and then flush the data */
                   if( stream->request.page_found->url_content_type == WICED_DYNAMIC_URL_CONTENT )
                   {
                       wiced_http_response_stream_disable_chunked_transfer( &stream->response );
                   }

                   WICED_VERIFY( wiced_http_response_stream_flush( &stream->response ) );
                }

                return WICED_SUCCESS;
            }
        }
    }

    if ((cached_string != NULL) || (strnstrn(request_string, request_length, CRLF_CRLF, sizeof(CRLF_CRLF) - 1) == NULL))
    {
      char* new_cached_string;
      size_t new_cached_length = cached_length + request_length;

      HTTP_DEBUG((" %s() : ----- ### DBG : Caching the request\n", __FUNCTION__));

      if (new_cached_length > MAXIMUM_CACHED_LENGTH) {
          HTTP_DEBUG((" %s() : ----- ### DBG : Request exceeds %d bytes\n", __FUNCTION__, MAXIMUM_CACHED_LENGTH));
          free(cached_string);
          cached_string = NULL;
          cached_length = 0;
          goto exit;
      }

      new_cached_string = realloc(cached_string, new_cached_length);

      if (new_cached_string == NULL)
      {
          HTTP_DEBUG((" %s() : ----- ### DBG : Not enough memory\n", __FUNCTION__));
          free(cached_string);
          cached_string = NULL;
          cached_length = 0;
          goto exit;
      }

      memcpy(new_cached_string + cached_length, request_string, request_length);

      cached_string = new_cached_string;
      cached_length = new_cached_length;

      if (strnstrn(cached_string, (uint16_t)cached_length, CRLF_CRLF, sizeof(CRLF_CRLF) - 1) == NULL)
      {
          HTTP_DEBUG((" %s() : ----- ### DBG : Not found the end of request\n", __FUNCTION__));
          goto exit;
      }
      else
      {
          request_string = cached_string;
          request_length = (uint16_t)cached_length;

          cached_string_to_be_freed = cached_string;

          cached_string = NULL;
          cached_length = 0;
      }
    }

    /* Verify we have enough data to start processing */
    if ( request_length < MINIMUM_REQUEST_LINE_LENGTH )
    {
        result = WICED_ERROR;
        goto exit;
    }

    /* First extract the URL from the packet */
    HTTP_DEBUG((" %s() : ----- ### DBG : Extract the request type\n", __FUNCTION__));
    result = http_server_get_request_type_and_url( request_string, request_length, &http_message_body.request_type, &start_of_url, &url_length );
    if ( result == WICED_ERROR )
    {
        goto exit;
    }

    /* Remove escape strings from URL */
    new_url_length = http_server_remove_escaped_characters( start_of_url, url_length, start_of_url, url_length );

    /* Now extract packet payload info such as data, data length, data type and message length */
    HTTP_DEBUG((" %s() : ----- ### DBG : Extract payload\n", __FUNCTION__));
    http_message_body.data = (uint8_t*) strnstrn( request_string, request_length, CRLF_CRLF, sizeof( CRLF_CRLF ) - 1 );

    /* This indicates start of data/end of header was not found, so exit */
    if ( http_message_body.data == NULL )
    {
        result = WICED_ERROR;
        goto exit;
    }
    else
    {
        /* Payload starts just after the header */
        http_message_body.data += strlen( CRLF_CRLF );

        /* if there is no payload after header just set data pointer to NULL */
        if ( ( (uint8_t*) ( request_string + request_length ) - http_message_body.data ) == 0 )
        {
            http_message_body.data = NULL;
            http_message_body.message_data_length = 0;
        }
    }

    HTTP_DEBUG((" %s() : ----- ### DBG : Extract content type\n", __FUNCTION__));
    mime = strnstrn( request_string, request_length, HTTP_HEADER_CONTENT_TYPE, sizeof( HTTP_HEADER_CONTENT_TYPE ) - 1 );
    if ( ( mime != NULL ) && ( mime < (char*) http_message_body.data ) )
    {
        mime += strlen( HTTP_HEADER_CONTENT_TYPE );
        http_message_body.mime_type = http_server_get_mime_type( mime );
    }
    else
    {
        http_message_body.mime_type = MIME_TYPE_ALL;
    }

    if ( strnstrn( request_string, request_length, HTTP_HEADER_CHUNKED, sizeof( HTTP_HEADER_CHUNKED ) - 1 ) )
    {
        /* Indicate the format of this frame is chunked. Its up to the application to parse and reassemble the chunk */
        http_message_body.chunked_transfer = WICED_TRUE;
        if ( http_message_body.data != NULL )
        {
            http_message_body.message_data_length = (uint16_t) ( (uint8_t*) ( request_string + request_length ) - http_message_body.data );
        }
    }
    else
    {
        message_data_length_string = strnstrn( request_string, request_length, HTTP_HEADER_CONTENT_LENGTH, sizeof( HTTP_HEADER_CONTENT_LENGTH ) - 1 );

        /* This case handles case where content-length : X but there is no data present in payload, in this case atleast application should be informed about correct
         * remaining data length so application can take appropriate action
         */
        if ( ( message_data_length_string != NULL ) && ( http_message_body.data == NULL ) )
        {
            message_data_length_string += ( sizeof( HTTP_HEADER_CONTENT_LENGTH ) - 1 );

            http_message_body.total_message_data_remaining = (uint16_t) ( strtol( message_data_length_string, NULL, 10 ) - http_message_body.message_data_length );

            stream->request.total_data_remaning = http_message_body.total_message_data_remaining;
        }
        else if ( ( message_data_length_string != NULL ) && ( message_data_length_string < (char*) http_message_body.data ) )
        {
            http_message_body.message_data_length = (uint16_t) ( (uint8_t*) ( request_string + request_length ) - http_message_body.data );

            message_data_length_string += ( sizeof( HTTP_HEADER_CONTENT_LENGTH ) - 1 );

            http_message_body.total_message_data_remaining = (uint16_t) ( strtol( message_data_length_string, NULL, 10 ) - http_message_body.message_data_length );

            stream->request.total_data_remaning = http_message_body.total_message_data_remaining;
        }
        else
        {
            http_message_body.message_data_length = 0;
            stream->request.total_data_remaning = 0;
        }
        stream->request.mime_type = http_message_body.mime_type;
        stream->request.request_type = http_message_body.request_type;
    }

    HTTP_DEBUG((" %s() : ----- ### DBG : Process the URL request\n", __FUNCTION__));
    result = http_server_process_url_request( stream, server->page_database, start_of_url, new_url_length, &http_message_body );

exit:
    free(cached_string_to_be_freed);

    if ( disconnect_current_connection == WICED_TRUE )
    {
        wiced_http_response_stream_disconnect( &stream->response );
    }

    return result;
}

static wiced_result_t http_server_process_url_request( wiced_http_stream_t* stream, const wiced_http_page_t* page_database, char* url, uint32_t url_length, wiced_http_message_body_t* http_message_body )
{
    char*                    url_query_parameters = url;
    uint32_t                 query_length         = url_length;
    wiced_http_page_t*       page_found           = NULL;
    wiced_packet_mime_type_t mime_type            = MIME_TYPE_ALL;
    http_status_codes_t      status_code;
    wiced_result_t           result = WICED_SUCCESS;

    url[ url_length ] = '\x00';

    while ( ( *url_query_parameters != '?' ) && ( query_length > 0 ) && ( *url_query_parameters != '\0' ) )
    {
        url_query_parameters++;
        query_length--;
    }

    if ( query_length != 0 )
    {
        url_length = url_length - query_length;
        *url_query_parameters = '\x00';
        url_query_parameters++;
    }
    else
    {
        url_query_parameters = NULL;
    }

    WPRINT_WEBSERVER_DEBUG( ("Processing request for: %s\n", url) );

    /* Find URL in server page database */
    if ( http_server_find_url_in_page_database( url, url_length, http_message_body, page_database, &page_found, &mime_type ) == WICED_SUCCESS )
    {
        stream->request.page_found = page_found;
        status_code = HTTP_200_TYPE; /* OK */
    }
    else
    {
        stream->request.page_found = NULL;
        status_code = HTTP_404_TYPE; /* Not Found */
    }

    if ( status_code == HTTP_200_TYPE )
    {
        HTTP_DEBUG((" %s() : ----- ### DBG : URL content type = [%d]\n", __FUNCTION__, page_found->url_content_type));
        /* Call the content handler function to write the page content into the packet and adjust the write pointers */
        switch ( page_found->url_content_type )
        {
            case WICED_DYNAMIC_URL_CONTENT:
                HTTP_DEBUG((" %s() : ----- ### DBG : WICED_DYNAMIC_URL_CONTENT\n", __FUNCTION__));
                wiced_http_response_stream_enable_chunked_transfer( &stream->response );
                wiced_http_response_stream_write_header( &stream->response, status_code, CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, mime_type );
                result = page_found->url_content.dynamic_data.generator( url, url_query_parameters, &stream->response, page_found->url_content.dynamic_data.arg, http_message_body );
                /* if content length is < MTU then just disable chunked transfer and flush the data */
                if(stream->request.total_data_remaning == 0)
                {
                    wiced_http_response_stream_disable_chunked_transfer( &stream->response );
                    WICED_VERIFY( wiced_http_response_stream_flush( &stream->response ) );
                }
                break;

            case WICED_RAW_DYNAMIC_URL_CONTENT:
                HTTP_DEBUG((" %s() : ----- ### DBG : WICED_RAW_DYNAMIC_URL_CONTENT\n", __FUNCTION__));
                result = page_found->url_content.dynamic_data.generator( url, url_query_parameters, &stream->response, page_found->url_content.dynamic_data.arg, http_message_body );
                /* if content length is < MTU then just flush the response */
                if(stream->request.total_data_remaning == 0)
                {
                    WICED_VERIFY( wiced_http_response_stream_flush( &stream->response ) );
                }
                break;

            case WICED_STATIC_URL_CONTENT:
                HTTP_DEBUG((" %s() : ----- ### DBG : WICED_STATIC_URL_CONTENT\n", __FUNCTION__));
                wiced_http_response_stream_write_header( &stream->response, status_code, page_found->url_content.static_data.length, HTTP_CACHE_ENABLED, mime_type );
                wiced_http_response_stream_write( &stream->response, page_found->url_content.static_data.ptr, page_found->url_content.static_data.length );
                WICED_VERIFY( wiced_http_response_stream_flush( &stream->response ) );
                break;

            case WICED_RAW_STATIC_URL_CONTENT: /* This is just a Location header */
                HTTP_DEBUG((" %s() : ----- ### DBG : WICED_RAW_STATIC_URL_CONTENT\n", __FUNCTION__));
                wiced_http_response_stream_write( &stream->response, HTTP_HEADER_301, strlen( HTTP_HEADER_301 ) );
                wiced_http_response_stream_write( &stream->response, CRLF, strlen( CRLF ) );
                wiced_http_response_stream_write( &stream->response, HTTP_HEADER_LOCATION, strlen( HTTP_HEADER_LOCATION ) );
                wiced_http_response_stream_write( &stream->response, page_found->url_content.static_data.ptr, page_found->url_content.static_data.length );
                wiced_http_response_stream_write( &stream->response, CRLF, strlen( CRLF ) );
                wiced_http_response_stream_write( &stream->response, HTTP_HEADER_CONTENT_LENGTH, strlen( HTTP_HEADER_CONTENT_LENGTH ) );
                wiced_http_response_stream_write( &stream->response, "0", 1 );
                wiced_http_response_stream_write( &stream->response, CRLF_CRLF, strlen( CRLF_CRLF ) );
                WICED_VERIFY( wiced_http_response_stream_flush( &stream->response ) );
                break;

            case WICED_RESOURCE_URL_CONTENT:
                HTTP_DEBUG((" %s() : ----- ### DBG : WICED_RESOURCE_URL_CONTENT\n", __FUNCTION__));
                /* Fall through */
            case WICED_RAW_RESOURCE_URL_CONTENT:
                HTTP_DEBUG((" %s() : ----- ### DBG : WICED_RAW_RESOURCE_URL_CONTENT\n", __FUNCTION__));
                wiced_http_response_stream_enable_chunked_transfer( &stream->response );
                wiced_http_response_stream_write_header( &stream->response, status_code, CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, mime_type );
                wiced_http_response_stream_write_resource( &stream->response, page_found->url_content.resource_data );
                wiced_http_response_stream_disable_chunked_transfer( &stream->response );
                WICED_VERIFY( wiced_http_response_stream_flush( &stream->response ) );
                break;

            default:
                HTTP_DEBUG((" %s() : ----- ### DBG : default\n", __FUNCTION__));
                wiced_assert("Unknown entry in URL list", 0 != 0 );
                break;
        }
    }
    else if ( status_code >= HTTP_400_TYPE )
    {
        wiced_http_response_stream_write_header( &stream->response, status_code, NO_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_TEXT_HTML );
        WICED_VERIFY( wiced_http_response_stream_flush( &stream->response ) );
    }

    wiced_assert( "Page Serve finished with data still in stream", stream->response.tcp_stream.tx_packet == NULL );
    return result;
}

static uint16_t http_server_remove_escaped_characters( char* output, uint16_t output_length, const char* input, uint16_t input_length )
{
    uint16_t bytes_copied;
    int a;

    for ( bytes_copied = 0; ( input_length > 0 ) && ( bytes_copied != output_length ); ++bytes_copied )
    {
        if ( *input == '%' )
        {
            --input_length;
            /* If there is only % remain in encoded URL string, then just return as it is not valid */
            if ( input_length == 0 )
            {
                return 0;
            }

            ++input;

            /* Valid encoding should result as % followed by two hexadecimal digits. If encoded URL comes with %% then just return as it is invalid */
            if ( *input == '%' )
            {
                return 0;
            }
            else
            {
                *output = 0;

                /* As per RFC 3986 : percent encoding octet is encoded as character triplet, consisting of "%" followed by two hexadecimal digits. After percentage if we are not able to
                 * find two hexadecimal digits then return as it is invalid encoded URL */
                if (input_length < 2)
                {
                    return 0;
                }

                for ( a = 0; (a < 2); ++a )
                {
                    *output = (char) ( *output << 4 );
                    if ( *input >= '0' && *input <= '9' )
                    {
                        *output = (char) ( *output + *input - '0' );
                    }
                    else if ( *input >= 'a' && *input <= 'f' )
                    {
                        *output = (char) ( *output + *input - 'a' + 10 );
                    }
                    else if ( *input >= 'A' && *input <= 'F' )
                    {
                        *output = (char) ( *output + *input - 'A' + 10 );
                    }
                    else
                    {
                        return 0;
                    }
                    --input_length;
                    if ( input_length > 0 )
                    {
                        ++input;
                    }
                }
                ++output;
            }
        }
        else
        {
            /* If there is + present in encoded URL then replace with space */
            if( *input == '+' )
            {
                *output = ' ';
            }
            else
            {
                *output = *input;
            }

            --input_length;
            if ( input_length > 0 )
            {
                ++input;
                ++output;
            }
        }
    }

    return bytes_copied;
}

static wiced_packet_mime_type_t http_server_get_mime_type( const char* request_data )
{
    wiced_packet_mime_type_t mime_type = MIME_TYPE_TLV;

    if ( request_data != NULL )
    {
        while ( mime_type < MIME_TYPE_ALL )
        {
            if ( strncmp( request_data, http_mime_array[ mime_type ], strlen( http_mime_array[ mime_type ] ) ) == COMPARE_MATCH )
            {
                break;
            }
            mime_type++;
        }
    }
    else
    {
        /* If MIME not specified, assumed all supported (according to rfc2616)*/
        mime_type = MIME_TYPE_ALL;
    }
    return mime_type;
}

static wiced_result_t http_server_get_request_type_and_url( char* request, uint16_t request_length, wiced_http_request_type_t* type, char** url_start, uint16_t* url_length )
{
    char* end_of_url;

    end_of_url = strnstrn( request, request_length, HTTP_1_1_TOKEN, sizeof( HTTP_1_1_TOKEN ) - 1 );
    if ( end_of_url == NULL )
    {
        return WICED_ERROR;
    }

    if ( memcmp( request, GET_TOKEN, sizeof( GET_TOKEN ) - 1 ) == COMPARE_MATCH )
    {
        /* Get type  */
        *type = WICED_HTTP_GET_REQUEST;
        *url_start = request + sizeof( GET_TOKEN ) - 1;
        *url_length = (uint16_t) ( end_of_url - *url_start );
    }
    else if ( memcmp( request, POST_TOKEN, sizeof( POST_TOKEN ) - 1 ) == COMPARE_MATCH )
    {
        *type = WICED_HTTP_POST_REQUEST;
        *url_start = request + sizeof( POST_TOKEN ) - 1;
        *url_length = (uint16_t) ( end_of_url - *url_start );
    }
    else if ( memcmp( request, PUT_TOKEN, sizeof( PUT_TOKEN ) - 1 ) == COMPARE_MATCH )
    {
        *type = WICED_HTTP_PUT_REQUEST;
        *url_start = request + sizeof( PUT_TOKEN ) - 1;
        *url_length = (uint16_t) ( end_of_url - *url_start );
    }
    else
    {
        *type = REQUEST_UNDEFINED;
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t http_server_find_url_in_page_database( char* url, uint32_t length, wiced_http_message_body_t* http_request, const wiced_http_page_t* page_database, wiced_http_page_t** page_found, wiced_packet_mime_type_t* mime_type )
{
    uint32_t i = 0;

    /* Search URL list to determine if request matches one of our pages, and break out when found */
    while ( page_database[ i ].url != NULL )
    {
        if ( match_string_with_wildcard_pattern( url, length, page_database[ i ].url ) != 0 )
        {
            *mime_type = http_server_get_mime_type( page_database[ i ].mime_type );

            if ( ( *mime_type == http_request->mime_type ) || ( http_request->mime_type == MIME_TYPE_ALL ) )
            {
                *page_found = (wiced_http_page_t*)&page_database[i];
                return WICED_SUCCESS;
            }
        }
        i++;
    }

    return WICED_NOT_FOUND;
}

static void http_server_event_thread_main( uint32_t arg )
{
    wiced_http_server_t*   server = (wiced_http_server_t*) arg;
    server_event_message_t current_event;
    wiced_result_t         result;

    while ( server->quit != WICED_TRUE )
    {
        result = wiced_rtos_pop_from_queue( &server->event_queue, &current_event, WICED_NEVER_TIMEOUT );

        if ( result != WICED_SUCCESS )
        {
            current_event.socket     = NULL;
            current_event.event_type = SOCKET_ERROR_EVENT;
        }

        HTTP_DEBUG((" L%d : %s() : ----- Current event = [%d]\n", __LINE__, __FUNCTION__, current_event.event_type));
        switch ( current_event.event_type )
        {
            case SOCKET_DISCONNECT_EVENT:
            {
                http_stream_node_t* stream;

                /* Search in active stream whether stream for this socket is available. If available, removed it */
                HTTP_DEBUG((" ### DBG : Search Stream to be removed\n"));
                if ( linked_list_find_node( &server->active_stream_list, http_server_compare_stream_socket, (void*)current_event.socket, (linked_list_node_t**)&stream ) == WICED_SUCCESS )
                {
                    HTTP_DEBUG((" %s() : ----- ### DBG : Found Stream to be removed = [0x%X], Socket = [0x%X]\n", __FUNCTION__, (unsigned int)stream, (unsigned int)current_event.socket));
                    HTTP_DEBUG((" L%d : %s() : ----- Disconnect event for Stream = [0x%X]\n", __LINE__, __FUNCTION__, (unsigned int)&stream->stream.response));
                    /* If application registers a disconnection callback, call the callback before further processing */
                    if( server->disconnect_callback != NULL )
                    {
                        server->disconnect_callback( &stream->stream.response );
                    }

                    linked_list_remove_node( &server->active_stream_list, &stream->node );
                    linked_list_insert_node_at_rear( &server->inactive_stream_list, &stream->node );
                    wiced_http_response_stream_deinit( &stream->stream.response );
                }
                else
                {
                    /* Check for NULL sockets in the active stream list and remove them */
                    uint32_t        active_stream_cnt;
                    wiced_result_t  find_null_socket = WICED_NOT_FOUND;

                    HTTP_DEBUG((" L%d : %s() : ----- Check and remove invalid streams\n", __LINE__, __FUNCTION__));
                    for (active_stream_cnt = server->active_stream_list.count; active_stream_cnt>0; active_stream_cnt-- )
                    {
                        find_null_socket = linked_list_find_node(&server->active_stream_list, http_server_compare_stream_socket, (void*)NULL, (linked_list_node_t**)&stream );
                        /* if all active stream has valid socket, stop search */
                        if ( find_null_socket != WICED_SUCCESS )
                        {
                            break;
                        }

                        /* If application registers a disconnection callback, call the callback before further processing */
                        HTTP_DEBUG((" L%d : %s() : ----- Disconnect event for Stream = [0x%X]\n", __LINE__, __FUNCTION__, (unsigned int)&stream->stream.response));
                        if( server->disconnect_callback != NULL )
                        {
                            server->disconnect_callback( &stream->stream.response );
                        }

                        /* active stream list must have valid socket */
                        /* re-claim from active stream if socket is null */
                        HTTP_DEBUG(("found null socket in active list, active.cnt=%ld, inactive.cnt=%ld\n",
                            server->active_stream_list.count,
                            server->inactive_stream_list.count));
                        linked_list_remove_node( &server->active_stream_list, &stream->node );
                        linked_list_insert_node_at_rear( &server->inactive_stream_list, &stream->node );
                    }
                }

                wiced_tcp_server_disconnect_socket( &server->tcp_server, current_event.socket );
                if ( current_event.socket->tls_context != NULL && current_event.socket->context_malloced == WICED_TRUE )
                {
                    wiced_tls_deinit_context( current_event.socket->tls_context );
                }

                break;
            }
            case SERVER_STOP_EVENT:
            {
                http_stream_node_t* stream;

                server->quit = WICED_TRUE;

                /* Deinit all response stream */
                linked_list_get_front_node( &server->active_stream_list, (linked_list_node_t**)&stream );
                while ( stream != NULL )
                {
                    linked_list_remove_node( &server->active_stream_list, &stream->node );
                    HTTP_DEBUG((" %s() : ----- De-Init Stream = [0x%X]\n", __FUNCTION__, (unsigned int)&stream->stream.response));
                    wiced_http_response_stream_deinit( &stream->stream.response );
                    linked_list_get_front_node( &server->active_stream_list, (linked_list_node_t**)&stream );
                }
                HTTP_DEBUG((" ### DBG : Stop event processed\n"));
                break;
            }

            case SOCKET_PACKET_RECEIVED_EVENT:
            {
                wiced_tcp_socket_t*     socket = (wiced_tcp_socket_t*)current_event.socket;
                wiced_packet_t*         packet = NULL;
                http_stream_node_t*     stream = NULL;
                wiced_result_t          res;

                wiced_tcp_receive( socket, &packet, HTTP_SERVER_RECEIVE_TIMEOUT );
                if ( packet != NULL )
                {
                    /* Search if a stream has been created for this socket. If not found, create and insert one to active list */
                    if ( linked_list_find_node( &server->active_stream_list, http_server_compare_stream_socket, (void*)socket, (linked_list_node_t**)&stream ) != WICED_SUCCESS )
                    {
                        res = linked_list_remove_node_from_front( &server->inactive_stream_list, (linked_list_node_t**)&stream );
                        if (res != WICED_SUCCESS)
                        {
                            /* Check for NULL sockets in the active stream list and remove them */
                            uint32_t active_stream_cnt = server->active_stream_list.count;
                            wiced_result_t find_null_socket = WICED_NOT_FOUND;

                            HTTP_DEBUG(("WARN! no more http session is available, active=%ld, inactive=%ld\n",
                                server->active_stream_list.count,
                                server->inactive_stream_list.count));

                            for ( ; active_stream_cnt>0; active_stream_cnt--)
                            {
                                find_null_socket = linked_list_find_node(
                                                        &server->active_stream_list,
                                                        http_server_compare_stream_socket,
                                                        (void*)NULL,
                                                        (linked_list_node_t**)&stream );

                                /* stop search if all active stream has valid socket */
                                if ( find_null_socket != WICED_SUCCESS )
                                {
                                    break;
                                }

                                /* active stream list must have valid socket */
                                /* re-claim from active stream if socket is null */
                                linked_list_remove_node(&server->active_stream_list, &stream->node);
                                linked_list_insert_node_at_rear(&server->inactive_stream_list, &stream->node);
                            }

                            /* try to get new stream once again after re-claim */
                            res = linked_list_remove_node_from_front( &server->inactive_stream_list, (linked_list_node_t**)&stream );
                            if (res != WICED_SUCCESS)
                            {
                                HTTP_DEBUG((" %s() : ----- ### DBG : Failed to allocated stream for the new request. Res = [%d]\n", __FUNCTION__, res));
                                HTTP_DEBUG((" %s() : ----- ### DBG : Dropping the request and freeing up the packet\n", __FUNCTION__));
                                wiced_packet_delete( packet );
                                wiced_assert("No more sockets available for the new request. Dropping the request", 0 != 0 );
                                break;
                            }
                        }
                        HTTP_DEBUG((" %s() : ----- ### DBG : Stream = [0x%X], Socket = [0x%X]\n", __FUNCTION__, (unsigned int)stream, (unsigned int)socket));

                        linked_list_insert_node_at_rear( &server->active_stream_list, &stream->node );
                        wiced_http_response_stream_init( &stream->stream.response, current_event.socket );
                        memset( &stream->stream.request, 0, sizeof( stream->stream.request ) );
                    }

                    /* Process packet */
                    result = http_server_parse_receive_packet( server, &stream->stream, packet );
                    wiced_packet_delete( packet );
                }

                break;
            }
            case SOCKET_ERROR_EVENT: /* Fall through */
            default:
            {
                break;
            }
        }
    }

    HTTP_DEBUG((" %s() : ----- ### DBG : Stop TCP server\n", __FUNCTION__));
    wiced_tcp_server_stop( &server->tcp_server );
    HTTP_DEBUG((" %s() : ----- ### DBG : Deinit the queue\n", __FUNCTION__));
    wiced_rtos_deinit_queue( &server->event_queue );

    HTTP_DEBUG((" %s() : ----- ### Successfully exiting from ev process\n", __FUNCTION__));
    WICED_END_OF_CURRENT_THREAD( );
}

static wiced_bool_t http_server_compare_stream_socket( linked_list_node_t* node_to_compare, void* user_data )
{
    wiced_tcp_socket_t*          socket = (wiced_tcp_socket_t*)user_data;
    http_stream_node_t* stream = (http_stream_node_t*)node_to_compare;

    if ( stream->stream.response.tcp_stream.socket == socket )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

static wiced_result_t http_server_deferred_connect_callback( void* arg )
{
    wiced_tcp_socket_t*  socket = (wiced_tcp_socket_t*)arg;
    wiced_http_server_t* server = (wiced_http_server_t*)socket->callback_arg;
    wiced_tls_context_t* context;
    wiced_result_t result = WICED_SUCCESS;

    HTTP_DEBUG( (" try to connect \n") );
    if ( server->tcp_server.tls_identity != NULL )
    {
        if ( socket->tls_context == NULL )
        {
            context = malloc_named("https", sizeof(wiced_tls_context_t));
            if (context == NULL)
            {
                return WICED_OUT_OF_HEAP_SPACE;
            }
            socket->context_malloced = WICED_TRUE;
        }
        else
        {
            context = socket->tls_context;
        }
        wiced_tls_init_context( context, server->tcp_server.tls_identity, NULL );
        wiced_tcp_enable_tls( socket, context );
    }

    result = wiced_tcp_server_accept( &server->tcp_server, socket );
    /* if handshake is successfully completed, then post receive event to read application data packets which were queued in NetX while handshake was in progress */
    if ( result == WICED_SUCCESS )
    {
        http_server_receive_callback ( socket, server );
    }

    return result;
}
