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

#include "ota2_server.h"
#include "wwd_assert.h"
#include "wiced.h"
#include "wiced_resource.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define OTA_SERVER_THREAD_PRIORITY (WICED_DEFAULT_LIBRARY_PRIORITY)
#define OTA_SERVER_STACK_SIZE      (10000)

static const char ok_header[] =
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: ";

static const char crlfcrlf[] ="\r\n\r\n";

#ifndef USE_404

static const char not_found_header[] = "HTTP/1.0 301\r\nLocation: /\r\n\r\n";

#else /* ifndef USE_404 */
static const char not_found_header[] =
    "HTTP/1.0 404 Not Found\r\n"
    "Content-Type: text/html\r\n\r\n"
    "<!doctype html>\n"
    "<html><head><title>404 - WICED Web Server</title></head><body>\n"
    "<h1>Address not found on WICED Web Server</h1>\n"
    "<p><a href=\"/\">Return to home page</a></p>\n"
    "</body>\n</html>\n";

#endif /* ifndef USE_404 */

#define NO_CACHE_HEADER "\r\n" \
                        "Expires: Mon, 26 Jul 1997 05:00:00 GMT\r\n" \
                        "Cache-Control: no-store, no-cache, must-revalidate, max-age=0, post-check=0, pre-check=0\r\n" \
                        "Pragma: no-cache"

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
 *               Function Declarations
 ******************************************************/

static wiced_result_t process_request               ( ota_http_request_message_t* request, wiced_tcp_socket_t* socket, const ota_http_page_t* server_url_list );
static wiced_result_t ota_server_process_request    ( ota_server_t* server, wiced_tcp_socket_t* socket );
static void           ota_server_thread_main        ( uint32_t arg );
static wiced_result_t get_http_request_type_and_url ( uint8_t* http_data, uint16_t http_data_length, ota_http_request_message_t* request );
static wiced_result_t write_reply_header            ( wiced_tcp_stream_t* stream, const char* mime_type, wiced_bool_t nocache );

/******************************************************
 *                 Static Variables
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t ota_server_start( ota_server_t* server, uint16_t port, const ota_http_page_t* page_database, wiced_interface_t interface )
{
    memset( server, 0, sizeof( *server ) );

    /* Create the TCP socket */
    WICED_VERIFY(wiced_tcp_create_socket( &server->socket, interface ));
    if (wiced_tcp_listen( (wiced_tcp_socket_t*) &server->socket, port ) != WICED_SUCCESS)
    {
        wiced_tcp_delete_socket(&server->socket);
        return WICED_ERROR;
    }

    server->page_database = page_database;
    return wiced_rtos_create_thread(&server->thread, OTA_SERVER_THREAD_PRIORITY, "HTTPserver", ota_server_thread_main, OTA_SERVER_STACK_SIZE, server);
}

wiced_result_t ota_server_stop (ota_server_t* server)
{
    server->quit = WICED_TRUE;
    if ( wiced_rtos_is_current_thread( &server->thread ) != WICED_SUCCESS )
    {
        wiced_rtos_thread_force_awake( &server->thread );
        wiced_rtos_thread_join( &server->thread );
        wiced_rtos_delete_thread( &server->thread );
    }
    return WICED_SUCCESS;
}

void init_request( ota_server_t* server )
{
    int i = 0;
    for( i = 0; i < 3; i++ )
    {
        server->request.body_chunks[i].data = NULL;
        server->request.body_chunks[i].size = 0;
    }

    /* Init request details */
    server->request.content_length = 0;
    server->request.header_ptr = NULL;
    server->request.header_size = 0;
    server->request.protocol_type_length = 0;
    server->request.protocol_type_ptr = NULL;
    server->request.request_type = 0;
    server->request.request_type_length = 0;
    server->request.url_length = 0;
    server->request.url_ptr = 0;
    server->request.query_length = 0;
    server->request.query_ptr = 0;
    server->request.current_packet_index = 0;
}

static void ota_server_thread_main(uint32_t arg)
{
    ota_server_t*          server          = (ota_server_t*) arg;
    uint32_t               total_body_size = 0;
    int                    i               = 0;

    server->reboot_required = WICED_FALSE;
    while ( server->quit != WICED_TRUE )
    {
        wiced_packet_t* temp_packet = NULL;

        /* Wait for a connection */
        wiced_result_t result = wiced_tcp_accept( &server->socket );
        if ( result == WICED_SUCCESS )
        {
            for(;;)
            {
                if ( wiced_tcp_receive( &server->socket, &temp_packet, WICED_WAIT_FOREVER ) == WICED_SUCCESS )
                {
                    char* request_string;
                    uint16_t request_length;
                    uint16_t available_data_length;

                    if( server->state == READING_HEADER )
                    {
                        uint8_t         temp   = 0;
                        wiced_result_t  result = WICED_ERROR;

                        if ( temp_packet == NULL )
                        {
                            goto disconnect;
                        }
                        init_request(server);

                        server->request.request_packets[server->request.current_packet_index] = temp_packet;

                        wiced_packet_get_data(temp_packet, 0, (uint8_t**)&request_string, &request_length, &available_data_length);
                        if ( request_length != available_data_length )
                        {
                            /* Fragmented packets not supported */
                            result = WICED_ERROR;
                            wiced_packet_delete(temp_packet);
                            goto disconnect;
                        }

                        /* Check that this is a GET or POST request, abort everything else */
                        if ( ( strstr( request_string, "GET" ) == 0 ) && ( strstr( request_string, "POST") == 0 ) )
                        {
                            result = WICED_ERROR;
                            wiced_packet_delete(temp_packet);
                            goto disconnect;
                        }

                        /* Get header pointer and header size */
                        server->request.header_ptr = (uint8_t*)request_string;
                        server->request.header_size = ( (char*)strstr( (char*)request_string, crlfcrlf ) + strlen( crlfcrlf ) ) - (char*)request_string;
                        if( server->request.header_size == strlen( crlfcrlf ) )
                        {
                            goto disconnect;
                        }

                        /* Get content length */
                        server->request.content_length = 0;
                        if( strstr( request_string, "Content-Length") != NULL )
                        {
                            uint8_t* content_length_value = (uint8_t*)strstr( request_string, "Content-Length") + strlen("Content-Length:");
                            server->request.content_length = atoi((const char*)content_length_value);
                        }

                        temp = request_string[ server->request.header_size ];
                        request_string[ server->request.header_size ] ='\0';
                        request_string[ server->request.header_size ] = temp;

                        /* Get request type and the url */
                        result = get_http_request_type_and_url( (uint8_t*)request_string, request_length, &server->request);
                        if ( result == WICED_ERROR )
                        {
                           goto disconnect;
                        }

                        server->state = READING_BODY;
                    }

                    if( server->state == READING_BODY )
                    {
                        http_body_chunk_t*          current_body_chunk = &server->request.body_chunks[server->request.current_packet_index];

                        if( server->request.current_packet_index != 0 )
                        {
                            server->request.request_packets[server->request.current_packet_index] = temp_packet;
                        }

                        wiced_packet_get_data(temp_packet, 0, (uint8_t**)&request_string, &request_length, &available_data_length);

                        if ( request_length != available_data_length )
                        {
                            /* Fragmented packets not supported */
                            result = WICED_ERROR;
                            wiced_packet_delete( temp_packet );
                            goto disconnect;
                        }

                        if( server->request.current_packet_index == 0 )
                        {
                            current_body_chunk->data = server->request.header_ptr + server->request.header_size;
                            current_body_chunk->size = ( request_string + request_length ) - (char*)current_body_chunk->data;
                        }
                        else
                        {
                            current_body_chunk->data = (uint8_t*)request_string;
                            current_body_chunk->size = request_length;
                        }

                        /* calculate total combined size of all body chunks which belongs to this message */
                        total_body_size = 0;

                        for( i = 0; i < ( server->request.current_packet_index + 1 ) ; i++ )
                        {
                            total_body_size+= server->request.body_chunks[i].size;
                        }

                        server->request.current_packet_index++;

                        /* Check whether the combined size of the previous chunks and the current one is equal to the content length received in the first packet */
                        if( total_body_size == server->request.content_length )
                        {
                            ota_server_process_request( server, &server->socket );
                            /* Delete all packets belonging to the message */
                            for( i = 0; i < server->request.current_packet_index; i++ )
                            {
                                wiced_packet_delete(server->request.request_packets[i]);
                            }
                            server->state = READING_HEADER;
                            break;
                        }
                    }
                }
                else
                {
                    goto disconnect;
                }
            }
        }
disconnect:
        wiced_tcp_disconnect( &server->socket );
    }

    wiced_tcp_delete_socket( &server->socket );
    if( server->reboot_required == WICED_TRUE )
    {
        /* Give some for the response to be sent properly */
        wiced_rtos_delay_milliseconds(2000);

        /* Perform a reboot!!! */
        wiced_framework_reboot();
    }
    WICED_END_OF_CURRENT_THREAD( );
}

static wiced_result_t get_http_request_type_and_url( uint8_t* http_data, uint16_t http_data_length, ota_http_request_message_t* request )
{
    uint8_t* ptr;
    uint8_t* end_of_url;
    uint8_t* start_of_protocol;
    uint8_t* start_of_query;
    uint8_t* end_of_protocol;
    uint8_t* data_ptr;
    uint8_t  end_of_url_character = 0;


    ptr = http_data;
    request->request_type = http_data;
    while( ( *ptr != '\0' ) && ( *ptr != ' ' ) && ( *ptr != '/' ) )
    {
        ptr++;
    }

    request->request_type_length = (uint16_t)(ptr - http_data);
    http_data[request->request_type_length] = '\0';
    data_ptr = &http_data[request->request_type_length + 1];

    request->url_ptr  = http_data + request->request_type_length + 1 ;

    end_of_url = request->url_ptr;

    /* Find the end of the url path ( space, newline, null, or end of packet ) */
    while ( ( *end_of_url != ' ' ) && ( *end_of_url != '\n' ) && ( *end_of_url != '\x00' ) && ( *end_of_url != '?' ) )
    {
        end_of_url++;

        /* Check if we've run out of data */
        if ( end_of_url > &http_data[http_data_length] )
        {
            /* Can't find end of URL, perhaps only part of the request*/
            return WICED_ERROR;
        }
    }
    end_of_url_character = *end_of_url;

    request->url_length = (uint16_t)(end_of_url - request->url_ptr);
    data_ptr[request->url_length] = '\0';
    data_ptr = &data_ptr[ request->url_length + 1 ];

    if( end_of_url_character == '?' )
    {
        /* Find the start of the query and its length */
        start_of_query = data_ptr;
        while ( ( *start_of_query != ' ' ) && ( *start_of_query != '\r' ) && ( *start_of_query != '\n' ) && ( *start_of_query != '\x00' ) )
        {
            start_of_query++;
            /* Check if we've run out of data */
            if ( start_of_query > &http_data[http_data_length] )
            {
                /* Can't find end of URL, perhaps only part of the request*/
                return WICED_ERROR;
            }
        }
        request->query_ptr = data_ptr;
        request->query_length = (uint16_t)(start_of_query - data_ptr);
        data_ptr[request->query_length] = '\0';
        data_ptr = &data_ptr[ request->url_length + 1 ];
    }

    start_of_protocol = data_ptr;
    request->protocol_type_ptr = start_of_protocol;
    end_of_protocol = start_of_protocol;
    while( ( *end_of_protocol != '\0' ) && ( *end_of_protocol != '\r' ) && ( *end_of_protocol != '\n' ) )
    {
        end_of_protocol++;
    }

    request->protocol_type_length = (uint16_t)(end_of_protocol - start_of_protocol);
    data_ptr[request->protocol_type_length] = '\0';

    return WICED_SUCCESS;
}

static wiced_result_t ota_server_process_request( ota_server_t* server, wiced_tcp_socket_t* socket)
{
    wiced_result_t result = WICED_ERROR;
    result = process_request( &server->request, (wiced_tcp_socket_t*)socket, server->page_database );
    return result;
}

static wiced_result_t process_request( ota_http_request_message_t* request, wiced_tcp_socket_t* socket, const ota_http_page_t* server_url_list )
{
    /* Search the url to find the question mark if there is one */
    int i = 0;
    wiced_tcp_stream_t stream;
    wiced_bool_t found = WICED_FALSE;

    /* Init the tcp stream */
    wiced_tcp_stream_init(&stream, socket);

    /* Search URL list to determine if request matches one of our pages */
    while ( server_url_list[i].ota_url != NULL )
    {
        /* Compare request to a path */
        if ( 0 == strncmp( server_url_list[i].ota_url, (const char*)request->url_ptr, request->url_length ) )
        {
            /* Matching path found */
            found = WICED_TRUE;
            /* Call the content handler function to write the page content into the packet and adjust the write pointers */
            switch (server_url_list[i].ota_url_content_type)
            {
                case OTA_DYNAMIC_URL_CONTENT:
                    write_reply_header(&stream, server_url_list[i].ota_mime_type, WICED_TRUE);
                    /* Fall through */
                case OTA_RAW_DYNAMIC_URL_CONTENT:
                    server_url_list[i].ota_url_content.ota_dynamic_data.generator( request, &stream, server_url_list[i].ota_url_content.ota_dynamic_data.arg );
                    wiced_tcp_stream_flush(&stream);
                    break;

                case OTA_STATIC_URL_CONTENT:
                    write_reply_header(&stream, server_url_list[i].ota_mime_type, WICED_FALSE);
                    /* Fall through */
                case OTA_RAW_STATIC_URL_CONTENT:
                    wiced_tcp_stream_write(&stream, server_url_list[i].ota_url_content.ota_static_data.ptr, server_url_list[i].ota_url_content.ota_static_data.length);
                    wiced_tcp_stream_flush(&stream);
                    break;
                case OTA_RESOURCE_URL_CONTENT:
                    write_reply_header(&stream, server_url_list[i].ota_mime_type, WICED_FALSE);
                    /* Fall through */
                case OTA_RAW_RESOURCE_URL_CONTENT:
                    wiced_tcp_stream_write_resource( &stream, server_url_list[i].ota_url_content.ota_resource_data );
                    wiced_tcp_stream_flush( &stream );
                    break;
                default:
                    wiced_assert("Unknown entry in URL list", 0 != 0 );
                    break;

            }
            break;
        }
        i++;
    }

    /* Check if page was not found */
    if ( found == WICED_FALSE )
    {
        /* Send back 404 */
        wiced_tcp_stream_write(&stream, not_found_header, sizeof(not_found_header)-1);
        wiced_tcp_stream_flush(&stream);
    }

    wiced_assert( "Page Serve finished with data still in stream", stream.tx_packet == NULL );

    return WICED_SUCCESS;
}

wiced_result_t write_reply_header(wiced_tcp_stream_t* stream, const char* mime_type, wiced_bool_t nocache )
{
    /* Copy HTTP OK header into packet and adjust the write pointers */
    wiced_tcp_stream_write( stream, ok_header, sizeof( ok_header ) - 1 );

    /* Add Mime type */
    wiced_tcp_stream_write( stream, mime_type, strlen( mime_type ) );

    /* Output No-Cache headers if required */
    if ( nocache == WICED_TRUE )
    {
        wiced_tcp_stream_write( stream, NO_CACHE_HEADER, sizeof( NO_CACHE_HEADER ) - 1 );
    }

    /* Add double carriage return, line feed */
    wiced_tcp_stream_write( stream, crlfcrlf, sizeof( crlfcrlf ) - 1 );

    return WICED_SUCCESS;
}
