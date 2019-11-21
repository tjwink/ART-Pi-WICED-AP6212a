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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "http_stream.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "base64.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* maximum number of the sessions that can be opened at the same time */
#define NUM_HTTP_SESSIONS               ( 5 )
#define TCP_CONNECTION_TIMEOUT          ( 5000 )
#define HOST_LOOKUP_TIMEOUT             ( 5000 )

#define HTTP_HEADER_STR                 "HTTP/"
#define CONNECTION_KEEP_ALIVE_STR       "Connection: Keep-Alive\r\n"
#define CONNECTION_CLOSE_STR            "Connection: Close\r\n"
#define KEEP_ALIVE_STR                  "Keep-Alive"
#define HTTP_1_1_STR                    " HTTP/1.1\r\n"
#define AUTHORIZATION_STR               "Authorization"
#define BASIC_STR                       "Basic "

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const char* http_methods[] =
{
    [HTTP_OPTIONS]  = "OPTIONS ",
    [HTTP_GET]      = "GET ",
    [HTTP_HEAD]     = "HEAD ",
    [HTTP_POST]     = "POST ",
    [HTTP_PUT]      = "PUT ",
    [HTTP_DELETE]   = "DELETE ",
    [HTTP_TRACE]    = "TRACE ",
    [HTTP_CONNECT]  = "CONNECT ",
    [HTTP_NOTIFY]   = "NOTIFY ",
    [HTTP_M_SEARCH] = "M-SEARCH ",
    [HTTP_END]      = "END ",
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t http_deinit_stream( http_stream_t* session )
{
    wiced_assert("Bad args", session != NULL);

    session->is_connected = 0;

    wiced_tcp_stream_deinit(&session->tcp_stream);

    wiced_tcp_delete_socket( &session->socket );

    return WICED_SUCCESS;
}

wiced_result_t http_init_stream( http_stream_t* session, wiced_interface_t interface, const char* host_name, wiced_ip_address_t* ip_address, uint16_t port )
{
    uint8_t        count  = 0;
    wiced_result_t result = WICED_ERROR;

    wiced_assert("Bad args", session != NULL );

    /* Clear the session object */
    memset(session, 0, sizeof(http_stream_t));

    /* check whether the hostname size is supported */
    if ( host_name != NULL )
    {
        if ( strlen( host_name ) > ( MAX_HTTP_HOST_NAME_SIZE - 1 ) )
        {
            return WICED_BADARG;
        }

        while ( wiced_hostname_lookup( host_name, &session->host_ip, HOST_LOOKUP_TIMEOUT, interface ) != WICED_SUCCESS )
        {
            count++;
            if ( count >= 2 )
            {
                WPRINT_LIB_DEBUG(( "Unable to find a host\n\r" ));
                goto return_error;
            }
        }

        /* copy hostname locally */
        strcpy( session->hostname, host_name );
    }
    else if (ip_address != NULL)
    {
        /* otherwise assign IP address */
        SET_IPV4_ADDRESS( session->host_ip, GET_IPV4_ADDRESS((*ip_address)) );
    }
    else
    {
        return WICED_BADARG;
    }

    /* Create a TCP socket */
    if ( wiced_tcp_create_socket( &session->socket, interface ) != WICED_SUCCESS )
    {
        goto return_error;
    }

    /* Open connection to the HTTP server */
    if ( wiced_tcp_connect( &session->socket, &session->host_ip, port, TCP_CONNECTION_TIMEOUT ) != WICED_SUCCESS )
    {
        goto return_error_with_socket;
    }

    /* Init the TCP stream */
    if ( wiced_tcp_stream_init( &session->tcp_stream, &session->socket ) != WICED_SUCCESS )
    {
        goto return_error_with_socket;
    }

    session->is_connected = 1;

    return WICED_SUCCESS;

return_error_with_socket:
    wiced_tcp_delete_socket( &session->socket );

return_error:

    return result;
}


#if 0 /* unreferenced */
wiced_result_t http_stream_send_request( http_stream_t* session, http_request_t method, const char* url, const http_header_t* headers, uint16_t number_of_headers,  uint8_t* body, uint16_t body_length, wiced_bool_t keep_alive )
{
    wiced_result_t result;

    /* start request with specifying the http method and the url */
    http_stream_start_headers(session, method, url );

    /* add all required headers */
    result = http_stream_add_headers( session, headers, number_of_headers );\
    if( result != WICED_SUCCESS )
    {
        return result;
    }
    /* finish with headers and set the last header keep_alive */
    http_stream_end_headers(session, keep_alive);

    /* add the body to the http request */
    http_stream_write( session, body, body_length );
    http_stream_flush( session );

    /* return with no error */
    return WICED_SUCCESS;
}
#endif /* if 0 unreferenced */

wiced_result_t http_stream_start_headers( http_stream_t* session, http_request_t method, const char* url )
{
    wiced_assert("Bad args", (session != NULL) && (url != NULL));

    /* add method and version, host and url to the tcp stream */
    WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, http_methods[method], strlen( http_methods[method] ) ) );
    WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, url, strlen( url ) ) );
    WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, HTTP_1_1_STR, sizeof(HTTP_1_1_STR) - 1 ) );

    /* Write host name required by http/1.0 version */
    if ( strlen( session->hostname ) != 0 )
    {
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, "Host: ", strlen("Host: ") ) );
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, session->hostname, strlen( session->hostname ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, HTTP_NEW_LINE, strlen( HTTP_NEW_LINE ) ) );
    }

    return WICED_SUCCESS;
}

/* add a list of headers to the request */
wiced_result_t http_stream_add_headers( http_stream_t* session, const http_header_t* headers, uint16_t number_of_headers )
{
    uint16_t i = 0;

    wiced_assert("Bad args", (session != NULL) && (headers != NULL));

    for ( i = 0; i < number_of_headers; ++i, ++headers )
    {
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, headers->name, strlen( headers->name ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, ": ", strlen( ": " ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, headers->value, strlen( headers->value ) ) );
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, HTTP_NEW_LINE, strlen( HTTP_NEW_LINE ) ) );
    }

    return WICED_SUCCESS;
}

wiced_result_t http_stream_end_headers( http_stream_t* session, wiced_bool_t keep_alive)
{
    if ( keep_alive == WICED_TRUE )
    {
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, CONNECTION_KEEP_ALIVE_STR, sizeof(CONNECTION_KEEP_ALIVE_STR) - 1 ) );
    }
    else
    {
        WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, CONNECTION_CLOSE_STR, sizeof(CONNECTION_CLOSE_STR) - 1 ) );
    }

    WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, HTTP_NEW_LINE, strlen( HTTP_NEW_LINE ) ) );

    return WICED_SUCCESS;
}

/* this function will add a body to the http request */
wiced_result_t http_stream_write( http_stream_t* session, uint8_t* data, uint16_t length )
{
    wiced_assert("Bad args", (session != NULL) && (data != NULL));

    WICED_VERIFY( wiced_tcp_stream_write( &session->tcp_stream, data, length ) );

    return WICED_SUCCESS;
}

wiced_result_t http_stream_read( http_stream_t* session, uint8_t* data, uint16_t length, uint32_t timeout, uint32_t* read_count )
{
    wiced_assert("Bad args", (session != NULL) && (data != NULL));

    WICED_VERIFY( wiced_tcp_stream_read_with_count( &session->tcp_stream, data, length, timeout, read_count ) );

    return WICED_SUCCESS;
}

wiced_result_t http_stream_flush( http_stream_t* session )
{
    wiced_assert("Bad args", (session != NULL));

    WICED_VERIFY( wiced_tcp_stream_flush( &session->tcp_stream ) );

    return WICED_SUCCESS;
}

wiced_result_t http_send_basic_authorization( http_stream_t* session, char* username_password )
{
    wiced_result_t result;
    int            base64_datalen;
    unsigned int   value_size;
    http_header_t  basic_auth = { AUTHORIZATION_STR, NULL };

    wiced_assert("Bad args", ((session != NULL) && (username_password != NULL)) );

    value_size = sizeof( BASIC_STR ) + DIV_ROUND_UP( (strlen(username_password) + 2) * 4, 3 ) + 1;

    basic_auth.value = malloc( value_size );
    if ( basic_auth.value == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    /* Add Authorisation basic field */
    strcpy( basic_auth.value, BASIC_STR );

    base64_datalen = base64_encode( (unsigned char*) username_password, (int32_t) strlen( username_password ), (unsigned char*)&basic_auth.value[sizeof( BASIC_STR ) - 1], value_size - ( sizeof( BASIC_STR ) - 1 ), BASE64_STANDARD );
    if ( base64_datalen < 0 )
    {
        free( basic_auth.value );
        return WICED_ERROR;
    }

    result = http_stream_add_headers( session, &basic_auth, 1 );

    free( basic_auth.value );

    return result;

}


/* Extract a header value from the HTTP response received from the server */
wiced_result_t http_extract_headers( wiced_packet_t* packet, http_header_t* headers, uint16_t number_of_headers )
{
    uint8_t* data;
    uint16_t data_length;
    uint16_t available_data_length;
    char*    header_location;
    int a;
    wiced_bool_t finished = WICED_FALSE;
    uint16_t matched_headers = 0;

    wiced_assert("Bad args", (packet != NULL) && (headers != NULL) );

    if (number_of_headers < 1)
    {
        return WICED_BADARG;
    }

    WICED_VERIFY( wiced_packet_get_data( packet, 0, &data, &data_length, &available_data_length ) );
    if ( data_length < available_data_length )
    {
        /* Fragmented packets not supported */
        return WICED_ERROR;
    }

    /* verify that it is a packet with HTTP header before checking for headers */
    if (memcmp( (char*) data, HTTP_HEADER_STR, sizeof(HTTP_HEADER_STR) - 1 ) != 0)
    {
        return WICED_NOT_FOUND;
    }


    header_location = (char*)data;
    do
    {
        /* Skip to the next header entry */
        header_location = strchr( (const char*) header_location, '\n' );
        if ( header_location == 0 )
        {
            return WICED_ERROR;
        }
        ++header_location;

        /* Check if we haven't reached the end of the headers */
        if (((uint8_t*)header_location - data) < data_length &&
            !(header_location[0] == '\r' && header_location[1] == '\n'))
        {
            /* Try find a matching header */
            for ( a = 0; a < number_of_headers; ++a )
            {
                unsigned int string_length = strlen( headers[a].name );
                if ( ( memcmp( header_location, headers[a].name, string_length ) == 0 ) &&
                     ( *( header_location + string_length ) == ':' ) )
                {
                    headers[a].value = header_location + string_length + 1;
                    ++matched_headers;
                    break;
                }
            }
        }
        else
        {
            finished = WICED_TRUE;
        }
    } while ( ( finished == WICED_FALSE ) && ( matched_headers != number_of_headers ) );

    return WICED_SUCCESS;
}

/* get body of the response received from the server */
wiced_result_t http_get_body( wiced_packet_t* packet, uint8_t** body, uint32_t* body_length )
{
    uint8_t* packet_data;
    uint16_t packet_data_length;
    uint16_t available_data_length;
    char*    start_of_body;

    wiced_assert("Bad args", (packet != NULL) && (body != NULL) && (body_length != NULL));

    WICED_VERIFY( wiced_packet_get_data( packet, 0, &packet_data, &packet_data_length, &available_data_length ) );
    if ( packet_data_length < available_data_length )
    {
        /* Fragmented packets not supported */
        return WICED_ERROR;
    }

    /* verify that it is a packet with HTTP header before checking for body */
    if (memcmp( (char*) packet_data, HTTP_HEADER_STR, sizeof(HTTP_HEADER_STR) - 1 ) != 0)
    {
        return WICED_NOT_FOUND;
    }

    /* Find the first occurrence of "\r\n\r\n", this signifies the end of the headers */
    start_of_body = strnstrn( (const char*) packet_data, packet_data_length, HTTP_HEADERS_BODY_SEPARATOR, sizeof(HTTP_HEADERS_BODY_SEPARATOR) - 1);
    if ( start_of_body == NULL )
    {
        return WICED_NOT_FOUND;
    }
    /* Skip the "\r\n\r\n" */
    start_of_body += 4;

    *body_length = (uint32_t)( packet_data_length - ( ( start_of_body - (char*) packet_data ) ) );

    if(*body_length  == 0)
    {
        *body = NULL;
        return WICED_SUCCESS;
    }
    *body        = (uint8_t*) start_of_body;

    return WICED_SUCCESS;
}

wiced_result_t http_stream_receive( http_stream_t* session, wiced_packet_t** packet, uint32_t timeout)
{
    wiced_assert("Bad args", (session != NULL) && (packet != NULL) );

    /* Receive a response from the tcp, connection is considered to be established */
    WICED_VERIFY( wiced_tcp_receive( &session->socket, packet, timeout ) );

    return WICED_SUCCESS;
}

wiced_result_t http_process_response( wiced_packet_t* packet, http_status_code_t* response_code )
{
    char*          response_status;
    uint8_t*       response;
    uint16_t       response_length;
    uint16_t       available_data_length;

    wiced_assert("Bad args", (packet != NULL) && (response_code != NULL));

    /* Get pointer to data from a packet */
    WICED_VERIFY( wiced_packet_get_data( packet, 0, &response, &response_length, &available_data_length ) );
    if ( response_length < available_data_length )
    {
        /* Fragmented packets not supported */
        return WICED_ERROR;
    }

    /* Check we have enough data to identify the response number */
    if (response_length < 12)
    {
        return WICED_ERROR;
    }

    /* verify that it is a packet with HTTP header before checking for body */
    if (memcmp( (char*) response, HTTP_HEADER_STR, sizeof(HTTP_HEADER_STR) - 1 ) != 0)
    {
        return WICED_NOT_FOUND;
    }

    /* Find the HTTP/x.x part*/
    response_status = strnstrn( (const char*) response, response_length, HTTP_HEADER_STR, sizeof(HTTP_HEADER_STR) - 1 );
    if (response_status == 0)
    {
        return WICED_ERROR;
    }

    /* Please check we have enough data before dereferencing it */
    if(((response + response_length) - (uint8_t *)response_status) < 13)
    {
       return WICED_ERROR;
    }

    /* Skip the "HTTP/" and the version "x.x"*/
    response_status += 5 + 3;

    /* Verify next character is a space*/
    if ( *response_status != ' ' )
    {
        return WICED_ERROR;
    }
    /* Skip the space */
    ++response_status;

    /* Verify response is 3 characters followed by a space */
    if ( *(response_status + 3) != ' ' )
    {
        return WICED_ERROR;
    }
    *response_code = atoi( response_status );

    return WICED_SUCCESS;
}

/* Process the header values in place in the HTTP packet */
wiced_result_t http_process_headers_in_place( wiced_packet_t* packet, http_header_t* headers, uint16_t* number_of_headers )
{
    uint8_t*        data;
    uint16_t        data_length;
    uint16_t        available_data_length;

    uint16_t        headers_parsed;     /* count the * headers parsed */

    char*           data_location;      /* walk through the data */
    char*           line_end;
    char*           name_start;
    char*           name_end;
    char*           value_start;
    char*           value_end;

    if ((packet == NULL) || (headers == NULL) || (number_of_headers == NULL))
    {
        return WICED_BADARG;
    }

    if (*number_of_headers == 0)
    {
        /* no headers in the header struct? */
        return WICED_BADARG;
    }

    WICED_VERIFY( wiced_packet_get_data( packet, 0, &data, &data_length, &available_data_length ) );
    if ( data_length < available_data_length )
    {
        /* Fragmented packets not supported */
        return WICED_ERROR;
    }

    /* verify that it is a packet with HTTP header before checking for body */
    if (memcmp( (char*) data, HTTP_HEADER_STR, sizeof(HTTP_HEADER_STR) - 1 ) != 0)
    {
        return WICED_NOT_FOUND;
    }

    headers_parsed = 0;
    /* first line is the query, we skip that here  */
    data_location = strchr( (const char*)data, '\n');
    if (data_location == NULL)
    {
        return WICED_ERROR;
    }

    data_location += 1; /* skip thje '\n' */
    while ( (((uint8_t*)data_location - data) < data_length) && (headers_parsed < *number_of_headers) )
    {
        line_end =  strchr( (const char*)data_location, '\n');
        if ( line_end == NULL )
        {
            break;
        }

        /* We may have a header here - Is there a ':' before a '\n' ? */
        name_start = data_location;
        name_end =  strchr( (const char*)data_location, ':');
        if ((name_end == NULL) || (name_end > line_end))
        {
            break;
        }

        /* we have a header here - find the value */
        value_start = name_end + 1;    /* skip the ':' */
        /* skip any spaces after the ':' */
        while( *value_start == ' ')
        {
            value_start++;
        }
        value_end = strchr( (const char*)value_start, '\r');
        if (value_end == NULL)
        {
            value_end = strchr( (const char*)value_start, '\n');
        }
        if ((value_end == NULL) || (value_end > line_end))
        {
            break;
        }
        /* replace the ':' with a '\0' to mark the end of the name string */
        *name_end = '\0';
        /* replace the '\r' (or '\n') with a '\0' to mark the end of the value string */
        *value_end = '\0';

        /* point to the header & value, increment to next header info struct */
        headers[headers_parsed].name  = name_start;
        headers[headers_parsed].value = value_start;
        headers_parsed++;

        /* Check if we have reached the end of the headers */
        data_location = line_end + 1;    /* skip the '\n' */
        if ( (data_location[0] == '\r') && (data_location[1] == '\n') )
        {
            break;
        }

    }

    /* return headers that we parsed */
    *number_of_headers = headers_parsed;
    return WICED_SUCCESS;
}

http_request_t  http_determine_method(wiced_packet_t* packet)
{
    uint8_t*        data;
    uint16_t        data_length;
    uint16_t        available_data_length;
    uint16_t        i;

    if (packet == NULL)
    {
        return HTTP_UNKNOWN;
    }

    if( wiced_packet_get_data( packet, 0, &data, &data_length, &available_data_length ) != WICED_SUCCESS)
    {
        return HTTP_UNKNOWN;
    }
    if ( data_length < available_data_length )
    {
        /* Fragmented packets not supported */
        return HTTP_UNKNOWN;
    }

    /* verify that it is a packet with HTTP header before checking for body */
    if (memcmp( (char*) data, HTTP_HEADER_STR, sizeof(HTTP_HEADER_STR) - 1 ) != 0)
    {
        return HTTP_UNKNOWN;
    }

    for( i=0; i < HTTP_METHODS_MAX; i++ )
    {
        uint16_t method_name_len = sizeof(http_methods[i]);
        if ( strncmp((char *)data, http_methods[i], method_name_len) == 0)
        {
            return (http_request_t)i;
        }
    }

    return HTTP_UNKNOWN;
}
