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
#include "http_server.h"
#include "mesh.h"
#include "mesh_uri.h"
#include "wiced_hci_bt_mesh.h"
#include "wwd_debug.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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

static char* get_payload( const char* url_path );
mesh_value_handle_t* reverse_mesh_byte_stream( mesh_value_handle_t* val );
mesh_value_handle_t* create_mesh_value( const char* payload );




/******************************************************
 *               Variables Definitions
 ******************************************************/

const char json_object_start         [] = "{\r\n";
const char json_data_actuator_status [] = "\"Status \": \"";
const char json_data_end4            [] = "\"\r\n}\r\n";

/******************************************************
 *               Function Definitions
 ******************************************************/

int32_t process_send_mesh_data( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    char*                payload        = get_payload( url_path );
    mesh_value_handle_t* value          = create_mesh_value( payload );
    mesh_value_handle_t* reversed_value = reverse_mesh_byte_stream( value );

    WPRINT_APP_INFO(("[%s] len =%lu\n",__func__,reversed_value->length));
    wiced_bt_mesh_send_proxy_packet(reversed_value->value, reversed_value->length);

    wiced_http_response_stream_enable_chunked_transfer( stream );
    wiced_http_response_stream_disable_chunked_transfer( stream );
    wiced_http_response_stream_flush( stream );
    wiced_http_response_stream_disconnect( stream );
    free( value );
    free( reversed_value );
    return 0;
}


wiced_result_t mesh_send_status( wiced_http_response_stream_t* stream, const char* status )
{
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_actuator_status, sizeof( json_data_actuator_status ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, status, strlen(status) ));
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_end4, sizeof( json_data_end4 ) - 1 ) );
    return WICED_SUCCESS;
}

static char* get_payload( const char* url_path )
{
    /* /mesh/<command>/values/<payload> */
    char* current_path = (char*) url_path;
    uint32_t a = 0;

    while ( a < 4 )
    {
        if ( *current_path == '\0' )
        {
            /* <service> not found */
            return NULL;
        }
        else if ( *current_path == '/' )
        {
            a++;
        }

        current_path++;
    }

    return current_path;

}

mesh_value_handle_t* reverse_mesh_byte_stream( mesh_value_handle_t* val )
{
    mesh_value_handle_t* ret_val = (mesh_value_handle_t*) malloc( sizeof(mesh_value_handle_t) + val->length );
    WPRINT_APP_INFO(("Received mesh proxy packet \n"));
    for ( int i = 0; i < val->length; i++ )
    {
        ret_val->value[ i ] = val->value[ val->length - 1 - i ];
        WPRINT_APP_INFO((" %02X " , ret_val->value[ i ]));
    }
    ret_val->length = val->length;
    printf( "\n" );
    return ret_val;
}

mesh_value_handle_t* create_mesh_value( const char* payload )
{
    uint32_t temp  = strlen( payload ) / 2;
    mesh_value_handle_t* value = (mesh_value_handle_t*)malloc_named( "value", sizeof( mesh_value_handle_t ) + temp );
    if ( value != NULL )
    {
        uint32_t a;
        uint32_t b;
        memset( value, 0, sizeof( mesh_value_handle_t ) + temp );
        value->length = temp;
        for ( a = 0, b = ( value->length - 1 ) * 2 ; a < value->length; a++, b-=2 )
        {
            string_to_unsigned( &payload[b], 2, &temp, 1 );
            value->value[a] = (uint8_t)( temp & 0xff );
        }
    }

    return value;
}
