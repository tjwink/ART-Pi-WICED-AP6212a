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
#include "restful_smart_server.h"
#include "restful_smart_response.h"
#include "restful_smart_ble.h"
#include "restful_smart_constants.h"
#include "http_server.h"
#include "wiced_bt_dev.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define CHECK_STREAM( stream ) { if ( stream == NULL ) { return WICED_ERROR; } if ( stream->tcp_stream.socket == NULL ) { return WICED_ERROR; } wiced_http_response_stream_enable_chunked_transfer( stream ); }

/******************************************************
 *                    Constants
 ******************************************************/

#define LTK_MASK  0x08
#define EDIV_MASK 0x04
#define IRK_MASK  0x02
#define CSRK_MASK 0x01

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

/******************************************************
 *               Variables Definitions
 ******************************************************/

const char slash_gatt_slash_nodes_slash      [] = "/gatt/nodes/";
const char slash_management_slash_nodes_slash[] = "/management/nodes/";
const char slash_gap_slash_nodes_slash       [] = "/gap/nodes/";
const char slash_services_slash              [] = "/services/";
const char slash_characteristics_slash       [] = "/characteristics/";
const char slash_descriptors_slash           [] = "/descriptors/";
const char json_data_self_start              [] = "\"self\"       : { \"href\" : \"http://";
const char json_data_self_end                [] = "\"},\r\n";
const char json_data_bdaddr                  [] = "\"bdaddr\"     : \"";
const char json_data_handle                  [] = "\"handle\"     : \"";
const char json_data_uuid                    [] = "\"uuid\"       : \"";
const char json_data_primary                 [] = "\"primary\"    : \"";
const char json_data_properties              [] = "\"properties\" : \"";
const char json_data_value                   [] = "\"value\"      : \"";
const char json_node_array_start             [] = "{\r\n\"nodes\" : [";
const char json_service_array_start          [] = "{\r\n\"services\" : [";
const char json_characteristic_array_start   [] = "{\r\n\"characteristics\" : [";
const char json_descriptor_array_start       [] = "{\r\n\"descriptors\" : [";
const char json_adv_array_start              [] = "\"AD\" : [";
const char json_object_start                 [] = "{\r\n";
const char json_data_bdaddrtype              [] = "\"bdaddrType\" : \"";
const char json_data_ltk                     [] = "\"LTK\"        : ";
const char json_data_ediv                    [] = "\"EDIV\"       : ";
const char json_data_irk                     [] = "\"IRK\"        : ";
const char json_data_csrk                    [] = "\"CSRK\"       : ";
const char json_data_rssi                    [] = "\"rssi\"       : ";
const char json_data_name                    [] = "\"name\"       : \"";
const char json_adv_type                     [] = "\"ADType\"     : ";
const char json_adv_value                    [] = "\"ADValue\"    : \"";
const char json_data_true                    [] = "true";
const char json_data_false                   [] = "false";
const char json_data_reason_code             [] = "reasonCode:         ";
const char json_data_pairing_status_code     [] = "pairingStatusCode:  ";
const char json_data_pairing_status          [] = "pairingStatus:      \"";
const char json_data_pairing_id              [] = "pairingID:          \"0x";
const char json_data_display                 [] = "display:            \"";
const char json_data_bdaddra                 [] = "bdaddra:            \"";
const char json_data_ra                      [] = "ra:                 \"0x";
const char json_data_ca                      [] = "ca:                 \"0x";
const char comma                             [] = ",";
const char close_square_bracket              [] = "]";
const char close_curly_bracket               [] = "}";
const char double_qoute                      [] = "\"";
const char crlf                              [] = "\r\n";

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t rest_smart_response_end_stream( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );
    wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 );
    wiced_http_response_stream_disable_chunked_transfer( stream );
    wiced_http_response_stream_flush( stream );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_status_code( wiced_http_response_stream_t* stream, rest_smart_status_code_t status )
{
    CHECK_STREAM( stream );
    WICED_VERIFY ( wiced_http_response_stream_write_header( stream, restful_smart_http_status_table[status], CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_JSON ) );
    WICED_VERIFY ( wiced_http_response_stream_flush( stream ) );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_send_error_message( wiced_http_response_stream_t* stream, rest_smart_status_code_t status )
{
    CHECK_STREAM( stream );
    WICED_VERIFY( wiced_http_response_stream_enable_chunked_transfer( stream ) );
    WICED_VERIFY( wiced_http_response_stream_write_header( stream, restful_smart_http_status_table[status], CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_JSON ) );
    WICED_VERIFY( rest_smart_response_end_stream( stream ) );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_send_node_data( wiced_http_response_stream_t* stream, const smart_node_handle_t* node )
{
    char address_string[BD_ADDR_BUFFER_LENGTH] = { 0 };
    char node_string   [NODE_BUFFER_LENGTH]    = { 0 };

    CHECK_STREAM( stream );

    wiced_http_response_stream_enable_chunked_transfer( stream );
    rest_smart_response_write_status_code( stream, REST_SMART_STATUS_200 );
    rest_smart_response_write_node_array_start( stream );
    format_node_string( node_string, &node->bda, node->type );
    device_address_to_string( &node->bda, address_string );
    rest_smart_response_write_node( stream, (const char*)node_string, (const char*)address_string );
    rest_smart_response_write_array_end( stream );
    rest_smart_response_end_stream( stream );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_node( wiced_http_response_stream_t* stream, const char* node_handle, const char* bdaddr )
{
    wiced_ip_address_t address;
    char               address_string[16];

    CHECK_STREAM( stream );

    /* Leading \r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,        sizeof( json_data_self_start        ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,              strlen( address_string              )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gap_slash_nodes_slash, sizeof( slash_gap_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle,                 strlen( node_handle                 )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,          sizeof( json_data_self_end          ) - 1 ) );

    /* "handle" : "<node1>", */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle, sizeof( json_data_handle ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle,      strlen( node_handle      )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute,     sizeof( double_qoute     ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma,            sizeof( comma            ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,             sizeof( crlf             ) - 1 ) );

    /* "bdaddr" : "<bdaddr1>" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_bdaddr, sizeof( json_data_bdaddr ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, bdaddr,           strlen( bdaddr           )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute,     sizeof( double_qoute     ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,             sizeof( crlf             ) - 1 ) );

    /* } */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );

    return WICED_SUCCESS;

}

wiced_result_t rest_smart_response_write_service( wiced_http_response_stream_t* stream, const char* node, const char* service, uint16_t handle, const char* uuid, wiced_bool_t is_primary_service, wiced_bool_t is_first_entry )
{
    wiced_ip_address_t address;
    char               address_string[16];
    char               handle_string[5];
    char*              is_primary_string;

    CHECK_STREAM( stream );

    if ( is_first_entry == WICED_FALSE )
    {
        /* Trailing ',' from previous object */
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    }

    /* Leading \r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>/services/<service1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,         sizeof( json_data_self_start         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,               strlen( address_string               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gatt_slash_nodes_slash, sizeof( slash_gatt_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node,                         strlen( node                         )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_services_slash,         sizeof( slash_services_slash         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, service,                      strlen( service                      )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,           sizeof( json_data_self_end           ) - 1 ) );

    /* "handle" : "<service1>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle, sizeof( json_data_handle ) - 1 ) );
    unsigned_to_hex_string( (uint32_t)handle, handle_string, 4, 4 );
    WICED_VERIFY( wiced_http_response_stream_write( stream, handle_string,    strlen( handle_string    )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "uuid" : "<uuid1>,\r\n" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_uuid, sizeof( json_data_uuid ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, uuid,           strlen( uuid           )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "primary" : "<true/false>,\r\n" */
    is_primary_string = ( is_primary_service == WICED_TRUE ) ? "true" : "false";
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_primary, sizeof( json_data_primary ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, is_primary_string, strlen( is_primary_string )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* } */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_characteristic( wiced_http_response_stream_t* stream, const char* node, const char* characteristic, uint16_t handle, const char* uuid, uint8_t properties, wiced_bool_t is_first_entry )
{
    wiced_ip_address_t address;
    char               address_string[16];
    char               handle_string[5];
    char               properties_string[3];

    CHECK_STREAM( stream );

    if ( is_first_entry == WICED_FALSE )
    {
        /* Trailing ',' from previous object */
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    }

    /* Leading \r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>/characteristics/<characteristic1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,         sizeof( json_data_self_start         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,               strlen( address_string               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gatt_slash_nodes_slash, sizeof( slash_gatt_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node,                         strlen( node                         )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_characteristics_slash,  sizeof( slash_characteristics_slash  ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, characteristic,               strlen( characteristic               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,           sizeof( json_data_self_end           ) - 1 ) );

    /* "handle" : "<characteristic1>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle, sizeof( json_data_handle ) - 1 ) );
    unsigned_to_hex_string( (uint32_t)handle, handle_string, 4, 4 );
    WICED_VERIFY( wiced_http_response_stream_write( stream, handle_string,    strlen( handle_string    )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "uuid" : "<uuid1>,\r\n" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_uuid, sizeof( json_data_handle ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, uuid,           strlen( uuid             )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "properties" : "<prop1>,\r\n" */
    memset( &properties_string, 0, sizeof( properties_string ) );
    unsigned_to_hex_string( (uint32_t)properties, properties_string, 2, 2 );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_properties, sizeof( json_data_properties ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, properties_string,    strlen( properties_string    )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* } */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_descriptor( wiced_http_response_stream_t* stream, const char* node, const char* descriptor, uint16_t handle, const char* uuid, wiced_bool_t is_first_entry )
{
    wiced_ip_address_t address;
    char               address_string[16];
    char               handle_string[5];

    CHECK_STREAM( stream );

    if ( is_first_entry == WICED_FALSE )
    {
        /* Trailing ',' from previous object */
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    }

    /* Leading \r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>/descriptors/<descriptor1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,         sizeof( json_data_self_start         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,               strlen( address_string               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gatt_slash_nodes_slash, sizeof( slash_gatt_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node,                         strlen( node                         )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_descriptors_slash,      sizeof( slash_descriptors_slash      ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, descriptor,                   strlen( descriptor                   )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,           sizeof( json_data_self_end           ) - 1 ) );

    /* "handle" : "<descriptor1>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle, sizeof( json_data_handle ) - 1 ) );
    unsigned_to_hex_string( (uint32_t)handle, handle_string, 4, 4 );
    WICED_VERIFY( wiced_http_response_stream_write( stream, handle_string,    strlen( handle_string    )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "uuid" : "<uuid1>\r\n" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_uuid, sizeof( json_data_handle ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, uuid,           strlen( uuid             )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* } */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_long_characteristic_value_start( wiced_http_response_stream_t* stream, const char* node_handle, const char* characteristic_value_handle )
{
    wiced_ip_address_t address;
    char               address_string[16];

    CHECK_STREAM( stream );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>/characteristics/<characteristic1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,         sizeof( json_data_self_start         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,               strlen( address_string               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gatt_slash_nodes_slash, sizeof( slash_gatt_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle,                  strlen( node_handle                  )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_characteristics_slash,  sizeof( slash_characteristics_slash  ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, characteristic_value_handle,  strlen( characteristic_value_handle  )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,           sizeof( json_data_self_end           ) - 1 ) );

    /* "handle" : "<characteristic_value_handle>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle,            sizeof( json_data_handle            ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, characteristic_value_handle, strlen( characteristic_value_handle )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "value" : "<value>,\r\n" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_value, sizeof( json_data_value ) - 1 ) );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_long_partial_characteristic_value( wiced_http_response_stream_t* stream, const uint8_t* partial_value, uint32_t length )
{
    uint32_t a;

    CHECK_STREAM( stream );

    for ( a = 0; a < length; a++ )
    {
        char value_char[3];

        memset( value_char, 0, sizeof( value_char ) );
        unsigned_to_hex_string( partial_value[a], value_char, 2, 2 );
        WICED_VERIFY( wiced_http_response_stream_write( stream, value_char, 2 ) );
    }

    wiced_http_response_stream_flush( stream );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_long_characteristic_value_end( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );

    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* } */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_characteristic_value( wiced_http_response_stream_t* stream, const char* node, const char* characteristic, uint16_t handle, const uint8_t* value, uint32_t value_length )
{
    wiced_ip_address_t address;
    char               address_string[16];
    char               handle_string[5];
    int32_t            a;

    CHECK_STREAM( stream );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>/characteristics/<characteristic1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,         sizeof( json_data_self_start         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,               strlen( address_string               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gatt_slash_nodes_slash, sizeof( slash_gatt_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node,                         strlen( node                         )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_characteristics_slash,  sizeof( slash_characteristics_slash  ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, characteristic,               strlen( characteristic               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,           sizeof( json_data_self_end           ) - 1 ) );

    /* "handle" : "<handle>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle, sizeof( json_data_handle ) - 1 ) );
    unsigned_to_hex_string( (uint32_t)handle, handle_string, 4, 4 );
    WICED_VERIFY( wiced_http_response_stream_write( stream, handle_string,    strlen( handle_string    )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );


    /* "value" : "<value>\r\n". Value is written in network byte order */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_value, sizeof( json_data_value ) - 1 ) );
    for ( a = 0; a < value_length; a++ )
    {
        char value_char[3];

        memset( value_char, 0, sizeof( value_char ) );
        unsigned_to_hex_string( value[a], value_char, 2, 2 );
        WICED_VERIFY( wiced_http_response_stream_write( stream, value_char, 2 ) );
    }
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* }\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_descriptor_value( wiced_http_response_stream_t* stream, const char* node, const char* descriptor_value_handle, const uint8_t* value, uint32_t value_length )
{
    wiced_ip_address_t address;
    char               address_string[16];
    int32_t            a;

    CHECK_STREAM( stream );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>/descriptors/<descriptor1>/value"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,         sizeof( json_data_self_start         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,               strlen( address_string               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gatt_slash_nodes_slash, sizeof( slash_gatt_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node,                         strlen( node                         )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_descriptors_slash,      sizeof( slash_descriptors_slash      ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, descriptor_value_handle,      strlen( descriptor_value_handle      )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,           sizeof( json_data_self_end           ) - 1 ) );

    /* "value" : "<value>\r\n". Value is written in network byte order */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_value, sizeof( json_data_value ) - 1 ) );
    for ( a = 0; a < value_length; a++ )
    {
        char value_char[3];

        memset( value_char, 0, sizeof( value_char ) );
        unsigned_to_hex_string( value[a], value_char, 2, 2 );
        WICED_VERIFY( wiced_http_response_stream_write( stream, value_char, 2 ) );
    }
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* },\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_cached_value( wiced_http_response_stream_t* stream, const char* handle, const char* value )
{
    return WICED_SUCCESS;
}

wiced_result_t restful_smart_write_bonded_nodes( wiced_http_response_stream_t* stream, const char* node_handle, const char* bdaddr, const char* bdaddrtype, uint8_t key_mask, wiced_bool_t is_first_entry )
{
    wiced_ip_address_t address;
    char               address_string[16];
    char*              json_data_state;

    CHECK_STREAM( stream );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/management/nodes/<node1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start, sizeof( json_data_self_start ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string, strlen( address_string ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_management_slash_nodes_slash, sizeof( slash_management_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle, strlen( node_handle ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end, sizeof( json_data_self_end ) - 1 ) );

    /* "handle" : "<node>", */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle, sizeof( json_data_handle ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle, strlen( node_handle ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "bdaddr" : "<bdaddr1>" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_bdaddr, sizeof( json_data_bdaddr ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, bdaddr, strlen( bdaddr ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "bdaddrType" : "<bdaddrType>", */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_bdaddrtype, sizeof( json_data_bdaddrtype ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, bdaddrtype, strlen( bdaddrtype ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "LTK" : <true|false>, */
    json_data_state = (char*) ( ( key_mask & LTK_MASK ) ? json_data_true : json_data_false );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_ltk, sizeof( json_data_ltk ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_state, strlen( json_data_state ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "EDIV" : <true|false>, */
    json_data_state = (char*) ( ( key_mask & EDIV_MASK ) ? json_data_true : json_data_false );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_ediv, sizeof( json_data_ediv ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_state, strlen( json_data_state ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "IRK" : <true|false>, */
    json_data_state = (char*) ( ( key_mask & IRK_MASK ) ? json_data_true : json_data_false );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_irk, sizeof( json_data_irk ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_state, strlen( json_data_state ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* "CSRK" : <true|false>, */
    json_data_state = (char*) ( ( key_mask & CSRK_MASK ) ? json_data_true : json_data_false );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_csrk, sizeof( json_data_csrk ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_state, strlen( json_data_state ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* } */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_gap_node( wiced_http_response_stream_t* stream, const char* node_handle, const char* bdaddr, const char* bdaddrtype, const char* rssi, wiced_bool_t is_first_entry )
{
    wiced_ip_address_t address;
    char               address_string[16];

    CHECK_STREAM( stream );

    if ( is_first_entry == WICED_FALSE )
    {
        /* Trailing ',' from previous object */
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    }

    /* Leading \r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gap/nodes/<node1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,        sizeof( json_data_self_start        ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,              strlen( address_string              )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gap_slash_nodes_slash, sizeof( slash_gap_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle,                 strlen( node_handle                 )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,          sizeof( json_data_self_end          ) - 1 ) );

    /* "handle" : "<node>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_handle, sizeof( json_data_handle ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle,      strlen( node_handle      )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute,     sizeof( double_qoute     ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma,            sizeof( comma            ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,             sizeof( crlf             ) - 1 ) );

    /* "bdaddr" : "<bdaddr>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_bdaddr, sizeof( json_data_bdaddr ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, bdaddr,           strlen( bdaddr           )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute,     sizeof( double_qoute     ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma,            sizeof( comma            ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,             sizeof( crlf             ) - 1 ) );

    /* "bdaddrType" : "<bdaddrType>",\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_bdaddrtype, sizeof( json_data_bdaddrtype ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, bdaddrtype,           strlen( bdaddrtype           )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute,         sizeof( double_qoute         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma,                sizeof( comma                ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,                 sizeof( crlf                 ) - 1 ) );

    /* "rssi" : "<signed value>", */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_rssi, sizeof( json_data_rssi ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, rssi,           strlen( rssi           )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma,          sizeof( comma          ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,           sizeof( crlf           ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_adv_data( wiced_http_response_stream_t* stream, const char* adv_type, const char* adv_data, wiced_bool_t is_first_entry )
{
    CHECK_STREAM( stream );

    if ( is_first_entry == WICED_FALSE )
    {
        /* Trailing ',' from previous object */
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    }

    /* Leading \r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* "ADType" : <type>, */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_adv_type, sizeof( json_adv_type ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, adv_type,      strlen( adv_type      )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma,         sizeof( comma         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,          sizeof( crlf          ) - 1 ) );

    /* "ADValue" : "<value>" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_adv_value, sizeof( json_adv_value ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, adv_data,       strlen( adv_data       )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute,   sizeof( double_qoute   ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf,           sizeof( crlf           ) - 1 ) );

    /* } */
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );

    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_send_pairing_data( wiced_http_response_stream_t* stream, rest_smart_pairing_response_t* response )
{
    CHECK_STREAM( stream );

    wiced_http_response_stream_enable_chunked_transfer( stream );

    /* HTTP code */
    WICED_VERIFY( rest_smart_response_write_status_code( stream, response->status_code ) );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start, sizeof( json_object_start ) - 1 ) );

    /* pairingStatusCode: */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_pairing_status_code, sizeof( json_data_pairing_status_code ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, pairing_status_table[ response->pairing_status_code ].status_code, 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* pairingStatus: */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_pairing_status, sizeof( json_data_pairing_status ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, pairing_status_table[response->pairing_status_code].status_string, strlen( pairing_status_table[ response->pairing_status_code ].status_string ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    if ( response->pairing_status_code != PAIRING_SUCCESSFUL )
    {
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
    }
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

    /* reason code */
    if ( response->pairing_status_code == PAIRING_FAILED )
    {
        char reason_code[2] = { 0 };

        unsigned_to_decimal_string( (uint32_t)response->reason_code, reason_code, 1, 1 );
        WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_reason_code, sizeof( json_data_reason_code ) - 1 ) );
        WICED_VERIFY( wiced_http_response_stream_write( stream, reason_code, 1 ) );
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
        WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
    }

    /* Display */
    switch ( response->pairing_status_code )
    {
        case LE_LEGACY_OOB_EXPECTED:
        case PASSKEY_DISPLAY_EXPECTED:
        case NUMERIC_COMPARISON_EXPECTED:
        {
            char passkey[ 11 ] = { 0 };

            unsigned_to_decimal_string( response->passkey, passkey, 1, sizeof( passkey ) - 1 );
            WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_display, sizeof( json_data_display ) - 1 ) );
            WICED_VERIFY( wiced_http_response_stream_write( stream, passkey, strlen( passkey ) ) );
            WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
            WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
            WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
            break;
        }
        default:
            break;
    }

    /* Pairing ID */
    switch ( response->pairing_status_code )
    {
        case LE_SECURE_OOB_EXPECTED:
        case LE_LEGACY_OOB_EXPECTED:
        case PASSKEY_INPUT_EXPECTED:
        case PASSKEY_DISPLAY_EXPECTED:
        case NUMERIC_COMPARISON_EXPECTED:
        {
            uint8_t a;

            WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_pairing_id, sizeof( json_data_pairing_id ) - 1 ) );
            for ( a = 0; a < 16; a++ )
            {
                char hex_digit[ 3 ] = { 0 };

                unsigned_to_hex_string( (uint32_t)response->pairing_id[a], hex_digit, 2, 2 );
                WICED_VERIFY( wiced_http_response_stream_write( stream, hex_digit, 2 ) );
            }
            if ( response->pairing_status_code == LE_SECURE_OOB_EXPECTED )
            {
                WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
            }
            WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
            break;
        }
        default:
            break;
    }

    /* bdaddra, ra, and ca */
    if ( response->pairing_status_code == LE_SECURE_OOB_EXPECTED )
    {
        uint8_t a;

        /* bdaddra */
        WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_bdaddra, sizeof( json_data_bdaddra ) - 1 ) );
        for ( a = 0; a < 6; a++ )
        {
            char hex_digit[ 3 ] = { 0 };

            unsigned_to_hex_string( (uint32_t)response->bdaddra[a], hex_digit, 2, 2 );
            WICED_VERIFY( wiced_http_response_stream_write( stream, hex_digit, 2 ) );
        }
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
        WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

        /* ra */
        WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_ra, sizeof( json_data_ra ) - 1 ) );
        for ( a = 0; a < 16; a++ )
        {
            char hex_digit[ 3 ] = { 0 };

            unsigned_to_hex_string( (uint32_t)response->ra[a], hex_digit, 2, 2 );
            WICED_VERIFY( wiced_http_response_stream_write( stream, hex_digit, 2 ) );
        }
        WICED_VERIFY( wiced_http_response_stream_write( stream, comma, sizeof( comma ) - 1 ) );
        WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );

        /* ca */
        WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_ca, sizeof( json_data_ca ) - 1 ) );
        for ( a = 0; a < 16; a++ )
        {
            char hex_digit[ 3 ] = { 0 };

            unsigned_to_hex_string( (uint32_t)response->ca[a], hex_digit, 2, 2 );
            WICED_VERIFY( wiced_http_response_stream_write( stream, hex_digit, 2 ) );
        }
        WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
    }

    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
    WICED_VERIFY( rest_smart_response_end_stream( stream ) );
    return WICED_SUCCESS;
}

wiced_result_t restful_smart_write_node_name( wiced_http_response_stream_t* stream, const char* node_handle, const char* bdaddr, const char* node_name )
{
    wiced_ip_address_t address;
    char               address_string[16];

    CHECK_STREAM( stream );

    /* {\r\n */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_object_start,            sizeof( json_object_start            ) - 1 ) );

    /* "self" : {"href"="http://<gateway>/gatt/nodes/<node1>"},\r\n */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &address );
    memset( &address_string, 0, sizeof( address_string ) );
    ip_to_str( (const wiced_ip_address_t*)&address, address_string );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_start,         sizeof( json_data_self_start         ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, address_string,               strlen( address_string               )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, slash_gatt_slash_nodes_slash, sizeof( slash_gatt_slash_nodes_slash ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_handle,                  strlen( node_handle                  )     ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_self_end,           sizeof( json_data_self_end           ) - 1 ) );

    /* "name" : "<name>" */
    WICED_VERIFY( wiced_http_response_stream_write( stream, json_data_name, sizeof( json_data_name ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, node_name, strlen( node_name ) ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, double_qoute, sizeof( double_qoute ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket, sizeof( close_curly_bracket ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );


    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_write_node_array_start( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
    return wiced_http_response_stream_write( stream, json_node_array_start, sizeof( json_node_array_start ) - 1 );
}

wiced_result_t rest_smart_response_write_service_array_start( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );
    return wiced_http_response_stream_write( stream, json_service_array_start, sizeof( json_service_array_start ) - 1 );
}

wiced_result_t rest_smart_response_write_characteristic_array_start( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );
    return wiced_http_response_stream_write( stream, json_characteristic_array_start, sizeof( json_characteristic_array_start ) - 1 );
}

wiced_result_t rest_smart_response_write_descriptor_array_start( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );
    return wiced_http_response_stream_write( stream, json_descriptor_array_start, sizeof( json_descriptor_array_start ) - 1 );
}

wiced_result_t rest_smart_response_write_adv_array_start( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );
    return wiced_http_response_stream_write( stream, json_adv_array_start, sizeof( json_adv_array_start ) - 1 );
}

wiced_result_t rest_smart_response_write_array_end( wiced_http_response_stream_t* stream )
{
    CHECK_STREAM( stream );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_square_bracket,  sizeof( close_square_bracket  ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, crlf, sizeof( crlf ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_write( stream, close_curly_bracket,  sizeof( close_curly_bracket  ) - 1 ) );
    wiced_http_response_stream_flush( stream );
    return WICED_SUCCESS;
}

wiced_result_t rest_smart_response_send_notification( wiced_http_response_stream_t* stream, wiced_bt_gatt_data_t* data )
{
     uint32_t a;
    if ( stream == NULL )
     {
        return WICED_ERROR;
     }

    if ( stream->tcp_stream.socket == NULL )
     {
        return WICED_ERROR;
     }

    /* SSE is prefixed with "data: " */
    WICED_VERIFY( wiced_http_response_stream_write( stream, (const void*)EVENT_STREAM_DATA, sizeof( EVENT_STREAM_DATA ) - 1 ) );

    /* Send current data back to the client */
    for ( a = 0; a < data->len; a++ )
    {
        char value_char[3];
        memset( value_char, 0, sizeof( value_char ) );
        unsigned_to_hex_string( data->p_data[a], value_char, 2, 2 );
        WICED_VERIFY( wiced_http_response_stream_write( stream, value_char, 2 ) );
    }
    //WICED_VERIFY( wiced_http_response_stream_write( stream, (const void*)data->p_data, (uint32_t)data->len ) );

    /* SSE is ended with two line feeds */
    WICED_VERIFY( wiced_http_response_stream_write( stream, (const void*)LFLF, sizeof( LFLF ) - 1 ) );
    WICED_VERIFY( wiced_http_response_stream_flush( stream ) );
    return WICED_SUCCESS;
}

void string_to_device_address( const char* string, wiced_bt_device_address_t* address )
{
    uint32_t i;
    uint32_t j;

    /* BD_ADDR is passed within the URL. Parse it here */
    for ( i = 0, j = 0; i < 6; i++, j += 2 )
    {
        char  buffer[3] = {0};
        char* end;

        buffer[0] = string[j];
        buffer[1] = string[j + 1];
        (*address)[i] = strtoul( buffer, &end, 16 );
    }
}

void device_address_to_string( const wiced_bt_device_address_t* device_address, char* string )
{
    uint8_t a;

    for ( a = 0; a < 6; a++ )
    {
        unsigned_to_hex_string( (uint32_t)(*device_address)[a], string, 2, 2 );
        string += 2;
    }
}

void string_to_uuid( const char* string, wiced_bt_uuid_t* uuid )
{
    int8_t   i;
    uint8_t  j;
    uint8_t* uuid_iter = (uint8_t*)&uuid->uu;

    uuid->len = strlen( string ) / 2;

    for ( i = uuid->len - 1, j = 0; i >= 0; i--, j += 2 )
    {
        char  buffer[3] = {0};
        char* end;

        buffer[0] = string[j];
        buffer[1] = string[j + 1];
        uuid_iter[i] = strtoul( buffer, &end, 16 );
    }
}

void uuid_to_string( const wiced_bt_uuid_t* uuid, char* string )
{
    int8_t   i;
    uint8_t  j;
    uint8_t* uuid_iter = (uint8_t*)&uuid->uu;

    for ( i = uuid->len - 1, j = 0; i >= 0; i--, j += 2 )
    {
        unsigned_to_hex_string( (uint32_t)uuid_iter[i], &string[j], 2, 2 );
    }
}

void format_node_string( char* output, const wiced_bt_device_address_t* address, wiced_bt_ble_address_type_t type )
{
    device_address_to_string( address, output );
    unsigned_to_hex_string( (uint32_t)type, output + 12, 1, 1 );
}

void format_service_string( char* output, uint16_t start_handle, uint16_t end_handle )
{
    unsigned_to_hex_string( (uint32_t)end_handle,   &output[0], 4 , 4 );
    unsigned_to_hex_string( (uint32_t)start_handle, &output[4], 4 , 4 );
}

void format_characteristic_string( char* output, uint16_t start_handle, uint16_t end_handle, uint16_t value_handle )
{
    unsigned_to_hex_string( (uint32_t)end_handle,   &output[0], 4, 4 );
    unsigned_to_hex_string( (uint32_t)value_handle, &output[4], 4, 4 );
    unsigned_to_hex_string( (uint32_t)start_handle, &output[8], 4, 4 );
}

int ip_to_str( const wiced_ip_address_t* address, char* string )
{
    if ( address->version == WICED_IPV4 )
    {
        unsigned_to_decimal_string( ( ( address->ip.v4 >> 24 ) & 0xff ), string, 1, 3 );
        string += strlen( string );
        *string++ = '.';

        unsigned_to_decimal_string( ( ( address->ip.v4 >> 16 ) & 0xff ), string, 1, 3 );
        string += strlen( string );
        *string++ = '.';

        unsigned_to_decimal_string( ( ( address->ip.v4 >> 8 ) & 0xff ), string, 1, 3 );
        string += strlen( string );
        *string++ = '.';

        unsigned_to_decimal_string( ( address->ip.v4 & 0xff ), string, 1, 3 );
        string += strlen( string );
        *string++ = '\0';
    }
    else
    {
        return -1;
    }

    return 0;
}
