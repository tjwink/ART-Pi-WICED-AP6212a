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
#include "restful_smart_ble.h"
#include "restful_smart_response.h"
#include "restful_smart_uri.h"
#include "http_server.h"
#include "wiced_bt_dev.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define VERIFY_CONDITION( condition ) \
    do \
    { \
        if ( !( condition ) ) \
        { \
            result = WICED_BADARG; \
            goto exit; \
        } \
    } \
    while ( 0 )

#define VERIFY_PARAMETER_COUNT( expected_count ) \
    do \
    { \
        if ( wiced_http_get_query_parameter_count( url_query_string ) != ( expected_count ) ) \
        { \
            result = WICED_BADARG; \
            goto exit; \
        } \
    } \
    while ( 0 )

#define VERIFY_METHOD( expected_method ) \
    do \
    { \
        if ( http_message_body->request_type != ( expected_method ) ) \
        { \
            result = WICED_BADARG; \
            goto exit; \
        } \
    } \
    while ( 0 )

#define VERIFY_PARAMETER_VALUE( value, length ) \
    do \
    { \
        if ( ( value == NULL ) || ( length == 0 ) ) \
        { \
            result = WICED_BADARG; \
            goto exit; \
        } \
    } \
    while ( 0 )

#ifdef DEBUG
#define PRINT_REST_API( rest_api ) \
    do \
    { \
        WPRINT_LIB_INFO( ( rest_api ) ); \
    } \
    while ( 0 )
#else
#define PRINT_REST_API( rest_api )
#endif

/******************************************************
 *                    Constants
 ******************************************************/

#define REST_PARAMETER_KEY_ACTIVE               (const char*)"active"
#define REST_PARAMETER_KEY_PASSIVE              (const char*)"passive"
#define REST_PARAMETER_KEY_ENABLE               (const char*)"enable"
#define REST_PARAMETER_KEY_CONNECT              (const char*)"connect"
#define REST_PARAMETER_KEY_LONG                 (const char*)"long"
#define REST_PARAMETER_KEY_NAME                 (const char*)"name"
#define REST_PARAMETER_KEY_PRIMARY              (const char*)"primary"
#define REST_PARAMETER_KEY_UUID                 (const char*)"uuid"
#define REST_PARAMETER_KEY_INDICATE             (const char*)"indicate"
#define REST_PARAMETER_KEY_NOTIFY               (const char*)"notify"
#define REST_PARAMETER_KEY_EVENT                (const char*)"event"
#define REST_PARAMETER_KEY_MULTIPLE             (const char*)"multiple"
#define REST_PARAMETER_KEY_RELIABLE             (const char*)"reliable"
#define REST_PARAMETER_KEY_NO_RESPONSE          (const char*)"no-response"
#define REST_PARAMETER_KEY_BOND                 (const char*)"bond"
#define REST_PARAMETER_KEY_IO_CAPABILITY        (const char*)"io-capability"
#define REST_PARAMETER_KEY_OOB                  (const char*)"oob"
#define REST_PARAMETER_KEY_LEGACY_OOB           (const char*)"legacy-oob"
#define REST_PARAMETER_KEY_PAIRING_ID           (const char*)"pairing-id"
#define REST_PARAMETER_KEY_PAIRING_ABORT        (const char*)"pairing-abort"
#define REST_PARAMETER_KEY_TK                   (const char*)"tk"
#define REST_PARAMETER_KEY_PASSKEY              (const char*)"passkey"
#define REST_PARAMETER_KEY_BDADDRB              (const char*)"bdaddrb"
#define REST_PARAMETER_KEY_RB                   (const char*)"rb"
#define REST_PARAMETER_KEY_CB                   (const char*)"cb"

#define REST_PARAMETER_VALUE_0                  (const char*)"0"
#define REST_PARAMETER_VALUE_1                  (const char*)"1"
#define REST_PARAMETER_VALUE_DISPLAY_ONLY       (const char*)"DisplayOnly"
#define REST_PARAMETER_VALUE_DISPLAY_YES_NO     (const char*)"DisplayYesNo"
#define REST_PARAMETER_VALUE_KEYBOARD_ONLY      (const char*)"KeyboardOnly"
#define REST_PARAMETER_VALUE_NO_INPUT_NO_OUTPUT (const char*)"NoInputNoOutput"
#define REST_PARAMETER_VALUE_KEYBOARD_DISPLAY   (const char*)"KeyboardDisplay"

#define JSON_NAME_HANDLE                        (const char*)"\"handle\""

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

static wiced_result_t parse_node_handle                         ( const char* url_path, smart_node_handle_t* node );
static wiced_result_t parse_service_handle                      ( const char* url_path, smart_service_handle_t* service );
static wiced_result_t parse_characteristic_handle               ( const char* url_path, smart_characteristic_handle_t* characteristic );
static wiced_result_t parse_descriptor_handle                   ( const char* url_path, uint16_t* descriptor );
static wiced_result_t parse_128bit_hex_string                   ( const char* input, uint8_t* output );
static wiced_result_t create_value_handle                       ( const char* url_path, smart_value_handle_t** value );
static wiced_result_t delete_value_handle                       ( smart_value_handle_t* value );
//static wiced_result_t parse_read_multiple_values_message_body   ( const char* message_body, wiced_bt_gatt_read_param_t* parameter );
static wiced_result_t send_error_response_header                ( wiced_http_response_stream_t* stream, wiced_result_t result );

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

int32_t discover_services( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    wiced_bool_t is_primary = WICED_FALSE;
    smart_node_handle_t node;
    uint32_t value_length;
    uint32_t parameter_count;
    char* parameter_value;

    VERIFY_METHOD( WICED_HTTP_GET_REQUEST );

    parse_node_handle( url_path, &node );
    parameter_count = wiced_http_get_query_parameter_count( url_query_string );

    if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_PRIMARY, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        if ( parameter_count == 1 )
        {
            PRINT_REST_API( "Discover all primary services\n" );
            result = restful_smart_discover_all_primary_services( stream, &node );
        }
        is_primary = 1;
    }

    if ( wiced_http_get_query_parameter_value( url_query_string, REST_PARAMETER_KEY_UUID, &parameter_value, &value_length ) == WICED_SUCCESS )
    {
        wiced_bt_uuid_t uuid;

        VERIFY_PARAMETER_VALUE( parameter_value, value_length );

        /* Discovery primary services by UUID within <node> ( parameter: {primary=1&}uuid=<uuid> ) */
        if ( ( parameter_count == 1 ) || ( ( parameter_count == 2 ) && ( is_primary == WICED_TRUE ) ) )
        {
            PRINT_REST_API( "Discover primary services by UUID\n" );
            string_to_uuid( parameter_value, &uuid );
            result = restful_smart_discover_primary_services_by_uuid( stream, &node, &uuid );
        }
    }

    exit: if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t discover_characteristics_of_a_service( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_node_handle_t node;
    smart_service_handle_t service;

    VERIFY_PARAMETER_COUNT( 0 );
    VERIFY_METHOD( WICED_HTTP_GET_REQUEST );

    parse_node_handle( url_path, &node );
    parse_service_handle( url_path, &service );

    PRINT_REST_API( "Discover characteristics of a service\n" );
    result = restful_smart_discover_characteristics_of_a_service( stream, &node, &service );

    exit: if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t discover_characteristics_by_uuid( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_node_handle_t node;
    wiced_bt_uuid_t uuid;
    char* parameter_value;
    uint32_t value_length;

    VERIFY_PARAMETER_COUNT( 1 );

    wiced_http_get_query_parameter_value( url_query_string, REST_PARAMETER_KEY_UUID, &parameter_value, &value_length );
    VERIFY_PARAMETER_VALUE( parameter_value, value_length );

    parse_node_handle( url_path, &node );
    string_to_uuid( parameter_value, &uuid );

    PRINT_REST_API( "Discover characteristics by UUID\n" );
    result = restful_smart_discover_characteristics_by_uuid( stream, &node, &uuid );

    exit: if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t read_characteristic_value( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_node_handle_t node;
    smart_characteristic_handle_t characteristic;
    uint32_t parameter_count;

    parameter_count = wiced_http_get_query_parameter_count( url_query_string );
    VERIFY_CONDITION( parameter_count <= 2 );

    parse_node_handle( url_path, &node );
    parse_characteristic_handle( url_path, &characteristic );

    switch ( parameter_count )
    {
        case 0:
            PRINT_REST_API( "Read characteristic value\n" );
            result = restful_smart_read_characteristic_value( stream, &node, &characteristic );
            break;
        case 1:
            if ( ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_INDICATE, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS ) || ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_NOTIFY, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS ) )
            {
                PRINT_REST_API( "Read latest cached value\n" );
                result = restful_smart_read_cached_value( stream, &node, &characteristic );
            }
            else if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_LONG, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
            {
                PRINT_REST_API( "Read long characteristic value\n" );
                result = restful_smart_read_characteristic_value( stream, &node, &characteristic );
            }
            break;
        case 2:
            if ( ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_INDICATE, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS ) && ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_EVENT, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS ) )
            {
                PRINT_REST_API( "Subscribe to indication event\n" );
                /* Subscribe to indication (parameter: indicate=1&event=1) */
                result = restful_smart_subscribe_indication(stream,&node,&characteristic);
            }
            else if ( ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_NOTIFY, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS ) && ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_EVENT, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS ) )
            {
                PRINT_REST_API( "Subscribe to notification event\n" );
                /*  Subscribe to notification (parameter: notify=1&event=1) */
                result = restful_smart_subscribe_notification(stream,&node,&characteristic);
            }
            break;
        default:
            break;
    }

    exit: if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return 0;
}

int32_t read_write_characteristic_values( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_node_handle_t node;
    smart_characteristic_handle_t characteristic;
    char* parameter_value;
    uint32_t value_length;

    VERIFY_PARAMETER_COUNT( 1 );

    parse_node_handle( url_path, &node );
    parse_characteristic_handle( url_path, &characteristic );

    if ( wiced_http_get_query_parameter_value( url_query_string, REST_PARAMETER_KEY_UUID, &parameter_value, &value_length ) == WICED_SUCCESS )
    {
        wiced_bt_uuid_t uuid;

        VERIFY_METHOD( WICED_HTTP_GET_REQUEST );
        VERIFY_PARAMETER_VALUE( parameter_value, value_length );

        /* Read values of characteristic with given UUID (parameter: uuid=<uuid> */
        string_to_uuid( parameter_value, &uuid );

        PRINT_REST_API( "Read characteristic values by UUID\n" );
        result = restful_smart_read_characteristic_values_by_uuid( stream, &node, NULL, &uuid );
    }
    else if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_MULTIPLE, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        /* TODO: Read multiple (parameter: multiple=1) */

//        wiced_bt_gatt_read_param_t parameter;
//        VERIFY_METHOD( WICED_HTTP_GET_REQUEST );
//        VERIFY_CONDITION( parse_read_multiple_values_message_body( (const char*)http_message_body->data, &parameter ) == WICED_SUCCESS );
//        PRINT_REST_API( "Read multiple characteristic values\n" );
//        result = restful_smart_read_multiple_characteristic_values( stream, &node, &parameter );
    }
    else if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_RELIABLE, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        VERIFY_METHOD( WICED_HTTP_PUT_REQUEST );

        PRINT_REST_API( "Perform reliable write\n" );
        /* TODO: Reliable write (parameter: reliable=1) */
    }

    exit: if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t write_characteristic_value( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_value_handle_t* value = NULL;
    smart_node_handle_t node;
    smart_characteristic_handle_t characteristic;
    uint32_t parameter_count;

    VERIFY_METHOD( WICED_HTTP_PUT_REQUEST );

    parameter_count = wiced_http_get_query_parameter_count( url_query_string );
    VERIFY_CONDITION( parameter_count <= 1 );

    if ( create_value_handle( url_path, &value ) != WICED_SUCCESS )
    {
        goto exit;
    }

    parse_node_handle( url_path, &node );
    parse_characteristic_handle( url_path, &characteristic );

    switch ( parameter_count )
    {
        case 0:
            PRINT_REST_API( "Write characteristic value\n" );
            result = restful_smart_write_characteristic_value( stream, &node, &characteristic, value );
            break;
        case 1:
            if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_NO_RESPONSE, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
            {
                PRINT_REST_API( "Write characteristic value without response\n" );
                result = restful_smart_write_characteristic_value_without_response( stream, &node, &characteristic, value );
            }
            else if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_LONG, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
            {
                PRINT_REST_API( "Write long characteristic value\n" );
                result = restful_smart_write_characteristic_value( stream, &node, &characteristic, value );
            }
            break;
        default:
            break;
    }

    exit: if ( value != NULL )
    {
        delete_value_handle( value );
    }
    if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t manage_characteristic( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_node_handle_t node;
    smart_characteristic_handle_t characteristic;

    VERIFY_PARAMETER_COUNT( 1 );

    parse_node_handle( url_path, &node );
    parse_characteristic_handle( url_path, &characteristic );

    if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_INDICATE, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        PRINT_REST_API( "Enable indication\n" );
        result = restful_smart_enable_indication( stream, &node, &characteristic );
    }
    else if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_INDICATE, REST_PARAMETER_VALUE_0 ) == WICED_SUCCESS )
    {
        PRINT_REST_API( "Disable indication\n" );
        result = restful_smart_disable_indication( stream, &node, &characteristic );
    }
    else if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_NOTIFY, REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        PRINT_REST_API( "Enable notification\n" );
        result = restful_smart_enable_notification( stream, &node, &characteristic );
    }
    else if ( wiced_http_match_query_parameter( url_query_string, REST_PARAMETER_KEY_NOTIFY, REST_PARAMETER_VALUE_0 ) == WICED_SUCCESS )
    {
        PRINT_REST_API( "Disable notification\n" );
        result = restful_smart_disable_notification( stream, &node, &characteristic );
    }

    exit: if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t discover_descriptors( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_node_handle_t node;
    smart_characteristic_handle_t characteristic;

    VERIFY_PARAMETER_COUNT( 0 );
    VERIFY_METHOD( WICED_HTTP_GET_REQUEST );

    parse_node_handle( url_path, &node );
    parse_characteristic_handle( url_path, &characteristic );

    PRINT_REST_API( "Discover characteristic descriptors\n" );
    result = restful_smart_discover_characteristic_descriptors( stream, &node, &characteristic );

    exit: if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t read_descriptor_value( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_node_handle_t node;
    uint16_t descriptor;
    uint32_t parameter_count;

    parameter_count = wiced_http_get_query_parameter_count( url_query_string );
    VERIFY_CONDITION( parameter_count <= 1 );
    VERIFY_METHOD( WICED_HTTP_GET_REQUEST );

    parse_node_handle( url_path, &node );
    parse_descriptor_handle( url_path, &descriptor );

    PRINT_REST_API( "Read descriptor value\n" );
    result = restful_smart_read_descriptor_value( stream, &node, descriptor );

    exit:
    if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t write_descriptor_value( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_BADARG;
    smart_value_handle_t* value = NULL;
    smart_node_handle_t node;
    uint16_t descriptor;
    uint32_t parameter_count;

    parameter_count = wiced_http_get_query_parameter_count( url_query_string );
    VERIFY_CONDITION( parameter_count <= 1 );
    VERIFY_METHOD( WICED_HTTP_PUT_REQUEST );

    if ( create_value_handle( url_path, &value ) != WICED_SUCCESS )
    {
        result = WICED_NOT_FOUND;
        goto exit;
    }

    parse_node_handle( url_path, &node );
    parse_descriptor_handle( url_path, &descriptor );

    PRINT_REST_API( "Write descriptor value\n" );
    result = restful_smart_write_descriptor_value( stream, &node, descriptor, value );

    exit:
    if ( value != NULL )
    {
        delete_value_handle( value );
    }
    if ( result == WICED_BADARG )
    {
        send_error_response_header( stream, result );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t list_bonded_nodes( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    send_error_response_header( stream, HTTP_404_TYPE );
    return -1;
}

int32_t manage_node_security( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    char* parameter_value = NULL;
    uint32_t parameter_length = 0;
    wiced_result_t result = WICED_ERROR;
    smart_node_handle_t node;

    parse_node_handle( url_path, &node );

    if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_BOND, (const char*) REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        rest_smart_pairing_type_t pairing_type;

        if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_OOB, (const char*) REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
        {
            pairing_type = REST_SMART_PAIRING_SECURE_CONNECTIONS_OOB;
        }
        else if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_LEGACY_OOB, (const char*) REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
        {
            pairing_type = REST_SMART_PAIRING_LEGACY_OOB;
        }
        else if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_IO_CAPABILITY, (const char*) REST_PARAMETER_VALUE_DISPLAY_ONLY ) == WICED_SUCCESS )
        {
            pairing_type = REST_SMART_PAIRING_DISPLAY_ONLY;
        }
        else if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_IO_CAPABILITY, (const char*) REST_PARAMETER_VALUE_DISPLAY_YES_NO ) == WICED_SUCCESS )
        {
            pairing_type = REST_SMART_PAIRING_DISPLAY_YES_NO;
        }
        else if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_IO_CAPABILITY, (const char*) REST_PARAMETER_VALUE_KEYBOARD_ONLY ) == WICED_SUCCESS )
        {
            pairing_type = REST_SMART_PAIRING_KEYBOARD_ONLY;
        }
        else if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_IO_CAPABILITY, (const char*) REST_PARAMETER_VALUE_NO_INPUT_NO_OUTPUT ) == WICED_SUCCESS )
        {
            pairing_type = REST_SMART_PAIRING_NO_IO;
        }
        else
        {
            pairing_type = REST_SMART_PAIRING_KEYBOARD_DISPLAY;
        }

        result = restful_smart_start_pairing( stream, &node, pairing_type );
    }
    else if ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_PAIRING_ID, &parameter_value, &parameter_length ) == WICED_SUCCESS )
    {
        rest_smart_client_pairing_response_t response;

        memset( &response, 0, sizeof( response ) );

        /* Parse pairing ID */
        parse_128bit_hex_string( parameter_value, response.pairing_id );
        response.pairing_id_available = WICED_TRUE;

        if ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_TK, &parameter_value, &parameter_length ) == WICED_SUCCESS )
        {
            parse_128bit_hex_string( parameter_value, response.tk );
            response.tk_available = WICED_TRUE;
        }

        if ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_PASSKEY, &parameter_value, &parameter_length ) == WICED_SUCCESS )
        {
            string_to_unsigned( parameter_value, parameter_length, &response.passkey, 0 );
            response.passkey_available = WICED_TRUE;
        }

        if ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_BDADDRB, &parameter_value, &parameter_length ) == WICED_SUCCESS )
        {
            string_to_device_address( parameter_value, &response.bdaddrb );
            response.bdaddrb_available = WICED_TRUE;
        }

        if ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_RB, &parameter_value, &parameter_length ) == WICED_SUCCESS )
        {
            parse_128bit_hex_string( parameter_value, response.rb );
            response.rb_available = WICED_TRUE;
        }

        if ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_CB, &parameter_value, &parameter_length ) == WICED_SUCCESS )
        {
            parse_128bit_hex_string( parameter_value, response.cb );
            response.cb_available = WICED_TRUE;
        }

        result = restful_smart_process_client_pairing_response( stream, &node, &response );
    }
    else if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_PAIRING_ABORT, (const char*) REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        result = restful_smart_cancel_pairing( stream, &node );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t discover_nodes( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_ERROR;
    char* parameter_value;
    uint32_t value_length;

    UNUSED_PARAMETER( url_path );

    if ( wiced_http_get_query_parameter_count( url_query_string ) != 1 )
    {
        /* TODO: return HTTP code here */
        return -1;
    }

    if ( wiced_http_match_query_parameter( url_query_string, (const char*) REST_PARAMETER_KEY_PASSIVE, (const char*) REST_PARAMETER_VALUE_1 ) == WICED_SUCCESS )
    {
        result = restful_smart_passive_scan( stream );
    }
    else if ( ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_ACTIVE, &parameter_value, &value_length ) == WICED_SUCCESS ) && ( strncmp( parameter_value, REST_PARAMETER_VALUE_1, value_length ) == 0 ) )
    {
        result = restful_smart_active_scan( stream );
    }
    else if ( ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_ENABLE, &parameter_value, &value_length ) == WICED_SUCCESS ) && ( strncmp( parameter_value, REST_PARAMETER_VALUE_1, value_length ) == 0 ) )
    {
//        result = restful_smart_enabled_node_data( stream, NULL );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

int32_t manage_node_connection( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    wiced_result_t result = WICED_ERROR;
    char* parameter_value;
    uint32_t value_length;
    uint32_t parameter_count;
    smart_node_handle_t node;

    parse_node_handle( url_path, &node );

    parameter_count = wiced_http_get_query_parameter_count( url_query_string );
    if ( parameter_count == 1 )
    {
        if ( ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_ENABLE, &parameter_value, &value_length ) == WICED_SUCCESS ) && ( strncmp( parameter_value, REST_PARAMETER_VALUE_0, value_length ) == 0 ) )
        {
            /* Disable node (parameter: enable=0) */
//            result = restful_smart_remove_device( stream, &node.address );
        }
        else if ( ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_NAME, &parameter_value, &value_length ) == WICED_SUCCESS ) && ( strncmp( parameter_value, REST_PARAMETER_VALUE_1, value_length ) == 0 ) )
        {
            /* Discovery node name (parameter: node=1) */
//            result = restful_smart_discover_node_name( stream, &node.address );
        }
    }

    if ( ( wiced_http_get_query_parameter_value( url_query_string, (const char*) REST_PARAMETER_KEY_CONNECT, &parameter_value, &value_length ) == WICED_SUCCESS ) && ( strncmp( parameter_value, REST_PARAMETER_VALUE_1, value_length ) == 0 ) )
    {
        if ( parameter_count > 1 )
        {
            /* TODO: process optional parameters */
        }

        /* Connect to node (parameter: connect=1) */
        result = restful_smart_connect( stream, &node );
    }

    return ( result == WICED_SUCCESS ) ? 0 : -1;
}

static wiced_result_t parse_node_handle( const char* url_path, smart_node_handle_t* node )
{
    /* /gatt/nodes/<node> */
    char* current_path = (char*) url_path;
    uint32_t a = 0;
    uint32_t temp = 0;

    while ( a < 3 )
    {
        if ( *current_path == '\0' )
        {
            /* <node> not found */
            return WICED_NOT_FOUND;
        }
        else if ( *current_path == '/' )
        {
            a++;
        }

        current_path++;
    }

    string_to_device_address( current_path, &node->bda );
    string_to_unsigned( current_path + 12, 1, &temp, 1 );
    node->type = (wiced_bt_ble_address_type_t) ( temp & 0xff );
    return WICED_SUCCESS;
}

static wiced_result_t parse_service_handle( const char* url_path, smart_service_handle_t* service )
{
    /* /gatt/nodes/<node>/services/<service> */
    char* current_path = (char*) url_path;
    uint32_t a = 0;
    uint32_t temp = 0;

    while ( a < 5 )
    {
        if ( *current_path == '\0' )
        {
            /* <service> not found */
            return WICED_NOT_FOUND;
        }
        else if ( *current_path == '/' )
        {
            a++;
        }

        current_path++;
    }

    string_to_unsigned( current_path, 4, &temp, 1 );
    service->end_handle = (uint16_t) ( temp & 0xffff );
    string_to_unsigned( current_path + 4, 4, &temp, 1 );
    service->start_handle = (uint16_t) ( temp & 0xffff );
    return WICED_SUCCESS;
}

static wiced_result_t parse_characteristic_handle( const char* url_path, smart_characteristic_handle_t* characteristic )
{
    /* /gatt/nodes/<node>/characteristics/<characteristic> */
    char* current_path = (char*) url_path;
    uint32_t a = 0;
    uint32_t temp = 0;

    while ( a < 5 )
    {
        if ( *current_path == '\0' )
        {
            /* <service> not found */
            return WICED_NOT_FOUND;
        }
        else if ( *current_path == '/' )
        {
            a++;
        }

        current_path++;
    }

    string_to_unsigned( current_path, 4, &temp, 1 );
    characteristic->end_handle = (uint16_t) ( temp & 0xffff );
    string_to_unsigned( current_path + 4, 4, &temp, 1 );
    characteristic->value_handle = (uint16_t) ( temp & 0xffff );
    string_to_unsigned( current_path + 8, 4, &temp, 1 );
    characteristic->start_handle = (uint16_t) ( temp & 0xffff );
    return WICED_SUCCESS;
}

static wiced_result_t parse_descriptor_handle( const char* url_path, uint16_t* descriptor )
{
    /* /gatt/nodes/<node>/descriptors/<descriptor> */
    char* current_path = (char*) url_path;
    uint32_t a = 0;
    uint32_t temp = 0;

    while ( a < 5 )
    {
        if ( *current_path == '\0' )
        {
            /* <service> not found */
            return WICED_NOT_FOUND;
        }
        else if ( *current_path == '/' )
        {
            a++;
        }

        current_path++;
    }

    string_to_unsigned( current_path, 4, &temp, 1 );
    *descriptor = (uint16_t) ( temp & 0xffff );
    return WICED_SUCCESS;
}

static wiced_result_t create_value_handle( const char* url_path, smart_value_handle_t** value )
{
    /* /gatt/nodes/<node>/characteristics/<characteristic>/value/<value> */
    char* current_path = (char*) url_path;
    uint32_t temp = 0;
    uint32_t a = 0;
    uint32_t b = 0;

    while ( a < 7 )
    {
        if ( *current_path == '\0' )
        {
            /* <service> not found */
            return WICED_NOT_FOUND;
        }
        else if ( *current_path == '/' )
        {
            a++;
        }

        current_path++;
    }

    temp = strlen( current_path ) / 2;
    *value = (smart_value_handle_t*) malloc_named( "value", sizeof(smart_value_handle_t) + temp );
    if ( *value == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    memset( *value, 0, sizeof(smart_value_handle_t) + temp );
    ( *value )->length = temp;
    /* Value is parsed in network byte order */
    for ( a = 0, b = 0; a < ( *value )->length; a++, b += 2 )
    {
        string_to_unsigned( &current_path[ b ], 2, &temp, 1 );
        ( *value )->value[ a ] = (uint8_t) ( temp & 0xff );
    }

    return WICED_SUCCESS;
}

static wiced_result_t delete_value_handle( smart_value_handle_t* value )
{
    wiced_assert( "invalid value handle", ( value != NULL ) );
    free( value );
    return WICED_SUCCESS;
}

static wiced_result_t send_error_response_header( wiced_http_response_stream_t* stream, wiced_result_t result )
{
    wiced_http_response_stream_enable_chunked_transfer( stream );

    switch ( result )
    {
        case WICED_NOT_FOUND:
            wiced_http_response_stream_write_header( stream, HTTP_404_TYPE, CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_JSON );
            break;
        case WICED_BADARG:
            wiced_http_response_stream_write_header( stream, HTTP_400_TYPE, CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_JSON );
            break;
        default:
            return WICED_ERROR;
    }

    wiced_http_response_stream_disable_chunked_transfer( stream );
    wiced_http_response_stream_flush( stream );
    return WICED_SUCCESS;
}

static wiced_result_t parse_128bit_hex_string( const char* input, uint8_t* output )
{
    char* current = (char*) input;
    uint32_t a;

    /* Skip leading "0x" */
    current += 2;

    for ( a = 0; a < 16; a++ )
    {
        uint32_t temp;

        string_to_unsigned( current, 2, &temp, 1 );
        output[ a ] = (uint8_t) ( temp & 0xff );
        current += 2;
    }

    return WICED_SUCCESS;
}

//static wiced_result_t parse_read_multiple_values_message_body( const char* message_body, wiced_bt_gatt_read_param_t* parameter )
//{
//    char* current = (char*) message_body;
//
//    memset( parameter, 0, sizeof( *parameter ) );
//    parameter->read_multiple.auth_req = GATT_AUTH_REQ_NONE;
//    while( ( *current != '\0' ) && ( parameter->read_multiple.num_handles < GATT_MAX_READ_MULTI_HANDLES ) )
//    {
//        if ( strncmp( JSON_NAME_HANDLE, current, sizeof( JSON_NAME_HANDLE ) - 1 ) == 0 )
//        {
//            uint32_t temp = 0;
//            uint32_t a    = 0;
//
//            /* handle is found. Need to consume both ':' and '"' */
//            while ( *current != ':' )
//            {
//                if ( *current == '\0' )
//                {
//                    return WICED_BADARG;
//                }
//                current++;
//            }
//            while ( *current != '"' )
//            {
//                if ( *current == '\0' )
//                {
//                    return WICED_BADARG;
//                }
//                current++;
//            }
//            /* Skip 4 digits. First 4 digits are start handle */
//            while ( a < 4 )
//            {
//                if ( *current == '\0' )
//                {
//                    return WICED_BADARG;
//                }
//                a++;
//                current++;
//            }
//            string_to_unsigned( current, 4, &temp, 1 );
//            parameter->read_multiple.handles[parameter->read_multiple.num_handles++] = (uint16_t)( temp & 0xffff );
//            current += 12;
//        }
//        current++;
//    }
//
//    return WICED_SUCCESS;
//}
