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

#include "restful_smart_ble.h"
#include "wiced_bt_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* Include NUL-terminator */
#define NODE_BUFFER_LENGTH           (14)
#define SERVICE_BUFFER_LENGTH        (9)
#define CHARACTERISTIC_BUFFER_LENGTH (13)
#define DESCRIPTOR_BUFFER_LENGTH     (5)
#define UUID_BUFFER_LENGTH           (33)
#define BD_ADDR_BUFFER_LENGTH        (13)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    REST_SMART_STATUS_200, /* OK */
    REST_SMART_STATUS_400, /* Bad Request */
    REST_SMART_STATUS_403,
    REST_SMART_STATUS_404, /* URI Not Found */
    REST_SMART_STATUS_405,
    REST_SMART_STATUS_406,
    REST_SMART_STATUS_412,
    REST_SMART_STATUS_415,
    REST_SMART_STATUS_504,
} rest_smart_status_code_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    rest_smart_status_code_t   status_code;
    rest_smart_pairing_status_t pairing_status_code;
    uint8_t                    pairing_id[16];
    uint32_t                   passkey;
    wiced_bt_device_address_t  bdaddra;
    uint8_t                    reason_code;
    uint8_t                    ra[16];
    uint8_t                    ca[16];
} rest_smart_pairing_response_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* Send response API includes header, data, and final chunk packet, followed by stream flush */
wiced_result_t rest_smart_response_end_stream                             ( wiced_http_response_stream_t* stream );
wiced_result_t rest_smart_response_send_node_data                         ( wiced_http_response_stream_t* stream, const smart_node_handle_t* node );
wiced_result_t rest_smart_response_send_pairing_data                      ( wiced_http_response_stream_t* stream, rest_smart_pairing_response_t* response );
wiced_result_t rest_smart_response_send_notification                      ( wiced_http_response_stream_t* stream, wiced_bt_gatt_data_t* data );
wiced_result_t rest_smart_response_send_error_message                     ( wiced_http_response_stream_t* stream, rest_smart_status_code_t status );

/* Write response API only writes the data to the stream. The stream will handle the sending of the data */
wiced_result_t rest_smart_response_write_status_code                      ( wiced_http_response_stream_t* stream, rest_smart_status_code_t status );
wiced_result_t rest_smart_response_write_node_array_start                 ( wiced_http_response_stream_t* stream );
wiced_result_t rest_smart_response_write_service_array_start              ( wiced_http_response_stream_t* stream );
wiced_result_t rest_smart_response_write_characteristic_array_start       ( wiced_http_response_stream_t* stream );
wiced_result_t rest_smart_response_write_adv_array_start                  ( wiced_http_response_stream_t* stream );
wiced_result_t rest_smart_response_write_descriptor_array_start           ( wiced_http_response_stream_t* stream );
wiced_result_t rest_smart_response_write_array_end                        ( wiced_http_response_stream_t* stream );
wiced_result_t rest_smart_response_write_gap_node                         ( wiced_http_response_stream_t* stream, const char* node_handle, const char* bdaddr, const char* bdaddrtype, const char* rssi, wiced_bool_t is_first_entry );
wiced_result_t rest_smart_response_write_adv_data                         ( wiced_http_response_stream_t* stream, const char* adv_type, const char* adv_data, wiced_bool_t is_first_entry );
wiced_result_t rest_smart_response_write_service                          ( wiced_http_response_stream_t* stream, const char* node, const char* service, uint16_t handle, const char* uuid, wiced_bool_t is_primary_service, wiced_bool_t is_first_entry );
wiced_result_t rest_smart_response_write_characteristic                   ( wiced_http_response_stream_t* stream, const char* node, const char* characteristic, uint16_t handle, const char* uuid, uint8_t properties, wiced_bool_t is_first_entry );
wiced_result_t rest_smart_response_write_descriptor                       ( wiced_http_response_stream_t* stream, const char* node, const char* descriptor, uint16_t handle, const char* uuid, wiced_bool_t is_first_entry );
wiced_result_t rest_smart_response_write_node                             ( wiced_http_response_stream_t* stream, const char* node, const char* bdaddr );
wiced_result_t rest_smart_response_write_characteristic_value             ( wiced_http_response_stream_t* stream, const char* node, const char* characteristic, uint16_t handle, const uint8_t* value, uint32_t value_length );
wiced_result_t rest_smart_response_write_descriptor_value                 ( wiced_http_response_stream_t* stream, const char* node, const char* descriptor_value_handle, const uint8_t* value, uint32_t value_length );
wiced_result_t rest_smart_response_write_cached_value                     ( wiced_http_response_stream_t* stream, const char* value_handle, const char* value );
wiced_result_t rest_smart_response_write_long_characteristic_value_start  ( wiced_http_response_stream_t* stream, const char* node_handle, const char* characteristic_value_handle );
wiced_result_t rest_smart_response_write_long_partial_characteristic_value( wiced_http_response_stream_t* stream, const uint8_t* partial_value, uint32_t length );
wiced_result_t rest_smart_response_write_long_characteristic_value_end    ( wiced_http_response_stream_t* stream );

int            ip_to_str                                                  ( const wiced_ip_address_t* address, char* string );
void           string_to_device_address                                   ( const char* string, wiced_bt_device_address_t* device_address );
void           device_address_to_string                                   ( const wiced_bt_device_address_t* device_address, char* string );
void           string_to_uuid                                             ( const char* string, wiced_bt_uuid_t* uuid );
void           uuid_to_string                                             ( const wiced_bt_uuid_t* uuid, char* string );
void           format_node_string                                         ( char* output, const wiced_bt_device_address_t* address, wiced_bt_ble_address_type_t type );
void           format_service_string                                      ( char* output, uint16_t start_handle, uint16_t end_handle );
void           format_characteristic_string                               ( char* output, uint16_t start_handle, uint16_t end_handle, uint16_t value_handle );

#ifdef __cplusplus
} /* extern "C" */
#endif
