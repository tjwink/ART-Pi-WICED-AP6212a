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

#include "http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define RESTFUL_SMART_URIS \
    { "/gatt/nodes/*/services",                      "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { discover_services,                     0 }, }, \
    { "/gatt/nodes/*/services/*/characteristics",    "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { discover_characteristics_of_a_service, 0 }, }, \
    { "/gatt/nodes/*/characteristics",               "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { discover_characteristics_by_uuid,      0 }, }, \
    { "/gatt/nodes/*/characteristics/*/descriptors", "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { discover_descriptors,                  0 }, }, \
    { "/gatt/nodes/*/characteristics/*/value",       "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { read_characteristic_value,             0 }, }, \
    { "/gatt/nodes/*/characteristics/value",         "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { read_write_characteristic_values,      0 }, }, \
    { "/gatt/nodes/*/characteristics/*/value/*",     "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { write_characteristic_value,            0 }, }, \
    { "/gatt/nodes/*/characteristics/*",             "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { manage_characteristic,                 0 }, }, \
    { "/gatt/nodes/*/descriptors/*/value",           "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { read_descriptor_value,                 0 }, }, \
    { "/gatt/nodes/*/descriptors/*/value/*",         "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { write_descriptor_value,                0 }, }, \
    { "/management/nodes/*",                         "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { manage_node_security,                  0 }, }, \
    { "/management/nodes",                           "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { list_bonded_nodes,                     0 }, }, \
    { "/gap/nodes/*",                                "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { manage_node_connection,                0 }, }, \
    { "/gap/nodes",                                  "application/json",  WICED_RAW_DYNAMIC_URL_CONTENT, .url_content.dynamic_data = { discover_nodes,                        0 }, },

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

int32_t discover_services                    ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t discover_characteristics_of_a_service( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t discover_characteristics_by_uuid     ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t manage_characteristic                ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t read_characteristic_value            ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t read_write_characteristic_values     ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t write_characteristic_value           ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t discover_descriptors                 ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t read_descriptor_value                ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t write_descriptor_value               ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t discover_nodes                       ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t manage_node_connection               ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t list_bonded_nodes                    ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );
int32_t manage_node_security                 ( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body );

#ifdef __cplusplus
} /* extern "C" */
#endif
