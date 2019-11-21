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

#include "http2.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
//#define ENABLE_DEBUG
#ifdef ENABLE_DEBUG
#define HTTP_DEBUG( X )                 printf("  ======= HTTP/2 - DEBUG : "); printf X
#else
#define HTTP_DEBUG( X )
#endif

//#define ENABLE_HEX_DUMP
#ifdef ENABLE_HEX_DUMP
#define HTTP_DEBUG_WO_FORMATTING( X )   printf X
#else
#define HTTP_DEBUG_WO_FORMATTING( X )
#endif


#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

#define  READ_BE24( data )  ( (uint32_t)( data[2] ) + (uint32_t)( data[1] << 8 ) + (uint32_t)( data[0] << 16 ) )

#define  WRITE_BE24( data, value )      \
do                                      \
{   data[0] = (uint8_t)( value >> 16 ); \
    data[1] = (uint8_t)( value >> 8  ); \
    data[2] = (uint8_t)( value );       \
}while(0)

/******************************************************
 *            Enumerations
 ******************************************************/
/* Defined as per RFC: https://tools.ietf.org/html/rfc7540#page-50 */
typedef enum http2_error_codes_s
{
    HTTP2_ERROR_CODE_NO_ERROR            = 0x0,
    HTTP2_ERROR_CODE_PROTOCOL_ERROR      = 0x1,
    HTTP2_ERROR_CODE_INTERNAL_ERROR      = 0x2,
    HTTP2_ERROR_CODE_FLOW_CONTROL_ERROR  = 0x3,
    HTTP2_ERROR_CODE_SETTINGS_TIMEOUT    = 0x4,
    HTTP2_ERROR_CODE_STREAM_CLOSED       = 0x5,
    HTTP2_ERROR_CODE_FRAME_SIZE_ERROR    = 0x6,
    HTTP2_ERROR_CODE_REFUSED_STREAM      = 0x7,
    HTTP2_ERROR_CODE_CANCEL              = 0x8,
    HTTP2_ERROR_CODE_COMPRESSION_ERROR   = 0x9,
    HTTP2_ERROR_CODE_CONNECT_ERROR       = 0xA,
    HTTP2_ERROR_CODE_ENHANCE_YOUR_CALM   = 0xB,
    HTTP2_ERROR_CODE_INADEQUATE_SECURITY = 0xC,
    HTTP2_ERROR_CODE_HTTP_1_1_REQUIRED   = 0xD,
} http2_error_codes_t;
/******************************************************
 *                    Constants
 ******************************************************/
#define HTTP_CLIENT_CONNECTION_TIMEOUT            (5000)          /* TCP connection timeout */
#define HTTP_CLIENT_CONNECTION_NUMBER_OF_RETRIES  (3)             /* TCP connection retries */

#define HTTP2_FRAME_TYPE_DATA                     (0x0)
#define HTTP2_FRAME_TYPE_HEADERS                  (0x1)
#define HTTP2_FRAME_TYPE_PRIORITY                 (0x2)
#define HTTP2_FRAME_TYPE_RST_STREAM               (0x3)
#define HTTP2_FRAME_TYPE_SETTINGS                 (0x4)
#define HTTP2_FRAME_TYPE_PUSH_PROMISE             (0x5)
#define HTTP2_FRAME_TYPE_PING                     (0x6)
#define HTTP2_FRAME_TYPE_GOAWAY                   (0x7)
#define HTTP2_FRAME_TYPE_WINDOW_UPDATE            (0x8)
#define HTTP2_FRAME_TYPE_CONTINUATION             (0x9)

#define HTTP2_SETTINGS_HEADER_TABLE_SIZE_ID       (0x1)
#define HTTP2_SETTINGS_ENABLE_PUSH_ID             (0x2)
#define HTTP2_SETTINGS_MAX_CONCURRENT_STREAMS_ID  (0x3)
#define HTTP2_SETTINGS_INITIAL_WINDOW_SIZE_ID     (0x4)
#define HTTP2_SETTINGS_MAX_FRAME_SIZE_ID          (0x5)
#define HTTP2_SETTINGS_MAX_HEADER_LIST_SIZE_ID    (0x6)

#define HTTP2_SETTINGS_HEADER_TABLE_SIZE_DEFAULT       (4096)
#define HTTP2_SETTINGS_ENABLE_PUSH_DEFAULT             (1)
#define HTTP2_SETTINGS_MAX_CONCURRENT_STREAMS_DEFAULT  (0xFFFFFFFF)
#define HTTP2_SETTINGS_INITIAL_WINDOW_SIZE_DEFAULT     (65535)
#define HTTP2_SETTINGS_MAX_FRAME_SIZE_DEFAULT          (16384)
#define HTTP2_SETTINGS_MAX_HEADER_LIST_SIZE_DEFAULT    (0xFFFFFFFF)

#define HTTP2_CLIENT_INITIAL_STREAM_ID              (1)
#define HTTP2_SERVER_INITIAL_STREAM_ID              (0)

#define HTTP2_HUFFMAN_EOS                           (0x3FFFFFFF)

#define HTTP2_PREFACE_LEN                           (24)
#define HTTP2_FRAME_FIXED_HEADER_LEN                (sizeof(http2_frame_t) - 1)
#define HTTP2_SETTINGS_FRAME_PARAMETER_LEN          (6)
#define HTTP2_MAX_SETTINGS_FRAME_PARAMETERS         (6)
#define HTTP2_SETTINGS_FRAME_MAX_SIZE               (HTTP2_MAX_SETTINGS_FRAME_PARAMETERS * HTTP2_SETTINGS_FRAME_PARAMETER_LEN)
#define HTTP2_GOAWAY_FRAME_SIZE                     (8)             /* Stream ID and Error code */
#define HTTP2_DEFAULT_STREAM_PRIORITY_WEIGHT        (255)
/******************************************************
 *                 Type Definitions
 ******************************************************/

#pragma pack(1)
typedef struct http2_frame_s
{
    uint8_t     length[3];      /* Frame payload size. Excluding the fixed header size */
    uint8_t     type;           /* Frame type */
    uint8_t     flags;          /* Flags specific to the frame */
    uint32_t    stream_id;      /* Stream ID of the frame */
    uint8_t     data[1];        /* Frame payload */
} http2_frame_t;

typedef struct http2_settings_frame_s
{
    uint16_t    id;
    uint32_t    value;
}http2_setting_frame_t;

typedef struct http2_priority_frame_s
{
    uint32_t    dependency_stream;
    uint8_t     weight;
}http2_priority_frame_t;

typedef struct http2_header_frame_s
{
    uint8_t     pad_len;
    uint32_t    dependency_stream;
    uint8_t     weight;
}http2_header_frame_t;

typedef struct http2_goaway_frame_s
{
    uint32_t    last_stream_id;
    uint32_t    error_code;
    uint8_t*    info;
} http2_goaway_frame_t;

#pragma pack()

typedef struct http_packet_s
{
    wiced_packet_t* packet;
    uint8_t*        data;
} http_packet_t;

/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *             Function declarations
 ******************************************************/
wiced_result_t http2_network_init           ( http_connection_t* connect );
wiced_result_t http2_network_deinit         ( http_connection_t* connect );
wiced_result_t http_network_tls_init        ( http_socket_t *socket, const http_security_info_t *security, http_client_config_info_t* config );
wiced_result_t http_network_tls_deinit      ( http_socket_t *socket );
wiced_result_t http_network_connect         ( http_socket_t *socket, const wiced_ip_address_t *server_ip_address, uint16_t portnumber, wiced_interface_t interface );
wiced_result_t http_network_disconnect      ( http_socket_t *socket );
wiced_result_t http_network_create_packet   ( http_socket_t* socket, http_packet_t* packet, uint16_t size, uint16_t *size_available );
wiced_result_t http_network_delete_packet   ( http_packet_t* packet );
wiced_result_t http_network_send_packet     ( http_socket_t* socket,  http_packet_t* packet );

wiced_result_t http_receive_callback        ( uint8_t* data, uint32_t* size, void* arg );
wiced_result_t http_disconnect_callback     ( void* arg );

wiced_result_t http_frame_get_settings  ( http2_frame_t* frame, http_connection_settings_t* settings, uint8_t* flags );
wiced_result_t http_frame_put_settings  ( http2_frame_t* frame, http_connection_settings_t* settings, uint8_t  flags );
wiced_result_t http_frame_put_ack       ( http2_frame_t* frame, uint8_t ack );
wiced_result_t http_frame_put_preface   ( uint8_t* data );
wiced_result_t http_frame_put_headers   ( http2_frame_t* frame, http_header_info_t* headers, uint8_t flags, uint32_t stream_id, uint32_t size, uint8_t type );
wiced_result_t http_frame_get_headers   ( http2_frame_t* frame, http_header_info_t* head, uint8_t* header_buffer, uint32_t header_size );
wiced_result_t http_frame_put_priority  ( http2_frame_t* frame, uint32_t stream_id, uint32_t dependency_stream, uint8_t weight );
wiced_result_t http_frame_put_goaway    ( http2_frame_t* frame, uint32_t last_stream_id, http2_error_codes_t error );
wiced_result_t http_frame_put_data      ( http2_frame_t* frame, uint8_t* data, uint32_t size, uint8_t flags, uint32_t stream_id );
wiced_result_t http_frame_put_ping      ( http2_frame_t* frame, uint8_t data[HTTP2_PING_FRAME_DATA_LENGTH], uint8_t flags );
wiced_result_t http_frame_put_window_update( http2_frame_t* frame, uint32_t stream_id, uint32_t window_inc_size );

wiced_result_t http_hpack_put_integer   ( uint32_t value, uint8_t prefix, uint8_t** buffer, uint32_t* size );
wiced_result_t http_hpack_get_integer   ( uint32_t* value, uint8_t prefix, uint8_t** buffer, uint32_t* size );
wiced_result_t http_hpack_decode_headers( http_header_info_t* header, uint8_t** header_buffer, uint32_t* header_buffer_size, uint8_t** buffer, uint32_t* size );
wiced_result_t http_hpack_encode_headers( http_header_info_t* header, uint8_t** buffer, uint32_t* size );

#ifdef __cplusplus
} /* extern "C" */
#endif
