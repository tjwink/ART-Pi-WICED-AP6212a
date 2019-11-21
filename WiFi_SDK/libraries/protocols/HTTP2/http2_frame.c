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
#include "http2.h"
#include "http2_pvt.h"

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static const uint8_t preface[] = { 0x50,0x52,0x49,0x20,0x2a,0x20,0x48,0x54,0x54,0x50
        ,0x2f,0x32,0x2e,0x30,0x0d,0x0a,0x0d,0x0a,0x53,0x4d
        ,0x0d,0x0a,0x0d,0x0a
       }; /* PRI * HTTP/2.0\r\n\r\nSM\r\n\r\n */


/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t http_frame_get_settings( http2_frame_t* frame, http_connection_settings_t* settings, uint8_t* flags )
{
    uint32_t count = 0;
    uint32_t size  = READ_BE24( frame->length );
    *flags = frame->flags;
    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    while ( count < size )
    {
        http2_setting_frame_t* setting = (http2_setting_frame_t*) ( frame->data + count );
        switch ( htobe16( setting->id ) )
        {
            case HTTP2_SETTINGS_HEADER_TABLE_SIZE_ID:
                HTTP_DEBUG(( "Setting SETTINGS_HEADER_TABLE_SIZE: %u\n", (unsigned int)htobe32( setting->value )));
                settings->header_table_size = htobe32( setting->value );
                break;
            case HTTP2_SETTINGS_ENABLE_PUSH_ID:
                HTTP_DEBUG(( "Setting SETTINGS_ENABLE_PUSH_ID: %u\n", (unsigned int)htobe32( setting->value )));
                settings->enable_push = htobe32( setting->value );
                break;
            case HTTP2_SETTINGS_MAX_CONCURRENT_STREAMS_ID:
                HTTP_DEBUG(( "Setting SETTINGS_MAX_CONCURRENT_STREAMS: %u\n", (unsigned int)htobe32( setting->value )));
                settings->max_concurrent_streams = htobe32( setting->value );
                break;
            case HTTP2_SETTINGS_INITIAL_WINDOW_SIZE_ID:
                HTTP_DEBUG(( "Setting SETTINGS_INITIAL_WINDOW_SIZE: %u\n", (unsigned int)htobe32( setting->value )));
                settings->initial_window_size = htobe32( setting->value );
                break;
            case HTTP2_SETTINGS_MAX_FRAME_SIZE_ID:
                HTTP_DEBUG(( "Setting SETTINGS_MAX_FRAME_SIZE: %u\n", (unsigned int)htobe32( setting->value )));
                settings->max_frame_size = htobe32( setting->value );
                break;
            case HTTP2_SETTINGS_MAX_HEADER_LIST_SIZE_ID:
                HTTP_DEBUG(( "Setting SETTINGS_MAX_HEADER_LIST_SIZE: %u\n", (unsigned int)htobe32( setting->value )));
                settings->max_header_list_size = htobe32( setting->value );
                break;
            default:
                HTTP_DEBUG(( "Unidentified setting: %u\n", (unsigned int)htobe32( setting->id )));
                break;
        }
        count += sizeof( http2_setting_frame_t );
    }
    return WICED_SUCCESS;
}

wiced_result_t http_frame_put_settings( http2_frame_t* frame, http_connection_settings_t* settings, uint8_t ack )
{
    uint32_t count = 0;
    http2_setting_frame_t* setting_frame = (http2_setting_frame_t*) frame->data;

    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    frame->type      = HTTP2_FRAME_TYPE_SETTINGS;
    frame->flags     = ack;
    frame->stream_id = 0;

    if ( settings->header_table_size != HTTP2_SETTINGS_HEADER_TABLE_SIZE_DEFAULT )
    {
        setting_frame->id = htobe16( HTTP2_SETTINGS_HEADER_TABLE_SIZE_ID );
        setting_frame->value = htobe32( settings->header_table_size );
        count += sizeof(*setting_frame );
        setting_frame += 1;
    }
    if ( settings->enable_push != HTTP2_SETTINGS_ENABLE_PUSH_DEFAULT )
    {
        setting_frame->id = htobe16( HTTP2_SETTINGS_ENABLE_PUSH_ID );
        setting_frame->value = htobe32( settings->enable_push );
        count += sizeof(*setting_frame );
        setting_frame += 1;
    }
    if ( settings->max_concurrent_streams != HTTP2_SETTINGS_MAX_CONCURRENT_STREAMS_DEFAULT )
    {
        setting_frame->id = htobe16( HTTP2_SETTINGS_MAX_CONCURRENT_STREAMS_ID );
        setting_frame->value = htobe32( settings->max_concurrent_streams );
        count += sizeof(*setting_frame );
        setting_frame += 1;
    }
    if ( settings->initial_window_size != HTTP2_SETTINGS_INITIAL_WINDOW_SIZE_DEFAULT )
    {
        setting_frame->id = htobe16( HTTP2_SETTINGS_INITIAL_WINDOW_SIZE_ID );
        setting_frame->value = htobe32( settings->initial_window_size );
        count += sizeof(*setting_frame );
        setting_frame += 1;
    }
    if ( settings->max_frame_size != HTTP2_SETTINGS_MAX_FRAME_SIZE_DEFAULT )
    {
        setting_frame->id = htobe16( HTTP2_SETTINGS_MAX_FRAME_SIZE_ID );
        setting_frame->value = htobe32( settings->max_frame_size );
        count += sizeof(*setting_frame );
        setting_frame += 1;
    }
    if ( settings->max_header_list_size != HTTP2_SETTINGS_MAX_HEADER_LIST_SIZE_DEFAULT )
    {
        setting_frame->id = htobe16( HTTP2_SETTINGS_MAX_HEADER_LIST_SIZE_ID );
        setting_frame->value = htobe32( settings->max_header_list_size );
        count += sizeof(*setting_frame );
        setting_frame += 1;
    }
    WRITE_BE24( frame->length, count );
    return WICED_SUCCESS;
}

wiced_result_t http_frame_put_ack( http2_frame_t* frame, uint8_t ack )
{
    uint32_t count = 0;

    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    frame->type      = HTTP2_FRAME_TYPE_SETTINGS;
    frame->flags     = ack;
    frame->stream_id = 0;
    WRITE_BE24( frame->length, count );
    return WICED_SUCCESS;
}

wiced_result_t http_frame_put_window_update( http2_frame_t* frame, uint32_t stream_id, uint32_t window_inc_size )
{
    uint32_t*  window_size_p = (uint32_t *)frame->data;
    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    frame->type      = HTTP2_FRAME_TYPE_WINDOW_UPDATE;
    frame->flags     = 0;
    frame->stream_id = htobe32( stream_id );
    *window_size_p   = htobe32( window_inc_size );
    WRITE_BE24( frame->length, 4 );
    return WICED_SUCCESS;
}

wiced_result_t http_frame_put_preface( uint8_t* data )
{
    memcpy( data, preface, sizeof(preface) );
    return WICED_SUCCESS;
}

wiced_result_t http_frame_put_headers( http2_frame_t* frame, http_header_info_t* headers, uint8_t flags, uint32_t stream_id, uint32_t size, uint8_t type )
{
    wiced_result_t  result;
    uint8_t*    data = frame->data;
    uint32_t    initial_size = size;

    /* Initialize */
    WRITE_BE24( frame->length, 0 );

    /* Update Header fields */
    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    frame->type      = type;
    frame->flags     = flags;
    frame->stream_id = htobe32( stream_id );
    result = http_hpack_encode_headers( headers, &data, &size );
    if ( result == WICED_SUCCESS )
    {
        /* Update the size post compression */
        size = initial_size - size;
        WRITE_BE24( frame->length, size );
    }
    return result;
}

wiced_result_t http_frame_put_data( http2_frame_t* frame, uint8_t* data, uint32_t size, uint8_t flags, uint32_t stream_id )
{
    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    frame->type      = HTTP2_FRAME_TYPE_DATA;
    frame->flags     = flags;
    frame->stream_id = htobe32( stream_id );
    memcpy( frame->data, data, size );
    WRITE_BE24( frame->length, size );
    return WICED_SUCCESS;
}

wiced_result_t http_frame_put_goaway( http2_frame_t* frame, uint32_t last_stream_id, http2_error_codes_t error )
{
    http2_goaway_frame_t*   goaway;
    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    frame->type      = HTTP2_FRAME_TYPE_GOAWAY;
    frame->flags     = 0;
    frame->stream_id = 0;
    goaway = ( http2_goaway_frame_t*)frame->data;
    goaway->last_stream_id = htobe32( last_stream_id );
    goaway->error_code     = htobe32( error );
    WRITE_BE24( frame->length, 8 );
    return WICED_SUCCESS;
}

wiced_result_t http_frame_put_ping( http2_frame_t* frame, uint8_t data[HTTP2_PING_FRAME_DATA_LENGTH], uint8_t flags )
{
    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    frame->type      = HTTP2_FRAME_TYPE_PING;
    frame->flags     = flags;
    frame->stream_id = 0;
    memcpy( frame->data, data, HTTP2_PING_FRAME_DATA_LENGTH );
    WRITE_BE24( frame->length, HTTP2_PING_FRAME_DATA_LENGTH );
    return WICED_SUCCESS;
}

wiced_result_t http_frame_get_headers( http2_frame_t* frame, http_header_info_t* head, uint8_t* header_buffer, uint32_t header_size )
{
    wiced_result_t  result;
    uint8_t*    data = frame->data;
    uint32_t    size = READ_BE24(frame->length);

    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));
    result = http_hpack_decode_headers( head, &header_buffer, &header_size, &data, &size );

    return result;
}

wiced_result_t http_frame_put_priority( http2_frame_t* frame, uint32_t stream_id, uint32_t dependency_stream, uint8_t weight )
{
    http2_priority_frame_t* priority;
    HTTP_DEBUG(( "[%s()] : [%d] : \n", __FUNCTION__,__LINE__ ));

    frame->type      = HTTP2_FRAME_TYPE_PRIORITY;
    frame->flags     = 0;
    frame->stream_id = htobe32( stream_id );
    priority         = ( http2_priority_frame_t*)frame->data;
    priority->dependency_stream = htobe32( dependency_stream );
    priority->weight = weight;
    WRITE_BE24( frame->length, 5 );
    return WICED_SUCCESS;
}


