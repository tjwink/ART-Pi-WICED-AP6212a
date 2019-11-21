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

#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef wiced_tcp_stream_t   xively_tcp_stream_t;

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    const char*                id;
    const char*                api_key;
    wiced_tcp_socket_t         socket;
    wiced_tls_context_t        tls_context;
} xively_feed_t;

typedef struct
{
    const char*        channel_name;
    wiced_tcp_stream_t tcp_stream;
    uint32_t           data_size;
    uint32_t           number_of_datapoints;
} xively_datastream_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

extern wiced_result_t xively_open_feed         ( xively_feed_t* feed );
extern wiced_result_t xively_close_feed        ( xively_feed_t* feed );
extern wiced_result_t xively_create_datastream ( xively_feed_t* feed, xively_datastream_t* stream, const char* channel_name, uint32_t data_size, uint32_t number_of_datapoints );
extern wiced_result_t xively_write_datapoint   ( xively_datastream_t* stream, const uint8_t* data, const wiced_iso8601_time_t* timestamp );
extern wiced_result_t xively_flush_datastream  ( xively_datastream_t* stream );
extern void           xively_u16toa            ( uint16_t value, char* output, uint8_t min_length );

#ifdef __cplusplus
} /* extern "C" */
#endif
