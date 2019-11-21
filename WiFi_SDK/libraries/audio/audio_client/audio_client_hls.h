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

/**
 * @file
 *
 * File Audio Client HLS handling header
 */

#pragma once

#include "audio_client_private.h"
#include "audio_client_hls_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t audio_client_hls_init(audio_client_t* client);
wiced_result_t audio_client_hls_deinit(audio_client_t* client);
wiced_result_t audio_client_hls_stop(audio_client_t* client);
wiced_result_t audio_client_hls_process_line_buffer(audio_client_t* client, audio_client_http_params_t* params, uint8_t* data, uint32_t data_length);
wiced_result_t audio_client_hls_process_playlist(audio_client_t* client, audio_client_http_params_t* params);
wiced_result_t audio_client_hls_get_next_playlist_entry(audio_client_t* client, audio_client_http_params_t* params);
wiced_result_t audio_client_hls_process_reload(audio_client_t* client, audio_client_http_params_t* params);
void           audio_client_hls_flush_list(audio_client_t* client);
wiced_bool_t   audio_client_hls_is_playlist_active(audio_client_t* client);
wiced_result_t audio_client_hls_suspend(audio_client_t* client, audio_client_suspend_t** suspend);
wiced_result_t audio_client_hls_realloc_stream_uri(audio_client_t* client, audio_client_http_params_t* params, char* new_uri, uint32_t new_uri_length);

#ifdef __cplusplus
} /*extern "C" */
#endif
