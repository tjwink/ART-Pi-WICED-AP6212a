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
 * Audio Client Utility Routines
 */

#pragma once


#ifdef __cplusplus
extern "C" {
#endif

#include "audio_client_private.h"

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HTTP_HEADER_STR                     "HTTP/"
#define HTTP_HEADER_CONTENT_LENGTH          "Content-Length"
#define HTTP_HEADER_ACCEPT_RANGES           "Accept-Ranges"
#define HTTP_HEADER_CONTENT_TYPE            "Content-Type"
#define HTTP_HEADER_TRANSFER_ENCODING       "Transfer-Encoding"
#define HTTP_HEADER_LOCATION                "Location"
#define HTTP_HEADER_NONE                    "none"
#define HTTP_HEADER_CHUNKED                 "chunked"

#define HTTP_HEADERS_BODY_SEPARATOR         "\r\n\r\n"

#define ICY_HEADER_STR                      "ICY"

#define HTTP_STATUS_OK                      (200)
#define HTTP_STATUS_PARTIAL_CONTENT         (206)
#define HTTP_STATUS_MOVED_PERMANENTLY       (301)
#define HTTP_STATUS_FOUND                   (302)
#define HTTP_STATUS_SEE_OTHER               (303)
#define HTTP_STATUS_TEMPORARY_REDIRECT      (307)
#define HTTP_STATUS_PERMANENT_REDIRECT      (308)

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

wiced_result_t audio_client_http_get_response_code(char* data, uint16_t data_length, uint16_t *code);
audio_client_playlist_t* audio_client_check_for_playlist(char* mime_type);
AUDIO_CLIENT_CODEC_T audio_client_check_uri_for_audio_codec(char* uri);
AUDIO_CLIENT_CODEC_T audio_client_probe_for_audio_codec(audio_client_t* client, char* uri);

#ifdef __cplusplus
} /*extern "C" */
#endif
