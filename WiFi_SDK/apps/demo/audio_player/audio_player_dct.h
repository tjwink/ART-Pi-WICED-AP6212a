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

#ifdef __cplusplus
extern "C" {
#endif

#include "platform_audio.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define AUDIO_PLAYER_VOLUME_MIN               0
#define AUDIO_PLAYER_VOLUME_DEFAULT          70
#define AUDIO_PLAYER_VOLUME_MAX             100

#define AUDIO_PLAYER_NUM_HTTP_BUFFERS       (200)
#define AUDIO_PLAYER_NUM_AUDIO_BUFFERS      (80)
#define AUDIO_PLAYER_SIZE_AUDIO_BUFFERS     (2048)
#define AUDIO_PLAYER_AUDIO_PERIOD_SIZE      (0)

#define AUDIO_PLAYER_HTTP_THRESHOLD_HIGH    (AUDIO_PLAYER_NUM_HTTP_BUFFERS - 5)
#define AUDIO_PLAYER_HTTP_THRESHOLD_LOW     (10)
#define AUDIO_PLAYER_HTTP_PREROLL           (AUDIO_PLAYER_HTTP_THRESHOLD_HIGH)
#define AUDIO_PLAYER_HTTP_READ_INHIBIT      (0)
#define AUDIO_PLAYER_HTTP_DISABLE_PREROLL   (1)

#define AUDIO_PLAYER_DEFAULT_APP_PLAYBACK   (0)

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

typedef struct
{
    int volume;
    platform_audio_device_id_t audio_device_tx;
    int app_playback;
    int http_buffer_num;
    int http_buffer_preroll;
    int audio_buffer_num;
    int audio_buffer_size;
    int audio_period_size;
    int http_threshold_high;
    int http_threshold_low;
    int http_read_inhibit;
    int disable_preroll;            /* Disable preroll for live streaming (content length == 0) */
} audio_player_dct_t;


#ifdef __cplusplus
} /* extern "C" */
#endif
