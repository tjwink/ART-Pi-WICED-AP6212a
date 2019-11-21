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
 * File Audio Client HLS handling header - types -
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

#define AUDIO_CLIENT_HLS_IS_DONE(client) ((client)->hls_playlist_active && (client)->hls_playlist_done)

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_CLIENT_HLS_PLAYLIST_TYPE_EVENT          = 0,
    AUDIO_CLIENT_HLS_PLAYLIST_TYPE_VOD
} AUDIO_CLIENT_HLS_PLAYLIST_TYPE;

typedef enum
{
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_UNKNOWN         =  0,
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_VERSION         = (1 << 0),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_ALLOW_CACHE     = (1 << 1),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_STREAM_INF      = (1 << 2),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_TARGET_DURATION = (1 << 3),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_MEDIA_SEQUENCE  = (1 << 4),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_KEY             = (1 << 5),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_INF             = (1 << 6),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_TYPE            = (1 << 7),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_ENDLIST         = (1 << 8),
    AUDIO_CLIENT_HLS_PLAYLIST_TAG_DISCONTINUITY   = (1 << 9),
} AUDIO_CLIENT_HLS_PLAYLIST_TAG_T;

typedef enum
{
    AUDIO_CLIENT_HLS_NODE_TYPE_UNKNOWN            = 0,
    AUDIO_CLIENT_HLS_NODE_TYPE_PLAYLIST,
    AUDIO_CLIENT_HLS_NODE_TYPE_MEDIA
} AUDIO_CLIENT_HLS_NODE_TYPE_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint32_t                       tags;             /* bitmask of AUDIO_CLIENT_HLS_PLAYLIST_TAG_T    */
    uint32_t                       version;          /* HLS protocol compatibility version            */
    wiced_bool_t                   allow_cache;      /* allow playlist caching                        */
    uint32_t                       bandwidth;        /* max bandwidth in bits per second              */
    uint32_t                       target_duration;  /* duration of longest media segment in seconds  */
    uint32_t                       media_sequence;   /* media segment sequence number                 */
    wiced_bool_t                   encrypted;        /* encryption flag                               */
    double                         segment_duration; /* duration of specific media segment in seconds */
    AUDIO_CLIENT_HLS_PLAYLIST_TYPE type;             /* playlist type                                 */
    uint32_t                       entry_number;     /* segment or master playlist number             */
} audio_client_hls_playlist_info_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /*extern "C" */
#endif
