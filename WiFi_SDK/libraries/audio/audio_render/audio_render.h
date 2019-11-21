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
 * Audio render - Audio playback library.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced_result.h"
#include "wiced_audio.h"

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define AUDIO_RENDER_VOLUME_MIN     (0)
#define AUDIO_RENDER_VOLUME_MAX     (100)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_RENDER_CMD_STOP = 0,              /* Arg is NULL                                                                             */
    AUDIO_RENDER_CMD_FLUSH,                 /* Arg is NULL                                                                             */
    AUDIO_RENDER_CMD_PAUSE,                 /* Arg is NULL                                                                             */
    AUDIO_RENDER_CMD_RESUME,                /* Arg is NULL                                                                             */
    AUDIO_RENDER_CMD_SET_VOLUME,            /* Arg is uint32_t volume value                                                            */
    AUDIO_RENDER_CMD_SET_SPEED,             /* Arg is pointer to float speed adjustment in PPM                                         */
    AUDIO_RENDER_CMD_GET_QUEUE_WEIGHT,      /* Arg is pointer to audio_render_queue_weight_t                                           */
    AUDIO_RENDER_CMD_GET_STATS,             /* Arg is pointer to audio_render_stats_t                                                  */
    AUDIO_RENDER_CMD_SET_EFFECT,            /* Arg is uint32_t effect mode                                                             */
    AUDIO_RENDER_CMD_SET_BUFFER_MS,         /* Arg is uint32_t buffering/pre-roll time                                                 */
    AUDIO_RENDER_CMD_SET_THRESHOLD_NS,      /* Arg is pointer to int64_t nanosecond threshold for adding silence/dropping audio frames */
    AUDIO_RENDER_CMD_MAX
} AUDIO_RENDER_CMD_T;

typedef enum
{
    AUDIO_RENDER_EVENT_EOS = 0,             /* Arg is NULL                                      */
    AUDIO_RENDER_EVENT_CONFIG_ERROR,        /* Arg is NULL - Audio configuration failed         */

    AUDIO_RENDER_EVENT_MAX
} AUDIO_RENDER_EVENT_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct audio_render_s *audio_render_ref;

/**
 * Audio buffer structure.
 */

typedef struct
{
    int64_t  pts;
    uint8_t* data_buf;
    uint32_t data_offset;
    uint32_t data_length;
    void*    opaque;
} audio_render_buf_t;

/**
 * Callback for releasing audio buffers.
 * Note that the internal audio render buffer mutex is held during this callback. Meaning
 * that any push calls will be block until the release callback returns.
 */

typedef wiced_result_t (*audio_render_buf_release_cb_t)(audio_render_buf_t* buf, void* userdata);

/**
 * Callback for providing reference time in nanoseconds
 */

typedef wiced_result_t (*audio_render_time_get_cb_t)(int64_t *time_in_nanosecs, void* userdata);

/**
 * Callback for receiving audio render event notifications.
 */

typedef wiced_result_t (*audio_render_event_cb_t)(audio_render_ref handle, void* userdata, AUDIO_RENDER_EVENT_T event, void* arg);

/**
 * Audio queue weight
 */

typedef struct
{
    uint32_t input_queue_weight;
    uint32_t output_buffer_weight;
    uint32_t output_buffer_size;
} audio_render_queue_weight_t;

/**
 * Audio playback statistics
 */

typedef struct
{
    uint64_t     audio_frames_played;
    uint64_t     audio_frames_dropped;
    uint64_t     audio_frames_inserted;
    int64_t      audio_max_early;                           /* Maximum amount of time before threshold window                           */
    int64_t      audio_max_late;                            /* Maximum amount of time after threshold window                            */
    uint64_t     audio_underruns;
} audio_render_stats_t;

/**
 * Configuration parameters for starting audio render.
 * Note that the timing parameters (buffer_ms, threshold_ms) are only used
 * when operating with the clock enabled.
 */

typedef struct
{
    uint16_t                        buffer_nodes;           /* Number of buffer nodes for audio render to allocate                      */
    uint16_t                        period_size;            /* When non-zero, override the default audio period buffer size             */
    uint32_t                        buffer_ms;              /* Buffering (pre-roll) time that audio render should use                   */
    uint32_t                        threshold_ms;           /* Threshold in ms for adding silence/dropping audio frames                 */
    int                             clock_enable;           /* 0 = disable (blind push), 1 = enable                                     */
    platform_audio_device_id_t      device_id;              /* device identifier                                                        */
    wiced_bool_t                    release_device_on_stop; /* release audio playback device after a stop command or after an EOS event */
    void*                           userdata;               /* Application pointer passed back in the following callbacks               */
    audio_render_buf_release_cb_t   buf_release_cb;         /* Audio buffer release callback                                            */
    audio_render_time_get_cb_t      time_get_cb;            /* OPTIONAL - reference nanosecond time callback                            */
    audio_render_event_cb_t         event_cb;               /* Audio event callback                                                     */
} audio_render_params_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/
/******************************************************
 *               Function Declarations
 ******************************************************/
/*****************************************************************************/
/**
 *
 *  @defgroup multimedia     WICED Multimedia
 *  @ingroup  multimedia
 *
 *  @addtogroup audio_render  WICED Audio Rendering API
 *  @ingroup multimedia
 *
 * This library implements audio playback using the audio_render API
 *
 *  @{
 */
/*****************************************************************************/
/******************************************************
 *               Function Declarations
 ******************************************************/

/** Initialize the audio render library.
 *
 * @param[in] params : Pointer to the audio configuration parameters.
 *
 * @return Pointer to the audio render instance or NULL
 */
audio_render_ref audio_render_init(audio_render_params_t* params);

/** Deinitialize the audio render library.
 *
 * @param[in] handle : Pointer to the audio render instance.
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_render_deinit(audio_render_ref handle);

/** Configure the audio render audio format.
 *
 * @param[in] handle : Pointer to the audio render instance.
 * @param[in] config : Pointer to the audio configuration.
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_render_configure(audio_render_ref handle, wiced_audio_config_t* config);

/** Push an audio buffer to the audio render library.
 * @note The audio render becomes owner of the buffer pointed to
 * by the data_buf member until it is release via the buffer release
 * callback.
 *
 * @param[in] handle : Pointer to the audio render instance.
 * @param[in] buf    : Pointer to the audio buffer.
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_render_push(audio_render_ref handle, audio_render_buf_t* buf);

/** Send a command to the audio render.
 *
 * @param[in] handle : Pointer to the audio render instance.
 * @param[in] cmd    : The command to process.
 * @param[in] arg    : Pointer to command specific argument
 *
 * @return    Status of the operation.
 */
wiced_result_t audio_render_command(audio_render_ref handle, AUDIO_RENDER_CMD_T cmd, void* arg);


#ifdef __cplusplus
} /*extern "C" */
#endif
