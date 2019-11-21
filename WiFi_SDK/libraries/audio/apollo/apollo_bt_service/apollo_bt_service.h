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
 * This file provides definitions and function prototypes for Apollo BT service (GATT + A2DP + AVRCP + ...)
 * device
 *
 */
#pragma once

#include "wiced_result.h"
#include "wiced_audio.h"

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

typedef enum
{
    APOLLO_BT_A2DP_EVENT_CONNECTED,
    APOLLO_BT_A2DP_EVENT_DISCONNECTED,
    APOLLO_BT_A2DP_EVENT_STARTED,
    APOLLO_BT_A2DP_EVENT_STOPPED,
    APOLLO_BT_A2DP_EVENT_CODEC_CONFIG
} apollo_bt_a2dp_sink_event_t;

typedef enum
{
    APOLLO_BT_REMOTE_CONTROL_EVENT_CONNECTED,
    APOLLO_BT_REMOTE_CONTROL_EVENT_DISCONNECTED,
    APOLLO_BT_REMOTE_CONTROL_EVENT_ABSOLUTE_VOLUME,
    APOLLO_BT_REMOTE_CONTROL_EVENT_TRACK_PLAYBACK_STATUS,
    APOLLO_BT_REMOTE_CONTROL_EVENT_TRACK_METADATA
} apollo_bt_remote_control_event_t;

typedef enum
{
    APOLLO_BT_REMOTE_CONTROL_CMD_VOLUME_UP   = 0,
    APOLLO_BT_REMOTE_CONTROL_CMD_VOLUME_DOWN,
    APOLLO_BT_REMOTE_CONTROL_CMD_MUTE,
    APOLLO_BT_REMOTE_CONTROL_CMD_PLAY,
    APOLLO_BT_REMOTE_CONTROL_CMD_STOP,
    APOLLO_BT_REMOTE_CONTROL_CMD_PAUSE,
    APOLLO_BT_REMOTE_CONTROL_CMD_REWIND,
    APOLLO_BT_REMOTE_CONTROL_CMD_FAST_FORWARD,
    APOLLO_BT_REMOTE_CONTROL_CMD_NEXT,
    APOLLO_BT_REMOTE_CONTROL_CMD_PREVIOUS,

    APOLLO_BT_REMOTE_CONTROL_CMD_UNSUPPORTED,
    APOLLO_BT_REMOTE_CONTROL_CMD_MAX         = APOLLO_BT_REMOTE_CONTROL_CMD_UNSUPPORTED,
} apollo_bt_remote_control_command_t;

typedef enum
{
    APOLLO_BT_A2DP_OUTPUT_MODE_PCM,                         /*<! invokes data_cbf from @ref apollo_bt_a2dp_sink_data_callback_t */
    APOLLO_BT_A2DP_OUTPUT_MODE_I2S,                         /*<! provide signal directly over I2S lines                         */
} apollo_bt_a2dp_sink_output_mode_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint32_t position;
    uint32_t duration;
} apollo_bt_track_playback_status_t;

typedef struct
{
    int8_t *artist;
    int8_t *title;
} apollo_bt_track_metadata_t;

typedef struct
{
    uint8_t volume_current;
    uint8_t volume_min;
    uint8_t volume_max;
    uint8_t volume_step;
} apollo_bt_audio_volume_t;

typedef union
{
    wiced_audio_config_t              codec_config;
} apollo_bt_a2dp_sink_event_data_t;

typedef union
{
    apollo_bt_audio_volume_t          volume;
    apollo_bt_track_metadata_t        metadata;
    apollo_bt_track_playback_status_t playback_status;
} apollo_bt_remote_control_event_data_t;

typedef void (*apollo_bt_a2dp_sink_event_callback_t)(apollo_bt_a2dp_sink_event_t event, apollo_bt_a2dp_sink_event_data_t *event_data, void *user_context);
typedef void (*apollo_bt_a2dp_sink_data_callback_t)(uint8_t *data, uint32_t data_length, void *user_context);
typedef void (*apollo_bt_remote_control_event_callback_t)(apollo_bt_remote_control_event_t event, apollo_bt_remote_control_event_data_t *event_data, void *user_context);

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint32_t app_dct_offset;                                /*!< Offset at which Bluetooth subsystem will write non-volatile data to flash     */
} apollo_bt_service_init_params_t;

typedef struct
{
    apollo_bt_a2dp_sink_output_mode_t         output_mode;  /*!< Audio output mode @ref apollo_bt_a2dp_sink_output_mode_t                      */
    apollo_bt_a2dp_sink_event_callback_t      event_cbf;    /*!< Bluetooth A2DP sink event callback @ref apollo_bt_a2dp_sink_event_t           */
    apollo_bt_a2dp_sink_data_callback_t       data_cbf;     /*!< Bluetooth A2DP PCM data callback                                              */
    apollo_bt_remote_control_event_callback_t rc_event_cbf; /*!< Bluetooth remote control event callback @ref apollo_bt_remote_control_event_t */
    void                                     *user_context; /*!< User context pointer                                                          */
} apollo_bt_a2dp_sink_init_params_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Perform the main initialization of Bluetooth subsystem.
 * Must be called first in order to access other BT features like A2DP, GATT and AVRCP.
 *
 * @param[in]   params         Bluetooth subsystem initialization parameters @ref apollo_bt_service_init_params_t
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_bt_service_init  ( apollo_bt_service_init_params_t *params );


/**
 * Shutdown and cleanup Bluetooth subsystem.
 * Must be called last; A2DP, GATT, AVRCP must be shutdown first.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_bt_service_deinit( void );


/**
 * Start Bluetooth A2DP audio sink.
 * apollo_bt_service_init() must be called first; otherwise, this call will fail.
 *
 * @param[in]  params          A2DP sink initialization parameters @apollo_bt_a2dp_sink_init_params_t
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_bt_a2dp_sink_init  ( apollo_bt_a2dp_sink_init_params_t *params );


/**
 * Shutdown and cleanup Bluetooth A2DP audio sink.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_bt_a2dp_sink_deinit( void );

/**
 * Select audio output mode:
 * - apollo_bt_a2dp_sink_data_callback_t provide PCM samples
 * - digital audio signal over I2S (if service library supports it)
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_bt_a2dp_sink_set_ouput_mode( apollo_bt_a2dp_sink_output_mode_t output_mode );


/**
 * Send a command to the remote BT peer
 * apollo_bt_a2dp_sink_init() must be called first; otherwise, this call will fail.
 *
 * @param[in]  cmd             Remote control command @apollo_bt_remote_control_command_t
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_bt_remote_control_send_command( apollo_bt_remote_control_command_t cmd );

#ifdef __cplusplus
} /* extern "C" */
#endif
