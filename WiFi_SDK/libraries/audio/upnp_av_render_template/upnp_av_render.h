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
 * @file UPnP AV rendering service library
 */

#pragma once

#include "wiced_result.h"
#include "audio_client.h"

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
    UPNPAVRENDER_EVENT_PLAY,
    UPNPAVRENDER_EVENT_PAUSE,
    UPNPAVRENDER_EVENT_RESUME,
    UPNPAVRENDER_EVENT_STOP,
    UPNPAVRENDER_EVENT_SEEK,
    UPNPAVRENDER_EVENT_PLAYBACK_PROGRESS,
    UPNPAVRENDER_EVENT_MUTE,
    UPNPAVRENDER_EVENT_VOLUME
} upnpavrender_event_t;

typedef enum
{
    UPNPAVRENDER_TRANSPORT_STOPPED,
    UPNPAVRENDER_TRANSPORT_PLAYING,
    UPNPAVRENDER_TRANSPORT_TRANSITIONING,
    UPNPAVRENDER_TRANSPORT_PAUSED_PLAYBACK,
    UPNPAVRENDER_TRANSPORT_PAUSED_RECORDING,
    UPNPAVRENDER_TRANSPORT_RECORDING,
    UPNPAVRENDER_TRANSPORT_NO_MEDIA_PRESENT
} upnpavrender_transport_state_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/*
 * UPnP AV rendering service handle
 */

typedef struct upnpavrender_service_s* upnpavrender_service_ref;

/*
 * UPnP AV rendering service playback events
 */

typedef struct
{
    uint32_t position_msecs;                               /*!< Playback position in milliseconds (relative to track duration below)      */
    uint32_t duration_msecs;                               /*!< Track duration in milliseconds                                            */
} upnpavrender_playback_progress_t;

typedef union
{
    wiced_bool_t                     mute_enabled;         /*!, Audio mute state                                                          */
    uint8_t                          volume;               /*!< Audio volume (0 - 100)                                                    */
    upnpavrender_playback_progress_t playback;             /*!< Current playback position and track duration in milliseconds              */
    uint32_t                         seek_position_msecs;  /*!< Seek position in milliseconds                                             */
} upnpavrender_event_data_t;

typedef wiced_result_t (*upnpavrender_event_cb_t)(void* user_context, upnpavrender_event_t event, upnpavrender_event_data_t *event_data);

/******************************************************
 *                    Structures
 ******************************************************/

/*
 * UPnP AV rendering service initialization parameters
 */

typedef struct
{
    wiced_interface_t        interface;           /*!< Network interface that will be used by the UPnP AV rendering service @ref wiced_interface_t */
    uint16_t                 listening_port;      /*!< Listening port number must be no less than 49152                                            */
    char*                    uuid;                /*!< String containing the UUID used to uniquely identify the UPnP AV renderer                   */
    char*                    friendly_name;       /*!< String containing a friendly / human readable name to identify the UPnP AV renderer         */
    audio_client_params_t    audio_client_params; /*!< Audio client initialization structure @ref audio_client_params_t                            */
    upnpavrender_event_cb_t  event_cb;            /*!< UPnP AV rendering event callback function @ref upnpavrender_event_cbf_t                     */
    void*                    user_context;        /*!< User context passed along with the event callback function @ref upnpavrender_event_cbf_t    */
} upnpavrender_service_params_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Start UPnP AV rendering service
 *
 * @param[in]  params  : UPnP AV rendering service initialization parameters @ref upnpavrender_service_params_t
 * @param[out] phandle : Pointer to UPnP AV rendering service handle @ref upnpavrender_service_ref
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_start( upnpavrender_service_params_t* params, upnpavrender_service_ref* phandle );


/**
 * Stop UPnP AV rendering service
 *
 * @param[in]  handle  : UPnP AV rendering service handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_stop( upnpavrender_service_ref handle );


/**
 * Pause current audio playing
 *
 * @param[in]  handle  : UPnP AV rendering service handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_pause( upnpavrender_service_ref handle );


/**
 * Resume current paused audio
 *
 * @param[in]  handle  : UPnP AV rendering service handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_resume( upnpavrender_service_ref handle );


/**
 * stop playing audio
 *
 * @param[in]  handle  : UPnP AV rendering service handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_stop_playing( upnpavrender_service_ref handle );


/**
 * Get state of upnpavrender
 *
 * @param[in]  handle  : UPnP AV rendering service handle
 * @param[out] state   : Pointer to transport state @ref upnpavrender_transport_state_t
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_get_transport_state( upnpavrender_service_ref handle, upnpavrender_transport_state_t *state );


/**
 * Set volume ofupnpavrender service
 *
 * @param[in]  handle  : UPnP AV rendering service handle
 * @param[in]  volume  : volume to be set
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_set_volume( upnpavrender_service_ref handle, int volume );


/**
 * Get volume of upnpavrender service
 *
 * @param[in]  handle  : UPnP AV rendering service handle
 * @param[out] volume  : Pointer to volume to be obtained @ref int
 *
 * @return @ref wiced_result_t
 */
wiced_result_t upnpavrender_service_get_volume( upnpavrender_service_ref handle, int *volume );

/**
 * Return Audio Client Handle
 *
 * @param[in] handle : UPnP AV rendering service handle @ref upnpavrender_service_ref
 *
 * @return @ref audio_client_ref
 */
audio_client_ref upnpavrender_service_get_audio_handle( upnpavrender_service_ref handle );


#ifdef __cplusplus
} /* extern "C" */
#endif
