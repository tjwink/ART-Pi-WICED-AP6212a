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
#pragma once

#include "wiced_audio.h"
#include "audio_client.h"
#include "audio_client_private.h"

#include "callback_loop.h"
#include "dpm.h"
#include "wiced_duktape.h"

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
    LWS_AUDIO_CALLBACK_EVENT_ONERROR,
    LWS_AUDIO_CALLBACK_EVENT_ONSTATECHANGE,
    LWS_AUDIO_CALLBACK_EVENT_PULLING,

} dpm_audio_callback_event_id;

typedef enum
{
    LWS_AUDIO_ERROR_NONE = 0,
    LWS_AUDIO_ERROR_INTERNAL,
    LWS_AUDIO_ERROR_NOMEM,

} dpm_audio_errorno_t;

typedef enum
{
    LWS_AUDIO_PLAYBACK_UNKNOWN = 0,
    LWS_AUDIO_PLAYBACK_STARTING,
    LWS_AUDIO_PLAYBACK_STARTED,
    LWS_AUDIO_PLAYBACK_STOPPING,
    LWS_AUDIO_PLAYBACK_STOPPED,
    LWS_AUDIO_PLAYBACK_PAUSING,
    LWS_AUDIO_PLAYBACK_PAUSED,
    LWS_AUDIO_PLAYBACK_EOS,
} dpm_audio_playback_state_t;

typedef enum
{
    LWS_AUDIO_READYSTATE_NOTHING = 0,
    LWS_AUDIO_READYSTATE_METADATA = 1,
    LWS_AUDIO_READYSTATE_CURRENT_DATA = 2,
    LWS_AUDIO_READYSTATE_FUTURE_DATA = 3,
    LWS_AUDIO_READYSTATE_ENOUGH_DATA = 4
} dpm_audio_ready_state_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
struct audio_time_range {
    double length;
    double start;
    double end;
};

struct audio_property
{
    // properties with getter and setter
    double currentTime;
    double volume;
    double duration;

    // properties with getter
    struct audio_time_range played;
    struct audio_time_range seekable;


    int ended;
    int paused;

    int readyState;

    // attributes
    char *autoplay;
    char *preload;
    char *src;
};

typedef struct
{
  duk_context *ctx;
  void *data;
  audio_client_ref client;
  wiced_timed_event_t timed_event;

  dpm_audio_errorno_t error;
  dpm_audio_playback_state_t state;
  struct audio_property properties;

}dpm_audio_handle_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
duk_ret_t dpm_new_audio(duk_context *ctx);
// method
duk_ret_t dpm_audio_load(duk_context *ctx);
duk_ret_t dpm_audio_play(duk_context *ctx);
duk_ret_t dpm_audio_pause(duk_context *ctx);
// attribute
duk_ret_t dpm_audio_attribute(duk_context *ctx);
// property getter and setter
duk_ret_t dpm_audio_property_getter(duk_context *ctx);
duk_ret_t dpm_audio_property_setter(duk_context *ctx);
// event
duk_ret_t dpm_audio_on_event(duk_context *ctx);
// shutdown
duk_ret_t dpm_audio_shutdown(duk_context *ctx);


void audio_callback_handler(wiced_duktape_callback_queue_element_t *message);

#ifdef __cplusplus
} /* extern "C" */
#endif
